import time
import numpy as np
import cv2
import pybullet as p

try:
    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics
    from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
except ImportError as e:
    raise ImportError(
        "gym-pybullet-drones is not installed correctly. Run install_macos.sh "
        "(macOS) or install_other.sh (Linux) from the project folder."
    ) from e


class PyBulletBackend:
    """Single-drone backend used by :class:`telloedu.Tello`.

    ``simulation_mode='fast'`` runs as fast as possible. ``'observe'``
    synchronizes every physics/control step to wall-clock time. In observe mode
    ``simulation_speed=1.0`` means real time, ``0.5`` means half speed, etc.

    The PyBullet debug camera can follow the drone. This only changes the GUI
    view; it never changes the virtual onboard camera used by computer vision.
    """

    def __init__(self, gui=True, ctrl_freq=48, simulation_mode="fast",
                 simulation_speed=1.0, follow_camera=True,
                 camera_distance=1.6, camera_yaw=45.0, camera_pitch=-28.0):
        self.gui = bool(gui)
        self.ctrl_freq = int(ctrl_freq)
        self.dt = 1.0 / self.ctrl_freq

        self.simulation_mode = str(simulation_mode).lower()
        if self.simulation_mode not in ("fast", "observe"):
            raise ValueError("simulation_mode must be 'fast' or 'observe'")
        self.simulation_speed = float(simulation_speed)
        if self.simulation_speed <= 0:
            raise ValueError("simulation_speed must be > 0")

        self.follow_camera = bool(follow_camera)
        self.camera_distance = float(camera_distance)
        self.camera_yaw = float(camera_yaw)
        self.camera_pitch = float(camera_pitch)

        self.env = CtrlAviary(
            drone_model=DroneModel.CF2X,
            num_drones=1,
            initial_xyzs=np.array([[0.0, 0.0, 0.10]]),
            physics=Physics.PYB,
            pyb_freq=240,
            ctrl_freq=self.ctrl_freq,
            gui=self.gui,
        )
        self.ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
        self.obs, _ = self.env.reset(seed=42)
        self.target_pos = np.array([0.0, 0.0, 0.10], dtype=float)
        self.target_yaw = 0.0
        self.connected = False
        self.airborne = False
        self.start_time = time.time()
        self.battery = 100

        # Deterministic synthetic IMU. The accelerometer returns specific
        # force in the body frame [m/s^2], including gravity at rest.
        self._rng = np.random.default_rng(12345)
        self._prev_vel = self.state()["vel"].copy()
        self._imu_acc_body = np.array([0.0, 0.0, 9.81], dtype=float)
        self._imu_gyro_body = np.zeros(3, dtype=float)
        self.imu_accel_noise_std = 0.035
        self.imu_gyro_noise_std = np.deg2rad(0.20)

        self._update_follow_camera()
        # Flight-data logger
        self.log = {
            "time": [],
            "pos": [],
            "target_pos": [],
            "rpy": [],
            "target_rpy": [],
            "vel": [],
            "ang_vel": [],
        }
        self.sim_time = 0.0
        self.world = None

    @property
    def client_id(self):
        return self.env.CLIENT

    def set_simulation_mode(self, mode="fast", speed=None):
        mode = str(mode).lower()
        if mode not in ("fast", "observe"):
            raise ValueError("mode must be 'fast' or 'observe'")
        self.simulation_mode = mode
        if speed is not None:
            self.set_simulation_speed(speed)

    def set_simulation_speed(self, speed):
        speed = float(speed)
        if speed <= 0:
            raise ValueError("speed must be > 0")
        self.simulation_speed = speed

    def configure_follow_camera(self, enabled=None, distance=None, yaw=None, pitch=None):
        if enabled is not None:
            self.follow_camera = bool(enabled)
        if distance is not None:
            distance = float(distance)
            if distance <= 0:
                raise ValueError("camera distance must be > 0")
            self.camera_distance = distance
        if yaw is not None:
            self.camera_yaw = float(yaw)
        if pitch is not None:
            self.camera_pitch = float(pitch)
        self._update_follow_camera()

    def state(self):
        s = np.asarray(self.obs[0], dtype=float)
        return {
            "pos": s[0:3].copy(),
            "quat": s[3:7].copy(),
            "rpy": s[7:10].copy(),
            "vel": s[10:13].copy(),
            "ang_vel": s[13:16].copy(),
            "rpm": s[16:20].copy(),
        }

    def _pace(self, step_started):
        if self.simulation_mode != "observe":
            return
        target_wall_dt = self.dt / self.simulation_speed
        elapsed = time.perf_counter() - step_started
        if target_wall_dt > elapsed:
            time.sleep(target_wall_dt - elapsed)

    def _update_follow_camera(self):
        if not self.gui or not self.follow_camera:
            return
        try:
            drone_id = self.env.DRONE_IDS[0]
            pos, _ = p.getBasePositionAndOrientation(
                drone_id, physicsClientId=self.client_id
            )
            p.resetDebugVisualizerCamera(
                cameraDistance=self.camera_distance,
                cameraYaw=self.camera_yaw,
                cameraPitch=self.camera_pitch,
                cameraTargetPosition=pos,
                physicsClientId=self.client_id,
            )
        except Exception:
            # The debug camera is a convenience feature only. It must never
            # break flight simulation (e.g. during GUI shutdown).
            pass

    def _step_to(self, target_pos=None, target_rpy=None, target_vel=None):
        step_started = time.perf_counter()
        tp = self.target_pos if target_pos is None else np.asarray(target_pos, float)
        tr = (np.array([0.0, 0.0, self.target_yaw])
              if target_rpy is None else np.asarray(target_rpy, float))
        tv = np.zeros(3) if target_vel is None else np.asarray(target_vel, float)

        rpm, _, _ = self.ctrl.computeControlFromState(
            control_timestep=self.dt,
            state=np.asarray(self.obs[0]),
            target_pos=tp,
            target_rpy=tr,
            target_vel=tv,
        )
        prev_vel = self.state()["vel"].copy()
        self.obs, _, _, _, _ = self.env.step(np.array([rpm]))

        # Synthetic IMU from rigid-body state. Specific force is transformed
        # into the body frame and includes gravity, as on a real accelerometer.
        st_new = self.state()
        acc_world = (st_new["vel"] - prev_vel) / self.dt
        quat = st_new["quat"]
        rot = np.asarray(p.getMatrixFromQuaternion(quat), dtype=float).reshape(3, 3)
        specific_force_body = rot.T @ (acc_world + np.array([0.0, 0.0, 9.81]))
        self._imu_acc_body = specific_force_body + self._rng.normal(
            0.0, self.imu_accel_noise_std, 3
        )
        self._imu_gyro_body = rot.T @ st_new["ang_vel"] + self._rng.normal(
            0.0, self.imu_gyro_noise_std, 3
        )
        self._prev_vel = st_new["vel"].copy()

        self.sim_time += self.dt
        if self.world is not None:
            self.world.update(self.sim_time)
        self._log_state()

        self._update_follow_camera()
        self._pace(step_started)

    def hold(self, seconds):
        steps = max(1, int(round(float(seconds) * self.ctrl_freq)))
        for _ in range(steps):
            self._step_to()

    def goto(self, xyz, speed=0.5, timeout=20.0):
        """Fly to a world-frame point.

        ``speed`` is used to move an intermediate reference toward the final
        point. This makes the speed argument meaningful and avoids an abrupt
        full-distance position step.
        """
        goal = np.asarray(xyz, dtype=float)
        speed = max(0.05, float(speed))
        max_steps = max(1, int(float(timeout) * self.ctrl_freq))

        ref = self.state()["pos"].copy()
        for _ in range(max_steps):
            pos = self.state()["pos"]
            remaining = goal - ref
            remaining_dist = float(np.linalg.norm(remaining))

            # Reference advances at approximately requested speed [m/s],
            # independently of the current tracking error.
            max_ref_step = speed * self.dt
            if remaining_dist > max_ref_step:
                ref = ref + remaining / remaining_dist * max_ref_step
            else:
                ref = goal.copy()

            self.target_pos = ref.copy()
            self._step_to(target_pos=ref)

            if np.linalg.norm(pos - goal) < 0.06 and remaining_dist <= max_ref_step:
                self.target_pos = goal.copy()
                return True

        self.target_pos = goal.copy()
        return False

    def velocity_body(self, lr, fb, ud, yaw):
        # DJITelloPy RC values [-100, 100]. Educational mapping: 100 ~= 1 m/s.
        st = self.state()
        psi = st["rpy"][2]
        v_body = np.clip(np.array([fb, -lr, ud], dtype=float), -100, 100) / 100.0
        c, s = np.cos(psi), np.sin(psi)
        v_world = np.array([
            c * v_body[0] - s * v_body[1],
            s * v_body[0] + c * v_body[1],
            v_body[2],
        ])
        self.target_pos = st["pos"] + v_world * 0.35
        yaw = float(np.clip(yaw, -100, 100))
        self.target_yaw += np.deg2rad(yaw * 0.35) * self.dt
        self._step_to(target_vel=v_world)

    def get_camera_frame(self, width=640, height=480, fov=60.0,
                         near=0.05, far=20.0):
        """Return forward-looking onboard-camera frame in OpenCV BGR format."""
        drone_id = self.env.DRONE_IDS[0]
        position, quaternion = p.getBasePositionAndOrientation(
            drone_id, physicsClientId=self.client_id
        )
        position = np.asarray(position, dtype=float)
        rotation = np.asarray(
            p.getMatrixFromQuaternion(quaternion), dtype=float
        ).reshape(3, 3)

        forward = rotation[:, 0]  # body +X
        up = rotation[:, 2]       # body +Z
        camera_position = position + 0.04 * forward
        camera_target = camera_position + 5.0 * forward

        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_position.tolist(),
            cameraTargetPosition=camera_target.tolist(),
            cameraUpVector=up.tolist(),
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=float(fov),
            aspect=float(width) / float(height),
            nearVal=float(near),
            farVal=float(far),
        )
        _, _, rgba, _, _ = p.getCameraImage(
            width=int(width),
            height=int(height),
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            physicsClientId=self.client_id,
        )
        rgba = np.asarray(rgba, dtype=np.uint8).reshape(int(height), int(width), 4)
        return cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR)

    def get_imu(self):
        """Return synthetic IMU sample in SI units.

        accel: body-frame specific force [m/s^2]
        gyro:  body angular velocity [rad/s]
        """
        return {
            "accel": self._imu_acc_body.copy(),
            "gyro": self._imu_gyro_body.copy(),
            "dt": float(self.dt),
            "time": float(self.sim_time),
        }

    def ray_distance(self, direction_body=(1,0,0), max_distance=10.0):
        """Distance [m] from drone to first obstacle along a body-frame ray."""
        st=self.state(); pos=st["pos"]; rot=np.asarray(p.getMatrixFromQuaternion(st["quat"]),float).reshape(3,3)
        d=rot @ np.asarray(direction_body,float); d=d/(np.linalg.norm(d)+1e-12)
        end=pos+d*float(max_distance)
        hit=p.rayTest(pos.tolist(),end.tolist(),physicsClientId=self.client_id)[0]
        return float(max_distance) if hit[0] < 0 else float(hit[2])*float(max_distance)

    def ray_distance_horizontal(self, angle_world, max_distance=4.0):
        """SIM-only horizontal world-frame scan at drone altitude.

        Starts outside the drone collision body to avoid self-hits. Unlike
        ray_distance(), this ray is not tilted by the drone roll/pitch.
        """
        st = self.state()
        origin = np.asarray(st["pos"], dtype=float)
        direction = np.array([np.cos(angle_world), np.sin(angle_world), 0.0])
        offset = 0.16
        start = origin + offset * direction
        end = origin + float(max_distance) * direction
        result = p.rayTest(start.tolist(), end.tolist(), physicsClientId=self.client_id)[0]
        if result[0] < 0 or result[0] == self.env.DRONE_IDS[0]:
            return float(max_distance)
        return float(np.clip(offset + float(result[2]) * (float(max_distance) - offset), 0, max_distance))

    def close(self):
        self.env.close()
    def _log_state(self):
        st = self.state()

        self.log["time"].append(self.sim_time)
        self.log["pos"].append(st["pos"].copy())
        self.log["target_pos"].append(self.target_pos.copy())

        self.log["rpy"].append(st["rpy"].copy())
        self.log["target_rpy"].append(
            np.array([0.0, 0.0, self.target_yaw])
        )

        self.log["vel"].append(st["vel"].copy())
        self.log["ang_vel"].append(st["ang_vel"].copy())