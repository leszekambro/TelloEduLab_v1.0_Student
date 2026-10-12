import time
import numpy as np
from .backend import PyBulletBackend


class _FrameRead:
    """Small DJITelloPy-compatible frame reader."""

    def __init__(self, tello):
        self._tello = tello
        self.stopped = False

    @property
    def frame(self):
        if self.stopped or not self._tello.is_streaming:
            return None
        return self._tello._sim.get_camera_frame()

    def stop(self):
        self.stopped = True


class Tello:
    """Educational simulator API shaped after ``djitellopy.Tello``.

    Distances are in cm, angles in degrees and RC commands in [-100, 100].
    Simulator-only keyword arguments are optional, so student flight code can
    remain close to code used later with a physical Tello.
    """

    def __init__(self, host="192.168.10.1", retry_count=3, gui=True,
                 simulation_mode="fast", simulation_speed=1.0,
                 follow_camera=True, camera_distance=1.6,
                 camera_yaw=45.0, camera_pitch=-28.0, **kwargs):
        self.host = host
        self.name = kwargs.pop("name", "sim-1")
        self.retry_count = retry_count
        self._sim = PyBulletBackend(
            gui=gui,
            simulation_mode=simulation_mode,
            simulation_speed=simulation_speed,
            follow_camera=follow_camera,
            camera_distance=camera_distance,
            camera_yaw=camera_yaw,
            camera_pitch=camera_pitch,
        )
        self._frame = _FrameRead(self)
        self.is_streaming = False
        self.mission_pad_detection_direction = 0

    def connect(self, wait_for_state=True):
        self._sim.connected = True
        return True

    def end(self):
        self._sim.close()

    def takeoff(self):
        self._sim.airborne = True
        p = self._sim.state()["pos"]
        return self._sim.goto([p[0], p[1], 1.0], speed=0.5)

    def land(self):
        p = self._sim.state()["pos"]
        ok = self._sim.goto([p[0], p[1], 0.10], speed=0.4)
        self._sim.airborne = False
        return ok

    def emergency(self):
        self._sim.airborne = False
        self._sim.target_pos = self._sim.state()["pos"]
        return True

    def _move(self, dx=0, dy=0, dz=0, speed=50):
        p = self._sim.state()["pos"]
        psi = self._sim.state()["rpy"][2]
        body = np.array([dx, dy, dz], dtype=float) / 100.0
        c, s = np.cos(psi), np.sin(psi)
        world = np.array([
            c * body[0] - s * body[1],
            s * body[0] + c * body[1],
            body[2],
        ])
        return self._sim.goto(p + world, speed=max(0.1, speed / 100.0))

    def move_forward(self, x): return self._move(dx=x)
    def move_back(self, x): return self._move(dx=-x)
    def move_left(self, x): return self._move(dy=x)
    def move_right(self, x): return self._move(dy=-x)
    def move_up(self, x): return self._move(dz=x)
    def move_down(self, x): return self._move(dz=-x)
    def move(self, direction, x): return getattr(self, "move_" + direction)(x)

    def rotate_clockwise(self, x):
        self._sim.target_yaw -= np.deg2rad(x)
        self._sim.hold(max(0.5, x / 90.0))
        return True

    def rotate_counter_clockwise(self, x):
        self._sim.target_yaw += np.deg2rad(x)
        self._sim.hold(max(0.5, x / 90.0))
        return True

    def send_rc_control(self, left_right_velocity, forward_backward_velocity,
                        up_down_velocity, yaw_velocity):
        self._sim.velocity_body(
            left_right_velocity, forward_backward_velocity,
            up_down_velocity, yaw_velocity
        )

    def go_xyz_speed(self, x, y, z, speed):
        # Tello command coordinates are relative to the current body frame.
        return self._move(dx=x, dy=-y, dz=z, speed=speed)

    def curve_xyz_speed(self, x1, y1, z1, x2, y2, z2, speed):
        # Educational approximation: two consecutive relative waypoints.
        self.go_xyz_speed(x1, y1, z1, speed)
        return self.go_xyz_speed(x2 - x1, y2 - y1, z2 - z1, speed)

    def get_height(self): return int(round(self._sim.state()["pos"][2] * 100))
    def get_flight_time(self): return int(time.time() - self._sim.start_time)
    def get_battery(self): return int(self._sim.battery)
    def get_temperature(self): return 42
    def get_lowest_temperature(self): return 40
    def get_highest_temperature(self): return 44
    def get_barometer(self): return self._sim.state()["pos"][2] * 100
    def get_distance_tof(self): return int(max(0, self._sim.state()["pos"][2] * 100))
    def get_speed_x(self): return int(self._sim.state()["vel"][0] * 100)
    def get_speed_y(self): return int(self._sim.state()["vel"][1] * 100)
    def get_speed_z(self): return int(self._sim.state()["vel"][2] * 100)
    def get_acceleration_x(self): return float(self.get_imu()["accel"][0])
    def get_acceleration_y(self): return float(self.get_imu()["accel"][1])
    def get_acceleration_z(self): return float(self.get_imu()["accel"][2])
    def get_pitch(self): return int(np.rad2deg(self._sim.state()["rpy"][1]))
    def get_roll(self): return int(np.rad2deg(self._sim.state()["rpy"][0]))
    def get_yaw(self): return int(np.rad2deg(self._sim.state()["rpy"][2]))

    def get_state_field(self, key):
        fields = {
            "h": self.get_height(), "bat": self.get_battery(),
            "pitch": self.get_pitch(), "roll": self.get_roll(),
            "yaw": self.get_yaw(),
        }
        if key not in fields:
            raise KeyError(key)
        return fields[key]

    def get_current_state(self):
        return {
            "h": self.get_height(), "bat": self.get_battery(),
            "pitch": self.get_pitch(), "roll": self.get_roll(),
            "yaw": self.get_yaw(),
        }

    def streamon(self):
        self.is_streaming = True
        self._frame.stopped = False
        return True

    def streamoff(self):
        self.is_streaming = False
        return True

    def get_frame_read(self, with_queue=False, max_queue_len=32):
        return self._frame

    # ------------------------------------------------------------------
    # Educational sensor/control helpers
    # ------------------------------------------------------------------
    def get_imu(self):
        """Return one IMU sample: accel [m/s^2], gyro [rad/s], dt [s]."""
        return self._sim.get_imu()

    def get_position(self):
        """Simulator ground-truth position [m]. For evaluation, not sensing."""
        return self._sim.state()["pos"].copy()

    def get_velocity(self):
        """Simulator ground-truth world velocity [m/s]."""
        return self._sim.state()["vel"].copy()

    def get_front_distance(self, max_distance=10.0): return self._sim.ray_distance((1,0,0), max_distance)
    def get_back_distance(self, max_distance=10.0): return self._sim.ray_distance((-1,0,0), max_distance)
    def get_left_distance(self, max_distance=10.0): return self._sim.ray_distance((0,1,0), max_distance)
    def get_right_distance(self, max_distance=10.0): return self._sim.ray_distance((0,-1,0), max_distance)
    def get_up_distance(self, max_distance=10.0): return self._sim.ray_distance((0,0,1), max_distance)
    def get_down_distance(self, max_distance=10.0): return self._sim.ray_distance((0,0,-1), max_distance)

    def get_state(self):
        p = self.get_position(); v = self.get_velocity()
        return {"name": self.name, "host": self.host, "position": p,
                "velocity_mps": v,
                "rpy_deg": np.array([self.get_roll(), self.get_pitch(), self.get_yaw()], float),
                "height_cm": self.get_height(), "tof_cm": self.get_distance_tof(),
                "battery_pct": self.get_battery()}

    def capabilities(self):
        return {"camera":True,"height":True,"tof":True,"imu":True,"raycast":True,
                "ground_truth":True,"global_position":True,"swarm":True}

    def control_step(self, left_right_velocity=0, forward_backward_velocity=0,
                     up_down_velocity=0, yaw_velocity=0, dt=0.05):
        """Apply an RC command for approximately ``dt`` seconds.

        This helper has the same meaning in simulation and RealTello and is
        useful for portable feedback-control examples.
        """
        steps = max(1, int(round(float(dt) / self._sim.dt)))
        for _ in range(steps):
            self.send_rc_control(left_right_velocity, forward_backward_velocity,
                                 up_down_velocity, yaw_velocity)

    # ------------------------------------------------------------------
    # Simulator-only helpers (not part of physical Tello API)
    # ------------------------------------------------------------------
    def set_simulation_mode(self, mode="fast", speed=None):
        self._sim.set_simulation_mode(mode, speed)

    def set_simulation_speed(self, speed):
        self._sim.set_simulation_speed(speed)

    def set_follow_camera(self, enabled=True, distance=None, yaw=None, pitch=None):
        self._sim.configure_follow_camera(enabled, distance, yaw, pitch)

    def set_camera_distance(self, distance):
        self._sim.configure_follow_camera(distance=distance)

    def sim_wait(self, seconds):
        """Advance simulation while holding position (simulator only)."""
        self._sim.hold(seconds)

    def add_aruco_marker(self, marker_id=23, size=0.30,
                         position=(2.0, 0.0, 1.0), yaw=180.0):
        """Add an instructor/world ArUco target without exposing private state."""
        from .aruco_marker import ArucoMarker
        return ArucoMarker(
            marker_id=marker_id, size=size, position=position, yaw=yaw,
            client_id=self._sim.client_id
        )

    def enable_mission_pads(self): return True
    def disable_mission_pads(self): return True
    def set_mission_pad_detection_direction(self, x): self.mission_pad_detection_direction = x; return True
    def get_mission_pad_id(self): return -1
    def get_mission_pad_distance_x(self): return 0
    def get_mission_pad_distance_y(self): return 0
    def get_mission_pad_distance_z(self): return 0
    def get_mission_pad_angle_x(self): return 0
    def get_mission_pad_angle_y(self): return 0
    def get_mission_pad_angle_z(self): return 0

    # Compatibility/emulation for hardware/network settings.
    def set_speed(self, x): return True
    def query_speed(self): return 50
    def set_wifi_credentials(self, ssid, password): return True
    def connect_to_wifi(self, ssid, password): return True
    def set_network_ports(self, state_packet_port, video_stream_port): return True
    def query_wifi_signal_noise_ratio(self): return 100
    def query_sdk_version(self): return "TelloEduLab"
    def query_serial_number(self): return "SIM00000001"
    def turn_motor_on(self): return True
    def turn_motor_off(self): return True
    def set_video_bitrate(self, bitrate): return True
    def set_video_resolution(self, resolution): return True
    def set_video_fps(self, fps): return True
    def set_video_direction(self, direction): return True
    def send_command_with_return(self, command, timeout=7): return "ok"
    def send_control_command(self, command, timeout=7): return True
    def send_read_command(self, command): return "0"
    def send_command_without_return(self, command): return None
    # ============================================================
    # FLIGHT DATA LOGGING
    # ============================================================

    def get_flight_log(self):
        """
        Return recorded flight data as a dictionary of NumPy arrays.

        Returns
        -------
        dict
            time        [s]
            pos         [m]
            target_pos  [m]
            rpy         [rad]
            target_rpy  [rad]
            vel         [m/s]
            ang_vel     [rad/s]
        """
        import numpy as np

        log = self._sim.log

        return {
            key: np.asarray(value).copy()
            for key, value in log.items()
        }

    def clear_flight_log(self):
        """Clear all recorded flight data."""

        for key in self._sim.log:
            self._sim.log[key].clear()

        self._sim.sim_time = 0.0

    def save_flight_data(self, filename="flight_log.csv"):
        """
        Save recorded flight data to CSV.
        """
        import csv
        import numpy as np

        log = self.get_flight_log()

        if len(log["time"]) == 0:
            print("No flight data available.")
            return

        with open(filename, "w", newline="") as f:

            writer = csv.writer(f)

            writer.writerow([
                "time_s",

                "x_m", "y_m", "z_m",
                "x_ref_m", "y_ref_m", "z_ref_m",

                "roll_deg", "pitch_deg", "yaw_deg",
                "roll_ref_deg", "pitch_ref_deg", "yaw_ref_deg",

                "vx_mps", "vy_mps", "vz_mps",

                "p_degps", "q_degps", "r_degps",
            ])

            rpy_deg = np.rad2deg(log["rpy"])
            rpy_ref_deg = np.rad2deg(log["target_rpy"])
            ang_vel_deg = np.rad2deg(log["ang_vel"])

            for i in range(len(log["time"])):
                writer.writerow([
                    log["time"][i],

                    *log["pos"][i],
                    *log["target_pos"][i],

                    *rpy_deg[i],
                    *rpy_ref_deg[i],

                    *log["vel"][i],

                    *ang_vel_deg[i],
                ])

        print(f"Flight data saved to: {filename}")

    def plot_flight_data(self):
        """
        Plot automatically recorded flight data.
        """

        import numpy as np
        import matplotlib.pyplot as plt

        log = self.get_flight_log()

        if len(log["time"]) == 0:
            print("No flight data available.")
            return

        t = log["time"]

        pos = log["pos"]
        pos_ref = log["target_pos"]

        rpy = np.rad2deg(log["rpy"])
        rpy_ref = np.rad2deg(log["target_rpy"])

        vel = log["vel"]
        ang_vel = np.rad2deg(log["ang_vel"])

        # ========================================================
        # 1. POSITION
        # ========================================================

        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 7))

        labels = ["x", "y", "z"]

        for i in range(3):
            ax[i].plot(
                t,
                pos[:, i],
                label="Measured"
            )

            ax[i].plot(
                t,
                pos_ref[:, i],
                "--",
                label="Reference"
            )

            ax[i].set_ylabel(f"{labels[i]} [m]")
            ax[i].grid(True)
            ax[i].legend()

        ax[-1].set_xlabel("Time [s]")

        fig.suptitle("Position tracking")
        fig.tight_layout()

        # ========================================================
        # 2. ATTITUDE - ROLL, PITCH, YAW
        # ========================================================

        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 7))

        labels = ["Roll", "Pitch", "Yaw"]

        for i in range(3):
            ax[i].plot(
                t,
                rpy[:, i],
                label="Measured"
            )

            ax[i].plot(
                t,
                rpy_ref[:, i],
                "--",
                label="Reference"
            )

            ax[i].set_ylabel(f"{labels[i]} [deg]")
            ax[i].grid(True)
            ax[i].legend()

        ax[-1].set_xlabel("Time [s]")

        fig.suptitle("Attitude")
        fig.tight_layout()

        # ========================================================
        # 3. LINEAR VELOCITY
        # ========================================================

        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 7))

        labels = ["vx", "vy", "vz"]

        for i in range(3):
            ax[i].plot(
                t,
                vel[:, i]
            )

            ax[i].set_ylabel(f"{labels[i]} [m/s]")
            ax[i].grid(True)

        ax[-1].set_xlabel("Time [s]")

        fig.suptitle("Linear velocity")
        fig.tight_layout()

        # ========================================================
        # 4. ANGULAR VELOCITY
        # ========================================================

        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 7))

        labels = ["p", "q", "r"]

        for i in range(3):
            ax[i].plot(
                t,
                ang_vel[:, i]
            )

            ax[i].set_ylabel(f"{labels[i]} [deg/s]")
            ax[i].grid(True)

        ax[-1].set_xlabel("Time [s]")

        fig.suptitle("Angular velocity")
        fig.tight_layout()

        # ========================================================
        # 5. XY TRAJECTORY
        # ========================================================

        plt.figure(figsize=(7, 7))

        plt.plot(
            pos_ref[:, 0],
            pos_ref[:, 1],
            "--",
            label="Reference"
        )

        plt.plot(
            pos[:, 0],
            pos[:, 1],
            label="Measured"
        )

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")

        plt.title("XY trajectory")

        plt.axis("equal")
        plt.grid(True)
        plt.legend()

        plt.tight_layout()

        # ========================================================
        # 6. 3D TRAJECTORY
        # ========================================================

        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection="3d")

        # Reference trajectory
        ax.plot(
            pos_ref[:, 0],
            pos_ref[:, 1],
            pos_ref[:, 2],
            "--",
            linewidth=2,
            label="Reference"
        )

        # Measured trajectory
        ax.plot(
            pos[:, 0],
            pos[:, 1],
            pos[:, 2],
            linewidth=2,
            label="Measured"
        )

        # Start point
        ax.scatter(
            pos[0, 0],
            pos[0, 1],
            pos[0, 2],
            s=60,
            label="Start"
        )

        # End point
        ax.scatter(
            pos[-1, 0],
            pos[-1, 1],
            pos[-1, 2],
            s=60,
            label="End"
        )

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")

        ax.set_title("3D flight trajectory")

        ax.grid(True)
        ax.legend()

        # Equal scaling of X, Y and Z axes
        x_all = np.concatenate((pos[:, 0], pos_ref[:, 0]))
        y_all = np.concatenate((pos[:, 1], pos_ref[:, 1]))
        z_all = np.concatenate((pos[:, 2], pos_ref[:, 2]))

        x_range = np.ptp(x_all)
        y_range = np.ptp(y_all)
        z_range = np.ptp(z_all)

        max_range = max(x_range, y_range, z_range, 0.1)

        x_mid = (np.max(x_all) + np.min(x_all)) / 2
        y_mid = (np.max(y_all) + np.min(y_all)) / 2
        z_mid = (np.max(z_all) + np.min(z_all)) / 2

        ax.set_xlim(
            x_mid - max_range / 2,
            x_mid + max_range / 2
        )

        ax.set_ylim(
            y_mid - max_range / 2,
            y_mid + max_range / 2
        )

        ax.set_zlim(
            max(0, z_mid - max_range / 2),
            z_mid + max_range / 2
        )

        plt.tight_layout()

        plt.show()