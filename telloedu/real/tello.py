"""Physical DJI Tello/Tello EDU adapter with backend-neutral telemetry logging."""
import time
import numpy as np
from djitellopy import Tello as _DJITello
from ..common.flightlog import FlightLogger

class RealTello:
    def __init__(self, host="192.168.10.1", name=None, retry_count=3, **kwargs):
        for key in ("gui","simulation_mode","simulation_speed","follow_camera","camera_distance","camera_yaw","camera_pitch","ctrl_freq"):
            kwargs.pop(key, None)
        self.host=str(host); self.name=name or self.host
        # DJITelloPy uses host for station-mode multi-drone operation.
        self._tello=_DJITello(host=self.host, retry_count=retry_count, **kwargs)
        self._last_att=None; self._last_imu_time=None
        self._logger=FlightLogger(self.name); self._last_cmd=(0,0,0,0)
    def __getattr__(self,name): return getattr(self._tello,name)
    def connect(self, wait_for_state=True):
        result=self._tello.connect(wait_for_state=wait_for_state); self.log_sample(); return result
    def end(self):
        try: self._tello.send_rc_control(0,0,0,0)
        except Exception: pass
        return self._tello.end()
    def takeoff(self):
        out=self._tello.takeoff(); self.log_sample(); return out
    def land(self):
        out=self._tello.land(); self.log_sample(); return out
    def send_rc_control(self, lr, fb, ud, yaw):
        self._last_cmd=(int(lr),int(fb),int(ud),int(yaw)); out=self._tello.send_rc_control(*self._last_cmd); self.log_sample(); return out
    def control_step(self,left_right_velocity=0,forward_backward_velocity=0,up_down_velocity=0,yaw_velocity=0,dt=0.05):
        self.send_rc_control(left_right_velocity,forward_backward_velocity,up_down_velocity,yaw_velocity)
        time.sleep(max(0.02,float(dt))); self.log_sample()
    def get_imu(self):
        now=time.perf_counter(); att=np.deg2rad(np.array([self._tello.get_roll(),self._tello.get_pitch(),self._tello.get_yaw()],float))
        if self._last_att is None: gyro=np.zeros(3); dt=.05
        else:
            dt=max(1e-3,now-self._last_imu_time); da=(att-self._last_att+np.pi)%(2*np.pi)-np.pi; gyro=da/dt
        self._last_att=att; self._last_imu_time=now
        acc=np.array([self._tello.get_acceleration_x(),self._tello.get_acceleration_y(),self._tello.get_acceleration_z()],float)
        if np.linalg.norm(acc)>30: acc/=100.0
        return {"accel":acc,"gyro":gyro,"dt":dt,"time":now,
                "gyro_source":"euler_angle_finite_difference_not_raw_gyro",
                "accel_source":"tello_sdk_acceleration_heuristic_scale"}
    def get_velocity(self): return np.array([self._tello.get_speed_x(),self._tello.get_speed_y(),self._tello.get_speed_z()],float)/100.0
    def get_position(self): raise NotImplementedError("Stock Tello has no global x-y position. Use an estimator, mission pads, vision/SLAM or external tracking.")
    def get_state(self):
        v=self.get_velocity()
        return {"name":self.name,"host":self.host,"position":None,"velocity_mps":v,
                "rpy_deg":np.array([self.get_roll(),self.get_pitch(),self.get_yaw()],float),
                "height_cm":self.get_height(),"tof_cm":self.get_distance_tof(),"battery_pct":self.get_battery()}
    def log_sample(self):
        try:
            s=self.get_state(); v=s["velocity_mps"]; r=s["rpy_deg"]
            self._logger.append(roll_deg=r[0],pitch_deg=r[1],yaw_deg=r[2],vx_mps=v[0],vy_mps=v[1],vz_mps=v[2],height_cm=s["height_cm"],tof_cm=s["tof_cm"],battery_pct=s["battery_pct"],cmd_lr=self._last_cmd[0],cmd_fb=self._last_cmd[1],cmd_ud=self._last_cmd[2],cmd_yaw=self._last_cmd[3])
        except Exception: pass
    def get_flight_log(self): return self._logger.arrays()
    def clear_flight_log(self): self._logger.clear()
    def save_flight_data(self,filename="flight_log.csv"): return self._logger.save_csv(filename)
    def plot_flight_data(self,show=True): return self._logger.plot(show=show)
    def capabilities(self): return {"camera":True,"height":True,"tof":True,"imu":True,"raycast":False,"ground_truth":False,"global_position":False,"swarm":True}
    def set_follow_camera(self,*a,**k): return None
    def set_camera_distance(self,*a,**k): return None
    def set_simulation_mode(self,*a,**k): return None
    def set_simulation_speed(self,*a,**k): return None
