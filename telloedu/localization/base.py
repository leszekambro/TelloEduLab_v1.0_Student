from __future__ import annotations
from dataclasses import dataclass
import time
import numpy as np

@dataclass
class PoseEstimate:
    t: float
    position: np.ndarray
    rpy_deg: np.ndarray
    velocity: np.ndarray
    source: str
    valid: bool = True

class LocalizationProvider:
    name = "localization"
    def reset(self): pass
    def update(self, dt=None) -> PoseEstimate: raise NotImplementedError

class GroundTruthLocalization(LocalizationProvider):
    name = "ground_truth"
    def __init__(self, drone): self.drone=drone
    def update(self, dt=None):
        s=self.drone.get_state()
        return PoseEstimate(time.perf_counter(), np.asarray(s.get('position',[np.nan]*3),float),
                            np.asarray(s.get('rpy_deg',[np.nan]*3),float),
                            np.asarray(s.get('velocity',[np.nan]*3),float), self.name)

class DeadReckoningLocalization(LocalizationProvider):
    """Simple teaching estimator: integrates SDK/body velocity after yaw rotation."""
    name = "dead_reckoning"
    def __init__(self, drone, initial_position=(0,0,0)):
        self.drone=drone; self.initial=np.asarray(initial_position,float); self.reset()
    def reset(self): self.p=self.initial.copy(); self.last=time.perf_counter()
    def update(self, dt=None):
        now=time.perf_counter(); dt=(now-self.last) if dt is None else float(dt); self.last=now
        v=np.asarray(self.drone.get_velocity(),float)
        yaw=np.deg2rad(float(self.drone.get_yaw()))
        c,s=np.cos(yaw),np.sin(yaw); R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        vw=R@v; self.p += vw*max(dt,0.0)
        return PoseEstimate(now,self.p.copy(),np.array([self.drone.get_roll(),self.drone.get_pitch(),self.drone.get_yaw()],float),vw,self.name)

class ArucoLocalization(LocalizationProvider):
    """Adapter for a user detector returning (position_xyz_m, rpy_deg) or None."""
    name="aruco"
    def __init__(self, detector): self.detector=detector
    def update(self, dt=None):
        now=time.perf_counter(); result=self.detector()
        if result is None: return PoseEstimate(now,np.full(3,np.nan),np.full(3,np.nan),np.full(3,np.nan),self.name,False)
        p,rpy=result
        return PoseEstimate(now,np.asarray(p,float),np.asarray(rpy,float),np.full(3,np.nan),self.name,True)

class MissionPadLocalization(LocalizationProvider):
    name="mission_pad"
    def __init__(self, drone): self.drone=drone
    def update(self, dt=None):
        now=time.perf_counter(); mid=self.drone.get_mission_pad_id()
        if mid < 0: return PoseEstimate(now,np.full(3,np.nan),np.full(3,np.nan),np.full(3,np.nan),self.name,False)
        p=np.array([self.drone.get_mission_pad_distance_x(),self.drone.get_mission_pad_distance_y(),self.drone.get_mission_pad_distance_z()],float)/100.0
        r=np.array([self.drone.get_mission_pad_angle_x(),self.drone.get_mission_pad_angle_y(),self.drone.get_mission_pad_angle_z()],float)
        return PoseEstimate(now,p,r,np.full(3,np.nan),self.name,True)

class ExternalLocalization(LocalizationProvider):
    """Hook for SLAM/mocap/VIO. callback must return PoseEstimate or None."""
    def __init__(self, callback, name="external"): self.callback=callback; self.name=name
    def update(self, dt=None):
        out=self.callback()
        if out is not None: return out
        return PoseEstimate(time.perf_counter(),np.full(3,np.nan),np.full(3,np.nan),np.full(3,np.nan),self.name,False)
