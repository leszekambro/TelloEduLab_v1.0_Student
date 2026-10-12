"""Explicitly typed obstacle sensing; no fake metric range from RGB."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class DistanceReading:
    distance_m: float
    valid: bool
    source: str
    confidence: float = 0.0

class RaycastObstacleDetector:
    def __init__(self, drone, max_distance_m=5.0):
        if not drone.capabilities().get("raycast", False):
            raise RuntimeError("Raycasting is a SIM-only capability")
        self.drone=drone; self.max_distance_m=float(max_distance_m)
    def measure(self, frame=None):
        d=float(self.drone.get_front_distance(self.max_distance_m))
        return DistanceReading(d, math.isfinite(d), "pybullet_raycast", 1.0)

class VisionObstacleDetector:
    """Metric range ONLY to known-size ArUco marker, not arbitrary obstacles.

    This is a demonstrator for marked obstacles. An unmarked object or a
    missed detection produces valid=False, and the flight controller MUST stop.
    """
    def __init__(self, marker_size_m, marker_id, camera_matrix, dist_coeffs=None, dictionary_id=None):
        import cv2
        if not hasattr(cv2, "aruco"):
            raise ImportError("Install opencv-contrib-python for cv2.aruco")
        self.cv2=cv2; self.marker_size_m=float(marker_size_m); self.marker_id=int(marker_id)
        if self.marker_size_m<=0:raise ValueError("marker_size_m must be positive")
        self.K=np.asarray(camera_matrix,dtype=float).reshape(3,3)
        self.dist=np.zeros(5) if dist_coeffs is None else np.asarray(dist_coeffs,dtype=float)
        if dictionary_id is None:dictionary_id=cv2.aruco.DICT_4X4_50
        self.dictionary=cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.detector=cv2.aruco.ArucoDetector(self.dictionary,cv2.aruco.DetectorParameters())
    def measure(self, frame):
        if frame is None:return DistanceReading(float("nan"),False,"aruco",0.0)
        gray=self.cv2.cvtColor(frame,self.cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame
        corners,ids,_=self.detector.detectMarkers(gray)
        if ids is None:return DistanceReading(float("nan"),False,"aruco",0.0)
        half=self.marker_size_m/2
        obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],dtype=np.float32)
        for c,marker in zip(corners,ids.flatten()):
            if int(marker)!=self.marker_id:continue
            ok,rvec,tvec=self.cv2.solvePnP(obj,c.reshape(4,2),self.K,self.dist,flags=self.cv2.SOLVEPNP_IPPE_SQUARE)
            if ok and float(tvec[2])>0:
                return DistanceReading(float(tvec[2]),True,"aruco_marker",1.0)
        return DistanceReading(float("nan"),False,"aruco",0.0)
