import cv2, numpy, pybullet, gymnasium
from telloedu import Tello, RealTello, World, Swarm, PID, create_tello
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
print('TelloEduLab v0.5 imports: OK')
print('OpenCV ArUco:', hasattr(cv2,'aruco'))
