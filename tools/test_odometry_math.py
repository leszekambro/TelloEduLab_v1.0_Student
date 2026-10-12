"""Deterministic unit checks for Lab 05 coordinate transforms."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import importlib
s = importlib.import_module("labs_solutions.05_odometry.solution")
R = s.rotation_body_to_world(0, 0, 0)
assert np.allclose(R, np.eye(3))
assert np.allclose(s.specific_force_to_world([0,0,9.81], (0,0,0), np.zeros(3)), [0,0,0])
assert np.allclose(s.specific_force_to_world([0,0,9.81], (0,0,90), np.zeros(3)), [0,0,0], atol=1e-10)
assert np.allclose(s.rotation_body_to_world(0,0,90) @ [1,0,0], [0,1,0], atol=1e-10)
for roll,pitch,yaw in [(5,8,10),(-15,12,45),(0,0,180)]:
    R = s.rotation_body_to_world(roll,pitch,yaw)
    assert np.allclose(R.T@R,np.eye(3),atol=1e-12)
    f=R.T @ [0,0,9.81]
    assert np.allclose(s.specific_force_to_world(f,(roll,pitch,yaw),np.zeros(3)),[0,0,0],atol=1e-10)
print("PASS: odometry orientation/gravity unit tests")
