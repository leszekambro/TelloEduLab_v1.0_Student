from telloedu import create_tello


# TelloEduLab v0.8.1 -- one SIM/REAL configuration standard
MODE = "sim"              # "sim" or "real"
HOST = "192.168.10.1"     # IP address of the physical Tello/Tello EDU

SIMULATION_MODE = "observe"
SPEED = 0.5       # 0.5 = half real-time speed

# Follow-camera keeps the drone centered in the PyBullet GUI.
tello = create_tello(
    MODE,
    host=HOST,
    gui=True,
    simulation_mode=SIMULATION_MODE,
    simulation_speed=SPEED,
    follow_camera=True,
    camera_distance=1.6,
)

try:
    tello.connect()
    tello.takeoff()
    for _ in range(4):
        tello.move_forward(100)              # cm
        tello.rotate_counter_clockwise(90)   # deg
    tello.land()
finally:
    tello.end()
