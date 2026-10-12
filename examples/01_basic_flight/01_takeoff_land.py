from telloedu import create_tello


# TelloEduLab -- one SIM/REAL configuration standard
MODE = "sim"              # "sim" or "real"
HOST = "192.168.10.1"     # IP address of the physical Tello/Tello EDU

# observe = synchronized animation; 1.0 real time, 0.5 half speed.
tello = create_tello(MODE, host=HOST, gui=True, simulation_mode="observe", simulation_speed=1.0)


def print_state(drone):
    """Display current drone parameters."""
    state = drone.get_state()
    print("\n========== TELLO STATE ==========")
    for key, value in state.items():
        print(f"{key:20s}: {value}")
    print("=================================\n")

try:
    tello.connect()
    print("Battery:", tello.get_battery(), "%")
    print("TAKEOFF")
    tello.takeoff()
    # Portable hold: advances PyBullet in SIM and waits on the physical drone in REAL.
    tello.control_step(0, 0, 0, 0, dt=2.0)
    print("Height:", tello.get_height(), "cm")
    print_state(tello)
    print("LAND")
    tello.land()
finally:
    tello.end()
