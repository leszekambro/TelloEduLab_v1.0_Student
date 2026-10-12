from telloedu import create_tello



MODE = "sim"              # "sim" or "real"
HOST = "192.168.10.1"     # IP address of the physical Tello/Tello EDU

def student_solution(tello):
    # TODO: use only public Tello-compatible commands.
    pass


if __name__ == "__main__":
    t = create_tello(MODE, host=HOST, gui=True, simulation_mode="observe", simulation_speed=1.0)
    try:
        t.connect(); t.takeoff(); student_solution(t); t.land()
    finally:
        t.end()
