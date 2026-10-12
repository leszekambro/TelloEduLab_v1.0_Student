"""STUDENT WORKSPACE - use public Tello methods only."""
from telloedu import create_tello

MODE = "sim"              # "sim" or "real"
HOST = "192.168.10.1"     # IP address of the physical Tello/Tello EDU

def student_solution(tello):
    # TODO: write your algorithm here.
    pass


def main():
    tello = create_tello(MODE, host=HOST, gui=True, simulation_mode="observe", simulation_speed=1.0)
    try:
        tello.connect(); tello.takeoff(); student_solution(tello); tello.land()
    finally:
        tello.end()


if __name__ == "__main__":
    main()
