# TelloEduLab v1.0

## SIM -> REAL standard

Core examples now use one backend-neutral pattern:

```python
from telloedu import create_tello

MODE = "sim"              # change only to "real" for hardware
HOST = "192.168.10.1"     # physical Tello/Tello EDU IP

drone = create_tello(
    MODE,
    host=HOST,
    gui=True,
    simulation_mode="observe",
    simulation_speed=1.0,
)
```

For a physical flight set `MODE = "real"`, connect the computer to the correct Tello network (or configure EDU station mode), verify `HOST`, remove propellers for the first connectivity test, and only then perform a flight test in a safe area. Simulator-only examples that depend on `World`, PyBullet ray casting, dynamic simulated obstacles, or simulator ground truth remain explicitly simulator-oriented.

## Architecture

```text
telloedu/
  sim/       Tello simulator, PyBullet backend, World, ArUco, Swarm
  real/      physical DJI Tello EDU adapter (DJITelloPy)
  common/    reusable algorithms such as PID
```

Compatibility imports from v0.4 remain available:

```python
from telloedu import Tello, RealTello, create_tello
```

## Installation

Recommended: Python 3.11 or 3.12 in a clean virtual/Conda environment.

### macOS

```bash
chmod +x install_macos.sh
./install_macos.sh
```

The script installs PyBullet first and includes the Apple-Silicon fallback used during development.

### Windows

```bat
install_windows.bat
```

### Linux

```bash
chmod +x install_linux.sh
./install_linux.sh
```

### Optional deep learning

```bash
python -m pip install -r requirements_deep_learning.txt
```

For physical-Tello-only computers:

```bash
python -m pip install -r requirements_minimal_real.txt
```

## 1. Simulation and real drone

Simulation:

```python
from telloedu import create_tello

tello = create_tello("sim", gui=True, simulation_mode="observe")
```

Physical Tello EDU:

```python
tello = create_tello("real")
```

The standard calls (`connect`, `takeoff`, `land`, `send_rc_control`, camera streaming, telemetry) are intentionally similar. Simulator-only ground truth is **not** emulated on hardware.

Check portability explicitly:

```python
print(tello.capabilities())
```

`global_position=True` in simulation but `False` for a stock real Tello. Algorithms intended for SIM->REAL should therefore use camera/ToF/telemetry/estimators rather than simulator ground truth.

## 2. Worlds

```python
from telloedu import Tello, World

tello = Tello(gui=True)
tello.connect()
world = World(tello)
world.load("obstacle_course")
```

Ready worlds in `worlds/`:

- `empty`
- `obstacle_course`
- `corridor`
- `aruco_room`
- `aruco_landing`
- `vision_lab`
- `dynamic_obstacles`
- `swarm_arena`

Build scenes from Python:

```python
world.add_box([2, 0, .5], size=[.4, .8, 1.0])
world.add_wall([3, 1, 1], size=[2, .1, 2])
world.add_cylinder([3, -1, .5], radius=.2, height=1)
world.add_gate([4, 0, 1])
world.add_aruco(marker_id=23, position=[2.5, 0, 1])
world.add_moving_box([2,-1,.7], [2,1,.7], speed=.5)
```

World objects are instructor/environment state. Student navigation code should use sensors rather than object coordinates.

## 3. Raycast distance sensors

```python
d_front = tello.get_front_distance()
d_left  = tello.get_left_distance()
d_right = tello.get_right_distance()
```

Also available: back/up/down. They are simulator teaching sensors, not stock Tello EDU SDK sensors.


## Safety for real Tello EDU

Use a clear indoor test area, propeller guards, low initial velocities and an operator ready to issue `land()`/`emergency()`. Validate algorithms in simulation first. Simulator timing, camera rendering, sensing and Crazyflie dynamics used by gym-pybullet-drones are not an exact model of DJI Tello dynamics.

## Third-party software

TelloEduLab uses `gym-pybullet-drones` and PyBullet for simulation and DJITelloPy for physical Tello communication. Check the respective upstream licenses when redistributing the project. `gym-pybullet-drones` is used as an external dependency rather than copied into this repository.

Recommended citation for gym-pybullet-drones:
J. Panerati, H. Zheng, S. Zhou, J. Xu, A. Prorok, A. P. Schoellig, “Learning to Fly—A Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control,” IROS, 2021.

