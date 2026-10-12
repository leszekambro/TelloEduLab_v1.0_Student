TelloEduLab v1.0

### From Simulation to Real Flight — A Python Framework for Autonomous Drone Education and Research

**TelloEduLab** is an open-source-oriented Python framework designed for education, experimentation, and research in autonomous aerial robotics using the **DJI Tello and Tello EDU** platforms.
The framework bridges the gap between simulation and real-world experimentation by providing a unified programming interface for virtual and physical drones.
Students and researchers can develop, test, visualize, and evaluate flight-control and navigation algorithms in a local **PyBullet-based 3D simulator**, then transfer their implementations to a physical drone with minimal modifications.

---

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

## Third-party software

TelloEduLab uses `gym-pybullet-drones` and PyBullet for simulation and DJITelloPy for physical Tello communication. Check the respective upstream licenses when redistributing the project. `gym-pybullet-drones` is used as an external dependency rather than copied into this repository.

Recommended citation for gym-pybullet-drones:
J. Panerati, H. Zheng, S. Zhou, J. Xu, A. Prorok, A. P. Schoellig, “Learning to Fly—A Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control,” IROS, 2021.

