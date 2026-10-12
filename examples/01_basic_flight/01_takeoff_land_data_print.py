from telloedu import create_tello
import matplotlib.pyplot as plt
import time


# ============================================================
# CONFIGURATION
# ============================================================

MODE = "sim"               # "sim" or "real"
HOST = "192.168.10.1"

tello = create_tello(
    MODE,
    host=HOST,
    gui=True,
    simulation_mode="observe",
    simulation_speed=1.0
)


# ============================================================
# DATA STORAGE
# ============================================================

t_data = []
height_data = []

roll_data = []
pitch_data = []
yaw_data = []

vx_data = []
vy_data = []
vz_data = []


try:
    # ========================================================
    # CONNECT
    # ========================================================

    tello.connect()

    print("Mode:", MODE)
    print("IP:", HOST)
    print("Battery:", tello.get_battery(), "%")

    # ========================================================
    # TAKEOFF
    # ========================================================

    print("TAKEOFF")
    tello.takeoff()

    # ========================================================
    # TELEMETRY COLLECTION
    # ========================================================

    experiment_time = 5.0
    dt = 0.1

    start_time = time.time()

    while time.time() - start_time < experiment_time:

        # Zero control = hover
        tello.control_step(
            0, 0, 0, 0,
            dt=dt
        )

        # Time
        t = time.time() - start_time

        # Sensors
        height = tello.get_height()

        roll = tello.get_roll()
        pitch = tello.get_pitch()
        yaw = tello.get_yaw()

        velocity = tello.get_velocity()

        vx = velocity[0]
        vy = velocity[1]
        vz = velocity[2]

        # Save
        t_data.append(t)

        height_data.append(height)

        roll_data.append(roll)
        pitch_data.append(pitch)
        yaw_data.append(yaw)

        vx_data.append(vx)
        vy_data.append(vy)
        vz_data.append(vz)

        # Terminal telemetry
        print(
            f"t={t:5.2f} s | "
            f"h={height:4.0f} cm | "
            f"R={roll:4.0f}° "
            f"P={pitch:4.0f}° "
            f"Y={yaw:4.0f}° | "
            f"v=({vx:5.1f}, {vy:5.1f}, {vz:5.1f})"
        )

    # ========================================================
    # LAND
    # ========================================================

    print("LAND")
    tello.land()

finally:

    tello.end()


# ============================================================
# PLOTS
# ============================================================

# HEIGHT
plt.figure()

plt.plot(t_data, height_data)

plt.xlabel("Time [s]")
plt.ylabel("Height [cm]")
plt.title("Tello - altitude")

plt.grid(True)
plt.tight_layout()


# ATTITUDE
plt.figure()

plt.plot(t_data, roll_data, label="Roll")
plt.plot(t_data, pitch_data, label="Pitch")
plt.plot(t_data, yaw_data, label="Yaw")

plt.xlabel("Time [s]")
plt.ylabel("Angle [deg]")
plt.title("Tello - attitude")

plt.legend()
plt.grid(True)
plt.tight_layout()


# VELOCITY
plt.figure()

plt.plot(t_data, vx_data, label="vx")
plt.plot(t_data, vy_data, label="vy")
plt.plot(t_data, vz_data, label="vz")

plt.xlabel("Time [s]")
plt.ylabel("Velocity")
plt.title("Tello - velocity")

plt.legend()
plt.grid(True)
plt.tight_layout()


plt.show()