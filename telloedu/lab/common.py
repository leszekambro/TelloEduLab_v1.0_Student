"""Shared data logging and plotting for instructor laboratory solutions.
Units: position/velocity SI; DJI get_height is centimetres, attitude degrees.
Never substitutes simulated ground truth for a hardware measurement.
"""
from pathlib import Path
from datetime import datetime
import csv, json, math, time
import numpy as np

class LabRun:
    """Record measured/estimated/reference/GT data with separate simulation and wall time."""
    def __init__(self, name, mode='offline', root='runs', config=None):
        self.mode = mode
        self.rows = []
        self.start = time.monotonic()
        self.dir = Path(root) / (datetime.now().strftime('%Y%m%d_%H%M%S_') + name + '_' + mode)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.config = config or {}
        (self.dir / 'configuration.json').write_text(
            json.dumps({'lab': name, 'mode': mode, **self.config}, indent=2), encoding='utf-8')

    def add(self, **fields):
        if self.mode == 'real' and any(k.endswith('_gt') or k.startswith('gt_') or '_gt_' in k for k in fields):
            raise ValueError('Ground truth cannot be logged in REAL mode')
        # t_s is the historical laboratory experiment clock; retain it for compatibility.
        # Wall time is measured independently, and may differ for accelerated simulation.
        row = {'t_wall_s': round(time.monotonic() - self.start, 6), **fields}
        if 't_s' in row:
            row['t_sim_s' if self.mode == 'sim' else 't_experiment_s'] = row['t_s']
        self.rows.append(row)

    def save(self):
        keys = list(dict.fromkeys(k for row in self.rows for k in row))
        if self.rows:
            with (self.dir / 'data.csv').open('w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(self.rows)
        plot_files = []
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            xkey = next((k for k in ('t_sim_s','t_experiment_s','t_s','t_wall_s') if k in keys), None)
            if xkey:
                x = np.asarray([float(row.get(xkey, np.nan)) for row in self.rows])
                groups = [
                    ('commands', ['lr_cmd','fb_cmd','ud_cmd','yaw_cmd','cmd_lr','cmd_fb','cmd_ud','cmd_yaw','u_ud','u_fb','u_lr','u_yaw'], 'RC command [-100, 100]'),
                    ('altitude', ['z_ref','z_meas','z_raw','height_m','z_est','z_gt','z_moving_avg','z_filtered'], 'Altitude [m]'),
                    ('position_x', ['x_ref','x_est','x_gt','x_gt_rel'], 'X position [m]'),
                    ('position_y', ['y_ref','y_est','y_gt','y_gt_rel'], 'Y position [m]'),
                    ('velocity', ['vx_mps','vy_mps','vz_mps','vx_est','vy_est','vz_est','vz_meas'], 'Velocity [m/s]'),
                    ('position_telemetry', ['x_telemetry_integrated','y_telemetry_integrated'], 'Velocity-integrated position [m]'),
                    ('attitude', ['roll_deg','pitch_deg','yaw_deg','roll_est_deg','pitch_est_deg','yaw_est_deg'], 'Attitude [deg]'),
                    ('gyro', ['gx_radps','gy_radps','gz_radps'], 'Angular rate [rad/s]'),
                    ('acceleration', ['ax_mps2','ay_mps2','az_mps2'], 'Acceleration [m/s²]'),
                    ('battery', ['battery_pct'], 'Battery [%]'),
                    ('tracking_error', ['error_z','error_x','error_y'], 'Tracking error [m]'),
                ]
                used = set()
                for name, candidates, ylabel in groups:
                    signals = [k for k in candidates if k in keys and k != xkey]
                    if not signals:
                        continue
                    fig, ax = plt.subplots(figsize=(10, 4.5))
                    plotted = False
                    for k in signals:
                        try:
                            y = np.asarray([float(r.get(k, np.nan)) if r.get(k) not in (None,'') else np.nan for r in self.rows])
                        except (ValueError, TypeError):
                            continue
                        if not np.isfinite(y).any():
                            continue
                        ax.plot(x, y, label=k, linewidth=1.5)
                        used.add(k)
                        plotted = True
                    if plotted:
                        ax.set(xlabel=f'{xkey} [s]', ylabel=ylabel, title=name.replace('_',' ').title())
                        ax.grid(alpha=.3)
                        ax.legend(fontsize=8)
                        fig.tight_layout()
                        filename = f'{name}.png'
                        fig.savefig(self.dir / filename, dpi=160)
                        plot_files.append(filename)
                    plt.close(fig)
                # Unclassified numeric signals are kept, but never mixed across units.
                for k in keys:
                    if k in used or k in ('t_s','t_sim_s','t_experiment_s','t_wall_s'):
                        continue
                    try:
                        y = np.asarray([float(r.get(k, np.nan)) if r.get(k) not in (None,'') else np.nan for r in self.rows])
                    except (ValueError, TypeError):
                        continue
                    if not np.isfinite(y).any():
                        continue
                    fig, ax = plt.subplots(figsize=(10, 4))
                    ax.plot(x, y, label=k)
                    ax.set(xlabel=f'{xkey} [s]', ylabel=k, title=k)
                    ax.grid(alpha=.3)
                    fig.tight_layout()
                    filename = 'signal_' + k + '.png'
                    fig.savefig(self.dir / filename, dpi=140)
                    plot_files.append(filename)
                    plt.close(fig)
                if any({xx, yy} <= set(keys) for xx, yy in [('x_gt','y_gt'),('x_est','y_est'),('x_telemetry_integrated','y_telemetry_integrated')]):
                    fig, ax = plt.subplots(figsize=(6,6))
                    for xx,yy,label in [('x_gt','y_gt','SIM ground truth'),('x_est','y_est','IMU estimate'),('x_telemetry_integrated','y_telemetry_integrated','velocity integration (SIM: GT-derived)')]:
                        if xx in keys and yy in keys:
                            ax.plot([r.get(xx,np.nan) for r in self.rows],
                                    [r.get(yy,np.nan) for r in self.rows], label=label)
                    ax.set(xlabel='x [m]',ylabel='y [m]',title='XY trajectory')
                    ax.axis('equal')
                    ax.grid(alpha=.3)
                    ax.legend()
                    fig.tight_layout()
                    fig.savefig(self.dir/'trajectory_xy.png',dpi=160)
                    plot_files.append('trajectory_xy.png')
                    plt.close(fig)
        except ImportError:
            (self.dir / 'PLOTTING_SKIPPED.txt').write_text('Install matplotlib to generate PNG plots.')
        summary = {'samples':len(self.rows), 'columns':keys, 'plots':plot_files,
                   'wall_duration_s':round(time.monotonic()-self.start,4)}
        if self.rows and 't_sim_s' in keys:
            summary['logged_sim_duration_s'] = float(self.rows[-1]['t_sim_s'])
        (self.dir/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        print('Saved:', self.dir)
        return self.dir

def height_m(drone):return float(drone.get_height())/100.0

def imu_read(drone):
    data=drone.get_imu()
    return np.asarray(data['accel'],float),np.asarray(data['gyro'],float)

def position_sim(drone):return np.asarray(drone.get_position(),float)

def run_flight(name, mode, procedure, *, duration=6., dt=.1, host='192.168.10.1', gui=None, **kwargs):
    """Takeoff is opt-in through procedure. Guarantees stop, landing attempt and end."""
    from telloedu import create_tello
    if mode not in ('sim','real'):raise ValueError('MODE must be sim or real')
    run=LabRun(name,mode,config={'duration_s':duration,'dt_s':dt,**kwargs})
    drone=create_tello(mode,host=host,gui=(mode=='sim' if gui is None else gui),simulation_mode='observe',simulation_speed=1.0)
    airborne=False
    try:
        drone.connect()
        if mode=='real':
            batt=int(drone.get_battery())
            if batt<45:raise RuntimeError(f'Battery too low for lab: {batt}%')
            if not kwargs.get('real_authorized',False):raise RuntimeError('REAL flight locked. Set REAL_AUTHORIZED=True only after preflight.')
        drone.takeoff();airborne=True
        procedure(drone,run,duration,dt,mode)
    finally:
        try:drone.send_rc_control(0,0,0,0)
        except Exception:pass
        if airborne:
            try:drone.land()
            except Exception as e:print('Landing failed:',e)
        try:drone.end()
        except Exception as e:print('Cleanup failed:',e)
        run.save()
    return run

def clamp(x,limit):return float(np.clip(x,-limit,limit))

def altitude_pid(drone,run,duration,dt,mode,target=1.0):
    integ=0.;last_err=0.
    for i in range(int(duration/dt)):
        h=height_m(drone);err=target-h
        integ=clamp(integ+err*dt,1.0)
        deriv=(err-last_err)/dt;last_err=err
        u=clamp(48*err+9*integ+7*deriv,25)
        drone.control_step(0,0,int(u),0,dt=dt)
        run.add(t_s=i*dt,z_ref=target,z_meas=h,error_z=err,u_ud=u,battery_pct=drone.get_battery())

def get_frame(drone):
    frame=drone.get_frame_read().frame
    if frame is None:raise RuntimeError('Camera returned no frame')
    return np.asarray(frame).copy()


def telemetry(drone, mode):
    """Actual backend readings only. GT exclusively SIM; no invented REAL XYZ."""
    out = {'height_m':float(drone.get_height())/100.,
           'roll_deg':float(drone.get_roll()), 'pitch_deg':float(drone.get_pitch()),
           'yaw_deg':float(drone.get_yaw()), 'battery_pct':float(drone.get_battery())}
    try:
        v=np.asarray(drone.get_velocity(),float)
        out.update(vx_mps=v[0],vy_mps=v[1],vz_mps=v[2])
    except Exception: pass
    if mode=='sim':
        pos=np.asarray(drone.get_position(),float)
        out.update(x_gt=pos[0],y_gt=pos[1],z_gt=pos[2])
    return out


def record_imu(drone):
    """Gyro on REAL is derived from Euler telemetry, not raw hardware gyro."""
    imu=drone.get_imu(); a=np.asarray(imu['accel'],float); g=np.asarray(imu['gyro'],float)
    return {'ax_mps2':a[0], 'ay_mps2':a[1], 'az_mps2':a[2],
            'gx_radps':g[0], 'gy_radps':g[1], 'gz_radps':g[2]}


def safe_stop(drone):
    try: drone.send_rc_control(0,0,0,0)
    except Exception: pass


def require_real_authorization(drone, authorized, minimum_battery=45):
    if not authorized: raise RuntimeError('REAL disabled. Complete preflight, then set REAL_AUTHORIZED=True.')
    battery=int(drone.get_battery())
    if battery<minimum_battery: raise RuntimeError(f'Battery {battery}% below {minimum_battery}%.')


def yaw_rotate_xy(vector, yaw_deg):
    """Body horizontal vector to world horizontal frame; convention must be verified on REAL."""
    theta=np.deg2rad(float(yaw_deg)); c=np.cos(theta);s=np.sin(theta)
    return np.array([c*vector[0]-s*vector[1],s*vector[0]+c*vector[1]])


def initialize_accel_reference(drone, n=15):
    """Calibrate stationary specific-force baseline (NOT raw IMU bias)."""
    samples=[np.asarray(drone.get_imu()['accel'],float) for _ in range(n)]
    return np.median(np.asarray(samples),axis=0)
