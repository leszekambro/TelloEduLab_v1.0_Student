"""Multi-drone PyBullet simulation with per-drone control and logging."""
import csv, time
from pathlib import Path
import numpy as np
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

class SwarmDrone:
    def __init__(self,swarm,index): self.swarm=swarm; self.index=index; self.name=swarm.names[index]; self.host=f"sim://{index+1}"
    def get_position(self): return self.swarm.positions()[self.index].copy()
    def get_velocity(self): return np.asarray(self.swarm.obs)[self.index,10:13].copy()
    def get_height(self): return int(round(self.get_position()[2]*100))
    def get_roll(self): return int(np.rad2deg(self.swarm.obs[self.index][7]))
    def get_pitch(self): return int(np.rad2deg(self.swarm.obs[self.index][8]))
    def get_yaw(self): return int(np.rad2deg(self.swarm.obs[self.index][9]))
    def get_battery(self): return 100
    def get_state(self): return self.swarm.get_states()[self.index]
    def goto(self,xyz,seconds=2.): self.swarm.targets[self.index]=np.asarray(xyz,float); return self.swarm.hold(seconds)
    def send_rc_control(self,lr,fb,ud,yaw): self.swarm.set_rc(self.index,lr,fb,ud,yaw)
    def get_flight_log(self): return self.swarm.get_flight_log(self.index)

class SimSwarm:
    def __init__(self,num_drones=3,names=None,gui=True,spacing=.8,ctrl_freq=48,simulation_mode='observe',simulation_speed=1.0,**kwargs):
        self.n=int(num_drones)
        if self.n<1: raise ValueError('num_drones must be >= 1')
        self.names=names or [f"drone{i+1}" for i in range(self.n)]
        if len(self.names)!=self.n: raise ValueError('names length must equal num_drones')
        self.dt=1/ctrl_freq; self.mode=simulation_mode; self.speed=float(simulation_speed); self.sim_time=0.; self._rc=np.zeros((self.n,4)); self._rc_active=np.zeros(self.n,dtype=bool)
        init=np.array([[0,(i-(self.n-1)/2)*spacing,.10] for i in range(self.n)],float)
        self.env=CtrlAviary(drone_model=DroneModel.CF2X,num_drones=self.n,initial_xyzs=init,physics=Physics.PYB,pyb_freq=240,ctrl_freq=ctrl_freq,gui=gui)
        self.obs,_=self.env.reset(seed=42); self.ctrl=[DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(self.n)]; self.targets=init.copy(); self.drones=[SwarmDrone(self,i) for i in range(self.n)]
        self.logs=[[] for _ in range(self.n)]
    def __len__(self): return self.n
    def __getitem__(self,i): return self.drones[i]
    def connect(self): return True
    def positions(self): return np.asarray(self.obs)[:,0:3].copy()
    def set_rc(self,i,lr,fb,ud,yaw): self._rc[i]=np.clip([lr,fb,ud,yaw],-100,100); self._rc_active[i]=True
    def step(self):
        started=time.perf_counter(); actions=[]
        for i in range(self.n):
            st=np.asarray(self.obs[i]); tv=np.zeros(3); tr=np.array([0.,0.,0.])
            if self._rc_active[i]:
                lr,fb,ud,yaw=self._rc[i]; psi=st[9]; vb=np.array([fb,lr,ud])/100.; c,s=np.cos(psi),np.sin(psi); tv=np.array([c*vb[0]-s*vb[1],s*vb[0]+c*vb[1],vb[2]]); self.targets[i]=st[:3]+tv*.35; tr[2]=psi+np.deg2rad(yaw*.35)*self.dt
            rpm,_,_=self.ctrl[i].computeControlFromState(control_timestep=self.dt,state=st,target_pos=self.targets[i],target_vel=tv,target_rpy=tr); actions.append(rpm)
        self.obs,_,_,_,_=self.env.step(np.asarray(actions)); self.sim_time+=self.dt; self._log()
        if self.mode=='observe': time.sleep(max(0,self.dt/self.speed-(time.perf_counter()-started)))
    def _log(self):
        for i,s in enumerate(np.asarray(self.obs)):
            self.logs[i].append([self.sim_time,*s[:3],*np.rad2deg(s[7:10]),*s[10:13],s[2]*100,s[2]*100,100,*self._rc[i]])
    def hold(self,seconds):
        for _ in range(max(1,int(seconds/self.dt))): self.step()
    def takeoff(self,height=1.): self._rc_active[:]=False; self.targets[:,2]=height; self.hold(3.)
    def land(self): self._rc_active[:]=False; self.targets[:,2]=.08; self.hold(3.)
    def emergency(self): self.targets[:,2]=.08
    def send_rc_control(self,lr=0,fb=0,ud=0,yaw=0):
        for i in range(self.n): self.set_rc(i,lr,fb,ud,yaw)
    def control_step(self,commands=None,dt=.05):
        commands=commands or [(0,0,0,0)]*self.n
        if len(commands)!=self.n: raise ValueError('commands must contain one tuple per drone')
        for i,c in enumerate(commands): self.set_rc(i,*c)
        for _ in range(max(1,int(round(dt/self.dt)))): self.step()
    def parallel(self,fn): return [fn(i,d) for i,d in enumerate(self.drones)]
    def sequential(self,fn): return [fn(i,d) for i,d in enumerate(self.drones)]
    def get_states(self):
        out=[]
        for i,s in enumerate(np.asarray(self.obs)):
            out.append({'name':self.names[i],'host':f'sim://{i+1}','position':s[:3].copy(),'velocity_mps':s[10:13].copy(),'rpy_deg':np.rad2deg(s[7:10]),'height_cm':s[2]*100,'tof_cm':s[2]*100,'battery_pct':100})
        return out
    def get_flight_log(self,i):
        cols=['time_s','x_m','y_m','z_m','roll_deg','pitch_deg','yaw_deg','vx_mps','vy_mps','vz_mps','height_cm','tof_cm','battery_pct','cmd_lr','cmd_fb','cmd_ud','cmd_yaw']; a=np.asarray(self.logs[i],float); return {k:(a[:,j] if len(a) else np.array([])) for j,k in enumerate(cols)}
    def clear_flight_log(self): self.logs=[[] for _ in range(self.n)]; self.sim_time=0.
    def save_flight_data(self,directory='logs',prefix='swarm'):
        cols=['time_s','x_m','y_m','z_m','roll_deg','pitch_deg','yaw_deg','vx_mps','vy_mps','vz_mps','height_cm','tof_cm','battery_pct','cmd_lr','cmd_fb','cmd_ud','cmd_yaw']; path=Path(directory); path.mkdir(parents=True,exist_ok=True); files=[]
        for i,rows in enumerate(self.logs):
            f=path/f'{prefix}_{i+1}_{self.names[i]}.csv';
            with f.open('w',newline='') as h: w=csv.writer(h); w.writerow(cols); w.writerows(rows)
            files.append(str(f))
        return files
    def plot_flight_data(self,show=True):
        import matplotlib.pyplot as plt
        figs=[]
        for i in range(self.n):
            a=self.get_flight_log(i)
            if not len(a['time_s']): continue
            fig,ax=plt.subplots(3,1,sharex=True,figsize=(9,7)); figs.append(fig)
            for j,k in enumerate(('x_m','y_m','z_m')): ax[j].plot(a['time_s'],a[k]); ax[j].set_ylabel(k); ax[j].grid(True)
            ax[-1].set_xlabel('Time [s]'); fig.suptitle(f'{self.names[i]}: position'); fig.tight_layout()
        if show: plt.show()
        return figs
    def set_formation(self,kind='line',spacing=.8,center=(0,0,1.)):
        self._rc_active[:]=False; cx,cy,cz=center
        if kind=='line': offs=np.array([[0,(i-(self.n-1)/2)*spacing,0] for i in range(self.n)])
        elif kind=='column': offs=np.array([[(i-(self.n-1)/2)*spacing,0,0] for i in range(self.n)])
        elif kind=='circle': offs=np.array([[spacing*np.cos(2*np.pi*i/self.n),spacing*np.sin(2*np.pi*i/self.n),0] for i in range(self.n)])
        elif kind=='square':
            if self.n>4: raise ValueError('square is defined for <=4 drones; use circle/grid for larger swarms')
            offs=np.array([[-.5,-.5,0],[-.5,.5,0],[.5,.5,0],[.5,-.5,0]])[:self.n]*spacing
        elif kind=='grid':
            cols=int(np.ceil(np.sqrt(self.n))); offs=np.array([[((i//cols)-(np.ceil(self.n/cols)-1)/2)*spacing,((i%cols)-(cols-1)/2)*spacing,0] for i in range(self.n)])
        else: raise ValueError('formation: line, column, circle, square, grid')
        self.targets=np.array([cx,cy,cz])+offs; return self.targets.copy()
    def leader_follower(self,leader_target,offsets=None,seconds=5.):
        offsets=np.asarray(offsets if offsets is not None else [[-.7*i,0,0] for i in range(self.n)],float); leader_target=np.asarray(leader_target,float)
        for i in range(self.n): self.targets[i]=leader_target+offsets[i]
        self._rc_active[:]=False; self.hold(seconds)
    def end(self): self.env.close()
    close=end

Swarm=SimSwarm
