"""Physical Tello EDU swarm using independent station-mode IP addresses."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time
from .tello import RealTello

class RealSwarm:
    def __init__(self, ips, names=None, **kwargs):
        if not ips: raise ValueError("Real swarm requires at least one IP address")
        names=names or [f"drone{i+1}" for i in range(len(ips))]
        if len(names)!=len(ips): raise ValueError("names and ips must have the same length")
        self.drones=[RealTello(host=ip,name=name,**kwargs) for name,ip in zip(names,ips)]
        self.n=len(self.drones)
    def __len__(self): return self.n
    def __getitem__(self,i): return self.drones[i]
    def _parallel(self, fn):
        with ThreadPoolExecutor(max_workers=self.n) as ex: return list(ex.map(fn,self.drones))
    def connect(self): return self._parallel(lambda d:d.connect())
    def takeoff(self): return self._parallel(lambda d:d.takeoff())
    def land(self): return self._parallel(lambda d:d.land())
    def emergency(self): return self._parallel(lambda d:d.emergency())
    def end(self): return self._parallel(lambda d:d.end())
    close=end
    def parallel(self, fn):
        with ThreadPoolExecutor(max_workers=self.n) as ex: return list(ex.map(lambda x:fn(x[0],x[1]),enumerate(self.drones)))
    def sequential(self, fn): return [fn(i,d) for i,d in enumerate(self.drones)]
    def send_rc_control(self,lr=0,fb=0,ud=0,yaw=0): return self._parallel(lambda d:d.send_rc_control(lr,fb,ud,yaw))
    def control_step(self,commands=None,dt=.05):
        commands=commands or [(0,0,0,0)]*self.n
        if len(commands)!=self.n: raise ValueError("commands must contain one (lr,fb,ud,yaw) tuple per drone")
        with ThreadPoolExecutor(max_workers=self.n) as ex:
            out=list(ex.map(lambda x:x[1].send_rc_control(*commands[x[0]]),enumerate(self.drones)))
        time.sleep(max(.02,float(dt))); self.log_sample(); return out
    def get_states(self): return [d.get_state() for d in self.drones]
    def log_sample(self): return [d.log_sample() for d in self.drones]
    def clear_flight_log(self): [d.clear_flight_log() for d in self.drones]
    def save_flight_data(self,directory="logs",prefix="swarm"):
        path=Path(directory); path.mkdir(parents=True,exist_ok=True)
        return [d.save_flight_data(path/f"{prefix}_{i+1}_{d.name}.csv") for i,d in enumerate(self.drones)]
    def plot_flight_data(self,show=True):
        figs=[]
        for d in self.drones: figs.extend(d.plot_flight_data(show=False))
        if show:
            import matplotlib.pyplot as plt; plt.show()
        return figs
