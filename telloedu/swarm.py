"""Public backend-neutral swarm facade."""
class Swarm:
    def __new__(cls, num_drones=None, mode="sim", ips=None, names=None, **kwargs):
        mode=str(mode).lower()
        if mode in ("sim","simulation","simulator"):
            from .sim.swarm import SimSwarm
            n=int(num_drones if num_drones is not None else (len(ips) if ips else 3))
            return SimSwarm(num_drones=n,names=names,**kwargs)
        if mode in ("real","hardware","tello"):
            from .real.swarm import RealSwarm
            if not ips: raise ValueError("mode='real' requires ips=[...] (Tello EDU station-mode addresses)")
            return RealSwarm(ips=ips,names=names,**kwargs)
        raise ValueError("mode must be 'sim' or 'real'")
