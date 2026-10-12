from __future__ import annotations
import csv, json, platform, time
from datetime import datetime, timezone
from pathlib import Path
import numpy as np

class DataRecorder:
    """Synchronized experiment recorder for one drone or a swarm."""
    def __init__(self, output_dir, metadata=None):
        self.root=Path(output_dir); self.root.mkdir(parents=True,exist_ok=True)
        self.rows=[]; self.t0=time.perf_counter(); self.metadata=dict(metadata or {})
    def sample(self, drone, name='drone', reference=None, localization=None, command=None):
        t=time.perf_counter()-self.t0; row={'time_s':t,'drone':name}
        try: s=drone.get_state()
        except Exception: s={}
        p=np.asarray(s.get('position',[np.nan]*3),float); v=np.asarray(s.get('velocity_mps', s.get('velocity',[np.nan]*3)),float); r=np.asarray(s.get('rpy_deg',[np.nan]*3),float)
        for key,val in zip(('meas_x','meas_y','meas_z'),p): row[key]=val
        for key,val in zip(('vx','vy','vz'),v): row[key]=val
        for key,val in zip(('roll','pitch','yaw'),r): row[key]=val
        if localization is not None:
            est=localization.update(); row['localization']=est.source
            for key,val in zip(('est_x','est_y','est_z'),est.position): row[key]=val
        if reference is not None:
            ref=np.asarray(reference,float)
            for key,val in zip(('ref_x','ref_y','ref_z'),ref): row[key]=val
            ep=(np.asarray(row.get('est_x',p[0:1].tolist()+[np.nan,np.nan]) if False else [row.get('est_x',p[0]),row.get('est_y',p[1]),row.get('est_z',p[2])])-ref)
            row['position_error_m']=float(np.linalg.norm(ep))
        if command is not None:
            for key,val in zip(('cmd_lr','cmd_fb','cmd_ud','cmd_yaw'),command): row[key]=val
        self.rows.append(row); return row
    def sample_swarm(self, swarm, references=None, localizations=None, commands=None):
        for i,d in enumerate(swarm):
            self.sample(d, getattr(d,'name',f'drone{i+1}'),
                        None if references is None else references[i],
                        None if localizations is None else localizations[i],
                        None if commands is None else commands[i])
    def formation_error(self, positions, desired_offsets):
        p=np.asarray(positions,float); q=np.asarray(desired_offsets,float)
        errs=[]
        for i in range(len(p)):
            for j in range(i+1,len(p)):
                errs.append(np.linalg.norm((p[j]-p[i])-(q[j]-q[i])))
        return float(np.sqrt(np.mean(np.square(errs)))) if errs else 0.0
    def save(self):
        fields=sorted({k for r in self.rows for k in r}) if self.rows else ['time_s','drone']
        with (self.root/'data.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(self.rows)
        with (self.root/'metadata.json').open('w') as f: json.dump(self.metadata,f,indent=2,default=str)
        return self.root
    def plot_3d(self, show=True):
        import matplotlib.pyplot as plt
        fig=plt.figure(); ax=fig.add_subplot(111,projection='3d')
        for name in sorted({r['drone'] for r in self.rows}):
            rr=[r for r in self.rows if r['drone']==name]
            for prefix,label in [('meas','measured'),('est','estimated'),('ref','reference')]:
                xyz=np.array([[r.get(prefix+'_x',np.nan),r.get(prefix+'_y',np.nan),r.get(prefix+'_z',np.nan)] for r in rr])
                if np.isfinite(xyz).any(): ax.plot(xyz[:,0],xyz[:,1],xyz[:,2],label=f'{name} {label}')
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]'); ax.legend(); fig.tight_layout()
        if show: plt.show()
        return fig

class Experiment:
    def __init__(self, name, backend, vehicle=None, swarm=None, rate_hz=20, controller=None, parameters=None, output_root='runs'):
        stamp=datetime.now().strftime('%Y%m%d_%H%M%S'); self.name=name; self.rate_hz=float(rate_hz); self.dt=1/self.rate_hz
        hosts=[]
        targets=[vehicle] if vehicle is not None else list(swarm or [])
        for d in targets: hosts.append(getattr(d,'host',None))
        meta={'experiment':name,'created_utc':datetime.now(timezone.utc).isoformat(),'backend':backend,'rate_hz':self.rate_hz,
              'controller':controller,'parameters':parameters or {},'vehicle_names':[getattr(d,'name',f'drone{i+1}') for i,d in enumerate(targets)],
              'ips':hosts,'python':platform.python_version(),'platform':platform.platform(),'telloedulab_version':'0.8.1'}
        self.recorder=DataRecorder(Path(output_root)/f'{stamp}_{name}',meta); self.vehicle=vehicle; self.swarm=swarm
    def wait(self): time.sleep(self.dt)
    def save(self): return self.recorder.save()
