"""World builder for TelloEduLab simulation."""
from pathlib import Path
import json, math
import numpy as np
import pybullet as p
from .aruco_marker import ArucoMarker

class World:
    """Build static/dynamic PyBullet scenes without exposing PyBullet to students."""
    def __init__(self, tello, auto_attach=True):
        self.tello=tello; self.backend=tello._sim; self.client_id=self.backend.client_id
        self.objects={}; self.dynamic=[]; self._counter=0
        if auto_attach: self.backend.world=self
    def _name(self,prefix): self._counter+=1; return f"{prefix}_{self._counter}"
    def add_box(self, position, size=(0.5,0.5,0.5), name=None, mass=0.0, rgba=(0.65,0.65,0.68,1)):
        size=np.asarray(size,float); half=(size/2).tolist()
        col=p.createCollisionShape(p.GEOM_BOX,halfExtents=half,physicsClientId=self.client_id)
        vis=p.createVisualShape(p.GEOM_BOX,halfExtents=half,rgbaColor=rgba,physicsClientId=self.client_id)
        bid=p.createMultiBody(baseMass=mass,baseCollisionShapeIndex=col,baseVisualShapeIndex=vis,basePosition=position,physicsClientId=self.client_id)
        name=name or self._name("box"); self.objects[name]=bid; return name
    def add_wall(self, position, size=(0.1,2.0,1.5), **kw): return self.add_box(position,size,**kw)
    def add_cylinder(self, position, radius=0.2, height=1.0, name=None, rgba=(0.55,0.55,0.58,1)):
        col=p.createCollisionShape(p.GEOM_CYLINDER,radius=radius,height=height,physicsClientId=self.client_id)
        vis=p.createVisualShape(p.GEOM_CYLINDER,radius=radius,length=height,rgbaColor=rgba,physicsClientId=self.client_id)
        bid=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=col,baseVisualShapeIndex=vis,basePosition=position,physicsClientId=self.client_id)
        name=name or self._name("cylinder"); self.objects[name]=bid; return name
    def add_sphere(self, position, radius=0.25, name=None, rgba=(0.6,0.6,0.65,1)):
        col=p.createCollisionShape(p.GEOM_SPHERE,radius=radius,physicsClientId=self.client_id)
        vis=p.createVisualShape(p.GEOM_SPHERE,radius=radius,rgbaColor=rgba,physicsClientId=self.client_id)
        bid=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=col,baseVisualShapeIndex=vis,basePosition=position,physicsClientId=self.client_id)
        name=name or self._name("sphere"); self.objects[name]=bid; return name
    def add_gate(self, position=(2,0,1), width=1.2, height=1.4, thickness=.10, name="gate"):
        x,y,z=position; self.add_box((x,y-width/2,z),(thickness,thickness,height),name+"_L")
        self.add_box((x,y+width/2,z),(thickness,thickness,height),name+"_R")
        self.add_box((x,y,z+height/2),(thickness,width+thickness,thickness),name+"_T"); return name
    def add_aruco(self, marker_id=23, size=.30, position=(2,0,1), yaw=180, name=None):
        obj=ArucoMarker(marker_id,size,position,yaw,client_id=self.client_id)
        name=name or f"aruco_{marker_id}"; self.objects[name]=obj; return obj
    def add_moving_box(self, start, end, size=(.35,.35,.8), speed=.5, name=None):
        name=self.add_box(start,size,name=name); self.dynamic.append({"name":name,"start":np.asarray(start,float),"end":np.asarray(end,float),"speed":float(speed)})
        return name
    def set_position(self,name,position):
        obj=self.objects[name]; bid=obj.body_id if hasattr(obj,"body_id") else obj
        _,q=p.getBasePositionAndOrientation(bid,physicsClientId=self.client_id)
        p.resetBasePositionAndOrientation(bid,position,q,physicsClientId=self.client_id)
    def remove(self,name):
        obj=self.objects.pop(name); bid=obj.body_id if hasattr(obj,"body_id") else obj
        p.removeBody(bid,physicsClientId=self.client_id)
    def update(self,t):
        for d in self.dynamic:
            a,b=d['start'],d['end']; L=float(np.linalg.norm(b-a))
            if L<1e-9: continue
            phase=(float(t)*d['speed']/L)%2.0; alpha=phase if phase<=1 else 2-phase
            self.set_position(d['name'],(a+alpha*(b-a)).tolist())
    def load(self,name):
        path=Path(__file__).resolve().parents[2]/"worlds"/f"{name}.json"
        if not path.exists(): raise FileNotFoundError(f"Unknown world '{name}'. Available JSON worlds are in {path.parent}")
        spec=json.loads(path.read_text())
        for o in spec.get('objects',[]):
            typ=o.pop('type'); getattr(self,f"add_{typ}")(**o)
        return self
