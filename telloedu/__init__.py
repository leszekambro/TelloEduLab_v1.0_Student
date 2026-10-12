"""TelloEduLab public API. Heavy simulation/hardware dependencies are lazy-loaded."""
from .factory import create_tello
from .swarm import Swarm
from .common.pid import PID
from .research import Experiment, DataRecorder
from .localization import create_localization
__version__="0.9.0-course-preview"
__all__=['Tello','RealTello','create_tello','World','Swarm','PID','Experiment','DataRecorder','create_localization']

def __getattr__(name):
    if name=='Tello':
        from .sim.tello import Tello; return Tello
    if name=='RealTello':
        from .real.tello import RealTello; return RealTello
    if name=='World':
        from .sim.world import World; return World
    raise AttributeError(name)
