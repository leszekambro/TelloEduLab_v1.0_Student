def create_tello(mode='sim', **kwargs):
    """Create one simulator or physical Tello without importing the unused backend."""
    mode=str(mode).lower()
    if mode in ('sim','simulation','simulator'):
        from .sim.tello import Tello
        return Tello(**kwargs)
    if mode in ('real','hardware','tello'):
        from .real.tello import RealTello
        return RealTello(**kwargs)
    raise ValueError("mode must be 'sim' or 'real'")
