class SimulatorInspector:
    """Instructor/debug interface. Do not use in student solutions."""
    def __init__(self,tello): self.tello=tello
    def ground_truth(self): return self.tello._sim.state()
    def print_state(self):
        s=self.ground_truth(); print('GT position [m]:',s['pos']); print('GT RPY [rad]:',s['rpy']); print('GT velocity [m/s]:',s['vel'])
