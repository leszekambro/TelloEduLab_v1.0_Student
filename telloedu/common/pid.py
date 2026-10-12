class PID:
    """Small teaching PID with output and integral limits."""
    def __init__(self, kp, ki=0.0, kd=0.0, output_limits=(-100, 100), integral_limit=None):
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.output_limits = output_limits
        self.integral_limit = integral_limit
        self.reset()
    def reset(self):
        self.integral = 0.0; self.prev_error = None
    def update(self, error, dt):
        dt = max(float(dt), 1e-6); error = float(error)
        self.integral += error * dt
        if self.integral_limit is not None:
            lim = abs(float(self.integral_limit)); self.integral = max(-lim, min(lim, self.integral))
        derivative = 0.0 if self.prev_error is None else (error-self.prev_error)/dt
        self.prev_error = error
        u = self.kp*error + self.ki*self.integral + self.kd*derivative
        lo, hi = self.output_limits
        return max(lo, min(hi, u))
