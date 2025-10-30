# closed_loop.py
# ==============================================================================
# PI Velocity Controller for Romi motors (in rad/s)
# ==============================================================================

from time import ticks_diff, ticks_ms

class ClosedLoop:
    """Proportional-Integral (PI) controller for velocity control in rad/s."""

    def __init__(self, Kp=0.0, Ki=0.0, setpoint=0.0, effort_limits=(-100, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.setpoint = setpoint
        self.effort_min, self.effort_max = effort_limits
        self.integrator = 0.0
        self.last_time = ticks_ms()
        self.output = 0.0

    def reset(self):
        """Reset controller integrator and state."""
        self.integrator = 0.0
        self.output = 0.0
        self.last_time = ticks_ms()

    def set_gains(self, Kp, Ki):
        self.Kp = Kp
        self.Ki = Ki

    def set_setpoint(self, sp):
        self.setpoint = sp

    def run(self, feedback_rad_per_s):
        """Compute control output (effort %) from velocity feedback."""
        now = ticks_ms()
        dt = max(ticks_diff(now, self.last_time), 1) / 1000.0
        self.last_time = now

        error = self.setpoint - feedback_rad_per_s
        self.integrator += error * dt

        u = self.Kp * error + self.Ki * self.integrator

        # Clamp output and apply simple anti-windup
        if u > self.effort_max:
            u = self.effort_max
            if error > 0:
                self.integrator -= error * dt
        elif u < self.effort_min:
            u = self.effort_min
            if error < 0:
                self.integrator -= error * dt

        self.output = u
        return u