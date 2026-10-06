# Reusable PID controller: output = feedforward + Kp*error + Ki*integral(error) + Kd*d(error)/dt
#
# one loop with its own output limits:
#     controller = PIDController(Kp=2.0, Ki=0.5, output_min=0.0, output_max=100.0)
#     output = controller.update(setpoint - measured, dt_s)
#
# when the limit depends on more than this one output (like a d/q voltage vector),
# get the output, limit it yourself, then tell the controller whether it got saturated:
#     output = controller.getOutput(error, dt_s)
#     ...apply the limit...
#     controller.updateState(error, dt_s, integrate=not saturated)

class PIDController:
    def __init__(self, Kp, Ki=0.0, Kd=0.0, output_min=None, output_max=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.reset()

    # clears the controller's memory
    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.saturated = False

    # output for this error, without changing the controller's memory
    def getOutput(self, error, dt_s, feedforward=0.0):
        if dt_s <= 0:
            raise ValueError("Time step must be positive")

        integral = self.integral + error * dt_s
        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt_s

        return feedforward + self.Kp * error + self.Ki * integral + self.Kd * derivative

    # saves this step into the controller's memory
    # anti-windup: pass integrate=False while the output is saturated so the integral doesn't run away
    def updateState(self, error, dt_s, integrate=True):
        if integrate:
            self.integral += error * dt_s
        self.previous_error = error

    # one full step: output, clamp to output_min/output_max, update memory
    def update(self, error, dt_s, feedforward=0.0):
        output = self.getOutput(error, dt_s, feedforward)

        limited_output = output
        if self.output_max is not None:
            limited_output = min(limited_output, self.output_max)
        if self.output_min is not None:
            limited_output = max(limited_output, self.output_min)

        self.saturated = limited_output != output
        self.updateState(error, dt_s, integrate=not self.saturated)
        return limited_output

    def getIntegral(self):
        return self.integral

    def isSaturated(self):
        return self.saturated
