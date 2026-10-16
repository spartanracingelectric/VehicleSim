class VCUConfig:
    # stand-in for the flashed VCU: turns the pedal (APPS %) into a torque request per motor, once per loop.
    # the real firmware's power limiting and traction control aren't modeled here (the bms only flags overpower)
    def __init__(self, loop_period_s, max_torque_request_Nm):
        self.loop_period_s = loop_period_s                   # how often the VCU runs its loop
        self.max_torque_request_Nm = max_torque_request_Nm   # request per motor at 100% pedal

        self.apps_percent = 0.0
        self.torque_request_Nm = 0.0
        self.time_to_next_loop_s = 0.0    # runs its first loop on the first update

    # call every sim step. the request only changes when a loop comes round, in between it holds the last one
    def update(self, apps_percent, dt_s):
        if self.time_to_next_loop_s < dt_s / 2:
            self.runLoop(apps_percent)
            self.time_to_next_loop_s += self.loop_period_s
        self.time_to_next_loop_s -= dt_s
        return self.torque_request_Nm

    # one VCU loop: read the pedal, map it to torque
    # TODO: swap the straight line for the real pedal map from the firmware (deadband etc.)
    def runLoop(self, apps_percent):
        self.apps_percent = max(0.0, min(apps_percent, 100.0))
        self.torque_request_Nm = self.apps_percent / 100 * self.max_torque_request_Nm

    def getTorqueRequest_Nm(self):
        return self.torque_request_Nm
