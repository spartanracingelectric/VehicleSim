# Pedal shapes for a Scenario: each one returns a function of time (s) -> pedal % (APPS)
#
#   hold(30)                        30% the whole time
#   ramp(to=100, over_s=0.8)        0 -> 100% over 0.8 s, then holds 100%
#   steps((0, 100), (2.0, 0))       100% from 0 s, 0% from 2 s (each value holds until the next)
#   pulses(100, on_s=0.5, off_s=0.5) on/off/on/off...
#
# TODO: fromLog(csv_path, start_s, end_s) to replay APPS from a MoTeC log

def hold(percent):
    def pedal(time_s):
        return percent
    return pedal

def ramp(to=100.0, over_s=0.8, start_s=0.0):
    def pedal(time_s):
        if time_s <= start_s:
            return 0.0
        return min(to, to * (time_s - start_s) / over_s)
    return pedal

# (time_s, percent) pairs, each value holds until the next one
def steps(*points):
    points = sorted(points)
    def pedal(time_s):
        percent = 0.0
        for step_time_s, step_percent in points:
            if time_s >= step_time_s:
                percent = step_percent
        return percent
    return pedal

def pulses(percent, on_s, off_s, start_s=0.0):
    def pedal(time_s):
        if time_s < start_s:
            return 0.0
        time_in_pulse_s = (time_s - start_s) % (on_s + off_s)
        return percent if time_in_pulse_s < on_s else 0.0
    return pedal
