import numpy as np

# Pass/fail rules for a scenario's result. Each one returns a rule: a function that takes the result
# and raises AssertionError with a readable message when it's broken.
#
# ALWAYS (bottom of the file) runs on every scenario, so a scenario only lists what's special about it.

def describe(description):
    def attach(rule):
        rule.description = description
        return rule
    return attach

# True for the ticks between after_s and until_s
def window(result, after_s=0.0, until_s=None):
    mask = result.time_s >= after_s
    if until_s is not None:
        mask &= result.time_s <= until_s
    assert mask.any(), f"nothing to check between {after_s} s and {until_s} s, the run is {result.time_s[-1]:.2f} s long"
    return mask

def firstTime(result, flags):
    return result.time_s[np.argmax(flags)]


# ---- torque

def torqueAbout(torque_Nm, after_s=0.0, until_s=None, tolerance=0.02):
    @describe(f"torque feedback about {torque_Nm} Nm (+/-{tolerance:.0%}) from {after_s} s")
    def rule(result):
        torque_Nm_in_window = result.torque_feedback_Nm[window(result, after_s, until_s)]
        error_Nm = np.abs(torque_Nm_in_window - torque_Nm)
        allowed_Nm = tolerance * abs(torque_Nm) + 0.05
        worst_Nm = torque_Nm_in_window[np.argmax(error_Nm)]
        assert np.all(error_Nm <= allowed_Nm), \
            f"torque feedback reached {worst_Nm:.2f} Nm, expected {torque_Nm} Nm +/-{allowed_Nm:.2f}"
    return rule

# None = the motor's own max. the current loop can overshoot for a few ms after a sudden step
# (like a pedal slammed from 0 to 100%), so a short, small overshoot is allowed
def torqueNeverAbove(torque_Nm=None, tolerance=0.02, overshoot=0.15, overshoot_s=0.005):
    @describe(f"torque feedback never above {torque_Nm or 'the motor max'} (bar a short overshoot)")
    def rule(result):
        limit_Nm = result.peak_torque_Nm if torque_Nm is None else torque_Nm
        torque_Nm_abs = np.abs(result.torque_feedback_Nm)
        worst_Nm = np.max(torque_Nm_abs)
        assert worst_Nm <= (1 + overshoot) * limit_Nm, \
            f"torque feedback reached {worst_Nm:.2f} Nm, limit is {limit_Nm} Nm"
        over_s = np.sum(torque_Nm_abs > (1 + tolerance) * limit_Nm) * result.record_every * result.control_period_s
        assert over_s <= overshoot_s, f"torque feedback was over {limit_Nm} Nm for {over_s * 1000:.1f} ms"
    return rule


# ---- motion

def accelerates():
    @describe("speeds up until it tops out, never slows down")
    def rule(result):
        top = np.argmax(result.speed_mps)
        # ignores wiggles under 0.1 mm/s (current loop settling while it's still parked)
        assert np.all(np.diff(result.speed_mps[:top + 1]) >= -1e-4), "the car slowed down before reaching its top speed"
        assert result.speed_mps[top] - result.speed_mps[-1] < 0.1 / 3.6, "the car lost speed after its top speed"
    return rule

# no torque and slowing down from drag and rolling resistance alone
def coasts(after_s):
    @describe(f"coasts from {after_s} s")
    def rule(result):
        mask = window(result, after_s)
        worst_Nm = np.max(np.abs(result.torque_feedback_Nm[mask]))
        assert worst_Nm < 0.1, f"still {worst_Nm:.2f} Nm of torque while it should be coasting"
        assert np.all(np.diff(result.speed_mps[mask]) <= 1e-12), "the car sped up while coasting"
    return rule

def staysParked():
    @describe("never moves")
    def rule(result):
        assert np.max(result.speed_mps) == 0, f"the car moved, up to {np.max(result.speed_mps) * 3.6:.2f} km/h"
    return rule

def reachesDistance(distance_m, within_s=None):
    @describe(f"reaches {distance_m} m" + (f" within {within_s} s" if within_s else ""))
    def rule(result):
        assert result.distance_m[-1] >= distance_m, f"only got to {result.distance_m[-1]:.1f} m"
        reached_s = firstTime(result, result.distance_m >= distance_m)
        assert within_s is None or reached_s <= within_s, f"took {reached_s:.2f} s to reach {distance_m} m"
    return rule

def reachesSpeed(speed_kph, within_s=None):
    @describe(f"reaches {speed_kph} km/h" + (f" within {within_s} s" if within_s else ""))
    def rule(result):
        top_kph = np.max(result.speed_mps) * 3.6
        assert top_kph >= speed_kph, f"only got to {top_kph:.1f} km/h"
        reached_s = firstTime(result, result.speed_mps * 3.6 >= speed_kph)
        assert within_s is None or reached_s <= within_s, f"took {reached_s:.2f} s to reach {speed_kph} km/h"
    return rule


# ---- bms

def noOverpower():
    @describe("pack never flagged over 80 kW")
    def rule(result):
        assert not result.overpower.any(), \
            f"bms flagged overpower at {firstTime(result, result.overpower):.2f} s (peak {np.max(result.bms_power_kW):.1f} kW)"
    return rule

def overpowerFlagged():
    @describe("bms flags the pack going over 80 kW")
    def rule(result):
        assert result.overpower.any(), \
            f"expected an overpower flag, but the bms only ever measured {np.max(result.bms_power_kW):.1f} kW"
    return rule

def noShutdown():
    @describe("bms never shuts down")
    def rule(result):
        assert not result.shutdown.any(), f"bms shut down at {firstTime(result, result.shutdown):.2f} s"
    return rule

# shuts down, stays latched, and nothing flows after that
def shutsDown(by_s=None):
    @describe("bms shuts down and stays off" + (f" by {by_s} s" if by_s else ""))
    def rule(result):
        assert result.shutdown.any(), "expected a bms shutdown, none happened"
        first = np.argmax(result.shutdown)
        assert by_s is None or result.time_s[first] <= by_s, f"shut down at {result.time_s[first]:.2f} s, expected by {by_s} s"
        after = slice(first + 1, None)
        assert result.shutdown[after].all(), "the shutdown didn't stay latched"
        assert np.all(result.torque_feedback_Nm[after] == 0), "torque after the shutdown"
        assert np.all(result.pack_current_A[after] == 0), "pack current after the shutdown"
    return rule


# ---- rules every scenario has to follow

# on every vcu loop (10 ms) the request is the pedal it just read, and it only ever changes on a loop
def vcuUpdatesOnItsLoop():
    @describe("vcu request only changes on its loop and follows the pedal")
    def rule(result):
        periods_per_loop = round(result.vcu_loop_period_s / result.control_period_s)
        on_loop = result.tick % periods_per_loop == 0
        pedal_percent = np.clip(result.apps_percent[on_loop], 0, 100)
        assert np.allclose(result.torque_request_Nm[on_loop], pedal_percent / 100 * result.max_torque_request_Nm), \
            "a vcu request didn't match the pedal it read"

        # only checkable when every tick was logged
        if result.record_every == 1:
            changes = np.nonzero(np.diff(result.torque_request_Nm))[0] + 1
            off_loop = changes[result.tick[changes] % periods_per_loop != 0]
            assert len(off_loop) == 0, f"vcu request changed off its loop at {result.time_s[off_loop[0]]:.4f} s"
    return rule

# the bms flag follows its last reading, and each reading is the pack power when it sampled
# (worked out from its own rounded cell voltages, so a hair of difference is allowed)
def bmsFlagMatchesItsReading():
    @describe("bms overpower flag matches what it measured")
    def rule(result):
        threshold_kW = result.power_threshold_kW[0]
        assert np.all(result.overpower == (result.bms_power_kW > threshold_kW)), \
            "the overpower flag doesn't match the bms's own power reading"
        periods_per_sample = round(result.bms_sample_period_s / result.control_period_s)
        sampled = result.tick % periods_per_sample == 0
        assert np.allclose(result.bms_power_kW[sampled], result.pack_power_W[sampled] / 1000, rtol=0.005, atol=0.05), \
            "a bms reading doesn't match the pack power at the moment it sampled"
    return rule

ALWAYS = [torqueNeverAbove(), vcuUpdatesOnItsLoop(), bmsFlagMatchesItsReading()]
