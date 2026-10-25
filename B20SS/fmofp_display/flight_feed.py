"""What the PFD reads from the FMS, made from the PFD terminal's bus data.

PrimaryFlightDisplay.update_flight_data() polls fms.get_flight_data() and
fms_control.get_tactical_status() sixty times a second. FlightDataFeed answers
both from PFD_FLIGHT_DATA, PFD_FMS_STATUS and PFD_ATTITUDE_STATUS as the
display terminal receives them (ICD v1.5). A field whose SSM is not NORMAL is
left out of the dictionary, so the display's own validity tracking lets it age
out and draws the amber X (README section 4, "Python displays"); when nothing
fresh has arrived the dictionary is empty and every tape goes amber.
"""

from fmofp_bus.icd import messages as m
from fmofp_bus.icd.enums import FmsFlightMode, Ssm
from fmofp_display.terminal import STALE_AFTER_S

# fmsControl's flight profiles: max g and max angle of attack per mode, as the
# fms_flight_mode enum documents them. The PFD scales its tactical indicators by these.
PROFILE_LIMITS = {
    FmsFlightMode.NORMAL: (2.5, 15.0),
    FmsFlightMode.COMBAT: (9.0, 25.0),
    FmsFlightMode.STEALTH: (5.0, 20.0),
    FmsFlightMode.TRAINING: (7.0, 22.0),
    FmsFlightMode.EMERGENCY: (9.5, 30.0),
}
DEFAULT_LIMITS = (9.0, 25.0)  # pfd.py's own defaults when the FMS said nothing

# FMS_ATTITUDE_STATUS limited bits -> the warning names fmsControl.update_attitude raised.
ENVELOPE_WARNINGS = (("bank_limited", "BANK_ANGLE"), ("pitch_limited", "PITCH_ANGLE"), ("roll_rate_limited", "ROLL_RATE"),
                     ("pitch_rate_limited", "PITCH_RATE"), ("yaw_rate_limited", "YAW_RATE"))


def _valid(ssm):
    return ssm == Ssm.NORMAL


class FlightDataFeed:
    """Stands in for the FMS and the FMS control the PFD polls."""

    def __init__(self, terminal, stale_after_s=STALE_AFTER_S):
        self.terminal = terminal
        self.stale_after_s = stale_after_s

    def get_flight_data(self):
        """pfd.update_flight_data()'s dictionary, with only the fields the bus vouches for."""
        message = self.terminal.fresh(m.PfdFlightData, self.stale_after_s)
        if message is None:
            return {}
        p = message.payload
        out = {}
        attitude = {}
        if _valid(p.roll_ssm):
            attitude["roll"] = p.roll
        if _valid(p.pitch_ssm):
            attitude["pitch"] = p.pitch
        if attitude:
            out["attitude"] = attitude
        velocity = {}
        if _valid(p.airspeed_ssm):
            velocity["airspeed"] = p.airspeed
        if _valid(p.vertical_speed_ssm):
            velocity["vertical_speed"] = p.vertical_speed
        if _valid(p.mach_ssm):
            velocity["mach"] = p.mach
        if velocity:
            out["velocity"] = velocity
        navigation = {}
        if _valid(p.heading_ssm):
            navigation["heading"] = p.heading
        if _valid(p.altitude_ssm):
            navigation["altitude"] = p.altitude
        if navigation:
            out["navigation"] = navigation
        tactical = {}
        if _valid(p.g_ssm):
            tactical["g_force"] = p.g_force
        if _valid(p.aoa_ssm):
            tactical["aoa"] = p.angle_of_attack
        if _valid(p.energy_ssm):
            tactical["energy_state"] = p.energy_state
        if tactical:
            out["tactical"] = tactical
        out["status"] = {"mode": p.flight_mode.name}
        return out

    def get_tactical_status(self):
        """fmsControl.get_tactical_status()'s dictionary, as far as the PFD reads it."""
        status = self.terminal.fresh(m.PfdFmsStatus, self.stale_after_s)
        attitude = self.terminal.fresh(m.PfdAttitudeStatus, self.stale_after_s)
        mode = status.payload.flight_mode if status is not None else None
        max_g, max_aoa = PROFILE_LIMITS.get(mode, DEFAULT_LIMITS)
        warnings = []
        if attitude is not None:
            warnings = [name for field, name in ENVELOPE_WARNINGS if getattr(attitude.payload, field)]
        return {"mode": mode.name if mode is not None else "UNKNOWN", "envelope_warnings": warnings,
                "profile_limits": {"max_g_force": max_g, "max_aoa": max_aoa}}

    def status_text(self):
        message, age = self.terminal.latest(m.PfdFlightData)
        if message is None:
            return "FMS: no data"
        p = message.payload
        stale = " (stale)" if age is not None and age > self.stale_after_s else ""
        return f"FMS {p.flight_mode.name}  alt {p.altitude:.0f} ft  ias {p.airspeed:.0f} kt  hdg {p.heading:.0f}{stale}"
