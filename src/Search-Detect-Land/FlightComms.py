"""
Shared MAVLink helpers for the Search-Detect-Land scripts.

These started life inside SpinOnGuided.py, where they were what turned "it
doesn't spin" from a guess into a diagnosis. Now that the GUIDED_NOGPS attitude
fix is CONFIRMED against the vehicle (2026-07-27), every script that commands
the FC gets the same pre-flight checks, so no future run can fail silently in a
way we have already learned to detect.

pymavlink ONLY -- deliberately no cv2 / picamera2 / numpy. SpinOnGuided is the
dependency-light diagnostic tool and must stay runnable on a machine where the
camera stack is missing or broken, so it cannot pull these in via
LandOnAprilTag (which imports the whole vision pipeline at module level).

What lives here
---------------
  request_message_interval  ask the FC to stream a message at a given rate
  read_param                read one parameter -- and prove Pi -> FC works
  check_tx_path             startup self-test; refuses to fly a dead link
  GroundGate                landed_state / armed tracking + the ON_GROUND warning
"""

import time

from pymavlink import mavutil


# ======================================================================
# STREAM SETUP
# ======================================================================

def request_message_interval(mav, msg_id, hz):
    """Ask the FC to stream msg_id at hz. Best effort; no ack is waited for."""
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


# ======================================================================
# PI -> FC PROOF OF LIFE
# ======================================================================

def read_param(mav, name, timeout=2.0):
    """Read one parameter. Returns its value, or None on timeout.

    Doubles as our Pi->FC proof of life: a PARAM_VALUE coming back for the
    exact name we asked for means our bytes reached the FC and were parsed.
    """
    mav.mav.param_request_read_send(
        mav.target_system, mav.target_component, name.encode('ascii'), -1)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
        if msg is None:
            continue
        param_id = msg.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode('ascii', 'ignore')
        if param_id.strip('\x00') == name:
            return msg.param_value
    return None


def check_tx_path(mav, log, hover_thrust):
    """Prove the Pi->FC direction works, and dump the params that decide what
    our thrust field means. Returns True if the FC answered us.

    Receiving telemetry only proves FC -> Pi. Nothing in a normal startup
    distinguishes "the FC is listening" from "our TX wire is dead and we are
    talking to ourselves" -- except a round-trip. Call this before commanding.
    """
    log.event("Checking the Pi -> FC direction (parameter round-trip) ...")

    guid_options = read_param(mav, 'GUID_OPTIONS')
    if guid_options is None:
        # One retry -- the first request can land while the FC is busy
        # streaming its startup banner.
        guid_options = read_param(mav, 'GUID_OPTIONS', timeout=3.0)

    if guid_options is None:
        log.event("!!! No reply. The FC is NOT receiving anything we send.")
        log.event("!!! Telemetry arriving proves only the FC -> Pi direction.")
        log.event("!!! Check: Pi TX -> FC RX wire, that this is the right port,")
        log.event("!!! and that the FC's SERIALn_PROTOCOL for it is MAVLink.")
        return False

    guid_timeout = read_param(mav, 'GUID_TIMEOUT')

    opts = int(guid_options)
    thrust_as_thrust = bool(opts & 8)     # GUID_OPTIONS bit 3
    log.event(f"    Pi -> FC OK.  GUID_OPTIONS={opts}  GUID_TIMEOUT="
              f"{'?' if guid_timeout is None else round(guid_timeout, 2)}")
    if thrust_as_thrust:
        log.event(f"    !! GUID_OPTIONS bit 3 set: thrust is raw THRUST, not "
                  f"climb rate. HOVER_THRUST={hover_thrust} does NOT mean "
                  f"'hold altitude' on this FC -- it means 50% throttle.")
    else:
        log.event(f"    thrust field = climb rate; HOVER_THRUST="
                  f"{hover_thrust} -> 0 m/s (note: a 0 climb rate while the FC "
                  f"still thinks it is landed means no response on the ground).")
    return True


# ======================================================================
# GROUND GATE
# ======================================================================

LANDED_STATE = {
    0: 'UNDEFINED',
    1: 'ON_GROUND',
    2: 'IN_AIR',
    3: 'TAKEOFF',
    4: 'LANDING',
}


class GroundGate:
    """Tracks whether the FC believes it is landed, and whether it is armed.

    Why this matters to every powered script, not just the spin test:
    ArduPilot's angle controller bails out to `make_safe_ground_handling()`
    while `land_complete` is true and we are not commanding a positive climb
    rate. At HOVER_THRUST = 0.5 (zero climb rate) that is exactly our case on
    the ground -- so a props-off bench run accepts every command and moves
    NOTHING. Without this warning that looks identical to the type_mask bug we
    just spent a flight diagnosing, which is the trap it exists to prevent.

    Usage:
        gate = GroundGate(log)
        gate.request(mav)                  # after wait_heartbeat()
        ...
        gate.observe(msg)                  # every message in the drain loop
        gate.on_active("spin")             # when control starts
        gate.poll()                        # at the status-line rate
    """

    def __init__(self, log):
        self.log = log
        self.landed_state = 0
        self.motors_armed = False
        self._warned = False

    def request(self, mav, hz=2):
        """Ask the FC to stream EXTENDED_SYS_STATE (source of landed_state)."""
        request_message_interval(
            mav, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, hz)

    def observe(self, msg):
        """Feed every drained message. Ignores the ones it doesn't care about."""
        mtype = msg.get_type()
        if mtype == 'EXTENDED_SYS_STATE':
            self.landed_state = msg.landed_state
        elif mtype == 'HEARTBEAT':
            self.motors_armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def on_ground(self):
        return self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

    @property
    def in_air(self):
        return self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR

    @property
    def name(self):
        return LANDED_STATE.get(self.landed_state, self.landed_state)

    def on_active(self, action="command"):
        """Call when control becomes active. Warns once if we are on the
        ground, where the FC will run ground idle instead of our attitude."""
        if self.on_ground:
            self._warned = True
            self.log.event(f">>> !! FC reports ON_GROUND -- it runs ground "
                           f"idle instead of the attitude controller, so the "
                           f"{action} will not move the vehicle until you are "
                           f"airborne.")

    def poll(self):
        """Call periodically. Announces the ON_GROUND -> IN_AIR transition so
        the log says exactly when the FC started acting on us."""
        if self._warned and self.in_air:
            self._warned = False
            self.log.event(">>> Now IN_AIR -- the FC will start acting on our "
                           "commands from here.")

    def reset(self):
        """Call when control stops, so a later start re-checks the gate."""
        self._warned = False
