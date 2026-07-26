"""
Spin-on-GUIDED_NOGPS test.

Companion-computer (Raspberry Pi) script for the Search-Detect-Land quad.

How the gate works
------------------
The RC 3-position switch on channel 6 selects the flight mode on the flight
controller itself (FLTMODE_CH=6; UP -> GUIDED_NOGPS, DOWN/MIDDLE -> STABILIZE).
So flipping the switch UP does two things at once:

  1. the FC changes mode to GUIDED_NOGPS, which is the only Copter mode that
     honours the offboard SET_ATTITUDE_TARGET messages this script sends, and
  2. this script -- which watches the FC's reported flight mode -- sees the
     mode become GUIDED_NOGPS and starts commanding a slow yaw spin.

Flip the switch back down and the FC returns to STABILIZE, which ignores every
command this script could send. The SWITCH (via the flight mode), not this
script, is the real safety gate: this script only ever commands the vehicle
while it observes GUIDED_NOGPS, and even if it misbehaved, STABILIZE would
drop its commands on the floor at the FC level.

Re-flicking STABILIZE <-> GUIDED_NOGPS is fine and repeatable: leaving
GUIDED_NOGPS stops the spin (you get manual control instantly), and returning
starts a fresh spin. There is ONE guard on top of that: after this process
(re)starts, it will NOT spin until it has seen the vehicle in a non-GUIDED_NOGPS
mode at least once (the `armed` latch below). That way an auto-restart while the
switch is already UP can't silently resume the spin -- it takes a deliberate
flick DOWN then UP.

  !!  BENCH-TEST FIRST, PROPS OFF.  !!
  Verify the yaw direction/rate and that the switch cleanly starts/stops the
  spin before ever doing this with props on. See the thrust/altitude note on
  HOVER_THRUST below -- in GUIDED_NOGPS the FC controls throttle, not you.


WHY THE FIRST VERSION OF THIS SCRIPT DID NOT SPIN  (fixed -- see TYPE_MASK)
--------------------------------------------------------------------------
It used `type_mask = 1 | 2`, i.e. "ignore body roll rate and body pitch rate,
but honour body yaw rate". That looks reasonable and it is what most tutorials
show, but ArduPilot rejects it outright. From the Copter handler
(ArduCopter/GCS_Mavlink.cpp on Copter-4.6.3, the firmware this vehicle runs):

    Vector3f ang_vel_body;
    if (!roll_rate_ignore && !pitch_rate_ignore && !yaw_rate_ignore) {
        ang_vel_body.x = packet.body_roll_rate;
        ang_vel_body.y = packet.body_pitch_rate;
        ang_vel_body.z = packet.body_yaw_rate;
    } else if (!(roll_rate_ignore && pitch_rate_ignore && yaw_rate_ignore)) {
        // The body rates are ill-defined
        // input is not valid so stop
        copter.mode_guided.init(true);
        return;
    }

The three body-rate ignore bits must be ALL SET or ALL CLEAR. Anything in
between is "ill-defined" and the message is discarded before it ever reaches
the attitude controller -- and each discarded message additionally calls
`mode_guided.init(true)`, re-initialising guided mode. So the old script was
not being politely ignored; it was resetting guided mode 10 times a second.
(Later firmware calls `hold_position()` here instead; same rejection either
way.)

The fix (below) is to provide all three body rates -- roll 0, pitch 0, yaw =
the spin -- alongside a level attitude quaternion, so `type_mask = 0`.

Because the FC does not integrate our yaw rate into the attitude target for us,
we also advance the quaternion's yaw ourselves each send (see `target_yaw`).
That keeps roll/pitch under proper self-levelling attitude control while the
heading walks round at YAW_RATE_DEGS, and it does not depend on the internals
of AC_AttitudeControl's rate feed-forward.

OTHER THINGS THAT STOP A GUIDED ATTITUDE COMMAND
------------------------------------------------
Kept here because they are the next things to check if it still misbehaves:

  1. NOTHING WE SEND REACHES THE FC (TX wired wrong / wrong port / port not set
     to a MAVLink protocol). Receiving telemetry only proves the FC->Pi
     direction. The startup self-test below proves Pi->FC by round-tripping a
     parameter read, and refuses to continue without it.

  2. ON THE GROUND / NOT auto_armed. `angle_control_run()` bails out early if
     the vehicle is disarmed or `auto_armed` is false, and does ground handling
     rather than running the attitude controller while it still believes it is
     landed. This does NOT apply to a mid-flight mode flip -- `land_complete` is
     false in the air and `auto_armed` latches true once you are armed with the
     throttle off the bottom stop -- but it does mean you will never see a spin
     from a bench/ground test at HOVER_THRUST = 0.5. The status line prints
     `landed=` and `motors=` so you can see which side of that you are on.

  3. `GUID_OPTIONS` has the "thrust as thrust" bit set, changing what the thrust
     field means (see HOVER_THRUST). Read and printed at startup.

  4. GUID_TIMEOUT. If updates stop for longer than this (default 3 s) the FC
     levels off and zeroes the climb rate. SEND_HZ = 10 is far inside it.

THE SLIGHT CLIMB WHEN YOU FLIP THE SWITCH
------------------------------------------
Mostly expected: in STABILIZE the throttle stick drives throttle directly, and
GUIDED_NOGPS hands throttle to the FC's altitude controller, which takes over at
whatever it thinks holds altitude -- usually a touch above what you were
holding, so it steps up and settles. It is NOT our thrust value; 0.5 maps to a
commanded climb rate of exactly zero.

Note though that the old rejected-message spam described above landed on the FC
from the very first message after the switch, re-initialising guided mode each
time, so it may have contributed. Worth re-checking now that the messages are
actually accepted.

Confirmed from this vehicle's own dataflash log: GUID_OPTIONS = 0, so thrust IS
a climb rate here and 0.5 really is zero climb. WPNAV_SPEED_UP = 250 cm/s and
WPNAV_SPEED_DN = 150 cm/s set the scaling, so LandOnAprilTag's
DESCEND_THRUST = 0.42 works out to (0.5-0.42) * 2 * 150 = 24 cm/s of descent.
"""

from pymavlink import mavutil
import math
import time

import FlightLog

# --- CONNECTION ---
CONNECTION_STRING = '/dev/ttyAMA0'
BAUD_RATE = 115200

# --- BEHAVIOUR ---
# pymavlink's name for Copter mode 20. Confirm this exact string against
# mav.flightmode on your own FC before relying on it (printed at startup).
TARGET_MODE = 'GUIDED_NOGPS'

YAW_RATE_DEGS = 45                       # ~1 rotation / 8 s. Deliberately
                                         # gentle: this is the first rate to
                                         # trust after the type_mask fix.
                                         # Raise once direction + the switch
                                         # start/stop are confirmed in the air.
YAW_RATE_RADS = math.radians(YAW_RATE_DEGS)

SEND_HZ = 10                             # comfortably inside GUID_TIMEOUT
SEND_PERIOD = 1.0 / SEND_HZ              # (default 3 s) so the spin never
                                         # stalls between commands

# type_mask for SET_ATTITUDE_TARGET.
#
# ZERO -- we provide EVERYTHING: the attitude quaternion, all three body rates,
# and the thrust. ArduPilot requires the three body-rate ignore bits
# (1 = roll, 2 = pitch, 4 = yaw) to be all set or all clear; a partial mask such
# as the old `1 | 2` is treated as "ill-defined" and the whole message is
# thrown away. See the long note in the module docstring.
TYPE_MASK = 0

# In GUIDED_NOGPS the FC interprets the thrust field as a climb-rate command:
# 0.5 = hold altitude, >0.5 = climb, <0.5 = descend. (This mapping can be
# changed by the GUID_OPTIONS "thrust as thrust" bit -- if you set that,
# revisit this value. Verify climb behaviour on the bench.)
#
# NOTE the ground gate in the module docstring: 0.5 = zero climb rate, and a
# zero climb rate while the FC still thinks it is landed means the whole
# command is discarded. Be airborne before you flip the switch.
HOVER_THRUST = 0.5

# --- DIAGNOSTICS ---
STATUS_HZ = 2                            # status line rate while spinning
STATUS_PERIOD = 1.0 / STATUS_HZ

# Send a HEARTBEAT from the Pi. ArduPilot does NOT need this to accept
# SET_ATTITUDE_TARGET (that is a PX4-offboard requirement, not an ArduPilot
# one), and it has a real downside: if FS_GCS_ENABLE is on, our heartbeats
# start satisfying the GCS failsafe, so this script dying mid-flight would then
# TRIGGER that failsafe. Left off deliberately.
SEND_HEARTBEAT = False

_LANDED_STATE = {
    0: 'UNDEFINED',
    1: 'ON_GROUND',
    2: 'IN_AIR',
    3: 'TAKEOFF',
    4: 'LANDING',
}


def level_quaternion_at_yaw(yaw_rads):
    """(w, x, y, z) for zero roll, zero pitch, heading `yaw_rads`."""
    half = yaw_rads * 0.5
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def send_spin_command(mav, target_yaw_rads, yaw_rate_rads):
    """Command a level attitude at `target_yaw_rads`, spinning at
    `yaw_rate_rads`, holding altitude.

    The quaternion carries the ABSOLUTE target heading, which the caller walks
    forward at the spin rate; body_yaw_rate is sent alongside it as the rate
    feed-forward so the controller leads the moving target instead of chasing
    it. Roll and pitch rates are 0 -- they must be present (not masked off) for
    ArduPilot to accept the message at all, and 0 is correct: we want the
    attitude quaternion, not a rate, holding the vehicle level.
    """
    mav.mav.set_attitude_target_send(
        0,                       # time_boot_ms (0 is fine)
        mav.target_system,
        mav.target_component,
        TYPE_MASK,
        level_quaternion_at_yaw(target_yaw_rads),
        0.0,                     # body_roll_rate  -> hold level
        0.0,                     # body_pitch_rate -> hold level
        yaw_rate_rads,           # body_yaw_rate   -> the spin (feed-forward)
        HOVER_THRUST,            # thrust -> climb rate (0.5 = hold alt)
    )


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


def request_message_interval(mav, msg_id, hz):
    """Ask the FC to stream msg_id at hz. Best effort; no ack is waited for."""
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def check_tx_path(mav, log):
    """Prove the Pi->FC direction works, and dump the params that decide what
    our thrust field means. Returns True if the FC answered us."""
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
                  f"climb rate. HOVER_THRUST={HOVER_THRUST} does NOT mean "
                  f"'hold altitude' on this FC -- it means 50% throttle.")
    else:
        log.event(f"    thrust field = climb rate; HOVER_THRUST="
                  f"{HOVER_THRUST} -> 0 m/s (note: a 0 climb rate while the FC "
                  f"still thinks it is landed means no spin on the ground).")
    return True


def main():
    log = FlightLog.open_log("SpinOnGuided")
    log.config(YAW_RATE_DEGS=YAW_RATE_DEGS, SEND_HZ=SEND_HZ,
               TYPE_MASK=TYPE_MASK, HOVER_THRUST=HOVER_THRUST,
               TARGET_MODE=TARGET_MODE, CONNECTION=CONNECTION_STRING,
               BAUD=BAUD_RATE)

    log.event(f"Connecting to {CONNECTION_STRING} @ {BAUD_RATE} ...")
    mav = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    mav.wait_heartbeat()
    log.event(f"Heartbeat: system {mav.target_system}, "
              f"component {mav.target_component}")

    if not check_tx_path(mav, log):
        log.event("Refusing to continue -- fix the link first, or the spin can "
                  "never work no matter what the mode says.")
        return

    # ATTITUDE gives us the measured heading (which the spin target is seeded
    # from) and the measured yaw RATE (so we can see whether the FC actually
    # acted on us); EXTENDED_SYS_STATE gives landed_state.
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)
    request_message_interval(
        mav, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)

    log.event(f"FC reports flight mode: '{mav.flightmode}'  "
              f"(spin triggers on '{TARGET_MODE}')")
    log.event("Flip channel-6 UP -> GUIDED_NOGPS -> spin starts.  "
              "DOWN/MIDDLE -> STABILIZE -> spin stops.")
    log.event("Vehicle must be ARMED (your 2-pos switch) for motors to "
              "actually turn.")
    log.event("Vehicle must also be AIRBORNE -- while the FC still believes "
              "it is landed it runs ground handling instead of the "
              "attitude controller.")
    log.event("Waiting to see a non-GUIDED_NOGPS mode once before the spin "
              "can trigger...")

    spinning = False
    # armed: latched False on every launch. We refuse to spin until we have
    # observed the vehicle OUT of GUIDED_NOGPS at least once since this process
    # started -- proof the switch is genuinely down and a later UP is a fresh,
    # deliberate action rather than a stale switch we happened to boot into.
    armed = False
    warned_unarmed = False
    warned_on_ground = False
    last_send = 0.0
    last_status = 0.0
    last_heartbeat = 0.0
    sent = 0

    # Live FC state, refreshed by the drain loop below.
    measured_yaw_rads = 0.0     # ATTITUDE.yawspeed (rate)
    measured_heading = 0.0      # ATTITUDE.yaw (absolute, rad)
    measured_roll = 0.0         # ATTITUDE.roll
    measured_pitch = 0.0        # ATTITUDE.pitch
    landed_state = 0            # EXTENDED_SYS_STATE.landed_state
    motors_armed = False        # HEARTBEAT.base_mode & SAFETY_ARMED
    alt_m = float('nan')        # VFR_HUD.alt

    # The attitude target we walk forward at YAW_RATE_RADS. Seeded from the
    # vehicle's actual heading when a spin starts, so we never command a step
    # change in heading at the moment the switch goes up.
    target_yaw = 0.0

    while True:
        # Drain whatever has arrived so mav.flightmode stays current. It is
        # updated as a side effect of parsing HEARTBEAT (>=1 Hz); we don't
        # block on a single type so the loop stays responsive. We also keep the
        # few fields the status line needs.
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == 'ATTITUDE':
                measured_yaw_rads = msg.yawspeed
                measured_heading = msg.yaw
                measured_roll = msg.roll
                measured_pitch = msg.pitch
            elif mtype == 'EXTENDED_SYS_STATE':
                landed_state = msg.landed_state
            elif mtype == 'HEARTBEAT':
                motors_armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif mtype == 'VFR_HUD':
                alt_m = msg.alt

        now = time.monotonic()

        if SEND_HEARTBEAT and (now - last_heartbeat) >= 1.0:
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_heartbeat = now

        in_target = (mav.flightmode == TARGET_MODE)

        if not in_target:
            # Switch is down/middle (or a failsafe pulled us out). Arm on the
            # first such observation, and stop any spin in progress.
            if not armed:
                armed = True
                log.event(f">>> Saw '{mav.flightmode}' (switch down/middle) "
                          f"-- armed. A later {TARGET_MODE} will now trigger "
                          f"the spin.")
            if spinning:
                spinning = False
                warned_on_ground = False
                log.event(f">>> Mode is {mav.flightmode} -- stopping "
                          f"(FC ignores our commands outside {TARGET_MODE}). "
                          f"Sent {sent} commands.")
        else:
            # In GUIDED_NOGPS.
            if armed:
                if not spinning:
                    spinning = True
                    sent = 0
                    # Start the target at where we actually point, so the first
                    # command is a no-op in heading and the spin ramps from
                    # there rather than snapping to an arbitrary bearing.
                    target_yaw = measured_heading
                    last_send = now
                    log.event(f">>> Mode is {TARGET_MODE} and armed -- "
                              f"starting slow spin ({YAW_RATE_DEGS} deg/s) "
                              f"from heading "
                              f"{math.degrees(measured_heading):.0f} deg")
                    if landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                        warned_on_ground = True
                        log.event(">>> !! FC reports ON_GROUND -- it runs "
                                  "ground idle instead of the attitude "
                                  "controller, so nothing will spin until "
                                  "you are airborne.")
            elif not warned_unarmed:
                # Booted with the switch already UP -- refuse until a real cycle.
                warned_unarmed = True
                log.event(f">>> In {TARGET_MODE} but NOT armed (started with "
                          f"the switch already up). Flip to STABILIZE, then "
                          f"back, to trigger.")

        if spinning and (now - last_send) >= SEND_PERIOD:
            # Walk the heading target forward by however long we actually
            # waited (not the nominal period), then wrap to keep it bounded.
            dt = now - last_send
            target_yaw = (target_yaw + YAW_RATE_RADS * dt) % (2.0 * math.pi)
            send_spin_command(mav, target_yaw, YAW_RATE_RADS)
            sent += 1
            last_send = now

            # One CSV row per COMMAND (not per loop iteration -- the loop runs
            # ~5x faster and each row is flushed to disk). This is the record
            # that shows whether the FC acted (meas tracking cmd) or discarded
            # us (meas flat while sent climbs).
            log.row(
                    mode=mav.flightmode, active=1, phase="spin",
                    target_yaw_deg=round(math.degrees(target_yaw) % 360, 2),
                    cmd_yaw_rate_degs=YAW_RATE_DEGS,
                    thrust=HOVER_THRUST, sent=sent,
                    heading_deg=round(math.degrees(measured_heading) % 360, 2),
                    meas_yaw_rate_degs=round(math.degrees(measured_yaw_rads), 2),
                    roll_meas_deg=round(math.degrees(measured_roll), 2),
                    pitch_meas_deg=round(math.degrees(measured_pitch), 2),
                    alt_m=round(alt_m, 2) if alt_m == alt_m else "",
                    landed=_LANDED_STATE.get(landed_state, landed_state),
                    fc_armed=int(motors_armed),
            )

        if spinning and (now - last_status) >= STATUS_PERIOD:
            measured_degs = math.degrees(measured_yaw_rads)
            log.event(f"[spin] sent={sent:<5d} cmd={YAW_RATE_DEGS:+.0f} deg/s  "
                      f"meas={measured_degs:+6.1f} deg/s  "
                      f"hdg={math.degrees(measured_heading) % 360:5.1f}->"
                      f"{math.degrees(target_yaw) % 360:5.1f}  "
                      f"landed={_LANDED_STATE.get(landed_state, landed_state)}  "
                      f"motors={'ARMED' if motors_armed else 'disarmed'}  "
                      f"alt={alt_m:.2f}m")
            if warned_on_ground and landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
                warned_on_ground = False
                log.event(">>> Now IN_AIR -- the FC will start acting on "
                          "the spin command from here.")
            last_status = now

        time.sleep(0.02)


if __name__ == "__main__":
    main()
