from pymavlink import mavutil
import math
import time

# --- CONFIGURATION ---
CONNECTION_STRING = '/dev/ttyAMA10'
BAUD_RATE = 115200

BROADCAST_NAME = b'SPIN'    # must match BROADCAST_NAME in spin_trigger.lua

YAW_RATE_DEGS = 15                       # slow spin rate
YAW_RATE_RADS = math.radians(YAW_RATE_DEGS)

# type_mask bits: ignore body roll rate (1) + ignore body pitch rate (2)
# + ignore attitude/use-rate (32). Leaves yaw rate AND thrust/climb-rate
# active, so climb_rate=0 holds altitude while yawing.
# (SET_ATTITUDE_TARGET is the only message GUIDED_NOGPS accepts — this
# replaces the old RC_CHANNELS_OVERRIDE approach, which doesn't apply here.)
TYPE_MASK_YAW_RATE_ONLY = 1 | 2 | 32
IDENTITY_QUATERNION = [1, 0, 0, 0]  # required field, ignored per type_mask


def send_spin_command(mav, yaw_rate_rads):
    """
    Command a yaw rate via SET_ATTITUDE_TARGET, leaving roll/pitch/attitude
    alone and holding a 0 climb rate to maintain altitude. This is the
    correct control path for GUIDED_NOGPS — it does not use RC override.

    NOTE: verify TYPE_MASK bit meanings and field order against the
    mavlink message definitions / docs.lua for your firmware before
    flying: https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
    """
    mav.mav.set_attitude_target_send(
        0,                          # time_boot_ms, 0 is acceptable
        mav.target_system,
        mav.target_component,
        TYPE_MASK_YAW_RATE_ONLY,
        IDENTITY_QUATERNION,
        0,                          # body_roll_rate (ignored)
        0,                          # body_pitch_rate (ignored)
        yaw_rate_rads,              # body_yaw_rate
        0.0                         # thrust field == climb rate (0 = hold alt)
                                    # unless GUID_OPTIONS bit 3 is set
    )


EXPECTED_MODE = 'GUIDED_NOGPS'  # pymavlink's name for Copter mode 20.
                                 # Verify this string against mav.flightmode
                                 # on your own setup before relying on it.


def in_expected_mode(mav):
    """
    True only if the flight controller's actual current mode (from the
    HEARTBEAT stream) matches what we expect to be commanding in.
    We never trust our own program_active flag alone — if the FC has
    dropped out of GUIDED_NOGPS for any reason (the switch, a failsafe,
    anything), our commands are being ignored by the vehicle regardless,
    and we should know that rather than spam them blind.
    """
    return mav.flightmode == EXPECTED_MODE


def main():
    mav = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    mav.wait_heartbeat()
    print("Heartbeat detected")

    program_active = False
    last_message_time = None
    STALE_TIMEOUT = 1.0  # our own bookkeeping only — GUID_TIMEOUT (set on
                          # the flight controller, default 3s) is the real
                          # backstop and works even if this script dies

    # ARM/DISARM GATE — this is what protects against a crash-and-restart.
    # If this process dies and comes back (e.g. a systemd auto-restart)
    # while the switch is still physically ON, we must NOT trust that ON
    # reading — there was no fresh human action behind it. We only allow
    # an ON reading to start the spin after we've explicitly observed an
    # OFF reading first, from this process's own clean start. Starts
    # False on every launch, on purpose.
    armed = False

    print(f"Listening for NAMED_VALUE_FLOAT '{BROADCAST_NAME.decode()}' from flight controller...")
    print("Waiting to see the switch OFF at least once before it's allowed to trigger anything...")

    while True:
        msg = mav.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=1)
        # NOTE: pymavlink updates mav.flightmode as a side effect of
        # parsing ANY incoming message (including HEARTBEATs it passes
        # over while filtering for NAMED_VALUE_FLOAT above), so this is
        # already current — no extra recv_match call needed.
        now = time.time()

        if msg is not None and msg.name == BROADCAST_NAME:
            last_message_time = now
            switch_on = msg.value > 0.5

            if not switch_on:
                if not armed:
                    armed = True
                    print(">>> Switch confirmed OFF — armed. A future ON will now be trusted.")
                if program_active:
                    program_active = False
                    print(">>> FC reports switch OFF — stopping (mode change already killed the spin)")
            elif switch_on and armed and not program_active:
                program_active = True
                print(">>> FC reports switch ON  — starting slow spin")
            elif switch_on and not armed:
                # Switch was already ON when this process started (or
                # restarted) — refuse to act on it. Operator must cycle
                # the switch OFF then ON to intentionally arm and trigger.
                print(">>> Switch is ON but not armed yet (stale/startup state) — ignoring. Flip it OFF then ON to start.")

        # Bookkeeping only. If this never fires because the Pi died,
        # GUID_TIMEOUT on the flight controller levels the vehicle anyway.
        if program_active and last_message_time is not None:
            if now - last_message_time > STALE_TIMEOUT:
                program_active = False
                print(">>> No update from FC — stopping as a precaution")

        if program_active:
            if in_expected_mode(mav):
                send_spin_command(mav, YAW_RATE_RADS)
            else:
                # Switch says "on" but the vehicle isn't actually in
                # GUIDED_NOGPS — our commands would do nothing right now.
                # Don't pretend otherwise: drop out of program_active and
                # make the operator re-toggle once the real problem
                # (failsafe, mode-change failure, whatever) is resolved.
                program_active = False
                print(f">>> Vehicle is in '{mav.flightmode}', not '{EXPECTED_MODE}' — "
                      f"commands would be ignored. Stopping and requiring a fresh toggle.")

        time.sleep(0.1)  # well within GUID_TIMEOUT's default 3s window


if __name__ == "__main__":
    main()
