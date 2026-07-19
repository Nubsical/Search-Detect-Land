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
"""

from pymavlink import mavutil
import math
import time

# --- CONNECTION ---
CONNECTION_STRING = '/dev/ttyAMA0'
BAUD_RATE = 115200

# --- BEHAVIOUR ---
# pymavlink's name for Copter mode 20. Confirm this exact string against
# mav.flightmode on your own FC before relying on it (printed at startup).
TARGET_MODE = 'GUIDED_NOGPS'

YAW_RATE_DEGS = 15                       # slow spin
YAW_RATE_RADS = math.radians(YAW_RATE_DEGS)

SEND_HZ = 10                             # comfortably inside GUID_TIMEOUT
SEND_PERIOD = 1.0 / SEND_HZ              # (default 3 s) so the spin never
                                         # stalls between commands

# type_mask for SET_ATTITUDE_TARGET: ignore the body ROLL and PITCH RATES so
# the quaternion below sets a level roll/pitch attitude, while body_yaw_rate
# drives the spin and thrust holds altitude. We deliberately do NOT ignore the
# attitude or the yaw rate or the thrust.
#   BODY_ROLL_RATE_IGNORE  = 1
#   BODY_PITCH_RATE_IGNORE = 2
TYPE_MASK = 1 | 2

LEVEL_QUATERNION = [1.0, 0.0, 0.0, 0.0]  # (w, x, y, z) -> level attitude

# In GUIDED_NOGPS the FC interprets the thrust field as a climb-rate command:
# 0.5 = hold altitude, >0.5 = climb, <0.5 = descend. (This mapping can be
# changed by the GUID_OPTIONS "thrust as thrust" bit -- if you set that,
# revisit this value. Verify climb behaviour on the bench.)
HOVER_THRUST = 0.5


def send_spin_command(mav, yaw_rate_rads):
    """Command a slow yaw rate at a level attitude, holding altitude."""
    mav.mav.set_attitude_target_send(
        0,                       # time_boot_ms (0 is fine)
        mav.target_system,
        mav.target_component,
        TYPE_MASK,
        LEVEL_QUATERNION,
        0.0,                     # body_roll_rate  (ignored via type_mask)
        0.0,                     # body_pitch_rate (ignored via type_mask)
        yaw_rate_rads,           # body_yaw_rate   (the spin)
        HOVER_THRUST,            # thrust -> climb rate (0.5 = hold alt)
    )


def main():
    print(f"Connecting to {CONNECTION_STRING} @ {BAUD_RATE} ...")
    mav = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    mav.wait_heartbeat()
    print(f"Heartbeat: system {mav.target_system}, component {mav.target_component}")
    print(f"FC reports flight mode: '{mav.flightmode}'  "
          f"(spin triggers on '{TARGET_MODE}')")
    print("Flip channel-6 UP -> GUIDED_NOGPS -> spin starts.  "
          "DOWN/MIDDLE -> STABILIZE -> spin stops.")
    print("Vehicle must be ARMED (your 2-pos switch) for motors to actually turn.")
    print("Waiting to see a non-GUIDED_NOGPS mode once before the spin can trigger...")

    spinning = False
    # armed: latched False on every launch. We refuse to spin until we have
    # observed the vehicle OUT of GUIDED_NOGPS at least once since this process
    # started -- proof the switch is genuinely down and a later UP is a fresh,
    # deliberate action rather than a stale switch we happened to boot into.
    armed = False
    warned_unarmed = False
    last_send = 0.0

    while True:
        # Drain whatever has arrived so mav.flightmode stays current. It is
        # updated as a side effect of parsing HEARTBEAT (>=1 Hz); we don't
        # block on a single type so the loop stays responsive.
        while mav.recv_match(blocking=False) is not None:
            pass

        in_target = (mav.flightmode == TARGET_MODE)

        if not in_target:
            # Switch is down/middle (or a failsafe pulled us out). Arm on the
            # first such observation, and stop any spin in progress.
            if not armed:
                armed = True
                print(f">>> Saw '{mav.flightmode}' (switch down/middle) -- armed. "
                      f"A later {TARGET_MODE} will now trigger the spin.")
            if spinning:
                spinning = False
                print(f">>> Mode is {mav.flightmode} -- stopping "
                      f"(FC ignores our commands outside {TARGET_MODE})")
        else:
            # In GUIDED_NOGPS.
            if armed:
                if not spinning:
                    spinning = True
                    print(f">>> Mode is {TARGET_MODE} and armed -- starting slow "
                          f"spin ({YAW_RATE_DEGS} deg/s)")
            elif not warned_unarmed:
                # Booted with the switch already UP -- refuse until a real cycle.
                warned_unarmed = True
                print(f">>> In {TARGET_MODE} but NOT armed (started with the switch "
                      f"already up). Flip to STABILIZE, then back, to trigger.")

        now = time.time()
        if spinning and (now - last_send) >= SEND_PERIOD:
            send_spin_command(mav, YAW_RATE_RADS)
            last_send = now

        time.sleep(0.02)


if __name__ == "__main__":
    main()
