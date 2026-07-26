"""
Centre-on-AprilTag test  --  HOLDS ALTITUDE, NEVER DESCENDS.

Companion-computer (Raspberry Pi) script for the Search-Detect-Land quad.

What it does
------------
This is LandOnAprilTag.py with the descent and the LAND hand-off removed. While
the flight controller is in GUIDED_NOGPS (channel-6 switch UP), this script:

  1. detects the downward-facing AprilTag (same camera/detector pipeline), and
  2. drives the quad to sit directly ABOVE the tag while holding altitude -- a
     PD controller tilts roll/pitch to zero the tag's horizontal offset, and a
     yaw rate lines the quad's front up with the FRONT of the tag.

That is ALL it does. Thrust is pinned to HOVER_THRUST for the entire run, so the
quad only ever translates and yaws to park itself over the tag -- it does not
descend, and it never hands off to LAND. Use this to prove the centring/align
behaviour in the air before you trust the full descend-and-land script.

Single source of truth
-----------------------
Everything that has to match the real landing run -- the camera setup, the
detector, the PD controller, and above all the roll/pitch/yaw SIGN CONVENTIONS
(INVERT_ROLL / INVERT_PITCH / SWAP_XY / INVERT_YAW) and the gains -- is imported
from LandOnAprilTag, exactly like BenchDryRun.py / BenchTiltProbe.py. Confirm
those signs on the bench there first; whatever you set in LandOnAprilTag is what
flies here.

Same safety model as SpinOnGuided.py / LandOnAprilTag.py
--------------------------------------------------------
The RC 3-position switch on channel 6 selects the FC flight mode itself
(FLTMODE_CH=6; UP -> GUIDED_NOGPS, DOWN/MIDDLE -> STABILIZE). GUIDED_NOGPS is the
only Copter mode that honours the offboard SET_ATTITUDE_TARGET messages this
script sends. Flip the switch DOWN at ANY time and the FC returns to STABILIZE,
which drops every command this script sends -- you get manual control instantly.
The SWITCH (via the flight mode), not this script, is the real safety gate.

There is ONE guard on top of that: after this process (re)starts it will NOT act
until it has seen the vehicle OUT of GUIDED_NOGPS at least once (the `armed`
latch), so an auto-restart while the switch is already UP can't silently resume
control -- it takes a deliberate flick DOWN then UP.

  !!  BENCH-TEST FIRST, PROPS OFF.  !!
  Confirm the sign conventions with BenchDryRun.py / BenchTiltProbe.py before
  running this powered. In GUIDED_NOGPS the FC controls throttle via the thrust
  field -- here that is fixed at HOVER_THRUST (hold altitude).
"""

import sys
import math
import time
from pathlib import Path

import cv2

# Import the REAL flight code so this script uses the identical camera, detector,
# controller and -- critically -- the same sign conventions and gains that the
# landing run uses. Importing does NOT open the camera or MAVLink (that lives
# under LandOnAprilTag's __main__ guard).
sys.path.insert(0, str(Path(__file__).resolve().parent))
import LandOnAprilTag as L

from pymavlink import mavutil

# Preview window title / toggle (mirrors LandOnAprilTag.SHOW_WINDOW).
SHOW_WINDOW = L.SHOW_WINDOW


def main():
    # ---- MAVLink ----
    print(f"Connecting to {L.CONNECTION_STRING} @ {L.BAUD_RATE} ...")
    mav = mavutil.mavlink_connection(L.CONNECTION_STRING, baud=L.BAUD_RATE)
    mav.wait_heartbeat()
    print(f"Heartbeat: system {mav.target_system}, component {mav.target_component}")
    print(f"FC reports flight mode: '{mav.flightmode}' (control triggers on '{L.TARGET_MODE}')")
    print("Flip channel-6 UP -> GUIDED_NOGPS -> centre-over-tag starts (HOLDS ALTITUDE).  "
          "DOWN/MIDDLE -> STABILIZE -> hands back to you.")
    print("This script NEVER descends -- it only centres + yaw-aligns over the tag.")
    print("Waiting to see a non-GUIDED_NOGPS mode once before control can trigger...")

    # ---- Camera / calibration / detector (shared helpers) ----
    camera_matrix, dist_coeffs, camera_params = L.load_calibration()
    map1, map2 = L.build_undistort_maps(camera_matrix, dist_coeffs)
    picam2 = L.open_camera()
    detector = L.build_detector()

    # ---- State (same latch semantics as SpinOnGuided / LandOnAprilTag) ----
    active = False          # are we currently commanding the vehicle?
    armed = False           # seen a non-GUIDED_NOGPS mode since startup?
    warned_unarmed = False
    last_send = 0.0
    last_seen = 0.0         # time we last had a valid tag
    prev = {"x": 0.0, "y": 0.0, "t": None}

    try:
        while True:
            # Drain MAVLink so mav.flightmode stays current (side effect of
            # parsing HEARTBEAT). Don't block -- keep the vision loop responsive.
            while mav.recv_match(blocking=False) is not None:
                pass

            in_target = (mav.flightmode == L.TARGET_MODE)

            if not in_target:
                if not armed:
                    armed = True
                    print(f">>> Saw '{mav.flightmode}' (switch down/middle) -- armed. "
                          f"A later {L.TARGET_MODE} will now trigger centring.")
                if active:
                    active = False
                    prev["t"] = None
                    print(f">>> Mode is {mav.flightmode} -- stopping "
                          f"(FC ignores our commands outside {L.TARGET_MODE})")
            else:
                if armed and not active:
                    active = True
                    print(f">>> Mode is {L.TARGET_MODE} and armed -- searching for tag, "
                          f"then centre / align (holding altitude).")
                elif not armed and not warned_unarmed:
                    warned_unarmed = True
                    print(f">>> In {L.TARGET_MODE} but NOT armed (started with the switch "
                          f"already up). Flip to STABILIZE, then back, to trigger.")

            # ---- Vision: grab a frame and look for the tag ----
            frame, gray = L.grab_gray(picam2, map1, map2, want_color=SHOW_WINDOW)
            results = detector.detect(gray, estimate_tag_pose=True,
                                      camera_params=camera_params, tag_size=L.TAG_SIZE)

            # Pick the target tag (specific id if configured, else the first).
            tag = None
            for r in results:
                if L.TARGET_TAG_ID is None or r.tag_id == L.TARGET_TAG_ID:
                    tag = r
                    break

            now = time.time()
            do_send = active and (now - last_send) >= L.SEND_PERIOD

            if tag is not None:
                tx, ty, tz = tag.pose_t.flatten()
                _, _, tag_yaw = L.rotation_matrix_to_euler_angles(tag.pose_R)
                last_seen = now

                if do_send:
                    dt = L.SEND_PERIOD if prev["t"] is None else (now - last_send)
                    roll_c, pitch_c, yaw_rate, centred, aligned = L.compute_command(
                        tx, ty, tz, tag_yaw, prev, dt)

                    # Always hold altitude -- this script never descends.
                    L.send_attitude(mav, roll_c, pitch_c, yaw_rate, L.HOVER_THRUST)
                    last_send = now

                    status = "PARKED" if (centred and aligned) else "CENTRE"
                    print(f"[{status}] x={tx:+.2f} y={ty:+.2f} z={tz:.2f}m "
                          f"yawErr={tag_yaw:+.1f} -> roll={roll_c:+.1f} "
                          f"pitch={pitch_c:+.1f} yawRate={math.degrees(yaw_rate):+.0f} "
                          f"thr={L.HOVER_THRUST:.2f} centred={int(centred)} "
                          f"aligned={int(aligned)}")

                if SHOW_WINDOW:
                    corners = tag.corners.astype(int)
                    for i in range(4):
                        cv2.line(frame, tuple(corners[i]),
                                 tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
                    c = tuple(tag.center.astype(int))
                    cv2.circle(frame, c, 5, (0, 0, 255), -1)
                    # Arrow from image centre to tag: the way the quad must translate.
                    cv2.arrowedLine(frame, (frame.shape[1] // 2, frame.shape[0] // 2),
                                    c, (0, 255, 255), 2, tipLength=0.2)
                    cv2.putText(frame, f"ID{tag.tag_id} x={tx:+.2f} y={ty:+.2f} z={tz:.2f}",
                                (c[0] + 10, c[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 0, 0), 2)

            else:
                # No tag this frame -- hold level and hold altitude (never move blind).
                if do_send:
                    L.send_attitude(mav, 0.0, 0.0, 0.0, L.HOVER_THRUST)
                    prev["t"] = None
                    last_send = now
                    print(f"[HOLD] tag lost {now - last_seen:.1f}s -- hovering level")

            if SHOW_WINDOW:
                cv2.putText(frame, "CENTRE-ONLY: HOLDS ALTITUDE, NEVER DESCENDS",
                            (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 165, 255), 2)
                cv2.imshow("Centre-on-AprilTag (holds altitude)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            time.sleep(0.005)

    finally:
        picam2.stop()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
