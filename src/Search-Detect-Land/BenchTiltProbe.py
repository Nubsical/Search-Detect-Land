"""
Bench TILT-PROBE for LandOnAprilTag.py  --  POWERED test, confirms roll/pitch
translation SIGNS that a dry-run cannot (they depend on how the camera is
mounted vs. the airframe, and only reveal themselves when the quad moves).

  !!  This one COMMANDS the vehicle.  Read this before running.  !!

Safety model -- identical to SpinOnGuided.py / LandOnAprilTag.py:
  * Commands are only sent while the FC reports GUIDED_NOGPS (channel-6 switch
    UP). Flip the switch DOWN at any instant -> STABILIZE -> every command is
    dropped, you have manual control.
  * The `armed` latch refuses to act until it has seen a non-GUIDED_NOGPS mode
    once since this process started (a stale switch can't auto-resume).

On top of that, this probe is deliberately gentle and single-axis:
  * thrust is pinned to HOVER_THRUST -- it NEVER descends, only holds altitude.
  * only ONE axis is exercised at a time (PROBE_AXIS), the others are zeroed.
  * the commanded tilt is clamped to PROBE_MAX_TILT_DEG (small).

Recommended procedure (do it TETHERED or hand-held, low, over a soft surface):
  1. PROBE_AXIS = 'roll'. Put the tag off to one side of the camera. In
     GUIDED_NOGPS the quad should try to tilt so it would translate TOWARD the
     tag. If it tilts AWAY, press 'r' to flip INVERT_ROLL. Confirm both sides.
  2. PROBE_AXIS = 'pitch'. Tag ahead/behind. Same check; 'p' flips INVERT_PITCH.
  3. PROBE_AXIS = 'yaw'. Rotate the tag; the quad should yaw to face the tag
     front (reduce yawErr). 'y' flips INVERT_YAW.
  4. Press 'c' to dump the confirmed flags, paste them into LandOnAprilTag.py
     (and into its SWAP_XY too if you changed it here).

It reuses LandOnAprilTag's real controller (compute_command), so the signs you
confirm here are exactly the ones that will fly.

Keys:  r/p/s/y toggle INVERT_ROLL/INVERT_PITCH/SWAP_XY/INVERT_YAW
        1/2/3/4 set PROBE_AXIS = roll/pitch/yaw/full
        c print config    q quit
"""

import sys
import math
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
import LandOnAprilTag as L

from pymavlink import mavutil

# ---- Probe limits (kept small on purpose) ----
PROBE_MAX_TILT_DEG = 4.0     # hard cap on commanded roll/pitch during probing
PROBE_AXIS = 'roll'          # 'roll' | 'pitch' | 'yaw' | 'full'


def print_config():
    print("\n" + "=" * 60)
    print("Confirmed signs -- paste into LandOnAprilTag.py:")
    print(f"  INVERT_ROLL  = {L.INVERT_ROLL}")
    print(f"  INVERT_PITCH = {L.INVERT_PITCH}")
    print(f"  SWAP_XY      = {L.SWAP_XY}")
    print(f"  INVERT_YAW   = {L.INVERT_YAW}")
    print("=" * 60 + "\n")


def main():
    global PROBE_AXIS

    print(f"Connecting to {L.CONNECTION_STRING} @ {L.BAUD_RATE} ...")
    mav = mavutil.mavlink_connection(L.CONNECTION_STRING, baud=L.BAUD_RATE)
    mav.wait_heartbeat()
    print(f"Heartbeat: system {mav.target_system}, component {mav.target_component}")
    print(f"FC mode: '{mav.flightmode}' (probe acts only in '{L.TARGET_MODE}')")
    print("!! POWERED probe. Tether/hand-hold, low, soft surface. Props-off first "
          "to sanity-check, then a careful powered pass. !!")
    print("Waiting to see a non-GUIDED_NOGPS mode once before it can act...")

    camera_matrix, dist_coeffs, camera_params = L.load_calibration()
    map1, map2 = L.build_undistort_maps(camera_matrix, dist_coeffs)
    picam2 = L.open_camera()
    detector = L.build_detector()

    active = False
    armed = False
    warned_unarmed = False
    last_send = 0.0
    prev = {"x": 0.0, "y": 0.0, "t": None}

    try:
        while True:
            while mav.recv_match(blocking=False) is not None:
                pass
            in_target = (mav.flightmode == L.TARGET_MODE)

            if not in_target:
                if not armed:
                    armed = True
                    print(f">>> Saw '{mav.flightmode}' -- armed. A later "
                          f"{L.TARGET_MODE} will start probing.")
                if active:
                    active = False
                    prev["t"] = None
                    print(f">>> Mode {mav.flightmode} -- probe stopped.")
            else:
                if armed and not active:
                    active = True
                    print(f">>> {L.TARGET_MODE} + armed -- probing axis '{PROBE_AXIS}'.")
                elif not armed and not warned_unarmed:
                    warned_unarmed = True
                    print(f">>> In {L.TARGET_MODE} but NOT armed (booted switch-up). "
                          f"Flip DOWN then UP.")

            frame, gray = L.grab_gray(picam2, map1, map2, want_color=True)
            results = detector.detect(gray, estimate_tag_pose=True,
                                      camera_params=camera_params, tag_size=L.TAG_SIZE)
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
                dt = L.SEND_PERIOD if prev["t"] is None else (now - last_send)
                roll_c, pitch_c, yaw_rate, _, _ = L.compute_command(
                    tx, ty, tz, tag_yaw, prev, dt)

                # Single-axis, small: zero the axes we aren't probing and clamp.
                if PROBE_AXIS == 'roll':
                    pitch_c, yaw_rate = 0.0, 0.0
                elif PROBE_AXIS == 'pitch':
                    roll_c, yaw_rate = 0.0, 0.0
                elif PROBE_AXIS == 'yaw':
                    roll_c, pitch_c = 0.0, 0.0
                roll_c = L.clamp(roll_c, -PROBE_MAX_TILT_DEG, PROBE_MAX_TILT_DEG)
                pitch_c = L.clamp(pitch_c, -PROBE_MAX_TILT_DEG, PROBE_MAX_TILT_DEG)

                if do_send:
                    # Always hold altitude -- never descend during a probe.
                    L.send_attitude(mav, roll_c, pitch_c, yaw_rate, L.HOVER_THRUST)
                    last_send = now
                    print(f"[{PROBE_AXIS}] x={tx:+.2f} y={ty:+.2f} yawErr={tag_yaw:+.1f} "
                          f"-> roll={roll_c:+.1f} pitch={pitch_c:+.1f} "
                          f"yawRate={math.degrees(yaw_rate):+.0f} thr={L.HOVER_THRUST:.2f}")

                corners = tag.corners.astype(int)
                for i in range(4):
                    cv2.line(frame, tuple(corners[i]),
                             tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
                tc = tuple(tag.center.astype(int))
                cv2.arrowedLine(frame, (frame.shape[1] // 2, frame.shape[0] // 2),
                                tc, (0, 255, 255), 2, tipLength=0.2)
            else:
                prev["t"] = None
                if do_send:
                    # No tag: command level hover (no tilt), still hold altitude.
                    L.send_attitude(mav, 0.0, 0.0, 0.0, L.HOVER_THRUST)
                    last_send = now

            banner = (f"axis={PROBE_AXIS}  active={int(active)}  "
                      f"R:{int(L.INVERT_ROLL)} P:{int(L.INVERT_PITCH)} "
                      f"SWAP:{int(L.SWAP_XY)} Y:{int(L.INVERT_YAW)}")
            cv2.putText(frame, banner, (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.putText(frame, "POWERED PROBE -- switch DOWN = manual",
                        (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 2)

            cv2.imshow("BenchTiltProbe (POWERED)", frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            elif k == ord('r'):
                L.INVERT_ROLL = not L.INVERT_ROLL
            elif k == ord('p'):
                L.INVERT_PITCH = not L.INVERT_PITCH
            elif k == ord('s'):
                L.SWAP_XY = not L.SWAP_XY
            elif k == ord('y'):
                L.INVERT_YAW = not L.INVERT_YAW
            elif k == ord('1'):
                PROBE_AXIS = 'roll'
            elif k == ord('2'):
                PROBE_AXIS = 'pitch'
            elif k == ord('3'):
                PROBE_AXIS = 'yaw'
            elif k == ord('4'):
                PROBE_AXIS = 'full'
            elif k == ord('c'):
                print_config()

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        print_config()


if __name__ == "__main__":
    main()
