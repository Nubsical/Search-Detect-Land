"""
Bench DRY-RUN for LandOnAprilTag.py  --  SENDS NOTHING TO THE FLIGHT CONTROLLER.

This is the safe tool (props on or off, motors won't be commanded because no
MAVLink command is ever sent). It runs the REAL controller from
LandOnAprilTag.py against the live camera and shows you, frame by frame:

  * the detected tag and its pose  (x/y/z offset, yaw error)
  * the exact roll / pitch / yaw-rate / thrust the flight script WOULD send
  * a "WANT" arrow from image centre to the tag -- the direction the quad must
    translate to centre itself over the tag

Use it to:
  1. Confirm the tag is detected reliably and the x/y/z and yaw ranges look sane.
  2. VERIFY THE YAW SIGN: rotate the tag under the camera and watch yawRate --
     it must point the way that REDUCES yawErr toward 0. If it grows the error,
     press 'y' to flip INVERT_YAW.
  3. TUNE GAINS: move the tag to a realistic offset and check the commanded
     roll/pitch aren't slammed to the MAX_TILT clamp (too hot) or barely moving
     (too soft). Adjust KP/KD live with the keys below.
  4. Dump a config block (press 'c') to paste straight back into LandOnAprilTag.

NOTE: the translation SIGN of roll/pitch (does +roll move the quad toward the
tag or away?) depends on how the camera is bolted on and CANNOT be confirmed
without motion -- do that powered test with BenchTiltProbe.py. This dry-run
verifies everything else.

Keys:
  r  toggle INVERT_ROLL      p  toggle INVERT_PITCH
  s  toggle SWAP_XY          y  toggle INVERT_YAW
  [ ]  KP_TILT  -/+          - =  KD_TILT  -/+
  ; '  KP_YAW   -/+          , .  MAX_TILT_DEG  -/+
  c  print config block      q  quit (also prints config)
"""

import sys
import math
import time
from pathlib import Path

import cv2

# Import the REAL flight code so this tool uses the identical controller,
# helpers and tunables (single source of truth). Importing does NOT open the
# camera or MAVLink -- that all lives under LandOnAprilTag's __main__ guard.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import LandOnAprilTag as L


def print_config():
    print("\n" + "=" * 60)
    print("Paste these into LandOnAprilTag.py:")
    print(f"  INVERT_ROLL  = {L.INVERT_ROLL}")
    print(f"  INVERT_PITCH = {L.INVERT_PITCH}")
    print(f"  SWAP_XY      = {L.SWAP_XY}")
    print(f"  INVERT_YAW   = {L.INVERT_YAW}")
    print(f"  KP_TILT      = {L.KP_TILT:.2f}")
    print(f"  KD_TILT      = {L.KD_TILT:.2f}")
    print(f"  KP_YAW       = {L.KP_YAW:.2f}")
    print(f"  MAX_TILT_DEG = {L.MAX_TILT_DEG:.2f}")
    print("=" * 60 + "\n")


def main():
    camera_matrix, dist_coeffs, camera_params = L.load_calibration()
    map1, map2 = L.build_undistort_maps(camera_matrix, dist_coeffs)
    picam2 = L.open_camera()
    detector = L.build_detector()
    print("DRY-RUN -- NO COMMANDS ARE SENT TO THE FC. Press 'q' to quit.")
    print_config()

    prev = {"x": 0.0, "y": 0.0, "t": None}
    last = time.time()

    try:
        while True:
            frame, gray = L.grab_gray(picam2, map1, map2, want_color=True)
            results = detector.detect(gray, estimate_tag_pose=True,
                                      camera_params=camera_params, tag_size=L.TAG_SIZE)

            tag = None
            for r in results:
                if L.TARGET_TAG_ID is None or r.tag_id == L.TARGET_TAG_ID:
                    tag = r
                    break

            h, w = frame.shape[:2]
            cx_img, cy_img = w // 2, h // 2
            cv2.drawMarker(frame, (cx_img, cy_img), (200, 200, 200),
                           cv2.MARKER_CROSS, 20, 1)

            now = time.time()
            dt = now - last
            last = now

            if tag is not None:
                tx, ty, tz = tag.pose_t.flatten()
                _, _, tag_yaw = L.rotation_matrix_to_euler_angles(tag.pose_R)
                roll_c, pitch_c, yaw_rate, centred, aligned = L.compute_command(
                    tx, ty, tz, tag_yaw, prev, dt)

                # Draw the tag outline + a WANT arrow (centre -> tag centre).
                corners = tag.corners.astype(int)
                for i in range(4):
                    cv2.line(frame, tuple(corners[i]),
                             tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
                tc = tuple(tag.center.astype(int))
                cv2.circle(frame, tc, 5, (0, 0, 255), -1)
                cv2.arrowedLine(frame, (cx_img, cy_img), tc, (0, 255, 255), 2,
                                tipLength=0.2)

                sat = "  <-- CLAMPED" if (abs(roll_c) >= L.MAX_TILT_DEG - 1e-3 or
                                          abs(pitch_c) >= L.MAX_TILT_DEG - 1e-3) else ""
                print(f"x={tx:+.2f} y={ty:+.2f} z={tz:.2f}m yawErr={tag_yaw:+6.1f} "
                      f"-> roll={roll_c:+5.1f} pitch={pitch_c:+5.1f} "
                      f"yawRate={math.degrees(yaw_rate):+5.0f} "
                      f"centred={int(centred)} aligned={int(aligned)}{sat}")

                lines = [
                    f"ID{tag.tag_id} x={tx:+.2f} y={ty:+.2f} z={tz:.2f}m",
                    f"yawErr={tag_yaw:+.1f}  roll={roll_c:+.1f} pitch={pitch_c:+.1f}",
                    f"centred={int(centred)} aligned={int(aligned)}{sat}",
                ]
                for i, txt in enumerate(lines):
                    cv2.putText(frame, txt, (10, 24 + i * 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
            else:
                prev["t"] = None
                cv2.putText(frame, "no tag", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            flags = (f"R:{int(L.INVERT_ROLL)} P:{int(L.INVERT_PITCH)} "
                     f"SWAP:{int(L.SWAP_XY)} Y:{int(L.INVERT_YAW)} | "
                     f"KP{L.KP_TILT:.1f} KD{L.KD_TILT:.1f} "
                     f"KPy{L.KP_YAW:.1f} MAX{L.MAX_TILT_DEG:.0f}")
            cv2.putText(frame, flags, (10, h - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, "DRY-RUN: NO COMMANDS SENT", (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

            cv2.imshow("BenchDryRun (no commands sent)", frame)
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
            elif k == ord('['):
                L.KP_TILT = max(0.0, L.KP_TILT - 0.5)
            elif k == ord(']'):
                L.KP_TILT += 0.5
            elif k == ord('-'):
                L.KD_TILT = max(0.0, L.KD_TILT - 0.5)
            elif k == ord('='):
                L.KD_TILT += 0.5
            elif k == ord(';'):
                L.KP_YAW = max(0.0, L.KP_YAW - 0.1)
            elif k == ord("'"):
                L.KP_YAW += 0.1
            elif k == ord(','):
                L.MAX_TILT_DEG = max(1.0, L.MAX_TILT_DEG - 1.0)
            elif k == ord('.'):
                L.MAX_TILT_DEG += 1.0
            elif k == ord('c'):
                print_config()

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        print_config()


if __name__ == "__main__":
    main()
