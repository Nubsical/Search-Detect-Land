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
import FlightLog

from pymavlink import mavutil

# ---- Probe limits (kept small on purpose) ----
PROBE_MAX_TILT_DEG = 4.0     # hard cap on commanded roll/pitch during probing
PROBE_AXIS = 'roll'          # 'roll' | 'pitch' | 'yaw' | 'full'


def print_config(log=None):
    """Dump the confirmed sign flags for copy-paste -- and into the log, since
    these flags are the entire product of a probe run."""
    emit = log.event if log is not None else print
    emit("=" * 60)
    emit("Confirmed signs -- paste into LandOnAprilTag.py:")
    emit(f"  INVERT_ROLL  = {L.INVERT_ROLL}")
    emit(f"  INVERT_PITCH = {L.INVERT_PITCH}")
    emit(f"  SWAP_XY      = {L.SWAP_XY}")
    emit(f"  INVERT_YAW   = {L.INVERT_YAW}")
    emit("=" * 60)


def main():
    global PROBE_AXIS

    log = FlightLog.open_log("BenchTiltProbe")
    log.config(PROBE_AXIS=PROBE_AXIS, PROBE_MAX_TILT_DEG=PROBE_MAX_TILT_DEG,
               TYPE_MASK=L.TYPE_MASK, HOVER_THRUST=L.HOVER_THRUST,
               KP_TILT=L.KP_TILT, KD_TILT=L.KD_TILT, KP_YAW=L.KP_YAW,
               INVERT_ROLL=L.INVERT_ROLL, INVERT_PITCH=L.INVERT_PITCH,
               SWAP_XY=L.SWAP_XY, INVERT_YAW=L.INVERT_YAW)

    log.event(f"Connecting to {L.CONNECTION_STRING} @ {L.BAUD_RATE} ...")
    mav = mavutil.mavlink_connection(L.CONNECTION_STRING, baud=L.BAUD_RATE)
    mav.wait_heartbeat()
    log.event(f"Heartbeat: system {mav.target_system}, "
              f"component {mav.target_component}")
    log.event(f"FC mode: '{mav.flightmode}' (probe acts only in "
              f"'{L.TARGET_MODE}')")
    log.event("!! POWERED probe. Tether/hand-hold, low, soft surface. "
              "Props-off first to sanity-check, then a careful powered pass.")
    log.event("Waiting to see a non-GUIDED_NOGPS mode once before it can "
              "act...")

    camera_matrix, dist_coeffs, camera_params = L.load_calibration()
    map1, map2 = L.build_undistort_maps(camera_matrix, dist_coeffs)
    picam2 = L.open_camera()
    detector = L.build_detector()

    # send_attitude needs an ABSOLUTE heading target, so we track ATTITUDE.yaw.
    L.request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                               L.SEND_HZ)

    active = False
    armed = False
    warned_unarmed = False
    last_send = 0.0
    prev = {"x": 0.0, "y": 0.0, "t": None}
    heading_rads = None
    warned_no_heading = False
    # Measured tilt, logged beside the commanded tilt -- for a sign probe this
    # pairing IS the result: it shows the vehicle responding, and which way.
    meas_roll = meas_pitch = meas_yawspeed = 0.0

    try:
        while True:
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == 'ATTITUDE':
                    heading_rads = msg.yaw
                    meas_roll = msg.roll
                    meas_pitch = msg.pitch
                    meas_yawspeed = msg.yawspeed
            in_target = (mav.flightmode == L.TARGET_MODE)

            if not in_target:
                if not armed:
                    armed = True
                    log.event(f">>> Saw '{mav.flightmode}' -- armed. A later "
                              f"{L.TARGET_MODE} will start probing.")
                if active:
                    active = False
                    prev["t"] = None
                    log.event(f">>> Mode {mav.flightmode} -- probe stopped.")
            else:
                if armed and not active:
                    active = True
                    log.event(f">>> {L.TARGET_MODE} + armed -- probing axis "
                              f"'{PROBE_AXIS}'.")
                elif not armed and not warned_unarmed:
                    warned_unarmed = True
                    log.event(f">>> In {L.TARGET_MODE} but NOT armed (booted "
                              f"switch-up). Flip DOWN then UP.")

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

            # Without a heading we cannot build an absolute yaw target, and
            # guessing would command a swing to north. Hold off.
            if do_send and heading_rads is None:
                do_send = False
                if not warned_no_heading:
                    warned_no_heading = True
                    log.event("!! No ATTITUDE from the FC yet -- not "
                              "commanding (cannot build a heading target).")

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
                    # Heading target = current heading + this cycle's turn (0
                    # unless we're actually probing the yaw axis).
                    target_yaw_deg = math.degrees(heading_rads + yaw_rate * dt)
                    L.send_attitude(mav, roll_c, pitch_c, target_yaw_deg,
                                    yaw_rate, L.HOVER_THRUST)
                    last_send = now
                    log.event(f"[{PROBE_AXIS}] x={tx:+.2f} y={ty:+.2f} "
                              f"yawErr={tag_yaw:+.1f} -> roll={roll_c:+.1f} "
                              f"pitch={pitch_c:+.1f} "
                              f"yawRate={math.degrees(yaw_rate):+.0f} "
                              f"thr={L.HOVER_THRUST:.2f}")
                    log.row(
                        mode=mav.flightmode, active=1, phase=PROBE_AXIS,
                        tag=1, tag_id=tag.tag_id,
                        x=round(tx, 4), y=round(ty, 4), z=round(tz, 4),
                        yaw_err_deg=round(tag_yaw, 2),
                        roll_cmd_deg=round(roll_c, 2),
                        pitch_cmd_deg=round(pitch_c, 2),
                        target_yaw_deg=round(target_yaw_deg % 360, 2),
                        cmd_yaw_rate_degs=round(math.degrees(yaw_rate), 2),
                        thrust=L.HOVER_THRUST,
                        heading_deg=round(math.degrees(heading_rads) % 360, 2),
                        meas_yaw_rate_degs=round(math.degrees(meas_yawspeed), 2),
                        roll_meas_deg=round(math.degrees(meas_roll), 2),
                        pitch_meas_deg=round(math.degrees(meas_pitch), 2),
                    )

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
                    # No tag: command level hover (no tilt), still hold altitude
                    # and HOLD the current heading (0 would mean "point north").
                    L.send_attitude(mav, 0.0, 0.0, math.degrees(heading_rads),
                                    0.0, L.HOVER_THRUST)
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
                print_config(log)

    except KeyboardInterrupt:
        log.event(">>> Ctrl-C -- stopping.")
    except Exception:
        log.exception("in main loop")
        raise
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        # Dump the flags BEFORE closing, so the run's conclusion is in the file
        # and not just on a console that scrolls away.
        print_config(log)
        log.event("Logs: %s | %s", log.log_path.name, log.csv_path.name)
        log.close()


if __name__ == "__main__":
    main()
