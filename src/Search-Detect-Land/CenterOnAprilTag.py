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

Watching it (running over SSH)
------------------------------
The annotated camera view -- tag outline, centre dot, the arrow showing which
way the quad has to move, the pose numbers -- goes to TWO places:

  * a live cv2 window, when there is a display. Over a plain `ssh pi@...` there
    isn't one, so the window switches itself off rather than letting cv2.imshow
    raise into the control loop. (`ssh -X` gives you a window, but it tunnels
    every frame and will slow the loop down.)
  * a recording: logs/CenterOnAprilTag_<stamp>.mp4, next to that run's .log and
    .csv. This is the one to use for a real flight -- afterwards you can watch
    exactly what the detector had to work with, and the `t=` stamped on each
    frame is the same clock as the CSV's `t` column, so video and numbers line
    up. `scp` it off the Pi when you land.

    $ python3 CenterOnAprilTag.py                 # record, no window over SSH
    $ python3 CenterOnAprilTag.py --no-video      # logs only
    $ python3 CenterOnAprilTag.py --video-width 640 --video-fps 5   # smaller

  !!  BENCH-TEST FIRST, PROPS OFF.  !!
  Confirm the sign conventions with BenchDryRun.py / BenchTiltProbe.py before
  running this powered. In GUIDED_NOGPS the FC controls throttle via the thrust
  field -- here that is fixed at HOVER_THRUST (hold altitude).
"""

import argparse
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
import FlightLog
import FlightVideo

from pymavlink import mavutil


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Centre over an AprilTag while holding altitude "
                    "(never descends, never lands).")
    return L.view_args(p).parse_args(argv)


def main(args=None):
    args = args if args is not None else parse_args()
    log = FlightLog.open_log("CenterOnAprilTag")

    # How this run gets watched. Over SSH there is no display, so the preview
    # window switches itself off and the annotated view goes to the recording
    # (logs/CenterOnAprilTag_<stamp>.mp4) instead -- same overlay, watchable
    # afterwards. `draw` is what decides whether the overlay is worth building.
    show_window, video = L.open_view(args, log)
    draw = show_window or (video is not None)

    log.config(TYPE_MASK=L.TYPE_MASK, HOVER_THRUST=L.HOVER_THRUST,
               KP_TILT=L.KP_TILT, KD_TILT=L.KD_TILT,
               MAX_TILT_DEG=L.MAX_TILT_DEG, KP_YAW=L.KP_YAW,
               MAX_YAW_RATE_DEGS=L.MAX_YAW_RATE_DEGS,
               INVERT_ROLL=L.INVERT_ROLL, INVERT_PITCH=L.INVERT_PITCH,
               SWAP_XY=L.SWAP_XY, INVERT_YAW=L.INVERT_YAW,
               CENTRE_TOL_M=L.CENTRE_TOL_M, YAW_TOL_DEG=L.YAW_TOL_DEG,
               SHOW_WINDOW=show_window, RECORD_VIDEO=(video is not None))

    # ---- MAVLink ----
    log.event(f"Connecting to {L.CONNECTION_STRING} @ {L.BAUD_RATE} ...")
    mav = mavutil.mavlink_connection(L.CONNECTION_STRING, baud=L.BAUD_RATE)
    mav.wait_heartbeat()
    log.event(f"Heartbeat: system {mav.target_system}, "
              f"component {mav.target_component}")
    log.event(f"FC reports flight mode: '{mav.flightmode}' "
              f"(control triggers on '{L.TARGET_MODE}')")
    log.event("Flip channel-6 UP -> GUIDED_NOGPS -> centre-over-tag starts "
              "(HOLDS ALTITUDE).  DOWN/MIDDLE -> STABILIZE -> hands back.")
    log.event("This script NEVER descends -- it only centres + yaw-aligns.")
    log.event("Waiting to see a non-GUIDED_NOGPS mode once before control can "
              "trigger...")

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
    heading_rads = None     # ATTITUDE.yaw -- send_attitude needs it
    warned_no_heading = False
    meas_roll = meas_pitch = meas_yawspeed = 0.0
    # Last phase / command, kept for the overlay so the recording shows what we
    # were doing at each instant, not just what the camera saw.
    hud_phase = "IDLE"
    hud_cmd = ""

    # send_attitude builds an ABSOLUTE heading target, so we need ATTITUDE.
    L.request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                               L.SEND_HZ)

    try:
        while True:
            # Drain MAVLink so mav.flightmode stays current (side effect of
            # parsing HEARTBEAT) and keep the latest heading. Don't block --
            # keep the vision loop responsive.
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
                    log.event(f">>> Saw '{mav.flightmode}' (switch down/"
                              f"middle) -- armed. A later {L.TARGET_MODE} "
                              f"will now trigger centring.")
                if active:
                    active = False
                    prev["t"] = None
                    log.event(f">>> Mode is {mav.flightmode} -- stopping "
                              f"(FC ignores our commands outside "
                              f"{L.TARGET_MODE})")
            else:
                if armed and not active:
                    active = True
                    log.event(f">>> Mode is {L.TARGET_MODE} and armed -- "
                              f"searching for tag, then centre / align "
                              f"(holding altitude).")
                elif not armed and not warned_unarmed:
                    warned_unarmed = True
                    log.event(f">>> In {L.TARGET_MODE} but NOT armed (started "
                              f"with the switch already up). Flip to "
                              f"STABILIZE, then back, to trigger.")

            # ---- Vision: grab a frame and look for the tag ----
            frame, gray = L.grab_gray(picam2, map1, map2, want_color=draw)
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
                last_seen = now

                if do_send:
                    dt = L.SEND_PERIOD if prev["t"] is None else (now - last_send)
                    roll_c, pitch_c, yaw_rate, centred, aligned = L.compute_command(
                        tx, ty, tz, tag_yaw, prev, dt)

                    # Always hold altitude -- this script never descends.
                    # Heading target = current heading + this cycle's turn.
                    target_yaw_deg = math.degrees(heading_rads + yaw_rate * dt)
                    L.send_attitude(mav, roll_c, pitch_c, target_yaw_deg,
                                    yaw_rate, L.HOVER_THRUST)
                    last_send = now

                    status = "PARKED" if (centred and aligned) else "CENTRE"
                    hud_phase = status
                    hud_cmd = (f"cmd roll={roll_c:+.1f} pitch={pitch_c:+.1f} "
                               f"yawRate={math.degrees(yaw_rate):+.0f}deg/s")
                    log.row(
                        mode=mav.flightmode, active=1, phase=status,
                        tag=1, tag_id=tag.tag_id,
                        x=round(tx, 4), y=round(ty, 4), z=round(tz, 4),
                        yaw_err_deg=round(tag_yaw, 2),
                        centred=int(centred), aligned=int(aligned),
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
                    log.event(f"[{status}] x={tx:+.2f} y={ty:+.2f} z={tz:.2f}m "
                          f"yawErr={tag_yaw:+.1f} -> roll={roll_c:+.1f} "
                          f"pitch={pitch_c:+.1f} yawRate={math.degrees(yaw_rate):+.0f} "
                          f"thr={L.HOVER_THRUST:.2f} centred={int(centred)} "
                          f"aligned={int(aligned)}")

                if draw:
                    corners = tag.corners.astype(int)
                    for i in range(4):
                        cv2.line(frame, tuple(corners[i]),
                                 tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
                    c = tuple(tag.center.astype(int))
                    cv2.circle(frame, c, 5, (0, 0, 255), -1)
                    # Crosshair = image centre = where the tag ends up when
                    # we're centred. Arrow = how far off we still are.
                    FlightVideo.draw_crosshair(frame)
                    # Arrow from image centre to tag: the way the quad must translate.
                    cv2.arrowedLine(frame, (frame.shape[1] // 2, frame.shape[0] // 2),
                                    c, (0, 255, 255), 2, tipLength=0.2)
                    cv2.putText(frame, f"ID{tag.tag_id} x={tx:+.2f} y={ty:+.2f} z={tz:.2f}",
                                (c[0] + 10, c[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 0, 0), 2)

            else:
                # No tag this frame -- hold level and hold altitude (never move
                # blind), keeping the CURRENT heading (0 would mean "north").
                if do_send:
                    L.send_attitude(mav, 0.0, 0.0, math.degrees(heading_rads),
                                    0.0, L.HOVER_THRUST)
                    prev["t"] = None
                    last_send = now
                    hud_phase = "HOLD"
                    hud_cmd = "cmd level, holding altitude"
                    log.event(f"[HOLD] tag lost {now - last_seen:.1f}s "
                              f"-- hovering level")
                    log.row(mode=mav.flightmode, active=1, phase="HOLD", tag=0,
                            roll_cmd_deg=0.0, pitch_cmd_deg=0.0,
                            cmd_yaw_rate_degs=0.0, thrust=L.HOVER_THRUST,
                            target_yaw_deg=round(math.degrees(heading_rads) % 360, 2),
                            heading_deg=round(math.degrees(heading_rads) % 360, 2),
                            meas_yaw_rate_degs=round(math.degrees(meas_yawspeed), 2),
                            roll_meas_deg=round(math.degrees(meas_roll), 2),
                            pitch_meas_deg=round(math.degrees(meas_pitch), 2))

            if draw:
                # Status block. `t=` is the SAME monotonic clock as the CSV's
                # `t` column, so any moment in the recording can be looked up
                # in the numbers (and the other way round).
                state = ("ACTIVE" if active else
                         "waiting for GUIDED_NOGPS" if armed else
                         "NOT ARMED (flick ch6 down, then up)")
                if tag is not None:
                    tag_line = (f"ID{tag.tag_id}  x={tx:+.2f} y={ty:+.2f} "
                                f"z={tz:.2f}m  yawErr={tag_yaw:+.1f}deg")
                else:
                    tag_line = ("NO TAG" if not last_seen else
                                f"NO TAG for {now - last_seen:.1f}s")
                FlightVideo.draw_status(frame, [
                    f"CenterOnAprilTag  t={log.elapsed():6.1f}s  [{hud_phase}]",
                    f"mode={mav.flightmode}  {state}",
                    tag_line,
                    hud_cmd,
                ])
                cv2.putText(frame, "CENTRE-ONLY: HOLDS ALTITUDE, NEVER DESCENDS",
                            (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 165, 255), 2)
                if video is not None:
                    video.write(frame)
                if show_window:
                    cv2.imshow("Centre-on-AprilTag (holds altitude)", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

            time.sleep(0.005)

    except KeyboardInterrupt:
        log.event(">>> Ctrl-C -- stopping.")
    except Exception:
        log.exception("in main loop")
        raise
    finally:
        picam2.stop()
        if video is not None:
            video.close()       # release the encoder or the file is unplayable
        if show_window:
            cv2.destroyAllWindows()
        log.event("Logs: %s | %s", log.log_path.name, log.csv_path.name)
        log.close()


if __name__ == "__main__":
    main()
