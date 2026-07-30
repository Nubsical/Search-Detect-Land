"""
Tag watcher  --  FLY IT MANUALLY (STABILIZE) AND SEE WHAT THE CAMERA SEES.

SENDS NO FLIGHT COMMANDS. Ever. Not a single SET_ATTITUDE_TARGET, no mode
change, no arming. It only listens to the FC and looks at the camera, so it is
safe to leave running in STABILIZE (or ALT_HOLD, or anything else) while you fly
the quad by hand. The flight mode is recorded, never required and never changed.

Why this exists
---------------
"The program isn't detecting the tag" is not one question, it is four:

    is the tag detected at all?  ...on the ground?  ...in the air?
    at what altitude does it stop?  and what does the frame look like when it does?

CenterOnAprilTag / LandOnAprilTag only tell you any of that while the ch6 switch
is UP and they are flying the aircraft, which is a bad moment to be debugging
perception. This script answers the four questions with no control authority at
all: hand-fly the quad over the tag in STABILIZE and afterwards you have

    logs/WatchAprilTag_<stamp>.log   TAG ID.. DETECTED / TAG LOST, with the
                                     altitude and pose at each edge
    logs/WatchAprilTag_<stamp>.csv   one row per frame: detections, pose,
                                     detect time, loop rate, exposure, focus
    logs/WatchAprilTag_<stamp>.mp4   the camera's point of view for the whole
                                     flight, with the outline drawn round the
                                     tag on every frame it was detected

...plus a summary printed on exit: detection rate, per-id counts, and the
GREATEST RANGE the tag actually decoded at, which is the number you want when
you are deciding whether the detector or the flight code is at fault.

While you are flying
--------------------
You cannot read the console mid-flight, so detections also go out as MAVLink
STATUSTEXT -- they pop up in Mission Planner / QGroundControl / on the OSD as
the tag is picked up and lost. `--no-gcs-text` turns that off.

Watching it (running over SSH)
------------------------------
Same deal as the other scripts: no DISPLAY means no preview window (cv2.imshow
would raise), so the annotated view goes to the .mp4 instead. `scp` it off the
Pi when you land -- that recording IS the deliverable here.

    $ python3 WatchAprilTag.py                  # record, no window over SSH
    $ python3 WatchAprilTag.py --no-mavlink     # bench: camera only, no FC
    $ python3 WatchAprilTag.py --quad-sigma 0.0 --tag-size 0.20   # try knobs

Blurry when it moves?
---------------------
That is the usual reason a tag that is obviously in frame refuses to decode, and
it is two different faults that look the same. The overlay tells them apart --
it prints the exposure and the sensor's own focus figure-of-merit on every frame:

    exp= high (>10000us)        -> MOTION BLUR. Cap the shutter:
                                   --manual-exposure-us 4000 --gain 6
    exp= low but still blurry   -> AUTOFOCUS HUNTING. Pin the lens:
                                   --fixed-focus 0        (0 dioptres = infinity)

Whatever works here goes into LandOnAprilTag's MAX_EXPOSURE_US /
MANUAL_EXPOSURE_US / AF_MODE, which is where every other script reads it from.

Everything about the camera, the calibration and the detector is imported from
LandOnAprilTag, so what this script sees IS what the flight scripts see -- if a
tag decodes here it will decode there. The --tag-size / --quad-* / --tag-id
flags override those constants FOR THIS RUN ONLY, so you can sweep detector
settings in the field without editing (and risking) the flight code. The values
actually used are written into the .log's config block, so a recording can never
be separated from the settings that made it.

  !!  TAG_SIZE MUST BE THE REAL PRINTED SIZE  !!
  It is the black square's edge in metres (currently 0.125 in LandOnAprilTag).
  A wrong TAG_SIZE does not stop detection -- it silently scales z, so every
  altitude in the log and every landing decision downstream is wrong by the same
  factor. Measure the tag you actually taped down.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2

# Import the REAL flight code so this tool uses the identical camera, calibration
# and detector as the flight scripts (single source of truth). Importing does NOT
# open the camera or MAVLink -- that lives under LandOnAprilTag's __main__ guard.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import LandOnAprilTag as L
import FlightLog
import FlightVideo

from pymavlink import mavutil


# A single dropped frame is normal (motion blur, a bad exposure), so don't cry
# "lost" on the first miss -- wait this long without any detection first.
TAG_LOST_GRACE_S = 1.0
# While nothing is in view, repeat the "still looking" line this often, so a
# silent log means "script died", not "no tag".
NO_TAG_NOTE_S = 5.0
# Heartbeat line with the running detection rate, so a long flight's log shows
# progress even when the tag is never seen.
STATUS_PERIOD_S = 15.0
# Camera metadata (exposure, focus, gain) costs a frame wait, so sample it
# slowly -- it changes on the timescale of the light, not the frame.
META_PERIOD_S = 1.0
# Floor on the gap between STATUSTEXT messages, so a tag flickering on the edge
# of detection cannot flood the telemetry link (it is the same link the pilot
# needs for everything else).
GCS_MIN_INTERVAL_S = 2.0

# Overlay colours (BGR). The target tag is green like every other script in the
# repo; a tag we are filtering OUT is drawn grey, so "it IS detecting something,
# just not the id you asked for" is obvious in the recording.
COLOR_TAG = (0, 255, 0)
COLOR_OTHER = (150, 150, 150)
COLOR_CENTRE = (0, 0, 255)


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Passive AprilTag watcher: reports detections and records "
                    "the annotated camera view. Sends NO flight commands, so "
                    "it is safe to run while hand-flying in STABILIZE.")
    L.view_args(p)
    p.add_argument("--no-mavlink", action="store_true",
                   help="do not talk to the flight controller at all (bench "
                        "runs, or when the FC is not powered)")
    p.add_argument("--no-gcs-text", action="store_true",
                   help="do not send detections to the GCS as STATUSTEXT")
    p.add_argument("--fc-timeout", type=float, default=10.0,
                   help="seconds to wait for a heartbeat before giving up and "
                        "running camera-only (default 10)")
    p.add_argument("--tag-size", type=float, default=None,
                   help=f"black-square edge in METRES; wrong values silently "
                        f"scale z (default {L.TAG_SIZE})")
    p.add_argument("--tag-id", type=int, default=None,
                   help="only treat this tag id as the target (default: any). "
                        "Other ids are still detected, logged and drawn grey.")
    p.add_argument("--quad-decimate", type=float, default=None,
                   help=f"detector downsample before quad detection; higher is "
                        f"faster but kills detection range (default "
                        f"{L.QUAD_DECIMATE})")
    p.add_argument("--quad-sigma", type=float, default=None,
                   help=f"detector blur; a little HELPS small/distant tags on a "
                        f"noisy image (default {L.QUAD_SIGMA})")
    p.add_argument("--max-exposure-us", type=int, default=None,
                   help=f"cap the shutter to fight motion blur, AE still on; "
                        f"0 disables (default {L.MAX_EXPOSURE_US}). The overlay "
                        f"shows the exposure actually used -- check it moved.")
    p.add_argument("--manual-exposure-us", type=int, default=None,
                   help="pin the shutter with AE OFF -- always takes effect, "
                        "but does not adapt to light (use --gain with it)")
    p.add_argument("--gain", type=float, default=None,
                   help=f"analogue gain for --manual-exposure-us (default "
                        f"{L.MANUAL_GAIN}); raise it if the image goes dark")
    p.add_argument("--fixed-focus", type=float, nargs="?", const=0.0,
                   default=None, metavar="DIOPTRES",
                   help="disable continuous autofocus and pin the lens "
                        "(0 = infinity, 1.0 = 1 m, 2.0 = 0.5 m). Try this if "
                        "capping exposure did not sharpen fast motion -- AF "
                        "hunting looks just like blur.")
    return p.parse_args(argv)


def apply_overrides(args, log):
    """Push the CLI overrides into LandOnAprilTag's constants for this run.

    The camera/detector helpers read these module globals when they are called,
    which happens after this -- so setting them here is all that is needed, and
    nothing on disk is modified.
    """
    if args.tag_size is not None:
        L.TAG_SIZE = args.tag_size
    if args.tag_id is not None:
        L.TARGET_TAG_ID = args.tag_id
    if args.quad_decimate is not None:
        L.QUAD_DECIMATE = args.quad_decimate
    if args.quad_sigma is not None:
        L.QUAD_SIGMA = args.quad_sigma
    if args.max_exposure_us is not None:
        L.MAX_EXPOSURE_US = args.max_exposure_us
    if args.manual_exposure_us is not None:
        L.MANUAL_EXPOSURE_US = args.manual_exposure_us
    if args.gain is not None:
        L.MANUAL_GAIN = args.gain
    if args.fixed_focus is not None:
        L.AF_MODE = 0
        L.LENS_POSITION = args.fixed_focus
    log.config(TAG_SIZE=L.TAG_SIZE, TARGET_TAG_ID=L.TARGET_TAG_ID,
               CAM_RES=L.CAM_RES, SENSOR_MODE=L.SENSOR_MODE,
               QUAD_DECIMATE=L.QUAD_DECIMATE, QUAD_SIGMA=L.QUAD_SIGMA,
               NTHREADS=L.NTHREADS, MAX_EXPOSURE_US=L.MAX_EXPOSURE_US,
               MANUAL_EXPOSURE_US=L.MANUAL_EXPOSURE_US,
               MANUAL_GAIN=L.MANUAL_GAIN, AF_MODE=L.AF_MODE,
               LENS_POSITION=L.LENS_POSITION)


def connect_fc(args, log):
    """Connect to the FC for TELEMETRY ONLY. Returns the link, or None.

    Nothing here is required: this script's job is perception, so a missing or
    unpowered FC degrades to "no mode/altitude in the log" rather than stopping
    the run. The only bytes we ever send are stream requests and (optionally)
    STATUSTEXT -- never a control message.
    """
    if args.no_mavlink:
        log.event("--no-mavlink: camera only, not connecting to the FC.")
        return None
    log.event("Connecting to %s @ %d (telemetry only) ...",
              L.CONNECTION_STRING, L.BAUD_RATE)
    try:
        mav = mavutil.mavlink_connection(L.CONNECTION_STRING, baud=L.BAUD_RATE)
    except Exception as exc:
        log.event("!! Cannot open %s (%s) -- carrying on camera-only.",
                  L.CONNECTION_STRING, exc)
        return None
    if mav.wait_heartbeat(timeout=args.fc_timeout) is None:
        log.event("!! No heartbeat in %.0fs -- carrying on camera-only "
                  "(mode/altitude columns will be blank).", args.fc_timeout)
        try:
            mav.close()
        except Exception:
            pass
        return None
    log.event("Heartbeat: system %d, component %d",
              mav.target_system, mav.target_component)
    # Altitude is the single most useful number to have next to a detection --
    # "it stopped seeing the tag at 6 m" is the answer we are usually after.
    L.request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 4)
    return mav


def gcs_say(mav, args, state, text):
    """Send one STATUSTEXT, rate-limited. Best effort, never raises.

    `state` carries the last-sent time between calls so the limiter survives
    across the two places that report (detected / lost).
    """
    if mav is None or args.no_gcs_text:
        return
    now = time.monotonic()
    if now - state["last_gcs"] < GCS_MIN_INTERVAL_S:
        return
    state["last_gcs"] = now
    try:
        mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                                text[:50].encode("utf-8"))
    except Exception:
        pass        # a telemetry hiccup must not interrupt the watch


def read_meta(picam2, log_once):
    """Exposure / gain / focus from the camera, or {} if unavailable.

    FocusFoM is the useful one for this job: it is the sensor's own focus
    figure-of-merit, and a frame with a low FoM is a blurry frame, which is the
    usual reason a tag that "should" be visible does not decode. ExposureTime
    tells you the other half (long exposure -> motion blur while moving).
    """
    try:
        return picam2.capture_metadata() or {}
    except Exception as exc:
        log_once(exc)
        return {}


def draw_tag(frame, r, is_target, tz=None):
    """Outline the tag, mark its centre, label it."""
    color = COLOR_TAG if is_target else COLOR_OTHER
    corners = r.corners.astype(int)
    for i in range(4):
        cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                 color, 2, cv2.LINE_AA)
    # Solid blobs on the corners: at altitude the outline is only a few pixels
    # across and the plain quad becomes hard to see in a replay.
    for c in corners:
        cv2.circle(frame, tuple(c), 3, color, -1)
    centre = tuple(r.center.astype(int))
    cv2.circle(frame, centre, 5, COLOR_CENTRE, -1)
    label = f"ID{r.tag_id}" if is_target else f"ID{r.tag_id} (not target)"
    if tz is not None:
        label += f" z={tz:.2f}m m={r.decision_margin:.0f}"
    cv2.putText(frame, label, (centre[0] + 10, centre[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, label, (centre[0] + 10, centre[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)


def summarise(log, stats, frames, frames_with_tag, elapsed, video):
    """The point of the whole run: did it see the tag, and how far away."""
    log.event("=" * 62)
    log.event("SUMMARY")
    rate = (frames / elapsed) if elapsed > 0 else 0.0
    log.event("  frames: %d in %.1fs (%.1f fps)", frames, elapsed, rate)
    if frames:
        log.event("  frames with a tag: %d (%.1f%%)", frames_with_tag,
                  100.0 * frames_with_tag / frames)
    if not stats:
        log.event("  NO TAG WAS EVER DETECTED.")
        log.event("  Check, in this order: is the tag in frame at all (watch "
                  "the recording)? is it in focus / not motion-blurred? is it "
                  "tag36h11? is it big enough in the image?")
    for tid in sorted(stats):
        s = stats[tid]
        # z_max is the headline number: the greatest range at which this tag
        # actually decoded, i.e. the altitude ceiling of the whole pipeline.
        log.event("  ID%-4d %6d frames  decoded out to %.2f m (closest %.2f m)"
                  "  best margin %.0f  %s",
                  tid, s["count"], s["z_max"], s["z_min"], s["margin_max"],
                  "TARGET" if s["target"] else "(not the target id)")
    if video is not None and video.path is not None:
        log.event("  video: %s   <-- scp this off the Pi", video.path.name)
    log.event("=" * 62)


def main(args=None):
    args = args if args is not None else parse_args()
    log = FlightLog.open_log("WatchAprilTag")
    log.event("PASSIVE WATCH -- no flight commands are sent. Safe to run while "
              "hand-flying in STABILIZE.")
    apply_overrides(args, log)

    show_window, video = L.open_view(args, log)
    # Without a window and without a recording there is nothing to draw for, so
    # skip the colour remap in grab_gray entirely (it is not free at CAM_RES).
    draw = show_window or (video is not None)

    mav = connect_fc(args, log)
    gate = L.GroundGate(log) if mav is not None else None
    if gate is not None:
        gate.request(mav)

    camera_matrix, dist_coeffs, camera_params = L.load_calibration()
    map1, map2 = L.build_undistort_maps(camera_matrix, dist_coeffs)
    picam2 = L.open_camera()
    detector = L.build_detector()
    log.event("Watching for tag36h11 (%s). Ctrl-C to stop and write the "
              "summary.",
              "any id" if L.TARGET_TAG_ID is None else f"id {L.TARGET_TAG_ID}")

    # Per-id detection bookkeeping -- this is what the summary is built from.
    stats = {}
    state = {"last_gcs": 0.0}
    frames = 0
    frames_with_tag = 0
    last_any_seen = 0.0         # last frame with ANY detection
    last_no_tag_note = 0.0
    last_status = time.monotonic()
    last_meta = 0.0
    meta = {}
    alt_m = None
    mode = "n/a"
    loop_hz = 0.0
    last_loop = None
    meta_warned = []

    def meta_warn(exc):
        if not meta_warned:
            meta_warned.append(True)
            log.event("!! Camera metadata unavailable (%s) -- exposure/focus "
                      "columns will be blank.", exc)

    t_start = time.monotonic()
    try:
        while True:
            # Drain telemetry so `mode` and the altitude stay current. Never
            # block: the vision loop is the thing that has to keep running.
            if mav is not None:
                while True:
                    msg = mav.recv_match(blocking=False)
                    if msg is None:
                        break
                    gate.observe(msg)
                    if msg.get_type() == 'VFR_HUD':
                        alt_m = msg.alt
                mode = mav.flightmode

            frame, gray = L.grab_gray(picam2, map1, map2, want_color=draw)

            t_detect = time.monotonic()
            results = detector.detect(gray, estimate_tag_pose=True,
                                      camera_params=camera_params,
                                      tag_size=L.TAG_SIZE)
            detect_ms = (time.monotonic() - t_detect) * 1000.0

            now = time.monotonic()
            if last_loop is not None and now > last_loop:
                # Smoothed, because a single slow frame is not "the loop rate".
                inst_hz = 1.0 / (now - last_loop)
                loop_hz = inst_hz if not loop_hz else 0.8 * loop_hz + 0.2 * inst_hz
            last_loop = now
            frames += 1

            if now - last_meta >= META_PERIOD_S:
                last_meta = now
                meta = read_meta(picam2, meta_warn)

            # ---- Detections ------------------------------------------------
            target = None       # the one matching TARGET_TAG_ID (or the first)
            seen_now = set()
            for r in results:
                is_target = (L.TARGET_TAG_ID is None or
                             r.tag_id == L.TARGET_TAG_ID)
                tz = float(r.pose_t.flatten()[2])
                seen_now.add(r.tag_id)

                s = stats.get(r.tag_id)
                if s is None:
                    s = stats[r.tag_id] = {"count": 0, "visible": False,
                                           "last": 0.0, "target": is_target,
                                           "z_min": tz, "z_max": tz,
                                           "margin_max": 0.0}
                s["count"] += 1
                s["last"] = now
                s["z_min"] = min(s["z_min"], tz)
                s["z_max"] = max(s["z_max"], tz)
                s["margin_max"] = max(s["margin_max"], float(r.decision_margin))

                # Edge-triggered: a tag held in view logs ONE line, not one per
                # frame -- otherwise the interesting edges drown in the stream.
                if not s["visible"]:
                    s["visible"] = True
                    tx, ty, _ = r.pose_t.flatten()
                    log.event(">>> TAG ID%d DETECTED  x=%+.2f y=%+.2f z=%.2fm  "
                              "margin=%.0f hamming=%d  alt=%s mode=%s%s",
                              r.tag_id, tx, ty, tz, r.decision_margin,
                              r.hamming,
                              "?" if alt_m is None else f"{alt_m:.1f}m", mode,
                              "" if is_target else "  [NOT the target id]")
                    gcs_say(mav, args, state,
                            f"AprilTag ID{r.tag_id} SEEN z={tz:.1f}m")

                if is_target and target is None:
                    target = r
                if draw:
                    draw_tag(frame, r, is_target, tz)

            if seen_now:
                last_any_seen = now
                frames_with_tag += 1

            # ---- Lost edges --------------------------------------------------
            for tid, s in stats.items():
                if s["visible"] and tid not in seen_now and \
                        (now - s["last"]) >= TAG_LOST_GRACE_S:
                    s["visible"] = False
                    log.event(">>> TAG ID%d LOST (%.1fs without a detection)  "
                              "alt=%s mode=%s", tid, now - s["last"],
                              "?" if alt_m is None else f"{alt_m:.1f}m", mode)
                    gcs_say(mav, args, state, f"AprilTag ID{tid} LOST")

            if not seen_now and (now - last_no_tag_note) >= NO_TAG_NOTE_S:
                last_no_tag_note = now
                if last_any_seen:
                    log.event("... no tag in view (%.1fs since the last one)",
                              now - last_any_seen)
                else:
                    log.event("... no tag in view yet (nothing detected since "
                              "start)")

            # Periodic proof-of-life with the running numbers, so the log is
            # useful even on a flight where the tag never appeared.
            if now - last_status >= STATUS_PERIOD_S:
                last_status = now
                log.event("[watch] t=%.0fs  %.1f fps  detect %.0f ms  "
                          "%d/%d frames with a tag  mode=%s alt=%s",
                          now - t_start, loop_hz, detect_ms, frames_with_tag,
                          frames, mode,
                          "?" if alt_m is None else f"{alt_m:.1f}")

            # ---- Log one row per frame --------------------------------------
            row = dict(mode=mode, active=0,
                       phase="TAG" if seen_now else "SEARCH",
                       tag=int(bool(seen_now)), n_tags=len(results),
                       detect_ms=round(detect_ms, 1),
                       loop_hz=round(loop_hz, 2),
                       exposure_us=meta.get("ExposureTime"),
                       focus_fom=meta.get("FocusFoM"))
            if alt_m is not None:
                row["alt_m"] = round(alt_m, 2)
            if gate is not None:
                row["landed"] = gate.name
                row["fc_armed"] = int(gate.motors_armed)
            if target is not None:
                tx, ty, tz = target.pose_t.flatten()
                _, _, tag_yaw = L.rotation_matrix_to_euler_angles(target.pose_R)
                row.update(tag_id=target.tag_id, x=round(tx, 4),
                           y=round(ty, 4), z=round(tz, 4),
                           yaw_err_deg=round(tag_yaw, 2),
                           decision_margin=round(float(target.decision_margin), 1))
            log.row(**row)

            # ---- Overlay + recording ----------------------------------------
            if draw:
                if target is not None:
                    tx, ty, tz = target.pose_t.flatten()
                    tag_line = (f"ID{target.tag_id}  x={tx:+.2f} y={ty:+.2f} "
                                f"z={tz:.2f}m  margin={target.decision_margin:.0f}")
                elif seen_now:
                    tag_line = f"tags {sorted(seen_now)} -- none is the target id"
                else:
                    tag_line = ("NO TAG" if not last_any_seen else
                                f"NO TAG for {now - last_any_seen:.1f}s")
                exp = meta.get("ExposureTime")
                fom = meta.get("FocusFoM")
                FlightVideo.draw_status(frame, [
                    f"WatchAprilTag  t={log.elapsed():6.1f}s  PASSIVE",
                    f"mode={mode}  alt={'?' if alt_m is None else f'{alt_m:.1f}m'}"
                    f"  {loop_hz:.1f}fps  detect {detect_ms:.0f}ms",
                    tag_line,
                    f"seen {frames_with_tag}/{frames} frames"
                    f"{'' if exp is None else f'   exp={exp}us'}"
                    f"{'' if fom is None else f' focus={fom}'}",
                ])
                FlightVideo.draw_crosshair(frame)
                cv2.putText(frame, "PASSIVE WATCH: NO COMMANDS SENT",
                            (10, frame.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
                if video is not None:
                    video.write(frame)
                if show_window:
                    cv2.imshow("WatchAprilTag (passive)", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

    except KeyboardInterrupt:
        log.event(">>> Ctrl-C -- stopping.")
    except Exception:
        log.exception("in main loop")
        raise
    finally:
        picam2.stop()
        summarise(log, stats, frames, frames_with_tag,
                  time.monotonic() - t_start, video)
        if video is not None:
            video.close()       # release the encoder or the file is unplayable
        if show_window:
            cv2.destroyAllWindows()
        if mav is not None:
            try:
                mav.close()
            except Exception:
                pass
        log.event("Logs: %s | %s", log.log_path.name, log.csv_path.name)
        log.close()


if __name__ == "__main__":
    main()
