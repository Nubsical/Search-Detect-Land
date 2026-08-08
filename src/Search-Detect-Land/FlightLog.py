"""
Shared flight logging for the Search-Detect-Land scripts.

Two streams per run, both under `logs/` at the project root:

  logs/<script>_<timestamp>.log   human-readable events (same text you see on
                                  the console, plus timestamps and tracebacks)
  logs/<script>_<timestamp>.csv   one row per control cycle, for plotting

(FlightVideo adds a third file with the same stem -- .mp4 of the annotated
camera view -- when a script is recording. See `video_stem` below.)

Why both: the .log tells you the story ("entered GUIDED_NOGPS", "handed off to
LAND"), the .csv tells you the numbers. The .csv is the one that answers
questions like "was the FC actually acting on us?" -- it records the COMMANDED
yaw rate next to the MEASURED one, so a flat `meas` against a non-zero `cmd`
means the FC is discarding our messages. That is exactly the failure that made
SpinOnGuided look dead, and it would have shown up in one flight's log.

Crash safety
------------
These run on a Pi that can lose power abruptly, and the most interesting log is
the one from the flight that ended badly. Every event is flushed immediately;
CSV rows are flushed every row and fsync'd every FSYNC_PERIOD_S so at most a
fraction of a second is ever at risk. That costs a little SD-card throughput,
which is a fair trade at 10 Hz.

Clock caveat
------------
A Pi with no RTC boots at epoch/last-known time until NTP fixes it, so the wall
timestamps in a log can be wrong (and filenames with them). The `t` column is
elapsed seconds from a MONOTONIC clock, so it is always correct and is what you
should plot against.

Usage
-----
    log = FlightLog.open_log("SpinOnGuided")
    log.config(YAW_RATE_DEGS=YAW_RATE_DEGS, TYPE_MASK=TYPE_MASK)
    log.event("Heartbeat: system %d", mav.target_system)
    log.row(mode=mav.flightmode, cmd_yaw_rate_degs=120, meas_yaw_rate_degs=118)
    ...
    log.close()

`open_log` also installs an excepthook so an uncaught exception lands in the
file instead of only on a console nobody was watching.
"""

import atexit
import csv
import os
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

# Flush-to-disk cadence for the CSV. Every row is handed to the OS immediately;
# this is how often we additionally force it onto the card.
FSYNC_PERIOD_S = 2.0

# CSV schema. Fixed and shared by every script so one plotting script works on
# all of them -- a script simply leaves the columns it has no data for empty.
# Add new columns at the END so old logs stay readable.
FIELDS = [
    "t",                    # seconds since start, MONOTONIC (plot against this)
    "wall",                 # wall-clock ISO time (see clock caveat above)
    "mode",                 # FC flight mode string
    "active",               # 1 = we are commanding the vehicle
    "phase",                # script-specific: ALIGN/DESCEND/HOLD/spin/...
    # --- vision ---
    "tag",                  # 1 = tag detected this cycle
    "tag_id",
    "x", "y", "z",          # tag pose in the CAMERA frame (m)
    "yaw_err_deg",          # tag yaw error
    "centred", "aligned",
    # --- what we commanded ---
    "roll_cmd_deg", "pitch_cmd_deg",
    "target_yaw_deg",       # ABSOLUTE heading target in the quaternion
    "cmd_yaw_rate_degs",
    "thrust",
    "sent",                 # cumulative count of messages sent
    # --- what the FC reported back ---
    "heading_deg",          # ATTITUDE.yaw
    "meas_yaw_rate_degs",   # ATTITUDE.yawspeed  <-- compare to cmd_yaw_rate_degs
    "roll_meas_deg", "pitch_meas_deg",
    "alt_m",                # VFR_HUD.alt
    "landed",               # EXTENDED_SYS_STATE.landed_state
    "fc_armed",             # motors armed per HEARTBEAT
    # --- perception diagnostics (WatchAprilTag; blank elsewhere) ---
    "n_tags",               # tags detected this frame, target filter ignored
    "decision_margin",      # detector confidence; low = a marginal decode
    "detect_ms",            # time in detector.detect() for this frame
    "loop_hz",              # smoothed vision loop rate
    "exposure_us",          # camera ExposureTime -- long = motion blur
    "gain",                 # camera AnalogueGain -- high = AE starved for light
    "clip_pct",             # % of frame pinned at white; >0 risks a blown tag
    "focus_fom",            # camera FocusFoM -- low = out of focus/blurred
]


def _log_dir():
    """<project root>/logs, created on demand."""
    root = Path(__file__).resolve().parents[2]
    d = root / "logs"
    d.mkdir(parents=True, exist_ok=True)
    return d


class FlightLog:
    """Event log + CSV telemetry log for one run of one script."""

    def __init__(self, name, echo=True):
        self.name = name
        self.echo = echo
        self._t0 = time.monotonic()
        self._last_fsync = 0.0
        self._closed = False

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        d = _log_dir()
        self.log_path = d / f"{name}_{stamp}.log"
        self.csv_path = d / f"{name}_{stamp}.csv"
        # Same stem, no extension: FlightVideo hangs the annotated-camera
        # recording off this so all three files of a run sort together.
        self.video_stem = d / f"{name}_{stamp}"

        self._log_file = open(self.log_path, "a", buffering=1)  # line-buffered
        self._csv_file = open(self.csv_path, "w", newline="")
        self._csv = csv.DictWriter(self._csv_file, fieldnames=FIELDS,
                                   extrasaction="ignore")
        self._csv.writeheader()
        self._csv_file.flush()

        self.event("=== %s starting (log %s) ===", name, self.log_path.name)

    # -- events ---------------------------------------------------------
    def event(self, msg, *args):
        """Log a human-readable line (and echo it to the console)."""
        text = (msg % args) if args else msg
        if self.echo:
            print(text)
        if not self._closed:
            self._log_file.write(f"[{self.elapsed():9.3f}] {text}\n")

    def config(self, **values):
        """Record the tuning/config a run used, so a log is self-describing.

        Without this you cannot tell whether a log came from before or after a
        gain change -- which matters a lot when comparing two flights.
        """
        self.event("--- config ---")
        for k in sorted(values):
            self.event("    %s = %r", k, values[k])
        self.event("--------------")

    def exception(self, context=""):
        """Log the currently-handled exception with its traceback."""
        self.event("!! EXCEPTION %s: %s", context, traceback.format_exc())

    # -- telemetry ------------------------------------------------------
    def row(self, **fields):
        """Write one CSV row. Unknown keys are ignored, missing ones blank."""
        if self._closed:
            return
        fields.setdefault("t", round(self.elapsed(), 3))
        fields.setdefault("wall", datetime.now().isoformat(timespec="milliseconds"))
        self._csv.writerow(fields)
        self._csv_file.flush()

        now = time.monotonic()
        if now - self._last_fsync >= FSYNC_PERIOD_S:
            self._last_fsync = now
            for f in (self._csv_file, self._log_file):
                try:
                    os.fsync(f.fileno())
                except OSError:
                    pass    # never let logging take down the control loop

    def elapsed(self):
        return time.monotonic() - self._t0

    # -- teardown -------------------------------------------------------
    def close(self):
        if self._closed:
            return
        self.event("=== %s finished (%.1f s) ===", self.name, self.elapsed())
        self._closed = True
        for f in (self._csv_file, self._log_file):
            try:
                f.flush()
                os.fsync(f.fileno())
            except OSError:
                pass
            f.close()


def open_log(name, echo=True):
    """Start a FlightLog, routing uncaught exceptions and exit into it."""
    log = FlightLog(name, echo=echo)

    prev_hook = sys.excepthook

    def hook(exc_type, exc, tb):
        if not log._closed:
            log.event("!! UNCAUGHT %s", "".join(
                traceback.format_exception(exc_type, exc, tb)))
        log.close()
        prev_hook(exc_type, exc, tb)

    sys.excepthook = hook
    atexit.register(log.close)
    return log
