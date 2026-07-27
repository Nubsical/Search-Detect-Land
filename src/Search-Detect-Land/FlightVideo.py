"""
Annotated-video recording for the Search-Detect-Land scripts.

Why this exists
---------------
The flight scripts draw an overlay on each camera frame -- the tag outline, its
centre, the arrow showing which way the quad has to move, the pose numbers --
and show it in a cv2 preview window. That window only works when there is a
display attached. Run the script over a plain `ssh pi@...` and there is no
DISPLAY at all, so the preview is useless (worse: cv2.imshow RAISES, which
would take the control loop down mid-flight).

So instead of showing the overlay, we write it to a video file next to the run's
.log and .csv:

    logs/CenterOnAprilTag_20260726_143012.log
    logs/CenterOnAprilTag_20260726_143012.csv
    logs/CenterOnAprilTag_20260726_143012.mp4   <-- this

After the flight you `scp` the .mp4 off the Pi and watch exactly what the
detector saw and what the controller was doing about it. It is the piece the
CSV cannot give you: whether the tag was even in frame, whether it was blurred,
whether the overlay was pointing the right way.

Cost
----
Encoding is NOT free and this runs inside the control loop, so:
  - frames are downscaled to VIDEO_WIDTH first (CAM_RES is 2304x1296 -- far too
    big to encode at 10 Hz on a Pi), and
  - anything that goes wrong (no codec, disk full, encoder error) disables
    recording and logs it, but NEVER raises into the flight loop. Same rule as
    FlightLog: logging must not be able to crash the aircraft.

Playback timing
---------------
A video file has one fixed frame rate, but the vision loop does not run at a
fixed rate -- detection time varies with what is in frame. Writing one file
frame per loop iteration would give a video whose playback speed drifts with
detector load, which makes it useless for judging how fast the quad reacted.
Instead the recorder keeps a real-time timeline: it drops frames when the loop
runs faster than the file rate and repeats the last frame when it runs slower,
so one second of video is one second of flight.
"""

import time
from pathlib import Path

import cv2

# Tried in order; first one that actually opens wins. mp4v/.mp4 plays anywhere;
# MJPG/.avi is the fallback for an OpenCV build without the mp4 encoder (it
# makes much bigger files, which is why it is second).
CODECS = (("mp4v", ".mp4"), ("MJPG", ".avi"))

# Cap on how many duplicate frames one write() may emit to catch up. Without it
# a long stall (say the detector chokes for 30 s) would dump 300 identical
# frames into the file in one loop iteration -- a stall in its own right.
MAX_CATCHUP_S = 2.0


class VideoRecorder:
    """Writes annotated frames to a video file, real-time paced.

    Opens lazily on the first frame, because the output size is whatever the
    first frame turns out to be after downscaling.
    """

    def __init__(self, stem, fps=10.0, width=960, log=None):
        self._stem = Path(stem)         # path WITHOUT extension; codec picks it
        self.fps = float(fps)
        self.width = int(width)
        self._log = log
        self._writer = None
        self._size = None               # (w, h) the file was opened with
        self.path = None                # set once a codec opens
        self._t0 = None                 # monotonic time of the first frame
        self._slots = 0                 # position on the fixed-rate timeline
        self._frames = 0                # frames actually written
        self._dead = False              # gave up; stop trying

    # -- internals ------------------------------------------------------
    def _note(self, msg, *args):
        if self._log is not None:
            self._log.event(msg, *args)
        else:
            print(msg % args if args else msg)

    def _scale(self, frame):
        h, w = frame.shape[:2]
        if not self.width or w <= self.width:
            return frame
        s = self.width / float(w)
        # Keep both dimensions even -- several encoders reject odd sizes.
        size = (max(2, int(w * s) // 2 * 2), max(2, int(h * s) // 2 * 2))
        return cv2.resize(frame, size, interpolation=cv2.INTER_AREA)

    def _open(self, frame):
        h, w = frame.shape[:2]
        for fourcc, ext in CODECS:
            path = self._stem.with_suffix(ext)
            writer = cv2.VideoWriter(str(path),
                                     cv2.VideoWriter_fourcc(*fourcc),
                                     self.fps, (w, h))
            if writer.isOpened():
                self._writer, self.path, self._size = writer, path, (w, h)
                self._note("Recording annotated video -> %s (%dx%d @ %.1f fps, "
                           "%s)", path.name, w, h, self.fps, fourcc)
                return True
            writer.release()
        self._dead = True
        self._note("!! No usable video codec (tried %s) -- running without a "
                   "recording.", ", ".join(c for c, _ in CODECS))
        return False

    # -- api ------------------------------------------------------------
    def write(self, frame):
        """Add one annotated frame. Never raises."""
        if self._dead or frame is None:
            return
        try:
            frame = self._scale(frame)
            if self._writer is None:
                if not self._open(frame):
                    return
                self._t0 = time.monotonic()
            if (frame.shape[1], frame.shape[0]) != self._size:
                frame = cv2.resize(frame, self._size)   # camera mode changed?

            # Where this frame belongs on the real-time timeline.
            slot = int((time.monotonic() - self._t0) * self.fps) + 1
            if slot <= self._slots:
                return          # loop is ahead of the file rate: drop it
            pad = min(slot - self._slots, max(1, int(self.fps * MAX_CATCHUP_S)))
            for _ in range(pad):
                self._writer.write(frame)
                self._frames += 1
            # Jump the timeline to `slot` even if we padded less than the full
            # gap: after a long stall we'd rather lose real time from the video
            # than spend the loop writing a burst of identical frames.
            self._slots = slot
        except Exception as exc:      # a bad frame must not end the flight
            self._dead = True
            self._note("!! Video recording failed (%s) -- continuing without "
                       "it.", exc)

    def close(self):
        if self._writer is None:
            return
        try:
            self._writer.release()
        except Exception:
            pass
        self._writer = None
        self._note("Video: %s (%d frames, %.1f s)", self.path.name,
                   self._frames, self._frames / self.fps)


def open_video(log, fps=10.0, width=960):
    """Start a recorder named to match `log`'s .log/.csv pair."""
    return VideoRecorder(log.video_stem, fps=fps, width=width, log=log)


# ----------------------------------------------------------------------
# Overlay helpers (shared so every script's recording reads the same way)
# ----------------------------------------------------------------------
def draw_status(frame, lines, org=(10, 24), color=(255, 255, 255), scale=0.6):
    """Draw a block of status text top-left, on a dark strip so it stays legible.

    The recording has to be readable on its own months later -- "the tag is
    there" is only half the story, you also need to know what mode the FC was
    in and what we were commanding at that instant.
    """
    x, y = org
    step = int(28 * scale / 0.6)
    for i, line in enumerate(lines):
        pos = (x, y + i * step)
        cv2.putText(frame, line, pos, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 4, cv2.LINE_AA)          # outline for contrast
        cv2.putText(frame, line, pos, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, 1, cv2.LINE_AA)


def draw_crosshair(frame, color=(200, 200, 200), size=20):
    """Mark image centre -- where the tag has to end up for 'centred'."""
    cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
    cv2.line(frame, (cx - size, cy), (cx + size, cy), color, 1)
    cv2.line(frame, (cx, cy - size), (cx, cy + size), color, 1)
