"""
Land-on-AprilTag test.

Companion-computer (Raspberry Pi) script for the Search-Detect-Land quad.

What it does
------------
While the flight controller is in GUIDED_NOGPS (channel-6 switch UP, exactly
like SpinOnGuided.py), this script:

  1. detects a downward-facing AprilTag with the Pi camera (same pipeline as
     AprilTagTesting_2.py -- calibration pickles, pupil_apriltags Detector),
  2. drives the quad to sit directly ABOVE the tag while holding altitude
     (a PD controller tilts roll/pitch to zero out the tag's horizontal
     offset), and yaws the quad so its front lines up with the FRONT of the
     tag, then
  3. centres + yaw-aligns WHILE descending, and keeps tracking the tag all the
     way down. It hands the final touchdown to the FC's LAND mode only once it
     can no longer see the tag because the quad is right on top of it (the tag
     overflows the camera frame), or a safety-floor altitude is reached. LAND
     then descends in place and cuts the motors when it detects the ground.

Same safety model as SpinOnGuided.py
------------------------------------
The RC 3-position switch on channel 6 selects the FC flight mode itself
(FLTMODE_CH=6; UP -> GUIDED_NOGPS, DOWN/MIDDLE -> STABILIZE). GUIDED_NOGPS is
the only Copter mode that honours the offboard SET_ATTITUDE_TARGET messages
this script sends. Flip the switch DOWN at ANY time and the FC returns to
STABILIZE, which drops every command this script could send -- you get manual
control instantly. The SWITCH (via the flight mode), not this script, is the
real safety gate.

There is ONE guard on top of that: after this process (re)starts it will NOT
act until it has seen the vehicle OUT of GUIDED_NOGPS at least once (the `armed`
latch). An auto-restart while the switch is already UP can't silently resume
control -- it takes a deliberate flick DOWN then UP.

  !!  BENCH-TEST FIRST, PROPS OFF.  !!
  The roll/pitch and yaw SIGN CONVENTIONS below depend on how the camera is
  mounted (which way is "camera right"/"camera forward"). Hold the airframe by
  hand over a tag, watch the printed commanded roll/pitch/yaw, and confirm the
  tilt pushes the quad TOWARD the tag (not away) before ever doing this with
  props on. In GUIDED_NOGPS the FC controls throttle via the thrust field, not
  you -- see HOVER_THRUST / DESCEND_THRUST.
"""

import argparse
import math
import os
import time
from pathlib import Path
import pickle

import cv2
import numpy as np
from pupil_apriltags import Detector
from picamera2 import Picamera2
from pymavlink import mavutil

import FlightLog
import FlightVideo

# ======================================================================
# CONNECTION
# ======================================================================
CONNECTION_STRING = '/dev/ttyAMA0'
BAUD_RATE = 115200

# ======================================================================
# FLIGHT-MODE GATE  (see module docstring -- identical model to SpinOnGuided)
# ======================================================================
# pymavlink's name for Copter mode 20. Printed at startup so you can confirm
# it against mav.flightmode on your own FC.
TARGET_MODE = 'GUIDED_NOGPS'
LAND_MODE = 'LAND'          # FC mode we hand off to for the final touchdown

# ======================================================================
# CAMERA / DETECTOR  (Raspberry Pi Camera Module 3 = Sony IMX708, on a
# Raspberry Pi 5 8GB. Tuned to detect the tag from AS HIGH AS POSSIBLE.)
#
# NOT the NoIR variant: this module has an IR-cut filter, so daylight contrast is
# better and low light is worse. Same sensor and driver either way -- the sensor
# modes and controls below are unchanged. What DOES change with the module is the
# LENS: the Wide (~102 deg horizontal) has roughly half the focal length of the
# standard (~66 deg), so it resolves a tag from about half the altitude but keeps
# it in frame closer to the ground. Recalibrate after any module swap.
# ======================================================================
TAG_SIZE = 0.125            # metres, black square edge length
TARGET_TAG_ID = None        # set to an int to only accept that tag id; None = any

# --- Sensor mode / resolution ---------------------------------------------
# Detection range is set by how many PIXELS land on the tag, so for max altitude
# we want resolution, not speed. The IMX708 has two full-field-of-view readouts:
#   (2304, 1296)  2x2-binned  -- fast, lower noise
#   (4608, 2592)  full 11.9MP -- ~2x the range, but ~4x the CPU per frame
# We drive the camera in the binned full-FOV mode and hand that straight to the
# detector (no downscale) -- a strong range/speed balance the Pi 5 can sustain.
#
# SENSOR_MODE picks the raw readout (keep it a FULL-FOV native mode so the lens
# calibration stays valid). CAM_RES is what the detector sees; keep it == or
# below SENSOR_MODE. To reach even higher, set both to (4608, 2592) and expect
# a much lower frame rate. To recover speed, lower CAM_RES (still full FOV,
# just fewer pixels) BEFORE you touch quad_decimate.
#   !! Recalibrate (cameraMatrix.pkl / dist.pkl) at EXACTLY CAM_RES + full FOV,
#      or the pose (x/y/z and the LAND_HANDOFF_ALT check) will be wrong. !!
SENSOR_MODE = (2304, 1296)  # IMX708 binned, FULL FOV -- the raw readout
CAM_RES = (2304, 1296)      # what the detector sees (<= SENSOR_MODE)

# --- Detector tuning ------------------------------------------------------
# QUAD_DECIMATE is the #1 speed knob AND the #1 enemy of altitude: it downsamples
# the image before quad detection, so anything above 1.0 shrinks how high a tag
# is still resolvable. Left at 1.0 ON PURPOSE for max range. Only raise it if you
# decide you'd trade detection altitude for a faster control loop.
QUAD_DECIMATE = 1.0
# A little blur denoises the image and actually HELPS decode small/distant tags.
# 0.0 = off. 0.8 was picked for the NOISIER NoIR sensor; with the IR-cut module
# the image is cleaner, so start LOWER (~0.4-0.5) and only raise it if distant
# tags decode better with more blur. Tune on-bench with BenchDryRun.
QUAD_SIGMA = 0.5
NTHREADS = 4                # Pi 5 has 4 cores; try 3 if the main loop starves

# ======================================================================
# COMMAND RATE
# ======================================================================
SEND_HZ = 10                # comfortably inside GUID_TIMEOUT (default 3 s)
SEND_PERIOD = 1.0 / SEND_HZ

# ======================================================================
# ATTITUDE-TARGET MESSAGE
# ======================================================================
# type_mask: ZERO -- we provide EVERYTHING (attitude quaternion, all three body
# rates, thrust). Same mask as SpinOnGuided, and for the same reason.
#
# It used to be `1 | 2` ("ignore roll+pitch rate, honour yaw rate"). ArduPilot
# REJECTS that. From ArduCopter/GCS_Mavlink.cpp on Copter-4.6.3 (this vehicle):
#
#     if (!roll_rate_ignore && !pitch_rate_ignore && !yaw_rate_ignore) {
#         ang_vel_body.x = packet.body_roll_rate;   ...
#     } else if (!(roll_rate_ignore && pitch_rate_ignore && yaw_rate_ignore)) {
#         // The body rates are ill-defined
#         copter.mode_guided.init(true);
#         return;
#     }
#
# The three body-rate ignore bits must be ALL SET or ALL CLEAR. A partial mask
# is discarded before it reaches the attitude controller -- and each discarded
# message re-initialises guided mode. Every command this script sent was being
# thrown away, so the centring/descent logic never actually drove the vehicle.
TYPE_MASK = 0

# In GUIDED_NOGPS the FC interprets thrust as a climb-rate command:
# 0.5 = hold altitude, >0.5 = climb, <0.5 = descend. (Changed by the
# GUID_OPTIONS "thrust as thrust" bit -- if you set that, revisit these.)
HOVER_THRUST = 0.50         # hold altitude while lining up / when tag lost
DESCEND_THRUST = 0.42       # gentle descent once centred + aligned

# ======================================================================
# HORIZONTAL POSITION CONTROLLER  (tag offset -> commanded tilt)
# ======================================================================
# The pupil_apriltags pose_t is [x, y, z] in the CAMERA frame:
#   x -> camera-right, y -> camera-down (image), z -> distance to tag (~altitude)
# We want x -> 0 and y -> 0 (tag centred under the camera). A tilt makes the
# quad accelerate in that direction, so this is a PD controller: P on the
# offset, D on how fast the offset is changing (damping, kills oscillation).
#
# SIGN CONVENTIONS depend on camera mounting -- FLIP THESE ON THE BENCH so the
# commanded tilt drives the quad toward the tag. Verify props-off first.
KP_TILT = 8.0               # deg of tilt per metre of offset
KD_TILT = 4.0               # deg of tilt per (metre/second) of offset change
MAX_TILT_DEG = 8.0          # clamp: never command more than this much roll/pitch

INVERT_ROLL = False         # flip if roll pushes AWAY from the tag in x
INVERT_PITCH = False        # flip if pitch pushes AWAY from the tag in y
SWAP_XY = False             # set True if camera-right/forward are swapped vs body

# --- Camera mounting offset (lever arm) -----------------------------------
# The controller below zeros the tag's (x, y) in the CAMERA frame, which parks
# the CAMERA over the tag. If the camera is not at the quad's centre, that
# leaves the quad centre offset from the tag by the camera's mounting lever arm.
# These two numbers move the target point so the QUAD CENTRE (not the camera)
# ends up over the tag. Because pose_t is in METRES, this is a fixed offset that
# is correct at every altitude -- no scaling needed.
#
# EASIEST WAY TO MEASURE (no sign/geometry reasoning needed): hand-hold the quad
# LEVEL with its centre directly above the tag centre, run BenchDryRun.py, and
# read the x and y it prints. Those readings ARE the values below -- paste them
# in. (They equal the negative of the camera's lever arm, but you don't need to
# work that out; whatever the bench reads when correctly parked is the setpoint.)
SETPOINT_X_M = 0.0          # camera-frame x where the tag sits when quad-centred
SETPOINT_Y_M = 0.0          # camera-frame y where the tag sits when quad-centred

# ======================================================================
# YAW ALIGNMENT  (face the FRONT of the tag)
# ======================================================================
# pose_R gives the tag's rotation; its yaw is the tag's rotation about the
# camera's vertical axis in the image plane. We spin the quad to drive that
# yaw error to zero so the quad's front lines up with the tag's front.
KP_YAW = 1.2                # rad/s of yaw rate per rad of yaw error
MAX_YAW_RATE_DEGS = 45.0
INVERT_YAW = False          # flip if yaw error grows instead of shrinks

# ======================================================================
# CENTRING / DESCENT / LANDING THRESHOLDS
# ======================================================================
CENTRE_TOL_M = 0.08         # |x|,|y| must be under this (m) to be "centred"
YAW_TOL_DEG = 8.0           # yaw error must be under this (deg) to be "aligned"

# --- Continuous-descent hand-off -------------------------------------------
# We centre + descend under vision for as long as we can SEE the tag. The tag
# disappears when the quad gets so low the tag overflows the camera frame -- and
# THAT "lost while very low" is the cue that we're on top of it and it's time to
# let the FC finish the touchdown. A tag lost while HIGH is treated as a spurious
# dropout instead: we hover and never blind-descend.
COMMIT_ALT_M = 0.50         # if the tag is lost while its last seen z was at or
                            # below this (m) AND we were centred, treat it as
                            # "too close to see" and hand off to FC LAND.
COMMIT_LOST_S = 0.4         # ...but only after the tag has been gone this long
                            # (debounce: a single dropped frame must NOT trigger
                            # LAND). Keep it short so touchdown isn't delayed.
MIN_TRACK_ALT_M = 0.20      # hard safety floor: if the tag is STILL visible but
                            # z drops to/below this while centred, stop pushing
                            # the vision descent lower and hand off to LAND.

# ======================================================================
# WATCHING WHAT THE CAMERA SEES
# ======================================================================
# Two independent ways to see the annotated view (tag outline, centre, pose):
#
#   SHOW_WINDOW   a live cv2 preview. Needs a display. Over a plain SSH session
#                 there is none, so display_available() turns this off by
#                 itself rather than letting cv2.imshow raise into the flight
#                 loop. `ssh -X` works but tunnels every frame -- it will slow
#                 the control loop down; prefer the recording.
#   RECORD_VIDEO  writes the SAME annotated frames to logs/<script>_<stamp>.mp4
#                 alongside the .log/.csv. This is the one to use for a real
#                 flight: nothing to watch live, and afterwards you can see
#                 exactly what the detector had to work with.
SHOW_WINDOW = True
RECORD_VIDEO = True
# CAM_RES (2304x1296) is far too big to encode inside the control loop, so the
# recording is downscaled. 960 keeps the tag and the overlay text readable.
VIDEO_WIDTH = 960
VIDEO_FPS = 10.0            # file timeline rate; the recorder pads/drops frames
                            # so playback stays real-time (see FlightVideo)


# ----------------------------------------------------------------------
# Calibration loading (numpy _core -> core shim, from AprilTagTesting_2)
# ----------------------------------------------------------------------
class NumpyCompatUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith("numpy._core"):
            module = module.replace("numpy._core", "numpy.core", 1)
        return super().find_class(module, name)


def load_compat_pickle(path):
    with open(path, "rb") as f:
        return NumpyCompatUnpickler(f).load()


def rotation_matrix_to_euler_angles(R):
    """Return (roll, pitch, yaw) in DEGREES from a 3x3 rotation matrix."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return np.degrees([roll, pitch, yaw])


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """(roll, pitch, yaw) in DEGREES -> quaternion (w, x, y, z)."""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y_, z]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ----------------------------------------------------------------------
# Camera / calibration / detector -- shared so LandOnAprilTag and the bench
# tools (BenchDryRun, BenchTiltProbe) all set the camera up identically.
# ----------------------------------------------------------------------
def load_calibration():
    """Load the calibration pickles and return (matrix, dist, camera_params)."""
    project_root = Path(__file__).resolve().parents[2]
    calibration_path = project_root / "resources" / "calibration"
    camera_matrix = load_compat_pickle(calibration_path / "cameraMatrix.pkl")
    dist_coeffs = load_compat_pickle(calibration_path / "dist.pkl")
    camera_params = [camera_matrix[0][0], camera_matrix[1][1],
                     camera_matrix[0][2], camera_matrix[1][2]]
    return camera_matrix, dist_coeffs, camera_params


def open_camera():
    """
    Start the Pi Camera Module 3 (IMX708) in a full-FOV binned mode.

    - raw = SENSOR_MODE selects the binned FULL-field-of-view readout (fast,
      low-noise), and main = CAM_RES is what we actually process. Keeping raw a
      native full-FOV mode is what keeps the lens calibration valid.
    - format RGB888 gives a 3-channel array so cv2.cvtColor(..., BGR2GRAY) works.
    - Camera Module 3 has phase-detect autofocus, and our subject distance
      changes hugely from high altitude down to touchdown, so we run CONTINUOUS
      autofocus rather than a fixed focus that would only be sharp at one height.
    - AeExposureMode=Short biases the auto-exposure toward short shutter times,
      which cuts motion blur while the quad is moving/descending -- important for
      detecting a tag from far away. This module has an IR-cut filter, so dim
      light hurts more than it did on the NoIR -- IR illumination will NOT help
      any more; if it underexposes, relax AeExposureMode to 0 (Normal) and accept
      more motion blur, or fly it in better light.
    """
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": CAM_RES, "format": "RGB888"},
        raw={"size": SENSOR_MODE},   # full-FOV binned readout; keep it native
    )
    picam2.configure(config)
    picam2.start()
    # Integer enum values (avoids importing libcamera):
    #   AfMode: 0=Manual 1=Auto 2=Continuous  | AfSpeed: 0=Normal 1=Fast
    #   AeExposureMode: 0=Normal 1=Short 2=Long
    picam2.set_controls({
        "AfMode": 2,           # continuous autofocus (tracks changing altitude)
        "AfSpeed": 1,          # fast
        "AeEnable": True,
        "AeExposureMode": 1,   # prefer short exposures -> less motion blur
    })
    return picam2


def build_detector():
    """The tag36h11 detector, configured identically everywhere."""
    return Detector(families="tag36h11", nthreads=NTHREADS,
                    quad_decimate=QUAD_DECIMATE, quad_sigma=QUAD_SIGMA,
                    refine_edges=1, decode_sharpening=0.25)


def build_undistort_maps(camera_matrix, dist_coeffs):
    """
    Precompute fixed-point remap tables ONCE for CAM_RES.

    cv2.undistort() rebuilds these maps on every single call, which is wasteful
    at these resolutions. We build them once here and reuse them with cv2.remap
    in grab_gray -- much cheaper per frame, which is what lets the Pi 5 run the
    detector at full resolution.
    """
    size = CAM_RES  # (w, h)
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, camera_matrix, size, cv2.CV_16SC2)
    return map1, map2


def grab_gray(picam2, map1, map2, want_color=True):
    """
    Capture one frame, undistort via the precomputed maps, return (bgr, gray).

    Gray is always undistorted (the detector needs it). The color frame is only
    remapped when want_color is True (i.e. there is an overlay to draw, for a
    preview window OR a recording); runs with neither skip that work. When
    want_color is False the first return value is None.
    """
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.remap(gray, map1, map2, cv2.INTER_LINEAR)
    color = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR) if want_color else None
    return color, gray


def display_available(log=None):
    """True if a cv2 preview window can actually be opened.

    Over a plain `ssh pi@...` there is no DISPLAY, and cv2.imshow does NOT
    degrade gracefully there -- it raises, and in these scripts that exception
    would come out of the main loop mid-flight. So check first: an SSH run
    quietly goes windowless instead, and RECORD_VIDEO is how you see what
    happened. (`ssh -X` sets DISPLAY, so it still gets a window.)
    """
    if not SHOW_WINDOW:
        return False
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return True
    if log is not None:
        log.event("No DISPLAY (SSH session?) -- preview window disabled. "
                  "The annotated view goes to the recording instead.")
    return False


def view_args(parser):
    """Add the --no-window / --no-video / video-size flags to a script.

    Shared so CenterOnAprilTag and LandOnAprilTag are driven the same way from
    the command line -- you should never have to remember which script wanted
    which flag while standing in a field with a live quad.
    """
    parser.add_argument("--no-window", action="store_true",
                        help="never open the cv2 preview window (default over "
                             "SSH, where there is no display anyway)")
    parser.add_argument("--no-video", action="store_true",
                        help="do not record the annotated view to logs/*.mp4")
    parser.add_argument("--video-width", type=int, default=VIDEO_WIDTH,
                        help=f"downscale recorded frames to this width "
                             f"(default {VIDEO_WIDTH})")
    parser.add_argument("--video-fps", type=float, default=VIDEO_FPS,
                        help=f"recording frame rate (default {VIDEO_FPS})")
    return parser


def open_view(args, log):
    """Resolve the flags + constants into (show_window, recorder).

    Returns the recorder (or None). `show_window or recorder is not None` is
    what decides whether the overlay is worth drawing at all.
    """
    show = (not args.no_window) and display_available(log)
    record = RECORD_VIDEO and not args.no_video
    video = (FlightVideo.open_video(log, fps=args.video_fps,
                                    width=args.video_width)
             if record else None)
    if not show and video is None:
        log.event("No preview window and no recording -- running blind "
                  "(logs only).")
    return show, video


def set_mode(mav, mode_name, log=None):
    """Request a flight-mode change on the FC (best effort)."""
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        msg = f"!! FC has no mode '{mode_name}' -- cannot hand off"
        # A failed LAND hand-off is the last thing you want to discover only
        # from a console, so this goes in the log too.
        (log.event if log is not None else print)(msg)
        return
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )


def send_attitude(mav, roll_deg, pitch_deg, target_yaw_deg, yaw_rate_rads, thrust):
    """Command absolute roll/pitch/heading + a yaw rate + a thrust value.

    target_yaw_deg is an ABSOLUTE heading in the earth frame, the same frame
    ATTITUDE.yaw reports in -- NOT a relative correction. It used to be
    hard-coded to 0.0, which does not mean "keep the current heading", it means
    "point north"; every command was asking the quad to swing round to north
    while body_yaw_rate simultaneously asked for a tag-alignment turn. That
    never showed up because the old type_mask meant the FC discarded the whole
    message (see TYPE_MASK). Callers now pass current heading + the correction.

    Roll and pitch rates are 0: they must be PRESENT (not masked off) for
    ArduPilot to accept the message, and 0 is what we want -- the quaternion,
    not a rate, is what holds the commanded tilt.
    """
    quat = euler_to_quaternion(roll_deg, pitch_deg, target_yaw_deg)
    mav.mav.set_attitude_target_send(
        0,                       # time_boot_ms (0 is fine)
        mav.target_system,
        mav.target_component,
        TYPE_MASK,
        quat,
        0.0,                     # body_roll_rate  -> hold commanded roll
        0.0,                     # body_pitch_rate -> hold commanded pitch
        yaw_rate_rads,           # body_yaw_rate   -> yaw alignment (feed-forward)
        thrust,                  # thrust -> climb rate (0.5 = hold alt)
    )


def request_message_interval(mav, msg_id, hz):
    """Ask the FC to stream msg_id at hz. Best effort; no ack is waited for."""
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def compute_command(offset_x, offset_y, offset_z, tag_yaw_deg, prev, dt):
    """
    Turn a tag pose into (roll_deg, pitch_deg, yaw_rate_rads, centred, aligned).

    prev is a dict carrying the previous (x, y) offsets for the D term; it is
    updated in place. dt is the loop period in seconds.
    """
    # Shift the target by the camera mounting offset so we park the QUAD CENTRE
    # (not the camera) over the tag. With the defaults (0, 0) this is a no-op.
    offset_x -= SETPOINT_X_M
    offset_y -= SETPOINT_Y_M

    # Optionally swap which camera axis maps to roll vs pitch.
    ox, oy = (offset_y, offset_x) if SWAP_XY else (offset_x, offset_y)

    # Derivative (rate of change of offset) for damping.
    if dt > 1e-3 and prev["t"] is not None:
        dx = (ox - prev["x"]) / dt
        dy = (oy - prev["y"]) / dt
    else:
        dx = dy = 0.0
    prev["x"], prev["y"], prev["t"] = ox, oy, time.time()

    roll_cmd = KP_TILT * ox + KD_TILT * dx
    pitch_cmd = KP_TILT * oy + KD_TILT * dy
    if INVERT_ROLL:
        roll_cmd = -roll_cmd
    if INVERT_PITCH:
        pitch_cmd = -pitch_cmd
    roll_cmd = clamp(roll_cmd, -MAX_TILT_DEG, MAX_TILT_DEG)
    pitch_cmd = clamp(pitch_cmd, -MAX_TILT_DEG, MAX_TILT_DEG)

    # Yaw: drive the tag's yaw error to zero so our front faces the tag front.
    yaw_err_deg = tag_yaw_deg
    yaw_rate = math.radians(KP_YAW * yaw_err_deg)
    if INVERT_YAW:
        yaw_rate = -yaw_rate
    max_yaw = math.radians(MAX_YAW_RATE_DEGS)
    yaw_rate = clamp(yaw_rate, -max_yaw, max_yaw)

    centred = (abs(offset_x) < CENTRE_TOL_M and abs(offset_y) < CENTRE_TOL_M)
    aligned = (abs(yaw_err_deg) < YAW_TOL_DEG)
    return roll_cmd, pitch_cmd, yaw_rate, centred, aligned


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Centre on an AprilTag, descend, and hand off to LAND.")
    return view_args(p).parse_args(argv)


def main(args=None):
    args = args if args is not None else parse_args()
    log = FlightLog.open_log("LandOnAprilTag")
    # Preview window and/or recording. Resolve BEFORE the config block so the
    # log records what this run could actually see.
    show_window, video = open_view(args, log)
    draw = show_window or (video is not None)
    # Record the tuning this flight actually used, so a log is self-describing
    # and two flights can be compared without guessing what changed.
    log.config(
        TYPE_MASK=TYPE_MASK, SEND_HZ=SEND_HZ,
        HOVER_THRUST=HOVER_THRUST, DESCEND_THRUST=DESCEND_THRUST,
        KP_TILT=KP_TILT, KD_TILT=KD_TILT, MAX_TILT_DEG=MAX_TILT_DEG,
        KP_YAW=KP_YAW, MAX_YAW_RATE_DEGS=MAX_YAW_RATE_DEGS,
        INVERT_ROLL=INVERT_ROLL, INVERT_PITCH=INVERT_PITCH,
        SWAP_XY=SWAP_XY, INVERT_YAW=INVERT_YAW,
        SETPOINT_X_M=SETPOINT_X_M, SETPOINT_Y_M=SETPOINT_Y_M,
        CENTRE_TOL_M=CENTRE_TOL_M, YAW_TOL_DEG=YAW_TOL_DEG,
        COMMIT_ALT_M=COMMIT_ALT_M, COMMIT_LOST_S=COMMIT_LOST_S,
        MIN_TRACK_ALT_M=MIN_TRACK_ALT_M,
        TAG_SIZE=TAG_SIZE, TARGET_TAG_ID=TARGET_TAG_ID,
        CAM_RES=CAM_RES, SENSOR_MODE=SENSOR_MODE,
        QUAD_DECIMATE=QUAD_DECIMATE, QUAD_SIGMA=QUAD_SIGMA,
        SHOW_WINDOW=show_window, RECORD_VIDEO=(video is not None),
    )

    # ---- MAVLink ----
    log.event(f"Connecting to {CONNECTION_STRING} @ {BAUD_RATE} ...")
    mav = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
    mav.wait_heartbeat()
    log.event(f"Heartbeat: system {mav.target_system}, "
              f"component {mav.target_component}")
    log.event(f"FC reports flight mode: '{mav.flightmode}' "
              f"(control triggers on '{TARGET_MODE}')")
    log.event("Flip channel-6 UP -> GUIDED_NOGPS -> approach+land starts.  "
              "DOWN/MIDDLE -> STABILIZE -> hands back to you.")
    log.event("Waiting to see a non-GUIDED_NOGPS mode once before control can "
              "trigger...")

    # We need ATTITUDE.yaw to build the ABSOLUTE heading target in send_attitude.
    # Ask for it at the command rate so the heading we build on is never stale.
    request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, SEND_HZ)

    # ---- Camera / calibration / detector (shared helpers) ----
    camera_matrix, dist_coeffs, camera_params = load_calibration()
    map1, map2 = build_undistort_maps(camera_matrix, dist_coeffs)
    picam2 = open_camera()
    detector = build_detector()

    # ---- State (same latch semantics as SpinOnGuided) ----
    active = False          # are we currently commanding the vehicle?
    armed = False           # seen a non-GUIDED_NOGPS mode since startup?
    warned_unarmed = False
    landing = False         # handed off to FC LAND mode
    last_send = 0.0
    last_seen = 0.0         # time we last had a valid tag
    last_tz = None          # z (altitude) at the last valid detection
    last_centred = False    # were we centred at the last valid detection?
    prev = {"x": 0.0, "y": 0.0, "t": None}

    # Vehicle heading (earth frame, radians) from ATTITUDE. Every attitude
    # command is built relative to this, so we must not command anything until
    # we have actually received one.
    heading_rads = None
    warned_no_heading = False
    # Measured attitude, logged next to what we COMMANDED. If commanded roll
    # swings and measured roll never follows, the FC is not acting on us.
    meas_roll = meas_pitch = meas_yawspeed = 0.0
    alt_m = float('nan')

    try:
        while True:
            # Drain MAVLink so mav.flightmode stays current (side effect of
            # parsing HEARTBEAT) and keep the latest heading. Don't block --
            # keep the vision loop responsive.
            while True:
                msg = mav.recv_match(blocking=False)
                if msg is None:
                    break
                mtype = msg.get_type()
                if mtype == 'ATTITUDE':
                    heading_rads = msg.yaw
                    meas_roll = msg.roll
                    meas_pitch = msg.pitch
                    meas_yawspeed = msg.yawspeed
                elif mtype == 'VFR_HUD':
                    alt_m = msg.alt

            in_target = (mav.flightmode == TARGET_MODE)

            if not in_target:
                if not armed:
                    armed = True
                    log.event(f">>> Saw '{mav.flightmode}' (switch down/"
                              f"middle) -- armed. A later {TARGET_MODE} will "
                              f"now trigger approach+land.")
                if active:
                    active = False
                    landing = False
                    prev["t"] = None
                    log.event(f">>> Mode is {mav.flightmode} -- stopping "
                              f"(FC ignores our commands outside "
                              f"{TARGET_MODE})")
            else:
                if armed and not active:
                    active = True
                    log.event(f">>> Mode is {TARGET_MODE} and armed -- "
                              f"searching for tag, then centre / align / "
                              f"land.")
                elif not armed and not warned_unarmed:
                    warned_unarmed = True
                    log.event(f">>> In {TARGET_MODE} but NOT armed (started "
                              f"with the switch already up). Flip to "
                              f"STABILIZE, then back, to trigger.")

            # ---- Vision: grab a frame and look for the tag ----
            frame, gray = grab_gray(picam2, map1, map2, want_color=draw)
            results = detector.detect(gray, estimate_tag_pose=True,
                                      camera_params=camera_params, tag_size=TAG_SIZE)

            # Pick the target tag (specific id if configured, else the first).
            tag = None
            for r in results:
                if TARGET_TAG_ID is None or r.tag_id == TARGET_TAG_ID:
                    tag = r
                    break

            now = time.time()
            do_send = active and (now - last_send) >= SEND_PERIOD

            # No heading yet -> we cannot build an absolute yaw target, and
            # guessing would command a swing to north. Refuse to send.
            if do_send and heading_rads is None:
                do_send = False
                if not warned_no_heading:
                    warned_no_heading = True
                    log.event("!! No ATTITUDE from the FC yet -- holding off "
                              "on commands (cannot build a heading target). "
                              "Check the FC is streaming ATTITUDE.")

            if tag is not None:
                tx, ty, tz = tag.pose_t.flatten()
                _, _, tag_yaw = rotation_matrix_to_euler_angles(tag.pose_R)
                last_seen = now
                last_tz = tz

                if do_send and not landing:
                    dt = SEND_PERIOD if prev["t"] is None else (now - last_send)
                    roll_c, pitch_c, yaw_rate, centred, aligned = compute_command(
                        tx, ty, tz, tag_yaw, prev, dt)
                    last_centred = centred

                    ready_to_descend = centred and aligned
                    if centred and tz <= MIN_TRACK_ALT_M:
                        # Safety floor: still SEE the tag, but we're this low and
                        # centred -- don't push the vision descent any lower, let
                        # the FC land it. (Normally we'd lose the tag before here.)
                        landing = True
                        log.event(f">>> At z={tz:.2f} m (<= "
                                  f"{MIN_TRACK_ALT_M} floor) and centred -- "
                                  f"handing off to {LAND_MODE} for touchdown.")
                        set_mode(mav, LAND_MODE, log)
                    else:
                        # Keep centring + descending. Descend only while centred+
                        # aligned so we track the tag DOWN rather than sliding off
                        # it; otherwise hold altitude and correct first.
                        thrust = DESCEND_THRUST if ready_to_descend else HOVER_THRUST
                        # Absolute heading target = where we point now, plus the
                        # correction this cycle's yaw rate will achieve. Seeding
                        # from the MEASURED heading every cycle keeps the target
                        # from running away if the vehicle can't keep up, and
                        # rate-limits the turn by construction (yaw_rate is
                        # already clamped to MAX_YAW_RATE_DEGS in
                        # compute_command). The yaw rate rides along as
                        # feed-forward so the controller leads the turn.
                        target_yaw_deg = math.degrees(
                            heading_rads + yaw_rate * dt)
                        send_attitude(mav, roll_c, pitch_c, target_yaw_deg,
                                      yaw_rate, thrust)
                        phase = "DESCEND" if ready_to_descend else "ALIGN"
                        log.event(f"[{phase}] x={tx:+.2f} y={ty:+.2f} "
                                  f"z={tz:.2f}m yawErr={tag_yaw:+.1f} -> "
                                  f"roll={roll_c:+.1f} pitch={pitch_c:+.1f} "
                                  f"yawRate={math.degrees(yaw_rate):+.0f} "
                                  f"thr={thrust:.2f}")
                        log.row(
                            mode=mav.flightmode, active=1, phase=phase,
                            tag=1, tag_id=tag.tag_id,
                            x=round(tx, 4), y=round(ty, 4), z=round(tz, 4),
                            yaw_err_deg=round(tag_yaw, 2),
                            centred=int(centred), aligned=int(aligned),
                            roll_cmd_deg=round(roll_c, 2),
                            pitch_cmd_deg=round(pitch_c, 2),
                            target_yaw_deg=round(target_yaw_deg % 360, 2),
                            cmd_yaw_rate_degs=round(math.degrees(yaw_rate), 2),
                            thrust=thrust,
                            heading_deg=round(math.degrees(heading_rads) % 360, 2),
                            meas_yaw_rate_degs=round(math.degrees(meas_yawspeed), 2),
                            roll_meas_deg=round(math.degrees(meas_roll), 2),
                            pitch_meas_deg=round(math.degrees(meas_pitch), 2),
                            alt_m=round(alt_m, 2) if alt_m == alt_m else "",
                        )
                    last_send = now

                if draw:
                    corners = tag.corners.astype(int)
                    for i in range(4):
                        cv2.line(frame, tuple(corners[i]),
                                 tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
                    c = tuple(tag.center.astype(int))
                    cv2.circle(frame, c, 5, (0, 0, 255), -1)
                    FlightVideo.draw_crosshair(frame)
                    cv2.arrowedLine(frame,
                                    (frame.shape[1] // 2, frame.shape[0] // 2),
                                    c, (0, 255, 255), 2, tipLength=0.2)
                    cv2.putText(frame, f"ID{tag.tag_id} x={tx:+.2f} y={ty:+.2f} z={tz:.2f}",
                                (c[0] + 10, c[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 0, 0), 2)

            else:
                # No tag this frame.
                if do_send and not landing:
                    lost_for = now - last_seen
                    close_when_lost = (last_tz is not None
                                       and last_tz <= COMMIT_ALT_M
                                       and last_centred)
                    if close_when_lost and lost_for >= COMMIT_LOST_S:
                        # Lost the tag while low AND centred -> we're on top of it
                        # (it overflowed the frame). Commit the touchdown to LAND.
                        landing = True
                        log.event(f">>> Tag lost {lost_for:.1f}s at low "
                                  f"z~{last_tz:.2f} m, was centred -- too "
                                  f"close to see it; handing off to "
                                  f"{LAND_MODE} for touchdown.")
                        set_mode(mav, LAND_MODE, log)
                    else:
                        # Lost while high (or not long enough, or off-centre) ->
                        # hold level + altitude, NEVER blind-descend. Pilot can
                        # flip channel-6 DOWN to take over. Heading target is
                        # our CURRENT heading (hold it) -- not 0, which would
                        # command a swing to north.
                        send_attitude(mav, 0.0, 0.0, math.degrees(heading_rads),
                                      0.0, HOVER_THRUST)
                        z_txt = f"{last_tz:.2f}" if last_tz is not None else "?"
                        log.event(f"[HOLD] tag lost {lost_for:.1f}s (last "
                                  f"z~{z_txt} m) -- hovering level")
                        log.row(
                            mode=mav.flightmode, active=1, phase="HOLD",
                            tag=0,
                            roll_cmd_deg=0.0, pitch_cmd_deg=0.0,
                            target_yaw_deg=round(math.degrees(heading_rads) % 360, 2),
                            cmd_yaw_rate_degs=0.0, thrust=HOVER_THRUST,
                            heading_deg=round(math.degrees(heading_rads) % 360, 2),
                            meas_yaw_rate_degs=round(math.degrees(meas_yawspeed), 2),
                            roll_meas_deg=round(math.degrees(meas_roll), 2),
                            pitch_meas_deg=round(math.degrees(meas_pitch), 2),
                            alt_m=round(alt_m, 2) if alt_m == alt_m else "",
                        )
                    prev["t"] = None
                    last_send = now

            if draw:
                # The `t=` here is the SAME monotonic clock as the CSV's `t`
                # column, so a moment in the video can be looked up in the
                # numbers (and vice versa).
                status = ("LAND (FC)" if landing else
                          "ACTIVE" if active else
                          "waiting for GUIDED_NOGPS" if armed else "not armed")
                if tag is not None:
                    tag_line = (f"ID{tag.tag_id}  x={tx:+.2f} y={ty:+.2f} "
                                f"z={tz:.2f}m")
                else:
                    tag_line = ("NO TAG" if not last_seen else
                                f"NO TAG ({now - last_seen:.1f}s)")
                FlightVideo.draw_status(frame, [
                    f"LandOnAprilTag  t={log.elapsed():6.1f}s",
                    f"mode={mav.flightmode}  {status}",
                    tag_line,
                    f"alt={alt_m:.2f}m" if alt_m == alt_m else "alt=?",
                ])
                if video is not None:
                    video.write(frame)
                if show_window:
                    cv2.imshow("Land-on-AprilTag", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

            time.sleep(0.005)

    except KeyboardInterrupt:
        log.event(">>> Ctrl-C -- stopping.")
    except Exception:
        # A crash mid-flight is exactly the log you most want to read, so make
        # sure the traceback reaches the file before we re-raise.
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
