"""
Camera calibration for the AprilTag pipeline  --  makes cameraMatrix.pkl / dist.pkl.

This is checklist item #1: everything downstream (LandOnAprilTag, CenterOnAprilTag,
BenchDryRun, BenchTiltProbe, AprilTagTesting*) calls `L.load_calibration()`, which
reads two pickles out of `resources/calibration/`:

    cameraMatrix.pkl   3x3 float64 K = [[fx,0,cx],[0,fy,cy],[0,0,1]]
    dist.pkl           lens distortion coefficients (k1 k2 p1 p2 k3 ...)

fx/fy/cx/cy become the `camera_params` handed to pupil_apriltags, so THEY are what
turn a tag in the image into metres of x/y/z. A bad calibration doesn't stop
detection -- it quietly biases the pose, which biases both the centering PD loop and
the altitude gates (COMMIT_ALT_M / MIN_TRACK_ALT_M) that decide when to hand off to
LAND. Get this right first.

  !!  Calibrate at EXACTLY the resolution the detector runs at.  !!
  Intrinsics are in PIXELS, so they are only valid for the resolution they were
  measured at. This script reads CAM_RES / SENSOR_MODE straight from
  LandOnAprilTag.py and opens the camera with its own `open_camera()`, so the
  calibration always matches the flight config. Change CAM_RES -> rerun this.

Autofocus caveat (Camera Module 3)
----------------------------------
The IMX708 focus motor moves the lens, and moving the lens changes the focal
length slightly. LandOnAprilTag flies with CONTINUOUS autofocus (subject distance
goes from altitude to touchdown), so the honest thing is to calibrate the same
way, over a RANGE of board distances -- you get intrinsics averaged across the
focus travel, which is what the flight actually experiences. That is the default
here. If you instead decide to LOCK focus in flight, calibrate with the same lock:
`--lens-position 0.0` (dioptres, 0 = infinity) and set AfMode 0 + the same
LensPosition in LandOnAprilTag.open_camera().

Usage
-----
    # 0. print a target (SVG, exact mm -- print at 100% / "actual size", no fit-to-page)
    python CalibrateCamera.py board

    # 1. capture + solve in one go (the normal path)
    python CalibrateCamera.py all

    # just one stage:
    python CalibrateCamera.py capture          # live preview, saves frames
    python CalibrateCamera.py solve            # solves from the saved frames
    python CalibrateCamera.py solve --from-dir /some/other/folder
    python CalibrateCamera.py verify           # live raw-vs-undistorted A/B check

Keys during `capture`
---------------------
  SPACE  grab this frame          a  toggle auto-capture (on by default)
  u      undo the last grab       d  done -> solve       q  abort (no solve)

How to move the board (this is what makes or breaks the result)
---------------------------------------------------------------
Tape the printed board to something FLAT and rigid, then fill every corner of the
frame with it, at several distances, and TILT it (~30-45 deg) in different
directions. Tilt is what separates focal length from distance -- a stack of
head-on shots gives a confident, wrong answer. The overlay shows a 3x3 coverage
grid and a near/mid/far counter; keep going until nothing reads 0.
"""

import argparse
import json
import pickle
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

# Import the flight code so the camera is opened with the IDENTICAL resolution and
# sensor mode it will fly with (single source of truth -- see BenchDryRun). On a
# non-Pi machine this import fails on picamera2/pymavlink, which is fine as long as
# you are only running `board` or `solve --from-dir`.
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    import LandOnAprilTag as L
    CAM_RES = L.CAM_RES
except Exception as _import_err:            # noqa: BLE001 - want the message, not the type
    L = None
    CAM_RES = (2304, 1296)                  # must match LandOnAprilTag.CAM_RES
    print(f"[warn] could not import LandOnAprilTag ({_import_err}).")
    print(f"[warn] camera capture is unavailable; assuming CAM_RES={CAM_RES}.")

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CALIB_DIR = PROJECT_ROOT / "resources" / "calibration"
CAPTURE_DIR = CALIB_DIR / "captures"
BACKUP_DIR = CALIB_DIR / "backup"

# --- Board defaults -------------------------------------------------------
# "9x6" counts INNER CORNERS (where 4 squares meet), not squares -- a 9x6 board is
# 10x7 printed squares. An asymmetric board (cols != rows) is required: it removes
# the 180-degree ambiguity in the corner ordering.
BOARD_COLS, BOARD_ROWS = 9, 6
SQUARE_MM = 20.0            # 9x6 @ 20 mm = 240x180 mm page -> fits A4/Letter LANDSCAPE.
                            # Print bigger (A3, --square-mm 30) if you can: a larger
                            # board is easier to hold flat and to fill the frame with.
# NOTE: square size sets the world scale, so it affects the (discarded) per-image
# board poses -- NOT fx/fy/cx/cy or the distortion coefficients, which are what we
# save. So a printer that scales the page slightly does not corrupt your intrinsics.

# --- Capture defaults -----------------------------------------------------
TARGET_SHOTS = 20           # 20 good, varied views is plenty; below ~12 gets shaky
DETECT_WIDTH = 960          # find corners for the LIVE overlay at this width (speed);
                            # the solve always re-detects on the full-res saved file
PREVIEW_WIDTH = 1280        # window size cap
MIN_GAP_S = 0.8             # seconds between auto-captures
MOVE_FRAC = 0.06            # board must move this fraction of frame width to re-grab
AREA_CHANGE = 0.25          # ...or change apparent area by this much (new distance)
BLUR_MIN = 55.0             # variance-of-Laplacian floor; rejects motion-blurred frames

# --- Solve defaults -------------------------------------------------------
MAX_RMS_PX = 1.0            # overall reprojection error we consider acceptable
PRUNE_ERR_PX = 1.0          # drop individual views worse than this, then re-solve


# ----------------------------------------------------------------------
# Board generation
# ----------------------------------------------------------------------
def write_board_svg(path, cols, rows, square_mm):
    """
    Write a printable chessboard as SVG with real mm units.

    SVG (not PNG) because the physical square size has to survive printing: an SVG
    with mm units prints at exactly that size when you disable "fit to page" /
    "shrink oversized pages". A margin of >= 1 square of white is left around the
    board -- the corner finder needs that quiet zone.
    """
    sq_x, sq_y = cols + 1, rows + 1          # printed squares from inner corners
    margin = square_mm
    w = sq_x * square_mm + 2 * margin
    h = sq_y * square_mm + 2 * margin

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}mm" height="{h}mm" '
        f'viewBox="0 0 {w} {h}">',
        f'<rect x="0" y="0" width="{w}" height="{h}" fill="#ffffff"/>',
    ]
    for j in range(sq_y):
        for i in range(sq_x):
            if (i + j) % 2 == 0:
                x = margin + i * square_mm
                y = margin + j * square_mm
                parts.append(f'<rect x="{x}" y="{y}" width="{square_mm}" '
                             f'height="{square_mm}" fill="#000000"/>')
    label = (f'{cols}x{rows} inner corners  |  {square_mm:g} mm squares  |  '
             f'PRINT AT 100% (actual size), then MEASURE a square')
    parts.append(f'<text x="{margin}" y="{h - margin / 2.5}" font-family="sans-serif" '
                 f'font-size="{max(3.0, square_mm / 6):.1f}" fill="#000000">{label}</text>')
    parts.append('</svg>')

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts), encoding="utf-8")

    print(f"Wrote {path}")
    print(f"  board  : {cols}x{rows} inner corners ({sq_x}x{sq_y} squares)")
    print(f"  size   : {sq_x * square_mm:.0f} x {sq_y * square_mm:.0f} mm "
          f"(page {w:.0f} x {h:.0f} mm{'' if max(w, h) <= 297 and min(w, h) <= 210 else ' -- LARGER THAN A4/Letter, print on A3 or lower --square-mm'})")
    print("  print  : 100% / actual size, NO fit-to-page. Then measure one square")
    print("           with a ruler and pass the real number as --square-mm.")
    print("  mount  : tape it FLAT to rigid board/glass. Any bow = bad calibration.")


# ----------------------------------------------------------------------
# Corner detection
# ----------------------------------------------------------------------
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)


def find_corners(gray, pattern, fast=False):
    """
    Locate the inner corners. Returns Nx1x2 float32 array, or None.

    findChessboardCornersSB (OpenCV >= 4.0) is markedly better than the classic
    detector on blur, tilt and uneven lighting, and it already returns sub-pixel
    positions. We use it for the real solve; `fast` uses the cheap classic path
    with FAST_CHECK for the live overlay, where we only need "is a board roughly
    there and where", not accuracy.
    """
    if fast:
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
                 | cv2.CALIB_CB_FAST_CHECK)
        ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
        return corners if ok else None

    sb = getattr(cv2, "findChessboardCornersSB", None)
    if sb is not None:
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        try:
            ok, corners = sb(gray, pattern, flags=flags)
        except cv2.error:
            ok, corners = False, None
        if ok:
            return corners

    ok, corners = cv2.findChessboardCorners(
        gray, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)


def sharpness(gray):
    """Variance of the Laplacian -- the standard cheap focus/motion-blur score."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


# ----------------------------------------------------------------------
# Capture stage
# ----------------------------------------------------------------------
class Coverage:
    """
    Tracks WHERE in the frame and at WHAT distance we have seen the board.

    Distortion coefficients are only constrained where you actually put the board,
    and the edges/corners are exactly where distortion is largest -- so a pile of
    centred shots leaves k1/k2 guessing. This turns "did I cover it?" into a
    number on screen.
    """

    def __init__(self, frame_w, frame_h):
        self.w, self.h = frame_w, frame_h
        self.grid = np.zeros((3, 3), dtype=int)     # board centre, 3x3 image cells
        self.dist = {"near": 0, "mid": 0, "far": 0}  # apparent board area buckets

    @staticmethod
    def _bucket(area_frac):
        if area_frac > 0.25:
            return "near"
        if area_frac > 0.07:
            return "mid"
        return "far"

    def add(self, corners):
        pts = corners.reshape(-1, 2)
        cx, cy = pts.mean(axis=0)
        col = min(2, max(0, int(cx / self.w * 3)))
        row = min(2, max(0, int(cy / self.h * 3)))
        self.grid[row, col] += 1
        self.dist[self._bucket(self._area_frac(pts))] += 1

    def _area_frac(self, pts):
        hull = cv2.convexHull(pts.astype(np.float32))
        return float(cv2.contourArea(hull)) / (self.w * self.h)

    def bucket_of(self, corners):
        return self._bucket(self._area_frac(corners.reshape(-1, 2)))

    def holes(self):
        n = int((self.grid == 0).sum())
        missing = [k for k, v in self.dist.items() if v == 0]
        return n, missing

    def summary(self):
        cells, missing = self.holes()
        d = "/".join(f"{k}:{v}" for k, v in self.dist.items())
        return f"empty cells: {cells}/9   distance {d}"


def draw_overlay(view, scale, corners, pattern, cov, shots, target, auto, blur, msg):
    """Annotate the preview: corners, coverage grid, counters, status."""
    h, w = view.shape[:2]

    # 3x3 coverage grid -- a cell we already have is tinted green, empty stays red.
    for r in range(3):
        for c in range(3):
            x0, y0 = int(c * w / 3), int(r * h / 3)
            x1, y1 = int((c + 1) * w / 3), int((r + 1) * h / 3)
            got = cov.grid[r, c] > 0
            cv2.rectangle(view, (x0, y0), (x1, y1),
                          (0, 180, 0) if got else (0, 0, 200), 1)
            cv2.putText(view, str(cov.grid[r, c]), (x0 + 6, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 180, 0) if got else (0, 0, 200), 1)

    if corners is not None:
        cv2.drawChessboardCorners(view, pattern, corners * scale, True)

    bar = [
        f"shots {shots}/{target}",
        f"auto {'ON' if auto else 'OFF'}",
        f"blur {blur:.0f}" + ("" if blur >= BLUR_MIN else " LOW!"),
        cov.summary(),
    ]
    cv2.rectangle(view, (0, 0), (w, 26), (0, 0, 0), -1)
    cv2.putText(view, "   ".join(bar), (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(view, "SPACE grab   a auto   u undo   d done   q abort",
                (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    if msg:
        cv2.putText(view, msg, (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2)
    return view


def open_source(args):
    """
    Return (grab_fn, close_fn) for the frame source.

    The Pi path deliberately reuses LandOnAprilTag.open_camera() so resolution,
    sensor mode and AE settings are identical to flight. We grab the RAW array --
    NOT L.grab_gray(), which undistorts with the calibration we are trying to
    create.
    """
    if args.usb is not None:
        print(f"[warn] --usb {args.usb}: calibrating a DIFFERENT camera than the "
              f"flight one only makes sense for testing this script.")
        cap = cv2.VideoCapture(args.usb)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_RES[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_RES[1])
        if not cap.isOpened():
            sys.exit(f"could not open USB camera {args.usb}")

        def grab():
            ok, frame = cap.read()
            return frame if ok else None

        return grab, cap.release

    if L is None:
        sys.exit("LandOnAprilTag import failed -- cannot open the Pi camera. "
                 "Run this on the Pi, or use `solve --from-dir`.")

    picam2 = L.open_camera()
    if args.lens_position is not None:
        # AfMode 0 = Manual; LensPosition is in dioptres (1/metres), 0 = infinity.
        picam2.set_controls({"AfMode": 0, "LensPosition": args.lens_position})
        print(f"focus LOCKED at LensPosition={args.lens_position} dioptres "
              f"-- LandOnAprilTag.open_camera() must lock to the same value.")
    else:
        print("focus: CONTINUOUS autofocus (same as flight). Capture the board at "
              "several distances so the intrinsics average over the focus travel.")
    time.sleep(1.0)   # let AE/AWB settle before the first frame

    def close():
        picam2.stop()
        picam2.close()

    return picam2.capture_array, close


def capture(args):
    """Live capture loop -> writes full-resolution PNGs into CAPTURE_DIR."""
    pattern = (args.cols, args.rows)
    out_dir = Path(args.out_dir)
    if out_dir.exists() and any(out_dir.glob("*.png")):
        if args.keep:
            print(f"keeping existing frames in {out_dir}")
        else:
            n = len(list(out_dir.glob("*.png")))
            print(f"clearing {n} old frame(s) from {out_dir} (pass --keep to add to them)")
            for f in out_dir.glob("*.png"):
                f.unlink()
    out_dir.mkdir(parents=True, exist_ok=True)

    grab, close = open_source(args)
    cov = None
    shots = []
    auto = not args.no_auto
    last_t = 0.0
    last_pts = None
    last_area = None
    msg, msg_until = "", 0.0

    print(f"\ncapturing to {out_dir}  (target {args.shots} shots)")
    print("fill the FRAME EDGES, vary DISTANCE, and TILT the board ~30-45 deg.\n")

    try:
        while True:
            frame = grab()
            if frame is None:
                print("no frame from camera")
                break
            fh, fw = frame.shape[:2]
            if cov is None:
                cov = Coverage(fw, fh)
                if (fw, fh) != tuple(CAM_RES):
                    print(f"[warn] camera returned {fw}x{fh} but CAM_RES is "
                          f"{CAM_RES[0]}x{CAM_RES[1]} -- the calibration will only "
                          f"be valid for {fw}x{fh}.")

            scale = fw / float(DETECT_WIDTH) if fw > DETECT_WIDTH else 1.0
            small = cv2.resize(frame, (int(fw / scale), int(fh / scale))) if scale != 1.0 else frame
            gray_s = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            corners_s = find_corners(gray_s, pattern, fast=True)
            blur = sharpness(gray_s)

            take = False
            if corners_s is not None:
                pts = corners_s.reshape(-1, 2)
                centre = pts.mean(axis=0)
                area = cv2.contourArea(cv2.convexHull(pts.astype(np.float32)))
                now = time.time()
                if auto and blur >= args.blur_min and (now - last_t) >= MIN_GAP_S:
                    if last_pts is None:
                        take = True
                    else:
                        moved = float(np.linalg.norm(centre - last_pts))
                        grew = abs(area - last_area) / max(area, last_area, 1.0)
                        # Require a genuinely NEW view: either the board moved across
                        # the frame or its apparent size changed (new distance).
                        take = (moved > MOVE_FRAC * small.shape[1]) or (grew > AREA_CHANGE)
                if take:
                    last_t, last_pts, last_area = now, centre, area

            key = -1
            if not args.no_preview:
                pscale = PREVIEW_WIDTH / float(fw) if fw > PREVIEW_WIDTH else 1.0
                view = cv2.resize(frame, (int(fw * pscale), int(fh * pscale))) \
                    if pscale != 1.0 else frame.copy()
                draw_overlay(view, pscale * scale, corners_s, pattern, cov,
                             len(shots), args.shots, auto, blur,
                             msg if time.time() < msg_until else "")
                cv2.imshow("calibration capture", view)
                key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                if corners_s is None:
                    msg, msg_until = "no board visible", time.time() + 1.0
                else:
                    take = True
            elif key == ord('a'):
                auto = not auto
            elif key == ord('u') and shots:
                gone = shots.pop()
                Path(gone).unlink(missing_ok=True)
                msg, msg_until = f"removed {Path(gone).name}", time.time() + 1.5
                print(f"undo -> {gone}")
            elif key == ord('d'):
                break
            elif key == ord('q'):
                close()
                cv2.destroyAllWindows()
                print("aborted -- nothing solved.")
                return None

            if take:
                if blur < args.blur_min:
                    msg, msg_until = "too blurry -- hold still", time.time() + 1.0
                else:
                    path = out_dir / f"cal_{len(shots):03d}.png"
                    cv2.imwrite(str(path), frame)
                    shots.append(str(path))
                    cov.add(corners_s * scale)
                    bucket = cov.bucket_of(corners_s * scale)
                    msg, msg_until = f"grabbed {path.name}", time.time() + 0.8
                    print(f"[{len(shots):2d}/{args.shots}] {path.name}  "
                          f"{bucket:4s}  blur {blur:5.0f}  {cov.summary()}")

            if len(shots) >= args.shots:
                cells, missing = cov.holes()
                if (cells == 0 and not missing) or args.no_preview:
                    # Headless has no 'd' key, so the count is the only stop signal.
                    print("\ntarget reached." if not (cells or missing)
                          else f"\ntarget reached ({cov.summary()}).")
                    break
                print(f"\ntarget count reached but coverage is thin "
                      f"({cov.summary()}) -- keep going, or press 'd' to solve anyway.")
                args.shots = len(shots) + 5
    finally:
        close()
        if not args.no_preview:
            cv2.destroyAllWindows()

    if not shots:
        print("no frames captured.")
        return None
    cells, missing = cov.holes()
    if cells or missing:
        print(f"[warn] incomplete coverage -- {cov.summary()}"
              + (f" (no {'/'.join(missing)} shots)" if missing else ""))
    print(f"\ncaptured {len(shots)} frame(s) in {out_dir}")
    return out_dir


# ----------------------------------------------------------------------
# Solve stage
# ----------------------------------------------------------------------
def object_points(cols, rows, square_mm):
    """One board's 3D corner grid, Z=0, in metres."""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return objp * (square_mm / 1000.0)


def per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, D):
    """RMS reprojection error per image, in pixels."""
    errs = []
    for objp, imgp, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(objp, rv, tv, K, D)
        # reshape both to Nx2 -- the two corner finders disagree on channel layout,
        # and cv2.norm refuses to compare CV_32FC1 against CV_32FC2.
        d = imgp.reshape(-1, 2).astype(np.float64) - proj.reshape(-1, 2)
        errs.append(float(np.sqrt((d ** 2).sum(axis=1).mean())))
    return errs


def solve(args, image_dir):
    pattern = (args.cols, args.rows)
    files = sorted(p for p in Path(image_dir).iterdir()
                   if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
    if not files:
        sys.exit(f"no images in {image_dir}")

    print(f"\ndetecting corners in {len(files)} image(s) at FULL resolution "
          f"(slow but accurate)...")
    objp = object_points(args.cols, args.rows, args.square_mm)
    objpoints, imgpoints, used, size = [], [], [], None
    for i, f in enumerate(files, 1):
        img = cv2.imread(str(f))
        if img is None:
            print(f"  [{i:2d}/{len(files)}] {f.name}: unreadable, skipped")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if size is None:
            size = gray.shape[::-1]                 # (w, h)
        elif gray.shape[::-1] != size:
            print(f"  [{i:2d}/{len(files)}] {f.name}: {gray.shape[::-1]} != {size}, skipped")
            continue
        corners = find_corners(gray, pattern)
        if corners is None:
            print(f"  [{i:2d}/{len(files)}] {f.name}: no board found, skipped")
            continue
        objpoints.append(objp)
        imgpoints.append(corners)
        used.append(f)
        print(f"  [{i:2d}/{len(files)}] {f.name}: ok")

    if len(objpoints) < 6:
        sys.exit(f"only {len(objpoints)} usable view(s) -- need at least 6 "
                 f"(aim for {TARGET_SHOTS}). Recapture.")
    if tuple(size) != tuple(CAM_RES):
        print(f"\n[warn] images are {size[0]}x{size[1]} but LandOnAprilTag.CAM_RES is "
              f"{CAM_RES[0]}x{CAM_RES[1]}. The saved intrinsics are ONLY valid at "
              f"{size[0]}x{size[1]}.")

    # CALIB_RATIONAL_MODEL adds k4..k6 -- worth it for wide/fisheye-ish lenses where
    # the 5-coefficient model can't follow the edge distortion. Plain 5-coeff is the
    # right default for the standard Camera Module 3.
    flags = cv2.CALIB_RATIONAL_MODEL if args.rational else 0

    def run(objs, imgs):
        return cv2.calibrateCamera(objs, imgs, size, None, None, flags=flags)

    print(f"\nsolving with {len(objpoints)} view(s)"
          + (" (rational 8-coeff model)" if args.rational else "") + "...")
    rms, K, D, rvecs, tvecs = run(objpoints, imgpoints)
    errs = per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, D)
    print(f"  overall RMS reprojection error: {rms:.4f} px")

    # A single bad view (board bowed, corner mis-ordered, motion blur) drags the
    # whole fit. Drop the outliers and solve again on what's left.
    if not args.no_prune:
        keep = [i for i, e in enumerate(errs) if e <= args.prune_err]
        dropped = [(used[i].name, errs[i]) for i in range(len(errs)) if i not in keep]
        if dropped and len(keep) >= 6:
            for name, e in dropped:
                print(f"  pruning {name}: {e:.3f} px > {args.prune_err} px")
            objpoints = [objpoints[i] for i in keep]
            imgpoints = [imgpoints[i] for i in keep]
            used = [used[i] for i in keep]
            rms, K, D, rvecs, tvecs = run(objpoints, imgpoints)
            errs = per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, D)
            print(f"  RMS after pruning ({len(used)} views): {rms:.4f} px")
        elif dropped:
            print(f"  [warn] {len(dropped)} view(s) over {args.prune_err} px but too "
                  f"few would remain -- keeping everything.")

    report(rms, K, D, errs, used, size, args)
    save(K, D, rms, size, used, args)
    return K, D


def report(rms, K, D, errs, used, size, args):
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    w, h = size
    # Field of view straight from the pinhole model (no sensor size needed).
    fovx = np.degrees(2 * np.arctan(w / (2 * fx)))
    fovy = np.degrees(2 * np.arctan(h / (2 * fy)))

    print("\n" + "=" * 62)
    print(f"  resolution : {w} x {h}")
    print(f"  fx, fy     : {fx:9.3f}  {fy:9.3f}")
    print(f"  cx, cy     : {cx:9.3f}  {cy:9.3f}   (frame centre {w/2:.1f}, {h/2:.1f})")
    print(f"  FOV        : {fovx:.1f} deg horizontal, {fovy:.1f} deg vertical")
    print(f"  dist       : {np.array2string(D.ravel(), precision=5, suppress_small=True)}")
    print(f"  views      : {len(used)}   RMS {rms:.4f} px   "
          f"worst {max(errs):.3f} px ({used[int(np.argmax(errs))].name})")
    print("=" * 62)

    # Sanity checks. These catch the failure modes that still "converge" to numbers
    # that look plausible until the pose is 30% off in flight.
    problems = []
    if rms > MAX_RMS_PX:
        problems.append(f"RMS {rms:.3f} px > {MAX_RMS_PX} px -- corners are not being "
                        f"fit well (bowed board? wrong --square-mm layout? blur?)")
    if abs(fx - fy) / max(fx, fy) > 0.05:
        problems.append(f"fx and fy differ by {abs(fx-fy)/max(fx,fy)*100:.1f}% -- "
                        f"square pixels should agree within a few percent; often means "
                        f"the frames were resized/cropped, or too few tilted views")
    if abs(cx - w / 2) > 0.1 * w or abs(cy - h / 2) > 0.1 * h:
        problems.append("principal point is >10% away from the frame centre -- usually "
                        "a cropped (non-full-FOV) sensor mode or thin edge coverage")
    if len(used) < 12:
        problems.append(f"only {len(used)} views used -- capture ~{TARGET_SHOTS} for a "
                        f"stable fit")
    if problems:
        print("\nCHECK THESE BEFORE FLYING:")
        for p in problems:
            print(f"  !! {p}")
    else:
        print("\nsanity checks passed.")
    print(f"\nfor reference, the FOV above should look like the lens you have "
          f"(Camera Module 3 standard ~66 deg, wide ~102 deg horizontal).")


def save(K, D, rms, size, used, args):
    """
    Write the two pickles LandOnAprilTag.load_calibration() expects, plus a JSON
    sidecar so future-you can tell what resolution/board/date they came from.
    Any existing pair is moved into backup/<timestamp>/ first -- never overwritten.
    """
    CALIB_DIR.mkdir(parents=True, exist_ok=True)
    km = CALIB_DIR / "cameraMatrix.pkl"
    dp = CALIB_DIR / "dist.pkl"

    existing = [p for p in (km, dp) if p.exists()]
    if existing:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        dest = BACKUP_DIR / stamp
        dest.mkdir(parents=True, exist_ok=True)
        for p in existing:
            shutil.copy2(p, dest / p.name)
        info = CALIB_DIR / "calibration_info.json"
        if info.exists():
            shutil.copy2(info, dest / info.name)
        print(f"\nbacked up previous calibration -> {dest}")

    K = np.asarray(K, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64)
    with open(km, "wb") as f:
        pickle.dump(K, f, protocol=4)
    with open(dp, "wb") as f:
        pickle.dump(D, f, protocol=4)

    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "resolution": [int(size[0]), int(size[1])],
        "cam_res_expected": [int(CAM_RES[0]), int(CAM_RES[1])],
        "board_inner_corners": [args.cols, args.rows],
        "square_mm": args.square_mm,
        "model": "rational (k1..k6,p1,p2)" if args.rational else "standard (k1,k2,p1,p2,k3)",
        "lens_position": args.lens_position,
        "views_used": [p.name for p in used],
        "rms_px": float(rms),
        "camera_matrix": K.tolist(),
        "dist_coeffs": D.ravel().tolist(),
        "camera_params_fx_fy_cx_cy": [float(K[0, 0]), float(K[1, 1]),
                                      float(K[0, 2]), float(K[1, 2])],
    }
    (CALIB_DIR / "calibration_info.json").write_text(json.dumps(meta, indent=2))

    print(f"wrote {km}")
    print(f"wrote {dp}")
    print(f"wrote {CALIB_DIR / 'calibration_info.json'}")
    print("\nnext: `python CalibrateCamera.py verify` for a live undistortion check, "
          "then BenchDryRun.py -- hold a tag a MEASURED distance from the lens and "
          "confirm the reported z matches the tape measure.")


# ----------------------------------------------------------------------
# Verify stage
# ----------------------------------------------------------------------
def verify(args):
    """
    Live raw-vs-undistorted A/B. Straight edges (a door frame, a table edge) that
    bow in the raw feed must come out straight on the right -- especially near the
    frame edges. If the right side looks MORE bent, the calibration is wrong.
    """
    if L is None:
        sys.exit("verify needs the Pi camera (LandOnAprilTag import failed).")
    K, D, params = L.load_calibration()
    print(f"loaded fx={params[0]:.2f} fy={params[1]:.2f} cx={params[2]:.2f} "
          f"cy={params[3]:.2f}")
    map1, map2 = L.build_undistort_maps(K, D)

    grab, close = open_source(args)
    try:
        while True:
            frame = grab()
            if frame is None:
                break
            fixed = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            pair = np.hstack([frame, fixed])
            ph, pw = pair.shape[:2]
            s = 1600.0 / pw
            pair = cv2.resize(pair, (int(pw * s), int(ph * s)))
            cv2.putText(pair, "RAW", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255), 2)
            cv2.putText(pair, "UNDISTORTED", (pair.shape[1] // 2 + 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow("verify (q to quit)", pair)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        close()
        cv2.destroyAllWindows()


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Generate cameraMatrix.pkl / dist.pkl for the AprilTag pipeline.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("mode", choices=["board", "capture", "solve", "verify", "all"],
                    nargs="?", default="all",
                    help="board=print a target, capture=grab frames, solve=fit, "
                         "verify=live undistortion check, all=capture+solve")
    ap.add_argument("--cols", type=int, default=BOARD_COLS,
                    help="INNER corners across")
    ap.add_argument("--rows", type=int, default=BOARD_ROWS,
                    help="INNER corners down")
    ap.add_argument("--square-mm", type=float, default=SQUARE_MM,
                    help="measured square size in mm (does not affect the saved "
                         "intrinsics, only the reported board poses)")
    ap.add_argument("--shots", type=int, default=TARGET_SHOTS,
                    help="how many frames to capture")
    ap.add_argument("--out-dir", default=str(CAPTURE_DIR),
                    help="where capture writes frames")
    ap.add_argument("--from-dir", default=None,
                    help="solve from this folder of images instead of --out-dir")
    ap.add_argument("--keep", action="store_true",
                    help="add to the existing captures instead of clearing them")
    ap.add_argument("--no-auto", action="store_true",
                    help="manual capture only (SPACE)")
    ap.add_argument("--no-preview", action="store_true",
                    help="headless: auto-capture with no window")
    ap.add_argument("--blur-min", type=float, default=BLUR_MIN,
                    help="reject frames below this variance-of-Laplacian")
    ap.add_argument("--rational", action="store_true",
                    help="8-coefficient rational distortion model (wide lenses)")
    ap.add_argument("--no-prune", action="store_true",
                    help="keep every view, even high-error ones")
    ap.add_argument("--prune-err", type=float, default=PRUNE_ERR_PX,
                    help="drop views with reprojection error above this (px)")
    ap.add_argument("--lens-position", type=float, default=None,
                    help="lock focus at this LensPosition in dioptres (0=infinity) "
                         "instead of continuous AF; only if flight locks it too")
    ap.add_argument("--usb", type=int, default=None,
                    help="use a USB camera index instead of the Pi camera (testing)")
    ap.add_argument("--board-file", default=str(CALIB_DIR / "chessboard.svg"),
                    help="where `board` writes the printable target")
    args = ap.parse_args()

    if args.mode == "board":
        write_board_svg(Path(args.board_file), args.cols, args.rows, args.square_mm)
        return
    if args.mode == "verify":
        verify(args)
        return
    if args.mode == "solve":
        solve(args, Path(args.from_dir or args.out_dir))
        return

    out = capture(args)                       # "capture" and "all"
    if args.mode == "all" and out is not None:
        solve(args, out)


if __name__ == "__main__":
    main()
