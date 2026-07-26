# LandOnAprilTag — testing & tuning checklist

Everything to verify/tune before trusting `LandOnAprilTag.py` to center over an
AprilTag and land on it. Tools referenced:

- **`BenchDryRun.py`** — sends NOTHING to the FC (100% safe). Live overlay of the
  real controller's output; toggle sign flags and tune gains live.
- **`BenchTiltProbe.py`** — POWERED, gentle, single-axis. Confirms the one thing a
  dry run can't: roll/pitch translation direction vs. camera mount.

Same safety gate as `SpinOnGuided.py` everywhere: commands only apply in
`GUIDED_NOGPS` (channel-6 switch UP); flip DOWN → STABILIZE → instant manual
control. The `armed` latch also refuses to act until it has seen a
non-GUIDED_NOGPS mode once since the process started.

---

## 1. Camera calibration — `CalibrateCamera.py`
- [ ] `python CalibrateCamera.py board` → print `resources/calibration/chessboard.svg` at **100% / actual size**, tape it FLAT to something rigid, measure a square.
- [ ] `python CalibrateCamera.py all` → capture ~20 views (fill the frame EDGES, vary DISTANCE, TILT ~30–45°) then solve. It reads `CAM_RES`/`SENSOR_MODE` from `LandOnAprilTag.py`, so the result matches flight by construction.
- [ ] Check the printed sanity block: RMS < 1.0 px, fx≈fy, cx/cy near frame centre, FOV plausible.
- [ ] `python CalibrateCamera.py verify` → straight edges must come out straight, especially at the frame edges.
- [ ] If you change `CAM_RES`, recalibrate to match (intrinsics are in pixels).
- Why it matters: pose accuracy drives BOTH horizontal centering AND the
  altitude gates for the LAND hand-off (`COMMIT_ALT_M` / `MIN_TRACK_ALT_M`).

## 2. Roll/pitch translation signs — `BenchTiltProbe.py`
- [ ] `PROBE_AXIS='roll'` (key `1`): tag off to one side → confirm it tilts TOWARD the tag. Flip `INVERT_ROLL` (`r`) if it goes away. Check both sides.
- [ ] `PROBE_AXIS='pitch'` (key `2`): tag ahead/behind → same check. Flip `INVERT_PITCH` (`p`).
- [ ] Note whether `SWAP_XY` (`s`) was needed (camera mounted rotated).
- [ ] Do it tethered/hand-held, low, over a soft surface.

## 3. Yaw sign — `BenchDryRun.py`
- [ ] Rotate the tag under the camera → confirm `yawRate` shrinks `yawErr` toward 0. Flip `INVERT_YAW` (`y`) if it grows.
- [ ] Confirm which tag edge is the "front" = the heading you want at touchdown.

## 4. Gains — `BenchDryRun.py`, then careful flight
- [ ] `KP_TILT` / `KD_TILT`: start soft. Watch for the `CLAMPED` flag (too hot) or sluggish response (too soft).
- [ ] `KP_YAW`, `MAX_YAW_RATE_DEGS`.
- [ ] `MAX_TILT_DEG` (cap on commanded roll/pitch).
- [ ] Press `c` to dump the tuned config, paste back into `LandOnAprilTag.py`.

## 5. Thrust / altitude  *(bench props-off → tethered)*
- [ ] `HOVER_THRUST = 0.5` — **verify 0.5 actually holds altitude on your FC.** If the `GUID_OPTIONS` "thrust as thrust" bit is set, this mapping changes.
- [ ] `DESCEND_THRUST = 0.42` — tune for a gentle descent rate.
- [ ] `COMMIT_ALT_M = 0.50` — if the tag is lost (centred) at/below this z, it's read as "too close to see" → hand off to `LAND`. Set it a bit ABOVE the altitude where your tag actually drops out of frame (measure that in `BenchDryRun.py` by lowering onto the tag).
- [ ] `MIN_TRACK_ALT_M = 0.20` — hard floor: still-visible + centred at/below this z hands off anyway. Keep below `COMMIT_ALT_M`.
- [ ] `COMMIT_LOST_S = 0.4` — debounce so one dropped frame doesn't trigger `LAND`.

## 6. Tolerances
- [ ] `CENTRE_TOL_M = 0.08` and `YAW_TOL_DEG = 8.0`. Too tight → never commits to descending; too loose → lands off-center/off-heading.

## 7. Detector performance  *(Pi 5 / IMX708)*
- [ ] Measure actual FPS at `2304×1296`.
- [ ] If the control loop feels unstable from a low update rate: lower `CAM_RES` FIRST (still full FOV), only then raise `QUAD_DECIMATE` (it costs altitude).
- [ ] Tune `QUAD_SIGMA` (default 0.8) for best long-range detection on the NoIR sensor.

## 8. Autofocus
- [ ] Confirm continuous AF (`AfMode=2`) keeps the tag sharp across altitudes without hunting-induced dropouts.
- [ ] If it hunts badly, fall back to manual focus at a compromise distance.

## 9. NoIR / lighting
- [ ] Bright sun: check the tag isn't overexposed/washed out (kills black/white contrast).
- [ ] Low light: short exposures may underexpose → may need IR illumination or relax `AeExposureMode`.
- [ ] Motion blur vs. exposure is the key trade while the quad is moving.

## 10. Physical tag
- [ ] `TAG_SIZE = 0.125` must match the printed tag edge (metres).
- [ ] Detection altitude scales with physical tag size — **a bigger tag is the cheapest way to gain altitude.**

## 11. LAND handoff
- [ ] Confirm the FC has `LAND` mode and that entering it without GPS descends in place acceptably.
- [ ] After handoff this script stops commanding. Flipping channel-6 down then goes LAND → STABILIZE.

## 12. Tag-loss policy
- [ ] Loss while **high** (last z > `COMMIT_ALT_M`) or while **off-centre** → hovers level indefinitely, never blind-descends. Flip channel-6 down to take over.
- [ ] Loss while **low + centred** (last z ≤ `COMMIT_ALT_M`, held ≥ `COMMIT_LOST_S`) → interpreted as "on top of the tag" → hands off to `LAND`. **Verify on the bench that your tag really does drop out only when the quad is genuinely low**, so a mid-air dropout can't be mistaken for touchdown. A larger tag or higher `COMMIT_ALT_M` widens the margin.

## 13. Camera mounting orientation
- [ ] Axis mapping assumes the camera is roughly aligned with the airframe (`SWAP_XY` handles a 90° rotation). If mounted at an odd angle, that assumption breaks.

---

### Suggested order
1. Calibrate (#1).
2. `BenchDryRun.py` — detection sanity, yaw sign, gains, FPS (#3, #4, #7, #8, #9).
3. `BenchTiltProbe.py` — roll/pitch signs (#2), tethered.
4. Paste tuned flags/gains into `LandOnAprilTag.py`.
5. Props-off → tethered → free-flight, checking thrust/altitude (#5) and land handoff (#11) last.
