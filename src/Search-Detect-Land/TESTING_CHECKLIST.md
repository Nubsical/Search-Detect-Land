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

> *Note, so this doesn't get re-flagged from the logs:* channel 6 sits at 982 µs
> for the whole of logs 76 / 79 / 80 / 81 simply because those were Stabilize and
> AltHold test flights — the switch was never thrown. The mapping is fine:
> `FLTMODE_CH = 6`, `FLTMODE1–4 = STABILIZE`, `FLTMODE5/6 = GUIDED_NOGPS (20)`,
> so C6 ≥ 1621 µs selects GUIDED_NOGPS. Channel 7 (`RC7_OPTION = 70`, AltHold) is
> a separate aux-function switch and is *not* the escape — don't confuse the two.

> **Anything you "verified" with `BenchTiltProbe.py` before the `TYPE_MASK` fix
> must be re-done.** Every `SET_ATTITUDE_TARGET` these scripts sent was being
> discarded by the FC (partial body-rate ignore mask — see the note on
> `TYPE_MASK` in `LandOnAprilTag.py`), so the probe never actually moved the
> vehicle and §2 below could not have told you anything. Sign conventions are
> still UNCONFIRMED.
>
> The fix itself is now **confirmed in flight (2026-07-27, §0)**, so a re-run
> of §2 will finally measure something real.

**Every script that commands the FC now self-tests on startup** (`SpinOnGuided`,
`LandOnAprilTag`, `CenterOnAprilTag`, `BenchTiltProbe`), via the shared
`FlightComms.py`:

- **Pi → FC round-trip.** It reads a parameter back before doing anything else.
  No reply → it refuses to run. Receiving telemetry only ever proved FC → Pi;
  this is the half that was never checked, and the half that fails silently.
- **Ground gate.** It streams `EXTENDED_SYS_STATE` and warns when control starts
  while the FC reports `ON_GROUND`, then announces the `IN_AIR` transition. On
  the ground ArduPilot runs ground handling instead of the attitude controller,
  so a props-off run accepts every command and moves nothing — which looks
  exactly like the bug we just spent a flight diagnosing. The `landed` and
  `fc_armed` CSV columns are now populated by all four scripts.

---

> ## ⛔ GATE — airframe and FC tune are both unverified *(2026-08-02)*
>
> Flight 81 ended in a crash from ~8 m. **Nothing below that powers the motors
> runs until both of these clear** — that includes §2 (`BenchTiltProbe`, powered
> and tethered), §5, §11 and §12. Props-off bench work (§1, §3, §4, §7–§10) is
> unaffected.
>
> 1. **Inspect the airframe.** Arms, motor bells, prop shafts, FC mounting. Every
>    gain recorded below was measured on a frame that has since hit a parking lot.
> 2. **Validate the retune.** Params are loaded and verified — all 18 changes in
>    `resources/params/fixes-2026-08-02.param` confirmed applied, nothing else
>    touched. Fly **Stabilize only**, ~1 m, hover 30–60 s, then check the log for:
>    - `RATE.R` — the 10 Hz peak must be gone or much smaller (was 40 °/s RMS)
>    - `RCOU` — motors must stop sitting at 1090 (were pinned there 82% of the time)
>    - `CTUN.ThO` at steady hover — that is your true `MOT_THST_HOVER`
> 3. **Let `MOT_THST_HOVER` relearn** on a real AltHold hover before trusting §5.
>    `MOT_HOVER_LEARN = 2`, so it will overwrite the 0.11 seed on its own.
> 4. **Then AUTOTUNE**, one axis at a time via `AUTOTUNE_AXES`.
>
> Why this blocks the CV work and not just the flying: the rate loop was sitting
> at loop gain 0.99 — a sustained 10 Hz, ±0.6° limit cycle. Any *in-flight* pose
> noise, motion blur, or centring behaviour measured before the retune was
> measured through that oscillation. Bench measurements taken with the airframe
> static (§3, §4) are unaffected and still stand.

---

## 0. Confirm commands are being accepted at all  ✅ *2026-07-26*
- [x] Run `SpinOnGuided.py`. Its startup round-trips a parameter read, which is the only proof the **Pi → FC** direction works (receiving telemetry only proves FC → Pi).
- [x] Airborne, flip to GUIDED_NOGPS and check its status line: `meas=` should track `cmd=`. If `sent` climbs while `meas` stays ~0, commands are still being rejected — fix that before anything below.
      *(Observed: it spins. Because `TYPE_MASK = 0` there is no partial-acceptance path — the FC takes or discards the whole `SET_ATTITUDE_TARGET`, so an observed yaw response proves the quaternion, all three body rates AND thrust are reaching the attitude controller. The 2292c70 fix is confirmed against the vehicle.)*
      **Proves acceptance only.** It says nothing about the roll/pitch sign conventions (§2, still unconfirmed) or whether `HOVER_THRUST = 0.5` is actually neutral (§5).
- [x] Note: nothing will spin on the ground. At `HOVER_THRUST = 0.5` (zero climb rate) the FC runs ground handling instead of the attitude controller while it still believes it is landed. *(Now detected and announced automatically — the scripts warn on `ON_GROUND` at the moment control starts, so this can never again be mistaken for a rejection bug.)*

---

## 1. Camera calibration — `CalibrateCamera.py`  ✅ *done 2026-07-26*
- [x] `python CalibrateCamera.py board` → print `resources/calibration/chessboard.svg` at **100% / actual size**, tape it FLAT to something rigid, measure a square. *(9×6 inner corners, 14.14 mm square)*
- [x] `python CalibrateCamera.py all` → capture ~20 views (fill the frame EDGES, vary DISTANCE, TILT ~30–45°) then solve. It reads `CAM_RES`/`SENSOR_MODE` from `LandOnAprilTag.py`, so the result matches flight by construction. *(27 of 30 views solved, at 2304×1296 == `CAM_RES`)*
- [x] Check the printed sanity block: RMS < 1.0 px, fx≈fy, cx/cy near frame centre, FOV plausible. *(RMS 0.56 px; fx 1718.1 / fy 1715.9 = 0.13% apart; cx 1143 vs 1152, cy 626 vs 648; HFOV ≈ 68° — matches a standard IMX708.)*
- [x] `python CalibrateCamera.py verify` → straight edges must come out straight, especially at the frame edges.
- [ ] If you change `CAM_RES`, recalibrate to match (intrinsics are in pixels).

  Result lives in `resources/calibration/{cameraMatrix,dist}.pkl`, with the full
  record (views used, board geometry, RMS) in `calibration_info.json`. Previous
  intrinsics backed up to `resources/calibration/backup/20260726-183833/`.
- Why it matters: pose accuracy drives BOTH horizontal centering AND the
  altitude gates for the LAND hand-off (`COMMIT_ALT_M` / `MIN_TRACK_ALT_M`).

## 2. Roll/pitch translation signs — `BenchTiltProbe.py`
- [ ] `PROBE_AXIS='roll'` (key `1`): tag off to one side → confirm it tilts TOWARD the tag. Flip `INVERT_ROLL` (`r`) if it goes away. Check both sides.
- [ ] `PROBE_AXIS='pitch'` (key `2`): tag ahead/behind → same check. Flip `INVERT_PITCH` (`p`).
- [ ] Note whether `SWAP_XY` (`s`) was needed (camera mounted rotated).
- [ ] Do it tethered/hand-held, low, over a soft surface.

## 3. Yaw sign — `BenchDryRun.py`
- [x] Rotate the tag under the camera → confirm `yawRate` shrinks `yawErr` toward 0. Flip `INVERT_YAW` (`y`) if it grows. *(2026-07-26: sign correct, `INVERT_YAW = False` kept. Proportional band verified — 33°→40 deg/s, 13°→17 deg/s, slope ≈1.15 vs `KP_YAW` 1.2. Clamps above 37.5° error as expected.)*
- [ ] Confirm which tag edge is the "front" = the heading you want at touchdown.

## 4. Gains — `BenchDryRun.py`, then careful flight
- [x] `KP_TILT` / `KD_TILT`: start soft. Watch for the `CLAMPED` flag (too hot) or sluggish response (too soft). *(2026-07-26 bench: defaults `KP_TILT = 8.0` / `KD_TILT = 4.0` kept. No `CLAMPED` at 0.5 m offset held steady; fast hand-waves peaked at 3.0 of 8.0. Static noise test — tag and camera both fixed — showed no measurable jitter, so KD is not amplifying pose noise.)*
- [x] `KP_YAW` — verified in §3. **`MAX_YAW_RATE_DEGS` partly settled**: §0 flew a sustained 45 deg/s yaw and the vehicle tracked it, so 45 is achievable — the airframe is not the limit. What is still untested is 45 deg/s *while centring*: rotating the camera swings the tag's x/y in frame, so the yaw loop and the tilt loop fight each other. Consider 25–30 for the first tag-tracking flight and raise once centring is stable. Not a bench call. Note `c` does NOT dump this one — edit `LandOnAprilTag.py` directly.
- [x] `MAX_TILT_DEG` (cap on commanded roll/pitch). *(8.0 kept; only reached during deliberate fast waves.)*
- [x] Press `c` to dump the tuned config, paste back into `LandOnAprilTag.py`. *(No gains changed, so defaults stand — nothing to paste.)*
- [ ] **Re-check these in flight after the retune.** They should still be the right starting point — the FC's *angle* loop (`ATC_ANG_RLL/PIT_P = 4.5`) was not touched, so a commanded 5° still produces 5°. What changed is the rate loop underneath it, which was adding ±0.6° of 10 Hz jitter on top of every commanded angle. Expect the same gains to feel *cleaner*, not different. If they now feel sluggish, that is the rate retune being conservative (loop gain 0.52) and is AUTOTUNE's job to fix, not `KP_TILT`'s.

## 5. Thrust / altitude  *(bench props-off → tethered)*
- [x] `HOVER_THRUST = 0.5` — **verify 0.5 actually holds altitude on your FC.** If the `GUID_OPTIONS` "thrust as thrust" bit is set, this mapping changes. *(Resolved: `GUID_OPTIONS = 0` confirmed from this vehicle's dataflash log, so the thrust field IS a climb rate and 0.5 = zero climb. `SpinOnGuided.py` also re-reads and prints it at startup. Scaling: `WPNAV_SPEED_UP` 250 / `WPNAV_SPEED_DN` 150 cm/s.)*
- [ ] Re-check the slight climb on switching to GUIDED_NOGPS. **Cause identified 2026-08-02 — it was almost certainly not the `TYPE_MASK` rejection spam.** `MOT_THST_HOVER` was `0.3297` while real hover throttle is `0.108`. Whenever the attitude controller is working hard, ArduPilot floors *average* throttle at `MOT_THST_HOVER × ATC_THR_MIX_MAX` = **0.165** to preserve roll/pitch/yaw authority — which is *above* hover, so the vehicle climbs regardless of what thrust you command. In flight 81 this pinned it climbing at 2.5 m/s for 1.7 s with the throttle stick on the bottom stop, and it is what crashed the vehicle. The 10 Hz rate-loop oscillation is what kept the attitude controller busy enough to trigger it. Both are now fixed — re-check whether any climb remains.
      *Same mechanism applies in GUIDED_NOGPS: `HOVER_THRUST = 0.5` means "zero climb rate", but the mixer floor overrides it, so a commanded zero climb still climbed.*
- [ ] `DESCEND_THRUST = 0.42` — tune for a gentle descent rate.
- [x] `COMMIT_ALT_M = 0.50` — if the tag is lost (centred) at/below this z, it's read as "too close to see" → hand off to `LAND`. Set it a bit ABOVE the altitude where your tag actually drops out of frame (measure that in `BenchDryRun.py` by lowering onto the tag).
      *(2026-07-26 bench: dropout measured at z = 0.18 while actively centred, vs 0.165 predicted from the vertical FOV (41.4°) — 9% agreement, so the geometry model holds. Extrapolating to the `CENTRE_TOL_M = 0.08` worst case: ~0.23 m if offset along the long axis, **~0.41 m along the short axis** (vertical FOV is the binding constraint). 0.50 sits ~9 cm above that worst case → **KEEP AS IS**. Note the centred-only 0.18 figure makes 0.50 look far too conservative; it isn't. Off-centre is the case that matters.)*
- [ ] **Confirm the ~0.41 m off-centre dropout by measurement** — it is extrapolated, not observed. Repeat the lowering test holding the tag ~8 cm off-centre along the SHORT frame axis.
- [ ] `MIN_TRACK_ALT_M = 0.20` → **raise to 0.25.** Hard floor: still-visible + centred at/below this z hands off anyway. At 0.20 it sits only 2 cm above the measured 0.18 centred dropout, so the tag usually vanishes before the gate can fire. 0.25 gives it room and stays well under `COMMIT_ALT_M`.
- [ ] `COMMIT_LOST_S = 0.4` — debounce so one dropped frame doesn't trigger `LAND`.

## 6. Tolerances
- [ ] `CENTRE_TOL_M = 0.08` and `YAW_TOL_DEG = 8.0`. Too tight → never commits to descending; too loose → lands off-center/off-heading.

## 7. Detector performance  *(Pi 5 / IMX708)*
- [ ] Measure actual FPS at `2304×1296`.
- [ ] If the control loop feels unstable from a low update rate: lower `CAM_RES` FIRST (still full FOV), only then raise `QUAD_DECIMATE` (it costs altitude).
- [ ] Tune `QUAD_SIGMA` (default 0.5) for best long-range detection. Sweep 0.0 → 1.0 and watch which value decodes the most distant tag.

## 8. Autofocus
- [ ] Confirm continuous AF (`AfMode=2`) keeps the tag sharp across altitudes without hunting-induced dropouts.
- [ ] If it hunts badly, fall back to manual focus at a compromise distance.

## 9. Lighting  *(IR-cut module — no IR illumination option)*
- [ ] Bright sun: check the tag isn't overexposed/washed out (kills black/white contrast). Matte paper, not glossy — specular glare off a laser print destroys the quad edges.
- [ ] Low light: short exposures may underexpose → relax `AeExposureMode` to 0 (Normal) and accept more motion blur, or fly in better light.
- [ ] Motion blur vs. exposure is the key trade while the quad is moving.
- [ ] **Re-measure in-flight blur after the retune.** The airframe was oscillating at 10 Hz, ±0.6°, continuously, for the whole of every flight logged so far — a peak angular rate of ±43 °/s. At 2304×1296 with a 68° HFOV (33.9 px/°) that is ≈**1460 px/s** of image motion:

      | exposure | smear |
      |---|---|
      | 1 ms | 1.5 px |
      | 2 ms | 2.9 px |
      | 4 ms | 5.8 px |
      | 8 ms | 11.7 px |
      | 16 ms | 23.3 px |

      Ceiling is ~41 px (the full 1.2° peak-to-peak excursion), since the motion reverses every 50 ms. So the tag was being smeared on *every* frame, in the hover, with the vehicle nominally holding still. The exposure cap added in `0b1e746` was treating the symptom — re-check whether it is still needed, or can be relaxed to buy back low-light performance (§9).

## 10. Physical tag
- [x] `TAG_SIZE = 0.125` must match the printed tag edge (metres). *(2026-07-26: measured with calipers, 0.125 confirmed.)*
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

**Gate first — nothing powered until these clear:**
1. Inspect the airframe (post-crash).
2. Stabilize hover, validate the retune from the log (`RATE.R`, `RCOU`, `CTUN.ThO`).
3. Let `MOT_THST_HOVER` relearn on an AltHold hover, then AUTOTUNE.

**Then:**
4. Calibrate (#1) — props-off, can be done any time.
5. `BenchDryRun.py` — detection sanity, yaw sign, gains, FPS (#3, #4, #7, #8, #9).
6. `BenchTiltProbe.py` — roll/pitch signs (#2), tethered. *Needs the gate cleared.*
7. Paste tuned flags/gains into `LandOnAprilTag.py`.
8. Props-off → tethered → free-flight, checking thrust/altitude (#5) and land handoff (#11) last.
