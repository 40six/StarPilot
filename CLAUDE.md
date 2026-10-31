# StarPilot — Fisker Ocean fork (Dave / 40six)

Project context and hard-won knowledge for working on this fork. Read this before touching
Fisker lateral/longitudinal code — much of it is non-obvious and expensive to rediscover.

## What this is

- A **StarPilot** (FrogPilot-derived openpilot) fork for a **Fisker Ocean** on a **Comma 3X**.
- Repo: **github.com/40six/StarPilot** (`40six/openpilot` redirects to it so the one-click comma
  installer URL works). Upstream is `firestar5683/StarPilot` — NOT ours. Built on jos-technion's
  and moistpope's sunnypilot/DBC work.
- Goal: a well-behaved openpilot for the Ocean — AOL (always-on lateral) like MADS on sunnypilot,
  plus openpilot longitudinal (model-based accel + braking).

## Branches

- **`fisker-ocean`** — the locked, working **lateral baseline**. Don't destabilize it. (HEAD was
  `1bbb50c` at last Cowork session.)
- **`fisker-long`** — longitudinal experiments on top of the baseline. (HEAD `e5cb1c7`.)
  Key commits: `82ca3c3` (don't send 0x118), `f452af4` (0x118 takeover + panda block), `e5cb1c7`
  (0x118 gap fix — send across the whole controls_allowed window).
- `fisker-ocean-clean` — older clean point.

## Deploy workflow (now local, via Claude Code)

- Edit locally → commit → `git push origin <branch>`. (The old Cowork flow used git bundles
  because the cloud session couldn't push; from the laptop you just push.)
- On the comma: `cd /data/openpilot && git fetch origin && git reset --hard origin/<branch>`,
  then reboot.
- **Python-only changes** (carcontroller.py, carstate.py, values.py, interface.py, fiskercan.py,
  latcontrol_angle.py) → quick boot, no reflash.
- **Panda changes** (`opendbc_repo/opendbc/safety/modes/fisker.h`) → the panda firmware must be
  rebuilt and reflashed: on the comma `cd /data/openpilot && scons -j4 panda/` then reboot. The
  first boot after is **slow** — that slow boot IS the reflash; a quick boot means it did NOT
  reflash.
  - **Do NOT run a full `scons`** — the full tree fails on a missing `eigen3/Eigen/Dense` include
    (`common/transformations`). Build only `scons -j4 panda/`.
  - **Gotcha:** a `git reset` that crosses a `fisker.h` change can leave the panda on stale/old
    firmware even after a quick reboot (the git-hash version mismatch reflashes from whatever bin
    is on disk). After any branch switch across a panda change, rebuild panda and slow-boot, then
    **verify** with the takeover logger (see below) that 0x117/0x118 are single-source (ours ≈ OEM,
    1:1), not 2×.

## Key files (Fisker)

- `opendbc_repo/opendbc/car/fisker/carcontroller.py` — all TX logic: lateral angle (0x1D0) +
  activation (0x1C0), SecOC stamping, counter alignment, driver-override/AOL/standstill/angle-
  divergence handling, longitudinal (0x121 accel, 0x117 status, 0x118 ESP/AEB handshake).
- `.../carstate.py` — signal decode, cruise state (ADAS 0x313 vs VCU 0x358), OEM counter snapshots
  for takeover alignment, button events (MFS_0x514), fault mapping.
- `.../values.py` — constants: `CarControllerParams` (ANGLE_LIMITS, ACCEL_MIN/MAX), vehicle model
  seed (wheelbase 2.921, steerRatio 15.0 *first-cut*, tireStiffnessFactor 0.85), override/standstill
  thresholds, `BUTTON_MAP`, `CANBUS` (pt=0 vehicle side, cam=2 OEM ADAS side).
- `.../interface.py` — `steerControlType=angle`, `steerActuatorDelay=0.15`, `steerLimitTimer=2.0`,
  `minSteerSpeed=0.0`, `alphaLongitudinalAvailable=True`; longitudinal gated behind `alpha_long`.
- `.../fiskercan.py` — frame builders (create_steer, create_lat_control, create_accel_command,
  create_long_status, create_long_esp_handshake, the ICC 0x52A spoof, EPS 0x1C2 spoof, checksums).
- `opendbc_repo/opendbc/safety/modes/fisker.h` — panda safety + forwarding (fisker_fwd_hook /
  fisker_fwd_modify): the bus2↔bus0 relays, 0x1D0 candidate substitution, long-message blocking.
- `selfdrive/controls/lib/latcontrol_angle.py` — angle command = `get_steer_from_curvature`
  (VehicleModel). The `ascent_*` low-speed logic is Subaru-only; does NOT apply to Fisker.

## Lateral architecture (works — don't break it)

- **0x1D0 steering angle is SecOC-protected** (AUTOSAR Short SecOC, AES-CMAC). The MAC is computed
  in userspace, so panda can't re-stamp. Mechanism: carcontroller ships a window of fully-stamped
  candidate 0x1D0 payloads as **0x5D0 on bus 1**; panda's `fisker_fwd_modify` substitutes the
  candidate matching OEM's current wire counter into the bus2→bus0 0x1D0 stream.
- **Counter continuation:** at every engage transition, our alive/SecOC counters are seeded to
  OEM's current bus-2 value + 1 so the EPS sees a seamless continuation. The GW_Syn_All (0x20)
  reset must be keyed on the 0x1D0 frame's own reset flag, NOT 0x20's counter — keying on 0x20
  caused a +64 SecOC desync and abort=8. (See the long comment block in carcontroller.)
- **AOL / engagement:** lateral session = `CC.enabled OR always_on_lateral_enabled`. AOL toggles on
  a short press of MFS_RiBtnSouth. Driver override uses the EPS's dedicated
  `EPS_DrvrIntvSteerWhlDetd` bit (not raw column torque) plus a torque-override latch.
- Angle-divergence override + full de-assert prevents a permanent U-turn EPS fault; settle-wait on
  resume; standstill pause below ~1 mph.

## Longitudinal state (ACC NOW CODED — re-characterize)

- **NEW (2026-10-31): Dave coded ACC into the car.** Before coding, on-car testing proved the ADAS
  never entered ACC (`ADAS_Sts_ACC_ICC` / `adasCC` stuck at 0), so the **VCU basic cruise** owned
  the speed floor and **ignored all negative AccelReq** — openpilot could only ADD throttle, never
  brake. That was the wall (and Majd's wall).
- Hypothesis to test first: with ACC coded, the existing `0x52A` ICC spoof may finally flip the
  ADAS to **Active (adasCC=3)**, and then the VCU should honor our `AccelReq` including decel.
  Factory ACC does NOT adaptive-cruise on its own (no radar-driven accel), but that's separate from
  whether the VCU honors an externally-supplied AccelReq.
- **Longitudinal frames:** 0x121 `ADAS_LgtCtrl_AccelReq` (SecOC, pos=accel neg=decel), 0x117
  status (Sts=3/Typ=2 ACC_Stop_and_Go — the Typ that was believed to unlock VCU decel), 0x118
  ESP/AEB handshake (ESP_Sts=3). All gated behind `alpha_long` (Settings → Developer → "openpilot
  Longitudinal Control (Alpha)"); panda physically blocks 0x121 without the LONG_CONTROL safety flag.
- **0x118 == AEB channel:** decel-authorization and the factory AEB handshake are the SAME frame.
  Taking over 0x118 (current `fisker-long`) suppresses factory AEB while engaged. On `fisker-long`,
  panda blocks OEM 0x117/0x118 while controls_allowed and we send ours, counter-aligned (seed from
  OEM 0x118 at the rising edge). The send is gated on `cruiseState.enabled` to exactly span the
  blocked window (a `long_active`-only gate left a ~150 ms 0x118 gap → ESP_FltIndcn_AEB fault).
- **First test now:** on `fisker-long` with alpha-long ON and ACC coded, engage and command accel
  then gentle decel; watch the validation logger for `adasCC`→3 and `aEgo` following a negative
  `cmd`. If decel works, decide whether to keep or drop the 0x118/0x52A workarounds now that the
  ADAS is real (the real ADAS 0x118 may carry proper decel authorization — our takeover may no
  longer be needed and only costs AEB). Watch for real-ACC-vs-our-injection conflicts.

## Open LATERAL problems (as of 2026-10-30)

1. **Late/lazy steering into curves** — lethargic response; likely lat-accel limiting / model
   under-commit, not steerRatio.
2. **Right-lane hugging** — progressively worse; was fine when the fork started. Manual CameraOffset,
   LaneCenterOffset, model offset made little difference → points to a **drifting learner**, not a
   static setting.
3. **Constant large LEFT offset on the UI steering arc** — beyond road crown, on flat and
   left-crowned roads; car does NOT physically pull right (not mechanical). Likely same root as #2.
4. **Low-speed weave** (~3–8 mph) — a low-speed closed-loop instability. A fixed custom steerRatio
   below the learner's converged ~15.5 calms it but makes curves lazier. ~14.8 was calmer; Dave was
   bracketing for the highest steerRatio that still kills the weave. Set via UI: Advanced Lateral
   Tuning → Steer Ratio (overrides the online learner).

### Learner-drift diagnosis for #2/#3 (in progress)

The two online learners that bias lane position and accumulate over drives:
- **Camera calibration** — `CalibrationParams` (rpyCalib = roll/pitch/yaw; roll assumed 0). A
  learned **yaw** offset rotates perceived "straight" → car sits off-center. #1 suspect.
- **Steering angle offset** — `angleOffsetAverageDeg` in `LiveParametersV2` (was −1.12°).
- Reader (comma, car on): subscribe to `liveCalibration` + `liveParameters`, print rpyCalib (deg)
  and angleOffsetAverageDeg. A yaw > ~±1.5° or a grown angleOffset is the smoking gun.
- Reset (comma): `python3 -c "from openpilot.common.params import Params; p=Params(); [p.remove(k) for k in ('CalibrationParams','LiveParametersV2','LiveParameters')]"` then reboot and recalibrate
  (drive straight >15 mph a few min; lateral limited until calibrated). Zero the manual
  CameraOffset/LaneCenterOffset/model offsets first so they don't re-bias the clean relearn.

## Diagnostic loggers (on the comma at /data/, run `cd /data/openpilot && python3 /data/<name>`)

- `fisker_handover4.py` — 0x1D0 lateral handover: our vs OEM counters, drops, GW resets around an
  EPS abort. For lateral engage/override faults.
- `fisker_long.py` — longitudinal **validation**: vEgo, longActive, commanded accel, 0x121 AccelReq
  TX, AccelVld, aEgo, gas/brake, **vcuCC/vcuSet + adasCC/adasSet** (cruise state + setpoint), state.
  The main "does it accel/decel" logger.
- `fisker_long_takeover.py` — longitudinal **takeover**: per-address OEM(src2) vs ours(src0/128)
  frame counts + AliveCounters for 0x117/0x118/0x121, dumps a window on `ESP_FltIndcn_AEB` 0→1.
  Use to verify panda blocking is live (ours ≈ OEM 1:1) and diagnose AEB faults.
- `fisker_mfs.py` — MFS_0x514 buttons + VCU setpoint: what a right-thumbwheel speed click looks like
  (value, frames, step) and how VCU_CcTrgSpdDisp responds. (Groundwork for VCU-setpoint control, a
  fallback decel path if ADAS decel never worked — may be moot now that ACC is coded.)
- `fisker_bias.py` — straight-road steering bias (desired vs measured angle, learned angleOffset,
  road roll). For the lane-hug / left-arc bias.
- `fisker_lowspeed.py` — low-speed lateral oscillation: 20 Hz desAng/measAng/err, desCurv/curv,
  angleOffset. Characterizes the weave (command vs servo vs loop-gain).

Loggers read published topics / sendcan only (diagnostic). They echo to console and write
/data/fisker_*.txt.

## Conventions

- Git commit messages end with: `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>` (no other
  trailer).
- Comma CAN math reference: steering angle `EPS_SteerWhlAgSig` 0.0625°/LSB offset −780 (left +,
  right −); command `ADAS_LatCtrl_SteerAnReq` same encoding; 16 CAN/deg. ADAS AliveCounter = byte1
  low nibble (0..14, 15 is the E2E invalid sentinel). Cruise state enum: 0 Off, 1 Init, 2 Standby,
  3 Active, 4 Override, 9/10 Fault.
- Test suite (from `opendbc_repo/`): `python3 -m pytest opendbc/car/fisker/tests/ -q` (297 tests)
  and `opendbc/safety/tests/test_fisker.py` + `test_fisker_secoc_candidate.py` (rebuild
  `scons panda/tests/libsafety` / libsafety.so first — the .so is dlopened, not recompiled by the
  test). Panda C compiles under `-Werror`.
- Safety on-car: no ACC fallback historically; test longitudinal on empty roads, foot over both
  pedals; factory AEB is suppressed while engaged whenever the 0x118 takeover is active.
