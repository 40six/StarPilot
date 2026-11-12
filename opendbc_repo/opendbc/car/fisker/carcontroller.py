import math
import os

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.fisker.fiskercan import FiskerCAN
from opendbc.car.fisker.secoc import stamp_secoc, sync_mac
from opendbc.car.fisker.values import (ANGLE_OVERRIDE_DEG, ANGLE_OVERRIDE_RELEASE_DEG,
                                        CarControllerParams, FiskerFlagsSP, ICC_0x35B_OVERRIDES,
                                        SETTLE_ANGLE_STEP, SETTLE_MAX_TICKS, STANDSTILL_PAUSE_SPEED)


VisualAlert = structs.CarControl.HUDControl.VisualAlert

# CAN IDs of the SecOC-protected actuator messages we transmit.
STEER_CAN_ID = 0x1D0
ACCEL_CAN_ID = 0x121

# 0x1D0 SecOC candidate pool (race-free handover; see opendbc/safety/modes/fisker.h).
# Instead of transmitting one 0x1D0 on bus 0 and fighting OEM's identical next frame for
# the EPS, we ship a small WINDOW of fully-stamped 0x1D0 payloads each tick as 0x5D0
# addressed to bus 1. Panda buffers them and substitutes the one matching OEM's OWN current
# counter in place of OEM's bus-2 0x1D0 — so the EPS rides OEM's counter with our angle,
# with no duplicate/backward value at engage or disengage. The window straddles OEM's
# counter as carstate last saw it, covering the Python→boardd→USB latency so the value
# panda needs is always present; a miss is just a dropped frame, never a bad counter.
# Bus 1 is the MRR's private link (receive only), so panda's safety buffers each
# candidate and then rejects it: nothing is ever transmitted there.
LAT_CANDIDATE_ADDR = 0x5D0   # carries an 0x1D0 payload; panda-internal (never on a real bus)
LAT_CANDIDATE_BUS = 1        # addressed to the MRR link; panda rejects it after buffering
# The window has to cover the whole round trip: OEM frame -> panda -> pandad -> card ->
# sendcan -> panda. Candidate k is used ~(10*k - latency) ms after it reaches panda, and
# panda only uses candidates younger than FISKER_CAND_FRESH_US (60 ms). Upstream's -1..+3
# window covers ~30 ms. On a comma 3X running StarPilot that wasn't enough: almost every
# substitution missed, the EPS saw 0x1D0 gaps and aborted with AbortFb=8 ~100 ms after going
# Active. -1..+9 covers round trips up to ~90 ms; later candidates for the same counter
# overwrite earlier ones in panda, so the angle used is always the newest available.
LAT_CANDIDATE_WINDOW = 11    # number of candidates per tick
LAT_CANDIDATE_LEAD = 1       # how many below OEM's observed counter to start (straddle it)

# 0x121 accel candidate pool: the same mechanism for longitudinal. With ADAS 3.1.1 + ACC coded
# the OEM ADAS runs a real ACC; it controls speed through 0x121 alone (0x117 Typ=1, 0x118 ESP
# channel Off — 0x118 is only the AEB channel). Taking over 0x121/0x117/0x118 outright faulted
# the ADAS irreversibly within 100 ms of ACC going Active (on-vehicle 2026-11-09). So instead
# openpilot rides INSIDE the ADAS's own ACC session: only the AccelReq value of the ADAS's own
# 0x121 is replaced, re-signed for the ADAS's own counter; 0x117/0x118 (and AEB) are untouched.
# Panda forwards the ADAS's original 0x121 whenever no matching candidate is ready.
LONG_CANDIDATE_ADDR = 0x5A1  # carries an 0x121 payload; panda-internal (never on a real bus)
LONG_CANDIDATE_BUS = 1
LONG_CANDIDATE_WINDOW = 11
LONG_CANDIDATE_LEAD = 1
# How far openpilot's accel may move away from the ADAS's own request (m/s²). Both 0 =
# pass-through: the substituted frame carries the ADAS's own AccelReq (step 2: prove the
# substitution itself is transparent before letting openpilot change anything).
LONG_SUB_MAX_ACCEL_DELTA = 0.0
LONG_SUB_MAX_DECEL_DELTA = 0.0

# Steering angle tracking compensation (CarController._update_angle_comp).
ANGLE_COMP_TAU_S = 20.0         # integration time constant: a 1 deg gap is learned in ~20 s
# Cap: the measured gap was ~1.0 deg; at 2.0 the learned value crept to +1.56 over three drives
# (2026-11-11/12) and the car hugged left of center and pulled back hard on right corrections.
ANGLE_COMP_MAX = 1.2            # deg, hard cap on the correction
ANGLE_COMP_MAX_DESIRED = 5.0    # deg: learn only when the model asks for near-straight
ANGLE_COMP_COOLDOWN_TICKS = 300 # no learning during and for 3 s after a driver override / blend back
ANGLE_COMP_MIN_SPEED = 9.0      # m/s (~20 mph): learn only at speed
ANGLE_COMP_MAX_ANGLE = 10.0     # deg: learn only near center (straight / gentle curves)
ANGLE_COMP_MAX_ERROR = 3.0      # deg: ignore transients (turn-in, overrides)
ANGLE_COMP_STEADY_STEP = 0.05   # deg per 10 ms tick: wheel must be steady
# The learned value is carried across drives in this file (a new Params key would need a rebuilt
# params library, which the prebuilt tree can't do). None disables load/save (tests).
ANGLE_COMP_FILE: str | None = "/data/fisker_angle_comp"

# Follow-the-driver override: while the driver is overriding, keep the lateral request active and
# move the target with the wheel instead of releasing the EPS. The old release (Req=0 until driver
# torque < OVERRIDE_RELEASE_TORQUE) never resumed mid-curve: holding a curve without assist takes
# 0.4-2.6 Nm (route 000000bd seg 20: paused 11.6 s, torque > 0.5 Nm 81% of it, EPS intervention bit
# never set). With the EPS still holding the wheel where the driver put it, the driver needs little
# torque, the override latch releases, and the target blends back to the model over FOLLOW_BLEND_S.
# Creating this file turns it off (read at startup).
FOLLOW_DRIVER_OFF_FILE: str | None = "/data/fisker_follow_driver_off"
FOLLOW_BLEND_S = 1.0   # s to blend from the wheel position back to the model after an override
# Driver authority ramps over this torque band (0.3..1.2 Nm was too eager, 2026-11-12).
FOLLOW_TQ_START = 0.6  # Nm: driver authority starts above resting hands
FOLLOW_TQ_FULL = 2.0   # Nm: full driver authority (the target is the wheel + lead)
# A hard push releases the request (normal power steering) before the EPS ends active control by
# itself: in low-speed turns drivers pushed 3-6 Nm and the EPS went Active -> Available/Off with our
# request still set (route 000000c9 segs 5-7), then took its own time to come back. Steering
# resumes as soon as the driver eases below FOLLOW_RESUME_TQ, mid-turn, from the wheel position.
FOLLOW_RELEASE_TQ = 2.5   # Nm
FOLLOW_RESUME_TQ = 1.5    # Nm
FOLLOW_RESUME_FRAMES = 20 # torque must stay below FOLLOW_RESUME_TQ this long (0.2 s)
# While following, the target leads the wheel in the direction the driver pushes (deg per Nm of
# driver torque, capped): just tracking the measured angle made the EPS hold every position until
# the driver dragged it on, which felt heavy (on-vehicle 2026-11-12). The lead acts like power
# assist and is self-limiting: as the wheel moves the driver's way, the driver's torque drops.
# A flat 1.5 deg/Nm with a 5 deg cap over-steered on driver input (on-vehicle 2026-11-12): the
# lead now shrinks with speed, starts above a deadband (resting hands; column torque also carries
# some motor reaction), and is capped lower.
FOLLOW_TORQUE_GAIN_BP = [5., 15., 25., 30.]  # m/s
FOLLOW_TORQUE_GAIN_V = [0.6, 0.35, 0.15, 0.1]  # deg / Nm above the deadband (cut again 2026-11-12: still too much lead)
FOLLOW_TORQUE_DEADBAND = 0.2                 # Nm (was 0.5: the lead kicked in with a jump)
FOLLOW_LEAD_MAX = 3.0                        # deg


def follow_lead(torque: float, speed: float) -> float:
  """Target lead (deg, + = left) for driver torque (Nm, + = left) while following the driver."""
  excess = max(abs(torque) - FOLLOW_TORQUE_DEADBAND, 0.0)
  lead = excess * float(np.interp(speed, FOLLOW_TORQUE_GAIN_BP, FOLLOW_TORQUE_GAIN_V))
  return math.copysign(min(lead, FOLLOW_LEAD_MAX), torque)


def follow_driver_enabled(off_file: str | None) -> bool:
  return off_file is None or not os.path.exists(off_file)


def load_angle_comp(path: str | None) -> float:
  """Last learned compensation, clamped to the cap; 0 if missing or unreadable."""
  if path is None:
    return 0.0
  try:
    with open(path) as f:
      value = float(f.read().strip())
  except (OSError, ValueError):
    return 0.0
  return float(np.clip(value, -ANGLE_COMP_MAX, ANGLE_COMP_MAX)) if math.isfinite(value) else 0.0


def save_angle_comp(path: str | None, value: float) -> None:
  """Atomically store the learned compensation; failures are ignored (it is re-learned anyway)."""
  if path is None:
    return
  try:
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
      f.write(f"{value:.4f}\n")
    os.replace(tmp, path)
  except OSError:
    pass


def apply_fisker_steer_angle_limits(desired_angle, last_angle, speed, measured_angle, lat_active, controls_permitted):
  # Req=0 releases the EPS, but Panda still rate-checks 0x1D0 while ACC/MADS permits
  # control. Follow the driver's measured angle at the normal rate instead of
  # snapping to it and dropping frames at Panda. With both permissions off, Panda
  # requires a measured-angle heartbeat instead.
  target = desired_angle if lat_active else measured_angle
  return apply_std_steer_angle_limits(target, last_angle, speed, measured_angle,
                                     controls_permitted, CarControllerParams.ANGLE_LIMITS)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.fcan = FiskerCAN(CP, self.packer)

    self.apply_angle_last = 0.0

    # SecOC message counter. The counter is a per-Reset-window frame index, NOT a free
    # monotonic counter. It restarts at 1 on the first 0x1D0/0x121 frame after the GW
    # Reset counter (0x20) increments, then +1 per 100 Hz frame (reaching ~101 before the
    # next Reset ~1 s later). Only the low 6 bits appear on the wire (SSecOC_Fresh_Byte0);
    # the MAC authenticates the full 64-bit freshness. The EPS reconstructs this window
    # index for anti-replay, so we must reproduce it exactly.
    #
    # Counter policy (confirmed on-vehicle 2026-10-19, user's model): SNAP ONCE at the
    # block moment to OEM's reconstructed full counter + 1, then free-run +1 per tick.
    # We become OEM's exact continuation: at the first TX, OEM just delivered counter N
    # to the EPS, we send N+1; we then march in lockstep with OEM's (now-blocked) bus-2
    # stream, so at disengage OEM resumes at our_last + 1 — clean handoff both ways.
    # Snap-once (not continuous-mirror) is essential: mirroring OEM every tick emits a
    # DUPLICATE whenever our 100 Hz read phase catches the same OEM frame twice, which
    # the EPS rejects. Re-snap on a GW Reset boundary (counter restarts there). 0x1D0 and
    # 0x121 share the index (same 100 Hz restart), so the longitudinal path uses it too.
    self.secoc_window_ctr = 0
    self.was_tx_secoc_prev = False  # rising edge of "transmitting any SecOC PDU" -> seed counter
    self.secoc_reset_prev = None    # GW reset change while transmitting -> re-seed to the new window

    # Ticks remaining in which to ALSO emit low-counter (post-reset) 0x1D0 candidates. At a
    # GW SecOC reset the OEM wire counter restarts at 1, but carstate's reconstructed
    # oem_1d0_secoc_full_ctr still reads the stale pre-reset value for ~2 frames (the 0x1D0
    # restart lags the GW_Syn_All edge, and there's pipeline latency on top). For those
    # frames the normal window is centred on the stale high counter and has no candidate for
    # wire 1/2, so panda drops them and the EPS aborts=8 right at its own counter restart
    # (observed on-vehicle 2026-10-25 — two drops per GW reset). Emitting a second window at
    # the post-reset low counters for a few ticks covers the restart until carstate catches up.
    self._post_reset_cand_ticks = 0
    self.POST_RESET_CAND_TICKS = 4

    self.secoc_key_verified = False
    self.secoc_warn_logged = False

    # Steering angle tracking compensation (see _update_angle_comp). Starts from the value learned
    # on previous drives and keeps learning from there.
    self.angle_comp = load_angle_comp(ANGLE_COMP_FILE)
    carlog.info(f"Fisker angle comp start {self.angle_comp:+.3f} deg")
    self._comp_prev_angle = 0.0
    self._comp_log_ticks = 0
    self._comp_cooldown = 0

    # "We ARE OEM" AliveCounter substitution: every OEM 0x1D0/0x1C0 tick that panda
    # blocks, we transmit a frame with the SAME AliveCounter that OEM would have sent —
    # our stream literally continues OEM's numbering during the blocked window. At
    # disengage panda unblocks OEM, and OEM's next tick lands at our_last + 1 which is
    # exactly what EPS was expecting next — seamless handoff both directions, no
    # momentary BSM error on disengage (see user's diagram). Only transmit when OEM's
    # bus-2 counter has advanced since our previous send, else we'd emit a duplicate
    # (EPS rejects duplicates in the E2E monotonic check).
    self.alive_1d0 = 0
    self.alive_1c0 = 0
    self.was_lat_engaged_prev = False   # tracks lat_engaged (MADS) rising edge for engage delay
    self.was_tx_lateral_prev = False    # rising edge of lateral TX -> snap alive counters from OEM
    self._override_eps_recovery = False # EPS latched Off by a driver overpower -> de-assert to reset
    self._eps_was_active = False        # EPS reached Active this engagement (gates override recovery)
    self.was_lat_active_prev = False    # rising edge of lat_active within a session -> smooth re-entry
    self._settle_hold = 0               # ticks left to wait for the wheel to stop self-centering
    self.resume_hold_remaining = 0      # ticks left holding Req=0 after a resume (still transmitting)
    self._prev_steer_angle = 0.0        # for the settle angle-rate check
    self._angle_override_latch = False  # measured wheel diverged far from command -> driver steering

    # ICC 0x52A spoof: emit ONE frame per OEM tick (detected by AliveCounter change).
    # ICC broadcasts at ~10 Hz while we control at 100 Hz, so a naive tx-every-tick
    # would emit ~10 duplicates per OEM tick — the receiver's E2E monotonic-counter
    # validator would flag those as replays and reject them.
    self.last_spoofed_icc_alive = -1
    # EPS_0x1C2 spoof: same cadence rule (one spoof per real OEM tick, ~50 Hz).
    self.last_spoofed_eps_1c2_alive = -1

    # Lateral fade-out: ticks remaining of post-disengage grace. On MADS falling edge
    # we extend the TX window by this many ticks so the EPS sees Req=0 for a beat
    # and transitions Active→Available smoothly, before our stream fully drops. Then
    # the panda lateral relay (60 ms) lets OEM resume to the EPS right after our last
    # fade frame — our counter is OEM's exact continuation so OEM picks up at our_last+N
    # with a tiny N (clean handoff). Keep the fade short: 10 ticks @ 100 Hz = 100 ms is
    # plenty for the EPS to see Req=0 and release, and keeps the total post-disengage
    # blackout (fade + relay) to ~160 ms instead of the old ~1 s. Was 50 (500 ms).
    self.lat_fade_remaining = 0
    self.FADE_TICKS = 10

    # Engagement delay: wait this many ticks after lat_engaged rising edge
    # before we start TX'ing. Between our carstate's OEM-counter snapshot
    # and our first TX arriving at EPS, OEM can send one more 0x1D0 frame
    # on bus 2 that panda forwards to bus 0 (panda's lateral relay only
    # activates after our FIRST TX). If that OEM frame beats our TX to EPS,
    # EPS's last-accepted counter == our TX counter → duplicate → abort=8.
    # We re-snap each tick during the delay to track OEM's latest wire value.
    #
    # The delay ALSO covers a second, larger race seen on-vehicle 2026-10-26:
    # on this no-ACC-coded car the panda's AOL authorization (aol_allowed, i.e.
    # controlsAllowed) rises ~180 ms AFTER openpilot first decides lat is engaged,
    # because the two read the vehicle cruise/MFS state through independent code
    # paths (software carstate vs. safety firmware) with different latencies.
    # While openpilot TXes candidates but the panda is not yet authorized, panda
    # arms the relay (blocking OEM's 0x1D0/0x1C0 from the EPS) but won't substitute
    # our candidate → the EPS receives NEITHER OEM nor our frames → it starves and
    # aborts (abort=8, EPS Available→Active never completes). The moving-engage log
    # showed exactly this: a ~160 ms burst of CA=False drops right at the
    # Available→Active transition. Holding off our TX until the delay expires keeps
    # the relay disarmed (OEM keeps feeding the EPS) across the whole authorization
    # window, so we only ever start injecting once the panda can actually deliver.
    # 25 ticks (250 ms @ 100 Hz) covers the measured ~180 ms race with margin while
    # keeping the engage handoff short; the OEM heartbeat steers the car during the
    # hold (OEM ADAS is in Standby on this car, so it's just the keep-alive stream).
    self.engage_delay_remaining = 0
    self.ENGAGE_DELAY_TICKS = 25
    self.RESUME_HOLD_TICKS = 25   # same 250 ms ease-in on a resume, but without stopping the stream

    self.follow_driver = follow_driver_enabled(FOLLOW_DRIVER_OFF_FILE)
    self.follow_blend = 1.0       # 0 = target at the wheel (driver overriding) .. 1 = model target
    self.hard_override = False    # follow mode: driver pushing hard -> request released (see FOLLOW_RELEASE_TQ)
    self._hard_override_prev = False
    self._resume_tq_frames = 0
    carlog.info(f"Fisker follow-the-driver override {'on' if self.follow_driver else 'off'}")

    # StarPilot Always On Lateral state, set by card.py each tick before apply()
    # (StarPilotCard.always_on_lateral_enabled). Replaces sunnypilot's CC_SP.mads.active.
    self.always_on_lateral_enabled = False

  def _fisker_flags(self) -> int:
    # FPCP (StarPilotCarParams) is attached by CarInterfaceBase after __init__.
    return int(self.FPCP.flags) if self.FPCP is not None else 0

  def _maybe_verify_key(self, CS) -> None:
    """Verify the stored SecOC key against the GW sync MAC once at startup."""
    if self.secoc_key_verified or not CS.secoc_sync_seen or not self.secoc_key:
      return
    if sync_mac(self.secoc_key, CS.secoc_trip, CS.secoc_reset) == CS.secoc_sync_mac:
      self.secoc_key_verified = True
      carlog.info("Fisker SecOC key verified against GW sync MAC")
    elif not self.secoc_warn_logged:
      carlog.error("Fisker SecOC key mismatch — GW sync MAC does not match stored SecOCKey")
      self.secoc_warn_logged = True

  def _update_angle_comp(self, CS, desired_angle_deg: float, lat_active: bool) -> None:
    """Learn the EPS's steady angle-tracking gap and fold it into the command.

    The Ocean's EPS settles ~1 deg to the right of the commanded angle, at every speed (lane-bias
    drive 2026-11-11: commanded - measured +1.01/+1.00/+0.99 deg at 25-45/45-60/60+ mph). The OEM
    ADAS has the same gap and closes it itself (its own steady requests sit +1.4 deg above the
    measured angle). openpilot's angle command has no feedback on that gap, so the car settled
    ~0.4 m right of lane center. This integrates (desired - measured) while we are actively and
    steadily steering near center, and adds it to the command: a slow integral on the angle
    tracking error, capped so it can never become a large steering input.
    """
    meas = CS.out.steeringAngleDeg
    steady = abs(meas - self._comp_prev_angle) < ANGLE_COMP_STEADY_STEP
    self._comp_prev_angle = meas
    error = desired_angle_deg - meas
    # A driver override and the blend back afterwards are deliberate gaps, not EPS tracking error.
    if CS.driver_intervening or self.follow_blend < 1.0:
      self._comp_cooldown = ANGLE_COMP_COOLDOWN_TICKS
    elif self._comp_cooldown > 0:
      self._comp_cooldown -= 1
    if (lat_active and steady and CS.eps_lat_sts == 2 and not CS.out.steeringPressed and not CS.driver_intervening
        and self._comp_cooldown == 0 and abs(desired_angle_deg) < ANGLE_COMP_MAX_DESIRED
        and CS.out.vEgoRaw > ANGLE_COMP_MIN_SPEED and abs(meas) < ANGLE_COMP_MAX_ANGLE and abs(error) < ANGLE_COMP_MAX_ERROR):
      self.angle_comp = float(np.clip(self.angle_comp + error * DT_CTRL / ANGLE_COMP_TAU_S, -ANGLE_COMP_MAX, ANGLE_COMP_MAX))
      self._comp_log_ticks += 1
      if self._comp_log_ticks % 1000 == 0:   # ~every 10 s of learning
        carlog.info(f"Fisker angle comp {self.angle_comp:+.3f} deg (error {error:+.3f})")
        save_angle_comp(ANGLE_COMP_FILE, self.angle_comp)

  def _stamp(self, msg, can_id, trip, reset, msg_counter):
    """Fill the SecOC tail (fresh byte + MAC) of a packed frame with the relay's SecOC counter."""
    addr, data, bus = msg
    stamped = stamp_secoc(self.secoc_key, can_id, data, trip, reset, msg_counter)
    return addr, stamped, bus

  def update(self, CC, CS, now_nanos, starpilot_toggles=None):
    actuators = CC.actuators
    can_sends = []

    self._maybe_verify_key(CS)
    secoc_ok = self.CP.secOcKeyAvailable and self.secoc_key_verified
    trip, reset = CS.secoc_trip, CS.secoc_reset

    # SecOC + alive counters are seeded-once-at-block then free-run — see the block
    # right before the lateral TX below (and __init__ for the policy rationale).

    # E2E AliveCounter (byte1 low nibble): 0..14 counter, +1 per frame, never 15 (15 is
    # the E2E invalid sentinel — DBC range [0|14]). For LATERAL (0x1D0/0x1C0) we track
    # per-PDU below so we can align to OEM's most recent bus-2 value at every engage
    # transition — that way our first re-engage frame is (OEM_last + 1), exactly what EPS
    # expects. If we free-ran with self.frame%15 through disengaged periods, EPS's
    # last-accepted-from-OEM would diverge from our next-transmit and the first re-engage
    # frame would be rejected (surfaces as an LKA fault / "ADAS error" on cluster and
    # MADS auto-off). The longitudinal candidates take their counters from the OEM 0x121.

    # ---- Lateral (steering angle 0x1D0 + activation 0x1C0 @ 100 Hz) ----
    # Lateral TX is driven by the lateral session (see lat_session below), not by raw
    # cruise state. While openpilot isn't steering it stays transparent on the lateral
    # path so the OEM ADAS keeps running its own LKA (gated by the user's ICC_LKASetting).
    # Panda's fisker_fwd_hook has the matching rule: OEM 0x1D0/0x1C0 are only replaced
    # while openpilot is actively injecting (time-based relay + 0x5D0 candidate pool).
    #
    # Engagement variables:
    #   `engaged`     – ACC OR lateral session. Gates the long path + SecOC housekeeping.
    #   `lat_engaged` – lateral session only. Gates the lateral TX (0x1D0 + 0x1C0) and the
    #                   OEM-counter seeding (which only matters when we're about to start
    #                   transmitting). Rising edge of lat_engaged is the moment we snap
    #                   our aliveCounter / SecOC wire counter to OEM's current value so
    #                   our first TX continues OEM's numbering seamlessly.
    #
    # Driver-override release. CS.driver_intervening is driven by the EPS's dedicated
    # EPS_DrvrIntvSteerWhlDetd bit (0x1C4) — a conservative "real driver overpower"
    # signal that EPS computes with internal knowledge of its motor contribution, so
    # it doesn't false-fire from reaction torque the way a plain column-torque
    # threshold does. When it trips we drop lat_active so ADAS_LatCtrl_Req=0 and the
    # EPS stops servoing. We intentionally do NOT key this off CS.out.steeringPressed
    # — that's the sensitive, column-torque-based "driver is touching the wheel"
    # signal, and consuming it here caused the EPS to release on every light nudge
    # (which also killed openpilot's auto-lane-change nudge detection since ALC
    # watches steeringPressed too). Separating the two sensitivities lets light
    # nudges feed ALC while hard overpower releases the EPS.
    # StarPilot port: the lateral session (sunnypilot's `mads.active`) is "openpilot is
    # engaged" (selfdriveState.enabled -> CC.enabled: enabled/softDisabling/overriding) OR
    # StarPilot Always On Lateral is enabled (set by card.py from StarPilotCard each tick;
    # AOL already excludes park/no-calibration/etc., the analogue of MADS `paused`). Only
    # then do we have any business TXing on 0x1D0/0x1C0; otherwise OEM lateral passes
    # through untouched.
    lat_session = bool(CC.enabled) or bool(self.always_on_lateral_enabled)
    engaged = (CS.out.cruiseState.enabled or lat_session) and secoc_ok
    lat_engaged = lat_session and secoc_ok
    # Pause actuation below ~1 mph so the wheel doesn't creep/center at a stop (we keep the
    # session and feed the EPS with Req=0, same as a driver-override pause).
    standstill_pause = CS.out.vEgoRaw < STANDSTILL_PAUSE_SPEED
    # Angle-divergence override: measured wheel far from the model's commanded angle => the
    # driver is steering independently (hard manual input the torque yield missed because the
    # EPS was assisting). Yield instead of commanding into it. Latch with hysteresis so it
    # holds through the maneuver and clears once the wheel is back near the command.
    angle_diverge = abs(CS.out.steeringAngleDeg - float(CC.actuators.steeringAngleDeg))
    if angle_diverge > ANGLE_OVERRIDE_DEG:
      self._angle_override_latch = True
    elif angle_diverge < ANGLE_OVERRIDE_RELEASE_DEG:
      self._angle_override_latch = False
    # With follow-the-driver on, a driver override no longer drops the request (see FOLLOW_DRIVER_OFF_FILE).
    # Follow mode: only a hard push releases the request (hysteresis on driver torque).
    driver_tq = abs(CS.out.steeringTorque)
    if not self.follow_driver:
      self.hard_override = False
    elif driver_tq > FOLLOW_RELEASE_TQ:
      self.hard_override = True
      self._resume_tq_frames = 0
    elif self.hard_override:
      self._resume_tq_frames = self._resume_tq_frames + 1 if driver_tq < FOLLOW_RESUME_TQ else 0
      if self._resume_tq_frames >= FOLLOW_RESUME_FRAMES:
        self.hard_override = False
    driver_released = self.hard_override if self.follow_driver else CS.driver_intervening
    lat_active = (CC.latActive and secoc_ok and not driver_released
                  and not standstill_pause and not self._angle_override_latch)
    # Coming out of a hard push: resume from the wheel (full driver authority, blending back over
    # FOLLOW_BLEND_S) instead of the Req=0 resume hold.
    hard_override_ended = self._hard_override_prev and not self.hard_override
    self._hard_override_prev = self.hard_override
    if hard_override_ended:
      self.follow_blend = 0.0

    # Smooth re-entry after any pause (driver override release, or crossing the standstill
    # speed) while the session stays engaged: hold Req=0 for RESUME_HOLD_TICKS (extended by the
    # settle wait below) so we don't snap. We KEEP TRANSMITTING through the hold — the stream
    # never stopped during the pause, so the EPS is still ours and the counters are continuous;
    # apply_angle_last follows the wheel (target = measured while not active) and then slews to
    # the model target. This used to re-arm the engage delay instead, which STOPPED the stream
    # for 250 ms: panda dropped OEM 0x1D0 while the candidate pool was still armed, OEM fed the
    # EPS, then we grabbed it back — and the re-grab aborted the EPS (AbortFb=8, "LKAS Fault")
    # on-vehicle 2026-11-09 after a gas override. The fresh-engage rising edge (below) still
    # uses the engage delay; this handles resumes within a session.
    resume_edge = lat_active and not self.was_lat_active_prev and self.was_lat_engaged_prev and not hard_override_ended
    if resume_edge:
      self.resume_hold_remaining = self.RESUME_HOLD_TICKS
      self._settle_hold = SETTLE_MAX_TICKS
    # Settle wait: a fixed hold still grabbed a wheel that was still self-centering fast on
    # release, which fought the wheel — a jerk, a transient EPS abort ("steering temporarily
    # unavailable"), and the rate limiter saturating (a false "limit" alert). On-vehicle
    # 2026-10-26. So while settling, keep the hold extended as long as the wheel is still
    # moving (> SETTLE_ANGLE_STEP deg/tick), and only let the resume hold run out and steer
    # again once it has steadied. Bounded by SETTLE_MAX_TICKS so it can't wait forever.
    ang = CS.out.steeringAngleDeg
    if self._settle_hold > 0:
      self._settle_hold -= 1
      if abs(ang - self._prev_steer_angle) > SETTLE_ANGLE_STEP:
        self.resume_hold_remaining = self.RESUME_HOLD_TICKS
      else:
        self._settle_hold = 0
    self._prev_steer_angle = ang
    self.was_lat_active_prev = lat_active
    if self.resume_hold_remaining > 0:
      self.resume_hold_remaining -= 1
      lat_active = False   # Req=0 and follow the wheel, while still transmitting

    desired_angle_deg = float(actuators.steeringAngleDeg)
    self._update_angle_comp(CS, desired_angle_deg, lat_active)
    angle_comp = self.angle_comp if lat_active else 0.0
    # Follow-the-driver: during an override the target is the wheel itself (the EPS holds it where
    # the driver puts it instead of pulling back); afterwards it blends back to the model target.
    # Driver authority (1 - follow_blend) rises with driver torque immediately and continuously
    # (FOLLOW_TQ_START .. FOLLOW_TQ_FULL), so the target starts yielding as soon as the driver leans
    # on the wheel (a hard push beyond that releases the request, see FOLLOW_RELEASE_TQ). Waiting
    # for the 3 Nm override latch made the EPS hold the model's line at full stiffness until 3 Nm,
    # then let go all at once: a delay, then a jump and an overcorrection (on-vehicle 2026-11-12).
    # Authority decays back over FOLLOW_BLEND_S once the driver eases off.
    if self.follow_driver and lat_active:
      authority = float(np.clip((driver_tq - FOLLOW_TQ_START) / (FOLLOW_TQ_FULL - FOLLOW_TQ_START), 0.0, 1.0))
    else:
      authority = 0.0
    self.follow_blend = min(1.0 - authority, self.follow_blend + DT_CTRL / FOLLOW_BLEND_S)
    meas_angle = CS.out.steeringAngleDeg
    if self.follow_blend < 1.0:
      # steeringTorque and steeringAngleDeg share a sign (+ = CCW / left), so this leads the wheel
      # the way the driver is turning it.
      meas_angle += follow_lead(CS.out.steeringTorque, CS.out.vEgoRaw)
    target_angle = meas_angle + self.follow_blend * (desired_angle_deg + angle_comp - meas_angle)
    # Rate limiting through the handover (see apply_fisker_steer_angle_limits): on a
    # driver override or in the post-disengage fade, Req drops to 0 but 0x1D0 keeps
    # slewing to the measured angle at the normal rate while ACC or AOL still permits
    # control, so Panda's rate check keeps accepting the frames; with neither permission
    # it carries the measured angle as the heartbeat. The tracking compensation rides inside
    # the target, so it ramps in/out at the same rate limits Panda checks.
    self.apply_angle_last = apply_fisker_steer_angle_limits(
      target_angle, self.apply_angle_last, CS.out.vEgoRaw,
      CS.out.steeringAngleDeg, lat_active, engaged,
    )

    # "We ARE OEM" counter continuation. All three counters (alive_1d0, alive_1c0,
    # secoc_window_ctr) are mirrored from OEM's authoritative bus-2 values + 1 at the
    # top of update() — see there. OEM's counters advance in real time regardless of
    # our control-loop jitter, so mirroring them keeps our injected stream on exactly
    # the value the EPS expects next, both at the engage handoff and on every frame
    # after. Panda blocks OEM's own 0x1D0/0x1C0 from the EPS while we TX, so the EPS
    # sees a seamless continuation of OEM's numbering through our frames.
    #
    #   OEM on bus 2 (all ticks):        1  2  3  4  5  6  7  8  9  10 11 12
    #   Panda forwards to bus 0:         1  2  3  4                    11 12
    #                                    └── ENGAGE ──┘     └─ DISENGAGE
    #   Our TX (= OEM_bus2 + 1):                   6  7  8  9  10 11
    if lat_engaged and not self.was_lat_engaged_prev:
      # Rising edge: start the engage-delay window. During the delay we DO NOT TX —
      # we let OEM's own 0x1D0/0x1C0 keep flowing to the EPS so the handoff point is
      # clean, and our counters are already mirroring OEM continuously (set at the
      # top of update()), so the first TX after the delay lands on OEM_current + 1.
      self.engage_delay_remaining = self.ENGAGE_DELAY_TICKS
    if lat_engaged and self.engage_delay_remaining > 0:
      self.engage_delay_remaining -= 1
    # Fade-out window: on lat_engaged falling edge we extend the TX window by FADE_TICKS
    # so the EPS keeps receiving our Req=0 stream for a beat after disengage. During
    # fade: panda's time-based lateral relay keeps blocking OEM 0x1D0/0x1C0, the EPS
    # 0x1C2 spoof keeps running (OEM ADAS still sees EPS=Available), and the EPS
    # transitions Active→Available smoothly off our commands rather than off a sudden
    # OEM-command injection. The 0x1C0 wire fields during fade keep Sts=1/Typ=1 (we
    # haven't "left") and only drop Req to 0 — observed on-vehicle 2026-10-18 that
    # slamming Sts/Typ to 0 at disengage caused EPS abort=8 the same way the mid-
    # engage driver-override transition did.
    # Fade is PRIMED (reset to FADE_TICKS) on every engaged tick so that the
    # moment we disengage it's already full. It's CONSUMED (counted down)
    # only when not engaged. The subtle part: `tx_lateral` must only treat
    # the fade as "active" during the post-disengage window — never during
    # engagement — otherwise priming the fade on the rising edge immediately
    # satisfies the TX gate and we bypass the engage delay. That's exactly
    # what showed up on-vehicle 2026-10-19: first TX was happening ~15 ms
    # into engagement (one tick) instead of ~50 ms (after the 5-tick delay),
    # which bypassed the OEM-counter settle and let the first few SecOC
    # frames go out with a counter the EPS hadn't caught up to yet.
    if lat_engaged:
      self.lat_fade_remaining = self.FADE_TICKS
      in_fade = False
    else:
      in_fade = self.lat_fade_remaining > 0
      if in_fade:
        self.lat_fade_remaining -= 1
    self.was_lat_engaged_prev = lat_engaged
    tx_lateral = (lat_engaged and self.engage_delay_remaining == 0) or in_fade

    # EPS override-latch recovery. On-vehicle 2026-10-26: a hard overpower drives the EPS to
    # Off (EPS_AdasLatCtrlSts == 0) and it will NOT come back while we keep asserting
    # engagement (Sts=1/Req=0). It sat at sts=0 / intervention=1 for ~48 s — through the
    # driver repeatedly releasing — until openpilot stopped asserting, then recovered to
    # Available in ~0.1 s. So the EPS needs us to DE-ASSERT (stop transmitting -> Sts=0) to
    # clear the override, then a fresh engage to return. Previously only an AOL off/on cycle
    # supplied that, so lateral paused on the first overpower and never resumed on its own.
    #
    # While engaged and the EPS reads Off, stop transmitting so OEM's own 0x1D0/0x1C0 flow to
    # the EPS and it resets to Available. Once it's back (sts != 0) AND the driver has let go
    # (not steeringPressed), re-arm the normal engage delay: OEM keeps feeding the EPS through
    # the hold, the SecOC/alive counters re-snap from OEM on the first TX after (was_tx_* went
    # False while we were de-asserted), and apply_angle_last re-seeds to the measured wheel
    # angle — a clean fresh handshake, so steering resumes with no button press.
    # Gate recovery on the EPS having actually come up this engagement: only a drop to Off
    # AFTER it reached Active is an overpower knock-out. At initial engage the EPS may read
    # Off/Available before our handshake brings it up — we must keep transmitting then, not
    # de-assert, so recovery must not fire until it has been Active at least once.
    if not lat_engaged:
      self._eps_was_active = False
      self._override_eps_recovery = False
    elif CS.eps_lat_sts == 2:
      self._eps_was_active = True
    if lat_engaged and CS.eps_lat_sts == 0 and self._eps_was_active:
      self._override_eps_recovery = True
    if self._override_eps_recovery:
      tx_lateral = False   # de-assert so the EPS can clear its override latch
      driver_let_go = (driver_tq < FOLLOW_RESUME_TQ) if self.follow_driver else not CS.out.steeringPressed
      if lat_engaged and CS.eps_lat_sts != 0 and driver_let_go:
        self._override_eps_recovery = False
        self._eps_was_active = False
        self.engage_delay_remaining = self.ENGAGE_DELAY_TICKS

    # Driver angle-override: fully de-assert, don't just drop Req. When the wheel has diverged
    # far from our command (driver steering hard, e.g. to full lock) we must STOP transmitting
    # rather than keep our SecOC candidate stream running at an extreme angle — on-vehicle
    # 2026-10-27 a GW SecOC reset landing in that stream at/near full lock dropped frames and
    # aborted (abort=9), latching a permanent LKAS fault on the return-to-center re-grab.
    # De-asserting lets OEM's own 0x1D0 feed the EPS through the maneuver (OEM handles its own
    # resets); the latch clears on return-to-center and the resume-edge re-arm re-grabs cleanly.
    if self._angle_override_latch:
      tx_lateral = False

    # Handoff seeding: whenever we are NOT transmitting lateral (not engaged, or still
    # inside the engage-delay window), glue apply_angle_last to the measured wheel angle.
    # The first frame we send after the hold is then a legal one-step slew FROM the real
    # wheel position, so it sits inside both the panda angle-vs-measured rate check and
    # the EPS's own error window.
    #
    # On-vehicle 2026-10-26 (fisker_handover3.py) the abort burst was exactly this: at a
    # moving engage our command was still near 0 (the model's straight-ahead target slewed
    # in during the delay) while the wheel was already at ~-2.4 deg, so panda dropped every
    # candidate for ~160 ms until the limiter walked the command out to the measured angle —
    # the drops cleared the instant cmd == meas. The counters climbed cleanly through the
    # burst and CA stayed False even on the frames that DID deliver, ruling out SecOC and
    # authorization. Seeding to measured removes the stale reference so there is no slew-in
    # gap. apply_angle_last was computed above by slewing toward the model target; overwrite
    # it here on non-TX ticks (the value is unused on those ticks since we send nothing).
    if not tx_lateral:
      self.apply_angle_last = CS.out.steeringAngleDeg

    # ---- SecOC + alive counter: seed once at the block moment, then free-run +1 ----
    # We become OEM's exact continuation. On the first TX after an idle gap (the
    # "block moment", i.e. when the panda relay starts stealing OEM's frames), snap
    # to OEM's reconstructed full counter + 1: OEM just delivered N to the EPS, so we
    # send N+1. Then free-run +1 per tick FOREVER (until we stop TXing), marching in
    # lockstep with OEM's now-blocked bus-2 stream — so at disengage OEM resumes at
    # our_last + 1, never a repeat. Snap ONCE (not every tick): re-reading OEM emits a
    # duplicate whenever our 100 Hz read phase lands on the same OEM frame twice, which
    # the EPS rejects (on-vehicle 2026-10-19: continuous-mirror produced STEP-0
    # duplicates).
    #
    # GW reset handling. On-vehicle 2026-10-19 (OEM-vs-ours CMP instrumentation)
    # finally showed what the counter does at a ~1 Hz GW reset: OEM RESTARTS its
    # 0x1D0 SecOC counter (our CMP caught OEM's wire drop from ~40 to ~0 the moment
    # its reset_bits flipped). So the per-window counter is NOT continuous across a
    # reset — it restarts. The EPS only sees OUR 0x1D0 (OEM's is blocked), so it
    # follows our stream: when our reset field increments it expects the counter to
    # restart. We restart to 1 (the documented "first frame after reset" value), in
    # lockstep with our own reset_bits (both driven by CS.secoc_reset), so the
    # freshness we sign and the freshness the EPS reconstructs stay consistent.
    #
    # Do NOT re-snap to carstate's oem_full at the reset: OEM's 0x1D0 restarts ~10 ms
    # AFTER the GW_Syn_All reset (CMP showed oursResetBits flip a tick before OEM's),
    # so at the GW_Syn_All edge carstate's OEM wire is still the stale pre-restart
    # value — re-snapping to it yanked our counter ~25 backward -> EPS replay abort.
    txing_secoc = tx_lateral
    if txing_secoc:
      if not self.was_tx_secoc_prev:
        # Block moment: seed the full per-window counter from OEM's current value.
        self.secoc_window_ctr = int(CS.oem_1d0_secoc_full_ctr) + 1
      elif reset != self.secoc_reset_prev:
        # GW reset: per-window counter restarts at 1.
        self.secoc_window_ctr = 1
        # Cover the OEM wire restart with low candidates until carstate catches up.
        self._post_reset_cand_ticks = self.POST_RESET_CAND_TICKS
      else:
        self.secoc_window_ctr += 1
      self.secoc_reset_prev = reset
    self.was_tx_secoc_prev = txing_secoc

    # Lateral E2E alive counters (0x1D0 / 0x1C0): same seed-once-then-+1, from each
    # PDU's own OEM source. These are independent 4-bit counters that do NOT restart
    # on a GW Reset, so no reset re-seed — just wrap mod 15.
    if tx_lateral:
      if not self.was_tx_lateral_prev:
        self.alive_1d0 = (int(CS.oem_1d0_alive) + 1) % 15
        self.alive_1c0 = (int(CS.oem_1c0_alive) + 1) % 15
      else:
        self.alive_1d0 = (self.alive_1d0 + 1) % 15
        self.alive_1c0 = (self.alive_1c0 + 1) % 15
    self.was_tx_lateral_prev = tx_lateral

    if tx_lateral:
      # Counters (alive_1d0/1c0, secoc_window_ctr) were seeded/incremented just above.
      # Lateral control type: sunnypilot FiskerLateralType toggle. Flag set = LCA/TJA
      # (Typ=3), flag clear = LKA (Typ=1, baseline). The Ocean's EPS has historically
      # only honoured LKA, so defaulting to 1 keeps the port working.
      lat_ctrl_typ = 3 if (self._fisker_flags() & FiskerFlagsSP.LAT_CTRL_LCA.value) else 1
      # Two independent wire bits (see create_lat_control docstring for the full why):
      #   * `lat_engaged_wire` — are we in an engaged lateral session? Drives
      #     Sts/Typ/ReqVld. Stable True across driver-override transients AND
      #     across the fade-out window — only drops once the fade has actually
      #     elapsed and we're about to stop transmitting entirely.
      #   * `lat_active_wire`  — on this tick, are we actively servoing the angle
      #     or backing off for the driver? Drives Req only.
      #
      # Both faults observed on-vehicle 2026-10-18 came from slamming Sts/Typ
      # to 0 while EPS was still in state Active: once on the first driver-
      # torque pulse (carControl.latActive dropped), once on MADS release
      # (lat_engaged dropped and the old fade code TX'd Sts=0/Typ=0). Splitting
      # the bits so Sts/Typ stay 1 while only Req follows lat_active keeps the
      # EPS in Active state right up to the moment we actually stop transmitting.
      lat_engaged_wire = lat_engaged or (self.lat_fade_remaining > 0)
      lat_active_wire = lat_active and lat_engaged
      # 0x1D0: emit a WINDOW of candidates (0x5D0 on bus 1) instead of one direct 0x1D0.
      # Each candidate is a complete 0x1D0 frame stamped for a counter around OEM's CURRENT
      # value — so we continuously mirror OEM (the phase-aliasing that broke continuous-
      # mirror before is now resolved IN PANDA, which picks exactly the counter OEM is on).
      # alive (4-bit, byte 1) and the SecOC full counter advance together from OEM's own
      # values, so every candidate is internally consistent for its counter.
      oem_secoc = int(CS.oem_1d0_secoc_full_ctr)
      oem_alive = int(CS.oem_1d0_alive)
      # (SecOC full counter, E2E alive) pairs to stamp this tick. Each pair is a frame some
      # number of frames ahead of OEM's current one; the counter and alive both step +1 per
      # frame in steady state, so an offset j from "now" gives (oem_secoc+j, oem_alive+j).
      # The MAC is computed over the frame (which carries alive in byte 1) plus the freshness
      # (which carries the counter), so the pair must be self-consistent — that's why alive
      # is tied to the frame offset, not to the counter value.
      cand = [(oem_secoc + k, (oem_alive + k) % 15)
              for k in range(-LAT_CANDIDATE_LEAD, LAT_CANDIDATE_WINDOW - LAT_CANDIDATE_LEAD)]
      # For a few ticks after a GW reset, ALSO stamp the post-reset low counters. At the
      # reset OEM's wire counter restarts at 1 while alive keeps counting, so the j-th frame
      # after now is (counter=j, alive=oem_alive+j). carstate's reconstructed counter lags
      # the restart by ~2 frames, so without these the normal window (still centred on the
      # stale high counter) has nothing for wire 1/2 and the EPS aborts=8 at every reset
      # (observed on-vehicle 2026-10-25). Panda buffers by wire index (counter & 0x3F), so
      # the low set lands in different slots from the high set and never collides; we re-emit
      # each tick and panda keeps the freshest, so the exact reset-landing frame is covered.
      # Sign the window around OEM's counter with the reset counter OEM's 0x1D0 is still using:
      # GW_Syn_All announces a new reset ~10 ms before 0x1D0 restarts, and candidates signed for
      # the new window then match none of OEM's last old-window frames, so panda drops them
      # (EPS AbortFb=8 on-vehicle 2026-11-09, 7 ms before a 0x1D0 restart). The post-reset low
      # candidates are for the new window, so they keep the newest reset.
      oem_reset = next((r for r in (reset, reset - 1, reset + 1) if (r & 0x3) == CS.oem_1d0_rflag), reset)
      cand = [(secoc_k, alive_k, oem_reset) for secoc_k, alive_k in cand]
      if self._post_reset_cand_ticks > 0:
        self._post_reset_cand_ticks -= 1
        cand += [(j, (oem_alive + j) % 15, reset) for j in range(1, LAT_CANDIDATE_WINDOW + 1)]
      for secoc_k, alive_k, cand_reset in cand:
        if secoc_k < 1:
          continue  # counter restarts at 1 each window; <1 is never what the EPS expects
        steer_msg = self.fcan.create_steering_control(self.apply_angle_last, alive_k)
        _addr, stamped, _bus = self._stamp(steer_msg, STEER_CAN_ID, trip, cand_reset, secoc_k)
        can_sends.append((LAT_CANDIDATE_ADDR, stamped, LAT_CANDIDATE_BUS))
      can_sends.append(self.fcan.create_lat_control(lat_active_wire, self.alive_1c0,
                                                    driver_override=CS.driver_intervening,
                                                    lat_ctrl_typ=lat_ctrl_typ,
                                                    lat_engaged=lat_engaged_wire))

      # EPS_0x1C2 spoof: DISABLED. On-vehicle testing with the verified checksum
      # algorithm (CRC-8 J1850, data_id=0x90) STILL caused ADAS errors in MADS and
      # Both modes, even though the frames themselves are checksum-valid and the
      # byte-6 override does what it claims. Hypothesis: the OEM ADAS doesn't only
      # look at EPS_AdasLatCtrlSts on 0x1C2 — it probably cross-references with
      # EPS_0x1C4 (driver torque) and/or EPS_0x475 (EPS mode) which we don't spoof,
      # so the "Available" story we're telling on 0x1C2 contradicts the "actively
      # being moved" story the other EPS signals still carry. Spoofing all three
      # consistently would be the next step, but requires brute-forcing more
      # checksums and risks more side-effects on the cluster. The fade-out alone
      # addresses the disengage transient, which is the user-visible symptom.
      #
      # Everything stays in place for easy revival once we have a reason to try
      # again with a multi-signal spoof:
      #   * E2E_PARAMS entry (0x1C2: (0x90, 64)) in fiskercan.py — verified against
      #     200 captured frames (test_checksum.py), can't hurt to keep.
      #   * create_eps_spoof_0x1c2() in fiskercan.py — self-contained helper,
      #     passes its own tests.
      #   * EPS_0x1C2_SIGNALS + the carstate snapshot — same, harmless unused.
      # if self.last_spoofed_eps_1c2_alive != CS.eps_1c2_alive and CS.eps_1c2_values:
      #   can_sends.append(self.fcan.create_eps_spoof_0x1c2(CS.eps_1c2_values))
      #   self.last_spoofed_eps_1c2_alive = CS.eps_1c2_alive

    # ---- Longitudinal: AccelReq substitution inside the OEM ADAS's own ACC (0x5A1 -> 0x121) ----
    # Only while the ADAS's ACC is commanding (cruiseState.enabled comes from its 0x313),
    # openpilot is long-active and the driver isn't on the gas. Each candidate is the ADAS's
    # latest 0x121 with AccelReq replaced, stamped for a counter around the ADAS's current one;
    # panda swaps in the one matching the ADAS's next frame (fisker_fwd_modify) and forwards
    # the ADAS's own frame otherwise, so a miss just means one frame of stock ACC.
    # Sign with the GW reset counter whose low bits the ADAS's 0x121 is using: its window
    # restarts ~10 ms after GW_Syn_All announces a new reset counter (and can briefly lead it).
    long_reset = next((r for r in (reset, reset - 1, reset + 1) if (r & 0x3) == CS.oem_121_rflag), None)
    if (self.CP.openpilotLongitudinalControl and secoc_ok and CC.longActive and
        CS.out.cruiseState.enabled and not CS.out.gasPressed and CS.oem_121_ctr_synced and
        long_reset is not None):
      oem_accel = CS.oem_121_accel
      accel = oem_accel + float(np.clip(actuators.accel - oem_accel,
                                        -LONG_SUB_MAX_DECEL_DELTA, LONG_SUB_MAX_ACCEL_DELTA))
      for k in range(-LONG_CANDIDATE_LEAD, LONG_CANDIDATE_WINDOW - LONG_CANDIDATE_LEAD):
        secoc_k = CS.oem_121_secoc_full_ctr + k
        if secoc_k < 1:
          continue  # counter restarts at 1 each window
        accel_msg = self.fcan.create_accel_substitute(CS.oem_121_values, accel, (CS.oem_121_alive + k) % 15)
        _addr, stamped, _bus = self._stamp(accel_msg, ACCEL_CAN_ID, trip, long_reset, secoc_k)
        can_sends.append((LONG_CANDIDATE_ADDR, stamped, LONG_CANDIDATE_BUS))

    # ---- ICC SVS / BSD / APA settings (0x35B, ICC on bus 0 -> ADAS module on bus 2) ----
    # Optional: only relayed when ICC_0x35B_OVERRIDES has entries. Panda blocks the ICC's own
    # 0x35B only while our copies are arriving, so with no overrides it's forwarded untouched.
    if ICC_0x35B_OVERRIDES:
      for icc_values in CS.icc_0x35b_frames:
        can_sends.append(self.fcan.create_icc_0x35b(icc_values))

    # ---- ICC 0x52A spoof (bus 2) ----
    # Replace the OEM ICC settings frame flowing bus 0 -> bus 2 with a mutated copy
    # that forces byte 4 to 0x5B (ICCACCSwt=1 + ICCACCFuncTyp=2 + 3 more — see
    # create_icc_spoof_0x52a). Everything else (AliveCounter, reserved bits, other
    # settings) passes through byte-identical, and the CheckSum is patched via XOR
    # delta (no need to know 0x52A's DataID).
    #
    # IMPORTANT: this must be sent UNCONDITIONALLY, not gated on
    # cruiseState.enabled. ADAS stays at ACC_Off until it sees ICC_ACCSwt=1 (and the
    # other byte-4 fields) in a valid 0x52A — i.e. the spoof is what enables ACC to
    # transition out of Off into Standby/Active. Gating on cruiseState.enabled was
    # chicken-and-egg: cruise can't become enabled without the spoof going out
    # first, and the spoof was never sent because cruise wasn't enabled.
    #
    # Cadence: one spoof per OEM tick (detected by AliveCounter change) so we never
    # emit duplicates that would trip ADAS's E2E monotonic-counter check. ICC runs
    # at ~5 Hz; openpilot runs at 100 Hz, so most carcontroller ticks won't fire.
    #
    # Panda's fisker_fwd_hook blocks the OEM's bus-0 -> bus-2 forwarding of 0x52A
    # whenever openpilot has recently emitted its own copy (time-gated, not
    # controls_allowed-gated — same reason as above).
    if CS.icc_52a_seen and CS.icc_52a_alive != self.last_spoofed_icc_alive:
      # Honour the sunnypilot UI toggles:
      #   FiskerACCAutoSpeed → ACC_AUTO_SPEED_OFF flag → clears byte-4 bit 35
      #   FiskerACCTerrain   → ACC_TERRAIN_ON flag    → sets byte-6 bit 0
      # Flags are stored in whichever polarity makes "no flag" = baseline behaviour.
      # ICC_0x52A_OVERRIDES (edited live from the web HUD) goes on top and wins.
      acc_auto_speed = not bool(self._fisker_flags() & FiskerFlagsSP.ACC_AUTO_SPEED_OFF.value)
      terrain_setting = bool(self._fisker_flags() & FiskerFlagsSP.ACC_TERRAIN_ON.value)
      can_sends.append(self.fcan.create_icc_spoof_0x52a(CS.icc_52a_values,
                                                        acc_auto_speed=acc_auto_speed,
                                                        terrain_setting=terrain_setting))
      self.last_spoofed_icc_alive = CS.icc_52a_alive

    # ---- HUD ----
    # Forwarding intercept: the OEM ADAS module stays alive on bus 2 and the panda
    # forwards its status/HUD frames (ACC HUD 0x31C, warning HUD 0x317) to the cluster,
    # so openpilot must NOT also transmit them — that would collide with the OEM's. It
    # only injects the steering command; the OEM keeps driving the cluster/HUD.

    new_actuators = actuators.as_builder()
    # Report the output in the requested-angle frame (without the tracking compensation), so
    # anything comparing actuators vs actuatorsOutput sees only real limiting.
    new_actuators.steeringAngleDeg = self.apply_angle_last - angle_comp

    self.frame += 1
    return new_actuators, can_sends
