import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
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

    # Once openpilot has been long-active at least once, keep the ADAS heartbeat going for
    # the rest of the drive. If we go silent between engagements the ESP loses our 0x118 AEB
    # state (falls into "AEB unavailable" fault) and the VCU's E2E AliveCounter validator
    # rejects our first re-engage frames as out-of-sequence — surfacing as "ADAS error,
    # emergency brake unavailable" on the cluster and a cruise fault in openpilot.
    self.long_ever_active = False

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
    # MADS auto-off). The longitudinal path (0x121/0x117/0x118, alpha_long only) still
    # uses the free-running counter for now — same alignment could help there too but is
    # scoped separately.
    alive = self.frame % 15

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
    lat_active = (CC.latActive and secoc_ok and not CS.driver_intervening
                  and not standstill_pause and not self._angle_override_latch)

    # Smooth re-entry after any pause (driver override release, or crossing the standstill
    # speed) while the session stays engaged: re-arm the engage delay so we don't snap. During
    # the hold apply_angle_last re-seeds to the live wheel angle (see handoff seeding below)
    # and the counters re-snap, so we pick up from where the wheel actually is and slew to the
    # target rather than yanking it. The fresh-engage rising edge (below) handles the very
    # first engage; this handles resumes within a session.
    resume_edge = lat_active and not self.was_lat_active_prev and self.was_lat_engaged_prev
    if resume_edge:
      self.engage_delay_remaining = self.ENGAGE_DELAY_TICKS
      self._settle_hold = SETTLE_MAX_TICKS
    # Settle wait: a fixed hold still grabbed a wheel that was still self-centering fast on
    # release, which fought the wheel — a jerk, a transient EPS abort ("steering temporarily
    # unavailable"), and the rate limiter saturating (a false "limit" alert). On-vehicle
    # 2026-10-26. So while settling, keep the hold extended as long as the wheel is still
    # moving (> SETTLE_ANGLE_STEP deg/tick), and only let the engage delay run out and re-grab
    # once it has steadied. Bounded by SETTLE_MAX_TICKS so it can't wait forever.
    ang = CS.out.steeringAngleDeg
    if self._settle_hold > 0:
      self._settle_hold -= 1
      if abs(ang - self._prev_steer_angle) > SETTLE_ANGLE_STEP:
        self.engage_delay_remaining = self.ENGAGE_DELAY_TICKS
      else:
        self._settle_hold = 0
    self._prev_steer_angle = ang
    self.was_lat_active_prev = lat_active

    desired_angle_deg = float(actuators.steeringAngleDeg)
    # Rate limiting through the handover (see apply_fisker_steer_angle_limits): on a
    # driver override or in the post-disengage fade, Req drops to 0 but 0x1D0 keeps
    # slewing to the measured angle at the normal rate while ACC or AOL still permits
    # control, so Panda's rate check keeps accepting the frames; with neither permission
    # it carries the measured angle as the heartbeat.
    self.apply_angle_last = apply_fisker_steer_angle_limits(
      desired_angle_deg, self.apply_angle_last, CS.out.vEgoRaw,
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
      if lat_engaged and CS.eps_lat_sts != 0 and not CS.out.steeringPressed:
        self._override_eps_recovery = False
        self._eps_was_active = False
        self.engage_delay_remaining = self.ENGAGE_DELAY_TICKS

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
    long_txing = self.CP.openpilotLongitudinalControl and (engaged or self.long_ever_active)
    txing_secoc = tx_lateral or long_txing
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
      if self._post_reset_cand_ticks > 0:
        self._post_reset_cand_ticks -= 1
        cand += [(j, (oem_alive + j) % 15) for j in range(1, LAT_CANDIDATE_WINDOW + 1)]
      for secoc_k, alive_k in cand:
        if secoc_k < 1:
          continue  # counter restarts at 1 each window; <1 is never what the EPS expects
        steer_msg = self.fcan.create_steering_control(self.apply_angle_last, alive_k)
        _addr, stamped, _bus = self._stamp(steer_msg, STEER_CAN_ID, trip, reset, secoc_k)
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

    # ---- Longitudinal (accel 0x121 + status 0x117/0x118 @ 100 Hz) ----
    # Same architecture as lateral (see comment above): send the whole triple across the
    # WHOLE engaged window so the VCU/ESP never see the ADAS heartbeat vanish. Content
    # modulates on state:
    #   engaged + long_active + not gas_override: Sts=Active(3) + Typ=ACC(1), AccelVld=1,
    #                                             AccelReq = openpilot's commanded accel
    #   engaged + gas_override:                    Sts=Active(3) + Typ=ACC(1), AccelVld=1,
    #                                             AccelReq = 0 (yield to driver pedal —
    #                                             the VCU arbitrates pedal vs request)
    #   engaged + !long_active:                    Sts=Active(3) + Typ=ACC(1) but
    #                                             AccelVld=0 (idle heartbeat)
    #   !engaged:                                   frames not sent (nothing on bus 2 to
    #                                             forward on this trim, but if the OEM
    #                                             ADAS SW is ever restored it will get
    #                                             through and Sts=Off/Typ=Not_Active)
    #
    # NOTE: the CC->ACC transition is a value change (Sts 0->3) INSIDE this continuous
    # stream — it is not a one-shot event message. The VCU is designed to hand accel
    # control to the ADAS the moment it sees Sts=Active + Typ=ACC on 0x117.
    if self.CP.openpilotLongitudinalControl:
      long_active = CC.longActive and secoc_ok
      gas_override = CS.out.gasPressed         # driver commanding accel via pedal
      accel_active = long_active and not gas_override
      accel = 0.0 if not accel_active else float(np.clip(actuators.accel,
                                                          self.params.ACCEL_MIN,
                                                          self.params.ACCEL_MAX))
      if long_active:
        self.long_ever_active = True

      # Continue sending the ADAS heartbeat once we've ever been active, so the AliveCounter
      # never has a gap and the ESP never sees 0x118 (AEB state) disappear. Idle values
      # (long_active=False -> Sts=Off, AccelVld=Init) mimic what a real ADAS would publish
      # when powered but not commanding.
      # Braking authorization: VCU-side ACC deceleration requires ADAS_LgtCtrl_Typ=TJA/ICA
      # (= value 2, ACC_Stop_and_Go). That's set inside create_long_status when long_active
      # is True — no per-frame gating needed here. ISA_CutOffReq is a separate Speed-Limit
      # mechanism that requires driver-activated Speed Limiter mode; not the path openpilot
      # needs.

      if engaged or self.long_ever_active:
        # 0x121 accel command (SecOC)
        accel_msg = self.fcan.create_accel_command(accel, accel_active, alive)
        can_sends.append(self._stamp(accel_msg, ACCEL_CAN_ID, trip, reset, self.secoc_window_ctr))

        # 0x117 status + 0x118 ESP handshake (plain E2E, no SecOC)
        can_sends.append(self.fcan.create_long_status(long_active, gas_override,
                                                       gear_req=0, counter=alive))
        can_sends.append(self.fcan.create_long_esp_handshake(long_active, gas_override,
                                                              counter=alive))

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
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
