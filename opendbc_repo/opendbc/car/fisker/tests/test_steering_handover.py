import pytest

from cereal import custom
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
import opendbc.car.fisker.carcontroller as ccmod
from opendbc.car.fisker.carcontroller import (CarController, LAT_CANDIDATE_ADDR, LAT_CANDIDATE_BUS,
                                              apply_fisker_steer_angle_limits, load_angle_comp, save_angle_comp)
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.values import (ANGLE_OVERRIDE_DEG, ANGLE_OVERRIDE_RELEASE_DEG, CANBUS, CAR,
                                        DBC, OVERRIDE_RELEASE_FRAMES, OVERRIDE_RELEASE_TORQUE,
                                        OVERRIDE_STEER_TORQUE)
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.fisker.values import CarControllerParams


def candidate_angle(frames):
  """The 0x1D0 angle carried by this tick's candidate window (every candidate carries the same one)."""
  angles = {int.from_bytes(data[2:4], 'big') * .0625 - 780
            for addr, data, bus in frames if addr == LAT_CANDIDATE_ADDR and bus == LAT_CANDIDATE_BUS}
  assert len(angles) == 1
  return angles.pop()


@pytest.fixture
def car(monkeypatch):
  monkeypatch.setattr(ccmod, "ANGLE_COMP_FILE", None)   # never read/write the device's learned value
  monkeypatch.setattr(ccmod, "FOLLOW_DRIVER_OFF_FILE", None)   # follow-the-driver on, whatever the device has
  cp = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  cp.secOcKeyAvailable = True
  sp = custom.StarPilotCarParams.new_message()
  cs = CarState(cp, sp)
  return cp, sp, cs


@pytest.mark.parametrize('last,measured,speed', [(-1.8125, 3.3125, 18.52), (18.875, 11.6875, 18.78), (46.875, 41.25, 10.4)])
@pytest.mark.parametrize('cruise', [False, True])
@pytest.mark.parametrize('follow', [False, True])
def test_override_releases_immediately_without_angle_jump(car, last, measured, speed, cruise, follow):
  # Follow off: the override drops Req at once. Follow on: Req stays active and the angle starts
  # moving toward the wheel. Either way there's no jump, and the override bit is signalled.
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.follow_driver = follow
  controller.was_lat_active_prev = True   # steering was active when the override started
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)  # synthetic key, no vehicle credentials
  controller.apply_angle_last = last
  controller.was_lat_engaged_prev = True
  cs.out = structs.CarState(steeringAngleDeg=measured, vEgoRaw=speed)
  cs.out.cruiseState.enabled = cruise
  cs.driver_intervening = True
  cc = structs.CarControl(latActive=True)
  cc.actuators.steeringAngleDeg = last
  controller.always_on_lateral_enabled = True  # StarPilot AOL session (was MADS active)
  _, frames = controller.update(cc.as_reader(), cs, 0)
  messages = {addr: data for addr, data, bus in frames if bus == CANBUS.pt}
  assert (messages[0x1C0][3] >> 6) & 3 == (1 if follow else 0)
  assert messages[0x1C0][2] & 32  # override remains signalled
  assert 0x1D0 not in messages  # the angle reaches the EPS only through panda's candidate substitution
  sent_angle = candidate_angle(frames)
  assert abs(sent_angle - last) < 2.0
  assert abs(sent_angle - measured) > 3


@pytest.mark.parametrize('speed', [0, 5, 10, 25, 40])
@pytest.mark.parametrize('last,target', [(-20, 20), (20, -20), (0, 900), (0, -900)])
def test_active_limits_unchanged(speed, last, target):
  assert apply_fisker_steer_angle_limits(target, last, speed, 0, True, True) == apply_std_steer_angle_limits(
    target, last, speed, 0, True, CarControllerParams.ANGLE_LIMITS)


def test_disabled_heartbeat_matches_measured_angle():
  assert apply_fisker_steer_angle_limits(50, 20, 15, -10, False, False) == -10


@pytest.mark.parametrize('cruise', [False, True])
def test_disengagement_fade_respects_remaining_permission(car, cruise):
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.apply_angle_last = 20
  controller.was_lat_engaged_prev = True
  controller.lat_fade_remaining = controller.FADE_TICKS
  cs.out = structs.CarState(steeringAngleDeg=-10, vEgoRaw=15)
  cs.out.cruiseState.enabled = cruise
  cc = structs.CarControl(latActive=False)
  _, frames = controller.update(cc.as_reader(), cs, 0)
  messages = {addr: data for addr, data, bus in frames if bus == CANBUS.pt}
  assert (messages[0x1C0][3] >> 6) & 3 == 0
  angle = candidate_angle(frames)
  if cruise:
    assert 18 < angle < 20  # ACC still leaves Panda's active rate check enabled
  else:
    assert angle == -10  # no permission: inactive check requires measured angle


def test_fault_requires_valid_eps_recovery(car):
  cp, sp, cs = car
  parsers = cs.get_can_parsers(cp)
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])

  def update(state, validity, abort, field='steerFaultTemporary'):
    frame = packer.make_can_msg('EPS_0x1C2', CANBUS.pt, {
      'EPS_AdasLatCtrlSts': state, 'EPS_AdasLatCtrlStsVld': validity, 'EPS_AbortFb': abort})
    for parser in parsers.values():
      parser.update([(0, [frame])])
    return getattr(cs.update(parsers, None)[0], field)

  assert not update(0, 0, 0)  # startup is not a historical fault
  assert not update(2, 1, 0)
  assert not update(0, 0, 1)  # expected driver intervention alone
  assert update(3, 1, 8)
  for _ in range(100):
    assert update(0, 0, 0)  # recorded false-recovery state
  assert update(1, 2, 0)  # invalid Available must not clear it
  assert update(2, 0, 0)  # initializing Active must not clear it
  assert update(1, 1, 8)  # valid Available with an abort is still faulted
  assert not update(1, 1, 0)
  assert update(3, 1, 6)
  assert not update(2, 1, 0)

  # Sustained abort escalates to a permanent fault (lateral disengage); it tracks the live
  # abort, so it clears as soon as the EPS stops reporting one, even while the soft latch holds.
  assert not update(1, 1, 8, 'steerFaultPermanent')
  assert not update(1, 1, 8, 'steerFaultPermanent')
  assert update(1, 1, 8, 'steerFaultPermanent')
  assert not update(0, 0, 0, 'steerFaultPermanent')
  assert update(0, 0, 0)  # soft latch still waiting for a valid recovery


def test_angle_divergence_override(car):
  # On-vehicle 2026-10-27: during a hand U-turn the EPS assisted the turn, so column torque
  # stayed under the yield threshold and the EPS intervention bit never tripped, yet the wheel
  # ran ~70 deg past our command -> openpilot kept commanding into a near-full-lock abort=9
  # (permanent LKAS fault). When the measured wheel diverges far from the commanded angle we
  # must yield. Hysteresis: latch at OVERRIDE_DEG, release within RELEASE_DEG.
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.always_on_lateral_enabled = True
  controller.was_lat_engaged_prev = True
  controller.was_lat_active_prev = True
  controller.engage_delay_remaining = 0
  controller.apply_angle_last = 5.0
  cs.out = structs.CarState(steeringAngleDeg=5.0, vEgoRaw=20.0)
  cs.eps_lat_sts = 2
  cc = structs.CarControl(latActive=True)
  cc.actuators.steeringAngleDeg = 5.0   # model commands 5 deg throughout

  controller.update(cc.as_reader(), cs, 0)
  assert not controller._angle_override_latch                      # wheel on target

  cs.out.steeringAngleDeg = 5.0 + ANGLE_OVERRIDE_DEG + 10.0         # driver turns well past
  _, frames = controller.update(cc.as_reader(), cs, 0)
  assert controller._angle_override_latch                          # -> yield
  # fully de-assert while latched: no candidates sent, so OEM's own 0x1D0 feeds the EPS
  assert LAT_CANDIDATE_ADDR not in {addr for addr, _, bus in frames if bus == LAT_CANDIDATE_BUS}

  cs.out.steeringAngleDeg = 5.0 + (ANGLE_OVERRIDE_DEG + ANGLE_OVERRIDE_RELEASE_DEG) / 2
  controller.update(cc.as_reader(), cs, 0)
  assert controller._angle_override_latch                          # held in hysteresis band

  cs.out.steeringAngleDeg = 5.0 + ANGLE_OVERRIDE_RELEASE_DEG - 2.0  # wheel back near command
  controller.update(cc.as_reader(), cs, 0)
  assert not controller._angle_override_latch                      # -> resume


def test_torque_override_yields_below_eps_threshold(car):
  # The EPS's own intervention bit only trips at ~6.5 Nm. We yield proactively at
  # OVERRIDE_STEER_TORQUE (driver torque reads ~0 hands-off), so an override takes less
  # effort and releases before the EPS fights up to its hard threshold.
  cp, sp, cs = car
  parsers = cs.get_can_parsers(cp)
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])

  def intervening(nm, detd=0, vld=1):
    frame = packer.make_can_msg('EPS_0x1C4', CANBUS.pt, {
      'EPS_DrvrSteerTq': abs(nm), 'EPS_DrvrSteerTqDir': 1 if nm < 0 else 0,
      'EPS_DrvrIntvSteerWhlDetd': detd, 'EPS_DrvrIntvSteerWhlVld': vld})
    for parser in parsers.values():
      parser.update([(0, [frame])])
    return cs.update(parsers, None)[0] is not None and cs.driver_intervening

  assert not intervening(OVERRIDE_STEER_TORQUE - 1.0)   # under threshold, EPS bit off -> hold
  assert intervening(OVERRIDE_STEER_TORQUE + 0.5)       # over threshold -> yield (latched)
  assert intervening(OVERRIDE_RELEASE_TORQUE + 0.5)     # held in the hysteresis band, no chatter
  assert intervening(-(OVERRIDE_STEER_TORQUE + 0.5))    # re-trip, either direction
  # Release only after torque stays below the release gate for the full hold time.
  released = False
  for _ in range(OVERRIDE_RELEASE_FRAMES + 2):
    released = not intervening(0.0)
  assert released
  assert intervening(0.0, detd=1)                       # EPS bit remains an independent backstop


def test_standstill_pauses_lateral(car):
  # Below ~1 mph we pause actuation so the wheel doesn't creep/center at a stop: Req follows
  # the measured angle (heartbeat) instead of steering to the model target.
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.always_on_lateral_enabled = True
  controller.was_lat_engaged_prev = True
  controller.was_lat_active_prev = True
  controller.engage_delay_remaining = 0
  controller.apply_angle_last = 10.0
  cs.out = structs.CarState(steeringAngleDeg=10.0, vEgoRaw=0.0)   # stopped
  cs.eps_lat_sts = 2
  cc = structs.CarControl(latActive=True)
  cc.actuators.steeringAngleDeg = 30.0                           # model wants 30 deg
  _, frames = controller.update(cc.as_reader(), cs, 0)
  assert abs(candidate_angle(frames) - 10.0) < 2.0              # paused: holds measured, not 30


def test_resume_from_pause_holds_without_stopping_stream(car):
  # Coming out of a pause (override release or crossing the standstill speed) within an
  # engaged session eases back in with Req=0 for a hold, but KEEPS transmitting: stopping the
  # stream handed the EPS back to OEM and the re-grab aborted it (AbortFb=8, 2026-11-09).
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.always_on_lateral_enabled = True
  controller.was_lat_engaged_prev = True
  controller.was_lat_active_prev = False    # lateral was paused last tick
  controller.engage_delay_remaining = 0
  cs.out = structs.CarState(steeringAngleDeg=0.0, vEgoRaw=20.0)
  cs.eps_lat_sts = 2
  cc = structs.CarControl(latActive=True)
  for tick in range(controller.RESUME_HOLD_TICKS + 2):
    _, frames = controller.update(cc.as_reader(), cs, tick * 10_000_000)
    messages = {addr: data for addr, data, bus in frames if bus == CANBUS.pt}
    assert candidate_wires(frames), "the candidate stream must not stop during a resume"
    assert controller.engage_delay_remaining == 0
    req = (messages[0x1C0][3] >> 6) & 3
    assert req == (0 if tick < controller.RESUME_HOLD_TICKS else 1), tick


def test_settle_wait_holds_until_wheel_steadies(car):
  # On resume we must not re-grab a wheel that's still self-centering fast, or we fight it
  # (jerk + transient EPS abort + false limit alert). Keep holding while the wheel moves,
  # resume only once it steadies.
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.always_on_lateral_enabled = True
  controller.was_lat_engaged_prev = True
  controller.was_lat_active_prev = False    # resuming from a pause
  controller.engage_delay_remaining = 0
  cs.eps_lat_sts = 2
  cc = structs.CarControl(latActive=True)

  ang = 60.0
  for _ in range(15):                       # wheel self-centering fast (4 deg/tick)
    cs.out = structs.CarState(steeringAngleDeg=ang, vEgoRaw=20.0)
    controller.update(cc.as_reader(), cs, 0)
    ang -= 4.0
  assert controller._settle_hold > 0                                        # still waiting
  assert controller.resume_hold_remaining >= controller.RESUME_HOLD_TICKS - 1     # still held
  assert controller.engage_delay_remaining == 0                             # ...without stopping TX

  for _ in range(5):                        # wheel steady now
    cs.out = structs.CarState(steeringAngleDeg=8.0, vEgoRaw=20.0)
    controller.update(cc.as_reader(), cs, 0)
  assert controller._settle_hold == 0                                       # settled -> resume


def test_override_eps_off_deasserts_then_rearms(car):
  # On-vehicle 2026-10-26: a hard overpower drives the EPS to Off (sts=0) and it stays there
  # while we keep asserting Sts=1, until we stop transmitting. So on EPS-Off we must de-assert
  # (send nothing) to let it reset to Available, then re-arm a fresh engage once it's back and
  # the driver has let go — otherwise lateral pauses on the first override and never resumes.
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.always_on_lateral_enabled = True
  controller.was_lat_engaged_prev = True
  controller.was_lat_active_prev = True
  controller.engage_delay_remaining = 0
  cs.out = structs.CarState(steeringAngleDeg=0.0, vEgoRaw=20.0)
  cs.eps_lat_sts = 2  # Active
  cc = structs.CarControl(latActive=True)

  def candidates_present():
    _, frames = controller.update(cc.as_reader(), cs, 0)
    return LAT_CANDIDATE_ADDR in {addr for addr, _, bus in frames if bus == LAT_CANDIDATE_BUS}

  assert candidates_present()                      # steady state: transmitting

  # Hard overpower drops the EPS to Off -> we must stop transmitting so it can reset.
  cs.eps_lat_sts = 0
  cs.out.steeringPressed = True
  cs.out.steeringTorque = 3.0
  for _ in range(3):
    assert not candidates_present()
  assert controller._override_eps_recovery

  # EPS back to Available but driver still holding -> stay de-asserted (don't fight).
  cs.eps_lat_sts = 1
  assert not candidates_present()
  assert controller._override_eps_recovery

  # EPS Available AND driver released -> re-arm the engage delay for a fresh handshake.
  cs.out.steeringPressed = False
  cs.out.steeringTorque = 0.0
  controller.update(cc.as_reader(), cs, 0)
  assert not controller._override_eps_recovery
  assert controller.engage_delay_remaining == controller.ENGAGE_DELAY_TICKS


def test_engage_handoff_seeds_from_measured_angle(car):
  # On-vehicle 2026-10-26: at a moving engage our command was still near 0 (model target)
  # while the wheel sat at -2.4 deg, so panda dropped every candidate for ~160 ms until the
  # limiter slewed in. Through the engage-delay hold apply_angle_last must track the measured
  # wheel angle so the first transmitted frame is a legal one-step slew from the real wheel
  # position, not from a stale reference.
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.always_on_lateral_enabled = True
  controller.apply_angle_last = 0.0            # stale straight-ahead reference
  measured = -2.4
  cs.out = structs.CarState(steeringAngleDeg=measured, vEgoRaw=20.0)
  cc = structs.CarControl(latActive=True)
  cc.actuators.steeringAngleDeg = 0.0          # model wants straight

  # Rising edge opens the engage delay; through the hold we send nothing and
  # apply_angle_last stays glued to the measured wheel angle. The rising-edge tick seeds
  # the delay and then counts one off it, so there are ENGAGE_DELAY_TICKS - 1 pure hold
  # ticks before the first TX.
  for _ in range(controller.ENGAGE_DELAY_TICKS - 1):
    _, frames = controller.update(cc.as_reader(), cs, 0)
    assert LAT_CANDIDATE_ADDR not in {addr for addr, _, bus in frames if bus == LAT_CANDIDATE_BUS}
    assert controller.apply_angle_last == pytest.approx(measured, abs=1e-3)

  # First tick past the hold: we transmit, and the candidate is a legal slew FROM measured
  # (within one step of it), never the stale 0.
  _, frames = controller.update(cc.as_reader(), cs, 0)
  sent = candidate_angle(frames)
  assert abs(sent - measured) < 2.0


def candidate_wires(frames):
  """SecOC wire counters ((byte4 >> 2) & 0x3F) of this tick's 0x1D0 candidates."""
  return sorted({(data[4] >> 2) & 0x3F for addr, data, bus in frames
                 if addr == LAT_CANDIDATE_ADDR and bus == LAT_CANDIDATE_BUS})


def test_gw_reset_emits_low_counter_candidates(car):
  # At a GW SecOC reset the OEM wire counter restarts at 1 while carstate's reconstructed
  # full counter still reads the stale high value for a couple of frames. The controller
  # must emit post-reset low-counter candidates (wires 1..WINDOW) for a few ticks so panda
  # has a match when the EPS's counter restarts — otherwise those frames drop and abort=8.
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.always_on_lateral_enabled = True
  controller.was_lat_engaged_prev = True   # already engaged: past the engage-delay window
  controller.was_lat_active_prev = True
  controller.engage_delay_remaining = 0
  cs.out = structs.CarState(steeringAngleDeg=0.0, vEgoRaw=20.0)
  cs.secoc_trip, cs.secoc_reset = 1, 5
  cs.oem_1d0_alive, cs.oem_1c0_alive = 3, 3
  cs.oem_1d0_secoc_full_ctr = 47   # stale high counter, as carstate reports right after a reset
  cc = structs.CarControl(latActive=True)

  # Steady state: candidates straddle the (high) reconstructed counter, none near wire 1.
  _, frames = controller.update(cc.as_reader(), cs, 0)
  assert 47 in candidate_wires(frames)
  assert not any(w <= 11 for w in candidate_wires(frames))

  # GW reset: secoc_reset increments. carstate still reports the stale high counter.
  cs.secoc_reset = 6
  _, frames = controller.update(cc.as_reader(), cs, 10_000_000)
  wires = candidate_wires(frames)
  assert {1, 2, 3}.issubset(wires), wires        # low window present for OEM's restart
  assert 47 in wires                             # high window still present until carstate catches up

  # The low window persists for a few ticks, then stops once the hole is covered.
  for _ in range(controller.POST_RESET_CAND_TICKS):
    _, frames = controller.update(cc.as_reader(), cs, 20_000_000)
  assert not any(w <= 11 for w in candidate_wires(frames) if w not in range(47 - 1, 47 + 11))


def test_candidates_signed_for_oem_window_during_gw_reset_lag(car):
  # GW_Syn_All announces a new reset counter ~10 ms before OEM's 0x1D0 restarts. During that lag
  # the candidates for OEM's remaining old-window frames must carry OEM's reset flag and be signed
  # with the old reset counter, or panda finds no match and the EPS aborts (AbortFb=8).
  from opendbc.car.fisker.secoc import stamp_secoc
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = key = bytes(range(16))
  controller.always_on_lateral_enabled = True
  controller.was_lat_engaged_prev = True
  controller.was_lat_active_prev = True
  controller.engage_delay_remaining = 0
  cs.out = structs.CarState(steeringAngleDeg=0.0, vEgoRaw=20.0)
  cs.secoc_trip, cs.secoc_reset = 1, 6          # GW already announced reset 6 (flag 2)...
  cs.oem_1d0_rflag = 1                          # ...but OEM's 0x1D0 is still in window 5 (flag 1)
  cs.oem_1d0_alive, cs.oem_1c0_alive = 3, 3
  cs.oem_1d0_secoc_full_ctr = 98
  cc = structs.CarControl(latActive=True)
  _, frames = controller.update(cc.as_reader(), cs, 0)
  cands = [data for addr, data, bus in frames if addr == LAT_CANDIDATE_ADDR and bus == LAT_CANDIDATE_BUS]
  high = [d for d in cands if (d[4] >> 2) & 0x3F == 98 & 0x3F]
  assert high and all(d[4] & 3 == 1 for d in high), "OEM's current frame needs a candidate in OEM's window"
  d = high[0]
  assert d == stamp_secoc(key, 0x1D0, d, 1, 5, 98), "signed with OEM's (old) reset counter"


def _comp_controller(car):
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
  controller.FPCP = sp
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.always_on_lateral_enabled = True
  controller.was_lat_engaged_prev = True
  controller.was_lat_active_prev = True
  controller.engage_delay_remaining = 0
  cs.eps_lat_sts = 2
  cs.driver_intervening = False
  return controller, cs


def _run_comp(controller, cs, ticks, meas=-1.0, desired=0.0, speed=25.0, pressed=False, lat_accel=0.0):
  cs.out = structs.CarState(steeringAngleDeg=meas, vEgoRaw=speed, steeringPressed=pressed)
  cc = structs.CarControl(latActive=True)
  cc.actuators.steeringAngleDeg = desired
  cc.actuators.curvature = lat_accel / max(speed, 0.1) ** 2
  out = frames = None
  for t in range(ticks):
    out, frames = controller.update(cc.as_reader(), cs, t * 10_000_000)
  return out, frames


def test_angle_comp_learns_steady_tracking_gap(car):
  # Wheel steadily 1 deg right of the command: the correction integrates the gap (1 deg per 10 s)
  # and the transmitted angle carries it, while the reported output stays in the requested frame.
  controller, cs = _comp_controller(car)
  out, frames = _run_comp(controller, cs, 500)
  assert controller.angle_comp == pytest.approx(0.25, abs=0.02)   # 1 deg gap, 20 s time constant, 5 s
  assert candidate_angle(frames) == pytest.approx(controller.angle_comp, abs=0.1)
  assert out.steeringAngleDeg == pytest.approx(0.0, abs=0.1)


@pytest.mark.parametrize("kw", [dict(pressed=True), dict(speed=5.0), dict(meas=-12.0, desired=-11.0), dict(meas=-5.0)])
def test_angle_comp_does_not_learn_outside_steady_steering(car, kw):
  controller, cs = _comp_controller(car)
  _run_comp(controller, cs, 300, **kw)
  assert controller.angle_comp == 0.0


def test_angle_comp_needs_active_eps_and_is_capped(car):
  controller, cs = _comp_controller(car)
  cs.eps_lat_sts = 1
  _run_comp(controller, cs, 300)
  assert controller.angle_comp == 0.0
  cs.eps_lat_sts = 2
  _run_comp(controller, cs, 6000, meas=-2.9)
  assert controller.angle_comp == pytest.approx(ccmod.ANGLE_COMP_MAX)


def test_angle_comp_persists_across_drives(car, monkeypatch):
  # The learned value is saved while learning and a new controller (next drive) starts from it.
  import os
  import tempfile
  path = os.path.join(tempfile.mkdtemp(), "fisker_angle_comp")
  assert load_angle_comp(path) == 0.0                     # nothing saved yet
  monkeypatch.setattr(ccmod, "ANGLE_COMP_FILE", path)
  controller, cs = _comp_controller(car)
  _run_comp(controller, cs, 1001)                         # 1000 learning ticks (tick 0 is not yet steady)
  assert load_angle_comp(path) == pytest.approx(controller.angle_comp, abs=1e-3)
  assert controller.angle_comp > 0.45
  restarted = CarController(DBC[CAR.FISKER_OCEAN], car[0])
  assert restarted.angle_comp == pytest.approx(controller.angle_comp, abs=1e-3)


def test_angle_comp_file_is_validated():
  import os
  import tempfile
  path = os.path.join(tempfile.mkdtemp(), "fisker_angle_comp")
  save_angle_comp(path, 7.5)
  assert load_angle_comp(path) == ccmod.ANGLE_COMP_MAX    # clamped to the cap
  for junk in ("nan", "", "abc"):
    with open(path, "w") as f:
      f.write(junk)
    assert load_angle_comp(path) == 0.0
  assert load_angle_comp(None) == 0.0
  save_angle_comp("/nonexistent-dir/x", 1.0)               # unwritable: ignored, no exception


def test_follow_driver_keeps_request_and_tracks_wheel(car):
  # Driver overriding in a curve: the request stays active, the target follows the wheel (no
  # pulling back toward the model), and there is no stream stop or resume hold afterwards.
  controller, cs = _comp_controller(car)
  cs.out = structs.CarState(steeringAngleDeg=0.0, vEgoRaw=25.0)
  cc = structs.CarControl(latActive=True)
  cc.actuators.steeringAngleDeg = 0.0
  controller.update(cc.as_reader(), cs, 0)

  # driver steers 6 deg right of the model, with full-authority torque (below the hard-push release)
  cs.out = structs.CarState(steeringAngleDeg=-6.0, vEgoRaw=25.0, steeringTorque=-2.2)
  for t in range(100):
    _, frames = controller.update(cc.as_reader(), cs, t * 10_000_000)
    messages = {addr: data for addr, data, bus in frames if bus == CANBUS.pt}
    assert (messages[0x1C0][3] >> 6) & 3 == 1, t        # still requesting
    assert candidate_wires(frames), t                    # still transmitting
  assert controller.follow_blend == 0.0
  assert candidate_angle(frames) == pytest.approx(-6.0 + ccmod.follow_lead(-2.2, 25.0), abs=0.2)   # at the wheel + lead

  cs.out = structs.CarState(steeringAngleDeg=-6.0, vEgoRaw=25.0)   # driver relaxes
  for t in range(50):
    _, frames = controller.update(cc.as_reader(), cs, (100 + t) * 10_000_000)
  assert controller.resume_hold_remaining == 0
  assert controller.engage_delay_remaining == 0
  assert 0.45 < controller.follow_blend < 0.55           # halfway through the 1 s blend
  assert -4.5 < candidate_angle(frames) < -1.5            # heading back toward the model (0 deg)
  for t in range(60):
    _, frames = controller.update(cc.as_reader(), cs, (150 + t) * 10_000_000)
  assert controller.follow_blend == 1.0
  assert candidate_angle(frames) == pytest.approx(0.0, abs=0.3)


def test_follow_driver_off_switch(monkeypatch):
  import os
  import tempfile
  off = os.path.join(tempfile.mkdtemp(), "fisker_follow_driver_off")
  assert ccmod.follow_driver_enabled(off)
  open(off, "w").close()
  assert not ccmod.follow_driver_enabled(off)
  assert ccmod.follow_driver_enabled(None)


def test_angle_comp_pauses_around_overrides_and_curves(car):
  # An override and the blend back are deliberate gaps: no learning during them or for 3 s after.
  # Nor when the model asks for a real turn (only near-straight driving shows the EPS gap).
  controller, cs = _comp_controller(car)
  cs.driver_intervening = True
  _run_comp(controller, cs, 50)
  cs.driver_intervening = False
  _run_comp(controller, cs, ccmod.ANGLE_COMP_COOLDOWN_TICKS - 10)   # most of the cooldown
  assert controller.angle_comp == 0.0
  _run_comp(controller, cs, 200)
  assert controller.angle_comp > 0.0

  controller, cs = _comp_controller(car)
  _run_comp(controller, cs, 300, meas=7.0, desired=8.0)   # model asking for a curve
  assert controller.angle_comp == 0.0


def test_follow_driver_leads_in_push_direction(car):
  # While following, the target leads the wheel the way the driver pushes (power-assist feel):
  # torque above the deadband x the speed-scaled gain (0.35 deg/Nm at 15 m/s).
  controller, cs = _comp_controller(car)
  cc = structs.CarControl(latActive=True)
  for torque, expected in ((-2.2, -6.0 - 2.0 * 0.35), (+2.2, -6.0 + 2.0 * 0.35)):
    cs.out = structs.CarState(steeringAngleDeg=-6.0, vEgoRaw=15.0, steeringTorque=torque)
    for t in range(200):
      _, frames = controller.update(cc.as_reader(), cs, t * 10_000_000)
    assert candidate_angle(frames) == pytest.approx(expected, abs=0.2), torque


def test_follow_lead_deadband_speed_scaling_and_cap():
  lead = ccmod.follow_lead
  assert lead(0.15, 15.0) == 0.0 and lead(-0.15, 15.0) == 0.0        # below the deadband: no lead
  assert lead(2.0, 5.0) == pytest.approx(1.8 * 0.6)                    # low speed: 0.6 deg/Nm
  assert lead(2.0, 30.0) == pytest.approx(1.8 * 0.1)                   # highway: 0.1 deg/Nm
  assert lead(-2.0, 30.0) == pytest.approx(-1.8 * 0.1)                 # sign follows the push
  assert lead(8.0, 5.0) == ccmod.FOLLOW_LEAD_MAX                       # capped
  assert abs(lead(2.0, 30.0)) < abs(lead(2.0, 15.0)) < abs(lead(2.0, 5.0))


def test_driver_authority_is_continuous_and_immediate(car):
  # Leaning on the wheel yields at once in proportion to torque (no wait for the 3 Nm latch):
  # nothing at resting hands, partial in between, full from FOLLOW_TQ_FULL.
  controller, cs = _comp_controller(car)
  cc = structs.CarControl(latActive=True)
  cs.driver_intervening = False
  mid = (ccmod.FOLLOW_TQ_START + ccmod.FOLLOW_TQ_FULL) / 2
  for torque, blend in ((0.2, 1.0), (mid, 0.5), (ccmod.FOLLOW_TQ_FULL, 0.0), (-mid, 0.5)):
    controller.follow_blend = 1.0
    cs.out = structs.CarState(steeringAngleDeg=-3.0, vEgoRaw=20.0, steeringTorque=torque)
    controller.update(cc.as_reader(), cs, 0)
    assert controller.follow_blend == pytest.approx(blend, abs=0.01), torque
  # Easing off: authority decays over FOLLOW_BLEND_S, never jumps back to the model.
  cs.out = structs.CarState(steeringAngleDeg=-3.0, vEgoRaw=20.0, steeringTorque=0.0)
  controller.follow_blend = 0.0
  controller.update(cc.as_reader(), cs, 0)
  assert controller.follow_blend == pytest.approx(0.01)


def test_hard_push_releases_and_resumes_mid_turn(car):
  # A hard push (> FOLLOW_RELEASE_TQ) releases the request before the EPS ends active control by
  # itself; easing below FOLLOW_RESUME_TQ for FOLLOW_RESUME_FRAMES resumes at once, mid-turn, from
  # the wheel (full driver authority, no Req=0 resume hold).
  controller, cs = _comp_controller(car)
  cc = structs.CarControl(latActive=True)
  cc.actuators.steeringAngleDeg = 0.0

  def req(torque, ticks):
    cs.out = structs.CarState(steeringAngleDeg=-20.0, vEgoRaw=6.0, steeringTorque=torque)
    for _ in range(ticks):
      _, frames = controller.update(cc.as_reader(), cs, 0)
    return (({a: d for a, d, b in frames if b == CANBUS.pt})[0x1C0][3] >> 6) & 3, frames

  assert req(-1.0, 5)[0] == 1                          # leaning on it: still active (follow)
  assert req(-3.5, 2)[0] == 0                          # hard push: released
  assert req(-1.0, ccmod.FOLLOW_RESUME_FRAMES - 2)[0] == 0   # easing, not long enough yet
  r, frames = req(-1.0, 3)
  assert r == 1                                        # resumed mid-turn
  assert controller.resume_hold_remaining == 0
  assert controller.follow_blend < 0.1                 # starts from the wheel, blends back
  assert candidate_wires(frames)


def test_eps_recovery_resumes_on_low_torque_in_follow_mode(car):
  # After the EPS drops to Off, follow mode re-arms once driver torque is below FOLLOW_RESUME_TQ
  # (hands still on the wheel in a turn), not only once steeringPressed clears.
  controller, cs = _comp_controller(car)
  cc = structs.CarControl(latActive=True)
  controller.update(cc.as_reader(), cs, 0)
  controller._eps_was_active = True
  cs.eps_lat_sts = 0
  cs.out = structs.CarState(steeringAngleDeg=-20.0, vEgoRaw=6.0, steeringPressed=True, steeringTorque=1.2)
  controller.update(cc.as_reader(), cs, 0)
  assert controller._override_eps_recovery
  cs.eps_lat_sts = 1
  controller.update(cc.as_reader(), cs, 0)
  assert not controller._override_eps_recovery
  assert controller.engage_delay_remaining == controller.ENGAGE_DELAY_TICKS


def test_turn_comp_closes_curve_shortfall_then_fades(car):
  # Sustained curve (1 m/s^2), wheel steady 0.5 deg short of the command toward center: the turn
  # hold integrates it (2 s time constant) into the transmitted angle, the reported output stays in
  # the requested frame, and angle_comp (near-center learner) stays out of it. Out of the curve the
  # hold fades back to zero.
  controller, cs = _comp_controller(car)
  out, frames = _run_comp(controller, cs, 301, meas=7.5, desired=8.0, lat_accel=1.0)
  assert controller.turn_comp == pytest.approx(0.75, abs=0.03)    # 0.5 deg x 3 s / 2 s
  assert controller.angle_comp == 0.0
  assert candidate_angle(frames) == pytest.approx(8.0 + controller.turn_comp, abs=0.1)
  assert out.steeringAngleDeg == pytest.approx(8.0, abs=0.1)
  _run_comp(controller, cs, 300, meas=0.5, desired=0.0)
  assert abs(controller.turn_comp) < 0.05


def test_turn_comp_is_signed_by_turn_direction(car):
  controller, cs = _comp_controller(car)
  _run_comp(controller, cs, 301, meas=-7.5, desired=-8.0, lat_accel=1.0)
  assert controller.turn_comp == pytest.approx(-0.75, abs=0.03)


@pytest.mark.parametrize("kw", [dict(pressed=True), dict(speed=4.0), dict(lat_accel=0.3), dict(meas=3.0)])
def test_turn_comp_does_not_learn_outside_steady_curves(car, kw):
  # driver input, intersection speeds, gentle bends (left to angle_comp), and transients (> 3 deg)
  controller, cs = _comp_controller(car)
  args = dict(meas=7.5, desired=8.0, lat_accel=1.0) | kw
  _run_comp(controller, cs, 300, **args)
  assert controller.turn_comp == 0.0


def test_turn_comp_is_capped_and_needs_active_eps(car):
  controller, cs = _comp_controller(car)
  cs.eps_lat_sts = 1
  _run_comp(controller, cs, 300, meas=7.5, desired=8.0, lat_accel=1.0)
  assert controller.turn_comp == 0.0
  cs.eps_lat_sts = 2
  _run_comp(controller, cs, 3000, meas=5.5, desired=8.0, lat_accel=1.0)
  assert controller.turn_comp == pytest.approx(ccmod.TURN_COMP_MAX)


def test_angle_comp_does_not_learn_in_curves(car):
  # A near-center wheel angle in a real curve (e.g. the highway) is the turn hold's job, not a
  # persistent offset.
  controller, cs = _comp_controller(car)
  _run_comp(controller, cs, 300, meas=-1.0, desired=0.0, lat_accel=1.0)
  assert controller.angle_comp == 0.0
