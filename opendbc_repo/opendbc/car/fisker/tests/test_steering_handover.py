import pytest

from cereal import custom
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.fisker.carcontroller import (CarController, LAT_CANDIDATE_ADDR, LAT_CANDIDATE_BUS,
                                              apply_fisker_steer_angle_limits)
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.values import (CANBUS, CAR, DBC, OVERRIDE_RELEASE_FRAMES,
                                        OVERRIDE_RELEASE_TORQUE, OVERRIDE_STEER_TORQUE)
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.fisker.values import CarControllerParams


def candidate_angle(frames):
  """The 0x1D0 angle carried by this tick's candidate window (every candidate carries the same one)."""
  angles = {int.from_bytes(data[2:4], 'big') * .0625 - 780
            for addr, data, bus in frames if addr == LAT_CANDIDATE_ADDR and bus == LAT_CANDIDATE_BUS}
  assert len(angles) == 1
  return angles.pop()


@pytest.fixture
def car():
  cp = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  cp.secOcKeyAvailable = True
  sp = custom.StarPilotCarParams.new_message()
  cs = CarState(cp, sp)
  return cp, sp, cs


@pytest.mark.parametrize('last,measured,speed', [(-1.8125, 3.3125, 18.52), (18.875, 11.6875, 18.78), (46.875, 41.25, 10.4)])
@pytest.mark.parametrize('cruise', [False, True])
def test_override_releases_immediately_without_angle_jump(car, last, measured, speed, cruise):
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp)
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
  assert (messages[0x1C0][3] >> 6) & 3 == 0
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


def test_resume_from_pause_rearms_engage_delay(car):
  # Coming out of a pause (override release or crossing the standstill speed) within an
  # engaged session re-arms the engage delay, so we ease back in instead of yanking the wheel.
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
  controller.update(cc.as_reader(), cs, 0)
  assert controller.engage_delay_remaining >= controller.ENGAGE_DELAY_TICKS - 1


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
  assert controller.engage_delay_remaining >= controller.ENGAGE_DELAY_TICKS - 1   # still held

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
  for _ in range(3):
    assert not candidates_present()
  assert controller._override_eps_recovery

  # EPS back to Available but driver still holding -> stay de-asserted (don't fight).
  cs.eps_lat_sts = 1
  assert not candidates_present()
  assert controller._override_eps_recovery

  # EPS Available AND driver released -> re-arm the engage delay for a fresh handshake.
  cs.out.steeringPressed = False
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
