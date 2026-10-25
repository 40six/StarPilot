"""
Cruise state comes from the ADAS module's ACC (ADAS_0x313 / ADAS_0x31C on bus 2), or from the
VCU's basic cruise (VCU_0x358 on bus 0) on cars without ACC coded.
"""

import pytest

from cereal import custom
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.values import CANBUS, CAR, DBC

# ADAS_Sts_ACC_ICC -> (enabled, available, standstill)
ACC_STATES = {
  0: (False, False, False),   # ACC_Off
  1: (False, False, False),   # Initialization
  2: (False, True, False),    # Standby
  3: (True, True, False),     # Active
  4: (True, True, False),     # Override
  5: (True, True, True),      # Standstill_active
  6: (True, True, True),      # Standstill_wait
  7: (False, True, False),    # Deactivation_brake
  8: (False, True, False),    # Deactivation_other
  9: (False, False, False),   # Failure_reversible
  10: (False, False, False),  # Failure_irreversible
  11: (True, True, True),     # Standstill_GoNotification
}


def run_carstate(acc_state: int, set_speed: int, mph: bool, vcu_state: int = 0, vcu_set_speed: int = 255):
  CP = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  FPCP = custom.StarPilotCarParams.new_message()
  CS = CarState(CP, FPCP)
  parsers = CS.get_can_parsers(CP)
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])

  frames = [
    packer.make_can_msg("ADAS_0x313", CANBUS.cam, {"ADAS_Sts_ACC_ICC": acc_state}),
    packer.make_can_msg("ADAS_0x31C", CANBUS.cam, {"ADAS_AccTrgSpdDisp": set_speed}),
    # ACC set speed unit follows the cluster's own unit (ICC_DispVehSpdUnit), not the
    # unreliable ADAS_DispSpdUnit_ACC bit — see carstate.py.
    packer.make_can_msg("ICC_0x531", CANBUS.pt, {"ICC_DispVehSpdUnit": int(mph)}),
    packer.make_can_msg("VCU_0x358", CANBUS.pt, {"VCU_Sts_CC_ICC": vcu_state, "VCU_CcTrgSpdDisp": vcu_set_speed}),
  ]
  for cp in parsers.values():
    cp.update([(0, frames)])
  ret, _ = CS.update(parsers, None)
  return ret


@pytest.mark.parametrize("acc_state", ACC_STATES)
def test_cruise_state_from_adas_acc(acc_state):
  ret = run_carstate(acc_state, 100, mph=False)
  enabled, available, standstill = ACC_STATES[acc_state]
  assert ret.cruiseState.enabled == enabled
  assert ret.cruiseState.available == available
  assert ret.cruiseState.standstill == standstill


@pytest.mark.parametrize("set_speed,mph,expected", [
  (100, False, 100 * CV.KPH_TO_MS),
  (65, True, 65 * CV.MPH_TO_MS),
  (255, False, 0.0),  # no_display
])
def test_cruise_set_speed(set_speed, mph, expected):
  ret = run_carstate(3, set_speed, mph)
  assert ret.cruiseState.speed == pytest.approx(expected)
  assert ret.cruiseState.speedCluster == pytest.approx(expected)


# VCU_Sts_CC_ICC -> (enabled, available); basic cruise has no standstill states
VCU_STATES = {0: (False, False), 1: (False, False), 2: (False, True), 3: (True, True),
              4: (True, True), 9: (False, False), 10: (False, False)}


@pytest.mark.parametrize("vcu_state", VCU_STATES)
def test_cruise_state_from_vcu_basic_cruise(vcu_state):
  ret = run_carstate(0, 255, mph=True, vcu_state=vcu_state, vcu_set_speed=55)
  enabled, available = VCU_STATES[vcu_state]
  assert ret.cruiseState.enabled == enabled
  assert ret.cruiseState.available == available
  assert not ret.cruiseState.standstill
  if available:
    assert ret.cruiseState.speed == pytest.approx(55 * CV.MPH_TO_MS)


def _feed_1d0(cs, parsers, packer, frames):
  """Feed a sequence of (wire, rflag) ADAS_0x1D0 frames and return the reconstructed
  full counters (CS.oem_1d0_secoc_full_ctr after each)."""
  out = []
  for wire, rflag in frames:
    msg = packer.make_can_msg("ADAS_0x1D0", CANBUS.cam,
                              {"ADAS_1D0_SSecOC_Fresh_Byte0": ((wire & 0x3F) << 2) | (rflag & 0x3)})
    for cp in parsers.values():
      cp.update([(0, [msg])])
    cs.update(parsers, None)
    out.append(cs.oem_1d0_secoc_full_ctr)
  return out


def test_secoc_full_counter_restarts_cleanly_across_reset():
  # The OEM 0x1D0 counter runs 1..102 per window (wire 1..63, wrap to 0..38), then the
  # window restarts at wire 1 with a new reset flag. The reconstructed full counter must
  # follow 1..102 then restart at 1 — never gain an extra 64-frame wrap from the wire drop
  # at the restart (the on-vehicle abort=8: delivered counter ran OEM+64).
  CP = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  FPCP = custom.StarPilotCarParams.new_message()
  CS = CarState(CP, FPCP)
  parsers = CS.get_can_parsers(CP)
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])

  # One full window on rflag=2: wire 1..63 (full 1..63), wire 0..38 (full 64..102).
  win = [(w, 2) for w in list(range(1, 64)) + list(range(39))]
  full = _feed_1d0(CS, parsers, packer, win)
  assert full == list(range(1, 103)), full[:5] + ["..."] + full[-3:]

  # Window restart: reset flag flips to 3, wire drops back to 1. Full must restart at 1,
  # not jump to 65 (= 64 + 1), which was the +64 bug.
  restart = _feed_1d0(CS, parsers, packer, [(1, 3), (2, 3), (3, 3)])
  assert restart == [1, 2, 3], restart
