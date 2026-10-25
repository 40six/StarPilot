#!/usr/bin/env python3
import unittest  # noqa: TID251 (ported from the sunnypilot Fisker port as-is)

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py

ICC_SETTINGS = 0x52A
ICC_FRAME = bytes.fromhex("BE2026A043008220")
TIMEOUT_US = 500_000


class TestFiskerIccSettings(unittest.TestCase):
  """ICC_0x52A: openpilot re-sends the ICC's frame on bus 2; panda drops the ICC's original
  (bus 0 -> 2) only while openpilot's copies keep arriving."""
  TX_MSGS = None  # not a PandaSafetyTest; skipped by test_tx_hook_on_wrong_safety_mode

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)  # init_tests() also resets the timer to 0; keep init's timestamp consistent
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _tx(self, bus):
    return self.safety.safety_tx_hook(common.make_msg(bus, ICC_SETTINGS, dat=ICC_FRAME))

  def _fwd(self, bus):
    return self.safety.safety_fwd_hook(bus, ICC_SETTINGS)

  def test_tx_allowed_on_adas_bus_only(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(2))
      self.assertFalse(self._tx(0))
      self.assertFalse(self._tx(1))

  def test_tx_allowed_with_long_flag(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 1)
    self.assertTrue(self._tx(2))

  def test_blocked_at_init(self):
    # openpilot is already sending when the safety mode starts: block from the first ICC frame
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(TIMEOUT_US - 1)
    self.assertEqual(self._fwd(0), -1)

  def test_forwarded_if_openpilot_never_sends(self):
    self.safety.set_timer(TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

  def test_blocked_while_openpilot_sends(self):
    # each of our frames extends the window past the init window
    for t in range(0, 5_000_000, 200_000):
      self.safety.set_timer(t)
      self.assertTrue(self._tx(2))
      self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US - 1)
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

  def test_fallback_to_icc_when_openpilot_stops(self):
    self.assertTrue(self._tx(2))
    self.safety.set_timer(TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

    # resumes blocking as soon as openpilot sends again
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(0), -1)

  def test_window_restarts_on_safety_mode_init(self):
    self.safety.set_timer(2 * TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.assertEqual(self._fwd(0), -1)

  def test_adas_side_not_blocked(self):
    # only the ICC -> ADAS direction is intercepted
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(2), 0)

  def test_no_relay_malfunction(self):
    # 0x52A on bus 2 must not trip relay malfunction (that would stop all forwarding)
    self.safety.safety_rx_hook(common.make_msg(2, ICC_SETTINGS, dat=ICC_FRAME))
    self.assertFalse(self.safety.get_relay_malfunction())


class TestFiskerIcc0x35B(unittest.TestCase):
  """ICC_0x35B: optional relay — the ICC's original is only blocked once openpilot's copies are
  actually arriving on bus 2, and forwarding resumes if they stop."""
  TX_MSGS = None  # not a PandaSafetyTest; skipped by test_tx_hook_on_wrong_safety_mode

  ADDR = 0x35B

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)  # init_tests() also resets the timer to 0; keep init's timestamp consistent
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _tx(self, bus):
    return self.safety.safety_tx_hook(common.make_msg(bus, self.ADDR, 8))

  def _fwd(self, bus):
    return self.safety.safety_fwd_hook(bus, self.ADDR)

  def test_tx_allowed_on_adas_bus_only(self):
    for param in (0, 1):
      self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, param)
      self.assertTrue(self._tx(2))
      self.assertFalse(self._tx(0))
      self.assertFalse(self._tx(1))

  def test_forwarded_while_openpilot_not_relaying(self):
    # no overrides configured -> openpilot never sends 0x35B -> ICC's frame always forwarded
    for t in (0, TIMEOUT_US, 10 * TIMEOUT_US):
      self.safety.set_timer(t)
      self.assertEqual(self._fwd(0), 2)

  def test_blocked_while_openpilot_relays(self):
    for t in range(0, 5_000_000, 200_000):
      self.safety.set_timer(t)
      self.assertTrue(self._tx(2))
      self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US - 1)
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

  def test_reset_on_safety_mode_init(self):
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.assertEqual(self._fwd(0), 2)

  def test_independent_of_0x52a(self):
    # relaying 0x52A doesn't block 0x35B, and vice versa
    self.assertTrue(self.safety.safety_tx_hook(common.make_msg(2, ICC_SETTINGS, dat=ICC_FRAME)))
    self.assertEqual(self._fwd(0), 2)
    self.safety.set_timer(TIMEOUT_US)
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(0), -1)
    self.assertEqual(self.safety.safety_fwd_hook(0, ICC_SETTINGS), 2)

  def test_adas_side_not_blocked(self):
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(2), 0)


class TestFiskerCruiseState(unittest.TestCase):
  """Cruise engagement follows the ADAS module's ACC state (ADAS_0x313, bus 2)."""
  TX_MSGS = None  # not a PandaSafetyTest; skipped by test_tx_hook_on_wrong_safety_mode

  # ADAS_Sts_ACC_ICC: Active, Override, Standstill_active, Standstill_wait, Standstill_GoNotification
  ENGAGED_STATES = {3, 4, 5, 6, 11}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _state_msg(self, addr, bus, acc_state):
    dat = bytearray(8)
    dat[4] = acc_state & 0x0F  # 4-bit @ Motorola start 35 -> data[4] low nibble
    return common.make_msg(bus, addr, dat=bytes(dat))

  def _rx_acc_state(self, acc_state, addr=0x313, bus=2):
    self.safety.safety_rx_hook(self._state_msg(addr, bus, acc_state))

  def test_engaged_states(self):
    for acc_state in range(16):
      with self.subTest(acc_state=acc_state):
        self._rx_acc_state(0)
        self.assertFalse(self.safety.get_controls_allowed())
        self._rx_acc_state(acc_state)
        self.assertEqual(self.safety.get_controls_allowed(), acc_state in self.ENGAGED_STATES)

  def test_disengage_on_acc_exit(self):
    for acc_state in set(range(16)) - self.ENGAGED_STATES:
      with self.subTest(acc_state=acc_state):
        self._rx_acc_state(0)
        self._rx_acc_state(3)
        self.assertTrue(self.safety.get_controls_allowed())
        self._rx_acc_state(acc_state)
        self.assertFalse(self.safety.get_controls_allowed())

  def test_stays_engaged_through_standstill(self):
    self._rx_acc_state(0)
    for acc_state in (3, 5, 6, 11, 3, 4):
      self._rx_acc_state(acc_state)
      self.assertTrue(self.safety.get_controls_allowed())

  def test_other_sources_ignored(self):
    # 0x313 only counts from the ADAS side (bus 2), 0x358 only from the vehicle side (bus 0)
    for addr, bus in ((0x358, 2), (0x313, 0)):
      with self.subTest(addr=hex(addr), bus=bus):
        self._rx_acc_state(0)
        self._rx_acc_state(3, addr=addr, bus=bus)
        self.assertFalse(self.safety.get_controls_allowed())

  def test_vcu_basic_cruise_engages(self):
    # Cars without ACC coded: ADAS stays Off, VCU basic cruise (0x358, bus 0) engages
    for vcu_state in range(16):
      with self.subTest(vcu_state=vcu_state):
        self._rx_acc_state(0)
        self._rx_acc_state(0, addr=0x358, bus=0)
        self.assertFalse(self.safety.get_controls_allowed())
        self._rx_acc_state(vcu_state, addr=0x358, bus=0)
        self.assertEqual(self.safety.get_controls_allowed(), vcu_state in (3, 4))

  def test_vcu_basic_cruise_standby_is_acc_main(self):
    for vcu_state in range(16):
      with self.subTest(vcu_state=vcu_state):
        self._rx_acc_state(0)
        self._rx_acc_state(vcu_state, addr=0x358, bus=0)
        self.assertEqual(self.safety.get_acc_main_on(), vcu_state not in (0, 1, 9, 10))

  def test_adas_off_does_not_drop_vcu_cruise(self):
    self._rx_acc_state(3, addr=0x358, bus=0)
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx_acc_state(0)  # ADAS ACC (not coded) keeps reporting Off
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx_acc_state(2, addr=0x358, bus=0)  # brake -> Standby
    self.assertFalse(self.safety.get_controls_allowed())


class TestFiskerAlwaysOnLateral(unittest.TestCase):
  """StarPilot port: Always On Lateral replaces sunnypilot MADS. acc_main_on follows the ADAS
  ACC state on bus 2 (>= Standby), and the MFS_RiBtnSouth LONG press (MFS_0x514) toggles
  lkas_on while ACC main is on. aol_allowed = (acc_main_on || lkas_on) && ALT_EXP flag."""
  TX_MSGS = None  # not a PandaSafetyTest; skipped by test_tx_hook_on_wrong_safety_mode

  ALT_EXP_ALWAYS_ON_LATERAL = 32

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()
    self.safety.set_alternative_experience(self.ALT_EXP_ALWAYS_ON_LATERAL)

  def _acc(self, state):
    dat = bytearray(8)
    dat[4] = state & 0x0F
    self.assertTrue(self.safety.safety_rx_hook(common.make_msg(2, 0x313, dat=bytes(dat))))

  def _button(self, rbs):
    dat = bytearray(8)
    dat[4] = rbs & 0x03
    self.assertTrue(self.safety.safety_rx_hook(common.make_msg(0, 0x514, dat=bytes(dat))))

  def test_acc_main_follows_adas_state(self):
    for state in range(12):
      with self.subTest(state=state):
        self._acc(state)
        self.assertEqual(self.safety.get_acc_main_on(), state not in (0, 1, 9, 10))

  def test_lkas_long_press_toggles_only_with_acc_main(self):
    self._acc(0)
    self._button(2)
    self._button(0)
    self.assertFalse(self.safety.get_lkas_on())

    self._acc(2)  # Standby
    self._button(2)
    self.assertTrue(self.safety.get_lkas_on())
    self._button(2)  # held: no re-toggle
    self.assertTrue(self.safety.get_lkas_on())
    self._button(0)
    self._button(2)
    self.assertFalse(self.safety.get_lkas_on())

  def test_short_press_ignored(self):
    self._acc(2)
    for _ in range(3):
      self._button(1)  # short press = car's follow-distance adjust
      self._button(0)
    self.assertFalse(self.safety.get_lkas_on())

  def test_acc_off_clears_lkas(self):
    self._acc(2)
    self._button(2)
    self._button(0)
    self.assertTrue(self.safety.get_lkas_on())
    self._acc(0)
    self.assertFalse(self.safety.get_lkas_on())

  def test_steering_needs_aol_or_cruise(self):
    self._acc(2)  # Standby: AOL allowed via acc_main_on
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.get_aol_allowed())
    self.safety.set_alternative_experience(0)
    self._acc(2)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_aol_allowed())


if __name__ == "__main__":
  unittest.main()
