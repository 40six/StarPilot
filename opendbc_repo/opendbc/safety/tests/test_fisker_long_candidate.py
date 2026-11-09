#!/usr/bin/env python3
"""
Panda-side proof for the Fisker 0x121 accel candidate substitution (openpilot long inside the
OEM ADAS's own ACC session).

openpilot ships stamped 0x121 payloads as 0x5A1 on bus 1. Panda buffers them keyed by byte 4
(wire counter + reset flag), never transmits them, and swaps the matching one into the OEM
ADAS's own bus-2 0x121 as it is forwarded to the vehicle. Unlike 0x1D0 a miss is never a gap:
the ADAS's own 0x121 is forwarded unchanged whenever there is no fresh, in-limit, matching
candidate or controls aren't allowed.
"""

import unittest  # noqa: TID251 (same style as test_fisker_secoc_candidate.py)
from opendbc.safety.tests.libsafety import libsafety_py

SAFETY_FISKER = 39
FLAG_LONG = 1
ARM_US = 60000


def _accel_raw(accel):
  return round((accel + 16) / 0.0004882)


def _frame(full, accel=0.0, rflag=0, tag=0):
  """An 0x121-shaped payload. byte4 = (wire<<2)|rflag; byte5 is a test marker so we can tell
  whose payload the vehicle would receive."""
  raw = _accel_raw(accel)
  d = [0] * 8
  d[2] = (raw >> 8) & 0xFF
  d[3] = raw & 0xFF
  d[4] = (((full & 0x3F) << 2) | (rflag & 3)) & 0xFF
  d[5] = tag & 0xFF
  return d


class TestFiskerLongCandidate(unittest.TestCase):
  TX_MSGS = None  # not a PandaSafetyTest; skipped by test_tx_hook_on_wrong_safety_mode

  def setUp(self, param=FLAG_LONG):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(SAFETY_FISKER, param)
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(1_000_000)

  def _cand(self, full, **kw):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x5A1, 1, bytes(_frame(full, **kw))))

  def _oem(self, full, **kw):
    pkt = libsafety_py.make_CANPacket(0x121, 2, bytes(_frame(full, **kw)))
    emit = self.safety.safety_fwd_modify_hook(pkt)
    return emit, pkt

  def test_candidates_never_transmitted(self):
    for c in range(10, 15):
      self.assertFalse(self._cand(c, accel=-1.0, tag=c), "candidates must be buffered, never put on bus 1 (MRR link)")

  def test_armed_substitution(self):
    for c in range(10, 15):
      self._cand(c, accel=-1.0, tag=c)
    emit, pkt = self._oem(12, tag=0xAA)
    self.assertTrue(emit)
    self.assertEqual(pkt.data[5], 12, "the ADAS's 0x121 should carry our candidate for counter 12")

  def test_no_candidate_forwards_oem(self):
    for c in range(10, 15):
      self._cand(c, tag=c)
    emit, pkt = self._oem(20, tag=0xAA)   # 20 not buffered
    self.assertTrue(emit, "0x121 is never dropped: the stock ACC request goes through instead")
    self.assertEqual(pkt.data[5], 0xAA)

  def test_wrong_reset_window_forwards_oem(self):
    self._cand(12, rflag=0, tag=12)
    emit, pkt = self._oem(12, rflag=1, tag=0xAA)
    self.assertTrue(emit)
    self.assertEqual(pkt.data[5], 0xAA, "a candidate from the wrong GW-reset window must not be emitted")

  def test_stale_candidate_forwards_oem(self):
    self._cand(12, tag=12)
    self.safety.set_timer(1_000_000 + 4 * ARM_US)
    emit, pkt = self._oem(12, tag=0xAA)
    self.assertTrue(emit)
    self.assertEqual(pkt.data[5], 0xAA)

  def test_controls_not_allowed_forwards_oem(self):
    self._cand(12, tag=12)
    self.safety.set_controls_allowed(False)
    emit, pkt = self._oem(12, tag=0xAA)
    self.assertTrue(emit)
    self.assertEqual(pkt.data[5], 0xAA, "without controls_allowed the ADAS's own request must reach the vehicle")

  def test_out_of_limit_accel_forwards_oem(self):
    for accel in (-4.0, 2.5):   # outside FISKER_ACCEL_MIN/MAX (-3.5 .. +2.0)
      self._cand(12, accel=accel, tag=12)
      emit, pkt = self._oem(12, tag=0xAA)
      self.assertTrue(emit)
      self.assertEqual(pkt.data[5], 0xAA, f"accel {accel} is out of limits and must not be substituted")

  def test_candidate_captured_without_controls_is_not_used(self):
    self.safety.set_controls_allowed(False)
    self._cand(12, accel=-1.0, tag=12)   # only the inactive accel is allowed without controls
    self.safety.set_controls_allowed(True)
    emit, pkt = self._oem(12, tag=0xAA)
    self.assertTrue(emit)
    self.assertEqual(pkt.data[5], 0xAA)

  def test_lateral_only_mode_ignores_candidates(self):
    self.setUp(param=0)
    self.assertFalse(self._cand(12, tag=12))
    emit, pkt = self._oem(12, tag=0xAA)
    self.assertTrue(emit)
    self.assertEqual(pkt.data[5], 0xAA, "without the long flag panda never substitutes 0x121")

  def test_0x117_0x118_never_blocked(self):
    for addr in (0x117, 0x118):
      self.assertEqual(self.safety.safety_fwd_hook(2, addr), 0, f"OEM 0x{addr:X} must always be forwarded to the vehicle")
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x121), 0, "OEM 0x121 must reach fwd_modify, never be blocked")


if __name__ == "__main__":
  unittest.main()
