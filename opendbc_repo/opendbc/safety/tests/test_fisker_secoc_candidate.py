#!/usr/bin/env python3
"""
Panda-side proof for the Fisker 0x1D0 SecOC candidate-substitution (race-free handover).

openpilot ships a window of fully-stamped 0x1D0 payloads as 0x5D0 addressed to bus 1.
Panda buffers them keyed by on-wire counter+reset-flag (byte 4) and then rejects them (bus 1
is the MRR's receive-only link, so nothing is ever transmitted there). Instead of forwarding
OEM's own bus-2 0x1D0 to the EPS, substitutes our candidate that matches OEM's CURRENT
byte 4 (fisker_fwd_modify). The EPS therefore rides OEM's own counter with our angle, so
there is no duplicate/backward value at engage or disengage. This test exercises that logic
directly through libsafety.
"""

import unittest  # noqa: TID251 (ported from the sunnypilot Fisker port as-is)
from opendbc.safety.tests.libsafety import libsafety_py

SAFETY_FISKER = 39
ANGLE_ZERO = 12480  # FISKER_ANGLE_ZERO_CAN -> 0 deg
ARM_US = 60000


def _frame(full, angle_raw=ANGLE_ZERO, rflag=0, tag=0):
  """An 0x1D0-shaped payload. byte4 = (wire<<2)|rflag; byte5 is a test marker so we can
  tell whose payload the EPS would receive."""
  wire = full & 0x3F
  d = [0] * 8
  d[2] = (angle_raw >> 8) & 0xFF
  d[3] = angle_raw & 0xFF
  d[4] = ((wire << 2) | (rflag & 3)) & 0xFF
  d[5] = tag & 0xFF
  return d


class TestFiskerSecOCCandidate(unittest.TestCase):
  TX_MSGS = None  # not a PandaSafetyTest; skipped by test_tx_hook_on_wrong_safety_mode
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(SAFETY_FISKER, 0)
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(1_000_000)

  def _cand(self, full, **kw):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x5D0, 1, bytes(_frame(full, **kw))))

  def _oem(self, full, **kw):
    pkt = libsafety_py.make_CANPacket(0x1D0, 2, bytes(_frame(full, **kw)))
    emit = self.safety.safety_fwd_modify_hook(pkt)
    return emit, pkt

  def test_candidates_never_transmitted(self):
    for c in range(10, 15):
      self.assertFalse(self._cand(c, tag=c), "candidates must be buffered, never put on bus 1 (MRR link)")

  def test_armed_substitution(self):
    for c in range(10, 15):
      self._cand(c, tag=c)
    emit, pkt = self._oem(12, tag=0xAA)
    self.assertTrue(emit)
    self.assertEqual(pkt.data[5], 12, "OEM payload should be replaced by our candidate for counter 12")

  def test_no_candidate_drops(self):
    for c in range(10, 15):
      self._cand(c, tag=c)
    emit, _ = self._oem(20, tag=0xAA)   # 20 not buffered
    self.assertFalse(emit, "armed but no matching candidate must drop (gap), never emit a wrong frame")

  def test_wrong_reset_window_drops(self):
    self._cand(12, rflag=0, tag=12)
    emit, _ = self._oem(12, rflag=1, tag=0xAA)  # same wire, different reset flag
    self.assertFalse(emit, "a candidate from the wrong GW-reset window must not be emitted")

  def test_disarmed_passthrough(self):
    self._cand(12, tag=12)
    self.safety.set_timer(1_000_000 + 4 * ARM_US)   # let the candidate go stale
    emit, pkt = self._oem(12, tag=0xAA)
    self.assertTrue(emit)
    self.assertEqual(pkt.data[5], 0xAA, "disarmed must forward OEM's own 0x1D0 unchanged (seamless disengage)")

  def test_bad_angle_candidate_dropped(self):
    self._cand(12, angle_raw=ANGLE_ZERO + 5000, tag=12)  # over the rate limit
    emit, _ = self._oem(12, tag=0xAA)
    self.assertFalse(emit, "a candidate that fails the angle-rate check must never reach the EPS")


if __name__ == "__main__":
  unittest.main()
