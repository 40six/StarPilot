import unittest  # noqa: TID251 (ported from the sunnypilot Fisker port as-is)
from unittest.mock import patch  # noqa: TID251

from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.fisker.secoc_key import recover_secoc_key

# Synthetic bytes only; never a vehicle key.
KEY = bytes(range(16))


class Radar:
  def __init__(self, payload=b'\x62\xef\xf5' + KEY, pending=True, complete=True, bus=0, bad_sequence=False):
    self.payload = payload
    self.pending = pending
    self.complete = complete
    self.bus = bus
    self.bad_sequence = bad_sequence
    self.queue = []
    self.sent = []

  def recv(self, wait_for_one=False):
    assert not wait_for_one
    frames, self.queue = self.queue, []
    return [frames] if frames else []

  def send(self, messages):
    self.sent.extend(messages)
    for addr, data, bus in messages:
      assert (addr, bus) == (0x781, 0)
      if data[:4] == b'\x03\x22\xef\xf5':
        if self.pending:
          self.queue.append(CanData(0x789, b'\x03\x7f\x22\x78\x00\x00\x00\x00', self.bus))
        self.queue.append(CanData(0x789, (bytes([0x10, len(self.payload)]) + self.payload[:6]).ljust(8, b'\x00'), self.bus))
      elif data[0] == 0x30 and self.complete:
        for seq, offset in enumerate(range(6, len(self.payload), 7), 1):
          index = seq + int(self.bad_sequence)
          self.queue.append(CanData(0x789, (bytes([0x20 | index]) + self.payload[offset:offset + 7]).ljust(8, b'\x00'), self.bus))


class TestSecocRecovery(unittest.TestCase):
  def recover(self, radar):
    # Exercise the actual ISO-TP implementation, with a short deadline for failures.
    with patch('opendbc.car.fisker.secoc_key.RECOVERY_TIMEOUT', 0.05):
      return recover_secoc_key(radar.recv, radar.send)

  def test_pending_multiframe_and_flow_control(self):
    radar = Radar()
    self.assertEqual(self.recover(radar), KEY)
    self.assertEqual([m.dat[0] for m in radar.sent], [0x03, 0x30])

  def test_exact_length_and_did(self):
    for payload in (b'\x62\xef\xf5' + KEY[:-1], b'\x62\xef\xf5' + KEY + b'\x00', b'\x62\xef\xf4' + KEY):
      with self.subTest(length=len(payload)):
        self.assertIsNone(self.recover(Radar(payload=payload)))

  def test_missing_consecutive_frames(self):
    self.assertIsNone(self.recover(Radar(complete=False)))

  def test_wrong_bus(self):
    self.assertIsNone(self.recover(Radar(bus=1)))

  def test_bad_sequence(self):
    self.assertIsNone(self.recover(Radar(bad_sequence=True)))

  def test_silent_bus_and_repeated_pending(self):
    for frames in ([], [CanData(0x789, b'\x03\x7f\x22\x78\x00\x00\x00\x00', 0)]):
      self.assertIsNone(recover_secoc_key_with_deadline(frames))

  def test_transport_error_is_safe(self):
    def fail(**kwargs):
      raise RuntimeError('sensitive ECU response')
    with self.assertLogs(carlog, level='DEBUG') as logs:
      self.assertIsNone(recover_secoc_key(fail, lambda msgs: None))
    self.assertNotIn('sensitive', '\n'.join(logs.output))

  def test_debug_logs_do_not_expose_payloads(self):
    with self.assertLogs(carlog, level='DEBUG') as logs:
      self.assertEqual(self.recover(Radar()), KEY)
    text = '\n'.join(logs.output)
    self.assertNotIn(KEY.hex(), text)
    self.assertNotIn('000102', text)
    self.assertNotIn('03040506070809', text)
    self.assertNotIn('0a0b0c0d0e0f', text)


def recover_secoc_key_with_deadline(frames):
  with patch('opendbc.car.fisker.secoc_key.RECOVERY_TIMEOUT', 0.02):
    return recover_secoc_key(lambda **kwargs: [frames], lambda msgs: None)


if __name__ == '__main__':
  unittest.main()
