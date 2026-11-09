"""
The cached single-block AES-CMAC in secoc.py must be bit-identical to a full RFC 4493 CMAC
(pycryptodome's), for every input length — a wrong MAC is a frame the gateway/EPS/VCU drops.
"""
import os
import random

from Crypto.Cipher import AES
from Crypto.Hash import CMAC

from opendbc.car.fisker.secoc import _cmac24, secoc_mac, stamp_secoc, sync_mac


def _reference(key: bytes, data: bytes) -> bytes:
  c = CMAC.new(key, ciphermod=AES)
  c.update(data)
  return c.digest()[:3]


def test_rfc4493_vectors():
  # RFC 4493 §4 example key and messages (lengths 0, 16, 40, 64)
  key = bytes.fromhex("2b7e151628aed2a6abf7158809cf4f3c")
  msg = bytes.fromhex("6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51"
                      "30c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710")
  for length, mac in ((0, "bb1d69"), (16, "070a16"), (40, "dfa667"), (64, "51f0be")):
    assert _cmac24(key, msg[:length]).hex() == mac


def test_matches_reference_for_all_lengths():
  rng = random.Random(1234)
  for _ in range(20):
    key = bytes(rng.getrandbits(8) for _ in range(16))
    for length in range(0, 49):
      data = bytes(rng.getrandbits(8) for _ in range(length))
      assert _cmac24(key, data) == _reference(key, data), (key.hex(), length)


def test_secoc_frames_match_reference():
  key = os.urandom(16)
  frame = bytes.fromhex("40d5800563498658")
  for counter in (1, 63, 64, 101):
    for can_id in (0x121, 0x1D0):
      stamped = stamp_secoc(key, can_id, frame, 0x1234, 0x56789A, counter)
      fresh = (0x1234).to_bytes(2, "big") + (0x56789A).to_bytes(3, "big") + counter.to_bytes(2, "big") + bytes([(0x56789A & 3) << 6])
      assert stamped[5:8] == _reference(key, can_id.to_bytes(2, "big") + stamped[:4] + fresh)
      assert secoc_mac(key, can_id, stamped[:4], 0x1234, 0x56789A, counter) == stamped[5:8]
  assert sync_mac(key, 0x1234, 0x56789A) == _reference(key, bytes.fromhex("0020") + (0x1234).to_bytes(2, "big") + (0x56789A).to_bytes(3, "big"))


def test_key_change_uses_new_key():
  a, b = os.urandom(16), os.urandom(16)
  data = bytes(range(14))
  assert _cmac24(a, data) == _reference(a, data)
  assert _cmac24(b, data) == _reference(b, data)
  assert _cmac24(a, data) == _reference(a, data)
