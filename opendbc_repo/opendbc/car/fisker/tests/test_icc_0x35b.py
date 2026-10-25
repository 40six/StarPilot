"""
ICC_0x35B (SVS view requests + BSD/DOW/APA settings): optionally re-sent on bus 2 with
ICC_0x35B_OVERRIDES applied; the camera-view signals in ICC_0x35B_PASSTHROUGH never change.
"""

import random

import pytest

from cereal import custom
from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC as DBCFile
from opendbc.car import structs
from opendbc.car.fisker import carcontroller, fiskercan
from opendbc.car.fisker.fiskercan import FiskerCAN
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.values import CANBUS, CAR, ICC_0x35B_OVERRIDES, ICC_0x35B_PASSTHROUGH

DBC_NAME = "fisker_ocean_adas"
MSG = "ICC_0x35B"
SIGNALS = DBCFile(DBC_NAME).name_to_msg[MSG].sigs

EXAMPLE_OVERRIDES = {"ICC_BSDSetting": 2, "ICC_DOW_Setting": 2, "ICC_AHBA_Setting": 1, "ICC_BSD_Sensitivity": 1}

rng = random.Random(0x35B)
RANDOM_FRAMES = [bytes(rng.getrandbits(8) for _ in range(8)) for _ in range(50)]


def decode(frames: list[bytes], bus: int = CANBUS.pt) -> list[dict[str, float]]:
  cp = CANParser(DBC_NAME, [(MSG, float('nan'))], bus)
  cp.update([(i * 200_000_000, [(0x35B, f, bus)]) for i, f in enumerate(frames)])
  msgs = cp.vl_all[MSG]
  return [{sig: vals[i] for sig, vals in msgs.items()} for i in range(len(frames))]


@pytest.fixture
def overrides(monkeypatch):
  monkeypatch.setattr(fiskercan, "ICC_0x35B_OVERRIDES", EXAMPLE_OVERRIDES)
  monkeypatch.setattr(carcontroller, "ICC_0x35B_OVERRIDES", EXAMPLE_OVERRIDES)


def test_every_bit_round_trips():
  # all 64 bits are defined (incl. the undocumented bits 0..11), so any payload survives a repack
  packer = CANPacker(DBC_NAME)
  for frame, values in zip(RANDOM_FRAMES, decode(RANDOM_FRAMES), strict=True):
    assert packer.make_can_msg(MSG, CANBUS.pt, values)[1] == frame


def test_override_config_valid():
  assert ICC_0x35B_PASSTHROUGH <= SIGNALS.keys()
  for cfg in (ICC_0x35B_OVERRIDES, EXAMPLE_OVERRIDES):
    assert not (cfg.keys() & ICC_0x35B_PASSTHROUGH)
    for sig, raw in cfg.items():
      assert sig in SIGNALS, sig
      assert 0 <= raw < (1 << SIGNALS[sig].size), sig


def test_create_icc_0x35b(overrides):
  fcan = FiskerCAN(None, CANPacker(DBC_NAME))
  for src in decode(RANDOM_FRAMES):
    addr, data, bus = fcan.create_icc_0x35b(src)
    assert (addr, bus, len(data)) == (0x35B, CANBUS.cam, 8)

    out = decode([data], bus=CANBUS.cam)[0]
    for sig, val in src.items():
      assert out[sig] == EXAMPLE_OVERRIDES.get(sig, val), sig
    for sig in ICC_0x35B_PASSTHROUGH:
      assert out[sig] == src[sig], sig


def _controller_sends(icc_frames):
  CP = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  CI = CarInterface(CP, custom.StarPilotCarParams.new_message())
  CI.CS.icc_0x35b_frames = icc_frames
  _, can_sends = CI.apply(structs.CarControl().as_reader(), 0, None)
  return [msg for msg in can_sends if msg[0] == 0x35B]


def test_not_relayed_without_overrides(monkeypatch):
  monkeypatch.setattr(carcontroller, "ICC_0x35B_OVERRIDES", {})
  assert _controller_sends(decode(RANDOM_FRAMES[:3])) == []


def test_relayed_one_per_icc_frame(overrides):
  src = decode(RANDOM_FRAMES[:3])
  sends = _controller_sends(src)
  assert len(sends) == 3
  for (_, data, bus), icc in zip(sends, src, strict=True):
    assert bus == CANBUS.cam
    assert decode([data], bus=CANBUS.cam)[0]["ICC_ViewReq"] == icc["ICC_ViewReq"]
