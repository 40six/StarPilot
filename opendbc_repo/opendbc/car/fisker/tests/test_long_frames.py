"""
The 0x121 accel frame openpilot injects (create_accel_command) must encode accel on the stock
ADAS module's scale, carry a valid plain-E2E checksum, and signal AccelVld only while openpilot is
actually commanding longitudinal — an idle heartbeat (accel_active=False) sends AccelVld=Init so the
VCU ignores the request. Scale/offset are the DBC's for ADAS_LgtCtrl_AccelReq; the golden raw values
come from stock frames in drives 000000af--28a9f95752--1 and 000000b4--d0f733ebb2--5. openpilot never
replaces the stock 0x117/0x118 content beyond its own status frames. The SecOC tail is stamped
separately (not exercised here).
"""
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.fisker.fiskercan import FiskerCAN, fisker_plain_checksum
from opendbc.car.fisker.values import CAR, DBC


def make():
  return FiskerCAN(None, CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt]))


def test_0x121_zero_accel_matches_stock_raw():
  # accel 0.0 encodes to the stock idle raw value 0x8005 in bytes 2-3.
  _, data, _ = make().create_accel_command(0.0, True, 7)
  assert ((data[2] << 8) | data[3]) == 0x8005
  assert data[1] & 0xF == 7                      # AliveCounter is byte1 low nibble


def test_0x121_accel_scales_on_the_dbc_factor():
  for accel in (-3.5, -0.85, 0.5, 2.0):
    d = make().create_accel_command(accel, True, 0)[1]
    raw = (d[2] << 8) | d[3]
    assert abs(raw * 0.0004882 - 16 - accel) < 0.001


def test_0x121_accel_vld_follows_active():
  # AccelVld (byte1 bits 4-5): 1(Valid) while commanding, 0(Init) on the idle heartbeat.
  assert (make().create_accel_command(0.0, True, 0)[1][1] >> 4) & 0x3 == 1
  assert (make().create_accel_command(0.0, False, 0)[1][1] >> 4) & 0x3 == 0


def test_checksum_valid():
  addr, data, _ = make().create_accel_command(1.0, True, 3)
  assert fisker_plain_checksum(addr, bytes([0]) + data[1:]) == data[0]


def test_accel_cap_matches_panda():
  from opendbc.car.fisker.interface import CarInterface
  from opendbc.car.fisker.values import CarControllerParams
  assert CarControllerParams.ACCEL_MAX == 1.0
  assert CarInterface.get_pid_accel_limits(None, 10.0, 20.0) == (CarControllerParams.ACCEL_MIN, 1.0)
  d = make().create_accel_command(CarControllerParams.ACCEL_MAX, True, 0)[1]
  assert (d[2] << 8) | d[3] == 34822   # FISKER_ACCEL_MAX in safety/modes/fisker.h
