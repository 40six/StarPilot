"""Bounded SecOC provisioning over the camera-splice ADAS diagnostic path.

Uses card's existing CAN callbacks while Panda is still in ELM327 safety. Bus 1
is the MRR private link; neither it nor OBD multiplexing is used for recovery.
"""
import time

from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.uds import CanClient, IsoTpMessage

DIAG_BUS = 0  # vehicle side of the ADAS splice
RADAR_ADDR = 0x781
RESPONSE_ADDR = 0x789
REQUEST = b"\x22\xef\xf5"
RESPONSE = b"\x62\xef\xf5"
RECOVERY_TIMEOUT = 3.0


def recover_secoc_key(can_recv: CanRecvCallable, can_send: CanSendCallable) -> bytes | None:
  """Read exactly 16 bytes from DID EFF5; GW-sync verification remains mandatory.

  Nonblocking receive and a single absolute deadline also bound repeated pending
  responses and incomplete multi-frame replies. No session changes or retries.
  """
  def recv():
    return [msg for packet in can_recv(wait_for_one=False) for msg in packet]

  def send(addr, data, bus):
    can_send([CanData(addr, data, bus)])

  # Suppress payloads at both CAN and ISO-TP layers, including LOGPRINT=DEBUG.
  client = CanClient(send, recv, RADAR_ADDR, RESPONSE_ADDR, DIAG_BUS, log_data=False)
  message = IsoTpMessage(client, timeout=0, separation_time=0.01)
  deadline = time.monotonic() + RECOVERY_TIMEOUT
  try:
    can_recv(wait_for_one=False)  # discard stale diagnostic replies
    message.send(REQUEST)
    while time.monotonic() < deadline:
      data, _ = message.recv()
      if data is not None:
        if data == b"\x7f\x22\x78":
          # IsoTpMessage accepts a new first frame after a complete pending reply
          # and sends flow control before assembling the consecutive frames.
          continue
        if len(data) == len(RESPONSE) + 16 and data.startswith(RESPONSE):
          return data[len(RESPONSE):]
        break
      time.sleep(0.01)
  except Exception:
    # Exception text/locals may contain ECU payloads. Do not log them.
    pass
  carlog.warning("Fisker SecOC recovery failed; controls remain unavailable")
  return None
