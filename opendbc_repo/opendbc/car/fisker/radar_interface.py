from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.fisker.values import CANBUS, DBC

# Mid-range radar (MRR) on its private CAN-FD link. Every 65 ms cycle brings a header (0x300) and 32 object
# slots (0x310..0x32F, filled from the first slot up; empty slots carry ID 0). Slots are sent after the header,
# so the last slot completes a cycle.
RADAR_HEADER = 0x300
RADAR_START_ADDR = 0x310
RADAR_MSG_COUNT = 32

# The radar tracks objects it saw for as little as a few cycles and keeps them a while after losing them.
# State rises 0->3 as a track matures and falls to 0 right before deletion; Age is in 65 ms cycles.
MIN_STATE = 1
MIN_AGE_CYCLES = 3


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [("MRR_Header", 15)] + [(f"MRR_Obj{n:02d}", 15) for n in range(RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CANBUS.radar)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

    self.last_meas: tuple[int, int, int] | None = None
    self.ids: dict[int, tuple[int, int]] = {}   # radar track ID -> (our trackId, last Age)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    hdr = self.rcp.vl["MRR_Header"]
    # the radar now and then sends a cycle again (same MeasTime and counter): a cycle measured once is handled once
    meas = (int(hdr["MRR_MeasTime_Sec"]), int(hdr["MRR_MeasTime_NSec"]), int(hdr["MRR_CycleCounter"]))
    if meas == self.last_meas:
      return None
    self.last_meas = meas

    seen = set()
    for n in range(RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"MRR_Obj{n:02d}"]
      p = f"MRR_Obj{n:02d}_"

      rid = int(msg[p + "ID"])
      if rid == 0 or int(msg[p + "CycleCounter"]) != meas[2]:   # empty slot, or one left over from another cycle
        continue
      if msg[p + "State"] < MIN_STATE or msg[p + "Age"] < MIN_AGE_CYCLES:
        continue

      age = int(msg[p + "Age"])
      # the radar's ID wraps; a track whose age went back down is a different object
      if rid not in self.ids or age < self.ids[rid][1]:
        if rid in self.ids:
          self.pts.pop(self.ids[rid][0], None)
        self.ids[rid] = (self.track_id, age)
        self.track_id += 1
      track_id = self.ids[rid][0]
      self.ids[rid] = (track_id, age)
      seen.add(rid)

      pt = self.pts.get(track_id)
      if pt is None:
        pt = self.pts[track_id] = structs.RadarData.RadarPoint()
        pt.trackId = track_id
      pt.dRel = msg[p + "DistLong"]
      pt.yRel = -msg[p + "DistLat"]        # the radar's lateral axis is +right, openpilot's +left
      pt.vRel = msg[p + "VrelLong"]

    for rid in list(self.ids):
      if rid not in seen:
        self.pts.pop(self.ids.pop(rid)[0], None)

    ret.points = list(self.pts.values())
    return ret
