from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.radar_interface import RADAR_HEADER, RADAR_MSG_COUNT
from opendbc.car.fisker.values import CANBUS, CAR, DBC


def make_ri(radar_present=True):
  fingerprint = {i: {} for i in range(8)}
  if radar_present:
    fingerprint[CANBUS.radar][RADAR_HEADER] = 32
  CP = CarInterface.get_params(CAR.FISKER_OCEAN, fingerprint, [], False, False, False, None)
  return CarInterface.RadarInterface(CP), CP


class Cycle:
  """Builds the radar's frames for one cycle; the radar's ns clock and counter advance with each."""
  def __init__(self):
    self.packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.radar])
    self.counter = 0
    self.t = 0

  def frames(self, objs, counter=None, nsec=None):
    counter = self.counter if counter is None else counter
    nsec = self.t if nsec is None else nsec
    msgs = [self.packer.make_can_msg("MRR_Header", CANBUS.radar, {
      "MRR_MeasTime_Sec": 100, "MRR_MeasTime_NSec": nsec, "MRR_CycleCounter": counter, "MRR_NumObjects": len(objs)})]
    for n in range(RADAR_MSG_COUNT):
      sig = {f"MRR_Obj{n:02d}_CycleCounter": counter}
      if n < len(objs):
        sig.update({f"MRR_Obj{n:02d}_{k}": v for k, v in objs[n].items()})
      msgs.append(self.packer.make_can_msg(f"MRR_Obj{n:02d}", CANBUS.radar, sig))
    return msgs

  def step(self, objs, **kw):
    out = self.frames(objs, **kw)
    self.counter = (self.counter + 1) % 64
    self.t += 65_000_000
    return [(self.t, out)]


def obj(id_, age=10, state=3, **kw):
  return {"ID": id_, "Age": age, "State": state, "DistLong": 30.0, "DistLat": 2.0, "VrelLong": -5.0,
          "VrelLat": 1.0, "ArelLong": 0.5, "MeasHistory": 1, **kw}


def test_radar_unavailable_follows_bus1():
  assert not make_ri(True)[1].radarUnavailable
  assert make_ri(False)[1].radarUnavailable


def test_points_and_signs():
  ri, _ = make_ri()
  rr = ri.update(Cycle().step([obj(7)]))
  assert len(rr.points) == 1
  pt = rr.points[0]
  assert abs(pt.dRel - 30.0) < 0.06
  assert abs(pt.yRel - -2.0) < 0.06      # radar +right -> openpilot +left
  assert abs(pt.vRel - -5.0) < 0.07
  assert not rr.errors.canError


def test_skips_empty_and_immature():
  ri, _ = make_ri()
  rr = ri.update(Cycle().step([obj(7), obj(8, age=1), obj(9, state=0), obj(0)]))
  assert [p.trackId for p in rr.points] == [0]


def test_resent_cycle_not_reprocessed():
  ri, _ = make_ri()
  c = Cycle()
  first = ri.update(c.step([obj(7)]))
  assert first.points
  # same MeasTime and counter again: nothing is published
  assert ri.update([(c.t + 1_000_000, c.frames([obj(8)], counter=(c.counter - 1) % 64, nsec=c.t - 65_000_000))]) is None
  # the next cycle carries on from the first one
  assert [p.trackId for p in ri.update(c.step([obj(7, age=11)])).points] == [p.trackId for p in first.points]


def test_stale_slot_dropped():
  ri, _ = make_ri()
  c = Cycle()
  frames = c.frames([obj(7), obj(8)])
  other = c.packer.make_can_msg("MRR_Obj01", CANBUS.radar, {"MRR_Obj01_ID": 8, "MRR_Obj01_Age": 10, "MRR_Obj01_State": 3,
                                                           "MRR_Obj01_CycleCounter": 33})
  frames[2] = other
  rr = ri.update([(1, frames)])
  assert len(rr.points) == 1


def test_track_ids_stable_and_reused_ids_renewed():
  ri, _ = make_ri()
  c = Cycle()
  a = ri.update(c.step([obj(7, age=10)])).points[0].trackId
  assert ri.update(c.step([obj(7, age=11)])).points[0].trackId == a
  assert not ri.update(c.step([])).points
  # the radar's ID wrapped round: same ID, younger age, new object
  b = ri.update(c.step([obj(7, age=4)])).points[0].trackId
  assert b != a
  assert ri.update(c.step([obj(7, age=3)])).points[0].trackId != b


def test_unavailable_radar_sends_nothing_but_periodic_empty():
  ri, CP = make_ri(False)
  assert CP.radarUnavailable
  outs = [ri.update(None) for _ in range(5)]
  assert all(o is None for o in outs[:4]) and hasattr(outs[4], 'points')
