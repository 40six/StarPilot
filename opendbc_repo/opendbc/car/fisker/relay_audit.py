"""
Checks the vehicle-bus stream of one relayed ADAS message (0x1D0, 0x1C0 or 0x121) against the relay
invariants (see safety/modes/fisker.h and relay.py):

  I1 one source:        stock and openpilot frames never interleave; the source changes only at a
                        takeover (stock -> ours) or a release (ours -> stock)
  I2 counter continuity: the alive counter is strictly +1 (mod 15) inside a source, and across a
                        hand-off it is at worst a small bounded step forward, never a repeat or a step back
                        (a skipped counter inside the stock stream is only noted: frames missing from the log)
  gap                   no more than a few stock periods between consecutive frames

Used by the replay test of the C arbiter (safety/tests/test_fisker_relay_replay.py) and by the drive-log
audit (openpilot/sunnypilot/webhud/tools/fisker_handoff_audit.py), so the same rules judge the simulation
and the car.
"""
from dataclasses import dataclass, field

STOCK, OURS = "stock", "ours"

MAX_GAP_S = 0.045         # stock period is 10 ms; the arbiter's fail-safe is 50 ms
TAKEOVER_MAX_DELTA = 3    # first of ours vs the last stock frame: repeat (0) up to 3 ahead
RELEASE_MAX_DELTA = 4     # first stock frame back vs our last: +1 exactly, or up to 3 skipped


@dataclass
class BusFrame:
  t: float       # seconds
  source: str    # STOCK or OURS
  ctr: int       # AliveCounter 0..14


@dataclass
class Finding:
  kind: str
  t: float
  detail: str


@dataclass
class AuditResult:
  violations: list[Finding] = field(default_factory=list)   # break an invariant
  notes: list[Finding] = field(default_factory=list)        # bounded, expected at a hand-off (a repeat or a small skip)
  takeovers: int = 0
  releases: int = 0
  max_gap_s: float = 0.0

  @property
  def ok(self) -> bool:
    return not self.violations


def delta(a: int, b: int) -> int:
  """b - a on the 0..14 alive counter"""
  return (b - a) % 15


def audit_stream(frames: list[BusFrame], max_gap_s: float = MAX_GAP_S, min_run: int = 3, failsafe_ok: bool = False) -> AuditResult:
  """`failsafe_ok`: a release that happened because openpilot went silent (not a planned release) may
  jump further, since panda hands the stock stream back after 50 ms regardless of its counter."""
  res = AuditResult()
  # run lengths, to catch flicker (a source that shows up for only a frame or two between others)
  runs: list[list] = []
  for f in frames:
    if runs and runs[-1][0] == f.source:
      runs[-1][1] += 1
    else:
      runs.append([f.source, 1, f.t])
  for i, (src, n, t) in enumerate(runs):
    if 0 < i < len(runs) - 1 and n < min_run:
      res.violations.append(Finding("interleave", t, f"{src} ran for only {n} frame(s) between {runs[i - 1][0]} and {runs[i + 1][0]}"))

  for a, b in zip(frames, frames[1:], strict=False):
    d = delta(a.ctr, b.ctr)
    gap = b.t - a.t
    res.max_gap_s = max(res.max_gap_s, gap)
    if gap > max_gap_s:
      res.violations.append(Finding("gap", b.t, f"{gap * 1e3:.0f} ms between frames"))
    if a.source == b.source:
      if d == 1:
        continue
      kind = "repeat" if d == 0 else ("step_back" if d >= 5 else "jump")
      if a.source == STOCK and kind == "jump":
        # frames missing from the stock stream (the log's echo drops a frame now and then): not something openpilot did
        res.notes.append(Finding("stock_gap", b.t, f"stock counter {a.ctr} -> {b.ctr}"))
      else:
        res.violations.append(Finding(f"{a.source}_{kind}", b.t, f"counter {a.ctr} -> {b.ctr}"))
    elif a.source == STOCK and b.source == OURS:
      res.takeovers += 1
      if d > TAKEOVER_MAX_DELTA:
        res.violations.append(Finding("takeover_jump", b.t, f"stock {a.ctr} -> ours {b.ctr}"))
      elif d != 1:
        res.notes.append(Finding("takeover_repeat" if d == 0 else "takeover_skip", b.t, f"stock {a.ctr} -> ours {b.ctr}"))
    else:
      res.releases += 1
      if d > RELEASE_MAX_DELTA or d == 0:
        if not (failsafe_ok and d != 0):
          res.violations.append(Finding("release_jump" if d else "release_repeat", b.t, f"ours {a.ctr} -> stock {b.ctr}"))
      elif d != 1:
        res.notes.append(Finding("release_skip", b.t, f"ours {a.ctr} -> stock {b.ctr}"))
  return res
