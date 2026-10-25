from opendbc.car.fisker.relay_audit import OURS, STOCK, BusFrame, audit_stream


def frames(spec, t0=0.0, dt=0.01):
  """spec: [(source, counter), ...] at a steady 10 ms"""
  return [BusFrame(t0 + i * dt, s, c) for i, (s, c) in enumerate(spec)]


def kinds(res):
  return sorted({v.kind for v in res.violations})


def seq(source, start, n):
  return [(source, (start + i) % 15) for i in range(n)]


def test_clean_handoffs():
  # stock, takeover continuing the counter, release resuming at ours + 1
  spec = seq(STOCK, 3, 5) + seq(OURS, 8, 30) + seq(STOCK, 8 + 30, 10)
  res = audit_stream(frames(spec))
  assert res.ok and not res.notes and (res.takeovers, res.releases) == (1, 1)


def test_bounded_steps_are_notes_not_violations():
  spec = seq(STOCK, 3, 5) + seq(OURS, 9, 30) + seq(STOCK, 9 + 30 + 2, 10)   # skip 1 at takeover, 2 at release
  res = audit_stream(frames(spec))
  assert res.ok and {n.kind for n in res.notes} == {"takeover_skip", "release_skip"}
  spec = seq(STOCK, 3, 5) + seq(OURS, 7, 30) + seq(STOCK, 7 + 30, 10)       # a repeat of the last stock frame at takeover
  assert {n.kind for n in audit_stream(frames(spec)).notes} == {"takeover_repeat"}


def test_flags_counter_errors_inside_a_source():
  for spec, kind in ((seq(OURS, 0, 5) + [(OURS, 5), (OURS, 5)] + seq(OURS, 6, 5), "ours_repeat"),
                     (seq(OURS, 0, 5) + [(OURS, 8)] + seq(OURS, 9, 5), "ours_jump"),
                     (seq(OURS, 0, 5) + [(OURS, 2)] + seq(OURS, 3, 5), "ours_step_back"),
                     (seq(STOCK, 0, 5) + [(STOCK, 4)] + seq(STOCK, 5, 5), "stock_repeat")):
    assert kinds(audit_stream(frames(spec))) == [kind], kind


def test_missing_stock_frames_are_only_a_note():
  res = audit_stream(frames(seq(STOCK, 0, 5) + seq(STOCK, 6, 5)))
  assert res.ok and [n.kind for n in res.notes] == ["stock_gap"]


def test_flags_unbounded_handoffs():
  assert kinds(audit_stream(frames(seq(STOCK, 0, 5) + seq(OURS, 9, 10)))) == ["takeover_jump"]     # 5 ahead
  assert kinds(audit_stream(frames(seq(STOCK, 0, 5) + seq(OURS, 2, 10)))) == ["takeover_jump"]     # behind the stock counter
  assert kinds(audit_stream(frames(seq(OURS, 0, 10) + seq(STOCK, 9, 5)))) == ["release_repeat"]    # stock repeats our last counter
  assert kinds(audit_stream(frames(seq(OURS, 0, 10) + seq(STOCK, 20, 5)))) == ["release_jump"]


def test_flags_flicker_and_gaps():
  spec = seq(OURS, 0, 10) + [(STOCK, 10), (OURS, 11)] + seq(OURS, 12, 10)       # one stock frame leaks mid-engagement
  assert "interleave" in kinds(audit_stream(frames(spec)))
  fr = frames(seq(OURS, 0, 5)) + frames(seq(OURS, 5, 5), t0=0.2)
  assert kinds(audit_stream(fr)) == ["gap"]


def test_failsafe_release_may_jump_but_never_repeat():
  spec = seq(OURS, 0, 10) + seq(STOCK, 17, 5)          # stock came back far ahead of ours
  assert kinds(audit_stream(frames(spec))) == ["release_jump"]
  assert kinds(audit_stream(frames(spec), failsafe_ok=True)) == []
  assert kinds(audit_stream(frames(seq(OURS, 0, 10) + seq(STOCK, 9, 5)), failsafe_ok=True)) == ["release_repeat"]


# ---- the drive that faulted the car (000000c0--af318d9f6b--3): vehicle-bus sequences as recorded ----
# (time s, source, alive counter) around the ACC-main press at 197.47 s
RECORDED_0X121 = [
  (197.51, OURS, 7), (197.52, OURS, 8), (197.53, OURS, 9), (197.54, OURS, 10), (197.552, OURS, 11), (197.562, OURS, 12),
  (197.57, OURS, 13), (197.58, OURS, 14), (197.60, STOCK, 5), (197.61, STOCK, 6), (197.622, STOCK, 7), (197.63, STOCK, 8),
]
RECORDED_0X1D0 = [
  (197.552, OURS, 11), (197.562, OURS, 12), (197.57, OURS, 13), (197.58, OURS, 14), (197.60, OURS, 0), (197.60, OURS, 1),
  (197.61, OURS, 2), (197.61, OURS, 3), (197.622, OURS, 4), (200.11, STOCK, 1), (200.121, STOCK, 2), (200.131, STOCK, 3),
  (200.14, STOCK, 4), (200.151, STOCK, 5), (200.163, STOCK, 6), (200.163, STOCK, 7), (200.181, STOCK, 8), (200.19, STOCK, 9),
]


def test_catches_the_recorded_accel_handoff():
  res = audit_stream([BusFrame(t, s, c) for t, s, c in RECORDED_0X121])
  assert [v.kind for v in res.violations] == ["release_jump"]       # ours 14 -> stock 5


def test_catches_the_recorded_steering_starvation():
  res = audit_stream([BusFrame(t, s, c) for t, s, c in RECORDED_0X1D0])
  assert {"gap", "release_jump"} <= {v.kind for v in res.violations}
  assert res.max_gap_s > 2.4
