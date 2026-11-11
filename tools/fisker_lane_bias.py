#!/usr/bin/env python3
# Fisker lane-position bias logger. Read-only (published topics + a few params at start).
# Records, at 10 Hz, every stage where a lateral offset could enter, so a drive can show WHERE
# the car's right-of-center bias comes from:
#   1. what the model SEES:      lane lines / road edges relative to the car (offset from center)
#   2. what the model ASKS for:  planned path lateral position + the model's desired curvature
#   3. what StarPilot changes:   controlsd desiredCurvature (after lane centering etc.)
#   4. what the car DOES:        commanded vs output vs measured wheel angle, actual curvature
#   + the learners/settings that shift 1-4: liveCalibration rpy, liveParameters angle offset /
#     steer ratio / roll, liveDelay, CameraOffset / LaneCenterOffset / SteerRatio params.
# openpilot frame: x forward, y to the RIGHT (laneLines[1] = left line, y < 0; [2] = right, y > 0).
# lane_ctr_y = (left_y + right_y) / 2 is where the lane center is relative to the car:
#   negative -> lane center is to the LEFT -> the car sits RIGHT of center.
#
# Run on the comma:  cd /data/openpilot && python3 /data/fisker_lane_bias.py [minutes]
# Output: /data/fisker_lane_bias.csv (+ a JSON header line with params/calibration at start).

import csv
import json
import math
import sys
import time

import cereal.messaging as m
from openpilot.common.params import Params

MPH = 2.237
HZ = 10.
minutes = float(sys.argv[1]) if len(sys.argv) > 1 else 60.
OUT = "/data/fisker_lane_bias.csv"


def y_at(xyzt, x_target):
  """Lateral y of an XYZTData polyline at distance x_target (m), linearly interpolated."""
  xs, ys = list(xyzt.x), list(xyzt.y)
  if not xs:
    return float("nan")
  if x_target <= xs[0]:
    return ys[0]
  for i in range(1, len(xs)):
    if xs[i] >= x_target:
      f = (x_target - xs[i - 1]) / max(xs[i] - xs[i - 1], 1e-6)
      return ys[i - 1] + f * (ys[i] - ys[i - 1])
  return ys[-1]


def param(params, key):
  try:
    v = params.get(key)
    return v.decode() if isinstance(v, bytes) else v
  except Exception:
    return None


sm = m.SubMaster(["modelV2", "controlsState", "carState", "carControl", "carOutput", "liveParameters",
                  "liveCalibration", "liveDelay", "selfdriveState"])
params = Params()
header = {k: param(params, k) for k in ("CameraOffset", "LaneCenterOffset", "LaneCentering", "SteerRatio",
                                        "SteerRatioStock", "AlwaysOnLateral", "AlphaLongitudinalEnabled")}

COLS = ["t", "v_mph", "lat_active", "enabled", "blinker", "steer_pressed", "lc_state",
        "ll_left_y0", "ll_right_y0", "ll_left_y10", "ll_right_y10", "ll_left_prob", "ll_right_prob",
        "edge_left_y0", "edge_right_y0", "lane_ctr_y0", "lane_ctr_y10", "lane_width0",
        "path_y10", "path_y20", "model_curv", "ctrl_desired_curv", "ctrl_curv",
        "angle_desired", "angle_cmd", "angle_out", "angle_meas",
        "angle_offset", "angle_offset_avg", "steer_ratio", "stiffness", "lp_roll_deg", "lp_valid",
        "cal_roll_deg", "cal_pitch_deg", "cal_yaw_deg", "cal_perc", "lat_delay"]
out = open(OUT, "w", newline="")
out.write("# " + json.dumps(header) + "\n")
w = csv.writer(out)
w.writerow(COLS)

t0 = time.monotonic()
end = t0 + minutes * 60.
next_row = next_sum = t0
straight = []   # (lane_ctr_y0, path_y10, angle_cmd - angle_meas, model_curv) on straight engaged driving
print(f"# lane-bias logger: {minutes:.0f} min -> {OUT}  params {header}", flush=True)
try:
  while time.monotonic() < end:
    sm.update(20)
    now = time.monotonic()
    if now < next_row:
      continue
    next_row += 1. / HZ
    md, cs, cst, cc, co = sm["modelV2"], sm["controlsState"], sm["carState"], sm["carControl"], sm["carOutput"]
    lp, lc, ld = sm["liveParameters"], sm["liveCalibration"], sm["liveDelay"]
    ll = list(md.laneLines)
    re = list(md.roadEdges)
    probs = list(md.laneLineProbs)
    l0 = y_at(ll[1], 0.) if len(ll) > 2 else float("nan")
    r0 = y_at(ll[2], 0.) if len(ll) > 2 else float("nan")
    l10 = y_at(ll[1], 10.) if len(ll) > 2 else float("nan")
    r10 = y_at(ll[2], 10.) if len(ll) > 2 else float("nan")
    try:
      ang = cs.lateralControlState.angleState
      angle_desired = ang.steeringAngleDesiredDeg
    except Exception:
      angle_desired = float("nan")
    rpy = [math.degrees(x) for x in lc.rpyCalib] if len(lc.rpyCalib) == 3 else [float("nan")] * 3
    row = {
      "t": round(now - t0, 2), "v_mph": round(cst.vEgo * MPH, 2), "lat_active": int(cc.latActive),
      "enabled": int(sm["selfdriveState"].enabled), "blinker": int(cst.leftBlinker) - int(cst.rightBlinker),
      "steer_pressed": int(cst.steeringPressed), "lc_state": str(md.meta.laneChangeState),
      "ll_left_y0": round(l0, 3), "ll_right_y0": round(r0, 3), "ll_left_y10": round(l10, 3), "ll_right_y10": round(r10, 3),
      "ll_left_prob": round(probs[1], 2) if len(probs) > 2 else "", "ll_right_prob": round(probs[2], 2) if len(probs) > 2 else "",
      "edge_left_y0": round(y_at(re[0], 0.), 3) if len(re) > 1 else "", "edge_right_y0": round(y_at(re[1], 0.), 3) if len(re) > 1 else "",
      "lane_ctr_y0": round((l0 + r0) / 2, 3), "lane_ctr_y10": round((l10 + r10) / 2, 3), "lane_width0": round(r0 - l0, 3),
      "path_y10": round(y_at(md.position, 10.), 3), "path_y20": round(y_at(md.position, 20.), 3),
      "model_curv": round(md.action.desiredCurvature, 6), "ctrl_desired_curv": round(cs.desiredCurvature, 6),
      "ctrl_curv": round(cs.curvature, 6),
      "angle_desired": round(angle_desired, 3), "angle_cmd": round(cc.actuators.steeringAngleDeg, 3),
      "angle_out": round(co.actuatorsOutput.steeringAngleDeg, 3), "angle_meas": round(cst.steeringAngleDeg, 3),
      "angle_offset": round(lp.angleOffsetDeg, 3), "angle_offset_avg": round(lp.angleOffsetAverageDeg, 3),
      "steer_ratio": round(lp.steerRatio, 3), "stiffness": round(lp.stiffnessFactor, 3),
      "lp_roll_deg": round(math.degrees(lp.roll), 3), "lp_valid": int(lp.valid),
      "cal_roll_deg": round(rpy[0], 3), "cal_pitch_deg": round(rpy[1], 3), "cal_yaw_deg": round(rpy[2], 3),
      "cal_perc": int(lc.calPerc), "lat_delay": round(ld.lateralDelay, 3),
    }
    w.writerow([row[c] for c in COLS])

    # Straight, engaged, no blinker / lane change / driver input, both lines confident.
    if (cc.latActive and cst.vEgo > 11 and abs(md.action.desiredCurvature) < 0.0005 and row["blinker"] == 0
        and row["lc_state"] == "off" and not cst.steeringPressed and len(probs) > 2 and min(probs[1], probs[2]) > 0.7):
      straight.append((row["lane_ctr_y0"], row["path_y10"], row["angle_cmd"] - row["angle_meas"], row["model_curv"]))
    if now >= next_sum:
      next_sum = now + 30.
      out.flush()
      if straight:
        n = len(straight)
        mean = [sum(s[i] for s in straight) / n for i in range(4)]
        print(f"{now - t0:7.0f}s straight {n / HZ:5.0f}s: lane center {mean[0]:+.3f} m (neg = car RIGHT of center) | "
              f"path@10m {mean[1]:+.3f} m | cmd-meas {mean[2]:+.2f} deg | model curv {mean[3]:+.6f} | "
              f"cal yaw {rpy[2]:+.2f} deg | angleOffset {lp.angleOffsetDeg:+.2f} deg", flush=True)
except KeyboardInterrupt:
  pass
finally:
  out.close()
  print(f"# done, {OUT}", flush=True)
