from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.fisker.carcontroller import CarController
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.radar_interface import RADAR_HEADER, RadarInterface
from opendbc.car.fisker.values import CANBUS, CarControllerParams, FiskerSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    # the planner/PID must know the car's cap, or it winds up against the controller's clip
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw,
                  alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "fisker"
    # CarState reads the Ocean's OEM blind-spot alerts from the ADAS bus.
    ret.enableBsm = True
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.fisker)]

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    # Reduced from 0.20: the higher delay made MPC over-anticipate the commanded
    # angle, causing overshoot on curve entry/exit.
    ret.steerActuatorDelay = 0.15
    # steerSaturated ("Turn Exceeds Steering Limit") fires when the measured wheel angle
    # lags the command by >2.5 deg for this long. At 0.4 s it nagged during normal steering
    # (worse on left turns, where road crown demands a bit more angle so the EPS tracking lag
    # is larger) even though the car was tracking fine. 2.0 s requires a genuinely sustained
    # saturation. Advisory only -- does not change steering behavior or any safety limit.
    ret.steerLimitTimer = 2.0
    ret.steerAtStandstill = True

    # SecOC is mandatory on Ocean. The 16-byte AES key must be in params under "SecOCKey".
    ret.secOcRequired = True
    ret.safetyConfigs[0].safetyParam |= FiskerSafetyFlags.SECOC.value

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= FiskerSafetyFlags.LONG_CONTROL.value
      ret.longitudinalActuatorDelay = 0.30
      # vEgoStopping / vEgoStarting were moved to the CarParams deprecated group in
      # this openpilot release — they no longer gate the stop/start state machine, so
      # we don't set them here. Longitudinal stop/start defaults for the Ocean can be
      # tuned via stopAccel / startAccel later once we have on-vehicle logs.

    ret.minSteerSpeed = 0.0

    # The ADAS object list (ADAS_Obj0..7) has position + class but no relative velocity, so the
    # radar tracks come from the mid-range radar's private link on bus 1 (RadarInterface). Without
    # that tap (older harnesses/logs) bus 1 is silent and openpilot stays vision-only, like Tesla.
    ret.radarUnavailable = RADAR_HEADER not in fingerprint[CANBUS.radar]
    ret.radarDelay = 0.08   # measured -> published: radar MeasTime vs the cycle's last frame, median ~80 ms on bus 1

    # Lateral-only by default: openpilotLongitudinalControl stays False unless alpha_long
    # is enabled, and the fisker panda safety mode without the LONG flag has no accel
    # (0x121) in its TX allow-list, so the firmware physically blocks longitudinal — only
    # steering (0x1D0) can go out. Engagement is additionally gated by secOcRequired +
    # GW-sync key verification in the carcontroller (wrong/absent key => no steering).
    ret.dashcamOnly = False
    return ret
