from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig

Ecu = CarParams.Ecu


@dataclass
class FiskerCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class FiskerPlatformConfig(PlatformConfig):
  # Everything openpilot needs is on ADASBUS — the gateway mirrors the body/HMI
  # signals (gear, pedal, doors, seatbelt, blinkers, MFSS buttons) onto it — so
  # the port taps a single bus. fisker_ocean_body remains in opendbc/dbc for
  # reference but is not wired into a CAN parser.
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'fisker_ocean_adas',
    # Mid-range radar private link, panda bus 1. Reverse-engineered; see the DBC's comments.
    Bus.radar: 'fisker_ocean_mrr',
  })


class CAR(Platforms):
  FISKER_OCEAN = FiskerPlatformConfig(
    [FiskerCarDocs("Fisker Ocean 2023-24")],
    CarSpecs(
      mass=2410.,                # Ocean Extreme curb + typical equipment
      wheelbase=2.921,           # 2921 mm per published Ocean data sheet (was 2.99,
                                 # made the yaw model expect slower turn-in and
                                 # caused MPC to over-command angle on curve entry)
      steerRatio=15.0,           # first-cut; refine from step-response logs
      # centerToFrontRatio kept at default 0.5 — battery-in-floor EV is roughly 50/50.
      tireStiffnessFactor=0.85,  # crossover-typical; default 1.0 assumed sportier tires,
                                 # so MPC expected more grip than the car has → overshoot.
    ),
  )


class CANBUS:
  # Camera-splice harness: panda sits between the vehicle-side gateway (bus 0) and the
  # OEM ADAS module (bus 2). Bus 1 is the mid-range radar's private link, when the harness
  # taps it (see radar_interface.py). Verified from a live fingerprint capture:
  # bus 0 has 92 addresses (gateway-mirrored body/HMI + ESP/EPS/YRS/BCM/VCU/MFS/ICC),
  # bus 2 has ~65 addresses (all ADAS-authored: 0x117/0x118/0x121/0x1C0/0x1D0/0x313/
  # 0x314/0x315/0x316/0x317/0x31A/0x31C + ADAS radar object list 0x2C7..0x2E9).
  pt = 0           # ADASBUS, vehicle side (CAN-FD)
  cam = 2          # ADASBUS, OEM ADAS module side (CAN-FD)
  radar = 1        # MRR private link (CAN-FD), receive only: safety never forwards or sends on it


# VCU_GearSig enum (from DBC VAL_): 1=P, 2=N, 3=R, 4=D, 6=E, 7=S
GEAR_MAP = {
  0: CarParams.Ecu.unknown,  # placeholder; mapped to GearShifter in carstate.py
}


# MFSS button mapping
# Both MFS_RiBtnNorth and MFS_RiBtnEast engage the car's ACC on the vehicle side (VCU/ICC
# treat them identically for cc_state transitions) and emit ButtonType.mainCruise.
# StarPilot port: there is no sunnypilot MADS / per-button "ACC-only" arming. Engaging
# cruise engages openpilot (lateral + the car's ACC); MFS_RiBtnSouth emits ButtonType.lkas
# on any press (short tap or long hold), which StarPilot's Always On Lateral uses (when
# "LKAS button" AOL control is selected) to toggle AOL on/off with a single tap.
BUTTON_MAP = {
  "MFS_RiBtnNorth":  "mainCruise",   # ACC main
  "MFS_RiBtnEast":   "mainCruise",   # ACC main
  "MFS_LeRollPress": "setCruise",    # engage cruise
  "MFS_LeRollUp":    "accelCruise",  # +speed
  "MFS_LeRollDwn":   "decelCruise",  # -speed
  "MFS_RiBtnSouth":  "lkas",         # Always On Lateral toggle — any press (short or long).
                                     #   The car's short-press follow-distance function is
                                     #   inert here (no ACC coded), so a tap is free to use.
  # cancel / resume / gapAdjustCruise: TBD from bench testing
}


class CarControllerParams:
  # Steering angle command — ADAS_LatCtrl_SteerAnReq in 0x1D0
  # Signal range ±780°, 0.0625°/LSB. Cycle time 10 ms (100 Hz).
  # Fisker's carcontroller uses apply_std_steer_angle_limits (not the VM variant),
  # so ANGLE_LIMITS only carries STEER_ANGLE_MAX + up/down rate lookups.
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    600,                                  # STEER_ANGLE_MAX (deg) — DBC permits ±780, conservative 600
    ([0., 5., 25.], [2.5, 1.5, 0.2]),     # ANGLE_RATE_LIMIT_UP (deg/frame by m/s)
    ([0., 5., 25.], [5.0, 2.0, 0.3]),     # ANGLE_RATE_LIMIT_DOWN
  )

  STEER_STEP = 1                          # 0x1D0 sent every frame (10 ms ≈ 100 Hz) — verify cycle from logs
  # Capped at 1.0 (was 2.0): the stock ACC never asked for more than ~0.9 m/s2 in recorded drives, and in
  # 000000c6--51d13b87bd--1 the car faulted itself (VCU_ACCRdy dropped, ACC Failure, ESP/AEB faults) after
  # accelerating at ~1.5 m/s2 for ~0.6 s, 0.4-0.6 above our request (which had peaked at +1.29), most
  # likely tripping an actual-vs-requested acceleration plausibility check. Panda enforces the same cap
  # (FISKER_ACCEL_MAX in safety/modes/fisker.h).
  ACCEL_MAX = 1.0                         # m/s²
  ACCEL_MIN = -3.5                        # m/s²

  # 0x121 ADAS_LgtCtrl_AccelReq scale: 0.0004882 m/s²/LSB, offset -16
  ACCEL_SCALE = 0.0004882
  ACCEL_OFFSET = -16.0


class FiskerSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  SECOC = 2


class FiskerFlags(IntFlag):
  LONG_CONTROL = 1


class FiskerFlagsSP(IntFlag):
  # StarPilot port: these live in StarPilotCarParams.flags (FPCP.flags) instead of
  # sunnypilot's CarParamsSP.flags. Stored in whichever polarity makes "no flag set" the
  # baseline on-vehicle behaviour, so an install with no toggles gets the wire output the
  # port shipped with.
  ACC_AUTO_SPEED_OFF = 1    # ICC_0x52A ICCACCAutoSpdSts forced off (default: on)
  ACC_TERRAIN_ON = 2        # ICC_0x52A ICCACCTerrainSetting forced on (default: off)
  LAT_CTRL_LCA = 4          # 0x1C0 lateral type LCA/TJA (Typ=3) instead of LKA (Typ=1)


# SecOC-protected message IDs on ADASBUS (from DBC scan).
# DataId in the AUTOSAR Short SecOC profile equals the CAN message ID for these.
SECOC_TX_MSGS = {
  0x1D0: "ADAS_0x1D0",   # steering angle command
  0x121: "ADAS_0x121",   # accel command
}
SECOC_SYNC_MSG = (0x20, "GW_Syn_All")    # 50 ms, full 40-bit freshness on wire


# ICC_0x52A: the ICC's feature settings, re-sent to the ADAS module (bus 2) on every ICC frame
# (fiskercan.create_icc_spoof_0x52a) with these signals replaced. Everything else, including the
# E2E AliveCounter, passes through from the ICC's frame and the CheckSum is patched. The defaults
# below are what enable ACC (byte 4 = 0x5B with auto speed on). ICCACCAutoSpdSts and
# ICCACCTerrainSetting default to FiskerFlagsSP (FPCP.flags; baseline auto speed on, terrain off); an
# entry here wins over them. (The sunnypilot web HUD live editor is not part of the StarPilot port.)
# Raw values; names from the DBC.
ICC_0x52A_OVERRIDES: dict[str, float] = {
  "ICCACCSwt": 1,                 # 0=Invalid 1=On 2=Off -- ACC master switch; ADAS stays ACC_Off without On
  "ICCACCFuncTyp": 2,             # 0=Default 1=Basic 2=Advanced 3=ISA
  "ICCACCSpdStepSize": 1,         # 0=Step_1_unit 1=Step_5_unit
  "ICCLaneTrajectorySetting": 1,  # 0=Off 1=On
  # "ICCACCAutoSpdSts": 1,          # 0=Off 1=On -- set speed follows the speed limit
  # "ICCACCTerrainSetting": 0,      # 0=Not_enabled 1=Enabled
  # "ICCACCSpdLimOffsTyp": 1,       # 0=Exceeding_limit_not_allowed 1=Percentage 2=Speed_value
  # "ICCACCSpdLimOffs": 5,          # 0..31.5, step 0.5: % or speed over the limit, per the type
}

# Driver acknowledgements (time-gap prompt, ISA warning) and the undefined bits -- always passed
# through from the ICC's frame.
ICC_0x52A_PASSTHROUGH = frozenset({
  "ICCACCTiGapCfm",
  "ICCISAWarnStopReq",
  "ICC_0x52A_Rsv49",
  "ICC_0x52A_Rsv55",
})
assert not (ICC_0x52A_OVERRIDES.keys() & ICC_0x52A_PASSTHROUGH), "ICC_0x52A pass-through signals can't be overridden"


# ICC_0x35B: surround-view (SVS) requests + BSD/DOW/DCAA/APA/park-assist/AHBA settings. No E2E
# (no checksum/counter). Optional: while this dict is empty openpilot leaves 0x35B alone and
# panda forwards the ICC's frame as-is. Once any override is set, openpilot re-sends every ICC
# frame to the ADAS module (bus 2) with these signals replaced; everything else, including the
# undocumented bits, passes through unchanged. Raw values; names from the DBC VAL_ tables.
ICC_0x35B_OVERRIDES: dict[str, int] = {
  "ICC_EnbLnChgAsst": 1,          # 0=Auto_steer_not_enabled 1=Auto_steer_enabled
  "ICC_BSDSetting": 1,            # 0=OFF 1=ON_with_Visual 2=ON_with_visual_and_audio
  #                                 #   3=ON_with_visual_and_audio_and_Steering_Wheel_Vibration
  # "ICC_BSD_Sensitivity": 0,       # 0=Normal 1=Early 2=Late_or_reduced
  # "ICC_DOW_Setting": 2,           # 0=OFF 1=ON_with_Visual 2=ON_with_visual_and_audio
  # "ICC_DCAASetting": 1,           # 0=OFF 1=ON
  "ICC_AHBA_Setting": 1,          # 0=Off 1=On
  # "ICC_WarnTypeSetting": 2,       # 0=OFF 1=ON_with_Visual 2=ON_with_visual_and_audio
  #                                 #   3=ON_with_visual_and_audio_and_Steering_Wheel_Vibration
  # "ICC_LKA_SettingWrnTyp": 1,     # 0=Audio_Visual_and_Haptic 1=Visual_and_Audio
  "ICC_SteerWhlVibrSet": 1,       # 0=Off 1=On
  "ICC_APAActivation": 1,          # 0=Off 1=On
  "ICC_APA_Setting": 2,           # 0=Coded_off_(not_equipped) 1=Off_(by_user) 2=Enabled
  "ICC_APAParkInDirSetting": 1,   # 0=Nose-in 1=Back-in
  "ICC_APAParkOutDirSetting": 1,  # 0=Right 1=Left
  "ICC_ParkAsstChmAlrt": 1,       # 0=Off 1=On
  "ICC_WSPPA_Setting": 1,         # 0=No_curb_protection 1=Warning_only 2=Braking_intervention
  "ICC_RAP_Setting": 1,           # 0=Remote_Automated_Parking_Disabled 1=..._Enabled
  "ICC_TP_Setting": 1,            # 0=Trained_Parking_Disabled 1=Trained_Parking_Enabled
  # Also in this frame but momentary user actions, not settings — overriding one holds it
  # asserted on every frame: ICC_APAParkSelect (slot), ICC_APAActivation (1=On starts APA),
  # ICC_APAParkOutAct (1=Request starts park-out).
}

# Live user interactions with the camera view — always passed through from the ICC's frame.
ICC_0x35B_PASSTHROUGH = frozenset({
  "ICC_ViewReq",
  "ICC_ShowSts",
  "ICC_SVS_AutoVwBttnReq",
  "ICC_SVS_CamVwCnclReq",
  "ICC_AutomaticViewReq",
  "ICC_360FreeFloatingReq",
  "ICC_GraphicOverlayReq",
  "ICC_OperationType",
  "ICC_ZOOM_Start",
  "ICC_ZOOM_End",
  "ICC_ZOOM_ScalingFactor",
})
assert not (ICC_0x35B_OVERRIDES.keys() & ICC_0x35B_PASSTHROUGH), "ICC_0x35B pass-through signals can't be overridden"


# No FW-version fingerprinting for Fisker: FW_VERSIONS is intentionally empty
# (single-model port, CAN fingerprint is authoritative), so declare zero requests
# to keep the brand out of fw_versions.get_brand_ecu_matches — otherwise upstream's
# `len(brand_matches[b])` divide-by-zero crashes card at startup. The fw_version_regex
# is not part of StarPilot's FwQueryConfig, so it is omitted.
FW_QUERY_CONFIG = FwQueryConfig(requests=[])


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 0.5  # Nm — driver torque to count as "hands on" (steeringPressed, ALC/UI)

# Driver torque at which we proactively yield lateral (drop the request so the driver can
# steer). Tunable. On-vehicle 2026-10-26 the EPS's own intervention detector only trips at
# ~6.5 Nm, which is a lot of effort and lets the EPS fight up to that point before releasing
# all at once (the wheel "jump"). Driver torque (EPS_DrvrSteerTq) reads ~0.0-0.3 Nm hands-off,
# so 3.0 Nm sits well clear of the floor while being < half the EPS trip: easier override, and
# yielding early means little stored-up fight to release. Raise it if it ever yields on its
# own (e.g. a sharp curve); lower it for an even lighter override.
OVERRIDE_STEER_TORQUE = 3.0  # Nm
# Hysteresis for the torque yield: once latched, only resume after driver torque drops below
# OVERRIDE_RELEASE_TORQUE and stays there OVERRIDE_RELEASE_FRAMES (~100 Hz). Prevents the
# grab/release chatter when the driver holds steady near the trip point.
OVERRIDE_RELEASE_TORQUE = 0.5  # Nm — resume gate (above the ~0.3 Nm hands-off floor; tuned on-car)
OVERRIDE_RELEASE_FRAMES = 25   # ~0.25 s

# Pause lateral actuation below this speed (keep the EPS fed with Req=0) so the wheel
# doesn't creep/center while stopped. 0.447 m/s = 1 mph.
STANDSTILL_PAUSE_SPEED = 0.447  # m/s

# Settle wait on resume: don't re-grab the wheel until it has stopped self-centering. Hold
# while it moves faster than SETTLE_ANGLE_STEP deg per ~100 Hz tick (~0.2 = ~20 deg/s),
# bounded by SETTLE_MAX_TICKS (~1.5 s) so it can't wait forever.
SETTLE_ANGLE_STEP = 0.2   # deg/tick
SETTLE_MAX_TICKS = 150    # ~1.5 s
