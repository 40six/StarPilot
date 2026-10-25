# StarPilot for the Fisker Ocean

This branch (`fisker-ocean`) is StarPilot with the community Fisker Ocean port brought over from
sunnypilot. The CAN/DBC, SecOC, and panda safety work is by jos-technion (Majd Srour),
duncan999007, benkayuk, and moistpope. See [CREDITS.md](CREDITS.md) for exact source commits.

**Status: not yet tested on a car.** The unit and safety tests pass, and the panda firmware
builds. The first drives are bring-up drives. Keep your hands on the wheel.

## What's in it

| Area | Where | Notes |
| --- | --- | --- |
| Car port | `opendbc_repo/opendbc/car/fisker/` | Platform `FISKER_OCEAN`. Angle steering, SecOC, ICC_0x52A/0x35B relays, MRR radar tracks on bus 1, BSM |
| DBCs | `opendbc_repo/opendbc/dbc/fisker_ocean_{adas,mrr}.dbc` | |
| Panda safety | `opendbc_repo/opendbc/safety/modes/fisker.h` | `SAFETY_FISKER` (39). Candidate-substitution SecOC handover via a new `fwd_modify` hook |
| Panda firmware | `panda/board/obj/*.bin.signed` | **Rebuilt and committed** with the Fisker mode, `fwd_modify`, and CAN ignition |
| SecOC key | `selfdrive/car/card.py` | Uses the stored `SecOCKey`. If there isn't one, it reads the key once from the ADAS radar over UDS (DID 0xEFF5) and saves it |
| Launch | `launch_env.sh` | `SKIP_FW_QUERY=1`. The Ocean is matched by CAN fingerprint only |

StarPilot ships as a prebuilt tree (the `prebuilt` marker). Only Python files and panda firmware
changed here, so nothing needs to compile on the device. `car.capnp` gained `fisker @39` at the
end of `SafetyModel`, plus two reserved placeholders at 37 and 38 so the number matches the panda.
Old C++ binaries just pass 39 through.

## How it drives (compared to the sunnypilot port)

StarPilot has no MADS, so the engagement model is StarPilot's:

- **Engage:** press either ACC main button (RiBtnNorth or RiBtnEast) to bring ADAS ACC up.
  When cruise engages, openpilot engages with lateral, and the car's own ACC handles
  longitudinal. The sunnypilot port's North = "ACC only" / East = "ACC + lateral" split
  (`FiskerMadsArmed`) is **not ported**. Both buttons behave the same.
- **Always On Lateral:** turn on *Always On Lateral* in StarPilot. Then either:
  - Main-cruise mode: lateral works whenever ADAS ACC is in Standby or above, or
  - LKAS-button mode: set *Vehicle → LKAS Button* to *Always On Lateral*. A **long press of
    RiBtnSouth** toggles it (a short press stays the car's follow-distance button). Panda only
    lets the toggle work while ACC is at least in Standby.
- While openpilot isn't steering, the OEM ADAS lane keeping passes through untouched.
- A sustained EPS abort sets `steerFaultPermanent`, which disengages immediately, just like the
  sunnypilot port.

## Ignition

The firmware reads CAN ignition from `BCM_0x333.BCM_PwrMod == Run` in **every** firmware
variant. It only trusts the frame when the Fisker AliveCounter steps correctly, because Ford also
uses 0x333. If your harness holds the comma's IGN line high all the time (moistpope's board does),
turn on **Vehicle → CAN Ignition Only**. That loads the `*_can_ignition_only` panda firmware so
the device goes offroad when the car turns off.

## Install

1. Push this branch to your own GitHub fork of StarPilot (see the delivery notes).
2. On the comma, either uninstall and enter the custom software URL
   `installer.comma.ai/<your-github-user>/fisker-ocean`. That installer pulls from
   `github.com/<your-github-user>/openpilot`, so the fork has to be named `openpilot`. Or switch
   branches over SSH:
   `cd /data/openpilot && git remote add fisker https://github.com/<you>/StarPilot && git fetch fisker fisker-ocean && git checkout -B fisker-ocean fisker/fisker-ocean && sudo reboot`.
3. On first boot pandad reflashes the panda (the firmware signature changed). That's expected.
4. SecOC key: if you already have one from sunnypilot, it's in `/data/params/d/SecOCKey` and
   carries over. Otherwise the first onroad start tries the UDS read. Check the logs for
   `Fisker SecOC key verified against GW sync MAC` before you expect steering.

## Not ported (sunnypilot-only)

- Web HUD / Oceanfront View bridge, the live CAN-override editor (`FiskerCanOverrides`)
- Phone remote-control steering
- The `FiskerACCAutoSpeed`, `FiskerACCTerrain`, and `FiskerLateralType` UI toggles. Their
  baseline defaults are used (auto speed on, terrain off, LKA lateral type). They can be set
  through `FiskerFlagsSP` in `StarPilotCarParams.flags` later. New params would need a rebuilt
  `params_pyx.so`, which a prebuilt tree can't do.
- `debug_fisker_*` scripts

## Things to check on the first drive

1. Device goes onroad with the car on, and offroad with it off.
2. `carParams.carFingerprint == FISKER_OCEAN` and `safetyModel == fisker`, with no
   `controlsMismatch` or `canError`.
3. The SecOC key is verified (see above).
4. Engage on cruise at low speed on a straight road. Watch for "ADAS error" on the cluster.
5. Try AOL (main-cruise mode first), then the LKAS long press.
6. Disengage handover: brake, then cancel. The OEM LKA should come back with no cluster fault.

Upstream setting to be aware of: `ICC_0x35B_OVERRIDES` in `fisker/values.py` holds
`ICC_APAActivation = 1`, which upstream's own comment describes as a momentary APA start
action. It's kept as-is from moistpope/opendbc master. Comment it out if parking assist does
anything unexpected.

## Developer notes

- Panda firmware was built with xPack `arm-none-eabi-gcc 13.3.1` (`scons` in `panda/`, debug
  cert, like the original StarPilot tree). Rebuild it after any change under
  `opendbc_repo/opendbc/safety` or `panda/board`, and commit the `board/obj` outputs.
- Tests: `opendbc/car/fisker/tests`, `opendbc/safety/tests/test_fisker*.py`, and
  `test_car_interfaces[FISKER_OCEAN]`.
