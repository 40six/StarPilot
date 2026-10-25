#pragma once

#include "opendbc/safety/declarations.h"

// Fisker Ocean — angle steering control, AUTOSAR "Short SecOC" on the
// actuator commands. Panda enforces signal-level bounds and rate limits; it does
// NOT validate the SecOC MAC (the car gateway does that, and the key lives in
// userspace). Panda validates counters + frequency + actuator limits.
//
// Buses: ADASBUS = bus 0.
//   TX  0x1D0  ADAS_LatCtrl_SteerAnReq    steering angle (SecOC)
//   TX  0x1C0  ADAS_LatCtrl activation    plain E2E, lateral activation/status
//   TX  0x121  ADAS_LgtCtrl_AccelReq      accel (SecOC, longitudinal only)
//   TX  0x117  ADAS long control status   plain E2E, Sts/Typ (CC -> ACC transition)
//   TX  0x118  ADAS long/ESP handshake    plain E2E, ESP-side mirror + jerk/prefill/AEB
//   TX  0x52A  ICC settings spoof         bus 2 (to ADAS); plain E2E — see below
//   RX  0x115  wheel speeds -> vehicle_moving
//   RX  0x318  vehicle speed + brake pedal
//   RX  0x1C2  EPS steering angle
//   RX  0x1C4  EPS driver torque
//   RX  0x313  ADAS ACC state (cruise engaged) — cam-side (bus 2)

static bool fisker_longitudinal = false;

// ICC_0x52A (ICC feature settings, 200 ms): openpilot reads the ICC's frame on bus 0 and
// re-sends it to the ADAS module on bus 2 with byte 4 overridden to enable ACC, keeping
// the ICC's own AliveCounter. The ICC's original is blocked bus 0 -> bus 2 only while
// openpilot's copies are flowing; if openpilot stops (crash, disabled), forwarding
// resumes so ADAS never actually loses 0x52A — just the "ACC enabled" overrides.
// 500 ms = 2.5 missed 200 ms cycles.
#define FISKER_ICC_RELAY_TIMEOUT_US 500000U
static uint32_t fisker_icc_settings_tx_last = 0U;

static bool fisker_icc_relay_active(void) {
  return safety_get_ts_elapsed(microsecond_timer_get(), fisker_icc_settings_tx_last) < FISKER_ICC_RELAY_TIMEOUT_US;
}

// ICC_0x35B (SVS view requests + BSD/DOW/APA/park-assist settings, ~5 Hz): openpilot re-sends
// the ICC's frame on bus 2 with ICC_0x35B_OVERRIDES applied. Same time-based relay as 0x52A,
// but seeded idle: this relay is optional (only runs when overrides are configured), so the
// OEM's own 0x35B flows through until openpilot actually starts emitting its copies.
static uint32_t fisker_icc_0x35b_tx_last = 0U;
static bool fisker_icc_0x35b_tx_seen = false;

static bool fisker_icc_0x35b_relay_active(void) {
  // Only block the OEM's 0x35B once openpilot has actually started sending its own copies
  // (seen flag), and only within the timeout since the last one. Without the seen gate a
  // zero-seeded timestamp would read as "recent" right after init and wrongly block.
  return fisker_icc_0x35b_tx_seen && (safety_get_ts_elapsed(microsecond_timer_get(), fisker_icc_0x35b_tx_last) < FISKER_ICC_RELAY_TIMEOUT_US);
}

// (EPS_0x1C2 spoof panda plumbing was removed again after on-vehicle testing showed
// the spoofed byte-6 override doesn't quiet the OEM ADAS "LKA not available" alerts
// in MADS/Both — probably because ADAS cross-references other EPS signals we don't
// touch (0x1C4 torque, 0x475 mode). The Python-side helper + E2E data_id are kept
// as-is in fiskercan.py for easy revival once we have reason to try again with a
// multi-signal spoof. See the commented block at the spoof call site in carcontroller.py.)

// Lateral TX relay: while openpilot is injecting 0x1D0/0x1C0, panda blocks OEM's own
// 0x1D0/0x1C0 from reaching the EPS (bus 2 → bus 0) so the EPS is fed from exactly one
// source. The timestamp is refreshed on every injected frame, so the relay stays active
// as long as we TX at 100 Hz (plus the short post-disengage fade). TIMEOUT sets how long
// after our LAST frame the OEM stream resumes to the EPS — i.e. the disengage handoff
// gap. It was 500 ms, which left the EPS unfed for up to ~500 ms after the fade ended
// (OEM blocked the whole time) and could fault it on the gap. 60 ms = 6 control ticks:
// long enough to ride out normal TX jitter during engagement (we never miss 6 ticks in
// a row on the CAN-locked 100 Hz loop), short enough that OEM resumes almost immediately
// after we stop — our counter is OEM's exact continuation, so OEM picks up at our_last+N
// for a tiny N, never a backward/duplicate value (on-vehicle 2026-10-19 user request).
#define FISKER_LAT_RELAY_TIMEOUT_US 60000U
static uint32_t fisker_lat_tx_last = 0U;

static bool fisker_lat_relay_active(void) {
  return safety_get_ts_elapsed(microsecond_timer_get(), fisker_lat_tx_last) < FISKER_LAT_RELAY_TIMEOUT_US;
}

// ---- 0x1D0 candidate substitution (race-free SecOC handover) --------------------
// The SecOC MAC lives in userspace, so panda can't re-stamp 0x1D0. But the SecOC
// counter/MAC collision at the OEM→openpilot handover is a hard race in Python
// (our first frame vs OEM's identical next frame). We fix it entirely in panda WITHOUT
// crypto: openpilot ships a small WINDOW of fully-stamped 0x1D0 payloads each tick as
// 0x5D0 on the unused bus 1 (the "candidate pool"). Panda buffers them keyed by the
// on-wire counter+reset-flag (byte 4). Then, instead of forwarding OEM's own bus-2
// 0x1D0 to the EPS, panda SUBSTITUTES our candidate that matches OEM's current byte 4
// (fisker_fwd_modify). Result: the EPS receives exactly one valid frame per OEM frame,
// on OEM's own counter, carrying our angle — so there is no duplicate/backward value at
// engage OR disengage, regardless of timing (proven in sim 2026-10-19). If no matching
// fresh candidate exists we emit nothing that frame (benign 1-frame gap), never a wrong
// counter. "Armed" = we received a candidate recently; stop sending → OEM resumes.
#define FISKER_LAT_CANDIDATE_ADDR 0x5D0U
#define FISKER_CAND_FRESH_US   60000U   // a buffered candidate is usable this long
#define FISKER_CAND_ARM_US     60000U   // substitution armed while candidates keep arriving

typedef struct {
  uint8_t data[8];
  uint8_t byte4;     // on-wire counter+reset-flag this candidate was stamped for
  uint32_t rx_ts;    // when captured
  bool angle_ok;     // angle passed steer_angle_cmd_checks at capture
} FiskerCandidate;

static FiskerCandidate fisker_cand[64];   // keyed by wire counter = (byte4 >> 2) & 0x3F
static uint32_t fisker_cand_last_rx = 0U;

static bool fisker_cand_armed(void) {
  return safety_get_ts_elapsed(microsecond_timer_get(), fisker_cand_last_rx) < FISKER_CAND_ARM_US;
}

// Steering: raw CAN at 0.0625 deg/LSB, offset -780 deg. Raw 12480 == 0 deg.
#define FISKER_ANGLE_ZERO_CAN 12480

// Accel: raw CAN at 0.0004882 m/s^2/LSB, offset -16 m/s^2. raw = (accel + 16) / 0.0004882
#define FISKER_ACCEL_INACTIVE 32773  //  0.0 m/s^2
#define FISKER_ACCEL_MAX      36870  // +2.0 m/s^2
#define FISKER_ACCEL_MIN      25604  // -3.5 m/s^2

// Most Ocean messages carry a 4-bit AliveCounter in byte 1 bits 8..11.
static uint8_t fisker_get_counter(const CANPacket_t *msg) {
  return (msg->data[1] >> 4) & 0x0FU;
}


// Cruise sources. Cars with ACC coded run the ADAS module's full ACC (ADAS_0x313, bus 2);
// cars without it (ACC not coded) only have the VCU's basic hold-speed cruise
// (VCU_0x358, bus 0) — the source jos-technion's original port used. Both share the
// same enum (0=Off 1=Init 2=Standby 3=Active 4=Override 9/10=Failure; ADAS adds the
// standstill states 5/6/11), so either one engages / makes ACC main available.
static int fisker_adas_cc_state = 0;
static int fisker_vcu_cc_state = 0;

static bool fisker_cc_engaged(int cc_state, bool adas) {
  bool engaged = (cc_state == 3) || (cc_state == 4);
  if (adas) {
    engaged = engaged || (cc_state == 5) || (cc_state == 6) || (cc_state == 11);
  }
  return engaged;
}

static bool fisker_cc_available(int cc_state) {
  return (cc_state != 0) && (cc_state != 1) && (cc_state != 9) && (cc_state != 10);
}

static void fisker_update_cruise(void) {
  pcm_cruise_check(fisker_cc_engaged(fisker_adas_cc_state, true) || fisker_cc_engaged(fisker_vcu_cc_state, false));

  // StarPilot AOL: ACC main is "on" whenever either cruise reports Standby or above.
  // Mirrors carstate's cruiseState.available. Dropping out also clears the LKAS-button
  // AOL latch so the next drive starts with lateral off.
  acc_main_on = fisker_cc_available(fisker_adas_cc_state) || fisker_cc_available(fisker_vcu_cc_state);
  if (!acc_main_on) {
    lkas_on = false;
  }
}

static void fisker_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // EPS_0x1C2: measured steering wheel angle (BE 16-bit @ start 23 -> bytes 2,3).
    if (msg->addr == 0x1C2U) {
      int raw = (msg->data[2] << 8) | msg->data[3];
      update_sample(&angle_meas, raw - FISKER_ANGLE_ZERO_CAN);
    }

    // EPS_0x1C4: driver steering torque magnitude (BE 12-bit @ start 23 -> bytes 2, hi-nibble 3).
    if (msg->addr == 0x1C4U) {
      int torque = (msg->data[2] << 4) | (msg->data[3] >> 4);
      update_sample(&torque_driver, torque);
    }

    // ESP_0x318: vehicle speed (BE 16-bit @ start 47 -> bytes 5,6, 0.1 km/h) + brake pedal.
    if (msg->addr == 0x318U) {
      float speed = ((msg->data[5] << 8) | msg->data[6]) * 0.1f * KPH_TO_MS;
      UPDATE_VEHICLE_SPEED(speed);
      brake_pressed = ((msg->data[1] >> 6) & 0x1U) != 0U;  // ESP_BrkPedlSts @ bit 14
    }

    // ESP_0x115: front-right wheel speed (BE 14-bit @ start 21) -> vehicle_moving.
    if (msg->addr == 0x115U) {
      int whl_rf = ((msg->data[1] & 0x3FU) << 8) | msg->data[2];
      vehicle_moving = whl_rf > 0;
    }

    // VCU_0x358: VCU basic cruise state (VCU_Sts_CC_ICC, 4-bit @ start 35 -> data[4]
    // low nibble). The only cruise on cars without ACC coded.
    if (msg->addr == 0x358U) {
      fisker_vcu_cc_state = msg->data[4] & 0x0FU;
      fisker_update_cruise();
    }

    // MFS_0x514: MFSS steering-wheel buttons. MFS_RiBtnSouth (2-bit @ start 33, big-endian
    // -> byte 4 bits 0,1) is sunnypilot's MADS engage/disengage toggle. Values are
    // 0=No_Pressed, 1=Pressed(short), 2=Long_Press, 3=Reserved. We ONLY treat the long
    // press as the MADS trigger — short press is consumed by the car for the ACC
    // follow-distance adjustment, so firing MADS on it would clash. Openpilot's carstate
    // uses the same long-press-only predicate for the ButtonType.lkas edge, keeping the
    // two sides in lockstep.
    //
    // StarPilot: there is no MADS in StarPilot. Always On Lateral (AOL) is opened by
    // aol_allowed = (acc_main_on || lkas_on) && ALT_EXP_ALWAYS_ON_LATERAL (safety.h). The
    // long press toggles lkas_on on its rising edge, only while ADAS cruise is at least in
    // Standby (acc_main_on) — the port's "nothing steers unless cruise is >= Standby" rule.
    if (msg->addr == 0x514U) {
      int rbs = msg->data[4] & 0x03U;
      bool lkas_button = (rbs == 2);
      if (lkas_button && !lkas_button_prev && acc_main_on) {
        lkas_on = !lkas_on;
      }
      lkas_button_prev = lkas_button;
    }
  }

  // ADAS_0x313 lives on bus 2 (cam side). We read cruise-engaged state from the
  // ADAS module itself instead of the VCU basic-CC path — now that the ICC_0x52A
  // spoof forces ICCACCFuncTyp=2, ADAS enters full ACC mode and is the authoritative
  // source. ADAS_Sts_ACC_ICC is a 4-bit signal @ start bit 35 (big-endian) → the
  // low nibble of data[4] (byte 4 covers bits 32..39 in Motorola numbering; a 4-bit
  // field with MSB at 35 spans bits 35..32).
  //
  // ADAS_Sts_ACC_ICC enum: 0=ACC_Off, 1=Init, 2=Standby, 3=Active, 4=Override,
  //   5=Standstill_active, 6=Standstill_wait, 7=Deactivation_brake,
  //   8=Deactivation_other, 9=Failure_reversible, 10=Failure_irreversible,
  //   11=Standstill_GoNotification.
  // Treat every state where the ACC controller is commanding the car as engaged:
  // Active(3), Override(4), and the three Standstill_* holds (5, 6, 11).
  //
  // sunnypilot deliberately left acc_main_on unset (its MADS state machine falling-edged
  // on it). StarPilot has no MADS: acc_main_on here only feeds aol_allowed, so it is set
  // from the ADAS ACC state below. Brake (3 -> 2, Standby) keeps acc_main_on true, so AOL
  // keeps steering through a brake press exactly like sunnypilot's "Remain Active".
  if (msg->bus == 2U) {
    if (msg->addr == 0x313U) {
      fisker_adas_cc_state = msg->data[4] & 0x0FU;
      fisker_update_cruise();
    }
  }
}


// Steering angle limits (mirror CarControllerParams.ANGLE_LIMITS in values.py). The
// upstream AngleSteeringLimits struct in this openpilot release is minimal — the
// tizi-era fields (max_angle_error, angle_error_min_speed, angle_is_curvature,
// enforce_angle_error, inactive_angle_is_zero) were dropped from the C struct when
// angle-error enforcement moved into the VM-based limits path. Fisker's carcontroller
// still uses the non-VM apply_std_steer_angle_limits, so the 5 fields below are all
// that apply.
static const AngleSteeringLimits FISKER_STEERING_LIMITS = {
  .max_angle = 9600,             // 600 deg * 16 CAN/deg
  .angle_deg_to_can = 16.0f,     // 1 / 0.0625
  .angle_rate_up_lookup = {
    {0., 5., 25.},
    {2.5, 1.5, 0.2}
  },
  .angle_rate_down_lookup = {
    {0., 5., 25.},
    {5.0, 2.0, 0.3}
  },
};

static const LongitudinalLimits FISKER_LONG_LIMITS = {
  .max_accel = FISKER_ACCEL_MAX,
  .min_accel = FISKER_ACCEL_MIN,
  .inactive_accel = FISKER_ACCEL_INACTIVE,
};


static bool fisker_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // Steering angle command (0x1D0). 0x1D0 carries no explicit enable bit, so the
  // steer-active state is derived from (controls_allowed || aol_allowed) —
  // openpilot only transmits this frame when it intends to steer, and StarPilot's
  // Always On Lateral (aol_allowed) opens the lateral path even without cruise engaged.
  if (msg->addr == 0x1D0U) {
    int raw_angle = (msg->data[2] << 8) | msg->data[3];
    int desired_angle = raw_angle - FISKER_ANGLE_ZERO_CAN;

    if (steer_angle_cmd_checks(desired_angle, controls_allowed || aol_allowed, FISKER_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Lateral-control activation (0x1C0). Angle actuation is gated by 0x1D0's checks; also
  // only permit declaring the lateral request ACTIVE while lateral is authorized (either
  // by cruise or by Always On Lateral).
  // ADAS_LatCtrl_Req @ 31|2 -> data[3] bits 7..6 (0=Not_active, 1=Angle_request_active).
  if (msg->addr == 0x1C0U) {
    int lat_req = (msg->data[3] >> 6) & 0x03U;
    if ((lat_req != 0) && !(controls_allowed || aol_allowed)) {
      tx = false;
    }
  }

  // Accel command (0x121)
  if (msg->addr == 0x121U) {
    int raw_accel = (msg->data[2] << 8) | msg->data[3];

    if (fisker_longitudinal) {
      if (longitudinal_accel_checks(raw_accel, FISKER_LONG_LIMITS)) {
        tx = false;
      }
    } else {
      // Not controlling longitudinal: only the inactive value may be sent.
      if (raw_accel != FISKER_ACCEL_INACTIVE) {
        tx = false;
      }
    }
  }

  // 0x1D0 candidate pool (0x5D0 on bus 1). Each candidate is a fully-stamped 0x1D0
  // payload for the counter in its byte 4. Validate the angle with the SAME limits as a
  // direct 0x1D0 (panda's actuator-rate safety must still hold — the angle reaches the
  // EPS via fisker_fwd_modify, not a normal TX), then buffer it keyed by wire counter.
  // Bus 1 is NOT unused here: it is the mid-range radar's private link, which we only
  // receive. So once buffered, the candidate is REJECTED (tx = false) and never reaches
  // the wire — fisker_fwd_modify is the only consumer. The cost is one rejected-frame
  // echo per candidate in the host's can stream. A candidate whose angle fails the check
  // is buffered as angle_ok=false and will be skipped (gap), never emitted.
  if ((msg->addr == FISKER_LAT_CANDIDATE_ADDR) && (msg->bus == 1U)) {
    int raw_angle = (msg->data[2] << 8) | msg->data[3];
    int desired_angle = raw_angle - FISKER_ANGLE_ZERO_CAN;
    bool angle_ok = !steer_angle_cmd_checks(desired_angle, controls_allowed || aol_allowed, FISKER_STEERING_LIMITS);
    uint8_t wire = (msg->data[4] >> 2) & 0x3FU;
    FiskerCandidate *c = &fisker_cand[wire];
    for (int i = 0; i < 8; i++) {
      c->data[i] = msg->data[i];
    }
    c->byte4 = msg->data[4];
    c->rx_ts = microsecond_timer_get();
    c->angle_ok = angle_ok;
    fisker_cand_last_rx = c->rx_ts;
    // Candidates keep the lateral relay fresh too (they mean "we're steering"), so OEM's
    // 0x1C0 stays blocked bus 2 -> bus 0 while we inject our own 0x1C0.
    fisker_lat_tx_last = c->rx_ts;
    tx = false;  // panda-internal: never transmit onto the radar link
  }

  // ICC settings spoof (0x52A, bus 2): record the TX timestamp so the fwd_hook knows
  // whether the ICC's own 0x52A should still be blocked bus 0 -> bus 2.
  if ((msg->addr == 0x52AU) && (msg->bus == 2U)) {
    fisker_icc_settings_tx_last = microsecond_timer_get();
  }
  // ICC_0x35B settings relay (bus 2): record the TX timestamp so the fwd_hook knows
  // whether the ICC's own 0x35B should still be blocked bus 0 -> bus 2.
  if ((msg->addr == 0x35BU) && (msg->bus == 2U)) {
    fisker_icc_0x35b_tx_last = microsecond_timer_get();
    fisker_icc_0x35b_tx_seen = true;
  }
  // Lateral activation (0x1C0, bus 0): record the TX timestamp so the fwd_hook keeps
  // blocking OEM 0x1C0 bus 2 → bus 0 across the engaged window and the fade. (0x1D0 is
  // now handled by the candidate pool above, not a direct bus-0 TX.)
  if ((msg->addr == 0x1C0U) && (msg->bus == 0U)) {
    fisker_lat_tx_last = microsecond_timer_get();
  }

  return tx;
}

// Substitute OEM's bus-2 0x1D0 with our matching candidate just before it is forwarded to
// the EPS on bus 0. See the candidate-pool comment above. Returns false to DROP the
// forward (benign one-frame gap) when armed but no fresh matching candidate exists.
static bool fisker_fwd_modify(CANPacket_t *to_send) {
  bool emit = true;
  if ((to_send->addr == 0x1D0U) && (to_send->bus == 2U) && fisker_cand_armed()) {
    uint8_t wire = (to_send->data[4] >> 2) & 0x3FU;
    const FiskerCandidate *c = &fisker_cand[wire];
    bool fresh = safety_get_ts_elapsed(microsecond_timer_get(), c->rx_ts) < FISKER_CAND_FRESH_US;
    // Match the FULL byte 4 (wire counter AND reset flag) so a candidate from the wrong
    // GW-reset window is never emitted — worst case is a one-frame gap, not a bad MAC.
    if (fresh && c->angle_ok && (c->byte4 == to_send->data[4])) {
      for (int i = 0; i < 8; i++) {
        to_send->data[i] = c->data[i];
      }
    } else {
      emit = false;  // armed but nothing valid to send this frame -> gap
    }
  }
  return emit;
}


static safety_config fisker_init(uint16_t param) {
  // Forwarding intercept: the OEM ADAS module stays alive on bus 2 and its status/HUD
  // frames are forwarded to the car, so openpilot only injects the command it replaces.
  // disable_static_blocking lets fisker_fwd_hook forward the OEM's 0x1D0 when disengaged
  // and block it only while openpilot steers; check_relay still guards against the OEM's
  // steering leaking onto bus 0.
  // 0x52A goes to the ADAS module on bus 2. No check_relay: a relay malfunction stops
  // ALL forwarding, and bus 2 presence isn't reliable evidence of one — fisker_fwd_hook
  // blocks the OEM original via a time-based relay instead.
  static const CanMsg FISKER_TX_MSGS[] = {
    {0x1D0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // steering angle (fwd path; see fisker_fwd_modify)
    {0x5D0, 1, 8, .check_relay = false},                                    // 0x1D0 candidate pool (buffered, then rejected: never sent on bus 1)
    {0x1C0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // lateral activation
    {0x52A, 2, 8, .check_relay = false},                                    // ICC settings spoof (bus 2)
    {0x35B, 2, 8, .check_relay = false},                                    // ICC SVS/BSD/APA settings (bus 2)
  };
  static const CanMsg FISKER_LONG_TX_MSGS[] = {
    {0x1D0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // steering angle (fwd path; see fisker_fwd_modify)
    {0x5D0, 1, 8, .check_relay = false},                                    // 0x1D0 candidate pool (buffered, then rejected: never sent on bus 1)
    {0x1C0, 0, 8, .check_relay = true, .disable_static_blocking = true},    // lateral activation
    {0x121, 0, 8, .check_relay = true, .disable_static_blocking = true},    // accel (op long only)
    {0x117, 0, 8, .check_relay = true, .disable_static_blocking = true},    // long control status
    {0x118, 0, 8, .check_relay = true, .disable_static_blocking = true},    // long/ESP handshake
    {0x52A, 2, 8, .check_relay = false},                                    // ICC settings spoof (bus 2)
    {0x35B, 2, 8, .check_relay = false},                                    // ICC SVS/BSD/APA settings (bus 2)
  };

  static RxCheck fisker_rx_checks[] = {
    {.msg = {{0x115, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // wheel speeds
    {.msg = {{0x318, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // speed + brake
    {.msg = {{0x1C2, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // steering angle
    {.msg = {{0x1C4, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // driver torque
    // ADAS_0x313 on bus 2 (cam side): ADAS ACC state (ADAS_Sts_ACC_ICC). Replaces
    // the previous VCU_0x358 read now that we spoof ICC_0x52A into ADAS ACC mode.
    // The safety framework gates rx_hook execution on rx_checks membership, so this
    // must be present for our pcm_cruise_check to actually fire on 0x313.
    {.msg = {{0x313, 2, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ADAS ACC state
    // VCU_0x358: VCU basic cruise state — the cruise source on cars without ACC coded.
    {.msg = {{0x358, 0, 8,  10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // VCU basic CC state
    // MFS_0x514 (MFSS buttons) whitelisted so our fisker_rx_hook actually runs on it —
    // the safety framework gates rx_hook execution on rx_checks membership. Without this,
    // our MFS_RiBtnSouth read in the rx hook was dead code and mads_button_press never
    // toggled, so the lateral gate never opened on a button press.
    {.msg = {{0x514, 0, 8,  20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // MFSS buttons (MADS trigger)
  };

  // Fisker on-vehicle bring-up: honor the LONG_CONTROL bit on release too. Stock openpilot
  // hides longitudinal behind ALLOW_DEBUG for release builds, but this is a personal test
  // fork and we need the flag to work so alpha_long can gate 0x121/0x117/0x118 in this
  // firmware (release-tizi panda is not built with ALLOW_DEBUG). openpilot still gates
  // openpilotLongitudinalControl behind AlphaLongitudinalEnabled, so this is only reachable
  // by an explicit developer opt-in.
  const uint16_t FISKER_FLAG_LONGITUDINAL_CONTROL = 1;
  fisker_longitudinal = GET_FLAG(param, FISKER_FLAG_LONGITUDINAL_CONTROL);
  fisker_adas_cc_state = 0;
  fisker_vcu_cc_state = 0;

  // Initialise the ICC relay tracker. Seeding to "now" opens its window immediately so
  // the OEM's first bus-0 frame can't race openpilot's spoof — openpilot is already
  // pushing 0x52A spoofs by the time safety mode starts.
  fisker_icc_settings_tx_last = microsecond_timer_get();
  // Optional ICC_0x35B relay stays idle at init — the OEM's own 0x35B flows until
  // openpilot actually starts sending its overridden copy.
  fisker_icc_0x35b_tx_last = 0U;
  fisker_icc_0x35b_tx_seen = false;
  // Lateral relay stays idle at init — carcontroller only starts TXing lateral
  // when MADS engages, and we want OEM 0x1D0/0x1C0 to flow through until then.
  fisker_lat_tx_last = 0U;

  // cppcheck-suppress knownConditionTrueFalse
  return fisker_longitudinal ? BUILD_SAFETY_CFG(fisker_rx_checks, FISKER_LONG_TX_MSGS)
                             : BUILD_SAFETY_CFG(fisker_rx_checks, FISKER_TX_MSGS);
}

static bool fisker_fwd_hook(int bus_num, int addr) {
  // Forwarding intercept: keep the OEM ADAS module (bus 2) alive by forwarding all
  // traffic between it and the vehicle (bus 0), so the car stays fault-free. Only steal
  // the steering command — block the OEM's 0x1D0 (bus 2 -> vehicle) while openpilot is
  // actively steering; openpilot injects its own on bus 0. When disengaged the OEM's
  // 0x1D0 is forwarded so the EPS keeps receiving a steering frame.
  bool block_msg = false;
  if (bus_num == 2) {
    // Steering angle (0x1D0) + lateral activation (0x1C0): time-based relay. OEM's
    // bus-2 → bus-0 stream is blocked while openpilot has TX'd its own in the last
    // 500 ms. That window covers both the live MADS engagement AND the post-disengage
    // fade (carcontroller keeps TXing Req=0 for FADE_TICKS so the EPS transitions
    // Active→Available on our terms, not from an OEM command surprise). If openpilot
    // crashes mid-engagement, the window naturally expires and OEM resumes.
    // 0x1C0 (lateral activation): time-based relay as before — openpilot sends its own
    // 0x1C0 directly on bus 0, so block OEM's while we're engaged.
    if ((addr == 0x1C0) && fisker_lat_relay_active()) {
      block_msg = true;
    }
    // 0x1D0 is NOT blocked here: it must reach the forward path so fisker_fwd_modify can
    // substitute our candidate (when armed) or pass OEM's through (when disarmed). The
    // modify hook drops it when armed-but-no-candidate, so OEM's raw 0x1D0 never reaches
    // the EPS during engagement.
    // Longitudinal (0x121) is unaffected by MADS — MADS is lateral-only. Only block the
    // OEM's accel when cruise is engaged and openpilot is driving longitudinal.
    if (fisker_longitudinal && (addr == 0x121) && controls_allowed) {
      block_msg = true;
    }
  }
  if (bus_num == 0) {
    // ICC settings spoof (0x52A, ICC → ADAS/FCM): block the real ICC_0x52A from
    // reaching bus 2 while openpilot is TXing its overridden copy. The spoof is the
    // ENABLE for ACC — ADAS stays at ACC_Off until it sees our overridden byte 4 —
    // so blocking must NOT be gated on controls_allowed (chicken-and-egg: cruise
    // can't enable because the spoof is gated on cruise). Time-based: block only
    // while openpilot has recently sent one of its own copies. If openpilot stops,
    // the ICC's original resumes flowing to ADAS cleanly.
    if ((addr == 0x52A) && fisker_icc_relay_active()) {
      block_msg = true;
    }
    // ICC_0x35B (ICC -> ADAS): block the OEM's own 0x35B whenever openpilot has recently
    // emitted its overridden copy on bus 2. Idle (not blocking) until op's first TX.
    if ((addr == 0x35B) && fisker_icc_0x35b_relay_active()) {
      block_msg = true;
    }
    // Keep openpilot's injected actuator frames OFF the OEM ADAS module's bus.
    // ADAS_0x1D0/0x1C0 (and the longitudinal 0x121/0x117/0x118) are authored BY
    // the ADAS module; in stock operation they only ever travel ADAS(bus2) ->
    // vehicle(bus0). openpilot injects its own copies on bus 0 to drive the EPS/VCU
    // directly. Panda's default bus0->bus2 forwarding reflects those injections back
    // to the OEM ADAS, which then sees a SECOND sender of its own message IDs and
    // logs a DTC — observed on-vehicle 2026-10-19 as ADAS_Sts_ACC_ICC going to the
    // fault state and staying there until an ignition cycle. Blocking is safe
    // UNCONDITIONALLY: nothing on bus 0 legitimately originates these IDs, so the
    // only bus0->bus2 copy that ever exists is openpilot's own injection. (EPS_*,
    // VCU_*, wheel-speed etc. still forward bus0->bus2 normally so the ADAS keeps
    // sensing vehicle state.)
    if ((addr == 0x1D0) || (addr == 0x1C0)) {
      block_msg = true;
    }
    if (fisker_longitudinal && ((addr == 0x121) || (addr == 0x117) || (addr == 0x118))) {
      block_msg = true;
    }
    // (EPS_0x1C2 block disabled — see fiskercan.py / carcontroller.py comments.)
  }
  return block_msg;
}

const safety_hooks fisker_hooks = {
  .init = fisker_init,
  .rx = fisker_rx_hook,
  .tx = fisker_tx_hook,
  .fwd = fisker_fwd_hook,
  .get_counter = fisker_get_counter,
  .fwd_modify = fisker_fwd_modify,
};
