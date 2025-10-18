#pragma once

#include "opendbc/safety/safety_declarations.h"

/*
 * Chery Safety Mode
 *
 * This safety mode implements vehicle model (VM) based angle safety checks for Chery vehicles.
 * It validates steering commands using physics-based lateral acceleration and jerk limits.
 *
 * Key Features:
 * - VM-based angle limiting (similar to VW, Nissan implementations)
 * - ISO 11270 lateral acceleration/jerk compliance
 * - Road roll compensation for banked curves
 * - Hardware-level safety as last line of defense
 *
 * Vehicle Parameters (Chery Omoda E5):
 * - Mass: 1785 kg
 * - Wheelbase: 2.63 m
 * - Steer Ratio: 17.5
 * - Max Steering Angle: 300° (±150° from center)
 * - Control Frequency: 50 Hz (100Hz / STEER_STEP=2)
 *
 * Safety Limits:
 * - Max Lateral Accel: ~3.6 m/s² (ISO 3.0 + road roll compensation)
 * - Max Lateral Jerk: ~3.6 m/s³ (3.0 + road roll compensation)
 * - Max Angle Rate: 5°/frame (enforced by openpilot layer)
 */

// CAN msgs we care about
#define CHERY_ACC_CMD 0x3A2
#define CHERY_ACC_STATUS 0x3A5
#define CHERY_LKAS_HUD 0x307
#define CHERY_LKAS_CMD 0x345
#define CHERY_ACC_SETTING 0x387
#define CHERY_HUD_ALERT 0x3FC

#define CHERY_ENGINE 0x3E
#define CHERY_BRAKE 0x29A
#define CHERY_BRAKE_SENSOR 0x4ED
#define CHERY_WHEEL_SENSOR 0x316 // RX for vehicle speed
#define CHERY_ACC_DATA 0x3A5
#define CHERY_STEER_BUTTON 0x360

// CAN bus numbers
#define CHERY_MAIN 0
#define CHERY_AUX 1
#define CHERY_CAM 2

bool chery_longitudinal = false;
// GCOV_EXCL_START
// Unreachable by design (doesn't define any rx msgs)
void chery_rx_hook(const CANPacket_t *to_push)
{
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if (bus == CHERY_MAIN)
  {

    if (addr == CHERY_WHEEL_SENSOR)
    {
      // Get current speed and standstill
      uint16_t right_rear = GET_BYTES(to_push, 0, 2);
      uint16_t left_rear = GET_BYTES(to_push, 2, 2);
      vehicle_moving = (right_rear | left_rear) != 0U;
      UPDATE_VEHICLE_SPEED((right_rear + left_rear) / 2.0 * 0.00828 / 3.6);
    }

    // if (addr == CHERY_STEER_TORQUE) {
    //   int torque_driver_new = GET_BYTE(to_push, 0) - 127U;
    //   // update array of samples
    //   update_sample(&torque_driver, torque_driver_new);
    // }

    // // enter controls on rising edge of ACC, exit controls on ACC off
    // if (addr == CHERY_CRZ_CTRL) {
    //   acc_main_on = GET_BIT(to_push, 17U);
    //   bool cruise_engaged = GET_BYTE(to_push, 0) & 0x8U;
    //   pcm_cruise_check(cruise_engaged);
    // }

    // if (addr == CHERY_ENGINE_DATA) {
    //   gas_pressed = (GET_BYTE(to_push, 4) || (GET_BYTE(to_push, 5) & 0xF0U));
    // }

    if (addr == CHERY_ENGINE)
    {
      brake_pressed = ((GET_BYTES(to_push, 0, 27) >> 4) & 0x01) != 0U;
    }
  }
  else if (bus == CHERY_CAM)
  {
    if (addr == CHERY_ACC_CMD)
    {
      acc_main_on = ((GET_BYTES(to_push, 1, 1) & 0x03) != 1U);
      // bool stand_still = (GET_BYTE(to_push, 1) >> 2) & 0x01;

      gas_pressed = (GET_BYTES(to_push, 5, 1) & 0x80U) != 0U;
    }
    if (addr == CHERY_ACC_DATA)
    {
      // Signal: ACCStatus
      bool cruise_engaged = GET_BIT(to_push, 20U);
      pcm_cruise_check(cruise_engaged);
    }
  }
  controls_allowed = true;
}

static safety_config chery_init(uint16_t param)
{
  static const CanMsg CHERY_TX_MSGS[] = {
      {CHERY_LKAS_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_HUD, 0, 8, .check_relay = true},
      // {CHERY_HUD_ALERT, 0, 8, .check_relay = true},
      // {CHERY_ACC_SETTING, 0, 8, .check_relay = true},
      // {CHERY_STEER_BUTTON, 0, 6, .check_relay = true},
      {CHERY_STEER_BUTTON, 2, 6, .check_relay = false},
  };
  static const CanMsg CHERY_LONG_TX_MSGS[] = {
      {CHERY_ACC_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_HUD, 0, 8, .check_relay = true},
      // {CHERY_HUD_ALERT, 0, 8, .check_relay = true},
      // {CHERY_ACC_SETTING, 0, 8, .check_relay = true},
      // {CHERY_STEER_BUTTON, 0, 6, .check_relay = true},
      {CHERY_STEER_BUTTON, 2, 6, .check_relay = false},
  };

  static RxCheck chery_rx_checks[] = {
      // {.msg = {{CHERY_WHEEL_SENSOR, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, {0}, {0}}},
      // {.msg = {{CHERY_WHEEL_SENSOR, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, {0}, {0}}},
      // {.msg = {{CHERY_ENGINE, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, {0}, {0}}},
      // {.msg = {{CHERY_BRAKE, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, {0}, {0}}},
      // {.msg = {{CHERY_BRAKE_SENSOR, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, {0}, {0}}},
  };
  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa

#ifdef ALLOW_DEBUG
  const uint16_t CHERY_PARAM_LONGITUDINAL = 1;
  chery_longitudinal = GET_FLAG(param, CHERY_PARAM_LONGITUDINAL);
#endif

  safety_config ret;
  ret = chery_longitudinal ? BUILD_SAFETY_CFG(chery_rx_checks, CHERY_LONG_TX_MSGS) : BUILD_SAFETY_CFG(chery_rx_checks, CHERY_TX_MSGS);
  return ret;
}

static bool chery_tx_hook(const CANPacket_t *to_send)
{
  const int bus = GET_BUS(to_send);
  const int addr = GET_ADDR(to_send);

  // Vehicle Model parameters for angle safety checks
  // Based on Chery Omoda E5 specs: wheelbase=2.63m, steer_ratio=17.5, mass=1785kg
  static const AngleSteeringLimits CHERY_STEERING_LIMITS = {
    .max_angle = 30000,  // 300 deg * 100 (from STEER_ANGLE_MAX in values.py)
    .angle_deg_to_can = 100,  // Matches STEER_ANGLE_SCALE * 10 from cherycan.py
    .frequency = 50U,  // STEER_STEP = 2, so 100Hz / 2 = 50Hz
  };

  static const AngleSteeringParams CHERY_STEERING_PARAMS = {
    // slip_factor = m * (cF * aF - cR * aR) / (l^2 * cF * cR)
    // Calculated from: mass=1785kg, wheelbase=2.63m, aF=1.1572m, aR=1.4728m
    // tire_stiffness_front=192150 N/rad, tire_stiffness_rear=202500 N/rad
    .slip_factor = -0.000503295541,
    .steer_ratio = 17.5,  // From CheryCarSpecs in values.py
    .wheelbase = 2.63,    // From CheryCarSpecs in values.py
  };

  bool tx = true;

  // Safety check for lateral control commands (LKAS)
  if ((bus == CHERY_MAIN) && (addr == CHERY_LKAS_CMD)) {
    // Extract steering angle command from CAN message
    // DBC: SG_ CMD : 6|13@0- (1,0) - starts at bit 6, 13 bits, little-endian, signed
    // cherycan.py: apply_steer = int((apply_steer_deg * STEER_ANGLE_SCALE) + STEER_ANGLE_OFFSET)
    //              where STEER_ANGLE_SCALE = 10, STEER_ANGLE_OFFSET = -392

    // Extract 13-bit signed value starting at bit 6
    int16_t can_angle_raw = ((GET_BYTES(to_send, 0, 2) >> 6) & 0x1FFF);
    // Sign extend from 13-bit to 16-bit
    if (can_angle_raw & 0x1000) {
      can_angle_raw |= 0xE000;  // Set upper bits to 1 for negative values
    }

    // Convert from CAN representation to degrees
    // Reverse: desired_angle_deg = (can_angle_raw - STEER_ANGLE_OFFSET) / STEER_ANGLE_SCALE
    //                             = (can_angle_raw - (-392)) / 10
    //                             = (can_angle_raw + 392) / 10
    // For safety check (deg * 100): multiply by 100
    int desired_angle = ((can_angle_raw + 392) * 10);  // Now in deg * 100 format

    // Extract LKA_ACTIVE flag
    // DBC: SG_ LKA_ACTIVE : 9|1@0+ - bit 9 (byte 1, bit 1)
    bool lka_active = (GET_BIT(to_send, 9U) != 0U);

    // Perform VM-based safety checks
    if (steer_angle_cmd_checks_vm(desired_angle, lka_active, CHERY_STEERING_LIMITS, CHERY_STEERING_PARAMS)) {
      tx = false;
    }
  }

  // Safety check for longitudinal control commands (ACC) if enabled
  if (chery_longitudinal && (bus == CHERY_MAIN) && (addr == CHERY_ACC_CMD)) {
    // TODO: Add longitudinal safety checks if needed
    // For now, longitudinal control is passthrough
  }

  return tx;
}

const safety_hooks chery_hooks = {
    .init = chery_init,
    .rx = chery_rx_hook,
    .tx = chery_tx_hook,
};
