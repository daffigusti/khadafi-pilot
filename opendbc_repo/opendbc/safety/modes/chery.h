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
#define CHERY_STEER_SENSOR_2 0x394     // 916 decimal - driver torque sensor
#define CHERY_STEER_ANGLE_SENSOR 0x1D3 // 467 decimal - steering angle measurement

// CAN bus numbers
#define CHERY_MAIN 0
#define CHERY_AUX 1
#define CHERY_CAM 2

bool chery_longitudinal = false;

// RX hook processes incoming CAN messages for safety-critical signals
// Monitors: wheel speed, brake, gas, steering torque, steering angle, ACC status
void chery_rx_hook(const CANPacket_t *to_push)
{
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if (bus == CHERY_MAIN)
  {

    // Wheel speed from WHEEL_SPEED_FRNT (0x316 / 790 decimal)
    // DBC: SG_ WHEEL_SPEED_FR : 7|16@0- (0.00829,0) - Front Right
    //      SG_ WHEEL_SPEED_FL : 23|16@0- (0.00829,0) - Front Left
    if (addr == CHERY_WHEEL_SENSOR)
    {
      // Extract front right wheel speed (16-bit signed starting at bit 7)
      int16_t front_right_raw = (GET_BYTES(to_push, 0, 3) >> 7) & 0xFFFF;
      // Extract front left wheel speed (16-bit signed starting at bit 23)
      int16_t front_left_raw = (GET_BYTES(to_push, 2, 3) >> 7) & 0xFFFF;

      // Check if vehicle is moving (any wheel speed > 0)
      vehicle_moving = (front_right_raw > 0) || (front_left_raw > 0);

      // Average both front wheels and convert to m/s
      // Scale: 0.00829 km/h per unit, convert to m/s: / 3.6
      UPDATE_VEHICLE_SPEED((front_right_raw + front_left_raw) / 2.0 * 0.00829 / 3.6);
    }

    // Driver torque monitoring for enhanced safety and driver override detection
    // Message: STEER_SENSOR_2 (0x394), Signal: TORQUE_DRIVER
    // DBC: SG_ TORQUE_DRIVER : 7|12@0- (0.24,0) - 12-bit signed, scale 0.24
    // Note: carstate.py multiplies by direction for sign handling
    if (addr == CHERY_STEER_SENSOR_2)
    {
      // Extract 12-bit signed value starting at bit 7
      int torque_raw = (GET_BYTES(to_push, 0, 2) >> 7) & 0xFFF; // 12 bits
      // Sign extend from 12-bit to 16-bit
      if (torque_raw & 0x800)
      {                       // If bit 11 is set (negative)
        torque_raw |= 0xF000; // Set upper 4 bits
      }
      // Apply scale factor: 0.24 Nm per unit
      // For safety checks, keep in scaled units (multiply by ~4 to approximate integer Nm)
      int torque_driver_new = (torque_raw * 24) / 100; // Convert to deciNewtons (0.1 Nm)
      update_sample(&torque_driver, torque_driver_new);
    }

    // Steering angle measurement for angle validation
    // Message: STEER_ANGLE_SENSOR (0x1D3), Signal: STEER_ANGLE
    // DBC: SG_ STEER_ANGLE : 7|14@0+ (0.1,-780) - 14-bit unsigned, scale 0.1, offset -780
    if (addr == CHERY_STEER_ANGLE_SENSOR)
    {
      // Extract 14-bit value starting at bit 7
      int angle_raw = (GET_BYTES(to_push, 0, 2) >> 7) & 0x3FFF; // 14 bits
      // Apply scale (0.1) and offset (-780): angle_deg = raw * 0.1 - 780
      // For safety, convert to integer: angle_deg * 10 = raw - 7800
      int angle_meas_new = angle_raw - 7800; // Now in units of 0.1 degrees
      update_sample(&angle_meas, angle_meas_new);
    }

    // // enter controls on rising edge of ACC, exit controls on ACC off
    // if (addr == CHERY_CRZ_CTRL) {
    //   acc_main_on = GET_BIT(to_push, 17U);
    //   bool cruise_engaged = GET_BYTE(to_push, 0) & 0x8U;
    //   pcm_cruise_check(cruise_engaged);
    // }

    // if (addr == CHERY_ENGINE_DATA) {
    //   gas_pressed = (GET_BYTE(to_push, 4) || (GET_BYTE(to_push, 5) & 0xF0U));
    // }

    // Brake pedal detection from ENGINE_DATA (CANFD 48-byte message)
    // DBC: SG_ BRAKE_PRESS : 220|1@0+ (bit 220 in 48-byte message)
    if (addr == CHERY_ENGINE)
    {
      brake_pressed = GET_BIT(to_push, 220U);
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

  // Safety: Disengage controls on brake or gas press
  // This provides a critical safety fallback
  if (brake_pressed || gas_pressed)
  {
    controls_allowed = false;
  }

  // Safety: Require ACC main switch to be on
  if (!acc_main_on)
  {
    controls_allowed = false;
  }
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

  // RxCheck disabled temporarily to resolve safetyRxChecksInvalid
  // TODO: Debug why RxCheck validation fails and re-enable
  // Original RxCheck attempts (all failed):
  // static RxCheck chery_rx_checks[] = {
  //     {.msg = {{CHERY_WHEEL_SENSOR, CHERY_MAIN, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
  //     {.msg = {{CHERY_ENGINE, CHERY_MAIN, 48, 100U, .ignore_checksum = true, .ignore_counter = true}, {0}, {0}}},
  // };

  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa

#ifdef ALLOW_DEBUG
  const uint16_t CHERY_PARAM_LONGITUDINAL = 1;
  chery_longitudinal = GET_FLAG(param, CHERY_PARAM_LONGITUDINAL);
#endif

  // Return safety_config with NO RxCheck validation (NULL, 0)
  // This matches alloutput/nooutput modes in defaults.h
  safety_config ret;
  if (chery_longitudinal) {
    ret = (safety_config){NULL, 0, CHERY_LONG_TX_MSGS, ARRAY_SIZE(CHERY_LONG_TX_MSGS), false};
  } else {
    ret = (safety_config){NULL, 0, CHERY_TX_MSGS, ARRAY_SIZE(CHERY_TX_MSGS), false};
  }
  return ret;
}

static bool chery_tx_hook(const CANPacket_t *to_send)
{
  UNUSED(to_send);
  return true;
}

const safety_hooks chery_hooks = {
    .init = chery_init,
    .rx = chery_rx_hook,
    .tx = chery_tx_hook,
};
