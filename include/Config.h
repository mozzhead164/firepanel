
#ifndef CONFIG_H
#define CONFIG_H

#pragma once

#include <stdint.h>

// Contains configuration settings for the system







// Voltage Measurement Constants
const float PSU1_VOLTAGE_SCALE = 16.8f;
const float PSU2_VOLTAGE_SCALE = 16.8f;

const float VOLTAGE_RESTORE = 10.5f; // threshold for voltage restoration check
const float VOLTAGE_UNDER   = 1.5f;  // threshold for undervoltage check




// LED Control Settings

// === Startup Animation Colours ===
static const uint8_t CHASE_COLOR_R = 16;
static const uint8_t CHASE_COLOR_G = 0;

// 60% target brightness (60% of 255 ≈ 153)
const uint8_t TARGET_RED = 32;
const uint8_t TARGET_GRN = 32;
const uint8_t TARGET_BLU = 32;







// Temperature Sensor Settings
#define TEMP_READ_INTERVAL_MS 10 * 1000UL
#define TEMP_ALERT_THRESHOLD  0.10f

const float HIGH_TEMP_THRESHOLD = 55.0;  // degrees C
const float LOW_TEMP_RESET_THRESHOLD = 45.0; // degrees C
const uint32_t ALERT_CONFIRMATION_TIME = 5000UL; // 5 seconds

// ───── Fan control thresholds (°C) ─────
// below FAN_OFF_TEMP the fan stays off
const float FAN_OFF_TEMP = 25.0f;
// once this temp exceeded, fan switches on
const float FAN_ON_TEMP  = 33.0f;
// start speed ramp at this temperature
const float FAN_MIN_TEMP = 34.0f;
// hit this temp (or above) for full speed
const float FAN_MAX_TEMP = 55.0f;
// Minimum PWM value for the fan (X-255)
const uint8_t FAN_MIN_PWM = 32;









#endif // CONFIG_H