
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// Contains configuration settings for the system


// Temperature Sensor Settings
#define TEMP_READ_INTERVAL_MS 10*1000UL
#define TEMP_ALERT_THRESHOLD  0.10f

const float HIGH_TEMP_THRESHOLD = 55.0;  // degrees C
const float LOW_TEMP_RESET_THRESHOLD = 45.0; // degrees C
const uint32_t ALERT_CONFIRMATION_TIME = 5000UL; // 5 seconds

  // ───── Fan control thresholds (°C) ─────
  // below FAN_OFF_TEMP the fan stays off
  const float FAN_OFF_TEMP = 25.0f;
  // once this temp exceeded, fan switches on
  const float FAN_ON_TEMP  = 30.0f;
  // start speed ramp at this temperature
  const float FAN_MIN_TEMP = 30.0f;
  // hit this temp (or above) for full speed
  const float FAN_MAX_TEMP = 50.0f;
  // Minimum PWM value for the fan (X-255)
  const uint8_t FAN_MIN_PWM = 80;







#endif // CONFIG_H