
#include <Arduino.h>
#include "Temperature.h"


// Raw-state for our non-blocking DS18B20 driver
enum TempState { TS_IDLE, TS_CONVERT, TS_READ };
TempState tempState = TS_IDLE;



static void updateTemperatureAlerts();





// PF2/A2 on ATmega128 (the ONE_WIRE_BUS pin) :contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}
#define OW_DDR   DDRF
#define OW_PORT  PORTF
#define OW_PIN   PINF
#define OW_BIT   2

//— low-level OneWire bit-bang helpers —//
static inline void ow_low()     { OW_PORT &= ~(1<<OW_BIT); OW_DDR |=  (1<<OW_BIT); }
static inline void ow_release() { OW_DDR  &= ~(1<<OW_BIT); OW_PORT |=  (1<<OW_BIT); }
static inline bool ow_read()    { OW_DDR  &= ~(1<<OW_BIT); return OW_PIN & (1<<OW_BIT); }

// Reset + presence detect
static bool ow_reset() {
  ow_low(); delayMicroseconds(480);
  ow_release(); delayMicroseconds(70);
  bool pres = !ow_read();
  delayMicroseconds(410);
  return pres;
}

// Write/read 1 bit
static void     ow_writeBit(bool b) {
  if (b) { ow_low(); delayMicroseconds(6); ow_release(); delayMicroseconds(64); }
  else   { ow_low(); delayMicroseconds(70); ow_release(); delayMicroseconds(10); }
}
static bool     ow_readBit() {
  ow_low(); delayMicroseconds(6);
  ow_release(); delayMicroseconds(9);
  bool b = ow_read();
  delayMicroseconds(55);
  return b;
}

// Write/read full byte (LSB first)
static void     ow_writeByte(uint8_t v) { for (int i=0;i<8;i++, v>>=1) ow_writeBit(v&1); }
static uint8_t  ow_readByte()           { uint8_t v=0; for(int i=0;i<8;i++) if(ow_readBit()) v|=1<<i; return v; }



void initTemperature() {

  // ensure PF2/A2 has pull-up
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  delay(10);

  tempState = TS_IDLE;

  #ifdef DEBUG_TEMPERATURE
    Serial.println("\n Initialising Temperature Sensor: ");
    Serial.print(" ✅ OneWire Bus Detected: ");
    Serial.println(ow_reset() ? "OK" : "FAIL");
  #endif
}

/// call every loop(); does the state machine
void requestTemperature() {

  const uint8_t AVG_WINDOW_SIZE = 5;
  static float tempBuf[AVG_WINDOW_SIZE];
  static uint8_t bufIdx = 0, bufCount = 0;
  static uint32_t lastDebugTs = 0UL; // for debug-printing every 5s

  static unsigned long lastRequestTs = 0;
  

  switch (tempState) {
    case TS_IDLE:
      if (millis() - lastRequestTs >= TEMP_READ_INTERVAL_MS) {
        if (ow_reset()) {
          ow_writeByte(0xCC); // SKIP ROM
          ow_writeByte(0x44); // CONVERT T
          tempState = TS_CONVERT;
          lastRequestTs = millis();
        }
      }
      break;

    case TS_CONVERT:
      if (ow_readBit()) {
        tempState = TS_READ;
      }
      break;

    case TS_READ:
    {
      if (!ow_reset()) { tempState = TS_IDLE; break; }
      ow_writeByte(0xCC); ow_writeByte(0xBE);
      uint8_t data[9];
      for (uint8_t i = 0; i < 9; i++) data[i] = ow_readByte();
      int16_t raw = (int16_t(data[1])<<8)|data[0];

      
      float cur = raw * 0.0625;
      // prevTemperature = cur;

      // update buffer & compute average
      tempBuf[bufIdx] = cur;
      bufIdx = (bufIdx + 1) % AVG_WINDOW_SIZE;
      if (bufCount < AVG_WINDOW_SIZE) bufCount++;
      float sum = 0;
      for (uint8_t i = 0; i < bufCount; i++) sum += tempBuf[i];
      float avgTemp = sum / bufCount;

      // detect >10% jump
      static float baselineTemp = NAN;

      // initialize baseline on first run
      if (isnan(baselineTemp)) {
        baselineTemp = avgTemp;
      }

      // if we’ve cooled below baseline, track that
      if (avgTemp < baselineTemp) {
        baselineTemp = avgTemp;
      }

      // if we jump high enough above baseline, fire alert
      if (avgTemp > baselineTemp * (1 + TEMP_ALERT_THRESHOLD)) {
        systemData.tempAlert = true;
        baselineTemp = avgTemp;       // bump the baseline up to avoid re-alerting immediately
      }

      systemData.averageTemp = avgTemp;

      updateTemperatureAlerts();

      // new: debug-print every 5s on Serial
      #ifdef DEBUG_TEMPERATURE
      if (millis() - lastDebugTs >= 5000) {
        lastDebugTs = millis();
        Serial.print(" [DEBUG] AvgTemp=");
        Serial.print(avgTemp, 2);
        Serial.print(" °C");
        Serial.println();
      }
      #endif

      tempState = TS_IDLE;
      break;
    }
  }
}

void controlFanSpeed() {

  static bool   fanActive = false;
  float         t         = systemData.averageTemp;
  uint8_t       pwm       = 0;

  // hysteresis: only arm the fan at or above FAN_ON_TEMP,
  // and only disarm once you cool below FAN_OFF_TEMP
  if (!fanActive && t >= FAN_ON_TEMP) {
    fanActive = true;
  } 
  else if ( fanActive && t <= FAN_OFF_TEMP) {
    fanActive = false;
  }

  // if armed, compute a PWM between FAN_MIN_PWM…255
  if (fanActive) {
    if      (t >= FAN_MAX_TEMP) pwm = 255;
    else if (t <= FAN_MIN_TEMP) pwm = FAN_MIN_PWM;
    else {
      float frac = (t - FAN_MIN_TEMP) / (FAN_MAX_TEMP - FAN_MIN_TEMP);
      pwm = uint8_t(FAN_MIN_PWM + frac * (255 - FAN_MIN_PWM));
    }
  }

  analogWrite(FAN_CONTROL, pwm);
}






static void updateTemperatureAlerts() {

  float temp = systemData.averageTemp;
  static bool tempHighPending = false;
  static uint32_t tempHighStartTime = 0;

  if (!systemData.tempAlert) {
    if (temp > HIGH_TEMP_THRESHOLD) {
      if (!tempHighPending) {
        tempHighPending = true;
        tempHighStartTime = millis();
      }
      else if (millis() - tempHighStartTime > ALERT_CONFIRMATION_TIME) {
        systemData.tempAlert = true;
        Serial.println(F("[ALERT ⚠️] High temperature detected!"));
      }
    } else {
      tempHighPending = false;
    }
  }
  else {
    if (temp < LOW_TEMP_RESET_THRESHOLD) {
      systemData.tempAlert = false;
      Serial.println(F("[INFO ℹ️] Temperature back to safe level."));
    }
  }
}
