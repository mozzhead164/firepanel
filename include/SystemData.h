#ifndef SYSTEM_DATA_H
#define SYSTEM_DATA_H

#include <stdint.h>
#include <Bounce2.h>


// Per-Channel Camera Sata Structure
struct ChannelData 
{
  // ——— Camera-input fields ———
  uint8_t       pin;                // Camera Pins to debouncer.attach()
  Bounce        debouncer;          // per-channel Bounce object
  volatile bool state;              // last raw state (LOW = Pressed)
  volatile bool triggered;          // set on debouncer.fell()
  uint32_t      triggerTimestamp;   // when that happened
  bool cableConnected;              // cable connected state (LOW = Connected)

  // ——— Trigger-latch flags ———
  bool cameraTriggered;             // Track Camera Input
  bool     dummyTriggered;          // Track Dummy Output
  bool thermalTriggered;            // Track Thermal Output
  uint32_t lastTriggerTime;         // Timestamp of Camera / Dummy Activation
  uint32_t dummyLastTrigger;
  uint32_t thermalLastTrigger;      // Timestamp of Thermal Activation

  // ——— Output-confirmation flags ———
  bool liveOutputConfirmed;         // Track Live Output Confirmation
  bool dummyOutputConfirmed;        // Track Dummy Output Confirmation
  
  // ——— Front-panel button flags ———
  bool buttonPressed;               // Track Button Press
  uint32_t lastPressedTime;         // Timestamp of Button Press
  
  // ——— LED caching ———
  uint8_t ledRed;                   // LED Red State
  uint8_t ledGreen;                 // LED Green State
};





// System Mode
enum SystemMode {
  MODE_ARMED,
  MODE_TEST,
  MODE_NO_OUTPUT
};

struct ModeSwitchData {
  uint8_t     pin;        // physical pin
  bool        state;      // last raw reading (true=HIGH, false=LOW)
  bool        prevState;  // previous raw reading
  Bounce      debouncer;  // per-switch debouncer
  const char* name;       // "ARMED", "TEST", or "NO OUTPUT"
};


// Full system-wide data
struct SystemData 
{
  // ——— Camera State Fields ———
  ChannelData channels[8];          // Array of Camera Channels

  // ——— Mode Switch State ———
  ModeSwitchData modeSwitches[3];   // our three positions
  SystemMode systemMode;            // Current System Mode

  // ——— BreakGlass State ———
  uint8_t bgPin;                    // Break Glass Pin
  Bounce bgDebouncer;               // per-channel Bounce object
  volatile bool bgState;            // last raw state (LOW = Pressed)
  bool bgTriggered;                 // set on debouncer.fell()
  uint32_t bgTimestamp;             // when that happened
  
  // ——— Temperature Fields ——— 
  float averageTemp;                // Average Temperature
  bool tempAlert;                   // Temperature Alert Flag     

  // ——— PSU Voltage Fields ———
  float psu1Voltage;                // PSU 1 Voltage
  float psu2Voltage;                // PSU 2 Voltage
  bool psu1UnderVolt;            // PSU 1 Under Voltage Flag
  bool psu2UnderVolt;            // PSU 2 Under Voltage Flag
  
  // ——— LED caching ———
  uint8_t sysLedRed;                // System LED Red State
  uint8_t sysLedGreen;              // System LED Green State
  uint8_t sysLedBlue;               // System LED Blue State
};


extern SystemData systemData;       // Global instance of SystemData

#endif // SYSTEM_DATA_H