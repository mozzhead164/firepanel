#include <Arduino.h>
#include "Pins.h"
#include "Buzzer.h"
#include "SelfTest.h"
#include "SystemState.h"
#include "SystemData.h"
#include "LedControl.h"
#include "Temperature.h"
#include "SerialComms.h"
#include "EventManager.h"
#include "HandleInputs.h"
#include "VoltageMeasure.h"



/* TODO
- Broken Front Panel Button Presses
- Broken Mode Switch + LED
- Check All Other Functions Since Code Overhaul
- Check JSON Serial Communication - Changed From v7 to v6

*/

// Arduino Compile Test 3

// Function Prototypes
void updateStartupAnimation();
void startupAnimation();
void updateCameraInputs();
void processChannelStates();
void initTLCs();


// Declare Pi Serial Communication Variables
SystemConnectionState systemConnectionState = STATE_BOOTING;

// Historical System Mode for Comparison
SystemMode prevSystemMode = MODE_NO_OUTPUT;

StaticJsonDocument<128> jsonDoc;


// the setup function runs once when you press reset or power the board
void setup()
{
  // Begin Serial Monitor
  Serial.begin(115200);

  // System Startup Message
  Serial.print("\n\n ✅ System Initialising...✅ \n\n");

  // Begin Serial Communication with Raspberry Pi
  initPiSerial();

  // Initialise Camera Inputs + Mode Switch
  initInputs();

  // Initialise Outputs
  initOutputs();

  #ifdef USE_BUZZER_OUTPUT
    // Initialise Buzzer
    Buzzer::init();
  #endif

  // Initialize LED Control
  initTLCs();

  // Initialize PCF8574 IO Expanders
  initPCFs();

  // Initialize Interrupts
  initPinInterrupts();

  // Initialize Mode Switch
  initModeSwitch();

  // Initialize Temperature Sensor
  initTemperature();

  // Initialize PSU Voltage Measurement
  read_psuVoltages();

  // Register the event handler.
  registerEventCallback(eventHandler);

  // Initialize Startup Animation
  startupAnimation();

  // Set Connection State For Pi Communication
  systemConnectionState = STATE_WAITING_FOR_PI;

  // Setup Complete
  Serial.println(F("\n\n ✅ Initialisation Complete...✅\n\n"));

  #ifdef DEBUG_STARTUP
    runSystemSelfTest();
  #endif

  #ifdef SKIP_STARTUP_ANIMATION
    startupPhase = FINAL_WAIT;
    startupAnimationDone = true;
    updateCableConnectedStates();
  #endif

  #ifdef USE_BUZZER_OUTPUT
    // Short Buzzer Beep On Startup
    Buzzer::beep(50);
  #endif
}


// the loop function runs over and over again forever
void loop()
{
  // Update System State
  updateModeSwitch();

  // ——— detect mode changes and notify the Pi ———
  if (systemData.systemMode != prevSystemMode)
  {
    prevSystemMode = systemData.systemMode;

    // build a JSON packet
    jsonDoc.clear();
    jsonDoc["type"] = "alert";
    jsonDoc["alertType"] = "mode_change";
    jsonDoc["mode"] = modeToStr(systemData.systemMode);

    // local log
    #ifdef DEBUG_MODE_CHANGE
        Serial.print(F(" ↔ Mode changed to "));
        Serial.println(jsonDoc["mode"].as<const char *>());
    #endif

    // send the JSON packet to the Pi
    sendJson(jsonDoc);
  }

  // Handle Pi Serial Communication
  pollJsonSerial();  // keep processing any replies

  // Send Heartbeat to Pi Every 10 Seconds
  sendHeartbeat();  // Send Heartbeat to Pi

  // Determine mode
  uint8_t mode = systemData.systemMode;

  // Check & Update Start Animation - is it complete?
  if (!startupAnimationDone) { updateStartupAnimation(mode); }

  // Update Front Panel LEDs
  updateTriggeredChannelLEDs();

  // Update System Mode LED
  updateTriColourLED(systemData.systemMode, RGB_BRIGHTNESS);

  // Check Camera Inputs
  updateInputs();

  // Check Break Glass Input
  updateBreakGlassInput();

  // Check ISR Flags For Triggered Events
  HandleInterrupts();

  // Update Front Panel Button States
  updateFpButtonStates();

  // Check For Channel Triggers and Activate Outputs
  processChannelStates();

  // Request PCB Temperature
  requestTemperature();

  // Proactive alert: if dramatic change occurred
  if (systemData.tempAlert)
  {
    jsonDoc.clear();
    jsonDoc["type"] = "alert";
    jsonDoc["alertType"] = "temp_alert";

    // create a "data" object and put avgTemp in it
    JsonObject d = jsonDoc["data"].to<JsonObject>();
    d["avgTemp"] = systemData.averageTemp;

    sendJson(jsonDoc);
    systemData.tempAlert = false;
  }

  // Control Fan Speed
  controlFanSpeed();

  // Read PSU Voltages
  read_psuVoltages();

  #ifdef USE_BUZZER_OUTPUT
    // Update Buzzer State
    Buzzer::update();
  #endif
}
