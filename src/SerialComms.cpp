#include "SerialComms.h"    // for JSON handling
#include "SystemState.h"    // for systemConnectionState
#include "SystemData.h"     // for systemData
#include "SelfTest.h"       // for selfTestPassed
#include "Pins.h"           // for Serial1
#include "LedControl.h"     // for mode LED updates
#include "HandleInputs.h"   // for thermalPins access
#include "EventManager.h"   // for dispatching events if needed
#include <ArduinoJson.h>    // for JSON handling



extern SystemConnectionState systemConnectionState; // for tracking Pi status



// Serial Communication Defines
#define SERIAL_BAUD 115200
#define START_MARKER '<'
#define END_MARKER '>'
#define MAX_DATA_LENGTH 64



// Initialize the Serial1 port for communication with the Pi
void initPiSerial()
{
  Serial1.begin(SERIAL_BAUD);
  // Optional: give it a moment and flush
  delay(10);
  while (Serial1.available())
    Serial1.read();

  jsonDoc.clear();
}


// Accumulate incoming <…> frames, parse JSON, dispatch by "type"
void pollJsonSerial()
{
  static bool receiving = false;
  static char  buffer[MAX_DATA_LENGTH+1];
  static uint8_t bufpos = 0;

  while (Serial1.available())
  {
    char c = Serial1.read();

    #ifdef DEBUG_PI_SERIAL
      Serial.print(c);
    #endif

    if (!receiving)
    {
      if (c == START_MARKER)
      {
        receiving = true;
        bufpos    = 0;
      }
    }
    else
    {
      if (c == END_MARKER)
      {
        receiving = false;
        buffer[bufpos] = '\0';

        #ifdef DEBUG_PI_SERIAL
          Serial.print(millis()); Serial.print(" ms: Received frame: <");
          Serial.print(buffer);
          Serial.println(">");
        #endif

        // Attempt to parse JSON
        DeserializationError err = deserializeJson(jsonDoc, buffer);
        if (!err)
        {
          handleIncomingCommand(jsonDoc);  // 🔥 CALL PROPER COMMAND HANDLER
          #ifdef DEBUG_PI_SERIAL
            Serial.print(millis()); Serial.println(" ms: Calling handleIncomingCommand");
          #endif
        }
        else
        {
          Serial.print(F("[ERROR 💥] JSON parse error: "));
          Serial.println(err.c_str());
        }
      }
      else if (bufpos < MAX_DATA_LENGTH) {
        buffer[bufpos++] = c;
      }
    }
  }
}


// Handle incoming commands from the Pi
void handleIncomingCommand(const JsonDocument& doc) {
  if (!doc["type"].is<const char*>()) {
    sendNack("Missing type field");
    return;
  }

  const char* type = doc["type"].as<const char*>();

  if (!type)
  {
    sendNack("Missing type field");
    return;
  }

  if (strcmp(type, "get_data") == 0) {
    handleGetData();
  }
  else if (strcmp(type, "trigger_thermal") == 0) {
    handleTriggerThermal();
  }
  else if (strcmp(type, "reset_thermal") == 0) {
    handleResetThermal();
  }
  else {
    sendNack("Unknown command type");
  }
}


// Validate the incoming JSON command
bool validateCommand(const JsonDocument& doc, const char* requiredType, const char* requiredField) {
  if (!doc["type"].is<const char*>()) {
    return false;
  }

  const char* type = doc["type"].as<const char*>();
  if (strcmp(type, requiredType) != 0) {
    return false;
  }

  if (requiredField && !doc[requiredField].is<JsonVariantConst>()) {
    return false;
  }

  return true;
}


// Convert SystemMode enum to string for debugging
const char* modeToStr(SystemMode mode) {
  switch (mode) {
    case MODE_ARMED: return "ARMED";
    case MODE_TEST: return "TEST";
    case MODE_NO_OUTPUT: return "NO_OUTPUT";
    default: return "UNKNOWN";
  }
}


// Send an ACK or NACK response to the Pi
void sendAck(const char *type, uint8_t channel)
{
  jsonDoc.clear();
  jsonDoc["type"] = "ack";
  jsonDoc["command"] = type;
  if (channel > 0)
  {
    jsonDoc["channel"] = channel; // optional if related to a channel
  }
  sendJson(jsonDoc);
}


// Send a NACK response to the Pi with a reason
void sendNack(const char *reason)
{
  jsonDoc.clear();
  jsonDoc["type"] = "nack";
  jsonDoc["reason"] = reason;
  sendJson(jsonDoc);
}




// Send a heartbeat message to the Pi every 10 seconds
void sendHeartbeat()
{
  static unsigned long lastHeartbeat = 0;

  if (millis() - lastHeartbeat > 10*1000UL) { // every 10 seconds
    jsonDoc.clear();
    jsonDoc["type"] = "heartbeat";
    jsonDoc["payload"] = "alive";
    sendJson(jsonDoc);
    lastHeartbeat = millis();
  }
}



// Send a framed JSON document over Serial1
void sendJson(const JsonDocument &doc)
{
  Serial1.write(START_MARKER);
  serializeJson(doc, Serial1);
  Serial1.write(END_MARKER);

  #ifdef DEBUG_PI_SERIAL
    Serial.print(F("[DEBUG 🔧] Sending framed JSON: <"));
    serializeJsonPretty(doc, Serial);
    Serial.println(F(">"));
  #endif
}



// Handle the "get_data" command from the Pi
void handleGetData()
{
  // 1) compute bitmasks
  // Create Bitmasks for camera, thermal, and cable states
  uint8_t cameraMask = 0, thermalMask = 0, confirmMask = 0, cableMask = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    // Update Camera Mask
    if (systemData.channels[i].cameraTriggered)  cameraMask  |= (1 << i);
    // Update Thermal Mask
    if (systemData.channels[i].thermalTriggered) thermalMask |= (1 << i);
    // Update Cable Connected Mask
    if (systemData.channels[i].cableConnected)   cableMask   |= (1 << i);
    // Update Confirm Mask
    bool driven = (systemData.systemMode == MODE_ARMED)
                  ? systemData.channels[i].liveOutputConfirmed
                  : systemData.channels[i].dummyOutputConfirmed;
    if (driven) confirmMask |= (1 << i);
  }

  // Create JSON object
  jsonDoc.clear();
  jsonDoc["type"] = "data";

  // Scalars
  jsonDoc["systemModeStr"] = modeToStr(systemData.systemMode);
  jsonDoc["avgTemp"]       = systemData.averageTemp;
  jsonDoc["breakGlass"]    = systemData.bgTriggered;
  jsonDoc["tempAlert"]     = systemData.tempAlert;
  jsonDoc["psu1UnderVolt"] = systemData.psu1UnderVolt;
  jsonDoc["psu2UnderVolt"] = systemData.psu2UnderVolt;

  // Masks
  jsonDoc["cameraMask"]    = cameraMask;
  jsonDoc["thermalMask"]   = thermalMask;
  jsonDoc["cableMask"]     = cableMask;
  jsonDoc["confirmMask"]   = confirmMask;


  #ifdef DEBUG_SERIAL
    Serial.print("[DEBUG 🔧] Masks C,T,CB = ");
    Serial.print(cameraMask, BIN);  Serial.print(", ");
    Serial.print(thermalMask, BIN); Serial.print(", ");
    Serial.print(confirmMask, BIN); Serial.print(", ");
    Serial.println(cableMask, BIN);

    Serial.println(F("[DEBUG 🔧] Sending system data to Pi"));
    serializeJsonPretty(jsonDoc, Serial);
    Serial.println();
  #endif

  sendJson(jsonDoc);
}


// Handle the "trigger_thermal" command from the Pi
void handleTriggerThermal()
{
  if (!jsonDoc["channel"].is<int>())
  {
    sendNack("Missing channel field");
    return;
  }

  uint8_t ch = jsonDoc["channel"];

  if (ch < 1 || ch > 8)
  {
    sendNack("Invalid channel number");
    return;
  }

  systemData.channels[ch - 1].thermalTriggered = true;
  systemData.channels[ch - 1].thermalLastTrigger = millis();

  uint8_t pin = pgm_read_byte_near(thermalPins_P + (ch - 1));
  digitalWrite(pin, HIGH);

  #ifdef USE_BUZZER_OUTPUT
    Buzzer::beep(2000); // short beep
  #endif

  #ifdef DEBUG_TRIGGER
    Serial.print(F("[THERMAL 🔥] Triggered channel: "));
    Serial.println(ch);
  #endif

  sendAck("trigger_thermal", ch);
}


// Handle the "reset_thermal" command from the Pi
void handleResetThermal()
{
  if (!jsonDoc["channel"].is<int>())
  {
    sendNack("Missing channel field");
    return;
  }

  uint8_t ch = jsonDoc["channel"];
  if (ch < 1 || ch > 8)
  {
    sendNack("Invalid channel number");
    return;
  }

  systemData.channels[ch - 1].thermalTriggered = false;

  uint8_t pin = pgm_read_byte_near(thermalPins_P + (ch - 1));
  digitalWrite(pin, LOW);

  Serial.print(F("[THERMAL 🔥] Deactivated channel "));
  Serial.println(ch);

  sendAck("reset_thermal", ch);
}

