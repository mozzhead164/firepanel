#include <Arduino.h>
#include "EventManager.h"
#include "LedControl.h"
#include "SerialComms.h"



// Internal global variable to hold the registered callback.
static EventCallback globalEventCallback = 0;

void registerEventCallback(EventCallback cb) {
    globalEventCallback = cb;
}

// Event callback: called whenever an event is dispatched.
void eventHandler(const Event *event)
{
  // Handle Cable Change Event
  if (event->type == EVENT_CABLE_CHANGE)
  {
    // Update the global cableConnected array defined in LedControl.cpp
    systemData.channels[event->channel].cableConnected = event->connected;


    // Update the LED immediately based on the new cable state.
    uint8_t ledNum = event->channel + 1;
    if (event->connected)
    {
      setFrontPanelLED(ledNum, 0, TARGET_GRN); // Connected: green-ish color.
    }
    else
    {
      setFrontPanelLED(ledNum, TARGET_RED, 0); // Disconnected: red-ish color.
    }
  }
  
  // Handle Channel Trigger Event
  if (event->type == EVENT_CHANNEL_TRIGGER)
  {
    // 1-based channel in JSON for clarity
    uint8_t humanCh = event->channel + 1;

    // send a "channel_trigger" frame to the Pi
    jsonDoc.clear();
    jsonDoc["type"] = "channel_trigger";
    jsonDoc["channel"] = humanCh;
    sendJson(jsonDoc);

    #ifdef DEBUG_SERIAL
        Serial.print(F("↔ Sent channel_trigger for CH"));
        Serial.println(humanCh);
    #endif
  }

  // Handle Output Confirmation Event
  if (event->type == EVENT_OUTPUT_CONFIRM)
  {
    // Build and send the JSON frame for output confirmations
    uint8_t humanCh = event->channel;       // 1–8 as you mapped them
    uint8_t idx     = humanCh - 1;          // convert to 0–7 array index
    bool isDmy = event->connected; // true=dummy, false=live

    // 1) Local log
    #ifdef DEBUG_OUTPUT_SENSE
      Serial.print(" ↔ Confirmed output on channel ");
      Serial.print(humanCh);
      if (isDmy)
        Serial.print(" (Dummy)");
      Serial.println(".");
    #endif

    // 2) Build + debug‐print the raw JSON
    jsonDoc.clear();
    jsonDoc["type"] = "output_confirm";
    jsonDoc["channel"] = humanCh;
    jsonDoc["dummy"] = isDmy;

    // 3) record for LED flashing:
    systemData.channels[idx].liveOutputConfirmed  = !isDmy;
    systemData.channels[idx].dummyOutputConfirmed =  isDmy;
    systemData.channels[idx].lastTriggerTime      = millis();


    #ifdef DEBUG_SERIAL
        Serial.print("[DEBUG 🔧] → ");
        serializeJson(jsonDoc, Serial);
        Serial.println();
    #endif

    // 3) Actually send to the Pi
    sendJson(jsonDoc);
    Serial1.flush(); // ensure it all goes out before we move on
  }
  
  // Handle Break‐Glass Event
  if (event->type == EVENT_BREAK_GLASS) {
    // 1) Local alert
    Serial.println(" [ ⚠️  ALERT!  ⚠️ ] Break Glass Triggered!");

    // 2) Send a JSON frame to the Pi
    jsonDoc.clear();
    jsonDoc["type"] = "alert";
    jsonDoc["alertType"] = "break_glass";
    sendJson(jsonDoc);

  #ifdef DEBUG_BREAK_GLASS
      Serial.println("[DEBUG 🔧] → { \"type\": \"break_glass\" }");
  #endif
}

  // Handle PSU Undervoltage Event
  if (event->type == EVENT_PSU_UNDERVOLTAGE) {
    uint8_t psu = event->channel;  // 1 or 2

    // Local alert
    Serial.print(" [ ⚠️  ALERT!  ⚠️ ] PSU ");
    Serial.print(psu);
    Serial.println(" Under-Voltage!");

    // Send JSON frame to the Pi
    jsonDoc.clear();
    jsonDoc["type"] = "alert";
    jsonDoc["alertType"] = "psu_undervoltage";
    jsonDoc["psu"]  = psu;
    jsonDoc["voltage"] = (psu == 1)
                         ? systemData.psu1Voltage
                         : systemData.psu2Voltage;
    sendJson(jsonDoc);
  }

  // Handle PSU Voltage Restoration
  if (event->type == EVENT_PSU_RESTORED) {
      uint8_t psu = event->channel;
      Serial.print(" [ ℹ️  Alert!  ℹ️ ] PSU ");
      Serial.print(psu);
      Serial.println(" Voltage Restored.");
      
      jsonDoc.clear();
      jsonDoc["type"] = "alert";
      jsonDoc["alertType"] = "psu_restored";
      jsonDoc["psu"]     = psu;
      jsonDoc["voltage"] = (psu == 1)
                            ? systemData.psu1Voltage
                            : systemData.psu2Voltage;
      sendJson(jsonDoc);
  }

}

void dispatchEvent(const Event* event) {
    if (globalEventCallback) {
       globalEventCallback(event);
    }
}
