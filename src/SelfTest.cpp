#include <Arduino.h>
#include "SelfTest.h"
#include "LedControl.h"
#include "SerialComms.h"
#include "Temperature.h"
#include "SystemData.h"
#include <Arduino.h>
#include <Wire.h>
#include "MemoryUtils.h"


bool selfTestPassed = false;
bool selfTestCompleted = false;


bool ow_reset();


void runSystemSelfTest() {

    #ifdef DEBUG_STARTUP
      Serial.println(F("\n\n 🔧 Running System Self-Test... 🔧\n"));
    #endif

    // Initialisation System Self-Test
    Serial.println(F(" [INFO ℹ️] Checking I2C devices...\n"));
    bool i2cOk = true;

    // Check TLC1 (assume 0x60)
    if (!checkI2CDevice(0x60)) {
    Serial.println(F(" [FAIL ❌] TLC59116 (0x60) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F(" [PASS ✅] TLC59116 (0x60) OK"));
    }

    // Check TLC2 (assume 0x61 if you have a second one)
    if (!checkI2CDevice(0x61)) {
    Serial.println(F(" [FAIL ❌] TLC59116 (0x61) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F(" [PASS ✅] TLC59116 (0x61) OK"));
    }

    // Check PCF8574 (Front Panel Switches) 0x22
    if (!checkI2CDevice(0x22)) {
    Serial.println(F(" [FAIL ❌] PCF8574 (0x22) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F(" [PASS ✅] PCF8574 (0x22) OK"));
    }

    // Check PCF8574 (Cable Connectors) 0x23
    if (!checkI2CDevice(0x23)) {
    Serial.println(F(" [FAIL ❌] PCF8574 (0x23) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F(" [PASS ✅] PCF8574 (0x23) OK"));
    }

    // Check PCA9555 (Output Sensing) 0x24
    if (!checkI2CDevice(0x24)) {
    Serial.println(F(" [FAIL ❌] PCA9555 (0x24) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F(" [PASS ✅] PCA9555 (0x24) OK"));
    }

    Serial.print("\n ✅ OneWire Bus Detected: ");
    Serial.println(ow_reset() ? "OK" : "FAIL");

    // Final Summary
    if (i2cOk) {
    Serial.println(F("\n [✅ # SELF TEST PASS # ✅] - All I2C devices Responded OK.\n"));
    } else {
    Serial.println(F("\n [❌ # SELF TEST FAIL # ❌] - One or More I2C Devices Are Missing!\n"));
    // Optional: flash RED LED, or send special alert to Pi
    }

    selfTestPassed = i2cOk;       // true if all I2C devices OK
    selfTestCompleted = true;     // mark self-test done

    #ifdef DEBUG_STARTUP
      // Optional: send a JSON frame to the Pi
      jsonDoc.clear();
      jsonDoc["type"]           = "self_test";
      jsonDoc["passed"]         = selfTestPassed;
      jsonDoc["fwVersion"]      = FIRMWARE_VERSION;
      jsonDoc["systemMode"]     = modeToStr(systemData.systemMode);
      jsonDoc["psu1Voltage"]    = systemData.psu1Voltage;
      jsonDoc["psu2Voltage"]    = systemData.psu2Voltage;
      jsonDoc["channelCount"]   = 8;
      jsonDoc["i2cOk"]          = (Wire.endTransmission() == 0);
      jsonDoc["heapFree"]       = freeMemory();  // if you have this helper
      sendJson(jsonDoc);
    #endif

}

bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();
  return (error == 0);
}
