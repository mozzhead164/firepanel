#include <Arduino.h>
#include "SelfTest.h"
#include "LedControl.h"
#include "SerialComms.h"
#include "SystemData.h"
#include <Arduino.h>
#include <Wire.h>


bool selfTestPassed = false;
bool selfTestCompleted = false;


void runSystemSelfTest() {
    // Initialisation System Self-Test
    Serial.println(F("[INFO] Checking I2C devices..."));
    bool i2cOk = true;

    // Check TLC1 (assume 0x60)
    if (!checkI2CDevice(0x60)) {
    Serial.println(F("[FAIL] TLC59116 (0x60) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F("[PASS] TLC59116 (0x60) OK"));
    }

    // Check TLC2 (assume 0x61 if you have a second one)
    if (!checkI2CDevice(0x61)) {
    Serial.println(F("[FAIL] TLC59116 (0x61) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F("[PASS] TLC59116 (0x61) OK"));
    }

    // Check PCF8574 (Front Panel Switches) 0x22
    if (!checkI2CDevice(0x22)) {
    Serial.println(F("[FAIL] PCF8574 (0x22) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F("[PASS] PCF8574 (0x22) OK"));
    }

    // Check PCF8574 (Cable Connectors) 0x23
    if (!checkI2CDevice(0x23)) {
    Serial.println(F("[FAIL] PCF8574 (0x23) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F("[PASS] PCF8574 (0x23) OK"));
    }

    // Check PCA9555 (Output Sensing) 0x24
    if (!checkI2CDevice(0x24)) {
    Serial.println(F("[FAIL] PCA9555 (0x24) not responding!"));
    i2cOk = false;
    } else {
    Serial.println(F("[PASS] PCA9555 (0x24) OK"));
    }

    // Final Summary
    if (i2cOk) {
    Serial.println(F("[PASS] All I2C devices OK."));
    } else {
    Serial.println(F("[FAIL] One or more I2C devices missing!"));
    // Optional: flash RED LED, or send special alert to Pi
    }

    selfTestPassed = i2cOk;       // true if all I2C devices OK
    selfTestCompleted = true;     // mark self-test done

}

bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();
  return (error == 0);
}
