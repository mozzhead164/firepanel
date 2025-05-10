
#include <Arduino.h>
#include "SystemData.h"
#include "EventManager.h"
#include "VoltageMeasure.h"





void read_psuVoltages() 
{
  // Get Current Time Stamp
  uint32_t timeNow = millis();

  static float psu1_voltage = 0.0, prevPsu1Voltage = 0.0;
  static float psu2_voltage = 0.0, prevPsu2Voltage = 0.0;

  static bool underVoltage_1 = false;
  static bool underVoltage_2 = false;

  static uint32_t lastVoltage = 0;


  // Periodically Read PSU Voltages
  if(timeNow - lastVoltage >= 250UL)
  {
    // Take Raw Reading - PSU 1
    psu1_voltage = analogRead(ADC0);
    psu1_voltage = (psu1_voltage / 1024.0) * 16.8;

    // Take Raw Reading - PSU 2
    psu2_voltage = analogRead(ADC1);
    psu2_voltage = (psu2_voltage / 1024.0) * 16.8;

    // Check For Under Voltage on PSU 1
    if(psu1_voltage <= 1.5 && !underVoltage_1) 
    {
      Serial.println("\n\n PSU 1 - UNDER VOLTAGE ALERT!!!");
      underVoltage_1 = true;
      
      // dispatch undervoltage event for PSU1
      Event e{ EVENT_PSU_UNDERVOLTAGE, 1, false };
      dispatchEvent(&e);

    }
    
    // Check For Voltage Restored on PSU 1
    else if(underVoltage_1 == 1 && psu1_voltage >= 10.5) 
    { 
      underVoltage_1 = false; 
      delay(1000);
      Serial.println("\n PSU 1 - Voltage Restored"); 
      Event e{ EVENT_PSU_RESTORED, 1, false };
      dispatchEvent(&e);

    }
    



    // Check For Under Voltage on PSU 2
    if(psu2_voltage <= 5.0 && !underVoltage_2) 
    {
      Serial.println("\n\n PSU 2 - UNDER VOLTAGE ALERT!!!");
      underVoltage_2 = true;

      // dispatch undervoltage event for PSU2
      Event e{ EVENT_PSU_UNDERVOLTAGE, 2, false };
      dispatchEvent(&e);
    }

    // Check For Voltage Restored on PSU 2
    if(underVoltage_2 == 1 && psu2_voltage >= 10.0) 
    { 
      underVoltage_2 = false; 
      Serial.println("\n PSU 2 - Voltage Restored");
      Event e{ EVENT_PSU_RESTORED, 2, false };
      dispatchEvent(&e);
    }

    // Timestamp For Last Voltage Reading
    lastVoltage = timeNow;

    systemData.psu1Voltage = psu1_voltage;
    systemData.psu2Voltage = psu2_voltage;
  }

  #ifdef DEBUG_VOLTAGE
    // Periodically Print Voltage Readings
    if(timeNow - lastPrint >= 9500L)
    {
        Serial.print("\n # PSU Voltages # - ");
        Serial.print(" PSU 1: ");
        Serial.print(psu1_voltage);
        Serial.print("v   ");
        Serial.print("PSU 2: ");
        Serial.print(psu2_voltage);
        Serial.println("v");
            
        // Timestamp For Last Print
        lastPrint = timeNow;
    }
  #endif

}
