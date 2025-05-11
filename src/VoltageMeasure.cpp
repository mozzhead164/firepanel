
#include <Arduino.h>
#include "SystemData.h"
#include "EventManager.h"
#include "VoltageMeasure.h"
#include "Config.h"




void read_psuVoltages() 
{
  // Get Current Time Stamp
  uint32_t timeNow = millis();

  static float psu1_voltage = 0.0;
  static float psu2_voltage = 0.0;

  static bool underVoltage_1 = false;
  static bool underVoltage_2 = false;

  static uint8_t  psu1_restore_count  = 0;
  static uint8_t  psu2_restore_count  = 0;

  static uint32_t lastVoltage = 0UL;


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
    if(psu1_voltage <= VOLTAGE_UNDER && !underVoltage_1) 
    {
      Serial.println("\n\n PSU 1 - UNDER VOLTAGE ALERT!!!");
      underVoltage_1 = true;

      systemData.psu1UnderVolt = true;    // update the flag
      systemData.psu1Voltage   = psu1_voltage;  // add this member too
      
      // dispatch undervoltage event for PSU1
      Event e{ EVENT_PSU_UNDERVOLTAGE, 1, false };
      dispatchEvent(&e);

      psu1_restore_count = 0;  // reset restore counter
    }
    
    // Check For Voltage Restored on PSU 1
    else if(underVoltage_1) 
    { 
      // re-read the voltage right now:
      int raw = analogRead(ADC0);
      float voltage = raw / 1024.0 * PSU1_VOLTAGE_SCALE; 

      if (voltage >= VOLTAGE_RESTORE) 
      {
          if (++psu1_restore_count >= 2) 
          {
            // OK, sustained high voltage
            underVoltage_1     = false;
            psu1_restore_count = 0;

            Serial.print("\n PSU 1 - Voltage Restored @ ");
            Serial.print(voltage, 2);
            Serial.println(" V"); 

            // stash voltage in systemData for handler to grab
            raw = analogRead(ADC0);
            voltage = raw / 1024.0 * PSU1_VOLTAGE_SCALE; 
            systemData.psu1UnderVolt = false;    // update the flag
            systemData.psu1Voltage   = voltage;  // add this member too

            Event e{ EVENT_PSU_RESTORED, 1, false };
            dispatchEvent(&e);
          }
      } else {
        // voltage fell back below threshold, reset counter
        psu1_restore_count = 0;
      }
    }
    



    // Check For Under Voltage on PSU 2
    if(psu2_voltage <= VOLTAGE_UNDER && !underVoltage_2) 
    {
      Serial.println("\n\n PSU 2 - UNDER VOLTAGE ALERT!!!");
      underVoltage_2 = true;

      systemData.psu2UnderVolt = true;    // update the flag
      systemData.psu2Voltage   = psu2_voltage;  // add this member too

      // dispatch undervoltage event for PSU2
      Event e{ EVENT_PSU_UNDERVOLTAGE, 2, false };
      dispatchEvent(&e);

      psu2_restore_count = 0;  // reset restore counter
    }

    // Check For Voltage Restored on PSU 2
    else if(underVoltage_2) 
    { 
      // re-read the voltage right now:
      int raw = analogRead(ADC1);
      float voltage = raw / 1024.0 * PSU2_VOLTAGE_SCALE;

      if (voltage >= VOLTAGE_RESTORE) 
      {
          if (++psu2_restore_count >= 2) 
          {
            // OK, sustained high voltage
            underVoltage_2     = false;
            psu2_restore_count = 0;

            Serial.print("\n PSU 2 - Voltage Restored @ ");
            Serial.print(voltage, 2);
            Serial.println(" V"); 

            // stash voltage in systemData for handler to grab
            raw = analogRead(ADC1);
            voltage = raw / 1024.0 * PSU2_VOLTAGE_SCALE;
            systemData.psu2UnderVolt = false;    // update the flag
            systemData.psu2Voltage   = voltage; 

            Event e{ EVENT_PSU_RESTORED, 2, false };
            dispatchEvent(&e);
          }
      } else {
        // voltage fell back below threshold, reset counter
        psu2_restore_count = 0;
      }
    }

    // Timestamp For Last Voltage Reading
    lastVoltage = timeNow;

    systemData.psu1Voltage = psu1_voltage;
    systemData.psu2Voltage = psu2_voltage;
  }


  #ifdef DEBUG_VOLTAGE
    static uint32_t lastPrint = 0;
    // Periodically Print Voltage Readings
    if(timeNow - lastPrint >= 5000L)
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
