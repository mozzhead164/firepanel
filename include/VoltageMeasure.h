// src/VoltageMeasure.h
#ifndef VOLTAGEMEASURE_H
#define VOLTAGEMEASURE_H

#include <Arduino.h>
#include "Pins.h"



extern float psu1_voltage;      // PSU 1 Voltage
extern float psu2_voltage;      // PSU 2 Voltage
extern float prevPsu1Voltage;   // Previous PSU 1 Voltage
extern float prevPsu2Voltage;   // Previous PSU 2 Voltage

void read_psuVoltages();        // Read PSU Voltages


#endif // VOLTAGEMEASURE_H