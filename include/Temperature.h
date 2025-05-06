#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <stdint.h>
#include "Pins.h"

// OneWire bus pin and sensor update interval
#define TIME_INTERVAL 5000

// Global variable (if needed)





// Function prototypes
void initTemperature();
void requestTemperature();
void controlFanSpeed();


#endif // TEMPERATURE_SENSOR_H
