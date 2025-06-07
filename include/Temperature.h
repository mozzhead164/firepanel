#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <stdint.h>
#include "Pins.h"           // Include pin definitions
#include "Config.h"         // Include configuration
#include "SystemData.h"     // Include system data





// Function prototypes
void initTemperature();
void requestTemperature();
bool ow_reset();
void controlFanSpeed();


#endif // TEMPERATURE_SENSOR_H
