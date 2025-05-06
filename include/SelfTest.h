#ifndef SELF_TEST_H
#define SELF_TEST_H

#include <stdint.h>

extern bool selfTestPassed;
extern bool selfTestCompleted;


void runSystemSelfTest();
bool checkI2CDevice(uint8_t address);

#endif // SELF_TEST_H
