#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>

// System connection / operating states
enum SystemConnectionState {
  STATE_BOOTING,
  STATE_WAITING_FOR_PI,
  STATE_RUNNING,
  STATE_PI_OFFLINE
};

// Extern declaration (global instance lives in main.cpp)
extern SystemConnectionState systemConnectionState;

#endif // SYSTEM_STATE_H
