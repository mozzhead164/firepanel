
#include <Arduino.h>
#include "Buzzer.h"
#include "Pins.h"  // Include pin definitions

namespace {
    bool          active     = false;
    unsigned long offAt      = 0;
}

void Buzzer::init()
{
    pinMode(BUZZER_OUT, OUTPUT);
    digitalWrite(BUZZER_OUT, LOW);     // idle state (adapt if active‑low)
}

void Buzzer::beep(uint16_t ms)
{
    digitalWrite(BUZZER_OUT, HIGH);    // turn buzzer ON
    active = true;
    offAt  = millis() + ms;            // schedule off‑time
}

void Buzzer::update()
{
    if (active && ( (long)(millis() - offAt) >= 0 )) {
        digitalWrite(BUZZER_OUT, LOW); // turn buzzer OFF
        active = false;
    }
}

bool Buzzer::busy()
{
    return active;
}