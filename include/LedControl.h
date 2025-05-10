#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>
#include "SystemData.h"
#include <stdint.h>
#include "Config.h"
#include <Wire.h>
#include "Pins.h"  // Include pin definitions


// Number of outputs on each TLC chip (16 outputs per chip)
#define TLC_NUM_OUTPUTS 16

extern uint8_t tlc1_cache[TLC_NUM_OUTPUTS];     // TLC1: 8 LEDs x 2 channels (red, green) = 16 outputs
extern uint8_t tlc2_cache[TLC_NUM_OUTPUTS];     // TLC2: 3 channels for tri-colour LED + 8 channels for PCB LEDs = 11 outputs

// Define RGB LED Brightness
#define RGB_BRIGHTNESS 100  // brightness value between 0 (off) and 255 (full brightness)


// Variables for TLC59116 LED Startup Animation
extern unsigned long startupAnimStartTime;      // Time when the startup animation started.
extern unsigned long lastTestUpdateTime;        // Last time the test LED was updated.
extern int nextTestLED;                         // LEDs are marked tested (start fade) in left-to-right order.
extern unsigned long ledTestFadeStart[8];       // For each front panel LED, record the time when its "tested" fade started (0 if not started).
extern bool startupAnimationDone;               // Flag to indicate if startup animation is complete.
extern bool cableConnected[8];                  // Placeholder for cable detection.
extern uint8_t currentMode;      


extern const uint8_t TARGET_RED;
extern const uint8_t TARGET_GRN;
extern const uint8_t TARGET_BLU;

extern const uint8_t FLASH_RED;
extern const uint8_t FLASH_GRN;


// Function prototypes

// TLC59116 Functions
void initTLCs();
bool setFrontPanelLED(uint8_t ledNum, uint8_t red, uint8_t green);
bool setModeSelectLED(uint8_t red, uint8_t green, uint8_t blue);
bool setPCBLED(uint8_t ledNum, uint8_t brightness);

// State machine for the startup animation:
enum StartupPhase {
    FADE_IN,
    HOLD,
    FADE_OUT,
    CHASE_CHECK,
    FINAL_WAIT
};

extern StartupPhase startupPhase;
extern unsigned long animPhaseStartTime;
extern int animCurrentChannel;
extern bool channelsFinalized[8];


// Startup Animation Functions
void startupAnimation();
void updateStartupAnimation(uint8_t mode);
uint8_t computeChaseBrightness(float effectivePhase);
void getTriColourValues(unsigned long now, uint8_t &r, uint8_t &g, uint8_t &b);
void updateTriColourLED(SystemMode mode, uint8_t brightness);


#endif // LED_CONTROL_H
