// handleInputs.h
#ifndef HANDLE_INPUTS_H
#define HANDLE_INPUTS_H

#include <stdint.h>
#include <avr/pgmspace.h>

#include <Bounce2.h>
#include "Pins.h"       // Include pin definitions
#include "Buzzer.h"     // Include Buzzer Control



// Time in milliseconds that a triggered event (camera or thermal) stays active
#define DEFAULT_TRIGGER_LATCH_TIME_MS 25 * 1000UL



// -----------------------------------------------------------------
// Global Data Declarations

// Array of camera pins and number of cameras
extern const uint8_t cameraPins_P[] PROGMEM;
extern const uint8_t thermalPins_P[] PROGMEM;
extern const uint8_t dummyPins_P[] PROGMEM;

extern const int numCameras;




// Map each PCA9555 bit (0–15) to a channel (1–8)…
extern const uint8_t  outputSenseChannel[16];
// …and whether that bit is the dummy line (true) or live line (false)
extern const bool     outputSenseIsDummy[16];



// I2C addresses for the two PCF8574 chips (we use 0x22 for the switches)
#define FP_SWITCH_ADDR 0x22  // PCF Chip Address for Front Panel Switches
#define CABLE_CON_ADDR 0x23  // PCF Chip Address for Cable Connection Status
#define OUTP_SENS_ADDR 0x24  // PCF Chip Address for Output Sensing

#ifdef USE_ATMEGA128

extern volatile bool int4Flag;
extern volatile bool int5Flag;
extern volatile bool int6Flag;
extern volatile bool int7Flag;
extern bool fpIntFlag;

#else

// Global volatile flag set in the INT0 ISR.
extern volatile bool pcfInterruptFlag;

#endif // USE_ATMEGA128
          // true means not pressed (pullup)



// -----------------------------------------------------------------
// -------------         Function Prototypes        ----------------


// Initialize the TLC59116 chips
void initPCFs();

// Initialize the PCF8574 chips
void initPCF8574(uint8_t chipAddress);

// Initialize the PCA9555 chips
void initPCA9555(uint8_t chipAddress);

// Initialize the PCF8574 interrupt
void initPinInterrupts();

// Read the state of the PCF8574 chip
extern uint8_t pcf8574Read(uint8_t chipAddress);

// Initialize all camera and mode switch inputs
void initInputs();

// Initialise the Mode Switch on Startup
void initModeSwitch();

// Initialise Output Pins
void initOutputs();

// Print a triggered message based on system state
void printTriggeredMessage(SystemMode mode);

// Update the current system state based on mode switch inputs
void updateModeSwitch();

// Update the Camera and Break Glass Inputs
void updateInputs();

// Update Break Glass Input
void updateBreakGlassInput();

// Check for channel trigger flags (for camera and break glass inputs)
void processChannelStates();

// Check for camera inputs and update their LED states
void updateTriggeredChannelLEDs();

// Check Cable Connection Status
void checkCableConnected();

// Check if specific channel cable is connected
bool checkCableForChannel(uint8_t channel);

// Processes Front Panel button state changes
void updateFpButtonStates();

// Cleanup function for the dummy output
void dummyCleanup();

// Processes Output Sense state changes
void updateOutputSenseStates();

// Update the cable connected states
void updateCableConnectedStates();

// Process the PCF8574 interrupt flag
void HandleInterrupts();

// Custom debounce function: compares currentReading with lastStableState and updates it if the new value has been stable
bool debounceInput(bool currentReading, bool &lastStableState, uint32_t &lastChangeTime, uint16_t debounceDelay = 50);


#endif // HANDLE_INPUTS_H
