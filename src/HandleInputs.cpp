
#include <Arduino.h>        // Include Arduino Library
#include <Bounce2.h>        // Include Bounce2 Library
#include <Wire.h>           // Include Wire Library
#include "Pins.h"           // Include Pin Definitions
#include "SystemData.h"     // Include System Data
#include "HandleInputs.h"   // Include Handle Inputs
#include "EventManager.h"   // Include Event Manager
#include <avr/interrupt.h>  // Include Interrupts
#include "LedControl.h"     // Include LED Control
#include <avr/pgmspace.h>



// Global Variables Definitions

SystemData systemData;


volatile bool confirmedIntFlag = false;

// Flag + Timer For Break Glass Debouncing
unsigned long bgLastInterrupt = 0;
bool bgPending = false;

// Initialize Mode Switch Inputs and human-readable names
const uint8_t MODE_PINS_P[] PROGMEM   = { MODE_ARM, MODE_TST, MODE_NOP };

// each C-string in flash
const char PROGMEM MODE_NAME_0[] = "ARMED";
const char PROGMEM MODE_NAME_1[] = "TEST";
const char PROGMEM MODE_NAME_2[] = "NO OUTPUT";

// array of pointers in flash
const char * const PROGMEM MODE_NAMES_P[] = {
  MODE_NAME_0,
  MODE_NAME_1,
  MODE_NAME_2
};



// // Forward declaration
bool setFrontPanelLED(uint8_t ledNum, uint8_t red, uint8_t green);  // Set LED color


void scanI2C() 
{
  Serial.println("\nI2C Scanner:");

  // Initialize I2C communication
  Serial.println("Initializing I2C bus at 100kHz...");
  Wire.setClock(100000UL);
  
  // Start I2C communication
  Serial.println("Starting I2C communication...");
  Wire.begin();
  
  Serial.println("Scanning I2C bus...");

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(" • Found device @ 0x");
      Serial.println(addr, HEX);
    } else if (err == 4) {
      Serial.print(" • Unknown error @ 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Scanner done.\n");
}


// Initialisation Functions



// Initialize camera inputs, break glass input, and mode switches.
void initInputs() {

  #ifdef DEBUG_STARTUP
    Serial.println("\n Initializing Inputs...\n");
  #endif

    // Initialize Camera Inputs
    for (int i = 0; i < numCameras; i++) {

      auto &ch = systemData.channels[i];
      

      uint8_t camPin = pgm_read_byte_near(cameraPins_P + i);
      ch.pin = camPin;                        // Camera Pins
      ch.state = true;                        // Initialize as open (Not Triggered)      
      ch.triggered = false;                   // Clear trigger flag
      ch.triggerTimestamp = 0;                // Clear timestamp
      ch.cableConnected = false;              // Clear cable connected state

      ch.debouncer.attach(ch.pin, INPUT);     // Attach Debouncer
      ch.debouncer.interval(5);               // 5 ms debounce interval

      ch.cameraTriggered  = false;            // Clear channel trigger
      ch.dummyTriggered   = false;            // Clear dummy trigger
      ch.thermalTriggered = false;            // Clear thermal trigger
      ch.liveOutputConfirmed = false;         // Clear live output confirmation
      ch.dummyOutputConfirmed = false;        // Clear dummy output confirmation

      ch.buttonPressed = false;               // Clear button pressed state
      ch.lastPressedTime = 0;                 // Clear last pressed time

      ch.ledRed = 0;                          // Clear LED red state
      ch.ledGreen = 0;                        // Clear LED green state
      
      ch.lastTriggerTime = 0;                 // Clear last trigger time
      ch.dummyLastTrigger = 0;                // Clear last dummy trigger time
      ch.thermalLastTrigger = 0;              // Clear last thermal trigger time
    }


    // Initialize Break Glass Input
    systemData.bgPin = INT6_BRK_GLS;
    systemData.bgState = true;
    systemData.bgDebouncer.attach(systemData.bgPin, INPUT);
    systemData.bgDebouncer.interval(5);
    systemData.bgTriggered = false;
    systemData.bgTimestamp = 0UL;



  for (uint8_t i = 0; i < 3; ++i) {
    auto &ms = systemData.modeSwitches[i];  // Mode Switches - Reference
    
    // Pin still read from a RAM array or PROGMEM similarly
    ms.pin = pgm_read_byte_near(MODE_PINS_P + i);

    // Read the pointer to the flash string out of PROGMEM:
    PGM_P p = (PGM_P)pgm_read_word_near(MODE_NAMES_P + i);
    ms.name = p;  // now points at "ARMED", "TEST" or "NO OUTPUT" in flash

    ms.state     = HIGH;                    // Initialize as open (Not Triggered)
    ms.prevState = HIGH;                    // Previous State
    ms.debouncer.attach(ms.pin, INPUT);     // Attach Debouncer
    ms.debouncer.interval(5);               // 5 ms debounce interval
  }
}

// Initialise Thermal / Dummy Outputs
void initOutputs() {

  #ifdef DEBUG_STARTUP
    Serial.println("\n Initializing Outputs...\n");
  #endif

  // Initialise Thermal Outputs
  for (int i = 0; i < numThermals; i++) {
    // First Read the Thermal Pins From PROGMEM
    uint8_t thermPin = pgm_read_byte_near(thermalPins_P + i);
    pinMode(thermPin, OUTPUT);
    digitalWrite(thermPin, LOW); // Set to LOW initially
  }

  // Initialise Dummy Outputs
  for (int i = 0; i < numDummy; i++) {
    uint8_t pin = pgm_read_byte_near(dummyPins_P + i);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); // Set to LOW initially
  }

  // Initialise Buzzer Output
  pinMode(BUZZER_OUT, OUTPUT);

  // Initialise Fan Control Output
  pinMode(FAN_CONTROL, OUTPUT);

  // Initialise TLC Reset Pin
  pinMode(LED_TLC_RST, OUTPUT);
  
}

// Initialise the PCF8574 chip
void initPCF8574(uint8_t chipAddress) {
  
  #ifdef DEBUG_EXPANDER
    Serial.print("\n Initializing PCF8574 IO Expanders...\n");
  #endif

  Wire.beginTransmission(chipAddress);
  Wire.write(0xFF);  // Set all pins high (inputs with pullups)
  uint8_t err = Wire.endTransmission();
  
    if (err != 0) {

      #ifdef DEBUG_EXPANDER
        Serial.print(" ⚠️ Error Initializing PCF8574 at Address 0x");
        Serial.print(chipAddress, HEX);
      #endif
    } else {

      #ifdef DEBUG_EXPANDER
        Serial.print(" ✅ PCF8574 Detected at Address 0x");
        Serial.print(chipAddress, HEX);
        Serial.println(" Initialized.");
      #endif
    }
}

// Initialize the PCA9555 chip
void initPCA9555(uint8_t chipAddress) {

  #ifdef DEBUG_EXPANDER
    Serial.print("\n Initializing PCA9555 IO Expanders...\n");
  #endif

    Wire.beginTransmission(chipAddress);
    Wire.write(0x06);   // config Port0
    Wire.write(0xFF);
    Wire.write(0xFF);   // config Port1
    uint8_t err = Wire.endTransmission();

  #ifdef DEBUG_EXPANDER
      if (err) {
        Serial.print(" ⚠️ Error Initialising PCA9555 at Address 0x");
        Serial.print(chipAddress, HEX);
        Serial.print(" (err=");
        Serial.print(err);
        Serial.println(")");
      } else {
        Serial.print(" ✅ PCA9555 Detected at Address 0x");
        Serial.println(chipAddress, HEX);
        Serial.println(" Initialised.");
      }
  #endif
}

// Init 3x PCF Chips with Their Addresses
void initPCFs() {
  
  #ifdef DEBUG_STARTUP
    Serial.println("\n Initializing I2C PCF8574 & PCA9555 IO Expanders...\n");
  #endif

  initPCF8574(FP_SWITCH_ADDR);
  initPCF8574(CABLE_CON_ADDR);
  initPCA9555(OUTP_SENS_ADDR);
}

// Initialize Interrupts
void initPinInterrupts() {

  #ifdef DEBUG_STARTUP
    Serial.println("\n Initializing Interrupts...\n");
  #endif

  // Set up the interrupt pins for ATmega128 or ATmega32

  // For ATmega128, we use INT4..7 for different inputs.
  // For ATmega32, we use INT0 for the PCF interrupt.

#ifdef USE_ATMEGA128
    // --------- ATmega128 Interrupt Setup ---------
    // For falling edge triggers on INT4..7:
    //   EICRB: (ISC4x=10), (ISC5x=10), (ISC6x=10), (ISC7x=10)
    EICRB = (1 << ISC41) | (1 << ISC51) | (1 << ISC61) | (1 << ISC71);

    // Enable INT4..7
    EIMSK = (1 << INT4) | (1 << INT5) | (1 << INT6) | (1 << INT7);

    pinMode(INT7_OP_CONF, INPUT_PULLUP);

    // Global interrupts on
    sei();

#else
    // --------- Legacy ATmega32 Interrupt Setup ---------
    // (Example: INT0 for PCF interrupt)
    MCUCR |= (1 << ISC01);    // falling edge
    MCUCR &= ~(1 << ISC00);   // ensure ISC00=0
    GICR  |= (1 << INT0);     // enable INT0
    sei();

#endif
}



// Initialisation Functions





// Helper Functions



// Custom Debounce Function
// This function waits until the new value has been stable for debounceDelay ms before updating.
bool debounceInput(bool currentReading, bool &lastStableState, uint32_t &lastChangeTime, uint16_t debounceDelay) {
    uint32_t now = millis();
    // If the reading has changed from the stable state:
    if (currentReading != lastStableState) {
        // If this is the first detection of the change, record the time.
        if (lastChangeTime == 0) {
            lastChangeTime = now;
        }
        // If the change has persisted for at least debounceDelay, update the stable state.
        else if (now - lastChangeTime >= debounceDelay) {
            lastStableState = currentReading;
            lastChangeTime = 0; // Reset for the next change.
        }
    } else {
        // If the reading equals the stable state, ensure the timer is reset.
        lastChangeTime = 0;
    }
    return lastStableState;
}

// Function to print the triggered message based on the system state
void printTriggeredMessage(SystemMode mode) {
  switch (mode) {
    case MODE_ARMED:
      Serial.print(" - ARMED Terminal(s) Activated.\n");
      break;
    case MODE_TEST:
      Serial.print(" - TEST / Dummy Terminal(s) Activated.\n");
      break;
    case MODE_NO_OUTPUT:
      Serial.print(" - No Output Terminal is Activated.\n");
      break;
  }
}

// Reads one byte from a PCF8574 device. Returns the 8-bit state.
uint8_t pcf8574Read(uint8_t chipAddress) {
  Wire.beginTransmission(chipAddress);
  Wire.endTransmission();
  
  Wire.requestFrom(chipAddress, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF; // In case of error, assume all pins are high (no button pressed)
}


// Interrupt Service Routines for ATmega128 and ATmega32
#ifdef USE_ATMEGA128

// ATmega128 approach
volatile bool int4Flag = false;
volatile bool int5Flag = false;
volatile bool int6Flag = false;
volatile bool int7Flag = false;
bool fpIntFlag = false;               // set by ISR, cleared in loop

static uint8_t stableState = 0xFF;    // debounced states (1 = released)
static uint8_t lastRaw = 0xFF;        // last raw byte we sampled
static uint8_t counter[8] = {0};      // per‑bit debounce counters

  // Interrupt Service Routines

  // Int4 = Front Panel Button
  ISR(INT4_vect)
  {
      int4Flag = true;
  }
  
  // Int5 = Cable Connection
  ISR(INT5_vect)
  {
      int5Flag = true;
  }
  
  // Int6 = Break Glass
  ISR(INT6_vect)
  {
      int6Flag = true;
      bgLastInterrupt = millis();
      bgPending = true;
  }
  
  // Int7 = Output Sense
  ISR(INT7_vect)
  {
      int7Flag = true;
  }



  // Handle Interrupts
  void HandleInterrupts() {

    // ATmega128: Multiple External Interrupt Lines

      // Front panel button triggered:
      if (int4Flag) 
      {
          // updateFpButtonStates(); // Read the front panel IO expander
          fpIntFlag = true;       // Set the flag to indicate front panel button event

          #ifdef DEBUG_INTERRUPT
            Serial.println("Front panel button interrupt triggered!");
          #endif

          int4Flag = false; 
      }

      // Cable connection triggered:
      if (int5Flag) 
      {
          updateCableConnectedStates();

          #ifdef DEBUG_INTERRUPT
            Serial.println("Cable connection interrupt triggered!");
          #endif

          int5Flag = false;
      }

      // Break glass triggered:
      if (int6Flag) 
      {
          updateBreakGlassInput();  // Read the break-glass IO expander

          #ifdef DEBUG_INTERRUPT
            Serial.println("Break Glass Interrupt Triggered!");
          #endif

          int6Flag = false;       // Reset the flag
      }

      // Output sense triggered:
      if (int7Flag) {

        // keep polling the expander until all bits have debounced
        updateOutputSenseStates();

        #ifdef DEBUG_INTERRUPT
          Serial.println("Output sense interrupt triggered!");
        #endif 

        // No need to reset the flag here; it will be cleared in updateOutputSenseStates()
      }
  }

// ATmega32
#else

  // PCF Interrupt Flag
  static volatile bool pcfInterruptFlag = false;

  // ATMega32 INT0 Interrupt Service Routine.
  ISR(INT0_vect) {
    pcfInterruptFlag = true;
    // (Optional debug: Serial.println(F("Front Panel Button ISR")); )
  }

  // Checks the interrupt flag and, if set, processes the button states.
  void HandleInterrupts() {
    if (pcfInterruptFlag) {
      pcfInterruptFlag = false;
      updateFpButtonStates();
      // updateCableConnectedStates();
    }
  }
#endif // USE_ATMEGA128



// Helper Functions








// GPIO Update Functions
void initModeSwitch() {

  #ifdef DEBUG_STARTUP
    Serial.println("\n Initializing Mode Switches...\n"); 
  #endif

  // Initialize Mode Switch
  
  for (uint8_t i = 0; i < 3; ++i) 
  {
    auto &ms = systemData.modeSwitches[i];

    ms.debouncer.update();

    if (ms.debouncer.read() == LOW) 
    {
      systemData.systemMode = SystemMode(i);
      char buf[12];
      strcpy_P(buf, ms.name);
      Serial.print("Mode Switch: ");
      Serial.println(buf);
    }
  }
}


// Update System State - Mode Select
void updateModeSwitch() {
  
  for (uint8_t i = 0; i < 3; ++i) 
  {
    auto &ms = systemData.modeSwitches[i];

    ms.debouncer.update();

    if (ms.debouncer.fell()) 
    {
      // optional beep
      #ifdef USE_BUZZER_OUTPUT
        Buzzer::beep(66);
      #endif

      systemData.systemMode = SystemMode(i);
      char buf[12];
      strcpy_P(buf, ms.name);

      Serial.print("Mode Switch: ");
      Serial.println(buf);
    }
  }
}


// Check Camera Input States
void updateInputs() {

  // Current Timestamp
  uint32_t now = millis();

  // Update Camera Input States
  for (int i = 0; i < numCameras; i++) 
  {
    // Reference the ChannelData struct
    auto &ch = systemData.channels[i];    

    // Update the debouncer for the current camera input.
    ch.debouncer.update();

    // Check if the input has just been pressed.
    if (ch.debouncer.fell()) {
      // Set the triggered flag to true so that processChannelStates() can act on it.
      ch.triggered = true;
      
      // Optionally update the state if you need to keep track (e.g., LOW means pressed)
      ch.state = LOW;
      
      // Also, if you want to immediately reflect this in your data struct:
      ch.cameraTriggered = true;
      ch.lastTriggerTime = now;

      // Record the timestamp of when this input was triggered.
      ch.triggerTimestamp = now;  // Record the trigger time.

      #ifdef DEBUG_TRIGGER
        Serial.print("\n[Trigger 🔥] Camera Input ");
        Serial.print(i + 1);
        Serial.print(" Triggered ");
      #endif
    }

    // Check if the Input is Still LOW (Output To Buzzer)
    if(ch.debouncer.read() == LOW) {
      #ifdef USE_BUZZER_OUTPUT
        Buzzer::beep(100); // Beep sound for 2 seconds
      #endif
    }

    // Check if the Input has just been Released.
    else if (ch.debouncer.rose()) {
      // Update the state to indicate the input is no longer pressed.
      ch.state = HIGH;
    }
  }
}


// Update Break Glass Input State
void updateBreakGlassInput() {
 
  // Check break‐glass input state
  systemData.bgDebouncer.update();

  // Current Timestamp
  uint32_t now = millis();

  // Serial.print("Checking Break Glass Input...");
  // Serial.println(systemData.bgDebouncer.read());

  // Check if the break-glass input has just been pressed.
  if (systemData.bgDebouncer.fell()) 
  {
    systemData.bgState     = LOW;
    systemData.bgTriggered = true;
    systemData.bgTimestamp = now;

    Serial.println("\n Break Glass Triggered!");
    
    // dispatch break-glass event
    Event e = {
        .type      = EVENT_BREAK_GLASS,
        .channel   = 0,        // not used
        .connected = false     // not used
    };
    dispatchEvent(&e);

    systemData.bgTriggered = false;   // reset the flag
  }

  else if (systemData.bgDebouncer.rose()) {
      systemData.bgState = HIGH;      // Break Glass Released
  }
}


// Update All Inputs
void updateAllInputs() {

  // Update Camera Inputs
  updateInputs();

  // Update Break Glass Input
  updateBreakGlassInput();

  // Update Mode Switch Inputs
  updateModeSwitch();

}


// Check For Channel Trigger Flag
void processChannelStates() 
{

  const uint32_t LATCH = DEFAULT_TRIGGER_LATCH_TIME_MS;
  unsigned long   now   = millis();

  for (int i = 0; i < numCameras; i++) 
  {
    auto &ch = systemData.channels[i];

    if (ch.triggered) 
    {
      printTriggeredMessage(systemData.systemMode);
      ch.cameraTriggered = true;
      ch.lastTriggerTime = millis();
      ch.triggered = false;

      // notify via EventManager
      Event e = {
        .type      = EVENT_CHANNEL_TRIGGER,
        .channel   = (uint8_t)i,      // zero-based; +1 for human if you like
        .connected = false   // not used for this event
      };
      dispatchEvent(&e);
    }

    // Check camera trigger timeout
    if (ch.cameraTriggered && (now - ch.lastTriggerTime >= LATCH)) {
      ch.cameraTriggered = false;
    }

    // Check thermal trigger timeout
    if (ch.thermalTriggered && (now - ch.thermalLastTrigger >= LATCH)) {
      ch.thermalTriggered = false;
      // also turn it off in hardware:
      uint8_t thermPin = pgm_read_byte_near(thermalPins_P + i);
      digitalWrite(thermPin, LOW);
    }

    if (ch.dummyTriggered && (now - ch.dummyLastTrigger  >= LATCH)) {
      ch.dummyTriggered = false;
      // also turn it off in hardware:
      uint8_t pin = pgm_read_byte_near(dummyPins_P + i);
      digitalWrite(pin, LOW);
      #ifdef DEBUG_FP
        Serial.print(F("[DEBUG_FP] Dummy pin reset for channel "));
        Serial.println(i + 1);
      #endif
    }

    // expire live-confirm after the latch interval
    if (ch.liveOutputConfirmed && (now - ch.lastTriggerTime >= LATCH))
    {
      ch.liveOutputConfirmed = false;
    }

    // expire dummy-confirm after the latch interval
    if (ch.dummyOutputConfirmed && (now - ch.lastTriggerTime >= LATCH))
    {
      ch.dummyOutputConfirmed = false;
    }  
  }
  
  // Break Glass Triggered
  if (systemData.bgTriggered) 
  { 
    printTriggeredMessage(systemData.systemMode);
    systemData.bgTriggered = false;
  }
}


// Update Triggered Channel LEDs
void updateTriggeredChannelLEDs() {
  if (!startupAnimationDone) { return; }

  unsigned long now = millis();
  bool flashOn = ((now / 150UL) % 2) == 0;
  
  for (uint8_t i = 0; i < 8; i++) {
    
    // 1) Confirmed-output flash: alternate red/green
    if (systemData.channels[i].liveOutputConfirmed ||
        systemData.channels[i].dummyOutputConfirmed) {
      if (flashOn) {
        setFrontPanelLED(i + 1, TARGET_RED,   0);
      } else {
        setFrontPanelLED(i + 1, 0,         TARGET_GRN);
      }
      continue;  // priority, so skip the rest
    }

    // 2) Camera- or thermal-trigger flash (red only)
    if (systemData.channels[i].cameraTriggered || systemData.channels[i].thermalTriggered) {
      // Flash red
      if (flashOn) {
        setFrontPanelLED(i + 1, TARGET_RED, 0);
        systemData.channels[i].ledRed = TARGET_RED;
        systemData.channels[i].ledGreen = 0;
      } else {
        setFrontPanelLED(i + 1, 0, 0);
        systemData.channels[i].ledRed = 0;
        systemData.channels[i].ledGreen = 0;
      }
    }

    // 3) Normal “cable connected” / “cable disconnected”
    else if (systemData.channels[i].cableConnected) {
      // Solid green
      setFrontPanelLED(i + 1, 0, TARGET_GRN);
      systemData.channels[i].ledRed = 0;
      systemData.channels[i].ledGreen = TARGET_GRN;
    }
    else {
      // Solid red (disconnected)
      setFrontPanelLED(i + 1, TARGET_RED, 0);
      systemData.channels[i].ledRed = TARGET_RED;
      systemData.channels[i].ledGreen = 0;
    }
  }
}


// Process Front Panel button state changes from the PCF8574 switch expander.
void updateFpButtonStates()
{
    const uint8_t SAMPLE_MS = 5;          
    const uint8_t STABLE_MS = 50;         
    const uint8_t THRESHOLD = STABLE_MS / SAMPLE_MS;
    static uint16_t lastSample = 0;

    #ifdef USE_BUZZER_OUTPUT

      // ——— non-blocking dual-beep state ———
      static bool     nonTestBeepActive = false;
      static uint32_t nonTestBeepNext   = 0;

      static const uint32_t NONTEST_BEEP_GAP_MS = 120;       // gap between tones
      static const uint32_t NONTEST_BEEP_DURATION_MS = 100;  // each tone’s length

      // — non-blocking second tone —
      if (nonTestBeepActive && millis() >= nonTestBeepNext) {
          tone(BUZZER_OUT, 1000, NONTEST_BEEP_DURATION_MS);
          nonTestBeepActive = false;
      }
    #endif

    // only sample every SAMPLE_MS or on interrupt
    if (!fpIntFlag && (millis() - lastSample < SAMPLE_MS))
        return;

    // if we’re here, we’re either on a timer or an interrupt
    lastSample = millis();
    fpIntFlag  = false;

    // read the PCF8574 chip
    uint8_t raw = pcf8574Read(FP_SWITCH_ADDR);

    // Debounce the 8 bits of the raw button state
    for (uint8_t i = 0; i < 8; ++i) {
        bool rawBit  = (raw >> i) & 0x01;       // 1 = released
        bool lastBit = (lastRaw >> i) & 0x01;

        // debounce step 1
        if (rawBit != lastBit) {
            counter[i] = 0;
        } else if (counter[i] < THRESHOLD) {
            counter[i]++;
        }

        // debounce step 2: confirmed edge?
        if (counter[i] == THRESHOLD &&
            rawBit != ((stableState >> i) & 0x01))
        {
            if (rawBit == 0) {
                Serial.print(F("Front Panel Button "));
                Serial.print(i + 1);
                Serial.println(F(" - Pressed."));

                if (systemData.systemMode == MODE_TEST) {
                    #ifdef USE_BUZZER_OUTPUT
                      // single beep + fire thermal
                      Buzzer::beep(550);
                    #endif

                    uint8_t pin = pgm_read_byte_near(dummyPins_P + i);
                    digitalWrite(pin, HIGH);
                    
                    systemData.channels[i].dummyTriggered     = true;       // track dummy
                    systemData.channels[i].dummyLastTrigger   = millis();   // timestamp dummy
                    
                    // 🔧 Immediately generate a CONFIRM event
                    Event e = {
                      .type      = EVENT_OUTPUT_CONFIRM,
                      .channel   = uint8_t(i+1),    // 1–8 for human
                      .connected = true             // dummy output
                    };
                    dispatchEvent(&e);
                    #ifdef DEBUG_FP
                      Serial.print(F("[DEBUG_FP] Dispatched dummy confirm for CH"));
                      Serial.println(i+1);
                    #endif

                } else {
                    #ifdef USE_BUZZER_OUTPUT
                      // non-blocking dual tone
                      tone(BUZZER_OUT, 2200, NONTEST_BEEP_DURATION_MS);
                      nonTestBeepActive = true;
                      nonTestBeepNext   = millis()
                                        + NONTEST_BEEP_DURATION_MS
                                        + NONTEST_BEEP_GAP_MS;
                    #endif
                }
            } else {
                Serial.print(F("Front Panel Button "));
                Serial.print(i + 1);
                Serial.println(F(" - Released."));
            }

            // update the stableState bit
            if (rawBit)
                stableState |=  (1 << i);
            else
                stableState &= ~(1 << i);
        }
    }

    // remember this raw sample for next time
    lastRaw = raw;
}


// Processes Cable Connected state changes from the PCF8574 switch expander.
// Uses the custom debounce function and prints (via Serial) on a falling edge.
void updateCableConnectedStates() {

  // one-time flag so we only do the boot summary once
  static bool cableInitDone = false;
  // remember last reported state so we don’t fire edges on first pass
  static bool lastReported[8];

  // static variables for debounce
  static bool     cableStates[8];
  static uint32_t cableLastChange[8];

  // read the PCF8574 chip
  uint8_t newState = pcf8574Read(CABLE_CON_ADDR);

  // ——— Boot-up summary & LED priming ———
  if (!cableInitDone) {
    bool rawReading[8];
    for (uint8_t i = 0; i < 8; ++i) {
      rawReading[i]      = ((newState >> i) & 0x01) != 0;
      cableStates[i]     = rawReading[i];
      cableLastChange[i] = 0;
      lastReported[i]    = rawReading[i];
    }

    Serial.println(F("=== Cable Connected Detection ==="));

    // Connected Cables list...
    Serial.print  (F("Connected Cables: "));
    bool first = true;
    for (uint8_t i = 0; i < 8; ++i) {
      if (!rawReading[i]) {
        if (!first) Serial.print(F(", "));
        Serial.print(i+1);
        first = false;
      }
    }
    if (first) Serial.print(F("None"));
    Serial.println();

    // Disconnected Cables list...
    Serial.print  (F("Disconnected Cables: "));
    first = true;
    for (uint8_t i = 0; i < 8; ++i) {
      if (rawReading[i]) {
        if (!first) Serial.print(F(", "));
        Serial.print(i+1);
        first = false;
      }
    }
    if (first) Serial.print(F("None"));
    Serial.println(F("\n"));

    // ——— NEW: Prime the front-panel LEDs to match the above state ———
    for (uint8_t i = 0; i < 8; ++i) {
        uint8_t ledNum = i + 1;
        bool  present = !rawReading[i];            // PCF8574 is active-low
        // paint the LED
        if (present) {
            setFrontPanelLED(ledNum, 0, TARGET_GRN);
        } else {
            setFrontPanelLED(ledNum, TARGET_RED, 0);
        }
        // *also* tell the systemData what we just saw
        systemData.channels[i].cableConnected = present;
    }
    // ——— end of priming block ———

    cableInitDone = true;
    return;
  }

  // ——— Normal debounced edge detection ———
  for (uint8_t i = 0; i < 8; i++) {
    // true = released/disconnected; false = pressed/connected
    bool rawReading = ((newState >> i) & 0x01) != 0;
    bool stableState = debounceInput(
      rawReading,
      cableStates[i],
      cableLastChange[i],
      50  // debounce interval
    );

    // falling edge (true→false) = Connected
    if (!stableState && lastReported[i]) {
      Serial.print(F("Debounced Cable "));
      Serial.print(i+1);
      Serial.println(F(" Connected."));
      systemData.channels[i].cableConnected = true;

      Event event{ EVENT_CABLE_CHANGE, (uint8_t)i, true };
      dispatchEvent(&event);

      lastReported[i] = false;
    }
    // rising edge (false→true) = Disconnected
    else if (stableState && !lastReported[i]) {
      Serial.print(F("Debounced Cable "));
      Serial.print(i+1);
      Serial.println(F(" Disconnected."));
      systemData.channels[i].cableConnected = false;

      Event event{ EVENT_CABLE_CHANGE, (uint8_t)i, false };
      dispatchEvent(&event);

      lastReported[i] = true;
    }
  }
}


// Check if a cable is connected for a specific channel (0-7).
bool checkCableForChannel(uint8_t channel) {
    uint8_t state = pcf8574Read(CABLE_CON_ADDR);
    bool raw = ((state >> channel) & 0x01) != 0;
    // Since active low: if raw is false, cable is connected.
    return !raw;
}

// Output Confirmation Struct
struct SenseInfo {
  uint8_t channel;  // 1–8
  bool    dummy;    // true = dummy line, false = live line
};


// Map PCA9555 P0 and P1 bits to channel numbers
// [portIndex][bitIndex]  →  IO0_x is portIndex=0, IO1_x is portIndex=1
static constexpr SenseInfo senseMap[2][8] = {
  // PCA9555 P0 bits 0–7 = IO0_0 … IO0_7
  { 
    {8,false},  // IO0_0 → LIVE_CH8  
    {4,false},  // IO0_1 → LIVE_CH4  
    {8, true},  // IO0_2 → DMY_CH8   
    {4, true},  // IO0_3 → DMY_CH4   
    {7,false},  // IO0_4 → LIVE_CH7  
    {3,false},  // IO0_5 → LIVE_CH3  
    {7, true},  // IO0_6 → DMY_CH7   
    {3, true}   // IO0_7 → DMY_CH3   
  },
  // PCA9555 P1 bits 0–7 = IO1_0 … IO1_7
  {
    {6,false},  // IO1_0 → LIVE_CH6  
    {2,false},  // IO1_1 → LIVE_CH2  
    {6, true},  // IO1_2 → DMY_CH6   
    {2, true},  // IO1_3 → DMY_CH2   
    {5,false},  // IO1_4 → LIVE_CH5  
    {1,false},  // IO1_5 → LIVE_CH1  
    {5, true},  // IO1_6 → DMY_CH5   
    {1, true}   // IO1_7 → DMY_CH1   
  }
};


// Update Channel Output Sensing Inputs IO Expander
void updateOutputSenseStates() {

  static bool     outputSenseState[16]      = { false };
  static uint32_t outputSenseLastChange[16] = {   0   };
  static bool     prevStable[16]            = { false };
  static bool     firedThisHigh[16]         = { false };


  // read both ports
  Wire.beginTransmission(OUTP_SENS_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)OUTP_SENS_ADDR, (uint8_t)2);

  uint8_t p0 = Wire.read();
  uint8_t p1 = Wire.read();

  // for each of the 16 bits…
  for (uint8_t bit = 0; bit < 16; ++bit) {
    bool raw = (bit < 8)
      ? !((p0 >> bit) & 1)
      : !((p1 >> (bit - 8)) & 1);

    // debounceInput updates lastStableState internally,
    // but returns the _new_ stable reading (true/false).
    bool stable = debounceInput(
      raw,
      outputSenseState[bit],       // holds lastStableState
      outputSenseLastChange[bit],  // debounce timer
      50                           // ms debounce window
    );

    // detect edges cleanly:
    if (stable && !prevStable[bit]) {
      // rising edge!
      const auto &info = senseMap[bit/8][bit%8];

      // dispatch exactly once for this high
      if (!firedThisHigh[bit]) {
        firedThisHigh[bit] = true;

        Event e = {
          .type      = EVENT_OUTPUT_CONFIRM,
          .channel   = info.channel,
          .connected = info.dummy
        };
        dispatchEvent(&e);
        
        #ifdef DEBUG_OUTPUT_SENSE
          Serial.print("\n[Confirm ✅] Output Channel ");
          Serial.print(info.channel);
          Serial.print(info.dummy ? " (dummy)" : " (live)");
          Serial.println(" Confirmed!");
        #endif
      }
    }
    else if (!stable && prevStable[bit]) {
      // falling edge: reset for next rising
      firedThisHigh[bit] = false;
    }

    // remember for next loop
    prevStable[bit] = stable;
  }


  // ——— once all debounce timers are zero, clear pending flag ———
  bool anyStillBouncing = false;
  for (uint8_t i = 0; i < 16; ++i) 
  {
      if (outputSenseLastChange[i] != 0) 
      {
          anyStillBouncing = true;
          break;
      }
  }
  if (!anyStillBouncing) {
      int7Flag = false;  // reset the interrupt flag
  }

}  // updateOutputSenseStates(); // Read and debounce the PCA9555 bits


// GPIO Update Functions