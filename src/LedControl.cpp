#include "SystemState.h"
#include "SystemData.h"
#include "LedControl.h"


// TLC59116 I2C addresses
#define TLC1_ADDR 0x60  // Front panel bi-colour LEDs
#define TLC2_ADDR 0x61  // Mode select tri-colour LED + PCB LEDs

// Base register for PWM registers (datasheet: first PWM register is 0x02)
#define PWM_BASE 0x02


// Caches for current brightness values. For TLC1, we have 16 channels (8 LEDs x 2 channels).
// For TLC2, we have 11 channels: 3 for the tri‑colour LED (channels 0,1,2) and 8 for PCB LEDs (channels 3–10).
uint8_t tlc1_cache[TLC_NUM_OUTPUTS] = {0};
uint8_t tlc2_cache[TLC_NUM_OUTPUTS] = {0};  // We'll only use channels 0–10 on TLC2


// Variables for TLC59116 LED Startup Animation

// Time when the startup animation started.
unsigned long startupAnimStartTime = 0;      

// Last time the test LED was updated.
unsigned long lastTestUpdateTime = 0;        

// LEDs are marked tested (start fade) in left-to-right order.
int nextTestLED = 0;

// For each front panel LED, record the time when its "tested" fade started (0 if not started).
unsigned long ledTestFadeStart[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// Placeholder for cable detection.
// In the final implementation, this array (or a function call) will be updated with the
// actual status from the PCF8674 I2C expander (active low).
bool cableConnected[8] = { false, true, false, false, false, false, false, false };

// Flag to control chase direction for untested LEDs:
// true  => chase flows from left to right,
// false => chase flows from right to left.
bool chaseLeftToRight = true;

// Flag to indicate if the startup animation is complete.
bool startupAnimationDone;


// Constants for the chase waveform.
const float lowerBound = 0.35;     // Fraction of the cycle where the LED starts turning on.
const float upperBound = 0.75;     // Fraction where it turns off again.
const float chasePeriod = 1500.0;  // Full period of one chase cycle (ms).

// Fade duration for the tested LED
const unsigned long testedFadeDuration = 1000;  // 1000 ms total fade per LED








StartupPhase startupPhase = FADE_IN;
unsigned long animPhaseStartTime = 0;
int animCurrentChannel = 0;
bool channelsFinalized[8] = { false, false, false, false, false, false, false, false };

// Cache Mode LED Values for Optimising I2C Writes
static uint8_t lastModeLedR = 255;
static uint8_t lastModeLedG = 255;
static uint8_t lastModeLedB = 255;


bool checkCableForChannel(uint8_t channel);



// Helper function: writes a single PWM value to a TLC59116 channel.
bool writePWM(uint8_t chipAddr, uint8_t channel, uint8_t value) 
{
  Wire.beginTransmission(chipAddr);
  Wire.write(PWM_BASE + channel);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print(" I2C write error on chip 0x");
    Serial.print(chipAddr, HEX);
    Serial.print(" channel ");
    Serial.println(channel);
    return false;
  }
  return true;
}

// Front Panel LED helper.
// LED numbering: 1..8. Mapping for TLC1: LED 1: red at index 0, green at index 1; LED 2: red at index 2, green at index 3; etc.
bool setFrontPanelLED(uint8_t ledNum, uint8_t red, uint8_t green) 
{
  if (ledNum < 1 || ledNum > 8) {
    /*Serial.println(" Invalid front panel LED number.");*/
    return false;
  } else { /*Serial.println(" Valid front panel LED number.");*/ }
  
  uint8_t baseChannel = (ledNum - 1) * 2;  // Red is at baseChannel; green is at baseChannel+1.
  bool success = true;
  
  // Update only if the new value differs from the cached value.
  if (tlc1_cache[baseChannel] != red) {
    if (writePWM(TLC1_ADDR, baseChannel, red)) {
      tlc1_cache[baseChannel] = red;
    } else {
      success = false;
    }
  }
  if (tlc1_cache[baseChannel + 1] != green) {
    if (writePWM(TLC1_ADDR, baseChannel + 1, green)) {
      tlc1_cache[baseChannel + 1] = green;
    } else {
      success = false;
    }
  }
  return success;
}

// Mode Select LED helper for the tri‑colour LED on TLC2 (common anode).
// Mapping: red at channel 0, green at channel 1, blue at channel 2.
bool setModeSelectLED(uint8_t red, uint8_t green, uint8_t blue) 
{
  bool success = true;
  
  if (tlc2_cache[0] != red) {
    if (writePWM(TLC2_ADDR, 0, red)) {
      tlc2_cache[0] = red;
    } else {
      success = false;
    }
  }
  if (tlc2_cache[1] != green) {
    if (writePWM(TLC2_ADDR, 1, green)) {
      tlc2_cache[1] = green;
    } else {
      success = false;
    }
  }
  if (tlc2_cache[2] != blue) {
    if (writePWM(TLC2_ADDR, 2, blue)) {
      tlc2_cache[2] = blue;
    } else {
      success = false;
    }
  }
  return success;
}

// PCB LED helper for the 8 single-colour LEDs on TLC2.
// Mapping for TLC2 PCB LEDs: LED 1 on channel 3, LED 2 on channel 4, …, LED 8 on channel 10.
bool setPCBLED(uint8_t ledNum, uint8_t brightness) 
{
  if (ledNum < 1 || ledNum > 8) {
    Serial.println("Invalid PCB LED number.");
    return false;
  }
  uint8_t channel = 7 + ledNum;  // LED1 -> channel 3, LED2 -> channel 4, etc.
  if (tlc2_cache[channel] != brightness) {
    if (writePWM(TLC2_ADDR, channel, brightness)) {
      tlc2_cache[channel] = brightness;
      return true;
    } else {
      return false;
    }
  }
  return true;  // No change, so consider it a success.
}


void updateModeLEDIfChanged(uint8_t r, uint8_t g, uint8_t b) {
  if (r != lastModeLedR || g != lastModeLedG || b != lastModeLedB) {
    setModeSelectLED(r, g, b);
    lastModeLedR = r;
    lastModeLedG = g;
    lastModeLedB = b;
  }
}


// Update the tri‑colour LED based on the current mode.
// mode: 0 = ARMED, 1 = TEST, 2 = NO_OUTPUT.
void updateTriColourLED(SystemMode mode, uint8_t brightness) 
{
    if (!startupAnimationDone) 
    {
        // Startup - Slow Pulse Blue 
        uint32_t now = millis();

        // Choose a pulse period (e.g., 3000 ms for a 3-second cycle)
        const float pulsePeriod = 3000.0;
        // Compute phase value between 0 and 1:
        float phase = fmod(now, pulsePeriod) / pulsePeriod;
        // Use a sine function to generate a smooth pulse between 0 and 1.
        // The sine value ranges from -1 to +1; shift and scale it to 0..1:
        float pulseFactor = (1.0 + sin(phase * 2 * PI - PI/2)) / 2.0; 
        // pulseFactor will go from 0 (minimum) up to 1 (maximum).
        // Multiply by baseBrightness for the blue channel.
        uint8_t blueIntensity = (uint8_t)(pulseFactor * RGB_BRIGHTNESS);
        // Set the LED to blue only.
        updateModeLEDIfChanged(0, 0, blueIntensity);
    } else {
        // --- Post Startup: Set static color based on mode ---
        uint8_t r, g, b;
        // Mode mapping: 0: ARMED (Red), 1: TEST (Blue), 2: NO_OUTPUT (Green)
        switch (mode) {
            case MODE_ARMED:
                r = TARGET_RED; g = 0; b = 0;   // Red for ARMED
                break;
            case MODE_TEST:
                r = 0; g = TARGET_GRN; b = 0;   // Green for TEST
                break;
            case MODE_NO_OUTPUT:
                r = 0; g = 0; b = TARGET_BLU;   // Blue for NO_OUTPUT
                break;
            default:
                r = 0; g = 0; b = 0;
                break;
        }
        // Scale the static color by the brightness parameter.
        r = (r * brightness) / 255;
        g = (g * brightness) / 255;
        b = (b * brightness) / 255;
        updateModeLEDIfChanged(r, g, b);
    }
}



// --- Startup Functions ---

// Initializes one TLC59116 chip with basic configuration and sets LEDOUT registers to PWM mode.
void initTLC(uint8_t chipAddr) 
{
  uint8_t err;
  
  #ifdef DEBUG_LED_EXPANDER
    Serial.println("\n\n Initializing TLC59116 Expanders");
  #endif
  
  // MODE1 register: normal mode, auto-increment as desired (0x00 for now).
  Wire.beginTransmission(chipAddr);
  Wire.write(0x00);  // MODE1 register address.
  Wire.write(0x00);
  err = Wire.endTransmission();

  #ifdef DEBUG_LED_EXPANDER
  if (err != 0) {
    Serial.print(" Error initializing MODE1 on chip 0x");
    Serial.println(chipAddr, HEX);
  } else {
    Serial.print(" ✅ TLC59116 Detected at Address 0x");
    Serial.print(chipAddr, HEX);
    Serial.println(" Initialized.");
  }
  #endif
  
  // MODE2 register: for TLC2 (0x61), set OUTDRV bit (0x04) to drive outputs properly.
  uint8_t mode2Val = 0x00;
  if (chipAddr == TLC2_ADDR) { mode2Val = 0x04; }
  Wire.beginTransmission(chipAddr);
  Wire.write(0x01);  // MODE2 register address.
  Wire.write(mode2Val);
  err = Wire.endTransmission();
  if (err != 0) {
    Serial.print(" Error initializing MODE2 on chip 0x");
    Serial.println(chipAddr, HEX);
  }
  
  // Set LEDOUT registers (4 registers starting at 0x14) to PWM mode.
  // Each 2-bit field set to "10" (binary) yields 0xAA for each register.
  for (uint8_t i = 0; i < 4; i++) {
    Wire.beginTransmission(chipAddr);
    Wire.write(0x14 + i);  // LEDOUT registers: 0x14, 0x15, 0x16, 0x17.
    Wire.write(0xAA);      // 0xAA = 10101010 in binary.
    err = Wire.endTransmission();
    if (err != 0) {
      Serial.print(" Error setting LEDOUT on chip 0x");
      Serial.println(chipAddr, HEX);
    }
  }
  
  // Initialize all PWM registers to 0 (turn all outputs off).
  for (uint8_t ch = 0; ch < TLC_NUM_OUTPUTS; ch++) {
    writePWM(chipAddr, ch, 0);
  }
}

// Call this in setup() to initialize both TLC59116 chips.
void initTLCs() 
{
  Wire.begin();
  Wire.setClock(100000UL); // Set I2C clock speed to 100kHz

  pinMode(LED_TLC_RST, OUTPUT);     // Set TLC Reset Pin as Output
  digitalWrite(LED_TLC_RST, LOW);   // Set TLC Reset Pin Low
  delay(500);                       // Wait for 500ms
  digitalWrite(LED_TLC_RST, HIGH);  // Set TLC Reset Pin High
  delay(500);                       // Wait for 500ms

  initTLC(TLC1_ADDR);  // Initialize TLC1 (front panel LEDs)
  initTLC(TLC2_ADDR);  // Initialize TLC2 (mode select LED + PCB LEDs)

  // Immediately set all outputs to 0 after initialization.
  for (uint8_t ch = 0; ch < TLC_NUM_OUTPUTS; ch++) {
      writePWM(TLC1_ADDR, ch, 0);
  }
  for (uint8_t ch = 0; ch < TLC_NUM_OUTPUTS; ch++) {
      writePWM(TLC2_ADDR, ch, 0);
  }

}

// --- Yellow Chase Brightness Function ---
// Uses cosine interpolation between lowerBound and upperBound for a smooth pulse.
uint8_t computeChaseBrightness(float effectivePhase) {
  if (effectivePhase < lowerBound || effectivePhase > upperBound) {
    return 0;
  }
  float normalized = (effectivePhase - lowerBound) / (upperBound - lowerBound); // 0 to 1
  // Cosine interpolation: 0 at 0, peaks (1) at 0.5, then 0 at 1.
  float brightnessFactor = (1 - cos(normalized * PI)) / 2;
  return (uint8_t)(brightnessFactor * 255);
}


// Call this in setup() after initTLCs() to begin the startup animation
void startupAnimation() {
  startupAnimStartTime = millis();
  lastTestUpdateTime = startupAnimStartTime;
  nextTestLED = 0;
  for (int i = 0; i < 8; i++) {
    ledTestFadeStart[i] = 0;
  }
  startupAnimationDone = false;

  // Reset the state-machine globals
  startupPhase = FADE_IN;
  animPhaseStartTime = millis();
  animCurrentChannel = 0;
  for (int i = 0; i < 8; i++) {
      channelsFinalized[i] = false;
  }
}

void updateStartupTriColourLED() {
  static const float pulsePeriod = 3000.0; // ms for full sine cycle
  unsigned long now = millis();
  float phase = fmod(now, pulsePeriod) / pulsePeriod;
  float pulseFactor = (1.0 + sin(phase * 2 * PI - PI/2)) / 2.0;
  uint8_t brightness = (uint8_t)(pulseFactor * RGB_BRIGHTNESS);

  uint8_t r = 0, g = 0, b = 0;

  switch (systemConnectionState) {
    case STATE_WAITING_FOR_PI:
      r = 0;
      g = 0;
      b = brightness;  // Blue breathing
      break;

    case STATE_RUNNING:
      r = 0;
      g = brightness;  // Green breathing
      b = 0;
      break;

    case STATE_PI_OFFLINE:
      r = brightness;  // Red breathing
      g = 0;
      b = 0;
      break;

    default:
      r = 0;
      g = 0;
      b = brightness;  // Default to blue
      break;
  }

  updateModeLEDIfChanged(r, g, b);
}


// --- Main Update Function for Startup Animation ---
// This function should be called repeatedly (from loop()) until startupAnimationDone becomes true.
// LedControl.cpp
void updateStartupAnimation(uint8_t mode) {
    // Timing constants (in milliseconds)
    static const unsigned long FADE_IN_DURATION    = 1000;  // X sec fade in
    static const unsigned long HOLD_DURATION       = 2000;   // X sec hold at 60% red
    static const unsigned long FADE_OUT_DURATION   = 1500;  // X sec fade out
    static const unsigned long INITIAL_CHASE_DELAY = 3500;  // X seconds pure chase at start.
    static const unsigned long CHECK_DURATION      = 2000;  // 1.5 sec allocated per channel check
    static const unsigned long FINAL_CHASE_WAIT    = 330;   // Final wait before finishing startup
    

    unsigned long now = millis();

    if (startupPhase == FADE_IN || startupPhase == HOLD || startupPhase == FADE_OUT) {
      updateStartupTriColourLED();
    }

    switch (startupPhase) 
    {
        case FADE_IN: 
        {
            float progress = (now - animPhaseStartTime) / (float)FADE_IN_DURATION;
            if (progress > 1.0) progress = 1.0;
            uint8_t redVal = (uint8_t)(TARGET_RED * progress);
            for (uint8_t i = 0; i < 8; i++) {
                setFrontPanelLED(i + 1, redVal, 0);
            }
            if (progress >= 1.0) {
                startupPhase = HOLD;
                animPhaseStartTime = now;
            }
            break;
        }
        case HOLD: {
            for (uint8_t i = 0; i < 8; i++) {
                setFrontPanelLED(i + 1, TARGET_RED, 0);
            }
            if (now - animPhaseStartTime >= HOLD_DURATION) {
                startupPhase = FADE_OUT;
                animPhaseStartTime = now;
            }
            break;
        }
        case FADE_OUT: {
            float progress = (now - animPhaseStartTime) / (float)FADE_OUT_DURATION;
            if (progress > 1.0) progress = 1.0;
            uint8_t redVal = (uint8_t)(TARGET_RED * (1.0 - progress));
            for (uint8_t i = 0; i < 8; i++) {
                setFrontPanelLED(i + 1, redVal, 0);
            }
            if (progress >= 1.0) {
                startupPhase = CHASE_CHECK;
                animPhaseStartTime = now;  // Reset timer for CHASE_CHECK phase.
                animCurrentChannel = 0;           // Start from first channel.
            }
            break;
        }
      
      
        case CHASE_CHECK: {

        static unsigned long chaseStartTime = 0;
        // channelCheckStartTime is reset with each channel finalization.
        static unsigned long channelCheckStartTime = 0;
        // On the first call into CHASE_CHECK, initialize the timers.
        if (chaseStartTime == 0) {
            chaseStartTime = now;
            channelCheckStartTime = now;
        }

        // Calculate the overall elapsed time since entering CHASE_CHECK.
        unsigned long elapsedSinceState = now - animPhaseStartTime;

        // For the first INITIAL_CHASE_DELAY milliseconds, run the chase animation on all channels.
        if (elapsedSinceState < INITIAL_CHASE_DELAY) {
            float globalPhase = fmod((now - chaseStartTime), chasePeriod) / chasePeriod;
            for (uint8_t i = 0; i < 8; i++) {
                // Display chase animation on every channel.
                float offset = chaseLeftToRight ? ((float)(7 - i) / 8.0) : ((float)i / 8.0);
                float effectivePhase = fmod(globalPhase + offset, 1.0);
                uint8_t brightness = computeChaseBrightness(effectivePhase);
                uint8_t chaseRed = (uint8_t)((brightness * CHASE_COLOR_R) / 255);
                uint8_t chaseGreen = (uint8_t)((brightness * CHASE_COLOR_G) / 255);
                setFrontPanelLED(i + 1, chaseRed, chaseGreen);
                setPCBLED(8-i, chaseRed); // Set PCB LEDs to 3/4 brightness (192 out of 255).
            }
        } else {
            // After the initial delay, finalize channels one-by-one.
            if (animCurrentChannel < 8 && (now - channelCheckStartTime) >= CHECK_DURATION) {
                // Poll the cable for the current channel (using the helper function).
                bool isConnected = checkCableForChannel(animCurrentChannel);
                if (isConnected) {
                    setFrontPanelLED(animCurrentChannel + 1, 0, TARGET_GRN);  // Green for connected.
                } else {
                    setFrontPanelLED(animCurrentChannel + 1, TARGET_RED, 0);   // Red for disconnected.
                }
                channelsFinalized[animCurrentChannel] = true;
                animCurrentChannel++;
                channelCheckStartTime = now;  // Reset timer for next channel finalization.
            }
            
            // For channels not finalized yet, run the chase animation continuously.
            float globalPhase = fmod((now - chaseStartTime), chasePeriod) / chasePeriod;
            for (uint8_t i = 0; i < 8; i++) {
                if (!channelsFinalized[i]) {
                    float offset = chaseLeftToRight ? ((float)(7 - i) / 8.0) : ((float)i / 8.0);
                    float effectivePhase = fmod(globalPhase + offset, 1.0);
                    uint8_t brightness = computeChaseBrightness(effectivePhase);
                    uint8_t chaseRed = (uint8_t)((brightness * CHASE_COLOR_R) / 255);
                    uint8_t chaseGreen = (uint8_t)((brightness * CHASE_COLOR_G) / 255);
                    setFrontPanelLED(i + 1, chaseRed, chaseGreen);
                    setPCBLED(8-i, chaseRed); // Set PCB LEDs to 3/4 brightness (192 out of 255).
                }
            }
        }
        
        // Check if all channels are finalized and if a final wait has elapsed.
        if (animCurrentChannel >= 8 && (now - channelCheckStartTime >= FINAL_CHASE_WAIT)) {
            // Reset the chase timer variables for future re-runs if needed.
            chaseStartTime = 0;
            channelCheckStartTime = 0;
            startupPhase = FINAL_WAIT;
            animPhaseStartTime = now;  // Reset for FINAL_WAIT phase.
            
            for(int i = 0; i < 8; i++) {
              setPCBLED(i, 0); // Set all Front Panel LEDs to Off
            }
        }
        break;
    }


        case FINAL_WAIT: {
            uint8_t r = 0, g = 0, b = 0;
            switch (mode) {
                case 0:  r = TARGET_RED; g = 0;          b = 0;          break;   // ARMED (Red)
                case 1:  r = 0;          g = 0;          b = TARGET_BLU; break;   // TEST (Blue)
                case 2:  r = 0;          g = TARGET_GRN; b = 0;          break;   // NO_OUTPUT (Green)
                default: r = 0;          g = 0;          b = 0;          break;
            }
            updateModeLEDIfChanged(r, g, b);
            if (now - animPhaseStartTime >= FINAL_CHASE_WAIT) {
                startupAnimationDone = true;
            }
            break;
        }
    }
}


