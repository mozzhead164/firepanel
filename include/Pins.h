
#ifndef PINS_H
#define PINS_H

#include <Arduino.h>
#include <avr/pgmspace.h>


    // Define Pins for ATmega128
    #ifdef USE_ATMEGA128

        // INPUT PINS

        // Camera Input Pins
        #define Camera_CH1 22
        #define Camera_CH2 23
        #define Camera_CH3 24
        #define Camera_CH4 25
        #define Camera_CH5 A4   // 49 
        #define Camera_CH6 A5   // 50
        #define Camera_CH7 A6   // 51
        #define Camera_CH8 A7   // 52

        // External Interrupt Pins
        #define INT4_FP_BTNS 4      // INT4 - Front Panel Buttons 
        #define INT5_CAB_CON 5      // INT5 - Cable Connection Status
        #define INT6_BRK_GLS 6      // INT6 - Break Glass Input
        #define INT7_OP_CONF 7      // INT7 - Output Confirmation

        // Mode Selector Switch
        #define MODE_ARM 12         // Mode Switch - ARMED
        #define MODE_TST 13         // Mode Switch - TEST
        #define MODE_NOP 14         // Mode Switch - NO OUTPUT

        // ADC Input Pins - PSU Voltage Measurement + Temperature
        #define ADC0 A0             // ADC Pin for PSU 1
        #define ADC1 A1             // ADC Pin for PSU 2
        #define ONE_WIRE_BUS 47     // OneWire Bus Pin for Temperature Sensor


        // OUTPUT PINS

        // Thermal Alarm Output Pins
        #define THERMAL_1 28
        #define THERMAL_2 30
        #define THERMAL_3 32
        #define THERMAL_4 34
        #define THERMAL_5 29
        #define THERMAL_6 31
        #define THERMAL_7 33
        #define THERMAL_8 35

        // Test / Dummy Output Pins - Triggers Dummy Output Channels from FP Buttons
        #define DUMMY_OUT_1 44
        #define DUMMY_OUT_2 43
        #define DUMMY_OUT_3 42
        #define DUMMY_OUT_4 41
        #define DUMMY_OUT_5 40
        #define DUMMY_OUT_6 39
        #define DUMMY_OUT_7 38
        #define DUMMY_OUT_8 37

        // Output Control Pins
        #define BUZZER_OUT 2        // Buzzer Output Pin
        #define FAN_CONTROL 3        // Fan Control PWM Pin
        #define LED_TLC_RST 17      // TLC59116 Reset Pin


        // Misc Pins

        // Raspberry Pi Serial Communication Pins
        #define RASPI_TX 20         // RXD Pin for Serial Communication
        #define RASPI_RX 21         // TXD Pin for Serial Communication

        // SPI Pins for Bootloader
        #define UC_SCK   9          // SCK Pin for SPI (Bootloader)
        #define UC_MOSI 10          // MOSI Pin for SPI (Bootloader)
        #define UC_MISO 11          // MISO Pin for SPI (Bootloader)

        // Camera Pins and related arrays
        const uint8_t PROGMEM cameraPins_P[] = {  Camera_CH1, Camera_CH2, Camera_CH3, Camera_CH4, 
                                                  Camera_CH5, Camera_CH6, Camera_CH7, Camera_CH8  };

        const int numCameras = sizeof(cameraPins_P) / sizeof(cameraPins_P[0]);


        // Thermal Pins Array
        const uint8_t PROGMEM thermalPins_P[] = { THERMAL_1, THERMAL_2, THERMAL_3, THERMAL_4, 
                                                  THERMAL_5, THERMAL_6, THERMAL_7, THERMAL_8 };

        const int numThermals = sizeof(thermalPins_P) / sizeof(thermalPins_P[0]);


        // Dummy Pins Array
        const uint8_t PROGMEM dummyPins_P[] = {  DUMMY_OUT_1, DUMMY_OUT_2, DUMMY_OUT_3, DUMMY_OUT_4, 
                                                 DUMMY_OUT_5, DUMMY_OUT_6, DUMMY_OUT_7, DUMMY_OUT_8  };

        const int numDummy = sizeof(dummyPins_P) / sizeof(dummyPins_P[0]);


    // Define Pins for ATmega32
    #else

        // Camera Input Pins
        #define Camera_CH1 0  
        #define Camera_CH2 1
        #define Camera_CH3 2
        #define Camera_CH4 3
        #define Camera_CH5 4
        #define Camera_CH6 5
        #define Camera_CH7 6
        #define Camera_CH8 7
        #define INT6_BRK_GLS 11

        // Define Selector Switch
        #define MODE_ARM A3         // Mode Switch - ARMED
        #define MODE_TST A4         // Mode Switch - TEST
        #define MODE_NOP A5         // Mode Switch - NO OUTPUT
        
        // ADC Input Pins - PSU Voltage Measurement
        #define ADC0 A0             // ADC Pin for PSU 1
        #define ADC1 A1             // ADC Pin for PSU 2
        #define ONE_WIRE_BUS A2     // OneWire Bus Pin for Temperature Sensor

    #endif // USE_ATMEGA128



#endif // PINS_H
