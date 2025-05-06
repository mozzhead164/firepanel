#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "SystemData.h"



extern StaticJsonDocument<500> jsonDoc;
extern bool piHandshakeComplete;

// Public Functions
void initPiSerial();
void handleGetData();
void sendHeartbeat();
void pollJsonSerial();
void testPiHandshake();
void handleHandshake(const JsonDocument& doc);
void handleResetThermal();
void handleTriggerThermal();

const char* modeToStr(SystemMode mode);
void sendJson(const JsonDocument& doc);
void sendAck(const char* type, uint8_t channel = 0);
void sendNack(const char* reason);
void handleIncomingCommand(const JsonDocument& doc); // if you want to expose it (optional)
bool validateCommand(const JsonDocument &doc, const char *requiredType, const char *requiredField = nullptr);


#endif // SERIAL_COMMS_H
