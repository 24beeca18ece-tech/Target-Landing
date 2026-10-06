#pragma once
#include "Arduino.h"
struct SMS_STS { HardwareSerial *pSerial = nullptr;
  int Ping(uint8_t id) { return id; }
  int ReadPos(uint8_t) { return 2048; }
  int ReadVoltage(int) { return 74; }
  int ReadTemper(int) { return 30; }
  int EnableTorque(uint8_t, uint8_t) { return 1; }
  int WritePosEx(uint8_t, int16_t, uint16_t, uint8_t) { return 1; } };
