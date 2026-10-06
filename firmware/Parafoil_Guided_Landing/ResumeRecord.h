/*
  ResumeRecord.h
  EEPROM record that lets the flight computer resume after an in-flight
  watchdog reset. Kept in its own header (not in the .ino) because the
  Arduino IDE inserts auto-generated function prototypes near the top of the
  .ino, before any struct defined there, which breaks functions that take a
  ResumeRecord parameter ("'ResumeRecord' does not name a type").
*/
#pragma once
#include <stdint.h>

struct ResumeRecord {
  uint32_t magic;
  uint8_t  state;
  float    groundPressurePa;
  double   targetLat, targetLon;
  float    elevOffset;
  float    gpsAltAtArm;
  uint32_t check;
};

const uint32_t RESUME_MAGIC = 0x50475231u;   // "PGR1"
