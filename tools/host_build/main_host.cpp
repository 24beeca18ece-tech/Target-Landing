/*
  PC driver for the flight firmware.

  Compiles the real Parafoil_Guided_Landing.ino (after gen_prototypes.py)
  against the stand-in libraries in mock/, feeds it USB serial commands from
  the command line and runs loop() on a virtual 1 ms clock. With no real
  sensors attached, the useful command is SIM, which flies the firmware's
  built-in simulated canopy end to end and prints the miss distance.

  The mock I2C bus has no devices, so IMU / BMP581 / QMC5883 report FAIL and
  the SD card reports FAIL. That is expected here.
*/
#include "Arduino.h"
#include "Wire.h"
#include "SD.h"
#include "EEPROM.h"

uint64_t g_us = 0;
HardwareSerial Serial, Serial1, Serial3;
TwoWire Wire;
SDClass SD;
EEPROMClass EEPROM;

#include "ino_with_protos.cpp"

int main(int argc, char **argv) {
  setup();
  // Commands are separated by ';' on the command line, e.g. "TLM OFF;SIM 400 300 500 2 1"
  std::string cmds = argc > 1 ? argv[1] : "TLM OFF;SIM 400 300 500 2 1";
  for (char &c : cmds) if (c == ';') c = '\n';
  Serial.in = cmds + "\n";
  Serial.pos = 0;
  for (long i = 0; i < 3000000; i++) {      // up to 3000 s of virtual time
    loop();
    g_us += 1000;
    if (fsState == FS_IDLE && i > 1000 && Serial.pos >= Serial.in.size()) break;
  }
  return 0;
}
