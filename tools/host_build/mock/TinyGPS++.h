#pragma once
#include <stdint.h>
struct TGVal { bool isValid() const { return false; } bool isUpdated() { return false; } uint32_t age() const { return 99999; }
  double lat() const { return 0; } double lng() const { return 0; } uint32_t value() const { return 0; }
  double hdop() const { return 99; } double meters() const { return 0; } double mps() const { return 0; } double deg() const { return 0; } };
struct TinyGPSPlus { TGVal location, satellites, hdop, altitude, speed, course; bool encode(char) { return false; } };
