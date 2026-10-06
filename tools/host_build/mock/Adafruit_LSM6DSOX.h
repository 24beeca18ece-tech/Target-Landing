#pragma once
#include "Wire.h"
struct sensors_vec_t { float x, y, z; };
struct sensors_event_t { sensors_vec_t acceleration; sensors_vec_t gyro; float temperature; };
enum { LSM6DS_ACCEL_RANGE_16_G, LSM6DS_GYRO_RANGE_2000_DPS, LSM6DS_RATE_208_HZ };
struct Adafruit_LSM6DSOX { bool begin_I2C(uint8_t, TwoWire *) { return false; }
  void setAccelRange(int) {} void setGyroRange(int) {} void setAccelDataRate(int) {} void setGyroDataRate(int) {}
  bool getEvent(sensors_event_t *, sensors_event_t *, sensors_event_t *) { return false; } };
