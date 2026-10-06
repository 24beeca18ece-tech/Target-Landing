/*
  EARLY SENSOR BRING-UP SKETCH (superseded).
  Written before the guided-landing firmware and never run on the hardware.
  The flight firmware (../Parafoil_Guided_Landing) uses a register-level
  BMP581 driver and the Adafruit LSM6DSOX driver instead of the libraries
  named below. Kept for reference only.
*/
/*
  ============================================================================
  Sensor Bring-Up Firmware -- Teensy 4.1
  Foxeer M10Q-180 GPS | QMC5883L Compass | LSM6DSO32 IMU | BMP581 Barometer
  ============================================================================

  Purpose: standalone bring-up / bench-test sketch. Brings all four sensors
  online with a non-blocking scheduler, a hardware watchdog, and per-sensor
  re-init retries so a loose header pin or a brown-out on one module doesn't
  hang or crash the rest. This is NOT the flight FSM -- it's the bench build
  to confirm wiring and get clean readings from every sensor before this
  goes into the packet-format telemetry stream.

  Board:  Teensy 4.1, Tools > Board > Teensy 4.1, USB Type: Serial,
          CPU Speed: 600 MHz

  Pinout (matches the wiring table this sketch was written against):
    GPS   (Foxeer M10Q-180)  VCC->5V   GND->GND   TX->pin15(RX3)  RX->pin14(TX3)
    Compass (QMC5883L)       VCC->3.3V GND->GND   SDA->pin18      SCL->pin19   addr 0x0D
    IMU     (LSM6DSO32)      VCC->3.3V GND->GND   SDA->pin18      SCL->pin19   addr 0x6A
    Baro    (BMP581)         VCC->3.3V GND->GND   SDA->pin18      SCL->pin19   addr 0x46 or 0x47

  Required libraries (install via Library Manager unless noted):
    - TinyGPSPlus            by Mikal Hart
    - Adafruit LSM6DS        (provides Adafruit_LSM6DSO32.h) -- also pulls in
                              Adafruit BusIO and Adafruit Unified Sensor
    - QMC5883LCompass        by MPrograms
    - SparkFun BMP581 Arduino Library
    - Watchdog_t4            by tonton81 (search GitHub if not in Library
                              Manager on your IDE version)

  KNOWN UNCERTAINTY (flagging per project convention -- verify before flight
  use, not just before bench power-up):
    - The exact enum names used below for the LSM6DSO32 accel/gyro range and
      ODR (LSM6DSO32_ACCEL_RANGE_16_G, LSM6DS_GYRO_RANGE_2000_DPS,
      LSM6DS_RATE_104_HZ) come from the Adafruit_LSM6DS family convention.
      If your installed library version names them differently, the compiler
      will point at the exact line -- swap in the names from that library's
      own example sketch.
    - The SparkFun BMP581 library's beginI2C()/getSensorData() signatures and
      the BMP5_OK / bmp5_sensor_data symbol names are written from the
      library's documented pattern. Same instruction: if it doesn't compile,
      match the symbol names to your installed version's example sketch.
    - Both are common, actively maintained libraries -- this is a "confirm
      the exact API surface" flag, not a "this approach is wrong" flag.
*/

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSO32.h>
#include <QMC5883LCompass.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include <Watchdog_t4.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define GPS_SERIAL          Serial3
#define GPS_BAUD            9600      // uBlox M10-series NMEA default; change
                                       // if this module has been reconfigured

#define LSM6DSO32_ADDR      0x6A
#define BMP581_ADDR_PRIMARY 0x47
#define BMP581_ADDR_ALT     0x46
#define QMC5883_ADDR        0x0D

static const uint32_t I2C_CLOCK_HZ        = 400000;  // Fast-mode I2C
static const uint32_t SENSOR_PERIOD_MS    = 50;       // 20 Hz IMU/baro/mag read
static const uint32_t GPS_PRINT_PERIOD_MS = 1000;      // 1 Hz GPS status print
static const uint32_t RETRY_INTERVAL_MS   = 2000;      // re-init cadence for a
                                                         // sensor that's offline

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
TinyGPSPlus       gps;
Adafruit_LSM6DSO32 imu;
QMC5883LCompass    compass;
BMP581             baro;
WDT_T4<WDT1>       wdt;

bool imuOnline     = false;
bool baroOnline    = false;
bool compassOnline = false;

uint8_t bmpAddrInUse = BMP581_ADDR_PRIMARY;

uint32_t lastSensorRead   = 0;
uint32_t lastGpsPrint     = 0;
uint32_t lastImuRetry     = 0;
uint32_t lastBaroRetry    = 0;
uint32_t lastCompassRetry = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
bool i2cDevicePresent(uint8_t addr);
void scanI2CBus();
bool initIMU();
bool initBaro();
bool initCompass();
void serviceGPS();
void serviceSensors();

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    // Wait up to 3 s for a USB host to open the port, then proceed headless
    // so this doesn't hang forever on a bench run without a laptop attached.
  }

  GPS_SERIAL.begin(GPS_BAUD);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println(F("== Teensy 4.1 sensor bring-up =="));
  scanI2CBus();

  imuOnline     = initIMU();
  baroOnline    = initBaro();
  compassOnline = initCompass();

  // Hardware watchdog: if the loop stalls (e.g. a wedged I2C transaction
  // that Wire's own timeout doesn't clear) the board force-resets instead
  // of sitting dead on the bench or, worse, in the air.
  WDT_timings_t config;
  config.trigger  = 1;  // seconds -- warning callback fires here
  config.timeout  = 2;  // seconds -- hard reset fires here
  config.callback = []() {
    Serial.println(F("WDT WARNING: loop stall detected, reset imminent"));
  };
  wdt.begin(config);

  Serial.println(F("== Bring-up complete, entering main loop =="));
}

// ---------------------------------------------------------------------------
// Main loop -- fully non-blocking
// ---------------------------------------------------------------------------
void loop() {
  wdt.feed();

  serviceGPS();

  uint32_t now = millis();
  if (now - lastSensorRead >= SENSOR_PERIOD_MS) {
    lastSensorRead = now;
    serviceSensors();
  }

  // Any sensor that failed at boot, or that serviceSensors() marked offline
  // after a failed read, gets a fresh re-init attempt on its own cadence
  // rather than blocking the rest of the loop.
  if (!imuOnline && now - lastImuRetry >= RETRY_INTERVAL_MS) {
    lastImuRetry = now;
    imuOnline = initIMU();
  }
  if (!baroOnline && now - lastBaroRetry >= RETRY_INTERVAL_MS) {
    lastBaroRetry = now;
    baroOnline = initBaro();
  }
  if (!compassOnline && now - lastCompassRetry >= RETRY_INTERVAL_MS) {
    lastCompassRetry = now;
    compassOnline = initCompass();
  }
}

// ---------------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------------
bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void scanI2CBus() {
  Serial.println(F("I2C scan:"));
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (i2cDevicePresent(addr)) {
      Serial.print(F("  0x"));
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  (nothing responded -- check power, SDA/SCL wiring, pull-ups)"));
  }
}

// ---------------------------------------------------------------------------
// Per-sensor init (each gated on a raw I2C ACK check first, independent of
// what each library's own return code says, so a library-level false
// positive can't mask a wiring fault)
// ---------------------------------------------------------------------------
bool initIMU() {
  if (!i2cDevicePresent(LSM6DSO32_ADDR)) {
    Serial.println(F("LSM6DSO32: no ACK on 0x6A"));
    return false;
  }
  if (!imu.begin_I2C(LSM6DSO32_ADDR, &Wire)) {
    Serial.println(F("LSM6DSO32: begin_I2C() failed"));
    return false;
  }
  imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  Serial.println(F("LSM6DSO32 online (0x6A)"));
  return true;
}

bool initBaro() {
  bmpAddrInUse = BMP581_ADDR_PRIMARY;
  if (!i2cDevicePresent(bmpAddrInUse)) {
    bmpAddrInUse = BMP581_ADDR_ALT;
  }
  if (!i2cDevicePresent(bmpAddrInUse)) {
    Serial.println(F("BMP581: no ACK on 0x47 or 0x46"));
    return false;
  }
  int8_t err = baro.beginI2C(bmpAddrInUse, Wire);
  if (err != BMP5_OK) {
    Serial.print(F("BMP581: beginI2C() failed, code "));
    Serial.println(err);
    return false;
  }
  Serial.print(F("BMP581 online (0x"));
  Serial.print(bmpAddrInUse, HEX);
  Serial.println(F(")"));
  return true;
}

bool initCompass() {
  if (!i2cDevicePresent(QMC5883_ADDR)) {
    Serial.println(F("QMC5883L: no ACK on 0x0D"));
    return false;
  }
  compass.init();
  Serial.println(F("QMC5883L online (0x0D)"));
  return true;
}

// ---------------------------------------------------------------------------
// GPS servicing -- byte-drain every loop (non-blocking), status print at 1 Hz
// ---------------------------------------------------------------------------
void serviceGPS() {
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  uint32_t now = millis();
  if (now - lastGpsPrint < GPS_PRINT_PERIOD_MS) {
    return;
  }
  lastGpsPrint = now;

  Serial.print(F("GPS fix="));
  Serial.print(gps.location.isValid() ? F("Y") : F("N"));
  Serial.print(F(" sats="));
  Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);

  if (gps.location.isValid()) {
    Serial.print(F(" lat="));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" lon="));
    Serial.print(gps.location.lng(), 6);
  }
  if (gps.altitude.isValid()) {
    Serial.print(F(" alt="));
    Serial.print(gps.altitude.meters(), 1);
  }
  if (gps.charsProcessed() < 10 && now > 10000) {
    Serial.print(F(" [WARNING: no NMEA characters seen after 10s -- check "
                    "TX/RX crossover, baud rate, and 5V supply]"));
  }
  Serial.println();
}

// ---------------------------------------------------------------------------
// IMU / baro / compass servicing at SENSOR_PERIOD_MS
// ---------------------------------------------------------------------------
void serviceSensors() {
  float ax = NAN, ay = NAN, az = NAN, gx = NAN, gy = NAN, gz = NAN, imuTempC = NAN;
  if (imuOnline) {
    sensors_event_t accelEvt, gyroEvt, tempEvt;
    if (imu.getEvent(&accelEvt, &gyroEvt, &tempEvt)) {
      ax = accelEvt.acceleration.x;
      ay = accelEvt.acceleration.y;
      az = accelEvt.acceleration.z;
      gx = gyroEvt.gyro.x;
      gy = gyroEvt.gyro.y;
      gz = gyroEvt.gyro.z;
      imuTempC = tempEvt.temperature;
    } else {
      Serial.println(F("LSM6DSO32: read failed, marking offline for retry"));
      imuOnline = false;
    }
  }

  float pressurePa = NAN, baroTempC = NAN;
  if (baroOnline) {
    bmp5_sensor_data data = {0};
    int8_t err = baro.getSensorData(&data);
    if (err == BMP5_OK) {
      pressurePa = data.pressure;
      baroTempC = data.temperature;
    } else {
      Serial.println(F("BMP581: read failed, marking offline for retry"));
      baroOnline = false;
    }
  }

  int mx = 0, my = 0, mz = 0, azimuth = 0;
  if (compassOnline) {
    if (i2cDevicePresent(QMC5883_ADDR)) {
      compass.read();
      mx = compass.getX();
      my = compass.getY();
      mz = compass.getZ();
      azimuth = compass.getAzimuth();
    } else {
      Serial.println(F("QMC5883L: dropped off bus, marking offline for retry"));
      compassOnline = false;
    }
  }

  // Bench status line. Swap this for CSV / packet framing once folded into
  // the flight telemetry stream -- kept as plain text here so it's readable
  // straight off the Serial Monitor during bring-up.
  Serial.print(F("t="));
  Serial.print(millis());

  Serial.print(F(" imu="));
  Serial.print(imuOnline ? F("OK") : F("OFF"));
  if (imuOnline) {
    Serial.print(F(" a=["));
    Serial.print(ax, 2); Serial.print(',');
    Serial.print(ay, 2); Serial.print(',');
    Serial.print(az, 2);
    Serial.print(F("] g=["));
    Serial.print(gx, 2); Serial.print(',');
    Serial.print(gy, 2); Serial.print(',');
    Serial.print(gz, 2);
    Serial.print(F("] T="));
    Serial.print(imuTempC, 1);
  }

  Serial.print(F(" baro="));
  Serial.print(baroOnline ? F("OK") : F("OFF"));
  if (baroOnline) {
    Serial.print(F(" P="));
    Serial.print(pressurePa, 1);
    Serial.print(F("Pa T="));
    Serial.print(baroTempC, 2);
  }

  Serial.print(F(" mag="));
  Serial.print(compassOnline ? F("OK") : F("OFF"));
  if (compassOnline) {
    Serial.print(F(" xyz=["));
    Serial.print(mx); Serial.print(',');
    Serial.print(my); Serial.print(',');
    Serial.print(mz);
    Serial.print(F("] hdg="));
    Serial.print(azimuth);
  }

  Serial.println();
}
