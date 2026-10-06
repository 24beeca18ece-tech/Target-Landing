/*
  ============================================================================
  TEENSY 4.1  AUTONOMOUS PARAFOIL  TARGETED LANDING  (CanSat, 80 m requirement)
  ============================================================================

  WHAT IS NEW COMPARED TO THE PREVIOUS LAB CODE
    Old: point the canopy at the target, centre the servos inside 35 m.
         That overflies the target with height to spare and lands wherever
         the remaining glide takes it (the 617 m simulation result).
    New: energy-managed guidance (see ParafoilGuidance.h for the maths):
         HOMING -> LOITER (burn height in a 50 m orbit) -> FINAL (leave the
         orbit when the remaining glide just reaches the target, S-turns burn
         the last metres) -> TERMINAL (wings level below 8 m).
         Wind is measured in flight from the loiter turns; airspeed and sink
         rate are measured in flight; nothing about the canopy is assumed
         except the starting guesses.

  The guidance core (ParafoilGuidance.h) is the SAME file that was run through
  the PC Monte-Carlo test (host_sim/monte_carlo.cpp). Keep them together.

  ----------------------------------------------------------------------------
  HARDWARE (unchanged from the verified lab setup)
  ----------------------------------------------------------------------------
    GPS  Foxeer M10Q-180   TX -> pin 15 (RX3), RX -> pin 14 (TX3), 38400 baud
    I2C  SDA 18, SCL 19    QMC5883 0x0D, LSM6DSO32 0x6A, BMP581 0x46/0x47
    ST-3009 x2             Serial1 half duplex 1 Mbps, ID 1 and ID 2, neutral 2048
    SG90 deploy servo      pin 8
    SD                     BUILTIN_SDCARD

  ----------------------------------------------------------------------------
  FLIGHT SEQUENCE
  ----------------------------------------------------------------------------
    IDLE --ARM--> ARMED --climb--> ASCENT --apogee--> DROGUE
         --h <= 500 m above target--> DEPLOYING (SG90 release, 1.5 s)
         --> CANOPY (guidance core: DAMPING 8 s -> GUIDED -> TERMINAL)
         --> LANDED
    A watchdog reset in flight resumes from EEPROM (state, ground pressure,
    target) instead of starting over on the pad.

  ----------------------------------------------------------------------------
  BENCH / FIELD TEST ORDER (do these in order, do not skip)
  ----------------------------------------------------------------------------
    1. PING           both ST-3009 answer
    2. LEFT / RIGHT   LEFT must shorten the LEFT brake line, RIGHT the right
                      one. If not, fix LEFT_PULL_SIGN / RIGHT_PULL_SIGN.
    3. GYROTEST       rotate the CanSat CLOCKWISE seen from above: the printed
                      heading must INCREASE. If not, flip YAW_RATE_SIGN.
    4. BARO           pressure ~ local station pressure; lift 1 m -> h ~ +1 m
    5. SIM            closed-loop simulated flight driving the REAL servos.
                      Ends by printing the simulated miss distance.
    6. WALK           walk with the CanSat (GPS outside). Servo shows which
                      way to turn toward the target; inside ~100 m it asks
                      you to circle the target (that is the loiter).
    7. Drop / flight tests: measure turn rate per count, sink, airspeed
       (the log has everything), then set TURN_SIGN, SLACK, GAINS.

  ----------------------------------------------------------------------------
  SERIAL COMMANDS (USB, 115200, newline terminated, case-insensitive)
  ----------------------------------------------------------------------------
    HELP | STATUS | ARM | DISARM | CLEAR
    TARGET <lat> <lon>    TARGETHERE    ELEV <m>  (target ground minus pad, m)
    DEPLOY | PREP | CENTER | LEFT [counts] | RIGHT [counts] | MOTORTEST | PING
    SIM [north east height windN windE]   WALK   STOP
    GAINS <kPsi> <kR> <kI> <kFF>   SLACK <counts>   LOITER <radius> <dir +1/-1>
    LAB | FLIGHT  (GPS HDOP gate 8.0 / 4.0)
    GYROTEST | BARO | LOG ON | LOG OFF
  ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SD.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SCServo.h>
#include <Adafruit_LSM6DSOX.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#if __has_include(<Watchdog_t4.h>)
  #include <Watchdog_t4.h>
  #define HAVE_WDT 1
  WDT_T4<WDT1> wdt;
#else
  #define HAVE_WDT 0
#endif

#include "ParafoilGuidance.h"
#include "ResumeRecord.h"

// ============================================================================
// USER CONFIGURATION
// ============================================================================

// ---------------------------- Target ----------------------------------------
double TARGET_LAT = 32.6321769;            // change here or with TARGET / TARGETHERE
double TARGET_LON = 75.0143889;
float  TARGET_ELEV_OFFSET_M = 0.0f;        // target ground height MINUS the ARM-point height.
                                           // 10 m wrong here costs ~10-25 m of landing error.

// ---------------------------- Flight thresholds -----------------------------
const float PARAFOIL_DEPLOY_HEIGHT_M = 500.0f;   // above target ground
const float LAUNCH_DETECT_HEIGHT_M   = 30.0f;
const float LAUNCH_DETECT_VUP_MPS    = 8.0f;
const float APOGEE_VDOWN_MPS         = 2.0f;     // descending faster than this ...
const float APOGEE_CONFIRM_S         = 1.0f;     // ... for this long = apogee
const float APOGEE_DROP_M            = 15.0f;    // or this far below max height
const float LOW_APOGEE_DEPLOY_DELAY_S = 2.0f;    // apogee below deploy height: deploy after this
const float LANDED_HEIGHT_M          = 20.0f;
const float LANDED_STILL_S           = 5.0f;
const float MAX_CANOPY_TIME_S        = 900.0f;

// ---------------------------- Flare (OFF until drop-tested) -----------------
const bool  FLARE_ENABLED   = false;
const float FLARE_HEIGHT_M  = 3.0f;
const int   FLARE_COUNTS    = 350;

// ---------------------------- GPS -------------------------------------------
#define GPS_SERIAL Serial3
const uint32_t GPS_BAUD          = 38400;    // verified working baud
const uint8_t  GPS_RATE_HZ       = 5;        // requested via UBX-CFG-VALSET at boot
const uint8_t  GPS_MIN_SATS      = 5;
const float    GPS_MAX_HDOP_FLIGHT = 4.0f;
const float    GPS_MAX_HDOP_LAB    = 8.0f;   // the 5.13 HDOP that blocked the old code passes here
const uint32_t GPS_MAX_AGE_MS    = 1500;

// ---------------------------- I2C devices -----------------------------------
const uint8_t QMC5883_ADDR   = 0x0D;
const uint8_t LSM6DSO32_ADDR = 0x6A;
const uint8_t BMP581_ADDR_1  = 0x46;
const uint8_t BMP581_ADDR_2  = 0x47;

// Compass calibration (verified standalone test values)
const float MAG_OFFSET_X = 180.0000f, MAG_OFFSET_Y = 336.0000f, MAG_OFFSET_Z = 615.0000f;
const float MAG_SCALE_X  = 0.956484f, MAG_SCALE_Y  = 0.904406f, MAG_SCALE_Z  = 1.178127f;
const bool  COMPASS_MIRROR_FOR_NAV   = true;      // verified: navHeading = 360 - heading
const float COMPASS_HEADING_OFFSET_DEG = 0.0f;

// ---------------------------- IMU yaw axis ----------------------------------
// Heading convention is clockwise-positive seen from above. If the IMU Z axis
// points UP, a clockwise turn gives a NEGATIVE gyro Z, so the sign is -1.
// VERIFY WITH GYROTEST.
const uint8_t YAW_RATE_AXIS = 2;        // 0 = X, 1 = Y, 2 = Z
const float   YAW_RATE_SIGN = -1.0f;

// ---------------------------- ST-3009 steering ------------------------------
const uint8_t SERVO1_ID = 1;
const uint8_t SERVO2_ID = 2;
const bool    SERVO1_IS_LEFT = true;
const int     SERVO_NEUTRAL  = 2048;
const int     SERVO_MIN_POS  = 200;
const int     SERVO_MAX_POS  = 3895;
const int     MAX_STEER_COUNTS = 450;
// Position change direction that PULLS each brake line (from the verified bench code).
const int     LEFT_PULL_SIGN  = -1;
const int     RIGHT_PULL_SIGN = -1;
// +1: positive demand pulls the RIGHT line. Flip to -1 only if the first drop
// test shows the canopy turning the opposite way to the commanded direction.
const int     TURN_SIGN = +1;
// Speed 500 / accel 50 = the values in the user's working ST-3009 test sketch.
// Changeable at runtime with the SPEED command.
int           steerServoSpeed   = 500;
const int     STEER_SERVO_ACCEL = 50;
const uint32_t SERVO_UPDATE_MS  = 50;
const uint8_t  SERVO_MIN_STEP   = 4;      // counts; smaller changes are not sent
const uint32_t SERVO_BUS_GAP_MS = 10;     // pause after each bus write (known-good used 15-20)
const uint32_t SERVO_HEALTH_MS  = 2000;

// ---------------------------- SG90 deployment -------------------------------
const uint8_t  DEPLOY_SERVO_PIN = 8;
const int      PREP_ANGLE   = 140;        // locked
const int      DEPLOY_ANGLE = 180;        // released
const uint32_t DEPLOY_MOVE_MS = 1500;

// ---------------------------- Rates -----------------------------------------
const uint32_t IMU_PERIOD_US     = 10000;   // 100 Hz
const uint32_t BARO_PERIOD_MS    = 40;      // 25 Hz
const uint32_t COMPASS_PERIOD_MS = 100;     // 10 Hz
const uint32_t CONTROL_PERIOD_MS = 20;      // 50 Hz
const uint32_t TLM_PERIOD_MS     = 250;     // 4 Hz USB telemetry
const uint32_t LOG_PERIOD_MS     = 100;     // 10 Hz SD log
const uint32_t LOG_FLUSH_MS      = 1000;

// ============================================================================
// GLOBALS
// ============================================================================
TinyGPSPlus        gps;
SMS_STS            st;
Adafruit_LSM6DSOX  imu;
Servo              deployServo;

pg::FlightCore     core;
pg::CoreConfig     coreCfg;
pg::GeoRef         geo;
pg::AltBetaFilter  fsmAlt;        // height filter for launch / apogee / landing logic
pg::SimVehicle     simVeh;
pg::Rng            simRng;

static uint8_t gpsRxBuffer[1024];

enum FlightState : uint8_t {
  FS_IDLE = 0, FS_ARMED, FS_ASCENT, FS_DROGUE, FS_DEPLOYING, FS_CANOPY, FS_LANDED, FS_SIM, FS_WALK
};
const char *fsName(uint8_t s) {
  switch (s) {
    case FS_IDLE: return "IDLE";       case FS_ARMED: return "ARMED";
    case FS_ASCENT: return "ASCENT";   case FS_DROGUE: return "DROGUE";
    case FS_DEPLOYING: return "DEPLOYING"; case FS_CANOPY: return "CANOPY";
    case FS_LANDED: return "LANDED";   case FS_SIM: return "SIM";
    case FS_WALK: return "WALK";
  }
  return "?";
}
uint8_t  fsState = FS_IDLE;
uint32_t fsEnteredMs = 0;

// Health flags
bool imuOK = false, bmpOK = false, compassOK = false, sdOK = false;
bool servo1OK = false, servo2OK = false;
uint8_t bmpAddr = 0;
uint8_t bmpChipId = 0;

// GPS state
bool   gpsGood = false;
double gpsLat = 0, gpsLon = 0;
float  gpsAltM = 0, gpsSpeed = 0, gpsCourse = 0, gpsHdop = 99.9f;
uint8_t gpsSats = 0;
uint32_t gpsFixCount = 0, gpsFixCountLast = 0;
float  gpsFixRateHz = 0;
uint32_t gpsRateWindowMs = 0;
float  gpsMaxHdop = GPS_MAX_HDOP_FLIGHT;
float  gpsAltAtArm = 0;

// Baro state
float pressurePa = 0, baroTempC = 0;
float groundPressurePa = 0;       // captured at ARM
bool  groundSet = false;
float hTarget = 0;                // m above target ground (raw)
uint32_t lastBaroMs = 0;

// IMU / compass
float yawRateDegS = 0, gyroBiasDegS = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float compassNavDeg = 0;
uint8_t compassFails = 0;
uint32_t lastImuUs = 0, lastCompassMs = 0;

// Flight-logic bookkeeping
float hMax = 0;
uint32_t apogeeTimerMs = 0, apogeeMs = 0, landedTimerMs = 0;
bool  apogeeSeen = false, deployFired = false, flareActive = false;

// Steering output
float    steerDemand = 0;          // counts, + = right turn (before TURN_SIGN)
int      manualDemand = 0;         // bench commands
int      cmdS1 = SERVO_NEUTRAL, cmdS2 = SERVO_NEUTRAL;
int      sentS1 = -1, sentS2 = -1;
uint32_t lastServoMs = 0, lastHealthMs = 0;

// Timers
uint32_t lastControlMs = 0, lastTlmMs = 0, lastLogMs = 0, lastFlushMs = 0;

// SD log
File logFile;
bool logEnabled = true;
bool tlmEnabled = true;    // TLM ON/OFF: GUIDE lines on USB (SD logging is unaffected)
char logName[16] = "";

// Gyro test
bool gyroTestActive = false;
uint32_t gyroTestEndMs = 0;
float gyroTestPsi = 0;

// SIM
float simLastT = 0;
uint32_t simLastUs = 0;

// Serial command buffer
char cmdBuf[96];
uint8_t cmdLen = 0;

// ============================================================================
// EEPROM RESUME RECORD
// ============================================================================
// struct ResumeRecord and RESUME_MAGIC live in ResumeRecord.h (see note there).

uint32_t recordCheck(const ResumeRecord &r) {
  const uint8_t *p = (const uint8_t *)&r;
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < offsetof(ResumeRecord, check); i++) { h ^= p[i]; h *= 16777619u; }
  return h;
}
void saveResume() {
  ResumeRecord r;
  memset(&r, 0, sizeof(r));
  r.magic = RESUME_MAGIC; r.state = fsState;
  r.groundPressurePa = groundPressurePa;
  r.targetLat = TARGET_LAT; r.targetLon = TARGET_LON;
  r.elevOffset = TARGET_ELEV_OFFSET_M; r.gpsAltAtArm = gpsAltAtArm;
  r.check = recordCheck(r);
  EEPROM.put(0, r);
}
void clearResume() {
  ResumeRecord r;
  memset(&r, 0, sizeof(r));
  EEPROM.put(0, r);
}
bool loadResume(ResumeRecord &r) {
  EEPROM.get(0, r);
  if (r.magic != RESUME_MAGIC || r.check != recordCheck(r)) return false;
  return r.state == FS_ARMED || r.state == FS_ASCENT || r.state == FS_DROGUE ||
         r.state == FS_DEPLOYING || r.state == FS_CANOPY;
}

// ============================================================================
// SMALL HELPERS
// ============================================================================
float tSec() { return millis() * 0.001f; }

void setState(uint8_t s) {
  if (s == fsState) return;
  Serial.print("# STATE "); Serial.print(fsName(fsState)); Serial.print(" -> "); Serial.println(fsName(s));
  fsState = s;
  fsEnteredMs = millis();
  if (s == FS_ARMED || s == FS_ASCENT || s == FS_DROGUE || s == FS_DEPLOYING || s == FS_CANOPY) saveResume();
  if (s == FS_LANDED || s == FS_IDLE) clearResume();
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

float pressureToHeight(float p, float p0) {
  if (!(p > 0.0f) || !(p0 > 0.0f)) return 0.0f;
  return 44330.0f * (1.0f - powf(p / p0, 0.190295f));
}

// ============================================================================
// BMP581 (register level; Bosch BMP5 register map)
// ============================================================================
const uint8_t BMP5_REG_CHIP_ID    = 0x01;
const uint8_t BMP5_REG_TEMP_XLSB  = 0x1D;
const uint8_t BMP5_REG_INT_STATUS = 0x27;
const uint8_t BMP5_REG_STATUS     = 0x28;
const uint8_t BMP5_REG_OSR_CONFIG = 0x36;
const uint8_t BMP5_REG_ODR_CONFIG = 0x37;
const uint8_t BMP5_REG_CMD        = 0x7E;
const uint8_t BMP5_ODR_50HZ       = 0x0F;

bool bmpWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bmpAddr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool bmpRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(bmpAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(bmpAddr, n) != n) { while (Wire.available()) Wire.read(); return false; }
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool initBMP581() {
  if (i2cPresent(BMP581_ADDR_1)) bmpAddr = BMP581_ADDR_1;
  else if (i2cPresent(BMP581_ADDR_2)) bmpAddr = BMP581_ADDR_2;
  else return false;

  bmpWrite(BMP5_REG_CMD, 0xB6);              // soft reset
  delay(5);
  uint8_t v = 0;
  if (!bmpRead(BMP5_REG_CHIP_ID, &v, 1)) return false;
  bmpChipId = v;
  if (v != 0x50 && v != 0x51) {
    Serial.print("# BMP581: unexpected chip id 0x"); Serial.println(v, HEX);
    return false;
  }
  if (bmpRead(BMP5_REG_STATUS, &v, 1)) {
    if (!(v & 0x02) || (v & 0x04)) { Serial.print("# BMP581: NVM status 0x"); Serial.println(v, HEX); }
  }
  bmpRead(BMP5_REG_INT_STATUS, &v, 1);       // clear power-on flag

  if (!bmpWrite(BMP5_REG_ODR_CONFIG, 0x80)) return false;                 // standby, deep standby off
  delay(3);
  if (!bmpWrite(BMP5_REG_OSR_CONFIG, 0x40 | (0x03 << 3) | 0x00)) return false;  // press on, OSR_P x8, OSR_T x1
  if (!bmpWrite(BMP5_REG_ODR_CONFIG, 0x80 | (BMP5_ODR_50HZ << 2) | 0x01)) return false;  // normal mode
  delay(30);
  return true;
}

bool readBMP581(float &pPa, float &tC) {
  uint8_t b[6];
  if (!bmpRead(BMP5_REG_TEMP_XLSB, b, 6)) return false;
  int32_t traw = (int32_t)(((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0]);
  if (traw & 0x00800000) traw |= (int32_t)0xFF000000;
  const uint32_t praw = ((uint32_t)b[5] << 16) | ((uint32_t)b[4] << 8) | b[3];
  tC = traw / 65536.0f;
  pPa = praw / 64.0f;
  return pPa > 30000.0f && pPa < 115000.0f && tC > -40.0f && tC < 85.0f;
}

// ============================================================================
// QMC5883 (verified register settings)
// ============================================================================
bool initQMC5883() {
  if (!i2cPresent(QMC5883_ADDR)) return false;
  Wire.beginTransmission(QMC5883_ADDR); Wire.write(0x0B); Wire.write(0x01);
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  Wire.beginTransmission(QMC5883_ADDR); Wire.write(0x09); Wire.write(0x1D);
  if (Wire.endTransmission() != 0) return false;
  delay(20);
  return true;
}

bool readQMC5883(float &navDeg) {
  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(QMC5883_ADDR, (uint8_t)6) != 6) { while (Wire.available()) Wire.read(); return false; }
  const uint8_t xl = Wire.read(), xh = Wire.read(), yl = Wire.read(), yh = Wire.read();
  Wire.read(); Wire.read();   // Z not needed for heading
  const float mx = ((float)(int16_t)((xh << 8) | xl) - MAG_OFFSET_X) * MAG_SCALE_X;
  const float my = ((float)(int16_t)((yh << 8) | yl) - MAG_OFFSET_Y) * MAG_SCALE_Y;
  float hdg = pg::wrap360(atan2f(my, mx) * pg::kRad2Deg + COMPASS_HEADING_OFFSET_DEG);
  navDeg = COMPASS_MIRROR_FOR_NAV ? pg::wrap360(360.0f - hdg) : hdg;
  return true;
}

// ============================================================================
// IMU
// ============================================================================
bool initIMU() {
  if (!imu.begin_I2C(LSM6DSO32_ADDR, &Wire)) return false;
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_208_HZ);
  return true;
}

bool readYawRateRaw(float &rateDegS) {
  sensors_event_t a, g, t;
  if (!imu.getEvent(&a, &g, &t)) return false;
  accelX = a.acceleration.x; accelY = a.acceleration.y; accelZ = a.acceleration.z;
  const float axis = (YAW_RATE_AXIS == 0) ? g.gyro.x : (YAW_RATE_AXIS == 1) ? g.gyro.y : g.gyro.z;
  rateDegS = YAW_RATE_SIGN * axis * pg::kRad2Deg;
  return isfinite(rateDegS);
}

// Average the gyro while stationary (call only on the pad / bench).
void calibrateGyro() {
  if (!imuOK) return;
  float sum = 0; int n = 0;
  for (int i = 0; i < 200; i++) {
    float r;
    if (readYawRateRaw(r)) { sum += r; n++; }
    delay(5);
#if HAVE_WDT
    wdt.feed();
#endif
  }
  if (n > 50) gyroBiasDegS = sum / n;
  Serial.print("# gyro bias = "); Serial.print(gyroBiasDegS, 3); Serial.println(" deg/s");
}

// ============================================================================
// GPS
// ============================================================================
void ubxSend(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  const uint8_t hdr[4] = {cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  GPS_SERIAL.write(0xB5); GPS_SERIAL.write(0x62);
  for (uint8_t i = 0; i < 4; i++) { GPS_SERIAL.write(hdr[i]); ckA += hdr[i]; ckB += ckA; }
  for (uint16_t i = 0; i < len; i++) { GPS_SERIAL.write(payload[i]); ckA += payload[i]; ckB += ckA; }
  GPS_SERIAL.write(ckA); GPS_SERIAL.write(ckB);
}

static uint8_t *putKeyU1(uint8_t *p, uint32_t key, uint8_t v) {
  *p++ = key & 0xFF; *p++ = (key >> 8) & 0xFF; *p++ = (key >> 16) & 0xFF; *p++ = (key >> 24) & 0xFF;
  *p++ = v; return p;
}
static uint8_t *putKeyU2(uint8_t *p, uint32_t key, uint16_t v) {
  *p++ = key & 0xFF; *p++ = (key >> 8) & 0xFF; *p++ = (key >> 16) & 0xFF; *p++ = (key >> 24) & 0xFF;
  *p++ = v & 0xFF; *p++ = v >> 8; return p;
}

// u-blox M10: UBX-CFG-VALSET to RAM. If the module refuses a key it NAKs the
// message and keeps its old settings (no harm). STATUS shows the real fix rate.
void configureGps() {
  uint8_t buf[64];
  uint8_t *p = buf;
  *p++ = 0x00; *p++ = 0x01; *p++ = 0x00; *p++ = 0x00;          // version, layer RAM, reserved
  p = putKeyU2(p, 0x30210001UL, (uint16_t)(1000 / GPS_RATE_HZ)); // CFG-RATE-MEAS (ms)
  p = putKeyU1(p, 0x20110021UL, 6);                            // CFG-NAVSPG-DYNMODEL = airborne <1g
  p = putKeyU1(p, 0x209100C5UL, 0);                            // NMEA GSV on UART1 off
  p = putKeyU1(p, 0x209100CAUL, 0);                            // NMEA GLL on UART1 off
  p = putKeyU1(p, 0x209100B1UL, 0);                            // NMEA VTG on UART1 off
  ubxSend(0x06, 0x8A, buf, (uint16_t)(p - buf));
}

void serviceGps() {
  while (GPS_SERIAL.available()) gps.encode(GPS_SERIAL.read());

  // One update per GPS epoch: trigger on the RMC sentence (the only one that
  // carries speed). Triggering on location fired twice per epoch (GGA and RMC
  // both carry position), which double-counted the fix rate and fed every
  // fix to the guidance twice.
  if (gps.speed.isUpdated()) {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
    gpsSats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
    gpsHdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 99.9f;
    if (gps.altitude.isValid()) gpsAltM = (float)gps.altitude.meters();
    if (gps.speed.isValid()) gpsSpeed = (float)gps.speed.mps();
    if (gps.course.isValid()) gpsCourse = (float)gps.course.deg();
    gpsFixCount++;

    gpsGood = gps.location.isValid() && gps.location.age() < GPS_MAX_AGE_MS &&
              gpsSats >= GPS_MIN_SATS && gpsHdop <= gpsMaxHdop;

    if (gpsGood && fsState != FS_SIM) {
      float n, e;
      geo.toNE(gpsLat, gpsLon, n, e);
      const float c = gpsCourse * pg::kDeg2Rad;
      core.onGps(tSec(), true, n, e, gpsSpeed * cosf(c), gpsSpeed * sinf(c));
    }
  }

  const uint32_t now = millis();
  if (now - gpsRateWindowMs >= 2000) {
    gpsFixRateHz = (gpsFixCount - gpsFixCountLast) * 1000.0f / (now - gpsRateWindowMs);
    gpsFixCountLast = gpsFixCount;
    gpsRateWindowMs = now;
  }
}

// ============================================================================
// ST-3009 STEERING SERVOS
// ============================================================================
bool servoHealthAuto = true;   // HEALTH ON/OFF: periodic background PING
bool servoDebug      = true;   // SERVODBG ON/OFF: print every servo write and its ACK
uint32_t servoWriteFails1 = 0, servoWriteFails2 = 0;

void updateServoHealth(bool force) {
  if (!force && !servoHealthAuto) return;
  if (!force && millis() - lastHealthMs < SERVO_HEALTH_MS) return;
  lastHealthMs = millis();
  servo1OK = st.Ping(SERVO1_ID) >= 0;
  delay(SERVO_BUS_GAP_MS);
  servo2OK = st.Ping(SERVO2_ID) >= 0;
  delay(SERVO_BUS_GAP_MS);
}

// demand > 0 = turn RIGHT (pull right brake). Flare pulls both.
void demandToPositions(float demand, int &posLeft, int &posRight) {
  posLeft = SERVO_NEUTRAL;
  posRight = SERVO_NEUTRAL;
  if (flareActive) {
    posLeft  = SERVO_NEUTRAL + LEFT_PULL_SIGN * FLARE_COUNTS;
    posRight = SERVO_NEUTRAL + RIGHT_PULL_SIGN * FLARE_COUNTS;
  } else {
    const float d = pg::clampf(demand * TURN_SIGN, -MAX_STEER_COUNTS, MAX_STEER_COUNTS);
    const int mag = (int)lroundf(fabsf(d));
    if (d > 0.5f)       posRight = SERVO_NEUTRAL + RIGHT_PULL_SIGN * mag;
    else if (d < -0.5f) posLeft  = SERVO_NEUTRAL + LEFT_PULL_SIGN * mag;
  }
  posLeft  = constrain(posLeft, SERVO_MIN_POS, SERVO_MAX_POS);
  posRight = constrain(posRight, SERVO_MIN_POS, SERVO_MAX_POS);
}

// Make sure both steering servos have torque on. Torque-enable is a RAM
// register; if an earlier program or a protection event turned it off, the
// servo still answers PING and ReadPos but will not drive its motor.
void enableSteeringTorque() {
  st.EnableTorque(SERVO1_ID, 1);
  delay(SERVO_BUS_GAP_MS);
  st.EnableTorque(SERVO2_ID, 1);
  delay(SERVO_BUS_GAP_MS);
}

// PING diagnostics: commanded vs real position, supply voltage, temperature.
void printServoDiag(uint8_t id, const char *name, bool ok, int commanded) {
  if (!ok) return;
  const int pos = st.ReadPos(id);   delay(SERVO_BUS_GAP_MS);
  const int v10 = st.ReadVoltage(id); delay(SERVO_BUS_GAP_MS);
  const int tC  = st.ReadTemper(id);  delay(SERVO_BUS_GAP_MS);
  Serial.print("#   servo"); Serial.print(name);
  Serial.print("  commanded "); Serial.print(commanded);
  Serial.print("  real pos "); Serial.print(pos);
  Serial.print("  supply "); Serial.print(v10 >= 0 ? v10 / 10.0f : -1.0f, 1);
  Serial.print(" V  temp "); Serial.print(tC); Serial.print(" C  missed ACKs ");
  Serial.println(id == SERVO1_ID ? servoWriteFails1 : servoWriteFails2);
}

// Send a position only when it changed. WritePosEx returns 1 when the servo
// ACKs the packet. On a missing ACK the position is NOT marked as sent, so it
// is retried on the next servo update (every SERVO_UPDATE_MS).
void writeServoIfChanged(uint8_t id, int pos, int &sent) {
  const bool backToNeutral = (pos == SERVO_NEUTRAL && sent != SERVO_NEUTRAL);
  if (sent < 0 || abs(pos - sent) >= SERVO_MIN_STEP || backToNeutral) {
    const int ack = st.WritePosEx(id, pos, steerServoSpeed, STEER_SERVO_ACCEL);
    if (ack > 0) {
      sent = pos;
    } else {
      if (id == SERVO1_ID) servoWriteFails1++; else servoWriteFails2++;
    }
    if (servoDebug) {
      Serial.print("# write ID"); Serial.print(id); Serial.print(" -> "); Serial.print(pos);
      Serial.print("  speed "); Serial.print(steerServoSpeed);
      Serial.println(ack > 0 ? "  ACK OK" : "  NO ACK (will retry)");
    }
    delay(SERVO_BUS_GAP_MS);
  }
}

// Exact replica of the user's working ST-3009 test sketch (same positions,
// speed 500, accel 50, 1.5 s holds), run from inside this firmware. If this
// moves both servos but LEFT/RIGHT do not, the problem is in the loop, not
// in the bus or the servos.
void busTestOne(uint8_t id) {
  const int seq[5] = {2048, 1748, 2048, 2348, 2048};
  for (int i = 0; i < 5; i++) {
    const int ack = st.WritePosEx(id, seq[i], 500, 50);
    Serial.print("# BUSTEST ID"); Serial.print(id); Serial.print(" -> "); Serial.print(seq[i]);
    Serial.println(ack > 0 ? "  ACK OK" : "  NO ACK");
    for (int k = 0; k < 15; k++) {
      delay(100);
#if HAVE_WDT
      wdt.feed();
#endif
    }
    const int pos = st.ReadPos(id);
    Serial.print("#            real pos "); Serial.println(pos);
  }
}

void serviceServos(bool force) {
  if (!force && millis() - lastServoMs < SERVO_UPDATE_MS) return;
  lastServoMs = millis();
  int posL, posR;
  demandToPositions(steerDemand, posL, posR);
  cmdS1 = SERVO1_IS_LEFT ? posL : posR;
  cmdS2 = SERVO1_IS_LEFT ? posR : posL;
  writeServoIfChanged(SERVO1_ID, cmdS1, sentS1);
  writeServoIfChanged(SERVO2_ID, cmdS2, sentS2);
}

// ============================================================================
// SG90 DEPLOYMENT
// ============================================================================
void deployLock() {
  deployServo.attach(DEPLOY_SERVO_PIN);
  deployServo.write(PREP_ANGLE);
}
void deployRelease() {
  deployServo.attach(DEPLOY_SERVO_PIN);
  deployServo.write(DEPLOY_ANGLE);
  deployFired = true;
  Serial.println("# DEPLOY: SG90 -> release");
}

// ============================================================================
// SENSOR SERVICE
// ============================================================================
void serviceImu() {
  const uint32_t nowUs = micros();
  if (nowUs - lastImuUs < IMU_PERIOD_US) return;
  const float dt = (lastImuUs == 0) ? 0.01f : (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (!imuOK) return;
  float r;
  if (!readYawRateRaw(r)) return;
  yawRateDegS = r - gyroBiasDegS;
  if (fsState != FS_SIM) core.onImu(yawRateDegS, dt);
  if (gyroTestActive) gyroTestPsi = pg::wrap360(gyroTestPsi + yawRateDegS * dt);
}

void serviceBaro() {
  const uint32_t now = millis();
  if (now - lastBaroMs < BARO_PERIOD_MS) return;
  const float dt = (lastBaroMs == 0) ? 0.04f : (now - lastBaroMs) * 0.001f;
  lastBaroMs = now;

  bool have = false;
  if (bmpOK) {
    float p = 0, t = 0;
    if (readBMP581(p, t)) { pressurePa = p; baroTempC = t; have = true; }
  }
  if (!groundSet) return;

  float hArm;
  if (have) hArm = pressureToHeight(pressurePa, groundPressurePa);
  else if (gpsGood) hArm = gpsAltM - gpsAltAtArm;          // fallback: GPS altitude
  else return;

  hTarget = hArm - TARGET_ELEV_OFFSET_M;
  fsmAlt.update(hTarget, dt);
  if (fsState != FS_SIM && fsState != FS_WALK) core.onBaro(hTarget, dt);
}

void serviceCompass() {
  const uint32_t now = millis();
  if (now - lastCompassMs < COMPASS_PERIOD_MS) return;
  lastCompassMs = now;
  if (!compassOK) {
    if (++compassFails % 50 == 0) compassOK = initQMC5883();
    return;
  }
  float hdg;
  if (readQMC5883(hdg)) {
    compassNavDeg = hdg; compassFails = 0;
    if (fsState != FS_SIM) core.onCompass(tSec(), hdg);
  } else if (++compassFails >= 5) {
    compassOK = initQMC5883();
    compassFails = 0;
  }
}

// ============================================================================
// FLIGHT STATE MACHINE
// ============================================================================
void startCanopyGuidance() {
  core.begin(coreCfg);
  core.startDescent(tSec());
  setState(FS_CANOPY);
}

void serviceFlightLogic() {
  const float h = fsmAlt.h;
  const float vUp = fsmAlt.v;
  const uint32_t now = millis();

  switch (fsState) {
    case FS_ARMED:
      steerDemand = 0;
      if (h > LAUNCH_DETECT_HEIGHT_M && vUp > LAUNCH_DETECT_VUP_MPS) { hMax = h; setState(FS_ASCENT); }
      break;

    case FS_ASCENT:
      steerDemand = 0;
      if (h > hMax) hMax = h;
      if (vUp < -APOGEE_VDOWN_MPS) {
        if (apogeeTimerMs == 0) apogeeTimerMs = now;
      } else {
        apogeeTimerMs = 0;
      }
      if ((apogeeTimerMs && now - apogeeTimerMs >= (uint32_t)(APOGEE_CONFIRM_S * 1000)) || h < hMax - APOGEE_DROP_M) {
        apogeeSeen = true; apogeeMs = now;
        Serial.print("# APOGEE at "); Serial.print(hMax, 1); Serial.println(" m");
        setState(FS_DROGUE);
      }
      break;

    case FS_DROGUE: {
      steerDemand = 0;
      const bool lowApogee = hMax < PARAFOIL_DEPLOY_HEIGHT_M + 20.0f;
      const bool reached = h <= PARAFOIL_DEPLOY_HEIGHT_M && vUp < -1.0f;
      const bool lowTimer = lowApogee && (now - apogeeMs) >= (uint32_t)(LOW_APOGEE_DEPLOY_DELAY_S * 1000);
      if (reached || lowTimer) {
        deployRelease();
        setState(FS_DEPLOYING);
      }
      break;
    }

    case FS_DEPLOYING:
      steerDemand = 0;
      if (now - fsEnteredMs >= DEPLOY_MOVE_MS) {
        deployServo.detach();
        startCanopyGuidance();
      }
      break;

    case FS_CANOPY: {
      flareActive = FLARE_ENABLED && h <= FLARE_HEIGHT_M;
      const bool still = h < LANDED_HEIGHT_M && fabsf(vUp) < 0.4f && gpsSpeed < 1.0f;
      if (still) { if (!landedTimerMs) landedTimerMs = now; } else landedTimerMs = 0;
      if ((landedTimerMs && now - landedTimerMs >= (uint32_t)(LANDED_STILL_S * 1000)) ||
          (now - fsEnteredMs) >= (uint32_t)(MAX_CANOPY_TIME_S * 1000)) {
        flareActive = false;
        steerDemand = 0;
        float n, e; geo.toNE(gpsLat, gpsLon, n, e);
        Serial.print("# LANDED. Distance to target = "); Serial.print(pg::hyp(n, e), 1); Serial.println(" m");
        setState(FS_LANDED);
      }
      break;
    }

    case FS_LANDED:
      steerDemand = 0;
      flareActive = false;
      break;

    default:
      break;
  }
}

// ============================================================================
// SIM AND WALK BENCH MODES
// ============================================================================
void startSim(float n0, float e0, float h0, float wN, float wE) {
  pg::SimParams sp;
  sp.windN = wN; sp.windE = wE; sp.gustSigma = 0.3f;
  simRng.seed(micros());
  simVeh.reset(sp, n0, e0, h0, simRng.uniform(0.0f, 360.0f));
  core.begin(coreCfg);
  core.startDescent(tSec());
  simLastUs = micros();
  setState(FS_SIM);
  Serial.print("# SIM start: N "); Serial.print(n0, 0); Serial.print(" E "); Serial.print(e0, 0);
  Serial.print(" h "); Serial.print(h0, 0); Serial.print(" wind N "); Serial.print(wN, 1);
  Serial.print(" E "); Serial.println(wE, 1);
}

void serviceSim() {
  if (fsState != FS_SIM) return;
  const uint32_t nowUs = micros();
  float dt = (nowUs - simLastUs) * 1e-6f;
  if (dt < 0.01f) return;
  simLastUs = nowUs;
  if (dt > 0.1f) dt = 0.1f;

  simVeh.step(steerDemand, dt, simRng);
  const float t = tSec();
  core.onImu(simVeh.r, dt);
  core.onBaro(simVeh.h + 0.2f * simRng.gauss(), dt);
  static float gpsAcc = 0.0f;
  gpsAcc += dt;
  if (gpsAcc >= 1.0f / GPS_RATE_HZ) {
    gpsAcc = 0.0f;
    core.onGps(t, true, simVeh.n + 0.8f * simRng.gauss(), simVeh.e + 0.8f * simRng.gauss(),
               simVeh.vgN + 0.1f * simRng.gauss(), simVeh.vgE + 0.1f * simRng.gauss());
  }
  if (simVeh.h <= 0.0f) {
    Serial.println("========================================");
    Serial.print("# SIM LANDED. Miss distance = ");
    Serial.print(pg::hyp(simVeh.n, simVeh.e), 1);
    Serial.println(" m  (requirement: 80 m)");
    Serial.println("========================================");
    steerDemand = 0;
    setState(FS_IDLE);
    core.begin(coreCfg);
  }
}

void startWalk() {
  pg::CoreConfig wc = coreCfg;
  wc.guide.maxWind = 0.0f;              // no wind when walking
  wc.minGpsSpeedForHeading = 0.6f;      // walking pace
  wc.guide.terminalHeight = -1.0f;      // never go terminal
  gpsMaxHdop = GPS_MAX_HDOP_LAB;
  core.begin(wc);
  core.forceGuided(tSec());
  setState(FS_WALK);
  Serial.println("# WALK: walk toward the target; servo shows the commanded turn. Inside ~100 m it asks you to circle.");
}

void serviceWalk() {
  static uint32_t last = 0;
  if (fsState != FS_WALK) return;
  if (millis() - last < 40) return;
  last = millis();
  core.onHeightDirect(300.0f, 1.0f, 0.04f);   // pretend: plenty of height, slow sink
}

// ============================================================================
// CONTROL TICK
// ============================================================================
void serviceControl() {
  const uint32_t now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;
  const float dt = (lastControlMs == 0) ? 0.02f : (now - lastControlMs) * 0.001f;
  lastControlMs = now;

  if (fsState == FS_CANOPY || fsState == FS_SIM || fsState == FS_WALK) {
    steerDemand = core.step(tSec(), dt);
  } else if (fsState == FS_IDLE) {
    steerDemand = (float)manualDemand;
  } else {
    steerDemand = 0;
  }
}

// ============================================================================
// TELEMETRY + SD LOG
// ============================================================================
// GUIDE,ms,state,coreMode,phase,lat,lon,pN,pE,h,vz,gs,psi,psiCmd,rateCmd,yawRate,
//       demand,S1,S2,wN,wE,windOK,va,tgo,L,d,excess,vtN,vtE,predMiss,sats,hdop,gpsOK
int buildGuideLine(char *out, size_t n) {
  float pN = core.pN, pE = core.pE, h = core.h;
  if (fsState == FS_SIM) { pN = simVeh.n; pE = simVeh.e; h = simVeh.h; }
  const pg::GuidanceOutput &g = core.gout;
  return snprintf(out, n,
    "GUIDE,%lu,%s,%s,%s,%.7f,%.7f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,"
    "%.0f,%d,%d,%.2f,%.2f,%d,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%.2f,%d",
    (unsigned long)millis(), fsName(fsState), pg::coreModeName(core.coreMode()), pg::phaseName(g.phase),
    gpsLat, gpsLon, pN, pE, h, core.sinkRate(), pg::hyp(core.vN, core.vE),
    core.heading.psi, core.psiCmd, core.steering.rateCmd, core.yawRate,
    steerDemand, cmdS1, cmdS2, core.wind.wN, core.wind.wE, core.wind.valid ? 1 : 0, core.airspeed(),
    g.tgo, g.L, g.d, g.excess, g.vtN, g.vtE, g.predMiss, gpsSats, gpsHdop, core.gpsOk ? 1 : 0);
}

void openLog() {
  if (!sdOK || !logEnabled) return;
  if (logFile) logFile.close();
  for (int i = 0; i < 1000; i++) {
    snprintf(logName, sizeof(logName), "FLT%03d.CSV", i);
    if (!SD.exists(logName)) break;
  }
  logFile = SD.open(logName, FILE_WRITE);
  if (logFile) {
    logFile.println("tag,ms,state,coreMode,phase,lat,lon,pN,pE,h,vz,gs,psi,psiCmd,rateCmd,yawRate,"
                    "demand,S1,S2,wN,wE,windOK,va,tgo,L,d,excess,vtN,vtE,predMiss,sats,hdop,gpsOK");
    Serial.print("# LOG -> "); Serial.println(logName);
  } else {
    Serial.println("# LOG: could not open file");
  }
}

void serviceTelemetry() {
  const uint32_t now = millis();
  char line[300];
  bool built = false;

  if (now - lastTlmMs >= TLM_PERIOD_MS) {
    lastTlmMs = now;
    buildGuideLine(line, sizeof(line));
    built = true;
    if (tlmEnabled) Serial.println(line);
    if (gyroTestActive) {
      Serial.print("# GYROTEST heading = "); Serial.print(gyroTestPsi, 1);
      Serial.println("  (rotate CLOCKWISE seen from above: must INCREASE)");
      if (now > gyroTestEndMs) { gyroTestActive = false; Serial.println("# GYROTEST done"); }
    }
  }
  if (logFile && now - lastLogMs >= LOG_PERIOD_MS) {
    lastLogMs = now;
    if (!built) buildGuideLine(line, sizeof(line));
    logFile.println(line);
  }
  if (logFile && now - lastFlushMs >= LOG_FLUSH_MS) {
    lastFlushMs = now;
    logFile.flush();
  }
}

// ============================================================================
// COMMANDS
// ============================================================================
void printHelp() {
  Serial.println("Commands: HELP STATUS ARM DISARM CLEAR | TARGET lat lon | TARGETHERE | ELEV m");
  Serial.println("          DEPLOY PREP CENTER LEFT [n] RIGHT [n] MOTORTEST PING TORQUE SPEED n");
  Serial.println("          BUSTEST | HEALTH ON|OFF | SERVODBG ON|OFF | TLM ON|OFF");
  Serial.println("          SIM [n e h wN wE] | WALK | STOP | GAINS kPsi kR kI kFF | SLACK n");
  Serial.println("          LOITER radius dir | LAB | FLIGHT | GYROTEST | BARO | LOG ON|OFF");
}

void printStatus() {
  float n = 0, e = 0;
  geo.toNE(gpsLat, gpsLon, n, e);
  Serial.println("================ STATUS ================");
  Serial.print("State        : "); Serial.print(fsName(fsState)); Serial.print(" / core ");
  Serial.print(pg::coreModeName(core.coreMode())); Serial.print(" / "); Serial.println(pg::phaseName(core.gout.phase));
  Serial.print("Target       : "); Serial.print(TARGET_LAT, 7); Serial.print(", "); Serial.print(TARGET_LON, 7);
  Serial.print("  elev offset "); Serial.print(TARGET_ELEV_OFFSET_M, 1); Serial.println(" m");
  Serial.print("GPS          : "); Serial.print(gpsGood ? "GOOD" : "NOT GOOD");
  Serial.print("  sats "); Serial.print(gpsSats); Serial.print("  HDOP "); Serial.print(gpsHdop, 2);
  Serial.print(" (gate "); Serial.print(gpsMaxHdop, 1); Serial.print(")  rate "); Serial.print(gpsFixRateHz, 1);
  Serial.println(" Hz");
  Serial.print("Position     : "); Serial.print(gpsLat, 7); Serial.print(", "); Serial.print(gpsLon, 7);
  Serial.print("  -> "); Serial.print(pg::hyp(n, e), 1); Serial.print(" m from target, bearing ");
  Serial.println(pg::bearingDeg(-n, -e), 1);
  Serial.print("Speed/course : "); Serial.print(gpsSpeed, 2); Serial.print(" m/s  "); Serial.println(gpsCourse, 1);
  Serial.print("Baro         : "); Serial.print(bmpOK ? "OK" : "FAIL"); Serial.print(" chip 0x"); Serial.print(bmpChipId, HEX);
  Serial.print("  P "); Serial.print(pressurePa, 1); Serial.print(" Pa  T "); Serial.print(baroTempC, 1);
  Serial.print(" C  h(target) "); Serial.print(fsmAlt.h, 1); Serial.println(groundSet ? " m" : " m (NOT ARMED: no ground ref)");
  Serial.print("IMU          : "); Serial.print(imuOK ? "OK" : "FAIL"); Serial.print("  yaw rate ");
  Serial.print(yawRateDegS, 2); Serial.print(" deg/s  bias "); Serial.println(gyroBiasDegS, 3);
  Serial.print("Compass      : "); Serial.print(compassOK ? "OK" : "FAIL"); Serial.print("  nav heading ");
  Serial.println(compassNavDeg, 1);
  Serial.print("Heading est  : "); Serial.print(core.heading.init ? "" : "(not initialised) ");
  Serial.println(core.heading.psi, 1);
  Serial.print("Servos       : ID1 "); Serial.print(servo1OK ? "OK" : "FAIL"); Serial.print("  ID2 ");
  Serial.print(servo2OK ? "OK" : "FAIL"); Serial.print("  S1 "); Serial.print(cmdS1); Serial.print("  S2 ");
  Serial.println(cmdS2);
  Serial.print("Wind est     : "); Serial.print(core.wind.valid ? "VALID " : "not yet ");
  Serial.print(core.wind.wN, 2); Serial.print(" N, "); Serial.print(core.wind.wE, 2); Serial.print(" E  Va ");
  Serial.print(core.airspeed(), 2); Serial.print("  sink "); Serial.println(core.sinkRate(), 2);
  Serial.print("Gains        : kPsi "); Serial.print(coreCfg.steer.kPsi, 2); Serial.print(" kR ");
  Serial.print(coreCfg.steer.kR, 2); Serial.print(" kI "); Serial.print(coreCfg.steer.kI, 2);
  Serial.print(" kFF "); Serial.print(coreCfg.steer.kFF, 2); Serial.print(" slack ");
  Serial.println(coreCfg.steer.slackCounts, 0);
  Serial.print("Loiter       : R "); Serial.print(coreCfg.guide.loiterRadius, 0); Serial.print(" m dir ");
  Serial.println(coreCfg.guide.loiterDir);
  Serial.print("SD           : "); Serial.print(sdOK ? "OK " : "FAIL "); Serial.println(logName);
  Serial.print("Watchdog     : "); Serial.println(HAVE_WDT ? "Watchdog_t4 2 s" : "NOT INSTALLED");
  Serial.println("========================================");
}

bool doArm() {
  if (!bmpOK && !gpsGood) { Serial.println("# ARM rejected: no baro and no GPS altitude"); return false; }
  if (!servo1OK || !servo2OK) { Serial.println("# ARM rejected: ST-3009 not answering (run PING)"); return false; }
  if (bmpOK) {
    float sum = 0; int n = 0;
    for (int i = 0; i < 25; i++) {
      float p = 0, t = 0;
      if (readBMP581(p, t)) { sum += p; n++; }
      delay(20);
    }
    if (n < 10) { Serial.println("# ARM rejected: baro read failures"); return false; }
    groundPressurePa = sum / n;
  }
  gpsAltAtArm = gpsAltM;
  groundSet = true;
  calibrateGyro();
  fsmAlt.reset(-TARGET_ELEV_OFFSET_M);
  hMax = 0; apogeeTimerMs = 0; apogeeSeen = false; deployFired = false; landedTimerMs = 0;
  manualDemand = 0;
  deployLock();
  core.begin(coreCfg);
  openLog();
  setState(FS_ARMED);
  Serial.print("# ARMED. Ground pressure "); Serial.print(groundPressurePa, 1); Serial.println(" Pa");
  return true;
}

void handleCommand(char *line) {
  for (char *p = line; *p; p++) *p = toupper(*p);
  char *tok = strtok(line, " ,\t");
  if (!tok) return;
  char *a1 = strtok(nullptr, " ,\t");
  char *a2 = strtok(nullptr, " ,\t");
  char *a3 = strtok(nullptr, " ,\t");
  char *a4 = strtok(nullptr, " ,\t");
  char *a5 = strtok(nullptr, " ,\t");

  const bool flying = (fsState == FS_ASCENT || fsState == FS_DROGUE || fsState == FS_DEPLOYING || fsState == FS_CANOPY);

  if (!strcmp(tok, "HELP")) printHelp();
  else if (!strcmp(tok, "STATUS")) printStatus();
  else if (!strcmp(tok, "ARM")) { if (fsState == FS_IDLE) doArm(); else Serial.println("# ARM only from IDLE"); }
  else if (!strcmp(tok, "DISARM") || !strcmp(tok, "STOP")) {
    steerDemand = 0; manualDemand = 0; flareActive = false;
    core.begin(coreCfg);
    gpsMaxHdop = GPS_MAX_HDOP_FLIGHT;
    setState(FS_IDLE);
  }
  else if (!strcmp(tok, "CLEAR")) { clearResume(); Serial.println("# resume record cleared"); }
  else if (!strcmp(tok, "TARGET") && a1 && a2) {
    if (flying) { Serial.println("# TARGET refused in flight"); return; }
    TARGET_LAT = atof(a1); TARGET_LON = atof(a2);
    geo.setOrigin(TARGET_LAT, TARGET_LON);
    Serial.print("# TARGET = "); Serial.print(TARGET_LAT, 7); Serial.print(", "); Serial.println(TARGET_LON, 7);
  }
  else if (!strcmp(tok, "TARGETHERE")) {
    if (flying || !gpsGood) { Serial.println("# TARGETHERE needs good GPS and not flying"); return; }
    TARGET_LAT = gpsLat; TARGET_LON = gpsLon;
    geo.setOrigin(TARGET_LAT, TARGET_LON);
    Serial.print("# TARGET = here: "); Serial.print(TARGET_LAT, 7); Serial.print(", "); Serial.println(TARGET_LON, 7);
  }
  else if (!strcmp(tok, "ELEV") && a1) { TARGET_ELEV_OFFSET_M = atof(a1); Serial.print("# ELEV offset = "); Serial.println(TARGET_ELEV_OFFSET_M, 1); }
  else if (!strcmp(tok, "DEPLOY")) { if (!flying) { deployRelease(); } }
  else if (!strcmp(tok, "PREP")) { if (!flying) { deployLock(); Serial.println("# SG90 -> lock"); } }
  else if (!strcmp(tok, "CENTER")) { if (fsState == FS_IDLE) { manualDemand = 0; Serial.println("# CENTER"); } }
  else if (!strcmp(tok, "LEFT")) { if (fsState == FS_IDLE) { manualDemand = -(a1 ? atoi(a1) : 300); Serial.print("# LEFT "); Serial.println(-manualDemand); } }
  else if (!strcmp(tok, "RIGHT")) { if (fsState == FS_IDLE) { manualDemand = (a1 ? atoi(a1) : 300); Serial.print("# RIGHT "); Serial.println(manualDemand); } }
  else if (!strcmp(tok, "MOTORTEST")) {
    if (fsState != FS_IDLE) return;
    const int seq[5] = {0, -300, 0, 300, 0};
    for (int i = 0; i < 5; i++) {
      steerDemand = seq[i];
      serviceServos(true);
      Serial.print("# MOTORTEST demand "); Serial.print(seq[i]); Serial.print("  S1 "); Serial.print(cmdS1);
      Serial.print("  S2 "); Serial.println(cmdS2);
      for (int k = 0; k < 10; k++) {
        delay(100);
#if HAVE_WDT
        wdt.feed();
#endif
      }
    }
    manualDemand = 0;
  }
  else if (!strcmp(tok, "PING")) {
    updateServoHealth(true);
    Serial.print("# ST-3009 ID1 "); Serial.print(servo1OK ? "OK" : "NO ANSWER");
    Serial.print("  ID2 "); Serial.println(servo2OK ? "OK" : "NO ANSWER");
    printServoDiag(SERVO1_ID, "1", servo1OK, cmdS1);
    printServoDiag(SERVO2_ID, "2", servo2OK, cmdS2);
  }
  else if (!strcmp(tok, "BUSTEST")) {
    if (fsState != FS_IDLE) { Serial.println("# BUSTEST only from IDLE"); return; }
    manualDemand = 0;
    busTestOne(SERVO1_ID);
    busTestOne(SERVO2_ID);
    sentS1 = -1; sentS2 = -1;              // resync the normal servo loop afterwards
    Serial.println("# BUSTEST done");
  }
  else if (!strcmp(tok, "TLM") && a1) {
    tlmEnabled = !strcmp(a1, "ON");
    Serial.print("# GUIDE telemetry "); Serial.println(tlmEnabled ? "ON" : "OFF");
  }
  else if (!strcmp(tok, "HEALTH") && a1) {
    servoHealthAuto = !strcmp(a1, "ON");
    Serial.print("# background PING "); Serial.println(servoHealthAuto ? "ON" : "OFF");
  }
  else if (!strcmp(tok, "SERVODBG") && a1) {
    servoDebug = !strcmp(a1, "ON");
    Serial.print("# servo write debug "); Serial.println(servoDebug ? "ON" : "OFF");
  }
  else if (!strcmp(tok, "SPEED") && a1) {
    steerServoSpeed = constrain(atoi(a1), 50, 3400);
    sentS1 = -1; sentS2 = -1;            // force the current position to be re-sent with the new speed
    Serial.print("# servo speed = "); Serial.println(steerServoSpeed);
  }
  else if (!strcmp(tok, "TORQUE")) {
    enableSteeringTorque();
    Serial.println("# torque enable sent to ID1 and ID2");
  }
  else if (!strcmp(tok, "SIM")) {
    if (fsState != FS_IDLE) { Serial.println("# SIM only from IDLE"); return; }
    startSim(a1 ? atof(a1) : 400.0f, a2 ? atof(a2) : 300.0f, a3 ? atof(a3) : 500.0f,
             a4 ? atof(a4) : 0.0f, a5 ? atof(a5) : 0.0f);
  }
  else if (!strcmp(tok, "WALK")) { if (fsState == FS_IDLE) startWalk(); }
  else if (!strcmp(tok, "GAINS") && a1 && a2 && a3 && a4) {
    coreCfg.steer.kPsi = atof(a1); coreCfg.steer.kR = atof(a2); coreCfg.steer.kI = atof(a3); coreCfg.steer.kFF = atof(a4);
    if (!flying) core.begin(coreCfg); else core.steering.cfg = coreCfg.steer;
    Serial.println("# gains updated");
  }
  else if (!strcmp(tok, "SLACK") && a1) {
    coreCfg.steer.slackCounts = atof(a1); core.steering.cfg.slackCounts = coreCfg.steer.slackCounts;
    Serial.print("# slack = "); Serial.println(coreCfg.steer.slackCounts, 0);
  }
  else if (!strcmp(tok, "LOITER") && a1 && a2) {
    if (flying) { Serial.println("# LOITER refused in flight"); return; }
    coreCfg.guide.loiterRadius = pg::clampf(atof(a1), 25.0f, 150.0f);
    coreCfg.guide.loiterDir = atoi(a2) >= 0 ? +1 : -1;
    core.begin(coreCfg);
    Serial.println("# loiter updated");
  }
  else if (!strcmp(tok, "LAB")) { gpsMaxHdop = GPS_MAX_HDOP_LAB; Serial.println("# LAB: HDOP gate 8.0"); }
  else if (!strcmp(tok, "FLIGHT")) { gpsMaxHdop = GPS_MAX_HDOP_FLIGHT; Serial.println("# FLIGHT: HDOP gate 4.0"); }
  else if (!strcmp(tok, "GYROTEST")) { gyroTestActive = true; gyroTestPsi = 0; gyroTestEndMs = millis() + 15000; }
  else if (!strcmp(tok, "BARO")) {
    Serial.print("# P "); Serial.print(pressurePa, 1); Serial.print(" Pa  T "); Serial.print(baroTempC, 2);
    Serial.print(" C  h "); Serial.print(fsmAlt.h, 2); Serial.print(" m  vUp "); Serial.println(fsmAlt.v, 2);
  }
  else if (!strcmp(tok, "LOG") && a1) {
    logEnabled = !strcmp(a1, "ON");
    if (!logEnabled && logFile) { logFile.close(); }
    else if (logEnabled && !logFile) openLog();
    Serial.print("# log "); Serial.println(logEnabled ? "ON" : "OFF");
  }
  else { Serial.print("# unknown: "); Serial.println(tok); }
}

void serviceCommands() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
#if HAVE_WDT
void wdtWarning() { Serial.println("# WDT WARNING: loop stalled"); }
#endif

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.println();
  Serial.println("==== TEENSY 4.1 PARAFOIL TARGETED LANDING ====");

  geo.setOrigin(TARGET_LAT, TARGET_LON);
  coreCfg = pg::CoreConfig();       // defaults validated in host_sim/monte_carlo.cpp
  core.begin(coreCfg);

  // GPS
  GPS_SERIAL.addMemoryForRead(gpsRxBuffer, sizeof(gpsRxBuffer));   // servo bus writes block briefly
  GPS_SERIAL.begin(GPS_BAUD);
  delay(100);
  configureGps();

  // I2C sensors
  Wire.begin();
  Wire.setClock(400000);
  imuOK = initIMU();
  bmpOK = initBMP581();
  compassOK = initQMC5883();
  Serial.print("# IMU "); Serial.print(imuOK ? "OK" : "FAIL");
  Serial.print("  BMP581 "); Serial.print(bmpOK ? "OK" : "FAIL");
  Serial.print("  QMC5883 "); Serial.println(compassOK ? "OK" : "FAIL");
  if (imuOK) calibrateGyro();

  // Steering bus
  Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);
  st.pSerial = &Serial1;
  delay(300);
  updateServoHealth(true);
  Serial.print("# ST-3009 ID1 "); Serial.print(servo1OK ? "OK" : "FAIL");
  Serial.print("  ID2 "); Serial.println(servo2OK ? "OK" : "FAIL");
  enableSteeringTorque();
  steerDemand = 0;
  serviceServos(true);

  // SD
  sdOK = SD.begin(BUILTIN_SDCARD);
  Serial.print("# SD "); Serial.println(sdOK ? "OK" : "FAIL");

  // Resume after an in-flight reset?
  ResumeRecord r;
  if (loadResume(r)) {
    TARGET_LAT = r.targetLat; TARGET_LON = r.targetLon; TARGET_ELEV_OFFSET_M = r.elevOffset;
    geo.setOrigin(TARGET_LAT, TARGET_LON);
    groundPressurePa = r.groundPressurePa; gpsAltAtArm = r.gpsAltAtArm; groundSet = true;
    openLog();
    Serial.print("# RESUMED FROM EEPROM in state "); Serial.println(fsName(r.state));
    Serial.println("# (send DISARM if this is a bench power-cycle, not a flight)");
    if (r.state == FS_DEPLOYING || r.state == FS_CANOPY) {
      deployFired = true;
      core.begin(coreCfg);
      core.forceGuided(tSec());       // canopy already open: go straight to guidance
      fsState = FS_CANOPY; fsEnteredMs = millis();
    } else if (r.state == FS_DROGUE || r.state == FS_ASCENT) {
      hMax = PARAFOIL_DEPLOY_HEIGHT_M + 100.0f;   // apogee already passed or unknown: wait for deploy height
      apogeeSeen = true; apogeeMs = millis();
      fsState = FS_DROGUE; fsEnteredMs = millis();
      deployLock();
    } else {
      fsState = FS_ARMED; fsEnteredMs = millis();
      deployLock();
    }
  }

#if HAVE_WDT
  WDT_timings_t cfg;
  cfg.trigger = 1;      // seconds (Watchdog_t4 units are SECONDS)
  cfg.timeout = 2;      // seconds
  cfg.callback = wdtWarning;
  wdt.begin(cfg);
  Serial.println("# watchdog 2 s");
#else
  Serial.println("# WARNING: Watchdog_t4 not installed, no watchdog");
#endif

  gpsRateWindowMs = millis();
  printHelp();
}

void loop() {
#if HAVE_WDT
  wdt.feed();
#endif
  serviceCommands();
  serviceGps();
  serviceImu();
  serviceBaro();
  serviceCompass();
  serviceSim();
  serviceWalk();
  serviceFlightLogic();
  serviceControl();
  serviceServos(false);
  updateServoHealth(false);
  serviceTelemetry();
}
