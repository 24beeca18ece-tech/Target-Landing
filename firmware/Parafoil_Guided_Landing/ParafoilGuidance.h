/*
  ============================================================================
  ParafoilGuidance.h
  Guidance core for the autonomous parafoil targeted landing (CanSat).
  ============================================================================

  Pure C++ with no Arduino dependencies. The SAME file is compiled by:
    - the Teensy 4.1 flight firmware (Parafoil_Guided_Landing.ino)
    - the PC Monte-Carlo test harness (host_sim/monte_carlo.cpp)
  so the guidance code that is tested on the PC is byte-for-byte the code
  that flies.

  CONVENTIONS (used everywhere in this file)
    Local frame   : North / East metres, ORIGIN AT THE TARGET.
    Headings      : degrees, 0 = North, positive clockwise (90 = East).
    Steering u    : servo counts, POSITIVE = TURN RIGHT (clockwise).
    Height h      : metres above the target's ground level (AGL).
    Sink rate vz  : m/s, POSITIVE = going down.

  THE IDEA IN ONE PARAGRAPH
    A parafoil cannot stop or slow down, it only has a fixed glide. So the
    problem is not "point at the target", it is "arrive at the target at the
    exact moment the altitude runs out". All the wind does is carry the whole
    air mass (and us in it) downwind. So we pick the point in the AIR MASS
    that the wind will carry onto the target exactly at touchdown:
        VT = target - wind * t_go          (t_go = h / vz)
    VT is fixed relative to the air, so in the air mass the problem becomes
    "no wind". We know the air path we still have, L = Va * t_go, and the
    straight distance to VT, d. The difference (L - d) is EXCESS ENERGY in
    metres of air path that must be burned before landing.
      HOMING  : far away -> fly straight at VT.
      LOITER  : near VT with lots of excess -> orbit VT and burn altitude.
      FINAL   : excess almost used up -> leave the orbit, fly at VT and burn
                the last few metres with S-turns.
      TERMINAL: below a few metres -> wings level, no turning near ground.
    Wind is measured in flight from GPS ground velocity during turns (the
    velocity vectors trace a circle whose centre IS the wind and whose radius
    IS the airspeed), so nothing has to be assumed beforehand.
  ============================================================================
*/
#pragma once

#include <math.h>
#include <stdint.h>

namespace pg {

// ---------------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------------
static const float kPi      = 3.14159265358979f;
static const float kDeg2Rad = kPi / 180.0f;
static const float kRad2Deg = 180.0f / kPi;

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float wrap360(float a) { a = fmodf(a, 360.0f); if (a < 0.0f) a += 360.0f; return a; }
inline float wrap180(float a) { a = fmodf(a + 180.0f, 360.0f); if (a < 0.0f) a += 360.0f; return a - 180.0f; }
inline float bearingDeg(float dN, float dE) { return wrap360(atan2f(dE, dN) * kRad2Deg); }
inline float hyp(float a, float b) { return sqrtf(a * a + b * b); }
// First-order low-pass blend factor for a step dt and time constant tau.
inline float lpfAlpha(float dt, float tau) { return (tau <= 0.0f) ? 1.0f : clampf(dt / (tau + dt), 0.0f, 1.0f); }

// ---------------------------------------------------------------------------
// Lat/Lon -> local North/East metres around the target.
// Equirectangular projection; error < 0.1 % over a few km, far below GPS noise.
// ---------------------------------------------------------------------------
struct GeoRef {
  double lat0 = 0.0, lon0 = 0.0;
  double mPerDegLat = 110900.0, mPerDegLon = 93500.0;
  bool   isSet = false;

  void setOrigin(double lat, double lon) {
    lat0 = lat;
    lon0 = lon;
    const double phi = lat * 3.14159265358979323846 / 180.0;
    mPerDegLat = 111132.92 - 559.82 * cos(2.0 * phi) + 1.175 * cos(4.0 * phi);
    mPerDegLon = 111412.84 * cos(phi) - 93.5 * cos(3.0 * phi);
    isSet = true;
  }
  void toNE(double lat, double lon, float &n, float &e) const {
    n = (float)((lat - lat0) * mPerDegLat);
    e = (float)((lon - lon0) * mPerDegLon);
  }
};

// ---------------------------------------------------------------------------
// Alpha-beta filter on barometric height -> smooth height and vertical speed.
// ---------------------------------------------------------------------------
struct AltBetaFilter {
  float h = 0.0f;       // m
  float v = 0.0f;       // m/s, positive UP
  float alpha = 0.12f;
  float beta  = 0.008f;
  bool  init  = false;

  void reset(float h0) { h = h0; v = 0.0f; init = true; }
  void update(float meas, float dt) {
    if (!isfinite(meas)) return;
    if (!init) { reset(meas); return; }
    if (dt <= 0.0f || dt > 0.5f) { h = meas; return; }   // gap: re-seed height, keep velocity
    const float hp = h + v * dt;
    const float r  = meas - hp;
    h = hp + alpha * r;
    v = v + (beta / dt) * r;
  }
};

// ---------------------------------------------------------------------------
// Heading filter: gyro yaw rate propagates, GPS-derived air heading corrects.
// Output is the AIR heading (where the canopy points), not the ground track.
// ---------------------------------------------------------------------------
struct HeadingFilter {
  float psi  = 0.0f;   // deg
  bool  init = false;

  void propagate(float rateDegS, float dt) {
    if (init && dt > 0.0f && dt < 0.5f) psi = wrap360(psi + rateDegS * dt);
  }
  void correct(float measDeg, float gain) {
    if (!isfinite(measDeg)) return;
    if (!init) { psi = wrap360(measDeg); init = true; return; }
    psi = wrap360(psi + clampf(gain, 0.0f, 1.0f) * wrap180(measDeg - psi));
  }
};

// ---------------------------------------------------------------------------
// Wind + airspeed estimator.
// Ground velocity = wind + airspeed * (cos psi, sin psi). While the canopy
// turns, the (vN, vE) samples lie on a circle: centre = wind, radius = Va.
// Algebraic least-squares circle fit over a sliding window. No compass and no
// assumed airspeed needed.
// ---------------------------------------------------------------------------
class WindEstimator {
 public:
  static const int kN = 160;          // sliding window length (samples)

  // Outputs
  float    wN = 0.0f, wE = 0.0f;      // m/s, direction the air is MOVING TO
  float    va = 0.0f;                 // m/s, airspeed from the fit radius
  float    rms = 0.0f;                // m/s, fit residual
  int      coverage = 0;              // occupied 30-degree bins around the circle
  bool     valid = false;
  uint32_t fits = 0;                  // accepted fits since reset

  // Acceptance limits
  float maxWind = 12.0f;
  float minVa   = 2.5f;
  float maxVa   = 15.0f;
  float maxRms  = 1.2f;
  int   minCoverageBins   = 7;        // 7 x 30 deg: >= 210 deg of turn in the window
  int   firstCoverageBins = 9;        // first fit needs >= 270 deg of turn
  float maxVaJump = 2.5f;             // m/s, reject fits whose airspeed jumps more than this
  float blend   = 0.3f;               // new-fit weight once a fit exists

  void reset() {
    count = 0; head = 0; valid = false; fits = 0;
    wN = wE = va = rms = 0.0f; coverage = 0;
  }
  // psiRel: gyro-integrated heading (deg, any zero). Used only to measure how
  // much of a turn the window really contains; a short noisy arc can otherwise
  // be fitted by a small wrong circle that "looks" well covered.
  void addSample(float vN, float vE, float psiRel) {
    if (!isfinite(vN) || !isfinite(vE) || !isfinite(psiRel)) return;
    bufN[head] = vN;
    bufE[head] = vE;
    bufPsi[head] = wrap360(psiRel);
    head = (head + 1) % kN;
    if (count < kN) count++;
  }
  int samples() const { return count; }

  bool solve() {
    if (count < 30) return false;

    // Centre the data first for numerical conditioning.
    double mx = 0.0, my = 0.0;
    for (int i = 0; i < count; i++) { mx += bufN[i]; my += bufE[i]; }
    mx /= count; my /= count;

    double Sxx = 0, Syy = 0, Sxy = 0, Sxz = 0, Syz = 0, Sz = 0;
    for (int i = 0; i < count; i++) {
      const double x = bufN[i] - mx, y = bufE[i] - my, z = x * x + y * y;
      Sxx += x * x; Syy += y * y; Sxy += x * y;
      Sxz += x * z; Syz += y * z; Sz += z;
    }
    // Circle: x^2 + y^2 + a x + b y + c = 0
    const double det = Sxx * Syy - Sxy * Sxy;
    if (det <= 1e-3 * (Sxx * Syy) || det <= 1e-9) return false;   // samples on a line
    const double a = (-Sxz * Syy + Syz * Sxy) / det;
    const double b = (-Syz * Sxx + Sxz * Sxy) / det;
    const double c = -Sz / count;
    const double r2 = 0.25 * (a * a + b * b) - c;
    if (r2 <= 0.0) return false;
    const double cx = mx - 0.5 * a, cy = my - 0.5 * b, r = sqrt(r2);

    // Residual about the fitted circle, and turn coverage from the gyro heading.
    int bins[12] = {0};
    double ss = 0.0;
    for (int i = 0; i < count; i++) {
      const double dx = bufN[i] - cx, dy = bufE[i] - cy;
      const double rr = sqrt(dx * dx + dy * dy);
      ss += (rr - r) * (rr - r);
      int bi = (int)(bufPsi[i] / 30.0f);
      if (bi < 0) bi = 0;
      if (bi > 11) bi = 11;
      bins[bi]++;
    }
    int cov = 0;
    for (int i = 0; i < 12; i++) if (bins[i] >= 2) cov++;
    const float fitRms = (float)sqrt(ss / count);
    const float fw = (float)sqrt(cx * cx + cy * cy);

    rms = fitRms;
    coverage = cov;
    if (cov < (valid ? minCoverageBins : firstCoverageBins)) return false;
    if (fitRms > maxRms) return false;
    if (r < minVa || r > maxVa) return false;
    if (fw > maxWind) return false;
    if (valid && fabsf((float)r - va) > maxVaJump) return false;

    if (!valid) {
      wN = (float)cx; wE = (float)cy; va = (float)r;
    } else {
      wN += blend * ((float)cx - wN);
      wE += blend * ((float)cy - wE);
      va += blend * ((float)r - va);
    }
    valid = true;
    fits++;
    return true;
  }

 private:
  float bufN[kN];
  float bufE[kN];
  float bufPsi[kN];
  int   count = 0, head = 0;
};

// ---------------------------------------------------------------------------
// Outer loop: where should the canopy point?
// ---------------------------------------------------------------------------
enum Phase : uint8_t { PH_HOMING = 0, PH_LOITER = 1, PH_FINAL = 2, PH_TERMINAL = 3 };

inline const char *phaseName(uint8_t p) {
  switch (p) {
    case PH_HOMING:   return "HOMING";
    case PH_LOITER:   return "LOITER";
    case PH_FINAL:    return "FINAL";
    case PH_TERMINAL: return "TERMINAL";
  }
  return "?";
}

struct GuidanceConfig {
  float loiterRadius        = 50.0f;   // m, orbit radius around VT (must exceed the canopy's turn radius)
  int   loiterDir           = +1;      // +1 = clockwise (right turns), -1 = counter-clockwise
  float orbitGain           = 1.5f;    // vector-field convergence gain onto the orbit
  float loiterEntryFactor   = 2.0f;    // HOMING -> LOITER when d < factor * radius
  float loiterLeaveFactor   = 5.0f;    // LOITER -> HOMING if d grows past factor * radius
  float finalExitExtra      = 30.0f;   // m of air path still in hand when the orbit is left
  float finalReturnToLoiter = 150.0f;  // FINAL -> LOITER if excess climbs back above this
  float sTurnMaxDeg         = 55.0f;   // max heading offset used to burn energy on final
  float sTurnXtrackLimit    = 20.0f;   // m, S-turn reverses when cross-track exceeds this
  float finalNoTurnDist     = 12.0f;   // m, inside this distance of VT stop chasing it
  float terminalHeight      = 8.0f;    // m AGL, below this: wings level
  float maxWind             = 12.0f;   // m/s, clamp on the wind used for VT
  float minVz = 0.8f, maxVz = 8.0f;    // m/s, clamps on sink rate
  float minVa = 2.5f, maxVa = 15.0f;   // m/s, clamps on airspeed
};

struct GuidanceInput {
  float pN = 0, pE = 0;     // m, vehicle position relative to target
  float h  = 0;             // m AGL
  float vz = 0;             // m/s sink (straight-flight value preferred)
  float va = 0;             // m/s airspeed
  float wN = 0, wE = 0;     // m/s wind
  bool  windValid = false;
  float psiAir = 0;         // deg, current air heading estimate
};

struct GuidanceOutput {
  float   psiCmd   = 0;     // deg, commanded air heading
  bool    steer    = false; // false = wings level
  uint8_t phase    = PH_HOMING;
  float   tgo      = 0;     // s
  float   L        = 0;     // m, air path remaining
  float   d        = 0;     // m, distance to VT
  float   excess   = 0;     // m, L - d
  float   vtN = 0, vtE = 0; // m, virtual target
  float   predMiss = 0;     // m, miss if we flew straight on the current heading from now
  float   rateFF   = 0;     // deg/s, feed-forward turn rate (orbit), + = right
  int8_t  sSign    = 0;
};

class Guidance {
 public:
  GuidanceConfig cfg;

  void reset() { phase = PH_HOMING; sSign = +1; finalRef = 0.0f; }
  uint8_t currentPhase() const { return phase; }

  GuidanceOutput update(const GuidanceInput &in) {
    GuidanceOutput o;
    const float vz = clampf(in.vz, cfg.minVz, cfg.maxVz);
    const float va = clampf(in.va, cfg.minVa, cfg.maxVa);
    const float h  = in.h > 0.0f ? in.h : 0.0f;
    const float tgo = h / vz;

    float wN = 0.0f, wE = 0.0f;
    if (in.windValid) {
      wN = in.wN; wE = in.wE;
      const float wm = hyp(wN, wE);
      if (wm > cfg.maxWind && wm > 0.0f) { wN *= cfg.maxWind / wm; wE *= cfg.maxWind / wm; }
    }

    // Virtual target: the air-mass point the wind will carry onto the target at touchdown.
    const float vtN = -wN * tgo, vtE = -wE * tgo;
    const float dN = vtN - in.pN, dE = vtE - in.pE;
    const float d = hyp(dN, dE);
    const float L = va * tgo;
    const float excess = L - d;
    const float brgVT = bearingDeg(dN, dE);

    // ---------------- phase transitions ----------------
    if (phase != PH_TERMINAL && in.h <= cfg.terminalHeight) phase = PH_TERMINAL;

    switch (phase) {
      case PH_HOMING:
        if (excess <= cfg.finalExitExtra) enterFinal(brgVT);
        else if (d <= cfg.loiterEntryFactor * cfg.loiterRadius) phase = PH_LOITER;
        break;
      case PH_LOITER:
        if (excess <= cfg.finalExitExtra) enterFinal(brgVT);
        else if (d > cfg.loiterLeaveFactor * cfg.loiterRadius) phase = PH_HOMING;
        break;
      case PH_FINAL:
        if (excess > cfg.finalReturnToLoiter) {
          phase = (d <= cfg.loiterEntryFactor * cfg.loiterRadius) ? PH_LOITER : PH_HOMING;
        }
        break;
      default:
        break;
    }

    // ---------------- heading command ----------------
    float psiCmd = in.psiAir;
    float rateFF = 0.0f;
    bool steer = true;

    if (phase == PH_HOMING) {
      psiCmd = brgVT;
    } else if (phase == PH_LOITER) {
      // Vector field onto a circle of radius R about VT.
      const float rN = -dN, rE = -dE;                    // VT -> vehicle
      const float gamma = bearingDeg(rN, rE);
      const float R = cfg.loiterRadius;
      const float corr = atanf(cfg.orbitGain * (d - R) / R) * kRad2Deg;   // >0 outside, pulls inward
      psiCmd = wrap360(gamma + (float)cfg.loiterDir * (90.0f + corr));
      // Steady turn rate needed to fly this circle in still air: Va / R.
      rateFF = (float)cfg.loiterDir * (va / R) * kRad2Deg * clampf(R / (d > 1.0f ? d : 1.0f), 0.0f, 1.0f);
    } else if (phase == PH_FINAL) {
      if (d <= cfg.finalNoTurnDist) {
        psiCmd = finalRef;                               // do not pivot over VT
      } else {
        float delta = 0.0f;
        if (excess > 0.0f && L > 1.0f) {
          delta = acosf(clampf(d / L, 0.0f, 1.0f)) * kRad2Deg;
          if (delta > cfg.sTurnMaxDeg) delta = cfg.sTurnMaxDeg;
        }
        // Cross-track from the reference final line through VT.
        const float rr = finalRef * kDeg2Rad;
        const float xt = (-dN) * (-sinf(rr)) + (-dE) * cosf(rr);  // + = right of line
        if (sSign > 0 && xt > cfg.sTurnXtrackLimit) sSign = -1;
        else if (sSign < 0 && xt < -cfg.sTurnXtrackLimit) sSign = +1;
        psiCmd = wrap360(brgVT + (float)sSign * delta);
      }
    } else {
      steer = false;
    }

    // Predicted miss if the canopy flew straight on its present heading from here.
    const float pr = in.psiAir * kDeg2Rad;
    const float landN = in.pN + (va * cosf(pr) + wN) * tgo;
    const float landE = in.pE + (va * sinf(pr) + wE) * tgo;

    o.psiCmd = psiCmd; o.steer = steer; o.phase = phase;
    o.tgo = tgo; o.L = L; o.d = d; o.excess = excess;
    o.vtN = vtN; o.vtE = vtE; o.predMiss = hyp(landN, landE); o.sSign = sSign;
    o.rateFF = rateFF;
    return o;
  }

 private:
  uint8_t phase = PH_HOMING;
  int8_t  sSign = +1;
  float   finalRef = 0.0f;

  void enterFinal(float brgVT) { phase = PH_FINAL; finalRef = brgVT; sSign = +1; }
};

// ---------------------------------------------------------------------------
// Inner loop: heading -> turn-rate command -> brake counts.
//
//   r_cmd = Kpsi * e + r_ff                 (e = heading error deg)
//   u     = Kff * r_cmd + Kr * (r_cmd - r) + I,   I += Ki * (r_cmd - r) * dt
//
// Why not the plain u = Kp*e - Kd*r from the earlier code: an orbit needs a
// STEADY turn. With u = Kp*e - Kd*r the damping term fights that steady turn,
// so the canopy can only hold the orbit with a large permanent heading error
// (tens of degrees with brake slack). Commanding a turn RATE and closing the
// loop on the gyro removes that, and the integrator soaks up the unknown
// counts-per-deg/s gain and brake-line slack.
// ---------------------------------------------------------------------------
struct SteeringConfig {
  float kPsi        = 0.6f;     // deg/s of turn rate per deg of heading error
  float rateCmdMax  = 22.0f;    // deg/s
  float kFF         = 15.0f;    // counts per deg/s (~ 1 / expected turn gain)
  float kR          = 6.0f;     // counts per deg/s of rate error
  float kI          = 8.0f;     // counts per (deg/s * s)
  float iMax        = 250.0f;   // counts
  float slackCounts = 0.0f;     // measured brake-line slack, added outside neutral
  float deadbandDeg = 2.0f;
  float maxCounts   = 450.0f;
  float slewPerSec  = 900.0f;   // counts per second
};

class SteeringController {
 public:
  SteeringConfig cfg;
  float u = 0.0f;
  float integ = 0.0f;
  float rateCmd = 0.0f;

  void reset() { u = 0.0f; integ = 0.0f; rateCmd = 0.0f; }

  // Heading hold / orbit. enable=false slews back to neutral.
  float update(float psiCmd, float psi, float yawRateDegS, float rateFF, float dt, bool enable) {
    if (!enable) {
      rateCmd = 0.0f;
      integ += (0.0f - integ) * lpfAlpha(dt, 1.0f);
      return slewTo(0.0f, dt);
    }
    float e = wrap180(psiCmd - psi);
    if (fabsf(e) < cfg.deadbandDeg) e = 0.0f;
    return rateLoop(cfg.kPsi * e + rateFF, yawRateDegS, dt);
  }

  // Pure turn-rate command (used for the wind-identification turn).
  float rate(float rateCmdDegS, float yawRateDegS, float dt) {
    return rateLoop(rateCmdDegS, yawRateDegS, dt);
  }

 private:
  float rateLoop(float rc, float r, float dt) {
    rc = clampf(rc, -cfg.rateCmdMax, cfg.rateCmdMax);
    rateCmd = rc;
    const float er = rc - r;
    // Integrate only while the output is not pinned in the same direction (anti-windup).
    const bool pinned = (u >= cfg.maxCounts && er > 0.0f) || (u <= -cfg.maxCounts && er < 0.0f);
    if (!pinned) integ = clampf(integ + cfg.kI * er * dt, -cfg.iMax, cfg.iMax);
    float target = cfg.kFF * rc + cfg.kR * er + integ;
    if (target > 1.0f) target += cfg.slackCounts;
    else if (target < -1.0f) target -= cfg.slackCounts;
    return slewTo(target, dt);
  }

  float slewTo(float target, float dt) {
    target = clampf(target, -cfg.maxCounts, cfg.maxCounts);
    const float step = cfg.slewPerSec * (dt > 0.0f ? dt : 0.0f);
    u += clampf(target - u, -step, step);
    return u;
  }
};

// ---------------------------------------------------------------------------
// FlightCore: estimators + guidance + steering orchestration for the
// canopy-open part of the flight. The firmware feeds it sensors and sends
// its output to the servos; the PC harness feeds it simulated sensors.
// ---------------------------------------------------------------------------
enum CoreMode : uint8_t { CM_IDLE = 0, CM_DAMPING = 1, CM_WINDID = 2, CM_GUIDED = 3, CM_TERMINAL = 4 };

inline const char *coreModeName(uint8_t m) {
  switch (m) {
    case CM_IDLE:     return "IDLE";
    case CM_DAMPING:  return "DAMPING";
    case CM_WINDID:   return "WINDID";
    case CM_GUIDED:   return "GUIDED";
    case CM_TERMINAL: return "TERMINAL";
  }
  return "?";
}

struct CoreConfig {
  GuidanceConfig guide;
  SteeringConfig steer;

  float dampingTime           = 8.0f;    // s, "8-second patience" after canopy opening
  bool  windIdTurnEnabled     = false;   // dedicated wind-ID turn after damping. OFF by default:
                                         // Monte-Carlo showed it costs more altitude than it
                                         // returns; the loiter turns measure the wind for free.
  float windIdRate            = 15.0f;   // deg/s, gyro-closed right turn to measure wind
  float windIdTurnDeg         = 330.0f;  // deg of turn wanted before leaving the ID turn
  float windIdTimeout         = 45.0f;   // s
  float windIdMinHeight       = 200.0f;  // m, skip the ID turn below this
  float windIdMinMargin       = 300.0f;  // m, skip the ID turn unless the target is this comfortably in reach
  float gpsTimeout            = 1.5f;    // s without a fix -> GPS lost
  float gpsLossHoldTime       = 8.0f;    // s, hold last heading on the gyro, then fly straight
  float headingGpsGain        = 0.30f;   // per GPS sample
  float minGpsSpeedForHeading = 1.5f;    // m/s
  float gpsVelLatency         = 0.15f;   // s, GPS velocity lag compensation
  bool  useCompassFallback    = true;
  float compassGain           = 0.05f;   // per compass sample, only when GPS heading is stale
  float defaultVa             = 6.0f;    // m/s, used until measured in flight
  float defaultVz             = 2.5f;    // m/s, used until measured in flight
  float straightRate          = 6.0f;    // deg/s, below this the canopy counts as straight
  float straightHoldTime      = 2.0f;    // s straight before vz/Va samples are trusted
  float vzTau                 = 3.0f;    // s
  float vaTau                 = 4.0f;    // s
  float altAlpha              = 0.12f;
  float altBeta               = 0.008f;
  float windSolvePeriod       = 1.0f;    // s
};

class FlightCore {
 public:
  CoreConfig          cfg;
  Guidance            guidance;
  SteeringController  steering;
  WindEstimator       wind;
  HeadingFilter       heading;
  AltBetaFilter       alt;
  GuidanceOutput      gout;

  // Navigation state (public for telemetry)
  float pN = 0, pE = 0, vN = 0, vE = 0;
  bool  haveGps = false;
  float lastGpsT = -1000.0f;
  float lastHeadingFixT = -1000.0f;
  float yawRate = 0.0f;
  float gyroPsi = 0.0f;   // pure gyro integral, only for turn-coverage bookkeeping
  float h = 0.0f;
  float vzAll = 0.0f, vzStraight = 0.0f;
  bool  vzAllValid = false, vzStraightValid = false;
  float vaStraight = 0.0f;
  bool  vaStraightValid = false;
  float psiCmd = 0.0f;
  float u = 0.0f;
  bool  gpsOk = false;
  bool  holdingHeading = false;
  float lastReachMargin = 0.0f;   // m, reach margin estimated at the end of damping

  void begin(const CoreConfig &c) {
    cfg = c;
    guidance.cfg = c.guide;
    steering.cfg = c.steer;
    alt.alpha = c.altAlpha;
    alt.beta  = c.altBeta;
    wind.maxWind = c.guide.maxWind;
    reset();
  }

  void reset() {
    mode = CM_IDLE; modeT = 0.0f;
    guidance.reset(); steering.reset(); wind.reset();
    haveGps = false; lastGpsT = -1000.0f; lastHeadingFixT = -1000.0f;
    vzAllValid = vzStraightValid = vaStraightValid = false;
    straightT = 0.0f; lastSolveT = -1000.0f; newGps = false;
    psiCmd = 0.0f; u = 0.0f; holdingHeading = false; gpsOk = false;
  }

  uint8_t coreMode() const { return mode; }

  // Called at canopy line-stretch / deployment.
  void startDescent(float t) {
    mode = CM_DAMPING; modeT = t; dampSinkReset = false;
    guidance.reset(); steering.reset(); wind.reset();
    vzStraightValid = false; vaStraightValid = false;
  }
  // Skip damping/wind-ID (bench and walking tests).
  void forceGuided(float t) {
    mode = CM_GUIDED; modeT = t;
    guidance.reset(); steering.reset();
  }

  // Gyro yaw rate, deg/s, clockwise positive (already bias-corrected).
  void onImu(float yawRateDegS, float dt) {
    if (!isfinite(yawRateDegS)) return;
    yawRate = yawRateDegS;
    heading.propagate(yawRateDegS, dt);
    if (dt > 0.0f && dt < 0.5f) gyroPsi = wrap360(gyroPsi + yawRateDegS * dt);
    if (fabsf(yawRateDegS) < cfg.straightRate) straightT += dt; else straightT = 0.0f;
  }

  // Raw barometric height AGL (m). Runs the alpha-beta filter.
  void onBaro(float hRaw, float dt) {
    alt.update(hRaw, dt);
    feedHeight(alt.h, -alt.v, dt);
  }

  // Already-filtered height and sink (used by WALK test and GPS-altitude fallback).
  void onHeightDirect(float hAGL, float vzDown, float dt) { feedHeight(hAGL, vzDown, dt); }

  // One GPS solution. pN/pE relative to target (m), vN/vE ground velocity (m/s).
  void onGps(float t, bool valid, float n, float e, float velN, float velE) {
    if (!valid || !isfinite(n) || !isfinite(e) || !isfinite(velN) || !isfinite(velE)) return;
    const float dtg = (haveGps && t > lastGpsT) ? (t - lastGpsT) : 0.2f;
    pN = n; pE = e; vN = velN; vE = velE;
    haveGps = true; lastGpsT = t; newGps = true;

    const float gs = hyp(vN, vE);
    if (mode != CM_IDLE && gs > 1.0f) wind.addSample(vN, vE, gyroPsi);

    if (gs >= cfg.minGpsSpeedForHeading) {
      const float aN = vN - (wind.valid ? wind.wN : 0.0f);
      const float aE = vE - (wind.valid ? wind.wE : 0.0f);
      if (hyp(aN, aE) > 0.8f) {
        const float meas = bearingDeg(aN, aE) + yawRate * cfg.gpsVelLatency;
        heading.correct(meas, cfg.headingGpsGain);
        lastHeadingFixT = t;
      }
      if (wind.valid && straightT >= cfg.straightHoldTime) {
        const float vaMeas = hyp(aN, aE);
        if (!vaStraightValid) { vaStraight = vaMeas; vaStraightValid = true; }
        else vaStraight += lpfAlpha(dtg, cfg.vaTau) * (vaMeas - vaStraight);
      }
    }
  }

  // Optional compass heading (deg). Only used while GPS heading is stale.
  void onCompass(float t, float psiDeg) {
    if (!cfg.useCompassFallback || !isfinite(psiDeg)) return;
    if (!heading.init || (t - lastHeadingFixT) > 2.0f) heading.correct(psiDeg, cfg.compassGain);
  }

  float airspeed() const {
    if (vaStraightValid) return vaStraight;
    if (wind.valid) return wind.va;
    return cfg.defaultVa;
  }
  float sinkRate() const {
    if (vzStraightValid) return vzStraight;
    if (vzAllValid) return vzAll;
    return cfg.defaultVz;
  }

  // Run once per control tick. Returns the steering demand (counts, + = right).
  float step(float t, float dt) {
    gpsOk = haveGps && (t - lastGpsT) <= cfg.gpsTimeout;

    if ((mode == CM_WINDID || mode == CM_GUIDED) && h <= cfg.guide.terminalHeight) {
      mode = CM_TERMINAL; modeT = t;
    }

    switch (mode) {
      case CM_IDLE:
        u = steering.update(0, 0, 0, 0, dt, false);
        break;

      case CM_DAMPING:
        u = steering.update(0, 0, 0, 0, dt, false);
        // Throw away the opening-shock part of the sink history; keep the settled part.
        if (!dampSinkReset && (t - modeT) >= 0.6f * cfg.dampingTime) { vzAllValid = false; dampSinkReset = true; }
        if (t - modeT >= cfg.dampingTime) {
          // The ID turn costs altitude. Only spend it if the target is comfortably
          // reachable even with no wind help; otherwise head for the target now and
          // let the wind be measured in the loiter turns later.
          const float vzE = clampf(sinkRate(), cfg.guide.minVz, cfg.guide.maxVz);
          const float reachMargin = cfg.defaultVa * (h / vzE) - hyp(pN, pE);
          const bool doId = cfg.windIdTurnEnabled && haveGps && h > cfg.windIdMinHeight &&
                            reachMargin > cfg.windIdMinMargin;
          lastReachMargin = reachMargin;
          mode = doId ? CM_WINDID : CM_GUIDED;
          modeT = t;
          idTurned = 0.0f;
          guidance.reset();
        }
        break;

      case CM_WINDID:
        trySolveWind(t);
        u = steering.rate(cfg.windIdRate, yawRate, dt);
        idTurned += fabsf(yawRate) * dt;
        psiCmd = heading.psi;
        if ((wind.valid && idTurned >= cfg.windIdTurnDeg) || (t - modeT) > cfg.windIdTimeout ||
            h < 0.6f * cfg.windIdMinHeight) {
          mode = CM_GUIDED; modeT = t;
          guidance.reset();
        }
        break;

      case CM_GUIDED: {
        trySolveWind(t);
        bool enable = true;
        if (gpsOk) {
          holdingHeading = false;
          if (newGps) runGuidance();
          enable = gout.steer;
        } else {
          // GPS lost: keep flying the last commanded heading on the gyro for a
          // while (the plan is still valid for a few seconds), then go straight.
          holdingHeading = true;
          if (!haveGps || (t - lastGpsT) > cfg.gpsTimeout + cfg.gpsLossHoldTime) enable = false;
        }
        const float ff = (gpsOk && enable) ? gout.rateFF : 0.0f;
        u = steering.update(psiCmd, heading.psi, yawRate, ff, dt, enable && heading.init);
        break;
      }

      case CM_TERMINAL:
        u = steering.update(0, 0, 0, 0, dt, false);
        if (newGps && gpsOk) runGuidance();          // keep telemetry alive, no steering
        break;
    }
    newGps = false;
    return u;
  }

 private:
  uint8_t mode = CM_IDLE;
  float   modeT = 0.0f;
  float   straightT = 0.0f;
  float   lastSolveT = -1000.0f;
  float   idTurned = 0.0f;
  bool    dampSinkReset = false;
  bool    newGps = false;

  void feedHeight(float hAGL, float vzDown, float dt) {
    if (!isfinite(hAGL) || !isfinite(vzDown)) return;
    h = hAGL;
    if (mode == CM_IDLE) return;
    if (!vzAllValid) { vzAll = vzDown; vzAllValid = true; }
    else vzAll += lpfAlpha(dt, cfg.vzTau) * (vzDown - vzAll);
    if ((mode == CM_GUIDED || mode == CM_WINDID) && straightT >= cfg.straightHoldTime) {
      if (!vzStraightValid) { vzStraight = vzAll; vzStraightValid = true; }
      else vzStraight += lpfAlpha(dt, cfg.vzTau) * (vzDown - vzStraight);
    }
  }

  void trySolveWind(float t) {
    if (t - lastSolveT >= cfg.windSolvePeriod) {
      lastSolveT = t;
      wind.solve();
    }
  }

  void runGuidance() {
    GuidanceInput gi;
    gi.pN = pN; gi.pE = pE; gi.h = h;
    gi.vz = sinkRate(); gi.va = airspeed();
    gi.wN = wind.wN; gi.wE = wind.wE; gi.windValid = wind.valid;
    gi.psiAir = heading.psi;
    gout = guidance.update(gi);
    psiCmd = gout.psiCmd;
  }
};

// ---------------------------------------------------------------------------
// Simple parafoil + sensor model. Used by the PC Monte-Carlo harness and by
// the firmware's SIM bench mode (real servos move, simulated sky).
// ---------------------------------------------------------------------------
struct Rng {
  uint32_t s = 0x12345678u;
  void seed(uint32_t v) { s = v ? v : 0x9E3779B9u; }
  uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
  float uniform() { return (next() >> 8) * (1.0f / 16777216.0f); }
  float uniform(float a, float b) { return a + (b - a) * uniform(); }
  float gauss() {
    float u1 = uniform(); if (u1 < 1e-7f) u1 = 1e-7f;
    const float u2 = uniform();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * kPi * u2);
  }
};

struct SimParams {
  float va            = 6.0f;    // m/s airspeed
  float vz            = 2.5f;    // m/s straight-flight sink
  float turnGain      = 0.06f;   // deg/s per count of brake
  float rightLeftRatio = 1.0f;   // right-turn gain / left-turn gain
  float tau           = 0.6f;    // s, turn-rate response lag
  float deadzone      = 0.0f;    // counts of brake-line slack
  float rateMax       = 35.0f;   // deg/s
  float sinkTurnFactor = 0.4f;   // extra sink fraction at rateMax
  float windN = 0.0f, windE = 0.0f;   // m/s at hRef
  float hRef          = 500.0f;
  float shearExp      = 0.14f;   // power-law wind shear
  float veerDeg       = 0.0f;    // wind direction change from hRef to ground
  float gustSigma     = 0.0f;    // m/s
  float gustTau       = 4.0f;    // s
};

class SimVehicle {
 public:
  SimParams p;
  float n = 0, e = 0, h = 0, psi = 0, r = 0;   // truth
  float gN = 0, gE = 0;                         // gust state
  float vgN = 0, vgE = 0;                       // last ground velocity

  void reset(const SimParams &prm, float n0, float e0, float h0, float psi0) {
    p = prm; n = n0; e = e0; h = h0; psi = wrap360(psi0); r = 0; gN = gE = 0; vgN = vgE = 0;
  }

  void windAt(float hh, float &wN, float &wE) const {
    const float hc = hh > 1.0f ? hh : 1.0f;
    const float scale = powf(hc / p.hRef, p.shearExp);
    const float rot = p.veerDeg * kDeg2Rad * (1.0f - clampf(hc / p.hRef, 0.0f, 1.0f));
    const float c = cosf(rot), s = sinf(rot);
    wN = scale * (p.windN * c - p.windE * s);
    wE = scale * (p.windN * s + p.windE * c);
  }

  void step(float uCounts, float dt, Rng &rng) {
    float ue = 0.0f;
    if (fabsf(uCounts) > p.deadzone) ue = uCounts - (uCounts > 0 ? p.deadzone : -p.deadzone);
    float rss = p.turnGain * ue * (ue > 0 ? p.rightLeftRatio : 1.0f);
    rss = clampf(rss, -p.rateMax, p.rateMax);
    r += (rss - r) * clampf(dt / p.tau, 0.0f, 1.0f);
    psi = wrap360(psi + r * dt);

    if (p.gustSigma > 0.0f) {
      const float k = sqrtf(2.0f * dt / p.gustTau) * p.gustSigma;
      gN += -gN * dt / p.gustTau + k * rng.gauss();
      gE += -gE * dt / p.gustTau + k * rng.gauss();
    }
    float wN, wE;
    windAt(h, wN, wE);
    wN += gN; wE += gE;

    const float rr = r / p.rateMax;
    const float vzNow = p.vz * (1.0f + p.sinkTurnFactor * rr * rr);
    vgN = p.va * cosf(psi * kDeg2Rad) + wN;
    vgE = p.va * sinf(psi * kDeg2Rad) + wE;
    n += vgN * dt;
    e += vgE * dt;
    h -= vzNow * dt;
  }
};

}  // namespace pg
