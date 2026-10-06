/*
  ============================================================================
  Monte-Carlo test harness for ParafoilGuidance.h (runs on a PC, not Teensy)
  ============================================================================
  Build:   g++ -O2 -std=c++14 -o mc monte_carlo.cpp
  Run:     ./mc [runs] [gpsHz] [seed] [trajDir]
           ./mc 2000 5 1            -> 2000 random flights, 5 Hz GPS
           ./mc 2000 1 1            -> same flights with 1 Hz GPS
           ./mc 6 5 7 traj          -> also writes traj/run_<i>.csv for plotting

  Every flight draws a different parafoil (airspeed, sink, turn response,
  brake slack, left/right asymmetry), different wind (speed, direction,
  shear, veer, gusts), different sensor errors (GPS noise and latency, baro
  ground-reference bias, gyro bias), a different deployment point, and a
  10 s GPS dropout in 20 % of flights. The controller knows NONE of the true
  values: it starts from defaultVa = 6 m/s, defaultVz = 2.5 m/s and measures
  the rest in flight, exactly as the firmware does.
  ============================================================================
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <vector>
#include "../ParafoilGuidance.h"

using namespace pg;

struct RunResult {
  float miss;
  float vz, turnGain, deadzone, tau, rateMax, finalExcess, finalWN, finalWE, trueWN, trueWE;
  int lastMode, lastPhase;
  float windRef;
  float va;
  bool  unreachable;   // straight glide from deployment cannot even reach the target
  float margin;        // m of spare air path at deployment (truth, approximate)
  bool  dropout;
  float deployDist;
};

static RunResult runOne(Rng &rng, float gpsHz, FILE *traj) {
  // ----------------------------- truth draw -----------------------------
  SimParams sp;
  sp.va            = rng.uniform(5.0f, 8.0f);
  sp.vz            = rng.uniform(1.8f, 3.5f);
  sp.turnGain      = rng.uniform(0.035f, 0.09f);
  sp.rightLeftRatio = rng.uniform(0.8f, 1.25f);
  sp.tau           = rng.uniform(0.3f, 1.2f);
  sp.deadzone      = rng.uniform(0.0f, 80.0f);
  sp.rateMax       = rng.uniform(25.0f, 45.0f);
  sp.sinkTurnFactor = rng.uniform(0.2f, 0.8f);
  const float wRef = rng.uniform(0.0f, 6.0f);
  const float wDir = rng.uniform(0.0f, 360.0f);
  sp.windN = wRef * cosf(wDir * kDeg2Rad);
  sp.windE = wRef * sinf(wDir * kDeg2Rad);
  sp.hRef = 500.0f;
  sp.shearExp = rng.uniform(0.08f, 0.25f);
  sp.veerDeg = rng.uniform(-25.0f, 25.0f);
  sp.gustSigma = rng.uniform(0.2f, 1.0f);
  sp.gustTau = 4.0f;

  const float h0 = rng.uniform(470.0f, 530.0f);
  const float r0 = rng.uniform(0.0f, 900.0f);
  const float a0 = rng.uniform(0.0f, 360.0f);
  SimVehicle veh;
  veh.reset(sp, r0 * cosf(a0 * kDeg2Rad), r0 * sinf(a0 * kDeg2Rad), h0, rng.uniform(0.0f, 360.0f));

  // Sensor error draws
  const float baroSpan  = getenv("MC_BARO") ? (float)atof(getenv("MC_BARO")) : 3.0f;
  const float baroBias  = rng.uniform(-baroSpan, baroSpan); // ground reference error, m
  const float gyroBias  = rng.uniform(-0.6f, 0.6f);      // residual after pad calibration, deg/s
  const float gpsLat    = 0.15f;                         // s
  float gpsWalkN = 0, gpsWalkE = 0;
  const bool  dropout   = rng.uniform() < 0.2f;
  const float dropStart = rng.uniform(20.0f, 120.0f);

  // ----------------------------- controller -----------------------------
  CoreConfig cc;   // defaults = firmware defaults
  // Optional overrides for tuning sweeps (env vars); unset = firmware defaults.
  if (getenv("MC_KPSI"))  cc.steer.kPsi = (float)atof(getenv("MC_KPSI"));
  if (getenv("MC_KR"))    cc.steer.kR   = (float)atof(getenv("MC_KR"));
  if (getenv("MC_KI"))    cc.steer.kI   = (float)atof(getenv("MC_KI"));
  if (getenv("MC_KFF"))   cc.steer.kFF  = (float)atof(getenv("MC_KFF"));
  if (getenv("MC_RCMAX")) cc.steer.rateCmdMax = (float)atof(getenv("MC_RCMAX"));
  if (getenv("MC_R"))     cc.guide.loiterRadius = (float)atof(getenv("MC_R"));
  if (getenv("MC_EXTRA")) cc.guide.finalExitExtra = (float)atof(getenv("MC_EXTRA"));
  if (getenv("MC_OGAIN")) cc.guide.orbitGain = (float)atof(getenv("MC_OGAIN"));
  if (getenv("MC_IDMARGIN")) { cc.windIdTurnEnabled = true; cc.windIdMinMargin = (float)atof(getenv("MC_IDMARGIN")); }
  if (getenv("MC_OLD")) {
    // Emulate the previous "point at the target" controller for comparison:
    // no wind model, no loiter, no final/energy logic, just bearing pursuit.
    cc.guide.loiterEntryFactor = 0.0f;
    cc.guide.finalExitExtra = -1e9f;
    cc.guide.maxWind = 0.0f;
    cc.windIdMinMargin = 1e9f;
  }
  FlightCore core;
  core.begin(cc);
  core.startDescent(0.0f);

  const float dt = 0.01f;
  float t = 0.0f, nextBaro = 0.0f, nextGps = 0.0f, nextCtl = 0.0f, nextLog = 0.0f;
  float u = 0.0f;

  // GPS latency buffer (truth snapshots)
  struct Snap { float t, n, e, vn, ve; };
  std::vector<Snap> hist;

  while (veh.h > 0.0f && t < 1500.0f) {
    veh.step(u, dt, rng);
    t += dt;
    hist.push_back({t, veh.n, veh.e, veh.vgN, veh.vgE});
    if (hist.size() > 200) hist.erase(hist.begin());

    // IMU every step (100 Hz)
    core.onImu(veh.r + gyroBias + 0.5f * rng.gauss(), dt);

    // Baro 25 Hz
    if (t >= nextBaro) {
      nextBaro += 0.04f;
      core.onBaro(veh.h + baroBias + 0.25f * rng.gauss(), 0.04f);
    }

    // GPS
    if (t >= nextGps) {
      nextGps += 1.0f / gpsHz;
      const bool inDrop = dropout && t > dropStart && t < dropStart + 10.0f;
      if (!inDrop) {
        // pick the snapshot gpsLat seconds old
        Snap s = hist.front();
        for (size_t i = hist.size(); i-- > 0;) {
          if (hist[i].t <= t - gpsLat) { s = hist[i]; break; }
        }
        gpsWalkN += 0.15f * rng.gauss(); gpsWalkN *= 0.995f;
        gpsWalkE += 0.15f * rng.gauss(); gpsWalkE *= 0.995f;
        core.onGps(t, true,
                   s.n + gpsWalkN + 0.8f * rng.gauss(),
                   s.e + gpsWalkE + 0.8f * rng.gauss(),
                   s.vn + 0.15f * rng.gauss(),
                   s.ve + 0.15f * rng.gauss());
      }
    }

    // Control 50 Hz
    if (t >= nextCtl) {
      nextCtl += 0.02f;
      u = core.step(t, 0.02f);
    }

    if (traj && t >= nextLog) {
      nextLog += 0.5f;
      fprintf(traj, "%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%.2f,%.2f,%.1f,%.1f,%.1f\n",
              t, veh.n, veh.e, veh.h, veh.psi, u, core.coreMode(), core.gout.phase,
              core.gout.vtN, core.gout.vtE, core.gout.excess, core.wind.wN, core.wind.wE);
    }
  }

  float wN0, wE0;
  veh.windAt(h0, wN0, wE0);
  RunResult rr;
  rr.miss = hyp(veh.n, veh.e);
  rr.windRef = wRef;
  rr.va = sp.va;
  {
    // Truth reachability: mean wind over the descent, straight glide after damping.
    float wmN = 0, wmE = 0;
    for (int k = 0; k < 20; k++) { float a, b; veh.windAt(h0 * (k + 0.5f) / 20.0f, a, b); wmN += a / 20; wmE += b / 20; }
    const float tAvail = (h0 - sp.vz * cc.dampingTime) / sp.vz;
    const float n0 = r0 * cosf(a0 * kDeg2Rad), e0 = r0 * sinf(a0 * kDeg2Rad);
    rr.margin = sp.va * tAvail - hyp(n0 + wmN * tAvail, e0 + wmE * tAvail);
    rr.unreachable = rr.margin < 0.0f;
  }
  rr.dropout = dropout;
  rr.deployDist = r0;
  rr.vz = sp.vz; rr.turnGain = sp.turnGain; rr.deadzone = sp.deadzone; rr.tau = sp.tau; rr.rateMax = sp.rateMax;
  rr.finalExcess = core.gout.excess; rr.finalWN = core.wind.wN; rr.finalWE = core.wind.wE;
  rr.trueWN = wN0; rr.trueWE = wE0; rr.lastMode = core.coreMode(); rr.lastPhase = core.gout.phase;
  return rr;
}

static float pct(std::vector<float> v, float p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t k = (size_t)(p * (v.size() - 1));
  return v[k];
}

int main(int argc, char **argv) {
  const int   runs  = argc > 1 ? atoi(argv[1]) : 1000;
  const float gpsHz = argc > 2 ? (float)atof(argv[2]) : 5.0f;
  const uint32_t seed = argc > 3 ? (uint32_t)atoi(argv[3]) : 1;
  const char *trajDir = argc > 4 ? argv[4] : nullptr;

  Rng rng;
  rng.seed(seed * 2654435761u);
  std::vector<float> all, reach, drop, hiWind, comfy, marg;
  int hit80 = 0, hit50 = 0, hit80r = 0, nReach = 0;

  for (int i = 0; i < runs; i++) {
    FILE *tf = nullptr;
    if (getenv("MC_TRAJ_RUN") && atoi(getenv("MC_TRAJ_RUN")) == i) {
      tf = fopen("traj_single.csv", "w");
      if (tf) fprintf(tf, "t,n,e,h,psi,u,mode,phase,vtN,vtE,excess,wN,wE\n");
    } else if (trajDir) {
      char fn[256];
      snprintf(fn, sizeof(fn), "%s/run_%d.csv", trajDir, i);
      tf = fopen(fn, "w");
      if (tf) fprintf(tf, "t,n,e,h,psi,u,mode,phase,vtN,vtE,excess,wN,wE\n");
    }
    rng.seed((seed * 7919u + (uint32_t)i) * 2654435761u);   // per-flight seed: runs are paired across code changes
    RunResult r = runOne(rng, gpsHz, tf);
    if (tf) fclose(tf);
    all.push_back(r.miss);
    if (getenv("MC_MISSFILE")) {
      static FILE *mf = fopen(getenv("MC_MISSFILE"), "w");
      if (mf) { fprintf(mf, "%.2f,%.1f,%d\n", r.miss, r.margin, r.unreachable ? 1 : 0); fflush(mf); }
    }
    if (r.miss <= 80.0f) hit80++;
    if (r.miss <= 50.0f) hit50++;
    if (!r.unreachable) { nReach++; reach.push_back(r.miss); if (r.miss <= 80.0f) hit80r++; }
    if (r.dropout) drop.push_back(r.miss);
    if (r.margin > 150.0f) comfy.push_back(r.miss);
    else if (r.margin >= 0.0f) marg.push_back(r.miss);
    if (r.windRef >= 4.0f) hiWind.push_back(r.miss);
    if (getenv("MC_FAIL") && r.miss > 80.0f)
      printf("FAIL %d miss %.0f wind %.1f va %.1f vz %.1f G %.3f dz %.0f tau %.2f rmax %.0f deploy %.0f drop %d margin %.0f | est W %.1f,%.1f true %.1f,%.1f exc %.0f mode %d ph %d\n",
             i, r.miss, r.windRef, r.va, r.vz, r.turnGain, r.deadzone, r.tau, r.rateMax, r.deployDist, r.dropout, r.margin,
             r.finalWN, r.finalWE, r.trueWN, r.trueWE, r.finalExcess, r.lastMode, r.lastPhase);
    if (trajDir) printf("run %d: miss %.1f m (wind %.1f m/s, Va %.1f, deploy %.0f m out)\n",
                        i, r.miss, r.windRef, r.va, r.deployDist);
  }

  printf("\n==== %d flights, GPS %.0f Hz ====\n", runs, gpsHz);
  printf("within 80 m        : %5.1f %%\n", 100.0f * hit80 / runs);
  printf("within 50 m        : %5.1f %%\n", 100.0f * hit50 / runs);
  printf("median miss        : %6.1f m\n", pct(all, 0.5f));
  printf("90th pct miss      : %6.1f m\n", pct(all, 0.9f));
  printf("95th pct miss      : %6.1f m\n", pct(all, 0.95f));
  printf("worst miss         : %6.1f m\n", pct(all, 1.0f));
  printf("reachable flights  : %d  (within 80 m: %.1f %%)\n", nReach,
         nReach ? 100.0f * hit80r / nReach : 0.0f);
  {
    int h = 0; for (float m : marg) if (m <= 80.0f) h++;
    printf("0 < margin <= 150  : %zu  (within 80 m: %.1f %%, median %.1f m)\n", marg.size(),
           marg.empty() ? 0.0f : 100.0f * h / marg.size(), pct(marg, 0.5f));
  }
  {
    int h = 0; for (float m : comfy) if (m <= 80.0f) h++;
    printf("margin > 150 m     : %zu  (within 80 m: %.1f %%, median %.1f m, 95th %.1f m)\n", comfy.size(),
           comfy.empty() ? 0.0f : 100.0f * h / comfy.size(), pct(comfy, 0.5f), pct(comfy, 0.95f));
  }
  printf("GPS-dropout flights: %zu  (median %.1f m, 95th %.1f m)\n", drop.size(),
         pct(drop, 0.5f), pct(drop, 0.95f));
  printf("wind >= 4 m/s      : %zu  (median %.1f m, 95th %.1f m)\n", hiWind.size(),
         pct(hiWind, 0.5f), pct(hiWind, 0.95f));
  return 0;
}
