# CanSat parafoil targeted landing: design summary

Requirement: land within 80 m of the target. Full detail in
`Parafoil_Guidance_Report.pdf`.

## Algorithm
- Virtual target VT = target − wind × t_go (t_go = h / sink). VT is fixed in the air mass, so the wind problem becomes a no-wind problem.
- Excess energy E = Va × t_go − distance to VT (metres of air path to burn).
- Phases: HOMING (fly at VT) → LOITER (50 m orbit about VT, vector field plus Va/R turn-rate feed-forward) → FINAL (leave orbit when E ≤ 30 m, S-turns burn the rest) → TERMINAL (wings level below 8 m).
- Wind and airspeed from a least-squares circle fit of GPS ground velocity during turns (no compass needed). Turn coverage judged from gyro heading, not the fitted centre (short noisy arcs gave false fits).
- Inner loop: turn-rate command closed on the gyro (r_cmd = kPsi·e + r_ff; u = kFF·r_cmd + kR·(r_cmd − r) + I). The earlier u = Kp·e − Kd·r fights the steady turn an orbit needs.
- Heading: gyro-propagated, corrected by GPS-derived air heading; compass only as fallback.
- No dedicated wind-identification turn: Monte-Carlo showed it costs more altitude than it returns.

## Verification (PC Monte-Carlo, same header file as the firmware)
Randomised canopy (Va 5 to 8 m/s, sink 1.8 to 3.5 m/s, turn gain, lag, brake slack 0 to 80 counts, left/right asymmetry), wind 0 to 6 m/s with shear, veer and gusts, GPS noise and 0.15 s latency, baro bias ±3 m, gyro bias, 20 % of flights with a 10 s GPS dropout, deployment 0 to 900 m from the target.
- Reachable flights: 98.3 % within 80 m, median 14 m (old point-at-target logic: 97.2 %, median 30 m).
- Reach margin above 150 m: 99.8 % within 80 m, 95th percentile 34 m.
- 1 Hz GPS only slightly worse (99.5 %). A ±20 m height-reference error pushes the 95th percentile to about 64 m.

## Hardware verification so far
- Compiles and runs on the Teensy 4.1. BMP581 register driver verified (chip 0x50, sensible pressure, ±0.15 m noise).
- Gyro sign and scale verified (180° turn reads 180°). GPS 5 Hz configuration verified (12 sats, HDOP 0.7).
- Servo mapping verified on the bench: LEFT moves ID 1, RIGHT moves ID 2. Firmware SIM descent on real servos: 13.9 m miss.

## Open items
- Drop tests: confirm TURN_SIGN with the canopy attached; measure Va, sink, turn rate per count and brake slack; set SLACK and GAINS.
- u-blox dynamic model and NMEA trimming keys sent but not confirmed.
- SD logging and watchdog installation pending.
- Target must be within glide reach of the deployment point; TARGET_ELEV_OFFSET_M must be known within a few metres.
