# Bench and ground test procedure

Run in this order. Serial Monitor at 115200 baud, line ending **Newline**.
Type `TLM OFF` first to stop the 4 Hz `GUIDE` lines while testing.
Results recorded on 5 October 2026 are given under each step.

## 1. Compile, upload, boot
Keep the CanSat still for 2 s after power-up (gyro calibration). Expect
`IMU OK`, `BMP581 OK`, `QMC5883 OK`, `ST-3009 ID1 OK ID2 OK`.

*Result: all sensors and both servos detected; gyro bias 0.32 °/s. SD and
watchdog not yet installed.* **Pass**

## 2. Steering servos (brake lines off)
`PING`, `LEFT 300`, `CENTER`, `RIGHT 300`, `CENTER`. Only ID 1 moves for
LEFT, only ID 2 for RIGHT. `PING` prints commanded vs real position, supply
voltage, temperature and missed ACKs. `BUSTEST` replays the original
stand-alone servo test from inside the firmware.

*Result: supply 7.6 / 7.7 V; correct servo moves for each command.* **Pass**
(after changing servo speed 2400 → 500, enabling torque at boot and adding
ACK checking with resend).

## 3. Gyro direction and scale
`GYROTEST`, then turn the CanSat clockwise (seen from above) by exactly 180°
using a table edge as reference. The heading must increase to about 180.

*Result: increased, read 180.* **Pass**

## 4. Barometer
`BARO`, `ARM`, `BARO` on the table, lift about 1 m and hold 5 s, `BARO`,
put back, `BARO`, `DISARM`.

*Result: 94 938 Pa, 22 °C; ±0.15 m noise at rest; +0.6 m when lifted
(7.5 Pa), back to −0.14 m.* **Pass**

## 5. Simulated descent on the real servos
`SIM 400 300 500 2 1` (start 400 m N, 300 m E of target, 500 m height,
wind 2 m/s N + 1 m/s E). About 3.5 minutes real time. Check phases with
`STATUS`.

*Result: full HOMING → LOITER → FINAL → TERMINAL sequence, no missed ACKs,
miss 13.9 m.* **Pass**

## 6. GPS outdoors
Open sky, wait 2 to 3 minutes, `STATUS`. Need at least 5 satellites, HDOP
below 4, rate about 5 Hz.

*Result: 12 satellites, HDOP 0.70, 5.0 Hz (after the once-per-epoch fix).*
**Pass**

## 7. Walking test
`TARGETHERE` at the chosen target, walk about 150 m away, `WALK`, walk back
carrying the CanSat upright and facing forward. Drift so the target is on
the right: ID 2 pulls. On the left: ID 1 pulls. Inside about 100 m it
switches to LOITER and asks for a clockwise circle. `STOP` to finish.

*Result: correct servo for each side; LOITER engaged at 29 m; GPS 5.0 Hz,
HDOP 0.48.* **Pass**

## 8. SD logging and watchdog
FAT32 microSD (32 GB or less) in the Teensy slot; install WDT_T4. Boot must
show `# SD OK` and `# watchdog 2 s`. `ARM`, wait 15 s, `DISARM`; check that
`FLT000.CSV` has a header and about 150 `GUIDE` rows.

*Result: pending.*

## Next: drop tests (parafoil attached)
1. Steering sign: fixed `LEFT` / `RIGHT` demands; confirm the turn direction.
2. Characterisation: straight segments for airspeed, sink and glide ratio;
   step demands of 100 to 450 counts for turn rate per count, lag and slack.
3. Set `SLACK`, rerun the Monte-Carlo with the measured ranges, retune.
4. Guided drops from increasing heights with a ground target.
