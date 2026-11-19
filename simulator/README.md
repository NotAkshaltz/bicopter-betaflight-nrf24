# Bicopter simulator

A browser-based simulator for the tilt-rotor bicopter: Raspberry Pi Pico 2 running Betaflight 2026.6.2 with the NRF24_BICOPTER v2 receiver, A2212 1000KV motors, SimonK 30A ESCs, MG90S tilt servos and a 3S pack. You can change the firmware settings, fly the aircraft, inject faults, and run automated engineering checks and flight tests.

It needs no install or internet connection. Open `index.html` in Chrome, Edge or Firefox. If your browser blocks local scripts, serve the folder instead:

```bash
python -m http.server 8765 --directory simulator
```

Then browse to http://localhost:8765.

## What it covers

| Area | What is modelled / checked |
|---|---|
| Firmware | Betaflight-style CLI (`set/get/diff/dump/aux/servo/smix reverse/mmix/rxfail/resource/save`). RX watchdog, rxfail, failsafe stage 1/2 (DROP / AUTO-LAND), arming flags, ACTUAL rates, angle mode, PID with Betaflight's P/I/D/F scaling, TPA, air mode, BI motor mixer, servo mixer (servo 4/5 min/max/mid/rate/reverse), 200 Hz motor / fixed 50 Hz servo output timing, battery monitoring, board alignment. There is also a JavaScript hook for prototyping new firmware logic. |
| Radio | A line-by-line JS port of `nrf24_bicopter_link.c`, checked against the C test vector `B10234095A…2A89`. Handset session, sample sequence, restart announcement and input faults. Receiver adapter logic from `nrf24_bicopter.c`: hop sync, freshness, arm interlock, restart → `failsafeOnReceiverReset()`. Path loss, PA/LNA saturation, Wi-Fi overlap, dropouts and packet loss. |
| Thrust vectoring | Physical nacelle response to the mixer for each axis, servo directions and mounting, and gyroscopic coupling of the tilted props. Flight tests confirm that pitch, yaw and roll move the right way. |
| Weight & balance | Component masses and positions, CG, inertia tensor, trim attitude, pitch authority (CG depth below the tilt axis), nacelle balance. |
| Propulsion | Motor/prop model, thrust-to-weight, hover point, full-throttle current against the motor, ESC and battery ratings, sag, endurance, tip speed, temperatures, ESC calibration, protocol and rate compatibility. |
| Power & electronics | Wire gauge and connector ampacity, the Pico 5 V rail (ESC linear BEC dissipation and thermal shutdown), servo UBEC, Pico 3V3 budget, radio LDO headroom and TX-peak dip, MPU6500 supply, common ground, VBAT divider and `vbat_scale`, brown-out and reboot. |
| Pins & buses | Pin conflicts, Pico 2 internal pins, legal RP2350 SPI pins per bus, PWM slice sharing (motor and servo on the same slice is a fault), ADC pin, actual SPI clocks after the divider, matching link IDs. |
| Structure | Arm root stress (climb, hard landing, manoeuvre; von Mises with torsion), deflection, first bending mode against rotor and blade-pass frequency, servo torque margin against the gyroscopic H×ω load, prop clearances. Live arm stress is shown in flight. |
| Flight tests | Arming interlock, hover, pitch/yaw/roll vectoring, gusts, full-throttle punch, aggressive roll, servo rail, 0.1 s / 0.4 s / 3 s dropouts, handset reboot, input fault, 30 % packet loss, hard landing, low battery. Every run can be replayed in 3D. |

## Headless run

```bash
node simulator/tests/run_tests.js            # everything
node simulator/tests/run_tests.js tv_yaw hover  # selected flight tests
```

## Findings with the default (project) configuration

These are model predictions. Verify each one on the bench.

- **Yaw is unstable with the stock Betaflight quad tune.** Half yaw stick asks for 185 °/s. The yaw loop overshoots, and the tilted, counter-rotating props add gyroscopic pitch torque (2·r·H·sin δ) that exceeds the pitch authority, so the aircraft flips. The **Suggested bicopter tune** preset passes: `yaw_srate 25`, `p_yaw 12`, `i_yaw 30`, `f_yaw 15`, `roll/pitch_srate 30`, `angle_limit 30`.
- **MG90S vs 10" props:** at about 300 °/s roll, the props' gyroscopic torque reaches the servo stall torque (~1.8 kg·cm). Keep roll rates low, use stronger servos, or gear the tilt.
- **A2212 + 1045 on 3S draws ~20 A at full throttle**, against a 13 A motor rating. `motor_output_limit ≈ 71` keeps it within the rating.
- **Stage-1 failsafe drops throttle to idle** (rxfail AUTO). A 0.4 s dropout costs about 0.8 m of height even though control resumes.
- **The project CLI assigns no ANGLE mode.** The simulator adds `aux 1 1 1 1300 2100 0 0`; add it to the real board too, or the aircraft flies in ACRO.

## Limits

Masses, coefficients and ratings are catalogue estimates, so weigh and measure your parts. The flight controller models Betaflight's behaviour; it is not the compiled firmware. Filters are simplified, and aerodynamics are quasi-static. A pass here is necessary but not sufficient: follow the props-off checklist in `HANDOFF.md` §13 of the firmware repo before flying.
