# Protocol v2 validation — 15 November 2026

The three review failures are fixed in the current sources and new UF2. No hardware was flashed.

| Verification | Result |
|---|---|
| Original Betaflight `make test -j6` | 67 executables PASS, 619 test cases PASS (includes one added reset test) |
| Packet core | 1,522,481 checks, 0 failures |
| Actual radio adapter with model | 4,761 checks, 0 failures |
| Actual portable handset | PASS, 100/200/1000/2000Hz acquisition at 200Hz RF |
| ASan / UBSan | Clean; `-Werror` enabled |
| Golden packet | Full v2 vector asserted; independently verified CRC |
| Cortex-M3 compile | Portable handset application/core PASS; board HAL still absent |
| Pico UF2 build | PASS; target RP2350 Arm Secure, 4MB flash |

## Reset behavior now enforced

The handset transmits a startup/recovery announcement for at least one second and until ARM OFF plus low throttle are newly sampled. It transmits safe ARM/throttle values while locked. The receiver independently latches its own guard on startup, reset and prolonged loss. New session IDs and explicit announcements call the real Betaflight failsafe-reset helper, which immediately invokes the normal disarm API and existing recovery state machine. An announcement handles reuse of the same session byte. To rearm, hold ARM OFF and throttle low until recovery blocks clear, then switch ARM ON.

Receiver tests cover short-session resets, same-session explicit reset, repeated announcements, high ARM/high throttle, wrong safe-input ordering, and deliberate rearming. The upstream-framework test executes the real failsafe helper/state machine and checks its disarm call and arming flags. It uses the existing disarm stub, not physical motor outputs.

## Files and reproducibility

- Current UF2: `firmware/output/betaflight_2026.6.2_RP2350A_BICOPTER_PICO2_NRF24_V2.uf2`
- UF2 SHA256: `6b5a5719e07446440b395b5e95dc00e2241c55b103ddf34eb405fe46ba41c86a`
- All current artifact hashes: `firmware/output/SHA256SUMS.txt`
- Source/artifact manifest: `firmware/output/BUILD_MANIFEST.json`
- Test logs: `firmware/logs/upstream-tests.log`, `firmware/logs/host-sanitizers.log`
- Build and binary inspection: `firmware/logs/corrected-v2-build.log`, `uf2-info.log`, `firmware-symbols.log`
- Patches and full changed files are updated; build instructions are in HANDOFF.md.

The container was Ubuntu 24.04 with Clang 18.1.3, GNU Make 4.3, Arm GNU 13.3.Rel1, picotool 2.3.1, and release-pinned SDK/config commits. The upstream default suite was run without suppressing tests; the expanded all-board matrix was not run.

## Remaining limits

The STM32 hardware-specific HAL/programming image remains missing. The portable code is implemented and target-compiled, but cannot yet be flashed as a complete handset program. Calibration, actual pins and acquisition completion semantics need real board integration. Hardware RF, SPI/IRQ timing under gyro load, power integrity, actual motor-stop latency, ESC/servo operation and range remain unverified. Run the props-off checklist before powered commissioning. These results do not establish flight readiness.
