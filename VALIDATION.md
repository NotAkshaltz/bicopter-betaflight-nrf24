# Protocol v2 validation — 15 November 2026

The three review failures are fixed in the current sources and new UF2. No hardware was flashed.

| Verification | Result |
|---|---|
| Original Betaflight `make test -j6` | 67 executables PASS, 619 test cases PASS (includes one added reset test) |
| Packet core | 1,522,481 checks, 0 failures |
| Actual radio adapter with model | 5,147 checks, 0 failures (after the dropout/input-fault fix) |
| Actual portable handset | PASS, 100/200/1000/2000Hz acquisition at 200Hz RF |
| ASan / UBSan | Clean; `-Werror` enabled |
| Golden packet | Full v2 vector asserted; independently verified CRC |
| Cortex-M3 compile | Portable handset application/core PASS; board HAL still absent |
| Pico UF2 build | PASS; target RP2350 Arm Secure, 4MB flash |

## Reset behavior now enforced

The handset transmits a startup announcement for at least one second after boot/re-init and until ARM OFF plus low throttle are newly sampled. It transmits safe ARM/throttle values while locked. The receiver independently latches its own guard on startup and on a detected handset reset. RF dropouts and input faults do not latch it; see below. New session IDs and explicit announcements call the real Betaflight failsafe-reset helper, which immediately invokes the normal disarm API and existing recovery state machine. An announcement handles reuse of the same session byte. To rearm, hold ARM OFF and throttle low until recovery blocks clear, then switch ARM ON.

Receiver tests cover short-session resets, same-session explicit reset, repeated announcements, high ARM/high throttle, wrong safe-input ordering, and deliberate rearming. The upstream-framework test executes the real failsafe helper/state machine and checks its disarm call and arming flags. It uses the existing disarm stub, not physical motor outputs.

## Dropout and input-fault fix

Two in-flight disarm paths were removed and covered by regression tests (which fail against the earlier v2 code):

- An RF gap longer than 150 ms no longer re-engages the receiver guard. Tested with 100 ms to 3 s gaps while armed: ARM and throttle pass through unchanged when the link returns. RF loss is handled only by Betaflight failsafe.
- One bad stick reading or a late acquisition only flags `TX_INPUT_FAULT` on those packets. The handset no longer enters its restart lock, and the receiver no longer re-engages its guard, on input faults.

After this fix: packet core 1,522,481 checks and adapter 5,147 checks with 0 failures, handset PASS, with gcc `-Wall -Wextra -Werror` at -O0 to -O3. Upstream `make test` and ASan/UBSan were not re-run after the fix; the fix does not touch `failsafe.c` or upstream-tested code. The UF2 was rebuilt from the updated patch on a clean 2026.6.2 tree with no new warnings.

## Files and reproducibility

- Current UF2: `firmware/output/betaflight_2026.6.2_RP2350A_BICOPTER_PICO2_NRF24_V2.uf2`
- UF2 SHA256: `fb8ecdb3445cf99753bb365b414a7a87b98c9c86b677957faeae8453ef63bd98`
- All current artifact hashes: `firmware/output/SHA256SUMS.txt`
- Source/artifact manifest: `firmware/output/BUILD_MANIFEST.json`
- Test logs: `firmware/logs/upstream-tests.log`, `firmware/logs/host-sanitizers.log`
- Build and binary inspection: `firmware/logs/corrected-v2-build.log`, `uf2-info.log`, `firmware-symbols.log`
- Patches and full changed files are updated; build instructions are in HANDOFF.md.

The container was Ubuntu 24.04 with Clang 18.1.3, GNU Make 4.3, Arm GNU 13.3.Rel1, picotool 2.3.1, and release-pinned SDK/config commits. The upstream default suite was run without suppressing tests; the expanded all-board matrix was not run.

## Remaining limits

The STM32 hardware-specific HAL/programming image remains missing. The portable code is implemented and target-compiled, but cannot yet be flashed as a complete handset program. Calibration, actual pins and acquisition completion semantics need real board integration. Hardware RF, SPI/IRQ timing under gyro load, power integrity, actual motor-stop latency, ESC/servo operation and range remain unverified. Run the props-off checklist before powered commissioning. These results do not establish flight readiness.
