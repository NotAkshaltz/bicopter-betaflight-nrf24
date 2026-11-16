# Complete regression rerun — 15 November 2026

**Result: all applicable tests of the current implementation pass.** This reruns the software checks recorded in Claude's handoff, the original review, the protocol-v2 correction, the dropout correction and the full audit. No hardware was flashed or tested. The expanded upstream all-board unit-test matrix was never part of those earlier runs and was not run here.

## Results

| Check | Fresh result |
|---|---|
| Native Windows GCC 16.1.0, `-O0`, `-O1`, `-O2`, `-O3`, warnings as errors | All 12 packet/adapter/handset builds and runs PASS |
| Current packet tests | 1,653,716 checks, zero failures per run |
| Current receiver tests | 15,596 checks, zero failures per run |
| Current handset tests | PASS: 100/200/1000/2000Hz acquisition, faults, resets, calibrated rails, stalls, pending TX/MAX_RT and timer wrap |
| Clang 18.1.3 ASan/UBSan | All three current host programs PASS |
| Betaflight default suite | 67 executables / 619 cases PASS, including the added reset-disarm test |
| Current code with the earlier `98aa346` receiver/handset test suites | 5,147 receiver checks PASS; handset PASS |
| Original stale-sequence probe | Delayed sequences 10 and 11 remain stale; reference remains 100 |
| Original short-session probe, plus explicit v2 safety assertions | Reset helper called once; ARM/throttle remain 1000/1000; 18 assertions PASS |
| Original one-IRQ backlog probe | Final frame time 46,200us, with no rollback to the first 5,000us IRQ |
| Original 1kHz acquisition scenario, extended beyond v2 startup | 237 accepted frames over 2.2 seconds; skipped acquisition sequences accepted correctly |
| Historical dropout negative controls on `4570001` | Expected 10 receiver / 5 handset assertion failures reproduced |
| Full-audit negative controls on `98aa346` | Expected 6 receiver / 1,803 handset assertion failures reproduced; repeated timing violations account for many assertions |
| Seeded libFuzzer + ASan/UBSan | 1,000,000 inputs PASS, including a valid v2 golden-vector corpus seed |
| Clang static analyzer | Zero diagnostics in protocol core, receiver adapter and handset application |
| Cortex-M3 compile | Actual portable handset application/core PASS with Arm GNU 13.3, warnings as errors |
| Fresh current `BICOPTER_PICO2` build | PASS |
| Fresh patched `PICO2_2350A` reference build | PASS |
| Fresh patched `RASPBERRY_PI_UAVFC` reference build | PASS |
| Original unpatched `PICO2_2350A` | Expected pre-existing USB MSC link failure reproduced |
| Original unpatched `RASPBERRY_PI_UAVFC` control | PASS |
| All three current build images | Official picotool confirms RP2350 Arm Secure; only the known linker RWX warning remains |
| Actual compiler macro dump and ELF | RX/IMU/output pins, radio protocol, 1MHz radio SPI limit, PWM 200Hz defaults, 4MB flash layout, reset-disarm and NRF code symbols verified |
| Both distribution patches | Apply to clean pinned base files and reproduce all 20 changed/new firmware/config files byte-for-byte |
| Full local source checkout versus exported files | All 20 agree after the synchronization described below; line endings normalized for comparison |
| Shared protocol source/header | Byte-identical between handset and receiver |
| Independent Python packet packing/CRC | Complete v2 golden vector and standard CRC check match |
| Current and archived image checksums | PASS for current artifacts and both historical archives |
| Packaged UF2 structure/layout | All 1,563 blocks valid; payload addresses remain below the reserved config sector |

The packet count increased because the hopping simulation now explicitly checks that packet-completion events are not in the future. Assertion counts are not separate test cases or separate bugs.

## What the rerun found and corrected

1. **A second timing-model weakness:** the pure-core hopping simulation in `tests/test_link.c` still fed a future packet-completion timestamp to the hop state machine. It now schedules each jittered transmission once and waits for its full 1ms modeled airtime before delivery, requiring RX settling before packet start. All existing acquisition, clock-drift, random-loss and reacquisition thresholds still pass. Runtime firmware source was unchanged by this test correction.
2. **A stale full local checkout:** `firmware-work/betaflight/src/main/rx/nrf24_bicopter.c` and `nrf24_bicopter_link.h` still matched the older `4570001` delivery. The earlier fix had updated the exported package and Linux build but missed these two local files. They were backed up under `Research/full-regression-2026-11-15/stale-checkout-before-sync/` and synchronized with the tested exports. Rebuilding from that checkout now uses the current receiver behavior.

The original one-second acquisition probe still exits 1 when run unchanged because protocol v2 deliberately announces startup for at least one second. Extending it to 2.2 seconds verifies that acquisition succeeds after that intentional startup interval. The old session diagnostic also prints an unconditional historical conclusion; assertions on its actual output show the controls are safely clamped now.

A new macro-inspection assertion initially assumed DShot/RPM capabilities should be absent. The original captured macro dump proves these were already compiled capabilities; the selected default remains PWM at 200Hz. The inspection was corrected to verify selected settings rather than require removal of unused capabilities. No firmware change was needed for this.

The failures reproduced on historical sources and the unpatched baseline are expected negative controls, not failures of the current implementation.

## Artifacts and reproducibility

The packaged runtime sources and flashable image did not change during this rerun. Current UF2 SHA256 remains:

`17d384bd387d4d341a2d6de108935c3628f4ecd47bad3d1175d8476aad771486`

Fresh independently rebuilt artifacts are retained in the disposable validation environment; their hashes are saved with the logs. Build metadata/output paths can change binary hashes, so successful rebuilding is not claimed as bit-for-bit reproducibility of the packaged UF2.

Complete raw logs, native compiler results, historical probes, source backups and rerun scripts are in `../Research/full-regression-2026-11-15/`. Selected current logs and a build summary are also under `firmware/logs/full-regression-*`. Updated source hashes and current test counts are in `firmware/output/BUILD_MANIFEST.json`.

## Remaining hardware work

No additional runtime regression was found by these checks. They do not prove that every possible defect is absent. The radio models do not reproduce every interrupt, DMA, power or scheduler interaction. The STM32F103 board HAL and complete handset image are still missing. Pico-only bench checks can begin with props removed; full RC-link, physical motor-stop, gyro-load, power-integrity and range tests remain necessary before flight.
