# Bicopter Betaflight with a direct NRF24 receiver

Protocol **v2**: custom Betaflight 2026.6.2 for a DIY tilt-rotor bicopter. A Raspberry Pi Pico 2 (RP2350A)
reads an nRF24L01+ PA/LNA radio directly over SPI. The handset is an STM32F103 with its own
nRF24L01+.

**Both aircraft and handset must use protocol v2.** Old v1 images are archived under `firmware/archive/protocol-v1/`.

**Status:** the firmware compiles and passes PC-side tests. It has **not** been flashed or
tested on hardware, and is **not** flight-ready. The STM32F103 board layer for the handset is
not written yet; its inputs are still missing.

Start with [HANDOFF.md](HANDOFF.md). It covers the full context, every change with its reason,
pinned revisions, build steps, wiring, Betaflight CLI setup, flashing, the props-off test
checklist, and status. [HANDOFF_FULL.md](HANDOFF_FULL.md) is the same document with all code appended.

Verification: **67 upstream test executables / 619 cases passed**, plus packet/adapter/handset tests under ASan and UBSan. See [VALIDATION.md](VALIDATION.md). Detected restarts now disarm through Betaflight and require safe inputs followed by deliberate rearming. Short RF dropouts and single bad stick readings do not disarm: RF loss is left to Betaflight failsafe (see section 17 of HANDOFF.md).

The subsequent full audit fixes calibrated ADC endpoints, handset catch-up transmissions and queued RX timestamps, and corrects the timing models. The current UF2 includes those fixes. Read [AUDIT_FIXES.md](AUDIT_FIXES.md) for validation, the previous-image archive and the limits on moving to hardware tests. Pico-only bench checks can start with props removed; complete RC-link testing still requires the STM32F103 board HAL.

## Layout

| Path | Contents |
|---|---|
| `firmware/output/` | `betaflight_2026.6.2_RP2350A_BICOPTER_PICO2_NRF24_V2.uf2` (flash this), `.elf`, `.map`, checksums |
| `firmware/patches/` | Patch for Betaflight at tag `2026.6.2` (`e0b7bb01`) and patch for the config submodule at `3a406c5` |
| `firmware/src-changed/` | Full copies of every changed or new Betaflight file, at repo-relative paths |
| `firmware/build-scripts/` | WSL build environment scripts (tools installed without sudo, environment, export, verification) |
| `firmware/logs/` | Baseline and final build logs, preprocessed define dump |
| `handset/` | Portable handset code: protocol core (shared verbatim with the receiver), TX app, HAL interface, host test |
| `tests/` | Host tests for the protocol core and for the real receiver adapter against an nRF24 model |

## Quick test run (host)

```bash
cd tests
gcc -std=c11 -O2 -I../firmware/src-changed/src/main -o test_link test_link.c ../firmware/src-changed/src/main/rx/nrf24_bicopter_link.c && ./test_link
gcc -std=c11 -O1 -Istubs -I../firmware/src-changed/src/main -o test_rx_adapter test_rx_adapter.c ../firmware/src-changed/src/main/rx/nrf24_bicopter.c ../firmware/src-changed/src/main/rx/nrf24_bicopter_link.c && ./test_rx_adapter
cd ../handset/test
gcc -std=c11 -O2 -o test_handset test_handset.c ../src/handset_app.c ../shared/nrf24_bicopter_link.c && ./test_handset
```

## Licence

The firmware changes modify Betaflight, which is licensed under GPL-3.0-or-later. The
patches, the changed source files and the compiled firmware are distributed under the same
licence; see <https://github.com/betaflight/betaflight/blob/master/LICENSE>. The handset code
and tests in this repository are also released under GPL-3.0-or-later.

Latest complete retest: [FULL_REGRESSION.md](FULL_REGRESSION.md), including all earlier software checks, reference-board builds, fuzzing and historical safety probes. The packaged UF2 remains unchanged; the separate hopping test model and stale full local checkout were corrected.
