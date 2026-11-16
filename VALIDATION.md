# Protocol v2 validation — 15 November 2026

The full-audit corrections are included in the current sources and rebuilt UF2. The subsequent complete rerun is documented in [FULL_REGRESSION.md](FULL_REGRESSION.md); its current logs use the `full-regression-` prefix. No hardware was flashed. See [AUDIT_FIXES.md](AUDIT_FIXES.md) for the fixes and hardware-testing gates.

| Verification | Result |
|---|---|
| Betaflight default `make test -j6` | 67 executables PASS, 619 cases PASS (includes one added reset test) |
| Packet core | 1,653,716 checks, zero failures |
| Actual receiver adapter with corrected arrival model | 15,596 checks, zero failures |
| Actual portable handset with finite SPI/TX model | PASS at 100/200/1000/2000Hz acquisition; 5ms minimum TX spacing |
| ASan / UBSan, Clang 18.1.3 | All three host programs clean, warnings treated as errors |
| Cortex-M3 compilation, Arm GNU 13.3 | Portable handset application/core PASS; board HAL still absent |
| Clean Pico build | PASS, UF2/ELF/map regenerated; existing linker RWX warning only |

## Regression coverage

Calibrated ADC endpoints at 0 and 4095, reverse mapping, invalid ADC values and out-of-calibration rail faults; safe startup at minimum throttle; late-loop stalls; no overlapping transmissions or flush/load/retune while busy; delayed TX completion and MAX_RT accounting; one-IRQ RX backlog timestamps in interrupt and polling modes; timer wrap; packet rejection, frozen samples, RF dropout recovery and handset reset/disarm interlocks.

The final expanded tests also fail against the previous runtime sources: six receiver assertions and 1,803 handset assertions fail, including repeated timing violations. Those counts are failing assertions, not separate bugs. Both fixed implementations pass. The comparison logs are `audit-regression-old-adapter.log` and `audit-regression-old-handset.log`.

Ordinary RF/input faults continue to use Betaflight failsafe. A detected new handset session or explicit restart still calls the real reset-disarm helper and requires safe inputs followed by deliberate rearming. The upstream test verifies the disarm call and arming flags using its existing stub; it does not measure physical motor outputs.

## Current artifacts and evidence

- Current UF2: `firmware/output/betaflight_2026.6.2_RP2350A_BICOPTER_PICO2_NRF24_V2.uf2`
- SHA256: `17d384bd387d4d341a2d6de108935c3628f4ecd47bad3d1175d8476aad771486`
- Source/artifact hashes: `firmware/output/BUILD_MANIFEST.json` and `SHA256SUMS.txt`
- Logs: `firmware/logs/audit-fixed-host-sanitizers.log`, `audit-fixed-upstream-tests.log`, `audit-fixed-build.log`, `audit-fixed-image-info.log`
- Prior image: `firmware/archive/protocol-v2-pre-audit-98aa346/`

Verification used Ubuntu 24.04, Clang 18.1.3, Arm GNU 13.3.Rel1, picotool 2.3.1 and the pinned SDK/config commits. The default upstream suite was run; the expanded all-board matrix was not.

## Remaining limits

The STM32F103 board-specific HAL and complete programming image are still missing. Pico-only bench checks can begin with props removed. Complete RC-link tests require the actual handset integration and calibration. Actual SPI/IRQ/DMA timing under gyro load, radio/power behavior, motor-stop latency, ESC/servo operation and range remain unverified. Passing these software tests is not flight qualification, and an audit cannot prove there are no other defects.
