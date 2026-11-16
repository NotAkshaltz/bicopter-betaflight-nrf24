# Full-audit fixes and testing status

Updated 15 November 2026, starting from delivery commit `98aa346`. Protocol remains v2; the wire format is unchanged. Use the updated handset sources together with the updated aircraft UF2.

## Fixed behavior

1. **Calibrated endpoints:** a measured ADC endpoint at 0 or 4095 is valid. Rail readings outside the individual stick's calibrated range, values above 4095 and invalid switch values still flag input faults. With calibration 0/2048/4095, minimum throttle now releases the startup interlock after safe ARM/throttle inputs and the announcement period. A disconnected wire producing a calibrated endpoint cannot be distinguished by ADC value alone; any additional disconnect detection belongs in the board integration.
2. **Delayed handset loop:** missed transmit slots are skipped. The next launch is scheduled at least 5 ms after the actual preceding launch, rather than attempting a catch-up burst. An outstanding radio transaction must report TX_DS or MAX_RT before another payload is loaded or the channel is changed. A radio that never finishes stops sending; normal aircraft link-loss failsafe applies. Nominal rate is 200 Hz; foreground and SPI latency reduce the measured rate slightly (about 198 Hz in the host model).
3. **Queued RX timestamps:** packets in a backlog use service time, marked unreliable for re-anchoring an already synchronized hop clock. The first IRQ edge is no longer assigned to the last queued packet. Isolated packets still use a valid captured IRQ time. Accepted frame timing cannot move backwards because of an old IRQ timestamp.
4. **Test timing:** simulated RX packets become available only after their modeled arrival, with receiver settling required before packet start. The handset model includes finite SPI transfer time, a minimum CE pulse and a busy interval before TX completion. It rejects overlapping launches and flush/load/retune operations while busy.

## Evidence

- The expanded regression tests fail against the previous runtime sources and pass against the fixes.
- Actual receiver tests cover three packets behind one IRQ, interrupt and polled operation, and draining across the 32-bit timer wrap.
- Actual handset tests cover all calibrated axes, reversed endpoints, rail faults outside calibration, startup at zero throttle, 9/20/100 ms loop stalls, delayed TX completion, MAX_RT accounting and timer wrap.
- All custom tests pass with Clang 18 ASan/UBSan and warnings treated as errors. The packet core reports 1,653,716 checks with zero failures.
- Betaflight's default suite passes: 67 executables / 619 test cases, including the added reset-disarm case. This is not the expanded all-board matrix.
- Portable handset application/core compile for Cortex-M3 with Arm GNU 13.3 and warnings treated as errors.
- The Pico firmware was clean-built with the pinned source, config and SDK revisions. The existing linker RWX warning remains.
- Both distribution patches apply to clean pinned base files and reproduce all 20 changed/new firmware and configuration files byte-for-byte.

Current logs: `firmware/logs/audit-fixed-host-sanitizers.log`, `audit-fixed-upstream-tests.log`, `audit-fixed-build.log`, `audit-fixed-image-info.log`, and `audit-regression-old-*.log`. `firmware/output/BUILD_MANIFEST.json` and `SHA256SUMS.txt` identify the current sources and images.

The previous packaged UF2, ELF, map, manifest and checksums are preserved in `firmware/archive/protocol-v2-pre-audit-98aa346/`. They are historical artifacts, not the image to flash now.

## Moving to hardware tests

The identified audit issues are fixed. That does not prove the absence of other bugs or establish flight readiness. Host timing models do not reproduce every radio, power, interrupt, DMA or scheduler interaction.

**Pico-only bench testing can begin with props removed.** Use the current image from `firmware/output/`, then follow HANDOFF sections 10–13:

1. Check the physical wiring and 3.3 V radio supply before applying power.
2. Flash the current UF2 using Pico BOOTSEL. Verify USB connection, board identity, IMU response/orientation and radio initialization (`debug[7] = 0`).
3. With props removed, verify output mapping, motor/servo direction, pulse rates and safe startup before testing the link while armed.
4. Complete the STM32F103 HAL and measure/calibrate the actual handset inputs. There is still no complete flashable handset image in this repository. Do not use an old transmitter implementation as evidence that these handset fixes work.
5. Run end-to-end control, endpoint, frozen-input, RF-loss, handset-reset and deliberate-rearm checks. Measure actual motor-stop latency and check gyro-loop timing while RX traffic is active.
6. Finish power-integrity, range/interference and sustained-operation tests before considering flight. Record failures and measurements; passing the PC suite alone is insufficient.

No hardware was flashed or physically tested during this software correction.
