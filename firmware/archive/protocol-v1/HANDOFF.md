# Bicopter Betaflight + NRF24 direct receiver: project handoff

Status date: 2026-11-15. This document is self-contained. `HANDOFF_FULL.md` is this
document with all changed code appended.

## 1. Project context

- Aircraft: DIY bicopter. Two A2212 1000KV motors on SimonK 30A ESCs, two MG90S tilt
  servos (nacelle tilt), 3S LiPo, PDB with regulated outputs, independent servo supply.
- Flight controller: Raspberry Pi Pico 2 (RP2350A, 4 MB flash) running modified Betaflight.
  It uses an MPU6500 on SPI0 and an nRF24L01+ PA/LNA on SPI1, read directly by Betaflight
  (no UART receiver, no bridge MCU).
- Handset: STM32F103 (listed as C8T6) + nRF24L01+ PA/LNA + joysticks and switches. The exact
  board, existing source, programmer and wiring are **unknown / not provided**.
- Problem solved: upstream Betaflight's Pico target does `#undef USE_RX_SPI`, so it has no
  NRF24 receiver. The fix adds an opt-in NRF24 path and a new protocol, `NRF24_BICOPTER` v1.
- Constraints from the owner:
  - Keep Betaflight's flight control, mixer and failsafe; no rewrite.
  - Built-in BI mixer, PWM ESCs at 200 Hz (commissioning candidate), 50 Hz servos.
  - DShot, bidirectional DShot, RPM filtering and dynamic idle stay off.
  - Do not invent handset pins.
  - The owner flashes and tests the hardware; this work covers the source and builds only.

## 2. Pinned revisions (verified)

| Item | Revision |
|---|---|
| Betaflight | tag `2026.6.2` = commit `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`. The tag is lightweight, and `git ls-remote` and the checkout agree. |
| Config submodule (`src/config`) | `3a406c5c30c20d7c3bc7dd5ed547cca961baf0a9`, the release-pinned submodule commit. New folder `configs/BICOPTER_PICO2` is added there, uncommitted. |
| pico-sdk | `ee68c78d0afae2b69c03ae1a72bf5cc267a2d94c` (2.1.1-1-gee68c78d). Its tinyusb is `86ad6e56` (0.18.0). |
| Toolchain | Arm GNU Toolchain 13.3.Rel1 (13.3.1 20240614), installed by `make arm_sdk_install` |
| picotool | v2.3.1, official prebuilt from raspberrypi/pico-sdk-tools release v2.3.1-0 (sha256 `878a1a1e…248e`) |
| make | GNU Make 4.4.1, extracted from the Ubuntu `.deb` without sudo |
| Build host | WSL2 Ubuntu 26.04 on Windows 11. The checkout is at `~/bicopter-fw/betaflight` on branch `bicopter-nrf24-dev`; nothing is committed. |

Target: RP2350 **Arm** (Cortex-M33), image family `rp2350-arm-s` (confirmed with `picotool info`).

## 3. Output

- UF2: `firmware/output/betaflight_2026.6.2_RP2350A_BICOPTER_PICO2.uf2`
  (sha256 `6d8fa2d678abd29cd025161465f62a2e3c66257a550bba9b56e87417f1f0258d`).
  The `.elf` and `.map` are in the same folder.
- Size: FLASH 390 KB of 4032 KB, RAM 376,628 B of 512 KB.
- Config storage: `__config_start=0x103F0000`, `__config_end=0x10400000`, the last 64 KB of the 4 MB part.

## 4. Findings in the pinned source (verified, not assumed)

1. **The baseline does not link.** Unchanged `make CONFIG=PICO2_2350A` fails with undefined
   `pico_msc_active` and `tud_msc_*_cb`:
   - `common_post.h` undefines `USE_USB_MSC` when a board has no blackbox storage.
   - TinyUSB MSC and `usb_descriptors.c` are still compiled.

   The control build `RASPBERRY_PI_UAVFC`, which has an SD card, builds unchanged. Fix: inert
   stubs in `usb_msc_pico.c`. With that fix, `PICO2_2350A` builds, and it still builds with the
   full patch.
2. `target_RP2350.h` has an unconditional `#undef USE_RX_SPI`. Include order is config.h →
   common_pre → target.h → common_post, so a board config cannot override it without a change.
   It is now guarded by `PICO_ENABLE_RX_SPI_NRF24`.
3. **No** `rx/nrf24_*.c` protocol file, including `nrf24_inav.c`, is in `mk/source.mk`. Only the
   driver `drivers/rx/rx_nrf24l01.c` is listed.
4. The legacy driver uses compile-time `RX_CE_PIN`, `RX_NSS_PIN` (in the unused "Fast" read
   helper, but still compiled) and `RX_SPI_INSTANCE`. `pg/rx_spi.c` uses `RX_SPI_CS_PIN`,
   `RX_SPI_EXTI_PIN` and `RX_SPI_INSTANCE`. All are defined; NSS and CS are the same pin.
   The driver had never been compiled for the Pico and needed `drivers/io_impl.h` for `ioRecs`.
5. **SPI clock.** The generic RX SPI default is 13.5 MHz, above the nRF24L01+ 10 MHz limit. The
   Pico divider rounds down, so 13.5 MHz gives 12.5 MHz from the 150 MHz `clk_peri`.
   `NRF24L01_Initialize()` now caps the clock (default 8 MHz). This board sets 1 MHz for bring-up.
6. **IRQ edge.** The generic RX SPI EXTI default is rising edge, and the INAV init ignores
   `extiConfig`. The new protocol sets falling edge, because the nRF24 IRQ is active-low.
   - The Pico `EXTIConfig` ignores the pull configuration. That is fine: the nRF24 IRQ is push-pull.
   - `rxSpiExtiInit()`'s "missed edge" check reads the raw level. With a falling trigger and an
     idle-high line, this causes one spurious service at boot. That is harmless: the protocol
     reads STATUS, finds the FIFO empty and backs off.
7. **RX_DR handling.** `NRF24L01_ReadPayloadIfAvailable()` never clears `RX_DR`, so after the
   first packet the IRQ line stays low and no further falling edges occur. It also reads a fixed
   length with dynamic payloads on. The new code does the following:
   - reads `R_RX_PL_WID`;
   - flushes on a width above 32;
   - clears `RX_DR` after reading;
   - re-checks FIFO_STATUS;
   - reads one payload per call.
8. **Timestamps.** `rxSpiFrameStatus()` always used the EXTI time, which is 0 forever without
   EXTI. A backward-compatible optional `protocolFrameTimeUs` hook was added. The protocol stamps
   frames with the EXTI time when that time belongs to the packet, otherwise with `micros()`.
9. **SPI numbering.** On the Pico, `USE_SPI_DEVICE_0` gives `SPIDEV_0=0` and `SPIDEV_1=1`, and
   `SPI_DEV_TO_CFG(x)=x+1`. So hardware SPI1 is stored as `rx_spi_bus = 2` and SPI0 as `1`.
10. **Flash size.** RP2350 defaults are `MCU_FLASH_SIZE=8192` and `PICO_FLASH_SIZE_BYTES=8 MB`,
    while the linker default was already 4 MB. The new board `config.mk` sets 4096 KB and
    `4194304`. The board's `pico_flash_mem.ld` sets 4M with no font area. Verified in the
    preprocessed defines and in the ELF symbols.
11. **Legacy INAV protocol weaknesses** (the reason for a project variant):
    - no data-packet validation;
    - bind is not persisted (`rxSpiId` is forced NULL);
    - the bind ACK cannot reach the TX, because the RX changes address immediately;
    - the shared payload buffer can be overwritten by a later rejected packet before it is decoded.
12. **Servo driver:** fixed 50 Hz, using the compile-time `SYS_CLK_HZ` = 150 MHz, which matches
    the SDK clock init. `servo_pwm_rate` is ignored.
13. **Motor driver:** `MOTOR_PROTOCOL_PWM` is continuous; it honours `motor_pwm_rate`, capped at
    480. At 200 Hz: clkdiv 12, wrap 62499, 12.5 counts/µs.
14. **PWM slices:** GP10/11 are slice 5, GP20/21 are slice 2, so the frequencies are independent.
15. **BI mixer** (`USE_UNCOMMON_MIXERS` is on):
    - Motors: left `{thr 1, roll +1}`, right `{thr 1, roll −1}`.
    - Servo outputs: output 1 is `SERVO_BICOPTER_LEFT` (yaw +100, pitch −100); output 2 is
      `SERVO_BICOPTER_RIGHT` (yaw +100, pitch +100).
16. **Failsafe:** only `RX_FRAME_COMPLETE` refreshes `needRxSignalBefore`. RXLOSS comes after
    150 ms, then Betaflight failsafe stage 1/2. Re-arming after failsafe is blocked by
    Betaflight's arming-disable flags until the arm switch is cycled. This is standard Betaflight
    behaviour, not modified; verify it on hardware.

## 5. Changes made (Betaflight tree)

All in `firmware/patches/betaflight-2026.6.2-bicopter-nrf24.patch` (main repo) and
`firmware/patches/config-BICOPTER_PICO2.patch` (config submodule). Full copies are in
`firmware/src-changed/`.

| File | Change |
|---|---|
| `src/platform/PICO/usb/usb_msc_pico.c` | `#else` branch with inert MSC callbacks and `pico_msc_active=false` (fixes the storage-less Pico link) |
| `src/platform/PICO/target/common/target_RP2350.h` | `#undef USE_RX_SPI` only if `!defined(PICO_ENABLE_RX_SPI_NRF24)` |
| `mk/source.mk` | adds `rx/nrf24_bicopter.c`, `rx/nrf24_bicopter_link.c` |
| `src/main/rx/rx_spi.h` | appends `RX_SPI_NRF24_BICOPTER` (MSP-order safe: appended) |
| `src/main/cli/settings.c` | appends `"NRF24_BICOPTER"` to `lookupTableRxSpi` |
| `src/main/rx/rx_spi.c` | protocol case under `USE_RX_NRF24_BICOPTER`; optional `protocolFrameTimeUs` hook |
| `src/main/drivers/rx/rx_nrf24l01.c/.h` | SPI clock cap `NRF24L01_MAX_SPI_CLK_HZ` (default 8 MHz, `#error` above 10 MHz); `NRF24L01_SetChannelRxMode()` (CE low → RF_CH → CE high); `NRF24L01_ReadPayloadWidth()`; `#include io_impl.h` |
| `src/main/build/debug.h/.c` | appends debug mode `RX_NRF24_BICOPTER` |
| `src/main/rx/nrf24_bicopter_link.c/.h` (new) | hardware-independent protocol core, shared verbatim with the handset |
| `src/main/rx/nrf24_bicopter.c/.h` (new) | Betaflight RX_SPI adapter: radio setup with read-back verification, IRQ/poll service, hop scheduling, freshness |
| `src/config/configs/BICOPTER_PICO2/{config.h,config.mk,pico_flash_mem.ld}` (new) | board definition |

Upstream's INAV protocol is untouched and not compiled.

## 6. NRF24_BICOPTER protocol v1 (both ends)

**Radio**
- 250 kbps, 2-byte radio CRC, 5-byte address, EN_AA and dynamic payload on pipe 0.
- No ACK payload, no retransmits (TX `SETUP_RETR=0x20`: ARD 750 µs, ARC 0).
- RX `RF_SETUP=0x22` (−12 dBm, used only for the ACK).
- Address bytes are written in array order (`addr[0]` = LSByte, sent first).

**Pairing**
- There is no over-the-air bind. Both ends are built with link ID `0xBE5336DF`:
  `BICOPTER_NRF24_LINK_ID` in `config.h` and `HANDSET_LINK_ID` in `handset/config/link_config.h`.
  Change both together. `rx_spi_id` is not used.
- From the ID: address `3C 41 58 CB F7`, RF_CH `20 39 58 77`, tag `0x0934`, CRC seed `0xFEFA`.
  Derivation: `blMix32` (lowbias32 hash). Address bytes 00/FF/55/AA are XORed with 0x3C.
  Channels are `4 + (mix%20) + 19·i`.

**Packet (20 bytes, little-endian)**

| Bytes | Field |
|---|---|
| `[0]` | `0xB1` |
| `[1]` | version 1 |
| `[2..3]` | link tag |
| `[4]` | session (random, non-zero per handset boot) |
| `[5..6]` | sampleSeq: incremented **only per new input acquisition** |
| `[7..16]` | 8 × 10-bit channels, LSB-first; v ∈ 0..1000 gives 1000+v µs; 1001..1023 invalid |
| `[17]` | flags: bit0 = TX_INPUT_FAULT; the other bits must be 0 |
| `[18..19]` | CRC-16/CCITT (poly 0x1021) over bytes 0..17, seed from the link ID |

Known test vector (link `0xBE5336DF`, session `0x5A`, seq `0x1234`, channels `1000+(i·137 mod 1001)`):
`B10134095A3412002422D16624B66AF3EF000CB8`.

**Validation (RX).** A packet is rejected if any of the following holds: length ≠ 20, wrong
type or version, wrong tag, wrong CRC, a reserved flag bit set, a channel out of range,
session 0, or pipe ≠ 0. Rejected packets never refresh the watchdog and never touch the
shared payload buffer.

**Freshness**
- Same session and seq step 1..200: accept.
- Step 0: duplicate.
- Backward step: stale.
- New session or a jump above 200: becomes a candidate. It is accepted only when the next packet
  is the candidate seq+1 in the same session.
- seq is 16-bit modulo, so wrap is seamless.
- TX_INPUT_FAULT is never accepted.
- Effects: a frozen acquisition produces only duplicates, so RXLOSS follows after 150 ms. A
  handset restart costs one packet. Restarting with the same session byte (1/255 chance)
  recovers after 2 consecutive packets.

**Hopping**
- The TX sends every 5 ms, packet k on `ch[k mod 4]`.
- The RX retunes at packet time + 2.5 ms, i.e. mid-slot, so the ACK is never cut, and allows
  ±2.5 ms tolerance.
- Each authentic packet (even a duplicate) with a reliable timestamp re-anchors the slot timing.
- More than 8 empty slots (40 ms) switch to search: 25 ms dwell per channel.
- A delayed call advances N slots arithmetically, never in a loop.
- A FIFO-late packet (timestamp before the last retune) is attributed to the previous slot.

**RX runtime cost**
- Idle: a GPIO read (IRQ wired) or a STATUS read every 500 µs (no IRQ).
- Per call: at most one payload and one retune. The radio model measured at most 5 SPI
  transactions per call (about 29 bytes; roughly 250–300 µs at 1 MHz SPI, about 40 µs at 8 MHz).

**Debug mode `RX_NRF24_BICOPTER`**

| Value | Meaning |
|---|---|
| debug[0] | accepted |
| debug[1] | malformed + flushes |
| debug[2] | duplicate + stale |
| debug[3] | resync-pending + TX-fault |
| debug[4] | (state<<8) \| RF_CH, where state 1 = synced |
| debug[5] | missed slots |
| debug[6] | spurious IRQ/poll |
| debug[7] | init status (0 OK; 1 SETUP_AW, 2 FEATURE, 3 registers, 4 address, 5 CONFIG readback failed) |

If init fails, the RX is disabled (permanent RXLOSS) until reboot.

## 7. Reproducible build

The commands below run inside WSL Ubuntu. The scripts are in `firmware/build-scripts/`.
`hosttools.sh` installs make and libusb from `.deb` files without sudo, plus picotool 2.3.1.
`env.sh` sets PATH.
```bash
mkdir -p ~/bicopter-fw && cd ~/bicopter-fw
git clone --branch 2026.6.2 https://github.com/betaflight/betaflight.git && cd betaflight
git checkout -b bicopter-nrf24-dev
git submodule update --init src/config
git submodule update --init --checkout --recursive -- lib/modules/pico-sdk
bash /path/to/firmware/build-scripts/hosttools.sh
mkdir -p tools/picotool && cp ~/bicopter-fw/hosttools/pt/picotool/picotool tools/picotool/
git apply /path/to/firmware/patches/betaflight-2026.6.2-bicopter-nrf24.patch
(cd src/config && git apply /path/to/firmware/patches/config-BICOPTER_PICO2.patch)
bash /path/to/firmware/build-scripts/env.sh make arm_sdk_install
bash /path/to/firmware/build-scripts/env.sh make CONFIG=BICOPTER_PICO2 -j24
# -> obj/betaflight_2026.6.2_RP2350A_BICOPTER_PICO2.uf2
```
Logs are in `firmware/logs/`:
- the baseline failure and the MSC fix;
- the UAVFC control build;
- the final clean build;
- the full preprocessed define dump `BICOPTER_PICO2_defines.txt`.

## 8. Software tests (host, MinGW gcc 16.2; all pass)

- `tests/test_link.c`: the protocol core (about 1.5 million checks).
  - Encode/decode round trip, the known vector, and every rejection class.
  - All 160 single-bit errors are detected; 1 of 200,000 random multi-byte corruptions passed
    the CRC (inherent to a 16-bit CRC; the radio CRC is in front of it).
  - Address and channel derivation over 100,000 IDs.
  - All freshness rules, including wrap, restart, frozen acquisition and TX fault.
  - Hop simulation:
    - worst acquisition within 4 dwells;
    - ±500 ppm clock error with a 1 kHz loop: 100% received;
    - 30% loss: 68% received;
    - reacquisition 17 ms after a 2 s outage;
    - FIFO-late packet, delayed call and µs-timer wrap cases.
- `tests/test_rx_adapter.c`: the **real `nrf24_bicopter.c`** against an nRF24 register/FIFO/IRQ
  model, in three modes (IRQ at 4 kHz, IRQ at 1 kHz, polled at 4 kHz).
  - Register setup and address checked; radio absent → init returns false.
  - 599/600 frames received on a clean link.
  - Frozen sequence, TX fault and foreign link ID: no fresh frames.
  - First fresh frame about 25 ms after a handset restart.
  - 25% loss: maximum gap 25 ms (below the 150 ms RXLOSS threshold).
  - Corrupt width → flush; three queued packets are read one per call; spurious-IRQ back-off;
    at most 5 SPI operations per call.
- `handset/test/test_handset.c`: `handset_app.c` with a fake HAL and SPI decoder.
  - Every packet decodes with the RX core and uses the hop order.
  - About 200 accepted frames per second while sampling; none once sampling freezes, and the
    fault flag is set after 50 ms.
  - Stick-at-rail detection.
- `handset/shared/` is byte-identical to the Betaflight copy (checked with `cmp`).

Run (from the `tests` folder):
`gcc -std=c11 -O2 -I../firmware/src-changed/src/main -o t test_link.c ../firmware/src-changed/src/main/rx/nrf24_bicopter_link.c`.
For the adapter test add `-Istubs` before that include path, plus `test_rx_adapter.c` and `nrf24_bicopter.c`.

## 9. Handset firmware: what exists and what is missing

These files exist and are tested on the host; they also compile warning-free for Cortex-M3:
- `handset/src/handset_app.c/.h`: nRF24 TX setup, 5 ms frame scheduler, hop order, session,
  sequence-per-acquisition, input fault (no sample for 50 ms, stick at rail, invalid switch),
  stick calibration mapping, ACK statistics.
- `handset/config/link_config.h`: link ID, TX power, channel order (AETR + AUX1 = arm, AUX2–4).
- `handset/shared/`: the protocol core.

**Not written: the STM32F103 board layer that implements `handset/src/handset_hal.h`.** It is
blocked on these missing inputs:
1. Exact board (Blue Pill or other; HSE crystal value; whether it is a genuine STM32F103C8 or a clone).
2. Existing handset source, if any, and the preferred toolchain/framework (STM32CubeIDE/HAL,
   PlatformIO/Arduino STM32duino, libopencm3, or bare CMSIS).
3. Programmer (ST-Link, USB-serial bootloader on PA9/PA10, or DFU).
4. Handset wiring:
   - nRF24 SCK, MOSI, MISO, CSN, CE (IRQ optional on the TX);
   - which ADC pins carry which stick;
   - switch pins and types (2- or 3-position);
   - an LED;
   - the radio power arrangement.
5. Stick calibration (min/center/max) and channel directions.

The HAL needs these functions:
- `hal_micros`;
- `hal_take_new_inputs`, which must return true once per completed ADC/DMA acquisition and never
  for repeated old data;
- `hal_boot_random_nonzero`, `hal_spi_xfer` (mode 0, at most 8 MHz), `hal_nrf_csn`, `hal_nrf_ce`,
  `hal_delay_us`, `hal_led`.

Main loop: `hs_init(cal)` once, then `hs_poll()` continuously.

## 10. Proposed wiring

All 3.3 V logic. These are proposals; confirm them against the real harness.

| Function | Pico GPIO | Pin |
|---|---|---|
| MPU6500 SCK / MOSI / MISO / CS | GP2 / GP3 / GP4 / GP5 (SPI0) | 4 / 5 / 6 / 7 |
| MPU6500 INT | GP22 | 29 |
| Left ESC (M1) / right ESC (M2) | GP10 / GP11 | 14 / 15 |
| Left servo (S1) / right servo (S2) | GP20 / GP21 | 26 / 27 |
| nRF24 SCK / MOSI / MISO / CSN | GP14 / GP15 / GP12 / GP13 (SPI1) | 19 / 20 / 16 / 17 |
| nRF24 CE / IRQ | GP16 / GP17 | 21 / 22 |
| VBAT divider (optional) | GP26 (ADC0), at most 3.3 V at 12.6 V | 31 |

- Radio power: PA/LNA module on its own 3.3 V regulator sized for its TX peaks (≥ 250 mA
  recommended), 10–100 µF close to the module, common ground. Do not power it from the Pico's
  3V3 pin without checking its budget.
- Servos: separate 5–6 V supply, ground common with the Pico, signal only from the Pico.
- MPU6500 breakout: check whether it has its own regulator (VCC 3.3 V vs 5 V input) before connecting.
- GP6/GP7 are listed in the Pico UART1 driver table, so they work as a fallback UART1;
  this is not hardware-tested.

## 11. Betaflight configuration (CLI after flashing)

Defaults already compiled in:
- BI mixer;
- `FEATURE_RX_SPI`;
- `rx_spi_protocol = NRF24_BICOPTER`;
- `motor_pwm_protocol = PWM`, `motor_pwm_rate = 200`;
- motor pins GP10/11, servo pins GP20/21;
- DShot telemetry off.

Paste and check, then run `save`:
```
set rx_spi_protocol = NRF24_BICOPTER
set motor_pwm_protocol = PWM
set motor_pwm_rate = 200
set dshot_bidir = OFF
set dyn_idle_min_rpm = 0
set failsafe_procedure = DROP
set failsafe_delay = 5
set debug_mode = RX_NRF24_BICOPTER
aux 0 0 0 1700 2100 0 0
save
```
- The `aux` line sets ARM on AUX1 high (handset channel 5).
- Check these in `get` and `resource` output: `rx_spi_bus` should read 2, `RX_SPI_CS` should be
  A13, `RX_SPI_EXTI` A17, `MOTOR 1/2` A10/A11, `SERVO 1/2` A20/A21.
- Calibrate the ESC end points with props off, using the SimonK procedure and the motors tab.
- Set `min_check`, `max_check` and `rxrange` after watching the receiver tab.
- Set servo `servo` and `smix` directions on the real aircraft.
- Set `align_board_roll`, `align_board_pitch` and `align_board_yaw` after checking the IMU orientation in the 3D view.
- Set `vbat_scale` if a divider is fitted; otherwise `set battery_meter = NONE`.

## 12. Flashing

**Option 1: Betaflight App (official configurator)**
1. Hold BOOTSEL and plug in the Pico 2.
2. In the Firmware Flasher, choose "Load Firmware [Local]" and pick the `.uf2`. Recent App
   versions support RP2350 UF2 / mass-storage flashing; if yours does not, use option 2.

**Option 2: BOOTSEL drag-and-drop**
1. Hold BOOTSEL while plugging in USB. The `RP2350` drive appears.
2. Copy the `.uf2` onto it. The board reboots and enumerates as "Betaflight - RP2350A" (USB VCP).

Flashing does not erase the config sector unless the image overlaps it. After the first flash,
run `defaults` (or a full erase) to load the board defaults.

## 13. Props-off hardware test checklist

**Stage 0: radio presence**
1. Set `debug_mode = RX_NRF24_BICOPTER` and reboot.
2. Sensors tab: debug[7] must be 0. Non-zero means SPI wiring, radio power or the clock is wrong.
3. Measure SCK with a scope or logic analyser: it should be ≤ 1 MHz.

**Stage 1: gyro**
- The Setup tab shows MPU6500; the 3D model follows rotation in all axes.
- Check the loop time in the Status bar.

**Stage 2: reception**
1. With the handset on: debug[4] high byte = 1 (synced), debug[0] increasing about 200/s,
   debug[1] near 0.
2. Receiver tab: each stick moves only its channel, in the right direction, with 1000/1500/2000
   end points.
3. Switches: AUX1–4.
4. Leave it running for 10+ minutes and check that reception never drops (repeated reception).

**Stage 3: link loss**
1. Switch the handset off. Within about 150 ms the receiver tab or OSD shows RXLOSS / failsafe.
2. Disarmed: arming must be blocked.
3. Armed on the bench with props off: the motors must stop (DROP procedure). Measure the time
   with a video or logic analyser on the ESC signal.

**Stage 4: stale and frozen samples**
- Use a handset debug build that stops calling `hal_take_new_inputs()` (freeze) while still
  transmitting. Expected: RXLOSS within about 150 ms; debug[2] rising (duplicates).
- With a fault-flag build, debug[3] rises.

**Stage 5: handset reset**
1. Reset the handset while the FC is disarmed. Reception resumes within about 50 ms of the
   handset transmitting again.
2. While armed (props off): failsafe triggers. After the link returns, arming stays blocked until
   the arm switch is cycled. Verify this explicitly.

**Stage 6: recovery and range**
- Shadow, walk away and come back; the link reacquires.
- Bench test at low TX power (`HANDSET_RF_PWR_BITS`), with the modules at least 1–2 m apart.

**Stage 7: outputs** (props off, or motors disconnected for the servo checks)
1. Servos centre at 1500 at 50 Hz (scope).
2. Pitch input tilts both nacelles in the same physical direction; yaw tilts them in opposite directions.
3. The ESCs arm at 200 Hz PWM. The roll command raises the left motor for a right roll.

**Stage 8: SPI clock**
- Optionally raise `NRF24L01_MAX_SPI_CLK_HZ` (e.g. 4–8 MHz) after stage 2 passes; this needs a rebuild.

## 14. Status

| Item | Status |
|---|---|
| Firmware compiles and links; UF2 produced; clean rebuild reproduced; flash layout and preprocessed features checked | Compiled |
| Protocol core, real RX adapter against a radio model, handset logic; shared code identical on both ends | Software-tested on PC |
| Flashing, gyro, real nRF24 RF behaviour (PA/LNA, timing, CE retune, ACK), IRQ edges on real pins, ESC acceptance of 200 Hz, servo directions, IMU orientation, failsafe timing, re-arm blocking, range | **Not verified on hardware** |
| STM32F103 board layer (`handset_hal.h` implementation) | **Not written** (inputs missing) |
| Flight readiness | **Not established** |

## 15. Open items for the next engineer

1. Implement the STM32F103 HAL once the inputs in §9 are known.
2. Optionally commit the work in the Betaflight branch and the config submodule. Nothing is
   committed yet; the work lives in the patches and in the working tree.
3. After a stable link, consider an ACK-payload telemetry channel (battery, armed state) and a
   CLI-settable link ID.
4. Review `failsafe_delay` and the procedure for the real aircraft.
