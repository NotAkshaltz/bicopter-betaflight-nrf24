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

---

# Appendix A: Betaflight patch (apply in the betaflight repo root at tag 2026.6.2)
```diff
diff --git a/mk/source.mk b/mk/source.mk
index 5dd11e1d2..dffb21d99 100644
--- a/mk/source.mk
+++ b/mk/source.mk
@@ -203,6 +203,8 @@ COMMON_SRC = \
             rx/rx_bind.c \
             rx/rx_spi.c \
             rx/rx_spi_common.c \
+            rx/nrf24_bicopter.c \
+            rx/nrf24_bicopter_link.c \
             rx/crsf.c \
             rx/ghst.c \
             rx/sbus.c \
diff --git a/src/main/build/debug.c b/src/main/build/debug.c
index f92330957..7709729ab 100644
--- a/src/main/build/debug.c
+++ b/src/main/build/debug.c
@@ -128,4 +128,5 @@ const char * const debugModeNames[DEBUG_COUNT] = {
     [DEBUG_MAVLINK_TELEMETRY] = "MAVLINK_TELEMETRY",
     [DEBUG_AUTOPILOT_PID] = "AUTOPILOT_PID",
     [DEBUG_AUTOPILOT_STOP] = "AUTOPILOT_STOP",
+    [DEBUG_RX_NRF24_BICOPTER] = "RX_NRF24_BICOPTER",
 };
diff --git a/src/main/build/debug.h b/src/main/build/debug.h
index dd6985f7b..ac8475c2a 100644
--- a/src/main/build/debug.h
+++ b/src/main/build/debug.h
@@ -131,6 +131,7 @@ typedef enum {
     DEBUG_AUTOPILOT_PID,
     DEBUG_POSITION_NAV,
     DEBUG_AUTOPILOT_STOP,
+    DEBUG_RX_NRF24_BICOPTER,
     DEBUG_COUNT
 } debugType_e;
 
diff --git a/src/main/cli/settings.c b/src/main/cli/settings.c
index 489d254f7..0ffec7a66 100644
--- a/src/main/cli/settings.c
+++ b/src/main/cli/settings.c
@@ -292,6 +292,7 @@ static const char * const lookupTableRxSpi[] = {
     "FRSKY_X_V2",
     "FRSKY_X_LBT_V2",
     "EXPRESSLRS",
+    "NRF24_BICOPTER",
 };
 #endif
 
diff --git a/src/main/drivers/rx/rx_nrf24l01.c b/src/main/drivers/rx/rx_nrf24l01.c
index 9e14fc318..515aafa10 100644
--- a/src/main/drivers/rx/rx_nrf24l01.c
+++ b/src/main/drivers/rx/rx_nrf24l01.c
@@ -35,6 +35,7 @@
 
 #include "drivers/bus_spi.h"
 #include "drivers/io.h"
+#include "drivers/io_impl.h"    // DEFIO_IO() needs ioRecs (not pulled in indirectly on PICO)
 #include "drivers/rx/rx_spi.h"
 #include "drivers/time.h"
 
@@ -43,6 +44,16 @@
 #define NRF24_CE_HI()   {IOHi(DEFIO_IO(RX_CE_PIN));}
 #define NRF24_CE_LO()   {IOLo(DEFIO_IO(RX_CE_PIN));}
 
+// nRF24L01+ product specification: SPI clock 0..10 MHz. The generic RX SPI driver
+// defaults to 13.5 MHz, which is out of specification for this radio. A board config
+// may lower this further (e.g. for bring-up with long jumper wires).
+#ifndef NRF24L01_MAX_SPI_CLK_HZ
+#define NRF24L01_MAX_SPI_CLK_HZ 8000000
+#endif
+#if NRF24L01_MAX_SPI_CLK_HZ > 10000000
+#error "NRF24L01_MAX_SPI_CLK_HZ exceeds the nRF24L01+ 10 MHz SPI limit"
+#endif
+
 // Instruction Mnemonics
 // nRF24L01:  Table 16. Command set for the nRF24L01 SPI. Product Specification, p46
 // nRF24L01+: Table 20. Command set for the nRF24L01+ SPI. Product Specification, p51
@@ -138,6 +149,9 @@ static uint8_t standbyConfig;
 void NRF24L01_Initialize(uint8_t baseConfig)
 {
     standbyConfig = BIT(NRF24L01_00_CONFIG_PWR_UP) | baseConfig;
+    // Limit the SPI clock before the first transfer to the radio (all NRF24 protocols use this path)
+    rxSpiSetNormalSpeedMhz(NRF24L01_MAX_SPI_CLK_HZ);
+    rxSpiNormalSpeed();
     NRF24L01_InitGpio();
     // nRF24L01+ needs 100 milliseconds settling time from PowerOnReset to PowerDown mode
     static const uint32_t settlingTimeUs = 100000;
@@ -214,6 +228,26 @@ void NRF24L01_SetChannel(uint8_t channel)
     NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel);
 }
 
+/*
+ * Retune while receiving: drop to standby (CE low), change RF_CH, re-enter RX mode.
+ * The radio needs 130us settling before it can receive on the new channel.
+ */
+void NRF24L01_SetChannelRxMode(uint8_t channel)
+{
+    NRF24_CE_LO();
+    NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel);
+    NRF24_CE_HI();
+}
+
+/*
+ * Width of the payload at the top of the RX FIFO (dynamic payload length).
+ * A value above 32 indicates a corrupt packet and the RX FIFO must be flushed.
+ */
+uint8_t NRF24L01_ReadPayloadWidth(void)
+{
+    return rxSpiReadCommand(R_RX_PL_WID, NOP);
+}
+
 bool NRF24L01_ReadPayloadIfAvailable(uint8_t *data, uint8_t length)
 {
     if (NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BIT(NRF24L01_17_FIFO_STATUS_RX_EMPTY)) {
diff --git a/src/main/drivers/rx/rx_nrf24l01.h b/src/main/drivers/rx/rx_nrf24l01.h
index 9db0ce2fc..0c5e1cba8 100644
--- a/src/main/drivers/rx/rx_nrf24l01.h
+++ b/src/main/drivers/rx/rx_nrf24l01.h
@@ -196,4 +196,6 @@ void NRF24L01_SetRxMode(void);
 void NRF24L01_SetTxMode(void);
 void NRF24L01_ClearAllInterrupts(void);
 void NRF24L01_SetChannel(uint8_t channel);
+void NRF24L01_SetChannelRxMode(uint8_t channel);
+uint8_t NRF24L01_ReadPayloadWidth(void);
 bool NRF24L01_ReadPayloadIfAvailable(uint8_t *data, uint8_t length);
diff --git a/src/main/rx/nrf24_bicopter.c b/src/main/rx/nrf24_bicopter.c
new file mode 100644
index 000000000..21b7197bd
--- /dev/null
+++ b/src/main/rx/nrf24_bicopter.c
@@ -0,0 +1,361 @@
+/*
+ * This file is part of Betaflight.
+ *
+ * Betaflight is free software. You can redistribute this software
+ * and/or modify this software under the terms of the GNU General
+ * Public License as published by the Free Software Foundation,
+ * either version 3 of the License, or (at your option) any later
+ * version.
+ *
+ * Betaflight is distributed in the hope that it will be useful,
+ * but WITHOUT ANY WARRANTY; without even the implied warranty of
+ * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
+ *
+ * See the GNU General Public License for more details.
+ *
+ * You should have received a copy of the GNU General Public
+ * License along with this software.
+ *
+ * If not, see <http://www.gnu.org/licenses/>.
+ */
+
+/*
+ * NRF24_BICOPTER receiver: Betaflight RX_SPI protocol adapter for an nRF24L01+.
+ *
+ * Derived in spirit from rx/nrf24_inav.c (same radio settings: 250 kbps, 5-byte
+ * address, 2-byte CRC, dynamic payload, auto-ACK) but with a project specific
+ * packet format and link state machine; see rx/nrf24_bicopter_link.h.
+ *
+ * Runtime behaviour (called from rxFrameCheck() once per scheduler loop):
+ *  - Idle cost: one GPIO read (IRQ wired) or one STATUS read every
+ *    BICOPTER_NRF24_POLL_INTERVAL_US (IRQ not wired), plus a time comparison.
+ *  - At most ONE payload is read per call (bounded SPI traffic); if more are
+ *    waiting in the RX FIFO they are read on following calls.
+ *  - At most one retune per call.
+ *  - Only fresh, valid control samples return RX_SPI_RECEIVED_DATA, which is the
+ *    only thing that refreshes Betaflight's RX signal watchdog.
+ */
+
+#include <stdbool.h>
+#include <stdint.h>
+#include <string.h>
+
+#include "platform.h"
+
+#ifdef USE_RX_NRF24_BICOPTER
+
+#include "build/debug.h"
+
+#include "common/utils.h"
+
+#include "drivers/io.h"
+#include "drivers/rx/rx_nrf24l01.h"
+#include "drivers/rx/rx_spi.h"
+#include "drivers/time.h"
+
+#include "rx/rx.h"
+#include "rx/rx_spi.h"
+#include "rx/nrf24_bicopter_link.h"
+#include "rx/nrf24_bicopter.h"
+
+#ifndef BICOPTER_NRF24_LINK_ID
+#error "BICOPTER_NRF24_LINK_ID must be defined in the board config (same value as the handset firmware)"
+#endif
+
+#ifndef BICOPTER_NRF24_RX_RF_PWR
+// Only used for the auto-ACK transmitted by the receiver.
+#define BICOPTER_NRF24_RX_RF_PWR NRF24L01_06_RF_SETUP_RF_PWR_n12dbm
+#endif
+
+#ifndef BICOPTER_NRF24_POLL_INTERVAL_US
+#define BICOPTER_NRF24_POLL_INTERVAL_US 500     // STATUS poll period when no IRQ pin is configured
+#endif
+
+#define BICOPTER_NRF24_SPURIOUS_BACKOFF_US 1000 // IRQ low but FIFO empty: re-check the level after this
+
+STATIC_ASSERT(BL_PAYLOAD_LEN <= RX_SPI_MAX_PAYLOAD_SIZE, bicopter_payload_too_large);
+STATIC_ASSERT(BL_CHANNEL_COUNT <= MAX_SUPPORTED_RC_CHANNEL_COUNT, bicopter_too_many_channels);
+
+#define NRF24_STATUS_RX_P_NO_SHIFT  1
+#define NRF24_STATUS_RX_P_NO_MASK   0x07
+#define NRF24_RX_P_NO_FIFO_EMPTY    7
+#define NRF24_ACTIVATE_FEATURES     0x73
+
+typedef enum {
+    INIT_OK = 0,
+    INIT_FAIL_SETUP_AW = 1,
+    INIT_FAIL_FEATURE = 2,
+    INIT_FAIL_REGISTERS = 3,
+    INIT_FAIL_ADDRESS = 4,
+    INIT_FAIL_CONFIG = 5,
+} initStatus_e;
+
+static uint32_t linkId;
+static uint8_t rfChannels[BL_RF_CHANNEL_COUNT];
+static uint8_t rxAddr[BL_ADDR_LEN];
+
+static blFreshness_t freshness;
+static blHop_t hop;
+static nrf24BicopterStats_t stats;
+
+static bool irqConfigured;
+static bool fifoPending;
+static bool havePendingExtiTime;
+static timeUs_t pendingExtiTimeUs;
+static timeUs_t nextPollUs;
+static timeUs_t nextLevelCheckUs;
+static timeUs_t lastFrameTimeUs;
+static initStatus_e initStatus;
+
+static void tuneToHopChannel(void)
+{
+    NRF24L01_SetChannelRxMode(rfChannels[hop.idx]);
+}
+
+static bool writeRegVerify(uint8_t reg, uint8_t value)
+{
+    NRF24L01_WriteReg(reg, value);
+    return NRF24L01_ReadReg(reg) == value;
+}
+
+static void updateDebug(void)
+{
+    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 0, (int16_t)stats.accepted);
+    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 1, (int16_t)(stats.malformed + stats.fifoFlush));
+    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 2, (int16_t)(stats.duplicate + stats.stale));
+    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 3, (int16_t)(stats.resyncPending + stats.txFault));
+    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 4, (int16_t)((hop.state << 8) | rfChannels[hop.idx]));
+    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 5, (int16_t)hop.missedSlots);
+    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 6, (int16_t)stats.spurious);
+    DEBUG_SET(DEBUG_RX_NRF24_BICOPTER, 7, (int16_t)initStatus);
+}
+
+static rx_spi_received_e serviceRadio(uint8_t *payload, timeUs_t nowUs)
+{
+    const uint8_t status = NRF24L01_ReadReg(NRF24L01_07_STATUS);
+    const uint8_t pipe = (status >> NRF24_STATUS_RX_P_NO_SHIFT) & NRF24_STATUS_RX_P_NO_MASK;
+
+    if (pipe == NRF24_RX_P_NO_FIFO_EMPTY) {
+        if (status & BIT(NRF24L01_07_STATUS_RX_DR)) {
+            NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR));
+        }
+        fifoPending = false;
+        havePendingExtiTime = false;
+        nextLevelCheckUs = nowUs + BICOPTER_NRF24_SPURIOUS_BACKOFF_US;
+        stats.spurious++;
+        return RX_SPI_RECEIVED_NONE;
+    }
+
+    uint8_t buf[NRF24L01_MAX_PAYLOAD_SIZE];
+    const uint8_t width = NRF24L01_ReadPayloadWidth();
+    const bool widthOk = (width >= 1) && (width <= NRF24L01_MAX_PAYLOAD_SIZE);
+    if (widthOk) {
+        NRF24L01_ReadPayload(buf, width);
+    } else {
+        // nRF24L01+ product specification: a width above 32 means a corrupt packet; flush RX FIFO
+        NRF24L01_FlushRx();
+        stats.fifoFlush++;
+    }
+    // Clear RX_DR only after the payload has been read, then look at what is left
+    NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR));
+    fifoPending = widthOk && !(NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS) & BIT(NRF24L01_17_FIFO_STATUS_RX_EMPTY));
+
+    if (!widthOk) {
+        havePendingExtiTime = false;
+        return RX_SPI_RECEIVED_NONE;
+    }
+
+    // The EXTI timestamp belongs to the most recently received packet, so it is only
+    // attributed to the packet that empties the FIFO.
+    bool timeReliable = false;
+    timeUs_t packetTimeUs = nowUs;
+    if (!fifoPending) {
+        if (irqConfigured) {
+            timeReliable = havePendingExtiTime;
+            if (havePendingExtiTime) {
+                packetTimeUs = pendingExtiTimeUs;
+            }
+        } else {
+            timeReliable = true;    // polled: error bounded by the poll interval
+        }
+        havePendingExtiTime = false;
+    }
+
+    blControl_t control;
+    if (pipe != NRF24L01_PIPE0 || blDecode(linkId, buf, width, &control) != BL_DECODE_OK) {
+        stats.malformed++;
+        return RX_SPI_RECEIVED_NONE;
+    }
+
+    // Authentic packet from our handset: it keeps the hop timing aligned even if the
+    // sample itself is not fresh.
+    blHopOnPacket(&hop, packetTimeUs, timeReliable);
+
+    switch (blFreshnessCheck(&freshness, control.session, control.sampleSeq, control.flags)) {
+    case BL_FRESH_ACCEPT:
+        break;
+    case BL_FRESH_DUPLICATE:
+        stats.duplicate++;
+        return RX_SPI_RECEIVED_NONE;
+    case BL_FRESH_STALE:
+        stats.stale++;
+        return RX_SPI_RECEIVED_NONE;
+    case BL_FRESH_TX_FAULT:
+        stats.txFault++;
+        return RX_SPI_RECEIVED_NONE;
+    case BL_FRESH_RESYNC_PENDING:
+    default:
+        stats.resyncPending++;
+        return RX_SPI_RECEIVED_NONE;
+    }
+
+    // Copy into the shared RX payload buffer only once the packet is accepted, so a
+    // later rejected packet can never overwrite data that is waiting to be decoded.
+    memcpy(payload, buf, BL_PAYLOAD_LEN);
+    lastFrameTimeUs = packetTimeUs;
+    stats.accepted++;
+    return RX_SPI_RECEIVED_DATA;
+}
+
+rx_spi_received_e nrf24BicopterDataReceived(uint8_t *payload)
+{
+    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
+    const timeUs_t nowUs = micros();
+    bool service = fifoPending;
+
+    if (irqConfigured) {
+        if (rxSpiPollExti()) {
+            // Falling edge on IRQ (RX_DR asserted)
+            rxSpiResetExti();
+            pendingExtiTimeUs = rxSpiGetLastExtiTimeUs();
+            havePendingExtiTime = true;
+            service = true;
+        } else if (!rxSpiGetExtiState() && cmpTimeUs(nowUs, nextLevelCheckUs) >= 0) {
+            // IRQ is active-low and still asserted without a new edge (missed edge)
+            service = true;
+        }
+    } else if (cmpTimeUs(nowUs, nextPollUs) >= 0) {
+        nextPollUs = nowUs + BICOPTER_NRF24_POLL_INTERVAL_US;
+        service = true;
+    }
+
+    if (service) {
+        ret = serviceRadio(payload, nowUs);
+    }
+
+    if (blHopTick(&hop, micros())) {
+        tuneToHopChannel();
+    }
+
+    updateDebug();
+    return ret;
+}
+
+void nrf24BicopterSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload)
+{
+    blControl_t control;
+    // payload was validated before it was accepted; decode again to unpack it
+    if (blDecode(linkId, payload, BL_PAYLOAD_LEN, &control) != BL_DECODE_OK) {
+        return;
+    }
+    for (int ch = 0; ch < BL_CHANNEL_COUNT; ch++) {
+        rcData[ch] = control.channelUs[ch];
+    }
+}
+
+timeUs_t nrf24BicopterFrameTimeUs(void)
+{
+    return lastFrameTimeUs;
+}
+
+const nrf24BicopterStats_t *nrf24BicopterGetStats(void)
+{
+    return &stats;
+}
+
+static initStatus_e nrf24BicopterSetup(void)
+{
+    // PWR_UP, 2-byte CRC; IRQ pin reports RX_DR only
+    NRF24L01_Initialize(BIT(NRF24L01_00_CONFIG_EN_CRC) | BIT(NRF24L01_00_CONFIG_CRCO) |
+                        BIT(NRF24L01_00_CONFIG_MASK_MAX_RT) | BIT(NRF24L01_00_CONFIG_MASK_TX_DS));
+
+    // A wrong value here usually means SPI wiring, clock or radio power is wrong
+    if (!writeRegVerify(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES)) {
+        return INIT_FAIL_SETUP_AW;
+    }
+
+    // nRF24L01+ has FEATURE always accessible; original nRF24L01 and some clones need ACTIVATE
+    if (!writeRegVerify(NRF24L01_1D_FEATURE, BIT(NRF24L01_1D_FEATURE_EN_DPL))) {
+        NRF24L01_Activate(NRF24_ACTIVATE_FEATURES);
+        if (!writeRegVerify(NRF24L01_1D_FEATURE, BIT(NRF24L01_1D_FEATURE_EN_DPL))) {
+            return INIT_FAIL_FEATURE;
+        }
+    }
+
+    bool ok = true;
+    ok &= writeRegVerify(NRF24L01_01_EN_AA, BIT(NRF24L01_01_EN_AA_ENAA_P0));
+    ok &= writeRegVerify(NRF24L01_02_EN_RXADDR, BIT(NRF24L01_02_EN_RXADDR_ERX_P0));
+    ok &= writeRegVerify(NRF24L01_04_SETUP_RETR, 0);
+    ok &= writeRegVerify(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | BICOPTER_NRF24_RX_RF_PWR);
+    ok &= writeRegVerify(NRF24L01_1C_DYNPD, BIT(NRF24L01_1C_DYNPD_DPL_P0));
+    ok &= writeRegVerify(NRF24L01_11_RX_PW_P0, NRF24L01_MAX_PAYLOAD_SIZE);  // ignored with DPL
+    ok &= writeRegVerify(NRF24L01_05_RF_CH, rfChannels[hop.idx]);
+    if (!ok) {
+        return INIT_FAIL_REGISTERS;
+    }
+
+    uint8_t readBack[BL_ADDR_LEN];
+    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxAddr, BL_ADDR_LEN);
+    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rxAddr, BL_ADDR_LEN);  // ACKs use pipe 0 address
+    NRF24L01_ReadRegisterMulti(NRF24L01_0A_RX_ADDR_P0, readBack, BL_ADDR_LEN);
+    if (memcmp(readBack, rxAddr, BL_ADDR_LEN) != 0) {
+        return INIT_FAIL_ADDRESS;
+    }
+
+    NRF24L01_FlushTx();
+    NRF24L01_FlushRx();
+    NRF24L01_SetRxMode();   // clears interrupts, CE high
+
+    const uint8_t expectedConfig = BIT(NRF24L01_00_CONFIG_EN_CRC) | BIT(NRF24L01_00_CONFIG_CRCO) |
+        BIT(NRF24L01_00_CONFIG_MASK_MAX_RT) | BIT(NRF24L01_00_CONFIG_MASK_TX_DS) |
+        BIT(NRF24L01_00_CONFIG_PWR_UP) | BIT(NRF24L01_00_CONFIG_PRIM_RX);
+    if (NRF24L01_ReadReg(NRF24L01_00_CONFIG) != expectedConfig) {
+        return INIT_FAIL_CONFIG;
+    }
+    return INIT_OK;
+}
+
+bool nrf24BicopterInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
+{
+    UNUSED(rxSpiConfig);    // the link ID is fixed at build time, rx_spi_id is not used
+
+    linkId = BICOPTER_NRF24_LINK_ID;
+    blDeriveAddress(linkId, rxAddr);
+    blDeriveRfChannels(linkId, rfChannels);
+    blFreshnessInit(&freshness);
+    memset(&stats, 0, sizeof(stats));
+    fifoPending = false;
+    havePendingExtiTime = false;
+    lastFrameTimeUs = 0;
+
+    rxRuntimeState->channelCount = BL_CHANNEL_COUNT;
+
+    // nRF24L01+ IRQ is active low. rxSpiExtiInit() is called by rxSpiInit() after this.
+    extiConfig->ioConfig = IOCFG_IPU;
+    extiConfig->trigger = BETAFLIGHT_EXTI_TRIGGER_FALLING;
+    irqConfigured = rxSpiExtiConfigured();
+
+    blHopInit(&hop, micros());
+    initStatus = nrf24BicopterSetup();
+    // The search dwell starts once the radio is actually listening
+    blHopInit(&hop, micros());
+
+    const timeUs_t nowUs = micros();
+    nextPollUs = nowUs;
+    nextLevelCheckUs = nowUs;
+
+    updateDebug();
+    return initStatus == INIT_OK;
+}
+
+#endif // USE_RX_NRF24_BICOPTER
diff --git a/src/main/rx/nrf24_bicopter.h b/src/main/rx/nrf24_bicopter.h
new file mode 100644
index 000000000..e5c2fe84f
--- /dev/null
+++ b/src/main/rx/nrf24_bicopter.h
@@ -0,0 +1,45 @@
+/*
+ * This file is part of Betaflight.
+ *
+ * Betaflight is free software. You can redistribute this software
+ * and/or modify this software under the terms of the GNU General
+ * Public License as published by the Free Software Foundation,
+ * either version 3 of the License, or (at your option) any later
+ * version.
+ *
+ * Betaflight is distributed in the hope that it will be useful,
+ * but WITHOUT ANY WARRANTY; without even the implied warranty of
+ * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
+ *
+ * See the GNU General Public License for more details.
+ *
+ * You should have received a copy of the GNU General Public
+ * License along with this software.
+ *
+ * If not, see <http://www.gnu.org/licenses/>.
+ */
+
+#pragma once
+
+#include <stdint.h>
+
+#include "common/time.h"
+
+#include "rx/rx_spi.h"
+
+typedef struct nrf24BicopterStats_s {
+    uint32_t accepted;      // fresh control samples handed to the RX layer
+    uint32_t malformed;     // bad width/length/type/version/tag/CRC/flags/range/session
+    uint32_t duplicate;
+    uint32_t stale;
+    uint32_t resyncPending;
+    uint32_t txFault;
+    uint32_t spurious;      // IRQ/poll found an empty RX FIFO
+    uint32_t fifoFlush;     // invalid payload width -> RX FIFO flushed
+} nrf24BicopterStats_t;
+
+bool nrf24BicopterInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig);
+rx_spi_received_e nrf24BicopterDataReceived(uint8_t *payload);
+void nrf24BicopterSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload);
+timeUs_t nrf24BicopterFrameTimeUs(void);
+const nrf24BicopterStats_t *nrf24BicopterGetStats(void);
diff --git a/src/main/rx/nrf24_bicopter_link.c b/src/main/rx/nrf24_bicopter_link.c
new file mode 100644
index 000000000..b34c3ace6
--- /dev/null
+++ b/src/main/rx/nrf24_bicopter_link.c
@@ -0,0 +1,317 @@
+/*
+ * This file is part of Betaflight.
+ *
+ * Betaflight is free software. You can redistribute this software
+ * and/or modify this software under the terms of the GNU General
+ * Public License as published by the Free Software Foundation,
+ * either version 3 of the License, or (at your option) any later
+ * version.
+ *
+ * Betaflight is distributed in the hope that it will be useful,
+ * but WITHOUT ANY WARRANTY; without even the implied warranty of
+ * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
+ *
+ * See the GNU General Public License for more details.
+ *
+ * You should have received a copy of the GNU General Public
+ * License along with this software.
+ *
+ * If not, see <http://www.gnu.org/licenses/>.
+ */
+
+/*
+ * NRF24_BICOPTER link protocol v1 - hardware independent core.
+ * Shared verbatim with the handset firmware; see nrf24_bicopter_link.h.
+ */
+
+#include <string.h>
+
+#include "nrf24_bicopter_link.h"
+
+#define BL_OFFSET_TAG       2
+#define BL_OFFSET_SESSION   4
+#define BL_OFFSET_SEQ       5
+#define BL_OFFSET_CHANNELS  7
+#define BL_OFFSET_FLAGS     17
+#define BL_OFFSET_CRC       18
+#define BL_CHANNEL_BITS     10
+
+uint32_t blMix32(uint32_t x)
+{
+    // "lowbias32" integer hash: good avalanche, identical on every 32-bit C target
+    x ^= x >> 16;
+    x *= 0x7feb352dU;
+    x ^= x >> 15;
+    x *= 0x846ca68bU;
+    x ^= x >> 16;
+    return x;
+}
+
+uint16_t blLinkTag(uint32_t linkId)
+{
+    return (uint16_t)(blMix32(linkId) >> 16);
+}
+
+uint16_t blCrcSeed(uint32_t linkId)
+{
+    uint16_t seed = (uint16_t)(blMix32(linkId ^ 0xC0FFEE11U) & 0xFFFFU);
+    if (seed == 0) {
+        seed = 0x1D0F;
+    }
+    return seed;
+}
+
+uint16_t blCrc16(uint16_t crc, const uint8_t *data, size_t len)
+{
+    // CRC-16/CCITT, polynomial 0x1021, MSB first, no reflection, no final xor
+    for (size_t i = 0; i < len; i++) {
+        crc ^= (uint16_t)data[i] << 8;
+        for (int bit = 0; bit < 8; bit++) {
+            if (crc & 0x8000) {
+                crc = (uint16_t)((crc << 1) ^ 0x1021);
+            } else {
+                crc = (uint16_t)(crc << 1);
+            }
+        }
+    }
+    return crc;
+}
+
+static uint8_t blFixAddressByte(uint8_t b)
+{
+    // Avoid address bytes with no or only regular bit transitions; 0x55/0xAA also
+    // resemble the radio preamble.
+    if (b == 0x00 || b == 0xFF || b == 0x55 || b == 0xAA) {
+        b ^= 0x3C;
+    }
+    return b;
+}
+
+void blDeriveAddress(uint32_t linkId, uint8_t addr[BL_ADDR_LEN])
+{
+    const uint32_t a = blMix32(linkId ^ 0xA5C3B1E7U);
+    const uint32_t b = blMix32(a ^ linkId);
+    addr[0] = blFixAddressByte((uint8_t)(a & 0xFF));
+    addr[1] = blFixAddressByte((uint8_t)((a >> 8) & 0xFF));
+    addr[2] = blFixAddressByte((uint8_t)((a >> 16) & 0xFF));
+    addr[3] = blFixAddressByte((uint8_t)((a >> 24) & 0xFF));
+    addr[4] = blFixAddressByte((uint8_t)(b & 0xFF));
+}
+
+void blDeriveRfChannels(uint32_t linkId, uint8_t channels[BL_RF_CHANNEL_COUNT])
+{
+    const uint8_t offset = (uint8_t)(blMix32(linkId ^ 0x5EED5EEDU) % BL_RF_CHANNEL_OFFSET_SPAN);
+    for (int i = 0; i < BL_RF_CHANNEL_COUNT; i++) {
+        channels[i] = (uint8_t)(BL_RF_CHANNEL_FIRST + offset + i * BL_RF_CHANNEL_SPACING);
+    }
+}
+
+bool blEncode(uint32_t linkId, const blControl_t *control, uint8_t out[BL_PAYLOAD_LEN])
+{
+    if (control->session == 0 || (control->flags & (uint8_t)~BL_FLAGS_DEFINED_MASK)) {
+        return false;
+    }
+    for (int ch = 0; ch < BL_CHANNEL_COUNT; ch++) {
+        if (control->channelUs[ch] < BL_CHANNEL_US_MIN || control->channelUs[ch] > BL_CHANNEL_US_MAX) {
+            return false;
+        }
+    }
+
+    memset(out, 0, BL_PAYLOAD_LEN);
+    const uint16_t tag = blLinkTag(linkId);
+    out[0] = BL_PKT_TYPE_CONTROL;
+    out[1] = BL_PROTO_VERSION;
+    out[BL_OFFSET_TAG] = (uint8_t)(tag & 0xFF);
+    out[BL_OFFSET_TAG + 1] = (uint8_t)(tag >> 8);
+    out[BL_OFFSET_SESSION] = control->session;
+    out[BL_OFFSET_SEQ] = (uint8_t)(control->sampleSeq & 0xFF);
+    out[BL_OFFSET_SEQ + 1] = (uint8_t)(control->sampleSeq >> 8);
+
+    unsigned bitPos = 0;
+    for (int ch = 0; ch < BL_CHANNEL_COUNT; ch++) {
+        const uint16_t value = (uint16_t)(control->channelUs[ch] - BL_CHANNEL_US_MIN);
+        for (int bit = 0; bit < BL_CHANNEL_BITS; bit++, bitPos++) {
+            if (value & (1U << bit)) {
+                out[BL_OFFSET_CHANNELS + (bitPos >> 3)] |= (uint8_t)(1U << (bitPos & 7));
+            }
+        }
+    }
+
+    out[BL_OFFSET_FLAGS] = control->flags;
+    const uint16_t crc = blCrc16(blCrcSeed(linkId), out, BL_OFFSET_CRC);
+    out[BL_OFFSET_CRC] = (uint8_t)(crc & 0xFF);
+    out[BL_OFFSET_CRC + 1] = (uint8_t)(crc >> 8);
+    return true;
+}
+
+blDecodeResult_e blDecode(uint32_t linkId, const uint8_t *buf, uint8_t len, blControl_t *out)
+{
+    if (len != BL_PAYLOAD_LEN) {
+        return BL_DECODE_BAD_LENGTH;
+    }
+    if (buf[0] != BL_PKT_TYPE_CONTROL) {
+        return BL_DECODE_BAD_TYPE;
+    }
+    if (buf[1] != BL_PROTO_VERSION) {
+        return BL_DECODE_BAD_VERSION;
+    }
+    const uint16_t tag = (uint16_t)(buf[BL_OFFSET_TAG] | (buf[BL_OFFSET_TAG + 1] << 8));
+    if (tag != blLinkTag(linkId)) {
+        return BL_DECODE_BAD_TAG;
+    }
+    const uint16_t crc = (uint16_t)(buf[BL_OFFSET_CRC] | (buf[BL_OFFSET_CRC + 1] << 8));
+    if (crc != blCrc16(blCrcSeed(linkId), buf, BL_OFFSET_CRC)) {
+        return BL_DECODE_BAD_CRC;
+    }
+    const uint8_t flags = buf[BL_OFFSET_FLAGS];
+    if (flags & (uint8_t)~BL_FLAGS_DEFINED_MASK) {
+        return BL_DECODE_BAD_FLAGS;
+    }
+    const uint8_t session = buf[BL_OFFSET_SESSION];
+    if (session == 0) {
+        return BL_DECODE_BAD_SESSION;
+    }
+
+    blControl_t decoded;
+    unsigned bitPos = 0;
+    for (int ch = 0; ch < BL_CHANNEL_COUNT; ch++) {
+        uint16_t value = 0;
+        for (int bit = 0; bit < BL_CHANNEL_BITS; bit++, bitPos++) {
+            if (buf[BL_OFFSET_CHANNELS + (bitPos >> 3)] & (1U << (bitPos & 7))) {
+                value |= (uint16_t)(1U << bit);
+            }
+        }
+        if (value > BL_CHANNEL_VALUE_MAX) {
+            return BL_DECODE_BAD_RANGE;
+        }
+        decoded.channelUs[ch] = (uint16_t)(BL_CHANNEL_US_MIN + value);
+    }
+    decoded.session = session;
+    decoded.sampleSeq = (uint16_t)(buf[BL_OFFSET_SEQ] | (buf[BL_OFFSET_SEQ + 1] << 8));
+    decoded.flags = flags;
+
+    if (out) {
+        *out = decoded;
+    }
+    return BL_DECODE_OK;
+}
+
+void blFreshnessInit(blFreshness_t *f)
+{
+    memset(f, 0, sizeof(*f));
+}
+
+/*
+ * Freshness rules:
+ *  - A packet flagged BL_FLAG_TX_INPUT_FAULT is never accepted.
+ *  - Same session as the reference and sequence advanced by 1..BL_SEQ_ACCEPT_WINDOW: accept.
+ *    (The handset only advances the sequence when a new input sample exists, so a frozen
+ *    acquisition produces duplicates, never acceptances.)
+ *  - Sequence unchanged: duplicate.
+ *  - Anything else (handset restart = new session, a sequence step backwards, or a jump
+ *    larger than the window after a long outage) is not accepted. It becomes a
+ *    candidate, and the stream is re-anchored only when the NEXT packet carries the same
+ *    session and the candidate sequence + 1. So reacquisition costs one packet (5 ms).
+ *  - Sequence numbers are 16-bit and compared modulo 2^16, so wrap-around is seamless.
+ */
+blFreshResult_e blFreshnessCheck(blFreshness_t *f, uint8_t session, uint16_t seq, uint8_t flags)
+{
+    if (flags & BL_FLAG_TX_INPUT_FAULT) {
+        return BL_FRESH_TX_FAULT;
+    }
+
+    const bool sameSession = f->haveRef && session == f->session;
+    if (sameSession) {
+        const uint16_t step = (uint16_t)(seq - f->lastSeq);
+        if (step == 0) {
+            return BL_FRESH_DUPLICATE;
+        }
+        if (step <= BL_SEQ_ACCEPT_WINDOW) {
+            f->lastSeq = seq;
+            f->haveCandidate = false;
+            return BL_FRESH_ACCEPT;
+        }
+    }
+
+    if (f->haveCandidate && f->candSession == session && (uint16_t)(seq - f->candSeq) == 1) {
+        f->haveRef = true;
+        f->session = session;
+        f->lastSeq = seq;
+        f->haveCandidate = false;
+        return BL_FRESH_ACCEPT;
+    }
+
+    const bool backwards = sameSession && (int16_t)(uint16_t)(seq - f->lastSeq) < 0;
+    f->haveCandidate = true;
+    f->candSession = session;
+    f->candSeq = seq;
+    return backwards ? BL_FRESH_STALE : BL_FRESH_RESYNC_PENDING;
+}
+
+void blHopInit(blHop_t *h, uint32_t nowUs)
+{
+    memset(h, 0, sizeof(*h));
+    h->state = BL_HOP_SEARCH;
+    h->idx = 0;
+    h->lastHopUs = nowUs;
+    h->nextHopUs = nowUs + BL_SEARCH_DWELL_US;
+}
+
+/*
+ * Slot timing. The handset transmits packet k at T0 + k*P on channel k mod N.
+ * The receiver retunes half way between expected packets, so every packet arrives
+ * in the middle of its listening window with +/- P/2 tolerance, and the retune never
+ * interrupts the auto-ACK that follows a packet.
+ */
+void blHopOnPacket(blHop_t *h, uint32_t packetTimeUs, bool timeReliable)
+{
+    h->heardThisSlot = true;
+    h->missedSlots = 0;
+
+    if (h->state == BL_HOP_SYNCED && !timeReliable) {
+        return;  // keep the existing slot timing
+    }
+
+    uint32_t next = packetTimeUs + BL_FRAME_PERIOD_US / 2;
+    if ((int32_t)(packetTimeUs - h->lastHopUs) < 0) {
+        // The packet arrived before the last retune (it waited in the FIFO), so it belongs
+        // to the previous channel; the channel tuned now carries the following packet.
+        next += BL_FRAME_PERIOD_US;
+    }
+    h->nextHopUs = next;
+    h->state = BL_HOP_SYNCED;
+}
+
+bool blHopTick(blHop_t *h, uint32_t nowUs)
+{
+    const int32_t late = (int32_t)(nowUs - h->nextHopUs);
+    if (late < 0) {
+        return false;
+    }
+
+    if (h->state == BL_HOP_SEARCH) {
+        // Dwell on each channel for longer than one full hop cycle of the handset.
+        h->idx = (uint8_t)((h->idx + 1) % BL_RF_CHANNEL_COUNT);
+        h->nextHopUs = nowUs + BL_SEARCH_DWELL_US;
+        h->lastHopUs = nowUs;
+        h->heardThisSlot = false;
+        return true;
+    }
+
+    // Number of slot boundaries passed (normally 1; more if this call was delayed).
+    const uint32_t slots = 1U + (uint32_t)late / BL_FRAME_PERIOD_US;
+    const uint32_t missedNow = slots - (h->heardThisSlot ? 1U : 0U);
+    const uint32_t missed = (uint32_t)h->missedSlots + missedNow;
+    h->missedSlots = (uint16_t)(missed > 0xFFFFU ? 0xFFFFU : missed);
+    h->heardThisSlot = false;
+    h->idx = (uint8_t)((h->idx + slots) % BL_RF_CHANNEL_COUNT);
+    h->nextHopUs += slots * BL_FRAME_PERIOD_US;
+    h->lastHopUs = nowUs;
+
+    if (h->missedSlots > BL_SYNC_MAX_MISSED_SLOTS) {
+        h->state = BL_HOP_SEARCH;
+        h->missedSlots = 0;
+        h->nextHopUs = nowUs + BL_SEARCH_DWELL_US;
+    }
+    return true;
+}
diff --git a/src/main/rx/nrf24_bicopter_link.h b/src/main/rx/nrf24_bicopter_link.h
new file mode 100644
index 000000000..e7480fea2
--- /dev/null
+++ b/src/main/rx/nrf24_bicopter_link.h
@@ -0,0 +1,155 @@
+/*
+ * This file is part of Betaflight.
+ *
+ * Betaflight is free software. You can redistribute this software
+ * and/or modify this software under the terms of the GNU General
+ * Public License as published by the Free Software Foundation,
+ * either version 3 of the License, or (at your option) any later
+ * version.
+ *
+ * Betaflight is distributed in the hope that it will be useful,
+ * but WITHOUT ANY WARRANTY; without even the implied warranty of
+ * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
+ *
+ * See the GNU General Public License for more details.
+ *
+ * You should have received a copy of the GNU General Public
+ * License along with this software.
+ *
+ * If not, see <http://www.gnu.org/licenses/>.
+ */
+
+/*
+ * NRF24_BICOPTER link protocol, version 1 - hardware independent core.
+ *
+ * This file (and nrf24_bicopter_link.c) is shared VERBATIM between the
+ * Betaflight receiver and the STM32F103 handset firmware. It has no
+ * dependency on Betaflight or on any MCU so it can also be unit tested on a PC.
+ *
+ * Radio layer (both ends):
+ *   nRF24L01+, 250 kbps, 2-byte radio CRC, 5-byte address, auto-acknowledge
+ *   on pipe 0 with dynamic payload length, no ACK payload, no retransmits.
+ *   The 5 address bytes are written to RX_ADDR_P0 / TX_ADDR in array order
+ *   (addr[0] first, i.e. addr[0] is the least significant address byte).
+ *
+ * Pairing: there is no over-the-air bind. Both ends are built with the same
+ * 32-bit link ID; it determines the radio address, the hop channels, the
+ * packet identity tag and the CRC seed.
+ *
+ * Control packet (BL_PAYLOAD_LEN = 20 bytes, multi-byte fields little-endian):
+ *   [0]      type      = BL_PKT_TYPE_CONTROL (0xB1)
+ *   [1]      version   = BL_PROTO_VERSION (1)
+ *   [2..3]   link tag  = blLinkTag(linkId)
+ *   [4]      session   = random non-zero value chosen at handset boot
+ *   [5..6]   sampleSeq = incremented ONLY when a new joystick/switch sample set
+ *                        has been acquired (not per transmission)
+ *   [7..16]  8 channels x 10 bits, packed LSB-first; value v in 0..1000 means
+ *            (1000 + v) microseconds. Values 1001..1023 are invalid.
+ *   [17]     flags: bit0 = BL_FLAG_TX_INPUT_FAULT, bits 1..7 must be zero
+ *   [18..19] CRC-16/CCITT (poly 0x1021, MSB first) over bytes 0..17,
+ *            initial value blCrcSeed(linkId)
+ *
+ * Timing: the handset sends one packet every BL_FRAME_PERIOD_US, packet k on
+ * hop channel (k mod BL_RF_CHANNEL_COUNT). See blHop* below for the receiver.
+ */
+
+#pragma once
+
+#include <stdbool.h>
+#include <stddef.h>
+#include <stdint.h>
+
+#define BL_PROTO_VERSION            1
+#define BL_PKT_TYPE_CONTROL         0xB1
+#define BL_PAYLOAD_LEN              20
+#define BL_ADDR_LEN                 5
+#define BL_CHANNEL_COUNT            8
+#define BL_CHANNEL_VALUE_MAX        1000
+#define BL_CHANNEL_US_MIN           1000
+#define BL_CHANNEL_US_MAX           (BL_CHANNEL_US_MIN + BL_CHANNEL_VALUE_MAX)
+
+#define BL_RF_CHANNEL_COUNT         4
+#define BL_RF_CHANNEL_FIRST         4      // lowest nRF24 RF_CH used (2404 MHz)
+#define BL_RF_CHANNEL_SPACING       19     // MHz between hop channels
+#define BL_RF_CHANNEL_OFFSET_SPAN   20     // per-link offset 0..19 -> highest RF_CH = 80
+
+#define BL_FRAME_PERIOD_US          5000   // 200 packets per second
+#define BL_SEQ_ACCEPT_WINDOW        200    // largest forward sequence step treated as continuous (1 s)
+#define BL_SYNC_MAX_MISSED_SLOTS    8      // consecutive empty slots before searching again (40 ms)
+#define BL_SEARCH_DWELL_US          ((BL_RF_CHANNEL_COUNT + 1) * BL_FRAME_PERIOD_US)
+
+#define BL_FLAG_TX_INPUT_FAULT      0x01
+#define BL_FLAGS_DEFINED_MASK       (BL_FLAG_TX_INPUT_FAULT)
+
+typedef struct blControl_s {
+    uint8_t session;
+    uint16_t sampleSeq;
+    uint8_t flags;
+    uint16_t channelUs[BL_CHANNEL_COUNT];   // 1000..2000
+} blControl_t;
+
+typedef enum {
+    BL_DECODE_OK = 0,
+    BL_DECODE_BAD_LENGTH,
+    BL_DECODE_BAD_TYPE,
+    BL_DECODE_BAD_VERSION,
+    BL_DECODE_BAD_TAG,
+    BL_DECODE_BAD_CRC,
+    BL_DECODE_BAD_FLAGS,
+    BL_DECODE_BAD_RANGE,
+    BL_DECODE_BAD_SESSION,
+} blDecodeResult_e;
+
+typedef enum {
+    BL_FRESH_ACCEPT = 0,        // new control sample: may refresh the RX watchdog
+    BL_FRESH_DUPLICATE,         // same sample as the last accepted one
+    BL_FRESH_STALE,             // older than the last accepted sample (same session)
+    BL_FRESH_RESYNC_PENDING,    // new session / big jump: waiting for a consecutive confirmation
+    BL_FRESH_TX_FAULT,          // handset reported its own input acquisition fault
+} blFreshResult_e;
+
+typedef struct blFreshness_s {
+    bool haveRef;
+    uint8_t session;
+    uint16_t lastSeq;
+    bool haveCandidate;
+    uint8_t candSession;
+    uint16_t candSeq;
+} blFreshness_t;
+
+typedef enum {
+    BL_HOP_SEARCH = 0,
+    BL_HOP_SYNCED = 1,
+} blHopState_e;
+
+typedef struct blHop_s {
+    blHopState_e state;
+    uint8_t idx;                // index into the hop channel table currently tuned
+    uint32_t nextHopUs;         // time of the next retune
+    uint32_t lastHopUs;         // time of the last retune
+    uint16_t missedSlots;       // consecutive slots without an authentic packet
+    bool heardThisSlot;
+} blHop_t;
+
+uint32_t blMix32(uint32_t x);
+uint16_t blLinkTag(uint32_t linkId);
+uint16_t blCrcSeed(uint32_t linkId);
+uint16_t blCrc16(uint16_t crc, const uint8_t *data, size_t len);
+void blDeriveAddress(uint32_t linkId, uint8_t addr[BL_ADDR_LEN]);
+void blDeriveRfChannels(uint32_t linkId, uint8_t channels[BL_RF_CHANNEL_COUNT]);
+
+// Returns false (and leaves out unspecified) if any channel is outside 1000..2000us,
+// session is zero, or undefined flag bits are set.
+bool blEncode(uint32_t linkId, const blControl_t *control, uint8_t out[BL_PAYLOAD_LEN]);
+blDecodeResult_e blDecode(uint32_t linkId, const uint8_t *buf, uint8_t len, blControl_t *out);
+
+void blFreshnessInit(blFreshness_t *f);
+blFreshResult_e blFreshnessCheck(blFreshness_t *f, uint8_t session, uint16_t seq, uint8_t flags);
+
+void blHopInit(blHop_t *h, uint32_t nowUs);
+// An authentic (decoded OK) packet was read. If timeReliable is false the
+// timestamp is only approximate (e.g. a packet left over in the RX FIFO) and the
+// slot timing is not re-anchored while synchronised.
+void blHopOnPacket(blHop_t *h, uint32_t packetTimeUs, bool timeReliable);
+// Returns true if h->idx changed and the radio must be retuned. Bounded: never loops.
+bool blHopTick(blHop_t *h, uint32_t nowUs);
diff --git a/src/main/rx/rx_spi.c b/src/main/rx/rx_spi.c
index e7c86e6e2..733c99ca6 100644
--- a/src/main/rx/rx_spi.c
+++ b/src/main/rx/rx_spi.c
@@ -49,6 +49,7 @@
 #include "rx/nrf24_h8_3d.h"
 #include "rx/nrf24_inav.h"
 #include "rx/nrf24_kn.h"
+#include "rx/nrf24_bicopter.h"
 #include "rx/a7105_flysky.h"
 #include "rx/cc2500_sfhss.h"
 #include "rx/cyrf6936_spektrum.h"
@@ -65,12 +66,15 @@ typedef rx_spi_received_e (*protocolDataReceivedFnPtr)(uint8_t *payload);
 typedef rx_spi_received_e (*protocolProcessFrameFnPtr)(uint8_t *payload);
 typedef void (*protocolSetRcDataFromPayloadFnPtr)(uint16_t *rcData, const uint8_t *payload);
 typedef void (*protocolStopFnPtr)(void);
+typedef timeUs_t (*protocolFrameTimeUsFnPtr)(void);
 
 static protocolInitFnPtr protocolInit;
 static protocolDataReceivedFnPtr protocolDataReceived;
 static protocolProcessFrameFnPtr protocolProcessFrame;
 static protocolSetRcDataFromPayloadFnPtr protocolSetRcDataFromPayload;
 static protocolStopFnPtr protocolStop = nullProtocolStop;
+// Optional: protocols that timestamp frames themselves (e.g. when polling without EXTI)
+static protocolFrameTimeUsFnPtr protocolFrameTimeUs;
 
 static rxSpiExtiConfig_t extiConfig = {
     .ioConfig = IOCFG_IN_FLOATING,
@@ -139,6 +143,14 @@ STATIC_UNIT_TESTED bool rxSpiSetProtocol(rx_spi_protocol_e protocol)
         protocolSetRcDataFromPayload = inavNrf24SetRcDataFromPayload;
         break;
 #endif
+#ifdef USE_RX_NRF24_BICOPTER
+    case RX_SPI_NRF24_BICOPTER:
+        protocolInit = nrf24BicopterInit;
+        protocolDataReceived = nrf24BicopterDataReceived;
+        protocolSetRcDataFromPayload = nrf24BicopterSetRcDataFromPayload;
+        protocolFrameTimeUs = nrf24BicopterFrameTimeUs;
+        break;
+#endif
 #if defined(USE_RX_FRSKY_SPI)
 #if defined(USE_RX_FRSKY_SPI_D)
     case RX_SPI_FRSKY_D:
@@ -211,9 +223,14 @@ static uint8_t rxSpiFrameStatus(rxRuntimeState_t *rxRuntimeState)
 
     if (result & RX_SPI_RECEIVED_DATA) {
         rxSpiNewPacketAvailable = true;
-        // use SPI EXTI time as frame time
-        // note that there is not rx time without EXTI
-        rxRuntimeState->lastRcFrameTimeUs = rxSpiGetLastExtiTimeUs();
+        if (protocolFrameTimeUs) {
+            // protocol provides the time of the accepted frame
+            rxRuntimeState->lastRcFrameTimeUs = protocolFrameTimeUs();
+        } else {
+            // use SPI EXTI time as frame time
+            // note that there is not rx time without EXTI
+            rxRuntimeState->lastRcFrameTimeUs = rxSpiGetLastExtiTimeUs();
+        }
         status = RX_FRAME_COMPLETE;
     }
 
diff --git a/src/main/rx/rx_spi.h b/src/main/rx/rx_spi.h
index 5f1cd307d..e0a88c485 100644
--- a/src/main/rx/rx_spi.h
+++ b/src/main/rx/rx_spi.h
@@ -49,6 +49,7 @@ typedef enum {
     RX_SPI_FRSKY_X_V2,
     RX_SPI_FRSKY_X_LBT_V2,
     RX_SPI_EXPRESSLRS,
+    RX_SPI_NRF24_BICOPTER,
     RX_SPI_PROTOCOL_COUNT
 } rx_spi_protocol_e;
 
diff --git a/src/platform/PICO/target/common/target_RP2350.h b/src/platform/PICO/target/common/target_RP2350.h
index 5a6adf56f..00750786f 100644
--- a/src/platform/PICO/target/common/target_RP2350.h
+++ b/src/platform/PICO/target/common/target_RP2350.h
@@ -111,7 +111,12 @@
 
 // Various untested or unsupported elements are undefined below
 
+// RX_SPI is untested on PICO. A board config can opt in to the NRF24 SPI receiver
+// path only, by defining PICO_ENABLE_RX_SPI_NRF24 together with USE_RX_SPI and
+// USE_RX_NRF24. Other SPI receivers (CC2500, ExpressLRS, SX127x/SX1280) stay disabled.
+#if !defined(PICO_ENABLE_RX_SPI_NRF24)
 #undef USE_RX_SPI
+#endif
 #undef USE_RX_PWM
 #undef USE_RX_PPM
 #undef USE_RX_CC2500
diff --git a/src/platform/PICO/usb/usb_msc_pico.c b/src/platform/PICO/usb/usb_msc_pico.c
index 60578b440..292fa9e23 100644
--- a/src/platform/PICO/usb/usb_msc_pico.c
+++ b/src/platform/PICO/usb/usb_msc_pico.c
@@ -326,6 +326,65 @@ void mscTask(void)
 {
     tud_task();
 }
+
+#else // !USE_USB_MSC
+
+// The TinyUSB MSC class driver is always compiled for PICO (CFG_TUD_MSC=1 in tusb_config.h)
+// and usb_descriptors.c always references pico_msc_active. When a board has no blackbox
+// storage, common_post.h undefines USE_USB_MSC and none of the symbols above exist, so the
+// link fails. Provide inert definitions: pico_msc_active stays false, so the MSC
+// configuration descriptor is never offered and these callbacks are never invoked.
+
+#include <string.h>
+
+#include "tusb_config.h"
+
+#define _STDIO_H_
+#include "tusb.h"
+
+#include "common/utils.h"
+
+bool pico_msc_active = false;
+
+int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
+{
+    UNUSED(lun); UNUSED(lba); UNUSED(offset); UNUSED(buffer); UNUSED(bufsize);
+    return -1;
+}
+
+int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
+{
+    UNUSED(lun); UNUSED(lba); UNUSED(offset); UNUSED(buffer); UNUSED(bufsize);
+    return -1;
+}
+
+void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
+{
+    UNUSED(lun);
+    memset(vendor_id, ' ', 8);
+    memset(product_id, ' ', 16);
+    memset(product_rev, ' ', 4);
+}
+
+bool tud_msc_test_unit_ready_cb(uint8_t lun)
+{
+    UNUSED(lun);
+    return false;
+}
+
+void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
+{
+    UNUSED(lun);
+    *block_count = 0;
+    *block_size = 512;
+}
+
+int32_t tud_msc_scsi_cb(uint8_t lun, const uint8_t scsi_cmd[16], void *buffer, uint16_t bufsize)
+{
+    UNUSED(lun); UNUSED(scsi_cmd); UNUSED(buffer); UNUSED(bufsize);
+    return -1;
+}
+
 #endif // USE_USB_MSC
 
 
```

# Appendix B: config submodule patch (apply in src/config at 3a406c5)
```diff
diff --git a/configs/BICOPTER_PICO2/config.h b/configs/BICOPTER_PICO2/config.h
new file mode 100644
index 0000000..8d5f7ec
--- /dev/null
+++ b/configs/BICOPTER_PICO2/config.h
@@ -0,0 +1,120 @@
+/*
+ * This file is part of Betaflight.
+ *
+ * Betaflight is free software. You can redistribute this software
+ * and/or modify this software under the terms of the GNU General
+ * Public License as published by the Free Software Foundation,
+ * either version 3 of the License, or (at your option) any later
+ * version.
+ *
+ * Betaflight is distributed in the hope that it will be useful,
+ * but WITHOUT ANY WARRANTY; without even the implied warranty of
+ * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
+ *
+ * See the GNU General Public License for more details.
+ *
+ * You should have received a copy of the GNU General Public
+ * License along with this software.
+ *
+ * If not, see <http://www.gnu.org/licenses/>.
+ */
+
+/*
+ * DIY bicopter: Raspberry Pi Pico 2 (RP2350A, 4MB flash), MPU6500 on SPI0,
+ * nRF24L01+ PA/LNA on SPI1 using the NRF24_BICOPTER RX_SPI protocol,
+ * two PWM ESCs (SimonK) and two tilt servos, built-in BI mixer.
+ *
+ * PROPOSED wiring - must be checked against the real harness before flight.
+ * Pin names: PAn == GPIOn (GPn on the Pico silk screen).
+ *
+ *   Function                  GPIO   Pico pin
+ *   MPU6500 SCK  (SPI0)       GP2    4
+ *   MPU6500 MOSI (SPI0 TX)    GP3    5
+ *   MPU6500 MISO (SPI0 RX)    GP4    6
+ *   MPU6500 CS                GP5    7
+ *   MPU6500 INT               GP22   29
+ *   Motor 1 (left ESC)        GP10   14    PWM slice 5 A
+ *   Motor 2 (right ESC)       GP11   15    PWM slice 5 B
+ *   Servo 1 (left tilt)       GP20   26    PWM slice 2 A
+ *   Servo 2 (right tilt)      GP21   27    PWM slice 2 B
+ *   nRF24 SCK  (SPI1)         GP14   19
+ *   nRF24 MOSI (SPI1 TX)      GP15   20
+ *   nRF24 MISO (SPI1 RX)      GP12   16
+ *   nRF24 CSN                 GP13   17
+ *   nRF24 CE                  GP16   21
+ *   nRF24 IRQ (active low)    GP17   22
+ *   VBAT divider (ADC0)       GP26   31    optional
+ *   On-board LED              GP25   -
+ *
+ * Motors and servos are on different PWM slices so motor_pwm_rate and the
+ * (fixed 50Hz) servo frequency are independent.
+ */
+
+#pragma once
+
+#define FC_TARGET_MCU     RP2350A
+#define BOARD_NAME        BICOPTER_PICO2
+#define MANUFACTURER_ID   DIYB
+
+#define LED0_PIN             PA25
+
+// ---- Gyro / accelerometer: MPU6500 on hardware SPI0 ----
+#define USE_GYRO
+#define USE_ACC
+#define USE_GYRO_SPI_MPU6500
+#define USE_ACC_SPI_MPU6500
+
+#define SPI0_SCK_PIN         PA2
+#define SPI0_SDO_PIN         PA3
+#define SPI0_SDI_PIN         PA4
+
+#define GYRO_1_SPI_INSTANCE  SPI0
+#define GYRO_1_CS_PIN        PA5
+#define GYRO_1_EXTI_PIN      PA22
+// Placeholder: the real breakout orientation must be measured on the airframe
+// (CLI: align_board_roll/pitch/yaw).
+#define GYRO_1_ALIGN         CW0_DEG
+
+// ---- Receiver: nRF24L01+ on hardware SPI1, NRF24_BICOPTER protocol ----
+// Opt in to RX_SPI on PICO (target_RP2350.h otherwise removes it), NRF24 only.
+#define PICO_ENABLE_RX_SPI_NRF24
+#define USE_RX_SPI
+#define USE_RX_NRF24
+#define USE_RX_NRF24_BICOPTER
+
+#define SPI1_SCK_PIN         PA14
+#define SPI1_SDO_PIN         PA15
+#define SPI1_SDI_PIN         PA12
+
+#define RX_SPI_INSTANCE      SPI1
+#define RX_SPI_CS_PIN        PA13     // read by pg/rx_spi.c (CLI resource RX_SPI_CS)
+#define RX_NSS_PIN           PA13     // legacy name used by drivers/rx/rx_nrf24l01.c, same pin
+#define RX_CE_PIN            PA16     // compile-time only (not a CLI resource)
+#define RX_SPI_EXTI_PIN      PA17     // nRF24 IRQ, active low (CLI resource RX_SPI_EXTI)
+
+#define DEFAULT_RX_FEATURE       FEATURE_RX_SPI
+#define RX_SPI_DEFAULT_PROTOCOL  RX_SPI_NRF24_BICOPTER
+
+// Bring-up SPI clock for the radio. The Pico divider rounds down, so the actual
+// clock is <= this value. Raise (max 10MHz per nRF24L01+ spec) only after the link
+// has been validated, to reduce the time each packet read takes in the gyro loop.
+#define NRF24L01_MAX_SPI_CLK_HZ  1000000
+
+// Must match HANDSET_LINK_ID in the handset firmware (handset/config/link_config.h).
+#define BICOPTER_NRF24_LINK_ID   0xBE5336DFU
+
+// ---- Outputs ----
+#define MOTOR1_PIN           PA10
+#define MOTOR2_PIN           PA11
+#define SERVO1_PIN           PA20
+#define SERVO2_PIN           PA21
+
+#define DEFAULT_MIXER            MIXER_BICOPTER
+// SimonK ESCs: standard (continuous) PWM, 1000-2000us. 200Hz is a commissioning
+// candidate only; confirm the ESCs arm and respond correctly at this rate.
+#define DEFAULT_MOTOR_PROTOCOL   MOTOR_PROTOCOL_PWM
+#define DEFAULT_MOTOR_PWM_RATE   200
+
+// ---- Battery voltage (optional divider on ADC0) ----
+#define USE_ADC
+#define ADC_VBAT_PIN         PA26
diff --git a/configs/BICOPTER_PICO2/config.mk b/configs/BICOPTER_PICO2/config.mk
new file mode 100644
index 0000000..9b4f25e
--- /dev/null
+++ b/configs/BICOPTER_PICO2/config.mk
@@ -0,0 +1,12 @@
+# Raspberry Pi Pico 2: RP2350A with 4MB QSPI flash (W25Q32 class).
+# Override the RP2350 defaults (8MB) so the pico-sdk, Betaflight's flash checks
+# and the linker script all agree on the physical flash size.
+
+# Override MCU_FLASH_SIZE (KB)
+TARGET_FLASH_SIZE = 4096
+
+# 4194304 = 4 * 1024 * 1024
+PICO_FLASH_DEFINES = \
+                   -DPICO_FLASH_SPI_CLKDIV=2 \
+                   -DPICO_FLASH_SIZE_BYTES=4194304 \
+                   -DPICO_BOOT_STAGE2_CHOOSE_W25Q080=1
diff --git a/configs/BICOPTER_PICO2/pico_flash_mem.ld b/configs/BICOPTER_PICO2/pico_flash_mem.ld
new file mode 100644
index 0000000..a8d6044
--- /dev/null
+++ b/configs/BICOPTER_PICO2/pico_flash_mem.ld
@@ -0,0 +1,5 @@
+# Raspberry Pi Pico 2: 4MB primary flash.
+PRIMARY_FLASH_LENGTH = 4M;
+
+# No framebuffer OSD on this board, so no font area.
+FONT_LENGTH = 0;
```

# Appendix: handset/config/link_config.h
```c
// Handset link configuration. HANDSET_LINK_ID MUST equal BICOPTER_NRF24_LINK_ID in
// betaflight src/config/configs/BICOPTER_PICO2/config.h.
#pragma once

#define HANDSET_LINK_ID         0xBE5336DFU

// nRF24 RF_SETUP power bits for the handset transmitter (0x06 = 0 dBm before the PA).
// Use 0x00 (-18 dBm) for close-range bench tests: a PA/LNA module at full power can
// saturate the receiver's LNA when the two are less than ~1-2 m apart.
#define HANDSET_RF_PWR_BITS     0x00

// Channel order sent on air (AETR + 4 aux). Betaflight default rcmap is AETR1234.
enum { HS_CH_ROLL = 0, HS_CH_PITCH, HS_CH_THROTTLE, HS_CH_YAW, HS_CH_AUX1_ARM, HS_CH_AUX2, HS_CH_AUX3, HS_CH_AUX4 };

// A new input sample must arrive at least this often or packets carry TX_INPUT_FAULT.
#define HANDSET_SAMPLE_TIMEOUT_US   50000
// Raw ADC readings this close to the rails are treated as a disconnected stick (12-bit ADC).
#define HANDSET_ADC_RAIL_MARGIN     8
#define HANDSET_ADC_MAX             4095
```

# Appendix: handset/src/handset_hal.h
```c
// Hardware abstraction the STM32F103 board port must implement.
// Nothing here assumes a pin map, clock tree, HAL library or toolchain.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define HS_ANALOG_COUNT 4      // roll, pitch, throttle, yaw sticks
#define HS_SWITCH_COUNT 4      // aux1 (arm) .. aux4, value 0..2 (2- or 3-position)

typedef struct {
    uint16_t analog[HS_ANALOG_COUNT];   // raw 12-bit ADC counts
    uint8_t sw[HS_SWITCH_COUNT];        // 0 = low, 1 = mid, 2 = high
} hsRawInputs_t;

// Monotonic microsecond counter (32-bit wrap allowed).
uint32_t hal_micros(void);
// Returns true exactly once per COMPLETED new acquisition (e.g. from the ADC DMA
// transfer-complete interrupt plus a switch read) and copies it out. Returns false
// if no new acquisition finished since the previous call. Must not return true for a
// repeated copy of old data: this is what makes frozen acquisition detectable.
bool hal_take_new_inputs(hsRawInputs_t *out);
// Non-zero random byte that differs on each boot (e.g. hash of unique ID + ADC noise + timer).
uint8_t hal_boot_random_nonzero(void);

// nRF24 SPI: mode 0, MSB first, <= 8 MHz. CSN is handled by hal_nrf_csn().
uint8_t hal_spi_xfer(uint8_t byte);
void hal_nrf_csn(bool high);
void hal_nrf_ce(bool high);
void hal_delay_us(uint32_t us);
void hal_led(bool on);
```

# Appendix: handset/src/handset_app.h
```c
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "handset_hal.h"

typedef struct {
    uint16_t min, center, max;   // raw ADC calibration per stick
    bool reverse;
} hsStickCal_t;

typedef struct {
    uint32_t sent, acked, noAck, inputFaultFrames;
} hsStats_t;

// Returns false if the radio did not respond (check wiring/power).
bool hs_init(const hsStickCal_t cal[HS_ANALOG_COUNT]);
// Call as often as possible from the main loop; transmits one packet every BL_FRAME_PERIOD_US.
void hs_poll(void);
const hsStats_t *hs_stats(void);
// Exposed for tests: convert raw inputs to channel microseconds.
bool hs_inputs_to_channels(const hsRawInputs_t *in, const hsStickCal_t cal[HS_ANALOG_COUNT], uint16_t outUs[8]);
```

# Appendix: handset/src/handset_app.c
```c
// Portable handset application: input sampling -> NRF24_BICOPTER v1 packets -> nRF24L01+ TX.
#include <string.h>

#include "../config/link_config.h"
#include "../shared/nrf24_bicopter_link.h"
#include "handset_app.h"

// nRF24L01+ commands / registers
#define W_REGISTER 0x20
#define R_REGISTER 0x00
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define ACTIVATE 0x50
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_RX_ADDR_P0 0x0A
#define REG_TX_ADDR 0x10
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D
#define ST_TX_DS 0x20
#define ST_MAX_RT 0x10

// CONFIG: EN_CRC | CRCO (2-byte CRC) | PWR_UP, PRIM_RX = 0 (transmitter)
#define TX_CONFIG 0x0E

static uint8_t rfCh[BL_RF_CHANNEL_COUNT];
static uint8_t addr[BL_ADDR_LEN];
static hsStickCal_t cal[HS_ANALOG_COUNT];
static blControl_t ctl;
static uint32_t nextTxUs, lastSampleUs, hop;
static bool haveSample, awaitingResult;
static hsStats_t stats;

static void wreg(uint8_t r, uint8_t v) { hal_nrf_csn(false); hal_spi_xfer(W_REGISTER | r); hal_spi_xfer(v); hal_nrf_csn(true); }
static uint8_t rreg(uint8_t r) { hal_nrf_csn(false); hal_spi_xfer(R_REGISTER | r); uint8_t v = hal_spi_xfer(0xFF); hal_nrf_csn(true); return v; }
static void wmulti(uint8_t cmd, const uint8_t *d, uint8_t n) { hal_nrf_csn(false); hal_spi_xfer(cmd); for (uint8_t i = 0; i < n; i++) hal_spi_xfer(d[i]); hal_nrf_csn(true); }
static void cmd(uint8_t c) { hal_nrf_csn(false); hal_spi_xfer(c); hal_nrf_csn(true); }

static uint16_t mapStick(uint16_t raw, const hsStickCal_t *c)
{
    int32_t us;
    if (raw >= c->center) {
        const int32_t span = (int32_t)c->max - c->center;
        us = 1500 + (span > 0 ? ((int32_t)(raw - c->center) * 500) / span : 0);
    } else {
        const int32_t span = (int32_t)c->center - c->min;
        us = 1500 - (span > 0 ? ((int32_t)(c->center - raw) * 500) / span : 0);
    }
    if (us < BL_CHANNEL_US_MIN) us = BL_CHANNEL_US_MIN;
    if (us > BL_CHANNEL_US_MAX) us = BL_CHANNEL_US_MAX;
    if (c->reverse) us = 3000 - us;
    return (uint16_t)us;
}

bool hs_inputs_to_channels(const hsRawInputs_t *in, const hsStickCal_t c[HS_ANALOG_COUNT], uint16_t outUs[8])
{
    bool ok = true;
    for (int i = 0; i < HS_ANALOG_COUNT; i++) {
        // A stick reading pinned at a rail usually means a broken wire/potentiometer
        if (in->analog[i] <= HANDSET_ADC_RAIL_MARGIN || in->analog[i] >= HANDSET_ADC_MAX - HANDSET_ADC_RAIL_MARGIN) ok = false;
        outUs[i] = mapStick(in->analog[i], &c[i]);
    }
    for (int i = 0; i < HS_SWITCH_COUNT; i++) {
        outUs[HS_ANALOG_COUNT + i] = in->sw[i] == 0 ? 1000 : (in->sw[i] == 1 ? 1500 : 2000);
        if (in->sw[i] > 2) ok = false;
    }
    return ok;
}

bool hs_init(const hsStickCal_t c[HS_ANALOG_COUNT])
{
    memcpy(cal, c, sizeof(cal));
    blDeriveAddress(HANDSET_LINK_ID, addr);
    blDeriveRfChannels(HANDSET_LINK_ID, rfCh);
    memset(&ctl, 0, sizeof(ctl));
    memset(&stats, 0, sizeof(stats));
    ctl.session = hal_boot_random_nonzero();
    ctl.sampleSeq = 0;
    for (int i = 0; i < BL_CHANNEL_COUNT; i++) ctl.channelUs[i] = 1500;
    ctl.channelUs[2] = 1000; ctl.channelUs[4] = 1000;   // throttle low, arm off until sampled
    ctl.flags = BL_FLAG_TX_INPUT_FAULT;                 // nothing valid until the first sample
    haveSample = false;

    hal_nrf_ce(false);
    hal_delay_us(100000);                 // power-on reset
    wreg(REG_CONFIG, TX_CONFIG);
    hal_delay_us(5000);                   // power down -> standby
    wreg(REG_SETUP_AW, 0x03);
    if (rreg(REG_SETUP_AW) != 0x03) return false;
    wreg(REG_FEATURE, 0x04);              // EN_DPL
    if (rreg(REG_FEATURE) != 0x04) {
        const uint8_t key = 0x73;         // ACTIVATE is a command, not a register write
        wmulti(ACTIVATE, &key, 1);
        wreg(REG_FEATURE, 0x04);
        if (rreg(REG_FEATURE) != 0x04) return false;
    }
    wreg(REG_EN_AA, 0x01);
    wreg(REG_EN_RXADDR, 0x01);
    wreg(REG_SETUP_RETR, 0x20);           // ARD 750 us, ARC 0 (no retransmit: next frame is newer data)
    wreg(REG_RF_SETUP, 0x20 | HANDSET_RF_PWR_BITS);   // 250 kbps
    wreg(REG_DYNPD, 0x01);
    wmulti(W_REGISTER | REG_TX_ADDR, addr, BL_ADDR_LEN);
    wmulti(W_REGISTER | REG_RX_ADDR_P0, addr, BL_ADDR_LEN);   // to receive the auto-ACK
    cmd(FLUSH_TX); cmd(FLUSH_RX);
    wreg(REG_STATUS, 0x70);
    nextTxUs = hal_micros();
    lastSampleUs = hal_micros();
    hop = 0;
    awaitingResult = false;
    return rreg(REG_CONFIG) == TX_CONFIG;
}

void hs_poll(void)
{
    hsRawInputs_t in;
    const uint32_t now = hal_micros();
    if (hal_take_new_inputs(&in)) {
        uint16_t us[8];
        const bool ok = hs_inputs_to_channels(&in, cal, us);
        memcpy(ctl.channelUs, us, sizeof(us));
        ctl.sampleSeq++;                  // ONLY here: one increment per acquisition
        ctl.flags = ok ? 0 : BL_FLAG_TX_INPUT_FAULT;
        lastSampleUs = now;
        haveSample = true;
    }
    if (!haveSample || (int32_t)(now - lastSampleUs) > HANDSET_SAMPLE_TIMEOUT_US) {
        ctl.flags |= BL_FLAG_TX_INPUT_FAULT;
    }

    if ((int32_t)(now - nextTxUs) < 0) return;
    nextTxUs += BL_FRAME_PERIOD_US;
    if ((int32_t)(now - nextTxUs) > BL_FRAME_PERIOD_US) nextTxUs = now;   // resync after a stall

    if (awaitingResult) {
        const uint8_t st = rreg(REG_STATUS);
        if (st & ST_TX_DS) stats.acked++; else stats.noAck++;
        hal_led((st & ST_TX_DS) != 0);
    }

    uint8_t pkt[BL_PAYLOAD_LEN];
    if (!blEncode(HANDSET_LINK_ID, &ctl, pkt)) return;
    if (ctl.flags & BL_FLAG_TX_INPUT_FAULT) stats.inputFaultFrames++;

    hal_nrf_ce(false);
    wreg(REG_RF_CH, rfCh[hop % BL_RF_CHANNEL_COUNT]);
    wreg(REG_STATUS, 0x70);
    cmd(FLUSH_TX);
    wmulti(W_TX_PAYLOAD, pkt, BL_PAYLOAD_LEN);
    hal_nrf_ce(true);
    hal_delay_us(15);                     // >10 us CE pulse starts one transmission
    hal_nrf_ce(false);
    hop++;
    stats.sent++;
    awaitingResult = true;
}

const hsStats_t *hs_stats(void) { return &stats; }
```

# Appendix: handset/test/test_handset.c
```c
// Host test: handset_app.c with a fake HAL whose SPI is decoded by an nRF24 command model.
// Every transmitted payload is decoded with the receiver's protocol core.
#include <stdio.h>
#include <string.h>
#include "../config/link_config.h"
#include "../shared/nrf24_bicopter_link.h"
#include "../src/handset_app.h"

static int fails;
#define CHECK(c) do { if (!(c)) { fails++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static uint32_t now;
static bool csnLow, ce;
static uint8_t frame[40]; static int flen;
static uint8_t regs[32], txAddr[5];
static int txCount; static uint8_t lastPkt[32]; static uint8_t lastCh[2000];
static bool newSample = true, frozen;
static uint16_t adcVal = 2048;

uint32_t hal_micros(void) { return now; }
void hal_delay_us(uint32_t us) { now += us; }
void hal_led(bool on) { (void)on; }
uint8_t hal_boot_random_nonzero(void) { return 0x77; }
bool hal_take_new_inputs(hsRawInputs_t *o) {
    if (!newSample || frozen) return false;
    newSample = false;
    for (int i = 0; i < 4; i++) o->analog[i] = adcVal;
    for (int i = 0; i < 4; i++) o->sw[i] = (uint8_t)(i % 3);
    return true;
}
static void endFrame(void) {
    if (flen == 0) return;
    uint8_t c = frame[0];
    if ((c & 0xE0) == 0x20 && flen >= 2) { uint8_t r = c & 31; if (r == 0x10) memcpy(txAddr, &frame[1], 5); else if (r == 7) regs[7] &= (uint8_t)~(frame[1] & 0x70); else regs[r] = frame[1]; }
    if (c == 0xA0) { memcpy(lastPkt, &frame[1], flen - 1); }
    flen = 0;
}
void hal_nrf_csn(bool high) { if (high && csnLow) endFrame(); csnLow = !high; }
uint8_t hal_spi_xfer(uint8_t b) {
    uint8_t ret = 0;
    if (flen == 1 && (frame[0] & 0xE0) == 0x00) ret = regs[frame[0] & 31];
    if (flen < 40) frame[flen++] = b;
    return ret;
}
void hal_nrf_ce(bool high) {
    if (ce && !high) { // falling edge of a CE pulse after W_TX_PAYLOAD -> one transmission
        if (txCount < 2000) lastCh[txCount] = regs[5];
        txCount++; regs[7] |= 0x20;
        blControl_t d;
        CHECK(blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &d) == BL_DECODE_OK);
    }
    ce = high;
}

int main(void)
{
    hsStickCal_t cal[4];
    for (int i = 0; i < 4; i++) { cal[i].min = 100; cal[i].center = 2048; cal[i].max = 4000; cal[i].reverse = false; }
    CHECK(hs_init(cal));
    uint8_t a[5]; blDeriveAddress(HANDSET_LINK_ID, a);
    CHECK(memcmp(a, txAddr, 5) == 0);
    CHECK(regs[6] == (0x20 | HANDSET_RF_PWR_BITS));
    CHECK(regs[0] == 0x0E && regs[0x1C] == 1 && regs[0x1D] == 4 && regs[4] == 0x20);

    // Feed the RX freshness logic with what the handset sends
    blFreshness_t f; blFreshnessInit(&f);
    int accepted = 0, faultFrames = 0;
    uint8_t ch[4]; blDeriveRfChannels(HANDSET_LINK_ID, ch);
    int lastTx = 0;
    for (uint32_t t = 0; t < 2000000; t += 100) {
        now += 100;
        if (t % 5000 == 0) newSample = true;           // ADC completes at 200 Hz
        if (t == 1000000) frozen = true;               // acquisition stops after 1 s
        hs_poll();
        if (txCount != lastTx) {
            lastTx = txCount;
            blControl_t d; blDecode(HANDSET_LINK_ID, lastPkt, BL_PAYLOAD_LEN, &d);
            if (d.flags) faultFrames++;
            if (blFreshnessCheck(&f, d.session, d.sampleSeq, d.flags) == BL_FRESH_ACCEPT) accepted++;
            if (t > 1000000 + HANDSET_SAMPLE_TIMEOUT_US + 6000) CHECK(d.flags & BL_FLAG_TX_INPUT_FAULT);
            CHECK(d.channelUs[0] == 1500 && d.channelUs[4] == 1000 && d.channelUs[5] == 1500 && d.channelUs[6] == 2000);
        }
    }
    for (int i = 0; i < 400 && i < txCount; i++) CHECK(lastCh[i] == ch[i % 4]);
    printf("  sent %d packets in 2 s, %d accepted by RX freshness, %d flagged input-fault\n", txCount, accepted, faultFrames);
    CHECK(txCount >= 398 && txCount <= 402);
    CHECK(accepted >= 195 && accepted <= 201);   // only the first second carried fresh samples
    // Rail detection
    hsRawInputs_t in = { .analog = {2048, 2048, 2, 2048}, .sw = {0, 0, 0, 0} };
    uint16_t us[8];
    CHECK(!hs_inputs_to_channels(&in, cal, us));
    in.analog[2] = 2048;
    CHECK(hs_inputs_to_channels(&in, cal, us));
    printf("%s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
    return fails != 0;
}
```

# Appendix: handset/shared/nrf24_bicopter_link.{c,h}
Identical to src/main/rx/nrf24_bicopter_link.{c,h} in Appendix A.

# Appendix: tests/test_common.h
```c
#pragma once
#include <stdio.h>
#include <stdlib.h>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failures++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { g_checks++; long long _a = (long long)(a), _b = (long long)(b); if (_a != _b) { g_failures++; \
    printf("  FAIL %s:%d: %s == %s (%lld != %lld)\n", __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)
#define RUN(test) do { int _f = g_failures; test(); printf("%s %s\n", g_failures == _f ? "PASS" : "FAIL", #test); } while (0)
#define SUMMARY() do { printf("\n%d checks, %d failures\n", g_checks, g_failures); return g_failures ? 1 : 0; } while (0)

// Deterministic PRNG for tests
static unsigned long long g_rng = 0x1234567887654321ULL;
static unsigned rng32(void) { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 7; g_rng ^= g_rng << 17; return (unsigned)(g_rng >> 16); }
```

# Appendix: tests/test_link.c
```c
// Unit tests for the hardware independent NRF24_BICOPTER protocol core.
#include <string.h>

#include "rx/nrf24_bicopter_link.h"
#include "test_common.h"

#define LINK 0xBE5336DFU

static blControl_t makeControl(uint8_t session, uint16_t seq)
{
    blControl_t c;
    memset(&c, 0, sizeof(c));
    c.session = session;
    c.sampleSeq = seq;
    for (int i = 0; i < BL_CHANNEL_COUNT; i++) {
        c.channelUs[i] = (uint16_t)(1000 + (i * 137) % 1001);
    }
    return c;
}

static void testRoundTrip(void)
{
    uint8_t buf[BL_PAYLOAD_LEN];
    for (int iter = 0; iter < 20000; iter++) {
        blControl_t c = makeControl((uint8_t)(1 + rng32() % 255), (uint16_t)rng32());
        for (int i = 0; i < BL_CHANNEL_COUNT; i++) {
            c.channelUs[i] = (uint16_t)(1000 + rng32() % 1001);
        }
        c.flags = (uint8_t)(rng32() & BL_FLAGS_DEFINED_MASK);
        CHECK(blEncode(LINK, &c, buf));
        blControl_t d;
        CHECK_EQ(blDecode(LINK, buf, BL_PAYLOAD_LEN, &d), BL_DECODE_OK);
        CHECK(memcmp(&c.channelUs, &d.channelUs, sizeof(c.channelUs)) == 0);
        CHECK_EQ(d.session, c.session);
        CHECK_EQ(d.sampleSeq, c.sampleSeq);
        CHECK_EQ(d.flags, c.flags);
        if (g_failures) return;
    }
    // extremes
    blControl_t c = makeControl(1, 0);
    for (int i = 0; i < BL_CHANNEL_COUNT; i++) c.channelUs[i] = (i & 1) ? 2000 : 1000;
    CHECK(blEncode(LINK, &c, buf));
    blControl_t d;
    CHECK_EQ(blDecode(LINK, buf, BL_PAYLOAD_LEN, &d), BL_DECODE_OK);
    CHECK(memcmp(&c.channelUs, &d.channelUs, sizeof(c.channelUs)) == 0);
}

static void testKnownVector(void)
{
    // Fixed vector so the handset implementation can be checked byte-for-byte.
    blControl_t c = makeControl(0x5A, 0x1234);
    uint8_t buf[BL_PAYLOAD_LEN];
    CHECK(blEncode(LINK, &c, buf));
    printf("  vector(link=0x%08X): ", LINK);
    for (int i = 0; i < BL_PAYLOAD_LEN; i++) printf("%02X", buf[i]);
    printf("\n");
    CHECK_EQ(buf[0], 0xB1);
    CHECK_EQ(buf[1], 1);
    CHECK_EQ(buf[4], 0x5A);
    CHECK_EQ(buf[5], 0x34);
    CHECK_EQ(buf[6], 0x12);
    // CRC-16/CCITT-FALSE check value for "123456789" with seed 0xFFFF is 0x29B1
    CHECK_EQ(blCrc16(0xFFFF, (const uint8_t *)"123456789", 9), 0x29B1);
}

static void testEncodeRejects(void)
{
    uint8_t buf[BL_PAYLOAD_LEN];
    blControl_t c = makeControl(1, 1);
    c.channelUs[3] = 999;
    CHECK(!blEncode(LINK, &c, buf));
    c.channelUs[3] = 2001;
    CHECK(!blEncode(LINK, &c, buf));
    c = makeControl(0, 1);
    CHECK(!blEncode(LINK, &c, buf));
    c = makeControl(1, 1);
    c.flags = 0x02;
    CHECK(!blEncode(LINK, &c, buf));
}

static void testDecodeRejects(void)
{
    uint8_t buf[BL_PAYLOAD_LEN], bad[32];
    blControl_t c = makeControl(7, 99);
    CHECK(blEncode(LINK, &c, buf));

    memcpy(bad, buf, BL_PAYLOAD_LEN); bad[20] = 0;
    CHECK_EQ(blDecode(LINK, bad, 19, NULL), BL_DECODE_BAD_LENGTH);
    CHECK_EQ(blDecode(LINK, bad, 21, NULL), BL_DECODE_BAD_LENGTH);
    CHECK_EQ(blDecode(LINK, bad, 32, NULL), BL_DECODE_BAD_LENGTH);
    CHECK_EQ(blDecode(LINK, bad, 0, NULL), BL_DECODE_BAD_LENGTH);

    memcpy(bad, buf, BL_PAYLOAD_LEN); bad[0] = 0xAE;   // e.g. an INAV bind packet
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_TYPE);
    memcpy(bad, buf, BL_PAYLOAD_LEN); bad[1] = 2;
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_VERSION);

    // A packet from a handset built with a different link ID is rejected
    CHECK(blDecode(LINK ^ 1, buf, BL_PAYLOAD_LEN, NULL) != BL_DECODE_OK);
    CHECK(blDecode(0x12345678U, buf, BL_PAYLOAD_LEN, NULL) != BL_DECODE_OK);

    // Every single bit error is detected
    int undetected = 0;
    for (int bit = 0; bit < BL_PAYLOAD_LEN * 8; bit++) {
        memcpy(bad, buf, BL_PAYLOAD_LEN);
        bad[bit / 8] ^= (uint8_t)(1 << (bit % 8));
        if (blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL) == BL_DECODE_OK) undetected++;
    }
    CHECK_EQ(undetected, 0);
    // Random corruption of several bytes
    undetected = 0;
    for (int iter = 0; iter < 200000; iter++) {
        memcpy(bad, buf, BL_PAYLOAD_LEN);
        int n = 2 + rng32() % 4;
        for (int k = 0; k < n; k++) bad[rng32() % BL_PAYLOAD_LEN] ^= (uint8_t)(1 + rng32() % 255);
        if (memcmp(bad, buf, BL_PAYLOAD_LEN) != 0 && blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL) == BL_DECODE_OK) undetected++;
    }
    printf("  random multi-byte corruptions accepted: %d / 200000\n", undetected);
    CHECK(undetected <= 10);

    // Out-of-range channel with a valid CRC (crafted) is rejected
    memcpy(bad, buf, BL_PAYLOAD_LEN);
    bad[7] |= 0xFF; bad[8] |= 0x03;   // channel 0 = 1023
    uint16_t crc = blCrc16(blCrcSeed(LINK), bad, 18);
    bad[18] = (uint8_t)crc; bad[19] = (uint8_t)(crc >> 8);
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_RANGE);

    // Reserved flag bit with valid CRC
    memcpy(bad, buf, BL_PAYLOAD_LEN);
    bad[17] = 0x80;
    crc = blCrc16(blCrcSeed(LINK), bad, 18);
    bad[18] = (uint8_t)crc; bad[19] = (uint8_t)(crc >> 8);
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_FLAGS);

    // Session 0 with valid CRC
    memcpy(bad, buf, BL_PAYLOAD_LEN);
    bad[4] = 0;
    crc = blCrc16(blCrcSeed(LINK), bad, 18);
    bad[18] = (uint8_t)crc; bad[19] = (uint8_t)(crc >> 8);
    CHECK_EQ(blDecode(LINK, bad, BL_PAYLOAD_LEN, NULL), BL_DECODE_BAD_SESSION);
}

static void testDerivation(void)
{
    for (int iter = 0; iter < 100000; iter++) {
        uint32_t id = iter < 3 ? (uint32_t[]){0, 0xFFFFFFFFU, LINK}[iter] : (uint32_t)rng32() ^ ((uint32_t)rng32() << 16);
        uint8_t a[BL_ADDR_LEN], a2[BL_ADDR_LEN], ch[BL_RF_CHANNEL_COUNT];
        blDeriveAddress(id, a);
        blDeriveAddress(id, a2);
        CHECK(memcmp(a, a2, BL_ADDR_LEN) == 0);
        for (int i = 0; i < BL_ADDR_LEN; i++) {
            CHECK(a[i] != 0x00 && a[i] != 0xFF && a[i] != 0x55 && a[i] != 0xAA);
        }
        blDeriveRfChannels(id, ch);
        for (int i = 0; i < BL_RF_CHANNEL_COUNT; i++) {
            CHECK(ch[i] >= 4 && ch[i] <= 80);
            if (i) CHECK_EQ(ch[i] - ch[i - 1], BL_RF_CHANNEL_SPACING);
        }
        CHECK(blCrcSeed(id) != 0);
        if (g_failures) return;
    }
    uint8_t a[5], ch[4];
    blDeriveAddress(LINK, a);
    blDeriveRfChannels(LINK, ch);
    printf("  link 0x%08X: address %02X %02X %02X %02X %02X, RF_CH %d %d %d %d, tag 0x%04X, crcSeed 0x%04X\n",
           LINK, a[0], a[1], a[2], a[3], a[4], ch[0], ch[1], ch[2], ch[3], blLinkTag(LINK), blCrcSeed(LINK));
}

static void testFreshness(void)
{
    blFreshness_t f;
    blFreshnessInit(&f);
    // Power-up: first packet needs confirmation by the next consecutive one
    CHECK_EQ(blFreshnessCheck(&f, 10, 500, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 10, 500, 0), BL_FRESH_RESYNC_PENDING);   // duplicate of candidate
    CHECK_EQ(blFreshnessCheck(&f, 10, 501, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 10, 502, 0), BL_FRESH_ACCEPT);
    // Duplicate (frozen acquisition / repeated sample)
    for (int i = 0; i < 1000; i++) CHECK_EQ(blFreshnessCheck(&f, 10, 502, 0), BL_FRESH_DUPLICATE);
    // Lost packets within the window are fine
    CHECK_EQ(blFreshnessCheck(&f, 10, 510, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 10, 510 + BL_SEQ_ACCEPT_WINDOW, 0), BL_FRESH_ACCEPT);
    uint16_t s = 510 + BL_SEQ_ACCEPT_WINDOW;
    // Stale
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s - 1), 0), BL_FRESH_STALE);
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s - 100), 0), BL_FRESH_STALE);
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s + 1), 0), BL_FRESH_ACCEPT);  // stream continues
    s++;
    // TX fault flag never accepted, does not move the reference
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s + 1), BL_FLAG_TX_INPUT_FAULT), BL_FRESH_TX_FAULT);
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s + 2), BL_FLAG_TX_INPUT_FAULT), BL_FRESH_TX_FAULT);
    CHECK_EQ(blFreshnessCheck(&f, 10, (uint16_t)(s + 3), 0), BL_FRESH_ACCEPT);
    s += 3;
    // Wrap-around 65535 -> 0
    blFreshnessInit(&f);
    CHECK_EQ(blFreshnessCheck(&f, 3, 65534, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 3, 65535, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 3, 0, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 3, 1, 0), BL_FRESH_ACCEPT);
    CHECK_EQ(blFreshnessCheck(&f, 3, 65535, 0), BL_FRESH_STALE);
    // Long outage: jump beyond the window needs one confirming packet
    CHECK_EQ(blFreshnessCheck(&f, 3, 1 + BL_SEQ_ACCEPT_WINDOW + 1, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 3, 1 + BL_SEQ_ACCEPT_WINDOW + 2, 0), BL_FRESH_ACCEPT);
    // Handset restart: new session, sequence restarts
    CHECK_EQ(blFreshnessCheck(&f, 4, 0, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 4, 1, 0), BL_FRESH_ACCEPT);
    // Old session packet after the switch is not accepted
    CHECK_EQ(blFreshnessCheck(&f, 3, 300, 0), BL_FRESH_RESYNC_PENDING);
    CHECK_EQ(blFreshnessCheck(&f, 4, 2, 0), BL_FRESH_ACCEPT);
    // Restart with the SAME session byte (1/255 chance): recovers after two consecutive packets
    CHECK_EQ(blFreshnessCheck(&f, 4, 0, 0), BL_FRESH_STALE);
    CHECK_EQ(blFreshnessCheck(&f, 4, 1, 0), BL_FRESH_ACCEPT);
    // Non-consecutive candidates never anchor
    blFreshnessInit(&f);
    for (int i = 0; i < 1000; i++) CHECK(blFreshnessCheck(&f, 9, (uint16_t)(i * 2), 0) != BL_FRESH_ACCEPT);
}

// ---- Hop scheduler simulation (pure core, ideal radio) ----
typedef struct {
    double ppm;          // handset clock error
    int jitterUs;        // +/- jitter on each transmission
    int lossPct;         // random packet loss
    uint32_t phaseUs;    // handset start offset
    uint32_t stepUs;     // receiver processing period
    uint32_t outageFromUs, outageToUs;
} hopSim_t;

static void runHopSim(const hopSim_t *s, uint32_t durationUs, uint32_t *firstSyncUs, double *rxRatioAfterSync, uint32_t *reacqUs)
{
    uint8_t ch[BL_RF_CHANNEL_COUNT];
    blDeriveRfChannels(LINK, ch);
    blHop_t h;
    uint32_t now = 1000000;   // arbitrary start time
    blHopInit(&h, now);
    uint32_t start = now;
    uint32_t k = 0;
    double nextTx = start + s->phaseUs;
    uint32_t sent = 0, heard = 0;
    *firstSyncUs = 0;
    *reacqUs = 0;
    uint32_t tunedAt = now;
    uint8_t tunedIdx = h.idx;
    uint32_t lastHeard = 0;
    for (; now - start < durationUs; now += s->stepUs) {
        while (nextTx <= now) {
            uint32_t t = (uint32_t)nextTx + (s->jitterUs ? (uint32_t)((int)(rng32() % (2 * s->jitterUs + 1)) - s->jitterUs) : 0);
            uint8_t txIdx = k % BL_RF_CHANNEL_COUNT;
            bool inOutage = (t - start) >= s->outageFromUs && (t - start) < s->outageToUs;
            bool lost = (int)(rng32() % 100) < s->lossPct;
            if (!inOutage) {
                if (*firstSyncUs) sent++;
                // received if tuned to this channel for >=130us before the packet ends (1ms airtime)
                if (!lost && txIdx == tunedIdx && (int32_t)(t - tunedAt) >= 130) {
                    blHopOnPacket(&h, t + 1000, true);
                    if (!*firstSyncUs) *firstSyncUs = t - start; else heard++;
                    if (s->outageToUs && (t - start) >= s->outageToUs && !*reacqUs) *reacqUs = (t - start) - s->outageToUs;
                    lastHeard = t;
                }
            }
            k++;
            nextTx += BL_FRAME_PERIOD_US * (1.0 + s->ppm * 1e-6);
        }
        if (blHopTick(&h, now)) {
            tunedIdx = h.idx;
            tunedAt = now;
        }
    }
    (void)lastHeard;
    *rxRatioAfterSync = sent ? (double)heard / sent : 0;
}

static void testHopping(void)
{
    uint32_t first, reacq;
    double ratio;
    uint32_t worstFirst = 0;
    for (int trial = 0; trial < 200; trial++) {
        hopSim_t s = { .ppm = 0, .jitterUs = 50, .lossPct = 0, .phaseUs = rng32() % 20000, .stepUs = 125 };
        runHopSim(&s, 3000000, &first, &ratio, &reacq);
        CHECK(first != 0);
        const int32_t acq = (int32_t)(first - s.phaseUs);   // jitter can make this slightly negative
        if (acq > (int32_t)worstFirst) worstFirst = (uint32_t)acq;
        CHECK(ratio > 0.999);
        if (g_failures) { printf("  trial %d phase %u first %u ratio %f\n", trial, s.phaseUs, first, ratio); return; }
    }
    printf("  worst initial acquisition: %.1f ms\n", worstFirst / 1000.0);
    CHECK(worstFirst < 4 * BL_SEARCH_DWELL_US);

    // Clock error and slow processing loop
    const double ppms[] = { -500, -200, 200, 500 };
    for (int i = 0; i < 4; i++) {
        hopSim_t s = { .ppm = ppms[i], .jitterUs = 100, .lossPct = 0, .phaseUs = 7000, .stepUs = 1000 };
        runHopSim(&s, 10000000, &first, &ratio, &reacq);
        printf("  ppm %+5.0f, 1 kHz loop: received %.4f\n", ppms[i], ratio);
        CHECK(ratio > 0.999);
    }
    // 30% random loss: stays synchronised (ratio ~0.70)
    hopSim_t s = { .ppm = 100, .jitterUs = 50, .lossPct = 30, .phaseUs = 3000, .stepUs = 250 };
    runHopSim(&s, 10000000, &first, &ratio, &reacq);
    printf("  30%% loss: received %.4f\n", ratio);
    CHECK(ratio > 0.66);
    // Outage then recovery
    hopSim_t o = { .ppm = 150, .jitterUs = 50, .lossPct = 0, .phaseUs = 1234, .stepUs = 250,
                   .outageFromUs = 2000000, .outageToUs = 4000000 };
    runHopSim(&o, 8000000, &first, &ratio, &reacq);
    printf("  2 s outage: reacquired %.1f ms after the handset is audible again\n", reacq / 1000.0);
    CHECK(reacq != 0 && reacq < 4 * BL_SEARCH_DWELL_US);
}

static void testHopLateFifoPacket(void)
{
    // A packet read after the retune (left in FIFO) with a reliable timestamp must not skip a channel.
    blHop_t h;
    blHopInit(&h, 0);
    blHopOnPacket(&h, 10000, true);          // anchored: next hop at 12500
    CHECK_EQ(h.nextHopUs, 12500);
    CHECK(blHopTick(&h, 12500));
    uint8_t idx = h.idx;
    // The packet at 15000 arrives on idx, but is only processed at 17600 (after the 17500 retune)
    CHECK(blHopTick(&h, 17600));
    CHECK_EQ(h.idx, (idx + 1) % BL_RF_CHANNEL_COUNT);
    blHopOnPacket(&h, 15000, true);
    CHECK_EQ(h.nextHopUs, 15000 + 2500 + 5000);   // current channel's slot is kept
    CHECK(!blHopTick(&h, 22000));
    CHECK(blHopTick(&h, 22500));
    // Unreliable timestamp while synced does not move timing
    uint32_t before = h.nextHopUs;
    blHopOnPacket(&h, 24000 + 1700, false);
    CHECK_EQ(h.nextHopUs, before);
    // Missed slots lead to search after BL_SYNC_MAX_MISSED_SLOTS
    blHopInit(&h, 0);
    blHopOnPacket(&h, 1000, true);
    uint32_t t = 3500;
    int ticks = 0;
    while (h.state == BL_HOP_SYNCED && ticks < 100) { blHopTick(&h, t); t += BL_FRAME_PERIOD_US; ticks++; }
    CHECK_EQ(h.state, BL_HOP_SEARCH);
    CHECK_EQ(ticks, BL_SYNC_MAX_MISSED_SLOTS + 2);
    // A delayed call advances several slots at once (no loop)
    blHopInit(&h, 0);
    blHopOnPacket(&h, 1000, true);
    uint8_t i0 = h.idx;
    CHECK(blHopTick(&h, 3500 + 3 * BL_FRAME_PERIOD_US + 10));
    CHECK_EQ(h.idx, (i0 + 4) % BL_RF_CHANNEL_COUNT);
    CHECK_EQ(h.missedSlots, 3);
    // Timer wrap-around of the 32-bit microsecond counter
    blHopInit(&h, 0xFFFFF000U);
    blHopOnPacket(&h, 0xFFFFFF00U, true);
    CHECK(!blHopTick(&h, 0xFFFFFF00U + 2000));
    CHECK(blHopTick(&h, 0xFFFFFF00U + 2500));
}

int main(void)
{
    RUN(testRoundTrip);
    RUN(testKnownVector);
    RUN(testEncodeRejects);
    RUN(testDecodeRejects);
    RUN(testDerivation);
    RUN(testFreshness);
    RUN(testHopLateFifoPacket);
    RUN(testHopping);
    SUMMARY();
}
```

# Appendix: tests/test_rx_adapter.c
```c
// Runs the REAL src/main/rx/nrf24_bicopter.c against a register-level nRF24L01+ model
// (RX FIFO of 3, RX_DR flag, active-low IRQ line with edge capture) and a simulated handset.
#include <string.h>

#include "platform.h"
#include "build/debug.h"
#include "drivers/rx/rx_nrf24l01.h"
#include "drivers/rx/rx_spi.h"
#include "rx/nrf24_bicopter.h"
#include "rx/nrf24_bicopter_link.h"
#include "test_common.h"

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = DEBUG_RX_NRF24_BICOPTER;

// ---------------- radio model ----------------
static struct {
    bool present, irqWired;
    uint8_t regs[32], rxAddr[5];
    bool rxDr, ce;
    uint32_t tunedAt;
    struct { uint8_t len; uint8_t d[32]; } fifo[3];
    int count;
    bool extiFlag; uint32_t extiTs;
    int ops;          // SPI transactions
} R;
static uint32_t nowUs;

timeUs_t micros(void) { return nowUs; }
static void op(void) { R.ops++; }

void NRF24L01_Initialize(uint8_t base) { op(); R.regs[0] = base | 0x02; R.ce = false; }
void NRF24L01_WriteReg(uint8_t reg, uint8_t v) {
    op(); if (!R.present) return;
    if (reg == NRF24L01_07_STATUS) { if (v & 0x40) R.rxDr = false; return; }
    R.regs[reg & 31] = v;
}
void NRF24L01_WriteRegisterMulti(uint8_t reg, const uint8_t *d, uint8_t n) { op(); if (R.present && reg == NRF24L01_0A_RX_ADDR_P0) memcpy(R.rxAddr, d, n); }
uint8_t NRF24L01_ReadReg(uint8_t reg) {
    op(); if (!R.present) return 0xFF;
    if (reg == NRF24L01_07_STATUS) return (uint8_t)((R.rxDr ? 0x40 : 0) | ((R.count ? 0 : 7) << 1));
    if (reg == NRF24L01_17_FIFO_STATUS) return (uint8_t)((R.count == 0) | ((R.count == 3) << 1) | 0x10);
    return R.regs[reg & 31];
}
void NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t *d, uint8_t n) { op(); if (reg == NRF24L01_0A_RX_ADDR_P0) memcpy(d, R.present ? R.rxAddr : (uint8_t[5]){0xFF,0xFF,0xFF,0xFF,0xFF}, n); }
uint8_t NRF24L01_ReadPayloadWidth(void) { op(); return R.count ? R.fifo[0].len : 0; }
void NRF24L01_ReadPayload(uint8_t *d, uint8_t n) { op(); memcpy(d, R.fifo[0].d, n); memmove(&R.fifo[0], &R.fifo[1], sizeof(R.fifo[0]) * 2); R.count--; }
void NRF24L01_FlushRx(void) { op(); R.count = 0; }
void NRF24L01_FlushTx(void) { op(); }
void NRF24L01_Activate(uint8_t c) { op(); (void)c; }
void NRF24L01_SetRxMode(void) { op(); R.regs[0] |= 1; R.rxDr = false; R.ce = true; R.tunedAt = nowUs; }
void NRF24L01_SetChannelRxMode(uint8_t ch) { op(); R.regs[NRF24L01_05_RF_CH] = ch; R.ce = true; R.tunedAt = nowUs; }

bool rxSpiExtiConfigured(void) { return R.irqWired; }
bool rxSpiGetExtiState(void) { return !R.rxDr; }      // line high unless RX_DR asserted
bool rxSpiPollExti(void) { return R.extiFlag; }
void rxSpiResetExti(void) { R.extiFlag = false; }
timeUs_t rxSpiGetLastExtiTimeUs(void) { return R.extiTs; }

static void radioDeliver(const uint8_t *d, uint8_t len, uint8_t rfch, uint32_t t)
{
    if (!R.ce || R.regs[NRF24L01_05_RF_CH] != rfch || (int32_t)(t - R.tunedAt) < 130) return;
    if (R.count == 3) return;                 // FIFO full: radio drops the packet
    R.fifo[R.count].len = len; memcpy(R.fifo[R.count].d, d, len > 32 ? 32 : len); R.count++;
    if (!R.rxDr) { R.rxDr = true; if (R.irqWired) { R.extiFlag = true; R.extiTs = t; } }
}

// ---------------- harness ----------------
static uint8_t payload[RX_SPI_MAX_PAYLOAD_SIZE];
static rxRuntimeState_t rs;
static rxSpiExtiConfig_t exti;

static bool initRx(bool irq, bool present)
{
    memset(&R, 0, sizeof(R)); R.present = present; R.irqWired = irq;
    rxSpiConfig_t cfg = {0};
    return nrf24BicopterInit(&cfg, &rs, &exti);
}

typedef struct { uint8_t session; uint16_t seq; bool freezeSeq; uint8_t flags; uint32_t linkId; int lossPct; } txSim_t;

// Run for durationUs; RX called every loopUs. Returns number of DATA results; tracks max RX gap.
static int run(txSim_t *tx, uint32_t durationUs, uint32_t loopUs, uint32_t *k, double *nextTx, uint32_t *maxGapUs, uint32_t *lastData, int *maxOps, bool txOn)
{
    uint8_t ch[4]; blDeriveRfChannels(BICOPTER_NRF24_LINK_ID, ch);
    int data = 0;
    uint32_t end = nowUs + durationUs;
    for (; (int32_t)(end - nowUs) > 0; nowUs += loopUs) {
        while (*nextTx <= nowUs) {
            if (txOn && (int)(rng32() % 100) >= tx->lossPct) {
                blControl_t c; memset(&c, 0, sizeof(c));
                c.session = tx->session; c.sampleSeq = tx->seq; c.flags = tx->flags;
                for (int i = 0; i < 8; i++) c.channelUs[i] = (uint16_t)(1000 + (tx->seq + i * 100) % 1001);
                uint8_t buf[20]; blEncode(tx->linkId, &c, buf);
                radioDeliver(buf, 20, ch[*k % 4], (uint32_t)*nextTx + 1000);
            }
            if (!tx->freezeSeq) tx->seq++;
            (*k)++; *nextTx += BL_FRAME_PERIOD_US;
        }
        R.ops = 0;
        if (nrf24BicopterDataReceived(payload) == RX_SPI_RECEIVED_DATA) {
            data++;
            uint32_t gap = nowUs - *lastData;
            if (*lastData && gap > *maxGapUs) *maxGapUs = gap;
            *lastData = nowUs;
            uint16_t rc[18] = {0}; nrf24BicopterSetRcDataFromPayload(rc, payload);
            CHECK(rc[0] >= 1000 && rc[0] <= 2000);
        }
        if (R.ops > *maxOps) *maxOps = R.ops;
    }
    return data;
}

static void testInit(void)
{
    CHECK(initRx(true, true));
    CHECK_EQ(rs.channelCount, 8);
    CHECK_EQ(exti.trigger, BETAFLIGHT_EXTI_TRIGGER_FALLING);
    CHECK_EQ(R.regs[NRF24L01_06_RF_SETUP], 0x22);     // 250 kbps, -12 dBm (ACK only)
    CHECK_EQ(R.regs[NRF24L01_01_EN_AA], 0x01);
    CHECK_EQ(R.regs[NRF24L01_1C_DYNPD], 0x01);
    CHECK_EQ(R.regs[NRF24L01_1D_FEATURE], 0x04);
    CHECK_EQ(R.regs[NRF24L01_03_SETUP_AW], 0x03);
    uint8_t a[5]; blDeriveAddress(BICOPTER_NRF24_LINK_ID, a);
    CHECK(memcmp(a, R.rxAddr, 5) == 0);
    CHECK(!initRx(true, false));                       // radio absent / miswired -> RX disabled
    CHECK(debug[7] != 0);
}

static void scenario(bool irq, uint32_t loopUs, const char *name)
{
    nowUs = 1000;
    CHECK(initRx(irq, true));
    txSim_t tx = { .session = 0x42, .seq = 100, .linkId = BICOPTER_NRF24_LINK_ID };
    uint32_t k = 0, gap = 0, last = 0; double next = nowUs + 3333; int maxOps = 0;
    int d = run(&tx, 3000000, loopUs, &k, &next, &gap, &last, &maxOps, true);
    printf("  [%s] 3 s clean: %d fresh frames of %u sent, max gap %.1f ms, max SPI ops/call %d\n", name, d, k, gap / 1000.0, maxOps);
    CHECK(d > (int)k - 15);
    CHECK(gap < 20000);
    CHECK(maxOps <= 12);

    // Frozen acquisition: transport continues, sequence frozen -> no fresh frames at all
    tx.freezeSeq = true;
    int frozen = run(&tx, 500000, loopUs, &k, &next, &gap, &last, &maxOps, true);
    CHECK(frozen <= 1);   // only the sample taken just before the freeze
    tx.freezeSeq = false;
    // Handset input fault flag -> no fresh frames
    tx.flags = BL_FLAG_TX_INPUT_FAULT;
    CHECK_EQ(run(&tx, 300000, loopUs, &k, &next, &gap, &last, &maxOps, true), 0);
    tx.flags = 0;
    // Foreign handset (different link ID) on the same channels -> nothing accepted
    tx.linkId = 0x11223344U;
    CHECK_EQ(run(&tx, 300000, loopUs, &k, &next, &gap, &last, &maxOps, true), 0);
    tx.linkId = BICOPTER_NRF24_LINK_ID;
    // Recovery after these
    last = 0; gap = 0;
    uint32_t t0 = nowUs;
    d = run(&tx, 1000000, loopUs, &k, &next, &gap, &last, &maxOps, true);
    CHECK(d > 150);
    // Link loss 2 s then handset restart (new session, seq from 0)
    run(&tx, 2000000, loopUs, &k, &next, &gap, &last, &maxOps, false);
    tx.session = 0x43; tx.seq = 0;
    last = 0; gap = 0; t0 = nowUs;
    uint32_t firstAfter = 0;
    for (int i = 0; i < 400 && !firstAfter; i++) {
        if (run(&tx, 1000, loopUs, &k, &next, &gap, &last, &maxOps, true)) firstAfter = nowUs - t0;
    }
    printf("  [%s] handset restart after 2 s loss: first fresh frame after %.1f ms\n", name, firstAfter / 1000.0);
    CHECK(firstAfter != 0 && firstAfter < 120000);
    // 25% loss: stream continues, no gap reaching the 150 ms RXLOSS threshold
    tx.lossPct = 25; gap = 0;
    d = run(&tx, 5000000, loopUs, &k, &next, &gap, &last, &maxOps, true);
    printf("  [%s] 25%% loss: %d fresh frames in 5 s, max gap %.1f ms\n", name, d, gap / 1000.0);
    CHECK(gap < 150000);
    const nrf24BicopterStats_t *st = nrf24BicopterGetStats();
    printf("  [%s] stats: acc %u malformed %u dup %u stale %u pend %u fault %u spurious %u flush %u\n", name,
        st->accepted, st->malformed, st->duplicate, st->stale, st->resyncPending, st->txFault, st->spurious, st->fifoFlush);
}

static void testCorruptWidthAndBacklog(void)
{
    nowUs = 5000;
    CHECK(initRx(true, true));
    // Corrupt width (>32) -> FIFO flushed, nothing accepted
    R.ce = true; R.count = 1; R.fifo[0].len = 40; R.rxDr = true; R.extiFlag = true; R.extiTs = nowUs;
    CHECK_EQ(nrf24BicopterDataReceived(payload), RX_SPI_RECEIVED_NONE);
    CHECK_EQ(R.count, 0);
    CHECK_EQ(nrf24BicopterGetStats()->fifoFlush, 1);
    // Three packets queued behind one edge: read one per call, all consumed, two accepted
    uint8_t ch[4]; blDeriveRfChannels(BICOPTER_NRF24_LINK_ID, ch);
    for (int i = 0; i < 3; i++) {
        blControl_t c; memset(&c, 0, sizeof(c)); c.session = 9; c.sampleSeq = (uint16_t)(10 + i);
        for (int j = 0; j < 8; j++) c.channelUs[j] = 1500;
        uint8_t b[20]; blEncode(BICOPTER_NRF24_LINK_ID, &c, b);
        R.tunedAt = 0; radioDeliver(b, 20, R.regs[NRF24L01_05_RF_CH], nowUs);
    }
    CHECK_EQ(R.count, 3);
    int got = 0;
    for (int i = 0; i < 3; i++) { nowUs += 100; got += nrf24BicopterDataReceived(payload) == RX_SPI_RECEIVED_DATA; CHECK_EQ(R.count, 2 - i); }
    CHECK_EQ(got, 2);    // first packet of a new session is the re-anchor candidate
    // Spurious: IRQ line low with empty FIFO -> one check, then back-off
    R.rxDr = true; int ops = 0;
    for (int i = 0; i < 5; i++) { R.ops = 0; nowUs += 100; nrf24BicopterDataReceived(payload); ops += R.ops; }
    CHECK(ops <= 3);
}

int main(void)
{
    RUN(testInit);
    RUN(testCorruptWidthAndBacklog);
    scenario(true, 250, "IRQ, 4 kHz loop");
    scenario(true, 1000, "IRQ, 1 kHz loop");
    scenario(false, 250, "polled, 4 kHz loop");
    SUMMARY();
}
```

# Appendix: tests/stubs (host stand-ins for Betaflight headers)
## tests/stubs/build/debug.h
```c
#pragma once
#include <stdint.h>
#define DEBUG16_VALUE_COUNT 8
enum { DEBUG_NONE = 0, DEBUG_RX_NRF24_BICOPTER = 1 };
extern int16_t debug[DEBUG16_VALUE_COUNT];
extern uint8_t debugMode;
#define DEBUG_SET(mode, index, value) do { if (debugMode == (mode)) { debug[(index)] = (value); } } while (0)
```
## tests/stubs/common/time.h
```c
#pragma once
#include <stdint.h>
typedef uint32_t timeUs_t;
typedef int32_t timeDelta_t;
static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }
```
## tests/stubs/common/utils.h
```c
#pragma once
#define BIT(x) (1 << (x))
#define UNUSED(x) (void)(x)
#define STATIC_ASSERT(condition, name) _Static_assert((condition), #name)
```
## tests/stubs/drivers/io.h
```c
#pragma once
#include <stdint.h>
typedef uint8_t ioConfig_t;
#define IOCFG_IN_FLOATING 0
#define IOCFG_IPU 1
```
## tests/stubs/drivers/rx/rx_spi.h
```c
#pragma once
#include <stdbool.h>
#include "common/time.h"
#define RX_SPI_MAX_PAYLOAD_SIZE 35
bool rxSpiExtiConfigured(void);
bool rxSpiGetExtiState(void);
bool rxSpiPollExti(void);
void rxSpiResetExti(void);
timeUs_t rxSpiGetLastExtiTimeUs(void);
```
## tests/stubs/drivers/time.h
```c
#pragma once
#include "common/time.h"
timeUs_t micros(void);
```
## tests/stubs/platform.h
```c
// Host-test stand-in for Betaflight's platform.h: only what rx/nrf24_bicopter.c needs.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define USE_RX_NRF24_BICOPTER
#ifndef BICOPTER_NRF24_LINK_ID
#define BICOPTER_NRF24_LINK_ID 0xBE5336DFU
#endif
```
## tests/stubs/rx/rx.h
```c
#pragma once
#include <stdint.h>
#define MAX_SUPPORTED_RC_CHANNEL_COUNT 18
typedef struct rxRuntimeState_s {
    uint8_t channelCount;
} rxRuntimeState_t;
```
## tests/stubs/rx/rx_spi.h
```c
// Host-test stand-in for rx/rx_spi.h (types copied from Betaflight 2026.6.2).
#pragma once
#include <stdint.h>
#include "drivers/io.h"
#include "rx/rx.h"

typedef enum { BETAFLIGHT_EXTI_TRIGGER_RISING = 0, BETAFLIGHT_EXTI_TRIGGER_FALLING = 1, BETAFLIGHT_EXTI_TRIGGER_BOTH = 2 } extiTrigger_t;

typedef struct rxSpiConfig_s {
    uint8_t rx_spi_protocol;
    uint32_t rx_spi_id;
    uint8_t rx_spi_rf_channel_count;
} rxSpiConfig_t;

typedef enum {
    RX_SPI_RECEIVED_NONE = 0,
    RX_SPI_RECEIVED_BIND = (1 << 0),
    RX_SPI_RECEIVED_DATA = (1 << 1),
    RX_SPI_PROCESSING_REQUIRED = (1 << 2),
} rx_spi_received_e;

typedef struct {
    ioConfig_t ioConfig;
    extiTrigger_t trigger;
} rxSpiExtiConfig_t;
```

# Appendix: firmware/build-scripts/apply_edits.py
```
#!/usr/bin/env python3
"""Exact, single-occurrence text replacements in the Betaflight checkout."""
import os
import sys

ROOT = os.path.expanduser("~/bicopter-fw/betaflight")

EDITS = [
    ("src/main/rx/rx_spi.h",
     "    RX_SPI_EXPRESSLRS,\n    RX_SPI_PROTOCOL_COUNT\n",
     "    RX_SPI_EXPRESSLRS,\n    RX_SPI_NRF24_BICOPTER,\n    RX_SPI_PROTOCOL_COUNT\n"),
    ("src/main/cli/settings.c",
     '    "EXPRESSLRS",\n};\n',
     '    "EXPRESSLRS",\n    "NRF24_BICOPTER",\n};\n'),
    ("src/main/build/debug.h",
     "    DEBUG_AUTOPILOT_STOP,\n    DEBUG_COUNT\n",
     "    DEBUG_AUTOPILOT_STOP,\n    DEBUG_RX_NRF24_BICOPTER,\n    DEBUG_COUNT\n"),
    ("src/main/build/debug.c",
     '    [DEBUG_AUTOPILOT_STOP] = "AUTOPILOT_STOP",\n};\n',
     '    [DEBUG_AUTOPILOT_STOP] = "AUTOPILOT_STOP",\n    [DEBUG_RX_NRF24_BICOPTER] = "RX_NRF24_BICOPTER",\n};\n'),
    ("mk/source.mk",
     "            rx/rx_spi.c \\\n            rx/rx_spi_common.c \\\n",
     "            rx/rx_spi.c \\\n            rx/rx_spi_common.c \\\n"
     "            rx/nrf24_bicopter.c \\\n            rx/nrf24_bicopter_link.c \\\n"),
    ("src/platform/PICO/target/common/target_RP2350.h",
     "// Various untested or unsupported elements are undefined below\n\n#undef USE_RX_SPI\n",
     "// Various untested or unsupported elements are undefined below\n\n"
     "// RX_SPI is untested on PICO. A board config can opt in to the NRF24 SPI receiver\n"
     "// path only, by defining PICO_ENABLE_RX_SPI_NRF24 together with USE_RX_SPI and\n"
     "// USE_RX_NRF24. Other SPI receivers (CC2500, ExpressLRS, SX127x/SX1280) stay disabled.\n"
     "#if !defined(PICO_ENABLE_RX_SPI_NRF24)\n#undef USE_RX_SPI\n#endif\n"),
]


def main():
    ok = True
    for rel, old, new in EDITS:
        path = os.path.join(ROOT, rel)
        with open(path, "r", newline="") as fh:
            text = fh.read()
        if new in text:
            print(f"already applied: {rel}")
            continue
        count = text.count(old)
        if count != 1:
            print(f"ERROR {rel}: expected 1 match, found {count}")
            ok = False
            continue
        with open(path, "w", newline="") as fh:
            fh.write(text.replace(old, new, 1))
        print(f"edited: {rel}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
```

# Appendix: firmware/build-scripts/dump_defines.sh
```
#!/bin/bash
# Re-run the captured compile of rx/rx_spi.c with -dM -E to dump the final macro set.
cd ~/bicopter-fw/betaflight
CMD=$(sed -e 's/^[^a-zA-Z/]*//' ~/bicopter-fw/logs/cc_cmd.txt)
CMD=$(echo "$CMD" | sed -E 's/ -o [^ ]+/ /; s/ -c / /; s/-MMD -MP -MF [^ ]+//; s/-flto[^ ]*//g; s/ -Wa,[^ ]+//g')
eval "$CMD -dM -E -o ~/bicopter-fw/logs/BICOPTER_PICO2_defines.txt"
echo "exit=$?"
D=~/bicopter-fw/logs/BICOPTER_PICO2_defines.txt
wc -l $D
for m in USE_RX_SPI USE_RX_NRF24 USE_RX_NRF24_BICOPTER USE_RX_INAV USE_RX_CC2500 USE_RX_EXPRESSLRS USE_RX_SX1280 USE_RX_SX127X \
         PICO_ENABLE_RX_SPI_NRF24 RX_SPI_INSTANCE RX_SPI_CS_PIN RX_NSS_PIN RX_CE_PIN RX_SPI_EXTI_PIN RX_SPI_DEFAULT_PROTOCOL DEFAULT_RX_FEATURE \
         NRF24L01_MAX_SPI_CLK_HZ BICOPTER_NRF24_LINK_ID USE_SERVOS USE_UNCOMMON_MIXERS DEFAULT_MIXER DEFAULT_MOTOR_PROTOCOL DEFAULT_MOTOR_PWM_RATE \
         USE_DSHOT USE_DSHOT_TELEMETRY DEFAULT_DSHOT_TELEMETRY USE_RPM_FILTER USE_DYN_IDLE USE_GYRO_SPI_MPU6500 USE_ACC_SPI_MPU6500 GYRO_1_SPI_INSTANCE \
         GYRO_1_CS_PIN GYRO_1_EXTI_PIN GYRO_1_ALIGN USE_USB_MSC USE_BLACKBOX USE_FLASHFS USE_SDCARD PICO_FLASH_SIZE_BYTES PICO_FLASH_SPI_CLKDIV \
         SYS_CLK_HZ CONFIG_IN_FLASH MOTOR1_PIN MOTOR2_PIN SERVO1_PIN SERVO2_PIN ADC_VBAT_PIN USE_ADC TARGET_BOARD_IDENTIFIER USE_TELEMETRY_NRF24_LTM; do
  line=$(grep -E "^#define $m( |$)" $D)
  echo "${line:-#undef $m   (not defined)}"
done
```

# Appendix: firmware/build-scripts/env.sh
```
#!/bin/bash
# Usage: bash env.sh <command...>   (runs inside ~/bicopter-fw/betaflight)
H=~/bicopter-fw/hosttools
export PATH=$H/root/usr/bin:$H/pt/picotool:$PATH
export LD_LIBRARY_PATH=$H/root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
cd ~/bicopter-fw/betaflight
exec "$@"
```

# Appendix: firmware/build-scripts/export.sh
```
#!/bin/bash
# Export modified Betaflight sources, patches, build outputs and logs to the session folder.
set -e
W=/mnt/c/Users/aksha/Downloads/drone
BF=~/bicopter-fw/betaflight
mkdir -p $W/firmware/src-changed $W/firmware/patches $W/firmware/output $W/firmware/logs
cd $BF
# main repo: tracked modifications + new files
FILES=$( (git diff --name-only; git ls-files --others --exclude-standard -- src/main) | sort -u)
for f in $FILES; do mkdir -p $W/firmware/src-changed/$(dirname $f); cp $f $W/firmware/src-changed/$f; done
git add -N src/main/rx/nrf24_bicopter.c src/main/rx/nrf24_bicopter.h src/main/rx/nrf24_bicopter_link.c src/main/rx/nrf24_bicopter_link.h
git diff > $W/firmware/patches/betaflight-2026.6.2-bicopter-nrf24.patch
git reset -q -- src/main/rx/nrf24_bicopter.c src/main/rx/nrf24_bicopter.h src/main/rx/nrf24_bicopter_link.c src/main/rx/nrf24_bicopter_link.h
# config submodule: new board folder
mkdir -p $W/firmware/src-changed/src/config/configs/BICOPTER_PICO2
cp src/config/configs/BICOPTER_PICO2/* $W/firmware/src-changed/src/config/configs/BICOPTER_PICO2/
(cd src/config && git add -N configs/BICOPTER_PICO2 && git diff -- configs/BICOPTER_PICO2 > $W/firmware/patches/config-BICOPTER_PICO2.patch && git reset -q -- configs/BICOPTER_PICO2)
# outputs
for f in obj/betaflight_2026.6.2_RP2350A_BICOPTER_PICO2.uf2 obj/main/betaflight_RP2350A_BICOPTER_PICO2.elf obj/main/betaflight_RP2350A_BICOPTER_PICO2.map; do
  [ -f "$f" ] && cp "$f" $W/firmware/output/
done
cp ~/bicopter-fw/logs/*.log ~/bicopter-fw/logs/BICOPTER_PICO2_defines.txt $W/firmware/logs/ 2>/dev/null || true
(cd $W/firmware/output && sha256sum * > SHA256SUMS.txt)
echo "exported:"; find $W/firmware -type f | sed "s|$W/||" | sort
```

# Appendix: firmware/build-scripts/final_verify.sh
```
#!/bin/bash
W=/mnt/c/Users/aksha/Downloads/drone
cd ~/bicopter-fw/betaflight
GCC=$PWD/tools/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc
echo "== handset portable code, Cortex-M3 (STM32F103 core) compile"
$GCC -mcpu=cortex-m3 -mthumb -std=c11 -Os -Wall -Wextra -Werror -c $W/handset/src/handset_app.c -o /tmp/hs_app.o && \
$GCC -mcpu=cortex-m3 -mthumb -std=c11 -Os -Wall -Wextra -Werror -c $W/handset/shared/nrf24_bicopter_link.c -o /tmp/hs_link.o && \
$PWD/tools/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-size /tmp/hs_app.o /tmp/hs_link.o
echo "== clean rebuild BICOPTER_PICO2"
make clean CONFIG=BICOPTER_PICO2 >/dev/null 2>&1
( echo "betaflight $(git describe --tags --always) $(git rev-parse HEAD) (+ working-tree patch)";
  echo "config     $(git -C src/config rev-parse HEAD) (+ new BICOPTER_PICO2 folder)";
  echo "pico-sdk   $(git -C lib/modules/pico-sdk rev-parse HEAD) $(git -C lib/modules/pico-sdk describe --tags)";
  echo "tinyusb    $(git -C lib/modules/pico-sdk/lib/tinyusb rev-parse HEAD)";
  $GCC --version | head -1; tools/picotool/picotool version; make --version | head -1;
  time make CONFIG=BICOPTER_PICO2 -j24 ) > ~/bicopter-fw/logs/bicopter_final_build.log 2>&1
echo "rc=$?"
grep -E "warning:|error" ~/bicopter-fw/logs/bicopter_final_build.log | grep -v RWX | sort -u | head
head -7 ~/bicopter-fw/logs/bicopter_final_build.log
tail -12 ~/bicopter-fw/logs/bicopter_final_build.log
sha256sum obj/betaflight_2026.6.2_RP2350A_BICOPTER_PICO2.uf2
tools/picotool/picotool info -a obj/betaflight_2026.6.2_RP2350A_BICOPTER_PICO2.uf2 2>&1 | head -20
echo "== baseline PICO2_2350A still builds with the full patch"
make CONFIG=PICO2_2350A -j24 > ~/bicopter-fw/logs/PICO2_2350A_with_patch.log 2>&1; echo "rc=$?"
```

# Appendix: firmware/build-scripts/hosttools.sh
```
#!/bin/bash
set -e
H=~/bicopter-fw/hosttools
mkdir -p $H/debs $H/root
cd $H/debs
apt-get download make libusb-1.0-0
ls -la
for d in *.deb; do dpkg -x "$d" $H/root; done
ls $H/root/usr/bin $H/root/usr/lib/x86_64-linux-gnu
cd $H
curl -sSL -o picotool.tgz https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.3.1-0/picotool-2.3.1-x86_64-lin.tar.gz
mkdir -p pt && tar -xzf picotool.tgz -C pt
find pt -maxdepth 3 | head
sha256sum picotool.tgz debs/*.deb
```
