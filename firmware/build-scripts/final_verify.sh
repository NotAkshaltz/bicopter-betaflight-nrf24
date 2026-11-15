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
