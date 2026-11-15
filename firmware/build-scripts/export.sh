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
