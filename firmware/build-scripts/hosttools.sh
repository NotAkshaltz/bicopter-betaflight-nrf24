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
