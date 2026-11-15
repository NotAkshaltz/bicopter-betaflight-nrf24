#!/bin/bash
# Usage: bash env.sh <command...>   (runs inside ~/bicopter-fw/betaflight)
H=~/bicopter-fw/hosttools
export PATH=$H/root/usr/bin:$H/pt/picotool:$PATH
export LD_LIBRARY_PATH=$H/root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
cd ~/bicopter-fw/betaflight
exec "$@"
