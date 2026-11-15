#!/usr/bin/env bash
set -euo pipefail
# Usage: bash test_host.sh /path/to/betaflight /path/to/delivery [output-folder]
bf_root=$(realpath "$1")
delivery_root=$(realpath "$2")
test_output=${3:-"$delivery_root/tests/output"}
mkdir -p "$test_output"
test_cc=${CC:-clang-18}
test_flags=(-std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
"$test_cc" "${test_flags[@]}" -I"$bf_root/src/main" "$delivery_root/tests/test_link.c" "$bf_root/src/main/rx/nrf24_bicopter_link.c" -o "$test_output/test_link"
"$test_output/test_link"
"$test_cc" "${test_flags[@]}" -I"$delivery_root/tests/stubs" -I"$bf_root/src/main" "$delivery_root/tests/test_rx_adapter.c" "$bf_root/src/main/rx/nrf24_bicopter.c" "$bf_root/src/main/rx/nrf24_bicopter_link.c" -o "$test_output/test_rx_adapter"
"$test_output/test_rx_adapter"
"$test_cc" "${test_flags[@]}" "$delivery_root/handset/test/test_handset.c" "$delivery_root/handset/src/handset_app.c" "$delivery_root/handset/shared/nrf24_bicopter_link.c" -o "$test_output/test_handset"
"$test_output/test_handset"
cmp "$bf_root/src/main/rx/nrf24_bicopter_link.c" "$delivery_root/handset/shared/nrf24_bicopter_link.c"
cmp "$bf_root/src/main/rx/nrf24_bicopter_link.h" "$delivery_root/handset/shared/nrf24_bicopter_link.h"
echo 'PASS: shared protocol copies identical; host tests clean under ASan/UBSan.'
