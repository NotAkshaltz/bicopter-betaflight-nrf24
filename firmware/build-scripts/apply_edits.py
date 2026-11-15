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
