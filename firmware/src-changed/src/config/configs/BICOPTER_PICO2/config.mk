# Raspberry Pi Pico 2: RP2350A with 4MB QSPI flash (W25Q32 class).
# Override the RP2350 defaults (8MB) so the pico-sdk, Betaflight's flash checks
# and the linker script all agree on the physical flash size.

# Override MCU_FLASH_SIZE (KB)
TARGET_FLASH_SIZE = 4096

# 4194304 = 4 * 1024 * 1024
PICO_FLASH_DEFINES = \
                   -DPICO_FLASH_SPI_CLKDIV=2 \
                   -DPICO_FLASH_SIZE_BYTES=4194304 \
                   -DPICO_BOOT_STAGE2_CHOOSE_W25Q080=1
