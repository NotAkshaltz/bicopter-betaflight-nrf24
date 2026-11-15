/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * DIY bicopter: Raspberry Pi Pico 2 (RP2350A, 4MB flash), MPU6500 on SPI0,
 * nRF24L01+ PA/LNA on SPI1 using the NRF24_BICOPTER RX_SPI protocol,
 * two PWM ESCs (SimonK) and two tilt servos, built-in BI mixer.
 *
 * PROPOSED wiring - must be checked against the real harness before flight.
 * Pin names: PAn == GPIOn (GPn on the Pico silk screen).
 *
 *   Function                  GPIO   Pico pin
 *   MPU6500 SCK  (SPI0)       GP2    4
 *   MPU6500 MOSI (SPI0 TX)    GP3    5
 *   MPU6500 MISO (SPI0 RX)    GP4    6
 *   MPU6500 CS                GP5    7
 *   MPU6500 INT               GP22   29
 *   Motor 1 (left ESC)        GP10   14    PWM slice 5 A
 *   Motor 2 (right ESC)       GP11   15    PWM slice 5 B
 *   Servo 1 (left tilt)       GP20   26    PWM slice 2 A
 *   Servo 2 (right tilt)      GP21   27    PWM slice 2 B
 *   nRF24 SCK  (SPI1)         GP14   19
 *   nRF24 MOSI (SPI1 TX)      GP15   20
 *   nRF24 MISO (SPI1 RX)      GP12   16
 *   nRF24 CSN                 GP13   17
 *   nRF24 CE                  GP16   21
 *   nRF24 IRQ (active low)    GP17   22
 *   VBAT divider (ADC0)       GP26   31    optional
 *   On-board LED              GP25   -
 *
 * Motors and servos are on different PWM slices so motor_pwm_rate and the
 * (fixed 50Hz) servo frequency are independent.
 */

#pragma once

#define FC_TARGET_MCU     RP2350A
#define BOARD_NAME        BICOPTER_PICO2
#define MANUFACTURER_ID   DIYB

#define LED0_PIN             PA25

// ---- Gyro / accelerometer: MPU6500 on hardware SPI0 ----
#define USE_GYRO
#define USE_ACC
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500

#define SPI0_SCK_PIN         PA2
#define SPI0_SDO_PIN         PA3
#define SPI0_SDI_PIN         PA4

#define GYRO_1_SPI_INSTANCE  SPI0
#define GYRO_1_CS_PIN        PA5
#define GYRO_1_EXTI_PIN      PA22
// Placeholder: the real breakout orientation must be measured on the airframe
// (CLI: align_board_roll/pitch/yaw).
#define GYRO_1_ALIGN         CW0_DEG

// ---- Receiver: nRF24L01+ on hardware SPI1, NRF24_BICOPTER protocol ----
// Opt in to RX_SPI on PICO (target_RP2350.h otherwise removes it), NRF24 only.
#define PICO_ENABLE_RX_SPI_NRF24
#define USE_RX_SPI
#define USE_RX_NRF24
#define USE_RX_NRF24_BICOPTER

#define SPI1_SCK_PIN         PA14
#define SPI1_SDO_PIN         PA15
#define SPI1_SDI_PIN         PA12

#define RX_SPI_INSTANCE      SPI1
#define RX_SPI_CS_PIN        PA13     // read by pg/rx_spi.c (CLI resource RX_SPI_CS)
#define RX_NSS_PIN           PA13     // legacy name used by drivers/rx/rx_nrf24l01.c, same pin
#define RX_CE_PIN            PA16     // compile-time only (not a CLI resource)
#define RX_SPI_EXTI_PIN      PA17     // nRF24 IRQ, active low (CLI resource RX_SPI_EXTI)

#define DEFAULT_RX_FEATURE       FEATURE_RX_SPI
#define RX_SPI_DEFAULT_PROTOCOL  RX_SPI_NRF24_BICOPTER

// Bring-up SPI clock for the radio. The Pico divider rounds down, so the actual
// clock is <= this value. Raise (max 10MHz per nRF24L01+ spec) only after the link
// has been validated, to reduce the time each packet read takes in the gyro loop.
#define NRF24L01_MAX_SPI_CLK_HZ  1000000

// Must match HANDSET_LINK_ID in the handset firmware (handset/config/link_config.h).
#define BICOPTER_NRF24_LINK_ID   0xBE5336DFU

// ---- Outputs ----
#define MOTOR1_PIN           PA10
#define MOTOR2_PIN           PA11
#define SERVO1_PIN           PA20
#define SERVO2_PIN           PA21

#define DEFAULT_MIXER            MIXER_BICOPTER
// SimonK ESCs: standard (continuous) PWM, 1000-2000us. 200Hz is a commissioning
// candidate only; confirm the ESCs arm and respond correctly at this rate.
#define DEFAULT_MOTOR_PROTOCOL   MOTOR_PROTOCOL_PWM
#define DEFAULT_MOTOR_PWM_RATE   200

// ---- Battery voltage (optional divider on ADC0) ----
#define USE_ADC
#define ADC_VBAT_PIN         PA26
