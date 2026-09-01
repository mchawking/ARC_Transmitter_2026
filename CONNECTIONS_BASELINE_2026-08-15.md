# ARC V2.1 Connection Baseline

**Baseline date:** 2026-08-15  
**Scope:** Connections explicitly evidenced by the current ARC V2.1 firmware and PlatformIO configuration.  
**Target:** ESP32-S3 DevKitC-1, Arduino / PlatformIO environment `esp32-s3-devkitc-1`.

This is a code-derived PCB reference. It identifies firmware-defined nets and device roles; it does not establish connector pin numbering, component footprints, power regulation, or electrical requirements not present in the code.

## ESP32 To/From Wiring

| From | To | Signal / purpose | Firmware-defined behavior |
|---|---|---|---|
| ESP32-S3 GPIO 12 | ST7789 TFT `SCLK` | SPI clock | Display serial clock. |
| ESP32-S3 GPIO 11 | ST7789 TFT `MOSI` / `SDA` | SPI data | ESP32-to-display data. |
| ESP32-S3 GPIO 9 | ST7789 TFT `CS` | SPI chip select | Display select. |
| ESP32-S3 GPIO 10 | ST7789 TFT `DC` / `A0` | Command/data select | Low for commands; high for display data. |
| ESP32-S3 GPIO 15 | ST7789 TFT `RST` | Display reset | Display reset control. |
| ESP32-S3 GPIO 18 | Back pushbutton, terminal 1 | Back input | `INPUT_PULLUP`; button terminal 2 goes to GND; pressed is LOW. |
| ESP32-S3 GPIO 19 | Select pushbutton, terminal 1 | Select input | `INPUT_PULLUP`; button terminal 2 goes to GND; pressed is LOW. |
| ESP32-S3 GPIO 13 | Up pushbutton, terminal 1 | Up input | `INPUT_PULLUP`; button terminal 2 goes to GND; pressed is LOW. |
| ESP32-S3 GPIO 14 | Down pushbutton, terminal 1 | Down input | `INPUT_PULLUP`; button terminal 2 goes to GND; pressed is LOW. |
| ESP32-S3 GPIO 16 | Arm/unarm toggle | Arm-state input | `INPUT_PULLUP`; GPIO 16 connected to GND means armed; open/high means unarmed. |
| ESP32-S3 GPIO 6 | Shared I2C bus `SDA` | I2C data | Connects to FRAM and TCA9548A. |
| ESP32-S3 GPIO 7 | Shared I2C bus `SCL` | I2C clock | Connects to FRAM and TCA9548A. |
| ESP32-S3 GPIO 21 | CRSF/ELRS module `TX` | ESP32 `Serial1` RX | UART cross-over: radio module sends data to ESP32. |
| ESP32-S3 GPIO 20 | CRSF/ELRS module `RX` | ESP32 `Serial1` TX | UART cross-over: ESP32 sends CRSF frames to radio module. |

## Display

| From | To | Signal / purpose | Notes |
|---|---|---|---|
| ESP32-S3 power rail | ST7789 TFT `VCC` | Display power | Supply voltage is not defined by firmware; verify the display module requirement. |
| ESP32-S3 GND | ST7789 TFT `GND` | Display ground | Common ground. |

The firmware initializes an ST7789 display as 170 x 320 pixels with rotation 1, presenting a 320 x 170 landscape user interface. TFT `MISO` and a backlight-control pin are not used or defined by the firmware.

## Buttons and Toggle

| Control | ESP32 GPIO | Wiring state |
|---|---:|---|
| Back | 18 | Momentary switch between GPIO and GND; internal pull-up. |
| Select | 19 | Momentary switch between GPIO and GND; internal pull-up. |
| Up | 13 | Momentary switch between GPIO and GND; internal pull-up. |
| Down | 14 | Momentary switch between GPIO and GND; internal pull-up. |
| Arm/unarm toggle | 16 | Toggle switch between GPIO and GND; internal pull-up; LOW is armed. |

## I2C Bus and Sensors

| Bus connection | Device | I2C address | Purpose |
|---|---|---:|---|
| GPIO 6 `SDA`, GPIO 7 `SCL` | I2C FRAM | `0x50` | Stores calibration values and encoder state. |
| GPIO 6 `SDA`, GPIO 7 `SCL` | TCA9548A I2C multiplexer | `0x70` | Separates same-address AS5600 encoders. |
| TCA9548A channel 0 `SDA` / `SCL` | Shoulder AS5600 | `0x36` | Shoulder magnetic angle encoder. |
| TCA9548A channel 1 `SDA` / `SCL` | Upper AS5600 | `0x36` | Upper-joint magnetic angle encoder. |
| TCA9548A channel 2 `SDA` / `SCL` | Lower AS5600 | `0x36` | Lower-joint magnetic angle encoder. |
| TCA9548A channel 3 `SDA` / `SCL` | Hand AS5600 | `0x36` | Hand magnetic angle encoder. |

All I2C devices require a compatible power rail and common ground. The code does not specify resistor values, but the bus requires appropriate pull-ups with care to avoid excessive parallel pull-up strength. TCA9548A channels 4 through 7 are unused. AS5600 `OUT`, `DIR`, and `PGO` pins are not used by the firmware.

## CRSF / ELRS Radio Link

| From | To | Signal / purpose | Notes |
|---|---|---|---|
| Radio module `TX` | ESP32-S3 GPIO 21 | CRSF data to ESP32 | `Serial1` RX. |
| ESP32-S3 GPIO 20 | Radio module `RX` | CRSF data from ESP32 | `Serial1` TX. |
| Required radio-module supply | Radio module `VCC` | Radio power | Confirm supply voltage and current against the exact module datasheet. |
| ESP32-S3 GND | Radio module `GND` | Radio ground | Common ground required. |

The firmware uses 420000 baud and transmits CRSF frames at approximately 250 Hz. Although the CRSF class is constructed with a `halfDuplex` flag, the UART is initialized with distinct ESP32 RX and TX pins and the code both transmits and reads. Preserve both UART nets in the PCB unless the exact radio module documentation requires another topology.

### CRSF Channel Assignment

| CRSF channel | Source / state |
|---:|---|
| 1 | Shoulder AS5600 position, mapped from calibrated range to 1000-2000 us. |
| 2 | Upper AS5600 position, mapped from calibrated range to 1000-2000 us. |
| 3 | Lower AS5600 position, mapped from calibrated range to 1000-2000 us. |
| 4 | Hand AS5600 position, mapped from calibrated range to 1000-2000 us. |
| 5 | Arm toggle: 2000 us armed, 1000 us unarmed. |
| 6 | Hold Select on calibration menu `Rx Calibration`: sends 2000 us while unarmed; releasing Select or arming sends 1000 us. |
| 7-16 | Neutral / centered. |

## Not Defined by Current Firmware

No explicit firmware pin assignment exists for LEDs, buzzer, battery-voltage measurement, battery charging, power switching, display backlight control, or encoder magnet-status outputs. These items must not be assumed connected based on this baseline.

## Code Sources

- `src/main.cpp`: GPIO assignments, display, buttons, I2C devices, AS5600 mux channels, and CRSF channel mapping.
- `src/crsf.cpp` and `src/crsf.h`: UART initialization and CRSF timing.
- `platformio.ini`: ESP32-S3 DevKitC-1 board target and platform configuration.
