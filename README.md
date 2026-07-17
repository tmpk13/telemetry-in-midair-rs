# telemetry-in-midair-rs
[Kicad Board](https://github.com/tmpk13/telemetry-in-midair) https://github.com/tmpk13/telemetry-in-midair

## ESP32-C6
`ESP32-C6-MINI-1U-H4`
`4MB Flash`

| Pin | Function |
|-|-|
| I03 | LED D2 |
| IO2 | PWR EN GPS/Radio (AP2112K-3.3) |
| IO4 | RX/GPIO |
| IO5 | TX/GPIO |
| IO6 | WIO-E5 RST |
| RXD0 | WIO-E5 PA2 |
| TXD0 | WIO-E5 PA3 |
| IO12 | USB D- |
| IO13 | USB D+ |

*Boot pad on back*

## WIO-E5

| Pin | Function |
|-|-|
| PB6 (TX) | GPS RX |
| PB7 (RX) | GPS TX |
| PB10 | EXT INT GPS |
| PC1 | I2C SCL (JST SH) |
| PC1 | I2C SDA (JST SH) |
| PB3 | SD SCK |
| PB4 | SD CITO |
| PB5 | SD COTI |
| PA0 | SD CS |
| PA9 | LED D6 |
| PA10 | LED D5 |

*Reset (RST) pad on back*


## Connectors
#### JST SH
*As of Version 1*

**I2C** *(J6)*

| Pin | Function |
|-|-|
| 4 | SCL |
| 3 | SDA |
| 2 | 3V3 |
| 1 | GND |

**SWD** *(J5)*

| Pin | Function |
|-|-|
| 4 | SWDIO |
| 3 | SWDCLK |
| 2 | 3V3 |
| 1 | GND |



