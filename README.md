# A Rust-based Pico driver for the ESP32 wireless feather board from Pimoroni

This is a proof-of-concept project that translates enough of [Pimoroni's C++ code](https://github.com/pimoroni/pimoroni-pico/tree/main/examples/pico_wireless) into a Rust implementation to POST to an [Ambi web backend instance](https://github.com/Jim-Hodapp-Coaching/ambi) with live temperature, pressure and humidity values via a [BME280 sensor](https://www.sparkfun.com/products/13676).

This project will eventually turn into a full open source library that implements most or all of the WiFi functionality for use in any embedded Rust application.

## Getting started

For more details see the following article on getting started for getting your environment set up
on Mac/Linux:
https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry

At the time of writing, this code is heavily influenced by the Pimoroni C++ Wifi driver:
https://github.com/pimoroni/pimoroni-pico/blob/main/drivers/esp32spi/

## Hardware

In order to run this code you need to purchase some hardware. This section provides a list of required hardware
needed at minimum, and some suggested items to make your life even easier.

### Required Hardware

1. [Raspberry Pi Pico with pre-soldered headers](https://www.elektor.com/raspberry-pi-pico-rp2040-with-pre-soldered-headers) (2x)
   * [Alternate distributors](https://www.raspberrypi.com/products/raspberry-pi-pico/)

2. [Pimoroni Pico Wireless Pack](https://shop.pimoroni.com/products/pico-wireless-pack?variant=32369508581459) (1x)
   * [Alternate distributor](https://www.elektor.com/pimoroni-raspberry-pi-pico-wireless-pack)

3. [BME280 temperature/pressure/humidity sensor](https://www.sparkfun.com/products/13676) (1x)
   * __Note__: headers are not pre-soldered, but still easy to connect this board with 3 wires
   * [Hookup guide](https://learn.sparkfun.com/tutorials/sparkfun-bme280-breakout-hookup-guide)

4. [Breadboard](https://www.sparkfun.com/products/12614) (1x)
   * __Note__: If you already have a medium/large breadboard, then don't worry about purchasing this specific one


### Optional but Helpful Hardware

1. [Break Away Headers](https://www.sparkfun.com/products/116) (1x)
   * If you want to solder headers to your BME280 sensor board from above

2. [Multi-length Jumper Wire Kit 140pcs](https://www.sparkfun.com/products/124) (1x)

3. [Straight 7" Jumper Wires M/M](https://www.sparkfun.com/products/11026) (1x)
   * Helpful to have some of these on hand

4. [Straight 6" Jumper Wires M/F](https://www.sparkfun.com/products/12794) (1x)
   * Helpful to have some of these on hand

5. [Saleae Logic 8](https://www.saleae.com/) (1x)
   * __Note__: Only needed if you'd like to participate in developing/debugging parts of this project that communicate
   on the SPI/I2C buses
   
### Wiring Details

Start with the section [Pico to Pico Wiring in this article](https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry?s=w) to set up using two Picos together, one as a Picoprobe (flash/debug) and the other as your embedded target.

__Pico to ESP32 WiFi__

The following table lists the pin name and pin number to properly wire between a Pico board and an ESP32 WiFi. This can be done on a breadboard such as the one listed above. Note that V+/- rail means the +/- columns on the breadboard for use as +5 VDC and GND respectively.

| Pico              | ESP32 WiFi       | Breadboard |
| ----------------- | ---------------- | ---------- |
|                   | GND (Pin 3)      | V- rail    |
| GP2 (Pin 4)       | GPIO0 (Pin 4)    |            |
| GP7 (Pin 10)      | ESP_CSn (Pin 10) |            |
| GP8 (Pin 11)      | ESP_RX (Pin 11)  |            |
| GP9 (Pin 12)      | ESP_TX (Pin 12)  |            |
| GP10 (Pin 14)     | ACK (Pin 14)     |            |
| GP11 (Pin 15)     | RESETn (Pin 15)  |            |
| GP12 (Pin 16)     | SW_A (Pin 16)    |            |
|                   | GND (Pin 18)     | V- rail    |
| VBUS (Pin 40)     | VBUS (Pin 40)    |            |
| VSYS (Pin 39)     | VSYS (Pin 39)    | V+ rail    |
| GND (Pin 38)      | GND (Pin 38)     | V- rail    |
| 3V3(OUT) (Pin 36) | 3V3 (Pin 36)     |            |
| GP19 (Pin 25)     | MOSI (Pin 25)    |            |
| GP18 (Pin 24)     | SCLK (Pin 24)    |            |
|                   | GND (Pin 23)     | V- rail    |
| GP16 (Pin 21)     | MISO (Pin 21)    |            |

__BME280 to Pico__

| BME280 | Pico              | Breadboard |
| ------ | ----------------- | ---------- |
| GND    |                   | V- rail    |
| 3.3V   | 3V3(OUT) (Pin 36) |            |
| SDA    | I2C1 SDA (Pin 31) |            |
| SCL    | I2C1 SCL (Pin 32) |            |

***

## Software Requirements
- The standard Rust tooling (cargo, rustup) which you can install from https://rustup.rs/

- Toolchain support for the cortex-m0+ processors in the rp2040 (thumbv6m-none-eabi)

- flip-link - this allows you to detect stack-overflows on the first core, which is the only supported target for now.

## Installation of development dependencies
```
rustup target install thumbv6m-none-eabi
cargo install flip-link
cargo add panic_halt
```

## Running

For a debug build
```
cargo run
```
For a release build
```
cargo run --release
```

To see debug output on the serial console on macOS:
```
minicom -D /dev/tty.usbmodem14201 -b 115200
```
or on Linux:
```
minicom -D /dev/ttyUSB0 -b 115200
```

Note that you'll most likely
need to find the current /dev link assigned to the Pico UART for your particular machine.

## License

This project is licensed under the [BSD + Patent license](https://opensource.org/licenses/BSDplusPatent).

Any submissions to this project (e.g. as Pull Requests) must be made available under these terms.
