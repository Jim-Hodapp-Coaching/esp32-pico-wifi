# A Rust-based Pico driver for the ESP32 wireless feather board from Pimoroni

## Getting started

For more details see the following article on getting started for getting your environment set up
on Mac/Linux:
https://reltech.substack.com/p/getting-started-with-rust-on-a-raspberry

At the time of writing, this code is heavily influenced by the Pimoroni C++ Wifi driver:
https://github.com/pimoroni/pimoroni-pico/blob/main/drivers/esp32spi/

## Requirements
- The standard Rust tooling (cargo, rustup) which you can install from https://rustup.rs/

- Toolchain support for the cortex-m0+ processors in the rp2040 (thumbv6m-none-eabi)

- flip-link - this allows you to detect stack-overflows on the first core, which is the only supported target for now.

## Installation of development dependencies
```
rustup target install thumbv6m-none-eabi
cargo install --git https://github.com/rp-rs/probe-run --branch rp2040-support
cargo install flip-link
cargo install cargo-edit
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
