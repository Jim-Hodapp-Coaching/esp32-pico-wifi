//! # SPI ESP Pico Wireless Example
//!
//! This application demonstrates how to use the SPI Driver to talk to a remote
//! ESP32 wifi SPI device.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

#![allow(unused_variables)]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
use cortex_m::prelude::*;
use core::fmt::Write;
use embedded_time::rate::Extensions;
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::clocks::Clock;
use rp2040_hal::{pac, gpio::{bank0::Gpio2, bank0::Gpio7, bank0::Gpio10, bank0::Gpio11, bank0::Gpio16, bank0::Gpio18, bank0::Gpio19,
     Pin, Input, FloatingInput, Output, PushPull}};

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::delay::DelayMs;

use crate::hal::spi::Enabled;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const START_CMD: u8 = 0xE0u8;
const END_CMD: u8 = 0xEEu8;
const ERR_CMD: u8 = 0xEFu8;

const PARAMS_ARRAY_LEN: usize = 5;

const REPLY_FLAG: u8 = 1 << 7;

const GET_FW_VERSION: u8 = 0x37u8;

struct Esp32Pins {
    cs: Pin<Gpio7, hal::gpio::PushPullOutput>,
    gpio0: Pin<Gpio2, hal::gpio::PushPullOutput>,
    resetn: Pin<Gpio11, hal::gpio::PushPullOutput>,
    ack: Pin<Gpio10, hal::gpio::FloatingInput>,
}

struct SpiDrv {
    spi: hal::Spi::<Enabled, pac::SPI0, 8>,
    esp32_pins: Esp32Pins,
}

impl SpiDrv {
    fn new(spi: hal::Spi<Enabled, pac::SPI0, 8>,
           pins: Esp32Pins,
        ) -> SpiDrv {
        SpiDrv {
            spi: spi,
            esp32_pins: pins,
        }
    }

    fn init(&mut self) {
        // Chip select is active-low, so we'll initialise it to a driven-high state
        self.esp32_pins.cs.set_high().unwrap();
    }

    pub fn reset<D: DelayMs<u16>>(&mut self, delay: &mut D) {
        self.esp32_pins.gpio0.set_high().unwrap();
        self.esp32_pins.cs.set_high().unwrap();
        self.esp32_pins.resetn.set_low().unwrap();
        delay.delay_ms(10);
        self.esp32_pins.resetn.set_high().unwrap();
        delay.delay_ms(750);
    }

    fn esp_select(&mut self) {
        self.esp32_pins.cs.set_low().unwrap();
    }

    fn esp_deselect(&mut self) {
        self.esp32_pins.cs.set_high().unwrap();
    }

    fn get_esp_ready(&self) -> bool {
        self.esp32_pins.ack.is_low().unwrap()
    }

    fn get_esp_ack(&self) -> bool {
        self.esp32_pins.ack.is_high().unwrap()
    }

    fn wait_for_esp_ready(&self) {
        while self.get_esp_ready() != true {
            cortex_m::asm::nop(); // Make sure rustc doesn't optimize this loop out
        }
    }

    fn wait_for_esp_ack(&self) {
        while self.get_esp_ack() != true {
            cortex_m::asm::nop(); // Make sure rustc doesn't optimize this loop out
        }
    }

    fn wait_for_esp_select(&mut self) {
        self.wait_for_esp_ready();   
        self.esp_select();
        self.wait_for_esp_ack();
    }

    fn read_byte(&mut self) -> u8 {
        self.get_param()
    }

    fn read_and_check_byte(&mut self, check_byte: u8) -> Result<bool, u8> {
        let byte_out = self.get_param();
        if byte_out == check_byte {
            Ok(true)
        } else {
            Err(byte_out)
        }
    }

    fn wait_for_byte(&mut self, wait_byte: u8) -> bool {
        let mut timeout: u16 = 1000u16;

        let byte_read = loop {
            let byte_read = self.read_byte();
            if byte_read == ERR_CMD {
                break ERR_CMD;
            } else if byte_read == wait_byte {
                break byte_read;
            } else if timeout == 0 {
                break ERR_CMD;
            }
            timeout -= 1;
        };
        byte_read == wait_byte
    }

    fn check_start_cmd(&mut self) -> bool {
        self.wait_for_byte(START_CMD)
    } 

    fn wait_response_cmd(&mut self, cmd: u8, num_param: u8) -> Result<[u8; PARAMS_ARRAY_LEN], u8> {
        // TODO: can we turn this into more of a functional syntax to clean
        // up the deep nesting?
        if self.check_start_cmd() {
            let check_result = self.read_and_check_byte(cmd | REPLY_FLAG);
            match check_result {
                Ok(_) => {
                    let check_result = self.read_and_check_byte(num_param);
                    match check_result {
                        Ok(_) => {
                            let num_param_read: usize = self.get_param() as usize;
                            let mut i: usize = 0;
                            let mut params: [u8; PARAMS_ARRAY_LEN] = [0, 0, 0, 0, 0];
                            while i < num_param_read {
                                params[i] = self.get_param();
                                i += 1;
                            }

                            let check_result = self.read_and_check_byte(END_CMD);
                            match check_result {
                                Ok(_) => {
                                    Ok(params)
                                }
                                Err(wrong_byte) => {
                                    Err(wrong_byte)
                                }
                            }
                        }
                        Err(wrong_byte) => {
                            Err(wrong_byte)
                        }
                    }
                }
                Err(wrong_byte) => {
                    Err(wrong_byte)
                }
            }
        } else {
            Err(START_CMD)
        }
    }

    fn get_param(&mut self) -> u8 {
        // Blocking read, don't return until we've read a byte successfully
        let byte = loop {
            let read_success = self.spi.read();
            match read_success {
                Ok(byte) => {
                    break byte;
                }
                Err(e) => todo!(),
            }
        };
        byte
    }

    fn send_cmd(&mut self, cmd: u8, num_param: u8) {
        let _send_success = self.spi.send(START_CMD).ok().unwrap();
        let _send_success = self.spi.send(cmd & !(REPLY_FLAG)).ok().unwrap();
        let _send_success = self.spi.send(num_param).ok().unwrap();
        if num_param == 0 {
            let _send_success = self.spi.send(END_CMD).ok().unwrap();
        }
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters reveived by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );

    let mut uart = hal::uart::UartPeripheral::<_, _, _>::new(pac.UART0, uart_pins, &mut pac.RESETS)
    .enable(
        hal::uart::common_configs::_115200_8_N_1,
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    uart.write_full_blocking(b"ESP32 example\r\n");

    // init()
    // These are implicitly used by the spi driver if they are in the correct mode
    let spi_miso = pins.gpio16.into_mode::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let esp32_pins = Esp32Pins {
        // Chip select is active-low, so we'll initialise it to a driven-high state
        cs: pins.gpio7.into_mode::<hal::gpio::PushPullOutput>(),
        gpio0: pins.gpio2.into_mode::<hal::gpio::PushPullOutput>(),
        resetn: pins.gpio11.into_mode::<hal::gpio::PushPullOutput>(),
        ack: pins.gpio10.into_mode::<hal::gpio::FloatingInput>(),
    };

    // Note: Pass an Enabled Spi instance
    let mut spi_drv = SpiDrv::new(spi, esp32_pins);

    spi_drv.init();
    spi_drv.reset(&mut delay);

    //delay.delay_ms(500);

    // uart.write_full_blocking(b"wait_for_esp_select()\r\n");
    // spi_drv.wait_for_esp_select();
    // // //spi_cs.set_low().unwrap();

    // uart.write_full_blocking(b"send_cmd(GET_FW_VERSION)\r\n");
    // // // write 0x37, then check the return
    // spi_drv.send_cmd(GET_FW_VERSION, 0);

    // spi_drv.esp_deselect();
    // spi_drv.wait_for_esp_select();

    // // Get the ESP32 firmware version
    // let wait_response = spi_drv.wait_response_cmd(GET_FW_VERSION, 1);
    // match wait_response {
    //     Ok(params) => {
    //         writeln!(uart, "ESP32 firmware version: {:?}\r\n", params[0])
    //             .ok()
    //             .unwrap();
    //     }
    //     Err(wrong_byte) => {
    //         writeln!(uart, "ESP32 SPI send GET_FW_VERSION wrong_byte: {:?}\r\n", wrong_byte)
    //             .ok()
    //             .unwrap() 
    //     }
    // }

    // spi_drv.esp_deselect();

    // write 0x37, then check the return
    // let send_success = spi.send(GET_FW_VERSION);
    // match send_success {
    //     Ok(_) => {
    //         spi_drv.esp_deselect();
    //         //spi_cs.set_high().unwrap();
    //         spi_drv.wait_for_esp_select();

    //         spi_drv.wait_response_cmd();
    //         // We succeeded, check the read value
    //         // if let Ok(x) = spi.read() {
    //         //     // We got back `x` in exchange for the 0x37 we sent.
    //         //     writeln!(uart, "ESP32 firmware version: {:?}\r\n", x).ok().unwrap();
    //         // };

    //         spi_drv.esp_deselect();
    //     }
    //     Err(e) => writeln!(uart, "ESP32 SPI send GET_FW_VERSION err: {:?}\r\n", e).ok().unwrap()
    // }


    // Do a read+write at the same time. Data in `buffer` will be replaced with
    // the data read from the SPI device.
    //let mut buffer: [u8; 4] = [1, 2, 3, 4];
    //let transfer_success = spi.transfer(&mut buffer);
    //#[allow(clippy::single_match)]
    //match transfer_success {
    //    Ok(_) => {}  // Handle success
    //    Err(_) => {} // handle errors
    //};

    loop {
        // delay.delay_ms(5);
        // spi_cs.set_low().unwrap();
        // delay.delay_ms(5);
        // spi_cs.set_high().unwrap();
        // delay.delay_ms(10);

        // spi_cs.set_low().unwrap();

        // // write 0x37, then check the return
        // let send_success = spi.send(GET_FW_VERSION);
        // match send_success {
        //     Ok(_) => {
        //         // We succeeded, check the read value
        //         if let Ok(x) = spi.read() {
        //             // We got back `x` in exchange for the 0x37 we sent.
        //             writeln!(uart, "ESP32 firmware version: {:?}\r\n", x).ok().unwrap();
        //         };
        //     }
        //     Err(e) => writeln!(uart, "ESP32 SPI send GET_FW_VERSION err: {:?}\r\n", e).ok().unwrap()
        // }
    
        // spi_cs.set_high().unwrap();

        // delay.delay_ms(1000);
    }
}

// End of file
