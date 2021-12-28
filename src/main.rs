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
use rp2040_hal::{pac, gpio::{bank0::Gpio7, bank0::Gpio10, Pin, Input, Floating, Output, PushPull}};

use embedded_hal::digital::v2::OutputPin;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const GET_FW_VERSION: u8 = 0x37u8;

fn esp_select(cs_pin: &mut Pin<Gpio7, Output<PushPull>>) {
    cs_pin.set_low().unwrap();
}

fn esp_deselect(cs_pin: &mut Pin<Gpio7, Output<PushPull>>) {
    cs_pin.set_high().unwrap();
}

fn get_esp_ready(ack_pin: &mut Pin<Gpio10, Input<Floating>>) -> bool {
    true
}

fn get_esp_ack(ack_pin: &mut Pin<Gpio10, Input<Floating>>) -> bool {
    true
}

fn wait_for_esp_ready(ack_pin: &mut Pin<Gpio10, Input<Floating>>) {
    while get_esp_ready(ack_pin) != true {
        cortex_m::asm::nop(); // Make sure rustc doesn't optimize this loop out
    }
}

fn wait_for_esp_ack(ack_pin: &mut Pin<Gpio10, Input<Floating>>) {
    while get_esp_ack(ack_pin) != true {
        cortex_m::asm::nop(); // Make sure rustc doesn't optimize this loop out
    }
}

fn wait_for_esp_select(cs_pin: &mut Pin<Gpio7, Output<PushPull>>,
                       ack_pin: &mut Pin<Gpio10, Input<Floating>>) {
    wait_for_esp_ready(ack_pin);   
    esp_select(cs_pin);
    wait_for_esp_ack(ack_pin);
}

fn wait_response_cmd() {

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

    uart.write_full_blocking(b"ESP32 example\r\n");

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio16.into_mode::<hal::gpio::FunctionSpi>();
    //let _spi_cs = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
    let mut spi_cs = pins.gpio7.into_mode::<hal::gpio::PushPullOutput>();
    let mut spi_ack = pins.gpio10.into_mode::<hal::gpio::FloatingInput>();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Write out 0, ignore return value
    //if spi.write(&[0]).is_ok() {
        // SPI write was succesful
    //};

    wait_for_esp_select(&mut spi_cs, &mut spi_ack);
    //spi_cs.set_low().unwrap();

    // write 0x37, then check the return
    let send_success = spi.send(GET_FW_VERSION);
    match send_success {
        Ok(_) => {
            esp_deselect(&mut spi_cs);
            //spi_cs.set_high().unwrap();
            wait_for_esp_select(&mut spi_cs, &mut spi_ack);

            wait_response_cmd();
            // We succeeded, check the read value
            // if let Ok(x) = spi.read() {
            //     // We got back `x` in exchange for the 0x37 we sent.
            //     writeln!(uart, "ESP32 firmware version: {:?}\r\n", x).ok().unwrap();
            // };

            esp_deselect(&mut spi_cs);
        }
        Err(e) => writeln!(uart, "ESP32 SPI send GET_FW_VERSION err: {:?}\r\n", e).ok().unwrap()
    }


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

        spi_cs.set_low().unwrap();

        // write 0x37, then check the return
        let send_success = spi.send(GET_FW_VERSION);
        match send_success {
            Ok(_) => {
                // We succeeded, check the read value
                if let Ok(x) = spi.read() {
                    // We got back `x` in exchange for the 0x37 we sent.
                    writeln!(uart, "ESP32 firmware version: {:?}\r\n", x).ok().unwrap();
                };
            }
            Err(e) => writeln!(uart, "ESP32 SPI send GET_FW_VERSION err: {:?}\r\n", e).ok().unwrap()
        }
    
        spi_cs.set_high().unwrap();

        delay.delay_ms(1000);
    }
}

// End of file
