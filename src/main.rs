//! # SPI ESP Pico Wireless Example
//!
//! This application demonstrates how to use the SPI Driver to talk to a remote
//! ESP32 wifi SPI device.
//! 
//! It's based off of the Pimoroni C++ code located here:
//! https://github.com/pimoroni/pimoroni-pico/tree/main/examples/pico_wireless
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
use rp2040_hal::{pac, gpio::{bank0::Gpio2, bank0::Gpio7, bank0::Gpio10, bank0::Gpio11, Pin}};
use rp2040_hal::gpio::bank0::{Gpio0, Gpio1};

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::delay::DelayMs;

use crate::hal::spi::Enabled;

use heapless::String;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const DUMMY_DATA: u8 = 0xFFu8;

const START_CMD: u8 = 0xE0u8;
const END_CMD: u8 = 0xEEu8;
const ERR_CMD: u8 = 0xEFu8;

const CMD_FLAG: u8 = 0;
const REPLY_FLAG: u8 = 1 << 7;
const DATA_FLAG: u8 = 0x40u8;

const PARAMS_ARRAY_LEN: usize = 8;
const STR_LEN: usize = 24;

const ESP_LED_R: u8 = 25;
const ESP_LED_G: u8 = 26;
const ESP_LED_B: u8 = 27;

const SET_PASSPHRASE: u8 = 0x11u8;
const GET_FW_VERSION: u8 = 0x37u8;
const GET_CONN_STATUS: u8 = 0x20u8;
const GET_SOCKET: u8 = 0x3fu8;

const SET_ANALOG_WRITE: u8 = 0x52u8;

type SpiResult<T> = Result<T, nb::Error<core::convert::Infallible>>;

type EnabledUart = hal::uart::UartPeripheral<rp2040_hal::uart::Enabled, pac::UART0,
    (rp2040_hal::gpio::Pin<Gpio0, rp2040_hal::gpio::Function<rp2040_hal::gpio::Uart>>, rp2040_hal::gpio::Pin<Gpio1, rp2040_hal::gpio::Function<rp2040_hal::gpio::Uart>>)>;

type Params = [u8];

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
        while !self.get_esp_ack() {
            cortex_m::asm::nop(); // Make sure rustc doesn't optimize this loop out
        }
    }

    fn wait_for_esp_select(&mut self) {
        self.wait_for_esp_ready();   
        self.esp_select();
        self.wait_for_esp_ack();
    }

    fn get_param(&mut self, uart: &mut EnabledUart) -> Result<u8, nb::Error<core::convert::Infallible>> {
        // Blocking read, don't return until we've read a byte successfully
        loop {
            let word_out = &mut[DUMMY_DATA];
            write!(uart, "\t\tsending byte: 0x{:X?} -> ", DUMMY_DATA).ok().unwrap();
            let read_result = self.spi.transfer(word_out);
            match read_result {
                Ok(word) => {
                    let byte: u8 = word[0] as u8;
                    write!(uart, "get_param() read byte: 0x{:X?}\r\n", byte).ok().unwrap();
                    return Ok(byte);
                }
                Err(e) => { continue; }
            }
        }
    }

    fn read_byte(&mut self, uart: &mut EnabledUart) -> SpiResult<u8> {
        let result = self.get_param(uart);
        match result {
            Ok(byte_out) => { return Ok(byte_out); }
            Err(e) => { return Err(e); }
        }
    }

    fn read_and_check_byte(&mut self, uart: &mut EnabledUart, check_byte: u8) -> SpiResult<bool> {
        let result = self.get_param(uart);
        match result {
            Ok(byte_out) => {
                write!(uart, "\tread_and_check_byte(): 0x{:X?} == 0x{:X?}: {:?}\r\n",
                    byte_out, check_byte, byte_out == check_byte).ok().unwrap();
                return Ok(byte_out == check_byte);
            }
            Err(e) => { return Err(e); }
        }
    }

    fn wait_for_byte(&mut self, uart: &mut EnabledUart, wait_byte: u8) -> SpiResult<bool> {
        let mut timeout: u16 = 1000u16;

        loop {
            let result = self.read_byte(uart);
            match result {
                Ok(byte_read) => {
                    if byte_read == ERR_CMD {
                        return Err(nb::Error::WouldBlock);
                    } else if byte_read == wait_byte {
                        return Ok(true);
                    } else if timeout == 0 {
                        return Ok(false);
                    }
                    timeout -= 1;
                }
                Err(e) => { return Err(e); }
            }
        }
    }

    fn check_start_cmd(&mut self, uart: &mut EnabledUart) -> SpiResult<bool> {
        let result = self.wait_for_byte(uart, START_CMD);
        match result {
            Ok(b) => { return Ok(b); }
            Err(e) => { return Err(e); }
        }
    } 

    fn wait_response_cmd(&mut self, uart: &mut EnabledUart, cmd: u8, num_param: u8) -> SpiResult<[u8; PARAMS_ARRAY_LEN]> {
        // TODO: can we turn this into more of a functional syntax to clean
        // up the deep nesting? Investigate `map` for `Result` in Rust by Example, or use of Combinators
        let result = self.check_start_cmd(uart);
        match result {
            Ok(b) => {
                uart.write_full_blocking(b"\tSuccess: check_start_cmd()\r\n");
                let check_result = self.read_and_check_byte(uart, cmd | REPLY_FLAG);
                match check_result {
                    Ok(_) => {
                        uart.write_full_blocking(b"\tSuccess: read_and_check_byte(cmd | REPLY_FLAG)\r\n");
                        let check_result = self.read_and_check_byte(uart, num_param);
                        match check_result {
                            Ok(_) => {
                                uart.write_full_blocking(b"\tSuccess: read_and_check_byte(num_param)\r\n");
                                let num_param_read: usize = self.get_param(uart).ok().unwrap() as usize;
                                write!(uart, "\t\tnum_param_read: {:?}\r\n", num_param_read).ok().unwrap();
                                if num_param_read > PARAMS_ARRAY_LEN {
                                    // TODO: refactor the type of error this method returns away from nb::Error,
                                    // perhaps to something custom
                                    return Err(nb::Error::WouldBlock);
                                }
                                let mut i: usize = 0;
                                let mut params: [u8; PARAMS_ARRAY_LEN] = [0, 0, 0, 0, 0, 0, 0, 0];
                                while i < num_param_read {
                                    params[i] = self.get_param(uart).ok().unwrap();
                                    write!(uart, "\t\tparams[{:?}]: 0x{:X?}\r\n", i, params[i]).ok().unwrap();
                                    i += 1;
                                }

                                let check_result = self.read_and_check_byte(uart, END_CMD);
                                match check_result {
                                    Ok(_) => {
                                        uart.write_full_blocking(b"\tSuccess: read_and_check_byte(END_CMD)\r\n");
                                        Ok(params)
                                    }
                                    Err(wrong_byte) => {
                                        uart.write_full_blocking(b"\tFailed to read_and_check_byte(END_CMD)\r\n");
                                        Err(wrong_byte)
                                    }
                                }
                            }
                            Err(wrong_byte) => {
                                uart.write_full_blocking(b"\tFailed to read_and_check_byte(num_param)\r\n");
                                Err(wrong_byte)
                            }
                        }
                    }
                    Err(wrong_byte) => {
                        uart.write_full_blocking(b"\tFailed to read_and_check_byte(cmd | REPLY_FLAG)\r\n");
                        Err(wrong_byte)
                    }
                }
            }
            Err(e) => {
                uart.write_full_blocking(b"\tFailed to check_start_cmd()\r\n");
                return Err(e); }
        }
    }

    fn send_cmd(&mut self, uart: &mut EnabledUart, cmd: u8, num_param: u8) -> SpiResult<()> {
        let buf: [u8; 3] = [START_CMD,
                            cmd & !(REPLY_FLAG),
                            num_param];
        for byte in buf {
            let byte_buf = &mut[byte];
            write!(uart, "\t\tsending byte: 0x{:X?} -> ", byte).ok().unwrap();
            let transfer_results = self.spi.transfer(byte_buf);
            match transfer_results {
                Ok(byte) => { 
                  write!(uart, "read byte: 0x{:X?}\r\n", byte).ok().unwrap();
                  continue; 
                }
                Err(e) => {
                  write!(uart, "send_cmd transfer error: 0x{:X?}\r\n", e).ok().unwrap();
                  continue; 
                }
            }
        }

        if num_param == 0 {
            let byte_buf = &mut[END_CMD];
            write!(uart, "\t\tsending byte: 0x{:X?} -> ", END_CMD).ok().unwrap();
            let transfer_results = self.spi.transfer(byte_buf);
            match transfer_results {
                Ok(byte) => {
                    write!(uart, "read byte: 0x{:X?}\r\n", byte).ok().unwrap();
                    return Ok(()
                ); }
                Err(e) => {
                    write!(uart, "send_cmd transfer error: 0x{:X?}\r\n", e).ok().unwrap();
                    return Err(nb::Error::WouldBlock);
                }
            }
        }
        Ok(())
    }

    fn send_param_len8(&mut self, uart: &mut EnabledUart, param_len: u8) -> SpiResult<()> {
        let byte_buf = &mut[param_len];
        write!(uart, "\t\tsending byte: 0x{:X?} -> ", param_len).ok().unwrap();
        let transfer_results = self.spi.transfer(byte_buf);
        match transfer_results {
            Ok(byte) => {
                write!(uart, "read byte: 0x{:X?}\r\n", byte).ok().unwrap();
                return Ok(());
            }
            Err(e) => {
                return Err(nb::Error::WouldBlock);
            }
        }
    }

    // TODO: replace last_param with an enumerated type, e.g. NO_LAST_PARAM, LAST_PARAM
    fn send_param(&mut self, uart: &mut EnabledUart, params: &mut Params, last_param: bool) -> SpiResult<()> {
        let param_len: u8 = params.len() as u8;
        let res = self.send_param_len8(uart, param_len);
        match res {
            Ok(_) => {
                // TODO: this doesn't quite match the C++ code yet, seems it can send a
                // variable length buf
                let byte_buf = params;
                let transfer_results = self.spi.transfer(byte_buf);
                match transfer_results {
                    Ok(transfer_buf) => {
                        write!(uart, "\t\tread bytes: 0x{:X?}\r\n", transfer_buf).ok().unwrap();
                        if last_param {
                            let end_command = &mut[END_CMD];
                            write!(uart, "\t\t\tsending byte: 0x{:X?} -> ", end_command).ok().unwrap();
                            let transfer_results = self.spi.transfer(end_command);
                            match transfer_results {
                                Ok(byte) => {
                                    write!(uart, "\t\tread byte: 0x{:X?}\r\n", byte).ok().unwrap();
                                    return Ok(());
                                }
                                Err(e) => {
                                    write!(uart, "\t\t\tsend_param transfer error: 0x{:X?}\r\n", e).ok().unwrap();
                                    return Err(nb::Error::WouldBlock);
                                }
                            } 
                        } else {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        write!(uart, "send_param transfer error: 0x{:X?}\r\n", e).ok().unwrap();
                        return Err(nb::Error::WouldBlock);
                    }
                }
            }
            Err(e) => { return Err(e); }
        }
    }

    // TODO: replace last_param with an enumerated type, e.g. NO_LAST_PARAM, LAST_PARAM
    fn send_param_word(&mut self, uart: &mut EnabledUart, param: u16, last_param: bool) -> SpiResult<()> {
        let res = self.send_param_len8(uart, 2);
        match res {
            Ok(_) => {
                let buf: [u8; 2] = [ ((param & 0xff00) >> 8) as u8, (param & 0xff) as u8 ];
                // FIXME: send both buf bytes, not just the first one
                // FIXME: switch to using transfer(), not send()
                let transfer_results = self.spi.send(buf[0]);
                match transfer_results {
                    Ok(_) => {
                        if last_param {
                            let buf: [u8; 1] = [END_CMD];
                            // FIXME: switch to using transfer(), not send()
                            let transfer_results = self.spi.send(buf[0]);
                            match transfer_results {
                                Ok(_) => { return Ok(()); }
                                Err(e) => { return Err(e); }
                            } 
                        } else {
                            return Ok(());
                        }
                    }
                    Err(e) => { return Err(e); }
                }
            }
            Err(e) => { return Err(e); }
        }
    }

    fn pad_to_multiple_of_4(&mut self, uart: &mut EnabledUart, mut command_size: u8) {
        while command_size % 4 == 0 {
            self.read_byte(uart).ok().unwrap();
            command_size += 1;
        }
    }
}

fn set_led(spi_drv: &mut SpiDrv, uart: &mut EnabledUart, red: u8, green: u8, blue: u8) {
    write!(uart, "Calling analog_write(ESP_LED_R, {:?})\r\n", 255 - red).ok().unwrap();
    analog_write(spi_drv, uart, ESP_LED_R, 255 - red);

    write!(uart, "Calling analog_write(ESP_LED_G, {:?})\r\n", 255 - green).ok().unwrap();
    analog_write(spi_drv, uart, ESP_LED_G, 255 - green);

    write!(uart, "Calling analog_write(ESP_LED_B, {:?})\r\n", 255 - blue).ok().unwrap();
    analog_write(spi_drv, uart, ESP_LED_B, 255 - blue);
}

fn analog_write(spi_drv: &mut SpiDrv, uart: &mut EnabledUart, pin: u8, value: u8) {
    uart.write_full_blocking(b"\twait_for_esp_select()\r\n");
    spi_drv.wait_for_esp_select();

    uart.write_full_blocking(b"\tsend_cmd(SET_ANALOG_WRITE)\r\n");
    spi_drv.send_cmd(uart, SET_ANALOG_WRITE, 2).ok().unwrap();
    uart.write_full_blocking(b"\tsend_param(pin)\r\n");
    let pin_byte: &mut [u8] = &mut [pin];
    spi_drv.send_param(uart, pin_byte, false).ok().unwrap();
    uart.write_full_blocking(b"\tsend_param(value)\r\n");
    let value_byte: &mut [u8] = &mut [value];
    spi_drv.send_param(uart, value_byte, true).ok().unwrap(); // LAST_PARAM

    uart.write_full_blocking(b"\tread_byte()\r\n");
    spi_drv.read_byte(uart).ok().unwrap();

    uart.write_full_blocking(b"\tesp_deselect()\r\n");
    spi_drv.esp_deselect();
    uart.write_full_blocking(b"\twait_for_esp_select()\r\n");
    spi_drv.wait_for_esp_select();

    // Wait for a reply from ESP32
    uart.write_full_blocking(b"\twait_response_cmd()\r\n");
    let wait_response = spi_drv.wait_response_cmd(uart, SET_ANALOG_WRITE, 1);
    match wait_response {
        Ok(params) => {
            write!(uart, "\tSET_ANALOG_WRITE: ").ok().unwrap();
            for byte in params {
                let c = byte as char;
                write!(uart, "{:?}", c).ok().unwrap();
            }
            writeln!(uart, "\r\n").ok().unwrap();
        }
        Err(e) => {
            writeln!(uart, "\twait_response_cmd(SET_ANALOG_WRITE) Err: {:?}\r", e).ok().unwrap();
        }
    }

    uart.write_full_blocking(b"\tesp_deselect()\r\n");
    spi_drv.esp_deselect();
}

fn wifi_set_passphrase(spi_drv: &mut SpiDrv, uart: &mut EnabledUart, mut ssid: String<STR_LEN>, mut passphrase: String<STR_LEN>) -> bool {
    uart.write_full_blocking(b"wifi_set_passphrase()\r\n");
    spi_drv.wait_for_esp_select();

    spi_drv.send_cmd(uart, SET_PASSPHRASE, 2).ok().unwrap();

    // FIXME: for the real crate, don't use unsafe
    let ssid_bytes: &mut [u8] = unsafe { ssid.as_bytes_mut() };
    writeln!(uart, "\tssid: {:?}\r", ssid_bytes).ok().unwrap();
    spi_drv.send_param(uart, ssid_bytes, false).ok().unwrap();

    // FIXME: for the real crate, don't use unsafe
    let passphrase_bytes: &mut [u8] = unsafe { passphrase.as_bytes_mut() };
    writeln!(uart, "\tpassphrase: {:?}\r", passphrase_bytes).ok().unwrap();
    spi_drv.send_param(uart, passphrase_bytes, true).ok().unwrap();

    let command_size: u8 = 6 + ssid.len() as u8 + passphrase.len() as u8;
    spi_drv.pad_to_multiple_of_4(uart, command_size);
 
    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    // Wait for reply
    let wait_response = spi_drv.wait_response_cmd(uart, SET_PASSPHRASE, 1);
    match wait_response {
        Ok(params) => {
            write!(uart, "\twifi_set_passphrase_response: ").ok().unwrap();
            for byte in params {
                let c = byte as char;
                write!(uart, "{:?}", c).ok().unwrap();
            }
            writeln!(uart, "\r\n").ok().unwrap();
        }
        Err(e) => {
            writeln!(uart, "\twifi_set_passphrase_response Err: {:?}\r", e).ok().unwrap();
            spi_drv.esp_deselect();
            return false;
        }
    }

    spi_drv.esp_deselect();

    true
}

fn get_connection_status(spi_drv: &mut SpiDrv, uart: &mut EnabledUart) -> SpiResult<bool> {
    spi_drv.wait_for_esp_select(); 
   
    spi_drv.send_cmd(uart, GET_CONN_STATUS, 0).ok().unwrap();

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    // Wait for reply
    let mut connected = false;
    let wait_response = spi_drv.wait_response_cmd(uart, GET_CONN_STATUS, 1);
    match wait_response {
        Ok(params) => {
            write!(uart, "\tget_connection_status_response: {:?}\r\n", params[0]).ok().unwrap();
            // TODO: Replace connected status with enumerated type (i.e. 0x3 in this case)
            connected = params[0] == 0x3;
        }
        Err(e) => {
            writeln!(uart, "\tget_connection_status_response Err: {:?}\r", e).ok().unwrap();
            spi_drv.esp_deselect();
            return Err(e);
        }
    }
    spi_drv.esp_deselect();

    Ok(connected)
}

fn get_fw_version(spi_drv: &mut SpiDrv, uart: &mut EnabledUart) -> bool {
    uart.write_full_blocking(b"wait_for_esp_select()\r\n");
    spi_drv.wait_for_esp_select();
    uart.write_full_blocking(b"\tesp selected\r\n");

    uart.write_full_blocking(b"send_cmd(GET_FW_VERSION)\r\n");
    let results = spi_drv.send_cmd(uart, GET_FW_VERSION, 0);
    match results {
        Ok(_) => { uart.write_full_blocking(b"\tsent GET_FW_VERSION command\r\n"); }
        Err(e) => { writeln!(uart, "\t** Failed to send GET_FW_VERSION command: {:?}\r\n", e).ok().unwrap();
        return false;
        }
    }
    spi_drv.esp_deselect();
    uart.write_full_blocking(b"esp_deselect()\r\n");

    uart.write_full_blocking(b"\r\nNow waiting for firmware version response...\r\n");
    uart.write_full_blocking(b"wait_for_esp_select()\r\n");
    spi_drv.wait_for_esp_select();
    uart.write_full_blocking(b"\tesp selected\r\n");

    // Get the ESP32 firmware version
    uart.write_full_blocking(b"wait_response_cmd()\r\n");
    let wait_response = spi_drv.wait_response_cmd(uart, GET_FW_VERSION, 1);
    match wait_response {
        Ok(params) => {
            write!(uart, "\tESP32 firmware version: ").ok().unwrap();
                for byte in params {
                    let c = byte as char;
                    write!(uart, "{:?}", c).ok().unwrap();
            }
            writeln!(uart, "\r\n").ok().unwrap();
        }
        Err(e) => {
            writeln!(uart, "\twait_response_cmd(GET_FW_VERSION) Err: {:?}\r", e)
                .ok()
                .unwrap();
            return false;
        }
    }
    uart.write_full_blocking(b"wait_response_cmd() returned\r\n");
    spi_drv.esp_deselect();
    uart.write_full_blocking(b"esp_deselect()\r\n");

    true
}

fn get_socket(spi_drv: &mut SpiDrv, uart: &mut EnabledUart) -> SpiResult<u8> {
    spi_drv.wait_for_esp_select();
    
    let results = spi_drv.send_cmd(uart, GET_SOCKET, 0);
    match results {
        Ok(_) => { uart.write_full_blocking(b"\tSent GET_SOCKET command\r\n"); }
        Err(e) => { writeln!(uart, "\t** Failed to send GET_SOCKET command: {:?}\r\n", e).ok().unwrap(); }
    }

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    let mut socket = 0;
    let wait_response = spi_drv.wait_response_cmd(uart, GET_FW_VERSION, 1);
    match wait_response {
        Ok(params) => {
            write!(uart, "\tget_socket: {:?}\r\n", params[0]).ok().unwrap();
            socket = params[0];
            spi_drv.esp_deselect();
            Ok(socket)
        }
        Err(e) => {
            writeln!(uart, "\twait_response_cmd(GET_SOCKET) Err: {:?}\r", e)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            Err(e)
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

    uart.write_full_blocking(b"\r\nESP32 Wifi PoC (pre-crate)\r\n");

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
        500_000u32.Hz(),
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

    // Turn the ESP32's onboard multi-color LED off
    set_led(&mut spi_drv, &mut uart, 0, 0, 0);
    delay.delay_ms(500);

    // Set wifi passphrase - ESP32 will attempt to connect after receving this cmd
    wifi_set_passphrase(&mut spi_drv, &mut uart, String::from("ssid"), String::from("password"));
    delay.delay_ms(1000);

    let led_pin = pins.gpio25.into_push_pull_output();

    let socket = get_socket(&mut spi_drv, &mut uart).ok().unwrap();
    writeln!(uart, "socket: {:?}\r\n", socket).ok().unwrap();

    let mut i: u32 = 0;
    loop {
        // Check for connection in loop and set led on if connected succesfully
        let result = get_connection_status(&mut spi_drv, &mut uart);
        match result {
            Ok(connected) => {
                if connected {
                    uart.write_full_blocking(b"** Connected to WiFi\r\n");
                    // Set ESP32 LED green when successfully connected to WiFi AP
                    set_led(&mut spi_drv, &mut uart, 0, 255, 0);
                }
                else {
                    uart.write_full_blocking(b"** Not connected to WiFi\r\n");
                    // Set ESP32 LED green when successfully connected to WiFi AP
                    set_led(&mut spi_drv, &mut uart, 255, 0, 0);
                }
            }
            Err(e) => {
                uart.write_full_blocking(b"** Failed to get WiFi connection status\r\n");
            }
        }

        write!(uart, "Loop ({:?}) ...\r", i).ok().unwrap();

        delay.delay_ms(1000);
        i += 1;
    }
}

// End of file
