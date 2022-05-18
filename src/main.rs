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

// Needed for debug output symbols to be linked in binary image
use defmt_rtt as _;

// Ensure we halt the program on panic and print a backtrace
use panic_probe as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
use core::fmt::Write;
use cortex_m::prelude::*;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use rp2040_hal::clocks::Clock;
use rp2040_hal::i2c::I2C;
use rp2040_hal::gpio::bank0::{Gpio0, Gpio1, Gpio2, Gpio7, Gpio10, Gpio11};
use rp2040_hal::gpio::Pin;
use rp2040_hal::pac;

use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::digital::blocking::InputPin;
use embedded_hal::digital::blocking::OutputPin;
use embedded_hal_02::spi::MODE_0;

use crate::hal::spi::Enabled;

use no_std_net::{Ipv4Addr, SocketAddrV4};

use bme280::i2c::BME280;

include!("secrets.rs");

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

const _CMD_FLAG: u8 = 0;
const REPLY_FLAG: u8 = 1 << 7;
const _DATA_FLAG: u8 = 0x40u8;

const PARAMS_ARRAY_LEN: usize = 8;
const STR_LEN: usize = 512;
const RESPONSE_BUF_LEN: usize = 1024;

const ESP_LED_R: u8 = 25;
const ESP_LED_G: u8 = 26;
const ESP_LED_B: u8 = 27;

const SET_PASSPHRASE: u8 = 0x11u8;
const GET_FW_VERSION: u8 = 0x37u8;
const GET_CONN_STATUS: u8 = 0x20u8;
const GET_SOCKET: u8 = 0x3fu8;
const START_CLIENT_TCP: u8 = 0x2d;
const SEND_DATA_TCP: u8 = 0x44;
const AVAIL_DATA_TCP: u8 = 0x2b;
const GET_DATABUF_TCP: u8 = 0x45;
const STOP_CLIENT_TCP: u8 = 0x2e;

const SET_ANALOG_WRITE: u8 = 0x52u8;
const GET_CLIENT_STATE_TCP: u8 = 0x2fu8;

// TODO: this should be renamed or split up so SPI-level functionality and functional layers
// above can use these errors without being confused by the naming (i.e. SpiDrv...)
#[derive(Debug)]
enum SpiDrvError {
    // Receive data errors
    CmdResponseError,           // Received a CMD_ERR byte from the ESP32
    CmdResponseTimeout,         // SPI data reading timed out while waiting on the ESP32
    CmdResponseCheckFailed(u8), // Expected a certain byte but received a different byte
    CmdResponseInvalidParamNum, // Received an unexpected number of params back from the ESP32

    // Send (transfer) data errors
    TransferFailed,    // Failed to send a byte of data to the ESP32
    ServerCommTimeout, // Communication from client to server timed out
}

// Defines the mode types that the ESP32 firmware can be put into when starting
// a new client or server instance
#[repr(u8)]
#[derive(Debug)]
#[allow(dead_code)]
enum SvProtocolMode {
    TCP = 0,
    UDP = 1,
    TLS = 2,
    UDPMulticast = 3,
    TLSBearSSL = 4,
}

// Defines all possible TCP connection states
#[repr(u8)]
#[derive(PartialEq, PartialOrd, Debug)]
enum WlTcpState {
    Closed = 0,
    Listen = 1,
    SYNSent = 2,
    SYNReceived = 3,
    Established = 4,
    FinWait1 = 5,
    FinWait2 = 6,
    CloseWait = 7,
    Closing = 8,
    LastAck = 9,
    TimeWait = 10,
}

// Implements both from() and into()
impl From<u8> for WlTcpState {
    fn from(state_u8: u8) -> Self {
        if state_u8 == 0 {
            return WlTcpState::Closed;
        } else if state_u8 == 1 {
            return WlTcpState::Listen;
        } else if state_u8 == 2 {
            return WlTcpState::SYNSent;
        } else if state_u8 == 3 {
            return WlTcpState::SYNReceived;
        } else if state_u8 == 4 {
            return WlTcpState::Established;
        } else if state_u8 == 5 {
            return WlTcpState::FinWait1;
        } else if state_u8 == 6 {
            return WlTcpState::FinWait2;
        } else if state_u8 == 7 {
            return WlTcpState::CloseWait;
        } else if state_u8 == 8 {
            return WlTcpState::Closing;
        } else if state_u8 == 9 {
            return WlTcpState::LastAck;
        } else if state_u8 == 10 {
            return WlTcpState::TimeWait;
        } else {
            return WlTcpState::Closed;
        }
    }
}

// Defines the WiFi network connection status
#[repr(u8)]
#[derive(PartialEq, PartialOrd, Debug)]
enum WlStatus {
    NoShield = 255,
    IdleStatus = 0,
    NoSsidAvailable,
    ScanCompleted,
    Connected,
    ConnectFailed,
    ConnectionLost,
    Disconnected,
    ApListening,
    ApConnected,
    ApFailed,
}

// Implements both from() and into()
impl From<u8> for WlStatus {
    fn from(state_u8: u8) -> Self {
        if state_u8 == 255 {
            return WlStatus::NoShield;
        } else if state_u8 == 0 {
            return WlStatus::IdleStatus;
        } else if state_u8 == 1 {
            return WlStatus::NoSsidAvailable;
        } else if state_u8 == 2 {
            return WlStatus::ScanCompleted;
        } else if state_u8 == 3 {
            return WlStatus::Connected;
        } else if state_u8 == 4 {
            return WlStatus::ConnectFailed;
        } else if state_u8 == 5 {
            return WlStatus::ConnectionLost;
        } else if state_u8 == 6 {
            return WlStatus::Disconnected;
        } else if state_u8 == 7 {
            return WlStatus::ApListening;
        } else if state_u8 == 8 {
            return WlStatus::ApConnected;
        } else if state_u8 == 9 {
            return WlStatus::ApFailed;
        } else {
            return WlStatus::IdleStatus;
        }
    }
}

type SpiResult<T> = Result<T, SpiDrvError>;

type EnabledUart = hal::uart::UartPeripheral<
    rp2040_hal::uart::Enabled,
    pac::UART0,
    (
        rp2040_hal::gpio::Pin<Gpio0, rp2040_hal::gpio::Function<rp2040_hal::gpio::Uart>>,
        rp2040_hal::gpio::Pin<Gpio1, rp2040_hal::gpio::Function<rp2040_hal::gpio::Uart>>,
    ),
>;

type Params = [u8];

// Until cortex_m implements the DelayUs trait needed for embedded-hal-1.0.0,
// provide a wrapper around it
pub struct DelayWrap(cortex_m::delay::Delay);

impl embedded_hal::delay::blocking::DelayUs for DelayWrap {
    type Error = core::convert::Infallible;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        self.0.delay_us(us);

        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.0.delay_ms(ms);
        Ok(())
    } 
}

struct Esp32Pins {
    cs: Pin<Gpio7, hal::gpio::PushPullOutput>,
    gpio0: Pin<Gpio2, hal::gpio::PushPullOutput>,
    resetn: Pin<Gpio11, hal::gpio::PushPullOutput>,
    ack: Pin<Gpio10, hal::gpio::FloatingInput>,
}

struct SpiDrv {
    spi: hal::Spi<Enabled, pac::SPI0, 8>,
    esp32_pins: Esp32Pins,
}

impl SpiDrv {
    fn new(spi: hal::Spi<Enabled, pac::SPI0, 8>, pins: Esp32Pins) -> SpiDrv {
        SpiDrv {
            spi: spi,
            esp32_pins: pins,
        }
    }

    fn init(&mut self) {
        // Chip select is active-low, so we'll initialise it to a driven-high state
        self.esp32_pins.cs.set_high().unwrap();
    }

    pub fn reset<D: DelayUs>(&mut self, delay: &mut D) {
        self.esp32_pins.gpio0.set_high().unwrap();
        self.esp32_pins.cs.set_high().unwrap();
        self.esp32_pins.resetn.set_low().unwrap();
        delay.delay_ms(10).ok().unwrap();
        self.esp32_pins.resetn.set_high().unwrap();
        delay.delay_ms(750).ok().unwrap();
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

    fn get_param(&mut self, uart: &mut EnabledUart) -> Result<u8, SpiDrvError> {
        // Blocking read, don't return until we've read a byte successfully
        loop {
            let word_out = &mut [DUMMY_DATA];
            // write!(uart, "\t\tsending byte: 0x{:X?} -> ", DUMMY_DATA)
            //     .ok()
            //     .unwrap();
            match self.spi.transfer(word_out) {
                Ok(word) => {
                    let byte: u8 = word[0] as u8;
                    return Ok(byte);
                }
                Err(e) => {
                    continue;
                }
            }
        }
    }

    fn read_byte(&mut self, uart: &mut EnabledUart) -> SpiResult<u8> {
        match self.get_param(uart) {
            Ok(byte_out) => {
                return Ok(byte_out);
            }
            Err(e) => {
                return Err(e);
            }
        }
    }

    fn read_and_check_byte(&mut self, uart: &mut EnabledUart, check_byte: u8) -> SpiResult<bool> {
        match self.get_param(uart) {
            Ok(byte_out) => {
                write!(
                    uart,
                    "\tread_and_check_byte(): 0x{:X?} == 0x{:X?}: {:?}\r\n",
                    byte_out,
                    check_byte,
                    byte_out == check_byte
                )
                .ok()
                .unwrap();
                return Ok(byte_out == check_byte);
            }
            Err(e) => {
                return Err(e);
            }
        }
    }

    fn wait_for_byte(&mut self, uart: &mut EnabledUart, wait_byte: u8) -> SpiResult<bool> {
        let mut timeout: u16 = 1000u16;

        loop {
            match self.read_byte(uart) {
                Ok(byte_read) => {
                    if byte_read == ERR_CMD {
                        return Err(SpiDrvError::CmdResponseError);
                    } else if byte_read == wait_byte {
                        return Ok(true);
                    } else if timeout == 0 {
                        write!(uart, "*** wait_for_byte timed out\r\n")
                            .ok()
                            .unwrap();
                        return Err(SpiDrvError::CmdResponseTimeout);
                    }
                    timeout -= 1;
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }
    }

    fn check_start_cmd(&mut self, uart: &mut EnabledUart) -> SpiResult<bool> {
        match self.wait_for_byte(uart, START_CMD) {
            Ok(b) => {
                return Ok(b);
            }
            Err(e) => {
                return Err(e);
            }
        }
    }

    fn wait_response_cmd(
        &mut self,
        uart: &mut EnabledUart,
        cmd: u8,
        num_param: u8,
    ) -> SpiResult<[u8; PARAMS_ARRAY_LEN]> {
        uart.write_full_blocking(b"wait_response_cmd()\r\n");
        // TODO: can we turn this into more of a functional syntax to clean
        // up the deep nesting? Investigate `map` for `Result` in Rust by Example, or use of Combinators
        self.check_start_cmd(uart)?;
        uart.write_full_blocking(b"\tSuccess: check_start_cmd()\r\n");

        let result = self.read_and_check_byte(uart, cmd | REPLY_FLAG)?;
        // Ensure we see a cmd byte
        if !result {
            return Err(SpiDrvError::CmdResponseCheckFailed(cmd));
        }
        uart.write_full_blocking(b"\tSuccess: read_and_check_byte(cmd | REPLY_FLAG)\r\n");

        let result = self.read_and_check_byte(uart, num_param)?;
        // Ensure we see the number of params we expected to receive back
        if !result {
            return Err(SpiDrvError::CmdResponseCheckFailed(num_param));
        }
        uart.write_full_blocking(b"\tSuccess: read_and_check_byte(num_param)\r\n");

        let num_params_to_read = self.get_param(uart)? as usize;
        write!(uart, "\tnum_param_read: {:?}\r\n", num_params_to_read)
            .ok()
            .unwrap();

        if num_params_to_read > PARAMS_ARRAY_LEN {
            uart.write_full_blocking(b"\tnum_param_read is larger than PARAMS_ARRAY_LEN\r\n");
            return Err(SpiDrvError::CmdResponseInvalidParamNum);
        }

        let mut params: [u8; PARAMS_ARRAY_LEN] = [0; PARAMS_ARRAY_LEN];
        for i in 0..num_params_to_read {
            params[i] = self.get_param(uart).unwrap();
            write!(uart, "\t\tparams[{:?}]: 0x{:X?}\r\n", i, params[i])
                .ok()
                .unwrap();
        }

        self.read_and_check_byte(uart, END_CMD)?;
        uart.write_full_blocking(b"\tSuccess: read_and_check_byte(END_CMD)\r\n");

        Ok(params)
    }

    fn wait_response_data16(
        &mut self,
        uart: &mut EnabledUart,
        cmd: u8,
        num_param: u8,
    ) -> SpiResult<[u8; RESPONSE_BUF_LEN]> {
        uart.write_full_blocking(b"\tStarting wait_response_data16()\r\n");

        let result = self.check_start_cmd(uart)?;
        // Ensure we see a cmd byte
        if !result {
            return Err(SpiDrvError::CmdResponseCheckFailed(cmd));
        }
        uart.write_full_blocking(b"\tSuccess: check_start_cmd()\r\n");

        let result = self.read_and_check_byte(uart, cmd | REPLY_FLAG)?;
        if !result {
            return Err(SpiDrvError::CmdResponseCheckFailed(cmd));
        }
        uart.write_full_blocking(b"\tSuccess: read_and_check_byte(cmd | REPLY_FLAG)\r\n");

        let num_params_to_read = self.read_byte(uart)?;
        write!(uart, "\tnum_params_to_read: {:?}\r\n", num_params_to_read).unwrap();
        let mut param_len: usize = 0;
        if num_params_to_read > 0 {
            param_len = self.read_param_len16(uart)? as usize;
            write!(uart, "\tparam_len: {:?}\r\n", param_len).unwrap();
        }

        if param_len > RESPONSE_BUF_LEN {
            uart.write_full_blocking(b"\tnum_param_read is larger than RESPONSE_BUF_LEN \r\n");
            return Err(SpiDrvError::CmdResponseInvalidParamNum);
        }

        let mut params: [u8; RESPONSE_BUF_LEN] = [0; RESPONSE_BUF_LEN];
        for i in 0..param_len as usize {
            params[i] = self.get_param(uart).unwrap();
        }

        let result = self.read_and_check_byte(uart, END_CMD)?;
        // Ensure we see an end byte
        if !result {
            return Err(SpiDrvError::CmdResponseCheckFailed(END_CMD));
        }
        uart.write_full_blocking(b"\tSuccess: read_and_check_byte(END_CMD)\r\n");

        Ok(params)
    }

    fn send_cmd(&mut self, uart: &mut EnabledUart, cmd: u8, num_param: u8) -> SpiResult<()> {
        let buf: [u8; 3] = [START_CMD, cmd & !(REPLY_FLAG), num_param];
        for byte in buf {
            let byte_buf = &mut [byte];
            write!(uart, "\t\tsending byte: 0x{:X?} -> ", byte).unwrap();
            match self.spi.transfer(byte_buf) {
                Ok(byte) => {
                    write!(uart, "read byte: 0x{:X?}\r\n", byte).ok().unwrap();
                }
                Err(e) => {
                    write!(uart, "send_cmd transfer error: 0x{:X?}\r\n", e).unwrap();
                }
            }
        }

        if num_param == 0 {
            match self.send_end_cmd(uart) {
                Ok(byte) => {
                    return Ok(());
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }
        Ok(())
    }

    // Sends END_CMD byte to ESP32 firmware over SPI bus
    fn send_end_cmd(&mut self, uart: &mut EnabledUart) -> SpiResult<()> {
        let end_command = &mut [END_CMD];
        write!(uart, "\t\tsending byte (END_CMD): 0x{:X?} -> ", end_command)
            .ok()
            .unwrap();
        match self.spi.transfer(end_command) {
            Ok(byte) => {
                write!(uart, "read byte: 0x{:X?}\r\n", byte).ok().unwrap();
                return Ok(());
            }
            Err(e) => {
                write!(uart, "\t\t\tsend_param transfer error: {:?}\r\n", e)
                    .ok()
                    .unwrap();
                return Err(SpiDrvError::TransferFailed);
            }
        }
    }

    // Accepts two separate bytes and packs them into 2 combined bytes as a u16
    // byte 0 is the LSB, byte1 is the MSB
    // See: https://en.wikipedia.org/wiki/Bit_numbering#LSB_0_bit_numbering
    fn combine_2_bytes(&mut self, byte0: u8, byte1: u8) -> u16 {
        let word0: u16 = byte0 as u16;
        let word1: u16 = byte1 as u16;
        (word1 << 8) | (word0 & 0xff)
    }

    fn read_param_len16(&mut self, uart: &mut EnabledUart) -> SpiResult<u16> {
        let buf: [u8; 2] = [
            self.read_byte(uart).ok().unwrap(),
            self.read_byte(uart).ok().unwrap(),
        ];
        uart.write_full_blocking(b"\tread_param_len16()\r\n");

        let param_length: u16 = self.combine_2_bytes(buf[1], buf[0]);
        write!(
            uart,
            "\t\tread_param_len_16 length =  {:?}\r\n",
            param_length
        )
        .unwrap();
        Ok(param_length)
    }

    fn send_param_len8(&mut self, uart: &mut EnabledUart, param_len: u8) -> SpiResult<()> {
        match self.spi.transfer(&mut [param_len]) {
            Ok(byte) => {
                write!(uart, "\t\tread byte: 0x{:X?}\r\n", byte).unwrap();
                return Ok(());
            }
            Err(e) => {
                return Err(SpiDrvError::TransferFailed);
            }
        }
    }

    // https://en.wikipedia.org/wiki/Bit_numbering#LSB_0_bit_numbering
    fn send_param_len16(&mut self, uart: &mut EnabledUart, param_len: u16) -> SpiResult<()> {
        let byte_buf: &mut [u8; 2] =
            &mut [((param_len & 0xff00) >> 8) as u8, (param_len & 0xff) as u8];
        write!(
            uart,
            "\t\tsending byte_buf [0x{:X?}, 0x{:X?}] (len16)\r\n",
            byte_buf[0], byte_buf[1]
        )
        .ok()
        .unwrap();
        match self.spi.transfer(byte_buf) {
            Ok(byte) => {
                write!(uart, "\t\tread byte: 0x{:X?}\r\n", byte).unwrap();
                return Ok(());
            }
            Err(e) => {
                return Err(SpiDrvError::TransferFailed);
            }
        }
    }

    fn send_buffer(
        &mut self,
        uart: &mut EnabledUart,
        buffer: &mut [u8],
        last_param: bool,
    ) -> SpiResult<()> {
        write!(uart, "\t\tbuffer.len(): {:?}\r\n", buffer.len())
            .ok()
            .unwrap();
        self.send_param_len16(uart, buffer.len() as u16)?;

        match self.spi.transfer(buffer) {
            Ok(transfer_buf) => {
                write!(
                    uart,
                    "\t\tread bytes: send_buffer() 0x{:X?}\r\n",
                    transfer_buf
                )
                .unwrap();
                if last_param {
                    match self.send_end_cmd(uart) {
                        Ok(byte) => {
                            return Ok(());
                        }
                        Err(e) => {
                            return Err(SpiDrvError::TransferFailed);
                        }
                    }
                } else {
                    return Ok(());
                }
            }
            Err(e) => {
                write!(uart, "send_buffer transfer error: 0x{:X?}\r\n", e).unwrap();
                return Err(SpiDrvError::TransferFailed);
            }
        }
    }

    // TODO: replace last_param with an enumerated type, e.g. NO_LAST_PARAM, LAST_PARAM
    fn send_param(
        &mut self,
        uart: &mut EnabledUart,
        params: &mut Params,
        last_param: bool,
    ) -> SpiResult<()> {
        let param_len: u8 = params.len() as u8;

        self.send_param_len8(uart, param_len)?;
        match self.spi.transfer(params) {
            Ok(transfer_buf) => {
                write!(uart, "\t\tread bytes: 0x{:X?}\r\n", transfer_buf).unwrap();
                if last_param {
                    match self.send_end_cmd(uart) {
                        Ok(byte) => {
                            write!(uart, "\t\tread byte: 0x{:X?}\r\n", byte).unwrap();
                            return Ok(());
                        }
                        Err(e) => {
                            write!(uart, "\t\t\tsend_param transfer error: 0x{:X?}\r\n", e)
                                .unwrap();
                            return Err(e);
                        }
                    }
                } else {
                    return Ok(());
                }
            }
            Err(e) => {
                return Err(SpiDrvError::TransferFailed);
            }
        }
    }

    fn send_param_word_len16(
        &mut self,
        uart: &mut EnabledUart,
        param: u16,
        last_param: bool,
    ) -> SpiResult<()> {
        match self.send_param_len16(uart, 2) {
            Ok(_) => {
                let byte_buf: &mut [u8; 2] =
                    &mut [((param & 0xff00) >> 8) as u8, (param & 0xff) as u8];
                let transfer_results = self.spi.transfer(byte_buf);
                match transfer_results {
                    Ok(byte) => {
                        write!(uart, "\t\tread byte: 0x{:X?}\r\n", byte)
                            .ok()
                            .unwrap();
                        if last_param {
                            match self.send_end_cmd(uart) {
                                Ok(byte) => {
                                    return Ok(());
                                }
                                Err(e) => {
                                    return Err(e);
                                }
                            }
                        } else {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        return Err(SpiDrvError::TransferFailed);
                    }
                }
            }
            Err(e) => {
                return Err(e);
            }
        }
    }

    fn send_param_word(
        &mut self,
        uart: &mut EnabledUart,
        param: u16,
        last_param: bool,
    ) -> SpiResult<()> {
        let res = self.send_param_len8(uart, 2);
        match res {
            Ok(_) => {
                let byte_buf: &mut [u8; 2] =
                    &mut [((param & 0xff00) >> 8) as u8, (param & 0xff) as u8];
                let transfer_results = self.spi.transfer(byte_buf);
                match transfer_results {
                    Ok(byte) => {
                        write!(uart, "\t\tread byte: 0x{:X?}\r\n", byte)
                            .ok()
                            .unwrap();
                        if last_param {
                            match self.send_end_cmd(uart) {
                                Ok(byte) => {
                                    write!(uart, "\t\tread byte: 0x{:X?}\r\n", byte)
                                        .ok()
                                        .unwrap();
                                    return Ok(());
                                }
                                Err(e) => {
                                    write!(uart, "\t\t\tsend_param transfer error: 0x{:X?}\r\n", e)
                                        .ok()
                                        .unwrap();
                                    return Err(e);
                                }
                            }
                        } else {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        return Err(SpiDrvError::TransferFailed);
                    }
                }
            }
            Err(e) => {
                return Err(e);
            }
        }
    }

    fn pad_to_multiple_of_4(&mut self, uart: &mut EnabledUart, mut command_size: u16) {
        while command_size % 4 == 0 {
            self.read_byte(uart).ok().unwrap();
            command_size += 1;
        }
    }
}

fn set_led(spi_drv: &mut SpiDrv, uart: &mut EnabledUart, red: u8, green: u8, blue: u8) {
    write!(uart, "Calling analog_write(ESP_LED_R, {:?})\r\n", 255 - red)
        .ok()
        .unwrap();
    analog_write(spi_drv, uart, ESP_LED_R, 255 - red);

    write!(
        uart,
        "Calling analog_write(ESP_LED_G, {:?})\r\n",
        255 - green
    )
    .ok()
    .unwrap();
    analog_write(spi_drv, uart, ESP_LED_G, 255 - green);

    write!(
        uart,
        "Calling analog_write(ESP_LED_B, {:?})\r\n",
        255 - blue
    )
    .ok()
    .unwrap();
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
            writeln!(uart, "\twait_response_cmd(SET_ANALOG_WRITE) Err: {:?}\r", e)
                .ok()
                .unwrap();
        }
    }

    uart.write_full_blocking(b"\tesp_deselect()\r\n");
    spi_drv.esp_deselect();
}

fn wifi_set_passphrase(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    mut ssid: String<STR_LEN>,
    mut passphrase: String<STR_LEN>,
) -> bool {
    uart.write_full_blocking(b"wifi_set_passphrase()\r\n");
    spi_drv.wait_for_esp_select();

    spi_drv.send_cmd(uart, SET_PASSPHRASE, 2).ok().unwrap();

    // FIXME: for the real crate, don't use unsafe
    let ssid_bytes: &mut [u8] = unsafe { ssid.as_bytes_mut() };
    writeln!(uart, "\tssid: {:?}\r", ssid_bytes).ok().unwrap();
    spi_drv.send_param(uart, ssid_bytes, false).ok().unwrap();

    // FIXME: for the real crate, don't use unsafe
    let passphrase_bytes: &mut [u8] = unsafe { passphrase.as_bytes_mut() };
    writeln!(uart, "\tpassphrase: {:?}\r", passphrase_bytes)
        .ok()
        .unwrap();
    spi_drv
        .send_param(uart, passphrase_bytes, true)
        .ok()
        .unwrap();

    let command_size: u8 = 6 + ssid.len() as u8 + passphrase.len() as u8;
    spi_drv.pad_to_multiple_of_4(uart, command_size as u16);

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    // Wait for reply
    let wait_response = spi_drv.wait_response_cmd(uart, SET_PASSPHRASE, 1);
    match wait_response {
        Ok(params) => {
            write!(uart, "\twifi_set_passphrase_response: ")
                .ok()
                .unwrap();
            for byte in params {
                let c = byte as char;
                write!(uart, "{:?}", c).ok().unwrap();
            }
            writeln!(uart, "\r\n").ok().unwrap();
        }
        Err(e) => {
            write!(uart, "\twifi_set_passphrase_response Err: {:?}\r\n", e)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            return false;
        }
    }

    spi_drv.esp_deselect();

    true
}

fn get_connection_status(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
) -> Result<WlStatus, String<STR_LEN>> {
    spi_drv.wait_for_esp_select();

    spi_drv.send_cmd(uart, GET_CONN_STATUS, 0).ok().unwrap();

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    // Wait for reply
    match spi_drv.wait_response_cmd(uart, GET_CONN_STATUS, 1) {
        Ok(params) => {
            let status: WlStatus = params[0].into();
            write!(uart, "\tget_connection_status_response: {:?}\r\n", status)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();

            return Ok(status);
        }
        Err(e) => {
            write!(uart, "\tget_connection_status_response Err: {:?}\r\n", e)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            return Err(String::from("Failed to get connection status response."));
        }
    }
}

#[allow(dead_code)]
fn get_fw_version(spi_drv: &mut SpiDrv, uart: &mut EnabledUart) -> bool {
    uart.write_full_blocking(b"wait_for_esp_select()\r\n");
    spi_drv.wait_for_esp_select();
    uart.write_full_blocking(b"\tesp selected\r\n");

    uart.write_full_blocking(b"send_cmd(GET_FW_VERSION)\r\n");
    let results = spi_drv.send_cmd(uart, GET_FW_VERSION, 0);
    match results {
        Ok(_) => {
            uart.write_full_blocking(b"\tsent GET_FW_VERSION command\r\n");
        }
        Err(e) => {
            writeln!(
                uart,
                "\t** Failed to send GET_FW_VERSION command: {:?}\r\n",
                e
            )
            .ok()
            .unwrap();
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

fn send_data(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    socket: u8,
    mut data: String<STR_LEN>,
) -> Result<[u8; PARAMS_ARRAY_LEN], String<STR_LEN>> {
    spi_drv.wait_for_esp_select();

    spi_drv.send_cmd(uart, SEND_DATA_TCP, 2).ok().unwrap();
    spi_drv
        .send_buffer(uart, &mut [socket], false)
        .ok()
        .unwrap();

    let data_bytes: &mut [u8] = unsafe { data.as_bytes_mut() };
    write!(uart, "\tHTTP request bytes: {:?}\r\n", data_bytes).unwrap();
    spi_drv.send_buffer(uart, data_bytes, true).ok().unwrap();

    // this could be a usize
    let command_size = (9 + data.len()) as u16;
    spi_drv.pad_to_multiple_of_4(uart, command_size);

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    match spi_drv.wait_response_cmd(uart, SEND_DATA_TCP, 1) {
        Ok(params) => {
            spi_drv.esp_deselect();
            return Ok(params);
        }
        Err(e) => {
            writeln!(uart, "\twait_response_cmd(SEND_DATA_TCP) Err: {:?}\r", e)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            return Err(String::from("Error wait_response_cmd(SEND_DATA_TCP)"));
        }
    }
}

fn get_socket(spi_drv: &mut SpiDrv, uart: &mut EnabledUart) -> SpiResult<u8> {
    spi_drv.wait_for_esp_select();

    match spi_drv.send_cmd(uart, GET_SOCKET, 0) {
        Ok(_) => {
            uart.write_full_blocking(b"\tSent GET_SOCKET command\r\n");
        }
        Err(e) => {
            writeln!(uart, "\t** Failed to send GET_SOCKET command: {:?}\r\n", e)
                .ok()
                .unwrap();
        }
    }

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    match spi_drv.wait_response_cmd(uart, GET_SOCKET, 1) {
        Ok(params) => {
            write!(uart, "\tget_socket: {:?}\r\n", params[0])
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            return Ok(params[0]);
        }
        Err(e) => {
            writeln!(uart, "\twait_response_cmd(GET_SOCKET) Err: {:?}\r", e)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            return Err(e);
        }
    }
}

fn start_client(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    host_address_port: SocketAddrV4,
    socket: u8,
    transport_mode: SvProtocolMode,
) -> Result<bool, SpiDrvError> {
    spi_drv.wait_for_esp_select();
    spi_drv.send_cmd(uart, START_CLIENT_TCP, 4)?;

    write!(
        uart,
        "\tSending host IP: {:?}\r\n",
        &mut host_address_port.ip()
    )
    .ok()
    .unwrap();
    spi_drv
        .send_param(uart, &mut host_address_port.ip().octets(), false)
        .ok()
        .unwrap();

    write!(
        uart,
        "\tSending host port: {:?}\r\n",
        host_address_port.port()
    )
    .ok()
    .unwrap();
    spi_drv
        .send_param_word(uart, host_address_port.port(), false)
        .ok()
        .unwrap();

    write!(uart, "\tSending socket: {:?}\r\n", socket)
        .ok()
        .unwrap();
    spi_drv.send_param(uart, &mut [socket], false).ok().unwrap();

    write!(uart, "\tSending transport_mode: {:?}\r\n", transport_mode)
        .ok()
        .unwrap();
    spi_drv
        .send_param(uart, &mut [transport_mode as u8], true)
        .ok()
        .unwrap();

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    // Wait for reply
    match spi_drv.wait_response_cmd(uart, START_CLIENT_TCP, 1) {
        Ok(params) => {
            write!(uart, "\tstart_client response param: {:?}\r\n", params[0])
                .ok()
                .unwrap();
            spi_drv.esp_deselect();

            return Ok(params[0] == 1);
        }
        Err(e) => {
            write!(
                uart,
                "\twait_response_cmd(START_CLIENT_TCP) Err: {:?}\r\n",
                e
            )
            .ok()
            .unwrap();
            spi_drv.esp_deselect();
            return Err(SpiDrvError::ServerCommTimeout);
        }
    }
}

fn get_client_state(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    socket: u8,
) -> Result<WlTcpState, SpiDrvError> {
    spi_drv.wait_for_esp_select();

    spi_drv
        .send_cmd(uart, GET_CLIENT_STATE_TCP, 1)
        .ok()
        .unwrap();
    spi_drv.send_param(uart, &mut [socket], true).ok().unwrap();

    spi_drv.read_byte(uart).ok().unwrap();
    spi_drv.read_byte(uart).ok().unwrap();

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    // Wait for reply
    match spi_drv.wait_response_cmd(uart, GET_CLIENT_STATE_TCP, 1) {
        Ok(params) => {
            let state: WlTcpState = params[0].into();
            writeln!(uart, "\tGET_CLIENT_STATE state {:?}\r", state)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            return Ok(state);
        }
        Err(e) => {
            writeln!(uart, "\tGET_CLIENT_STATE response Err: {:?}\r", e)
                .ok()
                .unwrap();

            spi_drv.esp_deselect();
            return Err(SpiDrvError::ServerCommTimeout);
        }
    }
}

fn connect<D: DelayUs>(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    delay: &mut D,
    host_address_port: SocketAddrV4,
    socket: u8,
) -> Result<bool, SpiDrvError> {
    let mut timeout: u16 = 10000;
    while timeout > 0 {
        match start_client(
            spi_drv,
            uart,
            host_address_port,
            socket,
            SvProtocolMode::TCP,
        ) {
            Ok(started) => match started {
                true => break,
                false => return Ok(false),
            },
            Err(SpiDrvError::ServerCommTimeout) => {
                write!(
                    uart,
                    "ServerCommTimeout for start_client(), retry #{:?}\r\n",
                    timeout
                )
                .ok()
                .unwrap();
                delay.delay_ms(1000).ok().unwrap();
                timeout -= 1;
            }
            Err(e) => {
                return Err(e);
            }
        }
    }

    timeout = 10000;
    while timeout > 0 {
        match get_client_state(spi_drv, uart, socket) {
            Ok(state) => {
                if state == WlTcpState::Established {
                    return Ok(true);
                }
            }
            Err(e) => {
                return Err(e);
            }
        }
        delay.delay_ms(10).ok().unwrap();
        timeout -= 1;
    }

    return Ok(false);
}

fn stop_client(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    socket: u8,
) -> Result<bool, SpiDrvError> {
    spi_drv.wait_for_esp_select();

    spi_drv.send_cmd(uart, STOP_CLIENT_TCP, 1).ok().unwrap();
    spi_drv.send_param(uart, &mut [socket], true).ok().unwrap();

    // Pad to a multiple of 4
    spi_drv.read_byte(uart).unwrap();
    spi_drv.read_byte(uart).unwrap();

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    match spi_drv.wait_response_cmd(uart, STOP_CLIENT_TCP, 1) {
        Ok(params) => {
            write!(uart, "\tstop_client: {:?}\r\n", params[0])
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            return Ok(params[0] == 1);
        }
        Err(e) => {
            writeln!(uart, "\twait_response_cmd(STOP_CLIENT_TCP) Err: {:?}\r", e)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();
            return Err(SpiDrvError::ServerCommTimeout);
        }
    }
}

fn avail_data(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    socket: u8,
) -> Result<usize, String<STR_LEN>> {
    spi_drv.wait_for_esp_select();

    spi_drv.send_cmd(uart, AVAIL_DATA_TCP, 1).ok().unwrap();
    let socket_bytes: &mut [u8] = &mut [socket];
    spi_drv.send_param(uart, socket_bytes, true).ok().unwrap();

    spi_drv.read_byte(uart).ok().unwrap();
    spi_drv.read_byte(uart).ok().unwrap();

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    match spi_drv.wait_response_cmd(uart, AVAIL_DATA_TCP, 1) {
        Ok(params) => {
            writeln!(uart, "\tavail_data() {:?}\r", params)
                .ok()
                .unwrap();
            spi_drv.esp_deselect();

            // Combine the two separate u8's into a single u16 len
            let combined_len: usize = spi_drv.combine_2_bytes(params[0], params[1]) as usize;
            write!(uart, "\tcombined_len: {:?}\r\n", combined_len)
                .ok()
                .unwrap();
            return Ok(combined_len);
        }
        Err(e) => {
            writeln!(uart, "\tavail_data Err: {:?}\r", e).ok().unwrap();

            spi_drv.esp_deselect();

            return Err(String::from("Failed to get get avail_data"));
        }
    }
}

fn get_data_buf(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    socket: u8,
    avail_len: u16,
) -> Result<[u8; RESPONSE_BUF_LEN], String<STR_LEN>> {
    spi_drv.wait_for_esp_select();

    spi_drv.send_cmd(uart, GET_DATABUF_TCP, 2).ok().unwrap();
    spi_drv
        .send_buffer(uart, &mut [socket], false)
        .ok()
        .unwrap();
    spi_drv
        .send_param_word_len16(uart, avail_len, true)
        .ok()
        .unwrap();

    spi_drv.read_byte(uart).unwrap();

    spi_drv.esp_deselect();
    spi_drv.wait_for_esp_select();

    match spi_drv.wait_response_data16(uart, GET_DATABUF_TCP, 1) {
        Ok(buf) => {
            spi_drv.esp_deselect();
            return Ok(buf);
        }
        Err(e) => {
            writeln!(
                uart,
                "\twait_response_data16(GET_DATABUF_TCP) Err: {:?}\r",
                e
            )
            .ok()
            .unwrap();
            spi_drv.esp_deselect();
            return Err(String::from("Error wait_response_data16(GET_DATABUF_TCP)"));
        }
    }
}

fn get_server_response<D: DelayUs>(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    delay: &mut D,
    socket: u8,
) -> Result<httparse::Status<usize>, String<STR_LEN>> {
    let response_length: u16 = 0;
    let mut avail_length: usize = 0;
    let n = 0;
    let response_buf: [u8; RESPONSE_BUF_LEN] = [0; RESPONSE_BUF_LEN];
    let mut timeout: u16 = 1000;

    while timeout > 0 {
        delay.delay_ms(50).ok().unwrap();
        avail_length = avail_data(spi_drv, uart, socket)?;
        if avail_length > 0 {
            break;
        }

        timeout -= 1;
    }

    write!(
        uart,
        "\tresponse_length: {:?} avail_length {:?}\r\n",
        response_length, avail_length
    )
    .ok()
    .unwrap();

    let response_buf = get_data_buf(spi_drv, uart, socket, avail_length as u16)?;
    let response_str = core::str::from_utf8(&response_buf).unwrap();

    writeln!(
        uart,
        "\tHTTP server response string: {:?}\r\n",
        response_str
    )
    .ok()
    .unwrap();

    let mut headers = [httparse::EMPTY_HEADER; 64];
    let mut response = httparse::Response::new(&mut headers);
    match response.parse(&response_buf) {
        Ok(parsed) => {
            write!(
                uart,
                "HTTP response version: {:?}\r\n",
                response.version.unwrap()
            )
            .ok()
            .unwrap();
            write!(uart, "HTTP response code: {:?}\r\n", response.code.unwrap())
                .ok()
                .unwrap();
            writeln!(
                uart,
                "HTTP response reason: {:?}\r\n",
                response.reason.unwrap()
            )
            .ok()
            .unwrap();

            if response.code.unwrap() == 200 {
                write!(uart, "Got successful response from HTTP server.\r\n")
                    .ok()
                    .unwrap();
            } else if response.code.unwrap() == 400 {
                write!(uart, "** Got error response from HTTP server.\r\n")
                    .ok()
                    .unwrap();
            }
            return Ok(parsed);
        }

        Err(e) => {
            write!(uart, "Failed to parse HTTP server response: {:?}\r\n", e)
                .ok()
                .unwrap();
            return Err(String::from("Failed to parse HTTP server response"));
        }
    }
}

fn http_request<D: DelayUs>(
    spi_drv: &mut SpiDrv,
    uart: &mut EnabledUart,
    delay: &mut D,
    socket: u8,
    host_address_port: SocketAddrV4,
    request_path: String<STR_LEN>,
    temperature: f32,
    humidity: f32,
    pressure: f32,
) -> Result<bool, String<STR_LEN>> {
    match connect(spi_drv, uart, delay, host_address_port, socket) {
        Ok(connected) => {
            if connected {
                uart.write_full_blocking(b"Successfully connected to remote TCP server.\r\n");

                let mut http_post_request: String<STR_LEN> = String::from("POST ");
                http_post_request.push_str(&request_path).ok().unwrap();
                http_post_request
                    .push_str(" HTTP/1.1\r\nHost: ")
                    .ok()
                    .unwrap();
                let mut host_address_str: String<STR_LEN> = String::new();
                write!(
                    host_address_str,
                    "{}.{}.{}.{}:{:?}\r\n",
                    host_address_port.ip().octets()[0],
                    host_address_port.ip().octets()[1],
                    host_address_port.ip().octets()[2],
                    host_address_port.ip().octets()[3],
                    host_address_port.port()
                )
                .unwrap();
                http_post_request.push_str(&host_address_str).ok().unwrap();
                http_post_request
                    .push_str("User-Agent: edge/0.0.1\r\n")
                    .ok()
                    .unwrap();
                http_post_request.push_str("Accept: */*\r\n").ok().unwrap();
                http_post_request
                    .push_str("Content-Type: application/json\r\n")
                    .ok()
                    .unwrap();
                let mut json_str: String<STR_LEN> = String::new();
                write!(json_str,
                    "{{\"temperature\":\"{:.1?}\",\"humidity\":\"{:.1?}\",\"pressure\":\"{:.0?}\",\"dust_concentration\":\"200\",\"air_purity\":\"Low Pollution\"}}\r\n",
                    temperature, humidity, pressure / 100.0
                ).ok().unwrap();
                let mut content_len_str: String<STR_LEN> = String::new();
                write!(content_len_str, "{:?}\r\n", json_str.len())
                    .ok()
                    .unwrap();
                http_post_request.push_str("Content-Length: ").ok().unwrap();
                http_post_request.push_str(&content_len_str).ok().unwrap();
                http_post_request.push_str("\r\n").ok().unwrap();
                http_post_request.push_str(&json_str).ok().unwrap();
                http_post_request.push_str("\r\n").ok().unwrap();
                writeln!(uart, "\thttp_post_request: {:?}\r\n", http_post_request)
                    .ok()
                    .unwrap();

                let response = send_data(spi_drv, uart, socket, http_post_request);
                match response {
                    Ok(data) => {
                        writeln!(
                            uart,
                            "\tsend_data() response data.len(): {:?}\r\n",
                            data.len()
                        )
                        .ok()
                        .unwrap();
                        return Ok(connected);
                    }
                    Err(e) => {
                        return Err(e);
                    }
                }
            } else {
                return Err(String::from("Failed to connect() to remote host"));
            }
        }
        Err(e) => {
            return Err(String::from("Failed to connect() to remote host"));
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

    let mut delay = DelayWrap(cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer()));

    defmt::info!("\r\nESP32 Wifi PoC (pre-crate)");

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio26.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio27.into_mode::<hal::gpio::FunctionI2C>();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    // Initialise the BME280 using the secondary I2C address 0x77
    let mut bme280 = BME280::new_secondary(i2c);

    let mut bme280_success = false;
    // Initialise the sensor
    let res = bme280.init(&mut delay);
    match res {
        Ok(_) => {
            defmt::debug!("Successfully initialized BME280 device");
            bme280_success = true;
        }
        Err(_) => defmt::error!("Failed to initialize BME280 device"),
    }

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
        &MODE_0,
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
    delay.delay_ms(500).ok().unwrap();

    // Set wifi passphrase - ESP32 will attempt to connect after receving this cmd
    wifi_set_passphrase(
        &mut spi_drv,
        &mut uart,
        String::from(SSID),
        String::from(PASSPHRASE),
    );
    delay.delay_ms(1000).ok().unwrap();

    let led_pin = pins.gpio25.into_push_pull_output();

    let socket = get_socket(&mut spi_drv, &mut uart).ok().unwrap();

    let mut i: u32 = 0;
    let mut sleep: u32 = 1500;
    let mut did_once = false; // Only send HTTP POST one time
    loop {
        // Check for connection in loop and set led on if connected succesfully
        match get_connection_status(&mut spi_drv, &mut uart) {
            Ok(status) => {
                if status == WlStatus::Connected && !did_once {
                    uart.write_full_blocking(b"** Connected to WiFi\r\n");

                    // Set ESP32 LED green when successfully connected to WiFi AP
                    set_led(&mut spi_drv, &mut uart, 0, 255, 0);

                    did_once = true;
                } else if status == WlStatus::Connected && did_once {
                    let host_address_port = SocketAddrV4::new(
                        Ipv4Addr::new(
                            NON_PROD_AMBI_IP[0],
                            NON_PROD_AMBI_IP[1],
                            NON_PROD_AMBI_IP[2],
                            NON_PROD_AMBI_IP[3],
                        ),
                        NON_PROD_AMBI_PORT,
                    );

                    let request_path = String::from("/api/readings/add");

                    writeln!(
                        uart,
                        "\r\nMaking HTTP request to: http://{:?}{:?}\r\n",
                        host_address_port, request_path
                    )
                    .ok()
                    .unwrap();

                    let t;
                    let p;
                    let h;
                    match bme280_success {
                        true => {
                            // Reads live ambient values from the BME280 sensor
                            let measurements = bme280.measure(&mut delay).unwrap();
                            t = measurements.temperature;
                            p = measurements.pressure;
                            h = measurements.humidity;
                        }
                        false => {
                            t = 23.0;
                            p = 980.0;
                            h = 32.0;
                        }
                    }

                    http_request(
                        &mut spi_drv,
                        &mut uart,
                        &mut delay,
                        socket,
                        host_address_port,
                        request_path,
                        t,
                        h,
                        p,
                    )
                    .ok()
                    .unwrap();

                    uart.write_full_blocking(
                        b"Getting server response...\r\n---------------------------\r\n",
                    );
                    match get_server_response(&mut spi_drv, &mut uart, &mut delay, socket) {
                        Ok(status) => {
                            writeln!(uart, "Successful HTTP response: {:?}\r\n", status)
                                .ok()
                                .unwrap();
                        }
                        Err(e) => {
                            writeln!(uart, "** HTTP response error: {:?}\r\n", e)
                                .ok()
                                .unwrap();
                        }
                    }

                    // It's important to stop the existing client before trying to start the client again,
                    // otherwise expect Undefined behavior
                    let stopped = stop_client(&mut spi_drv, &mut uart, socket).ok().unwrap();
                    if !stopped {
                        writeln!(uart, "** Failed to stop ESP32 TCP client.")
                            .ok()
                            .unwrap();
                    }

                    // Sleep 10s in between sending sensor readings to Ambi backend
                    sleep = 10000;
                } else if status != WlStatus::Connected {
                    writeln!(uart, "** Not connected to WiFi").ok().unwrap();
                    // Set ESP32 LED green when successfully connected to WiFi AP
                    set_led(&mut spi_drv, &mut uart, 255, 0, 0);

                    // Until we're connected to WiFi and sending sensor readings, sleep 1.5s
                    sleep = 1500;
                }
            }
            Err(e) => {
                uart.write_full_blocking(b"** Failed to get WiFi connection status\r\n");
            }
        }

        write!(uart, "Loop ({:?}) ...\r", i).ok().unwrap();

        delay.delay_ms(sleep).ok().unwrap();
        i += 1;
    }
}

// End of file
