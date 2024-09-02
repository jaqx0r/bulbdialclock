/*
BulbdialClock.ino

Default software for the Bulbdial Clock kit designed by
Evil Mad Scientist Laboratories: http://www.evilmadscientist.com/go/bulbdialkit

Updated to work with Arduino 1.0 by Ray Ramirez
Also requires Time library:  http://www.arduino.cc/playground/Code/Time

Target: ATmega168, clock at 16 MHz.

Version 1.0.1 - 1/14/2012
Copyright (c) 2009 Windell H. Oskay.  All right reserved.
http://www.evilmadscientist.com/

This library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this library.  If not, see <http://www.gnu.org/licenses/>.

*/

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod timer;
use crate::timer::*;
use arduino_hal::{hal::port, prelude::*};
use core::cmp::Ordering;

#[cfg(not(feature = "panic-serial"))]
use panic_halt as _;
#[cfg(feature = "panic-serial")]
panic_serial::impl_panic_handler!(
    arduino_hal::usart::UsartWriter<
    arduino_hal::pac::USART0,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Input, arduino_hal::hal::port::PD0>,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Output, arduino_hal::hal::port::PD1>,
    >
);

/// EEPROM variables that are saved:  7
struct Settings {
    /// Brightness setting (range: 1-8)  Default: 8   (Fully bright)
    main_bright: u8,

    /// Red brightness  (range: 0-63)    Default: 20 (But initialized to 30 in original!  TODO add feature for monocrhome)
    hr_bright: u8,

    /// Green brightness (range: 0-63)   Default: 63
    min_bright: u8,

    /// Blue brightness (range: 0-63)    Default: 63
    sec_bright: u8,

    /// Time direction (Range: 0,1)      Default: 0  (Clockwise)
    ccw: bool,

    /// Fade style (Range: 0,1)         Default: 1  (Fade enabled)
    fade_mode: bool,

    /// Alignment mode                  Default: 0
    // bool; never used

    // Last brightness saved to EEPROM used to determine if changed.
    last_saved_brightness: u8,
}

// "Factory" default configuration can be configured here:
const MAIN_BRIGHT_DEFAULT: u8 = 8;

const RED_BRIGHT_DEFAULT: u8 = 63; // Use 63, default, for kits with monochrome LEDs!
const GREEN_BRIGHT_DEFAULT: u8 = 63;
const BLUE_BRIGHT_DEFAULT: u8 = 63;

const CCW_DEFAULT: bool = false;
const FADE_MODE_DEFAULT: bool = true;

const TIME_MSG_LEN: usize = 11; // time sync to PC is HEADER followed by unix time_t as ten ascii digits
const TIME_HEADER: u8 = 255; // Header tag for serial time sync message

const TEMP_FADE: u8 = 63;

impl Default for Settings {
    fn default() -> Self {
        Self {
            main_bright: MAIN_BRIGHT_DEFAULT,
            hr_bright: RED_BRIGHT_DEFAULT,
            min_bright: GREEN_BRIGHT_DEFAULT,
            sec_bright: BLUE_BRIGHT_DEFAULT,
            ccw: CCW_DEFAULT,
            fade_mode: FADE_MODE_DEFAULT,
            last_saved_brightness: MAIN_BRIGHT_DEFAULT,
        }
    }
}

fn delay_time(time: u8) {
    for _ in 0..time {
        avr_device::asm::nop();
    }
}

type SerialReader = arduino_hal::usart::UsartReader<
    arduino_hal::pac::USART0,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Input, arduino_hal::hal::port::PD0>,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Output, arduino_hal::hal::port::PD1>,
>;

/// Try to read a time from the serial port, returning either a parsed time or nothing.
fn get_pc_time(s_rx: &mut SerialReader) -> Option<(u8, u8, u8)> {
    // if time sync available from serial port, update time and return true
    match s_rx.read() {
        Ok(TIME_HEADER) => {
            // read unix time from serial port, header byte and ten ascii digits
            let mut pctime: u32 = 0;
            for _ in 0..TIME_MSG_LEN {
                match s_rx.read() {
                    Ok(c) => {
                        if let Some(d) = char::from(c).to_digit(10) {
                            pctime = (10 * pctime) + d;
                        }
                    }
                    _ => return None,
                }
            }
            let sec_now: u8 = match (pctime % 60).try_into() {
                Ok(v) => v,
                Err(_) => return None,
            };
            let min_now: u8 = match ((pctime / 60) % 60).try_into() {
                Ok(v) => v,
                Err(_) => return None,
            };
            let hr_now: u8 = match ((pctime / 60 / 60) % 24).try_into() {
                Ok(v) => v,
                Err(_) => return None,
            };
            Some((hr_now, min_now, sec_now))
        }
        _ => None,
    }
}

const SEC_HI: [u8; 30] = [
    2, 3, 4, 5, 6, 1, 3, 4, 5, 6, 1, 2, 4, 5, 6, 1, 2, 3, 5, 6, 1, 2, 3, 4, 6, 1, 2, 3, 4, 5,
];
const SEC_LO: [u8; 30] = [
    1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
];

const MIN_HI: [u8; 30] = [
    1, 7, 1, 8, 1, 9, 2, 7, 2, 8, 2, 9, 3, 7, 3, 8, 3, 9, 4, 7, 4, 8, 4, 9, 5, 7, 5, 8, 5, 9,
];
const MIN_LO: [u8; 30] = [
    7, 1, 8, 1, 9, 1, 7, 2, 8, 2, 9, 2, 7, 3, 8, 3, 9, 3, 7, 4, 8, 4, 9, 4, 7, 5, 8, 5, 9, 5,
];

const HR_HI: [u8; 12] = [10, 1, 2, 10, 10, 6, 3, 10, 10, 4, 5, 10];
const HR_LO: [u8; 12] = [1, 10, 10, 2, 6, 10, 10, 3, 4, 10, 10, 5];

/// Read the EEPROM into settings, or return a default if an error occurs during read.
#[must_use]
fn eeprom_read_settings(eeprom: &arduino_hal::Eeprom) -> Settings {
    let mut vals: [u8; 7] = [255; 7];
    if eeprom.read(0, &mut vals).is_err() {
        return Settings::default();
    }
    let main_bright = match vals[0] {
        v @ 1..=MAIN_BRIGHT_DEFAULT => v,
        _ => MAIN_BRIGHT_DEFAULT,
    };
    Settings {
        main_bright,
        hr_bright: match vals[1] {
            v @ 0..=RED_BRIGHT_DEFAULT => v,
            _ => RED_BRIGHT_DEFAULT,
        },
        min_bright: match vals[2] {
            v @ 0..=GREEN_BRIGHT_DEFAULT => v,
            _ => GREEN_BRIGHT_DEFAULT,
        },
        sec_bright: match vals[3] {
            v @ 0..=BLUE_BRIGHT_DEFAULT => v,
            _ => BLUE_BRIGHT_DEFAULT,
        },
        ccw: match vals[4] {
            v @ 0..=1 => v == 1,
            _ => CCW_DEFAULT,
        },
        fade_mode: match vals[5] {
            v @ 0..=1 => v == 1,
            _ => FADE_MODE_DEFAULT,
        },
        last_saved_brightness: main_bright,
    }
}

#[must_use]
fn eeprom_save_settings(
    eeprom: &mut arduino_hal::Eeprom,
    main_bright: u8,
    hour_bright: u8,
    min_bright: u8,
    sec_bright: u8,
    ccw: bool,
    fade_mode: bool,
) -> u8 {
    //EEPROM.write(Addr, Value);

    // Careful if you use  this function: EEPROM has a limited number of write
    // cycles in its life.  Good for human-operated buttons, bad for automation.

    eeprom.write_byte(0, main_bright);
    eeprom.write_byte(1, hour_bright);
    eeprom.write_byte(2, min_bright);
    eeprom.write_byte(3, sec_bright);
    eeprom.write_byte(4, if ccw { 1 } else { 0 });
    eeprom.write_byte(5, if fade_mode { 1 } else { 0 });

    // Optional: Blink LEDs off to indicate when we're writing to the EEPROM
    // leds.all_off();
    // delay(100);
    main_bright
}

#[must_use]
fn normal_time_display(sec_now: u8, min_now: u8, hr_now: u8) -> (u8, u8, u8, u8, u8, u8) {
    // Offset by 30 s to project *shadow* in the right place.
    // Divide by two, since there are 30 LEDs, not 60.
    let sec_disp = (sec_now.wrapping_add(30) % 60).wrapping_div(2);
    let sec_next = sec_disp.wrapping_add(1) % 30;

    // Offset by 30 m to project *shadow* in the right place.
    // Divide by two, since there are 30 LEDs, not 60.
    let min_disp = (min_now.wrapping_add(30) % 60).wrapping_div(2);
    let min_next = min_disp.wrapping_add(1) % 30;

    // Offset by 6 h to project *shadow* in the right place.
    let hr_disp = hr_now.wrapping_add(6) % 12;
    let hr_next = hr_disp.wrapping_add(1) % 12;

    (sec_disp, sec_next, min_disp, min_next, hr_disp, hr_next)
}

/// Fade multipliers for the hour, minute, and second rings.
struct Fades {
    /// Hour ring fade multiplier for the outgoing LED. 0-63
    hr_disp: u8,
    /// Hour ring fade multiplier for the incoming LED. 0-63
    hr_next: u8,
    /// Minute ring fade multiplier for the outgoing LED. 0-63
    min_disp: u8,
    /// Minute ring fade multiplier for the incoming LED. 0-63
    min_next: u8,
    /// Second ring fade multiplier for the outgoing LED. 0-63
    sec_disp: u8,
    /// Second ring fade multiplier for the incoming LED. 0-63
    sec_next: u8,
}

impl Fades {
    /// Compute the normal fade for a given timestamp.  Fades set the brightness multiplier for the incoming and outgoing LED for each ring.
    fn normal(&mut self, time_delta_ms: u16, fade_mode: bool, sec_now: u8, min_now: u8) {
        if fade_mode {
            // Normal time display
            if sec_now & 1 != 0
            // ODD time
            {
                self.sec_next = (63 * time_delta_ms / 1000) as u8;
                self.sec_disp = 63 - self.sec_next;
            }

            // ODD time
            if min_now & 1 != 0 && sec_now == 59 {
                self.min_next = self.sec_next;
                self.min_disp = self.sec_disp;
            }

            // End of the hour, only:
            if min_now == 59 && sec_now == 59 {
                self.hr_next = self.sec_next;
                self.hr_disp = self.sec_disp;
            }
        } else {
            // no fading
            self.hr_disp = TEMP_FADE;
            self.min_disp = TEMP_FADE;
            self.sec_disp = TEMP_FADE;
        }
    }
}

// 104 is the DS3231 RTC device address
const RTC_ADDRESS: u8 = 104;

fn rtc_set_time(i2c: &mut arduino_hal::I2c, hour_in: u8, minute_in: u8, second_in: u8) {
    macro_rules! bcd_encode {
        ($v:expr) => {{
            let t = $v / 10;
            let o = $v - t * 10;
            (t << 4) | o
        }};
    }
    let buf: [u8; 3] = [
        bcd_encode!(second_in),
        bcd_encode!(minute_in),
        bcd_encode!(hour_in),
    ];
    i2c.write(RTC_ADDRESS, &buf).unwrap(); // TODO handle result.
}

fn rtc_get_time(i2c: &mut arduino_hal::I2c) -> Result<(u8, u8, u8), arduino_hal::i2c::Error> {
    // Read out time from RTC module, if present
    // send request to receive data starting at register 0
    const REG: [u8; 1] = [0];
    let mut buf: [u8; 3] = [0, 0, 0];
    i2c.write_read(RTC_ADDRESS, &REG, &mut buf)?;

    let seconds = ((buf[0] & 0b11110000) >> 4) * 10 + (buf[0] & 0b00001111); // convert BCD to decimal
    let minutes = ((buf[1] & 0b11110000) >> 4) * 10 + (buf[1] & 0b00001111); // convert BCD to decimal
    let hours = ((buf[2] & 0b00110000) >> 4) * 10 + (buf[2] & 0b00001111); // convert BCD to decimal (assume 24 hour mode)

    Ok((seconds, minutes, hours))
}

struct Leds {
    d10: port::Pin<port::mode::Output, port::PB2>,
    a0: port::Pin<port::mode::Output, port::PC0>,
    a1: port::Pin<port::mode::Output, port::PC1>,
    a2: port::Pin<port::mode::Output, port::PC2>,
    a3: port::Pin<port::mode::Output, port::PC3>,
    d4: port::Pin<port::mode::Output, port::PD4>,
    d2: port::Pin<port::mode::Output, port::PD2>,
    d8: port::Pin<port::mode::Output, port::PB0>,
    d3: port::Pin<port::mode::Output, port::PD3>,
    d9: port::Pin<port::mode::Output, port::PB1>,
}

impl Leds {
    fn take_high(&mut self, led: u8) {
        match led {
            1 => self.d10.set_high(),
            2 => self.a0.set_high(),
            3 => self.a1.set_high(),
            4 => self.a2.set_high(),
            5 => self.a3.set_high(),
            6 => self.d4.set_high(),
            7 => self.d2.set_high(),
            8 => self.d8.set_high(),
            9 => self.d3.set_high(),
            10 => self.d9.set_high(),
            _ => (),
        };
    }

    fn take_low(&mut self, led: u8) {
        match led {
            1 => self.d10.set_low(),
            2 => self.a0.set_low(),
            3 => self.a1.set_low(),
            4 => self.a2.set_low(),
            5 => self.a3.set_low(),
            6 => self.d4.set_low(),
            7 => self.d2.set_low(),
            8 => self.d8.set_low(),
            9 => self.d3.set_low(),
            10 => self.d9.set_low(),
            _ => (),
        };
    }

    fn all_off(&mut self) {
        self.d10.set_low();
        self.a0.set_low();
        self.a1.set_low();
        self.a2.set_low();
        self.a3.set_low();
        self.d4.set_low();
        self.d2.set_low();
        self.d8.set_low();
        self.d3.set_low();
        self.d9.set_low();
    }
}

const START_OPT_TIME_LIMIT: u8 = 30;

/// SettingTime enumaretes the time setting states.
#[derive(PartialEq)]
enum SettingTime {
    No,
    Hours,
    Minutes,
    Seconds,
    // "4: not setting time"
}

impl SettingTime {
    /// next moves to the next time setting state
    fn next(&self) -> Self {
        use SettingTime::*;
        match &self {
            No => No,
            Hours => Minutes,
            Minutes => Seconds,
            Seconds => Hours,
        }
    }
}

/// OptionMode enumerates the configuration option setting modes.
#[derive(PartialEq)]
enum OptionMode {
    No,
    Red,              // red (upper) colour balance
    Green,            // green (middle) colour balance
    Blue,             // blue (lower) colour balance
    CounterClockwise, // CW vs CCW
    Fade,             // Fade Mode
}

impl OptionMode {
    /// next moves to the next configuration option setting mode
    fn next(&self) -> Self {
        use OptionMode::*;
        match &self {
            No => No,
            Red => Green,
            Green => Blue,
            Blue => CounterClockwise,
            CounterClockwise => Fade,
            Fade => Red,
        }
    }
}

/// AlignMode enumerates the LED alignment configuration states.  The boolean sets auto-advance mode in each state.
#[derive(PartialEq)]
enum AlignMode {
    No,
    Hours(bool),
    Minutes(bool),
    Seconds(bool),
}

impl AlignMode {
    /// next moves to the next LED alignment configuration state
    fn next(&self) -> Self {
        use AlignMode::*;
        match *self {
            No => No,
            Hours(true) => Hours(false),
            Hours(false) => Minutes(true),
            Minutes(true) => Minutes(false),
            Minutes(false) => Seconds(true),
            Seconds(true) => Seconds(false),
            Seconds(false) => Hours(true),
        }
    }

    /// is_auto_advance is true if this is an auto-advancing LED alignment configuration state
    fn is_auto_advance(&self) -> bool {
        use AlignMode::*;
        matches!(*self, Hours(true) | Minutes(true) | Seconds(true))
    }
}

struct AlignValue {
    value: u8,
}

impl AlignValue {
    /// Increment the alignment value, wrapping past the maximum based on the alignment mode.
    fn incr(&mut self, align_mode: &AlignMode) {
        match align_mode {
            AlignMode::Seconds(_) | AlignMode::Minutes(_) => {
                if self.value > 28 {
                    self.value = 0;
                } else {
                    self.value = self.value.wrapping_add(1);
                }
            }
            AlignMode::Hours(_) => {
                if self.value > 10 {
                    self.value = 0;
                } else {
                    self.value = self.value.wrapping_add(1);
                }
            }
            _ => {}
        };
    }

    /// Decrement the alignment value, wrapping past zero based on the alignment mode.
    fn decr(&mut self, align_mode: &AlignMode) {
        if self.value > 0 {
            self.value -= 1;
        } else {
            match align_mode {
                AlignMode::Seconds(_) | AlignMode::Minutes(_) => {
                    self.value = 29;
                }
                AlignMode::Hours(_) => {
                    self.value = 11;
                }
                _ => {}
            };
        }
    }

    fn value(&self) -> u8 {
        self.value
    }

    fn reset(&mut self) {
        self.value = 0;
    }
}

/// HoldMode enumerates the button hold state.  The u8 records the time the button is held.
#[derive(PartialEq)]
enum HoldMode {
    None,
    TimeSet(u8),
    Option(u8),
    Align(u8),
}

#[arduino_hal::entry]
fn main() -> ! {
    let mut sec_now: u8 = 0;
    let mut min_now: u8 = 0;
    let mut hr_now: u8 = 0;
    let mut hr_disp: u8 = 0;
    let mut min_disp: u8 = 0;
    let mut sec_disp: u8 = 0;

    let mut last_time: u16 = 0;
    let mut time_since_button: u8 = 0;

    // Modes:
    let mut sleep_mode: bool = false;

    let mut vcr_mode: bool = true; // In VCR mode, the clock blinks at you because the time hasn't been set yet.  Initially 1 because time is NOT yet set.
    let mut factory_reset_disable: bool = false; // To make sure that we don't accidentally reset the settings...

    let mut setting_time = SettingTime::No;
    let mut align_mode = AlignMode::No;
    let mut option_mode = OptionMode::No;
    let mut align_value = AlignValue { value: 0 };
    let mut align_rate: i8 = 2;

    let mut align_loop_count: u8 = 0;
    let mut starting_option: u8 = 0;

    let mut hold_mode = HoldMode::None;

    let mut momentary_override_plus: bool = false;
    let mut momentary_override_minus: bool = false;
    let mut momentary_override_z: bool = false;

    // Initialised in normalTimeDisplay
    let mut sec_next: u8 = 0;
    let mut min_next: u8 = 0;
    let mut hr_next: u8 = 0;
    // Initialised at end of RefreshTime conditional
    let mut hr_disp_hi: u8 = 0;
    let mut hr_next_hi: u8 = 0;
    let mut min_disp_hi: u8 = 0;
    let mut min_next_hi: u8 = 0;
    let mut sec_disp_hi: u8 = 0;
    let mut sec_next_hi: u8 = 0;
    let mut hr_disp_lo: u8 = 0;
    let mut hr_next_lo: u8 = 0;
    let mut min_disp_lo: u8 = 0;
    let mut min_next_lo: u8 = 0;
    let mut sec_disp_lo: u8 = 0;
    let mut sec_next_lo: u8 = 0;

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let serial = arduino_hal::default_serial!(dp, pins, 19200);

    let (mut s_rx, mut s_tx) = {
        #[cfg(feature = "panic-serial")]
        {
            let (s_rx, s_tx) = serial.split();
            let s_tx = share_serial_port_with_panic(s_tx);
            (s_rx, s_tx)
        }
        #[cfg(not(feature = "panic-serial"))]
        serial.split()
    };

    init_tc0(dp.TC0);

    // Converted from original by correlating the Arduino C PORTx and DDRx bit manipulation against
    // https://docs.arduino.cc/hacking/hardware/PinMapping168
    let mut leds = Leds {
        d10: pins.d10.into_output(), // 1 - PB2
        a0: pins.a0.into_output(),   // 2 - PC0
        a1: pins.a1.into_output(),   // 3 - PC1
        a2: pins.a2.into_output(),   // 4 - PC2
        a3: pins.a3.into_output(),   // 5 - PC3
        d4: pins.d4.into_output(),   // 6 - PD4
        d2: pins.d2.into_output(),   // 7 - PD2
        d8: pins.d8.into_output(),   // 8 - PB0
        d3: pins.d3.into_output(),   // 9 - PD3
        d9: pins.d9.into_output(),   // 10 - PB1
    };

    // Pull-up resistors for buttons
    let (plus, minus, z) = (
        pins.d5.into_pull_up_input(),
        pins.d6.into_pull_up_input(),
        pins.d7.into_pull_up_input(),
    );

    let mut ep = arduino_hal::Eeprom::new(dp.EEPROM);
    let mut settings = eeprom_read_settings(&ep);

    // Pull up inputs are HIGH when open, and LOW when pressed.
    let (mut plus_last, mut minus_last, mut z_last) = (plus.is_low(), minus.is_low(), z.is_low());
    // ButtonHold = 0;

    /*
    // HIGHLY OPTIONAL: Set jardcoded RTC Time from within the program.
    // Example: Set time to 2:52:45.

    RTCsetTime(2,52,45);
    */

    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000, // Copied from examples.
    );

    // Check if RTC is available, and use it to set the time if so.
    let ext_rtc = match rtc_get_time(&mut i2c) {
        Ok(v) => {
            (sec_now, min_now, hr_now) = v;
            vcr_mode = false;
            true
        }
        Err(e) => {
            ufmt::uwriteln!(&mut s_tx, "i2c error in rtc_get_time: {:?}", e).unwrap();
            false
        }
    };

    unsafe { avr_device::interrupt::enable() };

    loop {
        let mut refresh_time = (align_mode != AlignMode::No)
            || (setting_time != SettingTime::No)
            || (option_mode != OptionMode::No);

        let (plus_copy, minus_copy, z_copy) = (plus.is_low(), minus.is_low(), z.is_low());

        if plus_copy != plus_last || minus_copy != minus_last || z_copy != z_last
        // Button change detected
        {
            vcr_mode = false; // End once any buttons have been pressed...
            time_since_button = 0;

            if !plus_copy && plus_last {
                // "+" Button was pressed previously, and was just released!

                if momentary_override_plus  {
                    momentary_override_plus = false;
                    // Ignore this transition if it was part of a hold sequence.
                } else if sleep_mode {
                    sleep_mode = false;
                } else if align_mode != AlignMode::No {
                    if align_mode.is_auto_advance() {
                        if align_rate < 2 {
                            align_rate = align_rate.wrapping_add(1);
                        }
                    } else {
                        align_value.incr(&align_mode);
                    }
                } else if option_mode != OptionMode::No {
                    if option_mode == OptionMode::Red && settings.hr_bright < 62 {
                        settings.hr_bright += 2;
                    }
                    if option_mode == OptionMode::Green && settings.min_bright < 62 {
                        settings.min_bright += 2;
                    }
                    if option_mode == OptionMode::Blue && settings.sec_bright < 62 {
                        settings.sec_bright += 2;
                    }
                    if option_mode == OptionMode::CounterClockwise {
                        settings.ccw = false;
                    }
                    if option_mode == OptionMode::Fade {
                        settings.fade_mode = true;
                    }
                } else if setting_time != SettingTime::No {
                    if setting_time == SettingTime::Hours {
                        hr_now = hr_now.wrapping_add(1);
                        if hr_now > 11 {
                            hr_now = 0;
                        }
                    }
                    if setting_time == SettingTime::Minutes {
                        min_now = min_now.wrapping_add(1);
                        if min_now > 59 {
                            min_now = 0;
                        }
                    }
                    if setting_time == SettingTime::Seconds {
                        sec_now = sec_now.wrapping_add(1);
                        if sec_now > 59 {
                            sec_now = 0;
                        }
                    }
                } else {
                    // Brightness control mode
                    settings.main_bright = settings.main_bright.wrapping_add(1);
                    if settings.main_bright > 8 {
                        settings.main_bright = 1;
                    }
                }
            }

            if !minus_copy && minus_last {
                // "-" Button was pressed and just released!

                vcr_mode = false; // End once any buttons have been pressed...
                time_since_button = 0;

                if momentary_override_minus  {
                    momentary_override_minus = false;
                    // Ignore this transition if it was part of a hold sequence.
                } else if sleep_mode {
                    sleep_mode = false;
                } else if align_mode != AlignMode::No {
                    if align_mode.is_auto_advance() {
                        if align_rate > -3 {
                            align_rate -= 1;
                        }
                    } else {
                        align_value.decr(&align_mode);
                    }
                } else if option_mode != OptionMode::No {
                    if option_mode == OptionMode::Red && settings.hr_bright > 1 {
                        settings.hr_bright -= 2;
                    }
                    if option_mode == OptionMode::Green && settings.min_bright > 1 {
                        settings.min_bright -= 2;
                    }
                    if option_mode == OptionMode::Blue && settings.sec_bright > 1 {
                        settings.sec_bright -= 2;
                    }
                    if option_mode == OptionMode::CounterClockwise {
                        settings.ccw = true;
                    }
                    if option_mode == OptionMode::Fade {
                        settings.fade_mode = false;
                    }
                } else if setting_time != SettingTime::No {
                    if setting_time == SettingTime::Hours {
                        if hr_now > 0 {
                            hr_now -= 1;
                        } else {
                            hr_now = 11;
                        }
                    }
                    if setting_time == SettingTime::Minutes {
                        if min_now > 0 {
                            min_now -= 1;
                        } else {
                            min_now = 59;
                        }
                    }
                    if setting_time == SettingTime::Seconds {
                        if sec_now > 0 {
                            sec_now -= 1;
                        } else {
                            sec_now = 59;
                        }
                    }
                } else {
                    // Normal brightness adjustment mode
                    if settings.main_bright > 1 {
                        settings.main_bright -= 1;
                    } else {
                        settings.main_bright = 8;
                    }
                }
            }

            if !z_copy && z_last {
                // "Z" Button was pressed and just released!

                vcr_mode = false; // End once any buttons have been pressed...
                time_since_button = 0;

                if momentary_override_z {
                    momentary_override_z = false;
                    // Ignore this transition if it was part of a hold sequence.
                } else if align_mode != AlignMode::No {
                    align_mode.next();
                    align_value.reset();
                    align_rate = 2;
                } else if option_mode != OptionMode::No {
                    option_mode.next();
                    starting_option = 0;
                } else if setting_time != SettingTime::No {
                    setting_time.next();
                } else {
                    sleep_mode = !sleep_mode;
                }
            }
        }

        (plus_last, minus_last, z_last) = (plus_copy, minus_copy, z_copy);
        let millis_copy = millis();

        // The next if statement detects and deals with the millis() rollover.
        // This introduces an error of up to  1 s, about every 50 days.
        //
        // (If you have the standard quartz timebase, this will not dominate the inaccuracy.
        // If you have the optional RTC, this error will be corrected next time we read the
        // time from the RTC.)

        if millis_copy < last_time {
            last_time = 0;
        }
        let time_delta_ms = millis_copy - last_time;
        if time_delta_ms >= 1000 {
            last_time = last_time.wrapping_add(1000);

            // Check to see if any buttons are being held down:
            hold_mode = match (plus.is_high(), minus.is_high(), z.is_high()) {
                (true, true, true) => {
                    // No buttons are pressed.
                    factory_reset_disable = true;

                    if time_since_button < 250 {
                        time_since_button = time_since_button.wrapping_add(1);
                    }
                    // 10 s after last button released...
                    if time_since_button == 10
                        && settings.last_saved_brightness != settings.main_bright
                    {
                        settings.last_saved_brightness = eeprom_save_settings(
                            &mut ep,
                            settings.main_bright,
                            settings.hr_bright,
                            settings.min_bright,
                            settings.sec_bright,
                            settings.ccw,
                            settings.fade_mode,
                        );
                    }
                    HoldMode::None
                }
                (false, false, true) => {
                    // "+" and "-" are pressed down. "Z" is up.
                    // We are holding for alignment mode.
                    match hold_mode {
                        HoldMode::Align(x) => HoldMode::Align(x + 1),
                        _ => HoldMode::Align(1),
                    }
                }
                (false, true, false) => {
                    // "+" and "Z" are pressed down. "-" is up.
                    // We are holding for option setting mode.
                    match hold_mode {
                        HoldMode::Option(x) => HoldMode::Option(x + 1),
                        _ => HoldMode::Option(1),
                    }
                }
                (true, true, false) => {
                    // "Z" is pressed down. "+" and "-" are up.
                    // We are holding for time setting mode.
                    match hold_mode {
                        HoldMode::TimeSet(x) => HoldMode::TimeSet(x + 1),
                        _ => HoldMode::TimeSet(1),
                    }
                }
                _ => hold_mode,
            };

            match hold_mode {
                HoldMode::Align(3) => {
                    momentary_override_plus = true; // Override momentary-action of switches
                    momentary_override_minus = true; // since we've detected a hold-down condition.

                    option_mode = OptionMode::No;
                    setting_time = SettingTime::No;

                    // Hold + and - for 3 s AT POWER ON to restore factory settings.
                    if !factory_reset_disable {
                        settings = Settings::default();
                        settings.last_saved_brightness = eeprom_save_settings(
                            &mut ep,
                            settings.main_bright,
                            settings.hr_bright,
                            settings.min_bright,
                            settings.sec_bright,
                            settings.ccw,
                            settings.fade_mode,
                        );
                        leds.all_off(); // Blink LEDs off to indicate restoring data
                        arduino_hal::delay_ms(100);
                    } else if align_mode != AlignMode::No {
                        align_mode = AlignMode::No;
                    } else {
                        align_mode = AlignMode::Hours(true);
                        align_value.reset();
                        align_rate = 2;
                    }
                }

                HoldMode::Option(3) => {
                    momentary_override_plus = true;
                    momentary_override_z = true;
                    align_mode = AlignMode::No;
                    setting_time = SettingTime::No;

                    if option_mode != OptionMode::No {
                        option_mode = OptionMode::No;
                        // Save options if exiting option mode!
                        settings.last_saved_brightness = eeprom_save_settings(
                            &mut ep,
                            settings.main_bright,
                            settings.hr_bright,
                            settings.min_bright,
                            settings.sec_bright,
                            settings.ccw,
                            settings.fade_mode,
                        );
                        leds.all_off(); // Blink LEDs off to indicate saving data
                        arduino_hal::delay_ms(100);
                    } else {
                        option_mode = OptionMode::Red;
                        starting_option = 0;
                    }
                }

                HoldMode::TimeSet(3) => {
                    momentary_override_z = true;

                    if (align_mode != AlignMode::No)
                        || (option_mode != OptionMode::No)
                        || setting_time != SettingTime::No
                    {
                        // If we were in any of these modes, let's now return us to normalcy.
                        // IF we are exiting time-setting mode, save the time to the RTC, if present:
                        if setting_time != SettingTime::No && ext_rtc {
                            rtc_set_time(&mut i2c, hr_now, min_now, sec_now);
                            leds.all_off(); // Blink LEDs off to indicate saving time
                            arduino_hal::delay_ms(100);
                        }

                        if option_mode != OptionMode::No {
                            // Save options if exiting option mode!
                            settings.last_saved_brightness = eeprom_save_settings(
                                &mut ep,
                                settings.main_bright,
                                settings.hr_bright,
                                settings.min_bright,
                                settings.sec_bright,
                                settings.ccw,
                                settings.fade_mode,
                            );
                            leds.all_off(); // Blink LEDs off to indicate saving data
                            arduino_hal::delay_ms(100);
                        }

                        setting_time = SettingTime::No;
                    } else {
                        // Go to setting mode IF and ONLY IF we were in regular-clock-display mode.
                        setting_time = SettingTime::Hours; // Start with HOURS in setting mode.
                    }

                    align_mode = AlignMode::No;
                    option_mode = OptionMode::No;
                }

                _ => {}
            }

            // Note: this section could act funny if you hold the buttons for 256 or more seconds.
            // So... um... don't do that.  :P

            sec_now = sec_now.wrapping_add(1);

            if sec_now > 59 {
                sec_now = 0;
                min_now = min_now.wrapping_add(1);

                // Do not check RTC time, if we are in time-setting mode.
                if (setting_time == SettingTime::No) && ext_rtc {
                    // Check value at RTC ONCE PER MINUTE, if enabled.
                    if let Ok((seconds, minutes, hours)) = rtc_get_time(&mut i2c) {
                        // IF time is off by MORE than two seconds, then correct the displayed time.
                        // Otherwise, DO NOT update the time, it may be a sampling error rather than an
                        // actual offset.
                        // Skip checking if minutes == 0. -- the 12:00:00 rollover is distracting,
                        // UNLESS this is the first time running after reset.

                        let mut updatetime = false;
                        if (minutes != 0) && (min_now != 0) {
                            let temptime1: i16 =
                                3600 * hours as i16 + 60 * minutes as i16 + seconds as i16; // Values read from RTC
                            let temptime2: i16 =
                                3600 * hr_now as i16 + 60 * min_now as i16 + sec_now as i16; // Internally stored time estimate.

                            if temptime1 > temptime2 {
                                if (temptime1 - temptime2) > 2 {
                                    updatetime = true;
                                }
                            } else if (temptime2 - temptime1) > 2 {
                                updatetime = true;
                            }
                        }

                        // if (ExtRTC) is equivalent to saying,  "if this has run before"
                        if !ext_rtc {
                            updatetime = true;
                        }
                        if updatetime {
                            sec_now = seconds;
                            min_now = minutes;
                            hr_now = hours;

                            // Convert 24-hour mode to 12-hour mode
                            if hr_now > 11 {
                                hr_now -= 12;
                            }
                        }
                    }
                }
            }

            if min_now > 59 {
                min_now = 0;
                hr_now = hr_now.wrapping_add(1);

                if hr_now > 11 {
                    hr_now = 0;
                }
            }

            refresh_time = true;
        }

        if refresh_time {
            // Calculate which LEDs to light up to give the correct shadows:

            if align_mode != AlignMode::No {
                match align_mode {
                    AlignMode::Hours(true)
                    | AlignMode::Minutes(true)
                    | AlignMode::Seconds(true) => {
                        // ODD mode, auto-advances

                        // Absolute value of AlignRate
                        let align_rate_abs: u8 = if align_rate >= 0 {
                            (align_rate.wrapping_add(1)) as u8
                        } else {
                            (-align_rate) as u8
                        };

                        // Serial.println(AlignRateAbs,DEC);

                        align_loop_count = align_loop_count.wrapping_add(1);

                        let scale_rate: u8 = match 2.cmp(&align_rate_abs) {
                            Ordering::Less => 10,
                            Ordering::Equal => 50,
                            Ordering::Greater => 250,
                        };

                        if align_loop_count > scale_rate {
                            align_loop_count = 0;

                            if align_rate >= 0 {
                                align_value.incr(&align_mode);
                            } else {
                                align_value.decr(&align_mode);
                            }
                        }
                    }
                    _ => {}
                }

                sec_disp = align_value.value() + 15; // Offset by 30 s to project *shadow* in the right place.
                if sec_disp > 29 {
                    sec_disp -= 30;
                }
                min_disp = sec_disp;
                hr_disp = align_value.value() + 6; // Offset by 6 h to project *shadow* in the right place.

                if hr_disp > 11 {
                    hr_disp -= 12;
                }
            } else if option_mode != OptionMode::No {
                // Option setting mode

                if starting_option < START_OPT_TIME_LIMIT {
                    align_loop_count = align_loop_count.wrapping_add(1); // Borrowing a counter variable...

                    if align_loop_count > 3 {
                        align_loop_count = 0;
                        starting_option = starting_option.wrapping_add(1);

                        if option_mode == OptionMode::Red
                        // Red (upper) ring color balance
                        {
                            hr_disp = hr_disp.wrapping_add(1);
                            if hr_disp > 11 {
                                hr_disp = 0;
                            }
                        }
                        if option_mode == OptionMode::Green
                        // Green (middle) ring color balance
                        {
                            min_disp = min_disp.wrapping_add(1);
                            if min_disp > 29 {
                                min_disp = 0;
                            }
                        }
                        if option_mode == OptionMode::Blue
                        // Blue (lower) ring color balance
                        {
                            sec_disp = sec_disp.wrapping_add(1);
                            if sec_disp > 29 {
                                sec_disp = 0;
                            }
                        }
                        if option_mode == OptionMode::CounterClockwise
                            || option_mode == OptionMode::Fade
                        // CW vs CCW OR fade mode
                        {
                            starting_option = START_OPT_TIME_LIMIT; // Exit this loop
                        }
                    }
                } // end "if (StartingOption < StartOptTimeLimit){}"

                if starting_option >= START_OPT_TIME_LIMIT {
                    if option_mode == OptionMode::CounterClockwise {
                        min_disp = min_disp.wrapping_add(1);
                        if min_disp > 29 {
                            min_disp = 0;
                        }
                        sec_disp = min_disp;
                    } else {
                        (sec_disp, sec_next, min_disp, min_next, hr_disp, hr_next) =
                            normal_time_display(sec_now, min_now, hr_now);
                    }
                }
            } else {
                // Regular clock display

                (sec_disp, sec_next, min_disp, min_next, hr_disp, hr_next) =
                    normal_time_display(sec_now, min_now, hr_now);
            }

            let (
                hr_disp_offset,
                hr_next_offset,
                min_disp_offset,
                min_next_offset,
                sec_disp_offset,
                sec_next_offset,
            ) = if settings.ccw {
                (
                    12 - hr_disp,
                    12 - hr_next,
                    30 - min_disp,
                    30 - min_next,
                    30 - sec_disp,
                    30 - sec_next,
                )
            } else {
                (hr_disp, hr_next, min_disp, min_next, sec_disp, sec_next)
            };

            // This Hour
            hr_disp_hi = HR_HI[hr_disp_offset as usize];
            hr_disp_lo = HR_LO[hr_disp_offset as usize];

            // Next Hour
            hr_next_hi = HR_HI[hr_next_offset as usize];
            hr_next_lo = HR_LO[hr_next_offset as usize];

            // This Min
            min_disp_hi = MIN_HI[min_disp_offset as usize];
            min_disp_lo = MIN_LO[min_disp_offset as usize];

            // Next Min
            min_next_hi = MIN_HI[min_next_offset as usize];
            min_next_lo = MIN_LO[min_next_offset as usize];

            // This Sec
            sec_disp_hi = SEC_HI[sec_disp_offset as usize];
            sec_disp_lo = SEC_LO[sec_disp_offset as usize];

            // Next Sec
            sec_next_hi = SEC_HI[sec_next_offset as usize];
            sec_next_lo = SEC_LO[sec_next_offset as usize];
        }

        let mut fades = Fades {
            sec_next: 0,
            sec_disp: 63,
            min_next: 0,
            min_disp: 63,
            hr_next: 0,
            hr_disp: 63,
        };

        if setting_time != SettingTime::No
        // i.e., if (SettingTime is nonzero)
        {
            fades.hr_disp = 5;
            fades.min_disp = 5;
            fades.sec_disp = 5;

            if setting_time == SettingTime::Hours
            // hours
            {
                fades.hr_disp = TEMP_FADE;
            }
            if setting_time == SettingTime::Minutes
            // minutes
            {
                fades.min_disp = TEMP_FADE;
            }
            if setting_time == SettingTime::Seconds
            // seconds
            {
                fades.sec_disp = TEMP_FADE;
            }
        } else if (align_mode != AlignMode::No) || option_mode != OptionMode::No
        // if either...
        {
            fades.hr_disp = 0;
            fades.min_disp = 0;
            fades.sec_disp = 0;

            if align_mode != AlignMode::No {
                match align_mode {
                    AlignMode::Hours(_) => {
                        fades.hr_disp = TEMP_FADE;
                    }
                    AlignMode::Minutes(_) => {
                        fades.min_disp = TEMP_FADE;
                    }
                    AlignMode::Seconds(_) => {
                        fades.sec_disp = TEMP_FADE;
                    }
                    _ => {}
                }
            } else {
                // Must be OptionMode....
                if starting_option < START_OPT_TIME_LIMIT {
                    if option_mode == OptionMode::Red {
                        fades.hr_disp = TEMP_FADE;
                    }
                    if option_mode == OptionMode::Green {
                        fades.min_disp = TEMP_FADE;
                    }
                    if option_mode == OptionMode::Blue {
                        fades.sec_disp = TEMP_FADE;
                    }
                    if option_mode == OptionMode::CounterClockwise
                    // CW vs CCW
                    {
                        fades.sec_disp = TEMP_FADE;
                        fades.min_disp = TEMP_FADE;
                    }
                } else {
                    // No longer in starting mode.

                    fades.hr_disp = TEMP_FADE;
                    fades.min_disp = TEMP_FADE;
                    fades.sec_disp = TEMP_FADE;

                    if option_mode == OptionMode::CounterClockwise
                    // CW vs CCW
                    {
                        fades.hr_disp = 0;
                    } else {
                        fades.normal(time_delta_ms, settings.fade_mode, sec_now, min_now);
                    }
                }
            }
        } else {
            fades.normal(time_delta_ms, settings.fade_mode, sec_now, min_now);
        }

        // if setting_time != 0
        let tempbright: u16 = if sleep_mode || (vcr_mode && (sec_now & 1 != 0)) {
            0
        } else {
            settings.main_bright as u16
        };

        // 0-63 (6) * 0-63 (6) * 0-8 (3) dynamic range is 15 bits.
        // Shifted 7 puts the high bits into a u8.
        let hr_disp_delay =
            ((settings.hr_bright as u16 * fades.hr_disp as u16 * tempbright) >> 7) as u8;
        let hr_next_delay =
            ((settings.hr_bright as u16 * fades.hr_next as u16 * tempbright) >> 7) as u8;
        let min_disp_delay =
            ((settings.min_bright as u16 * fades.min_disp as u16 * tempbright) >> 7) as u8;
        let min_next_delay =
            ((settings.min_bright as u16 * fades.min_next as u16 * tempbright) >> 7) as u8;
        let sec_disp_delay =
            ((settings.sec_bright as u16 * fades.sec_disp as u16 * tempbright) >> 7) as u8;
        let sec_next_delay =
            ((settings.sec_bright as u16 * fades.sec_next as u16 * tempbright) >> 7) as u8;

        // unsigned long  temp = millis();

        // This is the loop where we actually light up the LEDs:
        // 128 cycles: ROUGHLY 39 ms  => Full redraw at about 3 kHz.
        for _ in 0..128 {
            if hr_disp_delay > 0 {
                leds.take_high(hr_disp_hi);
                leds.take_low(hr_disp_lo);
                delay_time(hr_disp_delay);
                leds.all_off();
            }

            if hr_next_delay > 0 {
                leds.take_high(hr_next_hi);
                leds.take_low(hr_next_lo);
                delay_time(hr_next_delay);
                leds.all_off();
            }

            if min_disp_delay > 0 {
                leds.take_high(min_disp_hi);
                leds.take_low(min_disp_lo);
                delay_time(min_disp_delay);
                leds.all_off();
            }

            if min_next_delay > 0 {
                leds.take_high(min_next_hi);
                leds.take_low(min_next_lo);
                delay_time(min_next_delay);
                leds.all_off();
            }

            if sec_disp_delay > 0 {
                leds.take_high(sec_disp_hi);
                leds.take_low(sec_disp_lo);
                delay_time(sec_disp_delay);
                leds.all_off();
            }

            if sec_next_delay > 0 {
                leds.take_high(sec_next_hi);
                leds.take_low(sec_next_lo);
                delay_time(sec_next_delay);
                leds.all_off();
            }

            if settings.main_bright < 8 {
                delay_time((8 - settings.main_bright) << 5);
                delay_time((8 - settings.main_bright) << 5);
                delay_time((8 - settings.main_bright) << 5);
            }
        }

        /*
        temp = millis() - temp;
        Serial.println(temp,DEC);
         */

        // Can this sync be tried only once per second?
        if let Some(v) = get_pc_time(&mut s_rx) {
            (hr_now, min_now, sec_now) = v;

            if hr_now > 11 {
                // Convert 24-hour mode to 12-hour mode
                hr_now -= 12;
            }

            // Print confirmation
            ufmt::uwriteln!(s_tx, "Clock synced at: {}:{}:{}", hr_now, min_now, sec_now).unwrap();

            if ext_rtc {
                rtc_set_time(&mut i2c, hr_now, min_now, sec_now);
            }
        }
    }
}
