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

// Serial-writer panic handler overflows .text region
//mod panic;
mod time;
mod timer;
use crate::time::*;
use crate::timer::*;
use arduino_hal::prelude::*;
use panic_abort as _;

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

type SerialReader = arduino_hal::hal::usart::UsartReader<
    arduino_hal::pac::USART0,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Input, arduino_hal::hal::port::PD0>,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Output, arduino_hal::hal::port::PD1>,
    arduino_hal::clock::MHz16,
>;

fn get_pc_time(s_rx: &mut SerialReader) -> bool {
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
                        set_time(pctime);
                        return true;
                    }
                    _ => return false,
                };
            }
            return false;
        }
        _ => return false,
    }
}

type SerialWriter = arduino_hal::hal::usart::UsartWriter<
    arduino_hal::pac::USART0,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Input, arduino_hal::hal::port::PD0>,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Output, arduino_hal::hal::port::PD1>,
    arduino_hal::clock::MHz16,
>;

fn digital_clock_display(s_tx: &mut SerialWriter) {
    // digital clock display of current date and time
    ufmt::uwrite!(s_tx, "{}:{}:{}", hour(), minute(), second()).unwrap();
    ufmt::uwriteln!(s_tx, " {} {} {}", weekday(), month(), day()).unwrap();
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
fn eeprom_read_settings(eeprom: &mut arduino_hal::Eeprom) -> Settings {
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

    let last_saved_brightness = main_bright;

    // Optional: Blink LEDs off to indicate when we're writing to the EEPROM
    // AllLEDsOff!();
    // delay(100);
    last_saved_brightness
}

#[must_use]
fn normal_time_display(sec_now: u8, min_now: u8, hr_now: u8) -> (u8, u8, u8, u8, u8, u8) {
    // Offset by 30 s to project *shadow* in the right place.
    // Divide by two, since there are 30 LEDs, not 60.
    let sec_disp = ((sec_now + 30) % 60) / 2;
    let sec_next = (sec_disp + 1) % 30;

    // Offset by 30 m to project *shadow* in the right place.
    // Divide by two, since there are 30 LEDs, not 60.
    let min_disp = ((min_now + 30) % 60) / 2;
    let min_next = (min_disp + 1) % 30;

    // Offset by 6 h to project *shadow* in the right place.
    let hr_disp = (hr_now + 6) % 12;
    let hr_next = (hr_disp + 1) % 12;

    (sec_disp, sec_next, min_disp, min_next, hr_disp, hr_next)
}

#[must_use]
fn normal_fades(
    millis: u32,
    last_time: u32,
    fade_mode: bool,
    sec_now: u8,
    min_now: u8,
    mut sec_fade_1: u8,
    mut sec_fade_2: u8,
    mut min_fade_1: u8,
    mut min_fade_2: u8,
    mut hr_fade_1: u8,
    mut hr_fade_2: u8,
) -> (u8, u8, u8, u8, u8, u8) {
    if fade_mode {
        // Normal time display
        if sec_now & 1 != 0
        // ODD time
        {
            sec_fade_2 = (63 * (millis - last_time) / 1000) as u8;
            sec_fade_1 = 63 - sec_fade_2;
        }

        if min_now & 1 != 0
        // ODD time
        {
            if sec_now == 59 {
                min_fade_2 = sec_fade_2;
                min_fade_1 = sec_fade_1;
            }
        }

        if min_now == 59
        // End of the hour, only:
        {
            if sec_now == 59 {
                hr_fade_2 = sec_fade_2;
                hr_fade_1 = sec_fade_1;
            }
        }
    } else {
        // no fading

        hr_fade_1 = TEMP_FADE;
        min_fade_1 = TEMP_FADE;
        sec_fade_1 = TEMP_FADE;
    }
    (
        sec_fade_1, sec_fade_2, min_fade_1, min_fade_2, hr_fade_1, hr_fade_2,
    )
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

#[must_use]
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

#[must_use]
fn incr_align_val(mut align_value: u8, align_mode: u8) -> u8 {
    align_value += 1;

    if align_mode < 5
    // seconds or minutes
    {
        if align_value > 29 {
            align_value = 0;
        }
    } else {
        if align_value > 11 {
            align_value = 0;
        }
    }
    align_value
}
#[must_use]
fn decr_align_val(mut align_value: u8, align_mode: u8) -> u8 {
    if align_value > 0 {
        align_value -= 1;
    } else if align_mode < 5
    // seconds or minutes
    {
        align_value = 29;
    } else
    // hours
    {
        align_value = 11;
    }
    align_value
}

const START_OPT_TIME_LIMIT: u8 = 30;

#[arduino_hal::entry]
fn main() -> ! {
    let mut sec_now: u8 = 0;
    let mut min_now: u8 = 0;
    let mut hr_now: u8 = 0;
    let mut hr_disp: u8 = 0;
    let mut min_disp: u8 = 0;
    let mut sec_disp: u8 = 0;

    let mut last_time: u32 = 0;
    let mut time_since_button: u8 = 0;

    // Modes:
    let mut sleep_mode: bool = false;

    let mut vcr_mode: bool = true; // In VCR mode, the clock blinks at you because the time hasn't been set yet.  Initially 1 because time is NOT yet set.
    let mut factory_reset_disable: u8 = 0; // To make sure that we don't accidentally reset the settings...

    let mut setting_time: u8 = 0; // Normally 0.
                                  // 1: hours, 2: minutes, 3: seconds, 4: not setting time

    let mut align_mode: u8 = 0; // Normally 0.
    let mut option_mode: u8 = 0; // NOrmally 0
    let mut align_value: u8 = 0;
    let mut align_rate: i8 = 2;

    let mut align_loop_count: u8 = 0;
    let mut starting_option: u8 = 0;

    let mut hold_time_set: u8 = 0;
    let mut hold_option: u8 = 0;
    let mut hold_align: u8 = 0;

    let mut momentary_override_plus: u8 = 0;
    let mut momentary_override_minus: u8 = 0;
    let mut momentary_override_z: u8 = 0;

    // Initialised in normalTimeDisplay
    let mut sec_next: u8 = 0;
    let mut min_next: u8 = 0;
    let mut hr_next: u8 = 0;
    // Initialised at end of RefreshTime conditional
    let mut h0: u8 = 0;
    let mut h1: u8 = 0;
    let mut h2: u8 = 0;
    let mut h3: u8 = 0;
    let mut h4: u8 = 0;
    let mut h5: u8 = 0;
    let mut l0: u8 = 0;
    let mut l1: u8 = 0;
    let mut l2: u8 = 0;
    let mut l3: u8 = 0;
    let mut l4: u8 = 0;
    let mut l5: u8 = 0;
    let mut d0: u8;
    let mut d1: u8;
    let mut d2: u8;
    let mut d3: u8;
    let mut d4: u8;
    let mut d5: u8;
    // Initialised in normalFades or at end of RefreshTime conditional
    let mut hr_fade_1: u8;
    let mut hr_fade_2: u8;
    let mut min_fade_1: u8;
    let mut min_fade_2: u8;
    let mut sec_fade_1: u8;
    let mut sec_fade_2: u8;

    let mut prevtime: u32 = 0;

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let serial = arduino_hal::default_serial!(dp, pins, 19200);
    let (mut s_rx, mut s_tx) = serial.split();

    init_tc0(dp.TC0);

    set_time(0);

    // Converted from original by correlating the Arduino C PORTx and DDRx bit manipulation against
    // https://docs.arduino.cc/hacking/hardware/PinMapping168
    let mut leds = [
        pins.d10.into_output().downgrade(), // PB2
        pins.a0.into_output().downgrade(),  // PC0
        pins.a1.into_output().downgrade(),  // PC1
        pins.a2.into_output().downgrade(),  // PC2
        pins.a3.into_output().downgrade(),  // PC3
        pins.d4.into_output().downgrade(),  // PD4
        pins.d2.into_output().downgrade(),  // PD2
        pins.d8.into_output().downgrade(),  // PB0
        pins.d3.into_output().downgrade(),  // PD3
        pins.d9.into_output().downgrade(),  // PB1
    ];

    macro_rules! TakeHigh {
        ($led:expr) => {{
            leds[$led as usize].set_high();
        }};
    }
    macro_rules! TakeLow {
        ($led:expr) => {{
            leds[$led as usize].set_low();
        }};
    }
    macro_rules! AllLEDsOff {
        () => {{
            for led in leds.iter_mut() {
                led.set_low();
            }
        }};
    }

    // Pull-up resistors for buttons
    let (plus, minus, z) = (
        pins.d5.into_pull_up_input(),
        pins.d6.into_pull_up_input(),
        pins.d7.into_pull_up_input(),
    );

    let mut ep = arduino_hal::Eeprom::new(dp.EEPROM);
    let mut settings = eeprom_read_settings(&mut ep);

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
            true
        }
        Err(e) => {
            ufmt::uwriteln!(&mut s_tx, "i2c error in rtc_get_time: {:?}", e).unwrap();
            false
        }
    };

    if ext_rtc {
        // If time is already set from the RTC...
        vcr_mode = false;
    }

    unsafe { avr_device::interrupt::enable() };

    loop {
        let mut refresh_time = align_mode + setting_time + option_mode;

        let (plus_copy, minus_copy, z_copy) = (plus.is_low(), minus.is_low(), z.is_low());

        if plus_copy != plus_last || minus_copy != minus_last || z_copy != z_last
        // Button change detected
        {
            vcr_mode = false; // End once any buttons have been pressed...
            time_since_button = 0;

            if !plus_copy && plus_last {
                // "+" Button was pressed previously, and was just released!

                if momentary_override_plus != 0 {
                    momentary_override_plus = 0;
                    // Ignore this transition if it was part of a hold sequence.
                } else {
                    if sleep_mode {
                        sleep_mode = false;
                    } else {
                        if align_mode != 0 {
                            if align_mode & 1 != 0
                            // Odd mode:
                            {
                                if align_rate < 2 {
                                    align_rate += 1;
                                }
                            } else {
                                align_value = incr_align_val(align_value, align_mode);
                                // Even mode:
                            }
                        } else if option_mode != 0 {
                            if option_mode == 1 {
                                if settings.hr_bright < 62 {
                                    settings.hr_bright += 2;
                                }
                            }
                            if option_mode == 2 {
                                if settings.min_bright < 62 {
                                    settings.min_bright += 2;
                                }
                            }
                            if option_mode == 3 {
                                if settings.sec_bright < 62 {
                                    settings.sec_bright += 2;
                                }
                            }
                            if option_mode == 4 {
                                settings.ccw = false;
                            }
                            if option_mode == 5 {
                                settings.fade_mode = true;
                            }
                        } else if setting_time != 0 {
                            if setting_time == 1 {
                                hr_now += 1;
                                if hr_now > 11 {
                                    hr_now = 0;
                                }
                            }
                            if setting_time == 2 {
                                min_now += 1;
                                if min_now > 59 {
                                    min_now = 0;
                                }
                            }
                            if setting_time == 3 {
                                sec_now += 1;
                                if sec_now > 59 {
                                    sec_now = 0;
                                }
                            }
                        } else {
                            // Brightness control mode
                            settings.main_bright += 1;
                            if settings.main_bright > 8 {
                                settings.main_bright = 1;
                            }
                        }
                    }
                }
            }

            if !minus_copy && minus_last {
                // "-" Button was pressed and just released!

                vcr_mode = false; // End once any buttons have been pressed...
                time_since_button = 0;

                if momentary_override_minus != 0 {
                    momentary_override_minus = 0;
                    // Ignore this transition if it was part of a hold sequence.
                } else {
                    if sleep_mode {
                        sleep_mode = false;
                    } else {
                        if align_mode != 0 {
                            if align_mode & 1 != 0
                            // Odd mode:
                            {
                                if align_rate > -3 {
                                    align_rate -= 1;
                                }
                            } else {
                                align_value = decr_align_val(align_value, align_mode);
                                // Even mode:
                            }
                        } else if option_mode != 0 {
                            if option_mode == 1 {
                                if settings.hr_bright > 1 {
                                    settings.hr_bright -= 2;
                                }
                            }
                            if option_mode == 2 {
                                if settings.min_bright > 1 {
                                    settings.min_bright -= 2;
                                }
                            }
                            if option_mode == 3 {
                                if settings.sec_bright > 1 {
                                    settings.sec_bright -= 2;
                                }
                            }
                            if option_mode == 4 {
                                settings.ccw = true;
                            }
                            if option_mode == 5 {
                                settings.fade_mode = false;
                            }
                        } else if setting_time != 0 {
                            if setting_time == 1 {
                                if hr_now > 0 {
                                    hr_now -= 1;
                                } else {
                                    hr_now = 11;
                                }
                            }
                            if setting_time == 2 {
                                if min_now > 0 {
                                    min_now -= 1;
                                } else {
                                    min_now = 59;
                                }
                            }
                            if setting_time == 3 {
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
                }
            }

            if !z_copy && z_last {
                // "Z" Button was pressed and just released!

                vcr_mode = false; // End once any buttons have been pressed...
                time_since_button = 0;

                if momentary_override_z != 0 {
                    momentary_override_z = 0;
                    // Ignore this transition if it was part of a hold sequence.
                } else {
                    if align_mode != 0 {
                        align_mode += 1;
                        if align_mode > 6 {
                            align_mode = 1;
                        }
                        align_value = 0;
                        align_rate = 2;
                    } else if option_mode != 0 {
                        option_mode += 1;
                        starting_option = 0;

                        if option_mode > 5 {
                            option_mode = 1;
                        }
                    } else if setting_time != 0 {
                        setting_time += 1;
                        if setting_time > 3 {
                            setting_time = 1;
                        }
                    } else {
                        if !sleep_mode {
                            sleep_mode = true;
                        } else {
                            sleep_mode = false;
                        }
                    }
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
        if (millis_copy - last_time) >= 1000 {
            last_time += 1000;

            // Check to see if any buttons are being held down:

            if plus.is_high() & minus.is_high() && z.is_high() {
                // No buttons are pressed.
                // Reset the variables that check to see if buttons are being held down.

                hold_time_set = 0;
                hold_option = 0;
                hold_align = 0;
                factory_reset_disable = 1;

                if time_since_button < 250 {
                    time_since_button += 1;
                }
                if time_since_button == 10
                // 10 s after last button released...
                {
                    if settings.last_saved_brightness != settings.main_bright {
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
                }
            } else {
                // Note which buttons are being held down

                if plus.is_low() & minus.is_low()
                // "+" and "-" are pressed down. "Z" is up.
                {
                    hold_align += 1; // We are holding for alignment mode.
                    hold_option = 0;
                    hold_time_set = 0;
                }
                if plus.is_low() & z.is_low()
                // "+" and "Z" are pressed down. "-" is up.
                {
                    hold_option += 1; // We are holding for option setting mode.
                    hold_time_set = 0;
                    hold_align = 0;
                }
                if z.is_low()
                // "Z" is pressed down. "+" and "-" are up.
                {
                    hold_time_set += 1; // We are holding for time setting mode.
                    hold_option = 0;
                    hold_align = 0;
                }
            }

            if hold_align == 3 {
                momentary_override_plus = 1; // Override momentary-action of switches
                momentary_override_minus = 1; // since we've detected a hold-down condition.

                option_mode = 0;
                setting_time = 0;

                // Hold + and - for 3 s AT POWER ON to restore factory settings.
                if factory_reset_disable == 0 {
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
                    AllLEDsOff!(); // Blink LEDs off to indicate restoring data
                    arduino_hal::delay_ms(100);
                } else {
                    if align_mode != 0 {
                        align_mode = 0;
                    } else {
                        align_mode = 1;
                        align_value = 0;
                        align_rate = 2;
                    }
                }
            }

            if hold_option == 3 {
                momentary_override_plus = 1;
                momentary_override_z = 1;
                align_mode = 0;
                setting_time = 0;

                if option_mode != 0 {
                    option_mode = 0;
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
                    AllLEDsOff!(); // Blink LEDs off to indicate saving data
                    arduino_hal::delay_ms(100);
                } else {
                    option_mode = 1;
                    starting_option = 0;
                }
            }

            if hold_time_set == 3 {
                momentary_override_z = 1;

                if align_mode + option_mode + setting_time != 0 {
                    // If we were in any of these modes, let's now return us to normalcy.
                    // IF we are exiting time-setting mode, save the time to the RTC, if present:
                    if setting_time != 0 && ext_rtc {
                        rtc_set_time(&mut i2c, hr_now, min_now, sec_now);
                        AllLEDsOff!(); // Blink LEDs off to indicate saving time
                        arduino_hal::delay_ms(100);
                    }

                    if option_mode != 0 {
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
                        AllLEDsOff!(); // Blink LEDs off to indicate saving data
                        arduino_hal::delay_ms(100);
                    }

                    setting_time = 0;
                } else {
                    // Go to setting mode IF and ONLY IF we were in regular-clock-display mode.
                    setting_time = 1; // Start with HOURS in setting mode.
                }

                align_mode = 0;
                option_mode = 0;
            }

            // Note: this section could act funny if you hold the buttons for 256 or more seconds.
            // So... um... don't do that.  :P

            sec_now += 1;

            if sec_now > 59 {
                sec_now = 0;
                min_now += 1;

                // Do not check RTC time, if we are in time-setting mode.
                if (setting_time == 0) && ext_rtc {
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
                            } else {
                                if (temptime2 - temptime1) > 2 {
                                    updatetime = true;
                                }
                            }
                        }

                        // if (ExtRTC) is equivalent to saying,  "if this has run before"
                        if !ext_rtc {
                            updatetime = true;
                        }
                        if updatetime {
                            sec_now = seconds as u8;
                            min_now = minutes as u8;
                            hr_now = hours as u8;

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
                hr_now += 1;

                if hr_now > 11 {
                    hr_now = 0;
                }
            }

            refresh_time = 1;
        }

        if refresh_time != 0 {
            // Calculate which LEDs to light up to give the correct shadows:

            if align_mode != 0 {
                if align_mode & 1 != 0 {
                    // ODD mode, auto-advances

                    // Absolute value of AlignRate
                    let align_rate_abs: u8 = if align_rate >= 0 {
                        (align_rate + 1) as u8
                    } else {
                        (-align_rate) as u8
                    };

                    // Serial.println(AlignRateAbs,DEC);

                    align_loop_count += 1;

                    let scale_rate: u8 = if align_rate_abs > 2 {
                        10
                    } else if align_rate_abs == 2 {
                        50
                    } else {
                        250
                    };

                    if align_loop_count > scale_rate {
                        align_loop_count = 0;

                        if align_rate >= 0 {
                            align_value = incr_align_val(align_value, align_mode);
                        } else {
                            align_value = decr_align_val(align_value, align_mode);
                        }
                    }
                }

                sec_disp = align_value + 15; // Offset by 30 s to project *shadow* in the right place.
                if sec_disp > 29 {
                    sec_disp -= 30;
                }
                min_disp = sec_disp;
                hr_disp = align_value + 6; // Offset by 6 h to project *shadow* in the right place.

                if hr_disp > 11 {
                    hr_disp -= 12;
                }
            } else if option_mode != 0 {
                // Option setting mode

                if starting_option < START_OPT_TIME_LIMIT {
                    align_loop_count += 1; // Borrowing a counter variable...

                    if align_loop_count > 3 {
                        align_loop_count = 0;
                        starting_option += 1;

                        if option_mode == 1
                        // Red (upper) ring color balance
                        {
                            hr_disp += 1;
                            if hr_disp > 11 {
                                hr_disp = 0;
                            }
                        }
                        if option_mode == 2
                        // Green (middle) ring color balance
                        {
                            min_disp += 1;
                            if min_disp > 29 {
                                min_disp = 0;
                            }
                        }
                        if option_mode == 3
                        // Blue (lower) ring color balance
                        {
                            sec_disp += 1;
                            if sec_disp > 29 {
                                sec_disp = 0;
                            }
                        }
                        if option_mode >= 4
                        // CW vs CCW OR fade mode
                        {
                            starting_option = START_OPT_TIME_LIMIT; // Exit this loop
                        }
                    }
                } // end "if (StartingOption < StartOptTimeLimit){}"

                if starting_option >= START_OPT_TIME_LIMIT {
                    if option_mode == 4 {
                        min_disp += 1;
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

            h3 = hr_disp;
            l3 = hr_next;
            h4 = min_disp;
            l4 = min_next;
            h5 = sec_disp;
            l5 = sec_next;

            if settings.ccw {
                // Counterclockwise
                if hr_disp != 0 {
                    h3 = 12 - hr_disp;
                }
                if hr_next != 0 {
                    l3 = 12 - hr_next;
                }
                if min_disp != 0 {
                    h4 = 30 - min_disp;
                }
                if min_next != 0 {
                    l4 = 30 - min_next;
                }
                if sec_disp != 0 {
                    h5 = 30 - sec_disp;
                }
                if sec_next != 0 {
                    l5 = 30 - sec_next;
                }

                // Serial.print(hr_disp,DEC);
                // Serial.print(", ");
                // Serial.println(h3,DEC);
            }

            h0 = HR_HI[h3 as usize];
            l0 = HR_LO[h3 as usize];

            h1 = HR_HI[l3 as usize];
            l1 = HR_LO[l3 as usize];

            h2 = MIN_HI[h4 as usize];
            l2 = MIN_LO[h4 as usize];

            h3 = MIN_HI[l4 as usize];
            l3 = MIN_LO[l4 as usize];

            h4 = SEC_HI[h5 as usize];
            l4 = SEC_LO[h5 as usize];

            h5 = SEC_HI[l5 as usize];
            l5 = SEC_LO[l5 as usize];
        }

        sec_fade_2 = 0;
        sec_fade_1 = 63;

        min_fade_2 = 0;
        min_fade_1 = 63;

        hr_fade_2 = 0;
        hr_fade_1 = 63;

        if setting_time != 0
        // i.e., if (SettingTime is nonzero)
        {
            hr_fade_1 = 5;
            min_fade_1 = 5;
            sec_fade_1 = 5;

            if setting_time == 1
            // hours
            {
                hr_fade_1 = TEMP_FADE;
            }
            if setting_time == 2
            // minutes
            {
                min_fade_1 = TEMP_FADE;
            }
            if setting_time == 3
            // seconds
            {
                sec_fade_1 = TEMP_FADE;
            }
        } else if align_mode + option_mode != 0
        // if either...
        {
            hr_fade_1 = 0;
            min_fade_1 = 0;
            sec_fade_1 = 0;

            if align_mode != 0 {
                if align_mode < 3 {
                    sec_fade_1 = TEMP_FADE;
                } else if align_mode > 4 {
                    hr_fade_1 = TEMP_FADE;
                } else {
                    min_fade_1 = TEMP_FADE;
                }
            } else {
                // Must be OptionMode....
                if starting_option < START_OPT_TIME_LIMIT {
                    if option_mode == 1 {
                        hr_fade_1 = TEMP_FADE;
                    }
                    if option_mode == 2 {
                        min_fade_1 = TEMP_FADE;
                    }
                    if option_mode == 3 {
                        sec_fade_1 = TEMP_FADE;
                    }
                    if option_mode == 4
                    // CW vs CCW
                    {
                        sec_fade_1 = TEMP_FADE;
                        min_fade_1 = TEMP_FADE;
                    }
                } else {
                    // No longer in starting mode.

                    hr_fade_1 = TEMP_FADE;
                    min_fade_1 = TEMP_FADE;
                    sec_fade_1 = TEMP_FADE;

                    if option_mode == 4
                    // CW vs CCW
                    {
                        hr_fade_1 = 0;
                    } else {
                        (
                            sec_fade_1, sec_fade_2, min_fade_1, min_fade_2, hr_fade_1, hr_fade_2,
                        ) = normal_fades(
                            millis_copy,
                            last_time,
                            settings.fade_mode,
                            sec_now,
                            min_now,
                            sec_fade_1,
                            sec_fade_2,
                            min_fade_1,
                            min_fade_2,
                            hr_fade_1,
                            hr_fade_2,
                        );
                    }
                }
            }
        } else {
            (
                sec_fade_1, sec_fade_2, min_fade_1, min_fade_2, hr_fade_1, hr_fade_2,
            ) = normal_fades(
                millis_copy,
                last_time,
                settings.fade_mode,
                sec_now,
                min_now,
                sec_fade_1,
                sec_fade_2,
                min_fade_1,
                min_fade_2,
                hr_fade_1,
                hr_fade_2,
            );
        }

        let mut tempbright: u8 = settings.main_bright;

        if sleep_mode {
            tempbright = 0;
        }

        if vcr_mode {
            if sec_now & 1 != 0 {
                tempbright = 0;
            }
        }

        d0 = settings.hr_bright * hr_fade_1 * tempbright >> 7;
        d1 = settings.hr_bright * hr_fade_2 * tempbright >> 7;
        d2 = settings.min_bright * min_fade_1 * tempbright >> 7;
        d3 = settings.min_bright * min_fade_2 * tempbright >> 7;
        d4 = settings.sec_bright * sec_fade_1 * tempbright >> 7;
        d5 = settings.sec_bright * sec_fade_2 * tempbright >> 7;

        // unsigned long  temp = millis();

        // This is the loop where we actually light up the LEDs:
        let mut i: u8 = 0;
        while i < 128
        // 128 cycles: ROUGHLY 39 ms  => Full redraw at about 3 kHz.
        {
            if d0 > 0 {
                TakeHigh!(h0);
                TakeLow!(l0);
                delay_time(d0);
                AllLEDsOff!();
            }

            if d1 > 0 {
                TakeHigh!(h1);
                TakeLow!(l1);
                delay_time(d1);
                AllLEDsOff!();
            }

            if d2 > 0 {
                TakeHigh!(h2);
                TakeLow!(l2);
                delay_time(d2);
                AllLEDsOff!();
            }

            if d3 > 0 {
                TakeHigh!(h3);
                TakeLow!(l3);
                delay_time(d3);
                AllLEDsOff!();
            }

            if d4 > 0 {
                TakeHigh!(h4);
                TakeLow!(l4);
                delay_time(d4);
                AllLEDsOff!();
            }

            if d5 > 0 {
                TakeHigh!(h5);
                TakeLow!(l5);
                delay_time(d5);
                AllLEDsOff!();
            }

            if settings.main_bright < 8 {
                delay_time((8 - settings.main_bright) << 5);
                delay_time((8 - settings.main_bright) << 5);
                delay_time((8 - settings.main_bright) << 5);
            }

            i += 1;
        }

        /*
        temp = millis() - temp;
        Serial.println(temp,DEC);
         */

        // Can this sync be tried only once per second?
        if get_pc_time(&mut s_rx) {
            // try to get time sync from pc

            // Set time to that given from PC.
            min_now = minute() as u8;
            sec_now = second() as u8;
            hr_now = hour() as u8;

            if hr_now > 11 {
                // Convert 24-hour mode to 12-hour mode
                hr_now -= 12;
            }

            // Print confirmation
            ufmt::uwriteln!(s_tx, "Clock synced at: {}", now()).unwrap();

            if time_status() == TimeStatus::TimeSet {
                // update clocks if time has been synced

                if prevtime != now() {
                    if ext_rtc {
                        rtc_set_time(&mut i2c, hr_now, min_now, sec_now);
                    }

                    time_status(); // refresh the Date and time properties
                    digital_clock_display(&mut s_tx); // update digital clock
                    prevtime = now();
                }
            }
        }
    }
}
