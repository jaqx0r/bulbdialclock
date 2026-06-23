// This library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this library.  If not, see <http://www.gnu.org/licenses/>.
//! Firmware for the Bulbdial Clock kit designed by Evil Mad Scientist
//! Laboratories: <http://www.evilmadscientist.com/go/bulbdialkit>
//!
//! Target: ATmega168, clock at 16 MHz.
//!
//! Rust port by Jamie Wilkinson
//!
//! Updated to work with Arduino 1.0 by Ray Ramirez
//!
//! Version 1.0.1 - 1/14/2012
//! Copyright (c) 2009 Windell H. Oskay.  All right reserved.
//! <http://www.evilmadscientist.com/>

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod ds3231;
mod leds;
mod settings;
use crate::leds::{DisplayController, Leds};
use crate::settings::Settings;
use embedded_hal::digital::InputPin;
mod timer;
use crate::timer::*;
use arduino_hal::prelude::*;
use core::cmp::Ordering;

#[cfg(feature = "serial-sync")]
mod pc_sync;

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

const TEMP_FADE: u8 = 63;

/// Introduce a delay of `cycles` cycles.  At 16MHz, each cycle is 0.0625µs.
fn delay_time(cycles: u8) {
    for _ in 0..cycles {
        avr_device::asm::nop();
    }
}

/// Array of pin pairs to set as output hi/lo to activate a LED in the blue
/// Seconds ring.  Pin numbering refers to the index in `Leds.take*`.
// LED pin activation is a 6x6 matrix of column high and row low; with no
// diagonal that gives 30 LEDs.  See page 1 of the schematic at
// https://bcdn.evilmadscientist.com/source/beedyschem.pdf
const SEC_PINS: [(u8, u8); 30] = [
    (2, 1), // D21
    (3, 1), // D31
    (4, 1), // D41
    (5, 1), // D51
    (6, 1), // D61
    (1, 2), // D12
    (3, 2), // D32
    (4, 2), // D42
    (5, 2), // D52
    (6, 2), // D62
    (1, 3), // D13
    (2, 3), // D23
    (4, 3), // D43
    (5, 3), // D53
    (6, 3), // D63
    (1, 4), // D14
    (2, 4), // D24
    (3, 4), // D34
    (5, 4), // D54
    (6, 4), // D64
    (1, 5), // D15
    (2, 5), // D25
    (3, 5), // D35
    (4, 5), // D45
    (6, 5), // D65
    (1, 6), // D16
    (2, 6), // D26
    (3, 6), // D36
    (4, 6), // D46
    (5, 6), // D56
];

/// Array of pin pairs to set as output hi/lo to activate a LED in the green
/// Minutes ring.  Pin numbering refers to the index in `Leds.take*`.
// LED pin activation is 2x5x3 matrices of column high and row low, giving 30
// LEDs.  See page 2 of the schematic at
// https://bcdn.evilmadscientist.com/source/beedyschem.pdf
const MIN_PINS: [(u8, u8); 30] = [
    (1, 7), // D17
    (7, 1), // D71
    (1, 8), // D18
    (8, 1), // D81
    (1, 9), // D19
    (9, 1), // D91
    (2, 7), // D27
    (7, 2), // D72
    (2, 8), // D28
    (8, 2), // D82
    (2, 9), // D29
    (9, 2), // D92
    (3, 7), // D37
    (7, 3), // D73
    (3, 8), // D38
    (8, 3), // D83
    (3, 9), // D39
    (9, 3), // D93
    (4, 7), // D47
    (7, 4), // D74
    (4, 8), // D48
    (8, 4), // D84
    (4, 9), // D49
    (9, 4), // D94
    (5, 7), // D57
    (7, 5), // D75
    (5, 8), // D58
    (8, 5), // D85
    (5, 9), // D59
    (9, 5), // D95
];

/// Array of pin pairs to set as output hi/lo to activate a LED in the red Hour
/// ring.  Pin numbering refers to the index in `Leds.take*`.
// LED pin activation is 2x6x1 arrays of column hi and row low, giving 12 LEDs.
// See page 3 of the schematic at
// https://bcdn.evilmadscientist.com/source/beedyschem.pdf
const HR_PINS: [(u8, u8); 12] = [
    (10, 1), // D101
    (1, 10), // D110
    (2, 10), // D210
    (10, 2), // D102
    (10, 6), // D106
    (6, 10), // D610
    (3, 10), // D310
    (10, 3), // D103
    (10, 4), // D104
    (4, 10), // D410
    (5, 10), // D510
    (10, 5), // D105
];

// A time limit after which we exit option setting mode.
const START_OPT_TIME_LIMIT: u8 = 30;

/// ClockMode enumerates the overall state of the clock.
#[derive(PartialEq, Clone, Copy)]
enum ClockMode {
    Vcr,
    Normal,
    Sleep,
    SettingTime(SettingTime),
    Option(OptionMode),
    Align(AlignMode),
}

/// SettingTime enumerates the time setting states.
#[derive(PartialEq, Clone, Copy)]
enum SettingTime {
    Hours,
    Minutes,
    Seconds,
}

impl SettingTime {
    /// next moves to the next time setting state
    fn next(&self) -> Self {
        use SettingTime::*;
        match &self {
            Hours => Minutes,
            Minutes => Seconds,
            Seconds => Hours,
        }
    }
}

/// OptionMode enumerates the configuration option setting modes.
#[derive(PartialEq, Clone, Copy)]
enum OptionMode {
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
            Red => Green,
            Green => Blue,
            Blue => CounterClockwise,
            CounterClockwise => Fade,
            Fade => Red,
        }
    }
}

/// AlignMode enumerates the LED alignment configuration states.  The boolean sets auto-advance mode in each state.
#[derive(PartialEq, Clone, Copy)]
enum AlignMode {
    Hours(bool),
    Minutes(bool),
    Seconds(bool),
}

impl AlignMode {
    /// next moves to the next LED alignment configuration state
    fn next(&self) -> Self {
        use AlignMode::*;
        match *self {
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
        };
    }

    /// Decrement the alignment value, wrapping past zero based on the alignment mode.
    fn decr(&mut self, align_mode: &AlignMode) {
        if self.value > 0 {
            self.value = self.value.wrapping_sub(1);
        } else {
            match align_mode {
                AlignMode::Seconds(_) | AlignMode::Minutes(_) => {
                    self.value = 29;
                }
                AlignMode::Hours(_) => {
                    self.value = 11;
                }
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

struct Button<P> {
    pin: P,
    last_state: bool,
    momentary_override: bool,
}

impl<P> Button<P>
where
    P: InputPin,
{
    fn new(pin: P) -> Self {
        Self {
            pin,
            last_state: false,
            momentary_override: false,
        }
    }

    fn is_pressed(&mut self) -> bool {
        self.pin.is_low().unwrap_or(false)
    }

    fn update(&mut self) -> bool {
        let current = self.is_pressed();
        let changed = current != self.last_state;
        self.last_state = current;
        changed
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    // Current HMS.
    let mut sec_now: u8 = 0;
    let mut min_now: u8 = 0;
    let mut hr_now: u8 = 0;

    // Current HMS ring value (to fade from).
    let mut hr_disp: u8 = 0;
    let mut min_disp: u8 = 0;
    let mut sec_disp: u8 = 0;

    // Copy of last millis() to detect counter wraparound.
    let mut last_time: u16 = 0;

    // Time since a button has been held, in milliseconds (counted by loop passes).
    let mut time_since_button: u8 = 0;

    // Modes:
    let mut factory_reset_disable: bool = false; // To make sure that we don't accidentally reset the settings...

    let mut mode = ClockMode::Vcr;
    let mut align_value = AlignValue { value: 0 };
    let mut align_rate: i8 = 2;
    let mut align_loop_count: u8 = 0;
    let mut starting_option: u8 = 0;

    let mut hold_mode = HoldMode::None;

    let mut controller = DisplayController::new();

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let serial = arduino_hal::default_serial!(dp, pins, 19200);

    #[allow(unused_variables)]
    #[allow(unused_mut)]
    // s_rx unused if serial-sync feature disabled
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
    let mut leds = Leds::new([
        pins.d10.into_floating_input().downgrade(), // 1 - PB2
        pins.a0.into_floating_input().downgrade(),  // 2 - PC0
        pins.a1.into_floating_input().downgrade(),  // 3 - PC1
        pins.a2.into_floating_input().downgrade(),  // 4 - PC2
        pins.a3.into_floating_input().downgrade(),  // 5 - PC3
        pins.d4.into_floating_input().downgrade(),  // 6 - PD4
        pins.d2.into_floating_input().downgrade(),  // 7 - PD2
        pins.d8.into_floating_input().downgrade(),  // 8 - PB0
        pins.d3.into_floating_input().downgrade(),  // 9 - PD3
        pins.d9.into_floating_input().downgrade(),  // 10 - PB1
    ]);

    let hr_ring = crate::leds::LedRing::new(&HR_PINS);
    let min_ring = crate::leds::LedRing::new(&MIN_PINS);
    let sec_ring = crate::leds::LedRing::new(&SEC_PINS);

    let mut plus = Button::new(pins.d5.into_pull_up_input());
    let mut minus = Button::new(pins.d6.into_pull_up_input());
    let mut z = Button::new(pins.d7.into_pull_up_input());

    let mut ep = arduino_hal::Eeprom::new(dp.EEPROM);
    let mut settings = Settings::new(&ep);

    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000, // Copied from examples.
    );

    // Check if RTC is available, and use it to set the time if so.
    let ext_rtc = match ds3231::rtc_get_time(&mut i2c) {
        Ok(v) => {
            (sec_now, min_now, hr_now) = v;
            mode = ClockMode::Normal;
            true
        }
        Err(e) => {
            ufmt::uwriteln!(&mut s_tx, "i2c error in rtc_get_time: {:?}", e).unwrap_infallible();
            false
        }
    };

    unsafe { avr_device::interrupt::enable() };

    loop {
        let mut refresh_time = mode != ClockMode::Normal && mode != ClockMode::Sleep;

        let (plus_changed, minus_changed, z_changed) = (plus.update(), minus.update(), z.update());

        if plus_changed || minus_changed || z_changed {
            // Button change detected

            mode = ClockMode::Normal; // End once any buttons have been pressed...
            time_since_button = 0;

            if !plus.is_pressed() && !plus.last_state {
                // "+" Button was pressed previously, and was just released!

                if plus.momentary_override {
                    plus.momentary_override = false;
                    // Ignore this transition if it was part of a hold sequence.
                } else if mode == ClockMode::Sleep {
                    mode = ClockMode::Normal;
                } else {
                    match mode {
                        ClockMode::Align(align_mode) => {
                            if align_mode.is_auto_advance() {
                                if align_rate < 2 {
                                    align_rate = align_rate.wrapping_add(1);
                                }
                            } else {
                                align_value.incr(&align_mode);
                            }
                        }
                        ClockMode::Option(option_mode) => {
                            if option_mode == OptionMode::Red && settings.hr_bright < 62 {
                                settings.hr_bright = settings.hr_bright.wrapping_add(2);
                            }
                            if option_mode == OptionMode::Green && settings.min_bright < 62 {
                                settings.min_bright = settings.min_bright.wrapping_add(2);
                            }
                            if option_mode == OptionMode::Blue && settings.sec_bright < 62 {
                                settings.sec_bright = settings.sec_bright.wrapping_add(2);
                            }
                            if option_mode == OptionMode::CounterClockwise {
                                settings.ccw = false;
                            }
                            if option_mode == OptionMode::Fade {
                                settings.fade_mode = true;
                            }
                        }
                        ClockMode::SettingTime(setting_time) => match setting_time {
                            SettingTime::Hours => {
                                hr_now = hr_now.wrapping_add(1);
                                if hr_now > 11 {
                                    hr_now = 0;
                                }
                            }
                            SettingTime::Minutes => {
                                min_now = min_now.wrapping_add(1);
                                if min_now > 59 {
                                    min_now = 0;
                                }
                            }
                            SettingTime::Seconds => {
                                sec_now = sec_now.wrapping_add(1);
                                if sec_now > 59 {
                                    sec_now = 0;
                                }
                            }
                        },
                        ClockMode::Vcr | ClockMode::Sleep => {}
                        ClockMode::Normal => {
                            // Brightness control mode
                            settings.main_bright = settings.main_bright.wrapping_add(1);
                            if settings.main_bright > 8 {
                                settings.main_bright = 1;
                            }
                        }
                    }
                }
            }

            if !minus.is_pressed() && !minus.last_state {
                // "-" Button was pressed and just released!

                mode = ClockMode::Normal; // End once any buttons have been pressed...
                time_since_button = 0;

                if minus.momentary_override {
                    minus.momentary_override = false;
                    // Ignore this transition if it was part of a hold sequence.
                } else if mode == ClockMode::Sleep {
                    mode = ClockMode::Normal;
                } else {
                    match mode {
                        ClockMode::Align(align_mode) => {
                            if align_mode.is_auto_advance() {
                                if align_rate > -3 {
                                    align_rate = align_rate.wrapping_sub(1);
                                }
                            } else {
                                align_value.decr(&align_mode);
                            }
                        }
                        ClockMode::Option(option_mode) => {
                            if option_mode == OptionMode::Red && settings.hr_bright > 1 {
                                settings.hr_bright = settings.hr_bright.wrapping_sub(2);
                            }
                            if option_mode == OptionMode::Green && settings.min_bright > 1 {
                                settings.min_bright = settings.min_bright.wrapping_sub(2);
                            }
                            if option_mode == OptionMode::Blue && settings.sec_bright > 1 {
                                settings.sec_bright = settings.sec_bright.wrapping_sub(2);
                            }
                            if option_mode == OptionMode::CounterClockwise {
                                settings.ccw = true;
                            }
                            if option_mode == OptionMode::Fade {
                                settings.fade_mode = false;
                            }
                        }
                        ClockMode::SettingTime(setting_time) => match setting_time {
                            SettingTime::Hours => {
                                hr_now = if hr_now > 0 {
                                    hr_now.wrapping_sub(1)
                                } else {
                                    11
                                }
                            }
                            SettingTime::Minutes => {
                                min_now = if min_now > 0 {
                                    min_now.wrapping_sub(1)
                                } else {
                                    59
                                }
                            }
                            SettingTime::Seconds => {
                                sec_now = if sec_now > 0 {
                                    sec_now.wrapping_sub(1)
                                } else {
                                    59
                                }
                            }
                        },
                        ClockMode::Vcr | ClockMode::Sleep => {}
                        ClockMode::Normal => {
                            // Normal brightness adjustment mode
                            settings.main_bright = if settings.main_bright > 1 {
                                settings.main_bright.wrapping_sub(1)
                            } else {
                                8
                            }
                        }
                    }
                }
            }

            if !z.is_pressed() && !z.last_state {
                // "Z" Button was pressed and just released!

                mode = ClockMode::Normal; // End once any buttons have been pressed...
                time_since_button = 0;

                if z.momentary_override {
                    z.momentary_override = false;
                    // Ignore this transition if it was part of a hold sequence.
                } else {
                    match mode {
                        ClockMode::Align(ref mut align_mode) => {
                            *align_mode = align_mode.next();
                            align_value.reset();
                            align_rate = 2;
                        }
                        ClockMode::Option(ref mut option_mode) => {
                            *option_mode = option_mode.next();
                            starting_option = 0;
                        }
                        ClockMode::SettingTime(ref mut setting_time) => {
                            *setting_time = setting_time.next();
                        }
                        ClockMode::Vcr => {}
                        ClockMode::Normal => {
                            mode = ClockMode::Sleep;
                        }
                        ClockMode::Sleep => {
                            mode = ClockMode::Normal;
                        }
                    }
                }
            }
        }

        // The next block detects and deals with the millis() rollover.
        // This introduces an error of about 1 s every 50 days.
        //
        // (If you have the standard quartz timebase, this will not dominate the inaccuracy.
        // If you have the optional RTC, this error will be corrected next time we read the
        // time from the RTC.)
        let millis_copy = millis();
        if millis_copy < last_time {
            last_time = 0;
        }
        let time_delta_ms = millis_copy - last_time;

        if time_delta_ms >= 1000 {
            last_time = last_time.wrapping_add(1000);

            // Check to see if any buttons are being held down:
            hold_mode = match (plus.is_pressed(), minus.is_pressed(), z.is_pressed()) {
                (false, false, false) => {
                    // No buttons are pressed.
                    factory_reset_disable = true;

                    if time_since_button < 250 {
                        time_since_button = time_since_button.wrapping_add(1);
                    }
                    // 10 s after last button released...
                    if time_since_button == 10 && settings.has_changed_since_last_save() {
                        settings.save(&mut ep);
                    }
                    HoldMode::None
                }
                (true, true, false) => {
                    // "+" and "-" are pressed down. "Z" is up.
                    // We are holding for alignment mode.
                    match hold_mode {
                        HoldMode::Align(x) => HoldMode::Align(x + 1),
                        _ => HoldMode::Align(1),
                    }
                }
                (true, false, true) => {
                    // "+" and "Z" are pressed down. "-" is up.
                    // We are holding for option setting mode.
                    match hold_mode {
                        HoldMode::Option(x) => HoldMode::Option(x + 1),
                        _ => HoldMode::Option(1),
                    }
                }
                (false, false, true) => {
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
                    // Override momentary-action of switches
                    // since we've detected a hold-down condition.
                    plus.momentary_override = true;
                    minus.momentary_override = true;

                    // Hold + and - for 3 s AT POWER ON to restore factory settings.
                    if !factory_reset_disable {
                        settings = Settings::default();
                        settings.save(&mut ep);
                        leds.all_off(); // Blink LEDs off to indicate restoring data
                        arduino_hal::delay_ms(100);
                        mode = ClockMode::Normal;
                    } else if let ClockMode::Align(_) = mode {
                        mode = ClockMode::Normal;
                    } else {
                        mode = ClockMode::Align(AlignMode::Hours(true));
                        align_value.reset();
                        align_rate = 2;
                    }
                }

                HoldMode::Option(3) => {
                    // Override momentary-action of switches
                    // since we've detected a hold-down condition.
                    plus.momentary_override = true;
                    z.momentary_override = true;

                    if let ClockMode::Option(_) = mode {
                        mode = ClockMode::Normal;
                        // Save options if exiting option mode!
                        settings.save(&mut ep);
                        leds.all_off(); // Blink LEDs off to indicate saving data
                        arduino_hal::delay_ms(100);
                    } else {
                        mode = ClockMode::Option(OptionMode::Red);
                        starting_option = 0;
                    }
                }

                HoldMode::TimeSet(3) => {
                    // Override momentary-action of switches
                    // since we've detected a hold-down condition.
                    z.momentary_override = true;

                    if mode != ClockMode::Normal {
                        // If we were in any of these modes, let's now return us to normalcy.
                        // IF we are exiting time-setting mode, save the time to the RTC, if present:
                        if let ClockMode::SettingTime(_) = mode {
                            if ext_rtc {
                                ds3231::rtc_set_time(&mut i2c, hr_now, min_now, sec_now);
                                leds.all_off(); // Blink LEDs off to indicate saving time
                                arduino_hal::delay_ms(100);
                            }
                        }

                        if let ClockMode::Option(_) = mode {
                            // Save options if exiting option mode!
                            settings.save(&mut ep);
                            leds.all_off(); // Blink LEDs off to indicate saving data
                            arduino_hal::delay_ms(100);
                        }

                        mode = ClockMode::Normal;
                    } else {
                        // Go to setting mode IF and ONLY IF we were in regular-clock-display mode.
                        mode = ClockMode::SettingTime(SettingTime::Hours); // Start with HOURS in setting mode.
                    }
                }

                _ => {}
            }

            // Note: this section could act funny if buttons are held for 256 or more seconds.
            // So... um... don't do that.

            sec_now = sec_now.wrapping_add(1);

            if sec_now > 59 {
                sec_now = 0;
                min_now = min_now.wrapping_add(1);

                // Do not check RTC time, if we are in time-setting mode.
                if !matches!(mode, ClockMode::SettingTime(_)) && ext_rtc {
                    // Check value at RTC ONCE PER MINUTE, if enabled.
                    if let Ok((seconds, minutes, hours)) = ds3231::rtc_get_time(&mut i2c) {
                        // IF time is off by MORE than two seconds, then correct the displayed time.
                        // Otherwise, DO NOT update the time, as it may be a sampling error
                        // rather than an actual offset.
                        // Skip checking if minutes == 0 to avoid distraction during 12:00:00 rollover,
                        // unless this is the first time running after reset.

                        let updatetime = if (minutes != 0) && (min_now != 0) {
                            // Values read from RTC
                            let temptime1: i16 = 3600i16
                                .wrapping_mul(hours as i16)
                                .wrapping_add(60i16.wrapping_mul(minutes as i16))
                                .wrapping_add(seconds as i16);
                            // Internally stored time estimate.
                            let temptime2: i16 = 3600i16
                                .wrapping_mul(hr_now as i16)
                                .wrapping_add(60i16.wrapping_mul(min_now as i16))
                                .wrapping_add(sec_now as i16);

                            if temptime1 > temptime2 {
                                temptime1.wrapping_sub(temptime2) > 2
                            } else {
                                temptime2.wrapping_sub(temptime1) > 2
                            }
                        } else {
                            // !ext_rtc indicates this has run before.
                            !ext_rtc
                        };

                        if updatetime {
                            sec_now = seconds;
                            min_now = minutes;
                            hr_now = hours;

                            // Convert 24-hour mode to 12-hour mode
                            if hr_now > 11 {
                                hr_now = hr_now.wrapping_sub(12);
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

            if let ClockMode::Align(align_mode) = mode {
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

                sec_disp = align_value.value().wrapping_add(15); // Offset by 30 s to project *shadow* in the right place.
                if sec_disp > 29 {
                    sec_disp = sec_disp.wrapping_sub(30);
                }
                min_disp = sec_disp;
                hr_disp = align_value.value().wrapping_add(6); // Offset by 6 h to project *shadow* in the right place.

                if hr_disp > 11 {
                    hr_disp = hr_disp.wrapping_sub(12);
                }
            } else if let ClockMode::Option(option_mode) = mode {
                // Option setting mode

                if starting_option < START_OPT_TIME_LIMIT {
                    align_loop_count = align_loop_count.wrapping_add(1); // Borrowing a counter variable...

                    if align_loop_count > 3 {
                        align_loop_count = 0;
                        starting_option = starting_option.wrapping_add(1);

                        if option_mode == OptionMode::Red {
                            // Red (upper) ring color balance
                            hr_disp = hr_disp.wrapping_add(1);
                            if hr_disp > 11 {
                                hr_disp = 0;
                            }
                        }
                        if option_mode == OptionMode::Green {
                            // Green (middle) ring color balance
                            min_disp = min_disp.wrapping_add(1);
                            if min_disp > 29 {
                                min_disp = 0;
                            }
                        }
                        if option_mode == OptionMode::Blue {
                            // Blue (lower) ring color balance
                            sec_disp = sec_disp.wrapping_add(1);
                            if sec_disp > 29 {
                                sec_disp = 0;
                            }
                        }
                        if option_mode == OptionMode::CounterClockwise
                            || option_mode == OptionMode::Fade
                        {
                            // CW vs CCW or fade mode
                            starting_option = START_OPT_TIME_LIMIT; // Exit this loop
                        }
                    }
                }

                if starting_option >= START_OPT_TIME_LIMIT {
                    if option_mode == OptionMode::CounterClockwise {
                        min_disp = min_disp.wrapping_add(1);
                        if min_disp > 29 {
                            min_disp = 0;
                        }
                        sec_disp = min_disp;
                    } else {
                        // Regular time display
                        controller.update_time(hr_now, min_now, sec_now, settings.ccw);
                    }
                }
            } else {
                // Regular clock display
                controller.update_time(hr_now, min_now, sec_now, settings.ccw);
            }
        }

        controller.fades = {
            let mut f = crate::leds::Fades {
                sec_next: 0,
                sec_disp: 63,
                min_next: 0,
                min_disp: 63,
                hr_next: 0,
                hr_disp: 63,
            };

            if let ClockMode::SettingTime(setting_time) = mode {
                f.hr_disp = 5;
                f.min_disp = 5;
                f.sec_disp = 5;

                match setting_time {
                    SettingTime::Hours => {
                        f.hr_disp = TEMP_FADE;
                    }
                    SettingTime::Minutes => {
                        f.min_disp = TEMP_FADE;
                    }
                    SettingTime::Seconds => {
                        f.sec_disp = TEMP_FADE;
                    }
                }
            } else if let ClockMode::Align(align_mode) = mode {
                f.hr_disp = 0;
                f.min_disp = 0;
                f.sec_disp = 0;

                match align_mode {
                    AlignMode::Hours(_) => {
                        f.hr_disp = TEMP_FADE;
                    }
                    AlignMode::Minutes(_) => {
                        f.min_disp = TEMP_FADE;
                    }
                    AlignMode::Seconds(_) => {
                        f.sec_disp = TEMP_FADE;
                    }
                }
            } else if let ClockMode::Option(option_mode) = mode {
                f.hr_disp = 0;
                f.min_disp = 0;
                f.sec_disp = 0;
                if starting_option < START_OPT_TIME_LIMIT {
                    if option_mode == OptionMode::Red {
                        f.hr_disp = TEMP_FADE;
                    }
                    if option_mode == OptionMode::Green {
                        f.min_disp = TEMP_FADE;
                    }
                    if option_mode == OptionMode::Blue {
                        f.sec_disp = TEMP_FADE;
                    }
                    if option_mode == OptionMode::CounterClockwise {
                        f.sec_disp = TEMP_FADE;
                        f.min_disp = TEMP_FADE;
                    }
                } else {
                    f.hr_disp = TEMP_FADE;
                    f.min_disp = TEMP_FADE;
                    f.sec_disp = TEMP_FADE;

                    if option_mode == OptionMode::CounterClockwise {
                        f.hr_disp = 0;
                    } else {
                        f.normal(time_delta_ms, settings.fade_mode, sec_now, min_now);
                    }
                }
            } else {
                f.normal(time_delta_ms, settings.fade_mode, sec_now, min_now);
            }
            f
        };

        // Update the LEDs with the ring state.
        controller.render(&mut leds, &settings, &hr_ring, &min_ring, &sec_ring);

        // Can this sync be tried only once per second?
        #[cfg(feature = "serial-sync")]
        if let Some(v) = pc_sync::get_pc_time(&mut s_rx) {
            (hr_now, min_now, sec_now) = v;

            // Print confirmation
            ufmt::uwriteln!(s_tx, "Clock synced at: {}:{}:{}", hr_now, min_now, sec_now)
                .unwrap_infallible();

            if ext_rtc {
                ds3231::rtc_set_time(&mut i2c, hr_now, min_now, sec_now);
            }
        }
    }
}
