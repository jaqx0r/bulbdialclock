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
#![allow(non_upper_case_globals, non_camel_case_types, non_snake_case)]

mod rtc;
mod time;
use crate::rtc::*;
use crate::time::*;
use arduino_hal::prelude::*;
use core::mem;

/*
EEPROM variables that are saved:  7

* Brightness setting (range: 1-8)  Default: 8   (Fully bright)

* Red brightness  (range: 0-63)    Default: 20
* Green brightness (range: 0-63)   Default: 63
* Blue brightness (range: 0-63)    Default: 63

* Time direction (Range: 0,1)      Default: 0  (Clockwise)
* Fade style (Range: 0,1)         Default: 1  (Fade enabled)

* Alignment mode                  Default: 0

*/

// "Factory" default configuration can be configured here:
const MainBrightDefault: u8 = 8;

const RedBrightDefault: u8 = 63; // Use 63, default, for kits with monochrome LEDs!
const GreenBrightDefault: u8 = 63;
const BlueBrightDefault: u8 = 63;

const CCWDefault: u8 = 0;
const FadeModeDefault: u8 = 1;

const AlignModeDefault: u8 = 0;

const TIME_MSG_LEN: usize = 11; // time sync to PC is HEADER followed by unix time_t as ten ascii digits
const TIME_HEADER: u8 = 255; // Header tag for serial time sync message

// The buttons are located at D5, D6, & D7.
const buttonmask: u8 = 224;

const tempfade: u8 = 63;

fn delayTime(time: u8) {
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

fn getPCtime(s_rx: &mut SerialReader) -> bool {
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
                        //setTime(pctime)
                        return true;
                    }
                    _ => return false,
                };
            }
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

fn printDigits(s_tx: &mut SerialWriter, digits: u8) {
    // utility function for digital clock display: prints preceding colon and leading 0
    ufmt::uwrite!(s_tx, ":").unwrap();
    if (digits < 10) {
        ufmt::uwrite!(s_tx, "0").unwrap();
    }
    ufmt::uwrite!(s_tx, "{}", digits).unwrap();
}

fn digitalClockDisplay(s_tx: &mut SerialWriter) {
    // digital clock display of current date and time
    ufmt::uwrite!(s_tx, "{}", hour());
    printDigits(s_tx, minute());
    printDigits(s_tx, second());
    ufmt::uwriteln!(s_tx, " {} {} {}", weekday(), month(), day()).unwrap();
}

const SecHi: [u8; 30] = [
    2, 3, 4, 5, 6, 1, 3, 4, 5, 6, 1, 2, 4, 5, 6, 1, 2, 3, 5, 6, 1, 2, 3, 4, 6, 1, 2, 3, 4, 5,
];
const SecLo: [u8; 30] = [
    1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
];

const MinHi: [u8; 30] = [
    1, 7, 1, 8, 1, 9, 2, 7, 2, 8, 2, 9, 3, 7, 3, 8, 3, 9, 4, 7, 4, 8, 4, 9, 5, 7, 5, 8, 5, 9,
];
const MinLo: [u8; 30] = [
    7, 1, 8, 1, 9, 1, 7, 2, 8, 2, 9, 2, 7, 3, 8, 3, 9, 3, 7, 4, 8, 4, 9, 4, 7, 5, 8, 5, 9, 5,
];

const HrHi: [u8; 12] = [10, 1, 2, 10, 10, 6, 3, 10, 10, 4, 5, 10];
const HrLo: [u8; 12] = [1, 10, 10, 2, 6, 10, 10, 3, 4, 10, 10, 5];

static mut SecNow: u8 = 0;
static mut MinNow: u8 = 0;
static mut HrNow: u8 = 0;
static mut HrDisp: u8 = 0;
static mut MinDisp: u8 = 0;
static mut SecDisp: u8 = 0;

const EELength: u8 = 7;
static mut EEvalues: [u8; EELength] = [mem::MaybeUninit::<u8>::uninit(); EELength];

// Variables to store brightness of the three LED rings.
static mut HourBright: u8 = 30;
static mut MinBright: u8 = 63;
static mut SecBright: u8 = 63;
static mut MainBright: u8 = 8; // 8 is maximum value.

static mut LastTime: u32 = mem::MaybeUninit::<u32>::uninit();
static mut TimeNow: u32 = mem::MaybeUninit::<u32>::uninit();
static mut TimeSinceButton: u8 = 0;
static mut LastSavedBrightness: u8 = mem::MaybeUninit::<u8>::uninit();

// Modes:
static mut CCW: u8 = 0; // presume clockwise, not counterclockwise
static mut ExtRTC: u8 = 0;
static mut SleepMode: u8 = 0;
static mut FadeMode: u8 = 1; // Presume fading is enabled.

static mut VCRmode: u8 = 1; // In VCR mode, the clock blinks at you because the time hasn't been set yet.  Initially 1 because time is NOT yet set.
static mut FactoryResetDisable: u8 = 0; // To make sure that we don't accidentally reset the settings...

static mut SettingTime: u8 = 0; // Normally 0.
                                // 1: hours, 2: minutes, 3: seconds, 4: not setting time

static mut AlignMode: u8 = 0; // Normally 0.
static mut OptionMode: u8 = 0; // NOrmally 0
static mut AlignValue: u8 = 0;
static mut AlignRate: i8 = 2;

static mut AlignLoopCount: u8 = 0;
static mut StartingOption: u8 = 0;

static mut HoldTimeSet: u8 = 0;
static mut HoldOption: u8 = 0;
static mut HoldAlign: u8 = 0;

static mut MomentaryOverridePlus: u8 = 0;
static mut MomentaryOverrideMinus: u8 = 0;
static mut MomentaryOverrideZ: u8 = 0;

static mut prevtime: u32 = mem::MaybeUninit::<u32>::uninit();
static mut millisCopy: u32 = mem::MaybeUninit::<u32>::uninit();

static mut SecNext: u8 = mem::MaybeUninit::<u8>::uninit();
static mut MinNext: u8 = mem::MaybeUninit::<u8>::uninit();
static mut HrNext: u8 = mem::MaybeUninit::<u8>::uninit();
static mut h0: u8 = mem::MaybeUninit::<u8>::uninit();
static mut h1: u8 = mem::MaybeUninit::<u8>::uninit();
static mut h2: u8 = mem::MaybeUninit::<u8>::uninit();
static mut h3: u8 = mem::MaybeUninit::<u8>::uninit();
static mut h4: u8 = mem::MaybeUninit::<u8>::uninit();
static mut h5: u8 = mem::MaybeUninit::<u8>::uninit();
static mut l0: u8 = mem::MaybeUninit::<u8>::uninit();
static mut l1: u8 = mem::MaybeUninit::<u8>::uninit();
static mut l2: u8 = mem::MaybeUninit::<u8>::uninit();
static mut l3: u8 = mem::MaybeUninit::<u8>::uninit();
static mut l4: u8 = mem::MaybeUninit::<u8>::uninit();
static mut l5: u8 = mem::MaybeUninit::<u8>::uninit();
static mut d0: u8 = mem::MaybeUninit::<u8>::uninit();
static mut d1: u8 = mem::MaybeUninit::<u8>::uninit();
static mut d2: u8 = mem::MaybeUninit::<u8>::uninit();
static mut d3: u8 = mem::MaybeUninit::<u8>::uninit();
static mut d4: u8 = mem::MaybeUninit::<u8>::uninit();
static mut d5: u8 = mem::MaybeUninit::<u8>::uninit();
static mut HrFade1: u8 = mem::MaybeUninit::<u8>::uninit();
static mut HrFade2: u8 = mem::MaybeUninit::<u8>::uninit();
static mut MinFade1: u8 = mem::MaybeUninit::<u8>::uninit();
static mut MinFade2: u8 = mem::MaybeUninit::<u8>::uninit();
static mut SecFade1: u8 = mem::MaybeUninit::<u8>::uninit();
static mut SecFade2: u8 = mem::MaybeUninit::<u8>::uninit();

fn ApplyDefaults() {
    /*
     * Brightness setting (range: 1-8)  Default: 8  (Fully bright)
     * Red brightness  (range: 0-63)    Default: 20
     * Green brightness (range: 0-63)   Default: 63
     * Blue brightness (range: 0-63)    Default: 63
     * Time direction (Range: 0,1)      Default: 0  (Clockwise)
     * Fade style (Range: 0,1)          Default: 1  (Fade enabled)
     */

    MainBright = MainBrightDefault;
    HourBright = RedBrightDefault;
    MinBright = GreenBrightDefault;
    SecBright = BlueBrightDefault;
    CCW = CCWDefault;
    FadeMode = FadeModeDefault;
}

fn EEReadSettings(eeprom: arduino_hal::pac::EEPROM) {
    // TODO: Detect ANY bad values, not just 255.
    let mut detectBad: u8 = 0;
    let mut value: u8 = 255;

    value = eeprom.read_byte(0);
    if value > 8 {
        // MainBright has a maximum possible value of 8.
        detectBad = 1
    } else {
        MainBright = value
    }
    if (value == 0) {
        MainBright = 1; // Turn back on when power goes back on-- don't leave it dark.
    }
    value = eeprom.read_byte(1);
    if (value > 63) {
        detectBad = 1;
    } else {
        HourBright = value;
    }

    value = eeprom.read_byte(2);
    if (value > 63) {
        detectBad = 1;
    } else {
        MinBright = value;
    }

    value = eeprom.read_byte(3);
    if (value > 63) {
        detectBad = 1;
    } else {
        SecBright = value;
    }

    value = eeprom.read_byte(4);
    if (value > 1) {
        detectBad = 1;
    } else {
        CCW = value;
    }

    value = eeprom.read_byte(5);
    if (value == 255) {
        detectBad = 1;
    } else {
        FadeMode = value;
    }

    if (detectBad) {
        ApplyDefaults();
    }

    LastSavedBrightness = MainBright;
}

fn EESaveSettings(eeprom: arduino_hal::pac::EEPROM) {
    //EEPROM.write(Addr, Value);

    // Careful if you use  this function: EEPROM has a limited number of write
    // cycles in its life.  Good for human-operated buttons, bad for automation.

    eeprom.write_byte(0, MainBright);
    eeprom.write_byte(1, HourBright);
    eeprom.write_byte(2, MinBright);
    eeprom.write_byte(3, SecBright);
    eeprom.write_byte(4, CCW);
    eeprom.write_byte(5, FadeMode);

    LastSavedBrightness = MainBright;

    // Optional: Blink LEDs off to indicate when we're writing to the EEPROM
    // AllLEDsOff();
    // delay(100);
}

fn normalTimeDisplay() {
    SecDisp = (SecNow + 30); // Offset by 30 s to project *shadow* in the right place.
    if (SecDisp > 59) {
        SecDisp -= 60;
    }
    SecDisp >>= 1; // Divide by two, since there are 30 LEDs, not 60.

    SecNext = SecDisp + 1;
    if (SecNext > 29) {
        SecNext = 0;
    }
    MinDisp = (MinNow + 30); // Offset by 30 m to project *shadow* in the right place.
    if (MinDisp > 59) {
        MinDisp -= 60;
    }
    MinDisp >>= 1; // Divide by two, since there are 30 LEDs, not 60.

    MinNext = MinDisp + 1;
    if (MinNext > 29) {
        MinNext = 0;
    }
    HrDisp = (HrNow + 6); // Offset by 6 h to project *shadow* in the right place.

    if (HrDisp > 11) {
        HrDisp -= 12;
    }
    HrNext = HrDisp + 1;
    if (HrNext > 11) {
        HrNext = 0;
    }
}

fn normalFades() {
    if (FadeMode) {
        // Normal time display
        if (SecNow & 1)
        // ODD time
        {
            SecFade2 = (63 * (millisCopy - LastTime) / 1000);
            SecFade1 = 63 - SecFade2;
        }

        if (MinNow & 1)
        // ODD time
        {
            if (SecNow == 59) {
                MinFade2 = SecFade2;
                MinFade1 = SecFade1;
            }
        }

        if (MinNow == 59)
        // End of the hour, only:
        {
            if (SecNow == 59) {
                HrFade2 = SecFade2;
                HrFade1 = SecFade1;
            }
        }
    } else {
        // no fading

        HrFade1 = tempfade;
        MinFade1 = tempfade;
        SecFade1 = tempfade;
    }
}

// 104 is the DS3231 RTC device address
const RTC_ADDRESS: u8 = 104;

fn RTCsetTime(i2c: &mut arduino_hal::I2c, hourIn: u8, minuteIn: u8, secondIn: u8) {
    let ts: u8 = secondIn / 10;
    let os: u8 = secondIn - ts * 10;
    let ss: u8 = (ts << 4) + os;

    let tm: u8 = minuteIn / 10;
    let om: u8 = minuteIn - tm * 10;
    let sm: u8 = (tm << 4) | om;

    let th: u8 = hourIn / 10;
    let oh: u8 = hourIn - th * 10;
    let sh: u8 = (th << 4) | oh;

    let buf: [u8; 3] = [ss, sm, sh];
    i2c.write(RTC_ADDRESS, &buf).unwrap(); // TODO handle result.
}

fn RTCgetTime(i2c: &mut arduino_hal::I2c) -> u8 {
    // Read out time from RTC module, if present
    // send request to receive data starting at register 0

    let mut status: u8 = 0;

    let mut buf: [u8; 3] = [0, 0, 0];
    if let Err(_) = i2c.read(RTC_ADDRESS, &buf) {
        return 0;
    }

    let mut seconds: i16;
    let mut minutes: i16;
    let mut hours: i16;
    let mut temptime1: u16;
    let mut temptime2: u16;
    let mut updatetime: u8 = 0;

    {
        status = 1;
        seconds = buf[0]; // get seconds
        minutes = buf[1]; // get minutes
        hours = buf[2]; // get hours
    }

    // IF time is off by MORE than two seconds, then correct the displayed time.
    // Otherwise, DO NOT update the time, it may be a sampling error rather than an
    // actual offset.
    // Skip checking if minutes == 0. -- the 12:00:00 rollover is distracting,
    // UNLESS this is the first time running after reset.

    // if (ExtRTC) is equivalent to saying,  "if this has run before"

    if (status) {
        seconds = (((seconds & 0b11110000) >> 4) * 10 + (seconds & 0b00001111)); // convert BCD to decimal
        minutes = (((minutes & 0b11110000) >> 4) * 10 + (minutes & 0b00001111)); // convert BCD to decimal
        hours = (((hours & 0b00110000) >> 4) * 10 + (hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)

        // Optional: report time::
        // Serial.print(hours); Serial.print(":"); Serial.print(minutes); Serial.print(":"); Serial.println(seconds);

        if ((minutes) && (MinNow)) {
            temptime1 = 3600 * hours + 60 * minutes + seconds; // Values read from RTC
            temptime2 = 3600 * HrNow + 60 * MinNow + SecNow; // Internally stored time estimate.

            if (temptime1 > temptime2) {
                if ((temptime1 - temptime2) > 2) {
                    updatetime = 1;
                }
            } else {
                if ((temptime2 - temptime1) > 2) {
                    updatetime = 1;
                }
            }
        }

        if (ExtRTC == 0) {
            updatetime = 1;
        }
        if (updatetime) {
            SecNow = seconds;
            MinNow = minutes;
            HrNow = hours;

            // Convert 24-hour mode to 12-hour mode
            if (HrNow > 11) {
                HrNow -= 12;
            }
        }
    }

    return status;
}

fn IncrAlignVal() {
    AlignValue += 1;

    if (AlignMode < 5)
    // seconds or minutes
    {
        if (AlignValue > 29) {
            AlignValue = 0;
        }
    } else {
        if (AlignValue > 11) {
            AlignValue = 0;
        }
    }
}

fn DecrAlignVal() {
    if (AlignValue > 0) {
        AlignValue -= 1;
    } else if (AlignMode < 5)
    // seconds or minutes
    {
        AlignValue = 29;
    } else
    // hours
    {
        AlignValue = 11;
    }
}

const StartOptTimeLimit: u8 = 30;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let serial = arduino_hal::default_serial!(dp, pins, 19200);
    let (mut s_rx, mut s_tx) = serial.split();

    init_tc0(dp.TC0);

    //setTime(0);

    // Converted from original by correlating the Arduino C PORTx and DDRx bit manipulation against
    // https://docs.arduino.cc/hacking/hardware/PinMapping168
    let led1 = pins.pb2.into_output(); // PB2
    let led2 = pins.pc0.into_output(); // PC0
    let led3 = pins.pc1.into_output(); // PC1
    let led4 = pins.pc2.into_output(); // PC2
    let led5 = pins.pc3.into_output(); // PC3
    let led6 = pins.pd4.into_output(); // PD4
    let led7 = pins.pd2.into_output(); // PD2
    let led8 = pins.pb0.into_output(); // PB0
    let led9 = pins.pd3.into_output(); // PD3
    let led10 = pins.pb1.into_output(); // PB1

    let TakeHigh = |LEDLine: u8| match LEDLine {
        1 => led1.set_high(),
        2 => led2.set_high(),
        3 => led3.set_high(),
        4 => led4.set_high(),
        5 => led6.set_high(),
        6 => led6.set_high(),
        7 => led7.set_high(),
        8 => led8.set_high(),
        9 => led9.set_high(),
        10 => led10.set_high(),
    };
    let TakeLow = |LEDLine: u8| match LEDLine {
        1 => led1.set_low(),
        2 => led2.set_low(),
        3 => led3.set_low(),
        4 => led4.set_low(),
        5 => led5.set_low(),
        6 => led6.set_low(),
        7 => led7.set_low(),
        8 => led8.set_low(),
        9 => led9.set_low(),
        10 => led10.set_low(),
    };
    let AllLEDsOff = || {
        for i in 1..10 {
            TakeLow(i);
        }
    };

    // Pull-up resistors for buttons
    let (plus, minus, z) = (
        pins.d5.into_pull_up_input(),
        pins.d6.into_pull_up_input(),
        pins.d7.into_pull_up_input(),
    );

    TimeNow = millis();

    let mut ep = arduino_hal::pac::EEPROM::new(dp.EEPROM);
    EEReadSettings(ep);

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
    ExtRTC = RTCgetTime(&mut i2c);
    // If no RTC is found, no attempt will be made to use it thereafter.

    if (ExtRTC) {
        // If time is already set from the RTC...
        VCRmode = 0;
    }

    unsafe { avr_device::interrupt::enable() };

    loop {
        let mut HighLine: u8;
        let mut LowLine: u8;
        let mut RefreshTime = AlignMode + SettingTime + OptionMode;

        let (plus_copy, minus_copy, z_copy) = (plus.is_low(), minus.is_low(), z.is_low());

        if (plus_copy != plus_last || minus_copy != minus_last || z_copy != z_last)
        // Button change detected
        {
            VCRmode = 0; // End once any buttons have been pressed...
            TimeSinceButton = 0;

            if (!plus_copy && plus_last) {
                // "+" Button was pressed previously, and was just released!

                if (MomentaryOverridePlus) {
                    MomentaryOverridePlus = 0;
                    // Ignore this transition if it was part of a hold sequence.
                } else {
                    if (SleepMode) {
                        SleepMode = 0;
                    } else {
                        if (AlignMode) {
                            if (AlignMode & 1)
                            // Odd mode:
                            {
                                if (AlignRate < 2) {
                                    AlignRate += 1;
                                }
                            } else {
                                IncrAlignVal(); // Even mode:
                            }
                        } else if (OptionMode) {
                            if (OptionMode == 1) {
                                if (HourBright < 62) {
                                    HourBright += 2;
                                }
                            }
                            if (OptionMode == 2) {
                                if (MinBright < 62) {
                                    MinBright += 2;
                                }
                            }
                            if (OptionMode == 3) {
                                if (SecBright < 62) {
                                    SecBright += 2;
                                }
                            }
                            if (OptionMode == 4) {
                                CCW = 0;
                            }
                            if (OptionMode == 5) {
                                FadeMode = 1;
                            }
                        } else if (SettingTime) {
                            if (SettingTime == 1) {
                                HrNow += 1;
                                if (HrNow > 11) {
                                    HrNow = 0;
                                }
                            }
                            if (SettingTime == 2) {
                                MinNow += 1;
                                if (MinNow > 59) {
                                    MinNow = 0;
                                }
                            }
                            if (SettingTime == 3) {
                                SecNow += 1;
                                if (SecNow > 59) {
                                    SecNow = 0;
                                }
                            }
                        } else {
                            // Brightness control mode
                            MainBright += 1;
                            if (MainBright > 8) {
                                MainBright = 1;
                            }
                        }
                    }
                }
            }

            if (!minus_copy && minus_last) {
                // "-" Button was pressed and just released!

                VCRmode = 0; // End once any buttons have been pressed...
                TimeSinceButton = 0;

                if (MomentaryOverrideMinus) {
                    MomentaryOverrideMinus = 0;
                    // Ignore this transition if it was part of a hold sequence.
                } else {
                    if (SleepMode) {
                        SleepMode = 0;
                    } else {
                        if (AlignMode) {
                            if (AlignMode & 1)
                            // Odd mode:
                            {
                                if (AlignRate > -3) {
                                    AlignRate -= 1;
                                }
                            } else {
                                DecrAlignVal(); // Even mode:
                            }
                        } else if (OptionMode) {
                            if (OptionMode == 1) {
                                if (HourBright > 1) {
                                    HourBright -= 2;
                                }
                            }
                            if (OptionMode == 2) {
                                if (MinBright > 1) {
                                    MinBright -= 2;
                                }
                            }
                            if (OptionMode == 3) {
                                if (SecBright > 1) {
                                    SecBright -= 2;
                                }
                            }
                            if (OptionMode == 4) {
                                CCW = 1;
                            }
                            if (OptionMode == 5) {
                                FadeMode = 0;
                            }
                        } else if (SettingTime) {
                            if (SettingTime == 1) {
                                if (HrNow > 0) {
                                    HrNow -= 1;
                                } else {
                                    HrNow = 11;
                                }
                            }
                            if (SettingTime == 2) {
                                if (MinNow > 0) {
                                    MinNow -= 1;
                                } else {
                                    MinNow = 59;
                                }
                            }
                            if (SettingTime == 3) {
                                if (SecNow > 0) {
                                    SecNow -= 1;
                                } else {
                                    SecNow = 59;
                                }
                            }
                        } else {
                            // Normal brightness adjustment mode
                            if (MainBright > 1) {
                                MainBright -= 1;
                            } else {
                                MainBright = 8;
                            }
                        }
                    }
                }
            }

            if (!z_copy && z_last) {
                // "Z" Button was pressed and just released!

                VCRmode = 0; // End once any buttons have been pressed...
                TimeSinceButton = 0;

                if (MomentaryOverrideZ) {
                    MomentaryOverrideZ = 0;
                    // Ignore this transition if it was part of a hold sequence.
                } else {
                    if (AlignMode) {
                        AlignMode += 1;
                        if (AlignMode > 6) {
                            AlignMode = 1;
                        }
                        AlignValue = 0;
                        AlignRate = 2;
                    } else if (OptionMode) {
                        OptionMode += 1;
                        StartingOption = 0;

                        if (OptionMode > 5) {
                            OptionMode = 1;
                        }
                    } else if (SettingTime) {
                        SettingTime += 1;
                        if (SettingTime > 3) {
                            SettingTime = 1;
                        }
                    } else {
                        if (SleepMode == 0) {
                            SleepMode = 1;
                        } else {
                            SleepMode = 0;
                        }
                    }
                }
            }
        }

        (plus_last, minus_last, z_last) = (plus_copy, minus_copy, z_copy);
        millisCopy = millis();

        // The next if statement detects and deals with the millis() rollover.
        // This introduces an error of up to  1 s, about every 50 days.
        //
        // (If you have the standard quartz timebase, this will not dominate the inaccuracy.
        // If you have the optional RTC, this error will be corrected next time we read the
        // time from the RTC.)

        if (millisCopy < LastTime) {
            LastTime = 0;
        }
        if ((millisCopy - LastTime) >= 1000) {
            LastTime += 1000;

            // Check to see if any buttons are being held down:

            if (plus.is_high() & minus.is_high() && z.is_high()) {
                // No buttons are pressed.
                // Reset the variables that check to see if buttons are being held down.

                HoldTimeSet = 0;
                HoldOption = 0;
                HoldAlign = 0;
                FactoryResetDisable = 1;

                if (TimeSinceButton < 250) {
                    TimeSinceButton += 1;
                }
                if (TimeSinceButton == 10)
                // 10 s after last button released...
                {
                    if (LastSavedBrightness != MainBright) {
                        EESaveSettings();
                    }
                }
            } else {
                // Note which buttons are being held down

                if (plus.is_low() & minus.is_low())
                // "+" and "-" are pressed down. "Z" is up.
                {
                    HoldAlign += 1; // We are holding for alignment mode.
                    HoldOption = 0;
                    HoldTimeSet = 0;
                }
                if (plus.is_low() & z.is_low())
                // "+" and "Z" are pressed down. "-" is up.
                {
                    HoldOption += 1; // We are holding for option setting mode.
                    HoldTimeSet = 0;
                    HoldAlign = 0;
                }
                if (z.is_low())
                // "Z" is pressed down. "+" and "-" are up.
                {
                    HoldTimeSet += 1; // We are holding for time setting mode.
                    HoldOption = 0;
                    HoldAlign = 0;
                }
            }

            if (HoldAlign == 3) {
                MomentaryOverridePlus = 1; // Override momentary-action of switches
                MomentaryOverrideMinus = 1; // since we've detected a hold-down condition.

                OptionMode = 0;
                SettingTime = 0;

                // Hold + and - for 3 s AT POWER ON to restore factory settings.
                if (FactoryResetDisable == 0) {
                    ApplyDefaults();
                    EESaveSettings();
                    AllLEDsOff(); // Blink LEDs off to indicate restoring data
                    arduino_hal::delay_ms(100);
                } else {
                    if (AlignMode) {
                        AlignMode = 0;
                    } else {
                        AlignMode = 1;
                        AlignValue = 0;
                        AlignRate = 2;
                    }
                }
            }

            if (HoldOption == 3) {
                MomentaryOverridePlus = 1;
                MomentaryOverrideZ = 1;
                AlignMode = 0;
                SettingTime = 0;

                if (OptionMode) {
                    OptionMode = 0;
                    EESaveSettings(); // Save options if exiting option mode!
                    AllLEDsOff(); // Blink LEDs off to indicate saving data
                    arduino_hal::delay_ms(100);
                } else {
                    OptionMode = 1;
                    StartingOption = 0;
                }
            }

            if (HoldTimeSet == 3) {
                MomentaryOverrideZ = 1;

                if (AlignMode + OptionMode + SettingTime) {
                    // If we were in any of these modes, let's now return us to normalcy.
                    // IF we are exiting time-setting mode, save the time to the RTC, if present:
                    if (SettingTime && ExtRTC) {
                        RTCsetTime(&mut i2c, HrNow, MinNow, SecNow);
                        AllLEDsOff(); // Blink LEDs off to indicate saving time
                        arduino_hal::delay_ms(100);
                    }

                    if (OptionMode) {
                        EESaveSettings(); // Save options if exiting option mode!
                        AllLEDsOff(); // Blink LEDs off to indicate saving data
                        arduino_hal::delay_ms(100);
                    }

                    SettingTime = 0;
                } else {
                    // Go to setting mode IF and ONLY IF we were in regular-clock-display mode.
                    SettingTime = 1; // Start with HOURS in setting mode.
                }

                AlignMode = 0;
                OptionMode = 0;
            }

            // Note: this section could act funny if you hold the buttons for 256 or more seconds.
            // So... um... don't do that.  :P

            SecNow += 1;

            if (SecNow > 59) {
                SecNow = 0;
                MinNow += 1;

                if ((SettingTime == 0) && ExtRTC) {
                    // Check value at RTC ONCE PER MINUTE, if enabled.
                    RTCgetTime(&mut i2c); // Do not check RTC time, if we are in time-setting mode.
                }
            }

            if (MinNow > 59) {
                MinNow = 0;
                HrNow += 1;

                if (HrNow > 11) {
                    HrNow = 0;
                }
            }

            RefreshTime = 1;
        }
        if (RefreshTime) {
            // Calculate which LEDs to light up to give the correct shadows:

            if (AlignMode) {
                if (AlignMode & 1) {
                    // ODD mode, auto-advances

                    let mut AlignRateAbs: u8; // Absolute value of AlignRate

                    if (AlignRate >= 0) {
                        AlignRateAbs = AlignRate + 1;
                    } else {
                        AlignRateAbs = -AlignRate;
                    }

                    // Serial.println(AlignRateAbs,DEC);

                    AlignLoopCount += 1;

                    let mut ScaleRate: u8;
                    if (AlignRateAbs > 2) {
                        ScaleRate = 10;
                    } else if (AlignRateAbs == 2) {
                        ScaleRate = 50;
                    } else {
                        ScaleRate = 250;
                    }

                    if (AlignLoopCount > ScaleRate) {
                        AlignLoopCount = 0;

                        if (AlignRate >= 0) {
                            IncrAlignVal();
                        } else {
                            DecrAlignVal();
                        }
                    }
                }

                SecDisp = (AlignValue + 15); // Offset by 30 s to project *shadow* in the right place.
                if (SecDisp > 29) {
                    SecDisp -= 30;
                }
                MinDisp = SecDisp;
                HrDisp = (AlignValue + 6); // Offset by 6 h to project *shadow* in the right place.

                if (HrDisp > 11) {
                    HrDisp -= 12;
                }
            } else if (OptionMode) {
                // Option setting mode

                if (StartingOption < StartOptTimeLimit) {
                    AlignLoopCount += 1; // Borrowing a counter variable...

                    if (AlignLoopCount > 3) {
                        AlignLoopCount = 0;
                        StartingOption += 1;

                        if (OptionMode == 1)
                        // Red (upper) ring color balance
                        {
                            HrDisp += 1;
                            if (HrDisp > 11) {
                                HrDisp = 0;
                            }
                        }
                        if (OptionMode == 2)
                        // Green (middle) ring color balance
                        {
                            MinDisp += 1;
                            if (MinDisp > 29) {
                                MinDisp = 0;
                            }
                        }
                        if (OptionMode == 3)
                        // Blue (lower) ring color balance
                        {
                            SecDisp += 1;
                            if (SecDisp > 29) {
                                SecDisp = 0;
                            }
                        }
                        if (OptionMode >= 4)
                        // CW vs CCW OR fade mode
                        {
                            StartingOption = StartOptTimeLimit; // Exit this loop
                        }
                    }
                } // end "if (StartingOption < StartOptTimeLimit){}"

                if (StartingOption >= StartOptTimeLimit) {
                    if (OptionMode == 4) {
                        MinDisp += 1;
                        if (MinDisp > 29) {
                            MinDisp = 0;
                        }
                        SecDisp = MinDisp;
                    } else {
                        normalTimeDisplay();
                    }
                }
            } else {
                // Regular clock display

                normalTimeDisplay();
            }

            h3 = HrDisp;
            l3 = HrNext;
            h4 = MinDisp;
            l4 = MinNext;
            h5 = SecDisp;
            l5 = SecNext;

            if (CCW) {
                // Counterclockwise
                if (HrDisp) {
                    h3 = 12 - HrDisp;
                }
                if (HrNext) {
                    l3 = 12 - HrNext;
                }
                if (MinDisp) {
                    h4 = 30 - MinDisp;
                }
                if (MinNext) {
                    l4 = 30 - MinNext;
                }
                if (SecDisp) {
                    h5 = 30 - SecDisp;
                }
                if (SecNext) {
                    l5 = 30 - SecNext;
                }

                // Serial.print(HrDisp,DEC);
                // Serial.print(", ");
                // Serial.println(h3,DEC);
            }

            h0 = HrHi[h3];
            l0 = HrLo[h3];

            h1 = HrHi[l3];
            l1 = HrLo[l3];

            h2 = MinHi[h4];
            l2 = MinLo[h4];

            h3 = MinHi[l4];
            l3 = MinLo[l4];

            h4 = SecHi[h5];
            l4 = SecLo[h5];

            h5 = SecHi[l5];
            l5 = SecLo[l5];
        }

        SecFade2 = 0;
        SecFade1 = 63;

        MinFade2 = 0;
        MinFade1 = 63;

        HrFade2 = 0;
        HrFade1 = 63;

        if (SettingTime)
        // i.e., if (SettingTime is nonzero)
        {
            HrFade1 = 5;
            MinFade1 = 5;
            SecFade1 = 5;

            if (SettingTime == 1)
            // hours
            {
                HrFade1 = tempfade;
            }
            if (SettingTime == 2)
            // minutes
            {
                MinFade1 = tempfade;
            }
            if (SettingTime == 3)
            // seconds
            {
                SecFade1 = tempfade;
            }
        } else if (AlignMode + OptionMode)
        // if either...
        {
            HrFade1 = 0;
            MinFade1 = 0;
            SecFade1 = 0;

            if (AlignMode) {
                if (AlignMode < 3) {
                    SecFade1 = tempfade;
                } else if (AlignMode > 4) {
                    HrFade1 = tempfade;
                } else {
                    MinFade1 = tempfade;
                }
            } else {
                // Must be OptionMode....
                if (StartingOption < StartOptTimeLimit) {
                    if (OptionMode == 1) {
                        HrFade1 = tempfade;
                    }
                    if (OptionMode == 2) {
                        MinFade1 = tempfade;
                    }
                    if (OptionMode == 3) {
                        SecFade1 = tempfade;
                    }
                    if (OptionMode == 4)
                    // CW vs CCW
                    {
                        SecFade1 = tempfade;
                        MinFade1 = tempfade;
                    }
                } else {
                    // No longer in starting mode.

                    HrFade1 = tempfade;
                    MinFade1 = tempfade;
                    SecFade1 = tempfade;

                    if (OptionMode == 4)
                    // CW vs CCW
                    {
                        HrFade1 = 0;
                    } else {
                        normalFades();
                    }
                }
            }
        } else {
            normalFades();
        }

        let mut tempbright: u8 = MainBright;

        if (SleepMode) {
            tempbright = 0;
        }

        if (VCRmode) {
            if (SecNow & 1) {
                tempbright = 0;
            }
        }

        d0 = HourBright * HrFade1 * tempbright >> 7;
        d1 = HourBright * HrFade2 * tempbright >> 7;
        d2 = MinBright * MinFade1 * tempbright >> 7;
        d3 = MinBright * MinFade2 * tempbright >> 7;
        d4 = SecBright * SecFade1 * tempbright >> 7;
        d5 = SecBright * SecFade2 * tempbright >> 7;

        // unsigned long  temp = millis();

        // This is the loop where we actually light up the LEDs:
        let mut i: u8 = 0;
        while (i < 128)
        // 128 cycles: ROUGHLY 39 ms  => Full redraw at about 3 kHz.
        {
            if (d0 > 0) {
                TakeHigh(h0);
                TakeLow(l0);
                delayTime(d0);
                AllLEDsOff();
            }

            if (d1 > 0) {
                TakeHigh(h1);
                TakeLow(l1);
                delayTime(d1);
                AllLEDsOff();
            }

            if (d2 > 0) {
                TakeHigh(h2);
                TakeLow(l2);
                delayTime(d2);
                AllLEDsOff();
            }

            if (d3 > 0) {
                TakeHigh(h3);
                TakeLow(l3);
                delayTime(d3);
                AllLEDsOff();
            }

            if (d4 > 0) {
                TakeHigh(h4);
                TakeLow(l4);
                delayTime(d4);
                AllLEDsOff();
            }

            if (d5 > 0) {
                TakeHigh(h5);
                TakeLow(l5);
                delayTime(d5);
                AllLEDsOff();
            }

            if (MainBright < 8) {
                delayTime((8 - MainBright) << 5);
                delayTime((8 - MainBright) << 5);
                delayTime((8 - MainBright) << 5);
            }

            i += 1;
        }

        /*
        temp = millis() - temp;
        Serial.println(temp,DEC);
         */

        // Can this sync be tried only once per second?
        if (getPCtime(s_rx)) {
            // try to get time sync from pc

            // Set time to that given from PC.
            MinNow = minute();
            SecNow = second();
            HrNow = hour();

            if (HrNow > 11) {
                // Convert 24-hour mode to 12-hour mode
                HrNow -= 12;
            }

            // Print confirmation
            ufmt::uwriteln!(s_tx, "Clock synced at: {}", now());

            if (timeStatus() == timeStatus_t::timeSet) {
                // update clocks if time has been synced

                if (prevtime != now()) {
                    if (ExtRTC) {
                        RTCsetTime(HrNow, MinNow, SecNow);
                    }

                    timeStatus(); // refresh the Date and time properties
                    digitalClockDisplay(s_tx); // update digital clock
                    prevtime = now();
                }
            }
        }
    }
}
