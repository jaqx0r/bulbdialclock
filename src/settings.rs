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
//! EEPROM settings

/// EEPROM variables that are saved:  7
pub struct Settings {
    /// Brightness setting (range: 1-8)  Default: 8   (Fully bright)
    pub main_bright: u8,

    /// Red brightness  (range: 0-63)    Default: 20 (But initialized to 30 in original!  TODO add feature for monocrhome)
    pub hr_bright: u8,

    /// Green brightness (range: 0-63)   Default: 63
    pub min_bright: u8,

    /// Blue brightness (range: 0-63)    Default: 63
    pub sec_bright: u8,

    /// Time direction (Range: 0,1)      Default: 0  (Clockwise)
    pub ccw: bool,

    /// Fade style (Range: 0,1)         Default: 1  (Fade enabled)
    pub fade_mode: bool,

    // Last brightness saved to EEPROM used to determine if changed.
    pub last_saved_brightness: u8,
}

// "Factory" default configuration can be configured here:
const MAIN_BRIGHT_DEFAULT: u8 = 8;

const RED_BRIGHT_DEFAULT: u8 = 63; // Use 63, default, for kits with monochrome LEDs!
const GREEN_BRIGHT_DEFAULT: u8 = 63;
const BLUE_BRIGHT_DEFAULT: u8 = 63;

const CCW_DEFAULT: bool = false;
const FADE_MODE_DEFAULT: bool = true;

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

/// Read the EEPROM into settings, or return a default if an error occurs during read.
#[must_use]
pub fn eeprom_read_settings(eeprom: &arduino_hal::Eeprom) -> Settings {
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
pub fn eeprom_save_settings(
    eeprom: &mut arduino_hal::Eeprom,
    main_bright: u8,
    hour_bright: u8,
    min_bright: u8,
    sec_bright: u8,
    ccw: bool,
    fade_mode: bool,
) -> u8 {
    // Careful if you use  this function: EEPROM has a limited number of write
    // cycles in its life.  Good for human-operated buttons, bad for automation.

    eeprom.write_byte(0, main_bright);
    eeprom.write_byte(1, hour_bright);
    eeprom.write_byte(2, min_bright);
    eeprom.write_byte(3, sec_bright);
    eeprom.write_byte(4, if ccw { 1 } else { 0 });
    eeprom.write_byte(5, if fade_mode { 1 } else { 0 });

    main_bright
}
