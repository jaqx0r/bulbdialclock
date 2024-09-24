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
//! DS3231 RTC interface

use arduino_hal::prelude::*;

// 104 is the DS3231 RTC device address
const RTC_ADDRESS: u8 = 104;

/// Set the time on the onboard DS3231.
pub fn rtc_set_time(i2c: &mut arduino_hal::I2c, hour_in: u8, minute_in: u8, second_in: u8) {
    macro_rules! bcd_encode {
        ($v:expr) => {{
            let t = $v.wrapping_div(10);
            let o = $v.wrapping_sub(t).wrapping_mul(10);
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

/// Get the time from the onboard DS3231.
pub fn rtc_get_time(i2c: &mut arduino_hal::I2c) -> Result<(u8, u8, u8), arduino_hal::i2c::Error> {
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
