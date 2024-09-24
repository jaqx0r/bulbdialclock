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
//! Synchonize time from a host PC.

use arduino_hal::prelude::*;

 // time sync to PC is HEADER followed by unix time_t as ten ascii digits
const TIME_MSG_LEN: usize = 11;

 // Header tag for serial time sync message
const TIME_HEADER: u8 = 255;

type SerialReader = arduino_hal::usart::UsartReader<
    arduino_hal::pac::USART0,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Input, arduino_hal::hal::port::PD0>,
    arduino_hal::hal::port::Pin<arduino_hal::hal::port::mode::Output, arduino_hal::hal::port::PD1>,
>;

/// Try to read a time from the serial port, returning either a parsed time or nothing.
pub fn get_pc_time(s_rx: &mut SerialReader) -> Option<(u8, u8, u8)> {
    // if time sync available from serial port, update time and return true
    match s_rx.read() {
        Ok(TIME_HEADER) => {
            // read unix time from serial port, header byte and ten ascii digits
            let mut pctime: u32 = 0;
            for _ in 0..TIME_MSG_LEN {
                match s_rx.read() {
                    Ok(c) => {
                        if let Some(d) = char::from(c).to_digit(10) {
                            pctime = pctime.wrapping_mul(10).wrapping_add(d);
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
            let hr_now: u8 = match ((pctime / 60 / 60) % 12).try_into() {
                Ok(v) => v,
                Err(_) => return None,
            };
            Some((hr_now, min_now, sec_now))
        }
        _ => None,
    }
}
