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
//! Module for managing the Bulbdial Clock LED display system.
//!
//! This module provides functionality for controlling individual LEDs through
//! Charlieplexing, a technique allowing multiple LEDs to be driven by a limited
//! number of pins.
//!
//! Key structures include:
//! - `DisplayController`: Orchestrates the state and rendering logic for the clock face.
//! - `Leds`: Manages the low-level hardware pin states for the charlieplexed matrix.
//! - `LedRing`: Handles mapping logical LED indices to specific physical pin pairs.
//!
//! Charlieplexing logic is derived from the official Bulbdial documentation and
//! original C source code, specifically utilizing high-impedance (Hi-Z) states
//! to isolate individual LEDs in the matrix.

use arduino_hal::hal::port::{Dynamic, Pin};
use arduino_hal::port::mode::{Floating, Input, Output};

use crate::settings::Settings;

pub struct CharliePin {
    pin: Pin<Input<Floating>, Dynamic>,
}

impl CharliePin {
    pub fn new(pin: Pin<Input<Floating>, Dynamic>) -> Self {
        Self { pin }
    }

    pub fn into_output_high(self) -> Pin<Output, Dynamic> {
        let mut pin = self.pin.into_output();
        pin.set_high();
        pin
    }

    pub fn into_output(self) -> Pin<Output, Dynamic> {
        self.pin.into_output()
    }
}

pub struct Offsets {
    pub disp: u8,
    pub next: u8,
}

pub struct RingOffsets {
    pub hr: Offsets,
    pub min: Offsets,
    pub sec: Offsets,
}

impl RingOffsets {
    pub fn apply_ccw(&mut self) {
        self.hr.disp = 12 - self.hr.disp;
        self.hr.next = 12 - self.hr.next;
        self.min.disp = 30 - self.min.disp;
        self.min.next = 30 - self.min.next;
        self.sec.disp = 30 - self.sec.disp;
        self.sec.next = 30 - self.sec.next;
    }
}

/// Fade multipliers for the hour, minute, and second rings.
pub struct Fades {
    /// Hour ring fade multiplier for the outgoing LED. 0-63
    pub hr_disp: u8,
    /// Hour ring fade multiplier for the incoming LED. 0-63
    pub hr_next: u8,
    /// Minute ring fade multiplier for the outgoing LED. 0-63
    pub min_disp: u8,
    /// Minute ring fade multiplier for the incoming LED. 0-63
    pub min_next: u8,
    /// Second ring fade multiplier for the outgoing LED. 0-63
    pub sec_disp: u8,
    /// Second ring fade multiplier for the incoming LED. 0-63
    pub sec_next: u8,
}

impl Fades {
    /// Compute the normal fade for a given timestamp. Fades set the brightness
    /// multiplier for the incoming and outgoing LED for each ring.
    pub fn normal(&mut self, time_delta_ms: u16, fade_mode: bool, sec_now: u8, min_now: u8) {
        if fade_mode {
            // On odd seconds only, as there are only 30 LEDs
            if sec_now & 1 != 0 {
                self.sec_next = 63u16.wrapping_mul(time_delta_ms).wrapping_div(1000) as u8;
                self.sec_disp = 63u8.wrapping_sub(self.sec_next);
            }

            // End of the minute, on odd minutes only, as there are only 30 LEDs.
            if min_now & 1 != 0 && sec_now == 59 {
                self.min_next = self.sec_next;
                self.min_disp = self.sec_disp;
            }

            // End of the hour, only.
            if min_now == 59 && sec_now == 59 {
                self.hr_next = self.sec_next;
                self.hr_disp = self.sec_disp;
            }
        } else {
            self.hr_disp = 63;
            self.min_disp = 63;
            self.sec_disp = 63;
        }
    }
}

pub struct DisplayController {
    pub hr: u8,
    pub min: u8,
    pub sec: u8,
    pub offsets: RingOffsets,
    pub fades: Fades,
}

impl DisplayController {
    pub fn new() -> Self {
        Self {
            hr: 0,
            min: 0,
            sec: 0,
            offsets: RingOffsets {
                hr: Offsets { disp: 0, next: 0 },
                min: Offsets { disp: 0, next: 0 },
                sec: Offsets { disp: 0, next: 0 },
            },
            fades: Fades {
                hr_disp: 63,
                hr_next: 0,
                min_disp: 63,
                min_next: 0,
                sec_disp: 63,
                sec_next: 0,
            },
        }
    }

    pub fn update_time(&mut self, hr: u8, min: u8, sec: u8, ccw: bool) {
        self.hr = hr;
        self.min = min;
        self.sec = sec;
        // Offsets   by half to project the *shadow* in the right place. There are only
        // 30 leds on the seconds and minutes rings, so they are mapped from 0-59 to
        // 0-29. Hours are mapped from 0-11.
        self.offsets.hr.disp = hr.wrapping_add(6) % 12;
        self.offsets.hr.next = self.offsets.hr.disp.wrapping_add(1) % 12;
        self.offsets.min.disp = (min.wrapping_add(30) % 60).wrapping_div(2);
        self.offsets.min.next = self.offsets.min.disp.wrapping_add(1) % 30;
        self.offsets.sec.disp = (sec.wrapping_add(30) % 60).wrapping_div(2);
        self.offsets.sec.next = self.offsets.sec.disp.wrapping_add(1) % 30;

        if ccw {
            self.offsets.apply_ccw();
        }
    }

    pub fn render(
        &self,
        leds: &mut Leds,
        settings: &Settings,
        hr_ring: &LedRing,
        min_ring: &LedRing,
        sec_ring: &LedRing,
    ) {
        let tempbright: u16 =
            if settings.is_sleep_mode() || (settings.is_vcr_mode() && (self.sec & 1 != 0)) {
                0
            } else {
                settings.main_bright as u16
            };

        #[inline]
        fn calc_delay(bright: u8, disp: u8, tempbright: u16) -> u8 {
            ((bright as u16)
                .wrapping_mul(disp as u16)
                .wrapping_mul(tempbright)
                >> 7) as u8
        }

        let hr_disp_delay = calc_delay(settings.hr_bright, self.fades.hr_disp, tempbright);
        let hr_next_delay = calc_delay(settings.hr_bright, self.fades.hr_next, tempbright);
        let min_disp_delay = calc_delay(settings.min_bright, self.fades.min_disp, tempbright);
        let min_next_delay = calc_delay(settings.min_bright, self.fades.min_next, tempbright);
        let sec_disp_delay = calc_delay(settings.sec_bright, self.fades.sec_disp, tempbright);
        let sec_next_delay = calc_delay(settings.sec_bright, self.fades.sec_next, tempbright);

        for _ in 0..128 {
            if hr_disp_delay > 0 {
                hr_ring.activate(leds, self.offsets.hr.disp, hr_disp_delay);
            }

            if hr_next_delay > 0 {
                hr_ring.activate(leds, self.offsets.hr.next, hr_next_delay);
            }

            if min_disp_delay > 0 {
                min_ring.activate(leds, self.offsets.min.disp, min_disp_delay);
            }

            if min_next_delay > 0 {
                min_ring.activate(leds, self.offsets.min.next, min_next_delay);
            }

            if sec_disp_delay > 0 {
                sec_ring.activate(leds, self.offsets.sec.disp, sec_disp_delay);
            }

            if sec_next_delay > 0 {
                sec_ring.activate(leds, self.offsets.sec.next, sec_next_delay);
            }

            if settings.main_bright < 8 {
                let dt = 8u8.wrapping_sub(settings.main_bright) << 5;
                crate::delay_time(dt);
                crate::delay_time(dt);
                crate::delay_time(dt);
            }
        }
    }
}

pub struct Leds {
    pub pins: [CharliePin; 10],
}

impl Leds {
    pub fn new(pins: [Pin<Input<Floating>, Dynamic>; 10]) -> Self {
        Self {
            pins: pins.map(CharliePin::new),
        }
    }

    /// Activate a LED given a pin pair in the charlieplexed array.  See the
    /// bulbdial schematic
    /// <https://bcdn.evilmadscientist.com/source/beedyschem.pdf> and refer to
    /// the Evil Mad Scientist article on the design of the Bulbdial clock for
    /// more details on charlieplexing:
    /// <https://www.evilmadscientist.com/2010/on-the-design-of-the-bulbdial-clock/>
    /// Per that documentation and the C source code we know that the high
    /// impedance (hi Z) pin mode is performed by setting the pin into input
    /// mode.
    pub fn activate(&mut self, hi: u8, lo: u8, delay: u8) {
        let hi_off = hi.wrapping_sub(1) as usize;
        let lo_off = lo.wrapping_sub(1) as usize;

        // Take ownership of the pins temporarily
        // SAFETY: The `CharliePin`s are restored to the same indices at the end
        // of this function.
        let hi_pin = unsafe { core::ptr::read(&self.pins[hi_off]) };
        let lo_pin = unsafe { core::ptr::read(&self.pins[lo_off]) };

        // Perform the activation
        let hi_output = hi_pin.into_output_high();
        let mut lo_output = lo_pin.into_output();
        lo_output.set_low();

        crate::delay_time(delay);

        // Put them back
        self.pins[hi_off] = CharliePin::new(hi_output.into_floating_input());
        self.pins[lo_off] = CharliePin::new(lo_output.into_floating_input());
    }

    pub fn all_off(&mut self) {
        // all off by default, kept for backwards compatibility in the option setting modes.
    }
}

pub struct LedRing {
    pins: &'static [(u8, u8)],
}

impl LedRing {
    pub const fn new(pins: &'static [(u8, u8)]) -> Self {
        Self { pins }
    }

    pub fn activate(&self, leds: &mut Leds, index: u8, delay: u8) {
        let (hi, lo) = self.pins[index as usize];
        leds.activate(hi, lo, delay);
    }
}
