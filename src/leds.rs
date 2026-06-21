use arduino_hal::hal::port::{Pin, Dynamic};
use arduino_hal::port::mode::{Output, Floating, Input};

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
