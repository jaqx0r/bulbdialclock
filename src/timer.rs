// License assumed to be MIT based on https://github.com/Rahix/avr-hal/blob/main/examples/arduino-uno/src/bin/uno-millis.rsm
//! A basic implementation of the `millis()` function from Arduino.
//!
//! <https://www.arduino.cc/reference/en/language/functions/time/millis/>
//!
//! Uses timer `TC0` and one of its interrupts to update a global millisecond
//! counter.  A walkthough of this code is available at
//! <https://blog.rahix.de/005-avr-hal-millis/>
//!
//! A deep dive into the ATmega168 timer can be found at
//! <https://protostack.com.au/2010/09/timer-interrupts-on-an-atmega168/>

use avr_device::interrupt::Mutex;
use core::cell;

// Possible Values:
//
// ╔═══════════╦══════════════╦═══════════════════╗
// ║ PRESCALER ║ TIMER_COUNTS ║ Overflow Interval ║
// ╠═══════════╬══════════════╬═══════════════════╣
// ║        64 ║          250 ║              1 ms ║
// ║       256 ║          125 ║              2 ms ║
// ║       256 ║          250 ║              4 ms ║
// ║      1024 ║          125 ║              8 ms ║
// ║      1024 ║          250 ║             16 ms ║
// ╚═══════════╩══════════════╩═══════════════════╝
/// Prescaler divider to slow down the tick rate.
const PRESCALER: u32 = 1024;
/// Represents the overflow maximum for Clear Timer on Compare mode.
const TIMER_COUNTS: u32 = 125;

/// How much to increment the millisecond counter by, each overflow interrupt.
const MILLIS_INCREMENT: u16 = (PRESCALER * TIMER_COUNTS / 16000) as _;

// Stores the global millisecond counter.
static MILLIS_COUNTER: Mutex<cell::Cell<u16>> = Mutex::new(cell::Cell::new(0));

/// Timer/Counter 0 Compare Match A interrupt service routine.
///
/// Users must enable `#[feature(abi_avr_interrupt)]` in main.
#[avr_device::interrupt(atmega168)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter.wrapping_add(MILLIS_INCREMENT));
    })
}

/// Return the number of milliseconds counted since [`init_tc0()`] has been called.
pub fn millis() -> u16 {
    avr_device::interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
}

/// Initialise Timer/Counter 0 for counting milliseconds.
///
/// Configures the `TC0` timer for the interval defined by consts [`PRESCALER`]
/// and [`TIMER_COUNTS`] (in Clear Timer on Compare mode).
pub fn init_tc0(tc0: arduino_hal::pac::TC0) {
    // Configure prescaling in `TCCR0B`.  Arduino chooses 64 by default, here
    // we support more options.  This sets the tick rate from the system clock.
    tc0.tccr0b.write(|w| match PRESCALER {
        1 => w.cs0().direct(), // 16MHz, no prescaling
        8 => w.cs0().prescale_8(), // 16MHz/8
        64 => w.cs0().prescale_64(),  // 16MHz/64
        256 => w.cs0().prescale_256(), // 16MHz/256
        1024 => w.cs0().prescale_1024(), // 16MHz/1024
        _ => panic!(),
    });

    // Set the overflow maximum for CTC mode in output compare register
    // `OCR0A`, so that the ISR is triggered when the `TC0`'s tick counter
    // reaches this value.
    tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));

    // Set overflow behaviour of the timer in `TCCR0A` to Clear Timer on
    // Compare mode.  Issues [`TIMER0_COMPA`] interrupt on overflow, and resets
    // the tick counter when it reaches `OCR0A`'s value.
    tc0.tccr0a.write(|w| w.wgm0().ctc());

    // Enable the [`TIMER0_COMPA`] overflow interrupt in `TIMSK0`.  From this point on when the timer
    // overflows and interrupts are enabled, the ISR will run.
    tc0.timsk0.write(|w| w.ocie0a().set_bit());

    // Reset the global millisecond counter.
    avr_device::interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    });
}
