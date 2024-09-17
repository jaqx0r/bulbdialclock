/// A basic implementation of the `millis()` function from Arduino.
///
/// https://www.arduino.cc/reference/en/language/functions/time/millis/
///
/// Uses timer TC0 and one of its interrupts to update a global millisecond
/// counter.  A walkthough of this code is available here:
///
/// Based on https://blog.rahix.de/005-avr-hal-millis/
///
/// License assumed to be MIT based on https://github.com/Rahix/avr-hal/blob/main/examples/arduino-uno/src/bin/uno-millis.rsm
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
const PRESCALER: u32 = 1024;
const TIMER_COUNTS: u32 = 125;

const MILLIS_INCREMENT: u16 = (PRESCALER * TIMER_COUNTS / 16000) as _;

static MILLIS_COUNTER: Mutex<cell::Cell<u16>> = Mutex::new(cell::Cell::new(0));

/// Timer/Counter 0 Compare Match A interrupt service routine.
#[avr_device::interrupt(atmega168)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter.wrapping_add(MILLIS_INCREMENT));
    })
}

/// Return the number of milliseconds counted since `init_tc0()` has been called.
pub fn millis() -> u16 {
    avr_device::interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
}

/// Initialise Timer/Counter 0 for counting milliseconds.
/// Configures the TC0 timer for the interval defined by consts PRESCALER and TIMER_COUNTS (in CTC mode).
// https://blog.rahix.de/005-avr-hal-millis/
// More explanation on the atmega168 timer at https://protostack.com.au/2010/09/timer-interrupts-on-an-atmega168/
pub fn init_tc0(tc0: arduino_hal::pac::TC0) {
    // Set overflow behaviour of the timer in TCCR0A to Clear Timer on Compare mode.
    // Use TIMER0_COMPA interrupt as a result.
    tc0.tccr0a.write(|w| w.wgm0().ctc());

    // Set the overflow maximum for CTC mode in OCR0A.
    tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));

    // Configure prescaling in TCCR0B.  Arduino chooses 64, here support more options.
    tc0.tccr0b.write(|w| match PRESCALER {
        1 => w.cs0().direct(),
        8 => w.cs0().prescale_8(),
        64 => w.cs0().prescale_64(),
        256 => w.cs0().prescale_256(),
        1024 => w.cs0().prescale_1024(),
        _ => panic!(),
    });

    // Enable overflow interrupt in TIMSK0.  From this point on when the timer overflows and interrupts are enabled, the ISR will run.
    tc0.timsk0.write(|w| w.ocie0a().set_bit());

    // Reset the counter.
    avr_device::interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    });
}
