// Based on https://blog.rahix.de/005-avr-hal-millis/
// License assumed to be MIT based on https://github.com/Rahix/avr-hal/blob/main/examples/arduino-uno/src/bin/uno-millis.rsm
#![feature(abi_avr_interrupt)]

use core::cell;

const PRESCALER: u32 = 1024;
const TIMER_COUNTS: u32 = 125;
const MILLIS_INCREMENT: u32 = PRESCALER * TIMER_COUNTS / 16000;

static MILLIS_COUNTER: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

#[avr_device::interrupt(atmega168)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MILLIS_INCREMENT);
    })
}

pub fn millis() -> u32 {
    avr_device::interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
}

// https://blog.rahix.de/005-avr-hal-millis/f
pub fn init_tc0(tc0: arduino_hal::pac::TC0) {
    // Set overflow behaviour of the timer in TCCR0A to Clear Timer on Compare mode.
    // Use TIMER0_COMPA interrupt as a result.
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    // Set the overflow maximum for CTC mode in OCR0A.
    tc0.ocr0a.write(|w| unsafe { w.bits(TIMER_COUNTS as u8) });
    // Configure prescaling in TCCR0B.  Arduino chooses 64, here support more options.
    tc0.tccr0b.write(|w| match PRESCALER {
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
