// Based on https://github.com/Rahix/avr-hal/blob/main/examples/arduino-uno/src/bin/uno-panic.rs
// License MIT

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // disable interrupts - no longer necessary after panic
    avr_device::interrupt::disable();

    // re-get the peripherals so we can access serial
    // SAFETY: we're never returning so stealing the peripherals is ok
    let dp = unsafe { arduino_hal::Peripherals::steal() };
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 19200);

    ufmt::uwriteln!(&mut serial, "panic\r").unwrap();

    if let Some(loc) = info.location() {
        ufmt::uwriteln!(
            &mut serial,
            " at {}:{}:{}\r",
            loc.file(),
            loc.line(),
            loc.column(),
        )
        .unwrap();
    }
    loop {}
}
