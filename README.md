# Bulbdial Clock Firmware

This is a port of the Arduino sketch for the Bulbdial Clock sold by
http://evilmadscience.com.

It is licensed under the same license as the original: the GPL-3, with the
exception of [src/timer.rs](src/timer.rs) which is (presumed to be) MIT.

This was a learning project -- I wanted to learn both Rust and embedded
programming, and I had the clock, so this seemed like a great idea before I
began.  It was a very steep learning curve.


# References

https://wiki.evilmadscientist.com/Bulbdial is the home page for the Bulbdial
Clock kit, with links to the original source code, the hardware designs,
schematics, and design prose.  Definitely read the article
https://www.evilmadscientist.com/2010/on-the-design-of-the-bulbdial-clock/ to
understand this device and re-read the section on Charlieplexing.

For type conversion from the Arduino C to Rust integers, I used
https://learn.sparkfun.com/tutorials/data-types-in-arduino/all which indicates
the size and signedness of Arduino C types.

Thanks to the Embedded Rust book, the regular Rust book, [Cliffle's Learn Rust
the Dangerous Way](https://cliffle.com/p/dangerust/1/), and of course `Rahix`
for the [`avr-hal`](https://github.com/Rahix/avr-hal/) crate.

`millis()` in rtc.rs from https://blog.rahix.de/005-avr-hal-millis/ .
Crossreferencing `millis()` with Arduino Time.h from https://github.com/michaelmargolis/arduino_time/blob/master/Time/ to make sure I understood what was happening and which methods from the original I could discard.


For conversion of the LED manipulation code,
https://docs.arduino.cc/hacking/software/PortManipulation and
https://docs.arduino.cc/hacking/hardware/PinMapping168 , though I definitely
didn't understand either Charlieplexing or the Arduino port bitbanging code
until quite late in the game. https://en.wikipedia.org/wiki/Charlieplexing
explains the technique used for lighting the LEDs.

I cared deeply about binary size because the device has only 16k of storage,
and using `panic-serial` used up most of this.  Debugging this without `panic-serial` is difficult
difficult, but I managed with merely `ufmt::uwriteln!` in the right places in
the end.

I managed to squeeze bytes out by using the `let match` forms and changing mode
varibles into `enum`s, which was very satisfying, and helped refactor the code
into something I could understand :D I could get more out by using the explicit
`wrapped_*` arithmetic instead of leaving the bounds-checking default in the
`dev` build.

