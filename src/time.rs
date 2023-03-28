// Time functions based on Arduino Time.h
// https://github.com/michaelmargolis/arduino_time/blob/master/Time/Time.cpp
// SAFETY: All unsafe global access, just like the original :D

use crate::rtc::millis;

// Arduino time_t is an unsigned long (32 bit)
// https://forum.arduino.cc/t/combining-int-and-time-t/907182
static mut sysTime: u32 = 0;
static mut prevMillis: u32 = 0;
static mut nextSyncTime: u32 = 0;

const syncInterval: u32 = 300;

#[derive(PartialEq, Clone, Copy)]
pub enum timeStatus_t {
    timeNotSet,
    timeSet,
}

static mut status: timeStatus_t = timeStatus_t::timeNotSet;

pub fn now() -> u32 {
    while (millis() - unsafe { prevMillis } >= 1000) {
        unsafe { sysTime += 1 };
        unsafe { prevMillis += 1000 };
    }
    unsafe { sysTime }
}

pub fn setTime(t: u32) {
    unsafe { sysTime = t };
    unsafe { nextSyncTime = t + syncInterval };
    unsafe { status = timeStatus_t::timeSet };
    unsafe { prevMillis = millis() };
}

pub fn timeStatus() -> timeStatus_t {
    now(); // required to actually update the status
    unsafe { status }
}

// fakes
pub fn hour() -> u32 {
    0
}

pub fn minute() -> u32 {
    0
}

pub fn second() -> u32 {
    0
}

pub fn weekday() -> u32 {
    0
}

pub fn month() -> u32 {
    0
}

pub fn day() -> u32 {
    0
}
