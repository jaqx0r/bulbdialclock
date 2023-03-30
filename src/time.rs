// Time functions based on Arduino Time.h
// https://github.com/michaelmargolis/arduino_time/blob/master/Time/Time.cpp
// SAFETY: All unsafe global access, just like the original :D

use crate::rtc::millis;

// Arduino time_t is an unsigned long (32 bit)
// https://forum.arduino.cc/t/combining-int-and-time-t/907182
static mut SYS_TIME: u32 = 0;
static mut PREV_MILLIS: u32 = 0;
static mut NEXT_SYNC_TIME: u32 = 0;

const SYNC_INTERVAL: u32 = 300;

#[derive(PartialEq, Clone, Copy)]
pub enum TimeStatus {
    TimeNotSet,
    TimeSet,
}

static mut STATUS: TimeStatus = TimeStatus::TimeNotSet;

pub fn now() -> u32 {
    while millis() - unsafe { PREV_MILLIS } >= 1000 {
        unsafe { SYS_TIME += 1 };
        unsafe { PREV_MILLIS += 1000 };
    }
    unsafe { SYS_TIME }
}

pub fn set_time(t: u32) {
    unsafe { SYS_TIME = t };
    unsafe { NEXT_SYNC_TIME = t + SYNC_INTERVAL };
    unsafe { STATUS = TimeStatus::TimeSet };
    unsafe { PREV_MILLIS = millis() };
}

pub fn time_status() -> TimeStatus {
    now(); // required to actually update the status
    unsafe { STATUS }
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
