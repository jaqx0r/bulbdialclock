// Time functions based on Arduino Time.h
// https://github.com/michaelmargolis/arduino_time/blob/master/Time/Time.cpp

use crate::rtc::millis;

// Arduino time_t is an unsigned long (32 bit)
// https://forum.arduino.cc/t/combining-int-and-time-t/907182
static mut cacheTime: u32 = 0;
static mut sysTime: u32 = 0;
static mut prevMillis: u32 = 0;
static mut nextSyncTime: u32 = 0;

const syncInterval: u32 = 300;

pub enum timeStatus_t {
    timeNotSet,
    timeNeedsSync,
    timeSet,
}

static mut status: timeStatus_t = timeStatus_t::timeNotSet;

pub fn now() -> u32 {
    while (millis() - prevMillis >= 1000) {
        sysTime += 1;
        prevMillis += 1000;
    }
    sysTime
}

pub fn setTime(t: u32) {
    sysTime = t;
    nextSyncTime = t + syncInterval;
    status = timeStatus_t::timeSet;
    prevMillis = millis();
}

pub fn timeStatus() -> timeStatus_t {
    now(); // required to actually update the status
    return status;
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
