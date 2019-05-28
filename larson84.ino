// 8-LED Larson Scanner with voltmeter for ATtiny84
// Copyright (C) 2019 by Jack Christensen and licensed
// under GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html//
//
// Tested with Arduino 1.8.9 and the ATTiny Core,
// https://github.com/SpenceKonde/ATTinyCore
//
// Clock: 1MHz internal
// BOD: Disabled
// Pin Mapping: Counterclockwise (like old ATTinyCore and x41-series)
//
// Connect the PORTA pins (Arduino pins 3-10) to eight LEDs through
// appropriate current-limiting resistors.
//
// Press and release the button to read Vcc in millivolts as four BCD digits
// on the LEDs. A long press reads the battery voltage by briefly turning
// off the boost regulator.
//
// The code continuously monitors Vcc and if it drops below 3 volts
// (meaning the battery is exhausted and the boost regulator cannot
// compensate) then the MCU goes into power down sleep mode. A reset
// (and a new battery!) is then required.
//
// Jack Christensen 19May2019

#include "scanner.h"

// pin assignments
constexpr uint8_t
    hbLedPin(0),                // heartbeat/diagnostic LED
    buttonPin(1),               // press button to display Vcc
    regEnablePin(2);            // boost regulator enable

// object definitions
Button btn(buttonPin);
HB_LED hb(hbLedPin, 20, 980);
Scanner scanner(&btn, &hb, regEnablePin);

void setup()
{
    scanner.begin();
}

void loop()
{
    scanner.run();
}
