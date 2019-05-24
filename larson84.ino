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

#include <avr/sleep.h>
#include <JC_Button.h>          // https://github.com/JChristensen/JC_Button
#include <movingAvg.h>          // https://github.com/JChristensen/movingAvg
#include "classes.h"

// timing constants.
// for a bit different appearance, overlap turns the next LED on before
// turning the previous one off. set msOverlap to zero to disable the effect.
const uint32_t
    msInterval(250),            // milliseconds between changing the LED pattern
    msOverlap(0),               // milliseconds overlap between patterns
    msVccInterval(1000);        // milliseconds between Vcc reads

const uint8_t
    HB_LED_PIN(0),              // heartbeat/diagnostic LED
    BUTTON_PIN(1),              // press button to display Vcc
    REG_ENABLE_PIN(2);          // boost regulator enable

Button btn(BUTTON_PIN);
movingAvg avgVcc(6);
HB_LED hb(HB_LED_PIN, 20, 980);

void setup()
{
    pinMode(REG_ENABLE_PIN, OUTPUT);
    digitalWrite(REG_ENABLE_PIN, HIGH);
    ADCSRA = _BV(ADEN);         // enable the ADC
    DDRA = 0xff;                // set all PORTA pins as outputs
    PORTA = 0x00;               // all LEDs off
    btn.begin();
    avgVcc.begin();
    hb.begin();
    hb.disable();
}

enum states_t {wait, overlap, next, showVcc1, showVcc2};    // states for the state machine

void loop()
{
    static states_t state;
    static uint8_t pattern(0x01);       // LED pattern
    static uint8_t prevPattern(0x01);   // previous LED pattern
    static bool scanDirection(true);
    static uint32_t msLast;             // last time we changed LEDs
    static uint32_t msLastVcc;          // last time we read Vcc
    static uint32_t msDisplay;          // voltage display timer
    const uint32_t msDisplayMax(10000); // max time to display voltage
    static int vcc;                     // current Vcc moving average value
    static uint8_t bcdVcc;              // variable to hold the BCD voltage display values
    const int minVcc(3000);             // sleep the mcu if Vcc falls below this value
    uint32_t ms = millis();

    btn.read();
    hb.run();

    // check to see if it's time to read Vcc
    if (ms - msLastVcc >= msVccInterval)
    {
        msLastVcc = ms;
        vcc = avgVcc.reading(readVcc());
        // sleep if the regulator can't keep the voltage up
        if (vcc < minVcc)
        {
            digitalWrite(HB_LED_PIN, LOW);
            digitalWrite(REG_ENABLE_PIN, LOW);
            PORTA = 0x00;
            goToSleep();
        }
    }

    switch (state)
    {
        // wait until time to change the LED pattern
        case wait:
            // display Vcc on the LEDs
            if (btn.wasReleased())
            {
                state = showVcc1;
                msDisplay = ms;
                bcdVcc = dec2bcd(vcc / 100);    // most significant two digits
                PORTA = bcdVcc;
                bcdVcc = dec2bcd(vcc % 100);    // least significant two digits
            }
            // display battery voltage on the LEDs
            else if (btn.pressedFor(1000))
            {
                state = showVcc1;
                msDisplay = ms;
                digitalWrite(REG_ENABLE_PIN, LOW);
                PORTA = 0x00;
                while (!btn.wasReleased()) btn.read();
                delay(50);      // some Vcc settling time
                int vBat = readVcc();
                digitalWrite(REG_ENABLE_PIN, HIGH);
                bcdVcc = dec2bcd(vBat / 100);      // most significant two digits
                PORTA = bcdVcc;
                bcdVcc = dec2bcd(vBat % 100);      // least significant two digits
            }
            else if (ms - msLast >= msInterval - msOverlap)
            {
                // overlap: turn on both current and previous LED patterns.
                if (msOverlap > 0)
                {
                    state = overlap;
                    PORTA = pattern | prevPattern;
                }
                else    // no overlap
                {
                    state = next;
                }
            }
            break;

        // wait until the end of the overlap period
        case overlap:
            if (ms - msLast >= msInterval) state = next;
            break;

        // display the current pattern
        case next:
            state = wait;
            msLast = ms;
            PORTA = pattern;

            // calculate the next pattern
            prevPattern = pattern;
            if (scanDirection)
            {
                pattern *= 2;
                if (pattern == 0x80) scanDirection = !scanDirection;
            }
            else
            {
                pattern /= 2;
                if (pattern == 0x01) scanDirection = !scanDirection;
            }
            break;

        // display the most significant digits of the voltage on the LEDs
        case showVcc1:
            if (btn.wasReleased() || ms - msDisplay > msDisplayMax)
            {
                state = showVcc2;
                msDisplay = ms;
                PORTA = bcdVcc;
            }
            break;

        // display the least significant digits of the voltage on the LEDs
        case showVcc2:
            if (btn.wasReleased() || ms - msDisplay > msDisplayMax)
            {
                state = wait;
            }
            break;
    }
}

// read 1.1V reference against AVcc
// from http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
// returns Vcc in millivolts
int readVcc()
{
    ADMUX = _BV(MUX5) | _BV(MUX0);
    delay(2);                                   // Vref settling time
    ADCSRA |= _BV(ADSC);                        // start conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);      // wait for it to complete
    return 1126400L / ADC;                      // calculate AVcc in mV (1.1 * 1000 * 1024)
}

void goToSleep()
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    //MCUCR &= ~(_BV(ISC01) | _BV(ISC00));        // INT0 on low level
    //GIMSK |= _BV(INT0);                         // enable INT0
    byte adcsra = ADCSRA;                       // save ADCSRA
    ADCSRA &= ~_BV(ADEN);                       // disable ADC
    cli();                                      // stop interrupts to ensure the BOD timed sequence executes as required
    uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);    // turn off the brown-out detector
    uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);
    MCUCR = mcucr1;
    MCUCR = mcucr2;
    sei();                                    // ensure interrupts enabled so we can wake up again
    sleep_cpu();                                // go to sleep
    sleep_disable();                            // wake up here
    ADCSRA = adcsra;                            // restore ADCSRA
}

// Decimal-to-BCD conversion
uint8_t dec2bcd(uint8_t n)
{
    return n + 6 * (n / 10);
}
