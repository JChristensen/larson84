// scanner class
// for a bit different appearance, overlap turns the next LED on before
// turning the previous one off. set msOverlap to zero to disable the effect.

#include <avr/sleep.h>
#include <JC_Button.h>          // https://github.com/JChristensen/JC_Button
#include <movingAvg.h>          // https://github.com/JChristensen/movingAvg
#include "classes.h"

movingAvg avgVcc(6);

enum scannerType_t {larson, left, right};

struct ScannerParameters
{
    uint32_t interval;
    uint32_t overlap;
    scannerType_t type;
};

class Scanner
{
    public:
        Scanner(Button *btn, HB_LED *led, uint8_t regEnablePin, uint32_t interval=250, uint32_t overlap=0)
            : m_btn(btn), m_hbLED(led), m_regEnable(regEnablePin), m_msInterval(interval), m_msOverlap(overlap),
              m_scanDirection(true), m_type(larson) {}
        void begin();
        void run();

    private:
        void goToSleep();
        int readVcc();
        uint8_t dec2bcd(uint8_t n);

        // states for the state machine
        enum states_t {wait, overlap, next, showVcc1, showVcc2, sleep};

        // constants
        static constexpr uint32_t
            m_msVccInterval{1000},      // milliseconds between Vcc reads
            m_msDisplayMax{10000},      // max time to display voltage on LEDs
            m_msLongPress{1000};        // long button press
        static constexpr int
            m_minVcc{3000};             // sleep the mcu if Vcc falls below this value

        // class variables
        states_t m_state;
        Button *m_btn;
        HB_LED *m_hbLED;
        uint8_t m_regEnable;
        uint32_t m_msInterval;          // milliseconds between changing the LED pattern
        uint32_t m_msOverlap;           // milliseconds overlap between patterns
        uint8_t m_pattern;              // LED pattern
        uint8_t m_prevPattern;          // previous LED pattern
        bool m_scanDirection;
        uint32_t m_msLast;              // last time we changed LEDs
        uint32_t m_msLastVcc;           // last time we read Vcc
        uint32_t m_msDisplay;           // voltage display timer
        int m_vcc;                      // current Vcc moving average value
        uint8_t m_bcdVcc;               // variable to hold the BCD voltage display values
        uint32_t m_ms;                  // current value from millis()
        scannerType_t m_type;           // scanner type
};

void Scanner::begin()
{
    pinMode(m_regEnable, OUTPUT);
    digitalWrite(m_regEnable, HIGH);
    m_pattern = (m_type == right) ? 0x80 : 0x01;
    m_prevPattern = m_pattern;
    ADCSRA = _BV(ADEN);         // enable the ADC
    DDRA = 0xff;                // set all PORTA pins as outputs
    PORTA = 0x00;               // all LEDs off
    m_btn->begin();
    avgVcc.begin();
    m_hbLED->begin();
    m_hbLED->disable();
}

void Scanner::run()
{
    m_ms = millis();
    m_btn->read();
    m_hbLED->run();

    // check to see if it's time to read Vcc
    if (m_ms - m_msLastVcc >= m_msVccInterval)
    {
        m_msLastVcc = m_ms;
        m_vcc = avgVcc.reading(readVcc());
        // sleep if the regulator can't keep the voltage up
        if (m_vcc < m_minVcc) m_state = sleep;
    }

    switch (m_state)
    {
        // wait until time to change the LED pattern, watch for switch presses
        case wait:
            // display Vcc on the LEDs
            if (m_btn->wasReleased())
            {
                m_state = showVcc1;
                m_msDisplay = m_ms;
                m_bcdVcc = dec2bcd(m_vcc / 100);    // most significant two digits
                PORTA = m_bcdVcc;
                m_bcdVcc = dec2bcd(m_vcc % 100);    // least significant two digits
            }
            // display battery voltage on the LEDs
            else if (m_btn->pressedFor(m_msLongPress))
            {
                m_state = showVcc1;
                m_msDisplay = m_ms;
                digitalWrite(m_regEnable, LOW);
                PORTA = 0x00;
                while (!m_btn->wasReleased()) m_btn->read();
                delay(50);      // some Vcc settling time
                int vBat = readVcc();
                digitalWrite(m_regEnable, HIGH);
                m_bcdVcc = dec2bcd(vBat / 100);     // most significant two digits
                PORTA = m_bcdVcc;
                m_bcdVcc = dec2bcd(vBat % 100);     // least significant two digits
            }
            else if (m_ms - m_msLast >= m_msInterval - m_msOverlap)
            {
                // overlap: turn on both current and previous LED patterns.
                if (m_msOverlap > 0)
                {
                    m_state = overlap;
                    PORTA = m_pattern | m_prevPattern;
                }
                else    // no overlap
                {
                    m_state = next;
                }
            }
            break;

        // wait until the end of the overlap period
        case overlap:
            if (m_ms - m_msLast >= m_msInterval) m_state = next;
            break;

        // display the current pattern
        case next:
            m_state = wait;
            m_msLast = m_ms;
            PORTA = m_pattern;

            // calculate the next pattern
            m_prevPattern = m_pattern;
            switch (m_type)
            {
                case larson:
                    if (m_scanDirection)
                    {
                        m_pattern *= 2;
                        if (m_pattern == 0x80) m_scanDirection = !m_scanDirection;
                    }
                    else
                    {
                        m_pattern /= 2;
                        if (m_pattern == 0x01) m_scanDirection = !m_scanDirection;
                    }
                    break;

                case left:
                    m_pattern = (m_pattern == 0x80) ? 0x01 : m_pattern * 2;
                    break;

                case right:
                    m_pattern = (m_pattern == 0x01) ? 0x80 : m_pattern / 2;
                    break;
            }
            break;

        // display the most significant digits of the voltage on the LEDs
        case showVcc1:
            if (m_btn->wasReleased() || m_ms - m_msDisplay > m_msDisplayMax)
            {
                m_state = showVcc2;
                m_msDisplay = m_ms;
                PORTA = m_bcdVcc;
            }
            // long press here to sleep the mcu
            else if (m_btn->pressedFor(m_msLongPress))
            {
                m_state = sleep;
            }
            break;

        // display the least significant digits of the voltage on the LEDs
        case showVcc2:
            if (m_btn->wasReleased() || m_ms - m_msDisplay > m_msDisplayMax)
            {
                m_state = wait;
            }
            break;

        // go to sleep. no way out except reset
        case sleep:
            m_hbLED->disable();
            PORTA = 0x00;
            digitalWrite(m_regEnable, LOW);
            goToSleep();
            break;
    }
}

void Scanner::goToSleep()
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    byte adcsra = ADCSRA;                       // save ADCSRA
    ADCSRA &= ~_BV(ADEN);                       // disable ADC
    cli();                                      // stop interrupts to ensure the BOD timed sequence executes as required
    uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);    // turn off the brown-out detector
    uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);
    MCUCR = mcucr1;
    MCUCR = mcucr2;
    sei();                                      // ensure interrupts enabled so we can wake up again
    sleep_cpu();                                // go to sleep
    sleep_disable();                            // wake up here
    ADCSRA = adcsra;                            // restore ADCSRA
}

// read 1.1V reference against AVcc
// from http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
// returns Vcc in millivolts
int Scanner::readVcc()
{
    ADMUX = _BV(MUX5) | _BV(MUX0);
    delay(2);                                   // Vref settling time
    ADCSRA |= _BV(ADSC);                        // start conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);      // wait for it to complete
    return 1126400L / ADC;                      // calculate AVcc in mV (1.1 * 1000 * 1024)
}

// Decimal-to-BCD conversion
uint8_t Scanner::dec2bcd(uint8_t n)
{
    return n + 6 * (n / 10);
}
