// scanner class.
// eight different scan patterns are defined.

#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <JC_Button.h>          // https://github.com/JChristensen/JC_Button
#include <movingAvg.h>          // https://github.com/JChristensen/movingAvg
#include "classes.h"

enum scannerType_t {scanLarson, scanLeft, scanRight, scanRandom};

// struct to define a single scan pattern.
// for a bit different appearance, overlap turns the next LED on before
// turning the previous one off. set overlap to zero to disable the effect.
struct ScannerParameters
{
    uint32_t interval;          // delay between changing the LEDs
    uint32_t overlap;           // overlap time between two LED states
    scannerType_t type;         // type of scan
};

// define the eight scan patterns that can be selected
ScannerParameters params[8]
    {{250, 0, scanLarson}, {250, 0, scanLeft}, {250, 0, scanRight}, {200, 0, scanRandom},
     {250, 100, scanLarson}, {250, 100, scanLeft}, {250, 100, scanRight}, {200, 100, scanRandom}};

class Scanner
{
    public:
        Scanner(Button *btn, HB_LED *led, uint8_t regEnablePin)
            : m_btn(btn), m_hbLED(led), m_regEnable(regEnablePin), avgVcc(6) {}
        void begin();                   // setup
        void run();                     // run the state machine

    private:
        void init();                    // initialize scanner parameters and other variables
        void goToSleep();               // sleep the mcu
        int readVcc();                  // read the supply voltage
        void calibrate();               // calibrate the internal RC oscillator
        uint8_t dec2bcd(uint8_t n);     // convert a regular integer to binard coded decimal

        // states for the state machine
        enum states_t {checkSet, setMode, checkButton, waitNext, waitOverlap, next,
                       displayBatt, displayReg, displayVoltage1, displayVoltage2, sleep};

        // constants
        static constexpr uint32_t
            m_msVccInterval{1000},      // milliseconds between Vcc reads
            m_msDisplayMax{6000},       // max time to display voltage on LEDs
            m_msLongPress{1000},        // long button press
            m_msSetTimeout{5000};       // set mode timeout
        static constexpr uint8_t
            m_maxMode{7};               // mode number must be between 0 and 7

        // class variables
        Button *m_btn;                  // select button
        HB_LED *m_hbLED;                // heartbeat LED
        uint8_t m_regEnable;            // boost regulator enable pin
        movingAvg avgVcc;
        states_t m_state;               // state machine state
        uint32_t m_msInterval;          // milliseconds between changing the LED pattern
        uint32_t m_msOverlap;           // milliseconds overlap between patterns
        uint8_t m_pattern;              // LED pattern
        uint8_t m_prevPattern;          // previous LED pattern
        bool m_scanDirection;           // for larson scan type
        uint32_t m_msLast;              // last time we changed LEDs
        uint32_t m_msLastVcc;           // last time we read Vcc
        uint32_t m_msDisplay;           // voltage display timer
        int m_vcc;                      // current Vcc moving average value
        int m_minVcc;                   // sleep if Vcc falls below this value
        uint8_t m_bcdVcc;               // variable to hold the BCD voltage display values
        uint32_t m_ms;                  // current value from millis()
        scannerType_t m_type;           // scanner type
        uint8_t m_mode;                 // copy of m_mode_ee from EEPROM
        static EEMEM uint8_t m_mode_ee; // copy of m_mode in EEPROM
        bool m_inOverlap;               // currently in an overlap interval
};

EEMEM uint8_t Scanner::m_mode_ee;       // copy of m_mode in EEPROM

// set up the hardware and initialize the scanner
void Scanner::begin()
{
    pinMode(m_regEnable, OUTPUT);
    digitalWrite(m_regEnable, HIGH);
    ADCSRA = _BV(ADEN);                 // enable the ADC
    DDRA = 0xff;                        // set all PORTA pins as outputs
    PORTA = 0x00;                       // all LEDs off
    m_btn->begin();
    avgVcc.begin();
    m_hbLED->begin();
    calibrate();
    randomSeed(readVcc());
    // get mode, set to zero if not valid
    m_mode = eeprom_read_byte(&m_mode_ee);
    if (m_mode > m_maxMode)
    {
        m_mode = 0;
        eeprom_update_byte(&m_mode_ee, m_mode);
    }
    // set scan parameters
    m_scanDirection = true;
    m_msInterval = params[m_mode].interval;
    m_msOverlap = params[m_mode].overlap;
    m_type = params[m_mode].type;
    init();
}

// run the state machine
void Scanner::run()
{
    m_ms = millis();
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
        // check to see if the user wants to change the mode
        case checkSet:
            m_btn->read();
            if (m_btn->isPressed())
            {
                m_state = setMode;
                PORTA = _BV(m_mode);
                while (!m_btn->wasReleased()) m_btn->read();
            }
            else
            {
                m_state = checkButton;
            }
            break;

        // set the mode (indicates the mode 0-7 on the LEDs)
        case setMode:
            m_btn->read();
            if (m_btn->wasReleased())
            {
                if (++m_mode > m_maxMode) m_mode = 0;
                PORTA = _BV(m_mode);
            }
            else if (m_btn->pressedFor(m_msLongPress))
            {
                m_state = checkButton;
                eeprom_update_byte(&m_mode_ee, m_mode);
                PORTA = 0x00;
                while (!m_btn->wasReleased()) m_btn->read();
                init();
            }
            else if (m_ms - m_btn->lastChange() >= m_msSetTimeout)
            {
                m_state = checkButton;
                //eeprom_update_byte(&m_mode_ee, m_mode);
                PORTA = 0x00;
                init();
            }
            break;

        // watch for button presses, this is the main running state
        case checkButton:
            // display battery voltage on the LEDs
            m_btn->read();
            if (m_btn->wasReleased())
            {
                m_state = displayReg;
            }
            // display regulator voltage on the LEDs
            else if (m_btn->pressedFor(m_msLongPress))
            {
                m_state = displayBatt;
            }
            // wait for the next time to change the LEDs
            // if in an overlap interval or if there is no overlap, wait for the next interval end time
            else if (m_inOverlap || m_msOverlap == 0)
            {
                m_state = waitNext;
            }
            // else wait for the overlap start time
            else
            {
                m_state = waitOverlap;
            }
            break;

        // wait for end of LED interval
        case waitNext:
            if (m_ms - m_msLast >= m_msInterval)
            {
                m_state = next;
                m_inOverlap = false;
            }
            else
            {
                m_state = checkButton;
            }
            break;

        // wait for the beginning of the overlap period
        case waitOverlap:
            m_state = checkButton;
            if (m_ms - m_msLast >= m_msInterval - m_msOverlap)
            {
                m_inOverlap = true;
                PORTA = m_pattern | m_prevPattern;
            }
            break;

        // display the current pattern
        case next:
            m_state = checkButton;
            m_msLast = m_ms;
            PORTA = m_pattern;
            m_prevPattern = m_pattern;

            // calculate the next pattern
            switch (m_type)
            {
                case scanLarson:
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

                case scanLeft:
                    m_pattern = (m_pattern == 0x80) ? 0x01 : m_pattern * 2;
                    break;

                case scanRight:
                    m_pattern = (m_pattern == 0x01) ? 0x80 : m_pattern / 2;
                    break;

                case scanRandom:
                    m_pattern = _BV(random(8));
            }
            break;

        // display the battery voltage
        case displayBatt:
            m_state = displayVoltage1;
            m_msDisplay = m_ms;
            PORTA = 0x00;
            digitalWrite(m_regEnable, LOW);
            while (!m_btn->wasReleased()) m_btn->read();
            delay(50);                              // some Vcc settling time
            {
                int vBat = readVcc();
                digitalWrite(m_regEnable, HIGH);
                m_bcdVcc = dec2bcd(vBat / 100);     // most significant two digits
                PORTA = m_bcdVcc;
                m_bcdVcc = dec2bcd(vBat % 100);     // least significant two digits
            }
            break;

        // display the regulator voltage
        case displayReg:
            m_state = displayVoltage1;
            m_msDisplay = m_ms;
            m_bcdVcc = dec2bcd(m_vcc / 100);        // most significant two digits
            PORTA = m_bcdVcc;
            m_bcdVcc = dec2bcd(m_vcc % 100);        // least significant two digits
            break;

        // display the most significant digits of the voltage on the LEDs
        case displayVoltage1:
            m_btn->read();
            if (m_btn->wasReleased() || m_ms - m_msDisplay > m_msDisplayMax)
            {
                m_state = displayVoltage2;
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
        case displayVoltage2:
            m_btn->read();
            if (m_btn->wasReleased() || m_ms - m_msDisplay > m_msDisplayMax)
            {
                m_state = checkButton;
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

// initialize scan parameters and variables
void Scanner::init()
{
    // set scan parameters
    m_scanDirection = true;
    m_msInterval = params[m_mode].interval;
    m_msOverlap = params[m_mode].overlap;
    m_type = params[m_mode].type;

    // set initial pattern
    if (m_type == scanRandom)
        m_pattern = _BV(random(8));
    else if (m_type == scanRight)
        m_pattern = 0x80;
    else
        m_pattern = 0x01;
    m_prevPattern = m_pattern;
}

// sleep the mcu. a reset is required to wake.
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

// calibrate the internal rc oscillator using value previously stored in eeprom.
void Scanner::calibrate()
{
    // eeprom addresses
    uint16_t* eeSignature = (uint16_t*)0x1f8;       // address for the signature
    uint8_t* eeOSCCAL3 = (uint8_t*)0x1fa;           // address for the calibrated OSCCAL 3.3V value
    uint8_t* eeOSCCAL5 = (uint8_t*)0x1fb;           // address for the calibrated OSCCAL 5V value
    constexpr uint16_t OSCCAL_SIGNATURE(0xaa55);    // the signature value (do not change this)

    // calculate minimum Vcc value, circuit sleeps if Vcc falls below this value
    // (~0.90 * observed Vcc)
    int vcc = readVcc();
    m_minVcc = (vcc / 10) * 9;

    uint16_t signature = eeprom_read_word(eeSignature);
    if (signature == OSCCAL_SIGNATURE)
    {
        uint8_t osccal;
        if (vcc < 3600)
        {
            osccal = eeprom_read_byte(eeOSCCAL3);
            m_hbLED->enable();
            delay(250);
            m_hbLED->disable();
        }
        else
        {
            osccal = eeprom_read_byte(eeOSCCAL5);
            m_hbLED->enable();
            delay(250);
            m_hbLED->disable();
            delay(250);
            m_hbLED->enable();
            delay(250);
            m_hbLED->disable();
        }
        OSCCAL = osccal;
    }
}
