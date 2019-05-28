// heartbeat led class
class HB_LED
{
    public:
        HB_LED(uint8_t pin, uint32_t interval)
            : m_pin(pin), m_onInterval(interval), m_offInterval(interval), m_state(LED_ON) {}
        HB_LED(uint8_t pin, uint32_t onInterval, uint32_t offInterval)
            : m_pin(pin), m_onInterval(onInterval), m_offInterval(offInterval), m_state(LED_ON) {}
        void begin();
        void run();
        void disable();
        void enable();

    private:
        enum states_t {LED_OFF, LED_ON, LED_DISABLED};  // states for the state machine
        uint8_t m_pin;
        uint32_t m_onInterval;
        uint32_t m_offInterval;
        uint32_t m_lastChange;
        uint8_t m_state;
};

void HB_LED::begin()
{
    pinMode(m_pin, OUTPUT);
    if (m_state == LED_ON) digitalWrite(m_pin, HIGH);
    m_lastChange = millis();
}

void HB_LED::disable()
{
    m_state = LED_DISABLED;
    digitalWrite(m_pin, LOW);
}

void HB_LED::enable()
{
    m_state = LED_ON;
    digitalWrite(m_pin, HIGH);
    m_lastChange = millis();
}

void HB_LED::run()
{
    uint32_t ms = millis();
    switch (m_state)
    {
        case LED_OFF:
            if (ms - m_lastChange >= m_offInterval)
            {
                m_lastChange = ms;
                m_state = LED_ON;
                digitalWrite(m_pin, HIGH);
            }
            break;

        case LED_ON:
            if (ms - m_lastChange >= m_onInterval)
            {
                m_lastChange = ms;
                m_state = LED_OFF;
                digitalWrite(m_pin, LOW);
            }
            break;

        case LED_DISABLED:
            break;
    }
}
