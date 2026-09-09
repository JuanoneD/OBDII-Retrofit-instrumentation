#include "DebugSerial.h"

#ifdef DEBUG_SERIAL_ENABLED
bool DebugSerial::m_enabled = DEBUG_SERIAL_ENABLED;
#else
bool DebugSerial::m_enabled = true;
#endif

void DebugSerial::begin(unsigned long baudRate) {
    Serial.begin(baudRate);
}

void DebugSerial::init(unsigned long baudRate) {
    begin(baudRate);
}

void DebugSerial::setEnabled(bool enabled) {
    m_enabled = enabled;
}

bool DebugSerial::isEnabled() {
    return m_enabled;
}

void DebugSerial::println() {
    if (m_enabled) {
        Serial.println();
    }
}
