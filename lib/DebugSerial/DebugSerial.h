#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include <utility>
#include "config.h"

/**
 * @class DebugSerial
 * @brief Static utility class for sending debug messages via Serial.
 *        Contains only static methods; no instantiation required.
 */
class DebugSerial {
public:
    // Prevent instantiation
    DebugSerial() = delete;
    ~DebugSerial() = delete;
    DebugSerial(const DebugSerial&) = delete;
    DebugSerial& operator=(const DebugSerial&) = delete;

    /**
     * @brief Initializes the serial port with the specified baud rate.
     * @param baudRate Baud rate for communication (default: SERIAL_BAUD_RATE from config.h).
     */
    static void begin(unsigned long baudRate = SERIAL_BAUD_RATE);
    static void init(unsigned long baudRate = SERIAL_BAUD_RATE);

    /**
     * @brief Enables or disables serial debug output.
     * @param enabled true to enable, false to disable.
     */
    static void setEnabled(bool enabled);

    /**
     * @brief Returns whether serial debug is enabled.
     */
    static bool isEnabled();

    /**
     * @brief Prints data to serial (without trailing newline).
     */
    template <typename T>
    static void print(const T& data) {
        if (m_enabled) {
            Serial.print(data);
        }
    }

    template <typename T>
    static void print(const T& data, int format) {
        if (m_enabled) {
            Serial.print(data, format);
        }
    }

    /**
     * @brief Prints data to serial followed by a newline.
     */
    template <typename T>
    static void println(const T& data) {
        if (m_enabled) {
            Serial.println(data);
        }
    }

    template <typename T>
    static void println(const T& data, int format) {
        if (m_enabled) {
            Serial.println(data, format);
        }
    }

    /**
     * @brief Prints an empty newline.
     */
    static void println();

    /**
     * @brief Prints formatted data to serial (printf style).
     */
    template <typename... Args>
    static void printf(const char* format, Args&&... args) {
        if (m_enabled) {
            Serial.printf(format, std::forward<Args>(args)...);
        }
    }

private:
    static bool m_enabled;
};

#endif // DEBUG_SERIAL_H
