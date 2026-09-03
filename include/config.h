#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// GLOBAL CONFIGURATION & CONSTANTS CENTRALIZATION
// All static parameters, intervals, limits, and pin definitions.
// =============================================================================

// -----------------------------------------------------------------------------
// Serial Communication & Debugging
// -----------------------------------------------------------------------------
#define SERIAL_BAUD_RATE            115200

// -----------------------------------------------------------------------------
// Hardware Pinout (ESP32)
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Timer Intervals (in milliseconds)
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Flag & Cycle Counter Configurations
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Enum Definitions
// -----------------------------------------------------------------------------

enum class OBDIISTATUS {
    DISCONNECTED,
    CONNECTED
};

enum class ECUSTATUS {
    OFFLINE,
    ONLINE
};

// -----------------------------------------------------------------------------
// OBDII Manager defines
// -----------------------------------------------------------------------------

#define OBDII_NAME_PREFIX            "OBD"
#define OBDII_NAME_PREFIX_2          "ELM"
#define OBDII_NAME_PREFIX_3          "Vlink"

#define OBDII_SERVICE_UUID            "0000fff0-0000-1000-8000-00805f9b34fb"
#define OBDII_CHAR_UUID_TX            "0000fff2-0000-1000-8000-00805f9b34fb" // Write
#define OBDII_CHAR_UUID_RX            "0000fff1-0000-1000-8000-00805f9b34fb" // Read

#define OBDII_SCAN_TIME_SEC          3

#define DEFAULT_TIMEOUT               400
#define AT_COMMAND_TIMEOUT            1000

#include <Arduino.h>
#include <functional>

/// Callback type for raw message events
using RawMessageCallback = std::function<void(const String& data)>;

#endif // CONFIG_H
