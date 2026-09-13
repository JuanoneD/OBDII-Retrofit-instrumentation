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
#define DEBUG_SERIAL_ENABLED        true

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
    TRYING_TO_CONNECT,
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

#define DEFAULT_TIMEOUT               2000
#define AT_COMMAND_TIMEOUT            4000

// -----------------------------------------------------------------------------
// OBD-II Standard PIDs (Mode 01)
// -----------------------------------------------------------------------------
#define PID_ENGINE_LOAD                 0x04
#define PID_COOLANT_TEMP                0x05
#define PID_LONG_TERM_FUEL_TRIM         0x07
#define PID_MAP                         0x0B
#define PID_ENGINE_RPM                  0x0C
#define PID_VEHICLE_SPEED               0x0D
#define PID_TIMING_ADVANCE              0x0E
#define PID_THROTTLE_POSITION           0x11
#define PID_CONTROL_MODULE_VOLTAGE      0x42

// Command strings for querying Mode 01 PIDs
#define PID_ENGINE_LOAD_STR             "0104"
#define PID_COOLANT_TEMP_STR            "0105"
#define PID_LONG_TERM_FUEL_TRIM_STR     "0107"
#define PID_MAP_STR                     "010B"
#define PID_ENGINE_RPM_STR              "010C"
#define PID_VEHICLE_SPEED_STR           "010D"
#define PID_TIMING_ADVANCE_STR          "010E"
#define PID_CONTROL_MODULE_VOLTAGE_STR  "0142"
#define PID_THROTTLE_POSITION_STR       "0111"

// -----------------------------------------------------------------------------
// OBD-II Mathematical Constants for Formulas
// -----------------------------------------------------------------------------
#define OBD_COOLANT_TEMP_OFFSET         40.0f
#define OBD_ENGINE_LOAD_FACTOR          100.0f
#define OBD_ENGINE_LOAD_DIVISOR         255.0f
#define OBD_TIMING_ADVANCE_DIVISOR      2.0f
#define OBD_TIMING_ADVANCE_OFFSET       64.0f
#define OBD_VOLTAGE_DIVISOR             1000.0f
#define OBD_FUEL_TRIM_FACTOR            100.0f
#define OBD_FUEL_TRIM_DIVISOR           128.0f
#define OBD_FUEL_TRIM_OFFSET            100.0f

#include <Arduino.h>
#include <functional>

/// Callback type for raw message events
using RawMessageCallback = std::function<void(const String& data)>;

#endif // CONFIG_H
