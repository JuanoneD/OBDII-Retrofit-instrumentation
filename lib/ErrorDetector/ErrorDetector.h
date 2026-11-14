#ifndef ERROR_DETECTOR_H
#define ERROR_DETECTOR_H

#include <Arduino.h>
#include "config.h"

/**
 * @class ErrorDetector
 * ------------------------------------------------------------------
 * Real-time system error detection from OBD-II PIDs (Mode 01).
 *
 * A periodic update() (timer-driven, every 1000 ms) evaluates the
 * current values in VehicleData (RPM, speed, coolant temp, engine load,
 * LTFT) against 16 failure conditions. When a condition stays true for
 * its dwell time, an error is queued in VehicleData (FIFO) for cellular
 * transmission (POST /devices/idEsp/addDetectError) and its title is
 * surfaced on the LCD.
 *
 * Anti-spam: each error has a per-condition cooldown and a dwell time,
 * so a momentary spike doesn't fire and a persistent fault doesn't
 * flood the backend.
 *
 * Public flag: newErrorDetectedFlag (for CallbackManager flag watchers,
 * used to trigger an immediate cellular send).
 */
class ErrorDetector {
public:
    // Error IDs (1..16, matching the backend error table)
    enum class ErrorId : uint8_t {
        HIGH_TEMP = 1,
        CRITICAL_TEMP,
        COLD_ENGINE,
        SLOW_WARMUP,
        HIGH_RPM_LOW_SPEED,
        UNSTABLE_IDLE,
        HIGH_IDLE,
        LOW_IDLE,
        HIGH_LOAD,
        HIGH_LOAD_IDLE,
        LOW_LOAD_MOVING,
        LEAN_MIXTURE,
        RICH_MIXTURE,
        FUEL_TRIM_LIMIT,
        OVERHEAT_IDLE,
        OVERHEAT_LOAD
    };

    static void update();

    // Display hook: returns the title of the most recently detected error
    // while it is still "active" (shown on the LCD for a few seconds after
    // firing), or nullptr when there is nothing to show (RPM bar is drawn).
    static const char* getActiveErrorTitle();

    // Flag watcher hook: set to 1 whenever a new error is queued.
    static int newErrorDetectedFlag;

private:
    ErrorDetector() = delete;

    // Evaluates one condition. conditionActive: raw boolean right now.
    // dwellMs: how long it must stay true before firing.
    // cooldownMs: minimum time between two firings of the same error.
    static void checkCondition(ErrorId id, bool conditionActive,
                               unsigned long dwellMs, unsigned long cooldownMs,
                               const String& title, const String& description);

    static void pushError(ErrorId id, const String& title, const String& description);

    // Title of the last fired error, kept for the LCD overlay. Static fixed
    // buffer (20 chars + NUL, LCD width) so getActiveErrorTitle() can return
    // a stable const char* without heap lifetime concerns.
    static char m_activeTitle[21];
    static unsigned long m_activeTitleMs;
    static unsigned long m_lastUpdateMs;

    // Per-condition state (indexed by (int)id - 1)
    struct ConditionState {
        unsigned long conditionStartMs; // 0 = inactive
        unsigned long lastFiredMs;      // millis() of last firing (0 = never)
    };
    static ConditionState m_states[16];

    static unsigned long m_engineOnMs;   // accumulated running time (rpm > 0)
    static unsigned long m_continuousRunStartMs; // 0 = not continuously running
    static int m_tempAtRunStart;

    // Idle RPM window for instability detection (speed == 0)
    static constexpr int IDLE_WINDOW_SAMPLES = 12; // 12 x 500ms = 6 s
    static int m_idleRpmWindow[IDLE_WINDOW_SAMPLES];
    static int m_idleWindowIndex;
    static bool m_idleWindowFull;
};

#endif // ERROR_DETECTOR_H
