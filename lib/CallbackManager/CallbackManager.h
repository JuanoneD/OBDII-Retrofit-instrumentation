#ifndef CALLBACK_MANAGER_H
#define CALLBACK_MANAGER_H

#include <Arduino.h>
#include <vector>
#include <functional>

/**
 * @brief Standard callback function signature executed by the CallbackManager.
 */
using CallbackFunction = std::function<void()>;

/**
 * @brief Representation structure of an independent software timer.
 */
struct TimerItem {
    uint32_t id;                    ///< Unique timer identifier
    unsigned long intervalMs;       ///< Trigger interval in milliseconds
    unsigned long lastExecution;    ///< Timestamp of the last execution (millis())
    bool enabled;                   ///< Activation state (true = active, false = paused)
    bool repeat;                    ///< Indicates whether the timer repeats or runs once (one-shot)
    CallbackFunction callback;      ///< Function / lambda executed upon expiration
};

/**
 * @brief Representation structure of an integer pointer flag watcher.
 */
struct FlagWatcherItem {
    uint32_t id;                    ///< Unique flag watcher identifier
    int* flagPtr;                   ///< Pointer to the monitored integer variable
    int pendingCycles;              ///< Number of remaining cycles to execute
    bool enabled;                   ///< Activation state (true = active, false = paused)
    CallbackFunction callback;      ///< Function / lambda executed for each active cycle
};

/**
 * @class CallbackManager
 * @brief Centralized, dynamic, and non-blocking manager for periodic timers
 *        and flag-driven execution (pointer counter monitors).
 *        All methods are static - no instantiation required.
 * 
 * @example
 * // Example usage (no need to create an instance):
 * 
 * void setup() {
 *     // Add a periodic timer (executes every 1000ms)
 *     uint32_t timerId = CallbackManager::addTimer(1000, []() {
 *         Serial.println("Timer fired!");
 *     });
 * 
 *     // Add a flag watcher
 *     int myFlag = 0;
 *     CallbackManager::addFlagWatcher(&myFlag, []() {
 *         Serial.println("Flag triggered!");
 *     });
 * }
 * 
 * void loop() {
 *     // Must be called in every loop iteration
 *     CallbackManager::update();
 * }
 */
class CallbackManager {
public:
    // Prevent instantiation
    CallbackManager() = delete;
    ~CallbackManager() = delete;
    CallbackManager(const CallbackManager&) = delete;
    CallbackManager& operator=(const CallbackManager&) = delete;

    // =========================================================================
    // TIMER SUBSYSTEM
    // =========================================================================

    /**
     * @brief Registers a new periodic or one-shot timer.
     * @param intervalMs Trigger interval in milliseconds.
     * @param callback Function void() or lambda to execute.
     * @param enabled Whether the timer starts enabled (default: true).
     * @param repeat Whether the timer repeats indefinitely (default: true).
     * @return uint32_t Unique timer ID (0 on failure).
     */
    static uint32_t addTimer(unsigned long intervalMs, CallbackFunction callback, bool enabled = true, bool repeat = true);

    /**
     * @brief Pauses a specific timer.
     * @param timerId Timer ID returned by addTimer.
     * @return true if the timer was found and paused.
     */
    static bool pauseTimer(uint32_t timerId);

    /**
     * @brief Resumes a paused timer.
     * @param timerId Timer ID returned by addTimer.
     * @return true if the timer was found and resumed.
     */
    static bool resumeTimer(uint32_t timerId);

    /**
     * @brief Toggles the enabled state of a timer.
     * @param timerId Timer ID.
     * @return true if the timer was found.
     */
    static bool toggleTimer(uint32_t timerId);

    /**
     * @brief Removes and deallocates a timer from the dynamic collection.
     * @param timerId Timer ID to remove.
     * @return true if the timer was found and removed.
     */
    static bool removeTimer(uint32_t timerId);

    /**
     * @brief Modifies the interval of an existing timer and resets its base time.
     * @param timerId Timer ID.
     * @param newIntervalMs New interval in milliseconds.
     * @return true if the timer was found.
     */
    static bool setTimerInterval(uint32_t timerId, unsigned long newIntervalMs);

    /**
     * @brief Resets the time reference of a timer (lastExecution = millis()).
     * @param timerId Timer ID.
     * @return true if the timer was found.
     */
    static bool resetTimer(uint32_t timerId);

    /**
     * @brief Removes all registered timers.
     */
    static void clearTimers();

    // =========================================================================
    // FLAG WATCHER SUBSYSTEM (POINTER COUNTER)
    // =========================================================================

    /**
     * @brief Registers monitoring for an integer variable via pointer (int*).
     *        When *flagPtr > 0, the value is captured as pending cycles,
     *        the pointer is automatically reset (*flagPtr = 0), and the
     *        associated callback is executed consecutively on each update()
     *        cycle until pending cycles reach zero.
     * @param flagPtr Pointer to the integer variable to monitor.
     * @param callback Function void() or lambda executed for each pending cycle.
     * @param enabled Whether the watcher starts enabled (default: true).
     * @return uint32_t Unique flag watcher ID (0 if null pointer).
     */
    static uint32_t addFlagWatcher(int* flagPtr, CallbackFunction callback, bool enabled = true);

    /**
     * @brief Pauses a specific flag watcher.
     * @param flagId ID returned by addFlagWatcher.
     * @return true if the watcher was found and paused.
     */
    static bool pauseFlagWatcher(uint32_t flagId);

    /**
     * @brief Resumes a paused flag watcher.
     * @param flagId ID returned by addFlagWatcher.
     * @return true if the watcher was found and resumed.
     */
    static bool resumeFlagWatcher(uint32_t flagId);

    /**
     * @brief Removes and deallocates a flag watcher from the dynamic collection.
     * @param flagId ID of the watcher to remove.
     * @return true if the watcher was found and removed.
     */
    static bool removeFlagWatcher(uint32_t flagId);

    /**
     * @brief Gets the number of pending cycles for a flag watcher.
     * @param flagId Flag watcher ID.
     * @return int Number of pending cycles (-1 if invalid ID).
     */
    static int getPendingCycles(uint32_t flagId);

    /**
     * @brief Removes all registered flag watchers.
     */
    static void clearFlagWatchers();

    // =========================================================================
    // CORE PROCESSING ROUTINE (NON-BLOCKING)
    // =========================================================================

    /**
     * @brief Updates all timers and flag watchers.
     *        Must be called on every iteration of the ESP32 loop().
     *        Completely non-blocking and single-core friendly.
     */
    static void update();

    // =========================================================================
    // QUERY / DIAGNOSTIC METHODS
    // =========================================================================
    static size_t getTimerCount() { return m_timers.size(); }
    static size_t getFlagWatcherCount() { return m_flagWatchers.size(); }

private:
    static std::vector<TimerItem> m_timers;            ///< Dynamic collection of software timers
    static std::vector<FlagWatcherItem> m_flagWatchers;///< Dynamic collection of flag watchers
    static uint32_t m_nextId;                          ///< Sequential ID generator

    static uint32_t generateId();
};

#endif // CALLBACK_MANAGER_H
