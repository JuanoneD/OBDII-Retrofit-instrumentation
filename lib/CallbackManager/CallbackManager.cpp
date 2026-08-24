#include "CallbackManager.h"
#include <algorithm>

CallbackManager::CallbackManager()
    : m_nextId(1) {
}

CallbackManager::~CallbackManager() {
    clearTimers();
    clearFlagWatchers();
}

uint32_t CallbackManager::generateId() {
    uint32_t id = m_nextId++;
    if (m_nextId == 0) {
        m_nextId = 1; // Prevent ID 0
    }
    return id;
}

// =============================================================================
// TIMER SUBSYSTEM IMPLEMENTATION
// =============================================================================

uint32_t CallbackManager::addTimer(unsigned long intervalMs, CallbackFunction callback, bool enabled, bool repeat) {
    if (!callback) {
        return 0;
    }

    TimerItem timer;
    timer.id = generateId();
    timer.intervalMs = intervalMs;
    timer.lastExecution = millis();
    timer.enabled = enabled;
    timer.repeat = repeat;
    timer.callback = callback;

    m_timers.push_back(timer);
    return timer.id;
}

bool CallbackManager::pauseTimer(uint32_t timerId) {
    for (auto& timer : m_timers) {
        if (timer.id == timerId) {
            timer.enabled = false;
            return true;
        }
    }
    return false;
}

bool CallbackManager::resumeTimer(uint32_t timerId) {
    for (auto& timer : m_timers) {
        if (timer.id == timerId) {
            timer.enabled = true;
            timer.lastExecution = millis(); // Reset time base upon resuming
            return true;
        }
    }
    return false;
}

bool CallbackManager::toggleTimer(uint32_t timerId) {
    for (auto& timer : m_timers) {
        if (timer.id == timerId) {
            timer.enabled = !timer.enabled;
            if (timer.enabled) {
                timer.lastExecution = millis();
            }
            return true;
        }
    }
    return false;
}

bool CallbackManager::removeTimer(uint32_t timerId) {
    for (auto it = m_timers.begin(); it != m_timers.end(); ++it) {
        if (it->id == timerId) {
            m_timers.erase(it);
            return true;
        }
    }
    return false;
}

bool CallbackManager::setTimerInterval(uint32_t timerId, unsigned long newIntervalMs) {
    for (auto& timer : m_timers) {
        if (timer.id == timerId) {
            timer.intervalMs = newIntervalMs;
            timer.lastExecution = millis();
            return true;
        }
    }
    return false;
}

bool CallbackManager::resetTimer(uint32_t timerId) {
    for (auto& timer : m_timers) {
        if (timer.id == timerId) {
            timer.lastExecution = millis();
            return true;
        }
    }
    return false;
}

void CallbackManager::clearTimers() {
    m_timers.clear();
}

// =============================================================================
// FLAG WATCHER SUBSYSTEM (POINTER COUNTER) IMPLEMENTATION
// =============================================================================

uint32_t CallbackManager::addFlagWatcher(int* flagPtr, CallbackFunction callback, bool enabled) {
    if (flagPtr == nullptr || !callback) {
        return 0;
    }

    FlagWatcherItem watcher;
    watcher.id = generateId();
    watcher.flagPtr = flagPtr;
    watcher.pendingCycles = 0;
    watcher.enabled = enabled;
    watcher.callback = callback;

    m_flagWatchers.push_back(watcher);
    return watcher.id;
}

bool CallbackManager::pauseFlagWatcher(uint32_t flagId) {
    for (auto& watcher : m_flagWatchers) {
        if (watcher.id == flagId) {
            watcher.enabled = false;
            return true;
        }
    }
    return false;
}

bool CallbackManager::resumeFlagWatcher(uint32_t flagId) {
    for (auto& watcher : m_flagWatchers) {
        if (watcher.id == flagId) {
            watcher.enabled = true;
            return true;
        }
    }
    return false;
}

bool CallbackManager::removeFlagWatcher(uint32_t flagId) {
    for (auto it = m_flagWatchers.begin(); it != m_flagWatchers.end(); ++it) {
        if (it->id == flagId) {
            m_flagWatchers.erase(it);
            return true;
        }
    }
    return false;
}

int CallbackManager::getPendingCycles(uint32_t flagId) const {
    for (const auto& watcher : m_flagWatchers) {
        if (watcher.id == flagId) {
            return watcher.pendingCycles;
        }
    }
    return -1;
}

void CallbackManager::clearFlagWatchers() {
    m_flagWatchers.clear();
}

// =============================================================================
// MAIN PROCESSING ROUTINE (NON-BLOCKING UPDATE)
// =============================================================================

void CallbackManager::update() {
    const unsigned long now = millis();

    // 1. Process Timers
    // Use index iteration to remain safe if collection is modified inside callbacks
    for (size_t i = 0; i < m_timers.size(); ++i) {
        if (!m_timers[i].enabled) {
            continue;
        }

        // Safe overflow check for millis()
        if (static_cast<unsigned long>(now - m_timers[i].lastExecution) >= m_timers[i].intervalMs) {
            m_timers[i].lastExecution = now;

            // Execute associated callback
            if (m_timers[i].callback) {
                m_timers[i].callback();
            }

            // Handle one-shot timers
            if (!m_timers[i].repeat) {
                m_timers.erase(m_timers.begin() + i);
                --i; // Adjust loop index after removal
            }
        }
    }

    // 2. Process Flag Watchers (Pointer Counters)
    for (size_t i = 0; i < m_flagWatchers.size(); ++i) {
        if (!m_flagWatchers[i].enabled || m_flagWatchers[i].flagPtr == nullptr) {
            continue;
        }

        // Continuous monitoring: capture pointer value if new cycles were triggered
        if (*(m_flagWatchers[i].flagPtr) > 0) {
            m_flagWatchers[i].pendingCycles += *(m_flagWatchers[i].flagPtr);
            // Automatic reset of the source pointer variable
            *(m_flagWatchers[i].flagPtr) = 0;
        }

        // Per-cycle execution and automatic decrement of pending cycles
        if (m_flagWatchers[i].pendingCycles > 0) {
            m_flagWatchers[i].pendingCycles--;

            if (m_flagWatchers[i].callback) {
                m_flagWatchers[i].callback();
            }
        }
    }
}
