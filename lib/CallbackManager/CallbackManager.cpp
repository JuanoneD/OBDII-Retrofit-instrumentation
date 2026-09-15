#include "CallbackManager.h"
#include <algorithm>

// Initialize static members
std::vector<TimerItem> CallbackManager::m_timers;
std::vector<FlagWatcherItem> CallbackManager::m_flagWatchers;
uint32_t CallbackManager::m_nextId = 1;

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

int CallbackManager::getPendingCycles(uint32_t flagId) {
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
    //
    // Snapshot the set of timer IDs present at the start of this update cycle.
    // Iterating over this snapshot (and re-locating each timer by ID on every
    // iteration) makes the loop robust against arbitrary mutations performed
    // inside callbacks: a callback may remove earlier/later timers, remove
    // itself, or add new ones without causing us to skip or double-process
    // any timer. Timers added during this update() call are intentionally
    // deferred to the next cycle.
    std::vector<uint32_t> timerIdSnapshot;
    timerIdSnapshot.reserve(m_timers.size());
    for (const auto& t : m_timers) {
        timerIdSnapshot.push_back(t.id);
    }

    for (uint32_t timerId : timerIdSnapshot) {
        // Re-locate the timer by ID; it may have been removed by a prior callback.
        size_t idx = m_timers.size();
        for (size_t k = 0; k < m_timers.size(); ++k) {
            if (m_timers[k].id == timerId) {
                idx = k;
                break;
            }
        }
        if (idx == m_timers.size()) {
            continue; // Timer no longer exists
        }

        if (!m_timers[idx].enabled) {
            continue;
        }

        // Safe overflow check for millis()
        if (static_cast<unsigned long>(now - m_timers[idx].lastExecution) >= m_timers[idx].intervalMs) {
            m_timers[idx].lastExecution = now;

            // Snapshot properties before callback to handle collection mutations safely
            bool repeat = m_timers[idx].repeat;
            CallbackFunction cb = m_timers[idx].callback;

            // Execute associated callback
            if (cb) {
                cb();
            }

            // Handle one-shot timers: locate original entry by ID before erasing.
            // The callback may have shifted or removed the entry entirely.
            if (!repeat) {
                for (size_t j = 0; j < m_timers.size(); ++j) {
                    if (m_timers[j].id == timerId) {
                        m_timers.erase(m_timers.begin() + j);
                        break;
                    }
                }
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

            CallbackFunction cb = m_flagWatchers[i].callback;
            if (cb) {
                cb();
            }
        }
    }
}
