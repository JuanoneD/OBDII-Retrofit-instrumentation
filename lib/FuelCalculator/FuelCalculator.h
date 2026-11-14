#ifndef FUEL_CALCULATOR_H
#define FUEL_CALCULATOR_H

#include <Arduino.h>

/**
 * FuelCalculator
 * ----------------------------------------------------------------------------
 * Isolated loop responsible for estimating instantaneous fuel consumption and
 * decrementing the persistent gasoline level stored in NVS through the
 * VehicleData singleton.
 *
 * Formula:
 *   Consumption = (RPM * EngineLoad * K * LongTermFuelTrim) * dt
 *
 * Where:
 *   - RPM, EngineLoad, LongTermFuelTrim come from VehicleData (volatile).
 *   - K is the calibration factor, read fresh from VehicleData (NVS-backed)
 *     on every update() call so backend calibration changes apply immediately.
 *   - dt is the elapsed time (seconds) since the last update() call.
 */
class FuelCalculator {
public:
    // Singleton access
    static FuelCalculator& getInstance();

    // Load calibration factor K from NVS via VehicleData and reset timing.
    void begin();

    // Main loop step: reads volatile data, applies formula and decrements
    // the persistent gasoline level. Returns void as specified.
    void update();

private:
    FuelCalculator();
    ~FuelCalculator() = default;

    FuelCalculator(const FuelCalculator&) = delete;
    FuelCalculator& operator=(const FuelCalculator&) = delete;

    uint32_t lastUpdateMs;   // Timestamp of the last update() call
    bool initialized;
    bool wasLinkUp;          // Tracks OBD/ECU link state to detect reconnection

    // Accumulated (unpersisted) deltas. NVS/flash is only written every
    // PERSIST_INTERVAL_MS (or when the link drops), not on every 500ms tick:
    // persisting fuel level + odometer + trip consumption three times per
    // second would wear the ESP32 flash partition far faster than needed.
    static constexpr uint32_t PERSIST_INTERVAL_MS = 30000;
    double pendingFuelDelta = 0.0;      // negative (consumption)
    double pendingDistanceDelta = 0.0;  // positive
    double pendingTripDelta = 0.0;      // positive
    uint32_t lastPersistMs = 0;

    // Flushes the pending deltas into VehicleData (NVS). Called from
    // update() on the persistence interval and on link drop.
    void persistPending();
};

#endif // FUEL_CALCULATOR_H
