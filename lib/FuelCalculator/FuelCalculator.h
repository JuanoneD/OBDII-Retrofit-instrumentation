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
 *   - K is the calibration factor loaded from NVS (fuelConsumptionFactor).
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

    float k;                 // Calibration factor loaded from NVS
    uint32_t lastUpdateMs;   // Timestamp of the last update() call
    bool initialized;
};

#endif // FUEL_CALCULATOR_H
