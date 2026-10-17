#include "FuelCalculator.h"
#include "VehicleData.h"
#include "DebugSerial.h"

FuelCalculator& FuelCalculator::getInstance() {
    static FuelCalculator instance;
    return instance;
}

FuelCalculator::FuelCalculator()
    : lastUpdateMs(0), initialized(false), wasLinkUp(false) {
}

void FuelCalculator::begin() {
    // Timing/state reset. The calibration factor K is no longer cached here:
    // it is read fresh from VehicleData (NVS-backed) on every update() call
    // so that backend-driven calibration changes take effect immediately
    // instead of requiring a reboot.
    lastUpdateMs = millis();
    initialized = true;

    DebugSerial::println("[FuelCalculator] Initialized with K = " +
                          String(VehicleData::getInstance().getFuelConsumptionFactor(), 8));
}

void FuelCalculator::update() {
    if (!initialized) {
        begin();
        return;
    }

    VehicleData& vd = VehicleData::getInstance();

    bool linkUp = (vd.getObdiiStatus() == OBDIISTATUS::CONNECTED &&
                   vd.getEcuStatus() == ECUSTATUS::ONLINE);

    // If the ECU link is down, or it just came back up, do not accumulate
    // elapsed time: resync the time base instead of charging the whole
    // disconnected interval at whatever speed/consumption is first reported
    // after reconnection.
    if (!linkUp) {
        wasLinkUp = false;
        lastUpdateMs = millis();
        return;
    }
    if (!wasLinkUp) {
        wasLinkUp = true;
        lastUpdateMs = millis();
        return;
    }

    // 1. Delta time in seconds (millis handles rollover via unsigned subtraction)
    uint32_t now = millis();
    uint32_t deltaMs = now - lastUpdateMs;
    lastUpdateMs = now;

    if (deltaMs == 0) return;
    float dt = deltaMs / 1000.0f;

    // 2. Read volatile parameters from the ECU (via VehicleData)
    float rpm       = static_cast<float>(vd.getEngineRPM());
    float load      = vd.getEngineLoad();          // %
    float rawLtft   = vd.getLongTermFuelTrim();    // % (ex: -10.0, 5.0, etc.)
    float speedKmh  = static_cast<float>(vd.getVehicleSpeed()); // km/h

    if (rpm <= 0.0f) return;

    // 2b. Accumulate total distance travelled (odometer) from vehicle speed.
    // speed (km/h) * dt (s) / 3600 (s/h) = distance travelled in km during this tick.
    // Uses double precision internally (VehicleData stores totalDistance as
    // double) so small per-tick increments are not lost to float rounding
    // once the odometer reaches typical lifetime mileage (e.g. 100,000 km).
    if (speedKmh > 0.0f) {
        double distanceDeltaKm = (static_cast<double>(speedKmh) * dt) / 3600.0;
        vd.setTotalDistance(vd.getTotalDistance() + distanceDeltaKm);
    }

    // 3. Convert LTFT percentage to a multiplicative correction factor (e.g., -10% -> 0.90, +5% -> 1.05)
    float ltftMultiplier = 1.0f + (rawLtft / 100.0f);
    
    // Safety guard to prevent negative or zero multipliers in case of extreme sensor errors
    if (ltftMultiplier < 0.1f) {
        ltftMultiplier = 0.1f;
    }

    // 4. Apply formula: Consumption = (RPM * EngineLoad * K * LtftMultiplier) * dt
    // K is read fresh from VehicleData (NVS-backed) so a calibration update
    // pushed by the backend takes effect on the very next tick, without
    // requiring a reboot.
    float k = vd.getFuelConsumptionFactor();
    float consumption = (rpm * load * k * ltftMultiplier) * dt;

    if (consumption <= 0.0f) return;

    // 5. Subtract from persistent gasoline level and persist via setter (NVS)
    float currentLevel = vd.getGasolineLevel();
    float newLevel = currentLevel - consumption;
    if (newLevel < 0.0f) newLevel = 0.0f;

    vd.setGasolineLevel(newLevel);

    // Also accumulate on trip consumption for reporting
    vd.setTripConsumption(vd.getTripConsumption() + consumption);
}