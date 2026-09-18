#include "FuelCalculator.h"
#include "VehicleData.h"
#include "DebugSerial.h"

FuelCalculator& FuelCalculator::getInstance() {
    static FuelCalculator instance;
    return instance;
}

FuelCalculator::FuelCalculator()
    : k(0.0f), lastUpdateMs(0), initialized(false) {
}

void FuelCalculator::begin() {
    // Load the calibration factor K from NVS (via VehicleData singleton).
    k = VehicleData::getInstance().getFuelConsumptionFactor();
    lastUpdateMs = millis();
    initialized = true;

    DebugSerial::println("[FuelCalculator] Initialized with K = " + String(k, 8));
}

void FuelCalculator::update() {
    if (!initialized) {
        begin();
        return;
    }

    VehicleData& vd = VehicleData::getInstance();

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

    if (vd.getObdiiStatus() != OBDIISTATUS::CONNECTED || vd.getEcuStatus() != ECUSTATUS::ONLINE || rpm <= 0.0f) return;

    // 3. Convert LTFT percentage to a multiplicative correction factor (e.g., -10% -> 0.90, +5% -> 1.05)
    float ltftMultiplier = 1.0f + (rawLtft / 100.0f);
    
    // Safety guard to prevent negative or zero multipliers in case of extreme sensor errors
    if (ltftMultiplier < 0.1f) {
        ltftMultiplier = 0.1f;
    }

    // 4. Apply formula: Consumption = (RPM * EngineLoad * K * LtftMultiplier) * dt
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