#include "VehicleData.h"
#include "DebugSerial.h"

VehicleData& VehicleData::getInstance() {
    static VehicleData instance;
    return instance;
}

VehicleData::VehicleData()
    : obdiiStatus(OBDIISTATUS::DISCONNECTED),
      ecustatus(ECUSTATUS::OFFLINE),
      engineRPM(0),
      vehicleSpeed(0),
      coolantTemp(0),
      engineLoad(0.0f),
      timingAdvance(0.0f),
      throttlePosition(0.0f),
      moduleVoltage(0.0f),
      longTermFuelTrim(0.0f),
      mapPressure(0.0f),
      gasolineLevel(DEFAULT_GASOLINE_LEVEL),
      tankCapacity(DEFAULT_TANK_CAPACITY),
      fuelConsumptionFactor(DEFAULT_FUEL_CONSUMPTION_FACTOR),
      totalDistance(DEFAULT_TOTAL_DISTANCE),
      tripConsumption(DEFAULT_TRIP_CONSUMPTION) {
}

VehicleData::~VehicleData() {
    preferences.end();
}

void VehicleData::loadPersistentData() {
    preferences.begin("OBD2_READER", false);

    tankCapacity = preferences.getFloat("capacity", DEFAULT_TANK_CAPACITY);
    if (!preferences.isKey("capacity")) {
        preferences.putFloat("capacity", DEFAULT_TANK_CAPACITY);
    }

    gasolineLevel = preferences.getFloat("fuel", DEFAULT_GASOLINE_LEVEL);
    if (!preferences.isKey("fuel")) {
        preferences.putFloat("fuel", DEFAULT_GASOLINE_LEVEL);
    }

    fuelConsumptionFactor = preferences.getFloat("factor", DEFAULT_FUEL_CONSUMPTION_FACTOR);
    if (!preferences.isKey("factor")) {
        preferences.putFloat("factor", DEFAULT_FUEL_CONSUMPTION_FACTOR);
    }

    totalDistance = preferences.getFloat("distance", DEFAULT_TOTAL_DISTANCE);
    if (!preferences.isKey("distance")) {
        preferences.putFloat("distance", DEFAULT_TOTAL_DISTANCE);
    }

    tripConsumption = preferences.getFloat("tripFuel", DEFAULT_TRIP_CONSUMPTION);
    if (!preferences.isKey("tripFuel")) {
        preferences.putFloat("tripFuel", DEFAULT_TRIP_CONSUMPTION);
    }

    DebugSerial::println("[VehicleData] Loaded persistent data from NVS:");
    DebugSerial::println("  Tank Capacity: " + String(tankCapacity) + " L");
    DebugSerial::println("  Gasoline Level: " + String(gasolineLevel) + " L");
    DebugSerial::println("  Fuel Consumption Factor: " + String(fuelConsumptionFactor));
    DebugSerial::println("  Total Distance: " + String(totalDistance) + " km");
    DebugSerial::println("  Trip Consumption: " + String(tripConsumption) + " L");
}

// -----------------------------------------------------------------------------
// Status Getters & Setters
// -----------------------------------------------------------------------------

OBDIISTATUS VehicleData::getObdiiStatus() const {
    return obdiiStatus;
}

void VehicleData::setObdiiStatus(OBDIISTATUS status) {
    obdiiStatus = status;
}

ECUSTATUS VehicleData::getEcuStatus() const {
    return ecustatus;
}

void VehicleData::setEcuStatus(ECUSTATUS status) {
    ecustatus = status;
}

// -----------------------------------------------------------------------------
// Volatile Data Getters & Setters
// -----------------------------------------------------------------------------

int VehicleData::getEngineRPM() const {
    return engineRPM;
}

void VehicleData::setEngineRPM(int rpm) {
    engineRPM = rpm;
}

int VehicleData::getVehicleSpeed() const {
    return vehicleSpeed;
}

void VehicleData::setVehicleSpeed(int speed) {
    vehicleSpeed = speed;
}

int VehicleData::getCoolantTemp() const {
    return coolantTemp;
}

void VehicleData::setCoolantTemp(int temp) {
    coolantTemp = temp;
}

float VehicleData::getEngineLoad() const {
    return engineLoad;
}

void VehicleData::setEngineLoad(float load) {
    engineLoad = load;
}

float VehicleData::getTimingAdvance() const {
    return timingAdvance;
}

void VehicleData::setTimingAdvance(float advance) {
    timingAdvance = advance;
}

float VehicleData::getThrottlePosition() const {
    return throttlePosition;
}

void VehicleData::setThrottlePosition(float throttle) {
    throttlePosition = throttle;
}

float VehicleData::getModuleVoltage() const {
    return moduleVoltage;
}

void VehicleData::setModuleVoltage(float voltage) {
    moduleVoltage = voltage;
}

float VehicleData::getLongTermFuelTrim() const {
    return longTermFuelTrim;
}

void VehicleData::setLongTermFuelTrim(float trim) {
    longTermFuelTrim = trim;
}

float VehicleData::getMapPressure() const {
    return mapPressure;
}

void VehicleData::setMapPressure(float pressure) {
    mapPressure = pressure;
}

// -----------------------------------------------------------------------------
// Persistent Data Getters & Setters (Immediate NVS Flash write)
// -----------------------------------------------------------------------------

float VehicleData::getGasolineLevel() const {
    return gasolineLevel;
}

void VehicleData::setGasolineLevel(float level) {
    gasolineLevel = level;
    preferences.putFloat("fuel", gasolineLevel);
}

float VehicleData::getTankCapacity() const {
    return tankCapacity;
}

void VehicleData::setTankCapacity(float capacity) {
    tankCapacity = capacity;
    preferences.putFloat("capacity", tankCapacity);
}

float VehicleData::getFuelConsumptionFactor() const {
    return fuelConsumptionFactor;
}

void VehicleData::setFuelConsumptionFactor(float factor) {
    fuelConsumptionFactor = factor;
    preferences.putFloat("factor", fuelConsumptionFactor);
}

float VehicleData::getTotalDistance() const {
    return totalDistance;
}

void VehicleData::setTotalDistance(float distance) {
    totalDistance = distance;
    preferences.putFloat("distance", totalDistance);
}

float VehicleData::getTripConsumption() const {
    return tripConsumption;
}

void VehicleData::setTripConsumption(float consumption) {
    tripConsumption = consumption;
    preferences.putFloat("tripFuel", tripConsumption);
}
