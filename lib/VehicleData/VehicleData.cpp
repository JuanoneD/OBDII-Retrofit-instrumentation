#include "VehicleData.h"
#include "DebugSerial.h"
#include <math.h>

namespace {
// RAII guard for the VehicleData access mutex (see VehicleData::m_mutex).
// VehicleData is accessed from both the loop task and the SIM800L
// background send task; NVS/Preferences is not thread-safe, so every
// public getter/setter takes this lock for its whole duration.
struct VdLock {
    SemaphoreHandle_t h;
    explicit VdLock(SemaphoreHandle_t mutex) : h(mutex) {
        if (h != nullptr) xSemaphoreTake(h, portMAX_DELAY);
    }
    ~VdLock() {
        if (h != nullptr) xSemaphoreGive(h);
    }
    VdLock(const VdLock&) = delete;
    VdLock& operator=(const VdLock&) = delete;
};
}

VehicleData& VehicleData::getInstance() {
    static VehicleData instance;
    return instance;
}

VehicleData::VehicleData()
    : m_mutex(xSemaphoreCreateMutex()),
      obdiiStatus(OBDIISTATUS::DISCONNECTED),
      ecustatus(ECUSTATUS::OFFLINE),
      esimStatus(ESIMSTATUS::DISCONNECTED),
      engineRPM(0),
      vehicleSpeed(0),
      coolantTemp(0),
      engineLoad(0.0f),
      timingAdvance(0.0f),
      throttlePosition(0.0f),
      moduleVoltage(0.0f),
      longTermFuelTrim(0.0f),
      mapPressure(0.0f),
      // Not persisted: DTCs must be requested once per boot. Placed here
      // (not after tripConsumption) to match member declaration order.
      requestDTCsOnStarted(0),
      lastDetectedErrorMs(0),
      gasolineLevel(DEFAULT_GASOLINE_LEVEL),
      tankCapacity(DEFAULT_TANK_CAPACITY),
      fuelConsumptionFactor(DEFAULT_FUEL_CONSUMPTION_FACTOR),
      totalDistance(DEFAULT_TOTAL_DISTANCE),
      tripConsumption(DEFAULT_TRIP_CONSUMPTION),
      diagnoseMode(DEFAULT_DIAGNOSE_MODE) {
}

VehicleData::~VehicleData() {
    preferences.end();
}

void VehicleData::loadPersistentData() {
    VdLock lock(m_mutex);

    // begin() returns false when NVS is unavailable (corrupt/full partition);
    // without this check every setter's put* silently no-ops for the whole
    // session and the values just fall back to defaults with no clue why.
    if (!preferences.begin("OBD2_READER", false)) {
        DebugSerial::println("[VehicleData] ERROR: NVS init failed (corrupt or full?). Using defaults; persistence disabled this session.");
    }

    // get*(key, default) already returns the default when the key does not
    // exist yet, so the old "if (!isKey) put(...)" bookkeeping was
    // redundant: first boot simply gets the defaults in RAM and the keys
    // are written lazily by the setters. The only eager write kept is the
    // fallback for a corrupted factor.
    tankCapacity = preferences.getFloat("capacity", DEFAULT_TANK_CAPACITY);
    gasolineLevel = preferences.getFloat("fuel", DEFAULT_GASOLINE_LEVEL);
    fuelConsumptionFactor = preferences.getFloat("factor", DEFAULT_FUEL_CONSUMPTION_FACTOR);
    if (fuelConsumptionFactor <= 0.0f) {
        fuelConsumptionFactor = DEFAULT_FUEL_CONSUMPTION_FACTOR;
        preferences.putFloat("factor", DEFAULT_FUEL_CONSUMPTION_FACTOR);
    }
    totalDistance = preferences.getDouble("distance", DEFAULT_TOTAL_DISTANCE);
    tripConsumption = preferences.getFloat("tripFuel", DEFAULT_TRIP_CONSUMPTION);
    syncVersion = preferences.getInt("sync_version", DEFAULT_SYNC_VERSION);
    diagnoseMode = preferences.getInt("diagnose_mode", DEFAULT_DIAGNOSE_MODE);

    DebugSerial::println("[VehicleData] Loaded persistent data from NVS:");
    DebugSerial::println("  Tank Capacity: " + String(tankCapacity) + " L");
    DebugSerial::println("  Gasoline Level: " + String(gasolineLevel) + " L");
    DebugSerial::println("  Fuel Consumption Factor: " + String(fuelConsumptionFactor));
    DebugSerial::println("  Total Distance: " + String(totalDistance) + " km");
    DebugSerial::println("  Trip Consumption: " + String(tripConsumption) + " L");
    DebugSerial::println("  Request DTCs on Start: " + String(requestDTCsOnStarted));
    DebugSerial::println("  Diagnose Mode: " + String(diagnoseMode));
}

// -----------------------------------------------------------------------------
// Status Getters & Setters
// -----------------------------------------------------------------------------

OBDIISTATUS VehicleData::getObdiiStatus() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return obdiiStatus;
}

void VehicleData::setObdiiStatus(OBDIISTATUS status) {
    VdLock lock(m_mutex);
    obdiiStatus = status;
}

ECUSTATUS VehicleData::getEcuStatus() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return ecustatus;
}

void VehicleData::setEcuStatus(ECUSTATUS status) {
    VdLock lock(m_mutex);
    ecustatus = status;
}

ESIMSTATUS VehicleData::getEsimStatus() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return esimStatus;
}

void VehicleData::setEsimStatus(ESIMSTATUS status) {
    VdLock lock(m_mutex);
    esimStatus = status;
}

// -----------------------------------------------------------------------------
// Volatile Data Getters & Setters
// -----------------------------------------------------------------------------

int VehicleData::getEngineRPM() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return engineRPM;
}

void VehicleData::setEngineRPM(int rpm) {
    VdLock lock(m_mutex);
    engineRPM = rpm;
}

int VehicleData::getVehicleSpeed() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return vehicleSpeed;
}

void VehicleData::setVehicleSpeed(int speed) {
    VdLock lock(m_mutex);
    vehicleSpeed = speed;
}

int VehicleData::getCoolantTemp() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return coolantTemp;
}

void VehicleData::setCoolantTemp(int temp) {
    VdLock lock(m_mutex);
    coolantTemp = temp;
}

float VehicleData::getEngineLoad() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return engineLoad;
}

void VehicleData::setEngineLoad(float load) {
    VdLock lock(m_mutex);
    engineLoad = load;
}

float VehicleData::getTimingAdvance() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return timingAdvance;
}

void VehicleData::setTimingAdvance(float advance) {
    VdLock lock(m_mutex);
    timingAdvance = advance;
}

float VehicleData::getThrottlePosition() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return throttlePosition;
}

void VehicleData::setThrottlePosition(float throttle) {
    VdLock lock(m_mutex);
    throttlePosition = throttle;
}

float VehicleData::getModuleVoltage() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return moduleVoltage;
}

void VehicleData::setModuleVoltage(float voltage) {
    VdLock lock(m_mutex);
    moduleVoltage = voltage;
}

float VehicleData::getLongTermFuelTrim() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return longTermFuelTrim;
}

void VehicleData::setLongTermFuelTrim(float trim) {
    VdLock lock(m_mutex);
    longTermFuelTrim = trim;
}

float VehicleData::getMapPressure() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return mapPressure;
}

void VehicleData::setMapPressure(float pressure) {
    VdLock lock(m_mutex);
    mapPressure = pressure;
}

// -----------------------------------------------------------------------------
// Persistent Data Getters & Setters (Immediate NVS Flash write)
// -----------------------------------------------------------------------------

float VehicleData::getGasolineLevel() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return gasolineLevel;
}

void VehicleData::setGasolineLevel(float level) {
    VdLock lock(m_mutex);
    gasolineLevel = level;
    preferences.putFloat("fuel", gasolineLevel);
}

float VehicleData::getTankCapacity() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return tankCapacity;
}

void VehicleData::setTankCapacity(float capacity) {
    VdLock lock(m_mutex);
    tankCapacity = capacity;
    preferences.putFloat("capacity", tankCapacity);
}

float VehicleData::getFuelConsumptionFactor() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return fuelConsumptionFactor;
}

void VehicleData::setFuelConsumptionFactor(float factor) {
    // Defensive guard: never persist NaN, +/-Infinity, zero or negative
    // values. This is the calibration constant used to compute instant fuel
    // consumption, so a degenerate value here would zero out or blow up the
    // calculation (e.g. if it comes corrupted/truncated from the backend).
    if (!isfinite(factor) || factor <= 0.0f) {
        DebugSerial::println("[VehicleData] WARNING: Rejected invalid fuelConsumptionFactor (" +
                              String(factor, 10) + "). Keeping previous value.");
        return;
    }
    VdLock lock(m_mutex);
    fuelConsumptionFactor = factor;
    preferences.putFloat("factor", fuelConsumptionFactor);
}

double VehicleData::getTotalDistance() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return totalDistance;
}

void VehicleData::setTotalDistance(double distance) {
    VdLock lock(m_mutex);
    totalDistance = distance;
    preferences.putDouble("distance", totalDistance);
}

float VehicleData::getTripConsumption() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return tripConsumption;
}

void VehicleData::setTripConsumption(float consumption) {
    VdLock lock(m_mutex);
    tripConsumption = consumption;
    preferences.putFloat("tripFuel", tripConsumption);
}

void VehicleData::setSyncVersion(int version) {
    VdLock lock(m_mutex);
    syncVersion = version;
    preferences.putInt("sync_version", syncVersion);
}

int VehicleData::getSyncVersion() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return syncVersion;
}

int VehicleData::getDiagnoseMode() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return diagnoseMode;
}

void VehicleData::setDiagnoseMode(int mode) {
    VdLock lock(m_mutex);
    if (diagnoseMode != mode) {
        diagnoseMode = mode;
        preferences.putInt("diagnose_mode", diagnoseMode);
        if (diagnoseMode == 1) {
            requestDTCsOnStarted = 0;
        }
    }
}

int VehicleData::getRequestDTCsOnStarted() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return requestDTCsOnStarted;
}

void VehicleData::setRequestDTCsOnStarted(int request) {
    VdLock lock(m_mutex);
    requestDTCsOnStarted = request;
}

std::vector<String> VehicleData::getDTCs() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return dtcList;
}

void VehicleData::setDTCs(const std::vector<String>& dtcs) {
    VdLock lock(m_mutex);
    dtcList = dtcs;
}

void VehicleData::clearDTCs() {
    VdLock lock(m_mutex);
    dtcList.clear();
}

int VehicleData::getDTCCount() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return (int)dtcList.size();
}

// -----------------------------------------------------------------------------
// Real-time Detected Errors
// -----------------------------------------------------------------------------

void VehicleData::pushDetectedError(const String& title, const String& description) {
    VdLock lock(m_mutex);
    // Bounded queue: if the cellular link is down for a long time, drop the
    // oldest errors instead of growing the heap indefinitely.
    if (detectedErrors.size() >= 10) {
        detectedErrors.erase(detectedErrors.begin());
    }
    DetectedError e;
    e.title = title;
    e.description = description;
    detectedErrors.push_back(e);
    lastDetectedErrorTitle = title;
    lastDetectedErrorMs = millis();
    DebugSerial::println("[ErrorDetector] Detected: " + title + " - " + description);
}

bool VehicleData::popDetectedError(DetectedError& out) {
    VdLock lock(m_mutex);
    if (detectedErrors.empty()) return false;
    out = detectedErrors.front();
    detectedErrors.erase(detectedErrors.begin());
    return true;
}

bool VehicleData::peekDetectedError(DetectedError& out) const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    if (detectedErrors.empty()) return false;
    out = detectedErrors.front();
    return true;
}

int VehicleData::getDetectedErrorCount() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return (int)detectedErrors.size();
}

String VehicleData::getLastDetectedErrorTitle() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return lastDetectedErrorTitle;
}

unsigned long VehicleData::getLastDetectedErrorMs() const {
    VdLock lock(const_cast<VehicleData*>(this)->m_mutex);
    return lastDetectedErrorMs;
}
