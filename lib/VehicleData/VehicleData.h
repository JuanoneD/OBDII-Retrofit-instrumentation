#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <Arduino.h>
#include <vector>
#include <Preferences.h>
#include "config.h"

class VehicleData {
public:
    // Singleton access
    static VehicleData& getInstance();

    // Persistence initialization
    void loadPersistentData();

    // --- Status Getters & Setters ---
    OBDIISTATUS getObdiiStatus() const;
    void setObdiiStatus(OBDIISTATUS status);

    ECUSTATUS getEcuStatus() const;
    void setEcuStatus(ECUSTATUS status);

    ESIMSTATUS getEsimStatus() const;
    void setEsimStatus(ESIMSTATUS status);

    // --- Volatile Data (Real-time in RAM) ---
    int getEngineRPM() const;
    void setEngineRPM(int rpm);

    int getVehicleSpeed() const;
    void setVehicleSpeed(int speed);

    int getCoolantTemp() const;
    void setCoolantTemp(int temp);

    float getEngineLoad() const;
    void setEngineLoad(float load);

    float getTimingAdvance() const;
    void setTimingAdvance(float advance);

    float getThrottlePosition() const;
    void setThrottlePosition(float throttle);

    float getModuleVoltage() const;
    void setModuleVoltage(float voltage);

    float getLongTermFuelTrim() const;
    void setLongTermFuelTrim(float trim);

    float getMapPressure() const;
    void setMapPressure(float pressure);

    // --- Persistent Data (Flash NVS via Preferences) ---
    float getGasolineLevel() const;
    void setGasolineLevel(float level);

    float getTankCapacity() const;
    void setTankCapacity(float capacity);

    float getFuelConsumptionFactor() const;
    void setFuelConsumptionFactor(float factor);

    double getTotalDistance() const;
    void setTotalDistance(double distance);

    float getTripConsumption() const;
    void setTripConsumption(float consumption);

    int getSyncVersion() const;
    void setSyncVersion(int version);

    int getDiagnoseMode() const;
    void setDiagnoseMode(int mode);

    int getRequestDTCsOnStarted() const;
    void setRequestDTCsOnStarted(int request);

    // --- Diagnostic Trouble Codes (DTCs) ---
    std::vector<String> getDTCs() const;
    void setDTCs(const std::vector<String>& dtcs);
    void addDTC(const String& dtc);
    void clearDTCs();
    int getDTCCount() const;

private:
    VehicleData();
    ~VehicleData();

    // Prevent copying and assignment
    VehicleData(const VehicleData&) = delete;
    VehicleData& operator=(const VehicleData&) = delete;

    Preferences preferences;

    // Status attributes
    OBDIISTATUS obdiiStatus;
    ECUSTATUS ecustatus;
    ESIMSTATUS esimStatus;

    // Volatile attributes
    int engineRPM;
    int vehicleSpeed;
    int coolantTemp;
    float engineLoad;
    float timingAdvance;
    float throttlePosition;
    float moduleVoltage;
    float longTermFuelTrim;
    float mapPressure;
    std::vector<String> dtcList;
    // when the diagnosic mode is enabled, this flag is to marker if the DTCs have been requested; Is not persisted because the DTCs request must be done every time the system is started
    int requestDTCsOnStarted; 

    // Persistent attributes
    float gasolineLevel;
    float tankCapacity;
    float fuelConsumptionFactor;
    double totalDistance;   // double precision to avoid losing sub-meter
                             // increments once mileage reaches typical
                             // lifetime values (e.g. 100,000+ km) with float.
    float tripConsumption;
    int diagnoseMode; // 0 = disabled, 1 = enabled
    int syncVersion;

    // Default constants for persistence
    static constexpr float DEFAULT_GASOLINE_LEVEL = 45.0f;
    static constexpr float DEFAULT_TANK_CAPACITY = 45.0f;
    static constexpr float DEFAULT_FUEL_CONSUMPTION_FACTOR = 0.0000000078;
    static constexpr double DEFAULT_TOTAL_DISTANCE = 0.0;
    static constexpr float DEFAULT_TRIP_CONSUMPTION = 0.0f;
    static constexpr int DEFAULT_SYNC_VERSION = 0;
    static constexpr int DEFAULT_DIAGNOSE_MODE = 0;
};

#endif // VEHICLE_DATA_H
