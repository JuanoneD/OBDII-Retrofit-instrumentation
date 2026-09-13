#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <Arduino.h>
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

    float getTotalDistance() const;
    void setTotalDistance(float distance);

    float getTripConsumption() const;
    void setTripConsumption(float consumption);

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

    // Volatile attributes
    int engineRPM;
    int vehicleSpeed;
    int coolantTemp;
    float engineLoad;
    float timingAdvance;
    float moduleVoltage;
    float longTermFuelTrim;
    float mapPressure;

    // Persistent attributes
    float gasolineLevel;
    float tankCapacity;
    float fuelConsumptionFactor;
    float totalDistance;
    float tripConsumption;

    // Default constants for persistence
    static constexpr float DEFAULT_GASOLINE_LEVEL = 45.0f;
    static constexpr float DEFAULT_TANK_CAPACITY = 45.0f;
    static constexpr float DEFAULT_FUEL_CONSUMPTION_FACTOR = 0.008f;
    static constexpr float DEFAULT_TOTAL_DISTANCE = 0.0f;
    static constexpr float DEFAULT_TRIP_CONSUMPTION = 0.0f;
};

#endif // VEHICLE_DATA_H
