#include "OBDDecoder.h"

int OBDDecoder::ecu_online_flag = 0;
int OBDDecoder::ecu_offline_flag = 0;

OBDDecoder::OBDDecoder() {
}

OBDDecoder::~OBDDecoder() {
}

void OBDDecoder::decode(const String& rawResponse) {
    String clean = rawResponse;
    clean.replace(" ", "");
    clean.replace(">", "");
    clean.replace("\r", "");
    clean.replace("\n", "");
    clean.toUpperCase();

    int idx = clean.indexOf("41");
    if (idx == -1 || clean.length() < (unsigned int)(idx + 4)) {
        DebugSerial::println("Offline response: " + rawResponse);
        ecu_offline_flag = 1;
        return;
    }

    String pidStr = clean.substring(idx + 2, idx + 4);
    uint8_t pid = (uint8_t)strtoul(pidStr.c_str(), nullptr, 16);
    String data = clean.substring(idx + 4);

    switch (pid) {
        case PID_ENGINE_RPM:
            decodeRPM(data);
            break;
        case PID_VEHICLE_SPEED:
            decodeVehicleSpeed(data);
            break;
        case PID_COOLANT_TEMP:
            decodeCoolantTemp(data);
            break;
        case PID_ENGINE_LOAD:
            decodeEngineLoad(data);
            break;
        case PID_TIMING_ADVANCE:
            decodeTimingAdvance(data);
            break;
        case PID_THROTTLE_POSITION:
            decodeThrottlePosition(data);
            break;
        case PID_CONTROL_MODULE_VOLTAGE:
            decodeControlModuleVoltage(data);
            break;
        case PID_LONG_TERM_FUEL_TRIM:
            decodeLongTermFuelTrim(data);
            break;
        case PID_MAP:
            decodeMAP(data);
            break;
        default:
            DebugSerial::println("Unknown PID: " + pidStr);
            break;
    }
    ecu_online_flag = 1;
}

// PID 010C: ((A * 256) + B) / 4 [RPM]
void OBDDecoder::decodeRPM(const String& data) {
    if (data.length() < 4) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    uint8_t b = (uint8_t)strtoul(data.substring(2, 4).c_str(), nullptr, 16);
    int rpm = (int)(((a * 256) + b) / 4);
    VehicleData::getInstance().setEngineRPM(rpm);
    DebugSerial::println("RPM: " + String(rpm) + " RPM");
}

// PID 010D: A [km/h]
void OBDDecoder::decodeVehicleSpeed(const String& data) {
    if (data.length() < 2) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    int speed = a;
    VehicleData::getInstance().setVehicleSpeed(speed);
    DebugSerial::println("Speed: " + String(speed) + " km/h");
}

// PID 0105: A - 40 [°C]
void OBDDecoder::decodeCoolantTemp(const String& data) {
    if (data.length() < 2) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    int temp = (int)a - 40;
    VehicleData::getInstance().setCoolantTemp(temp);
    DebugSerial::println("Coolant Temp: " + String(temp) + " C");
}

// PID 0104: (A * 100) / 255 [%]
void OBDDecoder::decodeEngineLoad(const String& data) {
    if (data.length() < 2) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    float load = (a * 100.0f) / 255.0f;
    VehicleData::getInstance().setEngineLoad(load);
    DebugSerial::println("Engine Load: " + String(load) + " %");
}

// PID 010E: (A / 2) - 64 [°]
void OBDDecoder::decodeTimingAdvance(const String& data) {
    if (data.length() < 2) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    float advance = (a / 2.0f) - 64.0f;
    VehicleData::getInstance().setTimingAdvance(advance);
    DebugSerial::println("Timing Advance: " + String(advance) + " deg");
}

// PID 0111: (A * 100) / 255 [%]
void OBDDecoder::decodeThrottlePosition(const String& data) {
    if (data.length() < 2) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    float throttle = (a * 100.0f) / 255.0f;
    DebugSerial::println("Throttle Position: " + String(throttle) + " %");
}

// PID 0142: ((A * 256) + B) / 1000 [Volts]
void OBDDecoder::decodeControlModuleVoltage(const String& data) {
    if (data.length() < 4) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    uint8_t b = (uint8_t)strtoul(data.substring(2, 4).c_str(), nullptr, 16);
    float voltage = ((a * 256.0f) + b) / 1000.0f;
    VehicleData::getInstance().setModuleVoltage(voltage);
    DebugSerial::println("Control Module Voltage: " + String(voltage) + " V");
}

// PID 0107: (A * 100 / 128) - 100 [%]
void OBDDecoder::decodeLongTermFuelTrim(const String& data) {
    if (data.length() < 2) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    float trim = (a * 100.0f / 128.0f) - 100.0f;
    VehicleData::getInstance().setLongTermFuelTrim(trim);
    DebugSerial::println("Long Term Fuel Trim: " + String(trim) + " %");
}

// PID 010B: A [kPa]
void OBDDecoder::decodeMAP(const String& data) {
    if (data.length() < 2) return;
    uint8_t a = (uint8_t)strtoul(data.substring(0, 2).c_str(), nullptr, 16);
    float map = (float)a;
    VehicleData::getInstance().setMapPressure(map);
    DebugSerial::println("MAP: " + String(map) + " kPa");
}
