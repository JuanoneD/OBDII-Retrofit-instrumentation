#include "OBDDecoder.h"
#include "OBDManager.h"
#include <algorithm>

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

    // Handle Mode 03 (DTC) response
    if (clean.indexOf("43") != -1) {
        decodeDTC(rawResponse);
        ecu_online_flag = 1;
        return;
    }

    // Handle NO DATA response while in DTC collection mode
    if (OBDManager::isDtcCollectionMode() && (clean.indexOf("NODATA") != -1 || clean.indexOf("NO DATA") != -1)) {
        decodeDTC(rawResponse);
        return;
    }

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
    speed *= 1.07; // Adjust for speedometer calibration (7% increase)
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
    VehicleData::getInstance().setThrottlePosition(throttle);
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

void OBDDecoder::decodeDTC(const String& rawResponse) {
    DebugSerial::println("[OBDDecoder] Receiving DTC response: " + rawResponse);

    std::vector<String> dtcList;

    String upper = rawResponse;
    upper.toUpperCase();

    if (upper.indexOf("NO DATA") != -1 || upper.indexOf("NODATA") != -1) {
        DebugSerial::println("[OBDDecoder] No DTCs reported (NO DATA).");
        VehicleData::getInstance().setDTCs(dtcList);
        OBDManager::endDTCCollection(0);
        return;
    }

    std::vector<uint8_t> bytes;
    int lineStart = 0;
    int len = rawResponse.length();

    while (lineStart < len) {
        int lineEnd = rawResponse.indexOf('\n', lineStart);
        if (lineEnd == -1) lineEnd = len;

        String line = rawResponse.substring(lineStart, lineEnd);
        lineStart = lineEnd + 1;

        line.trim();
        line.replace("\r", "");
        line.replace(">", "");
        line.toUpperCase();

        if (line.length() == 0) continue;

        // Strip CAN sequence index like "0:", "1:"
        int colonIdx = line.indexOf(':');
        if (colonIdx != -1 && colonIdx < 4) {
            line = line.substring(colonIdx + 1);
            line.trim();
        }

        // Collect hex characters
        String hexOnly = "";
        for (size_t k = 0; k < line.length(); k++) {
            char c = line[k];
            if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) {
                hexOnly += c;
            }
        }

        // Convert hex pairs to bytes
        for (size_t k = 0; k + 1 < hexOnly.length(); k += 2) {
            String byteStr = hexOnly.substring(k, k + 2);
            uint8_t val = (uint8_t)strtoul(byteStr.c_str(), nullptr, 16);
            bytes.push_back(val);
        }
    }

    auto addDTCBytes = [&](uint8_t b1, uint8_t b2) {
        if (b1 == 0 && b2 == 0) return;
        char hexBuf[5];
        snprintf(hexBuf, sizeof(hexBuf), "%02X%02X", b1, b2);
        String dtc = String(hexBuf);
        if (std::find(dtcList.begin(), dtcList.end(), dtc) == dtcList.end()) {
            dtcList.push_back(dtc);
        }
    };

    size_t i = 0;
    while (i < bytes.size()) {
        if (bytes[i] == 0x43) {
            i++; // skip 0x43
            if (i >= bytes.size()) break;

            // Check if there are no DTCs (e.g. 43 00)
            if (bytes[i] == 0x00 && (i + 1 >= bytes.size() || bytes[i+1] == 0x00)) {
                break;
            }

            // Find segment boundary (until next 0x43 or end)
            size_t segEnd = i;
            while (segEnd < bytes.size() && bytes[segEnd] != 0x43) {
                segEnd++;
            }
            size_t segLen = segEnd - i;

            uint8_t countCandidate = bytes[i];
            bool hasCountByte = false;

            // If segLen is ODD (1 count byte + 2k DTC bytes)
            if ((segLen % 2 != 0) && countCandidate <= 32) {
                hasCountByte = true;
            }

            if (hasCountByte) {
                i++; // skip count byte
                int remainingToCollect = countCandidate;
                while (i + 1 <= segEnd && remainingToCollect > 0) {
                    if (i + 1 < bytes.size()) {
                        uint8_t b1 = bytes[i];
                        uint8_t b2 = bytes[i + 1];
                        i += 2;
                        if (b1 != 0 || b2 != 0) {
                            addDTCBytes(b1, b2);
                            remainingToCollect--;
                        }
                    } else {
                        break;
                    }
                }
                i = segEnd;
            } else {
                // segLen is EVEN (legacy ISO 9141-2 / J1850 or direct pairs)
                while (i + 1 <= segEnd) {
                    if (i + 1 < bytes.size()) {
                        uint8_t b1 = bytes[i];
                        uint8_t b2 = bytes[i + 1];
                        i += 2;
                        addDTCBytes(b1, b2);
                    } else {
                        break;
                    }
                }
                i = segEnd;
            }
        } else {
            i++;
        }
    }

    DebugSerial::println("[OBDDecoder] Saved " + String(dtcList.size()) + " DTC(s):");
    for (const String& dtc : dtcList) {
        DebugSerial::println("  - " + dtc);
    }

    VehicleData::getInstance().setDTCs(dtcList);
    OBDManager::endDTCCollection(dtcList.size());
}
