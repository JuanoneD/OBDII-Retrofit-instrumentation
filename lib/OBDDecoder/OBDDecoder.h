#ifndef OBD_DECODER_H
#define OBD_DECODER_H

#include <Arduino.h>
#include "config.h"
#include "DebugSerial.h"

class OBDDecoder {
public:
    OBDDecoder();
    ~OBDDecoder();

    static void decode(const String& rawResponse);

// Signals
    static int ecu_online_flag;
    static int ecu_offline_flag;

private:
    static void decodeRPM(const String& data);
    static void decodeVehicleSpeed(const String& data);
    static void decodeCoolantTemp(const String& data);
    static void decodeEngineLoad(const String& data);
    static void decodeTimingAdvance(const String& data);
    static void decodeThrottlePosition(const String& data);
    static void decodeControlModuleVoltage(const String& data);
    static void decodeLongTermFuelTrim(const String& data);
    static void decodeMAP(const String& data);
};

#endif // OBD_DECODER_H
