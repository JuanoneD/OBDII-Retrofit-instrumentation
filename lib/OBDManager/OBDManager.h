#ifndef OBD_MANAGER_H
#define OBD_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <functional>
#include <queue>
#include "config.h"
#include "CallbackManager.h"

/**
 * @class OBDManager
 * @brief Simplified manager focused on Stage 1: Scanning and Detecting the OBD-II Adapter.
 */
class OBDManager {
public:
    OBDManager();
    ~OBDManager();

    static void scanAndConnect();
    static void clearCommandQueue();
    static void addCommandToQueue(const String& command);
    static void setDebugSerial(Stream* serial);   
    static void setRawMessageCallback(RawMessageCallback callback);
    
    static int obdConnectedFlag;
    static int obdDisconnectedFlag;
    
private:
    static BLEClient* pClient;
    static RawMessageCallback rawMessageCallback;
    static void connectToDevice(BLEAdvertisedDevice& device);
    static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
    static void update();  // Process command queue and check timeouts
        
    // BLE Characteristics
    static BLERemoteCharacteristic* pCharTX;
    static BLERemoteCharacteristic* pCharRX;
    
    // UUIDs
    static BLEUUID serviceUUID;
    static BLEUUID charUUID_TX;
    static BLEUUID charUUID_RX;
    
    // State variables
    static bool echoDisabled;
    static String lastCommandSent;
    static bool messageReceived;
    static bool pendingSend;
    static String pendingCommand;
    static String lastResponse;
    static unsigned long lastCommandSentTime;
    static unsigned long currentTimeout;
    static int sendMessageFlag;
    static bool callbackInit;
    
    static void sendCommand(String command);
    static std::queue<String> commandQueue;

    static Stream* debugSerial;
    static void debugPrint(const String& message);
};

#endif // OBD_MANAGER_H