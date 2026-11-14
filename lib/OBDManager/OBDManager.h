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
#include "DebugSerial.h"

/**
 * @class OBDManager
 * @brief Simplified manager focused on Stage 1: Scanning and Detecting the OBD-II Adapter.
 */
class OBDManager {
public:
    static void scanAndConnect();
    static void clearCommandQueue();
    static void addCommandToQueue(const String& command);
    static void setRawMessageCallback(RawMessageCallback callback);
    static bool startedCollectDTCsMode();
    static bool isDtcCollectionMode();
    static void endDTCCollection(int count = 0);

    
    static int obdConnectedFlag;
    static int obdDisconnectedFlag;
    static int obdConnectionAttemptFlag;
    static int dtcCollectionEndedFlag;

private:
    // Max number of queued (unsent) commands. The 400ms polling timer in
    // main.cpp adds up to 2 commands per tick while commands are drained one
    // at a time; without a cap a slow/timing-out adapter would grow the queue
    // indefinitely with stale PIDs. When full, the backlog is dropped so the
    // freshest requests always go out.
    static constexpr size_t kMaxCommandQueue = 4;

    // True while the background connect task is running (see
    // connectTaskTrampoline). Prevents spawning a second connect against a
    // new scan result while the first is still in flight.
    static volatile bool connectInProgress;

    static bool isScanning;
    static bool scanNeedsClear;
    static unsigned long scanStartTime;
    static bool dtcCollectionMode;
    static BLEAdvertisedDevice* foundTargetDevice;
    static void onScanCompleted(BLEScanResults scanResults);
    static BLEClient* pClient;
    static RawMessageCallback rawMessageCallback;
    // Non-blocking dispatcher: hands the connect off to a dedicated task.
    static void connectToDevice(BLEAdvertisedDevice& device);
    // The actual (blocking) connect + service discovery, run on its own
    // FreeRTOS task: BLEClient::connect() / getService() can take tens of
    // seconds against a slow or vanishing adapter, and doing that inside
    // loop() used to freeze the display, OBD polling and every other timer.
    static void doConnectToDevice(BLEAdvertisedDevice& device);
    static void connectTaskTrampoline(void* param);
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
    static String lastCommandSent;
    static bool messageReceived;
    static String lastResponse;
    static unsigned long lastCommandSentTime;
    static unsigned long currentTimeout;
    static bool callbackInit;
    
    static void sendCommand(String command);
    static std::queue<String> commandQueue;
};

#endif // OBD_MANAGER_H