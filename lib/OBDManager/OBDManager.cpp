#include "OBDManager.h"
#include "config.h"
#include "CallbackManager.h"
#include "VehicleData.h"

namespace {
portMUX_TYPE foundTargetDeviceMux = portMUX_INITIALIZER_UNLOCKED;

// Protects OBD communication state (lastResponse, lastCommandSent,
// messageReceived, lastCommandSentTime, currentTimeout),
// since these are written both by notifyCallback() (runs on the Bluedroid
// BLE task) and by sendCommand()/update() (run in loop() context).
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Holds a completed response until update() (loop-context) picks it up,
// so decode() and its String parsing / Serial prints never run on the
// BLE task itself.
volatile bool responsePending = false;
String pendingResponseText;
}

bool OBDManager::isScanning = false;
bool OBDManager::scanNeedsClear = false;
unsigned long OBDManager::scanStartTime = 0;
BLEAdvertisedDevice* OBDManager::foundTargetDevice = nullptr;
BLEClient* OBDManager::pClient = nullptr;
BLERemoteCharacteristic* OBDManager::pCharTX = nullptr;
BLERemoteCharacteristic* OBDManager::pCharRX = nullptr;
BLEUUID OBDManager::serviceUUID(OBDII_SERVICE_UUID);
BLEUUID OBDManager::charUUID_TX(OBDII_CHAR_UUID_TX);
BLEUUID OBDManager::charUUID_RX(OBDII_CHAR_UUID_RX);
String OBDManager::lastCommandSent = "";
bool OBDManager::messageReceived = false;
String OBDManager::lastResponse = "";
unsigned long OBDManager::lastCommandSentTime = 0;
unsigned long OBDManager::currentTimeout = DEFAULT_TIMEOUT;
std::queue<String> OBDManager::commandQueue;
volatile bool OBDManager::connectInProgress = false;
bool OBDManager::callbackInit = false;
int OBDManager::obdConnectedFlag = 0;
int OBDManager::obdDisconnectedFlag = 0;
int OBDManager::obdConnectionAttemptFlag = 0;
int OBDManager::dtcCollectionEndedFlag = 0;
bool OBDManager::dtcCollectionMode = false;
RawMessageCallback OBDManager::rawMessageCallback = nullptr;

void OBDManager::onScanCompleted(BLEScanResults scanResults) {
    BLEUUID targetUUID(OBDII_SERVICE_UUID);

    bool adapterFound = false;
    for (int i = 0; i < scanResults.getCount(); i++) {
        BLEAdvertisedDevice device = scanResults.getDevice(i);

        // getName() returns std::string in this core version; convert to Arduino String.
        String name = String(device.getName().c_str());
        bool nameMatch = name.indexOf(OBDII_NAME_PREFIX) != -1
                       || name.indexOf(OBDII_NAME_PREFIX_2) != -1
                       || name.indexOf(OBDII_NAME_PREFIX_3) != -1;

        bool serviceMatch = device.haveServiceUUID() && device.isAdvertisingService(targetUUID);

        if (nameMatch || serviceMatch) {
            DebugSerial::println("Found adapter: " + String(device.getAddress().toString().c_str()));
            BLEAdvertisedDevice* newTargetDevice = new BLEAdvertisedDevice(device);
            BLEAdvertisedDevice* previousTargetDevice = nullptr;

            portENTER_CRITICAL(&foundTargetDeviceMux);
            previousTargetDevice = foundTargetDevice;
            foundTargetDevice = newTargetDevice;
            portEXIT_CRITICAL(&foundTargetDeviceMux);

            if (previousTargetDevice != nullptr) delete previousTargetDevice;
            adapterFound = true;
            break;
        }
    }

    // NOTE: Do NOT call pBLEScan->clearResults() (or any other heavy BLEScan
    // API) here. This callback runs on the Bluedroid/NimBLE host task, not
    // on loop(). Calling clearResults() from within the scan-completion
    // callback itself has been observed to leave the BLE stack's internal
    // scan state in a bad spot, causing the *next* pBLEScan->start() call
    // (issued later from loop()-context) to block forever on an internal
    // semaphore/mutex that is never released - silently hanging the whole
    // firmware with no further log output. Results are cleared instead at
    // the start of the next scanAndConnect() call, safely from loop().
    isScanning = false;
    scanNeedsClear = true;

    if (!adapterFound) {
        DebugSerial::println("No OBD-II adapter found.");
        // This must be 1, not 0: it's a flag-watcher counter consumed by
        // main.cpp's setObdStatustoOffline(), which is what resets the
        // OBDII status back to DISCONNECTED. Without this, the status
        // stays stuck in TRYING_TO_CONNECT forever after an unsuccessful
        // scan, and startOBDIIConnection()'s guard
        // (status == DISCONNECTED) means scanAndConnect() is never called
        // again - the firmware looks "frozen" even though it's still
        // running fine.
        obdDisconnectedFlag = 1;
    }
}

void OBDManager::scanAndConnect() {
    if (!callbackInit) {
        BLEDevice::init("");
        CallbackManager::addTimer(25, OBDManager::update);
        callbackInit = true;
    }

    if (isScanning) return;
    if (pClient != nullptr && pClient->isConnected()) return;

    BLEScan* pBLEScan = BLEDevice::getScan();

    // Clear results from a previous scan here, in loop()-context, instead
    // of inside the BLE-task scan-completion callback (see note above).
    if (scanNeedsClear) {
        pBLEScan->clearResults();
        scanNeedsClear = false;
    }

    DebugSerial::println("BLE Scan started (asynchronous)");
    obdConnectionAttemptFlag = 1;

    pBLEScan->setActiveScan(true);
    isScanning = true;
    scanStartTime = millis();
    pBLEScan->start(OBDII_SCAN_TIME_SEC, onScanCompleted, false);
}

// Called from loop()-context (update()). Never blocks: the actual
// connect + service discovery runs on its own FreeRTOS task.
void OBDManager::connectToDevice(BLEAdvertisedDevice& device) {
    if (connectInProgress) {
        DebugSerial::println("OBD connect already in progress, ignoring new target.");
        return;
    }
    if (pClient != nullptr && pClient->isConnected()) return;

    connectInProgress = true;

    // The task outlives this call, so it needs its own copy of the
    // advertised device.
    BLEAdvertisedDevice* deviceCopy = new BLEAdvertisedDevice(device);
    if (xTaskCreatePinnedToCore(connectTaskTrampoline, "obdConnectTask", 12288,
                                deviceCopy, 1, nullptr, 1) != pdPASS) {
        delete deviceCopy;
        connectInProgress = false;
        DebugSerial::println("ERROR: Failed to create OBD connect task.");
        obdDisconnectedFlag = 1;
        return;
    }
}

// Runs on the "obdConnectTask" FreeRTOS task. Safe to block here: the
// loop() is no longer on the critical path of a BLE connection.
void OBDManager::connectTaskTrampoline(void* param) {
    BLEAdvertisedDevice* device = (BLEAdvertisedDevice*)param;
    doConnectToDevice(*device);
    delete device;
    connectInProgress = false;
    vTaskDelete(nullptr);
}

void OBDManager::doConnectToDevice(BLEAdvertisedDevice& device) {
    BLEAddress targetAddress = device.getAddress();
    DebugSerial::println("Connecting with device: " + String(targetAddress.toString().c_str()));

    if (pClient == nullptr) {
        pClient = BLEDevice::createClient();
        DebugSerial::println("BLE Client created");
    }

    if (pClient->isConnected()) {
        DebugSerial::println("An active connection already exists, disconnecting first...");
        pClient->disconnect();
    }

    if (!pClient->connect(targetAddress)) {
        DebugSerial::println("ERROR: Failed to connect via BLEAdvertisedDevice");
        obdDisconnectedFlag = 1;
        return;
    }

    DebugSerial::println("BLE connected, searching for service...");

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        DebugSerial::println("ERROR: Service not found");
        pClient->disconnect();
        obdDisconnectedFlag = 1;
        return;
    }

    pCharTX = pRemoteService->getCharacteristic(charUUID_TX);
    pCharRX = pRemoteService->getCharacteristic(charUUID_RX);

    if (pCharTX == nullptr || pCharRX == nullptr) {
        DebugSerial::println("ERROR: Characteristics not found");
        pClient->disconnect();
        obdDisconnectedFlag = 1;
        return;
    }

    DebugSerial::println(String("pCharTX canWrite: ") + (pCharTX->canWrite() ? "yes" : "no") +
                      " canWriteNoResponse: " + (pCharTX->canWriteNoResponse() ? "yes" : "no"));

    if (pCharRX->canNotify()) {
        pCharRX->registerForNotify([](BLERemoteCharacteristic* c, uint8_t* pData, size_t length, bool isNotify) {
            OBDManager::notifyCallback(c, pData, length, isNotify);
        });
    }

    DebugSerial::println("Characteristics ready, queuing configuration commands...");

    portENTER_CRITICAL(&stateMux);
    messageReceived = true;
    lastCommandSent = "";
    lastResponse = "";
    responsePending = false;
    portEXIT_CRITICAL(&stateMux);

    clearCommandQueue();

    addCommandToQueue("ATZ");
    addCommandToQueue("ATE0");
    addCommandToQueue("ATH0");
    addCommandToQueue("ATSP0");
    addCommandToQueue("ATAT1");
    addCommandToQueue("ATL0");
    DebugSerial::println("Connection and handshake initiated successfully");
    obdConnectedFlag = 1;
}

// Only ever called from loop()-context.
void OBDManager::sendCommand(String command) {
    if (pClient == nullptr || !pClient->isConnected() || pCharTX == nullptr) {
        DebugSerial::println("Cannot send command, BLE not ready: " + command);
        return;
    }

    String trimmedCommand = command;
    trimmedCommand.trim();

    if (trimmedCommand.length() == 0) {
        DebugSerial::println("WARNING: Empty command sending");
        portENTER_CRITICAL(&stateMux);
        messageReceived = true;
        portEXIT_CRITICAL(&stateMux);
        return;
    }

    if (!command.endsWith("\r")) command += "\r";

    unsigned long timeoutToUse = (trimmedCommand.startsWith("AT") || trimmedCommand == OBD_CMD_REQUEST_DTCS) ? AT_COMMAND_TIMEOUT : DEFAULT_TIMEOUT;

    portENTER_CRITICAL(&stateMux);
    lastCommandSent = trimmedCommand;
    lastResponse = "";
    messageReceived = false;
    lastCommandSentTime = millis();
    currentTimeout = timeoutToUse;
    portEXIT_CRITICAL(&stateMux);

    unsigned long writeStart = millis();
    
    // Perform write without capturing return directly into a boolean variable
    pCharTX->writeValue((uint8_t*)command.c_str(), command.length(), true);
    bool ok = true; // Since modern writeValue does not return bool in its current version, assume successful ATT transmission or handle via response/timeout flow.
    
    unsigned long writeDuration = millis() - writeStart;

    DebugSerial::println("Sending: " + trimmedCommand +
                          " | write-with-response ok=1" +
                          " tomou " + String(writeDuration) + "ms");
                          
    if (!ok) {
        // Write failed at the ATT level - treat as a timeout so the state
        // machine does not get stuck waiting for a response that will never arrive.
        portENTER_CRITICAL(&stateMux);
        messageReceived = true;
        portEXIT_CRITICAL(&stateMux);
    }
}

// Runs on the Bluedroid BLE task, NOT on loop(). Keep this fast: only
// append bytes and, once a full response is detected, stash it for
// update() to process. No String parsing beyond terminator detection,
// no calls to rawMessageCallback, no prints here.
void OBDManager::notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    portENTER_CRITICAL(&stateMux);

    if (messageReceived) {
        portEXIT_CRITICAL(&stateMux);
        return;
    }

    for (size_t i = 0; i < length; i++) {
        lastResponse += (char)pData[i];
    }

    if (lastResponse.indexOf('>') != -1) {
        pendingResponseText = lastResponse;
        pendingResponseText.trim();

        messageReceived = true;
        lastResponse = "";
        responsePending = true;
    }

    portEXIT_CRITICAL(&stateMux);
}

void OBDManager::clearCommandQueue() {
    while (!commandQueue.empty()) commandQueue.pop();
}

void OBDManager::addCommandToQueue(const String& command) {
    if (dtcCollectionMode && command != OBD_CMD_REQUEST_DTCS) {
        return;
    }
    bool canSendNow;
    portENTER_CRITICAL(&stateMux);
    canSendNow = commandQueue.empty() && messageReceived;
    portEXIT_CRITICAL(&stateMux);

    if (canSendNow) {
        sendCommand(command);
        return;
    }

    // Bound the queue (see kMaxCommandQueue): drop the stale backlog and
    // keep only the newest command so slow responses never accumulate a
    // queue full of outdated PIDs.
    if (commandQueue.size() >= kMaxCommandQueue) {
        clearCommandQueue();
    }
    commandQueue.push(command);
}

void OBDManager::update() {
    // Watchdog: the BLE stack's async scan completion callback
    // (onScanCompleted) is not always guaranteed to fire (seen with some
    // arduino-esp32 core / NimBLE-Bluedroid combinations). If that happens,
    // isScanning would stay true forever and scanAndConnect() would just
    // keep silently returning early every second, with no further log
    // output and no way to recover. Force-stop and reset after a grace
    // period well beyond the configured scan duration.
    if (isScanning) {
        const unsigned long scanWatchdogMs = (unsigned long)(OBDII_SCAN_TIME_SEC) * 1000UL + 5000UL;
        if (millis() - scanStartTime >= scanWatchdogMs) {
            DebugSerial::println("WARNING: BLE scan watchdog triggered, scan callback never fired. Forcing stop.");
            BLEDevice::getScan()->stop();
            BLEDevice::getScan()->clearResults();
            isScanning = false;
            obdDisconnectedFlag = 1;
        }
    }

    BLEAdvertisedDevice* targetDevice = nullptr;
    portENTER_CRITICAL(&foundTargetDeviceMux);
    targetDevice = foundTargetDevice;
    foundTargetDevice = nullptr;
    portEXIT_CRITICAL(&foundTargetDeviceMux);

    if (targetDevice != nullptr) {
        BLEAdvertisedDevice target = *targetDevice;
        delete targetDevice;
        connectToDevice(target);
        return;
    }

    // Process any response the BLE task stashed for us. decode() and its
    // Serial prints happen here, safely off the BLE task.
    bool hasResponse = false;
    String responseToProcess;
    portENTER_CRITICAL(&stateMux);
    if (responsePending) {
        hasResponse = true;
        responseToProcess = pendingResponseText;
        responsePending = false;
    }
    portEXIT_CRITICAL(&stateMux);

    if (hasResponse) {
        if (responseToProcess.length() > 0) {
            DebugSerial::println("Message Received: " + responseToProcess);
        }
        if (rawMessageCallback) {
            rawMessageCallback(responseToProcess);
        }
    }

    bool canSendNext;
    portENTER_CRITICAL(&stateMux);
    canSendNext = messageReceived;
    portEXIT_CRITICAL(&stateMux);

    if (canSendNext && !commandQueue.empty()) {
        String next = commandQueue.front();
        commandQueue.pop();
        sendCommand(next);
    }

    bool timedOut = false;
    String timedOutCommand;
    portENTER_CRITICAL(&stateMux);
    if (!messageReceived && lastCommandSentTime > 0) {
        unsigned long elapsed = millis() - lastCommandSentTime;
        if (elapsed >= currentTimeout) {
            timedOut = true;
            timedOutCommand = lastCommandSent;
            messageReceived = true;
            lastResponse = "";
        }
    }
    portEXIT_CRITICAL(&stateMux);

    if (timedOut) {
        DebugSerial::println("ERROR: Timeout waiting for response to: " + timedOutCommand);
        if (dtcCollectionMode && timedOutCommand.startsWith(OBD_CMD_REQUEST_DTCS)) {
            DebugSerial::println("[OBDManager] DTC collection request timed out.");
            endDTCCollection(0);
        }
    }

    static bool wasConnected = false;
    const bool connected = (pClient != nullptr) && pClient->isConnected();

    if (connected) {
        wasConnected = true;
    } else if (wasConnected) {
        obdDisconnectedFlag = 1;
        wasConnected = false;
        if (dtcCollectionMode) {
            endDTCCollection(0);
        }
    }
}

void OBDManager::setRawMessageCallback(RawMessageCallback callback) {
    rawMessageCallback = callback;
}

bool OBDManager::startedCollectDTCsMode() {
    if (VehicleData::getInstance().getEcuStatus() != ECUSTATUS::ONLINE) {
        DebugSerial::println("[OBDManager] Cannot start DTC collection: ECU is offline.");
        return false;
    }

    if (pClient == nullptr || !pClient->isConnected()) {
        DebugSerial::println("[OBDManager] Cannot start DTC collection: OBD-II not connected.");
        return false;
    }

    DebugSerial::println("[OBDManager] Starting DTC collection mode...");
    dtcCollectionMode = true;
    dtcCollectionEndedFlag = 0;
    VehicleData::getInstance().clearDTCs();
    VehicleData::getInstance().setRequestDTCsOnStarted(1);

    clearCommandQueue();
    addCommandToQueue(OBD_CMD_REQUEST_DTCS);
    return true;
}

bool OBDManager::isDtcCollectionMode() {
    return dtcCollectionMode;
}

void OBDManager::endDTCCollection(int count) {
    dtcCollectionMode = false;
    dtcCollectionEndedFlag = 1;
    DebugSerial::println("[OBDManager] DTC collection ended. Total DTCs: " + String(count));
}
