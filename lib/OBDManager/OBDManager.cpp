#include "OBDManager.h"
#include "config.h"
#include "CallbackManager.h"

namespace {
portMUX_TYPE foundTargetDeviceMux = portMUX_INITIALIZER_UNLOCKED;

// Protects only the SHARED handoff state between notifyCallback() (runs
// on the Bluedroid BLE task) and sendCommand()/update() (run in loop()
// context): messageReceived, lastCommandSentTime, currentTimeout, and the
// pending-response handoff flags/text below.
//
// IMPORTANT: nothing that can allocate memory (String concatenation,
// indexOf, remove, etc.) may run while this critical section is held.
// On ESP32, portENTER_CRITICAL is a spinlock that can also block the
// other core; calling malloc/realloc (which String operations can do
// internally) while holding it is undefined-behavior-adjacent and can
// corrupt BLE stack timing or crash outright. That is why all echo
// stripping now happens in update(), never in notifyCallback().
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Only ever touched by the BLE task, sequentially (BLE notifications for
// one characteristic are delivered one at a time, never concurrently), so
// this needs no locking of its own. It accumulates raw bytes exactly as
// received, WITHOUT echo stripping.
String rawAccum;

// Handoff to update(): filled by notifyCallback() under stateMux with a
// plain assignment only (no String work while locked), consumed by
// update() to do the actual echo-stripping/decoding off the BLE task.
volatile bool responsePending = false;
String pendingRawResponse;
}

bool OBDManager::isScanning = false;
BLEAdvertisedDevice* OBDManager::foundTargetDevice = nullptr;
BLEClient* OBDManager::pClient = nullptr;
BLERemoteCharacteristic* OBDManager::pCharTX = nullptr;
BLERemoteCharacteristic* OBDManager::pCharRX = nullptr;
BLEUUID OBDManager::serviceUUID(OBDII_SERVICE_UUID);
BLEUUID OBDManager::charUUID_TX(OBDII_CHAR_UUID_TX);
BLEUUID OBDManager::charUUID_RX(OBDII_CHAR_UUID_RX);
bool OBDManager::echoDisabled = false;
String OBDManager::lastCommandSent = "";
bool OBDManager::messageReceived = false;
bool OBDManager::pendingSend = false;
String OBDManager::pendingCommand = "";
String OBDManager::lastResponse = "";
unsigned long OBDManager::lastCommandSentTime = 0;
unsigned long OBDManager::currentTimeout = DEFAULT_TIMEOUT;
std::queue<String> OBDManager::commandQueue;
int OBDManager::sendMessageFlag = 0;
bool OBDManager::callbackInit = false;
int OBDManager::obdConnectedFlag = 0;
int OBDManager::obdDisconnectedFlag = 0;
int OBDManager::obdConnectionAttemptFlag = 0;
RawMessageCallback OBDManager::rawMessageCallback = nullptr;

OBDManager::OBDManager() {}
OBDManager::~OBDManager() {}

void OBDManager::onScanCompleted(BLEScanResults scanResults) {
    BLEScan* pBLEScan = BLEDevice::getScan();
    BLEUUID targetUUID(OBDII_SERVICE_UUID);

    bool adapterFound = false;
    for (int i = 0; i < scanResults.getCount(); i++) {
        BLEAdvertisedDevice device = scanResults.getDevice(i);

        bool nameMatch = device.getName().find(OBDII_NAME_PREFIX) != std::string::npos
                       || device.getName().find(OBDII_NAME_PREFIX_2) != std::string::npos
                       || device.getName().find(OBDII_NAME_PREFIX_3) != std::string::npos;

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

    pBLEScan->clearResults();
    isScanning = false;

    if (!adapterFound) {
        DebugSerial::println("No OBD-II adapter found.");
        obdDisconnectedFlag = 0;
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

    DebugSerial::println("BLE Scan started (asynchronous)");
    obdConnectionAttemptFlag = 1;

    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    isScanning = true;
    pBLEScan->start(OBDII_SCAN_TIME_SEC, onScanCompleted, false);
}

void OBDManager::connectToDevice(BLEAdvertisedDevice& device) {
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

    // Reset state BEFORE registering for notifications, so there is no
    // window where a stray/late notification could be processed against
    // leftover state from a previous connection.
    rawAccum = "";
    echoDisabled = false;
    lastCommandSent = "";
    lastResponse = "";
    pendingRawResponse = "";

    portENTER_CRITICAL(&stateMux);
    messageReceived = true;
    responsePending = false;
    portEXIT_CRITICAL(&stateMux);

    if (pCharRX->canNotify()) {
        pCharRX->registerForNotify([](BLERemoteCharacteristic* c, uint8_t* pData, size_t length, bool isNotify) {
            OBDManager::notifyCallback(c, pData, length, isNotify);
        });
    }

    DebugSerial::println("Characteristics ready, queuing configuration commands...");

    clearCommandQueue();

    addCommandToQueue("ATZ");
    addCommandToQueue("ATE0");
    addCommandToQueue("ATH0");
    addCommandToQueue("ATSP0");
    addCommandToQueue("ATAT1");
    addCommandToQueue("ATL0");
    sendMessageFlag = 1;
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

    unsigned long timeoutToUse = trimmedCommand.startsWith("AT") ? AT_COMMAND_TIMEOUT : DEFAULT_TIMEOUT;

    // loop()-context only from here on for lastCommandSent/lastResponse/
    // rawAccum, so no lock needed for those; only the flags/timestamp
    // shared with the BLE task go through stateMux.
    lastCommandSent = trimmedCommand;
    lastResponse = "";
    rawAccum = "";

    portENTER_CRITICAL(&stateMux);
    messageReceived = false;
    lastCommandSentTime = millis();
    currentTimeout = timeoutToUse;
    portEXIT_CRITICAL(&stateMux);

    unsigned long writeStart = millis();

    pCharTX->writeValue((uint8_t*)command.c_str(), command.length(), true);
    bool ok = true;

    unsigned long writeDuration = millis() - writeStart;

    DebugSerial::println("Sending: " + trimmedCommand +
                          " | write-with-response ok=1" +
                          " tomou " + String(writeDuration) + "ms");

    if (!ok) {
        portENTER_CRITICAL(&stateMux);
        messageReceived = true;
        portEXIT_CRITICAL(&stateMux);
    }
}

// Runs on the Bluedroid BLE task, NOT on loop(). Keep this fast and
// allocation-free while holding stateMux: only plain assignments are
// allowed inside the critical section. String concatenation/indexOf/
// remove can trigger malloc/realloc internally, and doing that while a
// portMUX spinlock is held is what breaks things under fast, back-to-back
// command traffic (low delay) even though it "mostly works" with bigger
// delays between commands. All echo stripping now happens in update().
void OBDManager::notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    bool alreadyDone;
    portENTER_CRITICAL(&stateMux);
    alreadyDone = messageReceived;
    portEXIT_CRITICAL(&stateMux);
    if (alreadyDone) return;

    // Accumulate raw bytes and scan for the terminator OUTSIDE the
    // critical section. rawAccum is touched only by this task, so no
    // lock is needed for it.
    for (size_t i = 0; i < length; i++) {
        rawAccum += (char)pData[i];
    }

    if (rawAccum.indexOf('>') == -1) {
        return;
    }

    String finished = rawAccum;
    rawAccum = "";

    portENTER_CRITICAL(&stateMux);
    if (!messageReceived) {
        pendingRawResponse = finished;
        messageReceived = true;
        responsePending = true;
        sendMessageFlag = 1;
    }
    portEXIT_CRITICAL(&stateMux);
}

void OBDManager::clearCommandQueue() {
    while (!commandQueue.empty()) commandQueue.pop();
}

void OBDManager::addCommandToQueue(const String& command) {
    bool canSendNow;
    portENTER_CRITICAL(&stateMux);
    canSendNow = commandQueue.empty() && messageReceived;
    portEXIT_CRITICAL(&stateMux);

    if (canSendNow) {
        sendCommand(command);
        return;
    }
    commandQueue.push(command);
}

void OBDManager::update() {
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

    // Pull the raw response out of the shared handoff (cheap, lock held
    // only for the assignment/flag copy), then do all the actual String
    // work — echo stripping, OK detection, decode() — here, off the BLE
    // task.
    bool hasResponse = false;
    String rawResponse;
    portENTER_CRITICAL(&stateMux);
    if (responsePending) {
        hasResponse = true;
        rawResponse = pendingRawResponse;
        responsePending = false;
    }
    portEXIT_CRITICAL(&stateMux);

    if (hasResponse) {
        String processed = rawResponse;
        bool echoJustDisabled = false;

        // Echo stripping now runs entirely in loop() context.
        if (!echoDisabled && lastCommandSent.length() > 0) {
            int echoIndex = processed.indexOf(lastCommandSent);
            if (echoIndex != -1) {
                processed.remove(echoIndex, lastCommandSent.length());
            }
        }
        while (processed.length() > 0 && (processed[0] == '\r' || processed[0] == '\n')) {
            processed.remove(0, 1);
        }
        processed.trim();

        if (!echoDisabled && lastCommandSent == "ATE0" && processed.indexOf("OK") != -1) {
            echoDisabled = true;
            echoJustDisabled = true;
        }

        if (processed.length() > 0) {
            DebugSerial::println("Message Received: " + processed);
        }
        if (echoJustDisabled) {
            DebugSerial::println("Echo successfully disabled (ATE0 confirmed)");
        }
        if (rawMessageCallback) {
            rawMessageCallback(processed);
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
        }
    }
    portEXIT_CRITICAL(&stateMux);

    if (timedOut) {
        rawAccum = "";
        DebugSerial::println("ERROR: Timeout waiting for response to: " + timedOutCommand);
    }

    static bool wasConnected = false;
    const bool connected = (pClient != nullptr) && pClient->isConnected();

    if (connected) {
        wasConnected = true;
    } else if (wasConnected) {
        obdDisconnectedFlag = 1;
        wasConnected = false;
    }
}

void OBDManager::setRawMessageCallback(RawMessageCallback callback) {
    rawMessageCallback = callback;
}