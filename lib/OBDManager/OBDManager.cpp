#include "OBDManager.h"
#include "config.h"
#include "CallbackManager.h"

Stream* OBDManager::debugSerial = nullptr;
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
RawMessageCallback OBDManager::rawMessageCallback = nullptr;

OBDManager::OBDManager() {
}

OBDManager::~OBDManager() {
}

void OBDManager::scanAndConnect() {
    if (!callbackInit) {
        BLEDevice::init("");
        CallbackManager::addTimer(25, OBDManager::update);
        callbackInit = true;
    }

    debugPrint("BLE Scan started");

    BLEUUID serviceUUID(OBDII_SERVICE_UUID);

    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    BLEScanResults foundDevices = pBLEScan->start(OBDII_SCAN_TIME_SEC, false);

    for (int i = 0; i < foundDevices.getCount(); i++) {
        BLEAdvertisedDevice device = foundDevices.getDevice(i);

        bool nameMatch = device.getName().find(OBDII_NAME_PREFIX) != std::string::npos
                       || device.getName().find(OBDII_NAME_PREFIX_2) != std::string::npos
                       || device.getName().find(OBDII_NAME_PREFIX_3) != std::string::npos;

        bool serviceMatch = device.haveServiceUUID() && device.isAdvertisingService(serviceUUID);

        if (nameMatch || serviceMatch) {
            debugPrint("Found adapter: " + String(device.getAddress().toString().c_str()));
            pBLEScan->clearResults();
            connectToDevice(device);
            return;
        }
    }

    pBLEScan->clearResults();
    debugPrint("No OBD-II adapter found.");
    return;
}

void OBDManager::connectToDevice(BLEAdvertisedDevice& device) {
    BLEAddress targetAddress = device.getAddress();
    debugPrint("Connecting with device: " + String(targetAddress.toString().c_str()));

    if (pClient == nullptr) {
        pClient = BLEDevice::createClient();
        debugPrint("BLE Client created");
    }

    if (pClient->isConnected()) {
        debugPrint("An active connection already exists, disconnecting first...");
        pClient->disconnect();
    }

    if (!pClient->connect(targetAddress)) {
        debugPrint("ERROR: Failed to connect via BLEAdvertisedDevice");
        return;
    }

    debugPrint("BLE connected, searching for service...");

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        debugPrint("ERROR: Service not found");
        pClient->disconnect();
        return;
    }

    pCharTX = pRemoteService->getCharacteristic(charUUID_TX);
    pCharRX = pRemoteService->getCharacteristic(charUUID_RX);

    if (pCharTX == nullptr || pCharRX == nullptr) {
        debugPrint("ERROR: Characteristics not found");
        pClient->disconnect();
        return;
    }

    if (pCharRX->canNotify()) {
        pCharRX->registerForNotify([](BLERemoteCharacteristic* c, uint8_t* pData, size_t length, bool isNotify) {
            OBDManager::notifyCallback(c, pData, length, isNotify);
        });
    }

    debugPrint("Characteristics ready, queuing configuration commands...");

    // Reset all state for a new connection, including echo
    // (the ATZ resets the adapter, so echo returns to being enabled by default)
    clearCommandQueue();
    messageReceived = true;
    echoDisabled = false;
    lastCommandSent = "";

    addCommandToQueue("ATZ");
    addCommandToQueue("ATE0");
    addCommandToQueue("ATH0");
    addCommandToQueue("ATSP1");
    addCommandToQueue("ATAT1");
    addCommandToQueue("ATL0");
    sendMessageFlag = 1;
    debugPrint("Connection and handshake initiated successfully");
    obdConnectedFlag = 1;
}

void OBDManager::sendCommand(String command) {
    if (pClient == nullptr || !pClient->isConnected() || pCharTX == nullptr) {
        debugPrint("Cannot send command, BLE not ready: " + command);
        return;
    }

    // Trim and validate command
    String trimmedCommand = command;
    trimmedCommand.trim();
    
    // Don't send empty commands
    if (trimmedCommand.length() == 0) {
        debugPrint("WARNING: Empty command sending");
        messageReceived = true;  // Reset state
        return;
    }
    
    lastCommandSent = trimmedCommand;

    if (!command.endsWith("\r")) command += "\r";
    debugPrint("Sending: " + trimmedCommand);

    lastResponse = "";
    messageReceived = false;
    lastCommandSentTime = millis();
    currentTimeout = trimmedCommand.startsWith("AT") ? AT_COMMAND_TIMEOUT : DEFAULT_TIMEOUT;

    pCharTX->writeValue((uint8_t*)command.c_str(), command.length(), false);
}

void OBDManager::notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (messageReceived == true) {
        debugPrint("Warning: Unexpected notification while idle, discarding");
        return;
    }

    for (int i = 0; i < length; i++) {
        lastResponse += (char)pData[i];
    }

    // While the echo hasn't been confirmed as disabled, the adapter sends
    // the command itself back before (or together with) the real response.
    // Remove that part of the buffer so it isn't mistaken for ECU data.
    if (!echoDisabled && lastCommandSent.length() > 0) {
        int echoIndex = lastResponse.indexOf(lastCommandSent);
        if (echoIndex != -1) {
            debugPrint("Echo detected, removing: \"" + lastCommandSent + "\"");
            lastResponse.remove(echoIndex, lastCommandSent.length());

            // clear leftover CR/LF that remain right after the echo
            while (lastResponse.length() > 0 && (lastResponse[0] == '\r' || lastResponse[0] == '\n')) {
                lastResponse.remove(0, 1);
            }
        }
    }

    if (lastResponse.indexOf('>') != -1) {
        String cleanResponse = lastResponse;
        cleanResponse.trim();

        if (cleanResponse.length() > 0) {
            debugPrint("Message Received: " + cleanResponse);
        }

        if (!echoDisabled && lastCommandSent == "ATE0" && cleanResponse.indexOf("OK") != -1) {
            echoDisabled = true;
            debugPrint("Echo successfully disabled (ATE0 confirmed)");
        }

        messageReceived = true;
        lastResponse = "";

        sendMessageFlag = 1; // wake up update() on the next loop() cycle
        if (rawMessageCallback) {
            rawMessageCallback(cleanResponse);
        }
    }
}

void OBDManager::setDebugSerial(Stream* serial) {
    debugSerial = serial;
}

void OBDManager::debugPrint(const String& message) {
    if (debugSerial) {
        debugSerial->println(message);
    }
}

void OBDManager::clearCommandQueue() {
    while (!commandQueue.empty()) {
        commandQueue.pop();
    }
}

void OBDManager::addCommandToQueue(const String& command) {
    if (commandQueue.empty() && messageReceived) {
        sendCommand(command);
        return;
    }
    commandQueue.push(command);
}

void OBDManager::update() {
    // Process pending send from command queue
    if (messageReceived && !commandQueue.empty()) {
        sendCommand(commandQueue.front());
        commandQueue.pop();
    }

    // Check for timeout
    if (!messageReceived && lastCommandSentTime > 0) {
        unsigned long elapsed = millis() - lastCommandSentTime;
        if (elapsed >= currentTimeout) {
            debugPrint("ERROR: Timeout waiting for response to: " + lastCommandSent);
            messageReceived = true;  // Reset state to allow next command
            lastResponse = "";
        }
    }

    static bool wasConnected = false;
    const bool connected = (pClient != nullptr) && pClient->isConnected();

    if (connected) {
        wasConnected = true;
    } else if (wasConnected) {
        obdDisconnectedFlag = 1;
        wasConnected = false;
    }

void OBDManager::setRawMessageCallback(RawMessageCallback callback) {
    rawMessageCallback = callback;
}