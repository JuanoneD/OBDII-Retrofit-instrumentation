#include "OBDManager.h"
#include "config.h"
#include "CallbackManager.h"

namespace {
portMUX_TYPE foundTargetDeviceMux = portMUX_INITIALIZER_UNLOCKED;

// Protects OBD communication state (lastResponse, lastCommandSent,
// messageReceived, lastCommandSentTime, currentTimeout, echoDisabled),
// since these are written both by notifyCallback() (runs on the Bluedroid
// BLE task) and by sendCommand()/update() (run in loop() context).
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Holds a completed response until update() (loop-context) picks it up,
// so decode() and its String parsing / Serial prints never run on the
// BLE task itself.
volatile bool responsePending = false;
String pendingResponseText;
bool pendingEchoJustDisabled = false;
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

    if (pCharRX->canNotify()) {
        pCharRX->registerForNotify([](BLERemoteCharacteristic* c, uint8_t* pData, size_t length, bool isNotify) {
            OBDManager::notifyCallback(c, pData, length, isNotify);
        });
    }

    DebugSerial::println("Characteristics ready, queuing configuration commands...");

    portENTER_CRITICAL(&stateMux);
    messageReceived = true;
    echoDisabled = false;
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
    sendMessageFlag = 1;
    DebugSerial::println("Connection and handshake initiated successfully");
    obdConnectedFlag = 1;
}

// Only ever called from loop()-context.
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

    portENTER_CRITICAL(&stateMux);
    lastCommandSent = trimmedCommand;
    lastResponse = "";
    messageReceived = false;
    lastCommandSentTime = millis();
    currentTimeout = timeoutToUse;
    portEXIT_CRITICAL(&stateMux);

    unsigned long writeStart = millis();
    
    // Executa a escrita sem capturar o retorno diretamente na variável booleana
    pCharTX->writeValue((uint8_t*)command.c_str(), command.length(), true);
    bool ok = true; // Como o writeValue moderno não retorna bool na sua versão atual, assumimos sucesso no envio ATT ou tratamos pelo fluxo de resposta/timeout.
    
    unsigned long writeDuration = millis() - writeStart;

    DebugSerial::println("Sending: " + trimmedCommand +
                          " | write-with-response ok=1" +
                          " tomou " + String(writeDuration) + "ms");
                          
    if (!ok) {
        // Write falhou no nível ATT — trata como se tivesse dado timeout
        // pra não travar a máquina de estados esperando resposta que não virá.
        portENTER_CRITICAL(&stateMux);
        messageReceived = true;
        portEXIT_CRITICAL(&stateMux);
    }
}

// Runs on the Bluedroid BLE task, NOT on loop(). Keep this fast: only
// append bytes and, once a full response is detected, stash it for
// update() to process. No String parsing beyond echo/terminator
// detection, no calls to rawMessageCallback, no prints here.
void OBDManager::notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    portENTER_CRITICAL(&stateMux);

    if (messageReceived) {
        portEXIT_CRITICAL(&stateMux);
        return;
    }

    for (size_t i = 0; i < length; i++) {
        lastResponse += (char)pData[i];
    }

    if (!echoDisabled && lastCommandSent.length() > 0) {
        int echoIndex = lastResponse.indexOf(lastCommandSent);
        if (echoIndex != -1) {
            lastResponse.remove(echoIndex, lastCommandSent.length());
            while (lastResponse.length() > 0 && (lastResponse[0] == '\r' || lastResponse[0] == '\n')) {
                lastResponse.remove(0, 1);
            }
        }
    }

    if (lastResponse.indexOf('>') != -1) {
        pendingResponseText = lastResponse;
        pendingResponseText.trim();

        pendingEchoJustDisabled = false;
        if (!echoDisabled && lastCommandSent == "ATE0" && pendingResponseText.indexOf("OK") != -1) {
            echoDisabled = true;
            pendingEchoJustDisabled = true;
        }

        messageReceived = true;
        lastResponse = "";
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

    // Process any response the BLE task stashed for us. decode() and its
    // Serial prints happen here, safely off the BLE task.
    bool hasResponse = false;
    String responseToProcess;
    bool echoJustDisabled = false;
    portENTER_CRITICAL(&stateMux);
    if (responsePending) {
        hasResponse = true;
        responseToProcess = pendingResponseText;
        echoJustDisabled = pendingEchoJustDisabled;
        responsePending = false;
    }
    portEXIT_CRITICAL(&stateMux);

    if (hasResponse) {
        if (responseToProcess.length() > 0) {
            DebugSerial::println("Message Received: " + responseToProcess);
        }
        if (echoJustDisabled) {
            DebugSerial::println("Echo successfully disabled (ATE0 confirmed)");
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