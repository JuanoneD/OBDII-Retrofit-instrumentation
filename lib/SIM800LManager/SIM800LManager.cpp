#include "SIM800LManager.h"
#include <cstring>
#include <queue>
#include <math.h>
#include <esp_mac.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Static Member Definitions
// ---------------------------------------------------------------------------
HardwareSerial* SIM800LManager::m_serial = &Serial2;
int8_t SIM800LManager::m_rstPin = SIM800_RST_PIN;
ESIMSTATUS SIM800LManager::m_status = ESIMSTATUS::DISCONNECTED;
bool SIM800LManager::m_hardwareAvailable = false;
bool SIM800LManager::m_callbackInit = false;
RawMessageCallback SIM800LManager::m_rawMessageCallback = nullptr;

std::queue<String> SIM800LManager::m_commandQueue;
String SIM800LManager::m_lastCommandSent = "";
String SIM800LManager::m_lastResponse = "";
unsigned long SIM800LManager::m_lastCommandSentTime = 0;
unsigned long SIM800LManager::m_currentTimeout = DEFAULT_TIMEOUT;
bool SIM800LManager::m_messageReceived = true;
unsigned long SIM800LManager::m_lastConnectionAttemptTime = 0;

int SIM800LManager::esimConnectedFlag = 0;
int SIM800LManager::esimDisconnectedFlag = 0;
int SIM800LManager::esimWithoutSimFlag = 0;

int& SIM800LManager::simConnectedFlag = SIM800LManager::esimConnectedFlag;
int& SIM800LManager::simDisconnectedFlag = SIM800LManager::esimDisconnectedFlag;
int& SIM800LManager::withoutSimFlag = SIM800LManager::esimWithoutSimFlag;

volatile bool SIM800LManager::m_sendInProgress = false;
String SIM800LManager::m_sendTaskUrl = "";
SemaphoreHandle_t SIM800LManager::m_serialMutex = nullptr;

unsigned long SIM800LManager::m_lastRecoveryAttemptMs = 0;
unsigned long SIM800LManager::m_baudRateForRecovery = SIM800_BAUD_RATE;
int8_t SIM800LManager::m_rxPinForRecovery = SIM800_RX_PIN;
int8_t SIM800LManager::m_txPinForRecovery = SIM800_TX_PIN;

// ---------------------------------------------------------------------------
// Local helpers (no need to be in the header)
// ---------------------------------------------------------------------------

// Time without receiving any byte to consider that the module has finished responding
static const unsigned long kDrainIdleMs = 100;
// Absolute cap for the draining phase, regardless of whether bytes keep
// arriving. Without this, UART noise (or the module sending unsolicited URCs
// like +CREG:/+CPIN: continuously) reset the idle timer on every byte and
// hung the loop() forever inside this blocking call.
static const unsigned long kDrainMaxMs = 2000;

static bool differs(float a, float b, float eps) {
    return fabsf(a - b) > eps;
}

// Reads serial until "expected" (or "ERROR") is found or timeout expires.
// After finding it, keeps reading until serial becomes idle, to avoid cutting
// the message in the middle (e.g. "+CME ERROR: SIM not inserted", "+HTTPACTION: 1,200,45").
static String readSerialResponse(HardwareSerial* serial, unsigned long timeoutMs, const char* expected) {
    String response = "";
    const bool hasExpected = (expected != nullptr && expected[0] != '\0');
    unsigned long start = millis();
    bool done = false;

    while (!done && (millis() - start < timeoutMs)) {
        while (serial->available()) {
            response += (char)serial->read();
        }

        if (hasExpected && response.indexOf(expected) != -1) {
            done = true;
        } else if (response.indexOf("ERROR") != -1) {
            done = true;
        }

        if (!done) delay(10);
    }

    if (done) {
        unsigned long drainStart = millis();
        unsigned long lastByte = drainStart;
        while ((millis() - lastByte < kDrainIdleMs) &&
               (millis() - drainStart < kDrainMaxMs)) {
            while (serial->available()) {
                response += (char)serial->read();
                lastByte = millis();
            }
            delay(5);
        }
    }

    return response;
}

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
SIM800LManager::SIM800LManager() {}
SIM800LManager::~SIM800LManager() {}

// ---------------------------------------------------------------------------
// Device ID (MAC address)
// ---------------------------------------------------------------------------
String SIM800LManager::getDeviceId() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA); // works without enabling WiFi
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
void SIM800LManager::begin(unsigned long baudRate, int8_t rxPin, int8_t txPin, int8_t rstPin) {
    if (m_serialMutex == nullptr) {
        m_serialMutex = xSemaphoreCreateMutex();
    }

    DebugSerial::println("[SIM800L] Initializing hardware on RX=" + String(rxPin) +
                         " TX=" + String(txPin) + " Baud=" + String(baudRate));

    m_baudRateForRecovery = baudRate;
    m_rxPinForRecovery = rxPin;
    m_txPinForRecovery = txPin;

    m_rstPin = rstPin;
    if (m_rstPin >= 0) {
        pinMode(m_rstPin, OUTPUT);
        digitalWrite(m_rstPin, HIGH);
    }

    m_serial->begin(baudRate, SERIAL_8N1, rxPin, txPin);

    // Optional hardware reset pulse if reset pin is configured
    if (m_rstPin >= 0) {
        digitalWrite(m_rstPin, LOW);
        delay(100);
        digitalWrite(m_rstPin, HIGH);
        delay(3000); // Allow SIM800L module to complete boot sequence
    }

    // Set initial eSIM status to DISCONNECTED as required
    setStatus(ESIMSTATUS::DISCONNECTED);

    // Flush any garbage bytes from startup
    while (m_serial->available()) {
        m_serial->read();
    }

    // Test AT communication with retries and autobaud synchronization
    m_hardwareAvailable = false;
    for (int i = 0; i < 10; i++) {
        String resp = sendATCommand("AT", 1000, "OK");
        if (resp.indexOf("OK") != -1) {
            m_hardwareAvailable = true;
            break;
        }
        delay(500);
    }

    if (m_hardwareAvailable) {
        DebugSerial::println("[SIM800L] Module detected and communicating successfully.");
        sendATCommand("ATE0", 1000, "OK");      // Disable command echo
        sendATCommand("AT+CMEE=2", 1000, "OK"); // Enable detailed error codes
        sendATCommand("AT+IPR=" + String(baudRate), 1000, "OK"); // Lock baud rate permanently
        ensureAllBand();                        // Ensure ALL_BAND (quad-band: 850/900/1800/1900 MHz)
        checkSimStatus();
    } else {
        DebugSerial::println("[SIM800L] WARNING: Module not responding to AT commands.");
        setStatus(ESIMSTATUS::DISCONNECTED);
    }

    // Register update callback with CallbackManager (following OBDManager pattern)
    if (!m_callbackInit) {
        CallbackManager::addTimer(SIM800_UPDATE_INTERVAL_MS, SIM800LManager::update);
        m_callbackInit = true;
    }
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------
ESIMSTATUS SIM800LManager::getStatus() {
    return m_status;
}

void SIM800LManager::setStatus(ESIMSTATUS newStatus) {
    if (m_status == newStatus) return;

    m_status = newStatus;
    VehicleData::getInstance().setEsimStatus(newStatus);

    switch (newStatus) {
        case ESIMSTATUS::CONNECTED:
            DebugSerial::println("[SIM800L] Status transition -> CONNECTED");
            esimConnectedFlag = 1;
            break;
        case ESIMSTATUS::DISCONNECTED:
            DebugSerial::println("[SIM800L] Status transition -> DISCONNECTED");
            esimDisconnectedFlag = 1;
            break;
        case ESIMSTATUS::WITHOUT_SIM:
            DebugSerial::println("[SIM800L] Status transition -> WITHOUT_SIM");
            esimWithoutSimFlag = 1;
            break;
    }
}

bool SIM800LManager::isConnected() {
    return m_status == ESIMSTATUS::CONNECTED;
}

bool SIM800LManager::checkSimStatus() {
    if (!m_hardwareAvailable) return false;

    DebugSerial::println("[SIM800L] Querying SIM/eSIM presence (AT+CPIN?)...");

    for (int attempt = 0; attempt < 5; attempt++) {
        String resp = sendATCommand("AT+CPIN?", 3000, "OK");
        String low = resp;
        low.toLowerCase();

        if (low.indexOf("+cpin: ready") != -1) {
            DebugSerial::println("[SIM800L] SIM/eSIM status: READY");
            if (m_status == ESIMSTATUS::WITHOUT_SIM) {
                setStatus(ESIMSTATUS::DISCONNECTED);
            }
            return true;
        }

        if (low.indexOf("not inserted") != -1) {
            DebugSerial::println("[SIM800L] SIM/eSIM status: NOT INSERTED");
            setStatus(ESIMSTATUS::WITHOUT_SIM);
            return false;
        }

        // "SIM busy" or transient error right after power on: wait and retry
        DebugSerial::println("[SIM800L] CPIN not ready yet, retrying...");
        delay(1000);
    }

    DebugSerial::println("[SIM800L] SIM status could not be determined.");
    return false;
}

bool SIM800LManager::ensureAllBand() {
    if (!m_hardwareAvailable) return false;

    DebugSerial::println("[SIM800L] Checking operating band (AT+CBAND?)...");
    String cband = sendATCommand("AT+CBAND?", 2000, "OK");
    if (cband.indexOf("ALL_BAND") != -1) {
        DebugSerial::println("[SIM800L] Operating band is already ALL_BAND (quad-band).");
        return true;
    }

    DebugSerial::println("[SIM800L] Current band is not ALL_BAND (" + cband + "). Reconfiguring to ALL_BAND...");
    String resp = sendATCommand("AT+CBAND=\"ALL_BAND\"", 3000, "OK");
    if (resp.indexOf("OK") == -1) {
        DebugSerial::println("[SIM800L] Failed to set AT+CBAND=\"ALL_BAND\".");
        return false;
    }

    DebugSerial::println("[SIM800L] Operating band set to ALL_BAND successfully.");
    return true;
}

// ---------------------------------------------------------------------------
// GPRS
// ---------------------------------------------------------------------------
bool SIM800LManager::connectGPRS() {
    if (!m_hardwareAvailable) {
        DebugSerial::println("[SIM800L] Cannot connect GPRS: Module hardware not responding.");
        return false;
    }

    if (m_status == ESIMSTATUS::WITHOUT_SIM) {
        DebugSerial::println("[SIM800L] Cannot connect GPRS: WITHOUT_SIM state.");
        return false;
    }

    DebugSerial::println("[SIM800L] Starting GPRS connection sequence...");

    // 1. Verify SIM status
    if (!checkSimStatus()) {
        return false;
    }

    // 2. Check Network Registration (AT+CREG?) with multiple retries
    bool registered = false;
    for (int i = 0; i < 8; i++) {
        String cregResp = sendATCommand("AT+CREG?", 3000, "OK");
        if (cregResp.indexOf(",1") != -1 || cregResp.indexOf(",5") != -1) {
            registered = true;
            break;
        }
        DebugSerial::println("[SIM800L] Network not yet registered, waiting...");
        delay(1500);
    }

    if (!registered) {
        DebugSerial::println("[SIM800L] ERROR: Network registration failed. Running network diagnostics...");
        setStatus(ESIMSTATUS::DISCONNECTED);
        logNetworkDiagnostics();
        return false;
    }

    // 3. Attach GPRS
    sendATCommand("AT+CGATT=1", 5000, "OK");

    // 4. Check if Bearer 1 is already open
    String sapbrQuery = sendATCommand("AT+SAPBR=2,1", 3000, "OK");
    if (sapbrQuery.indexOf("+SAPBR: 1,1") != -1) {
        DebugSerial::println("[SIM800L] GPRS Bearer profile 1 already active.");
        setStatus(ESIMSTATUS::CONNECTED);
        return true;
    }

    // 5. Configure Bearer Profile (SAPBR)
    sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 2000, "OK");

    String apnCmd = "AT+SAPBR=3,1,\"APN\",\"" + String(SIM800_APN) + "\"";
    sendATCommand(apnCmd, 2000, "OK");

    if (strlen(SIM800_APN_USER) > 0) {
        String userCmd = "AT+SAPBR=3,1,\"USER\",\"" + String(SIM800_APN_USER) + "\"";
        sendATCommand(userCmd, 2000, "OK");
    }

    if (strlen(SIM800_APN_PASS) > 0) {
        String passCmd = "AT+SAPBR=3,1,\"PWD\",\"" + String(SIM800_APN_PASS) + "\"";
        sendATCommand(passCmd, 2000, "OK");
    }

    // 6. Open Bearer
    sendATCommand("AT+SAPBR=1,1", 10000, "OK");

    // 7. Verify allocated IP address
    sapbrQuery = sendATCommand("AT+SAPBR=2,1", 3000, "OK");
    if (sapbrQuery.indexOf("+SAPBR: 1,1") != -1) {
        DebugSerial::println("[SIM800L] GPRS Bearer connected successfully!");
        setStatus(ESIMSTATUS::CONNECTED);
        return true;
    } else {
        DebugSerial::println("[SIM800L] ERROR: GPRS Bearer failed to connect. Running network diagnostics...");
        setStatus(ESIMSTATUS::DISCONNECTED);
        logNetworkDiagnostics();
        return false;
    }
}

bool SIM800LManager::disconnectGPRS() {
    DebugSerial::println("[SIM800L] Disconnecting GPRS bearer...");
    sendATCommand("AT+SAPBR=0,1", 3000, "OK");
    setStatus(ESIMSTATUS::DISCONNECTED);
    return true;
}

// ---------------------------------------------------------------------------
// Backend Synchronization
// ---------------------------------------------------------------------------
String SIM800LManager::buildNonVolatileJson() {
    VehicleData& data = VehicleData::getInstance();

    String json = "{";
    json += "\"gasolineLevel\":" + String(data.getGasolineLevel(), 2) + ",";
    json += "\"tankCapacity\":" + String(data.getTankCapacity(), 2) + ",";
    json += "\"fuelConsumptionFactor\":" + String(data.getFuelConsumptionFactor(), 10) + ",";
    json += "\"totalDistance\":" + String(data.getTotalDistance(), 2) + ",";
    json += "\"tripConsumption\":" + String(data.getTripConsumption(), 2) + ",";
    json += "\"sync_version\":" + String(data.getSyncVersion());
    json += "}";
    return json;
}

bool SIM800LManager::isSendInProgress() {
    return m_sendInProgress;
}

// Runs on its own FreeRTOS task. All of this is blocking (AT commands with
// multi-second timeouts, up to 90s just for AT+HTTPACTION), which is exactly
// why it must NOT run inline from the CallbackManager/loop()-driven timer
// callback: doing so used to freeze the entire firmware (BLE, display, OBD
// polling, everything) for the whole duration of the HTTP transaction.
//
// Network diagnostics are triggered inside connectGPRS()/sendHttpPost() only
// for network-level failures. A server-side error (HTTP 4xx/5xx) is not a
// network problem, so it must not trigger them.
void SIM800LManager::sendTaskTrampoline(void* param) {
    {
        String url = m_sendTaskUrl;

        DebugSerial::println("[SIM800L] Triggering non-volatile data POST transmission...");

        if (!url.endsWith("/")) url += "/";
        url += "devices/" + getDeviceId();
        DebugSerial::println("[SIM800L] Sending non-volatile data to: " + url);

        String payload = buildNonVolatileJson();
        String responseBody;
        bool ok = sendHttpPost(url, payload, &responseBody);

        if (ok && responseBody.length() > 0) {
            applyServerResponse(responseBody);
        } else if (!ok) {
            DebugSerial::println("[SIM800L] Non-volatile POST transmission failed.");
        }
    }

    m_sendInProgress = false;
    vTaskDelete(nullptr);
}

bool SIM800LManager::sendNonVolatileData(const String& baseUrl) {
    if (m_sendInProgress) {
        DebugSerial::println("[SIM800L] WARNING: A non-volatile data send is already in progress. Skipping.");
        return false;
    }

    m_sendTaskUrl = baseUrl;
    m_sendInProgress = true;

    BaseType_t result = xTaskCreate(
        sendTaskTrampoline,
        "sim800SendTask",
        8192,      // stack size: builds URL/JSON Strings, AT responses, etc.
        nullptr,
        1,         // low-ish priority, don't starve BLE/other critical tasks
        nullptr
    );

    if (result != pdPASS) {
        DebugSerial::println("[SIM800L] ERROR: Failed to create send task.");
        m_sendInProgress = false;
        return false;
    }

    return true;
}

void SIM800LManager::applyServerResponse(const String& body) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        DebugSerial::println("[SIM800L] ERROR: invalid JSON in server response: " + String(err.c_str()));
        return;
    }

    if (doc["sync_version"].isNull()) {
        DebugSerial::println("[SIM800L] Server response without sync_version. Ignoring.");
        return;
    }

    VehicleData& d = VehicleData::getInstance();

    float gasoline = doc["gasolineLevel"] | d.getGasolineLevel();
    float tank     = doc["tankCapacity"] | d.getTankCapacity();
    float factor   = doc["fuelConsumptionFactor"] | d.getFuelConsumptionFactor();
    double dist    = doc["totalDistance"] | d.getTotalDistance();
    float trip     = doc["tripConsumption"] | d.getTripConsumption();
    int   version  = doc["sync_version"];

    // Guard against a malformed/degenerate value coming back from the backend
    // (NaN, +/-Infinity, zero or negative). The fuel consumption factor is a
    // very small positive calibration constant (~1e-7), so anything outside
    // that sane range must never be persisted or it will zero out / blow up
    // the fuel calculation on the ESP side.
    if (!isfinite(factor) || factor <= 0.0f) {
        DebugSerial::println("[SIM800L] WARNING: Server returned invalid fuelConsumptionFactor (" +
                              String(factor, 10) + "). Keeping local value.");
        factor = d.getFuelConsumptionFactor();
    }

    bool changed =
        differs(gasoline, d.getGasolineLevel(), 0.01f) ||
        differs(tank, d.getTankCapacity(), 0.01f) ||
        differs(factor, d.getFuelConsumptionFactor(), 0.000001f) ||
        differs((float)dist, (float)d.getTotalDistance(), 0.01f) ||
        differs(trip, d.getTripConsumption(), 0.01f) ||
        version != d.getSyncVersion();

    if (!changed) {
        DebugSerial::println("[SIM800L] Server data identical to local. Nothing to do.");
        return;
    }

    DebugSerial::println("[SIM800L] Server data differs. Overwriting local values.");
    d.setGasolineLevel(gasoline);
    d.setTankCapacity(tank);
    d.setFuelConsumptionFactor(factor);
    d.setTotalDistance(dist);
    d.setTripConsumption(trip);
    d.setSyncVersion(version);
}

bool SIM800LManager::sendHttpPost(const String& url, const String& jsonPayload, String* responseBody) {
    if (!m_hardwareAvailable) {
        DebugSerial::println("[SIM800L] ERROR: Cannot send HTTP POST (Hardware unavailable).");
        return false;
    }

    if (m_status == ESIMSTATUS::WITHOUT_SIM) {
        DebugSerial::println("[SIM800L] ERROR: Cannot send HTTP POST (WITHOUT_SIM).");
        return false;
    }

    if (m_status != ESIMSTATUS::CONNECTED) {
        DebugSerial::println("[SIM800L] eSIM disconnected. Establishing GPRS first...");
        if (!connectGPRS()) {
            DebugSerial::println("[SIM800L] ERROR: GPRS connection could not be established.");
            return false;
        }
    }

    DebugSerial::println("[SIM800L] Sending HTTP POST to: " + url);
    DebugSerial::println("[SIM800L] Payload: " + jsonPayload);

    // 1. Terminate any dangling HTTP session
    sendATCommand("AT+HTTPTERM", 1000, "OK");

    // 2. Initialize HTTP service
    String resp = sendATCommand("AT+HTTPINIT", 3000, "OK");
    if (resp.indexOf("OK") == -1) {
        // If already initialized or pending, terminate and retry
        sendATCommand("AT+HTTPTERM", 2000, "OK");
        delay(500);
        resp = sendATCommand("AT+HTTPINIT", 3000, "OK");
        if (resp.indexOf("OK") == -1) {
            DebugSerial::println("[SIM800L] ERROR: AT+HTTPINIT failed.");
            return false;
        }
    }

    // 3. Set Bearer profile identifier (CID = 1)
    resp = sendATCommand("AT+HTTPPARA=\"CID\",1", 2000, "OK");
    if (resp.indexOf("OK") == -1) {
        DebugSerial::println("[SIM800L] ERROR: AT+HTTPPARA CID failed.");
        sendATCommand("AT+HTTPTERM", 1000, "OK");
        return false;
    }

    // 4. Set Destination URL
    String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
    resp = sendATCommand(urlCmd, 3000, "OK");
    if (resp.indexOf("OK") == -1) {
        DebugSerial::println("[SIM800L] ERROR: AT+HTTPPARA URL failed.");
        sendATCommand("AT+HTTPTERM", 1000, "OK");
        return false;
    }

    // 5. Set Content-Type Header
    resp = sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 2000, "OK");
    if (resp.indexOf("OK") == -1) {
        DebugSerial::println("[SIM800L] ERROR: AT+HTTPPARA CONTENT failed.");
        sendATCommand("AT+HTTPTERM", 1000, "OK");
        return false;
    }

    // 6. Handle SSL if URL is HTTPS
    if (url.startsWith("https://")) {
        sendATCommand("AT+HTTPSSL=1", 2000, "OK");
    } else {
        sendATCommand("AT+HTTPSSL=0", 2000, "OK");
    }

    // 7. Request HTTPDATA transmission
    String dataCmd = "AT+HTTPDATA=" + String(jsonPayload.length()) + ",10000";
    resp = sendATCommand(dataCmd, 5000, "DOWNLOAD");
    if (resp.indexOf("DOWNLOAD") == -1) {
        DebugSerial::println("[SIM800L] ERROR: DOWNLOAD prompt not received.");
        sendATCommand("AT+HTTPTERM", 1000, "OK");
        return false;
    }

    // 8. Stream JSON payload bytes and wait for "OK" upload confirmation
    resp = sendRawData(jsonPayload, 5000, "OK");
    if (resp.indexOf("OK") == -1) {
        DebugSerial::println("[SIM800L] ERROR: Failed streaming JSON payload.");
        sendATCommand("AT+HTTPTERM", 1000, "OK");
        return false;
    }

    // 9. Execute HTTP POST action (Method 1).
    // The module answers "OK" immediately and the real result arrives later as
    // "+HTTPACTION: 1,<code>,<len>". On weak 2G / roaming it can take 60s or
    // more, so wait generously (this runs in the background task, so blocking
    // here does not freeze loop()).
    resp = sendATCommand("AT+HTTPACTION=1", 90000, "+HTTPACTION:");
    int httpCode = 0;
    int actionIdx = resp.indexOf("+HTTPACTION:");
    if (actionIdx != -1) {
        int firstComma = resp.indexOf(',', actionIdx);
        int secondComma = resp.indexOf(',', firstComma + 1);
        if (firstComma != -1 && secondComma != -1) {
            String codeStr = resp.substring(firstComma + 1, secondComma);
            codeStr.trim();
            httpCode = codeStr.toInt();
        }
    } else {
        DebugSerial::println("[SIM800L] WARNING: No +HTTPACTION result received (request still pending).");
    }

    DebugSerial::println("[SIM800L] HTTP POST Result Code: " + String(httpCode));

    bool success = (httpCode >= 200 && httpCode < 300);

    // 10. Read response body BEFORE HTTPTERM
    if (success && responseBody != nullptr) {
        String r = sendATCommand("AT+HTTPREAD", 5000, "+HTTPREAD:");
        int s = r.indexOf('{');
        int e = r.lastIndexOf('}');
        if (s != -1 && e > s) {
            *responseBody = r.substring(s, e + 1);
        } else {
            DebugSerial::println("[SIM800L] WARNING: No JSON body found in HTTP response.");
        }
    }

    // 11. Terminate HTTP session cleanly.
    // "+CME ERROR: operation not allowed" means the action is still pending:
    // wait a bit and try once more.
    String termResp = sendATCommand("AT+HTTPTERM", 2000, "OK");
    if (termResp.indexOf("OK") == -1) {
        delay(3000);
        sendATCommand("AT+HTTPTERM", 2000, "OK");
    }

    if (success) {
        DebugSerial::println("[SIM800L] HTTP POST successfully completed!");
    } else {
        DebugSerial::println("[SIM800L] ERROR: HTTP POST failed with HTTP status: " + String(httpCode));
    }

    // 0 (no result) and 6xx codes are SIM800L network errors. Drop the bearer so
    // the next attempt reopens it from scratch (otherwise connectGPRS() would
    // find a stale "already active" bearer), and run the network diagnostics.
    // 4xx/5xx are server-side errors and do not need network diagnostics.
    if (httpCode == 0 || httpCode >= 600) {
        sendATCommand("AT+SAPBR=0,1", 10000, "OK");
        setStatus(ESIMSTATUS::DISCONNECTED);
        logNetworkDiagnostics();
    }

    return success;
}

// ---------------------------------------------------------------------------
// AT Communication (blocking)
// ---------------------------------------------------------------------------
String SIM800LManager::sendATCommand(const String& command, unsigned long timeoutMs, const char* expectedResponse) {
    if (m_serial == nullptr) return "";

    // sendATCommand() can now be called both from loop()-context (begin(),
    // checkSimStatus() during init, etc.) and from the background send task
    // (sendNonVolatileData() -> sendHttpPost() -> connectGPRS()/sendATCommand()).
    // Guard access to m_serial so the two never read/write it concurrently.
    if (m_serialMutex != nullptr) {
        xSemaphoreTake(m_serialMutex, portMAX_DELAY);
    }

    while (m_serial->available()) {
        m_serial->read();
    }

    DebugSerial::println("[SIM800L] TX: " + command);
    m_serial->print(command);
    m_serial->print("\r\n");

    String response = readSerialResponse(m_serial, timeoutMs, expectedResponse);

    if (m_serialMutex != nullptr) {
        xSemaphoreGive(m_serialMutex);
    }

    response.trim();
    if (response.length() > 0) {
        DebugSerial::println("[SIM800L] RX: " + response);
    }

    if (m_rawMessageCallback && response.length() > 0) {
        m_rawMessageCallback(response);
    }

    return response;
}

String SIM800LManager::sendRawData(const String& data, unsigned long timeoutMs, const char* expectedResponse) {
    if (m_serial == nullptr) return "";

    if (m_serialMutex != nullptr) {
        xSemaphoreTake(m_serialMutex, portMAX_DELAY);
    }

    m_serial->print(data);

    String response = readSerialResponse(m_serial, timeoutMs, expectedResponse);

    if (m_serialMutex != nullptr) {
        xSemaphoreGive(m_serialMutex);
    }

    response.trim();
    if (response.length() > 0) {
        DebugSerial::println("[SIM800L] RX: " + response);
    }
    return response;
}

// ---------------------------------------------------------------------------
// Command Queue (asynchronous, via CallbackManager)
// ---------------------------------------------------------------------------
void SIM800LManager::clearCommandQueue() {
    while (!m_commandQueue.empty()) {
        m_commandQueue.pop();
    }
}

void SIM800LManager::addCommandToQueue(const String& command) {
    if (m_commandQueue.empty() && m_messageReceived) {
        sendCommand(command);
        return;
    }
    m_commandQueue.push(command);
}

void SIM800LManager::sendCommand(const String& command) {
    if (m_serial == nullptr) return;

    String trimmed = command;
    trimmed.trim();
    if (trimmed.length() == 0) return;

    m_lastCommandSent = trimmed;
    m_lastResponse = "";
    m_messageReceived = false;
    m_lastCommandSentTime = millis();
    m_currentTimeout = DEFAULT_TIMEOUT;

    m_serial->print(trimmed);
    m_serial->print("\r\n");

    DebugSerial::println("[SIM800L] Queued Command Dispatched: " + trimmed);
}

void SIM800LManager::update() {
    // Hardware not responding or SIM reported missing are otherwise permanent
    // barriers (both are only ever cleared from begin()/checkSimStatus()).
    // Give them a throttled chance to self-heal in the background: a modem
    // that comes back up, or a SIM that gets inserted/activated later, can
    // then resume telemetry without requiring a device restart.
    if (!m_hardwareAvailable || m_status == ESIMSTATUS::WITHOUT_SIM) {
        tryRecover();
    }

    readIncoming();

    // Process pending command from queue if available
    if (m_messageReceived && !m_commandQueue.empty()) {
        String nextCmd = m_commandQueue.front();
        m_commandQueue.pop();
        sendCommand(nextCmd);
    }

    // Timeout management for queued commands
    if (!m_messageReceived && m_lastCommandSentTime > 0) {
        if (millis() - m_lastCommandSentTime >= m_currentTimeout) {
            DebugSerial::println("[SIM800L] Timeout waiting for command: " + m_lastCommandSent);
            m_messageReceived = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Background recovery for hardware-unavailable / WITHOUT_SIM states
// ---------------------------------------------------------------------------
void SIM800LManager::tryRecover() {
    // Throttle: these probes involve several AT round-trips (up to ~1s each
    // with retries), so they must not run on every update() tick
    // (SIM800_UPDATE_INTERVAL_MS is only 100ms) or they'd starve the command
    // queue/serial line and spam the module.
    const unsigned long kRecoveryCooldownMs = 30UL * 1000UL;

    unsigned long now = millis();
    if (m_lastRecoveryAttemptMs != 0 && (now - m_lastRecoveryAttemptMs < kRecoveryCooldownMs)) {
        return;
    }
    m_lastRecoveryAttemptMs = now;

    // Don't fight the background send task for the serial line.
    if (m_serialMutex != nullptr) {
        if (xSemaphoreTake(m_serialMutex, 0) != pdTRUE) {
            // Remains scheduled: next call (after cooldown) will retry.
            m_lastRecoveryAttemptMs = 0;
            return;
        }
    }

    if (!m_hardwareAvailable) {
        DebugSerial::println("[SIM800L][RECOVERY] Hardware unavailable, reprobing module responsiveness...");

        // Flush any garbage accumulated while "unavailable".
        while (m_serial->available()) {
            m_serial->read();
        }

        bool responded = false;
        for (int i = 0; i < 3; i++) {
            String resp = sendATCommand("AT", 1000, "OK");
            if (resp.indexOf("OK") != -1) {
                responded = true;
                break;
            }
            delay(300);
        }

        if (responded) {
            DebugSerial::println("[SIM800L][RECOVERY] Module responded again. Repeating initialization sequence...");
            m_hardwareAvailable = true;
            sendATCommand("ATE0", 1000, "OK");
            sendATCommand("AT+CMEE=2", 1000, "OK");
            sendATCommand("AT+IPR=" + String(m_baudRateForRecovery), 1000, "OK");
            ensureAllBand();
            setStatus(ESIMSTATUS::DISCONNECTED);
            checkSimStatus();
        } else {
            DebugSerial::println("[SIM800L][RECOVERY] Module still not responding. Will retry later.");
        }
    } else if (m_status == ESIMSTATUS::WITHOUT_SIM) {
        DebugSerial::println("[SIM800L][RECOVERY] Re-checking SIM presence (AT+CPIN?)...");
        // checkSimStatus() itself clears WITHOUT_SIM -> DISCONNECTED when the
        // SIM reports ready; nothing further to do here either way.
        checkSimStatus();
    }

    if (m_serialMutex != nullptr) {
        xSemaphoreGive(m_serialMutex);
    }
}

void SIM800LManager::readIncoming() {
    if (m_serial == nullptr) return;

    // Non-blocking attempt: if the background send task
    // (sendNonVolatileData()->sendHttpPost()) currently holds the serial
    // mutex, just skip this cycle rather than blocking loop() waiting for
    // it - there will be another chance in SIM800_UPDATE_INTERVAL_MS.
    if (m_serialMutex != nullptr) {
        if (xSemaphoreTake(m_serialMutex, 0) != pdTRUE) {
            return;
        }
    }

    // Cap on bytes processed per call and on an unterminated line length,
    // to avoid this ever turning into an unbounded loop/alloc if the module
    // streams continuous garbage with no '\n' (electrical noise, bad baud,
    // etc.). update() is called frequently (every SIM800_UPDATE_INTERVAL_MS),
    // so leftover bytes are simply picked up on the next call.
    static const size_t kMaxBytesPerCall = 1024;
    static const size_t kMaxLineLength = 512;

    size_t processed = 0;
    while (m_serial->available() && processed < kMaxBytesPerCall) {
        char c = (char)m_serial->read();
        processed++;
        m_lastResponse += c;

        if (c == '\n') {
            String line = m_lastResponse;
            line.trim();
            m_lastResponse = "";

            if (line.length() > 0) {
                if (line.indexOf("+CPIN: NOT INSERTED") != -1) {
                    setStatus(ESIMSTATUS::WITHOUT_SIM);
                } else if (line.indexOf("+PDP: DEACT") != -1 || line.indexOf("CLOSED") != -1) {
                    setStatus(ESIMSTATUS::DISCONNECTED);
                }

                if (line.indexOf("OK") != -1 || line.indexOf("ERROR") != -1) {
                    m_messageReceived = true;
                }

                if (m_rawMessageCallback) {
                    m_rawMessageCallback(line);
                }
            }
        } else if (m_lastResponse.length() > kMaxLineLength) {
            // Garbage with no terminator: drop it so memory doesn't grow forever.
            m_lastResponse = "";
        }
    }

    if (m_serialMutex != nullptr) {
        xSemaphoreGive(m_serialMutex);
    }
}

void SIM800LManager::setRawMessageCallback(RawMessageCallback callback) {
    m_rawMessageCallback = callback;
}

// ---------------------------------------------------------------------------
// Network diagnostics
// ---------------------------------------------------------------------------
void SIM800LManager::logNetworkDiagnostics(bool force) {
    static unsigned long s_lastDiagTime = 0;
    static bool s_hasRun = false;
    // The diagnostics can take minutes (AT+COPS=? alone allows up to 2 min), so
    // the cooldown is counted from the END of the previous run, otherwise a
    // second failure right after would re-trigger it immediately.
    const unsigned long kDiagCooldownMs = 5UL * 60UL * 1000UL;

    if (!force && s_hasRun && (millis() - s_lastDiagTime < kDiagCooldownMs)) {
        DebugSerial::println("[SIM800L][DIAG] Diagnostics throttled (ran less than 5 min ago). Skipping.");
        return;
    }

    DebugSerial::println("[SIM800L][DIAG] ---- Network diagnostics ----");

    // Pre-check: Basic AT communication responsiveness
    String atCheck = sendATCommand("AT", 1000, "OK");
    if (atCheck.indexOf("OK") == -1) {
        atCheck = sendATCommand("AT", 1000, "OK");
    }
    if (atCheck.indexOf("OK") == -1) {
        DebugSerial::println("[SIM800L][DIAG] ERRO CRITICO: Modulo nao responde aos comandos AT na serial!");
        DebugSerial::println("[SIM800L][DIAG] Causa provavel: Modulo sem alimentacao/desligado (corte de sobretensao), fios TX/RX desconectados ou falta de GND comum.");
        DebugSerial::println("[SIM800L][DIAG] Verifique: LED onboard do SIM800L (deve estar aceso/piscando), RX (pino 16), TX (pino 17) e alimentacao.");
        DebugSerial::println("[SIM800L][DIAG] ---- End diagnostics ----");
        s_lastDiagTime = millis();
        s_hasRun = true;
        return;
    }

    // 1. Signal quality
    String csq = sendATCommand("AT+CSQ", 2000, "OK");
    int idx = csq.indexOf("+CSQ:");
    if (idx != -1) {
        int rssi = csq.substring(idx + 5).toInt();
        if (rssi == 99) {
            DebugSerial::println("[SIM800L][DIAG] Signal: 99 (sem sinal/desconhecido) -> verifique antena e cobertura 2G");
        } else {
            String quality = (rssi < 10) ? "fraco" : (rssi < 20) ? "ok" : "bom";
            DebugSerial::println("[SIM800L][DIAG] Signal: " + String(rssi) + " (~" +
                                 String(-113 + 2 * rssi) + " dBm, " + quality + ")");
        }
    } else {
        DebugSerial::println("[SIM800L][DIAG] Signal: sem resposta do AT+CSQ");
    }

    // 2. Operating frequency band (Vivo 2G uses 850MHz, Claro uses 900/1800MHz)
    String cband = sendATCommand("AT+CBAND?", 2000, "OK");
    DebugSerial::println("[SIM800L][DIAG] Active band: " + cband);
    if (cband.indexOf("ALL_BAND") == -1) {
        DebugSerial::println("[SIM800L][DIAG] AVISO: Banda nao e ALL_BAND! Vivo 2G no Brasil opera em 850MHz.");
    }

    // 3. Network registration status (Circuit Switched - Voice/SMS/Cell Tower)
    int cregStat = -1;
    String creg = sendATCommand("AT+CREG?", 2000, "OK");
    idx = creg.indexOf("+CREG:");
    if (idx != -1) {
        int comma = creg.indexOf(',', idx);
        if (comma != -1) {
            int stat = creg.substring(comma + 1).toInt();
            cregStat = stat;
            String txt;
            switch (stat) {
                case 0: txt = "nao registrado e nao esta procurando (problema no chip/operadora/banda)"; break;
                case 1: txt = "registrado (rede propria)"; break;
                case 2: txt = "procurando rede (sinal fraco ou sem cobertura 2G)"; break;
                case 3: txt = "registro NEGADO pela operadora"; break;
                case 4: txt = "desconhecido"; break;
                case 5: txt = "registrado (roaming)"; break;
                default: txt = "valor inesperado"; break;
            }
            DebugSerial::println("[SIM800L][DIAG] Registration (CREG): " + String(stat) + " -> " + txt);
        }
    } else {
        DebugSerial::println("[SIM800L][DIAG] Registration (CREG): sem resposta do AT+CREG?");
    }

    // 4. GPRS registration status (Packet Switched - Data/Internet)
    String cgreg = sendATCommand("AT+CGREG?", 2000, "OK");
    idx = cgreg.indexOf("+CGREG:");
    if (idx != -1) {
        int comma = cgreg.indexOf(',', idx);
        if (comma != -1) {
            int stat = cgreg.substring(comma + 1).toInt();
            String txt;
            switch (stat) {
                case 0: txt = "GPRS nao registrado e nao procurando"; break;
                case 1: txt = "GPRS registrado (rede propria)"; break;
                case 2: txt = "GPRS procurando rede..."; break;
                case 3: txt = "GPRS registro negado"; break;
                case 4: txt = "GPRS desconhecido"; break;
                case 5: txt = "GPRS registrado (roaming)"; break;
                default: txt = "valor inesperado"; break;
            }
            DebugSerial::println("[SIM800L][DIAG] GPRS Registration (CGREG): " + String(stat) + " -> " + txt);
        }
    } else {
        DebugSerial::println("[SIM800L][DIAG] GPRS Registration (CGREG): sem resposta do AT+CGREG?");
    }

    // 5. Current operator
    String cops = sendATCommand("AT+COPS?", 5000, "OK");
    int q1 = cops.indexOf('"');
    int q2 = (q1 != -1) ? cops.indexOf('"', q1 + 1) : -1;
    if (q1 != -1 && q2 > q1) {
        DebugSerial::println("[SIM800L][DIAG] Operator: " + cops.substring(q1 + 1, q2));
    } else {
        DebugSerial::println("[SIM800L][DIAG] Operator: nenhuma (nao conectado a nenhuma rede)");
    }

    // 6. Module supply voltage
    String cbc = sendATCommand("AT+CBC", 2000, "OK");
    idx = cbc.indexOf("+CBC:");
    if (idx != -1) {
        int lastComma = cbc.lastIndexOf(',');
        if (lastComma != -1) {
            int mv = cbc.substring(lastComma + 1).toInt();
            String note = " -> ok";
            if (mv < 3400) {
                note = " -> BAIXA (<3400 mV, alimentacao insuficiente!)";
            } else if (mv > 4400) {
                note = " -> ALERTA SOBRETENSAO (>4400 mV, risco de queimar o modulo!)";
            }
            DebugSerial::println("[SIM800L][DIAG] Supply voltage: " + String(mv) + " mV" + note);
        }
    } else {
        DebugSerial::println("[SIM800L][DIAG] Supply voltage: sem resposta do AT+CBC");
    }

    String cfun = sendATCommand("AT+CFUN?", 2000, "OK");   // should respond +CFUN: 1
    DebugSerial::println("[SIM800L][DIAG] Functionality: " + cfun);

    String ccid = sendATCommand("AT+CCID", 2000, "OK");    // SIM ICCID (confirms that the SIM is read)
    DebugSerial::println("[SIM800L][DIAG] SIM CCID: " + ccid);

    // 7. Operator scan. Only useful when NOT registered: while the module is
    // registered, AT+COPS=? briefly interrupts the current registration/data
    // session, which would make the failure we are diagnosing even worse.
    if (cregStat == 1 || cregStat == 5) {
        DebugSerial::println("[SIM800L][DIAG] Module is registered: skipping operator scan (AT+COPS=?) to avoid dropping the connection.");
    } else {
        cops = sendATCommand("AT+COPS=?", 120000, "OK"); // scans available operators (takes up to ~2 min)
        DebugSerial::println("[SIM800L][DIAG] Available operators: " + cops);
    }

    DebugSerial::println("[SIM800L][DIAG] ---- End diagnostics ----");

    s_lastDiagTime = millis();
    s_hasRun = true;
}