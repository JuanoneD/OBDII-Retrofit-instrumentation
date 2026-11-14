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

TinyGsm* SIM800LManager::m_modem = nullptr;
TinyGsmClient* SIM800LManager::m_gsmClient = nullptr;
SSLClient* SIM800LManager::m_sslClient = nullptr;

String SIM800LManager::m_lastResponse = "";

int SIM800LManager::esimConnectedFlag = 0;
int SIM800LManager::esimDisconnectedFlag = 0;
int SIM800LManager::esimWithoutSimFlag = 0;
int SIM800LManager::diagnoseModeReceivedFlag = 0;

int& SIM800LManager::simConnectedFlag = SIM800LManager::esimConnectedFlag;
int& SIM800LManager::simDisconnectedFlag = SIM800LManager::esimDisconnectedFlag;
int& SIM800LManager::withoutSimFlag = SIM800LManager::esimWithoutSimFlag;

volatile bool SIM800LManager::m_sendInProgress = false;
SemaphoreHandle_t SIM800LManager::m_serialMutex = nullptr;

portMUX_TYPE SIM800LManager::m_pendingResponseMux = portMUX_INITIALIZER_UNLOCKED;
std::vector<String> SIM800LManager::m_pendingSystemErrors;
SemaphoreHandle_t SIM800LManager::m_pendingErrorsMux = nullptr;
char SIM800LManager::m_pendingResponseBuffer[512] = {0};
volatile bool SIM800LManager::m_pendingResponseReady = false;

volatile bool SIM800LManager::m_recoveryInProgress = false;
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
// Cap for a single AT response. Noise or a very chatty response (e.g.
// AT+COPS=? with a 120s window) must never grow the String unbounded on the
// 16KB-stack send task.
static const size_t kMaxAtResponseLength = 8192;
// Cap for the pending system-error queue (bounded flush re-queueing, see
// flushPendingSystemErrors()).
static const size_t kMaxPendingSystemErrors = 20;

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
            if (response.length() > kMaxAtResponseLength) {
                done = true; // runaway response: stop accumulating
                break;
            }
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
            while (serial->available() && response.length() <= kMaxAtResponseLength) {
                response += (char)serial->read();
                lastByte = millis();
            }
            delay(5);
        }
    }

    return response;
}

// Reads one '\n'-terminated line from an Arduino Client using only
// NON-blocking reads, bounded by a millis() timeout. Returns true when a
// full line was read (trailing '\r' stripped), false on timeout/disconnect.
//
// The timeout uses the ROLLOVER-SAFE form (millis() - start >= timeout)
// instead of an absolute deadline: `deadline = millis() + timeout`
// comparisons break at the ~49.7-day millis() wraparound, causing spurious
// instant timeouts on every read after the wrap.
//
// Needed because Stream::readStringUntil() has two failure modes that bite
// us here: (1) it returns EARLY with a partial line when the buffer does not
// yet contain '\n' - which corrupts HTTP status-line parsing when only a
// few bytes of the response have arrived; and (2) it has NO timeout, so a
// stalled peer with the TCP connection still open would hang the send task
// forever (and with it m_sendInProgress, i.e. all future telemetry).
static bool readLineWithTimeout(Client& client, String& lineOut, unsigned long startMs, unsigned long timeoutMs) {
    for (;;) {
        int c = client.available() ? client.read() : -1;
        if (c != -1) {
            if (c == '\n') {
                if (lineOut.length() > 0 && lineOut[lineOut.length() - 1] == '\r') {
                    lineOut.remove(lineOut.length() - 1);
                }
                return true;
            }
            lineOut += (char)c;
            if (lineOut.length() > 1024) return false; // runaway/invalid line
        } else {
            if ((millis() - startMs >= timeoutMs) ||
                (!client.connected() && !client.available())) {
                return false;
            }
            delay(5);
        }
    }
}

// ---------------------------------------------------------------------------
// TinyGSM / SSLClient lazy initialization
// ---------------------------------------------------------------------------
// Created lazily (after m_serial->begin() in begin()) because TinyGsm's
// constructor just stores the Stream& reference, but we want the serial
// port already configured at the right baud rate first.
bool SIM800LManager::ensureModemClientsInitialized() {
    if (m_modem != nullptr && m_gsmClient != nullptr && m_sslClient != nullptr) {
        return true;
    }
    if (m_serial == nullptr) return false;

    if (m_modem == nullptr) {
        m_modem = new TinyGsm(*m_serial);
    }
    if (m_gsmClient == nullptr) {
        m_gsmClient = new TinyGsmClient(*m_modem);
    }
    if (m_sslClient == nullptr) {
        // SSLClient wraps the raw TinyGsmClient (TCP socket over the SIM800L)
        // and performs the TLS 1.2/1.3 handshake itself using the ESP32's CPU,
        // instead of relying on the SIM800L's own obsolete SSLv3/TLS1.0-only
        // engine (which is the root cause of the HTTPACTION 606 errors with
        // modern TLS-only backends like Tailscale Funnel).
        m_sslClient = new SSLClient(m_gsmClient);

        // We don't ship/pin a CA bundle for the backend's cert chain (e.g.
        // Let's Encrypt via Tailscale Funnel), so skip certificate validation
        // entirely rather than failing/hanging trying to validate against an
        // empty trust store. This only disables chain-of-trust verification;
        // the traffic is still fully encrypted via a real TLS 1.2/1.3
        // handshake, which is the actual problem we're solving here (the
        // SIM800L's own SSLv3/TLS1.0 engine couldn't complete the handshake
        // at all against the backend).
        m_sslClient->setInsecure();

        // A full TLS 1.2/1.3 handshake over 2G/GPRS (weak signal, roaming,
        // multiple round-trips for ClientHello/ServerHello/certificates/Finished)
        // routinely takes well past the library's default handshake timeout.
        // Give it generous headroom; this already runs inside the background
        // send task, so a long block here does not freeze loop().
        m_sslClient->setHandshakeTimeout(60000);
    }


    return true;
}

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
        // Recursive: sendATCommand() already takes this mutex
        // internally, and it is now also called from within long
        // operations (connectGPRS(), the HTTP/TLS POST) that need to hold
        // the lock for their entire duration (see comments in connectGPRS()/
        // sendHttpPost()). A plain mutex would deadlock the owning task on
        // the inner sendATCommand() call in that scenario.
        m_serialMutex = xSemaphoreCreateRecursiveMutex();
    }

    if (m_pendingErrorsMux == nullptr) {
        // Guards the pending system-error queue (created here, before any
        // send task starts, so no lazy-init race).
        m_pendingErrorsMux = xSemaphoreCreateMutex();
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

    // Optional hardware reset pulse if reset pin is configured
    if (m_rstPin >= 0) {
        digitalWrite(m_rstPin, LOW);
        delay(100);
        digitalWrite(m_rstPin, HIGH);
        delay(3000); // Allow SIM800L module to complete boot sequence
    }

    // Set initial eSIM status to DISCONNECTED as required
    setStatus(ESIMSTATUS::DISCONNECTED);

    // Autobaud: probe the configured rate first, then 115200 (the other
    // common SIM800L factory/previous AT+IPR setting). Without this, a
    // module previously locked to a different baud rate would be invisible
    // to the firmware - and to the background recovery, which uses the
    // same rate.
    const unsigned long baudCandidates[2] = { baudRate, 115200UL };
    size_t numCandidates = (baudCandidates[0] == baudCandidates[1]) ? 1 : 2;
    unsigned long activeBaud = baudRate;
    m_hardwareAvailable = false;
    for (size_t c = 0; c < numCandidates && !m_hardwareAvailable; c++) {
        activeBaud = baudCandidates[c];
        m_serial->end();
        delay(100);
        m_serial->begin(activeBaud, SERIAL_8N1, rxPin, txPin);
        delay(100);
        // Flush any garbage bytes from startup
        while (m_serial->available()) {
            m_serial->read();
        }
        for (int i = 0; i < 10 && !m_hardwareAvailable; i++) {
            String resp = sendATCommand("AT", 1000, "OK");
            if (resp.indexOf("OK") != -1) {
                m_hardwareAvailable = true;
                break;
            }
            delay(500);
        }
    }

    if (m_hardwareAvailable) {
        // Remember the rate that actually worked (recovery uses it too).
        m_baudRateForRecovery = activeBaud;
        DebugSerial::println("[SIM800L] Module detected at " + String(activeBaud) + " baud.");
        sendATCommand("ATE0", 1000, "OK");      // Disable command echo
        sendATCommand("AT+CMEE=2", 1000, "OK"); // Enable detailed error codes
        sendATCommand("AT+IPR=" + String(activeBaud), 1000, "OK"); // Lock baud rate permanently
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
// GPRS bring-up is now delegated to TinyGsm::gprsConnect(), which internally
// issues the equivalent AT+CGATT/AT+SAPBR-style sequence for SIM800 modems.
// We keep our own AT+CPIN?/AT+CREG? pre-checks (and logNetworkDiagnostics())
// since TinyGsm doesn't expose the same level of diagnostic detail we rely on.
//
// IMPORTANT: unlike sendATCommand() (which grabs the serial
// mutex only for its own single AT round-trip), everything TinyGsm does
// here (isGprsConnected(), gprsConnect()) talks to m_serial directly, NOT
// through sendATCommand(). If readIncoming() (called every
// SIM800_UPDATE_INTERVAL_MS from the loop()-context timer) or tryRecover()
// sneak in and steal/interleave bytes while TinyGsm is mid-exchange with the
// module, the response gets corrupted - this silently broke the OK/ERROR
// framing TinyGsm expects and could cascade into garbage bytes reaching the
// later TLS handshake, which is what caused the "invalid SSL record" /
// handshake timeout errors. So the WHOLE function holds the (recursive)
// serial mutex for its entire duration, not just per-command.
bool SIM800LManager::connectGPRS() {
    if (m_serialMutex != nullptr) {
        xSemaphoreTakeRecursive(m_serialMutex, portMAX_DELAY);
    }

    bool result = connectGPRSLocked();

    if (m_serialMutex != nullptr) {
        xSemaphoreGiveRecursive(m_serialMutex);
    }

    return result;
}

bool SIM800LManager::connectGPRSLocked() {
    if (!m_hardwareAvailable) {
        DebugSerial::println("[SIM800L] Cannot connect GPRS: Module hardware not responding.");
        return false;

    }

    if (m_status == ESIMSTATUS::WITHOUT_SIM) {
        DebugSerial::println("[SIM800L] Cannot connect GPRS: WITHOUT_SIM state.");
        return false;
    }

    if (!ensureModemClientsInitialized()) {
        DebugSerial::println("[SIM800L] ERROR: Could not initialize TinyGsm modem/client.");
        return false;
    }

    DebugSerial::println("[SIM800L] Starting GPRS connection sequence...");

    // 1. Verify SIM status
    if (!checkSimStatus()) {
        return false;
    }

    // 2. Check Network Registration (AT+CREG?) with multiple retries.
    // Parse the status digit properly: locate "+CREG:" first, then the digit
    // after the following comma. A blind indexOf(",1") used to match ",10"/
    // ",15" or stray digits anywhere in the response (even inside a CME
    // ERROR line), faking "registered" and skipping real diagnostics.
    bool registered = false;
    for (int i = 0; i < 8; i++) {
        String cregResp = sendATCommand("AT+CREG?", 3000, "OK");
        int cregIdx = cregResp.indexOf("+CREG:");
        if (cregIdx != -1) {
            int comma = cregResp.indexOf(',', cregIdx);
            if (comma != -1) {
                int stat = cregResp.substring(comma + 1).toInt();
                if (stat == 1 || stat == 5) { // 1 = home, 5 = roaming
                    registered = true;
                    break;
                }
            }
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

    // Already attached? Nothing more to do.
    if (m_modem->isGprsConnected()) {
        DebugSerial::println("[SIM800L] GPRS already connected.");
        setStatus(ESIMSTATUS::CONNECTED);
        return true;
    }

    // 3. Attach GPRS via TinyGsm (APN/user/pass taken from config.h, same as before)
    const char* apnUser = (strlen(SIM800_APN_USER) > 0) ? SIM800_APN_USER : nullptr;
    const char* apnPass = (strlen(SIM800_APN_PASS) > 0) ? SIM800_APN_PASS : nullptr;

    if (!m_modem->gprsConnect(SIM800_APN, apnUser, apnPass)) {
        DebugSerial::println("[SIM800L] ERROR: GPRS Bearer failed to connect. Running network diagnostics...");
        setStatus(ESIMSTATUS::DISCONNECTED);
        logNetworkDiagnostics();
        return false;
    }

    DebugSerial::println("[SIM800L] GPRS Bearer connected successfully! IP: " + m_modem->getLocalIP());
    setStatus(ESIMSTATUS::CONNECTED);
    return true;
}

bool SIM800LManager::disconnectGPRS() {
    // Talk to the modem only while holding the serial mutex, like every
    // other modem-touching path (currently only called with the mutex
    // already held from sendHttpPostLocked(), but keep this self-contained
    // so a future loop-context caller can't corrupt the serial stream).
    if (m_serialMutex != nullptr) {
        xSemaphoreTakeRecursive(m_serialMutex, portMAX_DELAY);
    }
    DebugSerial::println("[SIM800L] Disconnecting GPRS bearer...");
    if (m_modem != nullptr) {
        m_modem->gprsDisconnect();
    } else {
        sendATCommand("AT+SAPBR=0,1", 3000, "OK");
    }
    if (m_serialMutex != nullptr) {
        xSemaphoreGiveRecursive(m_serialMutex);
    }
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
    // The URL is heap-owned by this task (passed as the task parameter), so
    // a new sendNonVolatileData() call can safely overwrite any shared
    // staging variable while we run.
    String* urlParam = static_cast<String*>(param);
    {
        String url = urlParam ? *urlParam : String("");

        DebugSerial::println("[SIM800L] Triggering non-volatile data POST transmission...");

        if (!url.endsWith("/")) url += "/";
        String nvUrl = url + "devices/" + getDeviceId();
        DebugSerial::println("[SIM800L] Sending non-volatile data to: " + nvUrl);

        String payload = buildNonVolatileJson();
        String responseBody;
        bool ok = sendHttpPost(nvUrl, payload, &responseBody);

        if (ok && responseBody.length() > 0) {
            // Do NOT apply the response here: this runs on the send task,
            // and applyServerResponse() writes VehicleData/NVS, which
            // FuelCalculator also writes from the loop task. Stage it for
            // update() (loop context) instead.
            stashServerResponse(responseBody);
        } else if (!ok) {
            DebugSerial::println("[SIM800L] Non-volatile POST transmission failed.");
        }

        // If there are stored DTCs, transmit them to the backend
        if (VehicleData::getInstance().getDTCCount() > 0) {
            sendDTCs(url);
        }

        // Drain the real-time detected-error queue (ErrorDetector), if any
        sendDetectedErrors(url);

        // Finally, flush any real-time system errors queued by the
        // ErrorDetector since the last send task run.
        flushPendingSystemErrors(url);
    }

    delete urlParam;
    m_sendInProgress = false;
    vTaskDelete(nullptr);
}

bool SIM800LManager::sendNonVolatileData(const String& baseUrl) {
    if (m_sendInProgress) {
        DebugSerial::println("[SIM800L] WARNING: A non-volatile data send is already in progress. Skipping.");
        return false;
    }

    m_sendInProgress = true;

    // The task owns its own copy of the URL (freed in the trampoline).
    String* urlCopy = new String(baseUrl);

    // Pinned to core 1 (same as Arduino's loopTask): the TLS handshake
    // (mbedTLS, via SSLClient) is CPU-heavy and was previously starving
    // IDLE0 on core 0 long enough to trip the task watchdog and reboot the
    // device ("IDLE0 (CPU 0) did not reset the watchdog in time"). Pinning
    // this task to core 1 keeps core 0's idle task free to feed its own
    // watchdog. Stack bumped to 16KB: mbedTLS's handshake/record buffers
    // need considerably more than plain AT-command string handling did.
    BaseType_t result = xTaskCreatePinnedToCore(
        sendTaskTrampoline,
        "sim800SendTask",
        16384,     // stack size: TLS handshake (mbedTLS) + URL/JSON Strings, AT responses, etc.
        urlCopy,
        1,         // low-ish priority, don't starve BLE/other critical tasks
        nullptr,
        1          // core 1 (alongside loopTask), keep core 0's IDLE task free
    );

    if (result != pdPASS) {
        DebugSerial::println("[SIM800L] ERROR: Failed to create send task.");
        delete urlCopy;
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

    if (!doc["diagnoseMode"].isNull()) {
        int newMode = 0;
        if (doc["diagnoseMode"].is<bool>()) {
            newMode = doc["diagnoseMode"].as<bool>() ? 1 : 0;
        } else {
            newMode = doc["diagnoseMode"].as<int>() != 0 ? 1 : 0;
        }
        VehicleData::getInstance().setDiagnoseMode(newMode);
        DebugSerial::println("[SIM800L] Server updated diagnoseMode to: " + String(newMode));
        if (newMode == 1) {
            diagnoseModeReceivedFlag = 1;
        }
    }

    if (doc["sync_version"].isNull()) {
        DebugSerial::println("[SIM800L] Server response without sync_version. Ignoring non-volatile values.");
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

    // The fuel consumption factor is a tiny value (~1e-7), so a fixed
    // epsilon comparable to the value itself would swallow legitimate
    // small calibration adjustments (the old 7.8e-8 epsilon ignored any
    // change within ~100% of the current value). Compare relatively
    // instead: anything more than 1% away from the local value counts.
    const float localFactor = d.getFuelConsumptionFactor();
    bool factorChanged = (fabsf(factor - localFactor) > fabsf(localFactor) * 0.01f);

    bool changed =
        differs(gasoline, d.getGasolineLevel(), 0.01f) ||
        differs(tank, d.getTankCapacity(), 0.01f) ||
        factorChanged ||
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

String SIM800LManager::buildDTCJson() {
    const std::vector<String>& dtcs = VehicleData::getInstance().getDTCs();
    String json = "{\"dtcs\": [";
    for (size_t i = 0; i < dtcs.size(); i++) {
        json += "\"" + escapeJsonString(dtcs[i]) + "\"";
        if (i + 1 < dtcs.size()) {
            json += ", ";
        }
    }
    json += "]}";
    return json;
}

void SIM800LManager::queueSystemError(uint8_t id, const String& title, const String& description) {
    // Escape via ArduinoJson so titles/descriptions with quotes or
    // special characters produce valid JSON.
    JsonDocument doc;
    doc["id"] = id;
    doc["title"] = title;
    doc["description"] = description;

    String json;
    serializeJson(doc, json);

    // Heap allocation inside push_back() must never run with interrupts
    // disabled, so a normal mutex guards the queue (never used from ISR).
    if (m_pendingErrorsMux != nullptr) {
        xSemaphoreTake(m_pendingErrorsMux, portMAX_DELAY);
    }
    // Bounded queue: if the network is dead for a long time, drop the OLDEST
    // queued error instead of growing the heap without limit.
    while (m_pendingSystemErrors.size() >= kMaxPendingSystemErrors) {
        m_pendingSystemErrors.erase(m_pendingSystemErrors.begin());
    }
    m_pendingSystemErrors.push_back(json);
    if (m_pendingErrorsMux != nullptr) {
        xSemaphoreGive(m_pendingErrorsMux);
    }
    DebugSerial::println("[SIM800L] System error #" + String(id) +
                         " queued (" + String(pendingSystemErrorCount()) + " pending).");
}

size_t SIM800LManager::pendingSystemErrorCount() {
    if (m_pendingErrorsMux != nullptr) {
        xSemaphoreTake(m_pendingErrorsMux, portMAX_DELAY);
    }
    size_t n = m_pendingSystemErrors.size();
    if (m_pendingErrorsMux != nullptr) {
        xSemaphoreGive(m_pendingErrorsMux);
    }
    return n;
}

// Runs ONLY on the background cellular send task (sendTaskTrampoline):
// it performs blocking HTTP POSTs, which must never happen in loop()
// context. The serial mutex held by sendHttpPost() also serializes
// these against the non-volatile data / DTC posts in the same task.
void SIM800LManager::flushPendingSystemErrors(const String& baseUrl) {
    // Move the queue out under the mutex so new errors queued while
    // we transmit are kept for the next flush instead of being lost.
    std::vector<String> batch;
    if (m_pendingErrorsMux != nullptr) {
        xSemaphoreTake(m_pendingErrorsMux, portMAX_DELAY);
    }
    batch.swap(m_pendingSystemErrors);
    if (m_pendingErrorsMux != nullptr) {
        xSemaphoreGive(m_pendingErrorsMux);
    }

    if (batch.empty()) return;

    String url = baseUrl;
    if (!url.endsWith("/")) url += "/";
    url += "device/sendSystemError?idDevices=" + getDeviceId();

    DebugSerial::println("[SIM800L] Flushing " + String(batch.size()) + " queued system error(s).");

    std::vector<String> failed;
    for (size_t i = 0; i < batch.size(); i++) {
        String responseBody;
        int statusCode = 0;
        bool ok = sendHttpPost(url, batch[i], &responseBody, &statusCode);
        if (ok && statusCode >= 200 && statusCode < 300) {
            DebugSerial::println("[SIM800L] System error POST sent (HTTP " + String(statusCode) + ").");
        } else {
            DebugSerial::println("[SIM800L] ERROR: system error POST failed (HTTP " +
                                 String(statusCode) + "). Re-queueing payload for the next send.");
            failed.push_back(batch[i]);
        }
    }

    if (failed.empty()) return;

    // Re-queue the failures at the FRONT (order preserved) so they are
    // retried on the next send task run. Bound the queue so a persistently
    // dead network can't grow it forever: the oldest failures are dropped.
    if (m_pendingErrorsMux != nullptr) {
        xSemaphoreTake(m_pendingErrorsMux, portMAX_DELAY);
    }
    m_pendingSystemErrors.insert(m_pendingSystemErrors.begin(),
                                 failed.begin(), failed.end());
    if (m_pendingSystemErrors.size() > kMaxPendingSystemErrors) {
        m_pendingSystemErrors.erase(
            m_pendingSystemErrors.begin(),
            m_pendingSystemErrors.begin() + (m_pendingSystemErrors.size() - kMaxPendingSystemErrors));
    }
    if (m_pendingErrorsMux != nullptr) {
        xSemaphoreGive(m_pendingErrorsMux);
    }
}

bool SIM800LManager::sendDTCs(const String& baseUrl) {
    if (VehicleData::getInstance().getDTCCount() == 0) {
        DebugSerial::println("[SIM800L] No DTCs to send.");
        return true;
    }

    String url = baseUrl;
    if (!url.endsWith("/")) url += "/";
    url += "devices/sendDTCs?idDevice=" + getDeviceId();

    DebugSerial::println("[SIM800L] Sending DTCs to: " + url);
    String payload = buildDTCJson();
    String responseBody;
    int statusCode = 0;

    sendHttpPost(url, payload, &responseBody, &statusCode);

    if (statusCode == 200) {
        DebugSerial::println("[SIM800L] DTCs successfully sent (HTTP 200). Clearing stored DTCs.");
        VehicleData::getInstance().clearDTCs();
        return true;
    } else {
        DebugSerial::println("[SIM800L] ERROR: Failed to send DTCs. Status: " + String(statusCode) + ". Preserving DTCs.");
        return false;
    }
}

String SIM800LManager::escapeJsonString(const String& value) {
    String out;
    out.reserve(value.length() + 8);
    for (size_t i = 0; i < value.length(); i++) {
        char c = value.charAt(i);
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if ((unsigned char)c < 0x20) {
                    char buf[8];
                    snprintf(buf, sizeof(buf), "\\u%04X", c);
                    out += buf;
                } else {
                    out += c;
                }
                break;
        }
    }
    return out;
}

bool SIM800LManager::sendDetectedErrors(const String& baseUrl) {
    VehicleData& data = VehicleData::getInstance();
    bool allSent = true;

    // One POST per error, as the endpoint expects a single {title, description}
    // body. The front error is only popped AFTER a confirmed 2xx response, so
    // a failed POST leaves it (and everything behind it) queued and retried on
    // the next send task run.
    DetectedError err;
    while (data.peekDetectedError(err)) {
        String url = baseUrl;
        if (!url.endsWith("/")) url += "/";
        url += "devices/" + getDeviceId() + "/addDetectError";

        String payload = "{\"title\":\"" + escapeJsonString(err.title) +
                         "\",\"description\":\"" + escapeJsonString(err.description) +
                         "\"}";

        DebugSerial::println("[SIM800L] Sending detected error to: " + url);
        DebugSerial::println("[SIM800L] Payload: " + payload);

        int statusCode = 0;
        sendHttpPost(url, payload, nullptr, &statusCode);

        if (statusCode >= 200 && statusCode < 300) {
            // Delivered: remove it from the queue and continue with the next.
            data.popDetectedError(err);
            DebugSerial::println("[SIM800L] Detected error sent (HTTP " + String(statusCode) + ").");
        } else {
            DebugSerial::println("[SIM800L] ERROR: Failed to send detected error. Status: " +
                                 String(statusCode) + ". Keeping remaining errors queued.");
            allSent = false;
            break;
        }
    }

    return allSent;
}

bool SIM800LManager::sendHttpPost(const String& url, const String& jsonPayload, String* responseBody, int* statusCodeOut) {
    if (!m_hardwareAvailable) {
        DebugSerial::println("[SIM800L] ERROR: Cannot send HTTP POST (Hardware unavailable).");
        return false;
    }

    if (m_status == ESIMSTATUS::WITHOUT_SIM) {
        DebugSerial::println("[SIM800L] ERROR: Cannot send HTTP POST (WITHOUT_SIM).");
        return false;
    }

    // Holds the serial mutex for the WHOLE operation (GPRS bring-up +
    // TCP/TLS connect + HTTP request/response), not just per-AT-command:
    // TinyGsmClient/SSLClient talk to m_serial directly (not through
    // sendATCommand()), so without this, readIncoming()/tryRecover() running
    // from the loop()-context timer could interleave/steal bytes mid-handshake,
    // corrupting the TLS stream (this was the root cause of the "invalid SSL
    // record" / handshake timeout errors).
    if (m_serialMutex != nullptr) {
        xSemaphoreTakeRecursive(m_serialMutex, portMAX_DELAY);
    }

    bool result = sendHttpPostLocked(url, jsonPayload, responseBody, statusCodeOut);

    if (m_serialMutex != nullptr) {
        xSemaphoreGiveRecursive(m_serialMutex);
    }

    return result;
}

bool SIM800LManager::sendHttpPostLocked(const String& url, const String& jsonPayload, String* responseBody, int* statusCodeOut) {
    if (m_status != ESIMSTATUS::CONNECTED) {
        DebugSerial::println("[SIM800L] eSIM disconnected. Establishing GPRS first...");
        if (!connectGPRSLocked()) {
            DebugSerial::println("[SIM800L] ERROR: GPRS connection could not be established.");
            return false;
        }
    }

    if (!ensureModemClientsInitialized()) {
        DebugSerial::println("[SIM800L] ERROR: Could not initialize TinyGsm modem/client.");
        return false;
    }

    DebugSerial::println("[SIM800L] Sending HTTP POST to: " + url);
    DebugSerial::println("[SIM800L] Payload: " + jsonPayload);

    bool isHttps = url.startsWith("https://");

    // The SIM800L is used here only as a "dumb modem": TinyGsmClient opens a
    // raw TCP socket over it (AT+CIPSTART/AT+CIPSEND/AT+CIPCLOSE under the
    // hood). For HTTPS, SSLClient wraps that TCP socket and performs the
    // actual TLS 1.2/1.3 handshake + encryption using the ESP32's own CPU
    // (mbedTLS), completely bypassing the SIM800L's obsolete SSLv3/TLS1.0-only
    // engine that was causing HTTPACTION 606 against modern TLS-only
    // backends (e.g. Tailscale Funnel).
    int statusCode = 0;
    bool success = false;

    if (isHttps) {
        success = httpPostOverClient(*m_sslClient, url, jsonPayload, responseBody, &statusCode);
    } else {
        success = httpPostOverClient(*m_gsmClient, url, jsonPayload, responseBody, &statusCode);
    }

    if (statusCodeOut != nullptr) {
        *statusCodeOut = statusCode;
    }

    DebugSerial::println("[SIM800L] HTTP POST Result Code: " + String(statusCode));

    if (success) {
        DebugSerial::println("[SIM800L] HTTP POST successfully completed!");
    } else {
        DebugSerial::println("[SIM800L] ERROR: HTTP POST failed with HTTP status: " + String(statusCode));
    }

    // statusCode == 0 means we couldn't even connect/get a response (DNS,
    // TCP connect, or TLS handshake failure) - that's a network-level problem,
    // so run the connect-failure diagnostics WHILE the GPRS PDP context is
    // still alive (DNS query and raw TCP test need it), then drop GPRS and
    // run the general (throttled) diagnostics, same policy as before. A
    // proper 4xx/5xx from the server is NOT a network problem.
    if (statusCode == 0) {
        logConnectFailureDiagnostics(url);
        disconnectGPRS();
        logNetworkDiagnostics();
    }

    return success;
}

// Minimal HTTP/1.1 POST implementation over any Arduino Client (works for
// both the plain TinyGsmClient - HTTP - and the SSLClient-wrapped one -
// HTTPS). Parses the connection host/path out of the URL, writes the request
// line + headers + body manually, then reads the status line, headers
// (looking for Content-Length), and body.
bool SIM800LManager::httpPostOverClient(Client& client, const String& url,
                                        const String& jsonPayload, String* responseBody,
                                        int* statusCode) {
    if (statusCode) *statusCode = 0;

    // --- Parse URL into host/port/path ---
    String rest = url;
    bool https = false;
    if (rest.startsWith("https://")) {
        https = true;
        rest.remove(0, 8);
    } else if (rest.startsWith("http://")) {
        rest.remove(0, 7);
    }

    int pathIdx = rest.indexOf('/');
    String hostPort = (pathIdx == -1) ? rest : rest.substring(0, pathIdx);
    String path = (pathIdx == -1) ? "/" : rest.substring(pathIdx);
    if (path.length() == 0) path = "/";

    String host = hostPort;
    int port = https ? 443 : 80;
    int colonIdx = hostPort.indexOf(':');
    if (colonIdx != -1) {
        host = hostPort.substring(0, colonIdx);
        port = hostPort.substring(colonIdx + 1).toInt();
    }

    DebugSerial::println("[SIM800L] Connecting to " + host + ":" + String(port) +
                         (https ? " (TLS)" : " (plain TCP)") + "...");

    // Connection + TLS handshake (if any) happens inside connect().
    if (!client.connect(host.c_str(), port)) {
        DebugSerial::println("[SIM800L] ERROR: TCP/TLS connection to " + host + " failed.");
        return false;
    }

    // --- Send request ---
    // Every print() result is checked: over TinyGsmClient writes go through
    // AT+CIPSEND chunking, and a full modem buffer causes a short/failed
    // write. A truncated request (with an intact Content-Length) makes the
    // server wait for missing body bytes until the 30s deadline, then gets
    // misreported as a network failure (GPRS teardown + diagnostics storm).
    auto writeAll = [&](const String& chunk) -> bool {
        return client.print(chunk) == chunk.length();
    };
    if (!writeAll("POST " + path + " HTTP/1.1\r\n") ||
        !writeAll("Host: " + host + "\r\n") ||
        !writeAll("Content-Type: application/json\r\n") ||
        !writeAll("Content-Length: " + String(jsonPayload.length()) + "\r\n") ||
        !writeAll("Connection: close\r\n") ||
        !writeAll("\r\n") ||
        !writeAll(jsonPayload)) {
        DebugSerial::println("[SIM800L] ERROR: short write while sending HTTP request (truncated).");
        client.stop();
        return false;
    }

    // --- Read response ---
    // Everything below is bounded by a shared (rollover-safe) start time +
    // timeout so a stalled peer can never hang the send task (and thus
    // m_sendInProgress) indefinitely.
    const unsigned long kResponseTimeoutMs = 30000;
    unsigned long responseStart = millis();

    // Status line, e.g. "HTTP/1.1 200 OK"
    String statusLine;
    if (!readLineWithTimeout(client, statusLine, responseStart, kResponseTimeoutMs)) {
        DebugSerial::println("[SIM800L] ERROR: No status line received before timeout.");
        client.stop();
        return false;
    }
    int code = 0;
    int firstSpace = statusLine.indexOf(' ');
    if (firstSpace != -1) {
        code = statusLine.substring(firstSpace + 1, firstSpace + 4).toInt();
    }
    if (statusCode) *statusCode = code;

    // Headers
    long contentLength = -1;
    for (;;) {
        String line;
        if (!readLineWithTimeout(client, line, responseStart, kResponseTimeoutMs)) break;
        line.trim();
        if (line.length() == 0) break; // blank line = end of headers

        String lower = line;
        lower.toLowerCase();
        if (lower.startsWith("content-length:")) {
            contentLength = line.substring(line.indexOf(':') + 1).toInt();
        }
    }

    // Body
    String body;
    if (contentLength >= 0) {
        body.reserve(contentLength);
        long remaining = contentLength;
        unsigned long bodyStart = millis();
        while (remaining > 0 && (millis() - bodyStart < kResponseTimeoutMs)) {
            if (client.available()) {
                body += (char)client.read();
                remaining--;
            } else {
                delay(5);
            }
        }
    } else {
        // No Content-Length (or chunked, unsupported): just drain whatever
        // arrives until the server closes the connection.
        unsigned long bodyStart = millis();
        while ((client.connected() || client.available()) && (millis() - bodyStart < kResponseTimeoutMs)) {
            if (client.available()) {
                body += (char)client.read();
            } else {
                delay(5);
            }
        }
    }

    client.stop();

    if (responseBody) *responseBody = body;

    return (code >= 200 && code < 300);
}

// ---------------------------------------------------------------------------
// Connect-failure diagnostics (DNS vs TCP reachability vs transient)
// ---------------------------------------------------------------------------
// Called from sendHttpPostLocked() when the TCP/TLS connect() fails, while the
// GPRS PDP context is still alive (must run BEFORE disconnectGPRS()).
// NOTE: AT+CIPSTART/AT+CDNSGIP are NOT usable here: while TinyGSM manages the
// data connection the modem answers "+CME ERROR: operation not allowed" to
// them (confirmed on the field), so those results are inconclusive. Instead,
// the tests below use the exact same path the firmware uses (TinyGsmClient):
//   1. Raw TCP to the Funnel relay BY IP  -> carrier path + port reachability
//   2. Raw TCP to the backend BY HOSTNAME -> adds DNS on top
// If (1) passes and (2) fails, the problem is 100% carrier DNS.
void SIM800LManager::logConnectFailureDiagnostics(const String& url) {
    if (!ensureModemClientsInitialized()) {
        DebugSerial::println("[SIM800L][NETDIAG] Modem/client not initialized, cannot run diagnostics.");
        return;
    }

    // Parse the host out of the URL (same logic as httpPostOverClient()).
    String rest = url;
    if (rest.startsWith("https://")) rest.remove(0, 8);
    else if (rest.startsWith("http://")) rest.remove(0, 7);
    int pathIdx = rest.indexOf('/');
    String hostPort = (pathIdx == -1) ? rest : rest.substring(0, pathIdx);
    int colonIdx = hostPort.indexOf(':');
    String host = (colonIdx == -1) ? hostPort : hostPort.substring(0, colonIdx);

    DebugSerial::println("[SIM800L][NETDIAG] ---- Connect failure diagnostics ----");

    // Quick signal snapshot at the exact moment of the failure.
    String csq = sendATCommand("AT+CSQ", 2000, "OK");
    int csqIdx = csq.indexOf("+CSQ:");
    if (csqIdx != -1) {
        int rssi = csq.substring(csqIdx + 5).toInt();
        if (rssi == 99) {
            DebugSerial::println("[SIM800L][NETDIAG] Signal: 99 (sem sinal/desconhecido)");
        } else {
            DebugSerial::println("[SIM800L][NETDIAG] Signal: " + String(rssi) +
                                 " (~" + String(-113 + 2 * rssi) + " dBm)");
        }
    }

    // 1. Raw TCP to the Tailscale Funnel relay BY IP, bypassing DNS entirely
    //    (address from FUNNEL_RELAY_IP in config.h; verify it still matches
    //    the backend's funnel host if the endpoint ever changes). This uses
    //    the very same TinyGsmClient path the firmware uses for the real
    //    request, so it is a faithful test of carrier reachability.
    String relayIp = FUNNEL_RELAY_IP;
    bool ipOk = m_gsmClient->connect(relayIp.c_str(), 443);
    if (ipOk) {
        DebugSerial::println("[SIM800L][NETDIAG] Raw TCP " + relayIp + ":443 (relay Funnel): CONNECTED -> operadora permite o trafego; a falha e de DNS.");
        m_gsmClient->stop();
    } else {
        DebugSerial::println("[SIM800L][NETDIAG] Raw TCP " + relayIp + ":443 (relay Funnel): FAILED.");
        // Control tests: if a well-known host ALSO fails, the carrier is
        // blocking ALL outbound TCP (or data is effectively down despite the
        // IP assignment). If they succeed, the block is specific to the
        // Tailscale relay address range.
        bool ctrlIpOk = m_gsmClient->connect("1.1.1.1", 443);
        DebugSerial::println("[SIM800L][NETDIAG] Control raw TCP 1.1.1.1:443: " +
                             (ctrlIpOk ? String("CONNECTED -> bloqueio especifico do relay do Tailscale, nao da operadora em geral.") : String("FAILED -> TODA saida TCP esta bloqueada/sem dados reais (IP atribuido mas PDP inoperante).")));
        if (ctrlIpOk) m_gsmClient->stop();

        bool ctrlDnsOk = m_gsmClient->connect("google.com", 443);
        DebugSerial::println("[SIM800L][NETDIAG] Control raw TCP google.com:443 (via DNS): " +
                             (ctrlDnsOk ? String("CONNECTED -> DNS e TCP gerais funcionam.") : String("FAILED.")));
        if (ctrlDnsOk) m_gsmClient->stop();
    }

    // 2. Raw TCP to the backend hostname (TinyGSM resolves via the modem's
    //    DNS). If (1) connected but this fails, carrier DNS is the culprit.
    if (ipOk) {
        bool dnsOk = m_gsmClient->connect(host.c_str(), 443);
        if (dnsOk) {
            DebugSerial::println("[SIM800L][NETDIAG] Raw TCP " + host + ":443 (via DNS): CONNECTED -> DNS e rede OK; falha anterior era transiente (sinal/GPRS).");
            m_gsmClient->stop();
        } else {
            DebugSerial::println("[SIM800L][NETDIAG] Raw TCP " + host + ":443 (via DNS): FAILED -> DNS da operadora nao resolve " + host + " (problema especifico do dominio).");
        }
    } else {
        DebugSerial::println("[SIM800L][NETDIAG] DNS test skipped (raw TCP by IP already failed).");
    }

    DebugSerial::println("[SIM800L][NETDIAG] ---- End connect failure diagnostics ----");
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
    // Recursive mutex: the send task holds this same lock around the whole
    // connectGPRS()/HTTP-TLS-POST operation (see sendHttpPost()), and calls
    // back into sendATCommand() from within that span.
    if (m_serialMutex != nullptr) {
        xSemaphoreTakeRecursive(m_serialMutex, portMAX_DELAY);
    }

    while (m_serial->available()) {
        m_serial->read();
    }

    DebugSerial::println("[SIM800L] TX: " + command);
    m_serial->print(command);
    m_serial->print("\r\n");

    String response = readSerialResponse(m_serial, timeoutMs, expectedResponse);

    if (m_serialMutex != nullptr) {
        xSemaphoreGiveRecursive(m_serialMutex);
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

void SIM800LManager::update() {
    // Apply a server response stashed by the background send task, on the
    // loop context (see stashServerResponse()).
    // Heap-safety: the copy is made into a stack buffer INSIDE the critical
    // section (memcpy only); the String constructor would heap-allocate with
    // interrupts disabled, which is illegal on ESP32.
    char bodyBuf[sizeof(m_pendingResponseBuffer)];
    bool hasPending = false;
    portENTER_CRITICAL(&m_pendingResponseMux);
    if (m_pendingResponseReady) {
        m_pendingResponseReady = false;
        memcpy(bodyBuf, m_pendingResponseBuffer, sizeof(bodyBuf));
        hasPending = true;
    }
    portEXIT_CRITICAL(&m_pendingResponseMux);
    if (hasPending) {
        applyServerResponse(String(bodyBuf));
        return; // continue with the rest on the next tick
    }

    // Hardware not responding or SIM reported missing are otherwise permanent
    // barriers (both are only ever cleared from begin()/checkSimStatus()).
    // Give them a throttled chance to self-heal in the background: a modem
    // that comes back up, or a SIM that gets inserted/activated later, can
    // then resume telemetry without requiring a device restart. Scheduling
    // only; the blocking probing happens on a dedicated task (see
    // tryRecoverScheduled()) so loop() is never stalled by it.
    if (!m_hardwareAvailable || m_status == ESIMSTATUS::WITHOUT_SIM) {
        tryRecoverScheduled();
    }

    readIncoming();
}

void SIM800LManager::stashServerResponse(const String& body) {
    size_t copyLen = body.length();
    if (copyLen >= sizeof(m_pendingResponseBuffer)) {
        // Truncating a JSON payload makes it unparsable; applyServerResponse()
        // will then log the error and keep local values. The real backend
        // responses (diagnoseMode + a handful of numbers) are far shorter.
        copyLen = sizeof(m_pendingResponseBuffer) - 1;
    }
    portENTER_CRITICAL(&m_pendingResponseMux);
    if (m_pendingResponseReady) {
        // A previous response has not been consumed by update() yet (two
        // sends completing within one update tick). Dropping the NEW one
        // keeps the pending slot consistent; a diagnoseMode change that gets
        // lost here is re-sent by the backend on the next sync.
        portEXIT_CRITICAL(&m_pendingResponseMux);
        DebugSerial::println("[SIM800L] WARNING: server response stash still pending, new response dropped.");
        return;
    }
    memcpy(m_pendingResponseBuffer, body.c_str(), copyLen);
    m_pendingResponseBuffer[copyLen] = '\0';
    m_pendingResponseReady = true;
    portEXIT_CRITICAL(&m_pendingResponseMux);
}

// ---------------------------------------------------------------------------
// Background recovery for hardware-unavailable / WITHOUT_SIM states
// ---------------------------------------------------------------------------
// Light scheduler, called from update() (loop context). Only checks the
// cooldown and spawns a short-lived task; none of the blocking AT probing
// happens here (it used to run inline, freezing loop() - and with it BLE,
// the LCD and the fuel calculator - for up to ~20s every 30s while the
// module was dead or the SIM was out).
void SIM800LManager::tryRecoverScheduled() {
    const unsigned long kRecoveryCooldownMs = 30UL * 1000UL;

    unsigned long now = millis();
    if (m_lastRecoveryAttemptMs != 0 && (now - m_lastRecoveryAttemptMs < kRecoveryCooldownMs)) {
        return;
    }
    if (m_recoveryInProgress || m_sendInProgress) {
        return; // probe next cycle after the other task frees the line
    }
    m_lastRecoveryAttemptMs = now;
    m_recoveryInProgress = true;

    if (xTaskCreatePinnedToCore(recoverTaskTrampoline, "sim800RecoverTask", 8192,
                                 nullptr, 1, nullptr, 1) != pdPASS) {
        DebugSerial::println("[SIM800L][RECOVERY] ERROR: Failed to create recovery task.");
        m_recoveryInProgress = false;
        m_lastRecoveryAttemptMs = 0; // retry on the next cycle
    }
}

// Runs on the "sim800RecoverTask" FreeRTOS task; safe to block here.
void SIM800LManager::recoverTaskTrampoline(void* param) {
    (void)param;
    tryRecover();
    m_recoveryInProgress = false;
    vTaskDelete(nullptr);
}

void SIM800LManager::tryRecover() {
    // Don't fight the background send task for the serial line.
    if (m_serialMutex != nullptr) {
        if (xSemaphoreTakeRecursive(m_serialMutex, 0) != pdTRUE) {
            // Count this as a real attempt so the 30s cooldown is honored;
            // leaving it at 0 used to make tryRecoverScheduled() spawn a new
            // 8KB task on EVERY update() tick (100ms) while the mutex was busy.
            m_lastRecoveryAttemptMs = millis();
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
        xSemaphoreGiveRecursive(m_serialMutex);
    }
}

void SIM800LManager::readIncoming() {
    if (m_serial == nullptr) return;

    // Non-blocking attempt: if the background send task
    // (sendNonVolatileData()->sendHttpPost()) currently holds the serial
    // mutex, just skip this cycle rather than blocking loop() waiting for
    // it - there will be another chance in SIM800_UPDATE_INTERVAL_MS.
    if (m_serialMutex != nullptr) {
        if (xSemaphoreTakeRecursive(m_serialMutex, 0) != pdTRUE) {
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
                // "CLOSED" must be matched as a standalone URC line only:
                // an indexOf match also fired on the word appearing inside
                // SMS text / operator names / echoed payloads, dropping a
                // healthy GPRS state and forcing pointless reconnects.
                } else if (line.indexOf("+PDP: DEACT") != -1 || line.startsWith("CLOSED")) {
                    setStatus(ESIMSTATUS::DISCONNECTED);
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
        xSemaphoreGiveRecursive(m_serialMutex);
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