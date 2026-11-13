#ifndef SIM800L_MANAGER_H
#define SIM800L_MANAGER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <functional>
#include <esp_mac.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <TinyGsmClient.h>
#include <SSLClient.h>
#include "config.h"
#include "CallbackManager.h"
#include "DebugSerial.h"
#include "VehicleData.h"

/**
 * @class SIM800LManager
 * @brief Cellular communication manager based on the SIM800L module.
 *        Follows the architectural pattern of OBDManager:
 *        - Static methods and lifecycle management
 *        - Flags for CallbackManager flag watchers
 *        - Command queue and AT protocol processing
 *        - Hardware initialization and eSIM status monitoring
 *        - Structured HTTP POST routines for non-volatile telemetry
 */
class SIM800LManager {
public:
    // -------------------------------------------------------------------------
    // Hardware and eSIM Initialization
    // -------------------------------------------------------------------------
    /**
     * @brief Initializes hardware serial, reset pin, and sets initial eSIM status.
     * @param baudRate Baud rate for SIM800L communication (default: SIM800_BAUD_RATE)
     * @param rxPin ESP32 RX pin connected to SIM800L TX (default: SIM800_RX_PIN)
     * @param txPin ESP32 TX pin connected to SIM800L RX (default: SIM800_TX_PIN)
     * @param rstPin Reset pin connected to SIM800L RST (default: SIM800_RST_PIN)
     */
    static void begin(unsigned long baudRate = SIM800_BAUD_RATE,
                      int8_t rxPin = SIM800_RX_PIN,
                      int8_t txPin = SIM800_TX_PIN,
                      int8_t rstPin = SIM800_RST_PIN);

    // -------------------------------------------------------------------------
    // eSIM Status and Network Management
    // -------------------------------------------------------------------------
    static ESIMSTATUS getStatus();
    static void setStatus(ESIMSTATUS newStatus);
    static bool isConnected();
    static bool checkSimStatus();
    static bool connectGPRS();
    static bool disconnectGPRS();

    // -------------------------------------------------------------------------
    // HTTP POST Data Transmission
    // -------------------------------------------------------------------------
    /**
     * @brief Sends an HTTP POST request with the given JSON payload to the specified URL.
     * @param url Destination HTTP endpoint.
     * @param jsonPayload The JSON string body.
     * @param responseBody Optional pointer to String to store server response body.
     * @return true if HTTP response status is 2xx, false otherwise.
     */
    static bool sendHttpPost(const String& url, const String& jsonPayload, String* responseBody = nullptr, int* statusCodeOut = nullptr);

    /**
     * @brief Sends stored Diagnostic Trouble Codes (DTCs) to backend via HTTP POST.
     *        If HTTP response status is 200, the DTCs are cleared from VehicleData.
     *        Endpoint: device/sendDTCs?idDevices=<mac>
     *        Body format: {"DTCs": ["FFFF", "FFFF"]}
     * @param baseUrl Base backend URL (default: BACKEND_HTTP_ENDPOINT)
     * @return true if HTTP 200 OK was received and DTCs were cleared, false otherwise.
     */
    static bool sendDTCs(const String& baseUrl = BACKEND_HTTP_ENDPOINT);

    /**
     * @brief Formats stored DTCs into JSON body: {"DTCs": ["FFFF", "FFFF"]}
     */
    static String buildDTCJson();

    /**
     * @brief Formats WiFi STA MAC address as XX:XX:XX:XX:XX:XX.
     */
    static String getDeviceId();

    /**
     * @brief Builds a JSON payload containing all persistent/non-volatile data
     *        stored in VehicleData and sends it via HTTP POST to the backend endpoint.
     *        The actual blocking AT-command/HTTP sequence (which can take many
     *        seconds, e.g. up to 20s just for AT+HTTPACTION) runs on a dedicated
     *        FreeRTOS task instead of inline, so it never blocks loop()/CallbackManager
     *        (BLE handling, display updates, OBD polling, etc. would otherwise all
     *        freeze for the whole duration of the HTTP transaction).
     * @param url Optional override for destination URL (default: BACKEND_HTTP_ENDPOINT)
     * @return true if the send task was successfully started (NOT whether the HTTP
     *         POST itself succeeded - that happens asynchronously). Returns false if
     *         a send is already in progress or the task could not be created.
     */
    static bool sendNonVolatileData(const String& url = BACKEND_HTTP_ENDPOINT);

    /**
     * @brief Whether a background sendNonVolatileData() task is currently running.
     */
    static bool isSendInProgress();

    /**
     * @brief Consolidates all non-volatile system data from VehicleData into a JSON string.
     * @return JSON string representing tankCapacity, gasolineLevel, fuelConsumptionFactor,
     *         totalDistance, and tripConsumption.
     */
    static String buildNonVolatileJson();

    // -------------------------------------------------------------------------
    // AT Communication (OBDManager Pattern)
    // -------------------------------------------------------------------------
    static String sendATCommand(const String& command,
                                unsigned long timeoutMs = DEFAULT_TIMEOUT,
                                const char* expectedResponse = "OK");
    static bool ensureAllBand();
    static void logNetworkDiagnostics(bool force = false);

    // -------------------------------------------------------------------------
    // Periodic Processing Routine
    // -------------------------------------------------------------------------
    static void update();

    // -------------------------------------------------------------------------
    // Callback Registration
    // -------------------------------------------------------------------------
    static void setRawMessageCallback(RawMessageCallback callback);


    // -------------------------------------------------------------------------
    // Public Flags for CallbackManager Watcher Integration
    // -------------------------------------------------------------------------
    static int esimConnectedFlag;
    static int esimDisconnectedFlag;
    static int esimWithoutSimFlag;
    static int diagnoseModeReceivedFlag;

    // Convenience reference aliases
    static int& simConnectedFlag;
    static int& simDisconnectedFlag;
    static int& withoutSimFlag;

private:
    static HardwareSerial* m_serial;
    static int8_t m_rstPin;
    static ESIMSTATUS m_status;
    static bool m_hardwareAvailable;
    static bool m_callbackInit;
    static RawMessageCallback m_rawMessageCallback;

    // Actual connectGPRS() body, run while already holding m_serialMutex for
    // its entire duration (see connectGPRS() in the .cpp for why).
    static bool connectGPRSLocked();

    // Actual sendHttpPost() body, run while already holding m_serialMutex for
    // its entire duration (see sendHttpPost() in the .cpp for why).
    static bool sendHttpPostLocked(const String& url, const String& jsonPayload, String* responseBody, int* statusCodeOut = nullptr);

    // TinyGSM talks to the SIM800L purely as a "dumb modem" (AT+CIPSTART /
    // AT+CIPSEND raw TCP socket), and SSLClient performs the actual TLS
    // 1.2/1.3 handshake + encryption on the ESP32 itself (via its mbedTLS /
    // BearSSL-equivalent stack), since the SIM800L's own built-in SSL engine
    // only supports SSLv3/TLS1.0 and fails modern handshakes (HTTPACTION 606).
    static TinyGsm* m_modem;
    static TinyGsmClient* m_gsmClient;
    static SSLClient* m_sslClient;
    static bool ensureModemClientsInitialized();

    // Minimal hand-rolled HTTP/1.1 client over an Arduino Client (TCP or TLS).
    // Returns true on 2xx status; fills statusCode/responseBody when provided.
    static bool httpPostOverClient(Client& client, const String& url,
                                   const String& jsonPayload, String* responseBody,
                                   int* statusCode);

    // Connect-failure diagnostics (DNS vs TCP reachability vs transient).
    // Runs via AT commands while the GPRS PDP context is still alive, i.e.
    // must be called BEFORE disconnectGPRS(). See implementation for details.
    static void logConnectFailureDiagnostics(const String& url);

    // Line accumulation for readIncoming()
    static String m_lastResponse;

    static void readIncoming();
    static void applyServerResponse(const String& body);

    // Server-response staging: the background send task only STASHES the
    // HTTP response body here (fixed-size buffer + portMUX, no heap
    // allocation inside the critical section); update() (loop context)
    // picks it up and feeds applyServerResponse(). This keeps all
    // VehicleData/NVS writes from backend sync on the loop task, where
    // FuelCalculator also runs, instead of racing it from the send task.
    static portMUX_TYPE m_pendingResponseMux;
    static char m_pendingResponseBuffer[512];
    static volatile bool m_pendingResponseReady;
    static void stashServerResponse(const String& body);

    // Background recovery for permanent-looking failure states (hardware
    // not responding / SIM not inserted). tryRecover() itself is blocking
    // (several AT round-trips, seconds of delays), so it must NOT run in
    // loop() context: tryRecoverScheduled() (called from update()) only
    // checks a 30s cooldown and spawns a short-lived task that does the
    // actual probing.
    static void tryRecover();
    static void tryRecoverScheduled();
    static void recoverTaskTrampoline(void* param);
    static volatile bool m_recoveryInProgress;
    static unsigned long m_lastRecoveryAttemptMs;
    static unsigned long m_baudRateForRecovery;
    static int8_t m_rxPinForRecovery;
    static int8_t m_txPinForRecovery;

    // Background send task state (see sendNonVolatileData()).
    static volatile bool m_sendInProgress;
    static String m_sendTaskUrl;
    static void sendTaskTrampoline(void* param);

    // Serializes access to m_serial (and the blocking AT-command exchange)
    // between the periodic readIncoming()/update() (loop()-context, via
    // CallbackManager timer) and the background send task spawned by
    // sendNonVolatileData(). Without this, both could read/write Serial2
    // concurrently and corrupt/steal each other's bytes.
    static SemaphoreHandle_t m_serialMutex;
};

#endif // SIM800L_MANAGER_H
