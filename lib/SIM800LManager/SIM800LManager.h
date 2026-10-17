#ifndef SIM800L_MANAGER_H
#define SIM800L_MANAGER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <queue>
#include <functional>
#include <esp_mac.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
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
    SIM800LManager();
    ~SIM800LManager();

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
    static bool sendHttpPost(const String& url, const String& jsonPayload, String* responseBody = nullptr);

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
    // AT Command Queue & Communication (OBDManager Pattern)
    // -------------------------------------------------------------------------
    static void clearCommandQueue();
    static void addCommandToQueue(const String& command);
    static void sendCommand(const String& command);
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

    // Command queue & execution state
    static std::queue<String> m_commandQueue;
    static String m_lastCommandSent;
    static String m_lastResponse;
    static unsigned long m_lastCommandSentTime;
    static unsigned long m_currentTimeout;
    static bool m_messageReceived;
    static unsigned long m_lastConnectionAttemptTime;

    static String sendRawData(const String& data,
                              unsigned long timeoutMs = 5000,
                              const char* expectedResponse = "OK");
    static void readIncoming();
    static String getDeviceId();
    static void applyServerResponse(const String& body);

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

// Typedef alias for generic architecture references
using CellularManager = SIM800LManager;

#endif // SIM800L_MANAGER_H
