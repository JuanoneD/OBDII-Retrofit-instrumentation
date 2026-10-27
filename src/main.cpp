#include <Arduino.h>
#include "CallbackManager.h"
#include "OBDManager.h"
#include "config.h"
#include "DebugSerial.h"
#include "OBDDecoder.h"
#include "VehicleData.h"
#include "FuelCalculator.h"
#include "DisplayManager.h"
#include "SIM800LManager.h"

DisplayManager display;

uint32_t startOBDIIConnectionID = 0;
uint32_t ecuMessagesSenderID = 0;
uint32_t fuelCalculatorID = 0;
uint32_t displayUpdateID = 0;
uint32_t cellularSenderID = 0;

int messageIndex = 0;

void messageSendingCallback()
{
  if (VehicleData::getInstance().getObdiiStatus() != OBDIISTATUS::CONNECTED) return;

  switch (messageIndex)
  {
  case 0:
    OBDManager::addCommandToQueue(PID_ENGINE_RPM_STR); // RPM
    break;
  case 1:
    OBDManager::addCommandToQueue(PID_VEHICLE_SPEED_STR); // Vehicle Speed
    OBDManager::addCommandToQueue(PID_ENGINE_RPM_STR); // RPM
    break;
  case 2:
    OBDManager::addCommandToQueue(PID_COOLANT_TEMP_STR); // Coolant Temp
    OBDManager::addCommandToQueue(PID_ENGINE_RPM_STR); // RPM
    break;
  case 3:
    OBDManager::addCommandToQueue(PID_ENGINE_LOAD_STR); // Engine Load
    OBDManager::addCommandToQueue(PID_ENGINE_RPM_STR); // RPM
    break;
  case 4:
    OBDManager::addCommandToQueue(PID_LONG_TERM_FUEL_TRIM_STR); // Long Term Fuel Trim
    OBDManager::addCommandToQueue(PID_ENGINE_RPM_STR); // RPM
    messageIndex = -1; // Reset index to -1 so that it becomes 0 on the next increment
    break;
  default:
    messageIndex = -1; // Reset index to -1 so that it becomes 0 on the next increment
    break;
  }
  messageIndex++;
}

void setObdStatustoConnected()
{
  if (VehicleData::getInstance().getObdiiStatus() == OBDIISTATUS::CONNECTED) return;

  CallbackManager::pauseTimer(startOBDIIConnectionID);
  VehicleData::getInstance().setObdiiStatus(OBDIISTATUS::CONNECTED);
  DebugSerial::println("OBDII Connected!");
  CallbackManager::resumeTimer(ecuMessagesSenderID);
  CallbackManager::resumeTimer(fuelCalculatorID);
}

void setEcuStatustoOnline()
{
  if (VehicleData::getInstance().getEcuStatus() == ECUSTATUS::ONLINE) return;

  VehicleData::getInstance().setEcuStatus(ECUSTATUS::ONLINE);
  DebugSerial::println("ECU Online!");
}

void setEcuStatustoOffline()
{
  if (VehicleData::getInstance().getEcuStatus() == ECUSTATUS::OFFLINE) return;

  VehicleData::getInstance().setEcuStatus(ECUSTATUS::OFFLINE);
  DebugSerial::println("ECU Offline!");
}


void setObdStatustoOffline()
{
  if (VehicleData::getInstance().getObdiiStatus() == OBDIISTATUS::DISCONNECTED) return;

  VehicleData::getInstance().setObdiiStatus(OBDIISTATUS::DISCONNECTED);
  DebugSerial::println("OBDII Disconnected!");

  CallbackManager::resumeTimer(startOBDIIConnectionID);
  CallbackManager::pauseTimer(ecuMessagesSenderID);
  CallbackManager::pauseTimer(fuelCalculatorID);

  setEcuStatustoOffline();
  OBDManager::clearCommandQueue();
}

void setObdStatustoTryingToConnect()
{
  if(VehicleData::getInstance().getObdiiStatus() == OBDIISTATUS::CONNECTED) return;

  VehicleData::getInstance().setObdiiStatus(OBDIISTATUS::TRYING_TO_CONNECT);
  DebugSerial::println("OBDII Trying to Connect...");
}

void startOBDIIConnection()
{
  if(VehicleData::getInstance().getObdiiStatus() == OBDIISTATUS::DISCONNECTED)
    OBDManager::scanAndConnect();
}

void setEsimStatustoConnected()
{
  if (VehicleData::getInstance().getEsimStatus() == ESIMSTATUS::CONNECTED) return;

  VehicleData::getInstance().setEsimStatus(ESIMSTATUS::CONNECTED);
  DebugSerial::println("eSIM Connected!");
  CallbackManager::resumeTimer(cellularSenderID);
}

void setEsimStatustoDisconnected()
{
  if (VehicleData::getInstance().getEsimStatus() == ESIMSTATUS::DISCONNECTED) return;

  VehicleData::getInstance().setEsimStatus(ESIMSTATUS::DISCONNECTED);
  DebugSerial::println("eSIM Disconnected!");
  CallbackManager::resumeTimer(cellularSenderID);
}

void setEsimStatustoWithoutSim()
{
  if (VehicleData::getInstance().getEsimStatus() == ESIMSTATUS::WITHOUT_SIM) return;

  VehicleData::getInstance().setEsimStatus(ESIMSTATUS::WITHOUT_SIM);
  DebugSerial::println("eSIM: Without SIM!");
  CallbackManager::pauseTimer(cellularSenderID);
}

void setup() {
    DebugSerial::begin();

    // Initialize persistent data
    VehicleData::getInstance().loadPersistentData();

    // Initialize fuel consumption calculator (loads K from NVS)
    FuelCalculator::getInstance().begin();

    // Initialize LCD (renders "Connecting OBDII..." until status changes).

    if (!display.begin(0x27, 20, 4, 21, 22)) {
        DebugSerial::println("LCD Fail");
    }

    // Class initialization
    OBDManager::setRawMessageCallback(OBDDecoder::decode);
    SIM800LManager::begin();

    // Signals
    CallbackManager::addFlagWatcher(&OBDManager::obdConnectedFlag,setObdStatustoConnected);
    CallbackManager::addFlagWatcher(&OBDManager::obdDisconnectedFlag,setObdStatustoOffline);
    CallbackManager::addFlagWatcher(&OBDManager::obdConnectionAttemptFlag,setObdStatustoTryingToConnect);
    CallbackManager::addFlagWatcher(&OBDDecoder::ecu_online_flag,setEcuStatustoOnline);
    CallbackManager::addFlagWatcher(&OBDDecoder::ecu_offline_flag,setEcuStatustoOffline);

    // eSIM Signals
    CallbackManager::addFlagWatcher(&SIM800LManager::esimConnectedFlag, setEsimStatustoConnected);
    CallbackManager::addFlagWatcher(&SIM800LManager::esimDisconnectedFlag, setEsimStatustoDisconnected);
    CallbackManager::addFlagWatcher(&SIM800LManager::esimWithoutSimFlag, setEsimStatustoWithoutSim);

    // Timers
    startOBDIIConnectionID = CallbackManager::addTimer(1000, startOBDIIConnection);
    ecuMessagesSenderID = CallbackManager::addTimer(400, messageSendingCallback);
    fuelCalculatorID = CallbackManager::addTimer(500, []() { FuelCalculator::getInstance().update(); });
    displayUpdateID  = CallbackManager::addTimer(500, []() { display.updateAll(); });
    cellularSenderID = CallbackManager::addTimer(CELLULAR_SEND_INTERVAL_MS, []() { SIM800LManager::sendNonVolatileData(); });

    // Timers control
    CallbackManager::pauseTimer(ecuMessagesSenderID);
    CallbackManager::pauseTimer(fuelCalculatorID);
    // NOTE: cellularSenderID is intentionally NOT paused here.
    // It used to be paused, resumed only by setEsimStatustoConnected()
    // (triggered by SIM800LManager::esimConnectedFlag). But esimConnectedFlag
    // is only set as a side effect of a successful connectGPRS(), which is
    // only ever called from inside sendHttpPost(), which is only ever called
    // from sendNonVolatileData() - i.e. from this very timer's callback.
    // With the timer paused from the start, it would never fire once, so
    // GPRS would never get a chance to connect, so the flag would never be
    // set, so the timer would stay paused forever: sendNonVolatileData()
    // was never actually being called. sendHttpPost()/connectGPRS() already
    // handle connecting on demand and no-op safely if hardware isn't ready,
    // so it's safe to just let this timer run from the start.
  }

void loop() {
  CallbackManager::update();
}