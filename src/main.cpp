#include <Arduino.h>
#include "CallbackManager.h"
#include "OBDManager.h"
#include "config.h"
#include "DebugSerial.h"
#include "OBDDecoder.h"
#include "VehicleData.h"
#include "FuelCalculator.h"
#include "DisplayManager.h"

DisplayManager display;

uint32_t startOBDIIConnectionID = 0;
uint32_t ecuMessagesSenderID = 0;
uint32_t fuelCalculatorID = 0;
uint32_t displayUpdateID = 0;

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
    break;
  case 2:
    OBDManager::addCommandToQueue(PID_COOLANT_TEMP_STR); // Coolant Temp
    break;
  case 3:
    OBDManager::addCommandToQueue(PID_ENGINE_LOAD_STR); // Engine Load
    break;
  case 4:
    //OBDManager::addCommandToQueue(PID_TIMING_ADVANCE_STR); // Timing Advance
    break;
  case 5:
    //OBDManager::addCommandToQueue(PID_THROTTLE_POSITION_STR); // Throttle Position
    break;
  case 6:
    //OBDManager::addCommandToQueue(PID_CONTROL_MODULE_VOLTAGE_STR); // Control Module Voltage
    break;
  case 7:
    OBDManager::addCommandToQueue(PID_LONG_TERM_FUEL_TRIM_STR); // Long Term Fuel Trim
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

void setup() {
    DebugSerial::begin();

    // Initialize persistent data
    VehicleData::getInstance().loadPersistentData();

    // Initialize fuel consumption calculator (loads K from NVS)
    FuelCalculator::getInstance().begin();

    // Initialize LCD (renders "Connecting OBDII..." until status changes).
    display.begin(0x27, 20, 4);

    // Class initialization
    OBDManager::setRawMessageCallback(OBDDecoder::decode);

    // Signals
    CallbackManager::addFlagWatcher(&OBDManager::obdConnectedFlag,setObdStatustoConnected);
    CallbackManager::addFlagWatcher(&OBDManager::obdDisconnectedFlag,setObdStatustoOffline);
    CallbackManager::addFlagWatcher(&OBDManager::obdConnectionAttemptFlag,setObdStatustoTryingToConnect);
    CallbackManager::addFlagWatcher(&OBDDecoder::ecu_online_flag,setEcuStatustoOnline);
    CallbackManager::addFlagWatcher(&OBDDecoder::ecu_offline_flag,setEcuStatustoOffline);

    // Timers
    startOBDIIConnectionID = CallbackManager::addTimer(1000, startOBDIIConnection);
    ecuMessagesSenderID = CallbackManager::addTimer(400, messageSendingCallback);
    fuelCalculatorID = CallbackManager::addTimer(500, []() { FuelCalculator::getInstance().update(); });
    displayUpdateID  = CallbackManager::addTimer(500, []() { display.updateAll(); });

    // Timers control
    CallbackManager::pauseTimer(ecuMessagesSenderID);
  }

void loop() {
  CallbackManager::update();
}