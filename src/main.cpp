#include <Arduino.h>
#include "CallbackManager.h"
#include "OBDManager.h"
#include "config.h"
#include "DebugSerial.h"
#include "OBDDecoder.h"

OBDIISTATUS obdiiStatus = OBDIISTATUS::DISCONNECTED;
ECUSTATUS ecustatus = ECUSTATUS::OFFLINE;

uint32_t startOBDIIConnectionID = 0;
uint32_t ecuMessagesSenderID = 0;

int messageIndex = 0;

void messageSendingCallback()
{
  if (obdiiStatus != OBDIISTATUS::CONNECTED) return;

  switch (messageIndex)
  {
  case 0:
    OBDManager::addCommandToQueue(PID_ENGINE_RPM_STR); // RPM
    break;
  case 1:
    OBDManager::addCommandToQueue(PID_VEHICLE_SPEED_STR); // Vehicle Speed
    break;
  case 2:
    //OBDManager::addCommandToQueue(PID_COOLANT_TEMP_STR); // Coolant Temp
    break;
  case 3:
    //OBDManager::addCommandToQueue(PID_ENGINE_LOAD_STR); // Engine Load
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
    //OBDManager::addCommandToQueue(PID_LONG_TERM_FUEL_TRIM_STR); // Long Term Fuel Trim
    break;
  default:
    messageIndex = -1; // Reset index to -1 so that it becomes 0 on the next increment
    break;
  }
  messageIndex++;
}

void setObdStatustoConnected()
{
  if (obdiiStatus == OBDIISTATUS::CONNECTED) return;

  CallbackManager::pauseTimer(startOBDIIConnectionID);
  obdiiStatus = OBDIISTATUS::CONNECTED;
  DebugSerial::println("OBDII Connected!");
  CallbackManager::resumeTimer(ecuMessagesSenderID);
}

void setEcuStatustoOnline()
{
  if (ecustatus == ECUSTATUS::ONLINE) return;

  ecustatus = ECUSTATUS::ONLINE;
  DebugSerial::println("ECU Online!");
}

void setEcuStatustoOffline()
{
  if (ecustatus == ECUSTATUS::OFFLINE) return;

  ecustatus = ECUSTATUS::OFFLINE;
  DebugSerial::println("ECU Offline!");
}


void setObdStatustoOffline()
{
  if (obdiiStatus == OBDIISTATUS::DISCONNECTED) return;

  obdiiStatus = OBDIISTATUS::DISCONNECTED;
  DebugSerial::println("OBDII Disconnected!");

  CallbackManager::resumeTimer(startOBDIIConnectionID);
  CallbackManager::pauseTimer(ecuMessagesSenderID);

  OBDManager::clearCommandQueue();
}

void setObdStatustoTryingToConnect()
{
  if(obdiiStatus == OBDIISTATUS::CONNECTED) return;

  obdiiStatus = OBDIISTATUS::TRYING_TO_CONNECT;
  DebugSerial::println("OBDII Trying to Connect...");
}

void startOBDIIConnection()
{
  if(obdiiStatus == OBDIISTATUS::DISCONNECTED)
    OBDManager::scanAndConnect();
}

void setup() {
    DebugSerial::begin();

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

    // Timers control
    CallbackManager::pauseTimer(ecuMessagesSenderID);
  }

void loop() {
  CallbackManager::update();
}