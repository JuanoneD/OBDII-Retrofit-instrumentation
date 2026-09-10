#include <Arduino.h>
#include "CallbackManager.h"
#include "OBDManager.h"
#include "config.h"
#include "DebugSerial.h"

OBDIISTATUS obdiiStatus = OBDIISTATUS::DISCONNECTED;
ECUSTATUS ecustatus = ECUSTATUS::OFFLINE;
uint32_t startOBDIIConnectionID = 0;

void setObdStatustoConnected()
{
  if (obdiiStatus == OBDIISTATUS::CONNECTED) return;

  CallbackManager::pauseTimer(startOBDIIConnectionID);
  obdiiStatus = OBDIISTATUS::CONNECTED;
  DebugSerial::println("OBDII Connected!");
}

void setObdStatustoOffline()
{
  if (obdiiStatus == OBDIISTATUS::DISCONNECTED) return;

  obdiiStatus = OBDIISTATUS::DISCONNECTED;
  CallbackManager::resumeTimer(startOBDIIConnectionID);
  DebugSerial::println("OBDII Disconnected!");
}

void setObdStatustoTryingToConnect()
{
  obdiiStatus = OBDIISTATUS::TRYING_TO_CONNECT;
  DebugSerial::println("OBDII Trying to Connect...");
}

void startOBDIIConnection()
{
  if(obdiiStatus == OBDIISTATUS::DISCONNECTED)
    OBDManager::scanAndConnect();
}

void setup() {
    // put your setup code here, to run once:
    DebugSerial::begin();
    DebugSerial::println("Hello, world!");

    // Signals
    CallbackManager::addFlagWatcher(&OBDManager::obdConnectedFlag,setObdStatustoConnected);
    CallbackManager::addFlagWatcher(&OBDManager::obdDisconnectedFlag,setObdStatustoOffline);
    CallbackManager::addFlagWatcher(&OBDManager::obdConnectionAttemptFlag,setObdStatustoTryingToConnect);

    // Timers
    startOBDIIConnectionID = CallbackManager::addTimer(1000, startOBDIIConnection);

}

void loop() {
  // put your main code here, to run repeatedly:
  CallbackManager::update();
}