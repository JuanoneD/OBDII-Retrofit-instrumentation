#include <Arduino.h>
#include "CallbackManager.h"
#include "OBDManager.h"
#include "config.h"

OBDIISTATUS obdiiStatus = OBDIISTATUS::DISCONNECTED;
ECUSTATUS ecustatus = ECUSTATUS::OFFLINE;

void setObdStatustoConnected()
{
  if (obdiiStatus == OBDIISTATUS::CONNECTED) return;
  obdiiStatus = OBDIISTATUS::CONNECTED;
  Serial.println("OBDII Connected!");
}

void setObdStatustoOffline()
{
  obdiiStatus = OBDIISTATUS::DISCONNECTED;
  Serial.println("OBDII Disconnected!");
}

void startOBDIIConnection()
{
  if(obdiiStatus == OBDIISTATUS::DISCONNECTED)
    OBDManager::scanAndConnect();
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Hello, world!");

    OBDManager::setDebugSerial(&Serial);

    // Signals
    CallbackManager::addFlagWatcher(&OBDManager::obdConnectedFlag,setObdStatustoConnected);
    CallbackManager::addFlagWatcher(&OBDManager::obdDisconnectedFlag,setObdStatustoOffline);

    // Timers
    CallbackManager::addTimer((OBDII_SCAN_TIME_SEC + 1) * 1000UL, startOBDIIConnection);

}

void loop() {
  // put your main code here, to run repeatedly:
  CallbackManager::update();
}