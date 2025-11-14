#include "pins.hpp"
#include "Preferences.h"
#include "wifi/wifi.hpp"

static bool CamisOn = false;
static uint64_t lastCamOnTs = 0;
static uint64_t lastCamOffTs = 0;


static uint32_t WifiTimeout = 1000U * 60U * 5U;  //
static uint32_t No_MotionTimeout = 10U * 1000U;  //
static uint32_t SecureModeTimeout = 10U * 1000U; //
static uint64_t SnapShotTime = 2 * 60U * 1000U; // 2 hours


void loadTimingPref()
{
    Preferences pref;
    pref.begin("timing", true);
    WifiTimeout = pref.getULong("wifitm", WifiTimeout);                  //
    No_MotionTimeout = pref.getULong("nomotiontm", No_MotionTimeout);    //
    SecureModeTimeout = pref.getULong("securmodetm", SecureModeTimeout); //
    SnapShotTime = pref.getULong64("SnapShotTime", SnapShotTime); //
    mqttLogger.printf(" timing wifitm %d , nomotiontm %d , securmodetm %d \n", WifiTimeout, No_MotionTimeout, SecureModeTimeout);
}


uint64_t getSnapShotTime()
{
    return SnapShotTime;
}

uint32_t getWifiTimeout()
{
    return WifiTimeout;
}

uint32_t getNoMotionTimeout()
{
    return No_MotionTimeout;
}

uint32_t getSecureModeTimeout()
{
    return SecureModeTimeout;
}

void turnOnCamera()
{
    if (CamisOn == false)
    {
        lastCamOnTs = millis();
        digitalWrite(CAM_PIN, HIGH);
        CamisOn = true;
    }
}
void turnOffCamera()
{
    if (CamisOn == true)
    {
        lastCamOffTs = millis();
        digitalWrite(CAM_PIN, LOW);
        CamisOn = false;
    }
}


bool getCamIsON()
{
    return CamisOn;
}
uint64_t getLastCamOnTs()
{
    return lastCamOnTs;
}

uint64_t getLastCamOffTs()
{
    return lastCamOffTs;
}
