#ifndef MAIN_H_
#define MAIN_H_
#include <Arduino.h>

void turnOnCamera();
void turnOffCamera();
bool getCamIsON();

uint64_t getLastCamOnTs();

uint64_t getLastCamOffTs();

void loadTimingPref();

uint32_t getWifiTimeout();

uint32_t getNoMotionTimeout();

uint32_t getSecureModeTimeout();

uint64_t getSnapShotTime();

#endif // MAIN_H_