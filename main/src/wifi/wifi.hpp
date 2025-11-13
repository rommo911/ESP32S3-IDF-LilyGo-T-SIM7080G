#pragma once 
#include "WiFi.h"
#include <MqttLogger.h>

extern MqttLogger mqttLogger;
extern PubSubClient mqttclient;

void StartWifi();
bool GetWifiOn();
void StopWifi();