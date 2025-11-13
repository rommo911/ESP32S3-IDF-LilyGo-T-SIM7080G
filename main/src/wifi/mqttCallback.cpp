
#include "wifi/wifi.hpp"
extern bool simulatedMotionTrigger ;
extern bool simulatedLowPowerTrigger ;
extern bool simulatedCriticalLowPowerTrigger ;

void MqttReceiveCallback(char *topic, byte *payload, unsigned int length)
{
    // Safely convert payload (which may not be null-terminated) into a String.
    static String mqttReceStr;

    const unsigned int MAX_PAYLOAD = 1024; // prevent excessive allocation
    unsigned int len = length;
    if (len > MAX_PAYLOAD)
    {
        len = MAX_PAYLOAD;
        mqttLogger.println("Warning: payload truncated due to size");
    }

    char *buf = (char *)malloc(len + 1);
    if (buf == NULL)
    {
        mqttLogger.println("Error: malloc failed in MqttReceiveCallback");
        return;
    }

    if (len > 0)
    {
        memcpy(buf, payload, len);
    }
    buf[len] = '\0'; // ensure null termination

    mqttReceStr = String(buf);
    free(buf);
    if (mqttReceStr.length() < 1)
    {
        mqttLogger.println("Warning: received empty payload");
        return;
    }
    if (mqttReceStr == "motion_trigger")
    {
        simulatedMotionTrigger = true;
    }
    if (mqttReceStr == "low_power_trigger")
    {
        simulatedLowPowerTrigger = true;
    }
    if (mqttReceStr == "critical_low_power_trigger")
    {
        simulatedCriticalLowPowerTrigger = true;
    }
    Serial.printf("got message on topic %s = %s \n", topic, mqttReceStr.c_str());
    // handle message arrived
}
