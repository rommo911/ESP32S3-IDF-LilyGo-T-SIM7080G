#include "modem/modem.hpp"
#include "wifi/wifi.hpp"
namespace modem
{
    GPS_INFO gpsInfo;

    const GPS_INFO &getGPSInfo()
    {
        return gpsInfo;
    }

    bool getLocation()
    {
        if (modem7080g.getGPS(&gpsInfo.lat, &gpsInfo.lon, &gpsInfo.speed, &gpsInfo.alt, &gpsInfo.vsat, &gpsInfo.usat, &gpsInfo.accuracy,
                              &gpsInfo.year, &gpsInfo.month, &gpsInfo.day, &gpsInfo.hour, &gpsInfo.min, &gpsInfo.sec))
        {
            Serial.println();
            Serial.print("lat:");
            Serial.print(String(gpsInfo.lat, 8));
            Serial.print("\t");
            Serial.print("lon:");
            Serial.print(String(gpsInfo.lon, 8));
            Serial.println();
            Serial.print("speed:");
            Serial.print(gpsInfo.speed);
            Serial.print("\t");
            Serial.print("alt:");
            Serial.print(gpsInfo.alt);
            Serial.println();
            Serial.print("year:");
            Serial.print(gpsInfo.year);
            Serial.print(" month:");
            Serial.print(gpsInfo.month);
            Serial.print(" day:");
            Serial.print(gpsInfo.day);
            Serial.print(" hour:");
            Serial.print(gpsInfo.hour);
            Serial.print(" min:");
            Serial.print(gpsInfo.min);
            Serial.print(" sec:");
            Serial.print(gpsInfo.sec);
            Serial.print(" vsat:");
            Serial.print(gpsInfo.vsat);
            Serial.print(" usat:");
            Serial.print(gpsInfo.usat);
            Serial.print(" accuracy:");
            Serial.print(gpsInfo.accuracy);
            Serial.println();
            gpsInfo.lastupdateTs = millis();
            mqttLogger.printf("sim7080gps", "lat:%.8f  | lon:%.8f  ", gpsInfo.lat, gpsInfo.lon);
            return true;
        }
        else
        {
            Serial.println("Get location failed...");
        }
        // Serial.println("Please check the GPS antenna!");
        return false;
    }

}