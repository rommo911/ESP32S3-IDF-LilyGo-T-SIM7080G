
#pragma once
#define TINY_GSM_MODEM_SIM7080
#include <TinyGSM.h>
#include <TinyGsmClient.h>
#include <FS.h>

namespace modem
{
    class GPS_INFO
    {
    public:
        float lat;
        float lon;
        float speed;
        float alt;
        int vsat;
        int usat;
        float accuracy;
        int year;
        int month;
        int day;
        int hour;
        int min;
        int sec;
        uint32_t lastupdateTs;
    };

    bool initModem7080();
    bool startModem();
    bool SetGPS(bool enable);
    const GPS_INFO &getGPSInfo();
    bool SetGPRS(bool enable);
    bool shutdownModem();
    bool setRF(bool enable);
    bool sendFileToModem(File file, const char *destinationfilename = NULL);

    extern TinyGsm modem7080g;
    extern TinyGsmClient client;
    // extern TinyGsmClientSecure secureClient;
    // void loop_ppp();
    // void setup_ppp();

    String sendHttpGET(const char *url);
    String sendHttpsGET(const char *url);
    String SendhttpPOST(const char *url, const char *data);
    String SendHttpsPOST(const char *url, const char *data);

}