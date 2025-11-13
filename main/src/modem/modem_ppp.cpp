#include <Arduino.h>
#include "PPP.h"
#define PPP_MODEM_APN ""
#define PPP_MODEM_PIN "5461" // or NULL
#include "NetworkClient.h"
#include "pins.hpp"
// WaveShare SIM7600 HW Flow Control
#define PPP_MODEM_RST BOARD_MODEM_PWR_PIN
#define PPP_MODEM_RST_LOW false // active HIGH
#define PPP_MODEM_RST_DELAY 1000
#define PPP_MODEM_TX BOARD_MODEM_TXD_PIN
#define PPP_MODEM_RX BOARD_MODEM_RXD_PIN
#define PPP_MODEM_RTS -1
#define PPP_MODEM_CTS -1
#define PPP_MODEM_FC ESP_MODEM_FLOW_CONTROL_NONE
#define PPP_MODEM_MODEL PPP_MODEM_SIM7070

namespace modem
{
    void onEvent(arduino_event_id_t event, arduino_event_info_t info)
    {
        switch (event)
        {
        case ARDUINO_EVENT_PPP_START:
            Serial.println("PPP Started");
            break;
        case ARDUINO_EVENT_PPP_CONNECTED:
            Serial.println("PPP Connected");
            break;
        case ARDUINO_EVENT_PPP_GOT_IP:
            Serial.println("PPP Got IP");
            break;
        case ARDUINO_EVENT_PPP_LOST_IP:
            Serial.println("PPP Lost IP");
            break;
        case ARDUINO_EVENT_PPP_DISCONNECTED:
            Serial.println("PPP Disconnected");
            break;
        case ARDUINO_EVENT_PPP_STOP:
            Serial.println("PPP Stopped");
            break;
        default:
            break;
        }
    }

    void testClient(const char *host, uint16_t port)
    {
        NetworkClient client;
        if (!client.connect(host, port))
        {
            Serial.println("Connection Failed");
            return;
        }
        client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
        while (client.connected() && !client.available())
            ;
        while (client.available())
        {
            client.read(); // Serial.write(client.read());
        }

        Serial.println("Connection Success");
        client.stop();
    }

    void setup_ppp()
    {
        // Listen for modem events
        Network.onEvent(onEvent);

        // Configure the modem
        PPP.setApn(PPP_MODEM_APN);
        PPP.setPin(PPP_MODEM_PIN);
        PPP.setResetPin(PPP_MODEM_RST, PPP_MODEM_RST_LOW, PPP_MODEM_RST_DELAY);
        PPP.setPins(PPP_MODEM_TX, PPP_MODEM_RX, PPP_MODEM_RTS, PPP_MODEM_CTS, PPP_MODEM_FC);

        Serial.println("Starting the modem. It might take a while!");
        PPP.begin(PPP_MODEM_MODEL);

        Serial.print("Manufacturer: ");
        Serial.println(PPP.cmd("AT+CGMI", 10000));
        Serial.print("Model: ");
        Serial.println(PPP.moduleName().c_str());
        Serial.print("IMEI: ");
        Serial.println(PPP.IMEI().c_str());
        PPP.mode(ESP_MODEM_MODE_CMUX); // Set command mode
        bool attached = PPP.attached();
        if (!attached)
        {
            int i = 0;
            unsigned int s = millis();
            Serial.print("Waiting to connect to network");
            while (!attached && ((++i) < 600))
            {
                //Serial.print(".");
                delay(500);
                attached = PPP.attached();
            }
            Serial.print((millis() - s) / 1000.0, 1);
            Serial.println("s");
            attached = PPP.attached();
        }

        Serial.print("Attached: ");
        Serial.println(attached);
        Serial.print("State: ");
        Serial.println(PPP.radioState());
        if (attached)
        {
            Serial.print("Operator: ");
            Serial.println(PPP.operatorName().c_str());
            Serial.print("IMSI: ");
            Serial.println(PPP.IMSI().c_str());
            Serial.print("RSSI: ");
            Serial.println(PPP.RSSI());
            int ber = PPP.BER();
            if (ber > 0)
            {
                Serial.print("BER: ");
                Serial.println(ber);
                Serial.print("NetMode: ");
                Serial.println(PPP.networkMode());
            }

            Serial.println("Switching to data mode...");
            PPP.mode(ESP_MODEM_MODE_CMUX); // Data and Command mixed mode
            if (!PPP.waitStatusBits(ESP_NETIF_CONNECTED_BIT, 1000))
            {
                Serial.println("Failed to connect to internet!");
            }
            else
            {
                Serial.println("Connected to internet!");
                Serial.print("IP: ");
                Serial.println(PPP.localIP());
                Serial.print("Gateway: ");
                Serial.println(PPP.gatewayIP());
                Serial.print("DNS: ");
                Serial.println(PPP.dnsIP(0));
                Serial.print("Subnet: ");
                Serial.println(PPP.subnetMask());
                testClient("google.com", 80);

            }
        }
        else
        {
            Serial.println("Failed to connect to network!");
        }
    }

    void loop_ppp()
    {
        if (PPP.connected())
        {
            testClient("google.com", 80);
        }
        delay(20000);
    }

}