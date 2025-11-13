#include "modem.hpp"

namespace modem
{
    const String mqtt_server0 = "mqtt.example.com";
    const int mqtt_port0 = 1883;
    bool MqttIsConnect()
    {
        modem7080g.sendAT("+SMSTATE?");
        if (modem7080g.waitResponse("+SMSTATE: "))
        {
            String res = modem7080g.stream.readStringUntil('\r');
            return res.toInt();
        }
        return false;
    }

    void writeCaFiles(int index, const char *filename, const char *data,
                      size_t lenght)
    {
        modem7080g.sendAT("+CFSTERM");
        modem7080g.waitResponse();

        modem7080g.sendAT("+CFSINIT");
        if (modem7080g.waitResponse() != 1)
        {
            Serial.println("INITFS FAILED");
            return;
        }
        // AT+CFSWFILE=<index>,<filename>,<mode>,<filesize>,<input time>
        // <index>
        //      Directory of AP filesystem:
        //      0 "/custapp/" 1 "/fota/" 2 "/datatx/" 3 "/customer/"
        // <mode>
        //      0 If the file already existed, write the data at the beginning of the
        //      file. 1 If the file already existed, add the data at the end o
        // <file size>
        //      File size should be less than 10240 bytes. <input time> Millisecond,
        //      should send file during this period or you can’t send file when
        //      timeout. The value should be less
        // <input time> Millisecond, should send file during this period or you can’t
        // send file when timeout. The value should be less than 10000 ms.

        size_t payloadLenght = lenght;
        size_t totalSize = payloadLenght;
        size_t alardyWrite = 0;

        while (totalSize > 0)
        {
            size_t writeSize = totalSize > 10000 ? 10000 : totalSize;

            modem7080g.sendAT("+CFSWFILE=", index, ",", "\"", filename, "\"", ",",
                              !(totalSize == payloadLenght), ",", writeSize, ",", 10000);
            modem7080g.waitResponse(30000UL, "DOWNLOAD");
        REWRITE:
            modem7080g.stream.write(data + alardyWrite, writeSize);
            if (modem7080g.waitResponse(30000UL) == 1)
            {
                alardyWrite += writeSize;
                totalSize -= writeSize;
                Serial.printf("Writing:%d overage:%d\n", writeSize, totalSize);
            }
            else
            {
                Serial.println("Write failed!");
                delay(1000);
                goto REWRITE;
            }
        }

        Serial.println("Wirte done!!!");

        modem7080g.sendAT("+CFSTERM");
        if (modem7080g.waitResponse() != 1)
        {
            Serial.println("CFSTERM FAILED");
            return;
        }
    }
    bool MqttSetCertificate(const char *cert_pem,
                            const char *client_cert_pem = NULL,
                            const char *client_key_pem = NULL)
    {
        modem7080g.sendAT("+CFSINIT");
        modem7080g.waitResponse();

        if (cert_pem)
        {
            modem7080g.sendAT("+CFSWFILE=3,\"ca.crt\",0,", strlen(cert_pem), ",", 10000UL);
            if (modem7080g.waitResponse(10000UL, "DOWNLOAD") == 1)
            {
                modem7080g.stream.write(cert_pem);
            }
            if (modem7080g.waitResponse(5000UL) != 1)
            {
                DBG("Write ca_cert pem failed!");
                return false;
            }
        }

        if (client_cert_pem)
        {
            modem7080g.sendAT("+CFSWFILE=3,\"cert.pem\",0,", strlen(client_cert_pem), ",", 10000UL);
            if (modem7080g.waitResponse(10000UL, "DOWNLOAD") == 1)
            {
                modem7080g.stream.write(client_cert_pem);
            }
            if (modem7080g.waitResponse(5000) != 1)
            {
                DBG("Write cert pem failed!");
                return false;
            }
        }

        if (client_key_pem)
        {
            modem7080g.sendAT("+CFSWFILE=3,\"key_cert.pem\",0,", strlen(client_key_pem), ",", 10000UL);
            if (modem7080g.waitResponse(10000UL, "DOWNLOAD") == 1)
            {
                modem7080g.stream.write(client_key_pem);
            }
            if (modem7080g.waitResponse(5000) != 1)
            {
                DBG("Write key_cert failed!");
                return false;
            }
        }

        if (cert_pem)
        {
            // <ssltype>
            // 1 QAPI_NET_SSL_CERTIFICATE_E
            // 2 QAPI_NET_SSL_CA_LIST_E
            // 3 QAPI_NET_SSL_PSK_TABLE_E
            // AT+CSSLCFG="CONVERT",2,"ca_cert.pem"
            modem7080g.sendAT("+CSSLCFG=\"CONVERT\",2,\"ca.crt\"");
            if (modem7080g.waitResponse(5000UL) != 1)
            {
                DBG("convert ca_cert pem failed!");
                return false;
            }
        }

        if (client_cert_pem && client_key_pem)
        {
            modem7080g.sendAT("+CSSLCFG=\"CONVERT\",1,\"cert.pem\",\"key_cert.pem\"");
            if (modem7080g.waitResponse(5000) != 1)
            {
                DBG("convert client_cert_pem&client_key_pem pem failed!");
                return false;
            }

            //! AT+SMSSL=<index>,<ca list>,<cert name> , depes AT+CSSLCFG
            //! <index>SSL status, range: 0-6
            //!     0 Not support SSL
            //!     1-6 Corresponding to AT+CSSLCFG command parameter <ctindex>
            //! <ca list>CA_LIST file name, length 20 byte
            //! <cert name>CERT_NAME file name, length 20 byte
            modem7080g.sendAT("+SMSSL=1,\"ca_cert.pem\",\"cert.pem\"");
            modem7080g.waitResponse(3000);
        }

        modem7080g.sendAT("+CFSTERM");
        modem7080g.waitResponse();

        return true;
    }

    bool prepareMqtt()
    {
        // setCertificate(ca_cert, client_cert, client_cert_key);
        modem7080g.sendAT("+CCLK?");
        if (modem7080g.waitResponse(30000) == false)
        {
            Serial.println("Get time failed!");
            return false;
        }
        // If it is already connected, disconnect it first
        modem7080g.sendAT("+SMDISC");
        modem7080g.waitResponse();
        char buffer[1024];
        snprintf(buffer, 1024, "+SMCONF=\"URL\",\"%s\",%d", mqtt_server0.c_str(), mqtt_port0);
        modem7080g.sendAT(buffer);
        if (modem7080g.waitResponse() != 1)
        {
            Serial.println("Set URL failed!");
            return false;
        }

        snprintf(buffer, 1024, "+SMCONF=\"KEEPTIME\",60");
        modem7080g.sendAT(buffer);
        if (modem7080g.waitResponse() != 1)
        {
            Serial.println("Set keep time failed!");
            return false;
        }

        snprintf(buffer, 1024, "+SMCONF=\"CLEANSS\",1");
        modem7080g.sendAT(buffer);
        if (modem7080g.waitResponse() != 1)
        {
            return false;
        }
        const char *clientID = "your_client_id"; // Replace with your client ID
        const char *username = "your_username";   // Replace with your username
        const char *password = "your_password";   // Replace with your password
        snprintf(buffer, 1024, "+SMCONF=\"CLIENTID\",\"%s\"", clientID);
        modem7080g.sendAT(buffer);
        if (modem7080g.waitResponse() != 1)
        {
            return false;
        }

        snprintf(buffer, 1024, "+SMCONF=\"USERNAME\",\"%s\"", username);
        modem7080g.sendAT(buffer);
        if (modem7080g.waitResponse() != 1)
        {
            return false;
        }

        snprintf(buffer, 1024, "+SMCONF=\"PASSWORD\",\"%s\"", password);
        modem7080g.sendAT(buffer);
        if (modem7080g.waitResponse() != 1)
        {
            return false;
        }
        modem7080g.sendAT("+CSSLCFG=\"SSLVERSION\",0,3");
        if (!modem7080g.waitResponse())
        {
            Serial.println("Set SSL version failed!");
            return false;
        }

        modem7080g.sendAT("+CSSLCFG=\"CONVERT\",2,\"server-ca.crt\"");
        if (modem7080g.waitResponse() != 1)
        {
            Serial.println("Convert server-ca.crt failed!");
            return false;
        }
        modem7080g.sendAT("+SMSSL=1,\"server-ca.crt\",\"\"");
        if (modem7080g.waitResponse() != 1)
        {
            Serial.println("Convert ca failed!");
            return false;
        }

        // todo : fix me . Can I connect normally?

        int8_t ret;
        do
        {

            modem7080g.sendAT("+SMCONN");
            ret = modem7080g.waitResponse(60000UL);
            if (ret != 1)
            {
                Serial.println("Connect failed, retry connect ...");
                delay(1000);
            }

        } while (ret != 1);

        Serial.println("MQTT Client connected!");
        return true;
    }

    bool sendMqttMessage(const char *topic, const char *payload)
    {
        // Publish topic, below is an example, you need to create your own topic
        String pub_topic = "$aws/things/device_connected";

        // Publish payload (JSON format), below is an example, you need to create your own payload
        String pub_payload = "{create your own payload}";

        char buffer[1024];
        // AT+SMPUB=<topic>,<content length>,<qos>,<retain><CR>message is entered Quit edit mode if message length equals to <content length>
        snprintf(buffer, 1024, "+SMPUB=\"%s\",%d,0,0", pub_topic.c_str(), pub_payload.length()); // ! qos must be set to 0 since AWS IOT Core does not support QoS 1 SIM7080G

        modem7080g.sendAT(buffer);
        if (modem7080g.waitResponse(">") == 1)
        {
            modem7080g.stream.write(pub_payload.c_str(), pub_payload.length());
            Serial.println("");
            Serial.println(".......................................");
            Serial.println("Publishing below topic and payload now: ");
            Serial.println(pub_topic.c_str());
            Serial.println(pub_payload);

            if (modem7080g.waitResponse(3000))
            {
                Serial.println("Send Packet success!");
                return true;
            }
            else
            {
                Serial.println("Send Packet failed!");
                return false;
            }
        }
        else
        {
            Serial.println("Send Packet failed!");
            return false;
        }
    }

    bool mqttSubscribe(const char *topic)
    {
        char buffer[1024];
        snprintf(buffer, 1024, "+SMSUB=\"%s\",0", topic);
        modem7080g.sendAT(buffer);
        if (modem7080g.waitResponse(1000, "OK") == 1)
        {
            String result = modem7080g.stream.readStringUntil('\r');
            Serial.println();
            Serial.println(".......................................");
            Serial.println("Received message from mqtt: " + result);
        }
        else
        {
            Serial.println("Failed to subscribe message");
            return false;
        }
        return true;
    }

    String getMqttMessage()
    {
        String data = "";
        modem7080g.sendAT("+SMSTATE?");
        if (modem7080g.waitResponse("+SMSTATE:") == 1)
        {
            data = modem7080g.stream.readStringUntil('\r');
            Serial.println("Get message: " + data);
            return data;
        }
        return data;
    }
}