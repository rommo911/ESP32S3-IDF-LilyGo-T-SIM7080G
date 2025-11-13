/*
  MqttLogger - offer print() interface like Serial but by publishing to a given mqtt topic.
               Uses Serial as a fallback when no mqtt connection is available.

  Claus Denk
  https://androbi.com
*/

#ifndef MqttLogger_h
#define MqttLogger_h

#define MQTT_MAX_PACKET_SIZE 2048
#define MQTT_KEEPALIVE 10
#define MQTT_SOCKET_TIMEOUT 30

#include <PubSubClient.h>

enum MqttLoggerMode
{
    SerialOnly = 0,
    MqttOnly = 1,
    MqttAndSerial = 2,
};

class MqttLogger 
{
private:
    const char *topic = nullptr;
    PubSubClient *client = nullptr;
    MqttLoggerMode mode = MqttLoggerMode::MqttAndSerial;
    bool retained = false;

public:
    explicit MqttLogger(PubSubClient &client, const char *topic, MqttLoggerMode mode = MqttLoggerMode::MqttAndSerial, const bool retained = false);
    ~MqttLogger();

    void setClient(PubSubClient &client);
    void setTopic(const char *topic);
    void setMode(MqttLoggerMode mode);
    void setRetained(const boolean &retained);
    
    // formatted print helpers (default behavior: write into MqttLogger's buffer)
    size_t printf(const char *_topic, const char *format, ...); // printf that sets topic for that message
    size_t printf(const char *format, ...); // printf that sets topic for that message
    // simple topic-aware print/println
    size_t println(const char *_topic, const char *s);
    size_t println(const char *s);
};

#endif
