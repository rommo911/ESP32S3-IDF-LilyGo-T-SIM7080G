#include "wifi/wifi.hpp"
#include "ArduinoOTA.h"
#include "web_server/web.hpp"
#include "time.h"
#include "rtc/rtc.hpp"
#include "Preferences.h"
#include "power/power.hpp"
#include "sdcard/sdcard.h"
bool timeIsSynced = false;
bool wifiOn = false;
uint32_t last_ota_time = 0;
TaskHandle_t WifiTaskHandle = NULL;
WiFiClient espClient;
PubSubClient mqttclient(espClient);
String ssid("ssid");
String pass = ("pass");
String ap_ssid = ("ap_ssid");
String ap_pass = ("ap_pass");
String mqtt_server = ("mqtt_server");
uint16_t mqtt_port = 1833;
String mqtt_user = ("mqtt_user");
String mqtt_pass = ("mqtt_pass");
String mqtt_log_topic = ("mqtt_topic");
String mqtt_cmd_topic = ("mqtt_cmd_topic");
MqttLogger mqttLogger(mqttclient, mqtt_log_topic.c_str(), MqttLoggerMode::MqttAndSerial);

void setUpWifiOTA(void *arg);

void SetupArduinoOTA();

bool setupWifiSTA();

void setUpWifiAP();

bool GetWifiOn()
{
  return wifiOn;
}

void MqttReceiveCallback(char *topic, byte *payload, unsigned int length);

void StartWifi()
{
  if (wifiOn == false)
  {
    if (power::isPowerVBUSOn())
    {
      setCpuFrequencyMhz(160);
    }
    else
    {
      setCpuFrequencyMhz(80);
    }
    SetupArduinoOTA();
    setUpWifiOTA(NULL);
  }
  else
  {
    mqttLogger.println("WiFi OTA task already running");
  }
}

void StopWifi()
{
  wifiOn = false;
  delay(50);
  WiFi.disconnect(true);
  delay(50);
  WiFi.mode(WIFI_OFF);
  delay(50);
  // vTaskDelete(WifiTaskHandle);
  WifiTaskHandle = NULL;
  setCpuFrequencyMhz(80);
}

bool syncntpTime()
{
  // Configure Paris timezone and NTP
  const char *ntpServer = "fr.pool.ntp.org"; // French NTP pool for better accuracy
  const long gmtOffset_sec = 3600;           // Paris is UTC+1 (3600 seconds)
  const int daylightOffset_sec = 3600;       // +1 hour for summer time

  // Set timezone before configTime
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); // Set timezone rule for Paris
  tzset();

  // Init and get the time with retry
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "pool.ntp.org", "time.nist.gov");

  // Wait for time to be set
  time_t now = time(nullptr);
  int retry = 0;
  while (now < 24 * 3600 && retry < 30)
  {
    Serial.println("Waiting for NTP time sync...");
    delay(250);
    now = time(nullptr);
    retry++;
  }

  if (now > 24 * 3600)
  {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
      char strftime_buf[64];
      strftime(strftime_buf, sizeof(strftime_buf), "%A, %B %d %Y %H:%M:%S %Z", &timeinfo);
      Serial.printf("NTP Synchronized. Current Paris time: %s\n", strftime_buf);
      // rtc::setRtcTimeDateFromSystemTime();
      timeIsSynced = true;
      return true;
    }
    else
    {
      Serial.println("Failed to obtain time");
    }
  }
  timeIsSynced = false;
  return false;
}

void loopWifiStation(void *arg)
{
  uint8_t mqttReconnectCounter = 0;
  while (wifiOn == true)
  {
    delay(10);
    if (WiFi.status() != WL_CONNECTED)
    {
      delay(250);
      continue;
    }
    mqttReconnectCounter = 0;
    while (!mqttclient.connected() && mqttReconnectCounter++ < 3)
    {
      Serial.println("Attempting MQTT connection...");
      // Attempt to connect
      if (mqttclient.connect("ESP32Tsim7080Logger", mqtt_user.c_str(), mqtt_pass.c_str()))
      {
        // as we have a connection here, this will be the first message published to the mqtt server
        mqttLogger.println("connected");
        mqttclient.subscribe(mqtt_cmd_topic.c_str(), 1);
      }
      else
      {
        mqttLogger.printf("failed, rc=%d \n", mqttclient.state());
        // Wait before retrying
        delay(500);
      }
    }
    mqttclient.loop();
    ArduinoOTA.handle();
    fs::GetmyWebServer().run();
  }
  mqttclient.disconnect();
  ArduinoOTA.end();
  fs::GetmyWebServer().stop();
  sdcard::shutdownSdcard();
  Serial.println("loopWifiStation thread exit");
  WifiTaskHandle = NULL;
  vTaskDelete(NULL);
}

void loopWifiAP(void *arg)
{
  while (wifiOn == true)
  {
    ArduinoOTA.handle();
    fs::GetmyWebServer().run();
    delay(10);
  }
  ArduinoOTA.end();
  fs::GetmyWebServer().stop();
  Serial.println("loopWifiAP thread exit");
  WifiTaskHandle = NULL;
  vTaskDelete(NULL);
}

void setUpWifiOTA(void *arg)
{
  Preferences pref;
  pref.begin("wifi", true);
  ssid = pref.getString("ssid", "Livebox-EF90");
  pass = pref.getString("pass", "11112222");
  ap_ssid = pref.getString("ap_ssid");
  ap_pass = pref.getString("ap_pass");
  mqtt_server = pref.getString("mqtt_server");
  mqtt_port = pref.getUInt("mqtt_port");
  mqtt_user = pref.getString("mqtt_user");
  mqtt_pass = pref.getString("mqtt_pass");
  mqtt_log_topic = pref.getString("mqtt_topic");
  mqttLogger.setTopic(mqtt_log_topic.c_str());
  mqtt_cmd_topic = pref.getString("mqtt_cmd_topic");
  pref.end();
  wifiOn = true;
  if (setupWifiSTA())
  {
    mqttLogger.println("WiFi STA setup complete ");
    mqttLogger.println("OTA Ready");
    String IP = String("IP address:") + WiFi.localIP().toString();
    xTaskCreate(loopWifiStation, "WiFiSTA", 10000, NULL, 2, &WifiTaskHandle);
    mqttLogger.println("WiFi STA end setup ");
  }
  else
  {
    mqttLogger.println("WiFi STA setup failed ");
    mqttLogger.println("WiFi AP setup starting ");
    setUpWifiAP();
    xTaskCreate(loopWifiAP, "WiFiAP", 10000, NULL, 2, &WifiTaskHandle);
    mqttLogger.println("WiFi AP task ending ");
  }
}

void SetupArduinoOTA()
{
  ArduinoOTA
      .onStart([]()
               {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "firmware.......";
      } else {  // U_SPIFFS
        type = "filesystem.......";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  {
      if (millis() - last_ota_time > 1000) {
        Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      } })
      .onError([](ota_error_t error)
               {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      } });
  ArduinoOTA.setHostname("ESP32_Tsim7080GRami");
}

bool setupWifiSTA()
{

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  uint32_t counter = 0;
  Serial.print("connecting to wifi .");
  while (WiFi.status() != WL_CONNECTED && counter++ < 50)
  {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect(true);
    return false;
  }
  syncntpTime();
  mqttclient.setCallback(MqttReceiveCallback);
  mqttclient.setServer(mqtt_server.c_str(), mqtt_port);
  mqttclient.connect("ESP32Tsim7080", mqtt_user.c_str(), mqtt_pass.c_str());
  delay(500);
  mqttclient.subscribe(mqtt_cmd_topic.c_str(), 1);
  Serial.print("WiFi connected IP address: ");
  Serial.println(WiFi.localIP());
  ArduinoOTA.begin();
  fs::fs_server_setup();
  return true;
}

void setUpWifiAP()
{
  WiFi.mode(WIFI_AP);
  uint32_t counter = 0;
  Serial.print("creating wifi .");
  WiFi.softAP(ap_ssid, ap_pass);
  fs::fs_server_setup();
  ArduinoOTA.begin();
}
