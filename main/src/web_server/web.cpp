#include "web_server/web.hpp"
#include "WiFi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" // The current version of this program
#include <Preferences.h>   // For NVS storage
#include "ArduinoJson.h"   // For JSON parsing
#include "LittleFS.h"
#include <Update.h>
#include "sdcard/sdcard.h"
#include "wifi/wifi.hpp"
#include "SD_MMC.h"
#include "imu6500/imu_DMP6.hpp"
#include "main.hpp"

namespace fs
{
  enum class FServerSource
  {
    LittleFS,
    SDcard
  };
  static FSWebServer myWebServer(LittleFS, 80, "CarFsServer");
  static FSWebServer myWebServerSDMMC(SD_MMC, 80, "CarSDServer");
  static FServerSource FSsource = FServerSource::SDcard;
  static void getSdcardInfo(fsInfo_t *fsInfo)
  {
    fsInfo->fsName = "SDCard";
    fsInfo->totalBytes = SD_MMC.totalBytes();
    fsInfo->usedBytes = SD_MMC.usedBytes();
  }

  static void getFsInfo(fsInfo_t *fsInfo)
  {
    fsInfo->fsName = "LittleFS";
    fsInfo->totalBytes = LittleFS.totalBytes();
    fsInfo->usedBytes = LittleFS.usedBytes();
  }
  FSWebServer &GetmyWebServer()
  {
    if (FSsource == FServerSource::SDcard)
    {
      return myWebServerSDMMC;
    }
    return myWebServer;
  }
  /* Helper function to generate success response page */
  static String generateSuccessPage(const String &message)
  {
    return "<html><head><meta http-equiv='refresh' content='3;url=/serverIndex'>"
           "<style>body{text-align:center;font-family:sans-serif;margin-top:50px;}</style></head>"
           "<body><h1>" +
           message + "</h1>"
                     "<button onclick=\"window.location.href='/serverIndex'\">Return to Main Page</button>"
                     "</body></html>";
  }

  static void handleCar()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    GetmyWebServer().sendHeader("Connection", "close");
    GetmyWebServer().send(200, "text/html", CarserverIndex);
  }

  static void handleTimeDate()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    GetmyWebServer().sendHeader("Connection", "close");
    GetmyWebServer().send(200, "text/html", timeDatePage);
  }

  static void handleSetTime()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    if (GetmyWebServer().hasArg("plain"))
    {
      String body = GetmyWebServer().arg("plain");
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, body);

      if (error)
      {
        Serial.println("Failed to parse JSON");
        GetmyWebServer().send(400, "text/html", generateSuccessPage("Invalid JSON format"));
        return;
      }

      String datetime = doc["datetime"];
      String timezone = doc["timezone"] | "CET-1CEST,M3.5.0,M10.5.0/3";

      setenv("TZ", timezone.c_str(), 1);
      tzset();

      struct tm timeinfo = {0};
      if (strptime(datetime.c_str(), "%Y-%m-%dT%H:%M", &timeinfo) != NULL)
      {
        timeinfo.tm_sec = 0;

        time_t t = mktime(&timeinfo);
        struct timeval tv = {.tv_sec = t, .tv_usec = 0};

        if (settimeofday(&tv, NULL) == 0)
        {
          time_t now = time(nullptr);
          char buffer[26];
          strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S %Z", localtime(&now));
          Serial.printf("Time set successfully to time: %s\n", buffer);

          GetmyWebServer().send(200, "text/html", generateSuccessPage("Time set successfully to: " + String(buffer)));
        }
        else
        {
          Serial.println("Failed to set system time");
          GetmyWebServer().send(500, "text/html", generateSuccessPage("Failed to set system time"));
        }
      }
      else
      {
        Serial.println("Failed to parse datetime string");
        GetmyWebServer().send(400, "text/html", generateSuccessPage("Invalid datetime format"));
      }
    }
    else
    {
      GetmyWebServer().send(400, "text/html", generateSuccessPage("No data received"));
    }
  }

  static void handleBleUUID()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    GetmyWebServer().sendHeader("Connection", "close");
    GetmyWebServer().send(200, "text/html", bleUUIDPage);
  }

  static void handleImuThresholds()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    GetmyWebServer().sendHeader("Connection", "close");
    GetmyWebServer().send(200, "text/html", imuThresholdsPage);
  }

  static void handleWifiSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    GetmyWebServer().sendHeader("Connection", "close");
    GetmyWebServer().send(200, "text/html", wifiSettingsPage);
  }

  static void handleGetWifiSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    Preferences pref;
    pref.begin("wifi", true);
    String ssid = pref.getString("ssid", "");
    String pass = pref.getString("pass", "");
    String ap_ssid = pref.getString("ap_ssid", "");
    String ap_pass = pref.getString("ap_pass", "");
    String mqtt_server = pref.getString("mqtt_server", "");
    uint32_t mqtt_port = pref.getUInt("mqtt_port", 0);
    String mqtt_user = pref.getString("mqtt_user", "");
    String mqtt_pass = pref.getString("mqtt_pass", "");
    String mqtt_topic = pref.getString("mqtt_topic", "");
    String mqtt_cmd_topic = pref.getString("mqtt_cmd_topic", "");
    pref.end();

    String json = "{";
    json += "\"ssid\":\"" + ssid + "\",";
    json += "\"pass\":\"" + pass + "\",";
    json += "\"ap_ssid\":\"" + ap_ssid + "\",";
    json += "\"ap_pass\":\"" + ap_pass + "\",";
    json += "\"mqtt_server\":\"" + mqtt_server + "\",";
    json += "\"mqtt_port\":" + String(mqtt_port) + ",";
    json += "\"mqtt_user\":\"" + mqtt_user + "\",";
    json += "\"mqtt_pass\":\"" + mqtt_pass + "\",";
    json += "\"mqtt_topic\":\"" + mqtt_topic + "\",";
    json += "\"mqtt_cmd_topic\":\"" + mqtt_cmd_topic + "\"";
    json += "}";

    GetmyWebServer().send(200, "application/json", json);
  }

  static void handleSetWifiSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    if (GetmyWebServer().hasArg("plain"))
    {
      String body = GetmyWebServer().arg("plain");
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, body);
      if (err)
      {
        Serial.println("Failed to parse JSON for WiFi settings");
        GetmyWebServer().send(400, "text/html", generateSuccessPage("Invalid JSON format"));
        return;
      }
      Preferences pref;
      pref.begin("wifi", false);
      if (doc["ssis"].is<String>())
        pref.putString("ssid", String((const char *)doc["ssid"]));
      if (doc["pass"].is<String>())
        pref.putString("pass", String((const char *)doc["pass"]));
      if (doc["ap_ssid"].is<String>())
        pref.putString("ap_ssid", String((const char *)doc["ap_ssid"]));
      if (doc["ap_pass"].is<String>())
        pref.putString("ap_pass", String((const char *)doc["ap_pass"]));
      if (doc["mqtt_server"].is<String>())
        pref.putString("mqtt_server", String((const char *)doc["mqtt_server"]));
      if (doc["mqtt_port"].is<String>())
        pref.putUInt("mqtt_port", (uint32_t)doc["mqtt_port"]);
      if (doc["mqtt_user"].is<String>())
        pref.putString("mqtt_user", String((const char *)doc["mqtt_user"]));
      if (doc["mqtt_pass"].is<String>())
        pref.putString("mqtt_pass", String((const char *)doc["mqtt_pass"]));
      if (doc["mqtt_topic"].is<String>())
        pref.putString("mqtt_topic", String((const char *)doc["mqtt_topic"]));
      if (doc["mqtt_cmd_topic"].is<String>())
        pref.putString("mqtt_cmd_topic", String((const char *)doc["mqtt_cmd_topic"]));
      pref.end();

      Serial.println("WiFi settings saved to NVS");
      GetmyWebServer().send(200, "text/html", generateSuccessPage("WiFi settings saved successfully!"));
    }
    else
    {
      GetmyWebServer().send(400, "text/html", generateSuccessPage("No data received"));
    }
  }

  static void handleGetImuThresholds()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    Preferences imuPref;
    imuPref.begin("imu", true);
    float gx = imuPref.getFloat("M_TH_GX", 0.005f);
    float gy = imuPref.getFloat("M_TH_GY", 0.005f);
    float gz = imuPref.getFloat("M_TH_GZ", 0.005f);
    float roll = imuPref.getFloat("M_TH_ROLL", 0.5f);
    float yaw = imuPref.getFloat("M_TH_YAW", 0.5f);
    float pitch = imuPref.getFloat("M_TH_PITCH", 0.5f);
    imuPref.end();

    String jsonResponse = "{";
    jsonResponse += "\"M_TH_GX\":" + String(gx, 4) + ",";
    jsonResponse += "\"M_TH_GY\":" + String(gy, 4) + ",";
    jsonResponse += "\"M_TH_GZ\":" + String(gz, 4) + ",";
    jsonResponse += "\"M_TH_ROLL\":" + String(roll, 2) + ",";
    jsonResponse += "\"M_TH_YAW\":" + String(yaw, 2) + ",";
    jsonResponse += "\"M_TH_PITCH\":" + String(pitch, 2) + ",";
    jsonResponse += "}";

    GetmyWebServer().send(200, "application/json", jsonResponse);
  }

  static void handleSetImuThresholds()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    if (GetmyWebServer().hasArg("plain"))
    {
      String body = GetmyWebServer().arg("plain");
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, body);
      if (error)
      {
        Serial.println("Failed to parse JSON for IMU thresholds");
        GetmyWebServer().send(400, "text/html", generateSuccessPage("Invalid JSON format"));
        return;
      }
      Preferences imuPref;
      imuPref.begin("imu", false);
      if (doc["M_TH_GX"].is<float>())
        imuPref.putFloat("M_TH_GX", (float)doc["M_TH_GX"]);
      if (doc["M_TH_GY"].is<float>())
        imuPref.putFloat("M_TH_GY", (float)doc["M_TH_GY"]);
      if (doc["M_TH_GZ"].is<float>())
        imuPref.putFloat("M_TH_GZ", (float)doc["M_TH_GZ"]);
      if (doc["M_TH_ROLL"].is<float>())
        imuPref.putFloat("M_TH_ROLL", (float)doc["M_TH_ROLL"]);
      if (doc["M_TH_YAW"].is<float>())
        imuPref.putFloat("M_TH_YAW", (float)doc["M_TH_YAW"]);
      if (doc["M_TH_PITCH"].is<float>())
        imuPref.putFloat("M_TH_PITCH", (float)doc["M_TH_PITCH"]);

      // preferences saved; live WOM threshold update is handled by the WOM settings page
      imuPref.end();
      Serial.println("IMU thresholds saved to NVS");
      GetmyWebServer().send(200, "text/html", generateSuccessPage("IMU thresholds saved successfully!"));
    }
    else
    {
      GetmyWebServer().send(400, "text/html", generateSuccessPage("No data received"));
    }
  }

  static void handleWomSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    GetmyWebServer().sendHeader("Connection", "close");
    GetmyWebServer().send(200, "text/html", womSettingsPage);
  }

  static void handleGetWomSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    Preferences imuPref;
    imuPref.begin("imu", true);
    float wom = imuPref.getFloat("WOM_THR", 15.0f);
    uint32_t lpf = imuPref.getUInt("WOM_LPF", (uint32_t)imu6500_dmp::get_wom_lpf());
    uint32_t rate = imuPref.getUInt("WOM_RATE", (uint32_t)imu6500_dmp::get_wom_acc_output_rate());
    bool acc_compare = imuPref.getBool("ACC_COMR", false);
    imuPref.end();

    String json = "{";
    json += "\"WOM_THR\":" + String(wom, 2) + ",";
    json += "\"WOM_LPF\":" + String(lpf) + ",";
    json += "\"ACC_COMR\":" + String(acc_compare) + ",";
    json += "\"WOM_RATE\":" + String(rate);
    json += "}";

    GetmyWebServer().send(200, "application/json", json);
  }

  static void handleSetWomSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    if (GetmyWebServer().hasArg("plain"))
    {
      String body = GetmyWebServer().arg("plain");
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, body);
      if (error)
      {
        GetmyWebServer().send(400, "text/html", generateSuccessPage("Invalid JSON"));
        return;
      }

      if (doc["WOM_THR"].is<float>())
      // Update threshold live
      {
        imu6500_dmp::SetWakeOnMotionThresh((float)doc["WOM_THR"]);
      }

      // LPF and RATE updates: call imu functions which also persist
      if (doc["WOM_LPF"].is<uint8_t>())
      {
        uint8_t v = doc["WOM_LPF"];
        if (v >= MPU6500_ACCELEROMETER_LOW_PASS_FILTER_0 && v <= MPU6500_ACCELEROMETER_LOW_PASS_FILTER_7)
          imu6500_dmp::set_wom_lpf((mpu6500_accelerometer_low_pass_filter_t)v);
      }
      if (doc["WOM_RATE"].is<uint8_t>())
      {
        uint8_t v = doc["WOM_RATE"];
        if (v >= MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_0P24 && v <= MPU6500_LOW_POWER_ACCEL_OUTPUT_RATE_500)
          imu6500_dmp::set_wom_acc_output_rate((mpu6500_low_power_accel_output_rate_t)v);
      }
      if (doc["ACC_COMR"].is<bool>())
      {
        bool v = doc["ACC_COMR"];
        imu6500_dmp::SetAccelCompare(v);
      }
      imu6500_dmp::restart(imu6500_dmp::WOM);

      Serial.println("WOM settings saved to NVS");
      GetmyWebServer().send(200, "text/html", generateSuccessPage("WOM settings saved successfully!"));
    }
    else
    {
      GetmyWebServer().send(400, "text/html", generateSuccessPage("No data received"));
    }
  }

  static void handleSetUUID()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    if (GetmyWebServer().hasArg("plain"))
    {
      String body = GetmyWebServer().arg("plain");
      JsonDocument doc;
      deserializeJson(doc, body);
      String uuid = doc["uuid"];
      Preferences preferences;
      preferences.begin("ble-settings", false);
      preferences.putString("ibeacon_uuid", uuid);
      preferences.end();
      Serial.printf("Saved UUID: %s\n", uuid.c_str());
      GetmyWebServer().send(200, "text/html", generateSuccessPage("UUID saved successfully!"));
    }
    else
    {
      GetmyWebServer().send(400, "text/html", generateSuccessPage("Failed to save UUID."));
    }
  }

  static void handleSetCredentials()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    if (GetmyWebServer().hasArg("plain"))
    {
      String body = GetmyWebServer().arg("plain");
      JsonDocument doc;
      deserializeJson(doc, body);
      String oldUser = doc["oldUser"];
      String oldPwd = doc["oldPwd"];
      String newUser = doc["newUser"];
      String newPwd = doc["newPwd"];
      Preferences preferences;
      preferences.begin("auth-settings", true);
      String storedUser = preferences.getString("username", "admin");
      String storedPwd = preferences.getString("password", "admin");
      preferences.end();

      if (oldUser == storedUser && oldPwd == storedPwd && newUser.length() > 2 && newPwd.length() >= 10)
      {
        preferences.begin("auth-settings", false);
        preferences.putString("username", newUser);
        preferences.putString("password", newPwd);
        preferences.end();

        GetmyWebServer().setAuthentication(newUser.c_str(), newPwd.c_str());

        Serial.printf("Updated credentials: User=%s\n", newUser.c_str());
        GetmyWebServer().send(200, "text/html", generateSuccessPage("Credentials updated successfully!"));
      }
      else
      {
        GetmyWebServer().send(401, "text/html", generateSuccessPage("Unauthorized: Old credentials are incorrect. or short new creditntials."));
        Serial.println("Unauthorized: Old credentials are incorrect or short new credentials.");
        Serial.printf("Current User: %s, Current Pwd: %s\n", storedUser.c_str(), storedPwd.c_str());
        Serial.printf("New User: %s, New Pwd: %s\n", newUser.c_str(), newPwd.c_str());
      }
    }
    else
    {
      GetmyWebServer().send(400, "text/html", generateSuccessPage("Failed to update credentials."));
      Serial.println("Failed to update credentials, no args.");
    }
  }

  static void handleChangeCredentials()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    GetmyWebServer().sendHeader("Connection", "close");
    GetmyWebServer().send(200, "text/html", changeCredentialsPage);
  }

  static void handleGetCurrentTime()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    char buffer[26];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    String currentTime = String(buffer);
    String jsonResponse = "{\"currentTime\": \"" + currentTime + "\"}";
    GetmyWebServer().send(200, "application/json", jsonResponse);
  }

  static void handleGetCurrentUUID()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      Serial.println("Authentication failed, redirecting to login page.");
      return GetmyWebServer().requestAuthentication();
    }
    Preferences preferences;
    preferences.begin("ble-settings", true);
    String uuid = preferences.getString("ibeacon_uuid", "00000000-0000-0000-0000-000000000000");
    preferences.end();
    String jsonResponse = "{\"uuid\": \"" + uuid + "\"}";
    GetmyWebServer().send(200, "application/json", jsonResponse);
  }

  static void handleTimingSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      return;
    }
    GetmyWebServer().sendHeader("Connection", "close");
    GetmyWebServer().send(200, "text/html", timingSettingsPage);
  }

  static void handleGetTimingSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      return;
    }
    Preferences pref;
    pref.begin("timing", true);
    uint32_t wifiTimeout = pref.getUInt("wifitm", 300000);           // 5 minutes default
    uint32_t noMotionTimeout = pref.getUInt("nomotiontm", 10000);    // 10 seconds default
    uint32_t secureModeTimeout = pref.getUInt("securmodetm", 10000); // 10 seconds default
    pref.end();

    String jsonResponse = "{";
    jsonResponse += "\"wifitm\":" + String(wifiTimeout) + ",";
    jsonResponse += "\"nomotiontm\":" + String(noMotionTimeout) + ",";
    jsonResponse += "\"securmodetm\":" + String(secureModeTimeout);
    jsonResponse += "}";

    GetmyWebServer().send(200, "application/json", jsonResponse);
  }

  static void handleSetTimingSettings()
  {
    if (!GetmyWebServer().authenticate_internal())
    {
      return;
    }

    if (GetmyWebServer().hasArg("plain"))
    {
      String json = GetmyWebServer().arg("plain");
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, json);
      if (error)
      {
        GetmyWebServer().send(400, "text/plain", "Invalid JSON");
        return;
      }

      Preferences pref;
      pref.begin("timing", false);
      if (doc["wifitm"].is<uint32_t>())
      {
        pref.putUInt("wifitm", doc["wifitm"].as<uint32_t>());
      }
      if (doc["nomotiontm"].is<uint32_t>())
      {
        pref.putUInt("nomotiontm", doc["nomotiontm"].as<uint32_t>());
      }
      if (doc["securmodetm"].is<uint32_t>())
      {
        pref.putUInt("securmodetm", doc["securmodetm"].as<uint32_t>());
      }
      pref.end();

      loadTimingPref(); // from main.h

      GetmyWebServer().send(200, "text/plain", "Settings updated successfully");
    }
    else
    {
      GetmyWebServer().send(400, "text/plain", "No data received");
    }
  }
  static void handleNotFound()
  {
    GetmyWebServer().send(404, "text/plain", "404: Not Found");
  }
  static const unsigned long loginTimeout = 5 * 60 * 1000; // 5 minutes in milliseconds

  bool fs_server_setup()
  {
    // FILESYSTEM INIT
    FSsource = FServerSource::SDcard;
    Serial.println("Initializing FS...");

    if (sdcard::setupSdcard() == false)
    {
      FSsource = FServerSource::LittleFS;
      Serial.println("Using littlefs as filesystem for web server.");
      if (!LittleFS.begin(false, "/littlefs", 10, "storage"))
      {
        Serial.println("ERROR on mounting filesystem.");
        return false;
      }
    }
    else
    {
      Serial.println("Using SD Card as filesystem for web server.");
    }
    auto &myServer = GetmyWebServer();
    myServer.enableFsCodeEditor(FSsource == FServerSource::LittleFS ? getFsInfo : getSdcardInfo);
    Preferences preferences;
    preferences.begin("auth-settings", false);
    String storedUser = preferences.getString("username", "admin");
    String storedPwd = preferences.getString("password", "admin");
    if (storedUser == "admin" || storedPwd == "admin")
    {
      preferences.putString("username", "admin");
      preferences.putString("password", "admin");
      Serial.println("No credentials found, using default: admin/admin");
    }
    else
    {
      Serial.printf("Stored credentials: User=%s, Pwd=%s\n", storedUser.c_str(), storedPwd.c_str());
    }
    preferences.end();

    myServer.setAuthentication(storedUser.c_str(), storedPwd.c_str());
    myServer.printFileList(Serial, "/", 3);
    myServer.on("/car", HTTP_GET, handleCar);
    /* Time and Date Page */
    myServer.on("/timeDate", HTTP_GET, handleTimeDate);
    /* Set Time */
    myServer.on("/setTime", HTTP_POST, handleSetTime);
    /* BLE iBeacon UUID Page */
    myServer.on("/bleUUID", HTTP_GET, handleBleUUID);
    /* Set BLE iBeacon UUID */
    myServer.on("/setUUID", HTTP_POST, handleSetUUID);
    /* Change Username and Password */
    myServer.on("/setCredentials", HTTP_POST, handleSetCredentials);
    /* Change Credentials Page */
    myServer.on("/changeCredentials", HTTP_GET, handleChangeCredentials);
    /* Get Current Time */
    myServer.on("/getCurrentTime", HTTP_GET, handleGetCurrentTime);
    myServer.on("/getCurrentUUID", HTTP_GET, handleGetCurrentUUID);
    /* IMU Thresholds Pages */
    myServer.on("/imuThresholds", HTTP_GET, handleImuThresholds);
    myServer.on("/getImuThresholds", HTTP_GET, handleGetImuThresholds);
    myServer.on("/setImuThresholds", HTTP_POST, handleSetImuThresholds);
    /* WOM Settings Pages */
    myServer.on("/womSettings", HTTP_GET, handleWomSettings);
    myServer.on("/getWomSettings", HTTP_GET, handleGetWomSettings);
    myServer.on("/setWomSettings", HTTP_POST, handleSetWomSettings);
    /* WiFi Settings Pages */
    myServer.on("/wifiSettings", HTTP_GET, handleWifiSettings);
    myServer.on("/getWifiSettings", HTTP_GET, handleGetWifiSettings);
    myServer.on("/setWifiSettings", HTTP_POST, handleSetWifiSettings);

    /* Timing Settings Pages */
    myServer.on("/timingSettings", HTTP_GET, handleTimingSettings);
    myServer.on("/getTimingSettings", HTTP_GET, handleGetTimingSettings);
    myServer.on("/setTimingSettings", HTTP_POST, handleSetTimingSettings);

    myServer.begin();
    return true;
  }
  /* Helper function to check if the session is still valid */
}