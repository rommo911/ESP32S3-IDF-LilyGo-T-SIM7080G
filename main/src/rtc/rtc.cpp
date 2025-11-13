#include "rtc.hpp"
#include "Ds1302.h"
#include "pins.hpp"
namespace rtc
{
    bool timeisSynced = false;
    const gpio_num_t RTC_PIN_ENA = GPIO_NUM_7;
    Ds1302 rtc(RTC_PIN_ENA, I2C_SCL_PIN, I2C_SDA_PIN);
    const static char *WeekDays[] =
        {
            "Monday",
            "Tuesday",
            "Wednesday",
            "Thursday",
            "Friday",
            "Saturday",
            "Sunday"};

    bool ReadFromRTCDS1302()
    {
        rtc.init();
        // test if clock is halted and set a date-time (see example 2) to start it
        if (rtc.isHalted())
        {
            Serial.println("RTC is halted. Setting time...");
            timeisSynced = false;
            return false;
        }
        Ds1302::DateTime dt;
        rtc.getDateTime(&dt);
        Serial.print(F("RTC time: "));
        Serial.print(dt.year + 2000); // Adjust for the year offset
        Serial.print(F("-"));
        Serial.print(dt.month);
        Serial.print(F("-"));
        Serial.print(dt.day);
        Serial.print(F(" "));
        Serial.print(dt.hour);
        Serial.print(F(":"));
        Serial.print(dt.minute);
        Serial.print(F(":"));
        // set system time from the Ds1302
        //  Set ESP32 system time from RTC
        struct tm timeinfo = {};
        timeinfo.tm_year = dt.year + 100; // 2000 -> 1900 offset
        timeinfo.tm_mon = dt.month - 1;   // 0-based in struct tm
        timeinfo.tm_mday = dt.day;
        timeinfo.tm_hour = dt.hour;
        timeinfo.tm_min = dt.minute;
        timeinfo.tm_sec = dt.second;

        time_t t = mktime(&timeinfo);
        struct timeval now = {.tv_sec = t, .tv_usec = 0};
        settimeofday(&now, nullptr);
        timeisSynced = true;
        return true;
    }

    bool setRtcTimeDateFromSystemTime()
    {
        rtc.init();
        // test if clock is halted and set a date-time (see example 2) to start it
        if (rtc.isHalted())
        {
            rtc.start();
        }
        time_t now = time(nullptr);
        struct tm *timeinfo = localtime(&now);

        if (!timeinfo)
        {
            Serial.println(F("Failed to get local time"));
            return false;
        }
        Ds1302::DateTime dt;
        dt.year = timeinfo->tm_year - 100; // store offset from 2000
        dt.month = timeinfo->tm_mon + 1;
        dt.day = timeinfo->tm_mday;
        dt.hour = timeinfo->tm_hour;
        dt.minute = timeinfo->tm_min;
        dt.second = timeinfo->tm_sec;
        rtc.setDateTime(&dt);
        Serial.println(F("RTC updated from system time."));
        timeisSynced = true ;
        return true;
    }
    
    bool isTimeSynced()
    {
        return timeisSynced;
    }

}