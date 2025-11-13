#pragma once

namespace rtc
{
    bool isTimeSynced();
    bool ReadFromRTCDS1302();
    bool setRtcTimeDateFromSystemTime();
}