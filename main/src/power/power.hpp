/**
 * @file      power.h
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2022  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2022-09-16
 *
 */

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

namespace power
{
    enum WakeUpReason : uint8_t
    {
        MOTION = 0,
        START = 1,
        TIMER = 2,
        UNKNOWN
    };

    enum SleepCause : uint8_t
    {
        AFTER_MOTION = 0,
        AFTER_NO_MOTION = 1,
        SLEEP_SNAP_SHOT = 2,
        AFTER_ON = 4,
        NA
    };
    SleepCause getSleepCause();

    bool setupPower();
    esp_sleep_wakeup_cause_t getWakeupReason();
    WakeUpReason Get_wake_reason();

    XPowersPMU &getPMU();
    bool isBattCharging();
    bool isPowerVBUSOn();
    bool isBatLowLevel();
    bool isBatCriticalLevel();
    bool iskeyShortPressed();
    void DeepSleepWith_IMU_PMU_Timer_Wake(SleepCause cause, uint32_t ms = 0);
    void DeepSleepWith_PMU_Timer_Wake(SleepCause cause, uint32_t ms= 0);
    void DeepSleepWith_PMU_Wake();
    uint64_t getLastVbusInsertedTs();
    uint64_t getLastVbusRemovedTs();
};
