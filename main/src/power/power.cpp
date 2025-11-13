/**
 * @file      power.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2022  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2022-09-16
 *
 */

#include <Arduino.h>
#include "pins.hpp"
#include "power/power.hpp"
#include "wifi/wifi.hpp"
#include "modem/modem.hpp"
#include "sdcard/sdcard.h"
#include <atomic>
#include "driver/rtc_io.h"

namespace power
{
    esp_sleep_wakeup_cause_t wakeup_reason;
    std::atomic<bool> isCharging;
    std::atomic<bool> isVbusInserted;
    std::atomic<bool> isBatteryLowLevel;
    std::atomic<bool> isBatteryCriticalLevel;
    std::atomic<bool> isPekeyShortPressed;

    uint64_t VbusInsertTimestamp = 0;
    uint64_t VbusRemovedTimestamp = 0;

    void loopPower(void *arg);

    XPowersPMU PMU;

    EventGroupHandle_t pmuIrqEvent;

    void IRAM_ATTR setFlag(void)
    {
        xEventGroupSetBits(pmuIrqEvent, 0b1);
    }

    bool setupPower()
    {
        isCharging = false;
        isVbusInserted = false;
        isBatteryLowLevel = false;
        isBatteryCriticalLevel = false;
        isPekeyShortPressed = false;
        if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA_POWER, I2C_SCL_POWER))
        {
            mqttLogger.println("ERROR: Init PMU failed!");
            return false;
        }
        pmuIrqEvent = xEventGroupCreate();
        // Set VSY off voltage as 2600mV, Adjustment range 2600mV ~ 3300mV
        PMU.setSysPowerDownVoltage(3100);

        // Turn off not use power channel
        PMU.disableDC2();
        PMU.disableDC4();
        PMU.disableDC5();

        PMU.disableALDO1();
        PMU.disableALDO2();
        PMU.disableALDO3();
        PMU.disableALDO4();
        PMU.disableBLDO1();
        PMU.disableBLDO2();

        PMU.disableCPUSLDO();
        PMU.disableDLDO1();
        PMU.disableDLDO2();

        PMU.enableBLDO1();

        // ESP32S3 Core VDD 3300mV Don't change, default turn on
        PMU.setDC1Voltage(3300);
        PMU.enableDC1();

        // External row needle, 1400~3700mV // external supply from pmu to header
        PMU.setDC5Voltage(3300);
        PMU.enableDC5();

        // Set the minimum common working voltage of the PMU VBUS input,
        // below this value will turn off the PMU
        PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V28);

        // Set the maximum current of the PMU VBUS input,
        // higher than this value will turn off the PMU
        PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1000MA);

        // Set VSY off voltage as 2600mV , Adjustment range 2600mV ~ 3300mV
        PMU.setSysPowerDownVoltage(2800);

        mqttLogger.printf("getID:0x%x\n", PMU.getChipID());

        Serial.println("=========================================");
        Serial.printf("DC1  : %s   Voltage: %u mV\n", PMU.isEnableDC1() ? "+" : "-", PMU.getDC1Voltage());
        Serial.printf("DC2  : %s   Voltage: %u mV\n", PMU.isEnableDC2() ? "+" : "-", PMU.getDC2Voltage());
        Serial.printf("DC3  : %s   Voltage: %u mV\n", PMU.isEnableDC3() ? "+" : "-", PMU.getDC3Voltage());
        Serial.printf("DC4  : %s   Voltage: %u mV\n", PMU.isEnableDC4() ? "+" : "-", PMU.getDC4Voltage());
        Serial.printf("DC5  : %s   Voltage: %u mV\n", PMU.isEnableDC5() ? "+" : "-", PMU.getDC5Voltage());
        Serial.println("=========================================");
        Serial.printf("ALDO1: %s   Voltage: %u mV\n", PMU.isEnableALDO1() ? "+" : "-", PMU.getALDO1Voltage());
        Serial.printf("ALDO2: %s   Voltage: %u mV\n", PMU.isEnableALDO2() ? "+" : "-", PMU.getALDO2Voltage());
        Serial.printf("ALDO3: %s   Voltage: %u mV\n", PMU.isEnableALDO3() ? "+" : "-", PMU.getALDO3Voltage());
        Serial.printf("ALDO4: %s   Voltage: %u mV\n", PMU.isEnableALDO4() ? "+" : "-", PMU.getALDO4Voltage());
        Serial.println("=========================================");
        Serial.printf("BLDO1: %s   Voltage: %u mV\n", PMU.isEnableBLDO1() ? "+" : "-", PMU.getBLDO1Voltage());
        Serial.printf("BLDO2: %s   Voltage: %u mV\n", PMU.isEnableBLDO2() ? "+" : "-", PMU.getBLDO2Voltage());

        PMU.clearIrqStatus();

        PMU.enableVbusVoltageMeasure();
        PMU.enableBattVoltageMeasure();
        PMU.enableSystemVoltageMeasure();
        PMU.disableTemperatureMeasure();

        // TS Pin detection must be disable, otherwise it cannot be charged
        PMU.disableTSPinMeasure();

        // Disable all interrupts
        PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
        // Clear all interrupt flags
        PMU.clearIrqStatus();
        // Enable the required interrupt function
        PMU.enableIRQ(
            XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |    // BATTERY
            XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |  // VBUS
            XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |     // POWER KEY
            XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ // CHARGE
            // XPOWERS_PKEY_NEGATIVE_IRQ | XPOWERS_PKEY_POSITIVE_IRQ   |   //POWER KEY
        );

        pinMode(PMU_INPUT_PIN, INPUT_PULLUP);
        attachInterrupt(PMU_INPUT_PIN, setFlag, FALLING);

        PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);

        // Set the precharge charging current
        PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_100MA);
        // Set constant current charge current limit
        PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_1000MA);
        // Set stop charging termination current
        PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_50MA);

        // Set charge cut-off voltage
        PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

        // Set the time of pressing the button to turn off
        PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
        PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
        // Get the default low pressure warning percentage setting
        // setLowBatWarnThreshold Range:  5% ~ 20%
        // The following data is obtained from actual testing , Please see the description below for the test method.
        // 20% ~= 3.7v
        // 1%  ~= 3.4V
        PMU.setLowBatWarnThreshold(15); // Set to trigger interrupt when reaching 5%
                                        // Get the low voltage warning percentage setting
        PMU.enableInternalDischarge();

        // setLowBatShutdownThreshold Range:  0% ~ 15%
        // The following data is obtained from actual testing , Please see the description below for the test method.
        // 15% ~= 3.6v
        // 1%  ~= 3.4V
        PMU.setLowBatShutdownThreshold(5); // Set to trigger interrupt when reaching 1%

        switch ((uint8_t)PMU.getBatteryPercent())
        {
        case 90 ... 100:
        {
            PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
            break;
        }
        case 50 ... 84:
        {
            PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
            break;
        }
        case 0 ... 49:
        {
            PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
            break;
        }
        default:
        {
        }
        }
        isVbusInserted = PMU.isVbusIn();
        if (isVbusInserted)
        {
            VbusInsertTimestamp = millis();
        }
        else
        {
            VbusRemovedTimestamp = millis();
        }
        isBatteryCriticalLevel = PMU.getBatteryPercent() <= 5;
        isBatteryLowLevel = PMU.getBatteryPercent() <= 15;
        xTaskCreate(loopPower, "power", 4096, NULL, 1, NULL);
        return true;
    }

    void handleInterrupt()
    {
        // Get PMU Interrupt Status Register
        uint32_t status = PMU.getIrqStatus();
        if (PMU.isVbusInsertIrq())
        {
            mqttLogger.println("isVbusInsert");
            isVbusInserted = true;
            VbusInsertTimestamp = millis();
        }
        if (PMU.isVbusRemoveIrq())
        {
            mqttLogger.println("isVbusRemove");
            isVbusInserted = false;
            VbusRemovedTimestamp = millis();
        }
        if (PMU.isBatInsertIrq())
        {
            mqttLogger.println("isBatInsert");
        }
        if (PMU.isBatRemoveIrq())
        {
            mqttLogger.println("isBatRemove");
        }
        if (PMU.isPekeyShortPressIrq())
        {
            mqttLogger.println("isPekeyShortPress");
            isPekeyShortPressed = true;
        }
        if (PMU.isPekeyLongPressIrq())
        {
            mqttLogger.println("isPekeyLongPress");
        }
        if (PMU.isBatChagerDoneIrq())
        {
            mqttLogger.println("isBatChagerDoneIrq");
        }
        if (PMU.isBatChagerStartIrq())
        {
            mqttLogger.println("isBatChagerStartIrq");
        }
        // When the set low-voltage battery percentage warning threshold is reached,
        // set the threshold through getLowBatWarnThreshold( 5% ~ 20% )
        if (PMU.isDropWarningLevel2Irq())
        {
            mqttLogger.println("isDropWarningLevel2Irq");
        }
        // When the set low-voltage battery percentage shutdown threshold is reached
        // set the threshold through setLowBatShutdownThreshold()
        if (PMU.isDropWarningLevel1Irq())
        {
            mqttLogger.println("isDropWarningLevel1Irq");
        }
        // For more interrupt sources, please check XPowersLib
        // Clear PMU Interrupt Status Register
        PMU.clearIrqStatus();
    }

    void loopPower(void *arg)
    {
        // mqttLogger.println("entering power loop");
        static const uint32_t lifSignTimeout = 5U * 1000U;
        uint64_t lifesign = millis();
        while (1)
        {
            auto event = xEventGroupWaitBits(pmuIrqEvent, 0b01, pdTRUE, pdTRUE, pdMS_TO_TICKS(10000));
            isVbusInserted = PMU.isVbusIn();
            isBatteryCriticalLevel = PMU.getBatteryPercent() <= 5;
            isBatteryLowLevel = PMU.getBatteryPercent() <= 15;
            if (event & 0b01)
            {
                handleInterrupt();
            }
            if (PMU.isBatteryConnect())
            {
                if (((millis() - lifSignTimeout) > lifesign) || (event & 0b01))
                {
                    mqttLogger.printf("CarLog/batt", " %d level:%d ,vol %d \n", PMU.isCharging() ? 1 : 0, PMU.getBatteryPercent(), PMU.getBattVoltage());
                    lifesign = millis();
                    if (PMU.isCharging())
                    {
                        switch ((uint8_t)PMU.getBatteryPercent())
                        {
                        case 91 ... 100:
                        {
                            PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
                            break;
                        }
                        case 50 ... 90:
                        {
                            PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
                            break;
                        }
                        case 0 ... 49:
                        {
                            PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
                            break;
                        }
                        default:
                        {
                        }
                        }
                    }
                    else
                    {
                        PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
                        delay(50);
                        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
                    }
                }
            }
            else
            {
                PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
                delay(1000);
                PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
                delay(1000);
            }
        }
    }

    esp_sleep_wakeup_cause_t getWakeupReason()
    {
        wakeup_reason = esp_sleep_get_wakeup_cause();

        switch (wakeup_reason)
        {
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            //!< In case of deep sleep, reset was not caused by exit from deep sleep
            mqttLogger.println("In case of deep sleep, reset was not caused by exit from deep sleep");
            break;
        case ESP_SLEEP_WAKEUP_ALL:
            //!< Not a wakeup cause: used to disable all wakeup sources with esp_sleep_disable_wakeup_source
            mqttLogger.println("Not a wakeup cause: used to disable all wakeup sources with esp_sleep_disable_wakeup_source");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            //!< Wakeup caused by external signal using RTC_IO
            mqttLogger.println("Wakeup caused by external signal using RTC_IO");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            //!< Wakeup caused by external signal using RTC_CNTL
            mqttLogger.println("Wakeup caused by external signal using RTC_CNTL");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            //!< Wakeup caused by timer
            mqttLogger.println("Wakeup caused by timer");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            //!< Wakeup caused by touchpad
            mqttLogger.println("Wakeup caused by touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            //!< Wakeup caused by ULP program
            mqttLogger.println("Wakeup caused by ULP program");
            break;
        case ESP_SLEEP_WAKEUP_GPIO:
            //!< Wakeup caused by GPIO (light sleep only)
            mqttLogger.println("Wakeup caused by GPIO (light sleep only)");
            break;
        case ESP_SLEEP_WAKEUP_UART:
            //!< Wakeup caused by UART (light sleep only)
            mqttLogger.println("Wakeup caused by UART (light sleep only)");
            break;
        case ESP_SLEEP_WAKEUP_WIFI:
            //!< Wakeup caused by WIFI (light sleep only)
            mqttLogger.println("Wakeup caused by WIFI (light sleep only)");
            break;
        case ESP_SLEEP_WAKEUP_COCPU:
            //!< Wakeup caused by COCPU int
            mqttLogger.println("Wakeup caused by COCPU int");
            break;
        case ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG:
            //!< Wakeup caused by COCPU crash
            mqttLogger.println("Wakeup caused by COCPU crash");
            break;
        case ESP_SLEEP_WAKEUP_BT:
            //!< Wakeup caused by BT (light sleep only)
            mqttLogger.println("Wakeup caused by BT (light sleep only)");
            break;
        default:
            mqttLogger.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
            break;
        }
        return wakeup_reason;
    }

    XPowersPMU &getPMU()
    {
        return PMU;
    }

    uint64_t getLastVbusInsertedTs()
    {
        return VbusInsertTimestamp;
    }
    uint64_t getLastVbusRemovedTs()
    {
        return VbusInsertTimestamp;
    }

    bool isBattCharging()
    {
        return isCharging;
    }

    bool isPowerVBUSOn()
    {
        return isVbusInserted;
    }

    bool isBatLowLevel()
    {
        return isBatteryLowLevel;
    }

    bool isBatCriticalLevel()
    {
        return isBatteryCriticalLevel;
    }

    bool iskeyShortPressed()
    {
        if (isPekeyShortPressed)
        {
            isPekeyShortPressed = false;
            return true;
        }
        return false;
    }

    void DeepSleepWith_IMU_PMU_Wake()
    {
        // Configure wakeup source: IMU interrupt pin
        mqttLogger.println("Entering deep sleep mode with IMU and PMU wakeup");
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
        detachInterrupt(PMU_INPUT_PIN);
        detachInterrupt(MOTION_INTRRUPT_PIN);
        rtc_gpio_hold_en(MOTION_INTRRUPT_PIN);
        rtc_gpio_hold_en(PMU_INPUT_PIN_);
        rtc_gpio_hold_en(CAM_PIN);
        uint64_t wakeup_mask = (1ULL << MOTION_INTRRUPT_PIN) | (1ULL << PMU_INPUT_PIN);
        String str = "Going to sleep now with mask  " + String(wakeup_mask, BIN);
        mqttLogger.printf(str.c_str());
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(wakeup_mask, ESP_EXT1_WAKEUP_ANY_LOW));
        delay(100);
        esp_deep_sleep_start();
    }

    void DeepSleepWith_PMU_Wake()
    {
        mqttLogger.println("Entering deep sleep mode with PMU wakeup");
        // Configure wakeup source: IMU interrupt pin
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
        detachInterrupt(PMU_INPUT_PIN);
        detachInterrupt(MOTION_INTRRUPT_PIN);
        rtc_gpio_hold_en(CAM_PIN);
        uint64_t wakeup_mask = (1ULL << PMU_INPUT_PIN);
        String str = "Going to sleep now with mask  " + String(wakeup_mask, BIN);
        mqttLogger.printf(str.c_str());
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(wakeup_mask, ESP_EXT1_WAKEUP_ANY_LOW));
        delay(100);
        esp_deep_sleep_start();
    }

    void DeepSleepWith_IMU_Timer_Wake(uint32_t ms)
    {
        mqttLogger.println("Entering deep sleep mode with timer PMU wakeup");
        // Configure wakeup source: IMU interrupt pin
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
        detachInterrupt(PMU_INPUT_PIN);
        detachInterrupt(MOTION_INTRRUPT_PIN);
        rtc_gpio_hold_en(MOTION_INTRRUPT_PIN);
        rtc_gpio_hold_en(PMU_INPUT_PIN_);
        rtc_gpio_hold_en(CAM_PIN);
        uint64_t wakeup_mask = (1ULL << MOTION_INTRRUPT_PIN) | (1ULL << PMU_INPUT_PIN);
        String str = "Going to sleep now with mask " + String(wakeup_mask, BIN) + String(ms);
        mqttLogger.printf(str.c_str());
        esp_sleep_enable_ext1_wakeup_io(wakeup_mask, ESP_EXT1_WAKEUP_ANY_LOW);
        esp_sleep_enable_timer_wakeup(1000 * ms);
        esp_deep_sleep_start();
    }

    void DeepSleepWith_Timer_Wake(uint32_t ms)
    {
        mqttLogger.println("Entering deep sleep mode with timer PMU wakeup");
        // Configure wakeup source: IMU interrupt pin
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
        detachInterrupt(PMU_INPUT_PIN);
        detachInterrupt(MOTION_INTRRUPT_PIN);
        rtc_gpio_hold_en(MOTION_INTRRUPT_PIN);
        rtc_gpio_hold_en(PMU_INPUT_PIN_);
        rtc_gpio_hold_en(CAM_PIN);
        uint64_t wakeup_mask = (1ULL << PMU_INPUT_PIN);
        String str = "Going to sleep now with mask " + String(wakeup_mask, BIN) + String(ms);
        mqttLogger.printf(str.c_str());
        esp_sleep_enable_ext1_wakeup_io(wakeup_mask, ESP_EXT1_WAKEUP_ANY_LOW);
        esp_sleep_enable_timer_wakeup(1000 * ms);
        esp_deep_sleep_start();
    }

    WakeUpReason Get_wake_reason()
    {
        static WakeUpReason wakeUpReason = WakeUpReason::UNKNOWN;
        if (wakeUpReason != WakeUpReason::UNKNOWN)
        {
            return wakeUpReason;
        }
        auto cause = esp_sleep_get_wakeup_cause();
        switch (cause)
        {
        case ESP_SLEEP_WAKEUP_EXT1:
        {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask & ((uint64_t)1 << MOTION_INTRRUPT_PIN))
            {
                Serial.println("Wakeup cause detected: MPU motion interrupt");
                wakeUpReason = WakeUpReason::MOTION;
                break;
            }
            if (wakeup_pin_mask & ((uint64_t)1 << PMU_INPUT_PIN))
            {
                Serial.println("Wakeup cause detected: PMU interrupt");
                wakeUpReason = WakeUpReason::START;
                break;
            }
            else
            {
                Serial.printf("Wakeup cause detected: 0x%llx\n", wakeup_pin_mask);
                break;
            }
        }
        case ESP_SLEEP_WAKEUP_TIMER:
        {
            wakeUpReason = WakeUpReason::TIMER;
            Serial.println("Wakeup cause detected: TIMER");
            break;
        }
        default:
        {
            wakeUpReason = WakeUpReason::UNKNOWN;
            break;
        }
        }
        return wakeUpReason;
    }
};

// CAM DVDD 1500~1800mV
// PMU.setALDO1Voltage(1800);
// PMU.enableALDO1();

// CAM DVDD 2500~2800mV
// PMU.setALDO2Voltage(2800);
// PMU.enableALDO2();

// CAM AVDD 2800~3000mV
// PMU.setALDO4Voltage(3000);
// PMU.enableALDO4();

// Modem 2700~3400mV VDD
// PMU.setDC3Voltage(3000);
// PMU.enableDC3();

// Modem GPS Power
// PMU.setBLDO2Voltage(3300);
// PMU.enableBLDO2();