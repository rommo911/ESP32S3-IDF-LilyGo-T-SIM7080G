/**
 * @file      sdcard.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2022  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2022-09-23
 *
 */
#include "FS.h"
#include "SD_MMC.h"
#include "pins.hpp"
#include "wifi/wifi.hpp"
#include "power/power.hpp"
#include "FS.h"
#include <Update.h>

#define UPDATE_FILENAME "/firmware.bin"

namespace sdcard
{
    bool sdcardOK = false;
    bool shutdownSdcard()
    {
        SD_MMC.end();
        sdcardOK = false;
        return true;
    }

    bool setupSdcard()
    {
        if (sdcardOK)
        {
            return true;
        }

        SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA);
        // SD Card VDD 3300mV
        power::getPMU().setALDO3Voltage(3300);
        power::getPMU().enableALDO3();
        delay(200);
        if (!SD_MMC.begin("/sdcard", true, false, 20000))
        {
            mqttLogger.println("ERROR: SD Card Mount failed!");
            power::getPMU().disableALDO3();
            SD_MMC.end();
            return false;
        }
        uint8_t cardType = SD_MMC.cardType();

        if (cardType == CARD_NONE)
        {
            Serial.println("No SD_MMC card attached");
            power::getPMU().disableALDO3();
            SD_MMC.end();
            return false;
        }

        Serial.print("SD_MMC Card Type: ");
        if (cardType == CARD_MMC)
        {
            Serial.println("MMC");
        }
        else if (cardType == CARD_SD)
        {
            Serial.println("SDSC");
        }
        else if (cardType == CARD_SDHC)
        {
            Serial.println("SDHC");
        }
        else
        {
            Serial.println("UNKNOWN");
        }

        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        mqttLogger.printf("SD_MMC Card Size: %lluMB\n", cardSize);
        sdcardOK = true;
        return true;
    }

    // perform the actual update from a given stream
    bool performUpdate(Stream &updateSource, size_t updateSize)
    {
        if (Update.begin(updateSize))
        {
            size_t written = Update.writeStream(updateSource);
            if (written == updateSize)
            {
                Serial.println("Written : " + String(written) + " successfully");
            }
            else
            {
                Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
            }
            if (Update.end())
            {
                Serial.println("OTA via SD done!");
                if (Update.isFinished())
                {
                    Serial.println("Update successfully completed!");
                    return true;
                }
                else
                {
                    Serial.println("Update not finished? Something went wrong!");
                    return false;
                }
            }
            else
            {
                Serial.println("Error Occurred. Error #: " + String(Update.getError()));
                return false;
            }
        }
        else
        {
            Serial.println("Not enough space to begin OTA");
            return false;
        }
        return false;
    }

    // check given FS for valid UPDATE_FILENAME and perform update if available
    bool updateFromFS(fs::FS &fs)
    {
        File updateBin = fs.open(UPDATE_FILENAME);
        if (updateBin)
        {
            if (updateBin.isDirectory())
            {
                Serial.println("Error, " UPDATE_FILENAME " is not a file");
                updateBin.close();
                return false;
            }
            Serial.println("Update found");
            // print file details
            Serial.printf("file name %s\n", updateBin.name());
            Serial.printf("file size %d\n", updateBin.size());
            // Serial.printf("md5 %s\n", updateBin.md5)
            size_t updateSize = updateBin.size();
            // atleast 1 megabyte
            if (updateSize > (1024U * 1024U) && updateSize < (4U * 1024U * 1024U))
            {
                Serial.println("Try to start update");
                bool ok = performUpdate(updateBin, updateSize);
                updateBin.close();
                if (ok)
                {
                    // when finished remove the binary from sd card to indicate end of the process
                    fs.rename(UPDATE_FILENAME, UPDATE_FILENAME ".done");
                    return true;
                }
                else
                {
                    Serial.println("Update failed");
                    return false;
                }
            }
            else
            {
                Serial.println("Error, file is empty");
                return false;
            }
        }
        else
        {
            Serial.println("no " UPDATE_FILENAME " from sd root");
            return false;
        }
    }

    bool checkForupdatefromSD()
    {
        if (!sdcardOK)
        {
            return false;
        }
        return updateFromFS(SD_MMC);
    }
}