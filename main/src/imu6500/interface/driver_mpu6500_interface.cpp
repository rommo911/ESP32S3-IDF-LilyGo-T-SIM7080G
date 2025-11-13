/*
 * MPU6500 interface implementation for Arduino (ESP32) using Wire1
 * Uses pins from pins.hpp for SDA/SCL
 */
#include "imu6500/interface/driver_mpu6500_interface.h"
#include <Wire.h>
#include "pins.hpp"
#include <Arduino.h>
#include <stdarg.h>

extern "C"
{
    uint8_t mpu6500_interface_iic_init(void)
    {
        // Initialize Wire1 with pins from pins.hpp. Use 400kHz if supported.
        bool ok = Wire1.begin((int)I2C_SDA_PIN, (int)I2C_SCL_PIN, 400000);
        return ok ? 0 : 1;
    }

    uint8_t mpu6500_interface_iic_deinit(void)
    {
        Wire1.end();
        return 0;
    }

    uint8_t mpu6500_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
    {
        if (buf == NULL || len == 0)
        {
            // Serial.println("buffer NULL or len 0");
            return 1;
        }
        // driver uses 8-bit i2c address values (e.g. 0xD0), convert to 7-bit
        uint8_t dev = addr >> 1;

        Wire1.beginTransmission(dev);
        Wire1.write(reg);
        uint8_t err = Wire1.endTransmission(false); // send restart
        if (err != 0)
        {
            Serial.println("error transaction");
            return 1;
        }

        uint16_t got = 0;
        uint16_t toRequest = len;
        // requestFrom can return fewer bytes; read in loop if necessary
        uint16_t r = Wire1.requestFrom((int)dev, (int)toRequest);
        if (r != toRequest)
        {
            // try to read whatever arrived
            while (Wire1.available() && got < len)
            {
                buf[got++] = Wire1.read();
            }
            return (got == len) ? 0 : 1;
        }

        while (Wire1.available() && got < len)
        {
            buf[got++] = Wire1.read();
        }
        return (got == len) ? 0 : 1;
    }

    uint8_t mpu6500_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
    {
        if (buf == NULL && len != 0)
        {
            return 1;
        }

        uint8_t dev = addr >> 1;

        Wire1.beginTransmission(dev);
        Wire1.write(reg);
        if (len > 0)
        {
            Wire1.write(buf, (size_t)len);
        }
        uint8_t err = Wire1.endTransmission();
        return (err == 0) ? 0 : 1;
    }

    uint8_t mpu6500_interface_spi_init(void)
    {
        // SPI not implemented in this board-specific interface
        return 1;
    }

    uint8_t mpu6500_interface_spi_deinit(void)
    {
        return 1;
    }

    uint8_t mpu6500_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
    {
        (void)reg;
        (void)buf;
        (void)len;
        return 1;
    }

    uint8_t mpu6500_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
    {
        (void)reg;
        (void)buf;
        (void)len;
        return 1;
    }

    void mpu6500_interface_delay_ms(uint32_t ms)
    {
        delay((unsigned long)ms);
    }
    char mpuPrintbuf[512];

    void mpu6500_interface_debug_print(const char *const fmt, ...)
    {
        memset(mpuPrintbuf, 0, sizeof(mpuPrintbuf));
        va_list args;
        va_start(args, fmt);
        vsnprintf(mpuPrintbuf, sizeof(mpuPrintbuf), fmt, args);
        va_end(args);
        Serial.print(mpuPrintbuf);
    }

} // extern "C"
