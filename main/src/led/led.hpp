
#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

namespace led
{
    // Modes supported by the driver
    enum class LedMode
    {
        Solid,
        Blink,
        Fade
    };
    Adafruit_NeoPixel &getStrip();
    // Initialize the FastLED driver and worker task
    void led_init();

    // Unified control API. Calling this will stop any ongoing action for
    // the specified led index and start the requested one.
    // - index: led index (0..NUM_LEDS-1)
    // - mode: Solid / Blink / Fade
    // - color: for Solid/Blink it's the main color; for Fade it's the start color
    // - on_time_ms: blink ON time in ms (used for Blink)
    // - off_time_ms: blink OFF time in ms (used for Blink)
    // - timeout_s: timeout in seconds (stop action and return LED to off). -1 = no timeout
    // - color_to: target color for Fade
    // - fade_cycle_duration_ms: duration of the Fade in ms
    // New separated APIs:
    // Set a steady solid color (immediate)
    void set_solid(uint8_t index, uint32_t color);

    // Start blinking: on_time_ms = milliseconds LED is on, off_time_ms = milliseconds LED is off.
    // timeout_s = seconds after which blinking stops and LED is turned off. -1 = no timeout.
    void start_blink(uint8_t index,
                     uint32_t color,
                     uint32_t tocolor = 0,
                     uint16_t on_time_ms = 350,
                     uint16_t off_time_ms = 350,
                     uint32_t timeout_s = 0);

    // Start a repeating fade cycle between `color` and `color_to`.
    // The full cycle (color -> color_to -> color) lasts `fade_cycle_duration_ms` milliseconds.
    // The cycle repeats until `timeout_s` seconds elapse (if >0) or forever when `timeout_s` is -1.
    void start_fade(uint8_t index,
                    uint32_t color,
                    uint32_t color_to = 0,
                    uint16_t fade_cycle_duration_ms = 650,
                    int32_t timeout_s = -1);



    // Stop any action for the specified index and turn the led off.
    void stop_led(uint8_t index);

    static inline uint32_t _rgb( uint8_t r, uint8_t g, uint8_t b)
    {
        return (uint32_t)((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }

    uint32_t batteryColor(float percent);

}
