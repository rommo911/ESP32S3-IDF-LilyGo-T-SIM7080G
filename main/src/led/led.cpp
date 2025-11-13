#include "led.hpp"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "pins.hpp"
#include "portmacro.h"
#include <atomic>
#include <condition_variable>
#include <mutex>

// Unified, thread-safe per-led controller.

namespace led {

#define NUM_LEDS 2

std::mutex fast_led_mtx;

std::condition_variable cv;

uint32_t leds[NUM_LEDS] = {};
EventGroupHandle_t ledgroup;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, PIXEL_LED_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel &getStrip() { return strip; }

// Per-led runtime state
struct LedState {
  LedMode mode = LedMode::Solid;
  uint32_t color =
      0; // for Solid / Blink -> main color; for Fade -> start color
  uint32_t to_color = 0;         // fade target
  uint32_t on_time_ms = 250;     // blink ON time
  uint32_t off_time_ms = 250;    // blink OFF time
  int32_t timeout_ms = -1;       // timeout in milliseconds (-1 = no timeout)
  unsigned long last_toggle = 0; // last toggle time (ms)
  bool blink_on = false;         // current blink output state
  unsigned long start_time = 0;  // action start time (ms)
  int32_t duration_ms = -1;      // fade duration in ms
  bool active = false;           // whether an action is active
};

LedState states[NUM_LEDS];

TaskHandle_t workerTask = NULL;

static inline bool valid_index(uint8_t idx) { return idx < NUM_LEDS; }

// Linear interpolation helper for
static uint32_t lerp(const uint32_t &a, const uint32_t &b, float t) {
  /*uint8_t r = (uint8_t)((1.0f - t) * a.r + t * b.r);
  uint8_t g = (uint8_t)((1.0f - t) * a.g + t * b.g);
  uint8_t bl = (uint8_t)((1.0f - t) * a.b + t * b.b);*/
  return 0;
}

void loop_led(void *arg) {
  (void)arg;
  const TickType_t delayTicks = pdMS_TO_TICKS(50);
  Serial.println("entering loop_led");
  while (1) {
    bool any_change = false;
    xEventGroupWaitBits(ledgroup, 1, pdTRUE, pdTRUE, pdMS_TO_TICKS(30));
    auto now = millis();

    {
      std::lock_guard<std::mutex> lk(fast_led_mtx);
      for (uint8_t i = 0; i < NUM_LEDS; ++i) {
        auto &s = states[i];

        // If a timeout was configured and has expired, ensure LED is off and
        // stop the action.
        if (s.timeout_ms > 0 && (long)(now - s.start_time) >= s.timeout_ms) {
          if (leds[i] != 0) {
            leds[i] = 0;
            any_change = true;
          }
          s.mode = LedMode::Solid;
          s.color = 0;
          s.active = false;
          continue;
        }

        if (!s.active && s.mode == LedMode::Solid) {
          if (leds[i] != s.color) {
            leds[i] = s.color;
            any_change = true;
          }
          continue;
        }

        switch (s.mode) {
        case LedMode::Solid:
          if (leds[i] != s.color) {
            leds[i] = s.color;
            any_change = true;
          }
          s.active = false;
          break;

        case LedMode::Blink:
          if (s.last_toggle == 0) {
            // first tick: start visible
            s.last_toggle = now;
            s.blink_on = true;
            leds[i] = s.color;
            any_change = true;
          } else if ((unsigned long)(now - s.last_toggle) >=
                     (s.blink_on ? s.on_time_ms : s.off_time_ms)) {
            s.last_toggle = now;
            s.blink_on = !s.blink_on;
            leds[i] = s.blink_on ? s.color : s.to_color;
            any_change = true;
          }
          break;

        case LedMode::Fade:
          if (s.duration_ms <= 0) {
            // zero-duration -> instantly set to target
            leds[i] = s.to_color;
            any_change = true;
            s.mode = LedMode::Solid;
            s.color = s.to_color;
            s.active = false;
          } else {
            unsigned long elapsed = now - s.start_time;
            // If a timeout was set it was already checked at top-of-loop and
            // would have turned the LED off. Here we implement a repeating
            // "ping-pong" cycle: pos = fraction in [0,1) across the full cycle.
            // Use triangle waveform so t goes 0->1->0 over one cycle, then
            // repeat.
            unsigned long cycle = (unsigned long)s.duration_ms;
            unsigned long pos_ms = elapsed % cycle;
            float pos = (float)pos_ms / (float)cycle;  // [0,1)
            float t = 1.0f - fabsf(2.0f * pos - 1.0f); // triangle: 0->1->0
            auto val = lerp(s.color, s.to_color, t);
            if (val != leds[i]) {
              leds[i] = val;
              any_change = true;
            }
          }
          break;
        }
      }
    }

    if (any_change) {
      for (uint8_t i = 0; i < NUM_LEDS; ++i) {
        strip.setPixelColor(i, (leds[i]));
      }
      strip.show();
    }
    vTaskDelay(delayTicks);
  }
  vTaskDelete(NULL);
}

void led_init() {
  strip.begin();
  strip.setBrightness(255);
  // initialize LEDs to off
  for (uint8_t i = 0; i < NUM_LEDS; ++i) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
  ledgroup = xEventGroupCreate();
  xTaskCreate(loop_led, "led_worker", 4096, NULL, 1, &workerTask);
  Serial.printf("fast_led_init 5 \n");
}

// New separated implementations
void set_solid(uint8_t index, uint32_t color) {
  if (!valid_index(index))
    return;
  std::lock_guard<std::mutex> lk(fast_led_mtx);
  auto &s = states[index];
  s.mode = LedMode::Solid;
  s.color = color;
  s.to_color = 0;
  s.duration_ms = -1;
  s.timeout_ms = -1;
  s.last_toggle = 0;
  s.blink_on = false;
  s.active = false;
  leds[index] = color;
  strip.setPixelColor(index, (leds[index]));
  strip.show();
}

void start_blink(uint8_t index, uint32_t color, uint32_t tocolor,
                 uint16_t on_time_ms, uint16_t off_time_ms,
                 uint32_t timeout_ms) {
  if (!valid_index(index))
    return;
  std::lock_guard<std::mutex> lk(fast_led_mtx);
  led::LedState newstate;
  newstate.mode = LedMode::Blink;
  newstate.color = color;
  newstate.to_color = tocolor;
  newstate.on_time_ms = (on_time_ms == 0) ? 1 : on_time_ms;
  newstate.off_time_ms = (off_time_ms == 0) ? 1 : off_time_ms;
  newstate.duration_ms = -1;
  newstate.start_time = millis();
  newstate.last_toggle = 0;
  newstate.blink_on = false; // worker will switch on first tick
  newstate.active = true;
  newstate.timeout_ms = (timeout_ms > 50) ? (timeout_ms) : -1;
  auto &s = states[index];
  s = newstate;
  xEventGroupSetBits(ledgroup, 1);
}

void start_fade(uint8_t index, uint32_t color, uint32_t color_to,
                uint16_t fade_cycle_duration_ms, int32_t timeout_s) {
  if (!valid_index(index))
    return;
  std::lock_guard<std::mutex> lk(fast_led_mtx);
  auto &s = states[index];
  s.mode = LedMode::Fade;
  s.color = color;       // start color
  s.to_color = color_to; // target color
  s.duration_ms = (int32_t)fade_cycle_duration_ms;
  s.start_time = millis();
  s.last_toggle = 0;
  s.blink_on = false;
  s.active = true;
  s.timeout_ms = (timeout_s > 0) ? (timeout_s * 1000) : -1;
  xEventGroupSetBits(ledgroup, 1);
}

void stop_led(uint8_t index) {
  if (!valid_index(index))
    return;
  std::lock_guard<std::mutex> lk(fast_led_mtx);
  auto &s = states[index];
  s.active = false;
  s.mode = LedMode::Solid;
  s.color = 0;
  s.to_color = 0;
  s.timeout_ms = -1;
  s.last_toggle = 0;
  leds[index] = 0;
  strip.setPixelColor(index, 0);
  strip.show();
}
uint32_t batteryColor(float percent) {
  if (percent >= 90) {
    return strip.Color(0, 50, 0);
  }
  if (percent <= 10) {
    return strip.Color(50, 0, 0);
  }
  // map both red and green to 0..50 and make them complementary so
  // r + g == 50 (max). This keeps a smooth red->green gradient while
  // ensuring the sum never exceeds 50.
  uint8_t r = (uint8_t)(((uint16_t)(100 - percent) * 50) / 100);
  uint8_t g = (uint8_t)(((uint16_t)percent * 50) / 100);
  return strip.Color(r, g, 0);
}

} // namespace led