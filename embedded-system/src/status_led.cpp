// =============================================================================
//  status_led.cpp — ver status_led.h
// =============================================================================
#include "status_led.h"

#include <Arduino.h>

#include "config.h"

void StatusLed::begin(int pin, uint8_t brightness) {
  pin_ = pin;
  brightness_ = brightness;
  last_rgb_ = -1;
  update(LedState::BOOT, 0);
}

void StatusLed::write(uint8_t r, uint8_t g, uint8_t b) {
  const int32_t rgb = ((int32_t)r << 16) | ((int32_t)g << 8) | b;
  if (rgb == last_rgb_) return;
  last_rgb_ = rgb;
  const uint8_t rs = (uint8_t)((uint16_t)r * brightness_ / 255);
  const uint8_t gs = (uint8_t)((uint16_t)g * brightness_ / 255);
  const uint8_t bs = (uint8_t)((uint16_t)b * brightness_ / 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(pin_, rs, gs, bs);
#else
  neopixelWrite(pin_, rs, gs, bs);  // el núcleo ya envía en orden GRB
#endif
}

void StatusLed::update(LedState state, uint32_t now_ms) {
  state_ = state;
  switch (state) {
    case LedState::BOOT:
      write(0, 0, 255);
      break;
    case LedState::NORMAL:
      write(0, 255, 0);
      break;
    case LedState::EPISODE:
      write(255, 110, 0);  // ámbar
      break;
    case LedState::FAULT: {
      const bool on = ((now_ms / FAULT_BLINK_HALF_PERIOD_MS) % 2) == 0;
      if (on) write(255, 0, 0);
      else write(0, 0, 0);
      break;
    }
  }
}

const char* StatusLed::colorName() const {
  switch (state_) {
    case LedState::BOOT: return "blue";
    case LedState::NORMAL: return "green";
    case LedState::EPISODE: return "amber";
    case LedState::FAULT: return "red_blink";
  }
  return "?";
}
