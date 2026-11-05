// =============================================================================
//  status_led.h — LED RGB (WS2812) de la placa como salida del dispositivo.
//
//   verde fijo        -> monitorizando, sin episodio
//   ámbar fijo        -> episodio de temblor en curso
//   rojo parpadeando  -> fallo del sensor (2 Hz: 250 ms on / 250 ms off)
//   azul fijo         -> arrancando (antes de la primera lectura correcta)
// =============================================================================
#pragma once

#include <stdint.h>

enum class LedState : uint8_t { BOOT, NORMAL, EPISODE, FAULT };

class StatusLed {
 public:
  void begin(int pin, uint8_t brightness);
  // Llamar a menudo (cada ≤ 20 ms): gestiona el parpadeo y solo escribe en el
  // WS2812 cuando el color cambia.
  void update(LedState state, uint32_t now_ms);
  const char* colorName() const;

 private:
  void write(uint8_t r, uint8_t g, uint8_t b);

  int pin_ = -1;
  uint8_t brightness_ = 40;
  int32_t last_rgb_ = -1;
  LedState state_ = LedState::BOOT;
};
