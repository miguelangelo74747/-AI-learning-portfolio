// =============================================================================
//  tremor_dsp.h — análisis espectral y lógica de episodios (C++ puro).
//
//  No depende de Arduino ni de FreeRTOS: se compila igual en el ESP32-S3 y en
//  el PC (pio test -e native). tools/tremor_ref.py implementa exactamente los
//  mismos pasos en numpy para la prueba 7 de la propuesta.
// =============================================================================
#pragma once

#include <stdint.h>

#include "config.h"

// En el ESP32 se usa la FFT optimizada de esp-dsp si el núcleo Arduino la trae
// (W06). En el PC, o con -DTREMOR_NO_ESP_DSP, se usa la FFT radix-2 propia.
// Ambas dan el mismo resultado salvo redondeo de float32.
#if defined(ESP_PLATFORM) && !defined(TREMOR_NO_ESP_DSP) && defined(__has_include)
#if __has_include(<esp_dsp.h>)
#define TREMOR_USE_ESP_DSP 1
#endif
#endif

namespace tremor {

// ---------------------------------------------------------------------------
//  Biquad paso bajo RBJ (Direct Form I) para la inclinación del antebrazo.
// ---------------------------------------------------------------------------
class BiquadLowPass {
 public:
  void design(float fc_hz, float q, float fs_hz);
  // Pone el estado en régimen permanente para una entrada constante x
  // (evita el transitorio de arranque y tras un hueco de muestras).
  void reset(float x);
  float process(float x);

 private:
  float b0_ = 0, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;
  float x1_ = 0, x2_ = 0, y1_ = 0, y2_ = 0;
};

// ---------------------------------------------------------------------------
//  Resultado de una ventana de 256 muestras.
// ---------------------------------------------------------------------------
struct WindowMetrics {
  float peak_hz = 0;        // pico más fuerte en 0.5–15 Hz (interpolado)
  float tremor_dps = 0;     // RMS de la banda 3.5–7 Hz, °/s
  float concentration = 0;  // fracción de potencia 0.5–15 Hz a ±1 Hz del pico
  float move_dps = 0;       // RMS de la banda 0.5–3 Hz, °/s
  float tilt_deg = 0;       // ángulo máximo entre la gravedad más nueva y cualquier anterior
  bool c1 = false, c2 = false, c3 = false, c4 = false;
  bool positive() const { return c1 && c2 && c3 && c4; }
};

// ---------------------------------------------------------------------------
//  Analizador espectral: ventana Hann, FFT radix-2 de 256 puntos por eje,
//  suma de los espectros de potencia de los tres ejes del giróscopo.
// ---------------------------------------------------------------------------
class SpectrumAnalyzer {
 public:
  static constexpr int N = WINDOW_SAMPLES;
  static constexpr int NBINS = N / 2 + 1;

  bool begin();  // precalcula ventana y tablas de la FFT

  // gyro[axis][i] en °/s, grav[axis][i] en g (ya filtrada), orden temporal
  // (índice 0 = muestra más antigua, N-1 = más nueva).
  void analyze(const float* const gyro[3], const float* const grav[3], WindowMetrics& out);

  // Espectro de potencia de la última ventana analizada (suma de ejes),
  // en (°/s)² por bin, escalado de modo que sum(bins) ≈ media cuadrática.
  const float* powerSpectrum() const { return power_; }
  float binHz() const { return SAMPLE_RATE_HZ / N; }
  static const char* fftBackend();

 private:
  void fft(float* re, float* im);  // in-place, N puntos
  int binLow(float hz) const;      // primer bin con f >= hz
  int binHigh(float hz) const;     // último bin con f <= hz

  float window_[N];
  float window_power_sum_ = 0;  // sum(w²)
  float cos_[N / 2], sin_[N / 2];
  uint16_t bitrev_[N];
  float re_[N], im_[N];
  float power_[NBINS];
#if defined(TREMOR_USE_ESP_DSP)
  float cplx_[2 * N] __attribute__((aligned(16)));  // re, im intercalados
#endif
};

// ---------------------------------------------------------------------------
//  Máquina de estados de episodio: empieza tras TREMOR_MIN_WINDOWS ventanas
//  positivas seguidas y termina tras EPISODE_RELEASE_WINDOWS negativas seguidas.
// ---------------------------------------------------------------------------
struct EpisodeEvent {
  enum Type : uint8_t { NONE = 0, START, END } type = NONE;
  uint32_t id = 0;
  uint32_t first_pos_seq = 0;  // última muestra de la primera ventana positiva de la racha
  uint32_t seq = 0;            // muestra en la que se toma la decisión
  float mean_hz = 0;           // media de peak_hz en las ventanas positivas del episodio
  float mean_dps = 0;          // media de tremor_dps en las ventanas positivas
  float max_dps = 0;
  uint32_t positive_windows = 0;
  uint32_t total_windows = 0;
};

class EpisodeTracker {
 public:
  // valid = false para ventanas incompletas (cuentan como negativas).
  EpisodeEvent update(bool valid, const WindowMetrics& m, uint32_t window_end_seq);
  // Fuerza el fin del episodio (p. ej. hueco de muestras o fallo del sensor).
  EpisodeEvent forceEnd(uint32_t seq);
  bool active() const { return active_; }
  int positiveStreak() const { return pos_streak_; }
  int negativeStreak() const { return neg_streak_; }

 private:
  EpisodeEvent makeEvent(EpisodeEvent::Type t, uint32_t seq) const;

  bool active_ = false;
  int pos_streak_ = 0;
  int neg_streak_ = 0;
  uint32_t next_id_ = 1;
  uint32_t cur_id_ = 0;
  uint32_t streak_first_seq_ = 0;
  uint32_t first_pos_seq_ = 0;
  // Acumuladores de la racha positiva actual y del episodio en curso.
  double streak_sum_hz_ = 0, streak_sum_dps_ = 0;
  float streak_max_dps_ = 0;
  double sum_hz_ = 0, sum_dps_ = 0;
  float max_dps_ = 0;
  uint32_t n_pos_ = 0, n_total_ = 0;
};

}  // namespace tremor
