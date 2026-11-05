// =============================================================================
//  tremor_pipeline.h — lo que hace la tarea de procesado con cada muestra.
//
//  Pasos 3–5 de la propuesta (sección 3), sin Arduino ni FreeRTOS:
//   3. convertir a °/s y g, detectar números de muestra que faltan y mantener
//      las últimas 256 muestras (2.56 s) por eje;
//   4. cada 50 muestras (0.5 s) analizar la ventana (FFT, C1–C4);
//   5. actualizar el estado del episodio.
//
//  Las ventanas están alineadas con el número de muestra: se evalúan cuando
//  (seq + 1) % HOP_SAMPLES == 0, así numpy puede recomputarlas sabiendo solo
//  el "seq" final de cada línea W del log.
// =============================================================================
#pragma once

#include <stdint.h>

#include "config.h"
#include "tremor_dsp.h"

namespace tremor {

// Muestra cruda tal como la lee la tarea de muestreo (cuentas del ADC).
struct RawSample {
  uint32_t seq;   // número de tic del temporizador (0, 1, 2, ...)
  int64_t t_us;   // esp_timer_get_time() al empezar la lectura I2C
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct PipelineOutput {
  // Hueco en la numeración (muestras perdidas antes de esta).
  uint32_t gap_missing = 0;
  EpisodeEvent gap_event;  // END forzado si había un episodio activo

  // Ventana evaluada en esta muestra.
  bool window_evaluated = false;
  bool window_valid = false;  // false mientras la ventana no tenga 256 muestras seguidas
  uint32_t window_end_seq = 0;
  WindowMetrics metrics;
  EpisodeEvent event;
};

class TremorPipeline {
 public:
  bool begin();
  void push(const RawSample& s, PipelineOutput& out);

  bool episodeActive() const { return tracker_.active(); }
  int positiveStreak() const { return tracker_.positiveStreak(); }
  const SpectrumAnalyzer& analyzer() const { return analyzer_; }
  uint32_t missingTotal() const { return missing_total_; }

  static float gyroDps(int16_t raw) { return (float)raw / GYRO_LSB_PER_DPS; }
  static float accelG(int16_t raw) { return (float)raw / ACCEL_LSB_PER_G; }

 private:
  static constexpr int N = WINDOW_SAMPLES;

  SpectrumAnalyzer analyzer_;
  EpisodeTracker tracker_;
  BiquadLowPass lpf_[3];

  // Búferes circulares (índice head_ = posición de la próxima escritura).
  float gyro_ring_[3][N];
  float grav_ring_[3][N];
  int head_ = 0;
  int filled_ = 0;  // muestras consecutivas disponibles (máx. N)

  // Copias lineales en orden temporal para el análisis.
  float gyro_lin_[3][N];
  float grav_lin_[3][N];

  bool have_prev_ = false;
  uint32_t prev_seq_ = 0;
  uint32_t missing_total_ = 0;
};

}  // namespace tremor
