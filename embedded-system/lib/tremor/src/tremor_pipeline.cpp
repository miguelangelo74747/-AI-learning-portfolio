// =============================================================================
//  tremor_pipeline.cpp — ver tremor_pipeline.h
// =============================================================================
#include "tremor_pipeline.h"

namespace tremor {

bool TremorPipeline::begin() {
  for (auto& f : lpf_) f.design(TILT_LPF_HZ, TILT_LPF_Q, SAMPLE_RATE_HZ);
  head_ = 0;
  filled_ = 0;
  have_prev_ = false;
  missing_total_ = 0;
  return analyzer_.begin();
}

void TremorPipeline::push(const RawSample& s, PipelineOutput& out) {
  out = PipelineOutput();

  const float a[3] = {accelG(s.ax), accelG(s.ay), accelG(s.az)};
  const float g[3] = {gyroDps(s.gx), gyroDps(s.gy), gyroDps(s.gz)};

  // --- Números de muestra que faltan ---------------------------------------
  // Un hueco rompe la continuidad de la ventana: se vacía, se reinicia el
  // filtro de inclinación y se cierra el episodio en curso (no se rellenan
  // muestras con valores inventados).
  bool restart = !have_prev_;
  if (have_prev_ && s.seq != prev_seq_ + 1) {
    out.gap_missing = s.seq - prev_seq_ - 1;
    missing_total_ += out.gap_missing;
    out.gap_event = tracker_.forceEnd(s.seq);
    restart = true;
  }
  have_prev_ = true;
  prev_seq_ = s.seq;

  if (restart) {
    filled_ = 0;
    for (int i = 0; i < 3; ++i) lpf_[i].reset(a[i]);
  }

  // --- Búfer de 256 muestras ------------------------------------------------
  for (int i = 0; i < 3; ++i) {
    gyro_ring_[i][head_] = g[i];
    grav_ring_[i][head_] = lpf_[i].process(a[i]);
  }
  head_ = (head_ + 1) % N;
  if (filled_ < N) ++filled_;

  // --- Ventana cada HOP_SAMPLES muestras ------------------------------------
  if ((s.seq + 1) % HOP_SAMPLES != 0) return;

  out.window_evaluated = true;
  out.window_end_seq = s.seq;
  out.window_valid = (filled_ >= N);
  if (out.window_valid) {
    // head_ apunta a la muestra más antigua cuando el búfer está lleno.
    for (int i = 0; i < 3; ++i) {
      for (int n = 0; n < N; ++n) {
        const int idx = (head_ + n) % N;
        gyro_lin_[i][n] = gyro_ring_[i][idx];
        grav_lin_[i][n] = grav_ring_[i][idx];
      }
    }
    const float* gy[3] = {gyro_lin_[0], gyro_lin_[1], gyro_lin_[2]};
    const float* gr[3] = {grav_lin_[0], grav_lin_[1], grav_lin_[2]};
    analyzer_.analyze(gy, gr, out.metrics);
  }
  out.event = tracker_.update(out.window_valid, out.metrics, s.seq);
}

}  // namespace tremor
