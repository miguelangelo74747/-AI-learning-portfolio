// =============================================================================
//  tremor_dsp.cpp — ver tremor_dsp.h
// =============================================================================
#include "tremor_dsp.h"

#include <math.h>
#include <string.h>

#if defined(TREMOR_USE_ESP_DSP)
#include <esp_dsp.h>
#endif

namespace tremor {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 57.29577951308232f;
}  // namespace

// ---------------------------------------------------------------------------
//  BiquadLowPass (RBJ Audio EQ Cookbook, paso bajo)
// ---------------------------------------------------------------------------
void BiquadLowPass::design(float fc_hz, float q, float fs_hz) {
  const float w0 = 2.0f * kPi * fc_hz / fs_hz;
  const float cw = cosf(w0);
  const float alpha = sinf(w0) / (2.0f * q);
  const float a0 = 1.0f + alpha;
  b0_ = (1.0f - cw) * 0.5f / a0;
  b1_ = (1.0f - cw) / a0;
  b2_ = b0_;
  a1_ = -2.0f * cw / a0;
  a2_ = (1.0f - alpha) / a0;
  reset(0.0f);
}

void BiquadLowPass::reset(float x) {
  // Ganancia en continua = 1, así que el régimen permanente es y = x.
  x1_ = x2_ = y1_ = y2_ = x;
}

float BiquadLowPass::process(float x) {
  const float y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
  x2_ = x1_;
  x1_ = x;
  y2_ = y1_;
  y1_ = y;
  return y;
}

// ---------------------------------------------------------------------------
//  SpectrumAnalyzer
// ---------------------------------------------------------------------------
static_assert((SpectrumAnalyzer::N & (SpectrumAnalyzer::N - 1)) == 0, "N debe ser potencia de 2");

bool SpectrumAnalyzer::begin() {
  // Ventana Hann periódica: w[n] = 0.5 (1 - cos(2πn/N)), igual que en tremor_ref.py.
  window_power_sum_ = 0.0f;
  for (int n = 0; n < N; ++n) {
    window_[n] = 0.5f * (1.0f - cosf(2.0f * kPi * n / N));
    window_power_sum_ += window_[n] * window_[n];
  }
  for (int k = 0; k < N / 2; ++k) {
    cos_[k] = cosf(2.0f * kPi * k / N);
    sin_[k] = sinf(2.0f * kPi * k / N);
  }
  int bits = 0;
  while ((1 << bits) < N) ++bits;
  for (int i = 0; i < N; ++i) {
    int r = 0;
    for (int b = 0; b < bits; ++b) r |= ((i >> b) & 1) << (bits - 1 - b);
    bitrev_[i] = (uint16_t)r;
  }
  memset(power_, 0, sizeof(power_));
#if defined(TREMOR_USE_ESP_DSP)
  return dsps_fft2r_init_fc32(NULL, N) == ESP_OK;
#else
  return true;
#endif
}

const char* SpectrumAnalyzer::fftBackend() {
#if defined(TREMOR_USE_ESP_DSP)
  return "esp-dsp dsps_fft2r_fc32";
#else
  return "radix-2 propia";
#endif
}

void SpectrumAnalyzer::fft(float* re, float* im) {
#if defined(TREMOR_USE_ESP_DSP)
  for (int i = 0; i < N; ++i) {
    cplx_[2 * i] = re[i];
    cplx_[2 * i + 1] = im[i];
  }
  dsps_fft2r_fc32(cplx_, N);
  dsps_bit_rev_fc32(cplx_, N);
  for (int i = 0; i < N; ++i) {
    re[i] = cplx_[2 * i];
    im[i] = cplx_[2 * i + 1];
  }
#else
  // Permutación de bits invertidos.
  for (int i = 0; i < N; ++i) {
    const int j = bitrev_[i];
    if (j > i) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  // Mariposas radix-2, decimación en el tiempo. Factor de giro e^{-j2πk/N}.
  for (int size = 2; size <= N; size <<= 1) {
    const int half = size >> 1;
    const int step = N / size;
    for (int start = 0; start < N; start += size) {
      for (int j = 0; j < half; ++j) {
        const float c = cos_[j * step];
        const float s = sin_[j * step];
        const int a = start + j;
        const int b = a + half;
        const float tr = re[b] * c + im[b] * s;
        const float ti = im[b] * c - re[b] * s;
        re[b] = re[a] - tr;
        im[b] = im[a] - ti;
        re[a] += tr;
        im[a] += ti;
      }
    }
  }
#endif
}

int SpectrumAnalyzer::binLow(float hz) const {
  const int k = (int)ceilf(hz / binHz() - 1e-4f);
  return k < 0 ? 0 : (k > NBINS - 1 ? NBINS - 1 : k);
}

int SpectrumAnalyzer::binHigh(float hz) const {
  const int k = (int)floorf(hz / binHz() + 1e-4f);
  return k < 0 ? 0 : (k > NBINS - 1 ? NBINS - 1 : k);
}

void SpectrumAnalyzer::analyze(const float* const gyro[3], const float* const grav[3],
                               WindowMetrics& out) {
  // --- 1. Espectro de potencia de cada eje del giróscopo, sumado -------------
  // Escala de Parseval para ventana Hann: la suma de power_[k] sobre todos los
  // bins es la media cuadrática de la señal (sin su media) en (°/s)².
  const float scale_inner = 2.0f / ((float)N * window_power_sum_);
  const float scale_edge = 1.0f / ((float)N * window_power_sum_);
  memset(power_, 0, sizeof(power_));
  for (int axis = 0; axis < 3; ++axis) {
    const float* x = gyro[axis];
    float mean = 0.0f;
    for (int i = 0; i < N; ++i) mean += x[i];
    mean /= (float)N;
    for (int i = 0; i < N; ++i) {
      re_[i] = (x[i] - mean) * window_[i];
      im_[i] = 0.0f;
    }
    fft(re_, im_);
    for (int k = 0; k < NBINS; ++k) {
      const float p = re_[k] * re_[k] + im_[k] * im_[k];
      power_[k] += p * ((k == 0 || k == N / 2) ? scale_edge : scale_inner);
    }
  }

  // --- 2. Pico más fuerte en la banda de referencia 0.5–15 Hz (C1) ----------
  const int ref_lo = binLow(CONC_REF_LOW_HZ);
  const int ref_hi = binHigh(CONC_REF_HIGH_HZ);
  int kpk = ref_lo;
  float ref_power = 0.0f;
  for (int k = ref_lo; k <= ref_hi; ++k) {
    ref_power += power_[k];
    if (power_[k] > power_[kpk]) kpk = k;
  }
  // Interpolación gaussiana (parábola sobre ln P) con los dos bins vecinos.
  float delta = 0.0f;
  if (kpk > 0 && kpk < NBINS - 1) {
    const float a = power_[kpk - 1], b = power_[kpk], c = power_[kpk + 1];
    if (a > 0.0f && b > 0.0f && c > 0.0f) {
      const float la = logf(a), lb = logf(b), lc = logf(c);
      const float den = la - 2.0f * lb + lc;
      if (den < 0.0f) delta = 0.5f * (la - lc) / den;
      if (delta > 0.5f) delta = 0.5f;
      if (delta < -0.5f) delta = -0.5f;
    }
  }
  out.peak_hz = (ref_power > 0.0f) ? ((float)kpk + delta) * binHz() : 0.0f;

  // --- 3. Concentración alrededor del pico (C3) ------------------------------
  float near_power = 0.0f;
  for (int k = ref_lo; k <= ref_hi; ++k) {
    if (fabsf((float)k * binHz() - out.peak_hz) <= PEAK_HALF_WIDTH_HZ) near_power += power_[k];
  }
  out.concentration = (ref_power > 0.0f) ? near_power / ref_power : 0.0f;

  // --- 4. RMS de las bandas de temblor (C2) y de movimiento (C4) -------------
  float tremor_power = 0.0f;
  for (int k = binLow(TREMOR_BAND_LOW_HZ); k <= binHigh(TREMOR_BAND_HIGH_HZ); ++k) tremor_power += power_[k];
  float move_power = 0.0f;
  for (int k = binLow(MOVE_BAND_LOW_HZ); k <= binHigh(MOVE_BAND_HIGH_HZ); ++k) move_power += power_[k];
  out.tremor_dps = sqrtf(tremor_power);
  out.move_dps = sqrtf(move_power);

  // --- 5. Cambio de inclinación del antebrazo (C4) ---------------------------
  // Ángulo máximo entre el vector de gravedad más nuevo y cualquiera anterior
  // de la ventana: se busca el coseno mínimo y se hace un único acos.
  const float nx = grav[0][N - 1], ny = grav[1][N - 1], nz = grav[2][N - 1];
  const float nn = sqrtf(nx * nx + ny * ny + nz * nz);
  float min_cos = 1.0f;
  if (nn > 1e-6f) {
    for (int i = 0; i < N - 1; ++i) {
      const float gx = grav[0][i], gy = grav[1][i], gz = grav[2][i];
      const float gn = sqrtf(gx * gx + gy * gy + gz * gz);
      if (gn <= 1e-6f) continue;
      const float c = (nx * gx + ny * gy + nz * gz) / (nn * gn);
      if (c < min_cos) min_cos = c;
    }
  }
  if (min_cos < -1.0f) min_cos = -1.0f;
  out.tilt_deg = acosf(min_cos) * kRadToDeg;

  // --- 6. Condiciones ---------------------------------------------------------
  out.c1 = out.peak_hz >= TREMOR_BAND_LOW_HZ && out.peak_hz <= TREMOR_BAND_HIGH_HZ;
  out.c2 = out.tremor_dps >= TREMOR_MIN_DPS;
  out.c3 = out.concentration >= PEAK_CONCENTRATION_MIN;
  out.c4 = out.move_dps < MOVE_MAX_DPS && out.tilt_deg < REST_MAX_TILT_DEG;
}

// ---------------------------------------------------------------------------
//  EpisodeTracker
// ---------------------------------------------------------------------------
EpisodeEvent EpisodeTracker::makeEvent(EpisodeEvent::Type t, uint32_t seq) const {
  EpisodeEvent ev;
  ev.type = t;
  ev.id = cur_id_;
  ev.first_pos_seq = first_pos_seq_;
  ev.seq = seq;
  ev.mean_hz = n_pos_ ? (float)(sum_hz_ / n_pos_) : 0.0f;
  ev.mean_dps = n_pos_ ? (float)(sum_dps_ / n_pos_) : 0.0f;
  ev.max_dps = max_dps_;
  ev.positive_windows = n_pos_;
  ev.total_windows = n_total_;
  return ev;
}

EpisodeEvent EpisodeTracker::update(bool valid, const WindowMetrics& m, uint32_t window_end_seq) {
  EpisodeEvent ev;
  const bool pos = valid && m.positive();
  if (pos) {
    if (pos_streak_ == 0) {
      streak_first_seq_ = window_end_seq;
      streak_sum_hz_ = streak_sum_dps_ = 0.0;
      streak_max_dps_ = 0.0f;
    }
    ++pos_streak_;
    neg_streak_ = 0;
    streak_sum_hz_ += m.peak_hz;
    streak_sum_dps_ += m.tremor_dps;
    if (m.tremor_dps > streak_max_dps_) streak_max_dps_ = m.tremor_dps;

    if (active_) {
      ++n_total_;
      ++n_pos_;
      sum_hz_ += m.peak_hz;
      sum_dps_ += m.tremor_dps;
      if (m.tremor_dps > max_dps_) max_dps_ = m.tremor_dps;
    } else if (pos_streak_ >= TREMOR_MIN_WINDOWS) {
      // Inicio de episodio: las ventanas de la racha pasan a ser del episodio.
      active_ = true;
      cur_id_ = next_id_++;
      first_pos_seq_ = streak_first_seq_;
      sum_hz_ = streak_sum_hz_;
      sum_dps_ = streak_sum_dps_;
      max_dps_ = streak_max_dps_;
      n_pos_ = n_total_ = (uint32_t)pos_streak_;
      ev = makeEvent(EpisodeEvent::START, window_end_seq);
    }
  } else {
    pos_streak_ = 0;
    ++neg_streak_;
    if (active_) {
      ++n_total_;
      if (neg_streak_ >= EPISODE_RELEASE_WINDOWS) {
        ev = makeEvent(EpisodeEvent::END, window_end_seq);
        active_ = false;
      }
    }
  }
  return ev;
}

EpisodeEvent EpisodeTracker::forceEnd(uint32_t seq) {
  EpisodeEvent ev;
  if (active_) {
    ev = makeEvent(EpisodeEvent::END, seq);
    active_ = false;
  }
  pos_streak_ = 0;
  neg_streak_ = 0;
  return ev;
}

}  // namespace tremor
