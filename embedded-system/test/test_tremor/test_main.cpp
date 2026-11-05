// =============================================================================
//  Pruebas en el PC de lib/tremor:  pio test -e native
//
//  Señales sintéticas con respuesta conocida para cada condición C1–C4, la
//  máquina de episodios y la detección de huecos.
// =============================================================================
#include <math.h>
#include <stdlib.h>
#include <unity.h>

#include "tremor_dsp.h"
#include "tremor_pipeline.h"

using namespace tremor;

namespace {

constexpr int N = WINDOW_SAMPLES;
constexpr double kPi = 3.14159265358979323846;

float g_gyro[3][N];
float g_grav[3][N];

// Rellena la ventana: seno en el eje X del giróscopo (amplitud de pico en °/s)
// y gravedad fija en Z.
void fillSine(float freq_hz, float peak_dps, float noise_dps = 0.0f, unsigned seed = 1) {
  srand(seed);
  for (int i = 0; i < N; ++i) {
    const double t = i / (double)SAMPLE_RATE_HZ;
    for (int a = 0; a < 3; ++a) {
      const float noise = noise_dps * ((float)rand() / RAND_MAX - 0.5f) * 3.4641f;  // uniforme, RMS = noise_dps
      g_gyro[a][i] = 2.0f + noise;  // sesgo de 2 °/s: lo quita la media
    }
    g_gyro[0][i] += (float)(peak_dps * sin(2 * kPi * freq_hz * t));
    g_grav[0][i] = 0.0f;
    g_grav[1][i] = 0.0f;
    g_grav[2][i] = 1.0f;
  }
}

WindowMetrics analyzeWindow() {
  static SpectrumAnalyzer an;
  static bool init = false;
  if (!init) {
    TEST_ASSERT_TRUE(an.begin());
    init = true;
  }
  const float* gy[3] = {g_gyro[0], g_gyro[1], g_gyro[2]};
  const float* gr[3] = {g_grav[0], g_grav[1], g_grav[2]};
  WindowMetrics m;
  an.analyze(gy, gr, m);
  return m;
}

}  // namespace

void setUp() {}
void tearDown() {}

// --- Analizador espectral ---------------------------------------------------

void test_tremor_5hz_passes_all_conditions() {
  fillSine(5.0f, 20.0f);  // 20 °/s de pico = 14.14 °/s RMS
  const WindowMetrics m = analyzeWindow();
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 5.0f, m.peak_hz);
  TEST_ASSERT_FLOAT_WITHIN(0.03f * 14.142f, 14.142f, m.tremor_dps);
  TEST_ASSERT_TRUE(m.concentration > 0.95f);
  TEST_ASSERT_TRUE(m.move_dps < 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, m.tilt_deg);
  TEST_ASSERT_TRUE(m.c1 && m.c2 && m.c3 && m.c4);
  TEST_ASSERT_TRUE(m.positive());
}

void test_peak_interpolation_between_bins() {
  // 4.1 Hz cae entre los bins 10 (3.906 Hz) y 11 (4.297 Hz).
  fillSine(4.1f, 20.0f);
  const WindowMetrics m = analyzeWindow();
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 4.1f, m.peak_hz);
  TEST_ASSERT_TRUE(m.positive());
}

void test_2hz_fails_c1_and_c4() {
  fillSine(2.0f, 40.0f);  // 28 °/s RMS en la banda de movimiento
  const WindowMetrics m = analyzeWindow();
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, m.peak_hz);
  TEST_ASSERT_FALSE(m.c1);
  TEST_ASSERT_FALSE(m.c4);
  TEST_ASSERT_FALSE(m.positive());
}

void test_physiological_10hz_fails_c1() {
  fillSine(10.0f, 20.0f);
  const WindowMetrics m = analyzeWindow();
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, m.peak_hz);
  TEST_ASSERT_FALSE(m.c1);
}

void test_weak_tremor_fails_c2() {
  fillSine(5.0f, 4.0f);  // 2.83 °/s RMS < 5 °/s
  const WindowMetrics m = analyzeWindow();
  TEST_ASSERT_TRUE(m.c1);
  TEST_ASSERT_FALSE(m.c2);
  TEST_ASSERT_TRUE(m.c3);
}

void test_broadband_noise_fails_c3() {
  fillSine(5.0f, 0.0f, 30.0f, 7);  // solo ruido blanco
  const WindowMetrics m = analyzeWindow();
  TEST_ASSERT_TRUE(m.concentration < PEAK_CONCENTRATION_MIN);
  TEST_ASSERT_FALSE(m.c3);
}

void test_tilt_change_fails_c4() {
  fillSine(5.0f, 20.0f);
  // La gravedad gira 15° alrededor de X en la segunda mitad de la ventana.
  const float ang = 15.0f * (float)kPi / 180.0f;
  for (int i = N / 2; i < N; ++i) {
    g_grav[1][i] = sinf(ang);
    g_grav[2][i] = cosf(ang);
  }
  const WindowMetrics m = analyzeWindow();
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 15.0f, m.tilt_deg);
  TEST_ASSERT_FALSE(m.c4);
}

void test_parseval_scaling_of_power_spectrum() {
  // La suma de todos los bins debe dar la media cuadrática: (A²/2) para un seno.
  fillSine(6.25f, 10.0f);  // 6.25 Hz = bin 16 exacto
  static SpectrumAnalyzer an;
  an.begin();
  const float* gy[3] = {g_gyro[0], g_gyro[1], g_gyro[2]};
  const float* gr[3] = {g_grav[0], g_grav[1], g_grav[2]};
  WindowMetrics m;
  an.analyze(gy, gr, m);
  double total = 0;
  for (int k = 0; k < SpectrumAnalyzer::NBINS; ++k) total += an.powerSpectrum()[k];
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, (float)total);
}

// --- Filtro de inclinación ------------------------------------------------

void test_biquad_dc_gain_and_tremor_rejection() {
  BiquadLowPass f;
  f.design(TILT_LPF_HZ, TILT_LPF_Q, SAMPLE_RATE_HZ);
  f.reset(0.0f);
  float y = 0;
  for (int i = 0; i < 2000; ++i) y = f.process(1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, y);

  // Amplitud de salida a 3.5 Hz en régimen permanente: ≈ 2 % (propuesta).
  f.reset(0.0f);
  float peak = 0;
  for (int i = 0; i < 4000; ++i) {
    y = f.process(sinf(2.0f * (float)kPi * 3.5f * i / SAMPLE_RATE_HZ));
    if (i > 2000 && fabsf(y) > peak) peak = fabsf(y);
  }
  TEST_ASSERT_TRUE(peak < 0.03f);
  TEST_ASSERT_TRUE(peak > 0.01f);
}

// --- Máquina de episodios ---------------------------------------------------

void test_episode_needs_6_windows_and_releases_after_4() {
  EpisodeTracker t;
  WindowMetrics pos;
  pos.c1 = pos.c2 = pos.c3 = pos.c4 = true;
  pos.peak_hz = 5.0f;
  pos.tremor_dps = 10.0f;
  WindowMetrics neg;

  uint32_t seq = 49;
  for (int i = 0; i < TREMOR_MIN_WINDOWS - 1; ++i, seq += 50) {
    TEST_ASSERT_EQUAL(EpisodeEvent::NONE, t.update(true, pos, seq).type);
  }
  EpisodeEvent ev = t.update(true, pos, seq);
  TEST_ASSERT_EQUAL(EpisodeEvent::START, ev.type);
  TEST_ASSERT_EQUAL_UINT32(1, ev.id);
  TEST_ASSERT_EQUAL_UINT32(49, ev.first_pos_seq);
  TEST_ASSERT_EQUAL_UINT32(6, ev.positive_windows);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.0f, ev.mean_hz);
  TEST_ASSERT_TRUE(t.active());

  // 3 negativas y una positiva: el episodio sigue (no parpadea).
  for (int i = 0; i < EPISODE_RELEASE_WINDOWS - 1; ++i) TEST_ASSERT_EQUAL(EpisodeEvent::NONE, t.update(true, neg, seq += 50).type);
  TEST_ASSERT_EQUAL(EpisodeEvent::NONE, t.update(true, pos, seq += 50).type);
  TEST_ASSERT_TRUE(t.active());

  for (int i = 0; i < EPISODE_RELEASE_WINDOWS - 1; ++i) TEST_ASSERT_EQUAL(EpisodeEvent::NONE, t.update(false, pos, seq += 50).type);
  ev = t.update(false, pos, seq += 50);
  TEST_ASSERT_EQUAL(EpisodeEvent::END, ev.type);
  TEST_ASSERT_EQUAL_UINT32(7, ev.positive_windows);
  TEST_ASSERT_EQUAL_UINT32(6 + 3 + 1 + 4, ev.total_windows);
  TEST_ASSERT_FALSE(t.active());
}

void test_interrupted_streak_does_not_start() {
  EpisodeTracker t;
  WindowMetrics pos;
  pos.c1 = pos.c2 = pos.c3 = pos.c4 = true;
  WindowMetrics neg;
  uint32_t seq = 49;
  for (int rep = 0; rep < 5; ++rep) {
    for (int i = 0; i < TREMOR_MIN_WINDOWS - 1; ++i) TEST_ASSERT_EQUAL(EpisodeEvent::NONE, t.update(true, pos, seq += 50).type);
    TEST_ASSERT_EQUAL(EpisodeEvent::NONE, t.update(true, neg, seq += 50).type);
  }
  TEST_ASSERT_FALSE(t.active());
}

// --- Pipeline completo ------------------------------------------------------

RawSample makeSample(uint32_t seq, float freq_hz, float peak_dps) {
  RawSample s{};
  s.seq = seq;
  s.t_us = (int64_t)seq * SAMPLE_PERIOD_US;
  const double t = seq / (double)SAMPLE_RATE_HZ;
  s.gx = (int16_t)lround(peak_dps * sin(2 * kPi * freq_hz * t) * GYRO_LSB_PER_DPS);
  s.gy = (int16_t)(seq % 3);  // un poco de "ruido"
  s.gz = -5;
  s.ax = 0;
  s.ay = 0;
  s.az = (int16_t)ACCEL_LSB_PER_G;
  return s;
}

void test_pipeline_detects_episode_after_window_fill_plus_3s() {
  static TremorPipeline p;
  TEST_ASSERT_TRUE(p.begin());
  PipelineOutput out;
  int windows = 0, valid = 0;
  int64_t start_seq = -1;
  for (uint32_t seq = 0; seq < 1500; ++seq) {
    p.push(makeSample(seq, 5.0f, 30.0f), out);
    if (out.window_evaluated) {
      ++windows;
      if (out.window_valid) ++valid;
      TEST_ASSERT_EQUAL_UINT32(0, (out.window_end_seq + 1) % HOP_SAMPLES);
    }
    if (out.event.type == EpisodeEvent::START && start_seq < 0) start_seq = seq;
  }
  TEST_ASSERT_EQUAL(30, windows);
  // Primera ventana completa en seq 299 (la de seq 249 tiene solo 250 muestras);
  // 6 positivas seguidas -> START en seq 299 + 5 * 50 = 549.
  TEST_ASSERT_EQUAL(25, valid);
  TEST_ASSERT_EQUAL(549, start_seq);
  TEST_ASSERT_TRUE(p.episodeActive());
}

void test_pipeline_gap_resets_window_and_ends_episode() {
  static TremorPipeline p;
  TEST_ASSERT_TRUE(p.begin());
  PipelineOutput out;
  uint32_t seq = 0;
  for (; seq < 1000; ++seq) p.push(makeSample(seq, 5.0f, 30.0f), out);
  TEST_ASSERT_TRUE(p.episodeActive());

  seq += 7;  // se pierden 7 muestras
  p.push(makeSample(seq, 5.0f, 30.0f), out);
  TEST_ASSERT_EQUAL_UINT32(7, out.gap_missing);
  TEST_ASSERT_EQUAL(EpisodeEvent::END, out.gap_event.type);
  TEST_ASSERT_FALSE(p.episodeActive());
  TEST_ASSERT_EQUAL_UINT32(7, p.missingTotal());

  // Hasta tener otra vez 256 muestras seguidas las ventanas no son válidas.
  int invalid = 0;
  for (++seq; seq < 1007 + 256; ++seq) {
    p.push(makeSample(seq, 5.0f, 30.0f), out);
    if (out.window_evaluated) {
      TEST_ASSERT_FALSE(out.window_valid);
      ++invalid;
    }
  }
  TEST_ASSERT_TRUE(invalid >= 4);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_tremor_5hz_passes_all_conditions);
  RUN_TEST(test_peak_interpolation_between_bins);
  RUN_TEST(test_2hz_fails_c1_and_c4);
  RUN_TEST(test_physiological_10hz_fails_c1);
  RUN_TEST(test_weak_tremor_fails_c2);
  RUN_TEST(test_broadband_noise_fails_c3);
  RUN_TEST(test_tilt_change_fails_c4);
  RUN_TEST(test_parseval_scaling_of_power_spectrum);
  RUN_TEST(test_biquad_dc_gain_and_tremor_rejection);
  RUN_TEST(test_episode_needs_6_windows_and_releases_after_4);
  RUN_TEST(test_interrupted_streak_does_not_start);
  RUN_TEST(test_pipeline_detects_episode_after_window_fill_plus_3s);
  RUN_TEST(test_pipeline_gap_resets_window_and_ends_episode);
  return UNITY_END();
}
