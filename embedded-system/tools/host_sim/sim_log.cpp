// =============================================================================
//  sim_log.cpp — genera en el PC un log con el mismo formato que la placa.
//
//  Usa el código real de lib/tremor (TremorPipeline + log_format) sobre señales
//  sintéticas de giróscopo y acelerómetro. Sirve para probar los scripts de
//  Python sin hardware y como ejemplo de qué debe verse en cada prueba.
//  NO sustituye a las medidas reales: los tiempos (t_us, proc_us, reloj del
//  PC) son simulados.
//
//  Compilar y ejecutar (desde embedded-system/):
//    g++ -std=gnu++17 -O2 -Iinclude -Ilib/tremor/src tools/host_sim/sim_log.cpp lib/tremor/src/*.cpp -o sim_log
//    ./sim_log > logs/sim.log
//
//  Guion (s):  0–30 reposo | M1 30–45 temblor 5 Hz | 45–60 reposo |
//              M2 60–75 temblor 4 Hz | 75–90 reposo | M3 90–105 oscilación 2 Hz |
//              105–120 reposo | M4 120–135 brazo extendido (9.5 Hz fisiológico) |
//              135–150 escribir/teclear (banda ancha) | 150–155 cable SDA suelto |
//              155–200 reposo
// =============================================================================
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <chrono>
#include <random>

#include "config.h"
#include "log_format.h"
#include "tremor_pipeline.h"

using namespace tremor;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDuration = 200.0;      // s
constexpr double kHostClockPpm = 35.0;   // deriva simulada del reloj de la placa frente al PC
constexpr double kHostBase = 1793845956.0;

std::mt19937 rng(11);
std::normal_distribution<double> gauss(0.0, 1.0);

char line[256];
double g_host_t = kHostBase;

void out(const char* text, int len) {
  if (len <= 0) return;
  printf("%.6f\t%.*s", g_host_t, len, text);
}
void outSink(const char* text, int len, void*) { out(text, len); }

int16_t clamp16(double v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)lround(v);
}

// Oscilación de pronación–supinación alrededor del eje X del antebrazo, con la
// irregularidad típica de un movimiento voluntario (deriva de frecuencia y amplitud).
struct Oscillation {
  double f0, amp_dps, t_start, t_end;
  double phase = 0;
  double gyro(double t, double dt, double* angle_deg) {
    if (t < t_start || t >= t_end) {
      *angle_deg = 0;
      return 0;
    }
    const double f = f0 * (1.0 + 0.03 * sin(2 * kPi * 0.07 * t));
    const double a = amp_dps * (1.0 + 0.15 * sin(2 * kPi * 0.11 * t + 1.0));
    phase += 2 * kPi * f * dt;
    *angle_deg = -a / (2 * kPi * f) * cos(phase);  // integral de la velocidad angular
    return a * sin(phase);
  }
};

// Ruido de movimiento de banda ancha (escribir, teclear): ruido blanco filtrado
// por un paso bajo de 1.er orden (~3 Hz).
struct BroadbandMotion {
  double t_start, t_end, rms_dps;
  double state[3] = {0, 0, 0};
  double tilt_deg = 0;
  void gyro(double t, double out_dps[3]) {
    const double alpha = 0.17;
    for (int i = 0; i < 3; ++i) {
      double x = (t >= t_start && t < t_end) ? gauss(rng) * rms_dps * 2.6 : 0.0;
      state[i] += alpha * (x - state[i]);
      out_dps[i] = state[i];
    }
    if (t >= t_start && t < t_end) tilt_deg += out_dps[1] * 0.01 * 0.3;  // la postura cambia
    else tilt_deg *= 0.995;
  }
};

}  // namespace

int main() {
  TremorPipeline pipeline;
  pipeline.begin();
  formatHeader(outSink, nullptr, "simulador PC (radix-2 propia)");

  Oscillation tremor5{5.0, 25.0, 30.0, 45.0};
  Oscillation tremor4{4.0, 15.0, 60.0, 75.0};
  Oscillation slow2{2.0, 60.0, 90.0, 105.0};
  Oscillation physio{9.5, 3.0, 120.0, 135.0};
  BroadbandMotion writing{135.0, 150.0, 25.0};
  const double fault_start = 150.0, fault_end = 155.0;
  const double markers[] = {30.0, 60.0, 90.0, 120.0};
  int next_marker = 0;

  const double dt = 1.0 / SAMPLE_RATE_HZ;
  const uint32_t n_ticks = (uint32_t)(kDuration * SAMPLE_RATE_HZ);
  const int64_t boot_us = 1250000;  // la placa arranca 1.25 s antes de la primera muestra

  uint32_t samples_ok = 0, windows = 0, windows_valid = 0, episodes = 0, faults = 0, missing = 0;
  uint32_t max_window_us = 0;
  bool in_fault = false, fault_reported = false;
  int good_after_fault = 0;
  const char* led = "blue";
  auto setLed = [&](const char* color, int64_t t_us) {
    if (strcmp(led, color) == 0) return;
    led = color;
    out(line, snprintf(line, sizeof(line), "L,%" PRId64 ",%s\n", t_us, color));
  };

  for (uint32_t seq = 0; seq < n_ticks; ++seq) {
    const double t = seq * dt;
    const int64_t t_us = boot_us + (int64_t)seq * SAMPLE_PERIOD_US + (int64_t)lround(gauss(rng) * 3.0);
    g_host_t = kHostBase + (t_us / 1e6) * (1.0 + kHostClockPpm * 1e-6) + 0.002 + 0.0015 * fabs(gauss(rng));

    if (next_marker < 4 && t >= markers[next_marker]) {
      ++next_marker;
      out(line, snprintf(line, sizeof(line), "M,%" PRId64 ",%d\n", t_us, next_marker));
    }

    // --- Fallo del sensor: el cable SDA se suelta durante 5 s -------------------
    if (t >= fault_start && t < fault_end + 0.25) {
      // Lecturas fallidas: tras 10 seguidas (100 ms) -> FAULT; reintentos cada 1 s.
      if (!fault_reported && t >= fault_start + 0.09) {
        fault_reported = in_fault = true;
        ++faults;
        out(line, snprintf(line, sizeof(line), "F,FAULT,READ_FAIL,%" PRIu32 ",%" PRId64 ",0x00\n", seq, t_us));
        setLed("red_blink", t_us + 15000);
      }
      const uint32_t fault_tick = seq - (uint32_t)lround((fault_start + 0.09) * SAMPLE_RATE_HZ);
      if (fault_reported && fault_tick > 0 && fault_tick % (uint32_t)SAMPLE_RATE_HZ == 0 && t < fault_end) {
        out(line, snprintf(line, sizeof(line), "F,RETRY_FAIL,INIT_I2C,%" PRIu32 ",%" PRId64 ",0x00\n", seq, t_us));
      }
      continue;  // tic sin muestra
    }

    // --- Señal ---------------------------------------------------------------
    double ang_x_deg = 0, a5 = 0, a4 = 0, a2 = 0, aph = 0;
    double gx = tremor5.gyro(t, dt, &a5) + tremor4.gyro(t, dt, &a4) + slow2.gyro(t, dt, &a2) +
                physio.gyro(t, dt, &aph);
    ang_x_deg = a5 + a4 + a2 + aph;
    double wb[3];
    writing.gyro(t, wb);
    double g[3] = {gx + wb[0], wb[1], wb[2]};
    const double bias[3] = {-1.8, 0.9, 0.4};  // sesgo típico del MPU6050 (°/s)
    for (int i = 0; i < 3; ++i) g[i] += bias[i] + gauss(rng) * 0.12;

    // Gravedad: antebrazo apoyado (Z hacia arriba) girado ang_x alrededor de X
    // y writing.tilt alrededor de Y.
    const double rx = ang_x_deg * kPi / 180.0, ry = writing.tilt_deg * kPi / 180.0;
    double acc[3] = {sin(ry), -sin(rx) * cos(ry), cos(rx) * cos(ry)};
    for (int i = 0; i < 3; ++i) acc[i] += gauss(rng) * 0.004;

    RawSample s{};
    s.seq = seq;
    s.t_us = t_us;
    s.gx = clamp16(g[0] * GYRO_LSB_PER_DPS);
    s.gy = clamp16(g[1] * GYRO_LSB_PER_DPS);
    s.gz = clamp16(g[2] * GYRO_LSB_PER_DPS);
    s.ax = clamp16(acc[0] * ACCEL_LSB_PER_G);
    s.ay = clamp16(acc[1] * ACCEL_LSB_PER_G);
    s.az = clamp16(acc[2] * ACCEL_LSB_PER_G);
    ++samples_ok;
    if (samples_ok == 1) setLed("green", t_us);

    // --- Tarea de procesado (código real) ---------------------------------------
    PipelineOutput po;
    const auto c0 = std::chrono::steady_clock::now();
    pipeline.push(s, po);
    const uint32_t proc_us = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - c0)
                                 .count();
    if (po.gap_missing) {
      missing += po.gap_missing;
      out(line, formatGap(line, sizeof(line), s.seq, s.t_us, po.gap_missing));
      if (po.gap_event.type == EpisodeEvent::END) out(line, formatEvent(line, sizeof(line), po.gap_event, t_us, "gap"));
    }
    out(line, formatSample(line, sizeof(line), s));
    if (po.window_evaluated) {
      ++windows;
      if (po.window_valid) {
        ++windows_valid;
        if (proc_us > max_window_us) max_window_us = proc_us;
      }
      const int streak = pipeline.positiveStreak();
      out(line, formatWindow(line, sizeof(line), po.window_end_seq, s.t_us, po.window_valid, po.metrics,
                             (unsigned)(streak > 255 ? 255 : streak), pipeline.episodeActive(), proc_us));
      if (po.event.type == EpisodeEvent::START) {
        ++episodes;
        out(line, formatEvent(line, sizeof(line), po.event, t_us, "rule"));
      } else if (po.event.type == EpisodeEvent::END) {
        out(line, formatEvent(line, sizeof(line), po.event, t_us, "rule"));
      }
    }

    if (in_fault && ++good_after_fault >= RECOVER_GOOD_READS) {
      in_fault = false;
      out(line, snprintf(line, sizeof(line), "F,RECOVER,-,%" PRIu32 ",%" PRId64 ",0x00\n", seq, t_us));
    }
    setLed(in_fault ? "red_blink" : (pipeline.episodeActive() ? "amber" : "green"), t_us + 400);

    if ((seq + 1) % 1000 == 0) {  // informe cada 10 s
      out(line, snprintf(line, sizeof(line),
                         "R,%" PRId64 ",ticks=%" PRIu32 ",samples=%" PRIu32 ",missed_ticks=0,read_fail=%d,stale=0,"
                         "fault_ticks=%d,sample_q_full=0,log_q_full=0,seq_missing=%" PRIu32 ",windows=%" PRIu32
                         ",valid_windows=%" PRIu32 ",episodes=%" PRIu32 ",faults=%" PRIu32 ",max_window_us=%" PRIu32
                         ",max_read_us=0,log_q_min_free=%d\n",
                         t_us, seq + 1, samples_ok, faults ? 50 : 0, faults ? 525 : 0, missing, windows, windows_valid,
                         episodes, faults, max_window_us, LOG_QUEUE_LEN - 3));
    }
  }
  return 0;
}
