// =============================================================================
//  log_format.cpp — ver log_format.h
// =============================================================================
#include "log_format.h"

#include <inttypes.h>
#include <stdio.h>

namespace tremor {

int formatSample(char* buf, size_t size, const RawSample& s) {
  return snprintf(buf, size, "S,%" PRIu32 ",%" PRId64 ",%d,%d,%d,%d,%d,%d\n", s.seq, s.t_us, s.ax, s.ay, s.az,
                  s.gx, s.gy, s.gz);
}

int formatWindow(char* buf, size_t size, uint32_t seq_end, int64_t t_us, bool valid, const WindowMetrics& m,
                 unsigned pos_streak, bool episode, uint32_t proc_us) {
  return snprintf(buf, size, "W,%" PRIu32 ",%" PRId64 ",%d,%.4f,%.4f,%.4f,%.4f,%.3f,%d,%d,%d,%d,%u,%d,%" PRIu32 "\n",
                  seq_end, t_us, valid ? 1 : 0, (double)m.peak_hz, (double)m.tremor_dps, (double)m.concentration,
                  (double)m.move_dps, (double)m.tilt_deg, m.c1 ? 1 : 0, m.c2 ? 1 : 0, m.c3 ? 1 : 0, m.c4 ? 1 : 0,
                  pos_streak, episode ? 1 : 0, proc_us);
}

int formatEvent(char* buf, size_t size, const EpisodeEvent& ev, int64_t t_us, const char* reason) {
  const char* type = ev.type == EpisodeEvent::START ? "START" : "END";
  return snprintf(buf, size,
                  "E,%s,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRId64 ",%.3f,%.3f,%.3f,%" PRIu32 ",%" PRIu32 ",%s\n",
                  type, ev.id, ev.first_pos_seq, ev.seq, t_us, (double)ev.mean_hz, (double)ev.mean_dps,
                  (double)ev.max_dps, ev.positive_windows, ev.total_windows, reason);
}

int formatGap(char* buf, size_t size, uint32_t seq, int64_t t_us, uint32_t missing) {
  return snprintf(buf, size, "G,%" PRIu32 ",%" PRId64 ",%" PRIu32 "\n", seq, t_us, missing);
}

void formatHeader(LineSink emit, void* ctx, const char* fft_backend) {
  char line[160];
  int n;
#define TREMOR_EMIT(...)                          \
  do {                                            \
    n = snprintf(line, sizeof(line), __VA_ARGS__); \
    if (n > 0) emit(line, n, ctx);                \
  } while (0)
#define CFG_F(name) TREMOR_EMIT("C," #name ",%g\n", (double)(name))
#define CFG_I(name) TREMOR_EMIT("C," #name ",%ld\n", (long)(name))

  TREMOR_EMIT("# Wrist Rest-Tremor Analyzer, equipo %d, firmware %s\n", TEAM_NUMBER, FW_VERSION);
  TREMOR_EMIT("C,FW_VERSION,%s\n", FW_VERSION);
  TREMOR_EMIT("C,FFT_BACKEND,%s\n", fft_backend);
  CFG_F(SAMPLE_RATE_HZ);
  CFG_I(SAMPLE_PERIOD_US);
  CFG_I(WINDOW_SAMPLES);
  CFG_I(HOP_SAMPLES);
  CFG_F(GYRO_LSB_PER_DPS);
  CFG_F(ACCEL_LSB_PER_G);
  CFG_I(MPU_DLPF_CFG);
  CFG_F(TREMOR_BAND_LOW_HZ);
  CFG_F(TREMOR_BAND_HIGH_HZ);
  CFG_F(TREMOR_MIN_DPS);
  CFG_F(PEAK_HALF_WIDTH_HZ);
  CFG_F(PEAK_CONCENTRATION_MIN);
  CFG_F(CONC_REF_LOW_HZ);
  CFG_F(CONC_REF_HIGH_HZ);
  CFG_F(MOVE_BAND_LOW_HZ);
  CFG_F(MOVE_BAND_HIGH_HZ);
  CFG_F(MOVE_MAX_DPS);
  CFG_F(REST_MAX_TILT_DEG);
  CFG_F(TILT_LPF_HZ);
  CFG_F(TILT_LPF_Q);
  CFG_F(TREMOR_MIN_DURATION_S);
  CFG_F(EPISODE_RELEASE_S);
  CFG_I(TREMOR_MIN_WINDOWS);
  CFG_I(EPISODE_RELEASE_WINDOWS);
  TREMOR_EMIT("# S,seq,t_us,ax,ay,az,gx,gy,gz\n");
  TREMOR_EMIT("# W,seq_end,t_us,valid,peak_hz,tremor_dps,conc,move_dps,tilt_deg,c1,c2,c3,c4,pos_streak,episode,proc_us\n");
  TREMOR_EMIT("# E,type,id,first_pos_seq,seq,t_us,mean_hz,mean_dps,max_dps,pos_windows,total_windows,reason\n");
#undef CFG_F
#undef CFG_I
#undef TREMOR_EMIT
}

}  // namespace tremor
