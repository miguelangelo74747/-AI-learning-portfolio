// =============================================================================
//  output_task.cpp — salida del dispositivo: LED RGB y log serie.
//
//  Formato del log (una línea por mensaje, campos separados por comas; las
//  líneas S, W, E, G y la cabecera las genera lib/tremor/src/log_format.cpp):
//   C,clave,valor                         configuración al arrancar
//   S,seq,t_us,ax,ay,az,gx,gy,gz          muestra cruda (cuentas del ADC)
//   W,seq_end,t_us,valid,peak_hz,tremor_dps,conc,move_dps,tilt_deg,c1,c2,c3,c4,
//     pos_streak,episode,proc_us          ventana de 2.56 s cada 0.5 s
//   E,START|END,id,first_pos_seq,seq,t_us,mean_hz,mean_dps,max_dps,
//     pos_windows,total_windows,reason    episodio de temblor
//   F,FAULT|RECOVER|RETRY_FAIL,reason,seq,t_us,detail   estado del sensor
//   G,seq,t_us,missing                    hueco en la numeración de muestras
//   L,t_us,color                          cambio de color del LED
//   R,t_us,clave=valor,...                informe de contadores cada 10 s
//   M,t_us,n                              marca manual (tecla 'm')
//   # texto                               comentario
// =============================================================================
#include <esp_timer.h>
#include <inttypes.h>

#include "log_format.h"
#include "shared.h"
#include "status_led.h"

namespace {

StatusLed s_led;
char s_line[256];

void emit(int len) {
  if (len <= 0) return;
  if (len >= (int)sizeof(s_line)) len = sizeof(s_line) - 1;
  Serial.write((const uint8_t*)s_line, len);
}

void emitLine(const char* line, int len, void*) { Serial.write((const uint8_t*)line, len); }

void printStatus(const StatusMsg& st) {
  const char* kind =
      st.kind == StatusMsg::FAULT ? "FAULT" : (st.kind == StatusMsg::RECOVER ? "RECOVER" : "RETRY_FAIL");
  emit(snprintf(s_line, sizeof(s_line), "F,%s,%s,%" PRIu32 ",%" PRId64 ",0x%02X\n", kind,
                faultReasonName(st.reason), st.seq, st.t_us, st.detail));
}

void printReport() {
  const Stats& s = g_stats;
  emit(snprintf(s_line, sizeof(s_line),
                "R,%" PRId64 ",ticks=%" PRIu32 ",samples=%" PRIu32 ",missed_ticks=%" PRIu32 ",read_fail=%" PRIu32
                ",stale=%" PRIu32 ",fault_ticks=%" PRIu32 ",sample_q_full=%" PRIu32 ",log_q_full=%" PRIu32
                ",seq_missing=%" PRIu32 ",windows=%" PRIu32 ",valid_windows=%" PRIu32 ",episodes=%" PRIu32
                ",faults=%" PRIu32 ",max_window_us=%" PRIu32 ",max_read_us=%" PRIu32 ",log_q_min_free=%" PRIu32 "\n",
                esp_timer_get_time(), s.timer_ticks, s.samples_ok, s.missed_ticks, s.read_fail, s.stale_reads,
                s.fault_ticks, s.sample_q_full, s.log_q_full, s.seq_missing, s.windows, s.windows_valid, s.episodes,
                s.faults, s.max_window_us, s.max_read_us, s.log_q_min_free));
}

void printHeader() { tremor::formatHeader(emitLine, nullptr, tremor::SpectrumAnalyzer::fftBackend()); }

void printMessage(const LogMsg& msg) {
  switch (msg.type) {
    case LogMsg::SAMPLE:
      emit(tremor::formatSample(s_line, sizeof(s_line), msg.sample));
      break;
    case LogMsg::WINDOW: {
      const WindowMsg& w = msg.window;
      emit(tremor::formatWindow(s_line, sizeof(s_line), w.seq_end, w.t_us, w.valid, w.m, w.pos_streak, w.episode,
                                w.proc_us));
      break;
    }
    case LogMsg::EVENT:
      emit(tremor::formatEvent(s_line, sizeof(s_line), msg.event.ev, msg.event.t_us, msg.event.reason));
      break;
    case LogMsg::STATUS:
      printStatus(msg.status);
      break;
    case LogMsg::GAP:
      emit(tremor::formatGap(s_line, sizeof(s_line), msg.gap.seq, msg.gap.t_us, msg.gap.missing));
      break;
  }
}

LedState currentLedState() {
  if (g_sensor_fault) return LedState::FAULT;
  if (g_episode_active) return LedState::EPISODE;
  if (g_stats.samples_ok == 0) return LedState::BOOT;
  return LedState::NORMAL;
}

void handleSerialInput(uint32_t& markers) {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'm' || c == 'M') {
      emit(snprintf(s_line, sizeof(s_line), "M,%" PRId64 ",%" PRIu32 "\n", esp_timer_get_time(), ++markers));
    } else if (c == 'r' || c == 'R') {
      printReport();
    } else if (c == 'h' || c == 'H') {
      printHeader();
    }
  }
}

}  // namespace

void outputTask(void*) {
  s_led.begin(PIN_RGB_LED, LED_BRIGHTNESS);
  printHeader();

  uint32_t last_report_ms = millis();
  uint32_t markers = 0;
  LedState last_state = LedState::BOOT;
  LogMsg msg;

  for (;;) {
    // Vacía la cola; como mucho 20 ms de espera para que el LED parpadee a tiempo.
    if (xQueueReceive(g_log_queue, &msg, pdMS_TO_TICKS(20)) == pdTRUE) printMessage(msg);

    const uint32_t now = millis();
    const LedState st = currentLedState();
    s_led.update(st, now);
    if (st != last_state) {
      last_state = st;
      emit(snprintf(s_line, sizeof(s_line), "L,%" PRId64 ",%s\n", esp_timer_get_time(), s_led.colorName()));
    }

    const UBaseType_t free_slots = uxQueueSpacesAvailable(g_log_queue);
    if (free_slots < g_stats.log_q_min_free) g_stats.log_q_min_free = free_slots;

    if (now - last_report_ms >= REPORT_PERIOD_MS) {
      last_report_ms = now;
      printReport();
    }
    handleSerialInput(markers);
  }
}
