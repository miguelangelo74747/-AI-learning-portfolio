// =============================================================================
//  processing_task.cpp — pasos 3, 4 y 5 de la propuesta (sección 3).
//
//  Recibe cada muestra de la cola, la pasa por TremorPipeline (conversión a
//  °/s y g, ventana de 256 muestras, FFT y C1–C4 cada 0.5 s, episodios) y
//  reenvía a la tarea de salida la muestra, las ventanas y los episodios.
// =============================================================================
#include <esp_timer.h>

#include "shared.h"

namespace {

tremor::TremorPipeline s_pipeline;  // estático: ~16 KB fuera de la pila

void pushEvent(const tremor::EpisodeEvent& ev, const char* reason) {
  LogMsg msg;
  msg.type = LogMsg::EVENT;
  msg.event.ev = ev;
  msg.event.t_us = esp_timer_get_time();
  msg.event.reason = reason;
  logPush(msg);
}

}  // namespace

void processingTask(void*) {
  if (!s_pipeline.begin()) {
    Serial.println("# ERROR: no se pudo inicializar la FFT");
  }

  RawSample s;
  tremor::PipelineOutput out;
  for (;;) {
    if (xQueueReceive(g_sample_queue, &s, portMAX_DELAY) != pdTRUE) continue;

    const int64_t t0 = esp_timer_get_time();
    s_pipeline.push(s, out);
    const uint32_t proc_us = (uint32_t)(esp_timer_get_time() - t0);

    if (out.gap_missing) {
      g_stats.seq_missing = g_stats.seq_missing + out.gap_missing;
      LogMsg msg;
      msg.type = LogMsg::GAP;
      msg.gap.seq = s.seq;
      msg.gap.t_us = s.t_us;
      msg.gap.missing = out.gap_missing;
      logPush(msg);
      if (out.gap_event.type == tremor::EpisodeEvent::END) pushEvent(out.gap_event, "gap");
    }

    if (LOG_RAW_SAMPLES) {
      LogMsg msg;
      msg.type = LogMsg::SAMPLE;
      msg.sample = s;
      logPush(msg);
    }

    if (out.window_evaluated) {
      g_stats.windows = g_stats.windows + 1;
      if (out.window_valid) {
        g_stats.windows_valid = g_stats.windows_valid + 1;
        g_monitoring = true;
        if (proc_us > g_stats.max_window_us) g_stats.max_window_us = proc_us;
      }

      LogMsg msg;
      msg.type = LogMsg::WINDOW;
      msg.window.seq_end = out.window_end_seq;
      msg.window.t_us = s.t_us;
      msg.window.valid = out.window_valid;
      msg.window.m = out.metrics;
      const int streak = s_pipeline.positiveStreak();
      msg.window.pos_streak = (uint8_t)(streak > 255 ? 255 : streak);
      msg.window.episode = s_pipeline.episodeActive();
      msg.window.proc_us = proc_us;
      logPush(msg);

      if (out.event.type == tremor::EpisodeEvent::START) {
        g_stats.episodes = g_stats.episodes + 1;
        pushEvent(out.event, "rule");
      } else if (out.event.type == tremor::EpisodeEvent::END) {
        pushEvent(out.event, "rule");
      }
    }

    g_episode_active = s_pipeline.episodeActive();
  }
}
