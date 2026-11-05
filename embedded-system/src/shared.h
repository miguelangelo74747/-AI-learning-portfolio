// =============================================================================
//  shared.h — colas, mensajes y contadores compartidos por las tres tareas.
//
//   [timer HW 100 Hz] -> SamplingTask --sampleQueue--> ProcessingTask
//                             |                              |
//                             +-------- logQueue ------------+--> OutputTask
//                                                                 (LED + Serial)
// =============================================================================
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "config.h"
#include "tremor_pipeline.h"

using tremor::RawSample;

// ---------------------------------------------------------------------------
//  Mensajes hacia la tarea de salida (una línea del log cada uno)
// ---------------------------------------------------------------------------
enum class FaultReason : uint8_t {
  NONE = 0,
  INIT_I2C,        // el sensor no responde en el bus al arrancar
  INIT_WHO_AM_I,   // WHO_AM_I != 0x68
  INIT_CONFIG,     // la configuración leída no coincide con la escrita
  READ_FAIL,       // 10 lecturas I2C fallidas seguidas
  DRDY_DEAD,       // bandera DATA_RDY a 0 en 10 lecturas seguidas
  FROZEN,          // giróscopo idéntico durante 1 s
};
const char* faultReasonName(FaultReason r);

struct WindowMsg {
  uint32_t seq_end;
  int64_t t_us;
  bool valid;
  tremor::WindowMetrics m;
  uint8_t pos_streak;
  bool episode;
  uint32_t proc_us;  // tiempo de análisis de la ventana
};

struct EventMsg {
  tremor::EpisodeEvent ev;
  int64_t t_us;
  const char* reason;  // literal estático: "rule", "gap"
};

struct StatusMsg {
  enum Kind : uint8_t { FAULT, RECOVER, RETRY_FAIL } kind;
  FaultReason reason;
  uint32_t seq;
  int64_t t_us;
  uint8_t detail;  // p. ej. valor de WHO_AM_I leído
};

struct GapMsg {
  uint32_t seq;  // primera muestra después del hueco
  int64_t t_us;
  uint32_t missing;
};

struct LogMsg {
  enum Type : uint8_t { SAMPLE, WINDOW, EVENT, STATUS, GAP } type;
  union {
    RawSample sample;
    WindowMsg window;
    EventMsg event;
    StatusMsg status;
    GapMsg gap;
  };
  LogMsg() : type(SAMPLE), sample() {}
};

// ---------------------------------------------------------------------------
//  Contadores (cada uno lo escribe una sola tarea; lecturas de 32 bits atómicas)
// ---------------------------------------------------------------------------
struct Stats {
  volatile uint32_t timer_ticks;     // ISR del temporizador
  volatile uint32_t samples_ok;      // muestras válidas enviadas a procesado
  volatile uint32_t missed_ticks;    // tics perdidos por la tarea de muestreo (estando OK)
  volatile uint32_t read_fail;       // lecturas I2C fallidas
  volatile uint32_t stale_reads;     // lecturas sin DATA_RDY (no se usan)
  volatile uint32_t fault_ticks;     // tics sin muestra por estar en fallo
  volatile uint32_t sample_q_full;   // muestras perdidas por cola de muestras llena
  volatile uint32_t log_q_full;      // mensajes perdidos por cola de log llena
  volatile uint32_t seq_missing;     // muestras que faltan vistas por la tarea de procesado
  volatile uint32_t windows;         // ventanas evaluadas
  volatile uint32_t windows_valid;   // ventanas completas analizadas
  volatile uint32_t episodes;        // episodios iniciados
  volatile uint32_t faults;          // entradas en estado de fallo
  volatile uint32_t max_window_us;   // peor tiempo de análisis de ventana
  volatile uint32_t max_read_us;     // peor tiempo de lectura I2C
  volatile uint32_t log_q_min_free;  // mínimo de huecos libres en la cola de log
};

extern Stats g_stats;

// Estado para el LED (lo lee la tarea de salida).
extern volatile bool g_sensor_fault;
extern volatile bool g_episode_active;
extern volatile bool g_monitoring;  // primera ventana válida ya analizada

extern QueueHandle_t g_sample_queue;  // RawSample, muestreo -> procesado
extern QueueHandle_t g_log_queue;     // LogMsg, muestreo/procesado -> salida
extern TaskHandle_t g_sampling_task;

// Envía al log sin bloquear; cuenta el mensaje si la cola está llena.
void logPush(const LogMsg& msg);

void samplingTask(void* arg);
void processingTask(void* arg);
void outputTask(void* arg);
