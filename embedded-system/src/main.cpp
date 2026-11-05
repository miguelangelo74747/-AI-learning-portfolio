// =============================================================================
//  Wrist Rest-Tremor Analyzer — Equipo 11
//  ESP32-S3-DevKitC-1 (N16R8) + MPU6050 (GY-521)
//
//  Detecta en tiempo real, en la propia placa, un temblor de reposo de muñeca
//  regular de 3.5–7 Hz, > 5 °/s RMS, durante ≥ 3 s con el antebrazo quieto
//  (condiciones C1–C4 de la propuesta). Salida: LED RGB de la placa.
//
//  Tres tareas FreeRTOS unidas por colas (Figura 1 de la propuesta):
//    samplingTask   (prioridad 20) : temporizador HW 100 Hz -> lectura I2C
//    processingTask (prioridad 10) : ventana 2.56 s, FFT, C1–C4, episodios
//    outputTask     (prioridad  5) : LED + log serie a 921 600 baudios
//  El PC solo alimenta la placa y guarda el log; no decide nada.
// =============================================================================
#include <Arduino.h>

#include "shared.h"

Stats g_stats = {};
volatile bool g_sensor_fault = false;
volatile bool g_episode_active = false;
volatile bool g_monitoring = false;

QueueHandle_t g_sample_queue = nullptr;
QueueHandle_t g_log_queue = nullptr;
TaskHandle_t g_sampling_task = nullptr;

const char* faultReasonName(FaultReason r) {
  switch (r) {
    case FaultReason::NONE: return "-";
    case FaultReason::INIT_I2C: return "INIT_I2C";
    case FaultReason::INIT_WHO_AM_I: return "INIT_WHO_AM_I";
    case FaultReason::INIT_CONFIG: return "INIT_CONFIG";
    case FaultReason::READ_FAIL: return "READ_FAIL";
    case FaultReason::DRDY_DEAD: return "DRDY_DEAD";
    case FaultReason::FROZEN: return "FROZEN";
  }
  return "?";
}

void logPush(const LogMsg& msg) {
  if (xQueueSend(g_log_queue, &msg, 0) != pdTRUE) {
    // Lo llaman dos tareas: incremento atómico.
    __atomic_fetch_add(&g_stats.log_q_full, 1u, __ATOMIC_RELAXED);
  }
}

static void halt(const char* why) {
  for (;;) {
    Serial.printf("# FATAL: %s\n", why);
    delay(1000);
  }
}

void setup() {
  Serial.setTxBufferSize(4096);  // ~45 ms de log a 921 600 baudios
  Serial.begin(SERIAL_BAUD);
  delay(200);

  g_stats.log_q_min_free = LOG_QUEUE_LEN;
  g_sample_queue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(RawSample));
  g_log_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogMsg));
  if (!g_sample_queue || !g_log_queue) halt("no hay memoria para las colas");

  // La salida primero (imprime la cabecera), después el procesado y por último
  // el muestreo, que arranca el temporizador.
  if (xTaskCreatePinnedToCore(outputTask, "output", OUTPUT_TASK_STACK, nullptr, OUTPUT_TASK_PRIORITY, nullptr,
                              OUTPUT_TASK_CORE) != pdPASS ||
      xTaskCreatePinnedToCore(processingTask, "processing", PROCESSING_TASK_STACK, nullptr,
                              PROCESSING_TASK_PRIORITY, nullptr, PROCESSING_TASK_CORE) != pdPASS ||
      xTaskCreatePinnedToCore(samplingTask, "sampling", SAMPLING_TASK_STACK, nullptr, SAMPLING_TASK_PRIORITY,
                              &g_sampling_task, SAMPLING_TASK_CORE) != pdPASS) {
    halt("no se pudieron crear las tareas");
  }
}

void loop() {
  // Todo el trabajo lo hacen las tareas; loopTask no tiene nada que hacer.
  vTaskDelete(nullptr);
}
