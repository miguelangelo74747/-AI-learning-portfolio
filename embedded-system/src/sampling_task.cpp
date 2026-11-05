// =============================================================================
//  sampling_task.cpp — pasos 1 y 2 de la propuesta (sección 3).
//
//  1. Un temporizador hardware dispara cada 10 ms (100 Hz) y despierta esta
//     tarea, que tiene la prioridad más alta, con una notificación.
//  2. La tarea lee el giróscopo y el acelerómetro por I2C, comprueba que la
//     lectura es buena y le pone número de muestra y marca de tiempo. Una
//     lectura fallida se cuenta; nunca se sustituye por un valor viejo.
//
//  También vigila el sensor (lecturas fallidas, DATA_RDY muerto, giróscopo
//  congelado) y reintenta el arranque cada 1 s mientras está en fallo.
// =============================================================================
#include <Wire.h>
#include <esp_timer.h>

#include "mpu6050.h"
#include "shared.h"

namespace {

hw_timer_t* s_timer = nullptr;

void IRAM_ATTR onSampleTimer() {
  g_stats.timer_ticks = g_stats.timer_ticks + 1;
  BaseType_t hp_woken = pdFALSE;
  vTaskNotifyGiveFromISR(g_sampling_task, &hp_woken);
  if (hp_woken == pdTRUE) portYIELD_FROM_ISR();
}

void startSampleTimer() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  s_timer = timerBegin(1000000);  // 1 MHz -> 1 tic = 1 µs
  timerAttachInterrupt(s_timer, &onSampleTimer);
  timerAlarm(s_timer, SAMPLE_PERIOD_US, true, 0);
#else
  s_timer = timerBegin(0, 80, true);  // APB 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(s_timer, &onSampleTimer, true);
  timerAlarmWrite(s_timer, SAMPLE_PERIOD_US, true);
  timerAlarmEnable(s_timer);
#endif
}

void startI2C() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
}

void pushStatus(StatusMsg::Kind kind, FaultReason reason, uint32_t seq, uint8_t detail = 0) {
  LogMsg msg;
  msg.type = LogMsg::STATUS;
  msg.status.kind = kind;
  msg.status.reason = reason;
  msg.status.seq = seq;
  msg.status.t_us = esp_timer_get_time();
  msg.status.detail = detail;
  logPush(msg);
}

FaultReason initReasonOf(mpu::InitResult r) {
  switch (r) {
    case mpu::InitResult::NO_ACK: return FaultReason::INIT_I2C;
    case mpu::InitResult::BAD_WHO_AM_I: return FaultReason::INIT_WHO_AM_I;
    case mpu::InitResult::BAD_CONFIG: return FaultReason::INIT_CONFIG;
    default: return FaultReason::NONE;
  }
}

}  // namespace

void samplingTask(void*) {
  g_sampling_task = xTaskGetCurrentTaskHandle();  // antes de arrancar el temporizador
  mpu::Mpu6050 imu(Wire, MPU_ADDR);
  startI2C();

  // Estado del vigilante del sensor.
  bool configured = false;      // la secuencia de arranque terminó bien
  bool in_fault = false;        // LED rojo
  int consecutive_fail = 0;     // lecturas I2C fallidas seguidas
  int drdy_dead = 0;            // lecturas seguidas sin DATA_RDY
  int frozen_run = 1;           // longitud de la racha de giróscopo idéntico
  int good_reads = 0;           // lecturas buenas seguidas (para recuperar)
  int16_t last_g[3] = {0, 0, 0};
  bool have_last_g = false;
  uint32_t last_retry_ms = 0;

  auto enterFault = [&](FaultReason reason, uint32_t seq, uint8_t detail) {
    if (!in_fault) {
      in_fault = true;
      g_sensor_fault = true;
      g_stats.faults = g_stats.faults + 1;
      pushStatus(StatusMsg::FAULT, reason, seq, detail);
    } else {
      pushStatus(StatusMsg::RETRY_FAIL, reason, seq, detail);
    }
    configured = false;
    good_reads = 0;
    last_retry_ms = millis();
  };

  auto resetWatchdogs = [&]() {
    consecutive_fail = 0;
    drdy_dead = 0;
    frozen_run = 1;
    have_last_g = false;
  };

  // Arranque inicial (comprobaciones WHO_AM_I y configuración).
  {
    const mpu::InitResult r = imu.begin();
    if (r == mpu::InitResult::OK) {
      configured = true;
    } else {
      enterFault(initReasonOf(r), 0, imu.lastWhoAmI());
    }
  }

  uint32_t next_seq = 0;
  startSampleTimer();

  for (;;) {
    // Espera al temporizador. Si se acumulan varias notificaciones es que se
    // ha perdido algún tic: el número de muestra avanza igualmente y el hueco
    // queda visible en el log.
    const uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    if (n == 0) continue;  // no debería ocurrir: el temporizador se ha parado
    const uint32_t seq = next_seq + (n - 1);
    next_seq = seq + 1;
    if (n > 1) {
      if (in_fault) g_stats.fault_ticks = g_stats.fault_ticks + (n - 1);
      else g_stats.missed_ticks = g_stats.missed_ticks + (n - 1);
    }

    // --- En fallo y sin configurar: reintentar el arranque cada 1 s --------
    if (!configured) {
      g_stats.fault_ticks = g_stats.fault_ticks + 1;
      if (millis() - last_retry_ms >= FAULT_RETRY_PERIOD_MS) {
        last_retry_ms = millis();
        Wire.end();  // reinicia el periférico por si el bus quedó colgado
        startI2C();
        const mpu::InitResult r = imu.begin();
        if (r == mpu::InitResult::OK) {
          configured = true;
          good_reads = 0;
          resetWatchdogs();
        } else {
          pushStatus(StatusMsg::RETRY_FAIL, initReasonOf(r), seq, imu.lastWhoAmI());
        }
      }
      continue;
    }

    // --- Lectura ------------------------------------------------------------
    RawSample s;
    s.seq = seq;
    s.t_us = esp_timer_get_time();
    mpu::Reading r;
    const bool ok = imu.read(r);
    const uint32_t read_us = (uint32_t)(esp_timer_get_time() - s.t_us);
    if (read_us > g_stats.max_read_us) g_stats.max_read_us = read_us;

    if (!ok) {
      g_stats.read_fail = g_stats.read_fail + 1;
      if (in_fault) g_stats.fault_ticks = g_stats.fault_ticks + 1;
      good_reads = 0;
      if (++consecutive_fail >= FAULT_READ_FAILS) {
        enterFault(FaultReason::READ_FAIL, seq, 0);
        resetWatchdogs();
      }
      continue;
    }
    consecutive_fail = 0;

    if (!r.data_ready) {
      // Dato repetido: no se envía (sería un valor viejo).
      g_stats.stale_reads = g_stats.stale_reads + 1;
      if (in_fault) g_stats.fault_ticks = g_stats.fault_ticks + 1;
      good_reads = 0;
      if (++drdy_dead >= FAULT_DRDY_DEAD_READS) {
        enterFault(FaultReason::DRDY_DEAD, seq, 0);
        resetWatchdogs();
      }
      continue;
    }
    drdy_dead = 0;

    // Giróscopo congelado: los tres ejes idénticos durante 1 s. Con el sensor
    // vivo el ruido cambia siempre algún LSB.
    if (have_last_g && r.gx == last_g[0] && r.gy == last_g[1] && r.gz == last_g[2]) {
      if (++frozen_run >= FAULT_FROZEN_SAMPLES) {
        enterFault(FaultReason::FROZEN, seq, 0);
        resetWatchdogs();
        continue;
      }
    } else {
      frozen_run = 1;
    }
    last_g[0] = r.gx;
    last_g[1] = r.gy;
    last_g[2] = r.gz;
    have_last_g = true;

    // --- Muestra buena ---------------------------------------------------------
    s.ax = r.ax;
    s.ay = r.ay;
    s.az = r.az;
    s.gx = r.gx;
    s.gy = r.gy;
    s.gz = r.gz;
    if (xQueueSend(g_sample_queue, &s, 0) == pdTRUE) {
      g_stats.samples_ok = g_stats.samples_ok + 1;
    } else {
      g_stats.sample_q_full = g_stats.sample_q_full + 1;
    }

    if (in_fault && ++good_reads >= RECOVER_GOOD_READS) {
      in_fault = false;
      g_sensor_fault = false;
      pushStatus(StatusMsg::RECOVER, FaultReason::NONE, seq);
    }
  }
}
