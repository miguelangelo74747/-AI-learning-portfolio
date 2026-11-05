// =============================================================================
//  config.h — Wrist Rest-Tremor Analyzer (Equipo 11)
//
//  Todos los pines, tiempos y umbrales del proyecto en un solo sitio.
//  Cada constante lleva la unidad en el nombre o en el comentario, y su origen
//  (medida propia, datasheet, artículo o primera estimación a comprobar),
//  tal como pide la propuesta (sección 4, "Rule parameters").
//
//  Referencias:
//   [1] Timmermans et al., npj Parkinson's Disease 11, 205 (2025).
//   [2] Morrison et al., Neuroscience Letters 534, 69–74 (2013).
//   [DS] MPU-6000/MPU-6050 Register Map and Descriptions, rev. 4.2 (RM-MPU-6000A-00).
// =============================================================================
#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
//  Versión del firmware (aparece en la cabecera del log)
// -----------------------------------------------------------------------------
#define FW_VERSION "1.0.0"
#define TEAM_NUMBER 11

// -----------------------------------------------------------------------------
//  Pines (ESP32-S3-DevKitC-1 N16R8)
// -----------------------------------------------------------------------------
constexpr int PIN_I2C_SDA = 8;   // GY-521 SDA -> GPIO 8 (propuesta, sección 9)
constexpr int PIN_I2C_SCL = 9;   // GY-521 SCL -> GPIO 9 (propuesta, sección 9)
// LED RGB direccionable (WS2812) de la placa. GPIO 48 en la DevKitC-1 v1.0;
// en la revisión v1.1 está en GPIO 38. Elección razonable documentada en el README.
constexpr int PIN_RGB_LED = 48;
constexpr uint8_t LED_BRIGHTNESS = 40;  // 0–255; el WS2812 a 255 deslumbra en el vídeo

// -----------------------------------------------------------------------------
//  Bus I2C y MPU6050
// -----------------------------------------------------------------------------
constexpr uint32_t I2C_CLOCK_HZ = 400000;     // modo rápido; lectura de 15 bytes ≈ 0.45 ms
constexpr uint16_t I2C_TIMEOUT_MS = 5;        // < 10 ms: un cable suelto no bloquea el periodo
constexpr uint8_t MPU_ADDR = 0x68;            // AD0 a GND (escáner I2C de W05)
constexpr uint8_t MPU_WHO_AM_I_EXPECTED = 0x68;  // [DS] p. 45

// Configuración del sensor ([DS] pp. 11–15):
//  DLPF_CFG = 4  -> filtro paso bajo de 20 Hz (giro) / 21 Hz (acel.), retardo 8.3 ms
//  SMPLRT_DIV = 0 -> con DLPF activo, salida interna a 1 kHz: cada lectura es nueva
//  FS_SEL = 1   -> ±500 °/s, 65.5 LSB/(°/s)
//  AFS_SEL = 0  -> ±2 g, 16384 LSB/g
constexpr uint8_t MPU_DLPF_CFG = 4;
constexpr uint8_t MPU_SMPLRT_DIV = 0;
constexpr uint8_t MPU_GYRO_FS_SEL = 1;
constexpr uint8_t MPU_ACCEL_AFS_SEL = 0;
constexpr float GYRO_LSB_PER_DPS = 65.5f;     // [DS] p. 31, FS_SEL = 1
constexpr float ACCEL_LSB_PER_G = 16384.0f;   // [DS] p. 29, AFS_SEL = 0

// -----------------------------------------------------------------------------
//  Muestreo (Spec 01)
// -----------------------------------------------------------------------------
constexpr float SAMPLE_RATE_HZ = 100.0f;      // propuesta, sección 2
constexpr uint32_t SAMPLE_PERIOD_US = 10000;  // temporizador hardware: 10 000 µs

// -----------------------------------------------------------------------------
//  Ventana de análisis (Spec 02)
// -----------------------------------------------------------------------------
constexpr int WINDOW_SAMPLES = 256;           // 2.56 s a 100 Hz; resolución 0.390625 Hz
constexpr int HOP_SAMPLES = 50;               // regla evaluada cada 0.5 s
constexpr float WINDOW_HOP_S = HOP_SAMPLES / SAMPLE_RATE_HZ;

// -----------------------------------------------------------------------------
//  Regla de detección C1–C4 (propuesta, sección 4)
// -----------------------------------------------------------------------------
// C1 frecuencia: pico más fuerte del espectro 0.5–15 Hz dentro de 3.5–7 Hz.
constexpr float TREMOR_BAND_LOW_HZ = 3.5f;    // [1] (3–7 Hz) + 0.5 Hz de margen frente a la banda de movimiento
constexpr float TREMOR_BAND_HIGH_HZ = 7.0f;   // [1]; temblor fisiológico medido en W06 con pico en 9.38 Hz
// C2 amplitud: banda 3.5–7 Hz >= 5 °/s RMS.
constexpr float TREMOR_MIN_DPS = 5.0f;        // primera estimación: 3 × p99 del ruido con el antebrazo en reposo
// C3 regularidad: >= 50 % de la potencia 0.5–15 Hz a ±1 Hz del pico.
constexpr float PEAK_HALF_WIDTH_HZ = 1.0f;    // fuga espectral (W06: un solo bin leía un 12 % bajo con Hann)
constexpr float PEAK_CONCENTRATION_MIN = 0.50f;  // fracción; seno puro ≈ 1, movimiento de banda ancha ≈ 0.14
constexpr float CONC_REF_LOW_HZ = 0.5f;       // banda de referencia, borde inferior (= borde inferior del espectro de C1)
constexpr float CONC_REF_HIGH_HZ = 15.0f;     // cubre 8–12 Hz y queda bajo el DLPF de 20 Hz
// C4 reposo: banda 0.5–3 Hz < 20 °/s RMS y cambio de inclinación < 10°.
constexpr float MOVE_BAND_LOW_HZ = 0.5f;      // [1]
constexpr float MOVE_BAND_HIGH_HZ = 3.0f;     // [1]
constexpr float MOVE_MAX_DPS = 20.0f;         // primera estimación, fijada con registros propios
constexpr float REST_MAX_TILT_DEG = 10.0f;    // primera estimación; temblor severo (100 °/s RMS, 5 Hz) ≈ ±4.5°
constexpr float TILT_LPF_HZ = 0.5f;           // biquad RBJ 2.º orden (W06); deja pasar ≈ 2 % a 3.5 Hz
constexpr float TILT_LPF_Q = 0.7071f;         // Butterworth

// Duración de episodio
constexpr float TREMOR_MIN_DURATION_S = 3.0f; // 6 ventanas positivas consecutivas
constexpr float EPISODE_RELEASE_S = 2.0f;     // 4 ventanas negativas consecutivas
constexpr int TREMOR_MIN_WINDOWS = (int)(TREMOR_MIN_DURATION_S / WINDOW_HOP_S + 0.5f);  // = 6
constexpr int EPISODE_RELEASE_WINDOWS = (int)(EPISODE_RELEASE_S / WINDOW_HOP_S + 0.5f); // = 4

// -----------------------------------------------------------------------------
//  Detección de fallo del sensor (propuesta, tabla "Device output")
// -----------------------------------------------------------------------------
constexpr int FAULT_READ_FAILS = 10;          // 10 lecturas fallidas seguidas = 100 ms
constexpr int FAULT_DRDY_DEAD_READS = 10;     // bandera DATA_RDY a 0 en 10 lecturas seguidas = 100 ms
constexpr int FAULT_FROZEN_SAMPLES = 100;     // giróscopo idéntico durante 1 s (W05, Sketch 4)
constexpr uint32_t FAULT_RETRY_PERIOD_MS = 1000;  // reintento de arranque cada 1 s
constexpr int RECOVER_GOOD_READS = 10;        // 10 lecturas buenas seguidas -> verde de nuevo
constexpr uint32_t FAULT_BLINK_HALF_PERIOD_MS = 250;  // parpadeo rojo a 2 Hz (250 ms on / 250 ms off)

// -----------------------------------------------------------------------------
//  FreeRTOS: tareas y colas (Figura 1 de la propuesta)
// -----------------------------------------------------------------------------
constexpr int SAMPLING_TASK_PRIORITY = 20;    // la más alta de las tres
constexpr int PROCESSING_TASK_PRIORITY = 10;
constexpr int OUTPUT_TASK_PRIORITY = 5;
constexpr int SAMPLING_TASK_CORE = 1;
constexpr int PROCESSING_TASK_CORE = 1;
constexpr int OUTPUT_TASK_CORE = 0;           // Serial en el otro núcleo
constexpr uint32_t SAMPLING_TASK_STACK = 4096;   // bytes
constexpr uint32_t PROCESSING_TASK_STACK = 8192; // bytes (FFT y espectros viven en estáticos)
constexpr uint32_t OUTPUT_TASK_STACK = 6144;     // bytes (snprintf con %f usa ~1.5 KB de pila)
constexpr int SAMPLE_QUEUE_LEN = 100;         // 1 s de margen entre muestreo y procesado
constexpr int LOG_QUEUE_LEN = 200;            // 2 s de margen hacia el puerto serie

// -----------------------------------------------------------------------------
//  Log serie
// -----------------------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 921600;      // propuesta, sección 5
constexpr uint32_t REPORT_PERIOD_MS = 10000;  // línea "R" con contadores cada 10 s
constexpr bool LOG_RAW_SAMPLES = true;        // línea "S" por muestra (necesaria para Spec 04 y la prueba 7)
