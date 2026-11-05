# Wrist Rest-Tremor Analyzer — Equipo 11

Detector de **temblor de reposo de muñeca** (3.5–7 Hz) que decide en tiempo real,
**en la propia ESP32-S3**, con el giróscopo de un MPU6050. Es la implementación
de la propuesta *“Wrist Rest-Tremor Analyzer: a video-verified detector,
scalable to Parkinson’s disease monitoring”* (proyecto autopropuesto).

> **Regla (sección 4 de la propuesta):** un episodio de temblor es una oscilación
> regular de la muñeca de 3.5–7 Hz, de más de 5 °/s RMS, que dura al menos 3 s
> con el antebrazo quieto. Mientras dura, el LED RGB de la placa se pone **ámbar**
> y el log registra el inicio, la frecuencia y la amplitud.

**Estado:** el firmware compila y enlaza para ESP32-S3 sin avisos (con `-Wall -Wextra`).
La parte de cálculo pasa las pruebas en el PC, y los scripts están probados con un
log simulado. **Todavía no se ha probado en la placa real.** Los umbrales son las
primeras estimaciones de la propuesta y se fijarán con vuestras medidas.

---

## Contenido

1. [Hardware y conexiones](#1-hardware-y-conexiones)
2. [Decisiones tomadas donde la propuesta no concretaba](#2-decisiones-tomadas-donde-la-propuesta-no-concretaba)
3. [Estructura del proyecto](#3-estructura-del-proyecto)
4. [Compilar y flashear](#4-compilar-y-flashear)
5. [Uso del dispositivo](#5-uso-del-dispositivo)
6. [Scripts de Python](#6-scripts-de-python)
7. [Probar sin la placa](#7-probar-sin-la-placa)
8. [De dónde sale la evidencia de cada spec](#8-de-dónde-sale-la-evidencia-de-cada-spec)
9. [Cómo funciona por dentro](#9-cómo-funciona-por-dentro)
10. [Limitaciones y pendientes](#10-limitaciones-y-pendientes)

---

## 1. Hardware y conexiones

| Pieza | Cant. | Notas |
|---|---|---|
| ESP32-S3-DevKitC-1 **N16R8** | 1 | Su LED RGB (WS2812) es la salida del dispositivo |
| MPU6050 en módulo **GY-521** | 1 | Una sola IMU: giróscopo para el temblor, acelerómetro para la inclinación |
| Cables dupont hembra–hembra, 20–30 cm | 4 (+ repuesto) | Para llevarlo en el brazo (con protoboard no se puede) |
| Cable USB-C **de datos** | 1 | En el puerto **UART** de la placa (alimentación + log) |
| Velcro o cinta elástica | 1–2 | Placa en el antebrazo, IMU cerca de la muñeca |
| Móvil con metrónomo y vídeo a 240 fps, palito de 10 cm | — | Instrumento de referencia de las pruebas |

**Conexiones (GY-521 → ESP32-S3):**

| GY-521 | ESP32-S3-DevKitC-1 | |
|---|---|---|
| VCC | 3V3 | el módulo trae regulador y resistencias de pull-up de I2C |
| GND | GND | |
| SCL | GPIO 9 | `PIN_I2C_SCL` |
| SDA | GPIO 8 | `PIN_I2C_SDA` |
| AD0 | sin conectar (o GND) | dirección I2C 0x68 |
| INT, XDA, XCL | sin conectar | la bandera de dato nuevo se lee por I2C |

**Colocación.** La IMU va en el dorso de la muñeca o en la parte distal del antebrazo,
**bien fija** (cinta de doble cara o una base rígida). Si se mueve sobre la piel,
aparecen resonancias en el espectro. La orientación da igual, porque el firmware
suma los tres ejes del giróscopo.

**No uséis el motor de vibración** como salida: está justificado en la propuesta.
Un ERM a ≈ 200 Hz aparece por aliasing dentro de la banda de temblor.

---

## 2. Decisiones tomadas donde la propuesta no concretaba

Todas están en `include/config.h` o en el código señalado, así que se pueden cambiar.

| # | Tema | Decisión | Por qué |
|---|---|---|---|
| 1 | Pin del LED RGB | **GPIO 48** (`PIN_RGB_LED`) | Es el de la DevKitC-1 **v1.0**. En la **v1.1** el LED está en **GPIO 38**: si al arrancar no se enciende azul y luego verde, cambiad el pin |
| 2 | Puerto USB | Puerto **“UART”** (puente USB-serie), `ARDUINO_USB_CDC_ON_BOOT=0`, 921 600 baudios | El log va por UART0. Así el puerto no desaparece al reiniciar la placa |
| 3 | Memoria del módulo | `qio_opi`, flash de 16 MB | Es la configuración del N16R8 (8 MB de PSRAM octal). El firmware no usa la PSRAM |
| 4 | Plataforma | `espressif32@6.9.0`, que trae Arduino-ESP32 2.0.17 | Versión fijada para que todos compilen lo mismo. El código también admite el núcleo 3.x |
| 5 | Configuración del MPU6050 | DLPF_CFG = 4 (20 Hz), SMPLRT_DIV = 0 (1 kHz interno), ±500 °/s, ±2 g | La propuesta habla del “filtro de 20 Hz” y de ±500 °/s. Con 1 kHz interno, cada lectura a 100 Hz es nueva |
| 6 | Bus I2C | 400 kHz, timeout 5 ms | Una lectura de 15 bytes tarda ≈ 0.45 ms. Con un cable suelto la espera no ocupa todo el periodo de 10 ms |
| 7 | Lectura | Un solo burst de 15 bytes desde `INT_STATUS` (0x3A) hasta `GYRO_ZOUT_L` (0x48) | La bandera DATA_RDY y los datos se leen en la misma transacción. Una lectura sin DATA_RDY se descarta y se cuenta (`stale`) |
| 8 | Qué eje se analiza | Se **suman los espectros de potencia de los 3 ejes** del giróscopo | No depende de cómo esté orientada la IMU en la muñeca |
| 9 | Espectro | Se resta la media de cada eje, ventana Hann periódica, escala de Parseval (la suma de los bins es la media cuadrática) y pico con interpolación gaussiana entre bins | Restar la media quita el sesgo del giróscopo. Las bandas dan °/s RMS reales. La frecuencia del pico tiene una resolución mucho mejor que el bin de 0.39 Hz |
| 10 | Bins de cada banda | Se incluye el bin *k* si su frecuencia *k*·0.390625 Hz cae dentro de la banda (bordes incluidos) | Regla simple y la misma en C++ y en numpy |
| 11 | Momento de cada ventana | Se evalúa cuando `(seq + 1) % 50 == 0` | Numpy puede reconstruir cada ventana solo con el `seq` final (prueba 7) |
| 12 | Muestras perdidas | Ante cualquier hueco se vacía la ventana, se reinicia el filtro de inclinación y se cierra el episodio en curso (`reason = gap`). No se inventan muestras | Así lo pide la sección 3 (“no stale value”). Si falta una muestra, el resultado de la ventana no es fiable |
| 13 | Sensor | Cada intento de arranque hace un reset del MPU6050 (`PWR_MGMT_1 = 0x80`), comprueba WHO_AM_I, configura, **relee la configuración** y activa DATA_RDY. En los reintentos también reinicia el periférico I2C | Al volver la alimentación, el MPU6050 arranca en modo sleep (W05). Repetir la secuencia completa lo recupera |
| 14 | LED | Azul hasta la primera muestra válida, luego verde, ámbar durante un episodio, rojo parpadeando a 2 Hz en fallo. Brillo 40/255 | El azul (arranque) no está en la propuesta, pero distingue “aún no mide” de “mide y no hay temblor”. A brillo máximo el WS2812 deslumbra en el vídeo |
| 15 | Tareas FreeRTOS | Muestreo: prioridad 20, núcleo 1. Procesado: 10, núcleo 1. Salida: 5, núcleo 0. Colas de 100 muestras (1 s) y 200 mensajes | El muestreo nunca espera al procesado, y el puerto serie trabaja en el otro núcleo |
| 16 | FFT | `dsps_fft2r_fc32` de **esp-dsp** (W06) si el núcleo la trae (2.0.17 sí la trae); si no, una FFT radix-2 propia | Las dos dan el mismo resultado salvo redondeo. El log dice cuál se usó (`C,FFT_BACKEND,…`) |
| 17 | Datos del episodio | Frecuencia y amplitud = media de las ventanas positivas. El log guarda también la 1.ª ventana positiva de la racha (`first_pos_seq`) | Así hay inicio, frecuencia y amplitud, como pide la tabla “Device output” |

---

## 3. Estructura del proyecto

```
embedded-system/
├── platformio.ini              entornos esp32s3 (firmware) y native (pruebas en el PC)
├── include/config.h            TODOS los pines, tiempos y umbrales, con unidad y origen
├── src/                        firmware (Arduino + FreeRTOS)
│   ├── main.cpp                colas, tareas y arranque
│   ├── sampling_task.cpp       temporizador HW 100 Hz, lectura I2C, vigilancia del sensor
│   ├── processing_task.cpp     ventana 2.56 s, FFT, C1–C4, episodios
│   ├── output_task.cpp         LED + log serie
│   ├── mpu6050.{h,cpp}         driver mínimo del MPU6050
│   ├── status_led.{h,cpp}      LED RGB de la placa
│   └── shared.h                mensajes, colas y contadores compartidos
├── lib/tremor/src/             cálculo en C++ puro (sin Arduino): se compila en la placa y en el PC
│   ├── tremor_dsp.{h,cpp}      espectro, C1–C4, filtro de inclinación, máquina de episodios
│   ├── tremor_pipeline.{h,cpp} lo que se hace con cada muestra (pasos 3–5 de la propuesta)
│   └── log_format.{h,cpp}      formato de las líneas del log
├── test/test_tremor/           pruebas unitarias (pio test -e native)
└── tools/                      scripts de Python y simulador
    ├── log_serial.py           grabar el log desde el puerto serie
    ├── analyze_log.py          informe: Spec 01–04, episodios, prueba 7, ensayos
    ├── plot_log.py             gráficas del log y espectro de una ventana
    ├── tremorlog.py            lector del log (módulo)
    ├── tremor_ref.py           mismo algoritmo en numpy (módulo)
    ├── requirements.txt
    └── host_sim/sim_log.cpp    genera un log simulado con el código real de lib/tremor
```

---

## 4. Compilar y flashear

### Requisitos

* **PlatformIO**: la extensión *PlatformIO IDE* de VS Code, o la línea de comandos:

```bash
pip install platformio
```

* La primera compilación descarga la plataforma `espressif32@6.9.0` (unos cientos de MB).

### Compilar

Desde la carpeta `embedded-system/`:

```bash
pio run -e esp32s3
```

### Flashear

Conectad el cable al puerto USB marcado **UART** y:

```bash
pio run -e esp32s3 -t upload
```

Si no lo encuentra solo, indicad el puerto: `/dev/ttyUSB0` o `/dev/ttyACM0` en Linux,
`/dev/cu.usbserial-*` o `/dev/cu.wchusbserial*` en macOS, `COM5` en Windows.

```bash
pio run -e esp32s3 -t upload --upload-port /dev/ttyUSB0
```

Si la subida falla con *“Failed to connect”*, mantened pulsado **BOOT**, pulsad y soltad
**RESET**, soltad **BOOT** y repetid la subida.

### Ver el log con PlatformIO

```bash
pio device monitor -e esp32s3
```

El `platformio.ini` ya activa los filtros `time` (hora del PC en cada línea) y `log2file`
(guarda `platformio-device-monitor-*.log`). Los scripts de Python leen ese archivo
directamente. Para grabar ensayos es más cómodo `tools/log_serial.py` ([sección 6](#6-scripts-de-python)).

### Pruebas unitarias en el PC

```bash
pio test -e native
```

Hay 13 pruebas con señales sintéticas: cada condición C1–C4, la escala de Parseval, el filtro de
inclinación, la máquina de episodios (6 ventanas para empezar, 4 para terminar) y los huecos de muestras.
Necesitan un compilador de C++ en el PC (gcc/clang; en Windows, MinGW).

---

## 5. Uso del dispositivo

| LED | Significado |
|---|---|
| Azul fijo | Arrancando: todavía no hay ninguna muestra válida |
| **Verde fijo** | Monitorizando, sin episodio |
| **Ámbar fijo** | Episodio de temblor: desde que se cumple la regla hasta 2 s sin cumplirla |
| **Rojo parpadeando a 2 Hz** | Fallo del sensor: WHO_AM_I ≠ 0x68, configuración mal leída, 10 lecturas fallidas seguidas, DATA_RDY muerto o giróscopo congelado 1 s. Reintenta cada 1 s y vuelve a verde tras 10 lecturas buenas |

Tiempos que se esperan (medidos con el simulador): las métricas se calculan sobre los últimos
2.56 s, así que el ámbar se enciende ≈ 3.5–4 s después de empezar una oscilación clara y
se apaga ≈ 3.5 s después de que termine (la ventana tarda en vaciarse y luego hacen falta 2 s sin temblor).

**Teclas por el puerto serie** (desde `pio device monitor` o `log_serial.py`):

| Tecla | Acción |
|---|---|
| `m` | escribe una marca `M` en el log (inicio de un ensayo) |
| `r` | imprime ahora el informe de contadores `R` |
| `h` | vuelve a imprimir la cabecera de configuración |

### Formato del log

Una línea por mensaje, con los campos separados por comas.

| Línea | Campos | Cuándo |
|---|---|---|
| `C` | `C,clave,valor` | al arrancar: todos los umbrales de `config.h` |
| `S` | `S,seq,t_us,ax,ay,az,gx,gy,gz` (cuentas crudas del sensor) | cada muestra (100 por segundo) |
| `W` | `W,seq_end,t_us,valid,peak_hz,tremor_dps,conc,move_dps,tilt_deg,c1,c2,c3,c4,pos_streak,episode,proc_us` | cada 0.5 s |
| `E` | `E,START\|END,id,first_pos_seq,seq,t_us,mean_hz,mean_dps,max_dps,pos_windows,total_windows,reason` | inicio y fin de cada episodio |
| `F` | `F,FAULT\|RETRY_FAIL\|RECOVER,motivo,seq,t_us,detalle` | estado del sensor |
| `G` | `G,seq,t_us,muestras_perdidas` | hueco en la numeración |
| `L` | `L,t_us,color` | cambio de color del LED |
| `R` | `R,t_us,ticks=…,samples=…,missed_ticks=…,read_fail=…,…` | cada 10 s y con la tecla `r` |
| `M` | `M,t_us,n` | marca manual |

`seq` es el número de tic del temporizador. Avanza aunque no haya muestra, así que **cualquier
muestra perdida deja un hueco visible**. `t_us` es `esp_timer_get_time()` al empezar la lectura I2C.

---

## 6. Scripts de Python

Instalación (Python 3.9 o superior):

```bash
pip install -r tools/requirements.txt
```

### Grabar: `log_serial.py`

```bash
python tools/log_serial.py --list
```

```bash
python tools/log_serial.py -p /dev/ttyUSB0 --reset --duration 600 -o logs/run10min.log
```

* `--reset` reinicia la placa al abrir el puerto, así el log empieza con la cabecera `C`.
* Cada línea se guarda con la hora del PC delante: es el reloj secundario de la Spec 01.
* **Enter** envía una marca de ensayo (`M`) y **Ctrl+C** termina. Antes de cerrar pide un último informe `R`.
* Sin `--duration` graba hasta Ctrl+C.

### Analizar: `analyze_log.py`

```bash
python tools/analyze_log.py logs/run10min.log
```

Imprime:

* **Spec 01**: muestras / tiempo, media y d.t. del intervalo entre muestras, y la pendiente frente al reloj del PC (ppm).
* **Spec 04**: muestras esperadas, recibidas y perdidas, la **tasa de pérdida**, los huecos y el último informe `R` de la placa.
* **Spec 02**: porcentaje de ventanas que cumplen cada condición y tiempo de análisis por ventana.
* **Spec 03**: episodios (inicio, duración, frecuencia, amplitud), episodios por hora, fallos del sensor con el tiempo hasta el LED rojo, y cambios del LED.
* **Prueba 7**: recomputa en numpy cada ventana completa y la compara con la placa (pico ±0.01 Hz, amplitud ±1 %, condiciones y número de episodios).

Opciones:

```bash
python tools/analyze_log.py logs/prueba1_4hz.log --by-marker
```

`--by-marker` separa los ensayos por las marcas `M`. Da, para cada uno, si hubo detección,
la latencia desde la marca y la frecuencia y amplitud del episodio, y al final el porcentaje
de detección con **IC 95 % de Wilson**.

```bash
python tools/analyze_log.py logs/run10min.log --json logs/run10min_summary.json
```

* `--json` guarda el resumen para la presentación.
* `--last-boot` descarta lo anterior al último reinicio, si la placa se reinició a mitad del log.
* `--no-recompute` se salta la prueba 7 y va más rápido.

También acepta los logs de `pio device monitor` (filtro `time` + `log2file`).

### Graficar: `plot_log.py`

```bash
python tools/plot_log.py logs/prueba1_4hz.log
```

```bash
python tools/plot_log.py logs/prueba1_4hz.log --t0 20 --t1 60 --out figs/prueba1_zoom.png
```

```bash
python tools/plot_log.py logs/prueba1_4hz.log --spectrum 4049
```

* La figura principal tiene seis paneles con el mismo eje de tiempo: giróscopo, C1, C2, C3 y los dos de C4.
  Cada métrica aparece con su umbral, los episodios en ámbar, los fallos en rojo y las marcas `M`.
  Las cruces grises son los valores recomputados en numpy (`--no-ref` para quitarlas).
* `--spectrum SEQ` dibuja el espectro de la ventana que termina en `SEQ` (el primer campo de una línea `W`),
  con las bandas de movimiento, de temblor y ±1 Hz del pico.

---

## 7. Probar sin la placa

`tools/host_sim/sim_log.cpp` genera un log con **el mismo código de `lib/tremor`** y el mismo formato
que la placa, a partir de señales sintéticas. El guion dura 200 s: reposo, temblor de 5 Hz, temblor de 4 Hz,
oscilación de 2 Hz, brazo extendido a 9.5 Hz, escritura, un cable SDA suelto durante 5 s y reposo.
Sirve para aprender a usar los scripts y saber qué esperar. **No sustituye a las medidas reales:**
los tiempos son simulados.

```bash
g++ -std=gnu++17 -O2 -Iinclude -Ilib/tremor/src tools/host_sim/sim_log.cpp lib/tremor/src/*.cpp -o sim_log
```

```bash
mkdir -p logs && ./sim_log > logs/sim.log
```

```bash
python tools/analyze_log.py logs/sim.log --by-marker
```

```bash
python tools/plot_log.py logs/sim.log
```

Resultado esperado: 2 episodios (5 Hz y 4 Hz) y ninguno con 2 Hz, 9.5 Hz ni escritura;
1 fallo del sensor con 525 muestras perdidas; la placa y numpy coinciden en todas las ventanas.

---

## 8. De dónde sale la evidencia de cada spec

| Spec | Qué se pide | Evidencia |
|---|---|---|
| **01** Sensor ≥ 50 Hz | Frecuencia **medida** | `analyze_log.py` → “Spec 01”: muestras / tiempo, intervalos (media, d.t.) y comparación con el reloj del PC |
| **02** Decisión en la placa | La ESP32 decide mientras llegan los datos | C1–C4 cada 0.5 s en `processing_task.cpp` (líneas `W` con `proc_us`). El PC solo graba. La prueba 7 demuestra que la placa calculó lo mismo que numpy |
| **03** Regla con número y unidad + salida en el dispositivo | LED | Constantes con unidad en `config.h` (`TREMOR_MIN_DPS = 5.0 °/s RMS`, …). LED verde, ámbar y rojo, con cada cambio registrado en una línea `L` |
| **04** 10 min sin caídas ni pérdidas | Tasa de pérdida a partir del log crudo | `log_serial.py --reset --duration 600` y luego `analyze_log.py` → “Spec 04” (esperadas 60 000). Los contadores de la placa salen en las líneas `R` |

Las pruebas de la sección 6 de la propuesta se hacen igual: un log por prueba, **Enter al empezar cada
ensayo** y `analyze_log.py --by-marker`. La latencia real (inicio → ámbar) se mide en el vídeo a 240 fps con
el LED en el plano. Las líneas `L` del log ayudan a sincronizar.

---

## 9. Cómo funciona por dentro

```
temporizador HW (10 000 µs) ──notificación──▶ samplingTask (prioridad 20)
                                              lectura I2C de 15 bytes, seq + t_us,
                                              vigilancia del sensor
                                                    │ sampleQueue (100)
                                                    ▼
                                              processingTask (prioridad 10)
                                              °/s y g → búfer de 256 → cada 50 muestras:
                                              FFT × 3 ejes → pico, C1–C4 → episodio
                                                    │ logQueue (200)
                                                    ▼
                                              outputTask (prioridad 5, núcleo 0)
                                              LED RGB + líneas del log a 921 600 baudios
```

Para cada ventana de 256 muestras (2.56 s, resolución de 0.39 Hz):

1. Se resta la media de cada eje del giróscopo, se aplica la ventana Hann, se hace la FFT y se suman las potencias de los 3 ejes.
2. **C1**: el bin más fuerte entre 0.5 y 15 Hz, con interpolación gaussiana, debe caer en 3.5–7 Hz.
3. **C2**: el RMS de la banda 3.5–7 Hz debe ser ≥ 5 °/s.
4. **C3**: al menos el 50 % de la potencia de 0.5–15 Hz debe estar a ±1 Hz del pico.
5. **C4**: el RMS de la banda 0.5–3 Hz debe ser < 20 °/s. Además, el ángulo máximo entre la gravedad más reciente
   (acelerómetro tras un biquad RBJ de 0.5 Hz) y cualquier otra de la ventana debe ser < 10°.
6. Un episodio empieza con 6 ventanas positivas seguidas (3 s) y termina con 4 negativas seguidas (2 s).

---

## 10. Limitaciones y pendientes

* **No se ha probado aún en hardware.** El primer paso es flashear y comprobar el LED (pin 48 o 38, ver la decisión 1),
  la lectura del sensor (cabecera `C` y líneas `S`) y la Spec 04 con una grabación de 10 min.
* `TREMOR_MIN_DPS`, `MOVE_MAX_DPS`, `REST_MAX_TILT_DEG`, `PEAK_CONCENTRATION_MIN`, `TREMOR_MIN_DURATION_S` y
  `EPISODE_RELEASE_S` son primeras estimaciones de la propuesta. Según el plan, `TREMOR_MIN_DPS` se fija con
  las grabaciones de reposo de 5 × 60 s.
* Un solo hueco de muestras cierra el episodio en curso (decisión 12). Con el objetivo de 0 % de pérdidas no
  debería ocurrir, pero si ocurre aparece en el log como `reason = gap`.
* En la comparación con numpy, la inclinación es orientativa: si el log no empieza en el arranque, el filtro de la placa
  arrastra historia que el log no tiene. La frecuencia y las amplitudes no dependen de eso.
* Las pruebas con pacientes quedan fuera del curso (sección 1 de la propuesta).

## Referencias

1. N. A. Timmermans *et al.*, “A generalizable and open-source algorithm for real-life monitoring of tremor in
   Parkinson’s disease,” *npj Parkinson’s Disease* 11, 205 (2025). doi:10.1038/s41531-025-01056-2
2. S. Morrison *et al.*, “Variability, regularity and coupling measures distinguish PD tremor from voluntary 5 Hz
   tremor,” *Neuroscience Letters* 534, 69–74 (2013). doi:10.1016/j.neulet.2012.11.040
3. InvenSense, *MPU-6000/MPU-6050 Register Map and Descriptions*, rev. 4.2.
