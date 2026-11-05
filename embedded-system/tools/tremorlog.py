"""Lectura del log serie del Wrist Rest-Tremor Analyzer.

Acepta tres variantes de la misma línea del firmware:

    S,123,1230000,...                       línea tal cual (salida del firmware)
    1793845956.123456<TAB>S,123,...          guardada por log_serial.py
    14:05:31.123 > S,123,...                 PlatformIO monitor (filtro "time")

Uso como módulo:

    from tremorlog import load_log
    log = load_log("logs/run.log")
    log.samples["gx"], log.windows["peak_hz"], log.events, log.reports ...
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# Valores por defecto (iguales a include/config.h). Si el log trae líneas
# "C,clave,valor" de la cabecera, esas mandan.
DEFAULT_CONFIG: dict[str, float] = {
    "SAMPLE_RATE_HZ": 100.0,
    "SAMPLE_PERIOD_US": 10000,
    "WINDOW_SAMPLES": 256,
    "HOP_SAMPLES": 50,
    "GYRO_LSB_PER_DPS": 65.5,
    "ACCEL_LSB_PER_G": 16384.0,
    "TREMOR_BAND_LOW_HZ": 3.5,
    "TREMOR_BAND_HIGH_HZ": 7.0,
    "TREMOR_MIN_DPS": 5.0,
    "PEAK_HALF_WIDTH_HZ": 1.0,
    "PEAK_CONCENTRATION_MIN": 0.5,
    "CONC_REF_LOW_HZ": 0.5,
    "CONC_REF_HIGH_HZ": 15.0,
    "MOVE_BAND_LOW_HZ": 0.5,
    "MOVE_BAND_HIGH_HZ": 3.0,
    "MOVE_MAX_DPS": 20.0,
    "REST_MAX_TILT_DEG": 10.0,
    "TILT_LPF_HZ": 0.5,
    "TILT_LPF_Q": 0.7071,
    "TREMOR_MIN_DURATION_S": 3.0,
    "EPISODE_RELEASE_S": 2.0,
    "TREMOR_MIN_WINDOWS": 6,
    "EPISODE_RELEASE_WINDOWS": 4,
}

_HOST_UNIX = re.compile(r"^(\d+\.\d+)\t(.*)$")
_HOST_CLOCK = re.compile(r"^(\d{2}):(\d{2}):(\d{2})\.(\d{3}) > (.*)$")

SAMPLE_DTYPE = np.dtype(
    [
        ("seq", np.int64),
        ("t_us", np.int64),
        ("ax", np.int16),
        ("ay", np.int16),
        ("az", np.int16),
        ("gx", np.int16),
        ("gy", np.int16),
        ("gz", np.int16),
        ("host_t", np.float64),  # NaN si el log no trae hora del PC
    ]
)

WINDOW_DTYPE = np.dtype(
    [
        ("seq_end", np.int64),
        ("t_us", np.int64),
        ("valid", np.bool_),
        ("peak_hz", np.float64),
        ("tremor_dps", np.float64),
        ("conc", np.float64),
        ("move_dps", np.float64),
        ("tilt_deg", np.float64),
        ("c1", np.bool_),
        ("c2", np.bool_),
        ("c3", np.bool_),
        ("c4", np.bool_),
        ("pos_streak", np.int32),
        ("episode", np.bool_),
        ("proc_us", np.int64),
    ]
)


@dataclass
class TremorLog:
    path: Path
    config: dict[str, float | str] = field(default_factory=lambda: dict(DEFAULT_CONFIG))
    samples: np.ndarray = field(default_factory=lambda: np.zeros(0, SAMPLE_DTYPE))
    windows: np.ndarray = field(default_factory=lambda: np.zeros(0, WINDOW_DTYPE))
    events: list[dict] = field(default_factory=list)   # líneas E
    status: list[dict] = field(default_factory=list)   # líneas F
    gaps: list[dict] = field(default_factory=list)     # líneas G
    reports: list[dict] = field(default_factory=list)  # líneas R
    markers: list[dict] = field(default_factory=list)  # líneas M
    led: list[dict] = field(default_factory=list)      # líneas L
    comments: list[str] = field(default_factory=list)  # líneas #
    boots: int = 0          # cabeceras encontradas (reinicios de la placa)
    malformed: int = 0      # líneas que no se pudieron interpretar
    total_lines: int = 0

    def cfg(self, key: str) -> float:
        return float(self.config.get(key, DEFAULT_CONFIG[key]))

    @property
    def fs(self) -> float:
        return self.cfg("SAMPLE_RATE_HZ")

    def gyro_dps(self) -> np.ndarray:
        """Matriz (n, 3) del giróscopo en °/s."""
        s = self.samples
        return np.stack([s["gx"], s["gy"], s["gz"]], axis=1).astype(np.float64) / self.cfg("GYRO_LSB_PER_DPS")

    def accel_g(self) -> np.ndarray:
        """Matriz (n, 3) del acelerómetro en g."""
        s = self.samples
        return np.stack([s["ax"], s["ay"], s["az"]], axis=1).astype(np.float64) / self.cfg("ACCEL_LSB_PER_G")

    def t0_us(self) -> int:
        """Origen de tiempos para las gráficas: la primera muestra (o ventana)."""
        if len(self.samples):
            return int(self.samples["t_us"][0])
        if len(self.windows):
            return int(self.windows["t_us"][0])
        return 0


def _split_prefix(line: str) -> tuple[float, str]:
    m = _HOST_UNIX.match(line)
    if m:
        return float(m.group(1)), m.group(2)
    m = _HOST_CLOCK.match(line)
    if m:
        h, mi, s, ms = (int(m.group(i)) for i in range(1, 5))
        return h * 3600 + mi * 60 + s + ms / 1000.0, m.group(5)
    return float("nan"), line


def _num(text: str) -> float | str:
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return text


def load_log(path: str | Path, keep_last_boot: bool = False) -> TremorLog:
    """Lee un log completo.

    keep_last_boot=True descarta todo lo anterior a la última cabecera
    (útil si la placa se reinició durante la grabación).
    """
    path = Path(path)
    log = TremorLog(path=path)
    samples: list[tuple] = []
    windows: list[tuple] = []

    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            log.total_lines += 1
            raw = raw.rstrip("\r\n")
            if not raw:
                continue
            host_t, line = _split_prefix(raw)
            line = line.strip()
            if not line:
                continue
            tag = line[0]
            try:
                if tag == "#":
                    log.comments.append(line[1:].strip())
                    if "Wrist Rest-Tremor Analyzer" in line:
                        log.boots += 1
                        if keep_last_boot:
                            samples.clear()
                            windows.clear()
                            for lst in (log.events, log.status, log.gaps, log.reports, log.markers, log.led):
                                lst.clear()
                    continue
                f = line.split(",")
                if tag == "S" and f[0] == "S":
                    if len(f) != 9:
                        raise ValueError
                    samples.append((int(f[1]), int(f[2]), *(int(x) for x in f[3:9]), host_t))
                elif tag == "W" and f[0] == "W":
                    if len(f) != 16:
                        raise ValueError
                    windows.append(
                        (
                            int(f[1]), int(f[2]), f[3] == "1",
                            float(f[4]), float(f[5]), float(f[6]), float(f[7]), float(f[8]),
                            f[9] == "1", f[10] == "1", f[11] == "1", f[12] == "1",
                            int(f[13]), f[14] == "1", int(f[15]),
                        )
                    )
                elif tag == "E" and f[0] == "E":
                    keys = ["type", "id", "first_pos_seq", "seq", "t_us", "mean_hz", "mean_dps", "max_dps",
                            "pos_windows", "total_windows", "reason"]
                    if len(f) != len(keys) + 1:
                        raise ValueError
                    ev = {k: _num(v) for k, v in zip(keys, f[1:])}
                    ev["host_t"] = host_t
                    log.events.append(ev)
                elif tag == "F" and f[0] == "F":
                    keys = ["kind", "reason", "seq", "t_us", "detail"]
                    if len(f) != len(keys) + 1:
                        raise ValueError
                    st = {k: _num(v) for k, v in zip(keys, f[1:])}
                    st["host_t"] = host_t
                    log.status.append(st)
                elif tag == "G" and f[0] == "G":
                    log.gaps.append({"seq": int(f[1]), "t_us": int(f[2]), "missing": int(f[3]), "host_t": host_t})
                elif tag == "R" and f[0] == "R":
                    rep: dict = {"t_us": int(f[1]), "host_t": host_t}
                    for kv in f[2:]:
                        k, v = kv.split("=", 1)
                        rep[k] = int(v)
                    log.reports.append(rep)
                elif tag == "M" and f[0] == "M":
                    log.markers.append({"t_us": int(f[1]), "n": int(f[2]), "host_t": host_t})
                elif tag == "L" and f[0] == "L":
                    log.led.append({"t_us": int(f[1]), "color": f[2], "host_t": host_t})
                elif tag == "C" and f[0] == "C":
                    log.config[f[1]] = _num(",".join(f[2:]))
                else:
                    log.malformed += 1
            except (ValueError, IndexError):
                # Línea cortada (p. ej. al abrir el puerto a mitad de línea) o basura.
                log.malformed += 1

    if samples:
        log.samples = np.array(samples, dtype=SAMPLE_DTYPE)
    if windows:
        log.windows = np.array(windows, dtype=WINDOW_DTYPE)
    return log
