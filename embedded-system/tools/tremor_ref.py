"""Implementación de referencia en numpy de lo que hace la placa.

Repite, paso a paso y con los mismos parámetros, lib/tremor/src/tremor_dsp.cpp
y tremor_pipeline.cpp: ventana Hann periódica de 256 muestras, FFT de cada eje
del giróscopo, suma de espectros de potencia con escala de Parseval, pico con
interpolación gaussiana, concentración, RMS de bandas, inclinación con biquad
RBJ y máquina de episodios (6 ventanas para empezar, 4 para terminar).

Sirve para la prueba 7 de la propuesta: recomputar en numpy, a partir de las
muestras crudas del log, los números que la placa calculó en tiempo real.
La referencia trabaja en float64; la placa en float32.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from tremorlog import TremorLog


# ---------------------------------------------------------------------------
#  Filtro de inclinación
# ---------------------------------------------------------------------------
def biquad_lowpass_coeffs(fc_hz: float, q: float, fs_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """Coeficientes RBJ (Audio EQ Cookbook) normalizados: b = [b0 b1 b2], a = [1 a1 a2]."""
    w0 = 2.0 * math.pi * fc_hz / fs_hz
    cw = math.cos(w0)
    alpha = math.sin(w0) / (2.0 * q)
    a0 = 1.0 + alpha
    b = np.array([(1 - cw) / 2, 1 - cw, (1 - cw) / 2]) / a0
    a = np.array([1.0, -2 * cw / a0, (1 - alpha) / a0])
    return b, a


def biquad_filter(x: np.ndarray, b: np.ndarray, a: np.ndarray) -> np.ndarray:
    """Forma directa I con estado inicial en régimen permanente (como reset(x0))."""
    y = np.empty_like(x, dtype=np.float64)
    if len(x) == 0:
        return y
    x1 = x2 = y1 = y2 = float(x[0])
    b0, b1, b2 = b
    _, a1, a2 = a
    for i, xi in enumerate(x.astype(np.float64)):
        yi = b0 * xi + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1 = x1, xi
        y2, y1 = y1, yi
        y[i] = yi
    return y


def contiguous_segments(seq: np.ndarray) -> list[tuple[int, int]]:
    """Tramos [inicio, fin) de índices con números de muestra consecutivos."""
    if len(seq) == 0:
        return []
    breaks = np.nonzero(np.diff(seq) != 1)[0] + 1
    starts = np.concatenate([[0], breaks])
    ends = np.concatenate([breaks, [len(seq)]])
    return list(zip(starts.tolist(), ends.tolist()))


def gravity_lowpass(accel_g: np.ndarray, seq: np.ndarray, cfg) -> np.ndarray:
    """Gravedad filtrada (n, 3); el filtro se reinicia tras cada hueco."""
    b, a = biquad_lowpass_coeffs(cfg("TILT_LPF_HZ"), cfg("TILT_LPF_Q"), cfg("SAMPLE_RATE_HZ"))
    grav = np.empty_like(accel_g, dtype=np.float64)
    for s, e in contiguous_segments(seq):
        for axis in range(3):
            grav[s:e, axis] = biquad_filter(accel_g[s:e, axis], b, a)
    return grav


# ---------------------------------------------------------------------------
#  Análisis de una ventana
# ---------------------------------------------------------------------------
@dataclass
class Metrics:
    peak_hz: float
    tremor_dps: float
    conc: float
    move_dps: float
    tilt_deg: float
    c1: bool
    c2: bool
    c3: bool
    c4: bool

    @property
    def positive(self) -> bool:
        return self.c1 and self.c2 and self.c3 and self.c4


class Analyzer:
    def __init__(self, cfg):
        self.cfg = cfg
        self.n = int(cfg("WINDOW_SAMPLES"))
        self.fs = cfg("SAMPLE_RATE_HZ")
        self.df = self.fs / self.n
        self.nbins = self.n // 2 + 1
        k = np.arange(self.n)
        self.window = 0.5 * (1.0 - np.cos(2.0 * np.pi * k / self.n))  # Hann periódica
        self.wsum2 = float(np.sum(self.window**2))
        self.freqs = np.arange(self.nbins) * self.df

    def bin_low(self, hz: float) -> int:
        return int(min(max(math.ceil(hz / self.df - 1e-4), 0), self.nbins - 1))

    def bin_high(self, hz: float) -> int:
        return int(min(max(math.floor(hz / self.df + 1e-4), 0), self.nbins - 1))

    def power_spectrum(self, gyro_win: np.ndarray) -> np.ndarray:
        """Suma de los espectros de potencia de los 3 ejes, en (°/s)² por bin."""
        x = gyro_win - gyro_win.mean(axis=0, keepdims=True)
        spec = np.fft.rfft(x * self.window[:, None], axis=0)
        p = (np.abs(spec) ** 2).sum(axis=1)
        scale = np.full(self.nbins, 2.0 / (self.n * self.wsum2))
        scale[0] = scale[-1] = 1.0 / (self.n * self.wsum2)
        return p * scale

    def analyze(self, gyro_win: np.ndarray, grav_win: np.ndarray) -> Metrics:
        c = self.cfg
        p = self.power_spectrum(gyro_win)

        lo, hi = self.bin_low(c("CONC_REF_LOW_HZ")), self.bin_high(c("CONC_REF_HIGH_HZ"))
        ref = p[lo : hi + 1]
        ref_power = float(ref.sum())
        kpk = lo + int(np.argmax(ref))
        delta = 0.0
        if 0 < kpk < self.nbins - 1:
            a_, b_, c_ = p[kpk - 1], p[kpk], p[kpk + 1]
            if a_ > 0 and b_ > 0 and c_ > 0:
                la, lb, lc = math.log(a_), math.log(b_), math.log(c_)
                den = la - 2 * lb + lc
                if den < 0:
                    delta = 0.5 * (la - lc) / den
                delta = min(max(delta, -0.5), 0.5)
        peak_hz = (kpk + delta) * self.df if ref_power > 0 else 0.0

        k = np.arange(lo, hi + 1)
        near = np.abs(k * self.df - peak_hz) <= c("PEAK_HALF_WIDTH_HZ")
        conc = float(ref[near].sum() / ref_power) if ref_power > 0 else 0.0

        t_lo, t_hi = self.bin_low(c("TREMOR_BAND_LOW_HZ")), self.bin_high(c("TREMOR_BAND_HIGH_HZ"))
        m_lo, m_hi = self.bin_low(c("MOVE_BAND_LOW_HZ")), self.bin_high(c("MOVE_BAND_HIGH_HZ"))
        tremor = math.sqrt(float(p[t_lo : t_hi + 1].sum()))
        move = math.sqrt(float(p[m_lo : m_hi + 1].sum()))

        newest = grav_win[-1]
        nn = float(np.linalg.norm(newest))
        min_cos = 1.0
        if nn > 1e-6:
            g = grav_win[:-1]
            gn = np.linalg.norm(g, axis=1)
            ok = gn > 1e-6
            if np.any(ok):
                cos = (g[ok] @ newest) / (gn[ok] * nn)
                min_cos = min(1.0, float(cos.min()))
        tilt = math.degrees(math.acos(max(-1.0, min_cos)))

        return Metrics(
            peak_hz=peak_hz,
            tremor_dps=tremor,
            conc=conc,
            move_dps=move,
            tilt_deg=tilt,
            c1=c("TREMOR_BAND_LOW_HZ") <= peak_hz <= c("TREMOR_BAND_HIGH_HZ"),
            c2=tremor >= c("TREMOR_MIN_DPS"),
            c3=conc >= c("PEAK_CONCENTRATION_MIN"),
            c4=(move < c("MOVE_MAX_DPS")) and (tilt < c("REST_MAX_TILT_DEG")),
        )


# ---------------------------------------------------------------------------
#  Máquina de episodios (copia de EpisodeTracker)
# ---------------------------------------------------------------------------
class EpisodeTracker:
    def __init__(self, min_windows: int, release_windows: int):
        self.min_windows = min_windows
        self.release_windows = release_windows
        self.active = False
        self.pos = 0
        self.neg = 0
        self.next_id = 1
        self.first_pos_seq = None
        self.streak_first_seq = None

    def update(self, positive: bool, seq_end: int) -> str | None:
        if positive:
            if self.pos == 0:
                self.streak_first_seq = seq_end
            self.pos += 1
            self.neg = 0
            if not self.active and self.pos >= self.min_windows:
                self.active = True
                self.first_pos_seq = self.streak_first_seq
                self.next_id += 1
                return "START"
        else:
            self.pos = 0
            self.neg += 1
            if self.active and self.neg >= self.release_windows:
                self.active = False
                return "END"
        return None

    def force_end(self) -> str | None:
        was = self.active
        self.active = False
        self.pos = self.neg = 0
        return "END" if was else None


# ---------------------------------------------------------------------------
#  Recomputar todas las ventanas de un log
# ---------------------------------------------------------------------------
def recompute_windows(log: TremorLog) -> dict[str, np.ndarray]:
    """Recalcula cada línea W cuya ventana de 256 muestras está completa en el log.

    Devuelve arrays alineados con log.windows:
      ok        -> True si las 256 muestras de esa ventana están en el log
      peak_hz, tremor_dps, conc, move_dps, tilt_deg, c1..c4, positive
    """
    cfg = log.cfg
    an = Analyzer(cfg)
    n = an.n
    w = log.windows
    out = {k: np.full(len(w), np.nan) for k in ("peak_hz", "tremor_dps", "conc", "move_dps", "tilt_deg")}
    for k in ("ok", "c1", "c2", "c3", "c4", "positive"):
        out[k] = np.zeros(len(w), dtype=bool)
    if len(log.samples) == 0 or len(w) == 0:
        return out

    seq = log.samples["seq"]
    if np.any(np.diff(seq) <= 0):
        raise ValueError(
            "Los números de muestra no son crecientes (¿la placa se reinició durante el log?). "
            "Usa --last-boot o recorta el log."
        )
    gyro = log.gyro_dps()
    grav = gravity_lowpass(log.accel_g(), seq, cfg)

    idx = np.searchsorted(seq, w["seq_end"])
    for i, (seq_end, j) in enumerate(zip(w["seq_end"], idx)):
        if j >= len(seq) or seq[j] != seq_end or j - (n - 1) < 0:
            continue
        if seq[j] - seq[j - (n - 1)] != n - 1:
            continue  # falta alguna muestra de la ventana en el log
        m = an.analyze(gyro[j - n + 1 : j + 1], grav[j - n + 1 : j + 1])
        out["ok"][i] = True
        for k in ("peak_hz", "tremor_dps", "conc", "move_dps", "tilt_deg"):
            out[k][i] = getattr(m, k)
        out["c1"][i], out["c2"][i], out["c3"][i], out["c4"][i] = m.c1, m.c2, m.c3, m.c4
        out["positive"][i] = m.positive
    return out


def spectrum_of_window(log: TremorLog, seq_end: int) -> tuple[np.ndarray, np.ndarray] | None:
    """(frecuencias, potencia) de la ventana que termina en seq_end, o None si está incompleta."""
    an = Analyzer(log.cfg)
    seq = log.samples["seq"]
    j = int(np.searchsorted(seq, seq_end))
    if j >= len(seq) or seq[j] != seq_end or j - (an.n - 1) < 0 or seq[j] - seq[j - an.n + 1] != an.n - 1:
        return None
    return an.freqs, an.power_spectrum(log.gyro_dps()[j - an.n + 1 : j + 1])
