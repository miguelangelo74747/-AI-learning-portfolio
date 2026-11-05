#!/usr/bin/env python3
"""Gráficas de un log del Wrist Rest-Tremor Analyzer.

Figura principal (un panel por magnitud, eje de tiempo común):
  1. giróscopo x/y/z en °/s (muestras crudas S)
  2. C1 frecuencia de pico, con la banda 3.5–7 Hz
  3. C2 amplitud de la banda de temblor (°/s RMS, escala log) y su umbral
  4. C3 concentración del pico y su umbral
  5. C4 movimiento 0.5–3 Hz (°/s RMS, escala log) y su umbral
  6. C4 cambio de inclinación (°) y su umbral
Sombreado ámbar = episodio (LED ámbar); rojo = fallo del sensor (LED rojo).

Opcional: --spectrum SEQ dibuja el espectro de la ventana que termina en SEQ
(recalculado en numpy) con las bandas de la regla.

Ejemplos:
    python tools/plot_log.py logs/run10min.log
    python tools/plot_log.py logs/prueba1.log --t0 20 --t1 60 --out figs/prueba1.png
    python tools/plot_log.py logs/prueba1.log --spectrum 4049
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tremorlog import TremorLog, load_log  # noqa: E402
from tremor_ref import Analyzer, recompute_windows, spectrum_of_window  # noqa: E402

# Paleta (categórica validada para 3 series + estados reservados).
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # x, y, z
METRIC = SERIES[0]
EPISODE = "#fab219"   # estado "warning" -> LED ámbar
FAULT = "#d03b3b"     # estado "critical" -> LED rojo
PASS_ZONE = "#f0efec"


def style(plt) -> None:
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK_2, "axes.titlecolor": INK,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False,
        "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK,
        "font.family": "sans-serif", "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold",
        "axes.titlelocation": "left", "legend.frameon": False,
    })


def intervals(log: TremorLog) -> tuple[list[tuple[int, int]], list[tuple[int, int]]]:
    """(episodios, fallos) como listas de (t_inicio_us, t_fin_us)."""
    t_last = int(max(log.samples["t_us"][-1] if len(log.samples) else 0,
                     log.windows["t_us"][-1] if len(log.windows) else 0))
    episodes, faults = [], []
    open_ep = {}
    for e in log.events:
        if e["type"] == "START":
            open_ep[e["id"]] = e["t_us"]
        elif e["type"] == "END" and e["id"] in open_ep:
            episodes.append((open_ep.pop(e["id"]), e["t_us"]))
    episodes += [(t, t_last) for t in open_ep.values()]
    fault_start = None
    for st in log.status:
        if st["kind"] == "FAULT" and fault_start is None:
            fault_start = st["t_us"]
        elif st["kind"] == "RECOVER" and fault_start is not None:
            faults.append((fault_start, st["t_us"]))
            fault_start = None
    if fault_start is not None:
        faults.append((fault_start, t_last))
    return episodes, faults


def plot_timeline(log: TremorLog, t_lo: float | None, t_hi: float | None, with_ref: bool):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D

    style(plt)
    t0 = log.t0_us()
    c = log.cfg
    fig, axes = plt.subplots(6, 1, figsize=(12, 13), sharex=True,
                             gridspec_kw={"height_ratios": [1.6, 1, 1, 1, 1, 1]})

    # 1 · giróscopo
    ax = axes[0]
    if len(log.samples):
        s = log.samples
        t = (s["t_us"] - t0) / 1e6
        g = log.gyro_dps()
        # Huecos como cortes en la línea, no como rectas que los cruzan.
        brk = np.nonzero(np.diff(s["seq"]) != 1)[0] + 1
        for axis, name in enumerate("xyz"):
            ax.plot(np.insert(t, brk, np.nan), np.insert(g[:, axis], brk, np.nan),
                    color=SERIES[axis], lw=0.8, label=f"giro {name}")
        ax.legend(loc="upper right", ncol=3)
    ax.set_title("Velocidad angular de la muñeca")
    ax.set_ylabel("°/s")

    # Métricas por ventana (solo ventanas completas).
    w = log.windows
    v = w["valid"] if len(w) else np.zeros(0, bool)
    tw = (w["t_us"] - t0) / 1e6 if len(w) else np.zeros(0)

    def metric(ax, key, title, unit, thr=None, zone=None, logy=False, ref=None):
        y = np.where(v, w[key], np.nan) if len(w) else np.zeros(0)
        if logy:
            y = np.maximum(y, 0.01)
        if zone is not None:
            ax.axhspan(zone[0], zone[1], color=PASS_ZONE, zorder=0)
        if thr is not None:
            ax.axhline(thr, color=MUTED, lw=1.2, ls="--", zorder=1)
            ax.annotate(f"umbral {thr:g} {unit}", xy=(1.0, thr), xycoords=("axes fraction", "data"),
                        xytext=(-4, 3), textcoords="offset points", ha="right", va="bottom",
                        color=INK_2, fontsize=8)
        if ref is not None and with_ref:
            ax.plot(tw, np.where(v & ref["ok"], np.maximum(ref[key], 0.01) if logy else ref[key], np.nan),
                    color=MUTED, lw=0, marker="x", ms=3, label="numpy")
        ax.plot(tw, y, color=METRIC, lw=1.5, marker="o", ms=2.5, label="placa", zorder=3)
        if logy:
            ax.set_yscale("log")
        ax.set_title(title)
        ax.set_ylabel(unit)

    ref = recompute_windows(log) if with_ref and len(w) and len(log.samples) else None
    metric(axes[1], "peak_hz", "C1 · frecuencia del pico más fuerte (0.5–15 Hz)", "Hz",
           zone=(c("TREMOR_BAND_LOW_HZ"), c("TREMOR_BAND_HIGH_HZ")), ref=ref)
    axes[1].set_ylim(0, c("CONC_REF_HIGH_HZ"))
    metric(axes[2], "tremor_dps", "C2 · amplitud de la banda de temblor 3.5–7 Hz", "°/s RMS",
           thr=c("TREMOR_MIN_DPS"), logy=True, ref=ref)
    metric(axes[3], "conc", "C3 · fracción de potencia a ±1 Hz del pico", "fracción",
           thr=c("PEAK_CONCENTRATION_MIN"), ref=ref)
    axes[3].set_ylim(0, 1.05)
    metric(axes[4], "move_dps", "C4 · movimiento 0.5–3 Hz", "°/s RMS", thr=c("MOVE_MAX_DPS"), logy=True, ref=ref)
    metric(axes[5], "tilt_deg", "C4 · cambio de inclinación del antebrazo en la ventana", "°",
           thr=c("REST_MAX_TILT_DEG"), ref=ref)
    axes[5].set_xlabel("tiempo desde la primera muestra (s)")

    # Episodios, fallos y marcas sobre todos los paneles.
    episodes, faults = intervals(log)
    for ax in axes:
        for a, b in episodes:
            ax.axvspan((a - t0) / 1e6, (b - t0) / 1e6, color=EPISODE, alpha=0.28, lw=0, zorder=0)
        for a, b in faults:
            ax.axvspan((a - t0) / 1e6, (b - t0) / 1e6, color=FAULT, alpha=0.18, lw=0, zorder=0)
        for m in log.markers:
            ax.axvline((m["t_us"] - t0) / 1e6, color=INK_2, lw=0.6, ls=":", zorder=1)
    for m in log.markers:
        tm = (m["t_us"] - t0) / 1e6
        if (t_lo is not None and tm < t_lo) or (t_hi is not None and tm > t_hi):
            continue  # fuera del zoom: la etiqueta ensancharía la figura
        axes[0].annotate(f"M{m['n']}", xy=(tm, 1.0), xycoords=("data", "axes fraction"),
                         xytext=(2, -10), textcoords="offset points", fontsize=7, color=INK_2)

    handles = [Patch(color=EPISODE, alpha=0.5, label="episodio de temblor (LED ámbar)"),
               Patch(color=FAULT, alpha=0.35, label="fallo del sensor (LED rojo)"),
               Line2D([], [], color=METRIC, marker="o", ms=3, lw=1.5, label="valor de la placa"),
               Line2D([], [], color=MUTED, ls="--", lw=1.2, label="umbral de la regla")]
    if ref is not None:
        handles.append(Line2D([], [], color=MUTED, marker="x", lw=0, ms=4, label="recomputado en numpy"))
    fig.legend(handles=handles, loc="upper center", ncol=len(handles), bbox_to_anchor=(0.5, 0.995))
    fig.suptitle(f"{log.path.name} — {len(episodes)} episodio(s), {len(faults)} fallo(s) del sensor",
                 x=0.01, y=1.015, ha="left", fontsize=11, fontweight="bold")
    if t_lo is not None or t_hi is not None:
        axes[-1].set_xlim(t_lo, t_hi)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    return fig


def plot_spectrum(log: TremorLog, seq_end: int):
    import matplotlib.pyplot as plt

    style(plt)
    res = spectrum_of_window(log, seq_end)
    if res is None:
        raise SystemExit(f"La ventana que termina en seq {seq_end} no está completa en el log.")
    f, p = res
    an = Analyzer(log.cfg)
    seq = log.samples["seq"]
    j = int(np.searchsorted(seq, seq_end))
    from tremor_ref import gravity_lowpass
    grav = gravity_lowpass(log.accel_g(), seq, log.cfg)
    m = an.analyze(log.gyro_dps()[j - an.n + 1 : j + 1], grav[j - an.n + 1 : j + 1])
    c = log.cfg

    fig, ax = plt.subplots(figsize=(9, 4.2))
    ax.axvspan(c("MOVE_BAND_LOW_HZ"), c("MOVE_BAND_HIGH_HZ"), color=PASS_ZONE, zorder=0)
    ax.axvspan(c("TREMOR_BAND_LOW_HZ"), c("TREMOR_BAND_HIGH_HZ"), color=EPISODE, alpha=0.22, lw=0, zorder=0)
    ax.axvspan(m.peak_hz - c("PEAK_HALF_WIDTH_HZ"), m.peak_hz + c("PEAK_HALF_WIDTH_HZ"),
               color=METRIC, alpha=0.10, lw=0, zorder=0)
    ax.plot(f, p, color=METRIC, lw=1.5, marker="o", ms=3, drawstyle="default", zorder=3)
    ax.axvline(m.peak_hz, color=INK_2, lw=0.8, ls=":")
    ymax = p.max() * 1.25 if p.max() > 0 else 1.0
    ax.annotate(f"pico {m.peak_hz:.2f} Hz", xy=(m.peak_hz, p.max()), xytext=(6, -4), textcoords="offset points",
                color=INK, fontsize=8, va="top")
    for lo, hi, label in ((c("MOVE_BAND_LOW_HZ"), c("MOVE_BAND_HIGH_HZ"), "movimiento"),
                          (c("TREMOR_BAND_LOW_HZ"), c("TREMOR_BAND_HIGH_HZ"), "temblor")):
        ax.text((lo + hi) / 2, ymax * 0.98, label, ha="center", va="top", color=INK_2, fontsize=8)
    ax.set_xlim(0, c("CONC_REF_HIGH_HZ") + 5)
    ax.set_ylim(0, ymax)
    ax.set_xlabel("frecuencia (Hz)")
    ax.set_ylabel("potencia ((°/s)² por bin)")
    w = log.windows
    board = w[w["seq_end"] == seq_end]
    btxt = ""
    if len(board):
        b = board[0]
        btxt = f" (placa: {b['peak_hz']:.2f} Hz, {b['tremor_dps']:.2f} °/s)"
    ax.text(m.peak_hz, ymax * 0.90, f"±{c('PEAK_HALF_WIDTH_HZ'):g} Hz (C3)", ha="center", va="top",
            color=INK_2, fontsize=8)
    ax.set_title(f"Espectro de la ventana seq {seq_end - an.n + 1}–{seq_end} (recomputado en numpy)\n"
                 f"temblor {m.tremor_dps:.2f} °/s RMS · conc. {m.conc:.2f} · mov. {m.move_dps:.2f} °/s · "
                 f"incl. {m.tilt_deg:.1f}° → ventana {'POSITIVA' if m.positive else 'negativa'}{btxt}",
                 fontsize=9)
    fig.tight_layout()
    return fig


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path)
    ap.add_argument("--out", type=Path, help="PNG de salida (por defecto, junto al log)")
    ap.add_argument("--t0", type=float, help="inicio del zoom (s)")
    ap.add_argument("--t1", type=float, help="fin del zoom (s)")
    ap.add_argument("--spectrum", type=int, metavar="SEQ", help="espectro de la ventana que termina en SEQ")
    ap.add_argument("--no-ref", action="store_true", help="no superponer los valores recomputados en numpy")
    ap.add_argument("--last-boot", action="store_true", help="usar solo lo que hay tras el último arranque")
    ap.add_argument("--show", action="store_true", help="abrir una ventana además de guardar el PNG")
    args = ap.parse_args()

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    log = load_log(args.log, keep_last_boot=args.last_boot)
    if args.spectrum is not None:
        fig = plot_spectrum(log, args.spectrum)
        out = args.out or args.log.with_name(f"{args.log.stem}_spectrum_{args.spectrum}.png")
    else:
        fig = plot_timeline(log, args.t0, args.t1, with_ref=not args.no_ref)
        out = args.out or args.log.with_suffix(".png")
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Figura guardada en {out}")
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
