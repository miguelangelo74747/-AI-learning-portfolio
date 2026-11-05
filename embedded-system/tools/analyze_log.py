#!/usr/bin/env python3
"""Informe de un log del Wrist Rest-Tremor Analyzer.

Calcula, a partir del log guardado (no de lo que diga la placa):

  * Spec 01  frecuencia de muestreo real: muestras / tiempo, media y desviación
             típica de los intervalos, y comparación con el reloj del PC.
  * Spec 04  muestras perdidas y tasa de pérdida a partir de los números de
             muestra, junto con los contadores que informa la placa (línea R).
  * Spec 02  ventanas analizadas y cuántas cumplen cada condición C1–C4.
  * Spec 03  episodios (inicio, duración, frecuencia, amplitud), fallos del
             sensor y cambios del LED.
  * Prueba 7 recomputa en numpy cada ventana a partir de las muestras crudas y
             la compara con lo que calculó la placa (±0.01 Hz, ±1 %).

Ejemplos:
    python tools/analyze_log.py logs/run10min.log
    python tools/analyze_log.py logs/prueba1_4hz.log --by-marker
    python tools/analyze_log.py logs/run.log --json logs/run_summary.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tremorlog import TremorLog, load_log  # noqa: E402
from tremor_ref import EpisodeTracker, recompute_windows  # noqa: E402

PEAK_TOL_HZ = 0.01      # prueba 7: misma frecuencia de pico ±0.01 Hz
AMP_TOL_REL = 0.01      # prueba 7: misma amplitud de banda ±1 %
PRINT_RES_DPS = 1e-4    # el firmware imprime 4 decimales


def wilson_ci(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Intervalo de confianza del 95 % (Wilson) para una proporción k/n."""
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, centre - half), min(1.0, centre + half)


def section(title: str) -> None:
    print()
    print(f"=== {title} " + "=" * max(0, 70 - len(title)))


def fmt_s(t_us: float, t0_us: float) -> str:
    return f"{(t_us - t0_us) / 1e6:9.2f} s"


# ---------------------------------------------------------------------------
def report_file(log: TremorLog, summary: dict) -> None:
    section("Archivo")
    print(f"Log:                 {log.path}")
    print(f"Líneas:              {log.total_lines} (mal formadas: {log.malformed})")
    print(f"Arranques de placa:  {log.boots}")
    print(f"Firmware:            {log.config.get('FW_VERSION', '?')}   FFT: {log.config.get('FFT_BACKEND', '?')}")
    summary["file"] = {"path": str(log.path), "lines": log.total_lines, "malformed": log.malformed, "boots": log.boots}


def report_spec01(log: TremorLog, summary: dict) -> None:
    section("Spec 01 · frecuencia de muestreo")
    s = log.samples
    if len(s) < 2:
        print("No hay suficientes muestras (líneas S).")
        return
    seq, t = s["seq"], s["t_us"].astype(np.float64)
    duration = (t[-1] - t[0]) / 1e6
    rate_received = (len(s) - 1) / duration
    consecutive = np.diff(seq) == 1
    dt = np.diff(t)[consecutive]
    rate_timer = (seq[-1] - seq[0]) / duration
    print(f"Muestras recibidas:  {len(s)} en {duration:.3f} s (reloj de la placa)")
    print(f"Frecuencia efectiva: {rate_received:.4f} Hz   (muestras recibidas / tiempo)")
    print(f"Frecuencia del timer: {rate_timer:.4f} Hz   (números de muestra / tiempo)")
    print(f"Intervalo entre muestras consecutivas: media {dt.mean():.2f} µs, d.t. {dt.std(ddof=1):.2f} µs, "
          f"mín {dt.min():.0f} µs, máx {dt.max():.0f} µs  (n = {len(dt)})")
    summary["spec01"] = {
        "samples": int(len(s)),
        "duration_s": duration,
        "rate_effective_hz": rate_received,
        "rate_timer_hz": rate_timer,
        "interval_mean_us": float(dt.mean()),
        "interval_sd_us": float(dt.std(ddof=1)),
        "interval_min_us": float(dt.min()),
        "interval_max_us": float(dt.max()),
    }

    host = s["host_t"]
    ok = np.isfinite(host)
    if ok.sum() > 100:
        h = host[ok].copy()
        # El filtro "time" de PlatformIO da la hora del día: deshacer el paso por medianoche.
        wraps = np.concatenate([[0], np.cumsum(np.diff(h) < -43200)])
        h = h + wraps * 86400.0
        board = t[ok] / 1e6
        slope, _ = np.polyfit(board - board[0], h - h[0], 1)
        host_elapsed = h[-1] - h[0]
        ppm = (slope - 1.0) * 1e6
        print(f"Reloj del PC:        {host_elapsed:.3f} s transcurridos frente a {board[-1] - board[0]:.3f} s "
              f"de la placa; pendiente {slope:.6f} ({ppm:+.0f} ppm)")
        print(f"Frecuencia según el reloj del PC: {(seq[ok][-1] - seq[ok][0]) / (h[-1] - h[0]):.4f} Hz")
        print("  (la marca del PC lleva el retardo variable del USB: mirar la pendiente, no muestras sueltas)")
        summary["spec01"].update({"host_elapsed_s": host_elapsed, "clock_slope": slope, "clock_ppm": ppm})
    else:
        print("El log no trae hora del PC (usa log_serial.py o el filtro 'time' de PlatformIO).")


def report_spec04(log: TremorLog, summary: dict) -> None:
    section("Spec 04 · muestras perdidas")
    s = log.samples
    if len(s) < 2:
        print("No hay muestras.")
        return
    seq = s["seq"]
    expected = int(seq[-1] - seq[0] + 1)
    received = int(len(np.unique(seq)))
    missing = expected - received
    duplicates = int(len(seq) - received)
    drop = 100.0 * missing / expected
    duration = (s["t_us"][-1] - s["t_us"][0]) / 1e6
    print(f"Esperadas:           {expected} (seq {seq[0]} … {seq[-1]})")
    print(f"Recibidas:           {received}   duplicadas: {duplicates}")
    print(f"Perdidas:            {missing}  ->  tasa de pérdida {drop:.4f} %")
    print(f"Duración:            {duration:.1f} s  {'(≥ 10 min: OK)' if duration >= 600 else '(menos de 10 min)'}")
    d = np.diff(seq)
    gap_idx = np.nonzero(d > 1)[0]
    if len(gap_idx):
        print(f"Huecos en el log:    {len(gap_idx)} (los 10 mayores):")
        t0 = s["t_us"][0]
        for i in sorted(gap_idx, key=lambda i: -d[i])[:10]:
            print(f"   tras seq {seq[i]:>8}  faltan {d[i] - 1:>6}   en {fmt_s(s['t_us'][i], t0)}")
    summary["spec04"] = {
        "expected": expected, "received": received, "missing": missing, "duplicates": duplicates,
        "drop_rate_pct": drop, "duration_s": float(duration), "gaps": int(len(gap_idx)),
    }

    if log.reports:
        r = log.reports[-1]
        print("Último informe de la placa (línea R):")
        keys = ["ticks", "samples", "missed_ticks", "read_fail", "stale", "fault_ticks", "sample_q_full",
                "log_q_full", "seq_missing", "windows", "valid_windows", "episodes", "faults",
                "max_window_us", "max_read_us", "log_q_min_free"]
        print("   " + "  ".join(f"{k}={r[k]}" for k in keys if k in r))
        if r.get("log_q_full", 0):
            print("   ATENCIÓN: la cola del log se llenó; parte de las líneas S faltan en el log aunque "
                  "la placa sí procesó esas muestras.")
        summary["spec04"]["board_report"] = {k: v for k, v in r.items() if k != "host_t"}
    if log.gaps:
        print(f"Huecos vistos por la tarea de procesado (líneas G): {len(log.gaps)}, "
              f"{sum(g['missing'] for g in log.gaps)} muestras")


def report_spec02(log: TremorLog, summary: dict) -> None:
    section("Spec 02 · ventanas y condiciones C1–C4")
    w = log.windows
    if len(w) == 0:
        print("No hay líneas W.")
        return
    v = w[w["valid"]]
    print(f"Ventanas:            {len(w)} (completas: {len(v)})")
    if len(v):
        for c, name in (("c1", "C1 frecuencia 3.5–7 Hz"), ("c2", "C2 amplitud ≥ 5 °/s"),
                        ("c3", "C3 regularidad ≥ 0.50"), ("c4", "C4 reposo")):
            print(f"   {name:<24} {100.0 * v[c].mean():6.1f} % de las ventanas completas")
        pos = v["c1"] & v["c2"] & v["c3"] & v["c4"]
        print(f"   Positivas (C1–C4)        {100.0 * pos.mean():6.1f} %  ({pos.sum()} ventanas)")
        print(f"Tiempo de análisis por ventana: media {v['proc_us'].mean():.0f} µs, máx {v['proc_us'].max()} µs")
        summary["spec02"] = {
            "windows": int(len(w)), "valid": int(len(v)),
            "c1_pct": float(100 * v["c1"].mean()), "c2_pct": float(100 * v["c2"].mean()),
            "c3_pct": float(100 * v["c3"].mean()), "c4_pct": float(100 * v["c4"].mean()),
            "positive_windows": int(pos.sum()),
            "proc_us_mean": float(v["proc_us"].mean()), "proc_us_max": int(v["proc_us"].max()),
        }


def report_spec03(log: TremorLog, summary: dict) -> None:
    section("Spec 03 · episodios, fallos y LED")
    t0 = log.t0_us()
    period_us = 1e6 / log.fs
    starts = [e for e in log.events if e["type"] == "START"]
    ends = {e["id"]: e for e in log.events if e["type"] == "END"}
    print(f"Episodios:           {len(starts)}")
    episodes = []
    for e in starts:
        end = ends.get(e["id"])
        onset_us = e["t_us"] - (e["seq"] - e["first_pos_seq"]) * period_us
        dur = (end["t_us"] - onset_us) / 1e6 if end else float("nan")
        line = (f"   #{e['id']:<3} inicio {fmt_s(e['t_us'], t0)}  (1.ª ventana positiva {fmt_s(onset_us, t0)})  "
                f"{e['mean_hz']:.2f} Hz  {e['mean_dps']:.1f} °/s RMS")
        if end:
            line += (f"  -> fin {fmt_s(end['t_us'], t0)} [{end['reason']}], duración ≈ {dur:.1f} s, "
                     f"media {end['mean_hz']:.2f} Hz / {end['mean_dps']:.1f} °/s, máx {end['max_dps']:.1f} °/s")
        else:
            line += "  (sin fin en el log)"
        print(line)
        episodes.append({"id": e["id"], "start_s": (e["t_us"] - t0) / 1e6, "onset_s": (onset_us - t0) / 1e6,
                         "duration_s": dur, "start_mean_hz": e["mean_hz"], "start_mean_dps": e["mean_dps"],
                         "end_mean_hz": end["mean_hz"] if end else None,
                         "end_mean_dps": end["mean_dps"] if end else None,
                         "end_reason": end["reason"] if end else None})
    if len(log.samples) > 1:
        hours = (log.samples["t_us"][-1] - log.samples["t_us"][0]) / 3.6e9
        if hours > 0:
            print(f"Episodios por hora:  {len(starts) / hours:.2f}  (útil en la prueba de actividades diarias)")
    summary["spec03"] = {"episodes": episodes}

    if log.status:
        print(f"Estado del sensor ({len(log.status)} líneas F):")
        fault_t = None
        for st in log.status:
            extra = ""
            if st["kind"] == "FAULT":
                fault_t = st["t_us"]
                red = next((l for l in log.led if l["color"] == "red_blink" and l["t_us"] >= st["t_us"]), None)
                if red:
                    extra = f"  LED rojo {(red['t_us'] - st['t_us']) / 1e3:.0f} ms después"
            elif st["kind"] == "RECOVER" and fault_t is not None:
                extra = f"  ({(st['t_us'] - fault_t) / 1e6:.2f} s en fallo)"
            print(f"   {fmt_s(st['t_us'], t0)}  {st['kind']:<10} {st['reason']:<14} seq {st['seq']}{extra}")
    if log.led:
        changes = ", ".join(f"{(l['t_us'] - t0) / 1e6:.2f} s {l['color']}" for l in log.led[:12])
        print(f"LED ({len(log.led)} cambios): {changes}{' …' if len(log.led) > 12 else ''}")


def report_test7(log: TremorLog, summary: dict) -> dict | None:
    section("Prueba 7 · placa frente a numpy")
    w = log.windows
    if len(w) == 0 or len(log.samples) == 0:
        print("Hacen falta líneas S y W.")
        return None
    try:
        ref = recompute_windows(log)
    except ValueError as exc:
        print(f"No se puede recomputar: {exc}")
        return None
    sel = w["valid"] & ref["ok"]
    n = int(sel.sum())
    print(f"Ventanas comparables (completas en la placa y en el log): {n} de {len(w)}")
    if n == 0:
        return ref
    b, r = w[sel], {k: v[sel] for k, v in ref.items()}
    d_peak = np.abs(b["peak_hz"] - r["peak_hz"])
    amp_tol = AMP_TOL_REL * r["tremor_dps"] + PRINT_RES_DPS
    d_amp = np.abs(b["tremor_dps"] - r["tremor_dps"])
    rel_amp = d_amp / np.maximum(r["tremor_dps"], 1e-9)
    d_move = np.abs(b["move_dps"] - r["move_dps"])
    d_conc = np.abs(b["conc"] - r["conc"])
    d_tilt = np.abs(b["tilt_deg"] - r["tilt_deg"])
    ok_peak = d_peak <= PEAK_TOL_HZ
    ok_amp = d_amp <= amp_tol
    print(f"Frecuencia de pico:  {ok_peak.sum()}/{n} dentro de ±{PEAK_TOL_HZ} Hz   (máx |Δ| = {d_peak.max():.5f} Hz)")
    print(f"Amplitud 3.5–7 Hz:   {ok_amp.sum()}/{n} dentro de ±1 %          (máx |Δ| relativo = {100 * rel_amp.max():.4f} %)")
    print(f"Movimiento 0.5–3 Hz: máx |Δ| = {d_move.max():.5f} °/s")
    print(f"Concentración:       máx |Δ| = {d_conc.max():.5f}")
    print(f"Inclinación:         máx |Δ| = {d_tilt.max():.4f}°  (orientativo: el filtro de la placa arrastra "
          "historia anterior al inicio del log)")
    agree = {c: float((b[c] == r[c]).mean()) for c in ("c1", "c2", "c3", "c4")}
    print("Coincidencia de condiciones: " + "  ".join(f"{c.upper()} {100 * a:.2f} %" for c, a in agree.items()))
    bad = np.nonzero(~(ok_peak & ok_amp))[0]
    if len(bad):
        print("Ventanas que no coinciden (hasta 10). Un empate entre dos bins con espectro casi plano")
        print("puede mover el pico un bin entero entre float32 y float64:")
        for i in bad[:10]:
            print(f"   seq_end {b['seq_end'][i]:>8}: pico placa {b['peak_hz'][i]:.4f} / numpy {r['peak_hz'][i]:.4f} Hz, "
                  f"amplitud placa {b['tremor_dps'][i]:.4f} / numpy {r['tremor_dps'][i]:.4f} °/s")

    # Episodios recomputados con la misma máquina de estados.
    trk = EpisodeTracker(int(log.cfg("TREMOR_MIN_WINDOWS")), int(log.cfg("EPISODE_RELEASE_WINDOWS")))
    n_start = 0
    prev_seq = -1
    gap_seqs = np.array(sorted(g["seq"] for g in log.gaps), dtype=np.int64)
    for i in range(len(w)):
        seq_end = int(w["seq_end"][i])
        # Un hueco (línea G) entre la ventana anterior y esta cierra el episodio en la placa.
        if np.any((gap_seqs > prev_seq) & (gap_seqs <= seq_end)):
            trk.force_end()
        prev_seq = seq_end
        positive = bool(w["valid"][i] and ref["ok"][i] and ref["positive"][i])
        if trk.update(positive, int(w["seq_end"][i])) == "START":
            n_start += 1
    board_starts = sum(1 for e in log.events if e["type"] == "START")
    print(f"Episodios: placa {board_starts}, numpy {n_start}")
    summary["test7"] = {
        "windows_compared": n,
        "peak_within_tol": int(ok_peak.sum()), "peak_max_abs_diff_hz": float(d_peak.max()),
        "amp_within_tol": int(ok_amp.sum()), "amp_max_rel_diff_pct": float(100 * rel_amp.max()),
        "move_max_abs_diff_dps": float(d_move.max()), "conc_max_abs_diff": float(d_conc.max()),
        "tilt_max_abs_diff_deg": float(d_tilt.max()),
        "condition_agreement": agree, "episodes_board": board_starts, "episodes_numpy": n_start,
    }
    return ref


def report_trials(log: TremorLog, summary: dict) -> None:
    section("Ensayos separados por marcas (tecla 'm' / Enter en log_serial.py)")
    if not log.markers:
        print("El log no tiene marcas M.")
        return
    t0 = log.t0_us()
    t_end = int(log.samples["t_us"][-1]) if len(log.samples) else log.markers[-1]["t_us"]
    bounds = [m["t_us"] for m in log.markers] + [t_end]
    starts = [e for e in log.events if e["type"] == "START"]
    rows = []
    for k in range(len(log.markers)):
        a, b = bounds[k], bounds[k + 1]
        ev = [e for e in starts if a <= e["t_us"] < b]
        win = log.windows[(log.windows["t_us"] >= a) & (log.windows["t_us"] < b) & log.windows["valid"]]
        lat = (ev[0]["t_us"] - a) / 1e6 if ev else float("nan")
        rows.append({
            "trial": k + 1, "start_s": (a - t0) / 1e6, "length_s": (b - a) / 1e6, "detected": bool(ev),
            "latency_from_marker_s": lat,
            "mean_hz": ev[0]["mean_hz"] if ev else None, "mean_dps": ev[0]["mean_dps"] if ev else None,
            "max_tremor_dps": float(win["tremor_dps"].max()) if len(win) else None,
            "median_peak_hz": float(np.median(win["peak_hz"])) if len(win) else None,
        })
    def num(v, fmt: str, width: int) -> str:
        return format(v, fmt).rjust(width) if v is not None and not math.isnan(v) else "—".rjust(width)

    print(f"{'#':>3} {'inicio s':>9} {'dur. s':>7} {'detect.':>8} {'latencia s':>11} {'f ep. Hz':>9} "
          f"{'A ep. °/s':>10} {'f mediana Hz':>13}")
    for r in rows:
        print(f"{r['trial']:>3} {num(r['start_s'], '.1f', 9)} {num(r['length_s'], '.1f', 7)} "
              f"{'sí' if r['detected'] else 'no':>8} {num(r['latency_from_marker_s'], '.2f', 11)} "
              f"{num(r['mean_hz'], '.2f', 9)} {num(r['mean_dps'], '.1f', 10)} {num(r['median_peak_hz'], '.2f', 13)}")
    k = sum(r["detected"] for r in rows)
    lo, hi = wilson_ci(k, len(rows))
    print(f"Detectados: {k}/{len(rows)} = {100 * k / len(rows):.1f} %  (IC 95 % Wilson: {100 * lo:.1f}–{100 * hi:.1f} %)")
    print("La latencia aquí cuenta desde la marca; la latencia de la propuesta (inicio -> ámbar) se lee en el vídeo.")
    summary["trials"] = {"rows": rows, "detected": k, "n": len(rows), "ci95": [lo, hi]}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", type=Path, help="archivo de log (log_serial.py o PlatformIO log2file)")
    ap.add_argument("--last-boot", action="store_true", help="usar solo lo que hay tras el último arranque")
    ap.add_argument("--by-marker", action="store_true", help="separar ensayos por las marcas M")
    ap.add_argument("--no-recompute", action="store_true", help="no ejecutar la prueba 7 (más rápido)")
    ap.add_argument("--json", type=Path, help="guardar también un resumen en JSON")
    args = ap.parse_args()

    log = load_log(args.log, keep_last_boot=args.last_boot)
    if log.boots > 1 and not args.last_boot:
        print(f"Aviso: el log contiene {log.boots} arranques de la placa; considera --last-boot.")
    summary: dict = {}
    report_file(log, summary)
    report_spec01(log, summary)
    report_spec04(log, summary)
    report_spec02(log, summary)
    report_spec03(log, summary)
    if not args.no_recompute:
        report_test7(log, summary)
    if args.by_marker:
        report_trials(log, summary)

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(summary, indent=2, ensure_ascii=False, default=float), encoding="utf-8")
        print(f"\nResumen guardado en {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
