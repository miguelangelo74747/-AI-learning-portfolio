#!/usr/bin/env python3
"""Lee el puerto serie del Wrist Rest-Tremor Analyzer y guarda el log.

Cada línea se guarda tal cual, precedida de la hora del PC (segundos Unix) y un
tabulador. Esa hora es el "reloj secundario" para comprobar la frecuencia de
muestreo (Spec 01); el PC no toma ninguna decisión.

Mientras graba:
  * Enter            -> envía 'm' a la placa, que escribe una marca M en el log
                        (úsalo al empezar cada ensayo; analyze_log.py --by-marker)
  * Ctrl+C           -> pide un último informe ('r') y cierra el archivo

Ejemplos:
    python tools/log_serial.py --list
    python tools/log_serial.py -p /dev/ttyUSB0 --reset --duration 600 -o logs/run10min.log
    python tools/log_serial.py -p COM5 -o logs/prueba1_4hz.log
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.exit("Falta pyserial: pip install -r tools/requirements.txt")


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No se encontró ningún puerto serie.")
    for p in ports:
        print(f"{p.device:<20} {p.description}  [{p.hwid}]")


def open_port(port: str, baud: int, reset: bool) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.2
    # DTR/RTS sin activar: con el circuito de auto-reset de la placa así no se
    # queda en reset ni entra en el bootloader al abrir el puerto.
    ser.dtr = False
    ser.rts = False
    ser.open()
    if reset:
        # Pulso en EN (RTS) con IO0 (DTR) en alto: reinicio normal, el log
        # empieza con la cabecera C de la configuración.
        ser.rts = True
        time.sleep(0.12)
        ser.rts = False
    return ser


class Counters:
    def __init__(self) -> None:
        self.lines = 0
        self.samples = 0
        self.windows = 0
        self.episodes = 0
        self.faults = 0
        self.markers = 0
        self.last_window = ""
        self.last_status = ""


def stdin_markers(ser: serial.Serial, lock: threading.Lock, stop: threading.Event) -> None:
    """Hilo: cada Enter en la terminal envía una marca a la placa."""
    while not stop.is_set():
        line = sys.stdin.readline()
        if not line:  # stdin cerrado
            return
        with lock:
            ser.write(b"m")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="puerto serie (p. ej. /dev/ttyUSB0, /dev/cu.usbserial-*, COM5)")
    ap.add_argument("-b", "--baud", type=int, default=921600, help="baudios (por defecto 921600)")
    ap.add_argument("-o", "--out", type=Path, help="archivo de salida (por defecto logs/tremor_FECHA.log)")
    ap.add_argument("-d", "--duration", type=float, default=0.0, help="segundos a grabar (0 = hasta Ctrl+C)")
    ap.add_argument("--reset", action="store_true", help="reiniciar la placa al abrir el puerto")
    ap.add_argument("--list", action="store_true", help="listar puertos y salir")
    ap.add_argument("--quiet", action="store_true", help="sin resumen periódico en pantalla")
    args = ap.parse_args()

    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        ap.error("indica el puerto con -p (usa --list para verlos)")

    out = args.out or Path("logs") / f"tremor_{datetime.now():%Y%m%d_%H%M%S}.log"
    out.parent.mkdir(parents=True, exist_ok=True)

    ser = open_port(args.port, args.baud, args.reset)
    lock = threading.Lock()
    stop = threading.Event()
    threading.Thread(target=stdin_markers, args=(ser, lock, stop), daemon=True).start()

    c = Counters()
    t_start = time.time()
    t_status = t_start
    buf = b""
    print(f"Grabando {args.port} @ {args.baud} baudios en {out}")
    print("Enter = marca de ensayo, Ctrl+C = terminar")
    try:
        with out.open("w", encoding="utf-8", newline="\n") as fh:
            fh.write(f"# log_serial.py: puerto={args.port} baud={args.baud} inicio={datetime.now().isoformat()}\n")
            try:
                while True:
                    chunk = ser.read(4096)
                    now = time.time()
                    if chunk:
                        buf += chunk
                        *lines, buf = buf.split(b"\n")
                        for raw in lines:
                            line = raw.decode("utf-8", errors="replace").rstrip("\r")
                            fh.write(f"{now:.6f}\t{line}\n")
                            c.lines += 1
                            tag = line[:2]
                            if tag == "S,":
                                c.samples += 1
                            elif tag == "W,":
                                c.windows += 1
                                c.last_window = line
                            elif line.startswith("E,START"):
                                c.episodes += 1
                                print(f"\n>>> EPISODIO: {line}")
                            elif tag == "F,":
                                c.faults += line.startswith("F,FAULT")
                                c.last_status = line
                                print(f"\n>>> SENSOR: {line}")
                            elif tag == "M,":
                                c.markers += 1
                                print(f"\n>>> MARCA {c.markers}")
                            elif line.startswith("#"):
                                print(f"\n{line}")
                    if not args.quiet and now - t_status >= 5.0:
                        t_status = now
                        w = c.last_window.split(",")
                        wtxt = ""
                        if len(w) >= 9:
                            wtxt = f" | pico {float(w[4]):.2f} Hz, temblor {float(w[5]):.1f} °/s, mov {float(w[7]):.1f} °/s"
                        print(f"\r{now - t_start:7.1f} s  muestras {c.samples}  ventanas {c.windows}  "
                              f"episodios {c.episodes}  fallos {c.faults}{wtxt}   ", end="", flush=True)
                    if args.duration and now - t_start >= args.duration:
                        break
            except KeyboardInterrupt:
                pass
            # Último informe de contadores de la placa.
            with lock:
                ser.write(b"r")
            t_end = time.time() + 0.5
            while time.time() < t_end:
                chunk = ser.read(4096)
                if chunk:
                    buf += chunk
                    *lines, buf = buf.split(b"\n")
                    for raw in lines:
                        fh.write(f"{time.time():.6f}\t{raw.decode('utf-8', errors='replace').rstrip(chr(13))}\n")
    finally:
        stop.set()
        ser.close()
    print(f"\nGuardado: {out}  ({c.lines} líneas, {c.samples} muestras, {c.episodes} episodios)")
    print(f"Siguiente paso: python tools/analyze_log.py {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
