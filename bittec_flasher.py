# Bittec Flasher — Utility for flashing firmware and two-pass MAVLink parameter upload
# Copyright (C) 2026 Bittec (https://bittec.com.ua)
# Repository: https://github.com/zloytarik/bittec-flasher
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import array
import base64
import binascii
import fnmatch
import glob
import json
import math
import os
import platform
import queue
import re
import runpy
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
import zlib
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")

import serial  # noqa: E402, F401
import serial.tools.list_ports  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v10 import ardupilotmega as _mav10  # noqa: E402, F401
from pymavlink.dialects.v20 import ardupilotmega as _mav20  # noqa: E402, F401
from pymavlink.dialects.v20 import common as _mav20_common  # noqa: E402, F401

APP_NAME = "Bittec Flasher"
APP_VERSION = "1.0.0"
APP_COPYRIGHT = "© 2026 Bittec (https://bittec.com.ua)"
APP_REPO = "https://github.com/zloytarik/bittec-flasher"

if getattr(sys, "frozen", False):
    BASE_DIR = Path(sys.executable).resolve().parent
    RESOURCE_DIR = Path(getattr(sys, "_MEIPASS", BASE_DIR))
else:
    BASE_DIR = Path(__file__).resolve().parent
    RESOURCE_DIR = BASE_DIR

READONLY_PATTERNS = [
    r"^STAT_.*",
    r"^FORMAT_VERSION$",
    r"^.*_FORMAT_VERSION$",
    r"^SYSID_SW_MREV$",
    r"^SYSID_SW_TYPE$",
    r"^BRD_SERIAL_NUM$",
    r"^.*_DEVID\d*$",
    r"^COMPASS_DEV_ID\d*$",
    r"^COMPASS_PRIO\d+_ID$",
    r"^INS_ACC\d*_ID$",
    r"^INS_GYR\d*_ID$",
    r"^MIS_TOTAL$",
    r"^FENCE_TOTAL$",
    r"^RALLY_TOTAL$",
    r"^LOG_LASTFILE$",
    r"^GND_ABS_PRESS\d*$",
    r"^GND_TEMP\d*$",
    r"^BARO\d*_GND_PRESS$",
    r"^BARO\d*_GND_TEMP$",
    r"^INS_GYR\d*_CALTEMP$",
    r"^INS_ACC\d*_CALTEMP$",
]

CALIB_PATTERNS = [
    r"^INS_ACC\d*OFFS_.*",
    r"^INS_ACC\d*SCAL_.*",
    r"^INS_GYR\d*OFFS_.*",
    r"^COMPASS_OFS\d*.*",
    r"^COMPASS_DIA\d*.*",
    r"^COMPASS_ODI\d*.*",
    r"^ARSPD\d*_OFFSET$",
]

RO_RE = [re.compile(p, re.IGNORECASE) for p in READONLY_PATTERNS]
CAL_RE = [re.compile(p, re.IGNORECASE) for p in CALIB_PATTERNS]


def _maybe_run_embedded_uploader():
    if len(sys.argv) >= 2 and sys.argv[1] == "--run-uploader":
        uploader_path = RESOURCE_DIR / "uploader.py"
        if not uploader_path.is_file():
            sys.stderr.write(f"Не знайдено вбудований uploader.py: {uploader_path}\n")
            sys.exit(1)
        if sys.stdout and hasattr(sys.stdout, "reconfigure"):
            try:
                sys.stdout.reconfigure(line_buffering=True)
            except Exception:
                pass
        sys.argv = [str(uploader_path), *sys.argv[2:]]
        runpy.run_path(str(uploader_path), run_name="__main__")
        sys.exit(0)


def find_default_file(folder: Path, patterns: tuple[str, ...], skip_suffix: str = "") -> str:
    for pat in patterns:
        for cand in sorted(folder.glob(pat)):
            if cand.is_file() and (not skip_suffix or not cand.stem.endswith(skip_suffix)):
                return str(cand)
    return ""


def norm_param_id(pid) -> str:
    if isinstance(pid, bytes):
        return pid.decode("utf-8", errors="ignore").rstrip("\x00")
    return str(pid).rstrip("\x00")


def rank_port(port_info) -> int:
    desc = f"{port_info.description} {port_info.manufacturer or ''} {port_info.hwid}".lower()
    if "slcan" in desc:
        return 3
    if any(k in desc for k in ("mavlink", "pixhawk", "ardupilot", "holybro", "chibios")):
        return 0
    if "usb" in desc or "stm" in desc:
        return 1
    return 2


def prepare_param_file(src_path: Path, strip_calib: bool) -> tuple[Path, dict[str, float], dict]:
    clean_dict: dict[str, float] = {}
    removed_ro: list[str] = []
    removed_cal: list[str] = []
    kept_cal: list[str] = []

    for raw in src_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = re.split(r"[,\s]+", line, maxsplit=1)
        if len(parts) < 2:
            continue
        name = parts[0].strip()
        val_str = parts[1].split("#", 1)[0].strip()
        if not val_str:
            continue
        try:
            val_float = float(val_str)
        except ValueError:
            continue

        if any(r.match(name) for r in RO_RE):
            removed_ro.append(f"{name},{val_str}")
            continue

        is_cal = any(r.match(name) for r in CAL_RE)
        if is_cal and strip_calib:
            removed_cal.append(f"{name},{val_str}")
            continue
        if is_cal:
            kept_cal.append(name)

        clean_dict[name] = val_float

    stem = src_path.stem
    if stem.endswith("_clean"):
        out_clean = src_path.with_name(f"{stem}.param")
        out_removed = src_path.with_name(f"{stem}_removed.txt")
    else:
        out_clean = src_path.with_name(f"{stem}_clean.param")
        out_removed = src_path.with_name(f"{stem}_removed.txt")

    def fmt_num(v: float) -> str:
        return str(int(v)) if v.is_integer() else f"{v:g}"

    out_clean.write_text(
        "\n".join(f"{k},{fmt_num(v)}" for k, v in clean_dict.items()) + "\n",
        encoding="utf-8",
    )

    removed_all = ["# Видалені read-only та автооновлювані параметри:"] + removed_ro
    if removed_cal:
        removed_all += ["", "# Видалені калібрування датчиків:"] + removed_cal
    out_removed.write_text("\n".join(removed_all) + "\n", encoding="utf-8")

    gps_keys = sorted(k for k in clean_dict if k.upper().startswith("GPS"))
    legacy_gps = [k for k in gps_keys if k.upper() in ("GPS_TYPE", "GPS_TYPE2", "GPS_GNSS_MODE")]
    stats = {
        "clean_path": out_clean,
        "removed_path": out_removed,
        "kept_count": len(clean_dict),
        "removed_ro_count": len(removed_ro),
        "removed_cal_count": len(removed_cal),
        "kept_cal_count": len(kept_cal),
        "gps_count": len(gps_keys),
        "legacy_gps": legacy_gps,
    }
    return out_clean, clean_dict, stats


class BittecFlasherGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(f"{APP_NAME} v{APP_VERSION} — Прошивка та параметри MAVLink")
        self.root.geometry("790x620")
        self.root.minsize(710, 520)

        self.log_queue: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.active_proc: subprocess.Popen | None = None

        self.fw_var = tk.StringVar(value=find_default_file(BASE_DIR, ("*.apj",)))
        self.param_var = tk.StringVar(
            value=find_default_file(BASE_DIR, ("*.param", "*.parm"), skip_suffix="_clean")
        )
        self.port_var = tk.StringVar(value="Авто")
        self.strip_calib_var = tk.BooleanVar(value=False)
        self.skip_flash_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Готовий до роботи")

        self._build_ui()
        self.refresh_ports()
        self.root.after(100, self._poll_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        pad = {"padx": 10, "pady": 5}

        frame_files = ttk.LabelFrame(self.root, text="Файли прошивки та параметрів", padding=10)
        frame_files.pack(fill="x", **pad)
        frame_files.columnconfigure(1, weight=1)

        ttk.Label(frame_files, text="Прошивка (.apj):").grid(row=0, column=0, sticky="w", pady=4)
        ttk.Entry(frame_files, textvariable=self.fw_var).grid(row=0, column=1, sticky="ew", padx=6, pady=4)
        ttk.Button(frame_files, text="Обрати...", command=self._browse_fw).grid(row=0, column=2, pady=4)

        ttk.Label(frame_files, text="Параметри (.param/.parm):").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Entry(frame_files, textvariable=self.param_var).grid(row=1, column=1, sticky="ew", padx=6, pady=4)
        ttk.Button(frame_files, text="Обрати...", command=self._browse_param).grid(row=1, column=2, pady=4)

        frame_opts = ttk.LabelFrame(self.root, text="Параметри заливки", padding=10)
        frame_opts.pack(fill="x", **pad)
        frame_opts.columnconfigure(1, weight=1)

        ttk.Label(frame_opts, text="COM-порт:").grid(row=0, column=0, sticky="w", pady=4)
        self.port_combo = ttk.Combobox(frame_opts, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=6, pady=4)
        ttk.Button(frame_opts, text="Оновити порти", command=self.refresh_ports).grid(row=0, column=2, pady=4)

        ttk.Checkbutton(
            frame_opts,
            text="Вичистити калібрування датчиків (акселерометр, гіроскоп, компас — для серії бортів)",
            variable=self.strip_calib_var,
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=2)

        ttk.Checkbutton(
            frame_opts,
            text="Тільки підготувати й залити параметри (пропустити прошивку .apj)",
            variable=self.skip_flash_var,
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=2)

        frame_btns = ttk.Frame(self.root)
        frame_btns.pack(fill="x", padx=10, pady=6)

        self.btn_prep = ttk.Button(frame_btns, text="Тільки підготувати параметри", command=self.start_prepare_only)
        self.btn_prep.pack(side="left", padx=(0, 8))

        self.btn_run = ttk.Button(frame_btns, text="Підготувати та прошити", command=self.start_full_process)
        self.btn_run.pack(side="left", padx=(0, 8))

        self.btn_stop = ttk.Button(frame_btns, text="Зупинити", command=self.stop_process, state="disabled")
        self.btn_stop.pack(side="left")

        ttk.Button(frame_btns, text="Про програму", command=self._show_about).pack(side="right", padx=(6, 0))
        ttk.Button(frame_btns, text="Очистити лог", command=self._clear_log).pack(side="right")

        frame_prog = ttk.Frame(self.root)
        frame_prog.pack(fill="x", padx=10, pady=(2, 6))
        self.progress = ttk.Progressbar(frame_prog, mode="determinate", maximum=100)
        self.progress.pack(fill="x")
        ttk.Label(frame_prog, textvariable=self.status_var).pack(anchor="w", pady=(3, 0))

        frame_log = ttk.LabelFrame(self.root, text="Журнал виконання", padding=6)
        frame_log.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.log_box = scrolledtext.ScrolledText(frame_log, state="disabled", font=("Consolas", 9), wrap="word")
        self.log_box.pack(fill="both", expand=True)

    def _show_about(self):
        info = (
            f"{APP_NAME} v{APP_VERSION}\n"
            f"{APP_COPYRIGHT}\n"
            f"GitHub: {APP_REPO}\n"
            "Ліцензія: GNU GPLv3\n\n"
            "Незалежна утиліта для автоматичного очищення параметрів,\n"
            "прошивки (.apj) та двопрохідної заливки параметрів MAVLink\n"
            "для польотних контролерів, сумісних з ArduPilot / Pixhawk.\n\n"
            "Використані відкриті компоненти:\n"
            "• pymavlink (LGPLv3)\n"
            "• uploader.py — PX4 / ArduPilot (BSD-3-Clause / GPLv3)\n"
            "• pyserial, lxml (BSD-3-Clause), fastcrc (MIT)\n\n"
            "УВАГА: ПЗ надається «як є» (AS IS) без жодних гарантій.\n"
            "Перед прошивкою обов'язково знімайте пропелери та від'єднуйте\n"
            "силовий акумулятор."
        )
        messagebox.showinfo(f"Про {APP_NAME}", info)

    def _browse_fw(self):
        p = filedialog.askopenfilename(
            title="Оберіть файл прошивки (.apj)",
            initialdir=str(BASE_DIR),
            filetypes=[("Firmware files", "*.apj"), ("Усі файли", "*.*")],
        )
        if p:
            self.fw_var.set(p)

    def _browse_param(self):
        p = filedialog.askopenfilename(
            title="Оберіть файл параметрів",
            initialdir=str(BASE_DIR),
            filetypes=[("Файли параметрів", "*.param *.parm *.txt"), ("Усі файли", "*.*")],
        )
        if p:
            self.param_var.set(p)

    def refresh_ports(self):
        ports = sorted(serial.tools.list_ports.comports(), key=rank_port)
        items = ["Авто"] + [f"{p.device} — {p.description}" for p in ports]
        self.port_combo["values"] = items
        if self.port_var.get() not in items:
            self.port_var.set("Авто")

    def _get_selected_port(self) -> str | None:
        val = self.port_var.get().strip()
        if not val or val == "Авто":
            return None
        return val.split("—", 1)[0].strip()

    def _clear_log(self):
        self.log_box.configure(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.configure(state="disabled")

    def log(self, msg: str):
        self.log_queue.put(("log", msg))

    def set_progress(self, value: float, status: str):
        self.log_queue.put(("prog", (value, status)))

    def _set_running(self, running: bool):
        state_idle = "disabled" if running else "normal"
        self.btn_prep.configure(state=state_idle)
        self.btn_run.configure(state=state_idle)
        self.btn_stop.configure(state="normal" if running else "disabled")

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.log_queue.get_nowait()
                if kind == "log":
                    self.log_box.configure(state="normal")
                    self.log_box.insert("end", payload + "\n")
                    self.log_box.see("end")
                    self.log_box.configure(state="disabled")
                elif kind == "prog":
                    val, st = payload
                    self.progress["value"] = val
                    self.status_var.set(st)
                elif kind == "done":
                    self._set_running(False)
                    ok, summary = payload
                    if summary:
                        if ok:
                            messagebox.showinfo("Завершено", summary)
                        else:
                            messagebox.showwarning("Увага", summary)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    def _validate_inputs(self, need_fw: bool) -> tuple[Path | None, Path] | None:
        param_str = self.param_var.get().strip()
        if not param_str or not Path(param_str).is_file():
            messagebox.showerror("Помилка", "Оберіть існуючий файл параметрів (.param / .parm).")
            return None

        fw_path = None
        if need_fw:
            fw_str = self.fw_var.get().strip()
            if not fw_str or not Path(fw_str).is_file():
                messagebox.showerror("Помилка", "Оберіть існуючий файл прошивки (.apj).")
                return None
            fw_path = Path(fw_str)
            if not (RESOURCE_DIR / "uploader.py").is_file():
                messagebox.showerror("Помилка", f"Не знайдено uploader.py у {RESOURCE_DIR}.")
                return None

        return fw_path, Path(param_str)

    def start_prepare_only(self):
        res = self._validate_inputs(need_fw=False)
        if not res:
            return
        _, param_path = res
        self.stop_event.clear()
        self._set_running(True)
        self.worker = threading.Thread(
            target=self._run_prepare_only,
            args=(param_path, self.strip_calib_var.get()),
            daemon=True,
        )
        self.worker.start()

    def start_full_process(self):
        skip_flash = self.skip_flash_var.get()
        res = self._validate_inputs(need_fw=not skip_flash)
        if not res:
            return
        fw_path, param_path = res
        self.stop_event.clear()
        self._set_running(True)
        self.worker = threading.Thread(
            target=self._run_full_pipeline,
            args=(fw_path, param_path, self._get_selected_port(), self.strip_calib_var.get(), skip_flash),
            daemon=True,
        )
        self.worker.start()

    def stop_process(self):
        self.stop_event.set()
        self.log("[СТОП] Отримано запит на зупинку...")
        if self.active_proc and self.active_proc.poll() is None:
            try:
                self.active_proc.terminate()
            except Exception:
                pass

    def _on_close(self):
        if self.worker and self.worker.is_alive():
            if not messagebox.askyesno("Підтвердження", "Процес ще виконується. Зупинити та закрити вікно?"):
                return
            self.stop_process()
        self.root.destroy()

    def _log_prep_stats(self, target_params: dict[str, float], stats: dict):
        self.log("=== Підготовка параметрів завершена ===")
        self.log(f"Очищений файл: {stats['clean_path']}")
        self.log(f"Видалені параметри збережено у: {stats['removed_path']}")
        self.log(
            f"Збережено: {stats['kept_count']} | Видалено read-only/системних: {stats['removed_ro_count']} | "
            f"Видалено калібрувань: {stats['removed_cal_count']} (залишено: {stats['kept_cal_count']})"
        )
        self.log(f"Параметрів GPS*: {stats['gps_count']}")
        for k in ("GPS1_TYPE", "GPS2_TYPE", "GPS_PRIMARY", "GPS_AUTO_CONFIG"):
            if k in target_params:
                self.log(f"  {k} = {target_params[k]:g}")
        if stats["legacy_gps"]:
            self.log(f"[УВАГА] Знайдено старі імена GPS (до версії 4.6): {', '.join(stats['legacy_gps'])}")

    def _run_prepare_only(self, param_path: Path, strip_calib: bool):
        try:
            self.set_progress(25, "Очищення файлу параметрів...")
            _, target_params, stats = prepare_param_file(param_path, strip_calib)
            self._log_prep_stats(target_params, stats)
            self.set_progress(100, "Підготовка параметрів завершена")
            self.log_queue.put(("done", (True, f"Файл підготовлено:\n{stats['clean_path']}")))
        except Exception as e:
            self.log(f"[ПОМИЛКА] {e}")
            self.set_progress(0, "Помилка підготовки")
            self.log_queue.put(("done", (False, f"Помилка: {e}")))

    def _connect_mavlink(self, explicit_port: str | None, timeout_total: int = 60):
        deadline = time.time() + timeout_total
        while time.time() < deadline:
            if self.stop_event.is_set():
                raise InterruptedError("Зупинено користувачем")
            if explicit_port:
                candidates = [explicit_port]
            else:
                ports = sorted(serial.tools.list_ports.comports(), key=rank_port)
                candidates = [p.device for p in ports]

            for dev in candidates:
                if self.stop_event.is_set():
                    raise InterruptedError("Зупинено користувачем")
                conn = None
                try:
                    self.log(f"[MAVLink] Перевірка {dev}...")
                    conn = mavutil.mavlink_connection(dev, baud=115200)
                    hb = conn.wait_heartbeat(timeout=3.5)
                    if hb:
                        self.log(f"[MAVLink] Підключено до {dev} (sysid={conn.target_system})")
                        return conn
                except Exception:
                    pass
                if conn:
                    try:
                        conn.close()
                    except Exception:
                        pass
            time.sleep(1.5)
        raise RuntimeError("Не вдалося підключитись до контролера по MAVLink. Закрийте GCS (Mission Planner / QGC) і перевірте USB.")

    def _download_params(self, conn) -> dict[str, tuple[float, int]]:
        while conn.recv_match(blocking=False) is not None:
            pass
        conn.mav.param_request_list_send(conn.target_system, conn.target_component)
        params: dict[str, tuple[float, int]] = {}
        total = None
        last_rx = time.time()
        start = time.time()

        while True:
            if self.stop_event.is_set():
                raise InterruptedError("Зупинено користувачем")
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
            now = time.time()
            if msg is not None:
                name = norm_param_id(msg.param_id)
                params[name] = (float(msg.param_value), int(msg.param_type))
                if msg.param_count > 0:
                    total = int(msg.param_count)
                last_rx = now
                if total and len(params) >= total:
                    break
            elif params and (now - last_rx) >= 3.0:
                break
            if (now - start) > 35.0:
                break

        self.log(f"[PARAMS] Зчитано з борту: {len(params)}" + (f" / {total}" if total else ""))
        return params

    def _write_pass(self, conn, target: dict[str, float], current: dict[str, tuple[float, int]]):
        written, skipped = 0, 0
        missing, failed = [], []

        for name, want in target.items():
            if self.stop_event.is_set():
                raise InterruptedError("Зупинено користувачем")
            if name not in current:
                missing.append(name)
                continue
            cur_val, ptype = current[name]
            if math.isclose(cur_val, want, rel_tol=1e-5, abs_tol=1e-4):
                skipped += 1
                continue

            ok = False
            for _ in range(3):
                conn.mav.param_set_send(
                    conn.target_system,
                    conn.target_component,
                    name.encode("utf-8"),
                    float(want),
                    ptype,
                )
                t0 = time.time()
                while time.time() - t0 < 1.5:
                    msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                    if msg and norm_param_id(msg.param_id) == name:
                        if math.isclose(float(msg.param_value), want, rel_tol=1e-4, abs_tol=1e-3):
                            ok = True
                        break
                if ok:
                    break
            if ok:
                written += 1
            else:
                failed.append(name)

        return written, skipped, missing, failed

    def _reboot_fc(self, conn, wait_sec: int = 8):
        self.log("[REBOOT] Перезавантаження контролера...")
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        time.sleep(0.5)
        try:
            conn.close()
        except Exception:
            pass
        for _ in range(wait_sec):
            if self.stop_event.is_set():
                raise InterruptedError("Зупинено користувачем")
            time.sleep(1)

    def _run_uploader(self, fw_path: Path, explicit_port: str | None):
        if getattr(sys, "frozen", False):
            cmd = [sys.executable, "--run-uploader"]
        else:
            cmd = [sys.executable, "-u", str(Path(__file__).resolve()), "--run-uploader"]
        if explicit_port:
            cmd += ["--port", explicit_port]
        cmd.append(str(fw_path))

        self.log(f"[FLASH] Запуск вбудованого завантажувача для {fw_path.name}...")
        self.log("[FLASH] Підключіть польотний контролер до USB (або перепідключіть, якщо він уже вставлений).")

        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.active_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            cwd=str(BASE_DIR),
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
            creationflags=creationflags,
        )

        buf = ""
        last_cr_log = 0.0
        assert self.active_proc.stdout is not None
        while True:
            ch = self.active_proc.stdout.read(1)
            if not ch:
                break
            c = ch.decode("utf-8", errors="replace")
            if c == "\n":
                line = buf.strip()
                if line:
                    self.log(f"  {line}")
                buf = ""
            elif c == "\r":
                line = buf.strip()
                now = time.time()
                if line and (now - last_cr_log > 0.6 or "100%" in line):
                    self.log(f"  {line}")
                    last_cr_log = now
                buf = ""
            else:
                buf += c

        if buf.strip():
            self.log(f"  {buf.strip()}")

        rc = self.active_proc.wait()
        self.active_proc = None
        if self.stop_event.is_set():
            raise InterruptedError("Прошивку зупинено користувачем")
        if rc != 0:
            raise RuntimeError(f"Завантажувач завершився з кодом {rc}")

    def _run_full_pipeline(
        self,
        fw_path: Path | None,
        param_path: Path,
        explicit_port: str | None,
        strip_calib: bool,
        skip_flash: bool,
    ):
        try:
            self.set_progress(10, "Етап 1/5: Підготовка параметрів...")
            _, target_params, stats = prepare_param_file(param_path, strip_calib)
            self._log_prep_stats(target_params, stats)

            if not skip_flash and fw_path is not None:
                self.set_progress(25, "Етап 2/5: Прошивка .apj (підключіть USB)...")
                self._run_uploader(fw_path, explicit_port)
                self.log("[FLASH] Прошивка завершена. Очікування запуску контролера (8 сек)...")
                for _ in range(8):
                    if self.stop_event.is_set():
                        raise InterruptedError("Зупинено користувачем")
                    time.sleep(1)
            else:
                self.log("[FLASH] Прошивка .apj пропущена за налаштуванням.")

            self.set_progress(50, "Етап 3/5: Заливка параметрів (Прохід 1)...")
            conn = self._connect_mavlink(explicit_port)
            cur1 = self._download_params(conn)
            w1, s1, m1, f1 = self._write_pass(conn, target_params, cur1)
            self.log(f"[Прохід 1] Записано: {w1}, вже збігались: {s1}, ще відсутні: {len(m1)}, помилок: {len(f1)}")
            self._reboot_fc(conn, wait_sec=8)

            self.set_progress(75, "Етап 4/5: Дозаписування залежних параметрів (Прохід 2)...")
            conn = self._connect_mavlink(explicit_port)
            cur2 = self._download_params(conn)
            w2, s2, m2, f2 = self._write_pass(conn, target_params, cur2)
            self.log(f"[Прохід 2] Записано: {w2}, вже збігались: {s2}, відсутні: {len(m2)}, помилок: {len(f2)}")
            self._reboot_fc(conn, wait_sec=8)

            self.set_progress(90, "Етап 5/5: Фінальна звірка параметрів та GPS...")
            conn = self._connect_mavlink(explicit_port)
            final_fc = self._download_params(conn)
            conn.close()

            mismatches = []
            missing_final = []
            for k, want in target_params.items():
                if k not in final_fc:
                    missing_final.append((k, want))
                elif not math.isclose(final_fc[k][0], want, rel_tol=1e-4, abs_tol=1e-3):
                    mismatches.append((k, want, final_fc[k][0]))

            lines = [
                f"Всього параметрів: {len(target_params)}",
                f"Збіглись: {len(target_params) - len(mismatches) - len(missing_final)}",
                f"Розбіжностей: {len(mismatches)}",
                f"Відсутніх на борту: {len(missing_final)}",
                "",
                "--- Перевірка GPS та SERIAL ---",
            ]
            for k in sorted(
                k for k in target_params
                if k.upper().startswith("GPS") or (k.upper().startswith("SERIAL") and "PROTOCOL" in k.upper())
            ):
                want = target_params[k]
                if k not in final_fc:
                    lines.append(f"{k}: MISSING (очікувалось {want:g})")
                else:
                    got = final_fc[k][0]
                    ok = math.isclose(got, want, rel_tol=1e-4, abs_tol=1e-3)
                    lines.append(f"{k}: {'OK' if ok else 'MISMATCH'} (на борту {got:g}, у файлі {want:g})")

            if mismatches:
                lines.append("\n--- Усі розбіжності ---")
                for k, want, got in mismatches:
                    lines.append(f"{k}: на борту={got:g}, у файлі={want:g}")

            if missing_final:
                lines.append("\n--- Відсутні на борту ---")
                for k, want in missing_final:
                    lines.append(f"{k}: у файлі={want:g}")

            report_path = param_path.with_name(f"{param_path.stem}_verify_report.txt")
            report_text = "\n".join(lines) + "\n"
            report_path.write_text(report_text, encoding="utf-8")

            self.log("\n" + report_text)
            self.log(f"Звіт збережено у: {report_path}")
            self.set_progress(100, "Готово! Усі етапи завершено.")

            summary = (
                f"Готово!\nЗбіглись: {len(target_params) - len(mismatches) - len(missing_final)} / {len(target_params)}\n"
                f"Розбіжностей: {len(mismatches)} | Відсутніх: {len(missing_final)}\n\n"
                f"Звіт: {report_path}"
            )
            self.log_queue.put(("done", (len(mismatches) == 0, summary)))

        except InterruptedError as e:
            self.log(f"[ЗУПИНЕНО] {e}")
            self.set_progress(0, "Зупинено користувачем")
            self.log_queue.put(("done", (False, str(e))))
        except Exception as e:
            self.log(f"[ПОМИЛКА] {e}")
            self.set_progress(0, "Завершено з помилкою")
            self.log_queue.put(("done", (False, f"Помилка виконання:\n{e}")))


def main():
    _maybe_run_embedded_uploader()
    root = tk.Tk()
    try:
        style = ttk.Style()
        if "vista" in style.theme_names():
            style.theme_use("vista")
    except Exception:
        pass
    BittecFlasherGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()