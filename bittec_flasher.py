# Bittec Flasher — Utility for flashing firmware (.apj / STM32 DFU .hex/.bin)
# and two-pass MAVLink parameter upload
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
import shutil
import struct
import subprocess
import sys
import tempfile
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
APP_VERSION = "1.2.0"
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
import json

CFG_PATH = (
    (Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent)
    / "bittec_flasher_cfg.json"
)

UI_TEXT = {
    "ua": {
        "win_title": "{app} v{ver} — Прошивка (.apj / DFU) та параметри MAVLink",
        "frame_files": "Файли прошивки та параметрів",
        "lbl_fw": "Прошивка (.apj / .hex / .bin):",
        "btn_browse": "Обрати...",
        "lbl_param": "Параметри (.param/.parm, опційно):",
        "frame_opts": "Підключення та параметри заливки",
        "lbl_port": "COM-порт:",
        "port_auto": "Авто",
        "btn_refresh": "Оновити порти",
        "btn_dfu": "Перевірити DFU",
        "btn_unprotect": "Зняти захист",
        "chk_calib": "Вичистити калібрування датчиків (акселерометр, гіроскоп, компас — для серії бортів)",
        "chk_servo": "Не перезаписувати налаштування сервоприводів (ігнорувати всі SERVO*)",
        "chk_backup": "Створити резервну копію параметрів перед перезаписом",
        "lbl_backup_name": "Назва файлу резерву:",
        "btn_folder": "Папка...",
        "backup_preview": "Папка резерву (створиться автоматично): {path}",
        "chk_skip_flash": "Тільки підготувати й залити параметри (пропустити прошивку .apj / .hex)",
        "chk_skip_params": "Тільки прошити контролер (пропустити підготовку й заливку параметрів)",
        "btn_prep": "Тільки підготувати параметри",
        "btn_run": "Підготувати та прошити",
        "btn_stop": "Зупинити",
        "btn_clear": "Очистити лог",
        "btn_about": "Про програму",
        "frame_log": "Журнал виконання",
        "status_ready": "Готовий до роботи",
        "fw_empty": "Режим прошивки: оберіть .apj (оновлення ArduPilot) або *_with_bl.hex (первинна прошивка DFU)",
        "fw_apj": "Режим прошивки: ArduPilot Bootloader (.apj) — для Pixhawk 6C, Matek H743 та інших плат з ArduPilot",
        "fw_dfu": "Режим прошивки: STM32 DFU (.hex / .bin) — первинна прошивка через кнопку BOOT (Matek H743 тощо)",
        "fw_bad": "Непідтримуваний формат файлу ({ext}). Оберіть .apj, .hex або .bin.",
        "about_title": "Про {app}",
        "about_body": (
            "{app} v{ver}\n{copy}\nGitHub: {repo}\nЛіцензія: GNU GPLv3\n\n"
            "Незалежна утиліта для автоматичного очищення параметрів,\n"
            "прошивки (.apj та STM32 DFU .hex/.bin) і двопрохідної заливки\n"
            "параметрів MAVLink для контролерів ArduPilot (Pixhawk 6C, Matek H743 тощо).\n\n"
            "Використані відкриті компоненти:\n• pymavlink (LGPLv3)\n"
            "• uploader.py — PX4 / ArduPilot (BSD-3-Clause / GPLv3)\n"
            "• dfu-util ({dfu}, GPLv2+) та libusb (LGPLv2.1+)\n• pyserial, lxml (BSD-3-Clause), fastcrc (MIT)\n\n"
            "УВАГА: ПЗ надається «як є» (AS IS) без жодних гарантій.\n"
            "Перед прошивкою обов'язково знімайте пропелери та від'єднуйте силовий акумулятор."
        ),
        "dfu_embedded": "вшито",
        "dfu_missing": "не знайдено",
    },
    "en": {
        "win_title": "{app} v{ver} — Firmware (.apj / DFU) & MAVLink Parameters",
        "frame_files": "Firmware and Parameter Files",
        "lbl_fw": "Firmware (.apj / .hex / .bin):",
        "btn_browse": "Browse...",
        "lbl_param": "Parameters (.param/.parm, optional):",
        "frame_opts": "Connection and Flashing Options",
        "lbl_port": "COM Port:",
        "port_auto": "Auto",
        "btn_refresh": "Refresh Ports",
        "btn_dfu": "Check DFU",
        "btn_unprotect": "Unprotect",
        "chk_calib": "Strip sensor calibrations (accelerometer, gyroscope, compass — for fleet flashing)",
        "chk_servo": "Do not overwrite servo settings (ignore all SERVO*)",
        "chk_backup": "Create parameter backup before overwriting",
        "lbl_backup_name": "Backup file name:",
        "btn_folder": "Folder...",
        "backup_preview": "Backup folder (created automatically): {path}",
        "chk_skip_flash": "Prepare and upload parameters only (skip .apj / .hex flashing)",
        "chk_skip_params": "Flash controller only (skip parameter preparation and upload)",
        "btn_prep": "Prepare Parameters Only",
        "btn_run": "Prepare & Flash",
        "btn_stop": "Stop",
        "btn_clear": "Clear Log",
        "btn_about": "About",
        "frame_log": "Execution Log",
        "status_ready": "Ready",
        "fw_empty": "Flashing mode: select .apj (ArduPilot update) or *_with_bl.hex (initial DFU flash)",
        "fw_apj": "Flashing mode: ArduPilot Bootloader (.apj) — for Pixhawk 6C, Matek H743 and other ArduPilot boards",
        "fw_dfu": "Flashing mode: STM32 DFU (.hex / .bin) — initial flash via BOOT button (Matek H743, etc.)",
        "fw_bad": "Unsupported file format ({ext}). Select .apj, .hex or .bin.",
        "about_title": "About {app}",
        "about_body": (
            "{app} v{ver}\n{copy}\nGitHub: {repo}\nLicense: GNU GPLv3\n\n"
            "Standalone utility for automatic parameter cleanup,\n"
            "firmware flashing (.apj and STM32 DFU .hex/.bin) and multi-pass\n"
            "MAVLink parameter upload for ArduPilot controllers (Pixhawk 6C, Matek H743, etc.).\n\n"
            "Open-source components used:\n• pymavlink (LGPLv3)\n"
            "• uploader.py — PX4 / ArduPilot (BSD-3-Clause / GPLv3)\n"
            "• dfu-util ({dfu}, GPLv2+) and libusb (LGPLv2.1+)\n• pyserial, lxml (BSD-3-Clause), fastcrc (MIT)\n\n"
            "DISCLAIMER: Software is provided AS IS without warranty of any kind.\n"
            "Always remove propellers and disconnect the main battery before flashing."
        ),
        "dfu_embedded": "embedded",
        "dfu_missing": "not found",
    },
}

UA_TO_EN_EXACT = {
    "Готовий до роботи": "Ready",
    "Очищення файлу параметрів...": "Cleaning parameter file...",
    "Підготовка параметрів завершена": "Parameter preparation completed",
    "Помилка підготовки": "Preparation error",
    "Етап 1/5: Підготовка параметрів...": "Step 1/5: Preparing parameters...",
    "Створення резервної копії параметрів...": "Creating parameter backup...",
    "Етап 2/5: Первинна прошивка STM32 DFU...": "Step 2/5: Initial STM32 DFU flashing...",
    "Етап 2/5: Прошивка .apj (підключіть USB)...": "Step 2/5: Flashing .apj (connect USB)...",
    "Готово! Прошивку завершено.": "Done! Flashing completed.",
    "Прошивку контролера успішно завершено!": "Controller flashing completed successfully!",
    "Етап 3/5: Заливка параметрів (Прохід 1)...": "Step 3/5: Uploading parameters (Pass 1)...",
    "Очікування перепідключення USB (без кнопки BOOT)...": "Waiting for USB reconnect (without BOOT button)...",
    "Етап 4/5: Дозаписування залежних параметрів (Прохід 2)...": "Step 4/5: Writing dependent parameters (Pass 2)...",
    "Етап 5/5: Контрольна перевірка параметрів (Прохід 3)...": "Step 5/5: Verifying parameters (Pass 3)...",
    "Помилка": "Error",
    "Увага": "Warning",
    "Готово": "Done",
    "Підтвердження": "Confirmation",
    "Зупинено": "Stopped",
    "Не можна одночасно пропустити і прошивку, і параметри.": "Cannot skip both firmware flashing and parameter upload at the same time.",
    "Оберіть файл прошивки (.apj, .hex або .bin).": "Please select a firmware file (.apj, .hex, or .bin).",
    "Файл прошивки не знайдено.": "Firmware file not found.",
    "Оберіть файл параметрів (.param або .parm).": "Please select a parameter file (.param or .parm).",
    "Файл параметрів не знайдено.": "Parameter file not found.",
    "Не знайдено dfu-util.exe для перевірки DFU.": "dfu-util.exe was not found for DFU check.",
    "Захист пам'яті (RDP/WRP) успішно знято!\n\nВід'єднайте USB, затисніть BOOT, підключіть USB і запустіть прошивку.": "Memory protection (RDP/WRP) removed successfully!\n\nDisconnect USB, hold BOOT, reconnect USB, and start flashing.",
    "Контролер заблоковано (RDP/WRP). Виконати повне стирання чипа і зняття захисту?": "Controller memory is locked (RDP/WRP). Perform full chip erase and unprotect?",
    "Після DFU-прошивки від'єднайте кабель USB, відпустіть кнопку BOOT, підключіть USB знову та натисніть OK.": "After DFU flashing, disconnect the USB cable, release the BOOT button, reconnect USB, and click OK.",
    "Виконується операція. Зупинити та закрити програму?": "An operation is currently running. Stop and close the application?",
}

def _load_lang_cfg() -> str:
    try:
        if CFG_PATH.is_file():
            data = json.loads(CFG_PATH.read_text(encoding="utf-8"))
            if data.get("lang") in ("ua", "en"):
                return data["lang"]
    except Exception:
        pass
    return "ua"

def _save_lang_cfg(lang: str) -> None:
    try:
        CFG_PATH.write_text(json.dumps({"lang": lang}, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:
        pass

def _translate_dynamic_text(text: str, lang: str) -> str:
    if lang != "en" or not isinstance(text, str) or not text:
        return text
    if text in UA_TO_EN_EXACT:
        return UA_TO_EN_EXACT[text]
    out = text.replace(" [SLCAN / вторинний]", " [SLCAN / secondary]")
    if out.startswith("Файл підготовлено:\n"):
        out = "File prepared:\n" + out[len("Файл підготовлено:\n"):]
    if out.startswith("Помилка: "):
        out = "Error: " + out[len("Помилка: "):]
    if out.startswith("Виявлено пристрій STM32 DFU"):
        out = out.replace("Виявлено пристрій STM32 DFU", "STM32 DFU device detected")
    if out.startswith("Пристрій STM32 DFU не знайдено"):
        out = out.replace("Пристрій STM32 DFU не знайдено", "STM32 DFU device not found")
    return out

_ORIG_MB_INFO, _ORIG_MB_ERR, _ORIG_MB_WARN, _ORIG_MB_YESNO = (
    messagebox.showinfo, messagebox.showerror, messagebox.showwarning, messagebox.askyesno
)

def _install_i18n_messagebox(lang_var: tk.StringVar) -> None:
    def _wrap(fn):
        def inner(title, message, *args, **kwargs):
            lang = lang_var.get()
            return fn(_translate_dynamic_text(title, lang), _translate_dynamic_text(message, lang), *args, **kwargs)
        return inner
    messagebox.showinfo, messagebox.showerror, messagebox.showwarning, messagebox.askyesno = (
        _wrap(_ORIG_MB_INFO), _wrap(_ORIG_MB_ERR), _wrap(_ORIG_MB_WARN), _wrap(_ORIG_MB_YESNO)
    )
SERVO_RE = re.compile(r"^SERVO.*$", re.IGNORECASE)


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


def find_dfu_util() -> Path | None:
    candidates = [
        RESOURCE_DIR / "dfu_bin" / "dfu-util.exe",
        RESOURCE_DIR / "dfu-util.exe",
        BASE_DIR / "dfu_bin" / "dfu-util.exe",
        BASE_DIR / "dfu-util.exe",
    ]
    for cand in candidates:
        if cand.is_file():
            return cand
    found = shutil.which("dfu-util")
    return Path(found) if found else None


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


def is_secondary_iface(port_info) -> bool:
    desc = f"{port_info.description or ''} {getattr(port_info, 'interface', None) or ''}".lower()
    hwid = (port_info.hwid or "").lower()
    loc = (getattr(port_info, "location", None) or "").lower()
    if "slcan" in desc or "mi_02" in hwid:
        return True
    if re.search(r":x\.2\b", hwid) or re.search(r":x\.2\b", loc):
        return True
    return False


def is_known_fc_vid(port_info) -> bool:
    hwid = (port_info.hwid or "").lower()
    return any(
        tok in hwid
        for tok in (
            "1209:5740",
            "1209:5741",
            "2dae:",
            "3162:",
            "vid_1209&pid_5740",
            "vid_1209&pid_5741",
            "vid_2dae",
            "vid_3162",
        )
    )


def rank_port(port_info) -> int:
    desc = f"{port_info.description or ''} {port_info.manufacturer or ''} {port_info.hwid or ''}".lower()
    hwid = (port_info.hwid or "").lower()
    if is_secondary_iface(port_info):
        return 3
    if is_known_fc_vid(port_info) or any(
        k in desc for k in ("mavlink", "pixhawk", "ardupilot", "holybro", "matek", "chibios", "cube")
    ):
        return 0
    if "0483:" in hwid or "vid_0483" in hwid or "usb" in desc or "stm" in desc:
        return 1
    return 2


def format_port_label(port_info) -> str:
    tag = ""
    if is_secondary_iface(port_info):
        tag = " [SLCAN / вторинний]"
    elif is_known_fc_vid(port_info):
        tag = " [ArduPilot / Matek / Pixhawk]"
    return f"{port_info.device} — {port_info.description}{tag}"


def convert_intel_hex_to_bin(hex_path: Path, out_bin_path: Path) -> tuple[int, int]:
    upper_addr = 0
    chunks: list[tuple[int, bytes]] = []

    with hex_path.open("r", encoding="ascii", errors="ignore") as f:
        for line_no, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError(f"Некоректний рядок {line_no} у HEX-файлі (не починається з ':').")
            try:
                raw_bytes = bytes.fromhex(line[1:])
            except ValueError as e:
                raise ValueError(f"Помилка розбору HEX у рядку {line_no}: {e}") from e
            if len(raw_bytes) < 5 or (sum(raw_bytes) & 0xFF) != 0:
                raise ValueError(f"Помилка контрольної суми HEX у рядку {line_no}.")

            length = raw_bytes[0]
            addr16 = (raw_bytes[1] << 8) | raw_bytes[2]
            rtype = raw_bytes[3]
            data = raw_bytes[4 : 4 + length]
            if len(data) != length:
                raise ValueError(f"Неповні дані у рядку {line_no} HEX-файлу.")

            if rtype == 0x00:
                chunks.append((upper_addr + addr16, data))
            elif rtype == 0x01:
                break
            elif rtype == 0x02:
                upper_addr = int.from_bytes(data, "big") << 4
            elif rtype == 0x04:
                upper_addr = int.from_bytes(data, "big") << 16
            elif rtype in (0x03, 0x05):
                continue

    if not chunks:
        raise ValueError("HEX-файл не містить блоків прошивки.")

    min_addr = min(a for a, _ in chunks)
    max_addr = max(a + len(d) for a, d in chunks)
    span = max_addr - min_addr

    if span <= 0 or span > 8 * 1024 * 1024:
        raise ValueError(f"Підозрілий розмір образу ({span} байт, старт 0x{min_addr:08X}). Перевірте HEX-файл.")

    buf = bytearray(b"\xff") * span
    for a, d in chunks:
        off = a - min_addr
        buf[off : off + len(d)] = d

    out_bin_path.write_bytes(buf)
    return min_addr, span


def prepare_param_file(src_path: Path, strip_calib: bool, strip_servo: bool = False, removed_servo_out: list[str] | None = None) -> tuple[Path, dict[str, float], dict]:
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

        is_servo = bool(SERVO_RE.match(name))
        if is_servo and strip_servo:
            if removed_servo_out is not None:
                removed_servo_out.append(name)
            continue

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
        self.root.title(f"{APP_NAME} v{APP_VERSION} — Прошивка (.apj / DFU) та параметри MAVLink")
        self.root.geometry("820x650")
        self.root.minsize(730, 540)

        self.log_queue: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.active_proc: subprocess.Popen | None = None

        self.fw_var = tk.StringVar(value=find_default_file(BASE_DIR, ("*.apj", "*_with_bl.hex", "*.hex")))
        self.param_var = tk.StringVar(
            value=find_default_file(BASE_DIR, ("*.param", "*.parm"), skip_suffix="_clean")
        )
        self.port_var = tk.StringVar(value="Авто")
        self.lang_var = tk.StringVar(value=_load_lang_cfg())
        _install_i18n_messagebox(self.lang_var)
        self.strip_calib_var = tk.BooleanVar(value=False)
        self.strip_servo_var = tk.BooleanVar(value=False)
        self.backup_enable_var = tk.BooleanVar(value=False)
        self.backup_name_var = tk.StringVar(value=self._default_backup_filename())
        init_param = self.param_var.get().strip()
        init_bdir = str(Path(init_param).parent) if init_param else str(BASE_DIR)
        self.backup_dir_var = tk.StringVar(value=init_bdir)
        self.backup_path_preview_var = tk.StringVar(value="")
        self._backup_dir_custom = False
        self.skip_flash_var = tk.BooleanVar(value=False)
        self.skip_params_var = tk.BooleanVar(value=False)
        self.fw_mode_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Готовий до роботи")

        self._build_ui()
        self._apply_language(save=False)
        self.param_var.trace_add("write", lambda *_: self._sync_backup_dir_from_param())
        self.backup_dir_var.trace_add("write", lambda *_: self._update_backup_preview())
        self._update_backup_preview()
        self.fw_var.trace_add("write", lambda *_: self._update_fw_mode_label())
        self._update_fw_mode_label()
        self.refresh_ports()
        self.root.after(100, self._poll_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    @staticmethod
    def _create_flag_ua() -> tk.PhotoImage:
        img = tk.PhotoImage(width=20, height=14)
        img.put("#0057B7", to=(0, 0, 20, 7))
        img.put("#FFD700", to=(0, 7, 20, 14))
        return img

    @staticmethod
    def _create_flag_uk() -> tk.PhotoImage:
        img = tk.PhotoImage(width=20, height=14)
        img.put("#012169", to=(0, 0, 20, 14))
        for x in range(20):
            y1 = int(round(x * 13 / 19))
            y2 = 13 - y1
            for dy in (-1, 0, 1):
                if 0 <= y1 + dy < 14:
                    img.put("#FFFFFF", to=(x, y1 + dy, x + 1, y1 + dy + 1))
                if 0 <= y2 + dy < 14:
                    img.put("#FFFFFF", to=(x, y2 + dy, x + 1, y2 + dy + 1))
            img.put("#C8102E", to=(x, y1, x + 1, y1 + 1))
            img.put("#C8102E", to=(x, y2, x + 1, y2 + 1))
        img.put("#FFFFFF", to=(0, 5, 20, 9))
        img.put("#FFFFFF", to=(8, 0, 12, 14))
        img.put("#C8102E", to=(0, 6, 20, 8))
        img.put("#C8102E", to=(9, 0, 11, 14))
        return img

    def _t(self, key: str, **kwargs) -> str:
        lang = self.lang_var.get() if hasattr(self, "lang_var") else "ua"
        raw = UI_TEXT.get(lang, UI_TEXT["ua"]).get(key, UI_TEXT["ua"].get(key, key))
        return raw.format(**kwargs) if kwargs else raw

    def _set_language(self, lang: str):
        if lang in ("ua", "en") and self.lang_var.get() != lang:
            self.lang_var.set(lang)
            self._apply_language(save=True)

    def _apply_language(self, save: bool = True):
        lang = self.lang_var.get()
        if save:
            _save_lang_cfg(lang)
        self.root.title(self._t("win_title", app=APP_NAME, ver=APP_VERSION))
        self.frame_files.configure(text=self._t("frame_files"))
        self.lbl_fw.configure(text=self._t("lbl_fw"))
        self.btn_browse_fw.configure(text=self._t("btn_browse"))
        self.lbl_param.configure(text=self._t("lbl_param"))
        self.btn_browse_param.configure(text=self._t("btn_browse"))
        self.frame_opts.configure(text=self._t("frame_opts"))
        self.lbl_port.configure(text=self._t("lbl_port"))
        self.btn_refresh_ports.configure(text=self._t("btn_refresh"))
        self.btn_check_dfu.configure(text=self._t("btn_dfu"))
        self.btn_unprotect_dfu.configure(text=self._t("btn_unprotect"))
        self.chk_strip_calib.configure(text=self._t("chk_calib"))
        self.chk_strip_servo.configure(text=self._t("chk_servo"))
        self.chk_backup_enable.configure(text=self._t("chk_backup"))
        self.lbl_backup_name.configure(text=self._t("lbl_backup_name"))
        self.btn_browse_backup.configure(text=self._t("btn_folder"))
        self.chk_skip_flash.configure(text=self._t("chk_skip_flash"))
        self.chk_skip_params.configure(text=self._t("chk_skip_params"))
        self.btn_prep.configure(text=self._t("btn_prep"))
        self.btn_run.configure(text=self._t("btn_run"))
        self.btn_stop.configure(text=self._t("btn_stop"))
        self.btn_about.configure(text=self._t("btn_about"))
        self.btn_clear_log.configure(text=self._t("btn_clear"))
        self.frame_log.configure(text=self._t("frame_log"))
        if lang == "ua":
            self.btn_lang_ua.configure(relief="sunken", bg="#dbeafe", font=("Segoe UI", 9, "bold"))
            self.btn_lang_en.configure(relief="raised", bg="#f3f4f6", font=("Segoe UI", 9))
        else:
            self.btn_lang_en.configure(relief="sunken", bg="#dbeafe", font=("Segoe UI", 9, "bold"))
            self.btn_lang_ua.configure(relief="raised", bg="#f3f4f6", font=("Segoe UI", 9))
        cur_port = self.port_var.get()
        if cur_port in ("Авто", "Auto"):
            self.port_var.set(self._t("port_auto"))
        elif " [SLCAN / " in cur_port:
            self.port_var.set(cur_port.replace(" [SLCAN / вторинний]", " [SLCAN / secondary]") if lang == "en" else cur_port.replace(" [SLCAN / secondary]", " [SLCAN / вторинний]"))
        vals = list(self.port_combo["values"])
        if vals:
            self.port_combo["values"] = [
                self._t("port_auto") if v in ("Авто", "Auto") else (v.replace(" [SLCAN / вторинний]", " [SLCAN / secondary]") if lang == "en" else v.replace(" [SLCAN / secondary]", " [SLCAN / вторинний]"))
                for v in vals
            ]
        st = self.status_var.get()
        self.status_var.set(self._t("status_ready") if st in ("Готовий до роботи", "Ready") else _translate_dynamic_text(st, lang))
        self._update_fw_mode_label()
        self._update_backup_preview()

    def _build_ui(self):
        pad = {"padx": 10, "pady": 5}
        top_bar = tk.Frame(self.root)
        top_bar.pack(fill="x", padx=10, pady=(6, 0))
        lang_box = tk.Frame(top_bar)
        lang_box.pack(side="right")
        self._flag_ua_img = self._create_flag_ua()
        self._flag_uk_img = self._create_flag_uk()
        self.btn_lang_ua = tk.Button(lang_box, text=" UA", image=self._flag_ua_img, compound="left", padx=6, pady=1, cursor="hand2", command=lambda: self._set_language("ua"))
        self.btn_lang_ua.pack(side="left", padx=(0, 4))
        self.btn_lang_en = tk.Button(lang_box, text=" ENG", image=self._flag_uk_img, compound="left", padx=6, pady=1, cursor="hand2", command=lambda: self._set_language("en"))
        self.btn_lang_en.pack(side="left")

        self.frame_files = ttk.LabelFrame(self.root, text=self._t("frame_files"), padding=10)
        self.frame_files.pack(fill="x", **pad)
        self.frame_files.columnconfigure(1, weight=1)
        self.lbl_fw = ttk.Label(self.frame_files, text=self._t("lbl_fw"))
        self.lbl_fw.grid(row=0, column=0, sticky="w", pady=4)
        ttk.Entry(self.frame_files, textvariable=self.fw_var).grid(row=0, column=1, sticky="ew", padx=6, pady=4)
        self.btn_browse_fw = ttk.Button(self.frame_files, text=self._t("btn_browse"), command=self._browse_fw)
        self.btn_browse_fw.grid(row=0, column=2, pady=4)
        ttk.Label(self.frame_files, textvariable=self.fw_mode_var, foreground="#1f5aa6").grid(row=1, column=0, columnspan=3, sticky="w", pady=(0, 4))
        self.lbl_param = ttk.Label(self.frame_files, text=self._t("lbl_param"))
        self.lbl_param.grid(row=2, column=0, sticky="w", pady=4)
        ttk.Entry(self.frame_files, textvariable=self.param_var).grid(row=2, column=1, sticky="ew", padx=6, pady=4)
        self.btn_browse_param = ttk.Button(self.frame_files, text=self._t("btn_browse"), command=self._browse_param)
        self.btn_browse_param.grid(row=2, column=2, pady=4)

        self.frame_opts = ttk.LabelFrame(self.root, text=self._t("frame_opts"), padding=10)
        self.frame_opts.pack(fill="x", **pad)
        self.frame_opts.columnconfigure(1, weight=1)
        self.lbl_port = ttk.Label(self.frame_opts, text=self._t("lbl_port"))
        self.lbl_port.grid(row=0, column=0, sticky="w", pady=4)
        self.port_combo = ttk.Combobox(self.frame_opts, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=6, pady=4)
        port_btn_box = tk.Frame(self.frame_opts)
        port_btn_box.grid(row=0, column=2, pady=4)
        self.btn_refresh_ports = ttk.Button(port_btn_box, text=self._t("btn_refresh"), command=self.refresh_ports)
        self.btn_refresh_ports.pack(side="left", padx=(0, 6))
        self.btn_check_dfu = ttk.Button(port_btn_box, text=self._t("btn_dfu"), command=self.check_dfu_clicked)
        self.btn_check_dfu.pack(side="left", padx=(0, 6))
        self.btn_unprotect_dfu = ttk.Button(port_btn_box, text=self._t("btn_unprotect"), command=self.unprotect_dfu_clicked)
        self.btn_unprotect_dfu.pack(side="left")

        self.chk_strip_calib = ttk.Checkbutton(self.frame_opts, text=self._t("chk_calib"), variable=self.strip_calib_var)
        self.chk_strip_calib.grid(row=1, column=0, columnspan=3, sticky="w", pady=2)
        self.chk_strip_servo = ttk.Checkbutton(self.frame_opts, text=self._t("chk_servo"), variable=self.strip_servo_var)
        self.chk_strip_servo.grid(row=2, column=0, columnspan=3, sticky="w", pady=2)
        self.chk_backup_enable = ttk.Checkbutton(self.frame_opts, text=self._t("chk_backup"), variable=self.backup_enable_var, command=self._on_backup_toggle)
        self.chk_backup_enable.grid(row=3, column=0, columnspan=3, sticky="w", pady=2)
        self.lbl_backup_name = ttk.Label(self.frame_opts, text=self._t("lbl_backup_name"))
        self.lbl_backup_name.grid(row=4, column=0, sticky="w", pady=2)
        ttk.Entry(self.frame_opts, textvariable=self.backup_name_var).grid(row=4, column=1, sticky="ew", padx=6, pady=2)
        self.btn_browse_backup = ttk.Button(self.frame_opts, text=self._t("btn_folder"), command=self._browse_backup_dir)
        self.btn_browse_backup.grid(row=4, column=2, sticky="w", pady=2)
        ttk.Label(self.frame_opts, textvariable=self.backup_path_preview_var, foreground="#4a5568").grid(row=5, column=0, columnspan=3, sticky="w", pady=(0, 4))
        self.chk_skip_flash = ttk.Checkbutton(self.frame_opts, text=self._t("chk_skip_flash"), variable=self.skip_flash_var, command=self._on_skip_flash_toggle)
        self.chk_skip_flash.grid(row=6, column=0, columnspan=3, sticky="w", pady=2)
        self.chk_skip_params = ttk.Checkbutton(self.frame_opts, text=self._t("chk_skip_params"), variable=self.skip_params_var, command=self._on_skip_params_toggle)
        self.chk_skip_params.grid(row=7, column=0, columnspan=3, sticky="w", pady=2)

        frame_btns = ttk.Frame(self.root)
        frame_btns.pack(fill="x", padx=10, pady=6)
        self.btn_prep = ttk.Button(frame_btns, text=self._t("btn_prep"), command=self.start_prepare_only)
        self.btn_prep.pack(side="left", padx=(0, 8))
        self.btn_run = ttk.Button(frame_btns, text=self._t("btn_run"), command=self.start_full_process)
        self.btn_run.pack(side="left", padx=(0, 8))
        self.btn_stop = ttk.Button(frame_btns, text=self._t("btn_stop"), command=self.stop_process, state="disabled")
        self.btn_stop.pack(side="left")
        self.btn_about = ttk.Button(frame_btns, text=self._t("btn_about"), command=self._show_about)
        self.btn_about.pack(side="right", padx=(6, 0))
        self.btn_clear_log = ttk.Button(frame_btns, text=self._t("btn_clear"), command=self._clear_log)
        self.btn_clear_log.pack(side="right")

        frame_prog = ttk.Frame(self.root)
        frame_prog.pack(fill="x", padx=10, pady=(2, 6))
        self.progress = ttk.Progressbar(frame_prog, mode="determinate", maximum=100)
        self.progress.pack(fill="x")
        ttk.Label(frame_prog, textvariable=self.status_var).pack(anchor="w", pady=(3, 0))

        self.frame_log = ttk.LabelFrame(self.root, text=self._t("frame_log"), padding=6)
        self.frame_log.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.log_box = scrolledtext.ScrolledText(self.frame_log, state="disabled", font=("Consolas", 9), wrap="word")
        self.log_box.pack(fill="both", expand=True)

    def _update_fw_mode_label(self):
        fw_str = self.fw_var.get().strip()
        if not fw_str:
            self.fw_mode_var.set(self._t("fw_empty"))
            return
        ext = Path(fw_str).suffix.lower()
        if ext == ".apj":
            self.fw_mode_var.set(self._t("fw_apj"))
        elif ext in (".hex", ".bin"):
            self.fw_mode_var.set(self._t("fw_dfu"))
        else:
            self.fw_mode_var.set(self._t("fw_bad", ext=ext))

    def _show_about(self):
        dfu_path = find_dfu_util()
        dfu_state = self._t("dfu_embedded") if dfu_path else self._t("dfu_missing")
        info = self._t("about_body", app=APP_NAME, ver=APP_VERSION, copy=APP_COPYRIGHT, repo=APP_REPO, dfu=dfu_state)
        messagebox.showinfo(self._t("about_title", app=APP_NAME), info)

    def _browse_fw(self):
        p = filedialog.askopenfilename(
            title="Оберіть файл прошивки (.apj або *_with_bl.hex)",
            initialdir=str(BASE_DIR),
            filetypes=[
                ("Прошивки ArduPilot / DFU", "*.apj *.hex *.bin"),
                ("ArduPilot Bootloader (.apj)", "*.apj"),
                ("STM32 DFU (.hex / .bin)", "*.hex *.bin"),
                ("Усі файли", "*.*"),
            ],
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
        items = ["Авто"] + [format_port_label(p) for p in ports]
        self.port_combo["values"] = items
        if self.port_var.get() not in items:
            self.port_var.set("Авто")

    def check_dfu_clicked(self):
        dfu_exe = find_dfu_util()
        if not dfu_exe:
            messagebox.showerror("DFU недоступний", "Не знайдено dfu-util.exe поруч із програмою або всередині збірки.")
            return
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        try:
            proc = subprocess.run(
                [str(dfu_exe), "-l"],
                capture_output=True,
                text=True,
                timeout=8,
                creationflags=creationflags,
            )
            out = (proc.stdout or "") + "\n" + (proc.stderr or "")
        except Exception as e:
            messagebox.showerror("Помилка DFU", f"Не вдалося запустити dfu-util: {e}")
            return

        self.log("=== Перевірка STM32 DFU ===")
        for line in out.splitlines():
            if line.strip():
                self.log(f"  {line.strip()}")

        low = out.lower()
        if "0483:df11" in low and ("@internal flash" in low or "alt=0" in low):
            messagebox.showinfo(
                "DFU готовий",
                "Знайдено контролер STM32 у режимі DFU (0483:df11) з доступом до Internal Flash.\n\n"
                "Можна запускати прошивку файлу *_with_bl.hex.",
            )
        elif "cannot open dfu device" in low or "libusb_error_not_supported" in low or "libusb_error_access" in low:
            messagebox.showwarning(
                "Потрібен драйвер WinUSB",
                "Контролер у режимі DFU (0483:df11) знайдено, але Windows використовує несумісний драйвер (STTub30).\n\n"
                "Встановіть драйвер WinUSB через ImpulseRC Driver Fixer або Zadig (оберіть 'STM32 BOOTLOADER' → WinUSB) "
                "і повторіть перевірку.",
            )
        else:
            messagebox.showinfo(
                "DFU не знайдено",
                "Пристрій STM32 DFU (0483:df11) не виявлено.\n\n"
                "Щоб перевести Matek H743 у режим DFU:\n"
                "1. Від'єднайте USB та акумулятор.\n"
                "2. Затисніть кнопку BOOT на платі.\n"
                "3. Утримуючи BOOT, підключіть USB до ПК і відпустіть кнопку.",
            )

    def unprotect_dfu_clicked(self):
        dfu_exe = find_dfu_util()
        if not dfu_exe:
            messagebox.showerror("DFU недоступний", "Не знайдено dfu-util.exe поруч із програмою або всередині збірки.")
            return
        ok = messagebox.askyesno(
            "Підтвердження зняття захисту",
            "Зняття захисту (unprotect) повністю зітре внутрішню пам'ять контролера STM32 та перезавантажить його.\n\n"
            "Переконайтеся, що плата підключена в режимі BOOT (DFU) без пропелерів і силового акумулятора.\n\n"
            "Продовжити?",
        )
        if not ok:
            return
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        with tempfile.TemporaryDirectory(prefix="bittec_unprot_") as tmpdir:
            dummy_bin = Path(tmpdir) / "dummy.bin"
            dummy_bin.write_bytes(b"\x00\x00\x00\x00")
            try:
                proc = subprocess.run(
                    [str(dfu_exe), "-d", "0483:df11", "-a", "0", "-s", "0x08000000:unprotect:force", "-D", str(dummy_bin)],
                    capture_output=True,
                    text=True,
                    timeout=20,
                    creationflags=creationflags,
                )
                out = (proc.stdout or "") + "\n" + (proc.stderr or "")
            except Exception as e:
                messagebox.showerror("Помилка unprotect", f"Не вдалося виконати зняття захисту: {e}")
                return

        self.log("=== Зняття захисту STM32 DFU (unprotect) ===")
        for line in out.splitlines():
            if line.strip():
                self.log(f"  {line.strip()}")

        low = out.lower()
        if "erases flash" in low or "disconnects" in low or proc.returncode == 0:
            messagebox.showinfo(
                "Захист знято",
                "Команду зняття захисту виконано — контролер очищено та перезавантажено.\n\n"
                "Щоб прошити контролер, від'єднайте USB, затисніть BOOT, підключіть USB і натисніть «Перевірити DFU».",
            )
        else:
            messagebox.showwarning(
                "Перевірте стан DFU",
                "dfu-util повернув повідомлення про помилку. Перевірте журнал виконання та перепідключіть плату у режим BOOT.",
            )

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
        try:
            with (BASE_DIR / "bittec_flasher.log").open("a", encoding="utf-8") as lf:
                lf.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}\n")
        except Exception:
            pass

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
                elif kind == "prompt_unprotect":
                    ev, res_box = payload
                    res_box["unprotect"] = messagebox.askyesno(
                        "Пам'ять контролера заблоковано",
                        "Під час DFU-прошивки виявлено блокування пам'яті або стирання секторів (RDP / WRP).\n\n"
                        "Зняти захист (unprotect) зараз? УВАГА: це повністю зітре пам'ять контролера та перезавантажить його.\n"
                        "Перед продовженням зніміть пропелери та від'єднайте силовий акумулятор.",
                    )
                    ev.set()
                elif kind == "prompt_reconnect":
                    ev, res_box = payload
                    res_box["retry"] = messagebox.askretrycancel(
                        "Перепідключіть USB після DFU",
                        "DFU-прошивку завершено, але Windows не визначила COM-порт контролера автоматично.\n\n"
                        "1. Від'єднайте USB-кабель від ПК на 3–5 секунд.\n"
                        "2. Підключіть USB знову (БЕЗ натискання кнопки BOOT).\n"
                        "3. Дочекайтеся звуку підключення Windows і натисніть «Повторити».",
                    )
                    ev.set()
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

    def _validate_inputs(self, need_fw: bool, need_param: bool) -> tuple[Path | None, Path | None] | None:
        param_str = self.param_var.get().strip()
        param_path: Path | None = None
        if param_str:
            if not Path(param_str).is_file():
                messagebox.showerror("Помилка", "Вказаний файл параметрів не існує.")
                return None
            param_path = Path(param_str)
        elif need_param:
            messagebox.showerror("Помилка", "Оберіть існуючий файл параметрів (.param / .parm).")
            return None

        fw_path: Path | None = None
        if need_fw:
            fw_str = self.fw_var.get().strip()
            if not fw_str or not Path(fw_str).is_file():
                messagebox.showerror("Помилка", "Оберіть існуючий файл прошивки (.apj, .hex або .bin).")
                return None
            fw_path = Path(fw_str)
            ext = fw_path.suffix.lower()
            if ext == ".apj":
                if not (RESOURCE_DIR / "uploader.py").is_file():
                    messagebox.showerror("Помилка", f"Не знайдено uploader.py у {RESOURCE_DIR}.")
                    return None
            elif ext in (".hex", ".bin"):
                if not find_dfu_util():
                    messagebox.showerror("Помилка", "Не знайдено dfu-util.exe для прошивки через STM32 DFU.")
                    return None
            else:
                messagebox.showerror("Помилка", f"Непідтримуваний формат прошивки: {ext}. Оберіть .apj, .hex або .bin.")
                return None

        return fw_path, param_path

    def start_prepare_only(self):
        res = self._validate_inputs(need_fw=False, need_param=True)
        if not res or res[1] is None:
            return
        _, param_path = res
        self.stop_event.clear()
        self._set_running(True)
        self.worker = threading.Thread(
            target=self._run_prepare_only,
            args=(param_path, self.strip_calib_var.get(), self.strip_servo_var.get()),
            daemon=True,
        )
        self.worker.start()

    def _on_skip_flash_toggle(self):
        if self.skip_flash_var.get():
            self.skip_params_var.set(False)

    def _on_skip_params_toggle(self):
        if self.skip_params_var.get():
            self.skip_flash_var.set(False)

    @staticmethod
    def _default_backup_filename() -> str:
        return time.strftime("backup_%Y%m%d_%H%M%S.param")

    def _on_backup_toggle(self):
        if self.backup_enable_var.get():
            cur = self.backup_name_var.get().strip()
            if not cur or re.fullmatch(r"backup_\d{8}_\d{6}\.param", cur, re.IGNORECASE):
                self.backup_name_var.set(self._default_backup_filename())
        self._update_backup_preview()

    def _sync_backup_dir_from_param(self):
        if self._backup_dir_custom:
            return
        raw = self.param_var.get().strip()
        if raw:
            p = Path(raw)
            parent = p.parent if p.suffix else p
            if str(parent) and str(parent) != ".":
                self.backup_dir_var.set(str(parent))

    def _browse_backup_dir(self):
        initial = self.backup_dir_var.get().strip() or str(BASE_DIR)
        chosen = filedialog.askdirectory(title="Оберіть базову папку для резервних копій", initialdir=initial)
        if chosen:
            self._backup_dir_custom = True
            self.backup_dir_var.set(chosen)

    def _update_backup_preview(self):
        base = self.backup_dir_var.get().strip() or str(BASE_DIR)
        dated_dir = Path(base) / f"backup_{time.strftime('%Y-%m-%d')}"
        self.backup_path_preview_var.set(self._t("backup_preview", path=dated_dir))

    def _save_backup_params(
        self,
        params: dict[str, tuple[float, int]],
        base_dir_str: str,
        raw_filename: str,
        param_path: Path | None,
    ) -> Path:
        if base_dir_str.strip():
            base_dir = Path(base_dir_str.strip())
        elif param_path is not None:
            base_dir = param_path.parent
        else:
            base_dir = BASE_DIR
        dated_dir = base_dir / f"backup_{time.strftime('%Y-%m-%d')}"
        dated_dir.mkdir(parents=True, exist_ok=True)
        clean_name = re.sub(r'[<>:"/\\|?*]', "_", raw_filename.strip())
        if not clean_name:
            clean_name = self._default_backup_filename()
        if not clean_name.lower().endswith((".param", ".parm")):
            clean_name += ".param"
        out_path = dated_dir / clean_name
        lines = [
            f"# Backup created by Bittec Flasher on {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"# Parameters count: {len(params)}",
        ]
        for name in sorted(params.keys()):
            val, ptype = params[name]
            if math.isfinite(val) and float(val).is_integer() and abs(val) < 1e12 and ptype in (1, 2, 3, 4, 5, 6):
                val_str = str(int(round(val)))
            else:
                val_str = f"{val:.6f}".rstrip("0").rstrip(".") or "0"
                if val_str == "-0":
                    val_str = "0"
            lines.append(f"{name},{val_str}")
        out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return out_path

    def _ask_unprotect_on_lock(self) -> bool:
        ev = threading.Event()
        res_box = {"unprotect": False}
        self.log_queue.put(("prompt_unprotect", (ev, res_box)))
        while not ev.is_set():
            if self.stop_event.is_set():
                return False
            ev.wait(0.2)
        return bool(res_box["unprotect"])

    def _run_unprotect_cmd(self, dfu_exe: Path) -> tuple[bool, str]:
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        with tempfile.TemporaryDirectory(prefix="bittec_unprot_") as tmpdir:
            dummy_bin = Path(tmpdir) / "dummy.bin"
            dummy_bin.write_bytes(b"\x00\x00\x00\x00")
            proc = subprocess.run(
                [str(dfu_exe), "-d", "0483:df11", "-a", "0", "-s", "0x08000000:unprotect:force", "-D", str(dummy_bin)],
                capture_output=True,
                text=True,
                timeout=20,
                creationflags=creationflags,
            )
            out = (proc.stdout or "") + "\n" + (proc.stderr or "")
        self.log("=== Зняття захисту STM32 DFU (unprotect) ===")
        for line in out.splitlines():
            if line.strip():
                self.log(f"  {line.strip()}")
        low = out.lower()
        ok = ("erases flash" in low) or ("disconnects" in low) or (proc.returncode == 0)
        return ok, out

    def _ask_manual_usb_reconnect(self) -> bool:
        ev = threading.Event()
        res_box = {"retry": False}
        self.log_queue.put(("prompt_reconnect", (ev, res_box)))
        while not ev.is_set():
            if self.stop_event.is_set():
                return False
            ev.wait(0.2)
        return bool(res_box["retry"])

    def start_full_process(self):
        skip_flash = self.skip_flash_var.get()
        skip_params = self.skip_params_var.get()
        if skip_flash and skip_params:
            messagebox.showerror("Помилка", "Не можна одночасно пропустити і прошивку, і параметри.")
            return
        res = self._validate_inputs(need_fw=not skip_flash, need_param=skip_flash)
        if not res:
            return
        fw_path, param_path = res
        if skip_params:
            param_path = None
        self.stop_event.clear()
        self._set_running(True)
        self.worker = threading.Thread(
            target=self._run_full_pipeline,
            args=(
                fw_path,
                param_path,
                self._get_selected_port(),
                self.strip_calib_var.get(),
                skip_flash,
                self.strip_servo_var.get(),
                self.backup_enable_var.get(),
                self.backup_dir_var.get(),
                self.backup_name_var.get(),
            ),
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

    def _run_prepare_only(self, param_path: Path, strip_calib: bool, strip_servo: bool = False):
        try:
            self.set_progress(25, "Очищення файлу параметрів...")
            removed_servo: list[str] = []
            _, target_params, stats = prepare_param_file(param_path, strip_calib, strip_servo, removed_servo)
            self._log_prep_stats(target_params, stats)
            if strip_servo:
                self.log(f"Виключено налаштувань сервоприводів (SERVO*): {len(removed_servo)}")
            self.set_progress(100, "Підготовка параметрів завершена")
            self.log_queue.put(("done", (True, f"Файл підготовлено:\n{stats['clean_path']}")))
        except Exception as e:
            self.log(f"[ПОМИЛКА] {e}")
            self.set_progress(0, "Помилка підготовки")
            self.log_queue.put(("done", (False, f"Помилка: {e}")))

    def _connect_mavlink(self, explicit_port: str | None, timeout_total: int = 75):
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
        raise RuntimeError(
            "Не вдалося підключитись до контролера по MAVLink. Закрийте Mission Planner / QGC і перевірте USB."
        )

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

    def _reboot_fc(self, conn, wait_sec: int = 9):
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

    def _stream_process(self, cmd: list[str], cwd: Path) -> tuple[int, list[str]]:
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.active_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            cwd=str(cwd),
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
            creationflags=creationflags,
        )

        lines: list[str] = []
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
                    lines.append(line)
                    self.log(f"  {line}")
                buf = ""
            elif c == "\r":
                line = buf.strip()
                now = time.time()
                is_prog = bool(re.match(r"^(Erase|Download|Upload)\s+\[[^\]]*\]\s+\d+%\s+\d+\s+bytes$", line))
                if line and ((not is_prog) or (now - last_cr_log > 0.6) or ("100%" in line)):
                    lines.append(line)
                    self.log(f"  {line}")
                    if is_prog:
                        last_cr_log = now
                buf = ""
            else:
                buf += c

        if buf.strip():
            lines.append(buf.strip())
            self.log(f"  {buf.strip()}")

        rc = self.active_proc.wait()
        self.active_proc = None
        return rc, lines

    @staticmethod
    def _is_bootloader_port(port_info) -> bool:
        desc = f"{port_info.description or ''} {port_info.hwid or ''}".lower()
        return bool(re.search(r"\bbl\b|bootloader", desc))

    def _try_reboot_to_bootloader(self, explicit_port: str | None) -> str | None:
        ports = sorted(serial.tools.list_ports.comports(), key=rank_port)
        if explicit_port:
            candidates = [explicit_port]
        else:
            candidates = [p.device for p in ports if rank_port(p) <= 1]
        if not candidates:
            return None

        old_devices = {p.device for p in ports}
        for dev in candidates:
            if self.stop_event.is_set():
                raise InterruptedError("Прошивку зупинено користувачем")
            conn = None
            try:
                conn = mavutil.mavlink_connection(dev, baud=115200)
                hb = conn.wait_heartbeat(timeout=2.5)
                if not hb:
                    continue
                self.log(f"[FLASH .apj] Знайдено MAVLink на {dev} — переведення в завантажувач (param1=3)...")
                conn.mav.command_long_send(
                    conn.target_system,
                    conn.target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                    0,
                    3, 0, 0, 0, 0, 0, 0,
                )
                time.sleep(0.3)
                try:
                    conn.close()
                except Exception:
                    pass
                conn = None

                deadline = time.time() + 7.0
                saw_disconnect = False
                while time.time() < deadline:
                    if self.stop_event.is_set():
                        raise InterruptedError("Прошивку зупинено користувачем")
                    cur_ports = sorted(serial.tools.list_ports.comports(), key=rank_port)
                    cur_usb = [p for p in cur_ports if rank_port(p) <= 1]
                    cur_devs = {p.device for p in cur_usb}
                    if dev not in cur_devs:
                        saw_disconnect = True
                    bl_ports = [p for p in cur_usb if self._is_bootloader_port(p)]
                    if bl_ports:
                        time.sleep(0.25)
                        return bl_ports[0].device
                    new_ports = [p for p in cur_usb if p.device not in old_devices]
                    if new_ports:
                        time.sleep(0.25)
                        return new_ports[0].device
                    if saw_disconnect and cur_usb:
                        time.sleep(0.25)
                        return cur_usb[0].device
                    time.sleep(0.15)
                return None
            except InterruptedError:
                raise
            except Exception:
                pass
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass
        return None

    def _wait_for_flash_port(self, explicit_port: str | None, timeout_sec: float = 60.0) -> str | None:
        ports = sorted(serial.tools.list_ports.comports(), key=rank_port)
        usb_ports = [p for p in ports if rank_port(p) <= 1 or (explicit_port and p.device == explicit_port)]
        for p in usb_ports:
            if self._is_bootloader_port(p):
                return p.device

        bl_port = self._try_reboot_to_bootloader(explicit_port)
        if bl_port:
            return bl_port

        baseline = {p.device for p in serial.tools.list_ports.comports() if rank_port(p) <= 1}
        self.log("[FLASH .apj] Підключіть або перепідключіть кабель USB контролера...")
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.stop_event.is_set():
                raise InterruptedError("Прошивку зупинено користувачем")
            cur_ports = sorted(serial.tools.list_ports.comports(), key=rank_port)
            cur_usb = [p for p in cur_ports if rank_port(p) <= 1 or (explicit_port and p.device == explicit_port)]
            cur_devs = {p.device for p in cur_usb}
            baseline = baseline & cur_devs

            bl_ports = [p for p in cur_usb if self._is_bootloader_port(p)]
            if bl_ports:
                time.sleep(0.2)
                return bl_ports[0].device

            arrived = [p for p in cur_usb if p.device not in baseline]
            if arrived:
                time.sleep(0.25)
                cur_ports2 = sorted(serial.tools.list_ports.comports(), key=rank_port)
                cur_usb2 = [p for p in cur_ports2 if rank_port(p) <= 1 or (explicit_port and p.device == explicit_port)]
                arrived2 = [p for p in cur_usb2 if p.device not in baseline]
                if arrived2:
                    return arrived2[0].device
                return arrived[0].device
            time.sleep(0.15)
        return explicit_port

    def _run_uploader(self, fw_path: Path, explicit_port: str | None):
        self.log(f"[FLASH .apj] Пошук порту завантажувача для {fw_path.name}...")
        target_port = self._wait_for_flash_port(explicit_port)
        if target_port:
            self.log(f"[FLASH .apj] Вибрано порт завантажувача: {target_port}")

        if getattr(sys, "frozen", False):
            cmd = [sys.executable, "--run-uploader"]
        else:
            cmd = [sys.executable, "-u", str(Path(__file__).resolve()), "--run-uploader"]
        if target_port:
            cmd += ["--port", target_port]
        cmd.append(str(fw_path))

        rc, _ = self._stream_process(cmd, BASE_DIR)
        if self.stop_event.is_set():
            raise InterruptedError("Прошивку зупинено користувачем")
        if rc != 0:
            raise RuntimeError(f"Завантажувач завершився з кодом {rc}")

    def _run_dfu_flash(self, fw_path: Path):
        dfu_exe = find_dfu_util()
        if not dfu_exe:
            raise RuntimeError("Не знайдено dfu-util.exe для прошивки STM32 DFU.")

        self.log(f"[DFU] Підготовка прошивки {fw_path.name} для STM32 DFU (0483:df11)...")
        with tempfile.TemporaryDirectory(prefix="bittec_dfu_") as tmpdir:
            if fw_path.suffix.lower() == ".hex":
                bin_path = Path(tmpdir) / f"{fw_path.stem}.bin"
                base_addr, span = convert_intel_hex_to_bin(fw_path, bin_path)
                self.log(f"[DFU] Конвертовано HEX → BIN: старт 0x{base_addr:08X}, розмір {span / 1024:.1f} КБ.")
            else:
                bin_path = fw_path
                base_addr = 0x08000000
                self.log(f"[DFU] Використано BIN: старт 0x{base_addr:08X}, розмір {fw_path.stat().st_size / 1024:.1f} КБ.")

            if base_addr != 0x08000000:
                self.log(f"[DFU] Увага: базова адреса образу 0x{base_addr:08X} (нестандартна для повного *_with_bl.hex).")

            cmd = [
                str(dfu_exe),
                "-d", "0483:df11",
                "-a", "0",
                "-s", f"0x{base_addr:08x}:leave",
                "-D", str(bin_path),
            ]
            self.log("[DFU] Переконайтеся, що Matek H743 підключено в режимі BOOT (DFU). Запуск запису...")
            rc, lines = self._stream_process(cmd, dfu_exe.parent)

        if self.stop_event.is_set():
            raise InterruptedError("DFU-прошивку зупинено користувачем")

        joined = "\n".join(lines).lower()
        downloaded_ok = ("file downloaded successfully" in joined) or ("transitioning to dfumanifest" in joined) or (rc == 74 and bool(re.search(r"download\s*\[[^\]]*\]\s*100%", joined))) or (rc == 74 and bool(re.search(r"download\s*\[[^\]]*\]\s*100%", joined)))
        if rc != 0 and downloaded_ok:
            self.log("[DFU] Прошивку записано успішно (контролер вийшов із режиму DFU та перезавантажується).")
            return
        if rc != 0:
            if "cannot open dfu device" in joined or "libusb_error_not_supported" in joined or "libusb_error_access" in joined:
                raise RuntimeError(
                    "Немає доступу до STM32 DFU (помилка драйвера Windows). Встановіть драйвер WinUSB через "
                    "ImpulseRC Driver Fixer або Zadig (STM32 BOOTLOADER → WinUSB) і повторіть спробу."
                )
            if "no dfu capable usb device" in joined:
                raise RuntimeError(
                    "Контролер у режимі DFU не знайдено. Від'єднайте USB, затисніть кнопку BOOT на Matek H743, "
                    "підключіть USB і натисніть «Перевірити DFU»."
                )
        is_locked = (
            ("erase_page not correctly executed" in joined)
            or ("set_address not correctly executed" in joined)
            or ("mass_erase not correctly executed" in joined)
            or ("read protection" in joined)
            or ("write protect" in joined)
        )
        if is_locked:
            self.log("[DFU] Виявлено блокування пам'яті контролера (RDP/WRP).")
            if self._ask_unprotect_on_lock():
                ok_unprot, _ = self._run_unprotect_cmd(dfu_exe)
                if ok_unprot:
                    raise RuntimeError(
                        "Захист пам'яті знято, контролер очищено та перезавантажено. "
                        "Від'єднайте USB, затисніть BOOT, підключіть USB і запустіть прошивку повторно."
                    )
                raise RuntimeError("Не вдалося виконати зняття захисту. Перевірте журнал виконання та режим BOOT.")
            raise RuntimeError("Прошивку скасовано: пам'ять контролера заблоковано, зняття захисту відхилено.")
        raise RuntimeError(f"dfu-util завершився з кодом {rc}. Перевірте журнал виконання.")

    def _run_full_pipeline(
        self,
        fw_path: Path | None,
        param_path: Path | None,
        explicit_port: str | None,
        strip_calib: bool,
        skip_flash: bool,
        strip_servo: bool = False,
        backup_enable: bool = False,
        backup_dir: str = "",
        backup_name: str = "",
    ):
        try:
            target_params: dict[str, float] | None = None
            if param_path is not None:
                self.set_progress(10, "Етап 1/5: Підготовка параметрів...")
                removed_servo: list[str] = []
                _, target_params, stats = prepare_param_file(param_path, strip_calib, strip_servo, removed_servo)
                self._log_prep_stats(target_params, stats)
                if strip_servo:
                    self.log(f"Виключено налаштувань сервоприводів (SERVO*): {len(removed_servo)}")
            else:
                self.log("[PARAMS] Файл параметрів не вказано — буде виконано лише прошивку.")

            preloaded_conn = None
            preloaded_params = None
            if backup_enable and target_params is not None:
                self.set_progress(18, "Створення резервної копії параметрів...")
                if skip_flash:
                    preloaded_conn = self._connect_mavlink(explicit_port)
                    preloaded_params = self._download_params(preloaded_conn)
                    saved_bkp = self._save_backup_params(preloaded_params, backup_dir, backup_name, param_path)
                    self.log(f"[BACKUP] Резервну копію ({len(preloaded_params)} пар.) збережено: {saved_bkp}")
                else:
                    bkp_conn = None
                    try:
                        bkp_conn = self._connect_mavlink(explicit_port, timeout_total=10)
                        bkp_params = self._download_params(bkp_conn)
                        saved_bkp = self._save_backup_params(bkp_params, backup_dir, backup_name, param_path)
                        self.log(f"[BACKUP] Резервну копію перед прошивкою ({len(bkp_params)} пар.) збережено: {saved_bkp}")
                    except RuntimeError:
                        self.log("[BACKUP] Контролер не відповідає по MAVLink перед прошивкою (можливо, вже в режимі DFU/BOOT) — бекап пропущено.")
                    finally:
                        if bkp_conn is not None:
                            try:
                                bkp_conn.close()
                            except Exception:
                                pass

            if not skip_flash and fw_path is not None:
                is_dfu = fw_path.suffix.lower() in (".hex", ".bin")
                if is_dfu:
                    self.set_progress(25, "Етап 2/5: Первинна прошивка STM32 DFU...")
                    self._run_dfu_flash(fw_path)
                    wait_boot = 12
                    self.log(f"[DFU] Прошивка завершена. Очікування першого запуску ArduPilot ({wait_boot} сек)...")
                else:
                    self.set_progress(25, "Етап 2/5: Прошивка .apj (підключіть USB)...")
                    self._run_uploader(fw_path, explicit_port)
                    wait_boot = 9
                    self.log(f"[FLASH] Прошивка завершена. Очікування запуску контролера ({wait_boot} сек)...")

                if target_params is not None:
                    for _ in range(wait_boot):
                        if self.stop_event.is_set():
                            raise InterruptedError("Зупинено користувачем")
                        time.sleep(1)
            else:
                self.log("[FLASH] Прошивку пропущено за налаштуванням.")

            if target_params is None or param_path is None:
                self.set_progress(100, "Готово! Прошивку завершено.")
                self.log_queue.put(("done", (True, "Прошивку контролера успішно завершено!")))
                return

            self.set_progress(50, "Етап 3/5: Заливка параметрів (Прохід 1)...")
            used_dfu = (not skip_flash) and (fw_path is not None) and (fw_path.suffix.lower() in (".hex", ".bin"))
            if used_dfu:
                conn = None
                try:
                    conn = self._connect_mavlink(explicit_port, timeout_total=18)
                except RuntimeError:
                    self.log("[MAVLink] Автопідключення після DFU не вдалося — запит ручного перепідключення USB.")
                while conn is None:
                    self.set_progress(48, "Очікування перепідключення USB (без кнопки BOOT)...")
                    if not self._ask_manual_usb_reconnect():
                        raise InterruptedError("Зупинено користувачем під час очікування перепідключення USB")
                    self.log("[MAVLink] Повторний пошук контролера після ручного перепідключення USB...")
                    try:
                        conn = self._connect_mavlink(explicit_port, timeout_total=45)
                    except RuntimeError:
                        self.log("[MAVLink] Порт досі не знайдено. Перевірте кабель і спробуйте ще раз.")
            elif preloaded_conn is not None and preloaded_params is not None:
                conn = preloaded_conn
                cur1 = preloaded_params
            else:
                conn = self._connect_mavlink(explicit_port)
                cur1 = self._download_params(conn)
            w1, s1, m1, f1 = self._write_pass(conn, target_params, cur1)
            self.log(f"[Прохід 1] Записано: {w1}, вже збігались: {s1}, ще відсутні: {len(m1)}, помилок: {len(f1)}")
            self._reboot_fc(conn, wait_sec=9)

            self.set_progress(70, "Етап 4/5: Дозаписування залежних параметрів (Прохід 2)...")
            conn = self._connect_mavlink(explicit_port)
            cur2 = self._download_params(conn)
            w2, s2, m2, f2 = self._write_pass(conn, target_params, cur2)
            self.log(f"[Прохід 2] Записано: {w2}, вже збігались: {s2}, відсутні: {len(m2)}, помилок: {len(f2)}")
            self._reboot_fc(conn, wait_sec=9)

            self.set_progress(82, "Етап 4б/5: Повторна фіксація параметрів (Прохід 3)...")
            conn = self._connect_mavlink(explicit_port)
            cur3 = self._download_params(conn)
            w3, s3, m3, f3 = self._write_pass(conn, target_params, cur3)
            self.log(f"[Прохід 3] Записано: {w3}, вже збігались: {s3}, відсутні: {len(m3)}, помилок: {len(f3)}")
            if w3 > 0:
                self._reboot_fc(conn, wait_sec=9)
            else:
                try:
                    conn.close()
                except Exception:
                    pass

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
