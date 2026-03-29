#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from collections import Counter, deque
from datetime import datetime
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple
from xml.etree import ElementTree as ET


DEFAULT_PACKAGE = "com.openpositioning.PositionMe"
DEFAULT_MAIN_ACTIVITY = (
    "com.openpositioning.PositionMe.presentation.activity.MainActivity"
)
DEFAULT_DURATION_S = 90
DEFAULT_INTERVAL_S = 3
COMMON_ADB_PATHS = [
    "~/Library/Android/sdk/platform-tools/adb",
    "$ANDROID_SDK_ROOT/platform-tools/adb",
    "$ANDROID_HOME/platform-tools/adb",
]
TOP_CARD_CROP_HEIGHT_HINT = 500
GROUND_FLOOR_TOKENS = {"GF", "G", "GROUND", "GROUND FLOOR", "F0", "0F", "0"}
FIRST_FLOOR_TOKENS = {"1", "1F", "F1", "FIRST", "FIRST FLOOR"}


class MonitorError(RuntimeError):
    pass


class SessionLogger:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.Lock()
        self._handle = path.open("w", encoding="utf-8")

    def log(self, message: str) -> None:
        with self._lock:
            self._handle.write(message.rstrip("\n") + "\n")
            self._handle.flush()

    def close(self) -> None:
        with self._lock:
            self._handle.close()


class LogcatCollector:
    def __init__(
        self,
        adb_path: Path,
        serial: str,
        package_name: str,
        logger: SessionLogger,
    ) -> None:
        self.adb_path = adb_path
        self.serial = serial
        self.package_name = package_name
        self.logger = logger
        self.process: Optional[subprocess.Popen[str]] = None
        self.thread: Optional[threading.Thread] = None
        self.recent_lines: Deque[str] = deque(maxlen=500)

    def start(self) -> None:
        pid = try_get_pid(self.adb_path, self.serial, self.package_name)
        cmd = [str(self.adb_path), "-s", self.serial, "logcat", "-v", "threadtime"]
        if pid:
            cmd.extend(["--pid", pid])
        cmd.extend(
            [
                "ARROW_DBG:D",
                "POSE_DIAG:D",
                "DISPLAY_DIAG:D",
                "TrajectoryMapFragment:D",
                "SensorFusion:D",
                "IndoorMapManager:D",
                "*:S",
            ]
        )
        self.logger.log(
            f"logcat_start pid={pid or 'n/a'} cmd={' '.join(cmd)}"
        )
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
        self.thread = threading.Thread(target=self._pump, daemon=True)
        self.thread.start()

    def _pump(self) -> None:
        assert self.process is not None
        assert self.process.stdout is not None
        for line in self.process.stdout:
            clean = line.rstrip("\n")
            self.recent_lines.append(clean)
            self.logger.log(f"[LOGCAT] {clean}")

    def snapshot_recent_lines(self) -> List[str]:
        return list(self.recent_lines)

    def stop(self) -> None:
        if self.process is None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        if self.thread is not None:
            self.thread.join(timeout=2)


def discover_adb(explicit: Optional[str]) -> Tuple[Path, str]:
    if explicit:
        path = Path(explicit).expanduser().resolve()
        if path.is_file() and os.access(path, os.X_OK):
            return path, "explicit"
        raise MonitorError(f"adb not executable: {path}")

    adb_from_path = shutil.which("adb")
    if adb_from_path:
        return Path(adb_from_path).resolve(), "path"

    for candidate in COMMON_ADB_PATHS:
        expanded = Path(os.path.expandvars(candidate)).expanduser()
        if expanded.is_file() and os.access(expanded, os.X_OK):
            return expanded.resolve(), "sdk_fallback"

    raise MonitorError(
        "adb not found in PATH or common Android SDK locations. "
        "Install platform-tools or pass --adb /path/to/adb."
    )


def run(
    cmd: Sequence[str],
    *,
    timeout: int = 30,
    check: bool = True,
    binary: bool = False,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        list(cmd),
        timeout=timeout,
        check=check,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=not binary,
        encoding=None if binary else "utf-8",
        errors=None if binary else "replace",
    )


def adb_cmd(adb_path: Path, serial: Optional[str], *args: str) -> List[str]:
    cmd = [str(adb_path)]
    if serial:
        cmd.extend(["-s", serial])
    cmd.extend(args)
    return cmd


def check_adb(adb_path: Path) -> str:
    proc = run([str(adb_path), "version"], timeout=20)
    return proc.stdout.strip()


def list_online_devices(adb_path: Path) -> List[Dict[str, str]]:
    proc = run([str(adb_path), "devices", "-l"], timeout=20)
    devices: List[Dict[str, str]] = []
    for raw_line in proc.stdout.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("List of devices attached"):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        serial = parts[0]
        state = parts[1]
        extras = " ".join(parts[2:])
        devices.append({"serial": serial, "state": state, "extras": extras})
    return devices


def choose_device(
    devices: List[Dict[str, str]], requested_serial: Optional[str]
) -> Dict[str, str]:
    online = [d for d in devices if d["state"] == "device"]
    if requested_serial:
        for device in online:
            if device["serial"] == requested_serial:
                return device
        raise MonitorError(
            f"Requested serial {requested_serial} not found in online devices."
        )
    if not online:
        raise MonitorError("No online Android devices found in `adb devices -l`.")
    physical = [d for d in online if not d["serial"].startswith("emulator-")]
    if len(physical) == 1:
        return physical[0]
    if len(physical) > 1:
        raise MonitorError(
            "Multiple physical devices are online. Pass --serial to choose one."
        )
    if len(online) == 1:
        return online[0]
    raise MonitorError(
        "Multiple online devices found and no single physical device to prefer. "
        "Pass --serial."
    )


def adb_shell_text(adb_path: Path, serial: str, *args: str, timeout: int = 30) -> str:
    proc = run(adb_cmd(adb_path, serial, "shell", *args), timeout=timeout)
    return proc.stdout.strip()


def try_get_pid(adb_path: Path, serial: str, package_name: str) -> Optional[str]:
    try:
        pid = adb_shell_text(adb_path, serial, "pidof", "-s", package_name, timeout=10)
        return pid or None
    except Exception:
        return None


def get_device_info(adb_path: Path, serial: str) -> Dict[str, str]:
    return {
        "model": adb_shell_text(adb_path, serial, "getprop", "ro.product.model"),
        "android_version": adb_shell_text(
            adb_path, serial, "getprop", "ro.build.version.release"
        ),
        "wm_size": adb_shell_text(adb_path, serial, "wm", "size"),
    }


def launch_main_activity(
    adb_path: Path, serial: str, package_name: str, main_activity: str
) -> str:
    target = f"{package_name}/{main_activity}"
    proc = run(adb_cmd(adb_path, serial, "shell", "am", "start", "-n", target))
    return (proc.stdout + proc.stderr).strip()


def capture_screenshot(adb_path: Path, serial: str, out_path: Path) -> None:
    remote_path = "/sdcard/codex_screen_capture.png"
    write_proc = run(
        adb_cmd(adb_path, serial, "shell", "screencap", "-p", remote_path),
        timeout=30,
        check=False,
    )
    if write_proc.returncode != 0:
        raise MonitorError(
            f"screencap remote write failed with code {write_proc.returncode}: "
            f"{write_proc.stderr}"
        )
    with out_path.open("wb") as handle:
        proc = subprocess.run(
            adb_cmd(adb_path, serial, "exec-out", "cat", remote_path),
            stdout=handle,
            stderr=subprocess.PIPE,
            check=False,
            timeout=30,
        )
    if proc.returncode != 0:
        raise MonitorError(
            f"screencap pull failed with code {proc.returncode}: "
            f"{proc.stderr.decode('utf-8', errors='replace')}"
        )


def dump_uiautomator(adb_path: Path, serial: str, out_path: Path) -> Optional[str]:
    dump_proc = run(
        adb_cmd(adb_path, serial, "exec-out", "uiautomator", "dump", "/dev/tty"),
        timeout=30,
        check=False,
    )
    if dump_proc.returncode != 0 or not dump_proc.stdout.strip():
        return None
    xml_text = sanitize_uiautomator_output(dump_proc.stdout)
    if not xml_text:
        return None
    out_path.write_text(xml_text, encoding="utf-8")
    return xml_text


def sanitize_uiautomator_output(raw_text: str) -> Optional[str]:
    start = raw_text.find("<?xml")
    end = raw_text.rfind("</hierarchy>")
    if start == -1 or end == -1:
        return None
    end += len("</hierarchy>")
    cleaned = raw_text[start:end].strip()
    return cleaned or None


def get_foreground_info(adb_path: Path, serial: str) -> Dict[str, Optional[str]]:
    try:
        activity_dump = adb_shell_text(
            adb_path,
            serial,
            "dumpsys",
            "activity",
            "activities",
            timeout=10,
        )
    except Exception:
        return {
            "top_resumed_activity": None,
            "resumed_activity": None,
            "current_focus": None,
            "focused_app": None,
        }

    resumed = None
    top_resumed = None
    for line in activity_dump.splitlines():
        stripped = line.strip()
        if stripped.startswith("topResumedActivity="):
            top_resumed = stripped
        if stripped.startswith("ResumedActivity:"):
            resumed = stripped

    return {
        "top_resumed_activity": top_resumed,
        "resumed_activity": resumed,
        "current_focus": None,
        "focused_app": None,
    }


def parse_uiautomator_xml(xml_text: str) -> Dict[str, object]:
    root = ET.fromstring(xml_text)
    nodes: List[Dict[str, object]] = []
    all_texts: List[str] = []
    markers: Dict[str, List[Tuple[float, float]]] = {
        "wifi": [],
        "gnss": [],
        "fused": [],
        "pdr": [],
    }
    switches: Dict[str, Optional[bool]] = {
        "wifi_switch_checked": None,
        "gnss_switch_checked": None,
        "fused_switch_checked": None,
        "pdr_switch_checked": None,
    }

    for node in root.iter("node"):
        attrs = node.attrib
        text = attrs.get("text", "").strip()
        resource_id = attrs.get("resource-id", "")
        content_desc = attrs.get("content-desc", "").strip()
        bounds = attrs.get("bounds", "")
        checked = attrs.get("checked")
        payload = {
            "text": text,
            "resource_id": resource_id,
            "content_desc": content_desc,
            "bounds": bounds,
            "checked": checked,
        }
        nodes.append(payload)
        if text:
            all_texts.append(text)

        if resource_id.endswith("/wifiSwitch"):
            switches["wifi_switch_checked"] = checked == "true"
        elif resource_id.endswith("/gnssSwitch"):
            switches["gnss_switch_checked"] = checked == "true"
        elif resource_id.endswith("/fusedSwitch"):
            switches["fused_switch_checked"] = checked == "true"
        elif resource_id.endswith("/pdrSwitch"):
            switches["pdr_switch_checked"] = checked == "true"

        center = parse_bounds_center(bounds)
        if center is None:
            continue
        marker_key = marker_key_from_desc(content_desc)
        if marker_key:
            markers[marker_key].append(center)

    by_resource = {
        "floor": find_text_by_resource(nodes, "currentFloorStatus"),
        "elevator": find_text_by_resource(nodes, "elevatorStatus"),
        "offset": find_text_by_resource(nodes, "gnssError"),
        "confidence": find_text_by_resource(nodes, "trackingConfidence"),
        "dbg": find_text_by_resource(nodes, "debugStatusLine"),
        "last_update": find_text_by_resource(nodes, "lastUpdateTime"),
        "system_status": find_text_by_resource(nodes, "systemStatus"),
        "context_hint": find_text_by_resource(nodes, "trackingContextHint"),
        "venue": find_text_by_resource(nodes, "selectedVenueText"),
    }

    return {
        "nodes": nodes,
        "all_texts": all_texts,
        "joined_text": "\n".join(all_texts),
        "marker_stats": summarize_markers(markers),
        "switches": switches,
        "by_resource": by_resource,
    }


def parse_bounds_center(bounds: str) -> Optional[Tuple[float, float]]:
    match = re.match(r"\[(\d+),(\d+)\]\[(\d+),(\d+)\]", bounds or "")
    if not match:
        return None
    x1, y1, x2, y2 = [int(v) for v in match.groups()]
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def marker_key_from_desc(content_desc: str) -> Optional[str]:
    if not content_desc:
        return None
    if content_desc == "WiFi Position":
        return "wifi"
    if content_desc in {"GNSS Position", "Device Location"}:
        return "gnss"
    if content_desc == "Fused Position":
        return "fused"
    if content_desc == "PDR Position":
        return "pdr"
    return None


def summarize_markers(
    markers: Dict[str, List[Tuple[float, float]]]
) -> Dict[str, Dict[str, object]]:
    result: Dict[str, Dict[str, object]] = {}
    for key, centers in markers.items():
        centroid = None
        if centers:
            centroid = [
                round(sum(point[0] for point in centers) / len(centers), 2),
                round(sum(point[1] for point in centers) / len(centers), 2),
            ]
        result[key] = {
            "count": len(centers),
            "centroid": centroid,
            "centers": [[round(x, 2), round(y, 2)] for x, y in centers],
        }
    return result


def find_text_by_resource(
    nodes: Sequence[Dict[str, object]], suffix: str
) -> Optional[str]:
    ending = "/" + suffix
    for node in nodes:
        resource_id = str(node.get("resource_id") or "")
        if resource_id.endswith(ending):
            text = str(node.get("text") or "").strip()
            if text:
                return text
    return None


def try_ocr(
    screenshot_path: Path,
    tesseract_path: Optional[str],
    out_path: Path,
) -> Optional[str]:
    if not tesseract_path:
        return None
    cmd = [tesseract_path, str(screenshot_path), "stdout", "--psm", "11"]
    proc = run(cmd, timeout=60, check=False)
    if proc.returncode != 0 or not proc.stdout.strip():
        return None
    out_path.write_text(proc.stdout, encoding="utf-8")
    return proc.stdout


def parse_fields(
    *,
    ui_payload: Optional[Dict[str, object]],
    ocr_text: Optional[str],
    recent_log_lines: Sequence[str],
) -> Dict[str, object]:
    ui_text = ""
    by_resource: Dict[str, Optional[str]] = {}
    marker_stats = {}
    switches = {}
    if ui_payload:
        ui_text = str(ui_payload.get("joined_text") or "")
        by_resource = dict(ui_payload.get("by_resource") or {})
        marker_stats = dict(ui_payload.get("marker_stats") or {})
        switches = dict(ui_payload.get("switches") or {})

    floor_text = by_resource.get("floor") or search_first(
        [ui_text, ocr_text], r"Floor:\s*([^\n\r]+)"
    )
    elevator_text = by_resource.get("elevator") or search_first(
        [ui_text, ocr_text], r"Elevator:\s*([^\n\r]+)"
    )
    offset_text = by_resource.get("offset") or search_first(
        [ui_text, ocr_text],
        r"(?:GNSS|Device)\/fused offset:\s*([0-9]+(?:\.[0-9]+)?)\s*m",
    )
    confidence_text = by_resource.get("confidence") or search_first(
        [ui_text, ocr_text], r"Confidence:\s*([^\n\r]+)"
    )
    dbg_text = by_resource.get("dbg") or search_first(
        [ui_text, ocr_text], r"(DBG [^\n\r]+)"
    )
    last_update_text = by_resource.get("last_update") or search_first(
        [ui_text, ocr_text], r"Last update:\s*([^\n\r]+)"
    )

    dbg_source = "uiautomator" if by_resource.get("dbg") else None
    if not dbg_text:
        synthesized = synthesize_dbg_from_logs(recent_log_lines)
        if synthesized:
            dbg_text = synthesized
            dbg_source = "logcat"

    parsed_offset = parse_float(offset_text)
    parsed_confidence = parse_confidence_value(confidence_text)
    parsed_confidence_label = parse_confidence_label(confidence_text)
    dbg_fields = parse_dbg_fields(dbg_text)

    parse_source = "unknown"
    if any(
        [
            by_resource.get("floor"),
            by_resource.get("elevator"),
            by_resource.get("offset"),
            by_resource.get("confidence"),
            by_resource.get("last_update"),
        ]
    ):
        parse_source = "uiautomator"
    elif any([floor_text, elevator_text, offset_text, confidence_text, last_update_text]):
        parse_source = "ocr"

    return {
        "parsed_floor": clean_field_value(floor_text),
        "parsed_elevator": normalize_elevator(clean_field_value(elevator_text)),
        "parsed_offset_m": parsed_offset,
        "parsed_confidence": parsed_confidence,
        "parsed_confidence_label": parsed_confidence_label,
        "parsed_dbg_raw": clean_field_value(dbg_text),
        "parsed_dbg_fields": dbg_fields,
        "parsed_last_update": clean_field_value(last_update_text),
        "parse_source": parse_source,
        "dbg_parse_source": dbg_source or "unknown",
        "marker_stats": marker_stats,
        "switches": switches,
    }


def search_first(sources: Sequence[Optional[str]], pattern: str) -> Optional[str]:
    regex = re.compile(pattern, re.IGNORECASE)
    for source in sources:
        if not source:
            continue
        match = regex.search(source)
        if match:
            if match.lastindex:
                return match.group(1).strip()
            return match.group(0).strip()
    return None


def clean_field_value(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    cleaned = value.strip()
    return cleaned or None


def parse_float(value: Optional[str]) -> Optional[float]:
    if not value:
        return None
    match = re.search(r"-?[0-9]+(?:\.[0-9]+)?", value)
    if not match:
        return None
    try:
        return float(match.group(0))
    except ValueError:
        return None


def parse_confidence_value(value: Optional[str]) -> Optional[float]:
    if not value:
        return None
    match = re.search(r"\(([0-9]+(?:\.[0-9]+)?)\)", value)
    if match:
        return float(match.group(1))
    floats = re.findall(r"[0-9]+(?:\.[0-9]+)?", value)
    if floats:
        try:
            return float(floats[-1])
        except ValueError:
            return None
    return None


def parse_confidence_label(value: Optional[str]) -> Optional[str]:
    if not value:
        return None
    match = re.search(r"Confidence:\s*([A-Za-z]+)", value)
    if match:
        return match.group(1)
    return None


def parse_dbg_fields(value: Optional[str]) -> Dict[str, Optional[str]]:
    fields: Dict[str, Optional[str]] = {}
    if not value:
        return {
            "fc": None,
            "fs": None,
            "rej": None,
            "hd": None,
            "hold": None,
            "fused": None,
            "disp": None,
        }
    for key in ["fc", "fs", "rej", "hd", "hold", "fused", "disp"]:
        match = re.search(rf"\b{key}=([^\s]+)", value)
        fields[key] = match.group(1) if match else None
    return fields


def synthesize_dbg_from_logs(recent_lines: Sequence[str]) -> Optional[str]:
    pose_line = None
    display_line = None
    for line in recent_lines:
        if "POSE_DIAG" in line and "ui_pose" in line:
            pose_line = line
        if "DISPLAY_DIAG" in line and "display_pose" in line:
            display_line = line

    if not pose_line and not display_line:
        return None

    def extract(line: Optional[str], key: str) -> Optional[str]:
        if not line:
            return None
        match = re.search(rf"\b{re.escape(key)}=([^\s]+)", line)
        return match.group(1) if match else None

    fc = extract(pose_line, "floorConsensus")
    fs = extract(pose_line, "floorSource")
    rej = extract(pose_line, "lastReject") or extract(display_line, "rejectReason")
    hd = extract(pose_line, "headingSource")
    hold = extract(pose_line, "markerHoldReason")
    fused = extract(pose_line, "poseTs")
    disp = extract(pose_line, "displayDecision") or extract(display_line, "decision")

    synthesized = (
        "DBG"
        f" fc={fc or '--'}"
        f" fs={fs or '--'}"
        f" rej={rej or '--'}"
        f" hd={hd or '--'}"
        f" hold={hold or '--'}"
        f" fused={fused or '--'}"
        f" disp={disp or '--'}"
        " src=logcat"
    )
    return synthesized


def normalize_elevator(value: Optional[str]) -> Optional[str]:
    if not value:
        return None
    stripped = value.replace("Elevator:", "").strip()
    upper = stripped.upper()
    if upper in {"ON", "TRUE"}:
        return "On"
    if upper in {"OFF", "FALSE"}:
        return "Off"
    if stripped == "--":
        return "--"
    return stripped


def canonicalize_floor(value: Optional[str]) -> Optional[str]:
    if not value:
        return None
    stripped = value.replace("Floor:", "").strip()
    stripped = re.sub(r"\s+", " ", stripped).upper()
    if stripped.startswith("RELATIVE "):
        return "REL:" + stripped.split(" ", 1)[1]
    return stripped


def classify_floor_pair(previous: str, current: str) -> str:
    if previous in GROUND_FLOOR_TOKENS and current in FIRST_FLOOR_TOKENS:
        return "GF<->1F"
    if previous in FIRST_FLOOR_TOKENS and current in GROUND_FLOOR_TOKENS:
        return "GF<->1F"
    return f"{previous}->{current}"


def marker_jump_events(
    observations: Sequence[Dict[str, object]],
    marker_key: str,
    threshold_px: float,
) -> List[Dict[str, object]]:
    events: List[Dict[str, object]] = []
    previous_center = None
    previous_index = None
    for observation in observations:
        marker_stats = observation.get("marker_stats") or {}
        marker = marker_stats.get(marker_key) if isinstance(marker_stats, dict) else None
        if not isinstance(marker, dict):
            continue
        centroid = marker.get("centroid")
        if not centroid:
            continue
        current_center = (float(centroid[0]), float(centroid[1]))
        if previous_center is not None:
            distance = pixel_distance(previous_center, current_center)
            if distance >= threshold_px:
                events.append(
                    {
                        "from_index": previous_index,
                        "to_index": observation.get("sample_index"),
                        "timestamp": observation.get("timestamp"),
                        "distance_px": round(distance, 2),
                    }
                )
        previous_center = current_center
        previous_index = observation.get("sample_index")
    return events


def pixel_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def monotonic_non_decrease_ratio(values: Sequence[float], tolerance: float = 0.15) -> float:
    if len(values) < 2:
        return 0.0
    non_decrease = 0
    for left, right in zip(values, values[1:]):
        if right + tolerance >= left:
            non_decrease += 1
    return non_decrease / (len(values) - 1)


def summarize_session(
    observations: Sequence[Dict[str, object]],
    *,
    session_dir: Path,
    adb_path: Path,
    adb_source: str,
    device: Dict[str, str],
    device_info: Dict[str, str],
    ocr_available: bool,
    warnings: Sequence[str],
) -> str:
    floors: List[Tuple[str, str]] = []
    floor_transitions: List[str] = []
    previous_floor = None
    for observation in observations:
        floor_raw = observation.get("parsed_floor")
        canonical = canonicalize_floor(floor_raw if isinstance(floor_raw, str) else None)
        if canonical:
            floors.append((observation["timestamp"], canonical))
            if previous_floor and canonical != previous_floor:
                floor_transitions.append(
                    f"{observation['timestamp']} {classify_floor_pair(previous_floor, canonical)}"
                )
            previous_floor = canonical

    elevators = [
        value
        for value in (obs.get("parsed_elevator") for obs in observations)
        if isinstance(value, str) and value
    ]
    elevator_on_count = sum(1 for value in elevators if value == "On")
    elevator_toggles = 0
    previous_elevator = None
    for value in elevators:
        if previous_elevator and value != previous_elevator:
            elevator_toggles += 1
        previous_elevator = value

    offsets = [
        float(value)
        for value in (obs.get("parsed_offset_m") for obs in observations)
        if isinstance(value, (int, float))
    ]
    confidences = [
        float(value)
        for value in (obs.get("parsed_confidence") for obs in observations)
        if isinstance(value, (int, float))
    ]

    dbg_counters = {
        key: Counter(
            value
            for value in (
                (obs.get("parsed_dbg_fields") or {}).get(key)
                for obs in observations
                if isinstance(obs.get("parsed_dbg_fields"), dict)
            )
            if value
        )
        for key in ["fc", "fs", "rej", "hd", "hold", "fused", "disp"]
    }

    wifi_jumps = marker_jump_events(observations, "wifi", threshold_px=60.0)
    fused_jumps = marker_jump_events(observations, "fused", threshold_px=45.0)

    offset_drift = None
    offset_monotonic_ratio = None
    large_offset_drift = False
    if offsets:
        offset_drift = round(offsets[-1] - offsets[0], 3)
        offset_monotonic_ratio = round(monotonic_non_decrease_ratio(offsets), 3)
        large_offset_drift = (
            offset_drift >= 2.0 and (offset_monotonic_ratio or 0.0) >= 0.7
        )

    offset_range = None
    if offsets:
        offset_range = round(max(offsets) - min(offsets), 3)

    abnormal_jitter_reasons: List[str] = []
    if len(floor_transitions) >= 1:
        abnormal_jitter_reasons.append("floor_changed")
    if elevator_toggles >= 1:
        abnormal_jitter_reasons.append("elevator_toggled")
    if offset_range is not None and offset_range >= 2.5:
        abnormal_jitter_reasons.append("offset_swung")
    if wifi_jumps:
        abnormal_jitter_reasons.append("wifi_marker_jump")
    if fused_jumps:
        abnormal_jitter_reasons.append("fused_marker_jump")

    has_core_data = any(
        [
            floors,
            elevators,
            offsets,
            confidences,
        ]
    )
    if len(observations) < 3 or not has_core_data:
        judgment = "insufficient_data"
    elif floor_transitions:
        judgment = "unstable_floor"
    elif elevator_on_count > 0 or elevator_toggles > 0:
        judgment = "false_elevator"
    elif large_offset_drift:
        judgment = "large_offset_drift"
    else:
        judgment = "stable"

    lines: List[str] = []
    lines.append("# ADB Monitoring Summary")
    lines.append("")
    lines.append("## Session")
    lines.append(f"- Artifacts: `{session_dir}`")
    lines.append(f"- Device serial: `{device['serial']}`")
    lines.append(f"- Device model: `{device_info['model']}`")
    lines.append(f"- Android: `{device_info['android_version']}`")
    lines.append(f"- Resolution: `{device_info['wm_size']}`")
    lines.append(f"- adb path: `{adb_path}`")
    lines.append(f"- adb resolution: `{adb_source}`")
    lines.append(f"- Samples captured: `{len(observations)}`")
    lines.append(f"- OCR fallback available: `{ocr_available}`")
    lines.append("")
    lines.append("## Core Findings")
    lines.append(f"- Overall judgment: `{judgment}`")
    lines.append(
        "- Floor transitions: "
        + ("none" if not floor_transitions else "; ".join(floor_transitions))
    )
    lines.append(
        f"- Elevator false trigger evidence: `on_count={elevator_on_count}` `toggle_count={elevator_toggles}`"
    )
    lines.append(
        "- Offset stats: "
        + (
            "insufficient"
            if not offsets
            else f"`min={min(offsets):.2f}m` `max={max(offsets):.2f}m` `last={offsets[-1]:.2f}m` "
            f"`drift={offset_drift:.2f}m` `monotonic_ratio={offset_monotonic_ratio:.2f}`"
        )
    )
    lines.append(
        "- Confidence stats: "
        + (
            "insufficient"
            if not confidences
            else f"`min={min(confidences):.2f}` `max={max(confidences):.2f}`"
        )
    )
    for key in ["fc", "fs", "rej", "hd", "hold", "fused", "disp"]:
        counter = dbg_counters[key]
        most_common = counter.most_common(3)
        if most_common:
            rendered = ", ".join(f"{value} x{count}" for value, count in most_common)
        else:
            rendered = "insufficient"
        lines.append(f"- DBG `{key}` most common: {rendered}")
    lines.append(
        "- WiFi marker jump evidence: "
        + (
            "none"
            if not wifi_jumps
            else "; ".join(
                f"sample {event['from_index']}->{event['to_index']} ({event['distance_px']:.1f}px)"
                for event in wifi_jumps[:5]
            )
        )
    )
    lines.append(
        "- Fused marker jump evidence: "
        + (
            "none"
            if not fused_jumps
            else "; ".join(
                f"sample {event['from_index']}->{event['to_index']} ({event['distance_px']:.1f}px)"
                for event in fused_jumps[:5]
            )
        )
    )
    lines.append(
        "- Static-scene jitter evidence: "
        + ("none" if not abnormal_jitter_reasons else ", ".join(abnormal_jitter_reasons))
    )
    lines.append("")
    lines.append("## Files")
    lines.append("- Structured observations: `observations.jsonl`")
    lines.append("- Raw session log: `session_log.txt`")
    lines.append("- Screenshots: `screen_*.png`")
    lines.append("- UI dumps: `uia_*.xml`")
    lines.append("")
    lines.append("## Notes")
    if warnings:
        for warning in warnings:
            lines.append(f"- {warning}")
    else:
        lines.append("- No additional warnings.")
    return "\n".join(lines) + "\n"


def write_jsonl(path: Path, records: Sequence[Dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Observe PositionMe Recording screen via adb. "
            "Captures screenshots, uiautomator dumps, optional OCR fallback, and summary."
        )
    )
    parser.add_argument("--duration", type=int, default=DEFAULT_DURATION_S)
    parser.add_argument("--interval", type=int, default=DEFAULT_INTERVAL_S)
    parser.add_argument("--serial", help="adb device serial")
    parser.add_argument("--adb", help="explicit adb binary path")
    parser.add_argument("--package", default=DEFAULT_PACKAGE)
    parser.add_argument("--main-activity", default=DEFAULT_MAIN_ACTIVITY)
    parser.add_argument(
        "--launch-main",
        action="store_true",
        help="Attempt to launch the app main activity before monitoring.",
    )
    parser.add_argument(
        "--output-root",
        default="artifacts",
        help="Root directory for artifact output.",
    )
    return parser


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()

    if args.duration <= 0:
        raise SystemExit("--duration must be > 0")
    if args.interval <= 0:
        raise SystemExit("--interval must be > 0")

    session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = Path(args.output_root) / f"adb_monitor_{session_timestamp}"
    session_dir.mkdir(parents=True, exist_ok=False)
    session_log_path = session_dir / "session_log.txt"
    observations_path = session_dir / "observations.jsonl"
    summary_path = session_dir / "summary.md"
    logger = SessionLogger(session_log_path)
    warnings: List[str] = []
    logcat: Optional[LogcatCollector] = None

    try:
        adb_path, adb_source = discover_adb(args.adb)
        adb_version = check_adb(adb_path)
        devices = list_online_devices(adb_path)
        device = choose_device(devices, args.serial)
        serial = device["serial"]
        device_info = get_device_info(adb_path, serial)
        tesseract_path = shutil.which("tesseract")

        logger.log(f"session_dir={session_dir}")
        logger.log(f"adb_version={adb_version.replace(chr(10), ' | ')}")
        logger.log(f"adb_source={adb_source}")
        logger.log(f"device_serial={serial}")
        logger.log(f"device_extras={device.get('extras', '')}")
        logger.log(
            f"device_model={device_info['model']} android={device_info['android_version']} "
            f"wm_size={device_info['wm_size']}"
        )
        if adb_source != "path":
            warnings.append(
                f"`adb` was not resolved from PATH. The script used `{adb_path}`."
            )
        if not tesseract_path:
            warnings.append(
                "OCR fallback unavailable because `tesseract` was not found in PATH."
            )

        if args.launch_main:
            launch_output = launch_main_activity(
                adb_path, serial, args.package, args.main_activity
            )
            logger.log(f"launch_main_output={launch_output}")
            time.sleep(2)

        initial_foreground_info = get_foreground_info(adb_path, serial)
        if initial_foreground_info.get("top_resumed_activity"):
            logger.log(f"foreground={initial_foreground_info['top_resumed_activity']}")
        else:
            warnings.append("Could not read top resumed activity from dumpsys.")

        logcat = LogcatCollector(adb_path, serial, args.package, logger)
        logcat.start()

        sample_count = max(1, int(args.duration // args.interval) + 1)
        observations: List[Dict[str, object]] = []
        start_monotonic = time.monotonic()

        for sample_index in range(sample_count):
            sample_start = time.monotonic()
            now = datetime.now()
            stamp = now.strftime("%Y-%m-%dT%H:%M:%S")
            time_token = now.strftime("%H%M%S")

            screenshot_path = session_dir / f"screen_{sample_index:03d}_{time_token}.png"
            ui_dump_path = session_dir / f"uia_{sample_index:03d}_{time_token}.xml"
            ocr_path = session_dir / f"ocr_{sample_index:03d}_{time_token}.txt"

            capture_errors: List[Exception] = []
            xml_text_holder: Dict[str, Optional[str]] = {"xml": None}

            def screenshot_worker() -> None:
                try:
                    capture_screenshot(adb_path, serial, screenshot_path)
                except Exception as exc:  # pragma: no cover - device dependent
                    capture_errors.append(exc)

            def ui_worker() -> None:
                try:
                    xml_text_holder["xml"] = dump_uiautomator(adb_path, serial, ui_dump_path)
                except Exception as exc:  # pragma: no cover - device dependent
                    capture_errors.append(exc)

            screenshot_thread = threading.Thread(target=screenshot_worker)
            ui_thread = threading.Thread(target=ui_worker)
            screenshot_thread.start()
            ui_thread.start()
            screenshot_thread.join()
            ui_thread.join()

            if capture_errors:
                raise MonitorError(str(capture_errors[0]))

            xml_text = xml_text_holder["xml"]
            ui_payload = None
            if xml_text:
                try:
                    ui_payload = parse_uiautomator_xml(xml_text)
                except ET.ParseError as exc:
                    warnings.append(f"uiautomator XML parse failed at sample {sample_index}: {exc}")
                    ui_payload = None

            recent_log_lines = logcat.snapshot_recent_lines()
            ocr_text = None
            if ui_payload is None or not ui_payload.get("by_resource", {}).get("floor"):
                ocr_text = try_ocr(screenshot_path, tesseract_path, ocr_path)

            parsed = parse_fields(
                ui_payload=ui_payload,
                ocr_text=ocr_text,
                recent_log_lines=recent_log_lines,
            )
            observation = {
                "sample_index": sample_index,
                "timestamp": stamp,
                "elapsed_s": round(sample_start - start_monotonic, 2),
                "parsed_floor": parsed["parsed_floor"],
                "parsed_elevator": parsed["parsed_elevator"],
                "parsed_offset_m": parsed["parsed_offset_m"],
                "parsed_confidence": parsed["parsed_confidence"],
                "parsed_confidence_label": parsed["parsed_confidence_label"],
                "parsed_dbg_raw": parsed["parsed_dbg_raw"],
                "parsed_dbg_fields": parsed["parsed_dbg_fields"],
                "parsed_last_update": parsed["parsed_last_update"],
                "screenshot_path": str(screenshot_path),
                "ui_dump_path": str(ui_dump_path) if ui_dump_path.exists() else None,
                "ocr_text_path": str(ocr_path) if ocr_path.exists() else None,
                "parse_source": parsed["parse_source"],
                "dbg_parse_source": parsed["dbg_parse_source"],
                "foreground": initial_foreground_info,
                "marker_stats": parsed["marker_stats"],
                "layer_switches": parsed["switches"],
            }
            observations.append(observation)

            logger.log(
                "sample="
                + json.dumps(
                    {
                        "sample_index": sample_index,
                        "timestamp": stamp,
                        "parse_source": parsed["parse_source"],
                        "floor": parsed["parsed_floor"],
                        "elevator": parsed["parsed_elevator"],
                        "offset_m": parsed["parsed_offset_m"],
                        "confidence": parsed["parsed_confidence"],
                        "dbg": parsed["parsed_dbg_raw"],
                        "wifi_centroid": (
                            (parsed["marker_stats"] or {})
                            .get("wifi", {})
                            .get("centroid")
                        ),
                        "fused_centroid": (
                            (parsed["marker_stats"] or {})
                            .get("fused", {})
                            .get("centroid")
                        ),
                    },
                    ensure_ascii=False,
                )
            )

            next_target = start_monotonic + ((sample_index + 1) * args.interval)
            sleep_seconds = next_target - time.monotonic()
            if sample_index < sample_count - 1 and sleep_seconds > 0:
                time.sleep(sleep_seconds)

        logcat.stop()
        write_jsonl(observations_path, observations)
        summary_text = summarize_session(
            observations,
            session_dir=session_dir,
            adb_path=adb_path,
            adb_source=adb_source,
            device=device,
            device_info=device_info,
            ocr_available=bool(tesseract_path),
            warnings=warnings,
        )
        summary_path.write_text(summary_text, encoding="utf-8")

        print(f"Artifacts: {session_dir}")
        print(f"Summary:   {summary_path}")
        print(f"Observations: {observations_path}")
        return 0
    except MonitorError as exc:
        logger.log(f"ERROR {exc}")
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if logcat is not None:
            try:
                logcat.stop()
            except Exception:
                pass
        logger.close()


if __name__ == "__main__":
    raise SystemExit(main())
