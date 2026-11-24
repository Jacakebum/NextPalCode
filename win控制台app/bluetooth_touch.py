from __future__ import annotations
import math
import json
import os
import queue
import re
import socket
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from tkinter import messagebox, ttk
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import serial
from PIL import Image, ImageOps, ImageTk
from serial.tools import list_ports

try:
    import winreg
except ImportError:
    winreg = None

APP_NAME = "NextPal 四足机器人控制台"
APP_VERSION = "3.4.0"
DEFAULT_BAUD = 9600
EMERGENCY_ACTION = "整机回中站立"
BLUETOOTH_ADDRESS_RE = re.compile(r"&([0-9A-F]{12})_C", re.IGNORECASE)
ROBOT_PORT_HINTS = ("1004", "robot", "机器", "小狗")
HOME_COMMANDS = tuple(f"7{number}".encode("ascii") for number in range(1, 7))
DEFAULT_HA_URL = os.getenv("HOME_ASSISTANT_URL", "http://172.20.10.9:8123")
DEFAULT_HA_TOKEN = os.getenv("HOME_ASSISTANT_TOKEN", "")
DEFAULT_HA_ON_SCENE = os.getenv("HOME_ASSISTANT_ON_SCENE", "scene.kai_deng")
DEFAULT_HA_OFF_SCENE = os.getenv("HOME_ASSISTANT_OFF_SCENE", "scene.guan_deng")
DEFAULT_HA_NORMAL_SCENE = os.getenv("HOME_ASSISTANT_NORMAL_SCENE", "scene.zheng_chang")
DEFAULT_HA_PURPLE_SCENE = os.getenv("HOME_ASSISTANT_PURPLE_SCENE", "scene.zi_se_xin_qing")
HA_REQUEST_TIMEOUT = 5
NEXTPAL_API_HOST = os.getenv("NEXTPAL_API_HOST", "0.0.0.0")
NEXTPAL_API_PORT = int(os.getenv("NEXTPAL_API_PORT", "8765"))
NEXTPAL_API_TOKEN = os.getenv("NEXTPAL_API_TOKEN", "")
DEFAULT_CAMERA_SOURCE = os.getenv(
    "NEXTPAL_CAMERA_SOURCE", "tcp://192.168.100.102:8554"
)
CAMERA_RECONNECT_DELAY = 2.0
HA_LIGHT_STATE_BY_COMMAND = {
    "71": True,
    "72": False,
    "73": True,
    "74": False,
    "75": True,
    "76": False,
}
LIGHT_CODES = {
    (1, "on"): "71",
    (1, "off"): "72",
    (2, "on"): "73",
    (2, "off"): "74",
    (3, "on"): "75",
    (3, "off"): "76",
}


def _parse_voice_text(text: str):
    normalized = re.sub(r"[\s，。,.!?！？、]+", "", text.casefold())
    if "灯" in normalized and any(
        phrase in normalized for phrase in ("关灯", "关闭灯", "关掉灯", "关")
    ):
        return ("light", "72")
    if "灯" in normalized and any(
        phrase in normalized for phrase in ("开灯", "打开灯", "开启灯", "开")
    ):
        return ("light", "71")
    if any(
        phrase in normalized
        for phrase in ("紫色心情", "紫色模式", "调成紫色", "切换到紫色", "紫色")
    ):
        return ("preset", "purple")
    if any(
        phrase in normalized
        for phrase in ("正常模式", "恢复正常", "切换到正常", "正常")
    ):
        return ("preset", "normal")
    return None


COLORS = {
    "bg": "#0d1017",
    "panel": "#171b25",
    "panel_soft": "#202633",
    "line": "#303849",
    "text": "#f3f5fa",
    "muted": "#929bad",
    "accent": "#5c70f5",
    "accent_dark": "#465bd9",
    "accent_soft": "#293151",
    "green": "#42d392",
    "green_soft": "#173b31",
    "red": "#ff5c68",
    "red_soft": "#45242b",
    "orange": "#ff9f43",
    "blue": "#5c9dff",
    "shadow": "#080a0f",
    "terminal": "#090b10",
}


@dataclass(frozen=True)
class RobotCommand:
    code: str
    name: str
    icon: str
    category: str
    action: str
    reply: str


COMMANDS = [
    # 基础移动控制
    RobotCommand("00", "整机回中站立", "◎", "基础", "整机回中并站立", ""),
    RobotCommand("30", "好狗", "★", "基础", "整机回中站立，并附带语音指令“好狗”", ""),
    RobotCommand("01", "左转", "↶", "移动", "机器人左转", ""),
    RobotCommand("02", "前进", "↑", "移动", "机器人前进", ""),
    RobotCommand("03", "右转", "↷", "移动", "机器人右转", ""),
    RobotCommand("04", "左移", "←", "移动", "机器人左移", ""),
    RobotCommand("06", "右移", "→", "移动", "机器人右移", ""),
    RobotCommand("08", "后退", "↓", "移动", "机器人后退", ""),
    RobotCommand("09", "右转(09)", "↷", "移动", "机器人右转，发送备用右转码 09", ""),
    RobotCommand("0A", "小跑跳跃", "↥", "移动", "机器人执行小跑跳跃", ""),

    # 步态模式切换
    RobotCommand("10", "慢走 Walk", "🚶", "步态", "切换到慢走 Walk 模式", ""),
    RobotCommand("11", "小跑 Trot", "🏃", "步态", "切换到小跑 Trot 模式", ""),
    RobotCommand("12", "站立 Stand", "▥", "步态", "切换到站立 Stand 模式", ""),
    RobotCommand("13", "身体扭动 Body_Twist", "≈", "步态", "切换到身体扭动 Body_Twist 模式", ""),
    RobotCommand("14", "跳跃 Jump", "↟", "步态", "切换到跳跃 Jump 模式", ""),
    RobotCommand("07", "IMU自稳", "⌁", "步态", "开启 IMU 自稳；需先发送 13", ""),

    # 头部姿态调节
    RobotCommand("33", "抬头", "↑", "头部", "头部抬头", ""),
    RobotCommand("34", "低头", "↓", "头部", "头部低头", ""),
    RobotCommand("35", "Roll+5", "↗", "头部", "头部 Roll 增加 5", ""),
    RobotCommand("36", "Roll-5", "↘", "头部", "头部 Roll 减少 5", ""),
    RobotCommand("37", "点头", "✓", "头部", "机器人点头", ""),
    RobotCommand("38", "摇头", "↔", "头部", "机器人摇头", ""),
    RobotCommand("39", "歪头", "?", "头部", "机器人歪头", ""),
    RobotCommand("24", "扭头看左", "↩", "头部", "扭头看左边", ""),
    RobotCommand("25", "扭头看右", "↪", "头部", "扭头看右边", ""),
    RobotCommand("26", "语音点头", "✓", "头部", "执行语音点头动作", ""),
    RobotCommand("27", "语音歪头", "?", "头部", "执行语音歪头动作", ""),

    # 尾巴控制
    RobotCommand("40", "尾巴回中", "◎", "尾巴", "尾巴回中", ""),
    RobotCommand("41", "尾巴左摆", "←", "尾巴", "尾巴向左摆", ""),
    RobotCommand("42", "尾巴右摆", "→", "尾巴", "尾巴向右摆", ""),
    RobotCommand("43", "摇尾巴 Wag", "〰", "尾巴", "切换到摇尾巴 Wag 动作", ""),
    RobotCommand("22", "尾巴左右切换", "↔", "尾巴", "尾巴左右切换，附带语音“原地踏步”", ""),

    # 嘴巴控制
    RobotCommand("05", "嘴巴测试切换", "◐", "嘴巴", "切换嘴巴测试状态", ""),
    RobotCommand("23", "嘴巴开关切换", "◑", "嘴巴", "切换嘴巴开关状态，附带语音“扭一扭”", ""),
    RobotCommand("31", "嘴巴切换", "♫", "嘴巴", "切换嘴巴状态，附带语音“叫 / 看我”", ""),
    RobotCommand("50", "嘴巴半开回中", "◒", "嘴巴", "嘴巴半开并回中", ""),
    RobotCommand("51", "嘴巴张大", "◓", "嘴巴", "嘴巴张大", ""),
    RobotCommand("52", "嘴巴闭合", "●", "嘴巴", "嘴巴闭合", ""),

    # 运动参数调节
    RobotCommand("61", "步长增加", "+", "参数", "步长增加，范围 20~75", ""),
    RobotCommand("62", "步长减少", "-", "参数", "步长减少，范围 20~75", ""),
    RobotCommand("63", "步态周期增加", "+", "参数", "步态周期增加，范围 25~90", ""),
    RobotCommand("64", "步态周期减少", "-", "参数", "步态周期减少，范围 25~90", ""),
    RobotCommand("65", "抬腿高度增加", "+", "参数", "抬腿高度增加，范围 12~48", ""),
    RobotCommand("66", "抬腿高度减少", "-", "参数", "抬腿高度减少，范围 12~48", ""),

    # 智能家居联动
    RobotCommand("71", "智能灯1开启", "L1 ON", "联动", "通过 Home Assistant 开灯", ""),
    RobotCommand("72", "智能灯1关闭", "L1 OFF", "联动", "通过 Home Assistant 关灯", ""),
    RobotCommand("73", "智能灯2开启", "L2 ON", "联动", "通过 Home Assistant 开灯", ""),
    RobotCommand("74", "智能灯2关闭", "L2 OFF", "联动", "通过 Home Assistant 关灯", ""),
    RobotCommand("75", "智能灯3开启", "L3 ON", "联动", "通过 Home Assistant 开灯", ""),
    RobotCommand("76", "智能灯3关闭", "L3 OFF", "联动", "通过 Home Assistant 关灯", ""),
]

COMMAND_BY_NAME = {item.name: item for item in COMMANDS}
VALID_COMMAND_CODES = {item.code for item in COMMANDS}


def _is_bluetooth_incoming_port(port) -> bool:
    hwid = (port.hwid or "").upper()
    return "BTHENUM" in hwid and (
        "LOCALMFG&0000" in hwid or "000000000000_000000" in hwid
    )


def _bluetooth_device_name(port) -> str:
    if winreg is None:
        return ""

    match = BLUETOOTH_ADDRESS_RE.search(port.hwid or "")
    if not match:
        return ""

    registry_path = (
        r"SYSTEM\CurrentControlSet\Services\BTHPORT\Parameters\Devices"
        rf"\{match.group(1).lower()}"
    )
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, registry_path) as key:
            value, _kind = winreg.QueryValueEx(key, "Name")
    except OSError:
        return ""

    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").rstrip("\x00")
    return str(value).rstrip("\x00")


def _port_display_name(port) -> str:
    device_name = _bluetooth_device_name(port)
    if device_name:
        return f"{port.device} — {device_name}（蓝牙传出）"
    return f"{port.device} — {port.description}"


class HomeAssistantApiServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, server_address, app):
        self.app = app
        super().__init__(server_address, HomeAssistantApiHandler)


class HomeAssistantApiHandler(BaseHTTPRequestHandler):
    server: HomeAssistantApiServer

    def log_message(self, _format, *_args):
        return

    def _authorized(self) -> bool:
        token = NEXTPAL_API_TOKEN
        if not token:
            return True
        authorization = self.headers.get("Authorization", "")
        supplied = authorization.removeprefix("Bearer ").strip()
        return supplied == token or self.headers.get("X-NextPal-Token", "") == token

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length > 4096:
            raise ValueError("请求体过大")
        raw = self.rfile.read(length) if length else b"{}"
        payload = json.loads(raw.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("请求体必须是 JSON 对象")
        return payload

    def do_GET(self):
        if self.path != "/api/health":
            self._send_json(404, {"ok": False, "error": "not_found"})
            return
        self._send_json(200, {
            "ok": True,
            "robot_connected": self.server.app.link.connected,
            "light_connected": self.server.app.ha_available,
            "home_assistant_available": self.server.app.ha_available,
        })

    def do_POST(self):
        if not self._authorized():
            self._send_json(401, {"ok": False, "error": "unauthorized"})
            return
        try:
            payload = self._read_json()
            if self.path == "/api/command":
                code = str(payload.get("code", "")).upper()
                if code not in VALID_COMMAND_CODES:
                    raise ValueError("code 必须是动作表中的两位指令")
                self.server.app.events.put(("api", "command", code))
            elif self.path == "/api/light":
                channel = int(payload.get("channel", 0))
                state = str(payload.get("state", "")).lower()
                code = LIGHT_CODES.get((channel, state))
                if not code:
                    raise ValueError("channel 必须为 1~3，state 必须为 on 或 off")
                self.server.app.events.put(("api", "light", code))
            elif self.path == "/api/scene":
                preset = str(payload.get("preset", "")).lower()
                if preset not in ("normal", "purple"):
                    raise ValueError("preset 必须是 normal 或 purple")
                self.server.app.events.put(("api", "scene", preset))
            elif self.path == "/api/voice":
                text = str(payload.get("text", "")).strip()
                action = _parse_voice_text(text)
                if not action:
                    raise ValueError("无法识别语音指令，可用：开灯、关灯、正常、紫色心情")
                self.server.app.events.put(("api", "voice", action))
            else:
                self._send_json(404, {"ok": False, "error": "not_found"})
                return
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._send_json(400, {"ok": False, "error": str(exc)})
            return
        self._send_json(202, {"ok": True, "queued": True})


class SerialLink:
    def __init__(self, events: queue.Queue, channel: str):
        self.events = events
        self.channel = channel
        self.port: serial.Serial | None = None
        self.running = False
        self.reader: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return bool(self.port and self.port.is_open)

    def connect(self, port_name: str, baud: int) -> None:
        self.disconnect()
        self.port = serial.Serial(port_name, baudrate=baud, timeout=0.1, write_timeout=1)
        self.running = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        self.events.put((self.channel, "connected", port_name))

    def disconnect(self) -> None:
        self.running = False
        port = self.port
        self.port = None
        if port:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass

    def send(self, payload: bytes) -> None:
        if not self.connected or not self.port:
            raise serial.SerialException("尚未连接蓝牙串口")
        self.port.write(payload)
        self.events.put((self.channel, "tx", payload))

    def _read_loop(self) -> None:
        assert self.port is not None
        try:
            while self.running and self.port.is_open:
                data = self.port.read(self.port.in_waiting or 1)
                if data:
                    self.events.put((self.channel, "rx", data))
        except (serial.SerialException, OSError) as exc:
            if self.running:
                self.events.put((self.channel, "error", str(exc)))
        finally:
            self.running = False
            self.events.put((self.channel, "disconnected", None))


class CameraStreamClient:
    """Receive the board's compact RGB565 preview without owning the camera."""

    def __init__(self, events: queue.Queue, frames: queue.Queue):
        self.events = events
        self.frames = frames
        self.source = ""
        self.running = False
        self.thread: threading.Thread | None = None
        self.socket: socket.socket | None = None

    def start(self, source: str) -> None:
        self.stop()
        self.source = source.strip()
        self.running = True
        self.thread = threading.Thread(
            target=self._run, name="NextPalCameraStream", daemon=True
        )
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        worker = self.thread
        self.thread = None
        connection = self.socket
        self.socket = None
        if connection:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                connection.close()
            except OSError:
                pass
        if worker and worker is not threading.current_thread():
            worker.join(timeout=1.0)

    @staticmethod
    def _parse_source(source: str) -> tuple[str, int]:
        value = source.strip()
        if value.startswith("tcp://"):
            value = value[6:]
        host, separator, port_text = value.rpartition(":")
        if not separator or not host:
            raise ValueError("视频地址格式应为 tcp://板卡IP:端口")
        port = int(port_text)
        if not 1 <= port <= 65535:
            raise ValueError("视频端口必须在 1~65535 之间")
        return host, port

    @staticmethod
    def _receive_exact(connection: socket.socket, size: int) -> bytes:
        chunks = bytearray()
        while len(chunks) < size:
            block = connection.recv(size - len(chunks))
            if not block:
                raise ConnectionError("视频流已断开")
            chunks.extend(block)
        return bytes(chunks)

    def _put_latest(self, frame) -> None:
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            self.frames.put_nowait(frame)

    def _run(self) -> None:
        try:
            host, port = self._parse_source(self.source)
        except (ValueError, TypeError) as exc:
            self.events.put(("camera", "error", str(exc)))
            return

        last_error = ""
        while self.running:
            try:
                self.events.put(("camera", "connecting", f"{host}:{port}"))
                connection = socket.create_connection((host, port), timeout=4)
                connection.settimeout(5)
                self.socket = connection
                self.events.put(("camera", "connected", f"{host}:{port}"))
                last_error = ""
                while self.running:
                    header = self._receive_exact(connection, 16)
                    if header[:4] != b"NPF1":
                        raise ConnectionError("视频流协议不匹配")
                    width, height, payload_size, sequence = struct.unpack(
                        "!HHII", header[4:]
                    )
                    expected = width * height * 2
                    if not width or not height or payload_size != expected:
                        raise ConnectionError("视频帧尺寸无效")
                    payload = self._receive_exact(connection, payload_size)
                    self._put_latest((width, height, payload, sequence))
            except (OSError, ValueError, ConnectionError) as exc:
                if not self.running:
                    break
                error = str(exc)
                if error != last_error:
                    self.events.put(("camera", "error", error))
                    last_error = error
            finally:
                connection = self.socket
                self.socket = None
                if connection:
                    try:
                        connection.close()
                    except OSError:
                        pass
            deadline = time.monotonic() + CAMERA_RECONNECT_DELAY
            while self.running and time.monotonic() < deadline:
                time.sleep(0.1)


class ScrollFrame(ttk.Frame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.canvas = tk.Canvas(self, bg=COLORS["bg"], highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.content = ttk.Frame(self.canvas)
        self.window = self.canvas.create_window((0, 0), window=self.content, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.content.bind("<Configure>", lambda _e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self.window, width=e.width))
        self.canvas.bind_all("<MouseWheel>", self._wheel)

    def _wheel(self, event) -> None:
        if self.winfo_ismapped():
            self.canvas.yview_scroll(int(-event.delta / 120), "units")


class RobotJoystick(tk.Canvas):
    def __init__(self, master, callback, **kwargs):
        super().__init__(master, bg=COLORS["panel"], highlightthickness=0, takefocus=True, **kwargs)
        self.callback = callback
        self.x_value = 0.0
        self.y_value = 0.0
        self.active = False
        self.keys: set[str] = set()
        self.bind("<Configure>", lambda _e: self.redraw())
        self.bind("<Button-1>", self._press)
        self.bind("<B1-Motion>", self._move)
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<KeyPress>", self._key_down)
        self.bind("<KeyRelease>", self._key_up)

    def _geometry(self) -> tuple[float, float, float]:
        width, height = self.winfo_width(), self.winfo_height()
        radius = max(70, min(width, height) * 0.34)
        return width / 2, height / 2, radius

    def redraw(self) -> None:
        self.delete("all")
        cx, cy, radius = self._geometry()
        self.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill="#fffdfa", outline=COLORS["line"], width=2)
        self.create_oval(cx - radius * .67, cy - radius * .67, cx + radius * .67, cy + radius * .67, fill=COLORS["accent_soft"], outline="#ffd2b8")
        self.create_line(cx - radius * .83, cy, cx + radius * .83, cy, fill="#ffc7a8")
        self.create_line(cx, cy - radius * .83, cx, cy + radius * .83, fill="#ffc7a8")
        knob_x = cx + self.x_value * radius
        knob_y = cy - self.y_value * radius
        knob = radius * .27
        self.create_oval(knob_x - knob * 1.35, knob_y - knob * 1.35, knob_x + knob * 1.35, knob_y + knob * 1.35, fill="#ffe4d4", outline="")
        self.create_oval(knob_x - knob, knob_y - knob, knob_x + knob, knob_y + knob, fill=COLORS["accent"], outline="")
        self.create_oval(knob_x - 4, knob_y - 4, knob_x + 4, knob_y + 4, fill="#ffe6d7", outline="")
        self.create_text(cx, cy - radius - 16, text="前进", fill=COLORS["muted"])
        self.create_text(cx, cy + radius + 16, text="后退", fill=COLORS["muted"])
        self.create_text(cx - radius - 24, cy, text="左移", fill=COLORS["muted"])
        self.create_text(cx + radius + 24, cy, text="右移", fill=COLORS["muted"])

    def _pointer(self, event) -> None:
        cx, cy, radius = self._geometry()
        dx, dy = event.x - cx, event.y - cy
        distance = math.hypot(dx, dy)
        if distance > radius:
            dx, dy = dx * radius / distance, dy * radius / distance
        self.x_value, self.y_value = dx / radius, -dy / radius
        self.redraw()
        self.callback(self.x_value, self.y_value, False)

    def _press(self, event) -> None:
        self.focus_set()
        self.active = True
        self._pointer(event)

    def _move(self, event) -> None:
        if self.active:
            self._pointer(event)

    def _release(self, _event=None) -> None:
        self.active = False
        self.keys.clear()
        self.x_value = self.y_value = 0.0
        self.redraw()
        self.callback(0.0, 0.0, True)

    def _key_down(self, event) -> None:
        key = event.keysym.lower()
        if key not in {"w", "a", "s", "d", "up", "down", "left", "right"}:
            return
        self.keys.add(key)
        x = int(bool({"d", "right"} & self.keys)) - int(bool({"a", "left"} & self.keys))
        y = int(bool({"w", "up"} & self.keys)) - int(bool({"s", "down"} & self.keys))
        length = max(1.0, math.hypot(x, y))
        self.x_value, self.y_value = x / length, y / length
        self.redraw()
        self.callback(self.x_value, self.y_value, False)

    def _key_up(self, event) -> None:
        self.keys.discard(event.keysym.lower())
        if not self.keys:
            self._release()
        else:
            remaining = type("Key", (), {"keysym": next(iter(self.keys))})
            self._key_down(remaining)


class RobotDogApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_NAME)
        self.geometry("1280x860")
        self.minsize(1060, 720)
        if os.name == "nt":
            self.state("zoomed")
        self.configure(bg=COLORS["bg"])
        self.protocol("WM_DELETE_WINDOW", self._close)

        self.events: queue.Queue = queue.Queue()
        self.link = SerialLink(self.events, "robot")
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.ha_url_var = tk.StringVar(value=DEFAULT_HA_URL)
        self.ha_token_var = tk.StringVar(value=DEFAULT_HA_TOKEN)
        self.ha_on_scene_var = tk.StringVar(value=DEFAULT_HA_ON_SCENE)
        self.ha_off_scene_var = tk.StringVar(value=DEFAULT_HA_OFF_SCENE)
        self.ha_normal_scene_var = tk.StringVar(value=DEFAULT_HA_NORMAL_SCENE)
        self.ha_purple_scene_var = tk.StringVar(value=DEFAULT_HA_PURPLE_SCENE)
        self.camera_source_var = tk.StringVar(value=DEFAULT_CAMERA_SOURCE)
        self.camera_status_text = tk.StringVar(value="正在连接 2K300 摄像头…")
        self.page_title = tk.StringVar(value="机器人遥控")
        self.page_subtitle = tk.StringVar(value="两位文本指令 · 蓝牙串口控制")
        self.connection_text = tk.StringVar(value="未连接")
        self.light_connection_text = tk.StringVar(value="HA 未验证")
        self.status_text = tk.StringVar(value="等待连接蓝牙串口")
        self.light_status_text = tk.StringVar(value="等待灯光操作")
        self.tx_count = 0
        self.rx_count = 0
        self.ha_request_count = 0
        self.ack_buffer = b""
        self.home_command_buffer = b""
        self.ha_available = False
        self.api_server: HomeAssistantApiServer | None = None
        self.api_thread: threading.Thread | None = None
        self.last_direction: str | None = None
        self.last_direction_time = 0.0
        self.active_command: RobotCommand | None = None
        self.settings_visible = False
        self.camera_frames: queue.Queue = queue.Queue(maxsize=1)
        self.camera_client = CameraStreamClient(self.events, self.camera_frames)
        self.camera_image: Image.Image | None = None
        self.camera_photo: ImageTk.PhotoImage | None = None
        default_path = Path(__file__).with_name("robot_dog_default.jpg")
        self.camera_default_image = Image.open(default_path).convert("RGB")
        self.recent_action = tk.StringVar(value="暂无指令")
        self.recent_reply = tk.StringVar(value="等待 STM32 回复")
        self._configure_style()
        self._build_shell()
        self._build_pages()
        self.show_page("control")
        self.refresh_ports()
        self._start_home_assistant_api()
        self.camera_client.start(self.camera_source_var.get())
        self.after(60, self._poll_events)

    def _start_home_assistant_api(self) -> None:
        try:
            self.api_server = HomeAssistantApiServer(
                (NEXTPAL_API_HOST, NEXTPAL_API_PORT),
                self,
            )
        except OSError as exc:
            self.status_text.set(f"API 启动失败：{exc}")
            return
        self.api_thread = threading.Thread(
            target=self.api_server.serve_forever,
            name="NextPalHomeAssistantApi",
            daemon=True,
        )
        self.api_thread.start()

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", font=("Microsoft YaHei UI", 10), background=COLORS["bg"], foreground=COLORS["text"])
        style.configure("TFrame", background=COLORS["bg"])
        style.configure("Panel.TFrame", background=COLORS["panel"])
        style.configure("Soft.TFrame", background=COLORS["panel_soft"])
        style.configure("TLabel", background=COLORS["bg"], foreground=COLORS["text"])
        style.configure("Panel.TLabel", background=COLORS["panel"], foreground=COLORS["text"])
        style.configure("Soft.TLabel", background=COLORS["panel_soft"], foreground=COLORS["text"])
        style.configure("Muted.TLabel", background=COLORS["bg"], foreground=COLORS["muted"])
        style.configure("PanelMuted.TLabel", background=COLORS["panel"], foreground=COLORS["muted"])
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 20, "bold"), background=COLORS["bg"])
        style.configure("CardTitle.TLabel", font=("Microsoft YaHei UI", 12, "bold"), background=COLORS["panel"])
        style.configure("Metric.TLabel", font=("Segoe UI", 25, "bold"), background=COLORS["panel"], foreground=COLORS["accent"])
        style.configure("TButton", padding=(12, 8), borderwidth=1)
        style.configure(
            "TEntry", fieldbackground=COLORS["panel_soft"],
            foreground=COLORS["text"], insertcolor=COLORS["text"],
            bordercolor=COLORS["line"], lightcolor=COLORS["line"],
            darkcolor=COLORS["line"],
        )
        style.configure(
            "TCombobox", fieldbackground=COLORS["panel_soft"],
            background=COLORS["panel_soft"], foreground=COLORS["text"],
            arrowcolor=COLORS["text"], bordercolor=COLORS["line"],
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", COLORS["panel_soft"])],
            foreground=[("readonly", COLORS["text"])],
        )
        style.configure("Accent.TButton", background=COLORS["accent"], foreground="white", bordercolor=COLORS["accent"])
        style.map("Accent.TButton", background=[("active", COLORS["accent_dark"])])
        style.configure("Danger.TButton", background=COLORS["red"], foreground="white", bordercolor=COLORS["red"], font=("Microsoft YaHei UI", 11, "bold"))
        style.map("Danger.TButton", background=[("active", "#d92525")])
        style.configure("Nav.TButton", background=COLORS["panel"], foreground=COLORS["muted"], borderwidth=0, padding=(16, 12))
        style.configure("NavActive.TButton", background=COLORS["accent"], foreground="white", borderwidth=0, padding=(16, 12))
        style.configure("Chip.TButton", background=COLORS["panel"], foreground=COLORS["muted"], padding=(12, 5))
        style.configure("ChipActive.TButton", background=COLORS["accent"], foreground="white", padding=(12, 5))
        style.configure("TCheckbutton", background=COLORS["panel"])

    def _build_shell(self) -> None:
        self.vertical_pane = tk.PanedWindow(
            self, orient="vertical", bg=COLORS["line"], bd=0,
            sashwidth=8, sashrelief="flat", showhandle=True,
            opaqueresize=True,
        )
        self.vertical_pane.pack(fill="both", expand=True)
        header_host = ttk.Frame(self.vertical_pane)
        body_host = ttk.Frame(self.vertical_pane)
        self.vertical_pane.add(header_host, minsize=180, height=320)
        self.vertical_pane.add(body_host, minsize=330)

        self.horizontal_pane = tk.PanedWindow(
            header_host, orient="horizontal", bg=COLORS["line"], bd=0,
            sashwidth=8, sashrelief="flat", showhandle=True,
            opaqueresize=True,
        )
        self.horizontal_pane.pack(
            fill="both", expand=True, padx=18, pady=(14, 10)
        )

        camera_card = ttk.Frame(
            self.horizontal_pane, style="Panel.TFrame", padding=8
        )
        self.horizontal_pane.add(camera_card, minsize=320, width=600)
        camera_top = ttk.Frame(camera_card, style="Panel.TFrame")
        camera_top.pack(fill="x", padx=4, pady=(1, 7))
        ttk.Label(
            camera_top, text="USB 摄像头 · 2K300", style="CardTitle.TLabel"
        ).pack(side="left")
        ttk.Label(
            camera_top, text="拖动右侧 / 下方分隔线调节大小",
            style="PanelMuted.TLabel",
        ).pack(side="left", padx=(16, 0))
        self.camera_dot = tk.Canvas(
            camera_top, width=10, height=10, bg=COLORS["panel"],
            highlightthickness=0,
        )
        self.camera_dot.pack(side="right", padx=(7, 0))
        self.camera_dot_id = self.camera_dot.create_oval(
            1, 1, 9, 9, fill=COLORS["red"], outline=""
        )
        ttk.Label(
            camera_top, textvariable=self.camera_status_text,
            style="PanelMuted.TLabel",
        ).pack(side="right")
        self.camera_canvas = tk.Canvas(
            camera_card, width=500, height=245, bg="#090b12",
            highlightthickness=0,
        )
        self.camera_canvas.pack(fill="both", expand=True)
        self.camera_canvas.bind("<Configure>", self._camera_resized)

        connection = ttk.Frame(
            self.horizontal_pane, style="Panel.TFrame", padding=(14, 10)
        )
        self.horizontal_pane.add(connection, minsize=390)
        connection_header = ttk.Frame(connection, style="Panel.TFrame")
        connection_header.pack(fill="x")
        brand = ttk.Frame(connection_header, style="Panel.TFrame")
        brand.pack(side="left")
        ttk.Label(
            brand, text="NextPal 四足机器人控制台", style="CardTitle.TLabel"
        ).pack(anchor="w")
        ttk.Label(
            brand, text="监控 · 遥控 · 家居联动", style="PanelMuted.TLabel"
        ).pack(anchor="w", pady=(2, 0))
        self.settings_button = ttk.Button(
            connection_header, text="⚙  设置", command=self.toggle_settings
        )
        self.settings_button.pack(side="right")

        self.connection_overview = ttk.Frame(connection, style="Panel.TFrame")
        self.connection_overview.pack(fill="both", expand=True, pady=(18, 8))
        for title, variable in (
            ("机器人连接", self.connection_text),
            ("Home Assistant", self.light_connection_text),
            ("摄像头画面", self.camera_status_text),
        ):
            block = ttk.Frame(
                self.connection_overview, style="Soft.TFrame", padding=(13, 12)
            )
            block.pack(fill="x", pady=4)
            ttk.Label(block, text=title, style="Soft.TLabel").pack(side="left")
            ttk.Label(
                block, textvariable=variable, style="Soft.TLabel",
                font=("Microsoft YaHei UI", 10, "bold"),
            ).pack(side="right")

        self.settings_panel = ttk.Frame(connection, style="Panel.TFrame")
        robot_connection = ttk.Frame(self.settings_panel, style="Panel.TFrame")
        robot_connection.pack(fill="x", pady=(0, 5))
        ttk.Label(robot_connection, text="机器人", style="PanelMuted.TLabel", width=7).pack(side="left")
        self.port_combo = ttk.Combobox(robot_connection, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.pack(side="left", padx=(0, 7))
        ttk.Combobox(robot_connection, textvariable=self.baud_var, values=("9600",), width=8, state="readonly").pack(side="left", padx=(0, 7))
        self.connect_button = ttk.Button(robot_connection, text="连接", style="Accent.TButton", command=self.toggle_connection)
        self.connect_button.pack(side="left")

        camera_config = ttk.Frame(self.settings_panel, style="Panel.TFrame")
        camera_config.pack(fill="x", pady=(0, 5))
        ttk.Label(
            camera_config, text="视频地址", style="PanelMuted.TLabel", width=7
        ).pack(side="left")
        ttk.Entry(
            camera_config, textvariable=self.camera_source_var, width=40
        ).pack(side="left", fill="x", expand=True, padx=(0, 7))
        ttk.Button(
            camera_config, text="重新连接", command=self.reconnect_camera
        ).pack(side="left")

        ha_config = ttk.Frame(self.settings_panel, style="Panel.TFrame")
        ha_config.pack(fill="x")
        ttk.Label(ha_config, text="HA 地址", style="PanelMuted.TLabel", width=7).pack(side="left")
        ttk.Entry(ha_config, textvariable=self.ha_url_var, width=28).pack(side="left", padx=(0, 7))
        ttk.Label(ha_config, text="令牌", style="PanelMuted.TLabel").pack(side="left", padx=(0, 5))
        ttk.Entry(ha_config, textvariable=self.ha_token_var, width=22, show="*").pack(side="left")

        scene_config = ttk.Frame(self.settings_panel, style="Panel.TFrame")
        scene_config.pack(fill="x", pady=(5, 0))
        ttk.Label(scene_config, text="开灯场景", style="PanelMuted.TLabel", width=7).pack(side="left")
        ttk.Entry(scene_config, textvariable=self.ha_on_scene_var, width=22).pack(side="left", padx=(0, 7))
        ttk.Label(scene_config, text="关灯场景", style="PanelMuted.TLabel").pack(side="left", padx=(0, 5))
        ttk.Entry(scene_config, textvariable=self.ha_off_scene_var, width=22).pack(side="left")

        mood_config = ttk.Frame(self.settings_panel, style="Panel.TFrame")
        mood_config.pack(fill="x", pady=(5, 0))
        ttk.Label(mood_config, text="正常场景", style="PanelMuted.TLabel", width=7).pack(side="left")
        ttk.Entry(mood_config, textvariable=self.ha_normal_scene_var, width=22).pack(side="left", padx=(0, 7))
        ttk.Label(mood_config, text="紫色场景", style="PanelMuted.TLabel").pack(side="left", padx=(0, 5))
        ttk.Entry(mood_config, textvariable=self.ha_purple_scene_var, width=22).pack(side="left")

        connection_tools = ttk.Frame(self.settings_panel, style="Panel.TFrame")
        connection_tools.pack(fill="x", pady=(6, 0))
        ttk.Label(connection_tools, text="灯光控制", style="PanelMuted.TLabel").pack(side="left", padx=(0, 5))
        ttk.Button(
            connection_tools,
            text="开灯",
            command=lambda: self._send_ha_light("71"),
        ).pack(side="left", padx=2)
        ttk.Button(
            connection_tools,
            text="关灯",
            command=lambda: self._send_ha_light("72"),
        ).pack(side="left", padx=2)
        ttk.Button(
            connection_tools,
            text="正常",
            command=lambda: self._send_ha_scene_preset("normal"),
        ).pack(side="left", padx=2)
        ttk.Button(
            connection_tools,
            text="紫色心情",
            command=lambda: self._send_ha_scene_preset("purple"),
        ).pack(side="left", padx=2)
        ttk.Label(
            connection_tools,
            text="scene.turn_on",
            style="PanelMuted.TLabel",
        ).pack(side="left", padx=(8, 0))
        ttk.Button(connection_tools, text="刷新串口", command=self.refresh_ports).pack(side="right")

        status = ttk.Frame(body_host, style="Panel.TFrame", padding=(26, 9))
        status.pack(fill="x", padx=18)
        self.dot = tk.Canvas(status, width=12, height=12, bg=COLORS["panel"], highlightthickness=0)
        self.dot.pack(side="left")
        self.dot_id = self.dot.create_oval(2, 2, 10, 10, fill=COLORS["red"], outline="")
        ttk.Label(status, textvariable=self.connection_text, style="Panel.TLabel").pack(side="left", padx=(6, 18))
        ttk.Label(status, textvariable=self.status_text, style="PanelMuted.TLabel").pack(side="left")
        self.light_dot = tk.Canvas(status, width=12, height=12, bg=COLORS["panel"], highlightthickness=0)
        self.light_dot.pack(side="left", padx=(20, 0))
        self.light_dot_id = self.light_dot.create_oval(2, 2, 10, 10, fill=COLORS["red"], outline="")
        ttk.Label(status, textvariable=self.light_connection_text, style="Panel.TLabel").pack(side="left", padx=(6, 10))
        ttk.Label(status, textvariable=self.light_status_text, style="PanelMuted.TLabel").pack(side="left")
        ttk.Label(status, text="联动：机器人 71~76 → Home Assistant", style="PanelMuted.TLabel").pack(side="right")

        self.page_host = ttk.Frame(body_host, padding=(18, 14, 18, 8))
        self.page_host.pack(fill="both", expand=True)

        nav = ttk.Frame(body_host, style="Panel.TFrame", padding=(18, 8))
        nav.pack(fill="x", padx=18, pady=(0, 14))
        self.nav_buttons = {}
        for key, text in (("control", "▣  遥控"), ("terminal", "〉_  终端"), ("commands", "▤  指令")):
            button = ttk.Button(nav, text=text, style="Nav.TButton", command=lambda page=key: self.show_page(page))
            button.pack(side="left", fill="x", expand=True, padx=4)
            self.nav_buttons[key] = button

        self.after_idle(self._render_camera)
        self.after_idle(self._ensure_settings_collapsed)
        self.after_idle(self._set_initial_pane_sizes)

    def _set_initial_pane_sizes(self) -> None:
        self.update_idletasks()
        window_width = max(1060, self.winfo_width())
        window_height = max(720, self.winfo_height())
        self.horizontal_pane.sash_place(0, int(window_width * 0.53), 0)
        self.vertical_pane.sash_place(0, 0, int(window_height * 0.38))

    def _ensure_settings_collapsed(self) -> None:
        self.settings_visible = False
        self.settings_panel.pack_forget()
        if not self.connection_overview.winfo_manager():
            self.connection_overview.pack(
                fill="both", expand=True, pady=(18, 8)
            )
        self.settings_button.configure(text="⚙  设置")

    def toggle_settings(self) -> None:
        self.settings_visible = not self.settings_visible
        if self.settings_visible:
            self.connection_overview.pack_forget()
            self.settings_panel.pack(fill="x", pady=(8, 0))
            self.settings_button.configure(text="✕  收起设置")
        else:
            self.settings_panel.pack_forget()
            self.connection_overview.pack(
                fill="both", expand=True, pady=(18, 8)
            )
            self.settings_button.configure(text="⚙  设置")

    def reconnect_camera(self) -> None:
        self.camera_status_text.set("正在重新连接…")
        self.camera_dot.itemconfigure(self.camera_dot_id, fill=COLORS["orange"])
        self.camera_client.start(self.camera_source_var.get())

    def _camera_resized(self, _event=None) -> None:
        self.after_idle(self._render_camera)

    def _render_camera(self) -> None:
        if not hasattr(self, "camera_canvas"):
            return
        width = max(2, self.camera_canvas.winfo_width())
        height = max(2, self.camera_canvas.winfo_height())
        source = self.camera_image or self.camera_default_image
        fitted = ImageOps.fit(
            source, (width, height), method=Image.Resampling.LANCZOS,
            centering=(0.5, 0.5),
        )
        self.camera_photo = ImageTk.PhotoImage(fitted)
        self.camera_canvas.delete("all")
        self.camera_canvas.create_image(
            width // 2, height // 2, image=self.camera_photo, anchor="center"
        )
        if self.camera_image is None:
            self.camera_canvas.create_rectangle(
                12, height - 43, 210, height - 12,
                fill="#101522", outline="", stipple="gray50",
            )
            self.camera_canvas.create_text(
                24, height - 27, text="等待摄像头画面",
                fill="#eef1ff", anchor="w",
                font=("Microsoft YaHei UI", 10, "bold"),
            )

    def _build_pages(self) -> None:
        self.pages = {
            "control": ttk.Frame(self.page_host),
            "terminal": ttk.Frame(self.page_host),
            "commands": ttk.Frame(self.page_host),
        }
        for page in self.pages.values():
            page.place(relx=0, rely=0, relwidth=1, relheight=1)
        self._build_control()
        self._build_terminal()
        self._build_commands()

    def _card(self, parent, padding=18) -> ttk.Frame:
        return ttk.Frame(parent, style="Panel.TFrame", padding=padding)

    def _build_control(self) -> None:
        page = self.pages["control"]
        page.columnconfigure(0, weight=3)
        page.columnconfigure(1, weight=2)
        page.rowconfigure(0, weight=1)

        control_card = self._card(page, 18)
        control_card.grid(row=0, column=0, sticky="nsew", padx=(6, 6))
        summary = ttk.Frame(control_card, style="Panel.TFrame")
        summary.pack(fill="x")
        for label, variable in (
            ("发送格式", tk.StringVar(value="两位 ASCII 十六进制")),
            ("串口参数", tk.StringVar(value="9600 8N1")),
            ("最近指令", self.recent_action),
            ("机器人回复", self.recent_reply),
        ):
            block = ttk.Frame(summary, style="Panel.TFrame")
            block.pack(side="left", fill="x", expand=True)
            ttk.Label(block, text=label, style="PanelMuted.TLabel").pack()
            ttk.Label(block, textvariable=variable, style="Panel.TLabel", font=("Segoe UI", 12, "bold")).pack()

        self.joystick = RobotJoystick(control_card, self._joystick_changed, height=420)
        self.joystick.pack(fill="both", expand=True, pady=(6, 0))
        self.vector_text = tk.StringVar(value="Vx: 0.00    Vy: 0.00    指令: 待命")
        ttk.Label(control_card, textvariable=self.vector_text, style="PanelMuted.TLabel", font=("Consolas", 10)).pack(pady=(0, 10))

        gait_row = ttk.Frame(control_card, style="Panel.TFrame")
        gait_row.pack(fill="x")
        for action_name in ("站立 Stand", "慢走 Walk", "小跑 Trot", "跳跃 Jump"):
            ttk.Button(gait_row, text=action_name, command=lambda name=action_name: self.execute_action(name)).pack(side="left", fill="x", expand=True, padx=3)
        ttk.Button(control_card, text="ⓘ  回中站立", style="Danger.TButton", command=self.emergency_stop).pack(fill="x", pady=(14, 0))

        quick = self._card(page, 18)
        quick.grid(row=0, column=1, sticky="nsew", padx=(6, 6))
        ttk.Label(quick, text="常用动作", style="CardTitle.TLabel").pack(anchor="w")
        ttk.Label(quick, text="点击后立即通过蓝牙发送对应指令", style="PanelMuted.TLabel").pack(anchor="w", pady=(3, 12))
        grid = ttk.Frame(quick, style="Panel.TFrame")
        grid.pack(fill="both", expand=True)
        for column in range(2):
            grid.columnconfigure(column, weight=1)
        quick_actions = [
            "左转", "右转", "小跑跳跃", "扭头看左", "扭头看右", "点头", "摇头", "歪头",
            "智能灯1开启", "智能灯1关闭", "智能灯2开启", "智能灯2关闭",
            "智能灯3开启", "智能灯3关闭",
        ]
        for index, action_name in enumerate(quick_actions):
            command = COMMAND_BY_NAME[action_name]
            button = tk.Button(
                grid,
                text=f"{command.icon}  {command.name}\nCMD {command.code}",
                command=lambda name=action_name: self.execute_action(name),
                bg=COLORS["panel_soft"],
                fg=COLORS["text"],
                activebackground=COLORS["accent_soft"],
                relief="solid",
                bd=1,
                highlightthickness=0,
                font=("Microsoft YaHei UI", 10),
                padx=8,
                pady=10,
                cursor="hand2",
            )
            button.grid(row=index // 2, column=index % 2, sticky="nsew", padx=4, pady=4)

    def _build_terminal(self) -> None:
        page = self.pages["terminal"]
        log_card = self._card(page, 16)
        log_card.pack(fill="both", expand=True, padx=6)
        heading = ttk.Frame(log_card, style="Panel.TFrame")
        heading.pack(fill="x", pady=(0, 10))
        ttk.Label(heading, text="通信日志", style="CardTitle.TLabel").pack(side="left")
        self.counter_text = tk.StringVar(value="机器人 TX 0 / RX 0  ·  HA 请求 0")
        ttk.Label(heading, textvariable=self.counter_text, style="PanelMuted.TLabel").pack(side="right")
        self.log = tk.Text(
            log_card,
            bg=COLORS["terminal"],
            fg="#e9e3db",
            insertbackground="white",
            selectbackground=COLORS["accent_dark"],
            relief="flat",
            font=("Consolas", 10),
            wrap="word",
            state="disabled",
        )
        self.log.pack(fill="both", expand=True)

        quick = ttk.Frame(log_card, style="Panel.TFrame")
        quick.pack(fill="x", pady=(10, 6))
        ttk.Label(quick, text="快捷指令", style="PanelMuted.TLabel").pack(side="left", padx=(0, 8))
        for action_name in ("整机回中站立", "前进", "智能灯1开启", "智能灯1关闭", "智能灯2开启", "智能灯2关闭"):
            ttk.Button(quick, text=action_name, command=lambda name=action_name: self.execute_action(name)).pack(side="left", padx=3)
        ttk.Button(quick, text="清空", command=self._clear_log).pack(side="right")

        send_row = ttk.Frame(log_card, style="Panel.TFrame")
        send_row.pack(fill="x", pady=(4, 0))
        ttk.Label(send_row, text="两位十六进制", style="PanelMuted.TLabel").pack(side="left", padx=(0, 7))
        self.manual_text = tk.StringVar()
        entry = ttk.Entry(send_row, textvariable=self.manual_text)
        entry.pack(side="left", fill="x", expand=True, padx=(0, 7))
        entry.bind("<Return>", lambda _e: self._send_manual())
        ttk.Button(send_row, text="发送", style="Accent.TButton", command=self._send_manual).pack(side="left")

    def _build_commands(self) -> None:
        page = self.pages["commands"]
        top = ttk.Frame(page)
        top.pack(fill="x", padx=6, pady=(0, 10))
        detail = self._card(top, 14)
        detail.pack(side="left", fill="x", expand=True)
        self.command_icon = tk.StringVar(value="↑")
        self.command_name = tk.StringVar(value="前进")
        self.command_detail = tk.StringVar(value="CMD 02 · 机器人向前移动")
        self.command_reply = tk.StringVar(value="机器人执行动作")
        ttk.Label(detail, textvariable=self.command_icon, style="Panel.TLabel", font=("Segoe UI Emoji", 24)).pack(side="left", padx=(0, 14))
        text = ttk.Frame(detail, style="Panel.TFrame")
        text.pack(side="left", fill="x", expand=True)
        ttk.Label(text, textvariable=self.command_name, style="CardTitle.TLabel").pack(anchor="w")
        ttk.Label(text, textvariable=self.command_detail, style="PanelMuted.TLabel").pack(anchor="w")
        ttk.Label(text, textvariable=self.command_reply, style="PanelMuted.TLabel").pack(anchor="w", pady=(3, 0))
        self.execute_selected_button = ttk.Button(detail, text="执行", style="Accent.TButton", command=self._execute_selected)
        self.execute_selected_button.pack(side="right")

        protocol = self._card(top, 14)
        protocol.pack(side="right", padx=(10, 0))
        ttk.Label(protocol, text="发送格式", style="PanelMuted.TLabel").pack(anchor="w")
        ttk.Label(protocol, text="两位 ASCII 文本", style="Panel.TLabel", font=("Microsoft YaHei UI", 11, "bold")).pack(pady=(5, 0))

        filter_row = ttk.Frame(page)
        filter_row.pack(fill="x", padx=6, pady=(0, 8))
        self.category_var = tk.StringVar(value="全部")
        self.category_buttons = {}
        for category in ("全部", "基础", "移动", "步态", "头部", "尾巴", "嘴巴", "参数", "联动"):
            button = ttk.Button(filter_row, text=category, style="Chip.TButton", command=lambda c=category: self.filter_commands(c))
            button.pack(side="left", padx=(0, 6))
            self.category_buttons[category] = button

        self.command_scroll = ScrollFrame(page)
        self.command_scroll.pack(fill="both", expand=True, padx=6)
        self.command_grid = self.command_scroll.content
        self.command_buttons: list[tk.Button] = []
        self.selected_name = "前进"
        self.filter_commands("全部")

    def show_page(self, key: str) -> None:
        titles = {
            "control": ("机器人遥控", "两位 ASCII 十六进制指令 · 蓝牙串口控制"),
            "terminal": ("蓝牙终端", "SPP 串口通信与调试"),
            "commands": ("指令中心", "两位 ASCII 十六进制指令 · 动作库"),
        }
        self.page_title.set(titles[key][0])
        self.page_subtitle.set(titles[key][1])
        self.pages[key].tkraise()
        for name, button in self.nav_buttons.items():
            button.configure(style="NavActive.TButton" if name == key else "Nav.TButton")

    def refresh_ports(self) -> None:
        selected_robot = self._selected_port_name(self.port_var.get())
        all_ports = list(list_ports.comports())
        ports = [item for item in all_ports if not _is_bluetooth_incoming_port(item)]
        hidden_count = len(all_ports) - len(ports)
        values = [_port_display_name(item) for item in ports]
        self.port_combo["values"] = values

        robot_value = self._matching_port_value(values, selected_robot)
        if not robot_value:
            robot_value = self._hinted_port_value(values, ROBOT_PORT_HINTS)
        if not robot_value and values:
            robot_value = values[0]
        self.port_var.set(robot_value)

        found_text = f"发现 {len(values)} 个可连接串口"
        if hidden_count:
            found_text += f"，已隐藏 {hidden_count} 个蓝牙传入端口"
        if not self.link.connected:
            self.status_text.set(found_text)

    @staticmethod
    def _selected_port_name(selection: str) -> str:
        return selection.split(" — ", 1)[0] if selection else ""

    @staticmethod
    def _matching_port_value(values: list[str], port_name: str) -> str:
        return next(
            (value for value in values if value.split(" — ", 1)[0] == port_name),
            "",
        )

    @staticmethod
    def _hinted_port_value(values: list[str], hints: tuple[str, ...], excluded: str = "") -> str:
        for value in values:
            if value == excluded:
                continue
            lowered = value.casefold()
            if any(hint.casefold() in lowered for hint in hints):
                return value
        return ""

    def toggle_connection(self) -> None:
        if self.link.connected:
            self.emergency_stop(quiet=True)
            self.link.disconnect()
            self._set_connected(False)
            return
        if not self.port_var.get():
            messagebox.showinfo("未发现串口", "请先在 Windows 蓝牙设置中配对机器人，再点击刷新。")
            return
        try:
            port_name = self._selected_port_name(self.port_var.get())
            self.link.connect(port_name, int(self.baud_var.get()))
            self.status_text.set(f"正在连接 {port_name}…")
        except (ValueError, serial.SerialException, OSError) as exc:
            messagebox.showerror("连接失败", str(exc))

    def _set_connected(self, connected: bool, port_name: str = "") -> None:
        if connected:
            self.connection_text.set("已连接")
            self.status_text.set(f"{port_name} · {self.baud_var.get()} bps")
            self.dot.itemconfigure(self.dot_id, fill=COLORS["green"])
            self.connect_button.configure(text="断开", style="Danger.TButton")
        else:
            self.connection_text.set("未连接")
            self.status_text.set("等待连接蓝牙串口")
            self.dot.itemconfigure(self.dot_id, fill=COLORS["red"])
            self.connect_button.configure(text="连接", style="Accent.TButton")

    @staticmethod
    def _encode_command(code: str) -> bytes:
        return code.upper().encode("ascii")

    def execute_action(self, action_name: str, quiet: bool = False) -> bool:
        command = COMMAND_BY_NAME.get(action_name)
        if not command:
            return False
        if command.code in HA_LIGHT_STATE_BY_COMMAND:
            self.recent_action.set(f"{command.name} {command.code}")
            self.recent_reply.set("等待 Home Assistant")
            return self._send_ha_light(command.code, quiet=quiet, source="App")
        try:
            self.link.send(self._encode_command(command.code))
        except (serial.SerialException, OSError) as exc:
            if not quiet:
                messagebox.showwarning("无法发送", str(exc))
            return False
        self.active_command = command
        self.recent_action.set(f"{command.icon}  {command.name}")
        self.recent_reply.set(command.reply or command.action)
        self.recent_action.set(f"{command.name} {command.code}")
        self.recent_reply.set("等待 OK")
        self.status_text.set(f"已发送 CMD {command.code} · {command.name} · 等待 OK")
        return True

    def emergency_stop(self, quiet: bool = False) -> None:
        if self.execute_action(EMERGENCY_ACTION, quiet=quiet):
            command = COMMAND_BY_NAME[EMERGENCY_ACTION]
            self.status_text.set(f"回中站立指令已发送 · CMD {command.code} · 等待 OK")

    @staticmethod
    def _direction_action(x: float, y: float) -> str:
        if abs(x) < .24 and abs(y) < .24:
            return EMERGENCY_ACTION
        if abs(y) >= abs(x):
            return "前进" if y > 0 else "后退"
        return "右移" if x > 0 else "左移"

    def _joystick_changed(self, x: float, y: float, released: bool) -> None:
        action_name = EMERGENCY_ACTION if released else self._direction_action(x, y)
        command = COMMAND_BY_NAME[action_name]
        self.vector_text.set(f"Vx: {x:+.2f}    Vy: {y:+.2f}    指令: {command.name} ({command.code})")
        now = time.monotonic()
        if action_name != self.last_direction or now - self.last_direction_time > .35:
            self.last_direction = action_name
            self.last_direction_time = now
            self.execute_action(action_name, quiet=True)

    def filter_commands(self, category: str) -> None:
        self.category_var.set(category)
        for name, button in self.category_buttons.items():
            button.configure(style="ChipActive.TButton" if name == category else "Chip.TButton")
        for widget in self.command_grid.winfo_children():
            widget.destroy()
        for column in range(5):
            self.command_grid.columnconfigure(column, weight=1)
        filtered = [item for item in COMMANDS if category == "全部" or item.category == category]
        for index, command in enumerate(filtered):
            selected = command.name == self.selected_name
            button = tk.Button(
                self.command_grid,
                text=f"{command.icon}  {command.code}\n{command.name}",
                command=lambda name=command.name: self.select_command(name),
                bg=COLORS["accent_soft"] if selected else COLORS["panel"],
                fg=COLORS["red"] if command.name == EMERGENCY_ACTION else COLORS["text"],
                activebackground=COLORS["accent_soft"],
                relief="solid",
                bd=1,
                highlightthickness=1 if selected else 0,
                highlightbackground=COLORS["accent"],
                font=("Microsoft YaHei UI", 10),
                padx=8,
                pady=12,
                cursor="hand2",
            )
            button.grid(row=index // 5, column=index % 5, sticky="nsew", padx=5, pady=5)

    def select_command(self, action_name: str) -> None:
        self.selected_name = action_name
        command = COMMAND_BY_NAME[action_name]
        self.command_icon.set(command.icon)
        self.command_name.set(command.name)
        self.command_detail.set(f"CMD {command.code} · {command.action}")
        self.command_reply.set(command.reply or "机器人执行动作")
        self.filter_commands(self.category_var.get())

    def _execute_selected(self) -> None:
        self.execute_action(self.selected_name)

    def _send_manual(self) -> None:
        text = self.manual_text.get().strip().upper()
        if not text:
            return
        if text not in VALID_COMMAND_CODES:
            messagebox.showerror("指令无效", "请输入动作表中的两位 ASCII 十六进制指令。")
            return
        if text in HA_LIGHT_STATE_BY_COMMAND:
            self._send_ha_light(text)
            self.manual_text.set("")
            return
        try:
            self.link.send(text.encode("ascii"))
            self.manual_text.set("")
        except (serial.SerialException, OSError) as exc:
            messagebox.showwarning("无法发送", str(exc))

    def _poll_events(self) -> None:
        try:
            while True:
                channel, event, payload = self.events.get_nowait()
                if channel == "camera":
                    if event == "connecting":
                        self.camera_status_text.set("正在连接…")
                        self.camera_dot.itemconfigure(
                            self.camera_dot_id, fill=COLORS["orange"]
                        )
                    elif event == "connected":
                        self.camera_status_text.set("视频已连接")
                        self.camera_dot.itemconfigure(
                            self.camera_dot_id, fill=COLORS["green"]
                        )
                    elif event == "error":
                        self.camera_status_text.set("无画面 · 自动重连")
                        self.camera_dot.itemconfigure(
                            self.camera_dot_id, fill=COLORS["red"]
                        )
                        self.camera_image = None
                        self._render_camera()
                        self._write_log("摄像头", str(payload))
                elif channel == "robot":
                    if event == "connected":
                        self._set_connected(True, payload)
                    elif event == "disconnected":
                        self._set_connected(False)
                    elif event == "error":
                        self._write_log("机器人 ERR", str(payload))
                        messagebox.showerror("机器人蓝牙连接中断", str(payload))
                    elif event == "tx":
                        self.tx_count += len(payload)
                        self._write_log("机器人 TX", self._display_bytes(payload))
                    elif event == "rx":
                        self.rx_count += len(payload)
                        self._write_log("机器人 RX", self._display_bytes(payload))
                        self._consume_ack(payload)
                        self._consume_home_commands(payload)
                elif channel == "ha":
                    self._handle_ha_result(event, payload)
                elif channel == "api":
                    if event == "command":
                        self._execute_command_code(payload)
                    elif event == "light":
                        self._send_ha_light(payload, quiet=True, source="本地 API")
                    elif event == "scene":
                        self._send_ha_scene_preset(payload, quiet=True)
                    elif event == "voice":
                        action_type, action = payload
                        if action_type == "light":
                            self._send_ha_light(action, quiet=True, source="语音模型")
                        elif action_type == "preset":
                            self._send_ha_scene_preset(action, quiet=True)
                self.counter_text.set(
                    f"机器人 TX {self.tx_count} / RX {self.rx_count}  ·  "
                    f"HA 请求 {self.ha_request_count}"
                )
        except queue.Empty:
            pass
        try:
            width, height, pixels, _sequence = self.camera_frames.get_nowait()
            self.camera_image = Image.frombytes(
                "RGB", (width, height), pixels, "raw", "BGR;16"
            )
            self.camera_status_text.set("实时画面")
            self.camera_dot.itemconfigure(
                self.camera_dot_id, fill=COLORS["green"]
            )
            self._render_camera()
        except queue.Empty:
            pass
        except (ValueError, OSError) as exc:
            self.camera_status_text.set("画面解码失败")
            self._write_log("摄像头 ERR", str(exc))
        self.after(60, self._poll_events)

    def _execute_command_code(self, code: str) -> bool:
        command = next(
            (item for item in COMMANDS if item.code == code.upper()),
            None,
        )
        if not command:
            return False
        return self.execute_action(command.name, quiet=True)

    def _consume_home_commands(self, payload: bytes) -> None:
        self.home_command_buffer += payload
        while self.home_command_buffer:
            positions = [
                (index, command)
                for command in HOME_COMMANDS
                if (index := self.home_command_buffer.find(command)) >= 0
            ]
            if not positions:
                self.home_command_buffer = (
                    self.home_command_buffer[-1:]
                    if self.home_command_buffer.endswith(b"7")
                    else b""
                )
                return
            index, command = min(positions, key=lambda item: item[0])
            self.home_command_buffer = self.home_command_buffer[index + 2:]
            self._forward_home_command(command)

    def _forward_home_command(self, command: bytes) -> None:
        self._send_ha_light(command.decode(), quiet=True, source="机器人回传")

    def _send_ha_light(self, command: str, quiet: bool = False, source: str = "App") -> bool:
        turn_on = HA_LIGHT_STATE_BY_COMMAND.get(command)
        if turn_on is None:
            return False

        scene_id = (
            self.ha_on_scene_var.get().strip()
            if turn_on
            else self.ha_off_scene_var.get().strip()
        )
        action = "开灯" if turn_on else "关灯"
        return self._send_ha_scene_request(
            scene_id,
            action,
            command,
            quiet,
            source,
        )

    def _send_ha_scene_preset(self, preset: str, quiet: bool = False) -> bool:
        presets = {
            "normal": (self.ha_normal_scene_var.get().strip(), "正常"),
            "purple": (self.ha_purple_scene_var.get().strip(), "紫色心情"),
        }
        scene_data = presets.get(preset)
        if not scene_data:
            return False
        scene_id, action = scene_data
        return self._send_ha_scene_request(
            scene_id,
            action,
            f"preset:{preset}",
            quiet,
            "App",
        )

    def _send_ha_scene_request(
        self,
        scene_id: str,
        action: str,
        command: str,
        quiet: bool,
        source: str,
    ) -> bool:
        base_url = self.ha_url_var.get().strip().rstrip("/")
        token = self.ha_token_var.get().strip()
        if not base_url or not token or not scene_id:
            error = "请填写 Home Assistant 地址、长期访问令牌和对应场景 ID。"
            self.light_connection_text.set("HA 配置不完整")
            self.light_status_text.set(error)
            self.light_dot.itemconfigure(self.light_dot_id, fill=COLORS["red"])
            self._write_log("HA ERR", error)
            if not quiet:
                messagebox.showwarning("Home Assistant 配置", error)
            return False

        request = Request(
            f"{base_url}/api/services/scene/turn_on",
            data=json.dumps({"entity_id": scene_id}).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {token}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        context = {
            "command": command,
            "action": action,
            "scene_id": scene_id,
            "source": source,
            "notify": not quiet,
        }
        self.light_status_text.set(f"正在通过 Home Assistant {action}…")
        self._write_log("HA", f"{source} 请求{action} · {scene_id}")

        def worker() -> None:
            try:
                with urlopen(request, timeout=HA_REQUEST_TIMEOUT) as response:
                    response.read()
                self.events.put(("ha", "success", context))
            except HTTPError as exc:
                detail = exc.read().decode("utf-8", errors="replace").strip()
                reason = f"HTTP {exc.code} {exc.reason}"
                if detail:
                    reason = f"{reason}: {detail}"
                self.events.put(("ha", "error", (context, reason)))
            except (URLError, TimeoutError, OSError) as exc:
                reason = getattr(exc, "reason", exc)
                self.events.put(("ha", "error", (context, str(reason))))
            except Exception as exc:
                self.events.put(("ha", "error", (context, str(exc))))

        threading.Thread(
            target=worker,
            name="HomeAssistantSceneRequest",
            daemon=True,
        ).start()
        return True

    def _handle_ha_result(self, event: str, payload) -> None:
        self.ha_request_count += 1
        if event == "success":
            context = payload
            self.ha_available = True
            self.light_connection_text.set("HA 可用")
            self.light_status_text.set(f"{context['action']}成功 · {context['scene_id']}")
            self.light_dot.itemconfigure(self.light_dot_id, fill=COLORS["green"])
            self._write_log("HA OK", f"{context['source']} · {context['action']}成功")
            if context["source"] == "机器人回传":
                self.status_text.set(
                    f"机器人回传 {context['command']} · 已通过 Home Assistant {context['action']}"
                )
            self.recent_reply.set("Home Assistant 已执行")
            return

        context, error = payload
        self.ha_available = False
        self.light_connection_text.set("HA 连接失败")
        self.light_status_text.set(f"{context['action']}失败：{error}")
        self.light_dot.itemconfigure(self.light_dot_id, fill=COLORS["red"])
        self._write_log("HA ERR", f"{context['source']} · {error}")
        self.recent_reply.set("Home Assistant 请求失败")
        if context["notify"]:
            messagebox.showerror(
                "Home Assistant 连接失败",
                f"无法完成{context['action']}。\n\n{error}",
            )

    def _consume_ack(self, payload: bytes) -> None:
        self.ack_buffer = (self.ack_buffer + payload.upper())[-32:]
        if b"ERROR" in self.ack_buffer:
            self.status_text.set("STM32 返回 ERROR · 指令不在当前动作表中")
            self.ack_buffer = b""
        elif b"OK" in self.ack_buffer:
            if self.active_command:
                self.status_text.set(f"STM32 已确认 OK · CMD {self.active_command.code} · {self.active_command.name}")
            else:
                self.status_text.set("STM32 已确认 OK")
            self.recent_reply.set("OK")
            self.ack_buffer = b""

    @staticmethod
    def _display_bytes(payload: bytes) -> str:
        printable = payload.decode("utf-8", errors="replace").strip()
        hex_text = payload.hex(" ").upper()
        if printable and all(char.isprintable() for char in printable):
            return f"{hex_text}    {printable}"
        return hex_text

    def _write_log(self, prefix: str, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", f"[{time.strftime('%H:%M:%S')}] {prefix}  {text}\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _close(self) -> None:
        self.camera_client.stop()
        if self.link.connected:
            self.emergency_stop(quiet=True)
        if self.api_server:
            self.api_server.shutdown()
            self.api_server.server_close()
        self.link.disconnect()
        self.destroy()


def main() -> None:
    RobotDogApp().mainloop()


if __name__ == "__main__":
    main()
