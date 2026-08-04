import atexit
import json
import math
import os
import re
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import threading
import time
import uuid
from pathlib import Path
from urllib import error as uerr
from urllib import request as ureq
from urllib.parse import urljoin

from flask import Flask, abort, jsonify, render_template, request, send_file

try:
    from gpiozero import PWMOutputDevice
except Exception as exc:
    PWMOutputDevice = None
    GPIOZERO_IMPORT_ERROR = str(exc)
else:
    GPIOZERO_IMPORT_ERROR = ""

# OpenCV 用于异物识别画框
try:
    import cv2
    import numpy as np
except Exception as e:
    cv2 = None
    np = None
    CV_IMPORT_ERROR = str(e)
else:
    CV_IMPORT_ERROR = ""

# YOLOv8 用于孔洞识别
try:
    from ultralytics import YOLO
except Exception as e:
    YOLO = None
    YOLO_IMPORT_ERROR = str(e)
else:
    YOLO_IMPORT_ERROR = ""


try:
    import psutil
except Exception:
    psutil = None

# =========================
# ROS2 机器人控制（方案 A）
# =========================
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32, String
except Exception:
    rclpy = None
    Node = None
    Twist = None
    Int32 = None
    String = None


# =========================
# 基本路径
# =========================
BASE_DIR = Path(__file__).resolve().parent
TEMPLATE_DIR = BASE_DIR / "templates"
RUNTIME_DIR = BASE_DIR / "runtime"
RUNTIME_DIR.mkdir(parents=True, exist_ok=True)

CAPTURE_DIR = RUNTIME_DIR / "captures"
CAPTURE_DIR.mkdir(parents=True, exist_ok=True)

MEDIAMTX_CONFIG_PATH = RUNTIME_DIR / "mediamtx.yml"
MEDIAMTX_LOG_PATH = RUNTIME_DIR / "mediamtx.log"
PUBLISHER_LOG_PATH = RUNTIME_DIR / "publisher.log"
RECORDING_LOG_PATH = RUNTIME_DIR / "recording.log"
BASECONTROLLER_LOG_PATH = RUNTIME_DIR / "basecontroller.log"

app = Flask(__name__, template_folder=str(TEMPLATE_DIR))


# =========================
# 网络/服务配置
# =========================
HOST = "0.0.0.0"
PORT = 5000

WEBRTC_HTTP_PORT = 8889
WEBRTC_ICE_UDP_PORT = 8189
WEBRTC_ICE_TCP_PORT = 8190

# WebRTC 地址发现与网络切换监控。
# 不要把某个 DHCP 地址（例如 192.168.137.177）写死在这里。
# 程序会自动收集树莓派当前可用网卡的 IPv4 地址，并写入 MediaMTX 配置。
WEBRTC_NETWORK_WATCH_INTERVAL_SECONDS = max(
    3, int(os.environ.get("WEBRTC_NETWORK_WATCH_INTERVAL_SECONDS", "5"))
)

# 可选：需要额外公布的 IP 或域名，多个值用逗号、分号或空格隔开。
# 示例：export WEBRTC_ADDITIONAL_HOSTS="192.168.137.177,raspberrypi.local"
WEBRTC_ADDITIONAL_HOSTS_ENV = "WEBRTC_ADDITIONAL_HOSTS"

MEDIAMTX_BIN_ENV = "MEDIAMTX_BIN"
# 正常使用保持 info；排障时可设置环境变量：
# export MEDIAMTX_LOG_LEVEL=debug
MEDIAMTX_LOG_LEVEL = os.environ.get("MEDIAMTX_LOG_LEVEL", "info").strip().lower()
if MEDIAMTX_LOG_LEVEL not in ("error", "warn", "info", "debug"):
    MEDIAMTX_LOG_LEVEL = "info"

FFMPEG_BIN_ENV = "FFMPEG_BIN"


# =========================
# G1 云台 TCP / RTSP 配置
# =========================
# 当前 G1 已在 iOS 热点下设置为固定 IP：
#   G1             : 172.20.10.8
#   Raspberry Pi   : 172.20.10.10
#   iPhone gateway : 172.20.10.1
#
# 已验证：
#   TCP 控制端口   : 8888
#   RTSP 端口      : 554
#   RTSP 路径      : /H264
#
# 如以后修改网络，仍可通过环境变量覆盖：
#   export G1_CAMERA_IP=172.20.10.8
#   export G1_CONTROL_PORT=8888
#   export G1_RTSP_PORT=554
#   export G1_RTSP_PATH=/H264
G1_CAMERA_IP = os.environ.get("G1_CAMERA_IP", "172.20.10.8").strip()

try:
    G1_CONTROL_PORT = int(os.environ.get("G1_CONTROL_PORT", "8888"))
except Exception:
    G1_CONTROL_PORT = 8888

try:
    G1_RTSP_PORT = int(os.environ.get("G1_RTSP_PORT", "554"))
except Exception:
    G1_RTSP_PORT = 554

G1_RTSP_PATH = os.environ.get("G1_RTSP_PATH", "/H264").strip() or "/H264"
if not G1_RTSP_PATH.startswith("/"):
    G1_RTSP_PATH = "/" + G1_RTSP_PATH

G1_RTSP_URL = f"rtsp://{G1_CAMERA_IP}:{G1_RTSP_PORT}{G1_RTSP_PATH}"

try:
    G1_CONNECT_TIMEOUT = float(os.environ.get("G1_CONNECT_TIMEOUT", "2.0"))
except Exception:
    G1_CONNECT_TIMEOUT = 2.0

try:
    G1_RESPONSE_TIMEOUT = float(os.environ.get("G1_RESPONSE_TIMEOUT", "2.0"))
except Exception:
    G1_RESPONSE_TIMEOUT = 2.0

# 运动类命令默认不自动重发。
# 原因：如果设备其实已经执行了命令，只是响应丢失，再重发会造成重复动作。
try:
    G1_CONTROL_RETRIES = max(
        0,
        min(1, int(os.environ.get("G1_CONTROL_RETRIES", "0")))
    )
except Exception:
    G1_CONTROL_RETRIES = 0

# 查询类命令（姿态/模式）允许有限重试，不会直接产生运动。
try:
    G1_QUERY_RETRIES = max(
        0,
        min(2, int(os.environ.get("G1_QUERY_RETRIES", "1")))
    )
except Exception:
    G1_QUERY_RETRIES = 1

# 同一设备连续 TCP/协议事务之间留出恢复时间。
# 对当前 G1 这种“停止后再发下一条更稳定”的实机表现尤其重要。
try:
    G1_COMMAND_MIN_INTERVAL = max(
        0.04,
        min(0.50, float(os.environ.get("G1_COMMAND_MIN_INTERVAL", "0.10")))
    )
except Exception:
    G1_COMMAND_MIN_INTERVAL = 0.10

# 每个新动作前先发送零速中和帧，再等待一小段时间。
try:
    G1_NEUTRAL_SETTLE_SECONDS = max(
        0.04,
        min(0.50, float(os.environ.get("G1_NEUTRAL_SETTLE_SECONDS", "0.10")))
    )
except Exception:
    G1_NEUTRAL_SETTLE_SECONDS = 0.10

# 动作完成后的设备恢复窗口，避免下一次点击立刻压上来。
try:
    G1_POST_ACTION_SETTLE_SECONDS = max(
        0.05,
        min(0.60, float(os.environ.get("G1_POST_ACTION_SETTLE_SECONDS", "0.14")))
    )
except Exception:
    G1_POST_ACTION_SETTLE_SECONDS = 0.14

try:
    G1_PULSE_DURATION_MS = max(
        80,
        min(800, int(os.environ.get("G1_PULSE_DURATION_MS", "250")))
    )
except Exception:
    G1_PULSE_DURATION_MS = 250

try:
    G1_ROLL_SETTLE_SECONDS = max(
        0.10,
        min(1.2, float(os.environ.get("G1_ROLL_SETTLE_SECONDS", "0.30")))
    )
except Exception:
    G1_ROLL_SETTLE_SECONDS = 0.30

# 默认保持摄像头当前云台模式，避免首次点击因切换模式产生大幅跳动。
# 确实需要手动控制时强制锁定，可设置 G1_AUTO_LOCK_MODE=1。
G1_AUTO_LOCK_MODE = os.environ.get("G1_AUTO_LOCK_MODE", "0").strip().lower() not in {
    "0", "false", "no", "off"
}

# 航向/俯仰按住控制的安全超时。前端每秒续期；页面失联时会自动停止。
try:
    G1_MAX_JOG_SECONDS = max(1.5, float(os.environ.get("G1_MAX_JOG_SECONDS", "3.0")))
except Exception:
    G1_MAX_JOG_SECONDS = 3.0

# 若实机方向与界面相反，将对应环境变量设为 -1。
def _g1_direction_env(name, default=1):
    try:
        return -1 if int(os.environ.get(name, str(default))) < 0 else 1
    except Exception:
        return 1 if default >= 0 else -1


G1_YAW_DIRECTION = _g1_direction_env("G1_YAW_DIRECTION", 1)
G1_PITCH_DIRECTION = _g1_direction_env("G1_PITCH_DIRECTION", -1)
G1_ROLL_DIRECTION = _g1_direction_env("G1_ROLL_DIRECTION", 1)

G1_YAW_LIMIT = (-145.0, 145.0)
G1_ROLL_LIMIT = (-40.0, 40.0)
G1_PITCH_LIMIT = (-90.0, 90.0)
G1_POSE_CACHE_SECONDS = 1.0

# =========================
# 新相机栈（已验证可用）
# =========================
CAMERA_STACK_ROOT = Path("/opt/rpi-cam-stack")
CAMERA_BIN = CAMERA_STACK_ROOT / "bin" / "rpicam-vid"
CAMERA_LIB_DIR = CAMERA_STACK_ROOT / "lib" / "aarch64-linux-gnu"
CAMERA_IPA_MODULE_DIR = CAMERA_LIB_DIR / "libcamera" / "ipa"
CAMERA_IPA_PROXY_DIR = CAMERA_STACK_ROOT / "libexec" / "libcamera"


# =========================
# 风机 / 异物清理电调配置
# =========================

# GPIO13 = 物理 33 脚
CLEANING_ESC_PIN = 13

# 普通电调先用 50Hz，不要随便改 100/400Hz
CLEANING_ESC_FREQ = 50

# 电调标准脉宽：1000us 停止 / 最低油门，2000us 最高油门
CLEANING_MIN_US = 1000
CLEANING_MAX_US = 2000

# 点击“异物清理”后的目标油门
# 先用 60，不够再试 70
CLEANING_RUN_SPEED_PERCENT = 60

# 起转助推：防止低速启动一顿一顿
CLEANING_START_BOOST_PERCENT = 70
CLEANING_START_BOOST_TIME = 0.5

# 缓慢升降速，避免电流瞬间太大
CLEANING_RAMP_STEP = 5
CLEANING_RAMP_DELAY = 0.10



# =========================
# 视频配置
# =========================
RESOLUTION_OPTIONS = [
    (640, 360),
    (1280, 720),
    (1920, 1080),
]
FPS_OPTIONS = [15, 24, 30, 45, 60]

DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 15

BITRATE_MAP = {
    (640, 360): 1200000,
    (1280, 720): 3500000,
    (1920, 1080): 6000000,
}


# =========================
# 异物识别显示配置
# =========================
# 只做“画面识别框显示”，不在这里自动启动/停止电调风机；
# 异物清理风机仍然由前端“异物清理”按钮控制。

# 低延迟方案：视频仍然直接走 MediaMTX WebRTC，后端只做识别并输出坐标。
# 不再把识别后的画面重新 JPEG 编码成 MJPEG，因此画面显示延迟主要由 WebRTC 决定。

# 后台识别线程的目标处理帧率。YOLOv8 在树莓派 CPU 上较吃性能，建议 3~5。
DETECTION_WORKER_FPS = int(os.environ.get("DETECTION_WORKER_FPS", "4"))

# 识别时缩放到的最大宽度。优先降低延迟用 640；如果小碎屑漏检，可调到 800 或 960。
DETECTION_PROCESS_WIDTH = 640

# 小碎屑识别参数：主要检测“比局部背景更暗”的小目标，避免把亮反光误当异物。
DETECTION_MIN_AREA = 6
DETECTION_MAX_AREA = 1800
DETECTION_DARK_DIFF = 16
DETECTION_IGNORE_TOP_RATIO = 0.18
DETECTION_MAX_ASPECT_RATIO = 8.0
DETECTION_MIN_FILL_RATIO = 0.10

# 防抖：连续 2 帧确认；连续 10 帧没看到才认为消失。
DETECTION_CONFIRM_FRAMES = 2
DETECTION_LOST_FRAMES = 10

# 前端判断识别坐标是否过期的时间。
DETECTION_STALE_SECONDS = 3.0

# =========================
# YOLOv8 孔洞识别配置
# =========================
# 保留原来的 WebRTC 视频链路，后端只负责 YOLO 推理并输出坐标给前端 canvas 画框。
DETECTION_BACKEND = os.environ.get("DETECTION_BACKEND", "yolov8").strip().lower()

_yolo_path = os.environ.get("YOLO_MODEL_PATH", "best.pt").strip()
YOLO_MODEL_PATH = Path(_yolo_path).expanduser()
if not YOLO_MODEL_PATH.is_absolute():
    YOLO_MODEL_PATH = BASE_DIR / YOLO_MODEL_PATH

# 树莓派 CPU 推理建议先用 416；孔洞较小或漏检时可调到 640。
YOLO_IMGSZ = int(os.environ.get("YOLO_IMGSZ", "416"))
YOLO_CONF = float(os.environ.get("YOLO_CONF", "0.35"))
YOLO_IOU = float(os.environ.get("YOLO_IOU", "0.45"))
YOLO_DEVICE = os.environ.get("YOLO_DEVICE", "cpu").strip()

# 推理前把 RTSP 帧缩放到该宽度，降低延迟；孔洞很小可调大到 800/960。
YOLO_PROCESS_WIDTH = int(os.environ.get("YOLO_PROCESS_WIDTH", str(DETECTION_PROCESS_WIDTH)))

# 留空时使用 best.pt 内置类别名；想强制显示中文可设置 YOLO_FORCE_LABEL=孔洞。
YOLO_FORCE_LABEL = os.environ.get("YOLO_FORCE_LABEL", "").strip()

# CPU 线程数，过高可能导致树莓派卡顿；0 表示不主动设置。
YOLO_NUM_THREADS = int(os.environ.get("YOLO_NUM_THREADS", "2"))


# =========================
# YOLO 自动避障配置
# =========================
OBSTACLE_AVOIDANCE_DANGER_WIDTH_RATIO = 2.0 / 3.0
OBSTACLE_AVOIDANCE_DANGER_HEIGHT_RATIO = 1.0 / 8.0
OBSTACLE_AVOIDANCE_LOOP_INTERVAL_SECONDS = 0.08
OBSTACLE_AVOIDANCE_STALE_SECONDS = 1.0
OBSTACLE_AVOIDANCE_RELEASE_DELAY_SECONDS = 0.45
OBSTACLE_AVOIDANCE_MIN_HOLD_SECONDS = 0.35
OBSTACLE_AVOIDANCE_COMMAND_REFRESH_SECONDS = 0.24


# =========================
# 机器人运动控制配置
# =========================
BASECONTROLLER_CMD = ["ros2", "run", "basecontroller", "basecontroller"]

ROBOT_LINEAR_SPEED_MIN = 0.0
ROBOT_LINEAR_SPEED_MAX = 0.5
ROBOT_LINEAR_SPEED_DEFAULT = 0.0
ROBOT_ANGULAR_SPEED_DEFAULT = 0.5

ROBOT_FAN_MIN = 0
ROBOT_FAN_MAX = 180
ROBOT_FAN_STEP = 10

ROBOT_MODE_MANUAL = 0
ROBOT_MODE_AUTO = 1

ROBOT_MODE_NAME_MAP = {
    ROBOT_MODE_MANUAL: "manual",
    ROBOT_MODE_AUTO: "auto",
}

CONTROL_MODE_MANUAL = "manual"
CONTROL_MODE_AUTO = "auto"

AUTO_MOTION_STRAIGHT = "straight"
AUTO_MOTION_SPIRAL_LEFT = "spiral_left"
AUTO_MOTION_SPIRAL_RIGHT = "spiral_right"

_MODE_AUTO_BIT = 0x01
_TRAJECTORY_BITS = {
    AUTO_MOTION_STRAIGHT: 0x00,
    AUTO_MOTION_SPIRAL_LEFT: 0x02,
    AUTO_MOTION_SPIRAL_RIGHT: 0x04,
}
_CONTROLLER_KEY_VALUE_RE = re.compile(
    r"(?:^|[,;\s]+)([A-Za-z_][A-Za-z0-9_.-]*)\s*[:=]\s*([^,;\s]+)"
)


def normalize_control_mode(value):
    if isinstance(value, bool):
        return CONTROL_MODE_AUTO if value else CONTROL_MODE_MANUAL
    if isinstance(value, int):
        if value == 0:
            return CONTROL_MODE_MANUAL
        if value == 1:
            return CONTROL_MODE_AUTO

    text = str(value).strip().lower()
    if text in {"0", "manual", "man", "手动", "手控"}:
        return CONTROL_MODE_MANUAL
    if text in {"1", "auto", "automatic", "自动"}:
        return CONTROL_MODE_AUTO
    raise ValueError(f"不支持的控制模式: {value!r}")


def normalize_auto_motion_mode(value):
    text = str(value).strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "straight": AUTO_MOTION_STRAIGHT,
        "line": AUTO_MOTION_STRAIGHT,
        "forward": AUTO_MOTION_STRAIGHT,
        "直线": AUTO_MOTION_STRAIGHT,
        "spiral_left": AUTO_MOTION_SPIRAL_LEFT,
        "left_spiral": AUTO_MOTION_SPIRAL_LEFT,
        "left": AUTO_MOTION_SPIRAL_LEFT,
        "左螺旋": AUTO_MOTION_SPIRAL_LEFT,
        "spiral_right": AUTO_MOTION_SPIRAL_RIGHT,
        "right_spiral": AUTO_MOTION_SPIRAL_RIGHT,
        "right": AUTO_MOTION_SPIRAL_RIGHT,
        "右螺旋": AUTO_MOTION_SPIRAL_RIGHT,
    }
    try:
        return aliases[text]
    except KeyError as exc:
        raise ValueError(f"不支持的自动轨迹: {value!r}") from exc


def build_robot_mode_value(control_mode, auto_motion_mode):
    if normalize_control_mode(control_mode) == CONTROL_MODE_MANUAL:
        return ROBOT_MODE_MANUAL
    trajectory = normalize_auto_motion_mode(auto_motion_mode)
    return _MODE_AUTO_BIT | _TRAJECTORY_BITS[trajectory]


def decode_robot_mode_value(value):
    mode_value = int(value) & 0x07
    if not mode_value & _MODE_AUTO_BIT:
        return {
            "control_mode": CONTROL_MODE_MANUAL,
            "auto_motion_mode": AUTO_MOTION_STRAIGHT,
        }

    trajectory_bits = mode_value & 0x06
    trajectory = {
        0x02: AUTO_MOTION_SPIRAL_LEFT,
        0x04: AUTO_MOTION_SPIRAL_RIGHT,
    }.get(trajectory_bits, AUTO_MOTION_STRAIGHT)
    return {
        "control_mode": CONTROL_MODE_AUTO,
        "auto_motion_mode": trajectory,
    }


def coerce_controller_scalar(value):
    text = value.strip()
    lowered = text.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in {"none", "null"}:
        return None
    try:
        return int(text, 0)
    except ValueError:
        pass
    try:
        number = float(text)
        return number if math.isfinite(number) else text
    except ValueError:
        return text


def parse_controller_state_line(line):
    if isinstance(line, bytes):
        line = line.decode("utf-8", errors="replace")
    text = str(line or "").strip()
    if not text:
        raise ValueError("控制器状态为空")

    text = re.sub(r"^(?:CONTROLLER_STATE|STATE)\s*:\s*", "", text, flags=re.I)
    if text.startswith("{"):
        try:
            state = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"控制器状态 JSON 无效: {exc.msg}") from exc
        if not isinstance(state, dict):
            raise ValueError("控制器状态 JSON 必须是对象")
    else:
        state = {
            match.group(1): coerce_controller_scalar(match.group(2))
            for match in _CONTROLLER_KEY_VALUE_RE.finditer(text)
        }
        if not state:
            raise ValueError("控制器状态必须包含 key=value 字段")

    for key in ("mode_value", "robot_mode", "published_mode_value", "mode"):
        if key not in state:
            continue
        try:
            state.update(decode_robot_mode_value(state[key]))
        except (TypeError, ValueError):
            pass
        break
    return state


def controller_state_is_stale(received_monotonic, now=None, max_age_s=2.5):
    if received_monotonic is None:
        return True
    max_age_s = float(max_age_s)
    if not math.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError("max_age_s 必须是有限正数")
    current = time.monotonic() if now is None else float(now)
    received = float(received_monotonic)
    if not math.isfinite(current) or not math.isfinite(received):
        return True
    return max(0.0, current - received) > max_age_s

# =========================
# 全局状态
# =========================
# G1 云台网络控制状态
# TCP 读写使用单独的 RLock 串行化，避免 Flask 多线程同时占用相机端口。
gimbal_io_lock = threading.RLock()

# “动作锁”和“单帧 IO 锁”分开：
# - gimbal_action_lock：保证一个完整动作事务不会被姿态/模式/另一动作穿插；
# - gimbal_io_lock：保证每一帧 TCP 收发本身严格串行；
# - Stop 不等待 action_lock，而是先置 stop_event，再抢占下一次 IO，
#   因此仍然可以快速打断正在等待中的点按动作。
gimbal_action_lock = threading.RLock()
gimbal_state_lock = threading.RLock()
gimbal_watchdog_lock = threading.Lock()
gimbal_pulse_lock = threading.Lock()
gimbal_roll_lock = threading.RLock()

gimbal_stop_event = threading.Event()
gimbal_watchdog_timer = None
gimbal_watchdog_generation = 0

# 最近一次 TCP/协议事务完成时间，用于设备级节流。
gimbal_last_io_completed_monotonic = 0.0

# 横滚采用绝对角命令。首次需要真实姿态作为基准；之后使用软件目标缓存，
# 避免每次横滚点击都额外查询一次姿态。
gimbal_roll_target = None
gimbal_action_sequence = 0

gimbal_state = {
    # connected 保留给旧前端/API 使用，现在严格表示“TCP 端点可达”。
    "connected": False,
    "tcp_reachable": False,

    # 协议/控制与姿态读取分开。
    # 姿态读取失败绝不能再把 TCP connected 清零。
    "protocol_ok": False,
    "control_ready": False,
    "pose_ok": None,

    "mode": None,
    "mode_name": "未知",
    "moving": False,
    "axis": "",
    "direction": 0,
    "speed": 0,

    # last_error 只保存会影响“控制”的严重错误；
    # pose_error 单独保存姿态查询错误。
    "last_error": "",
    "last_tcp_error": "",
    "last_protocol_error": "",
    "last_control_error": "",
    "pose_error": "",

    "last_command_at": 0.0,
    "last_seen_at": 0.0,
    "pose_updated_at": 0.0,

    # 动作串行化 / 自动恢复诊断信息。
    "busy": False,
    "active_action": "",
    "sequence": 0,
    "recovery_count": 0,
    "last_recovery_reason": "",
    "last_recovery_at": 0.0,

    "pose": {
        "yaw": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "base_yaw": 0.0,
        "base_roll": 0.0,
        "base_pitch": 0.0,
    },
}

cleaning_esc = None
cleaning_lock = threading.Lock()
is_cleaning_on = False
cleaning_speed_percent = 0

video_state_lock = threading.RLock()
video_state = {
    "width": DEFAULT_WIDTH,
    "height": DEFAULT_HEIGHT,
    "fps": DEFAULT_FPS,
    "bitrate": BITRATE_MAP[(DEFAULT_WIDTH, DEFAULT_HEIGHT)],
}

# 异物识别状态
detection_lock = threading.RLock()
detection_count = 0
lost_count = 0
is_foreign_object_detected = False
last_detection_boxes = 0
last_detection_error = ""
last_detection_frame_time = 0.0
detection_total_frames = 0
last_detection_boxes_list = []
last_detection_frame_width = 0
last_detection_frame_height = 0
last_detection_infer_ms = 0.0
last_detection_worker_alive = False

# YOLO 自动避障状态
obstacle_avoidance_lock = threading.RLock()
is_obstacle_avoidance_enabled = False
obstacle_avoidance_state = {
    "active": False,
    "action": "idle",
    "direction": "none",
    "left_overlap": False,
    "right_overlap": False,
    "overlap_box_count": 0,
    "danger_zone": None,
    "last_action_time": 0.0,
    "last_error": "",
    "status": "自动避障已关闭",
}
obstacle_avoidance_thread_lock = threading.Lock()
obstacle_avoidance_thread = None
obstacle_avoidance_stop_event = threading.Event()

# OpenCV 暗目标异物识别状态
# 注意：该状态与上面的 YOLO 坑洞识别状态完全独立，避免覆盖原有 YOLO 逻辑。
foreign_detection_lock = threading.RLock()
foreign_detection_count = 0
foreign_lost_count = 0
is_dark_foreign_object_detected = False
last_foreign_detection_boxes = 0
last_foreign_detection_error = ""
last_foreign_detection_frame_time = 0.0
foreign_detection_total_frames = 0
last_foreign_detection_boxes_list = []
last_foreign_detection_frame_width = 0
last_foreign_detection_frame_height = 0
last_foreign_detection_infer_ms = 0.0
last_foreign_detection_worker_alive = False
is_foreign_detection_enabled = False

foreign_detection_thread_lock = threading.Lock()
foreign_detection_thread = None
foreign_detection_stop_event = threading.Event()

yolo_model = None
yolo_model_lock = threading.Lock()

detection_thread_lock = threading.Lock()
detection_thread = None
detection_stop_event = threading.Event()

mediamtx_lock = threading.RLock()
mediamtx_process = None
mediamtx_log_fp = None
last_stream_error = ""

publisher_lock = threading.RLock()
publisher_process = None
publisher_log_fp = None
last_publisher_error = ""

recording_lock = threading.RLock()
recording_process = None
recording_log_fp = None
current_recording_file = None
last_recording_error = ""

whep_sessions = {}
whep_sessions_lock = threading.Lock()

# WebRTC 网络地址变化监控。
# 地址变化后会重启 MediaMTX 与相机 publisher，使新的 ICE 候选地址立即生效。
webrtc_network_watch_lock = threading.RLock()
webrtc_network_watch_thread = None
webrtc_network_watch_stop_event = threading.Event()
last_webrtc_advertised_hosts = ()
webrtc_network_reconfigure_pending = False

# ROS2 / 机器人控制状态
robot_control_lock = threading.RLock()
robot_state = {
    "running": False,
    "linear_speed": ROBOT_LINEAR_SPEED_DEFAULT,
    "angular_speed": ROBOT_ANGULAR_SPEED_DEFAULT,
    "fan_speed": 0,
    "last_direction": "stop",
    "control_mode": ROBOT_MODE_MANUAL,
    "auto_motion_mode": AUTO_MOTION_STRAIGHT,
    "published_mode_value": ROBOT_MODE_MANUAL,
}

basecontroller_lock = threading.RLock()
basecontroller_process = None
basecontroller_log_fp = None
last_basecontroller_error = ""

ros_lock = threading.RLock()
ros_node = None
ros_thread = None
ros_ready = False

controller_state_lock = threading.RLock()
latest_controller_state = {}
latest_controller_state_received_monotonic = None
last_controller_state_error = ""


# =========================
# 工具函数
# =========================
def validate_video_constants():
    required = [
        "RESOLUTION_OPTIONS",
        "FPS_OPTIONS",
        "DEFAULT_WIDTH",
        "DEFAULT_HEIGHT",
        "DEFAULT_FPS",
        "BITRATE_MAP",
    ]
    for k in required:
        if k not in globals():
            raise RuntimeError(f"缺少全局配置: {k}")

    if not isinstance(RESOLUTION_OPTIONS, (list, tuple)) or len(RESOLUTION_OPTIONS) == 0:
        raise RuntimeError("RESOLUTION_OPTIONS 配置无效")

    if (DEFAULT_WIDTH, DEFAULT_HEIGHT) not in BITRATE_MAP:
        raise RuntimeError("BITRATE_MAP 缺少默认分辨率的码率映射")


def resolution_to_str(width, height):
    return f"{int(width)}x{int(height)}"


def parse_resolution(value):
    text = str(value).strip().lower().replace(" ", "")
    parts = re.split(r"[x×]", text, maxsplit=1)
    if len(parts) != 2 or not parts[0] or not parts[1]:
        raise ValueError("resolution 格式应为 1280x720")
    w, h = parts
    return int(w), int(h)


def is_supported_resolution(width, height):
    return (int(width), int(height)) in RESOLUTION_OPTIONS


def is_supported_fps(fps):
    return int(fps) in FPS_OPTIONS


def get_bitrate(width, height):
    return int(BITRATE_MAP.get((int(width), int(height)), 3500000))


def get_idr_period(fps):
    return max(1, int(fps))


def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"



def is_advertisable_ipv4(ip: str) -> bool:
    """
    判断地址是否适合公布给浏览器作为局域网 WebRTC ICE 候选地址。
    排除 loopback、0.0.0.0、169.254.x.x 自动私有地址和多播地址。
    """
    try:
        chunks = [int(x) for x in str(ip).strip().split(".")]
        if len(chunks) != 4 or any(x < 0 or x > 255 for x in chunks):
            return False
    except Exception:
        return False

    return not (
        chunks[0] in (0, 127, 224, 225, 226, 227, 228, 229, 230, 231,
                      232, 233, 234, 235, 236, 237, 238, 239)
        or (chunks[0] == 169 and chunks[1] == 254)
    )


def _append_unique_host(items, value):
    value = str(value or "").strip()
    if value and value not in items:
        items.append(value)


def get_interface_ipv4_addresses():
    """
    返回当前树莓派上可用于局域网访问的 IPv4 地址。

    优先使用 `ip -4 -o addr show scope global`，避免 get_ip_address()
    仅根据默认路由猜测一个地址；同时保留 psutil/socket 作为回退。
    """
    found = []

    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )
        for line in (result.stdout or "").splitlines():
            match = re.search(r"^\d+:\s+(\S+)\s+inet\s+(\d+\.\d+\.\d+\.\d+)/", line)
            if not match:
                continue

            iface, ip = match.groups()
            iface_l = iface.lower()
            # 容器、虚拟桥接、VPN 的地址多数不应提供给普通局域网浏览器。
            if iface_l.startswith(("docker", "br-", "veth", "virbr", "zt", "tailscale")):
                continue
            if is_advertisable_ipv4(ip):
                _append_unique_host(found, ip)
    except Exception:
        pass

    if psutil is not None:
        try:
            for iface, addresses in psutil.net_if_addrs().items():
                iface_l = str(iface).lower()
                if iface_l.startswith(("lo", "docker", "br-", "veth", "virbr", "zt", "tailscale")):
                    continue
                for addr in addresses:
                    if getattr(addr, "family", None) == socket.AF_INET:
                        ip = getattr(addr, "address", "")
                        if is_advertisable_ipv4(ip):
                            _append_unique_host(found, ip)
        except Exception:
            pass

    # 最后的回退：保留原逻辑获取到的默认出口地址。
    fallback_ip = get_ip_address()
    if is_advertisable_ipv4(fallback_ip):
        _append_unique_host(found, fallback_ip)

    return found


def get_webrtc_advertised_hosts():
    """
    生成写入 MediaMTX webrtcAdditionalHosts 的地址列表。
    WEBRTC_ADDITIONAL_HOSTS 可用于手动附加 IP / 主机名，但不需要修改代码。
    """
    hosts = list(get_interface_ipv4_addresses())

    extra = os.environ.get(WEBRTC_ADDITIONAL_HOSTS_ENV, "")
    for value in re.split(r"[,;\s]+", extra):
        _append_unique_host(hosts, value)

    return hosts


def tail_log(path: Path, n=80):
    try:
        lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
        return "\n".join(lines[-n:])
    except Exception:
        return ""


def get_cpu_temp():
    paths = [
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/devices/virtual/thermal/thermal_zone0/temp",
    ]
    for p in paths:
        try:
            v = float(Path(p).read_text().strip()) / 1000.0
            return f"{v:.1f}°C"
        except Exception:
            continue
    return "--.-°C"


def get_cpu_temp_float():
    paths = [
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/devices/virtual/thermal/thermal_zone0/temp",
    ]
    for p in paths:
        try:
            return float(Path(p).read_text().strip()) / 1000.0
        except Exception:
            continue
    return 0.0


def get_cpu_usage():
    if psutil is None:
        return "--%"
    try:
        return f"{psutil.cpu_percent(interval=0.06):.1f}%"
    except Exception:
        return "--%"


def get_mem_usage():
    if psutil is None:
        return "--%"
    try:
        return f"{psutil.virtual_memory().percent:.1f}%"
    except Exception:
        return "--%"


def terminate_process_group(proc, timeout=3.0):
    if proc is None:
        return
    try:
        if proc.poll() is None:
            if os.name != "nt":
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            else:
                proc.terminate()
            proc.wait(timeout=timeout)
    except Exception:
        try:
            if proc.poll() is None:
                if os.name != "nt":
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                else:
                    proc.kill()
        except Exception:
            pass


def kill_stray_mediamtx():
    if psutil is not None:
        for p in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                name = (p.info.get("name") or "").lower()
                cmd = " ".join(p.info.get("cmdline") or []).lower()
                if "mediamtx" in name or "mediamtx" in cmd:
                    p.kill()
            except Exception:
                pass
    else:
        try:
            subprocess.run(["pkill", "-f", "mediamtx"], check=False)
        except Exception:
            pass


def require_camera_stack():
    missing = []
    for p in [CAMERA_BIN, CAMERA_LIB_DIR, CAMERA_IPA_MODULE_DIR, CAMERA_IPA_PROXY_DIR]:
        if not p.exists():
            missing.append(str(p))
    if missing:
        raise RuntimeError(
            "未找到已验证可用的新相机栈 /opt/rpi-cam-stack。\n缺少:\n" + "\n".join(missing)
        )


ICE_SERVER_LINK_RE = re.compile(
    r'<([^>]+)>\s*;\s*rel="ice-server"'
    r'(?:\s*;\s*username="([^"]*)"\s*;\s*credential="([^"]*)"\s*;\s*credential-type="password")?',
    re.IGNORECASE,
)


def get_whep_url():
    return f"http://127.0.0.1:{WEBRTC_HTTP_PORT}/cam/whep"


def json_unquote_quoted_value(v: str):
    if v is None:
        return None
    try:
        return json.loads(f'"{v}"')
    except Exception:
        return v


def parse_whep_link_header(link_header: str):
    if not link_header:
        return []

    ice_servers = []
    for m in ICE_SERVER_LINK_RE.finditer(link_header):
        item = {
            "urls": [m.group(1)],
        }
        if m.group(2) is not None:
            item["username"] = json_unquote_quoted_value(m.group(2))
            item["credential"] = json_unquote_quoted_value(m.group(3) or "")
            item["credentialType"] = "password"
        ice_servers.append(item)

    return ice_servers


def build_stream_error_text():
    parts = []
    for item in [last_stream_error, last_publisher_error, last_recording_error, last_basecontroller_error]:
        item = (item or "").strip()
        if item and item not in parts:
            parts.append(item)
    return "\n".join(parts).strip()


# =========================
# G1 云台 TCP 控制
# =========================
G1_ERROR_TEXT = {
    0x00: "无错误",
    0x01: "缺少 0xAA 命令头",
    0x02: "未接收到正确命令",
    0x03: "命令总字节数不正确",
    0x04: "CRC 校验错误",
    0xFF: "相机忽略该指令",
}

G1_MODE_NAME = {
    0x00: "锁定模式",
    0x01: "航向跟随、俯仰锁定",
    0x02: "航向俯仰跟随",
    0x03: "全跟随",
}


def g1_format_hex(data):
    if data is None:
        return ""
    return " ".join("{:02X}".format(v) for v in bytearray(data))


def g1_crc8(data):
    """G1 协议 CRC8：初值 0x00，多项式 0xD5。"""
    crc = 0x00
    for value in bytearray(data):
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def g1_build_frame(command, params=b""):
    params = bytes(params or b"")
    total_length = 4 + len(params)
    if total_length > 255:
        raise ValueError("G1 命令长度超过 255 字节")

    body = bytes([0xAA, total_length, int(command) & 0xFF]) + params
    return body + bytes([g1_crc8(body)])


def g1_recv_exact(sock, size):
    result = bytearray()
    while len(result) < size:
        chunk = sock.recv(size - len(result))
        if not chunk:
            raise ConnectionError(
                "G1 TCP 连接提前关闭：期望 {} 字节，实际 {} 字节".format(
                    size, len(result)
                )
            )
        result.extend(chunk)
    return bytes(result)


def g1_recv_frame(sock):
    header = g1_recv_exact(sock, 2)
    if header[0] != 0x55:
        raise ValueError("G1 返回帧头错误：{}".format(g1_format_hex(header)))

    total_length = int(header[1])
    if total_length < 6:
        raise ValueError("G1 返回帧长度异常：{}".format(total_length))

    frame = header + g1_recv_exact(sock, total_length - 2)
    expected_crc = g1_crc8(frame[:-1])
    actual_crc = frame[-1]
    if expected_crc != actual_crc:
        raise ValueError(
            "G1 返回 CRC 错误：计算 {:02X}，收到 {:02X}，帧 {}".format(
                expected_crc, actual_crc, g1_format_hex(frame)
            )
        )
    return frame


def g1_parse_response(frame):
    if len(frame) < 6:
        raise ValueError("G1 返回帧过短：{}".format(g1_format_hex(frame)))

    result = {
        "raw": frame,
        "command": int(frame[2]),
        "error_code": int(frame[3]),
        "data_type": int(frame[4]),
        "params": bytes(frame[5:-1]),
        "checksum": int(frame[-1]),
    }

    if result["error_code"] != 0x00:
        code = result["error_code"]
        raise RuntimeError(
            "G1 返回错误 0x{:02X}：{}".format(
                code, G1_ERROR_TEXT.get(code, "未知错误")
            )
        )
    return result


def _g1_mark_transport_reachable():
    """TCP 已经实际 connect 成功；不等同于姿态查询成功。"""
    now = time.time()
    with gimbal_state_lock:
        gimbal_state["connected"] = True
        gimbal_state["tcp_reachable"] = True
        gimbal_state["last_tcp_error"] = ""
        gimbal_state["last_seen_at"] = now


def _g1_mark_transport_error(exc):
    """只有 TCP connect 本身失败，才把 connected 清零。"""
    text = str(exc)
    with gimbal_state_lock:
        gimbal_state["connected"] = False
        gimbal_state["tcp_reachable"] = False
        gimbal_state["protocol_ok"] = False
        gimbal_state["control_ready"] = False
        gimbal_state["last_tcp_error"] = text
        gimbal_state["last_control_error"] = text
        gimbal_state["last_error"] = text


def _g1_mark_protocol_success(affect_control_state=True):
    """
    收到并成功解析一帧 G1 协议响应。
    pose/mode 查询可以证明协议可通信，但不会被当成“控制命令成功”。
    """
    now = time.time()
    with gimbal_state_lock:
        gimbal_state["connected"] = True
        gimbal_state["tcp_reachable"] = True
        gimbal_state["protocol_ok"] = True
        gimbal_state["last_protocol_error"] = ""
        gimbal_state["last_seen_at"] = now

        if affect_control_state:
            gimbal_state["control_ready"] = True
            gimbal_state["last_control_error"] = ""
            gimbal_state["last_error"] = ""
            gimbal_state["last_command_at"] = now


def _g1_mark_operation_error(
    exc,
    *,
    operation="control",
    affect_control_state=True,
    transport_reachable=True,
):
    """
    TCP 已 connect 后发生的超时/协议错误，不应该误判成“TCP 未连接”。

    pose:
        只更新 pose_error / pose_ok。
    mode/query:
        记录协议查询错误，但不破坏已存在的控制状态。
    control:
        控制命令本身失败，才更新 control_ready / last_error。
    """
    text = str(exc)
    now = time.time()

    with gimbal_state_lock:
        if transport_reachable:
            gimbal_state["connected"] = True
            gimbal_state["tcp_reachable"] = True
            gimbal_state["last_seen_at"] = now

        if operation == "pose":
            gimbal_state["pose_ok"] = False
            gimbal_state["pose_error"] = text
            return

        gimbal_state["last_protocol_error"] = text

        if affect_control_state:
            gimbal_state["protocol_ok"] = False
            gimbal_state["control_ready"] = False
            gimbal_state["last_control_error"] = text
            gimbal_state["last_error"] = text


class G1ControlOperationError(RuntimeError):
    """控制动作失败；recovered=True 表示已经自动发送 Stop 并恢复到安全状态。"""

    def __init__(self, message, *, recovered=False, recovery_error=""):
        super().__init__(str(message))
        self.recovered = bool(recovered)
        self.recovery_error = str(recovery_error or "")


def _g1_begin_action(name):
    global gimbal_action_sequence

    with gimbal_state_lock:
        gimbal_action_sequence += 1
        sequence = int(gimbal_action_sequence)
        gimbal_state["busy"] = True
        gimbal_state["active_action"] = str(name or "")
        gimbal_state["sequence"] = sequence
    return sequence


def _g1_finish_action(sequence):
    with gimbal_state_lock:
        if int(gimbal_state.get("sequence") or 0) == int(sequence):
            gimbal_state["busy"] = False
            gimbal_state["active_action"] = ""


def _g1_wait_command_gap():
    """在 gimbal_io_lock 内调用，避免过快创建 TCP 会话/发送协议帧。"""
    global gimbal_last_io_completed_monotonic

    last = float(gimbal_last_io_completed_monotonic or 0.0)
    if last <= 0:
        return

    elapsed = time.monotonic() - last
    remaining = float(G1_COMMAND_MIN_INTERVAL) - elapsed
    if remaining > 0:
        time.sleep(remaining)


def _g1_note_io_completed():
    global gimbal_last_io_completed_monotonic
    gimbal_last_io_completed_monotonic = time.monotonic()


def _g1_set_motion_state(moving=False, axis="", direction=0, speed=0):
    with gimbal_state_lock:
        gimbal_state["moving"] = bool(moving)
        gimbal_state["axis"] = str(axis or "")
        gimbal_state["direction"] = int(direction or 0)
        gimbal_state["speed"] = int(speed or 0)


def _g1_record_recovery(reason):
    now = time.time()
    with gimbal_state_lock:
        gimbal_state["recovery_count"] = int(
            gimbal_state.get("recovery_count") or 0
        ) + 1
        gimbal_state["last_recovery_reason"] = str(reason or "")
        gimbal_state["last_recovery_at"] = now


def g1_probe_tcp():
    """
    无动作 TCP 探测。

    Probe 也通过 action/io 锁和最小间隔，避免状态检测在控制过程中
    额外打开一个 8888 TCP 连接。
    """
    with gimbal_action_lock:
        with gimbal_io_lock:
            _g1_wait_command_gap()
            try:
                with socket.create_connection(
                    (G1_CAMERA_IP, G1_CONTROL_PORT),
                    timeout=G1_CONNECT_TIMEOUT,
                ):
                    _g1_mark_transport_reachable()
                return True
            except (socket.timeout, TimeoutError, ConnectionError, OSError) as exc:
                _g1_mark_transport_error(exc)
                return False
            finally:
                _g1_note_io_completed()


def g1_send_frame(
    frame,
    retries=None,
    *,
    affect_control_state=True,
    operation="control",
):
    """发送一帧 G1 命令并返回解析后的响应。

    稳定性策略：
    1. 所有 TCP/协议事务严格串行；
    2. 相邻事务之间有最小间隔；
    3. control 默认零重试，防止“设备已执行但 ACK 丢失”后重复动作；
    4. pose/mode 查询可有限重试；
    5. TCP 已 connect 后的协议/响应错误不会被误报成“网络断开”。
    """
    if retries is None:
        if operation in ("pose", "mode", "query"):
            retry_count = int(G1_QUERY_RETRIES)
        else:
            retry_count = int(G1_CONTROL_RETRIES)
    else:
        retry_count = max(0, int(retries))

    last_exc = None

    with gimbal_io_lock:
        for attempt in range(retry_count + 1):
            connected_this_attempt = False

            _g1_wait_command_gap()

            try:
                with socket.create_connection(
                    (G1_CAMERA_IP, G1_CONTROL_PORT),
                    timeout=G1_CONNECT_TIMEOUT,
                ) as sock:
                    connected_this_attempt = True
                    _g1_mark_transport_reachable()

                    sock.settimeout(G1_RESPONSE_TIMEOUT)
                    sock.sendall(frame)
                    response = g1_recv_frame(sock)

                parsed = g1_parse_response(response)
                _g1_mark_protocol_success(
                    affect_control_state=affect_control_state
                )
                return parsed

            except (socket.timeout, TimeoutError, ConnectionError, OSError) as exc:
                last_exc = exc

                if attempt < retry_count:
                    # 查询类命令才会走到这里。控制命令默认 retry_count=0。
                    time.sleep(max(0.08, G1_COMMAND_MIN_INTERVAL))
                    continue

                if connected_this_attempt:
                    _g1_mark_operation_error(
                        exc,
                        operation=operation,
                        affect_control_state=affect_control_state,
                        transport_reachable=True,
                    )
                else:
                    _g1_mark_transport_error(exc)
                raise

            except Exception as exc:
                last_exc = exc
                _g1_mark_operation_error(
                    exc,
                    operation=operation,
                    affect_control_state=affect_control_state,
                    transport_reachable=connected_this_attempt,
                )
                raise

            finally:
                _g1_note_io_completed()

    if last_exc is not None:
        raise last_exc
    raise RuntimeError("G1 命令发送失败")


def g1_send_command(
    command,
    params=b"",
    *,
    affect_control_state=True,
    operation="control",
    retries=None,
):
    frame = g1_build_frame(command, params)
    return g1_send_frame(
        frame,
        retries=retries,
        affect_control_state=affect_control_state,
        operation=operation,
    )


def _g1_send_neutral(*, affect_control_state=True):
    """发送航向/俯仰零速帧，作为动作之间的明确“中和/停止”边界。"""
    return g1_send_frame(
        g1_build_joystick_frame(0, 0),
        retries=0,
        affect_control_state=affect_control_state,
        operation="control",
    )


def g1_query_mode():
    with gimbal_action_lock:
        response = g1_send_command(
            0x00,
            bytes([0x05, 0x01]),
            affect_control_state=False,
            operation="mode",
        )
        params = response["params"]
        if not params:
            raise ValueError("G1 云台模式响应没有参数")

        mode = int(params[-1])
        with gimbal_state_lock:
            gimbal_state["mode"] = mode
            gimbal_state["mode_name"] = G1_MODE_NAME.get(mode, "未知模式")
        return mode


def g1_set_mode(mode):
    mode = int(mode)
    if mode not in G1_MODE_NAME:
        raise ValueError("云台模式只允许 0、1、2、3")

    with gimbal_action_lock:
        g1_send_command(
            0x05,
            bytes([0x01, mode]),
            retries=0,
            operation="control",
        )
        with gimbal_state_lock:
            gimbal_state["mode"] = mode
            gimbal_state["mode_name"] = G1_MODE_NAME[mode]
        return mode


def g1_ensure_manual_mode():
    if not G1_AUTO_LOCK_MODE:
        return

    with gimbal_state_lock:
        current_mode = gimbal_state.get("mode")
    if current_mode != 0x00:
        g1_set_mode(0x00)


def g1_query_pose():
    """
    查询真实姿态。

    重要：只在用户主动读取姿态、首次需要横滚基准等明确场景调用。
    航向/俯仰每次点按结束后不再自动查询姿态。
    """
    global gimbal_roll_target

    with gimbal_action_lock:
        try:
            response = g1_send_command(
                0x00,
                bytes([0x05, 0x02]),
                affect_control_state=False,
                operation="pose",
            )

            params = response["params"]
            if len(params) < 12:
                raise ValueError(
                    "G1 姿态参数长度不足 12 字节：{}".format(
                        g1_format_hex(params)
                    )
                )

            values = struct.unpack(">hhhhhh", params[:12])
            pose = {
                "yaw": values[0] / 100.0,
                "roll": values[1] / 100.0,
                "pitch": values[2] / 100.0,
                "base_yaw": values[3] / 100.0,
                "base_roll": values[4] / 100.0,
                "base_pitch": values[5] / 100.0,
            }

            with gimbal_state_lock:
                gimbal_state["pose"] = dict(pose)
                gimbal_state["pose_updated_at"] = time.time()
                gimbal_state["pose_ok"] = True
                gimbal_state["pose_error"] = ""
                gimbal_state["connected"] = True
                gimbal_state["tcp_reachable"] = True
                gimbal_state["protocol_ok"] = True
                gimbal_state["last_protocol_error"] = ""
                gimbal_state["last_seen_at"] = time.time()

            with gimbal_roll_lock:
                gimbal_roll_target = float(pose["roll"])

            return pose

        except Exception as exc:
            with gimbal_state_lock:
                transport_reachable = bool(gimbal_state["tcp_reachable"])

            _g1_mark_operation_error(
                exc,
                operation="pose",
                affect_control_state=False,
                transport_reachable=transport_reachable,
            )
            raise


def g1_get_cached_pose(refresh=False):
    now = time.time()
    with gimbal_state_lock:
        cached = dict(gimbal_state["pose"])
        age = now - float(gimbal_state.get("pose_updated_at") or 0.0)

    if refresh and age >= G1_POSE_CACHE_SECONDS:
        try:
            return g1_query_pose()
        except Exception:
            return cached
    return cached


def g1_get_public_status(refresh_pose=False, probe_tcp=False):
    if probe_tcp:
        g1_probe_tcp()

    pose = g1_get_cached_pose(refresh=refresh_pose)

    with gimbal_state_lock:
        return {
            "camera_ip": G1_CAMERA_IP,
            "control_port": G1_CONTROL_PORT,
            "rtsp_port": G1_RTSP_PORT,
            "rtsp_path": G1_RTSP_PATH,
            "rtsp_url": G1_RTSP_URL,

            "connected": bool(gimbal_state["tcp_reachable"]),
            "tcp_reachable": bool(gimbal_state["tcp_reachable"]),
            "protocol_ok": bool(gimbal_state["protocol_ok"]),
            "control_ready": bool(gimbal_state["control_ready"]),
            "pose_ok": gimbal_state["pose_ok"],

            "mode": gimbal_state["mode"],
            "mode_name": gimbal_state["mode_name"],
            "moving": bool(gimbal_state["moving"]),
            "axis": gimbal_state["axis"],
            "direction": int(gimbal_state["direction"]),
            "speed": int(gimbal_state["speed"]),

            "busy": bool(gimbal_state.get("busy")),
            "active_action": gimbal_state.get("active_action") or "",
            "sequence": int(gimbal_state.get("sequence") or 0),

            "last_error": gimbal_state["last_error"],
            "tcp_error": gimbal_state["last_tcp_error"],
            "protocol_error": gimbal_state["last_protocol_error"],
            "control_error": gimbal_state["last_control_error"],
            "pose_error": gimbal_state["pose_error"],

            "recovery_count": int(gimbal_state.get("recovery_count") or 0),
            "last_recovery_reason": gimbal_state.get("last_recovery_reason") or "",
            "last_recovery_at": float(gimbal_state.get("last_recovery_at") or 0.0),

            "last_command_at": float(gimbal_state["last_command_at"] or 0.0),
            "last_seen_at": float(gimbal_state["last_seen_at"] or 0.0),
            "pose_updated_at": float(gimbal_state["pose_updated_at"] or 0.0),
            "pose": dict(pose),

            "control_retries": int(G1_CONTROL_RETRIES),
            "query_retries": int(G1_QUERY_RETRIES),
            "command_min_interval_ms": int(round(G1_COMMAND_MIN_INTERVAL * 1000)),
            "neutral_settle_ms": int(round(G1_NEUTRAL_SETTLE_SECONDS * 1000)),
            "post_action_settle_ms": int(round(G1_POST_ACTION_SETTLE_SECONDS * 1000)),
        }


def g1_build_joystick_frame(yaw_speed, pitch_speed):
    yaw_speed = max(-128, min(128, int(yaw_speed)))
    pitch_speed = max(-128, min(128, int(pitch_speed)))
    params = bytes([0x06]) + struct.pack(">hh", yaw_speed, pitch_speed)
    return g1_build_frame(0x05, params)


def _g1_angle_to_protocol(angle, limits):
    if angle is None:
        return -32768  # 0x8000：该轴不转动
    lo, hi = limits
    angle = max(float(lo), min(float(hi), float(angle)))
    return int(round(angle * 10.0))


def g1_build_absolute_angle_frame(yaw=None, roll=None, pitch=None, slow=True):
    yaw_raw = _g1_angle_to_protocol(yaw, G1_YAW_LIMIT)
    roll_raw = _g1_angle_to_protocol(roll, G1_ROLL_LIMIT)
    pitch_raw = _g1_angle_to_protocol(pitch, G1_PITCH_LIMIT)
    speed_mode = 0x01 if slow else 0x00
    params = bytes([0x05]) + struct.pack(
        ">hhhB",
        yaw_raw,
        roll_raw,
        pitch_raw,
        speed_mode,
    )
    return g1_build_frame(0x05, params)


def _cancel_gimbal_watchdog():
    global gimbal_watchdog_timer, gimbal_watchdog_generation
    with gimbal_watchdog_lock:
        gimbal_watchdog_generation += 1
        timer = gimbal_watchdog_timer
        gimbal_watchdog_timer = None
    if timer is not None:
        try:
            timer.cancel()
        except Exception:
            pass


def _gimbal_watchdog_expired(generation):
    global gimbal_watchdog_timer
    with gimbal_watchdog_lock:
        if generation != gimbal_watchdog_generation:
            return
        gimbal_watchdog_timer = None

    try:
        gimbal_stop_event.set()
        _g1_send_neutral()
        _g1_set_motion_state(False, "", 0, 0)
        print("[WARN] G1 云台按住控制超时，已自动发送停止命令")
    except Exception as exc:
        print("[WARN] G1 云台超时停止失败: {}".format(exc))


def _arm_gimbal_watchdog():
    global gimbal_watchdog_timer, gimbal_watchdog_generation
    with gimbal_watchdog_lock:
        gimbal_watchdog_generation += 1
        generation = gimbal_watchdog_generation
        old_timer = gimbal_watchdog_timer
        timer = threading.Timer(
            G1_MAX_JOG_SECONDS,
            _gimbal_watchdog_expired,
            args=(generation,),
        )
        timer.daemon = True
        gimbal_watchdog_timer = timer

    if old_timer is not None:
        try:
            old_timer.cancel()
        except Exception:
            pass
    timer.start()


def _g1_get_roll_reference():
    global gimbal_roll_target

    with gimbal_roll_lock:
        if gimbal_roll_target is not None:
            return float(gimbal_roll_target)

    with gimbal_state_lock:
        if (
            gimbal_state.get("pose_ok") is True
            and float(gimbal_state.get("pose_updated_at") or 0.0) > 0
        ):
            reference = float(gimbal_state["pose"].get("roll") or 0.0)
            with gimbal_roll_lock:
                gimbal_roll_target = reference
            return reference

    # 首次横滚确实需要一个真实基准；只在这里读取一次。
    pose = g1_query_pose()
    return float(pose["roll"])


def _g1_try_recovery_stop(reason):
    """
    控制事务出现异常后，尽最大努力发送一次零速 Stop。

    如果 Stop 成功，控制状态会被重新标记为 ready，避免一次 ACK/解析异常
    长时间污染后续状态；原始失败仍通过 recovered 字段告诉前端。
    """
    recovery_error = ""
    recovered = False

    gimbal_stop_event.set()
    _cancel_gimbal_watchdog()

    try:
        _g1_send_neutral(affect_control_state=True)
        _g1_set_motion_state(False, "", 0, 0)
        time.sleep(G1_POST_ACTION_SETTLE_SECONDS)
        recovered = True
        _g1_record_recovery(reason)
    except Exception as exc:
        recovery_error = str(exc)

    return recovered, recovery_error


def g1_start_jog(axis, direction, speed=17, roll_step=5.0):
    """
    兼容旧 start API。

    即使是 start 模式，也在新动作前自动插入一次零速中和，
    防止连续方向命令直接覆盖设备内部尚未稳定的上一状态。
    """
    global gimbal_roll_target

    axis = str(axis or "").strip().lower()
    if axis not in ("yaw", "pitch", "roll"):
        raise ValueError("axis 只允许 yaw、pitch、roll")

    direction = -1 if int(direction) < 0 else 1
    speed = max(1, min(128, int(round(float(speed)))))
    roll_step = max(0.1, min(40.0, float(roll_step)))

    with gimbal_action_lock:
        sequence = _g1_begin_action("jog_start")
        try:
            gimbal_stop_event.clear()
            _cancel_gimbal_watchdog()
            g1_ensure_manual_mode()

            _g1_send_neutral()
            if gimbal_stop_event.wait(G1_NEUTRAL_SETTLE_SECONDS):
                _g1_set_motion_state(False, "", 0, 0)
                return {
                    "response": None,
                    "interrupted": True,
                    "one_shot": True,
                    "sequence": sequence,
                }

            if axis == "roll":
                reference = _g1_get_roll_reference()
                target_roll = (
                    reference
                    + direction * G1_ROLL_DIRECTION * roll_step
                )
                target_roll = max(
                    G1_ROLL_LIMIT[0],
                    min(G1_ROLL_LIMIT[1], target_roll),
                )

                if gimbal_stop_event.is_set():
                    return {
                        "response": None,
                        "interrupted": True,
                        "one_shot": True,
                        "sequence": sequence,
                    }

                response = g1_send_frame(
                    g1_build_absolute_angle_frame(
                        yaw=None,
                        roll=target_roll,
                        pitch=None,
                        slow=True,
                    ),
                    retries=0,
                    operation="control",
                )

                with gimbal_roll_lock:
                    gimbal_roll_target = target_roll

                _g1_set_motion_state(False, "roll", direction, 0)
                gimbal_stop_event.wait(G1_ROLL_SETTLE_SECONDS)

                return {
                    "response": response,
                    "target_roll": target_roll,
                    "one_shot": True,
                    "sequence": sequence,
                }

            yaw_speed = 0
            pitch_speed = 0
            if axis == "yaw":
                yaw_speed = direction * G1_YAW_DIRECTION * speed
            else:
                pitch_speed = direction * G1_PITCH_DIRECTION * speed

            response = g1_send_frame(
                g1_build_joystick_frame(yaw_speed, pitch_speed),
                retries=0,
                operation="control",
            )
            _g1_set_motion_state(True, axis, direction, speed)
            _arm_gimbal_watchdog()

            return {
                "response": response,
                "yaw_speed": yaw_speed,
                "pitch_speed": pitch_speed,
                "one_shot": False,
                "sequence": sequence,
            }

        except Exception as exc:
            recovered, recovery_error = _g1_try_recovery_stop(
                "jog_start: {}".format(exc)
            )
            raise G1ControlOperationError(
                exc,
                recovered=recovered,
                recovery_error=recovery_error,
            ) from exc
        finally:
            _g1_finish_action(sequence)


def g1_pulse_motion(axis, direction, speed=17, duration_ms=None, roll_step=5.0):
    """
    稳定版“单击一次”动作事务。

    航向/俯仰：
        中和 Stop -> 稳定等待 -> 运动 -> 固定时长 -> Stop -> 恢复等待

    横滚：
        中和 Stop -> 稳定等待 -> 使用缓存的绝对横滚目标 -> 到位等待
        首次横滚才读取一次真实姿态作为基准，后续不再每次查询。

    整个动作期间不会自动查询姿态，因此不会再把“姿态查询不稳定”
    混进正常的方向控制链路。
    """
    global gimbal_roll_target

    axis = str(axis or "").strip().lower()
    if axis not in ("yaw", "pitch", "roll"):
        raise ValueError("axis 只允许 yaw、pitch、roll")

    direction = -1 if int(direction) < 0 else 1
    speed = max(1, min(128, int(round(float(speed)))))
    if duration_ms is None:
        duration_ms = G1_PULSE_DURATION_MS
    duration_ms = max(80, min(800, int(round(float(duration_ms)))))
    roll_step = max(0.1, min(20.0, float(roll_step)))

    with gimbal_action_lock:
        sequence = _g1_begin_action("jog_pulse")

        try:
            with gimbal_pulse_lock:
                gimbal_stop_event.clear()
                _cancel_gimbal_watchdog()
                g1_ensure_manual_mode()

                # 关键稳定性措施：每个动作之前都先明确 Stop/Neutral。
                _g1_send_neutral()

                if gimbal_stop_event.wait(G1_NEUTRAL_SETTLE_SECONDS):
                    _g1_set_motion_state(False, "", 0, 0)
                    return {
                        "response": None,
                        "stop_response": None,
                        "axis": axis,
                        "direction": direction,
                        "speed": speed,
                        "duration_ms": 0,
                        "interrupted": True,
                        "pose": g1_get_cached_pose(refresh=False),
                        "pose_fresh": False,
                        "one_shot": True,
                        "sequence": sequence,
                    }

                if axis == "roll":
                    reference = _g1_get_roll_reference()
                    target_roll = (
                        reference
                        + direction * G1_ROLL_DIRECTION * roll_step
                    )
                    target_roll = max(
                        G1_ROLL_LIMIT[0],
                        min(G1_ROLL_LIMIT[1], target_roll),
                    )

                    if gimbal_stop_event.is_set():
                        return {
                            "response": None,
                            "axis": axis,
                            "direction": direction,
                            "target_roll": reference,
                            "duration_ms": 0,
                            "interrupted": True,
                            "pose": g1_get_cached_pose(refresh=False),
                            "pose_fresh": False,
                            "one_shot": True,
                            "sequence": sequence,
                        }

                    response = g1_send_frame(
                        g1_build_absolute_angle_frame(
                            yaw=None,
                            roll=target_roll,
                            pitch=None,
                            slow=True,
                        ),
                        retries=0,
                        operation="control",
                    )

                    with gimbal_roll_lock:
                        gimbal_roll_target = target_roll

                    _g1_set_motion_state(False, "roll", direction, 0)

                    interrupted = gimbal_stop_event.wait(
                        G1_ROLL_SETTLE_SECONDS
                    )
                    time.sleep(G1_POST_ACTION_SETTLE_SECONDS)

                    return {
                        "response": response,
                        "axis": axis,
                        "direction": direction,
                        "target_roll": target_roll,
                        "duration_ms": 0,
                        "interrupted": bool(interrupted),
                        "pose": g1_get_cached_pose(refresh=False),
                        "pose_fresh": False,
                        "one_shot": True,
                        "sequence": sequence,
                    }

                yaw_speed = 0
                pitch_speed = 0
                if axis == "yaw":
                    yaw_speed = direction * G1_YAW_DIRECTION * speed
                else:
                    pitch_speed = direction * G1_PITCH_DIRECTION * speed

                response = None
                stop_response = None
                primary_exc = None
                stop_exc = None
                interrupted = False

                try:
                    response = g1_send_frame(
                        g1_build_joystick_frame(
                            yaw_speed,
                            pitch_speed,
                        ),
                        retries=0,
                        operation="control",
                    )
                    _g1_set_motion_state(
                        True,
                        axis,
                        direction,
                        speed,
                    )

                    interrupted = gimbal_stop_event.wait(
                        duration_ms / 1000.0
                    )

                except Exception as exc:
                    primary_exc = exc

                finally:
                    # 无论 START 是否正常返回，都尽最大努力完成 Stop。
                    try:
                        stop_response = _g1_send_neutral()
                    except Exception as exc:
                        stop_exc = exc
                    finally:
                        _g1_set_motion_state(False, "", 0, 0)

                time.sleep(G1_POST_ACTION_SETTLE_SECONDS)

                if primary_exc is not None or stop_exc is not None:
                    original = primary_exc if primary_exc is not None else stop_exc
                    recovered = stop_response is not None
                    recovery_error = "" if recovered else str(stop_exc or "")

                    if recovered:
                        _g1_record_recovery(
                            "pulse 自动 Stop 恢复: {}".format(original)
                        )

                    raise G1ControlOperationError(
                        original,
                        recovered=recovered,
                        recovery_error=recovery_error,
                    ) from original

                return {
                    "response": response,
                    "stop_response": stop_response,
                    "axis": axis,
                    "direction": direction,
                    "yaw_speed": yaw_speed,
                    "pitch_speed": pitch_speed,
                    "speed": speed,
                    "duration_ms": duration_ms,
                    "interrupted": bool(interrupted),
                    "pose": g1_get_cached_pose(refresh=False),
                    "pose_fresh": False,
                    "one_shot": True,
                    "sequence": sequence,
                }

        except G1ControlOperationError:
            raise

        except Exception as exc:
            recovered, recovery_error = _g1_try_recovery_stop(
                "jog_pulse: {}".format(exc)
            )
            raise G1ControlOperationError(
                exc,
                recovered=recovered,
                recovery_error=recovery_error,
            ) from exc

        finally:
            _g1_finish_action(sequence)


def g1_stop_motion(refresh_pose=False):
    """
    Stop 为高优先级操作：
    先置 stop_event，使正在等待中的 pulse 立刻退出；
    不做姿态查询，不等待 action_lock。
    """
    gimbal_stop_event.set()
    _cancel_gimbal_watchdog()

    response = _g1_send_neutral()
    _g1_set_motion_state(False, "", 0, 0)

    time.sleep(G1_POST_ACTION_SETTLE_SECONDS)

    # refresh_pose 参数保留兼容，但稳定版默认禁止 Stop 后自动查姿态。
    pose = g1_get_cached_pose(refresh=False)
    return response, pose


def stop_gimbal_motion_safe():
    try:
        g1_stop_motion(refresh_pose=False)
    except Exception as exc:
        print("[WARN] 停止 G1 云台失败: {}".format(exc))


def g1_reset_home():
    """停止当前运动并执行协议中的回中命令；回中后不自动读取姿态。"""
    global gimbal_roll_target

    gimbal_stop_event.set()

    with gimbal_action_lock:
        sequence = _g1_begin_action("reset_home")
        try:
            with gimbal_pulse_lock:
                _cancel_gimbal_watchdog()

                _g1_send_neutral()
                time.sleep(G1_NEUTRAL_SETTLE_SECONDS)

                g1_ensure_manual_mode()
                response = g1_send_command(
                    0x05,
                    bytes([0x02]),
                    retries=0,
                    operation="control",
                )

                _g1_set_motion_state(False, "", 0, 0)

                # “回中”后横滚软件目标重新以 0° 为基准。
                with gimbal_roll_lock:
                    gimbal_roll_target = 0.0

                with gimbal_state_lock:
                    gimbal_state["pose_ok"] = None
                    gimbal_state["pose_error"] = ""
                    gimbal_state["pose_updated_at"] = 0.0
                    gimbal_state["pose"] = {
                        "yaw": 0.0,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "base_yaw": 0.0,
                        "base_roll": 0.0,
                        "base_pitch": 0.0,
                    }

                time.sleep(max(0.55, G1_POST_ACTION_SETTLE_SECONDS))
                return response, g1_get_cached_pose(refresh=False)

        except Exception as exc:
            recovered, recovery_error = _g1_try_recovery_stop(
                "reset_home: {}".format(exc)
            )
            raise G1ControlOperationError(
                exc,
                recovered=recovered,
                recovery_error=recovery_error,
            ) from exc

        finally:
            _g1_finish_action(sequence)


# =========================
# 风机 / 异物清理
# 电调 PWM 控制版本
# =========================

def create_cleaning_fan():
    """
    创建异物清理电调 PWM 输出。
    CLEANING_ESC_PIN = 13 表示 GPIO13 / 物理 33 脚。
    """
    if PWMOutputDevice is None:
        raise RuntimeError(
            "gpiozero 不可用，无法初始化异物清理风机: "
            f"{GPIOZERO_IMPORT_ERROR or '未知导入错误'}"
        )

    return PWMOutputDevice(
        CLEANING_ESC_PIN,
        frequency=CLEANING_ESC_FREQ,
        initial_value=us_to_pwm_value(CLEANING_MIN_US),
    )


def us_to_pwm_value(pulse_us):
    """
    把脉宽 us 转成 gpiozero 的 value。
    50Hz 时周期是 20000us：
    1000us -> 0.05
    2000us -> 0.10
    """
    return (float(pulse_us) / 1000000.0) * float(CLEANING_ESC_FREQ)


def speed_to_us(speed_percent):
    """
    把 0~100% 油门转成 1000~2000us。
    """
    try:
        speed_percent = int(speed_percent)
    except Exception:
        speed_percent = 0

    speed_percent = max(0, min(100, speed_percent))

    pulse_us = CLEANING_MIN_US + (
        speed_percent / 100.0
    ) * (CLEANING_MAX_US - CLEANING_MIN_US)

    return speed_percent, pulse_us


def init_cleaning_fan():
    """
    初始化电调。
    程序启动后输出 1000us 最低油门信号，方便电调解锁。
    """
    global cleaning_esc

    if cleaning_esc is None:
        cleaning_esc = create_cleaning_fan()
        cleaning_esc.value = us_to_pwm_value(CLEANING_MIN_US)

        print(
            f"[INFO] Cleaning ESC initialized on GPIO{CLEANING_ESC_PIN} "
            f"/ physical pin 33, freq={CLEANING_ESC_FREQ}Hz"
        )
        print(
            f"[INFO] ESC idle signal: {CLEANING_MIN_US}us, "
            f"pwm_value={us_to_pwm_value(CLEANING_MIN_US):.4f}"
        )


def set_cleaning_speed(speed_percent):
    """
    设置异物清理风机油门。
    0% 不是断信号，而是输出 1000us 最低油门。
    """
    global cleaning_speed_percent

    init_cleaning_fan()

    speed_percent, pulse_us = speed_to_us(speed_percent)
    pwm_value = us_to_pwm_value(pulse_us)

    cleaning_speed_percent = speed_percent

    if cleaning_esc is not None:
        cleaning_esc.value = pwm_value

    print(
        f"[INFO] Cleaning ESC speed={speed_percent}% "
        f"pulse={pulse_us:.0f}us pwm_value={pwm_value:.4f}"
    )

    return speed_percent, pulse_us, pwm_value


def ramp_cleaning_speed(start_percent, target_percent):
    """
    缓慢升降油门，避免电调和电池瞬间冲击。
    """
    start_percent = int(start_percent)
    target_percent = int(target_percent)

    if start_percent == target_percent:
        return set_cleaning_speed(target_percent)

    if target_percent > start_percent:
        step = CLEANING_RAMP_STEP
    else:
        step = -CLEANING_RAMP_STEP

    current = start_percent

    while True:
        current += step

        if step > 0 and current >= target_percent:
            current = target_percent
        elif step < 0 and current <= target_percent:
            current = target_percent

        result = set_cleaning_speed(current)

        if current == target_percent:
            return result

        time.sleep(CLEANING_RAMP_DELAY)


def set_cleaning_state(enabled: bool):
    """
    保持原来的前端接口不变：
    enabled=True  -> 异物清理开启
    enabled=False -> 异物清理关闭
    """
    global is_cleaning_on

    with cleaning_lock:
        enabled = bool(enabled)

        if enabled:
            # 先给一个较高起转油门，解决低速抖动 / 一顿一顿
            set_cleaning_speed(CLEANING_START_BOOST_PERCENT)
            time.sleep(CLEANING_START_BOOST_TIME)

            # 再稳定到目标工作油门
            speed, pulse_us, pwm_value = ramp_cleaning_speed(
                CLEANING_START_BOOST_PERCENT,
                CLEANING_RUN_SPEED_PERCENT
            )

            is_cleaning_on = True

            print(
                f"[INFO] Cleaning fan ON, "
                f"speed={speed}%, pulse={pulse_us:.0f}us, pwm={pwm_value:.4f}"
            )

        else:
            # 关闭时也缓慢降到 0，减少冲击
            ramp_cleaning_speed(cleaning_speed_percent, 0)
            is_cleaning_on = False
            print("[INFO] Cleaning fan OFF")

        return is_cleaning_on


def get_cleaning_state():
    return bool(is_cleaning_on)


def stop_cleaning_fan():
    """
    停止异物清理风机。
    关机、退出、急停时调用。
    """
    global is_cleaning_on

    with cleaning_lock:
        try:
            if cleaning_esc is not None:
                cleaning_esc.value = us_to_pwm_value(CLEANING_MIN_US)
        except Exception as e:
            print(f"[WARN] cleaning_esc stop failed: {e}")

        is_cleaning_on = False
        print("[INFO] Cleaning fan stopped")


# =========================
# ROS2 节点（保持 Qt 逻辑）
# =========================
class Ros2Node(Node if Node is not None else object):
    def __init__(self, node_name="web_robot_gui_node"):
        if Node is None:
            raise RuntimeError(
                "rclpy 未安装或 ROS2 环境未正确 source，无法创建 ROS2 节点"
            )
        super().__init__(node_name)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.fan_speed_pub = self.create_publisher(Int32, "/fan_speed", 10)
        self.robot_mode_pub = self.create_publisher(Int32, "/robot_mode", 10)
        self.controller_state_sub = None
        if String is not None:
            self.controller_state_sub = self.create_subscription(
                String,
                "/controller_state",
                self.controller_state_callback,
                10,
            )
        self.get_logger().info(f"ROS2 节点 {node_name} 已启动")

    def controller_state_callback(self, msg):
        update_controller_state_from_line(msg.data)

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        self.get_logger().info(
            f"[CMD_VEL_DEBUG] 即将发布 /cmd_vel: linear_x={msg.linear.x}, angular_z={msg.angular.z}"
        )

        self.cmd_vel_pub.publish(msg)

    def publish_fan_speed(self, fan_speed):
        msg = Int32()
        msg.data = int(fan_speed)
        self.fan_speed_pub.publish(msg)
        self.get_logger().info(f"发布风扇速度: {fan_speed}")

    def publish_robot_mode(self, mode_value):
        msg = Int32()
        msg.data = int(mode_value)
        self.robot_mode_pub.publish(msg)
        self.get_logger().info(f"发布机器人模式: {mode_value}")

def ros_spin_worker():
    global ros_node
    try:
        if ros_node is not None:
            rclpy.spin(ros_node)
    except Exception as e:
        print(f"[ERROR] ROS2 spin 线程异常: {e}")


def ensure_ros_ready():
    global ros_node, ros_thread, ros_ready
    with ros_lock:
        if ros_ready:
            return

        if rclpy is None:
            raise RuntimeError("rclpy 未安装或 ROS2 环境未正确 source，无法启用机器人运动控制")

        if not rclpy.ok():
            rclpy.init(args=None)

        ros_node = Ros2Node()
        ros_thread = threading.Thread(target=ros_spin_worker, daemon=True)
        ros_thread.start()
        ros_ready = True
        print("[INFO] ROS2 control ready")


def shutdown_ros():
    global ros_node, ros_ready
    with ros_lock:
        try:
            if ros_node is not None:
                ros_node.destroy_node()
                ros_node = None
        except Exception:
            pass

        try:
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        ros_ready = False


# =========================
# 机器人控制
# =========================
def clamp_robot_speed(speed):
    return max(ROBOT_LINEAR_SPEED_MIN, min(ROBOT_LINEAR_SPEED_MAX, float(speed)))


def clamp_robot_fan_speed(speed):
    return max(ROBOT_FAN_MIN, min(ROBOT_FAN_MAX, int(speed)))


def publish_velocity(linear_x, angular_z):
    ensure_ros_ready()
    if ros_node is None:
        raise RuntimeError("ROS2 节点未初始化")
    ros_node.publish_velocity(float(linear_x), float(angular_z))


def publish_robot_fan_speed(fan_speed):
    ensure_ros_ready()
    if ros_node is None:
        raise RuntimeError("ROS2 节点未初始化")
    ros_node.publish_fan_speed(int(fan_speed))

def publish_robot_mode(mode_value):
    ensure_ros_ready()
    if ros_node is None:
        raise RuntimeError("ROS2 节点未初始化")
    ros_node.publish_robot_mode(int(mode_value))


def update_controller_state_from_line(line):
    global latest_controller_state
    global latest_controller_state_received_monotonic
    global last_controller_state_error

    try:
        state = parse_controller_state_line(line)
    except (TypeError, ValueError) as exc:
        with controller_state_lock:
            last_controller_state_error = str(exc)
        return False

    with controller_state_lock:
        latest_controller_state = state
        latest_controller_state_received_monotonic = time.monotonic()
        last_controller_state_error = ""
    return True


def get_controller_state_payload():
    with controller_state_lock:
        state = dict(latest_controller_state)
        received = latest_controller_state_received_monotonic
        parse_error = last_controller_state_error

    now = time.monotonic()
    stale = controller_state_is_stale(received, now=now)
    age = None if received is None else max(0.0, now - received)

    state.update({
        "available": bool(state),
        "stale": stale,
        "age_s": age,
        "parse_error": parse_error,
        "commanded_control_mode": get_robot_mode_name(),
        "commanded_auto_motion_mode": robot_state.get(
            "auto_motion_mode", AUTO_MOTION_STRAIGHT
        ),
        "commanded_mode_value": int(robot_state.get("published_mode_value", 0)),
    })
    return state


def get_robot_mode_name():
    mode_value = int(robot_state.get("control_mode", ROBOT_MODE_MANUAL))
    return ROBOT_MODE_NAME_MAP.get(mode_value, "manual")


def get_auto_motion_mode_name():
    return str(robot_state.get("auto_motion_mode", AUTO_MOTION_STRAIGHT))


def publish_robot_mode_state():
    mode_value = build_robot_mode_value(
        get_robot_mode_name(),
        get_auto_motion_mode_name(),
    )
    previous_mode_value = int(robot_state.get("published_mode_value", ROBOT_MODE_MANUAL))
    try:
        publish_robot_mode(mode_value)
    except Exception:
        robot_state["published_mode_value"] = previous_mode_value
        raise

    robot_state["published_mode_value"] = mode_value
    return mode_value


def set_robot_mode(mode_value):
    with robot_control_lock:
        mode_value = int(mode_value)
        if mode_value not in (ROBOT_MODE_MANUAL, ROBOT_MODE_AUTO):
            raise ValueError("无效模式，只允许 manual(0) 或 auto(1)")

        previous_control_mode = robot_state["control_mode"]
        previous_auto_motion_mode = robot_state["auto_motion_mode"]
        previous_published_mode_value = robot_state["published_mode_value"]

        try:
            robot_state["control_mode"] = mode_value
            if mode_value == ROBOT_MODE_MANUAL:
                robot_state["auto_motion_mode"] = AUTO_MOTION_STRAIGHT
            publish_robot_mode_state()
        except Exception:
            robot_state["control_mode"] = previous_control_mode
            robot_state["auto_motion_mode"] = previous_auto_motion_mode
            robot_state["published_mode_value"] = previous_published_mode_value
            raise

        return mode_value


def set_auto_motion_mode(mode_name):
    mode_name = normalize_auto_motion_mode(mode_name)
    controller_running = is_basecontroller_running()

    with robot_control_lock:
        if int(robot_state["control_mode"]) != ROBOT_MODE_AUTO:
            raise RuntimeError("自动轨迹只能在自动模式下选择")

        previous_auto_motion_mode = robot_state["auto_motion_mode"]
        previous_published_mode_value = robot_state["published_mode_value"]

        try:
            robot_state["auto_motion_mode"] = mode_name

            if controller_running:
                mode_value = publish_robot_mode_state()
            else:
                mode_value = build_robot_mode_value(
                    CONTROL_MODE_AUTO,
                    mode_name,
                )
                robot_state["published_mode_value"] = mode_value
        except Exception:
            robot_state["auto_motion_mode"] = previous_auto_motion_mode
            robot_state["published_mode_value"] = previous_published_mode_value
            raise

        return mode_name, mode_value


def is_basecontroller_running():
    with basecontroller_lock:
        return basecontroller_process is not None and basecontroller_process.poll() is None


def start_basecontroller():
    global basecontroller_process, basecontroller_log_fp, last_basecontroller_error
    with basecontroller_lock:
        if is_basecontroller_running():
            robot_state["running"] = True
            return True, "basecontroller 已在运行"

        try:
            ensure_ros_ready()

            if basecontroller_log_fp is None or basecontroller_log_fp.closed:
                basecontroller_log_fp = open(BASECONTROLLER_LOG_PATH, "ab", buffering=0)

            banner = "\n\n===== START BASECONTROLLER =====\n"
            basecontroller_log_fp.write(banner.encode("utf-8", errors="ignore"))

            basecontroller_process = subprocess.Popen(
                BASECONTROLLER_CMD,
                cwd=str(BASE_DIR),
                stdout=basecontroller_log_fp,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid if os.name != "nt" else None,
                env=os.environ.copy(),
            )

            time.sleep(1.0)
            if basecontroller_process.poll() is not None:
                last_basecontroller_error = "basecontroller 启动失败:\n" + tail_log(BASECONTROLLER_LOG_PATH, 120)
                basecontroller_process = None
                robot_state["running"] = False
                raise RuntimeError(last_basecontroller_error)

            last_basecontroller_error = ""
            robot_state["running"] = True
            # 启动成功后，把当前模式与风扇速度同步到底层
            try:
                publish_robot_mode_state()
            except Exception as e:
                print(f"[WARN] 同步机器人模式失败: {e}")

            try:
                publish_robot_fan_speed(int(robot_state["fan_speed"]))
            except Exception as e:
                print(f"[WARN] 同步风扇速度失败: {e}")

            print("[INFO] basecontroller started")
            return True, "机器人底层控制已启动"
        except Exception as e:
            last_basecontroller_error = str(e)
            robot_state["running"] = False
            return False, f"启动失败: {e}"


def stop_basecontroller():
    global basecontroller_process, basecontroller_log_fp, last_basecontroller_error
    with basecontroller_lock:
        try:
            stop_robot_motion()
        except Exception:
            pass

        if basecontroller_process is None:
            robot_state["running"] = False
            return True, "basecontroller 未运行"

        try:
            terminate_process_group(basecontroller_process, timeout=3.0)
            basecontroller_process = None
            robot_state["running"] = False
            robot_state["last_direction"] = "stop"

            if basecontroller_log_fp and (not basecontroller_log_fp.closed):
                try:
                    basecontroller_log_fp.flush()
                except Exception:
                    pass

            print("[INFO] basecontroller stopped")
            return True, "机器人底层控制已停止"
        except Exception as e:
            last_basecontroller_error = str(e)
            return False, f"停止失败: {e}"


def set_robot_speed(speed):
    with robot_control_lock:
        robot_state["linear_speed"] = clamp_robot_speed(speed)
        return robot_state["linear_speed"]


def set_robot_fan_speed(speed):
    with robot_control_lock:
        fan_speed = clamp_robot_fan_speed(speed)
        robot_state["fan_speed"] = fan_speed
        publish_robot_fan_speed(fan_speed)
        return fan_speed


def step_robot_fan_speed(delta_step):
    with robot_control_lock:
        current = int(robot_state["fan_speed"])
        target = current + int(delta_step) * ROBOT_FAN_STEP
        target = clamp_robot_fan_speed(target)
        robot_state["fan_speed"] = target
        publish_robot_fan_speed(target)
        return target


def reset_robot_fan_speed():
    with robot_control_lock:
        robot_state["fan_speed"] = 0
        publish_robot_fan_speed(0)
        return 0


def move_robot(direction, turn_scale=1.0):
    controller_running = is_basecontroller_running()

    with robot_control_lock:
        if not controller_running:
            robot_state["running"] = False
            raise RuntimeError("机器人底层控制未启动")

        direction = str(direction).strip().lower()
        linear_speed = float(robot_state["linear_speed"])
        angular_speed = float(robot_state["angular_speed"])
        turn_scale = float(turn_scale)
        control_mode = int(robot_state["control_mode"])

        if (
            control_mode == ROBOT_MODE_AUTO
            and direction not in ("forward", "backward", "stop")
        ):
            raise RuntimeError("自动模式下只允许前进、后退和停止")

        if direction == "forward":
            publish_velocity(linear_speed, 0.0)
        elif direction == "left":
            publish_velocity(0.0, angular_speed * turn_scale)
        elif direction == "stop":
            publish_velocity(0.0, 0.0)
        elif direction == "right":
            publish_velocity(0.0, -angular_speed * turn_scale)
        elif direction == "backward":
            publish_velocity(-linear_speed, 0.0)
        else:
            raise ValueError(f"未知方向: {direction}")

        robot_state["last_direction"] = direction

def stop_robot_motion():
    with robot_control_lock:
        publish_velocity(0.0, 0.0)
        robot_state["last_direction"] = "stop"


# =========================
# MediaMTX
# =========================
def find_mediamtx_bin():
    envp = os.environ.get(MEDIAMTX_BIN_ENV, "").strip()
    if envp:
        p = Path(envp).expanduser()
        if p.exists() and os.access(str(p), os.X_OK):
            return str(p)

    candidates = [
        shutil.which("mediamtx"),
        str((BASE_DIR / "bin" / "mediamtx").resolve()),
        str((BASE_DIR / "mediamtx").resolve()),
        "/usr/local/bin/mediamtx",
        "/usr/bin/mediamtx",
    ]
    for c in candidates:
        if not c:
            continue
        p = Path(c)
        if p.exists() and os.access(str(p), os.X_OK):
            return str(p)
    return None


def build_mediamtx_config():
    """
    构造 MediaMTX 配置。

    重要兼容性规则：
    - 当前局域网 IP 由 webrtcIPsFromInterfaces 自动获取；
    - webrtcAdditionalHosts 必须保持为空。部分 MediaMTX 版本在这里填入
      本机 LAN IP 时，会出现 "deadline exceeded while waiting connection"；
    - 仍保留网络 watcher：IP 改变后重启 MediaMTX，使其重新读取网卡地址。
    """
    observed_hosts = get_interface_ipv4_addresses()

    print(
        "[INFO] MediaMTX WebRTC interface IPs (observed): "
        + (", ".join(observed_hosts) if observed_hosts else "(none)")
    )
    print("[INFO] MediaMTX WebRTC additional hosts: (empty; interface discovery is enabled)")

    return f"""logLevel: {MEDIAMTX_LOG_LEVEL}

rtsp: yes
protocols: [tcp]
rtspAddress: :8554
rtpAddress: :10000
rtcpAddress: :10001

rtmp: no
hls: no
srt: no

webrtc: yes
webrtcAddress: :{WEBRTC_HTTP_PORT}
webrtcEncryption: no
webrtcAllowOrigin: '*'
webrtcLocalUDPAddress: :{WEBRTC_ICE_UDP_PORT}
webrtcLocalTCPAddress: :{WEBRTC_ICE_TCP_PORT}

# 让 MediaMTX 从当前网卡读取 IP。换网络后，app 内 watcher 会重启它以刷新地址。
webrtcIPsFromInterfaces: yes
webrtcIPsFromInterfacesList: []

# 不要把当前局域网 IP 写到这里；保留空列表。
webrtcAdditionalHosts: []

# 局域网固定端口直连不使用外部 STUN/TURN。
webrtcICEServers2: []

paths:
  cam:
    source: publisher
"""


def write_mediamtx_config():
    MEDIAMTX_CONFIG_PATH.write_text(build_mediamtx_config(), encoding="utf-8")
    txt = MEDIAMTX_CONFIG_PATH.read_text(encoding="utf-8", errors="ignore")

    if "source: publisher" not in txt:
        raise RuntimeError("mediamtx.yml 未写入 publisher 配置")
    if "webrtcIPsFromInterfaces: yes" not in txt:
        raise RuntimeError("mediamtx.yml 未启用网卡 IP 自动发现")
    if "webrtcAdditionalHosts: []" not in txt:
        raise RuntimeError("mediamtx.yml 的 webrtcAdditionalHosts 必须为空")


def mediamtx_is_running():
    global mediamtx_process
    return mediamtx_process is not None and mediamtx_process.poll() is None


def start_mediamtx():
    global mediamtx_process, mediamtx_log_fp, last_stream_error

    with mediamtx_lock:
        if mediamtx_is_running():
            return

        binp = find_mediamtx_bin()
        if not binp:
            raise RuntimeError("未找到 mediamtx 可执行文件。请安装或设置 MEDIAMTX_BIN。")

        kill_stray_mediamtx()
        write_mediamtx_config()

        if mediamtx_log_fp is None or mediamtx_log_fp.closed:
            mediamtx_log_fp = open(MEDIAMTX_LOG_PATH, "ab", buffering=0)

        print(f"[INFO] Starting MediaMTX: {binp}")
        mediamtx_process = subprocess.Popen(
            [binp, str(MEDIAMTX_CONFIG_PATH)],
            cwd=str(BASE_DIR),
            stdout=mediamtx_log_fp,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid if os.name != "nt" else None,
        )

        time.sleep(1.2)
        if mediamtx_process.poll() is not None:
            last_stream_error = "MediaMTX 启动失败:\n" + tail_log(MEDIAMTX_LOG_PATH, 120)
            raise RuntimeError(last_stream_error)

        last_stream_error = ""
        print("[INFO] MediaMTX started")


def stop_mediamtx():
    global mediamtx_process, mediamtx_log_fp
    with mediamtx_lock:
        if mediamtx_process is not None:
            terminate_process_group(mediamtx_process, timeout=3.0)
        mediamtx_process = None

        if mediamtx_log_fp and (not mediamtx_log_fp.closed):
            try:
                mediamtx_log_fp.flush()
            except Exception:
                pass

        with whep_sessions_lock:
            whep_sessions.clear()

        print("[INFO] MediaMTX stopped")



# =========================
# WebRTC 网络地址变化监控
# =========================
def restart_streamer_for_webrtc_network_change():
    """
    IP 切换后重新生成 MediaMTX 配置并重新建立 RTSP publisher。
    网络切换本身会中断 WebRTC，因此这里允许短暂重连；录像进行中不调用，
    由 watcher 等待录制结束后再执行。
    """
    global last_stream_error

    with video_state_lock:
        width = video_state["width"]
        height = video_state["height"]
        fps = video_state["fps"]
        bitrate = video_state["bitrate"]

    try:
        print("[INFO] WebRTC network changed; restarting MediaMTX and publisher")
        stop_publisher()
        stop_mediamtx()
        start_mediamtx()
        start_publisher(width, height, fps, bitrate)
        last_stream_error = ""
        print("[INFO] WebRTC network reconfiguration completed")
        return True
    except Exception as e:
        last_stream_error = f"网络地址变化后重启 WebRTC 流失败: {e}"
        print(f"[ERROR] {last_stream_error}")
        return False


def webrtc_network_watch_worker():
    """
    监控当前可公布的局域网地址。检测到稳定的新地址后自动刷新 MediaMTX。
    无地址阶段（例如 Wi-Fi 正在重连）不重启，避免使用空候选地址覆盖旧配置。
    """
    global last_webrtc_advertised_hosts, webrtc_network_reconfigure_pending

    while not webrtc_network_watch_stop_event.wait(WEBRTC_NETWORK_WATCH_INTERVAL_SECONDS):
        current_hosts = tuple(get_webrtc_advertised_hosts())

        if not current_hosts:
            continue

        with webrtc_network_watch_lock:
            if current_hosts != last_webrtc_advertised_hosts:
                old_hosts = last_webrtc_advertised_hosts
                last_webrtc_advertised_hosts = current_hosts
                webrtc_network_reconfigure_pending = True
                print(
                    "[INFO] WebRTC advertised hosts changed: "
                    f"{list(old_hosts)} -> {list(current_hosts)}"
                )

            pending = webrtc_network_reconfigure_pending

        if not pending:
            continue

        # 录像读取同一条 RTSP 流；等待录像结束，避免生成损坏文件。
        if recording_is_running():
            print("[INFO] WebRTC network change pending; waiting for recording to finish")
            continue

        # 流服务尚未启动时，后续 ensure_streamer_ready() 会写入最新配置。
        if not mediamtx_is_running() and not publisher_is_running():
            with webrtc_network_watch_lock:
                webrtc_network_reconfigure_pending = False
            continue

        if restart_streamer_for_webrtc_network_change():
            with webrtc_network_watch_lock:
                webrtc_network_reconfigure_pending = False


def start_webrtc_network_watcher():
    global webrtc_network_watch_thread, last_webrtc_advertised_hosts

    with webrtc_network_watch_lock:
        if (
            webrtc_network_watch_thread is not None
            and webrtc_network_watch_thread.is_alive()
        ):
            return

        last_webrtc_advertised_hosts = tuple(get_webrtc_advertised_hosts())
        webrtc_network_watch_stop_event.clear()
        webrtc_network_watch_thread = threading.Thread(
            target=webrtc_network_watch_worker,
            name="webrtc-network-watch",
            daemon=True,
        )
        webrtc_network_watch_thread.start()

    print(
        "[INFO] WebRTC network watcher started; initial hosts: "
        + (", ".join(last_webrtc_advertised_hosts) if last_webrtc_advertised_hosts else "(none)")
    )


def stop_webrtc_network_watcher():
    global webrtc_network_watch_thread

    webrtc_network_watch_stop_event.set()

    thread = None
    with webrtc_network_watch_lock:
        thread = webrtc_network_watch_thread
        webrtc_network_watch_thread = None

    if thread is not None and thread is not threading.current_thread():
        try:
            thread.join(timeout=1.5)
        except Exception:
            pass


# =========================
# Publisher
# =========================
def publisher_is_running():
    global publisher_process
    return publisher_process is not None and publisher_process.poll() is None


def build_publisher_env():
    require_camera_stack()

    env = {
        "PATH": f"{CAMERA_STACK_ROOT / 'bin'}:/usr/bin:/bin",
        "HOME": os.environ.get("HOME", str(BASE_DIR)),
        "LANG": os.environ.get("LANG", "C.UTF-8"),
        "LD_LIBRARY_PATH": f"{CAMERA_LIB_DIR}:/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu",
        "LIBCAMERA_IPA_MODULE_PATH": str(CAMERA_IPA_MODULE_DIR),
        "LIBCAMERA_IPA_PROXY_PATH": str(CAMERA_IPA_PROXY_DIR),
    }
    return env


def build_publisher_command(width, height, fps, bitrate):
    width = int(width)
    height = int(height)
    fps = int(fps)
    bitrate = int(bitrate)

    left = [
        str(CAMERA_BIN),
        "-n",
        "--width", str(width),
        "--height", str(height),
        "--framerate", str(fps),
        "--codec", "libav",
        "--libav-video-codec", "libx264",
        "--profile", "baseline",
        "--low-latency",
        "--libav-video-codec-opts", "bf=0;preset=ultrafast;tune=zerolatency",
        "--libav-format", "h264",
        "--bitrate", str(bitrate),
        "-t", "0",
        "-o", "-",
    ]

    right = [
        "gst-launch-1.0", "-q",
        "fdsrc", "fd=0", "do-timestamp=true", "!",
        "h264parse", "config-interval=-1", "!",
        "video/x-h264,stream-format=byte-stream,alignment=au", "!",
        "rtspclientsink",
        "location=rtsp://127.0.0.1:8554/cam",
        "protocols=tcp",
        "latency=0",
    ]

    return f"{shlex.join(left)} | {shlex.join(right)}"


def start_publisher(width, height, fps, bitrate):
    global publisher_process, publisher_log_fp, last_publisher_error

    with publisher_lock:
        if publisher_is_running():
            return

        if not mediamtx_is_running():
            raise RuntimeError("MediaMTX 未运行，无法启动 publisher")

        env = build_publisher_env()
        cmdline = build_publisher_command(width, height, fps, bitrate)

        if publisher_log_fp is None or publisher_log_fp.closed:
            publisher_log_fp = open(PUBLISHER_LOG_PATH, "ab", buffering=0)

        banner = f"\n\n===== START RPICAM PIPELINE =====\n{cmdline}\n"
        publisher_log_fp.write(banner.encode("utf-8", errors="ignore"))

        publisher_process = subprocess.Popen(
            ["bash", "-c", cmdline],
            cwd=str(BASE_DIR),
            stdout=publisher_log_fp,
            stderr=subprocess.STDOUT,
            env=env,
            preexec_fn=os.setsid if os.name != "nt" else None,
        )

        time.sleep(2.0)
        if publisher_process.poll() is not None:
            last_publisher_error = "Publisher 启动失败:\n" + tail_log(PUBLISHER_LOG_PATH, 120)
            publisher_process = None
            raise RuntimeError(last_publisher_error)

        last_publisher_error = ""
        print("[INFO] Publisher started")


def stop_publisher():
    global publisher_process, publisher_log_fp
    with publisher_lock:
        if publisher_process is not None:
            terminate_process_group(publisher_process, timeout=3.0)
        publisher_process = None

        if publisher_log_fp and (not publisher_log_fp.closed):
            try:
                publisher_log_fp.flush()
            except Exception:
                pass

        print("[INFO] Publisher stopped")


# =========================
# 本地拍照/录制
# =========================
def find_ffmpeg_bin():
    envp = os.environ.get(FFMPEG_BIN_ENV, "").strip()
    if envp:
        p = Path(envp).expanduser()
        if p.exists() and os.access(str(p), os.X_OK):
            return str(p)

    p = shutil.which("ffmpeg")
    if p:
        return p
    return None


def build_rtsp_url():
    return "rtsp://127.0.0.1:8554/cam"


# =========================
# 异物识别显示
# =========================
def get_detection_state():
    """
    返回前端 canvas 叠加框需要的识别状态。
    注意：这里不返回图片、不做 JPEG 编码，只返回坐标，降低视频链路延迟。
    """
    ensure_detection_worker()

    now = time.time()
    with detection_lock:
        if last_detection_frame_time > 0:
            frame_age = max(0.0, now - float(last_detection_frame_time))
        else:
            frame_age = -1.0

        stale = (frame_age < 0) or (frame_age > float(DETECTION_STALE_SECONDS))

        return {
            "detected": bool(is_foreign_object_detected) and not stale,
            "boxes": [dict(b) for b in last_detection_boxes_list] if not stale else [],
            "box_count": int(last_detection_boxes) if not stale else 0,
            "detection_count": int(detection_count),
            "lost_count": int(lost_count),
            "error": last_detection_error,
            "frames": int(detection_total_frames),
            "last_frame_time": float(last_detection_frame_time),
            "frame_age": frame_age,
            "frame_width": int(last_detection_frame_width),
            "frame_height": int(last_detection_frame_height),
            "infer_ms": float(last_detection_infer_ms),
            "worker_alive": bool(last_detection_worker_alive),
            "stale": bool(stale),
            "mode": f"webrtc_canvas_overlay_{DETECTION_BACKEND}",
            "backend": DETECTION_BACKEND,
            "yolo_model": str(YOLO_MODEL_PATH) if DETECTION_BACKEND == "yolov8" else "",
            "yolo_conf": float(YOLO_CONF) if DETECTION_BACKEND == "yolov8" else None,
            "yolo_imgsz": int(YOLO_IMGSZ) if DETECTION_BACKEND == "yolov8" else None,
        }


def _set_detection_error(msg):
    global last_detection_error
    with detection_lock:
        last_detection_error = str(msg or "")


def _set_detection_worker_alive(value: bool):
    global last_detection_worker_alive
    with detection_lock:
        last_detection_worker_alive = bool(value)


def _store_detection_result(frame_w, frame_h, boxes, infer_ms):
    """
    保存最新识别坐标。boxes 使用原始视频帧坐标，前端按 frame_width/frame_height 映射到 canvas。
    支持两种输入：
    1. 旧版 OpenCV 检测的 (x, y, w, h) 元组；
    2. YOLOv8 检测的 dict，包含 x/y/w/h、label、conf、class_id 等字段。
    """
    global detection_count, lost_count, is_foreign_object_detected
    global last_detection_boxes, last_detection_boxes_list
    global last_detection_frame_width, last_detection_frame_height
    global last_detection_frame_time, detection_total_frames, last_detection_infer_ms, last_detection_error

    frame_w = int(frame_w or 0)
    frame_h = int(frame_h or 0)
    if frame_w <= 0 or frame_h <= 0:
        return

    clean_boxes = []
    for i, item in enumerate(boxes or []):
        if isinstance(item, dict):
            x = item.get("x", item.get("x1", 0))
            y = item.get("y", item.get("y1", 0))

            if "w" in item and "h" in item:
                w = item.get("w", 1)
                h = item.get("h", 1)
            else:
                x2 = item.get("x2", float(x) + 1)
                y2 = item.get("y2", float(y) + 1)
                w = float(x2) - float(x)
                h = float(y2) - float(y)

            label = item.get("label", "孔洞")
            class_name = item.get("class_name", "孔洞")
            class_id = item.get("class_id", -1)
            conf = item.get("conf", None)
        else:
            x, y, w, h = item
            label = "孔洞"
            class_name = "孔洞"
            class_id = -1
            conf = None

        x = int(max(0, min(frame_w - 1, int(float(x)))))
        y = int(max(0, min(frame_h - 1, int(float(y)))))
        w = int(max(1, min(frame_w - x, int(float(w)))))
        h = int(max(1, min(frame_h - y, int(float(h)))))

        box = {
            "id": int(i),
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "x1": x,
            "y1": y,
            "x2": x + w,
            "y2": y + h,
            "label": str(label),
            "class_name": str(class_name),
            "class_id": int(class_id) if str(class_id).lstrip("-").isdigit() else -1,
        }

        if conf is not None:
            try:
                box["conf"] = round(float(conf), 4)
            except Exception:
                pass

        if isinstance(item, dict):
            if "box_type" in item:
                box["box_type"] = str(item.get("box_type") or "")
            if "polygon" in item and isinstance(item.get("polygon"), list):
                box["polygon"] = item.get("polygon")

        clean_boxes.append(box)

    found_object = len(clean_boxes) > 0

    with detection_lock:
        if found_object:
            detection_count += 1
            lost_count = 0
        else:
            lost_count += 1
            detection_count = 0

        if detection_count >= DETECTION_CONFIRM_FRAMES:
            is_foreign_object_detected = True

        if lost_count >= DETECTION_LOST_FRAMES:
            is_foreign_object_detected = False

        last_detection_boxes = len(clean_boxes)
        last_detection_boxes_list = clean_boxes
        last_detection_frame_width = frame_w
        last_detection_frame_height = frame_h
        last_detection_frame_time = time.time()
        detection_total_frames += 1
        last_detection_infer_ms = float(infer_ms)
        last_detection_error = ""

def open_detection_capture():
    """
    从 MediaMTX 的 RTSP 流取帧做识别，不直接抢占摄像头。
    识别线程只输出坐标，视频画面仍由浏览器直接播放 WebRTC。
    """
    if cv2 is None:
        raise RuntimeError(f"OpenCV 未安装或导入失败: {CV_IMPORT_ERROR}")

    # OpenCV/FFmpeg 选项格式：key;value|key;value
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "rtsp_transport;tcp|"
        "fflags;nobuffer|"
        "flags;low_delay|"
        "max_delay;0|"
        "probesize;32768|"
        "analyzeduration;0"
    )

    cap = cv2.VideoCapture(build_rtsp_url(), cv2.CAP_FFMPEG)

    for prop_name, value in [
        ("CAP_PROP_BUFFERSIZE", 1),
        ("CAP_PROP_OPEN_TIMEOUT_MSEC", 2000),
        ("CAP_PROP_READ_TIMEOUT_MSEC", 2000),
    ]:
        try:
            prop = getattr(cv2, prop_name, None)
            if prop is not None:
                cap.set(prop, value)
        except Exception:
            pass

    return cap



def get_yolo_model():
    """
    懒加载 YOLOv8 模型。第一次识别时加载 best.pt，之后复用同一个模型。
    """
    global yolo_model

    if YOLO is None:
        raise RuntimeError(f"ultralytics 未安装或导入失败: {YOLO_IMPORT_ERROR}")

    if not YOLO_MODEL_PATH.exists():
        raise RuntimeError(f"未找到 YOLO 权重文件: {YOLO_MODEL_PATH}")

    with yolo_model_lock:
        if yolo_model is None:
            print(f"[INFO] Loading YOLOv8 model: {YOLO_MODEL_PATH}")

            if YOLO_NUM_THREADS > 0:
                try:
                    import torch
                    torch.set_num_threads(int(YOLO_NUM_THREADS))
                    torch.set_num_interop_threads(max(1, int(YOLO_NUM_THREADS)))
                    print(f"[INFO] Torch CPU threads set to {YOLO_NUM_THREADS}")
                except Exception as e:
                    print(f"[WARN] Torch thread setting skipped: {e}")

            yolo_model = YOLO(str(YOLO_MODEL_PATH))

            try:
                yolo_model.fuse()
            except Exception as e:
                print(f"[WARN] YOLO fuse skipped: {e}")

            print(f"[INFO] YOLOv8 model loaded, names={getattr(yolo_model, 'names', None)}")

        return yolo_model

def detect_foreign_object_boxes(frame):
    """
    单帧孔洞识别，只返回坐标，不在图片上画框。

    支持两种 YOLOv8 输出：
    1. 普通检测模型：result.boxes
    2. OBB 旋转框模型：result.obb

    你的 best.pt 是 YOLOv8n-obb，因此必须读取 result.obb；
    前端目前使用 canvas 绘制普通矩形框，所以这里把 OBB 旋转框转换为外接矩形 x/y/w/h。
    """
    if cv2 is None or np is None or frame is None:
        return []

    frame_h, frame_w = frame.shape[:2]

    if DETECTION_BACKEND != "yolov8":
        return []

    model = get_yolo_model()

    proc = frame
    scale = 1.0

    if YOLO_PROCESS_WIDTH > 0 and frame_w > YOLO_PROCESS_WIDTH:
        scale = YOLO_PROCESS_WIDTH / float(frame_w)
        proc_h = max(1, int(frame_h * scale))
        proc = cv2.resize(frame, (YOLO_PROCESS_WIDTH, proc_h), interpolation=cv2.INTER_AREA)

    results = model.predict(
        source=proc,
        imgsz=int(YOLO_IMGSZ),
        conf=float(YOLO_CONF),
        iou=float(YOLO_IOU),
        device=YOLO_DEVICE,
        verbose=False,
    )

    boxes_out = []

    def _get_class_name(names, cls_id):
        if isinstance(names, dict):
            return names.get(cls_id, str(cls_id))
        if isinstance(names, (list, tuple)) and 0 <= cls_id < len(names):
            return names[cls_id]
        return str(cls_id)

    def _append_box(x1, y1, x2, y2, conf, cls_id, names, polygon=None, box_type="box"):
        """
        把模型输出坐标映射回原始视频帧，并加入 boxes_out。
        polygon 为 OBB 的 4 点坐标，可选；前端不支持多边形时仍会用外接矩形显示。
        """
        nonlocal boxes_out

        if scale != 1.0:
            x1 /= scale
            y1 /= scale
            x2 /= scale
            y2 /= scale

            if polygon:
                polygon = [
                    {
                        "x": float(p["x"]) / scale,
                        "y": float(p["y"]) / scale,
                    }
                    for p in polygon
                ]

        x1 = max(0, min(frame_w - 1, float(x1)))
        y1 = max(0, min(frame_h - 1, float(y1)))
        x2 = max(0, min(frame_w - 1, float(x2)))
        y2 = max(0, min(frame_h - 1, float(y2)))

        # 防止坐标顺序异常
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1

        w = max(1, x2 - x1)
        h = max(1, y2 - y1)

        class_name = _get_class_name(names, int(cls_id))
        label_name = YOLO_FORCE_LABEL or class_name or "孔洞"

        item = {
            "x": int(round(x1)),
            "y": int(round(y1)),
            "w": int(round(w)),
            "h": int(round(h)),
            "x1": int(round(x1)),
            "y1": int(round(y1)),
            "x2": int(round(x1 + w)),
            "y2": int(round(y1 + h)),
            "label": f"{label_name} {float(conf):.2f}",
            "class_name": str(label_name),
            "class_id": int(cls_id),
            "conf": round(float(conf), 4),
            "box_type": str(box_type),
        }

        if polygon:
            # 限制 OBB 多边形点在画面内
            clean_poly = []
            for p in polygon:
                px = max(0, min(frame_w - 1, float(p["x"])))
                py = max(0, min(frame_h - 1, float(p["y"])))
                clean_poly.append({"x": int(round(px)), "y": int(round(py))})
            item["polygon"] = clean_poly

        boxes_out.append(item)

    for result in results:
        names = getattr(result, "names", None) or getattr(model, "names", {}) or {}

        # 1) 优先读取 OBB 旋转框。你的模型是 YOLOv8n-obb，结果在这里。
        obb = getattr(result, "obb", None)
        if obb is not None:
            try:
                obb_xyxy = getattr(obb, "xyxy", None)
                obb_conf = getattr(obb, "conf", None)
                obb_cls = getattr(obb, "cls", None)
                obb_poly = getattr(obb, "xyxyxyxy", None)

                if obb_xyxy is not None and obb_conf is not None and obb_cls is not None:
                    xyxy_arr = obb_xyxy.detach().cpu().numpy()
                    conf_arr = obb_conf.detach().cpu().numpy()
                    cls_arr = obb_cls.detach().cpu().numpy()

                    poly_arr = None
                    if obb_poly is not None:
                        try:
                            poly_arr = obb_poly.detach().cpu().numpy()
                        except Exception:
                            poly_arr = None

                    for i in range(len(xyxy_arr)):
                        x1, y1, x2, y2 = [float(v) for v in xyxy_arr[i].tolist()]
                        conf = float(conf_arr[i])
                        cls_id = int(cls_arr[i])

                        polygon = None
                        if poly_arr is not None and i < len(poly_arr):
                            pts = poly_arr[i]
                            # 常见形状为 (4, 2)
                            try:
                                polygon = [
                                    {"x": float(pt[0]), "y": float(pt[1])}
                                    for pt in pts
                                ]
                            except Exception:
                                polygon = None

                        _append_box(
                            x1, y1, x2, y2,
                            conf=conf,
                            cls_id=cls_id,
                            names=names,
                            polygon=polygon,
                            box_type="obb",
                        )
            except Exception as e:
                print(f"[WARN] 解析 YOLO OBB 结果失败: {e}")

        # 2) 同时兼容普通 boxes 模型。
        boxes = getattr(result, "boxes", None)
        if boxes is not None:
            try:
                for b in boxes:
                    xyxy = b.xyxy[0].detach().cpu().numpy().tolist()
                    x1, y1, x2, y2 = [float(v) for v in xyxy]

                    conf = float(b.conf[0]) if getattr(b, "conf", None) is not None else 0.0
                    cls_id = int(b.cls[0]) if getattr(b, "cls", None) is not None else -1

                    _append_box(
                        x1, y1, x2, y2,
                        conf=conf,
                        cls_id=cls_id,
                        names=names,
                        polygon=None,
                        box_type="box",
                    )
            except Exception as e:
                print(f"[WARN] 解析 YOLO boxes 结果失败: {e}")

    # 去重：某些模型/版本可能同时给 boxes 和 obb，这里按近似位置去重。
    unique = []
    seen = set()
    for b in boxes_out:
        key = (
            int(round(b["x"] / 4)),
            int(round(b["y"] / 4)),
            int(round(b["w"] / 4)),
            int(round(b["h"] / 4)),
            b.get("class_id", -1),
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(b)

    unique.sort(key=lambda b: (b["y"], b["x"]))
    return unique

def detection_worker_loop():
    """
    后台异物识别线程：
    1. 从 RTSP 读取最新帧；
    2. 按 DETECTION_WORKER_FPS 做识别；
    3. 只更新 boxes 坐标；
    4. 不向浏览器推送图片，不影响 WebRTC 原始视频播放。
    """
    cap = None
    frame_interval = 1.0 / max(1, int(DETECTION_WORKER_FPS))
    last_process_time = 0.0

    _set_detection_worker_alive(True)

    try:
        while not detection_stop_event.is_set():
            if cv2 is None or np is None:
                _set_detection_error(f"OpenCV 未安装或导入失败: {CV_IMPORT_ERROR}")
                time.sleep(1.0)
                continue

            if DETECTION_BACKEND == "yolov8":
                if YOLO is None:
                    _set_detection_error(f"ultralytics 未安装或导入失败: {YOLO_IMPORT_ERROR}")
                    time.sleep(1.0)
                    continue
                if not YOLO_MODEL_PATH.exists():
                    _set_detection_error(f"未找到 YOLO 权重文件: {YOLO_MODEL_PATH}")
                    time.sleep(1.0)
                    continue

            try:
                if cap is None or not cap.isOpened():
                    ensure_streamer_ready()
                    cap = open_detection_capture()
                    if not cap.isOpened():
                        raise RuntimeError("无法打开 RTSP 识别视频流")

                ok, frame = cap.read()
                if not ok or frame is None:
                    raise RuntimeError("读取 RTSP 帧失败，正在重连")

                now = time.time()
                if now - last_process_time < frame_interval:
                    # 持续读帧但不每帧都识别，避免 RTSP 缓冲积压，同时控制 CPU 占用。
                    time.sleep(0.002)
                    continue

                last_process_time = now

                frame_h, frame_w = frame.shape[:2]
                t0 = time.perf_counter()
                boxes = detect_foreign_object_boxes(frame)
                infer_ms = (time.perf_counter() - t0) * 1000.0
                _store_detection_result(frame_w, frame_h, boxes, infer_ms)

            except Exception as e:
                _set_detection_error(e)
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
                cap = None
                time.sleep(0.25)

    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass
        _set_detection_worker_alive(False)


def ensure_detection_worker():
    """确保后台识别线程已启动。"""
    global detection_thread

    if cv2 is None or np is None:
        _set_detection_error(f"OpenCV 未安装或导入失败: {CV_IMPORT_ERROR}")
        return False

    if DETECTION_BACKEND == "yolov8":
        if YOLO is None:
            _set_detection_error(f"ultralytics 未安装或导入失败: {YOLO_IMPORT_ERROR}")
            return False
        if not YOLO_MODEL_PATH.exists():
            _set_detection_error(f"未找到 YOLO 权重文件: {YOLO_MODEL_PATH}")
            return False

    with detection_thread_lock:
        if detection_thread is not None and detection_thread.is_alive():
            return True

        detection_stop_event.clear()
        detection_thread = threading.Thread(
            target=detection_worker_loop,
            name="foreign-object-detection-worker",
            daemon=True,
        )
        detection_thread.start()
        return True


def stop_detection_worker(timeout=1.5):
    """程序退出或安全关机时停止后台识别线程。"""
    global detection_thread

    detection_stop_event.set()
    with detection_thread_lock:
        t = detection_thread
        detection_thread = None

    if t is not None and t.is_alive():
        try:
            t.join(timeout=float(timeout))
        except Exception:
            pass


# =========================
# YOLO 自动避障
# =========================
def build_obstacle_avoidance_danger_zone(frame_width, frame_height):
    frame_width = int(frame_width or 0)
    frame_height = int(frame_height or 0)
    if frame_width <= 0 or frame_height <= 0:
        return None

    danger_width = frame_width * OBSTACLE_AVOIDANCE_DANGER_WIDTH_RATIO
    x1 = int(round((frame_width - danger_width) / 2.0))
    x2 = int(round(x1 + danger_width))
    split_x = int(round((x1 + x2) / 2.0))
    y1 = int(round(frame_height * (1.0 - OBSTACLE_AVOIDANCE_DANGER_HEIGHT_RATIO)))
    y2 = frame_height

    return {
        "frame_width": frame_width,
        "frame_height": frame_height,
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "split_x": split_x,
        "left": {"x1": x1, "y1": y1, "x2": split_x, "y2": y2},
        "right": {"x1": split_x, "y1": y1, "x2": x2, "y2": y2},
    }


def rectangles_overlap(box, region):
    try:
        box_x1 = float(box.get("x", box.get("x1", 0)))
        box_y1 = float(box.get("y", box.get("y1", 0)))
        box_x2 = float(box.get("x2", box_x1 + float(box.get("w", 0))))
        box_y2 = float(box.get("y2", box_y1 + float(box.get("h", 0))))
        return (
            min(box_x2, float(region["x2"])) > max(box_x1, float(region["x1"]))
            and min(box_y2, float(region["y2"])) > max(box_y1, float(region["y1"]))
        )
    except (AttributeError, TypeError, ValueError):
        return False


def evaluate_obstacle_avoidance(boxes, frame_width, frame_height):
    danger_zone = build_obstacle_avoidance_danger_zone(frame_width, frame_height)
    if danger_zone is None:
        return {
            "action": "waiting",
            "direction": "none",
            "left_overlap": False,
            "right_overlap": False,
            "overlap_box_count": 0,
            "danger_zone": None,
        }

    left_overlap = False
    right_overlap = False
    overlap_box_count = 0

    for box in boxes or []:
        overlaps_left = rectangles_overlap(box, danger_zone["left"])
        overlaps_right = rectangles_overlap(box, danger_zone["right"])
        if overlaps_left or overlaps_right:
            overlap_box_count += 1
        left_overlap = left_overlap or overlaps_left
        right_overlap = right_overlap or overlaps_right

    if left_overlap and right_overlap:
        action = AUTO_MOTION_SPIRAL_LEFT
        direction = "left"
    elif left_overlap:
        action = AUTO_MOTION_SPIRAL_RIGHT
        direction = "right"
    elif right_overlap:
        action = AUTO_MOTION_SPIRAL_LEFT
        direction = "left"
    else:
        action = AUTO_MOTION_STRAIGHT
        direction = "straight"

    return {
        "action": action,
        "direction": direction,
        "left_overlap": left_overlap,
        "right_overlap": right_overlap,
        "overlap_box_count": overlap_box_count,
        "danger_zone": danger_zone,
    }


def get_obstacle_avoidance_enabled():
    with obstacle_avoidance_lock:
        return bool(is_obstacle_avoidance_enabled)


def _update_obstacle_avoidance_state(**kwargs):
    with obstacle_avoidance_lock:
        obstacle_avoidance_state.update(kwargs)


def get_obstacle_avoidance_state():
    with obstacle_avoidance_lock:
        state = dict(obstacle_avoidance_state)
        danger_zone = state.get("danger_zone")
        if isinstance(danger_zone, dict):
            state["danger_zone"] = {
                key: dict(value) if isinstance(value, dict) else value
                for key, value in danger_zone.items()
            }
        state["enabled"] = bool(is_obstacle_avoidance_enabled)

    state.update({
        "worker_alive": bool(
            obstacle_avoidance_thread is not None
            and obstacle_avoidance_thread.is_alive()
        ),
        "danger_width_ratio": OBSTACLE_AVOIDANCE_DANGER_WIDTH_RATIO,
        "danger_height_ratio": OBSTACLE_AVOIDANCE_DANGER_HEIGHT_RATIO,
        "stale_seconds": OBSTACLE_AVOIDANCE_STALE_SECONDS,
    })
    return state


def _get_detection_snapshot_for_avoidance():
    now = time.time()
    with detection_lock:
        frame_time = float(last_detection_frame_time)
        frame_age = max(0.0, now - frame_time) if frame_time > 0 else -1.0
        return {
            "boxes": [dict(box) for box in last_detection_boxes_list],
            "frame_width": int(last_detection_frame_width),
            "frame_height": int(last_detection_frame_height),
            "frame_age": frame_age,
        }


def _stop_obstacle_avoidance_motion():
    if not is_basecontroller_running():
        return
    try:
        stop_robot_motion()
    except Exception as exc:
        _update_obstacle_avoidance_state(last_error=str(exc))


def set_obstacle_avoidance_enabled(enabled):
    global is_obstacle_avoidance_enabled

    enabled = bool(enabled)
    if enabled and get_obstacle_avoidance_enabled():
        return get_obstacle_avoidance_state()

    if enabled:
        if get_robot_mode_name() != CONTROL_MODE_AUTO:
            raise RuntimeError("自动避障只能在自动模式下开启")
        if DETECTION_BACKEND != "yolov8":
            raise RuntimeError("自动避障依赖 YOLOv8 孔洞识别")
        if not ensure_detection_worker():
            detection_state = get_detection_state()
            raise RuntimeError(detection_state.get("error") or "YOLO 识别线程启动失败")

        ok, message = start_basecontroller()
        if not ok:
            raise RuntimeError(message)

        set_auto_motion_mode(AUTO_MOTION_STRAIGHT)
        stop_robot_motion()

        with obstacle_avoidance_lock:
            is_obstacle_avoidance_enabled = True
            obstacle_avoidance_state.update({
                "active": False,
                "action": "waiting",
                "direction": "none",
                "left_overlap": False,
                "right_overlap": False,
                "overlap_box_count": 0,
                "danger_zone": None,
                "last_action_time": 0.0,
                "last_error": "",
                "status": "自动避障已开启，等待最新 YOLO 画面",
            })

        ensure_obstacle_avoidance_worker()
    else:
        with obstacle_avoidance_lock:
            is_obstacle_avoidance_enabled = False

        if is_basecontroller_running():
            try:
                if get_robot_mode_name() == CONTROL_MODE_AUTO:
                    set_auto_motion_mode(AUTO_MOTION_STRAIGHT)
            except Exception as exc:
                _update_obstacle_avoidance_state(last_error=str(exc))
            _stop_obstacle_avoidance_motion()

        _update_obstacle_avoidance_state(
            active=False,
            action="idle",
            direction="none",
            left_overlap=False,
            right_overlap=False,
            overlap_box_count=0,
            last_error="",
            status="自动避障已关闭，车辆已停止",
        )

    return get_obstacle_avoidance_state()


def _obstacle_avoidance_status_text(result, held=False):
    if held:
        return "孔洞短暂漏检，保持当前避障方向"
    if result["left_overlap"] and result["right_overlap"]:
        return "左右危险区均与孔洞重叠，执行左螺旋"
    if result["left_overlap"]:
        return "左危险区与孔洞重叠，执行右螺旋"
    if result["right_overlap"]:
        return "右危险区与孔洞重叠，执行左螺旋"
    return "危险区内无孔洞，直线前进中"


def obstacle_avoidance_worker_loop():
    global is_obstacle_avoidance_enabled

    current_action = "waiting"
    command_started_at = 0.0
    last_command_at = 0.0
    last_danger_at = 0.0

    while not obstacle_avoidance_stop_event.is_set():
        if not get_obstacle_avoidance_enabled():
            current_action = "waiting"
            obstacle_avoidance_stop_event.wait(OBSTACLE_AVOIDANCE_LOOP_INTERVAL_SECONDS)
            continue

        if get_robot_mode_name() != CONTROL_MODE_AUTO:
            with obstacle_avoidance_lock:
                is_obstacle_avoidance_enabled = False
            _stop_obstacle_avoidance_motion()
            _update_obstacle_avoidance_state(
                active=False,
                action="idle",
                direction="none",
                last_error="自动避障只能在自动模式运行",
                status="自动避障已停止：当前不是自动模式",
            )
            continue

        if not is_basecontroller_running():
            with obstacle_avoidance_lock:
                is_obstacle_avoidance_enabled = False
            _update_obstacle_avoidance_state(
                active=False,
                action="idle",
                direction="none",
                last_error="机器人底层控制已停止",
                status="自动避障已停止：机器人底层未运行",
            )
            continue

        now = time.time()
        snapshot = _get_detection_snapshot_for_avoidance()
        fresh = (
            snapshot["frame_age"] >= 0
            and snapshot["frame_age"] <= OBSTACLE_AVOIDANCE_STALE_SECONDS
            and snapshot["frame_width"] > 0
            and snapshot["frame_height"] > 0
        )

        if not fresh:
            if current_action != "waiting":
                _stop_obstacle_avoidance_motion()
            current_action = "waiting"
            command_started_at = 0.0
            last_command_at = 0.0
            _update_obstacle_avoidance_state(
                active=False,
                action="waiting",
                direction="none",
                left_overlap=False,
                right_overlap=False,
                overlap_box_count=0,
                last_error="",
                status="YOLO 识别结果未就绪或已过期，车辆保持停止",
            )
            obstacle_avoidance_stop_event.wait(OBSTACLE_AVOIDANCE_LOOP_INTERVAL_SECONDS)
            continue

        result = evaluate_obstacle_avoidance(
            snapshot["boxes"],
            snapshot["frame_width"],
            snapshot["frame_height"],
        )
        desired_action = result["action"]
        held = False

        if desired_action != AUTO_MOTION_STRAIGHT:
            last_danger_at = now
        elif (
            current_action in (AUTO_MOTION_SPIRAL_LEFT, AUTO_MOTION_SPIRAL_RIGHT)
            and (now - last_danger_at) < OBSTACLE_AVOIDANCE_RELEASE_DELAY_SECONDS
        ):
            desired_action = current_action
            held = True

        if (
            current_action in (AUTO_MOTION_SPIRAL_LEFT, AUTO_MOTION_SPIRAL_RIGHT)
            and desired_action in (AUTO_MOTION_SPIRAL_LEFT, AUTO_MOTION_SPIRAL_RIGHT)
            and desired_action != current_action
            and (now - command_started_at) < OBSTACLE_AVOIDANCE_MIN_HOLD_SECONDS
        ):
            desired_action = current_action
            held = True

        command_changed = desired_action != current_action
        command_expired = (now - last_command_at) >= OBSTACLE_AVOIDANCE_COMMAND_REFRESH_SECONDS

        if command_changed or command_expired:
            try:
                with obstacle_avoidance_lock:
                    if not is_obstacle_avoidance_enabled:
                        continue
                    set_auto_motion_mode(desired_action)
                    move_robot("forward")
                if command_changed:
                    current_action = desired_action
                    command_started_at = now
                last_command_at = now
            except Exception as exc:
                _stop_obstacle_avoidance_motion()
                current_action = "waiting"
                _update_obstacle_avoidance_state(
                    active=False,
                    action="waiting",
                    direction="none",
                    last_error=str(exc),
                    status=f"自动避障控制失败: {exc}",
                )
                obstacle_avoidance_stop_event.wait(OBSTACLE_AVOIDANCE_LOOP_INTERVAL_SECONDS)
                continue

        direction = {
            AUTO_MOTION_STRAIGHT: "straight",
            AUTO_MOTION_SPIRAL_LEFT: "left",
            AUTO_MOTION_SPIRAL_RIGHT: "right",
        }.get(current_action, "none")
        with obstacle_avoidance_lock:
            if not is_obstacle_avoidance_enabled:
                continue
            obstacle_avoidance_state.update({
                "active": current_action in (
                    AUTO_MOTION_SPIRAL_LEFT,
                    AUTO_MOTION_SPIRAL_RIGHT,
                ),
                "action": current_action,
                "direction": direction,
                "left_overlap": bool(result["left_overlap"]),
                "right_overlap": bool(result["right_overlap"]),
                "overlap_box_count": int(result["overlap_box_count"]),
                "danger_zone": result["danger_zone"],
                "last_action_time": command_started_at,
                "last_error": "",
                "status": _obstacle_avoidance_status_text(result, held=held),
            })

        obstacle_avoidance_stop_event.wait(OBSTACLE_AVOIDANCE_LOOP_INTERVAL_SECONDS)

    _stop_obstacle_avoidance_motion()


def ensure_obstacle_avoidance_worker():
    global obstacle_avoidance_thread

    with obstacle_avoidance_thread_lock:
        if obstacle_avoidance_thread is not None and obstacle_avoidance_thread.is_alive():
            return True

        obstacle_avoidance_stop_event.clear()
        obstacle_avoidance_thread = threading.Thread(
            target=obstacle_avoidance_worker_loop,
            name="obstacle-avoidance-worker",
            daemon=True,
        )
        obstacle_avoidance_thread.start()
        return True


def stop_obstacle_avoidance_worker(timeout=1.5):
    global obstacle_avoidance_thread, is_obstacle_avoidance_enabled

    with obstacle_avoidance_lock:
        is_obstacle_avoidance_enabled = False

    obstacle_avoidance_stop_event.set()
    with obstacle_avoidance_thread_lock:
        thread = obstacle_avoidance_thread
        obstacle_avoidance_thread = None

    if thread is not None and thread.is_alive():
        try:
            thread.join(timeout=float(timeout))
        except Exception:
            pass

    _stop_obstacle_avoidance_motion()


# =========================
# OpenCV 多算法融合小异物识别（独立于 YOLO 孔洞识别）
# =========================
# 识别策略：
# 1. 单算法命中：不画框；
# 2. 多算法空间重合命中：进入候选；
# 3. 同一区域连续 2 帧稳定命中：正式画框；
# 4. 连续多帧丢失后才清空画框，减少闪烁；
# 5. 只针对小尺寸异物，过滤大面积阴影、边缘、背景块。
#
# 注意：
# 该区块只影响“异物识别”按钮对应的 OpenCV 异物识别功能；
# 不改变 YOLO 孔洞识别、WebRTC、拍照录像、风机、舵机、ROS2 机器人控制等其他功能。

# 小异物几何约束。使用 FOREIGN_* 独立参数，避免影响原 YOLO 孔洞识别配置。
FOREIGN_DETECTION_MIN_AREA = int(os.environ.get("FOREIGN_DETECTION_MIN_AREA", "3"))
FOREIGN_DETECTION_MAX_AREA = int(os.environ.get("FOREIGN_DETECTION_MAX_AREA", "800"))
FOREIGN_DETECTION_MAX_ASPECT_RATIO = float(os.environ.get("FOREIGN_DETECTION_MAX_ASPECT_RATIO", "6.0"))
FOREIGN_DETECTION_MIN_FILL_RATIO = float(os.environ.get("FOREIGN_DETECTION_MIN_FILL_RATIO", "0.08"))

# 局部亮/暗异常阈值。越大越保守，误报更少，但小异物可能漏检。
FOREIGN_LOCAL_DIFF = int(os.environ.get("FOREIGN_LOCAL_DIFF", "14"))

# 边缘/纹理突变阈值。越大越保守。
FOREIGN_EDGE_THRESHOLD = int(os.environ.get("FOREIGN_EDGE_THRESHOLD", "22"))

# 背景建模阈值。越大越保守。
FOREIGN_BG_THRESHOLD = int(os.environ.get("FOREIGN_BG_THRESHOLD", "180"))

# 至少多少种算法同时命中同一区域，才进入候选。
FOREIGN_VOTE_REQUIRED = int(os.environ.get("FOREIGN_VOTE_REQUIRED", "2"))

# 多算法框融合时的 IoU 阈值。小目标框 IoU 常偏低，因此还会结合中心点距离判断。
FOREIGN_FUSION_IOU = float(os.environ.get("FOREIGN_FUSION_IOU", "0.10"))

# 背景建模刚启动时容易不稳定，前几帧只用于学习背景，不参与投票。
FOREIGN_BG_WARMUP_FRAMES = int(os.environ.get("FOREIGN_BG_WARMUP_FRAMES", "8"))

# 连续帧稳定匹配阈值。默认沿用 DETECTION_CONFIRM_FRAMES=2。
FOREIGN_TRACK_MATCH_IOU = float(os.environ.get("FOREIGN_TRACK_MATCH_IOU", "0.10"))

# 背景建模器与连续帧候选轨迹。
foreign_bg_subtractor = None
foreign_bg_frames_seen = 0
foreign_candidate_tracks = []


def get_foreign_detection_enabled():
    with foreign_detection_lock:
        return bool(is_foreign_detection_enabled)


def set_foreign_detection_enabled(enabled: bool):
    global is_foreign_detection_enabled
    with foreign_detection_lock:
        is_foreign_detection_enabled = bool(enabled)
    return bool(enabled)


def reset_foreign_detection_result(reset_frames=True):
    """
    清空 OpenCV 多算法融合异物识别结果、候选轨迹、防抖状态和背景建模状态。
    """
    global foreign_detection_count, foreign_lost_count, is_dark_foreign_object_detected
    global last_foreign_detection_boxes, last_foreign_detection_error, last_foreign_detection_frame_time
    global foreign_detection_total_frames, last_foreign_detection_boxes_list
    global last_foreign_detection_frame_width, last_foreign_detection_frame_height
    global last_foreign_detection_infer_ms
    global foreign_bg_subtractor, foreign_bg_frames_seen, foreign_candidate_tracks

    with foreign_detection_lock:
        foreign_detection_count = 0
        foreign_lost_count = 0
        is_dark_foreign_object_detected = False
        last_foreign_detection_boxes = 0
        last_foreign_detection_error = ""
        last_foreign_detection_frame_time = 0.0
        last_foreign_detection_boxes_list = []
        last_foreign_detection_frame_width = 0
        last_foreign_detection_frame_height = 0
        last_foreign_detection_infer_ms = 0.0
        if reset_frames:
            foreign_detection_total_frames = 0

    # 每次重新开启异物识别时重置背景建模，避免旧场景残留。
    foreign_bg_subtractor = None
    foreign_bg_frames_seen = 0
    foreign_candidate_tracks = []


def get_foreign_detection_state():
    """
    返回 OpenCV 多算法融合异物识别状态。

    关键点：
    - 单算法命中不会出现在 boxes 中；
    - 多算法重合但未连续稳定 2 帧时，也不会画框；
    - 只有 confirmed 后 detected=True，前端才拿到正式 boxes。
    """
    now = time.time()
    with foreign_detection_lock:
        enabled = bool(is_foreign_detection_enabled)

        if last_foreign_detection_frame_time > 0:
            frame_age = max(0.0, now - float(last_foreign_detection_frame_time))
        else:
            frame_age = -1.0

        stale = (not enabled) or (frame_age < 0) or (frame_age > float(DETECTION_STALE_SECONDS))
        detected = bool(is_dark_foreign_object_detected) and not stale

        return {
            "enabled": enabled,
            "detected": detected,
            "boxes": [dict(b) for b in last_foreign_detection_boxes_list] if detected else [],
            "box_count": int(last_foreign_detection_boxes) if detected else 0,
            "detection_count": int(foreign_detection_count),
            "lost_count": int(foreign_lost_count),
            "error": last_foreign_detection_error,
            "frames": int(foreign_detection_total_frames),
            "last_frame_time": float(last_foreign_detection_frame_time),
            "frame_age": frame_age,
            "frame_width": int(last_foreign_detection_frame_width),
            "frame_height": int(last_foreign_detection_frame_height),
            "infer_ms": float(last_foreign_detection_infer_ms),
            "worker_alive": bool(last_foreign_detection_worker_alive),
            "stale": bool(stale),
            "mode": "webrtc_canvas_overlay_opencv_fusion_small_object",
            "backend": "opencv_fusion_small_object",
            "vote_required": int(FOREIGN_VOTE_REQUIRED),
            "confirm_frames": int(DETECTION_CONFIRM_FRAMES),
            "yolo_preserved": True,
        }


def _set_foreign_detection_error(msg):
    global last_foreign_detection_error
    with foreign_detection_lock:
        last_foreign_detection_error = str(msg or "")


def _set_foreign_detection_worker_alive(value: bool):
    global last_foreign_detection_worker_alive
    with foreign_detection_lock:
        last_foreign_detection_worker_alive = bool(value)


def _box_iou(a, b):
    """
    计算两个矩形框 IoU。
    box 格式：(x, y, w, h)
    """
    ax, ay, aw, ah = a
    bx, by, bw, bh = b

    ax2 = ax + aw
    ay2 = ay + ah
    bx2 = bx + bw
    by2 = by + bh

    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)

    inter = iw * ih
    union = aw * ah + bw * bh - inter

    if union <= 0:
        return 0.0

    return inter / float(union)


def _center_close(a, b):
    """
    小目标框很小，不同算法得到的框可能略偏，IoU 会偏低。
    因此增加中心点接近判断。
    """
    ax, ay, aw, ah = a
    bx, by, bw, bh = b

    acx = ax + aw / 2.0
    acy = ay + ah / 2.0
    bcx = bx + bw / 2.0
    bcy = by + bh / 2.0

    dist = ((acx - bcx) ** 2 + (acy - bcy) ** 2) ** 0.5
    limit = max(8.0, max(min(aw, ah, bw, bh) * 1.5, 1.0))

    return dist <= limit


def _same_small_target(a, b, iou_threshold=None):
    """
    判断两个小框是否对应同一个小异物。
    """
    if iou_threshold is None:
        iou_threshold = FOREIGN_FUSION_IOU

    return _box_iou(a, b) >= float(iou_threshold) or _center_close(a, b)


def _box_from_clean_item(item):
    return (
        int(item.get("x", 0)),
        int(item.get("y", 0)),
        int(item.get("w", 1)),
        int(item.get("h", 1)),
    )


def _store_foreign_detection_result(frame_w, frame_h, boxes, infer_ms):
    """
    保存 OpenCV 多算法融合异物识别坐标。

    输入 boxes 是 detect_opencv_fusion_foreign_object_boxes() 的结果：
    [
        {
            "box": (x, y, w, h),
            "methods": ["local_anomaly", "edge_detail"],
            "votes": 2
        },
        ...
    ]

    这里完成连续帧稳定确认：
    - 第 1 帧多算法重合：只进入候选，不画框；
    - 第 2 帧同一区域再次多算法重合：正式画框；
    - 短暂丢失：保留上一帧正式框，避免闪烁；
    - 连续丢失 DETECTION_LOST_FRAMES 帧：清空画框。
    """
    global foreign_detection_count, foreign_lost_count, is_dark_foreign_object_detected
    global last_foreign_detection_boxes, last_foreign_detection_boxes_list
    global last_foreign_detection_frame_width, last_foreign_detection_frame_height
    global last_foreign_detection_frame_time, foreign_detection_total_frames
    global last_foreign_detection_infer_ms, last_foreign_detection_error
    global foreign_candidate_tracks

    frame_w = int(frame_w or 0)
    frame_h = int(frame_h or 0)
    if frame_w <= 0 or frame_h <= 0:
        return

    clean_candidates = []

    for i, item in enumerate(boxes or []):
        if isinstance(item, dict):
            raw_box = item.get("box", item)
            methods = item.get("methods", [])
            votes = int(item.get("votes", len(methods) if isinstance(methods, (list, tuple, set)) else 0))
        else:
            raw_box = item
            methods = []
            votes = 0

        if isinstance(raw_box, dict):
            x = raw_box.get("x", raw_box.get("x1", 0))
            y = raw_box.get("y", raw_box.get("y1", 0))
            w = raw_box.get("w", max(1, float(raw_box.get("x2", 1)) - float(x)))
            h = raw_box.get("h", max(1, float(raw_box.get("y2", 1)) - float(y)))
        else:
            x, y, w, h = raw_box

        x = int(max(0, min(frame_w - 1, int(float(x)))))
        y = int(max(0, min(frame_h - 1, int(float(y)))))
        w = int(max(1, min(frame_w - x, int(float(w)))))
        h = int(max(1, min(frame_h - y, int(float(h)))))

        method_list = [str(m) for m in methods] if isinstance(methods, (list, tuple, set)) else []

        label = "异物"
        if votes > 0:
            label = f"异物 {votes}票"

        clean_candidates.append({
            "id": int(i),
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "x1": x,
            "y1": y,
            "x2": x + w,
            "y2": y + h,
            "label": label,
            "class_name": "fusion_small_foreign_object",
            "class_id": -1,
            "backend": "opencv_fusion_small_object",
            "votes": int(votes),
            "methods": method_list,
        })

    # 连续帧轨迹更新：同一区域连续稳定命中才确认。
    previous_tracks = list(foreign_candidate_tracks or [])
    updated_tracks = []
    used_prev = set()

    for cand in clean_candidates:
        cand_box = _box_from_clean_item(cand)

        best_idx = -1
        best_score = 0.0

        for idx, track in enumerate(previous_tracks):
            if idx in used_prev:
                continue

            prev_box = track.get("box", None)
            if not prev_box:
                continue

            iou = _box_iou(cand_box, prev_box)
            same = _same_small_target(cand_box, prev_box, iou_threshold=FOREIGN_TRACK_MATCH_IOU)

            if same and iou >= best_score:
                best_idx = idx
                best_score = iou

        if best_idx >= 0:
            prev_track = previous_tracks[best_idx]
            used_prev.add(best_idx)
            stable_frames = int(prev_track.get("stable_frames", 1)) + 1
        else:
            stable_frames = 1

        track = {
            "box": cand_box,
            "item": cand,
            "stable_frames": stable_frames,
        }
        updated_tracks.append(track)

    confirmed_items = [
        dict(t["item"])
        for t in updated_tracks
        if int(t.get("stable_frames", 0)) >= int(DETECTION_CONFIRM_FRAMES)
    ]

    max_stable_frames = 0
    if updated_tracks:
        max_stable_frames = max(int(t.get("stable_frames", 1)) for t in updated_tracks)

    with foreign_detection_lock:
        foreign_candidate_tracks = updated_tracks
        foreign_detection_count = int(max_stable_frames)

        if confirmed_items:
            foreign_lost_count = 0
            is_dark_foreign_object_detected = True

            # 重新编号，避免 track 匹配后 id 不连续。
            for idx, item in enumerate(confirmed_items):
                item["id"] = int(idx)

            last_foreign_detection_boxes = len(confirmed_items)
            last_foreign_detection_boxes_list = confirmed_items
        else:
            # 没有正式确认框。候选首帧也不画框。
            foreign_lost_count += 1

            if foreign_lost_count >= DETECTION_LOST_FRAMES:
                is_dark_foreign_object_detected = False
                last_foreign_detection_boxes = 0
                last_foreign_detection_boxes_list = []
            elif not is_dark_foreign_object_detected:
                # 尚未正式确认过，不能把候选框暴露给前端。
                last_foreign_detection_boxes = 0
                last_foreign_detection_boxes_list = []
            # 如果之前已确认，但当前短暂未确认，则暂时保留上一帧正式框，减少闪烁。

        last_foreign_detection_frame_width = frame_w
        last_foreign_detection_frame_height = frame_h
        last_foreign_detection_frame_time = time.time()
        foreign_detection_total_frames += 1
        last_foreign_detection_infer_ms = float(infer_ms)
        last_foreign_detection_error = ""


def _get_foreign_bg_subtractor():
    """
    背景建模器。
    history 越大，背景更新越慢；
    varThreshold 越大，越保守，误报更少，但漏检可能增加。
    """
    global foreign_bg_subtractor

    if foreign_bg_subtractor is None:
        foreign_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=120,
            varThreshold=20,
            detectShadows=False,
        )

    return foreign_bg_subtractor


def _resize_for_foreign_detection(frame):
    """
    缩放图像，降低树莓派 CPU 压力。
    """
    frame_h, frame_w = frame.shape[:2]
    scale = 1.0
    proc = frame

    if frame_w > DETECTION_PROCESS_WIDTH:
        scale = DETECTION_PROCESS_WIDTH / float(frame_w)
        proc_h = max(1, int(frame_h * scale))
        proc = cv2.resize(frame, (DETECTION_PROCESS_WIDTH, proc_h), interpolation=cv2.INTER_AREA)

    return proc, scale


def _valid_small_foreign_object_contour(cnt):
    """
    小异物几何过滤：面积、长宽比、填充率。
    """
    area = float(cv2.contourArea(cnt))

    if not (FOREIGN_DETECTION_MIN_AREA <= area <= FOREIGN_DETECTION_MAX_AREA):
        return None

    x, y, w, h = cv2.boundingRect(cnt)

    if w <= 1 or h <= 1:
        return None

    aspect = max(w / float(h), h / float(w))
    if aspect > FOREIGN_DETECTION_MAX_ASPECT_RATIO:
        return None

    fill_ratio = area / float(max(1, w * h))
    if fill_ratio < FOREIGN_DETECTION_MIN_FILL_RATIO:
        return None

    return x, y, w, h


def _boxes_from_foreign_mask(mask):
    """
    从二值 mask 中提取小异物候选框。
    """
    k1 = np.ones((2, 2), np.uint8)
    k2 = np.ones((3, 3), np.uint8)

    # 小目标不能过度开运算，否则容易被抹掉。
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k1, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k2, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes = []
    for cnt in contours:
        box = _valid_small_foreign_object_contour(cnt)
        if box is not None:
            boxes.append(box)

    boxes.sort(key=lambda b: (b[1], b[0]))
    return boxes


def _map_boxes_to_original_size(boxes, scale):
    """
    把缩放后图像坐标映射回原始视频坐标。
    """
    if scale == 1.0:
        return boxes

    mapped = []
    for x, y, w, h in boxes:
        mapped.append((
            int(x / scale),
            int(y / scale),
            int(w / scale),
            int(h / scale),
        ))

    return mapped


def _detector_local_anomaly(proc):
    """
    检测局部灰度异常。
    不再只检测更暗，而是检测“比局部背景更亮或更暗”的小异常。
    """
    proc_h, proc_w = proc.shape[:2]
    ignore_top = int(proc_h * float(DETECTION_IGNORE_TOP_RATIO))

    gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)

    # 用高斯模糊估计局部背景。
    bg = cv2.GaussianBlur(gray, (0, 0), 13)

    # 亮/暗异常统一检测。
    diff = cv2.absdiff(gray, bg)

    # 忽略顶部区域，减少远处背景、边界结构误报。
    diff[:ignore_top, :] = 0

    # 排除强反光区域及其边缘。
    bright_mask = cv2.inRange(gray, 240, 255)
    bright_mask = cv2.dilate(bright_mask, np.ones((5, 5), np.uint8), iterations=1)
    diff[bright_mask > 0] = 0

    _, mask = cv2.threshold(diff, int(FOREIGN_LOCAL_DIFF), 255, cv2.THRESH_BINARY)

    return _boxes_from_foreign_mask(mask)


def _detector_edge_detail(proc):
    """
    检测边缘/纹理突变。
    适用于异物与背景颜色接近，但存在边缘或局部纹理变化的情况。
    """
    proc_h, proc_w = proc.shape[:2]
    ignore_top = int(proc_h * float(DETECTION_IGNORE_TOP_RATIO))

    gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)

    lap = cv2.Laplacian(gray, cv2.CV_16S, ksize=3)
    lap = cv2.convertScaleAbs(lap)

    lap[:ignore_top, :] = 0

    _, mask = cv2.threshold(lap, int(FOREIGN_EDGE_THRESHOLD), 255, cv2.THRESH_BINARY)

    return _boxes_from_foreign_mask(mask)


def _detector_background_change(proc):
    """
    背景建模检测。
    如果异物是后来出现的，即使颜色接近背景，也可能被检测出来。

    注意：
    摄像头运动明显时，该分支可能更容易误报；
    因此它只作为多算法投票的一路，不会单独画框。
    """
    global foreign_bg_frames_seen

    proc_h, proc_w = proc.shape[:2]
    ignore_top = int(proc_h * float(DETECTION_IGNORE_TOP_RATIO))

    bg = _get_foreign_bg_subtractor()

    # learningRate 小一些，避免异物很快被吸收到背景里。
    fg = bg.apply(proc, learningRate=0.005)
    foreign_bg_frames_seen += 1

    # 热身阶段只学习背景，不参与检测。
    if foreign_bg_frames_seen <= FOREIGN_BG_WARMUP_FRAMES:
        return []

    fg[:ignore_top, :] = 0

    _, mask = cv2.threshold(fg, int(FOREIGN_BG_THRESHOLD), 255, cv2.THRESH_BINARY)

    return _boxes_from_foreign_mask(mask)


def _fuse_multimethod_foreign_boxes(method_boxes):
    """
    多算法候选框融合。

    逻辑：
    - 单算法命中：不输出；
    - 多算法空间重合命中：输出候选；
    - 连续 2 帧稳定确认：由 _store_foreign_detection_result() 完成。
    """
    all_candidates = []

    for method_name, boxes in method_boxes.items():
        for box in boxes:
            all_candidates.append({
                "method": method_name,
                "box": box,
            })

    fused = []
    used = [False] * len(all_candidates)

    for i, item in enumerate(all_candidates):
        if used[i]:
            continue

        base_box = item["box"]
        group_boxes = [base_box]
        group_methods = {item["method"]}
        used[i] = True

        for j in range(i + 1, len(all_candidates)):
            if used[j]:
                continue

            other_box = all_candidates[j]["box"]

            same_target = _same_small_target(base_box, other_box, iou_threshold=FOREIGN_FUSION_IOU)

            if same_target:
                group_boxes.append(other_box)
                group_methods.add(all_candidates[j]["method"])
                used[j] = True

        # 至少多种算法同时命中，才进入候选。
        if len(group_methods) >= FOREIGN_VOTE_REQUIRED:
            xs = [b[0] for b in group_boxes]
            ys = [b[1] for b in group_boxes]
            x2s = [b[0] + b[2] for b in group_boxes]
            y2s = [b[1] + b[3] for b in group_boxes]

            x1 = min(xs)
            y1 = min(ys)
            x2 = max(x2s)
            y2 = max(y2s)

            fused.append({
                "box": (
                    int(x1),
                    int(y1),
                    int(x2 - x1),
                    int(y2 - y1),
                ),
                "methods": sorted(list(group_methods)),
                "votes": len(group_methods),
            })

    fused.sort(key=lambda item: (item["box"][1], item["box"][0]))
    return fused


def detect_opencv_fusion_foreign_object_boxes(frame):
    """
    多算法融合版小异物检测。

    三个检测分支：
    1. local_anomaly：局部亮/暗异常；
    2. edge_detail：边缘/纹理突变；
    3. background_change：背景变化。

    输出：
    [
        {
            "box": (x, y, w, h),
            "methods": ["local_anomaly", "edge_detail"],
            "votes": 2
        },
        ...
    ]

    这里输出的是“候选”，不等于正式画框；
    正式画框还要经过连续 2 帧稳定确认。
    """
    if cv2 is None or np is None or frame is None:
        return []

    proc, scale = _resize_for_foreign_detection(frame)

    # 三路检测。
    boxes_local = _detector_local_anomaly(proc)
    boxes_edge = _detector_edge_detail(proc)
    boxes_bg = _detector_background_change(proc)

    # 映射回原图坐标。
    boxes_local = _map_boxes_to_original_size(boxes_local, scale)
    boxes_edge = _map_boxes_to_original_size(boxes_edge, scale)
    boxes_bg = _map_boxes_to_original_size(boxes_bg, scale)

    method_boxes = {
        "local_anomaly": boxes_local,
        "edge_detail": boxes_edge,
        "background_change": boxes_bg,
    }

    return _fuse_multimethod_foreign_boxes(method_boxes)


def detect_opencv_dark_foreign_object_boxes(frame):
    """
    兼容旧函数名。

    原函数名是暗目标检测；
    现在内部改为多算法融合小异物检测。
    这样可以不改其他调用位置。
    """
    return detect_opencv_fusion_foreign_object_boxes(frame)


def foreign_detection_worker_loop():
    """
    OpenCV 多算法融合异物识别后台线程。
    独立于原 YOLO detection_worker_loop；只在“异物识别”按钮开启后运行。
    """
    cap = None
    frame_interval = 1.0 / max(1, int(DETECTION_WORKER_FPS))
    last_process_time = 0.0

    _set_foreign_detection_worker_alive(True)

    try:
        while not foreign_detection_stop_event.is_set() and get_foreign_detection_enabled():
            if cv2 is None or np is None:
                _set_foreign_detection_error(f"OpenCV 未安装或导入失败: {CV_IMPORT_ERROR}")
                time.sleep(1.0)
                continue

            try:
                if cap is None or not cap.isOpened():
                    ensure_streamer_ready()
                    cap = open_detection_capture()
                    if not cap.isOpened():
                        raise RuntimeError("无法打开 RTSP 异物识别视频流")

                ok, frame = cap.read()
                if not ok or frame is None:
                    raise RuntimeError("读取 RTSP 帧失败，正在重连")

                now = time.time()
                if now - last_process_time < frame_interval:
                    # 持续读帧但不每帧识别，避免 RTSP 缓冲积压，同时控制 CPU 占用。
                    time.sleep(0.002)
                    continue

                last_process_time = now

                frame_h, frame_w = frame.shape[:2]
                t0 = time.perf_counter()
                boxes = detect_opencv_fusion_foreign_object_boxes(frame)
                infer_ms = (time.perf_counter() - t0) * 1000.0
                _store_foreign_detection_result(frame_w, frame_h, boxes, infer_ms)

            except Exception as e:
                _set_foreign_detection_error(e)
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
                cap = None
                time.sleep(0.25)

    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass
        _set_foreign_detection_worker_alive(False)


def ensure_foreign_detection_worker():
    """
    确保 OpenCV 多算法融合异物识别线程已启动。
    """
    global foreign_detection_thread

    if cv2 is None or np is None:
        _set_foreign_detection_error(f"OpenCV 未安装或导入失败: {CV_IMPORT_ERROR}")
        return False

    with foreign_detection_thread_lock:
        if foreign_detection_thread is not None and foreign_detection_thread.is_alive():
            return True

        foreign_detection_stop_event.clear()
        foreign_detection_thread = threading.Thread(
            target=foreign_detection_worker_loop,
            name="opencv-fusion-foreign-detection-worker",
            daemon=True,
        )
        foreign_detection_thread.start()
        return True


def stop_foreign_detection_worker(timeout=1.5):
    """
    停止 OpenCV 多算法融合异物识别线程。
    """
    global foreign_detection_thread

    foreign_detection_stop_event.set()
    with foreign_detection_thread_lock:
        t = foreign_detection_thread
        foreign_detection_thread = None

    if t is not None and t.is_alive():
        try:
            t.join(timeout=float(timeout))
        except Exception:
            pass


def start_foreign_detection_feature():
    """
    由前端“异物识别”按钮调用：
    开启 OpenCV 多算法融合异物识别。
    """
    if cv2 is None or np is None:
        _set_foreign_detection_error(f"OpenCV 未安装或导入失败: {CV_IMPORT_ERROR}")
        return False

    reset_foreign_detection_result(reset_frames=True)
    set_foreign_detection_enabled(True)

    if not ensure_foreign_detection_worker():
        set_foreign_detection_enabled(False)
        return False

    return True


def stop_foreign_detection_feature():
    """
    由前端“异物识别”按钮调用：
    关闭 OpenCV 多算法融合异物识别，并清空画框。
    """
    set_foreign_detection_enabled(False)
    stop_foreign_detection_worker()
    reset_foreign_detection_result(reset_frames=True)
    return True


def build_capture_filename(prefix, ext):
    ts = time.strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{ts}_{uuid.uuid4().hex[:8]}.{ext}"


def recording_is_running():
    global recording_process
    return recording_process is not None and recording_process.poll() is None


def stop_ffmpeg_process(proc, timeout=8.0):
    if proc is None:
        return
    try:
        if proc.poll() is None:
            if os.name != "nt":
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            else:
                proc.terminate()
            proc.wait(timeout=timeout)
    except Exception:
        try:
            if proc.poll() is None:
                if os.name != "nt":
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                else:
                    proc.kill()
        except Exception:
            pass


def capture_photo_frame():
    ensure_streamer_ready()

    ffmpeg = find_ffmpeg_bin()
    if not ffmpeg:
        raise RuntimeError("未找到 ffmpeg，请安装 ffmpeg 或设置 FFMPEG_BIN")

    out_name = build_capture_filename("photo", "jpg")
    out_path = CAPTURE_DIR / out_name

    cmd = [
        ffmpeg,
        "-hide_banner",
        "-loglevel", "error",
        "-nostdin",
        "-rtsp_transport", "tcp",
        "-i", build_rtsp_url(),
        "-frames:v", "1",
        "-q:v", "2",
        "-y",
        str(out_path),
    ]

    subprocess.run(cmd, check=True, timeout=20)

    if not out_path.exists() or out_path.stat().st_size <= 0:
        raise RuntimeError("拍照失败，输出文件为空")

    return out_path


def start_local_recording():
    global recording_process, recording_log_fp, current_recording_file, last_recording_error

    with recording_lock:
        if recording_is_running():
            return current_recording_file

        ensure_streamer_ready()

        ffmpeg = find_ffmpeg_bin()
        if not ffmpeg:
            raise RuntimeError("未找到 ffmpeg，请安装 ffmpeg 或设置 FFMPEG_BIN")

        out_name = build_capture_filename("record", "mp4")
        out_path = CAPTURE_DIR / out_name

        if recording_log_fp is None or recording_log_fp.closed:
            recording_log_fp = open(RECORDING_LOG_PATH, "ab", buffering=0)

        banner = f"\n\n===== START RECORDING =====\n{out_path}\n"
        recording_log_fp.write(banner.encode("utf-8", errors="ignore"))

        cmd = [
            ffmpeg,
            "-hide_banner",
            "-loglevel", "warning",
            "-nostdin",
            "-rtsp_transport", "tcp",
            "-i", build_rtsp_url(),
            "-map", "0:v:0",
            "-an",
            "-c:v", "copy",
            "-movflags", "+faststart",
            "-y",
            str(out_path),
        ]

        recording_process = subprocess.Popen(
            cmd,
            cwd=str(BASE_DIR),
            stdout=recording_log_fp,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid if os.name != "nt" else None,
        )

        current_recording_file = out_path
        time.sleep(1.0)

        if recording_process.poll() is not None:
            last_recording_error = "录像启动失败:\n" + tail_log(RECORDING_LOG_PATH, 120)
            recording_process = None
            current_recording_file = None
            raise RuntimeError(last_recording_error)

        last_recording_error = ""
        print(f"[INFO] Recording started: {out_name}")
        return out_path


def stop_local_recording():
    global recording_process, current_recording_file, last_recording_error

    with recording_lock:
        if recording_process is None:
            if current_recording_file and current_recording_file.exists():
                out_path = current_recording_file
                current_recording_file = None
                return out_path
            raise RuntimeError("当前没有正在录制的视频")

        proc = recording_process
        stop_ffmpeg_process(proc, timeout=8.0)

        out_path = current_recording_file
        recording_process = None
        current_recording_file = None

        if out_path and out_path.exists() and out_path.stat().st_size > 0:
            last_recording_error = ""
            print(f"[INFO] Recording stopped: {out_path.name}")
            return out_path

        raise RuntimeError(last_recording_error or "录制失败，未生成有效文件")


def ensure_streamer_ready():
    if mediamtx_is_running() and publisher_is_running():
        return

    with video_state_lock:
        w = video_state["width"]
        h = video_state["height"]
        f = video_state["fps"]
        b = video_state["bitrate"]

    if not mediamtx_is_running():
        start_mediamtx()

    if not publisher_is_running():
        start_publisher(w, h, f, b)
        time.sleep(0.8)


def apply_video_config(width, height, fps):
    width = int(width)
    height = int(height)
    fps = int(fps)

    if recording_is_running():
        raise RuntimeError("请先结束录制，再切换分辨率或 FPS")

    if not is_supported_resolution(width, height):
        raise ValueError("不支持的分辨率")
    if not is_supported_fps(fps):
        raise ValueError("不支持的 FPS")

    bitrate = get_bitrate(width, height)

    with video_state_lock:
        video_state["width"] = width
        video_state["height"] = height
        video_state["fps"] = fps
        video_state["bitrate"] = bitrate

    stop_publisher()
    if not mediamtx_is_running():
        start_mediamtx()
    start_publisher(width, height, fps, bitrate)
    time.sleep(0.4)

    with video_state_lock:
        return dict(video_state)


def get_video_config():
    with video_state_lock:
        d = dict(video_state)
    d["resolution"] = resolution_to_str(d["width"], d["height"])
    d["idr_period"] = get_idr_period(d["fps"])
    return d


# =========================
# 页面
# =========================
@app.route("/")
def index():
    return send_file(BASE_DIR / "index_汇总.html")


@app.route("/detection_view")
def detection_view():
    """
    兼容旧入口：这个页面也使用“原始 WebRTC + canvas 坐标叠加”，不再播放 MJPEG。
    主前端 index1.html 现在会直接加载 MediaMTX WebRTC，并在外层叠加 canvas。
    """
    client_host = (request.host.split(":", 1)[0] or get_ip_address()).strip()
    direct_url = f"http://{client_host}:{WEBRTC_HTTP_PORT}/cam"
    ensure_detection_worker()
    return f"""
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <style>
    html, body {{ margin: 0; width: 100%; height: 100%; overflow: hidden; background: #000; }}
    #wrap {{ position: relative; width: 100%; height: 100%; background: #000; overflow: hidden; }}
    iframe {{ position: absolute; inset: 0; width: 100%; height: 100%; border: 0; background: #000; }}
    canvas {{ position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; z-index: 2; }}
    #tip {{
      position: absolute; left: 12px; bottom: 12px; z-index: 3; max-width: calc(100% - 24px);
      padding: 6px 10px; color: #18f0a1; background: rgba(20, 27, 35, 0.82);
      border: 1px solid rgba(0,255,170,0.16); font: 13px/1.4 "Microsoft YaHei", Arial, sans-serif;
      white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
    }}
  </style>
</head>
<body>
  <div id="wrap">
    <iframe id="directFrame" src="{direct_url}" allow="autoplay; fullscreen; picture-in-picture" referrerpolicy="no-referrer"></iframe>
    <canvas id="overlay"></canvas>
    <div id="tip">原始 WebRTC 视频 + 异物坐标叠加</div>
  </div>
  <script>
    const canvas = document.getElementById('overlay');
    const ctx = canvas.getContext('2d');
    const wrap = document.getElementById('wrap');
    const tip = document.getElementById('tip');
    let latest = null;

    function resizeCanvas() {{
      const r = wrap.getBoundingClientRect();
      const dpr = Math.max(1, window.devicePixelRatio || 1);
      const w = Math.max(1, Math.round(r.width * dpr));
      const h = Math.max(1, Math.round(r.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {{
        canvas.width = w;
        canvas.height = h;
        canvas.style.width = r.width + 'px';
        canvas.style.height = r.height + 'px';
      }}
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }}

    function draw(data) {{
      latest = data || latest;
      resizeCanvas();
      const cw = canvas.clientWidth || wrap.clientWidth;
      const ch = canvas.clientHeight || wrap.clientHeight;
      ctx.clearRect(0, 0, cw, ch);
      if (!data || data.stale || !Array.isArray(data.boxes)) return;
      const fw = Number(data.frame_width || 1280);
      const fh = Number(data.frame_height || 720);
      if (!fw || !fh) return;
      const scale = Math.min(cw / fw, ch / fh);
      const dw = fw * scale;
      const dh = fh * scale;
      const ox = (cw - dw) / 2;
      const oy = (ch - dh) / 2;
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#00ff00';
      ctx.fillStyle = '#00ff00';
      ctx.font = 'bold 13px Microsoft YaHei, Arial, sans-serif';
      data.boxes.forEach((b) => {{
        const x = ox + Number(b.x || 0) * scale;
        const y = oy + Number(b.y || 0) * scale;
        const w = Number(b.w || 0) * scale;
        const h = Number(b.h || 0) * scale;
        ctx.strokeRect(x, y, w, h);
      }});
      if (data.boxes.length) {{
        ctx.fillText('DETECTION: FOREIGN OBJECT  boxes=' + data.boxes.length, 10, Math.max(20, ch - 14));
      }}
    }}

    async function poll() {{
      try {{
        const res = await fetch('/api/detection_status?ts=' + Date.now(), {{ cache: 'no-store' }});
        const data = await res.json();
        if (data.ok) {{
          draw(data);
          if (data.error) {{
            tip.textContent = '识别重连中：' + String(data.error).slice(0, 80);
            tip.style.display = 'block';
          }} else {{
            tip.style.display = 'none';
          }}
        }}
      }} catch (e) {{
        tip.textContent = '识别坐标读取失败';
        tip.style.display = 'block';
      }}
    }}

    window.addEventListener('resize', () => draw(latest));
    setInterval(poll, 160);
    poll();
  </script>
</body>
</html>
"""


@app.route("/api/detection_feed")
def api_detection_feed():
    """
    低延迟版本已关闭 MJPEG 识别视频流。
    前端应直接播放 MediaMTX WebRTC，并通过 /api/detection_status 获取 boxes 坐标后用 canvas 叠加。
    """
    return jsonify({
        "ok": False,
        "message": "MJPEG 识别视频流已关闭；请使用 WebRTC 原始视频 + /api/detection_status 坐标叠加。",
    }), 410


@app.route("/api/detection_status", methods=["GET"])
def api_detection_status():
    if cv2 is None or np is None:
        return jsonify({
            "ok": False,
            "message": f"OpenCV 未安装或导入失败: {CV_IMPORT_ERROR}",
            "obstacle_avoidance": get_obstacle_avoidance_state(),
            **get_detection_state(),
        }), 500

    if DETECTION_BACKEND == "yolov8":
        if YOLO is None:
            return jsonify({
                "ok": False,
                "message": f"ultralytics 未安装或导入失败: {YOLO_IMPORT_ERROR}",
                "obstacle_avoidance": get_obstacle_avoidance_state(),
                **get_detection_state(),
            }), 500
        if not YOLO_MODEL_PATH.exists():
            return jsonify({
                "ok": False,
                "message": f"未找到 YOLO 权重文件: {YOLO_MODEL_PATH}",
                "obstacle_avoidance": get_obstacle_avoidance_state(),
                **get_detection_state(),
            }), 500

    ensure_detection_worker()
    return jsonify({
        "ok": True,
        "obstacle_avoidance": get_obstacle_avoidance_state(),
        **get_detection_state(),
    })


@app.route("/api/foreign_detection_status", methods=["GET"])
def api_foreign_detection_status():
    state = get_foreign_detection_state()

    if not state.get("enabled"):
        return jsonify({
            "ok": True,
            "message": "OpenCV 暗目标异物识别未开启",
            **state,
        })

    if cv2 is None or np is None:
        return jsonify({
            "ok": False,
            "message": f"OpenCV 未安装或导入失败: {CV_IMPORT_ERROR}",
            **state,
        }), 500

    return jsonify({
        "ok": True,
        **state,
    })


@app.route("/api/start_foreign_detection", methods=["POST"])
@app.route("/api/start_detection", methods=["POST"])
def api_start_foreign_detection():
    if start_foreign_detection_feature():
        return jsonify({
            "ok": True,
            "message": "OpenCV 暗目标异物识别已开启",
            **get_foreign_detection_state(),
        })

    return jsonify({
        "ok": False,
        "message": last_foreign_detection_error or "OpenCV 暗目标异物识别开启失败",
        **get_foreign_detection_state(),
    }), 500


@app.route("/api/stop_foreign_detection", methods=["POST"])
@app.route("/api/stop_detection", methods=["POST"])
def api_stop_foreign_detection():
    stop_foreign_detection_feature()
    return jsonify({
        "ok": True,
        "message": "OpenCV 暗目标异物识别已关闭",
        **get_foreign_detection_state(),
    })



# =========================
# WHEP 代理 API
# =========================
@app.route("/api/whep_offer", methods=["POST"])
def api_whep_offer():
    try:
        ensure_streamer_ready()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500

    offer_sdp = request.get_data(as_text=True) or ""
    if not offer_sdp.strip():
        return jsonify({"ok": False, "message": "empty offer sdp"}), 400

    whep_url = get_whep_url()

    try:
        req = ureq.Request(
            whep_url,
            data=offer_sdp.encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/sdp"},
        )
        with ureq.urlopen(req, timeout=8) as resp:
            answer_sdp = resp.read().decode("utf-8", errors="ignore")
            loc = resp.headers.get("Location") or resp.headers.get("location") or ""

        if not answer_sdp.strip():
            return jsonify({"ok": False, "message": "empty answer from mediamtx"}), 502

        session_url = urljoin(whep_url, loc) if loc else ""
        sid = uuid.uuid4().hex

        with whep_sessions_lock:
            whep_sessions[sid] = session_url

        return jsonify({"ok": True, "answer": answer_sdp, "session_id": sid})

    except uerr.HTTPError as e:
        msg = e.read().decode("utf-8", errors="ignore")
        return jsonify({"ok": False, "message": f"WHEP HTTPError {e.code}: {msg}"}), 502
    except Exception as e:
        return jsonify({"ok": False, "message": f"WHEP proxy error: {e}"}), 502


@app.route("/api/whep_session/<sid>/patch", methods=["PATCH"])
def api_whep_session_patch(sid):
    with whep_sessions_lock:
        session_url = whep_sessions.get(sid, "")

    if not session_url:
        return jsonify({"ok": False, "message": "session not found"}), 404

    body = request.get_data() or b""
    if not body.strip():
        return jsonify({"ok": False, "message": "empty trickle body"}), 400

    try:
        req = ureq.Request(
            session_url,
            data=body,
            method="PATCH",
            headers={
                "Content-Type": request.headers.get("Content-Type", "application/trickle-ice-sdpfrag"),
                "If-Match": request.headers.get("If-Match", "*"),
            },
        )

        with ureq.urlopen(req, timeout=5) as resp:
            _ = resp.read()
            code = getattr(resp, "status", None) or resp.getcode()

        return ("", code if code else 204)

    except uerr.HTTPError as e:
        msg = e.read().decode("utf-8", errors="ignore")
        return jsonify({"ok": False, "message": f"WHEP PATCH HTTPError {e.code}: {msg}"}), 502
    except Exception as e:
        return jsonify({"ok": False, "message": f"WHEP PATCH proxy error: {e}"}), 502


@app.route("/api/whep_options", methods=["OPTIONS"])
def api_whep_options():
    try:
        ensure_streamer_ready()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500

    whep_url = get_whep_url()

    try:
        req = ureq.Request(whep_url, method="OPTIONS")
        with ureq.urlopen(req, timeout=5) as resp:
            links = []
            try:
                links = resp.headers.get_all("Link") or []
            except Exception:
                one = resp.headers.get("Link") or resp.headers.get("link") or ""
                if one:
                    links = [one]

        link_header = ", ".join([x for x in links if x])
        ice_servers = parse_whep_link_header(link_header)

        return jsonify({
            "ok": True,
            "ice_servers": ice_servers,
            "link_header": link_header,
        })

    except uerr.HTTPError as e:
        msg = e.read().decode("utf-8", errors="ignore")
        return jsonify({"ok": False, "message": f"WHEP OPTIONS HTTPError {e.code}: {msg}"}), 502
    except Exception as e:
        return jsonify({"ok": False, "message": f"WHEP OPTIONS proxy error: {e}"}), 502


@app.route("/api/whep_session/<sid>", methods=["DELETE"])
def api_whep_session_delete(sid):
    with whep_sessions_lock:
        session_url = whep_sessions.pop(sid, "")

    if session_url:
        try:
            req = ureq.Request(session_url, method="DELETE")
            ureq.urlopen(req, timeout=3).read()
        except Exception:
            pass

    return jsonify({"ok": True})


# =========================
# 业务 API
# =========================
@app.route("/api/webrtc_info", methods=["GET"])
def api_webrtc_info():
    try:
        ensure_streamer_ready()
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500

    host = request.host.split(":")[0]
    scheme = "https" if request.is_secure else "http"

    return jsonify({
        "ok": True,
        "whep_url_direct": f"{scheme}://{host}:{WEBRTC_HTTP_PORT}/cam/whep",
        "whep_offer_proxy": f"{scheme}://{host}:{PORT}/api/whep_offer",
        "browser_stream_url": f"{scheme}://{host}:{WEBRTC_HTTP_PORT}/cam",
    })


@app.route("/api/webrtc_debug", methods=["GET"])
def api_webrtc_debug():
    """
    返回当前 WebRTC 排障信息。仅用于本地运维页面/命令行确认配置是否生效。
    """
    config_text = ""
    try:
        config_text = MEDIAMTX_CONFIG_PATH.read_text(encoding="utf-8", errors="ignore")
    except Exception:
        pass

    return jsonify({
        "ok": True,
        "mediamtx_running": mediamtx_is_running(),
        "publisher_running": publisher_is_running(),
        "interface_ipv4_addresses": get_interface_ipv4_addresses(),
        "additional_hosts": [],
        "udp_ice_port": WEBRTC_ICE_UDP_PORT,
        "tcp_ice_port": WEBRTC_ICE_TCP_PORT,
        "log_level": MEDIAMTX_LOG_LEVEL,
        "config": config_text,
    })


@app.route("/api/status", methods=["GET"])
def api_status():
    cfg = get_video_config()
    ip = get_ip_address()
    stream_ok = mediamtx_is_running() and publisher_is_running()
    robot_running = is_basecontroller_running()
    robot_state["running"] = robot_running
    stream_error = build_stream_error_text()
    gimbal = g1_get_public_status(refresh_pose=False)
    avoidance = get_obstacle_avoidance_state()

    return jsonify({
        "ok": True,
        "cpu_temp": get_cpu_temp(),
        "ip": ip,
        "stream_mode": "WebRTC / H.264 (rpicam-vid + MediaMTX)",
        "stream_ok": stream_ok,
        "stream_error": stream_error,
        "access_url": f"http://{ip}:{PORT}",
        "resolution": cfg["resolution"],
        "fps": cfg["fps"],
        "bitrate": cfg["bitrate"],
        "recording_ok": recording_is_running(),
        "recording_file": current_recording_file.name if current_recording_file else "",
        "cleaning_on": get_cleaning_state(),
        "foreign_detection_enabled": get_foreign_detection_state().get("enabled", False),
        "foreign_detection_worker_alive": get_foreign_detection_state().get("worker_alive", False),
        "foreign_detection_error": get_foreign_detection_state().get("error", ""),
        "obstacle_avoidance_enabled": avoidance["enabled"],
        "obstacle_avoidance_active": avoidance["active"],
        "obstacle_avoidance_action": avoidance["action"],
        "obstacle_avoidance_direction": avoidance["direction"],
        "obstacle_avoidance_left_overlap": avoidance["left_overlap"],
        "obstacle_avoidance_right_overlap": avoidance["right_overlap"],
        "obstacle_avoidance_status": avoidance["status"],
        "obstacle_avoidance_error": avoidance["last_error"],

        # G1 云台状态
        "gimbal_connected": gimbal["connected"],
        "gimbal_mode": gimbal["mode"],
        "gimbal_mode_name": gimbal["mode_name"],
        "gimbal_moving": gimbal["moving"],
        "gimbal_pose": gimbal["pose"],
        "gimbal_error": gimbal["last_error"],

        # 机器人状态
        "robot_running": robot_running,
        "speed": robot_state["linear_speed"],
        "angular_speed": robot_state["angular_speed"],
        "fan_speed": robot_state["fan_speed"],
        "last_direction": robot_state["last_direction"],
        "control_mode": get_robot_mode_name(),
        "control_mode_value": int(robot_state["control_mode"]),
        "auto_motion_mode": get_auto_motion_mode_name(),
        "published_mode_value": int(robot_state["published_mode_value"]),
        "controller_state": get_controller_state_payload(),
        "basecontroller_error": last_basecontroller_error,
    })


@app.route("/api/system_status", methods=["GET"])
def api_system_status():
    cfg = get_video_config()
    stream_ok = mediamtx_is_running() and publisher_is_running()
    robot_running = is_basecontroller_running()
    stream_error = build_stream_error_text()
    gimbal = g1_get_public_status(refresh_pose=False)
    avoidance = get_obstacle_avoidance_state()

    return jsonify({
        "cpu_temp": get_cpu_temp(),
        "cpu_usage": get_cpu_usage(),
        "mem_usage": get_mem_usage(),
        "stream_mode": "WebRTC / H.264 (rpicam-vid + MediaMTX)",
        "stream_ok": stream_ok,
        "stream_error": stream_error,
        "resolution": cfg["resolution"],
        "fps": cfg["fps"],
        "bitrate": cfg["bitrate"],
        "recording_ok": recording_is_running(),
        "recording_file": current_recording_file.name if current_recording_file else "",
        "cleaning_on": get_cleaning_state(),
        "obstacle_avoidance_enabled": avoidance["enabled"],
        "obstacle_avoidance_active": avoidance["active"],
        "obstacle_avoidance_action": avoidance["action"],
        "obstacle_avoidance_direction": avoidance["direction"],
        "obstacle_avoidance_status": avoidance["status"],
        "gimbal_connected": gimbal["connected"],
        "gimbal_mode": gimbal["mode"],
        "gimbal_mode_name": gimbal["mode_name"],
        "gimbal_moving": gimbal["moving"],
        "gimbal_pose": gimbal["pose"],
        "gimbal_error": gimbal["last_error"],
        "robot_running": robot_running,
        "speed": robot_state["linear_speed"],
        "fan_speed": robot_state["fan_speed"],
        "last_direction": robot_state["last_direction"],
        "control_mode": get_robot_mode_name(),
        "control_mode_value": int(robot_state["control_mode"]),
        "auto_motion_mode": get_auto_motion_mode_name(),
        "published_mode_value": int(robot_state["published_mode_value"]),
    })


@app.route("/api/camera_options", methods=["GET"])
def api_camera_options():
    cfg = get_video_config()
    return jsonify({
        "ok": True,
        "current": cfg,
        "resolution_options": [
            {"label": f"{w}x{h}", "value": f"{w}x{h}", "width": w, "height": h}
            for w, h in RESOLUTION_OPTIONS
        ],
        "fps_options": FPS_OPTIONS,
    })


@app.route("/api/camera_config", methods=["POST"])
def api_camera_config():
    if recording_is_running():
        return jsonify({"ok": False, "message": "请先结束录制，再切换分辨率或 FPS"}), 409

    data = request.get_json(silent=True) or {}
    old_cfg = get_video_config()

    try:
        if "resolution" in data:
            width, height = parse_resolution(data["resolution"])
        else:
            width = int(data.get("width", old_cfg["width"]))
            height = int(data.get("height", old_cfg["height"]))

        fps = int(data.get("fps", old_cfg["fps"]))
        cfg = apply_video_config(width, height, fps)

        return jsonify({
            "ok": True,
            "message": "视频参数已更新",
            "camera": {
                **cfg,
                "resolution": resolution_to_str(cfg["width"], cfg["height"]),
                "idr_period": get_idr_period(cfg["fps"]),
            },
        })
    except Exception as e:
        try:
            apply_video_config(old_cfg["width"], old_cfg["height"], old_cfg["fps"])
        except Exception:
            pass
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/restart_streamer", methods=["POST"])
def api_restart_streamer():
    if recording_is_running():
        return jsonify({"ok": False, "message": "请先结束录制，再重启推流"}), 409

    try:
        stop_publisher()
        stop_mediamtx()
        ensure_streamer_ready()
        return jsonify({"ok": True, "message": "WebRTC 推流服务已重启"})
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/logs", methods=["GET"])
def api_logs():
    return jsonify({
        "ok": True,
        "mediamtx_log_tail": tail_log(MEDIAMTX_LOG_PATH, 120),
        "publisher_log_tail": tail_log(PUBLISHER_LOG_PATH, 120),
        "recording_log_tail": tail_log(RECORDING_LOG_PATH, 120),
        "basecontroller_log_tail": tail_log(BASECONTROLLER_LOG_PATH, 120),
    })


@app.route("/api/capture_photo", methods=["POST"])
def api_capture_photo():
    try:
        out_path = capture_photo_frame()
        return jsonify({
            "ok": True,
            "message": "拍照完成",
            "filename": out_path.name,
            "download_url": f"/api/media/{out_path.name}",
        })
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/start_recording", methods=["POST"])
def api_start_recording():
    try:
        out_path = start_local_recording()
        return jsonify({
            "ok": True,
            "message": "开始录制成功",
            "filename": out_path.name,
            "recording_ok": True,
        })
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/stop_recording", methods=["POST"])
def api_stop_recording():
    try:
        out_path = stop_local_recording()
        return jsonify({
            "ok": True,
            "message": "结束录制成功",
            "filename": out_path.name,
            "recording_ok": False,
            "download_url": f"/api/media/{out_path.name}",
        })
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/start_cleaning", methods=["POST"])
def api_start_cleaning():
    try:
        state = set_cleaning_state(True)
        return jsonify({
            "ok": True,
            "message": "异物清理已开启，风机运行中",
            "cleaning_on": state,
        })
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/stop_cleaning", methods=["POST"])
def api_stop_cleaning():
    try:
        state = set_cleaning_state(False)
        return jsonify({
            "ok": True,
            "message": "异物清理已结束，风机已关闭",
            "cleaning_on": state,
        })
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/cleaning_status", methods=["GET"])
def api_cleaning_status():
    return jsonify({
        "ok": True,
        "cleaning_on": get_cleaning_state(),
    })


# =========================
# 自动避障 API
# =========================
@app.route("/api/obstacle_avoidance", methods=["GET", "POST"])
def api_obstacle_avoidance():
    if request.method == "GET":
        return jsonify({"ok": True, **get_obstacle_avoidance_state()})

    try:
        data = request.get_json(silent=True) or {}
        raw_enabled = data.get("enabled")
        if isinstance(raw_enabled, bool):
            enabled = raw_enabled
        elif isinstance(raw_enabled, str):
            normalized = raw_enabled.strip().lower()
            if normalized in {"1", "true", "on", "yes"}:
                enabled = True
            elif normalized in {"0", "false", "off", "no"}:
                enabled = False
            else:
                raise ValueError("enabled 必须是布尔值")
        else:
            raise ValueError("enabled 必须是布尔值")

        state = set_obstacle_avoidance_enabled(enabled)
        return jsonify({
            "ok": True,
            "message": "自动避障已一键启动" if enabled else "自动避障已关闭，车辆已停止",
            "robot_running": is_basecontroller_running(),
            "control_mode": get_robot_mode_name(),
            "auto_motion_mode": get_auto_motion_mode_name(),
            **state,
        })
    except (ValueError, RuntimeError) as exc:
        return jsonify({
            "ok": False,
            "message": str(exc),
            **get_obstacle_avoidance_state(),
        }), 400
    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": f"自动避障控制失败: {exc}",
            **get_obstacle_avoidance_state(),
        }), 500


# =========================
# 机器人运动控制 API
# =========================
@app.route("/api/robot_start", methods=["POST"])
def api_robot_start():
    ok, message = start_basecontroller()
    code = 200 if ok else 500
    return jsonify({
        "ok": ok,
        "message": message,
        "robot_running": is_basecontroller_running(),
    }), code


@app.route("/api/robot_stop", methods=["POST"])
def api_robot_stop():
    if get_obstacle_avoidance_enabled():
        set_obstacle_avoidance_enabled(False)
    ok, message = stop_basecontroller()
    code = 200 if ok else 500
    return jsonify({
        "ok": ok,
        "message": message,
        "robot_running": is_basecontroller_running(),
    }), code

@app.route("/api/control_mode", methods=["GET", "POST"])
def api_control_mode():
    if request.method == "GET":
        return jsonify({
            "ok": True,
            "control_mode": get_robot_mode_name(),
            "control_mode_value": int(robot_state["control_mode"]),
            "auto_motion_mode": get_auto_motion_mode_name(),
            "published_mode_value": int(robot_state["published_mode_value"]),
            "robot_running": is_basecontroller_running(),
        })

    try:
        data = request.get_json(silent=True) or {}
        raw_mode = data.get("mode", "manual")

        if isinstance(raw_mode, str):
            mode_text = raw_mode.strip().lower()
            if mode_text == "manual":
                mode_value = ROBOT_MODE_MANUAL
            elif mode_text == "auto":
                mode_value = ROBOT_MODE_AUTO
            else:
                raise ValueError("mode 只允许 manual 或 auto")
        else:
            mode_value = int(raw_mode)
            if mode_value not in (ROBOT_MODE_MANUAL, ROBOT_MODE_AUTO):
                raise ValueError("mode 只允许 0(manual) 或 1(auto)")

        if mode_value == ROBOT_MODE_MANUAL and get_obstacle_avoidance_enabled():
            set_obstacle_avoidance_enabled(False)

        # 切模式时先停一下，避免模式切换瞬间车体误动作
        try:
            stop_robot_motion()
        except Exception:
            pass

        actual_mode = set_robot_mode(mode_value)
        robot_state["last_direction"] = "stop"

        return jsonify({
            "ok": True,
            "message": f"模式已切换为 {ROBOT_MODE_NAME_MAP[actual_mode]}",
            "control_mode": ROBOT_MODE_NAME_MAP[actual_mode],
            "control_mode_value": actual_mode,
            "auto_motion_mode": get_auto_motion_mode_name(),
            "published_mode_value": int(robot_state["published_mode_value"]),
            "robot_running": is_basecontroller_running(),
        })
    except Exception as e:
        return jsonify({
            "ok": False,
            "message": f"模式切换失败: {e}",
        }), 400


@app.route("/api/auto_motion_mode", methods=["GET", "POST"])
def api_auto_motion_mode():
    if request.method == "GET":
        return jsonify({
            "ok": True,
            "control_mode": get_robot_mode_name(),
            "auto_motion_mode": get_auto_motion_mode_name(),
            "published_mode_value": int(robot_state["published_mode_value"]),
            "robot_running": is_basecontroller_running(),
        })

    try:
        if get_obstacle_avoidance_enabled():
            raise RuntimeError("自动避障运行中，轨迹由避障程序接管")

        data = request.get_json(silent=True) or {}
        mode_name, mode_value = set_auto_motion_mode(
            data.get("mode", AUTO_MOTION_STRAIGHT)
        )
        return jsonify({
            "ok": True,
            "message": f"自动轨迹已选择: {mode_name}",
            "control_mode": get_robot_mode_name(),
            "auto_motion_mode": mode_name,
            "published_mode_value": mode_value,
            "robot_running": is_basecontroller_running(),
        })
    except Exception as e:
        return jsonify({
            "ok": False,
            "message": f"自动轨迹选择失败: {e}",
        }), 400


@app.route("/api/controller_state", methods=["GET"])
def api_controller_state():
    return jsonify({
        "ok": True,
        **get_controller_state_payload(),
    })

@app.route("/api/set_speed", methods=["POST"])
def api_set_speed():
    try:
        data = request.get_json(silent=True) or {}
        speed = float(data.get("speed", 0.0))
        actual = set_robot_speed(speed)

        return jsonify({
            "ok": True,
            "message": "速度设置成功",
            "speed": actual,
            "robot_running": is_basecontroller_running(),
            "recording_ok": recording_is_running(),
            "cleaning_on": get_cleaning_state(),
            "stream_ok": mediamtx_is_running() and publisher_is_running(),
            "resolution": resolution_to_str(video_state["width"], video_state["height"]),
            "fps": video_state["fps"],
            "fan_speed": robot_state["fan_speed"],
        })
    except Exception as e:
        return jsonify({
            "ok": False,
            "message": f"速度设置失败: {e}",
        }), 400


@app.route("/api/move", methods=["POST"])
def api_move():
    try:
        if get_obstacle_avoidance_enabled():
            raise RuntimeError("自动避障运行中，方向控制由避障程序接管；请先停止避障")

        data = request.get_json(silent=True) or {}

        direction = str(data.get("direction", "")).strip().lower()
        turn_scale = float(data.get("turnScale", 1.0))

        move_robot(direction, turn_scale=turn_scale)

        return jsonify({
            "ok": True,
            "message": f"方向控制成功: {direction}",
            "direction": direction,
            "turn_scale": turn_scale,
            "robot_running": is_basecontroller_running(),
        })
    except Exception as e:
        return jsonify({
            "ok": False,
            "message": f"方向控制失败: {e}",
        }), 400


@app.route("/api/move_stop", methods=["POST"])
def api_move_stop():
    try:
        if get_obstacle_avoidance_enabled():
            set_obstacle_avoidance_enabled(False)
        stop_robot_motion()
        return jsonify({
            "ok": True,
            "message": "机器人已停止",
            "direction": "stop",
            "robot_running": is_basecontroller_running(),
        })
    except Exception as e:
        return jsonify({
            "ok": False,
            "message": f"停止失败: {e}",
        }), 500


@app.route("/api/fan_speed_step", methods=["POST"])
def api_fan_speed_step():
    try:
        data = request.get_json(silent=True) or {}
        delta = int(data.get("delta", 0))
        fan_speed = step_robot_fan_speed(delta)

        return jsonify({
            "ok": True,
            "message": "机器人风扇速度已更新",
            "fan_speed": fan_speed,
        })
    except Exception as e:
        return jsonify({
            "ok": False,
            "message": f"风扇调速失败: {e}",
        }), 400


@app.route("/api/fan_speed_reset", methods=["POST"])
def api_fan_speed_reset():
    try:
        fan_speed = reset_robot_fan_speed()
        return jsonify({
            "ok": True,
            "message": "机器人风扇速度已重置",
            "fan_speed": fan_speed,
        })
    except Exception as e:
        return jsonify({
            "ok": False,
            "message": f"风扇重置失败: {e}",
        }), 500


# =========================
# G1 云台控制 API
# =========================
@app.route("/api/gimbal/status", methods=["GET"])
def api_gimbal_status():
    refresh = str(request.args.get("refresh", "0")).strip().lower() not in {
        "0", "false", "no", "off"
    }
    probe = str(request.args.get("probe", "1")).strip().lower() not in {
        "0", "false", "no", "off"
    }

    status = g1_get_public_status(
        refresh_pose=refresh,
        probe_tcp=probe,
    )

    tcp_ok = bool(status["tcp_reachable"])

    if not tcp_ok:
        message = status["tcp_error"] or "G1 云台 TCP 控制端口未连接"
    elif status["control_ready"]:
        message = "云台控制已连接，控制就绪"
    elif status["busy"]:
        message = "G1 TCP 已连接，云台正在执行控制动作"
    else:
        message = "G1 TCP 已连接，可进行云台控制"

    # 只有调用者明确 refresh=1 时，姿态错误才作为本次请求信息返回；
    # 普通状态轮询不再拿历史 pose_error 覆盖正常控制状态。
    if refresh and status["pose_ok"] is False and status["pose_error"]:
        message += "；本次姿态读取异常：" + status["pose_error"]

    return jsonify({
        "ok": tcp_ok,
        "message": message,
        "gimbal": status,
        "gimbal_pose": status["pose"],
    }), (200 if tcp_ok else 503)


@app.route("/api/gimbal/pose", methods=["GET"])
def api_gimbal_pose():
    try:
        pose = g1_query_pose()
        return jsonify({
            "ok": True,
            "message": "云台姿态读取成功",
            "gimbal_pose": pose,
            "pose_fresh": True,
            "gimbal": g1_get_public_status(refresh_pose=False),
        })
    except Exception as exc:
        status = g1_get_public_status(refresh_pose=False)
        return jsonify({
            "ok": False,
            "message": (
                "云台姿态读取失败（不会影响 TCP/控制状态）: {}"
                .format(exc)
            ),
            "pose_only_error": True,
            "gimbal_connected": bool(status["tcp_reachable"]),
            "gimbal": status,
        }), 503


@app.route("/api/gimbal/jog/pulse", methods=["POST"])
def api_gimbal_jog_pulse():
    """
    一次 HTTP 请求完成稳定动作事务：
    Neutral -> Move -> Stop，并由后端统一节流。
    """
    try:
        data = request.get_json(silent=True) or {}
        axis = str(data.get("axis", "")).strip().lower()
        if axis not in ("yaw", "pitch"):
            raise ValueError("当前页面方向控制只允许 yaw、pitch")

        direction = int(data.get("direction", 1))
        speed = int(round(float(data.get("speed", 17))))
        duration_ms = int(round(float(
            data.get("duration_ms", data.get("durationMs", G1_PULSE_DURATION_MS))
        )))

        # 每次方向点击都由后端完整执行：
        # Neutral -> Move -> Stop -> settle。
        # 页面不再提供独立 Stop，也不再提供横滚方向按钮。
        result = g1_pulse_motion(
            axis=axis,
            direction=direction,
            speed=speed,
            duration_ms=duration_ms,
        )

        action_name = {
            ("yaw", -1): "左转",
            ("yaw", 1): "右转",
            ("pitch", 1): "抬头",
            ("pitch", -1): "低头",
        }.get((axis, -1 if direction < 0 else 1), "云台动作")

        return jsonify({
            "ok": True,
            "message": "{}已执行，云台已自动停止".format(action_name),
            "axis": axis,
            "direction": -1 if direction < 0 else 1,
            "speed": max(1, min(128, speed)),
            "duration_ms": result.get("duration_ms", duration_ms),
            "interrupted": bool(result.get("interrupted")),
            "target_roll": result.get("target_roll"),
            "sequence": result.get("sequence"),
            "pose_fresh": bool(result.get("pose_fresh", False)),
            "gimbal_pose": result.get("pose"),
            "gimbal": g1_get_public_status(refresh_pose=False),
        })

    except (ValueError, TypeError) as exc:
        return jsonify({
            "ok": False,
            "message": "云台控制参数错误: {}".format(exc),
        }), 400

    except G1ControlOperationError as exc:
        app.logger.warning(
            "G1 点按动作失败；自动恢复=%s；原因=%s",
            exc.recovered,
            exc,
        )
        status = g1_get_public_status(refresh_pose=False)
        return jsonify({
            "ok": False,
            "message": (
                "本次云台动作未确认：{}"
                "{}"
            ).format(
                exc,
                "；已自动发送停止并恢复，可继续控制"
                if exc.recovered
                else "；自动停止恢复失败，请先点击停止",
            ),
            "recovered": bool(exc.recovered),
            "recovery_error": exc.recovery_error,
            "gimbal": status,
        }), 503

    except Exception as exc:
        app.logger.exception("G1 云台点按控制失败")
        recovered, recovery_error = _g1_try_recovery_stop(
            "api jog pulse: {}".format(exc)
        )
        return jsonify({
            "ok": False,
            "message": "云台控制失败: {}".format(exc),
            "recovered": bool(recovered),
            "recovery_error": recovery_error,
            "gimbal": g1_get_public_status(refresh_pose=False),
        }), 503


@app.route("/api/gimbal/jog/start", methods=["POST"])
def api_gimbal_jog_start():
    try:
        data = request.get_json(silent=True) or {}
        axis = str(data.get("axis", "")).strip().lower()
        if axis not in ("yaw", "pitch"):
            raise ValueError("当前页面方向控制只允许 yaw、pitch")

        direction = int(data.get("direction", 1))
        speed = int(round(float(data.get("speed", 17))))

        result = g1_start_jog(
            axis=axis,
            direction=direction,
            speed=speed,
        )

        message = (
            "云台横滚步进已执行"
            if result.get("one_shot")
            else "云台开始运动"
        )
        return jsonify({
            "ok": True,
            "message": message,
            "axis": axis,
            "direction": -1 if direction < 0 else 1,
            "speed": max(1, min(128, speed)),
            "one_shot": bool(result.get("one_shot")),
            "interrupted": bool(result.get("interrupted")),
            "target_roll": result.get("target_roll"),
            "sequence": result.get("sequence"),
            "gimbal_pose": g1_get_cached_pose(refresh=False),
            "pose_fresh": False,
            "gimbal": g1_get_public_status(refresh_pose=False),
        })
    except (ValueError, TypeError) as exc:
        return jsonify({
            "ok": False,
            "message": "云台控制参数错误: {}".format(exc),
        }), 400
    except G1ControlOperationError as exc:
        return jsonify({
            "ok": False,
            "message": "云台开始运动失败: {}".format(exc),
            "recovered": bool(exc.recovered),
            "recovery_error": exc.recovery_error,
            "gimbal": g1_get_public_status(refresh_pose=False),
        }), 503
    except Exception as exc:
        recovered, recovery_error = _g1_try_recovery_stop(
            "api jog start: {}".format(exc)
        )
        return jsonify({
            "ok": False,
            "message": "云台开始运动失败: {}".format(exc),
            "recovered": bool(recovered),
            "recovery_error": recovery_error,
            "gimbal": g1_get_public_status(refresh_pose=False),
        }), 503


@app.route("/api/gimbal/jog/stop", methods=["POST"])
def api_gimbal_jog_stop():
    try:
        _, pose = g1_stop_motion(refresh_pose=False)
        return jsonify({
            "ok": True,
            "message": "云台已停止，控制就绪",
            "gimbal_pose": pose,
            "pose_fresh": False,
            "gimbal": g1_get_public_status(refresh_pose=False),
        })
    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": "云台停止失败: {}".format(exc),
            "gimbal": g1_get_public_status(refresh_pose=False),
        }), 503


@app.route("/api/gimbal/reset", methods=["POST"])
def api_gimbal_reset():
    try:
        _, pose = g1_reset_home()
        return jsonify({
            "ok": True,
            "message": "云台已执行回中并恢复控制就绪",
            "gimbal_pose": pose,
            "pose_fresh": False,
            "gimbal": g1_get_public_status(refresh_pose=False),
        })
    except G1ControlOperationError as exc:
        return jsonify({
            "ok": False,
            "message": "云台视角重置失败: {}".format(exc),
            "recovered": bool(exc.recovered),
            "recovery_error": exc.recovery_error,
            "gimbal": g1_get_public_status(refresh_pose=False),
        }), 503
    except Exception as exc:
        recovered, recovery_error = _g1_try_recovery_stop(
            "api reset: {}".format(exc)
        )
        return jsonify({
            "ok": False,
            "message": "云台视角重置失败: {}".format(exc),
            "recovered": bool(recovered),
            "recovery_error": recovery_error,
            "gimbal": g1_get_public_status(refresh_pose=False),
        }), 503


@app.route("/api/gimbal/mode", methods=["GET", "POST"])
def api_gimbal_mode():
    try:
        if request.method == "GET":
            mode = g1_query_mode()
        else:
            data = request.get_json(silent=True) or {}
            mode = g1_set_mode(int(data.get("mode", 0)))

        return jsonify({
            "ok": True,
            "mode": mode,
            "mode_name": G1_MODE_NAME.get(mode, "未知模式"),
            "gimbal": g1_get_public_status(refresh_pose=False),
        })
    except (ValueError, TypeError) as exc:
        return jsonify({"ok": False, "message": str(exc)}), 400
    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": str(exc),
            "gimbal": g1_get_public_status(refresh_pose=False),
        }), 503


# =========================
# 下载 / 舵机 / 关机
# =========================
@app.route("/api/media/<path:filename>", methods=["GET"])
def api_media_download(filename):
    safe_name = os.path.basename(filename)
    path = (CAPTURE_DIR / safe_name).resolve()
    root = CAPTURE_DIR.resolve()

    if path != root and root not in path.parents:
        abort(404)
    if not path.exists():
        abort(404)

    return send_file(path, as_attachment=True, download_name=safe_name)


@app.route("/api/shutdown", methods=["POST"])
def api_shutdown():
    def do_shutdown():
        try:
            stop_obstacle_avoidance_worker()
        except Exception:
            pass
        try:
            stop_gimbal_motion_safe()
        except Exception:
            pass
        try:
            stop_cleaning_fan()
        except Exception:
            pass
        try:
            stop_local_recording()
        except Exception:
            pass
        try:
            stop_publisher()
        except Exception:
            pass
        try:
            stop_mediamtx()
        except Exception:
            pass
        try:
            reset_robot_fan_speed()
        except Exception:
            pass
        try:
            stop_basecontroller()
        except Exception:
            pass
        try:
            shutdown_ros()
        except Exception:
            pass

        time.sleep(1)
        os.system("sudo shutdown -h now")

    threading.Thread(target=do_shutdown, daemon=True).start()
    return jsonify({"ok": True, "message": "系统正在安全关机..."})


# =========================
# 兼容旧接口
# =========================
@app.route("/get_temp")
def get_temp_api():
    return jsonify({"temp": get_cpu_temp_float()})


@app.route("/shutdown")
def shutdown_legacy():
    def do_shutdown():
        try:
            stop_obstacle_avoidance_worker()
        except Exception:
            pass
        try:
            stop_gimbal_motion_safe()
        except Exception:
            pass
        try:
            stop_cleaning_fan()
        except Exception:
            pass
        try:
            stop_local_recording()
        except Exception:
            pass
        try:
            stop_publisher()
        except Exception:
            pass
        try:
            stop_mediamtx()
        except Exception:
            pass
        try:
            stop_basecontroller()
        except Exception:
            pass
        try:
            shutdown_ros()
        except Exception:
            pass
        time.sleep(1)
        os.system("sudo halt")

    threading.Thread(target=do_shutdown, daemon=True).start()
    return "Shutting down..."


# =========================
# 退出清理
# =========================
def cleanup():
    try:
        stop_obstacle_avoidance_worker()
    except Exception:
        pass
    try:
        stop_gimbal_motion_safe()
    except Exception:
        pass
    try:
        stop_webrtc_network_watcher()
    except Exception:
        pass
    try:
        stop_cleaning_fan()
    except Exception:
        pass
    try:
        stop_local_recording()
    except Exception:
        pass
    try:
        stop_foreign_detection_worker()
    except Exception:
        pass
    try:
        stop_detection_worker()
    except Exception:
        pass
    try:
        stop_publisher()
    except Exception:
        pass
    try:
        stop_mediamtx()
    except Exception:
        pass
    try:
        reset_robot_fan_speed()
    except Exception:
        pass
    try:
        stop_basecontroller()
    except Exception:
        pass
    try:
        shutdown_ros()
    except Exception:
        pass
atexit.register(cleanup)


if __name__ == "__main__":
    validate_video_constants()

    try:
        init_cleaning_fan()
        stop_cleaning_fan()
        print("[INFO] Cleaning fan initialized")
    except Exception as e:
        print(f"[WARN] Cleaning fan init failed: {e}")

    print(f"[INFO] Local access : http://127.0.0.1:{PORT}")
    print(f"[INFO] LAN access   : http://{get_ip_address()}:{PORT}")
    print(f"[INFO] G1 control   : {G1_CAMERA_IP}:{G1_CONTROL_PORT}")
    print("[INFO] WebRTC candidate hosts: " + ", ".join(get_webrtc_advertised_hosts()))

    try:
        start_webrtc_network_watcher()
    except Exception as e:
        print(f"[WARN] WebRTC network watcher start failed: {e}")

    try:
        ensure_streamer_ready()
        print("[INFO] Streamer started")
    except Exception as e:
        print(f"[WARN] Streamer pre-start failed: {e}")
        print("[HINT] 查看日志: /api/logs 或 runtime/mediamtx.log + runtime/publisher.log")

    try:
        ensure_detection_worker()
        print(f"[INFO] Detection worker started: WebRTC video + canvas boxes overlay, backend={DETECTION_BACKEND}")
    except Exception as e:
        print(f"[WARN] Detection worker start failed: {e}")

    try:
        ensure_ros_ready()
        print("[INFO] ROS2 initialized")
    except Exception as e:
        print(f"[WARN] ROS2 init failed: {e}")
        print("[HINT] 请确认已 source ROS2 环境与工作空间，再启动 app2.py")

    app.run(host=HOST, port=PORT, threaded=True, debug=False)
