# 첫 번째 코드 영상처리 유지 + 두 번째 코드 master TCP/연속 제품 제어 로직 적용
# 추가: 제품이 ROI 상단을 일부 벗어나도 3꼭짓점 + 100x70mm 규격 복원이
# 신뢰 가능하면 예측 외곽선으로 컨베이어 정지 및 좌표/각도 추출 허용
import cv2
import numpy as np
from collections import deque
import pyrealsense2 as rs
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

# =========================================================
# master.py TCP 통신 설정
# =========================================================
MASTER_HOST = "127.30.1.18"
MASTER_PORT = 5000
MASTER_RECONNECT_INTERVAL_SEC = 1.0
MASTER_CONNECT_TIMEOUT_SEC = 2.0
MASTER_RECV_TIMEOUT_SEC = 0.2
MASTER_RECV_SIZE = 1024

# 제품 전체가 ROI에 들어오면 master를 통해 MCU로 p를 전송하고,
# 이 시간 동안 컨베이어와 영상 좌표가 안정된 뒤 A/G 패킷을 전송한다.
STOP_SETTLE_DELAY_SEC = 2.0 # 1.0->2.0
STOP_SIGNAL = "p\r\n"
MAX_VISION_AGE_SEC = 0.5

# yaw 방향 보정
# +1 : 계산된 yaw 그대로 사용
# -1 : 다이나믹셀 회전 방향이 반대면 -1로 변경
YAW_SIGN = 1

# =========================================================
# XM-430 목표 각도 설정
# =========================================================
# 첫 번째 코드의 각도 계산 기준을 그대로 유지한다.
DXL_CENTER_DEG = 0.0
DXL_MIN_DEG = -90.0
DXL_MAX_DEG = 90.0

# =========================================================
# 좌표 + 각도 one-shot 전송 잠금 설정
# =========================================================
ONE_SHOT_LOCK_MODE = True

# 안정 판정 프레임 수
# 30FPS 기준 5프레임은 약 0.17초라서 크게 느려지지 않으면서 흔들림 판정이 좋아진다.
YAW_STABLE_FRAME_REQUIRED = 5

# 안정화 허용 범위
# 기존 yaw 5도 / center 60px는 너무 넓어서 흔들려도 READY가 쉽게 뜰 수 있었다.
YAW_STABLE_RANGE_DEG = 1.5
CENTER_STABLE_RANGE_PIXEL = 10

# 물체가 사라진 뒤 다음 물체를 받을 수 있게 하는 시간
OBJECT_LOST_UNLOCK_TIME = 2.0

# =========================================================
# 두 번째 코드의 연속 제품 처리 상태 로직
# =========================================================
# 제품 전체가 ROI 내부인 상태가 연속으로 확인되어야 p를 전송한다.
ROI_ENTRY_REQUIRED_FRAMES = 1
PRE_STOP_LOST_REACQUIRE_SEC = 0.25
PICKUP_STATE_FALLBACK_UNLOCK_SEC = 2.0

# STATE 3 직후 화면에 남아 있는 직전 흡착 제품의 재선택 방지값
MAX_CANDIDATE_COUNT = 8
RETIRED_TARGET_SAME_CENTER_RADIUS_PIXEL = 55.0
RETIRED_TARGET_BOX_IOU_THRESHOLD = 0.20
RETIRED_TARGET_FOLLOW_MAX_DISTANCE_PIXEL = 150.0
RETIRED_TARGET_LIFT_DEPTH_DELTA_M = 0.008
RETIRED_TARGET_CLEAR_REQUIRED_FRAMES = 5
RETIRED_TARGET_GUARD_TIMEOUT_SEC = 3.0

one_shot_locked = False

locked_target_deg = None
locked_cx = None
locked_cy = None
locked_z_m = None

last_object_seen_time = 0

# =========================================================
# 자동 배경 저장 설정
# =========================================================
AUTO_BACKGROUND_ENABLE = True
AUTO_BACKGROUND_DELAY_SEC = 5.0

program_start_time = None
auto_background_done = False

# =========================================================
# contour 형상 필터
# =========================================================
MIN_RECT_SHORT_SIDE = 50
MIN_RECT_LONG_SIDE = 120
MAX_ASPECT_RATIO = 4.5

MIN_VERTEX_COUNT = 4
MAX_VERTEX_COUNT = 10

# =========================================================
# 100 x 70 mm 규격 기반 부분 윤곽/세 꼭짓점 복원 설정
# =========================================================
# 정상적인 네 모서리 검출은 기존 영상처리 결과를 그대로 사용하고,
# 일부 영역이 어둡거나 가려져 3개 꼭짓점/부분 윤곽만 남을 때만 보조 복원을 사용한다.
USE_KNOWN_SIZE_COMPLETION = True
PRODUCT_LONG_SIDE_M = 0.100
PRODUCT_SHORT_SIDE_M = 0.070
KNOWN_SIZE_PIXEL_SCALE = 1.0

# 기존 완전 검출 기준에 미달하더라도 규격 복원 후보로 받아들일 최소 조건
PARTIAL_MIN_AREA = 3500
PARTIAL_MIN_FILL_RATIO = 0.18
PARTIAL_MIN_RECT_SHORT_SIDE = 18
PARTIAL_MIN_RECT_LONG_SIDE = 45
PARTIAL_MAX_ASPECT_RATIO = 8.0
PARTIAL_MIN_VERTEX_COUNT = 3
PARTIAL_MAX_VERTEX_COUNT = 20

# Depth로 계산한 예상 100 x 70 mm 크기 대비 허용 범위
KNOWN_SIZE_MAX_EXTENT_RATIO = 1.30
KNOWN_SIZE_MIN_VISIBLE_AREA_RATIO = 0.12
KNOWN_SIZE_MAX_VISIBLE_AREA_RATIO = 1.35

# 관측 길이가 예상 길이의 이 비율 미만이면 누락된 영역이 있다고 판단한다.
KNOWN_SIZE_FULL_EXTENT_RATIO = 0.93

# 기존 환경에서 제품 위쪽이 어두워지는 특성을 반영한다.
# 부분 검출 시 가능한 복원 중심 중 영상 위쪽으로 누락 영역을 보완한다.
COMPLETE_TOWARD_IMAGE_TOP = True

last_valid_yaw = None
last_valid_cx = None
last_valid_cy = None
last_valid_depth_m = None
last_valid_box = None

# STATE 3 이후 직전 흡착 제품을 다시 선택하지 않기 위한 상태
retired_target_center = None
retired_target_depth_m = None
retired_target_box = None
retired_target_started_at = 0.0
retired_target_absent_frames = 0

# =========================================================
# 바운딩 박스 / 좌표 흔들림 방지 설정
# =========================================================
SMOOTHING_ENABLE = True

# 이전값 반영 비율
# 값이 클수록 덜 흔들리지만 반응은 느려진다.
# 0.75~0.88 범위에서 조정하면 된다.
SMOOTH_ALPHA = 0.82

# 이 값보다 작은 변화는 노이즈로 보고 이전값을 유지한다.
CENTER_DEADBAND_PIXEL = 2.0
YAW_DEADBAND_DEG = 0.4
DEPTH_DEADBAND_M = 0.002
SIZE_DEADBAND_PIXEL = 2.0

smoothed_cx = None
smoothed_cy = None
smoothed_yaw = None
smoothed_z_m = None
smoothed_short_side = None
smoothed_long_side = None

# =========================================================
# master.py로 전송할 로봇 좌표 설정
# =========================================================
# 첫 번째 코드와 동일하게 Z축은 안전한 고정값을 사용한다.
STM32_PICK_Z_MM = 10.0

# 현재 제품의 TCP 처리 상태
stop_request_sent = False
stop_request_time = 0.0
capture_result_sent = False
roi_enter_hit_count = 0

# 상단 이탈 3꼭짓점 복구가 한 번 신뢰 가능하게 확인되면 현재 제품 처리 동안 유지한다.
# p 전송 후 제품이 조금 더 이동해 contour 형태가 흔들려도 좌표 안정화가 끊기지 않게 한다.
top_exit_recovery_latched = False

# master 수신 스레드 -> 영상 처리 메인 스레드 이벤트 전달
pickup_started_event = threading.Event()
pickup_completed_event = threading.Event()

# =========================================================
# RealSense / 영상처리 설정값
# =========================================================
WIDTH, HEIGHT, FPS = 1280, 720, 30

# 배경보다 현재 물체가 가까워진 정도를 기준으로 검출
MIN_DEPTH_DIFF = 0.012
MAX_DEPTH_DIFF = 0.100

USE_DEPTH_HOLE_MASK = False

USE_DARK_COLOR_FILTER = False
DARK_GRAY_THRESHOLD = 90

BACKGROUND_VALID_MIN = 50

# =========================================================
# ROI 절대 좌표
# =========================================================
# 영상 처리는 항상 전체 화면에서 수행한다.
# ROI는 제품 전체 진입 여부와 좌표 추출/전송 허용 조건으로만 사용한다.
USE_ROI = True
ROI_X1, ROI_Y1 = 362, 0
ROI_X2, ROI_Y2 = 920, 365 # 375->370->365

# 박스가 ROI 안에 완전히 들어왔는지 확인할 때 사용할 여유값
REQUIRE_FULL_BOX_IN_ROI = True
ROI_INNER_MARGIN = 1    # 2 ->

# =========================================================
# ROI/영상 상단 이탈 제품 규격 복원 허용 설정
# =========================================================
# 제품은 화면 아래에서 위로 이동한다. 정상적으로는 전체 제품이 ROI 안에
# 들어왔을 때 정지하지만, 인식 또는 제동 지연으로 제품 일부가 ROI/영상 상단을
# 벗어난 경우에는 보이는 윤곽과 100 x 70 mm 규격으로 전체 사각형을 복원한다.
ALLOW_TOP_EXIT_THREE_CORNER_RECOVERY = True

# 실제 윤곽은 표면 반사와 Depth 구멍 때문에 꼭짓점 수가 3으로 정확히 나오지 않을 수 있다.
# 따라서 정확히 3개 꼭짓점인 경우만 강제하지 않고, 상단에 닿은 부분 윤곽과
# 고정 규격 복원이 모두 유효하면 복구를 허용한다.
TOP_EXIT_RECOVERY_REQUIRE_THREE_CORNER = False

# 복원된 제품 사각형이 ROI 상단 밖으로 허용되는 최대 거리.
# 대각선 제품의 위쪽 꼭짓점이 영상 밖으로 나가는 경우까지 포함한다.
TOP_EXIT_RECOVERY_MAX_OVERFLOW_PX = 220

# 실제 관측 contour가 ROI/영상 상단에 닿았다고 판단할 허용 범위.
TOP_EXIT_RECOVERY_TOP_TOUCH_TOLERANCE_PX = 30

# 예상 전체 제품 면적 대비 실제로 보이는 contour 면적의 최소 비율.
# 사진과 같이 세 모서리와 제품 대부분이 보이는 경우를 허용한다.
TOP_EXIT_RECOVERY_MIN_VISIBLE_AREA_RATIO = 0.20

# 좌우 및 아래쪽 경계의 작은 contour 잡음을 허용하는 픽셀 여유값.
TOP_EXIT_RECOVERY_SIDE_TOLERANCE_PX = 8

MIN_AREA = 15000
MAX_AREA = 99999999

# 기존 0.40보다 살짝 낮춤
# 박스 표면 반사/구멍 때문에 마스크가 울퉁불퉁해도 검출 유지
MIN_FILL_RATIO = 0.35

USE_DEPTH_FILTERS = True

# =========================================================
# 고정 ArUco 원점 좌표계 설정
# =========================================================
# 사용 흐름:
# 1) ArUco 확인 코드에서 rvec, tvec 값을 확인한다.
# 2) 아래 FIXED_ARUCO_RVEC, FIXED_ARUCO_TVEC에 직접 넣는다.
# 3) 이 박스 검출 코드만 실행하면 저장 당시 ArUco 마커 중심이 Robot (0, 0)이 된다.
USE_FIXED_ARUCO_ORIGIN = True

# 현재 저장해둔 ArUco pose 값
# 의미: 이 pose로 계산되는 ArUco 마커 중심 = Robot X 0mm, Y 0mm
FIXED_ARUCO_RVEC = np.array(
    [0.02454024, -3.09082913, 0.00807591],
    dtype=np.float32
)

FIXED_ARUCO_TVEC = np.array(
    [0.05427309, -0.0591134, 0.37098354],
    dtype=np.float32
)

# 좌표축 방향 설정
# 기본값:
#   Robot X: ArUco 마커 오른쪽 방향 +
#   Robot Y: 화면/컨베이어 아래쪽 방향 +
# 결과가 반대로 나오면 부호만 바꾸면 된다.
ROBOT_X_SIGN = -1.0
ROBOT_Y_SIGN = 1.0

# 마커 중심과 실제 로봇 원점이 다르면 보정값을 넣는다.
# 단위: mm
ROBOT_OFFSET_X_MM = 4.0
ROBOT_OFFSET_Y_MM = -1.5

# RealSense color intrinsics 저장용
CAM_FX = None
CAM_FY = None
CAM_PPX = None
CAM_PPY = None


yaw_hist = deque(maxlen=YAW_STABLE_FRAME_REQUIRED)
cx_hist = deque(maxlen=YAW_STABLE_FRAME_REQUIRED)
cy_hist = deque(maxlen=YAW_STABLE_FRAME_REQUIRED)

DEBUG_DIFF_PRINT = False
DEBUG_PRINT_INTERVAL = 0.5
last_debug_print_time = 0


# =========================================================
# master.py TCP 통신 클래스
# =========================================================
@dataclass(frozen=True)
class MasterConfig:
    host: str = "127.30.1.18"
    port: int = 5000
    reconnect_interval_sec: float = 1.0
    connect_timeout_sec: float = 2.0
    recv_timeout_sec: float = 0.2
    recv_size: int = 1024


@dataclass
class VisionData:
    x_mm: float
    y_mm: float
    z_mm: float
    target_deg: float
    pixel_x: int
    pixel_y: int
    depth_m: float
    marker_z_mm: float
    yaw_deg: float
    updated_at: float = field(default_factory=time.time)


class OneShotFlags:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.angle_sent = False
        self.coord_sent = False

    def reset(self, print_log: bool = False) -> None:
        with self.lock:
            self.angle_sent = False
            self.coord_sent = False

        if print_log:
            print("좌표/각도 1회 전송 플래그 초기화")

    def consume_angle_send_chance(self) -> bool:
        with self.lock:
            if self.angle_sent:
                return False
            self.angle_sent = True
            return True

    def restore_angle_send_chance(self) -> None:
        with self.lock:
            self.angle_sent = False

    def consume_coord_send_chance(self) -> bool:
        with self.lock:
            if self.coord_sent:
                return False
            self.coord_sent = True
            return True

    def restore_coord_send_chance(self) -> None:
        with self.lock:
            self.coord_sent = False

    def is_angle_sent(self) -> bool:
        with self.lock:
            return self.angle_sent

    def is_coord_sent(self) -> bool:
        with self.lock:
            return self.coord_sent


class EventLogger:
    def __init__(self) -> None:
        self._flags = {}
        self._values = {}

    def once(self, key: str, msg: str) -> None:
        if self._flags.get(key):
            return
        print(msg)
        self._flags[key] = True

    def reset(self, *keys: str) -> None:
        if not keys:
            self._flags.clear()
            self._values.clear()
            return
        for key in keys:
            self._flags.pop(key, None)
            self._values.pop(key, None)

    def on_change(self, key: str, value, msg: str) -> None:
        if self._values.get(key) == value:
            return
        print(msg)
        self._values[key] = value


class MasterTcpClient:
    def __init__(self, config: MasterConfig, flags: OneShotFlags) -> None:
        self.config = config
        self.flags = flags
        self.sock: Optional[socket.socket] = None
        self.connected = False
        self.running = False
        self.lock = threading.Lock()

        self.latest_lock = threading.Lock()
        self.latest_vision: Optional[VisionData] = None

        self.trigger_lock = threading.Lock()
        self.sensor_trigger_pending = False

        self.log = EventLogger()

        # MCU 상태(STATE 1~7)를 비전 추적 객체에 전달하기 위한 콜백이다.
        self.robot_state_callback: Optional[Callable[[int], None]] = None

    def set_robot_state_callback(self, callback: Callable[[int], None]) -> None:
        self.robot_state_callback = callback

    def start(self) -> None:
        self.running = True
        threading.Thread(target=self._connect_loop, daemon=True, name="master_tcp_rx").start()

    def stop(self) -> None:
        self.running = False
        with self.lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None
            self.connected = False

    def update_latest_vision(self, vision: VisionData) -> None:
        """최신 비전 결과만 저장한다.

        실제 전송 시점은 p 전송 후 1초 경과 및 안정 상태 확인 뒤이다.
        """
        with self.latest_lock:
            self.latest_vision = vision

    def clear_latest_vision(self) -> None:
        with self.latest_lock:
            self.latest_vision = None

        self.log.reset("coord_stale", "coord_pending")

    def _connect_loop(self) -> None:
        rx_buffer = ""

        while self.running:
            if self.sock is None:
                if not self._try_connect():
                    time.sleep(self.config.reconnect_interval_sec)
                    continue
                rx_buffer = ""

            try:
                assert self.sock is not None
                data = self.sock.recv(self.config.recv_size)
                if not data:
                    raise ConnectionError("master disconnected")

                rx_buffer += data.decode("utf-8", errors="ignore")

                while "\n" in rx_buffer:
                    line, rx_buffer = rx_buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        self._handle_master_command(line)

            except socket.timeout:
                continue
            except Exception as exc:
                print("master.py TCP 연결 끊김:", exc)
                self._close_socket()
                time.sleep(self.config.reconnect_interval_sec)

    def _try_connect(self) -> bool:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.config.connect_timeout_sec)
            sock.connect((self.config.host, self.config.port))
            # p와 같은 짧은 제어 패킷이 Nagle 알고리즘 때문에 묶이지 않도록 한다.
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.config.recv_timeout_sec)

            with self.lock:
                self.sock = sock
                self.connected = True

            print(f"master.py TCP 연결 완료: {self.config.host}:{self.config.port}")
            self.log.reset("connect_fail")
            return True

        except OSError:
            with self.lock:
                self.sock = None
                self.connected = False
            self.log.once("connect_fail", "master.py TCP 연결 대기 중")
            return False

    def _close_socket(self) -> None:
        with self.lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except OSError:
                    pass
            self.sock = None
            self.connected = False

    def _handle_master_command(self, line: str) -> None:
        upper = line.upper()

        if line == "0" or upper == "SENSOR 0":
            self.log.once("legacy_sensor_0", "기존 SENSOR 0 수신 - 카메라 정지 방식에서는 사용하지 않음")
            return

        if upper.startswith("STATE "):
            parts = upper.split()

            if len(parts) != 2:
                self.log.on_change("invalid_robot_state", upper, f"잘못된 로봇 상태 형식: {line}")
                return

            try:
                state_num = int(parts[1])
            except ValueError:
                self.log.on_change("invalid_robot_state", upper, f"잘못된 로봇 상태 값: {line}")
                return

            self.log.on_change(
                "robot_state",
                state_num,
                f"로봇 동작 상태 수신: STATE {state_num}",
            )

            callback = self.robot_state_callback
            if callback is not None:
                callback(state_num)

            return

        self.log.on_change("master_rx", line, f"RX from master: {line}")

    def send_stop_request(self) -> bool:
        """master.py를 통해 MCU로 전달할 컨베이어 정지 신호를 보낸다."""
        return self._send_tcp(STOP_SIGNAL, "TX conveyor stop to master")

    def send_angle(self, target_deg: float) -> bool:
        return self._send_tcp("A {:.2f}\n".format(float(target_deg)), "TX angle to master")

    def send_stable_result(self) -> bool:
        """정지 후 안정된 최신 각도와 좌표를 각각 한 번만 전송한다."""
        with self.latest_lock:
            vision = self.latest_vision

        if vision is None:
            return False

        if not self.flags.is_angle_sent():
            if self.flags.consume_angle_send_chance():
                if not self.send_angle(vision.target_deg):
                    self.flags.restore_angle_send_chance()
                    return False

        return self.send_latest_vision()
    
    def send_latest_vision(self) -> bool:
        with self.latest_lock:
            vision = self.latest_vision

        if vision is None:
            return False

        if time.time() - vision.updated_at > MAX_VISION_AGE_SEC:
            self.log.once("coord_stale", "좌표가 오래되어 전송하지 않음")
            return False

        if not self.flags.consume_coord_send_chance():
            self.log.once("coord_skip", "좌표는 이미 1회 전송됨 - 중복 전송 생략")
            return True

        # master.py의 _parse_vision_packet()은 공백/콤마 구분을 모두 허용하지만,
        # master.py -> MCU 전송 포맷이 G,x,y,z 형태이므로 blackbox -> master도 콤마 구분으로 맞춘다.
        # 좌표는 소수점 1자리, 각도는 소수점 2자리로 전송한다.
        msg = "G,{:.1f},{:.1f},{:.1f},{:.2f}\n".format(
            float(vision.x_mm),
            float(vision.y_mm),
            float(vision.z_mm),
            vision.target_deg,
        )

        if not self._send_tcp(msg, "TX to master"):
            self.flags.restore_coord_send_chance()
            return False

        self.log.reset("coord_pending", "coord_stale")
        return True

    def _send_tcp(self, msg: str, label: str) -> bool:
        # socket 참조만 lock 안에서 가져오고 실제 send는 lock 밖에서 수행한다.
        # 전송 예외에서 _close_socket()이 같은 lock을 다시 획득하는 교착을 방지한다.
        with self.lock:
            sock = self.sock

        if sock is None:
            return False

        try:
            sock.sendall(msg.encode("utf-8"))
            print(f"{label}: {msg.strip()}")
            return True
        except OSError as exc:
            print(f"master.py 전송 오류: {exc}")
            self._close_socket()
            return False


def handle_robot_state(state_num):
    if state_num == 3:
        pickup_started_event.set()
    elif state_num == 4:
        pickup_completed_event.set()


master_flags = OneShotFlags()
master_client = MasterTcpClient(MasterConfig(), master_flags)
master_client.set_robot_state_callback(handle_robot_state)


# =========================================================
# 함수 정의
# =========================================================
def angle_diff_180(a, b):
    """
    180도 주기를 가지는 각도 차이 계산.
    minAreaRect / yaw는 -90~90 근처에서 표현이 바뀔 수 있으므로
    단순 뺄셈 대신 이 함수를 사용한다.
    """

    return (float(a) - float(b) + 90.0) % 180.0 - 90.0


def normalize_angle_180(angle):
    """
    각도를 -90~90 범위로 정리한다.
    """

    angle = float(angle)

    while angle > 90.0:
        angle = angle - 180.0

    while angle < -90.0:
        angle = angle + 180.0

    return angle


def angle_span_180(values):
    """
    yaw 히스토리의 흔들림 폭을 계산한다.
    단순 max-min보다 -90/90 경계에서 안전하다.
    """

    if len(values) == 0:
        return 9999.0

    ref = float(np.median(values))

    max_diff = 0.0

    for value in values:
        diff = abs(angle_diff_180(value, ref))

        if diff > max_diff:
            max_diff = diff

    return max_diff


def apply_deadband_scalar(prev, curr, deadband):
    """
    변화량이 deadband보다 작으면 이전값을 유지한다.
    """

    if prev is None:
        return float(curr)

    if abs(float(curr) - float(prev)) < deadband:
        return float(prev)

    return float(curr)


def apply_deadband_angle(prev, curr, deadband):
    """
    각도용 deadband.
    """

    if prev is None:
        return normalize_angle_180(curr)

    diff = angle_diff_180(curr, prev)

    if abs(diff) < deadband:
        return normalize_angle_180(prev)

    return normalize_angle_180(curr)


def ema_scalar(prev, curr, alpha):
    """
    지수 이동 평균.
    alpha는 이전값 비율이다.
    """

    if prev is None:
        return float(curr)

    return alpha * float(prev) + (1.0 - alpha) * float(curr)


def ema_angle_180(prev, curr, alpha):
    """
    180도 주기를 고려한 각도 EMA.
    """

    curr = normalize_angle_180(curr)

    if prev is None:
        return curr

    diff = angle_diff_180(curr, prev)
    result = float(prev) + (1.0 - alpha) * diff

    return normalize_angle_180(result)


def update_smoothed_detection(raw_cx, raw_cy, raw_yaw, raw_z_m, raw_short_side, raw_long_side):
    """
    center, yaw, depth, box size를 한 번에 안정화한다.
    화면에 그리는 바운딩 박스와 로봇으로 변환하는 좌표가
    프레임마다 흔들리는 것을 줄이기 위한 함수.
    """

    global smoothed_cx
    global smoothed_cy
    global smoothed_yaw
    global smoothed_z_m
    global smoothed_short_side
    global smoothed_long_side

    raw_cx = float(raw_cx)
    raw_cy = float(raw_cy)
    raw_yaw = normalize_angle_180(raw_yaw)
    raw_short_side = float(raw_short_side)
    raw_long_side = float(raw_long_side)

    if SMOOTHING_ENABLE == False:
        smoothed_cx = raw_cx
        smoothed_cy = raw_cy
        smoothed_yaw = raw_yaw

        if raw_z_m > 0:
            smoothed_z_m = float(raw_z_m)

        smoothed_short_side = raw_short_side
        smoothed_long_side = raw_long_side

        return (
            int(round(smoothed_cx)),
            int(round(smoothed_cy)),
            smoothed_yaw,
            0.0 if smoothed_z_m is None else smoothed_z_m,
            smoothed_short_side,
            smoothed_long_side
        )

    # 1) 너무 작은 변화는 이전값 유지
    cx_db = apply_deadband_scalar(smoothed_cx, raw_cx, CENTER_DEADBAND_PIXEL)
    cy_db = apply_deadband_scalar(smoothed_cy, raw_cy, CENTER_DEADBAND_PIXEL)
    yaw_db = apply_deadband_angle(smoothed_yaw, raw_yaw, YAW_DEADBAND_DEG)
    short_db = apply_deadband_scalar(smoothed_short_side, raw_short_side, SIZE_DEADBAND_PIXEL)
    long_db = apply_deadband_scalar(smoothed_long_side, raw_long_side, SIZE_DEADBAND_PIXEL)

    # depth가 0이면 이전 depth를 유지한다.
    if raw_z_m > 0:
        z_db = apply_deadband_scalar(smoothed_z_m, raw_z_m, DEPTH_DEADBAND_M)
    else:
        z_db = smoothed_z_m

    # 2) EMA smoothing
    smoothed_cx = ema_scalar(smoothed_cx, cx_db, SMOOTH_ALPHA)
    smoothed_cy = ema_scalar(smoothed_cy, cy_db, SMOOTH_ALPHA)
    smoothed_yaw = ema_angle_180(smoothed_yaw, yaw_db, SMOOTH_ALPHA)
    smoothed_short_side = ema_scalar(smoothed_short_side, short_db, SMOOTH_ALPHA)
    smoothed_long_side = ema_scalar(smoothed_long_side, long_db, SMOOTH_ALPHA)

    if z_db is not None:
        smoothed_z_m = ema_scalar(smoothed_z_m, z_db, SMOOTH_ALPHA)

    return (
        int(round(smoothed_cx)),
        int(round(smoothed_cy)),
        smoothed_yaw,
        0.0 if smoothed_z_m is None else smoothed_z_m,
        smoothed_short_side,
        smoothed_long_side
    )


def normalize_yaw(angle, w, h):
    """
    minAreaRect에서 얻은 각도를 긴 변 기준 yaw로 정리한다.
    긴 변이 세로이면 yaw = 0도 기준으로 사용한다.
    """

    if w < h:
        yaw = angle + 90
    else:
        yaw = angle

    if yaw > 90:
        yaw = yaw - 180

    if yaw < -90:
        yaw = yaw + 180

    # 가로 기준 -> 세로 기준
    yaw = yaw - 90

    if yaw > 90:
        yaw = yaw - 180

    if yaw < -90:
        yaw = yaw + 180

    return yaw


def clean_mask(mask):
    """
    Depth mask 정리 함수.
    외곽을 과하게 부풀리지 않도록 close 커널을 작게 사용한다.
    """

    # 작은 노이즈 제거용
    small_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    # 끊어진 외곽 연결용
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))

    # 작은 노이즈 제거
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, small_kernel, iterations=1)

    # 끊어진 영역 연결
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=1)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    filled = np.zeros_like(mask)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > 300:
            cv2.drawContours(filled, [cnt], -1, 255, -1)

    # 마지막 작은 점 제거
    filled = cv2.morphologyEx(filled, cv2.MORPH_OPEN, small_kernel, iterations=1)

    return filled


def limit_value(value, min_value, max_value):
    if value < min_value:
        value = min_value

    if value > max_value:
        value = max_value

    return value


def yaw_to_dxl_target_deg(yaw):
    """
    yaw 값을 OpenRB로 보낼 목표 각도로 변환한다.

    yaw = 0도    -> 0도
    yaw = +30도  -> +30도
    yaw = -30도  -> -30도
    """

    target_deg = YAW_SIGN * float(yaw)
    target_deg = limit_value(target_deg, DXL_MIN_DEG, DXL_MAX_DEG)

    return target_deg


def is_detection_stable():
    """
    yaw와 중심 좌표가 모두 안정되었는지 판단한다.
    """

    if len(yaw_hist) < YAW_STABLE_FRAME_REQUIRED:
        return False

    if len(cx_hist) < YAW_STABLE_FRAME_REQUIRED:
        return False

    if len(cy_hist) < YAW_STABLE_FRAME_REQUIRED:
        return False

    yaw_values = list(yaw_hist)
    cx_values = list(cx_hist)
    cy_values = list(cy_hist)

    yaw_range = angle_span_180(yaw_values)
    cx_range = max(cx_values) - min(cx_values)
    cy_range = max(cy_values) - min(cy_values)

    if yaw_range > YAW_STABLE_RANGE_DEG:
        return False

    if cx_range > CENTER_STABLE_RANGE_PIXEL:
        return False

    if cy_range > CENTER_STABLE_RANGE_PIXEL:
        return False

    return True


def reset_one_shot_lock():
    """
    다음 물체를 위해 one-shot 잠금 상태와 안정화 기준값을 초기화한다.
    """

    global one_shot_locked
    global locked_target_deg
    global locked_cx
    global locked_cy
    global locked_z_m
    global last_valid_yaw
    global last_valid_cx
    global last_valid_cy
    global last_valid_depth_m
    global last_valid_box
    global retired_target_center
    global retired_target_depth_m
    global retired_target_box
    global retired_target_started_at
    global retired_target_absent_frames
    global smoothed_cx
    global smoothed_cy
    global smoothed_yaw
    global smoothed_z_m
    global smoothed_short_side
    global smoothed_long_side

    one_shot_locked = False

    locked_target_deg = None
    locked_cx = None
    locked_cy = None
    locked_z_m = None

    last_valid_yaw = None
    last_valid_cx = None
    last_valid_cy = None
    last_valid_depth_m = None
    last_valid_box = None

    retired_target_center = None
    retired_target_depth_m = None
    retired_target_box = None
    retired_target_started_at = 0.0
    retired_target_absent_frames = 0

    smoothed_cx = None
    smoothed_cy = None
    smoothed_yaw = None
    smoothed_z_m = None
    smoothed_short_side = None
    smoothed_long_side = None

    # 현재 제품의 master TCP 전송 상태도 함께 초기화한다.
    global stop_request_sent
    global stop_request_time
    global capture_result_sent
    global roi_enter_hit_count
    global top_exit_recovery_latched

    stop_request_sent = False
    stop_request_time = 0.0
    capture_result_sent = False
    roi_enter_hit_count = 0
    top_exit_recovery_latched = False

    master_flags.reset()
    master_client.clear_latest_vision()


def is_valid_measurement(yaw_raw, cx, cy):
    """
    측정값 유효성 판단 함수.
    현재는 다양한 박스 각도를 허용하기 위해 항상 True.
    """

    return True


def is_box_fully_inside_roi(box, x1, y1, x2, y2, margin=0):
    """
    회전 사각형의 4개 꼭짓점이 모두 ROI 안에 들어왔는지 확인한다.
    현재 최종 로직에서는 contour local 기준 함수를 주로 사용한다.
    """

    for pt in box:
        px = int(pt[0])
        py = int(pt[1])

        if px < x1 + margin:
            return False

        if px > x2 - margin:
            return False

        if py < y1 + margin:
            return False

        if py > y2 - margin:
            return False

    return True


def is_contour_fully_inside_roi_global(cnt, x1, y1, x2, y2, margin=0):
    """
    전체 이미지 좌표계 contour의 모든 점이 ROI 내부에 있는지 확인한다.

    회전 사각형 네 꼭짓점뿐 아니라 실제 정제 contour까지 ROI 내부여야
    제품 전체가 들어온 것으로 판정한다.
    """

    if cnt is None or len(cnt) < 3:
        return False

    points = np.asarray(cnt, dtype=np.float32).reshape(-1, 2)

    if not np.all(np.isfinite(points)):
        return False

    margin = max(0, int(margin))

    return bool(
        np.all(points[:, 0] >= x1 + margin) and
        np.all(points[:, 0] < x2 - margin) and
        np.all(points[:, 1] >= y1 + margin) and
        np.all(points[:, 1] < y2 - margin)
    )


def is_top_exit_three_corner_recovery_valid(
    box,
    contour,
    known_fit,
    x1,
    y1,
    x2,
    y2,
    margin=0,
    latched_mode=False,
):
    """
    제품이 아래에서 위로 이동하여 ROI 또는 영상 상단에서 일부 잘린 경우,
    100 x 70 mm 규격 복원 결과를 정지 및 좌표 추출에 사용할 수 있는지 판단한다.

    주의:
    Depth 윤곽은 표면 반사와 영상 경계 절단 때문에 approxPolyDP 결과가 정확히
    3개 꼭짓점으로 나오지 않을 수 있다. 따라서 꼭짓점 개수만 강제하지 않고,
    상단 접촉, 규격 복원 사각형, 실제 가시 면적과 이동 방향을 함께 검사한다.
    """

    if not ALLOW_TOP_EXIT_THREE_CORNER_RECOVERY:
        return False

    if box is None or contour is None or known_fit is None:
        return False

    box_points = np.asarray(box, dtype=np.float32).reshape(-1, 2)
    contour_points = np.asarray(contour, dtype=np.float32).reshape(-1, 2)

    if len(box_points) < 4 or len(contour_points) < 3:
        return False
    if not np.all(np.isfinite(box_points)):
        return False
    if not np.all(np.isfinite(contour_points)):
        return False

    three_corner_used = bool(known_fit.get("three_corner_used", False))
    completion_used = bool(known_fit.get("completion_used", False))

    if (
        TOP_EXIT_RECOVERY_REQUIRE_THREE_CORNER
        and not latched_mode
        and not three_corner_used
    ):
        return False

    # 고정 규격 함수가 반환한 결과만 사용한다. 정확히 3꼭짓점 경로가 아니더라도
    # 일반 100 x 70 mm 방향 복원 결과이면 상단 절단 복구에 사용할 수 있다.
    if not (three_corner_used or completion_used or known_fit.get("rect") is not None):
        return False

    visible_area_ratio = float(known_fit.get("visible_area_ratio", 0.0))
    if visible_area_ratio < TOP_EXIT_RECOVERY_MIN_VISIBLE_AREA_RATIO:
        return False

    m = max(0, int(margin))
    side_tol = max(0, int(TOP_EXIT_RECOVERY_SIDE_TOLERANCE_PX))

    box_min_x = float(np.min(box_points[:, 0]))
    box_max_x = float(np.max(box_points[:, 0]))
    box_min_y = float(np.min(box_points[:, 1]))
    box_max_y = float(np.max(box_points[:, 1]))
    box_center_x = float(np.mean(box_points[:, 0]))
    box_center_y = float(np.mean(box_points[:, 1]))

    top_limit = float(y1 + m)
    top_overflow_px = top_limit - box_min_y

    # 복원 사각형이 실제로 상단을 벗어난 경우만 복구 모드로 인정한다.
    if top_overflow_px <= 0.0:
        return False
    if top_overflow_px > float(TOP_EXIT_RECOVERY_MAX_OVERFLOW_PX):
        return False

    # 제품은 아래에서 위로 이동하므로 좌우 또는 아래쪽까지 크게 벗어난 후보는 제외한다.
    if box_min_x < float(x1 + m - side_tol):
        return False
    if box_max_x >= float(x2 - m + side_tol):
        return False
    if box_max_y >= float(y2 - m + side_tol):
        return False

    # 복원 중심은 로봇이 접근 가능한 ROI 안에 남아 있어야 한다.
    if not (
        float(x1 + m) <= box_center_x < float(x2 - m)
        and float(y1) <= box_center_y < float(y2 - m)
    ):
        return False

    # 실제 관측 contour의 좌우/아래쪽은 ROI 안에 있어야 한다.
    if np.any(contour_points[:, 0] < float(x1 + m - side_tol)):
        return False
    if np.any(contour_points[:, 0] >= float(x2 - m + side_tol)):
        return False
    if np.any(contour_points[:, 1] >= float(y2 - m + side_tol)):
        return False

    # 최초 복구 시 실제 contour가 ROI 또는 영상 상단에 닿았는지 확인한다.
    contour_min_y = float(np.min(contour_points[:, 1]))
    top_touch_limit = float(
        y1 + m + TOP_EXIT_RECOVERY_TOP_TOUCH_TOLERANCE_PX
    )
    if not latched_mode and contour_min_y > top_touch_limit:
        return False

    return True

def is_contour_fully_inside_roi_local(cnt, roi_w, roi_h, margin=2):
    """
    ROI 내부에서 검출된 contour가 ROI 경계에 닿았는지 확인한다.
    contour 좌표는 ROI 기준 local 좌표이다.
    contour가 ROI 경계에 닿으면 박스가 잘렸을 가능성이 있으므로 False.
    """

    xs = cnt[:, 0, 0]
    ys = cnt[:, 0, 1]

    if np.min(xs) <= margin:
        return False

    if np.max(xs) >= roi_w - margin:
        return False

    if np.min(ys) <= margin:
        return False

    if np.max(ys) >= roi_h - margin:
        return False

    return True


def patch_median_depth_m(depth_u16, depth_scale, u, v, r=10, min_valid=8):
    """
    중심 픽셀 depth가 0으로 나오는 경우가 있어서
    주변 patch의 median depth를 사용한다.
    """

    h, w = depth_u16.shape[:2]

    u = int(round(u))
    v = int(round(v))

    u = max(0, min(w - 1, u))
    v = max(0, min(h - 1, v))

    x1 = max(0, u - r)
    x2 = min(w - 1, u + r)
    y1 = max(0, v - r)
    y2 = min(h - 1, v + r)

    roi = depth_u16[y1:y2 + 1, x1:x2 + 1]

    if roi.size == 0:
        return 0.0

    valid = roi[roi > 0].astype(np.float32) * float(depth_scale)

    if valid.size < min_valid:
        return 0.0

    return float(np.median(valid))



def pixel_depth_to_camera_point(u, v, z_m):
    """
    픽셀 좌표 + depth를 카메라 3D 좌표계로 변환한다.

    카메라 좌표계:
        X: 카메라 기준 오른쪽 +
        Y: 카메라 기준 아래쪽 +
        Z: 카메라 정면 +
    단위: m
    """

    if CAM_FX is None or CAM_FY is None:
        return None

    if CAM_PPX is None or CAM_PPY is None:
        return None

    if z_m <= 0:
        return None

    x_cam = (float(u) - CAM_PPX) / CAM_FX * float(z_m)
    y_cam = (float(v) - CAM_PPY) / CAM_FY * float(z_m)
    z_cam = float(z_m)

    return np.array([x_cam, y_cam, z_cam], dtype=np.float32)


def camera_point_to_fixed_aruco_point(p_cam):
    """
    카메라 좌표계의 점을 고정된 ArUco 마커 중심 기준 좌표계로 변환한다.

    solvePnP 관계:
        P_cam = R * P_marker + t

    역변환:
        P_marker = R.T * (P_cam - t)

    여기서 P_marker의 원점은 ArUco 마커 중심이다.
    따라서 이 좌표계에서 ArUco 마커 중심은 항상 (0, 0, 0)이다.
    """

    if p_cam is None:
        return None

    R_marker_to_cam, _ = cv2.Rodrigues(FIXED_ARUCO_RVEC)

    p_cam_col = np.array(p_cam, dtype=np.float32).reshape(3, 1)
    tvec_col = np.array(FIXED_ARUCO_TVEC, dtype=np.float32).reshape(3, 1)

    p_marker_col = np.dot(
        R_marker_to_cam.T,
        p_cam_col - tvec_col
    )

    return p_marker_col.flatten()


def pixel_depth_to_robot_xy_mm(u, v, z_m):
    """
    픽셀 좌표 + depth를 저장된 ArUco 중심 기준 Robot XY[mm]로 변환한다.

    수정:
    - 기존: p_marker[0] -> X, p_marker[1] -> Y
    - 변경: p_marker[1] -> X, p_marker[0] -> Y

    화면에서 X/Y가 바뀐 것처럼 보일 때 사용하는 버전.
    """

    if USE_FIXED_ARUCO_ORIGIN == False:
        return None

    p_cam = pixel_depth_to_camera_point(u, v, z_m)

    if p_cam is None:
        return None

    p_marker = camera_point_to_fixed_aruco_point(p_cam)

    if p_marker is None:
        return None

    # =================================================
    # X/Y 축 교체 버전
    # =================================================
    robot_x_mm = ROBOT_X_SIGN * float(p_marker[1]) * 1000.0 + ROBOT_OFFSET_X_MM
    robot_y_mm = ROBOT_Y_SIGN * float(p_marker[0]) * 1000.0 + ROBOT_OFFSET_Y_MM

    marker_z_mm = float(p_marker[2]) * 1000.0

    return robot_x_mm, robot_y_mm, marker_z_mm

    # p_marker[0] : 마커 오른쪽 방향 +
    # p_marker[1] : 마커 위쪽 방향 +
    # 화면/컨베이어 기준 아래쪽을 +Y로 쓰기 위해 ROBOT_Y_SIGN 기본값을 -1.0으로 둔다.
    robot_x_mm = ROBOT_X_SIGN * float(p_marker[0]) * 1000.0 + ROBOT_OFFSET_X_MM
    robot_y_mm = ROBOT_Y_SIGN * float(p_marker[1]) * 1000.0 + ROBOT_OFFSET_Y_MM
    marker_z_mm = float(p_marker[2]) * 1000.0

    return robot_x_mm, robot_y_mm, marker_z_mm



def projected_metric_length_px(length_m, image_axis, fx, fy, depth_m):
    """
    실제 길이[m]를 현재 Depth와 카메라 내부 파라미터 기준 픽셀 길이로 변환한다.
    회전된 축 방향에서는 fx/fy를 함께 반영한다.
    """
    if length_m <= 0 or fx is None or fy is None or depth_m <= 0:
        return 0.0

    axis = np.asarray(image_axis, dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-12:
        return 0.0

    axis = axis / norm

    metric_per_pixel_direction = np.sqrt(
        (float(axis[0]) / float(fx)) ** 2
        + (float(axis[1]) / float(fy)) ** 2
    )

    if metric_per_pixel_direction <= 1e-12:
        return 0.0

    pixels_per_meter = 1.0 / (float(depth_m) * metric_per_pixel_direction)
    return float(length_m) * pixels_per_meter


def contour_median_depth_m(depth_u16, contour_full, depth_scale, min_valid=30):
    """검출 contour 내부의 유효 Depth 중앙값을 반환한다."""
    if contour_full is None or len(contour_full) < 3:
        return 0.0

    h, w = depth_u16.shape[:2]
    x, y, bw, bh = cv2.boundingRect(contour_full)

    x1_local = max(0, x)
    y1_local = max(0, y)
    x2_local = min(w, x + bw)
    y2_local = min(h, y + bh)

    if x1_local >= x2_local or y1_local >= y2_local:
        return 0.0

    local_contour = contour_full.copy().astype(np.int32)
    local_contour[:, 0, 0] -= x1_local
    local_contour[:, 0, 1] -= y1_local

    local_mask = np.zeros(
        (y2_local - y1_local, x2_local - x1_local),
        dtype=np.uint8,
    )
    cv2.drawContours(local_mask, [local_contour], -1, 255, -1)

    depth_roi = depth_u16[y1_local:y2_local, x1_local:x2_local]
    valid = depth_roi[(local_mask > 0) & (depth_roi > 0)]

    if valid.size < min_valid:
        return 0.0

    return float(np.median(valid.astype(np.float32)) * float(depth_scale))


def choose_completed_axis_center(
    min_projection,
    max_projection,
    expected_size,
    axis_image_y,
    extent_ratio,
):
    """
    관측된 윤곽을 포함하면서 예상 규격 길이를 만족하는 축 중심을 선택한다.

    관측 길이가 충분하면 기존 중심을 그대로 사용한다.
    부족하면 현재 환경의 어두운 영역이 영상 위쪽이라는 조건으로
    누락된 부분을 영상 위쪽 방향으로 복원한다.
    """
    midpoint = (float(min_projection) + float(max_projection)) * 0.5

    if extent_ratio >= KNOWN_SIZE_FULL_EXTENT_RATIO:
        return midpoint

    half_size = float(expected_size) * 0.5

    # 예상 크기의 사각형이 현재 관측 범위를 모두 포함할 수 있는 중심 구간
    feasible_low = float(max_projection) - half_size
    feasible_high = float(min_projection) + half_size

    if feasible_low > feasible_high:
        return midpoint

    if not COMPLETE_TOWARD_IMAGE_TOP:
        return midpoint

    # 영상 좌표에서 y가 작아지는 방향이 위쪽이다.
    if float(axis_image_y) >= 0.0:
        return feasible_low

    return feasible_high


def evaluate_known_size_orientation(
    points,
    contour_area,
    depth_m,
    long_axis,
    orientation_index,
):
    """
    하나의 회전축 후보에 대해 100 x 70 mm 전체 사각형을 복원하고 품질을 평가한다.
    """
    if CAM_FX is None or CAM_FY is None:
        return None

    ux = np.asarray(long_axis, dtype=np.float32)
    ux_norm = float(np.linalg.norm(ux))

    if ux_norm <= 1e-6:
        return None

    ux = ux / ux_norm

    # 방향 부호가 프레임마다 180도 반전되지 않도록 고정
    if ux[0] < 0 or (abs(float(ux[0])) < 1e-6 and ux[1] < 0):
        ux = -ux

    uy = np.array([-ux[1], ux[0]], dtype=np.float32)

    long_px = projected_metric_length_px(
        PRODUCT_LONG_SIDE_M,
        ux,
        CAM_FX,
        CAM_FY,
        depth_m,
    ) * KNOWN_SIZE_PIXEL_SCALE

    short_px = projected_metric_length_px(
        PRODUCT_SHORT_SIDE_M,
        uy,
        CAM_FX,
        CAM_FY,
        depth_m,
    ) * KNOWN_SIZE_PIXEL_SCALE

    if long_px <= 1.0 or short_px <= 1.0:
        return None

    proj_long = points @ ux
    proj_short = points @ uy

    min_long = float(np.min(proj_long))
    max_long = float(np.max(proj_long))
    min_short = float(np.min(proj_short))
    max_short = float(np.max(proj_short))

    observed_long = max_long - min_long
    observed_short = max_short - min_short

    if observed_long <= 1.0 or observed_short <= 1.0:
        return None

    extent_ratio_long = observed_long / long_px
    extent_ratio_short = observed_short / short_px

    if extent_ratio_long > KNOWN_SIZE_MAX_EXTENT_RATIO:
        return None

    if extent_ratio_short > KNOWN_SIZE_MAX_EXTENT_RATIO:
        return None

    expected_area = long_px * short_px
    visible_area_ratio = float(contour_area) / max(expected_area, 1e-6)

    if visible_area_ratio < KNOWN_SIZE_MIN_VISIBLE_AREA_RATIO:
        return None

    if visible_area_ratio > KNOWN_SIZE_MAX_VISIBLE_AREA_RATIO:
        return None

    center_long = choose_completed_axis_center(
        min_long,
        max_long,
        long_px,
        float(ux[1]),
        extent_ratio_long,
    )

    center_short = choose_completed_axis_center(
        min_short,
        max_short,
        short_px,
        float(uy[1]),
        extent_ratio_short,
    )

    center = center_long * ux + center_short * uy
    cx_fit = float(center[0])
    cy_fit = float(center[1])

    angle_fit = float(np.degrees(np.arctan2(float(ux[1]), float(ux[0]))))

    while angle_fit >= 90.0:
        angle_fit -= 180.0

    while angle_fit < -90.0:
        angle_fit += 180.0

    rect_fit = (
        (cx_fit, cy_fit),
        (float(long_px), float(short_px)),
        angle_fit,
    )
    box_fit = cv2.boxPoints(rect_fit)

    completion_used = (
        extent_ratio_long < KNOWN_SIZE_FULL_EXTENT_RATIO
        or extent_ratio_short < KNOWN_SIZE_FULL_EXTENT_RATIO
    )

    candidate_yaw = normalize_yaw(angle_fit, long_px, short_px)

    angle_penalty = 0.0
    center_penalty = 0.0

    if last_valid_yaw is not None:
        angle_penalty = abs(
            angle_diff_180(candidate_yaw, last_valid_yaw)
        ) * 1.5

    if last_valid_cx is not None and last_valid_cy is not None:
        center_penalty = (
            (
                (cx_fit - float(last_valid_cx)) ** 2
                + (cy_fit - float(last_valid_cy)) ** 2
            ) ** 0.5
        ) * 0.03

    oversize_penalty = (
        max(0.0, extent_ratio_long - 1.0) ** 2
        + max(0.0, extent_ratio_short - 1.0) ** 2
    ) * 500.0

    best_supported_extent = min(
        1.0,
        max(extent_ratio_long, extent_ratio_short),
    )
    edge_support_penalty = (
        (1.0 - best_supported_extent) ** 2 * 220.0
    )

    orientation_penalty = float(orientation_index) * 2.0
    visibility_reward = min(1.0, visible_area_ratio) * 18.0

    quality_score = (
        oversize_penalty
        + edge_support_penalty
        + angle_penalty
        + center_penalty
        + orientation_penalty
        - visibility_reward
    )

    return {
        "rect": rect_fit,
        "box": np.int32(np.round(box_fit)),
        "depth_m": float(depth_m),
        "visible_area_ratio": float(visible_area_ratio),
        "completion_used": bool(completion_used),
        "quality_score": float(quality_score),
        "extent_ratio_long": float(extent_ratio_long),
        "extent_ratio_short": float(extent_ratio_short),
    }



def fit_three_corner_known_size_rect(full_cnt, depth_m):
    """
    윤곽 근사 결과가 세 꼭짓점인 경우, 직각 꼭짓점과 두 변을 찾아
    실제 100 x 70 mm 길이로 네 번째 꼭짓점을 직접 복원한다.
    """
    if full_cnt is None or len(full_cnt) < 3:
        return None

    perimeter = cv2.arcLength(full_cnt, True)

    if perimeter <= 0:
        return None

    # 작은 epsilon부터 순차 적용하여 세 꼭짓점 근사를 찾는다.
    triangle = None

    for epsilon_ratio in (0.025, 0.035, 0.050, 0.070):
        approx = cv2.approxPolyDP(
            cv2.convexHull(full_cnt),
            epsilon_ratio * perimeter,
            True,
        )

        if len(approx) == 3:
            triangle = approx.reshape(3, 2).astype(np.float32)
            break

    if triangle is None:
        return None

    contour_area = float(cv2.contourArea(full_cnt))
    best_fit = None

    # 세 점 중 어느 점이 직각 꼭짓점인지 모두 검사한다.
    for corner_index in range(3):
        corner = triangle[corner_index]
        other_indices = [
            index for index in range(3)
            if index != corner_index
        ]

        point_a = triangle[other_indices[0]]
        point_b = triangle[other_indices[1]]

        vector_a = point_a - corner
        vector_b = point_b - corner

        length_a = float(np.linalg.norm(vector_a))
        length_b = float(np.linalg.norm(vector_b))

        if length_a <= 1.0 or length_b <= 1.0:
            continue

        axis_a = vector_a / length_a
        axis_b = vector_b / length_b

        # 직각에 가까울수록 내적 절댓값이 0에 가깝다.
        perpendicular_error = abs(float(np.dot(axis_a, axis_b)))

        if perpendicular_error > 0.40:
            continue

        # a=긴 변/b=짧은 변, a=짧은 변/b=긴 변 두 경우를 모두 검사한다.
        for assignment_index in range(2):
            if assignment_index == 0:
                long_axis = axis_a
                short_axis = axis_b
                observed_long = length_a
                observed_short = length_b
            else:
                long_axis = axis_b
                short_axis = axis_a
                observed_long = length_b
                observed_short = length_a

            expected_long = projected_metric_length_px(
                PRODUCT_LONG_SIDE_M,
                long_axis,
                CAM_FX,
                CAM_FY,
                depth_m,
            ) * KNOWN_SIZE_PIXEL_SCALE

            expected_short = projected_metric_length_px(
                PRODUCT_SHORT_SIDE_M,
                short_axis,
                CAM_FX,
                CAM_FY,
                depth_m,
            ) * KNOWN_SIZE_PIXEL_SCALE

            if expected_long <= 1.0 or expected_short <= 1.0:
                continue

            long_ratio = observed_long / expected_long
            short_ratio = observed_short / expected_short

            # 세 꼭짓점이 실제 제품 모서리라면 두 관측 변이 예상 길이에 근접해야 한다.
            if long_ratio < 0.60 or long_ratio > 1.35:
                continue

            if short_ratio < 0.60 or short_ratio > 1.35:
                continue

            predicted_long_corner = (
                corner + long_axis * expected_long
            )
            predicted_short_corner = (
                corner + short_axis * expected_short
            )
            predicted_missing_corner = (
                predicted_long_corner
                + predicted_short_corner
                - corner
            )

            box_points = np.array(
                [
                    corner,
                    predicted_long_corner,
                    predicted_missing_corner,
                    predicted_short_corner,
                ],
                dtype=np.float32,
            )

            center = np.mean(box_points, axis=0)
            cx_fit = float(center[0])
            cy_fit = float(center[1])

            angle_fit = float(
                np.degrees(
                    np.arctan2(
                        float(long_axis[1]),
                        float(long_axis[0]),
                    )
                )
            )

            while angle_fit >= 90.0:
                angle_fit -= 180.0

            while angle_fit < -90.0:
                angle_fit += 180.0

            expected_area = expected_long * expected_short
            visible_area_ratio = (
                contour_area / max(expected_area, 1e-6)
            )

            if (
                visible_area_ratio
                < KNOWN_SIZE_MIN_VISIBLE_AREA_RATIO
                or visible_area_ratio
                > KNOWN_SIZE_MAX_VISIBLE_AREA_RATIO
            ):
                continue

            size_error = (
                abs(1.0 - long_ratio)
                + abs(1.0 - short_ratio)
            )

            angle_penalty = 0.0
            center_penalty = 0.0
            candidate_yaw = normalize_yaw(
                angle_fit,
                expected_long,
                expected_short,
            )

            if last_valid_yaw is not None:
                angle_penalty = abs(
                    angle_diff_180(
                        candidate_yaw,
                        last_valid_yaw,
                    )
                ) * 1.2

            if (
                last_valid_cx is not None
                and last_valid_cy is not None
            ):
                center_penalty = (
                    (
                        (cx_fit - float(last_valid_cx)) ** 2
                        + (cy_fit - float(last_valid_cy)) ** 2
                    ) ** 0.5
                ) * 0.02

            quality_score = (
                perpendicular_error * 250.0
                + size_error * 120.0
                + angle_penalty
                + center_penalty
                + float(assignment_index) * 1.0
                - min(1.0, visible_area_ratio) * 15.0
            )

            fit = {
                "rect": (
                    (cx_fit, cy_fit),
                    (
                        float(expected_long),
                        float(expected_short),
                    ),
                    angle_fit,
                ),
                "box": np.int32(np.round(box_points)),
                "depth_m": float(depth_m),
                "visible_area_ratio": float(
                    visible_area_ratio
                ),
                "completion_used": True,
                "quality_score": float(quality_score),
                "extent_ratio_long": float(long_ratio),
                "extent_ratio_short": float(short_ratio),
                "three_corner_used": True,
            }

            if (
                best_fit is None
                or fit["quality_score"]
                < best_fit["quality_score"]
            ):
                best_fit = fit

    return best_fit


def fit_known_size_rect(full_cnt, depth_image, depth_scale):
    """
    부분 윤곽 또는 세 꼭짓점만 검출된 제품을 실제 규격 100 x 70 mm로 복원한다.

    정상 검출 결과를 강제로 바꾸지 않으며, 물리 크기·Depth·회전 방향이
    허용 범위를 만족하는 경우에만 복원 결과를 반환한다.
    """
    if not USE_KNOWN_SIZE_COMPLETION:
        return None

    if CAM_FX is None or CAM_FY is None:
        return None

    if full_cnt is None or len(full_cnt) < 3:
        return None

    contour_area = float(cv2.contourArea(full_cnt))

    if contour_area <= 0:
        return None

    partial_rect = cv2.minAreaRect(full_cnt)
    (partial_cx, partial_cy), (partial_rw, partial_rh), _ = partial_rect

    if partial_rw <= 0 or partial_rh <= 0:
        return None

    depth_m = contour_median_depth_m(
        depth_image,
        full_cnt,
        depth_scale,
        min_valid=30,
    )

    if depth_m <= 0:
        depth_m = patch_median_depth_m(
            depth_image,
            depth_scale,
            int(round(partial_cx)),
            int(round(partial_cy)),
            r=20,
            min_valid=8,
        )

    if depth_m <= 0:
        return None

    # 세 꼭짓점이 명확하게 보이면 네 번째 꼭짓점을 직접 복원하는 경로를 우선한다.
    three_corner_fit = fit_three_corner_known_size_rect(
        full_cnt,
        depth_m,
    )

    if three_corner_fit is not None:
        return three_corner_fit

    partial_box = cv2.boxPoints(partial_rect).astype(np.float32)
    edges = np.roll(partial_box, -1, axis=0) - partial_box
    lengths = np.linalg.norm(edges, axis=1)

    longest_index = int(np.argmax(lengths))
    base_axis = edges[longest_index]
    base_norm = float(np.linalg.norm(base_axis))

    if base_norm <= 1e-6:
        return None

    base_axis = base_axis / base_norm

    if base_axis[0] < 0 or (
        abs(float(base_axis[0])) < 1e-6
        and base_axis[1] < 0
    ):
        base_axis = -base_axis

    perpendicular_axis = np.array(
        [-base_axis[1], base_axis[0]],
        dtype=np.float32,
    )

    orientation_candidates = [base_axis, perpendicular_axis]
    points = full_cnt.reshape(-1, 2).astype(np.float32)

    best_fit = None

    for orientation_index, long_axis in enumerate(
        orientation_candidates
    ):
        fit = evaluate_known_size_orientation(
            points,
            contour_area,
            depth_m,
            long_axis,
            orientation_index,
        )

        if fit is None:
            continue

        if (
            best_fit is None
            or fit["quality_score"] < best_fit["quality_score"]
        ):
            best_fit = fit

    return best_fit


def process_pending_master_events():
    """TCP 수신 스레드에서 받은 MCU 상태를 영상 처리 메인 스레드에 반영한다."""

    global retired_target_center
    global retired_target_depth_m
    global retired_target_box
    global retired_target_started_at
    global retired_target_absent_frames
    global last_object_seen_time

    if pickup_started_event.is_set():
        pickup_started_event.clear()

        # 두 번째 코드와 동일하게 p/A/G 처리가 완료된 제품의 STATE 3만
        # 정상 흡착 시작으로 인정한다.
        if not stop_request_sent or not capture_result_sent:
            print(
                "STATE 3 수신 - 처리 중인 제품 좌표 전송이 완료되지 않아 "
                "추적 상태 유지"
            )
        else:
            retired_center = None
            if last_valid_cx is not None and last_valid_cy is not None:
                retired_center = (float(last_valid_cx), float(last_valid_cy))

            retired_depth = last_valid_depth_m
            retired_box = (
                last_valid_box.copy()
                if last_valid_box is not None
                else None
            )

            print(
                "STATE 3 흡착 시작 수신 - 현재 제품 추적 종료, "
                "다음 제품 검출 및 ROI 정지 허용"
            )

            # 현재 제품 one-shot, p 전송, 안정화 이력을 모두 초기화한다.
            reset_one_shot_lock()
            yaw_hist.clear()
            cx_hist.clear()
            cy_hist.clear()

            # 초기화 전에 저장한 직전 제품을 임시 제외 대상으로 다시 등록한다.
            if retired_center is not None:
                retired_target_center = retired_center
                retired_target_depth_m = (
                    float(retired_depth) if retired_depth is not None else None
                )
                retired_target_box = retired_box
                retired_target_started_at = time.monotonic()
                retired_target_absent_frames = 0
                print(
                    "직전 흡착 제품 임시 제외 시작 - "
                    "center=({:.1f}, {:.1f}), depth={}".format(
                        retired_center[0],
                        retired_center[1],
                        retired_target_depth_m,
                    )
                )

            last_object_seen_time = time.time()

    if pickup_completed_event.is_set():
        pickup_completed_event.clear()
        # STATE 4 시점에는 다음 제품을 이미 추적하거나 정지했을 수 있으므로
        # 현재 비전/전송 상태를 초기화하지 않는다.
        print("STATE 4 흡착 완료 수신 - 현재 다음 제품 추적/정지 상태 유지")

def print_depth_diff_debug(diff, roi_depth):
    """
    depth 차분값 디버그 출력.
    DEBUG_DIFF_PRINT=True일 때만 동작.
    """

    global last_debug_print_time

    if DEBUG_DIFF_PRINT == False:
        return

    now = time.time()

    if now - last_debug_print_time < DEBUG_PRINT_INTERVAL:
        return

    valid_diff = diff[roi_depth > 0]

    if valid_diff.size > 0:
        print(
            "diff min:",
            round(float(np.min(valid_diff)), 4),
            "max:",
            round(float(np.max(valid_diff)), 4),
            "mean:",
            round(float(np.mean(valid_diff)), 4)
        )

    last_debug_print_time = now


def convex_box_iou(box_a, box_b):
    """두 회전 사각형의 교집합/합집합 비율을 계산한다."""
    if box_a is None or box_b is None:
        return 0.0

    a = np.asarray(box_a, dtype=np.float32).reshape(-1, 2)
    b = np.asarray(box_b, dtype=np.float32).reshape(-1, 2)

    if len(a) < 3 or len(b) < 3:
        return 0.0
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        return 0.0

    area_a = abs(float(cv2.contourArea(a)))
    area_b = abs(float(cv2.contourArea(b)))
    if area_a <= 1e-6 or area_b <= 1e-6:
        return 0.0

    try:
        intersection_area, _ = cv2.intersectConvexConvex(a, b)
    except cv2.error:
        return 0.0

    intersection = max(0.0, float(intersection_area))
    union = area_a + area_b - intersection
    if union <= 1e-6:
        return 0.0

    return max(0.0, min(1.0, intersection / union))


def clear_retired_target_guard(print_log=False):
    global retired_target_center
    global retired_target_depth_m
    global retired_target_box
    global retired_target_started_at
    global retired_target_absent_frames

    if print_log and retired_target_center is not None:
        print("직전 흡착 제품 화면 이탈 확인 - 임시 제외 해제")

    retired_target_center = None
    retired_target_depth_m = None
    retired_target_box = None
    retired_target_started_at = 0.0
    retired_target_absent_frames = 0


def filter_retired_target_candidates(valid_contours, depth_image, depth_scale):
    """STATE 3 직후 직전 흡착 제품만 후보 목록에서 제외한다.

    마스크 생성, contour 생성/보정, 중심·각도·좌표 계산은 변경하지 않고
    이미 생성된 후보 중 직전 제품과 일치하는 후보만 제어 로직상 제외한다.
    """
    global retired_target_center
    global retired_target_box
    global retired_target_absent_frames

    if retired_target_center is None:
        return valid_contours

    # 전체 화면 처리 시 후보 수가 많아지는 것만 제한한다.
    candidates = sorted(
        valid_contours,
        key=lambda item: item["area"],
        reverse=True,
    )[:max(1, int(MAX_CANDIDATE_COUNT))]

    remaining = []
    retired_seen = False
    followed_center = None
    followed_box = None
    best_retired_score = None

    retired_cx, retired_cy = retired_target_center

    for item in candidates:
        cnt = item["cnt"]
        rect = item["rect"]
        (cx, cy), _, _ = rect
        candidate_box = np.asarray(cv2.boxPoints(rect), dtype=np.float32)

        candidate_depth_m = patch_median_depth_m(
            depth_image,
            depth_scale,
            cx,
            cy,
            r=12,
            min_valid=8,
        )

        center_distance = (
            (float(cx) - float(retired_cx)) ** 2
            + (float(cy) - float(retired_cy)) ** 2
        ) ** 0.5
        box_iou = convex_box_iou(candidate_box, retired_target_box)

        same_position = (
            center_distance <= RETIRED_TARGET_SAME_CENTER_RADIUS_PIXEL
        )
        same_box = box_iou >= RETIRED_TARGET_BOX_IOU_THRESHOLD

        lifted_old_product = False
        if retired_target_depth_m is not None and candidate_depth_m > 0.0:
            lifted_old_product = (
                center_distance <= RETIRED_TARGET_FOLLOW_MAX_DISTANCE_PIXEL
                and candidate_depth_m
                <= float(retired_target_depth_m) - RETIRED_TARGET_LIFT_DEPTH_DELTA_M
            )

        if same_position or same_box or lifted_old_product:
            retired_seen = True
            retired_score = center_distance - box_iou * 100.0
            if best_retired_score is None or retired_score < best_retired_score:
                best_retired_score = retired_score
                followed_center = (float(cx), float(cy))
                followed_box = candidate_box.copy()
            continue

        remaining.append(item)

    if retired_seen and followed_center is not None:
        retired_target_center = followed_center
        retired_target_box = followed_box
        retired_target_absent_frames = 0
    else:
        retired_target_absent_frames += 1

    guard_elapsed = time.monotonic() - retired_target_started_at
    if (
        retired_target_absent_frames >= RETIRED_TARGET_CLEAR_REQUIRED_FRAMES
        or guard_elapsed >= RETIRED_TARGET_GUARD_TIMEOUT_SEC
    ):
        clear_retired_target_guard(print_log=True)

    return remaining


def select_best_contour(
    valid_contours,
    x1,
    y1,
    x2,
    y2,
    contour_offset_x=0,
    contour_offset_y=0
):
    """
    무조건 가장 큰 contour만 고르지 않고,
    이전 정상 중심 또는 ROI 중심에 가까운 contour를 우선 선택한다.

    전체 화면 처리에서는 contour 좌표가 이미 전체 이미지 좌표이므로
    contour_offset_x/y를 0으로 사용한다.
    """

    if len(valid_contours) == 0:
        return None

    if last_valid_cx is not None and last_valid_cy is not None:
        ref_cx = last_valid_cx
        ref_cy = last_valid_cy
    else:
        ref_cx = (x1 + x2) / 2
        ref_cy = (y1 + y2) / 2

    best_item = None
    best_score = None

    for item in valid_contours:
        area = item["area"]
        local_cx = item["local_cx"]
        local_cy = item["local_cy"]

        full_cx = local_cx + contour_offset_x
        full_cy = local_cy + contour_offset_y

        dx = full_cx - ref_cx
        dy = full_cy - ref_cy
        dist = (dx * dx + dy * dy) ** 0.5

        # 면적은 클수록 좋고, 기준점에서 멀수록 감점
        score = area - (dist * 80)

        if best_score is None:
            best_score = score
            best_item = item
        else:
            if score > best_score:
                best_score = score
                best_item = item

    return best_item


def refine_contour_for_rect(cnt, mask_shape, erode_size=3, erode_iter=1):
    """
    바운딩 박스를 더 타이트하게 잡기 위한 contour 보정 함수.
    검출용 contour는 그대로 쓰고,
    minAreaRect 계산용 contour만 살짝 안쪽으로 줄여서 사용한다.
    """

    temp_mask = np.zeros(mask_shape, dtype=np.uint8)

    # 선택된 contour만 흰색으로 채움
    cv2.drawContours(temp_mask, [cnt], -1, 255, -1)

    # 외곽을 살짝 안쪽으로 줄임
    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (erode_size, erode_size)
    )

    temp_mask = cv2.erode(
        temp_mask,
        kernel,
        iterations=erode_iter
    )

    contours, _ = cv2.findContours(
        temp_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        return cnt

    best_refined = max(contours, key=cv2.contourArea)

    if cv2.contourArea(best_refined) < 1000:
        return cnt

    return best_refined


def refine_top_surface_mask(roi_depth, cnt, depth_scale, local_cx, local_cy, depth_margin_m=0.008):
    """
    선택된 contour 안에서 박스의 윗면에 가까운 깊이만 남긴다.

    핵심:
    - RealSense depth는 값이 작을수록 카메라와 가까움
    - 박스 윗면은 옆면/아래 모서리보다 카메라에 가까운 편
    - contour 내부 depth 중 가까운 쪽 25% 지점을 기준 깊이로 사용
    - 기준 깊이보다 너무 먼 부분을 제거해서 옆면/튀어나온 부분을 줄임
    """

    temp_mask = np.zeros(roi_depth.shape, dtype=np.uint8)

    # 선택된 contour 내부만 흰색으로 채움
    cv2.drawContours(temp_mask, [cnt], -1, 255, -1)

    # contour 내부의 유효 depth만 추출
    valid_depth = roi_depth[
        (temp_mask > 0) &
        (roi_depth > 0)
    ]

    if valid_depth.size < 100:
        return cnt

    # 가까운 쪽 25% 깊이를 윗면 기준으로 사용
    top_depth_raw = np.percentile(valid_depth, 25)

    roi_depth_float = roi_depth.astype(np.float32)

    # 기준 윗면 깊이보다 얼마나 더 멀리 있는지 계산
    # 양수: 기준보다 멀다
    # 음수: 기준보다 더 가깝다
    diff_from_top_m = (roi_depth_float - float(top_depth_raw)) * depth_scale

    top_mask = np.zeros(roi_depth.shape, dtype=np.uint8)

    # -0.004는 너무 가까운 튐값을 약간 허용하기 위한 범위
    # depth_margin_m는 기준보다 먼 부분을 어느 정도까지 윗면으로 볼지 결정
    top_mask[
        (temp_mask > 0) &
        (roi_depth > 0) &
        (diff_from_top_m > -0.004) &
        (diff_from_top_m < depth_margin_m)
    ] = 255

    # 작은 구멍 메우기
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    top_mask = cv2.morphologyEx(
        top_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=1
    )

    # 얇게 튀어나온 부분 제거
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    top_mask = cv2.morphologyEx(
        top_mask,
        cv2.MORPH_OPEN,
        open_kernel,
        iterations=1
    )

    contours, _ = cv2.findContours(
        top_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        return cnt

    best_top = max(contours, key=cv2.contourArea)

    if cv2.contourArea(best_top) < 1000:
        return cnt

    return best_top


def remove_contour_corner_noise(cnt, mask_shape, open_size=11):
    """
    오른쪽 아래처럼 contour에 붙어 있는 튀어나온 덩어리를 줄이는 함수.
    바운딩 박스 계산 전에 한 번 더 contour를 정리한다.
    """

    temp_mask = np.zeros(mask_shape, dtype=np.uint8)

    cv2.drawContours(temp_mask, [cnt], -1, 255, -1)

    # 얇게 붙은 부분을 먼저 끊음
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    temp_mask = cv2.erode(
        temp_mask,
        erode_kernel,
        iterations=1
    )

    # 튀어나온 모서리 덩어리 완화
    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (open_size, open_size)
    )

    temp_mask = cv2.morphologyEx(
        temp_mask,
        cv2.MORPH_OPEN,
        open_kernel,
        iterations=1
    )

    # 다시 살짝 복원
    temp_mask = cv2.dilate(
        temp_mask,
        erode_kernel,
        iterations=1
    )

    contours, _ = cv2.findContours(
        temp_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        return cnt

    best_cnt = max(contours, key=cv2.contourArea)

    if cv2.contourArea(best_cnt) < 1000:
        return cnt

    return best_cnt

def trim_contour_by_percentile(cnt, trim_ratio=0.08):
    """
    contour의 튀어나온 극단점들을 제거하는 함수.

    원리:
    1. 먼저 contour 전체로 minAreaRect를 구한다.
    2. contour 점들을 회전 사각형의 로컬 좌표계로 변환한다.
    3. 로컬 x, y 방향에서 양 끝 8% 정도를 잘라낸다.
    4. 남은 점들만 다시 minAreaRect에 사용한다.

    오른쪽 아래처럼 특정 모서리만 튀는 경우에 효과가 좋다.
    """

    if cnt is None:
        return cnt

    if len(cnt) < 10:
        return cnt

    rect = cv2.minAreaRect(cnt)
    (cx, cy), (rw, rh), angle = rect

    if rw <= 0 or rh <= 0:
        return cnt

    points = cnt.reshape(-1, 2).astype(np.float32)

    theta = np.deg2rad(angle)

    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    local_points = []

    for p in points:
        x = p[0] - cx
        y = p[1] - cy

        # 회전 사각형 기준 로컬 좌표로 변환
        local_x = x * cos_t + y * sin_t
        local_y = -x * sin_t + y * cos_t

        local_points.append([local_x, local_y, p[0], p[1]])

    local_points = np.array(local_points, dtype=np.float32)

    lx = local_points[:, 0]
    ly = local_points[:, 1]

    x_low = np.percentile(lx, trim_ratio * 100.0)
    x_high = np.percentile(lx, 100.0 - trim_ratio * 100.0)

    y_low = np.percentile(ly, trim_ratio * 100.0)
    y_high = np.percentile(ly, 100.0 - trim_ratio * 100.0)

    kept_points = []

    for row in local_points:
        local_x = row[0]
        local_y = row[1]
        original_x = row[2]
        original_y = row[3]

        if local_x < x_low:
            continue

        if local_x > x_high:
            continue

        if local_y < y_low:
            continue

        if local_y > y_high:
            continue

        kept_points.append([original_x, original_y])

    if len(kept_points) < 10:
        return cnt

    kept_points = np.array(kept_points, dtype=np.float32)
    kept_points = kept_points.reshape(-1, 1, 2).astype(np.int32)

    return kept_points

# master.py TCP 연결 스레드 시작
master_client.start()

# =========================================================
# RealSense 초기화
# =========================================================
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

# =========================================================
# RealSense color 카메라 내부 파라미터 저장
# aligned depth를 color 좌표계에 맞춰 사용하므로 color intrinsics를 사용한다.
# =========================================================
color_stream_profile = profile.get_stream(rs.stream.color)
color_video_profile = color_stream_profile.as_video_stream_profile()
color_intrinsics = color_video_profile.get_intrinsics()

CAM_FX = float(color_intrinsics.fx)
CAM_FY = float(color_intrinsics.fy)
CAM_PPX = float(color_intrinsics.ppx)
CAM_PPY = float(color_intrinsics.ppy)

print("====================================")
print("고정 ArUco 원점 좌표계 사용")
print("저장 당시 ArUco 마커 중심 = Robot (0, 0)")
print("FIXED_ARUCO_RVEC:", FIXED_ARUCO_RVEC)
print("FIXED_ARUCO_TVEC:", FIXED_ARUCO_TVEC)
print("Camera fx:", CAM_FX)
print("Camera fy:", CAM_FY)
print("Camera ppx:", CAM_PPX)
print("Camera ppy:", CAM_PPY)
print("====================================")

if USE_DEPTH_FILTERS:
    spatial_filter = rs.spatial_filter()
    temporal_filter = rs.temporal_filter()
    hole_filling_filter = rs.hole_filling_filter()

background_depth = None

program_start_time = time.time()
auto_background_done = False


# =========================================================
# 메인 루프
# =========================================================
try:
    while True:
        process_pending_master_events()

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        if USE_DEPTH_FILTERS:
            depth_frame = spatial_filter.process(depth_frame)
            depth_frame = temporal_filter.process(depth_frame)
            depth_frame = hole_filling_filter.process(depth_frame)
            depth_frame = depth_frame.as_depth_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        display = color_image.copy()

        h, w = color_image.shape[:2]

        # =====================================================
        # ROI 설정
        # =====================================================
        if USE_ROI:
            x1 = ROI_X1
            y1 = ROI_Y1
            x2 = ROI_X2
            y2 = ROI_Y2
        else:
            x1 = 0
            y1 = 0
            x2 = w
            y2 = h

        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))

        # 영상 처리는 ROI crop이 아니라 전체 depth 프레임에서 수행한다.
        # x1~y2는 검출 범위가 아니라 제품 전체 진입 판정 범위로만 사용한다.
        processing_depth = depth_image

        mask = np.zeros(processing_depth.shape, dtype=np.uint8)

        # =====================================================
        # 5초 뒤 자동 배경 저장
        # =====================================================
        if AUTO_BACKGROUND_ENABLE and auto_background_done == False:
            elapsed_time = time.time() - program_start_time

            if elapsed_time >= AUTO_BACKGROUND_DELAY_SEC:
                background_depth = processing_depth.copy()

                yaw_hist.clear()
                cx_hist.clear()
                cy_hist.clear()

                reset_one_shot_lock()

                last_object_seen_time = time.time()
                auto_background_done = True

                print("자동 배경 저장 완료")
                print("one-shot 잠금 초기화 완료")

        # =====================================================
        # 배경 저장 후에만 검출 수행
        # =====================================================
        if background_depth is not None:
            diff = (background_depth.astype(np.float32) - processing_depth.astype(np.float32)) * depth_scale

            print_depth_diff_debug(diff, processing_depth)

            bg_valid = background_depth > BACKGROUND_VALID_MIN
            cur_valid = processing_depth > 0

            # =================================================
            # 1. 정상 depth 차분 마스크
            # =================================================
            depth_mask = np.zeros(processing_depth.shape, dtype=np.uint8)

            depth_mask[
                (diff > MIN_DEPTH_DIFF) &
                (diff < MAX_DEPTH_DIFF) &
                bg_valid &
                cur_valid
            ] = 255

            # =================================================
            # 2. Depth hole 마스크
            # =================================================
            hole_mask = np.zeros(processing_depth.shape, dtype=np.uint8)

            if USE_DEPTH_HOLE_MASK:
                hole_mask[bg_valid & (~cur_valid)] = 255

            mask = cv2.bitwise_or(depth_mask, hole_mask)

            # =================================================
            # 3. 검정 박스 색상 보조 필터
            # =================================================
            if USE_DARK_COLOR_FILTER:
                gray_full = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

                dark_mask = np.zeros(gray_full.shape, dtype=np.uint8)
                dark_mask[gray_full < DARK_GRAY_THRESHOLD] = 255

                dark_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                dark_mask = cv2.morphologyEx(
                    dark_mask,
                    cv2.MORPH_CLOSE,
                    dark_kernel,
                    iterations=1
                )

                mask = cv2.bitwise_and(mask, dark_mask)

            # =================================================
            # 4. 마스크 정리
            # =================================================
            mask = cv2.medianBlur(mask, 5)
            mask = clean_mask(mask)

            # =================================================
            # 5. contour 검출
            # =================================================
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            valid_contours = []

            for cnt in contours:
                area = cv2.contourArea(cnt)

                if area > MAX_AREA:
                    continue

                rect_temp = cv2.minAreaRect(cnt)
                (tcx, tcy), (tw, th), tangle = rect_temp

                rect_area = tw * th

                if rect_area <= 0:
                    continue

                fill_ratio = area / rect_area
                short_side = min(tw, th)
                long_side = max(tw, th)
                aspect_ratio = long_side / max(short_side, 1e-6)

                epsilon_check = 0.03 * cv2.arcLength(cnt, True)
                approx_check = cv2.approxPolyDP(
                    cnt,
                    epsilon_check,
                    True,
                )
                vertex_count = len(approx_check)

                # 기존 완전 검출 기준
                full_candidate = (
                    area >= MIN_AREA
                    and fill_ratio >= MIN_FILL_RATIO
                    and short_side >= MIN_RECT_SHORT_SIDE
                    and long_side >= MIN_RECT_LONG_SIDE
                    and aspect_ratio <= MAX_ASPECT_RATIO
                    and vertex_count >= MIN_VERTEX_COUNT
                    and vertex_count <= MAX_VERTEX_COUNT
                )

                # 세 꼭짓점/부분 윤곽을 규격 복원 후보로 받는 완화 기준
                partial_candidate = (
                    USE_KNOWN_SIZE_COMPLETION
                    and area >= PARTIAL_MIN_AREA
                    and fill_ratio >= PARTIAL_MIN_FILL_RATIO
                    and short_side >= PARTIAL_MIN_RECT_SHORT_SIDE
                    and long_side >= PARTIAL_MIN_RECT_LONG_SIDE
                    and aspect_ratio <= PARTIAL_MAX_ASPECT_RATIO
                    and vertex_count >= PARTIAL_MIN_VERTEX_COUNT
                    and vertex_count <= PARTIAL_MAX_VERTEX_COUNT
                )

                if not full_candidate and not partial_candidate:
                    continue

                # 완전 기준에 미달한 후보는 실제 100 x 70 mm 규격과
                # 현재 Depth가 맞는 경우에만 후보 목록에 추가한다.
                preliminary_known_fit = None

                if not full_candidate:
                    preliminary_known_fit = fit_known_size_rect(
                        cnt,
                        processing_depth,
                        depth_scale,
                    )

                    if preliminary_known_fit is None:
                        continue

                valid_contours.append({
                    "area": area,
                    "cnt": cnt,
                    "rect": rect_temp,
                    "local_cx": tcx,
                    "local_cy": tcy,
                    "fill_ratio": fill_ratio,
                    "aspect_ratio": aspect_ratio,
                    "short_side": short_side,
                    "long_side": long_side,
                    "vertex_count": vertex_count,
                    "partial_candidate": not full_candidate,
                    "preliminary_known_fit": preliminary_known_fit,
                })

            # =================================================
            # 6. STATE 3 이후 직전 흡착 제품 후보 제외
            # 영상처리 결과 자체는 변경하지 않고 후보 선택 단계에서만 제외한다.
            # =================================================
            valid_contours = filter_retired_target_candidates(
                valid_contours,
                processing_depth,
                depth_scale,
            )

            # =================================================
            # 7. 유효 contour 선택
            # =================================================
            best_item = select_best_contour(
                valid_contours,
                x1,
                y1,
                x2,
                y2,
                contour_offset_x=0,
                contour_offset_y=0
            )

            if best_item is not None:
                best_cnt = best_item["cnt"]

                # =================================================
                # 바운딩 박스 정밀도 개선
                # 1) 윗면에 가까운 깊이만 추출
                # 2) 오른쪽 아래처럼 붙는 튀어나온 contour 제거
                # 3) 살짝 안쪽으로 contour 보정
                # 4) convexHull로 외곽 안정화
                # 5) minAreaRect로 회전 사각형 계산
                # =================================================
                top_cnt = refine_top_surface_mask(
                processing_depth,
                best_cnt,
                depth_scale,
                best_item["local_cx"],
                best_item["local_cy"],
                depth_margin_m=0.010
            )

                clean_top_cnt = remove_contour_corner_noise(
                    top_cnt,
                    mask.shape,
                    open_size=9
                )

                rect_cnt = refine_contour_for_rect(
                    clean_top_cnt,
                    mask.shape,
                    erode_size=3,
                    erode_iter=1
                )

                # 핵심 추가:
                # contour의 극단적으로 튀어나온 점 제거
                trimmed_cnt = trim_contour_by_percentile(
                    rect_cnt,
                    trim_ratio=0.08
                )

                # 중요:
                # convexHull 사용하지 않음
                # convexHull은 튀어나온 오른쪽 아래 영역까지 감싸서 오히려 박스가 커짐

                # 전체 화면에서 검출했으므로 contour 좌표는 이미 전체 이미지 좌표이다.
                full_cnt = trimmed_cnt.copy()

                # 정상 검출은 기존 minAreaRect 결과를 그대로 사용한다.
                raw_rect = cv2.minAreaRect(full_cnt)
                (raw_cx, raw_cy), (raw_rw, raw_rh), raw_angle = raw_rect

                # 부분 검출/세 꼭짓점 검출일 때만 100 x 70 mm 규격 복원을 시도한다.
                # 크기 복원에는 윤곽 범위를 과하게 줄이는 trim 이전 contour를 사용한다.
                known_fit = fit_known_size_rect(
                    clean_top_cnt,
                    processing_depth,
                    depth_scale,
                )

                if known_fit is None:
                    known_fit = best_item.get(
                        "preliminary_known_fit"
                    )

                # 상단에서 잘린 제품은 면적이 커서 full_candidate로 분류될 수 있다.
                # 이 경우 completion_used가 False여도 규격 복원 사각형을 사용해야 하므로,
                # contour가 상단에 닿고 복원 사각형이 상단을 벗어나면 known_fit을 강제 사용한다.
                top_clipped_known_fit_candidate = False

                if known_fit is not None and clean_top_cnt is not None and len(clean_top_cnt) >= 3:
                    known_box_points = np.asarray(
                        known_fit["box"], dtype=np.float32
                    ).reshape(-1, 2)
                    clean_points = np.asarray(
                        clean_top_cnt, dtype=np.float32
                    ).reshape(-1, 2)

                    known_box_min_y = float(np.min(known_box_points[:, 1]))
                    clean_contour_min_y = float(np.min(clean_points[:, 1]))

                    top_clipped_known_fit_candidate = bool(
                        known_box_min_y < float(y1 + ROI_INNER_MARGIN)
                        and clean_contour_min_y
                        <= float(
                            y1
                            + ROI_INNER_MARGIN
                            + TOP_EXIT_RECOVERY_TOP_TOUCH_TOLERANCE_PX
                        )
                    )

                known_size_used = bool(
                    known_fit is not None
                    and (
                        best_item.get("partial_candidate", False)
                        or known_fit["completion_used"]
                        or top_clipped_known_fit_candidate
                    )
                )

                if known_size_used:
                    rect = known_fit["rect"]
                    (cx, cy), (rw, rh), angle = rect
                    current_box = known_fit["box"].copy()
                else:
                    rect = raw_rect
                    (cx, cy), (rw, rh), angle = rect
                    current_box = cv2.boxPoints(rect)
                    current_box = np.int32(current_box)

                yaw_raw = normalize_yaw(angle, rw, rh)

                # =================================================
                # 제품 전체 ROI 진입 또는 ROI 상단 이탈 3꼭짓점 복구 판정
                # =================================================
                full_inside_roi = True
                top_exit_recovery_valid = False

                if REQUIRE_FULL_BOX_IN_ROI:
                    box_inside_roi = is_box_fully_inside_roi(
                        current_box,
                        x1,
                        y1,
                        x2,
                        y2,
                        ROI_INNER_MARGIN
                    )

                    contour_inside_roi = is_contour_fully_inside_roi_global(
                        full_cnt,
                        x1,
                        y1,
                        x2,
                        y2,
                        ROI_INNER_MARGIN
                    )

                    full_inside_roi = box_inside_roi and contour_inside_roi

                    if not full_inside_roi and known_size_used:
                        # 최초 복구는 세 꼭짓점 직접 복원으로 엄격하게 확인한다.
                        top_exit_recovery_valid = (
                            is_top_exit_three_corner_recovery_valid(
                                current_box,
                                clean_top_cnt,
                                known_fit,
                                x1,
                                y1,
                                x2,
                                y2,
                                ROI_INNER_MARGIN,
                                latched_mode=False,
                            )
                        )

                        if top_exit_recovery_valid:
                            if not top_exit_recovery_latched:
                                print(
                                    "ROI/영상 상단 절단 규격 복원 확인 - "
                                    "복원 외곽선으로 정지/좌표 추출 허용"
                                )
                            top_exit_recovery_latched = True

                        elif top_exit_recovery_latched:
                            # 같은 제품에 대해 복구가 이미 확인된 뒤에는 p 전송 후
                            # contour 근사가 일시적으로 변해도 규격 복원 결과를 유지한다.
                            top_exit_recovery_valid = (
                                is_top_exit_three_corner_recovery_valid(
                                    current_box,
                                    clean_top_cnt,
                                    known_fit,
                                    x1,
                                    y1,
                                    x2,
                                    y2,
                                    ROI_INNER_MARGIN,
                                    latched_mode=True,
                                )
                            )

                coordinate_processing_allowed = (
                    full_inside_roi or top_exit_recovery_valid
                )

                if not coordinate_processing_allowed:
                    if not stop_request_sent:
                        roi_enter_hit_count = 0

                    # ROI 밖에서도 전체 화면 추적은 계속한다. 다만 아래쪽 진입 중이거나
                    # 좌우/아래 경계까지 벗어난 후보는 복원 좌표를 사용하지 않는다.
                    cv2.drawContours(display, [current_box], 0, (255, 128, 0), 2)
                    cv2.circle(
                        display,
                        (int(round(cx)), int(round(cy))),
                        5,
                        (0, 128, 255),
                        -1
                    )

                    cv2.putText(
                        display,
                        "FULL-FRAME TRACKING / WAIT FULL BOX IN ROI",
                        (20, 45),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2
                    )

                else:
                    # 정상 전체 진입 또는 신뢰 가능한 상단 이탈 3꼭짓점 복구일 때
                    # 좌표·각도 계산과 p/A/G 전송 로직을 수행한다.
                    last_object_seen_time = time.time()

                    # 이미 보낸 물체는 다시 판정하지 않음
                    if one_shot_locked:
                        target_deg = locked_target_deg if locked_target_deg is not None else 0.0

                        cv2.drawContours(display, [current_box], 0, (0, 255, 0), 2)

                        if locked_cx is not None and locked_cy is not None:
                            cv2.circle(display, (locked_cx, locked_cy), 5, (0, 0, 255), -1)

                        cv2.putText(
                            display,
                            "LOCKED - IGNORE CURRENT OBJECT",
                            (20, 45),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.9,
                            (0, 255, 255),
                            2
                        )

                        cv2.putText(
                            display,
                            "LOCKED XYZ=({}, {}, {:.3f})".format(
                                locked_cx,
                                locked_cy,
                                locked_z_m
                            ),
                            (20, 90),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 255),
                            2
                        )

                        cv2.putText(
                            display,
                            "LOCKED DXL: {:.2f} deg".format(target_deg),
                            (20, 130),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (255, 0, 0),
                            2
                        )

                    else:
                        measurement_ok = is_valid_measurement(yaw_raw, cx, cy)

                        if measurement_ok:
                            # =================================================
                            # 바운딩 박스 각도는 minAreaRect 기준 사용
                            # 단, XY 좌표용 중심점은 contour의 무게중심을 따로 사용
                            # 이렇게 하면 모서리 쪽으로 치우친 박스 중심보다 실제 마스크 중심에 가까워짐
                            # =================================================
                            if known_size_used:
                                # 부분 윤곽의 무게중심은 보이는 쪽으로 치우치므로
                                # 100 x 70 mm로 복원한 전체 제품 중심을 좌표 추출에 사용한다.
                                center_for_xy_x = int(round(cx))
                                center_for_xy_y = int(round(cy))
                            else:
                                M = cv2.moments(best_cnt)

                                if M["m00"] != 0:
                                    # 정상 검출에서는 기존 contour 무게중심 계산을 그대로 사용한다.
                                    center_for_xy_x = int(M["m10"] / M["m00"])
                                    center_for_xy_y = int(M["m01"] / M["m00"])
                                else:
                                    center_for_xy_x = int(cx)
                                    center_for_xy_y = int(cy)

                            yaw_hist.append(yaw_raw)
                            cx_hist.append(center_for_xy_x)
                            cy_hist.append(center_for_xy_y)

                            # 최근 몇 프레임의 중앙값으로 1차 튐 제거
                            yaw_median = float(np.median(yaw_hist))
                            cx_median = int(np.median(cx_hist))
                            cy_median = int(np.median(cy_hist))

                            z_raw_m = patch_median_depth_m(
                                depth_image,
                                depth_scale,
                                cx_median,
                                cy_median,
                                r=12,
                                min_valid=8
                            )

                            # 복원 중심이 어두운/가려진 영역에 위치해 patch Depth가 없으면
                            # 관측된 contour 내부 중앙 Depth를 사용한다.
                            if (
                                z_raw_m <= 0
                                and known_size_used
                                and known_fit is not None
                            ):
                                z_raw_m = float(known_fit["depth_m"])

                            raw_short_side = min(rw, rh)
                            raw_long_side = max(rw, rh)

                            # EMA + deadband로 2차 흔들림 제거
                            cx_s, cy_s, yaw, z_m, box_short_s, box_long_s = update_smoothed_detection(
                                cx_median,
                                cy_median,
                                yaw_median,
                                z_raw_m,
                                raw_short_side,
                                raw_long_side
                            )

                            target_deg = yaw_to_dxl_target_deg(yaw)

                            last_valid_yaw = yaw
                            last_valid_cx = cx_s
                            last_valid_cy = cy_s
                            last_valid_depth_m = z_m
                            last_valid_box = current_box.copy()

                            # =================================================
                            # 고정 ArUco 원점 기준 Robot XY[mm] 변환
                            # 저장 당시 ArUco 마커 중심이 Robot (0, 0)이 된다.
                            # =================================================
                            robot_result = pixel_depth_to_robot_xy_mm(
                                cx_s,
                                cy_s,
                                z_m
                            )

                            if robot_result is not None:
                                robot_x_mm = robot_result[0]
                                robot_y_mm = robot_result[1]
                                marker_z_mm = robot_result[2]

                                print(
                                    "Robot XY [mm] = X:{:.1f}, Y:{:.1f}, MarkerZ:{:.1f} | Pixel=({}, {}) Z={:.3f}m".format(
                                        robot_x_mm,
                                        robot_y_mm,
                                        marker_z_mm,
                                        cx_s,
                                        cy_s,
                                        z_m
                                    )
                                )

                            else:
                                robot_x_mm = None
                                robot_y_mm = None
                                marker_z_mm = None


                            # =================================================
                            # master.py TCP 통신
                            # 1) 제품 전체 ROI 진입 시 p 전송
                            # 2) 정지 안정화 시간 경과 후 최신 A/G 패킷 1회 전송
                            # =================================================
                            vision_data = VisionData(
                                x_mm=robot_x_mm if robot_x_mm is not None else 0.0,
                                y_mm=robot_y_mm if robot_y_mm is not None else 0.0,
                                z_mm=STM32_PICK_Z_MM,
                                target_deg=target_deg,
                                pixel_x=cx_s,
                                pixel_y=cy_s,
                                depth_m=z_m,
                                marker_z_mm=(
                                    marker_z_mm if marker_z_mm is not None else 0.0
                                ),
                                yaw_deg=yaw,
                            )

                            # 정지 후에도 매 프레임 최신 좌표를 갱신한다.
                            if robot_x_mm is not None and robot_y_mm is not None and z_m > 0:
                                master_client.update_latest_vision(vision_data)

                            if not stop_request_sent:
                                roi_enter_hit_count = min(
                                    ROI_ENTRY_REQUIRED_FRAMES,
                                    roi_enter_hit_count + 1,
                                )

                                if roi_enter_hit_count >= ROI_ENTRY_REQUIRED_FRAMES:
                                    if master_client.send_stop_request():
                                        stop_request_sent = True
                                        stop_request_time = time.monotonic()

                                        # 두 번째 코드와 동일하게 이동 중 누적값은 버리고,
                                        # 정지 이후 영상값만 좌표 전송에 사용한다.
                                        yaw_hist.clear()
                                        cx_hist.clear()
                                        cy_hist.clear()
                                        master_client.clear_latest_vision()

                                        smoothed_cx = None
                                        smoothed_cy = None
                                        smoothed_yaw = None
                                        smoothed_z_m = None
                                        smoothed_short_side = None
                                        smoothed_long_side = None

                                        stop_reason = (
                                            "ROI/영상 상단 절단 규격 복원"
                                            if top_exit_recovery_valid
                                            else "제품 전체 ROI 진입"
                                        )

                                        print(
                                            "{} 확인 -> master로 p 전송 완료 "
                                            "(연속 {}/{})".format(
                                                stop_reason,
                                                roi_enter_hit_count,
                                                ROI_ENTRY_REQUIRED_FRAMES,
                                            )
                                        )

                            if stop_request_sent and not capture_result_sent:
                                elapsed_after_stop = time.monotonic() - stop_request_time

                                ready_to_send = (
                                    elapsed_after_stop >= STOP_SETTLE_DELAY_SEC
                                    and is_detection_stable()
                                    and z_m > 0
                                    and robot_x_mm is not None
                                    and robot_y_mm is not None
                                )

                                if ready_to_send:
                                    if master_client.send_stable_result():
                                        capture_result_sent = True
                                        one_shot_locked = True

                                        locked_target_deg = target_deg
                                        locked_cx = cx_s
                                        locked_cy = cy_s
                                        locked_z_m = z_m

                                        print(
                                            "TCP ONE-SHOT LOCKED | "
                                            "Pixel=({}, {}) Z={:.3f} "
                                            "RobotXY=({:.1f}, {:.1f}) Target={:.2f}".format(
                                                locked_cx,
                                                locked_cy,
                                                locked_z_m,
                                                robot_x_mm,
                                                robot_y_mm,
                                                locked_target_deg,
                                            )
                                        )

                            # =================================================
                            # 시각화
                            # =================================================
                            # 화면에 그리는 박스도 raw minAreaRect가 아니라
                            # 안정화된 center / yaw / size로 그린다.
                            box = cv2.boxPoints(((cx_s, cy_s), (box_short_s, box_long_s), yaw))
                            box = np.int32(box)

                            cv2.drawContours(display, [box], 0, (0, 255, 0), 2)
                            cv2.circle(display, (cx_s, cy_s), 5, (0, 0, 255), -1)

                            if known_size_used:
                                cv2.putText(
                                    display,
                                    "100x70mm COMPLETION",
                                    (20, 300),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.75,
                                    (255, 0, 255),
                                    2,
                                )

                            if top_exit_recovery_valid:
                                cv2.putText(
                                    display,
                                    "TOP-CLIPPED SIZE RECOVERY",
                                    (20, 340),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.72,
                                    (0, 165, 255),
                                    2,
                                )

                            cv2.putText(
                                display,
                                "Yaw: {:.2f} deg".format(yaw),
                                (20, 45),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1.0,
                                (0, 0, 255),
                                2
                            )

                            cv2.putText(
                                display,
                                "Center: ({}, {})".format(cx_s, cy_s),
                                (20, 90),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.9,
                                (255, 0, 0),
                                2
                            )

                            cv2.putText(
                                display,
                                "Depth Z: {:.3f} m".format(z_m),
                                (20, 135),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.9,
                                (0, 255, 255),
                                2
                            )

                            if robot_x_mm is not None and robot_y_mm is not None:
                                cv2.putText(
                                    display,
                                    "Robot XY: X={:.1f}mm, Y={:.1f}mm".format(
                                        robot_x_mm,
                                        robot_y_mm
                                    ),
                                    (20, 180),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.8,
                                    (255, 255, 0),
                                    2
                                )

                                cv2.putText(
                                    display,
                                    "Origin: fixed ArUco center (0,0)",
                                    (20, 215),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    (255, 255, 0),
                                    2
                                )
                            else:
                                cv2.putText(
                                    display,
                                    "Robot XY: conversion failed",
                                    (20, 180),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.8,
                                    (0, 0, 255),
                                    2
                                )

                            if not master_client.connected:
                                status_text = "WAIT MASTER TCP"
                            elif capture_result_sent:
                                status_text = "TCP A/G SENT - LOCKED"
                            elif stop_request_sent:
                                elapsed_after_stop = time.time() - stop_request_time
                                remain = max(0.0, STOP_SETTLE_DELAY_SEC - elapsed_after_stop)
                                status_text = "p SENT / SETTLE {:.2f}s".format(remain)
                            elif top_exit_recovery_valid:
                                status_text = "3-CORNER RECOVERY / SEND p"
                            elif is_detection_stable() and z_m > 0:
                                status_text = "ROI IN / SEND p"
                            else:
                                status_text = "FAST STABLE: {}/{}".format(
                                    len(yaw_hist),
                                    YAW_STABLE_FRAME_REQUIRED
                                )

                            cv2.putText(
                                display,
                                status_text,
                                (20, 255),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 255, 255),
                                2
                            )

                        else:
                            cv2.putText(
                                display,
                                "UNSTABLE DETECTION",
                                (20, 45),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1.0,
                                (0, 0, 255),
                                2
                            )

            else:
                now = time.time()
                lost_elapsed = now - last_object_seen_time

                if retired_target_center is not None:
                    # 직전 흡착 제품만 후보에서 제외되어 검출 결과가 없는 경우
                    # 일반 초기화를 하면 직전 제품이 다시 선택되므로 그대로 기다린다.
                    cv2.putText(
                        display,
                        "WAIT OLD PRODUCT CLEAR",
                        (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 165, 255),
                        2
                    )

                elif not stop_request_sent:
                    if lost_elapsed >= PRE_STOP_LOST_REACQUIRE_SEC:
                        reset_one_shot_lock()
                        yaw_hist.clear()
                        cx_hist.clear()
                        cy_hist.clear()

                elif capture_result_sent:
                    # 정상 잠금 해제는 STATE 3이며, 이 조건은 통신 누락 시 fallback이다.
                    if lost_elapsed >= PICKUP_STATE_FALLBACK_UNLOCK_SEC:
                        if one_shot_locked:
                            print(
                                "STATE 3 미수신 - 2초 영상 미검출 fallback으로 "
                                "TCP one-shot 잠금 해제"
                            )

                        reset_one_shot_lock()
                        yaw_hist.clear()
                        cx_hist.clear()
                        cy_hist.clear()

                else:
                    cv2.putText(
                        display,
                        "WAIT SAME OBJECT AFTER p",
                        (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 165, 255),
                        2
                    )

                cv2.putText(
                    display,
                    "NO OBJECT",
                    (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2
                )

        else:
            if AUTO_BACKGROUND_ENABLE and auto_background_done == False:
                remain_time = AUTO_BACKGROUND_DELAY_SEC - (time.time() - program_start_time)

                if remain_time < 0:
                    remain_time = 0

                cv2.putText(
                    display,
                    "Auto background in {:.1f} sec".format(remain_time),
                    (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2
                )

                cv2.putText(
                    display,
                    "Keep full frame empty",
                    (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2
                )

            else:
                cv2.putText(
                    display,
                    "Press 'b' to save background",
                    (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2
                )

        # ROI 표시
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 255), 2)

        cv2.imshow("RealSense Tracking", display)
        # cv2.imshow("Clean Mask", mask)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('b'):
            background_depth = processing_depth.copy()

            yaw_hist.clear()
            cx_hist.clear()
            cy_hist.clear()

            reset_one_shot_lock()

            last_object_seen_time = time.time()
            auto_background_done = True

            print("수동 배경 저장 완료")
            print("one-shot 잠금 초기화 완료")

        elif key == ord('r'):
            background_depth = None

            yaw_hist.clear()
            cx_hist.clear()
            cy_hist.clear()

            reset_one_shot_lock()

            program_start_time = time.time()
            auto_background_done = False

            print("배경 초기화")
            print("5초 뒤 자동 배경 저장을 다시 시작합니다.")
            print("one-shot 잠금 초기화 완료")

        elif key == ord('u'):
            yaw_hist.clear()
            cx_hist.clear()
            cy_hist.clear()

            reset_one_shot_lock()

            print("수동 one-shot 잠금 해제 완료")

        elif key == ord('q') or key == 27:
            break

finally:
    master_client.stop()
    pipeline.stop()
    cv2.destroyAllWindows()
    print("master.py TCP 연결 종료")