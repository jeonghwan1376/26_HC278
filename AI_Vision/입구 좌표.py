
import cv2
import numpy as np
import pyrealsense2 as rs
import socket
import json
import time
import threading
import queue
from collections import deque

# =========================================================
# Isaac Sim TCP 설정
# =========================================================
USE_ISAAC_TCP = True
ISAAC_HOST = "172.30.1.64"
ISAAC_PORT = 9999
ISAAC_SOCKET_TIMEOUT_SEC = 0.5
ISAAC_RECONNECT_INTERVAL_SEC = 2.0
ISAAC_SEND_QUEUE_SIZE = 100

ISAAC_YAW_SIGN = 1.0
ISAAC_YAW_OFFSET_DEG = 0.0

isaac_socket = None
isaac_connected = False
isaac_send_queue = queue.Queue(maxsize=ISAAC_SEND_QUEUE_SIZE)
isaac_stop_event = threading.Event()
isaac_sender_thread = None
current_object_id = 1

# 현재 바운딩 경로 확인용
current_pose_mode = "NONE"
current_pose_solidity = 0.0
current_pose_fill_ratio = 0.0

# =========================================================
# RealSense / ROI / 배경차분 설정
# =========================================================
WIDTH, HEIGHT, FPS = 1280, 720, 30


USE_ROI = True
ROI_X1, ROI_Y1 = 460, 50
ROI_X2, ROI_Y2 = 970, 605

AUTO_BACKGROUND_ENABLE = True
AUTO_BACKGROUND_DELAY_SEC = 5.0

# 배경을 한 프레임만 저장하지 않고 여러 프레임의 중앙값으로 저장한다.
# 고정된 위치에서 일시적으로 Depth가 0이 되는 현상을 줄인다.
BACKGROUND_CAPTURE_FRAMES = 30
BACKGROUND_MIN_VALID_FRAMES = 3
BACKGROUND_INPAINT_RADIUS = 7
BACKGROUND_MEDIAN_BLUR_SIZE = 5

# 배경보다 현재 물체가 가까워진 정도[m]
MIN_DEPTH_DIFF = 0.006
MAX_DEPTH_DIFF = 0.400
BACKGROUND_VALID_MIN = 50

USE_SPATIAL_FILTER = True
USE_TEMPORAL_FILTER = False
USE_HOLE_FILLING_FILTER = False

# =========================================================
# RealSense Depth 밀도/안정성 설정
# =========================================================
# IR 도트 개수 자체는 하드웨어 고정이지만,
# High Density preset과 레이저 출력을 사용해 유효 Depth 픽셀을 늘린다.
USE_HIGH_DENSITY_PRESET = True
ENABLE_IR_EMITTER = True
USE_MAX_LASER_POWER = True
LASER_POWER_RATIO = 1.0

# 이동 중인 컨베이어이므로 Temporal Filter는 사용하지 않는다.
# Spatial Filter만 약하게 적용해 경계를 보존하면서 작은 깊이 노이즈를 줄인다.
SPATIAL_FILTER_MAGNITUDE = 2.0
SPATIAL_FILTER_SMOOTH_ALPHA = 0.55
SPATIAL_FILTER_SMOOTH_DELTA = 8.0
SPATIAL_FILTER_HOLES_FILL = 0.0

# 외곽의 0-depth 픽셀은 물체 밖까지 번질 수 있으므로 사용하지 않음
USE_DEPTH_HOLE_MASK = False
SAFE_DEPTH_HOLE_MASK = True
HOLE_SUPPORT_DILATE_SIZE = 15
HOLE_SUPPORT_DILATE_ITERATIONS = 1

# 컬러 배경 차분을 함께 사용해서 깊이 노이즈가 배경으로 번지는 현상 방지
USE_COLOR_BACKGROUND_MASK = True
COLOR_DIFF_THRESHOLD = 22
COLOR_BLUR_SIZE = 5
COLOR_OPEN_SIZE = 3
COLOR_CLOSE_SIZE = 5
DEPTH_COLOR_SUPPORT_DILATE_SIZE = 5

FINAL_MASK_ERODE_SIZE = 3
FINAL_MASK_ERODE_ITERATIONS = 0

SHOW_CLEAN_MASK = True

# =========================================================
# 박스 후보 필터
# =========================================================
MIN_AREA = 15000
MAX_AREA = 99999999

# 제품의 가로세로 비율은 고정하지 않음
MIN_RECT_SHORT_SIDE = 50
MIN_RECT_LONG_SIDE = 120

REQUIRE_FULL_BOX_IN_ROI = True
ROI_INNER_MARGIN = 2

# =========================================================
# Pose rect 복원 설정 - 정밀 바운딩 버전
# =========================================================
# 좌우에 가늘게 붙은 돌출 마스크를 먼저 제거
POSE_OPEN_SIZE = 19
POSE_OPEN_ITERATIONS = 1

# OPEN 이후 제품 내부의 작은 틈만 다시 연결
POSE_CLOSE_SIZE = 7
POSE_CLOSE_ITERATIONS = 1

# 각도 계산용 내부 코어: 외곽의 울퉁불퉁한 부분을 제외하고 PCA로 방향 계산
POSE_CORE_ERODE_SIZE = 15
POSE_CORE_ERODE_ITERATIONS = 1
POSE_CORE_MIN_PIXEL = 500

# 제품 본체가 반대 축 방향으로 충분히 길게 이어지는 구간만 외곽으로 사용
# 한쪽에 붙은 얇은 돌출이 바운딩 중심과 폭을 밀어내는 현상을 방지
POSE_EDGE_BIN_SIZE = 2.0
POSE_SHORT_EDGE_SUPPORT_RATIO = 0.42
POSE_LONG_EDGE_SUPPORT_RATIO = 0.42
POSE_EDGE_MIN_RUN_BINS = 5
POSE_EDGE_PADDING_PIXEL = 0.0

# 긴 방향은 조밀한 구간만 고르면 제품의 좁은 윗부분/아랫부분이 잘릴 수 있다.
# 따라서 긴 방향은 전체 마스크의 백분위 경계를 사용한다.
POSE_USE_DENSE_LONG_BOUNDS = False
POSE_LONG_END_PADDING_PIXEL = 2.0

# 조밀한 외곽 계산 실패 시에만 사용하는 fallback
POSE_TRIM_SHORT_PERCENT = 7.0
POSE_TRIM_LONG_PERCENT = 1.0

# 최종 바운딩 크기 미세 조절
POSE_SIZE_SCALE = 1.0
POSE_PADDING_PIXEL = 0.0

# 너무 큰 rect는 좌표 계산에 쓰지 않음
MAX_POSE_SHORT_SIDE = 330
MAX_POSE_LONG_SIDE = 540
MIN_POSE_AREA_RATIO = 0.35

# =========================================================
# 불량/변형 박스 적응형 바운딩 설정
# =========================================================
# 정상 박스는 기존 정밀 바운딩을 사용하고,
# 외곽이 패이거나 끊어진 불량 박스만 완화된 마스크 처리와 convex hull 복원을 사용한다.
USE_ADAPTIVE_DEFECT_POSE = True

# contour가 아래 기준보다 불규칙하면 불량 형상 경로를 사용한다.
# 이전 0.88 / 0.68은 삼각형 돌출이 있어도 거의 직사각형이면
# 정상으로 판정되는 문제가 있어 기준을 강화했다.
DEFECT_SOLIDITY_THRESHOLD = 0.975
DEFECT_FILL_RATIO_THRESHOLD = 0.945

# 한 항목이 매우 나쁜 경우에는 다른 항목과 관계없이 불량으로 판정한다.
DEFECT_STRONG_SOLIDITY_THRESHOLD = 0.93
DEFECT_STRONG_FILL_RATIO_THRESHOLD = 0.86

# 불량 박스에서는 큰 OPEN이 남은 외곽까지 지울 수 있으므로 작게 사용
DEFECT_POSE_OPEN_SIZE = 5
DEFECT_POSE_CLOSE_SIZE = 15

# 불량 박스는 정상 박스보다 일부 마스크가 사라질 수 있으므로 허용 범위를 완화
DEFECT_MIN_POSE_AREA_RATIO = 0.15

# 불량 돌출부를 사각형 크기에 포함하지 않도록,
# 긴 방향에서도 본체 폭이 충분히 이어지는 구간만 사용한다.
DEFECT_USE_DENSE_LONG_BOUNDS = True

# 불량에서는 긴 방향과 짧은 방향 모두 본체가 충분히 이어지는 구간만 사용한다.
# 값이 높을수록 좁게 튀어나온 불량부를 더 강하게 제외한다.
DEFECT_LONG_EDGE_SUPPORT_RATIO = 0.62
DEFECT_SHORT_EDGE_SUPPORT_RATIO = 0.62

DEFECT_LONG_END_PADDING_PIXEL = 0.0
DEFECT_TRIM_LONG_PERCENT = 2.0
DEFECT_TRIM_SHORT_PERCENT = 5.0

# 가장 긴 구간이 아니라 제품 중심을 포함하는 본체 구간을 우선 사용한다.
DEFECT_USE_CENTERED_LONG_BOUNDS = True
DEFECT_USE_CENTERED_SHORT_BOUNDS = True

# 불량 바운딩이 실제 외곽 밖으로 나가지 않도록
# 계산된 사각형을 네 방향에서 안쪽으로 줄인다.
DEFECT_INSET_LONG_PIXEL = 0.0
DEFECT_INSET_SHORT_PIXEL = 0.0

# 불량일 때만 본체 사각형 밖으로 튀어나온 삼각형/날개 부분을 제거한다.
DEFECT_REMOVE_OUTSIDE_PROTRUSION = True

# 1차로 계산한 본체 사각형을 이 값만큼 안쪽으로 줄인 뒤 마스크를 자른다.
# 너무 크게 하면 본체까지 잘릴 수 있으므로 1~3px 정도가 적당하다.
DEFECT_CLIP_RECT_INSET_PIXEL = 2.0

# 잘라낸 뒤 경계의 작은 끊김만 연결한다.
DEFECT_CLIP_CLOSE_SIZE = 5

# 잘라낸 마스크로 최종 바운딩을 다시 계산한다.
DEFECT_RECALCULATE_AFTER_CLIP = True

# 불량일 때는 외곽을 감싸는 사각형이 아니라,
# 실제 흰색 마스크 내부에 완전히 들어가는 가장 큰 사각형을 사용한다.
DEFECT_USE_MAX_INNER_RECT = True

# 최대 내접 사각형 계산 전에 외곽을 조금 안쪽으로 줄인다.
# 1이면 경계보다 약 1px 안쪽, 0이면 원래 마스크 경계까지 사용한다.
DEFECT_INNER_MASK_ERODE_SIZE = 1
DEFECT_INNER_MASK_ERODE_ITERATIONS = 0

# 계산된 내접 사각형을 네 방향에서 추가로 줄이는 값
DEFECT_INNER_RECT_INSET_PIXEL = 0.0

# warp/rounding 때문에 불량 내접 사각형이 약간 작게 나오는 경우가 있어
# 최종 표시용 사각형은 마스크 내부에서 가능한 만큼만 적응형으로 키운다.
DEFECT_INNER_RECT_EXPAND_PIXEL = 0.0

DEFECT_USE_ADAPTIVE_INNER_EXPAND = True
DEFECT_GROW_SEARCH_ITER = 8
DEFECT_GROW_SAMPLE_COUNT = 11

# 키운 뒤 실제 외곽을 넘지 않도록 아주 약한 안전 여유를 둔다.
DEFECT_GROW_SAFETY_PIXEL = 0.0

# 큰 삼각형/지붕 모양 불량에서는 비슷한 면적의 내접 사각형 후보가
# 여러 개 생길 수 있다. 이전 프레임과 가까운 후보를 우선해 점프를 막는다.
DEFECT_USE_INNER_RECT_CONTINUITY = True
DEFECT_INNER_CANDIDATE_AREA_RATIO = 0.84
DEFECT_INNER_CENTER_PENALTY = 1.25
DEFECT_INNER_SIZE_PENALTY = 0.50
DEFECT_INNER_DEFAULT_CENTER_PENALTY = 0.12

# 불량 내접 사각형이 작은 홈 때문에 왼쪽/위쪽에서 안쪽으로 들어오는 경우,
# 새로 추가되는 변의 일정 비율 이상이 실제 마스크일 때만 방향별로 확장한다.
DEFECT_USE_DIRECTIONAL_EDGE_EXPAND = True

# 물체 회전 방향에 따라 실제 문제 변이 warp 영상의 어느 방향이 될지 달라지므로
# 네 방향 모두 동일한 거리까지 검사한다.
DEFECT_EXPAND_LEFT_MAX_PIXEL = 10
DEFECT_EXPAND_TOP_MAX_PIXEL = 10
DEFECT_EXPAND_RIGHT_MAX_PIXEL = 10
DEFECT_EXPAND_BOTTOM_MAX_PIXEL = 10

# 현재 잘 맞는 확장 기준은 유지한다.
DEFECT_EDGE_EXPAND_SUPPORT_RATIO = 0.58

# 모서리의 삼각형 돌출을 제외하고 각 변의 중앙 구간만 검사한다.
DEFECT_EDGE_EXPAND_CENTRAL_RATIO = 0.68
DEFECT_EDGE_EXPAND_MIN_RUN_RATIO = 0.52

# 1픽셀 정도의 Depth 누락은 건너뛰고 다음 줄을 확인한다.
DEFECT_EDGE_EXPAND_MAX_FAIL = 1

# 확장 후에도 불량 돌출 방향의 변이 삼각형 쪽으로 남으면,
# 해당 변만 본체가 충분히 이어지는 위치까지 안쪽으로 다시 정리한다.
DEFECT_USE_BODY_EDGE_TRIM = True
DEFECT_BODY_EDGE_TRIM_MAX_PIXEL = 18
DEFECT_BODY_EDGE_TRIM_BAND = 3
DEFECT_BODY_EDGE_TRIM_CENTRAL_RATIO = 0.80
DEFECT_BODY_EDGE_TRIM_SUPPORT_RATIO = 0.83
DEFECT_BODY_EDGE_TRIM_MIN_RUN_RATIO = 0.70

# 너무 작은 잘못된 내접 사각형을 사용하지 않기 위한 최소 면적 비율
DEFECT_INNER_RECT_MIN_AREA_RATIO = 0.35

# Convex hull은 각도 안정화에만 내부적으로 사용하고,
# 바운딩 크기 계산에는 사용하지 않는다.
DEFECT_USE_CONVEX_HULL_FOR_SIZE = False

# =========================================================
# 안정화 / one-shot 설정
# =========================================================
# 컨베이어 이동 중에도 전송할 수 있도록 중심의 '전체 이동량'이 아니라
# 프레임 사이의 갑작스러운 점프만 검사한다.
ALLOW_MOVING_OBJECT_SEND = True

YAW_STABLE_FRAME_REQUIRED = 4
YAW_STABLE_RANGE_DEG = 3.0

# 정지 물체 모드에서만 사용하는 중심 안정 범위
CENTER_STABLE_RANGE_PIXEL = 10

# 이동 물체 모드에서 허용하는 프레임당 최대 중심 이동량
MOVING_MAX_CENTER_STEP_PIXEL = 35.0

# 프레임 사이 속도가 갑자기 크게 달라지는 잘못된 검출 방지
MOVING_MAX_STEP_CHANGE_PIXEL = 20.0

OBJECT_LOST_UNLOCK_TIME = 0.5

SMOOTH_ALPHA = 0.35
CENTER_DEADBAND_PIXEL = 3.0
YAW_DEADBAND_DEG = 0.8
DEPTH_DEADBAND_M = 0.003

DISPLAY_BOX_SMOOTH_FRAMES = 7
BOX_SIZE_SMOOTH_FRAMES = 7

# =========================================================
# 바운딩 흔들림 방지 설정
# =========================================================
# 물체가 컨베이어 위에서 이동해도 실제 크기와 각도는 거의 변하지 않으므로
# 처음 안정적으로 측정된 크기/각도를 잠그고 중심만 계속 추적한다.
USE_GEOMETRY_LOCK = True
GEOMETRY_LOCK_FRAME_REQUIRED = 5
GEOMETRY_SHORT_SPAN_MAX_PIXEL = 12.0
GEOMETRY_LONG_SPAN_MAX_PIXEL = 16.0
GEOMETRY_YAW_SPAN_MAX_DEG = 3.5

# NORMAL/DEFECT 경로가 프레임마다 바뀌는 것을 막는다.
USE_POSE_MODE_LOCK = True
POSE_MODE_VOTE_FRAMES = 5
POSE_MODE_DEFECT_VOTE_REQUIRED = 2

one_shot_locked = False
locked_cx = None
locked_cy = None
locked_z_m = None
locked_yaw = None
last_object_seen_time = 0.0

smoothed_cx = None
smoothed_cy = None
smoothed_yaw = None
smoothed_z_m = None

last_valid_cx = None
last_valid_cy = None
last_valid_yaw = None

# 현재 물체의 형상 경로와 크기/각도 잠금
locked_pose_mode = None
pose_mode_vote_hist = deque(maxlen=POSE_MODE_VOTE_FRAMES)

geometry_locked = False
locked_box_short = None
locked_box_long = None
locked_box_yaw = None
geometry_short_hist = deque(maxlen=GEOMETRY_LOCK_FRAME_REQUIRED)
geometry_long_hist = deque(maxlen=GEOMETRY_LOCK_FRAME_REQUIRED)
geometry_yaw_hist = deque(maxlen=GEOMETRY_LOCK_FRAME_REQUIRED)

# 큰 불량에서 최대 내접 사각형 후보가 프레임마다 바뀌지 않도록
# 이전 프레임의 정규화된 내접 사각형을 기억한다.
last_defect_inner_rect_norm = None

yaw_hist = deque(maxlen=YAW_STABLE_FRAME_REQUIRED)
cx_hist = deque(maxlen=YAW_STABLE_FRAME_REQUIRED)
cy_hist = deque(maxlen=YAW_STABLE_FRAME_REQUIRED)
short_hist = deque(maxlen=BOX_SIZE_SMOOTH_FRAMES)
long_hist = deque(maxlen=BOX_SIZE_SMOOTH_FRAMES)

display_cx_hist = deque(maxlen=DISPLAY_BOX_SMOOTH_FRAMES)
display_cy_hist = deque(maxlen=DISPLAY_BOX_SMOOTH_FRAMES)
display_yaw_hist = deque(maxlen=DISPLAY_BOX_SMOOTH_FRAMES)
display_short_hist = deque(maxlen=DISPLAY_BOX_SMOOTH_FRAMES)
display_long_hist = deque(maxlen=DISPLAY_BOX_SMOOTH_FRAMES)

# =========================================================
# 고정 ArUco 원점 좌표계 설정
# =========================================================
USE_FIXED_ARUCO_ORIGIN = True

FIXED_ARUCO_RVEC = np.array(
    [2.1768212,  -2.17777892,  0.03568968],
    dtype=np.float32
)

FIXED_ARUCO_TVEC = np.array(
    [0.03078465, -0.01319338,  0.39355337],
    dtype=np.float32
)

ROBOT_X_SIGN = -1.0
ROBOT_Y_SIGN = -1.0
ROBOT_OFFSET_X_MM = 0.0
ROBOT_OFFSET_Y_MM = 0.0

CAM_FX = None
CAM_FY = None
CAM_PPX = None
CAM_PPY = None

R_MARKER_TO_CAM, _ = cv2.Rodrigues(FIXED_ARUCO_RVEC)
FIXED_ARUCO_TVEC_COL = FIXED_ARUCO_TVEC.reshape(3, 1)

# =========================================================
# 각도 / 안정화 함수
# =========================================================
def odd_size(value):
    value = int(value)
    if value < 3:
        value = 3
    if value % 2 == 0:
        value += 1
    return value


def normalize_angle_180(angle):
    angle = float(angle)
    while angle > 90.0:
        angle -= 180.0
    while angle < -90.0:
        angle += 180.0
    return angle


def angle_diff_180(a, b):
    return (float(a) - float(b) + 90.0) % 180.0 - 90.0


def median_angle_180(values):
    if len(values) == 0:
        return 0.0
    reference = float(values[-1])
    unwrapped = [reference + angle_diff_180(v, reference) for v in values]
    return normalize_angle_180(float(np.median(unwrapped)))


def angle_span_180(values):
    if len(values) == 0:
        return 9999.0
    ref = median_angle_180(values)
    return max(abs(angle_diff_180(v, ref)) for v in values)


def normalize_yaw(angle, w, h):
    # 기존 코드와 동일하게 세로 긴 변 기준 yaw로 정리
    if w < h:
        yaw = angle + 90.0
    else:
        yaw = angle

    yaw = normalize_angle_180(yaw)
    yaw = normalize_angle_180(yaw - 90.0)
    return yaw


def yaw_to_isaac_yaw_deg(yaw):
    signed_yaw = (
        ISAAC_YAW_SIGN * float(yaw)
        + ISAAC_YAW_OFFSET_DEG
    )

    positive_yaw = signed_yaw % 180.0

    if positive_yaw >= 179.999999:
        positive_yaw = 0.0

    return float(positive_yaw)

def apply_deadband_scalar(prev, curr, deadband):
    if prev is None:
        return float(curr)
    if abs(float(curr) - float(prev)) < deadband:
        return float(prev)
    return float(curr)


def apply_deadband_angle(prev, curr, deadband):
    curr = normalize_angle_180(curr)
    if prev is None:
        return curr
    diff = angle_diff_180(curr, prev)
    if abs(diff) < deadband:
        return normalize_angle_180(prev)
    return curr


def ema_scalar(prev, curr, alpha):
    if prev is None:
        return float(curr)
    return alpha * float(prev) + (1.0 - alpha) * float(curr)


def ema_angle_180(prev, curr, alpha):
    curr = normalize_angle_180(curr)
    if prev is None:
        return curr
    diff = angle_diff_180(curr, prev)
    return normalize_angle_180(float(prev) + (1.0 - alpha) * diff)


def update_smoothed_detection(cx, cy, yaw, z_m):
    global smoothed_cx, smoothed_cy, smoothed_yaw, smoothed_z_m

    cx_db = apply_deadband_scalar(smoothed_cx, cx, CENTER_DEADBAND_PIXEL)
    cy_db = apply_deadband_scalar(smoothed_cy, cy, CENTER_DEADBAND_PIXEL)
    yaw_db = apply_deadband_angle(smoothed_yaw, yaw, YAW_DEADBAND_DEG)

    if z_m > 0:
        z_db = apply_deadband_scalar(smoothed_z_m, z_m, DEPTH_DEADBAND_M)
    else:
        z_db = smoothed_z_m

    smoothed_cx = ema_scalar(smoothed_cx, cx_db, SMOOTH_ALPHA)
    smoothed_cy = ema_scalar(smoothed_cy, cy_db, SMOOTH_ALPHA)
    smoothed_yaw = ema_angle_180(smoothed_yaw, yaw_db, SMOOTH_ALPHA)

    if z_db is not None:
        smoothed_z_m = ema_scalar(smoothed_z_m, z_db, SMOOTH_ALPHA)

    return (
        int(round(smoothed_cx)),
        int(round(smoothed_cy)),
        normalize_angle_180(smoothed_yaw),
        0.0 if smoothed_z_m is None else float(smoothed_z_m)
    )


def is_detection_stable():
    if len(yaw_hist) < YAW_STABLE_FRAME_REQUIRED:
        return False
    if len(cx_hist) < YAW_STABLE_FRAME_REQUIRED:
        return False
    if len(cy_hist) < YAW_STABLE_FRAME_REQUIRED:
        return False

    yaw_values = list(yaw_hist)
    cx_values = np.asarray(list(cx_hist), dtype=np.float32)
    cy_values = np.asarray(list(cy_hist), dtype=np.float32)

    yaw_range = angle_span_180(yaw_values)
    if yaw_range > YAW_STABLE_RANGE_DEG:
        return False

    if not ALLOW_MOVING_OBJECT_SEND:
        cx_range = float(np.max(cx_values) - np.min(cx_values))
        cy_range = float(np.max(cy_values) - np.min(cy_values))

        return (
            cx_range <= CENTER_STABLE_RANGE_PIXEL
            and cy_range <= CENTER_STABLE_RANGE_PIXEL
        )

    # 컨베이어가 움직이면 중심 좌표가 계속 변하는 것이 정상이다.
    # 따라서 전체 중심 범위를 제한하지 않고 프레임 사이 점프만 확인한다.
    dx = np.diff(cx_values)
    dy = np.diff(cy_values)
    center_steps = np.sqrt(dx * dx + dy * dy)

    if center_steps.size == 0:
        return False

    if float(np.max(center_steps)) > MOVING_MAX_CENTER_STEP_PIXEL:
        return False

    if center_steps.size >= 2:
        step_changes = np.abs(np.diff(center_steps))
        if float(np.max(step_changes)) > MOVING_MAX_STEP_CHANGE_PIXEL:
            return False

    return True



def update_pose_mode_lock(candidate_is_defect):
    """
    NORMAL/DEFECT 판정이 마스크 노이즈로 프레임마다 바뀌는 것을 방지한다.
    한 물체에 대해 모드가 정해지면 물체가 사라질 때까지 유지한다.
    """
    global locked_pose_mode

    if not USE_POSE_MODE_LOCK:
        return bool(candidate_is_defect)

    if locked_pose_mode == "DEFECT":
        return True

    if locked_pose_mode == "NORMAL":
        return False

    pose_mode_vote_hist.append(bool(candidate_is_defect))

    if len(pose_mode_vote_hist) >= POSE_MODE_VOTE_FRAMES:
        defect_votes = sum(1 for value in pose_mode_vote_hist if value)

        if defect_votes >= POSE_MODE_DEFECT_VOTE_REQUIRED:
            locked_pose_mode = "DEFECT"
            return True

        locked_pose_mode = "NORMAL"
        return False

    # 투표가 끝나기 전에는 현재 프레임 판정을 사용한다.
    return bool(candidate_is_defect)


def update_geometry_lock(short_side, long_side, yaw_deg):
    """
    처음 여러 프레임에서 안정된 크기와 각도를 구한 뒤 잠근다.
    이후에는 중심 좌표만 계속 이동시키므로 바운딩이 튀지 않는다.
    """
    global geometry_locked
    global locked_box_short, locked_box_long, locked_box_yaw

    short_side = float(short_side)
    long_side = float(long_side)
    yaw_deg = normalize_angle_180(float(yaw_deg))

    if not USE_GEOMETRY_LOCK:
        return short_side, long_side, yaw_deg

    # NORMAL/DEFECT 경로가 결정되기 전에 서로 다른 크기를 섞어 학습하면
    # 지붕형 불량에서 바운딩이 크게 전환될 수 있다.
    if USE_POSE_MODE_LOCK and locked_pose_mode is None:
        geometry_short_hist.clear()
        geometry_long_hist.clear()
        geometry_yaw_hist.clear()
        return short_side, long_side, yaw_deg

    if geometry_locked:
        return (
            float(locked_box_short),
            float(locked_box_long),
            float(locked_box_yaw)
        )

    geometry_short_hist.append(short_side)
    geometry_long_hist.append(long_side)
    geometry_yaw_hist.append(yaw_deg)

    if len(geometry_short_hist) < GEOMETRY_LOCK_FRAME_REQUIRED:
        return short_side, long_side, yaw_deg

    short_values = np.asarray(geometry_short_hist, dtype=np.float32)
    long_values = np.asarray(geometry_long_hist, dtype=np.float32)
    yaw_values = list(geometry_yaw_hist)

    short_span = float(np.max(short_values) - np.min(short_values))
    long_span = float(np.max(long_values) - np.min(long_values))
    yaw_span = float(angle_span_180(yaw_values))

    if (
        short_span <= GEOMETRY_SHORT_SPAN_MAX_PIXEL
        and long_span <= GEOMETRY_LONG_SPAN_MAX_PIXEL
        and yaw_span <= GEOMETRY_YAW_SPAN_MAX_DEG
    ):
        locked_box_short = float(np.median(short_values))
        locked_box_long = float(np.median(long_values))
        locked_box_yaw = float(median_angle_180(yaw_values))
        geometry_locked = True

        print(
            "[BOUNDING LOCK] "
            f"short={locked_box_short:.1f}px, "
            f"long={locked_box_long:.1f}px, "
            f"yaw={locked_box_yaw:.2f}deg"
        )

        return (
            locked_box_short,
            locked_box_long,
            locked_box_yaw
        )

    # 안정 범위를 벗어나면 오래된 값이 자동으로 빠지면서 다시 시도한다.
    return short_side, long_side, yaw_deg


def reset_tracking_state():
    global one_shot_locked, locked_cx, locked_cy, locked_z_m, locked_yaw
    global smoothed_cx, smoothed_cy, smoothed_yaw, smoothed_z_m
    global last_valid_cx, last_valid_cy, last_valid_yaw
    global locked_pose_mode
    global geometry_locked, locked_box_short, locked_box_long, locked_box_yaw
    global last_defect_inner_rect_norm

    one_shot_locked = False
    locked_cx = None
    locked_cy = None
    locked_z_m = None
    locked_yaw = None

    smoothed_cx = None
    smoothed_cy = None
    smoothed_yaw = None
    smoothed_z_m = None

    last_valid_cx = None
    last_valid_cy = None
    last_valid_yaw = None

    locked_pose_mode = None
    pose_mode_vote_hist.clear()

    geometry_locked = False
    locked_box_short = None
    locked_box_long = None
    locked_box_yaw = None
    geometry_short_hist.clear()
    geometry_long_hist.clear()
    geometry_yaw_hist.clear()

    last_defect_inner_rect_norm = None

    yaw_hist.clear()
    cx_hist.clear()
    cy_hist.clear()
    short_hist.clear()
    long_hist.clear()

    display_cx_hist.clear()
    display_cy_hist.clear()
    display_yaw_hist.clear()
    display_short_hist.clear()
    display_long_hist.clear()

# =========================================================
# 마스크 처리 / pose rect 계산
# =========================================================
def fill_internal_holes(mask):
    if mask is None:
        return mask

    h, w = mask.shape[:2]
    flood = mask.copy()
    flood_mask = np.zeros((h + 2, w + 2), dtype=np.uint8)
    cv2.floodFill(flood, flood_mask, (0, 0), 255)
    flood_inv = cv2.bitwise_not(flood)
    return cv2.bitwise_or(mask, flood_inv)


def clean_depth_mask(mask):
    # 큰 CLOSE 커널은 물체 외곽을 부풀리므로 작게 유지한다.
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

    cleaned = cv2.medianBlur(mask, 3)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, open_kernel, iterations=1)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, close_kernel, iterations=1)
    cleaned = keep_largest_component(cleaned)
    cleaned = fill_internal_holes(cleaned)

    # 최종 외곽을 안쪽으로 조금 줄여 배경 경계가 포함되지 않게 한다.
    erode_size = odd_size(FINAL_MASK_ERODE_SIZE)
    erode_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (erode_size, erode_size)
    )
    cleaned = cv2.erode(
        cleaned,
        erode_kernel,
        iterations=FINAL_MASK_ERODE_ITERATIONS
    )
    cleaned = keep_largest_component(cleaned)
    cleaned = fill_internal_holes(cleaned)
    return cleaned


def build_color_change_mask(current_bgr, background_bgr):
    """빈 컨베이어 컬러 영상과 현재 영상을 비교해 실제 물체 영역을 만든다."""
    if current_bgr is None or background_bgr is None:
        return None
    if current_bgr.shape != background_bgr.shape:
        return None

    blur_size = odd_size(COLOR_BLUR_SIZE)
    current_blur = cv2.GaussianBlur(current_bgr, (blur_size, blur_size), 0)
    background_blur = cv2.GaussianBlur(background_bgr, (blur_size, blur_size), 0)

    color_diff = cv2.absdiff(current_blur, background_blur)
    # 색상 채널 중 하나라도 충분히 변하면 물체 후보로 본다.
    max_channel_diff = np.max(color_diff, axis=2).astype(np.uint8)
    color_mask = np.zeros(max_channel_diff.shape, dtype=np.uint8)
    color_mask[max_channel_diff >= int(COLOR_DIFF_THRESHOLD)] = 255

    open_size = odd_size(COLOR_OPEN_SIZE)
    close_size = odd_size(COLOR_CLOSE_SIZE)
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_size, open_size))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (close_size, close_size))

    color_mask = cv2.morphologyEx(
        color_mask,
        cv2.MORPH_OPEN,
        open_kernel,
        iterations=1
    )
    color_mask = cv2.morphologyEx(
        color_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=1
    )
    return color_mask


def combine_depth_and_color_mask(depth_mask, color_mask):
    """
    깊이 변화와 컬러 변화를 동시에 만족하는 영역을 외곽 기준으로 사용한다.
    컬러 마스크는 깊이 물체 주변에서만 내부 결손을 보충한다.
    """
    if color_mask is None:
        return depth_mask.copy()

    color_support_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (5, 5)
    )
    color_support = cv2.dilate(color_mask, color_support_kernel, iterations=1)

    # 배경에서 깊이만 흔들린 픽셀은 제거한다.
    depth_core = cv2.bitwise_and(depth_mask, color_support)

    support_size = odd_size(DEPTH_COLOR_SUPPORT_DILATE_SIZE)
    depth_support_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (support_size, support_size)
    )
    depth_support = cv2.dilate(depth_core, depth_support_kernel, iterations=1)

    # 물체 안쪽의 깊이 누락만 컬러 변화로 보충한다.
    color_fill = cv2.bitwise_and(color_mask, depth_support)
    return cv2.bitwise_or(depth_core, color_fill)


def build_safe_hole_mask(depth_mask, bg_valid, cur_valid):
    raw_hole_mask = np.zeros(depth_mask.shape, dtype=np.uint8)
    raw_hole_mask[bg_valid & (~cur_valid)] = 255

    if not SAFE_DEPTH_HOLE_MASK:
        return raw_hole_mask

    size = odd_size(HOLE_SUPPORT_DILATE_SIZE)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    support = cv2.dilate(depth_mask, kernel, iterations=HOLE_SUPPORT_DILATE_ITERATIONS)
    return cv2.bitwise_and(raw_hole_mask, support)


def is_contour_fully_inside_roi_local(cnt, roi_w, roi_h, margin=2):
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


def contour_area_or_zero(cnt):
    if cnt is None:
        return 0.0
    return float(cv2.contourArea(cnt))


def choose_best_contour(contours, roi_w, roi_h):
    global last_valid_cx, last_valid_cy

    valid_items = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue

        if REQUIRE_FULL_BOX_IN_ROI:
            if not is_contour_fully_inside_roi_local(cnt, roi_w, roi_h, ROI_INNER_MARGIN):
                continue

        rect = cv2.minAreaRect(cnt)
        (cx, cy), (rw, rh), _ = rect
        rect_area = rw * rh
        if rect_area <= 0:
            continue

        short_side = min(rw, rh)
        long_side = max(rw, rh)
        if short_side < MIN_RECT_SHORT_SIDE or long_side < MIN_RECT_LONG_SIDE:
            continue

        fill_ratio = area / rect_area

        # 이전 중심이 있으면 가까운 후보 우선, 없으면 ROI 중심에 가까운 후보 우선
        if last_valid_cx is None or last_valid_cy is None:
            ref_x = roi_w / 2.0
            ref_y = roi_h / 2.0
        else:
            ref_x = last_valid_cx
            ref_y = last_valid_cy

        dist = ((cx - ref_x) ** 2 + (cy - ref_y) ** 2) ** 0.5

        # 특정 가로세로 비율을 강제하지 않고 면적, 채움률, 이전 위치만 사용
        score = area + area * fill_ratio * 0.7 - dist * 35.0
        valid_items.append((score, cnt))

    if len(valid_items) == 0:
        return None

    valid_items.sort(key=lambda x: x[0], reverse=True)
    return valid_items[0][1]


def keep_largest_component(mask):
    """이진 마스크에서 가장 큰 연결 성분만 남긴다."""
    if mask is None or mask.size == 0:
        return mask

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return np.zeros_like(mask)

    largest_label = 1
    largest_area = int(stats[1, cv2.CC_STAT_AREA])

    for i in range(2, count):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area > largest_area:
            largest_area = area
            largest_label = i

    result = np.zeros_like(mask)
    result[labels == largest_label] = 255
    return result


def percentile_bounds(values, trim_percent):
    """양쪽 극단 노이즈를 제외한 하한과 상한을 반환한다."""
    trim = max(0.0, min(20.0, float(trim_percent)))
    low, high = np.percentile(values, [trim, 100.0 - trim])
    return float(low), float(high)


def largest_true_run(valid_flags, min_run_bins):
    """True가 연속된 구간 중 가장 긴 구간의 시작/끝을 반환한다."""
    best_start = None
    best_end = None
    best_length = 0
    current_start = None

    for index, is_valid in enumerate(valid_flags):
        if is_valid:
            if current_start is None:
                current_start = index
        else:
            if current_start is not None:
                length = index - current_start
                if length > best_length:
                    best_start = current_start
                    best_end = index - 1
                    best_length = length
                current_start = None

    if current_start is not None:
        length = len(valid_flags) - current_start
        if length > best_length:
            best_start = current_start
            best_end = len(valid_flags) - 1
            best_length = length

    if best_start is None or best_length < int(min_run_bins):
        return None

    return best_start, best_end


def centered_true_run(valid_flags, center_index, min_run_bins):
    """
    True가 연속된 구간 중에서 중심 bin을 포함하는 구간을 우선 선택한다.
    중심을 포함하는 구간이 없으면 중심에 가장 가까운 구간을 선택한다.
    """
    runs = []
    current_start = None

    for index, is_valid in enumerate(valid_flags):
        if is_valid:
            if current_start is None:
                current_start = index
        else:
            if current_start is not None:
                end = index - 1
                length = end - current_start + 1
                if length >= int(min_run_bins):
                    runs.append((current_start, end, length))
                current_start = None

    if current_start is not None:
        end = len(valid_flags) - 1
        length = end - current_start + 1
        if length >= int(min_run_bins):
            runs.append((current_start, end, length))

    if len(runs) == 0:
        return None

    center_index = int(center_index)

    containing_runs = []
    for start, end, length in runs:
        if start <= center_index <= end:
            containing_runs.append((start, end, length))

    if len(containing_runs) > 0:
        containing_runs.sort(key=lambda x: x[2], reverse=True)
        return containing_runs[0][0], containing_runs[0][1]

    def run_distance(item):
        start, end, _ = item
        run_center = 0.5 * (start + end)
        return abs(run_center - center_index)

    runs.sort(key=lambda x: (run_distance(x), -x[2]))
    return runs[0][0], runs[0][1]


def dense_projection_bounds(axis_values, cross_values, bin_size,
                            support_ratio, min_run_bins, padding_pixel,
                            prefer_center=False, center_value=None):
    """
    한 축의 각 위치에서 반대 축 방향으로 본체가 얼마나 길게 이어지는지 계산한다.
    얇은 돌출은 지지 길이가 짧으므로 제외되고 조밀한 본체 구간만 남는다.
    """
    if axis_values.size < 100 or cross_values.size != axis_values.size:
        return None

    bin_size = max(1.0, float(bin_size))
    axis_min = float(np.min(axis_values))
    axis_max = float(np.max(axis_values))
    cross_min = float(np.min(cross_values))

    axis_count = int(np.floor((axis_max - axis_min) / bin_size)) + 1
    if axis_count < int(min_run_bins):
        return None

    axis_index = np.floor((axis_values - axis_min) / bin_size).astype(np.int32)
    cross_index = np.floor((cross_values - cross_min) / bin_size).astype(np.int32)
    axis_index = np.clip(axis_index, 0, axis_count - 1)

    pairs = np.column_stack((axis_index, cross_index))
    unique_pairs = np.unique(pairs, axis=0)
    occupancy = np.bincount(unique_pairs[:, 0], minlength=axis_count).astype(np.float32)

    if occupancy.size >= 3:
        occupancy = np.convolve(
            occupancy,
            np.array([0.25, 0.5, 0.25], dtype=np.float32),
            mode="same"
        )

    positive = occupancy[occupancy > 0]
    if positive.size == 0:
        return None

    reference_support = float(np.percentile(positive, 90.0))
    if reference_support <= 0.0:
        return None

    valid = occupancy >= reference_support * float(support_ratio)

    # 한 bin짜리 작은 공백은 메운다.
    if valid.size >= 3:
        closed = valid.copy()
        for i in range(1, valid.size - 1):
            if (not valid[i]) and valid[i - 1] and valid[i + 1]:
                closed[i] = True
        valid = closed

    if prefer_center:
        if center_value is None:
            center_value = 0.5 * (axis_min + axis_max)
        center_bin = int(np.floor((float(center_value) - axis_min) / bin_size))
        center_bin = max(0, min(axis_count - 1, center_bin))
        run = centered_true_run(valid, center_bin, min_run_bins)
    else:
        run = largest_true_run(valid, min_run_bins)

    if run is None:
        return None

    start_bin, end_bin = run
    lower = axis_min + start_bin * bin_size - float(padding_pixel)
    upper = axis_min + (end_bin + 1) * bin_size + float(padding_pixel)
    return float(lower), float(upper)




def contour_shape_metrics(cnt):
    """
    contour의 불규칙도를 계산한다.

    solidity:
        contour 면적 / convex hull 면적
        외곽이 패이거나 찌그러질수록 값이 작아진다.

    fill_ratio:
        contour 면적 / 최소 회전 사각형 면적
        박스 외곽이 많이 끊기거나 일부가 빠질수록 값이 작아진다.
    """
    if cnt is None or len(cnt) < 3:
        return 0.0, 0.0

    area = float(cv2.contourArea(cnt))
    if area <= 0.0:
        return 0.0, 0.0

    hull = cv2.convexHull(cnt)
    hull_area = float(cv2.contourArea(hull))
    solidity = area / hull_area if hull_area > 0.0 else 0.0

    rect = cv2.minAreaRect(cnt)
    rw, rh = rect[1]
    rect_area = float(rw) * float(rh)
    fill_ratio = area / rect_area if rect_area > 0.0 else 0.0

    return float(solidity), float(fill_ratio)


def is_irregular_box_contour(cnt):
    if not USE_ADAPTIVE_DEFECT_POSE:
        return False

    solidity, fill_ratio = contour_shape_metrics(cnt)

    # 삼각형 돌출처럼 전체적으로는 거의 직사각형인 불량은
    # solidity와 fill ratio가 둘 다 조금씩 나빠지는 형태다.
    combined_irregular = (
        solidity < DEFECT_SOLIDITY_THRESHOLD
        and fill_ratio < DEFECT_FILL_RATIO_THRESHOLD
    )

    # 한 지표가 매우 나쁜 심한 불량은 단독으로도 불량 처리한다.
    strongly_irregular = (
        solidity < DEFECT_STRONG_SOLIDITY_THRESHOLD
        or fill_ratio < DEFECT_STRONG_FILL_RATIO_THRESHOLD
    )

    return combined_irregular or strongly_irregular


def robust_pca_rect_from_mask(
    mask,
    use_dense_long_bounds=None,
    long_support_ratio=None,
    short_support_ratio=None,
    long_end_padding_pixel=None,
    trim_long_percent=None,
    trim_short_percent=None,
    use_centered_long_bounds=False,
    use_centered_short_bounds=False,
    inset_long_pixel=0.0,
    inset_short_pixel=0.0
):
    """
    바운딩 크기 계산 방식은 유지하되, 각도만 더 안정적으로 계산한다.

    기존 PCA 각도는 마스크 한쪽 모서리가 패이거나 튀어나오면
    전체 픽셀 분포가 한쪽으로 치우쳐 각도가 틀어질 수 있다.

    수정 방식:
    1. 침식한 내부 코어의 가장 큰 contour를 찾는다.
    2. 코어 contour의 convex hull로 패인 모서리를 복원한다.
    3. hull의 minAreaRect에서 가장 긴 변 방향을 각도로 사용한다.
    4. 실제 크기와 중심은 기존 조밀한 투영 경계 방식으로 계산한다.
    """
    if use_dense_long_bounds is None:
        use_dense_long_bounds = POSE_USE_DENSE_LONG_BOUNDS

    if long_support_ratio is None:
        long_support_ratio = POSE_LONG_EDGE_SUPPORT_RATIO

    if short_support_ratio is None:
        short_support_ratio = POSE_SHORT_EDGE_SUPPORT_RATIO

    if long_end_padding_pixel is None:
        long_end_padding_pixel = POSE_LONG_END_PADDING_PIXEL

    if trim_long_percent is None:
        trim_long_percent = POSE_TRIM_LONG_PERCENT

    if trim_short_percent is None:
        trim_short_percent = POSE_TRIM_SHORT_PERCENT

    component = keep_largest_component(mask)
    if component is None:
        return None

    component_pixel = int(cv2.countNonZero(component))
    if component_pixel < POSE_CORE_MIN_PIXEL:
        return None

    erode_size = odd_size(POSE_CORE_ERODE_SIZE)
    erode_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (erode_size, erode_size)
    )

    core_mask = cv2.erode(
        component,
        erode_kernel,
        iterations=POSE_CORE_ERODE_ITERATIONS
    )
    core_mask = keep_largest_component(core_mask)

    if core_mask is None or cv2.countNonZero(core_mask) < POSE_CORE_MIN_PIXEL:
        core_mask = component.copy()

    core_contours, _ = cv2.findContours(
        core_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    if len(core_contours) == 0:
        return None

    core_cnt = max(core_contours, key=cv2.contourArea)
    core_hull = cv2.convexHull(core_cnt)

    if core_hull is None or len(core_hull) < 3:
        return None

    angle_rect = cv2.minAreaRect(core_hull)
    angle_box = cv2.boxPoints(angle_rect).astype(np.float32)

    longest_vector = None
    longest_length = -1.0

    for i in range(4):
        p1 = angle_box[i]
        p2 = angle_box[(i + 1) % 4]
        vector = p2 - p1
        length = float(np.linalg.norm(vector))

        if length > longest_length:
            longest_length = length
            longest_vector = vector

    if longest_vector is None or longest_length <= 1e-6:
        return None

    long_axis = longest_vector / longest_length

    if (
        long_axis[1] < 0.0
        or (abs(float(long_axis[1])) < 1e-6 and long_axis[0] < 0.0)
    ):
        long_axis = -long_axis

    short_axis = np.array(
        [-long_axis[1], long_axis[0]],
        dtype=np.float32
    )

    origin = np.array(
        [float(angle_rect[0][0]), float(angle_rect[0][1])],
        dtype=np.float32
    )

    measure_y, measure_x = np.where(component > 0)
    if measure_x.size < POSE_CORE_MIN_PIXEL:
        return None

    measure_points = np.column_stack(
        (measure_x, measure_y)
    ).astype(np.float32)

    def measure_with_axes(axis_long, axis_short):
        relative = measure_points - origin

        long_values = np.dot(relative, axis_long)
        short_values = np.dot(relative, axis_short)

        # 정상 박스는 전체 긴 방향을 사용한다.
        # 불량 박스는 본체 폭이 충분히 유지되는 긴 구간만 사용해,
        # 위나 아래에 붙은 좁은 돌출부가 바운딩을 키우지 않게 한다.
        if use_dense_long_bounds:
            long_bounds = dense_projection_bounds(
                long_values,
                short_values,
                POSE_EDGE_BIN_SIZE,
                long_support_ratio,
                POSE_EDGE_MIN_RUN_BINS,
                POSE_EDGE_PADDING_PIXEL,
                prefer_center=use_centered_long_bounds,
                center_value=float(np.median(long_values))
            )
        else:
            long_bounds = None

        short_bounds = dense_projection_bounds(
            short_values,
            long_values,
            POSE_EDGE_BIN_SIZE,
            short_support_ratio,
            POSE_EDGE_MIN_RUN_BINS,
            POSE_EDGE_PADDING_PIXEL,
            prefer_center=use_centered_short_bounds,
            center_value=float(np.median(short_values))
        )

        if long_bounds is None:
            long_min, long_max = percentile_bounds(
                long_values,
                trim_long_percent
            )
        else:
            long_min, long_max = long_bounds

        # 본체 경계가 너무 안쪽으로 들어가지 않도록 아주 작은 여유만 추가한다.
        long_min -= float(long_end_padding_pixel)
        long_max += float(long_end_padding_pixel)

        if short_bounds is None:
            short_min, short_max = percentile_bounds(
                short_values,
                trim_short_percent
            )
        else:
            short_min, short_max = short_bounds

        # 불량 박스는 계산된 외곽보다 반드시 안쪽에 들어오도록
        # 긴 방향과 짧은 방향의 양 끝을 각각 안쪽으로 줄인다.
        inset_long = max(0.0, float(inset_long_pixel))
        inset_short = max(0.0, float(inset_short_pixel))

        if (long_max - long_min) > 2.0 * inset_long + 2.0:
            long_min += inset_long
            long_max -= inset_long

        if (short_max - short_min) > 2.0 * inset_short + 2.0:
            short_min += inset_short
            short_max -= inset_short

        long_center = (long_min + long_max) * 0.5
        short_center = (short_min + short_max) * 0.5

        center = (
            origin
            + long_center * axis_long
            + short_center * axis_short
        )

        measured_long = float(long_max - long_min)
        measured_short = float(short_max - short_min)

        return center, measured_long, measured_short

    center, long_side, short_side = measure_with_axes(
        long_axis,
        short_axis
    )

    if long_side < short_side:
        swapped_long_axis = short_axis.copy()
        swapped_short_axis = -long_axis.copy()

        if (
            swapped_long_axis[1] < 0.0
            or (
                abs(float(swapped_long_axis[1])) < 1e-6
                and swapped_long_axis[0] < 0.0
            )
        ):
            swapped_long_axis = -swapped_long_axis
            swapped_short_axis = -swapped_short_axis

        swapped_center, swapped_long, swapped_short = measure_with_axes(
            swapped_long_axis,
            swapped_short_axis
        )

        if swapped_long >= swapped_short:
            long_axis = swapped_long_axis
            short_axis = swapped_short_axis
            center = swapped_center
            long_side = swapped_long
            short_side = swapped_short

    if short_side <= 1.0 or long_side <= 1.0:
        return None

    short_side = (
        short_side * POSE_SIZE_SCALE
        + 2.0 * POSE_PADDING_PIXEL
    )
    long_side = (
        long_side * POSE_SIZE_SCALE
        + 2.0 * POSE_PADDING_PIXEL
    )

    long_axis_angle = np.degrees(
        np.arctan2(long_axis[1], long_axis[0])
    )
    rect_angle = normalize_angle_180(
        float(long_axis_angle) - 90.0
    )

    return (
        (float(center[0]), float(center[1])),
        (float(short_side), float(long_side)),
        float(rect_angle)
    )



def rotated_rect_to_filled_mask(rect, mask_shape, inset_pixel=0.0):
    """
    회전 사각형 내부만 흰색인 마스크를 만든다.

    inset_pixel이 양수이면 사각형의 네 방향을 안쪽으로 줄인다.
    """
    if rect is None:
        return None

    (cx, cy), (rw, rh), angle = rect

    inset = max(0.0, float(inset_pixel))
    clipped_w = float(rw) - 2.0 * inset
    clipped_h = float(rh) - 2.0 * inset

    if clipped_w <= 2.0 or clipped_h <= 2.0:
        return None

    inner_rect = (
        (float(cx), float(cy)),
        (float(clipped_w), float(clipped_h)),
        float(angle)
    )

    box = cv2.boxPoints(inner_rect)
    box = np.int32(np.round(box))

    rect_mask = np.zeros(mask_shape, dtype=np.uint8)
    cv2.fillConvexPoly(rect_mask, box, 255)

    return rect_mask


def remove_defect_outside_protrusions(pose_mask, body_rect):
    """
    불량 박스에서 본체 사각형 밖으로 튀어나온 삼각형, 날개, 접힌 부분을 제거한다.

    정상 박스에는 호출하지 않는다.
    """
    if pose_mask is None or body_rect is None:
        return pose_mask

    body_rect_mask = rotated_rect_to_filled_mask(
        body_rect,
        pose_mask.shape,
        DEFECT_CLIP_RECT_INSET_PIXEL
    )

    if body_rect_mask is None:
        return pose_mask

    clipped_mask = cv2.bitwise_and(pose_mask, body_rect_mask)
    clipped_mask = keep_largest_component(clipped_mask)

    if clipped_mask is None or cv2.countNonZero(clipped_mask) == 0:
        return pose_mask

    close_size = odd_size(DEFECT_CLIP_CLOSE_SIZE)
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (close_size, close_size)
    )

    clipped_mask = cv2.morphologyEx(
        clipped_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=1
    )
    clipped_mask = keep_largest_component(clipped_mask)
    clipped_mask = fill_internal_holes(clipped_mask)

    if clipped_mask is None or cv2.countNonZero(clipped_mask) < POSE_CORE_MIN_PIXEL:
        return pose_mask

    return clipped_mask



def order_quad_points(points):
    """4개 꼭짓점을 좌상, 우상, 우하, 좌하 순서로 정렬한다."""
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)

    sums = pts[:, 0] + pts[:, 1]
    diffs = pts[:, 1] - pts[:, 0]

    top_left = pts[np.argmin(sums)]
    bottom_right = pts[np.argmax(sums)]
    top_right = pts[np.argmin(diffs)]
    bottom_left = pts[np.argmax(diffs)]

    return np.array(
        [top_left, top_right, bottom_right, bottom_left],
        dtype=np.float32
    )


def largest_axis_aligned_white_rectangle(binary_mask):
    """
    흰색 픽셀로만 이루어진 큰 축 정렬 사각형을 찾는다.

    단순히 면적이 가장 큰 후보 하나만 선택하면,
    지붕형/삼각형 불량에서 비슷한 면적의 후보가 프레임마다 바뀌어
    중심이 크게 점프할 수 있다.

    따라서:
    1. 최대 면적에 가까운 후보들을 모은다.
    2. 이전 프레임 내접 사각형과 중심/크기가 가까운 후보를 선택한다.
    3. 이전 후보가 없으면 영상 중심에 가까운 후보를 우선한다.
    """
    global last_defect_inner_rect_norm

    if binary_mask is None or binary_mask.size == 0:
        return None

    white = binary_mask > 0
    height, width = white.shape[:2]

    if height <= 0 or width <= 0:
        return None

    histogram = np.zeros(width, dtype=np.int32)

    best_area = 0
    candidates = []

    for row_index in range(height):
        histogram = np.where(
            white[row_index],
            histogram + 1,
            0
        ).astype(np.int32)

        stack = []

        for column_index in range(width + 1):
            current_height = (
                int(histogram[column_index])
                if column_index < width
                else 0
            )

            start_index = column_index

            while stack and stack[-1][1] > current_height:
                previous_start, previous_height = stack.pop()
                rectangle_width = column_index - previous_start
                rectangle_area = previous_height * rectangle_width

                if rectangle_area > 0:
                    candidate = (
                        int(previous_start),
                        int(row_index - previous_height + 1),
                        int(rectangle_width),
                        int(previous_height),
                        int(rectangle_area)
                    )
                    candidates.append(candidate)

                    if rectangle_area > best_area:
                        best_area = rectangle_area

                start_index = previous_start

            if (
                not stack
                or stack[-1][1] < current_height
            ):
                stack.append((start_index, current_height))

    if best_area <= 0 or len(candidates) == 0:
        return None

    minimum_area = (
        float(best_area)
        * float(DEFECT_INNER_CANDIDATE_AREA_RATIO)
    )

    previous = (
        last_defect_inner_rect_norm
        if DEFECT_USE_INNER_RECT_CONTINUITY
        else None
    )

    best_score = -1e18
    selected = None

    for x, y, rect_w, rect_h, area in candidates:
        if float(area) < minimum_area:
            continue

        norm_cx = (float(x) + 0.5 * float(rect_w)) / float(width)
        norm_cy = (float(y) + 0.5 * float(rect_h)) / float(height)
        norm_w = float(rect_w) / float(width)
        norm_h = float(rect_h) / float(height)

        area_score = float(area) / float(best_area)

        if previous is not None:
            prev_cx, prev_cy, prev_w, prev_h = previous

            center_distance = (
                (norm_cx - prev_cx) ** 2
                + (norm_cy - prev_cy) ** 2
            ) ** 0.5

            size_distance = (
                abs(norm_w - prev_w)
                + abs(norm_h - prev_h)
            )

            score = (
                area_score
                - DEFECT_INNER_CENTER_PENALTY * center_distance
                - DEFECT_INNER_SIZE_PENALTY * size_distance
            )
        else:
            center_distance = (
                (norm_cx - 0.5) ** 2
                + (norm_cy - 0.5) ** 2
            ) ** 0.5

            score = (
                area_score
                - DEFECT_INNER_DEFAULT_CENTER_PENALTY
                * center_distance
            )

        if score > best_score:
            best_score = score
            selected = (
                int(x),
                int(y),
                int(rect_w),
                int(rect_h)
            )
            selected_norm = (
                float(norm_cx),
                float(norm_cy),
                float(norm_w),
                float(norm_h)
            )

    if selected is None:
        # 안전 fallback: 진짜 최대 면적 후보 사용
        x, y, rect_w, rect_h, _ = max(
            candidates,
            key=lambda item: item[4]
        )
        selected = (
            int(x),
            int(y),
            int(rect_w),
            int(rect_h)
        )
        selected_norm = (
            (float(x) + 0.5 * float(rect_w)) / float(width),
            (float(y) + 0.5 * float(rect_h)) / float(height),
            float(rect_w) / float(width),
            float(rect_h) / float(height)
        )

    last_defect_inner_rect_norm = selected_norm
    return selected

def sample_points_on_rotated_rect(center, long_axis, short_axis,
                                  half_long, half_short, sample_count):
    """
    회전 사각형의 테두리 샘플 포인트를 만든다.
    """
    sample_count = max(5, int(sample_count))

    c = np.asarray(center, dtype=np.float32)
    la = np.asarray(long_axis, dtype=np.float32)
    sa = np.asarray(short_axis, dtype=np.float32)

    points = []

    long_values = np.linspace(-half_long, half_long, sample_count)
    short_values = np.linspace(-half_short, half_short, sample_count)

    for v in long_values:
        points.append(c + v * la - half_short * sa)
        points.append(c + v * la + half_short * sa)

    for u in short_values:
        points.append(c - half_long * la + u * sa)
        points.append(c + half_long * la + u * sa)

    return np.asarray(points, dtype=np.float32)


def rotated_rect_fits_mask(mask, rect):
    """
    주어진 회전 사각형의 테두리 샘플이 모두 흰색 마스크 안에 있으면 True.
    """
    if mask is None or rect is None:
        return False

    (cx, cy), (rw, rh), angle = rect

    if rw <= 2.0 or rh <= 2.0:
        return False

    theta = np.deg2rad(float(angle) + 90.0)
    long_axis = np.array(
        [np.cos(theta), np.sin(theta)],
        dtype=np.float32
    )
    short_axis = np.array(
        [-np.sin(theta), np.cos(theta)],
        dtype=np.float32
    )

    center = np.array([float(cx), float(cy)], dtype=np.float32)
    half_long = float(rh) * 0.5
    half_short = float(rw) * 0.5

    points = sample_points_on_rotated_rect(
        center,
        long_axis,
        short_axis,
        half_long,
        half_short,
        DEFECT_GROW_SAMPLE_COUNT
    )

    h, w = mask.shape[:2]

    for px, py in points:
        ix = int(round(float(px)))
        iy = int(round(float(py)))

        if ix < 0 or iy < 0 or ix >= w or iy >= h:
            return False

        if mask[iy, ix] == 0:
            return False

    return True


def maximize_inner_rect_in_mask(mask, rect):
    """
    현재 불량 내접 사각형을 마스크 내부에서 가능한 만큼만 키운다.
    긴 방향과 짧은 방향을 각각 이분 탐색으로 확장한다.
    """
    if mask is None or rect is None:
        return rect

    (cx, cy), (rw, rh), angle = rect

    base_rect = (
        (float(cx), float(cy)),
        (float(rw), float(rh)),
        float(angle)
    )

    if not rotated_rect_fits_mask(mask, base_rect):
        return rect

    # 긴 방향 확장
    low = 0.0
    high = max(mask.shape) * 0.5

    for _ in range(int(DEFECT_GROW_SEARCH_ITER)):
        mid = 0.5 * (low + high)
        test_rect = (
            (float(cx), float(cy)),
            (float(rw), float(rh) + 2.0 * mid),
            float(angle)
        )

        if rotated_rect_fits_mask(mask, test_rect):
            low = mid
        else:
            high = mid

    grow_long = max(0.0, low - float(DEFECT_GROW_SAFETY_PIXEL))

    rect_after_long = (
        (float(cx), float(cy)),
        (float(rw), float(rh) + 2.0 * grow_long),
        float(angle)
    )

    # 짧은 방향 확장
    low = 0.0
    high = max(mask.shape) * 0.5

    for _ in range(int(DEFECT_GROW_SEARCH_ITER)):
        mid = 0.5 * (low + high)
        test_rect = (
            (float(cx), float(cy)),
            (float(rw) + 2.0 * mid, rect_after_long[1][1]),
            float(angle)
        )

        if rotated_rect_fits_mask(mask, test_rect):
            low = mid
        else:
            high = mid

    grow_short = max(0.0, low - float(DEFECT_GROW_SAFETY_PIXEL))

    final_rect = (
        (float(cx), float(cy)),
        (
            float(rw) + 2.0 * grow_short,
            float(rh) + 2.0 * grow_long
        ),
        float(angle)
    )

    return final_rect



def longest_true_ratio(values):
    """1차원 불리언 배열에서 가장 긴 연속 True 구간의 비율을 반환한다."""
    values = np.asarray(values, dtype=bool).reshape(-1)

    if values.size == 0:
        return 0.0

    best_length = 0
    current_length = 0

    for value in values:
        if value:
            current_length += 1
            if current_length > best_length:
                best_length = current_length
        else:
            current_length = 0

    return float(best_length) / float(values.size)


def get_central_range(start_value, end_value, ratio):
    """주어진 범위에서 중앙 구간만 잘라낸 시작/끝 인덱스를 반환한다."""
    start_value = int(start_value)
    end_value = int(end_value)

    if end_value < start_value:
        start_value, end_value = end_value, start_value

    total_length = end_value - start_value + 1
    ratio = max(0.20, min(1.0, float(ratio)))

    keep_length = max(1, int(round(total_length * ratio)))
    margin = max(0, (total_length - keep_length) // 2)

    central_start = start_value + margin
    central_end = min(end_value, central_start + keep_length - 1)

    return int(central_start), int(central_end)


def close_one_pixel_gaps(values):
    """경계선 안의 한 픽셀 크기 구멍만 메운다."""
    result = np.asarray(values, dtype=bool).reshape(-1).copy()

    if result.size >= 3:
        for index in range(1, result.size - 1):
            if (
                not result[index]
                and result[index - 1]
                and result[index + 1]
            ):
                result[index] = True

    return result


def line_is_supported(values, support_ratio, minimum_run_ratio):
    """경계선이 본체에 충분히 지지되는지 판단한다."""
    values = close_one_pixel_gaps(values)

    if values.size == 0:
        return False

    support = float(np.mean(values))
    run_ratio = longest_true_ratio(values)

    return (
        support >= float(support_ratio)
        and run_ratio >= float(minimum_run_ratio)
    )


def expand_inner_bounds_directionally(
    warped_mask,
    x1,
    y1,
    x2,
    y2
):
    """
    불량 최대 내접 사각형을 네 방향으로 확장한 뒤,
    삼각형/날개 쪽으로 잡힌 변만 본체 위치까지 다시 안쪽으로 정리한다.

    - 네 방향을 같은 거리까지 검사하므로 회전 방향에 따른 편차가 줄어든다.
    - 모서리가 아닌 중앙 구간을 검사하므로 삼각형 돌출을 본체로 오인하지 않는다.
    - 확장 후 엄격한 본체 지지율 검사를 추가해 불량 방향만 선택적으로 줄인다.
    """
    if (
        not DEFECT_USE_DIRECTIONAL_EDGE_EXPAND
        or warped_mask is None
        or warped_mask.size == 0
    ):
        return x1, y1, x2, y2

    mask_bool = warped_mask > 0
    mask_h, mask_w = mask_bool.shape[:2]

    left = int(max(0, round(float(x1))))
    top = int(max(0, round(float(y1))))
    right = int(min(mask_w - 1, round(float(x2))))
    bottom = int(min(mask_h - 1, round(float(y2))))

    expand_support = float(DEFECT_EDGE_EXPAND_SUPPORT_RATIO)
    expand_run = float(DEFECT_EDGE_EXPAND_MIN_RUN_RATIO)
    central_ratio = float(DEFECT_EDGE_EXPAND_CENTRAL_RATIO)
    maximum_fail = max(0, int(DEFECT_EDGE_EXPAND_MAX_FAIL))

    def expand_left(current_left, current_top, current_right, current_bottom):
        central_top, central_bottom = get_central_range(
            current_top,
            current_bottom,
            central_ratio
        )

        original = current_left
        best = current_left
        fail_count = 0

        for step in range(1, int(DEFECT_EXPAND_LEFT_MAX_PIXEL) + 1):
            candidate = original - step

            if candidate < 0:
                break

            values = mask_bool[
                central_top:central_bottom + 1,
                candidate
            ]

            if line_is_supported(values, expand_support, expand_run):
                best = candidate
                fail_count = 0
            else:
                fail_count += 1
                if fail_count > maximum_fail:
                    break

        return best

    def expand_right(current_left, current_top, current_right, current_bottom):
        central_top, central_bottom = get_central_range(
            current_top,
            current_bottom,
            central_ratio
        )

        original = current_right
        best = current_right
        fail_count = 0

        for step in range(1, int(DEFECT_EXPAND_RIGHT_MAX_PIXEL) + 1):
            candidate = original + step

            if candidate >= mask_w:
                break

            values = mask_bool[
                central_top:central_bottom + 1,
                candidate
            ]

            if line_is_supported(values, expand_support, expand_run):
                best = candidate
                fail_count = 0
            else:
                fail_count += 1
                if fail_count > maximum_fail:
                    break

        return best

    def expand_top(current_left, current_top, current_right, current_bottom):
        central_left, central_right = get_central_range(
            current_left,
            current_right,
            central_ratio
        )

        original = current_top
        best = current_top
        fail_count = 0

        for step in range(1, int(DEFECT_EXPAND_TOP_MAX_PIXEL) + 1):
            candidate = original - step

            if candidate < 0:
                break

            values = mask_bool[
                candidate,
                central_left:central_right + 1
            ]

            if line_is_supported(values, expand_support, expand_run):
                best = candidate
                fail_count = 0
            else:
                fail_count += 1
                if fail_count > maximum_fail:
                    break

        return best

    def expand_bottom(current_left, current_top, current_right, current_bottom):
        central_left, central_right = get_central_range(
            current_left,
            current_right,
            central_ratio
        )

        original = current_bottom
        best = current_bottom
        fail_count = 0

        for step in range(1, int(DEFECT_EXPAND_BOTTOM_MAX_PIXEL) + 1):
            candidate = original + step

            if candidate >= mask_h:
                break

            values = mask_bool[
                candidate,
                central_left:central_right + 1
            ]

            if line_is_supported(values, expand_support, expand_run):
                best = candidate
                fail_count = 0
            else:
                fail_count += 1
                if fail_count > maximum_fail:
                    break

        return best

    left = expand_left(left, top, right, bottom)
    right = expand_right(left, top, right, bottom)
    top = expand_top(left, top, right, bottom)
    bottom = expand_bottom(left, top, right, bottom)

    if not DEFECT_USE_BODY_EDGE_TRIM:
        return (
            float(left),
            float(top),
            float(right),
            float(bottom)
        )

    trim_support = float(DEFECT_BODY_EDGE_TRIM_SUPPORT_RATIO)
    trim_run = float(DEFECT_BODY_EDGE_TRIM_MIN_RUN_RATIO)
    trim_ratio = float(DEFECT_BODY_EDGE_TRIM_CENTRAL_RATIO)
    trim_band = max(1, int(DEFECT_BODY_EDGE_TRIM_BAND))
    trim_max = max(0, int(DEFECT_BODY_EDGE_TRIM_MAX_PIXEL))

    def vertical_band_is_body(x_start, x_end, y_start, y_end):
        central_top, central_bottom = get_central_range(
            y_start,
            y_end,
            trim_ratio
        )

        band = mask_bool[
            central_top:central_bottom + 1,
            x_start:x_end
        ]

        if band.size == 0:
            return False

        line_values = np.any(band, axis=1)
        support = float(np.mean(band))

        return (
            support >= trim_support
            and longest_true_ratio(close_one_pixel_gaps(line_values)) >= trim_run
        )

    def horizontal_band_is_body(y_start, y_end, x_start, x_end):
        central_left, central_right = get_central_range(
            x_start,
            x_end,
            trim_ratio
        )

        band = mask_bool[
            y_start:y_end,
            central_left:central_right + 1
        ]

        if band.size == 0:
            return False

        line_values = np.any(band, axis=0)
        support = float(np.mean(band))

        return (
            support >= trim_support
            and longest_true_ratio(close_one_pixel_gaps(line_values)) >= trim_run
        )

    # 삼각형 쪽 변은 지지율이 낮으므로 해당 변만 안쪽으로 이동한다.
    for _ in range(trim_max):
        x_end = min(mask_w, left + trim_band)

        if vertical_band_is_body(left, x_end, top, bottom):
            break

        left += 1

        if left >= right - 2:
            break

    for _ in range(trim_max):
        x_start = max(0, right - trim_band + 1)

        if vertical_band_is_body(x_start, right + 1, top, bottom):
            break

        right -= 1

        if right <= left + 2:
            break

    for _ in range(trim_max):
        y_end = min(mask_h, top + trim_band)

        if horizontal_band_is_body(top, y_end, left, right):
            break

        top += 1

        if top >= bottom - 2:
            break

    for _ in range(trim_max):
        y_start = max(0, bottom - trim_band + 1)

        if horizontal_band_is_body(y_start, bottom + 1, left, right):
            break

        bottom -= 1

        if bottom <= top + 2:
            break

    return (
        float(left),
        float(top),
        float(right),
        float(bottom)
    )

def build_defect_max_inner_rect(pose_mask, guide_rect):
    """
    불량 마스크를 guide_rect 방향으로 정렬한 뒤,
    실제 흰색 마스크 안에 완전히 들어가는 가장 큰 사각형을 계산한다.

    삼각형, 날개, 접힌 부분처럼 본체 밖으로 튀어나온 영역은
    최대 내접 사각형에 포함되지 않는다.
    """
    if pose_mask is None or guide_rect is None:
        return None, pose_mask

    guide_box = cv2.boxPoints(guide_rect).astype(np.float32)
    source_points = order_quad_points(guide_box)

    top_left, top_right, bottom_right, bottom_left = source_points

    width_top = float(np.linalg.norm(top_right - top_left))
    width_bottom = float(np.linalg.norm(bottom_right - bottom_left))
    height_left = float(np.linalg.norm(bottom_left - top_left))
    height_right = float(np.linalg.norm(bottom_right - top_right))

    warp_width = max(3, int(round(max(width_top, width_bottom))))
    warp_height = max(3, int(round(max(height_left, height_right))))

    destination_points = np.array(
        [
            [0.0, 0.0],
            [float(warp_width - 1), 0.0],
            [float(warp_width - 1), float(warp_height - 1)],
            [0.0, float(warp_height - 1)]
        ],
        dtype=np.float32
    )

    transform = cv2.getPerspectiveTransform(
        source_points,
        destination_points
    )
    inverse_transform = cv2.getPerspectiveTransform(
        destination_points,
        source_points
    )

    warped_mask = cv2.warpPerspective(
        pose_mask,
        transform,
        (warp_width, warp_height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0
    )

    warped_mask = np.where(
        warped_mask > 0,
        255,
        0
    ).astype(np.uint8)

    warped_mask = fill_internal_holes(warped_mask)

    erode_size = odd_size(DEFECT_INNER_MASK_ERODE_SIZE)
    if (
        DEFECT_INNER_MASK_ERODE_ITERATIONS > 0
        and erode_size >= 3
    ):
        erode_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (erode_size, erode_size)
        )
        warped_mask = cv2.erode(
            warped_mask,
            erode_kernel,
            iterations=DEFECT_INNER_MASK_ERODE_ITERATIONS
        )

    inner_result = largest_axis_aligned_white_rectangle(
        warped_mask
    )

    if inner_result is None:
        return None, pose_mask

    inner_x, inner_y, inner_width, inner_height = inner_result

    inset = max(
        0.0,
        float(DEFECT_INNER_RECT_INSET_PIXEL)
    )

    x1 = float(inner_x) + inset
    y1 = float(inner_y) + inset
    x2 = float(inner_x + inner_width - 1) - inset
    y2 = float(inner_y + inner_height - 1) - inset

    # 현재 문제인 왼쪽과 위쪽은 실제 마스크 지지율을 확인하며 추가 확장한다.
    x1, y1, x2, y2 = expand_inner_bounds_directionally(
        warped_mask,
        x1,
        y1,
        x2,
        y2
    )

    if x2 <= x1 + 2.0 or y2 <= y1 + 2.0:
        return None, pose_mask

    inner_area = (x2 - x1) * (y2 - y1)
    guide_area = max(
        float(warp_width * warp_height),
        1.0
    )

    if (
        inner_area / guide_area
        < DEFECT_INNER_RECT_MIN_AREA_RATIO
    ):
        return None, pose_mask

    canonical_corners = np.array(
        [
            [[x1, y1]],
            [[x2, y1]],
            [[x2, y2]],
            [[x1, y2]]
        ],
        dtype=np.float32
    )

    original_corners = cv2.perspectiveTransform(
        canonical_corners,
        inverse_transform
    ).reshape(4, 2)

    inner_rect = cv2.minAreaRect(
        original_corners.astype(np.float32)
    )

    # 고정 픽셀 확대 대신, 실제 마스크 내부에서 가능한 만큼만 적응형 확대한다.
    (rect_cx, rect_cy), (rect_w, rect_h), rect_angle = inner_rect

    inner_rect = (
        (float(rect_cx), float(rect_cy)),
        (float(rect_w), float(rect_h)),
        float(rect_angle)
    )

    if DEFECT_USE_ADAPTIVE_INNER_EXPAND:
        inner_rect = maximize_inner_rect_in_mask(
            pose_mask,
            inner_rect
        )
    else:
        expand_pixel = max(0.0, float(DEFECT_INNER_RECT_EXPAND_PIXEL))
        inner_rect = (
            (float(rect_cx), float(rect_cy)),
            (
                float(rect_w) + 2.0 * expand_pixel,
                float(rect_h) + 2.0 * expand_pixel
            ),
            float(rect_angle)
        )

    original_polygon = np.int32(
        np.round(original_corners)
    )

    inner_polygon_mask = np.zeros(
        pose_mask.shape,
        dtype=np.uint8
    )
    cv2.fillConvexPoly(
        inner_polygon_mask,
        original_polygon,
        255
    )

    clipped_pose_mask = cv2.bitwise_and(
        pose_mask,
        inner_polygon_mask
    )
    clipped_pose_mask = keep_largest_component(
        clipped_pose_mask
    )

    if (
        clipped_pose_mask is None
        or cv2.countNonZero(clipped_pose_mask)
        < POSE_CORE_MIN_PIXEL
    ):
        return None, pose_mask

    return inner_rect, clipped_pose_mask


def build_pose_rect_from_contour(cnt, mask_shape):
    global current_pose_mode, current_pose_solidity, current_pose_fill_ratio
    global geometry_locked

    """
    정상 박스와 불량/변형 박스를 자동으로 구분해 바운딩 방식을 바꾼다.

    정상 박스:
        기존 큰 OPEN을 사용해 얇은 외곽 노이즈를 제거한다.

    불량/변형 박스:
        큰 OPEN이 실제 남은 외곽을 잘라버리지 않도록 작은 OPEN을 사용한다.
        패인 모서리와 끊어진 외곽은 convex hull로 복원한 뒤
        중심, 크기, 각도를 계산한다.

    반환하는 pose_mask는 실제 검출 마스크이므로
    Depth 중앙값 계산에 배경 영역이 섞이지 않는다.
    """
    raw_mask = np.zeros(mask_shape, dtype=np.uint8)
    cv2.drawContours(raw_mask, [cnt], -1, 255, -1)

    raw_solidity, raw_fill_ratio = contour_shape_metrics(cnt)
    raw_candidate_is_irregular = is_irregular_box_contour(cnt)
    raw_is_irregular = update_pose_mode_lock(
        raw_candidate_is_irregular
    )

    current_pose_solidity = raw_solidity
    current_pose_fill_ratio = raw_fill_ratio
    current_pose_mode = "DEFECT" if raw_is_irregular else "NORMAL"

    if raw_is_irregular:
        open_size = odd_size(DEFECT_POSE_OPEN_SIZE)
        close_size = odd_size(DEFECT_POSE_CLOSE_SIZE)
    else:
        open_size = odd_size(POSE_OPEN_SIZE)
        close_size = odd_size(POSE_CLOSE_SIZE)

    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (open_size, open_size)
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (close_size, close_size)
    )

    # 불량은 작은 OPEN으로 외곽 손실을 줄이고 큰 CLOSE로 끊어진 부분을 연결한다.
    pose_mask = cv2.morphologyEx(
        raw_mask,
        cv2.MORPH_OPEN,
        open_kernel,
        iterations=POSE_OPEN_ITERATIONS
    )
    pose_mask = cv2.morphologyEx(
        pose_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=POSE_CLOSE_ITERATIONS
    )
    pose_mask = fill_internal_holes(pose_mask)
    pose_mask = keep_largest_component(pose_mask)
    pose_mask = fill_internal_holes(pose_mask)

    if pose_mask is None or cv2.countNonZero(pose_mask) == 0:
        fallback_rect = cv2.minAreaRect(cnt)
        return fallback_rect, raw_mask

    pose_contours, _ = cv2.findContours(
        pose_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(pose_contours) == 0:
        fallback_rect = cv2.minAreaRect(cnt)
        return fallback_rect, raw_mask

    core_cnt = max(pose_contours, key=cv2.contourArea)

    # OPEN/CLOSE 이후 형상도 다시 확인한다.
    core_candidate_is_irregular = is_irregular_box_contour(core_cnt)

    if locked_pose_mode == "DEFECT":
        use_defect_path = True
    elif locked_pose_mode == "NORMAL":
        use_defect_path = False
    else:
        use_defect_path = raw_is_irregular or core_candidate_is_irregular

    if use_defect_path:
        current_pose_mode = "DEFECT"
    else:
        current_pose_mode = "NORMAL"

    raw_area = max(contour_area_or_zero(cnt), 1.0)
    core_area = contour_area_or_zero(core_cnt)

    if use_defect_path:
        minimum_area_ratio = DEFECT_MIN_POSE_AREA_RATIO
    else:
        minimum_area_ratio = MIN_POSE_AREA_RATIO

    if core_area / raw_area < minimum_area_ratio:
        # 돌출부까지 감싸는 convex hull은 사용하지 않는다.
        fallback_rect = cv2.minAreaRect(core_cnt)
        return fallback_rect, pose_mask

    # 크기 계산은 항상 실제 정리 마스크를 사용한다.
    # convex hull로 전체를 채우면 불량 돌출부까지 포함되어 바운딩이 커진다.
    measurement_mask = pose_mask

    if use_defect_path:
        # 1차 본체 사각형은 돌출 제거용이므로 별도의 inset 없이 계산한다.
        preliminary_rect = robust_pca_rect_from_mask(
            measurement_mask,
            use_dense_long_bounds=DEFECT_USE_DENSE_LONG_BOUNDS,
            long_support_ratio=DEFECT_LONG_EDGE_SUPPORT_RATIO,
            short_support_ratio=DEFECT_SHORT_EDGE_SUPPORT_RATIO,
            long_end_padding_pixel=DEFECT_LONG_END_PADDING_PIXEL,
            trim_long_percent=DEFECT_TRIM_LONG_PERCENT,
            trim_short_percent=DEFECT_TRIM_SHORT_PERCENT,
            use_centered_long_bounds=DEFECT_USE_CENTERED_LONG_BOUNDS,
            use_centered_short_bounds=DEFECT_USE_CENTERED_SHORT_BOUNDS,
            inset_long_pixel=0.0,
            inset_short_pixel=0.0
        )

        inner_rect_used = False

        # 불량에서는 실제 마스크 안에 완전히 들어가는 최대 내접 사각형을 우선 사용한다.
        if (
            DEFECT_USE_MAX_INNER_RECT
            and preliminary_rect is not None
        ):
            inner_rect, inner_pose_mask = build_defect_max_inner_rect(
                pose_mask,
                preliminary_rect
            )

            if inner_rect is not None:
                pose_rect = inner_rect
                pose_mask = inner_pose_mask
                measurement_mask = inner_pose_mask
                inner_rect_used = True

        # 최대 내접 사각형 계산에 실패한 경우에만 기존 돌출 제거 방식을 사용한다.
        if not inner_rect_used:
            if (
                DEFECT_REMOVE_OUTSIDE_PROTRUSION
                and preliminary_rect is not None
            ):
                clipped_pose_mask = remove_defect_outside_protrusions(
                    pose_mask,
                    preliminary_rect
                )

                pose_mask = clipped_pose_mask
                measurement_mask = clipped_pose_mask

            if DEFECT_RECALCULATE_AFTER_CLIP:
                pose_rect = robust_pca_rect_from_mask(
                    measurement_mask,
                    use_dense_long_bounds=DEFECT_USE_DENSE_LONG_BOUNDS,
                    long_support_ratio=DEFECT_LONG_EDGE_SUPPORT_RATIO,
                    short_support_ratio=DEFECT_SHORT_EDGE_SUPPORT_RATIO,
                    long_end_padding_pixel=DEFECT_LONG_END_PADDING_PIXEL,
                    trim_long_percent=DEFECT_TRIM_LONG_PERCENT,
                    trim_short_percent=DEFECT_TRIM_SHORT_PERCENT,
                    use_centered_long_bounds=DEFECT_USE_CENTERED_LONG_BOUNDS,
                    use_centered_short_bounds=DEFECT_USE_CENTERED_SHORT_BOUNDS,
                    inset_long_pixel=DEFECT_INSET_LONG_PIXEL,
                    inset_short_pixel=DEFECT_INSET_SHORT_PIXEL
                )
            else:
                pose_rect = preliminary_rect
    else:
        pose_rect = robust_pca_rect_from_mask(
            measurement_mask,
            use_dense_long_bounds=POSE_USE_DENSE_LONG_BOUNDS,
            long_support_ratio=POSE_LONG_EDGE_SUPPORT_RATIO,
            short_support_ratio=POSE_SHORT_EDGE_SUPPORT_RATIO,
            long_end_padding_pixel=POSE_LONG_END_PADDING_PIXEL,
            trim_long_percent=POSE_TRIM_LONG_PERCENT,
            trim_short_percent=POSE_TRIM_SHORT_PERCENT,
            use_centered_long_bounds=False,
            use_centered_short_bounds=False,
            inset_long_pixel=0.0,
            inset_short_pixel=0.0
        )

    if pose_rect is None:
        # 돌출부까지 감싸는 hull 대신 실제 정리 contour를 사용한다.
        fallback_rect = cv2.minAreaRect(core_cnt)
        return fallback_rect, pose_mask

    (_, _), (rw, rh), _ = pose_rect
    short_side = min(rw, rh)
    long_side = max(rw, rh)

    if short_side > MAX_POSE_SHORT_SIDE or long_side > MAX_POSE_LONG_SIDE:
        return None, pose_mask

    return pose_rect, pose_mask

def get_smoothed_display_box(cx, cy, yaw_deg, short_side, long_side):
    display_cx_hist.append(float(cx))
    display_cy_hist.append(float(cy))
    display_yaw_hist.append(float(yaw_deg))
    display_short_hist.append(float(short_side))
    display_long_hist.append(float(long_side))

    smooth_cx = float(np.median(display_cx_hist))
    smooth_cy = float(np.median(display_cy_hist))
    smooth_yaw = median_angle_180(list(display_yaw_hist))
    smooth_short = float(np.median(display_short_hist))
    smooth_long = float(np.median(display_long_hist))

    box = cv2.boxPoints(((smooth_cx, smooth_cy), (smooth_short, smooth_long), smooth_yaw))
    return np.int32(np.round(box))

# =========================================================
# Depth / 좌표 변환
# =========================================================
def patch_median_depth_m(depth_u16, depth_scale, u, v, r=18, min_valid=8):
    h, w = depth_u16.shape[:2]
    u = int(round(max(0, min(w - 1, u))))
    v = int(round(max(0, min(h - 1, v))))

    x1 = max(0, u - r)
    x2 = min(w, u + r + 1)
    y1 = max(0, v - r)
    y2 = min(h, v + r + 1)

    patch = depth_u16[y1:y2, x1:x2]
    valid = patch[patch > 0].astype(np.float32) * float(depth_scale)

    if valid.size < min_valid:
        return 0.0

    return float(np.median(valid))


def masked_median_depth_m(roi_depth, pose_mask, depth_scale):
    valid = roi_depth[(pose_mask > 0) & (roi_depth > 0)].astype(np.float32) * float(depth_scale)
    if valid.size < 30:
        return 0.0
    return float(np.median(valid))


def pixel_depth_to_camera_point(u, v, z_m):
    if CAM_FX is None or CAM_FY is None or CAM_PPX is None or CAM_PPY is None:
        return None
    if z_m <= 0:
        return None

    x_cam = (float(u) - CAM_PPX) / CAM_FX * float(z_m)
    y_cam = (float(v) - CAM_PPY) / CAM_FY * float(z_m)
    z_cam = float(z_m)
    return np.array([x_cam, y_cam, z_cam], dtype=np.float32)


def camera_point_to_fixed_aruco_point(p_cam):
    if p_cam is None:
        return None

    p_cam_col = np.asarray(p_cam, dtype=np.float32).reshape(3, 1)
    p_marker_col = np.dot(R_MARKER_TO_CAM.T, p_cam_col - FIXED_ARUCO_TVEC_COL)
    return p_marker_col.flatten()


def pixel_depth_to_robot_xy_mm(u, v, z_m):
    if not USE_FIXED_ARUCO_ORIGIN:
        return None

    p_cam = pixel_depth_to_camera_point(u, v, z_m)
    p_marker = camera_point_to_fixed_aruco_point(p_cam)

    if p_marker is None:
        return None

    # 기존 코드의 X/Y 교체 방식 유지
    robot_x_mm = ROBOT_X_SIGN * float(p_marker[1]) * 1000.0 + ROBOT_OFFSET_X_MM
    robot_y_mm = ROBOT_Y_SIGN * float(p_marker[0]) * 1000.0 + ROBOT_OFFSET_Y_MM
    marker_z_mm = float(p_marker[2]) * 1000.0

    return robot_x_mm, robot_y_mm, marker_z_mm

# =========================================================
# Isaac TCP
# =========================================================
def close_isaac_socket():
    global isaac_socket, isaac_connected
    if isaac_socket is not None:
        try:
            isaac_socket.close()
        except Exception:
            pass
    isaac_socket = None
    isaac_connected = False


def connect_isaac_background():
    global isaac_socket, isaac_connected

    if not USE_ISAAC_TCP:
        isaac_connected = False
        return True

    close_isaac_socket()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(ISAAC_SOCKET_TIMEOUT_SEC)
        sock.connect((ISAAC_HOST, ISAAC_PORT))
        isaac_socket = sock
        isaac_connected = True
        print(f"[ISAAC TCP] 연결 성공: {ISAAC_HOST}:{ISAAC_PORT}")
        return True
    except Exception as error:
        close_isaac_socket()
        print(f"[ISAAC TCP] 연결 실패: {error} | {ISAAC_RECONNECT_INTERVAL_SEC:.1f}초 뒤 재시도")
        return False


def isaac_sender_worker():
    global isaac_socket

    pending_data = None
    next_connect_time = 0.0

    while not isaac_stop_event.is_set():
        if pending_data is None:
            try:
                pending_data = isaac_send_queue.get(timeout=0.1)
            except queue.Empty:
                continue

        if not USE_ISAAC_TCP:
            print("[ISAAC TCP TEST]", pending_data)
            pending_data = None
            continue

        if isaac_socket is None:
            now = time.time()
            if now < next_connect_time:
                time.sleep(0.05)
                continue
            if not connect_isaac_background():
                next_connect_time = time.time() + ISAAC_RECONNECT_INTERVAL_SEC
                continue

        try:
            message = json.dumps(pending_data, ensure_ascii=False) + "\n"
            isaac_socket.sendall(message.encode("utf-8"))

            pos = pending_data["position"]
            yaw = pending_data["rotation_deg"]["yaw"]
            print(
                "[ISAAC TCP SEND] "
                f"ID={pending_data['id']}, "
                f"X={pos['x']:.1f}mm, Y={pos['y']:.1f}mm, Z={pos['z']:.1f}mm, "
                f"Yaw={yaw:.2f}deg"
            )
            pending_data = None

        except Exception as error:
            print(f"[ISAAC TCP] 전송 실패: {error} | 메시지 보관 후 재연결")
            close_isaac_socket()
            next_connect_time = time.time() + ISAAC_RECONNECT_INTERVAL_SEC


def start_isaac_sender():
    global isaac_sender_thread
    if isaac_sender_thread is not None and isaac_sender_thread.is_alive():
        return

    isaac_stop_event.clear()
    isaac_sender_thread = threading.Thread(target=isaac_sender_worker, daemon=True)
    isaac_sender_thread.start()


def stop_isaac_sender():
    isaac_stop_event.set()
    close_isaac_socket()
    if isaac_sender_thread is not None:
        isaac_sender_thread.join(timeout=1.0)


def send_isaac_create_box(object_id, x_mm, y_mm, z_mm, yaw_deg):
    data = {
        "cmd": "CREATE_BOX",
        "id": int(object_id),
        "track_id": int(object_id),
        "position": {
            "x": float(x_mm),
            "y": float(y_mm),
            "z": float(z_mm)
        },
        "rotation_deg": {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": float(yaw_deg)
        },
        "position_unit": "mm",
        "coordinate_system": "fixed_aruco_center"
    }

    try:
        isaac_send_queue.put_nowait(data)
        print(
            "[ISAAC QUEUED] "
            f"ID={object_id}, X={x_mm:.1f}mm, Y={y_mm:.1f}mm, "
            f"Z={z_mm:.1f}mm, Yaw={yaw_deg:.2f}deg"
        )
        return True
    except queue.Full:
        print("[ISAAC TCP] 전송 대기열이 가득 찼습니다.")
        return False


# =========================================================
# 다중 프레임 배경 생성
# =========================================================
def build_robust_background_depth(depth_samples):
    """
    여러 프레임에서 유효한 Depth만 모아 픽셀별 중앙값 배경을 만든다.

    한두 프레임에서 Depth가 0으로 빠져도 다른 프레임의 값을 사용한다.
    여러 프레임 동안 계속 0인 위치는 주변 배경 깊이로 보간한다.
    """
    if depth_samples is None or len(depth_samples) == 0:
        return None, 0.0, 0.0

    stack = np.stack(depth_samples, axis=0).astype(np.float32)
    valid = stack > float(BACKGROUND_VALID_MIN)
    valid_count = np.sum(valid, axis=0)

    # 잘못된 Depth는 NaN으로 바꿔 중앙값 계산에서 제외한다.
    stack[~valid] = np.nan

    with np.errstate(invalid="ignore"):
        median_depth = np.nanmedian(stack, axis=0)

    original_valid_mask = (
        np.isfinite(median_depth)
        & (valid_count >= int(BACKGROUND_MIN_VALID_FRAMES))
    )

    valid_ratio_before = float(np.count_nonzero(original_valid_mask)) / float(
        original_valid_mask.size
    )

    median_depth[~original_valid_mask] = 0.0

    # 여러 프레임에서도 계속 비어 있던 고정 Depth hole을 주변 값으로 보간한다.
    missing_mask = np.zeros(median_depth.shape, dtype=np.uint8)
    missing_mask[~original_valid_mask] = 255

    if np.any(missing_mask):
        valid_values = median_depth[original_valid_mask]

        if valid_values.size > 0:
            fallback_depth = float(np.median(valid_values))
            working_depth = median_depth.astype(np.float32)

            # inpaint가 큰 구멍을 처리하기 쉽게 완전히 비어 있는 부분에
            # 우선 대표 배경값을 아주 약하게 제공한 뒤 주변 경계로 보간한다.
            working_depth[~original_valid_mask] = fallback_depth

            # 실제 inpaint 입력에서는 누락 마스크 위치를 복원 대상으로 지정한다.
            filled_depth = cv2.inpaint(
                working_depth,
                missing_mask,
                float(BACKGROUND_INPAINT_RADIUS),
                cv2.INPAINT_TELEA
            )
        else:
            return None, valid_ratio_before, 0.0
    else:
        filled_depth = median_depth.astype(np.float32)

    blur_size = odd_size(BACKGROUND_MEDIAN_BLUR_SIZE)
    if blur_size >= 3:
        filled_depth = cv2.medianBlur(filled_depth, blur_size)

    filled_depth = np.clip(
        np.rint(filled_depth),
        0,
        np.iinfo(np.uint16).max
    ).astype(np.uint16)

    valid_ratio_after = float(
        np.count_nonzero(filled_depth > BACKGROUND_VALID_MIN)
    ) / float(filled_depth.size)

    return filled_depth, valid_ratio_before, valid_ratio_after


def build_robust_background_color(color_samples):
    """여러 컬러 프레임의 중앙값으로 흔들림이 적은 배경 영상을 만든다."""
    if color_samples is None or len(color_samples) == 0:
        return None

    stack = np.stack(color_samples, axis=0).astype(np.float32)
    background = np.median(stack, axis=0)
    return np.clip(np.rint(background), 0, 255).astype(np.uint8)


# =========================================================
# RealSense 센서 옵션 설정
# =========================================================
def set_option_if_supported(sensor, option_name, value, label):
    try:
        if sensor.supports(option_name):
            option_range = sensor.get_option_range(option_name)
            safe_value = max(
                float(option_range.min),
                min(float(option_range.max), float(value))
            )
            sensor.set_option(option_name, safe_value)
            print(f"[RealSense] {label}: {safe_value}")
            return True
    except Exception as error:
        print(f"[RealSense] {label} 설정 실패: {error}")

    return False


def find_depth_sensor(device):
    """레이저 또는 emitter 옵션을 지원하는 Depth 센서를 찾는다."""
    try:
        sensors = device.query_sensors()
    except Exception:
        sensors = []

    for sensor in sensors:
        try:
            if (
                sensor.supports(rs.option.laser_power)
                or sensor.supports(rs.option.emitter_enabled)
                or sensor.supports(rs.option.visual_preset)
            ):
                return sensor
        except Exception:
            continue

    try:
        return device.first_depth_sensor()
    except Exception:
        return None


def set_high_density_preset(depth_sensor):
    """
    장치가 제공하는 visual preset 설명을 직접 확인해
    'High Density' preset을 찾아 적용한다.
    """
    if depth_sensor is None:
        return False

    try:
        if not depth_sensor.supports(rs.option.visual_preset):
            print("[RealSense] Visual preset 미지원")
            return False

        option_range = depth_sensor.get_option_range(
            rs.option.visual_preset
        )

        value = float(option_range.min)
        step = float(option_range.step)

        if step <= 0:
            step = 1.0

        while value <= float(option_range.max) + 1e-6:
            description = ""

            try:
                description = depth_sensor.get_option_value_description(
                    rs.option.visual_preset,
                    value
                )
            except Exception:
                description = ""

            if description and "high density" in description.lower():
                depth_sensor.set_option(
                    rs.option.visual_preset,
                    value
                )
                print(
                    f"[RealSense] Visual preset: "
                    f"{description} ({value})"
                )
                return True

            value += step

        print("[RealSense] High Density preset을 찾지 못함")
        return False

    except Exception as error:
        print(f"[RealSense] High Density preset 설정 실패: {error}")
        return False


def configure_realsense_depth_sensor(profile):
    """
    유효 Depth 픽셀을 최대한 확보하기 위한 센서 설정.

    순서:
    1. High Density preset
    2. IR emitter 활성화
    3. preset 적용 뒤 레이저 출력을 다시 설정
    """
    device = profile.get_device()
    depth_sensor = find_depth_sensor(device)

    if depth_sensor is None:
        print("[RealSense] Depth 센서를 찾지 못했습니다.")
        return None

    try:
        sensor_name = depth_sensor.get_info(
            rs.camera_info.name
        )
    except Exception:
        sensor_name = "Unknown Depth Sensor"

    print(f"[RealSense] Depth sensor: {sensor_name}")

    if USE_HIGH_DENSITY_PRESET:
        set_high_density_preset(depth_sensor)

    if ENABLE_IR_EMITTER:
        set_option_if_supported(
            depth_sensor,
            rs.option.emitter_enabled,
            1.0,
            "IR emitter"
        )

    if USE_MAX_LASER_POWER:
        try:
            if depth_sensor.supports(rs.option.laser_power):
                laser_range = depth_sensor.get_option_range(
                    rs.option.laser_power
                )

                ratio = max(
                    0.0,
                    min(1.0, float(LASER_POWER_RATIO))
                )

                target_power = (
                    float(laser_range.min)
                    + (
                        float(laser_range.max)
                        - float(laser_range.min)
                    ) * ratio
                )

                depth_sensor.set_option(
                    rs.option.laser_power,
                    target_power
                )

                print(
                    "[RealSense] Laser power: "
                    f"{target_power:.1f} "
                    f"(range {laser_range.min:.1f}"
                    f"~{laser_range.max:.1f})"
                )
            else:
                print("[RealSense] Laser power 옵션 미지원")

        except Exception as error:
            print(f"[RealSense] Laser power 설정 실패: {error}")

    return depth_sensor


def configure_spatial_filter(spatial_filter):
    if spatial_filter is None:
        return

    settings = [
        (
            rs.option.filter_magnitude,
            SPATIAL_FILTER_MAGNITUDE,
            "Spatial magnitude"
        ),
        (
            rs.option.filter_smooth_alpha,
            SPATIAL_FILTER_SMOOTH_ALPHA,
            "Spatial smooth alpha"
        ),
        (
            rs.option.filter_smooth_delta,
            SPATIAL_FILTER_SMOOTH_DELTA,
            "Spatial smooth delta"
        ),
        (
            rs.option.holes_fill,
            SPATIAL_FILTER_HOLES_FILL,
            "Spatial holes fill"
        )
    ]

    for option_name, value, label in settings:
        try:
            if spatial_filter.supports(option_name):
                spatial_filter.set_option(
                    option_name,
                    float(value)
                )
                print(f"[RealSense] {label}: {value}")
        except Exception as error:
            print(f"[RealSense] {label} 설정 실패: {error}")

# =========================================================
# Main
# =========================================================
def main():
    global CAM_FX, CAM_FY, CAM_PPX, CAM_PPY
    global current_object_id, one_shot_locked, locked_cx, locked_cy, locked_z_m, locked_yaw
    global last_object_seen_time, last_valid_cx, last_valid_cy, last_valid_yaw
    global current_pose_mode, current_pose_solidity, current_pose_fill_ratio

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    profile = pipeline.start(config)

    depth_sensor = configure_realsense_depth_sensor(profile)

    align = rs.align(rs.stream.color)

    # query_sensors()로 얻은 일반 sensor 객체에는 get_depth_scale()이 없을 수 있다.
    # Depth Scale은 항상 first_depth_sensor()에서 가져온다.
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_profile.get_intrinsics()
    CAM_FX = float(intr.fx)
    CAM_FY = float(intr.fy)
    CAM_PPX = float(intr.ppx)
    CAM_PPY = float(intr.ppy)

    print("====================================")
    print("고정 ArUco 원점 좌표계 사용")
    print("FIXED_ARUCO_RVEC:", FIXED_ARUCO_RVEC)
    print("FIXED_ARUCO_TVEC:", FIXED_ARUCO_TVEC)
    print("Camera fx/fy/ppx/ppy:", CAM_FX, CAM_FY, CAM_PPX, CAM_PPY)
    print("====================================")

    start_isaac_sender()

    spatial_filter = rs.spatial_filter() if USE_SPATIAL_FILTER else None
    temporal_filter = rs.temporal_filter() if USE_TEMPORAL_FILTER else None
    hole_filter = rs.hole_filling_filter() if USE_HOLE_FILLING_FILTER else None

    configure_spatial_filter(spatial_filter)

    background_depth = None
    background_color = None

    background_capture_active = False
    background_depth_samples = []
    background_color_samples = []

    program_start_time = time.time()
    auto_background_done = False

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            if spatial_filter is not None:
                depth_frame = spatial_filter.process(depth_frame)
            if temporal_filter is not None:
                depth_frame = temporal_filter.process(depth_frame)
            if hole_filter is not None:
                depth_frame = hole_filter.process(depth_frame)

            depth_frame = depth_frame.as_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            display = color_image.copy()
            h, w = color_image.shape[:2]

            if USE_ROI:
                x1, y1, x2, y2 = ROI_X1, ROI_Y1, ROI_X2, ROI_Y2
            else:
                x1, y1, x2, y2 = 0, 0, w, h

            x1 = max(0, min(w - 1, x1))
            x2 = max(0, min(w, x2))
            y1 = max(0, min(h - 1, y1))
            y2 = max(0, min(h, y2))

            roi_depth = depth_image[y1:y2, x1:x2]
            roi_color = color_image[y1:y2, x1:x2]
            roi_w = x2 - x1
            roi_h = y2 - y1
            display_mask = np.zeros(roi_depth.shape, dtype=np.uint8)

            # 자동 배경 캡처 시작
            if (
                AUTO_BACKGROUND_ENABLE
                and not auto_background_done
                and not background_capture_active
            ):
                elapsed = time.time() - program_start_time
                if elapsed >= AUTO_BACKGROUND_DELAY_SEC:
                    background_capture_active = True
                    background_depth_samples.clear()
                    background_color_samples.clear()
                    print(
                        f"자동 배경 캡처 시작: "
                        f"{BACKGROUND_CAPTURE_FRAMES}프레임 동안 ROI를 비워두세요."
                    )

            # 여러 프레임을 모아 안정적인 배경 생성
            if background_capture_active:
                background_depth_samples.append(roi_depth.copy())
                background_color_samples.append(roi_color.copy())

                capture_count = len(background_depth_samples)

                cv2.putText(
                    display,
                    f"Background capture: {capture_count}/{BACKGROUND_CAPTURE_FRAMES}",
                    (20, 165),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 255),
                    2
                )

                if capture_count >= BACKGROUND_CAPTURE_FRAMES:
                    (
                        new_background_depth,
                        valid_ratio_before,
                        valid_ratio_after
                    ) = build_robust_background_depth(
                        background_depth_samples
                    )
                    new_background_color = build_robust_background_color(
                        background_color_samples
                    )

                    if (
                        new_background_depth is not None
                        and new_background_color is not None
                    ):
                        background_depth = new_background_depth
                        background_color = new_background_color

                        reset_tracking_state()
                        last_object_seen_time = time.time()
                        auto_background_done = True

                        print(
                            "배경 저장 완료 | "
                            f"원본 유효 Depth={valid_ratio_before * 100.0:.1f}% | "
                            f"보정 후={valid_ratio_after * 100.0:.1f}%"
                        )
                    else:
                        background_depth = None
                        background_color = None
                        auto_background_done = False
                        program_start_time = time.time()
                        print("배경 저장 실패: ROI를 비우고 다시 시도합니다.")

                    background_capture_active = False
                    background_depth_samples.clear()
                    background_color_samples.clear()

            if background_depth is not None and not background_capture_active:
                diff = (background_depth.astype(np.float32) - roi_depth.astype(np.float32)) * depth_scale
                bg_valid = background_depth > BACKGROUND_VALID_MIN
                cur_valid = roi_depth > 0

                depth_mask = np.zeros(roi_depth.shape, dtype=np.uint8)
                depth_mask[
                    (diff > MIN_DEPTH_DIFF)
                    & (diff < MAX_DEPTH_DIFF)
                    & bg_valid
                    & cur_valid
                ] = 255

                # 0-depth hole mask는 외곽 바깥까지 번질 수 있으므로 기본적으로 사용하지 않는다.
                if USE_DEPTH_HOLE_MASK:
                    hole_mask = build_safe_hole_mask(depth_mask, bg_valid, cur_valid)
                    depth_mask = cv2.bitwise_or(depth_mask, hole_mask)

                color_mask = None
                if USE_COLOR_BACKGROUND_MASK and background_color is not None:
                    color_mask = build_color_change_mask(roi_color, background_color)

                mask = combine_depth_and_color_mask(depth_mask, color_mask)
                mask = clean_depth_mask(mask)
                display_mask = mask.copy()

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                best_cnt = choose_best_contour(contours, roi_w, roi_h)

                if best_cnt is not None:
                    pose_rect_local, pose_mask = build_pose_rect_from_contour(best_cnt, mask.shape)
                    display_mask = pose_mask.copy()

                    if pose_rect_local is not None:
                        local_box = np.int32(np.round(cv2.boxPoints(pose_rect_local)))
                        cv2.polylines(display_mask, [local_box], True, 180, 2)

                        (lcx, lcy), (rw, rh), rect_angle = pose_rect_local
                        cx = float(lcx) + x1
                        cy = float(lcy) + y1
                        yaw_raw = normalize_yaw(rect_angle, rw, rh)

                        current_short = min(rw, rh)
                        current_long = max(rw, rh)

                        (
                            stable_short,
                            stable_long,
                            stable_yaw_raw
                        ) = update_geometry_lock(
                            current_short,
                            current_long,
                            yaw_raw
                        )

                        short_hist.append(stable_short)
                        long_hist.append(stable_long)

                        # 형상이 잠긴 뒤에는 각도도 잠금값을 사용한다.
                        yaw_raw = stable_yaw_raw

                        z_raw_m = patch_median_depth_m(depth_image, depth_scale, cx, cy, r=18, min_valid=8)
                        if z_raw_m <= 0:
                            z_raw_m = masked_median_depth_m(roi_depth, pose_mask, depth_scale)

                        cx_s, cy_s, yaw_s, z_s = update_smoothed_detection(cx, cy, yaw_raw, z_raw_m)

                        yaw_hist.append(yaw_s)
                        cx_hist.append(cx_s)
                        cy_hist.append(cy_s)

                        target_deg = yaw_to_isaac_yaw_deg(yaw_s)

                        robot_result = pixel_depth_to_robot_xy_mm(cx_s, cy_s, z_s)
                        if robot_result is not None:
                            robot_x_mm, robot_y_mm, marker_z_mm = robot_result
                        else:
                            robot_x_mm, robot_y_mm, marker_z_mm = None, None, None

                        last_object_seen_time = time.time()
                        last_valid_cx = lcx
                        last_valid_cy = lcy
                        last_valid_yaw = yaw_s

                        # 표시용 박스
                        median_short = float(np.median(short_hist))
                        median_long = float(np.median(long_hist))
                        box = get_smoothed_display_box(cx_s, cy_s, yaw_s, median_short, median_long)

                        cv2.drawContours(display, [box], 0, (0, 255, 0), 2)
                        cv2.circle(display, (cx_s, cy_s), 5, (0, 0, 255), -1)

                        # 화면 글씨: 각도와 좌표만 표시
                        cv2.putText(display, f"Yaw: {yaw_s:.2f} deg", (20, 45),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                        cv2.putText(display, f"Center(px): ({cx_s}, {cy_s})", (20, 85),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                        if robot_x_mm is not None and robot_y_mm is not None:
                            cv2.putText(display, f"Robot XY(mm): X={robot_x_mm:.1f}, Y={robot_y_mm:.1f}",
                                        (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2)

                        send_mode_text = "Send mode: MOVING" if ALLOW_MOVING_OBJECT_SEND else "Send mode: STOP"
                        cv2.putText(display, send_mode_text, (20, 160),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

                        pose_color = (
                            (0, 0, 255)
                            if current_pose_mode == "DEFECT"
                            else (0, 255, 0)
                        )
                        cv2.putText(
                            display,
                            (
                                f"Pose: {current_pose_mode} "
                                f"S={current_pose_solidity:.3f} "
                                f"F={current_pose_fill_ratio:.3f}"
                            ),
                            (20, 195),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            pose_color,
                            2
                        )

                        geometry_text = (
                            "Bounding: LOCKED"
                            if geometry_locked
                            else "Bounding: LEARNING"
                        )
                        cv2.putText(
                            display,
                            geometry_text,
                            (20, 230),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            (255, 255, 0),
                            2
                        )

                        # one-shot Isaac 전송
                        if (
                            not one_shot_locked
                            and is_detection_stable()
                            and z_s > 0
                            and robot_x_mm is not None
                            and robot_y_mm is not None
                            and marker_z_mm is not None
                        ):
                            sent_ok = send_isaac_create_box(
                                current_object_id,
                                robot_x_mm,
                                robot_y_mm,
                                marker_z_mm,
                                target_deg
                            )
                            if sent_ok:
                                one_shot_locked = True
                                locked_cx = cx_s
                                locked_cy = cy_s
                                locked_z_m = z_s
                                locked_yaw = target_deg

                else:
                    if time.time() - last_object_seen_time > OBJECT_LOST_UNLOCK_TIME:
                        if one_shot_locked:
                            current_object_id += 1
                        reset_tracking_state()

            # ROI만 표시
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 255), 2)

            cv2.imshow("RealSense Tracking", display)
            if SHOW_CLEAN_MASK:
                cv2.imshow("Clean Mask", display_mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('b'):
                background_depth = None
                background_color = None

                background_depth_samples.clear()
                background_color_samples.clear()
                background_capture_active = True

                # 수동 캡처가 진행 중일 때 자동 캡처가 중복 시작되지 않게 한다.
                auto_background_done = True

                reset_tracking_state()
                last_object_seen_time = time.time()

                print(
                    f"수동 배경 캡처 시작: "
                    f"{BACKGROUND_CAPTURE_FRAMES}프레임 동안 ROI를 비워두세요."
                )

            elif key == ord('r'):
                background_depth = None
                background_color = None

                background_depth_samples.clear()
                background_color_samples.clear()
                background_capture_active = False

                reset_tracking_state()
                program_start_time = time.time()
                auto_background_done = False

                print(
                    f"배경 초기화: {AUTO_BACKGROUND_DELAY_SEC:.1f}초 뒤 "
                    f"{BACKGROUND_CAPTURE_FRAMES}프레임 자동 캡처"
                )

            elif key == ord('u'):
                if one_shot_locked:
                    current_object_id += 1
                reset_tracking_state()
                print(f"one-shot 잠금 해제 | 다음 Isaac ID={current_object_id}")

            elif key == ord('q') or key == 27:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        stop_isaac_sender()
        print("종료")


if __name__ == "__main__":
    main()