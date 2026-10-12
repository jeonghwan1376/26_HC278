# ============================================================
# v51.21 박스 생성 Pivot 자동 해제 + 정상/불량 로컬 테스트
# - v51.20의 기존 좌표/속도/통신/리젝터/적재/트레이로더 로직 유지
# - spawn_box() 완료 직전에 Stage 선택을 자동 해제하여
#   새 박스 생성 후 Viewport Pivot/Transform Manipulator가 남지 않도록 처리
# - 정상/불량 로컬 테스트 함수에서도 생성 처리 후 선택을 다시 해제
# - 정상: local_test_normal_box()
# - 불량: local_test_defect_box()
# ============================================================

# ============================================================
# v51.20 트레이로더 Upper 시작 충격 방지
# - Timeline 시작 전 Upper Drive 목표를 0으로 초기화
# - Timeline PLAY 시 Upper를 0에서 -167까지 S-Curve로 천천히 이동
# - 시작 직후 기존 -167 목표값으로 순간 이동하는 현상 방지
# - 기존 트레이 공급, Push, 적재, 통신, 리젝터 로직 유지
# ============================================================

# ============================================================
# v51.19 트레이로더 다중 박스/다음 Upper 위치 준비
# - 기존 업로드 코드의 좌표, 속도, 통신, 리젝터, 적재 로직 유지
# - 트레이당 정상 박스 2개 적재 완료 시 다음 트레이 공급 자동 요청
# - Push가 30으로 복귀한 뒤 Upper를 다음 미공급 Tray 위치로 미리 이동
# - 다음 place_slot=1 박스는 고정 Tray2가 아니라 실제 다음/활성 Tray를 감시
# ============================================================

# ============================================================
# v51.17 v51.16 기준 불량 박스 삭제 제거
# - 불량 박스 Z=640 도착 시 리젝터만 동작
# - 리젝터 0도 -> -100도 -> 0도 복귀
# - 불량 박스 DeletePrims 실행 안 함
# - 불량 박스는 Stage에 그대로 유지
# - 트레이/로봇/컨베이어 설정 유지
# ============================================================

# ============================================================
# v51.16 v51.15 기준 리젝터 미작동 수정
# - 원인: 불량 박스를 Z=640에서 먼저 삭제하여 Z=611 리젝터 감시 전에 사라짐
# - 수정: Z=640에서 리젝터를 먼저 -100도까지 동작
# - -100도 도착 확인 후 불량 박스 삭제
# - 이후 리젝터 0도 복귀
# - v51.15 트레이 적재 좌표/트리거 설정 유지
# ============================================================

# ============================================================
# v51.15 업로드된 코드 기준 트레이 적재 트리거 순서 수정
# - 트레이 진행 방향: 큰 X -> 작은 X
# - 1번 트레이 선택: X=4
# - 1번 실제 대각선 적재 시작: X=-30
# - 1번 X1/X2: 60 -> -19
# - 2번 X1/X2: -200 -> -264, 트레이 좌표 대기 없음
# - 그 외 로봇/컨베이어/불량 처리 설정은 변경하지 않음
# ============================================================

# ============================================================
# v51.12 불량 판정 즉시 빨간색 + Z=684 도착 시 박스 제거
# - 웹캠 INSPECTION_RESULT value=0 수신 즉시 빨간색 적용
# - 불량 박스 World Z=684 도착/통과 시 안전 비동기 삭제
# - 정상 박스의 픽업/적재 및 기존 속도/좌표는 변경하지 않음
# ============================================================

# ============================================================
# v51.11 현재 업로드 파일 기준 흡착 직후 Y 상승 속도 분리
# - 현재 좌표/원점/컨베이어/적재 설정 유지
# - X1/X2 운반 속도 5.80 유지
# - Z 운반 속도 8.80 유지
# - 일반 운반 Y 속도 3.80
# - 흡착 직후 LIFT_Y 단계만 Y 속도 4.20
# ============================================================

# ============================================================
# v51.8 흡착 후 적재 속도 1차 미세 상향
# - 흡착 후 운반 속도 약 10% 증가
# - 대각선 적재 S-Curve 90프레임 -> 84프레임
# - 픽업 전 이동, 트레이 좌표, 컨베이어 속도는 변경하지 않음
# ============================================================

# v51.7: Tray2 Local X=239 도착 전 첫 번째 박스 로봇 이동 전체 보류
# ============================================================
# v51.1 FIRST DEPTH CREATE_BOX ONLY + TRAY LOADER RESTORED
# - 기준: v51 성공 코드의 좌표/속도/공정 순서 유지
# - 박스 생성은 별도 첫 번째 Depth가 보내는 CREATE_BOX에서만 수행
# - MASTER_EVENT/DEPTH/VISION_PACKET은 좌표/각도 상태만 저장하고 박스 생성 금지
# - INSPECTION_RESULT는 이미 생성된 Depth 박스에 FIFO로 정상/불량 판정만 연결
# - 대기 중인 Depth 박스가 없으면 YOLO 결과로 새 박스를 생성하지 않음
# - 트레이로더: Tray2=-135, Tray3=-167, Tray4=-200, Tray5=-220
# - 트레이로더 Push: -190 전진 후 30 복귀, Upper는 공급 위치 유지
# ============================================================

# ============================================================
# v51.7 첫 번째 생성 박스 Z=500 트레이로더 시작
# - 첫 번째 Depth CREATE_BOX로 생성된 최초 박스만 트레이로더 시작 기준으로 사용
# - 해당 박스가 World Z=500에 도착/통과하면 첫 트레이 Upper 이동부터 시작
# - Upper 위치 도달 후 기존 순서대로 Push -190 전진 -> 30 복귀
# - Master 정상 판정 및 STM LBUSY에 의한 조기 트레이로더 시작은 비활성화
# - 기존 픽업/적재/리젝터/컨베이어 설정은 변경하지 않음
# ============================================================

# ============================================================
# v51.5 DEPTH(1).py + MASTER(1).py 호환 DIGITAL TWIN 통합본
# 기준본: 사용자가 실제 동작 확인한 성공 코드의 좌표/속도/순서 유지
#
# 입력 포트 9999
# - master(1).py: MASTER_EVENT, INSPECTION_RESULT
# - depth(1).py는 Isaac에 직접 연결하지 않고 master(1).py로 G 패킷 전송
# - MASTER_EVENT / DEPTH / VISION_PACKET은 상태 저장만 수행하고 박스 생성 금지
# - INSPECTION_RESULT는 기존 첫 번째 Depth 생성 박스에 정상/불량 판정만 연결
#
# 트윈 포트 10000 (선택)
# - Isaac -> Master/GUI: TWIN_STATE 10Hz 송신
# - 터미널에는 Master가 전달한 STM32/MCU 정보만 출력
# - Master/GUI -> Isaac: TWIN_COMMAND, MASTER_EVENT, CREATE_BOX,
#   INSPECTION_RESULT 수신
# - STM 좌표는 자동 공정이 IDLE일 때 3축에 반영 가능
# - Master 컨베이어 상태를 Isaac 검사/포장 컨베이어에 반영
# - 리젝터/트레이로더/3축/그리퍼를 명령으로 직접 제어 가능
#
# 중요: 아래 기존 공정 좌표/속도/순서는 수정하지 않았다.
# ============================================================

# ============================================================
# v47.0: 2박스 좌표 트리거 적재 + 트레이로더 Box Z=610 시작
# - 1번 박스: Tray X=129에서 준비, X1/X2=110·Z=-250, Tray X=4에서 X=80·Y=-50 동기 S-Curve 적재
# - 2번 박스: 동일 Tray를 X1/X2=-50·Z=-250으로 준비, Tray X=-269에서 X=-60·Y=-50 동기 S-Curve 적재
# - 픽업 후 Y=50 완료 시 적재 순번에 따라 Z=-270과 X1/X2를 동시에 동기 S-Curve 이동
#   · 1번 적재: Z=-270 + X1/X2=110
#   · 2번 적재: Z=-270 + X1/X2=-50
# - X1/X2 Joint 허용 범위: -500 ~ 200
# - 생성 박스 World Z=610 도달/통과 시 트레이로더 1회 시작
# v46.8: v46.7 전체 공정 + MasterBridge 전용 TCP 서버/STM 이벤트 표시
# - v46의 박스 생성, S-Curve 그리퍼 이동, Hard-Lock 흡착을 그대로 사용
# - 정상 박스가 픽업 위치에 도착하면 검사 컨베이어 즉시 정지
# - 현재 박스 Hard-Lock 흡착 성공 프레임에 검사 컨베이어를 -65로 즉시 재가동
# - 현재 박스를 트레이에 적재하는 동안 다음 박스를 픽업 위치까지 선이송
# - 다음 정상 박스가 픽업 위치에 도착하면 로봇 동작 중이어도 컨베이어 정지
# - 픽업 후 Y축이 50에 도착하면 Z축을 -270으로 먼저 이동
# - Z축 -270 도착 이후에만 그리퍼 0도 복귀 및 트레이 적재 단계 진입
# - 현재 박스 적재 후 HOME 복귀 즉시 대기 중인 정상 박스를 자동 픽업
# - 불량 박스는 픽업 대기 대상에서 제외하고 기존 리젝터 공정을 그대로 사용
# - 흡착 직후 컨베이어는 다음 정상 박스가 정지선에 도달할 때까지 매 프레임 -65 강제 유지
# - 컨베이어 재가동 실패가 Y/Z 이동을 중단시키지 않도록 공정을 분리
# - 픽업 후 Y=50, 이후 Z=-270 순서를 별도 강제 파이프라인으로 관리
# - 모션 완료 콜백이 누락되어도 다음 프레임 watchdog이 자동 재시도
# - 흡착 후 검사 컨베이어를 1.5초 정지한 뒤 재가동하여 박스 접촉 방지
# - Y=50 이후 Z=-270 이동 순서를 강제 파이프라인으로 관리
# - Y축 Drive가 50에 도착한 직후 Z축 Drive를 -270으로 반드시 이동
# - Z축 Drive -270 이동 완료 후에만 트레이 적재 감시 시작
# - 첫 박스가 HOME 복귀하기 전에는 다음 박스 PICK_XZ 시작을 하드 차단
# - 다음 박스는 픽업 위치에서 정지 대기하고 HOME 완료 후에만 픽업
# - MasterBridge의 IsaacSimClient JSON Lines 수신
# - MASTER_EVENT/INSPECTION_RESULT를 동일한 9999 서버에서 처리
# - 수신 즉시 JSON ACK 회신
# - Master TCP 연결과 STM32 정보만 터미널에 표시
# - 1차 박스 적재 중 2차 박스를 픽업 정지 위치까지 선이송
# - 트레이 흡착 해제 유지 프레임과 안착 대기를 단축
# - 흡착 중 박스가 그리퍼 회전을 계속 따라가도록 유지
# - 생성 박스 World Z=610 도달/통과 후 트레이로더 자동 시작
# - 트레이로더가 공급한 Tray2~Tray6을 로봇의 실제 적재 추종 대상으로 자동 전환
# - STOP 시 Upper 유지, Push=30, 트레이 Collider=Convex Hull 복구
# - 터미널에는 TCP, 박스 생성, 리젝터, 정지 위치, 그리퍼 이동, 트레이 이동, 흡착/해제만 출력
# ============================================================

# ============================================================
# TCP 좌표 기반 박스 생성 + Z 센서 정지 + 동적 그리퍼 추종 통합본
# - RealSense/OpenCV가 전송한 CREATE_BOX JSON의 position XYZ 수신
# - fixed_aruco_center 좌표를 Isaac Sim 월드 좌표로 변환
# - 인식된 X 위치에 맞춰 박스를 가상 컨베이어에 생성
# - 박스 X가 달라도 Z=57.68 센서선 통과 시 동일하게 정지
# - 정지 순간 박스 실제 XYZ를 저장하고 그리퍼가 해당 위치로 이동
# - 수신 yaw를 박스와 실제 RevoluteJoint_gripper에 함께 반영
# - Depth에서 먼저 생성된 박스에 검사 코드의 INSPECTION_RESULT를 연결
# - 불량 판정 박스는 판정 즉시 빨간색으로 변경
# ============================================================

# ============================================================
# 통합본: YOLO TCP 판정 박스 생성 + 원본 성공 코드 + 포장 컨베이어 3번 재출발 코드
# - 원본 성공 코드: 검사 컨베이어, 박스 생성, 리젝터, 3축 자동 픽업/배치
# - 추가 코드: 포장 컨베이어 / Tray 자동 정지, 3번 키 재출발, 0번 키 정지
# ============================================================

# 0526 성공코드 + YOLO 판정 기반 지연 리젝터
# + 자동 픽업/배치/복귀/컨베이어 재가동 통합본
# 최종 수정:
# - 자동 배치 위치를 박스 순번에 따라 변경
# - 1번째 박스: X1/X2 160, Z -130
# - 2번째 박스: X1/X2 85, Z -130
# - 이후 160 ↔ 85 반복

import omni.kit.commands
import carb.input
import omni.appwindow
import omni.timeline
import omni.kit.app
import omni.usd
import random
import asyncio
import time
import socket
import threading
import queue
import json
import builtins
import math
import re
import traceback
from collections import deque
from pxr import Gf, Usd, UsdPhysics, UsdGeom, UsdShade, Sdf, Vt, PhysxSchema

# ==================================================
# [터미널 출력 필터 - STM/MCU 수신 정보만 출력]
# ==================================================
_SYSTEM_PRINT = builtins.print

# 이 코드에서 직접 발생하는 Isaac Sim 내부 공정 로그는 숨기고,
# Master에서 전달된 STM32/MCU 관련 정보만 터미널에 표시한다.
_ALLOWED_EVENT_PREFIXES = (
    "[STM 수신]",
    "[STM 상태]",
    "[STM 응답]",
    "[STM 센서]",
    "[STM 좌표]",
    "[STM 명령]",
    "[STM YOLO]",
    "[STM 연결]",
    "[STM 기타]",
)


_SUPPRESSED_REALTIME_LOG_TOKENS = (
    "실시간 추종 중",
    "실제 중심 추종 중",
    "현재 World X=",
    "현재 Tray X=",
)


def event_log(message):
    """허용된 통신 및 공정 로그를 Isaac Sim 터미널에 출력한다."""
    message_text = str(message).strip()

    if any(
        token in message_text
        for token in _SUPPRESSED_REALTIME_LOG_TOKENS
    ):
        return

    if message_text.startswith(
        _ALLOWED_EVENT_PREFIXES
    ):
        timestamp_text = time.strftime("%H:%M:%S")
        _SYSTEM_PRINT(
            f"[{timestamp_text}] {message_text}",
            flush=True,
        )


def print(*args, **kwargs):
    """이 통합 코드 내부 일반 출력은 숨긴다. STM 로그는 _SYSTEM_PRINT로만 표시한다."""
    return None


def _terminal_compact_value(value, max_length=260):
    """통신 payload를 한 줄로 읽기 좋게 줄인다."""
    try:
        if isinstance(value, (dict, list, tuple)):
            text_value = json.dumps(
                value,
                ensure_ascii=False,
                separators=(",", ":"),
                default=str,
            )
        else:
            text_value = str(value)
    except Exception:
        text_value = str(value)

    text_value = text_value.replace("\r", "\\r").replace("\n", "\\n")

    if len(text_value) > int(max_length):
        return text_value[:int(max_length)] + "..."

    return text_value


def _terminal_message_summary(msg):
    """9999 포트에서 받은 JSON의 핵심 필드를 요약한다."""
    if not isinstance(msg, dict):
        return _terminal_compact_value(msg)

    cmd = str(msg.get("cmd", "")).strip().upper()

    if cmd == "MASTER_EVENT":
        category = str(msg.get("category", "")).strip().upper()
        event_name = str(msg.get("event", "")).strip().upper()
        data = msg.get("data", {})
        return (
            f"cmd=MASTER_EVENT | category={category} | "
            f"event={event_name} | data={_terminal_compact_value(data)}"
        )

    if cmd == "INSPECTION_RESULT":
        return (
            f"cmd=INSPECTION_RESULT | track_id={msg.get('track_id')} | "
            f"value={msg.get('value')} | label={msg.get('label')} | "
            f"source={msg.get('source')}"
        )

    if cmd in ("CREATE_BOX", "DECISION"):
        return (
            f"cmd={cmd} | id={msg.get('id', msg.get('track_id'))} | "
            f"result={msg.get('result')} | "
            f"position={_terminal_compact_value(msg.get('position'))} | "
            f"yaw={extract_received_yaw(msg) if 'extract_received_yaw' in globals() else msg.get('angle')}"
        )

    return f"cmd={cmd} | payload={_terminal_compact_value(msg)}"


def log_master_communication_event(category, event_name, data, raw=None):
    """Master에서 받은 이벤트를 종류별로 사람이 읽기 쉽게 출력한다."""
    category = str(category).strip().upper()
    event_name = str(event_name).strip().upper()

    if category == "DEPTH" and event_name == "VISION_PACKET":
        event_log(
            f"[통신 RX] DEPTH/VISION_PACKET | "
            f"count={data.get('count')} | "
            f"X={data.get('x')} | Y={data.get('y')} | Z={data.get('z')} | "
            f"angle={data.get('angle')}"
        )
        return

    if category == "DEPTH" and event_name == "ANGLE_RECEIVED":
        event_log(
            f"[통신 RX] DEPTH/ANGLE_RECEIVED | angle={data.get('angle')}"
        )
        return

    if category == "YOLO" and event_name == "RESULT":
        event_log(
            f"[통신 RX] YOLO/RESULT | track_id={data.get('track_id')} | "
            f"result={data.get('result')} | label={data.get('label')}"
        )
        return

    if category == "CONVEYOR" and event_name == "STATE":
        event_log(
            f"[통신 RX] CONVEYOR/STATE | "
            f"inspection={data.get('inspection_running')} | "
            f"packaging={data.get('packaging_running')}"
        )
        return

    if category == "OPENRB" and event_name == "TX_ANGLE":
        event_log(
            f"[통신 RX] OPENRB/TX_ANGLE | angle={data.get('angle')} | "
            f"success={data.get('success')}"
        )
        return

    if category in ("CONNECTION", "SYSTEM", "BLACKBOX"):
        event_log(
            f"[통신 RX] {category}/{event_name} | "
            f"data={_terminal_compact_value(data)}"
        )
        return

    # MCU/STM은 아래 전용 함수가 더 자세히 출력하므로 중복 요약을 생략한다.


def print_integration_algorithm():
    """Isaac Sim 터미널에 현재 연동 구조와 자동 공정 알고리즘을 출력한다."""
    local_addresses = get_local_ipv4_addresses() if 'get_local_ipv4_addresses' in globals() else []
    ip_text = ", ".join(local_addresses) if local_addresses else "확인 불가"

    event_log("[시스템 정보] ================================================")
    event_log("[시스템 정보] Isaac Sim 통합 디지털 트윈 연동 준비")
    event_log(f"[시스템 정보] Isaac 수신 서버 = {YOLO_TCP_HOST}:{YOLO_TCP_PORT}")
    event_log(f"[시스템 정보] Isaac PC IPv4 후보 = {ip_text}")
    event_log(f"[시스템 정보] Twin 상태/명령 서버 = {TWIN_TCP_HOST}:{TWIN_TCP_PORT}")
    event_log("[시스템 정보] 입력 프로토콜 = MASTER_EVENT, INSPECTION_RESULT, CREATE_BOX")
    event_log("[공정 알고리즘] 통신: depth.py -> master.py:5000 -> Isaac Sim:9999")
    event_log("[공정 알고리즘] 생성: 첫 번째 Depth CREATE_BOX -> 인식 위치에 박스 생성")
    event_log("[공정 알고리즘] 판정: INSPECTION_RESULT -> 기존 생성 박스에 정상/불량 FIFO 연결")
    event_log("[공정 알고리즘] Master VISION_PACKET -> 좌표/각도 상태 저장만 수행")
    event_log("[공정 알고리즘] 정상: X/Z 이동 -> 그리퍼 각도 맞춤 -> Y 하강 -> Hard-Lock 흡착")
    event_log("[공정 알고리즘] 정상: Y=50 상승 -> 적재순번 X/Z 이동 -> 트레이 적재 -> HOME")
    event_log("[공정 알고리즘] 불량: INSPECTION_RESULT value=0 -> 빨간 박스 -> 리젝터 -100도 왕복")
    event_log("[공정 알고리즘] 트레이로더: 첫 번째 생성 박스 Z=500 도착 -> 첫 트레이 공급")
    event_log("[공정 알고리즘] 1번 적재 후 다음 제품은 2번 적재 좌표로 자동 전환")
    event_log("[시스템 정보] ================================================")


def debug_print(*args, **kwargs):
    # 기존 디버그 출력 전체 숨김
    return None


def result_to_text(result):
    try:
        result_value = int(result)
    except Exception:
        return "미판정"

    if result_value == 0:
        return "불량"
    if result_value == 1:
        return "정상"
    return "미판정"

# ==================================================
# [1. 경로 설정]
# ==================================================

ORIGINAL_BOX_PATH_PREFIX = "/assembly714/Box"
ASSEMBLY_ROOT_PATH = "/assembly714"

CONVEYOR_PATH = "/assembly714/Check_conveyor/check_conveyor"

REJECTOR_JOINT_PATH = "/assembly714/Rejector/RevoluteJoint_joint1"

# 사용자가 Y_station과 Gripper 사이에 직접 연결한 실제 회전 조인트
GRIPPER_REVOLUTE_JOINT_PATH = (
    "/assembly714/RevoluteJoint_gripper"
)

PRISMATIC_JOINT_X1_PATH = "/assembly714/PrismaticJoint_X1_station"
PRISMATIC_JOINT_X2_PATH = "/assembly714/PrismaticJoint_X2_station"
PRISMATIC_JOINT_Y_PATH = "/assembly714/PrismaticJoint_Y_station"
PRISMATIC_JOINT_Z_PATH = "/assembly714/PrismaticJoint_Z_station"

PRISMATIC_JOINT_X_PATH = PRISMATIC_JOINT_X1_PATH

# 실제 Stage 구조에서 확인된 흡착부 Prim.
#
# Stage:
# /assembly714/Gripper
# └── gripper_surface
#     ├── Body1
#     ├── SurfaceGripper
#     └── D6Joint
#
# 존재하지 않는 /assembly714/Gripper/Gripper 경로는 사용하지 않는다.
GRIPPER_REFERENCE_PATH = (
    "/assembly714/Gripper/gripper_surface"
)

# 우선순위대로 실제 존재하는 Prim을 자동 선택한다.
# 첫 번째 gripper_surface Xform을 기본 흡착 기준으로 사용하고,
# 필요할 경우 그 자식 Body1까지 확인한다.
SUCTION_CENTER_PATH_CANDIDATES = [
    "/assembly714/Gripper/gripper_surface",
    "/assembly714/Gripper/gripper_surface/Body1",
]

SUCTION_CENTER_PATH = GRIPPER_REFERENCE_PATH


TARGET_ATTR_NAME = "drive:linear:physics:targetPosition"
VELOCITY_ATTR_NAME = "drive:linear:physics:targetVelocity"


# ==================================================
# [2. 컨베이어 Surface Velocity 속성 후보]
# ==================================================

CONVEYOR_ATTR_CANDIDATES = [
    "physxConveyor:surfaceVelocity",
    "physxSurfaceVelocity:surfaceVelocity",
    "physxSurfaceVelocity:surfaceLinearVelocity"
]

CONVEYOR_ENABLE_ATTR_CANDIDATES = [
    "physxConveyor:surfaceVelocityEnabled",
    "physxSurfaceVelocity:surfaceVelocityEnabled"
]

CONVEYOR_LOCAL_SPACE_ATTR_CANDIDATES = [
    "physxConveyor:surfaceVelocityLocalSpace",
    "physxSurfaceVelocity:surfaceVelocityLocalSpace"
]


# ==================================================
# [3. 설정 수치]
# ==================================================

BOX_SPAWN_POS = Gf.Vec3d(-107.29719, 28.99977, 896.77776)
# Isaac Sim 기본 Cube의 size는 2이므로,

BOX_TARGET_SCALE = Gf.Vec3d(34.0, 20.0, 53.0)
BOX_COLOR_RGB = Gf.Vec3f(0.7, 0.7, 0.7)

# ==================================================
# [RealSense/OpenCV 수신 좌표 -> Isaac 월드 좌표]
# ==================================================
# 송신 JSON 형식:
# {
#   "cmd": "CREATE_BOX",
#   "position": {"x": ..., "y": ..., "z": ...},
#   "position_unit": "mm",
#   "coordinate_system": "depth_xy"
# }
#
# Depth Camera에서 전달할 박스 위치:
#   x = 영상/Depth 기준 가로 위치
#   y = 영상/Depth 기준 진행 방향 위치
#   z = 깊이값 또는 참고값이며 Isaac 스폰 좌표에는 사용하지 않음
#
# Isaac Sim 장면:
#   X = 컨베이어 폭 방향
#   Y = 높이
#   Z = 컨베이어 진행 방향
#
# v17 좌표 매핑:
#   Depth position.x -> Isaac X
#   Depth position.y -> Isaac Z
#   Isaac Y          -> BOX_SPAWN_POS[1] 고정
#
# 즉, 웹캠 송신값 X, Y, Z 중:
#   X와 Y만 박스 생성 위치에 사용하고,
#   Depth Z는 로그용으로만 보관한다.
USE_RECEIVED_TCP_SPAWN_POSITION = True

# TCP 좌표 (0, 0, 0)이 Isaac에서 대응하는 기준 월드 좌표
# 현재 수동 B키의 기존 성공 스폰 위치를 ArUco 원점 대응점으로 사용
ISAAC_ARUCO_ORIGIN_WORLD = Gf.Vec3d(
    BOX_SPAWN_POS[0],
    BOX_SPAWN_POS[1],
    BOX_SPAWN_POS[2]
)

# 수신 좌표 기준점. ArUco 중심이 0,0,0이므로 기본값 0
TCP_REFERENCE_X_MM = 0.0
TCP_REFERENCE_Y_MM = 0.0
TCP_REFERENCE_Z_MM = 0.0

# 축 방향이 반대로 나오면 해당 SIGN만 -1.0으로 변경
TCP_X_TO_ISAAC_X_SIGN = 1.0
TCP_Y_TO_ISAAC_Z_SIGN = 1.0

# Isaac 장면과 TCP가 모두 mm 단위라면 1.0
TCP_TO_ISAAC_SCALE = 1.0

# 잘못된 좌표로 장면 밖에 생성되는 것을 방지하는 제한값
RECEIVED_SPAWN_X_MIN = -300.0
RECEIVED_SPAWN_X_MAX = 100.0
RECEIVED_SPAWN_Y_MIN = 20.0
RECEIVED_SPAWN_Y_MAX = 80.0
RECEIVED_SPAWN_Z_MIN = 896.0
RECEIVED_SPAWN_Z_MAX = 896.0

# YOLO 판정 결과별 박스 색상
# 정상(result=1)과 수동/미판정 박스는 기존 BOX_COLOR_RGB를 그대로 사용한다.
# 불량(result=0) 박스에만 빨간색 재질을 적용한다.
DEFECT_BOX_COLOR_RGB = Gf.Vec3f(1.0, 0.05, 0.02)   # 불량: 빨간색

# ==================================================
# [검사 결과 연동 및 불량 색상 변경 설정]
# ==================================================
# 불량(result=0) 판정이 들어오는 즉시 빨간색으로 변경한다.
# 위치 조건은 사용하지 않는다.
DEFECT_RED_IMMEDIATE = True

# v51.12:
# 웹캠에서 불량 판정이 들어온 박스는 즉시 빨간색으로 표시하고,
# 검사 컨베이어 진행 중 World Z가 이 좌표에 도착하거나 통과하면 Stage에서 제거한다.
#
# 첨부 화면의 Transform Translate Z가 +684.83805로 표시되어 있어
# 제거 기준은 +684.0으로 설정한다.
# 불량 박스는 더 이상 Stage에서 삭제하지 않는다.
# Z=640 도착/통과 시 리젝터만 1회 동작한다.
DEFECT_REMOVE_AT_Z_ENABLED = False
DEFECT_REMOVE_TRIGGER_Z = 640.0
DEFECT_REMOVE_TRIGGER_TOLERANCE_Z = 2.0

# 리젝터가 이미 동작 중일 때 기다릴 최대 프레임 수
DEFECT_REMOVE_REJECTOR_WAIT_MAX_FRAMES = 600

# ==================================================
# [추가] OpenCV에서 받은 회전각 적용 설정
# ==================================================
# 현재 장면에서 컨베이어 바닥면이 X-Z 평면이므로
# 영상에서 구한 yaw를 Isaac Sim의 Y축 회전으로 적용한다.
# 장면 축 구성이 다르면 "X" 또는 "Z"로 변경할 수 있다.
BOX_YAW_AXIS = "Y"

# 이 기준 코드에서는 OpenCV/Depth 송신 각도를 그대로 적용했을 때
# Isaac Sim 박스 방향이 정상적으로 일치한다.
INVERT_RECEIVED_YAW = True

# 박스의 기본 긴 변은 Isaac Z축 방향이다.
# 따라서 수신 yaw=0도면 세로, yaw=90도면 가로가 된다.
BOX_YAW_OFFSET_DEG = 0.0

# 0도/90도 부근의 검출 흔들림을 정확한 직각으로 스냅한다.
SNAP_CARDINAL_YAW = True
CARDINAL_YAW_SNAP_TOLERANCE_DEG = 2.0

# Dynamic Rigid Body에서 rotateXYZ가 물리 스텝에 의해 무시되거나
# 기본 회전으로 되돌아가는 현상을 막기 위해 quaternion orient를 사용한다.
USE_PHYSICS_COMPATIBLE_ORIENT = True

# 박스 생성 직후 PhysX 초기화 다음 프레임에 각도를 한 번 더 적용한다.
REAPPLY_YAW_AFTER_PHYSICS = True

# ==================================================
# [실제 Revolute Joint 그리퍼 각도 추종 설정]
# ==================================================
# 구조:
#   Body 0 = /assembly714/Y_station
#   Body 1 = /assembly714/Gripper
#   Joint  = /assembly714/RevoluteJoint_gripper
#   Axis   = Y
#
# 동작 순서:
#   박스 위 이동
#   → 박스 yaw만큼 그리퍼 회전
#   → Y 하강 및 흡착
#   → 상승/적재 위치 이동
#   → 박스를 잡은 상태에서 그리퍼 0도 복귀
#   → Y 하강 및 해제
#
# 조인트 회전 방향이 박스와 반대로 나오면 1.0을 -1.0으로 바꾼다.
GRIPPER_JOINT_DIRECTION_SIGN = 1.0

# Joint의 물리적 0도와 화면상 그리퍼 0도가 다를 때만 보정한다.
GRIPPER_JOINT_ZERO_OFFSET_DEG = 0.0

# 기본 자세
GRIPPER_JOINT_HOME_LOGICAL_DEG = 0.0

# 박스는 180도 대칭이므로 그리퍼는 -90~90도 최단 경로로 회전한다.
# 회전 속도를 낮추고 목표각 유지 프레임을 늘려 흔들림을 줄인다.
GRIPPER_JOINT_MAX_STEP_DEG = 4.00
GRIPPER_JOINT_TOLERANCE_DEG = 0.05
GRIPPER_JOINT_STABLE_FRAMES_REQUIRED = 3

# RevoluteJoint_gripper Angular Drive 강성.
# 기존보다 강한 Stiffness/Damping/MaxForce로 목표각을 단단히 유지한다.
GRIPPER_DRIVE_TYPE = "acceleration"
GRIPPER_DRIVE_STIFFNESS = 30000.0
GRIPPER_DRIVE_DAMPING = 5000.0
GRIPPER_DRIVE_MAX_FORCE = 1000000000.0

# 현재 Stage에 이미 Drive가 있어도 위 값을 강제로 적용한다.
FORCE_GRIPPER_DRIVE_TUNING = True

# ==================================================
# [TCP. 별도 Depth + Master → Isaac Sim 연동 설정]
# - 별도 Depth: CREATE_BOX
# - Master: MASTER_EVENT, INSPECTION_RESULT
# ==================================================

YOLO_TCP_HOST = "0.0.0.0"
YOLO_TCP_PORT = 9999
YOLO_TCP_STATE_NAME = "_YOLO_FINAL_COMBINED_TCP_STATE"

yolo_tcp_queue = queue.Queue()
yolo_tcp_state = {
    "running": True,
    "server_socket": None,
    "server_thread": None,
    "update_sub": None,
    "queue": yolo_tcp_queue
}


# ==================================================
# [depth(1).py + master(1).py 직접 호환 모드]
# ==================================================
# 박스 생성은 별도 첫 번째 Depth가 Isaac으로 직접 보내는 CREATE_BOX만 사용한다.
# Master에 연결된 Depth의 G,x,y,z,angle은 MASTER_EVENT/DEPTH/VISION_PACKET으로
# 전달되지만, 이 패킷은 실물 좌표/각도 상태 저장용이며 새 박스를 생성하지 않는다.
MASTER_BRIDGE_COMPAT_MODE = True
MASTER_VISION_AUTO_SPAWN_ENABLE = False
MASTER_DEFECT_AUTO_SPAWN_ENABLE = False

# 실제 제품은 VISION_PACKET 시점에 픽업 카메라/정지 구간에 있으므로
# 가상 박스도 검사 컨베이어 시작점이 아니라 Z=65 정지선에 생성한다.
MASTER_NORMAL_SPAWN_AT_PICK_LINE = True
MASTER_DEFECT_SPAWN_AT_REJECTOR_LINE = True

# 물리 공정은 검증된 Isaac 자동 상태 머신이 담당한다.
# Master의 컨베이어 표시값과 STM 좌표는 상태로 저장하되,
# 중복 명령으로 기존 성공 공정을 방해하지 않도록 직접 Drive에는 적용하지 않는다.
MASTER_APPLY_CONVEYOR_STATE_TO_PHYSICS = False
MASTER_APPLY_STM_COORDINATE_TO_ROBOT = False

# VISION_PACKET은 불량 제거 후 정상 제품 위치/각도 데이터이므로,
# INSPECTION_RESULT가 아직 도착하지 않은 경우에도 정상으로 생성한다.
MASTER_VISION_DEFAULT_RESULT = 1

# 첫 번째 적재 제품에서 트레이로더를 준비한다.
# STM의 LBUSY 응답도 트레이 공급 시작 신호로 사용한다.
MASTER_START_TRAY_LOADER_ON_FIRST_NORMAL = False
MASTER_START_TRAY_LOADER_ON_LBUSY = False

MASTER_AUTO_BOX_ID_BASE = 800000
MASTER_VISION_DUPLICATE_CACHE_SIZE = 256
MASTER_INSPECTION_DUPLICATE_CACHE_SIZE = 512


# ==================================================
# [DIGITAL TWIN 양방향 상태 동기화]
# ==================================================
# 기존 9999 포트의 성공 통신은 변경하지 않는다.
# 트윈 상태 스트림/명령은 별도 10000 포트를 사용한다.
TWIN_TCP_HOST = "0.0.0.0"
TWIN_TCP_PORT = 10000
TWIN_TCP_STATE_NAME = "_ASSEMBLY714_DIGITAL_TWIN_TCP_STATE_V48"
TWIN_STATE_SEND_HZ = 10.0
TWIN_STATE_MAX_BOXES = 30
TWIN_COMMANDS_PER_UPDATE = 20

# STM 좌표를 Isaac 3축 Joint 목표로 바꾸는 기본 매핑.
# 실물 좌표계와 다르면 SCALE/SIGN/OFFSET만 조정한다.
STM_X_TO_TWIN_X_SCALE = 1.0
STM_X_TO_TWIN_X_SIGN = 1.0
STM_X_TO_TWIN_X_OFFSET = 0.0
STM_Y_TO_TWIN_Y_SCALE = 1.0
STM_Y_TO_TWIN_Y_SIGN = 1.0
STM_Y_TO_TWIN_Y_OFFSET = 0.0
STM_Z_TO_TWIN_Z_SCALE = 1.0
STM_Z_TO_TWIN_Z_SIGN = 1.0
STM_Z_TO_TWIN_Z_OFFSET = 0.0

# 기존 자동 공정을 보호하면서 실물 정보를 동기화하는 기본 모드.
# - master_conveyor_to_twin: Master가 보낸 컨베이어 상태를 Isaac에 반영
# - stm_coordinate_mode: OFF / IDLE_ONLY / ALWAYS
# - explicit_process_events: TWIN/STM 명시 이벤트로 리젝터/트레이로더 제어
# - broadcast_enabled: TWIN_STATE 주기 송신
# - include_boxes: 생성 박스 목록 포함
# - include_master_history_tail: 최근 Master 이벤트 개수

digital_twin_sync_options = {
    "master_conveyor_to_twin": bool(MASTER_APPLY_CONVEYOR_STATE_TO_PHYSICS),
    "stm_coordinate_mode": (
        "IDLE_ONLY"
        if MASTER_APPLY_STM_COORDINATE_TO_ROBOT
        else "OFF"
    ),
    "explicit_process_events": True,
    "broadcast_enabled": True,
    "include_boxes": True,
    "include_master_history_tail": 20,
}

digital_twin_command_queue = queue.Queue()
digital_twin_socket_lock = threading.Lock()
digital_twin_clients_lock = threading.Lock()
digital_twin_sequence_lock = threading.Lock()
digital_twin_sequence = 0

digital_twin_state = {
    "running": True,
    "server_socket": None,
    "server_thread": None,
    "update_sub": None,
    "clients": {},
    "last_broadcast_time": 0.0,
    "last_command": None,
    "last_command_time": None,
    "last_command_result": None,
}

master_tcp_client_lock = threading.Lock()
master_tcp_client_state = {
    "connected_clients": {},
    "last_address": None,
    "last_command": None,
    "last_received_time": None,
    "message_count": 0,
}

# ==================================================
# [Master 전체 상태 수신 저장소]
# ==================================================
# 별도 Depth는 CREATE_BOX를 직접 전송한다.
# Master는 MASTER_EVENT와 INSPECTION_RESULT를 전송한다.
master_state_lock = threading.Lock()
master_event_history = deque(maxlen=500)

master_runtime_state = {
    "last_event": None,
    "last_timestamp": None,

    # STM32
    "mcu_raw": None,
    "robot_step": None,
    "previous_robot_step": None,
    "mcu_response": None,
    "sensor_trigger_count": 0,
    "mcu_connected": None,
    "mcu_port": None,
    "mcu_baud": None,
    "mcu_connection_error": None,
    "mcu_last_stop_command": None,
    "mcu_last_stop_success": None,
    "mcu_last_yolo_result": None,
    "mcu_last_yolo_success": None,

    # Master 연결 Depth
    "depth_x": None,
    "depth_y": None,
    "depth_z": None,
    "depth_angle": None,

    # MCU로 실제 전송한 위치
    "mcu_target_x": None,
    "mcu_target_y": None,
    "mcu_target_z": None,

    # OpenRB
    "openrb_angle": None,

    # 컨베이어
    "inspection_conveyor_running": None,
    "packaging_conveyor_running": None,

    # YOLO
    "yolo_track_id": None,
    "yolo_result": None,
    "yolo_label": None,

    # 연결 상태
    "connections": {},
}


def get_master_runtime_state():
    """
    다른 Isaac Sim 제어 로직에서 사용할 수 있도록
    현재 Master/STM 상태의 복사본을 반환한다.
    """
    with master_state_lock:
        snapshot = dict(master_runtime_state)
        snapshot["connections"] = dict(
            master_runtime_state.get(
                "connections",
                {}
            )
        )

    with master_tcp_client_lock:
        snapshot["master_tcp_connected"] = bool(
            master_tcp_client_state[
                "connected_clients"
            ]
        )
        snapshot["master_tcp_clients"] = list(
            master_tcp_client_state[
                "connected_clients"
            ].keys()
        )
        snapshot["master_tcp_last_address"] = (
            master_tcp_client_state.get(
                "last_address"
            )
        )
        snapshot["master_tcp_message_count"] = (
            master_tcp_client_state.get(
                "message_count",
                0
            )
        )

    return snapshot


def _format_stm_value(value):
    """
    STM 터미널 표시용 값 정리.
    None은 '-'로 표시하고 나머지는 문자열로 변환한다.
    """
    if value is None:
        return "-"

    if isinstance(value, float):
        return f"{value:.3f}"

    return str(value)


def print_stm_terminal_event(
    event_name,
    data,
    raw=None
):
    """
    Master에서 전달된 STM32/MCU 이벤트를 사람이 보기 쉬운 형태로 출력한다.

    지원 이벤트:
    - RX_RAW
    - ROBOT_STATE
    - RESPONSE
    - SENSOR_TRIGGER
    - TX_COORDINATE
    - CONNECTED / DISCONNECTED 등 기타 이벤트
    """
    event_name = str(
        event_name
    ).strip().upper()

    if not isinstance(
        data,
        dict
    ):
        data = {
            "value": data
        }

    if event_name == "RX_RAW":
        raw_value = data.get(
            "value",
            raw
        )

        event_log(
            f"[STM 수신] RAW | "
            f"{_format_stm_value(raw_value)}"
        )
        return

    if event_name == "ROBOT_STATE":
        current_step = data.get(
            "step"
        )
        previous_step = data.get(
            "previous_step"
        )

        event_log(
            f"[STM 상태] "
            f"{_format_stm_value(previous_step)}"
            f" -> "
            f"{_format_stm_value(current_step)}"
        )
        return

    if event_name == "RESPONSE":
        response_value = data.get(
            "response",
            data.get(
                "value",
                raw
            )
        )

        event_log(
            f"[STM 응답] "
            f"{_format_stm_value(response_value)}"
        )
        return

    if event_name == "SENSOR_TRIGGER":
        trigger_count = data.get(
            "count",
            0
        )

        event_log(
            f"[STM 센서] 감지 | "
            f"누적={_format_stm_value(trigger_count)}"
        )
        return

    if event_name == "TX_COORDINATE":
        x_value = data.get("x")
        y_value = data.get("y")
        z_value = data.get("z")
        success_value = data.get("success")

        event_log(
            f"[STM 좌표] 전송 | "
            f"X={_format_stm_value(x_value)} | "
            f"Y={_format_stm_value(y_value)} | "
            f"Z={_format_stm_value(z_value)} | "
            f"성공={_format_stm_value(success_value)}"
        )
        return

    if event_name == "TX_STOP":
        event_log(
            f"[STM 명령] 컨베이어 정지 | "
            f"명령={_format_stm_value(data.get('command', raw))} | "
            f"성공={_format_stm_value(data.get('success'))}"
        )
        return

    if event_name == "TX_YOLO_RESULT":
        event_log(
            f"[STM YOLO] 결과 전송 | "
            f"값={_format_stm_value(data.get('value', raw))} | "
            f"성공={_format_stm_value(data.get('success'))}"
        )
        return

    if event_name in (
        "CONNECTED",
        "CONNECT",
        "SERIAL_CONNECTED",
        "PORT_OPEN",
        "OPEN"
    ):
        port_value = data.get(
            "port",
            data.get(
                "device",
                data.get(
                    "value",
                    raw
                )
            )
        )

        event_log(
            f"[STM 연결] 연결됨 | "
            f"{_format_stm_value(port_value)}"
        )
        return

    if event_name in (
        "DISCONNECTED",
        "DISCONNECT",
        "SERIAL_DISCONNECTED",
        "PORT_CLOSED",
        "CLOSE"
    ):
        reason_value = data.get(
            "reason",
            data.get(
                "value",
                raw
            )
        )

        event_log(
            f"[STM 연결] 연결 해제 | "
            f"{_format_stm_value(reason_value)}"
        )
        return

    # Master에서 새로운 MCU 이벤트 이름을 추가해도 누락하지 않는다.
    generic_value = (
        data
        if data
        else raw
    )

    event_log(
        f"[STM 기타] {event_name} | "
        f"{_format_stm_value(generic_value)}"
    )


def _master_client_key(addr):
    try:
        return f"{str(addr[0])}:{int(addr[1])}"
    except Exception:
        return str(addr)


def mark_master_tcp_client(addr, msg):
    client_key = _master_client_key(addr)

    cmd = str(
        msg.get("cmd", "")
    ).strip().upper()

    source = str(
        msg.get("source", "")
    ).strip().upper()

    is_master_message = (
        cmd == "MASTER_EVENT"
        or (
            cmd == "INSPECTION_RESULT"
            and source in (
                "MASTER",
                "YOLO_VIA_MASTER",
            )
        )
    )

    if not is_master_message:
        return False

    first_connection = False

    with master_tcp_client_lock:
        clients = master_tcp_client_state[
            "connected_clients"
        ]

        if client_key not in clients:
            first_connection = True
            clients[client_key] = {
                "address": client_key,
                "connected_time": time.time(),
                "message_count": 0,
            }

        client_info = clients[client_key]
        client_info["message_count"] += 1
        client_info["last_command"] = cmd
        client_info["last_received_time"] = time.time()

        master_tcp_client_state["last_address"] = client_key
        master_tcp_client_state["last_command"] = cmd
        master_tcp_client_state["last_received_time"] = time.time()
        master_tcp_client_state["message_count"] += 1

    if first_connection:
        event_log(
            f"[MASTER 연결] TCP 연결 확인 | "
            f"{client_key} -> "
            f"{YOLO_TCP_HOST}:{YOLO_TCP_PORT}"
        )
        event_log(
            f"[통신 연결] Master 연동 완료 | peer={client_key} | "
            f"Isaac 수신={YOLO_TCP_HOST}:{YOLO_TCP_PORT} | "
            "프로토콜=MASTER_EVENT/INSPECTION_RESULT"
        )

        # 실제 Master 메시지가 처음 확인된 시점에 알고리즘을 다시 출력한다.
        try:
            print_integration_algorithm()
        except NameError:
            pass

    return True


def unmark_master_tcp_client(addr):
    client_key = _master_client_key(addr)

    with master_tcp_client_lock:
        removed = master_tcp_client_state[
            "connected_clients"
        ].pop(client_key, None)

    if removed is not None:
        event_log(
            f"[MASTER 연결] TCP 연결 종료 | "
            f"{client_key}"
        )

    return removed is not None


def print_master_system_event(event_name, data):
    event_name = str(
        event_name
    ).strip().upper()

    if event_name != "MASTER_STARTED":
        return False

    event_log(
        f"[MASTER 연결] MASTER_STARTED 수신 | "
        f"Master={_format_stm_value(data.get('master_host'))} | "
        f"Blackbox={_format_stm_value(data.get('blackbox_port'))} | "
        f"YOLO={_format_stm_value(data.get('yolo_port'))} | "
        f"MCU={_format_stm_value(data.get('mcu_port'))}"
        f"@{_format_stm_value(data.get('mcu_baud'))} | "
        f"OpenRB={_format_stm_value(data.get('openrb_port'))}"
        f"@{_format_stm_value(data.get('openrb_baud'))}"
    )

    return True


def print_stm_connection_from_master(event_name, data):
    event_name = str(
        event_name
    ).strip().upper()

    if not event_name.startswith("MCU_UART_"):
        return False

    if event_name == "MCU_UART_CONNECTED":
        event_log(
            f"[STM 연결] 연결 완료 | "
            f"포트={_format_stm_value(data.get('port'))} | "
            f"속도={_format_stm_value(data.get('baud'))}bps"
        )
        return True

    if event_name == "MCU_UART_DISABLED":
        event_log("[STM 연결] MCU UART 비활성화")
        return True

    if event_name == "MCU_UART_CONNECT_FAILED":
        event_log(
            f"[STM 연결] 연결 실패 | "
            f"{_format_stm_value(data.get('error'))}"
        )
        return True

    event_log(
        f"[STM 연결] {event_name} | "
        f"{_format_stm_value(data)}"
    )
    return True


def handle_master_event(msg):
    """
    Master가 전송한 MASTER_EVENT를 저장한다.

    이 함수는 박스를 생성하지 않는다.
    CREATE_BOX는 별도 Depth 카메라 메시지만 담당한다.
    """
    global master_runtime_state

    category = str(
        msg.get("category", "")
    ).strip().upper()

    event_name = str(
        msg.get("event", "")
    ).strip().upper()

    data = msg.get("data", {})
    if not isinstance(data, dict):
        data = {
            "value": data
        }

    raw = msg.get("raw", None)
    timestamp = msg.get(
        "timestamp",
        time.time()
    )

    log_master_communication_event(
        category=category,
        event_name=event_name,
        data=data,
        raw=raw,
    )

    history_item = {
        "category": category,
        "event": event_name,
        "data": dict(data),
        "raw": raw,
        "timestamp": timestamp,
    }

    with master_state_lock:
        master_event_history.append(
            history_item
        )

        master_runtime_state[
            "last_event"
        ] = (
            f"{category}/{event_name}"
        )

        master_runtime_state[
            "last_timestamp"
        ] = timestamp

        if category in (
            "MCU",
            "STM",
            "STM32"
        ):
            if event_name == "RX_RAW":
                master_runtime_state[
                    "mcu_raw"
                ] = data.get(
                    "value",
                    raw
                )

            elif event_name == "ROBOT_STATE":
                master_runtime_state[
                    "robot_step"
                ] = data.get("step")

                master_runtime_state[
                    "previous_robot_step"
                ] = data.get(
                    "previous_step"
                )

            elif event_name == "RESPONSE":
                master_runtime_state[
                    "mcu_response"
                ] = data.get(
                    "response"
                )

            elif event_name == "SENSOR_TRIGGER":
                master_runtime_state[
                    "sensor_trigger_count"
                ] = data.get(
                    "count",
                    0
                )

            elif event_name == "TX_COORDINATE":
                master_runtime_state[
                    "mcu_target_x"
                ] = data.get("x")

                master_runtime_state[
                    "mcu_target_y"
                ] = data.get("y")

                master_runtime_state[
                    "mcu_target_z"
                ] = data.get("z")

            elif event_name == "TX_STOP":
                master_runtime_state[
                    "mcu_last_stop_command"
                ] = data.get(
                    "command",
                    raw
                )
                master_runtime_state[
                    "mcu_last_stop_success"
                ] = data.get("success")

            elif event_name == "TX_YOLO_RESULT":
                master_runtime_state[
                    "mcu_last_yolo_result"
                ] = data.get(
                    "value",
                    raw
                )
                master_runtime_state[
                    "mcu_last_yolo_success"
                ] = data.get("success")

        elif category == "DEPTH":
            if event_name == "VISION_PACKET":
                master_runtime_state[
                    "depth_x"
                ] = data.get("x")

                master_runtime_state[
                    "depth_y"
                ] = data.get("y")

                master_runtime_state[
                    "depth_z"
                ] = data.get("z")

                if data.get(
                    "angle"
                ) is not None:
                    master_runtime_state[
                        "depth_angle"
                    ] = data.get(
                        "angle"
                    )

            elif event_name == "ANGLE_RECEIVED":
                master_runtime_state[
                    "depth_angle"
                ] = data.get(
                    "angle"
                )

        elif category == "OPENRB":
            if event_name == "TX_ANGLE":
                master_runtime_state[
                    "openrb_angle"
                ] = data.get(
                    "angle"
                )

        elif category == "CONVEYOR":
            if event_name == "STATE":
                master_runtime_state[
                    "inspection_conveyor_running"
                ] = data.get(
                    "inspection_running"
                )

                master_runtime_state[
                    "packaging_conveyor_running"
                ] = data.get(
                    "packaging_running"
                )

        elif category == "YOLO":
            if event_name == "RESULT":
                master_runtime_state[
                    "yolo_track_id"
                ] = data.get(
                    "track_id"
                )

                master_runtime_state[
                    "yolo_result"
                ] = data.get(
                    "result"
                )

                master_runtime_state[
                    "yolo_label"
                ] = data.get(
                    "label"
                )

        elif category == "CONNECTION":
            connections = master_runtime_state[
                "connections"
            ]

            connections[
                event_name
            ] = {
                "data": dict(data),
                "timestamp": timestamp,
            }

            if event_name == "MCU_UART_CONNECTED":
                master_runtime_state[
                    "mcu_connected"
                ] = True
                master_runtime_state[
                    "mcu_port"
                ] = data.get("port")
                master_runtime_state[
                    "mcu_baud"
                ] = data.get("baud")
                master_runtime_state[
                    "mcu_connection_error"
                ] = None

            elif event_name in (
                "MCU_UART_CONNECT_FAILED",
                "MCU_UART_DISABLED",
            ):
                master_runtime_state[
                    "mcu_connected"
                ] = False
                master_runtime_state[
                    "mcu_connection_error"
                ] = data.get("error")

    # depth(1).py/master(1).py 전용 호환 이벤트를 먼저 처리한다.
    # VISION_PACKET은 상태 저장만 하고, STM LBUSY 트레이로더 시작만 처리한다.
    try:
        handle_master_bridge_compatible_event(msg)
    except NameError:
        pass
    except Exception as error:
        event_log(
            f"[MASTER 호환] MASTER_EVENT 처리 오류 | {error}"
        )

    # Master에서 받은 정보를 디지털 트윈 액추에이터/상태에 반영한다.
    # 함수는 파일 후반에 정의되며, 실제 메시지는 전체 스크립트 로드 후 수신된다.
    try:
        apply_master_event_to_digital_twin(msg)
    except NameError:
        pass
    except Exception as error:
        event_log(
            f"[TWIN 동기화] MASTER_EVENT 반영 오류 | {error}"
        )

    # Isaac 내부 이벤트 및 Depth/OpenRB/YOLO/컨베이어 정보는
    # 터미널에 표시하지 않는다. STM32/MCU 이벤트만 출력한다.
    if category in (
        "MCU",
        "STM",
        "STM32"
    ):
        print_stm_terminal_event(
            event_name=event_name,
            data=data,
            raw=raw
        )
        return

    if category == "CONNECTION":
        print_stm_connection_from_master(
            event_name=event_name,
            data=data
        )
        return

    if category == "SYSTEM":
        print_master_system_event(
            event_name=event_name,
            data=data
        )


# YOLO id ↔ Isaac Sim 박스 경로 매핑
box_id_path_dict = {}
box_path_id_dict = {}
box_result_dict = {}      # box_path -> result, 1 정상 / 0 불량 / None 미판정
box_label_dict = {}       # box_path -> normal / defect / manual / unknown
box_yaw_dict = {}         # box_path -> Isaac Sim에 적용된 회전각(deg)

# Depth CREATE_BOX로 생성됐지만 아직 검사 결과를 받지 않은 박스 순서
pending_inspection_box_paths = []

# master(1).py에서 정상 판정이 VISION_PACKET보다 먼저 들어오는 경우 보관한다.
master_pending_normal_results = deque()
master_auto_box_sequence = 0

# TCP 재전송/재연결 시 동일 제품을 중복 생성하지 않기 위한 최근 키 캐시.
master_processed_vision_keys = deque()
master_processed_vision_key_set = set()
master_processed_inspection_keys = deque()
master_processed_inspection_key_set = set()

# 검사 카메라 ByteTrack ID -> Isaac 박스 경로 매핑
inspection_track_box_path_dict = {}

# 이미 빨간색이 적용된 불량 박스
defect_red_applied_box_set = set()

# Z 제거 조건에 도달하여 비동기 삭제가 예약된 불량 박스
# DeletePrims가 다음 update에서 실행되는 동안 중복 예약되는 것을 막는다.
defect_remove_scheduled_box_set = set()


CONVEYOR_RUN_X_SPEED = -65
CONVEYOR_STOP_X_SPEED = 0.0

# 정상 박스가 이 Z 위치를 통과하면 검사 컨베이어를 0으로 정지
STOP_TARGET_X = -107.95047
STOP_TARGET_Y = 36.00002
STOP_TARGET_Z = 65.0

STOP_TOLERANCE_X = 2.0
STOP_TOLERANCE_Y = 3.0
STOP_TOLERANCE_Z = 3.0

# 불량 박스의 월드 Z가 이 좌표에 도착하거나 한 프레임 사이에
# 통과하면 리젝터를 즉시 동작시킨다.
#
# 사용자 확인 좌표:
#   X=-107.29719, Y=28.99977, Z=611.18216
#
# 실제 작동 판정은 Z만 사용한다. Depth X 변화로 박스 X가 달라져도
# 같은 컨베이어 진행 위치에서 리젝터가 동작한다.
REJECTOR_TRIGGER_X = -107.29719
REJECTOR_TRIGGER_Y = 28.99977
REJECTOR_TRIGGER_Z = 611.18216

REJECTOR_TRIGGER_TOLERANCE_Z = 3.0

# 불량 박스 리젝터 동작
# Revolute Joint 제한 범위: -110도 ~ 0도
# 동작 순서: Z=611.18216 도착 → 0도 → -100도 → 잠시 유지 → 0도
# 좌표 도착 후 별도 초 단위 지연 없이 즉시 시작한다.
REJECTOR_PUSH_VALUE = -100.0
REJECTOR_HOME_VALUE = 0.0

# 한 방향 이동에 사용할 update 프레임 수
# 불량 배출 순간에는 비교적 빠르게 움직인다.
REJECTOR_MOVE_FRAMES = 25

# -100도에서 유지할 update 프레임 수
REJECTOR_HOLD_FRAMES = 15

# 리젝터 Drive 강화값
#
# 중요:
# 코드 로드/PLAY 시에는 이 값을 적용하지 않는다.
# 불량 박스가 Z=611.18216에 도착했을 때만 일시 적용하고,
# 0도로 복귀한 직후 원래 Drive 설정으로 되돌린다.
#
# 이렇게 해야 강한 리젝터 Angular Drive가 같은 장면의
# X1/X2/Y/Z Prismatic Joint 동작을 평상시에 방해하지 않는다.
REJECTOR_BOOST_TYPE = "acceleration"
REJECTOR_BOOST_STIFFNESS = 15000.0
REJECTOR_BOOST_DAMPING = 1500.0
REJECTOR_BOOST_MAX_FORCE = 1000000000.0

# 현재 -110도 등 다른 위치에 남아 있으면 먼저 0도로 복귀
REJECTOR_HOME_RECOVERY_FRAMES = 20
REJECTOR_HOME_HOLD_FRAMES = 5

# ==================================================
# [3축 시작/픽업/적재 위치]
# ==================================================

# 시작 및 원점 복귀 위치
AUTO_HOME_X1_POSITION = 50.0
AUTO_HOME_X2_POSITION = 50.0
AUTO_HOME_Y_POSITION = 0.0
AUTO_HOME_Z_POSITION = 0.0

# 박스 정지 후 그리퍼가 박스 위로 이동하는 기준 위치
#
# 박스가 PICK_REFERENCE_BOX_WORLD_X에 있을 때
# X1/X2 Joint targetPosition 40을 사용한다.
AUTO_PICK_X1_POSITION = 40.0
AUTO_PICK_X2_POSITION = 40.0

# 박스는 Isaac Z=STOP_TARGET_Z에서 멈추므로
# 그리퍼 Z축은 고정 픽업 위치를 사용한다.
AUTO_PICK_Z_POSITION = 270.0

# ==================================================
# [Depth X 기반 동적 그리퍼 X 이동]
# ==================================================
# Depth position.x
# → Isaac 박스 생성 X
# → 박스가 고정 Z 정지선까지 이동
# → 정지 순간 실제 박스 World X 확인
# → X1/X2 Joint 목표값 보정
PICK_REFERENCE_BOX_WORLD_X = STOP_TARGET_X

# 박스 World X 1 변화당 X1/X2 targetPosition 변화량
PICK_WORLD_X_TO_JOINT_SCALE = 1.0

# 반대 방향으로 움직이면 -1.0으로 변경
PICK_WORLD_X_TO_JOINT_SIGN = 1.0

# 박스 중심과 그리퍼 중심 사이의 추가 보정
PICK_WORLD_X_OFFSET = 0.0

# Y축은 targetPosition -50 명령 시 하중으로 실제 약 -70까지 내려감
AUTO_PICK_Y_COMMAND_POSITION = -50.0
AUTO_PICK_Y_REACHED_POSITION = -70.0

# 흡착 후 상승 위치
AUTO_PICK_Y_UP_POSITION = 25.0

# v46.3:
# 픽업한 박스를 Y=50까지 상승시킨 뒤,
# 트레이 적재 준비 전에 Z축을 반드시 이 위치까지 이동한다.
AUTO_POST_PICK_Z_POSITION = -270.0

# 이동 트레이 적재 기준 위치
# 과거 Tray1이 World X=-115.39968에 정지했을 때 사용하던 Joint 기준값이다.
AUTO_PLACE_X1_POSITION = 70.0
AUTO_PLACE_X2_POSITION = 70.0
AUTO_PLACE_Z_POSITION = -250.0

# v41: 박스를 트레이 윗면에 걸치지 않고 내부로 조금 더 넣는다.
# 픽업 명령 -50 / 실제 약 -70보다 10 mm 더 하강한다.
# 트레이 바닥과 너무 가까우면 이 값만 6~8로 낮추면 된다.
TRAY_PLACE_INSERT_DEPTH = 10.0
AUTO_PLACE_Y_COMMAND_POSITION = (
    AUTO_PICK_Y_COMMAND_POSITION
    - TRAY_PLACE_INSERT_DEPTH
)
AUTO_PLACE_Y_REACHED_POSITION = (
    AUTO_PICK_Y_REACHED_POSITION
    - TRAY_PLACE_INSERT_DEPTH
)
# 기존 수동 키 함수 호환용
AUTO_PLACE_Y_POSITION = AUTO_PLACE_Y_COMMAND_POSITION

# v46:
# 실제 Stage에서 확인한 적재 위치를 전체 적재 로직에 사용한다.
AUTO_PLACE_Z_POSITION = -250.0
AUTO_PLACE_Y_COMMAND_POSITION = -50.0
AUTO_PLACE_Y_REACHED_POSITION = -50.0
AUTO_PLACE_Y_POSITION = AUTO_PLACE_Y_COMMAND_POSITION

# ==================================================
# [포장 컨베이어 시작 조건 + 이동 트레이 실시간 추종]
# ==================================================
PACK_CONVEYOR_PATH = "/assembly714/Pack_conveyoor/pack_conveyor"

# 초기 적재 트레이
TRAY_PATH = "/assembly714/Tray1"

# ==================================================
# [v47 두 박스 좌표 트리거 적재]
# ==================================================
# Tray1은 assembly714 바로 아래에 있고,
# Tray2~Tray5는 Tray_loader 내부에 있다.
PACK_TRAY_PATH_BY_NUMBER = {
    1: "/assembly714/Tray1",
    2: "/assembly714/Tray_loader/Tray2",
    3: "/assembly714/Tray_loader/Tray3",
    4: "/assembly714/Tray_loader/Tray4",
    5: "/assembly714/Tray_loader/Tray5",
}

# ==================================================
# [v51.7 첫 번째 정상 박스 로봇 이동 시작 대기 조건]
# ==================================================
# 첫 번째 정상 박스(place_slot=1)가 픽업 정지 위치에 도착해도
# 직교로봇은 곧바로 박스 위로 이동하지 않는다.
# Tray2의 Transform Translate X가 239에 도착하거나 해당 좌표를 통과한 순간부터
# X1/X2/Z 박스 위 이동 -> 그리퍼 각도 맞춤 -> Y 하강 -> 흡착을 진행한다.
FIRST_TRAY_PICK_WAIT_ENABLE = True
FIRST_TRAY_PICK_WAIT_ONLY_PLACE_SLOT_1 = True
FIRST_TRAY_PICK_WAIT_PATH = PACK_TRAY_PATH_BY_NUMBER[2]
FIRST_TRAY_PICK_WAIT_TARGET_X = 250.0
FIRST_TRAY_PICK_WAIT_TOLERANCE_X = 1.0

# Isaac Property 창의 Transform/Translate X는 Local 좌표이므로 기본 LOCAL 사용.
# 실제 월드 좌표를 기준으로 쓰고 싶으면 "WORLD"로 변경한다.
FIRST_TRAY_PICK_WAIT_COORDINATE_MODE = "LOCAL"

# Tray2가 대기 시작 전에 이미 목표 X를 통과한 경우 영구 대기를 방지한다.
# 현재 장면에서는 Tray2의 X가 큰 값에서 작은 값으로 이동한다고 가정한다.
FIRST_TRAY_PICK_WAIT_EXPECTED_DIRECTION = -1.0
FIRST_TRAY_PICK_WAIT_ALLOW_ALREADY_PASSED = False

# 적재 순번별 좌표.
#
# 트레이 이동 방향은 큰 X -> 작은 X이다.
#
# place_slot=1:
#   Tray X=4에서 현재 적재 트레이 선택 및 X1/X2=60 준비
#   Tray X=-30에서 X1/X2 60 -> -19 대각선 적재 시작
#
# place_slot=2:
#   1번과 같은 트레이를 사용하며 트레이 좌표를 기다리지 않는다.
#   X1/X2=-200 준비 완료 직후 -264 대각선 적재 시작
TRAY_PLACE_SLOT_CONFIG = {
    1: {
        "select_trigger_x": 285.0,
        "place_trigger_x": -50.0,
        "prepare_x": 80.0,
        "final_x": -25.0,
        "wait_for_tray_trigger": True,
        "label": "1번 박스",
    },
    2: { 
        "select_trigger_x": None,
        # wait_for_tray_trigger=False이므로 이 값은 호환용이다.
        "place_trigger_x": -290.0,
        "prepare_x": -120.0,
        "final_x": -150.0,
        "wait_for_tray_trigger": False,
        "label": "2번 박스",
    },
}

# 트레이는 큰 X에서 작은 X 방향으로 이동한다.
PACK_TRAY_EXPECTED_DIRECTION = -1.0
TRAY_COORD_TRIGGER_TOLERANCE = 1.0
PACK_TRAY_WAIT_TIMEOUT_SEC = 90.0

# 동기 S-Curve는 모든 축에 같은 진행률을 적용한다.
# 따라서 X와 Y가 동시에 시작하고 동시에 끝나 실제 대각선 궤적을 만든다.
TRAY_DIAGONAL_S_CURVE_FRAMES = 75

TRAY_CONFIRMED_PLACE_Z_POSITION = -250.0
TRAY_CONFIRMED_PLACE_Y_POSITION = -50.0
TRAY_PRE_RELEASE_HOLD_SEC = 1.0

# 검사 컨베이어는 -65, 포장 컨베이어는 +35
PACK_CONVEYOR_RUN_SPEED = 35.0
PACK_CONVEYOR_STOP_SPEED = 0.0

# 트레이로더 공급이 완료된 후 포장 컨베이어를 움직이는 기존 조건은 유지한다.
PACK_START_TRIGGER_Z = 250.0
PACK_START_TRIGGER_TOLERANCE_Z = 3.0

# 아래 값은 기존 보조 함수 호환용이다.
PACK_TRAY_TRIGGER_WORLD_X = TRAY_PLACE_SLOT_CONFIG[1]["select_trigger_x"]
PACK_TRAY_TRIGGER_TOLERANCE = TRAY_COORD_TRIGGER_TOLERANCE
TRAY_REFERENCE_WORLD_X = -115.39968
TRAY_REFERENCE_JOINT_X1 = AUTO_PLACE_X1_POSITION
TRAY_REFERENCE_JOINT_X2 = AUTO_PLACE_X2_POSITION
TRAY_WORLD_X_TO_JOINT_SCALE = 1.0
TRAY_WORLD_X_TO_JOINT_SIGN = 1.0
TRAY_WORLD_Z_TO_JOINT_SCALE = 1.0
TRAY_WORLD_Z_TO_JOINT_SIGN = 1.0
TRAY_CATCH_LEAD_WORLD_X = 0.0

# ==================================================
# [v35 트레이 실제 형상 중심 + Z 기준 위치 복구]
# ==================================================
# v34에서는 박스-트레이 World 좌표 차이를 현재 Z Joint target에 더해
# 픽업 위치 Z=270을 기준으로 계산되어 적재 기준 Z=-250으로 가지 못했다.
# v35는 트레이 BBox 중심 추적은 유지하되 Z축은 검증된 적재 기준값 -250을 먼저 사용하고,
# 그 위치에 도달한 뒤에만 제한된 미세 보정을 적용한다.
TRAY_PLACE_OFFSET_WORLD_X = 0.0
TRAY_PLACE_OFFSET_WORLD_Z = 0.0

# 박스 중심과 트레이 중심의 실제 수평 정렬 허용 오차
TRAY_BOX_CENTER_TOLERANCE_X = 8.0
TRAY_BOX_CENTER_TOLERANCE_Z = 8.0

# CATCH 및 DESCEND 단계에서 실제 중심 정렬이 연속으로 유지되어야 하는 프레임
TRAY_CENTER_STABLE_FRAMES_REQUIRED = 5
TRAY_RELEASE_CENTER_STABLE_FRAMES_REQUIRED = 5

# World 오차를 Joint 목표 보정량으로 변환하는 비율
TRAY_CENTER_SERVO_GAIN_X = 1.0
TRAY_CENTER_SERVO_GAIN_Z = 1.0

# ==================================================
# [v35 Z축 기준 위치 복구 + 제한 미세 정렬]
# ==================================================
# 가장 먼저 검증된 적재 Z targetPosition(-250)으로 이동한다.
# v34처럼 픽업 위치의 현재 Z targetPosition(약 270)을 기준으로
# 박스-트레이 World 차이를 더하지 않는다.
TRAY_Z_USE_FIXED_PLACE_BASE = True

# 고정 적재 Z에 도달한 뒤에만 BBox 중심 오차를 조금씩 보정한다.
# 큰 World 좌표 오차를 한 번에 Z target에 더하지 않으므로
# Z가 20~30 부근에서 멈추거나 반대 방향으로 움직이는 현상을 막는다.
TRAY_Z_FINE_ALIGN_ENABLED = True
TRAY_Z_FINE_ALIGN_GAIN = 0.25
TRAY_Z_FINE_ALIGN_MAX_STEP = 2.0
TRAY_Z_FINE_ALIGN_MAX_OFFSET = 45.0
TRAY_Z_FINE_ALIGN_DEADBAND = 1.5

# STM tracking.c와 같은 추종 거리
TRAY_TRACK_DESCENT_DISTANCE = 50.0
TRAY_TRACK_RISE_DISTANCE = 10.0

# 이동 트레이 추종 속도. 포장 컨베이어 +50보다 충분히 빨라야 따라잡을 수 있다.
TRAY_CATCH_X_MAX_STEP = 8.0
TRAY_TRACK_X_MAX_STEP = 4.0
TRAY_TRACK_Z_MAX_STEP = 10.0
TRAY_TRACK_Y_MAX_STEP = 5.0

TRAY_FOLLOW_TARGET_TOLERANCE = 1.5
TRAY_CATCH_STABLE_FRAMES_REQUIRED = 3
TRAY_FOLLOW_TIMEOUT_FRAMES = 1200

# 기존 수동 키 함수 호환용
X_TARGET_POSITION = AUTO_PICK_X1_POSITION
Y_DOWN_POSITION = AUTO_PICK_Y_COMMAND_POSITION
Y_MID_POSITION = AUTO_PLACE_Y_POSITION
Y_HOME_POSITION = AUTO_HOME_Y_POSITION
Z_TARGET_POSITION = AUTO_PICK_Z_POSITION

# ==================================================
# [3축 허용 범위]
# ==================================================

PICK_X1_TARGET_MIN = -500.0
PICK_X1_TARGET_MAX = 200.0
PICK_X2_TARGET_MIN = -500.0
PICK_X2_TARGET_MAX = 200.0

# 화면에서 확인한 Y Joint 범위
PICK_Y_TARGET_MIN = -90.0
PICK_Y_TARGET_MAX = 170.0

# Z=270 및 Z=-250 이동이 가능하도록 확장
PICK_Z_TARGET_MIN = -300.0
PICK_Z_TARGET_MAX = 300.0

# ==================================================
# [S-Curve 및 실제 위치 완료 판정]
# ==================================================

# 한 프레임에 targetPosition을 변경하는 최대량
# 값이 작을수록 더 천천히 움직인다.
AXIS_MAX_STEP = {
    # v30/v31보다 빠른 일반 이동 속도
    "X1": 6.00,
    "X2": 6.00,
    "Y": 4.00,
    "Z": 10.00
}

# 시작과 끝에서도 지나치게 느려지지 않도록 최소 속도를 높인다.
S_CURVE_MIN_SPEED_RATIO = 0.35

# 실제 Joint position 허용 오차
AXIS_POSITION_TOLERANCE = {
    "X1": 2.0,
    "X2": 2.0,
    "Y": 3.0,
    "Z": 3.0
}

# Y 픽업/적재 하강은 target -50 / 실제 약 -70 조건이므로 별도 오차 사용
PICK_Y_REACHED_TOLERANCE = 6.0
PLACE_Y_REACHED_TOLERANCE = 6.0

# 최종 Drive targetPosition까지 S-Curve 명령이 완료된 뒤
# 물리가 따라갈 수 있도록 유지할 프레임 수.
#
# 이전 버전은 Y/Z Geometry raw 값의 진동 때문에 초기 위치 완료 판정이
# 계속 막힐 수 있었다. v10에서는 3축 이동 자체는 성공 코드와 동일하게
# Drive targetPosition을 기준으로 진행하고, 최종 명령 유지 프레임 후
# 다음 단계로 넘어간다.
MOTION_COMMAND_SETTLE_FRAMES_REQUIRED = 3

# Drive 명령은 끝났지만 실제 Geometry 위치가 도달하지 못한 경우의 안전 타임아웃
MOTION_ACTUAL_WAIT_TIMEOUT_FRAMES = 300

# 박스를 흡착한 뒤에는 3축을 더 천천히 움직여 흔들림을 줄인다.
CARRY_AXIS_MAX_STEP = {
    # 흡착 후 X1/X2/Z 운반 속도는 현재 파일의 값을 유지한다.
    # 일반 운반 및 적재 중 Y축은 3.80으로 사용한다.
    "X1": 5.80,
    "X2": 5.80,
    "Y": 4.20,
    "Z": 8.80
}

# 흡착 직후 박스를 들어 올리는 LIFT_Y 단계에서만 사용하는 전용 속도.
# 일반 운반, 대각선 적재 하강, 해제 후 상승에는 적용하지 않는다.
POST_PICK_LIFT_Y_MAX_STEP = 4.20

# 운반 중 목표 명령 유지 프레임을 최소화한다.
CARRY_MOTION_SETTLE_FRAMES_REQUIRED = 3

# 강한 소프트웨어 흡착 직후 최소 안정화 프레임
ATTACH_STABILIZE_FRAMES_REQUIRED = 2

# ==================================================
# [v41 사용자가 요청한 흡착/해제 설정]
# ==================================================
# 흡착 상태에서 그리퍼 회전량을 박스 회전에 계속 반영한다.
ATTACHED_BOX_ROTATION_FOLLOW_ENABLED = True

# 트레이 내부에서 논리 흡착을 해제한 뒤 Y축이 최소 이 거리만큼 상승하면
# Collision/RigidBody 복구를 허용한다.
SAFE_RELEASE_LIFT_Y = 5.0

# 물리를 비활성화한 채 트레이에 박스를 붙여 두는 최소 프레임 수.
SAFE_RELEASE_DISABLED_HOLD_FRAMES = 10

# 해제 직후 박스 속도를 0으로 강제하는 프레임 수.
SAFE_RELEASE_ZERO_VELOCITY_FRAMES = 2

# 최종 안착 안정화 프레임 수.
SAFE_RELEASE_SETTLE_FRAMES = 3

# 이동 트레이에서는 정지 안착 대기를 사용하지 않지만 호환용으로 유지
RELEASE_SETTLE_FRAMES_REQUIRED = SAFE_RELEASE_SETTLE_FRAMES

# ==================================================
# [v37 이동 트레이 최종 흡착 해제 + 그리퍼 이격 확인]
# ==================================================
# 박스를 그리퍼에서 논리적으로 분리한 뒤에는 RigidBody/Collision을 바로 켜지 않는다.
# 박스를 이동 중인 트레이에 계속 고정한 상태로 그리퍼를 먼저 충분히 상승시킨 뒤,
# 박스 물리를 복구하고 트레이와 같은 월드 속도를 전달한다.
#
# v36은 10프레임 뒤 물리를 복구했기 때문에 Y축이 -50 -> +50으로 완전히 상승하기 전에
# 그리퍼와 박스 Collision이 다시 활성화되어 박스가 그리퍼에 걸려 따라오는 현상이 있었다.
TRAY_RELEASE_LOCK_FRAMES_REQUIRED = SAFE_RELEASE_DISABLED_HOLD_FRAMES
TRAY_RELEASE_MAX_LOCK_FRAMES = 120
TRAY_RELEASE_SAFE_Y_COMMAND_TOLERANCE = 2.0
TRAY_RELEASE_MATCH_TRAY_VELOCITY = True
TRAY_RELEASE_MIN_VALID_SPEED = 0.5
TRAY_RELEASE_MAX_VALID_SPEED = 300.0

# 트레이 중심 정렬과 Y 하강이 끝난 뒤 너무 오래 50 mm를 기다리지 않고,
# 실제 중심이 맞는 구간에서 빠르게 논리 흡착을 해제한다.
TRAY_RELEASE_TRIGGER_DISTANCE = 5.0

# 적재 하강은 실제 Geometry 원점이 설치 상태에 따라 크게 달라질 수 있다.
# raw Geometry -70 조건 대신 Drive 명령 도달 + 박스/트레이 중심 정렬을
# 최종 해제 조건으로 사용한다. 픽업 흡착 조건은 기존 그대로 유지한다.
TRAY_RELEASE_USE_RAW_Y_CONDITION = False

# 실제 Joint 상태 속성 후보
JOINT_POSITION_ATTR_CANDIDATES = [
    "state:linear:physics:position",
    "physics:jointPosition",
    "jointState:position"
]

JOINT_VELOCITY_ATTR_CANDIDATES = [
    "state:linear:physics:velocity",
    "physics:jointVelocity",
    "jointState:velocity"
]

# 실제 Position 판독 방식 기록
# - joint_state: PhysxSchema.JointStateAPI
# - joint_geometry: body0/body1의 실제 월드 Transform으로 계산
axis_position_source_dict = {}
axis_position_source_log_set = set()

# Geometry raw position과 Drive targetPosition 사이의 고정 원점 차이
axis_geometry_offset_dict = {}
axis_geometry_raw_at_calibration_dict = {}
axis_target_at_calibration_dict = {}
axis_geometry_calibrated = False


def _get_joint_body_world_matrix(joint, body_index):
    """
    PrismaticJoint의 body0/body1 월드 행렬을 반환한다.
    body 관계가 비어 있으면 World 고정 바디로 보고 단위 행렬을 사용한다.
    """
    try:
        rel = (
            joint.GetBody0Rel()
            if body_index == 0
            else joint.GetBody1Rel()
        )

        targets = rel.GetTargets()

        if not targets:
            return Gf.Matrix4d(1.0)

        body_prim = get_prim(str(targets[0]))

        if body_prim is None:
            return None

        return UsdGeom.Xformable(
            body_prim
        ).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )

    except Exception:
        return None


def _quat_rotate_vector(quaternion, vector):
    try:
        rotation = Gf.Rotation(quaternion)

        rotated = rotation.TransformDir(
            Gf.Vec3d(
                float(vector[0]),
                float(vector[1]),
                float(vector[2])
            )
        )

        return Gf.Vec3d(
            float(rotated[0]),
            float(rotated[1]),
            float(rotated[2])
        )
    except Exception:
        return Gf.Vec3d(
            float(vector[0]),
            float(vector[1]),
            float(vector[2])
        )


def get_prismatic_position_from_geometry(joint_path):
    """
    USD에 실시간 JointState 속성이 없는 PrismaticJoint도 지원한다.

    실제 body0/body1 월드 Transform과 joint local frame을 사용하여
    두 Joint anchor 사이의 변위를 축 방향으로 투영한다.
    이 값은 Drive targetPosition이 아니라 현재 물리 자세에서 계산된다.
    """
    prim = get_prim(joint_path)

    if prim is None:
        return None

    try:
        joint = UsdPhysics.PrismaticJoint(
            prim
        )

        if not joint:
            return None

        body0_matrix = _get_joint_body_world_matrix(
            joint,
            0
        )
        body1_matrix = _get_joint_body_world_matrix(
            joint,
            1
        )

        if (
            body0_matrix is None
            or body1_matrix is None
        ):
            return None

        local_pos0 = joint.GetLocalPos0Attr().Get()
        local_pos1 = joint.GetLocalPos1Attr().Get()
        local_rot0 = joint.GetLocalRot0Attr().Get()

        if local_pos0 is None:
            local_pos0 = Gf.Vec3f(
                0.0,
                0.0,
                0.0
            )

        if local_pos1 is None:
            local_pos1 = Gf.Vec3f(
                0.0,
                0.0,
                0.0
            )

        anchor0_world = body0_matrix.Transform(
            Gf.Vec3d(
                float(local_pos0[0]),
                float(local_pos0[1]),
                float(local_pos0[2])
            )
        )

        anchor1_world = body1_matrix.Transform(
            Gf.Vec3d(
                float(local_pos1[0]),
                float(local_pos1[1]),
                float(local_pos1[2])
            )
        )

        axis_token = joint.GetAxisAttr().Get()
        axis_name = str(
            axis_token
        ).upper()

        if axis_name == "Y":
            axis_local = Gf.Vec3d(
                0.0,
                1.0,
                0.0
            )
        elif axis_name == "Z":
            axis_local = Gf.Vec3d(
                0.0,
                0.0,
                1.0
            )
        else:
            axis_local = Gf.Vec3d(
                1.0,
                0.0,
                0.0
            )

        if local_rot0 is not None:
            axis_body0 = _quat_rotate_vector(
                local_rot0,
                axis_local
            )
        else:
            axis_body0 = axis_local

        axis_world = body0_matrix.TransformDir(
            axis_body0
        )

        axis_length = axis_world.GetLength()

        if axis_length <= 1e-9:
            return None

        axis_world = axis_world / axis_length

        anchor_delta = (
            anchor1_world
            - anchor0_world
        )

        return float(
            Gf.Dot(
                anchor_delta,
                axis_world
            )
        )

    except Exception:
        return None


def get_axis_raw_geometry_position(axis_name):
    """
    오프셋 보정 전 PrismaticJoint Geometry 위치를 반환한다.
    """
    joint_path = AXIS_JOINT_PATHS.get(
        axis_name
    )

    if joint_path is None:
        return None

    return get_prismatic_position_from_geometry(
        joint_path
    )

def get_prismatic_joint_world_axis(axis_name):
    """
    PrismaticJoint의 양(+) 방향을 World 단위 벡터로 반환한다.

    포장 컨베이어의 local X가 Stage에서 World X인지 World Z인지와 무관하게,
    움직이는 Tray의 World 변위를 실제 로봇 Joint 축 방향으로 투영하기 위해 사용한다.
    """
    joint_path = AXIS_JOINT_PATHS.get(axis_name, None)

    if joint_path is None:
        return None

    prim = get_prim(joint_path)

    if prim is None:
        return None

    try:
        joint = UsdPhysics.PrismaticJoint(prim)

        if not joint:
            return None

        body0_matrix = _get_joint_body_world_matrix(joint, 0)

        if body0_matrix is None:
            return None

        local_rot0 = joint.GetLocalRot0Attr().Get()
        axis_token = joint.GetAxisAttr().Get()
        axis_token_name = str(axis_token).upper()

        if axis_token_name == "Y":
            axis_local = Gf.Vec3d(0.0, 1.0, 0.0)
        elif axis_token_name == "Z":
            axis_local = Gf.Vec3d(0.0, 0.0, 1.0)
        else:
            axis_local = Gf.Vec3d(1.0, 0.0, 0.0)

        if local_rot0 is not None:
            axis_body0 = _quat_rotate_vector(local_rot0, axis_local)
        else:
            axis_body0 = axis_local

        axis_world = body0_matrix.TransformDir(axis_body0)
        axis_length = axis_world.GetLength()

        if axis_length <= 1e-9:
            return None

        return axis_world / axis_length

    except Exception:
        return None


def get_axis_actual_from_geometry_offset(axis_name, raw_offset):
    """
    raw_offset = geometry_raw - Drive좌표 실제값 형태로 저장해 둔 보정값을 사용한다.
    """
    raw_now = get_axis_raw_geometry_position(axis_name)

    if raw_now is None or raw_offset is None:
        return None

    return float(raw_now) - float(raw_offset)


def calibrate_axis_geometry_offsets(force=False):
    """
    현재 Drive targetPosition과 실제 Body Transform 사이의 고정 원점 차이를
    축별로 자동 보정한다.

        offset = geometry_raw - drive_target
        actual = geometry_raw - offset
    """
    global axis_geometry_calibrated

    if axis_geometry_calibrated and not force:
        return True

    offsets = {}
    success = True

    for axis_name in (
        "X1",
        "X2",
        "Y",
        "Z"
    ):
        raw_position = get_axis_raw_geometry_position(
            axis_name
        )
        drive_target = get_axis_target_position(
            axis_name
        )

        if raw_position is None or drive_target is None:
            success = False
            event_log(
                f"[축 원점 보정 오류] {axis_name} | "
                f"raw={raw_position} | target={drive_target}"
            )
            continue

        raw_position = float(raw_position)
        drive_target = float(drive_target)
        offset = raw_position - drive_target

        offsets[axis_name] = offset
        axis_geometry_raw_at_calibration_dict[axis_name] = raw_position
        axis_target_at_calibration_dict[axis_name] = drive_target

        event_log(
            f"[축 원점 보정] {axis_name} | "
            f"raw={raw_position:.3f} | "
            f"drive={drive_target:.3f} | "
            f"offset={offset:.3f}"
        )

    if not offsets:
        return False

    # 일부 축 판독에 실패해도 성공한 축의 보정값은 유지한다.
    # 특히 Y축 값만 확보되어도 픽업/적재 실제 하강 위치 판정에 사용할 수 있다.
    axis_geometry_offset_dict.clear()
    axis_geometry_offset_dict.update(offsets)
    axis_geometry_calibrated = bool(success)

    event_log(
        "[축 원점 보정 완료] "
        + ", ".join(
            f"{axis}={offset:.3f}"
            for axis, offset in offsets.items()
        )
        + (
            ""
            if success
            else " | 일부 축 판독 실패, 성공 축만 사용"
        )
    )

    return True


def ensure_joint_state_api(axis_name):
    """
    기존 USD에 PhysX가 실제로 제공한 JointStateAPI가 있을 때만 반환한다.

    이전 버전처럼 position=0 속성을 새로 만들면 PhysX가 갱신하지 않는
    고정값을 실제 위치로 오인할 수 있으므로 절대 새로 생성하지 않는다.
    """
    joint_path = AXIS_JOINT_PATHS.get(
        axis_name
    )

    if joint_path is None:
        return None

    prim = get_prim(
        joint_path
    )

    if prim is None:
        return None

    try:
        position_attr = prim.GetAttribute(
            "state:linear:physics:position"
        )

        velocity_attr = prim.GetAttribute(
            "state:linear:physics:velocity"
        )

        if (
            position_attr.IsValid()
            and velocity_attr.IsValid()
        ):
            return PhysxSchema.JointStateAPI(
                prim,
                UsdPhysics.Tokens.linear
            )

    except Exception:
        pass

    return None

def ensure_all_joint_state_apis():
    """
    JointStateAPI를 임의로 생성하지 않는다.
    실제 위치는 body0/body1 월드 Transform 기반 geometry 판독을 우선 사용한다.
    """
    return


# 그리퍼 인식 범위
# 현재 장면의 높이축은 Y이므로 박스 윗면은 BBox의 max Y로 판정
TOP_SURFACE_MARGIN = 25.0

# 실제 흡착 기준점과 박스 윗면 사이의 최대 허용 거리.
# 이 범위를 벗어나면 강제로 잡지 않고 시퀀스를 중단한다.
MAX_VERTICAL_GAP = 15.0

# 실제 흡착 Prim과 컵 아랫면 사이에 오차가 있을 때만 보정한다.
SUCTION_CONTACT_Y_OFFSET = 0.0

# 중요:
# 흡착 순간 박스 위치를 강제로 끌어올리지 않는다.
# 현재 위치를 그대로 저장해 그리퍼 내부 관통을 방지한다.

SPAWN_DEBOUNCE_TIME = 0.5


# ==================================================
# [4. 이전 코드 정리]
# ==================================================

def cleanup_previous_standalone_test_runtime():
    """
    이전에 실행한 독립 그리퍼/컨베이어 테스트의 이벤트 구독을 해제한다.

    v46.2는 독립 테스트의 별도 motion_controller나 start_conveyor를
    사용하지 않고, 이 파일 안의 v46 상태 머신만 사용한다.
    """
    runtime_keys = (
        "_ASSEMBLY714_TWO_PER_TRAY_CONVEYOR_TEST_V1",
        "_ASSEMBLY714_REAL_GRIPPER_CONVEYOR_TWO_PER_TRAY_TEST_V2",
        "_ASSEMBLY714_REAL_GRIPPER_CONVEYOR_TWO_PER_TRAY_TEST_V2_1",
        "_ASSEMBLY714_REAL_GRIPPER_CONVEYOR_TWO_PER_TRAY_TEST_V2_2",
    )

    try:
        appwindow = (
            omni.appwindow
            .get_default_app_window()
        )
        input_interface = (
            carb.input
            .acquire_input_interface()
        )
        keyboard = appwindow.get_keyboard()
    except Exception:
        input_interface = None
        keyboard = None

    for runtime_key in runtime_keys:
        old_runtime = getattr(
            builtins,
            runtime_key,
            None,
        )

        if not isinstance(
            old_runtime,
            dict,
        ):
            continue

        old_runtime["active"] = False

        for subscription_key in (
            "keyboard_sub",
            "update_sub",
            "timeline_sub",
        ):
            subscription = old_runtime.get(
                subscription_key
            )

            if subscription is None:
                continue

            if (
                subscription_key == "keyboard_sub"
                and input_interface is not None
                and keyboard is not None
            ):
                try:
                    input_interface.unsubscribe_to_keyboard_events(
                        keyboard,
                        subscription,
                    )
                except Exception:
                    pass

            try:
                unsubscribe = getattr(
                    subscription,
                    "unsubscribe",
                    None,
                )

                if callable(unsubscribe):
                    unsubscribe()
            except Exception:
                pass

            old_runtime[subscription_key] = None


def cleanup_old_sessions():
    cleanup_previous_standalone_test_runtime()

    # 기존 TCP 서버 정리
    old_tcp_state = getattr(builtins, YOLO_TCP_STATE_NAME, None)

    if old_tcp_state is not None:
        debug_print("[TCP SERVER] 기존 YOLO TCP 서버 정리 시도")

        try:
            old_tcp_state["running"] = False
        except Exception:
            pass

        try:
            if old_tcp_state.get("server_socket") is not None:
                old_tcp_state["server_socket"].close()
        except Exception as e:
            debug_print("[TCP SERVER] 기존 소켓 종료 중 오류:", e)

        try:
            if old_tcp_state.get("update_sub") is not None:
                old_tcp_state["update_sub"].unsubscribe()
        except Exception as e:
            debug_print("[TCP SERVER] 기존 update 구독 해제 중 오류:", e)

    setattr(builtins, YOLO_TCP_STATE_NAME, yolo_tcp_state)

    # 기존 Digital Twin 서버/구독 정리
    old_twin_state = getattr(
        builtins,
        TWIN_TCP_STATE_NAME,
        None,
    )

    if isinstance(old_twin_state, dict):
        try:
            old_twin_state["running"] = False
        except Exception:
            pass

        try:
            old_server_socket = old_twin_state.get("server_socket")
            if old_server_socket is not None:
                old_server_socket.close()
        except Exception:
            pass

        try:
            old_update_sub = old_twin_state.get("update_sub")
            if old_update_sub is not None:
                old_update_sub.unsubscribe()
        except Exception:
            pass

        try:
            for client_info in list(
                old_twin_state.get("clients", {}).values()
            ):
                old_socket = client_info.get("socket")
                if old_socket is not None:
                    try:
                        old_socket.shutdown(socket.SHUT_RDWR)
                    except Exception:
                        pass
                    try:
                        old_socket.close()
                    except Exception:
                        pass
        except Exception:
            pass

    setattr(
        builtins,
        TWIN_TCP_STATE_NAME,
        digital_twin_state,
    )

    appwindow = omni.appwindow.get_default_app_window()
    input_interface = carb.input.acquire_input_interface()
    keyboard = appwindow.get_keyboard()

    kb_sub_names = [
        "master_kb_sub",
        "final_system_kb_sub",
        "auto_xyz_kb_sub",
        "spawn_rejector_kb_sub",
        "gripper_y_kb_sub",
        "y_axis_kb_sub",
        "gripper_test_kb_sub",
        "manual_grip_kb_sub",
        "fixed_grip_kb_sub",
        "dummy_fixed_grip_kb_sub",
        "top_surface_grip_kb_sub",
        "no_jump_grip_kb_sub",
        "strict_top_grip_kb_sub",
        "y_axis_top_grip_kb_sub",
        "clean_no_jump_grip_kb_sub",
        "xyz_grip_kb_sub",
        "conveyor_kb_sub",
        "conveyor_test_kb_sub",
        "conveyor_toggle_kb_sub",
        "conveyor_v_toggle_kb_sub",
        "conveyor_restore_kb_sub",
        "conveyor_auto_stop_kb_sub",
        "rejector_delay_kb_sub",
        "auto_rejector_kb_sub",
        "delayed_rejector_kb_sub",
        "odd_even_rejector_kb_sub",
        "yolo_tcp_kb_sub",
        "pack_conveyor_kb_sub"
    ]

    for name in kb_sub_names:
        if name in globals():
            try:
                input_interface.unsubscribe_to_keyboard_events(
                    keyboard,
                    globals()[name]
                )
                debug_print("이전 키보드 이벤트 제거:", name)
            except Exception as e:
                debug_print("이전 키보드 이벤트 제거 실패:", name, e)

            try:
                globals()[name] = None
            except:
                pass

    other_sub_names = [
        "master_timeline_sub",
        "master_update_sub",
        "final_system_timeline_sub",
        "final_system_update_sub",
        "auto_xyz_timeline_sub",
        "auto_xyz_update_sub",
        "conveyor_toggle_timeline_sub",
        "conveyor_restore_timeline_sub",
        "conveyor_v_toggle_timeline_sub",
        "conveyor_auto_stop_timeline_sub",
        "conveyor_auto_stop_update_sub",
        "xyz_grip_update_sub",
        "clean_no_jump_grip_update_sub",
        "no_jump_grip_update_sub",
        "fixed_grip_update_sub",
        "dummy_fixed_grip_update_sub",
        "top_surface_grip_update_sub",
        "strict_top_grip_update_sub",
        "y_axis_top_grip_update_sub",
        "rejector_delay_update_sub",
        "auto_rejector_update_sub",
        "delayed_rejector_update_sub",
        "odd_even_rejector_update_sub",
        "rejector_delay_timeline_sub",
        "auto_rejector_timeline_sub",
        "delayed_rejector_timeline_sub",
        "odd_even_rejector_timeline_sub",
        "yolo_tcp_update_sub",
        "pack_conveyor_update_sub",
        "pack_conveyor_timeline_sub"
    ]

    for name in other_sub_names:
        if name in globals():
            try:
                globals()[name] = None
                debug_print("이전 이벤트 참조 제거:", name)
            except:
                pass

    old_controller_names = [
        "spawn_rejector_controller",
        "automation_system",
        "box_system",
        "final_bot"
    ]

    for name in old_controller_names:
        if name in globals():
            try:
                obj = globals()[name]

                if hasattr(obj, "destroy"):
                    obj.destroy()

                del globals()[name]
                debug_print("이전 컨트롤러 제거:", name)

            except Exception as e:
                debug_print("이전 컨트롤러 제거 실패:", name, e)


def is_spawned_box_root_path(path):
    if not path.startswith(ORIGINAL_BOX_PATH_PREFIX + "_prim_spawned_"):
        return False

    if path.count("/") == 2:
        return True

    return False


def clear_stage_selection():
    """
    삭제될 Prim이 Viewport Transform Manipulator의 pivot으로 남아 있으면
    omni.kit.manipulator.prim.core에서 _pivot_prim=None 오류가 날 수 있다.
    삭제 전후 Stage 선택을 비워 stale pivot 참조를 제거한다.
    """
    try:
        selection = omni.usd.get_context().get_selection()
        selection.set_selected_prim_paths([], True)
        return True
    except Exception as error:
        event_log(
            f"[선택 해제 경고] Stage 선택 초기화 실패 | {error}"
        )
        return False


def collect_spawned_box_paths():
    stage = omni.usd.get_context().get_stage()
    delete_paths = []

    if stage is None:
        return delete_paths

    for prim in stage.Traverse():
        path = str(prim.GetPath())

        if is_spawned_box_root_path(path):
            delete_paths.append(path)

        if path.startswith("/World/Dynamic_Box"):
            delete_paths.append(path)

        if path.startswith("/World/TCP_Boxes"):
            delete_paths.append(path)

    return list(dict.fromkeys(delete_paths))


async def delete_prims_safely(paths, reason="cleanup"):
    """
    Viewport manipulator가 선택 변경을 처리할 프레임을 준 뒤 Prim을 삭제한다.
    삭제 자체는 반드시 Isaac Sim 메인 스레드의 async update 흐름에서 수행한다.
    """
    valid_paths = [
        str(path)
        for path in dict.fromkeys(paths or [])
        if get_prim(str(path)) is not None
    ]

    if not valid_paths:
        return

    clear_stage_selection()

    app = omni.kit.app.get_app()

    # Manipulator의 이전 pivot 참조가 해제될 시간을 확보한다.
    await app.next_update_async()
    await app.next_update_async()

    valid_paths = [
        path
        for path in valid_paths
        if get_prim(path) is not None
    ]

    if not valid_paths:
        clear_stage_selection()
        return

    try:
        omni.kit.commands.execute(
            "DeletePrims",
            paths=valid_paths
        )

        event_log(
            f"[안전 삭제 완료] reason={reason} | "
            f"{len(valid_paths)}개 Prim"
        )
    except Exception as error:
        event_log(
            f"[안전 삭제 오류] reason={reason} | {error}"
        )

    await app.next_update_async()
    clear_stage_selection()


async def cleanup_spawned_boxes_async(
    captured_delete_paths
):
    """
    스크립트 시작 순간 이미 존재하던 박스 경로만 삭제한다.

    중요:
    async 함수 안에서 Stage를 다시 탐색하면, 전체 코드 실행 직후 사용자가
    새로 생성한 박스까지 script_start 정리 대상으로 잡히는 경쟁 조건이 생긴다.
    따라서 삭제 대상은 async 작업을 예약하기 전에 동기적으로 캡처한다.
    """
    delete_paths = list(
        dict.fromkeys(
            captured_delete_paths or []
        )
    )

    if delete_paths:
        await delete_prims_safely(
            delete_paths,
            reason="script_start"
        )


cleanup_old_sessions()

# 반드시 async 예약 전에 기존 박스 경로를 캡처한다.
# 이 시점 이후 새로 생성되는 박스는 시작 정리 대상에 포함되지 않는다.
startup_delete_paths = collect_spawned_box_paths()

clear_stage_selection()

if startup_delete_paths:
    event_log(
        f"[시작 정리 예약] 기존 박스 "
        f"{len(startup_delete_paths)}개만 삭제 예정"
    )

asyncio.ensure_future(
    cleanup_spawned_boxes_async(
        startup_delete_paths
    )
)


# ==================================================
# [5. 상태 변수]
# ==================================================

spawned_boxes = []
last_box_pos_dict = {}
debug_count = 0
last_spawn_time = 0.0

# 첫 번째 Depth CREATE_BOX로 생성된 최초 박스만
# 첫 트레이 공급 시작 신호로 사용한다.
first_depth_spawned_box_path = None
first_depth_tray_loader_started = False

is_rejector_moving = False
is_conveyor_running = False

is_grip_attached = False
attached_box_path = None
suction_world_at_attach = Gf.Vec3d(0.0, 0.0, 0.0)
box_local_at_attach = Gf.Vec3d(0.0, 0.0, 0.0)

# 흡착 순간의 박스 yaw와 그리퍼 논리 yaw.
# 적재 위치에서 그리퍼를 0도로 돌릴 때 박스도 함께 회전시키는 기준값이다.
attached_box_yaw_at_attach = 0.0
attached_gripper_logical_yaw_at_attach = 0.0

# 흡착 중 박스 Collision 원래 상태 저장
attached_box_collision_state_dict = {}

# 흡착 직후 안정화 / 해제 후 안착 대기 컨트롤러
grip_stabilize_controller = {
    "active": False,
    "frames": 0,
    "box_path": None
}

release_settle_controller = {
    "active": False,
    "completed": False,
    "frames": 0,
    "box_path": None,
    "reference_tray_position": None,
    "reference_box_world_position": None,
    "last_tray_position": None,
    "last_sample_time": None,
    "estimated_velocity": Gf.Vec3d(0.0, 0.0, 0.0),
    "matched_velocity": Gf.Vec3d(0.0, 0.0, 0.0)
}

# 이동 중인 Tray1을 매 프레임 읽어 X1/X2/Z를 갱신하는 전용 컨트롤러
tray_follow_controller = {
    "active": False,
    "phase": "IDLE",
    "frames": 0,
    "stable_frames": 0,
    "phase_elapsed_sec": 0.0,
    "place_slot": None,
    "selected_tray_path": None,
    "previous_tray_x": None,
    "place_trigger_x": None,
    "prepare_x": None,
    "final_x": None,
    "start_tray_position": None,
    "reference_tray_position": None,
    "reference_box_center": None,
    "reference_joint_targets": None,
    "y_raw_offset": None,
    "released_box": None,
    "last_tray_position": None,
    "last_box_center": None,
    "last_alignment_error": None,
    "z_fine_offset": 0.0
}

# 1번 박스는 새 트레이의 World X=4에서 선택하고,
# Tray X=-30에서 실제 대각선 적재를 시작한다.
# 2번 박스는 1번과 동일한 트레이를 계속 사용한다.
pack_tray_trigger_controller = {
    "armed": False,
    "elapsed_sec": 0.0,
    "place_slot": None,
    "trigger_x": None,
    "previous_x_by_path": {},
    "selected_tray_path": None,
}

passed_box_count = 0
rejector_triggered_box_set = set()

is_auto_sequence_running = False
auto_sequence_done_box_set = set()

# Z 센서에서 정지한 박스의 실제 월드 좌표 저장
# key: box_path
# value: Gf.Vec3d(X, Y, Z)
stopped_box_world_position_dict = {}

# 자동 배치 순번 카운터
# 짝수(0,2,...) → 1번 박스 위치
# 홀수(1,3,...) → 2번 박스 위치
auto_place_count = 0


# ==================================================
# [6. 기본 유틸 함수]
# ==================================================

def get_stage():
    return omni.usd.get_context().get_stage()


def get_prim(path):
    stage = get_stage()
    prim = stage.GetPrimAtPath(path)

    if prim.IsValid():
        return prim

    return None


def get_world_position(path):
    prim = get_prim(path)

    if prim is None:
        return None

    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    try:
        pos = mat.ExtractTranslation()
        return Gf.Vec3d(pos[0], pos[1], pos[2])
    except:
        return Gf.Vec3d(mat[3][0], mat[3][1], mat[3][2])


def get_local_translate(path):
    prim = get_prim(path)

    if prim is None:
        return None

    xform = UsdGeom.Xformable(prim)

    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            value = op.Get()
            return Gf.Vec3d(value[0], value[1], value[2])

        if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
            mat = op.Get()

            if mat:
                try:
                    pos = mat.ExtractTranslation()
                    return Gf.Vec3d(pos[0], pos[1], pos[2])
                except:
                    return Gf.Vec3d(mat[3][0], mat[3][1], mat[3][2])

    return Gf.Vec3d(0.0, 0.0, 0.0)


def set_local_translate(path, local_pos):
    prim = get_prim(path)

    if prim is None:
        return False

    xform = UsdGeom.Xformable(prim)

    translate_op = None
    transform_op = None

    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op

        elif op.GetOpType() == UsdGeom.XformOp.TypeTransform:
            transform_op = op

    if translate_op is not None:
        translate_op.Set(Gf.Vec3d(local_pos[0], local_pos[1], local_pos[2]))
        return True

    if transform_op is not None:
        try:
            mat = transform_op.Get()

            if mat is None:
                mat = Gf.Matrix4d(1.0)

            mat.SetTranslate(Gf.Vec3d(local_pos[0], local_pos[1], local_pos[2]))
            transform_op.Set(mat)
            return True
        except Exception as e:
            debug_print("xformOp:transform 수정 실패:", e)

    try:
        new_op = xform.AddTranslateOp()
        new_op.Set(Gf.Vec3d(local_pos[0], local_pos[1], local_pos[2]))
        return True
    except Exception as e:
        debug_print("translate op 추가 실패:", e)
        return False


def world_delta_to_parent_local_delta(target_path, delta_world):
    prim = get_prim(target_path)

    if prim is None:
        return Gf.Vec3d(delta_world[0], delta_world[1], delta_world[2])

    parent = prim.GetParent()

    if not parent.IsValid():
        return Gf.Vec3d(delta_world[0], delta_world[1], delta_world[2])

    parent_xform = UsdGeom.Xformable(parent)
    parent_mat = parent_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    parent_inv = parent_mat.GetInverse()

    p0 = parent_inv.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
    p1 = parent_inv.Transform(Gf.Vec3d(delta_world[0], delta_world[1], delta_world[2]))

    local_delta = p1 - p0

    return Gf.Vec3d(local_delta[0], local_delta[1], local_delta[2])


# ==================================================
# [7. 컨베이어 속성 탐색]
# ==================================================

def find_attr_by_candidates(prim, candidates):
    for attr_name in candidates:
        attr = prim.GetAttribute(attr_name)

        if attr.IsValid():
            return attr

    return None


def find_conveyor_velocity_attr():
    prim = get_prim(CONVEYOR_PATH)

    if prim is None:
        debug_print("컨베이어 Prim 경로 오류:", CONVEYOR_PATH)
        return None

    attr = find_attr_by_candidates(prim, CONVEYOR_ATTR_CANDIDATES)

    if attr is not None:
        return attr

    for attr in prim.GetAttributes():
        name = attr.GetName()
        lower_name = name.lower()

        if "surface" in lower_name and "velocity" in lower_name:
            if "angular" not in lower_name and "enabled" not in lower_name and "local" not in lower_name:
                value = attr.Get()

                if isinstance(value, Gf.Vec3f) or isinstance(value, Gf.Vec3d):
                    debug_print("컨베이어 Surface Velocity 속성 자동 탐색 성공:", name)
                    return attr

    debug_print("컨베이어 Surface Velocity 속성을 찾지 못했습니다.")
    return None


def find_conveyor_enable_attr():
    prim = get_prim(CONVEYOR_PATH)

    if prim is None:
        return None

    attr = find_attr_by_candidates(prim, CONVEYOR_ENABLE_ATTR_CANDIDATES)

    if attr is not None:
        return attr

    for attr in prim.GetAttributes():
        name = attr.GetName()
        lower_name = name.lower()

        if "surface" in lower_name and "velocity" in lower_name and "enabled" in lower_name:
            return attr

    return None


def find_conveyor_local_space_attr():
    prim = get_prim(CONVEYOR_PATH)

    if prim is None:
        return None

    attr = find_attr_by_candidates(prim, CONVEYOR_LOCAL_SPACE_ATTR_CANDIDATES)

    if attr is not None:
        return attr

    for attr in prim.GetAttributes():
        name = attr.GetName()
        lower_name = name.lower()

        if "surface" in lower_name and "velocity" in lower_name and "local" in lower_name:
            return attr

    return None


def get_conveyor_x_velocity():
    attr = find_conveyor_velocity_attr()

    if attr is None:
        return None

    value = attr.Get()

    if isinstance(value, Gf.Vec3f) or isinstance(value, Gf.Vec3d):
        return float(value[0])

    try:
        return float(value)
    except:
        return None


def set_conveyor_x_velocity(x_value):
    attr = find_conveyor_velocity_attr()

    if attr is None:
        return False

    current_value = attr.Get()

    if isinstance(current_value, Gf.Vec3d):
        attr.Set(Gf.Vec3d(x_value, 0.0, 0.0))

    elif isinstance(current_value, Gf.Vec3f):
        attr.Set(Gf.Vec3f(x_value, 0.0, 0.0))

    else:
        try:
            attr.Set(float(x_value))
        except:
            debug_print("컨베이어 속도 설정 실패")
            return False

    enable_attr = find_conveyor_enable_attr()

    if enable_attr is not None:
        try:
            enable_attr.Set(True)
        except:
            pass

    local_attr = find_conveyor_local_space_attr()

    if local_attr is not None:
        try:
            local_attr.Set(True)
        except:
            pass

    debug_print("[컨베이어] X 속도 설정:", x_value)

    return True


# ==================================================
# [8. 박스 탐색 및 물리 복구]
# ==================================================

def get_spawned_box_root_paths_from_stage():
    stage = get_stage()
    box_paths = []

    for prim in stage.Traverse():
        path = str(prim.GetPath())

        if is_spawned_box_root_path(path):
            box_paths.append(path)

    return box_paths


def get_valid_spawned_boxes():
    valid_boxes = []

    for box_path in spawned_boxes:
        if get_prim(box_path) is not None:
            valid_boxes.append(box_path)

    if valid_boxes:
        return valid_boxes

    return get_spawned_box_root_paths_from_stage()


def get_current_target_box():
    valid_boxes = get_valid_spawned_boxes()

    if not valid_boxes:
        return None

    return valid_boxes[-1]


def get_monitor_box_paths():
    valid_boxes = get_valid_spawned_boxes()

    if valid_boxes:
        return valid_boxes

    return []


def restore_box_physics(box_path):
    prim = get_prim(box_path)

    if prim is None:
        return False

    if not prim.IsA(UsdGeom.Xformable):
        return False

    try:
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(prim)

        rb = UsdPhysics.RigidBodyAPI(prim)

        rb.CreateRigidBodyEnabledAttr().Set(True)
        rb.CreateKinematicEnabledAttr().Set(False)

        rb.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rb.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    except Exception as e:
        debug_print("박스 RigidBody 복구 실패:", box_path, e)

    try:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
    except:
        pass

    return True


def restore_all_box_physics():
    box_paths = get_monitor_box_paths()

    for box_path in box_paths:
        restore_box_physics(box_path)


# ==================================================
# [9. 좌표 판정]
# ==================================================

def is_inside_stop_target(pos):
    """
    3축 직교 로봇 앞 정지 판정.

    박스의 X 좌표는 YOLO 생성 위치에 따라 달라질 수 있으므로
    정지 조건에서 완전히 제외하고 Z 좌표만 사용한다.
    """
    if pos is None:
        return False

    current_z = float(pos[2])

    return (
        abs(current_z - STOP_TARGET_Z)
        <= STOP_TOLERANCE_Z
    )


def crossed_target_z(prev_pos, current_pos):
    """
    한 프레임 사이에 박스가 정지 Z 좌표를 지나친 경우까지 감지한다.

    예:
        이전 Z = 60
        현재 Z = 55
        목표 Z = 57.68

    위와 같이 허용범위를 건너뛰어도 센서 통과로 판정한다.
    """
    if prev_pos is None:
        return False

    if current_pos is None:
        return False

    prev_z = float(prev_pos[2])
    current_z = float(current_pos[2])

    move_min = min(prev_z, current_z)
    move_max = max(prev_z, current_z)

    return (
        move_min
        <= STOP_TARGET_Z
        <= move_max
    )


def has_reached_stop_target(prev_pos, current_pos):
    """
    현재 Z가 센서 허용범위에 들어왔거나,
    이전 프레임과 현재 프레임 사이에 목표 Z를 통과하면 True.
    """
    if current_pos is None:
        return False

    if is_inside_stop_target(current_pos):
        return True

    if crossed_target_z(prev_pos, current_pos):
        return True

    return False


def is_inside_rejector_trigger(pos):
    """
    현재 박스의 월드 Z가 리젝터 기준 Z 허용 범위 안인지 확인한다.
    X/Y는 판정에 사용하지 않는다.
    """
    if pos is None:
        return False

    return (
        abs(
            float(pos[2])
            - float(REJECTOR_TRIGGER_Z)
        )
        <= float(
            REJECTOR_TRIGGER_TOLERANCE_Z
        )
    )


def crossed_rejector_trigger_z(
    prev_pos,
    current_pos
):
    """
    박스가 한 프레임 사이에 Z=611.18216 기준선을 지나친 경우까지 감지한다.

    컨베이어 이동 방향이 반대로 바뀌어도 두 프레임 사이 범위를
    확인하므로 기준선을 통과하면 True가 된다.
    """
    if prev_pos is None or current_pos is None:
        return False

    prev_z = float(
        prev_pos[2]
    )
    current_z = float(
        current_pos[2]
    )

    move_min = min(
        prev_z,
        current_z
    )
    move_max = max(
        prev_z,
        current_z
    )

    return (
        move_min
        <= float(REJECTOR_TRIGGER_Z)
        <= move_max
    )


def has_passed_rejector_trigger(
    prev_pos,
    current_pos
):
    """
    현재 Z가 허용 범위 안이거나 이전/현재 프레임 사이에
    기준 Z를 통과하면 True.
    """
    if current_pos is None:
        return False

    if is_inside_rejector_trigger(
        current_pos
    ):
        return True

    return crossed_rejector_trigger_z(
        prev_pos,
        current_pos
    )


# ==================================================
# [10. 컨베이어 시작/정지/재가동/토글]
# ==================================================

def start_conveyor():
    global is_conveyor_running
    global last_box_pos_dict
    global debug_count

    restore_all_box_physics()

    last_box_pos_dict.clear()
    debug_count = 0

    for box_path in get_monitor_box_paths():
        pos = get_world_position(box_path)
        last_box_pos_dict[box_path] = pos

    set_conveyor_x_velocity(CONVEYOR_RUN_X_SPEED)
    is_conveyor_running = True

    debug_print("==================================================")
    debug_print("[컨베이어 시작]")
    debug_print(f"Z 센서 정지 좌표: Z={STOP_TARGET_Z} ± {STOP_TOLERANCE_Z}")
    debug_print("==================================================")


def resume_conveyor_after_sequence():
    global is_conveyor_running
    global last_box_pos_dict
    global debug_count

    restore_all_box_physics()

    last_box_pos_dict.clear()
    debug_count = 0

    for box_path in get_monitor_box_paths():
        pos = get_world_position(box_path)
        last_box_pos_dict[box_path] = pos

    set_conveyor_x_velocity(CONVEYOR_STOP_X_SPEED)
    set_conveyor_x_velocity(CONVEYOR_RUN_X_SPEED)

    is_conveyor_running = True

    debug_print("==================================================")
    debug_print("[컨베이어 재가동]")
    debug_print("자동 시퀀스 완료 후 컨베이어 X 속도 -10 재가동")
    debug_print("이미 처리된 박스는 자동 정지 감시에서 제외됩니다.")
    debug_print("==================================================")

def resume_conveyor_after_pick():
    """
    현재 박스가 Hard-Lock 흡착되어 검사 컨베이어에서 사라진 직후,
    첫 적재가 끝날 때까지 기다리지 않고 다음 박스를 픽업 위치로 보낸다.

    주의:
    resume_conveyor_after_sequence()와 달리 restore_all_box_physics()를 호출하지 않는다.
    흡착 중인 박스의 RigidBody/Collision을 다시 켜면 박스가 떨어지기 때문이다.
    """
    global is_conveyor_running
    global last_box_pos_dict
    global debug_count

    last_box_pos_dict.clear()
    debug_count = 0

    for box_path in get_monitor_box_paths():
        if box_path == attached_box_path:
            continue

        pos = get_world_position(box_path)

        if pos is not None:
            last_box_pos_dict[box_path] = copy_world_position(pos)

    set_conveyor_x_velocity(CONVEYOR_STOP_X_SPEED)
    set_conveyor_x_velocity(CONVEYOR_RUN_X_SPEED)
    is_conveyor_running = True

    event_log(
        f"[검사 컨베이어 즉시 재가동] "
        f"흡착 박스가 검사 위치에서 제거됨 | "
        f"속도={CONVEYOR_RUN_X_SPEED:.1f} | "
        "다음 박스는 정지 좌표에서 대기"
    )

    return True


# ==================================================
# [v39 흡착 박스 Y 이동 감지 기반 다음 박스 사전 이송]
# ==================================================
# 흡착 성공 직후에는 검사 컨베이어를 아직 움직이지 않는다.
# 흡착된 박스의 World Y가 흡착 기준점에서 실제로 변한 순간,
# 즉 그리퍼가 박스를 들어 올리기 시작한 것이 확인되면 컨베이어를 재가동한다.
# 다음 정상 박스가 STOP_TARGET_Z에 도착하면 로봇이 적재 중이어도 정지시킨다.
# v46.2 기본 동작:
# True이면 흡착 박스의 Y 상승을 기다리지 않고,
# Hard-Lock 흡착 성공 프레임에 검사 컨베이어를 즉시 재가동한다.
NEXT_BOX_PRELOAD_IMMEDIATE_AFTER_ATTACH = True

NEXT_BOX_PRELOAD_Y_CHANGE_THRESHOLD = 0.5
NEXT_BOX_PRELOAD_AXIS_Y_CHANGE_THRESHOLD = 0.25

# v46.5:
# 흡착 직후에는 박스 사이가 너무 가까워 접촉할 수 있으므로
# 이 시간 동안 검사 컨베이어를 정지한 뒤 다음 박스를 선이송한다.
NEXT_BOX_FEED_START_DELAY_SEC = 1.5

# Y=50, Z=-270 단계의 위치 도달 허용 오차
POST_PICK_Y_GEOMETRY_TOLERANCE = 3.0
POST_PICK_Z_GEOMETRY_TOLERANCE = 3.0

next_box_preload_active = False
next_box_preload_source_box = None
next_box_preload_log_latched = False
next_box_preload_attach_world_y = None
next_box_preload_attach_suction_world_y = None
next_box_preload_attach_axis_y = None
next_box_preload_conveyor_started = False


# ==================================================
# [v46.4 강제 다음 박스 선이송 컨트롤러]
# ==================================================
# 기존 next_box_preload 함수와 별도로 실제 컨베이어 속도를 매 프레임
# 확인하고 강제로 유지한다. 현재 흡착 박스는 물리를 건드리지 않고,
# 나머지 박스만 정상 RigidBody 상태로 복구한다.
forced_next_box_feed_controller = {
    "active": False,
    "source_box": None,
    "phase": "IDLE",
    "delay_sec": NEXT_BOX_FEED_START_DELAY_SEC,
    "delay_elapsed_sec": 0.0,
    "delay_start_time": None,
    "speed_confirmed": False,
    "retry_frames": 0,
    "waiting_box": None,
}


# ==================================================
# [v46.7 픽업 후 Y=50 -> Z=-270 강제 파이프라인]
# ==================================================
post_pick_axis_pipeline = {
    "active": False,
    "box_path": None,
    "phase": "IDLE",
    "axis": None,
    "drive_start": None,
    "drive_target": None,
    "geometry_start": None,
    "geometry_target": None,
    "geometry_tolerance": None,
    "retry_frames": 0,
}


def reset_next_box_preload():
    global next_box_preload_active
    global next_box_preload_source_box
    global next_box_preload_log_latched
    global next_box_preload_attach_world_y
    global next_box_preload_attach_suction_world_y
    global next_box_preload_attach_axis_y
    global next_box_preload_conveyor_started

    next_box_preload_active = False
    next_box_preload_source_box = None
    next_box_preload_log_latched = False
    next_box_preload_attach_world_y = None
    next_box_preload_attach_suction_world_y = None
    next_box_preload_attach_axis_y = None
    next_box_preload_conveyor_started = False


def get_box_preload_world_y(box_path):
    """흡착 박스의 실제 World Y를 읽는다. Xform 위치를 우선 사용한다."""
    if box_path is None or get_prim(box_path) is None:
        return None

    position = get_world_position(box_path)

    if position is not None:
        return float(position[1])

    center = get_prim_world_bbox_center(box_path)

    if center is not None:
        return float(center[1])

    return None


def arm_next_box_preload(source_box_path):
    """
    흡착 박스의 실제 상승을 감시한다. 흡착 직후에는 컨베이어를 정지하고,
    박스/흡착점/Y축 중 하나라도 움직이면 다음 박스 이송을 시작한다.
    """
    global next_box_preload_active
    global next_box_preload_source_box
    global next_box_preload_log_latched
    global next_box_preload_attach_world_y
    global next_box_preload_attach_suction_world_y
    global next_box_preload_attach_axis_y
    global next_box_preload_conveyor_started

    next_box_preload_active = True
    next_box_preload_source_box = source_box_path
    next_box_preload_log_latched = False
    next_box_preload_attach_world_y = get_box_preload_world_y(source_box_path)

    suction_position = get_suction_contact_position()
    next_box_preload_attach_suction_world_y = (
        float(suction_position[1])
        if suction_position is not None
        else None
    )

    next_box_preload_attach_axis_y = get_axis_actual_position("Y")
    next_box_preload_conveyor_started = False

    # 실제 상승이 확인되기 전까지 현재 박스를 안정적으로 유지한다.
    set_conveyor_x_velocity(CONVEYOR_STOP_X_SPEED)

    return True


def reset_forced_next_box_feed():
    forced_next_box_feed_controller[
        "active"
    ] = False
    forced_next_box_feed_controller[
        "source_box"
    ] = None
    forced_next_box_feed_controller[
        "phase"
    ] = "IDLE"
    forced_next_box_feed_controller[
        "delay_sec"
    ] = float(
        NEXT_BOX_FEED_START_DELAY_SEC
    )
    forced_next_box_feed_controller[
        "delay_elapsed_sec"
    ] = 0.0
    forced_next_box_feed_controller[
        "delay_start_time"
    ] = None
    forced_next_box_feed_controller[
        "speed_confirmed"
    ] = False
    forced_next_box_feed_controller[
        "retry_frames"
    ] = 0
    forced_next_box_feed_controller[
        "waiting_box"
    ] = None


def _restore_non_attached_boxes_for_feed(
    source_box_path
):
    """
    흡착 박스를 제외한 검사 컨베이어 위 박스만 물리 복구한다.
    흡착 박스의 RigidBody/Collision은 절대 다시 켜지 않는다.
    """
    for box_path in get_monitor_box_paths():
        if box_path == source_box_path:
            continue

        if get_prim(
            box_path
        ) is None:
            continue

        restore_box_physics(
            box_path
        )


def _refresh_non_attached_box_positions(
    source_box_path
):
    global debug_count

    last_box_pos_dict.clear()
    debug_count = 0

    for box_path in get_monitor_box_paths():
        if box_path == source_box_path:
            continue

        current_position = get_world_position(
            box_path
        )

        if current_position is not None:
            last_box_pos_dict[
                box_path
            ] = copy_world_position(
                current_position
            )


def force_start_next_box_feed(
    source_box_path
):
    """
    현재 박스를 흡착한 뒤 다음 박스 선이송을 예약한다.

    v46.5:
    - 흡착 직후에는 검사 컨베이어를 정지 상태로 유지한다.
    - NEXT_BOX_FEED_START_DELAY_SEC 경과 후에만 -65로 재가동한다.
    - 컨베이어 지연과 Y=50 -> Z=-270 로봇 이동은 서로 독립적이다.
    """
    global is_conveyor_running
    global next_box_preload_active
    global next_box_preload_source_box
    global next_box_preload_conveyor_started
    global next_box_preload_log_latched

    reset_forced_next_box_feed()

    forced_next_box_feed_controller[
        "active"
    ] = True
    forced_next_box_feed_controller[
        "source_box"
    ] = source_box_path
    forced_next_box_feed_controller[
        "phase"
    ] = "WAIT_DELAY"
    forced_next_box_feed_controller[
        "delay_sec"
    ] = float(
        NEXT_BOX_FEED_START_DELAY_SEC
    )
    forced_next_box_feed_controller[
        "delay_elapsed_sec"
    ] = 0.0
    forced_next_box_feed_controller[
        "delay_start_time"
    ] = time.perf_counter()

    # 기존 대기 박스 감시 상태와 동기화한다.
    next_box_preload_active = True
    next_box_preload_source_box = source_box_path
    next_box_preload_conveyor_started = False
    next_box_preload_log_latched = False

    # 흡착 직후에는 반드시 정지시킨다.
    set_conveyor_x_velocity(
        CONVEYOR_STOP_X_SPEED
    )
    is_conveyor_running = False

    _refresh_non_attached_box_positions(
        source_box_path
    )

    event_log(
        f"[그리퍼 이동] 흡착 후 검사 컨베이어 지연 시작 | "
        f"ID={box_path_id_dict.get(source_box_path, None)} | "
        f"지연={NEXT_BOX_FEED_START_DELAY_SEC:.2f}초 | "
        "박스 간 접촉 방지를 위해 정지 유지"
    )

    return True


def update_forced_next_box_feed(
    event=None
):
    """
    WAIT_DELAY:
        흡착 후 1.5초 동안 컨베이어 0 유지

    FEEDING:
        다음 정상 박스가 픽업 위치에 도착할 때까지 -65 유지

    다음 박스가 정지선에 도착하면 다시 0으로 정지하고,
    현재 박스 적재가 끝날 때까지 대기시킨다.
    """
    global is_conveyor_running
    global next_box_preload_active
    global next_box_preload_conveyor_started

    if not forced_next_box_feed_controller.get(
        "active",
        False
    ):
        return False

    source_box = forced_next_box_feed_controller.get(
        "source_box"
    )

    waiting_box = get_pending_stopped_pick_box()

    if (
        waiting_box is not None
        and waiting_box != source_box
    ):
        set_conveyor_x_velocity(
            CONVEYOR_STOP_X_SPEED
        )
        is_conveyor_running = False

        forced_next_box_feed_controller[
            "waiting_box"
        ] = waiting_box
        forced_next_box_feed_controller[
            "phase"
        ] = "WAITING_AT_PICK"
        forced_next_box_feed_controller[
            "active"
        ] = False

        next_box_preload_active = False
        next_box_preload_conveyor_started = False

        event_log(
            f"[그리퍼 이동] 다음 정상 박스 픽업 위치 도착/대기 | "
            f"ID={box_path_id_dict.get(waiting_box, None)} | "
            f"Z={STOP_TARGET_Z:.2f} | "
            "현재 박스 적재 후 HOME 복귀 즉시 픽업"
        )

        return True

    phase = forced_next_box_feed_controller.get(
        "phase",
        "IDLE"
    )

    if phase == "WAIT_DELAY":
        try:
            dt = float(
                event.payload.get(
                    "dt",
                    0.0
                )
            )
        except Exception:
            dt = 0.0

        forced_next_box_feed_controller[
            "delay_elapsed_sec"
        ] += max(
            0.0,
            dt
        )

        start_time = forced_next_box_feed_controller.get(
            "delay_start_time"
        )

        if start_time is not None:
            wall_elapsed = max(
                0.0,
                time.perf_counter()
                - float(start_time)
            )
        else:
            wall_elapsed = 0.0

        elapsed = max(
            float(
                forced_next_box_feed_controller.get(
                    "delay_elapsed_sec",
                    0.0
                )
            ),
            float(wall_elapsed)
        )

        # 지연 중에는 다른 코드가 켜더라도 다시 0으로 고정한다.
        set_conveyor_x_velocity(
            CONVEYOR_STOP_X_SPEED
        )
        is_conveyor_running = False

        if elapsed < float(
            forced_next_box_feed_controller.get(
                "delay_sec",
                NEXT_BOX_FEED_START_DELAY_SEC
            )
        ):
            return True

        _restore_non_attached_boxes_for_feed(
            source_box
        )
        _refresh_non_attached_box_positions(
            source_box
        )

        # 정지값에서 확실히 깨어나도록 0 -> -65 펄스
        set_conveyor_x_velocity(
            CONVEYOR_STOP_X_SPEED
        )
        run_ok = set_conveyor_x_velocity(
            CONVEYOR_RUN_X_SPEED
        )

        readback = get_conveyor_x_velocity()

        speed_confirmed = (
            readback is not None
            and abs(
                float(readback)
                - float(CONVEYOR_RUN_X_SPEED)
            )
            <= 0.01
        )

        forced_next_box_feed_controller[
            "phase"
        ] = "FEEDING"
        forced_next_box_feed_controller[
            "speed_confirmed"
        ] = bool(
            speed_confirmed
        )

        next_box_preload_conveyor_started = True
        is_conveyor_running = bool(
            run_ok
        )

        event_log(
            f"[그리퍼 이동] 흡착 후 지연 완료, "
            f"다음 박스 선이송 시작 | "
            f"ID={box_path_id_dict.get(source_box, None)} | "
            f"지연={elapsed:.2f}초 | "
            f"명령={CONVEYOR_RUN_X_SPEED:.1f} | "
            f"읽기={readback}"
        )

        return True

    if phase != "FEEDING":
        return False

    _restore_non_attached_boxes_for_feed(
        source_box
    )

    current_speed = get_conveyor_x_velocity()

    speed_wrong = (
        current_speed is None
        or abs(
            float(current_speed)
            - float(CONVEYOR_RUN_X_SPEED)
        )
        > 0.01
    )

    if speed_wrong:
        set_conveyor_x_velocity(
            CONVEYOR_STOP_X_SPEED
        )
        set_conveyor_x_velocity(
            CONVEYOR_RUN_X_SPEED
        )

        forced_next_box_feed_controller[
            "retry_frames"
        ] += 1

    current_speed = get_conveyor_x_velocity()

    forced_next_box_feed_controller[
        "speed_confirmed"
    ] = (
        current_speed is not None
        and abs(
            float(current_speed)
            - float(CONVEYOR_RUN_X_SPEED)
        )
        <= 0.01
    )

    is_conveyor_running = True

    return True


def reset_post_pick_axis_pipeline():
    post_pick_axis_pipeline[
        "active"
    ] = False
    post_pick_axis_pipeline[
        "box_path"
    ] = None
    post_pick_axis_pipeline[
        "phase"
    ] = "IDLE"
    post_pick_axis_pipeline[
        "axis"
    ] = None
    post_pick_axis_pipeline[
        "drive_start"
    ] = None
    post_pick_axis_pipeline[
        "drive_target"
    ] = None
    post_pick_axis_pipeline[
        "geometry_start"
    ] = None
    post_pick_axis_pipeline[
        "geometry_target"
    ] = None
    post_pick_axis_pipeline[
        "geometry_tolerance"
    ] = None
    post_pick_axis_pipeline[
        "retry_frames"
    ] = 0
    post_pick_axis_pipeline[
        "place_slot"
    ] = None
    post_pick_axis_pipeline[
        "transport_x_target"
    ] = None


def calculate_relative_geometry_target(
    axis_name,
    drive_target
):
    """
    현재 Drive 값과 실제 Geometry raw 값의 차이를 기준으로,
    새 Drive 목표에 해당하는 실제 Geometry 목표를 계산한다.

        geometry_target
        = geometry_start
        + (drive_target - drive_start)

    기존 코드처럼 Drive targetPosition만 50/240에 도달했다고
    실제 축 이동이 끝난 것으로 판단하지 않는다.
    """
    drive_start = get_axis_target_position(
        axis_name
    )
    geometry_start = get_axis_raw_geometry_position(
        axis_name
    )

    if (
        drive_start is None
        or geometry_start is None
    ):
        return None

    drive_target = clamp_axis_target(
        axis_name,
        drive_target
    )

    geometry_target = (
        float(geometry_start)
        + (
            float(drive_target)
            - float(drive_start)
        )
    )

    return {
        "axis": axis_name,
        "drive_start": float(drive_start),
        "drive_target": float(drive_target),
        "geometry_start": float(geometry_start),
        "geometry_target": float(geometry_target),
    }


def set_post_pick_axis_phase(
    box_path,
    phase,
    axis_name,
    drive_target,
    geometry_tolerance
):
    target_info = calculate_relative_geometry_target(
        axis_name,
        drive_target
    )

    if target_info is None:
        event_log(
            f"[그리퍼 이동 오류] {phase} | "
            f"{axis_name} 실제 Geometry 판독 실패"
        )
        return False

    post_pick_axis_pipeline[
        "active"
    ] = True
    post_pick_axis_pipeline[
        "box_path"
    ] = box_path
    post_pick_axis_pipeline[
        "phase"
    ] = phase
    post_pick_axis_pipeline[
        "axis"
    ] = axis_name
    post_pick_axis_pipeline[
        "drive_start"
    ] = target_info[
        "drive_start"
    ]
    post_pick_axis_pipeline[
        "drive_target"
    ] = target_info[
        "drive_target"
    ]
    post_pick_axis_pipeline[
        "geometry_start"
    ] = target_info[
        "geometry_start"
    ]
    post_pick_axis_pipeline[
        "geometry_target"
    ] = target_info[
        "geometry_target"
    ]
    post_pick_axis_pipeline[
        "geometry_tolerance"
    ] = float(
        geometry_tolerance
    )
    post_pick_axis_pipeline[
        "retry_frames"
    ] = 0

    return True


def is_post_pick_geometry_reached():
    axis_name = post_pick_axis_pipeline.get(
        "axis"
    )
    geometry_start = post_pick_axis_pipeline.get(
        "geometry_start"
    )
    geometry_target = post_pick_axis_pipeline.get(
        "geometry_target"
    )
    drive_start = post_pick_axis_pipeline.get(
        "drive_start"
    )
    drive_target = post_pick_axis_pipeline.get(
        "drive_target"
    )
    tolerance = post_pick_axis_pipeline.get(
        "geometry_tolerance"
    )

    if (
        axis_name is None
        or geometry_start is None
        or geometry_target is None
        or drive_start is None
        or drive_target is None
        or tolerance is None
    ):
        return False

    geometry_now = get_axis_raw_geometry_position(
        axis_name
    )

    if geometry_now is None:
        return False

    direction = (
        float(drive_target)
        - float(drive_start)
    )

    if direction > 0.0:
        return (
            float(geometry_now)
            >= float(geometry_target)
            - float(tolerance)
        )

    if direction < 0.0:
        return (
            float(geometry_now)
            <= float(geometry_target)
            + float(tolerance)
        )

    return (
        abs(
            float(geometry_now)
            - float(geometry_target)
        )
        <= float(tolerance)
    )


def start_post_pick_y50_pipeline(
    box_path
):
    """
    흡착 후 Y Station의 Drive targetPosition을 50으로 이동한다.

    v46.5의 Geometry 상대 목표 계산은 실제 Stage에서 완료 판정이
    걸려 다음 단계가 시작되지 않았으므로 제거한다.
    """
    reset_post_pick_axis_pipeline()

    post_pick_axis_pipeline[
        "active"
    ] = True
    post_pick_axis_pipeline[
        "box_path"
    ] = box_path
    post_pick_axis_pipeline[
        "phase"
    ] = "Y50"
    post_pick_axis_pipeline[
        "axis"
    ] = "Y"
    post_pick_axis_pipeline[
        "drive_target"
    ] = float(
        AUTO_PICK_Y_UP_POSITION
    )
    post_pick_axis_pipeline[
        "retry_frames"
    ] = 0

    set_robot_sequence_state(
        "LIFT_Y"
    )

    started = start_scurve_motion(
        "픽업 후 Y 상승",
        {
            "Y": AUTO_PICK_Y_UP_POSITION
        }
    )

    event_log(
        f"[그리퍼 이동] 픽업 후 Y 상승 시작 | "
        f"ID={box_path_id_dict.get(box_path, None)} | "
        f"Drive 목표={AUTO_PICK_Y_UP_POSITION:.2f} | "
        f"시작={'성공' if started else '실패'}"
    )

    if not started:
        abort_robot_sequence(
            "픽업 후 Y 상승 시작 실패"
        )

    return started


def get_post_pick_transport_x_target():
    """
    현재 박스의 적재 순번에 맞는 X1/X2 선이동 목표를 반환한다.

    1번 박스: X1/X2 = 110
    2번 박스: X1/X2 = -70
    """
    try:
        place_slot = int(
            robot_sequence.get(
                "place_slot",
                1 if int(auto_place_count) % 2 == 0 else 2
            )
        )
    except Exception:
        place_slot = 1 if int(auto_place_count) % 2 == 0 else 2

    if place_slot not in (1, 2):
        place_slot = 1 if int(auto_place_count) % 2 == 0 else 2

    config = TRAY_PLACE_SLOT_CONFIG[place_slot]

    return (
        int(place_slot),
        float(config["prepare_x"])
    )


def start_post_pick_z_neg270_pipeline(
    box_path
):
    """
    Y Station Drive=50 이동 완료 직후 적재 순번에 따라
    Z=-270과 X1/X2 선이동을 하나의 동기 S-Curve로 실행한다.

    1번 박스:
        X1/X2 -> 110, Z -> -270

    2번 박스:
        X1/X2 -> -70, Z -> -270
    """
    place_slot, transport_x_target = (
        get_post_pick_transport_x_target()
    )

    post_pick_axis_pipeline[
        "active"
    ] = True
    post_pick_axis_pipeline[
        "box_path"
    ] = box_path
    post_pick_axis_pipeline[
        "phase"
    ] = "Z_NEG270_X_TRANSPORT"
    post_pick_axis_pipeline[
        "axis"
    ] = "X1/X2/Z"
    post_pick_axis_pipeline[
        "drive_target"
    ] = float(
        AUTO_POST_PICK_Z_POSITION
    )
    post_pick_axis_pipeline[
        "place_slot"
    ] = int(place_slot)
    post_pick_axis_pipeline[
        "transport_x_target"
    ] = float(transport_x_target)
    post_pick_axis_pipeline[
        "retry_frames"
    ] = 0

    set_robot_sequence_state(
        "POST_PICK_Z_NEG270"
    )

    started = start_scurve_motion(
        f"{place_slot}번 적재 선이동 X={transport_x_target:.1f} / Z=-270",
        {
            "X1": transport_x_target,
            "X2": transport_x_target,
            "Z": AUTO_POST_PICK_Z_POSITION
        },
        synchronized=True
    )

    event_log(
        f"[그리퍼 이동] Y=50 도착, "
        f"{place_slot}번 적재 선이동 시작 | "
        f"ID={box_path_id_dict.get(box_path, None)} | "
        f"X1/X2 목표={transport_x_target:.2f} | "
        f"Z 목표={AUTO_POST_PICK_Z_POSITION:.2f} | "
        f"동기 S-Curve | "
        f"시작={'성공' if started else '실패'}"
    )

    if not started:
        abort_robot_sequence(
            f"Y=50 완료 후 {place_slot}번 X/Z 선이동 시작 실패"
        )

    return started


def finish_post_pick_z_neg270_pipeline(
    box_path
):
    """
    Z=-270 완료 후 트레이 적재 준비 회전으로 넘어간다.
    """
    completed_place_slot = post_pick_axis_pipeline.get(
        "place_slot"
    )

    reset_post_pick_axis_pipeline()

    set_robot_sequence_state(
        "GRIPPER_ROTATE_PLACE_HOME"
    )

    y_drive = get_axis_target_position(
        "Y"
    )
    x1_drive = get_axis_target_position(
        "X1"
    )
    x2_drive = get_axis_target_position(
        "X2"
    )
    z_drive = get_axis_target_position(
        "Z"
    )
    event_log(
        f"[그리퍼 이동] Y=50/X/Z 선이동 완료 | "
        f"ID={box_path_id_dict.get(box_path, None)} | "
        f"적재순번={completed_place_slot} | "
        f"Y Drive={y_drive} | "
        f"X1 Drive={x1_drive} | "
        f"X2 Drive={x2_drive} | "
        f"Z Drive={z_drive} | "
        "그리퍼 0도 복귀 시작"
    )

    started = start_gripper_joint_motion(
        "이동 트레이 적재 전 0도 복귀",
        GRIPPER_JOINT_HOME_LOGICAL_DEG
    )

    if not started:
        abort_robot_sequence(
            "Z=-270 완료 후 그리퍼 0도 복귀 실패"
        )

    return started


def update_post_pick_axis_pipeline_watchdog():
    """
    Y=50 또는 Z=-270 상태인데 모션이 비정상적으로 사라진 경우에만
    동일 Drive 목표를 다시 명령한다.

    단계 전환은 handle_robot_motion_complete()에서만 수행한다.
    """
    if not post_pick_axis_pipeline.get(
        "active",
        False
    ):
        return False

    if not is_robot_sequence_locked():
        reset_post_pick_axis_pipeline()
        return False

    if motion_controller.get(
        "active",
        False
    ):
        return True

    if gripper_joint_controller.get(
        "active",
        False
    ):
        return True

    state = robot_sequence.get(
        "state",
        "IDLE"
    )
    box_path = post_pick_axis_pipeline.get(
        "box_path"
    )

    if (
        box_path is None
        or get_prim(
            box_path
        ) is None
    ):
        abort_robot_sequence(
            "Y/Z 이동 대상 박스 없음"
        )
        return False

    post_pick_axis_pipeline[
        "retry_frames"
    ] += 1

    # 완료 콜백과 같은 프레임의 재시작을 막기 위해
    # 3프레임 이상 모션이 비어 있을 때만 재명령한다.
    if post_pick_axis_pipeline[
        "retry_frames"
    ] < 3:
        return False

    post_pick_axis_pipeline[
        "retry_frames"
    ] = 0

    if state == "LIFT_Y":
        return start_scurve_motion(
            "픽업 후 Y 상승 watchdog 재시도",
            {
                "Y": AUTO_PICK_Y_UP_POSITION
            }
        )

    if state == "POST_PICK_Z_NEG270":
        place_slot, transport_x_target = (
            get_post_pick_transport_x_target()
        )

        post_pick_axis_pipeline[
            "place_slot"
        ] = int(place_slot)
        post_pick_axis_pipeline[
            "transport_x_target"
        ] = float(transport_x_target)

        return start_scurve_motion(
            f"{place_slot}번 X/Z 선이동 watchdog 재시도",
            {
                "X1": transport_x_target,
                "X2": transport_x_target,
                "Z": AUTO_POST_PICK_Z_POSITION
            },
            synchronized=True
        )

    return False


def start_next_box_preload_immediately(source_box_path):
    """
    현재 박스가 Hard-Lock으로 흡착된 바로 그 프레임에
    검사 컨베이어를 -65로 재가동한다.

    start_conveyor()는 restore_all_box_physics()를 호출하므로 흡착 박스의
    RigidBody/Collision까지 복구할 수 있다. 따라서 이 함수는 흡착 박스는
    건드리지 않고 나머지 박스만 이동시키는 전용 재가동 로직을 사용한다.
    """
    if not NEXT_BOX_PRELOAD_IMMEDIATE_AFTER_ATTACH:
        return False

    if source_box_path is None:
        return False

    if get_prim(source_box_path) is None:
        return False

    if (
        not is_grip_attached
        or attached_box_path != source_box_path
    ):
        event_log(
            f"[그리퍼 이동] 다음 박스 선이송 취소 | "
            f"ID={box_path_id_dict.get(source_box_path, None)} | "
            "Hard-Lock 흡착 상태 확인 실패"
        )
        return False
    global next_box_preload_active
    global next_box_preload_source_box
    global next_box_preload_log_latched
    global next_box_preload_attach_world_y
    global next_box_preload_attach_suction_world_y
    global next_box_preload_attach_axis_y
    global next_box_preload_conveyor_started
    global is_conveyor_running
    global debug_count

    reset_next_box_preload()

    next_box_preload_active = True
    next_box_preload_source_box = source_box_path
    next_box_preload_log_latched = False
    next_box_preload_conveyor_started = True

    next_box_preload_attach_world_y = get_box_preload_world_y(source_box_path)

    suction_position = get_suction_contact_position()
    next_box_preload_attach_suction_world_y = (
        float(suction_position[1])
        if suction_position is not None
        else None
    )
    next_box_preload_attach_axis_y = get_axis_actual_position("Y")

    # 이전 프레임 위치가 오래되어 정지선을 잘못 통과했다고 판단하지 않도록
    # 흡착 박스를 제외한 모든 박스의 기준 좌표를 현재 좌표로 갱신한다.
    last_box_pos_dict.clear()
    debug_count = 0

    for box_path in get_monitor_box_paths():
        if box_path == source_box_path:
            continue

        current_position = get_world_position(box_path)

        if current_position is not None:
            last_box_pos_dict[box_path] = copy_world_position(current_position)

    set_conveyor_x_velocity(CONVEYOR_RUN_X_SPEED)
    is_conveyor_running = True

    event_log(
        f"[그리퍼 이동] 흡착 직후 검사 컨베이어 즉시 재가동 | "
        f"ID={box_path_id_dict.get(source_box_path, None)} | "
        f"검사 컨베이어={CONVEYOR_RUN_X_SPEED:.1f} | "
        "다음 박스 정지 위치까지 선이송"
    )

    return True


def update_next_box_preload_y_trigger():
    """
    흡착 박스의 World Y, 흡착점 World Y, Y축 실제 위치를 함께 감시한다.
    어느 하나라도 기준값에서 변하면 검사 컨베이어를 즉시 -65로 재가동한다.
    """
    global next_box_preload_attach_world_y
    global next_box_preload_attach_suction_world_y
    global next_box_preload_attach_axis_y
    global next_box_preload_conveyor_started
    global is_conveyor_running

    if not next_box_preload_active or next_box_preload_conveyor_started:
        return False

    source_box = next_box_preload_source_box

    if source_box is None:
        return False

    if not is_grip_attached or attached_box_path != source_box:
        return False

    current_box_y = get_box_preload_world_y(source_box)
    current_suction_position = get_suction_contact_position()
    current_suction_y = (
        float(current_suction_position[1])
        if current_suction_position is not None
        else None
    )
    current_axis_y = get_axis_actual_position("Y")

    box_delta = None
    suction_delta = None
    axis_delta = None

    if current_box_y is not None and next_box_preload_attach_world_y is not None:
        box_delta = float(current_box_y) - float(next_box_preload_attach_world_y)

    if (
        current_suction_y is not None
        and next_box_preload_attach_suction_world_y is not None
    ):
        suction_delta = (
            float(current_suction_y)
            - float(next_box_preload_attach_suction_world_y)
        )

    if current_axis_y is not None and next_box_preload_attach_axis_y is not None:
        axis_delta = float(current_axis_y) - float(next_box_preload_attach_axis_y)

    box_moved = (
        box_delta is not None
        and abs(float(box_delta)) >= float(NEXT_BOX_PRELOAD_Y_CHANGE_THRESHOLD)
    )
    suction_moved = (
        suction_delta is not None
        and abs(float(suction_delta)) >= float(NEXT_BOX_PRELOAD_Y_CHANGE_THRESHOLD)
    )
    axis_moved = (
        axis_delta is not None
        and abs(float(axis_delta))
        >= float(NEXT_BOX_PRELOAD_AXIS_Y_CHANGE_THRESHOLD)
    )

    if not (box_moved or suction_moved or axis_moved):
        return False

    # 재가동 직전 모든 비흡착 박스의 기준 좌표를 현재값으로 갱신하여
    # 오래된 prev_pos 때문에 정지선을 즉시 잘못 통과 판정하는 것을 막는다.
    for box_path in get_monitor_box_paths():
        if box_path == source_box:
            continue

        current_position = get_world_position(box_path)

        if current_position is not None:
            last_box_pos_dict[box_path] = copy_world_position(current_position)

    set_conveyor_x_velocity(CONVEYOR_RUN_X_SPEED)
    is_conveyor_running = True
    next_box_preload_conveyor_started = True

    event_log(
        f"[그리퍼 이동] 흡착 박스 Y 변화 감지 | "
        f"ID={box_path_id_dict.get(source_box, None)} | "
        f"Box ΔY={box_delta if box_delta is not None else 'None'} | "
        f"Suction ΔY={suction_delta if suction_delta is not None else 'None'} | "
        f"Axis ΔY={axis_delta if axis_delta is not None else 'None'} | "
        f"검사 컨베이어={CONVEYOR_RUN_X_SPEED:.1f} 재가동"
    )

    return True


def finish_next_box_preload_at_stop(waiting_box_path):
    global next_box_preload_active
    global next_box_preload_log_latched

    if not next_box_preload_active:
        return False

    if waiting_box_path == next_box_preload_source_box:
        return False

    next_box_preload_active = False

    if not next_box_preload_log_latched:
        event_log(
            f"[그리퍼 이동] 다음 박스 픽업 위치 대기 완료 | "
            f"ID={box_path_id_dict.get(waiting_box_path, None)} | "
            f"Z={STOP_TARGET_Z:.1f} | "
            "현재 박스 적재 완료 후 즉시 픽업"
        )
        next_box_preload_log_latched = True

    return True


def maintain_next_box_preload_conveyor():
    """
    Y 변화 감지 후 시작된 검사 컨베이어가 다음 박스 도착 전까지
    다른 코드에 의해 0으로 바뀌는 것을 막는다.
    단, 다음 박스가 이미 정지 위치에 대기 중이면 절대 다시 켜지 않는다.
    """
    global is_conveyor_running
    global next_box_preload_active

    if not next_box_preload_active:
        return False

    waiting_box = get_pending_stopped_pick_box()

    if (
        waiting_box is not None
        and waiting_box != next_box_preload_source_box
    ):
        set_conveyor_x_velocity(CONVEYOR_STOP_X_SPEED)
        is_conveyor_running = False
        finish_next_box_preload_at_stop(waiting_box)
        return True

    # 아직 흡착 박스의 Y 이동이 감지되지 않았다면 정지 상태를 유지한다.
    if not next_box_preload_conveyor_started:
        set_conveyor_x_velocity(CONVEYOR_STOP_X_SPEED)
        is_conveyor_running = False
        return False

    # Y 이동 감지 후에는 다음 박스가 정지 위치에 도착할 때까지 속도를 유지한다.
    current_speed = get_conveyor_x_velocity()

    if (
        current_speed is None
        or abs(float(current_speed) - float(CONVEYOR_RUN_X_SPEED)) > 0.01
    ):
        set_conveyor_x_velocity(CONVEYOR_RUN_X_SPEED)
        is_conveyor_running = True

    return True


def stop_conveyor():
    global is_conveyor_running

    set_conveyor_x_velocity(CONVEYOR_STOP_X_SPEED)
    is_conveyor_running = False

    debug_print("[컨베이어 정지]")


def toggle_conveyor():
    current_x = get_conveyor_x_velocity()

    if current_x is None:
        debug_print("현재 컨베이어 속도를 읽지 못해서 토글할 수 없습니다.")
        return

    if abs(current_x) < 0.001:
        start_conveyor()
    else:
        stop_conveyor()


# ==================================================
# [Depth 생성 박스 ↔ 검사 결과 연동]
# ==================================================

def apply_defect_red_if_needed(
    box_path,
    current_pos=None
):
    """
    불량(result=0) 판정이 들어온 박스에 한 번만 즉시 빨간색을 적용한다.

    current_pos 인자는 기존 호출부 호환을 위해 유지하지만
    색상 변경 조건에는 사용하지 않는다.
    """
    if box_path in defect_red_applied_box_set:
        return False

    if box_result_dict.get(
        box_path,
        None
    ) != 0:
        return False

    if not apply_box_color(
        box_path,
        DEFECT_BOX_COLOR_RGB
    ):
        return False

    defect_red_applied_box_set.add(
        box_path
    )

    event_log(
        f"[불량 색상 변경] "
        f"검사 Track ID="
        f"{box_path_id_dict.get(box_path, None)} | "
        "불량 판정 즉시 빨간색 적용"
    )

    return True



def has_reached_defect_remove_trigger(
    prev_pos,
    current_pos
):
    """
    불량 박스가 제거 기준 World Z에 도착했거나,
    한 update 사이에 기준 좌표를 통과했으면 True를 반환한다.

    컨베이어 속도가 빨라 허용 오차 구간을 건너뛰어도
    이전 Z와 현재 Z 사이에 기준값이 있으면 놓치지 않는다.
    """
    if not DEFECT_REMOVE_AT_Z_ENABLED:
        return False

    if current_pos is None:
        return False

    target_z = float(
        DEFECT_REMOVE_TRIGGER_Z
    )
    tolerance_z = float(
        DEFECT_REMOVE_TRIGGER_TOLERANCE_Z
    )
    current_z = float(
        current_pos[2]
    )

    if abs(
        current_z - target_z
    ) <= tolerance_z:
        return True

    if prev_pos is None:
        return False

    prev_z = float(
        prev_pos[2]
    )

    return (
        min(prev_z, current_z)
        <= target_z
        <= max(prev_z, current_z)
    )


def cleanup_removed_box_runtime_state(
    box_path
):
    """
    Stage에서 제거한 박스의 경로/판정/좌표 캐시를 모두 정리한다.
    다음 박스의 검사 결과 FIFO와 자동 공정에 stale 경로가 남지 않게 한다.
    """
    global first_depth_spawned_box_path

    while box_path in spawned_boxes:
        spawned_boxes.remove(
            box_path
        )

    last_box_pos_dict.pop(
        box_path,
        None
    )
    stopped_box_world_position_dict.pop(
        box_path,
        None
    )

    remove_pending_inspection_box(
        box_path
    )

    defect_red_applied_box_set.discard(
        box_path
    )
    rejector_triggered_box_set.discard(
        box_path
    )
    tray_loader_rejector_section_box_set.discard(
        box_path
    )
    tray_loader_z_triggered_box_set.discard(
        box_path
    )
    auto_sequence_done_box_set.discard(
        box_path
    )

    box_result_dict.pop(
        box_path,
        None
    )
    box_label_dict.pop(
        box_path,
        None
    )
    box_yaw_dict.pop(
        box_path,
        None
    )

    box_id = box_path_id_dict.pop(
        box_path,
        None
    )

    if (
        box_id is not None
        and box_id_path_dict.get(
            box_id
        ) == box_path
    ):
        box_id_path_dict.pop(
            box_id,
            None
        )

    for track_id, mapped_path in list(
        inspection_track_box_path_dict.items()
    ):
        if mapped_path == box_path:
            inspection_track_box_path_dict.pop(
                track_id,
                None
            )

    # 첫 번째 생성 박스가 불량으로 제거된 경우,
    # 다음에 남아 있는 생성 박스를 트레이로더 Z=500 감시 대상으로 넘긴다.
    if first_depth_spawned_box_path == box_path:
        next_first_box = None

        for candidate_path in spawned_boxes:
            if get_prim(
                candidate_path
            ) is not None:
                next_first_box = candidate_path
                break

        first_depth_spawned_box_path = next_first_box


async def remove_defect_box_at_trigger_async(
    box_path,
    detected_z
):
    """
    불량 박스가 World Z=640에 도착하면 리젝터만 동작시킨다.

        1. 기존 리젝터 동작 종료 대기
        2. 리젝터 0도 -> -100도
        3. -100도 유지
        4. 리젝터 -100도 -> 0도 복귀
        5. 불량 박스는 Stage에 그대로 유지

    함수 이름은 기존 호출부 호환을 위해 유지한다.
    """
    app = omni.kit.app.get_app()
    wait_frames = 0

    while (
        is_rejector_moving
        and get_prim(box_path) is not None
        and wait_frames < int(DEFECT_REMOVE_REJECTOR_WAIT_MAX_FRAMES)
    ):
        wait_frames += 1
        await app.next_update_async()

    if get_prim(box_path) is None:
        defect_remove_scheduled_box_set.discard(box_path)
        return False

    if is_rejector_moving:
        defect_remove_scheduled_box_set.discard(box_path)
        rejector_triggered_box_set.discard(box_path)
        return False

    completed = await run_rejector(
        trigger_box_path=box_path,
        delete_trigger_box_at_push=False,
        detected_z=detected_z,
    )

    defect_remove_scheduled_box_set.discard(box_path)

    return bool(completed)



def schedule_defect_box_removal(
    box_path,
    current_pos
):
    """
    동일 불량 박스의 리젝터/삭제 통합 작업을 한 번만 예약한다.
    """
    if (
        box_path is None
        or current_pos is None
        or box_path in defect_remove_scheduled_box_set
    ):
        return False

    if box_result_dict.get(box_path, None) != 0:
        return False

    defect_remove_scheduled_box_set.add(box_path)

    # Z=640 통합 작업에서 리젝터를 실행하므로
    # 기존 Z=611 감시가 같은 박스에 다시 실행되지 않게 한다.
    rejector_triggered_box_set.add(box_path)

    asyncio.ensure_future(
        remove_defect_box_at_trigger_async(
            box_path,
            float(current_pos[2])
        )
    )

    return True



def remove_pending_inspection_box(
    box_path
):
    """
    검사 결과를 받은 박스를 대기 목록에서 제거한다.
    """
    while box_path in pending_inspection_box_paths:
        pending_inspection_box_paths.remove(
            box_path
        )


def get_oldest_pending_inspection_box():
    """
    아직 판정되지 않은 가장 먼저 생성된 유효 박스를 반환한다.

    Depth 카메라의 object_id와 검사 카메라의 ByteTrack ID는
    서로 독립적이므로 기본 연결은 제품 통과 순서(FIFO)를 사용한다.
    """
    while pending_inspection_box_paths:
        box_path = pending_inspection_box_paths[0]

        if get_prim(box_path) is None:
            pending_inspection_box_paths.pop(0)
            continue

        if box_result_dict.get(box_path, None) is not None:
            pending_inspection_box_paths.pop(0)
            continue

        return box_path

    return None


def apply_inspection_result_to_depth_box(
    value,
    inspection_track_id=None
):
    """
    검사 코드가 보낸 INSPECTION_RESULT를 Depth에서 생성된 Isaac 박스에 연결한다.

    우선순위:
    1. 이전에 동일 검사 track_id와 연결된 박스
    2. Isaac 생성 ID와 검사 track_id가 우연히 같은 박스
    3. 아직 판정되지 않은 가장 오래된 Depth 생성 박스(FIFO)
    """
    try:
        result_value = int(value)
    except Exception:
        event_log(
            f"[검사 결과 오류] value={value}"
        )
        return False

    if result_value not in (0, 1):
        event_log(
            f"[검사 결과 오류] 0/1 이외의 값: {result_value}"
        )
        return False

    try:
        normalized_track_id = (
            int(inspection_track_id)
            if inspection_track_id is not None
            else None
        )
    except Exception:
        normalized_track_id = None

    box_path = None
    match_method = None

    # 이미 연결된 검사 track_id가 있으면 동일 박스 갱신
    if normalized_track_id is not None:
        mapped_path = inspection_track_box_path_dict.get(
            normalized_track_id,
            None
        )

        if mapped_path is not None and get_prim(mapped_path) is not None:
            box_path = mapped_path
            match_method = "기존 검사 Track 매핑"

    # Depth object_id와 검사 track_id가 동일한 경우
    if box_path is None and normalized_track_id is not None:
        same_id_path = box_id_path_dict.get(
            normalized_track_id,
            None
        )

        if (
            same_id_path is not None
            and get_prim(same_id_path) is not None
            and box_result_dict.get(same_id_path, None) is None
        ):
            box_path = same_id_path
            match_method = "동일 ID"

    # 두 카메라 ID가 다르면 생성/검사 순서로 연결
    if box_path is None:
        box_path = get_oldest_pending_inspection_box()
        match_method = "생성 순서 FIFO"

    if box_path is None:
        event_log(
            f"[검사 결과 연결 실패] "
            f"Track ID={normalized_track_id}, value={result_value} | "
            "대기 중인 Depth 박스 없음"
        )
        return False

    if normalized_track_id is not None:
        inspection_track_box_path_dict[
            normalized_track_id
        ] = box_path

    box_result_dict[box_path] = result_value
    box_label_dict[box_path] = (
        "defect"
        if result_value == 0
        else "normal"
    )

    remove_pending_inspection_box(
        box_path
    )

    if result_value == 0:
        changed_now = apply_defect_red_if_needed(
            box_path
        )

        if changed_now:
            color_state = "불량 판정 즉시 빨간색"
        else:
            color_state = "불량 빨간색 이미 적용"
    else:
        apply_box_color(
            box_path,
            BOX_COLOR_RGB
        )

        defect_red_applied_box_set.discard(
            box_path
        )

        color_state = "정상 회색 유지"

    event_log(
        f"[검사 결과 연결] "
        f"검사 Track ID={normalized_track_id} | "
        f"Isaac ID={box_path_id_dict.get(box_path, None)} | "
        f"판정={result_to_text(result_value)} | "
        f"연결={match_method} | {color_state}"
    )

    return True


# ==================================================
# [11. 박스 생성]
# ==================================================


def get_box_color_by_result(result):
    """
    반환값:
    - color_rgb: 박스에 사용할 색상
    - label: 판정 라벨
    - apply_defect_color: 불량 빨간색을 추가 적용할지 여부

    정상(result=1)은 원래 생성 색상 BOX_COLOR_RGB를 유지하고,
    불량(result=0)에만 빨간색을 적용한다.
    """
    try:
        if result is not None:
            result = int(result)
    except Exception:
        result = None

    if result == 0:
        return DEFECT_BOX_COLOR_RGB, "defect", True

    if result == 1:
        return BOX_COLOR_RGB, "normal", False

    return BOX_COLOR_RGB, "unknown", False


def apply_box_color(box_path, color_rgb):
    """
    박스마다 독립된 PreviewSurface 재질을 생성해 색상을 적용한다.
    displayColor도 함께 적용해 뷰포트 렌더 모드 차이에도 색상이 보이도록 한다.
    """
    stage = get_stage()
    prim = get_prim(box_path)

    if stage is None or prim is None:
        debug_print("[색상 적용 실패] Stage 또는 박스 Prim이 없습니다:", box_path)
        return False

    color = Gf.Vec3f(
        float(color_rgb[0]),
        float(color_rgb[1]),
        float(color_rgb[2])
    )

    # 1. 기본 뷰포트용 displayColor 적용
    try:
        gprim = UsdGeom.Gprim(prim)
        gprim.CreateDisplayColorAttr().Set([color])
    except Exception as e:
        debug_print("[경고] displayColor 적용 실패:", e)

    # 2. 박스별 독립 Material/Shader 생성
    mat_path = f"{box_path}/YOLO_Material"
    shader_path = f"{mat_path}/PreviewSurface"

    try:
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, shader_path)

        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput(
            "diffuseColor",
            Sdf.ValueTypeNames.Color3f
        ).Set(color)
        shader.CreateInput(
            "roughness",
            Sdf.ValueTypeNames.Float
        ).Set(0.45)
        shader.CreateInput(
            "metallic",
            Sdf.ValueTypeNames.Float
        ).Set(0.0)

        surface_output = shader.CreateOutput(
            "surface",
            Sdf.ValueTypeNames.Token
        )
        material.CreateSurfaceOutput().ConnectToSource(surface_output)

        binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
        binding_api.Bind(material)

        debug_print(f"[색상 적용 완료] {box_path} -> {color}")
        return True

    except Exception as e:
        debug_print("[색상 적용 실패] PreviewSurface 생성/연결 오류:", e)
        return False



# ==================================================
# [추가] TCP JSON에서 yaw 각도 추출 및 박스 회전 적용
# ==================================================

def extract_received_yaw(msg):
    """
    아래 형식을 모두 지원한다.

    1) "rotation_deg": {"roll": 0, "pitch": 0, "yaw": 25.0}
    2) "yaw_deg": 25.0
    3) "yaw": 25.0
    4) "angle": 25.0

    각도 정보가 없거나 숫자로 변환할 수 없으면 None을 반환한다.
    """
    if not isinstance(msg, dict):
        return None

    yaw_value = None

    rotation_deg = msg.get("rotation_deg", None)

    if isinstance(rotation_deg, dict):
        yaw_value = rotation_deg.get("yaw", None)

    if yaw_value is None:
        yaw_value = msg.get("yaw_deg", None)

    if yaw_value is None:
        yaw_value = msg.get("yaw", None)

    if yaw_value is None:
        yaw_value = msg.get("angle", None)

    if yaw_value is None:
        return None

    try:
        return float(yaw_value)
    except Exception:
        debug_print("[각도 수신 실패] yaw 값을 숫자로 변환할 수 없습니다:", yaw_value)
        return None


def normalize_received_yaw(yaw_deg):
    """
    수신 각도를 -180도 이상 180도 미만으로 정규화한다.

    박스는 180도 회전 대칭이므로:
        0도   = 긴 변이 Isaac Z축 방향
        90도  = 긴 변이 Isaac X축 방향
       -90도  = 긴 변이 Isaac X축 방향
    """
    yaw = float(yaw_deg)

    if INVERT_RECEIVED_YAW:
        yaw = -yaw

    yaw = yaw + float(BOX_YAW_OFFSET_DEG)

    while yaw >= 180.0:
        yaw -= 360.0

    while yaw < -180.0:
        yaw += 360.0

    if SNAP_CARDINAL_YAW:
        cardinal_angles = (
            -180.0,
            -90.0,
            0.0,
            90.0,
            180.0
        )

        nearest = min(
            cardinal_angles,
            key=lambda value: abs(yaw - value)
        )

        if abs(yaw - nearest) <= CARDINAL_YAW_SNAP_TOLERANCE_DEG:
            yaw = nearest

        if yaw >= 180.0:
            yaw -= 360.0

    return float(yaw)


def make_axis_quaternion(axis, angle_deg):
    """
    지정 축 회전 quaternion을 반환한다.
    Gf.Quatd는 scalar-first 형식: (w, xyz)
    """
    half_rad = math.radians(float(angle_deg)) * 0.5

    real = math.cos(half_rad)
    imag = math.sin(half_rad)

    axis = str(axis).upper()

    if axis == "X":
        vector = Gf.Vec3d(imag, 0.0, 0.0)
    elif axis == "Z":
        vector = Gf.Vec3d(0.0, 0.0, imag)
    else:
        vector = Gf.Vec3d(0.0, imag, 0.0)

    return Gf.Quatd(real, vector)


def set_physics_compatible_orientation(prim, axis, angle_deg):
    """
    Dynamic Rigid Body에 안정적으로 유지되는 xformOp:orient를 적용한다.

    최종 Xform 순서:
        translate -> orient -> scale

    기존 rotateXYZ는 xformOpOrder에서 제외한다.
    """
    if prim is None or not prim.IsValid():
        return False

    translate_attr = prim.GetAttribute(
        "xformOp:translate"
    )

    scale_attr = prim.GetAttribute(
        "xformOp:scale"
    )

    orient_attr = prim.GetAttribute(
        "xformOp:orient"
    )

    if not orient_attr.IsValid():
        orient_attr = prim.CreateAttribute(
            "xformOp:orient",
            Sdf.ValueTypeNames.Quatd,
            custom=False
        )

    quaternion = make_axis_quaternion(
        axis,
        angle_deg
    )

    orient_type = orient_attr.GetTypeName()

    if orient_type == Sdf.ValueTypeNames.Quatf:
        imag = quaternion.GetImaginary()

        orient_attr.Set(
            Gf.Quatf(
                float(quaternion.GetReal()),
                Gf.Vec3f(
                    float(imag[0]),
                    float(imag[1]),
                    float(imag[2])
                )
            )
        )
    else:
        orient_attr.Set(quaternion)

    op_order = []

    if translate_attr.IsValid():
        op_order.append("xformOp:translate")

    op_order.append("xformOp:orient")

    if scale_attr.IsValid():
        op_order.append("xformOp:scale")

    order_attr = prim.GetAttribute(
        "xformOpOrder"
    )

    if not order_attr.IsValid():
        order_attr = prim.CreateAttribute(
            "xformOpOrder",
            Sdf.ValueTypeNames.TokenArray,
            custom=False
        )

    order_attr.Set(
        Vt.TokenArray(op_order)
    )

    return True


def apply_box_yaw(box_path, yaw_deg):
    """
    OpenCV에서 받은 yaw를 박스에 적용한다.

    현재 장면의 바닥면은 X-Z 평면이고 높이축은 Y이므로
    BOX_YAW_AXIS="Y"를 사용한다.

        yaw=0도   -> 세로, 긴 변이 Isaac Z축
        yaw=90도  -> 가로, 긴 변이 Isaac X축
    """
    if yaw_deg is None:
        event_log(
            f"[각도 수신 없음] {box_path} | 기본 0도 적용"
        )
        yaw_deg = 0.0

    prim = get_prim(box_path)

    if prim is None:
        event_log(
            f"[각도 적용 실패] Prim 없음: {box_path}"
        )
        return None

    try:
        applied_yaw = normalize_received_yaw(
            yaw_deg
        )
    except Exception as error:
        event_log(
            f"[각도 적용 실패] yaw 변환 오류: {error}"
        )
        return None

    axis = str(BOX_YAW_AXIS).upper()

    if axis not in ("X", "Y", "Z"):
        axis = "Y"

    try:
        if USE_PHYSICS_COMPATIBLE_ORIENT:
            success = set_physics_compatible_orientation(
                prim,
                axis,
                applied_yaw
            )
        else:
            if axis == "X":
                rotation_xyz = (
                    applied_yaw,
                    0.0,
                    0.0
                )
            elif axis == "Z":
                rotation_xyz = (
                    0.0,
                    0.0,
                    applied_yaw
                )
            else:
                rotation_xyz = (
                    0.0,
                    applied_yaw,
                    0.0
                )

            rotate_attr = prim.GetAttribute(
                "xformOp:rotateXYZ"
            )

            if rotate_attr.IsValid():
                if (
                    rotate_attr.GetTypeName()
                    == Sdf.ValueTypeNames.Double3
                ):
                    rotate_attr.Set(
                        Gf.Vec3d(*rotation_xyz)
                    )
                else:
                    rotate_attr.Set(
                        Gf.Vec3f(*rotation_xyz)
                    )

                success = True
            else:
                xformable = UsdGeom.Xformable(
                    prim
                )

                rotate_op = xformable.AddRotateXYZOp(
                    UsdGeom.XformOp.PrecisionDouble,
                    "receivedYaw"
                )

                rotate_op.Set(
                    Gf.Vec3d(*rotation_xyz)
                )

                success = True

        if not success:
            event_log(
                f"[각도 적용 실패] {box_path}"
            )
            return None

        # 회전 적용 직후 회전 관성을 제거
        if prim.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            rigid_body = UsdPhysics.RigidBodyAPI(
                prim
            )

            rigid_body.CreateAngularVelocityAttr().Set(
                Gf.Vec3f(0.0, 0.0, 0.0)
            )

        box_yaw_dict[box_path] = float(
            applied_yaw
        )

        event_log(
            f"[각도 적용] ID={box_path_id_dict.get(box_path, None)} | "
            f"수신={float(yaw_deg):.2f}도 | "
            f"적용={applied_yaw:.2f}도 | "
            f"축={axis}"
        )

        return float(applied_yaw)

    except Exception as error:
        event_log(
            f"[각도 적용 실패] {box_path} | {error}"
        )
        return None


def normalize_applied_yaw(angle_deg):
    angle = float(angle_deg)

    while angle >= 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return float(angle)


def set_box_applied_yaw_direct(
    box_path,
    applied_yaw_deg,
    update_dict=True
):
    """
    이미 Isaac 좌표계에 적용할 값으로 계산된 yaw를 직접 설정한다.
    수신각 반전/오프셋은 다시 적용하지 않는다.
    """
    prim = get_prim(box_path)

    if prim is None:
        return False

    axis = str(BOX_YAW_AXIS).upper()

    if axis not in ("X", "Y", "Z"):
        axis = "Y"

    normalized_yaw = normalize_applied_yaw(
        applied_yaw_deg
    )

    try:
        success = set_physics_compatible_orientation(
            prim,
            axis,
            normalized_yaw
        )

        if not success:
            return False

        if update_dict:
            box_yaw_dict[
                box_path
            ] = float(normalized_yaw)

        return True

    except Exception as error:
        event_log(
            f"[박스 회전 추종 오류] "
            f"{box_path} | {error}"
        )
        return False


def apply_box_target_scale(box_path):
    """
    생성 박스의 scale을 BOX_TARGET_SCALE로 다시 고정한다.

    Dynamic Rigid Body에 yaw를 적용하거나 PhysX가 초기화되는 과정에서
    비균일 scale이 변형되어 보이는 현상을 줄이기 위해 사용한다.
    """
    prim = get_prim(box_path)

    if prim is None:
        return False

    scale_attr = prim.GetAttribute("xformOp:scale")

    try:
        if not scale_attr.IsValid():
            xformable = UsdGeom.Xformable(prim)
            scale_op = xformable.AddScaleOp(
                UsdGeom.XformOp.PrecisionDouble
            )
            scale_op.Set(
                Gf.Vec3d(
                    float(BOX_TARGET_SCALE[0]),
                    float(BOX_TARGET_SCALE[1]),
                    float(BOX_TARGET_SCALE[2])
                )
            )
            return True

        if scale_attr.GetTypeName() == Sdf.ValueTypeNames.Float3:
            scale_attr.Set(
                Gf.Vec3f(
                    float(BOX_TARGET_SCALE[0]),
                    float(BOX_TARGET_SCALE[1]),
                    float(BOX_TARGET_SCALE[2])
                )
            )
        else:
            scale_attr.Set(
                Gf.Vec3d(
                    float(BOX_TARGET_SCALE[0]),
                    float(BOX_TARGET_SCALE[1]),
                    float(BOX_TARGET_SCALE[2])
                )
            )

        return True

    except Exception as error:
        event_log(
            f"[박스 스케일 적용 실패] {box_path} | {error}"
        )
        return False


async def reapply_box_yaw_after_physics(
    box_path,
    yaw_deg
):
    """
    박스 생성 직후 PhysX가 Rigid Body transform을 초기화한 다음
    동일한 yaw를 한 번 더 적용한다.
    """
    try:
        app = omni.kit.app.get_app()

        await app.next_update_async()
        await app.next_update_async()

        if get_prim(box_path) is None:
            return

        apply_box_yaw(
            box_path,
            yaw_deg
        )

        # 물리 초기화 이후에도 목표 scale을 다시 고정한다.
        apply_box_target_scale(
            box_path
        )

    except Exception as error:
        event_log(
            f"[각도 재적용 실패] {box_path} | {error}"
        )



def clamp(value, minimum, maximum):
    """값을 minimum~maximum 범위로 제한한다."""
    return max(float(minimum), min(float(maximum), float(value)))


def clamp_spawn_value(value, minimum, maximum):
    return clamp(value, minimum, maximum)


def extract_received_position(msg):
    """
    CREATE_BOX JSON에서 Depth Camera의 X/Y 좌표를 읽는다.

    권장 형식:
        "position": {
            "x": 1.6,
            "y": -57.4,
            "z": 42.1
        }

    좌표 사용:
        position.x -> Isaac X
        position.y -> Isaac Z
        position.z -> 생성 위치에는 사용하지 않고 로그용으로만 저장

    position.x와 position.y는 필수이고,
    position.z는 생략 가능하다.
    """
    if not isinstance(msg, dict):
        return None

    position = msg.get("position", None)

    if not isinstance(position, dict):
        return None

    try:
        x_mm = float(position["x"])
        y_mm = float(position["y"])
    except (KeyError, TypeError, ValueError):
        debug_print(
            "[TCP 좌표 오류] position.x와 position.y가 필요합니다:",
            position
        )
        return None

    try:
        z_mm = float(
            position.get(
                "z",
                0.0
            )
        )
    except (TypeError, ValueError):
        z_mm = 0.0

    return {
        "x": x_mm,
        "y": y_mm,
        "z": z_mm,
        "unit": msg.get(
            "position_unit",
            "mm"
        ),
        "coordinate_system": msg.get(
            "coordinate_system",
            "depth_xy"
        )
    }


def convert_tcp_position_to_isaac_world(
    received_position
):
    """
    Depth Camera 좌표를 Isaac 월드 좌표로 변환한다.

    변환:
        Isaac X = 기준 X + (Depth x - 기준 x)
        Isaac Y = BOX_SPAWN_POS[1] 고정
        Isaac Z = 기준 Z + (Depth y - 기준 y)

    Depth position.z는 박스 생성 좌표에 사용하지 않는다.
    """
    if received_position is None:
        return Gf.Vec3d(
            BOX_SPAWN_POS[0],
            BOX_SPAWN_POS[1],
            BOX_SPAWN_POS[2]
        )

    depth_x = float(
        received_position["x"]
    )
    depth_y = float(
        received_position["y"]
    )

    delta_x = (
        depth_x - TCP_REFERENCE_X_MM
    ) * TCP_TO_ISAAC_SCALE

    delta_z = (
        depth_y - TCP_REFERENCE_Y_MM
    ) * TCP_TO_ISAAC_SCALE

    isaac_x = (
        float(ISAAC_ARUCO_ORIGIN_WORLD[0])
        + delta_x
        * TCP_X_TO_ISAAC_X_SIGN
    )

    # Isaac 높이는 컨베이어 위 고정값 사용
    isaac_y = float(
        BOX_SPAWN_POS[1]
    )

    # Depth Y를 Isaac Z로 사용
    isaac_z = (
        float(ISAAC_ARUCO_ORIGIN_WORLD[2])
        + delta_z
        * TCP_Y_TO_ISAAC_Z_SIGN
    )

    # 장면 밖 생성 방지
    isaac_x = clamp_spawn_value(
        isaac_x,
        RECEIVED_SPAWN_X_MIN,
        RECEIVED_SPAWN_X_MAX
    )

    isaac_y = clamp_spawn_value(
        isaac_y,
        RECEIVED_SPAWN_Y_MIN,
        RECEIVED_SPAWN_Y_MAX
    )

    isaac_z = clamp_spawn_value(
        isaac_z,
        RECEIVED_SPAWN_Z_MIN,
        RECEIVED_SPAWN_Z_MAX
    )

    return Gf.Vec3d(
        isaac_x,
        isaac_y,
        isaac_z
    )


def get_spawn_position_for_source(
    source,
    received_position
):
    """
    수동 B키는 기존 고정 위치,
    TCP/YOLO 박스는 수신 좌표 변환 위치를 사용한다.
    """
    if (
        source == "YOLO"
        and USE_RECEIVED_TCP_SPAWN_POSITION
        and received_position is not None
    ):
        return convert_tcp_position_to_isaac_world(
            received_position
        )

    return Gf.Vec3d(
        BOX_SPAWN_POS[0],
        BOX_SPAWN_POS[1],
        BOX_SPAWN_POS[2]
    )


def make_unique_box_path(box_id=None):
    if box_id is not None:
        base_path = f"{ORIGINAL_BOX_PATH_PREFIX}_prim_spawned_ID_{box_id}"
    else:
        base_path = f"{ORIGINAL_BOX_PATH_PREFIX}_prim_spawned_{random.randint(10000, 99999)}"

    new_box_path = base_path

    while get_prim(new_box_path) is not None:
        new_box_path = f"{base_path}_{random.randint(10000, 99999)}"

    return new_box_path


def spawn_box(
    box_id=None,
    result=None,
    track_id=None,
    label=None,
    source="MANUAL",
    yaw_deg=None,
    received_position=None
):
    """
    source="MANUAL" : B 키 수동 생성
    source="YOLO"   : YOLO TCP 판정 결과 기반 생성

    result = 1 : 정상 박스, 기존 생성 색상 유지
    result = 0 : 불량 박스, 빨간색 적용
    result = None : 미판정/수동, 기존 생성 색상 유지
    yaw_deg       : OpenCV에서 수신한 평면 회전각
    received_position : Depth Camera에서 수신한 X/Y/Z 좌표[mm]
                        X -> Isaac X
                        Y -> Isaac Z
                        Isaac Y는 고정 높이를 사용하며
                        Depth Z는 생성 위치에 사용하지 않는다.
    """
    global is_conveyor_running
    global debug_count
    global last_spawn_time
    global first_depth_spawned_box_path

    now_time = time.time()

    # 수동 B키 생성만 디바운스 적용
    if source == "MANUAL":
        if now_time - last_spawn_time < SPAWN_DEBOUNCE_TIME:
            debug_print("[중복 방지] B 입력이 너무 빠르거나 이벤트가 중복되어 박스 생성 무시")
            return None

        last_spawn_time = now_time

    debug_count = 0

    # YOLO id가 이미 생성된 경우 중복 생성 방지
    if box_id is not None:
        try:
            box_id = int(box_id)
        except Exception:
            debug_print("[YOLO TCP] box_id가 정수가 아닙니다:", box_id)
            return None

        if box_id in box_id_path_dict:
            existing_path = box_id_path_dict[box_id]

            if get_prim(existing_path) is not None:
                debug_print("==================================================")
                debug_print("[YOLO TCP] 이미 생성된 id 박스입니다. 중복 생성하지 않습니다.")
                debug_print("ID:", box_id)
                debug_print("Path:", existing_path)
                debug_print("==================================================")
                return existing_path

    color_rgb, auto_label, apply_defect_color = get_box_color_by_result(result)

    if label is None:
        label = auto_label

    spawn_position = get_spawn_position_for_source(
        source,
        received_position
    )

    new_box_path = make_unique_box_path(box_id)

    omni.kit.commands.execute(
        "CreatePrim",
        prim_type="Cube",
        prim_path=new_box_path
    )

    spawned_boxes.append(new_box_path)

    # 첫 번째로 생성된 박스를 트레이로더 기준 박스로 등록한다.
    # - 실제 Depth CREATE_BOX: source="YOLO"
    # - Script Editor B키 테스트: source="MANUAL"
    # Master 상태 이벤트는 박스를 생성하지 않으므로 여기 들어오지 않는다.
    if (
        source in ("YOLO", "MANUAL")
        and first_depth_spawned_box_path is None
    ):
        first_depth_spawned_box_path = new_box_path
        event_log(
            f"[트레이 이동] 첫 번째 생성 박스 등록 | "
            f"source={source} | ID={box_id} | path={new_box_path} | "
            "Z=500 도착 시 첫 트레이 공급 시작"
        )

    new_prim = get_prim(new_box_path)

    if new_prim is None:
        debug_print("박스 생성 실패:", new_box_path)
        return None

    UsdPhysics.CollisionAPI.Apply(new_prim)
    UsdPhysics.RigidBodyAPI.Apply(new_prim)

    omni.kit.commands.execute(
        "ChangeProperty",
        prop_path=f"{new_box_path}.xformOp:scale",
        value=BOX_TARGET_SCALE,
        prev=None
    )

    omni.kit.commands.execute(
        "TransformPrimSRT",
        path=new_box_path,
        new_translation=spawn_position
    )

    # TCP JSON에서 받은 위치와 yaw를 박스 생성 시 적용한다.
    # 수동 B키는 기존 BOX_SPAWN_POS를 유지한다.
    applied_yaw = apply_box_yaw(new_box_path, yaw_deg)

    # yaw 적용 뒤에도 박스 크기를 목표값으로 다시 고정한다.
    apply_box_target_scale(
        new_box_path
    )

    # Depth에서 생성되는 모든 박스는 우선 기존 회색으로 생성한다.
    # 검사 결과가 나중에 INSPECTION_RESULT로 들어오면 해당 박스 판정을 갱신한다.
    apply_box_color(
        new_box_path,
        BOX_COLOR_RGB
    )

    if apply_defect_color:
        color_status = "불량 판정 / 생성 직후 빨간색 적용 예정"
    else:
        color_status = "검사 결과 대기 또는 정상 회색 유지"

    restore_box_physics(new_box_path)

    # Rigid Body 적용 이후에도 scale을 한 번 더 고정한다.
    apply_box_target_scale(
        new_box_path
    )

    # PhysX가 첫 물리 프레임에서 회전을 기본값으로 되돌리는 경우를 막기 위해
    # quaternion orient 방식으로 다음 프레임에 한 번 더 적용한다.
    if (
        REAPPLY_YAW_AFTER_PHYSICS
        and yaw_deg is not None
    ):
        asyncio.ensure_future(
            reapply_box_yaw_after_physics(
                new_box_path,
                yaw_deg
            )
        )

    current_pos = get_world_position(new_box_path)
    last_box_pos_dict[new_box_path] = current_pos

    # YOLO id / result 저장
    if box_id is not None:
        box_id_path_dict[box_id] = new_box_path
        box_path_id_dict[new_box_path] = box_id

    try:
        box_result_dict[new_box_path] = int(result) if result is not None else None
    except Exception:
        box_result_dict[new_box_path] = None

    box_label_dict[new_box_path] = label

    # CREATE_BOX 단계에서 이미 result=0이 들어온 경우에는
    # 다음 update를 기다리지 않고 즉시 빨간색으로 변경한다.
    if box_result_dict.get(
        new_box_path,
        None
    ) == 0:
        apply_defect_red_if_needed(
            new_box_path
        )

    # Depth CREATE_BOX는 보통 result 없이 먼저 들어오므로 검사 결과 대기 등록
    if box_result_dict.get(new_box_path, None) is None:
        if new_box_path not in pending_inspection_box_paths:
            pending_inspection_box_paths.append(
                new_box_path
            )

        event_log(
            f"[검사 대기 등록] "
            f"Isaac ID={box_id} | "
            f"대기 순번={len(pending_inspection_box_paths)}"
        )

    if applied_yaw is None:
        box_yaw_dict[new_box_path] = 0.0

    # 박스 생성은 컨베이어 속도를 변경하지 않는다.
    # 컨베이어는 Timeline PLAY에서 계속 가동하고,
    # 지정된 정지 좌표에 도착했을 때만 0으로 변경한다.
    current_conveyor_speed = (
        get_conveyor_x_velocity()
    )

    if current_conveyor_speed is None:
        conveyor_state_text = (
            "현재 속도 읽기 실패 / 기존 상태 유지"
        )
    elif abs(current_conveyor_speed) < 0.001:
        conveyor_state_text = (
            "정지 상태 유지"
        )
    else:
        conveyor_state_text = (
            f"가동 유지 {current_conveyor_speed:.1f}"
        )

    if received_position is not None:
        received_depth_text = (
            f"Depth XYZ=("
            f"{float(received_position['x']):.3f}, "
            f"{float(received_position['y']):.3f}, "
            f"{float(received_position['z']):.3f}) | "
            f"매핑(X→X, Y→Z) | "
        )
    else:
        received_depth_text = ""

    event_log(
        f"[박스 생성] ID={box_id} | "
        f"{received_depth_text}"
        f"Isaac XYZ=({spawn_position[0]:.3f}, "
        f"{spawn_position[1]:.3f}, "
        f"{spawn_position[2]:.3f}) | "
        f"Isaac Y고정={BOX_SPAWN_POS[1]:.3f} | "
        f"각도={box_yaw_dict.get(new_box_path, 0.0):.2f}도 ({BOX_YAW_AXIS}축) | "
        f"판정={result_to_text(result)} | "
        f"컨베이어={conveyor_state_text}"
    )

    debug_print("==================================================")
    debug_print("[박스 스폰 완료]")
    debug_print("Source:", source)
    debug_print("생성 경로:", new_box_path)
    debug_print("YOLO ID:", box_id)
    debug_print("Track ID:", track_id)
    debug_print("Result:", result, "(1=정상/기존 색상, 0=불량/빨강)")
    debug_print("Label:", label)
    debug_print("Color Status:", color_status)
    debug_print("Received Position:", received_position)
    debug_print(
        "Isaac Spawn XYZ:",
        float(spawn_position[0]),
        float(spawn_position[1]),
        float(spawn_position[2])
    )
    debug_print("Received Yaw:", yaw_deg)
    debug_print("Applied Yaw:", box_yaw_dict.get(new_box_path, 0.0))
    debug_print("Rotation Axis:", BOX_YAW_AXIS)
    debug_print("컨베이어 가동 시작")
    debug_print("==================================================")

    # v51.21:
    # 생성된 박스가 Viewport 선택 상태로 남아 Pivot/Transform Manipulator가
    # 표시되지 않도록 박스 생성 완료 직후 Stage 선택을 비운다.
    clear_stage_selection()

    return new_box_path


# ==================================================
# [v49 depth(1).py + master(1).py 호환 처리]
# ==================================================

def _master_remember_recent_key(key, key_deque, key_set, maximum):
    if key in key_set:
        return False

    key_deque.append(key)
    key_set.add(key)

    while len(key_deque) > int(maximum):
        old_key = key_deque.popleft()
        key_set.discard(old_key)

    return True


def _master_make_inspection_key(value, track_id, timestamp):
    try:
        normalized_value = int(value)
    except Exception:
        normalized_value = value

    try:
        normalized_track_id = int(track_id) if track_id is not None else None
    except Exception:
        normalized_track_id = str(track_id)

    try:
        normalized_timestamp = round(float(timestamp), 4)
    except Exception:
        normalized_timestamp = None

    return (
        normalized_value,
        normalized_track_id,
        normalized_timestamp,
    )


def _master_make_vision_key(data, timestamp):
    try:
        count_value = int(data.get("count"))
    except Exception:
        count_value = None

    def _round_value(name, digits=3):
        try:
            value = data.get(name)
            if value is None:
                return None
            return round(float(value), digits)
        except Exception:
            return None

    try:
        timestamp_value = round(float(timestamp), 4)
    except Exception:
        timestamp_value = None

    return (
        count_value,
        timestamp_value,
        _round_value("x"),
        _round_value("y"),
        _round_value("z"),
        _round_value("angle", 2),
    )


def _next_master_auto_box_id():
    global master_auto_box_sequence

    while True:
        master_auto_box_sequence += 1
        candidate = int(MASTER_AUTO_BOX_ID_BASE) + int(master_auto_box_sequence)

        existing_path = box_id_path_dict.get(candidate)

        if existing_path is None or get_prim(existing_path) is None:
            return candidate


def _pop_master_pending_normal_result():
    while master_pending_normal_results:
        item = master_pending_normal_results.popleft()

        if not isinstance(item, dict):
            continue

        return item

    return None


def _queue_master_normal_result(track_id, label, timestamp):
    try:
        normalized_track_id = int(track_id) if track_id is not None else None
    except Exception:
        normalized_track_id = None

    # 같은 Track 결과가 큐에 이미 있으면 중복 저장하지 않는다.
    for item in master_pending_normal_results:
        if item.get("track_id") == normalized_track_id and normalized_track_id is not None:
            return False

    master_pending_normal_results.append({
        "value": 1,
        "track_id": normalized_track_id,
        "label": label or "NORMAL",
        "timestamp": timestamp,
    })

    event_log(
        f"[MASTER 호환] 정상 결과 대기 저장 | "
        f"track_id={normalized_track_id} | "
        f"대기={len(master_pending_normal_results)}"
    )

    return True


def _set_master_spawned_box_world_position(box_path, target_position):
    if box_path is None or get_prim(box_path) is None:
        return False

    success = set_world_position_keep_orientation(
        box_path,
        target_position,
    )

    if not success:
        return False

    stop_box_velocity(box_path)
    last_box_pos_dict[box_path] = copy_world_position(target_position)
    return True


def _master_world_x_from_robot_x(robot_x):
    try:
        robot_x = float(robot_x)
    except Exception:
        robot_x = 0.0

    world_x = (
        float(ISAAC_ARUCO_ORIGIN_WORLD[0])
        + (
            robot_x - float(TCP_REFERENCE_X_MM)
        )
        * float(TCP_TO_ISAAC_SCALE)
        * float(TCP_X_TO_ISAAC_X_SIGN)
    )

    return clamp_spawn_value(
        world_x,
        RECEIVED_SPAWN_X_MIN,
        RECEIVED_SPAWN_X_MAX,
    )


def spawn_master_defect_box_at_rejector(
    inspection_track_id=None,
    label=None,
):
    if not MASTER_DEFECT_AUTO_SPAWN_ENABLE:
        return None

    auto_box_id = _next_master_auto_box_id()

    box_path = spawn_box(
        box_id=auto_box_id,
        result=0,
        track_id=inspection_track_id,
        label=label or "DEFECT",
        source="MASTER_INSPECTION",
        yaw_deg=0.0,
        received_position=None,
    )

    if box_path is None:
        return None

    try:
        normalized_track_id = (
            int(inspection_track_id)
            if inspection_track_id is not None
            else None
        )
    except Exception:
        normalized_track_id = None

    if normalized_track_id is not None:
        inspection_track_box_path_dict[normalized_track_id] = box_path

    if MASTER_DEFECT_SPAWN_AT_REJECTOR_LINE:
        target_position = Gf.Vec3d(
            float(BOX_SPAWN_POS[0]),
            float(BOX_SPAWN_POS[1]),
            float(REJECTOR_TRIGGER_Z),
        )

        _set_master_spawned_box_world_position(
            box_path,
            target_position,
        )

    event_log(
        f"[MASTER 호환] 불량 박스 생성 | "
        f"track_id={normalized_track_id} | "
        f"Isaac ID={auto_box_id} | "
        f"Z={REJECTOR_TRIGGER_Z:.3f} | 리젝터 자동 감시"
    )

    return box_path


def _master_register_normal_box_at_pick_line(box_path, target_position):
    global is_conveyor_running

    if box_path is None or get_prim(box_path) is None:
        return False

    # 실물 제품이 이미 픽업 정지 위치에 있으므로 가상 검사 컨베이어도 정지한다.
    set_conveyor_x_velocity(CONVEYOR_STOP_X_SPEED)
    is_conveyor_running = False

    stop_box_velocity(box_path)

    stopped_position = copy_world_position(target_position)
    stopped_box_world_position_dict[box_path] = stopped_position
    last_box_pos_dict[box_path] = stopped_position

    # 첫 박스를 적재하는 중 들어온 다음 박스라면 대기 상태를 확정한다.
    finish_next_box_preload_at_stop(box_path)

    # 로봇이 준비되어 있으면 즉시 시작하고, 아니면 HOME 완료 후 자동 시작한다.
    try_start_pending_pick_sequence()

    return True


def spawn_master_normal_box_from_vision(data, timestamp=None):
    if not MASTER_VISION_AUTO_SPAWN_ENABLE:
        return None

    if not isinstance(data, dict):
        return None

    vision_key = _master_make_vision_key(data, timestamp)

    if not _master_remember_recent_key(
        vision_key,
        master_processed_vision_keys,
        master_processed_vision_key_set,
        MASTER_VISION_DUPLICATE_CACHE_SIZE,
    ):
        event_log(
            f"[MASTER 호환] 중복 VISION_PACKET 무시 | key={vision_key}"
        )
        return None

    pending_result = _pop_master_pending_normal_result()

    if pending_result is not None:
        result_value = 1
        inspection_track_id = pending_result.get("track_id")
        result_label = pending_result.get("label") or "NORMAL"
    else:
        result_value = int(MASTER_VISION_DEFAULT_RESULT)
        inspection_track_id = None
        result_label = "NORMAL_FROM_DEPTH"

    try:
        yaw_deg = float(data.get("angle", 0.0) or 0.0)
    except Exception:
        yaw_deg = 0.0

    auto_box_id = _next_master_auto_box_id()

    box_path = spawn_box(
        box_id=auto_box_id,
        result=result_value,
        track_id=inspection_track_id,
        label=result_label,
        source="MASTER_VISION",
        yaw_deg=yaw_deg,
        received_position=None,
    )

    if box_path is None:
        return None

    if inspection_track_id is not None:
        try:
            normalized_track_id = int(inspection_track_id)
            inspection_track_box_path_dict[normalized_track_id] = box_path
        except Exception:
            normalized_track_id = None
    else:
        normalized_track_id = None

    world_x = _master_world_x_from_robot_x(
        data.get("x", 0.0)
    )

    if MASTER_NORMAL_SPAWN_AT_PICK_LINE:
        target_position = Gf.Vec3d(
            float(world_x),
            float(BOX_SPAWN_POS[1]),
            float(STOP_TARGET_Z),
        )
    else:
        target_position = Gf.Vec3d(
            float(world_x),
            float(BOX_SPAWN_POS[1]),
            float(BOX_SPAWN_POS[2]),
        )

    if not _set_master_spawned_box_world_position(
        box_path,
        target_position,
    ):
        event_log(
            f"[MASTER 호환] 정상 박스 위치 설정 실패 | {box_path}"
        )
        return box_path

    if MASTER_NORMAL_SPAWN_AT_PICK_LINE:
        _master_register_normal_box_at_pick_line(
            box_path,
            target_position,
        )

    # 두 박스/한 트레이 구조에서 첫 번째 적재 순번일 때만 새 트레이 공급을 요청한다.
    if (
        MASTER_START_TRAY_LOADER_ON_FIRST_NORMAL
        and int(auto_place_count) % 2 == 0
    ):
        try:
            request_tray_loader_start(
                reason=(
                    "Master DEPTH/VISION_PACKET 첫 적재 제품 | "
                    f"Isaac ID={auto_box_id}"
                ),
                source_box_path=box_path,
            )
        except Exception as error:
            event_log(
                f"[MASTER 호환] 트레이로더 시작 요청 오류 | {error}"
            )

    event_log(
        f"[MASTER 호환] 정상 박스 생성/정지 등록 | "
        f"VISION count={data.get('count')} | "
        f"track_id={normalized_track_id} | "
        f"Isaac ID={auto_box_id} | "
        f"World X={world_x:.3f} | Z={target_position[2]:.3f} | "
        f"yaw={yaw_deg:.2f}"
    )

    return box_path


def handle_master_compatible_inspection_result(
    value,
    inspection_track_id=None,
    label=None,
    timestamp=None,
):
    try:
        result_value = int(value)
    except Exception:
        event_log(
            f"[MASTER 호환] INSPECTION_RESULT 값 오류 | value={value}"
        )
        return False

    if result_value not in (0, 1):
        return False

    inspection_key = _master_make_inspection_key(
        result_value,
        inspection_track_id,
        timestamp,
    )

    if not _master_remember_recent_key(
        inspection_key,
        master_processed_inspection_keys,
        master_processed_inspection_key_set,
        MASTER_INSPECTION_DUPLICATE_CACHE_SIZE,
    ):
        event_log(
            f"[MASTER 호환] 중복 INSPECTION_RESULT 무시 | key={inspection_key}"
        )
        return True

    # 기존 direct CREATE_BOX 또는 이미 생성된 미판정 박스가 있으면 기존 FIFO 연결을 우선한다.
    if apply_inspection_result_to_depth_box(
        value=result_value,
        inspection_track_id=inspection_track_id,
    ):
        return True

    # v51.1: YOLO/검사 결과는 이미 첫 번째 Depth가 CREATE_BOX로 만든
    # 미판정 박스에만 연결한다. 연결 대상이 없으면 새 박스를 만들거나
    # 정상 결과를 대기 큐에 저장하지 않고 무시한다.
    event_log(
        f"[검사 결과 무시] Track ID={inspection_track_id} | "
        f"판정={result_to_text(result_value)} | "
        "대기 중인 첫 번째 Depth 생성 박스 없음"
    )
    return False


def handle_master_bridge_compatible_event(msg):
    if not MASTER_BRIDGE_COMPAT_MODE or not isinstance(msg, dict):
        return False

    category = str(msg.get("category", "")).strip().upper()
    event_name = str(msg.get("event", "")).strip().upper()
    data = msg.get("data", {})

    if not isinstance(data, dict):
        data = {"value": data}

    timestamp = msg.get("timestamp", time.time())

    if category == "DEPTH" and event_name == "VISION_PACKET":
        # v51.1: Master에 연결된 Depth 패킷은 실제 직교로봇 좌표/각도
        # 동기화용이다. 박스 생성은 별도 첫 번째 Depth의 CREATE_BOX만 담당한다.
        return True

    if (
        MASTER_START_TRAY_LOADER_ON_LBUSY
        and category in ("MCU", "STM", "STM32")
        and event_name == "RESPONSE"
    ):
        response_value = str(
            data.get("response", data.get("value", ""))
        ).strip().upper()

        if response_value == "LBUSY":
            try:
                success = request_tray_loader_start(
                    reason="STM LBUSY 수신",
                    source_box_path=None,
                )
                event_log(
                    f"[MASTER 호환] STM LBUSY -> 트레이로더 시작 | "
                    f"성공={success}"
                )
                return bool(success)
            except Exception as error:
                event_log(
                    f"[MASTER 호환] STM LBUSY 처리 오류 | {error}"
                )
                return False

    return False


# Script Editor에서 현재 Master 호환 상태를 확인할 때 사용한다.
def master_compat_status():
    status = {
        "enabled": bool(MASTER_BRIDGE_COMPAT_MODE),
        "box_creation_mode": "FIRST_DEPTH_CREATE_BOX_ONLY",
        "master_vision_auto_spawn": bool(MASTER_VISION_AUTO_SPAWN_ENABLE),
        "master_defect_auto_spawn": bool(MASTER_DEFECT_AUTO_SPAWN_ENABLE),
        "pending_normal_results": list(master_pending_normal_results),
        "processed_vision_count": len(master_processed_vision_keys),
        "processed_inspection_count": len(master_processed_inspection_keys),
        "auto_box_sequence": int(master_auto_box_sequence),
        "master_connected": bool(
            get_master_runtime_state().get("master_tcp_connected", False)
        ),
        "last_master_event": get_master_runtime_state().get("last_event"),
        "robot_step": get_master_runtime_state().get("robot_step"),
        "depth": {
            "x": get_master_runtime_state().get("depth_x"),
            "y": get_master_runtime_state().get("depth_y"),
            "z": get_master_runtime_state().get("depth_z"),
            "angle": get_master_runtime_state().get("depth_angle"),
        },
    }

    _SYSTEM_PRINT(
        "[MASTER COMPAT STATUS]",
        status,
        flush=True,
    )

    return status


builtins.master_compat_status = master_compat_status


# ==================================================
# [12. 3축 직교 로봇 S-Curve 모션 제어]
# - 고정 시간 대기 없음
# - 매 프레임 targetPosition을 S-Curve 형태로 천천히 변경
# - 실제 Joint position이 목표에 도달한 뒤 다음 동작 실행
# ==================================================

AXIS_JOINT_PATHS = {
    "X1": PRISMATIC_JOINT_X1_PATH,
    "X2": PRISMATIC_JOINT_X2_PATH,
    "Y": PRISMATIC_JOINT_Y_PATH,
    "Z": PRISMATIC_JOINT_Z_PATH
}

AXIS_LIMITS = {
    "X1": (
        PICK_X1_TARGET_MIN,
        PICK_X1_TARGET_MAX
    ),
    "X2": (
        PICK_X2_TARGET_MIN,
        PICK_X2_TARGET_MAX
    ),
    "Y": (
        PICK_Y_TARGET_MIN,
        PICK_Y_TARGET_MAX
    ),
    "Z": (
        PICK_Z_TARGET_MIN,
        PICK_Z_TARGET_MAX
    )
}

motion_controller = {
    "active": False,
    "label": "",
    "commands": {},
    "start_targets": {},
    "start_raw_positions": {},
    "current_commands": {},
    "completion_targets": {},
    "tolerances": {},
    "settle_frames": 0,
    "completion_wait_frames": 0,
    "debug_frames": 0,
    "synchronized": False,
    "sync_total_frames": 0,
    "sync_current_frame": 0
}

missing_joint_state_log_set = set()


def clamp_axis_target(axis_name, value):
    minimum, maximum = AXIS_LIMITS[axis_name]

    return max(
        float(minimum),
        min(float(maximum), float(value))
    )


def get_joint_attr(axis_name, attr_name):
    joint_path = AXIS_JOINT_PATHS.get(
        axis_name
    )

    if joint_path is None:
        return None

    prim = get_prim(joint_path)

    if prim is None:
        return None

    attr = prim.GetAttribute(attr_name)

    if attr.IsValid():
        return attr

    return None


def get_axis_target_position(axis_name):
    attr = get_joint_attr(
        axis_name,
        TARGET_ATTR_NAME
    )

    if attr is None:
        return None

    try:
        return float(attr.Get())
    except Exception:
        return None


def set_axis_target_immediate(
    axis_name,
    target_position
):
    joint_path = AXIS_JOINT_PATHS.get(
        axis_name
    )

    if joint_path is None:
        event_log(
            f"[3축 오류] 알 수 없는 축: {axis_name}"
        )
        return False

    prim = get_prim(joint_path)

    if prim is None:
        event_log(
            f"[3축 오류] {axis_name} Joint 경로 없음 | "
            f"{joint_path}"
        )
        return False

    target_attr = prim.GetAttribute(
        TARGET_ATTR_NAME
    )

    if not target_attr.IsValid():
        event_log(
            f"[3축 오류] {axis_name} targetPosition 속성 없음 | "
            f"{joint_path}"
        )
        return False

    clamped_target = clamp_axis_target(
        axis_name,
        target_position
    )

    target_attr.Set(
        float(clamped_target)
    )

    velocity_attr = prim.GetAttribute(
        VELOCITY_ATTR_NAME
    )

    if velocity_attr.IsValid():
        try:
            velocity_attr.Set(0.0)
        except Exception:
            pass

    return True


def get_axis_actual_position(axis_name):
    """
    현재 모션 중에는 시작 Drive target과 Geometry raw 변화량을 합쳐
    Drive 좌표계의 실제 위치를 계산한다.

        actual = start_target + (raw_now - raw_start)

    이렇게 하면 Y=-21.04, Z=35.06처럼 Joint local frame의 고정 원점 차이를
    별도로 보정하지 않아도 된다.

    모션이 없을 때는 현재 Drive targetPosition을 반환한다.
    """
    current_target = get_axis_target_position(
        axis_name
    )

    if current_target is None:
        return None

    if not motion_controller.get(
        "active",
        False
    ):
        return float(current_target)

    start_targets = motion_controller.get(
        "start_targets",
        {}
    )
    start_raw_positions = motion_controller.get(
        "start_raw_positions",
        {}
    )

    if (
        axis_name not in start_targets
        or axis_name not in start_raw_positions
    ):
        return float(current_target)

    raw_now = get_axis_raw_geometry_position(
        axis_name
    )

    if raw_now is None:
        return float(current_target)

    return (
        float(start_targets[axis_name])
        + (
            float(raw_now)
            - float(start_raw_positions[axis_name])
        )
    )

def get_axis_actual_velocity(axis_name):
    joint_path = AXIS_JOINT_PATHS.get(
        axis_name
    )

    if joint_path is None:
        return None

    prim = get_prim(joint_path)

    if prim is None:
        return None

    for attr_name in JOINT_VELOCITY_ATTR_CANDIDATES:
        attr = prim.GetAttribute(attr_name)

        if not attr.IsValid():
            continue

        try:
            value = attr.Get()

            if isinstance(value, (int, float)):
                return float(value)
        except Exception:
            pass

    return None


def log_joint_position_sources():
    """
    축별 Drive target과 Geometry raw 값을 출력한다.
    완료 판정은 절대 raw 좌표가 아니라 이동 시작점 대비 변화량을 사용한다.
    """
    for axis_name in (
        "X1",
        "X2",
        "Y",
        "Z"
    ):
        target_value = get_axis_target_position(
            axis_name
        )
        raw_value = get_axis_raw_geometry_position(
            axis_name
        )

        target_text = (
            "None"
            if target_value is None
            else f"{float(target_value):.3f}"
        )
        raw_text = (
            "None"
            if raw_value is None
            else f"{float(raw_value):.3f}"
        )

        event_log(
            f"[3축 준비] {axis_name} | "
            f"drive={target_text} | raw={raw_text} | "
            "판독=상대 Geometry"
        )


def ensure_joint_limits():
    """
    현재 작업 좌표를 사용할 수 있도록 Prismatic Joint Limit을 확인/설정한다.
    """
    limit_values = {
        "X1": (
            PICK_X1_TARGET_MIN,
            PICK_X1_TARGET_MAX
        ),
        "X2": (
            PICK_X2_TARGET_MIN,
            PICK_X2_TARGET_MAX
        ),
        "Y": (
            PICK_Y_TARGET_MIN,
            PICK_Y_TARGET_MAX
        ),
        "Z": (
            PICK_Z_TARGET_MIN,
            PICK_Z_TARGET_MAX
        )
    }

    for axis_name, values in limit_values.items():
        joint_path = AXIS_JOINT_PATHS[
            axis_name
        ]

        prim = get_prim(joint_path)

        if prim is None:
            event_log(
                f"[Joint Limit 오류] {axis_name} 경로 없음 | "
                f"{joint_path}"
            )
            continue

        lower_value, upper_value = values

        lower_attr = prim.GetAttribute(
            "physics:lowerLimit"
        )
        upper_attr = prim.GetAttribute(
            "physics:upperLimit"
        )

        try:
            if lower_attr.IsValid():
                lower_attr.Set(
                    float(lower_value)
                )

            if upper_attr.IsValid():
                upper_attr.Set(
                    float(upper_value)
                )

            event_log(
                f"[Joint Limit] {axis_name} | "
                f"{lower_value:.1f} ~ {upper_value:.1f}"
            )

        except Exception as error:
            event_log(
                f"[Joint Limit 오류] {axis_name} | {error}"
            )


def s_curve_speed_ratio(progress):
    """
    위치 진행률 기반 속도 프로파일.
    시작과 끝은 느리고 중간 구간은 빠르게 움직인다.
    """
    progress = max(
        0.0,
        min(1.0, float(progress))
    )

    smooth_ratio = (
        math.sin(math.pi * progress) ** 2
    )

    return (
        float(S_CURVE_MIN_SPEED_RATIO)
        + (
            1.0
            - float(S_CURVE_MIN_SPEED_RATIO)
        )
        * smooth_ratio
    )


def start_scurve_motion(
    label,
    commands,
    completion_targets=None,
    tolerances=None,
    synchronized=False,
    duration_frames=None
):
    """
    Joint Drive targetPosition을 S-Curve로 갱신한다.

    completion_targets가 지정된 축은 Drive 명령값만 끝났다고 완료하지 않고,
    실제 Geometry raw 위치까지 확인한다.
    픽업 Y축처럼 targetPosition -50 명령 후 Geometry가 약 -70까지
    내려가야 하는 경우에 사용한다.
    """
    if motion_controller.get("active", False):
        event_log(f"[이동 오류] {label} | 다른 이동이 진행 중")
        return False

    if not isinstance(commands, dict) or not commands:
        event_log(f"[이동 오류] {label} | commands 형식 오류")
        return False

    normalized_commands = {}
    start_targets = {}
    start_raw_positions = {}
    current_commands = {}

    for axis_name, target_value in commands.items():
        if axis_name not in AXIS_JOINT_PATHS:
            event_log(f"[이동 오류] {label} | 알 수 없는 축 {axis_name}")
            return False

        current_target = get_axis_target_position(axis_name)

        if current_target is None:
            event_log(
                f"[이동 오류] {label} | "
                f"{axis_name} 현재 targetPosition 읽기 실패"
            )
            return False

        final_target = clamp_axis_target(axis_name, target_value)
        current_target = float(current_target)
        final_target = float(final_target)

        normalized_commands[axis_name] = final_target
        start_targets[axis_name] = current_target
        current_commands[axis_name] = current_target

        raw_position = get_axis_raw_geometry_position(axis_name)

        if raw_position is not None:
            start_raw_positions[axis_name] = float(raw_position)

    normalized_completion_targets = {}

    if isinstance(completion_targets, dict):
        for axis_name, target_value in completion_targets.items():
            if axis_name in normalized_commands:
                normalized_completion_targets[axis_name] = float(target_value)

    normalized_tolerances = {}

    if isinstance(tolerances, dict):
        for axis_name, tolerance_value in tolerances.items():
            if axis_name in normalized_commands:
                normalized_tolerances[axis_name] = max(
                    0.0,
                    float(tolerance_value)
                )

    for axis_name in normalized_completion_targets:
        if axis_name not in normalized_tolerances:
            normalized_tolerances[axis_name] = float(
                AXIS_POSITION_TOLERANCE.get(axis_name, 3.0)
            )

    motion_controller["active"] = True
    motion_controller["label"] = str(label)
    motion_controller["commands"] = normalized_commands
    motion_controller["start_targets"] = start_targets
    motion_controller["start_raw_positions"] = start_raw_positions
    motion_controller["current_commands"] = current_commands
    motion_controller["completion_targets"] = normalized_completion_targets
    motion_controller["tolerances"] = normalized_tolerances
    motion_controller["settle_frames"] = 0
    motion_controller["completion_wait_frames"] = 0
    motion_controller["debug_frames"] = 0
    motion_controller["synchronized"] = bool(synchronized)

    if bool(synchronized):
        if duration_frames is None:
            axis_step_table = (
                CARRY_AXIS_MAX_STEP
                if is_grip_attached
                else AXIS_MAX_STEP
            )
            estimated_frames = 1

            for axis_name, final_target in normalized_commands.items():
                distance = abs(
                    float(final_target)
                    - float(start_targets[axis_name])
                )
                max_step = max(
                    1e-6,
                    float(axis_step_table[axis_name])
                )
                estimated_frames = max(
                    estimated_frames,
                    int(math.ceil(distance / max_step))
                )

            duration_frames = max(30, int(estimated_frames * 1.5))

        motion_controller["sync_total_frames"] = max(
            1,
            int(duration_frames)
        )
        motion_controller["sync_current_frame"] = 0
    else:
        motion_controller["sync_total_frames"] = 0
        motion_controller["sync_current_frame"] = 0

    command_text = ", ".join(
        f"{axis}={value:.2f}"
        for axis, value in normalized_commands.items()
    )

    completion_text = ""

    if normalized_completion_targets:
        completion_text = " | 실제 완료(" + ", ".join(
            f"{axis}={value:.2f}±{normalized_tolerances[axis]:.2f}"
            for axis, value in normalized_completion_targets.items()
        ) + ")"

    sync_text = (
        f" | 동기 S-Curve={motion_controller['sync_total_frames']}프레임"
        if bool(synchronized)
        else ""
    )

    event_log(
        f"[그리퍼 이동] {label} | "
        f"Drive 목표({command_text})"
        f"{completion_text}"
        f"{sync_text}"
    )

    return True

def cancel_scurve_motion():
    motion_controller["active"] = False
    motion_controller["label"] = ""
    motion_controller["commands"] = {}
    motion_controller["start_targets"] = {}
    motion_controller["start_raw_positions"] = {}
    motion_controller["current_commands"] = {}
    motion_controller["completion_targets"] = {}
    motion_controller["tolerances"] = {}
    motion_controller["settle_frames"] = 0
    motion_controller["completion_wait_frames"] = 0
    motion_controller["debug_frames"] = 0
    motion_controller["synchronized"] = False
    motion_controller["sync_total_frames"] = 0
    motion_controller["sync_current_frame"] = 0


def update_scurve_motion():
    """
    매 프레임 Joint Drive targetPosition을 S-Curve로 갱신한다.

    완료 조건:
    1. 모든 Drive 명령이 최종값에 도달
    2. completion_targets가 있으면 실제 Geometry 위치도 도달
    3. 위 상태를 필요한 정착 프레임 동안 유지
    """
    if not motion_controller.get("active", False):
        return None

    commands = motion_controller["commands"]
    start_targets = motion_controller["start_targets"]
    start_raw_positions = motion_controller.get("start_raw_positions", {})
    current_commands = motion_controller["current_commands"]
    completion_targets = motion_controller.get("completion_targets", {})
    tolerances = motion_controller.get("tolerances", {})

    if is_grip_attached:
        # 기본 운반 속도표를 복사하여 특정 단계에서만 필요한 축을 조정한다.
        active_axis_max_step = dict(CARRY_AXIS_MAX_STEP)

        # 흡착 직후 Y축 상승 단계만 4.20을 사용한다.
        if robot_sequence.get("state") == "LIFT_Y":
            active_axis_max_step["Y"] = float(
                POST_PICK_LIFT_Y_MAX_STEP
            )

        required_settle_frames = CARRY_MOTION_SETTLE_FRAMES_REQUIRED
    else:
        active_axis_max_step = AXIS_MAX_STEP
        required_settle_frames = MOTION_COMMAND_SETTLE_FRAMES_REQUIRED

    synchronized_motion = bool(
        motion_controller.get("synchronized", False)
    )

    all_commands_finished = True

    if synchronized_motion:
        total_frames = max(
            1,
            int(motion_controller.get("sync_total_frames", 1))
        )
        current_frame = min(
            total_frames,
            int(motion_controller.get("sync_current_frame", 0)) + 1
        )
        motion_controller["sync_current_frame"] = current_frame

        progress = float(current_frame) / float(total_frames)
        progress = max(0.0, min(1.0, progress))
        blend = (
            10.0 * progress ** 3
            - 15.0 * progress ** 4
            + 6.0 * progress ** 5
        )

        all_commands_finished = current_frame >= total_frames

        for axis_name, final_target in commands.items():
            start_value = float(start_targets[axis_name])
            final_target = float(final_target)

            if all_commands_finished:
                next_value = final_target
            else:
                next_value = (
                    start_value
                    + (final_target - start_value) * blend
                )

            if not set_axis_target_immediate(axis_name, next_value):
                failed_label = motion_controller["label"]
                cancel_scurve_motion()
                event_log(
                    f"[이동 오류] {failed_label} | "
                    f"{axis_name} targetPosition 설정 실패"
                )
                return None

            current_commands[axis_name] = float(next_value)
    else:
        for axis_name, final_target in commands.items():
            start_value = float(start_targets[axis_name])
            current_value = float(current_commands[axis_name])
            final_target = float(final_target)
            remaining = final_target - current_value

            if abs(remaining) <= 1e-6:
                next_value = final_target
            else:
                all_commands_finished = False
                total_distance = abs(final_target - start_value)

                if total_distance <= 1e-9:
                    progress = 1.0
                else:
                    progress = abs(current_value - start_value) / total_distance

                speed_ratio = s_curve_speed_ratio(progress)
                max_step = float(active_axis_max_step[axis_name]) * speed_ratio
                step_value = min(abs(remaining), max_step)
                direction = 1.0 if remaining > 0.0 else -1.0
                next_value = current_value + direction * step_value

            if not set_axis_target_immediate(axis_name, next_value):
                failed_label = motion_controller["label"]
                cancel_scurve_motion()
                event_log(
                    f"[이동 오류] {failed_label} | "
                    f"{axis_name} targetPosition 설정 실패"
                )
                return None

            current_commands[axis_name] = float(next_value)

            if abs(float(next_value) - final_target) > 1e-6:
                all_commands_finished = False

    actual_position_dict = {}
    all_actual_targets_reached = True

    if all_commands_finished and completion_targets:
        motion_controller["completion_wait_frames"] += 1

        for axis_name, completion_target in completion_targets.items():
            raw_start = start_raw_positions.get(axis_name, None)
            raw_now = get_axis_raw_geometry_position(axis_name)
            start_target = start_targets.get(axis_name, None)

            if raw_now is None or start_target is None:
                actual_value = None
                reached = False
            else:
                # completion_targets 값은 사용자가 Isaac에서 확인한
                # Geometry raw 좌표 기준이다. Y=-70 조건도 raw 값 그대로 비교한다.
                actual_value = float(raw_now)

                tolerance = float(tolerances.get(axis_name, 3.0))
                # completion_target은 Geometry raw 좌표이고
                # start_target은 Drive 좌표이므로 두 값을 직접 빼면 안 된다.
                # 실제 이동 방향은 Drive 최종 명령과 Drive 시작값으로 판단한다.
                command_target = commands.get(
                    axis_name,
                    start_target
                )

                move_direction = (
                    float(command_target)
                    - float(start_target)
                )

                if actual_value is None:
                    reached = False
                elif move_direction < 0.0:
                    reached = (
                        actual_value
                        <= float(completion_target) + tolerance
                    )
                elif move_direction > 0.0:
                    reached = (
                        actual_value
                        >= float(completion_target) - tolerance
                    )
                else:
                    reached = (
                        abs(actual_value - float(completion_target))
                        <= tolerance
                    )

            actual_position_dict[axis_name] = actual_value

            if not reached:
                all_actual_targets_reached = False

        if (
            motion_controller["completion_wait_frames"]
            > int(MOTION_ACTUAL_WAIT_TIMEOUT_FRAMES)
        ):
            failed_label = motion_controller["label"]
            actual_text = ", ".join(
                f"{axis}={value}"
                for axis, value in actual_position_dict.items()
            )

            cancel_scurve_motion()
            abort_robot_sequence(
                f"{failed_label} 실제 위치 미도달 | {actual_text}"
            )
            return None
    else:
        motion_controller["completion_wait_frames"] = 0

    if all_commands_finished and all_actual_targets_reached:
        motion_controller["settle_frames"] += 1
    else:
        motion_controller["settle_frames"] = 0

    motion_controller["debug_frames"] += 1

    if motion_controller["debug_frames"] % 30 == 0:
        command_text = ", ".join(
            f"{axis}={value:.2f}"
            for axis, value in current_commands.items()
        )

        raw_items = []

        for axis_name in commands:
            raw_value = get_axis_raw_geometry_position(axis_name)

            if raw_value is not None:
                raw_items.append(f"{axis_name}={float(raw_value):.2f}")

        raw_text = ", ".join(raw_items) if raw_items else "판독 없음"

        actual_items = []

        for axis_name, target_value in completion_targets.items():
            actual_value = actual_position_dict.get(axis_name, None)
            actual_text = (
                "None"
                if actual_value is None
                else f"{float(actual_value):.2f}"
            )
            actual_items.append(
                f"{axis_name}={actual_text}/{float(target_value):.2f}"
            )

        actual_text = (
            " | 실제(" + ", ".join(actual_items) + ")"
            if actual_items
            else ""
        )

        event_log(
            f"[이동 중] {motion_controller['label']} | "
            f"Drive({command_text}) | "
            f"Geometry({raw_text})"
            f"{actual_text} | "
            f"정착={motion_controller['settle_frames']}/"
            f"{required_settle_frames}"
        )

    if motion_controller["settle_frames"] < required_settle_frames:
        return None

    completed_label = motion_controller["label"]

    completed_positions = {
        axis_name: float(value)
        for axis_name, value in current_commands.items()
    }

    for axis_name, actual_value in actual_position_dict.items():
        if actual_value is not None:
            completed_positions[f"{axis_name}_actual"] = float(actual_value)

    position_text = ", ".join(
        f"{axis}={value:.2f}"
        for axis, value in completed_positions.items()
    )

    cancel_scurve_motion()

    event_log(
        f"[이동 완료] {completed_label} | "
        f"최종값({position_text})"
    )

    return completed_label, completed_positions


def copy_world_position(position):
    if position is None:
        return None

    return Gf.Vec3d(
        float(position[0]),
        float(position[1]),
        float(position[2])
    )


def calculate_dynamic_pick_targets(
    stopped_world_position
):
    """
    정지한 박스의 실제 World X를 이용해 X1/X2 픽업 목표를 계산한다.

    Depth X → Isaac X → 고정 Z 정지 → 실제 정지 Box X
    → X1/X2 Joint targetPosition 보정
    """
    if stopped_world_position is None:
        return None

    box_world_x = (
        float(stopped_world_position[0])
        + float(PICK_WORLD_X_OFFSET)
    )

    delta_world_x = (
        box_world_x
        - float(PICK_REFERENCE_BOX_WORLD_X)
    )

    joint_delta_x = (
        delta_world_x
        * float(PICK_WORLD_X_TO_JOINT_SCALE)
        * float(PICK_WORLD_X_TO_JOINT_SIGN)
    )

    target_x1 = clamp_axis_target(
        "X1",
        float(AUTO_PICK_X1_POSITION)
        + joint_delta_x
    )

    target_x2 = clamp_axis_target(
        "X2",
        float(AUTO_PICK_X2_POSITION)
        + joint_delta_x
    )

    return {
        "box_world_x": float(box_world_x),
        "reference_world_x": float(
            PICK_REFERENCE_BOX_WORLD_X
        ),
        "delta_world_x": float(delta_world_x),
        "joint_delta_x": float(joint_delta_x),
        "x1": float(target_x1),
        "x2": float(target_x2),
        "z": float(AUTO_PICK_Z_POSITION)
    }


# 기존 키 입력 함수 호환
def move_x_to_target():
    return start_scurve_motion(
        "수동 X1 이동",
        {
            "X1": X_TARGET_POSITION
        }
    )


def move_y_down():
    return start_scurve_motion(
        "수동 Y 하강",
        {
            "Y": Y_DOWN_POSITION
        },
        completion_targets={
            "Y": AUTO_PICK_Y_REACHED_POSITION
        },
        tolerances={
            "Y": PICK_Y_REACHED_TOLERANCE
        }
    )


def move_y_mid():
    return start_scurve_motion(
        "수동 Y 적재 위치",
        {
            "Y": Y_MID_POSITION
        }
    )


def move_y_home():
    return start_scurve_motion(
        "수동 Y 원점",
        {
            "Y": Y_HOME_POSITION
        }
    )


def move_z_to_target():
    return start_scurve_motion(
        "수동 Z 이동",
        {
            "Z": Z_TARGET_POSITION
        }
    )


# ==================================================
# [13. No-Jump 그리퍼]
# ==================================================

def set_box_rigidbody_enabled(box_path, enabled):
    prim = get_prim(box_path)

    if prim is None:
        return

    if not prim.IsA(UsdGeom.Xformable):
        return

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)

    rb = UsdPhysics.RigidBodyAPI(prim)

    try:
        rb.CreateRigidBodyEnabledAttr().Set(bool(enabled))
    except:
        pass


def set_box_collision_enabled(
    box_path,
    enabled,
    restore_saved_state=False
):
    """
    흡착 중 박스 Collision을 꺼서 그리퍼/프레임과의 물리 충돌로 인한
    흔들림과 튕김을 막는다.

    해제 시에는 흡착 전 Collision 상태를 복구한다.
    """
    global attached_box_collision_state_dict

    root_prim = get_prim(box_path)

    if root_prim is None:
        return False

    changed = False

    try:
        for prim in Usd.PrimRange(root_prim):
            attr = prim.GetAttribute(
                "physics:collisionEnabled"
            )

            has_collision = (
                prim.HasAPI(UsdPhysics.CollisionAPI)
                or attr.IsValid()
            )

            if not has_collision:
                continue

            prim_path = str(
                prim.GetPath()
            )

            if not enabled:
                current_value = (
                    attr.Get()
                    if attr.IsValid()
                    else True
                )

                if current_value is None:
                    current_value = True

                attached_box_collision_state_dict[
                    prim_path
                ] = bool(current_value)

                collision_api = (
                    UsdPhysics.CollisionAPI.Apply(
                        prim
                    )
                )

                collision_api.CreateCollisionEnabledAttr().Set(
                    False
                )

            else:
                collision_api = (
                    UsdPhysics.CollisionAPI.Apply(
                        prim
                    )
                )

                if restore_saved_state:
                    target_value = (
                        attached_box_collision_state_dict.get(
                            prim_path,
                            True
                        )
                    )
                else:
                    target_value = True

                collision_api.CreateCollisionEnabledAttr().Set(
                    bool(target_value)
                )

            changed = True

        if enabled:
            attached_box_collision_state_dict.clear()

        return changed

    except Exception as error:
        event_log(
            f"[박스 Collision 설정 오류] "
            f"target={box_path} | enabled={enabled} | {error}"
        )
        return False


def stop_box_velocity(box_path):
    prim = get_prim(box_path)

    if prim is None:
        return

    if not prim.IsA(UsdGeom.Xformable):
        return

    rb = UsdPhysics.RigidBodyAPI(prim)

    try:
        rb.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rb.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    except:
        pass


def get_box_world_bbox(box_path):
    prim = get_prim(box_path)

    if prim is None:
        return None

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy
        ]
    )

    bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()

    return bbox.GetMin(), bbox.GetMax()


def get_prim_world_bbox_center(prim_path):
    """Prim의 실제 World BBox 중심을 반환한다."""
    bbox_result = get_box_world_bbox(prim_path)

    if bbox_result is None:
        return None

    min_pt, max_pt = bbox_result

    return Gf.Vec3d(
        (float(min_pt[0]) + float(max_pt[0])) * 0.5,
        (float(min_pt[1]) + float(max_pt[1])) * 0.5,
        (float(min_pt[2]) + float(max_pt[2])) * 0.5
    )


def get_tray_place_target_position():
    """
    Tray1 Prim 원점이 아니라 실제 형상의 World BBox 중심을 적재 목표로 사용한다.
    BBox를 읽지 못할 때만 Prim 원점으로 fallback한다.
    """
    tray_center = get_prim_world_bbox_center(TRAY_PATH)

    if tray_center is None:
        tray_center = get_world_position(TRAY_PATH)

    if tray_center is None:
        return None

    return Gf.Vec3d(
        float(tray_center[0]) + float(TRAY_PLACE_OFFSET_WORLD_X),
        float(tray_center[1]),
        float(tray_center[2]) + float(TRAY_PLACE_OFFSET_WORLD_Z)
    )


def get_attached_box_world_center():
    if not is_grip_attached or not attached_box_path:
        return None

    return get_prim_world_bbox_center(attached_box_path)


def get_tray_box_alignment_error(tray_target_position, box_center_position):
    """
    트레이 중심 - 박스 중심 오차를 X1 및 Z Joint의 실제 World 축에 투영한다.
    반환되는 x_error/z_error가 0에 가까울수록 트레이 중심 정렬 상태다.
    """
    if tray_target_position is None or box_center_position is None:
        return None

    delta_world = Gf.Vec3d(
        float(tray_target_position[0]) - float(box_center_position[0]),
        float(tray_target_position[1]) - float(box_center_position[1]),
        float(tray_target_position[2]) - float(box_center_position[2])
    )

    x_axis_world = get_prismatic_joint_world_axis("X1")
    z_axis_world = get_prismatic_joint_world_axis("Z")

    if x_axis_world is not None:
        x_error = float(Gf.Dot(delta_world, x_axis_world))
    else:
        x_error = float(delta_world[0])

    if z_axis_world is not None:
        z_error = float(Gf.Dot(delta_world, z_axis_world))
    else:
        z_error = float(delta_world[2])

    return {
        "delta_world": delta_world,
        "x_error": float(x_error),
        "z_error": float(z_error),
        "centered": (
            abs(float(x_error)) <= float(TRAY_BOX_CENTER_TOLERANCE_X)
            and abs(float(z_error)) <= float(TRAY_BOX_CENTER_TOLERANCE_Z)
        )
    }


resolved_suction_center_path = None
suction_path_log_done = False


def resolve_suction_center_path():
    """
    실제 Stage에 존재하는 흡착 기준 Prim을 자동 선택한다.

    우선순위:
    1. /assembly714/Gripper/gripper_surface
    2. /assembly714/Gripper/gripper_surface/Body1

    /assembly714/Gripper 본체 원점은 사용하지 않으므로,
    잘못된 기준점 때문에 박스가 그리퍼 내부로 들어가는 문제를 방지한다.
    """
    global resolved_suction_center_path
    global suction_path_log_done

    # 이미 선택한 경로가 아직 유효하면 그대로 사용한다.
    if (
        resolved_suction_center_path is not None
        and get_prim(resolved_suction_center_path) is not None
    ):
        return resolved_suction_center_path

    resolved_suction_center_path = None

    for candidate_path in SUCTION_CENTER_PATH_CANDIDATES:
        if get_prim(candidate_path) is not None:
            resolved_suction_center_path = candidate_path
            break

    if resolved_suction_center_path is None:
        if not suction_path_log_done:
            event_log(
                "[그리퍼 기준점 오류] 후보 Prim을 찾지 못함 | "
                + " | ".join(SUCTION_CENTER_PATH_CANDIDATES)
            )
            suction_path_log_done = True

        return None

    if not suction_path_log_done:
        suction_world = get_world_position(
            resolved_suction_center_path
        )

        if suction_world is None:
            position_text = "World XYZ 읽기 실패"
        else:
            position_text = (
                f"World XYZ=("
                f"{float(suction_world[0]):.3f}, "
                f"{float(suction_world[1]):.3f}, "
                f"{float(suction_world[2]):.3f})"
            )

        event_log(
            f"[실제 흡착 기준점 선택] "
            f"{resolved_suction_center_path} | "
            f"{position_text}"
        )

        suction_path_log_done = True

    return resolved_suction_center_path


def print_suction_reference_status():
    """
    Script Editor 수동 확인용:

        print_suction_reference_status()
    """
    path = resolve_suction_center_path()

    debug_print("===== 흡착 기준점 확인 =====")
    debug_print("선택 경로:", path)

    if path is None:
        debug_print("상태: Prim 없음")
        return False

    position = get_world_position(path)

    if position is None:
        debug_print("상태: World 좌표 읽기 실패")
        return False

    debug_print(
        "World XYZ:",
        tuple(
            round(float(value), 4)
            for value in position
        )
    )
    debug_print(
        "Y Offset:",
        float(SUCTION_CONTACT_Y_OFFSET)
    )
    debug_print(
        "실제 판정 Y:",
        round(
            float(position[1])
            + float(SUCTION_CONTACT_Y_OFFSET),
            4
        )
    )

    return True


def get_suction_contact_position():
    suction_path = resolve_suction_center_path()

    if suction_path is None:
        return None

    suction_pos = get_world_position(
        suction_path
    )

    if suction_pos is None:
        return None

    # 현재 장면의 높이축은 Y
    return Gf.Vec3d(
        float(suction_pos[0]),
        float(suction_pos[1])
        + float(SUCTION_CONTACT_Y_OFFSET),
        float(suction_pos[2])
    )


def is_suction_on_box_top(box_path):
    suction_pos = get_suction_contact_position()
    bbox_result = get_box_world_bbox(
        box_path
    )

    if suction_pos is None or bbox_result is None:
        return (
            False,
            suction_pos,
            None,
            None
        )

    min_pt, max_pt = bbox_result

    # 높이축이 Y이므로 박스 윗면은 max Y
    top_y = float(max_pt[1])

    x_min = (
        float(min_pt[0])
        - TOP_SURFACE_MARGIN
    )
    x_max = (
        float(max_pt[0])
        + TOP_SURFACE_MARGIN
    )

    z_min = (
        float(min_pt[2])
        - TOP_SURFACE_MARGIN
    )
    z_max = (
        float(max_pt[2])
        + TOP_SURFACE_MARGIN
    )

    in_x = (
        x_min
        <= float(suction_pos[0])
        <= x_max
    )

    in_z = (
        z_min
        <= float(suction_pos[2])
        <= z_max
    )

    vertical_gap = abs(
        float(suction_pos[1])
        - top_y
    )

    y_ok = (
        vertical_gap
        <= MAX_VERTICAL_GAP
    )

    top_point = Gf.Vec3d(
        suction_pos[0],
        top_y,
        suction_pos[2]
    )

    return (
        in_x and in_z and y_ok,
        suction_pos,
        top_point,
        vertical_gap
    )


def close_grip_no_jump(target_box=None, force=False):
    global is_grip_attached
    global suction_world_at_attach
    global box_local_at_attach
    global attached_box_path
    global attached_box_yaw_at_attach
    global attached_gripper_logical_yaw_at_attach

    if target_box is None:
        target_box = get_current_target_box()

    if not target_box:
        debug_print("[오류] 스폰된 박스가 없어 잡을 대상이 없습니다.")
        return False

    ok, suction_pos, top_point, vertical_gap = is_suction_on_box_top(target_box)
    box_local = get_local_translate(target_box)

    if suction_pos is None or box_local is None:
        debug_print("[오류] 그리퍼 또는 박스 좌표를 읽지 못했습니다.")
        return False

    debug_print("==================================================")
    debug_print("[그리퍼 흡착 검사]")
    debug_print("타겟:", target_box)
    debug_print("Suction:", suction_pos)
    debug_print("Top Point:", top_point)
    debug_print("Gap:", vertical_gap)

    if not ok and not force:
        event_log(
            f"[그리퍼 흡착 실패] "
            f"박스 범위 밖 | target={target_box} | "
            f"gap={vertical_gap}"
        )
        return False

    if not ok and force:
        event_log(
            f"[그리퍼] 보정된 픽업 위치 도달로 강제 흡착 | "
            f"target={target_box} | gap={vertical_gap}"
        )

    stop_box_velocity(target_box)

    # 흡착 중에는 박스 물리와 Collision을 모두 끄고
    # 그리퍼 Transform에 직접 고정한다.
    # 물리 하중으로 떨어지거나 흔들리지 않는 강력 고정 방식이다.
    set_box_collision_enabled(
        target_box,
        False
    )
    set_box_rigidbody_enabled(
        target_box,
        False
    )

    # --------------------------------------------------
    # No-Jump / No-Penetration 흡착
    # --------------------------------------------------
    # 흡착이 허용된 현재 박스 위치를 그대로 저장한다.
    # 박스를 흡착점으로 순간 이동시키지 않으므로,
    # 그리퍼 내부로 뚫고 들어가는 현상을 방지한다.
    suction_world_at_attach = Gf.Vec3d(
        float(suction_pos[0]),
        float(suction_pos[1]),
        float(suction_pos[2])
    )

    box_local_at_attach = Gf.Vec3d(
        float(box_local[0]),
        float(box_local[1]),
        float(box_local[2])
    )

    attached_box_yaw_at_attach = float(
        box_yaw_dict.get(
            target_box,
            0.0
        )
    )

    current_gripper_logical_yaw = (
        get_gripper_joint_logical_angle()
    )

    if current_gripper_logical_yaw is None:
        current_gripper_logical_yaw = (
            calculate_gripper_logical_target(
                attached_box_yaw_at_attach
            )
        )

    attached_gripper_logical_yaw_at_attach = float(
        current_gripper_logical_yaw
    )

    attached_box_path = target_box
    is_grip_attached = True

    event_log(
        f"[그리퍼] 강력 고정 흡착 연결 | "
        f"target={target_box} | "
        f"현재간격={float(vertical_gap):.2f} | "
        "RigidBody=OFF | Collision=OFF | 위치보정=없음 | "
        f"박스각={attached_box_yaw_at_attach:.2f}도 | "
        f"그리퍼각={attached_gripper_logical_yaw_at_attach:.2f}도 | "
        f"suction={resolve_suction_center_path()}"
    )

    return True


def open_grip_no_jump():
    global is_grip_attached
    global attached_box_path
    global attached_box_yaw_at_attach
    global attached_gripper_logical_yaw_at_attach

    if is_grip_attached and attached_box_path:
        stop_box_velocity(
            attached_box_path
        )

        # 먼저 Collision 상태를 복구한 다음 RigidBody를 다시 켠다.
        set_box_collision_enabled(
            attached_box_path,
            True,
            restore_saved_state=True
        )

        set_box_rigidbody_enabled(
            attached_box_path,
            True
        )

        stop_box_velocity(
            attached_box_path
        )

    released_box_path = attached_box_path

    is_grip_attached = False
    attached_box_path = None
    attached_box_yaw_at_attach = 0.0
    attached_gripper_logical_yaw_at_attach = 0.0

    event_log(
        f"[그리퍼] 안정 흡착 해제 | "
        f"target={released_box_path} | "
        "Collision/RigidBody 복구"
    )

    return released_box_path



def set_world_position_keep_orientation(path, target_world_position):
    """현재 회전/스케일은 유지하고 Prim의 World 위치만 이동한다."""
    current_world_position = get_world_position(path)
    current_local_position = get_local_translate(path)

    if current_world_position is None or current_local_position is None:
        return False

    world_delta = Gf.Vec3d(
        float(target_world_position[0]) - float(current_world_position[0]),
        float(target_world_position[1]) - float(current_world_position[1]),
        float(target_world_position[2]) - float(current_world_position[2])
    )

    local_delta = world_delta_to_parent_local_delta(path, world_delta)

    target_local_position = Gf.Vec3d(
        float(current_local_position[0]) + float(local_delta[0]),
        float(current_local_position[1]) + float(local_delta[1]),
        float(current_local_position[2]) + float(local_delta[2])
    )

    return set_local_translate(path, target_local_position)


def get_rigidbody_world_velocity(path):
    prim = get_prim(path)

    if prim is None or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return None

    try:
        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        velocity = rigid_body.GetVelocityAttr().Get()

        if velocity is None:
            return None

        return Gf.Vec3d(
            float(velocity[0]),
            float(velocity[1]),
            float(velocity[2])
        )
    except Exception:
        return None


def set_rigidbody_world_velocity(path, velocity):
    prim = get_prim(path)

    if prim is None:
        return False

    try:
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(prim)

        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        rigid_body.CreateVelocityAttr().Set(
            Gf.Vec3f(
                float(velocity[0]),
                float(velocity[1]),
                float(velocity[2])
            )
        )
        rigid_body.CreateAngularVelocityAttr().Set(
            Gf.Vec3f(0.0, 0.0, 0.0)
        )
        return True
    except Exception as error:
        event_log(
            f"[박스 속도 전달 오류] target={path} | {error}"
        )
        return False


def get_pack_conveyor_world_velocity_fallback():
    """포장 컨베이어 local X 속도를 World 속도 벡터로 변환한다."""
    conveyor_prim = get_prim(PACK_CONVEYOR_PATH)

    if conveyor_prim is None:
        return Gf.Vec3d(0.0, 0.0, 0.0)

    try:
        world_matrix = UsdGeom.Xformable(
            conveyor_prim
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

        world_axis = world_matrix.TransformDir(
            Gf.Vec3d(1.0, 0.0, 0.0)
        )
        axis_length = world_axis.GetLength()

        if axis_length <= 1e-9:
            return Gf.Vec3d(0.0, 0.0, 0.0)

        world_axis = world_axis / axis_length
        speed = (
            float(PACK_CONVEYOR_RUN_SPEED)
            if is_pack_conveyor_running
            else 0.0
        )

        return Gf.Vec3d(
            float(world_axis[0]) * speed,
            float(world_axis[1]) * speed,
            float(world_axis[2]) * speed
        )
    except Exception:
        return Gf.Vec3d(0.0, 0.0, 0.0)


def choose_tray_release_velocity():
    """실제 Tray 속도 → 위치 추정 속도 → 컨베이어 설정 속도 순으로 선택한다."""
    candidates = [
        get_rigidbody_world_velocity(TRAY_PATH),
        release_settle_controller.get("estimated_velocity", None),
        get_pack_conveyor_world_velocity_fallback()
    ]

    for velocity in candidates:
        if velocity is None:
            continue

        speed = math.sqrt(
            float(velocity[0]) * float(velocity[0])
            + float(velocity[1]) * float(velocity[1])
            + float(velocity[2]) * float(velocity[2])
        )

        if (
            speed >= float(TRAY_RELEASE_MIN_VALID_SPEED)
            and speed <= float(TRAY_RELEASE_MAX_VALID_SPEED)
        ):
            return Gf.Vec3d(
                float(velocity[0]),
                float(velocity[1]),
                float(velocity[2])
            )

    return Gf.Vec3d(0.0, 0.0, 0.0)


def cancel_release_settle(restore_physics=True, clear_completed=True):
    box_path = release_settle_controller.get("box_path", None)

    if restore_physics and box_path and get_prim(box_path) is not None:
        set_box_collision_enabled(
            box_path,
            True,
            restore_saved_state=True
        )
        set_box_rigidbody_enabled(box_path, True)

    release_settle_controller["active"] = False

    if clear_completed:
        release_settle_controller["completed"] = False

    release_settle_controller["frames"] = 0
    release_settle_controller["box_path"] = None
    release_settle_controller["reference_tray_position"] = None
    release_settle_controller["reference_box_world_position"] = None
    release_settle_controller["last_tray_position"] = None
    release_settle_controller["last_sample_time"] = None
    release_settle_controller["estimated_velocity"] = Gf.Vec3d(
        0.0, 0.0, 0.0
    )
    release_settle_controller["matched_velocity"] = Gf.Vec3d(
        0.0, 0.0, 0.0
    )


def start_release_settle_on_moving_tray():
    """
    그리퍼 Hard-Lock만 해제하고 박스 물리는 잠시 OFF 상태로 유지한다.
    이후 update_release_settle()이 박스를 트레이 변위에 고정한 뒤 물리를 복구한다.
    """
    global is_grip_attached
    global attached_box_path
    global attached_box_yaw_at_attach
    global attached_gripper_logical_yaw_at_attach

    if not is_grip_attached or not attached_box_path:
        event_log("[이동 트레이 흡착 해제 오류] 흡착된 박스가 없음")
        return None

    box_path = attached_box_path
    tray_position = get_tray_place_target_position()
    box_world_position = get_world_position(box_path)

    if tray_position is None or box_world_position is None:
        event_log(
            "[이동 트레이 흡착 해제 오류] Tray 또는 Box 좌표 읽기 실패"
        )
        return None

    # 기존 release controller가 남아 있으면 안전하게 초기화한다.
    cancel_release_settle(
        restore_physics=False,
        clear_completed=True
    )

    # 박스는 현재 위치 그대로 유지한다. 여기서는 RigidBody와 Collision을
    # 복구하지 않으므로 트레이/그리퍼 충돌로 튀지 않는다.
    stop_box_velocity(box_path)

    release_settle_controller["active"] = True
    release_settle_controller["completed"] = False
    release_settle_controller["frames"] = 0
    release_settle_controller["box_path"] = box_path
    release_settle_controller["reference_tray_position"] = copy_world_position(
        tray_position
    )
    release_settle_controller["reference_box_world_position"] = copy_world_position(
        box_world_position
    )
    release_settle_controller["last_tray_position"] = copy_world_position(
        tray_position
    )
    release_settle_controller["last_sample_time"] = time.perf_counter()
    release_settle_controller["estimated_velocity"] = Gf.Vec3d(
        0.0, 0.0, 0.0
    )

    is_grip_attached = False
    attached_box_path = None
    attached_box_yaw_at_attach = 0.0
    attached_gripper_logical_yaw_at_attach = 0.0

    event_log(
        f"[흡착 해제] 시작 | target={box_path} | "
        f"최소 {TRAY_RELEASE_LOCK_FRAMES_REQUIRED}프레임 동안 Tray에 고정 | "
        f"Y축이 적재 위치에서 {SAFE_RELEASE_LIFT_Y:.1f} 이상 상승한 뒤 물리 복구"
    )

    return box_path


def update_release_settle():
    if not release_settle_controller.get("active", False):
        return False

    box_path = release_settle_controller.get("box_path", None)
    reference_tray_position = release_settle_controller.get(
        "reference_tray_position", None
    )
    reference_box_world_position = release_settle_controller.get(
        "reference_box_world_position", None
    )

    if (
        not box_path
        or get_prim(box_path) is None
        or reference_tray_position is None
        or reference_box_world_position is None
    ):
        cancel_release_settle(
            restore_physics=True,
            clear_completed=True
        )
        event_log("[이동 트레이 안착 오류] 저장된 Box/Tray 정보 없음")
        return False

    current_tray_position = get_tray_place_target_position()

    if current_tray_position is None:
        cancel_release_settle(
            restore_physics=True,
            clear_completed=True
        )
        event_log("[이동 트레이 안착 오류] 현재 Tray 중심 읽기 실패")
        return False

    # 트레이가 이동한 만큼만 박스의 현재 World 위치를 함께 이동한다.
    tray_delta = Gf.Vec3d(
        float(current_tray_position[0]) - float(reference_tray_position[0]),
        float(current_tray_position[1]) - float(reference_tray_position[1]),
        float(current_tray_position[2]) - float(reference_tray_position[2])
    )

    target_box_world_position = Gf.Vec3d(
        float(reference_box_world_position[0]) + float(tray_delta[0]),
        float(reference_box_world_position[1]) + float(tray_delta[1]),
        float(reference_box_world_position[2]) + float(tray_delta[2])
    )

    if not set_world_position_keep_orientation(
        box_path,
        target_box_world_position
    ):
        cancel_release_settle(
            restore_physics=True,
            clear_completed=True
        )
        event_log("[이동 트레이 안착 오류] 박스 위치 고정 실패")
        return False

    # 실제 Tray 위치 변화로 속도를 추정한다.
    now = time.perf_counter()
    last_time = release_settle_controller.get("last_sample_time", None)
    last_tray_position = release_settle_controller.get(
        "last_tray_position", None
    )

    if last_time is not None and last_tray_position is not None:
        delta_time = float(now) - float(last_time)

        if delta_time > 1e-4:
            estimated_velocity = Gf.Vec3d(
                (
                    float(current_tray_position[0])
                    - float(last_tray_position[0])
                ) / delta_time,
                (
                    float(current_tray_position[1])
                    - float(last_tray_position[1])
                ) / delta_time,
                (
                    float(current_tray_position[2])
                    - float(last_tray_position[2])
                ) / delta_time
            )

            estimated_speed = estimated_velocity.GetLength()

            if estimated_speed <= float(TRAY_RELEASE_MAX_VALID_SPEED):
                release_settle_controller["estimated_velocity"] = (
                    estimated_velocity
                )

    release_settle_controller["last_sample_time"] = now
    release_settle_controller["last_tray_position"] = copy_world_position(
        current_tray_position
    )
    release_settle_controller["frames"] += 1

    release_frames = int(release_settle_controller["frames"])

    # 해제 직후 지정된 프레임 동안 잔류 속도를 완전히 제거한다.
    if release_frames <= int(SAFE_RELEASE_ZERO_VELOCITY_FRAMES):
        stop_box_velocity(box_path)

    # v41:
    # 고정 프레임 수만 보고 물리를 복구하지 않는다.
    # Y축 Drive가 상승 위치까지 도달했는지 확인하여 그리퍼가 박스에서
    # 충분히 떨어진 뒤에만 Collision/RigidBody를 다시 활성화한다.
    y_command_position = get_axis_target_position("Y")

    # 적재 명령 위치에서 SAFE_RELEASE_LIFT_Y만큼만 상승하면
    # 그리퍼와 박스가 이격되었다고 판단한다.
    safe_release_y_threshold = (
        float(AUTO_PLACE_Y_COMMAND_POSITION)
        + float(SAFE_RELEASE_LIFT_Y)
    )

    y_command_clear = (
        y_command_position is not None
        and float(y_command_position)
        >= (
            safe_release_y_threshold
            - float(TRAY_RELEASE_SAFE_Y_COMMAND_TOLERANCE)
        )
    )

    required_hold_frames = max(
        int(TRAY_RELEASE_LOCK_FRAMES_REQUIRED),
        int(SAFE_RELEASE_ZERO_VELOCITY_FRAMES)
        + int(SAFE_RELEASE_SETTLE_FRAMES)
    )

    minimum_hold_complete = (
        release_frames >= required_hold_frames
    )

    # Y targetPosition을 읽지 못하는 특수 상황에서만 최대 프레임을 fallback으로 사용한다.
    fallback_clear = (
        y_command_position is None
        and release_frames >= int(TRAY_RELEASE_MAX_LOCK_FRAMES)
    )

    if not (
        minimum_hold_complete
        and (y_command_clear or fallback_clear)
    ):
        return True

    matched_velocity = (
        choose_tray_release_velocity()
        if bool(TRAY_RELEASE_MATCH_TRAY_VELOCITY)
        else Gf.Vec3d(0.0, 0.0, 0.0)
    )

    # 박스는 현재 트레이 위 위치를 유지한 채 Collision → RigidBody 순서로 복구한다.
    set_box_collision_enabled(
        box_path,
        True,
        restore_saved_state=True
    )
    set_box_rigidbody_enabled(box_path, True)
    set_rigidbody_world_velocity(box_path, matched_velocity)

    release_settle_controller["matched_velocity"] = matched_velocity
    release_settle_controller["active"] = False
    release_settle_controller["completed"] = True

    event_log(
        f"[흡착 해제] 완료 | target={box_path} | "
        f"해제고정={release_frames}프레임 | "
        f"Y command={float(y_command_position) if y_command_position is not None else 'None'} | "
        f"Tray 속도 전달=({float(matched_velocity[0]):.2f}, "
        f"{float(matched_velocity[1]):.2f}, "
        f"{float(matched_velocity[2]):.2f}) | "
        "그리퍼 이격 후 Collision/RigidBody 복구"
    )

    return True


def update_attached_box_hard_lock():
    """
    흡착된 박스를 매 프레임 그리퍼 기준 위치/각도에 정확히 고정한다.

    RigidBody와 Collision이 OFF인 동안 Transform을 직접 갱신하므로,
    중력·관성·충돌로 흔들리거나 떨어지지 않는다.
    """
    if not is_grip_attached or not attached_box_path:
        return False

    if get_prim(attached_box_path) is None:
        return False

    suction_now = get_suction_contact_position()

    if suction_now is None:
        return False

    delta_world = (
        suction_now
        - suction_world_at_attach
    )

    delta_local = (
        world_delta_to_parent_local_delta(
            attached_box_path,
            delta_world
        )
    )

    target_local = Gf.Vec3d(
        box_local_at_attach[0]
        + delta_local[0],
        box_local_at_attach[1]
        + delta_local[1],
        box_local_at_attach[2]
        + delta_local[2]
    )

    position_ok = set_local_translate(
        attached_box_path,
        target_local
    )

    rotation_ok = (
        sync_attached_box_rotation_to_gripper()
    )

    return bool(
        position_ok or rotation_ok
    )


# ==================================================
# [실제 그리퍼 Revolute Joint Yaw S-Curve 제어]
# ==================================================

gripper_joint_controller = {
    "active": False,
    "label": "",
    "start_logical": 0.0,
    "target_logical": 0.0,
    "current_logical": 0.0,
    "stable_frames": 0
}


def normalize_gripper_logical_angle(angle_deg):
    """
    직육면체 박스의 180도 대칭을 이용해
    그리퍼 목표각을 -90도 이상 90도 이하 최단 경로로 정규화한다.
    """
    angle = float(angle_deg)

    while angle > 90.0:
        angle -= 180.0

    while angle < -90.0:
        angle += 180.0

    return float(angle)


def calculate_gripper_logical_target(box_applied_yaw_deg):
    """
    box_yaw_dict에 저장된 Isaac 적용 yaw를
    그리퍼의 화면상 논리 yaw로 변환한다.
    """
    return normalize_gripper_logical_angle(
        box_applied_yaw_deg
    )


def logical_yaw_to_drive_target(logical_angle_deg):
    """
    화면상 논리 yaw를 Revolute Joint targetPosition으로 변환한다.
    방향이 반대면 GRIPPER_JOINT_DIRECTION_SIGN=-1.0으로 바꾼다.
    """
    return (
        float(logical_angle_deg)
        * float(GRIPPER_JOINT_DIRECTION_SIGN)
        + float(GRIPPER_JOINT_ZERO_OFFSET_DEG)
    )


def drive_target_to_logical_yaw(drive_target_deg):
    sign = float(
        GRIPPER_JOINT_DIRECTION_SIGN
    )

    if abs(sign) <= 1e-9:
        sign = 1.0

    return normalize_gripper_logical_angle(
        (
            float(drive_target_deg)
            - float(GRIPPER_JOINT_ZERO_OFFSET_DEG)
        )
        / sign
    )


def prepare_gripper_joint_drive(
    emit_log=False
):
    """
    /assembly714/RevoluteJoint_gripper의 Angular Drive를 확인한다.

    기존 Angular Drive가 있으면 설정을 유지하고,
    Drive가 없을 때만 새로 생성해 기본 K/D/MaxForce를 적용한다.
    """
    joint_prim = get_prim(
        GRIPPER_REVOLUTE_JOINT_PATH
    )

    if joint_prim is None:
        event_log(
            f"[그리퍼 Joint 오류] Prim 없음 | "
            f"{GRIPPER_REVOLUTE_JOINT_PATH}"
        )
        return None

    try:
        target_attr = joint_prim.GetAttribute(
            "drive:angular:physics:targetPosition"
        )

        drive_created = False

        if not target_attr.IsValid():
            drive_api = UsdPhysics.DriveAPI.Apply(
                joint_prim,
                "angular"
            )

            drive_api.CreateTypeAttr().Set(
                GRIPPER_DRIVE_TYPE
            )
            drive_api.CreateStiffnessAttr().Set(
                float(GRIPPER_DRIVE_STIFFNESS)
            )
            drive_api.CreateDampingAttr().Set(
                float(GRIPPER_DRIVE_DAMPING)
            )
            drive_api.CreateMaxForceAttr().Set(
                float(GRIPPER_DRIVE_MAX_FORCE)
            )
            drive_api.CreateTargetVelocityAttr().Set(
                0.0
            )
            drive_api.CreateTargetPositionAttr().Set(
                logical_yaw_to_drive_target(
                    GRIPPER_JOINT_HOME_LOGICAL_DEG
                )
            )

            drive_created = True

            target_attr = joint_prim.GetAttribute(
                "drive:angular:physics:targetPosition"
            )

        elif FORCE_GRIPPER_DRIVE_TUNING:
            drive_api = UsdPhysics.DriveAPI.Get(
                joint_prim,
                "angular"
            )

            if not drive_api:
                drive_api = UsdPhysics.DriveAPI.Apply(
                    joint_prim,
                    "angular"
                )

            drive_api.CreateTypeAttr().Set(
                GRIPPER_DRIVE_TYPE
            )
            drive_api.CreateStiffnessAttr().Set(
                float(GRIPPER_DRIVE_STIFFNESS)
            )
            drive_api.CreateDampingAttr().Set(
                float(GRIPPER_DRIVE_DAMPING)
            )
            drive_api.CreateMaxForceAttr().Set(
                float(GRIPPER_DRIVE_MAX_FORCE)
            )

        if not target_attr.IsValid():
            event_log(
                f"[그리퍼 Joint 오류] Angular Drive 생성 실패 | "
                f"{GRIPPER_REVOLUTE_JOINT_PATH}"
            )
            return None

        velocity_attr = joint_prim.GetAttribute(
            "drive:angular:physics:targetVelocity"
        )

        if velocity_attr.IsValid():
            velocity_attr.Set(
                0.0
            )

        if emit_log:
            raw_target = target_attr.Get()

            event_log(
                f"[그리퍼 Joint 준비] "
                f"path={GRIPPER_REVOLUTE_JOINT_PATH} | "
                f"drive_created={drive_created} | "
                f"raw_target={raw_target} | "
                f"sign={GRIPPER_JOINT_DIRECTION_SIGN:.1f} | "
                f"offset={GRIPPER_JOINT_ZERO_OFFSET_DEG:.1f}"
            )

        return target_attr

    except Exception as error:
        event_log(
            f"[그리퍼 Joint 준비 예외] {error}"
        )
        return None


def get_gripper_joint_logical_angle():
    """
    현재 Angular Drive targetPosition을 논리 yaw로 변환해 반환한다.

    물리 Joint의 실제 회전값이 아니라 현재 명령값 기준이다.
    S-Curve 중에는 controller의 현재 논리각을 우선 사용한다.
    """
    if gripper_joint_controller.get(
        "active",
        False
    ):
        return float(
            gripper_joint_controller[
                "current_logical"
            ]
        )

    target_attr = prepare_gripper_joint_drive(
        emit_log=False
    )

    if target_attr is None:
        return None

    raw_value = target_attr.Get()

    if raw_value is None:
        return float(
            GRIPPER_JOINT_HOME_LOGICAL_DEG
        )

    return drive_target_to_logical_yaw(
        raw_value
    )


def set_gripper_joint_logical_angle_immediate(
    logical_angle_deg,
    emit_log=False
):
    """
    실제 Revolute Joint의 Angular Drive targetPosition을 즉시 갱신한다.
    호출부의 각도는 화면상/Isaac 기준 논리 yaw이다.
    """
    target_attr = prepare_gripper_joint_drive(
        emit_log=False
    )

    if target_attr is None:
        return False

    logical_angle = normalize_gripper_logical_angle(
        logical_angle_deg
    )

    raw_target = logical_yaw_to_drive_target(
        logical_angle
    )

    try:
        target_attr.Set(
            float(raw_target)
        )

        joint_prim = get_prim(
            GRIPPER_REVOLUTE_JOINT_PATH
        )

        if joint_prim is not None:
            velocity_attr = joint_prim.GetAttribute(
                "drive:angular:physics:targetVelocity"
            )

            if velocity_attr.IsValid():
                velocity_attr.Set(
                    0.0
                )

        if emit_log:
            event_log(
                f"[그리퍼 Joint 각도 설정] "
                f"논리각={logical_angle:.2f}도 | "
                f"Drive={raw_target:.2f}도"
            )

        return True

    except Exception as error:
        event_log(
            f"[그리퍼 Joint 각도 설정 오류] {error}"
        )
        return False


def start_gripper_joint_motion(
    label,
    target_logical_angle_deg
):
    if gripper_joint_controller.get(
        "active",
        False
    ):
        event_log(
            f"[그리퍼 회전 오류] {label} | "
            "다른 회전이 진행 중"
        )
        return False

    current_logical = (
        get_gripper_joint_logical_angle()
    )

    if current_logical is None:
        return False

    target_logical = (
        normalize_gripper_logical_angle(
            target_logical_angle_deg
        )
    )

    gripper_joint_controller[
        "active"
    ] = True
    gripper_joint_controller[
        "label"
    ] = str(label)
    gripper_joint_controller[
        "start_logical"
    ] = float(current_logical)
    gripper_joint_controller[
        "target_logical"
    ] = float(target_logical)
    gripper_joint_controller[
        "current_logical"
    ] = float(current_logical)
    gripper_joint_controller[
        "stable_frames"
    ] = 0

    event_log(
        f"[그리퍼 이동] 회전 {label} | "
        f"현재={current_logical:.2f}도 | "
        f"목표={target_logical:.2f}도 | "
        f"Drive목표={logical_yaw_to_drive_target(target_logical):.2f}도"
    )

    return True


def cancel_gripper_joint_motion():
    gripper_joint_controller[
        "active"
    ] = False
    gripper_joint_controller[
        "label"
    ] = ""
    gripper_joint_controller[
        "start_logical"
    ] = 0.0
    gripper_joint_controller[
        "target_logical"
    ] = 0.0
    gripper_joint_controller[
        "current_logical"
    ] = 0.0
    gripper_joint_controller[
        "stable_frames"
    ] = 0


def update_gripper_joint_motion():
    """
    매 update마다 실제 Revolute Joint의 targetPosition을
    S-Curve 형태로 조금씩 갱신한다.
    """
    if not gripper_joint_controller.get(
        "active",
        False
    ):
        return None

    start_angle = float(
        gripper_joint_controller[
            "start_logical"
        ]
    )
    target_angle = float(
        gripper_joint_controller[
            "target_logical"
        ]
    )
    current_angle = float(
        gripper_joint_controller[
            "current_logical"
        ]
    )

    remaining = (
        target_angle
        - current_angle
    )

    if abs(remaining) <= float(
        GRIPPER_JOINT_TOLERANCE_DEG
    ):
        next_angle = target_angle
    else:
        total_distance = abs(
            target_angle
            - start_angle
        )

        if total_distance <= 1e-9:
            progress = 1.0
        else:
            progress = abs(
                current_angle
                - start_angle
            ) / total_distance

        speed_ratio = s_curve_speed_ratio(
            progress
        )

        max_step = (
            float(GRIPPER_JOINT_MAX_STEP_DEG)
            * float(speed_ratio)
        )

        step_value = min(
            abs(remaining),
            max_step
        )

        direction = (
            1.0
            if remaining > 0.0
            else -1.0
        )

        next_angle = (
            current_angle
            + direction * step_value
        )

    if not set_gripper_joint_logical_angle_immediate(
        next_angle,
        emit_log=False
    ):
        cancel_gripper_joint_motion()
        return None

    gripper_joint_controller[
        "current_logical"
    ] = float(next_angle)

    if abs(
        target_angle
        - float(next_angle)
    ) <= float(
        GRIPPER_JOINT_TOLERANCE_DEG
    ):
        gripper_joint_controller[
            "stable_frames"
        ] += 1
    else:
        gripper_joint_controller[
            "stable_frames"
        ] = 0

    if (
        gripper_joint_controller[
            "stable_frames"
        ]
        < int(
            GRIPPER_JOINT_STABLE_FRAMES_REQUIRED
        )
    ):
        return None

    completed_label = str(
        gripper_joint_controller[
            "label"
        ]
    )
    completed_angle = float(
        target_angle
    )

    cancel_gripper_joint_motion()

    event_log(
        f"[그리퍼 회전 완료] "
        f"{completed_label} | "
        f"논리각={completed_angle:.2f}도 | "
        f"Drive={logical_yaw_to_drive_target(completed_angle):.2f}도"
    )

    return (
        completed_label,
        completed_angle
    )


def get_attached_box_follow_yaw():
    current_gripper_yaw = (
        get_gripper_joint_logical_angle()
    )

    if current_gripper_yaw is None:
        current_gripper_yaw = (
            attached_gripper_logical_yaw_at_attach
        )

    return normalize_applied_yaw(
        float(attached_box_yaw_at_attach)
        + (
            float(current_gripper_yaw)
            - float(
                attached_gripper_logical_yaw_at_attach
            )
        )
    )


def sync_attached_box_rotation_to_gripper():
    """
    박스 Rigid Body가 비활성화된 흡착 상태에서
    실제 그리퍼 Joint의 논리 yaw 변화량만큼 박스 yaw도 같이 갱신한다.
    """
    if not bool(ATTACHED_BOX_ROTATION_FOLLOW_ENABLED):
        return False

    if not is_grip_attached or not attached_box_path:
        return False

    if get_prim(attached_box_path) is None:
        return False

    return set_box_applied_yaw_direct(
        attached_box_path,
        get_attached_box_follow_yaw(),
        update_dict=True
    )


def test_gripper_joint_rotation(angle_deg=20.0):
    """
    Script Editor 수동 테스트:

        test_gripper_joint_rotation(20.0)
        test_gripper_joint_rotation(0.0)
        test_gripper_joint_rotation(-20.0)
    """
    return start_gripper_joint_motion(
        "수동 테스트",
        float(angle_deg)
    )


# ==================================================
# [14. 이동 트레이 실시간 추종 컨트롤러]
# - Tray1 World XYZ를 매 프레임 읽는다.
# - 실제 이동축은 Tray의 진행축인 World X를 X1/X2로 변환한다.
# - 따라잡기 → 50mm 추종 하강 → 해제 → 10mm 추가 추종 상승 순서다.
# ==================================================


def _move_axis_target_toward(axis_name, target_value, max_step):
    current_value = get_axis_target_position(axis_name)

    if current_value is None:
        return False, None, False

    target_value = clamp_axis_target(axis_name, target_value)
    current_value = float(current_value)
    remaining = float(target_value) - current_value

    if abs(remaining) <= float(TRAY_FOLLOW_TARGET_TOLERANCE):
        next_value = float(target_value)
    else:
        direction = 1.0 if remaining > 0.0 else -1.0
        next_value = current_value + direction * min(
            abs(remaining),
            float(max_step)
        )

    if not set_axis_target_immediate(axis_name, next_value):
        return False, current_value, False

    reached = (
        abs(float(target_value) - float(next_value))
        <= float(TRAY_FOLLOW_TARGET_TOLERANCE)
    )

    return True, float(next_value), reached


def calculate_tray_follow_joint_targets(
    tray_world_position,
    lead_world_x=0.0
):
    """
    실제로 정상 확인된 단순 추종식.

    Tray World X=-115.39968일 때 X1/X2=70을 기준으로
    현재 선택된 Tray의 World X를 Joint targetPosition으로 변환한다.

    Z축은 동적 BBox 보정을 사용하지 않고 항상 -250을 사용한다.
    """
    if tray_world_position is None:
        return None

    tray_world_x = float(
        tray_world_position[0]
    )

    joint_delta_x = (
        (
            tray_world_x
            + float(lead_world_x)
            - float(TRAY_REFERENCE_WORLD_X)
        )
        * float(
            TRAY_WORLD_X_TO_JOINT_SCALE
        )
        * float(
            TRAY_WORLD_X_TO_JOINT_SIGN
        )
    )

    x1_target = clamp_axis_target(
        "X1",
        float(
            TRAY_REFERENCE_JOINT_X1
        )
        + joint_delta_x,
    )

    x2_target = clamp_axis_target(
        "X2",
        float(
            TRAY_REFERENCE_JOINT_X2
        )
        + joint_delta_x,
    )

    z_target = clamp_axis_target(
        "Z",
        float(
            TRAY_CONFIRMED_PLACE_Z_POSITION
        ),
    )

    return {
        "tray_world_x": tray_world_x,
        "tray_world_y": float(
            tray_world_position[1]
        ),
        "tray_world_z": float(
            tray_world_position[2]
        ),
        "joint_delta_x": float(
            joint_delta_x
        ),
        "tray_motion_joint_delta_z": 0.0,
        "z_coarse_target": float(
            TRAY_CONFIRMED_PLACE_Z_POSITION
        ),
        "z_fine_offset": 0.0,
        "x1": float(x1_target),
        "x2": float(x2_target),
        "z": float(z_target),
    }


def get_pack_tray_candidate_paths():
    """
    Stage에 실제 존재하는 Tray1~Tray5 경로만 반환한다.

    Tray_loader 검색 결과도 확인하여 Stage 구조가 조금 달라도
    이름이 Tray2~Tray5라면 자동으로 포함한다.
    """
    paths = []

    for tray_number in range(1, 6):
        expected_path = PACK_TRAY_PATH_BY_NUMBER[
            tray_number
        ]

        if get_prim(expected_path) is not None:
            paths.append(expected_path)

    try:
        discovered_paths = (
            tray_loader_discover_tray_paths()
        )
    except Exception:
        discovered_paths = []

    for path in discovered_paths:
        tray_name = str(path).rsplit(
            "/",
            1,
        )[-1]

        match = re.fullmatch(
            r"Tray(\d+)",
            tray_name,
        )

        if match is None:
            continue

        tray_number = int(match.group(1))

        if (
            2 <= tray_number <= 5
            and path not in paths
            and get_prim(path) is not None
        ):
            paths.append(str(path))

    def tray_sort_key(path):
        match = re.search(
            r"Tray(\d+)$",
            str(path),
        )

        return (
            int(match.group(1))
            if match
            else 999
        )

    paths.sort(
        key=tray_sort_key
    )

    return paths


def get_place_slot_config(place_slot=None):
    if place_slot not in (1, 2):
        try:
            place_slot = int(
                robot_sequence.get("place_slot", 0)
            )
        except Exception:
            place_slot = 0

    if place_slot not in (1, 2):
        place_slot = 1 if int(auto_place_count) % 2 == 0 else 2

    return place_slot, dict(TRAY_PLACE_SLOT_CONFIG[place_slot])


def cancel_pack_tray_trigger():
    pack_tray_trigger_controller["armed"] = False
    pack_tray_trigger_controller["elapsed_sec"] = 0.0
    pack_tray_trigger_controller["place_slot"] = None
    pack_tray_trigger_controller["trigger_x"] = None
    pack_tray_trigger_controller["previous_x_by_path"] = {}
    pack_tray_trigger_controller["selected_tray_path"] = None


def _start_selected_tray_place(tray_path, place_slot):
    if not set_active_pack_tray(tray_path):
        return False

    pack_tray_trigger_controller["armed"] = False
    pack_tray_trigger_controller["selected_tray_path"] = str(tray_path)

    tray_position = get_world_position(tray_path)
    tray_name = str(tray_path).rsplit("/", 1)[-1]
    tray_x = None if tray_position is None else float(tray_position[0])

    event_log(
        f"[트레이 이동] {place_slot}번 적재 트레이 확정 | "
        f"Tray={tray_name} | World X={tray_x}"
    )

    return start_tray_follow_motion()


def arm_pack_tray_trigger():
    """
    1번 박스:
        공급된 새 트레이가 World X=4에 도달하면 트레이를 선택하고
        X1/X2=60, Y=25, Z=-250 준비 상태를 확인한다.
        이후 Tray X=-30에 도달하면 X1/X2 60 -> -19,
        Y 25 -> -50 대각선 적재를 시작한다.

    2번 박스:
        1번 적재에 사용한 동일 TRAY_PATH를 유지한다.
        X1/X2=-200, Y=25, Z=-250 준비 이동이 끝나면
        트레이 World X를 기다리지 않고 즉시 -264 대각선 적재를 시작한다.
    """
    cancel_pack_tray_trigger()

    place_slot, config = get_place_slot_config()

    if place_slot == 2:
        tray_path = TRAY_PATH

        if get_prim(tray_path) is None:
            tray_path = getattr(
                integrated_tray_loader_controller,
                "active_tray_path",
                None,
            )

        if not tray_path or get_prim(tray_path) is None:
            event_log(
                "[트레이 이동] 2번 적재 시작 실패 | "
                "1번 적재에 사용한 활성 트레이 없음"
            )
            return False

        event_log(
            "[트레이 이동] 2번 박스 준비 시작 | "
            f"Tray={str(tray_path).rsplit('/', 1)[-1]} | "
            f"X1/X2={float(config['prepare_x']):.1f} 준비 후 "
            f"트레이 좌표 대기 없이 {float(config['final_x']):.1f} 대각선 적재"
        )
        return _start_selected_tray_place(tray_path, place_slot)

    tray_paths = get_pack_tray_candidate_paths()
    preferred_tray = getattr(
        integrated_tray_loader_controller,
        "active_tray_path",
        None,
    )

    if preferred_tray and get_prim(preferred_tray) is not None:
        tray_paths = [str(preferred_tray)]

    if not tray_paths:
        event_log("[트레이 이동] 1번 적재용 트레이 경로를 찾지 못함")
        return False

    trigger_x = float(config["select_trigger_x"])
    previous_positions = {}

    for tray_path in tray_paths:
        tray_position = get_world_position(tray_path)
        previous_positions[tray_path] = (
            None if tray_position is None else float(tray_position[0])
        )

    pack_tray_trigger_controller["armed"] = True
    pack_tray_trigger_controller["elapsed_sec"] = 0.0
    pack_tray_trigger_controller["place_slot"] = place_slot
    pack_tray_trigger_controller["trigger_x"] = trigger_x
    pack_tray_trigger_controller["previous_x_by_path"] = previous_positions

    set_robot_sequence_state("WAIT_PACK_TRAY_TRIGGER")

    event_log(
        "[트레이 이동] 1번 박스 트레이 감시 시작 | "
        f"Trigger X={trigger_x:.1f} | "
        f"적재 시작 X={config['place_trigger_x']:.1f}"
    )

    # 로봇 준비가 늦어 트레이가 이미 X=4를 지났지만 아직 X=-30 이전이면 즉시 선택한다.
    for tray_path, current_x in previous_positions.items():
        if current_x is None:
            continue

        if (
            float(current_x) <= trigger_x + TRAY_COORD_TRIGGER_TOLERANCE
            and float(current_x) > float(config["place_trigger_x"])
        ):
            return _start_selected_tray_place(tray_path, place_slot)

    return True


def pack_tray_crossed_trigger(previous_x, current_x, trigger_x=None):
    if previous_x is None:
        return False

    previous_x = float(previous_x)
    current_x = float(current_x)
    trigger_x = float(
        PACK_TRAY_TRIGGER_WORLD_X
        if trigger_x is None
        else trigger_x
    )

    if abs(current_x - trigger_x) <= float(TRAY_COORD_TRIGGER_TOLERANCE):
        return True

    if PACK_TRAY_EXPECTED_DIRECTION < 0.0:
        return previous_x > trigger_x and current_x <= trigger_x

    return previous_x < trigger_x and current_x >= trigger_x


def tray_reached_coordinate(previous_x, current_x, trigger_x):
    current_x = float(current_x)
    trigger_x = float(trigger_x)

    if abs(current_x - trigger_x) <= float(TRAY_COORD_TRIGGER_TOLERANCE):
        return True

    # 트레이가 큰 X에서 작은 X로 이동하므로 이미 기준 이하이면 도달로 처리한다.
    if PACK_TRAY_EXPECTED_DIRECTION < 0.0:
        if current_x <= trigger_x:
            return True
        if previous_x is not None:
            return float(previous_x) > trigger_x and current_x <= trigger_x
        return False

    if current_x >= trigger_x:
        return True
    if previous_x is not None:
        return float(previous_x) < trigger_x and current_x >= trigger_x
    return False


def update_pack_tray_trigger(event):
    if not pack_tray_trigger_controller.get("armed", False):
        return False

    if not omni.timeline.get_timeline_interface().is_playing():
        return False

    try:
        dt = float(event.payload.get("dt", 0.0))
    except Exception:
        dt = 0.0

    pack_tray_trigger_controller["elapsed_sec"] += max(0.0, dt)

    if (
        pack_tray_trigger_controller["elapsed_sec"]
        > float(PACK_TRAY_WAIT_TIMEOUT_SEC)
    ):
        trigger_value = float(
            pack_tray_trigger_controller.get(
                "trigger_x",
                TRAY_PLACE_SLOT_CONFIG[1]["select_trigger_x"]
            )
        )

        cancel_pack_tray_trigger()
        abort_robot_sequence(
            f"1번 박스 Tray 선택 X={trigger_value:.1f} 대기 시간 초과"
        )
        return False

    place_slot = int(pack_tray_trigger_controller.get("place_slot", 1))
    trigger_x = float(
        pack_tray_trigger_controller.get(
            "trigger_x",
            TRAY_PLACE_SLOT_CONFIG[1]["select_trigger_x"]
        )
    )
    previous_map = pack_tray_trigger_controller["previous_x_by_path"]

    for tray_path in list(previous_map.keys()):

        tray_position = get_world_position(tray_path)
        if tray_position is None:
            continue

        current_x = float(tray_position[0])
        previous_x = previous_map.get(tray_path)
        reached = pack_tray_crossed_trigger(
            previous_x,
            current_x,
            trigger_x,
        )
        previous_map[tray_path] = current_x

        if reached:
            return _start_selected_tray_place(tray_path, place_slot)

    return False


def cancel_tray_follow_motion():
    tray_follow_controller["active"] = False
    tray_follow_controller["phase"] = "IDLE"
    tray_follow_controller["frames"] = 0
    tray_follow_controller["stable_frames"] = 0
    tray_follow_controller["phase_elapsed_sec"] = 0.0
    tray_follow_controller["place_slot"] = None
    tray_follow_controller["selected_tray_path"] = None
    tray_follow_controller["previous_tray_x"] = None
    tray_follow_controller["place_trigger_x"] = None
    tray_follow_controller["prepare_x"] = None
    tray_follow_controller["final_x"] = None
    tray_follow_controller["start_tray_position"] = None
    tray_follow_controller["reference_tray_position"] = None
    tray_follow_controller["reference_box_center"] = None
    tray_follow_controller["reference_joint_targets"] = None
    tray_follow_controller["y_raw_offset"] = None
    tray_follow_controller["released_box"] = None
    tray_follow_controller["last_tray_position"] = None
    tray_follow_controller["last_box_center"] = None
    tray_follow_controller["last_alignment_error"] = None
    tray_follow_controller["z_fine_offset"] = 0.0


def start_tray_follow_motion():
    if motion_controller.get("active", False):
        event_log("[트레이 이동] 준비 시작 실패 | 다른 3축 이동 진행 중")
        return False

    if not is_grip_attached or not attached_box_path:
        event_log("[트레이 이동] 준비 시작 실패 | 흡착된 박스 없음")
        return False

    place_slot, config = get_place_slot_config()
    tray_path = TRAY_PATH
    tray_position = get_world_position(tray_path)

    if tray_position is None:
        event_log(f"[트레이 이동] 준비 시작 실패 | Tray 좌표 없음: {tray_path}")
        return False

    cancel_tray_follow_motion()

    tray_follow_controller["active"] = True
    tray_follow_controller["phase"] = "PREPARE"
    tray_follow_controller["frames"] = 0
    tray_follow_controller["phase_elapsed_sec"] = 0.0
    tray_follow_controller["place_slot"] = place_slot
    tray_follow_controller["selected_tray_path"] = str(tray_path)
    tray_follow_controller["previous_tray_x"] = float(tray_position[0])
    tray_follow_controller["place_trigger_x"] = float(config["place_trigger_x"])
    tray_follow_controller["prepare_x"] = float(config["prepare_x"])
    tray_follow_controller["final_x"] = float(config["final_x"])
    tray_follow_controller["start_tray_position"] = copy_world_position(tray_position)
    tray_follow_controller["last_tray_position"] = copy_world_position(tray_position)

    set_robot_sequence_state("TRAY_SLOT_PREPARE")

    started = start_scurve_motion(
        f"{config['label']} 적재 준비",
        {
            "X1": float(config["prepare_x"]),
            "X2": float(config["prepare_x"]),
            "Y": AUTO_PICK_Y_UP_POSITION,
            "Z": TRAY_CONFIRMED_PLACE_Z_POSITION,
        },
    )

    if not started:
        cancel_tray_follow_motion()
        return False

    event_log(
        f"[트레이 이동] {config['label']} 준비 | "
        f"Tray X={float(tray_position[0]):.3f} | "
        f"X1/X2={float(config['prepare_x']):.1f} | "
        f"Y={AUTO_PICK_Y_UP_POSITION:.1f} | "
        f"Z={TRAY_CONFIRMED_PLACE_Z_POSITION:.1f} | "
        f"적재 Trigger X={float(config['place_trigger_x']):.1f}"
    )
    return True


def _start_tray_diagonal_motion():
    place_slot, config = get_place_slot_config(
        tray_follow_controller.get("place_slot")
    )

    tray_follow_controller["phase"] = "DIAGONAL"
    tray_follow_controller["phase_elapsed_sec"] = 0.0
    set_robot_sequence_state("TRAY_SLOT_DIAGONAL")

    started = start_scurve_motion(
        f"{config['label']} 대각선 적재",
        {
            "X1": float(config["final_x"]),
            "X2": float(config["final_x"]),
            "Y": TRAY_CONFIRMED_PLACE_Y_POSITION,
            "Z": TRAY_CONFIRMED_PLACE_Z_POSITION,
        },
        synchronized=True,
        duration_frames=TRAY_DIAGONAL_S_CURVE_FRAMES,
    )

    if started:
        event_log(
            f"[트레이 이동] {config['label']} 대각선 S-Curve 시작 | "
            f"X1/X2 {float(config['prepare_x']):.1f}"
            f"->{float(config['final_x']):.1f} | "
            f"Y {AUTO_PICK_Y_UP_POSITION:.1f}"
            f"->{TRAY_CONFIRMED_PLACE_Y_POSITION:.1f} | "
            f"Z={TRAY_CONFIRMED_PLACE_Z_POSITION:.1f} 유지"
        )

    return started


def _start_tray_home_return():
    target_box = robot_sequence.get("target_box")
    place_slot = tray_follow_controller.get("place_slot")
    selected_tray_path = tray_follow_controller.get("selected_tray_path") or TRAY_PATH
    tray_name = str(selected_tray_path).rsplit("/", 1)[-1]

    cancel_tray_follow_motion()
    cancel_pack_tray_trigger()
    set_robot_sequence_state("HOME_RETURN")

    event_log(
        f"[트레이 이동] {place_slot}번 박스 적재 완료 | "
        f"Tray={tray_name} | "
        f"Box ID={box_path_id_dict.get(target_box, None)} | 원점 복귀"
    )

    return start_scurve_motion(
        "두 박스 좌표 적재 후 원점 복귀",
        {
            "X1": AUTO_HOME_X1_POSITION,
            "X2": AUTO_HOME_X2_POSITION,
            "Y": AUTO_HOME_Y_POSITION,
            "Z": AUTO_HOME_Z_POSITION,
        },
    )


def update_tray_follow_motion(event=None):
    if not tray_follow_controller.get("active", False):
        return False

    tray_follow_controller["frames"] += 1

    try:
        dt = float(event.payload.get("dt", 0.0))
    except Exception:
        dt = 0.0

    tray_follow_controller["phase_elapsed_sec"] += max(0.0, dt)

    if tray_follow_controller["frames"] > int(TRAY_FOLLOW_TIMEOUT_FRAMES):
        cancel_tray_follow_motion()
        abort_robot_sequence("두 박스 좌표 적재 시간 초과")
        return False

    tray_path = tray_follow_controller.get("selected_tray_path") or TRAY_PATH
    tray_position = get_world_position(tray_path)

    if tray_position is None:
        cancel_tray_follow_motion()
        abort_robot_sequence("선택된 트레이 위치 읽기 실패")
        return False

    current_x = float(tray_position[0])
    previous_x = tray_follow_controller.get("previous_tray_x")
    tray_follow_controller["previous_tray_x"] = current_x
    tray_follow_controller["last_tray_position"] = copy_world_position(tray_position)

    phase = tray_follow_controller.get("phase", "IDLE")
    place_slot, config = get_place_slot_config(
        tray_follow_controller.get("place_slot")
    )

    # PREPARE / DIAGONAL / RISE는 일반 S-Curve 완료 콜백이 다음 단계로 전환한다.
    if phase in ("PREPARE", "DIAGONAL", "RISE"):
        return True

    if phase == "WAIT_PLACE_TRIGGER":
        trigger_x = float(tray_follow_controller["place_trigger_x"])

        if not tray_reached_coordinate(previous_x, current_x, trigger_x):
            return True

        event_log(
            f"[트레이 이동] {config['label']} 적재 좌표 도달 | "
            f"Tray X={current_x:.3f} | 기준={trigger_x:.1f}"
        )

        if not _start_tray_diagonal_motion():
            abort_robot_sequence(f"{config['label']} 대각선 적재 시작 실패")
            return False

        return True

    if phase == "PRE_RELEASE_HOLD":
        if (
            tray_follow_controller["phase_elapsed_sec"]
            < float(TRAY_PRE_RELEASE_HOLD_SEC)
        ):
            return True

        released_box = start_release_settle_on_moving_tray()

        if released_box is None:
            abort_robot_sequence(f"{config['label']} 흡착 해제 실패")
            return False

        tray_follow_controller["released_box"] = released_box
        tray_follow_controller["phase"] = "RISE"
        tray_follow_controller["phase_elapsed_sec"] = 0.0
        set_robot_sequence_state("TRAY_SLOT_RISE")

        started = start_scurve_motion(
            f"{config['label']} 흡착 해제 후 Y 상승",
            {
                "X1": float(config["final_x"]),
                "X2": float(config["final_x"]),
                "Y": AUTO_PICK_Y_UP_POSITION,
                "Z": TRAY_CONFIRMED_PLACE_Z_POSITION,
            },
        )

        if not started:
            abort_robot_sequence(f"{config['label']} 흡착 해제 후 상승 실패")
            return False

        return True

    if phase == "WAIT_RELEASE_COMPLETE":
        release_complete = (
            not release_settle_controller.get("active", False)
            and release_settle_controller.get("completed", False)
        )

        if not release_complete:
            return True

        if not _start_tray_home_return():
            abort_robot_sequence("트레이 적재 후 원점 복귀 실패")
            return False

        return True

    abort_robot_sequence(f"알 수 없는 좌표 적재 단계: {phase}")
    return False


# ==================================================
# [14-1. 실제 Position 기반 자동 픽업/적재 상태 머신]
# ==================================================
# v34~v37 파일은 Script Editor의 이전 실행 전역에 남아 있던 함수에
# 의존할 수 있었으므로, v40에서는 필요한 상태 머신을 파일 안에 모두 포함한다.

robot_sequence = {
    "state": "IDLE",
    "target_box": None,
    "target_gripper_logical_yaw": GRIPPER_JOINT_HOME_LOGICAL_DEG,
    "dynamic_pick_targets": None,
    "place_slot": None
}

# v46.6:
# is_auto_sequence_running 값이 다른 보조 로직에 의해 잘못 풀리더라도
# 현재 박스의 적재 및 HOME 복귀가 끝나기 전까지 다음 픽업을 막는다.
robot_sequence_guard = {
    "locked": False,
    "owner_box": None,
    "reason": "",
}

system_ready = False

# Tray2 X=239 도착 전까지 첫 번째 정상 박스의 로봇 이동을 보류하는 상태.
first_tray_pick_wait_controller = {
    "active": False,
    "target_box": None,
    "tray_path": FIRST_TRAY_PICK_WAIT_PATH,
    "previous_x": None,
    "last_x": None,
    "triggered": False,
    "sample_count": 0,
}


def get_pick_wait_tray_path():
    """
    place_slot=1 박스가 기다릴 실제 Tray 경로를 반환한다.

    다음 Tray 공급이 진행 중이면 target_tray_path를 우선 사용하고,
    공급이 완료된 상태이면 active_tray_path를 사용한다.
    최초 공급 전에는 기존 Tray2 경로를 그대로 사용한다.
    """
    try:
        target_tray_path = getattr(
            integrated_tray_loader_controller,
            "target_tray_path",
            None,
        )
        loader_active = bool(
            getattr(
                integrated_tray_loader_controller,
                "active",
                False,
            )
        )

        if (
            loader_active
            and target_tray_path
            and get_prim(str(target_tray_path)) is not None
        ):
            return str(target_tray_path)

        active_tray_path = getattr(
            integrated_tray_loader_controller,
            "active_tray_path",
            None,
        )

        if (
            active_tray_path
            and get_prim(str(active_tray_path)) is not None
        ):
            return str(active_tray_path)
    except Exception:
        pass

    return str(FIRST_TRAY_PICK_WAIT_PATH)


def reset_first_tray_pick_wait():
    first_tray_pick_wait_controller["active"] = False
    first_tray_pick_wait_controller["target_box"] = None
    first_tray_pick_wait_controller["tray_path"] = get_pick_wait_tray_path()
    first_tray_pick_wait_controller["previous_x"] = None
    first_tray_pick_wait_controller["last_x"] = None
    first_tray_pick_wait_controller["triggered"] = False
    first_tray_pick_wait_controller["sample_count"] = 0


def _has_tray2_reached_pick_wait_x(previous_x, current_x):
    if current_x is None:
        return False

    current_x = float(current_x)
    target_x = float(FIRST_TRAY_PICK_WAIT_TARGET_X)
    tolerance = float(FIRST_TRAY_PICK_WAIT_TOLERANCE_X)

    if abs(current_x - target_x) <= tolerance:
        return True

    if previous_x is not None:
        previous_x = float(previous_x)
        movement_min = min(previous_x, current_x)
        movement_max = max(previous_x, current_x)

        if movement_min <= target_x <= movement_max:
            return True

    if FIRST_TRAY_PICK_WAIT_ALLOW_ALREADY_PASSED:
        direction = float(FIRST_TRAY_PICK_WAIT_EXPECTED_DIRECTION)

        if direction < 0.0 and current_x < target_x - tolerance:
            return True

        if direction > 0.0 and current_x > target_x + tolerance:
            return True

    return False


def _start_pick_xz_motion(target_box):
    """Tray2 X=239 조건이 만족된 뒤 실제 박스 위 X/Z 이동을 시작한다."""
    if target_box is None or get_prim(target_box) is None:
        abort_robot_sequence("박스 위 이동 대상 박스 없음")
        return False

    dynamic_pick_targets = robot_sequence.get("dynamic_pick_targets")

    if not isinstance(dynamic_pick_targets, dict):
        abort_robot_sequence("박스 위 이동 목표 좌표 없음")
        return False

    set_robot_sequence_state("PICK_XZ", target_box)

    started = start_scurve_motion(
        "활성 Tray 도착 후 박스 위 동적 X/Z 고속 이동",
        {
            "X1": dynamic_pick_targets["x1"],
            "X2": dynamic_pick_targets["x2"],
            "Z": dynamic_pick_targets["z"],
        }
    )

    if not started:
        abort_robot_sequence("활성 Tray 대기 완료 후 박스 위 X/Z 이동 시작 실패")

    return bool(started)


def _start_pick_gripper_rotation(target_box):
    if target_box is None or get_prim(target_box) is None:
        abort_robot_sequence("그리퍼 회전 대상 박스 없음")
        return False

    box_applied_yaw = float(box_yaw_dict.get(target_box, 0.0))
    gripper_target_yaw = calculate_gripper_logical_target(box_applied_yaw)
    robot_sequence["target_gripper_logical_yaw"] = float(gripper_target_yaw)
    set_robot_sequence_state("GRIPPER_ROTATE_PICK")

    started = start_gripper_joint_motion(
        "픽업 전 박스 각도 맞춤",
        gripper_target_yaw
    )

    if not started:
        abort_robot_sequence("그리퍼 Joint 회전 시작 실패")

    return bool(started)


def get_first_tray_pick_wait_x(tray_path=None):
    if tray_path is None:
        tray_path = first_tray_pick_wait_controller.get(
            "tray_path",
            get_pick_wait_tray_path(),
        )

    tray_path = str(tray_path)
    coordinate_mode = str(
        FIRST_TRAY_PICK_WAIT_COORDINATE_MODE
    ).strip().upper()

    if coordinate_mode == "WORLD":
        tray_position = get_world_position(tray_path)
    else:
        tray_position = get_local_translate(tray_path)

    if tray_position is None:
        return None

    return float(tray_position[0])


def arm_first_tray_pick_wait(target_box):
    reset_first_tray_pick_wait()

    tray_path = get_pick_wait_tray_path()
    first_tray_pick_wait_controller["tray_path"] = str(tray_path)
    tray_x = get_first_tray_pick_wait_x(tray_path)

    first_tray_pick_wait_controller["active"] = True
    first_tray_pick_wait_controller["target_box"] = target_box
    first_tray_pick_wait_controller["tray_path"] = tray_path
    first_tray_pick_wait_controller["sample_count"] = 0

    if tray_x is not None:
        first_tray_pick_wait_controller["previous_x"] = float(tray_x)
        first_tray_pick_wait_controller["last_x"] = float(tray_x)

    set_robot_sequence_state("WAIT_TRAY2_X239_BEFORE_PICK_XZ", target_box)

    tray_x_text = "None" if tray_x is None else f"{float(tray_x):.3f}"

    tray_name = str(tray_path).rsplit("/", 1)[-1]

    event_log(
        f"[그리퍼 이동] 정상 박스 픽업 위치 정지, 로봇 이동 보류 | "
        f"ID={box_path_id_dict.get(target_box, None)} | "
        f"Tray={tray_name} | 현재 X={tray_x_text} | "
        f"좌표={FIRST_TRAY_PICK_WAIT_COORDINATE_MODE} | "
        f"목표 X={FIRST_TRAY_PICK_WAIT_TARGET_X:.1f}±"
        f"{FIRST_TRAY_PICK_WAIT_TOLERANCE_X:.1f}"
    )

    return True


def update_first_tray_pick_wait():
    if not first_tray_pick_wait_controller.get("active", False):
        return False

    if robot_sequence.get("state") != "WAIT_TRAY2_X239_BEFORE_PICK_XZ":
        reset_first_tray_pick_wait()
        return False

    target_box = first_tray_pick_wait_controller.get("target_box")

    if target_box is None or get_prim(target_box) is None:
        reset_first_tray_pick_wait()
        abort_robot_sequence("활성 Tray 대기 중 픽업 대상 박스 없음")
        return False

    tray_path = first_tray_pick_wait_controller.get(
        "tray_path",
        get_pick_wait_tray_path(),
    )
    current_x = get_first_tray_pick_wait_x(tray_path)

    if current_x is None:
        return False

    current_x = float(current_x)
    previous_x = first_tray_pick_wait_controller.get("previous_x")
    first_tray_pick_wait_controller["last_x"] = current_x
    first_tray_pick_wait_controller["sample_count"] += 1

    # arm_first_tray_pick_wait()가 호출된 같은 update 프레임에는 절대
    # 다음 단계로 넘어가지 않는다. Tray2의 다음 위치 샘플부터 감시한다.
    if first_tray_pick_wait_controller["sample_count"] <= 1:
        first_tray_pick_wait_controller["previous_x"] = current_x
        return False

    reached = _has_tray2_reached_pick_wait_x(previous_x, current_x)
    first_tray_pick_wait_controller["previous_x"] = current_x

    if not reached:
        return False

    first_tray_pick_wait_controller["active"] = False
    first_tray_pick_wait_controller["triggered"] = True

    tray_name = str(
        first_tray_pick_wait_controller.get(
            "tray_path",
            FIRST_TRAY_PICK_WAIT_PATH,
        )
    ).rsplit("/", 1)[-1]

    event_log(
        f"[그리퍼 이동] 활성 Tray 흡착 허용 위치 도착 | "
        f"ID={box_path_id_dict.get(target_box, None)} | "
        f"Tray={tray_name} | X={current_x:.3f} | "
        f"기준 X={FIRST_TRAY_PICK_WAIT_TARGET_X:.3f} | "
        "직교로봇 박스 위 X/Z 이동 시작"
    )

    return _start_pick_xz_motion(target_box)


def set_robot_sequence_state(state_name, target_box=None):
    previous_state = str(robot_sequence.get("state", "IDLE"))
    next_state = str(state_name)

    robot_sequence["state"] = next_state

    if target_box is not None:
        robot_sequence["target_box"] = target_box

    active_box = target_box or robot_sequence.get("target_box")
    active_box_id = box_path_id_dict.get(active_box, None) if active_box else None

    if previous_state != next_state:
        event_log(
            f"[공정 단계] {previous_state} -> {next_state} | "
            f"box_id={active_box_id} | "
            f"place_slot={robot_sequence.get('place_slot')}"
        )


def lock_robot_sequence(
    owner_box,
    reason
):
    robot_sequence_guard[
        "locked"
    ] = True
    robot_sequence_guard[
        "owner_box"
    ] = owner_box
    robot_sequence_guard[
        "reason"
    ] = str(reason)


def unlock_robot_sequence():
    robot_sequence_guard[
        "locked"
    ] = False
    robot_sequence_guard[
        "owner_box"
    ] = None
    robot_sequence_guard[
        "reason"
    ] = ""


def is_robot_sequence_locked():
    return bool(
        robot_sequence_guard.get(
            "locked",
            False
        )
    )


def enforce_robot_sequence_guard():
    """
    첫 박스의 픽업/적재/HOME 복귀 중에는 자동 시퀀스 플래그가
    실수로 False가 되더라도 다시 True로 복구한다.
    """
    global is_auto_sequence_running

    state = robot_sequence.get(
        "state",
        "IDLE"
    )

    if not is_robot_sequence_locked():
        return False

    if state in (
        "IDLE",
        "ERROR"
    ):
        return False

    is_auto_sequence_running = True
    return True


def abort_robot_sequence(reason):
    global is_auto_sequence_running

    cancel_scurve_motion()
    cancel_gripper_joint_motion()
    cancel_grip_stabilization()
    cancel_release_settle()
    cancel_tray_follow_motion()
    cancel_pack_tray_trigger()
    reset_next_box_preload()
    reset_forced_next_box_feed()
    reset_post_pick_axis_pipeline()
    reset_first_tray_pick_wait()
    unlock_robot_sequence()

    if is_grip_attached:
        open_grip_no_jump()

    event_log(f"[자동 시퀀스 오류] {reason}")

    robot_sequence["state"] = "ERROR"
    robot_sequence["dynamic_pick_targets"] = None
    robot_sequence["place_slot"] = None
    is_auto_sequence_running = False


def cancel_grip_stabilization():
    grip_stabilize_controller["active"] = False
    grip_stabilize_controller["frames"] = 0
    grip_stabilize_controller["box_path"] = None


def start_grip_stabilization(box_path):
    grip_stabilize_controller["active"] = True
    grip_stabilize_controller["frames"] = 0
    grip_stabilize_controller["box_path"] = box_path

    event_log(
        f"[흡착 안정화 시작] "
        f"ID={box_path_id_dict.get(box_path, None)} | "
        f"{ATTACH_STABILIZE_FRAMES_REQUIRED}프레임 고정"
    )


def update_grip_stabilization():
    if not grip_stabilize_controller.get("active", False):
        return False

    box_path = grip_stabilize_controller.get("box_path", None)

    if (
        not is_grip_attached
        or box_path is None
        or get_prim(box_path) is None
    ):
        cancel_grip_stabilization()
        abort_robot_sequence("흡착 안정화 중 박스 연결 해제")
        return False

    update_attached_box_hard_lock()
    grip_stabilize_controller["frames"] += 1

    if (
        grip_stabilize_controller["frames"]
        < int(ATTACH_STABILIZE_FRAMES_REQUIRED)
    ):
        return False

    cancel_grip_stabilization()

    event_log(
        f"[흡착 안정화 완료] "
        f"ID={box_path_id_dict.get(box_path, None)}"
    )

    set_robot_sequence_state("LIFT_Y")

    started = start_scurve_motion(
        "픽업 후 고속 Y 상승",
        {"Y": AUTO_PICK_Y_UP_POSITION}
    )

    if not started:
        abort_robot_sequence("픽업 후 Y 상승 시작 실패")

    return started


def start_initial_home_motion():
    global system_ready

    system_ready = False
    set_robot_sequence_state("HOME_INITIAL")

    return start_scurve_motion(
        "초기 위치 이동",
        {
            "X1": AUTO_HOME_X1_POSITION,
            "X2": AUTO_HOME_X2_POSITION,
            "Y": AUTO_HOME_Y_POSITION,
            "Z": AUTO_HOME_Z_POSITION
        }
    )


def start_auto_pick_sequence_for_box(box_path, stopped_world_position=None):
    global is_auto_sequence_running

    if box_path is None:
        event_log("[자동 시퀀스 오류] 대상 박스 없음")
        return False

    if get_prim(box_path) is None:
        event_log(f"[자동 시퀀스 오류] 박스 Prim 없음 | {box_path}")
        return False

    if is_auto_sequence_running:
        return False

    if is_robot_sequence_locked():
        return False

    if is_grip_attached:
        return False

    if robot_sequence.get(
        "state",
        "IDLE"
    ) != "IDLE":
        return False

    if box_path in auto_sequence_done_box_set:
        return False

    is_auto_sequence_running = True
    lock_robot_sequence(
        box_path,
        "픽업 시작부터 적재 및 HOME 복귀 완료까지"
    )
    auto_sequence_done_box_set.add(box_path)
    robot_sequence["target_box"] = box_path

    # 대기 목록에서 현재 픽업할 박스를 제거해 다음 박스만 대기로 판정한다.
    stopped_box_world_position_dict.pop(box_path, None)

    if stopped_world_position is None:
        stopped_world_position = get_world_position(box_path)

    if stopped_world_position is not None:
        stopped_world_position = copy_world_position(stopped_world_position)

    dynamic_pick_targets = calculate_dynamic_pick_targets(stopped_world_position)

    if dynamic_pick_targets is None:
        event_log(
            f"[자동 시퀀스 오류] "
            f"ID={box_path_id_dict.get(box_path, None)} | "
            "정지 박스 좌표 계산 실패"
        )
        is_auto_sequence_running = False
        auto_sequence_done_box_set.discard(box_path)
        return False

    robot_sequence["dynamic_pick_targets"] = dynamic_pick_targets
    robot_sequence["place_slot"] = (
        1 if int(auto_place_count) % 2 == 0 else 2
    )

    box_applied_yaw = float(box_yaw_dict.get(box_path, 0.0))
    gripper_target_yaw = calculate_gripper_logical_target(box_applied_yaw)
    robot_sequence["target_gripper_logical_yaw"] = float(gripper_target_yaw)

    event_log(
        f"[자동 시퀀스 시작] "
        f"ID={box_path_id_dict.get(box_path, None)} | "
        f"박스각={box_applied_yaw:.2f}도 | "
        f"그리퍼 목표각={gripper_target_yaw:.2f}도 | "
        f"적재순번={robot_sequence['place_slot']}번"
    )

    should_wait_for_tray2 = bool(FIRST_TRAY_PICK_WAIT_ENABLE)

    if FIRST_TRAY_PICK_WAIT_ONLY_PLACE_SLOT_1:
        should_wait_for_tray2 = (
            should_wait_for_tray2
            and int(robot_sequence["place_slot"]) == 1
        )

    # 각 트레이의 1번 적재 박스는 정상 박스가 픽업 정지 위치에 도착해도
    # X1/X2/Z를 움직이지 않고 실제 다음/활성 Tray의 X=250 도착을 먼저 기다린다.
    if should_wait_for_tray2:
        armed = arm_first_tray_pick_wait(box_path)

        if not armed:
            is_auto_sequence_running = False
            unlock_robot_sequence()
            auto_sequence_done_box_set.discard(box_path)

        return bool(armed)

    started = _start_pick_xz_motion(box_path)

    if not started:
        is_auto_sequence_running = False
        unlock_robot_sequence()
        auto_sequence_done_box_set.discard(box_path)

    return bool(started)


def force_start_auto_pick_sequence():
    target_box = get_current_target_box()

    if target_box is None:
        event_log("[자동 시퀀스 오류] 현재 생성된 박스가 없습니다")
        return False

    stop_conveyor()

    return start_auto_pick_sequence_for_box(
        target_box,
        get_world_position(target_box)
    )


def handle_robot_motion_complete(completed_label, actual_positions):
    global is_auto_sequence_running
    global system_ready
    global auto_place_count

    state = robot_sequence.get("state", "IDLE")
    target_box = robot_sequence.get("target_box", None)

    if state == "HOME_INITIAL":
        system_ready = True
        robot_sequence["state"] = "IDLE"

        event_log(
            f"[시스템 준비 완료] "
            f"HOME=({AUTO_HOME_X1_POSITION:.1f}, "
            f"{AUTO_HOME_X2_POSITION:.1f}, "
            f"{AUTO_HOME_Y_POSITION:.1f}, "
            f"{AUTO_HOME_Z_POSITION:.1f})"
        )

        if try_start_pending_pick_sequence():
            return

        start_conveyor()
        return

    if state == "PICK_XZ":
        if target_box is None:
            abort_robot_sequence("그리퍼 회전 대상 박스 없음")
            return

        # Tray2 Z=239 대기는 PICK_XZ 시작 전에 이미 완료되었다.
        # 박스 위 이동이 끝나면 기존 순서대로 그리퍼 회전과 Y 하강을 진행한다.
        _start_pick_gripper_rotation(target_box)
        return

    if state == "PICK_Y":
        if target_box is None:
            abort_robot_sequence(
                "흡착 대상 박스 없음"
            )
            return

        # v46에서 검증된 Hard-Lock 흡착 방식 유지
        grip_success = close_grip_no_jump(
            target_box,
            force=True
        )

        if not grip_success:
            abort_robot_sequence(
                "그리퍼 흡착 실패"
            )
            return

        event_log(
            f"[흡착 완료] "
            f"ID={box_path_id_dict.get(target_box, None)}"
        )

        # 1. 컨베이어 선이송과
        # 2. Y=50 -> Z=-270 로봇 이동을 서로 독립적으로 시작한다.
        #
        # 컨베이어 속성 읽기/설정이 한 프레임 실패해도
        # Y/Z 이동은 절대 중단하지 않는다.
        force_start_next_box_feed(
            target_box
        )

        start_post_pick_y50_pipeline(
            target_box
        )

        return

    if state == "LIFT_Y":
        # S-Curve Y=50 완료 콜백
        start_post_pick_z_neg270_pipeline(
            target_box
        )
        return

    if state == "POST_PICK_Z_NEG270":
        # S-Curve Z=-270 완료 콜백
        finish_post_pick_z_neg270_pipeline(
            target_box
        )
        return

    if state == "TRAY_SLOT_PREPARE":
        place_slot, config = get_place_slot_config(
            tray_follow_controller.get("place_slot")
        )

        # 2번 박스는 트레이 좌표를 기다리지 않는다.
        # X1/X2=-200, Y=25, Z=-250 준비 이동이 끝난 바로 다음 프레임에
        # X1/X2=-264, Y=-50, Z=-250 동기 대각선 적재를 시작한다.
        if not bool(config.get("wait_for_tray_trigger", True)):
            event_log(
                f"[트레이 이동] {config['label']} 준비 완료 | "
                f"X1/X2={float(config['prepare_x']):.1f} | "
                f"Y={AUTO_PICK_Y_UP_POSITION:.1f} | "
                f"Z={TRAY_CONFIRMED_PLACE_Z_POSITION:.1f} | "
                "트레이 좌표 대기 생략, 즉시 대각선 적재"
            )

            if not _start_tray_diagonal_motion():
                abort_robot_sequence(
                    f"{config['label']} 즉시 대각선 적재 시작 실패"
                )
            return

        tray_follow_controller["phase"] = "WAIT_PLACE_TRIGGER"
        tray_follow_controller["phase_elapsed_sec"] = 0.0
        tray_position = get_world_position(
            tray_follow_controller.get("selected_tray_path") or TRAY_PATH
        )
        if tray_position is not None:
            tray_follow_controller["previous_tray_x"] = float(tray_position[0])

        set_robot_sequence_state("TRAY_WAIT_PLACE_TRIGGER")
        event_log(
            f"[트레이 이동] {config['label']} 준비 완료 | "
            f"X1/X2={float(config['prepare_x']):.1f} | "
            f"Y={AUTO_PICK_Y_UP_POSITION:.1f} | "
            f"Z={TRAY_CONFIRMED_PLACE_Z_POSITION:.1f} | "
            f"Tray X={float(config['place_trigger_x']):.1f} 대기"
        )
        return

    if state == "TRAY_SLOT_DIAGONAL":
        place_slot, config = get_place_slot_config(
            tray_follow_controller.get("place_slot")
        )
        tray_follow_controller["phase"] = "PRE_RELEASE_HOLD"
        tray_follow_controller["phase_elapsed_sec"] = 0.0
        set_robot_sequence_state("TRAY_PRE_RELEASE_HOLD")
        event_log(
            f"[트레이 이동] {config['label']} 적재 위치 도착 | "
            f"X1/X2={float(config['final_x']):.1f} | "
            f"Y={TRAY_CONFIRMED_PLACE_Y_POSITION:.1f} | "
            f"Z={TRAY_CONFIRMED_PLACE_Z_POSITION:.1f} | "
            f"{TRAY_PRE_RELEASE_HOLD_SEC:.1f}초 후 흡착 해제"
        )
        return

    if state == "TRAY_SLOT_RISE":
        tray_follow_controller["phase"] = "WAIT_RELEASE_COMPLETE"
        tray_follow_controller["phase_elapsed_sec"] = 0.0
        set_robot_sequence_state("TRAY_WAIT_RELEASE_COMPLETE")
        return

    if state == "HOME_RETURN":
        completed_place_slot = robot_sequence.get("place_slot")
        auto_place_count += 1

        event_log(
            f"[자동 시퀀스 완료] "
            f"ID={box_path_id_dict.get(target_box, None)} | "
            f"적재순번={completed_place_slot}번 | "
            f"다음 적재순번={1 if auto_place_count % 2 == 0 else 2}번 | "
            "트레이 내부 적재 및 원점 복귀 완료"
        )

        robot_sequence["state"] = "IDLE"
        robot_sequence["target_box"] = None
        robot_sequence["target_gripper_logical_yaw"] = GRIPPER_JOINT_HOME_LOGICAL_DEG
        robot_sequence["dynamic_pick_targets"] = None
        robot_sequence["place_slot"] = None
        is_auto_sequence_running = False
        unlock_robot_sequence()
        cancel_pack_tray_trigger()
        reset_next_box_preload()
        reset_forced_next_box_feed()
        reset_post_pick_axis_pipeline()
        reset_first_tray_pick_wait()

        # 한 트레이에 2개 박스 적재가 끝나면 다음 미공급 Tray를 공급한다.
        # 이전 공급 사이클에서 Push=30 복귀 후 Upper가 이미 다음 위치로
        # 이동해 있으므로, 요청 시 Upper 정착 확인 후 Push 공급을 진행한다.
        if (
            TRAY_LOADER_AUTO_START_AFTER_FULL_TRAY
            and int(TRAY_LOADER_BOXES_PER_TRAY) > 0
            and int(auto_place_count) > 0
            and int(auto_place_count) % int(TRAY_LOADER_BOXES_PER_TRAY) == 0
        ):
            try:
                request_tray_loader_start(
                    reason=(
                        f"정상 박스 {auto_place_count}개 적재 완료 | "
                        f"트레이당 {TRAY_LOADER_BOXES_PER_TRAY}개"
                    ),
                    source_box_path=target_box,
                )
            except Exception:
                pass

        pending_box = get_pending_stopped_pick_box()

        if pending_box is not None:
            stop_conveyor()
            event_log(
                f"[그리퍼 이동] 복귀 즉시 다음 박스 픽업 | "
                f"ID={box_path_id_dict.get(pending_box, None)}"
            )

            if try_start_pending_pick_sequence():
                return

        resume_conveyor_after_sequence()
        return


def handle_gripper_joint_motion_complete(completed_label, completed_angle):
    state = robot_sequence.get("state", "IDLE")

    if state == "GRIPPER_ROTATE_PICK":
        set_robot_sequence_state("PICK_Y")

        started = start_scurve_motion(
            "픽업 Y 하강",
            {"Y": AUTO_PICK_Y_COMMAND_POSITION},
            completion_targets={"Y": AUTO_PICK_Y_REACHED_POSITION},
            tolerances={"Y": PICK_Y_REACHED_TOLERANCE}
        )

        if not started:
            abort_robot_sequence("픽업 Y 하강 시작 실패")
        return

    if state == "GRIPPER_ROTATE_PLACE_HOME":
        sync_attached_box_rotation_to_gripper()

        if not arm_pack_tray_trigger():
            abort_robot_sequence(
                "두 박스 좌표 적재 트레이 준비 시작 실패"
            )

        return


# ==================================================
# [15. 리젝터 - YOLO 불량 박스 제거]
# ==================================================

def read_rejector_drive_settings():
    """
    현재 리젝터 Angular Drive 설정을 저장한다.
    값을 변경하지 않고 읽기만 한다.
    """
    joint_prim = get_prim(
        REJECTOR_JOINT_PATH
    )

    if joint_prim is None:
        return None

    try:
        drive_api = UsdPhysics.DriveAPI.Get(
            joint_prim,
            UsdPhysics.Tokens.angular
        )

        if not drive_api:
            return None

        return {
            "type": drive_api.GetTypeAttr().Get(),
            "stiffness": drive_api.GetStiffnessAttr().Get(),
            "damping": drive_api.GetDampingAttr().Get(),
            "max_force": drive_api.GetMaxForceAttr().Get(),
            "target_velocity": drive_api.GetTargetVelocityAttr().Get()
        }

    except Exception:
        return None


def apply_rejector_boost_drive():
    """
    불량 배출 동작 중에만 리젝터 Drive를 강하게 설정한다.

    코드 로드 또는 Timeline PLAY에서는 호출하지 않는다.
    """
    joint_prim = get_prim(
        REJECTOR_JOINT_PATH
    )

    if joint_prim is None:
        event_log(
            f"[리젝터 강화 오류] Joint 없음 | "
            f"{REJECTOR_JOINT_PATH}"
        )
        return False

    try:
        drive_api = UsdPhysics.DriveAPI.Get(
            joint_prim,
            UsdPhysics.Tokens.angular
        )

        if not drive_api:
            drive_api = UsdPhysics.DriveAPI.Apply(
                joint_prim,
                UsdPhysics.Tokens.angular
            )

        drive_api.CreateTypeAttr().Set(
            UsdPhysics.Tokens.acceleration
        )
        drive_api.CreateStiffnessAttr().Set(
            float(REJECTOR_BOOST_STIFFNESS)
        )
        drive_api.CreateDampingAttr().Set(
            float(REJECTOR_BOOST_DAMPING)
        )
        drive_api.CreateMaxForceAttr().Set(
            float(REJECTOR_BOOST_MAX_FORCE)
        )
        drive_api.CreateTargetVelocityAttr().Set(
            0.0
        )

        event_log(
            "[리젝터 일시 강화 적용] "
            f"Type={REJECTOR_BOOST_TYPE} | "
            f"K={REJECTOR_BOOST_STIFFNESS:.0f} | "
            f"D={REJECTOR_BOOST_DAMPING:.0f} | "
            f"MaxForce={REJECTOR_BOOST_MAX_FORCE:.0f}"
        )

        return True

    except Exception as error:
        event_log(
            f"[리젝터 강화 예외] {error}"
        )
        return False


def restore_rejector_drive_settings(
    saved_settings
):
    """
    리젝터가 0도로 복귀한 뒤 원래 Angular Drive 설정으로 되돌린다.
    """
    if not isinstance(
        saved_settings,
        dict
    ):
        return False

    joint_prim = get_prim(
        REJECTOR_JOINT_PATH
    )

    if joint_prim is None:
        return False

    try:
        drive_api = UsdPhysics.DriveAPI.Get(
            joint_prim,
            UsdPhysics.Tokens.angular
        )

        if not drive_api:
            return False

        saved_type = saved_settings.get(
            "type",
            None
        )

        if saved_type is not None:
            drive_api.CreateTypeAttr().Set(
                saved_type
            )

        saved_stiffness = saved_settings.get(
            "stiffness",
            None
        )

        if saved_stiffness is not None:
            drive_api.CreateStiffnessAttr().Set(
                float(saved_stiffness)
            )

        saved_damping = saved_settings.get(
            "damping",
            None
        )

        if saved_damping is not None:
            drive_api.CreateDampingAttr().Set(
                float(saved_damping)
            )

        saved_max_force = saved_settings.get(
            "max_force",
            None
        )

        if saved_max_force is not None:
            drive_api.CreateMaxForceAttr().Set(
                float(saved_max_force)
            )

        saved_target_velocity = (
            saved_settings.get(
                "target_velocity",
                None
            )
        )

        if saved_target_velocity is not None:
            drive_api.CreateTargetVelocityAttr().Set(
                float(saved_target_velocity)
            )

        event_log(
            "[리젝터 Drive 원상 복구] "
            f"Type={saved_type} | "
            f"K={saved_stiffness} | "
            f"D={saved_damping} | "
            f"MaxForce={saved_max_force}"
        )

        return True

    except Exception as error:
        event_log(
            f"[리젝터 Drive 복구 예외] {error}"
        )
        return False


def get_rejector_target_attr():
    """
    리젝터 Revolute Joint의 Angular Drive Target Position 속성을 반환한다.
    """
    joint_prim = get_prim(
        REJECTOR_JOINT_PATH
    )

    if joint_prim is None:
        event_log(
            f"[리젝터 오류] Joint Prim 없음 | "
            f"{REJECTOR_JOINT_PATH}"
        )
        return None

    target_attr = joint_prim.GetAttribute(
        "drive:angular:physics:targetPosition"
    )

    if not target_attr.IsValid():
        event_log(
            f"[리젝터 오류] Angular Drive 없음 | "
            f"{REJECTOR_JOINT_PATH}"
        )
        return None

    return target_attr


async def move_rejector_target_scurve(
    start_angle,
    target_angle,
    frame_count
):
    """
    Angular Drive Target Position을 한 번에 점프시키지 않고
    smoothstep S-Curve로 조금씩 변경한다.
    """
    target_attr = get_rejector_target_attr()

    if target_attr is None:
        return False

    app = omni.kit.app.get_app()

    start_angle = float(start_angle)
    target_angle = float(target_angle)
    frame_count = max(
        1,
        int(frame_count)
    )

    for frame_index in range(
        frame_count + 1
    ):
        t = (
            float(frame_index)
            / float(frame_count)
        )

        # cubic smoothstep: 0 → 1, 시작/끝 속도 0
        smooth_t = (
            t * t * (3.0 - 2.0 * t)
        )

        command_angle = (
            start_angle
            + (
                target_angle
                - start_angle
            )
            * smooth_t
        )

        target_attr.Set(
            float(command_angle)
        )

        velocity_attr = get_prim(
            REJECTOR_JOINT_PATH
        ).GetAttribute(
            "drive:angular:physics:targetVelocity"
        )

        if velocity_attr.IsValid():
            velocity_attr.Set(
                0.0
            )

        await app.next_update_async()

    target_attr.Set(
        float(target_angle)
    )

    return True


async def hold_rejector_target(
    target_angle,
    frame_count
):
    """
    지정 각도를 update 프레임 수만큼 유지한다.
    """
    target_attr = get_rejector_target_attr()

    if target_attr is None:
        return False

    app = omni.kit.app.get_app()

    target_attr.Set(
        float(target_angle)
    )

    for _ in range(
        max(
            0,
            int(frame_count)
        )
    ):
        target_attr.Set(
            float(target_angle)
        )

        await app.next_update_async()

    return True


async def run_rejector(
    trigger_box_path=None,
    delete_trigger_box_at_push=False,
    detected_z=None,
):
    """
    불량 박스 리젝터 왕복 동작.

    현재 버전에서는 불량 박스를 삭제하지 않는다.
    인자는 기존 호출부 호환을 위해 유지한다.
    """
    global is_rejector_moving

    rejector_cycle_completed = False

    if is_rejector_moving:
        event_log("[리젝터] 이미 동작 중입니다.")
        return False

    target_attr = get_rejector_target_attr()

    if target_attr is None:
        return False

    saved_settings = read_rejector_drive_settings()
    is_rejector_moving = True

    try:
        if not apply_rejector_boost_drive():
            return False

        current_target = target_attr.Get()

        if current_target is None:
            current_target = REJECTOR_HOME_VALUE

        current_target = float(current_target)

        if abs(current_target - float(REJECTOR_HOME_VALUE)) > 0.1:
            recovered = await move_rejector_target_scurve(
                current_target,
                REJECTOR_HOME_VALUE,
                REJECTOR_HOME_RECOVERY_FRAMES
            )

            if not recovered:
                return False

        await hold_rejector_target(
            REJECTOR_HOME_VALUE,
            REJECTOR_HOME_HOLD_FRAMES
        )

        moved_out = await move_rejector_target_scurve(
            REJECTOR_HOME_VALUE,
            REJECTOR_PUSH_VALUE,
            REJECTOR_MOVE_FRAMES
        )

        if not moved_out:
            return False

        target_attr.Set(float(REJECTOR_PUSH_VALUE))

        await hold_rejector_target(
            REJECTOR_PUSH_VALUE,
            REJECTOR_HOLD_FRAMES
        )

        moved_home = await move_rejector_target_scurve(
            REJECTOR_PUSH_VALUE,
            REJECTOR_HOME_VALUE,
            REJECTOR_MOVE_FRAMES
        )

        if not moved_home:
            return False

        target_attr.Set(float(REJECTOR_HOME_VALUE))
        rejector_cycle_completed = True

    except Exception as error:
        event_log(f"[리젝터 예외] {error}")

        try:
            target_attr.Set(float(REJECTOR_HOME_VALUE))
        except Exception:
            pass

    finally:
        restore_rejector_drive_settings(saved_settings)
        is_rejector_moving = False

    return bool(rejector_cycle_completed)



def process_rejector_passed_box(
    box_path,
    current_pos
):
    """
    불량 박스가 Z=611.18216에 도착하거나 통과하면 즉시:

        Target Position 0
        → -100
        → 0

    순서로 리젝터를 작동한다.
    """
    global passed_box_count

    passed_box_count += 1

    result = box_result_dict.get(
        box_path,
        None
    )
    label = box_label_dict.get(
        box_path,
        "unknown"
    )
    yolo_id = box_path_id_dict.get(
        box_path,
        None
    )

    debug_print(
        "=================================================="
    )
    debug_print(
        "[리젝터 Z 위치 통과 감지]"
    )
    debug_print(
        "대상 박스:",
        box_path
    )
    debug_print(
        "YOLO ID:",
        yolo_id
    )
    debug_print(
        "통과 순번:",
        passed_box_count
    )
    debug_print(
        "YOLO 판정 result:",
        result,
        "(1=정상, 0=불량)"
    )
    debug_print(
        "Label:",
        label
    )
    debug_print(
        f"기준 Z={REJECTOR_TRIGGER_Z:.5f}"
    )
    debug_print(
        f"현재 Z={float(current_pos[2]):.5f}"
    )

    if result != 0:
        debug_print(
            "정상 또는 미판정 박스이므로 리젝터 미작동"
        )
        debug_print(
            "=================================================="
        )
        return False

    event_log(
        f"[리젝터 가동] "
        f"ID={yolo_id} | "
        f"Box Z={float(current_pos[2]):.5f} | "
        f"기준 Z={REJECTOR_TRIGGER_Z:.5f} | "
        f"{REJECTOR_HOME_VALUE:.1f}도 "
        f"→ {REJECTOR_PUSH_VALUE:.1f}도 "
        f"→ {REJECTOR_HOME_VALUE:.1f}도 즉시 실행"
    )

    debug_print(
        "불량 박스: 리젝터 즉시 실행"
    )
    debug_print(
        "=================================================="
    )

    asyncio.ensure_future(
        run_rejector(box_path)
    )

    return True


# ==================================================
# [16. YOLO TCP 수신 → 정상 원본 색상 / 불량 빨간색 박스 생성]
# ==================================================

def update_existing_box_color(box_path, result, label=None, yaw_deg=None):
    color_rgb, auto_label, apply_defect_color = get_box_color_by_result(result)

    if label is None:
        label = auto_label

    prim = get_prim(box_path)

    if prim is None:
        debug_print("[YOLO TCP] 판정 갱신 실패. 박스 prim 없음:", box_path)
        return False

    try:
        normalized_result = (
            int(result)
            if result is not None
            else None
        )
    except Exception:
        normalized_result = None

    box_result_dict[box_path] = normalized_result
    box_label_dict[box_path] = label

    remove_pending_inspection_box(
        box_path
    )

    if normalized_result == 0:
        changed_now = apply_defect_red_if_needed(
            box_path
        )

        color_ok = True

        if changed_now:
            color_status = "불량 판정 즉시 빨간색 적용"
        else:
            color_status = "불량 빨간색 이미 적용"
    else:
        color_ok = apply_box_color(
            box_path,
            BOX_COLOR_RGB
        )

        defect_red_applied_box_set.discard(
            box_path
        )

        color_status = "정상/미판정 회색 적용"

    # 같은 ID의 박스에 각도가 추가로 들어오면 기존 박스 회전도 갱신한다.
    if yaw_deg is not None:
        apply_box_yaw(box_path, yaw_deg)

    event_log(
        f"[박스 갱신] ID={box_path_id_dict.get(box_path, None)} | "
        f"각도={box_yaw_dict.get(box_path, 0.0):.2f}도 ({BOX_YAW_AXIS}축) | "
        f"판정={result_to_text(result)}"
    )

    debug_print("==================================================")
    debug_print("[YOLO TCP] 기존 박스 판정 갱신 완료")
    debug_print("Path:", box_path)
    debug_print("Result:", result)
    debug_print("Label:", label)
    debug_print("Color Status:", color_status)
    debug_print("Color Applied:", color_ok)
    debug_print("Received Yaw:", yaw_deg)
    debug_print("Applied Yaw:", box_yaw_dict.get(box_path, 0.0))
    debug_print("==================================================")

    return color_ok


def test_master_bridge_protocol():
    sample_messages = [
        {
            "cmd": "MASTER_EVENT",
            "source": "MASTER",
            "category": "SYSTEM",
            "event": "MASTER_STARTED",
            "data": {
                "mcu_port": "COM17",
                "mcu_baud": 57600,
                "openrb_port": "COM14",
                "openrb_baud": 1000000,
            },
            "timestamp": time.time(),
        },
        {
            "cmd": "MASTER_EVENT",
            "source": "MASTER",
            "category": "CONNECTION",
            "event": "MCU_UART_CONNECTED",
            "data": {
                "port": "COM17",
                "baud": 57600,
            },
            "timestamp": time.time(),
        },
        {
            "cmd": "MASTER_EVENT",
            "source": "MASTER",
            "category": "MCU",
            "event": "ROBOT_STATE",
            "data": {
                "step": 4,
                "previous_step": 3,
                "changed": True,
                "count": 1,
            },
            "raw": "4",
            "timestamp": time.time(),
        },
        {
            "cmd": "MASTER_EVENT",
            "source": "MASTER",
            "category": "MCU",
            "event": "TX_COORDINATE",
            "data": {
                "x": 10.0,
                "y": 20.0,
                "z": 30.0,
                "success": True,
            },
            "raw": "G,10.0,20.0,30.0",
            "timestamp": time.time(),
        },
    ]

    for sample in sample_messages:
        handle_yolo_tcp_message(sample)

    snapshot = get_master_runtime_state()

    event_log(
        f"[MASTER 연결] 프로토콜 테스트 완료 | "
        f"STEP={snapshot.get('robot_step')} | "
        f"STM={snapshot.get('mcu_port')} | "
        f"XYZ=("
        f"{snapshot.get('mcu_target_x')}, "
        f"{snapshot.get('mcu_target_y')}, "
        f"{snapshot.get('mcu_target_z')})"
    )

    return snapshot


def handle_yolo_tcp_message(msg):
    if not isinstance(msg, dict):
        debug_print("[YOLO TCP] 처리 불가 메시지:", msg)
        return

    cmd = str(msg.get("cmd", "")).strip().upper()

    event_log(
        f"[통신 상태] 메인 스레드 처리 시작 | {_terminal_message_summary(msg)}"
    )

    # Master 전체 상태/STM 이벤트
    if cmd == "MASTER_EVENT":
        handle_master_event(msg)
        return

    # 검사 카메라 코드에서 전송하는 메시지:
    # {
    #   "cmd": "INSPECTION_RESULT",
    #   "value": 0 또는 1,
    #   "track_id": ByteTrack ID
    # }
    if cmd == "INSPECTION_RESULT":
        event_log(
            f"[통신 RX] 검사 결과 처리 | "
            f"track_id={msg.get('track_id')} | value={msg.get('value')} | "
            f"label={msg.get('label')} | source={msg.get('source')}"
        )
        handle_master_compatible_inspection_result(
            value=msg.get("value", None),
            inspection_track_id=msg.get(
                "track_id",
                None
            ),
            label=msg.get("label", None),
            timestamp=msg.get("timestamp", time.time()),
        )
        return

    box_id = msg.get("id", msg.get("track_id", None))
    track_id = msg.get("track_id", box_id)
    result = msg.get("result", None)
    label = msg.get("label", None)

    # rotation_deg.yaw 또는 yaw/yaw_deg/angle 값을 읽는다.
    yaw_deg = extract_received_yaw(msg)

    # CREATE_BOX JSON의 Depth position X/Y/Z를 읽는다.
    # X -> Isaac X, Y -> Isaac Z, Isaac Y는 고정한다.
    # Depth Z는 생성 좌표에 사용하지 않는다.
    received_position = extract_received_position(
        msg
    )

    if cmd not in ["CREATE_BOX", "DECISION"]:
        debug_print("[YOLO TCP] 알 수 없는 cmd:", cmd, msg)
        return

    if box_id is None:
        debug_print("[YOLO TCP] id가 없어 박스 생성 불가:", msg)
        return

    try:
        box_id = int(box_id)
    except Exception:
        debug_print("[YOLO TCP] id가 정수가 아님:", msg)
        return

    # v51.1: 새 박스 생성은 첫 번째 Depth의 CREATE_BOX만 허용한다.
    # DECISION은 이미 같은 ID로 생성된 박스의 결과/색상 갱신만 가능하다.
    if cmd == "DECISION" and box_id not in box_id_path_dict:
        event_log(
            f"[DECISION 무시] ID={box_id} | "
            "기존 첫 번째 Depth 생성 박스 없음"
        )
        return

    if box_id in box_id_path_dict:
        box_path = box_id_path_dict[box_id]

        if get_prim(box_path) is not None:
            update_existing_box_color(
                box_path,
                result,
                label,
                yaw_deg=yaw_deg
            )
            return

    spawn_box(
        box_id=box_id,
        result=result,
        track_id=track_id,
        label=label,
        source="YOLO",
        yaw_deg=yaw_deg,
        received_position=received_position
    )


# ==================================================
# [로컬 테스트] 통신 없이 정상 원본 색상 / 불량 빨간색 박스 생성
# ==================================================

def local_test_normal_box(test_id=1001, yaw_deg=30.0):
    handle_yolo_tcp_message({
        "cmd": "CREATE_BOX",
        "id": int(test_id),
        "track_id": int(test_id),
        "result": 1,
        "label": "normal",
        "position": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0
        },
        "position_unit": "mm",
        "coordinate_system": "depth_xy",
        "rotation_deg": {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": float(yaw_deg)
        },
        "source": "LOCAL_TEST"
    })

    # v51.21: 로컬 생성 후 Pivot/Transform Manipulator 선택 해제
    clear_stage_selection()


def local_test_defect_box(test_id=1002, yaw_deg=-30.0):
    handle_yolo_tcp_message({
        "cmd": "CREATE_BOX",
        "id": int(test_id),
        "track_id": int(test_id),
        "result": 0,
        "label": "defect",
        "position": {
            "x": 25.0,
            "y": 0.0,
            "z": 0.0
        },
        "position_unit": "mm",
        "coordinate_system": "depth_xy",
        "rotation_deg": {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": float(yaw_deg)
        },
        "source": "LOCAL_TEST"
    })

    # v51.21: 로컬 생성 후 Pivot/Transform Manipulator 선택 해제
    clear_stage_selection()


def local_test_angle_box(test_id=1003, yaw_deg=45.0):
    """색상 판정 없이 기존 색상 박스를 지정 각도로 생성한다."""
    handle_yolo_tcp_message({
        "cmd": "CREATE_BOX",
        "id": int(test_id),
        "track_id": int(test_id),
        "result": -1,
        "label": "opencv_box",
        "position": {
            "x": -25.0,
            "y": 0.0,
            "z": 0.0
        },
        "position_unit": "mm",
        "coordinate_system": "depth_xy",
        "rotation_deg": {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": float(yaw_deg)
        },
        "source": "LOCAL_ANGLE_TEST"
    })



def get_local_ipv4_addresses():
    """
    Isaac Sim PC에서 다른 장치가 접속할 때 사용할 수 있는
    로컬 IPv4 후보를 반환한다.
    """
    addresses = []

    try:
        host_name = socket.gethostname()

        for address in socket.gethostbyname_ex(host_name)[2]:
            if address and not address.startswith("127."):
                if address not in addresses:
                    addresses.append(address)

    except Exception:
        pass

    return addresses


def send_tcp_ack(conn, status, detail=""):
    """
    송신 측에서 연결 및 수신 성공 여부를 확인할 수 있도록 ACK를 보낸다.
    송신 코드가 ACK를 읽지 않아도 동작에는 영향이 없다.
    """
    try:
        response = {
            "status": str(status),
            "detail": str(detail)
        }

        payload = (
            json.dumps(
                response,
                ensure_ascii=False
            )
            + "\n"
        )

        conn.sendall(
            payload.encode("utf-8")
        )

    except Exception:
        pass


def enqueue_tcp_message(msg, addr):
    """
    TCP 스레드에서 받은 메시지를 Isaac Sim 메인 업데이트 큐에 넣는다.
    """
    if not isinstance(msg, dict):
        event_log(
            f"[ISAAC TCP] 처리 불가 데이터 | "
            f"{addr[0]}:{addr[1]} | {msg}"
        )
        return False

    cmd = msg.get("cmd", "")
    box_id = msg.get("id", msg.get("track_id", None))

    mark_master_tcp_client(
        addr,
        msg
    )

    yolo_tcp_queue.put(msg)

    event_log(
        f"[통신 RX] TCP {addr[0]}:{addr[1]} -> Isaac:9999 | "
        f"{_terminal_message_summary(msg)}"
    )

    return True


def process_tcp_text_line(line, conn, addr):
    """
    한 줄 단위 JSON 또는 일반 문자열 명령을 처리한다.
    """
    line = str(line).strip()

    if line == "":
        return

    try:
        msg = json.loads(line)

    except json.JSONDecodeError as error:
        event_log(
            f"[ISAAC TCP] JSON 해석 실패 | "
            f"{addr[0]}:{addr[1]} | {error}"
        )

        send_tcp_ack(
            conn,
            "error",
            f"invalid_json: {error}"
        )
        return

    if enqueue_tcp_message(msg, addr):
        send_tcp_ack(
            conn,
            "ok",
            f"queued:{msg.get('cmd', '')}"
        )


def extract_complete_json_messages(buffer, conn, addr):
    """
    송신 코드가 JSON 끝에 줄바꿈을 붙이지 않아도
    완성된 JSON 객체라면 즉시 추출해 처리한다.

    JSON이 아직 덜 들어온 상태라면 남은 buffer를 그대로 반환한다.
    """
    decoder = json.JSONDecoder()
    remaining = str(buffer)

    while True:
        remaining = remaining.lstrip()

        if remaining == "":
            return ""

        try:
            msg, end_index = decoder.raw_decode(
                remaining
            )

        except json.JSONDecodeError:
            return remaining

        if enqueue_tcp_message(msg, addr):
            send_tcp_ack(
                conn,
                "ok",
                f"queued:{msg.get('cmd', '')}"
            )

        remaining = remaining[end_index:]


def yolo_tcp_client_thread(conn, addr):
    event_log(
        f"[통신 연결] Master/송신 클라이언트 연결 성공 | "
        f"peer={addr[0]}:{addr[1]} | local_port={YOLO_TCP_PORT}"
    )

    buffer = ""

    try:
        conn.settimeout(1.0)

        with conn:
            while yolo_tcp_state.get("running", False):
                try:
                    data = conn.recv(4096)

                except socket.timeout:
                    continue

                if not data:
                    break

                buffer += data.decode(
                    "utf-8",
                    errors="ignore"
                )

                # 1. 기존 방식: 줄바꿈 기준 JSON 처리
                while "\n" in buffer:
                    line, buffer = buffer.split(
                        "\n",
                        1
                    )

                    process_tcp_text_line(
                        line,
                        conn,
                        addr
                    )

                # 2. 개선 방식: 줄바꿈이 없어도 완성된 JSON이면 처리
                buffer = extract_complete_json_messages(
                    buffer,
                    conn,
                    addr
                )

            # 연결 종료 전에 남은 완성 JSON이 있으면 마지막으로 처리
            if buffer.strip():
                remaining = extract_complete_json_messages(
                    buffer,
                    conn,
                    addr
                )

                if remaining.strip():
                    event_log(
                        f"[ISAAC TCP] 미완성 데이터 폐기 | "
                        f"{addr[0]}:{addr[1]} | "
                        f"{remaining[:120]}"
                    )

    except Exception as error:
        event_log(
            f"[ISAAC TCP] 클라이언트 처리 오류 | "
            f"{addr[0]}:{addr[1]} | {error}"
        )

    unmark_master_tcp_client(
        addr
    )

    event_log(
        f"[통신 연결] TCP 연결 종료 | peer={addr[0]}:{addr[1]}"
    )


def yolo_tcp_server_thread():
    local_addresses = get_local_ipv4_addresses()

    if local_addresses:
        event_log(
            "[ISAAC TCP] 접속 가능한 로컬 IPv4 후보 | "
            + ", ".join(local_addresses)
        )

    server_socket = None

    try:
        server_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        yolo_tcp_state["server_socket"] = (
            server_socket
        )

        server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_KEEPALIVE,
            1
        )

        bind_success = False
        last_bind_error = None

        # 이전 Script 실행의 소켓이 닫히는 시간을 고려해 재시도
        for attempt in range(1, 6):
            try:
                server_socket.bind(
                    (
                        YOLO_TCP_HOST,
                        YOLO_TCP_PORT
                    )
                )

                bind_success = True
                break

            except OSError as error:
                last_bind_error = error

                event_log(
                    f"[ISAAC TCP] 포트 바인드 재시도 "
                    f"{attempt}/5 | {error}"
                )

                time.sleep(0.5)

        if not bind_success:
            raise OSError(
                f"포트 바인드 실패: {last_bind_error}"
            )

        server_socket.listen(5)
        server_socket.settimeout(0.5)

        event_log(
            f"[ISAAC 서버] 준비 완료 | "
            f"HOST={YOLO_TCP_HOST} | "
            f"PORT={YOLO_TCP_PORT} | "
            "MasterBridge 연결 대기"
        )

        while yolo_tcp_state.get("running", False):
            try:
                conn, addr = server_socket.accept()

            except socket.timeout:
                continue

            except OSError:
                break

            client_thread = threading.Thread(
                target=yolo_tcp_client_thread,
                args=(conn, addr),
                daemon=True
            )

            client_thread.start()

    except Exception as error:
        event_log(
            f"[ISAAC 서버] 시작 실패 | {error}"
        )

    finally:
        try:
            if server_socket is not None:
                server_socket.close()
        except Exception:
            pass

        yolo_tcp_state["server_socket"] = None

        event_log(
            "[ISAAC 서버] 서버 스레드 종료"
        )


def on_yolo_tcp_update(e):
    processed_count = 0

    while not yolo_tcp_queue.empty():
        msg = yolo_tcp_queue.get()

        try:
            handle_yolo_tcp_message(msg)
            processed_count += 1

        except Exception as error:
            event_log(
                f"[ISAAC TCP] 메시지 처리 오류 | {error}"
            )

    if processed_count > 0:
        event_log(
            f"[ISAAC TCP] 메인 스레드 처리 완료 | "
            f"{processed_count}건"
        )


def start_yolo_tcp_server():
    yolo_tcp_state["running"] = True

    server_thread = threading.Thread(
        target=yolo_tcp_server_thread,
        daemon=True,
        name="IsaacYoloTcpServer"
    )

    yolo_tcp_state["server_thread"] = (
        server_thread
    )

    server_thread.start()

    update_sub = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(
            on_yolo_tcp_update,
            name="yolo_tcp_main_thread_update"
        )
    )

    yolo_tcp_state["update_sub"] = update_sub

    event_log(
        "[ISAAC 서버] 시작 요청 완료"
    )

    debug_print("[YOLO TCP 판정 박스 생성 서버 로드 완료]")
    debug_print("YOLO에서 result=1 수신 → 기존 색상의 정상 박스 생성")
    debug_print("INSPECTION_RESULT value=0 수신 → 해당 Depth 박스를 불량으로 저장")
    debug_print("불량 박스는 판정 즉시 빨간색 변경")
    debug_print("rotation_deg.yaw 수신 → quaternion orient로 회전각 적용")
    debug_print("각도 적용 축:", BOX_YAW_AXIS)
    debug_print("불량 박스는 리젝터 위치 통과 시 리젝터 작동")
    debug_print("정상 박스는 그리퍼 위치에서 자동 픽업/트레이 적재")
    debug_print("==================================================")


# ==================================================
# [DIGITAL TWIN 서버 / 상태 스냅샷 / 명령 처리]
# ==================================================

def _twin_safe_float(value, default=None):
    try:
        if value is None:
            return default
        numeric_value = float(value)
        if math.isnan(numeric_value) or math.isinf(numeric_value):
            return default
        return numeric_value
    except Exception:
        return default


def _twin_vec3_to_list(value):
    if value is None:
        return None

    try:
        return [
            float(value[0]),
            float(value[1]),
            float(value[2]),
        ]
    except Exception:
        return None


def _twin_next_sequence():
    global digital_twin_sequence

    with digital_twin_sequence_lock:
        digital_twin_sequence += 1
        return int(digital_twin_sequence)


def _twin_get_joint_target_by_path(joint_path, drive_kind="linear"):
    prim = get_prim(joint_path)

    if prim is None:
        return None

    attr_name = f"drive:{drive_kind}:physics:targetPosition"
    attr = prim.GetAttribute(attr_name)

    if not attr.IsValid():
        return None

    return _twin_safe_float(attr.Get())


def _twin_get_rejector_target():
    prim = get_prim(REJECTOR_JOINT_PATH)

    if prim is None:
        return None

    for attr_name in (
        "drive:angular:physics:targetPosition",
        "drive:linear:physics:targetPosition",
    ):
        attr = prim.GetAttribute(attr_name)
        if attr.IsValid():
            value = _twin_safe_float(attr.Get())
            if value is not None:
                return value

    return None


def _twin_get_recent_master_events():
    try:
        tail_count = int(
            digital_twin_sync_options.get(
                "include_master_history_tail",
                20,
            )
        )
    except Exception:
        tail_count = 20

    tail_count = max(0, min(100, tail_count))

    if tail_count <= 0:
        return []

    with master_state_lock:
        history = list(master_event_history)[-tail_count:]

    result = []

    for item in history:
        if not isinstance(item, dict):
            continue

        result.append({
            "category": item.get("category"),
            "event": item.get("event"),
            "data": item.get("data", {}),
            "raw": item.get("raw"),
            "timestamp": item.get("timestamp"),
        })

    return result


def _twin_collect_boxes():
    if not digital_twin_sync_options.get("include_boxes", True):
        return []

    boxes = []
    valid_paths = get_monitor_box_paths()

    for box_path in valid_paths[-int(TWIN_STATE_MAX_BOXES):]:
        position = get_world_position(box_path)
        box_id = box_path_id_dict.get(box_path)

        boxes.append({
            "id": box_id,
            "path": box_path,
            "position": _twin_vec3_to_list(position),
            "yaw_deg": _twin_safe_float(
                box_yaw_dict.get(box_path, 0.0),
                0.0,
            ),
            "result": box_result_dict.get(box_path),
            "label": box_label_dict.get(box_path),
            "pending_inspection": box_path in pending_inspection_box_paths,
            "stopped_for_pick": box_path in stopped_box_world_position_dict,
            "auto_done": box_path in auto_sequence_done_box_set,
            "rejector_triggered": box_path in rejector_triggered_box_set,
            "tray_loader_triggered": box_path in tray_loader_z_triggered_box_set,
            "attached": bool(
                is_grip_attached
                and attached_box_path == box_path
            ),
        })

    return boxes


def _twin_collect_trays():
    trays = []
    seen = set()

    tray_paths = list(PACK_TRAY_PATH_BY_NUMBER.values())
    tray_paths.extend(tray_loader_discover_tray_paths())

    if TRAY_PATH:
        tray_paths.append(TRAY_PATH)

    for tray_path in tray_paths:
        if not tray_path or tray_path in seen:
            continue

        seen.add(tray_path)

        if get_prim(tray_path) is None:
            continue

        trays.append({
            "path": tray_path,
            "position": _twin_vec3_to_list(
                get_world_position(tray_path)
            ),
            "active": tray_path == TRAY_PATH,
        })

    return trays


def build_digital_twin_state():
    """Isaac Sim 메인 스레드에서 호출하는 JSON 직렬화 가능 상태 스냅샷."""
    timeline = omni.timeline.get_timeline_interface()

    axis_state = {}

    for axis_name in ("X1", "X2", "Y", "Z"):
        axis_state[axis_name.lower()] = {
            "target": _twin_safe_float(
                get_axis_target_position(axis_name)
            ),
            "actual": _twin_safe_float(
                get_axis_actual_position(axis_name)
            ),
            "geometry_raw": _twin_safe_float(
                get_axis_raw_geometry_position(axis_name)
            ),
            "velocity": _twin_safe_float(
                get_axis_actual_velocity(axis_name)
            ),
        }

    master_snapshot = get_master_runtime_state()
    active_tray_position = get_world_position(TRAY_PATH)

    try:
        gripper_logical_angle = get_gripper_joint_logical_angle()
    except Exception:
        gripper_logical_angle = None

    snapshot = {
        "cmd": "TWIN_STATE",
        "version": "v48.0",
        "sequence": _twin_next_sequence(),
        "timestamp": time.time(),
        "timeline": {
            "playing": bool(timeline.is_playing()),
            "system_ready": bool(system_ready),
        },
        "network": {
            "input_host": YOLO_TCP_HOST,
            "input_port": YOLO_TCP_PORT,
            "twin_host": TWIN_TCP_HOST,
            "twin_port": TWIN_TCP_PORT,
            "twin_clients": len(
                digital_twin_state.get("clients", {})
            ),
        },
        "master": master_snapshot,
        "master_event_tail": _twin_get_recent_master_events(),
        "conveyors": {
            "inspection": {
                "running": bool(is_conveyor_running),
                "speed": _twin_safe_float(
                    get_conveyor_x_velocity()
                ),
                "run_speed": float(CONVEYOR_RUN_X_SPEED),
            },
            "packaging": {
                "running": bool(is_pack_conveyor_running),
                "speed": (
                    float(PACK_CONVEYOR_RUN_SPEED)
                    if is_pack_conveyor_running
                    else float(PACK_CONVEYOR_STOP_SPEED)
                ),
                "run_speed": float(PACK_CONVEYOR_RUN_SPEED),
                "start_triggered": bool(pack_start_triggered),
            },
        },
        "robot": {
            "state": robot_sequence.get("state", "IDLE"),
            "place_slot": robot_sequence.get("place_slot"),
            "auto_place_count": int(auto_place_count),
            "busy": bool(is_auto_sequence_running),
            "guard_locked": bool(is_robot_sequence_locked()),
            "guard_owner": robot_sequence_guard.get("owner_box"),
            "target_box": robot_sequence.get("target_box"),
            "target_box_id": box_path_id_dict.get(
                robot_sequence.get("target_box")
            ),
            "motion_active": bool(
                motion_controller.get("active", False)
            ),
            "motion_label": motion_controller.get("label", ""),
            "motion_targets": dict(
                motion_controller.get("commands", {})
            ),
            "axes": axis_state,
        },
        "gripper": {
            "attached": bool(is_grip_attached),
            "attached_box": attached_box_path,
            "attached_box_id": box_path_id_dict.get(attached_box_path),
            "joint_angle_deg": _twin_safe_float(gripper_logical_angle),
            "joint_motion_active": bool(
                gripper_joint_controller.get("active", False)
            ),
            "joint_motion_label": gripper_joint_controller.get(
                "label",
                "",
            ),
        },
        "rejector": {
            "moving": bool(is_rejector_moving),
            "target_deg": _twin_get_rejector_target(),
            "home_deg": float(REJECTOR_HOME_VALUE),
            "push_deg": float(REJECTOR_PUSH_VALUE),
            "trigger_z": float(REJECTOR_TRIGGER_Z),
        },
        "tray_loader": {
            "active": bool(
                integrated_tray_loader_controller.active
            ),
            "state": integrated_tray_loader_controller.state,
            "cycle_started": bool(
                integrated_tray_loader_controller.cycle_started
            ),
            "cycle_completed": bool(
                integrated_tray_loader_controller.cycle_completed
            ),
            "tray_ready": bool(
                integrated_tray_loader_controller.tray_ready
            ),
            "active_tray_path": (
                integrated_tray_loader_controller.active_tray_path
            ),
            "start_reason": (
                integrated_tray_loader_controller.start_reason
            ),
            "upper_target": _twin_get_joint_target_by_path(
                TRAY_LOADER_UPPER_JOINT_PATH,
                "linear",
            ),
            "push_target": _twin_get_joint_target_by_path(
                TRAY_LOADER_PUSH_JOINT_PATH,
                "linear",
            ),
            "trigger_z": float(TRAY_LOADER_TRIGGER_Z),
        },
        "tray_tracking": {
            "active_tray_path": TRAY_PATH,
            "active_tray_position": _twin_vec3_to_list(
                active_tray_position
            ),
            "trigger_active": bool(
                pack_tray_trigger_controller.get("armed", False)
            ),
            "trigger_slot": pack_tray_trigger_controller.get(
                "place_slot"
            ),
            "follow_active": bool(
                tray_follow_controller.get("active", False)
            ),
            "follow_phase": tray_follow_controller.get(
                "phase",
                "IDLE",
            ),
            "trays": _twin_collect_trays(),
        },
        "boxes": _twin_collect_boxes(),
        "sync": {
            "options": dict(digital_twin_sync_options),
            "last_command": digital_twin_state.get("last_command"),
            "last_command_time": digital_twin_state.get(
                "last_command_time"
            ),
            "last_command_result": digital_twin_state.get(
                "last_command_result"
            ),
        },
    }

    return snapshot


def _twin_client_key(addr):
    try:
        return f"{addr[0]}:{int(addr[1])}"
    except Exception:
        return str(addr)


def _twin_register_client(conn, addr):
    client_key = _twin_client_key(addr)

    with digital_twin_clients_lock:
        digital_twin_state["clients"][client_key] = {
            "socket": conn,
            "address": client_key,
            "connected_time": time.time(),
            "last_received_time": None,
            "last_sent_time": None,
            "messages_received": 0,
            "messages_sent": 0,
            "send_lock": threading.Lock(),
        }

    event_log(
        f"[TWIN 연결] 연결됨 | {client_key}"
    )

    return client_key


def _twin_unregister_client(client_key):
    with digital_twin_clients_lock:
        client_info = digital_twin_state["clients"].pop(
            client_key,
            None,
        )

    if client_info is not None:
        event_log(
            f"[TWIN 연결] 연결 종료 | {client_key}"
        )


def _twin_send_json(client_key, payload):
    with digital_twin_clients_lock:
        client_info = digital_twin_state["clients"].get(
            client_key
        )

    if client_info is None:
        return False

    conn = client_info.get("socket")
    send_lock = client_info.get("send_lock")

    if conn is None or send_lock is None:
        return False

    try:
        packet = (
            json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            + "\n"
        ).encode("utf-8")

        with send_lock:
            conn.sendall(packet)

        client_info["last_sent_time"] = time.time()
        client_info["messages_sent"] = int(
            client_info.get("messages_sent", 0)
        ) + 1
        return True

    except Exception:
        _twin_unregister_client(client_key)
        try:
            conn.close()
        except Exception:
            pass
        return False


def broadcast_digital_twin_state(snapshot=None):
    if snapshot is None:
        snapshot = build_digital_twin_state()

    with digital_twin_clients_lock:
        client_keys = list(
            digital_twin_state["clients"].keys()
        )

    sent_count = 0

    for client_key in client_keys:
        if _twin_send_json(client_key, snapshot):
            sent_count += 1

    return sent_count


def _twin_queue_message(client_key, message):
    if not isinstance(message, dict):
        return False

    digital_twin_command_queue.put({
        "client_key": client_key,
        "message": message,
        "received_time": time.time(),
    })

    with digital_twin_clients_lock:
        client_info = digital_twin_state["clients"].get(client_key)
        if client_info is not None:
            client_info["last_received_time"] = time.time()
            client_info["messages_received"] = int(
                client_info.get("messages_received", 0)
            ) + 1

    return True


def _twin_extract_messages(buffer, client_key):
    decoder = json.JSONDecoder()
    remaining = str(buffer)

    while True:
        remaining = remaining.lstrip()

        if not remaining:
            return ""

        try:
            message, end_index = decoder.raw_decode(remaining)
        except json.JSONDecodeError:
            return remaining

        _twin_queue_message(client_key, message)
        remaining = remaining[end_index:]


def digital_twin_client_thread(conn, addr):
    client_key = _twin_register_client(conn, addr)
    buffer = ""

    try:
        conn.settimeout(1.0)

        _twin_send_json(client_key, {
            "cmd": "TWIN_HELLO",
            "version": "v48.0",
            "input_port": YOLO_TCP_PORT,
            "twin_port": TWIN_TCP_PORT,
            "message": "connected",
            "timestamp": time.time(),
        })

        # 메인 스레드에서 안전하게 USD 상태를 읽어 첫 스냅샷을 보낸다.
        _twin_queue_message(client_key, {
            "cmd": "TWIN_SNAPSHOT_REQUEST"
        })

        while digital_twin_state.get("running", False):
            try:
                data = conn.recv(8192)
            except socket.timeout:
                continue

            if not data:
                break

            buffer += data.decode("utf-8", errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    message = json.loads(line)
                except json.JSONDecodeError as error:
                    _twin_send_json(client_key, {
                        "cmd": "TWIN_ERROR",
                        "detail": f"invalid_json: {error}",
                        "timestamp": time.time(),
                    })
                    continue

                if str(message.get("cmd", "")).upper() == "PING":
                    _twin_send_json(client_key, {
                        "cmd": "PONG",
                        "timestamp": time.time(),
                    })
                    continue

                _twin_queue_message(client_key, message)

            buffer = _twin_extract_messages(
                buffer,
                client_key,
            )

    except Exception as error:
        event_log(
            f"[TWIN 연결] 클라이언트 오류 | {client_key} | {error}"
        )

    finally:
        _twin_unregister_client(client_key)
        try:
            conn.close()
        except Exception:
            pass


def digital_twin_server_thread():
    server_socket = None

    try:
        server_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )
        digital_twin_state["server_socket"] = server_socket

        server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )
        server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_KEEPALIVE,
            1,
        )

        bind_success = False
        last_error = None

        for attempt in range(1, 6):
            try:
                server_socket.bind((TWIN_TCP_HOST, TWIN_TCP_PORT))
                bind_success = True
                break
            except OSError as error:
                last_error = error
                event_log(
                    f"[TWIN 서버] 포트 바인드 재시도 {attempt}/5 | {error}"
                )
                time.sleep(0.5)

        if not bind_success:
            raise OSError(f"포트 바인드 실패: {last_error}")

        server_socket.listen(8)
        server_socket.settimeout(0.5)

        event_log(
            f"[TWIN 서버] 준비 완료 | HOST={TWIN_TCP_HOST} | "
            f"PORT={TWIN_TCP_PORT} | 상태주기={TWIN_STATE_SEND_HZ:.1f}Hz"
        )

        while digital_twin_state.get("running", False):
            try:
                conn, addr = server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            thread = threading.Thread(
                target=digital_twin_client_thread,
                args=(conn, addr),
                daemon=True,
                name=f"IsaacTwinClient-{addr[0]}-{addr[1]}",
            )
            thread.start()

    except Exception as error:
        event_log(
            f"[TWIN 서버] 시작 실패 | {error}"
        )

    finally:
        try:
            if server_socket is not None:
                server_socket.close()
        except Exception:
            pass

        digital_twin_state["server_socket"] = None
        event_log("[TWIN 서버] 서버 스레드 종료")


def _twin_is_robot_safe_for_external_move(force=False):
    if force:
        return True, "force"

    if is_auto_sequence_running:
        return False, "자동 적재 시퀀스 진행 중"

    if is_robot_sequence_locked():
        return False, "로봇 시퀀스 하드락 중"

    if motion_controller.get("active", False):
        return False, "기존 3축 모션 진행 중"

    if gripper_joint_controller.get("active", False):
        return False, "그리퍼 회전 진행 중"

    if is_grip_attached:
        return False, "박스 흡착 중"

    return True, "idle"


def _twin_convert_stm_coordinate(data):
    if not isinstance(data, dict):
        return None

    try:
        x_value = float(data["x"])
        y_value = float(data["y"])
        z_value = float(data["z"])
    except (KeyError, TypeError, ValueError):
        return None

    twin_x = (
        x_value * STM_X_TO_TWIN_X_SCALE * STM_X_TO_TWIN_X_SIGN
        + STM_X_TO_TWIN_X_OFFSET
    )
    twin_y = (
        y_value * STM_Y_TO_TWIN_Y_SCALE * STM_Y_TO_TWIN_Y_SIGN
        + STM_Y_TO_TWIN_Y_OFFSET
    )
    twin_z = (
        z_value * STM_Z_TO_TWIN_Z_SCALE * STM_Z_TO_TWIN_Z_SIGN
        + STM_Z_TO_TWIN_Z_OFFSET
    )

    return {
        "X1": clamp_axis_target("X1", twin_x),
        "X2": clamp_axis_target("X2", twin_x),
        "Y": clamp_axis_target("Y", twin_y),
        "Z": clamp_axis_target("Z", twin_z),
    }


def _twin_apply_conveyor_state(data):
    if not isinstance(data, dict):
        return False, "data 형식 오류"

    changed = []

    inspection_running = data.get("inspection_running")
    packaging_running = data.get("packaging_running")

    if inspection_running is not None:
        if bool(inspection_running):
            start_conveyor()
        else:
            stop_conveyor()
        changed.append(
            f"inspection={bool(inspection_running)}"
        )

    if packaging_running is not None:
        if bool(packaging_running):
            start_pack_conveyor("Master/Twin 상태 동기화")
        else:
            stop_pack_conveyor("Master/Twin 상태 동기화")
        changed.append(
            f"packaging={bool(packaging_running)}"
        )

    if not changed:
        return False, "적용할 컨베이어 상태 없음"

    return True, ", ".join(changed)


def apply_digital_twin_action(action, data=None, force=False):
    action = str(action or "").strip().upper()
    data = data if isinstance(data, dict) else {}
    force = bool(force or data.get("force", False))

    if action in ("SNAPSHOT", "GET_STATE", "STATE"):
        return True, "snapshot"

    if action in ("SET_SYNC_OPTIONS", "SYNC_OPTIONS"):
        allowed_keys = set(digital_twin_sync_options.keys())
        changed = {}

        for key, value in data.items():
            if key not in allowed_keys:
                continue

            if key == "stm_coordinate_mode":
                value = str(value).upper()
                if value not in ("OFF", "IDLE_ONLY", "ALWAYS"):
                    continue

            if key == "include_master_history_tail":
                try:
                    value = max(0, min(100, int(value)))
                except Exception:
                    continue

            digital_twin_sync_options[key] = value
            changed[key] = value

        return bool(changed), {
            "changed": changed,
            "options": dict(digital_twin_sync_options),
        }

    if action in ("SET_CONVEYOR", "SET_CONVEYORS", "CONVEYOR_STATE"):
        return _twin_apply_conveyor_state(data)

    if action in ("START_INSPECTION_CONVEYOR", "INSPECTION_START"):
        start_conveyor()
        return True, f"inspection speed={CONVEYOR_RUN_X_SPEED}"

    if action in ("STOP_INSPECTION_CONVEYOR", "INSPECTION_STOP"):
        stop_conveyor()
        return True, "inspection stopped"

    if action in ("START_PACK_CONVEYOR", "PACK_START"):
        return bool(start_pack_conveyor("TWIN_COMMAND")), "pack start"

    if action in ("STOP_PACK_CONVEYOR", "PACK_STOP"):
        return bool(stop_pack_conveyor("TWIN_COMMAND")), "pack stop"

    if action in ("START_REJECTOR", "REJECTOR_START", "REJECTOR_TRIGGER"):
        if is_rejector_moving:
            return False, "rejector already moving"
        asyncio.ensure_future(run_rejector())
        return True, "rejector cycle scheduled"

    if action in ("REJECTOR_HOME",):
        target_attr = get_rejector_target_attr()
        if target_attr is None:
            return False, "rejector target attribute missing"
        target_attr.Set(float(REJECTOR_HOME_VALUE))
        return True, f"rejector={REJECTOR_HOME_VALUE}"

    if action in ("START_TRAY_LOADER", "TRAY_LOADER_START", "TRAY_SUPPLY_START"):
        success = request_tray_loader_start(
            reason=str(data.get("reason", "TWIN_COMMAND")),
            source_box_path=data.get("source_box_path"),
        )
        return bool(success), "tray loader start requested"

    if action in ("STOP_TRAY_LOADER", "TRAY_LOADER_STOP"):
        integrated_tray_loader_controller.request_stop()
        return True, "tray loader stop/home requested"

    if action in ("MOVE_3AXIS", "ROBOT_TARGET", "STM_COORDINATE"):
        safe, safe_reason = _twin_is_robot_safe_for_external_move(force=force)

        if not safe:
            return False, safe_reason

        commands = {}

        if "x" in data:
            commands["X1"] = data["x"]
            commands["X2"] = data["x"]

        key_map = {
            "x1": "X1",
            "x2": "X2",
            "y": "Y",
            "z": "Z",
        }

        for source_key, axis_name in key_map.items():
            if source_key in data:
                commands[axis_name] = data[source_key]

        normalized = {}

        for axis_name, value in commands.items():
            try:
                normalized[axis_name] = clamp_axis_target(
                    axis_name,
                    float(value),
                )
            except Exception:
                return False, f"invalid {axis_name} value: {value}"

        if not normalized:
            return False, "3축 목표값 없음"

        synchronized = bool(data.get("synchronized", True))
        duration_frames = data.get("duration_frames")

        try:
            duration_frames = (
                int(duration_frames)
                if duration_frames is not None
                else None
            )
        except Exception:
            duration_frames = None

        success = start_scurve_motion(
            str(data.get("label", "Digital Twin 3축 동기화")),
            normalized,
            synchronized=synchronized,
            duration_frames=duration_frames,
        )

        return bool(success), {
            "commands": normalized,
            "synchronized": synchronized,
            "safe_reason": safe_reason,
        }

    if action in ("GRIPPER_CLOSE", "GRIP_CLOSE"):
        success = close_grip_no_jump(
            target_box=data.get("box_path"),
            force=force,
        )
        return bool(success), "gripper close"

    if action in ("GRIPPER_OPEN", "GRIP_OPEN"):
        released = open_grip_no_jump()
        return True, {"released_box": released}

    if action in ("TIMELINE_PLAY", "PLAY"):
        omni.timeline.get_timeline_interface().play()
        return True, "timeline play"

    if action in ("TIMELINE_STOP", "STOP"):
        omni.timeline.get_timeline_interface().stop()
        return True, "timeline stop"

    return False, f"unknown action: {action}"


def apply_master_event_to_digital_twin(msg):
    """기존 MASTER_EVENT를 상태 저장뿐 아니라 Twin 동작에도 선택적으로 반영."""
    if not isinstance(msg, dict):
        return False

    category = str(msg.get("category", "")).strip().upper()
    event_name = str(msg.get("event", "")).strip().upper()
    data = msg.get("data", {})

    if not isinstance(data, dict):
        data = {"value": data}

    if (
        category == "CONVEYOR"
        and event_name == "STATE"
        and digital_twin_sync_options.get(
            "master_conveyor_to_twin",
            True,
        )
    ):
        success, detail = _twin_apply_conveyor_state(data)
        if success:
            event_log(
                f"[TWIN 동기화] Master 컨베이어 반영 | {detail}"
            )
        return success

    if category in ("MCU", "STM", "STM32") and event_name == "TX_COORDINATE":
        coordinate_mode = str(
            digital_twin_sync_options.get(
                "stm_coordinate_mode",
                "IDLE_ONLY",
            )
        ).upper()

        if coordinate_mode == "OFF":
            return False

        force = coordinate_mode == "ALWAYS"
        safe, reason = _twin_is_robot_safe_for_external_move(force=force)

        if not safe:
            event_log(
                f"[TWIN 동기화] STM 좌표 저장만 수행 | "
                f"Isaac 3축 반영 보류={reason}"
            )
            return False

        commands = _twin_convert_stm_coordinate(data)

        if commands is None:
            return False

        success = start_scurve_motion(
            "STM 좌표 -> Digital Twin 3축",
            commands,
            synchronized=True,
        )

        if success:
            event_log(
                f"[TWIN 동기화] STM 좌표 3축 반영 | {commands}"
            )

        return bool(success)

    if not digital_twin_sync_options.get(
        "explicit_process_events",
        True,
    ):
        return False

    # Master에서 category=TWIN 또는 명시적인 STM 이벤트를 보내면 바로 동작한다.
    explicit_action_map = {
        "REJECTOR_START": "START_REJECTOR",
        "REJECTOR_TRIGGER": "START_REJECTOR",
        "TRAY_LOADER_START": "START_TRAY_LOADER",
        "TRAY_SUPPLY_START": "START_TRAY_LOADER",
        "TRAY_LOADER_STOP": "STOP_TRAY_LOADER",
        "ROBOT_TARGET": "MOVE_3AXIS",
        "TWIN_COORDINATE": "MOVE_3AXIS",
        "CONVEYOR_STATE": "SET_CONVEYORS",
    }

    if category == "TWIN":
        action = explicit_action_map.get(event_name, event_name)
        success, detail = apply_digital_twin_action(
            action,
            data,
            force=bool(data.get("force", False)),
        )
        event_log(
            f"[TWIN 동기화] MASTER_EVENT {event_name} | "
            f"성공={success} | {detail}"
        )
        return bool(success)

    if category in ("MCU", "STM", "STM32") and event_name in explicit_action_map:
        action = explicit_action_map[event_name]
        success, detail = apply_digital_twin_action(
            action,
            data,
            force=bool(data.get("force", False)),
        )
        event_log(
            f"[TWIN 동기화] STM 이벤트 {event_name} | "
            f"성공={success} | {detail}"
        )
        return bool(success)

    return False


def process_digital_twin_message(client_key, message):
    if not isinstance(message, dict):
        return False

    cmd = str(message.get("cmd", "")).strip().upper()

    if cmd in ("TWIN_SNAPSHOT_REQUEST", "GET_TWIN_STATE"):
        snapshot = build_digital_twin_state()
        _twin_send_json(client_key, snapshot)
        return True

    if cmd == "TWIN_COMMAND":
        action = message.get("action", "")
        data = message.get("data", {})
        force = bool(message.get("force", False))

        success, detail = apply_digital_twin_action(
            action,
            data,
            force=force,
        )

        result = {
            "cmd": "TWIN_COMMAND_RESULT",
            "request_id": message.get("request_id"),
            "action": action,
            "success": bool(success),
            "detail": detail,
            "timestamp": time.time(),
        }

        digital_twin_state["last_command"] = {
            "client": client_key,
            "request_id": message.get("request_id"),
            "action": action,
            "data": data,
            "force": force,
        }
        digital_twin_state["last_command_time"] = time.time()
        digital_twin_state["last_command_result"] = result

        _twin_send_json(client_key, result)

        event_log(
            f"[TWIN 명령] action={action} | "
            f"성공={success} | {detail}"
        )
        return bool(success)

    # 트윈 포트에서도 기존 입력 프로토콜을 그대로 받을 수 있다.
    if cmd in (
        "MASTER_EVENT",
        "CREATE_BOX",
        "DECISION",
        "INSPECTION_RESULT",
    ):
        handle_yolo_tcp_message(message)
        _twin_send_json(client_key, {
            "cmd": "TWIN_INPUT_RESULT",
            "input_cmd": cmd,
            "success": True,
            "timestamp": time.time(),
        })
        return True

    _twin_send_json(client_key, {
        "cmd": "TWIN_ERROR",
        "detail": f"unknown_cmd:{cmd}",
        "timestamp": time.time(),
    })
    return False


def on_digital_twin_update(event):
    processed = 0

    while (
        processed < int(TWIN_COMMANDS_PER_UPDATE)
        and not digital_twin_command_queue.empty()
    ):
        item = digital_twin_command_queue.get()
        processed += 1

        try:
            process_digital_twin_message(
                item.get("client_key"),
                item.get("message"),
            )
        except Exception as error:
            client_key = item.get("client_key")
            event_log(
                f"[TWIN 명령] 처리 오류 | {error}"
            )
            _twin_send_json(client_key, {
                "cmd": "TWIN_ERROR",
                "detail": str(error),
                "timestamp": time.time(),
            })

    if not digital_twin_sync_options.get("broadcast_enabled", True):
        return

    now = time.perf_counter()
    period = 1.0 / max(0.1, float(TWIN_STATE_SEND_HZ))
    last_time = float(
        digital_twin_state.get("last_broadcast_time", 0.0)
    )

    if now - last_time < period:
        return

    digital_twin_state["last_broadcast_time"] = now

    with digital_twin_clients_lock:
        has_clients = bool(digital_twin_state["clients"])

    if not has_clients:
        return

    snapshot = build_digital_twin_state()
    broadcast_digital_twin_state(snapshot)


def start_digital_twin_server():
    digital_twin_state["running"] = True

    server_thread = threading.Thread(
        target=digital_twin_server_thread,
        daemon=True,
        name="IsaacDigitalTwinTcpServer",
    )
    digital_twin_state["server_thread"] = server_thread
    server_thread.start()

    update_sub = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(
            on_digital_twin_update,
            name="assembly714_digital_twin_update_v48",
        )
    )
    digital_twin_state["update_sub"] = update_sub

    event_log(
        f"[TWIN 서버] 시작 요청 완료 | PORT={TWIN_TCP_PORT}"
    )


# Script Editor / 다른 Extension에서 직접 조회할 수 있는 공개 함수
builtins.get_assembly714_twin_state = build_digital_twin_state
builtins.broadcast_assembly714_twin_state = broadcast_digital_twin_state
builtins.apply_assembly714_twin_action = apply_digital_twin_action


# ==================================================
# [17. 상태 출력]
# ==================================================

def print_status():
    event_log(
        "[상태] "
        f"ready={system_ready} | "
        f"sequence={robot_sequence.get('state', 'IDLE')} | "
        f"motion={motion_controller.get('label', '')} | "
        f"grip={is_grip_attached}"
    )

    actual_parts = []

    for axis_name in (
        "X1",
        "X2",
        "Y",
        "Z"
    ):
        actual_value = get_axis_actual_position(
            axis_name
        )

        target_value = get_axis_target_position(
            axis_name
        )

        if actual_value is None:
            actual_text = "None"
        else:
            actual_text = f"{actual_value:.2f}"

        if target_value is None:
            target_text = "None"
        else:
            target_text = f"{target_value:.2f}"

        actual_parts.append(
            f"{axis_name}:actual={actual_text}/target={target_text}"
        )

    event_log(
        "[3축 상태] "
        + " | ".join(actual_parts)
    )

    conveyor_speed = get_conveyor_x_velocity()

    event_log(
        "[컨베이어 상태] "
        f"검사={conveyor_speed} | "
        f"정지기준 Z={STOP_TARGET_Z:.2f}"
    )

    target_box = get_current_target_box()

    if target_box is not None:
        box_pos = get_world_position(
            target_box
        )

        event_log(
            f"[박스 상태] path={target_box} | "
            f"position={box_pos} | "
            f"result={box_result_dict.get(target_box, None)}"
        )


def get_pending_stopped_pick_box():
    """
    검사 컨베이어 정지 좌표에서 멈췄지만 아직 자동 픽업을
    시작하지 않은 가장 먼저 기록된 정상/미판정 박스를 반환한다.
    """
    for box_path in list(
        stopped_box_world_position_dict.keys()
    ):
        if get_prim(box_path) is None:
            stopped_box_world_position_dict.pop(
                box_path,
                None
            )
            continue

        if box_path in auto_sequence_done_box_set:
            continue

        if box_result_dict.get(
            box_path,
            None
        ) == 0:
            continue

        return box_path

    return None


def try_start_pending_pick_sequence():
    """
    컨베이어는 좌표에서 먼저 정지시키고,
    로봇 초기 위치 이동이 끝난 뒤 실제 픽업을 시작한다.
    """
    if not system_ready:
        return False

    if is_auto_sequence_running:
        return False

    if is_robot_sequence_locked():
        return False

    if is_grip_attached:
        return False

    if robot_sequence.get(
        "state",
        "IDLE"
    ) != "IDLE":
        return False

    if post_pick_axis_pipeline.get(
        "active",
        False
    ):
        return False

    if pack_tray_trigger_controller.get(
        "active",
        False
    ):
        return False

    if tray_follow_controller.get(
        "active",
        False
    ):
        return False

    if motion_controller.get(
        "active",
        False
    ):
        return False

    pending_box = (
        get_pending_stopped_pick_box()
    )

    if pending_box is None:
        return False

    stopped_position = (
        stopped_box_world_position_dict.get(
            pending_box,
            None
        )
    )

    if stopped_position is None:
        stopped_position = get_world_position(
            pending_box
        )

    if stopped_position is None:
        return False

    event_log(
        f"[픽업 대기 해제] "
        f"ID={box_path_id_dict.get(pending_box, None)} | "
        "로봇 준비 완료, 자동 픽업 시작"
    )

    return start_auto_pick_sequence_for_box(
        pending_box,
        stopped_position
    )



# ==================================================
# [통합 트레이로더]
# - Timeline PLAY만으로는 시작하지 않는다.
# - 첫 공급은 최초 생성 박스가 Z=500에 도착할 때 시작한다.
# - 이후에는 트레이당 정상 박스 2개 적재 완료 시 다음 Tray를 공급한다.
# - Push=30 복귀 후 Upper는 다음 미공급 Tray 위치로 미리 이동한다.
# - 공급 완료 트레이를 TRAY_PATH로 자동 지정한다.
# ==================================================

TRAY_LOADER_ROOT_PATH = "/assembly714/Tray_loader"
TRAY_LOADER_UPPER_JOINT_PATH = (
    "/assembly714/Tray_loader/PrismaticJoint_Upper"
)
TRAY_LOADER_PUSH_JOINT_PATH = (
    "/assembly714/Tray_loader/PrismaticJoint_Push"
)

# 순차 트레이 공급 최종 설정
# Tray2=-135, Tray3=-167, Tray4=-200, Tray5=-220
# Push는 모든 Tray에서 -190 전진 후 30으로만 복귀한다.
# Upper는 공급 완료 후 현재 Tray 위치를 유지하며 0으로 복귀하지 않는다.
TRAY_LOADER_UPPER_TRAY2_POSITION = -135.0
TRAY_LOADER_UPPER_TRAY3_POSITION = -167.0
TRAY_LOADER_UPPER_TRAY4_POSITION = -200.0
TRAY_LOADER_UPPER_TRAY5_POSITION = -220.0
TRAY_LOADER_UPPER_FALLBACK_POSITION = -220.0
TRAY_LOADER_PUSH_SUPPLY_POSITION = -190.0
TRAY_LOADER_PUSH_HOME_POSITION = 30.0

TRAY_LOADER_UPPER_SUPPLY_DURATION_SEC = 6.0
TRAY_LOADER_PUSH_SUPPLY_DURATION_SEC = 4.0
TRAY_LOADER_PUSH_HOME_DURATION_SEC = 3.5

# Timeline 시작 시 Upper가 기존 Drive 목표(-167)로 순간 이동하지 않도록
# 먼저 목표를 0으로 맞춘 뒤 0 -> -167을 S-Curve로 천천히 이동한다.
TRAY_LOADER_STARTUP_UPPER_ENABLED = True
TRAY_LOADER_STARTUP_UPPER_HOME_POSITION = 0.0
TRAY_LOADER_STARTUP_UPPER_TARGET_POSITION = -135.0
TRAY_LOADER_STARTUP_UPPER_DURATION_SEC = 6.0
TRAY_LOADER_STARTUP_UPPER_SETTLE_SEC = 0.6

TRAY_LOADER_UPPER_SETTLE_SEC = 0.6
TRAY_LOADER_PUSH_SETTLE_SEC = 0.5
TRAY_LOADER_PUSH_HOME_SETTLE_SEC = 0.4
TRAY_LOADER_FINAL_HOLD_SEC = 0.8

# 한 트레이에 적재하는 제품 수.
# 2개 적재가 끝날 때마다 다음 미공급 트레이의 Push 공급을 자동 요청한다.
TRAY_LOADER_BOXES_PER_TRAY = 2
TRAY_LOADER_AUTO_START_AFTER_FULL_TRAY = True

# Push가 30으로 복귀한 뒤 다음 미공급 Tray의 Upper 위치로 미리 이동한다.
# 다음 공급 요청이 들어오면 Upper 이동 없이 바로 정착 후 Push를 시작할 수 있다.
TRAY_LOADER_PREPOSITION_NEXT_UPPER = True
TRAY_LOADER_NEXT_UPPER_MOVE_DURATION_SEC = 6.0
TRAY_LOADER_NEXT_UPPER_SETTLE_SEC = 0.6

TRAY_LOADER_SDF_RESOLUTION = 128
TRAY_LOADER_MIN_DISPLACEMENT = 0.5
TRAY_LOADER_ONE_SHOT_PER_PLAY = False

# ==================================================
# [v51.4 첫 번째 생성 박스 전용 트레이로더 Z 감지선]
# ==================================================
# Depth CREATE_BOX 또는 B키로 처음 생성된 박스가 World Z=500에
# 도착하거나 한 프레임 사이에 통과할 때만 첫 트레이 공급을 시작한다.
#
# 시작 순서:
#   최초 생성 박스 Z=500 도착
#   -> Upper가 첫 공급 Tray 위치로 이동
#   -> Upper 정착
#   -> Push -190 전진
#   -> Push 30 복귀
#
# 정상/불량 판정과 관계없이 "최초 생성 박스" 자체를 기준으로 한다.
TRAY_LOADER_TRIGGER_Z = 650.0
TRAY_LOADER_TRIGGER_TOLERANCE_Z = 0.5
TRAY_LOADER_START_REGARDLESS_OF_RESULT = True
TRAY_LOADER_FIRST_DEPTH_BOX_ONLY = True

TRAY_LOADER_TRAY_NAME_PATTERN = re.compile(r"^Tray\d+$")
TRAY_LOADER_TRAY_PATH_OVERRIDE = ""

# 트레이로더 Z 감지선 중복 시작 방지
tray_loader_z_triggered_box_set = set()

# 불량 리젝터 완료 후 중복 호출 방지용 기존 Set
tray_loader_rejector_section_box_set = set()


def tray_loader_log(message):
    event_log(f"[트레이 이동] {message}")


def is_inside_tray_loader_trigger(current_pos):
    """
    현재 박스가 트레이로더 전용 Z 감지선에 도착했는지 확인한다.

    검사 컨베이어는 Z 감소 방향으로 이동하므로:
        current_z <= TRAY_LOADER_TRIGGER_Z + tolerance
    이면 이미 감지선을 도착/통과한 것으로 처리한다.
    """
    if current_pos is None:
        return False

    current_z = float(current_pos[2])

    return (
        current_z
        <= (
            float(TRAY_LOADER_TRIGGER_Z)
            + float(TRAY_LOADER_TRIGGER_TOLERANCE_Z)
        )
    )


def crossed_tray_loader_trigger_z(
    prev_pos,
    current_pos,
):
    """
    프레임 간 이동량이 커서 Z=500.0을 건너뛴 경우도 감지한다.
    이동 방향이 반대여도 두 좌표 사이에 감지선이 있으면 True다.
    """
    if prev_pos is None or current_pos is None:
        return False

    prev_z = float(prev_pos[2])
    current_z = float(current_pos[2])
    trigger_z = float(TRAY_LOADER_TRIGGER_Z)

    return (
        min(prev_z, current_z)
        <= trigger_z
        <= max(prev_z, current_z)
    )


def has_reached_tray_loader_trigger(
    prev_pos,
    current_pos,
):
    """
    현재 위치가 감지선 이하이거나 프레임 사이에 감지선을 통과하면 True.
    """
    if current_pos is None:
        return False

    if is_inside_tray_loader_trigger(
        current_pos
    ):
        return True

    return crossed_tray_loader_trigger_z(
        prev_pos,
        current_pos,
    )


def tray_loader_scurve(t):
    t = max(0.0, min(1.0, float(t)))
    return 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5


def tray_loader_get_linear_drive(joint_path):
    stage = get_stage()

    if stage is None:
        raise RuntimeError("현재 열린 USD Stage가 없음")

    joint_prim = stage.GetPrimAtPath(joint_path)

    if not joint_prim or not joint_prim.IsValid():
        raise RuntimeError(f"Joint Prim을 찾을 수 없음: {joint_path}")

    if not joint_prim.IsA(UsdPhysics.PrismaticJoint):
        raise RuntimeError(f"PrismaticJoint가 아님: {joint_path}")

    drive_api = UsdPhysics.DriveAPI.Apply(
        joint_prim,
        UsdPhysics.Tokens.linear,
    )

    target_position_attr = drive_api.GetTargetPositionAttr()

    if not target_position_attr:
        target_position_attr = drive_api.CreateTargetPositionAttr(0.0)

    target_velocity_attr = drive_api.GetTargetVelocityAttr()

    if not target_velocity_attr:
        target_velocity_attr = drive_api.CreateTargetVelocityAttr(0.0)

    target_velocity_attr.Set(0.0)

    return drive_api


def tray_loader_get_drive_target(drive_api, fallback=0.0):
    value = drive_api.GetTargetPositionAttr().Get()

    if value is None:
        return float(fallback)

    return float(value)


def tray_loader_set_drive_target(drive_api, value):
    drive_api.GetTargetPositionAttr().Set(float(value))

    velocity_attr = drive_api.GetTargetVelocityAttr()

    if velocity_attr:
        try:
            velocity_attr.Set(0.0)
        except Exception:
            pass


def tray_loader_tray_number(tray_path):
    match = re.search(r"(\d+)$", str(tray_path))
    return int(match.group(1)) if match else 999999


def tray_loader_upper_target_for_path(tray_path):
    tray_number = tray_loader_tray_number(tray_path)

    target_map = {
        2: TRAY_LOADER_UPPER_TRAY2_POSITION,
        3: TRAY_LOADER_UPPER_TRAY3_POSITION,
        4: TRAY_LOADER_UPPER_TRAY4_POSITION,
        5: TRAY_LOADER_UPPER_TRAY5_POSITION,
    }

    return float(
        target_map.get(
            tray_number,
            TRAY_LOADER_UPPER_FALLBACK_POSITION,
        )
    )


def tray_loader_discover_tray_paths():
    stage = get_stage()

    if stage is None:
        return []

    root_prim = stage.GetPrimAtPath(TRAY_LOADER_ROOT_PATH)

    if not root_prim or not root_prim.IsValid():
        return []

    tray_paths = []

    for child in root_prim.GetChildren():
        if TRAY_LOADER_TRAY_NAME_PATTERN.match(child.GetName()):
            tray_paths.append(str(child.GetPath()))

    tray_paths.sort(key=tray_loader_tray_number)
    return tray_paths


def tray_loader_collect_positions(tray_paths):
    return {
        path: copy_world_position(get_world_position(path))
        for path in tray_paths
    }


def tray_loader_distance(position_a, position_b):
    if position_a is None or position_b is None:
        return 0.0

    delta = Gf.Vec3d(
        float(position_a[0]) - float(position_b[0]),
        float(position_a[1]) - float(position_b[1]),
        float(position_a[2]) - float(position_b[2]),
    )
    return float(delta.GetLength())


def tray_loader_find_moved_tray(tray_paths, positions_before_push):
    if TRAY_LOADER_TRAY_PATH_OVERRIDE:
        if get_prim(TRAY_LOADER_TRAY_PATH_OVERRIDE) is None:
            raise RuntimeError(
                "TRAY_LOADER_TRAY_PATH_OVERRIDE 경로 없음: "
                f"{TRAY_LOADER_TRAY_PATH_OVERRIDE}"
            )

        return TRAY_LOADER_TRAY_PATH_OVERRIDE, 0.0

    best_path = None
    best_displacement = -1.0

    for tray_path in tray_paths:
        before_position = positions_before_push.get(tray_path)
        after_position = get_world_position(tray_path)
        displacement = tray_loader_distance(before_position, after_position)

        if displacement > best_displacement:
            best_path = tray_path
            best_displacement = displacement

    return best_path, best_displacement


def tray_loader_switch_collider_to_sdf(tray_root_path):
    stage = get_stage()
    tray_root_prim = get_prim(tray_root_path)

    if stage is None or tray_root_prim is None:
        raise RuntimeError(f"트레이 Prim을 찾을 수 없음: {tray_root_path}")

    changed_mesh_paths = []

    for prim in Usd.PrimRange(tray_root_prim):
        if not prim.IsA(UsdGeom.Mesh):
            continue

        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue

        collision_api = UsdPhysics.CollisionAPI.Apply(prim)
        collision_enabled_attr = collision_api.GetCollisionEnabledAttr()

        if not collision_enabled_attr:
            collision_enabled_attr = collision_api.CreateCollisionEnabledAttr(True)

        current_enabled = collision_enabled_attr.Get()

        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        approximation_attr = mesh_collision_api.GetApproximationAttr()

        if not approximation_attr:
            approximation_attr = mesh_collision_api.CreateApproximationAttr(
                PhysxSchema.Tokens.sdf
            )

        approximation_attr.Set(PhysxSchema.Tokens.sdf)

        sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
        sdf_resolution_attr = sdf_api.GetSdfResolutionAttr()

        if not sdf_resolution_attr:
            sdf_resolution_attr = sdf_api.CreateSdfResolutionAttr(
                int(TRAY_LOADER_SDF_RESOLUTION)
            )

        sdf_resolution_attr.Set(int(TRAY_LOADER_SDF_RESOLUTION))

        if current_enabled is not None:
            collision_enabled_attr.Set(bool(current_enabled))

        changed_mesh_paths.append(str(prim.GetPath()))

    if not changed_mesh_paths:
        raise RuntimeError(
            "Collider가 적용된 Mesh를 찾지 못함: "
            f"{tray_root_path}"
        )

    return changed_mesh_paths


def tray_loader_restore_all_colliders_to_convex_hull(tray_paths=None):
    if tray_paths is None:
        tray_paths = tray_loader_discover_tray_paths()

    convex_hull_token = getattr(
        UsdPhysics.Tokens,
        "convexHull",
        "convexHull",
    )
    restored_paths = []

    for tray_path in tray_paths:
        tray_prim = get_prim(tray_path)

        if tray_prim is None:
            continue

        for prim in Usd.PrimRange(tray_prim):
            if not prim.IsA(UsdGeom.Mesh):
                continue

            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue

            mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
            approximation_attr = mesh_collision_api.GetApproximationAttr()

            if not approximation_attr:
                approximation_attr = mesh_collision_api.CreateApproximationAttr(
                    convex_hull_token
                )

            approximation_attr.Set(convex_hull_token)

            try:
                if prim.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    prim.RemoveAPI(PhysxSchema.PhysxSDFMeshCollisionAPI)
            except Exception:
                pass

            restored_paths.append(str(prim.GetPath()))

    return restored_paths


def set_active_pack_tray(tray_path):
    """트레이로더가 공급한 실제 트레이를 로봇 추종 대상으로 변경한다."""
    global TRAY_PATH
    global last_tray_world_position
    global pack_tray_reference_world_position

    if not tray_path or get_prim(tray_path) is None:
        return False

    TRAY_PATH = str(tray_path)
    tray_position = get_world_position(TRAY_PATH)

    if tray_position is not None:
        last_tray_world_position = copy_world_position(tray_position)
        pack_tray_reference_world_position = copy_world_position(tray_position)

    # wake_tray_body는 파일 후반에 정의되지만 실제 호출 시점에는 준비되어 있다.
    try:
        wake_tray_body()
    except Exception:
        pass

    tray_loader_log(
        f"활성 적재 트레이 변경 | Tray={TRAY_PATH}"
    )
    return True


class IntegratedTrayLoaderController:
    IDLE = "IDLE"
    STARTUP_UPPER_MOVE = "STARTUP_UPPER_MOVE"
    STARTUP_UPPER_SETTLE = "STARTUP_UPPER_SETTLE"
    UPPER_MOVE = "UPPER_MOVE"
    UPPER_SETTLE = "UPPER_SETTLE"
    PUSH_MOVE = "PUSH_MOVE"
    PUSH_SETTLE = "PUSH_SETTLE"
    SWITCH_SDF = "SWITCH_SDF"
    FINAL_HOLD = "FINAL_HOLD"
    PUSH_HOME_MOVE = "PUSH_HOME_MOVE"
    PUSH_HOME_SETTLE = "PUSH_HOME_SETTLE"
    NEXT_UPPER_MOVE = "NEXT_UPPER_MOVE"
    NEXT_UPPER_SETTLE = "NEXT_UPPER_SETTLE"
    DONE = "DONE"
    ERROR = "ERROR"

    def __init__(self):
        self.state = self.IDLE
        self.elapsed = 0.0
        self.motion_drive = None
        self.motion_start = 0.0
        self.motion_target = 0.0
        self.motion_duration = 1.0
        self.upper_drive = None
        self.push_drive = None
        self.tray_paths = []
        self.current_candidate_paths = []
        self.positions_before_push = {}
        self.supplied_tray_paths = set()
        self.active = False
        self.cycle_started = False
        self.cycle_completed = False
        self.cycle_count = 0
        self.tray_ready = False
        self.active_tray_path = None
        self.target_tray_path = None
        self.target_upper_position = None
        self.next_upper_tray_path = None
        self.next_upper_position = None
        self.start_reason = ""
        self.stop_requested = False
        self.restore_convex_after_home = False
        self.sdf_mesh_paths = []

    def ensure_resources(self):
        self.upper_drive = tray_loader_get_linear_drive(
            TRAY_LOADER_UPPER_JOINT_PATH
        )
        self.push_drive = tray_loader_get_linear_drive(
            TRAY_LOADER_PUSH_JOINT_PATH
        )
        self.tray_paths = tray_loader_discover_tray_paths()

        if not self.tray_paths:
            raise RuntimeError(
                f"Tray2 이상의 Tray를 찾지 못함: {TRAY_LOADER_ROOT_PATH}"
            )

    def prepare_startup_upper_home(self):
        """
        Timeline이 멈춘 상태에서 다음 PLAY를 준비한다.

        Upper의 Drive 목표를 0으로 미리 설정하여, PLAY 순간 기존 -167
        목표값으로 강하게 당겨지는 현상을 방지한다.
        """
        if not TRAY_LOADER_STARTUP_UPPER_ENABLED:
            return False

        try:
            self.ensure_resources()
            tray_loader_set_drive_target(
                self.upper_drive,
                TRAY_LOADER_STARTUP_UPPER_HOME_POSITION,
            )
            return True
        except Exception:
            return False

    def start_startup_upper_motion(self):
        """
        Timeline PLAY 시 Upper를 0에서 -167까지 S-Curve로 이동한다.

        이 동작은 트레이 공급 사이클과 별개이며, 완료 후 IDLE로 돌아가
        첫 박스 Z 트리거 또는 이후 2개 적재 완료 요청을 기다린다.
        """
        if not TRAY_LOADER_STARTUP_UPPER_ENABLED:
            return False

        if self.active:
            return False

        if not omni.timeline.get_timeline_interface().is_playing():
            return False

        try:
            self.ensure_resources()

            # PLAY 직후에도 시작 Drive 목표를 반드시 0으로 고정한다.
            tray_loader_set_drive_target(
                self.upper_drive,
                TRAY_LOADER_STARTUP_UPPER_HOME_POSITION,
            )

            self.active = True
            self.cycle_started = False
            self.cycle_completed = False
            self.tray_ready = False
            self.active_tray_path = None
            self.target_tray_path = None
            self.target_upper_position = None
            self.next_upper_tray_path = None
            self.next_upper_position = None
            self.start_reason = "Timeline PLAY Upper 초기 이동"
            self.stop_requested = False
            self.restore_convex_after_home = False
            self.positions_before_push = {}
            self.current_candidate_paths = []

            self.begin_motion(
                self.upper_drive,
                TRAY_LOADER_STARTUP_UPPER_TARGET_POSITION,
                TRAY_LOADER_STARTUP_UPPER_DURATION_SEC,
                self.STARTUP_UPPER_MOVE,
                "Upper 시작 위치 이동",
            )
            return True

        except Exception:
            self.active = False
            self.state = self.ERROR
            return False

    def get_remaining_trays(self):
        self.tray_paths = tray_loader_discover_tray_paths()
        return [
            tray_path
            for tray_path in self.tray_paths
            if tray_path not in self.supplied_tray_paths
        ]

    def begin_motion(self, drive_api, target, duration, state, label):
        self.motion_drive = drive_api
        self.motion_start = tray_loader_get_drive_target(drive_api, 0.0)
        self.motion_target = float(target)
        self.motion_duration = max(0.01, float(duration))
        self.elapsed = 0.0
        self.state = state

        tray_loader_log(
            f"{label} 시작 | {self.motion_start:.2f} "
            f"-> {self.motion_target:.2f} | {self.motion_duration:.2f}초"
        )

    def update_motion(self, dt):
        self.elapsed += max(0.0, float(dt))
        progress = max(0.0, min(1.0, self.elapsed / self.motion_duration))
        blend = tray_loader_scurve(progress)
        command = self.motion_start + (
            self.motion_target - self.motion_start
        ) * blend
        tray_loader_set_drive_target(self.motion_drive, command)
        return progress >= 1.0

    def request_start(self, reason, source_box_path=None):
        if self.active:
            return False

        if not omni.timeline.get_timeline_interface().is_playing():
            return False

        try:
            self.ensure_resources()
            remaining_trays = self.get_remaining_trays()

            if not remaining_trays:
                return False

            self.current_candidate_paths = list(remaining_trays)
            self.target_tray_path = remaining_trays[0]
            self.target_upper_position = tray_loader_upper_target_for_path(
                self.target_tray_path
            )
            self.next_upper_tray_path = None
            self.next_upper_position = None

            self.active = True
            self.cycle_started = True
            self.cycle_completed = False
            self.tray_ready = False
            self.active_tray_path = None
            self.start_reason = str(reason)
            self.stop_requested = False
            self.restore_convex_after_home = False
            self.positions_before_push = {}

            # 트레이 공급과 동시에 포장 컨베이어 +35 가동.
            # 공급이 끝나도 컨베이어는 계속 가동한다.
            try:
                start_pack_conveyor("트레이로더 순차 공급")
            except Exception:
                pass

            current_upper = tray_loader_get_drive_target(
                self.upper_drive,
                self.target_upper_position,
            )

            if abs(current_upper - self.target_upper_position) <= 0.01:
                self.elapsed = 0.0
                self.state = self.UPPER_SETTLE
            else:
                self.begin_motion(
                    self.upper_drive,
                    self.target_upper_position,
                    TRAY_LOADER_UPPER_SUPPLY_DURATION_SEC,
                    self.UPPER_MOVE,
                    "Upper 순차 위치 이동",
                )

            return True

        except Exception:
            self.active = False
            self.state = self.ERROR
            return False

    def start_next_upper_preposition(self):
        """
        Push가 30으로 복귀한 뒤 다음 미공급 Tray의 Upper 위치로 미리 이동한다.

        이 단계에서는 다음 Tray를 밀지 않는다.
        이후 트레이당 2개 박스 적재 완료 신호가 들어오면
        request_start()가 같은 Tray를 선택하고 Push 공급을 진행한다.
        """
        if not TRAY_LOADER_PREPOSITION_NEXT_UPPER:
            return False

        if self.restore_convex_after_home:
            return False

        remaining_trays = self.get_remaining_trays()

        if not remaining_trays:
            self.next_upper_tray_path = None
            self.next_upper_position = None
            return False

        self.next_upper_tray_path = str(remaining_trays[0])
        self.next_upper_position = tray_loader_upper_target_for_path(
            self.next_upper_tray_path
        )

        current_upper = tray_loader_get_drive_target(
            self.upper_drive,
            self.next_upper_position,
        )

        if abs(current_upper - self.next_upper_position) <= 0.01:
            tray_loader_set_drive_target(
                self.upper_drive,
                self.next_upper_position,
            )
            self.elapsed = 0.0
            self.state = self.NEXT_UPPER_SETTLE
        else:
            self.begin_motion(
                self.upper_drive,
                self.next_upper_position,
                TRAY_LOADER_NEXT_UPPER_MOVE_DURATION_SEC,
                self.NEXT_UPPER_MOVE,
                "다음 Tray Upper 미리 이동",
            )

        return True

    def request_stop(self):
        # 안전 정지는 포장 컨베이어를 멈추고 Push만 30으로 복귀한다.
        # Upper는 현재 위치를 유지한다.
        try:
            stop_pack_conveyor("트레이로더 정지")
        except Exception:
            pass

        try:
            self.ensure_resources()
        except Exception:
            self.active = False
            self.state = self.ERROR
            return False

        self.active = True
        self.stop_requested = False
        self.restore_convex_after_home = True
        self.start_push_home_sequence(
            reason="수동 STOP",
            restore_convex=True,
        )
        return True

    def start_push_home_sequence(self, reason, restore_convex=False):
        self.stop_requested = False

        if restore_convex:
            self.restore_convex_after_home = True

        self.begin_motion(
            self.push_drive,
            TRAY_LOADER_PUSH_HOME_POSITION,
            TRAY_LOADER_PUSH_HOME_DURATION_SEC,
            self.PUSH_HOME_MOVE,
            "Push 복귀",
        )

    def restore_convex(self, reason):
        restored = tray_loader_restore_all_colliders_to_convex_hull(
            self.tray_paths or None
        )
        self.sdf_mesh_paths = []
        self.tray_ready = False
        return restored

    def finish_cycle(self):
        tray_loader_set_drive_target(
            self.push_drive,
            TRAY_LOADER_PUSH_HOME_POSITION,
        )

        # Upper는 현재 Tray 공급 위치를 유지한다.
        if self.restore_convex_after_home:
            self.restore_convex("수동 STOP 복귀 완료")
            self.restore_convex_after_home = False

        self.active = False
        self.cycle_completed = True
        self.cycle_count += 1
        self.state = self.DONE

    def reset_for_play(self):
        self.state = self.IDLE
        self.elapsed = 0.0
        self.active = False
        self.cycle_started = False
        self.cycle_completed = False
        self.cycle_count = 0
        self.tray_ready = False
        self.active_tray_path = None
        self.target_tray_path = None
        self.target_upper_position = None
        self.next_upper_tray_path = None
        self.next_upper_position = None
        self.start_reason = ""
        self.stop_requested = False
        self.restore_convex_after_home = False
        self.positions_before_push = {}
        self.current_candidate_paths = []
        self.supplied_tray_paths.clear()
        self.sdf_mesh_paths = []
        tray_loader_rejector_section_box_set.clear()
        tray_loader_z_triggered_box_set.clear()

        for name in (
            "_assembly714_tray_loader_subscription",
            "_assembly714_tray_loader_timeline_subscription",
        ):
            old_subscription = getattr(builtins, name, None)
            if old_subscription is not None:
                try:
                    old_subscription.unsubscribe()
                except Exception:
                    pass
                setattr(builtins, name, None)

    def handle_timeline_stop(self):
        try:
            self.ensure_resources()
            tray_loader_set_drive_target(
                self.push_drive,
                TRAY_LOADER_PUSH_HOME_POSITION,
            )
            # 다음 PLAY에서 0 -> -167 S-Curve가 시작되도록 Upper 목표를 0으로 준비한다.
            tray_loader_set_drive_target(
                self.upper_drive,
                TRAY_LOADER_STARTUP_UPPER_HOME_POSITION,
            )
            self.restore_convex("Timeline STOP")
        except Exception:
            pass

        self.state = self.IDLE
        self.elapsed = 0.0
        self.active = False
        self.cycle_started = False
        self.cycle_completed = False
        self.cycle_count = 0
        self.tray_ready = False
        self.active_tray_path = None
        self.target_tray_path = None
        self.target_upper_position = None
        self.next_upper_tray_path = None
        self.next_upper_position = None
        self.start_reason = ""
        self.stop_requested = False
        self.restore_convex_after_home = False
        self.positions_before_push = {}
        self.current_candidate_paths = []
        self.supplied_tray_paths.clear()
        tray_loader_rejector_section_box_set.clear()
        tray_loader_z_triggered_box_set.clear()

    def update(self, event):
        if not self.active:
            return

        if not omni.timeline.get_timeline_interface().is_playing():
            return

        try:
            dt = float(event.payload.get("dt", 0.0))

            if self.state == self.STARTUP_UPPER_MOVE:
                if self.update_motion(dt):
                    tray_loader_set_drive_target(
                        self.upper_drive,
                        TRAY_LOADER_STARTUP_UPPER_TARGET_POSITION,
                    )
                    self.elapsed = 0.0
                    self.state = self.STARTUP_UPPER_SETTLE
                return

            if self.state == self.STARTUP_UPPER_SETTLE:
                self.elapsed += max(0.0, dt)
                if self.elapsed >= TRAY_LOADER_STARTUP_UPPER_SETTLE_SEC:
                    self.active = False
                    self.state = self.IDLE
                return

            if self.state == self.UPPER_MOVE:
                if self.update_motion(dt):
                    tray_loader_set_drive_target(
                        self.upper_drive,
                        self.target_upper_position,
                    )
                    self.elapsed = 0.0
                    self.state = self.UPPER_SETTLE
                return

            if self.state == self.UPPER_SETTLE:
                self.elapsed += max(0.0, dt)
                if self.elapsed >= TRAY_LOADER_UPPER_SETTLE_SEC:
                    self.positions_before_push = tray_loader_collect_positions(
                        self.current_candidate_paths
                    )
                    self.begin_motion(
                        self.push_drive,
                        TRAY_LOADER_PUSH_SUPPLY_POSITION,
                        TRAY_LOADER_PUSH_SUPPLY_DURATION_SEC,
                        self.PUSH_MOVE,
                        "Push 전진",
                    )
                return

            if self.state == self.PUSH_MOVE:
                if self.update_motion(dt):
                    tray_loader_set_drive_target(
                        self.push_drive,
                        TRAY_LOADER_PUSH_SUPPLY_POSITION,
                    )
                    self.elapsed = 0.0
                    self.state = self.PUSH_SETTLE
                return

            if self.state == self.PUSH_SETTLE:
                self.elapsed += max(0.0, dt)
                if self.elapsed >= TRAY_LOADER_PUSH_SETTLE_SEC:
                    self.state = self.SWITCH_SDF
                return

            if self.state == self.SWITCH_SDF:
                moved_tray_path, displacement = tray_loader_find_moved_tray(
                    self.current_candidate_paths,
                    self.positions_before_push,
                )

                if moved_tray_path is None:
                    raise RuntimeError("이동한 트레이를 찾지 못함")

                changed_mesh_paths = tray_loader_switch_collider_to_sdf(
                    moved_tray_path
                )
                self.sdf_mesh_paths = list(changed_mesh_paths)
                self.active_tray_path = moved_tray_path
                self.supplied_tray_paths.add(moved_tray_path)
                self.tray_ready = True
                set_active_pack_tray(moved_tray_path)

                self.elapsed = 0.0
                self.state = self.FINAL_HOLD
                return

            if self.state == self.FINAL_HOLD:
                self.elapsed += max(0.0, dt)
                if self.elapsed >= TRAY_LOADER_FINAL_HOLD_SEC:
                    self.start_push_home_sequence(
                        reason="트레이 공급 완료",
                        restore_convex=False,
                    )
                return

            if self.state == self.PUSH_HOME_MOVE:
                if self.update_motion(dt):
                    tray_loader_set_drive_target(
                        self.push_drive,
                        TRAY_LOADER_PUSH_HOME_POSITION,
                    )
                    self.elapsed = 0.0
                    self.state = self.PUSH_HOME_SETTLE
                return

            if self.state == self.PUSH_HOME_SETTLE:
                self.elapsed += max(0.0, dt)
                if self.elapsed >= TRAY_LOADER_PUSH_HOME_SETTLE_SEC:
                    if self.start_next_upper_preposition():
                        return

                    self.finish_cycle()
                return

            if self.state == self.NEXT_UPPER_MOVE:
                if self.update_motion(dt):
                    tray_loader_set_drive_target(
                        self.upper_drive,
                        self.next_upper_position,
                    )
                    self.elapsed = 0.0
                    self.state = self.NEXT_UPPER_SETTLE
                return

            if self.state == self.NEXT_UPPER_SETTLE:
                self.elapsed += max(0.0, dt)
                if self.elapsed >= TRAY_LOADER_NEXT_UPPER_SETTLE_SEC:
                    self.finish_cycle()
                return

        except Exception:
            self.active = False
            self.state = self.ERROR


integrated_tray_loader_controller = IntegratedTrayLoaderController()

# 이전에 단독 트레이로더 v3를 실행한 상태에서 통합본을 실행해도
# 중복 update/timeline 콜백이 남지 않도록 즉시 정리한다.
integrated_tray_loader_controller.reset_for_play()


def request_tray_loader_start(reason, source_box_path=None):
    source_box_id = box_path_id_dict.get(source_box_path, None) if source_box_path else None

    event_log(
        f"[공정 이벤트] 트레이로더 시작 요청 | "
        f"reason={reason} | source_box_id={source_box_id}"
    )

    success = integrated_tray_loader_controller.request_start(
        reason=reason,
        source_box_path=source_box_path,
    )

    event_log(
        f"[공정 이벤트] 트레이로더 요청 결과 | "
        f"success={success} | state={integrated_tray_loader_controller.state}"
    )

    return success


def start_tray_loader():
    """Script Editor Console 수동 시작용."""
    return request_tray_loader_start("수동 시작")


def stop_tray_loader():
    """Script Editor Console 안전 정지용."""
    integrated_tray_loader_controller.request_stop()


builtins.start_tray_loader = start_tray_loader
builtins.stop_tray_loader = stop_tray_loader


def multi_tray_place_status():
    tray_positions = {}

    for tray_path in get_pack_tray_candidate_paths():
        position = get_world_position(
            tray_path
        )

        tray_name = tray_path.rsplit(
            "/",
            1,
        )[-1]

        tray_positions[tray_name] = (
            None
            if position is None
            else (
                float(position[0]),
                float(position[1]),
                float(position[2]),
            )
        )

    status = {
        "robot_state": robot_sequence.get(
            "state",
            "IDLE",
        ),
        "trigger_armed": (
            pack_tray_trigger_controller.get(
                "armed",
                False,
            )
        ),
        "selected_tray": (
            pack_tray_trigger_controller.get(
                "selected_tray_path",
                None,
            )
            or tray_follow_controller.get(
                "selected_tray_path",
                None,
            )
        ),
        "follow_active": (
            tray_follow_controller.get(
                "active",
                False,
            )
        ),
        "follow_phase": (
            tray_follow_controller.get(
                "phase",
                "IDLE",
            )
        ),
        "place_y": (
            TRAY_CONFIRMED_PLACE_Y_POSITION
        ),
        "place_z": (
            TRAY_CONFIRMED_PLACE_Z_POSITION
        ),
        "release_wait_sec": (
            TRAY_PRE_RELEASE_HOLD_SEC
        ),
        "tray_positions": tray_positions,
    }

    _SYSTEM_PRINT(
        "[MULTI TRAY PLACE STATUS]",
        status,
    )

    return status


builtins.multi_tray_place_status = (
    multi_tray_place_status
)


def next_box_preload_status():
    pending_box = get_pending_stopped_pick_box()

    status = {
        "immediate_after_attach": (
            NEXT_BOX_PRELOAD_IMMEDIATE_AFTER_ATTACH
        ),
        "preload_active": (
            next_box_preload_active
        ),
        "conveyor_started": (
            next_box_preload_conveyor_started
        ),
        "source_box": (
            next_box_preload_source_box
        ),
        "pending_waiting_box": (
            pending_box
        ),
        "robot_state": (
            robot_sequence.get(
                "state",
                "IDLE",
            )
        ),
        "post_pick_y_target": (
            AUTO_PICK_Y_UP_POSITION
        ),
        "post_pick_z_target": (
            AUTO_POST_PICK_Z_POSITION
        ),
        "forced_feed_active": (
            forced_next_box_feed_controller.get(
                "active",
                False
            )
        ),
        "forced_feed_phase": (
            forced_next_box_feed_controller.get(
                "phase",
                "IDLE"
            )
        ),
        "forced_feed_delay_elapsed": (
            forced_next_box_feed_controller.get(
                "delay_elapsed_sec",
                0.0
            )
        ),
        "forced_feed_delay_target": (
            NEXT_BOX_FEED_START_DELAY_SEC
        ),
        "forced_feed_readback": (
            get_conveyor_x_velocity()
        ),
        "post_pick_pipeline_phase": (
            post_pick_axis_pipeline.get(
                "phase",
                "IDLE"
            )
        ),
        "y_drive_target": (
            get_axis_target_position(
                "Y"
            )
        ),
        "z_drive_target": (
            get_axis_target_position(
                "Z"
            )
        ),
        "y_geometry_actual": (
            get_axis_raw_geometry_position(
                "Y"
            )
        ),
        "z_geometry_actual": (
            get_axis_raw_geometry_position(
                "Z"
            )
        ),
        "post_pick_geometry_target": (
            post_pick_axis_pipeline.get(
                "geometry_target"
            )
        ),
        "robot_busy": (
            is_auto_sequence_running
        ),
        "sequence_guard_locked": (
            is_robot_sequence_locked()
        ),
        "sequence_guard_owner": (
            robot_sequence_guard.get(
                "owner_box"
            )
        ),
        "sequence_guard_reason": (
            robot_sequence_guard.get(
                "reason",
                ""
            )
        ),
        "conveyor_speed": (
            get_conveyor_x_velocity()
        ),
    }

    _SYSTEM_PRINT(
        "[NEXT BOX PRELOAD STATUS]",
        status,
        flush=True,
    )

    return status


builtins.next_box_preload_status = (
    next_box_preload_status
)


# ==================================================
# [17. 마스터 업데이트 루프]
# ==================================================

def on_update_loop(e):
    global is_conveyor_running
    global debug_count
    global robot_init_watchdog_frames
    global first_depth_spawned_box_path
    global first_depth_tray_loader_started

    # 0. 리젝터 구간 이후 시작된 트레이로더를 동일 update에서 제어
    integrated_tray_loader_controller.update(e)

    # 0-1. 3축 초기화 watchdog
    # 코드를 PLAY 중에 실행했거나 첫 초기화 순간 Drive가 준비되지 않았어도
    # 약 60 update 프레임마다 HOME 초기화를 다시 요청한다.
    if (
        omni.timeline
        .get_timeline_interface()
        .is_playing()
        and not system_ready
        and not is_auto_sequence_running
        and not motion_controller.get(
            "active",
            False
        )
        and robot_sequence.get(
            "state",
            "IDLE"
        ) in (
            "IDLE",
            "HOME_INITIAL"
        )
    ):
        robot_init_watchdog_frames += 1

        if robot_init_watchdog_frames >= 60:
            robot_init_watchdog_frames = 0

            request_robot_initialization(
                "update_watchdog"
            )
    else:
        robot_init_watchdog_frames = 0

    # 1. S-Curve targetPosition 갱신 및 완료 판정
    completed_motion = update_scurve_motion()

    if completed_motion is not None:
        completed_label, actual_positions = (
            completed_motion
        )

        handle_robot_motion_complete(
            completed_label,
            actual_positions
        )

    # 2. 실제 Gripper Revolute Joint S-Curve 회전
    completed_gripper_rotation = (
        update_gripper_joint_motion()
    )

    if completed_gripper_rotation is not None:
        (
            completed_rotation_label,
            completed_rotation_angle
        ) = completed_gripper_rotation

        handle_gripper_joint_motion_complete(
            completed_rotation_label,
            completed_rotation_angle
        )

    # 2-1. 첫 번째 적재 박스는 Tray2 Local Z=239까지 로봇 X/Z 이동 자체를 보류한다.
    update_first_tray_pick_wait()

    # 3. 흡착 직후 강력 고정 안정화
    update_grip_stabilization()

    # 4. 해제 후 트레이 안착 대기
    update_release_settle()

    # 5. 1번은 Tray X=4에서 선택, 2번은 동일 Tray를 사용한다.
    update_pack_tray_trigger(e)

    # 5-1. 1번은 Tray X=-30에서 대각선 적재,
    #      2번은 준비 완료 직후 즉시 대각선 적재한다.
    update_tray_follow_motion(e)

    # 먼저 흡착 박스를 현재 그리퍼 위치로 갱신한다.
    update_attached_box_hard_lock()

    # v46.2는 Hard-Lock 흡착 성공 순간 즉시 재가동하므로
    # Y 변화 기반 재가동은 예비 모드에서만 사용한다.
    if not NEXT_BOX_PRELOAD_IMMEDIATE_AFTER_ATTACH:
        update_next_box_preload_y_trigger()

    # v46.4:
    # 다음 박스가 픽업 위치에 도착할 때까지 검사 컨베이어를
    # 실제 속도 읽기와 무관하게 매 프레임 -65로 강제 유지한다.
    update_forced_next_box_feed(
        e
    )

    # Y=50 -> Z=-270 단계가 콜백 누락으로 멈추지 않도록
    # 매 프레임 파이프라인 상태를 확인하고 재시도한다.
    update_post_pick_axis_pipeline_watchdog()

    # 첫 번째 박스의 적재/HOME 복귀가 끝나기 전에는
    # 다른 코드가 is_auto_sequence_running을 False로 바꾸더라도 복구한다.
    enforce_robot_sequence_guard()

    # 정지 좌표에서 대기 중인 박스가 있어도 시퀀스 하드락이
    # 풀린 HOME 완료 이후에만 자동 픽업을 시작한다.
    try_start_pending_pick_sequence()

    # 2. 검사 컨베이어의 박스 위치 감시
    box_paths = get_monitor_box_paths()

    if box_paths:
        debug_count += 1

        for box_path in box_paths:
            current_pos = get_world_position(
                box_path
            )

            prev_pos = last_box_pos_dict.get(
                box_path,
                None
            )

            if current_pos is None:
                continue

            apply_defect_red_if_needed(
                box_path,
                current_pos
            )

            box_result = box_result_dict.get(
                box_path,
                None
            )

            # v51.17:
            # 불량 박스가 World Z=640에 도착하거나 통과하면
            # 리젝터 0도 -> -100도 -> 0도 복귀 순서로 처리한다.
            # 불량 박스는 Stage에서 삭제하지 않고 그대로 유지한다.
            if (
                box_result == 0
                and box_path
                not in defect_remove_scheduled_box_set
                and has_reached_defect_remove_trigger(
                    prev_pos,
                    current_pos
                )
            ):
                if schedule_defect_box_removal(
                    box_path,
                    current_pos
                ):
                    continue

            if box_path in defect_remove_scheduled_box_set:
                continue

            # 정상/미판정 박스는 로봇 준비 여부와 관계없이
            # 지정된 Z 좌표에 도착했을 때만 검사 컨베이어를 정지한다.
            if (
                is_conveyor_running
                and box_path
                not in auto_sequence_done_box_set
                and box_result != 0
            ):
                if has_reached_stop_target(
                    prev_pos,
                    current_pos
                ):
                    set_conveyor_x_velocity(
                        CONVEYOR_STOP_X_SPEED
                    )

                    is_conveyor_running = False

                    stop_box_velocity(
                        box_path
                    )

                    stopped_position = (
                        copy_world_position(
                            current_pos
                        )
                    )

                    stopped_box_world_position_dict[
                        box_path
                    ] = stopped_position

                    # 1차 박스를 적재하는 동안 도착한 2차 박스라면
                    # 이 위치에서 대기 상태를 확정한다.
                    finish_next_box_preload_at_stop(
                        box_path
                    )

                    event_log(
                        f"[정지 위치] "
                        f"속도={CONVEYOR_STOP_X_SPEED:.1f} | "
                        f"ID={box_path_id_dict.get(box_path, None)} | "
                        f"Box X={stopped_position[0]:.3f} | "
                        f"Box Z={stopped_position[2]:.3f} | "
                        f"정지 기준 Z={STOP_TARGET_Z:.3f}"
                    )

                    if (
                        system_ready
                        and not is_auto_sequence_running
                        and not is_robot_sequence_locked()
                        and not is_grip_attached
                        and robot_sequence.get(
                            "state",
                            "IDLE"
                        ) == "IDLE"
                        and not post_pick_axis_pipeline.get(
                            "active",
                            False
                        )
                        and not pack_tray_trigger_controller.get(
                            "active",
                            False
                        )
                        and not tray_follow_controller.get(
                            "active",
                            False
                        )
                        and not motion_controller.get(
                            "active",
                            False
                        )
                    ):
                        start_auto_pick_sequence_for_box(
                            box_path,
                            stopped_position
                        )
                    else:
                        event_log(
                            f"[그리퍼 이동] 픽업 위치 대기 | "
                            f"ID={box_path_id_dict.get(box_path, None)} | "
                            "검사 컨베이어는 좌표에서 정지 완료 | "
                            "로봇 준비 완료 후 자동 픽업"
                        )

            # ==================================================
            # v51.7: 첫 번째 생성 박스 Z=500 도착 시
            # 첫 트레이 Upper 이동 + Push 공급 시퀀스 시작
            # ==================================================
            tray_loader_trigger_reached = (
                has_reached_tray_loader_trigger(
                    prev_pos,
                    current_pos,
                )
            )

            current_result = box_result_dict.get(
                box_path,
                None
            )

            is_first_depth_box = (
                first_depth_spawned_box_path is not None
                and box_path == first_depth_spawned_box_path
            )

            tray_loader_box_allowed = (
                is_first_depth_box
                if TRAY_LOADER_FIRST_DEPTH_BOX_ONLY
                else True
            )

            if (
                tray_loader_trigger_reached
                and tray_loader_box_allowed
                and not first_depth_tray_loader_started
                and box_path not in tray_loader_z_triggered_box_set
            ):
                if TRAY_LOADER_START_REGARDLESS_OF_RESULT:
                    start_success = request_tray_loader_start(
                        reason=(
                            f"첫 번째 생성 박스 Z=500 도착 | "
                            f"현재 Z={float(current_pos[2]):.5f} | "
                            f"기준 Z={TRAY_LOADER_TRIGGER_Z:.5f} | "
                            f"판정={result_to_text(current_result)} | "
                            f"ID={box_path_id_dict.get(box_path, None)}"
                        ),
                        source_box_path=box_path,
                    )

                    if start_success:
                        first_depth_tray_loader_started = True
                        tray_loader_z_triggered_box_set.add(
                            box_path
                        )
                        tray_loader_log(
                            f"첫 번째 생성 박스 Z 감지 성공 | "
                            f"현재 Z={float(current_pos[2]):.5f} | "
                            f"기준 Z={TRAY_LOADER_TRIGGER_Z:.5f} | "
                            f"ID={box_path_id_dict.get(box_path, None)} | "
                            "Upper 이동 후 Push 공급 시작"
                        )
                    else:
                        # 시작 실패 시 Set에 넣지 않아 다음 프레임에 재시도한다.
                        tray_loader_log(
                            f"첫 트레이 시작 대기/재시도 | "
                            f"현재 Z={float(current_pos[2]):.5f} | "
                            f"기준 Z={TRAY_LOADER_TRIGGER_Z:.5f}"
                        )

            # 리젝터는 기존 Z=611.18216 감지선을 독립적으로 사용한다.
            rejector_section_reached = has_passed_rejector_trigger(
                prev_pos,
                current_pos
            )

            # 불량 판정은 기존처럼 늦게 들어와도 리젝터 기준선까지 계속 감시한다.
            if (
                current_result == 0
                and box_path not in rejector_triggered_box_set
                and rejector_section_reached
            ):
                rejector_triggered_box_set.add(box_path)
                tray_loader_rejector_section_box_set.add(box_path)

                process_rejector_passed_box(
                    box_path,
                    current_pos
                )

            last_box_pos_dict[
                box_path
            ] = current_pos

    # 5. 흡착된 박스를 그리퍼 위치/각도에 매 프레임 강제 고정
    update_attached_box_hard_lock()


# ==================================================
# [18. 키보드 입력]
# ==================================================

def is_exact_key(event, key_name):
    key_value = getattr(carb.input.KeyboardInput, key_name, None)

    if key_value is None:
        return False

    return event.input == key_value


def on_keyboard_event(event, *args, **kwargs):
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:

        if is_exact_key(event, "B"):
            spawn_box()

        elif is_exact_key(event, "V"):
            toggle_conveyor()

        elif is_exact_key(event, "P"):
            print_status()

        elif is_exact_key(event, "R"):
            asyncio.ensure_future(run_rejector())

        elif is_exact_key(event, "J"):
            move_y_down()

        elif is_exact_key(event, "L"):
            move_y_mid()

        elif is_exact_key(event, "K"):
            move_y_home()

        elif is_exact_key(event, "U"):
            move_z_to_target()

        elif is_exact_key(event, "I"):
            move_x_to_target()

        elif is_exact_key(event, "C"):
            close_grip_no_jump()

        elif is_exact_key(event, "O"):
            open_grip_no_jump()

        elif is_exact_key(event, "A"):
            force_start_auto_pick_sequence()

    return True


# ==================================================
# [19. 타임라인 이벤트]
# ==================================================

robot_play_init_running = False
robot_play_init_scheduled = False
robot_init_watchdog_frames = 0


def request_robot_initialization(reason):
    """
    Timeline이 이미 PLAY인 상태에서 코드를 실행해도 초기 HOME 이동을
    반드시 예약한다. 중복 coroutine은 생성하지 않는다.
    """
    global robot_play_init_scheduled

    timeline = (
        omni.timeline
        .get_timeline_interface()
    )

    if not timeline.is_playing():
        return False

    if system_ready:
        return False

    if robot_play_init_running or robot_play_init_scheduled:
        return False

    if motion_controller.get(
        "active",
        False
    ):
        return False

    state = robot_sequence.get(
        "state",
        "IDLE"
    )

    if state not in (
        "IDLE",
        "HOME_INITIAL"
    ):
        return False

    robot_play_init_scheduled = True

    event_log(
        f"[3축 초기화 예약] reason={reason}"
    )

    asyncio.ensure_future(
        initialize_robot_after_play(
            reason
        )
    )

    return True


async def initialize_robot_after_play(
    reason="unknown"
):
    """
    PhysX가 Joint Drive 속성을 준비한 뒤 초기 HOME 이동을 시작한다.

    v12 보완:
    - PLAY 상태에서 코드를 나중에 실행해도 동작
    - Drive 준비가 늦으면 update watchdog이 다시 요청
    - 시작 실패 시 HOME_INITIAL 상태에 고정되지 않음
    """
    global robot_play_init_running
    global robot_play_init_scheduled

    robot_play_init_scheduled = False

    if robot_play_init_running:
        return

    robot_play_init_running = True

    try:
        app = omni.kit.app.get_app()

        # PhysX/Stage가 Drive 속성을 준비할 프레임 확보
        await app.next_update_async()
        await app.next_update_async()
        await app.next_update_async()

        timeline = (
            omni.timeline
            .get_timeline_interface()
        )

        if not timeline.is_playing():
            event_log(
                f"[3축 초기화 취소] "
                f"Timeline STOP | reason={reason}"
            )
            return

        ensure_joint_limits()
        log_joint_position_sources()

        drive_status = []
        drive_ready = True

        for axis_name in (
            "X1",
            "X2",
            "Y",
            "Z"
        ):
            target_value = get_axis_target_position(
                axis_name
            )

            if target_value is None:
                drive_ready = False
                drive_status.append(
                    f"{axis_name}=읽기 실패"
                )
            else:
                drive_status.append(
                    f"{axis_name}={float(target_value):.2f}"
                )

        event_log(
            f"[3축 Drive 확인] "
            f"{', '.join(drive_status)} | "
            f"reason={reason}"
        )

        if not drive_ready:
            robot_sequence["state"] = "IDLE"

            event_log(
                "[3축 초기화 재시도 대기] "
                "일부 targetPosition 속성이 아직 준비되지 않음"
            )
            return

        if not system_ready:
            started = start_initial_home_motion()

            if not started:
                event_log(
                    "[초기 위치 오류] "
                    "Drive targetPosition 읽기/설정 실패, 자동 재시도 예정"
                )
            else:
                event_log(
                    f"[초기 위치 명령 시작] reason={reason}"
                )

    except Exception as error:
        robot_sequence["state"] = "IDLE"

        event_log(
            f"[3축 초기화 예외] {error}"
        )

    finally:
        robot_play_init_running = False


def on_timeline_event(e):
    global is_conveyor_running
    global is_grip_attached
    global attached_box_path
    global passed_box_count
    global is_rejector_moving
    global debug_count
    global is_auto_sequence_running
    global system_ready
    global robot_play_init_scheduled
    global robot_init_watchdog_frames
    global attached_box_yaw_at_attach
    global attached_gripper_logical_yaw_at_attach
    global auto_place_count
    global master_auto_box_sequence
    global first_depth_spawned_box_path
    global first_depth_tray_loader_started

    if e.type == int(
        omni.timeline.TimelineEventType.PLAY
    ):
        auto_place_count = 0
        master_auto_box_sequence = 0
        first_depth_spawned_box_path = None
        first_depth_tray_loader_started = False
        master_pending_normal_results.clear()
        master_processed_vision_keys.clear()
        master_processed_vision_key_set.clear()
        master_processed_inspection_keys.clear()
        master_processed_inspection_key_set.clear()
        robot_sequence["place_slot"] = None
        integrated_tray_loader_controller.reset_for_play()
        integrated_tray_loader_controller.start_startup_upper_motion()
        cancel_pack_tray_trigger()
        reset_next_box_preload()
        reset_first_tray_pick_wait()

        cancel_release_settle(
            restore_physics=True,
            clear_completed=True
        )

        prepare_gripper_joint_drive(
            emit_log=True
        )

        set_gripper_joint_logical_angle_immediate(
            GRIPPER_JOINT_HOME_LOGICAL_DEG,
            emit_log=False
        )

        # 박스 생성 여부나 로봇 준비 여부와 관계없이
        # 검사 컨베이어는 PLAY 시작과 함께 계속 가동한다.
        start_conveyor()

        request_robot_initialization(
            "timeline_play"
        )

        event_log(
            "[Timeline] PLAY | "
            f"검사 컨베이어 계속 가동={CONVEYOR_RUN_X_SPEED:.1f} | "
            "지정 Z 좌표에서만 자동 정지"
        )

    if e.type == int(
        omni.timeline.TimelineEventType.STOP
    ):
        # Timeline STOP에서는 물리 이동은 멈추지만 Target과 Collider는 즉시 복구한다.
        integrated_tray_loader_controller.handle_timeline_stop()
        reset_next_box_preload()
        reset_first_tray_pick_wait()

        cancel_scurve_motion()
        cancel_gripper_joint_motion()
        cancel_grip_stabilization()
        cancel_release_settle()
        cancel_tray_follow_motion()
        cancel_pack_tray_trigger()

        if is_grip_attached and attached_box_path:
            set_box_collision_enabled(
                attached_box_path,
                True,
                restore_saved_state=True
            )
            set_box_rigidbody_enabled(
                attached_box_path,
                True
            )

        set_gripper_joint_logical_angle_immediate(
            GRIPPER_JOINT_HOME_LOGICAL_DEG,
            emit_log=False
        )

        robot_play_init_scheduled = False
        robot_init_watchdog_frames = 0

        is_conveyor_running = False
        reset_forced_next_box_feed()
        reset_post_pick_axis_pipeline()
        unlock_robot_sequence()
        is_grip_attached = False
        attached_box_path = None
        attached_box_yaw_at_attach = 0.0
        attached_gripper_logical_yaw_at_attach = 0.0
        is_rejector_moving = False
        is_auto_sequence_running = False
        system_ready = False

        axis_geometry_calibrated = False
        axis_geometry_offset_dict.clear()
        axis_geometry_raw_at_calibration_dict.clear()
        axis_target_at_calibration_dict.clear()

        robot_sequence["state"] = "IDLE"
        robot_sequence["target_box"] = None
        robot_sequence[
            "target_gripper_logical_yaw"
        ] = GRIPPER_JOINT_HOME_LOGICAL_DEG
        robot_sequence[
            "dynamic_pick_targets"
        ] = None
        robot_sequence["place_slot"] = None
        auto_place_count = 0
        master_auto_box_sequence = 0
        first_depth_spawned_box_path = None
        first_depth_tray_loader_started = False
        master_pending_normal_results.clear()
        master_processed_vision_keys.clear()
        master_processed_vision_key_set.clear()
        master_processed_inspection_keys.clear()
        master_processed_inspection_key_set.clear()

        attached_box_collision_state_dict.clear()

        passed_box_count = 0
        debug_count = 0

        rejector_triggered_box_set.clear()
        tray_loader_rejector_section_box_set.clear()
        tray_loader_z_triggered_box_set.clear()
        auto_sequence_done_box_set.clear()
        defect_red_applied_box_set.clear()
        defect_remove_scheduled_box_set.clear()
        pending_inspection_box_paths.clear()
        inspection_track_box_path_dict.clear()

        box_id_path_dict.clear()
        box_path_id_dict.clear()
        box_result_dict.clear()
        box_label_dict.clear()
        box_yaw_dict.clear()

        set_conveyor_x_velocity(
            CONVEYOR_STOP_X_SPEED
        )

        try:
            joint_prim = get_prim(
                REJECTOR_JOINT_PATH
            )

            if joint_prim:
                for attr_str in [
                    "drive:angular:physics:targetPosition",
                    "drive:linear:physics:targetPosition"
                ]:
                    attr = joint_prim.GetAttribute(
                        attr_str
                    )

                    if attr.IsValid():
                        attr.Set(
                            float(
                                REJECTOR_HOME_VALUE
                            )
                        )
        except Exception:
            pass

        valid_boxes = get_valid_spawned_boxes()

        if valid_boxes:
            # Timeline STOP 콜백 안에서 선택된 Prim을 즉시 삭제하면
            # Transform Manipulator가 이미 삭제된 pivot을 참조할 수 있다.
            clear_stage_selection()

            asyncio.ensure_future(
                delete_prims_safely(
                    valid_boxes,
                    reason="timeline_stop"
                )
            )

            spawned_boxes.clear()
            last_box_pos_dict.clear()

        event_log(
            "[Timeline] STOP | "
            "컨베이어/시퀀스/생성 박스 초기화"
        )


# ==================================================
# [20. 이벤트 구독 연결]
# ==================================================

appwindow = omni.appwindow.get_default_app_window()
input_interface = carb.input.acquire_input_interface()
keyboard = appwindow.get_keyboard()

final_system_kb_sub = (
    input_interface
    .subscribe_to_keyboard_events(
        keyboard,
        on_keyboard_event
    )
)

timeline_stream = (
    omni.timeline
    .get_timeline_interface()
    .get_timeline_event_stream()
)

final_system_timeline_sub = (
    timeline_stream
    .create_subscription_to_pop(
        on_timeline_event,
        name="assembly714_main_timeline"
    )
)

update_stream = (
    omni.kit.app.get_app()
    .get_update_event_stream()
)

final_system_update_sub = (
    update_stream
    .create_subscription_to_pop(
        on_update_loop,
        name="assembly714_main_update"
    )
)

ensure_joint_limits()
ensure_all_joint_state_apis()

if (
    omni.timeline
    .get_timeline_interface()
    .is_playing()
):
    integrated_tray_loader_controller.start_startup_upper_motion()
    start_conveyor()

    request_robot_initialization(
        "script_loaded_while_playing"
    )

    event_log(
        f"[검사 컨베이어 즉시 가동] "
        f"속도={CONVEYOR_RUN_X_SPEED:.1f} | "
        "Timeline이 이미 PLAY 상태 | "
        "3축 초기화도 자동 예약"
    )
else:
    integrated_tray_loader_controller.prepare_startup_upper_home()
    set_conveyor_x_velocity(
        CONVEYOR_STOP_X_SPEED
    )

event_log(
    "[3축 대기] "
    "STOP 상태면 Play 시 초기화 / "
    "이미 PLAY 상태면 즉시 초기화 / "
    "Drive 준비 실패 시 자동 재시도"
)

# master(1).py 입력 TCP 서버 시작 (9999)
# 첫 번째 Depth는 CREATE_BOX를 Isaac:9999로 직접 전송한다.
# Master 연결 Depth는 좌표/상태만 전달하며 새 박스를 만들지 않는다.
start_yolo_tcp_server()

# 터미널에 네트워크 구성과 전체 자동 공정 알고리즘을 한 번 출력한다.
print_integration_algorithm()

event_log(
    "[MASTER 호환] v51.1 FIRST_DEPTH_CREATE_BOX_ONLY | "
    "CREATE_BOX=박스 생성 | VISION_PACKET=상태 저장만 | "
    "INSPECTION_RESULT=기존 박스 판정 | 첫 생성 박스 Z=500=트레이로더"
)

# Digital Twin 상태/명령 TCP 서버 시작 (선택 10000)
start_digital_twin_server()

event_log(
    "[TWIN 동기화] v48 양방향 트윈 준비 | "
    f"입력={YOLO_TCP_PORT} | Twin={TWIN_TCP_PORT} | "
    f"상태주기={TWIN_STATE_SEND_HZ:.1f}Hz | "
    f"STM좌표모드={digital_twin_sync_options['stm_coordinate_mode']}"
)

event_log(
    "[Pivot 안전 패치 적용] "
    "선택 Prim 해제 후 지연 삭제 방식 사용"
)

event_log(
    "[컨베이어 연속가동 패치 적용] "
    "PLAY 중 계속 가동 / 지정 좌표 도착 시에만 자동 정지"
)

event_log(
    "[3축 v7 직접 Drive 패치 적용] "
    "성공 코드 방식 targetPosition 직접 제어 + 상대 Geometry 완료판정"
)

event_log(
    "[그리퍼 v7 No-Jump 패치 적용] "
    "RigidBody 비활성화 후 그리퍼 이동량 추종"
)

event_log(
    "[시작 정리 v9 패치 적용] "
    "코드 실행 전에 존재하던 박스만 삭제 / "
    "실행 후 새로 생성한 박스는 삭제하지 않음"
)

event_log(
    "[3축 v10 복구 패치 적용] "
    "Geometry 완료판정 차단 제거 / "
    "Drive targetPosition S-Curve 직접 제어"
)

event_log(
    "[3축 시작 v12 패치 적용] "
    "PLAY 중 코드 실행 지원 / 초기 HOME 자동 재시도 / "
    "모션 시작 실패 시 상태 잠김 방지"
)

event_log(
    "[불량 처리 v22 패치 적용] "
    "불량 판정 즉시 빨간색 변경 | "
    f"Box Z={REJECTOR_TRIGGER_Z:.5f} 도착 시 "
    f"리젝터 {REJECTOR_HOME_VALUE:.1f}도 "
    f"→ {REJECTOR_PUSH_VALUE:.1f}도 "
    f"→ {REJECTOR_HOME_VALUE:.1f}도 즉시 실행"
)

event_log(
    "[Depth 좌표 v17 패치 적용] "
    "position.x → Isaac X | "
    "position.y → Isaac Z | "
    f"Isaac Y={BOX_SPAWN_POS[1]:.3f} 고정 | "
    "position.z는 위치 계산에서 제외"
)

event_log(
    "[통합 시스템 로드 완료] "
    f"검사 컨베이어={CONVEYOR_PATH} | "
    f"속도={CONVEYOR_RUN_X_SPEED:.1f} | "
    f"박스 정지 Z={STOP_TARGET_Z:.1f}"
)


event_log(
    f"[트레이 이동] v43 감지 준비 | "
    f"첫 번째 생성 박스 Z={TRAY_LOADER_TRIGGER_Z:.5f} 이하 또는 통과 시 "
    "판정과 관계없이 트레이로더 1회 시작"
)

event_log(
    "[3축 설정] "
    f"HOME=({AUTO_HOME_X1_POSITION:.1f}, "
    f"{AUTO_HOME_X2_POSITION:.1f}, "
    f"{AUTO_HOME_Y_POSITION:.1f}, "
    f"{AUTO_HOME_Z_POSITION:.1f}) | "
    f"PICK=({AUTO_PICK_X1_POSITION:.1f}, "
    f"{AUTO_PICK_X2_POSITION:.1f}, "
    f"Y command {AUTO_PICK_Y_COMMAND_POSITION:.1f}, "
    f"Y actual {AUTO_PICK_Y_REACHED_POSITION:.1f}, "
    f"Y up {AUTO_PICK_Y_UP_POSITION:.1f}, "
    f"Post-pick Z {AUTO_POST_PICK_Z_POSITION:.1f}, "
    f"Next Box Delay {NEXT_BOX_FEED_START_DELAY_SEC:.1f}s, "
    f"Z {AUTO_PICK_Z_POSITION:.1f})"
)

event_log(
    "[1차 적재 설정] "
    f"X1={AUTO_PLACE_X1_POSITION:.1f}, "
    f"X2={AUTO_PLACE_X2_POSITION:.1f}, "
    f"Y={AUTO_PLACE_Y_POSITION:.1f}, "
    f"Z={AUTO_PLACE_Z_POSITION:.1f}"
)

prepare_gripper_joint_drive(
    emit_log=True
)

event_log(
    "[Depth X + 강력 안정 흡착 v31 적용] "
    f"기준 Box X={PICK_REFERENCE_BOX_WORLD_X:.3f} | "
    f"기준 X1/X2=({AUTO_PICK_X1_POSITION:.1f}, "
    f"{AUTO_PICK_X2_POSITION:.1f}) | "
    f"Scale={PICK_WORLD_X_TO_JOINT_SCALE:.3f} | "
    f"Sign={PICK_WORLD_X_TO_JOINT_SIGN:.1f} | "
    f"CarryStep={CARRY_AXIS_MAX_STEP} | "
    f"GripDrive K={GRIPPER_DRIVE_STIFFNESS:.0f}, "
    f"D={GRIPPER_DRIVE_DAMPING:.0f}, "
    f"MaxForce={GRIPPER_DRIVE_MAX_FORCE:.0f}"
)

event_log(
    "[실제 Revolute Joint 그리퍼 v30 적용] "
    f"Joint={GRIPPER_REVOLUTE_JOINT_PATH} | "
    f"흡착 기준 후보={SUCTION_CENTER_PATH_CANDIDATES} | "
    f"허용 수직 간격={MAX_VERTICAL_GAP:.1f} | "
    f"방향 부호={GRIPPER_JOINT_DIRECTION_SIGN:.1f} | "
    "force=False | 흡착 순간 위치보정 없음 | "
    "박스 yaw 맞춤 → 흡착 → 적재 위치에서 0도 복귀 → 해제"
)


# ============================================================
# [v35] 포장 컨베이어 지연 시작 + 이동 Tray1 추적 + Z 기준 위치 복구
# - Timeline PLAY 직후에는 속도 0
# - 검사 컨베이어 위 박스 World Z가 250에 도달하면 +50 시작
# - 시작 후에는 자동 정지하지 않고 계속 이동
# - Tray1 World XYZ는 매 프레임 저장하며 로봇 추종 컨트롤러가 사용
# - 3번 키: 수동 시작 / 0번 키: 수동 정지
# ============================================================

is_pack_conveyor_running = False
pack_start_triggered = False
pack_manual_stop_latched = False
pack_start_waiting_for_tray = False
pack_last_box_z_dict = {}
last_tray_world_position = None
pack_tray_reference_world_position = None


def find_pack_conveyor_velocity_attr():
    prim = get_prim(PACK_CONVEYOR_PATH)

    if prim is None:
        event_log(
            f"[포장 컨베이어 오류] 경로 없음 | {PACK_CONVEYOR_PATH}"
        )
        return None

    attr = find_attr_by_candidates(
        prim,
        CONVEYOR_ATTR_CANDIDATES
    )

    if attr is not None:
        return attr

    for candidate_attr in prim.GetAttributes():
        name = candidate_attr.GetName()
        lower_name = name.lower()

        if (
            "surface" in lower_name
            and "velocity" in lower_name
            and "angular" not in lower_name
            and "enabled" not in lower_name
            and "local" not in lower_name
        ):
            value = candidate_attr.Get()

            if isinstance(value, (Gf.Vec3f, Gf.Vec3d)):
                return candidate_attr

    return None


def find_pack_enable_attrs():
    prim = get_prim(PACK_CONVEYOR_PATH)

    if prim is None:
        return []

    result = []

    for attr_name in CONVEYOR_ENABLE_ATTR_CANDIDATES:
        attr = prim.GetAttribute(attr_name)

        if attr.IsValid():
            result.append(attr)

    for attr in prim.GetAttributes():
        lower_name = attr.GetName().lower()

        if (
            "surface" in lower_name
            and "velocity" in lower_name
            and "enabled" in lower_name
            and attr not in result
        ):
            result.append(attr)

    return result


def find_pack_local_attrs():
    prim = get_prim(PACK_CONVEYOR_PATH)

    if prim is None:
        return []

    result = []

    for attr_name in CONVEYOR_LOCAL_SPACE_ATTR_CANDIDATES:
        attr = prim.GetAttribute(attr_name)

        if attr.IsValid():
            result.append(attr)

    for attr in prim.GetAttributes():
        lower_name = attr.GetName().lower()

        if (
            "surface" in lower_name
            and "velocity" in lower_name
            and "local" in lower_name
            and attr not in result
        ):
            result.append(attr)

    return result


def set_pack_conveyor_speed(speed):
    global is_pack_conveyor_running

    attr = find_pack_conveyor_velocity_attr()

    if attr is None:
        is_pack_conveyor_running = False
        return False

    current_value = attr.Get()

    try:
        if isinstance(current_value, Gf.Vec3d):
            attr.Set(Gf.Vec3d(float(speed), 0.0, 0.0))
        elif isinstance(current_value, Gf.Vec3f):
            attr.Set(Gf.Vec3f(float(speed), 0.0, 0.0))
        else:
            attr.Set(float(speed))
    except Exception as error:
        event_log(
            f"[포장 컨베이어 오류] 속도 설정 실패 | {error}"
        )
        is_pack_conveyor_running = False
        return False

    for local_attr in find_pack_local_attrs():
        try:
            local_attr.Set(True)
        except Exception:
            pass

    for enable_attr in find_pack_enable_attrs():
        try:
            enable_attr.Set(True)
        except Exception:
            pass

    is_pack_conveyor_running = abs(float(speed)) > 0.001
    return True


def wake_tray_body():
    prim = get_prim(TRAY_PATH)

    if prim is None:
        event_log(f"[Tray 오류] 경로 없음 | {TRAY_PATH}")
        return False

    try:
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body = UsdPhysics.RigidBodyAPI(prim)
            rigid_body.CreateRigidBodyEnabledAttr().Set(True)
            rigid_body.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            rigid_body.CreateAngularVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        return True
    except Exception as error:
        event_log(f"[Tray 오류] 물리 깨우기 실패 | {error}")
        return False


def start_pack_conveyor(reason="자동 시작"):
    global pack_start_triggered
    global pack_manual_stop_latched

    wake_tray_body()

    if not set_pack_conveyor_speed(PACK_CONVEYOR_RUN_SPEED):
        return False

    pack_start_triggered = True
    pack_manual_stop_latched = False

    event_log(
        f"[트레이 이동] 포장 컨베이어 시작 | 속도={PACK_CONVEYOR_RUN_SPEED:.1f} | {reason}"
    )

    return True


def restart_pack_conveyor():
    return start_pack_conveyor("3번 키 수동 시작")


def stop_pack_conveyor(reason="수동 정지", latch_manual=False):
    global pack_manual_stop_latched

    if not set_pack_conveyor_speed(PACK_CONVEYOR_STOP_SPEED):
        return False

    if latch_manual:
        pack_manual_stop_latched = True

    event_log(
        f"[트레이 이동] 포장 컨베이어 정지 | 속도={PACK_CONVEYOR_STOP_SPEED:.1f} | {reason}"
    )

    return True


def box_reached_pack_start_z(previous_z, current_z):
    current_z = float(current_z)

    if abs(current_z - float(PACK_START_TRIGGER_Z)) <= float(
        PACK_START_TRIGGER_TOLERANCE_Z
    ):
        return True

    if previous_z is None:
        # 코드를 늦게 실행해 이미 기준을 지난 경우도 즉시 시작한다.
        return current_z <= float(PACK_START_TRIGGER_Z)

    previous_z = float(previous_z)

    return (
        previous_z > float(PACK_START_TRIGGER_Z)
        and current_z <= float(PACK_START_TRIGGER_Z)
    )


def update_pack_start_condition():
    global pack_start_triggered
    global pack_start_waiting_for_tray

    if pack_start_triggered or pack_manual_stop_latched:
        return False

    tray_ready = integrated_tray_loader_controller.tray_ready

    # 박스는 이미 시작 좌표를 지났지만 트레이 공급이 늦은 경우,
    # 트레이가 포장 컨베이어에 올라온 직후 시작한다.
    if pack_start_waiting_for_tray:
        if tray_ready:
            pack_start_waiting_for_tray = False
            return start_pack_conveyor(
                "Box Z 조건 충족 후 트레이 공급 완료"
            )
        return False

    for box_path in get_monitor_box_paths():
        box_position = get_world_position(box_path)

        if box_position is None:
            continue

        current_z = float(box_position[2])
        previous_z = pack_last_box_z_dict.get(box_path, None)
        pack_last_box_z_dict[box_path] = current_z

        if box_reached_pack_start_z(previous_z, current_z):
            if not tray_ready:
                pack_start_waiting_for_tray = True
                tray_loader_log(
                    f"포장 컨베이어 시작 대기 | "
                    f"Box ID={box_path_id_dict.get(box_path, None)} | "
                    f"World Z={current_z:.3f} | 트레이 공급 중"
                )
                return False

            return start_pack_conveyor(
                f"Box ID={box_path_id_dict.get(box_path, None)} "
                f"World Z={current_z:.3f} 도달 "
                f"(기준 {PACK_START_TRIGGER_Z:.1f})"
            )

    return False


def on_pack_update_loop(e):
    global last_tray_world_position

    tray_position = get_world_position(TRAY_PATH)

    if tray_position is not None:
        last_tray_world_position = copy_world_position(tray_position)

    update_pack_start_condition()


def on_pack_keyboard_event(event, *args, **kwargs):
    if event.type != carb.input.KeyboardEventType.KEY_PRESS:
        return True

    if (
        event.input == carb.input.KeyboardInput.KEY_3
        or event.input == carb.input.KeyboardInput.NUMPAD_3
    ):
        restart_pack_conveyor()

    elif (
        event.input == carb.input.KeyboardInput.KEY_0
        or event.input == carb.input.KeyboardInput.NUMPAD_0
    ):
        stop_pack_conveyor("0번 키", latch_manual=True)

    return True


def reset_pack_conveyor_for_play():
    global pack_start_triggered
    global pack_manual_stop_latched
    global pack_start_waiting_for_tray
    global last_tray_world_position
    global pack_tray_reference_world_position

    pack_start_triggered = False
    pack_manual_stop_latched = False
    pack_start_waiting_for_tray = False
    pack_last_box_z_dict.clear()

    tray_position = get_world_position(TRAY_PATH)

    if tray_position is not None:
        last_tray_world_position = copy_world_position(tray_position)
        pack_tray_reference_world_position = copy_world_position(tray_position)
    else:
        last_tray_world_position = None
        pack_tray_reference_world_position = None

    wake_tray_body()
    set_pack_conveyor_speed(PACK_CONVEYOR_STOP_SPEED)

    if pack_tray_reference_world_position is not None:
        event_log(
            f"[Tray 기준 좌표 저장] "
            f"XYZ=({float(pack_tray_reference_world_position[0]):.3f}, "
            f"{float(pack_tray_reference_world_position[1]):.3f}, "
            f"{float(pack_tray_reference_world_position[2]):.3f}) | "
            f"기준 Joint X1/X2/Z=({AUTO_PLACE_X1_POSITION:.1f}, "
            f"{AUTO_PLACE_X2_POSITION:.1f}, "
            f"{AUTO_PLACE_Z_POSITION:.1f})"
        )


def on_pack_timeline_event(e):
    global pack_start_waiting_for_tray

    if e.type == int(omni.timeline.TimelineEventType.PLAY):
        reset_pack_conveyor_for_play()

        event_log(
            f"[Timeline] PLAY | 포장 컨베이어 대기=0 | "
            f"Box World Z={PACK_START_TRIGGER_Z:.1f} 도달 시 "
            f"속도 +{PACK_CONVEYOR_RUN_SPEED:.1f} 시작"
        )

    elif e.type == int(omni.timeline.TimelineEventType.STOP):
        pack_start_waiting_for_tray = False
        set_pack_conveyor_speed(PACK_CONVEYOR_STOP_SPEED)
        pack_last_box_z_dict.clear()


pack_conveyor_kb_sub = (
    input_interface
    .subscribe_to_keyboard_events(
        keyboard,
        on_pack_keyboard_event
    )
)

pack_conveyor_update_sub = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(
        on_pack_update_loop,
        name="assembly714_pack_update_v40"
    )
)

pack_conveyor_timeline_sub = (
    omni.timeline
    .get_timeline_interface()
    .get_timeline_event_stream()
    .create_subscription_to_pop(
        on_pack_timeline_event,
        name="assembly714_pack_timeline_v40"
    )
)

if omni.timeline.get_timeline_interface().is_playing():
    reset_pack_conveyor_for_play()

    event_log(
        f"[포장 컨베이어 대기] Timeline이 이미 PLAY | "
        f"현재 속도=0 | Box Z={PACK_START_TRIGGER_Z:.1f} 대기"
    )
else:
    set_pack_conveyor_speed(PACK_CONVEYOR_STOP_SPEED)


event_log(
    "[v36 이동 트레이 최종 흡착 해제 로드 완료] "
    f"검사속도={CONVEYOR_RUN_X_SPEED:.1f} | "
    f"포장속도={PACK_CONVEYOR_RUN_SPEED:.1f} | "
    f"포장 시작 Box Z={PACK_START_TRIGGER_Z:.1f} | "
    f"Tray={TRAY_PATH} | "
    f"해제 트리거 이동거리={TRAY_RELEASE_TRIGGER_DISTANCE:.1f} | "
    f"추종 상승거리={TRAY_TRACK_RISE_DISTANCE:.1f} | "
    f"해제고정프레임={TRAY_RELEASE_LOCK_FRAMES_REQUIRED} | "
    f"트레이내부깊이={TRAY_PLACE_INSERT_DEPTH:.1f} | "
    f"다음박스사전대기=True | "
    f"Tray 속도전달={TRAY_RELEASE_MATCH_TRAY_VELOCITY}"
)


event_log(
    "[트레이 이동] v42 통합 완료 | "
    "생성 박스 Z=610 도달 후 트레이로더 시작 | "
    "Upper=-135 -> Push=-170 -> SDF -> Push=30 -> Upper=0"
)

# v51.7 Script Editor 확인용
def first_tray_pick_wait_status():
    status = dict(first_tray_pick_wait_controller)
    status["target_x"] = float(FIRST_TRAY_PICK_WAIT_TARGET_X)
    status["tolerance_x"] = float(FIRST_TRAY_PICK_WAIT_TOLERANCE_X)
    status["coordinate_mode"] = str(FIRST_TRAY_PICK_WAIT_COORDINATE_MODE)
    status["current_tray2_x"] = get_first_tray_pick_wait_x()
    status["robot_state"] = robot_sequence.get("state", "IDLE")
    _SYSTEM_PRINT("[TRAY2 PICK WAIT STATUS]", status, flush=True)
    return status

builtins.first_tray_pick_wait_status = first_tray_pick_wait_status
