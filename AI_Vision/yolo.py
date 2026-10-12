#!/usr/bin/env python3
# 검사 공정 + 리젝터 통신 코드
#
# 동작 순서
# 1. 제품 전체가 검사 ROI 내부에 들어오면 검사 시작
# 2. ROI 내부를 이동하는 동안 불량 검출 결과 누적
# 3. 제품 외곽선이 ROI 출구 직전에 도달하면 최종 결과 전송
# 4. 검사 중 불량이 한 번이라도 확인되면 불량 상태 유지
# 5. 한 제품당 결과는 한 번만 전송

import os
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from collections import deque
import socket
import time


# =====================================================
# CPU 점유 최적화 설정
# =====================================================

# CPU 점유를 낮추려면 30.0 -> 15.0 또는 10.0
# 처리 반응속도를 높이려면 높은 FPS 유지
MAX_PROCESS_FPS = 30.0

CPU_COUNT = os.cpu_count() or 4

TORCH_CPU_THREADS = max(
    1,
    min(4, CPU_COUNT // 2)
)

OPENCV_THREADS = max(
    1,
    min(2, CPU_COUNT // 4)
)

torch.set_num_threads(TORCH_CPU_THREADS)

try:
    torch.set_num_interop_threads(1)
except RuntimeError:
    pass

cv2.setUseOptimized(True)
cv2.setNumThreads(OPENCV_THREADS)


# CUDA 사용 가능 시 GPU 추론
USE_CUDA = torch.cuda.is_available()
YOLO_DEVICE = 0 if USE_CUDA else "cpu"

YOLO_IMGSZ = 640

if USE_CUDA:
    torch.backends.cudnn.benchmark = True

print(
    f"[최적화 설정] "
    f"device={YOLO_DEVICE}, "
    f"torch_threads={TORCH_CPU_THREADS}, "
    f"opencv_threads={OPENCV_THREADS}, "
    f"max_fps={MAX_PROCESS_FPS}"
)


# =====================================================
# YOLO 모델 불러오기
# =====================================================

# defect 판정용 모델
detect_model = YOLO("best.pt")

# ByteTrack 추적용 모델
track_model = YOLO("best.pt")

BOX_CLASS_ID = 0
DEFECT_CLASS_ID = 1


# =====================================================
# 카메라 설정
# =====================================================

cap = cv2.VideoCapture(
    3,
    cv2.CAP_DSHOW
)

if not cap.isOpened():
    raise RuntimeError("웹캠을 열지 못했습니다.")

cap.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    1280
)

cap.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    720
)

cap.set(
    cv2.CAP_PROP_FPS,
    MAX_PROCESS_FPS
)

# 카메라 버퍼에 과거 영상이 쌓이는 것을 최소화
cap.set(
    cv2.CAP_PROP_BUFFERSIZE,
    1
)

# 자동 초점 비활성화
cap.set(
    cv2.CAP_PROP_AUTOFOCUS,
    0
)


# =====================================================
# 웹캠 왜곡 보정 파라미터
# =====================================================

calib = np.load(
    "webcam_calibration.npz"
)

mtx = calib["mtx"]
dist = calib["dist"]

ret, test_frame = cap.read()

if not ret:
    cap.release()
    raise RuntimeError("웹캠 프레임을 읽지 못했습니다.")

H, W = test_frame.shape[:2]

print(
    "검사 실제 해상도:",
    W,
    "x",
    H
)

FRAME_AREA = H * W

new_mtx, calibration_roi = cv2.getOptimalNewCameraMatrix(
    mtx,
    dist,
    (W, H),
    0,
    (W, H)
)

mapx, mapy = cv2.initUndistortRectifyMap(
    mtx,
    dist,
    None,
    new_mtx,
    (W, H),
    cv2.CV_32FC1
)

print("웹캠 왜곡 보정 적용 준비 완료")


# =====================================================
# MASTER TCP 설정
# =====================================================

MASTER_HOST = "127.30.1.18"
MASTER_PORT = 5001

master_sock = None


def connect_master():
    """
    MASTER 서버에 연결한다.

    연결에 실패하면 1초 간격으로 계속 재시도한다.
    """
    global master_sock

    while True:
        try:
            master_sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            master_sock.connect(
                (
                    MASTER_HOST,
                    MASTER_PORT
                )
            )

            print(
                f"[MASTER 연결 완료] "
                f"{MASTER_HOST}:{MASTER_PORT}"
            )

            return

        except OSError as e:
            print(
                f"[MASTER 연결 실패] {e} "
                f"-> 1초 후 재시도"
            )

            try:
                if master_sock is not None:
                    master_sock.close()
            except Exception:
                pass

            master_sock = None

            time.sleep(1)


def send_master_result(value, track_id):
    """
    정상 또는 불량 결과를 MASTER에 전송한다.

    value:
        't' = 정상
        'f' = 불량

    전송 형식:
        YOLO_RESULT,track_id,value
    """
    global master_sock

    msg = (
        f"YOLO_RESULT,"
        f"{track_id},"
        f"{value}\n"
    )

    try:
        master_sock.sendall(
            msg.encode("utf-8")
        )

        print(
            f"MASTER SEND -> "
            f"{msg.strip()}"
        )

    except OSError as e:
        print(
            f"[MASTER 전송 오류] {e} "
            f"-> 재연결 시도"
        )

        try:
            if master_sock is not None:
                master_sock.close()
        except Exception:
            pass

        master_sock = None

        connect_master()

        try:
            master_sock.sendall(
                msg.encode("utf-8")
            )

            print(
                f"MASTER SEND -> "
                f"{msg.strip()}"
            )

        except OSError as e2:
            print(
                f"[MASTER 재전송 실패] "
                f"{e2}"
            )


connect_master()


# =====================================================
# 판정 파라미터
# =====================================================

# YOLO detect에서 box와 defect를 가져오기 위한 최소 신뢰도
INFER_CONF_TH = 0.4

# ByteTrack에서 제품 box를 추적하기 위한 최소 신뢰도
BOX_CONF_TH = 0.3

# 불량 검출 최소 신뢰도
DEFECT_CONF_TH = 0.4


# 화면 표시용 신뢰도
BOX_DISPLAY_CONF_TH = 0.3
DEFECT_DISPLAY_CONF_TH = 0.6


# 제품 box가 화면 전체에서 차지해야 하는 최소 면적 비율
MIN_PRODUCT_AREA_RATIO = 0.10

# 제품 전체가 ROI 내부에서 연속으로 검출되어야 하는 프레임 수
PRODUCT_FRAME_REQUIRED = 2

# 불량으로 최종 판정하기 위한 불량 검출 횟수
DEFECT_HIT_REQUIRED = 1


# ByteTrack ID가 일시적으로 사라져도 상태를 유지할 프레임 수
#
# 30 FPS 기준:
# 15프레임은 최대 약 0.5초 동안 상태를 유지
STALE_TRACK_FRAME_LIMIT = 15


# =====================================================
# defect와 제품 box 연결 파라미터
# =====================================================

# 제품 box가 실제 제품보다 작게 검출될 수 있으므로
# 불량 중심점을 연결할 때 제품 box 영역을 확장한다.
ASSOC_MARGIN_X_RATIO = 0.35
ASSOC_MARGIN_Y_RATIO = 0.25


# =====================================================
# 검사 ROI 설정
# =====================================================

# 1280 x 720 화면 기준
INSPECT_X1 = 0    # 100
INSPECT_Y1 = 0

INSPECT_X2 = 1280    # 850
INSPECT_Y2 = 720


# 제품 외곽선이 ROI 경계에 너무 붙어 있는 경우에는
# 제품 전체가 들어왔다고 판단하지 않기 위한 여유값
FULL_BOX_MARGIN_PX = 10


# =====================================================
# 컨베이어 방향 및 ROI 출구 전송 설정
# =====================================================

# 왼쪽에서 오른쪽으로 이동:
# "LEFT_TO_RIGHT"
#
# 오른쪽에서 왼쪽으로 이동:
# "RIGHT_TO_LEFT"
CONVEYOR_DIRECTION = "RIGHT_TO_LEFT"


# 제품 외곽선이 ROI 출구 경계에서 몇 픽셀 전까지
# 도달하면 전송 준비 위치로 판단할지 설정
#
# 값이 클수록 출구보다 더 일찍 전송
# 값이 작을수록 출구에 더 가까워진 후 전송
EXIT_TRIGGER_MARGIN_PX = 300    # 120 -> 240 + 60


# 바운딩 박스 흔들림에 의한 오작동을 방지하기 위해
# 출구 위치에서 연속으로 확인할 프레임 수
EXIT_TRIGGER_REQUIRED_FRAMES = 2


# =====================================================
# 제품별 상태 저장
# =====================================================

product_states = {}

frame_count = 0


# =====================================================
# 화면 표시용 결과 필터
# =====================================================

def make_plot_results(results):
    """
    화면에 표시할 box와 defect를 신뢰도 기준으로 필터링한다.
    """
    plot_results = results

    if (
        results.boxes is None
        or len(results.boxes) == 0
    ):
        return plot_results

    # detect 결과:
    # [x1, y1, x2, y2, conf, cls]
    box_data = results.boxes.data

    keep_mask_list = []

    for row in box_data:
        conf = float(row[4].item())
        cls_id = int(row[5].item())

        if cls_id == BOX_CLASS_ID:
            keep_mask_list.append(
                conf >= BOX_DISPLAY_CONF_TH
            )

        elif cls_id == DEFECT_CLASS_ID:
            keep_mask_list.append(
                conf >= DEFECT_DISPLAY_CONF_TH
            )

        else:
            keep_mask_list.append(False)

    keep_mask = torch.tensor(
        keep_mask_list,
        dtype=torch.bool,
        device=box_data.device
    )

    filtered_boxes = box_data[
        keep_mask
    ]

    plot_results.update(
        boxes=filtered_boxes
    )

    return plot_results


# =====================================================
# defect와 특정 제품 box 연결
# =====================================================

def defect_related_to_box(
    defect_xyxy,
    box_xyxy
):
    """
    defect 중심점이 확장된 제품 box 내부에 있는지 확인한다.
    """
    dx1, dy1, dx2, dy2 = defect_xyxy
    bx1, by1, bx2, by2 = box_xyxy

    box_width = bx2 - bx1
    box_height = by2 - by1

    # 제품 box 확장
    expanded_x1 = (
        bx1
        - int(
            box_width
            * ASSOC_MARGIN_X_RATIO
        )
    )

    expanded_y1 = (
        by1
        - int(
            box_height
            * ASSOC_MARGIN_Y_RATIO
        )
    )

    expanded_x2 = (
        bx2
        + int(
            box_width
            * ASSOC_MARGIN_X_RATIO
        )
    )

    expanded_y2 = (
        by2
        + int(
            box_height
            * ASSOC_MARGIN_Y_RATIO
        )
    )

    # defect 중심점
    defect_center_x = (
        dx1 + dx2
    ) / 2.0

    defect_center_y = (
        dy1 + dy2
    ) / 2.0

    return (
        expanded_x1
        <= defect_center_x
        <= expanded_x2
        and
        expanded_y1
        <= defect_center_y
        <= expanded_y2
    )


# =====================================================
# ROI 출구 위치 계산
# =====================================================

def get_exit_trigger_position():
    """
    설정된 컨베이어 방향에 따라 ROI 출구 판정 위치를 반환한다.
    """
    if CONVEYOR_DIRECTION == "LEFT_TO_RIGHT":

        exit_boundary = (
            INSPECT_X2
            - FULL_BOX_MARGIN_PX
        )

        exit_trigger_position = (
            exit_boundary
            - EXIT_TRIGGER_MARGIN_PX
        )

    else:

        exit_boundary = (
            INSPECT_X1
            + FULL_BOX_MARGIN_PX
        )

        exit_trigger_position = (
            exit_boundary
            + EXIT_TRIGGER_MARGIN_PX
        )

    return (
        exit_boundary,
        exit_trigger_position
    )


# =====================================================
# 제품이 출구 근처인지 확인
# =====================================================

def check_exit_status(
    box_xyxy,
    box_inside_roi,
    inspection_active
):
    """
    제품 외곽선이 ROI 출구 전송 위치에 도달했는지 확인한다.

    반환값:
        near_exit:
            제품 전체가 ROI 내부에 있으면서
            출구 트리거 위치에 도달한 상태

        passed_exit:
            프레임 누락이나 빠른 이동으로 트리거 구간을 건너뛰고
            ROI 출구 경계를 넘어간 상태
    """
    x1, y1, x2, y2 = box_xyxy

    exit_boundary, exit_trigger_position = (
        get_exit_trigger_position()
    )

    if CONVEYOR_DIRECTION == "LEFT_TO_RIGHT":

        # 제품 오른쪽 외곽선 x2 기준
        near_exit = (
            box_inside_roi
            and
            x2 >= exit_trigger_position
        )

        # x2가 ROI 내부 허용 경계를 넘어가면
        # 트리거 구간을 건너뛴 것으로 판단
        passed_exit = (
            inspection_active
            and
            x2 > exit_boundary
        )

    else:

        # 제품 왼쪽 외곽선 x1 기준
        near_exit = (
            box_inside_roi
            and
            x1 <= exit_trigger_position
        )

        # x1이 ROI 내부 허용 경계를 넘어가면
        # 트리거 구간을 건너뛴 것으로 판단
        passed_exit = (
            inspection_active
            and
            x1 < exit_boundary
        )

    return (
        near_exit,
        passed_exit
    )


# =====================================================
# 메인 검사 루프
# =====================================================

try:
    while True:
        loop_start_time = time.perf_counter()

        ret, frame = cap.read()

        if not ret:
            print("[카메라 오류] 프레임을 읽지 못했습니다.")
            break


        # =================================================
        # [0] 웹캠 렌즈 왜곡 보정
        # =================================================

        frame = cv2.remap(
            frame,
            mapx,
            mapy,
            cv2.INTER_LINEAR
        )

        frame_count += 1

        tracked_boxes = []
        defects = []


        # =================================================
        # [1] YOLO defect 검출
        # =================================================

        with torch.inference_mode():
            detect_results = detect_model(
                frame,
                conf=INFER_CONF_TH,
                iou=0.5,
                imgsz=YOLO_IMGSZ,
                device=YOLO_DEVICE,
                half=False,
                verbose=False
            )[0]


        if (
            detect_results.boxes is not None
            and
            len(detect_results.boxes) > 0
        ):
            for detected_box in detect_results.boxes:

                cls_id = int(
                    detected_box.cls[0]
                )

                conf = float(
                    detected_box.conf[0]
                )

                x1, y1, x2, y2 = map(
                    int,
                    detected_box.xyxy[0]
                )

                if (
                    cls_id == DEFECT_CLASS_ID
                    and
                    conf >= DEFECT_CONF_TH
                ):
                    defects.append(
                        {
                            "conf": conf,
                            "xyxy": (
                                x1,
                                y1,
                                x2,
                                y2
                            )
                        }
                    )


        # =================================================
        # [2] ByteTrack 제품 box 추적
        # =================================================

        with torch.inference_mode():
            track_results = track_model.track(
                frame,
                conf=BOX_CONF_TH,
                iou=0.5,
                persist=True,
                tracker="bytetrack.yaml",
                classes=[BOX_CLASS_ID],
                imgsz=YOLO_IMGSZ,
                device=YOLO_DEVICE,
                half=False,
                verbose=False
            )[0]


        if (
            track_results.boxes is not None
            and
            len(track_results.boxes) > 0
        ):
            for tracked_box in track_results.boxes:

                if tracked_box.id is None:
                    continue

                cls_id = int(
                    tracked_box.cls[0]
                )

                conf = float(
                    tracked_box.conf[0]
                )

                if cls_id != BOX_CLASS_ID:
                    continue

                track_id = int(
                    tracked_box.id[0]
                )

                x1, y1, x2, y2 = map(
                    int,
                    tracked_box.xyxy[0]
                )

                box_width = max(
                    0,
                    x2 - x1
                )

                box_height = max(
                    0,
                    y2 - y1
                )

                box_area = (
                    box_width
                    * box_height
                )

                area_ratio = (
                    box_area
                    / FRAME_AREA
                )


                # 제품 외곽선 전체가 검사 ROI 안에 있는지 확인
                box_inside_inspect_roi = (
                    x1
                    >= INSPECT_X1
                    + FULL_BOX_MARGIN_PX
                    and
                    y1
                    >= INSPECT_Y1
                    + FULL_BOX_MARGIN_PX
                    and
                    x2
                    <= INSPECT_X2
                    - FULL_BOX_MARGIN_PX
                    and
                    y2
                    <= INSPECT_Y2
                    - FULL_BOX_MARGIN_PX
                )


                # 제품 검사에 사용할 수 있는 유효 상태
                has_product_valid = (
                    box_inside_inspect_roi
                    and
                    area_ratio
                    >= MIN_PRODUCT_AREA_RATIO
                    and
                    conf
                    >= BOX_CONF_TH
                )


                tracked_boxes.append(
                    {
                        "track_id": track_id,

                        "xyxy": (
                            x1,
                            y1,
                            x2,
                            y2
                        ),

                        "conf": conf,

                        "area_ratio": area_ratio,

                        "box_inside_roi":
                            box_inside_inspect_roi,

                        "has_product_valid":
                            has_product_valid
                    }
                )


        # =================================================
        # [3] track_id별 정상/불량 판정
        # =================================================

        for box in tracked_boxes:

            track_id = box["track_id"]
            box_xyxy = box["xyxy"]

            box_inside_roi = (
                box["box_inside_roi"]
            )

            has_product_valid = (
                box["has_product_valid"]
            )


            # ---------------------------------------------
            # 새로운 제품 상태 생성
            # ---------------------------------------------

            if track_id not in product_states:

                product_states[track_id] = {

                    # 제품 전체가 ROI 안에 연속으로 들어왔는지 확인
                    "product_history": deque(
                        maxlen=PRODUCT_FRAME_REQUIRED
                    ),

                    # 제품 검사가 시작되었는지
                    "inspection_active": False,

                    # 불량이 한 번이라도 확정되었는지
                    "defect_latched": False,

                    # 불량 검출 누적 횟수
                    "defect_hits": 0,

                    # 출구 근처 연속 검출 횟수
                    "exit_zone_frames": 0,

                    # 결과 전송 완료 여부
                    "sent": False,

                    # 마지막으로 제품이 검출된 프레임
                    "last_seen": frame_count
                }


            state = product_states[track_id]

            state["last_seen"] = frame_count


            # 이미 결과를 전송한 제품은 추가 판정하지 않음
            if state["sent"]:
                continue


            # ---------------------------------------------
            # 제품 전체 ROI 진입 확인
            # ---------------------------------------------

            state["product_history"].append(
                has_product_valid
            )

            product_stable = (
                len(
                    state["product_history"]
                )
                == PRODUCT_FRAME_REQUIRED
                and
                all(
                    state["product_history"]
                )
            )


            # 제품 전체가 ROI 내부에서 연속 검출되면 검사 시작
            if not state["inspection_active"]:

                if product_stable:

                    state["inspection_active"] = True

                    print(
                        f"[검사 시작] "
                        f"ID:{track_id}"
                    )

                else:
                    continue


            # ---------------------------------------------
            # ROI 내부 이동 중 불량 판정 누적
            # ---------------------------------------------

            if has_product_valid:

                defect_this_box = any(
                    defect_related_to_box(
                        defect["xyxy"],
                        box_xyxy
                    )
                    for defect in defects
                )


                if defect_this_box:

                    state["defect_hits"] += 1

                    print(
                        f"[불량 검출] "
                        f"ID:{track_id}, "
                        f"hits:"
                        f"{state['defect_hits']}"
                    )


                    # 필요한 횟수만큼 불량이 검출되면
                    # 이후 불량 검출이 사라져도 불량 상태 유지
                    if (
                        state["defect_hits"]
                        >= DEFECT_HIT_REQUIRED
                    ):
                        state["defect_latched"] = True


            # ---------------------------------------------
            # ROI 출구 접근 여부 확인
            # ---------------------------------------------

            near_exit, passed_exit = (
                check_exit_status(
                    box_xyxy=box_xyxy,
                    box_inside_roi=box_inside_roi,
                    inspection_active=state[
                        "inspection_active"
                    ]
                )
            )


            if near_exit:

                state["exit_zone_frames"] += 1

            else:

                state["exit_zone_frames"] = 0


            # 출구 근처가 연속으로 확인되었거나
            # 프레임 사이에 ROI 경계를 넘어간 경우 전송
            send_position_reached = (
                state["exit_zone_frames"]
                >= EXIT_TRIGGER_REQUIRED_FRAMES
                or
                passed_exit
            )


            # ---------------------------------------------
            # ROI 출구에서 최종 판정 전송
            # ---------------------------------------------

            if send_position_reached:

                if state["defect_latched"]:

                    send_master_result(
                        "f",
                        track_id
                    )

                    print(
                        f"[출구 최종 판정] "
                        f"ID:{track_id}, "
                        f"결과:불량, "
                        f"defect_hits:"
                        f"{state['defect_hits']}"
                    )

                else:

                    send_master_result(
                        "t",
                        track_id
                    )

                    print(
                        f"[출구 최종 판정] "
                        f"ID:{track_id}, "
                        f"결과:정상"
                    )


                # 한 제품당 한 번만 전송
                state["sent"] = True


        # =================================================
        # [4] 오래 보이지 않는 track_id 삭제
        # =================================================

        for track_id in list(
            product_states.keys()
        ):

            unseen_frames = (
                frame_count
                - product_states[
                    track_id
                ]["last_seen"]
            )

            if (
                unseen_frames
                > STALE_TRACK_FRAME_LIMIT
            ):

                deleted_state = (
                    product_states[
                        track_id
                    ]
                )

                # 검사는 시작했지만 출구 전송 전에
                # ID가 완전히 소실된 경우 경고 출력
                if (
                    deleted_state[
                        "inspection_active"
                    ]
                    and
                    not deleted_state["sent"]
                ):
                    print(
                        f"[TRACK 삭제 경고] "
                        f"ID:{track_id}, "
                        f"검사 시작 후 결과 전송 전에 "
                        f"추적이 소실되었습니다."
                    )

                del product_states[
                    track_id
                ]


        # =================================================
        # [5] 화면 표시
        # =================================================

        plot_results = make_plot_results(
            detect_results
        )

        vis = plot_results.plot()


        # 검사 ROI는 판정에만 사용하며 화면에는 표시하지 않음

        # 출구 전송 기준선 표시
        exit_boundary, exit_trigger_position = (
            get_exit_trigger_position()
        )

        cv2.line(
            vis,
            (
                exit_trigger_position,
                INSPECT_Y1
            ),
            (
                exit_trigger_position,
                INSPECT_Y2
            ),
            (
                255,
                255,
                0
            ),
            2
        )


        cv2.imshow(
            "Final Inspection Pipeline",
            vis
        )


        # ESC 키를 누르면 종료
        if (
            cv2.waitKey(1)
            & 0xFF
            == 27
        ):
            break


        # =================================================
        # [6] 처리 FPS 제한
        # =================================================

        if MAX_PROCESS_FPS > 0:

            target_period = (
                1.0
                / MAX_PROCESS_FPS
            )

            elapsed = (
                time.perf_counter()
                - loop_start_time
            )

            remain = (
                target_period
                - elapsed
            )

            if remain > 0:
                time.sleep(remain)


finally:

    cap.release()

    if master_sock is not None:
        try:
            master_sock.close()
        except Exception:
            pass

    cv2.destroyAllWindows()