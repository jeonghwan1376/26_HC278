from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import numpy as np

# =========================================================
# 전체 공정 웹캠 설정
# =========================================================
AUTO_FIND_CAMERA = False

CAMERA_INDEX = 0
CAMERA_SEARCH_MAX_INDEX = 5
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_FPS = 30
AUTOFOCUS = False
SHOW_LOCAL_PREVIEW = True

STREAM_BIND_HOST = "0.0.0.0"
STREAM_PORT = 5100
STREAM_FPS = 20
JPEG_QUALITY = 82

PREVIEW_WINDOW_NAME = "Full Process Camera - ESC to close"

cv2.setUseOptimized(True)
cv2.setNumThreads(1)


class FrameStore:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: Optional[bytes] = None
        self.sequence = 0
        self.camera_index: Optional[int] = None
        self.running = True
        self.last_frame_time = 0.0

    def update(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.sequence += 1
            self.last_frame_time = time.time()
            self.condition.notify_all()

    def wait_next(
        self,
        previous_sequence: int,
        timeout: float = 2.0,
    ) -> tuple[int, Optional[bytes]]:
        with self.condition:
            if self.sequence == previous_sequence:
                self.condition.wait(timeout)
            return self.sequence, self.jpeg


STORE = FrameStore()


def try_open_camera(camera_index: int) -> Optional[cv2.VideoCapture]:
    backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]

    for backend in backends:
        cap = cv2.VideoCapture(camera_index, backend)

        if not cap.isOpened():
            cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if AUTOFOCUS else 0)
        except Exception:
            pass

        for _ in range(10):
            ok, frame = cap.read()
            if ok and frame is not None and frame.size > 0:
                backend_name = {
                    cv2.CAP_DSHOW: "DSHOW",
                    cv2.CAP_MSMF: "MSMF",
                    cv2.CAP_ANY: "AUTO",
                }.get(backend, str(backend))

                print(
                    f"[카메라 연결 완료] index={camera_index}, "
                    f"backend={backend_name}"
                )
                return cap

            time.sleep(0.05)

        cap.release()

    return None


def open_camera() -> tuple[cv2.VideoCapture, int]:
    if not AUTO_FIND_CAMERA:
        cap = try_open_camera(CAMERA_INDEX)
        if cap is None:
            raise RuntimeError(f"카메라 {CAMERA_INDEX}번을 열지 못했습니다.")
        return cap, CAMERA_INDEX

    print(f"[카메라 자동 검색] 0~{CAMERA_SEARCH_MAX_INDEX}번 확인")

    for camera_index in range(CAMERA_SEARCH_MAX_INDEX + 1):
        print(f"[카메라 확인] index={camera_index}")
        cap = try_open_camera(camera_index)
        if cap is not None:
            return cap, camera_index

    raise RuntimeError(
        "사용 가능한 웹캠을 찾지 못했습니다. "
        "카메라 앱, Zoom, Teams 등을 종료한 뒤 다시 실행하세요."
    )


def fit_to_canvas(frame: np.ndarray) -> np.ndarray:
    source_height, source_width = frame.shape[:2]

    scale = min(
        FRAME_WIDTH / max(source_width, 1),
        FRAME_HEIGHT / max(source_height, 1),
    )

    target_width = max(1, int(round(source_width * scale)))
    target_height = max(1, int(round(source_height * scale)))

    resized = cv2.resize(
        frame,
        (target_width, target_height),
        interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR,
    )

    canvas = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    x_offset = (FRAME_WIDTH - target_width) // 2
    y_offset = (FRAME_HEIGHT - target_height) // 2
    canvas[
        y_offset:y_offset + target_height,
        x_offset:x_offset + target_width,
    ] = resized
    return canvas


def local_ipv4_addresses() -> list[str]:
    addresses: set[str] = set()

    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            address = info[4][0]
            if not address.startswith("127."):
                addresses.add(address)
    except OSError:
        pass

    return sorted(addresses)


class StreamHandler(BaseHTTPRequestHandler):
    server_version = "FullProcessCamera/1.0"

    def do_GET(self) -> None:
        if self.path in ("/", "/status"):
            payload = json.dumps(
                {
                    "ok": True,
                    "camera_index": STORE.camera_index,
                    "video": "/video",
                    "last_frame_time": STORE.last_frame_time,
                },
                ensure_ascii=False,
            ).encode("utf-8")

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
            return

        if not self.path.startswith("/video"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()

        sequence = -1

        try:
            while STORE.running:
                sequence, jpeg = STORE.wait_next(sequence)
                if jpeg is None:
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                )
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()

        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def log_message(self, _format: str, *_args) -> None:
        return


def capture_loop(cap: cv2.VideoCapture) -> None:
    target_period = 1.0 / max(1, STREAM_FPS)

    while STORE.running:
        started_at = time.perf_counter()
        ok, frame = cap.read()

        if not ok or frame is None:
            print("[카메라 오류] 프레임을 읽지 못했습니다.")
            time.sleep(0.1)
            continue

        canvas = fit_to_canvas(frame)
        ok_encode, encoded = cv2.imencode(
            ".jpg",
            canvas,
            [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
        )

        if ok_encode:
            STORE.update(encoded.tobytes())

        if SHOW_LOCAL_PREVIEW:
            cv2.imshow(PREVIEW_WINDOW_NAME, canvas)
            if (cv2.waitKey(1) & 0xFF) == 27:
                STORE.running = False
                break

        elapsed = time.perf_counter() - started_at
        remain = target_period - elapsed
        if remain > 0:
            time.sleep(remain)


def main() -> None:
    cap, camera_index = open_camera()
    STORE.camera_index = camera_index

    server = ThreadingHTTPServer(
        (STREAM_BIND_HOST, STREAM_PORT),
        StreamHandler,
    )
    server.daemon_threads = True

    server_thread = threading.Thread(
        target=server.serve_forever,
        kwargs={"poll_interval": 0.5},
        daemon=True,
        name="full-process-stream-server",
    )
    server_thread.start()

    print("=" * 66)
    print("전체 공정 웹캠 송출 시작")
    print(f"카메라 번호       : {camera_index}")
    print(f"통합 GUI 연결 주소: http://127.0.0.1:{STREAM_PORT}/video")
    print(f"브라우저 확인     : http://127.0.0.1:{STREAM_PORT}/video")

    for address in local_ipv4_addresses():
        print(f"다른 PC 확인      : http://{address}:{STREAM_PORT}/video")

    print("종료              : 미리보기 창에서 ESC")
    print("=" * 66)

    try:
        capture_loop(cap)
    except KeyboardInterrupt:
        print("\n[종료 요청]")
    finally:
        STORE.running = False
        try:
            server.shutdown()
            server.server_close()
        except Exception:
            pass
        cap.release()
        cv2.destroyAllWindows()
        print("[전체 공정 웹캠 종료]")


if __name__ == "__main__":
    main()
