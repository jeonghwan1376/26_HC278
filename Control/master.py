# master_bridge_openrb_refactored.py
# MCU UART 좌표 트리거('0') + 동작 신호(1~7) + blackbox TCP + OpenRB UART 통합 master

import json
import os
import queue
import re
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional, Tuple

try:
    import serial
except ImportError:
    serial = None

# =========================================================
# TCP 설정: blackbox.py 전용
# =========================================================
HOST = "127.30.1.18"
TCP_PORT = 5000
YOLO_TCP_PORT = 5001

BACKLOG = 1
RECV_SIZE = 1024
SOCKET_TIMEOUT_SEC = 1.0

# =========================================================
# GUI 텔레메트리 TCP 설정
# =========================================================
GUI_TELEMETRY_ENABLE = True
GUI_HOST = "172.30.1.20"
GUI_PORT = 5101
GUI_RECONNECT_INTERVAL_SEC = 1.0
GUI_CONNECT_TIMEOUT_SEC = 2.0
GUI_QUEUE_SIZE = 200
GUI_HEARTBEAT_INTERVAL_SEC = 1.0

# =========================================================
# Isaac Sim 전체 상태 송신 설정
# =========================================================
ISAAC_FORWARD_ENABLE = True
ISAAC_HOST = os.getenv("ISAAC_HOST", "172.30.1.64")
ISAAC_PORT = int(os.getenv("ISAAC_PORT", "9999"))

ISAAC_RECONNECT_INTERVAL_SEC = 1.0
ISAAC_CONNECT_TIMEOUT_SEC = 2.0
ISAAC_ACK_TIMEOUT_SEC = 0.10
ISAAC_QUEUE_SIZE = 500

# =========================================================
# MCU STEP별 GUI 컨베이어 표시값 매핑
# =========================================================
GUI_CONVEYOR_STATE_MAP = {
    1: (True, True),   # 대기 상태
    2: (False, True),  # 로봇 이동 중
    3: (False, True),  # 집기 동작 중
    4: (True, True),   # 집기 완료(흡착 완료) -> 검사 컨베이어 재구동
    5: (True, True),   # 적재 이동 중
    6: (True, True),   # 놓기 동작 중
    7: (True, True),   # 복귀 완료
}

# =========================================================
# MCU UART 설정
# =========================================================
MCU_UART_ENABLE = True
MCU_UART_PORT = "COM18"
MCU_UART_BAUD = 57600
MCU_UART_TIMEOUT = 0.05

# =========================================================
# OpenRB UART 설정
# =========================================================
OPENRB_ENABLE = True
OPENRB_PORT = "COM19"
OPENRB_BAUD = 1000000
OPENRB_UART_TIMEOUT = 0.05

OPENRB_USE_DEADBAND = True
OPENRB_ANGLE_DEADBAND_DEG = 1.0
OPENRB_MIN_SEND_INTERVAL_SEC = 0.02

# =========================================================
# 출력 설정
# =========================================================
PRINT_EVERY_PACKET = True
PRINT_EVERY_ANGLE = True

SHOW_HEARTBEAT = False
HEARTBEAT_INTERVAL_SEC = 3.0

PRINT_MCU_RAW_EVERY_TIME = False
PRINT_ROBOT_STATE_ONLY_ON_CHANGE = True
PRINT_MCU_RESPONSE_ONLY_ON_CHANGE = True

FORWARD_STATE_TO_BLACKBOX_ONLY_ON_CHANGE = True
RESET_OPENRB_ZERO_ONLY_ON_STEP4_ENTER = True

COMMAND_SET = {"0", "1", "2", "3", "4", "5", "6", "7", "IRUN", "ISTOP"}
ROBOT_STATE_SET = {"1", "2", "3", "4", "5", "6", "7"}
MCU_RESPONSE_SET = {"ACK", "BUSY", "LBUSY", "LDONE"}
MCU_INSPECTION_CONVEYOR_STATE_SET = {"IRUN", "ISTOP"}

NUMBER_RE = re.compile(r"[-+]?\d+(?:\.\d+)?")
ANGLE_NAME_RE = re.compile(
    r"(?:ANGLE|YAW|DEG|THETA|A)\s*[:=]\s*([-+]?\d+(?:\.\d+)?)",
    flags=re.IGNORECASE,
)


@dataclass
class VisionPacket:
    x: float
    y: float
    z: float = 0.0
    angle: Optional[float] = None
    raw: str = ""
    received_at: float = 0.0


@dataclass
class YoloResultPacket:
    track_id: int
    result: str
    label: str
    raw: str = ""
    received_at: float = 0.0


class EventLogger:
    def __init__(self) -> None:
        self._once_keys = set()
        self._last_values = {}
        self._lock = threading.Lock()

    def log_once(self, key: str, message: str) -> None:
        with self._lock:
            if key in self._once_keys:
                return
            self._once_keys.add(key)
        print(message)

    def log_on_change(self, key: str, value, message: str) -> bool:
        with self._lock:
            if self._last_values.get(key) == value:
                return False
            self._last_values[key] = value
        print(message)
        return True

    def reset_once(self, key: str) -> None:
        with self._lock:
            self._once_keys.discard(key)

    def reset_change(self, key: str) -> None:
        with self._lock:
            self._last_values.pop(key, None)


class GuiTelemetryClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.running = False
        self.connected = False
        self.sock: Optional[socket.socket] = None
        self.sock_lock = threading.Lock()
        self.outbox: queue.Queue = queue.Queue(maxsize=GUI_QUEUE_SIZE)
        self.thread: Optional[threading.Thread] = None
        self.log = EventLogger()

    def start(self) -> None:
        if not GUI_TELEMETRY_ENABLE or self.running:
            return

        self.running = True
        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="gui-telemetry-tx",
        )
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        self._close_socket()

        if self.thread is not None:
            self.thread.join(timeout=0.5)
            self.thread = None

    def is_connected(self) -> bool:
        with self.sock_lock:
            return self.connected and self.sock is not None

    def publish(self, payload: dict[str, Any]) -> None:
        if not GUI_TELEMETRY_ENABLE:
            return

        try:
            self.outbox.put_nowait(payload)
            return
        except queue.Full:
            pass

        try:
            self.outbox.get_nowait()
        except queue.Empty:
            pass

        try:
            self.outbox.put_nowait(payload)
        except queue.Full:
            pass

    def _run(self) -> None:
        pending_payload: Optional[dict[str, Any]] = None

        while self.running:
            if not self.is_connected():
                if not self._connect():
                    time.sleep(GUI_RECONNECT_INTERVAL_SEC)
                    continue

            if pending_payload is None:
                try:
                    pending_payload = self.outbox.get(timeout=0.2)
                except queue.Empty:
                    continue

            try:
                message = (
                    json.dumps(
                        pending_payload,
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                    + "\n"
                )

                with self.sock_lock:
                    sock = self.sock

                if sock is None:
                    raise ConnectionError("GUI socket is not connected")

                sock.sendall(message.encode("utf-8"))
                pending_payload = None

            except OSError as exc:
                self.log.log_on_change(
                    "gui_send_error",
                    str(exc),
                    f"[GUI TCP 전송 오류] {exc}",
                )
                self._close_socket()
                time.sleep(GUI_RECONNECT_INTERVAL_SEC)

    def _connect(self) -> bool:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(GUI_CONNECT_TIMEOUT_SEC)
            sock.connect((self.host, self.port))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(None)

            with self.sock_lock:
                self.sock = sock
                self.connected = True

            self.log.reset_once("gui_connect_wait")
            self.log.reset_change("gui_send_error")
            print(f"[GUI TCP 연결 완료] {self.host}:{self.port}")
            return True

        except OSError:
            self._close_socket()
            self.log.log_once(
                "gui_connect_wait",
                f"[GUI TCP 연결 대기] {self.host}:{self.port}",
            )
            return False

    def _close_socket(self) -> None:
        with self.sock_lock:
            sock = self.sock
            self.sock = None
            self.connected = False

        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


class IsaacSimClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.running = False
        self.connected = False
        self.sock: Optional[socket.socket] = None
        self.sock_lock = threading.Lock()
        self.outbox: queue.Queue = queue.Queue(maxsize=ISAAC_QUEUE_SIZE)
        self.thread: Optional[threading.Thread] = None
        self.log = EventLogger()

    def start(self) -> None:
        if not ISAAC_FORWARD_ENABLE or self.running:
            return

        self.running = True
        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="isaac-sim-tcp-tx",
        )
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        self._close_socket()

        if self.thread is not None:
            self.thread.join(timeout=0.5)
            self.thread = None

    def is_connected(self) -> bool:
        with self.sock_lock:
            return self.connected and self.sock is not None

    def publish(self, payload: dict[str, Any]) -> bool:
        if not ISAAC_FORWARD_ENABLE:
            return False

        try:
            self.outbox.put_nowait(payload)
            self.log.reset_once("isaac_queue_full")
            return True
        except queue.Full:
            self.log.log_once(
                "isaac_queue_full",
                "[ISAAC TCP 큐 가득 참] 메시지를 추가하지 못했습니다.",
            )
            return False

    def _run(self) -> None:
        pending_payload: Optional[dict[str, Any]] = None

        while self.running:
            if not self.is_connected():
                if not self._connect():
                    time.sleep(ISAAC_RECONNECT_INTERVAL_SEC)
                    continue

            if pending_payload is None:
                try:
                    pending_payload = self.outbox.get(timeout=0.2)
                except queue.Empty:
                    continue

            try:
                message = (
                    json.dumps(
                        pending_payload,
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                    + "\n"
                )

                with self.sock_lock:
                    sock = self.sock

                if sock is None:
                    raise ConnectionError("Isaac Sim socket is not connected")

                sock.sendall(message.encode("utf-8"))
                self._read_ack_if_available(sock)
                self._print_sent_payload(pending_payload)
                pending_payload = None

            except (OSError, ConnectionError) as exc:
                self.log.log_on_change(
                    "isaac_send_error",
                    str(exc),
                    f"[ISAAC TCP 전송 오류] {exc}",
                )
                self._close_socket()
                time.sleep(ISAAC_RECONNECT_INTERVAL_SEC)

    def _connect(self) -> bool:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(ISAAC_CONNECT_TIMEOUT_SEC)
            sock.connect((self.host, self.port))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.settimeout(None)

            with self.sock_lock:
                self.sock = sock
                self.connected = True

            self.log.reset_once("isaac_connect_wait")
            self.log.reset_change("isaac_send_error")

            print(f"[ISAAC TCP 연결 완료] {self.host}:{self.port}")
            return True

        except OSError:
            self._close_socket()
            self.log.log_once(
                "isaac_connect_wait",
                f"[ISAAC TCP 연결 대기] {self.host}:{self.port}",
            )
            return False

    @staticmethod
    def _read_ack_if_available(sock: socket.socket) -> None:
        try:
            sock.settimeout(ISAAC_ACK_TIMEOUT_SEC)
            data = sock.recv(4096)
            if data == b"":
                raise ConnectionError("Isaac Sim이 TCP 연결을 종료했습니다.")
        except socket.timeout:
            pass
        finally:
            try:
                sock.settimeout(None)
            except OSError:
                pass

    @staticmethod
    def _print_sent_payload(payload: dict[str, Any]) -> None:
        cmd = str(payload.get("cmd", ""))
        if cmd == "INSPECTION_RESULT":
            print(
                "[MASTER -> ISAAC] INSPECTION_RESULT "
                f"track_id={payload.get('track_id')} | "
                f"value={payload.get('value')}"
            )
            return

        if cmd == "MASTER_EVENT":
            print(
                "[MASTER -> ISAAC] MASTER_EVENT "
                f"{payload.get('category')}/{payload.get('event')}"
            )
            return

        print(f"[MASTER -> ISAAC] {cmd}")

    def _close_socket(self) -> None:
        with self.sock_lock:
            sock = self.sock
            self.sock = None
            self.connected = False

        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


class MasterBridge:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port

        self.running = False
        self.server_sock: Optional[socket.socket] = None
        self.yolo_server_sock: Optional[socket.socket] = None

        self.blackbox_sock: Optional[socket.socket] = None
        self.blackbox_addr: Optional[Tuple[str, int]] = None
        self.blackbox_send_lock = threading.Lock()

        self.mcu = None
        self.mcu_lock = threading.Lock()

        self.openrb = None
        self.openrb_lock = threading.Lock()
        self.last_openrb_angle: Optional[float] = None
        self.last_openrb_send_time = 0.0
        self.openrb_angle_count = 0
        self.openrb_angle_blocked = False

        self.pending_openrb_angle: Optional[float] = None
        self.pending_openrb_angle_time = 0.0

        self.pending_packet_to_mcu: Optional[VisionPacket] = None
        self.pending_packet_count = 0
        self.mcu_coord_in_progress = False
        self.mcu_coord_request_pending = False
        self.first_coord_allowed = True

        self.state_lock = threading.Lock()
        self.latest_packet: Optional[VisionPacket] = None
        self.latest_angle: Optional[float] = None
        self.latest_angle_time = 0.0
        self.packet_count = 0
        self.sensor_trigger_count = 0
        self.last_robot_state: Optional[int] = None
        self.robot_state_count = 0

        self.log = EventLogger()

        self.latest_yolo_result: Optional[YoloResultPacket] = None
        self.yolo_result_count = 0

        self.gui = GuiTelemetryClient(GUI_HOST, GUI_PORT)
        self.isaac = IsaacSimClient(ISAAC_HOST, ISAAC_PORT)

        self.good_count = 0
        self.defect_count = 0
        self.inspection_conveyor_running: Optional[bool] = None
        self.packaging_conveyor_running: Optional[bool] = True

    def start(self) -> None:
        self.running = True

        self.gui.start()
        self.isaac.start()

        threading.Thread(
            target=self._gui_heartbeat_loop,
            daemon=True,
            name="gui-heartbeat",
        ).start()

        self._open_mcu_uart()
        self._open_openrb_uart()
        self._open_tcp_server()
        self._open_yolo_tcp_server()

        threading.Thread(
            target=self._accept_yolo_loop,
            daemon=True,
            name="yolo-tcp-rx",
        ).start()

        self._print_startup_summary()

        threading.Thread(
            target=self._mcu_uart_rx_loop,
            daemon=True,
            name="mcu-uart-rx",
        ).start()

        if SHOW_HEARTBEAT:
            threading.Thread(
                target=self._heartbeat_loop,
                daemon=True,
                name="heartbeat",
            ).start()

        try:
            self._accept_blackbox_loop()
        except KeyboardInterrupt:
            print("\n[종료 요청] Ctrl+C 감지")
        finally:
            self.stop()

    def stop(self) -> None:
        self.running = False

        self._close_blackbox()
        self._close_socket("server_sock")
        self._close_socket("yolo_server_sock")
        self._close_serial("mcu")
        self._close_serial("openrb")
        self.gui.stop()
        self.isaac.stop()

        print("[MASTER 종료]")

    def _open_tcp_server(self) -> None:
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(BACKLOG)
        self.server_sock.settimeout(SOCKET_TIMEOUT_SEC)

    def _open_yolo_tcp_server(self) -> None:
        self.yolo_server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.yolo_server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.yolo_server_sock.bind((self.host, YOLO_TCP_PORT))
        self.yolo_server_sock.listen(1)
        self.yolo_server_sock.settimeout(SOCKET_TIMEOUT_SEC)

        print(f"[YOLO TCP 서버 시작] {self.host}:{YOLO_TCP_PORT}")

    def _accept_yolo_loop(self) -> None:
        while self.running:
            try:
                client_sock, client_addr = self.yolo_server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            print(f"[YOLO 연결] {client_addr}")
            self._send_master_event_to_isaac(
                "CONNECTION",
                "YOLO_CONNECTED",
                {"host": str(client_addr[0]), "port": int(client_addr[1])},
            )

            threading.Thread(
                target=self._handle_yolo_client,
                args=(client_sock, client_addr),
                daemon=True,
                name="yolo-client",
            ).start()

    def _handle_yolo_client(self, client_sock: socket.socket, client_addr) -> None:
        buffer = ""

        with client_sock:
            client_sock.settimeout(SOCKET_TIMEOUT_SEC)

            while self.running:
                try:
                    data = client_sock.recv(RECV_SIZE)
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    print(f"[YOLO 연결 강제 종료] {client_addr}")
                    self._send_master_event_to_isaac(
                        "CONNECTION",
                        "YOLO_RESET",
                        {"host": str(client_addr[0]), "port": int(client_addr[1])},
                    )
                    break
                except OSError:
                    break

                if not data:
                    print(f"[YOLO 연결 종료] {client_addr}")
                    self._send_master_event_to_isaac(
                        "CONNECTION",
                        "YOLO_DISCONNECTED",
                        {"host": str(client_addr[0]), "port": int(client_addr[1])},
                    )
                    break

                buffer += data.decode("utf-8", errors="ignore")
                buffer = self._consume_lines(buffer, self._handle_yolo_line)

    def _handle_yolo_line(self, line: str) -> None:
        yolo_result = self._parse_yolo_result(line)
        if yolo_result is None:
            print(f"[YOLO 수신 무시] {line}")
            return

        self._handle_yolo_result(yolo_result)

    def _print_startup_summary(self) -> None:
        print(f"[MASTER TCP 서버 시작] {self.host}:{self.port}")
        print(f"[MCU UART] {MCU_UART_PORT}, {MCU_UART_BAUD}bps")
        print(f"[OpenRB UART] {OPENRB_PORT}, {OPENRB_BAUD}bps")
        print("[대기 중] blackbox.py TCP 연결 대기")
        print("[좌표 흐름] MCU '0' -> master -> blackbox -> master -> MCU 'G,X,Y,Z'")
        print("[동작 신호] MCU '1'~'7' -> master 상태 기록, STEP 4 진입 시 OpenRB 0도 초기화")
        print(f"[GUI 텔레메트리] {GUI_HOST}:{GUI_PORT}로 JSON 전송")

        self._send_master_event_to_isaac(
            "SYSTEM",
            "MASTER_STARTED",
            {
                "master_host": self.host,
                "blackbox_port": self.port,
                "yolo_port": YOLO_TCP_PORT,
                "mcu_port": MCU_UART_PORT,
                "mcu_baud": MCU_UART_BAUD,
                "openrb_port": OPENRB_PORT,
                "openrb_baud": OPENRB_BAUD,
            },
        )

    def _close_socket(self, attr_name: str) -> None:
        sock = getattr(self, attr_name, None)
        if sock is None:
            return
        try:
            sock.close()
        except OSError:
            pass
        setattr(self, attr_name, None)

    def _close_serial(self, attr_name: str) -> None:
        port_obj = getattr(self, attr_name, None)
        if port_obj is None:
            return
        try:
            port_obj.close()
        except Exception:
            pass
        setattr(self, attr_name, None)

    def _close_blackbox(self) -> None:
        with self.blackbox_send_lock:
            if self.blackbox_sock is not None:
                try:
                    self.blackbox_sock.close()
                except OSError:
                    pass
            self.blackbox_sock = None
            self.blackbox_addr = None

    def _open_mcu_uart(self) -> None:
        if not MCU_UART_ENABLE:
            print("[MCU UART] 비활성화")
            self._send_master_event_to_isaac("CONNECTION", "MCU_UART_DISABLED", {})
            return

        if serial is None:
            print("[MCU UART 오류] pyserial이 없습니다.")
            return

        try:
            self.mcu = serial.Serial(MCU_UART_PORT, MCU_UART_BAUD, timeout=MCU_UART_TIMEOUT)
            time.sleep(2.0)
            print(f"[MCU UART 연결 완료] {MCU_UART_PORT}")
            self._send_master_event_to_isaac("CONNECTION", "MCU_UART_CONNECTED", {"port": MCU_UART_PORT, "baud": MCU_UART_BAUD})
        except Exception as e:
            self.mcu = None
            print(f"[MCU UART 연결 실패] {e}")
            self._send_master_event_to_isaac("CONNECTION", "MCU_UART_CONNECT_FAILED", {"error": str(e)})

    def _open_openrb_uart(self) -> None:
        if not OPENRB_ENABLE:
            print("[OpenRB UART] 비활성화")
            self._send_master_event_to_isaac("CONNECTION", "OPENRB_UART_DISABLED", {})
            return

        if serial is None:
            print("[OpenRB UART 오류] pyserial이 없습니다.")
            return

        try:
            self.openrb = serial.Serial(OPENRB_PORT, OPENRB_BAUD, timeout=OPENRB_UART_TIMEOUT)
            time.sleep(2.0)
            print(f"[OpenRB UART 연결 완료] {OPENRB_PORT}")
            self._send_master_event_to_isaac("CONNECTION", "OPENRB_UART_CONNECTED", {"port": OPENRB_PORT, "baud": OPENRB_BAUD})
        except Exception as e:
            self.openrb = None
            print(f"[OpenRB UART 연결 실패] {e}")
            self._send_master_event_to_isaac("CONNECTION", "OPENRB_UART_CONNECT_FAILED", {"error": str(e)})

    def _accept_blackbox_loop(self) -> None:
        while self.running:
            try:
                client_sock, client_addr = self.server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            print(f"[Blackbox 연결] {client_addr}")
            self._send_master_event_to_isaac("CONNECTION", "BLACKBOX_CONNECTED", {"host": str(client_addr[0]), "port": int(client_addr[1])})

            with self.blackbox_send_lock:
                if self.blackbox_sock is not None:
                    try:
                        self.blackbox_sock.close()
                    except OSError:
                        pass
                self.blackbox_sock = client_sock
                self.blackbox_addr = client_addr

            self.log.reset_once("blackbox_not_connected")
            self._handle_blackbox(client_sock, client_addr)

    def _handle_blackbox(self, client_sock: socket.socket, client_addr: Tuple[str, int]) -> None:
        buffer = ""

        with client_sock:
            client_sock.settimeout(SOCKET_TIMEOUT_SEC)

            while self.running:
                try:
                    data = client_sock.recv(RECV_SIZE)
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    print(f"[Blackbox 연결 강제 종료] {client_addr}")
                    self._send_master_event_to_isaac("CONNECTION", "BLACKBOX_RESET", {"host": str(client_addr[0]), "port": int(client_addr[1])})
                    break
                except OSError:
                    break

                if not data:
                    print(f"[Blackbox 연결 종료] {client_addr}")
                    self._send_master_event_to_isaac("CONNECTION", "BLACKBOX_DISCONNECTED", {"host": str(client_addr[0]), "port": int(client_addr[1])})
                    break

                buffer += data.decode("utf-8", errors="ignore")
                buffer = self._consume_lines(buffer, self._handle_blackbox_line)

        with self.blackbox_send_lock:
            if self.blackbox_sock is client_sock:
                self.blackbox_sock = None
                self.blackbox_addr = None

    def _send_to_blackbox(self, msg: str) -> bool:
        with self.blackbox_send_lock:
            if self.blackbox_sock is None:
                return False

            try:
                self.blackbox_sock.sendall(msg.encode("utf-8"))
                return True
            except OSError as e:
                print(f"[BLACKBOX TCP 전송 오류] {e}")
                self.blackbox_sock = None
                self.blackbox_addr = None
                return False

    def _handle_blackbox_line(self, line: str) -> None:
        command = line.strip()

        self._send_master_event_to_isaac("BLACKBOX", "RX_RAW", {"value": command}, raw=command)

        # Depth 카메라 정지 명령('p')
        if command.lower() == "p":
            self._send_master_event_to_isaac("DEPTH", "STOP_REQUEST", {"command": "p"}, raw=command)

            if self._send_stop_to_mcu():
                with self.state_lock:
                    self.mcu_coord_request_pending = True

                print("[카메라 좌표 요청 활성화] p 정지 명령 전달 -> 다음 G 좌표 MCU 전송 허용")
                self._set_gui_conveyor_state(inspection=False)
            else:
                print("[오류] MCU p 전송 실패 -> 좌표 전송 요청을 활성화하지 않음")

            return

        yolo_result = self._parse_yolo_result(line)
        if yolo_result is not None:
            self._handle_yolo_result(yolo_result)
            return

        angle_only = self._parse_angle_only(line)
        if angle_only is not None:
            self._send_master_event_to_isaac("DEPTH", "ANGLE_RECEIVED", {"angle": float(angle_only)}, raw=line)

            with self.state_lock:
                self.latest_angle = angle_only
                self.latest_angle_time = time.time()

                if self.openrb_angle_blocked:
                    self.pending_openrb_angle = angle_only
                    self.pending_openrb_angle_time = time.time()
                    print(f"[OpenRB 각도 보류] 제품 적재 전이므로 다음 각도 {angle_only:.2f}도 저장")
                    return

            self._send_angle_to_openrb(angle_only)
            return

        packet = self._parse_vision_packet(line)
        if packet is None:
            print(f"[BLACKBOX -> MASTER] {line}")
            self._send_master_event_to_isaac("BLACKBOX", "UNPARSED_LINE", {"value": line}, raw=line)
            return

        self._handle_vision_packet(packet)

    def _handle_vision_packet(self, packet: VisionPacket) -> None:
        with self.state_lock:
            self.latest_packet = packet
            self.packet_count += 1
            packet_count = self.packet_count

        if PRINT_EVERY_PACKET:
            angle_text = "0.00" if packet.angle is None else f"{packet.angle:.2f}"
            print(f"[좌표 데이터 수신 #{packet_count}] X={packet.x:.1f}, Y={packet.y:.1f}, Z={packet.z:.1f}, ANGLE={angle_text} | raw={packet.raw}")

        self._send_master_event_to_isaac(
            "DEPTH", "VISION_PACKET",
            {"count": int(packet_count), "x": float(packet.x), "y": float(packet.y), "z": float(packet.z), "angle": None if packet.angle is None else float(packet.angle)},
            raw=packet.raw
        )

        # Depth 좌표 도착 시: GUI에 depth_coord 유형으로 송신하여 비어있는 위치/각도 채우기
        self.gui.publish({
            "type": "depth_coord",
            "x": round(packet.x, 1),
            "y": round(packet.y, 1),
            "z": round(packet.z, 1),
            "angle": round(packet.angle, 2) if packet.angle is not None else (round(self.latest_angle, 2) if self.latest_angle else None),
        })

        self._queue_or_send_packet_to_mcu(packet)

    def _forward_yolo_result_to_isaac(self, packet: YoloResultPacket) -> bool:
        if packet.result == "t":
            value = 1
        elif packet.result == "f":
            value = 0
        else:
            return False

        payload = {
            "cmd": "INSPECTION_RESULT",
            "value": value,
            "track_id": int(packet.track_id),
            "label": packet.label.lower(),
            "source": "YOLO_VIA_MASTER",
        }
        try:
            return self.isaac.publish(payload)
        except Exception:
            return False

    def _send_master_event_to_isaac(self, category: str, event: str, data: Optional[dict[str, Any]] = None, raw: Optional[str] = None) -> bool:
        payload = {
            "cmd": "MASTER_EVENT", "source": "MASTER",
            "category": str(category).upper(), "event": str(event).upper(),
            "data": data or {}, "timestamp": time.time(),
        }
        if raw is not None:
            payload["raw"] = str(raw)
        try:
            return self.isaac.publish(payload)
        except Exception:
            return False

    def _is_robot_ready_for_new_coord_unlocked(self) -> bool:
        if self.mcu is None or self.mcu_coord_in_progress or not self.mcu_coord_request_pending:
            return False
        if self.first_coord_allowed:
            return True
        return self.last_robot_state == 7

    def _queue_or_send_packet_to_mcu(self, packet: VisionPacket) -> None:
        send_now = False
        with self.state_lock:
            if self._is_robot_ready_for_new_coord_unlocked():
                self.mcu_coord_in_progress = True
                self.mcu_coord_request_pending = False
                self.first_coord_allowed = False
                send_now = True
            else:
                self.pending_packet_to_mcu = packet
                self.pending_packet_count += 1

        if send_now:
            print("[좌표 즉시 전송] 로봇 대기 상태 확인 -> MCU로 좌표 전송")
            if not self._send_packet_to_mcu(packet):
                with self.state_lock:
                    self.mcu_coord_in_progress = False
                    self.pending_packet_to_mcu = packet

    def _try_send_pending_packet_to_mcu(self) -> None:
        packet_to_send = None
        with self.state_lock:
            if self.pending_packet_to_mcu is None or not self._is_robot_ready_for_new_coord_unlocked():
                return
            packet_to_send = self.pending_packet_to_mcu
            self.pending_packet_to_mcu = None
            self.mcu_coord_in_progress = True
            self.mcu_coord_request_pending = False
            self.first_coord_allowed = False

        print("[보류 좌표 전송] STEP 1/7 대기 상태 확인 -> MCU로 좌표 전송")
        if not self._send_packet_to_mcu(packet_to_send):
            with self.state_lock:
                self.mcu_coord_in_progress = False
                self.pending_packet_to_mcu = packet_to_send

    def _mcu_uart_rx_loop(self) -> None:
        buffer = ""
        while self.running:
            if self.mcu is None:
                self.log.log_once("mcu_not_connected", "[MCU UART] 미연결 - 수신 대기 중지 상태")
                time.sleep(0.2)
                continue

            try:
                data = self.mcu.read(64)
            except Exception as e:
                self.log.log_once("mcu_rx_error", f"[MCU UART 수신 오류] {e}")
                time.sleep(0.2)
                continue

            if not data:
                continue

            self.log.reset_once("mcu_rx_error")
            buffer += data.decode("utf-8", errors="ignore")
            buffer = self._consume_lines(buffer, self._handle_mcu_line)

            stripped = buffer.strip()
            if stripped in COMMAND_SET:
                buffer = ""
                self._handle_mcu_line(stripped)

    def _handle_mcu_line(self, line: str) -> None:
        cmd = line.strip()
        if not cmd:
            return

        self._log_mcu_raw(cmd)
        self._send_master_event_to_isaac("MCU", "RX_RAW", {"value": cmd}, raw=cmd)

        if cmd == "0":
            self._handle_sensor_trigger()
            return
        if cmd in ROBOT_STATE_SET:
            self._handle_robot_state(int(cmd))
            return

        response = cmd.upper()
        if response in MCU_INSPECTION_CONVEYOR_STATE_SET:
            self._handle_inspection_conveyor_state(response)
            return
        if response in MCU_RESPONSE_SET:
            self._handle_mcu_response(response)
            return

        if cmd.startswith("DBG") or cmd.startswith("RANGE_ERR"):
            print(f"[MCU DEBUG] {cmd}")
            return

    def _log_mcu_raw(self, cmd: str) -> None:
        if PRINT_MCU_RAW_EVERY_TIME:
            print(f"[MCU -> MASTER] {cmd}")
            return
        self.log.log_on_change("mcu_cmd", cmd, f"[MCU -> MASTER] {cmd}")

    def _handle_sensor_trigger(self) -> None:
        with self.state_lock:
            self.sensor_trigger_count += 1
            trigger_count = self.sensor_trigger_count
            self.mcu_coord_request_pending = True

        ok = self._send_to_blackbox("SENSOR 0\n")
        self._send_master_event_to_isaac("MCU", "SENSOR_TRIGGER", {"count": int(trigger_count), "forwarded_to_blackbox": bool(오케이)}, raw="0")

        if ok:
            print(f"[좌표 요청 이벤트 #{trigger_count}] MCU 0 수신 -> blackbox SENSOR 0 전송")
            self.log.reset_once("blackbox_not_connected")
            return

    def _handle_robot_state(self, state_num: int) -> None:
        with self.state_lock:
            prev_state = self.last_robot_state
            self.last_robot_state = state_num
            self.robot_state_count += 1
            state_count = self.robot_state_count

        state_changed = prev_state != state_num
        self._send_master_event_to_isaac("MCU", "ROBOT_STATE", {"step": int(state_num), "previous_step": prev_state, "changed": bool(state_changed), "count": int(state_count)}, raw=str(state_num))

        if state_changed:
            print(f"[로봇 상태 변경 #{state_count}] STEP={state_num}")

        conveyor_state = GUI_CONVEYOR_STATE_MAP.get(state_num)
        if conveyor_state is not None:
            inspection_restart = True if state_num in {1, 4} else None
            if inspection_restart is True and state_changed:
                reason = "직교 로봇 흡착 완료" if state_num == 4 else "초기/대기 상태"
                print(f"[검사 컨베이어 상태] STM STEP={state_num} 수신 -> {reason}, 구동")

            self._set_gui_conveyor_state(inspection=inspection_restart, packaging=True)
        else:
            self._publish_gui_status()

        if state_changed:
            self._send_to_blackbox(f"STATE {state_num}\n")

        if state_num == 4:
            if state_changed:
                print("[흡착 완료 이벤트] OpenRB 각도 0도 초기화")
                self._send_angle_to_openrb(0.0, force_send=True)
                with self.state_lock:
                    self.openrb_angle_blocked = True
                    self.pending_openrb_angle = None
                    self.pending_openrb_angle_time = 0.0

        if state_num == 7 and state_changed:
            pending_angle = None
            with self.state_lock:
                self.openrb_angle_blocked = False
                self.mcu_coord_in_progress = False
                if self.pending_openrb_angle is not None:
                    pending_angle = self.pending_openrb_angle
                    self.pending_openrb_angle = None

            if pending_angle is not None:
                print(f"[적재 완료 후 OpenRB 각도 전송] 보류 각도 {pending_angle:.2f}도")
                self._send_angle_to_openrb(pending_angle, force_send=True)

            self._try_send_pending_packet_to_mcu()

    def _handle_inspection_conveyor_state(self, state: str) -> None:
        running = (state == "IRUN")
        print(f"[검사 컨베이어 상태] STM {state} 수신 -> {'구동' if running else '정지'}")
        self._send_master_event_to_isaac("CONVEYOR", "INSPECTION_STATE", {"signal": state, "inspection_running": running}, raw=state)
        self._set_gui_conveyor_state(inspection=running)

    def _handle_mcu_response(self, response: str) -> None:
        self._send_master_event_to_isaac("MCU", "RESPONSE", {"response": response}, raw=response)
        if response == "BUSY":
            with self.state_lock:
                self.mcu_coord_in_progress = False
                if self.latest_packet is not None:
                    self.pending_packet_to_mcu = self.latest_packet

    def _send_packet_to_mcu(self, packet: VisionPacket) -> bool:
        if self.mcu is None:
            return False

        payload = f"G,{packet.x:.1f},{packet.y:.1f},{packet.z:.1f}\r\n".encode("ascii")
        try:
            with self.mcu_lock:
                self.mcu.write(payload)
                self.mcu.flush()
            print(f"[MASTER -> MCU] G,{packet.x:.1f},{packet.y:.1f},{packet.z:.1f}")
            return True
        except Exception as e:
            print(f"[MCU UART 전송 오류] {e}")
            return False

    def _send_stop_to_mcu(self) -> bool:
        if self.mcu is None:
            return False
        payload = b"p\r\n"
        try:
            with self.mcu_lock:
                self.mcu.write(payload)
                self.mcu.flush()
            print("[MASTER -> MCU STOP] p")
            return True
        except Exception as e:
            print(f"[MCU UART 정지 명령 전송 오류] {e}")
            return False

    def _send_angle_to_openrb(self, angle_deg: float, force_send: bool = False) -> bool:
        if self.openrb is None:
            return False

        msg = f"{angle_deg:.2f}\n"
        try:
            with self.openrb_lock:
                self.openrb.write(msg.encode("utf-8"))
            print(f"[MASTER -> OpenRB] {msg.strip()}")
            return True
        except Exception as e:
            print(f"[OpenRB UART 전송 오류] {e}")
            return False

    @staticmethod
    def _parse_angle_only(line: str) -> Optional[float]:
        raw = line.strip()
        upper = raw.upper()
        if upper.startswith(("A ", "ANGLE ", "YAW ", "DEG ", "THETA ")):
            numbers = NUMBER_RE.findall(raw)
            if numbers:
                try:
                    return float(numbers[0])
                except ValueError:
                    return None
        return MasterBridge._extract_named_angle(raw)

    @staticmethod
    def _parse_vision_packet(line: str) -> Optional[VisionPacket]:
        raw = line.strip()
        if raw.startswith("{") and raw.endswith("}"):
            return MasterBridge._parse_json_packet(raw)

        upper = raw.upper()
        if upper.startswith("G ") or upper.startswith("G,"):
            payload = raw[2:].strip(" ,")
        elif upper.startswith("VISION ") or upper.startswith("VISION,"):
            payload = raw[7:].strip(" ,")
        else:
            return None

        return MasterBridge._parse_numeric_packet(payload, raw)

    @staticmethod
    def _parse_json_packet(raw: str) -> Optional[VisionPacket]:
        try:
            obj = json.loads(raw)
            return VisionPacket(float(obj["x"]), float(obj["y"]), float(obj.get("z", 0.0)), float(obj.get("angle", 0.0)), raw, time.time())
        except Exception:
            return None

    @staticmethod
    def _parse_numeric_packet(payload: str, raw: str) -> Optional[VisionPacket]:
        angle = MasterBridge._extract_named_angle(payload)
        numbers = NUMBER_RE.findall(payload)
        if len(numbers) < 2:
            return None

        try:
            x, y = float(numbers[0]), float(numbers[1])
            z = float(numbers[2]) if len(numbers) >= 3 else 0.0
            if angle is None and len(numbers) >= 4:
                angle = float(numbers[3])
            return VisionPacket(x, y, z, angle, raw, time.time())
        except ValueError:
            return None

    @staticmethod
    def _extract_named_angle(text: str) -> Optional[float]:
        match = ANGLE_NAME_RE.search(text)
        if match is None:
            return None
        try:
            return float(match.group(1))
        except ValueError:
            return None

    @staticmethod
    def _parse_yolo_result(line: str) -> Optional[YoloResultPacket]:
        raw = line.strip()
        parts = re.split(r"[,\s]+", raw)
        if len(parts) < 3 or parts[0].upper() != "YOLO_RESULT":
            return None

        try:
            track_id = int(parts[1])
        except ValueError:
            return None

        result = parts[2].lower()
        label = "DEFECT" if result == "f" else ("NORMAL" if result == "t" else "UNKNOWN")
        return YoloResultPacket(track_id=track_id, result=result, label=label, raw=raw, received_at=time.time())

    @staticmethod
    def _consume_lines(buffer: str, callback) -> str:
        while "\n" in buffer or "\r" in buffer:
            parts = re.split(r"[\r\n]", buffer, maxsplit=1)
            line = parts[0].strip()
            buffer = parts[1]
            if line:
                callback(line)
        return buffer

    def _gui_heartbeat_loop(self) -> None:
        while self.running:
            time.sleep(GUI_HEARTBEAT_INTERVAL_SEC)
            if self.gui.is_connected():
                self._publish_gui_status()

    def _set_gui_conveyor_state(self, inspection: Optional[bool] = None, packaging: Optional[bool] = None) -> None:
        with self.state_lock:
            if inspection is not None:
                self.inspection_conveyor_running = bool(inspection)

        self._publish_gui_status()

    def _publish_gui_status(self) -> None:
        with self.state_lock:
            good_count = self.good_count
            defect_count = self.defect_count
            inspection_conveyor = self.inspection_conveyor_running

        self.gui.publish({
            "type": "update",
            "normal_count": good_count,
            "defect_count": defect_count,
            "inspection_conveyor_running": inspection_conveyor,
            "packaging_conveyor_running": True,
        })

    def _handle_yolo_result(self, packet: YoloResultPacket) -> None:
        with self.state_lock:
            self.latest_yolo_result = packet
            self.yolo_result_count += 1

            if packet.result == "t":
                self.good_count += 1
                result_str = "정상"
            elif packet.result == "f":
                self.defect_count += 1
                result_str = "불량"
            else:
                result_str = "-"

        # [수정된 핵심 로직] YOLO에서 정상(t)이든 불량(f)이든 수신 즉시 GUI에 전달하여 이력 표에 새 행 생성
        self.gui.publish({
            "type": "yolo_result",
            "inspection": result_str,
            "judgement": result_str,
            "track_id": packet.track_id,
            "normal_count": self.good_count,
            "defect_count": self.defect_count,
            "time": time.strftime("%H:%M:%S", time.localtime(packet.received_at)),
        })

        self._forward_yolo_result_to_isaac(packet)

        if packet.result == "t":
            self._send_yolo_result_to_mcu("t")
        elif packet.result == "f":
            self._send_yolo_result_to_mcu("f")

    def _send_yolo_result_to_mcu(self, value: str) -> bool:
        if self.mcu is None:
            return False

        payload = f"{value}\r\n".encode("ascii")
        try:
            with self.mcu_lock:
                self.mcu.write(payload)
                self.mcu.flush()
            print(f"[MASTER -> MCU YOLO] {value}")
            return True
        except Exception as e:
            print(f"[MCU UART YOLO 결과 전송 오류] {e}")
            return False


if __name__ == "__main__":
    bridge = MasterBridge(HOST, TCP_PORT)
    bridge.start()