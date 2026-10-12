from __future__ import annotations

import json
import queue
import socket
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk


APP_TITLE = "제품 불량 검출 및 포장 로봇"
LOGO_FILE = Path(__file__).resolve().parent / "school(1).png"

ISAAC = ("172.30.1.64", 5000)
FULL = ("127.0.0.1", 5100)
DEFECT = ("172.30.1.63", 5001)

TELEMETRY_BIND_HOST = "0.0.0.0"
TELEMETRY_PORT = 5101

REFRESH_MS = 50
RETRY_SEC = 2.0
VIDEO_BG = "#EEEEEE"

HISTORY_MAX_ROWS = 20
HISTORY_VISIBLE_HEIGHT = 125

cv2.setNumThreads(1)


def drain(target_queue: queue.Queue) -> None:
    try:
        while True:
            target_queue.get_nowait()
    except queue.Empty:
        pass


def put_latest(target_queue: queue.Queue, value: Any) -> None:
    drain(target_queue)
    try:
        target_queue.put_nowait(value)
    except queue.Full:
        pass


def frame_to_photo(frame: np.ndarray, width: int, height: int) -> tk.PhotoImage:
    source_height, source_width = frame.shape[:2]
    scale = min(width / max(source_width, 1), height / max(source_height, 1))

    resized_width = max(1, round(source_width * scale))
    resized_height = max(1, round(source_height * scale))

    interpolation = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=interpolation)

    canvas = np.full((height, width, 3), 238, dtype=np.uint8)
    offset_x = (width - resized_width) // 2
    offset_y = (height - resized_height) // 2

    canvas[offset_y:offset_y + resized_height, offset_x:offset_x + resized_width] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

    ppm_data = f"P6\n{width} {height}\n255\n".encode("ascii") + rgb.tobytes()
    return tk.PhotoImage(data=ppm_data, format="PPM")


class VideoCanvas(tk.Canvas):
    def __init__(self, parent: tk.Widget, placeholder: str) -> None:
        super().__init__(parent, bg=VIDEO_BG, bd=0, highlightthickness=0)
        self.placeholder = placeholder
        self.photo: Optional[tk.PhotoImage] = None

        self.image_id = self.create_image(0, 0, anchor="nw")
        self.text_id = self.create_text(
            0, 0, text=placeholder, fill="#3A3A3A",
            font=("Malgun Gothic", 28, "bold"), anchor="center"
        )
        self.bind("<Configure>", self._center)

    def _center(self, _event=None) -> None:
        self.coords(
            self.text_id,
            max(1, self.winfo_width()) // 2,
            max(1, self.winfo_height()) // 2,
        )

    def show_frame(self, frame: np.ndarray) -> None:
        width = max(320, self.winfo_width())
        height = max(180, self.winfo_height())

        self.photo = frame_to_photo(frame, width, height)
        self.itemconfigure(self.image_id, image=self.photo)
        self.itemconfigure(self.text_id, state="hidden")

    def reset(self) -> None:
        self.photo = None
        self.itemconfigure(self.image_id, image="")
        self.itemconfigure(self.text_id, text=self.placeholder, state="normal")
        self._center()


class MjpegWorker(threading.Thread):
    def __init__(self, key: str, url: str, frame_queue: queue.Queue, event_queue: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.key = key
        self.url = url
        self.frame_queue = frame_queue
        self.event_queue = event_queue
        self.stop_event = threading.Event()
        self.response = None

    def stop(self) -> None:
        self.stop_event.set()
        try:
            if self.response:
                self.response.close()
        except Exception:
            pass

    def run(self) -> None:
        while not self.stop_event.is_set():
            self.event_queue.put(("stream", self.key, False))
            try:
                request = urllib.request.Request(self.url, headers={"User-Agent": "FactoryGUI/1.0"})
                self.response = urllib.request.urlopen(request, timeout=8)

                if self.stop_event.is_set():
                    break

                self.event_queue.put(("stream", self.key, True))
                buffer = b""

                while not self.stop_event.is_set():
                    chunk = self.response.read(8192)
                    if not chunk:
                        raise ConnectionError
                    buffer += chunk

                    while True:
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2 if start >= 0 else 0)
                        if start < 0 or end <= start:
                            break

                        jpeg = buffer[start : end + 2]
                        buffer = buffer[end + 2 :]

                        encoded = np.frombuffer(jpeg, np.uint8)
                        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)

                        if frame is not None:
                            put_latest(self.frame_queue, frame)

                    if len(buffer) > 8_000_000:
                        buffer = buffer[-1_000_000:]

            except Exception:
                if not self.stop_event.is_set():
                    self.event_queue.put(("stream", self.key, False))
                    self.stop_event.wait(RETRY_SEC)
            finally:
                try:
                    if self.response:
                        self.response.close()
                except Exception:
                    pass
                self.response = None


class TelemetryServer(threading.Thread):
    def __init__(self, host: str, port: int, event_queue: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.event_queue = event_queue
        self.stop_event = threading.Event()
        self.server: Optional[socket.socket] = None

    def stop(self) -> None:
        self.stop_event.set()
        try:
            if self.server:
                self.server.close()
        except Exception:
            pass

    def run(self) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind((self.host, self.port))
            self.server.listen(5)
            self.server.settimeout(1)

            if self.stop_event.is_set():
                return

            self.event_queue.put(("server", True))

            while not self.stop_event.is_set():
                try:
                    client, client_addr = self.server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                self.event_queue.put(("telemetry_client", 1, client_addr[0]))
                threading.Thread(target=self._client, args=(client, client_addr), daemon=True).start()

        except Exception:
            self.event_queue.put(("server", False))
        finally:
            try:
                if self.server:
                    self.server.close()
            except Exception:
                pass

    def _client(self, client: socket.socket, client_addr: tuple[str, int]) -> None:
        client.settimeout(1)
        buffer = b""
        try:
            while not self.stop_event.is_set():
                try:
                    chunk = client.recv(8192)
                except socket.timeout:
                    continue

                if not chunk:
                    break
                buffer += chunk

                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    try:
                        data = json.loads(raw.strip().decode("utf-8"))
                    except Exception:
                        continue

                    if isinstance(data, dict):
                        self.event_queue.put(("telemetry", data))
        finally:
            try:
                client.close()
            except Exception:
                pass
            self.event_queue.put(("telemetry_client", -1, client_addr[0]))


class GuiApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root

        root.title(APP_TITLE)
        root.minsize(1000, 640)

        self.is_fullscreen = True
        root.attributes("-fullscreen", True)

        root.bind("<F11>", self.toggle_fullscreen)
        root.bind("<Escape>", self.exit_fullscreen)

        self.event_queue: queue.Queue = queue.Queue()
        self.frame_queues = {key: queue.Queue(maxsize=1) for key in ("isaac", "full", "defect")}

        self.workers: dict[str, MjpegWorker] = {}
        self.server: Optional[TelemetryServer] = None
        self.stream_connected = {key: False for key in self.frame_queues}

        self.server_connected = False
        self.telemetry_client_count = 0
        self.last_master_host: Optional[str] = None
        self.connecting = False

        self.good_count = 0
        self.defect_count = 0
        self.good_count_baseline: Optional[int] = None
        self.defect_count_baseline: Optional[int] = None

        self.sequence = 0
        self.last_signature = None
        self.history_rows: list[dict[str, Any]] = []

        self.configure_style()
        self.build_ui()
        self.set_connection_state("off")

        root.after(REFRESH_MS, self.update_loop)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def configure_style(self) -> None:
        self.colors = {
            "background": "#F0F0F0", "panel": "#F4F4F4", "white": "#FFFFFF",
            "border": "#D0D0D0", "text": "#1A1A1A", "muted": "#525252",
            "blue": "#2457C5", "green": "#16A34A", "red": "#DC2626",
        }

        self.root.configure(bg=self.colors["background"])
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure("TFrame", background=self.colors["background"])
        style.configure("Panel.TFrame", background=self.colors["panel"], borderwidth=1, relief="solid")
        style.configure("White.TFrame", background=self.colors["white"], borderwidth=1, relief="solid")
        style.configure("Title.TLabel", background=self.colors["background"], foreground=self.colors["text"], font=("Malgun Gothic", 34, "bold"))
        style.configure("Section.TLabel", background=self.colors["panel"], foreground=self.colors["text"], font=("Malgun Gothic", 18, "bold"))
        style.configure("Card.TLabel", background=self.colors["white"], foreground=self.colors["muted"], font=("Malgun Gothic", 18, "bold"), anchor="center")

        for style_name, color in (("Blue", self.colors["blue"]), ("Green", self.colors["green"]), ("Red", self.colors["red"])):
            style.configure(f"Value{style_name}.TLabel", background=self.colors["white"], foreground=color, font=("Malgun Gothic", 38, "bold"), anchor="center")

    def build_ui(self) -> None:
        header = ttk.Frame(self.root)
        header.pack(fill="x", padx=10, pady=(6, 5))

        ttk.Label(header, text=APP_TITLE, style="Title.TLabel").pack(side="left")
        self.add_logo(header)
        self.add_connection_controls(header)

        self.good_count_var = tk.StringVar(value="0")
        self.defect_count_var = tk.StringVar(value="0")
        self.inspection_conveyor_var = tk.StringVar(value="대기")
        self.packaging_conveyor_var = tk.StringVar(value="대기")

        video_area = ttk.Frame(self.root)
        video_area.pack(fill="both", expand=True, padx=10, pady=(0, 4))
        video_area.columnconfigure(0, weight=7, uniform="video")
        video_area.columnconfigure(1, weight=3, uniform="video")
        video_area.rowconfigure(0, weight=1)

        self.canvases: dict[str, VideoCanvas] = {}
        isaac_panel = self.create_video_panel(video_area, "isaac", "Isaac Sim 전체 공정 화면")
        isaac_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 3))

        side_area = ttk.Frame(video_area)
        side_area.grid(row=0, column=1, sticky="nsew", padx=(3, 0))
        side_area.columnconfigure(0, weight=1)
        side_area.rowconfigure(0, weight=1, uniform="side")
        side_area.rowconfigure(1, weight=1, uniform="side")

        full_panel = self.create_video_panel(side_area, "full", "정상 제품 적재")
        full_panel.grid(row=0, column=0, sticky="nsew", pady=(0, 4))

        defect_panel = self.create_video_panel(side_area, "defect", "제품 불량 판독")
        defect_panel.grid(row=1, column=0, sticky="nsew", pady=(4, 0))

        bottom_area = ttk.Frame(self.root)
        bottom_area.pack(fill="x", padx=10, pady=(0, 4))
        bottom_area.columnconfigure(0, weight=7, uniform="bottom")
        bottom_area.columnconfigure(1, weight=3, uniform="bottom")
        bottom_area.rowconfigure(0, weight=1)

        self.add_history(bottom_area)
        self.add_status_cards(bottom_area)

    def add_logo(self, header: ttk.Frame) -> None:
        try:
            image = cv2.imread(str(LOGO_FILE), cv2.IMREAD_UNCHANGED)
            if image is None:
                raise FileNotFoundError(LOGO_FILE)

            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGRA)
            elif image.shape[2] == 3:
                alpha = np.full(image.shape[:2], 255, dtype=np.uint8)
                image = np.dstack((image, alpha))

            alpha = image[:, :, 3]
            points = cv2.findNonZero((alpha > 5).astype(np.uint8))
            if points is not None:
                x, y, width, height = cv2.boundingRect(points)
                image = image[y : y + height, x : x + width]

            target_height = 78
            source_height, source_width = image.shape[:2]
            target_width = max(1, round(source_width * target_height / source_height))

            image = cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)
            background = np.full((target_height, target_width, 3), 240, dtype=np.uint8)

            logo_bgr = image[:, :, :3].astype(np.float32)
            alpha = image[:, :, 3].astype(np.float32)[:, :, None] / 255.0

            composited = (logo_bgr * alpha + background.astype(np.float32) * (1.0 - alpha)).astype(np.uint8)
            logo_rgb = cv2.cvtColor(composited, cv2.COLOR_BGR2RGB)

            ppm_data = f"P6\n{target_width} {target_height}\n255\n".encode("ascii") + logo_rgb.tobytes()
            self.logo = tk.PhotoImage(data=ppm_data, format="PPM")

            tk.Label(header, image=self.logo, bg=self.colors["background"], bd=0, highlightthickness=0).pack(side="right", padx=(16, 3), pady=1)
        except Exception:
            tk.Label(header, text="한국공학대학교", bg=self.colors["background"], fg="#2468B3", font=("Malgun Gothic", 18, "bold"), bd=0).pack(side="right", padx=(16, 3))

    def add_connection_controls(self, header: ttk.Frame) -> None:
        connection_box = tk.LabelFrame(header, text="연결", bg=self.colors["background"], fg=self.colors["text"], font=("Malgun Gothic", 11), padx=10, pady=7)
        connection_box.pack(side="right", padx=(10, 55))

        self.connect_button = tk.Button(connection_box, text="Connect", width=11, command=self.connect_all, font=("Malgun Gothic", 11), padx=2, pady=1)
        self.disconnect_button = tk.Button(connection_box, text="Disconnect", width=11, command=self.disconnect_all, font=("Malgun Gothic", 11), padx=2, pady=1)
        self.connection_text = tk.Label(connection_box, bg=self.colors["background"], font=("Malgun Gothic", 19, "bold"))

        self.connect_button.pack(side="left", padx=(0, 9))
        self.disconnect_button.pack(side="left", padx=(0, 13))
        self.connection_text.pack(side="left")

    def create_video_panel(self, parent: tk.Widget, key: str, title: str) -> ttk.Frame:
        panel = ttk.Frame(parent, style="Panel.TFrame")
        ttk.Label(panel, text=title, style="Section.TLabel").pack(anchor="w", padx=10, pady=(7, 5))

        canvas = VideoCanvas(panel, title)
        canvas.pack(fill="both", expand=True, padx=4, pady=(0, 4))
        self.canvases[key] = canvas
        return panel

    def add_history(self, parent: ttk.Frame) -> None:
        panel = ttk.Frame(parent, style="Panel.TFrame")
        panel.grid(row=0, column=0, sticky="nsew", padx=(0, 3))
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(1, weight=1)

        ttk.Label(panel, text="최근 판정 이력", style="Section.TLabel").grid(row=0, column=0, sticky="w", padx=10, pady=(6, 3))

        table_outer = tk.Frame(panel, bg=self.colors["white"], highlightbackground=self.colors["border"], highlightcolor=self.colors["border"], highlightthickness=1, bd=0)
        table_outer.grid(row=1, column=0, sticky="nsew", padx=4, pady=(0, 4))
        table_outer.columnconfigure(0, weight=1)
        table_outer.rowconfigure(1, weight=1)

        self.history_columns = ("순번", "판정", "위치(mm)", "각도(°)", "검출 시간")
        self.history_column_weights = (8, 12, 34, 14, 22)

        header_container = tk.Frame(table_outer, bg=self.colors["white"])
        header_container.grid(row=0, column=0, sticky="ew")
        header_container.columnconfigure(0, weight=1)
        header_container.columnconfigure(1, minsize=17)

        header_frame = tk.Frame(header_container, bg=self.colors["white"])
        header_frame.grid(row=0, column=0, sticky="ew")

        tk.Frame(header_container, bg=self.colors["white"]).grid(row=0, column=1, sticky="nsew")

        for column, weight in enumerate(self.history_column_weights):
            header_frame.columnconfigure(column, weight=weight, uniform="history_columns")

        for column, heading in enumerate(self.history_columns):
            tk.Label(
                header_frame, text=heading, bg="#F3F3F3", fg=self.colors["text"],
                font=("Malgun Gothic", 15, "bold"), anchor="center", padx=2, pady=5, bd=0,
                highlightbackground=self.colors["border"], highlightcolor=self.colors["border"], highlightthickness=1
            ).grid(row=0, column=column, sticky="nsew")

        body_container = tk.Frame(table_outer, bg=self.colors["white"])
        body_container.grid(row=1, column=0, sticky="nsew")
        body_container.columnconfigure(0, weight=1)
        body_container.rowconfigure(0, weight=1)

        self.history_canvas = tk.Canvas(body_container, bg=self.colors["white"], bd=0, highlightthickness=0, height=HISTORY_VISIBLE_HEIGHT)
        history_scrollbar = ttk.Scrollbar(body_container, orient="vertical", command=self.history_canvas.yview)
        self.history_canvas.configure(yscrollcommand=history_scrollbar.set)

        self.history_canvas.grid(row=0, column=0, sticky="nsew")
        history_scrollbar.grid(row=0, column=1, sticky="ns")

        history_scrollbar.bind("<Configure>", lambda event: header_container.columnconfigure(1, minsize=event.width))

        self.history_body = tk.Frame(self.history_canvas, bg=self.colors["white"])
        self.history_window = self.history_canvas.create_window((0, 0), window=self.history_body, anchor="nw")

        for column, weight in enumerate(self.history_column_weights):
            self.history_body.columnconfigure(column, weight=weight, uniform="history_columns")

        self.history_cells: list[list[tk.Label]] = []
        for row in range(HISTORY_MAX_ROWS):
            self.history_body.rowconfigure(row, minsize=34)
            row_cells = []
            for column in range(len(self.history_columns)):
                cell = tk.Label(
                    self.history_body, text="", bg=self.colors["white"], fg=self.colors["text"],
                    font=("Malgun Gothic", 14), anchor="center", padx=2, pady=5, bd=0,
                    highlightbackground=self.colors["border"], highlightcolor=self.colors["border"], highlightthickness=1
                )
                cell.grid(row=row, column=column, sticky="nsew")
                row_cells.append(cell)
            self.history_cells.append(row_cells)

        self.history_body.bind("<Configure>", self._update_history_scrollregion)
        self.history_canvas.bind("<Configure>", self._resize_history_body)
        self.history_canvas.bind("<MouseWheel>", self._scroll_history)

    def _update_history_scrollregion(self, _event=None) -> None:
        self.history_canvas.configure(scrollregion=self.history_canvas.bbox("all"))

    def _resize_history_body(self, event) -> None:
        self.history_canvas.itemconfigure(self.history_window, width=event.width)

    def _scroll_history(self, event) -> None:
        self.history_canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

    def render_history_table(self) -> None:
        """이력 데이터 목록을 GUI 표에 그려줌"""
        for row_index in range(HISTORY_MAX_ROWS):
            if row_index < len(self.history_rows):
                item = self.history_rows[row_index]
                row_values = (
                    item.get("seq", ""),
                    item.get("result", ""),
                    item.get("position", "-"),
                    item.get("angle", "-"),
                    item.get("time", "-"),
                )
            else:
                row_values = ("", "", "", "", "")

            for column_index, cell in enumerate(self.history_cells[row_index]):
                value = row_values[column_index] if column_index < len(row_values) else ""
                cell.configure(text="" if value is None else str(value))

        self.history_canvas.yview_moveto(0)

    def add_status_cards(self, parent: ttk.Frame) -> None:
        panel = ttk.Frame(parent, style="Panel.TFrame")
        panel.grid(row=0, column=1, sticky="nsew", padx=(3, 0))
        panel.rowconfigure(0, weight=1)

        for column in range(4):
            panel.columnconfigure(column, weight=1, uniform="status_cards")

        cards = (
            ("정상 개수", self.good_count_var, self.colors["green"]),
            ("불량 개수", self.defect_count_var, self.colors["red"]),
            ("검사\n컨베이어", self.inspection_conveyor_var, self.colors["blue"]),
            ("포장\n컨베이어", self.packaging_conveyor_var, self.colors["blue"]),
        )

        for column, (title, variable, value_color) in enumerate(cards):
            card = tk.Frame(panel, bg=self.colors["white"], highlightbackground=self.colors["border"], highlightcolor=self.colors["border"], highlightthickness=1, bd=0)
            card.grid(row=0, column=column, sticky="nsew", padx=3, pady=3)
            card.columnconfigure(0, weight=1)
            card.rowconfigure(0, weight=0, minsize=72)
            card.rowconfigure(1, weight=0, minsize=1)
            card.rowconfigure(2, weight=1)

            tk.Label(card, text=title, bg="#F3F3F3", fg=self.colors["text"], font=("Malgun Gothic", 17, "bold"), anchor="center", justify="center", bd=0).grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
            tk.Frame(card, bg=self.colors["border"], height=1, bd=0).grid(row=1, column=0, sticky="ew")
            tk.Label(card, textvariable=variable, bg=self.colors["white"], fg=value_color, font=("Malgun Gothic", 38, "bold"), anchor="center", justify="center", bd=0).grid(row=2, column=0, sticky="nsew", padx=2, pady=2)

    def set_connection_state(self, state: str) -> None:
        states = {
            "off": ("연결X", "#C62828", "normal", "disabled"),
            "wait": ("연결 중", "#E58A00", "disabled", "normal"),
            "on": ("연결O", "#1536C7", "disabled", "normal"),
        }
        text, color, connect_state, disconnect_state = states[state]
        self.connection_text.config(text=text, fg=color)
        self.connect_button.config(state=connect_state)
        self.disconnect_button.config(state=disconnect_state)

    def refresh_connection_state(self) -> None:
        if not self.connecting:
            self.set_connection_state("off")
        elif self.server_connected and self.telemetry_client_count > 0 and all(self.stream_connected.values()):
            self.set_connection_state("on")
        else:
            self.set_connection_state("wait")

    def connect_all(self) -> None:
        if self.connecting:
            return

        self.connecting = True
        self.server_connected = False
        self.telemetry_client_count = 0
        self.last_master_host = None

        self.stream_connected = {key: False for key in self.frame_queues}
        self.set_connection_state("wait")

        self.good_count = 0
        self.defect_count = 0
        self.good_count_baseline = None
        self.defect_count_baseline = None
        self.sequence = 0
        self.last_signature = None
        self.history_rows.clear()

        self.good_count_var.set("0")
        self.defect_count_var.set("0")
        self.render_history_table()

        self.inspection_conveyor_var.set("구동")
        self.packaging_conveyor_var.set("구동")

        self.server = TelemetryServer(TELEMETRY_BIND_HOST, TELEMETRY_PORT, self.event_queue)
        self.server.start()

        streams = {"isaac": ISAAC, "full": FULL, "defect": DEFECT}
        for key, stream_address in streams.items():
            host, port = stream_address
            worker = MjpegWorker(key, f"http://{host}:{port}/video", self.frame_queues[key], self.event_queue)
            self.workers[key] = worker
            worker.start()

    def disconnect_all(self) -> None:
        self.connecting = False
        workers = list(self.workers.values())

        for worker in workers:
            worker.stop()
        for worker in workers:
            worker.join(timeout=0.3)

        self.workers.clear()
        if self.server:
            self.server.stop()
            self.server.join(timeout=0.5)
            self.server = None

        for frame_queue in self.frame_queues.values():
            drain(frame_queue)
        drain(self.event_queue)

        self.server_connected = False
        self.telemetry_client_count = 0
        self.last_master_host = None
        self.stream_connected = {key: False for key in self.frame_queues}

        for canvas in self.canvases.values():
            canvas.reset()

        self.inspection_conveyor_var.set("대기")
        self.packaging_conveyor_var.set("대기")
        self.set_connection_state("off")

    def update_loop(self) -> None:
        self.process_events()
        for key, frame_queue in self.frame_queues.items():
            try:
                frame = frame_queue.get_nowait()
            except queue.Empty:
                continue
            self.canvases[key].show_frame(frame)

        self.root.after(REFRESH_MS, self.update_loop)

    def process_events(self) -> None:
        while True:
            try:
                event = self.event_queue.get_nowait()
            except queue.Empty:
                return

            event_type = event[0]
            if event_type == "stream" and self.connecting:
                self.stream_connected[event[1]] = bool(event[2])
                self.refresh_connection_state()
            elif event_type == "server":
                self.server_connected = bool(event[1]) if self.connecting else False
                self.refresh_connection_state()
            elif event_type == "telemetry_client" and self.connecting:
                delta = int(event[1])
                self.telemetry_client_count = max(0, self.telemetry_client_count + delta)
                if delta > 0:
                    self.last_master_host = str(event[2])
                    print(f"[MASTER TCP 연결] {self.last_master_host}:{TELEMETRY_PORT}")
                else:
                    print(f"[MASTER TCP 연결 종료] {event[2]}")
                    if self.telemetry_client_count == 0:
                        self.last_master_host = None
                self.refresh_connection_state()
            elif event_type == "telemetry" and self.connecting:
                self.apply_telemetry(event[1])

    @staticmethod
    def normalize_result(value: Any) -> str:
        text = "" if value is None else str(value).strip()
        upper = text.upper()
        if upper in {"T", "OK", "PASS", "NORMAL", "GOOD", "정상"}:
            return "정상"
        if upper in {"F", "NG", "FAIL", "DEFECT", "BAD", "불량"}:
            return "불량"
        return text or "-"

    @staticmethod
    def pick_value(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
        for key in keys:
            value = data.get(key)
            if value not in (None, ""):
                return value
        return default

    @staticmethod
    def display_text(value: Any, default: str = "-") -> str:
        text = "" if value is None else str(value).strip()
        return text or default

    @staticmethod
    def format_angle(value: Any) -> str:
        if value is None:
            return "-"
        text = str(value).strip()
        if not text or text == "-":
            return "-"
        text = text.rstrip("°").strip()
        return f"{text}°"

    @staticmethod
    def to_boolean(value: Any) -> Optional[bool]:
        if value is None:
            return None
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"1", "true", "on", "yes", "run", "running", "동작", "가동"}:
                return True
            if normalized in {"0", "false", "off", "no", "stop", "stopped", "정지"}:
                return False
            return None
        return bool(value)

    def apply_telemetry(self, data: dict[str, Any]) -> None:
        event_type = data.get("type")

        # 1. 컨베이어 상태 제어
        legacy_conveyor = self.pick_value(data, "conveyor_running", "conveyor", "belt_running")
        inspection_conveyor = self.pick_value(data, "inspection_conveyor_running", "inspection_conveyor", "inspection_belt_running", default=legacy_conveyor)
        
        if inspection_conveyor is not None:
            self.set_var_if_changed(self.inspection_conveyor_var, self.conveyor_text(inspection_conveyor))

        # 포장 컨베이어는 항시 구동 상태 유지
        self.set_var_if_changed(self.packaging_conveyor_var, "구동" if self.connecting else "대기")

        # 2. 카운트 수치 업데이트
        good_count = self.pick_value(data, "normal_count", "good_count", "ok_count")
        defect_count = self.pick_value(data, "defect_count", "ng_count", "bad_count")

        if good_count is not None:
            received_good_count = self.to_integer(good_count, 0)
            if self.good_count_baseline is None:
                self.good_count_baseline = received_good_count
            if received_good_count < self.good_count_baseline:
                self.good_count_baseline = received_good_count
            self.good_count = max(0, received_good_count - self.good_count_baseline)

        if defect_count is not None:
            received_defect_count = self.to_integer(defect_count, 0)
            if self.defect_count_baseline is None:
                self.defect_count_baseline = received_defect_count
            if received_defect_count < self.defect_count_baseline:
                self.defect_count_baseline = received_defect_count
            self.defect_count = max(0, received_defect_count - self.defect_count_baseline)

        self.good_count_var.set(str(self.good_count))
        self.defect_count_var.set(str(self.defect_count))

        # 3. 비동기 판정 이력(History) 업데이트
        raw_x = self.pick_value(data, "x")
        raw_y = self.pick_value(data, "y")
        raw_z = self.pick_value(data, "z")
        raw_angle = self.pick_value(data, "angle", "theta")
        has_coord = (raw_x is not None and raw_y is not None)

        result_raw = self.pick_value(data, "inspection", "judgement", "result", "status")
        result = self.normalize_result(result_raw) if result_raw else "-"

        # [A] Depth 좌표/각도가 들어온 경우 -> 가장 오래된(아래쪽) '정상' 항목 중 위치가 '-'인 항목에 채워넣기
        if event_type == "depth_coord" or (has_coord and result in {"-", "", "대기"}):
            x = self.display_text(raw_x)
            y = self.display_text(raw_y)
            z = self.display_text(raw_z)
            angle = self.format_angle(raw_angle)
            position = f"({x}, {y}, {z})"

            # reversed()를 이용해 리스트의 뒤(오래된 순)부터 검색
            for item in reversed(self.history_rows):
                if item["result"] == "정상" and item["position"] == "-":
                    item["position"] = position
                    item["angle"] = angle
                    break

            self.render_history_table()
            return

        # [B] 웹캠(YOLO)에서 판정(정상/불량)이 들어온 경우 -> 즉시 [순번, 판정, -, -, 검출시간] 행 생성
        if result in {"정상", "불량"}:
            track_id = data.get("track_id")
            detected_time = self.display_text(self.pick_value(data, "time", "timestamp", "detected_time"), default=time.strftime("%H:%M:%S"))

            signature = (result, track_id, detected_time)

            if signature != self.last_signature:
                self.last_signature = signature
                self.sequence += 1

                # 만약 웹캠 신호 시점에 좌표도 들어있다면 채우고, 없으면 '-'로 설정
                if has_coord:
                    x = self.display_text(raw_x)
                    y = self.display_text(raw_y)
                    z = self.display_text(raw_z)
                    position = f"({x}, {y}, {z})"
                    angle = self.format_angle(raw_angle)
                else:
                    position = "-"
                    angle = "-"

                new_item = {
                    "seq": self.sequence,
                    "result": result,
                    "position": position,
                    "angle": angle,
                    "time": detected_time,
                }

                self.history_rows.insert(0, new_item)
                self.history_rows = self.history_rows[:HISTORY_MAX_ROWS]
                self.render_history_table()

    @staticmethod
    def set_var_if_changed(variable: tk.StringVar, value: str) -> None:
        if variable.get() != value:
            variable.set(value)

    def conveyor_text(self, value: Any) -> str:
        state = self.to_boolean(value)
        if state is None:
            return "구동" if self.connecting else "대기"
        return "구동" if state else "정지"

    @staticmethod
    def to_integer(value: Any, default: int) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def toggle_fullscreen(self, _event=None) -> str:
        self.is_fullscreen = not self.is_fullscreen
        self.root.attributes("-fullscreen", self.is_fullscreen)
        if not self.is_fullscreen:
            try:
                self.root.state("zoomed")
            except tk.TclError:
                pass
        return "break"

    def exit_fullscreen(self, _event=None) -> str:
        self.is_fullscreen = False
        self.root.attributes("-fullscreen", False)
        try:
            self.root.state("zoomed")
        except tk.TclError:
            pass
        return "break"

    def close(self) -> None:
        self.disconnect_all()
        self.root.after(50, self.root.destroy)


def main() -> None:
    root = tk.Tk()
    GuiApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()