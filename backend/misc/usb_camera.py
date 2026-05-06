import time
import cv2
from cv2.typing import MatLike
import threading
from threading import Condition
import io


class StreamingOutput(io.BufferedIOBase):
    def __init__(self)-> None:
        self.frame: bytes | None = None
        self.condition: Condition = Condition()

    def write(self, buf: bytes) -> None:
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class USBCamera:
    def __init__(self, device: int = 8, preview_size: tuple[int, int] = (640, 480), stream_size: tuple[int, int] = (400, 300), fps: int = 20) -> None:
        self.device: int = device
        self.preview_size: tuple[int, int] = preview_size
        self.stream_size: tuple[int, int] = stream_size
        self.fps: int = fps

        self.cap: cv2.VideoCapture | None = None
        self.streaming_output: StreamingOutput = StreamingOutput()
        self.debug_output: StreamingOutput = StreamingOutput()
        self.streaming: bool = False

        self._thread: threading.Thread | None = None
        self._stop_event: threading.Event = threading.Event()
        self._writer: cv2.VideoWriter | None = None

    def _open(self):
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open webcam device {self.device}")
            _ = self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            _ = self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            _ = self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.preview_size[0])
            _ = self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.preview_size[1])
            _ = self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def start_image(self):
        self._open()

    def save_image(self, filename: str) -> dict[str, str | float]:
        self._open()
        ok, frame = self.cap.read() 
        if not ok:
            raise RuntimeError("Failed to capture image from USB webcam")
        ok = cv2.imwrite(filename, frame)
        if not ok:
            raise RuntimeError(f"Failed to write image to {filename}")
        return {"filename": filename, "timestamp": time.time()}

    def _stream_loop(self, record: bool = False) -> None:
        while not self._stop_event.is_set():
            ok, frame = self.cap.read()
            if not ok:
                continue

            frame = cv2.resize(frame, self.stream_size)

            if record and self._writer is not None:
                self._writer.write(frame)
            else:
                ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if ok:
                    self.streaming_output.write(jpg.tobytes())

    def set_debug_frame(self, frame: MatLike):
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            with self.debug_output.condition:
                self.debug_output.write(jpg.tobytes())

    def get_debug_frame(self, timeout: float | None = None) -> bytes | None:
        with self.debug_output.condition:
            if timeout is None:
                _ = self.debug_output.condition.wait()
            else:
                _ = self.debug_output.condition.wait(timeout=timeout)
            return self.debug_output.frame

    def start_stream(self, filename: str | None = None) -> None:
        if self.streaming:
            return

        self._open()
        self._stop_event.clear()

        if filename:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(filename, fourcc, self.fps, self.stream_size)
            if not self._writer.isOpened():
                self._writer = None
                raise RuntimeError(f"Failed to open video writer for {filename}")

        self._thread = threading.Thread(
            target=self._stream_loop, kwargs={"record": bool(filename)}, daemon=True
        )
        self._thread.start()
        self.streaming = True

    def stop_stream(self):
        if not self.streaming:
            return

        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

        if self._writer is not None:
            self._writer.release()
            self._writer = None

        self.streaming = False

    def get_frame(self, timeout: float | None = None) -> bytes | None:
        with self.streaming_output.condition:
            if timeout is None:
                _ = self.streaming_output.condition.wait()
            else:
                _ = self.streaming_output.condition.wait(timeout=timeout)
            return self.streaming_output.frame

    def save_video(self, filename: str | None, duration: float = 10.0):
        self.start_stream(filename)
        time.sleep(duration)
        self.stop_stream()

    def close(self):
        if self.streaming:
            self.stop_stream()
        if self.cap is not None:
            self.cap.release()
            self.cap = None