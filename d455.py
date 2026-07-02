#!/usr/bin/env python3
import argparse
import os
import queue
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


PROJECT_QR_LENGTH = 12
PROJECT_QR_ALLOWED_CHARS = frozenset("0123")


def is_project_qr(data):
    data = "" if data is None else str(data).strip()
    return (
        len(data) == PROJECT_QR_LENGTH
        and all(char in PROJECT_QR_ALLOWED_CHARS for char in data)
    )


def start_pipeline(width, height, fps):
    if rs is None:
        raise RuntimeError("pyrealsense2 is not installed")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color,
        int(width),
        int(height),
        rs.format.bgr8,
        int(fps),
    )
    print(f"Starting RealSense color stream: {width}x{height}@{fps}")
    profile = pipeline.start(config)
    device = profile.get_device()
    device_name = device.get_info(rs.camera_info.name)
    serial = device.get_info(rs.camera_info.serial_number)
    print(f"Connected RealSense device: {device_name} (S/N: {serial})")
    return pipeline, {
        "width": int(width),
        "height": int(height),
        "fps": int(fps),
        "device_name": device_name,
        "serial": serial,
    }


def draw_qr_points(frame, points, ok):
    if points is None:
        return
    pts = np.asarray(points, dtype=np.int32).reshape(-1, 2)
    color = (0, 220, 0) if ok else (0, 165, 255)
    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)


class D455QRScanner:
    def __init__(
        self,
        result_queue=None,
        width=640,
        height=480,
        fps=30,
        warmup_frames=15,
        stable_frame_count=5,
        show_window=True,
        window_name="D455 QR Test",
        stop_after_success=False,
        loop_interval_sec=0.01,
    ):
        self.result_queue = result_queue if result_queue is not None else queue.Queue()
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.warmup_frames = int(warmup_frames)
        self.stable_frame_count = int(stable_frame_count)
        self.show_window = bool(show_window)
        self.window_name = str(window_name)
        self.stop_after_success = bool(stop_after_success)
        self.loop_interval_sec = float(loop_interval_sec)

        self.stop_event = threading.Event()
        self.done_event = threading.Event()
        self.thread = None
        self.last_error = None
        self.last_qr_data = None
        self.last_stable_count = 0
        self.frame_count = 0
        self.stream_info = None

    def start(self):
        if self.thread is not None and self.thread.is_alive():
            return self.thread
        self.stop_event.clear()
        self.done_event.clear()
        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="d455_qr_scanner",
        )
        self.thread.start()
        return self.thread

    def stop(self):
        self.stop_event.set()

    def join(self, timeout=None):
        if self.thread is not None:
            self.thread.join(timeout=timeout)
        return not (self.thread is not None and self.thread.is_alive())

    def _run(self):
        pipeline = None
        detector = cv2.QRCodeDetector()
        stable_data = None
        stable_count = 0
        last_printed_data = None
        start_time = time.time()

        try:
            pipeline, self.stream_info = start_pipeline(
                width=self.width,
                height=self.height,
                fps=self.fps,
            )
            print("Warming up camera...")
            for _ in range(max(0, self.warmup_frames)):
                pipeline.wait_for_frames()

            print("QR scanner thread started. Press q or Esc in window to exit.")
            print("Project QR format: 12 chars, only 0/1/2/3.")
            print(f"Active stream: {self.stream_info}")

            while not self.stop_event.is_set():
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    print("No color frame in frameset")
                    time.sleep(self.loop_interval_sec)
                    continue

                self.frame_count += 1
                frame = np.asanyarray(color_frame.get_data())
                data, points, _ = detector.detectAndDecode(frame)
                data = "" if data is None else str(data).strip()
                ok = is_project_qr(data)

                if data:
                    if data == stable_data:
                        stable_count += 1
                    else:
                        stable_data = data
                        stable_count = 1
                else:
                    stable_data = None
                    stable_count = 0

                self.last_qr_data = stable_data
                self.last_stable_count = stable_count

                if data and data != last_printed_data:
                    print(f"QR detected: {data!r} project_format={ok}")
                    last_printed_data = data

                if stable_data is not None and stable_count >= self.stable_frame_count:
                    result = {
                        "qr_data": stable_data,
                        "project_format": is_project_qr(stable_data),
                        "stable_count": int(stable_count),
                        "frame_count": int(self.frame_count),
                    }
                    self.result_queue.put(result)
                    print(f"Stable QR result: {result}")
                    if self.stop_after_success:
                        self.stop_event.set()

                draw_qr_points(frame, points, ok)
                cv2.putText(
                    frame,
                    "QR: " + (data if data else "none"),
                    (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 220, 0) if ok else (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )
                elapsed = max(0.001, time.time() - start_time)
                cv2.putText(
                    frame,
                    f"frames={self.frame_count} fps={self.frame_count / elapsed:.1f}",
                    (20, 64),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"stable={stable_count}/{self.stable_frame_count}",
                    (20, 96),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                if self.show_window:
                    cv2.imshow(self.window_name, frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        self.stop_event.set()
                        break

                time.sleep(self.loop_interval_sec)
        except Exception as exc:
            self.last_error = exc
            print(f"QR scanner thread error: {exc!r}")
        finally:
            if pipeline is not None:
                pipeline.stop()
            if self.show_window:
                cv2.destroyWindow(self.window_name)
            self.done_event.set()
            print("D455 QR scanner thread stopped.")


def main():
    parser = argparse.ArgumentParser(
        description="Simple D455/D435i RealSense QR code test."
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--stable-frame-count", type=int, default=5)
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--stop-after-success", action="store_true")
    args = parser.parse_args()

    result_queue = queue.Queue()
    scanner = D455QRScanner(
        result_queue=result_queue,
        width=args.width,
        height=args.height,
        fps=args.fps,
        warmup_frames=args.warmup_frames,
        stable_frame_count=args.stable_frame_count,
        show_window=not args.no_window,
        stop_after_success=args.stop_after_success,
    )
    try:
        scanner.start()
        while not scanner.done_event.is_set():
            try:
                result = result_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            print(f"Main thread received QR result: {result}")
    except KeyboardInterrupt:
        print("\nStopping scanner...")
    finally:
        scanner.stop()
        scanner.join(timeout=2.0)
        if scanner.last_error is not None:
            raise scanner.last_error
        print("D455 QR test stopped.")


if __name__ == "__main__":
    main()
