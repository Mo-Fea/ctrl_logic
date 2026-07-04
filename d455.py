#!/usr/bin/env python3
import argparse
import multiprocessing as mp
import os
import queue
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import cv2
import numpy as np

from utils.process import (
    DEFAULT_COLOR_FPS,
    DEFAULT_COLOR_HEIGHT,
    DEFAULT_COLOR_WIDTH,
    DEFAULT_HEAVY_DETECT_INTERVAL,
    DEFAULT_REALSENSE_COLOR_AUTO_EXPOSURE,
    DEFAULT_REALSENSE_COLOR_EXPOSURE,
    DEFAULT_REALSENSE_COLOR_GAIN,
    DEFAULT_ROI_KEEP_FRAMES,
    create_qr_detector,
    detect_qr_realtime,
    get_color_frame,
    is_valid_qr_payload,
    open_d435i,
    qr_points_to_roi,
)


def draw_qr_points(frame, points, ok):
    if points is None:
        return
    pts = np.asarray(points, dtype=np.int32).reshape(-1, 2)
    color = (0, 220, 0) if ok else (0, 165, 255)
    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)


def _scanner_process_main(params, result_queue, status_queue, stop_event, done_event):
    pipeline = None
    detector = create_qr_detector()
    stable_data = None
    stable_count = 0
    last_printed_data = None
    frame_count = 0
    start_time = time.time()
    show_window = bool(params["show_window"])
    window_name = str(params["window_name"])
    preview_scale = float(params["preview_scale"])
    tracked_roi = None
    roi_last_seen_frame = 0
    heavy_detect_interval = max(0, int(params["heavy_detect_interval"]))
    roi_keep_frames = max(0, int(params["roi_keep_frames"]))

    try:
        pipeline = open_d435i(
            width=params["width"],
            height=params["height"],
            fps=params["fps"],
            warmup_frames=params["warmup_frames"],
            color_auto_exposure=params["color_auto_exposure"],
            color_exposure=params["color_exposure"],
            color_gain=params["color_gain"],
        )
        stream_info = {
            "width": int(params["width"]),
            "height": int(params["height"]),
            "fps": int(params["fps"]),
            "source": "utils.process.open_d435i",
        }
        status_queue.put({"type": "stream_info", "stream_info": stream_info})

        print("QR scanner process started. Press q or Esc in window to exit.")
        print("Project QR format: 12 chars, only 0/1/2/3.")
        print(f"Active stream: {stream_info}")

        while not stop_event.is_set():
            frame = get_color_frame(pipeline)
            if frame is None:
                print("No color frame in frameset")
                time.sleep(float(params["loop_interval_sec"]))
                continue

            frame_count += 1
            if (
                tracked_roi is not None
                and roi_keep_frames > 0
                and frame_count - roi_last_seen_frame > roi_keep_frames
            ):
                tracked_roi = None
            use_heavy_fallback = (
                heavy_detect_interval > 0
                and frame_count % heavy_detect_interval == 0
            )
            data, points, detect_path = detect_qr_realtime(
                frame,
                detector=detector,
                tracked_roi=tracked_roi,
                use_heavy_fallback=use_heavy_fallback,
                include_upscaled=bool(params["full_detect"]),
            )
            ok = is_valid_qr_payload(data)
            if points is not None:
                next_roi = qr_points_to_roi(points, frame.shape)
                if next_roi is not None:
                    tracked_roi = next_roi
                    roi_last_seen_frame = frame_count

            if data:
                if data == stable_data:
                    stable_count += 1
                else:
                    stable_data = data
                    stable_count = 1
            else:
                stable_data = None
                stable_count = 0

            status_queue.put({
                "type": "progress",
                "last_qr_data": stable_data,
                "last_stable_count": int(stable_count),
                "frame_count": int(frame_count),
            })

            if data and data != last_printed_data:
                print(f"QR detected: {data!r} project_format={ok} path={detect_path}")
                last_printed_data = data

            if stable_data is not None and stable_count >= int(params["stable_frame_count"]):
                result = {
                    "qr_data": stable_data,
                    "project_format": is_valid_qr_payload(stable_data),
                    "stable_count": int(stable_count),
                    "frame_count": int(frame_count),
                }
                result_queue.put(result)
                print(f"Stable QR result: {result}")
                if bool(params["stop_after_success"]):
                    stop_event.set()

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
                f"frames={frame_count} fps={frame_count / elapsed:.1f}",
                (20, 64),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"stable={stable_count}/{int(params['stable_frame_count'])} path={detect_path}",
                (20, 96),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            if tracked_roi is not None:
                left, top, right, bottom = tracked_roi
                cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 1)

            if show_window:
                if preview_scale > 0.0 and preview_scale != 1.0:
                    preview_frame = cv2.resize(
                        frame,
                        None,
                        fx=preview_scale,
                        fy=preview_scale,
                        interpolation=cv2.INTER_AREA,
                    )
                else:
                    preview_frame = frame
                cv2.imshow(window_name, preview_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    stop_event.set()
                    break

            time.sleep(float(params["loop_interval_sec"]))
    except Exception as exc:
        status_queue.put({
            "type": "error",
            "error_repr": repr(exc),
            "error_type": type(exc).__name__,
        })
        print(f"QR scanner process error: {exc!r}")
    finally:
        if pipeline is not None:
            pipeline.stop()
        if show_window:
            try:
                cv2.destroyWindow(window_name)
            except cv2.error:
                pass
        status_queue.put({"type": "stopped", "frame_count": int(frame_count)})
        done_event.set()
        print("D455 QR scanner process stopped.")


class D455QRScanner:
    def __init__(
        self,
        result_queue=None,
        width=DEFAULT_COLOR_WIDTH,
        height=DEFAULT_COLOR_HEIGHT,
        fps=DEFAULT_COLOR_FPS,
        warmup_frames=15,
        stable_frame_count=2,
        show_window=True,
        window_name="D455 QR Test",
        stop_after_success=False,
        loop_interval_sec=0.01,
        full_detect=False,
        preview_scale=0.4,
        heavy_detect_interval=DEFAULT_HEAVY_DETECT_INTERVAL,
        roi_keep_frames=DEFAULT_ROI_KEEP_FRAMES,
        color_auto_exposure=DEFAULT_REALSENSE_COLOR_AUTO_EXPOSURE,
        color_exposure=DEFAULT_REALSENSE_COLOR_EXPOSURE,
        color_gain=DEFAULT_REALSENSE_COLOR_GAIN,
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
        self.full_detect = bool(full_detect)
        self.preview_scale = float(preview_scale)
        self.heavy_detect_interval = int(heavy_detect_interval)
        self.roi_keep_frames = int(roi_keep_frames)
        self.color_auto_exposure = bool(color_auto_exposure)
        self.color_exposure = None if color_exposure is None else float(color_exposure)
        self.color_gain = None if color_gain is None else float(color_gain)

        self.stop_event = mp.Event()
        self.done_event = mp.Event()
        self.process = None
        self.thread = None
        self._process_result_queue = mp.Queue()
        self._status_queue = mp.Queue()
        self._bridge_stop_event = threading.Event()
        self._bridge_thread = None
        self.last_error = None
        self.last_qr_data = None
        self.last_stable_count = 0
        self.frame_count = 0
        self.stream_info = None

    def start(self):
        if self.process is not None and self.process.is_alive():
            return self.process
        self.stop_event.clear()
        self.done_event.clear()
        self._bridge_stop_event.clear()
        self.last_error = None

        params = {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "warmup_frames": self.warmup_frames,
            "stable_frame_count": self.stable_frame_count,
            "show_window": self.show_window,
            "window_name": self.window_name,
            "stop_after_success": self.stop_after_success,
            "loop_interval_sec": self.loop_interval_sec,
            "full_detect": self.full_detect,
            "preview_scale": self.preview_scale,
            "heavy_detect_interval": self.heavy_detect_interval,
            "roi_keep_frames": self.roi_keep_frames,
            "color_auto_exposure": self.color_auto_exposure,
            "color_exposure": self.color_exposure,
            "color_gain": self.color_gain,
        }
        self.process = mp.Process(
            target=_scanner_process_main,
            args=(
                params,
                self._process_result_queue,
                self._status_queue,
                self.stop_event,
                self.done_event,
            ),
            daemon=True,
            name="d455_qr_scanner_process",
        )
        self.process.start()

        self._bridge_thread = threading.Thread(
            target=self._bridge_child_queues,
            daemon=True,
            name="d455_qr_scanner_bridge",
        )
        self._bridge_thread.start()
        self.thread = self._bridge_thread
        return self.process

    def stop(self):
        self.stop_event.set()

    def join(self, timeout=None):
        if self.process is not None:
            self.process.join(timeout=timeout)
        process_stopped = not (self.process is not None and self.process.is_alive())
        if process_stopped:
            self._bridge_stop_event.set()
        self._drain_status_queue()
        if self._bridge_thread is not None:
            self._bridge_thread.join(timeout=1.0)
        self._drain_status_queue()
        return process_stopped

    def _bridge_child_queues(self):
        while True:
            self._drain_status_queue()

            try:
                result = self._process_result_queue.get(timeout=0.05)
            except queue.Empty:
                result = None
            if result is not None:
                self.result_queue.put(result)

            process_alive = self.process is not None and self.process.is_alive()
            if self._bridge_stop_event.is_set() and not process_alive:
                self._drain_status_queue()
                break

    def _drain_status_queue(self):
        while True:
            try:
                status = self._status_queue.get_nowait()
            except queue.Empty:
                break
            status_type = status.get("type")
            if status_type == "stream_info":
                self.stream_info = status.get("stream_info")
            elif status_type == "progress":
                self.last_qr_data = status.get("last_qr_data")
                self.last_stable_count = int(status.get("last_stable_count", 0))
                self.frame_count = int(status.get("frame_count", 0))
            elif status_type == "error":
                self.last_error = RuntimeError(status.get("error_repr", "scanner error"))
            elif status_type == "stopped":
                self.frame_count = int(status.get("frame_count", self.frame_count))


def main():
    parser = argparse.ArgumentParser(
        description="Simple D455/D435i RealSense QR code test."
    )
    parser.add_argument("--width", type=int, default=DEFAULT_COLOR_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_COLOR_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_COLOR_FPS)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--stable-frame-count", type=int, default=2)
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--stop-after-success", action="store_true")
    parser.add_argument(
        "--full-detect",
        action="store_true",
        help="Allow expensive 2x/3x QR preprocessing in periodic fallback frames.",
    )
    parser.add_argument("--preview-scale", type=float, default=0.4)
    parser.add_argument(
        "--heavy-detect-interval",
        type=int,
        default=DEFAULT_HEAVY_DETECT_INTERVAL,
        help="Run heavy full-frame QR fallback every N frames; 0 disables it.",
    )
    parser.add_argument(
        "--roi-keep-frames",
        type=int,
        default=DEFAULT_ROI_KEEP_FRAMES,
        help="Keep scanning the last QR ROI for this many frames after it was seen.",
    )
    parser.add_argument("--auto-exposure", action="store_true")
    parser.add_argument(
        "--color-exposure",
        type=float,
        default=DEFAULT_REALSENSE_COLOR_EXPOSURE,
    )
    parser.add_argument("--color-gain", type=float, default=DEFAULT_REALSENSE_COLOR_GAIN)
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
        full_detect=args.full_detect,
        preview_scale=args.preview_scale,
        heavy_detect_interval=args.heavy_detect_interval,
        roi_keep_frames=args.roi_keep_frames,
        color_auto_exposure=args.auto_exposure,
        color_exposure=args.color_exposure,
        color_gain=args.color_gain,
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
