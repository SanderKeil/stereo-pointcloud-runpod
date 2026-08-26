#!/usr/bin/env python3
"""Serve a live MJPEG stream from a webcam so you can watch the robot in a browser.

Run this on whatever machine has physical access to the webcam (e.g. the
robot's onboard computer), then open http://<that machine's IP>:<port>/
from any device on the network.

Usage:
    python webcam_viewer.py --device 0 --port 8080
    python webcam_viewer.py --device /dev/video2
    python webcam_viewer.py --device rtsp://192.168.1.50/stream
"""
import argparse
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = "frame"

INDEX_HTML = b"""<!doctype html>
<html>
  <head><title>Robot Webcam</title></head>
  <body style="margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh;">
    <img src="/stream" style="max-width:100%;max-height:100%;" />
  </body>
</html>
"""


class Camera:
    def __init__(self, device, width, height, fps):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open webcam device {device!r}")
        if width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self.lock:
                    self.frame = jpg.tobytes()

    def get_jpeg(self):
        with self.lock:
            return self.frame

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.cap.release()


class StreamHandler(BaseHTTPRequestHandler):
    camera = None  # set on the class by main() before the server starts

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(INDEX_HTML)
        elif self.path == "/stream":
            self._stream_mjpeg()
        else:
            self.send_error(404)

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        try:
            while True:
                jpg = self.camera.get_jpeg()
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(1 / 30)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="0", help="Webcam index, /dev/video path, or stream URL (default: 0)")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port to serve on (default: 8080)")
    parser.add_argument("--width", type=int, default=0, help="Requested capture width (0 = camera default)")
    parser.add_argument("--height", type=int, default=0, help="Requested capture height (0 = camera default)")
    parser.add_argument("--fps", type=int, default=0, help="Requested capture FPS (0 = camera default)")
    args = parser.parse_args()

    device = int(args.device) if args.device.isdigit() else args.device
    camera = Camera(device, args.width, args.height, args.fps)
    StreamHandler.camera = camera

    server = ThreadingHTTPServer(("0.0.0.0", args.port), StreamHandler)
    try:
        host_ip = socket.gethostbyname(socket.gethostname())
    except socket.gaierror:
        host_ip = "127.0.0.1"
    print(f"Serving webcam view at http://{host_ip}:{args.port}/  (Ctrl+C to stop)")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        camera.close()


if __name__ == "__main__":
    main()
