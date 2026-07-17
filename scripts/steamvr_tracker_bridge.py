#!/usr/bin/env python3
"""Stream generic SteamVR tracker poses to the hosted solver demo.

Install the one native dependency with `python -m pip install openvr`, start
SteamVR, then run this script on the same PC as the browser.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import socketserver
import struct
import threading
import time
from typing import Iterable
from urllib.parse import urlsplit


WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def is_allowed_origin(origin: str) -> bool:
    if origin == "https://grapplemap-solver-vr.web.app":
        return True
    parsed = urlsplit(origin)
    return parsed.scheme in ("http", "https") and parsed.hostname in ("127.0.0.1", "localhost")


def quaternion_from_matrix(matrix: Iterable[Iterable[float]]) -> list[float]:
    """Return an x/y/z/w quaternion from a SteamVR 3x4 pose matrix."""
    m = [list(row) for row in matrix]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        return [
            (m[2][1] - m[1][2]) / scale,
            (m[0][2] - m[2][0]) / scale,
            (m[1][0] - m[0][1]) / scale,
            0.25 * scale,
        ]
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return [
            0.25 * scale,
            (m[0][1] + m[1][0]) / scale,
            (m[0][2] + m[2][0]) / scale,
            (m[2][1] - m[1][2]) / scale,
        ]
    if m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return [
            (m[0][1] + m[1][0]) / scale,
            0.25 * scale,
            (m[1][2] + m[2][1]) / scale,
            (m[0][2] - m[2][0]) / scale,
        ]
    scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return [
        (m[0][2] + m[2][0]) / scale,
        (m[1][2] + m[2][1]) / scale,
        0.25 * scale,
        (m[1][0] - m[0][1]) / scale,
    ]


def websocket_text_frame(payload: str) -> bytes:
    encoded = payload.encode("utf-8")
    length = len(encoded)
    if length < 126:
        header = bytes((0x81, length))
    elif length <= 0xFFFF:
        header = bytes((0x81, 126)) + struct.pack("!H", length)
    else:
        header = bytes((0x81, 127)) + struct.pack("!Q", length)
    return header + encoded


class TrackerWebSocketServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int]):
        super().__init__(address, TrackerWebSocketHandler)
        self.clients: set[object] = set()
        self.clients_lock = threading.Lock()

    def broadcast(self, payload: dict) -> None:
        frame = websocket_text_frame(json.dumps(payload, separators=(",", ":")))
        with self.clients_lock:
            clients = list(self.clients)
        for client in clients:
            try:
                client.sendall(frame)
            except OSError:
                self.remove_client(client)

    def remove_client(self, client: object) -> None:
        with self.clients_lock:
            self.clients.discard(client)
        try:
            client.close()
        except OSError:
            pass


class TrackerWebSocketHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        request = b""
        while b"\r\n\r\n" not in request and len(request) < 16384:
            try:
                chunk = self.request.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            request += chunk
        request_lines = request.decode("latin-1").split("\r\n")
        request_parts = request_lines[0].split()
        method = request_parts[0] if request_parts else ""
        path = request_parts[1] if len(request_parts) > 1 else ""
        headers = {}
        for line in request_lines[1:]:
            if ":" in line:
                name, value = line.split(":", 1)
                headers[name.strip().lower()] = value.strip()
        origin = headers.get("origin", "")
        if not is_allowed_origin(origin):
            self.request.sendall(b"HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n")
            return
        if path == "/health" and method in ("GET", "OPTIONS"):
            body = b'{"ok":true}' if method == "GET" else b""
            self.request.sendall(
                ("HTTP/1.1 200 OK\r\n"
                 "Content-Type: application/json\r\n"
                 f"Access-Control-Allow-Origin: {origin}\r\n"
                 "Vary: Origin\r\n"
                 "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
                 "Access-Control-Allow-Private-Network: true\r\n"
                 "Cache-Control: no-store\r\n"
                 f"Content-Length: {len(body)}\r\n"
                 "Connection: close\r\n\r\n").encode("ascii") + body
            )
            return
        key = headers.get("sec-websocket-key")
        if not key or headers.get("upgrade", "").lower() != "websocket":
            self.request.sendall(b"HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n")
            return
        accept = base64.b64encode(
            hashlib.sha1((key + WEBSOCKET_GUID).encode("ascii")).digest()
        ).decode("ascii")
        self.request.sendall(
            ("HTTP/1.1 101 Switching Protocols\r\n"
             "Upgrade: websocket\r\n"
             "Connection: Upgrade\r\n"
             f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode("ascii")
        )
        server = self.server
        with server.clients_lock:
            server.clients.add(self.request)
        try:
            while self.request.recv(2048):
                pass
        except OSError:
            pass
        finally:
            server.remove_client(self.request)


def steamvr_tracker_poses(openvr, vr_system) -> list[dict]:
    poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
    vr_system.getDeviceToAbsoluteTrackingPose(
        openvr.TrackingUniverseStanding, 0, poses
    )
    trackers = []
    for index, pose in enumerate(poses):
        if (not pose.bDeviceIsConnected or not pose.bPoseIsValid or
                vr_system.getTrackedDeviceClass(index) != openvr.TrackedDeviceClass_GenericTracker):
            continue
        matrix = pose.mDeviceToAbsoluteTracking.m
        serial = vr_system.getStringTrackedDeviceProperty(
            index, openvr.Prop_SerialNumber_String
        )
        trackers.append({
            "id": serial or f"tracker-{index}",
            "position": [matrix[0][3], matrix[1][3], matrix[2][3]],
            "orientation": quaternion_from_matrix(matrix),
        })
    return trackers


def simulated_tracker_poses(elapsed: float) -> list[dict]:
    sway = math.sin(elapsed) * 0.015
    return [
        {"id": "sim-left", "position": [-0.18 + sway, 0.08, -0.55], "orientation": [0, 0, 0, 1]},
        {"id": "sim-right", "position": [0.18 + sway, 0.08, -0.55], "orientation": [0, 0, 0, 1]},
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=17373)
    parser.add_argument("--hz", type=float, default=60)
    parser.add_argument("--simulate", action="store_true", help="stream two test trackers without SteamVR")
    args = parser.parse_args()

    openvr = None
    vr_system = None
    if not args.simulate:
        try:
            import openvr as openvr_module
        except ImportError:
            parser.error("missing dependency: run `python -m pip install openvr`")
        openvr = openvr_module
        openvr.init(openvr.VRApplication_Background)
        vr_system = openvr.VRSystem()

    server = TrackerWebSocketServer((args.host, args.port))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print(f"SteamVR tracker bridge listening on ws://{args.host}:{args.port}")
    print("Press Ctrl+C to stop.")
    started = time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - started
            trackers = (simulated_tracker_poses(elapsed) if args.simulate else
                        steamvr_tracker_poses(openvr, vr_system))
            server.broadcast({"type": "poses", "trackers": trackers})
            time.sleep(max(0.001, 1 / max(1, args.hz)))
    except KeyboardInterrupt:
        return 0
    finally:
        server.shutdown()
        server.server_close()
        if openvr is not None:
            openvr.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
