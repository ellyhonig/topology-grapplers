#!/usr/bin/env python3
"""Stream generic SteamVR tracker poses to the hosted solver demo.

Install the one native dependency with `python -m pip install openvr`, start
SteamVR, then run this script on the same PC as the browser.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import http.server
import json
import math
import os
from pathlib import Path
import socket
import socketserver
import ssl
import struct
import subprocess
import threading
import time
from typing import Iterable
from urllib.parse import urlsplit
import webbrowser


WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
FIREBASE_ORIGINS = {
    "https://grapplemap-solver-vr.web.app",
    "https://grapplemap-solver-vr.firebaseapp.com",
}


def is_allowed_origin(origin: str) -> bool:
    if origin in FIREBASE_ORIGINS:
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
    allow_reuse_address = False
    daemon_threads = True

    def server_bind(self) -> None:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

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


def ensure_tls_certificate() -> tuple[Path, Path]:
    """Create and trust a per-user localhost certificate for the hosted page."""
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import rsa
        from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
    except ImportError as error:
        raise RuntimeError(
            "missing dependency: run `python -m pip install cryptography`"
        ) from error

    import datetime
    import ipaddress

    local_data = os.environ.get("LOCALAPPDATA")
    certificate_dir = ((Path(local_data) if local_data else Path.home()) /
                       "GrappleMap" / "steamvr-bridge")
    certificate_dir.mkdir(parents=True, exist_ok=True)
    certificate_path = certificate_dir / "localhost-cert.pem"
    certificate_der_path = certificate_dir / "localhost-cert.cer"
    key_path = certificate_dir / "localhost-key.pem"

    if not certificate_path.exists() or not certificate_der_path.exists() or not key_path.exists():
        key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        subject = issuer = x509.Name([
            x509.NameAttribute(NameOID.COMMON_NAME, "GrappleMap SteamVR Bridge"),
        ])
        now = datetime.datetime.now(datetime.timezone.utc)
        certificate = (
            x509.CertificateBuilder()
            .subject_name(subject)
            .issuer_name(issuer)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - datetime.timedelta(days=1))
            .not_valid_after(now + datetime.timedelta(days=1825))
            .add_extension(
                x509.SubjectAlternativeName([
                    x509.DNSName("localhost"),
                    x509.IPAddress(ipaddress.ip_address("127.0.0.1")),
                ]),
                critical=False,
            )
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(
                x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]),
                critical=False,
            )
            .sign(key, hashes.SHA256())
        )
        certificate_path.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))
        certificate_der_path.write_bytes(certificate.public_bytes(serialization.Encoding.DER))
        key_path.write_bytes(key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption(),
        ))
        os.chmod(key_path, 0o600)

    result = subprocess.run(
        ["certutil", "-user", "-addstore", "Root", str(certificate_der_path)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError("could not trust the localhost certificate: " + result.stderr.strip())
    return certificate_path, key_path


def start_secure_server(host: str, port: int) -> TrackerWebSocketServer:
    certificate_path, key_path = ensure_tls_certificate()
    server = TrackerWebSocketServer((host, port))
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certificate_path, key_path)
    server.socket = context.wrap_socket(server.socket, server_side=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


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


class DemoFileHandler(http.server.SimpleHTTPRequestHandler):
    """Serve the repository root with WebAssembly-safe content types."""

    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".wasm": "application/wasm",
    }

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, format: str, *args: object) -> None:
        pass


class DemoHttpServer(http.server.ThreadingHTTPServer):
    allow_reuse_address = False

    def server_bind(self) -> None:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def open_demo_browser(url: str) -> None:
    """Prefer Chromium browsers, which provide desktop OpenXR WebXR support."""
    program_files = [os.environ.get("PROGRAMFILES"), os.environ.get("PROGRAMFILES(X86)")]
    relative_paths = [
        Path("Google/Chrome/Application/chrome.exe"),
        Path("Microsoft/Edge/Application/msedge.exe"),
    ]
    for base in program_files:
        if not base:
            continue
        for relative in relative_paths:
            browser = Path(base) / relative
            if browser.is_file():
                try:
                    subprocess.Popen([str(browser), url])
                    return
                except OSError:
                    pass
    webbrowser.open(url)


def start_demo_server(port: int, open_browser: bool) -> http.server.ThreadingHTTPServer:
    repository_root = Path(__file__).resolve().parent.parent

    def handler(*args, **kwargs):
        return DemoFileHandler(*args, directory=str(repository_root), **kwargs)

    server = DemoHttpServer(("127.0.0.1", port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://localhost:{port}/src/solver-demo.html"
    print(f"GrappleMap demo listening on {url}")
    if open_browser:
        threading.Timer(0.4, lambda: open_demo_browser(url)).start()
    return server


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=17373)
    parser.add_argument("--secure-port", type=int, default=17374)
    parser.add_argument("--no-secure", action="store_true", help="disable the HTTPS/WSS Firebase listener")
    parser.add_argument("--hz", type=float, default=60)
    parser.add_argument("--simulate", action="store_true", help="stream two test trackers without SteamVR")
    parser.add_argument("--serve", action="store_true", help="also serve the demo from the repository root")
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--no-browser", action="store_true", help="do not open the demo page with --serve")
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

    demo_server = start_demo_server(args.http_port, not args.no_browser) if args.serve else None
    server = TrackerWebSocketServer((args.host, args.port))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    secure_server = None
    if not args.no_secure:
        try:
            secure_server = start_secure_server(args.host, args.secure_port)
        except RuntimeError as error:
            server.shutdown()
            server.server_close()
            parser.error(str(error))
    print(f"SteamVR tracker bridge listening on ws://{args.host}:{args.port}")
    if secure_server is not None:
        print(f"Secure Firebase bridge listening on wss://{args.host}:{args.secure_port}")
    print("Press Ctrl+C to stop.")
    started = time.monotonic()
    try:
        while True:
            elapsed = time.monotonic() - started
            trackers = (simulated_tracker_poses(elapsed) if args.simulate else
                        steamvr_tracker_poses(openvr, vr_system))
            payload = {"type": "poses", "trackers": trackers}
            server.broadcast(payload)
            if secure_server is not None:
                secure_server.broadcast(payload)
            time.sleep(max(0.001, 1 / max(1, args.hz)))
    except KeyboardInterrupt:
        return 0
    finally:
        server.shutdown()
        server.server_close()
        if secure_server is not None:
            secure_server.shutdown()
            secure_server.server_close()
        if demo_server is not None:
            demo_server.shutdown()
            demo_server.server_close()
        if openvr is not None:
            openvr.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
