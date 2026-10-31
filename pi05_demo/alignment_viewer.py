"""Read-only data feed for RViz. Default mode reads session telemetry only."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import socket
import threading
import time
from urllib.parse import parse_qs, urlsplit

import numpy as np

from .alignment import SOCKET_PATH, alignment_view, tool_transform, validate_sample
from .contract import MAX_WIDTH, vector
from .teleop_reference import j7_offset_from_config

ROOT = Path(__file__).resolve().parents[1]


class SampleStore:
    def __init__(self):
        self.sample = None
        self.lock = threading.Lock()

    def update(self, sample):
        validate_sample(sample)
        with self.lock:
            self.sample = sample

    def read(self):
        with self.lock:
            return self.sample


def demo_sample(config, offsets=None, aperture_mm=48.):
    offset = vector(offsets if offsets is not None else [3, -2, .8, 4, -1.5, 2, 6], 7, "demo offset")
    if np.max(np.abs(offset)) > 15 or not 0 <= aperture_mm <= MAX_WIDTH * 1000:
        raise ValueError("demo inputs out of range")
    q = np.asarray(config["arm_home"], dtype=float)
    return {"source": "demo", "mock": True, "phase": "standby",
            "sample_time": time.monotonic(), "q": q.tolist(),
            "target": (q + np.radians(offset)).tolist(), "width": .06,
            "target_width": aperture_mm / 1000, "reported_ready": False, "held": False}


def make_handler(store, config, tool, mode):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            route = urlsplit(self.path)
            if route.path == "/api/state":
                try:
                    sample = store.read()
                    if mode == "demo":
                        query = parse_qs(route.query)
                        offsets = json.loads(query["offsets"][0]) if "offsets" in query else None
                        sample = demo_sample(config, offsets, float(query.get("width", [48])[0]))
                    state = alignment_view(sample, tool)
                    state["mode"] = mode
                    state["gello_j7_offset_degrees"] = float(np.degrees(j7_offset_from_config(config)))
                    body = json.dumps(state, allow_nan=False).encode()
                except (ValueError, TypeError, KeyError) as exc:
                    self.send_error(400, str(exc))
                    return
                mime = "application/json; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass
    return Handler


def receive_samples(sock, store, stopped):
    sock.settimeout(.2)
    while not stopped.is_set():
        try:
            raw = sock.recv(16384)
            store.update(json.loads(raw))
        except socket.timeout:
            continue
        except (ValueError, TypeError, KeyError):
            continue
        except OSError:
            break


def read_gello_preview(leader):
    """Read the mapped pose for display, including angles beyond Panda limits.

    The pinned normalizer preserves its unclamped, continuous angles internally.
    Copy that diagnostic state only after a successful normalization. Collection
    still uses Gello.read(), which rejects every clipped command.
    """
    raw = vector(leader.agent.act({}), 8, "GELLO reading").copy()
    if not 0 <= raw[7] <= 1:
        raise ValueError("GELLO closure must be in [0,1]")
    raw[6] += leader.j7_offset_rad
    action, outside = leader.normalizer.normalize(raw)
    target = vector(leader.normalizer._continuous, 7, "unclamped GELLO pose").copy()
    return target, float((1 - action[7]) * MAX_WIDTH), outside


def read_home_reference(config, store, stopped, width_mm):
    # Explicit opt-in mode. Importing/starting session/demo modes never opens GELLO.
    from .collect import Gello
    from .teleop_reference import j7_offset_from_config
    leader = None
    try:
        leader = Gello(config["gello_port"], j7_offset_rad=j7_offset_from_config(config))
        while not stopped.is_set():
            target, width, outside = read_gello_preview(leader)
            store.update({"source": "home_reference", "mock": False,
                          "phase": "reference_only", "sample_time": time.monotonic(),
                          "q": config["arm_home"], "target": target.tolist(),
                          "width": None if width_mm is None else width_mm / 1000,
                          "target_width": width, "outside_joint_limits": list(outside),
                          "reported_ready": False, "held": False})
            stopped.wait(.05)
    except Exception as exc:
        print(f"GELLO feedback stopped: {exc}", flush=True)
        # Last sample expires; no fabricated or inferred robot feedback.
    finally:
        if leader is not None: leader.agent._robot._driver.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["session", "demo", "gello-home"], default="session")
    parser.add_argument("--config", default=str(ROOT / "configs/collection.json"))
    parser.add_argument("--tool-profile", default=str(ROOT / "configs/tools/robotiq_2f85_d405/endeffector-config.json"))
    parser.add_argument("--socket", default=SOCKET_PATH)
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--reference-width-mm", type=float, help="optional preset aperture; never a measured gripper state")
    args = parser.parse_args()
    if args.reference_width_mm is not None and not 0 <= args.reference_width_mm <= 85:
        parser.error("reference aperture must be in [0, 85] mm")
    config = json.loads(Path(args.config).read_text())
    j7_offset_from_config(config)
    tool = tool_transform(json.loads(Path(args.tool_profile).read_text()))
    store, stopped = SampleStore(), threading.Event()
    def stop_on_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_on_signal)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(store, config, tool, args.mode))
    sock = None
    socket_identity = None
    try:
        if args.mode == "session":
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            sock.bind(args.socket)  # Never remove/replace another viewer's socket.
            os.chmod(args.socket, 0o600)
            stat = os.stat(args.socket)
            socket_identity = (stat.st_dev, stat.st_ino)
            threading.Thread(target=receive_samples, args=(sock, store, stopped), daemon=True).start()
        elif args.mode == "gello-home":
            threading.Thread(target=read_home_reference,
                             args=(config, store, stopped, args.reference_width_mm), daemon=True).start()
        print(f"RViz data feed: 127.0.0.1:{args.port}; mode={args.mode}; read-only", flush=True)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        server.server_close()
        if sock is not None:
            sock.close()
        if socket_identity is not None:
            try:
                stat = os.stat(args.socket)
                if socket_identity == (stat.st_dev, stat.st_ino):
                    os.unlink(args.socket)
            except FileNotFoundError:
                pass


if __name__ == "__main__":
    main()
