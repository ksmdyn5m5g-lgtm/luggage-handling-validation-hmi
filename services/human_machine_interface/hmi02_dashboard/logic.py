"""
hmi02_dashboard - serves the Luggage Handling System HMI.

Presentation only; everything shown comes from Redis:
  hmi01_snapshot, hmi01_config       built by hmi01_configuration
  per07_live_view_cam1 / _cam2       annotated frames published by per07
  validation:results, error:alerts   per07 verdicts, relayed live over SSE

Stdlib http.server, so there is no web framework to install. Towards Redis it
only reads - GET, HGETALL and plain XREAD, never consumer groups.
"""
import json
import logging
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse, parse_qs

import redis

import config

logger = logging.getLogger("Logic")

_HERE = Path(__file__).resolve().parent
# URL path -> (file next to this module, content type, browser may cache it)
_FILES = {
    "/": ("hmi_lhs.dc.html", "text/html; charset=utf-8", False),
    "/index.html": ("hmi_lhs.dc.html", "text/html; charset=utf-8", False),
    "/support.js": ("support.js", "application/javascript; charset=utf-8", True),
    "/validation": ("hmi_dashboard.html", "text/html; charset=utf-8", False),
    "/popup": ("hmi_popup.html", "text/html; charset=utf-8", False),
}
# The only values allowed into a Redis key name built from the query string.
_VIEWS = ("cam1", "cam2")


def _id_ts(entry_id) -> float:
    if isinstance(entry_id, bytes):
        entry_id = entry_id.decode()
    return int(entry_id.split("-")[0]) / 1000.0


def _num(value, cast=float, default=None):
    try:
        return cast(value)
    except (TypeError, ValueError):
        return default


def _text(fields: Dict[bytes, bytes]) -> Dict[str, str]:
    return {k.decode(): v.decode("utf-8", "replace") for k, v in fields.items()}


def result_event(entry_id, fields: Dict[bytes, bytes]) -> Dict[str, Any]:
    """A validation:results entry as the typed event the pages expect. Stream
    values are all strings, and "False" must not reach the browser as a
    (truthy) string."""
    d = _text(fields)
    try:
        dims = json.loads(d["dimensions_m"]) if d.get("dimensions_m") else None
    except ValueError:
        dims = None
    return {"kind": "result", "baggage_id": d.get("baggage_id"), "status": d.get("status"),
            "final_score": _num(d.get("final_score"), float, 0.0),
            "sift_score": _num(d.get("sift_score"), float, 0.0),
            "class_match": d.get("class_match") == "True",
            "color_score": _num(d.get("color_score"), float, 0.0),
            "cycle_count": _num(d.get("cycle_count"), int, 0),
            "elapsed_transit_s": _num(d.get("elapsed_transit_s")),
            "dimensions_m": dims, "cam2_ts": _num(d.get("cam2_ts")),
            "server_ts": _id_ts(entry_id)}


def alert_event(entry_id, fields: Dict[bytes, bytes]) -> Dict[str, Any]:
    d = _text(fields)
    return {"kind": "alert", "baggage_id": d.get("baggage_id"), "reason": d.get("reason"),
            "cycles_attempted": _num(d.get("cycles_attempted"), int, 0),
            "source_service": d.get("source_service"), "server_ts": _id_ts(entry_id)}


class _Hub:
    """Thread-safe fan-out: one queue per connected /events client."""

    def __init__(self):
        self._lock = threading.Lock()
        self._clients: List["queue.Queue[str]"] = []

    def subscribe(self) -> "queue.Queue[str]":
        q: "queue.Queue[str]" = queue.Queue(maxsize=200)
        with self._lock:
            self._clients.append(q)
        return q

    def unsubscribe(self, q: "queue.Queue[str]") -> None:
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)

    def count(self) -> int:
        with self._lock:
            return len(self._clients)

    def publish(self, event: Dict[str, Any]) -> None:
        line = f"data: {json.dumps(event)}\n\n"
        with self._lock:
            targets = list(self._clients)
        for q in targets:
            try:
                q.put_nowait(line)
            except queue.Full:
                # Never block the relay on a stalled browser.
                logger.warning("An /events client is not keeping up - event dropped for it")


def _make_handler(logic: "ServiceLogic"):
    class Handler(BaseHTTPRequestHandler):
        server_version = "hmi02-dashboard/1.0"

        def log_message(self, fmt, *args):
            logger.debug("%s - %s", self.address_string(), fmt % args)

        def do_GET(self):
            route = urlparse(self.path)
            if route.path in _FILES:
                self._file(*_FILES[route.path])
            elif route.path == "/api/snapshot":
                self._redis_json(config.SNAPSHOT_KEY)
            elif route.path == "/api/config":
                self._redis_json(config.CONFIG_KEY)
            elif route.path == "/frame":
                self._frame(parse_qs(route.query).get("view", ["cam1"])[0])
            elif route.path == "/events":
                self._events()
            else:
                self.send_error(404, "Not found")

        def _send(self, code: int, body: bytes = b"", content_type: Optional[str] = None,
                  cache: bool = False, headers: Optional[Dict[str, str]] = None) -> None:
            self.send_response(code)
            if content_type:
                self.send_header("Content-Type", content_type)
            if code != 204:
                self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "max-age=300" if cache else "no-store")
            for k, v in (headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            try:
                if body:
                    self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # the viewer navigated away mid-response

        def _file(self, name: str, content_type: str, cache: bool) -> None:
            try:
                body = (_HERE / name).read_bytes()
            except FileNotFoundError:
                self.send_error(500, f"{name} is missing from hmi02_dashboard")
                return
            self._send(200, body, content_type, cache)

        def _redis_json(self, key: str) -> None:
            """503 with the reason when the key is missing, so the page can say
            what is not running instead of showing nothing."""
            try:
                raw = logic.r.get(key)
            except redis.RedisError as e:
                self._send(503, json.dumps({"error": f"Redis unavailable: {e}"}).encode(),
                           "application/json", headers={"Retry-After": "2"})
                return
            if raw is None:
                self._send(503, json.dumps({"error": f"no {key} in Redis - is hmi01_configuration running?"}).encode(),
                           "application/json", headers={"Retry-After": "2"})
                return
            self._send(200, raw, "application/json")

        def _frame(self, view: str) -> None:
            """Latest annotated per07 frame. 204 when there is none - normal
            while per07 is stopped or has not rendered that view yet."""
            if view not in _VIEWS:
                self.send_error(404, "Unknown view")
                return
            try:
                entry = logic.r.hgetall(config.LIVE_VIEW_KEY_PREFIX + view)
            except redis.RedisError:
                entry = {}
            jpeg = entry.get(b"jpeg")
            if not jpeg:
                self._send(204)
                return
            ts = _num((entry.get(b"ts") or b"").decode(), float)
            age = max(0.0, time.time() - ts) if ts is not None else 0.0
            self._send(200, jpeg, "image/jpeg", headers={"X-Frame-Age-S": f"{age:.2f}"})

        def _events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")  # some proxies buffer SSE without it
            self.end_headers()
            q = logic.hub.subscribe()
            logger.info(f"/events client connected ({self.client_address[0]})")
            try:
                # Retry hint + an immediate comment so EventSource fires 'open' at once.
                self.wfile.write(b"retry: 2000\n: connected\n\n")
                self.wfile.flush()
                while True:
                    try:
                        self.wfile.write(q.get(timeout=15).encode("utf-8"))
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")  # keeps idle connections open
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                logic.hub.unsubscribe(q)
                logger.info(f"/events client disconnected ({self.client_address[0]})")

    return Handler


class ServiceLogic:
    """The web server plus the live event relay."""

    def __init__(self, redis_client: Optional[redis.Redis] = None):
        self.r = redis_client or redis.Redis(host=config.REDIS_HOST, port=config.REDIS_PORT,
                                             decode_responses=False, socket_timeout=5)
        self.hub = _Hub()
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._relay_thread: Optional[threading.Thread] = None
        logger.info("Logic initialized.")

    def start(self) -> None:
        self._httpd = ThreadingHTTPServer((config.HTTP_HOST, config.HTTP_PORT), _make_handler(self))
        threading.Thread(target=self._httpd.serve_forever, name="hmi02-http", daemon=True).start()
        self._start_relay()
        logger.info(f"Serving the HMI on {config.HTTP_HOST}:{config.HTTP_PORT} "
                    f"(/validation, /popup, /api/snapshot, /api/config, /events)")
        logger.info(f"HMI link (team network and Tailscale): {config.PUBLIC_URL}")

    def process(self) -> Dict[str, Any]:
        """Periodic health check: restarts the live event relay if it died."""
        if self._relay_thread is None or not self._relay_thread.is_alive():
            if self._relay_thread is not None:
                logger.warning("Live event relay stopped - restarting it")
            self._start_relay()
        return {"status": "ok", "event_clients": self.hub.count()}

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()

    def _start_relay(self) -> None:
        self._relay_thread = threading.Thread(target=self._relay, name="hmi02-relay", daemon=True)
        self._relay_thread.start()

    def _tail_id(self, key: str) -> bytes:
        last = self.r.xrevrange(key, count=1)
        return last[0][0] if last else b"0-0"

    def _relay(self) -> None:
        """Relays new per07 verdicts to /events clients. Plain XREAD from the
        current tail - no consumer group, so per07's streams are untouched.
        The tail is resolved to a concrete id once: re-sending "$" on every
        read would silently skip entries published between two reads."""
        converters = {config.STREAM_VALIDATION_RESULTS: result_event, config.STREAM_ERROR_ALERTS: alert_event}
        last: Dict[str, Any] = {}
        while True:
            try:
                if not last:
                    last = {key: self._tail_id(key) for key in converters}
                response = self.r.xread(last, count=50, block=int(config.LOOP_DELAY_SEC * 1000))
            except redis.RedisError as e:
                logger.warning(f"Live event relay: Redis unavailable ({e}) - retrying in 2 s")
                time.sleep(2)
                continue
            for stream, entries in response or []:
                key = stream.decode()
                for entry_id, fields in entries:
                    last[key] = entry_id
                    self.hub.publish(converters[key](entry_id, fields))
