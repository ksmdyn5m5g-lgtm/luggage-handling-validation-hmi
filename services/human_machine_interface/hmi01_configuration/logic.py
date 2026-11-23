"""
hmi01_configuration - builds the Luggage Handling System HMI data model.

Reads the output of every AG4NBA service the HMI displays and folds it into
one JSON snapshot for hmi02_dashboard. Read-only by construction: XREVRANGE /
GET only, never XREADGROUP, so it creates no consumer groups and leaves no
pending entries on streams owned by other services.

Each section is built independently and records its own error, so one
malformed stream blanks one panel, never the whole HMI.
"""
import json
import logging
import time
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import redis

import config

logger = logging.getLogger("Logic")

# Never XREVRANGE these: one entry is a raw 4K frame or a full point cloud
# (~25 MB). Their freshness comes from per07's status instead.
_BINARY_ROLES = {"cam1", "cam2", "lidar"}


def _key(role: str) -> str:
    return config.SOURCES[role][0]


def _id_key(entry_id) -> Tuple[int, int]:
    """Stream ids as (ms, seq) integers. Comparing the raw bytes is wrong
    whenever seq changes digit count (b"...-10" sorts before b"...-9")."""
    if isinstance(entry_id, bytes):
        entry_id = entry_id.decode()
    ms, _, seq = entry_id.partition("-")
    return int(ms), int(seq or 0)


def _id_ts(entry_id) -> float:
    return _id_key(entry_id)[0] / 1000.0


def _fields(f: Dict[bytes, bytes]) -> Dict[str, str]:
    return {k.decode(): v.decode("utf-8", "replace") for k, v in f.items() if k != b"payload" or len(v) < 65536}


def _payload(f: Dict[bytes, bytes]) -> Dict[str, Any]:
    try:
        return json.loads(f[b"payload"])
    except (KeyError, ValueError):
        return {}


class ServiceLogic:
    """Builds one HMI snapshot per call to process()."""

    def __init__(self, redis_client: redis.Redis):
        self.r = redis_client
        self._history = deque(maxlen=config.HISTORY_MINUTES)  # [minute, detections, tracks, validations]
        self._last_seen: Dict[str, Tuple[int, int]] = {}
        self._started = time.time()
        self._per07: Optional[Dict[str, Any]] = None
        logger.info(f"Logic initialized: {len(config.SOURCES)} data sources.")

    # ---- public --------------------------------------------------------
    def process(self) -> Dict[str, Any]:
        now = time.time()
        self._per07 = self._read_per07_status()
        s: Dict[str, Any] = {}
        self._guard(s, "streams", lambda: self._streams(now))
        self._guard(s, "detection", lambda: self._detection(now))
        self._guard(s, "dimension", self._dimension)
        self._guard(s, "characterization", self._characterization)
        self._guard(s, "bbox3d", self._bbox3d)
        self._guard(s, "tracking", self._tracking)
        self._guard(s, "validation", lambda: self._validation(now))
        self._guard(s, "alerts", self._alerts)
        self._guard(s, "optimization", self._optimization)
        self._guard(s, "luggage_items", self._luggage_items)
        self._guard(s, "profiles", self._profiles)
        self._guard(s, "health", self._health)
        self._guard(s, "history", lambda: self._roll_history(now, s))
        window, max_cycle = self._rules()
        return {"generated_at": now, "uptime_s": round(now - self._started, 1),
                "stale_after_s": config.STALE_AFTER_SEC, "transit_window_s": window,
                "max_cycle": max_cycle, "per07_running": self._per07 is not None, "sections": s}

    def configuration(self) -> Dict[str, Any]:
        """What the HMI is configured to show, published as-is."""
        window, max_cycle = self._rules()
        return {"sources": [{"role": role, "key": key, "owner": owner, "label": label}
                            for role, (key, owner, label) in config.SOURCES.items()],
                "refresh_s": config.LOOP_DELAY_SEC, "stale_after_s": config.STALE_AFTER_SEC,
                "history_minutes": config.HISTORY_MINUTES,
                "belt_m": {"length": config.BELT_LENGTH_M, "width": config.BELT_WIDTH_M},
                "transit_window_s": window, "max_cycle": max_cycle}

    # ---- helpers -------------------------------------------------------
    def _read_per07_status(self) -> Optional[Dict[str, Any]]:
        raw = self.r.get(config.PER07_STATUS_KEY)
        if not raw:
            return None
        try:
            return json.loads(raw)
        except ValueError:
            logger.warning(f"{config.PER07_STATUS_KEY} is not valid JSON - ignored")
            return None

    def _rules(self) -> Tuple[List[float], int]:
        p = self._per07 or {}
        window = p.get("transit_window_s") or list(config.DEFAULT_TRANSIT_WINDOW_SEC)
        return [float(window[0]), float(window[1])], int(p.get("max_cycle") or config.DEFAULT_MAX_CYCLE)

    @staticmethod
    def _state(age: Optional[float]) -> str:
        return "live" if age is not None and age <= config.STALE_AFTER_SEC else "stale"

    def _latest(self, role: str, count: int = 1):
        return self.r.xrevrange(_key(role), count=count)

    def _new_since(self, role: str, entries) -> int:
        """Counts entries newer than the last poll, for the rolling history."""
        last = self._last_seen.get(role)
        # First poll after start: the retained backlog is history, not new work.
        fresh = [] if last is None else [e for e in entries if _id_key(e[0]) > last]
        if entries:
            self._last_seen[role] = max(_id_key(e[0]) for e in entries)
        return len(fresh)

    def _guard(self, sections: Dict[str, Any], name: str, fn) -> None:
        try:
            sections[name] = fn()
        except Exception as e:  # one bad stream must not blank the HMI
            sections[name] = {"error": f"{type(e).__name__}: {e}"}

    # ---- sections ------------------------------------------------------
    def _streams(self, now: float) -> List[Dict[str, Any]]:
        out = []
        inputs = (self._per07 or {}).get("last_input_ts") or {}
        for role, (key, owner, label) in config.SOURCES.items():
            row = {"role": role, "key": key, "owner": owner, "label": label}
            if not self.r.exists(key):
                row.update(state="absent", length=0, age_s=None, last_id=None)
                out.append(row)
                continue
            if role in _BINARY_ROLES:
                ts = inputs.get(role)
                if ts is not None:
                    age = now - float(ts)
                    row.update(length=self.r.xlen(key), last_id=None, age_s=round(age, 1),
                               source="per07_status", state=self._state(age))
                    out.append(row)
                    continue
                # per07 not running, or it has not received this input yet: fall
                # back to consumer-group bookkeeping. Approximate - a group that
                # stops reading freezes its last-delivered-id.
                row["approx"] = True
                ids = [g.get("last-delivered-id") for g in self.r.xinfo_groups(key) if g.get("last-delivered-id")]
                newest = max((_id_key(i) for i in ids), default=None)
                last_id = f"{newest[0]}-{newest[1]}" if newest else None
            else:
                last = self.r.xrevrange(key, count=1)
                last_id = last[0][0].decode() if last else None
            age = now - _id_ts(last_id) if last_id else None
            row.update(length=self.r.xlen(key), last_id=last_id,
                       age_s=None if age is None else round(age, 1), state=self._state(age))
            out.append(row)
        return out

    def _detection(self, now: float) -> Dict[str, Any]:
        entries = self._latest("detection", 200)
        edges = [e for e in entries if _fields(e[1]).get("detected") == "True"]
        last = _fields(entries[0][1]) if entries else {}
        return {"beam": 1 if last.get("detected") == "True" else 0,
                "edges_last_60s": sum(1 for e in edges if now - _id_ts(e[0]) <= 60),
                "recent": [1 if _fields(e[1]).get("detected") == "True" else 0 for e in entries[:18]][::-1],
                "_new": self._new_since("detection", edges)}

    def _dimension(self) -> Dict[str, Any]:
        rows = []
        for eid, f in self._latest("dimension", 10):
            d = _fields(f)
            rows.append({"ts": _id_ts(eid), "l": d.get("length_cm"), "w": d.get("width_cm"),
                         "h": d.get("height_cm"), "source": d.get("source_file")})
        return {"latest": rows[0] if rows else None, "recent": rows}

    def _characterization(self) -> Dict[str, Any]:
        e = self._latest("characterization", 1)
        if not e:
            return {"latest": None}
        p = _payload(e[0][1])
        objs = [{"class": o.get("class_name"), "conf": round(o.get("confidence") or 0, 3),
                 "handles": len((o.get("parts") or {}).get("handles") or []),
                 "wheels": len((o.get("parts") or {}).get("wheels") or [])} for o in p.get("objects") or []]
        return {"latest": {"ts": _id_ts(e[0][0]), "objects": objs[:8],
                           "num_main_objects": len(objs), "num_parts": (p.get("meta") or {}).get("num_parts", 0)}}

    def _bbox3d(self) -> Dict[str, Any]:
        e = self._latest("bbox3d", 1)
        if not e:
            return {"latest": None}
        p = _payload(e[0][1])
        return {"latest": {"ts": _id_ts(e[0][0]), "yaw_deg": p.get("yaw_deg"),
                           "dims_cm": p.get("dimensions_cm"), "points": p.get("point_count")}}

    def _tracking(self) -> Dict[str, Any]:
        entries = self._latest("tracking", 100)
        seen = set()
        enrolled = unresolved = 0
        for _eid, f in entries:
            for t in _payload(f).get("tracks") or []:
                k = t.get("luggage_id") or f"trk{t.get('raw_tracker_id')}"
                if k in seen:
                    continue
                seen.add(k)
                if t.get("luggage_id"):
                    enrolled += 1
                else:
                    unresolved += 1
        latest = _payload(entries[0][1]) if entries else {}
        tracks = [{"luggage_id": t.get("luggage_id"), "tracker_id": t.get("raw_tracker_id"),
                   "motion_id": t.get("detector_motion_id"),  # set when no tracker has picked the bag up yet
                   "class": t.get("class_name"), "conf": round(t.get("confidence") or 0, 3),
                   "zone": t.get("active_zone"), "events": t.get("events") or [],
                   "identity": t.get("identity_state"), "progress": t.get("route_progress")}
                  for t in latest.get("tracks") or []]
        return {"latest_ts": _id_ts(entries[0][0]) if entries else None, "tracks": tracks,
                "distinct_recent": len(seen), "enrolled_recent": enrolled, "unenrolled_recent": unresolved,
                "_new": self._new_since("tracking", entries)}

    def _validation(self, now: float) -> Dict[str, Any]:
        entries = self._latest("validation", 50) if self.r.exists(_key("validation")) else []
        results = [dict(_fields(f), ts=_id_ts(eid)) for eid, f in entries]
        (lo, hi), _max_cycle = self._rules()
        in_flight = []
        for b in (self._per07 or {}).get("in_flight") or []:
            t0 = b.get("t0")
            el = now - float(t0) if t0 is not None else 0.0
            in_flight.append({"baggage_id": b.get("baggage_id"), "class": b.get("class"),
                              "dims_m": b.get("dims_m"), "elapsed_s": round(el, 1), "cycles": b.get("cycles", 0),
                              "phase": "transit" if el < lo else "window" if el <= hi else "overdue"})
        return {"results": results[:20], "in_flight": in_flight, "per07_running": self._per07 is not None,
                "matched": sum(1 for x in results if x.get("status") == config.STATUS_MATCHED),
                "total": len(results), "_new": self._new_since("validation", entries)}

    def _alerts(self) -> List[Dict[str, Any]]:
        out = []
        if self.r.exists(_key("alerts")):
            for eid, f in self._latest("alerts", 20):
                d = _fields(f)
                out.append({"source": "per07", "code": d.get("reason"), "severity": "CRITICAL",
                            "baggage_id": d.get("baggage_id"), "ts": _id_ts(eid),
                            "text": f"Not re-identified at Zone 2 after {d.get('cycles_attempted')} cycle(s)."})
        for eid, f in self._latest("ems_inspection", 20):
            p = _payload(f)
            out.append({"source": "ems02", "code": p.get("error_code"), "severity": "WARNING",
                        "baggage_id": p.get("baggage_id"), "ts": _id_ts(eid),
                        "text": f"{p.get('quality_output', 'inspection')} - {p.get('status', '')}, "
                                f"retries {p.get('retry_count', 0)}."})
        return sorted(out, key=lambda a: a["ts"], reverse=True)[:25]

    def _optimization(self) -> Dict[str, Any]:
        inv = self.r.get(config.CART_INVENTORY_KEY)
        state = self.r.get(config.CART_STATE_KEY)
        plan = self._latest("cart_plan", 1)
        summary = None
        if plan:
            p = json.loads(_fields(plan[0][1]).get("plan") or "{}")
            carts = [c for cs in (p.get("flights") or {}).values() for c in cs]
            summary = {"ts": _id_ts(plan[0][0]), "algorithm": p.get("algorithm"),
                       "flights": len(p.get("flights") or {}), "carts": len(carts),
                       "bags_placed": sum(len(c.get("items") or []) for c in carts),
                       "mean_util": round(sum(c.get("util") or 0 for c in carts) / len(carts), 3) if carts else None}
        return {"inventory": json.loads(inv) if inv else None,
                "state": json.loads(state) if state else None, "plan": summary}

    def _luggage_items(self) -> List[Dict[str, Any]]:
        return [dict(_fields(f), ts=_id_ts(eid)) for eid, f in self._latest("luggage_items", 20)
                if _fields(f).get("event") == "bag"]

    def _profiles(self) -> List[Dict[str, Any]]:
        return [dict(_payload(f), ts=_id_ts(eid)) for eid, f in self._latest("profile", 20)]

    def _health(self) -> Dict[str, Any]:
        out = {}
        for role in ("health_cam1", "health_cam2", "health_lidar"):
            e = self._latest(role, 1)
            p = _payload(e[0][1]) if e else {}
            out[role] = {"sensor": p.get("sensor"), "host": p.get("hostname"), "status": p.get("status"),
                         "stream_active": p.get("stream_active"), "stream_age_s": p.get("stream_age_sec"),
                         "ts": _id_ts(e[0][0]) if e else None}
        return out

    def _roll_history(self, now: float, s: Dict[str, Any]) -> List[List[int]]:
        minute = int(now // 60)
        add = [int((s.get(k) or {}).get("_new", 0) or 0) for k in ("detection", "tracking", "validation")]
        if self._history and self._history[-1][0] == minute:
            for i, v in enumerate(add, start=1):
                self._history[-1][i] += v
        else:
            self._history.append([minute] + add)
        return [list(b) for b in self._history]
