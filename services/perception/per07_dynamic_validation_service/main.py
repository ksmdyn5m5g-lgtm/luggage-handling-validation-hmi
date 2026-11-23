"""
per07_dynamic_validation_service - Redis Streams live entrypoint.

This is what start.py's orchestrator actually launches (via
SERVICE_REGISTRY["per07_dynamic_validation_service"] -> this file, run as a
plain `python main.py` subprocess - matching per01/sns01/sns03 exactly, not
a package import). Consumes Cam1/Cam2/LiDAR frames from the real Redis
streams sns01_sensor_streaming publishes, runs baseline extraction +
dynamic multi-cycle re-id validation via ValidationService, and publishes
results to STREAM_VALIDATION_RESULTS.

CONFIRMED from sns01_sensor_streaming's actual source (not guessed):
  live_lidar, live_camera, live_camera193 - each message shaped
  {"type": "lidar"|"camera", "timestamp": "<ISO 8601 str>",
   "shape": "<numpy shape str>", "payload": <raw bytes>}.
  decode_responses=False is required - payload is binary.

ASSUMED, not confirmed: live_camera = Zone1/Cam1, live_camera193 =
Zone2/Cam2 (see config.STREAM_CAMERA1/STREAM_CAMERA2 for the reasoning and
how to swap them if backwards).

Zone 1 identity and detection both come from per04_luggage_tracking
(STREAM_TRACKING, its `output_redis`), not from per07 itself:
  - per04 owns the permanent business identity `luggage_id` (LUG_000001
    style). per07 baselines an item the first time a new luggage_id
    appears, so one physical item produces one baseline no matter how many
    frames it is tracked across.
  - per04's track carries the pure-YOLO `bbox` ([x1,y1,x2,y2] absolute px)
    and `class_name` for that item, so per07 crops that box instead of
    running a second detection pass over a frame per04 already processed.
per07 still owns the part per04 does not cover: per04 tracks on Cam1 only,
while per07 validates the Cam1 -> Cam2 crossing.

Cam1 frames are buffered here (JPEG-encoded, count-bounded) purely so the
pixels are still available when per04's message about a frame arrives,
which is necessarily after per07 has already seen that frame.

Dimensioning note: the pure-PCD measurement per07 does is provisional.
per02_luggage_dimension owns LiDAR dimensioning and is expected back; when
it returns, per07 should consume its `luggage_dimension` stream instead.
"""
import json
import logging
import signal
import sys
import time
from collections import OrderedDict, deque
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, Optional, Tuple, List

import cv2
import numpy as np
import redis

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import config
from service import ValidationService
from logic import compute_validation_window, decode_camera_payload, decode_lidar_payload, ValidationResult
from visualization import render_tracking_view, render_validation_view

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger("per07-dvs")

run_flag = True


def _signal_handler(sig, frame):
    global run_flag
    logger.info("Shutdown signal intercepted. Terminating after current batch...")
    run_flag = False


def _parse_iso_timestamp(raw: bytes) -> float:
    """sns01 publishes timestamp as datetime.now().isoformat() (naive, no
    tz) - parse back to a comparable Unix epoch float for the window/cap
    arithmetic that already exists in logic.py."""
    return datetime.fromisoformat(raw.decode("utf-8")).timestamp()


class DynamicValidationWorker:
    """Redis Streams event loop wrapping ValidationService.

    Uses a consumer group (not a bare XREAD with a "$"/"only new" cursor) so
    the worker: (a) picks up any backlog still queued from before it started
    or after a restart, instead of silently skipping it, and (b) only drops
    a message from the pending-entries list once it's XACK'd after
    successful processing - a crash mid-message leaves it re-deliverable
    rather than lost. Note sns01 caps each stream at
    REDIS_STREAM_MAXLEN=20 entries (~4s of buffer at 5fps) - a consumer that
    falls behind that far loses frames regardless of consumer-group
    bookkeeping, same as every other consumer of these streams (per01,
    sns03) already accepts."""

    def __init__(self, redis_client: Optional[redis.Redis] = None, consumer_name: str = config.CONSUMER_NAME):
        self.service = ValidationService()
        # decode_responses=False: payload is raw binary, matching
        # sns01/per01/sns03 exactly - a string-mode client would corrupt it.
        self.r = redis_client or redis.Redis(host=config.REDIS_HOST, port=config.REDIS_PORT, decode_responses=False)
        self.consumer_name = consumer_name
        self._streams = (config.STREAM_CAMERA1, config.STREAM_LIDAR, config.STREAM_CAMERA2, config.STREAM_TRACKING)
        for stream in self._streams:
            try:
                # start='0' so a fresh/restarted worker catches up on any
                # backlog rather than silently missing it (start='$' would
                # only deliver messages published after this call). Little
                # backlog to catch up on in practice given the 20-entry cap.
                self.r.xgroup_create(stream, config.CONSUMER_GROUP, id="0", mkstream=True)
            except redis.exceptions.ResponseError as e:
                if "BUSYGROUP" not in str(e):
                    raise  # group already exists - fine, anything else is a real error

        # Cam1 frames, JPEG-encoded, newest last. Count-bounded by the deque
        # itself, so this cannot grow without limit no matter what arrives.
        self._frame_buffer: Deque[Tuple[float, bytes]] = deque(maxlen=config.CAM1_FRAME_BUFFER_SIZE)
        # LiDAR clouds waiting for a tracking message near their timestamp.
        # Age-evicted in _evict_stale_lidar - see PENDING_RETENTION_S.
        self._pending_lidar: List[Tuple[float, "o3d.geometry.PointCloud"]] = []
        self._attempt_counts: Dict[str, int] = {}   # luggage_id -> cycles attempted so far
        # Bounded LRU set of luggage_ids already baselined, so a LUG_ID seen
        # across many consecutive per04 frames only triggers one baseline.
        self._seen_luggage_ids: "OrderedDict[str, float]" = OrderedDict()
        # Starvation diagnostics - see _note_unidentified_track. Counters are
        # cumulative for the life of the worker; the _last_*_warn floats
        # rate-limit the two warnings to STARVATION_WARN_INTERVAL_S apart.
        self._tracks_seen = 0
        self._tracks_without_id = 0
        self._last_starvation_warn = 0.0
        self._last_no_baseline_warn = 0.0
        # Live popup render throttle, per view. Annotating and JPEG-encoding a
        # 4K frame is far too expensive to do at full stream rate purely for a
        # monitoring view, and no operator needs 5fps to see what is happening.
        self._last_view_render: Dict[str, float] = {}
        # Most recent per04 tracks, kept so the Zone 1 popup view can be
        # driven by the Cam1 stream (a reliable 5fps) rather than by per04's
        # own publish rate, which is sparse and bursty - observed live at
        # roughly one message per 30s, which would leave the view blank.
        self._latest_tracks: List[Dict[str, Any]] = []
        self._latest_tracks_ts: Optional[float] = None
        # Newest sensor timestamp received per input ("cam1"/"cam2"/"lidar"),
        # published in STATUS_KEY so the HMI can show camera/LiDAR freshness
        # without pulling a ~25 MB raw frame out of Redis per check.
        self.last_input_ts: Dict[str, float] = {}
        self._last_status_publish = 0.0

    def _render_live_view(self, view: str, ts: float, render) -> None:
        """Rate-limited publish of one annotated view to Redis, where
        hmi02_dashboard's zone view (/popup) picks it up.

        `render` is a zero-arg callable so the annotation work itself is
        skipped entirely - not just its result discarded - whenever this call
        is throttled or live views are switched off."""
        if config.LIVE_VIEW_MAX_FPS <= 0:
            return
        min_gap = 1.0 / config.LIVE_VIEW_MAX_FPS
        if ts - self._last_view_render.get(view, 0.0) < min_gap:
            return
        self._last_view_render[view] = ts
        try:
            jpeg = render()
            if not jpeg:
                return
            key = config.LIVE_VIEW_KEY_PREFIX + view
            pipe = self.r.pipeline()
            pipe.hset(key, mapping={"jpeg": jpeg, "ts": f"{time.time():.3f}"})
            pipe.expire(key, config.LIVE_VIEW_TTL_S)
            pipe.execute()
        except Exception as e:
            # A drawing bug or Redis hiccup must never take down the validation
            # loop - the live view is an observability aid, not the contract.
            logger.warning(f"[LiveView] {view} not published: {e}")

    def _note_unidentified_track(self, track: Dict[str, Any], now_ts: float) -> None:
        """per04 publishes a track for every bag it is following, but only
        fills in luggage_id once it has ENROLLED that bag - which per04 does
        when the bag crosses its X-conveyor entry->transfer line moving
        forward. Until then luggage_id is None and per07 must skip it:
        re-identification at Zone 2 is meaningless without a stable identity
        to re-identify against, and per04's raw_tracker_id is not one (it
        switches on occlusion and gets recycled).

        Skipping was correct but silent, which made a fully-starved service
        indistinguishable from an idle one - observed live, where per04 only
        ever saw bags in its tracking_y/exit zones, never at enrollment, so
        every one of its tracks carried luggage_id=None and per07 built zero
        baselines while looking perfectly healthy."""
        if now_ts - self._last_starvation_warn < config.STARVATION_WARN_INTERVAL_S:
            return
        self._last_starvation_warn = now_ts
        logger.warning(
            f"[Tracking] {self._tracks_without_id}/{self._tracks_seen} per04 tracks so far have "
            f"luggage_id=None - per04 is tracking luggage but has not enrolled it, so per07 can "
            f"build no Zone 1 baseline and every Cam2 frame will be discarded. Latest track: "
            f"identity_state={track.get('identity_state')!r}, active_zone={track.get('active_zone')!r}, "
            f"zones={track.get('zones')!r}, enrollment_type={track.get('enrollment_type')!r}. "
            f"If bags are entering per04's view already past its enrollment line, per07 stays "
            f"starved until per04's enrollment coverage is fixed."
        )

    def _remember_luggage_id(self, luggage_id: str, ts: float) -> None:
        self._seen_luggage_ids[luggage_id] = ts
        self._seen_luggage_ids.move_to_end(luggage_id)
        while len(self._seen_luggage_ids) > config.SEEN_LUGGAGE_ID_MEMORY:
            self._seen_luggage_ids.popitem(last=False)

    def _evict_stale_lidar(self, now_ts: float) -> None:
        """Nothing older than the sync tolerance can ever pair with a future
        frame, so holding it is a pure leak. Evicting relative to the stream
        clock (not wall clock) because sensor timestamps come from sns01's
        own host and need not agree with ours."""
        cutoff = now_ts - config.PENDING_RETENTION_S
        before = len(self._pending_lidar)
        self._pending_lidar = [(ts, pcd) for ts, pcd in self._pending_lidar if ts >= cutoff]
        dropped = before - len(self._pending_lidar)
        if dropped:
            logger.debug(f"[LiDAR] evicted {dropped} cloud(s) older than {config.PENDING_RETENTION_S}s")

    def _take_lidar_near(self, ts: float) -> Optional["o3d.geometry.PointCloud"]:
        best_idx, best_dt = None, None
        for i, (lidar_ts, _pcd) in enumerate(self._pending_lidar):
            dt = abs(lidar_ts - ts)
            if dt <= config.CAM1_LIDAR_SYNC_TOLERANCE_S and (best_dt is None or dt < best_dt):
                best_idx, best_dt = i, dt
        if best_idx is None:
            return None
        return self._pending_lidar.pop(best_idx)[1]

    def _frame_near(self, ts: float) -> Optional["np.ndarray"]:
        """Nearest buffered Cam1 frame to a per04 track timestamp, decoded
        back from JPEG. per04 publishes only after it has processed the
        frame, so the frame is always already in the buffer by the time the
        track message referring to it arrives - unless per04 lagged further
        behind than the buffer is deep, which is reported rather than
        silently substituting a wrong frame."""
        best, best_dt = None, None
        for frame_ts, jpeg in self._frame_buffer:
            dt = abs(frame_ts - ts)
            if dt <= config.TRACK_FRAME_MATCH_TOLERANCE_S and (best_dt is None or dt < best_dt):
                best, best_dt = jpeg, dt
        if best is None:
            return None
        return cv2.imdecode(np.frombuffer(best, dtype=np.uint8), cv2.IMREAD_COLOR)

    def _handle_cam1(self, fields: Dict[bytes, bytes]):
        """Cam1 frames are only buffered here - they no longer trigger
        anything on their own. per04_luggage_tracking decides what counts as
        a new piece of luggage (see _handle_tracking); per07 keeps recent
        frames around so that when per04 names one, the pixels to crop are
        still available."""
        cam1_ts = _parse_iso_timestamp(fields[b"timestamp"])
        frame = decode_camera_payload(fields[b"payload"], fields[b"shape"].decode("utf-8"))
        ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), config.CAM1_FRAME_BUFFER_JPEG_QUALITY])
        if not ok:
            logger.warning(f"[Cam1] frame @ {cam1_ts:.3f}s could not be encoded for buffering - dropped")
            return
        self._frame_buffer.append((cam1_ts, jpeg.tobytes()))
        self.last_input_ts["cam1"] = cam1_ts

        # Zone 1 popup view. Driven from here rather than from _handle_tracking
        # so the picture stays live at the Cam1 stream rate regardless of how
        # sparsely per04 publishes; `frame` is already decoded at this point,
        # so this costs no extra JPEG decode. Tracks older than
        # LIVE_VIEW_TRACK_STALE_S are dropped rather than drawn at a position
        # the bag has since left, and the renderer says why the boxes are gone.
        age = None if self._latest_tracks_ts is None else cam1_ts - self._latest_tracks_ts
        fresh = age is not None and age <= config.LIVE_VIEW_TRACK_STALE_S
        self._render_live_view(
            "cam1", cam1_ts,
            lambda: render_tracking_view(
                frame, self._latest_tracks if fresh else [],
                max_width=config.LIVE_VIEW_MAX_WIDTH_PX,
                jpeg_quality=config.LIVE_VIEW_JPEG_QUALITY,
                tracks_age_s=None if age is None else (0.0 if fresh else age),
            ),
        )

    def _handle_lidar(self, fields: Dict[bytes, bytes]):
        lidar_ts = _parse_iso_timestamp(fields[b"timestamp"])
        pcd = decode_lidar_payload(fields[b"payload"], fields[b"shape"].decode("utf-8"))
        self._pending_lidar.append((lidar_ts, pcd))
        self.last_input_ts["lidar"] = lidar_ts
        self._evict_stale_lidar(lidar_ts)

    def _handle_tracking(self, fields: Dict[bytes, bytes]):
        """per04_luggage_tracking's output is per07's Zone 1 trigger AND its
        Zone 1 detector. Each message carries the tracks per04 currently
        holds; the first time a permanent luggage_id (LUG_000001 style)
        appears, per07 builds a baseline for it using per04's own bbox
        rather than running a second YOLO pass over the same frame.

        A luggage_id persists across many consecutive per04 frames, so
        _seen_luggage_ids is what makes this fire once per physical item
        instead of once per frame."""
        try:
            payload = json.loads(fields[b"payload"].decode("utf-8"))
        except (KeyError, ValueError) as e:
            logger.error(f"[Tracking] unreadable per04 payload: {e}")
            return

        meta = payload.get("meta") or {}
        track_ts = meta.get("timestamp_s")
        if track_ts is None:
            iso = payload.get("timestamp")
            if not iso:
                logger.error("[Tracking] per04 message has neither meta.timestamp_s nor timestamp - skipped")
                return
            track_ts = datetime.fromisoformat(iso).timestamp()
        track_ts = float(track_ts)

        # Handed to the Zone 1 popup view (drawn in _handle_cam1) as the FULL
        # track list, before the enrolment filter below - so the view shows
        # what per04 actually sees rather than only the subset per07 can act
        # on. An all-amber Zone 1 view is the clearest possible statement of
        # "tracked, but never enrolled".
        tracks = payload.get("tracks") or []
        self._latest_tracks = tracks
        self._latest_tracks_ts = track_ts

        for track in tracks:
            self._tracks_seen += 1
            luggage_id = track.get("luggage_id")
            if not luggage_id:
                # Tracked by per04, but not yet identified by it - nothing to
                # re-identify against at Zone 2. Reported, not silently dropped.
                self._tracks_without_id += 1
                self._note_unidentified_track(track, track_ts)
                continue
            if luggage_id in self._seen_luggage_ids:
                continue

            bbox = track.get("bbox")
            if not bbox or len(bbox) != 4:
                logger.warning(f"[Tracking] {luggage_id} has no usable bbox - skipped")
                continue

            frame = self._frame_near(track_ts)
            if frame is None:
                logger.warning(
                    f"[Tracking] {luggage_id} @ {track_ts:.3f}s - no Cam1 frame within "
                    f"{config.TRACK_FRAME_MATCH_TOLERANCE_S}s in a {len(self._frame_buffer)}-frame buffer. "
                    f"per04 is lagging further behind than the buffer is deep; no baseline for this item."
                )
                self._remember_luggage_id(luggage_id, track_ts)
                continue

            pcd = self._take_lidar_near(track_ts)
            if pcd is None:
                logger.warning(
                    f"[Tracking] {luggage_id} @ {track_ts:.3f}s - no LiDAR cloud within "
                    f"{config.CAM1_LIDAR_SYNC_TOLERANCE_S}s ({len(self._pending_lidar)} buffered). "
                    f"Baselining without 3D dimensions."
                )

            self._remember_luggage_id(luggage_id, track_ts)
            self.service.process_baseline(
                luggage_id, track_ts, frame, pcd,
                bbox=[int(v) for v in bbox],
                class_label=track.get("class_name"),
            )

    def _handle_cam2(self, fields: Dict[bytes, bytes]):
        """Validates this Cam2 frame against every still-active baseline whose
        [MIN,MAX]_VALIDATION_TRANSIT_S window it falls inside, tracking how
        many cycles each baggage_id has actually been attempted. A baseline
        is retired - and, if it never matched, reported to the Error
        Management Service - the moment EITHER of the two independent give-
        up conditions is hit: config.MAX_CYCLE (3) real attempts exhausted
        without a match, or the transit window closing with no attempts left
        possible. Without this, baseline_store grows without bound over the
        life of the service, and a bag that never re-identifies would just
        silently vanish instead of raising an alert."""
        cam2_ts = _parse_iso_timestamp(fields[b"timestamp"])
        cam2_frame = decode_camera_payload(fields[b"payload"], fields[b"shape"].decode("utf-8"))
        self.last_input_ts["cam2"] = cam2_ts

        touched_any = False
        last_result = None
        to_retire: List[str] = []
        for baggage_id, baseline in list(self.service.baseline_store.items()):
            t_min, t_max = compute_validation_window(baseline["timestamp"])

            if cam2_ts > t_max:
                # Window closed. If nothing was ever evaluated for this item
                # it never reached Zone 2 at all - a different failure from
                # "we looked at it 3 times and could not recognise it".
                attempts = self._attempt_counts.get(baggage_id, 0)
                self._give_up(
                    baggage_id, to_retire,
                    status=config.STATUS_MAX_CYCLE_EXCEEDED if attempts else config.STATUS_NO_ARRIVAL,
                )
                continue

            if t_min <= cam2_ts <= t_max:
                touched_any = True
                cycle = self._attempt_counts.get(baggage_id, 0) + 1
                self._attempt_counts[baggage_id] = cycle
                result = self.service.validate_candidate(baggage_id, cam2_ts, cam2_frame, cycle_count=cycle)
                last_result = result
                self._publish_result(result)

                if result.status == config.STATUS_MATCHED:
                    to_retire.append(baggage_id)
                elif cycle >= config.MAX_CYCLE:
                    self._give_up(baggage_id, to_retire)

        for baggage_id in to_retire:
            del self.service.baseline_store[baggage_id]
            self._attempt_counts.pop(baggage_id, None)
            logger.info(f"[Cleanup] {baggage_id} retired from active baseline store")

        # Rendered whether or not a validation ran: an idle or starved Zone 2
        # is exactly as important to see as a verdict, and the renderer draws
        # that state explicitly. A frame that produced a verdict bypasses the
        # throttle, since a verdict is the one thing worth never dropping.
        active = len(self.service.baseline_store)
        if last_result is not None:
            self._last_view_render["cam2"] = 0.0
        self._render_live_view(
            "cam2", cam2_ts,
            lambda: render_validation_view(
                cam2_frame, last_result, active,
                max_width=config.LIVE_VIEW_MAX_WIDTH_PX,
                jpeg_quality=config.LIVE_VIEW_JPEG_QUALITY,
            ),
        )

        if not touched_any and not to_retire:
            if self.service.baseline_store:
                logger.info(f"[Cam2] frame @ {cam2_ts:.3f}s falls in no active baseline's arrival window - ignored")
            elif cam2_ts - self._last_no_baseline_warn >= config.STARVATION_WARN_INTERVAL_S:
                # Materially different from the line above, which means "there
                # ARE bags in flight, this frame just isn't in any of their
                # windows". Here there are none at all, so Zone 1 is producing
                # nothing and Cam1/Cam2 are not being used together - worth a
                # warning, but rate-limited or it fires on every frame.
                self._last_no_baseline_warn = cam2_ts
                logger.warning(
                    f"[Cam2] frame @ {cam2_ts:.3f}s discarded - the baseline store is EMPTY. Zone 2 has "
                    f"nothing to validate against, so Cam1 and Cam2 are not being used together. Zone 1 "
                    f"has registered no luggage; see the [Tracking] warnings for why."
                )

    def _give_up(self, baggage_id: str, to_retire: List[str], status: str = config.STATUS_MAX_CYCLE_EXCEEDED):
        """Marks a luggage_id as exhausted: publishes a final verdict, alerts
        the Error Management Service, and queues it for retirement from
        baseline_store. `status` distinguishes the two ways this happens -
        MAX_CYCLE_EXCEEDED (evaluated MAX_CYCLE times, never matched) from
        NO_ARRIVAL (window closed without it ever being seen at Zone 2)."""
        result = ValidationResult(
            baggage_id=baggage_id,
            status=status,
            cycle_count=self._attempt_counts.get(baggage_id, 0),
        )
        self._publish_result(result)
        self._publish_error_alert(result)
        to_retire.append(baggage_id)

    def _publish_error_alert(self, result):
        """Notifies the Error Management Service that a piece of luggage was
        never re-identified - either MAX_CYCLE_EXCEEDED or NO_ARRIVAL.

        KNOWN MISMATCH, deliberately left for now: the real EMS consumes
        `ems01_input`, whose observed messages are
        {type: "baggage_validation", payload: {baggage_id, sensor{...},
        tracking{...}, dimension{...}, barcode_data{...}}} - a much richer
        aggregate than what is sent here, on a different stream. Nothing is
        currently consuming STREAM_ERROR_ALERTS. Switching over needs the
        EMS owner to confirm what they actually want from per07."""
        payload = {
            "baggage_id": result.baggage_id,
            "reason": result.status,
            "cycles_attempted": str(result.cycle_count),
            "source_service": "per07_dynamic_validation_service",
        }
        self.r.xadd(config.STREAM_ERROR_ALERTS, payload, maxlen=config.REDIS_STREAM_MAXLEN)
        logger.warning(f"[ErrorAlert] {result.baggage_id} never re-identified after "
                        f"{result.cycle_count} cycle(s) - published to {config.STREAM_ERROR_ALERTS}")

    def _publish_result(self, result):
        payload = {
            "baggage_id": result.baggage_id,
            "status": result.status,
            "final_score": f"{result.final_score:.4f}",
            "sift_score": f"{result.sift_score:.4f}",
            "class_match": str(result.class_match),
            "color_score": f"{result.color_score:.4f}",
            "cycle_count": str(result.cycle_count),
            "elapsed_transit_s": f"{result.elapsed_transit_s or 0.0:.3f}",
            # Empty string, not a missing field, when unknown: stream values are strings.
            "dimensions_m": json.dumps([round(v, 4) for v in result.dimensions_m]) if result.dimensions_m else "",
            "cam2_ts": f"{result.cam2_timestamp:.3f}" if result.cam2_timestamp else "",
        }
        self.r.xadd(config.STREAM_VALIDATION_RESULTS, payload, maxlen=config.REDIS_STREAM_MAXLEN)
        logger.info(f"[Result] {result.baggage_id} -> {result.status} (score={result.final_score:.3f})")

    def _publish_status(self) -> None:
        """Publishes per07's in-memory state for the HMI (hmi01_configuration
        reads it): luggage currently between the zones, the newest sensor
        timestamp per input, and per07's own rules. Expires after STATUS_TTL_S,
        so a stopped per07 disappears from the HMI instead of looking frozen."""
        now = time.time()
        if now - self._last_status_publish < config.STATUS_PUBLISH_INTERVAL_S:
            return
        self._last_status_publish = now
        status = {
            "ts": now,
            "transit_window_s": [config.MIN_VALIDATION_TRANSIT_S, config.MAX_VALIDATION_TRANSIT_S],
            "max_cycle": config.MAX_CYCLE,
            "last_input_ts": self.last_input_ts,
            "in_flight": [
                {"baggage_id": bid, "class": b.get("class_label"),
                 "dims_m": list(b["dimensions_m"]) if b.get("dimensions_m") else None,
                 "t0": b.get("timestamp"), "cycles": self._attempt_counts.get(bid, 0)}
                for bid, b in self.service.baseline_store.items()
            ],
            "tracks_seen": self._tracks_seen,
            "tracks_without_id": self._tracks_without_id,
        }
        try:
            self.r.set(config.STATUS_KEY, json.dumps(status, default=str), ex=config.STATUS_TTL_S)
        except redis.exceptions.RedisError as e:
            logger.warning(f"[Status] {config.STATUS_KEY} not published: {e}")

    def run_once(self, block_ms: int = 1000, count: int = 10):
        streams = {name: ">" for name in self._streams}  # ">" = only not-yet-delivered-to-this-group
        response = self.r.xreadgroup(config.CONSUMER_GROUP, self.consumer_name, streams, count=count, block=block_ms)
        for stream_name_b, messages in response:
            stream_name = stream_name_b.decode("utf-8")
            for msg_id, fields in messages:
                try:
                    if stream_name == config.STREAM_CAMERA1:
                        self._handle_cam1(fields)
                    elif stream_name == config.STREAM_LIDAR:
                        self._handle_lidar(fields)
                    elif stream_name == config.STREAM_CAMERA2:
                        self._handle_cam2(fields)
                    elif stream_name == config.STREAM_TRACKING:
                        self._handle_tracking(fields)
                    self.r.xack(stream_name, config.CONSUMER_GROUP, msg_id)
                except Exception as e:
                    logger.error(f"Failed to process {stream_name} message {msg_id}: {e} "
                                 f"(left unacked - will be redelivered)")

    def run(self):
        logger.info(f"per07 DVS listening on {config.STREAM_CAMERA1}, {config.STREAM_LIDAR}, "
                    f"{config.STREAM_CAMERA2}, {config.STREAM_TRACKING} (per04 luggage_id trigger)")
        global run_flag
        while run_flag:
            try:
                self.run_once()
                # run_once blocks at most 1 s, so this also runs while idle.
                self._publish_status()
            except redis.exceptions.ConnectionError as e:
                logger.error(f"Redis connection lost: {e}. Retrying in 2s...")
                time.sleep(2)
        logger.info("Worker stopped gracefully.")


def main():
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    logger.info("Starting per07_dynamic_validation_service...")
    r = redis.Redis(host=config.REDIS_HOST, port=config.REDIS_PORT, decode_responses=False)
    while True:
        try:
            r.ping()
            logger.info("Connected to Redis")
            break
        except Exception:
            logger.warning("Waiting for Redis...")
            time.sleep(2)

    # The HMI lives in services/human_machine_interface (hmi01_configuration
    # builds its data, hmi02_dashboard serves it); per07 only publishes to Redis.
    worker = DynamicValidationWorker(redis_client=r)
    worker.run()


if __name__ == "__main__":
    main()
