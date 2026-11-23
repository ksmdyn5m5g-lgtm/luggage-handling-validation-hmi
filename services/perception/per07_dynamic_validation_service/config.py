"""
per07_dynamic_validation_service - Configuration
"""

import os
from pathlib import Path

# Matches sns01/sns03's own DEFAULT_DATA_DIR pattern: this file lives at
# services/perception/per07_dynamic_validation_service/config.py, 3 levels
# under the repo root, so 4 parents (strip filename, then climb 3 dirs)
# lands on <root>/data/. Overridden at runtime anyway by start.py's own
# env["DATA_DIR"] injection when launched via the orchestrator.
DEFAULT_DATA_DIR = Path(__file__).resolve().parent.parent.parent.parent / "data"
DATA_DIR = Path(os.getenv("DATA_DIR", str(DEFAULT_DATA_DIR)))

# Belt geometry (from racetrack diagram)
BELT_LENGTH_M = 24.60
BELT_WIDTH_M = 6.79

C1_OFFSET_FROM_LEFT_M = 1.30
C2_OFFSET_FROM_RIGHT_M = 2.40
CAM1_TO_CAM2_DISTANCE_M = BELT_LENGTH_M - C1_OFFSET_FROM_LEFT_M - C2_OFFSET_FROM_RIGHT_M  # ~20.9m
CAM2_HEIGHT_M = 3.0

# Conveyor belt speed & tolerance - kept for reference/diagnostics, but no
# longer what determines the validation window (see MIN/MAX_VALIDATION_TRANSIT_S
# below): the physics estimate from these numbers (~38-60s) didn't match the
# real, stated expected Zone1->Zone2 transit time for the same bag (60-90s).
BELT_SPEED_MPS = 0.45
BELT_SPEED_TOLERANCE_MPS = 0.10
BELT_SPEED_MIN_MPS = BELT_SPEED_MPS - BELT_SPEED_TOLERANCE_MPS
BELT_SPEED_MAX_MPS = BELT_SPEED_MPS + BELT_SPEED_TOLERANCE_MPS

# Timestamp sync tolerance
CAM1_LIDAR_SYNC_TOLERANCE_S = 0.8

# Nothing pairs with a frame more than CAM1_LIDAR_SYNC_TOLERANCE_S away, so
# anything older than this in the pending buffers is dead weight that can
# never match. Without eviction those buffers grow forever: at 5fps LiDAR,
# ~500KB per o3d cloud, that is ~2.4 MB/s (~8.6 GB/hour) and an ever-slower
# linear scan. Kept a little above the sync tolerance for arrival jitter.
PENDING_RETENTION_S = float(os.getenv("PENDING_RETENTION_S", 5.0))

# Cam1 frames are buffered so a per04 track message (which arrives AFTER
# per04 has processed the same frame) can still find the pixels to crop.
# Stored JPEG-encoded: a raw 4K frame is ~25MB, so a few seconds of raw
# frames would be gigabytes. Quality is deliberately high - per07's SIFT
# margin is already thin (a real match measured only 9-13 inliers), so
# compression artifacts are not somewhere to save space.
CAM1_FRAME_BUFFER_SIZE = int(os.getenv("CAM1_FRAME_BUFFER_SIZE", 25))
CAM1_FRAME_BUFFER_JPEG_QUALITY = int(os.getenv("CAM1_FRAME_BUFFER_JPEG_QUALITY", 95))
# How far apart a per04 track timestamp and a buffered Cam1 frame may be and
# still be considered the same frame.
TRACK_FRAME_MATCH_TOLERANCE_S = float(os.getenv("TRACK_FRAME_MATCH_TOLERANCE_S", 0.5))
# Bounds the "already baselined" set so a long-running service doesn't keep
# every LUG_ID it has ever seen.
SEEN_LUGGAGE_ID_MEMORY = int(os.getenv("SEEN_LUGGAGE_ID_MEMORY", 2000))

# per07 can only baseline a bag per04 has actually ENROLLED (given a permanent
# luggage_id). When per04 is tracking bags but never enrolling them, per07 is
# starved: no Zone 1 baseline exists, so every Zone 2 frame is discarded and
# Cam1/Cam2 never get used together. That used to be completely silent - the
# service looked healthy and simply did nothing. The warnings this interval
# governs make the starvation visible without spamming a line per frame.
STARVATION_WARN_INTERVAL_S = float(os.getenv("STARVATION_WARN_INTERVAL_S", 30.0))

# Core business rule: the same physical bag should be seen at Zone2 between
# 1min and 1min30s after Zone1, never sooner or later. This IS the
# validation window now (see compute_validation_window in logic.py) - not a
# belt-speed estimate. A Cam2 candidate outside [MIN, MAX] is not this bag,
# full stop, regardless of how well its SIFT/color/class scores.
MIN_VALIDATION_TRANSIT_S = 60.0  # 1 min
MAX_VALIDATION_TRANSIT_S = 90.0  # 1 min 30 sec

# --- Model paths ---
# start.py's orchestrator spawns every service via subprocess.Popen without
# setting cwd=, so each service actually runs with start.py's OWN directory
# as cwd, not its own folder - confirmed by a real FileNotFoundError in
# production ("model.pt" resolved against the repository root,
# where it doesn't exist). Bare relative strings are never safe here;
# resolve everything against this file's own directory instead, matching
# the Path(__file__)-based pattern every sibling service already uses.
_HERE = Path(__file__).resolve().parent
_SHARED_MODELS_DIR = _HERE.parent / "models"  # services/perception/models/ - same dir per01 uses

# Cam1's model.pt shares the EXACT class taxonomy per01's already-deployed
# luggage_multiclass_v2.pt logs at startup ({backpack, carton, handbag,
# handle, luggage, wheel}) - very likely the same checkpoint, so this
# defaults to that shared model rather than requiring a duplicate copy.
# INFERRED, not confirmed identical - override CAM1_MODEL_PATH if wrong.
CAM1_MODEL_PATH = os.getenv(
    "CAM1_MODEL_PATH",
    os.getenv("MODEL_PATH", str(_SHARED_MODELS_DIR / "luggage" / "luggage_multiclass_v2.pt")),
)
# per07's own Zone 2 (Cam2, camera 193) detector: YOLO26n trained on camera 192/193
# footage, classes Luggage / Softcase / Hardcase. It learned the bags at the cameras'
# native 1280x720 scale, so it runs at CAM2_IMGSZ = 1280; a smaller size shrinks the
# bags at the far end of the belt and misses them. Zone 1 keeps the shared model above.
CAM2_MODEL_PATH = os.getenv("CAM2_MODEL_PATH", str(_HERE / "luggage_yolo26n.pt"))
CAM2_IMGSZ = int(os.getenv("CAM2_IMGSZ", "1280"))
MODEL_CONF_THRESHOLD = 0.35

# SIFT & Feature matching thresholds
SIFT_RATIO_THRESHOLD = 0.75
# Absolute inlier floor/ceiling for verified_sift_score, calibrated against
# real captures: true non-matching pairs measured 4-6 geometrically-verified
# inliers, a true matching pair measured 12-13. 8 sits safely in that gap.
# (Previously this divided inliers by min(keypoint_count) instead of using
# an absolute floor - that ratio actively punished small/upscaled crops for
# simply having more total keypoints, even with equal or better real
# evidence; see SIFT_UPSCALE_MIN_SIDE_PX below for why crops can be small.)
SIFT_MIN_GOOD_MATCHES = 8
SIFT_INLIER_SCORE_TARGET = 20   # inlier count at which sift_score saturates to 1.0
RANSAC_REPROJ_PX = 5.0          # homography inlier threshold, pixels
MIN_MATCHES_FOR_HOMOGRAPHY = 4
MASK_ERODE_PX = 3               # shrinks segmentation mask so crop edges/background don't pollute SIFT/color
# Cam2 candidate boxes are frequently tiny (tens of px) since a distant bag
# occupies little of the frame. Below SIFT_MIN_NATIVE_CROP_SIDE_PX, SIFT is
# not attempted at all: verified on real captures that a genuinely different
# bag at 31-44px native size produced RANSAC-verified inlier counts equal to
# or higher than a true match at 142x120px, regardless of upscaling or how
# tight the RANSAC threshold was set (tried 5px down to 1px) - too little
# independent geometric constraint in that few real pixels to be trustworthy,
# no amount of tuning fixed it. Identity for tiny crops is decided by
# class+color alone instead (see extract_sift_descriptors). Above that floor,
# SIFT_UPSCALE_MIN_SIDE_PX still helps: a 142x150px crop's keypoint count
# roughly quadrupled after upscaling with no false-positive risk observed.
SIFT_MIN_NATIVE_CROP_SIDE_PX = 70
SIFT_UPSCALE_MIN_SIDE_PX = 150
NMS_IOU_THRESHOLD = 0.5         # collapses near-duplicate overlapping detections of the same object

# Fused confidence weights (SIFT + Class Match + HSV Color Correlation)
FUSION_WEIGHT_SIFT = 0.50
FUSION_WEIGHT_CLASS = 0.30
FUSION_WEIGHT_COLOR = 0.20
# Calibrated against real captures, not guessed: Cam1's overhead angle vs
# Cam2's oblique angle already costs real correspondence even for a true
# match - a confirmed true match (verified by its distinctive marking, a
# 142x120px crop with real geometric support) measured only 9 verified
# inliers, landing at fused=0.405 - below the previous 0.48. Three confirmed
# different bags (all too small for reliable SIFT, gated to sift_score=0)
# capped at 0.32-0.36 on class+color alone. 0.40 sits just above that true
# match, comfortably above the confirmed negatives, and - critically - the
# sift_score > 0.0 gate in evaluate_cycle_result means a class+color-only
# candidate can never cross this threshold regardless of how low it's set,
# so lowering it does not reopen the false-positive risk it looks like it
# would.
FUSION_PASS_THRESHOLD = 0.40

# Cycle and retry rules
MAX_CYCLE = 3
STATUS_MATCHED = "MATCHED"
STATUS_NOT_MATCHED = "NOT_MATCHED"
STATUS_ERROR_VALIDATE = "ERROR_VALIDATE"
STATUS_MAX_CYCLE_EXCEEDED = "MAX_CYCLE_EXCEEDED"
# Distinct from MAX_CYCLE_EXCEEDED: the luggage was never seen at Zone 2 at
# all before its window closed, versus being seen and failing to match on
# every attempt. Downstream (and an operator) needs to tell "it vanished"
# apart from "we could not recognise it" - they have different causes.
STATUS_NO_ARRIVAL = "NO_ARRIVAL"
STATUS_TRANSIT_TIME_EXCEEDED = "TRANSIT_TIME_EXCEEDED"

# Bounding box margin
DEFAULT_MARGIN_PX = 20

# Suitcase benchmark sizes (L, W, H in meters)
LUGGAGE_BENCHMARKS_M = [
    (0.55, 0.35, 0.24),  # 20"
    (0.68, 0.48, 0.27),  # 24"
    (0.75, 0.52, 0.31),  # 28"
]

# --- Pure-PCD dimensioning (calibration-free, multi-object) ---
# Unlike the old single-YOLO-box-driven crop, this measures every luggage-
# sized cluster on the belt in one pass. The belt plane is RANSAC-fit fresh
# from each frame rather than assumed at a fixed sensor-to-belt distance
# (that distance is rig-specific and wasn't safe to copy from another rig's
# calibration), so it self-calibrates instead of relying on a hardcoded
# constant that could silently be wrong for this mount.
PCD_VOXEL_SIZE_M = 0.01
# 0.02 was too tight for this belt's actual surface texture: RANSAC latched
# onto a small spurious flat patch instead of the true belt (verified against
# real captures - the belt plane only converges correctly to the dominant,
# histogram-confirmed surface at ~0.04-0.10m; 0.05 is the mid-point).
PCD_PLANE_DIST_THRESHOLD_M = 0.05
PCD_PLANE_RANSAC_ITERATIONS = 1000
PCD_HEIGHT_ABOVE_BELT_MIN_M = 0.08     # below this = belt surface/noise, discarded
PCD_HEIGHT_ABOVE_BELT_MAX_M = 0.40     # above this = implausibly tall, discarded
# 0.08 let a thin belt-rail point line bridge into the luggage cluster,
# merging them into one ~2m-long invalid blob (verified against real
# captures). 0.04 cleanly separates rail structure from actual luggage.
PCD_DBSCAN_EPS_M = 0.04
PCD_DBSCAN_MIN_POINTS = 15
PCD_MIN_LENGTH_M = 0.30
PCD_MAX_LENGTH_M = 0.85
PCD_MIN_WIDTH_M = 0.18
PCD_MAX_WIDTH_M = 0.65

# --- Redis stream keys ---
# Confirmed from sns01_sensor_streaming (the actual producer): it publishes
# three streams - live_lidar, live_camera, live_camera193 - each message
# shaped {"type", "timestamp" (ISO 8601 str), "shape" (numpy shape str),
# "payload" (raw bytes)}. decode_responses=False is required (payload is
# binary), matching sns01/per01/sns03 exactly.
#
# ASSUMED, not confirmed: live_camera = Zone1/Cam1 and live_camera193 =
# Zone2/Cam2. Based on per01_luggage_detection defaulting to watching
# live_camera for its Zone-1-only presence gate. Swap
# STREAM_CAMERA1/STREAM_CAMERA2 below if this is backwards.
REDIS_HOST = os.getenv("REDIS_HOST", "localhost")
REDIS_PORT = int(os.getenv("REDIS_PORT", 6379))
STREAM_CAMERA1 = os.getenv("STREAM_CAMERA1", "live_camera")      # Zone 1 - ASSUMED, see above
STREAM_CAMERA2 = os.getenv("STREAM_CAMERA2", "live_camera193")   # Zone 2 - ASSUMED, see above
STREAM_LIDAR = os.getenv("STREAM_LIDAR", "live_lidar")
# per04_luggage_tracking's output: one JSON "payload" field per message,
# shaped {"timestamp", "tracks":[...], "lifecycle_events":[...], "meta":{...}}.
# Each track carries the permanent business identity `luggage_id` (format
# LUG_000001, from identity_v2.py:185) plus pure-YOLO `bbox` in absolute
# [x1,y1,x2,y2] pixels and `class_name`. per07 uses this as its Zone 1
# trigger and detection source instead of running its own Cam1 YOLO pass.
# Observed live on the host; the stream name is per04's own default.
STREAM_TRACKING = os.getenv("STREAM_TRACKING", "output_redis")
REDIS_STREAM_MAXLEN = int(os.getenv("REDIS_STREAM_MAXLEN", 200))  # per007's own output streams below

# per007's own output - no downstream consumer contract was available, so
# these names/payloads are this service's own choice, not inferred from
# anything else in the repo.
STREAM_VALIDATION_RESULTS = os.getenv("STREAM_VALIDATION_RESULTS", "validation:results")
# ASSUMED contract for the separate Error Management Service - no integration
# spec was available, so this follows the existing stream naming/payload
# pattern. Confirm the real stream name/fields with that service and adjust
# main.py's _publish_error_alert() if it differs; it's a one-line change.
STREAM_ERROR_ALERTS = os.getenv("STREAM_ERROR_ALERTS", "error:alerts")

CONSUMER_GROUP = os.getenv("CONSUMER_GROUP", "per07_dvs_group")
CONSUMER_NAME = f"per07_worker_{os.getpid()}"

# --- Annotated live views (shown by hmi02_dashboard at /popup) ---
# Annotated Cam1 (tracking) and Cam2 (validation) frames, published to Redis
# as LIVE_VIEW_KEY_PREFIX + "cam1" / "cam2" (hash: jpeg, ts) and expiring after
# LIVE_VIEW_TTL_S, so a stopped per07 does not leave a frozen picture behind.
# Cam1 frames are 4K: annotating and JPEG-encoding them at the full 5fps stream
# rate would cost far more than a monitoring view is worth, and no operator
# needs 5fps to see what is happening - so renders are throttled and downscaled.
# A frame that produced a verdict bypasses the throttle (see _handle_cam2).
# Set LIVE_VIEW_MAX_FPS=0 to turn the popup rendering off entirely.
LIVE_VIEW_MAX_FPS = float(os.getenv("LIVE_VIEW_MAX_FPS", 2.0))
LIVE_VIEW_MAX_WIDTH_PX = int(os.getenv("LIVE_VIEW_MAX_WIDTH_PX", 960))
LIVE_VIEW_JPEG_QUALITY = int(os.getenv("LIVE_VIEW_JPEG_QUALITY", 75))
# How long per04's last track list stays drawable on the Zone 1 view. per04
# publishes far more sparsely than Cam1 streams (observed live at roughly one
# message per 30s against Cam1's 5fps), so boxes are dropped once they are
# older than this rather than left hovering where the bag no longer is.
LIVE_VIEW_TRACK_STALE_S = float(os.getenv("LIVE_VIEW_TRACK_STALE_S", 2.0))
LIVE_VIEW_KEY_PREFIX = os.getenv("LIVE_VIEW_KEY_PREFIX", "per07_live_view_")
LIVE_VIEW_TTL_S = int(os.getenv("LIVE_VIEW_TTL_S", 15))

# --- Status for the HMI (read by hmi01_configuration) ---
# per07's in-memory state - luggage between the zones, newest sensor
# timestamps - exists nowhere else, so it is published as one JSON string.
# It expires after STATUS_TTL_S, so a stopped per07 disappears from the HMI.
STATUS_KEY = os.getenv("STATUS_KEY", "per07_status")
STATUS_PUBLISH_INTERVAL_S = float(os.getenv("STATUS_PUBLISH_INTERVAL_S", 1.0))
STATUS_TTL_S = int(os.getenv("STATUS_TTL_S", 15))
