import os

# --- GLOBAL CONFIGURATION ---
SERVICE_NAME = os.getenv("SERVICE_NAME", "Configuration")
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")

# --- INFRASTRUCTURE (MESSAGE BROKER) ---
REDIS_HOST = os.getenv("REDIS_HOST", "localhost")
REDIS_PORT = int(os.getenv("REDIS_PORT", "6379"))

# --- SERVICE SPECIFICS ---
# How often the HMI snapshot is rebuilt and republished.
LOOP_DELAY_SEC = float(os.getenv("LOOP_DELAY_SEC", "1.0"))

# Published for hmi02_dashboard. Both keys expire, so if this service stops
# the dashboard says so instead of showing a frozen picture.
SNAPSHOT_KEY = os.getenv("HMI_SNAPSHOT_KEY", "hmi01_snapshot")
CONFIG_KEY = os.getenv("HMI_CONFIG_KEY", "hmi01_config")
KEY_TTL_SEC = int(os.getenv("HMI_KEY_TTL_SEC", "15"))

# A stream whose newest entry is older than this is shown as stopped.
STALE_AFTER_SEC = float(os.getenv("HMI_STALE_AFTER_SEC", "30"))
# Redis keeps only ~20-400 entries per stream, so throughput history is
# counted here from service start: 1-minute buckets, 8 hours.
HISTORY_MINUTES = int(os.getenv("HMI_HISTORY_MINUTES", "480"))

# --- LINE ---
BELT_LENGTH_M = float(os.getenv("HMI_BELT_LENGTH_M", "24.60"))
BELT_WIDTH_M = float(os.getenv("HMI_BELT_WIDTH_M", "6.79"))
# per07 publishes its own transit window and cycle limit in PER07_STATUS_KEY;
# these apply only while per07 is not running.
DEFAULT_TRANSIT_WINDOW_SEC = (60.0, 90.0)
DEFAULT_MAX_CYCLE = 3
STATUS_MATCHED = "MATCHED"

# --- DATA SOURCES (read-only) ---
# role: (Redis stream, producing service, label), in pipeline order.
# Any stream name can be changed with its environment variable.
SOURCES = {
    "cam1": (os.getenv("HMI_STREAM_CAM1", "live_camera"), "sns01_sensor_streaming", "Cam1 frames (Zone 1)"),
    "lidar": (os.getenv("HMI_STREAM_LIDAR", "live_lidar"), "sns01_sensor_streaming", "LiDAR point clouds"),
    "cam2": (os.getenv("HMI_STREAM_CAM2", "live_camera193"), "sns01_sensor_streaming", "Cam2 frames (Zone 2)"),
    "detection": (os.getenv("HMI_STREAM_DETECTION", "detection_status"), "per01_luggage_detection", "Presence detection"),
    "dimension": (os.getenv("HMI_STREAM_DIMENSION", "luggage_dimension"), "per02_luggage_dimension", "Dimensions"),
    "characterization": (os.getenv("HMI_STREAM_CHARACTERIZATION", "luggage_characterization"), "per03_luggage_characterization", "Characterization"),
    "bbox3d": (os.getenv("HMI_STREAM_BBOX3D", "bbox_3d_output"), "lidar fusion (not on this branch)", "3D box / yaw"),
    "tracking": (os.getenv("HMI_STREAM_TRACKING", "output_redis"), "per04_luggage_tracking", "Tracking + identity"),
    "validation": (os.getenv("HMI_STREAM_VALIDATION", "validation:results"), "per07_dynamic_validation_service", "Zone 1 -> Zone 2 validation"),
    "alerts": (os.getenv("HMI_STREAM_ALERTS", "error:alerts"), "per07_dynamic_validation_service", "Validation error alerts"),
    "cart_plan": (os.getenv("HMI_STREAM_CART_PLAN", "cart_plan_stream"), "opt01_packing_optimizer", "Cart plan"),
    "luggage_items": (os.getenv("HMI_STREAM_LUGGAGE_ITEMS", "luggage_stream"), "opt02_dummy_streamer", "Luggage items (dummy)"),
    "ems_input": (os.getenv("HMI_STREAM_EMS_INPUT", "ems01_input"), "EMS input (dummy producer)", "EMS input"),
    "ems_inspection": (os.getenv("HMI_STREAM_EMS_INSPECTION", "ems02_inspection_events"), "ems02_manual_inspection", "Manual inspection queue"),
    "profile": (os.getenv("HMI_STREAM_PROFILE", "per06_digital_baggage_profile"), "per06 (not on this branch)", "Digital baggage profile"),
    "health_cam1": (os.getenv("HMI_STREAM_HEALTH_CAM1", "health_camera"), "sensor health", "Cam1 health"),
    "health_cam2": (os.getenv("HMI_STREAM_HEALTH_CAM2", "health_camera193"), "sensor health", "Cam2 health"),
    "health_lidar": (os.getenv("HMI_STREAM_HEALTH_LIDAR", "health_lidar"), "sensor health", "LiDAR health"),
}

# Plain keys (not streams)
PER07_STATUS_KEY = os.getenv("PER07_STATUS_KEY", "per07_status")
CART_INVENTORY_KEY = os.getenv("HMI_CART_INVENTORY_KEY", "cart_inventory")
CART_STATE_KEY = os.getenv("HMI_CART_STATE_KEY", "cart_state")
