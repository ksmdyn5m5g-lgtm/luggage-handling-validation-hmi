import os

# --- GLOBAL CONFIGURATION ---
SERVICE_NAME = os.getenv("SERVICE_NAME", "Dashboard")
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")

# --- INFRASTRUCTURE (MESSAGE BROKER) ---
REDIS_HOST = os.getenv("REDIS_HOST", "localhost")
REDIS_PORT = int(os.getenv("REDIS_PORT", "6379"))

# --- SERVICE SPECIFICS ---
# Health-check interval, and how long the live event relay waits per read.
LOOP_DELAY_SEC = float(os.getenv("LOOP_DELAY_SEC", "1.0"))

# Web server. The server's firewall lets 8078 through from 192.0.2.0/24 (the
# address Tailscale users arrive from), so another port needs its own rule.
# Not 8077: VS Code's port forwarding holds it whenever the repo is open over SSH.
HTTP_HOST = os.getenv("HMI_HOST", "0.0.0.0")
HTTP_PORT = int(os.getenv("HMI_PORT", "8078"))
# The link people open, printed at startup.
PUBLIC_URL = f"http://{os.getenv('HMI_PUBLIC_HOST', '192.0.2.20')}:{HTTP_PORT}/"

# --- WHAT THE DASHBOARD READS (read-only) ---
SNAPSHOT_KEY = os.getenv("HMI_SNAPSHOT_KEY", "hmi01_snapshot")                    # hmi01_configuration
CONFIG_KEY = os.getenv("HMI_CONFIG_KEY", "hmi01_config")                          # hmi01_configuration
LIVE_VIEW_KEY_PREFIX = os.getenv("PER07_LIVE_VIEW_KEY_PREFIX", "per07_live_view_")  # per07, + "cam1" / "cam2"
STREAM_VALIDATION_RESULTS = os.getenv("STREAM_VALIDATION_RESULTS", "validation:results")  # per07
STREAM_ERROR_ALERTS = os.getenv("STREAM_ERROR_ALERTS", "error:alerts")            # per07
