import json
import time
import logging
import signal
import sys
import redis
from pathlib import Path

sys.path.append(str(Path(__file__).parent.absolute()))
from config import (SERVICE_NAME, LOG_LEVEL, LOOP_DELAY_SEC, REDIS_HOST, REDIS_PORT,
                    SNAPSHOT_KEY, CONFIG_KEY, KEY_TTL_SEC)
from logic import ServiceLogic

# --- LOGGING SETUP --
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL.upper()),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("Main")

# --- GRACEFUL SHUTDOWN ARCHITECTURE ---
run_flag = True


def signal_handler(sig, frame):
    """Intercepts Ctrl+C safely so the event loop shuts down organically."""
    global run_flag
    logger.info("Shutdown signal intercepted. Terminating next frame loop...")
    run_flag = False


# --- MAIN EXECUTION ---
def main():
    # Register OS signals
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    logger.info(f"🚀 Booting Microservice: {SERVICE_NAME}")

    r = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=False, socket_timeout=5)
    while run_flag:
        try:
            r.ping()
            logger.info("Connected to Redis")
            break
        except redis.RedisError:
            logger.warning("Waiting for Redis...")
            time.sleep(2)

    # 1. INITIALIZE LOGIC
    logic = ServiceLogic(r)
    logger.info(f"Publishing {SNAPSHOT_KEY} and {CONFIG_KEY} every {LOOP_DELAY_SEC}s (read-only towards all sources)")

    # 2. MAIN EVENT LOOP
    while run_flag:
        started = time.time()
        try:
            snapshot = logic.process()
            pipe = r.pipeline()
            pipe.set(SNAPSHOT_KEY, json.dumps(snapshot, default=str), ex=KEY_TTL_SEC)
            pipe.set(CONFIG_KEY, json.dumps(logic.configuration()), ex=KEY_TTL_SEC)
            pipe.execute()
        except redis.RedisError as e:
            logger.error(f"Redis error: {e}")
            time.sleep(1)
        except Exception as e:
            logger.error(f"Loop Exception: {e}")
            time.sleep(1)  # Prevent infinite spin fault

        # Rate limiter: a slow build does not add a full extra delay on top.
        time.sleep(max(0.0, LOOP_DELAY_SEC - (time.time() - started)))

    logger.info("Service stopped gracefully.")


if __name__ == "__main__":
    main()
