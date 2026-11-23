import time
import logging
import signal
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.absolute()))
from config import SERVICE_NAME, LOG_LEVEL, LOOP_DELAY_SEC, HTTP_HOST, HTTP_PORT, PUBLIC_URL
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

    # 1. INITIALIZE LOGIC. The web server starts even while Redis is down;
    #    the pages then report what is missing instead of failing to load.
    logic = ServiceLogic()
    try:
        logic.start()
    except OSError as e:
        logger.error(f"Cannot serve on {HTTP_HOST}:{HTTP_PORT} ({e}). If the HMI is already running from "
                     f"another start.py, just open {PUBLIC_URL} - otherwise stop whatever holds the port.")
        sys.exit(1)

    # 2. MAIN EVENT LOOP - keeps the live event relay running.
    while run_flag:
        try:
            logic.process()
            time.sleep(LOOP_DELAY_SEC)
        except Exception as e:
            logger.error(f"Loop Exception: {e}")
            time.sleep(1)  # Prevent infinite spin fault

    logic.stop()
    logger.info("Service stopped gracefully.")


if __name__ == "__main__":
    main()
