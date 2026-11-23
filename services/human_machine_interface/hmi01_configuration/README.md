# hmi01 — Configuration

## Overview
Builds the data behind the **Luggage Handling System HMI**. Every second it reads the output of the AG4NBA services, checks how fresh each one is, and publishes a single JSON snapshot that [`hmi02_dashboard`](../hmi02_dashboard/README.md) displays.

It also holds the HMI's configuration: which stream feeds which panel, when a stream counts as stopped, and the line geometry.

It only **reads** other services' streams (`XREVRANGE` / `GET`) and never uses consumer groups, so it cannot slow down, block, or change any of them.

## Execution
Launched by the global `start.py` once `hmi01_configuration` is in `SERVICE_REGISTRY` and `ACTIVE_SERVICES` (root `config.py`).

To run it on its own:
```bash
python main.py
```

## Inputs and Outputs
* **Inputs/Fetches:** read-only, every `LOOP_DELAY_SEC`

  | Panel | Stream / key | Produced by |
  |---|---|---|
  | Camera / LiDAR freshness | `live_camera`, `live_lidar`, `live_camera193` (freshness only) | sns01 |
  | Detection | `detection_status` | per01 |
  | Dimension | `luggage_dimension` | per02 |
  | Characterization, 3D box | `luggage_characterization`, `bbox_3d_output` | per03, lidar fusion |
  | Tracking | `output_redis` | per04 |
  | Validation, alerts | `validation:results`, `error:alerts` | per07 |
  | Luggage between the zones | `per07_status` (key) | per07 |
  | Carts | `cart_plan_stream`, `cart_inventory`, `cart_state` | opt01 / est02 |
  | Error management | `ems01_input`, `ems02_inspection_events` | EMS |
  | Profiles, luggage items | `per06_digital_baggage_profile`, `luggage_stream` | per06, opt02 |
  | Sensor health | `health_camera`, `health_camera193`, `health_lidar` | sensor health |

* **Outputs/Publishes:**
  * `hmi01_snapshot` — everything the HMI shows, one JSON document (expires after 15 s)
  * `hmi01_config` — the HMI configuration: sources, thresholds, belt size, transit window (expires after 15 s)

  The keys expire so that, if this service stops, the dashboard reports it instead of showing old data.

## Settings (`config.py`)
| Setting | Default | Meaning |
|---|---|---|
| `LOOP_DELAY_SEC` | 1.0 | How often the snapshot is rebuilt |
| `STALE_AFTER_SEC` | 30 | Newest entry older than this → shown as stopped |
| `HISTORY_MINUTES` | 480 | Throughput history kept in memory |
| `BELT_LENGTH_M` / `BELT_WIDTH_M` | 24.60 / 6.79 | Belt size drawn on the HMI |
| `SOURCES` | see file | Every data source; each stream name has an environment variable |

The transit window (60–90 s) and cycle limit (3) come from per07 itself via `per07_status`; the values in `config.py` are used only while per07 is not running.

## Adding a data source
1. Add it to `SOURCES` in `config.py`.
2. If the panel needs parsed values, add a `_section()` method in `logic.py` and call it from `process()`.
3. Show it in `hmi02_dashboard/hmi_lhs.dc.html`.
4. Open http://192.0.2.20:8078/api/snapshot and check the value appears.

## Notes
* **Camera and LiDAR freshness comes from per07.** Their stream entries are raw frames of ~25 MB, so they are never read. If per07 is not running, freshness falls back to an estimate and is marked `approx`.
* **Throughput history restarts** whenever this service restarts.
* **Test data is passed through as-is**: `DUMMY…` ids from EMS, `BAG…` ids from `luggage_stream`.
