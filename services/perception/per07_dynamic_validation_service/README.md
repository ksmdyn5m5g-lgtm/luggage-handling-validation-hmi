# per07 — Dynamic Validation Service

per07 checks that the luggage seen at **Zone 1** (Cam1 + LiDAR) is the same physical piece of luggage that arrives at **Zone 2** (Cam2) **60–90 seconds later**. If Zone 2 never confirms it within 3 attempts, per07 raises an error alert.

Its results are shown on the Luggage Handling System HMI — see [`hmi02_dashboard`](../../human_machine_interface/hmi02_dashboard/README.md).

Location: `services/perception/per07_dynamic_validation_service/`

---

## How it works

```
per04 output_redis ──(luggage_id + bbox)──┐
live_camera  (Cam1) ──(buffered frames)───┼──► ZONE 1 BASELINE ──► in memory, per luggage_id
live_lidar   (LiDAR)──(nearest cloud)─────┘      SIFT + colour + class + L×W×H

live_camera193 (Cam2) ──► ZONE 2 CHECK (only 60–90 s after Zone 1)
                              score every detected candidate
                              ├─ MATCHED ───────────────► validation:results
                              ├─ not matched, < 3 tries ─► wait for next Cam2 frame
                              └─ 3 tries / no arrival ───► validation:results + error:alerts
```

1. **Zone 1 — baseline.** per04 gives a piece of luggage a permanent `luggage_id` (e.g. `LUG_000001`). The first time per07 sees that id, it crops per04's box out of the matching Cam1 frame, pairs the nearest LiDAR cloud (±0.8 s), and stores: SIFT features, a colour histogram, the class, and L×W×H from the point cloud.
2. **Zone 2 — re-identification.** Every Cam2 frame that falls 60–90 s after a baseline is checked. per07 runs its own YOLO model (`luggage_yolo26n.pt`) and scores **every** candidate — never just the closest one:
   - SIFT matches confirmed by RANSAC geometry — **50%**
   - class agrees — **30%** (Zone 2's `Softcase` and `Hardcase` count as Zone 1's `luggage`)
   - colour histogram similarity — **20%**

   A candidate is `MATCHED` only if the fused score is ≥ **0.40** **and** SIFT found real geometric evidence (colour + class alone can never pass).
3. **Give up.** After **3** failed cycles → `MAX_CYCLE_EXCEEDED`. If no Cam2 frame ever arrived in the window → `NO_ARRIVAL`. Both go to `error:alerts`, and the baseline is removed.

## Inputs and outputs

| Direction | Redis key | From / to | Content |
|---|---|---|---|
| in | `output_redis` (stream) | per04_luggage_tracking | tracks with `luggage_id`, `bbox`, `class_name` — **the trigger** |
| in | `live_camera` (stream) | sns01 | Cam1 frames (Zone 1) |
| in | `live_lidar` (stream) | sns01 | LiDAR point clouds (Zone 1) |
| in | `live_camera193` (stream) | sns01 | Cam2 frames (Zone 2) |
| out | `validation:results` (stream) | anyone | `baggage_id, status, final_score, sift_score, class_match, color_score, cycle_count, elapsed_transit_s, dimensions_m, cam2_ts` |
| out | `error:alerts` (stream) | Error Management | `baggage_id, reason, cycles_attempted, source_service` |
| out | `per07_status` (JSON key, expires 15 s) | hmi01_configuration | luggage between the zones, newest sensor timestamps, transit window, cycle limit |
| out | `per07_live_view_cam1` / `_cam2` (hash, expires 15 s) | hmi02_dashboard `/popup` | annotated JPEG + time |

per07 reads its inputs with its own consumer group `per07_dvs_group`.

## Result statuses

| Status | Meaning |
|---|---|
| `MATCHED` | Same luggage confirmed at Zone 2 |
| `NOT_MATCHED` | This attempt failed; per07 will try again (up to 3) |
| `MAX_CYCLE_EXCEEDED` | 3 attempts, never matched → alert |
| `NO_ARRIVAL` | Nothing seen at Zone 2 inside 60–90 s → alert |
| `ERROR_VALIDATE` | No baseline or unreadable frame |

## Run it

- **With the system:** `per07_dynamic_validation_service` is in `ACTIVE_SERVICES` in the root `config.py`; run `python3 start.py` from the repo root.
- **On its own:** `cd services/perception/per07_dynamic_validation_service && python3 main.py`

To watch it, run `hmi01_configuration` and `hmi02_dashboard` and open the HMI (see the hmi02_dashboard README).

## Files

| File | Role |
|---|---|
| `main.py` | Entry point. Redis worker: buffers frames, handles triggers, publishes results, status and live views |
| `service.py` | `ValidationService` — baseline + Zone 2 scoring |
| `logic.py` | Algorithms: decoding, SIFT/RANSAC, colour, fusion, point-cloud dimensions |
| `config.py` | Every setting (all overridable by environment variable) |
| `visualization.py` | Draws the annotated live-view frames |
| `luggage_yolo26n.pt` | Zone 2 (Cam2, camera 193) YOLO26n model: `Luggage`, `Softcase`, `Hardcase`. Zone 1 uses the shared `models/luggage/luggage_multiclass_v2.pt` |

## Settings you are most likely to change (`config.py`)

| Setting | Default | Meaning |
|---|---|---|
| `MIN_VALIDATION_TRANSIT_S` / `MAX_VALIDATION_TRANSIT_S` | 60 / 90 | Zone 1 → Zone 2 window |
| `MAX_CYCLE` | 3 | Attempts before an alert |
| `FUSION_PASS_THRESHOLD` | 0.40 | Score needed to match |
| `CAM2_MODEL_PATH` / `CAM2_IMGSZ` | `luggage_yolo26n.pt` / 1280 | Zone 2 detector and the image size it runs at (it was trained at the camera's native 1280 × 720) |
| `SIFT_MIN_NATIVE_CROP_SIDE_PX` | 70 | Smaller crops skip SIFT (too few pixels to trust) |
| `STREAM_CAMERA1` / `STREAM_CAMERA2` | `live_camera` / `live_camera193` | Which camera is which zone |
| `LIVE_VIEW_MAX_FPS` | 2 | Live-view render rate; `0` turns it off |
| `STATUS_KEY` | `per07_status` | Where the HMI status is published |

## Things to know

- **per07 depends on per04 enrolment.** No `luggage_id` from per04 → no baseline → nothing to validate. per07 logs a `[Tracking] … luggage_id=None` warning every 30 s when this happens. per04 only enrols luggage that crosses its enrolment line on the X conveyor.
- **Zone 2 needs `live_camera193` streaming.** If it stops, no validation can happen.
- **Zone 2 knows three classes:** `Luggage`, `Softcase`, `Hardcase`. A bag that Zone 1 calls `backpack`, `handbag` or `carton` never agrees on class in Zone 2, so it loses the 30% class share. The model was trained on a small set of bags that pass the cameras again and again, so expect weaker detection on new luggage.
- **Nothing is saved to disk.** Baselines live in memory and are lost on restart.
- **Dimensions** come from the point cloud alone (no camera–LiDAR calibration). With several objects on the belt, the densest cluster is measured.
- **`start.py` does not set the working directory**, so every path in `config.py` is built from the file's own location. Keep it that way.
- **Error Management contract is provisional.** Alerts go to `error:alerts`; the EMS input stream `ems01_input` expects a richer message. Agree the format with the EMS owner before switching.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `[Cam2] … baseline store is EMPTY` | per04 is not enrolling luggage (see `[Tracking]` warnings) |
| No `[Cam2]` lines at all | `live_camera193` has stopped — check sns01 |
| HMI shows no luggage in flight while per07 runs | `per07_status` is not being published — check the per07 log for `[Status]` warnings |
| `FileNotFoundError` for a model | `luggage_yolo26n.pt` missing from this folder, or set `CAM2_MODEL_PATH` |
