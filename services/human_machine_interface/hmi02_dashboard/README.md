# hmi02 — Dashboard

## Overview
Serves the **Luggage Handling System HMI**: a live web console for the whole AG4NBA luggage line. It shows what every service is producing, whether it is still producing, and which luggage needs attention.

This service only displays. The data is prepared by [`hmi01_configuration`](../hmi01_configuration/README.md), and everything comes from Redis.

```
 AG4NBA services ──► Redis ──► hmi01_configuration ──► hmi01_snapshot ──┐
 per07 ────────────► per07_live_view_cam1/2, validation:results ────────┤
                                                                        ▼
                                                    hmi02_dashboard (this service)
                                                                        │ HTTP :8078
                                                                        ▼
                                                                     browser
```

## Execution
Launched by the global `start.py` once `hmi02_dashboard` is in `SERVICE_REGISTRY` and `ACTIVE_SERVICES` (root `config.py`). Run `hmi01_configuration` too, or the pages have no data.

To run it on its own:
```bash
python main.py
```

### Opening the HMI
Open **http://192.0.2.20:8078/** on any computer connected to the team's Tailscale network or to the server's local network (192.0.2.0/24). Nothing else is needed: no VS Code, no SSH, no checkout of the branch. Any number of people can have it open at the same time.

The link works while `start.py` is running on the server. Start it with `python3 start.py` from the repository root only if it is not already running: a second `start.py` on the same server duplicates every camera frame, and its HMI cannot use port 8078.

The server's firewall lets port 8078 through from 192.0.2.0/24, which is also the address Tailscale users arrive from. Moving the HMI to another port needs a new firewall rule.

The page loads React and Babel from `unpkg.com`, so the **viewer's browser** needs internet access.

## Pages
Each address goes after the link, for example http://192.0.2.20:8078/popup.

| URL | Page | Use it for |
|---|---|---|
| `/` | **Luggage Handling System HMI** — Overview and Engineering views | Main console |
| `/popup` | Live zone view | Annotated Cam1 (tracking) and Cam2 (validation) images from per07 |
| `/validation` | Validation console | per07 results and alerts only |
| `/api/snapshot` | JSON | Everything the HMI shows (from `hmi01_snapshot`) |
| `/api/config` | JSON | The HMI configuration (from `hmi01_config`) |
| `/events` | Server-Sent Events | per07 results and alerts, pushed live |

**Overview** is for the operations floor: detections, identification rate, open issues, line status, where luggage is, throughput.
**Engineering** is for diagnosis: a card per pipeline stage with its real stream, newest entry and age; belt plan view; luggage ledger with details and validation checks; Error Management panel; stream topology.
In Engineering, the digital twin has a **Full screen** button: the belt plan fills the screen with the bag list beside it, and Esc returns. Each bag is labelled with its id; red markers are overdue.

## Inputs and Outputs
* **Inputs/Fetches** (read-only):
  * `hmi01_snapshot`, `hmi01_config` — from hmi01_configuration
  * `per07_live_view_cam1`, `per07_live_view_cam2` — annotated frames from per07
  * `validation:results`, `error:alerts` — per07 verdicts (plain `XREAD`, no consumer group)
* **Outputs/Publishes:** nothing to Redis. Serves HTTP on port 8078.

## Reading the page
Every stage shows the age of its newest entry:

| Colour | Meaning |
|---|---|
| green · live | newest entry ≤ 30 s old |
| amber | stopped less than 1 hour ago |
| red | stopped more than 1 hour ago |
| grey · absent | the stream does not exist yet |

"Stopped" means the producing service is not publishing. It does **not** mean the belt is empty.

| Number | How it is calculated |
|---|---|
| Bags detected | per01 `detected=True` entries in the last 60 s |
| Identified | share of recent per04 tracks that have a `luggage_id` |
| Needs attention | per07 error alerts + EMS manual-inspection items |
| Throughput chart | per01 detections per minute, counted by hmi01 since it started |
| Belt position (faded) | time since Zone 1 against per07's 60–90 s transit window |

## Settings (`config.py`)
| Setting | Default | Meaning |
|---|---|---|
| `HTTP_PORT` (env `HMI_PORT`) | 8078 | Web port, the one the server's firewall lets through |
| `HTTP_HOST` (env `HMI_HOST`) | 0.0.0.0 | Listen address |
| `PUBLIC_URL` (env `HMI_PUBLIC_HOST`) | `http://192.0.2.20:8078/` | The link printed in the log at startup |
| `SNAPSHOT_KEY`, `CONFIG_KEY` | `hmi01_snapshot`, `hmi01_config` | Where hmi01 publishes |
| `LIVE_VIEW_KEY_PREFIX` | `per07_live_view_` | Where per07 publishes annotated frames |

## Editing the page
`hmi_lhs.dc.html` is a **Claude Design** file: open it in Claude Design, or edit it as text and refresh the browser.
* Markup between `<x-dc>` and `</x-dc>` — layout and inline styles; `{{ name }}` is a value from the logic.
* `class Component extends DCLogic` — fetches `/api/snapshot` every 2 s and computes each `{{ name }}` in `renderVals()`.
* `data-props` on the script tag — adjustable constants (`startView`, `beltLength`, `beltWidth`, `expectedTransit`).
* `support.js` must stay next to it.

## Limitations
* **No acknowledging.** Alerts are shown, not cleared; clearing belongs to the Error Management System.
* **Test data is shown as-is**: `DUMMY…` ids from EMS, `BAG…` ids from `luggage_stream`.
* **EMS error codes E001–E003 are not documented** on this branch; ask the EMS owner.
* **Dimensions have no `luggage_id`**, so they cannot yet be joined to a tracked piece of luggage.
* **Built for large screens** (min. width 1180 px Overview, 1380 px Engineering).

## Troubleshooting
| Symptom | Likely cause |
|---|---|
| "waiting for hmi01_configuration" | hmi01_configuration is not running |
| Every stage "Stopped" | The producing services are not running — check `start.py` |
| `/popup` stays empty | per07 is not running, or `LIVE_VIEW_MAX_FPS=0` in per07 |
| The link does not load | The viewer is not on Tailscale or the local network, or `start.py` is not running on the server |
| `Cannot serve on 0.0.0.0:8078` in the log | Another program holds the port; stop it (a different `HMI_PORT` needs its own firewall rule) |
