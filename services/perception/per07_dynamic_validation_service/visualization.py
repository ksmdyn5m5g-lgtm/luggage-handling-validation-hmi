"""
PER_007 Dynamic Validation Service - HMI Visualization Module
"""

import cv2
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional, Dict, Any


def draw_sift_keypoint_matches(
    crop1: np.ndarray,
    crop2: np.ndarray,
    kp1: list,
    kp2: list,
    good_matches: list,
    status_text: str = "MATCHED",
    score: float = 0.0,
    max_draw: int = 35
) -> np.ndarray:
    """Draws side-by-side matching keypoints with color-coded connection lines between Cam1 and Cam2 crops."""
    if crop1 is None or crop2 is None:
        return np.zeros((300, 600, 3), dtype=np.uint8)

    # Resize crops to match height for clean side-by-side display
    h1, w1 = crop1.shape[:2]
    h2, w2 = crop2.shape[:2]
    target_h = max(h1, h2, 300)

    crop1_resized = cv2.resize(crop1, (int(w1 * target_h / h1), target_h))
    crop2_resized = cv2.resize(crop2, (int(w2 * target_h / h2), target_h))

    # Scale keypoints to resized dimensions
    scale1_x, scale1_y = (w1 * target_h / h1) / float(w1), target_h / float(h1)
    scale2_x, scale2_y = (w2 * target_h / h2) / float(w2), target_h / float(h2)

    scaled_kp1 = [cv2.KeyPoint(kp.pt[0] * scale1_x, kp.pt[1] * scale1_y, kp.size) for kp in kp1]
    scaled_kp2 = [cv2.KeyPoint(kp.pt[0] * scale2_x, kp.pt[1] * scale2_y, kp.size) for kp in kp2]

    match_color = (0, 255, 0) if status_text == "MATCHED" else (0, 165, 255)
    matched_img = cv2.drawMatches(
        crop1_resized, scaled_kp1,
        crop2_resized, scaled_kp2,
        good_matches[:max_draw], None,
        matchColor=match_color,
        singlePointColor=(255, 0, 0),
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )

    # Top HMI Header Banner
    h, w = matched_img.shape[:2]
    header = np.zeros((70, w, 3), dtype=np.uint8)
    header[:] = (25, 25, 35)

    status_color = (0, 255, 0) if status_text == "MATCHED" else (0, 0, 255)
    cv2.putText(header, f"ZONE 1 (CAM1)", (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    cv2.putText(header, f"ZONE 2 (CAM2)", (w - 220, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    cv2.putText(header, f"SIFT Re-ID: {len(good_matches)} Matches | Fused Score: {score:.2f} [{status_text}]", (w // 2 - 250, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)

    return np.vstack([header, matched_img])


def draw_dimension_hmi_overlay(
    cam1_frame: np.ndarray,
    bbox: List[int],
    dimensions_m: Optional[Tuple[float, float, float]],
    class_label: str = "luggage",
    confidence: float = 0.95
) -> np.ndarray:
    """Renders high-contrast bounding box and measurement banner over Camera 1 image."""
    img = cam1_frame.copy()
    h, w = img.shape[:2]

    x1, y1, x2, y2 = bbox
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 4)

    # Top HMI Banner
    banner_h = 90
    banner = np.zeros((banner_h, w, 3), dtype=np.uint8)
    banner[:] = (20, 20, 25)

    if dimensions_m:
        l_cm, w_cm, h_cm = dimensions_m[0] * 100.0, dimensions_m[1] * 100.0, dimensions_m[2] * 100.0
        vol_l = (l_cm * w_cm * h_cm) / 1000.0
        text1 = f"ITEM: {class_label.upper()} (Conf: {confidence*100:.1f}%)"
        text2 = f"3D DIMS: Length={l_cm:.1f}cm | Width={w_cm:.1f}cm | Height={h_cm:.1f}cm | Vol={vol_l:.1f}L"
    else:
        text1 = f"ITEM: {class_label.upper()}"
        text2 = "3D DIMS: Could not extract 3D dimensions from LiDAR"

    cv2.putText(banner, text1, (30, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(banner, text2, (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    return np.vstack([banner, img])


def render_hmi_dashboard_summary(
    baseline_info: Dict[str, Any],
    validation_res: Any,
    sift_match_img: np.ndarray,
    dimension_overlay_img: np.ndarray
) -> None:
    """Displays complete HMI visual dashboard using matplotlib."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    axes[0].imshow(cv2.cvtColor(dimension_overlay_img, cv2.COLOR_BGR2RGB))
    axes[0].set_title("1st & 4th: Zone 1 (C1 + LiDAR) Bounding Box & 3D Dimensions", fontsize=12, fontweight='bold', pad=10)
    axes[0].axis('off')

    axes[1].imshow(cv2.cvtColor(sift_match_img, cv2.COLOR_BGR2RGB))
    axes[1].set_title("2nd & 3rd: Zone 1 vs Zone 2 SIFT Keypoint Feature Matching (~1 min transit)", fontsize=12, fontweight='bold', pad=10)
    axes[1].axis('off')

    plt.tight_layout()
    plt.show()


# --- Live zone views -----------------------------------------------------
# Everything above renders for the notebook (matplotlib figures, offline
# analysis). The functions below render the annotated frames main.py publishes
# to Redis for hmi02_dashboard's zone view, so they return encoded JPEG bytes
# rather than arrays or figures, and never open a window: per07 runs headless
# on the remote host, where cv2.imshow would draw on a monitor nobody watches.

_COLOR_ENROLLED = (80, 220, 100)      # BGR green  - per04 gave it a luggage_id
_COLOR_UNENROLLED = (0, 170, 255)     # BGR amber  - tracked but not identified
_COLOR_MATCHED = (80, 220, 100)
_COLOR_NOT_MATCHED = (60, 60, 235)    # BGR red
_COLOR_IDLE = (150, 150, 150)
_COLOR_BANNER_BG = (32, 28, 24)


def _fit_width(frame: np.ndarray, max_width: int) -> Tuple[np.ndarray, float]:
    """Downscales for transport. 4K frames at several fps would otherwise cost
    far more in JPEG encoding and bandwidth than a monitoring view needs.
    Returns the resized frame and the scale factor applied, so boxes drawn in
    native coordinates can be scaled to match."""
    h, w = frame.shape[:2]
    if w <= max_width:
        return frame.copy(), 1.0
    scale = max_width / float(w)
    return cv2.resize(frame, (max_width, int(round(h * scale))), interpolation=cv2.INTER_AREA), scale


def _draw_banner(canvas: np.ndarray, lines: List[Tuple[str, Tuple[int, int, int]]]) -> np.ndarray:
    """Prepends a solid header strip carrying the view's state. Drawn as a
    separate strip rather than over the image so it can never sit on top of
    the luggage the viewer is trying to look at."""
    pad, line_h = 10, 22
    strip = np.full((pad * 2 + line_h * len(lines), canvas.shape[1], 3), _COLOR_BANNER_BG, dtype=np.uint8)
    for i, (text, color) in enumerate(lines):
        cv2.putText(strip, text, (pad, pad + line_h * i + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return np.vstack([strip, canvas])


def _label_box(canvas: np.ndarray, box, label: str, color, thickness: int = 2) -> None:
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)
    if not label:
        return
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    # Keep the caption on-canvas in both axes. Without the vertical flip a box
    # at the very top loses its label above the frame; without the horizontal
    # clamp a box near the right edge - routine at Zone 2, where bags enter and
    # leave frame - gets its luggage_id cut off mid-word.
    ty = y1 - 6 if y1 - th - 8 >= 0 else y2 + th + 8
    tx = max(0, min(x1, canvas.shape[1] - tw - 9))
    cv2.rectangle(canvas, (tx, ty - th - 5), (tx + tw + 8, ty + 3), color, -1)
    cv2.putText(canvas, label, (tx + 4, ty - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)


def render_tracking_view(frame: np.ndarray, tracks: List[Dict[str, Any]],
                         max_width: int = 960, jpeg_quality: int = 75,
                         tracks_age_s: Optional[float] = None) -> Optional[bytes]:
    """Zone 1 view: per04's tracks drawn over the Cam1 frame.

    Deliberately draws EVERY track, not just the ones per07 can baseline.
    A track with no luggage_id is exactly the case per07 has to skip, and
    showing it in amber alongside its identity_state is the whole point of
    the view - it makes "per04 is tracking bags but never enrolling them"
    visible at a glance instead of inferrable only from the logs."""
    if frame is None:
        return None
    canvas, scale = _fit_width(frame, max_width)

    enrolled = 0
    for track in tracks or []:
        box = track.get("bbox")
        if not box or len(box) != 4:
            continue
        luggage_id = track.get("luggage_id")
        if luggage_id:
            enrolled += 1
            color = _COLOR_ENROLLED
            label = f"{luggage_id} | {track.get('class_name', '?')}"
        else:
            color = _COLOR_UNENROLLED
            # raw_tracker_id is shown only to tell boxes apart on screen - it
            # is explicitly NOT what per07 keys baselines on (it switches on
            # occlusion and gets recycled).
            label = (f"UNENROLLED trk#{track.get('raw_tracker_id', '?')} | "
                     f"{track.get('identity_state', 'unknown')} | {track.get('active_zone', '-')}")
        _label_box(canvas, [v * scale for v in box], label, color)

    total = len(tracks or [])
    if total and not enrolled:
        headline = (f"ZONE 1 - TRACKING  |  {total} tracked, 0 enrolled  |  "
                    f"no luggage_id from per04 -> per07 cannot baseline")
        head_color = _COLOR_UNENROLLED
    elif total:
        headline = f"ZONE 1 - TRACKING  |  {total} tracked, {enrolled} enrolled"
        head_color = _COLOR_ENROLLED
    else:
        headline = "ZONE 1 - TRACKING  |  no tracks in frame"
        head_color = _COLOR_IDLE

    lines = [(headline, head_color)]
    # This view is driven by the Cam1 stream, not by per04, so the picture
    # stays live even when per04 goes quiet. When it does, say so explicitly
    # rather than showing an empty frame that reads as "nothing on the belt" -
    # "per04 has not spoken for 40s" is itself the diagnosis.
    if tracks_age_s is not None and tracks_age_s > 0:
        lines.append((f"per04 last published {tracks_age_s:.0f}s ago - boxes above may be stale",
                      _COLOR_UNENROLLED))
    elif tracks_age_s is None:
        lines.append(("per04 has published no tracks yet this session", _COLOR_IDLE))

    canvas = _draw_banner(canvas, lines)
    ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    return buf.tobytes() if ok else None


def render_validation_view(frame: np.ndarray, result: Optional[Any], active_baselines: int,
                           max_width: int = 960, jpeg_quality: int = 75) -> Optional[bytes]:
    """Zone 2 view: the Cam2 frame plus whatever verdict per07 just reached.

    result is None when no validation ran for this frame - either nothing is
    in flight, or this frame fell outside every open arrival window. That is
    a normal state and is shown as such, rather than being hidden: an empty
    baseline store is precisely the failure the operator needs to see."""
    if frame is None:
        return None
    canvas, scale = _fit_width(frame, max_width)

    if result is not None:
        matched = getattr(result, "status", "") == "MATCHED"
        color = _COLOR_MATCHED if matched else _COLOR_NOT_MATCHED
        box = getattr(result, "cam2_bbox", None)
        if box and len(box) == 4:
            _label_box(canvas, [v * scale for v in box],
                       f"{result.baggage_id} | {result.status} {result.final_score:.3f}", color, thickness=3)
        lines = [
            (f"ZONE 2 - VALIDATING  |  {result.baggage_id}  ->  {result.status}", color),
            (f"fused={result.final_score:.3f}  sift={result.sift_score:.3f}  "
             f"colour={result.color_score:.3f}  class={'Y' if result.class_match else 'N'}  "
             f"inliers={getattr(result, 'inlier_match_count', 0)}", color),
            (f"cycle {result.cycle_count}/3  |  transit +{result.elapsed_transit_s or 0.0:.1f}s  "
             f"(window 60-90s)  |  {active_baselines} baseline(s) in flight", _COLOR_IDLE),
        ]
    elif active_baselines:
        lines = [
            ("ZONE 2 - VALIDATING  |  idle", _COLOR_IDLE),
            (f"{active_baselines} baseline(s) in flight; this frame is outside every arrival window",
             _COLOR_IDLE),
        ]
    else:
        lines = [
            ("ZONE 2 - VALIDATING  |  STARVED", _COLOR_UNENROLLED),
            ("baseline store is EMPTY - Zone 1 has registered no luggage, so nothing can be validated",
             _COLOR_UNENROLLED),
        ]

    canvas = _draw_banner(canvas, lines)
    ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    return buf.tobytes() if ok else None
