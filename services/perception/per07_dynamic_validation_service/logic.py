"""
per07_dynamic_validation_service - Core Processing Logic

Flat/absolute imports (not relative package imports) - matches every
sibling service (per01, sns01, sns03), which run as plain `python main.py`.
"""

import ast
import glob
import json
import os
import re
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any, List

import cv2
import numpy as np
import open3d as o3d
from ultralytics import YOLO

import config


@dataclass
class ValidationResult:
    baggage_id: str
    status: str
    sift_score: float = 0.0
    class_match: bool = False
    color_score: float = 0.0
    final_score: float = 0.0
    cycle_count: int = 0
    inlier_match_count: int = 0
    dimensions_m: Optional[Tuple[float, float, float]] = None
    cam2_timestamp: Optional[float] = None
    cam2_bbox: Optional[List[int]] = None
    elapsed_transit_s: Optional[float] = None
    timestamp: float = field(default_factory=time.time)


# 0.03 was too strict for cross-camera matching (Cam1's overhead angle vs
# Cam2's oblique angle already loses some correspondence just from
# perspective) - verified on a real true-match pair it left only 4 inliers,
# below even a permissive floor. 0.02 is more permissive (more candidate
# keypoints), matching what the notebook already used.
_sift = cv2.SIFT_create(contrastThreshold=0.02, edgeThreshold=12)
_bf_matcher = cv2.BFMatcher()
_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

_IMG_TIMESTAMP_RE = re.compile(r"img_(\d{2})(\d{2})(\d{2})_(\d{6})\.(?:jpg|jpeg|png)$", re.IGNORECASE)


def decode_camera_payload(payload: bytes, shape_str: str) -> np.ndarray:
    """Reconstructs a BGR frame from a live_camera/live_camera193 Redis
    stream message, matching sns01_sensor_streaming's encoding exactly
    (HxWxC uint8, flattened to raw bytes) and sns03_data_storage's own
    reconstruction of the same payload. Uses ast.literal_eval rather than
    the producer's eval() for the shape string - same result on
    well-formed input, no code-execution risk on a value that arrived over
    the network."""
    shape = ast.literal_eval(shape_str)
    return np.frombuffer(payload, dtype=np.uint8).reshape(shape)


def decode_lidar_payload(payload: bytes, shape_str: str) -> o3d.geometry.PointCloud:
    """Reconstructs a point cloud from a live_lidar Redis stream message
    (Nx3 float32 XYZ, flattened to raw bytes) - matches
    sns01_sensor_streaming's encoding and sns03_data_storage's
    reconstruction."""
    shape = ast.literal_eval(shape_str)
    xyz = np.frombuffer(payload, dtype=np.float32).reshape(shape)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd


def to_grayscale(crop: np.ndarray) -> np.ndarray:
    """Converts input BGR/BGRA/RGBA image crop to 8-bit single channel grayscale with CLAHE normalization."""
    if crop is None or crop.size == 0:
        return np.zeros((10, 10), dtype=np.uint8)

    if crop.ndim == 2:
        gray = crop
    elif crop.shape[2] == 4:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGRA2GRAY)
    else:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

    if gray.dtype != np.uint8:
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    return _clahe.apply(gray)


def upscale_crop_if_small(
    crop: np.ndarray,
    mask: Optional[np.ndarray] = None,
    min_side: int = config.SIFT_UPSCALE_MIN_SIDE_PX,
):
    """Upscales a crop (and its mask, if any) so its longer side reaches
    min_side, if it's currently smaller. A no-op for crops already at or
    above min_side. See SIFT_UPSCALE_MIN_SIDE_PX for why this matters: a
    distant Cam2 detection can be tens of pixels across, and SIFT can't find
    meaningful keypoints in that little native pixel data no matter how the
    detector is tuned."""
    if crop is None or crop.size == 0:
        return crop, mask
    h, w = crop.shape[:2]
    scale = min_side / max(h, w)
    if scale <= 1.0:
        return crop, mask
    new_size = (int(round(w * scale)), int(round(h * scale)))
    crop_up = cv2.resize(crop, new_size, interpolation=cv2.INTER_CUBIC)
    mask_up = cv2.resize(mask, new_size, interpolation=cv2.INTER_NEAREST) if mask is not None else None
    return crop_up, mask_up


def extract_sift_descriptors(frame_crop: np.ndarray, mask: Optional[np.ndarray] = None):
    """Returns (None, None) - no SIFT attempted at all - when the crop's
    NATIVE (pre-upscale) size is below SIFT_MIN_NATIVE_CROP_SIDE_PX.

    Verified on real captures that upscaling cannot fix this: a genuinely
    different bag at 31-44px native size produced RANSAC-verified inlier
    counts equal to or HIGHER than a true match at 142x120px, regardless of
    whether the tiny crop was upscaled or left at native resolution, and
    regardless of the RANSAC threshold tried (5px down to 1px). There just
    isn't enough independent geometric constraint in that few real pixels
    for a homography fit to reliably discriminate a coincidental point
    arrangement from a genuine one - tightening thresholds or upscaling
    both failed to separate them. Below this floor, identity is decided by
    class+color alone (see fuse_confidence) rather than risk a confident
    false match."""
    if frame_crop is None or frame_crop.size == 0:
        return None, None
    h, w = frame_crop.shape[:2]
    if min(h, w) < config.SIFT_MIN_NATIVE_CROP_SIDE_PX:
        return None, None
    frame_crop, mask = upscale_crop_if_small(frame_crop, mask)
    gray = to_grayscale(frame_crop)
    keypoints, descriptors = _sift.detectAndCompute(gray, mask)
    return keypoints, descriptors


def get_seg_masks(results, frame_shape: Tuple[int, int, int]) -> List[Optional[np.ndarray]]:
    """Binary masks (uint8, 0/255) aligned by index with results[0].boxes.
    Falls back to a list of Nones when the model has no segmentation head."""
    boxes = results[0].boxes
    masks = results[0].masks
    if masks is None:
        return [None] * len(boxes)
    h, w = frame_shape[:2]
    out = []
    for m in masks.data.cpu().numpy():
        m_resized = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
        out.append(((m_resized > 0.5).astype(np.uint8)) * 255)
    return out


def masked_crop(img: np.ndarray, mask: Optional[np.ndarray], bbox: List[int], erode_px: int = config.MASK_ERODE_PX):
    """Crops img to bbox and, if a segmentation mask is available, crops+erodes
    it too so background/belt pixels inside the box don't pollute SIFT/color."""
    x1, y1, x2, y2 = bbox
    crop = img[y1:y2, x1:x2]
    if mask is None:
        return crop, None
    mask_crop = mask[y1:y2, x1:x2]
    if erode_px > 0:
        kernel = np.ones((erode_px, erode_px), np.uint8)
        mask_crop = cv2.erode(mask_crop, kernel, iterations=1)
    return crop, mask_crop


def extract_color_histogram(frame_crop: np.ndarray) -> Optional[np.ndarray]:
    if frame_crop is None or frame_crop.size == 0:
        return None
    bgr = cv2.cvtColor(frame_crop, cv2.COLOR_BGRA2BGR) if (frame_crop.ndim == 3 and frame_crop.shape[2] == 4) else frame_crop
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [18, 25], [0, 180, 0, 256])
    cv2.normalize(hist, hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
    return hist


def match_sift_raw(baseline_descriptors: Optional[np.ndarray], frame_descriptors: Optional[np.ndarray]) -> list:
    """Ratio-test matches, before geometric verification."""
    if baseline_descriptors is None or frame_descriptors is None:
        return []
    if len(baseline_descriptors) < 2 or len(frame_descriptors) < 2:
        return []
    matches = _bf_matcher.knnMatch(baseline_descriptors, frame_descriptors, k=2)
    return [m[0] for m in matches if len(m) == 2 and m[0].distance < config.SIFT_RATIO_THRESHOLD * m[1].distance]


def geometric_verify_matches(kp1: list, kp2: list, good_matches: list, ransac_thresh: float = config.RANSAC_REPROJ_PX) -> Tuple[list, float]:
    """Filters ratio-test matches down to the subset consistent with one real
    homography (RANSAC), discarding coincidental texture matches that would
    otherwise inflate the raw match count without meaning anything."""
    if not kp1 or not kp2 or len(good_matches) < config.MIN_MATCHES_FOR_HOMOGRAPHY:
        return [], 0.0
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, ransac_thresh)
    if H is None or mask is None:
        return [], 0.0
    inlier_mask = mask.ravel().astype(bool)
    inlier_matches = [m for m, keep in zip(good_matches, inlier_mask) if keep]
    inlier_ratio = len(inlier_matches) / len(good_matches)
    return inlier_matches, inlier_ratio


def verified_sift_score(inlier_matches: list, baseline_descriptors: Optional[np.ndarray], frame_descriptors: Optional[np.ndarray]) -> float:
    """Absolute-evidence score: 0 below SIFT_MIN_GOOD_MATCHES inliers, then
    ramps linearly to 1.0 at SIFT_INLIER_SCORE_TARGET inliers.

    This used to be inlier_count / min(keypoint_count) - a ratio that
    actively punishes a crop for having MORE total keypoints, regardless of
    whether those extra keypoints came from genuine texture or from
    upscaling a small crop (see upscale_crop_if_small). Measured on real
    captures: upscaling a genuine match's tiny Cam2 crop took it from 27 to
    ~115 keypoints while inliers stayed ~12-13 - under the old ratio that
    LOWERED the score (12/27=0.44 -> 13/130=0.10) despite having more real
    matching evidence (21->35 good matches), because the denominator grew
    faster than genuine matches did. An absolute inlier count doesn't have
    that problem. Floor/target were calibrated against real measurements:
    true non-matches landed at 4-6 verified inliers, a true match at 12-13."""
    if baseline_descriptors is None or frame_descriptors is None:
        return 0.0
    if len(baseline_descriptors) < 2 or len(frame_descriptors) < 2:
        return 0.0
    n = len(inlier_matches)
    if n < config.SIFT_MIN_GOOD_MATCHES:
        return 0.0
    span = config.SIFT_INLIER_SCORE_TARGET - config.SIFT_MIN_GOOD_MATCHES
    return min((n - config.SIFT_MIN_GOOD_MATCHES) / span, 1.0) if span > 0 else 1.0


# Suitcase subtypes that Zone 1's model simply calls "luggage".
_LUGGAGE_SUBTYPES = {"softcase", "hardcase", "suitcase", "soft_luggage", "hard_luggage"}


def normalize_class_label(name: Optional[str]) -> str:
    """Class name for comparing Zone 1 with Zone 2. The zones run different models
    with their own vocabularies: Zone 1 (luggage_multiclass_v2) says 'luggage',
    Zone 2 (luggage_yolo26n) says 'Luggage', 'Softcase' or 'Hardcase'. Case and
    whitespace are ignored and the suitcase subtypes count as luggage, so a
    hard-shell case that Zone 1 calls 'luggage' still agrees in Zone 2."""
    label = name.strip().lower() if name else ""
    return "luggage" if label in _LUGGAGE_SUBTYPES else label


def match_color_histograms(hist1: Optional[np.ndarray], hist2: Optional[np.ndarray]) -> float:
    if hist1 is None or hist2 is None:
        return 0.0
    score = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
    return float(max(0.0, score))


def fuse_confidence(sift_score: float, class_match: bool, color_score: float) -> float:
    class_score = 1.0 if class_match else 0.0
    return (
        config.FUSION_WEIGHT_SIFT * sift_score +
        config.FUSION_WEIGHT_CLASS * class_score +
        config.FUSION_WEIGHT_COLOR * color_score
    )


def compute_confidence_score(num_points: int, l_m: float, w_m: float, h_m: float) -> float:
    dims = sorted([l_m, w_m, h_m], reverse=True)
    best_err = float('inf')
    for b_l, b_w, b_h in config.LUGGAGE_BENCHMARKS_M:
        err = (abs(dims[0] - b_l)/b_l + abs(dims[1] - b_w)/b_w + abs(dims[2] - b_h)/b_h) / 3.0
        if err < best_err:
            best_err = err
    size_score = max(0.0, 1.0 - best_err)
    density_score = min(1.0, num_points / 200.0)
    return 0.7 * size_score + 0.3 * density_score


def compute_pca_dimensions_3d(cluster_pcd: o3d.geometry.PointCloud):
    pts = np.asarray(cluster_pcd.points)
    if len(pts) < 4:
        return None
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    cov = np.cov(centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = eigvals.argsort()[::-1]
    eigvecs = eigvecs[:, order]

    aligned = centered @ eigvecs
    lower = np.percentile(aligned, 1.0, axis=0)
    upper = np.percentile(aligned, 99.0, axis=0)
    local_extent = upper - lower
    dims_sorted = tuple(sorted([round(float(x), 4) for x in local_extent], reverse=True))
    return dims_sorted


def _iou(box_a: List[int], box_b: List[int]) -> float:
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def nms_filter_idxs(boxes, idxs: List[int], iou_threshold: float = config.NMS_IOU_THRESHOLD) -> List[int]:
    """Greedy NMS over a candidate index list: collapses near-duplicate
    overlapping boxes for the same physical object (a model can legitimately
    fire more than one box for one item - seen on real captures: two boxes
    at nearly the same location, one at 0.94 confidence and one at 0.51)
    down to the single highest-confidence box per cluster. Runs before
    candidates ever reach a human picker or a matching loop, so the same
    item doesn't show up multiple times."""
    ordered = sorted(idxs, key=lambda i: float(boxes[i].conf), reverse=True)
    kept: List[int] = []
    kept_boxes: List[List[int]] = []
    for i in ordered:
        bi = [int(v) for v in boxes[i].xyxy.tolist()[0]]
        if all(_iou(bi, kb) < iou_threshold for kb in kept_boxes):
            kept.append(i)
            kept_boxes.append(bi)
    return kept


def select_best_luggage_idx(boxes, image_shape: Tuple[int, int, int], model_names: list) -> Optional[int]:
    """Picks the single closest (largest bbox area, as a proxy for camera
    proximity) + highest-confidence luggage box, ignoring wheel/handle
    sub-detections, belt-edge false positives, and duplicate overlapping
    detections of the same item. Returns its index into `boxes`, or None if
    boxes is empty."""
    if boxes is None or len(boxes) == 0:
        return None
    h, w = image_shape[:2]
    idxs = [i for i, b in enumerate(boxes) if model_names[int(b.cls)].lower() not in ('wheel', 'handle')]
    if not idxs:
        idxs = list(range(len(boxes)))
    idxs = nms_filter_idxs(boxes, idxs)

    valid = []
    for i in idxs:
        b = boxes[i]
        x1, y1, x2, y2 = [int(v) for v in b.xyxy.tolist()[0]]
        box_w, box_h = x2 - x1, y2 - y1
        is_edge = (y2 >= h - 10 and box_h < 200) or (x1 <= 10 and box_w < 200)
        if not is_edge:
            valid.append((i, box_w * box_h))

    if not valid:
        valid = [(i, (int(boxes[i].xyxy[0][2]) - int(boxes[i].xyxy[0][0])) * (int(boxes[i].xyxy[0][3]) - int(boxes[i].xyxy[0][1]))) for i in idxs]

    best_i, _ = max(valid, key=lambda x: float(boxes[x[0]].conf) * np.sqrt(x[1]))
    return best_i


def extract_luggage_clusters_pcd(
    pcd: Optional[o3d.geometry.PointCloud],
) -> List[Dict[str, Any]]:
    """Calibration-free, multi-object dimensioning straight from the LiDAR
    cloud. RANSAC-fits the dominant flat surface in view (the belt) fresh
    from THIS frame rather than assuming a fixed sensor-to-belt distance -
    that distance is rig-specific, and a value tuned for a different rig's
    mount isn't safe to reuse here. Points are kept by signed distance from
    that fitted plane (not raw axis coordinates), so it stays correct even
    if the belt isn't perfectly perpendicular to the sensor. DBSCAN then
    separates whatever's left into individual items, so every bag on the
    belt gets measured in one pass instead of only whichever one a YOLO box
    happened to land on. Returns one dict per size-plausible cluster:
    dimensions_m, point_count, centroid_m - empty list if none qualify."""
    if pcd is None:
        return []  # no LiDAR for this item - caller falls back to no dimensions
    work = pcd.voxel_down_sample(voxel_size=config.PCD_VOXEL_SIZE_M) if config.PCD_VOXEL_SIZE_M > 0 else pcd

    pts = np.asarray(work.points)
    if len(pts) < config.PCD_DBSCAN_MIN_POINTS:
        return []

    plane_coeffs, _inliers = work.segment_plane(
        distance_threshold=config.PCD_PLANE_DIST_THRESHOLD_M,
        ransac_n=3,
        num_iterations=config.PCD_PLANE_RANSAC_ITERATIONS,
    )
    a, b, c, d = plane_coeffs
    plane_norm = np.sqrt(a * a + b * b + c * c)
    height_above_plane = np.abs(pts @ np.array([a, b, c]) + d) / plane_norm

    keep = (height_above_plane >= config.PCD_HEIGHT_ABOVE_BELT_MIN_M) & \
           (height_above_plane <= config.PCD_HEIGHT_ABOVE_BELT_MAX_M)
    above_pts = pts[keep]
    if len(above_pts) < config.PCD_DBSCAN_MIN_POINTS:
        return []

    above_pcd = o3d.geometry.PointCloud()
    above_pcd.points = o3d.utility.Vector3dVector(above_pts)
    labels = np.array(above_pcd.cluster_dbscan(eps=config.PCD_DBSCAN_EPS_M, min_points=config.PCD_DBSCAN_MIN_POINTS))
    if len(labels) == 0 or labels.max() < 0:
        return []

    results = []
    for cluster_id in range(labels.max() + 1):
        idx = np.where(labels == cluster_id)[0]
        if len(idx) < 4:
            continue
        cluster_pts = above_pts[idx]
        c_pcd = o3d.geometry.PointCloud()
        c_pcd.points = o3d.utility.Vector3dVector(cluster_pts)
        dims = compute_pca_dimensions_3d(c_pcd)
        if dims is None:
            continue
        l_m, w_m, h_m = dims
        if not (config.PCD_MIN_LENGTH_M <= l_m <= config.PCD_MAX_LENGTH_M):
            continue
        if not (config.PCD_MIN_WIDTH_M <= w_m <= config.PCD_MAX_WIDTH_M):
            continue
        results.append({
            "dimensions_m": dims,
            "point_count": len(cluster_pts),
            "centroid_m": tuple(cluster_pts.mean(axis=0).tolist()),
            "confidence": round(compute_confidence_score(len(cluster_pts), l_m, w_m, h_m), 4),
        })

    return results


def select_cluster_pcd(clusters: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Picks which measured cluster to report when the belt holds more than
    one. Takes the most point-dense, as the more likely genuine item over
    noise or a partial return.

    This used to project each cluster's centroid into image space with the
    LiDAR-camera calibration and pick whichever landed nearest the 2D box -
    strictly better disambiguation, but it needed a calibration file that is
    no longer part of this service."""
    if not clusters:
        return None
    if len(clusters) == 1:
        return clusters[0]
    return max(clusters, key=lambda c: c["point_count"])


def count_plausible_luggage_boxes(boxes, image_shape: Tuple[int, int, int], model_names: list) -> int:
    """Counts non-wheel/handle, non-edge-clipped candidate boxes (the same
    filtering select_best_luggage_idx applies before picking a winner).
    Zone 1 has no baseline yet to score candidates against, so when this is
    >1 the winning pick is a positional/size guess with no way to verify it
    picked the physically correct bag - callers should treat that baseline
    as unverified (log/flag it) rather than trust it silently."""
    if boxes is None or len(boxes) == 0:
        return 0
    h, w = image_shape[:2]
    idxs = [i for i, b in enumerate(boxes) if model_names[int(b.cls)].lower() not in ('wheel', 'handle')]
    if not idxs:
        idxs = list(range(len(boxes)))
    idxs = nms_filter_idxs(boxes, idxs)
    count = 0
    for i in idxs:
        b = boxes[i]
        x1, y1, x2, y2 = [int(v) for v in b.xyxy.tolist()[0]]
        box_w, box_h = x2 - x1, y2 - y1
        is_edge = (y2 >= h - 10 and box_h < 200) or (x1 <= 10 and box_w < 200)
        if not is_edge:
            count += 1
    return count if count > 0 else len(idxs)


def extract_dimensions_fused(
    cam1_frame: np.ndarray,
    pcd: o3d.geometry.PointCloud,
    model: Optional[YOLO] = None,
    bbox: Optional[List[int]] = None,
    class_label: Optional[str] = None,
) -> Optional[Dict[str, Any]]:
    """Builds a Zone1 baseline: a 2D re-id signature (box, SIFT
    keypoints/descriptors, color histogram, class label) from the camera
    image, fused with 3D dimensions from the pure-PCD multi-cluster method
    above.

    Two ways to get the 2D box:
      - pass `bbox` (absolute [x1,y1,x2,y2] px) and `class_label` - the
        normal live path, where per04_luggage_tracking has already detected
        and identified this luggage and per07 has no reason to repeat the
        detection. `model` is then unused and may be None.
      - pass `model` and let it run its own YOLO pass - used offline
        (notebook/CLI testing) where there is no per04 output to lean on.

    NOTE: the pure-PCD dimensioning below is provisional. per02_luggage_dimension
    owns LiDAR dimensioning and is expected back; when it returns, per07
    should consume its `luggage_dimension` stream rather than measuring
    again here."""
    if cam1_frame is None:
        return None

    candidate_count = 1
    mask = None

    if bbox is not None:
        x1, y1, x2, y2 = [int(v) for v in bbox]
        if class_label is None:
            class_label = "luggage"
    else:
        if model is None:
            return None
        results = model(cam1_frame, conf=config.MODEL_CONF_THRESHOLD, verbose=False)
        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            return None

        boxes = results[0].boxes
        best_idx = select_best_luggage_idx(boxes, cam1_frame.shape, model.names)
        if best_idx is None:
            return None
        candidate_count = count_plausible_luggage_boxes(boxes, cam1_frame.shape, model.names)
        best_box = boxes[best_idx]
        masks = get_seg_masks(results, cam1_frame.shape)
        mask = masks[best_idx]

        class_id = int(best_box.cls)
        class_label = model.names[class_id]
        x1, y1, x2, y2 = [int(v) for v in best_box.xyxy.tolist()[0]]

    clusters = extract_luggage_clusters_pcd(pcd)
    chosen = select_cluster_pcd(clusters)
    best_dims = chosen["dimensions_m"] if chosen else None
    # No detector confidence to fall back on when the box came from per04
    # (its own confidence lives on the track, not here) - 0.0 reads as
    # "no dimensioning confidence", which is what an unmeasured cluster is.
    best_score = chosen["confidence"] if chosen else 0.0

    crop_img, mask_crop = masked_crop(cam1_frame, mask, [x1, y1, x2, y2])
    keypoints, descriptors = extract_sift_descriptors(crop_img, mask_crop)
    color_hist = extract_color_histogram(crop_img)

    return {
        "class_label": class_label,
        "bbox": [x1, y1, x2, y2],
        "dimensions_m": best_dims,
        "sift_keypoints": keypoints,
        "sift_descriptors": descriptors,
        "color_hist": color_hist,
        "confidence": round(float(best_score), 4),
        "candidate_count": candidate_count,
        "ambiguous": candidate_count > 1,
    }


def detect_cam2_candidates(model: YOLO, frame: np.ndarray) -> List[Tuple[str, List[int], float, np.ndarray, Optional[np.ndarray]]]:
    """Runs the Cam2 model and returns EVERY luggage detection in the frame
    (class_label, bbox, conf, crop, mask_crop), not just the closest/highest-
    confidence one. When multiple bags share a Cam2 frame, picking a single
    candidate by proximity/confidence alone risks grabbing a different
    physical bag than the one Cam1 is tracking - matching identity has to be
    decided by SIFT/color/class similarity to the baseline, not by which box
    happens to be biggest. See select_best_cam2_match, which scores all of
    these against the baseline and keeps whichever one actually matches."""
    results = model(frame, conf=config.MODEL_CONF_THRESHOLD, imgsz=config.CAM2_IMGSZ, verbose=False)
    if not results or results[0].boxes is None or len(results[0].boxes) == 0:
        return []

    boxes = results[0].boxes
    masks = get_seg_masks(results, frame.shape)
    idxs = [i for i, b in enumerate(boxes) if model.names[int(b.cls)].lower() not in ('wheel', 'handle')]
    if not idxs:
        idxs = list(range(len(boxes)))
    idxs = nms_filter_idxs(boxes, idxs)

    candidates = []
    for i in idxs:
        b = boxes[i]
        class_label = model.names[int(b.cls)]
        conf = float(b.conf)
        bbox = [int(v) for v in b.xyxy.tolist()[0]]
        crop, mask_crop = masked_crop(frame, masks[i], bbox)
        if crop is not None and crop.size > 0:
            candidates.append((class_label, bbox, conf, crop, mask_crop))
    return candidates


def compute_validation_window(baseline_timestamp: float) -> Tuple[float, float]:
    """The core business rule: a Cam2 candidate is only even considered for
    this baggage_id if it falls between MIN_VALIDATION_TRANSIT_S and
    MAX_VALIDATION_TRANSIT_S after the Zone1 baseline (1min-1min30s) - not a
    belt-speed physics estimate, which didn't match the real expected
    timing. A candidate outside this window is not this bag, regardless of
    how well it scores on SIFT/color/class."""
    t_min = baseline_timestamp + config.MIN_VALIDATION_TRANSIT_S
    t_max = baseline_timestamp + config.MAX_VALIDATION_TRANSIT_S
    return t_min, t_max


def is_within_transit_cap(baseline_timestamp: float, candidate_timestamp: float) -> bool:
    """Hard 1min30s cap on Cam1->Cam2 transit time, independent of belt-speed
    physics. Guards against a stale/mismatched candidate slipping through if
    belt speed assumptions drift or the physics window is misconfigured."""
    return (candidate_timestamp - baseline_timestamp) <= config.MAX_VALIDATION_TRANSIT_S


def parse_frame_timestamp(path: str) -> Optional[float]:
    """Parses a `img_HHMMSS_ffffff.ext` filename into seconds-of-day."""
    m = _IMG_TIMESTAMP_RE.search(os.path.basename(path))
    if not m:
        return None
    hh, mm, ss, frac = m.groups()
    return int(hh) * 3600 + int(mm) * 60 + int(ss) + int(frac) / 1_000_000.0


def scan_cam2_candidates(cam2_dir: str, t_min: float, t_max: float) -> List[Tuple[float, str]]:
    """Lists Cam2 frames under cam2_dir whose parsed timestamp falls within
    [t_min, t_max], sorted chronologically."""
    out = []
    for path in glob.glob(os.path.join(cam2_dir, "*")):
        ts = parse_frame_timestamp(path)
        if ts is not None and t_min <= ts <= t_max:
            out.append((ts, path))
    return sorted(out, key=lambda x: x[0])


def evaluate_cycle_result(
    baggage_id: str,
    baseline_found: bool,
    frame_detected: bool,
    within_transit_cap: bool,
    sift_score: float,
    class_match: bool,
    color_score: float,
    final_score: float,
    cycle_count: int
) -> ValidationResult:
    if not baseline_found or not frame_detected:
        status = config.STATUS_ERROR_VALIDATE
    elif not within_transit_cap:
        status = config.STATUS_TRANSIT_TIME_EXCEEDED
    elif final_score >= config.FUSION_PASS_THRESHOLD and sift_score > 0.0:
        # sift_score > 0.0 is a hard requirement, not just a weighted
        # contributor: with FUSION_WEIGHT_CLASS + FUSION_WEIGHT_COLOR already
        # summing to 0.50, a candidate with ZERO geometric evidence (e.g. a
        # crop too small for SIFT at all, or a real non-match) could already
        # reach the pass threshold on class+color alone if color correlation
        # happened to be strong - two different same-colored bags of the
        # same class, for instance. Verified on real captures: three
        # genuinely different bags scored 0.32-0.36 on class+color alone
        # (no SIFT support). Requiring real verified geometric matches closes
        # that gap regardless of how the threshold is tuned.
        status = config.STATUS_MATCHED
    else:
        status = config.STATUS_NOT_MATCHED
    return ValidationResult(
        baggage_id=baggage_id,
        status=status,
        sift_score=round(sift_score, 4),
        class_match=class_match,
        color_score=round(color_score, 4),
        final_score=round(final_score, 4),
        cycle_count=cycle_count,
    )
