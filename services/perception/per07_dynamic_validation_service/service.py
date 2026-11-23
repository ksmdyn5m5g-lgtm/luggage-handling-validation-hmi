"""
PER_007 (per07_dynamic_validation_service) - ValidationService

The "brain": baseline extraction + dynamic re-id validation, decoupled from
any particular entrypoint (CLI, notebook, or the live Redis worker in
main.py) so it stays independently testable. Flat/absolute imports, not
relative package imports - matches every sibling service in this repo
(per01, sns01, sns03), all of which run as plain `python main.py`, not as
an installed package. Python auto-adds a script's own directory to
sys.path, so `import config` / `from logic import ...` just work as long as
this file, config.py and logic.py sit side by side.
"""

import sys
from pathlib import Path
from typing import Dict, Any, List, Optional, Union

import cv2
import numpy as np
import open3d as o3d
from ultralytics import YOLO

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import config
from logic import (
    extract_dimensions_fused,
    detect_cam2_candidates,
    extract_sift_descriptors,
    extract_color_histogram,
    match_sift_raw,
    geometric_verify_matches,
    verified_sift_score,
    match_color_histograms,
    normalize_class_label,
    fuse_confidence,
    compute_validation_window,
    is_within_transit_cap,
    scan_cam2_candidates,
    evaluate_cycle_result,
    ValidationResult
)


def _load_frame(source: Union[str, np.ndarray]) -> Optional[np.ndarray]:
    """Accepts either a file path (offline/notebook testing) or an
    already-decoded frame array (the live Redis path, where main.py decodes
    the stream payload in-memory - there's no file on disk to read, see
    per01_luggage_detection's store_data_enabled gating)."""
    if isinstance(source, np.ndarray):
        return source
    return cv2.imread(source)


def _load_pcd(source: Optional[Union[str, "o3d.geometry.PointCloud"]]) -> Optional["o3d.geometry.PointCloud"]:
    """Same dual-mode acceptance as _load_frame, for point clouds. None is
    allowed and passed through: re-identification only needs the 2D
    signature, so a LiDAR gap should cost the dimensions, not the whole
    baseline."""
    if source is None:
        return None
    if isinstance(source, o3d.geometry.PointCloud):
        return source
    return o3d.io.read_point_cloud(source)


class ValidationService:
    def __init__(
        self,
        cam1_model_path: str = config.CAM1_MODEL_PATH,
        cam2_model_path: str = config.CAM2_MODEL_PATH,
    ):
        print(f"Loading Cam1 (Zone 1) YOLO model from {cam1_model_path}...")
        self.model_cam1 = YOLO(cam1_model_path)
        print(f"Loading Cam2 (Zone 2) YOLO model from {cam2_model_path}...")
        self.model_cam2 = YOLO(cam2_model_path)

        self.baseline_store: Dict[str, Dict[str, Any]] = {}

    def process_baseline(
        self,
        baggage_id: str,
        timestamp: float,
        cam1_frame: Union[str, np.ndarray],
        lidar_pcd: Union[str, "o3d.geometry.PointCloud"],
        bbox: Optional[List[int]] = None,
        class_label: Optional[str] = None,
    ) -> Optional[Dict[str, Any]]:
        """cam1_frame/lidar_pcd each accept either a file path (offline/
        notebook testing) or an already-decoded frame/point cloud (the live
        path, decoded in main.py straight from the Redis payload bytes).

        bbox/class_label come from per04_luggage_tracking on the live path -
        per04 has already detected and identified this luggage, so per07
        crops that box rather than running a second YOLO pass over the same
        frame. Omit them offline and the Cam1 model runs its own detection."""
        pcd = _load_pcd(lidar_pcd)
        frame = _load_frame(cam1_frame)

        baseline = extract_dimensions_fused(
            frame, pcd, self.model_cam1, bbox=bbox, class_label=class_label,
        )
        if baseline:
            baseline["baggage_id"] = baggage_id
            baseline["timestamp"] = timestamp
            self.baseline_store[baggage_id] = baseline
            print(f"[Baseline] Registered {baggage_id} at t={timestamp:.3f}s: Class={baseline['class_label']}, Dims(m)={baseline['dimensions_m']}")
            if baseline.get("ambiguous"):
                # No baseline exists yet at Zone 1 to score candidates against
                # (unlike Zone 2), so with >1 plausible bag in frame this pick
                # is a positional/size guess that cannot be verified
                # automatically - flag it so a human can review/correct it
                # rather than let a possibly-wrong baseline pass silently.
                print(f"[Baseline] WARNING: {baseline['candidate_count']} plausible bags were in this Cam1 "
                      f"frame - {baggage_id}'s baseline was picked by position/size heuristic, not verified "
                      f"identity. Recommend manual review of this registration.")
        else:
            print(f"[Baseline] Failed to extract baseline for {baggage_id}")
        return baseline

    def validate_candidate(
        self,
        baggage_id: str,
        candidate_timestamp: float,
        cam2_frame: Union[str, np.ndarray],
        cycle_count: int = 1,
    ) -> ValidationResult:
        """cam2_frame accepts either a file path or an already-decoded frame
        (see process_baseline's docstring)."""
        if baggage_id not in self.baseline_store:
            return ValidationResult(baggage_id=baggage_id, status=config.STATUS_ERROR_VALIDATE)

        baseline = self.baseline_store[baggage_id]
        within_cap = is_within_transit_cap(baseline["timestamp"], candidate_timestamp)
        elapsed = round(candidate_timestamp - baseline["timestamp"], 3)

        cam2_frame = _load_frame(cam2_frame)
        if cam2_frame is None:
            res = ValidationResult(baggage_id=baggage_id, status=config.STATUS_ERROR_VALIDATE, cycle_count=cycle_count)
            res.cam2_timestamp = candidate_timestamp
            res.elapsed_transit_s = elapsed
            return res

        # Zones can hold multiple bags at once, so every candidate in the frame
        # is scored against the baseline - identity is decided by which one
        # actually matches (SIFT+class+color), never by proximity/confidence
        # alone. Picking a single "closest" box first (as this used to do) risks
        # comparing against the wrong physical bag whenever more than one is
        # in frame, which is exactly the "keeps showing a different luggage"
        # symptom this fixes.
        candidates = detect_cam2_candidates(self.model_cam2, cam2_frame)
        if not candidates:
            res = evaluate_cycle_result(
                baggage_id=baggage_id, baseline_found=True, frame_detected=False,
                within_transit_cap=within_cap, sift_score=0.0, class_match=False,
                color_score=0.0, final_score=0.0, cycle_count=cycle_count,
            )
            res.cam2_timestamp = candidate_timestamp
            res.elapsed_transit_s = elapsed
            return res

        best_res = None
        best_final_score = -1.0
        for frame_class, bbox, conf, cam2_crop, cam2_mask in candidates:
            frame_keypoints, frame_descriptors = extract_sift_descriptors(cam2_crop, cam2_mask)
            frame_hist = extract_color_histogram(cam2_crop)

            good_matches = match_sift_raw(baseline["sift_descriptors"], frame_descriptors)
            inlier_matches, _inlier_ratio = geometric_verify_matches(baseline["sift_keypoints"], frame_keypoints, good_matches)
            sift_score = verified_sift_score(inlier_matches, baseline["sift_descriptors"], frame_descriptors)
            class_match = (normalize_class_label(baseline["class_label"]) == normalize_class_label(frame_class))
            color_score = match_color_histograms(baseline.get("color_hist"), frame_hist)
            final_score = fuse_confidence(sift_score, class_match, color_score)

            res = evaluate_cycle_result(
                baggage_id=baggage_id,
                baseline_found=True,
                frame_detected=True,
                within_transit_cap=within_cap,
                sift_score=sift_score,
                class_match=class_match,
                color_score=color_score,
                final_score=final_score,
                cycle_count=cycle_count,
            )
            res.dimensions_m = baseline.get("dimensions_m")
            res.inlier_match_count = len(inlier_matches)
            res.cam2_timestamp = candidate_timestamp
            res.elapsed_transit_s = elapsed
            res.cam2_bbox = bbox

            if final_score > best_final_score:
                best_final_score = final_score
                best_res = res

        return best_res

    def run_dynamic_validation(self, baggage_id: str, cam2_dir: str) -> ValidationResult:
        """Dynamic multi-cycle validation: scans Cam2 frames chronologically
        within the [MIN_VALIDATION_TRANSIT_S, MAX_VALIDATION_TRANSIT_S]
        arrival window (1min-1min30s), evaluating up to config.MAX_CYCLE of
        them and stopping at the first MATCHED result."""
        if baggage_id not in self.baseline_store:
            return ValidationResult(baggage_id=baggage_id, status=config.STATUS_ERROR_VALIDATE)

        baseline = self.baseline_store[baggage_id]
        t_min, t_max = compute_validation_window(baseline["timestamp"])

        candidates = scan_cam2_candidates(cam2_dir, t_min, t_max)
        print(f"[Dynamic] {baggage_id}: window=[{t_min:.3f},{t_max:.3f}]s, candidates={len(candidates)}")

        if not candidates:
            return ValidationResult(baggage_id=baggage_id, status=config.STATUS_NOT_MATCHED, cycle_count=0)

        last_result = None
        for cycle_count, (ts, path) in enumerate(candidates[:config.MAX_CYCLE], start=1):
            last_result = self.validate_candidate(baggage_id, ts, path, cycle_count=cycle_count)
            print(
                f"[Cycle {cycle_count}/{config.MAX_CYCLE}] {baggage_id} @ t={ts:.3f}s "
                f"(+{ts - baseline['timestamp']:.1f}s) -> {last_result.status} (score={last_result.final_score:.3f})"
            )
            if last_result.status in (config.STATUS_MATCHED, config.STATUS_TRANSIT_TIME_EXCEEDED):
                return last_result

        if last_result.status == config.STATUS_NOT_MATCHED:
            last_result.status = config.STATUS_MAX_CYCLE_EXCEEDED
        return last_result
