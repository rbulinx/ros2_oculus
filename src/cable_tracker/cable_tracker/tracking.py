"""Image-space cable extraction and temporal sonar filtering algorithms."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np


@dataclass
class CameraTrack:
    """Result of one camera cable-centerline estimate."""

    detected: bool
    mask: np.ndarray
    centerline: np.ndarray
    lateral_error: float = 0.0
    bearing_rad: float = 0.0
    heading_error_rad: float = 0.0
    confidence: float = 0.0


@dataclass
class SonarTrack:
    """Result of one persistent sonar-target estimate."""

    detected: bool
    stable_mask: np.ndarray
    target_mask: np.ndarray
    range_m: float = 0.0
    bearing_rad: float = 0.0
    confidence: float = 0.0
    intensity: float = 0.0


def _odd_kernel(value: int) -> int:
    value = max(1, int(value))
    return value if value % 2 else value + 1


def sonar_bearing_to_image_x(
    sonar_bearing_rad: float,
    camera_to_sonar_yaw_rad: float,
    horizontal_fov_rad: float,
    image_width: int,
) -> tuple[float, bool]:
    """Project a horizontal sonar bearing onto a camera image column."""
    camera_right_bearing = camera_to_sonar_yaw_rad - sonar_bearing_rad
    normalized_x = math.tan(camera_right_bearing) / max(
        1.0e-6, math.tan(horizontal_fov_rad * 0.5)
    )
    pixel_x = (normalized_x + 1.0) * 0.5 * max(0, image_width - 1)
    return pixel_x, -1.0 <= normalized_x <= 1.0


def track_green_cable(
    bgr: np.ndarray,
    hsv_lower: tuple[int, int, int],
    hsv_upper: tuple[int, int, int],
    roi_top_fraction: float,
    row_step: int,
    morphology_kernel: int,
    minimum_rows: int,
    lookahead_fraction: float,
    horizontal_fov_rad: float,
) -> CameraTrack:
    """Estimate a cable centerline without assuming a fixed cable width or shape."""
    height, width = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray(hsv_lower, dtype=np.uint8),
        np.asarray(hsv_upper, dtype=np.uint8),
    )

    roi_top = int(np.clip(roi_top_fraction, 0.0, 0.95) * height)
    mask[:roi_top, :] = 0
    kernel_size = _odd_kernel(morphology_kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return CameraTrack(False, mask, np.empty((0, 2), dtype=np.float32))

    best_label = 0
    best_score = 0.0
    for label in range(1, count):
        x, y, component_width, component_height, area = stats[label]
        if area < minimum_rows:
            continue
        bottom_fraction = (y + component_height) / max(1, height)
        vertical_coverage = component_height / max(1, height - roi_top)
        score = area * (0.5 + bottom_fraction) * (0.5 + vertical_coverage)
        if score > best_score:
            best_score = score
            best_label = label

    if best_label == 0:
        return CameraTrack(False, mask, np.empty((0, 2), dtype=np.float32))

    cable_mask = np.where(labels == best_label, 255, 0).astype(np.uint8)
    samples = []
    weights = []
    for y in range(roi_top, height, max(1, int(row_step))):
        xs = np.flatnonzero(cable_mask[y])
        if xs.size == 0:
            continue
        samples.append((float(np.median(xs)), float(y)))
        # A float can make the component wide. It should influence the centerline less.
        weights.append(1.0 / math.sqrt(float(xs.size)))

    if len(samples) < minimum_rows:
        return CameraTrack(False, cable_mask, np.asarray(samples, dtype=np.float32))

    points = np.asarray(samples, dtype=np.float64)
    y_normalized = points[:, 1] / max(1.0, float(height - 1))
    x_normalized = (points[:, 0] - width * 0.5) / max(1.0, width * 0.5)
    fit_weights = np.asarray(weights, dtype=np.float64)
    degree = min(2, len(samples) - 1)
    inliers = np.ones(len(samples), dtype=bool)

    # Two robust fitting passes reject particles and broad asymmetric float edges.
    for _ in range(2):
        coefficients = np.polyfit(
            y_normalized[inliers],
            x_normalized[inliers],
            degree,
            w=fit_weights[inliers],
        )
        residual = np.abs(x_normalized - np.polyval(coefficients, y_normalized))
        median = float(np.median(residual))
        limit = max(0.025, 3.0 * median)
        new_inliers = residual <= limit
        if np.count_nonzero(new_inliers) < minimum_rows:
            break
        inliers = new_inliers

    lookahead_y = float(np.clip(lookahead_fraction, roi_top_fraction, 1.0))
    target_x_normalized = float(np.clip(np.polyval(coefficients, lookahead_y), -1.0, 1.0))
    derivative = float(np.polyval(np.polyder(coefficients), lookahead_y))
    pixel_slope = derivative * (width * 0.5) / max(1.0, height - 1.0)

    fitted_y = np.linspace(roi_top_fraction, 1.0, 80)
    fitted_x = np.polyval(coefficients, fitted_y)
    centerline = np.column_stack(
        (
            (fitted_x * width * 0.5 + width * 0.5),
            fitted_y * (height - 1),
        )
    ).astype(np.float32)

    coverage = len(samples) * max(1, int(row_step)) / max(1.0, height - roi_top)
    inlier_ratio = np.count_nonzero(inliers) / len(samples)
    confidence = float(np.clip(coverage * inlier_ratio, 0.0, 1.0))
    bearing_rad = math.atan(target_x_normalized * math.tan(horizontal_fov_rad * 0.5))

    return CameraTrack(
        detected=True,
        mask=cable_mask,
        centerline=centerline,
        lateral_error=target_x_normalized,
        bearing_rad=bearing_rad,
        heading_error_rad=math.atan(pixel_slope),
        confidence=confidence,
    )


class SonarPersistenceTracker:
    """Suppress flicker and select a stable sonar cluster near the camera bearing."""

    def __init__(
        self,
        history_length: int,
        required_hits: int,
        intensity_threshold: float,
        morphology_kernel: int,
        minimum_cluster_pixels: int,
        bearing_gate_rad: float,
        invert_bearing: bool,
    ) -> None:
        self.history_length = max(1, int(history_length))
        self.required_hits = int(np.clip(required_hits, 1, self.history_length))
        self.intensity_threshold = float(np.clip(intensity_threshold, 0.0, 1.0))
        self.morphology_kernel = _odd_kernel(morphology_kernel)
        self.minimum_cluster_pixels = max(1, int(minimum_cluster_pixels))
        self.bearing_gate_rad = max(0.0, float(bearing_gate_rad))
        self.invert_bearing = bool(invert_bearing)
        self._history: deque[np.ndarray] = deque(maxlen=self.history_length)

    def reset(self) -> None:
        self._history.clear()

    def update(
        self,
        intensity: np.ndarray,
        bearings_centideg: np.ndarray,
        range_resolution: float,
        camera_bearing_rad: Optional[float],
    ) -> SonarTrack:
        if intensity.ndim != 2 or intensity.size == 0:
            empty = np.zeros((0, 0), dtype=np.uint8)
            return SonarTrack(False, empty, empty)

        if intensity.dtype == np.uint16:
            normalized = intensity.astype(np.float32) / 65535.0
        else:
            normalized = intensity.astype(np.float32) / 255.0

        detected = normalized >= self.intensity_threshold
        if self._history and self._history[0].shape != detected.shape:
            self.reset()
        self._history.append(detected)

        hits = np.sum(np.stack(self._history, axis=0), axis=0)
        required = min(self.required_hits, len(self._history))
        stable = np.where(hits >= required, 255, 0).astype(np.uint8)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.morphology_kernel, self.morphology_kernel),
        )
        stable = cv2.morphologyEx(stable, cv2.MORPH_OPEN, kernel)
        stable = cv2.morphologyEx(stable, cv2.MORPH_CLOSE, kernel)

        if bearings_centideg.size != intensity.shape[1] or range_resolution <= 0.0:
            return SonarTrack(False, stable, np.zeros_like(stable))

        count, labels, stats, _ = cv2.connectedComponentsWithStats(stable, connectivity=8)
        best = None
        best_score = -1.0
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.minimum_cluster_pixels:
                continue
            ranges, beams = np.nonzero(labels == label)
            bearing_values = np.deg2rad(bearings_centideg[beams].astype(np.float64) / 100.0)
            if self.invert_bearing:
                bearing_values = -bearing_values
            bearing = float(np.median(bearing_values))
            bearing_error = 0.0
            if camera_bearing_rad is not None:
                bearing_error = abs(math.atan2(
                    math.sin(bearing - camera_bearing_rad),
                    math.cos(bearing - camera_bearing_rad),
                ))
                if bearing_error > self.bearing_gate_rad:
                    continue

            persistence = float(np.mean(hits[ranges, beams]) / max(1, len(self._history)))
            mean_intensity = float(np.mean(normalized[ranges, beams]))
            score = area * persistence * (0.25 + mean_intensity) / (1.0 + 4.0 * bearing_error)
            if score > best_score:
                best_score = score
                best = (label, ranges, beams, bearing, persistence, mean_intensity)

        if best is None:
            return SonarTrack(False, stable, np.zeros_like(stable))

        label, ranges, _, bearing, persistence, mean_intensity = best
        # The median is less sensitive than the nearest return to isolated multipath echoes.
        range_m = float(np.median((ranges.astype(np.float64) + 0.5) * range_resolution))
        confidence = float(np.clip(persistence * (0.5 + mean_intensity), 0.0, 1.0))
        return SonarTrack(
            detected=True,
            stable_mask=stable,
            target_mask=np.where(labels == label, 255, 0).astype(np.uint8),
            range_m=range_m,
            bearing_rad=bearing,
            confidence=confidence,
            intensity=mean_intensity,
        )
