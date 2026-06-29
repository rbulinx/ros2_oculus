"""Image-space cable extraction and temporal sonar filtering algorithms."""

from collections import deque
from dataclasses import dataclass, replace
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
    target_pixel: tuple[float, float] = (0.0, 0.0)


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


def _skeletonize(mask: np.ndarray) -> np.ndarray:
    """Return a connected one-pixel Zhang-Suen skeleton without opencv-contrib."""
    image = (mask > 0).astype(np.uint8)

    def neighbors(source: np.ndarray) -> tuple[np.ndarray, ...]:
        padded = np.pad(source, 1, mode="constant")
        return (
            padded[:-2, 1:-1],   # p2: north
            padded[:-2, 2:],     # p3: north-east
            padded[1:-1, 2:],    # p4: east
            padded[2:, 2:],      # p5: south-east
            padded[2:, 1:-1],    # p6: south
            padded[2:, :-2],     # p7: south-west
            padded[1:-1, :-2],   # p8: west
            padded[:-2, :-2],    # p9: north-west
        )

    changed = True
    while changed:
        changed = False
        for first_step in (True, False):
            adjacent = neighbors(image)
            neighbor_count = np.sum(adjacent, axis=0)
            transitions = sum(
                ((adjacent[index] == 0) & (adjacent[(index + 1) % 8] == 1))
                for index in range(8)
            )
            p2, _, p4, _, p6, _, p8, _ = adjacent
            if first_step:
                triplet_a = p2 * p4 * p6
                triplet_b = p4 * p6 * p8
            else:
                triplet_a = p2 * p4 * p8
                triplet_b = p2 * p6 * p8
            remove = (
                (image == 1)
                & (neighbor_count >= 2)
                & (neighbor_count <= 6)
                & (transitions == 1)
                & (triplet_a == 0)
                & (triplet_b == 0)
            )
            if np.any(remove):
                image[remove] = 0
                changed = True
    return image * 255


def _farthest_skeleton_point(
    points: set[tuple[int, int]],
    start: tuple[int, int],
) -> tuple[tuple[int, int], dict[tuple[int, int], tuple[int, int]]]:
    """Find an approximate graph-diameter endpoint with one breadth-first pass."""
    queue = deque([start])
    distance = {start: 0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    farthest = start
    neighbors = (
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1), (0, 1),
        (1, -1), (1, 0), (1, 1),
    )
    while queue:
        current = queue.popleft()
        if distance[current] > distance[farthest]:
            farthest = current
        for dy, dx in neighbors:
            candidate = (current[0] + dy, current[1] + dx)
            if candidate in points and candidate not in distance:
                distance[candidate] = distance[current] + 1
                parent[candidate] = current
                queue.append(candidate)
    return farthest, parent


def _longest_skeleton_path(skeleton: np.ndarray) -> np.ndarray:
    """Extract one continuous graph-diameter path and ignore side branches."""
    component_count, component_labels = cv2.connectedComponents(skeleton, connectivity=8)
    if component_count <= 1:
        return np.empty((0, 2), dtype=np.float32)
    component_sizes = np.bincount(component_labels.ravel())
    component_sizes[0] = 0
    largest_component = int(np.argmax(component_sizes))
    rows, columns = np.nonzero(component_labels == largest_component)
    points = set(zip(rows.tolist(), columns.tolist()))
    endpoint_a, _ = _farthest_skeleton_point(points, next(iter(points)))
    endpoint_b, parent = _farthest_skeleton_point(points, endpoint_a)
    path = [endpoint_b]
    while path[-1] != endpoint_a:
        previous = parent.get(path[-1])
        if previous is None:
            break
        path.append(previous)
    path.reverse()
    return np.asarray([(column, row) for row, column in path], dtype=np.float32)


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
    """Estimate an orientation-independent skeleton centerline."""
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
        _, _, component_width, component_height, area = stats[label]
        if area < minimum_rows:
            continue
        component_span = math.hypot(component_width, component_height)
        score = area * (1.0 + component_span / max(1.0, math.hypot(width, height)))
        if score > best_score:
            best_score = score
            best_label = label

    if best_label == 0:
        return CameraTrack(False, mask, np.empty((0, 2), dtype=np.float32))

    cable_mask = np.where(labels == best_label, 255, 0).astype(np.uint8)
    skeleton = _skeletonize(cable_mask)
    path = _longest_skeleton_path(skeleton)
    minimum_points = max(8, int(minimum_rows))
    if path.shape[0] < minimum_points:
        return CameraTrack(False, cable_mask, path)

    path_points = path.astype(np.float64)
    center = np.mean(path_points, axis=0)
    covariance = np.cov(path_points - center, rowvar=False)
    _, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, -1]
    if (abs(axis[1]) >= abs(axis[0]) and axis[1] > 0.0) or (
        abs(axis[0]) > abs(axis[1]) and axis[0] < 0.0
    ):
        axis = -axis

    sample_step = max(1, int(row_step))
    centerline = path[::sample_step]
    if not np.array_equal(centerline[-1], path[-1]):
        centerline = np.vstack((centerline, path[-1]))
    if centerline.shape[0] < minimum_points:
        return CameraTrack(False, cable_mask, centerline)

    normalized_dx = (centerline[:, 0] - width * 0.5) / max(1.0, width * 0.5)
    normalized_dy = (centerline[:, 1] - height * 0.5) / max(1.0, height * 0.5)
    target_index = int(np.argmin(normalized_dx ** 2 + normalized_dy ** 2))
    target_x = float(centerline[target_index, 0])
    target_y = float(centerline[target_index, 1])
    target_x_normalized = float(
        np.clip((target_x - width * 0.5) / max(1.0, width * 0.5), -1.0, 1.0)
    )

    segment_lengths = np.linalg.norm(np.diff(path_points, axis=0), axis=1)
    path_length = float(np.sum(segment_lengths))
    image_diagonal = max(1.0, math.hypot(width, height))
    confidence = float(np.clip(path_length / (0.30 * image_diagonal), 0.0, 1.0))
    bearing_rad = math.atan(target_x_normalized * math.tan(horizontal_fov_rad * 0.5))
    heading_error_rad = math.atan2(float(axis[0]), float(-axis[1]))

    return CameraTrack(
        detected=True,
        mask=cable_mask,
        centerline=centerline,
        lateral_error=target_x_normalized,
        bearing_rad=bearing_rad,
        heading_error_rad=heading_error_rad,
        confidence=confidence,
        target_pixel=(target_x, target_y),
    )


class CameraTrackLatch:
    """Debounce acquisition and bridge only short camera-detection dropouts."""

    def __init__(
        self,
        acquire_frames: int,
        loss_grace_seconds: float,
        minimum_confidence: float,
    ) -> None:
        self.acquire_frames = max(1, int(acquire_frames))
        self.loss_grace_seconds = max(0.0, float(loss_grace_seconds))
        self.minimum_confidence = max(0.0, float(minimum_confidence))
        self._hit_count = 0
        self._last_good_time: Optional[float] = None
        self._stable_track: Optional[CameraTrack] = None

    def update(self, track: CameraTrack, now_seconds: float) -> tuple[CameraTrack, bool]:
        valid = track.detected and track.confidence >= self.minimum_confidence
        if valid:
            self._hit_count += 1
            self._last_good_time = now_seconds
            if self._stable_track is not None or self._hit_count >= self.acquire_frames:
                self._stable_track = track
                return track, False
            return replace(track, detected=False, confidence=0.0), False

        self._hit_count = 0
        if self._stable_track is not None and self._last_good_time is not None:
            age = max(0.0, now_seconds - self._last_good_time)
            if age <= self.loss_grace_seconds:
                decay = 1.0 - age / max(1.0e-6, self.loss_grace_seconds)
                return replace(
                    self._stable_track,
                    confidence=self._stable_track.confidence * decay,
                ), True
        self._stable_track = None
        return replace(track, detected=False, confidence=0.0), False


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
