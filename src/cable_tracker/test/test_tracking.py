"""Algorithm tests for cable tracking with changing cable width and sonar flicker."""

import math

import cv2
import numpy as np

from cable_tracker.tracking import (
    CameraTrack,
    CameraTrackLatch,
    SonarPersistenceTracker,
    sonar_bearing_to_image_x,
    track_green_cable,
)


def test_camera_tracker_handles_a_wide_float():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:] = (80, 35, 10)
    cable_points = np.asarray([[250, 0], [300, 180], [360, 479]], dtype=np.int32)
    cv2.polylines(image, [cable_points], False, (0, 180, 0), 14)
    cv2.circle(image, (315, 250), 42, (0, 180, 0), -1)

    result = track_green_cable(
        image,
        hsv_lower=(35, 70, 35),
        hsv_upper=(95, 255, 255),
        roi_top_fraction=0.0,
        row_step=4,
        morphology_kernel=5,
        minimum_rows=12,
        lookahead_fraction=0.8,
        horizontal_fov_rad=math.radians(90.0),
    )

    assert result.detected
    assert result.confidence > 0.6
    assert abs(result.lateral_error) < 0.15
    assert result.centerline.shape[0] > 20
    assert np.max(np.linalg.norm(np.diff(result.centerline, axis=0), axis=1)) < 10.0


def test_camera_tracker_handles_horizontal_and_both_diagonals():
    lines = [
        ((40, 240), (600, 240)),
        ((40, 400), (600, 80)),
        ((40, 80), (600, 400)),
    ]
    for start, end in lines:
        image = np.full((480, 640, 3), (80, 35, 10), dtype=np.uint8)
        cv2.line(image, start, end, (0, 180, 0), 14, cv2.LINE_AA)
        result = track_green_cable(
            image,
            hsv_lower=(35, 70, 35),
            hsv_upper=(95, 255, 255),
            roi_top_fraction=0.0,
            row_step=4,
            morphology_kernel=3,
            minimum_rows=6,
            lookahead_fraction=0.8,
            horizontal_fov_rad=math.radians(90.0),
        )

        assert result.detected
        assert result.confidence > 0.7
        assert abs(result.lateral_error) < 0.1
        assert result.centerline.shape[0] > 50
        assert np.max(np.linalg.norm(np.diff(result.centerline, axis=0), axis=1)) < 10.0


def test_camera_latch_bridges_short_dropout_then_reports_loss():
    mask = np.ones((10, 10), dtype=np.uint8)
    valid = CameraTrack(
        detected=True,
        mask=mask,
        centerline=np.asarray([[1.0, 1.0], [8.0, 8.0]], dtype=np.float32),
        confidence=0.8,
    )
    missing = CameraTrack(False, np.zeros_like(mask), np.empty((0, 2), dtype=np.float32))
    latch = CameraTrackLatch(acquire_frames=2, loss_grace_seconds=0.2, minimum_confidence=0.1)

    first, _ = latch.update(valid, 0.0)
    acquired, _ = latch.update(valid, 0.05)
    held, was_held = latch.update(missing, 0.15)
    lost, _ = latch.update(missing, 0.30)

    assert not first.detected
    assert acquired.detected
    assert held.detected and was_held
    assert not lost.detected


def test_sonar_tracker_rejects_flicker_and_keeps_persistent_target():
    tracker = SonarPersistenceTracker(
        history_length=5,
        required_hits=3,
        intensity_threshold=0.25,
        morphology_kernel=3,
        minimum_cluster_pixels=8,
        bearing_gate_rad=math.radians(10.0),
        invert_bearing=True,
    )
    bearings = np.linspace(-3000, 3000, 64).astype(np.int16)
    result = None
    rng = np.random.default_rng(7)
    for _ in range(5):
        image = np.zeros((100, 64), dtype=np.uint8)
        image[38:44, 30:34] = 220
        noise_range = rng.integers(0, 100, size=30)
        noise_beam = rng.integers(0, 64, size=30)
        image[noise_range, noise_beam] = 255
        result = tracker.update(
            image,
            bearings,
            range_resolution=0.1,
            camera_bearing_rad=0.0,
        )

    assert result is not None
    assert result.detected
    assert 3.7 < result.range_m < 4.6
    assert abs(math.degrees(result.bearing_rad)) < 5.0
    assert np.count_nonzero(result.stable_mask) < 50


def test_sonar_bearing_projects_to_camera_column():
    center_x, visible = sonar_bearing_to_image_x(0.0, 0.0, math.radians(90.0), 640)
    right_x, right_visible = sonar_bearing_to_image_x(
        math.radians(-10.0), 0.0, math.radians(90.0), 640
    )

    assert visible
    assert right_visible
    assert abs(center_x - 319.5) < 0.1
    assert right_x > center_x
