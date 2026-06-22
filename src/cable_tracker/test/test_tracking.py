"""Algorithm tests for cable tracking with changing cable width and sonar flicker."""

import math

import cv2
import numpy as np

from cable_tracker.tracking import (
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
    assert 0.0 < result.lateral_error < 0.3
    assert result.centerline.shape[0] > 20


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
