import math

from unity_mavlink_bridge.control import (
    ManualControl,
    compute_follow_command,
    finite_clamped,
)


def test_manual_control_clamps_nonfinite_and_applies_inversion():
    command = ManualControl(surge=2.0, sway=-0.25, heave=math.nan, yaw=math.inf)

    assert command.mavlink_values(invert_sway=True, invert_heave=True, invert_yaw=True) == (
        1000,
        250,
        0,
        0,
    )
    assert finite_clamped(-3.0) == -1.0


def test_fused_tracking_holds_two_meters_and_centers_target():
    tracking = {
        "state": "FUSED_TRACK",
        "camera": {"detected": True, "lateral_error": 0.5},
        "sonar": {"detected": True, "range_m": 3.0, "bearing_deg": -10.0},
    }
    result = compute_follow_command(
        tracking,
        manual_heave=-0.2,
        target_distance_m=2.0,
        distance_kp=0.35,
        maximum_surge=0.4,
        sway_kp=0.8,
        maximum_sway=0.5,
        distance_deadband_m=0.1,
        center_deadband=0.03,
    )

    assert result.command.surge == 0.35
    assert result.command.sway == 0.4
    assert result.command.heave == -0.2
    assert result.command.yaw == 0.0
    assert result.distance_m == 3.0


def test_lost_target_stops_automatic_axes_but_keeps_manual_heave():
    result = compute_follow_command(
        {"state": "LOST"},
        manual_heave=0.3,
        target_distance_m=2.0,
        distance_kp=0.35,
        maximum_surge=0.4,
        sway_kp=0.8,
        maximum_sway=0.5,
        distance_deadband_m=0.1,
        center_deadband=0.03,
    )

    assert result.state == "LOST_STOP"
    assert result.command == ManualControl(heave=0.3)
