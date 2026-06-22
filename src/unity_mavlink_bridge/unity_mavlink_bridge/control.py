"""Pure command conversion and cable-following control logic."""

from dataclasses import dataclass
import math
from typing import Any, Optional


def finite_clamped(value: Any, lower: float = -1.0, upper: float = 1.0) -> float:
    """Convert non-finite/invalid inputs to zero and clamp finite values."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(number):
        return 0.0
    return min(upper, max(lower, number))


@dataclass(frozen=True)
class ManualControl:
    """Normalized MANUAL_CONTROL axes before conversion to MAVLink integers."""

    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    yaw: float = 0.0

    def mavlink_values(
        self,
        invert_sway: bool = False,
        invert_heave: bool = False,
        invert_yaw: bool = False,
    ) -> tuple[int, int, int, int]:
        """Return x, y, z and r in Unity's -1000..1000 convention."""
        sway = -self.sway if invert_sway else self.sway
        heave = -self.heave if invert_heave else self.heave
        yaw = -self.yaw if invert_yaw else self.yaw
        return tuple(
            int(round(1000.0 * finite_clamped(value)))
            for value in (self.surge, sway, heave, yaw)
        )


@dataclass(frozen=True)
class FollowResult:
    """Controller output plus a concise operating state."""

    command: ManualControl
    state: str
    distance_m: Optional[float]


def compute_follow_command(
    tracking: dict[str, Any],
    manual_heave: float,
    target_distance_m: float,
    distance_kp: float,
    maximum_surge: float,
    sway_kp: float,
    maximum_sway: float,
    distance_deadband_m: float,
    center_deadband: float,
) -> FollowResult:
    """Hold distance with surge and center with sway while leaving yaw at zero."""
    state = str(tracking.get("state", "LOST"))
    camera = tracking.get("camera") or {}
    sonar = tracking.get("sonar") or {}
    camera_detected = bool(camera.get("detected", False))
    sonar_detected = bool(sonar.get("detected", False))
    heave = finite_clamped(manual_heave)

    surge = 0.0
    distance = None
    if sonar_detected:
        candidate = sonar.get("range_m")
        try:
            candidate = float(candidate)
        except (TypeError, ValueError):
            candidate = math.nan
        if math.isfinite(candidate) and candidate >= 0.0:
            distance = candidate
            error = candidate - target_distance_m
            if abs(error) > max(0.0, distance_deadband_m):
                surge = finite_clamped(
                    distance_kp * error,
                    -abs(maximum_surge),
                    abs(maximum_surge),
                )

    sway = 0.0
    if camera_detected:
        lateral_error = finite_clamped(camera.get("lateral_error", 0.0))
        if abs(lateral_error) > max(0.0, center_deadband):
            # Camera +x and Unity sway/y are both positive to the right.
            sway = finite_clamped(
                sway_kp * lateral_error,
                -abs(maximum_sway),
                abs(maximum_sway),
            )
    elif sonar_detected:
        # Sonar bearing follows ROS convention (positive left), while Unity y is right-positive.
        bearing_deg = sonar.get("bearing_deg", 0.0)
        try:
            bearing_deg = float(bearing_deg)
        except (TypeError, ValueError):
            bearing_deg = 0.0
        if math.isfinite(bearing_deg):
            sway = finite_clamped(
                sway_kp * (-bearing_deg / 30.0),
                -abs(maximum_sway),
                abs(maximum_sway),
            )

    if not sonar_detected:
        surge = 0.0
    controller_state = state if (camera_detected or sonar_detected) else "LOST_STOP"
    return FollowResult(
        command=ManualControl(surge=surge, sway=sway, heave=heave, yaw=0.0),
        state=controller_state,
        distance_m=distance,
    )
