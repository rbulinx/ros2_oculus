"""Cable-follow controller producing /rov/cmd_vel."""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from unity_mavlink_bridge.control import ManualControl, compute_follow_command, finite_clamped


LATEST_ONLY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class CableFollowController(Node):
    """Hold target distance and center it while passing through manual heave."""

    def __init__(self) -> None:
        super().__init__("cable_follow_controller")
        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.target_distance_m = max(
            0.0, float(self.declare_parameter("target_distance_m", 2.0).value)
        )
        self.control_rate_hz = max(
            1.0, float(self.declare_parameter("control_rate_hz", 20.0).value)
        )
        self.tracking_timeout = max(
            0.0, float(self.declare_parameter("tracking_timeout_seconds", 0.5).value)
        )
        self.manual_timeout = max(
            0.0, float(self.declare_parameter("manual_timeout_seconds", 0.5).value)
        )
        self.distance_kp = float(self.declare_parameter("distance_kp", 0.35).value)
        self.maximum_surge = abs(
            float(self.declare_parameter("maximum_surge", 0.40).value)
        )
        self.sway_kp = float(self.declare_parameter("sway_kp", 0.80).value)
        self.maximum_sway = abs(float(self.declare_parameter("maximum_sway", 0.50).value))
        self.distance_deadband = max(
            0.0, float(self.declare_parameter("distance_deadband_m", 0.10).value)
        )
        self.center_deadband = max(
            0.0, float(self.declare_parameter("center_deadband", 0.03).value)
        )

        self.latest_tracking: dict = {"state": "LOST"}
        self.last_tracking_time: float | None = None
        self.manual_heave = 0.0
        self.last_manual_time: float | None = None

        self.command_pub = self.create_publisher(Twist, "/rov/cmd_vel", LATEST_ONLY_QOS)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(
            String,
            "/cable_tracking/state",
            self.tracking_callback,
            LATEST_ONLY_QOS,
        )
        self.create_subscription(
            Twist,
            "/rov/manual_cmd_vel",
            self.manual_callback,
            LATEST_ONLY_QOS,
        )
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_step)
        self.get_logger().info(
            f"Cable following enabled={self.enabled}, target distance={self.target_distance_m:.2f} m"
        )

    def tracking_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.latest_tracking = payload
            self.last_tracking_time = time.monotonic()

    def manual_callback(self, message: Twist) -> None:
        # Automatic control never owns heave; only this manual input does.
        self.manual_heave = finite_clamped(message.linear.z)
        self.last_manual_time = time.monotonic()

    def control_step(self) -> None:
        now = time.monotonic()
        tracking_fresh = (
            self.last_tracking_time is not None
            and now - self.last_tracking_time <= self.tracking_timeout
        )
        manual_fresh = (
            self.last_manual_time is not None
            and now - self.last_manual_time <= self.manual_timeout
        )
        tracking = self.latest_tracking if tracking_fresh else {"state": "LOST"}
        heave = self.manual_heave if manual_fresh else 0.0
        result = compute_follow_command(
            tracking=tracking,
            manual_heave=heave,
            target_distance_m=self.target_distance_m,
            distance_kp=self.distance_kp,
            maximum_surge=self.maximum_surge,
            sway_kp=self.sway_kp,
            maximum_sway=self.maximum_sway,
            distance_deadband_m=self.distance_deadband,
            center_deadband=self.center_deadband,
        )
        command = result.command if self.enabled else ManualControl(heave=heave)

        message = Twist()
        message.linear.x = command.surge
        message.linear.y = command.sway
        message.linear.z = command.heave
        message.angular.z = command.yaw
        self.command_pub.publish(message)

        status = String()
        status.data = json.dumps(
            {
                "enabled": self.enabled,
                "state": result.state,
                "target_distance_m": self.target_distance_m,
                "distance_m": result.distance_m,
                "surge": command.surge,
                "sway": command.sway,
                "heave": command.heave,
                "yaw": command.yaw,
            },
            separators=(",", ":"),
            allow_nan=False,
        )
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CableFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
