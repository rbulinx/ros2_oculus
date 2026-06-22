"""ROS 2 Twist to Unity MAVLink MANUAL_CONTROL bridge."""

import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from unity_mavlink_bridge.control import ManualControl, finite_clamped
from unity_mavlink_bridge.mavlink_sender import MavlinkSender


LATEST_ONLY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class UnityMavlinkBridge(Node):
    """Continuously send only the newest Twist as MANUAL_CONTROL."""

    def __init__(self) -> None:
        super().__init__("unity_mavlink_bridge")
        self.unity_ip = str(self.declare_parameter("unity_ip", "127.0.0.1").value)
        self.unity_port = int(self.declare_parameter("unity_port", 14550).value)
        self.send_rate_hz = max(
            1.0, float(self.declare_parameter("send_rate_hz", 20.0).value)
        )
        self.command_timeout = max(
            0.0,
            float(self.declare_parameter("command_timeout_seconds", 0.5).value),
        )
        self.target_system = int(self.declare_parameter("target_system", 1).value)
        self.invert_sway = bool(self.declare_parameter("invert_sway", False).value)
        self.invert_heave = bool(self.declare_parameter("invert_heave", False).value)
        self.invert_yaw = bool(self.declare_parameter("invert_yaw", False).value)

        if not self.unity_ip or not 1 <= self.unity_port <= 65535:
            raise ValueError("unity_ip must be set and unity_port must be in 1..65535")

        self.sender = MavlinkSender(
            self.unity_ip,
            self.unity_port,
            self.target_system,
        )
        self.latest_command = ManualControl()
        self.last_command_time: float | None = None
        self.last_error_log_time = -math.inf
        self.sent_stop = False

        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.create_subscription(Twist, "/rov/cmd_vel", self.command_callback, LATEST_ONLY_QOS)
        self.timer = self.create_timer(1.0 / self.send_rate_hz, self.send_latest)
        self.get_logger().info(
            f"Sending MANUAL_CONTROL to {self.sender.endpoint} at {self.send_rate_hz:.1f} Hz"
        )

    def command_callback(self, message: Twist) -> None:
        self.latest_command = ManualControl(
            surge=finite_clamped(message.linear.x),
            sway=finite_clamped(message.linear.y),
            heave=finite_clamped(message.linear.z),
            yaw=finite_clamped(message.angular.z),
        )
        self.last_command_time = time.monotonic()

    def send_latest(self) -> None:
        now = time.monotonic()
        fresh = (
            self.last_command_time is not None
            and now - self.last_command_time <= self.command_timeout
        )
        command = self.latest_command if fresh else ManualControl()
        values = command.mavlink_values(
            invert_sway=self.invert_sway,
            invert_heave=self.invert_heave,
            invert_yaw=self.invert_yaw,
        )
        sent = self._send(values, now)
        status = String()
        status.data = json.dumps(
            {
                "connected": sent,
                "command_fresh": fresh,
                "x": values[0],
                "y": values[1],
                "z": values[2],
                "r": values[3],
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(status)

    def _send(self, values: tuple[int, int, int, int], now: float) -> bool:
        try:
            self.sender.send_manual_control(*values)
            return True
        except Exception as error:
            if now - self.last_error_log_time >= 5.0:
                self.get_logger().error(f"MAVLink send failed: {error}")
                self.last_error_log_time = now
            return False

    def send_shutdown_stop(self) -> None:
        if self.sent_stop:
            return
        self.sent_stop = True
        stop = (0, 0, 0, 0)
        for _ in range(10):
            try:
                self.sender.send_manual_control(*stop)
            except (Exception, KeyboardInterrupt):
                # Finish shutdown even if Unity, the network, or ROS is already gone.
                pass
        self.sender.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnityMavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_shutdown_stop()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
