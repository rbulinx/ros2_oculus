"""Small fault-tolerant pymavlink UDP sender."""

import os
from typing import Callable, Optional


class MavlinkSender:
    """Own a lazy UDP-out pymavlink connection and send MANUAL_CONTROL."""

    def __init__(
        self,
        unity_ip: str,
        unity_port: int,
        target_system: int,
        connection_factory: Optional[Callable] = None,
    ) -> None:
        self.unity_ip = unity_ip
        self.unity_port = int(unity_port)
        self.target_system = int(target_system)
        self._connection_factory = connection_factory
        self._connection = None

    @property
    def endpoint(self) -> str:
        return f"udpout:{self.unity_ip}:{self.unity_port}"

    def connect(self) -> None:
        if self._connection is not None:
            return
        factory = self._connection_factory
        if factory is None:
            # Unity accepts MAVLink v1/v2; prefer v2 while retaining v1 message compatibility.
            os.environ.setdefault("MAVLINK20", "1")
            from pymavlink import mavutil

            factory = mavutil.mavlink_connection
        self._connection = factory(
            self.endpoint,
            source_system=255,
            source_component=190,
            autoreconnect=True,
        )

    def send_manual_control(self, x: int, y: int, z: int, r: int) -> None:
        self.connect()
        try:
            self._connection.mav.manual_control_send(
                self.target_system,
                int(x),
                int(y),
                int(z),
                int(r),
                0,
            )
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        connection = self._connection
        self._connection = None
        if connection is None:
            return
        try:
            connection.close()
        except Exception:
            pass
