from unity_mavlink_bridge.mavlink_sender import MavlinkSender


class FakeMav:
    def __init__(self):
        self.calls = []

    def manual_control_send(self, *values):
        self.calls.append(values)


class FakeConnection:
    def __init__(self):
        self.mav = FakeMav()
        self.closed = False

    def close(self):
        self.closed = True


def test_sender_uses_udpout_and_manual_control_only():
    created = {}

    def factory(endpoint, **options):
        created["endpoint"] = endpoint
        created["options"] = options
        created["connection"] = FakeConnection()
        return created["connection"]

    sender = MavlinkSender("192.168.50.177", 14550, 1, connection_factory=factory)
    sender.send_manual_control(100, -200, 300, -400)

    assert created["endpoint"] == "udpout:192.168.50.177:14550"
    assert created["connection"].mav.calls == [(1, 100, -200, 300, -400, 0)]
    sender.close()
    assert created["connection"].closed
