"""The telemetry helper requests only attitude rate, with bounded retries."""

from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rover_navigation.imu_telemetry_node import ImuTelemetry


def subject():
    client = Mock()
    client.service_is_ready.return_value = True
    future = Mock()
    future.done.return_value = False
    client.call_async.return_value = future
    return SimpleNamespace(
        connected=True,
        samples=deque(),
        attempts=0,
        next_attempt=0,
        future=None,
        client=client,
        get_logger=Mock(return_value=Mock()),
    )


def test_only_session_attitude_request_with_bounded_retries():
    node = subject()
    with patch('rover_navigation.imu_telemetry_node.time.monotonic', return_value=100):
        ImuTelemetry.tick(node)
    request = node.client.call_async.call_args.args[0]
    assert request.message_id == 31 and request.message_rate == 30.0
    for now in (116, 132, 148, 164):
        with patch('rover_navigation.imu_telemetry_node.time.monotonic', return_value=now):
            ImuTelemetry.tick(node)
    assert node.client.call_async.call_count == 3


def test_disconnect_cancels_pending_and_reconnect_rearms_request():
    node = subject()
    node.future = Mock()
    node.attempts = 3
    ImuTelemetry.on_state(node, SimpleNamespace(connected=False))
    assert node.future is None and node.attempts == 0
    node.client.remove_pending_request.assert_called_once()
    ImuTelemetry.tick(node)
    node.client.call_async.assert_not_called()
    ImuTelemetry.on_state(node, SimpleNamespace(connected=True))
    assert node.connected and node.attempts == 0


def test_recent_high_rate_prevents_repeated_requests_but_stale_rate_does_not():
    node = subject()
    node.attempts = 1
    node.samples = deque(99 + i / 30 for i in range(30))
    with patch('rover_navigation.imu_telemetry_node.time.monotonic', return_value=100):
        ImuTelemetry.tick(node)
    node.client.call_async.assert_not_called()
    with patch('rover_navigation.imu_telemetry_node.time.monotonic', return_value=104):
        ImuTelemetry.tick(node)
    node.client.call_async.assert_called_once()
