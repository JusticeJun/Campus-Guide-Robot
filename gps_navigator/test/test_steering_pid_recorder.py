import math
import struct

from mavros_msgs.msg import Mavlink

from gps_navigator.steering_pid_recorder_node import (
    CSV_FIELDS,
    STEERING_PID_AXIS,
    steering_pid_tuning,
)


def test_steering_pid_csv_contains_required_values():
    assert STEERING_PID_AXIS == 5
    assert {
        'monotonic_time_ns',
        'ros_time_ns',
        'message_time_ns',
        'desired_deg_s',
        'achieved_deg_s',
        'ff',
        'p',
        'i',
        'd',
    } == set(CSV_FIELDS)


def make_pid_tuning_message(axis):
    payload = struct.pack(
        '<6fB', 13.5, 4.0, 0.2, 0.1, 0.05, 0.01, axis
    )
    payload += b'\0' * (32 - len(payload))
    msg = Mavlink()
    msg.framing_status = Mavlink.FRAMING_OK
    msg.msgid = 194
    msg.len = 25
    msg.payload64 = list(struct.unpack('<4Q', payload))
    return msg


def test_only_steering_pid_tuning_is_selected():
    tuning = steering_pid_tuning(
        make_pid_tuning_message(STEERING_PID_AXIS)
    )

    assert math.isclose(tuning['desired'], 13.5, rel_tol=1e-6)
    assert math.isclose(tuning['achieved'], 4.0, rel_tol=1e-6)
    assert steering_pid_tuning(make_pid_tuning_message(4)) is None
