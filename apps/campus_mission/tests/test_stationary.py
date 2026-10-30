import math
import pytest
from campus_mission.stationary import StationaryDetector


def feed(detector, speed=lambda i:.02, x=lambda i:0., yaw=lambda i:0., rate=0.):
    for i in range(26):
        detector.update(10+i*.05, 100+i*.05, .02, speed(i), rate, x(i), 0., yaw(i))
    return detector.ready(11.25)


def test_isolated_lidar_noise_does_not_restart_standstill():
    d=StationaryDetector()
    assert feed(d, speed=lambda i:.1 if i%8==0 else .027,
                x=lambda i:.005*math.sin(i))
    assert not d.ready(12.3)  # Missing odometry never keeps stationary alive.


@pytest.mark.parametrize('case', ['drive','creep','spike','pose_jump','turn','invalid','stale'])
def test_motion_or_bad_input_is_not_treated_as_stationary(case):
    d=StationaryDetector()
    if case=='drive': assert not feed(d, speed=lambda i:.3)
    if case=='creep': assert not feed(d, speed=lambda i:.045)
    if case=='spike': assert not feed(d, speed=lambda i:.3 if i==25 else .01)
    if case=='pose_jump': assert not feed(d, x=lambda i:.08 if i>18 else 0.)
    if case=='turn': assert not feed(d, yaw=lambda i:i*.01, rate=.2)
    if case in ('invalid','stale'):
        assert feed(d)
        d.update(11.3,101.3,4. if case=='stale' else .01,
                 float('nan') if case=='invalid' else 0.,0.,0.,0.,0.)
        assert not d.ready(11.3)


def test_repeated_stamp_and_one_good_sample_do_not_confirm_standstill():
    d=StationaryDetector()
    d.update(1.,10.,.01,0.,0.,0.,0.,0.)
    assert not d.ready(1.)
    for i in range(30):d.update(1+i*.05,10.,.01,0.,0.,0.,0.,0.)
    assert not d.ready(2.5)


def test_yaw_wrap_is_not_a_rotation_jump():
    assert feed(StationaryDetector(), yaw=lambda i:math.pi-.001 if i%2 else -math.pi+.001)
