import pytest
from campus_mission.vehicle_authority import VehicleAuthority, OVERRIDE


def state(armed=False, mode='HOLD', connected=True):
    return dict(armed=armed, mode=mode, connected=connected)


def test_web_changes_do_not_revoke_preparation_but_manual_disarm_does():
    a = VehicleAuthority()
    a.observe(state())
    with a.expect(0, mode='GUIDED'):
        a.observe(state(mode='GUIDED'))
    with a.expect(0, armed=True):
        a.observe(state(True, 'GUIDED'))
    a.check(0)
    a.observe(state(False, 'GUIDED'))
    with pytest.raises(RuntimeError, match=OVERRIDE):
        a.check(0)
    a.check(a.revision)  # A NEW explicit request may start from current state.


@pytest.mark.parametrize('changed', [state(True), state(mode='MANUAL')])
def test_qgc_arming_or_mode_during_web_wait_invalidates_old_request(changed):
    a = VehicleAuthority(); a.observe(state())
    a.observe(changed)
    with pytest.raises(RuntimeError, match=OVERRIDE):
        with a.expect(0, mode='GUIDED'):
            pytest.fail('An old request must not send another command')


def test_mode_takeover_during_arm_response_and_reconnect_invalidate_request():
    a = VehicleAuthority(); a.observe(state(mode='GUIDED'))
    with a.expect(0, armed=True):
        a.observe(state(False, 'MANUAL'))
        a.observe(state(True, 'MANUAL'))
        with pytest.raises(RuntimeError, match=OVERRIDE): a.check(0)
    revision = a.revision
    a.observe(state(connected=False)); a.observe(state())
    with pytest.raises(RuntimeError, match=OVERRIDE): a.check(revision)
