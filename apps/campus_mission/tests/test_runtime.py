import asyncio

import pytest

from campus_mission.runtime import Runtime


class Bridge:
    def __init__(self):
        self.calls = []
        self.nodes = []
        self.state = {'ready': True, 'pose': {'x': 0}, 'map_match': True,
                      'map_received': True, 'stationary': True,
                      'fcu': {'connected': True, 'armed': False, 'mode': 'HOLD'}}

    async def call(self, op, **args):
        self.calls.append(op)
        if op == 'runtime':
            return {'nodes': self.nodes}
        if op == 'status':
            return self.state
        if op == 'prepare_vehicle':
            self.state['fcu'].update(armed=True, mode='GUIDED')
        if op == 'disarm_vehicle':
            self.state['fcu'].update(armed=False, mode='HOLD')
        return self.state['fcu']


class Supervisor:
    started = 0

    async def status(self):
        return 'inactive'

    async def start(self):
        self.started += 1


class Manager:
    active = None
    runner = None

    async def cancel(self):
        pass


def test_boot_is_idle_prepare_is_explicit_and_repeat_is_idempotent():
    async def scenario():
        b, s = Bridge(), Supervisor()
        r = Runtime(b, Manager(), s)
        assert not b.calls and s.started == 0
        await r.prepare()
        first = r.task
        await r.prepare()
        assert r.task is first
        await r.task
        assert s.started == 1 and b.calls.count('prepare_vehicle') == 1
        assert r.state['phase'] == 'ready'
        await r.disarm()
        assert not b.state['fcu']['armed'] and r.state['phase'] == 'idle'
    asyncio.run(scenario())


@pytest.mark.parametrize('problem', ['map', 'ready', 'moving', 'sensor_duplicate', 'fcu_rejection'])
def test_failure_never_forces_arm_or_restarts_navigation(problem):
    async def scenario():
        b, s = Bridge(), Supervisor()
        r = Runtime(b, Manager(), s, wait_seconds=.01)
        if problem == 'map': b.state['map_match'] = False
        if problem == 'ready': b.state['ready'] = False
        if problem == 'moving': b.state['stationary'] = False
        if problem == 'sensor_duplicate': b.nodes = ['/camera/camera']
        if problem == 'fcu_rejection':
            original = b.call
            async def reject(op, **args):
                if op == 'prepare_vehicle': raise RuntimeError('FCU denied')
                return await original(op, **args)
            b.call = reject
        await r.prepare()
        await r.task
        assert r.state['phase'] == 'failed' and not b.state['fcu']['armed']
        assert s.started <= 1
        if problem == 'sensor_duplicate': assert s.started == 0
    asyncio.run(scenario())


def test_reuses_manual_navigation_and_rejects_active_goal():
    async def scenario():
        b, s, manager = Bridge(), Supervisor(), Manager()
        b.nodes = ['/bt_navigator', '/camera/camera']
        r = Runtime(b, manager, s)
        manager.active = {'id': 'mission'}
        with pytest.raises(ValueError): await r.prepare()
        manager.active = None
        await r.prepare()
        await r.task
        assert s.started == 0 and b.state['fcu']['armed']
    asyncio.run(scenario())


def test_shutdown_while_waiting_does_not_arm_or_resume():
    async def scenario():
        b = Bridge(); b.state['ready'] = False
        r = Runtime(b, Manager(), Supervisor())
        await r.prepare()
        await asyncio.sleep(.02)
        await r.close()
        assert not b.state['fcu']['armed'] and r.state['phase'] == 'canceled'
        restarted = Runtime(b, Manager(), Supervisor())
        assert restarted.task is None
    asyncio.run(scenario())


def test_cancel_waiting_and_retry_reuses_navigation_without_late_arm():
    async def scenario():
        b, supervisor = Bridge(), Supervisor()
        b.nodes = ['/bt_navigator']; b.state['ready'] = False
        r = Runtime(b, Manager(), supervisor)
        await r.prepare(); await asyncio.sleep(.02)
        request_id = r.request_id
        await r.cancel(); await r.cancel_task
        assert r.state['phase'] == 'canceled'
        assert 'prepare_vehicle' not in b.calls and supervisor.started == 0
        b.state['ready'] = True
        await r.prepare(); await r.task
        assert r.request_id != request_id and r.state['phase'] == 'ready'
        assert b.calls.count('prepare_vehicle') == 1 and supervisor.started == 0
    asyncio.run(scenario())


def test_latched_external_fault_is_visible_not_endless_wait():
    async def scenario():
        b = Bridge(); b.state['ready'] = False
        b.state['external_navigation'] = {'fresh': True, 'fault_latched': True,
            'automatic_recovery_pending': False, 'reason': 'odometry position discontinuity'}
        r = Runtime(b, Manager(), Supervisor(), wait_seconds=.01)
        await r.prepare(); await r.task
        assert r.state['phase'] == 'failed'
        assert 'disarm_vehicle' in b.calls and 'prepare_vehicle' not in b.calls
        assert any(c['id']=='map' and c['ok'] for c in r.snapshot()['checks'])
        assert any(c['id']=='external' and not c['ok'] for c in r.snapshot()['checks'])
    asyncio.run(scenario())


def test_cancel_confirmation_failure_blocks_retry_until_acknowledged():
    async def scenario():
        b = Bridge(); b.state['ready'] = False
        original = b.call; fail = [True]
        async def call(op, **kwargs):
            if op == 'cancel_prepare_vehicle' and fail[0]: raise RuntimeError('worker unavailable')
            return await original(op, **kwargs)
        b.call = call
        r = Runtime(b, Manager(), Supervisor())
        await r.prepare(); await asyncio.sleep(.02)
        await r.cancel(); await r.cancel_task
        assert r.state['phase'] == 'cancel_failed'
        with pytest.raises(ValueError): await r.prepare()
        fail[0] = False
        await r.cancel(); await r.cancel_task
        assert r.state['phase'] == 'canceled'
    asyncio.run(scenario())


def test_immediate_cancel_before_task_runs_and_cancel_after_ready_are_safe():
    async def scenario():
        b = Bridge(); r = Runtime(b, Manager(), Supervisor())
        await r.prepare(); await r.cancel(); await r.cancel_task
        assert r.state['phase'] == 'canceled' and 'prepare_vehicle' not in b.calls
        await r.prepare(); await r.task
        before=list(b.calls)
        await r.cancel()
        assert b.calls == before and b.state['fcu']['armed']
    asyncio.run(scenario())


def test_explicit_failure_recovery_disarms_recovers_then_arms_without_goal():
    async def scenario():
        b=Bridge(); b.nodes=['/bt_navigator']
        b.state.update(ready=False,reason='stop_latched: acknowledge_tracking_stop required')
        b.state['fcu'].update(armed=True,mode='HOLD')
        b.state['external_navigation']={'fresh':True,'fault_latched':True,'reason':'odometry position discontinuity'}
        original=b.call
        async def call(op, **args):
            if op=='recover_externalnav':
                assert not b.state['fcu']['armed']
                b.calls.append(op); b.state['external_navigation'].update(fault_latched=False,ready=True)
                return {'success':True}
            if op=='acknowledge_tracking_stop':
                assert not b.state['fcu']['armed']
                b.calls.append(op);b.state.update(ready=True,reason='ready')
                return {'success':True}
            return await original(op,**args)
        b.call=call;supervisor=Supervisor();r=Runtime(b,Manager(),supervisor)
        await r.prepare(recover=True);await r.task
        assert r.state['phase']=='ready'
        ops=[x for x in b.calls if x in ['disarm_vehicle','recover_externalnav','acknowledge_tracking_stop','prepare_vehicle']]
        assert ops==['disarm_vehicle','recover_externalnav','acknowledge_tracking_stop','prepare_vehicle']
        assert supervisor.started==0 and 'navigate' not in b.calls
    asyncio.run(scenario())


def test_repeated_fault_after_recovery_fails_without_reset_loop():
    async def scenario():
        b=Bridge();b.state['ready']=False
        b.state['external_navigation']={'fresh':True,'fault_latched':True,'reason':'odometry position discontinuity'}
        original=b.call
        async def call(op,**args):
            if op=='recover_externalnav':b.calls.append(op);return {'success':True}
            return await original(op,**args)
        b.call=call;r=Runtime(b,Manager(),Supervisor())
        await r.prepare(recover=True);await r.task
        assert r.state['phase']=='failed' and b.calls.count('recover_externalnav')==1
        assert 'prepare_vehicle' not in b.calls
    asyncio.run(scenario())


@pytest.mark.parametrize('cause', ['startup', 'tracking_stop', 'fault'])
def test_plain_prepare_selects_recovery_from_fresh_server_state(cause):
    async def scenario():
        b=Bridge();b.nodes=['/bt_navigator'];b.state.update(ready=False,stationary=False)
        b.state['fcu'].update(armed=True,mode='HOLD')
        if cause=='startup':
            b.state['external_navigation']={'fresh':True,'reason':'need connected disarmed startup'}
        elif cause=='tracking_stop':
            b.state['reason']='stop_latched: acknowledge_tracking_stop required after recovery'
        else:
            b.state['external_navigation']={'fresh':True,'fault_latched':True,'automatic_recovery_pending':False}
        original=b.call
        async def call(op,**args):
            result=await original(op,**args)
            if op=='disarm_vehicle':
                b.state.update(ready=True,stationary=True,reason='ready',external_navigation={'fresh':True,'ready':True})
            return result
        b.call=call;r=Runtime(b,Manager(),Supervisor())
        await r.prepare();await r.task
        assert r.state['phase']=='ready'
        assert b.calls.index('disarm_vehicle')<b.calls.index('prepare_vehicle')
        assert b.calls.count('disarm_vehicle')==1 and 'navigate' not in b.calls
    asyncio.run(scenario())


def test_already_prepared_request_is_noop_even_when_stationary_noise_returns():
    async def scenario():
        b=Bridge();s=Supervisor();r=Runtime(b,Manager(),s)
        b.state.update(stationary=False)
        b.state['fcu'].update(armed=True,mode='GUIDED')
        for _ in range(2):
            result=await r.prepare()
            assert result['phase']=='ready' and r.task is None
        assert b.calls==['status','status'] and s.started==0
        # A later real disarm makes the same button perform preparation again.
        b.state.update(stationary=True)
        b.state['fcu'].update(armed=False,mode='MANUAL')
        await r.prepare();await r.task
        assert b.calls.count('prepare_vehicle')==1
    asyncio.run(scenario())


def test_arm_failure_cancels_old_request_then_retries_without_launch_restart():
    async def scenario():
        b=Bridge();b.nodes=['/bt_navigator'];original=b.call;arms=[];cancels=[]
        async def call(op,**args):
            if op=='prepare_vehicle':
                arms.append(args['request_id'])
                if len(arms)==1:raise RuntimeError('GUIDED response timeout')
            if op=='cancel_prepare_vehicle':cancels.append(args['request_id'])
            return await original(op,**args)
        b.call=call;s=Supervisor();r=Runtime(b,Manager(),s)
        await r.prepare();await r.task
        assert r.state['phase']=='ready' and len(arms)==2 and arms[0]!=arms[1]
        assert cancels==[arms[0]] and s.started==0
    asyncio.run(scenario())


def test_cancel_during_automatic_retry_never_arms_later():
    async def scenario():
        b=Bridge();b.nodes=['/bt_navigator'];original=b.call;failed=asyncio.Event();arms=[]
        async def call(op,**args):
            if op=='prepare_vehicle':
                arms.append(args['request_id']);failed.set();raise RuntimeError('temporary failure')
            return await original(op,**args)
        b.call=call;r=Runtime(b,Manager(),Supervisor())
        await r.prepare();await failed.wait();await asyncio.sleep(.05)
        await r.cancel();await r.cancel_task;await asyncio.sleep(.05)
        assert r.state['phase']=='canceled' and len(arms)==1 and not b.state['fcu']['armed']
    asyncio.run(scenario())


def test_retry_requires_acknowledged_cancellation():
    async def scenario():
        b=Bridge();b.nodes=['/bt_navigator'];original=b.call;arms=[]
        async def call(op,**args):
            if op=='prepare_vehicle':arms.append(op);raise RuntimeError('timeout')
            if op=='cancel_prepare_vehicle':raise RuntimeError('cancel unavailable')
            return await original(op,**args)
        b.call=call;r=Runtime(b,Manager(),Supervisor())
        await r.prepare();await r.task
        assert r.state['phase']=='cancel_failed' and len(arms)==1
    asyncio.run(scenario())


def test_transient_inventory_failure_retries_only_inventory():
    async def scenario():
        b=Bridge();b.nodes=['/bt_navigator'];original=b.call;attempts=[]
        async def call(op,**args):
            if op=='runtime':
                attempts.append(op)
                if len(attempts)==1:raise RuntimeError('worker busy')
            return await original(op,**args)
        b.call=call;s=Supervisor();r=Runtime(b,Manager(),s)
        await r.prepare();await r.task
        assert r.state['phase']=='ready' and len(attempts)==2 and s.started==0
    asyncio.run(scenario())


def test_persistent_arm_rejection_stops_after_three_cleaned_attempts():
    async def scenario():
        b=Bridge();b.nodes=['/bt_navigator'];original=b.call;arms=[];cancels=[]
        async def call(op,**args):
            if op=='prepare_vehicle':arms.append(args['request_id']);raise RuntimeError('FCU denied')
            if op=='cancel_prepare_vehicle':cancels.append(args['request_id'])
            return await original(op,**args)
        b.call=call;r=Runtime(b,Manager(),Supervisor())
        await r.prepare();await r.task
        assert r.state['phase']=='failed' and len(set(arms))==3
        assert cancels==arms and not b.state['fcu']['armed']
    asyncio.run(scenario())


@pytest.mark.parametrize('stage', ['waiting', 'arming'])
def test_external_vehicle_change_stops_preparation_without_undo_or_retry(stage):
    from campus_mission.vehicle_authority import OVERRIDE
    async def scenario():
        b = Bridge()
        original = b.call
        status_calls = 0
        async def manual_change(op, **args):
            nonlocal status_calls
            if op == 'status':
                status_calls += 1
                if stage == 'waiting' and status_calls == 2:
                    b.state['control_revision'] = 1
                    b.state['fcu'].update(armed=True, mode='MANUAL')
            if op == 'prepare_vehicle' and stage == 'arming':
                b.calls.append(op)
                b.state['control_revision'] = 1
                b.state['fcu'].update(armed=False, mode='MANUAL')
                raise RuntimeError(OVERRIDE)
            return await original(op, **args)
        b.call = manual_change
        r = Runtime(b, Manager(), Supervisor())
        await r.prepare(); await r.task
        assert r.state['phase'] == 'canceled'
        assert b.state['fcu']['mode'] == 'MANUAL'
        assert 'cancel_prepare_vehicle' not in b.calls
        assert 'disarm_vehicle' not in b.calls
        assert b.calls.count('prepare_vehicle') == (stage == 'arming')
        b.call = original  # Only a NEW user request can prepare again.
        await r.prepare(); await r.task
        assert r.state['phase'] == 'ready'
    asyncio.run(scenario())
