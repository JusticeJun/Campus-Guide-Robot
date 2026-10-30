"""Operator-triggered startup; boot and web reconnect never arm the rover."""
import asyncio
import time
import uuid
from .vehicle_authority import OVERRIDE


class SystemdNavigation:
    unit = 'campus-navigation.service'

    async def command(self, *args):
        process = await asyncio.create_subprocess_exec(
            'systemctl', '--user', *args, self.unit,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        try:
            out, err = await asyncio.wait_for(process.communicate(), 12)
        except asyncio.TimeoutError:
            process.kill()
            await process.wait()
            raise RuntimeError('서비스 관리자 응답이 지연됩니다.') from None
        return process.returncode, out.decode().strip(), err.decode().strip()

    async def status(self):
        _, out, _ = await self.command('is-active')
        return out

    async def start(self):
        code, _, error = await self.command('start', '--no-block')
        if code:
            raise RuntimeError('주행 서비스 시작 실패: '+error[:300])


class Runtime:
    def __init__(self, bridge, manager, supervisor=None, wait_seconds=900):
        self.bridge, self.manager = bridge, manager
        self.supervisor = supervisor or SystemdNavigation()
        self.wait_seconds = wait_seconds
        self.task = None
        self.cancel_task = None
        self.request_id = None
        self.control_revision = 0
        self.started_at = None
        self.started_monotonic = None
        self.state = {'phase': 'idle', 'message': '주행 준비를 누르면 센서·지도 준비 후 GUIDED 전환과 시동을 진행합니다.'}

    def update(self, phase, message):
        self.state = {**self.state, 'phase': phase, 'message': message,
                      'updated_at': time.time(), 'started_at': self.started_at}

    def snapshot(self):
        elapsed = time.monotonic()-self.started_monotonic if self.started_monotonic else 0
        return {**self.state, 'elapsed_seconds': round(elapsed),
                'checks': startup_checks(self.bridge.state)}

    def available(self):
        if self.manager.active or self.bridge.state.get('external_goal'):
            raise ValueError('실행 중인 Goal을 먼저 취소하세요.')

    async def retry_step(self, label, action):
        """Retry a failed preparation step, never the whole navigation launch."""
        for attempt in range(3):
            self.available()
            self.check_authority()
            try:
                return await action()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if OVERRIDE in str(exc) or attempt == 2:
                    raise
                self.update('recovering', f'{label} 응답 확인 중: {exc} · '
                            f'{attempt+1}회 복구 재시도 중입니다. 센서·지도는 유지됩니다.')
                await asyncio.sleep(1 + attempt)

    async def prepare(self, recover=False):
        if self.cancel_task and not self.cancel_task.done():
            raise ValueError('준비 취소 확인 중입니다. 완료 후 다시 시작하세요.')
        if self.state['phase'] == 'cancel_failed':
            raise ValueError('이전 시동 요청의 취소가 확인되지 않았습니다. 준비 취소를 다시 눌러 주세요.')
        if self.task and not self.task.done():
            return self.state
        self.available()
        s = await self.bridge.call('status', timeout=8)
        # Requests may overlap while awaiting fresh telemetry.
        if self.task and not self.task.done():
            return self.state
        self.available()
        fcu = s.get('fcu') or {}
        if (s.get('ready') and s.get('pose') and s.get('map_match')
                and fcu.get('connected') and fcu.get('armed')
                and fcu.get('mode') == 'GUIDED' and not needs_recovery(s)):
            self.update('ready', '이미 GUIDED·시동 완료 상태입니다. 목적지를 선택하세요.')
            return self.state
        self.control_revision = s.get('control_revision', 0)
        self.request_id = uuid.uuid4().hex
        self.started_at = time.time()
        self.started_monotonic = time.monotonic()
        self.state = {}
        self.update('starting', '센서·지도 실행 상태를 확인합니다.')
        self.task = asyncio.create_task(self.run(recover=recover))
        return self.state

    def check_authority(self, state=None):
        state = self.bridge.state if state is None else state
        if state.get('control_revision', 0) != self.control_revision:
            raise RuntimeError(OVERRIDE)

    async def run(self, recover=False):
        recovered = False
        arm_failures = 0
        try:
            if recover:
                self.update('recovering', '실패한 주행의 시동을 해제하고 센서 입력을 다시 확인합니다. 지도와 센서는 유지됩니다.')
                await self.retry_step('시동 해제', lambda: self.bridge.call('disarm_vehicle', request_id=self.request_id, control_revision=self.control_revision, timeout=75))
            inventory = await self.retry_step('실행 프로세스 확인', lambda: self.bridge.call('runtime', timeout=10))
            nodes = inventory['nodes']
            if any('mapping' in n or n.endswith('/rtabmapviz') for n in nodes):
                raise RuntimeError('매핑이 실행 중입니다. 매핑을 정상 종료한 뒤 주행을 준비하세요.')
            unit = await self.supervisor.status()
            if unit not in ('active', 'activating') and not any(n.endswith('/bt_navigator') for n in nodes):
                sensors = [n for n in nodes if any(k in n.lower() for k in
                           ('mavros', 'realsense', 'camera', 'rplidar', 'rgbd_odometry', 'rtabmap'))]
                if sensors:
                    raise RuntimeError('별도 센서 런치가 실행 중입니다. 서비스 첫 전환 전에 기존 센서 런치를 종료하세요: '
                                       + ', '.join(sensors[:4]))
                await self.retry_step('주행 서비스 시작', self.supervisor.start)
            self.update('localizing', '센서 수신·지도 정합을 기다립니다. 준비되면 GUIDED 전환과 시동을 진행합니다.')
            deadline = time.monotonic()+self.wait_seconds
            while time.monotonic() < deadline:
                s = await self.retry_step('차량 상태 수신', lambda: self.bridge.call('status', timeout=8))
                self.available()
                self.check_authority(s)
                if s.get('map_received') and not s.get('map_match'):
                    raise RuntimeError('현재 navigation의 지도와 웹 지도 설정이 다릅니다.')
                self.state['checks'] = startup_checks(s)
                self.update('recovering' if recover else 'localizing', startup_message(s))
                ext = s.get('external_navigation') or {}
                # Resolve from fresh server-side telemetry on EVERY request;
                # a stale browser must not select the wrong startup procedure.
                if not recover and needs_recovery(s):
                    recover = True
                    self.update('recovering', '이전 오류·시동 상태를 정리합니다. 시동 해제 후 위치 전송과 주행 준비를 다시 확인합니다.')
                    await self.retry_step('시동 해제', lambda: self.bridge.call('disarm_vehicle', request_id=self.request_id, control_revision=self.control_revision, timeout=75))
                    await asyncio.sleep(.3)
                    continue
                if ext.get('fresh') and ext.get('fault_latched') and not ext.get('automatic_recovery_pending'):
                    if not recover or recovered:
                        raise RuntimeError(startup_message(s) + ' 센서·지도는 유지됩니다. 오류 복구·다시 준비를 사용할 수 있습니다.')
                    result = await self.retry_step('위치·속도 전송 복구', lambda: self.bridge.call('recover_externalnav', request_id=self.request_id, control_revision=self.control_revision, timeout=15))
                    recovered = bool(result.get('success'))
                    if recovered:
                        self.update('recovering', '위치·속도 전송을 다시 연결했습니다. Pixhawk 준비와 정지 해제를 확인합니다.')
                    else:
                        self.update('recovering', '안정된 센서 입력을 기다립니다: ' + result.get('message', ''))
                    await asyncio.sleep(1)
                    continue
                if recover and 'acknowledge_tracking_stop required' in s.get('reason', '') and ext.get('fresh') and ext.get('ready'):
                    result = await self.retry_step('주행 정지 상태 해제', lambda: self.bridge.call('acknowledge_tracking_stop', request_id=self.request_id, control_revision=self.control_revision, timeout=15))
                    self.update('recovering', '정지 상태 해제 확인: ' + result.get('message', ''))
                    await asyncio.sleep(1)
                    continue
                fcu = s.get('fcu') or {}
                if s.get('ready') and s.get('pose') and s.get('map_match') and fcu.get('connected') and s.get('stationary'):
                    self.update('arming', '정지·주행 준비 확인 완료. GUIDED 모드와 시동 상태를 확인합니다.')
                    try:
                        await self.bridge.call('prepare_vehicle', request_id=self.request_id, control_revision=self.control_revision, timeout=60)
                    except asyncio.CancelledError:
                        raise
                    except Exception as exc:
                        if OVERRIDE in str(exc):
                            raise
                        self.check_authority()
                        # A timed-out command may still be executing. Confirm
                        # cancellation before issuing a new request identity.
                        try:
                            await self.bridge.call('cancel_prepare_vehicle', request_id=self.request_id, control_revision=self.control_revision, timeout=75)
                        except Exception as cancel_error:
                            self.update('cancel_failed', f'{exc} / 시동 요청 취소 미확인: {cancel_error}')
                            return
                        arm_failures += 1
                        if arm_failures >= 3 or time.monotonic() >= deadline:
                            self.update('failed', f'GUIDED·시동 확인 실패: {exc}. 미완료 요청은 정리했습니다. 원인을 확인한 뒤 다시 준비할 수 있습니다.')
                            return
                        self.request_id = uuid.uuid4().hex
                        self.update('recovering', f'GUIDED·시동 단계 복구 중: {exc}. 미완료 요청을 정리했으며 현재 상태부터 다시 확인합니다 ({arm_failures}/2).')
                        await asyncio.sleep(1)
                        continue
                    self.update('ready', 'GUIDED·시동 완료. 목적지를 다시 선택하세요. 이전 임무는 자동 재개하지 않습니다.' if recover else 'GUIDED·시동 완료. 목적지를 선택하세요. 목표를 주기 전에는 이동하지 않습니다.')
                    return
                await asyncio.sleep(1)
            raise RuntimeError('아직 주행 준비가 완료되지 않았습니다. 지도·센서 상태 확인 후 다시 준비를 누르세요. 런치는 유지됩니다.')
        except asyncio.CancelledError:
            pass  # cancel() owns the acknowledgement and final state.
        except Exception as exc:
            if OVERRIDE in str(exc) or self.bridge.state.get('control_revision', 0) != self.control_revision:
                self.update('canceled', OVERRIDE + ' 현재 시동·모드를 유지합니다. 필요할 때 다시 준비를 누르세요.')
                return
            if self.state.get('phase') == 'arming':
                try:
                    await self.bridge.call('cancel_prepare_vehicle', request_id=self.request_id, control_revision=self.control_revision, timeout=75)
                except Exception as cancel_error:
                    self.update('cancel_failed', f'{exc} / 시동 요청 취소 미확인: {cancel_error}')
                    return
            self.update('failed', str(exc))

    async def cancel(self):
        if self.cancel_task and not self.cancel_task.done():
            return self.state
        if not self.request_id or self.state.get('phase') in ('ready', 'idle', 'canceled'):
            return self.state
        self.available()
        self.update('canceling', '준비를 취소하고 미완료 시동 요청을 확인합니다. 센서와 지도는 유지됩니다.')
        if self.task and not self.task.done():
            self.task.cancel()
        self.cancel_task = asyncio.create_task(self._cancel())
        return self.state

    async def _cancel(self):
        if self.task and not self.task.done():
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        try:
            result = await self.bridge.call('cancel_prepare_vehicle', request_id=self.request_id, control_revision=self.control_revision, timeout=75)
            armed = (result or {}).get('armed')
            self.update('canceled', '준비를 취소했습니다. 센서·지도는 유지되어 다시 준비할 수 있습니다.'
                        + (' 기존 시동 상태는 유지됩니다.' if armed else ' 시동 꺼짐을 확인했습니다.' if armed is False else ' 차량 연결 상태를 확인하세요.'))
        except Exception as exc:
            self.update('cancel_failed', '준비 취소를 확인하지 못했습니다: ' + str(exc))

    async def disarm(self):
        if self.task and not self.task.done():
            await self.cancel()
        if self.cancel_task and not self.cancel_task.done():
            await self.cancel_task
        await self.manager.cancel()
        if self.manager.runner and not self.manager.runner.done():
            await asyncio.wait_for(asyncio.shield(self.manager.runner), 25)
        result = await self.bridge.call('disarm_vehicle', timeout=75)
        self.update('idle', '시동을 해제했습니다. 센서와 지도는 유지됩니다.')
        return result

    async def close(self):
        if self.task and not self.task.done():
            await self.cancel()
        if self.cancel_task and not self.cancel_task.done():
            await self.cancel_task


def startup_checks(s):
    fcu = s.get('fcu') or {}
    ext = s.get('external_navigation') or {}
    return [
        {'id': 'fcu', 'label': 'Pixhawk 연결', 'ok': bool(fcu.get('connected'))},
        {'id': 'camera', 'label': '카메라 영상', 'ok': s.get('camera_age') is not None and s['camera_age'] < 3},
        {'id': 'map', 'label': '지도 로딩', 'ok': bool(s.get('map_match'))},
        {'id': 'pose', 'label': '현재 위치 TF', 'ok': bool(s.get('pose'))},
        {'id': 'external', 'label': '위치·속도 전송', 'ok': bool(ext.get('fresh') and ext.get('ready'))},
        {'id': 'navigation', 'label': '주행 준비 판정', 'ok': bool(s.get('ready'))},
        {'id': 'guided', 'label': 'GUIDED 모드', 'ok': fcu.get('mode') == 'GUIDED'},
        {'id': 'armed', 'label': '시동 확인', 'ok': bool(fcu.get('armed'))},
    ]


def startup_message(s):
    ext = s.get('external_navigation') or {}
    reason = ext.get('reason', '')
    if ext.get('fresh') and ext.get('fault_latched'):
        translated = {'odometry position discontinuity': '추정 위치가 갑자기 바뀌어 위치·속도 전송이 중단됐습니다.',
                      'odometry input unavailable': '오도메트리 입력 중단 오류가 남아 있습니다.',
                      'odometry exceeds low-speed rover envelope': '추정 이동 속도가 허용 범위를 넘어 전송이 중단됐습니다.'}
        return translated.get(reason, '위치·속도 전송 오류: ' + reason)
    if not (s.get('fcu') or {}).get('connected'):
        return 'Pixhawk 연결을 기다립니다. 센서·주행 프로세스를 시작하는 중입니다.'
    if not s.get('map_received'):
        return '저장된 지도를 불러오고 있습니다. 지도 크기에 따라 시간이 걸릴 수 있습니다.'
    if not s.get('pose'):
        return '현재 위치 TF를 기다립니다. 카메라·오도메트리·지도 정합을 확인 중입니다.'
    if not s.get('ready'):
        detail = s.get('reason') or '주행 상태 수신 대기'
        if 'external_navigation_not_ready' in detail:
            return '현재 위치 TF 수신 중 · Pixhawk가 위치·속도 입력을 준비하는 중입니다.'
        return '주행 준비 판정 대기: ' + detail
    if not s.get('stationary'):
        return '지도·주행 준비 완료 · 차량 정지 확인을 기다립니다.'
    return '주행 준비 완료 · GUIDED 전환과 시동 확인을 진행합니다.'


def needs_recovery(s):
    ext = s.get('external_navigation') or {}
    return bool(
        'acknowledge_tracking_stop required' in s.get('reason', '')
        or (ext.get('fresh') and (
            (ext.get('fault_latched') and not ext.get('automatic_recovery_pending'))
            or ext.get('reason') == 'need connected disarmed startup'))
    )
