import asyncio
import base64
import copy
import json
import math
from pathlib import Path
import sqlite3
import time
import uuid

from .catalog import heading_candidates, path_score

TERMINAL = {'succeeded','failed','canceled','interrupted','analysis_failed'}


class Manager:
    def __init__(self, catalog, bridge, ai, data_dir):
        self.catalog, self.bridge, self.ai = catalog, bridge, ai
        self.root = Path(data_dir)
        self.root.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.root/'missions.sqlite3')
        self.db.execute('CREATE TABLE IF NOT EXISTS missions (id TEXT PRIMARY KEY, request_key TEXT UNIQUE, body TEXT)')
        self.active = None
        self.runner = None
        self.lock = asyncio.Lock()
        self.require_armed = False
        self.stop = asyncio.Event()
        self.pause_requested = False
        self.places = []
        self.resume_event = asyncio.Event()
        self.resume_event.set()
        self.pause_task = None
        # No automatic restart/movement after a process crash.
        for row in self.db.execute('SELECT id, body FROM missions').fetchall():
            mission = json.loads(row[1])
            if mission['state'] not in TERMINAL:
                mission.update(state='interrupted', message='임무 서버가 재시작되었습니다. 자동 재개하지 않습니다.')
                self.persist(mission)

    def persist(self, mission):
        mission['updated_at'] = time.time()
        self.db.execute('INSERT INTO missions VALUES (?, ?, ?) ON CONFLICT(id) DO UPDATE SET body=excluded.body',
                        (mission['id'], mission['request_key'], json.dumps(mission,ensure_ascii=False)))
        self.db.commit()

    def history(self):
        return [json.loads(row[0]) for row in self.db.execute('SELECT body FROM missions ORDER BY rowid DESC LIMIT 40')]

    def update(self, mission, state, message, **extra):
        mission.update(state=state, message=message, **extra)
        mission['events'].append({'at':time.time(),'state':state,'message':message})
        self.persist(mission)

    def ensure_available(self, resuming=False):
        if getattr(self, 'preparation_in_progress', lambda: False)():
            raise ValueError('주행 준비 또는 준비 취소 확인이 끝난 뒤 임무를 시작하세요.')
        if self.active and not resuming:
            raise ValueError('현재 임무를 완료하거나 취소한 뒤 새 임무를 시작하세요.')
        if self.bridge.mode != 'live':
            raise ValueError('화면 미리보기 모드입니다. 실제 이동은 live 모드에서 실행하세요.')
        state = self.bridge.state
        if self.require_armed:
            fcu = state.get('fcu') or {}
            if not fcu.get('connected') or not fcu.get('armed') or fcu.get('mode') != 'GUIDED':
                raise ValueError('먼저 주행 준비·시동 버튼으로 GUIDED·시동 상태를 확인하세요.')
        if not state.get('connected') or not state.get('ready') or not state.get('pose'):
            raise ValueError('주행 준비 상태가 아닙니다. 기존 navigation을 먼저 확인하세요.')
        if not state.get('map_match'):
            raise ValueError('실행 중인 ROS 지도와 장소 지도가 일치하지 않습니다.')
        if state.get('external_goal'):
            raise ValueError('다른 클라이언트의 Goal이 실행 중입니다. 먼저 취소하세요.')
        if not resuming and state.get('navigation') in ('accepting','executing','canceling'):
            raise ValueError('이전 Nav2 목표가 아직 종료되지 않았습니다. 취소 상태를 확인하세요.')

    async def start(self, request):
        async with self.lock:
            key = request['request_key']
            row = self.db.execute('SELECT body FROM missions WHERE request_key=?', (key,)).fetchone()
            signature = {k:request.get(k) for k in ('place_id','task','question','category','nearest')}
            if row:
                previous = json.loads(row[0])
                if previous['request'] != signature:
                    raise ValueError('같은 요청 ID에 다른 임무를 사용할 수 없습니다.')
                return previous
            self.ensure_available()
            task = request['task']
            if task not in ('navigate','photograph','inspect'):
                raise ValueError('지원하지 않는 임무입니다.')
            if task == 'inspect' and not self.ai.enabled:
                raise ValueError('현장 분석에는 API 키가 필요합니다. 사진 촬영은 키 없이 가능합니다.')
            if request.get('nearest'):
                candidates = [p for p in self.catalog.places.values() if p.category == request.get('category') and task in p.tasks]
            else:
                p = self.catalog.places.get(request.get('place_id'))
                candidates = [p] if p and task in p.tasks else []
            if not candidates:
                raise ValueError('현재 지도에 해당 임무를 지원하는 장소가 없습니다.')
            # Immutable copy: later edits cannot move an in-flight goal.
            mission = {'id':uuid.uuid4().hex,'request_key':key,'request':signature,
                       'task':task,'state':'planning','message':'도착 자세와 경로를 확인하고 있습니다.',
                       'map_id':self.catalog.map_id,'map_fingerprint':self.catalog.fingerprint,
                       'created_at':time.time(),'events':[],'photos':[], 'question':request.get('question',''),
                       'heading_policy':'flexible' if task=='navigate' else 'observation'}
            self.active = mission
            self.stop.clear()
            self.pause_requested = False
            self.resume_event.set()
            self.places = [p.model_copy(deep=True) for p in candidates]
            self.persist(mission)
            self.runner = asyncio.create_task(self.run(mission, self.places))
            await asyncio.sleep(0)
            return copy.deepcopy(mission)

    async def choose(self, mission, places):
        start = self.bridge.state['pose']
        # Limit expensive queries to the nearest three registered candidates.
        places.sort(key=lambda p: math.hypot(p.observation_pose.x-start['x'],p.observation_pose.y-start['y']))
        best = None
        for place in places[:3]:
            goal = place.observation_pose
            headings = heading_candidates(start,goal) if mission['task']=='navigate' else [goal.yaw]
            for angle in headings:
                if self.stop.is_set():
                    raise asyncio.CancelledError()
                pose = {'x':goal.x,'y':goal.y,'yaw':angle}
                try:
                    path = await self.bridge.call('plan', pose=pose)
                except RuntimeError:
                    continue
                if not path:
                    continue
                candidate = (path_score(path), place, pose)
                if best is None or candidate[0] < best[0]:
                    best = candidate
        if best is None:
            raise RuntimeError('후보 자세들에서 유효한 경로를 찾지 못했습니다. 목적지 위치와 주변 공간을 확인하세요.')
        return best[1:]

    async def run(self, mission, places):
        sent = False
        try:
            place, pose = await self.choose(mission, places)
            await self.resume_event.wait()
            if self.stop.is_set():
                raise asyncio.CancelledError()
            mission.update(place_id=place.id,place_name=place.name,goal=pose)
            # One Nav2 action survives pause/resume for the whole mission.
            mission['nav_goal_id'] = uuid.uuid4().hex
            departure_deadline = time.monotonic()+120
            while True:
                await self.resume_event.wait()
                if self.stop.is_set():
                    raise asyncio.CancelledError()
                state = await self.bridge.call('status', timeout=10)
                ext = state.get('external_navigation') or {}
                if ext.get('fresh') and ext.get('fault_latched'):
                    raise RuntimeError('위치·속도 전송 오류가 남아 있습니다. 오류 복구·다시 준비 후 목적지를 선택하세요.')
                if not state.get('map_match') or state.get('external_goal'):
                    raise RuntimeError('지도 일치 또는 다른 Goal 실행 상태를 확인하세요.')
                if self.require_armed:
                    f = state.get('fcu') or {}
                    if not f.get('connected') or not f.get('armed') or f.get('mode') != 'GUIDED':
                        raise RuntimeError('GUIDED·시동 상태가 해제됐습니다. 다시 주행 준비를 진행하세요.')
                if state.get('ready') and state.get('pose'):
                    sent = True  # Unknown RPC outcome still requires cancellation.
                    result = await self.bridge.call('navigate', goal_id=mission['nav_goal_id'], pose=pose,
                                                   observation=mission['task']!='navigate', timeout=25)
                    if not (isinstance(result, dict) and result.get('accepted') is False and result.get('retryable')):
                        break
                    sent = False  # Worker explicitly confirmed no goal submission.
                if time.monotonic() >= departure_deadline:
                    raise RuntimeError('출발 전 주행 준비가 복구되지 않았습니다: '+state.get('reason','상태 수신 대기'))
                message = '경로 확인 완료 · 출발 준비 복구를 기다립니다: '+state.get('reason','상태 수신 대기')
                if mission['message'] != message:
                    self.update(mission, 'planning', message)
                await asyncio.sleep(.3)
            self.update(mission,'navigating',place.name+'으로 이동 중입니다.')
            while True:
                await self.resume_event.wait()
                if self.stop.is_set():
                    raise asyncio.CancelledError()
                state = await self.bridge.call('status', timeout=10)
                await self.resume_event.wait()
                if state.get('goal_id') != mission['nav_goal_id']:
                    raise RuntimeError('현재 Nav2 Goal이 임무와 다릅니다.')
                nav = state['navigation']
                if nav == 'succeeded':
                    break
                if nav in ('failed','rejected','canceled'):
                    reason = state.get('reason', '')
                    ext = state.get('external_navigation') or {}
                    if ext.get('fresh') and ext.get('fault_latched'):
                        detail = ('추정 위치 불연속으로 위치·속도 전송 중단' if ext.get('coordinate_fault') or 'discontinuity' in ext.get('reason', '')
                                  else '위치·속도 전송 오류: '+ext.get('reason', ''))
                        raise RuntimeError(detail+' → Goal 종료. 오류 복구·다시 준비 후 목적지를 다시 선택하세요.')
                    raise RuntimeError('Nav2 이동 종료: '+nav+(' · '+reason if reason else ''))
                await asyncio.sleep(.5)
            sent = False
            if mission['task'] == 'navigate':
                self.update(mission,'succeeded','목적지에 도착했습니다. 다음 요청을 기다립니다.')
                return
            actual = state.get('pose')
            delta = abs(math.atan2(math.sin(actual['yaw']-pose['yaw']),math.cos(actual['yaw']-pose['yaw']))) if actual else 99.
            if not actual or math.hypot(actual['x']-pose['x'],actual['y']-pose['y']) > .65 or delta > .35:
                raise RuntimeError('Nav2는 도착 처리했지만 관찰 자세 오차가 큽니다. 사진 분석을 실행하지 않았습니다.')
            self.update(mission,'capturing','정지 후 새 카메라 사진을 확보하고 있습니다.')
            photos = []
            after = time.monotonic()
            for index in range(2):
                deadline = time.monotonic()+15
                while True:
                    if self.stop.is_set():
                        raise asyncio.CancelledError()
                    try:
                        photo = await self.bridge.call('capture', after=after, timeout=8)
                        break
                    except RuntimeError:
                        if time.monotonic() > deadline:
                            raise
                        await asyncio.sleep(.3)
                name = f'{mission["id"]}_{index}.jpg'
                (self.root/name).write_bytes(base64.b64decode(photo['jpeg'], validate=True))
                mission['photos'].append({'file':name,'stamp':photo['stamp'],'pose':photo['pose']})
                photos.append(photo)
                self.persist(mission)
                await asyncio.sleep(.6)
                after = time.monotonic()
            if mission['task'] == 'photograph':
                self.update(mission,'succeeded','도착 후 사진을 촬영했습니다.')
                return
            self.update(mission,'analyzing','촬영한 사진의 현장 상태를 분석하고 있습니다.')
            try:
                result = await self.ai.analyze(mission['question'] or '현재 보이는 상태를 설명해 줘.',place.name,photos)
            except Exception as exc:
                self.update(mission,'analysis_failed',str(exc))
                return
            if self.stop.is_set():
                raise asyncio.CancelledError()
            self.update(mission,'succeeded','관찰 결과를 확인하세요.', result=result)
        except asyncio.CancelledError:
            self.update(mission,'pausing' if self.pause_requested else 'canceling','이동 정지 요청을 확인하고 있습니다.')
            try:
                await self.bridge.call('stop_motion', goal_id=mission.get('nav_goal_id'), timeout=6)
            except Exception:
                # Still cancel Nav2 if the dedicated zero-output service is unavailable.
                if sent:
                    mission['cancel_unconfirmed'] = True
            if sent:
                await self.cancel_navigation(mission)
            if mission.get('cancel_unconfirmed'):
                self.update(mission,'interrupted','정지 요청을 보냈지만 종료 또는 차량 정지를 확인하지 못했습니다. 차량 상태를 확인하세요.')
            else:
                self.update(mission,'canceled','임무가 취소되었습니다.')
        except Exception as exc:
            if sent:
                await self.cancel_navigation(mission)
            self.update(mission,'failed',str(exc))
        finally:
            if self.pause_task and not self.pause_task.done():
                self.pause_task.cancel()
            self.active = None

    async def cancel_navigation(self, mission):
        try:
            target = mission.get('nav_goal_id', mission['id'])
            await self.bridge.call('cancel', goal_id=target, timeout=12)
            for _ in range(20):
                s = await self.bridge.call('status', timeout=5)
                if s.get('goal_id') != target or s.get('navigation') not in ('accepting','executing','canceling'):
                    return
                await asyncio.sleep(.25)
            mission['cancel_unconfirmed'] = True
            self.persist(mission)
        except Exception:
            mission['cancel_unconfirmed'] = True
            self.persist(mission)

    async def pause(self):
        async with self.lock:
            if not self.active or self.active['state'] not in ('planning','navigating','pausing','paused'):
                raise ValueError('경로 준비 또는 이동 중인 임무만 일시정지할 수 있습니다.')
            if not self.pause_requested:
                self.pause_requested = True
                self.resume_event.clear()
                self.update(self.active,'pausing','현재 Goal을 유지하며 주행 출력을 정지합니다.')
                self.pause_task = asyncio.create_task(self.confirm_pause(self.active))
            return {'message':'일시정지를 요청했습니다. 정지 완료 상태를 확인하세요.'}

    async def confirm_pause(self, mission):
        try:
            await self.bridge.call('stop_motion', goal_id=mission.get('nav_goal_id'), timeout=6)
            for _ in range(30):
                state = await self.bridge.call('status', timeout=3)
                if state.get('stationary'):
                    if self.active is mission and self.pause_requested:
                        self.update(mission,'paused','일시정지했습니다. 현재 Goal·시동·위치추정을 유지합니다.')
                    return
                await asyncio.sleep(.25)
            raise RuntimeError('차량 정지를 확인하지 못했습니다.')
        except asyncio.CancelledError:
            return
        except Exception as exc:
            if self.active is mission and self.pause_requested:
                mission['cancel_unconfirmed'] = True
                self.update(mission,'canceling',str(exc)+' 임무 취소를 요청합니다.')
                self.pause_requested = False
                self.stop.set()
                self.runner.cancel()

    async def resume(self):
        async with self.lock:
            if not self.active or self.active['state'] != 'paused':
                raise ValueError('정지가 확인된 일시정지 임무만 재개할 수 있습니다.')
            self.ensure_available(resuming=True)
            mission = self.active
            if mission.get('nav_goal_id'):
                state = await self.bridge.call('status', timeout=5)
                if state.get('goal_id') != mission['nav_goal_id'] or state.get('navigation') not in ('accepting','executing','succeeded'):
                    raise ValueError('기존 Nav2 Goal이 종료되었습니다. 임무 취소 후 새 목적지를 주세요.')
            await self.bridge.call('resume_motion', goal_id=mission.get('nav_goal_id'), timeout=6)
            self.pause_requested = False
            self.update(mission,'navigating' if mission.get('nav_goal_id') else 'planning','기존 임무를 재개합니다.')
            self.resume_event.set()
            return {'message':'기존 Goal로 임무를 재개합니다.'}

    async def cancel(self):
        async with self.lock:
            return await self._cancel()

    async def _cancel(self):
        self.pause_requested = False
        if self.active:
            if self.pause_task and not self.pause_task.done():
                self.pause_task.cancel()
            if not self.stop.is_set():
                self.stop.set()
                if self.runner:
                    if not self.runner.done():
                        self.runner.cancel()
            return {'message':'취소 요청을 전달했습니다. 완료 상태를 확인하세요.'}
        return {'message':'실행 중인 웹 임무가 없습니다.'}

    async def close(self):
        await self.cancel()
        if self.runner:
            try:
                await asyncio.wait_for(asyncio.shield(self.runner), 20)
            except (Exception, asyncio.CancelledError):
                pass
        await self.bridge.close()
        self.db.close()
