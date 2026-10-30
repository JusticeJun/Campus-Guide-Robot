import argparse
import asyncio
import base64
from contextlib import asynccontextmanager
import fcntl
import hmac
import os
from pathlib import Path
import re
import shutil
import time

from fastapi import FastAPI, Header, HTTPException, Depends
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

from .ai import AI
from .bridge import RosBridge, PreviewBridge
from .catalog import Catalog, Place, Pose
from .manager import Manager
from .runtime import Runtime

ROOT = Path(__file__).resolve().parents[1]


class MissionRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    request_key: str = Field(min_length=8, max_length=100)
    place_id: str = Field(default='', max_length=64)
    task: str = Field(pattern='^(navigate|photograph|inspect)$')
    question: str = Field(default='', max_length=2000)
    nearest: bool = False
    category: str = Field(default='', max_length=40)


class ChatRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    text: str = Field(min_length=1, max_length=2000)
    request_key: str = Field(min_length=8, max_length=100)


def create_app(config, bridge=None, ai=None):
    token = config.get('token','')
    if config['mode'] == 'live' and not token:
        raise ValueError('live 모드는 제어 링크 키(CAMPUS_WEB_TOKEN)가 필요합니다.')
    catalog = Catalog(config['map_yaml'],config['places_file'])
    if not catalog.path.exists():
        seed = ROOT/'config/map1.places.yaml'
        if seed.exists():
            try:
                example = Catalog(config['map_yaml'],seed)
                for p in example.places.values():
                    catalog.save(p)
            except ValueError:
                pass  # New maps always start empty, never inherit map1 coordinates.

    @asynccontextmanager
    async def lifespan(app):
        # Refuse a second web owner even if another HTTP port was selected.
        lock_path = Path(config['data_dir'])/'mission.lock'
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        with lock_path.open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            app.state.bridge = bridge or (RosBridge(config) if config['mode']=='live' else PreviewBridge())
            app.state.ai = ai or AI()
            app.state.manager = Manager(catalog,app.state.bridge,app.state.ai,config['data_dir'])
            app.state.manager.require_armed = bool(config.get('service_controls'))
            app.state.runtime = Runtime(app.state.bridge, app.state.manager) if config.get('service_controls') else None
            app.state.manager.preparation_in_progress = lambda: bool(app.state.runtime and
                app.state.runtime.state.get('phase') in ('starting', 'localizing', 'recovering', 'arming', 'canceling', 'cancel_failed'))
            try:
                yield
            finally:
                if app.state.runtime:
                    await app.state.runtime.close()
                await app.state.manager.close()

    app = FastAPI(title='PKNU Campus Rover', lifespan=lifespan, docs_url=None, redoc_url=None)

    async def access(authorization: str = Header(default='')):
        if token and not hmac.compare_digest(authorization, 'Bearer '+token):
            raise HTTPException(401,'전달받은 제어 링크로 접속해 주세요.')

    @app.middleware('http')
    async def privacy_headers(request, call_next):
        response = await call_next(request)
        response.headers['Referrer-Policy'] = 'no-referrer'
        response.headers['X-Robots-Tag'] = 'noindex, nofollow'
        return response

    @app.exception_handler(ValueError)
    async def invalid(_, exc):
        from fastapi.responses import JSONResponse
        return JSONResponse(status_code=400,content={'detail':str(exc)})

    @app.exception_handler(RuntimeError)
    async def runtime(_, exc):
        from fastapi.responses import JSONResponse
        return JSONResponse(status_code=409,content={'detail':str(exc)})

    @app.exception_handler(asyncio.TimeoutError)
    async def response_timeout(_, exc):
        from fastapi.responses import JSONResponse
        return JSONResponse(status_code=409, content={'detail':'차량 응답이 지연되고 있습니다. 현재 진행 상태를 확인해 주세요.'})

    @app.get('/api/state',dependencies=[Depends(access)])
    async def state():
        m = app.state.manager
        url_file = Path.home()/'.local/state/campus-rover/public-url'
        try:
            public_url = url_file.read_text().strip()
        except FileNotFoundError:
            public_url = None
        return {'mode':config['mode'],'public_url':public_url,'ai_enabled':app.state.ai.enabled,
                'model':app.state.ai.llm,'map':catalog.describe(),
                'rover':app.state.bridge.state,'active':m.active,'history':m.history(),
                'startup':app.state.runtime.snapshot() if app.state.runtime else None}

    @app.post('/api/vehicle/prepare',dependencies=[Depends(access)])
    async def prepare_vehicle():
        if not app.state.runtime:
            raise ValueError('서비스 모드에서 사용할 수 있습니다.')
        return await app.state.runtime.prepare()

    @app.post('/api/vehicle/recover',dependencies=[Depends(access)])
    async def recover_vehicle():
        if not app.state.runtime:
            raise ValueError('서비스 모드에서 사용할 수 있습니다.')
        return await app.state.runtime.prepare(recover=True)

    @app.post('/api/vehicle/cancel-prepare',dependencies=[Depends(access)])
    async def cancel_prepare():
        if not app.state.runtime:
            raise ValueError('서비스 모드에서 사용할 수 있습니다.')
        return await app.state.runtime.cancel()

    @app.post('/api/vehicle/disarm',dependencies=[Depends(access)])
    async def disarm_vehicle():
        if not app.state.runtime:
            raise ValueError('서비스 모드에서 사용할 수 있습니다.')
        return await app.state.runtime.disarm()

    @app.get('/api/map.png',dependencies=[Depends(access)])
    async def map_image():
        return Response(catalog.png, media_type='image/png',headers={'Cache-Control':'private, max-age=3600'})

    @app.get('/api/camera.jpg',dependencies=[Depends(access)])
    async def camera():
        p = await app.state.bridge.call('preview',timeout=5)
        return Response(base64.b64decode(p['jpeg']), media_type='image/jpeg',
                        headers={'Cache-Control':'no-store','X-Capture-Time':str(p['stamp'])})

    @app.post('/api/places',dependencies=[Depends(access)])
    async def save_place(place: Place):
        if app.state.manager.active:
            raise ValueError('임무 실행 중에는 장소를 변경할 수 없습니다.')
        catalog.save(place)
        return place.model_dump()

    @app.get('/api/current-pose',dependencies=[Depends(access)])
    async def current_pose():
        return await app.state.bridge.call('pose',timeout=6)

    @app.post('/api/missions',dependencies=[Depends(access)])
    async def start(request: MissionRequest):
        return await app.state.manager.start(request.model_dump())

    @app.post('/api/cancel',dependencies=[Depends(access)])
    async def cancel():
        return await app.state.manager.cancel()

    @app.post('/api/pause',dependencies=[Depends(access)])
    async def pause():
        return await app.state.manager.pause()

    @app.post('/api/resume',dependencies=[Depends(access)])
    async def resume():
        return await app.state.manager.resume()

    @app.get('/api/photos/{name}',dependencies=[Depends(access)])
    async def photo(name: str):
        if not re.fullmatch(r'[0-9a-f]{32}_[01]\.jpg',name):
            raise HTTPException(404)
        path = Path(config['data_dir'])/name
        if not path.is_file():
            raise HTTPException(404)
        return FileResponse(path,media_type='image/jpeg',headers={'Cache-Control':'private, max-age=3600'})

    @app.post('/api/requests',dependencies=[Depends(access)])
    async def chat(request: ChatRequest):
        # Retried HTTP request must not re-run interpretation or navigation.
        row = app.state.manager.db.execute('SELECT body FROM missions WHERE request_key=?',(request.request_key,)).fetchone()
        if row:
            import json
            return {'mission':json.loads(row[0]),'reply':'이미 접수한 임무 상태입니다.'}
        decision = await app.state.ai.interpret(request.text,list(catalog.places.values()))
        if decision['task']=='clarify':
            return {'reply':decision['reply']}
        if decision['task']=='cancel':
            return {'reply':(await app.state.manager.cancel())['message']}
        if decision['task']=='locate':
            candidates = [p for p in catalog.places.values() if
                          (p.category==decision['category'] if decision['nearest'] else p.id==decision['place_id'])]
            if not candidates:
                return {'reply':'현재 지도에 해당 장소가 등록되어 있지 않습니다.'}
            pose = app.state.bridge.state.get('pose')
            if decision['nearest'] and pose:
                candidates.sort(key=lambda p:(p.observation_pose.x-pose['x'])**2+(p.observation_pose.y-pose['y'])**2)
            return {'reply':('로버 기준 직선거리가 가까운 후보: ' if decision['nearest'] and pose else '등록 장소: ')
                           + ', '.join(p.name for p in candidates[:3])+'. 실제 이동 시 접근 가능한 경로를 비교합니다.',
                    'places':[p.id for p in candidates]}
        mission = MissionRequest(request_key=request.request_key,place_id=decision['place_id'],
                   task=decision['task'],question=decision['question'] or request.text,
                   nearest=decision['nearest'],category=decision['category'])
        return {'reply':decision['reply'],'mission':await app.state.manager.start(mission.model_dump())}

    # UI and API same origin; no permissive CORS or direct ROS bridge exposed.
    assets = ROOT/'web/dist'
    if assets.exists():
        app.mount('/', StaticFiles(directory=assets,html=True), name='web')
    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-dir',required=True)
    parser.add_argument('--mode',choices=['preview','live'],default='preview')
    parser.add_argument('--service-controls',action='store_true',help='웹에서 명시적으로 주행 준비·시동 요청 허용')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8080)
    parser.add_argument('--places-file')
    parser.add_argument('--data-dir',default=str(Path.home()/'ros2_ws/mission_data'))
    parser.add_argument('--image-transport', choices=('compressed', 'raw'), default='compressed')
    parser.add_argument('--image-topic',default='/camera/camera/color/image_raw')
    parser.add_argument('--odom-topic',default='/odom')
    args = parser.parse_args()
    config = vars(args)
    if args.service_controls and args.mode != 'live':
        parser.error('--service-controls는 live 모드에서만 사용할 수 있습니다.')
    config['map_yaml'] = str(Path(args.map_dir).expanduser()/'rtabmap.yaml')
    config['places_file'] = args.places_file or str(Path(args.map_dir).expanduser()/'places.yaml')
    config['data_dir'] = str(Path(args.data_dir).expanduser().resolve())
    config['token'] = os.environ.get('CAMPUS_WEB_TOKEN','')
    if args.host not in ('127.0.0.1','localhost','::1') and not config['token']:
        parser.error('LAN 공개 시 CAMPUS_WEB_TOKEN을 설정하세요.')
    import uvicorn
    uvicorn.run(create_app(config),host=args.host,port=args.port,workers=1,access_log=False)


if __name__ == '__main__':
    main()
