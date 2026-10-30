import asyncio
import base64
import copy
import io
import json
import math
from pathlib import Path
import time
import uuid

from PIL import Image
import pytest
import yaml
from fastapi.testclient import TestClient

from campus_mission.catalog import Catalog,Place,Pose,path_score,heading_candidates
from campus_mission.manager import Manager
from campus_mission.app import create_app
from campus_mission.ai import AI


@pytest.fixture
def config(tmp_path):
    im=Image.new('L',(100,100),254)
    im.putpixel((2,2),0); im.putpixel((3,3),205)
    im.save(tmp_path/'map.pgm')
    (tmp_path/'rtabmap.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.1,
        'origin':[-5.,-5.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
    return {'mode':'live','token':'test-secret','map_yaml':str(tmp_path/'rtabmap.yaml'),
            'places_file':str(tmp_path/'places.yaml'),'data_dir':str(tmp_path/'data'),
            'image_topic':'/camera/test','odom_topic':'/odom'}


def catalog(config):
    c=Catalog(config['map_yaml'],config['places_file'])
    c.save(Place(id='nuri',name='누리관',category='building_entrance',
                 observation_pose=Pose(x=1,y=1,yaw=1.57)))
    return c


class FakeBridge:
    mode='live'
    def __init__(self):
        self.state={'connected':True,'ready':True,'map_match':True,'pose':{'x':0.,'y':0.,'yaw':0.},
                    'navigation':'idle','external_goal':False}
        self.calls=[]; self.block=None; self.reject=False; self.wrong_pose=False
    async def call(self,op,**kw):
        self.calls.append((op,kw))
        if op=='plan':
            if self.block:
                await self.block.wait()
            if self.reject:
                raise RuntimeError('no path')
            return [self.state['pose'],kw['pose']]
        if op=='navigate':
            self.state.update(goal_id=kw['goal_id'],navigation='succeeded',
                              pose=kw['pose'] if not self.wrong_pose else {'x':-4,'y':-4,'yaw':0})
        if op=='cancel':
            self.state['navigation']='canceled';return {'accepted':True}
        if op=='status':return copy.deepcopy(self.state)
        if op=='capture':
            out=io.BytesIO();Image.new('RGB',(32,32),'blue').save(out,format='JPEG')
            return {'jpeg':base64.b64encode(out.getvalue()).decode(),'stamp':time.time(),
                    'pose':self.state['pose'],'received_at':time.monotonic()}
        if op=='pose':return self.state['pose']
    async def close(self):pass


class FakeAI:
    enabled=True;llm='test'
    async def analyze(self,*args):
        return {'verdict':'uncertain','summary':'표지판이 보이지 않습니다.','evidence':[],
                'limitations':['이 사진으로 영업 여부를 알 수 없습니다.']}


def request(task='navigate',key=None):
    return {'task':task,'place_id':'nuri','question':'입구를 확인해 줘',
            'request_key':key or uuid.uuid4().hex,'category':'','nearest':False}


def test_catalog_hash_coordinate_binding_and_relocation(config,tmp_path):
    c=catalog(config)
    assert Catalog(config['map_yaml'],config['places_file']).places['nuri'].name=='누리관'
    assert c.world_to_pixel(Pose(x=-4.75,y=4.75,yaw=0)) == (2.,2.)
    for p in [Pose(x=-4.75,y=4.75,yaw=0),Pose(x=-4.65,y=4.65,yaw=0),Pose(x=20,y=1,yaw=0)]:
        with pytest.raises(ValueError):c.validate_pose(p)
    im=Image.open(tmp_path/'map.pgm');im.putpixel((40,40),0);im.save(tmp_path/'map.pgm')
    with pytest.raises(ValueError,match='다릅니다'):Catalog(config['map_yaml'],config['places_file'])


def test_empty_map_no_seed_leak(config):
    c=Catalog(config['map_yaml'],config['places_file'])
    assert c.places=={}


def test_heading_candidates_and_reverse_scoring():
    p=Pose(x=4,y=0,yaw=math.pi/2)
    angles=heading_candidates({'x':0,'y':0},p)
    assert angles[0]==0 and math.pi/2 in angles and len(angles)<=5
    forward=[{'x':0,'y':0,'yaw':0},{'x':1,'y':0,'yaw':0}]
    reverse=list(reversed(forward))
    assert path_score(forward)==1 and path_score(reverse)==3
    assert path_score(forward+reverse)==5


@pytest.mark.parametrize('task',['navigate','photograph','inspect'])
def test_mission_complete_and_heading_contract(config,task):
    async def scenario():
        bridge=FakeBridge();m=Manager(catalog(config),bridge,FakeAI(),config['data_dir'])
        result=await m.start(request(task));await m.runner
        final=m.history()[0]
        assert final['state']=='succeeded'
        nav=[kw for op,kw in bridge.calls if op=='navigate'][0]
        if task=='navigate':
            assert nav['observation'] is False
            assert len([op for op,_ in bridge.calls if op=='plan'])>=3
            assert not final['photos']
        else:
            assert nav['observation'] is True
            assert nav['pose']['yaw']==1.57
            assert len(final['photos'])==2
            assert final['photos'][0]['stamp'] >= result['created_at']
        if task=='inspect':assert final['result']['verdict']=='uncertain'
        await m.close()
    asyncio.run(scenario())


def test_idempotency_busy_and_cancellation_before_motion(config):
    async def scenario():
        b=FakeBridge();b.block=asyncio.Event();m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        req=request();first=await m.start(req);await asyncio.sleep(0)
        assert (await m.start(req))['id']==first['id']
        with pytest.raises(ValueError):await m.start(request())
        changed={**req,'task':'photograph'}
        with pytest.raises(ValueError):await m.start(changed)
        await m.cancel();await m.runner
        assert m.history()[0]['state']=='canceled'
        assert not any(op=='navigate' for op,_ in b.calls)
        await m.close()
    asyncio.run(scenario())


@pytest.mark.parametrize('problem',['map','ready','path','pose','analysis'])
def test_failures_do_not_claim_completion(config,problem):
    async def scenario():
        b=FakeBridge();ai=FakeAI();m=Manager(catalog(config),b,ai,config['data_dir'])
        if problem in ('map','ready'):
            b.state['map_match' if problem=='map' else 'ready']=False
            with pytest.raises(ValueError):await m.start(request())
        else:
            b.reject=problem=='path';b.wrong_pose=problem=='pose'
            if problem=='analysis':
                async def fail(*a):raise RuntimeError('provider unavailable')
                ai.analyze=fail
            await m.start(request('inspect'));await m.runner
            result=m.history()[0]
            assert result['state']==('analysis_failed' if problem=='analysis' else 'failed')
            if problem!='analysis':assert not any(op=='capture' for op,_ in b.calls)
            else:assert len(result['photos'])==2
        await m.close()
    asyncio.run(scenario())


def test_restart_marks_interrupted_without_replay(config):
    c=catalog(config);b=FakeBridge();m=Manager(c,b,FakeAI(),config['data_dir'])
    m.persist({'id':'a','request_key':'key','state':'navigating','events':[]})
    m.db.close()
    m=Manager(c,b,FakeAI(),config['data_dir'])
    assert m.history()[0]['state']=='interrupted' and not b.calls
    m.db.close()


def test_http_auth_map_place_validation_and_preview(config):
    catalog(config)
    config['mode']='preview'
    app=create_app(config)
    with TestClient(app) as client:
        assert client.get('/api/state').status_code==401
        for endpoint in ('vehicle/prepare','vehicle/cancel-prepare','vehicle/recover','vehicle/disarm','missions','cancel','pause','resume','requests','places'):
            assert client.post('/api/'+endpoint,json={}).status_code==401
        assert client.get('/').headers['referrer-policy']=='no-referrer'
        client.headers['Authorization']='Bearer test-secret'
        state=client.get('/api/state').json()
        assert state['map']['places'][0]['name']=='누리관'
        assert client.get('/api/map.png').headers['content-type']=='image/png'
        assert client.post('/api/missions',json=request()).status_code==400
        p=state['map']['places'][0];p['observation_pose']['x']=999
        assert client.post('/api/places',json=p).status_code==400
        assert client.get('/api/photos/wrong.jpg').status_code==404


def test_live_requires_control_link_key(config):
    config['token']=''
    with pytest.raises(ValueError):create_app(config)


def test_service_missions_require_verified_guided_and_armed(config):
    async def scenario():
        b = FakeBridge()
        m = Manager(catalog(config), b, FakeAI(), config['data_dir'])
        m.require_armed = True
        for fcu in (None, {'connected': True, 'armed': False, 'mode': 'GUIDED'},
                    {'connected': True, 'armed': True, 'mode': 'HOLD'}):
            b.state['fcu'] = fcu
            with pytest.raises(ValueError, match='주행 준비'):
                await m.start(request())
        assert not b.calls
        b.state['fcu'] = {'connected': True, 'armed': True, 'mode': 'GUIDED'}
        await m.start(request())
        await m.runner
        assert m.history()[0]['state'] == 'succeeded'
        await m.close()
    asyncio.run(scenario())


def test_ai_strict_tool_and_image_evidence(config):
    async def scenario():
        ai=AI();seen=[]
        async def fake(payload):
            seen.append(payload)
            if 'tools' in payload:
                return {'output':[{'type':'function_call','name':'propose_mission','arguments':json.dumps({
                    'task':'photograph','place_id':'nuri','category':'','nearest':False,
                    'question':'사진','reply':'누리관에서 촬영하겠습니다.'})}]}
            return {'output':[{'type':'message','content':[{'type':'output_text','text':json.dumps({
                'verdict':'uncertain','summary':'가림','evidence':[],'limitations':['안내문 가림']})}]}]}
        ai.request=fake
        intent=await ai.interpret('누리관 사진 찍어줘',list(catalog(config).places.values()))
        assert intent['place_id']=='nuri' and seen[0]['tools'][0]['strict']
        result=await ai.analyze('영업 여부','누리관',[{'jpeg':'abc','stamp':1}])
        assert result['verdict']=='uncertain'
        assert seen[-1]['input'][0]['content'][1]['image_url']=='data:image/jpeg;base64,abc'
    asyncio.run(scenario())


def test_unknown_model_place_rejected(config):
    async def scenario():
        ai=AI()
        async def fake(p):return {'output':[{'type':'function_call','name':'propose_mission','arguments':json.dumps({
            'task':'navigate','place_id':'fake_cafe','category':'','nearest':False,'question':'','reply':''})}]}
        ai.request=fake
        with pytest.raises(ValueError):await ai.interpret('카페',list(catalog(config).places.values()))
    asyncio.run(scenario())


def test_native_bt_default_checker_preserved():
    import xml.etree.ElementTree as ET
    root=Path(__file__).resolve().parents[3]/'rover_navigation/behavior_trees'
    for p in root.glob('*.xml'):
        for node in ET.parse(p).findall('.//FollowPath'):
            assert node.attrib['goal_checker_id']=='goal_checker'


def test_unconfirmed_cancel_is_not_reported_as_stopped(config):
    async def scenario():
        b = FakeBridge()
        original = b.call
        moving = asyncio.Event()
        async def unresponsive(op, **kw):
            if op == 'navigate':
                b.state.update(goal_id=kw['goal_id'], navigation='executing')
                moving.set()
                return
            if op == 'cancel':
                raise RuntimeError('ROS connection lost')
            return await original(op, **kw)
        b.call = unresponsive
        m = Manager(catalog(config), b, FakeAI(), config['data_dir'])
        await m.start(request())
        await moving.wait()
        await m.cancel()
        await m.cancel()  # Repeated clicks must not cancel cancellation cleanup.
        await m.runner
        result = m.history()[0]
        assert result['state'] == 'interrupted' and result['cancel_unconfirmed']
        with pytest.raises(ValueError, match='아직 종료'):
            await m.start(request())
        await m.close()
    asyncio.run(scenario())



@pytest.mark.parametrize('task',['navigate','photograph'])
def test_pause_resume_keeps_same_nav_goal_without_disarming(config,task):
    async def scenario():
        b=FakeBridge();original=b.call;moving=asyncio.Event()
        async def controlled(op,**kw):
            if op=='navigate':
                b.calls.append((op,kw))
                b.state.update(goal_id=kw['goal_id'],navigation='executing',stationary=False)
                moving.set();return
            if op=='stop_motion':b.state['stationary']=True
            return await original(op,**kw)
        b.call=controlled
        m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        await m.start(request(task));await moving.wait()
        ident=m.active['id'];nav_id=b.state['goal_id'];goal=copy.deepcopy(m.active['goal'])
        await m.pause();await m.pause();await m.pause_task
        assert m.active['state']=='paused' and m.active['id']==ident
        assert b.state['navigation']=='executing' and not m.runner.done()
        assert not any(op=='cancel' for op,_ in b.calls)
        with pytest.raises(ValueError):await m.start(request())
        b.state['ready']=False
        with pytest.raises(ValueError):await m.resume()
        assert m.active['state']=='paused'
        b.state['ready']=True
        await m.resume();await asyncio.sleep(.01)
        assert m.active['state']=='navigating'
        assert b.state['goal_id']==nav_id
        assert len([op for op,_ in b.calls if op=='navigate'])==1
        assert all(op not in ('prepare_vehicle','disarm_vehicle','cancel') for op,_ in b.calls)
        b.state.update(navigation='succeeded',pose=goal,stationary=True)
        await m.runner
        assert m.history()[0]['state']=='succeeded'
        assert len(m.history()[0]['photos'])==(2 if task=='photograph' else 0)
        await m.close()
    asyncio.run(scenario())


def test_cancel_during_pause_wins_and_new_goal_is_allowed(config):
    async def scenario():
        b=FakeBridge();original=b.call;moving=asyncio.Event();stopping=asyncio.Event();release=asyncio.Event()
        async def controlled(op,**kw):
            if op=='navigate':
                b.state.update(goal_id=kw['goal_id'],navigation='executing');moving.set();return
            if op=='stop_motion':
                stopping.set();await release.wait()
                b.state['stationary']=True
            return await original(op,**kw)
        b.call=controlled;m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        await m.start(request());await moving.wait();await m.pause();await stopping.wait()
        assert m.active['state']=='pausing'
        with pytest.raises(ValueError):await m.resume()
        await m.cancel();release.set();await m.runner
        assert m.active is None and m.history()[0]['state']=='canceled'
        b.call=original
        await m.start(request());await m.runner
        assert m.history()[0]['state']=='succeeded'
        await m.close()
    asyncio.run(scenario())


def test_pause_failure_never_enables_resume(config):
    async def scenario():
        b=FakeBridge();original=b.call;moving=asyncio.Event()
        async def controlled(op,**kw):
            if op=='navigate':
                b.state.update(goal_id=kw['goal_id'],navigation='executing');moving.set();return
            if op=='stop_motion':raise RuntimeError('connection lost')
            return await original(op,**kw)
        b.call=controlled;m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        await m.start(request());await moving.wait();await m.pause();await m.pause_task;await m.runner
        assert m.history()[0]['state']=='interrupted'
        with pytest.raises(ValueError):await m.resume()
        await m.close()
    asyncio.run(scenario())


def test_pause_planning_then_cancel_without_sending_goal(config):
    async def scenario():
        b=FakeBridge();b.block=asyncio.Event();b.state['stationary']=True
        m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        await m.start(request());await m.pause();await m.pause_task
        assert m.active['state']=='paused'
        await m.cancel();await m.runner
        assert not m.active and m.history()[0]['state']=='canceled'
        assert not any(op in ('navigate','cancel','disarm_vehicle') for op,_ in b.calls)
        await m.close()
    asyncio.run(scenario())


def test_no_mission_can_start_during_preparation_cancel(config):
    async def scenario():
        b=FakeBridge(); m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        m.preparation_in_progress=lambda: True
        with pytest.raises(ValueError,match='준비 취소'):
            await m.start(request())
        assert not any(op=='navigate' for op,_ in b.calls)
        m.preparation_in_progress=lambda: False
        await m.start(request()); await m.runner
        assert m.history()[0]['state']=='succeeded'
        await m.close()
    asyncio.run(scenario())


def test_transient_readiness_loss_after_planning_waits_before_departure(config):
    async def scenario():
        b=FakeBridge();original=b.call;checks=[]
        async def call(op,**kw):
            if op=='plan':
                result=await original(op,**kw);b.state['ready']=False;return result
            if op=='status' and b.state['navigation']=='idle':
                checks.append(op);b.state['ready']=len(checks)>=3
            if op=='navigate':assert b.state['ready']
            return await original(op,**kw)
        b.call=call;m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        await m.start(request());await m.runner
        result=m.history()[0]
        assert result['state']=='succeeded' and len(checks)>=3
        assert any('복구를 기다립니다' in e['message'] for e in result['events'])
        assert len([op for op,_ in b.calls if op=='navigate'])==1
        await m.close()
    asyncio.run(scenario())


def test_readiness_race_retries_only_explicitly_unsent_goal(config):
    async def scenario():
        b=FakeBridge();original=b.call;attempts=[]
        async def call(op,**kw):
            if op=='navigate':
                attempts.append(kw['goal_id'])
                if len(attempts)==1:return {'accepted':False,'retryable':True}
            return await original(op,**kw)
        b.call=call;m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        await m.start(request());await m.runner
        assert m.history()[0]['state']=='succeeded'
        assert len(attempts)==2 and len(set(attempts))==1
        assert len([op for op,_ in b.calls if op=='navigate'])==1
        await m.close()
    asyncio.run(scenario())


def test_cancel_while_waiting_departure_never_submits_goal(config):
    async def scenario():
        b=FakeBridge();original=b.call;waiting=asyncio.Event()
        async def call(op,**kw):
            result=await original(op,**kw)
            if op=='plan':b.state['ready']=False
            if op=='status':waiting.set()
            return result
        b.call=call;m=Manager(catalog(config),b,FakeAI(),config['data_dir'])
        await m.start(request());await waiting.wait();await m.cancel();await m.runner
        assert m.history()[0]['state']=='canceled'
        assert not any(op=='navigate' for op,_ in b.calls)
        await m.close()
    asyncio.run(scenario())
