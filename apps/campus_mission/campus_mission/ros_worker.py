"""ROS actions and explicit operator startup, never direct velocity or PWM."""
import base64
import concurrent.futures
import hashlib
import io
import json
import math
from pathlib import Path
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from .stationary import StationaryDetector
from .vehicle_authority import VehicleAuthority


def run_worker(pipe, config):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as RosPath, Odometry
    from sensor_msgs.msg import Image as RosImage, CompressedImage
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool, Trigger
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from nav2_msgs.action import NavigateToPose, ComputePathToPose
    from rcl_interfaces.srv import GetParameters
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandBool, SetMode
    from unique_identifier_msgs.msg import UUID
    from tf2_ros import Buffer, TransformListener
    from PIL import Image
    from .catalog import Catalog

    send_lock = threading.Lock()
    running = threading.Event()
    running.set()

    def send(data):
        with send_lock:
            try:
                pipe.send(data)
            except (BrokenPipeError, EOFError, OSError):
                running.clear()

    def wait(future, timeout=120):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise RuntimeError('ROS 응답 대기시간 초과. 서버 상태를 확인하세요.')
        return future.result()

    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def pose_dict(p):
        return {'x': p.position.x, 'y': p.position.y, 'yaw': yaw(p.orientation)}

    class Worker(Node):
        def __init__(self):
            super().__init__('campus_mission_bridge')
            self.catalog = Catalog(config['map_yaml'], config['places_file'])
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
            self.goal = None
            self.goal_id = None
            self.nav_state = 'idle'
            self.distance = None
            self.active_ids = set()
            self.ready = False
            self.ready_at = 0.
            self.reason = '주행 상태 수신 대기'
            self.external_navigation = {}
            self.external_at = 0.
            self.canceled_preparations = set()
            self.arm_attempts = set()
            self.vehicle_authority = VehicleAuthority()
            self.preparation_revisions = {}
            self.preparation_lock = threading.Lock()
            self.fcu = None
            self.fcu_at = 0.
            self.map_match = False
            self.map_received = False
            self.path = []
            self.path_at = 0.
            self.raw_image = None
            self.image_at = 0.
            self.preview_cache = None
            self.preview_at = 0.
            self.standstill = StationaryDetector()
            self.motion_pause_type = SetBool
            self.motion_pause = self.create_client(SetBool, '/navigation/set_motion_paused')
            self.nav_lock = threading.Lock()
            self.vehicle_lock = threading.Lock()
            self.cancel_requested = False
            self.canceled_ids = set()
            self.paused_ids = set()
            transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, transient)
            self.create_subscription(Bool, '/navigation/ready', self.on_ready, 1)
            self.create_subscription(String, '/navigation/status', self.on_reason, transient)
            self.create_subscription(String, '/external_navigation/status', self.on_external, 1)
            self.create_subscription(State, '/mavros/state', self.on_fcu, 1)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_goals, transient)
            self.create_subscription(RosPath, '/plan', self.on_path, 1)
            self.create_subscription(Odometry, config['odom_topic'], self.on_odom, qos_profile_sensor_data)
            compressed = config.get('image_transport', 'raw') == 'compressed'
            self.create_subscription(CompressedImage if compressed else RosImage,
                                     config['image_topic'] + ('/compressed' if compressed else ''),
                                     self.on_image, qos_profile_sensor_data)
            self.create_timer(.5, lambda: send({'state': self.status()}))

        def on_map(self, msg):
            self.map_received = True
            m, c = msg.info, self.catalog
            self.map_match = (msg.header.frame_id == 'map' and m.width == c.width and m.height == c.height
                              and abs(m.resolution-c.resolution) < 1e-6
                              and abs(m.origin.position.x-c.origin[0]) < 1e-4
                              and abs(m.origin.position.y-c.origin[1]) < 1e-4
                              and abs(yaw(m.origin.orientation)-c.origin[2]) < 1e-4
                              and hashlib.sha256(msg.data.tobytes()).hexdigest() == c.grid_hash)

        def on_ready(self, msg):
            self.ready, self.ready_at = msg.data, time.monotonic()

        def on_reason(self, msg):
            self.reason = msg.data

        def on_external(self, msg):
            try:
                value = json.loads(msg.data)
                if isinstance(value, dict):
                    self.external_navigation, self.external_at = value, time.monotonic()
            except (ValueError, TypeError):
                pass

        def on_fcu(self, msg):
            self.fcu = {'armed': msg.armed, 'mode': msg.mode, 'connected': msg.connected}
            self.fcu_at = time.monotonic()
            self.vehicle_authority.observe(self.fcu)

        def on_goals(self, msg):
            self.active_ids = {bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list if s.status in (1, 2, 3)}

        def on_path(self, msg):
            if msg.header.frame_id == 'map':
                step = max(1, len(msg.poses)//600)
                self.path = [pose_dict(p.pose) for p in msg.poses[::step]]
                self.path_at = time.monotonic()

        def on_image(self, msg):
            self.raw_image, self.image_at = msg, time.monotonic()

        def on_odom(self, msg):
            now = time.monotonic()
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            age = self.get_clock().now().nanoseconds/1e9-stamp
            t = msg.twist.twist
            p = msg.pose.pose
            self.standstill.update(now, stamp, age, math.hypot(t.linear.x, t.linear.y),
                                  t.angular.z, p.position.x, p.position.y, yaw(p.orientation))

        def current_pose(self):
            t = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            age = self.get_clock().now().nanoseconds/1e9 - (t.header.stamp.sec+t.header.stamp.nanosec/1e9)
            if age > 3. or age < -1.:
                raise RuntimeError('현재 위치 TF가 오래되었습니다.')
            return {'x': t.transform.translation.x, 'y': t.transform.translation.y,
                    'yaw': yaw(t.transform.rotation)}

        def status(self):
            now = time.monotonic()
            try:
                pose = self.current_pose()
            except Exception:
                pose = None
            return {'connected': True, 'control_revision': self.vehicle_authority.revision, 'ready': self.ready and now-self.ready_at < 5.,
                    'map_match': self.map_match, 'map_received': self.map_received,
                    'pose': pose, 'reason': self.reason,
                    'stationary': self.standstill.ready(now),
                    'fcu': self.fcu if now-self.fcu_at < 5. else None,
                    'navigation': self.nav_state, 'goal_id': self.goal_id,
                    'external_goal': bool(self.active_ids-{self.goal_id}),
                    'distance_remaining': self.distance, 'path': self.path,
                    'path_age': now-self.path_at if self.path_at else None,
                    'external_navigation': {**self.external_navigation, 'fresh': bool(self.external_at and now-self.external_at < 5)},
                    'camera_age': now-self.image_at if self.image_at else None}

        def stamped(self, p):
            result = PoseStamped()
            result.header.frame_id = 'map'
            result.header.stamp = self.get_clock().now().to_msg()
            result.pose.position.x, result.pose.position.y = float(p['x']), float(p['y'])
            result.pose.orientation.z = math.sin(p['yaw']/2)
            result.pose.orientation.w = math.cos(p['yaw']/2)
            return result

        def check_start(self, s=None):
            s = self.status() if s is None else s
            if not s['map_match']:
                raise RuntimeError('실행 중인 /map과 선택한 장소 지도가 일치하지 않습니다.')
            if not s['ready'] or not s['pose']:
                raise RuntimeError('현재 주행 준비 상태가 아닙니다. 기존 navigation 상태를 확인하세요.')
            if s['external_goal']:
                raise RuntimeError('RViz 또는 다른 클라이언트의 Goal이 실행 중입니다.')

        def observation_tree(self):
            # Keep the actual loaded BT (including guard/replanning), change only
            # this mission's goal checker; never modify global tolerances in flight.
            def parameters(node, names):
                client = self.create_client(GetParameters, node+'/get_parameters')
                try:
                    if not client.wait_for_service(timeout_sec=3.):
                        raise RuntimeError('Nav2 파라미터 서비스가 없습니다.')
                    request = GetParameters.Request(names=names)
                    return wait(client.call_async(request), 8).values
                finally:
                    self.destroy_client(client)
            plugins = parameters('/controller_server', ['goal_checker_plugins'])[0].string_array_value
            if 'observation_goal_checker' not in plugins:
                raise RuntimeError('관찰용 Goal Checker를 적용하려면 navigation을 새 빌드로 재시작하세요.')
            source = parameters('/bt_navigator', ['default_nav_to_pose_bt_xml'])[0].string_value
            root = ET.parse(source)
            followers = root.findall('.//FollowPath')
            if not followers:
                raise RuntimeError('현재 행동트리에서 FollowPath를 찾지 못했습니다.')
            for node in followers:
                node.set('goal_checker_id', 'observation_goal_checker')
            path = Path(config['data_dir'])/'observation_bt.xml'
            root.write(path, encoding='utf-8')
            return str(path)

        def vehicle_service(self, kind, name, request, request_id=None):
            client = self.create_client(kind, name)
            try:
                if not client.wait_for_service(timeout_sec=3.):
                    raise RuntimeError('MAVROS 서비스가 없습니다: '+name)
                with self.vehicle_authority.lock:
                    self.check_preparation(request_id)
                    future = client.call_async(request)
                return wait(future, 8)
            finally:
                self.destroy_client(client)

        def wait_vehicle(self, predicate, message, seconds=8, request_id=None):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                self.check_preparation(request_id)
                s = self.status()
                if s.get('fcu') and s['fcu']['connected'] and predicate(s):
                    return s
                time.sleep(.1)
            raise RuntimeError(message)

        def control_vehicle(self, arm, request_id=None):
            if not config.get('service_controls'):
                raise RuntimeError('웹 차량 준비 기능이 활성화되지 않았습니다.')
            with self.vehicle_lock:
                if request_id:
                    self.preparation_revisions.setdefault(request_id, self.vehicle_authority.revision)
                self.check_preparation(request_id)
                s = self.status()
                if s['external_goal'] or s['navigation'] in ('accepting','executing','canceling'):
                    if arm:
                        raise RuntimeError('먼저 현재 Goal을 취소하세요.')
                    # Explicit operator disarm also stops a Goal left by a
                    # previous web process or RViz; boot never calls this.
                    self.vehicle_service(CancelGoal, '/navigate_to_pose/_action/cancel_goal', CancelGoal.Request())
                    self.wait_vehicle(lambda x:not x['external_goal'] and x['navigation'] not in
                                      ('accepting','executing','canceling'), 'Nav2 취소 완료를 확인하지 못했습니다.')
                if not s['fcu'] or not s['fcu']['connected']:
                    raise RuntimeError('Pixhawk 연결을 확인하세요.')
                if arm:
                    self.check_start()
                    if s['fcu']['armed'] and s['fcu']['mode'] == 'GUIDED':
                        return s['fcu']  # Already prepared; no new mode/arm request.
                    if not s['stationary']:
                        raise RuntimeError('차량 정지 확인을 기다려 주세요.')
                mode = 'GUIDED' if arm else 'HOLD'
                if s['fcu']['mode'] != mode:
                    self.check_preparation(request_id)
                    revision = self.preparation_revisions.get(request_id, self.vehicle_authority.revision)
                    with self.vehicle_authority.expect(revision, mode=mode):
                        response = self.vehicle_service(SetMode, '/mavros/set_mode', SetMode.Request(custom_mode=mode), request_id)
                        if not response.mode_sent:
                            raise RuntimeError(mode+' 모드 요청이 거절됐습니다. FCU 상태를 확인하세요.')
                        self.wait_vehicle(lambda x:x['fcu']['mode']==mode, mode+' 전환이 확인되지 않았습니다.', request_id=request_id)
                current = self.status()['fcu']
                if not arm and current and not current['armed']:
                    return current
                # HOLD acknowledgement precedes disarm. Bad odometry must not
                # prevent removing propulsion power during fault recovery.
                if arm:
                    self.wait_vehicle(lambda x:x['stationary'], '정지 확인이 되지 않아 시동 상태를 바꾸지 않았습니다.', request_id=request_id)
                    self.check_start()
                    if not running.is_set():
                        raise RuntimeError('웹 연결이 종료되어 시동 요청을 중단했습니다.')
                current = self.status()['fcu']
                if current and current['armed'] == arm:
                    return current
                # Cancellation and dispatch share a lock: a canceled queued
                # request cannot arm later. A sent arm is rolled back by cancel.
                revision = self.preparation_revisions.get(request_id, self.vehicle_authority.revision)
                with self.vehicle_authority.expect(revision, armed=arm):
                    with self.preparation_lock:
                        self.check_preparation(request_id)
                        if arm and request_id:
                            self.arm_attempts.add(request_id)
                        response = self.vehicle_service(CommandBool, '/mavros/cmd/arming', CommandBool.Request(value=arm), request_id)
                    if not response.success:
                        raise RuntimeError('FCU가 시동 요청을 거절했습니다 (result='+str(response.result)+'). PreArm 메시지를 확인하세요.')
                    return self.wait_vehicle(lambda x:x['fcu']['armed']==arm and x['fcu']['mode']==mode,
                                             'FCU 시동 상태 변경을 확인하지 못했습니다.', request_id=request_id)['fcu']

        def check_preparation(self, request_id):
            if request_id in self.preparation_revisions:
                self.vehicle_authority.check(self.preparation_revisions[request_id])
            if request_id in self.canceled_preparations:
                raise RuntimeError('사용자가 주행 준비를 취소했습니다.')

        def cancel_preparation(self, request_id):
            with self.preparation_lock:
                self.canceled_preparations.add(request_id)
            # Barrier against an already running/queued arming RPC. Only undo
            # an arm attempted by this request; preserve a preexisting arm.
            with self.vehicle_lock:
                undo_arm = (request_id in self.arm_attempts and
                            self.preparation_revisions.get(request_id, self.vehicle_authority.revision)
                            == self.vehicle_authority.revision)
            if undo_arm:
                rollback_id = uuid.uuid4().hex
                self.preparation_revisions[rollback_id] = self.preparation_revisions[request_id]
                result = self.control_vehicle(False, rollback_id)
                self.arm_attempts.discard(request_id)
                return result
            return self.status().get('fcu') or {}

        def execute(self, op, args):
            if args.get('request_id') and 'control_revision' in args and op != 'cancel_prepare_vehicle':
                self.preparation_revisions.setdefault(args['request_id'], args['control_revision'])
                self.check_preparation(args['request_id'])
            if op == 'runtime':
                return {'nodes':[ns.rstrip('/')+'/'+name for name, ns in self.get_node_names_and_namespaces()]}
            if op == 'prepare_vehicle':
                return self.control_vehicle(True, args.get('request_id'))
            if op == 'cancel_prepare_vehicle':
                return self.cancel_preparation(args['request_id'])
            if op == 'disarm_vehicle':
                return self.control_vehicle(False, args.get('request_id'))
            if op in ('recover_externalnav', 'acknowledge_tracking_stop'):
                with self.vehicle_lock:
                    self.check_preparation(args.get('request_id'))
                    s = self.status()
                    if (not s.get('fcu') or not s['fcu']['connected'] or s['fcu']['armed']
                            or s['external_goal'] or s['navigation'] in ('accepting', 'executing', 'canceling')):
                        raise RuntimeError('오류 복구는 Goal 종료·시동 해제 후 가능합니다.')
                    if not s['stationary'] or not s['pose'] or not s['map_match']:
                        return {'success': False, 'message': '차량 정지·현재 위치·지도 일치를 기다립니다.'}
                    service = ('/external_navigation/reinitialize' if op == 'recover_externalnav'
                               else '/navigation/acknowledge_tracking_stop')
                    response = self.vehicle_service(Trigger, service, Trigger.Request())
                    return {'success': response.success, 'message': response.message}
            if op == 'status':
                return self.status()
            if op == 'pose':
                self.check_start()
                return self.current_pose()
            if op == 'plan':
                self.check_start()
                if not self.planner.wait_for_server(timeout_sec=3.):
                    raise RuntimeError('Nav2 경로 서버가 없습니다.')
                g = ComputePathToPose.Goal()
                g.goal, g.planner_id, g.use_start = self.stamped(args['pose']), 'GridBased', False
                handle = wait(self.planner.send_goal_async(g), 10)
                if not handle.accepted:
                    raise RuntimeError('경로 요청이 거절됐습니다.')
                try:
                    result = wait(handle.get_result_async(), 120)
                except Exception:
                    handle.cancel_goal_async()
                    raise
                if result.status != 4 or not result.result.path.poses:
                    raise RuntimeError('해당 도착 자세로 갈 수 있는 경로를 찾지 못했습니다.')
                return [pose_dict(p.pose) for p in result.result.path.poses]
            if op in ('stop_motion', 'resume_motion'):
                with self.nav_lock:
                    target = args.get('goal_id')
                    if target and op == 'stop_motion':
                        self.paused_ids.add(target)
                    if target and op == 'resume_motion':
                        self.paused_ids.discard(target)
                    if not self.motion_pause.wait_for_service(timeout_sec=1.):
                        raise RuntimeError('주행 정지 서비스를 확인할 수 없습니다.')
                    response = wait(self.motion_pause.call_async(self.motion_pause_type.Request(data=op=='stop_motion')), 3)
                    if not response.success:
                        raise RuntimeError('주행 출력 정지가 거절됐습니다.')
                    return {'accepted': True}
            if op == 'navigate':
                with self.nav_lock:
                    if args['goal_id'] in self.canceled_ids:
                        raise RuntimeError('취소된 임무는 시작하지 않습니다.')
                    s = self.status()
                    if s['map_match'] and (not s['ready'] or not s['pose']) and not s['external_goal']:
                        # Nothing was submitted: the manager may wait and retry
                        # this same goal without ambiguous duplicate actions.
                        return {'accepted': False, 'retryable': True, 'reason': s['reason']}
                    self.check_start(s)
                    if config.get('service_controls'):
                        f = s.get('fcu') or {}
                        if not f.get('connected') or not f.get('armed') or f.get('mode') != 'GUIDED':
                            raise RuntimeError('GUIDED·시동 상태가 해제됐습니다. 다시 주행 준비를 진행하세요.')
                    if self.nav_state in ('accepting', 'executing', 'canceling'):
                        raise RuntimeError('이미 실행 중인 이동 임무가 있습니다.')
                    if not self.nav.wait_for_server(timeout_sec=3.):
                        raise RuntimeError('Nav2 주행 서버가 없습니다.')
                    if not self.motion_pause.wait_for_service(timeout_sec=1.):
                        raise RuntimeError('주행 정지 서비스가 없습니다. 갱신된 navigation을 실행하세요.')
                    response = wait(self.motion_pause.call_async(self.motion_pause_type.Request(data=args['goal_id'] in self.paused_ids)), 3)
                    if not response.success:
                        raise RuntimeError('주행 정지 해제를 확인하지 못했습니다.')
                    if args['observation']:
                        tree = self.observation_tree()
                    else:
                        tree = ''
                    self.cancel_requested = False
                    self.nav_state, self.goal_id = 'accepting', args['goal_id']
                g = NavigateToPose.Goal(pose=self.stamped(args['pose']), behavior_tree=tree)
                def feedback(msg):
                    self.distance = msg.feedback.distance_remaining
                future = self.nav.send_goal_async(g, feedback_callback=feedback,
                    goal_uuid=UUID(uuid=list(uuid.UUID(hex=self.goal_id).bytes)))
                # Late acceptance is handled even if the HTTP caller times out.
                def accepted(f):
                    try:
                        handle = f.result()
                        self.goal = handle
                        if not handle.accepted:
                            self.nav_state = 'rejected'
                            return
                        self.nav_state = 'executing'
                        def finished(done):
                            try:
                                self.nav_state = {4:'succeeded', 5:'canceled', 6:'failed'}.get(done.result().status, 'failed')
                            except Exception:
                                self.nav_state = 'failed'
                        handle.get_result_async().add_done_callback(finished)
                        if self.cancel_requested:
                            self.nav_state = 'canceling'
                            handle.cancel_goal_async()
                    except Exception:
                        self.nav_state = 'failed'
                future.add_done_callback(accepted)
                return {'goal_id': self.goal_id}
            if op == 'cancel':
                target = args.get('goal_id', self.goal_id)
                with self.nav_lock:
                    if target:
                        self.canceled_ids.add(target)
                    if target != self.goal_id:
                        return {'accepted': True}
                    self.cancel_requested = True
                if self.goal and self.goal.accepted and self.nav_state in ('executing', 'canceling'):
                    self.nav_state = 'canceling'
                    result = wait(self.goal.cancel_goal_async(), 8)
                    return {'accepted': bool(result.goals_canceling)}
                return {'accepted': self.nav_state == 'accepting'}
            if op in ('preview', 'capture'):
                now = time.monotonic()
                msg = self.raw_image
                if msg is None or now-self.image_at > 3.:
                    raise RuntimeError('새 카메라 영상이 없습니다.')
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
                if abs(self.get_clock().now().nanoseconds/1e9-stamp) > 3.:
                    raise RuntimeError('카메라 영상 timestamp가 오래되었습니다.')
                if op == 'capture':
                    if self.image_at <= args['after']:
                        raise RuntimeError('도착 이후의 새 영상을 기다리는 중입니다.')
                    if not self.standstill.ready(now):
                        raise RuntimeError('촬영 전 차량 정지를 기다리는 중입니다.')
                elif self.preview_cache and now-self.preview_at < .5:
                    return self.preview_cache
                enc = {'rgb8':('RGB','RGB'), 'bgr8':('RGB','BGR'),
                       'rgba8':('RGBA','RGBA'), 'bgra8':('RGBA','BGRA'), 'mono8':('L','L')}
                if isinstance(msg, CompressedImage):
                    im = Image.open(io.BytesIO(bytes(msg.data))).convert('RGB')
                else:
                    if msg.encoding not in enc:
                        raise RuntimeError('지원하지 않는 카메라 encoding: '+msg.encoding)
                    mode, raw = enc[msg.encoding]
                    im = Image.frombytes(mode, (msg.width, msg.height), bytes(msg.data), 'raw', raw, msg.step).convert('RGB')
                im.thumbnail((640, 480) if op == 'preview' else (1600, 1200))
                out = io.BytesIO()
                im.save(out, format='JPEG', quality=75 if op == 'preview' else 92)
                result = {'jpeg': base64.b64encode(out.getvalue()).decode(), 'stamp': stamp,
                          'received_at': self.image_at, 'pose': self.current_pose() if op == 'capture' else None}
                if op == 'preview':
                    self.preview_cache, self.preview_at = result, now
                return result
            if op == 'shutdown':
                self.execute('cancel', {})
                running.clear()
                return True
            raise ValueError('Unknown ROS operation')

    rclpy.init()
    node = Worker()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=3)

    def request(msg):
        try:
            send({'id': msg['id'], 'result': node.execute(msg['op'], msg['args'])})
        except Exception as exc:
            send({'id': msg['id'], 'error': str(exc)[:600]})

    try:
        while running.is_set():
            if pipe.poll(.2):
                pool.submit(request, pipe.recv())
    except (EOFError, OSError):
        pass
    finally:
        try:
            node.execute('cancel', {})
        except Exception:
            pass
        pool.shutdown(wait=False, cancel_futures=True)
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        rclpy.shutdown()
