"""Run only with an isolated ROS_DOMAIN_ID, synthetic sensors and fake actions."""
import asyncio
import math
import os
from pathlib import Path
import threading
import time
import uuid

import pytest


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='91',reason='requires isolated domain 91')
@pytest.mark.parametrize('transport', ['raw', 'compressed'])
def test_real_ros_process_actions_map_camera_and_cancel(tmp_path, transport):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer,CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid,Odometry
    from nav2_msgs.action import NavigateToPose,ComputePathToPose
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from std_msgs.msg import Bool
    from sensor_msgs.msg import Image as RosImage, CompressedImage
    from mavros_msgs.msg import State
    from mavros_msgs.srv import SetMode, CommandBool
    from std_srvs.srv import SetBool, Trigger
    from tf2_ros import TransformBroadcaster
    from PIL import Image
    import yaml
    from campus_mission.catalog import Catalog
    from campus_mission.bridge import RosBridge

    Image.new('L',(80,80),254).save(tmp_path/'map.pgm')
    (tmp_path/'rtabmap.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.1,
        'origin':[-4.,-4.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
    config={'map_yaml':str(tmp_path/'rtabmap.yaml'),'places_file':str(tmp_path/'places.yaml'),
            'data_dir':str(tmp_path),'service_controls':True,'odom_topic':'/odom','image_topic':'/camera/test'}
    config['image_transport'] = transport
    rclpy.init()
    node=Node('mission_fake_hardware')
    controller=Node('controller_server');controller.declare_parameter('goal_checker_plugins',['goal_checker','observation_goal_checker'])
    navigator=Node('bt_navigator')
    tree=tmp_path/'default.xml';tree.write_text('<root><BehaviorTree><FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker"/></BehaviorTree></root>')
    navigator.declare_parameter('default_nav_to_pose_bt_xml',str(tree))
    executor=MultiThreadedExecutor(num_threads=5)
    for n in (node,controller,navigator):executor.add_node(n)
    group=ReentrantCallbackGroup()
    pos=[0.,0.,0.];observed=[];pause=[False]
    fcu = State(connected=True, armed=False, mode='HOLD')
    vehicle_calls = []
    deny_arm = [False]
    arm_entered = threading.Event()
    arm_release = threading.Event(); arm_release.set()
    fcu_pub = node.create_publisher(State, '/mavros/state', 1)
    def change_mode(request, response):
        vehicle_calls.append(('mode', request.custom_mode))
        fcu.mode = request.custom_mode
        response.mode_sent = True
        return response
    def change_arm(request, response):
        vehicle_calls.append(('arm', request.value))
        if request.value:
            arm_entered.set(); arm_release.wait(5)
        if deny_arm[0] and request.value:
            response.success = False
            response.result = 2
            return response
        fcu.armed = request.value
        response.success = True
        return response
    node.create_service(SetMode, '/mavros/set_mode', change_mode, callback_group=group)
    node.create_service(CommandBool, '/mavros/cmd/arming', change_arm, callback_group=group)
    recovery_calls=[]
    def recover_input(request,response):
        recovery_calls.append('recover');response.success=True;response.message='ready';return response
    node.create_service(Trigger,'/external_navigation/reinitialize',recover_input,callback_group=group)
    node.create_service(Trigger,'/navigation/acknowledge_tracking_stop',recover_input,callback_group=group)
    pause_calls=[]
    def set_motion_paused(request,response):
        pause_calls.append(request.data);response.success=True;return response
    node.create_service(SetBool,'/navigation/set_motion_paused',set_motion_paused,callback_group=group)
    def plan(handle):
        result=ComputePathToPose.Result();result.path.header.frame_id='map'
        p=PoseStamped();p.pose.orientation.w=1.;p.pose.position.x=pos[0];p.pose.position.y=pos[1]
        result.path.poses=[p,handle.request.goal];handle.succeed();return result
    def navigate(handle):
        observed.append(handle.request)
        for _ in range(100 if pause[0] else 3):
            if handle.is_cancel_requested:
                handle.canceled();return NavigateToPose.Result()
            time.sleep(.05)
        p=handle.request.pose.pose
        pos[:]=[p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)]
        handle.succeed();return NavigateToPose.Result()
    a=ActionServer(node,ComputePathToPose,'/compute_path_to_pose',plan,callback_group=group)
    b=ActionServer(node,NavigateToPose,'/navigate_to_pose',navigate,callback_group=group,cancel_callback=lambda _:CancelResponse.ACCEPT)
    ready=node.create_publisher(Bool,'/navigation/ready',1)
    map_pub=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    image=node.create_publisher(CompressedImage if transport=='compressed' else RosImage,
                                '/camera/test'+('/compressed' if transport=='compressed' else ''),1)
    odom=node.create_publisher(Odometry,'/odom',1)
    tf=TransformBroadcaster(node)
    grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=80;grid.info.height=80;grid.info.resolution=.1
    grid.info.origin.position.x=-4.;grid.info.origin.position.y=-4.;grid.info.origin.orientation.w=1.;grid.data=[0]*6400
    map_pub.publish(grid)
    camera_on=[True]; odom_on=[True]
    def publish():
        stamp=node.get_clock().now().to_msg();ready.publish(Bool(data=True))
        fcu_pub.publish(fcu)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link';t.header.stamp=stamp
        t.transform.translation.x=pos[0];t.transform.translation.y=pos[1]
        t.transform.rotation.z=math.sin(pos[2]/2);t.transform.rotation.w=math.cos(pos[2]/2);tf.sendTransform(t)
        o=Odometry();o.header.stamp=stamp
        if odom_on[0]:odom.publish(o)
        if camera_on[0]:
            if transport == 'compressed':
                import io
                stream=io.BytesIO(); Image.new('RGB',(30,20),(50,100,200)).save(stream,format='JPEG')
                msg=CompressedImage();msg.header.stamp=stamp;msg.format='jpeg';msg.data=stream.getvalue()
            else:
                msg=RosImage();msg.header.stamp=stamp;msg.height=20;msg.width=30;msg.encoding='rgb8';msg.step=90;msg.data=bytes([50,100,200])*600
            image.publish(msg)
    timer=node.create_timer(.1,publish)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()

    async def scenario():
        bridge=RosBridge(config)
        try:
            for _ in range(100):
                if bridge.state.get('ready') and bridge.state.get('map_match') and bridge.state.get('pose'):break
                await asyncio.sleep(.2)
            assert bridge.state.get('map_match'),bridge.state
            assert bridge.state.get('ready')
            await asyncio.sleep(.8)
            assert vehicle_calls == []  # Starting/reconnecting the web never arms.
            await bridge.call('prepare_vehicle', timeout=25)
            assert vehicle_calls == [('mode','GUIDED'),('arm',True)]
            await bridge.call('prepare_vehicle', timeout=25)
            assert len(vehicle_calls) == 2  # Already prepared: no repeated command.
            odom_on[0]=False;await asyncio.sleep(2.2)
            assert not (await bridge.call('status'))['stationary']
            await bridge.call('disarm_vehicle', timeout=25)
            assert not fcu.armed  # Faulty/missing odometry cannot prevent disarm.
            odom_on[0]=True;await asyncio.sleep(.8)
            assert vehicle_calls[-2:] == [('mode','HOLD'),('arm',False)]
            deny_arm[0] = True
            with pytest.raises(RuntimeError,match='거절'):
                await bridge.call('prepare_vehicle',timeout=25)
            assert not fcu.armed
            deny_arm[0] = False
            await bridge.call('disarm_vehicle',timeout=25)
            # Cancel before a queued arm, and cancel after the service received
            # arm but before its response. Both must leave no delayed arm.
            request_id=uuid.uuid4().hex
            await bridge.call('cancel_prepare_vehicle',request_id=request_id)
            with pytest.raises(RuntimeError,match='취소'):
                await bridge.call('prepare_vehicle',request_id=request_id)
            assert not fcu.armed
            request_id=uuid.uuid4().hex
            arm_entered.clear(); arm_release.clear()
            preparing=asyncio.create_task(bridge.call('prepare_vehicle',request_id=request_id,timeout=30))
            assert await asyncio.to_thread(arm_entered.wait,10)
            canceling=asyncio.create_task(bridge.call('cancel_prepare_vehicle',request_id=request_id,timeout=30))
            await asyncio.sleep(.2); arm_release.set()
            await asyncio.gather(preparing,return_exceptions=True)
            await canceling
            assert not fcu.armed and fcu.mode=='HOLD'
            # A QGC-like mode change after web arming revokes that request's
            # cleanup permission: cancel must NOT disarm the operator's state.
            revision = (await bridge.call('status'))['control_revision']
            request_id = uuid.uuid4().hex
            await bridge.call('prepare_vehicle', request_id=request_id, control_revision=revision)
            fcu.mode = 'MANUAL'
            await asyncio.sleep(.6)
            before = len(vehicle_calls)
            await bridge.call('cancel_prepare_vehicle', request_id=request_id)
            assert fcu.mode == 'MANUAL' and fcu.armed
            assert len(vehicle_calls) == before
            with pytest.raises(RuntimeError, match='외부에서 변경'):
                await bridge.call('prepare_vehicle', request_id=uuid.uuid4().hex, control_revision=revision)
            assert len(vehicle_calls) == before
            # Manual disarm also invalidates an older queued preparation.
            revision = (await bridge.call('status'))['control_revision']
            fcu.armed = False
            await asyncio.sleep(.6)
            with pytest.raises(RuntimeError, match='외부에서 변경'):
                await bridge.call('prepare_vehicle', request_id=uuid.uuid4().hex, control_revision=revision)
            assert len(vehicle_calls) == before and not fcu.armed
            # A request that found the rover already armed must preserve it.
            await bridge.call('prepare_vehicle')
            request_id=uuid.uuid4().hex
            await bridge.call('prepare_vehicle',request_id=request_id)
            await bridge.call('cancel_prepare_vehicle',request_id=request_id)
            assert fcu.armed and fcu.mode=='GUIDED'
            with pytest.raises(RuntimeError,match='시동 해제'):
                await bridge.call('recover_externalnav',request_id=uuid.uuid4().hex)
            await bridge.call('disarm_vehicle')
            assert (await bridge.call('recover_externalnav',request_id=uuid.uuid4().hex))['success']
            assert (await bridge.call('acknowledge_tracking_stop',request_id=uuid.uuid4().hex))['success']
            assert recovery_calls==['recover','recover']
            target={'x':1.,'y':1.,'yaw':1.2}
            path=await bridge.call('plan',pose=target)
            assert path[-1]['x']==1.
            ident=uuid.uuid4().hex
            with pytest.raises(RuntimeError,match='GUIDED'):
                await bridge.call('navigate',goal_id=ident,pose=target,observation=True)
            await bridge.call('prepare_vehicle')
            await bridge.call('navigate',goal_id=ident,pose=target,observation=True)
            for _ in range(60):
                state=await bridge.call('status')
                if state['navigation']=='succeeded':break
                await asyncio.sleep(.1)
            assert state['navigation']=='succeeded'
            assert 'observation_goal_checker' in Path(observed[-1].behavior_tree).read_text()
            after=time.monotonic();await asyncio.sleep(.8)
            capture=await bridge.call('capture',after=after)
            assert capture['jpeg'] and abs(capture['pose']['yaw']-1.2)<.01
            camera_on[0]=False;await asyncio.sleep(3.3)
            with pytest.raises(RuntimeError,match='영상'):await bridge.call('preview')
            camera_on[0]=True
            pause[0]=True;ident=uuid.uuid4().hex
            await bridge.call('navigate',goal_id=ident,pose=target,observation=False)
            await asyncio.sleep(.3)
            await bridge.call('stop_motion',goal_id=ident)
            assert (await bridge.call('status'))['navigation']=='executing'
            before=len(vehicle_calls)
            await bridge.call('resume_motion',goal_id=ident)
            assert len(vehicle_calls)==before and pause_calls[-2:]==[True,False]
            await bridge.call('disarm_vehicle',timeout=30)  # Explicit stop also cancels active navigation.
            for _ in range(40):
                state=await bridge.call('status')
                if state['navigation']=='canceled':break
                await asyncio.sleep(.1)
            assert state['navigation']=='canceled'
            # A cancellation received before a delayed navigate RPC must win.
            canceled=uuid.uuid4().hex;await bridge.call('cancel',goal_id=canceled)
            with pytest.raises(RuntimeError,match='취소된'):
                await bridge.call('navigate',goal_id=canceled,pose=target,observation=False)
            grid.data[0]=100;map_pub.publish(grid);await asyncio.sleep(.8)
            assert not bridge.state['map_match']
        finally:
            await bridge.close()
    try:
        asyncio.run(scenario())
    finally:
        executor.shutdown(timeout_sec=3);a.destroy();b.destroy()
        for n in (node,controller,navigator):n.destroy_node()
        rclpy.shutdown()
