"""Explicitly isolated ROS navigation fixture. No robot nodes, velocity or audio requests."""
import asyncio
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import uuid


async def main():
    # Set transport before importing ROS. Tests cannot use a caller's robot DDS.
    os.environ.update(G1_NETWORK_INTERFACE='lo', ROS_DOMAIN_ID='181', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
        ROS_LOCALHOST_ONLY='0', CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Path as NavPath
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from lifecycle_msgs.srv import GetState
    from tf2_ros import TransformBroadcaster
    from g1_task_interfaces.action import ExecuteMission, Speak
    from g1_task_interfaces.srv import GetMissionState, ContinueMission
    from g1_web.adapters.ros_control import RosControlAdapter
    from g1_web.config import Settings
    from test_navigation import fixture

    with tempfile.TemporaryDirectory(prefix='g1-nav-ros-') as directory:
        root=Path(directory); fixture(root)
        os.environ['ROS_LOG_DIR']=str(root/'ros_logs')
        settings=Settings(mode='ros_control',token='fixture',workspace=str(root))
        context=Context(); rclpy.init(context=context)
        nodes=[]
        def node(name, params=None):
            n=Node(name,context=context)
            for key,value in (params or {}).items(): n.declare_parameter(key,value)
            nodes.append(n); return n
        manager=node('g1_task_manager',dict(execute_real=True,waypoints_file=str(root/'task_data/maps/laboratory_map_v1/waypoints.yaml'),missions_file=str(root/'task_data/missions/missions.yaml')))
        motion=node('g1_cmd_vel_dry_run',dict(dry_run=False,real_backend=True,backend_name='unitree_ros2',cmd_vel_topic='/cmd_vel_nav',enable_service_name='/g1_loco_bridge/enable_real_nav',require_localization=True,allow_lateral=False,allow_reverse=False,max_linear_x=.2,max_angular_z=.4))
        audio=node('g1_audio_bridge',dict(dry_run=False))
        source=node('synthetic_navigation')
        services=[]
        for name in ('controller_server','planner_server','recoveries_server','bt_navigator'):
            n=node(name)
            def lifecycle(request,response):
                response.current_state.id=3; response.current_state.label='active'; return response
            services.append(n.create_service(GetState,'/'+name+'/get_state',lifecycle))
        executor=MultiThreadedExecutor(num_threads=6,context=context)
        for n in nodes: executor.add_node(n)
        group=ReentrantCallbackGroup()
        gate_requests=[]; initial=[]; release=[False]; keep_publishing=[True]
        state=dict(active=False,simulated=False,run_id='',mission_id='',mission_name='',state='IDLE',current_step=0,total_steps=0,current_waypoint='',next_waypoint='',wait_id='',message='')
        def gate(request,response):
            gate_requests.append(request.data); response.success=True; response.message='fixture closed'; return response
        services.append(motion.create_service(SetBool,'/g1_loco_bridge/enable_real_nav',gate))
        initial_subscription=source.create_subscription(PoseWithCovarianceStamped,'/initialpose',initial.append,1)
        def get_state(request,response):
            for key,value in state.items(): setattr(response,key,value)
            return response
        def continue_wait(request,response):
            response.accepted=(request.run_id==state['run_id'] and request.wait_id==state['wait_id'] and state['state']=='WAITING_FOR_CONTINUE')
            if response.accepted: release[0]=True; state.update(wait_id='',state='CONTINUING')
            response.message='fixture release'; return response
        services.append(manager.create_service(GetMissionState,'/g1_task/get_state',get_state,callback_group=group))
        services.append(manager.create_service(ContinueMission,'/g1_task/continue',continue_wait,callback_group=group))
        def execute(handle):
            release[0]=False
            state.update(active=True,run_id=uuid.uuid4().hex,mission_id=handle.request.mission_id,mission_name='隔离测试',state='NAVIGATING',current_step=1,total_steps=1,current_waypoint='point_01',wait_id='')
            started=time.monotonic()
            while not handle.is_cancel_requested:
                if time.monotonic()-started>.3:
                    if handle.request.mission_id.startswith('waypoint:') or release[0]: break
                    if state['state']!='WAITING_FOR_CONTINUE': state.update(state='WAITING_FOR_CONTINUE',wait_id=uuid.uuid4().hex)
                feedback=ExecuteMission.Feedback()
                for key in ('state','run_id','wait_id','current_step','total_steps','current_waypoint','next_waypoint','simulated'): setattr(feedback,key,state[key])
                feedback.distance_remaining=.5
                handle.publish_feedback(feedback)
                time.sleep(.05)
            state.update(active=False,state='CANCELED' if handle.is_cancel_requested else 'SUCCEEDED',wait_id='')
            if handle.is_cancel_requested: handle.canceled()
            else: handle.succeed()
            return ExecuteMission.Result(success=state['state']=='SUCCEEDED',state=state['state'],message='fixture stopped',gate_closed=True,goals_terminal=True)
        server=ActionServer(manager,ExecuteMission,'/g1_task/execute_mission',execute,callback_group=group,cancel_callback=lambda handle:CancelResponse.ACCEPT,result_timeout=60)
        def forbidden_speech(handle):
            raise AssertionError('No audio goal should ever be sent by readiness checks')
        audio_server=ActionServer(audio,Speak,'/g1_audio/speak',forbidden_speech,callback_group=group)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        localization=source.create_publisher(String,'/g1_relocalization/status',qos)
        path_publisher=source.create_publisher(NavPath,'/plan',1)
        broadcaster=TransformBroadcaster(source)
        def publish():
            if not keep_publishing[0]: return
            now=source.get_clock().now()
            localization.publish(String(data=json.dumps(dict(state='TRACKING',valid=True,stamp_ns=now.nanoseconds,last_success_stamp_ns=now.nanoseconds))))
            tf=TransformStamped(); tf.header.frame_id='map'; tf.header.stamp=now.to_msg(); tf.child_frame_id='nav_base'
            tf.transform.translation.x=1.; tf.transform.translation.y=2.; tf.transform.rotation.w=1.
            broadcaster.sendTransform(tf)
            path=NavPath(); path.header=tf.header
            for x,y in [(1.,2.),(2.,2.),(3.,3.)]:
                p=PoseStamped(); p.header=tf.header; p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.; path.poses.append(p)
            path_publisher.publish(path)
        timer=source.create_timer(.1,publish)
        thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
        adapter=RosControlAdapter(settings)
        async def wait(predicate,timeout=10):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                data=await adapter.snapshot()
                if predicate(data): return data
                await asyncio.sleep(.05)
            raise AssertionError('ROS fixture timed out: '+str(data))
        try:
            await adapter.start()
            data=await wait(lambda d:d['navigation']['ready'] and d['path']['available'])
            assert data['pose']['x']==1 and data['pose']['y']==2
            assert len(data['path']['points'])==3
            await adapter.initial_pose(1.,2.,.5)
            await asyncio.sleep(.2)
            assert initial and gate_requests==[False]
            await adapter.start_mission('five_point_tour')
            data=await wait(lambda d:d['mission']['state']=='WAITING_FOR_CONTINUE')
            run_id,wait_id=data['mission']['run_id'],data['mission']['wait_id']
            goal_id=data['mission']['goal_id']
            await adapter.close()  # The fixture task keeps waiting across gateway loss.
            adapter=RosControlAdapter(settings); await adapter.start()
            await wait(lambda d:d['mission']['state']=='WAITING_FOR_CONTINUE' and not d['mission']['stale'])
            await adapter.continue_mission(run_id,wait_id)
            data=await wait(lambda d:not d['mission']['active'])
            assert data['mission']['state']=='SUCCEEDED' and data['mission']['gate_closed'], data['mission']
            # A second run must be canceled by UUID and not affect earlier goals.
            await wait(lambda d:d['navigation']['ready'])
            await adapter.start_mission('five_point_tour')
            data=await wait(lambda d:d['mission']['state']=='WAITING_FOR_CONTINUE')
            assert data['mission']['goal_id']!=goal_id
            await adapter.cancel_mission(data['mission']['run_id'])
            data=await wait(lambda d:not d['mission']['active'])
            assert data['mission']['state']=='CANCELED' and data['mission']['goals_terminal']
            keep_publishing[0]=False
            data=await wait(lambda d:d['pose'] is None and d['path']['stale'] and not d['localization']['valid'],timeout=5)
            assert not data['navigation']['ready']
            print('ROS navigation checks passed: TF/path/lifecycle/parameters, initial-pose acknowledgement, Action feedback, restart ownership, one-use release, UUID cancellation, confirmed stop, stale telemetry. Loopback only.')
        finally:
            await adapter.close()
            executor.shutdown(timeout_sec=3); thread.join(timeout=3)
            server.destroy(); audio_server.destroy()
            for n in nodes: n.destroy_node()
            context.try_shutdown()


if __name__=='__main__':
    asyncio.run(main())
