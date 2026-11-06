"""Actual Nav2/BT/guard with synthetic map and sensors; no motion backend."""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '184'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ.pop('CYCLONEDDS_URI', None)

import rclpy
from geometry_msgs.msg import TransformStamped
from g1_task_interfaces.action import Speak
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from g1_task_manager.action_client import SynchronizedActionClient


def until(predicate, timeout=15.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.025)
    raise AssertionError('condition timed out')


class SyntheticWorld(Node):
    def __init__(self):
        super().__init__('guard_stack_test_world')
        self.group = ReentrantCallbackGroup()
        self.wall = 'partial'
        self.speeches = []
        self.statuses = []
        self.route = None
        self.maps = {}
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.health_pub = self.create_publisher(String, '/g1_relocalization/status', qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered_body', 1)
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, '/g1_navigation/status',
                                 lambda m: self.statuses.append(json.loads(m.data)), 100)
        self.create_subscription(NavPath, '/g1_navigation/path', self.path_callback, 1)
        for topic in ('/global_costmap/costmap', '/g1_reference/global_costmap/costmap'):
            self.create_subscription(OccupancyGrid, topic,
                                     lambda m, key=topic: self.maps.update({key: m}), 1)
        self.speech = ActionServer(self, Speak, '/g1_audio/speak', self.speak,
                                   callback_group=self.group)
        self.client = SynchronizedActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=self.group)
        self.create_timer(.05, self.publish, callback_group=self.group)

    def path_callback(self, message):
        self.route = message

    def speak(self, handle):
        self.speeches.append(handle.request.text)
        handle.succeed()
        return Speak.Result(success=True, message='mock speaker; no audio emitted')

    def publish(self):
        now = self.get_clock().now()
        transforms = []
        for parent, child, inverted in (('map', 'camera_init', False),
                                         ('camera_init', 'body', True)):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.header.stamp = now.to_msg()
            transform.transform.rotation.x = 1. if inverted else 0.
            transform.transform.rotation.w = 0. if inverted else 1.
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        self.health_pub.publish(String(data=json.dumps(dict(
            valid=True, state='TRACKING', stamp_ns=now.nanoseconds,
            last_success_stamp_ns=now.nanoseconds))))
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'camera_init', 'body'
        odom.header.stamp = now.to_msg()
        odom.pose.pose.orientation.x = 1.
        self.odom_pub.publish(odom)
        grid = OccupancyGrid()
        grid.header.frame_id, grid.header.stamp = 'map', now.to_msg()
        grid.info.resolution, grid.info.width, grid.info.height = .05, 140, 48
        grid.info.origin.position.x, grid.info.origin.position.y = -1., -1.2
        grid.info.origin.orientation.w = 1.
        grid.data = [100 if x in (0, 139) or y in (0, 47) else 0
                     for y in range(48) for x in range(140)]
        self.map_pub.publish(grid)
        # Far returns clear previous marks; they are outside default marking
        # range, within raytrace range. Near returns mark the synthetic wall.
        points = [(2.6, y*.013, 0.) for y in range(-120, 121)]
        extent = 120 if self.wall == 'full' else 15
        if self.wall != 'clear':
            points += [(2., y*.01, 0.) for y in range(-extent, extent+1)]
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = 'nav_base', now.to_msg()
        cloud.height, cloud.width, cloud.point_step = 1, len(points), 12
        cloud.row_step, cloud.is_dense = len(points)*12, True
        cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.data = b''.join(struct.pack('<fff', *point) for point in points)
        self.cloud_pub.publish(cloud)

    def destroy_node(self):
        self.client.destroy()
        self.speech.destroy()
        return super().destroy_node()


class NavigationStackTest(unittest.TestCase):
    def test_real_planners_dwb_bt_block_and_clear_without_robot(self):
        directory = tempfile.TemporaryDirectory(prefix='g1_nav_stack_')
        os.environ['ROS_LOG_DIR'] = str(Path(directory.name) / 'ros_logs')
        rclpy.init(args=[])
        world = SyntheticWorld()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(world)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        output = open(Path(directory.name) / 'stack.log', 'w+')
        process = subprocess.Popen(['ros2', 'launch', 'g1_nav2_bringup', 'planning_only.launch.py'],
                                   stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        handle = None
        try:
            for name in ('controller_server', 'planner_server', 'bt_navigator', 'g1_reference/planner_server'):
                client = world.create_client(GetState, '/' + name + '/get_state', callback_group=world.group)
                until(lambda: client.service_is_ready(), 25.)
                def active():
                    future = client.call_async(GetState.Request())
                    until(future.done, 2.)
                    return future.result().current_state.id == 3
                until(active, 25.)
            until(lambda: world.client.server_is_ready())
            until(lambda: '/global_costmap/costmap' in world.maps and
                  world.maps['/global_costmap/costmap'].data[24*140+60] >= 99)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x = 5.
            goal.pose.pose.orientation.w = 1.
            accepted = world.client.send_goal_async(goal)
            until(accepted.done)
            handle = accepted.result()
            self.assertTrue(handle.accepted)
            until(lambda: world.statuses and world.statuses[-1]['permit'])
            self.assertTrue(any(abs(p.pose.position.y) > .4 for p in world.route.poses))
            self.assertEqual(world.speeches, [])
            self.assertEqual(len(world.maps), 2)
            # Reproduce the field sequence: cancel, then immediately resend the
            # identical pose. Late results from the old call must stay isolated.
            for _ in range(8):
                previous = handle.get_result_async()
                until(handle.cancel_goal_async().done)
                until(previous.done)
                self.assertEqual(previous.result().status, 5)
                session = world.statuses[-1]['session']
                accepted = world.client.send_goal_async(goal)
                until(accepted.done)
                handle = accepted.result()
                self.assertTrue(handle.accepted)
                until(lambda: world.statuses[-1]['permit'] and
                      world.statuses[-1]['session'] != session)
            world.wall = 'full'
            until(lambda: len(world.speeches) == 1)
            self.assertFalse(world.statuses[-1]['permit'])
            self.assertEqual(world.speeches, ['前方通行受阻，麻烦让一让。'])
            time.sleep(1.1)
            self.assertEqual(len(world.speeches), 1)
            before = time.monotonic()
            world.wall = 'clear'
            until(lambda: world.statuses[-1]['permit'])
            self.assertGreaterEqual(time.monotonic()-before, 1.9)
            session = world.statuses[-1]['session']
            goal.pose.pose.position.x = 4.5
            replacement = world.client.send_goal_async(goal)
            until(replacement.done)
            handle = replacement.result()
            self.assertTrue(handle.accepted)
            until(lambda: world.statuses[-1]['session'] != session and
                  world.statuses[-1]['permit'] and
                  world.statuses[-1]['goal']['x'] == 4.5)
            result = handle.get_result_async()
            until(handle.cancel_goal_async().done)
            until(result.done)
            self.assertEqual(result.result().status, 5)
            until(lambda: not world.statuses[-1]['active'])
            self.assertFalse(world.statuses[-1]['permit'])
        except Exception:
            print('map topics received:', list(world.maps))
            print('map headers:', [(key, str(value.header)) for key, value in world.maps.items()])
            print('clock:', world.get_clock().now().nanoseconds)
            print('latest status:', world.statuses[-1:] or None)
            output.flush()
            output.seek(0)
            print(output.read()[-22000:])
            raise
        finally:
            if handle and handle.status in (1, 2, 3):
                handle.cancel_goal_async()
                time.sleep(.2)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            executor.shutdown()
            thread.join(timeout=3)
            world.destroy_node()
            rclpy.shutdown()
            output.close()
            directory.cleanup()


if __name__ == '__main__':
    unittest.main()
