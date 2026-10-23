from g1_web.compat import to_thread
"""Opt-in Humble integration check, restricted to a loopback DDS interface.

Run with G1_NETWORK_INTERFACE=lo, an isolated ROS_DOMAIN_ID and loaded Humble
workspace. This fixture publishes synthetic localization and serves mission state.
It never starts the robot stack or sends a motion/control request.
"""
import asyncio
import json
import os
import threading
import time
import struct
import tempfile
from pathlib import Path

from g1_web.adapters.ros_readonly import RosReadOnlyAdapter
from g1_web.mapping import SCRIPT_SOURCE
from g1_mapping_save import save


async def main():
    if os.environ.get("G1_NETWORK_INTERFACE") != "lo" or not 100 <= int(os.environ.get("ROS_DOMAIN_ID", "0")) <= 232:
        raise RuntimeError("测试仅允许 lo 和隔离的 ROS_DOMAIN_ID=100..232")
    # The test itself fixes the DDS transport to loopback, regardless of shell configuration.
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces></General><Discovery><Peers><Peer address="127.0.0.1"/></Peers><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import PointCloud2, PointField
    from g1_task_interfaces.srv import GetMissionState

    context = Context()
    rclpy.init(context=context)
    fixture = Node("g1_web_readonly_fixture", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(fixture)
    thread = threading.Thread(target=executor.spin, daemon=True)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = fixture.create_publisher(String, "/g1_relocalization/status", qos)
    cloud_publisher = fixture.create_publisher(PointCloud2, '/cloud_registered',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    temporary = tempfile.TemporaryDirectory(prefix='g1-mapping-save-ros-')
    output = Path(temporary.name) / 'map.pcd'
    allow_save = [True]

    def save_map(request, response):
        response.success = allow_save[0]
        response.message = 'fixture saved' if allow_save[0] else 'fixture rejected'
        if allow_save[0]:
            output.write_bytes(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 1\nDATA binary\n' + struct.pack('<fff', 1, 2, 3))
        return response

    save_service = fixture.create_service(Trigger, '/map_save', save_map)

    def publish():
        now = fixture.get_clock().now().nanoseconds
        publisher.publish(String(data=json.dumps({"state": "TRACKING", "valid": True,
                          "stamp_ns": now, "last_success_stamp_ns": now, "fitness": 0.1})))
        cloud = PointCloud2()
        cloud.header.frame_id = 'camera_init'
        cloud.height, cloud.width = 1, 2
        cloud.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                        for index, name in enumerate(('x', 'y', 'z'))]
        cloud.point_step, cloud.row_step = 12, 24
        cloud.data = struct.pack('<ffffff', 1, 2, 3, 4, 5, 6)
        cloud_publisher.publish(cloud)

    def get_state(request, response):
        response.active = response.simulated = True
        response.run_id = "isolated-fixture-run"
        response.mission_id = "five_point_tour"
        response.mission_name = "隔离测试任务"
        response.state = "WAITING_FOR_CONTINUE"
        response.current_step, response.total_steps = 1, 5
        response.current_waypoint, response.next_waypoint = "point_01", "point_02"
        response.wait_id = "isolated-fixture-wait"
        return response

    timer = fixture.create_timer(0.1, publish)
    service = fixture.create_service(GetMissionState, "/g1_task/get_state", get_state)
    adapter = RosReadOnlyAdapter()
    thread.start()
    try:
        await adapter.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            data = await adapter.snapshot()
            if data["localization"]["valid"] and data["mission"]["run_id"] == "isolated-fixture-run" and data['mapping_preview']['available']:
                break
            await asyncio.sleep(0.1)
        else:
            raise AssertionError("未收到隔离 ROS 定位与任务状态")
        assert not any(data["capabilities"].values())
        assert data["mission"]["gate_closed"] is None and data["mission"]["goals_terminal"] is None
        assert data["pose"] is None and not data["map"]["available"]
        assert set(data['mapping_preview']['points']) == {(1., 2.), (4., 5.)}
        await to_thread(save, output, 5)
        allow_save[0] = False
        try:
            await to_thread(save, output, 5)
        except RuntimeError as exc:
            assert 'success=false' in str(exc)
        else:
            raise AssertionError('Trigger.success=false must not report the old PCD as saved')
        published = {name for name, types in adapter.node.get_publisher_names_and_types_by_node("g1_web_readonly", "/")}
        assert published <= {"/rosout", "/parameter_events"}, published
        timer.cancel()
        fixture.destroy_service(service)
        await asyncio.sleep(3.3)
        stale = await adapter.snapshot()
        assert stale["localization"]["stale"] and not stale["localization"]["valid"]
        assert stale["mission"]["stale"]
        assert stale['mapping_preview']['stale']
        print("ROS checks passed: isolated telemetry, mapping PointCloud2 preview, Trigger save success/rejection, no business publishers, stale-data detection.")
    finally:
        await adapter.close()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        fixture.destroy_node()
        context.try_shutdown()
        temporary.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
