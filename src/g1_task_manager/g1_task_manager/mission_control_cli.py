"""Short-lived terminal clients; never retain a release for a later station."""
import time


STATE_LABELS = {
    'STARTING': '启动中', 'CHECKING': '检查定位及导航状态', 'NAVIGATING': '正在导航',
    'ARRIVED': '已到达', 'SPEAKING_ZH': '中文播报及等待', 'SPEAKING_EN': '英文播报及等待',
    'DWELLING': '额外停留', 'WAITING_FOR_CONTINUE': '等待负责人放行',
    'CONTINUING': '放行已接受', 'CLEANING_UP': '正在收尾', 'SUCCEEDED': '已完成',
    'CANCELED': '已取消', 'FAILED': '失败', 'STOP_UNCONFIRMED': '停止未确认',
}


def add_parsers(subparsers):
    for command, help_text in (('status', '查询当前任务状态'), ('continue', '放行当前人工等待')):
        subparsers.add_parser(command, help=help_text)


def run_command(parsed):
    import rclpy
    from rclpy.node import Node
    from g1_task_interfaces.srv import GetMissionState, ContinueMission

    rclpy.init(args=[])
    node = Node('g1_mission_control_client')

    def call(client, request):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done() and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError('任务服务响应超时，请重新查询 status')
        return future.result()

    try:
        client = node.create_client(GetMissionState, '/g1_task/get_state')
        state = call(client, GetMissionState.Request())
        if state is None or not state.active:
            print('[INFO] 没有运行中的任务' + ('（任务状态服务不可用）' if state is None else ''))
            return 0 if parsed.command == 'status' else 2
        print('任务：{} ({}){}'.format(state.mission_name, state.mission_id,
                                    ' [模拟演练]' if state.simulated else ''))
        print('进度：第 {} / {} 点\n当前点：{}\n状态：{} ({})\n下一点：{}'.format(
            state.current_step, state.total_steps, state.current_waypoint or '尚未开始',
            STATE_LABELS.get(state.state, state.state), state.state, state.next_waypoint or '无'))
        if state.message:
            print('提示：' + state.message)
        if parsed.command == 'status':
            return 0
        if state.state != 'WAITING_FOR_CONTINUE' or not state.wait_id:
            print('[ERROR] 当前阶段不能放行；放行请求不会预存。')
            return 2
        client = node.create_client(ContinueMission, '/g1_task/continue')
        response = call(client, ContinueMission.Request(run_id=state.run_id, wait_id=state.wait_id))
        if response is None:
            print('[ERROR] 放行服务不可用，请重新查询 status')
            return 2
        print(('[OK] ' if response.accepted else '[ERROR] ') + response.message)
        return 0 if response.accepted else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
