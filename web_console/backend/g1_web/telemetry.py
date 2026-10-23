"""Finite, bounded navigation telemetry. No ROS imports or control side effects."""
import math


NAV2_NODES = ('controller_server', 'planner_server', 'recoveries_server', 'bt_navigator')


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def fresh_stamp(stamp, now, maximum):
    return stamp > 0 and -0.05 <= (now - stamp) / 1e9 <= maximum


def planar_pose(position, orientation):
    values = (position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('非有限位姿')
    norm = sum(v*v for v in values[2:])
    if abs(norm - 1) > 0.02:
        raise ValueError('姿态四元数未归一化')
    x, y, z, w = values[2:]
    return dict(x=position.x, y=position.y,
                yaw=math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def path_view(message, now_ns, received_age, limit=800):
    stamp = stamp_ns(message.header.stamp)
    stale = received_age > 3 or not fresh_stamp(stamp, now_ns, 3)
    if message.header.frame_id != 'map':
        return dict(available=False, stale=True, frame_id=message.header.frame_id, points=[])
    poses = message.poses
    # Retain both ends, with a strict bound on the browser payload.
    indices = range(len(poses)) if len(poses) <= limit else [round(i*(len(poses)-1)/(limit-1)) for i in range(limit)]
    points = []
    for i in indices:
        pose = poses[i]
        if pose.header.frame_id not in ('', 'map'):
            return dict(available=False, stale=True, frame_id='map', points=[])
        p = pose.pose.position
        if not math.isfinite(p.x) or not math.isfinite(p.y):
            return dict(available=False, stale=True, frame_id='map', points=[])
        points.append([p.x, p.y])
    return dict(available=bool(points), stale=stale, frame_id='map', stamp_ns=stamp, points=points)


def navigation_readiness(localization, pose, checks):
    reasons = []
    if not localization.get('valid'):
        reasons.append('定位无效或已过期')
    if not pose:
        reasons.append('map → nav_base 位姿不可用或已过期')
    if not all(checks.get(name, {}).get('ready') for name in NAV2_NODES):
        reasons.append('Nav2 尚未全部激活')
    if not checks.get('motion_bridge', {}).get('ready'):
        reasons.append('真实运动桥未就绪')
    if not checks.get('task_manager', {}).get('ready'):
        reasons.append('真实任务执行器未就绪或配置不一致')
    return dict(ready=not reasons, blockers=reasons, checks=checks)
