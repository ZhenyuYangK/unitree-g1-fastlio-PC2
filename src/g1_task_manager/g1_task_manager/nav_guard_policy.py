"""Pure route budget and obstacle-wait policy; no ROS or robot interfaces."""
import math

TEXT = '前方通行受阻，麻烦让一让。'


def path_length(path):
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    if not points or any(not math.isfinite(v) for p in points for v in p):
        raise ValueError('empty or nonfinite path')
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


class RouteBudget:
    """Freeze the original static route; replanning cannot reset the allowance."""
    def __init__(self, reference_length, extra=10.0):
        if not all(math.isfinite(v) and v >= 0 for v in (reference_length, extra)):
            raise ValueError('invalid route budget')
        self.reference = reference_length
        self.extra = extra
        self.travelled = 0.0

    def allows(self, remaining):
        return (math.isfinite(remaining) and remaining >= 0 and
                self.travelled + remaining <= self.reference + self.extra + 1e-6)

    def advance(self, distance):
        if not math.isfinite(distance) or distance < 0:
            raise ValueError('invalid travelled distance')
        self.travelled += distance


class ObstacleWait:
    def __init__(self, speech_delay=1.0, clear_hold=2.0):
        self.speech_delay, self.clear_hold = speech_delay, clear_hold
        self.waiting = False
        self.blocked_since = None
        self.clear_since = None
        self.spoken = False
        self.episode = 0
        self.confirmed = False

    def observe(self, now, passable, obstacle=False):
        if not passable:
            if not self.waiting:
                self.episode += 1
                self.spoken = False
                self.confirmed = False
            self.confirmed |= obstacle
            self.waiting = True
            self.clear_since = None
            if obstacle:
                if self.blocked_since is None:
                    self.blocked_since = now
            else:
                self.blocked_since = None
            return False
        self.blocked_since = None
        if not self.waiting:
            return True
        if self.clear_since is None:
            self.clear_since = now
        if now - self.clear_since < self.clear_hold:
            return False
        self.waiting = False
        self.clear_since = None
        self.spoken = False
        self.confirmed = False
        return True

    def speech_due(self, now):
        return (self.waiting and not self.spoken and self.blocked_since is not None
                and now - self.blocked_since >= self.speech_delay)


def cell(grid, x, y):
    """OccupancyGrid lookup with the map origin's yaw (not an axis assumption)."""
    q = grid.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    dx, dy = x - grid.info.origin.position.x, y - grid.info.origin.position.y
    c, s = math.cos(yaw), math.sin(yaw)
    col = math.floor((c * dx + s * dy) / grid.info.resolution)
    row = math.floor((-s * dx + c * dy) / grid.info.resolution)
    if not 0 <= col < grid.info.width or not 0 <= row < grid.info.height:
        return -1
    return grid.data[row * grid.info.width + col]


def dynamic_block_on_path(path, live, reference, start_index=0):
    if path.header.frame_id != live.header.frame_id or live.header.frame_id != reference.header.frame_id:
        return False
    if live.info.resolution <= 0 or reference.info.resolution <= 0:
        return False
    # An occupied/inscribed cell absent from the static reference is evidence
    # of a live obstruction. Unknown cells and service failures are not.
    return any(cell(live, p.pose.position.x, p.pose.position.y) >= 99 and
               0 <= cell(reference, p.pose.position.x, p.pose.position.y) < 99
               for p in path.poses[start_index:])


def evaluation_state(message):
    """Use DWB's structured result, never parse console logs as a sensor."""
    if not message.twists:
        return 'FAULT'
    legal = [t for t in message.twists if math.isfinite(t.total) and t.total >= 0]
    obstacle = any(s.name in ('BaseObstacle', 'ObstacleFootprint') and s.raw_score < 0
                   for t in message.twists if t.total < 0 for s in t.scores)
    if legal:
        index = getattr(message, 'best_index', 0)
        if not isinstance(index, int) or not 0 <= index < len(message.twists):
            return 'FAULT'
        best = message.twists[index]
        if not math.isfinite(best.total) or best.total < 0:
            return 'FAULT'
        if obstacle and hasattr(best, 'traj'):
            velocity = best.traj.velocity
            if abs(velocity.x) < 1e-4 and abs(velocity.y) < 1e-4 and abs(velocity.theta) < 1e-4:
                return 'STATIONARY_OBSTACLE'
        return 'CLEAR'
    if obstacle:
        return 'OBSTACLE'
    return 'FAULT'


def waiting_for_this_goal(status, received_age, sent_ns, pose):
    """Only fresh, matching blocked-navigation status may pause a task timeout."""
    try:
        return (status['schema'] == 'G1_NAVIGATION_WAIT_V1' and status['active'] is True and
                0 <= received_age <= .5 and status['stamp_ns'] >= sent_ns and
                status['state'] in ('BLOCKED', 'PROBING') and status['obstacle_wait'] is True and
                status['goal']['frame'] == pose.header.frame_id and
                abs(status['goal']['x'] - pose.pose.position.x) < 1e-4 and
                abs(status['goal']['y'] - pose.pose.position.y) < 1e-4 and
                all(abs(status['goal'][key] - getattr(pose.pose.orientation, key[1:])) < 1e-4
                    for key in ('qx', 'qy', 'qz', 'qw')))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def failure_for_this_goal(status, received_age, sent_ns, pose):
    """Attach a fresh coordinator explanation while preserving ABORTED status."""
    try:
        if (status['schema'] != 'G1_NAVIGATION_WAIT_V1' or
                not 0 <= received_age <= .5 or status['stamp_ns'] < sent_ns):
            return ''
        if status['state'] == 'STOP_UNCONFIRMED' and status.get('pending_owned_goals'):
            return '上一导航目标停止尚未确认，正在核实底层目标终态'
        goal = status.get('goal') or status.get('last_goal')
        matches = (goal['frame'] == pose.header.frame_id and
                   abs(goal['x'] - pose.pose.position.x) < 1e-4 and
                   abs(goal['y'] - pose.pose.position.y) < 1e-4 and
                   all(abs(goal[key] - getattr(pose.pose.orientation, key[1:])) < 1e-4
                       for key in ('qx', 'qy', 'qz', 'qw')))
        fault = status.get('last_fault')
        return fault if matches and isinstance(fault, str) else ''
    except (KeyError, TypeError, ValueError, OverflowError):
        return ''
