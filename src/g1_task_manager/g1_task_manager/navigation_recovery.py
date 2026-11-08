"""Bounded recovery policy. A hold never grants motion or resets route budget."""
import math


RECOVERABLE = frozenset(('OBSTACLE_SENSOR_STALE', 'ODOMETRY_STALE',
                         'NAVIGATION_TF_STALE', 'NAVIGATION_TF_UNAVAILABLE',
                         'TASK_TF_STALE', 'TASK_TF_UNAVAILABLE'))


class DataHold:
    def __init__(self, timeout=3.0, stable=0.5, max_episodes=3):
        self.timeout, self.stable, self.max_episodes = timeout, stable, max_episodes
        self.episode = 0
        self.active = False
        self.started = 0.0
        self.healthy_since = None
        self.acknowledged = False
        self.released = False
        self.stopped = False
        self.reason = ''

    def begin(self, now, reason):
        if reason not in RECOVERABLE:
            raise ValueError('Nonrecoverable navigation fault: ' + reason)
        if self.active:
            return
        if self.episode >= self.max_episodes:
            raise RuntimeError('DATA_RECOVERY_LIMIT:' + reason)
        self.episode += 1
        self.active = True
        self.started = now
        self.healthy_since = None
        self.acknowledged = self.released = self.stopped = False
        self.reason = reason

    def observe(self, now, healthy):
        if not healthy:
            self.healthy_since = None
            self.released = False
        elif self.healthy_since is None:
            self.healthy_since = now
        return self.ready(now)

    def ready(self, now):
        return (self.active and not self.expired(now) and self.stopped and
                self.acknowledged and self.healthy_since is not None and
                now - self.healthy_since >= self.stable)

    def expired(self, now):
        return self.active and now - self.started >= self.timeout

    def finish(self):
        self.active = False
        self.healthy_since = None
        self.released = False


def matched_status(status, received_age, sent_ns, pose, now_ns):
    """Only a fresh exact-goal status can participate in the hold handshake."""
    try:
        session = status['session']
        if (status['schema'] != 'G1_NAVIGATION_WAIT_V1' or status['active'] is not True or
                not math.isfinite(received_age) or not 0 <= received_age <= .5 or
                type(status['stamp_ns']) is not int or
                not sent_ns <= status['stamp_ns'] <= now_ns or
                not isinstance(session, str) or len(session) != 32):
            return None
        int(session, 16)
        goal = status['goal']
        if (goal['frame'] != pose.header.frame_id or
                not all(math.isfinite(goal[key]) for key in ('x', 'y', 'qx', 'qy', 'qz', 'qw')) or
                abs(goal['x'] - pose.pose.position.x) >= 1e-4 or
                abs(goal['y'] - pose.pose.position.y) >= 1e-4 or
                any(abs(goal[key] - getattr(pose.pose.orientation, key[1:])) >= 1e-4
                    for key in ('qx', 'qy', 'qz', 'qw'))):
            return None
        return status
    except (KeyError, TypeError, ValueError, OverflowError, AttributeError):
        return None
