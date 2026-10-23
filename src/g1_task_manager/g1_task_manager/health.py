"""Pure validation shared by preflight and in-motion monitoring."""
import json
import math

MAX_STAMP_NS = (1 << 63) - 1


def validate_limits(max_tf_age_sec, max_fitness, max_fitness_age_sec):
    for name, value in (('max_tf_age_sec', max_tf_age_sec),
                        ('max_fitness', max_fitness),
                        ('max_fitness_age_sec', max_fitness_age_sec)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('{} must be finite and positive'.format(name))


def health_reason(fitness, fitness_age, tf_stamp_ns, now_ns,
                  max_tf_age_sec=0.5, max_fitness=2.0,
                  max_fitness_age_sec=5.0):
    if fitness is None or fitness_age is None:
        return '尚未收到/localization_fitness'
    if not math.isfinite(fitness) or fitness < 0 or fitness > max_fitness:
        return '定位质量不满足要求：fitness={}'.format(fitness)
    if not math.isfinite(fitness_age) or fitness_age < 0 or fitness_age > max_fitness_age_sec:
        return 'localization_fitness已过期/时间无效：{}s'.format(fitness_age)
    return tf_reason(tf_stamp_ns, now_ns, max_tf_age_sec)


def tf_reason(tf_stamp_ns, now_ns, max_tf_age_sec=0.5):
    if (type(tf_stamp_ns) is not int or not 0 < tf_stamp_ns <= MAX_STAMP_NS
            or type(now_ns) is not int or not 0 < now_ns <= MAX_STAMP_NS):
        return 'TF时间戳无效'
    age = (now_ns - tf_stamp_ns) / 1e9
    if age < -0.1:
        return 'TF时间戳位于未来：{:.3f}s'.format(age)
    if age > max_tf_age_sec:
        return 'TF已过期：{:.3f}s'.format(age)
    return ''


def localization_status_reason(status, received_age_sec, now_ns,
                               max_success_age_sec=3.0):
    """A current TF/fitness cannot replace a confirmed, fresh successful scan."""
    if not isinstance(status, dict):
        return '尚未收到有效/g1_relocalization/status'
    if (received_age_sec is None or not math.isfinite(received_age_sec)
            or received_age_sec < 0 or received_age_sec > 1.0):
        return '定位状态接收已过期'
    if type(now_ns) is not int or not 0 < now_ns <= MAX_STAMP_NS:
        return '当前ROS时间无效'
    if (isinstance(max_success_age_sec, bool)
            or not isinstance(max_success_age_sec, (int, float))
            or not math.isfinite(max_success_age_sec) or max_success_age_sec <= 0):
        return '定位成功扫描年龄限制无效'
    for field, limit in (('stamp_ns', 1.0),
                         ('last_success_stamp_ns', min(3.0, max_success_age_sec))):
        stamp = status.get(field)
        # The publisher and ROS clock use int64 nanoseconds. Check the range
        # before division so malformed huge JSON integers fail closed as well.
        if type(stamp) is not int or not 0 < stamp <= MAX_STAMP_NS:
            return '定位状态{}无效'.format(field)
        age = (now_ns - stamp) / 1e9
        if age < 0 or age > limit:
            return '定位状态{}过期/未来：{:.3f}s'.format(field, age)
    if status['last_success_stamp_ns'] > status['stamp_ns']:
        return '成功扫描时间晚于定位状态发布时间'
    if status.get('valid') is not True or status.get('state') not in ('LOCALIZED', 'TRACKING'):
        return '定位未确认：{} {}'.format(status.get('state'), status.get('reason', ''))
    return ''


def parse_localization_status(data):
    try:
        status = json.loads(data)
    except (TypeError, ValueError):
        return None
    return status if isinstance(status, dict) else None

