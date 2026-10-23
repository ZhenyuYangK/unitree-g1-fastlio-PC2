"""Pure validation shared by preflight and in-motion monitoring."""
import math


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
    if tf_stamp_ns <= 0:
        return 'TF时间戳无效'
    age = (now_ns - tf_stamp_ns) / 1e9
    if age < -0.1:
        return 'TF时间戳位于未来：{:.3f}s'.format(age)
    if age > max_tf_age_sec:
        return 'TF已过期：{:.3f}s'.format(age)
    return ''
