"""Nested host wall-time attribution without extra simulation or GPU synchronization."""
from collections import defaultdict
from contextlib import contextmanager
from functools import wraps
from time import perf_counter_ns


class StepWallProfiler:
    def __init__(self, clock=perf_counter_ns):
        self.clock = clock
        self.active = False
        self.phase = None
        self.stack = []
        self.total_seconds = 0.0
        self.step_count = 0
        self.categories_ns = defaultdict(int)
        self.hooks = {}
        self.phases = {}

    def begin(self, phase):
        if self.active:
            raise RuntimeError('Step timing is already active')
        self.active = True
        self.phase = str(phase)
        self.phases.setdefault(self.phase, {
            'total_seconds': 0.0, 'step_count': 0, 'categories_ns': defaultdict(int)})

    def finish(self, total_seconds):
        if not self.active or self.stack:
            raise RuntimeError('Step timing must finish outside all active sections')
        self.total_seconds += total_seconds
        self.step_count += 1
        phase = self.phases[self.phase]
        phase['total_seconds'] += total_seconds
        phase['step_count'] += 1
        self.active = False
        self.phase = None

    @contextmanager
    def section(self, category, hook=None):
        if not self.active:
            yield
            return
        hook = hook or category
        frame = {'start': self.clock(), 'children_ns': 0}
        self.stack.append(frame)
        try:
            yield
        finally:
            elapsed = self.clock() - frame['start']
            self.stack.pop()
            exclusive = elapsed - frame['children_ns']
            if self.stack:
                self.stack[-1]['children_ns'] += elapsed
            self.categories_ns[category] += exclusive
            self.phases[self.phase]['categories_ns'][category] += exclusive
            entry = self.hooks.setdefault(hook, {
                'category': category, 'count': 0, 'inclusive_ns': 0, 'exclusive_ns': 0})
            entry['count'] += 1
            entry['inclusive_ns'] += elapsed
            entry['exclusive_ns'] += exclusive

    def wrap(self, obj, method_name, category, hook=None):
        original = getattr(obj, method_name)

        @wraps(original)
        def measured(*args, **kwargs):
            if not self.active:
                return original(*args, **kwargs)
            with self.section(category, hook or method_name):
                return original(*args, **kwargs)

        setattr(obj, method_name, measured)
        return original

    @staticmethod
    def _partition(total_seconds, categories_ns):
        seconds = {name: value / 1e9 for name, value in categories_ns.items()}
        residual = total_seconds - sum(seconds.values())
        seconds['other'] = max(0.0, residual)
        return {
            'total_seconds': total_seconds,
            'categories_seconds': seconds,
            'categories_percent': {
                name: 100.0 * value / total_seconds if total_seconds else 0.0
                for name, value in seconds.items()},
            'raw_residual_seconds': residual,
        }

    def report(self):
        report = self._partition(self.total_seconds, self.categories_ns)
        report['step_count'] = self.step_count
        report['hooks'] = {
            name: {
                'category': entry['category'], 'count': entry['count'],
                'inclusive_seconds': entry['inclusive_ns'] / 1e9,
                'exclusive_seconds': entry['exclusive_ns'] / 1e9,
            } for name, entry in self.hooks.items()}
        report['per_phase'] = {
            name: dict(self._partition(entry['total_seconds'], entry['categories_ns']),
                       step_count=entry['step_count'])
            for name, entry in self.phases.items()}
        return report
