"""Root-scoped risk collection for the isolated multi-environment experiment.

Call install_risk_isolation636() before constructing collectors. Each workflow
keeps its own task, collector, evaluator, policy, actor drivers and episode clock.
The coordinator still owns the single World.step and collision-group isolation.
"""
from dataclasses import dataclass
from functools import wraps


def _path(value):
    value = str(value).rstrip('/')
    if not value.startswith('/') or value in ('', '/World') or '..' in value.split('/'):
        raise ValueError('a specific absolute environment/shared prim root is required')
    return value


def under(path, root):
    path = str(path)
    return path == root or path.startswith(root + '/')


@dataclass(frozen=True)
class EnvironmentScope636:
    root: str
    shared_roots: tuple = ()

    def __post_init__(self):
        root = _path(self.root)
        shared = tuple(dict.fromkeys(_path(p) for p in self.shared_roots))
        if any(under(root, p) for p in shared):
            raise ValueError('shared roots cannot contain the environment root')
        shared = tuple(p for p in shared if not under(p, root))
        object.__setattr__(self, 'root', root)
        object.__setattr__(self, 'shared_roots', shared)

    def allows(self, path):
        return any(under(path, root) for root in (self.root, *self.shared_roots))

    def keeps_ancestor(self, path):
        return any(under(root, str(path)) for root in (self.root, *self.shared_roots))

    def contact_allowed(self, report):
        own = False
        for side in (0, 1):
            paths = [str(report.get(key) or '') for key in (f'actor{side}', f'collider{side}')]
            paths = [p for p in paths if p]
            if not paths or not all(self.allows(p) for p in paths):
                return False
            own |= any(under(p, self.root) for p in paths)
        return own

    def metadata(self):
        return {'version': 'environment-risk-scope636-v1', 'environment_root': self.root,
                'shared_physics_roots': list(self.shared_roots),
                'scope': 'one registered task subtree plus explicitly shared physics roots',
                'other_environment_geometry_and_contacts_excluded': True}


def scope_for_task636(task):
    return EnvironmentScope636(getattr(task, 'root_prim_path', ''),
                               tuple(getattr(task, 'shared_physics_roots636', ()) or ()))


def bind_collector_scope636(collector, task):
    if getattr(collector, '_isolation636_closed', False):
        raise RuntimeError('finalized environment collector cannot be reused')
    scope = scope_for_task636(task)
    old = getattr(collector, '_environment_scope636', None)
    if old is not None:
        if old != scope:
            raise RuntimeError('a collector cannot switch environment roots')
        online = getattr(collector, '_online_iso', None)
        if online is not None:
            online.metadata.setdefault('environment_isolation636', scope.metadata())
        return old
    for registry in ('robots', 'objects', 'fixtures'):
        for name, entity in (getattr(task, registry, {}) or {}).items():
            root = getattr(entity, 'prim_path', None) or getattr(entity, '_prim_path', None)
            if root is None or not scope.allows(str(root)):
                raise ValueError(f'{registry}/{name} is not inside its environment or explicit shared roots')
    collector._environment_scope636 = scope
    collector._pending_contact_reports = [r for r in collector._pending_contact_reports if scope.contact_allowed(r)]
    data = collector._data
    if 'pre_recording_contact_reports' in data:
        data['pre_recording_contact_reports'] = [r for r in data['pre_recording_contact_reports']
                                                if scope.contact_allowed(r)]
    data['environment_isolation636'] = scope.metadata()
    online = getattr(collector, '_online_iso', None)
    if online is not None:
        online.metadata['environment_isolation636'] = scope.metadata()
    return scope


def contact_report_belongs636(collector, report):
    if getattr(collector, '_isolation636_closed', False):
        return False
    scope = getattr(collector, '_environment_scope636', None)
    # A constructor can precede task binding during setup. These pending setup
    # events are pruned on bind, before initial metadata or frame collection.
    return scope is None or scope.contact_allowed(report)


def _subtree(prim):
    # Real USD traversal stays in its native iterator and uses the same
    # active/defined/loaded/non-abstract predicate as Stage.Traverse().
    # Keep the small fallback only for the no-USD FakePrim unit tests.
    try:
        from pxr import Usd
    except ImportError:
        Usd = None
    if Usd is not None and isinstance(prim, Usd.Prim):
        yield from Usd.PrimRange(prim)
        return
    stack = [prim]
    while stack:
        item = stack.pop()
        yield item
        stack.extend(reversed(list(item.GetChildren())))


class _ScopedRootLayer636:
    def __init__(self, stage, scope):
        self.stage, self.scope = stage, scope

    def __getattr__(self, name):
        return getattr(self.stage.GetRootLayer(), name)

    def Export(self, path, *args, **kwargs):
        """Copy and prune authored roots, preserving references and transforms."""
        from pxr import Sdf, Usd, UsdUtils
        source = self.stage.GetRootLayer()
        layer = Sdf.Layer.CreateAnonymous('environment636.usda')
        layer.TransferContent(source)
        # Copying a file-backed root to an anonymous/output layer changes the
        # anchor of relative asset references. Resolve them against the source.
        UsdUtils.ModifyAssetPaths(layer, lambda asset: Sdf.ComputeAssetPathRelativeToLayer(source, asset))
        isolated = Usd.Stage.Open(layer)
        pending = list(isolated.GetPseudoRoot().GetChildren())
        while pending:
            prim = pending.pop()
            prim_path = str(prim.GetPath())
            if self.scope.allows(prim_path):
                continue
            if self.scope.keeps_ancestor(prim_path):
                pending.extend(prim.GetChildren())
                continue
            isolated.RemovePrim(prim_path)
            # A branch inherited through a reference/sublayer cannot be deleted
            # from this layer. An inactive override removes it from composition.
            remaining = isolated.GetPrimAtPath(prim_path)
            if remaining.IsValid():
                isolated.OverridePrim(prim_path).SetActive(False)
        for prim in isolated.Traverse():
            p = str(prim.GetPath())
            if not self.scope.allows(p) and not self.scope.keeps_ancestor(p):
                raise RuntimeError('foreign environment remained in isolated USD export: ' + p)
        return layer.Export(path, *args, **kwargs)


class ScopedStage636:
    """Read facade: root-only traversal; the live USD stage is never modified."""
    def __init__(self, stage, scope):
        self.stage, self.scope = stage, scope

    def __getattr__(self, name):
        return getattr(self.stage, name)

    def GetPrimAtPath(self, path):
        if not self.scope.allows(str(path)) and not self.scope.keeps_ancestor(str(path)):
            raise ValueError('out-of-environment prim query: ' + str(path))
        return self.stage.GetPrimAtPath(path)

    def Traverse(self):
        seen = set()
        for root in (self.scope.root, *self.scope.shared_roots):
            prim = self.stage.GetPrimAtPath(root)
            if not prim.IsValid():
                raise ValueError('missing environment/shared root: ' + root)
            for child in _subtree(prim):
                path = str(child.GetPath())
                if path not in seen:
                    seen.add(path)
                    yield child

    def GetRootLayer(self):
        return _ScopedRootLayer636(self.stage, self.scope)


def scoped_stage636(task, stage):
    return ScopedStage636(stage, scope_for_task636(task)) if stage is not None else None


def finalize_collector636(collector):
    """Call after this environment is saved, before leaving the active barrier."""
    if collector is None or getattr(collector, '_isolation636_closed', False):
        return
    collector._isolation636_closed = True
    subscription = collector._contact_report_sub
    collector._contact_report_sub = None
    release = getattr(subscription, 'unsubscribe', None)
    if callable(release):
        release()
    # Native subscription bindings without unsubscribe use reference lifetime.
    del subscription
    collector._pending_contact_reports.clear()
    collector._data.setdefault('environment_isolation636', {}).update(
        finalized=True, contact_subscription_released=True)


def install_risk_isolation636():
    """Install task binding guards on the experimental collector copy only."""
    from . import physx_collector
    if not getattr(physx_collector, 'RISK_ISOLATION636_HOOKS', False):
        raise RuntimeError('install the experimental physx_collector.py copy with 636 hooks first')
    cls = physx_collector.PhysXDataCollector
    if getattr(cls, '_isolation636_installed', False):
        return cls
    for method_name in ('collect_step', '_collect_distances', '_collect_contact_data',
                        'capture_initial_pre_action_state', 'prime_intervention_state',
                        'update_intervention_stop_state'):
        original = getattr(cls, method_name)

        def wrapper(self, task, *args, _original=original, **kwargs):
            bind_collector_scope636(self, task)
            return _original(self, task, *args, **kwargs)

        setattr(cls, method_name, wraps(original)(wrapper))
    cls._isolation636_installed = True
    return cls
