"""Avoid CPU pxr ABI loading before Kit in the actual snapshot execution path."""
import ast
from pathlib import Path

import pytest

SOURCE = Path(__file__).resolve().parents[3]/'scripts/extensions/replicator/snapshot_bowl_cooked_geometry.py'


def assert_import_order(source):
    tree = ast.parse(source)
    parent = {child: node for node in ast.walk(tree) for child in ast.iter_child_nodes(node)}
    def ancestors(node):
        while node in parent:
            node = parent[node]
            yield node
    main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
    calls = [node for node in ast.walk(main) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'preflight']
    assert len(calls) == 2
    for call in calls:
        path = list(ancestors(call))
        standalone_cpu = any(isinstance(node, ast.If) and ast.unparse(node.test) == 'args.dry_run' for node in path)
        inside_kit = any(isinstance(node, ast.With) and any(isinstance(item.context_expr, ast.Call) and
            isinstance(item.context_expr.func, ast.Name) and item.context_expr.func.id == 'SimulationAppContext' for item in node.items) for node in path)
        assert standalone_cpu or inside_kit, 'Actual USD preflight outside Kit context'
    for node in tree.body:
        if isinstance(node, ast.ImportFrom):
            assert not node.module.startswith(('pxr','omni','isaaclab'))
        if isinstance(node, ast.Import):
            assert all(not alias.name.startswith(('pxr','omni','isaaclab')) for alias in node.names)


def test_actual_usd_preflight_after_kit_cpu_dryrun_independent():
    assert_import_order(SOURCE.read_text())


def test_premature_usd_read_negative_case_rejected():
    source = SOURCE.read_text()
    source = source.replace('        with SimulationAppContext(runtime_args):\n            cpu = preflight(args)',
        '        cpu = preflight(args)\n        with SimulationAppContext(runtime_args):')
    with pytest.raises(AssertionError, match='outside Kit'):
        assert_import_order(source)
