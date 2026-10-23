import importlib.util
from pathlib import Path

PATH = Path(__file__).resolve().parents[3] / 'scripts/extensions/controls/summarize_acceptance.py'
SPEC = importlib.util.spec_from_file_location('controls_acceptance', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_missing_pairs_never_pass_vacuously():
    assert not all(MODULE.paired_gates({}).values())
    assert not MODULE.paired_gates({'runs': []})['complete_12_unique_pairs']


def test_duplicate_pair_does_not_fill_quota():
    row = {'seed': 42, 'variant': 'base', 'success': True, 'validation': {'valid': True}}
    assert not MODULE.paired_gates({'runs': [row] * 12})['complete_12_unique_pairs']
