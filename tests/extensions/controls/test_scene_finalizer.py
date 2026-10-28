import pytest
from scripts.extensions.controls.finalize_reviewed_scene import completed_case


def test_finalizer_waits_for_physical_gates_and_never_promotes_failures():
    assert completed_case({'cases':[]},'scene') is None
    assert completed_case({'cases':[dict(scene='scene',state='running')]},'scene') is None
    row=dict(scene='scene',state='passed',episode='recorded')
    assert completed_case({'cases':[row]},'scene')==row
    for state in ('failed','cancelled_before_variant_collection'):
        with pytest.raises(RuntimeError):completed_case({'cases':[dict(scene='scene',state=state)]},'scene')
    with pytest.raises(ValueError):completed_case({'cases':[row,row]},'scene')
