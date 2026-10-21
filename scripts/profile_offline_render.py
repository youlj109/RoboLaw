"""Measure existing offline capture calls without adding GPU synchronization.

Diagnostic launcher only. All rendering arguments after ``--`` are passed to
render_recorded_episode.py unchanged. The report is written from renderer cleanup
because SimulationAppContext may terminate Python before runpy returns.
"""
import argparse
import json
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile-output', type=Path, required=True)
    args, remaining = parser.parse_known_args()
    if remaining and remaining[0] == '--':
        remaining.pop(0)
    args.profile_output.parent.mkdir(parents=True, exist_ok=True)

    from kitchen_safety import vectorized_dataset
    from kitchen_safety.step_profiling import StepWallProfiler
    from time import perf_counter

    profile = StepWallProfiler()
    original_capture = vectorized_dataset.VectorizedCameraWriter._capture
    original_close = vectorized_dataset.VectorizedCameraWriter.close
    original_numpy = vectorized_dataset.numpy
    profile.wrap(vectorized_dataset, '_ensure_render_update', 'rtx_update')
    wrapped_cameras = set()

    def measured_numpy(value):
        category = ('rgb_readback' if len(value.shape) == 4 and value.shape[-1] == 3
                    else 'pose_and_intrinsics_readback')
        with profile.section(category, category):
            return original_numpy(value)

    vectorized_dataset.numpy = measured_numpy

    def measured_capture(writer, **kwargs):
        for name in writer.names:
            camera = writer.scene[name]
            if id(camera) not in wrapped_cameras:
                profile.wrap(camera, 'update', 'camera_update', 'camera_update.' + name)
                wrapped_cameras.add(id(camera))
        started = perf_counter()
        profile.begin('capture')
        try:
            return original_capture(writer, **kwargs)
        finally:
            profile.finish(perf_counter() - started)

    def measured_close(writer):
        try:
            return original_close(writer)
        finally:
            result = profile.report()
            result.update(
                scope='Natural host wall attribution inside original _capture; no additional CUDA synchronization.',
                interpretation='Readback waiting can include earlier GPU work; these are caller costs, not GPU kernel durations.',
                production_quota_eligible=False,
            )
            args.profile_output.write_text(json.dumps(result, indent=2) + '\n')

    vectorized_dataset.VectorizedCameraWriter._capture = measured_capture
    vectorized_dataset.VectorizedCameraWriter.close = measured_close
    sys.argv = [str(ROOT / 'scripts/render_recorded_episode.py'), *remaining]
    runpy.run_path(sys.argv[0], run_name='__main__')


if __name__ == '__main__':
    main()
