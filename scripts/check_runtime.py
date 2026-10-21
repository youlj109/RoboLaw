"""Check installed dependencies and GPU visibility without starting Kit."""
from importlib.metadata import version
import json
import platform
from pathlib import Path

import fcl
import h5py
import numpy as np
import scipy
import torch


def main():
    expected = {
        "isaaclab-arena": "0.3.0",
        "isaaclab": "17.0.2",
        "isaacsim": "6.1.0.0",
        "torch": "2.11.0+cu128",
        "warp-lang": "1.16.0",
        "numpy": "2.3.1",
        "python-fcl": "0.7.0.11",
        "nvidia-curobo": "0.7.7.post1.dev5",
    }
    installed = {name: version(name) for name in expected}
    mismatches = {name: {"expected": wanted, "actual": installed[name]}
                  for name, wanted in expected.items() if installed[name] != wanted}
    # Exercise the native FCL binding, rather than only inspecting its version.
    a = fcl.CollisionObject(fcl.Sphere(.1), fcl.Transform(np.zeros(3)))
    b = fcl.CollisionObject(fcl.Sphere(.1), fcl.Transform(np.array([1., 0., 0.])))
    clearance = fcl.distance(a, b, fcl.DistanceRequest(), fcl.DistanceResult())
    assert abs(clearance - .8) < 1e-9, clearance
    assert torch.cuda.is_available(), "CUDA is unavailable inside the container"
    report = {
        "python": platform.python_version(),
        "packages": installed,
        "package_mismatches": mismatches,
        "gpus": [{"name": torch.cuda.get_device_name(i),
                  "capability": list(torch.cuda.get_device_capability(i))}
                 for i in range(torch.cuda.device_count())],
        "fcl_sphere_clearance_m": clearance,
        "h5py": h5py.__version__,
        "scipy": scipy.__version__,
        "kit_started": False,
    }
    root = Path(__file__).resolve().parents[1]
    (root / "reports").mkdir(exist_ok=True)
    (root / "reports/runtime_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    assert not mismatches, "Installed versions differ from the validated simulation stack"


if __name__ == "__main__":
    main()
