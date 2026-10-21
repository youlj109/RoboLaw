"""Read-only environment probe; run with the existing arena_python.sh wrapper.

Only writes the requested new JSON output. Does not import Isaac Sim, initialize
an arena, modify source, or read credentials. This is a new helper, not an
existing target-project entry point.
"""
import argparse
import datetime
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

FILES = [
    "scripts/arena_python.sh",
    "scripts/collect_obstacle_library.py",
    "scripts/run_nonhuman_augmentation.py",
    "scripts/finish_default_human_collection.py",
    "scripts/collect_kitchen_vectorized.py",
    "kitchen_safety/vectorized_episode.py",
    "scripts/validate_collection.py",
    "scripts/audit_collection_clearance.py",
    "scripts/export_safety_lerobot_v3.py",
    "configs/augmentation_nonhuman10.json",
    "configs/augmentation_human7.json",
]


def command(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=20)
        return {"returncode": result.returncode,
                "stdout": result.stdout[:24000], "stderr": result.stderr[:2000]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"error": str(exc)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path("/workspace"))
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    root = args.project.resolve()
    if not root.is_dir():
        parser.error("Project directory does not exist: " + str(root))
    if args.out.exists():
        parser.error("Output exists; choose a new probe output path")
    report = {
        "schema": "arena-observer-environment-probe-1",
        "created_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "project": str(root), "python_executable": sys.executable,
        "python_version": sys.version, "platform": platform.platform(),
        "gpu_inventory": command([
            "nvidia-smi", "--query-gpu=name,driver_version,memory.total",
            "--format=csv,noheader"]),
        "git_head": command(["git", "-C", str(root), "rev-parse", "HEAD"]),
        "packages": {}, "source_files": [], "config_summaries": {},
        "limitations": [
            "No simulation, robot binding, sensor coverage or action audit performed",
            "CUDA tensor smoke test does not verify cuRobo extensions or rendering",
            "Robot joint and camera identities must be collected from running scene",
        ],
    }
    for name in ["torch", "numpy", "pyarrow", "lerobot", "isaacsim", "isaaclab",
                 "isaaclab-arena", "curobo", "python-fcl", "PyYAML"]:
        try:
            report["packages"][name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            report["packages"][name] = None
    for rel in FILES:
        path = root / rel
        record = {"path": rel, "exists": path.is_file()}
        if path.is_file():
            data = path.read_bytes()
            record.update(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
            if path.suffix == ".json":
                try:
                    obj = json.loads(data)
                    report["config_summaries"][rel] = {
                        "root_type": type(obj).__name__,
                        "top_level_keys": list(obj) if isinstance(obj, dict) else None,
                        "contains_human_default_mpfb": "human_default_mpfb" in data.decode(),
                    }
                except (ValueError, UnicodeDecodeError) as exc:
                    report["config_summaries"][rel] = {"error": str(exc)}
        report["source_files"].append(record)
    try:
        import torch
        probe = {"version": torch.__version__, "cuda_build": torch.version.cuda,
                 "cuda_available": torch.cuda.is_available()}
        if probe["cuda_available"]:
            probe.update(device_name=torch.cuda.get_device_name(0),
                         capability=list(torch.cuda.get_device_capability(0)),
                         compiled_archs=torch.cuda.get_arch_list())
            x = torch.ones((16, 16), device="cuda")
            y = x @ x
            torch.cuda.synchronize()
            probe["tensor_smoke_sum"] = float(y.sum())
            probe["tensor_smoke_passed"] = probe["tensor_smoke_sum"] == 4096.0
        report["torch_probe"] = probe
    except Exception as exc:
        report["torch_probe"] = {"error": type(exc).__name__ + ": " + str(exc)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation avoids overwriting a previous environment receipt.
    with args.out.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(str(args.out))


if __name__ == "__main__":
    main()
