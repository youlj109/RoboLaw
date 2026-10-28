#!/usr/bin/env python3
"""Restore the assigned YCB banana from the pinned Arena registry source.

This keeps the USD bytes and relative references unchanged. Its manifest is
separate from the existing G-kitchen manifest; no simulation or GPU is started.
"""
from __future__ import annotations

import hashlib
import json
import re
import time
from pathlib import Path
from urllib.parse import urljoin
from urllib.request import Request, urlopen

from pxr import Sdf

ROOT = Path(__file__).resolve().parents[3]
BASE = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1/"
SOURCE = BASE + "Isaac/IsaacLab/Arena/assets/object_library/srl_robolab_assets/objects/ycb/banana.usd"
DESTINATION = ROOT / "assets/extensions/replicator/source"
MANIFEST = ROOT / "configs/extensions/replicator/banana_assets.json"
REGISTRY = ROOT / "third_party/IsaacLab-Arena/isaaclab_arena/assets/object_library.py"
BUILTINS = {"OmniPBR.mdl", "OmniGlass.mdl", "OmniSurface.mdl", "OmniSurfaceLite.mdl"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def restore(url: str) -> dict:
    if not url.startswith(BASE):
        raise ValueError(f"Dependency outside the pinned asset source: {url}")
    relative = url[len(BASE):]
    if ".." in Path(relative).parts:
        raise ValueError(f"Unsafe asset path: {relative}")
    path = DESTINATION / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        temporary = path.with_suffix(path.suffix + ".part")
        for attempt in range(2):
            try:
                with urlopen(Request(url, headers={"User-Agent": "RoboLaw assigned-task asset restore"}),
                             timeout=30) as response, temporary.open("wb") as output:
                    expected = response.headers.get("Content-Length")
                    received = 0
                    while block := response.read(1024 * 1024):
                        output.write(block)
                        received += len(block)
                if expected is not None and received != int(expected):
                    raise RuntimeError(f"Incomplete asset download: {received}/{expected}")
                temporary.replace(path)
                break
            except Exception:
                if attempt == 1:
                    raise
                time.sleep(1)
    references = []
    if path.suffix in (".usd", ".usda", ".usdc"):
        layer = Sdf.Layer.FindOrOpen(str(path))
        if not layer:
            raise RuntimeError(f"Invalid USD layer: {path}")
        # assetInfo identifiers express provenance, not composition dependencies.
        content = re.sub(r"asset identifier = @[^@]+@", "", layer.ExportToString())
        references = sorted(set(layer.GetExternalReferences()) | set(re.findall(r"@([^@]+)@", content)))
    dependencies = [{"authored_reference": reference,
                     "url": urljoin(url, reference),
                     "builtin": reference in BUILTINS} for reference in references]
    return {"path": str(path.relative_to(ROOT)), "url": url, "bytes": path.stat().st_size,
            "sha256": sha256(path), "dependencies": dependencies}


def main() -> int:
    manifest = {
        "schema_version": 1,
        "registry_name": "banana_ycb_robolab",
        "registry_source": str(REGISTRY.relative_to(ROOT)),
        "registry_source_sha256": sha256(REGISTRY),
        "source_url": SOURCE,
        "source_asset": str((DESTINATION / SOURCE[len(BASE):]).relative_to(ROOT)),
        "scope": "Original YCB banana and referenced files; asset availability is not physical task validation.",
        "files": [], "failures": [], "complete": False,
    }
    old_files = {}
    if MANIFEST.exists():
        old_files = {item["url"]: item["sha256"] for item in json.loads(MANIFEST.read_text())["files"]}
    pending, seen = [SOURCE], set()
    while pending:
        url = pending.pop(0)
        if url in seen:
            continue
        seen.add(url)
        try:
            record = restore(url)
            if url in old_files and old_files[url] != record["sha256"]:
                raise RuntimeError(f"Asset differs from recorded SHA256: {url}")
            manifest["files"].append(record)
            pending.extend(item["url"] for item in record["dependencies"] if not item["builtin"])
            print("ASSET", record["path"], flush=True)
        except Exception as error:
            manifest["failures"].append({"url": url, "error": str(error)})
            print("FAILED", url, str(error), flush=True)
    manifest["complete"] = not manifest["failures"]
    # A failed recheck must not erase the expected hashes used by the next run.
    target = MANIFEST
    if manifest["failures"] and MANIFEST.exists():
        target = ROOT / "reports/replicator_preparation/banana_asset_restore_failure.json"
        target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(manifest, indent=2) + "\n")
    return int(not manifest["complete"])


if __name__ == "__main__":
    raise SystemExit(main())
