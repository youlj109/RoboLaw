#!/usr/bin/env python3
"""Restore the four remaining assigned kitchens from the pinned Arena registry.

Existing G-kitchen assets and their manifest are immutable inputs. This command
records separate manifests and never claims that restored assets passed physics.
"""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path

from pxr import Usd

from scripts.extensions.replicator.restore_assets import BASE, DEST, ROOT, fetch

LAYOUTS = ("l_island", "l_shape", "peninsula", "u_shape")
REGISTRY = ROOT / "third_party/IsaacLab-Arena/isaaclab_arena/assets/background_library.py"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layouts", nargs="+", choices=LAYOUTS, default=list(LAYOUTS))
    parser.add_argument("--output", type=Path, required=True,
                        help="New report directory; existing evidence is never overwritten")
    parser.add_argument("--workers", type=int, choices=range(1, 7), default=3)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source = REGISTRY.read_text()
    starts = {}
    for layout in args.layouts:
        relative = f"Isaac/Environments/replicator_kitchen/kitchen_{layout}.usda"
        if f'"replicator_kitchen_{layout}"' not in source or f'/kitchen_{layout}.usda"' not in source:
            raise ValueError(f"Kitchen is not declared in the local pinned registry: {layout}")
        starts[layout] = BASE + relative

    expected = {}
    for path in [ROOT / "configs/extensions/replicator/external_assets.json",
                 ROOT / "configs/extensions/replicator/additional_kitchen_assets.json"]:
        if path.exists():
            for row in json.loads(path.read_text())["files"]:
                if row["url"] in expected and expected[row["url"]] != row["sha256"]:
                    raise ValueError(f"Conflicting pinned source hashes: {row['url']}")
                expected[row["url"]] = row["sha256"]
    report = dict(schema_version=1, complete=False, layouts=args.layouts,
                  source_registry=str(REGISTRY.relative_to(ROOT)), registry_sha256=sha(REGISTRY),
                  root_urls=starts, files=[], failures=[], dependencies={},
                  scope="Pinned asset bytes and USD composition only; no rendering or physical validation")
    pending, seen, futures = set(starts.values()), set(), {}

    def checked_fetch(url):
        row, dependencies = fetch(url)
        if url in expected and row["sha256"] != expected[url]:
            raise ValueError(f"Asset differs from pinned bytes: {row['path']}")
        return row, dependencies

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        while pending or futures:
            ordered = sorted(pending - seen, key=lambda u: (not u.endswith((".usd", ".usda", ".usdc", ".mdl")), u))
            for url in ordered[:max(0, args.workers - len(futures))]:
                seen.add(url)
                pending.discard(url)
                futures[pool.submit(checked_fetch, url)] = url
            pending -= seen
            if not futures:
                break
            completed, _ = concurrent.futures.wait(futures, return_when=concurrent.futures.FIRST_COMPLETED)
            for future in completed:
                url = futures.pop(future)
                try:
                    row, dependencies = future.result()
                    report["files"].append(row)
                    report["dependencies"][url] = sorted(set(dependencies))
                    pending.update(set(dependencies) - seen)
                    print("ASSET", row["path"], flush=True)
                except Exception as error:
                    report["failures"].append(dict(url=url, error=str(error)))
                    print("FAILED", url, str(error), flush=True)
            report["pending"] = len(pending) + len(futures)
            save(args.output / "restoration.json", report)
    report["composition"] = []
    if not report["failures"]:
        for layout, url in starts.items():
            path = DEST / url[len(BASE):]
            stage = Usd.Stage.Open(str(path))
            errors = [str(error) for error in stage.GetCompositionErrors()] if stage else ["Stage did not open"]
            report["composition"].append(dict(layout=layout, path=str(path.relative_to(ROOT)), errors=errors))
    report["files"].sort(key=lambda row: row["url"])
    report["complete"] = (not report["failures"] and len(report["composition"]) == len(starts)
                          and all(not row["errors"] for row in report["composition"]))
    save(args.output / "restoration.json", report)
    if report["complete"]:
        # A subset restore cannot erase a previously complete manifest for other layouts.
        manifest = ROOT / "configs/extensions/replicator/additional_kitchen_assets.json"
        if not manifest.exists() or set(args.layouts) == set(LAYOUTS):
            save(manifest, report)
        else:
            save(args.output / "subset_manifest.json", report)
    print(json.dumps(dict(complete=report["complete"], files=len(report["files"]), failures=report["failures"])), flush=True)
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
