#!/usr/bin/env python3
"""Publish the exact pinned reproduction parts to the existing ModelScope dataset.

Run via `uv run --no-project --with modelscope-hub python ... --upload` after
`modelscope login`. Authentication is read by the SDK, never copied into files.
The default only verifies local files; --upload adds files without deleting any
remote data. Public remote verification streams every selected object's SHA256.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[3]
REPO_ID = "YouLinjing/Isaac-Arena-Kitchen-Safety"


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/extensions/replicator/reproduction_bundle_20261029.json")
    parser.add_argument("--folder", type=Path, default=ROOT / "exports/robolaw_reproduction_20261029_r13_v1")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--upload", action="store_true")
    modes.add_argument("--verify-public", action="store_true")
    parser.add_argument("--profile", default="all")
    parser.add_argument("--exclude-profile", help="Exclude another profile's parts, e.g. upload all except rerun")
    parser.add_argument("--workers", type=int, default=16, help="Concurrent SDK transfers; must be positive")
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("--workers must be positive")
    lock = json.loads(args.config.read_text())
    selected = set(lock["profiles"][args.profile])
    if args.exclude_profile:
        selected -= set(lock["profiles"][args.exclude_profile])
    if not selected:
        parser.error("No parts selected")
    records = []
    for part in lock["parts"]:
        if part["id"] in selected:
            records.extend([part, part["manifest"]])
    prefix = f"https://www.modelscope.cn/datasets/{REPO_ID}/resolve/master/robolaw/{lock['bundle_id']}/"
    for record in records:
        url = record["url"]
        if not url.startswith(prefix) or "/" in url[len(prefix):] or not url[len(prefix):]:
            raise ValueError("Unexpected publication destination")
        name = url[len(prefix):]
        if args.verify_public:
            digest = hashlib.sha256()
            length = 0
            with urllib.request.urlopen(url, timeout=120) as response:
                for chunk in iter(lambda: response.read(1024 * 1024), b""):
                    digest.update(chunk)
                    length += len(chunk)
            if length != record["bytes"] or digest.hexdigest() != record["sha256"]:
                raise ValueError(f"Public download mismatch: {name}")
        else:
            path = args.folder / name
            if path.is_symlink() or path.stat().st_size != record["bytes"] or sha(path) != record["sha256"]:
                raise ValueError(f"Local publication file mismatch: {name}")
        print(json.dumps({"verified": name, "public": args.verify_public}), flush=True)
    if args.upload:
        from modelscope_hub import HubApi

        api = HubApi()
        api.whoami()  # Fail before any upload if no valid login exists.
        if json.loads((args.folder / "bundle.json").read_text()) != lock:
            raise ValueError("Published bundle.json differs from pinned Git configuration")
        names = [r["url"][len(prefix):] for r in records] + [
            "bundle.json", "README.md", "SHA256SUMS", "archive_verification.json",
            "isolated_restored_input_validation.json",
        ]
        for name in names:
            if not (args.folder / name).is_file() or (args.folder / name).is_symlink():
                raise ValueError(f"Missing publication file: {name}")

        def progress(event):
            print(json.dumps({key: event.get(key) for key in (
                "event", "batch_index", "num_batches", "committed_files", "total_files",
                "committed_bytes", "uploaded_files", "uploaded_bytes", "total_bytes", "elapsed")}), flush=True)

        api.upload_folder(
            repo_id=REPO_ID, repo_type="dataset", folder_path=str(args.folder),
            path_in_repo="robolaw/" + lock["bundle_id"],
            allow_patterns=names, max_workers=args.workers, use_cache=True, disable_tqdm=True,
            sync_remote_repo=False, commit_message=f"Add reproduction bundle {lock['bundle_id']}",
            tracker_path=str(args.folder / (".ms_upload_cache_" + args.profile
                             + ("_without_" + args.exclude_profile if args.exclude_profile else ""))),
            progress_callback=progress,
        )
        print("Upload call completed. Run --verify-public before claiming verified public availability.")


if __name__ == "__main__":
    main()
