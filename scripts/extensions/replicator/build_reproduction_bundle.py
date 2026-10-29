#!/usr/bin/env python3
"""Build immutable, file-hashed restore archives from an explicit reviewed inventory.

No simulator or credentials are needed. Only regular files under approved roots
are accepted; source files are streamed without loading large recordings in RAM.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import subprocess
import tarfile


def dump(path: Path, data: object) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")


def digest(path: Path) -> str:
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def relative(value: str) -> str:
    p = PurePosixPath(value)
    if p.is_absolute() or str(p) != value or ".." in p.parts or not p.parts:
        raise ValueError(f"Unsafe path: {value!r}")
    return value


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--inventory", type=Path, required=True)
    ap.add_argument("--root", type=Path, default=Path.cwd())
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--lock", type=Path, required=True)
    ap.add_argument("--max-part-bytes", type=int, default=1024**3)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--gzip-threads", type=int, default=2)
    args = ap.parse_args()
    if min(args.max_part_bytes, args.workers, args.gzip_threads) <= 0:
        ap.error("part size and worker/thread counts must be positive")
    spec = json.loads(args.inventory.read_text())
    root = args.root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    roots = [relative(x) for x in spec["allowed_roots"]]
    seen = set()
    groups: dict[str, list[dict]] = {}
    for item in spec["files"]:
        rel = relative(item["path"])
        if rel in seen:
            raise ValueError(f"Duplicate path: {rel}")
        seen.add(rel)
        if not any(rel == r or rel.startswith(r + "/") for r in roots):
            raise ValueError(f"Path outside approved roots: {rel}")
        p = root / rel
        if p.resolve() != p or not stat.S_ISREG(p.lstat().st_mode):
            raise ValueError(f"Not a regular, symlink-free source: {rel}")
        if p.is_relative_to(output):
            raise ValueError("Bundle cannot include itself")
        s = p.stat()
        groups.setdefault(item["group"], []).append({
            "path": rel, "bytes": s.st_size, "mtime_ns": s.st_mtime_ns,
            "mode": stat.S_IMODE(s.st_mode),
        })
    tasks = []
    for group, files in sorted(groups.items()):
        if not group.replace("-", "").isalnum():
            raise ValueError(f"Unsafe group: {group}")
        batch, size, number = [], 0, 0
        for item in sorted(files, key=lambda x: x["path"]):
            if batch and size + item["bytes"] > args.max_part_bytes:
                number += 1
                tasks.append((f"{group}-{number:03d}", group, batch))
                batch, size = [], 0
            batch.append(item)
            size += item["bytes"]
        if batch:
            tasks.append((f"{group}-{number+1:03d}", group, batch))

    def build(task):
        part_id, group, entries = task
        archive = output / f"{part_id}.tar.gz"
        manifest_path = output / f"{part_id}.manifest.json"
        metadata_path = output / f"{part_id}.build.json"
        inventory_sha = hashlib.sha256(json.dumps(entries, sort_keys=True).encode()).hexdigest()
        if metadata_path.exists():
            old = json.loads(metadata_path.read_text())
            if old["inventory_sha256"] != inventory_sha:
                raise ValueError(f"Source inventory changed for completed {part_id}")
            for p, record in [(archive, old["part"]), (manifest_path, old["part"]["manifest"])]:
                if p.stat().st_size != record["bytes"] or digest(p) != record["sha256"]:
                    raise ValueError(f"Completed output changed: {p}")
            for record in json.loads(manifest_path.read_text())["files"]:
                if digest(root / record["path"]) != record["sha256"]:
                    raise ValueError(f"Source content changed for completed {part_id}: {record['path']}")
            print(f"REUSED {part_id}", flush=True)
            return old["part"]
        if archive.exists() or manifest_path.exists():
            raise FileExistsError(f"Uncommitted output exists: {part_id}")
        temporary = archive.with_suffix(".partial")
        files = []
        with temporary.open("wb") as target:
            compressor = subprocess.Popen(
                ["pigz", "-1", "-n", "-p", str(args.gzip_threads)],
                stdin=subprocess.PIPE, stdout=target,
            )
            try:
                with tarfile.open(fileobj=compressor.stdin, mode="w|", format=tarfile.PAX_FORMAT) as tar:
                    for entry in entries:
                        p = root / entry["path"]
                        before = p.stat()
                        if (before.st_size, before.st_mtime_ns) != (entry["bytes"], entry["mtime_ns"]):
                            raise RuntimeError(f"Source changed before packaging: {p}")
                        h = hashlib.sha256()

                        class HashedReader:
                            def __init__(self, f):
                                self.f = f

                            def read(self, count=-1):
                                chunk = self.f.read(count)
                                h.update(chunk)
                                return chunk

                        info = tarfile.TarInfo(entry["path"])
                        info.size = entry["bytes"]
                        info.mode = entry["mode"]
                        info.mtime = 0
                        with p.open("rb") as source:
                            tar.addfile(info, HashedReader(source))
                        after = p.stat()
                        if (after.st_size, after.st_mtime_ns, after.st_ino) != (before.st_size, before.st_mtime_ns, before.st_ino):
                            raise RuntimeError(f"Source changed while packaging: {p}")
                        files.append({"path": entry["path"], "bytes": entry["bytes"],
                                      "sha256": h.hexdigest(), "mode": entry["mode"]})
                compressor.stdin.close()
                if compressor.wait() != 0:
                    raise RuntimeError(f"Compression failed for {part_id}")
            except BaseException:
                compressor.kill()
                compressor.wait()
                temporary.unlink(missing_ok=True)
                raise
        temporary.rename(archive)
        dump(manifest_path, {"kind": "robolaw_reproduction_part", "bundle_id": spec["bundle_id"],
                             "part_id": part_id, "files": files})
        base = spec["base_url"].rstrip("/") + "/"
        part = {"id": part_id, "group": group, "format": "tar.gz", "url": base + archive.name,
                "bytes": archive.stat().st_size, "sha256": digest(archive),
                "files": len(files), "uncompressed_bytes": sum(f["bytes"] for f in files),
                "manifest": {"url": base + manifest_path.name, "bytes": manifest_path.stat().st_size,
                             "sha256": digest(manifest_path)}}
        dump(metadata_path, {"inventory_sha256": inventory_sha, "part": part})
        print(f"BUILT {part_id}: {len(files)} files, {part['bytes']} compressed bytes", flush=True)
        return part

    print(json.dumps({"selected_files": len(seen), "source_bytes": sum(x["bytes"] for v in groups.values() for x in v),
                      "parts": len(tasks), "groups": {g: len(v) for g, v in groups.items()}}), flush=True)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        parts = list(pool.map(build, tasks))
    profiles = {name: [p["id"] for p in parts if p["group"] in names]
                for name, names in spec["profile_groups"].items()}
    lock = {"schema_version": 1, "kind": "robolaw_reproduction_bundle", "bundle_id": spec["bundle_id"],
            "source_git_commit": spec["source_git_commit"], "allowed_roots": roots,
            "default_profile": "rerun", "profiles": profiles, "parts": parts,
            "scope": spec.get("scope"), "exclusions": spec.get("exclusions", [])}
    dump(args.lock, lock)
    dump(output / "bundle.json", lock)
    print(json.dumps({"lock": str(args.lock), "compressed_bytes": sum(p["bytes"] for p in parts), "parts": len(parts)}), flush=True)


if __name__ == "__main__":
    main()
