#!/usr/bin/env python3
"""Verify the frozen P0 source baseline without ROS, network, or file writes."""

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def fingerprint(path):
    if path.is_symlink():
        return {"kind": "symlink", "target": os.readlink(str(path))}
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"kind": "file", "sha256": digest.hexdigest(),
            "size": path.stat().st_size}


def git(*args):
    return subprocess.check_output(["git"] + list(args), cwd=str(ROOT))


def relative_path(name):
    path = Path(name)
    if path.is_absolute() or ".." in path.parts or not path.parts:
        raise ValueError("Unsafe baseline path: " + name)
    return ROOT / path


def verify(local):
    baseline = json.loads((ROOT / "docs/migration/baseline.json").read_text())
    completion = baseline["p0_completion"]
    manifest_name = completion["source_manifest"]
    manifest = json.loads(relative_path(manifest_name).read_text())
    errors = []
    checks = []

    def check(condition, message):
        if not condition:
            errors.append(message)

    def check_files(entries, label):
        for name, expected in entries.items():
            path = relative_path(name)
            try:
                actual = fingerprint(path)
            except OSError as exc:
                errors.append("{}: {}: {}".format(label, name, exc))
                continue
            check(actual == expected, label + " differs: " + name)
        checks.append("{}: {} files".format(label, len(entries)))

    check_files(manifest["files"], "source SHA-256 / symlinks")
    missing = completion["unavailable_tracked_files"]
    for name in missing:
        path = relative_path(name)
        check(not path.exists() and not path.is_symlink(),
              "Previously unavailable path now exists: " + name)
    candidates = set(git("ls-files", "--cached", "--others",
                         "--exclude-standard", "-z").decode().split("\0"))
    candidates.discard("")
    candidates.discard(manifest_name)
    candidates.difference_update(missing)
    candidates.update(completion["local_evidence"])
    check(candidates == set(manifest["files"]),
          "Source inventory differs; added={}, removed={}".format(
              sorted(candidates - set(manifest["files"])),
              sorted(set(manifest["files"]) - candidates)))

    with (ROOT / "docs/migration/source-comparison.tsv").open() as stream:
        rows = list(csv.DictReader(stream, delimiter="\t"))
    for row in rows:
        expected = row["initial_local_blob_sha1"]
        name = row["current_location"]
        if not expected or (name.startswith("backup/") and not local):
            continue
        path = relative_path(name)
        if not path.is_file():
            errors.append("Original business source missing: " + name)
            continue
        content = path.read_bytes()
        actual = hashlib.sha1(b"blob " + str(len(content)).encode()
                              + b"\0" + content).hexdigest()
        check(actual == expected, "Original business source differs: " + name)
    checks.append("original comparison table: {} rows".format(len(rows)))

    readme = (ROOT / "README.md").read_text()
    for url in baseline["preserved_readme_image_urls"]:
        check(url in readme, "Original README image missing: " + url)
    checks.append("original README images: 3")

    for name in ["README.md", "script/g1_cruise.sh", "script/g1_waypoints.sh",
                 "src/g1_task_interfaces/action/ExecuteMission.action",
                 "src/g1_task_manager/package.xml"]:
        result = subprocess.run(["git", "check-ignore", "--no-index", "-q", name],
                                cwd=str(ROOT), check=False)
        check(result.returncode == 1, "Source is ignored or Git failed: " + name)
    for name in ["doc/p0-check.md", "docs/PC2_MIGRATION_PLAN.md",
                 "docs/CODE_STRUCTURE.md", "docs/migration/baseline.json",
                 "docs/migration/source-comparison.tsv", manifest_name,
                 "build/p0-check", "install/p0-check", "log/p0-check",
                 "runtime/p0-check", "backup/p0-check", "script/local.env",
                 "script/auto_start_pose.env", "task_data/p0-check.yaml",
                 "maps/p0-check.pcd", "maps/p0-check.pgm", "maps/p0-check.yaml"]:
        result = subprocess.run(["git", "check-ignore", "--no-index", "-q", name],
                                cwd=str(ROOT), check=False)
        check(result.returncode == 0, "Local data is not ignored: " + name)
    checks.append("source / untracked docs and local data ignore boundaries")

    if local:
        snapshot = relative_path(completion["local_snapshot"])
        record = json.loads((snapshot / "manifest.json").read_text())
        source_manifest = relative_path(manifest_name)
        check(fingerprint(source_manifest)["sha256"]
              == record["source_manifest_sha256"],
              "Source manifest differs from frozen snapshot")
        check_files(record["preserved_files"], "local data / historical copies")
        for name, expected in record["archives"].items():
            check(fingerprint(snapshot / name) == expected,
                  "Snapshot archive differs: " + name)
        checks.append("snapshot archives: {}".format(len(record["archives"])))
    else:
        checks.append("local archives / site data: skipped (use --local)")

    for message in checks:
        print("CHECK " + message)
    for message in errors:
        print("FAIL " + message, file=sys.stderr)
    if errors:
        print("P0 verification failed: {} issue(s)".format(len(errors)))
        return 1
    print("P0 verification passed; no ROS nodes started or files changed.")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local", action="store_true",
                        help="also check local snapshots, site data and archived copies")
    args = parser.parse_args()
    try:
        return verify(args.local)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        print("P0 verification failed: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
