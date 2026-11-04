#!/usr/bin/env python3
"""Run the independent miter validator and save a versioned report.

The validator (`miter_validator.py`) is never edited here; this wrapper only
adds provenance so that every run has a version, a label, the test parameters
and the hash of the validator that produced it.

The original baseline `results/reports/initial-validation.txt` is immutable;
this script refuses to overwrite it.
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import subprocess
import sys


def main() -> int:
    here = pathlib.Path(__file__).resolve().parent
    root = here.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", required=True, help="algorithm/model version, e.g. v0")
    parser.add_argument("--label", default="run", help="run label, e.g. default-matrix")
    parser.add_argument(
        "--param",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="test parameter, repeatable; recorded in the report header",
    )
    parser.add_argument(
        "--out-dir",
        default=str(root / "results" / "reports"),
        help="output directory (default: results/reports)",
    )
    args = parser.parse_args()

    validator = here / "miter_validator.py"
    if not validator.is_file():
        sys.stderr.write(f"missing validator: {validator}\n")
        return 2

    out_dir = pathlib.Path(args.out_dir)
    out = out_dir / f"{args.version}-{args.label}.txt"
    if out.resolve() == (root / "results" / "reports" / "initial-validation.txt").resolve():
        sys.stderr.write("refusing to overwrite the immutable baseline initial-validation.txt\n")
        return 2

    digest = hashlib.sha256(validator.read_bytes()).hexdigest()
    proc = subprocess.run(
        [sys.executable, str(validator)],
        cwd=str(here),
        capture_output=True,
        text=True,
        check=False,
    )

    header = [
        "# Universal miter validation report",
        f"version: {args.version}",
        f"label: {args.label}",
        "validator: prototype-python/miter_validator.py",
        f"validator_sha256: {digest}",
        f"python: {sys.version.split()[0]}",
        "parameters:",
    ]
    if args.param:
        header.extend(f"  {item}" for item in args.param)
    else:
        header.append("  (none)")
    body = "\n".join(header) + "\n\n" + proc.stdout
    if proc.stderr:
        body += "\n[stderr]\n" + proc.stderr

    out_dir.mkdir(parents=True, exist_ok=True)
    out.write_text(body, encoding="utf-8")
    sys.stdout.write(proc.stdout)
    sys.stderr.write(f"report: {out}\n")
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
