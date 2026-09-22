#!/usr/bin/env python3
"""Verify handoff files with Python's standard library. No robot code is imported."""
from pathlib import Path
import hashlib
import json
import sys


def main():
    root=Path(__file__).resolve().parents[1]
    manifest=json.loads((root/'HANDOFF_MANIFEST.json').read_text())
    bad=[]
    for item in manifest['files']:
        p=root/item['path']
        if not p.is_file() or p.is_symlink():
            bad.append(item['path']+': missing or symlink')
        elif hashlib.sha256(p.read_bytes()).hexdigest()!=item['sha256']:
            bad.append(item['path']+': SHA mismatch')
    if bad:
        print('\n'.join(bad));return 1
    print(f"PASS: {len(manifest['files'])} files match; no SDK imports or robot connection.")
    print('Git snapshot identities are in docs/version_refs.json; verification is not motion authorization.')
    return 0


if __name__=='__main__':
    sys.exit(main())
