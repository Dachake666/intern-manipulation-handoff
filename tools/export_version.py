#!/usr/bin/env python3
"""Restore a named historical snapshot to a NEW directory without running it."""
from pathlib import Path, PurePosixPath
import argparse
import io
import hashlib
import json
import subprocess
import tarfile


def main():
    root=Path(__file__).resolve().parents[1]
    versions=json.loads((root/'docs/version_refs.json').read_text())
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('node',choices=sorted(versions))
    ap.add_argument('destination',type=Path)
    a=ap.parse_args()
    dest=a.destination.resolve()
    if dest.exists():ap.error('Destination must not already exist; no overwrite is allowed.')
    record=versions[a.node]
    ref=record.get('commit') or record['ref']
    raw=subprocess.check_output(['git','-C',str(root),'archive','--format=tar',ref])
    with tarfile.open(fileobj=io.BytesIO(raw)) as tf:
        members=tf.getmembers()
        for m in members:
            p=PurePosixPath(m.name)
            if p.is_absolute() or '..' in p.parts or not(m.isdir() or m.isfile()):
                raise ValueError('Unsafe snapshot member')
        dest.mkdir(parents=True)
        for m in members:
            if m.isdir():continue
            p=dest/m.name;p.parent.mkdir(parents=True,exist_ok=True)
            p.write_bytes(tf.extractfile(m).read())
    if (dest/'HANDOFF_MANIFEST.json').is_file():
        # Large calibration images are shipped once outside Git, with versioned hashes.
        manifest=json.loads((dest/'HANDOFF_MANIFEST.json').read_text())
        for item in manifest['files']:
            name=PurePosixPath(item['path'])
            if name.parts[:2] != ('data','handeye'):
                continue
            if name.is_absolute() or '..' in name.parts:
                raise ValueError('Unsafe data path')
            raw=(root/str(name)).read_bytes()
            if hashlib.sha256(raw).hexdigest()!=item['sha256']:
                raise ValueError('Calibration payload identity mismatch')
            p=dest/str(name);p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw)
    print(f'Restored {a.node} to {dest}. No code was executed.')
    print('Export contains no Git database. Keep the complete delivery to access other versions.')
    if a.node != 'B1':
        print('Historical snapshot for inspection; it is not a current installation or motion guide.')


if __name__=='__main__':
    main()
