#!/usr/bin/env python3
"""Assemble hash-bound files for one recorded run; never import or execute them."""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import json
import subprocess


def safe_path(value):
    p = PurePosixPath(value)
    if p.is_absolute() or not p.parts or '..' in p.parts or '\\' in value:
        raise ValueError(f'Unsafe path: {value}')
    return p.as_posix()


def read_object(root, commit, path):
    return subprocess.check_output(['git', '-C', str(root), 'show', f'{commit}:{safe_path(path)}'])


def assemble(root, record, destination):
    refs = json.loads((root / 'docs/version_refs.json').read_text())
    version = refs[record['node']]
    commit = version.get('commit') or version['ref']
    snapshot = json.loads(read_object(root, commit, 'SNAPSHOT.json'))
    known = {x['path']: x for x in snapshot['files']}
    payload = {}
    manifest = []
    for binding in record.get('bindings', []):
        source = 'debian/' + safe_path(binding['source_relative'])
        name = safe_path(binding['restore_name'])
        if name in {'RESTORE_MANIFEST.json', 'recorded_run.json'}:
            raise ValueError('Reserved output filename')
        raw = read_object(root, commit, source)
        digest = hashlib.sha256(raw).hexdigest()
        if digest != binding['sha256'] or known[source]['sha256'] != digest:
            raise ValueError(f'Binding SHA mismatch: {source}')
        if name in payload and payload[name] != raw:
            raise ValueError(f'Conflicting restore filename: {name}')
        payload[name] = raw
        manifest.append(dict(binding, output=name, verified_sha256=digest))
    report_path = 'debian/' + safe_path(record['report_relative'])
    report_raw = read_object(root, commit, report_path)
    report_sha = hashlib.sha256(report_raw).hexdigest()
    if known[report_path]['sha256'] != report_sha:
        raise ValueError('Report SHA mismatch')
    payload['recorded_run.json'] = report_raw
    missing = list(record.get('unbound_local_dependency') or [])
    if record.get('core_hash_not_recorded'):
        missing.append('controller.py: source SHA was not recorded for this hardware run')
    if not record.get('bindings'):
        missing.append('executor/source binding unavailable: evidence only')
    result = {
        'schema': 'robot_run_restore.v1', 'node': record['node'], 'commit': commit,
        'report_relative': record['report_relative'], 'report_sha256': report_sha,
        'status': 'PARTIAL_SOURCE_IDENTITY' if missing else 'RECORDED_BINDINGS_VERIFIED',
        'unresolved': missing, 'files': manifest, 'motion_authorized': False,
        'execution_checked': False,
        'notice': 'Only recorded bindings restored. Review runtime arguments and hardware conditions separately.',
    }
    # Check every object before creating anything; do not leave a plausible partial assembly on hash errors.
    destination.mkdir(parents=True, exist_ok=False)
    for name, raw in payload.items():
        target = destination / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(raw)
    (destination / 'RESTORE_MANIFEST.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    records = json.loads((root / 'docs/debian_selection.json').read_text())['run_bindings']
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', nargs='?', help='Exact report_relative from --list')
    parser.add_argument('destination', nargs='?', type=Path)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--node')
    args = parser.parse_args()
    if args.list:
        for record in records:
            if not args.node or record['node'] == args.node:
                print(f"{record['node']}\t{record['report_relative']}")
        return
    if not args.report or not args.destination:
        parser.error('Supply a report and a new destination, or --list.')
    matches = [x for x in records if x['report_relative'] == args.report]
    if len(matches) != 1:
        parser.error('Report must match exactly one entry from --list.')
    destination = args.destination.resolve()
    if destination.exists():
        parser.error('Destination must not exist; no overwrite is allowed.')
    result = assemble(root, matches[0], destination)
    print(f"{result['status']}: {len(result['files'])} bound files; no code executed.")
    for gap in result['unresolved']:
        print('UNRESOLVED: ' + gap)


if __name__ == '__main__':
    main()
