#!/usr/bin/env python3
"""Build a self-contained delivery from an explicit selection and its Git history."""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import json
import os
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[1]
GENERATED = {'HANDOFF_MANIFEST.json', 'SHA256SUMS'}


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def excluded(path):
    return (any(x in {'.git', '.runtime', '__pycache__', '.pytest_cache', 'restored'} or
                x.startswith('.venv') for x in path.parts) or path.name == '.DS_Store' or path.suffix == '.pyc')


def active_files(root):
    selection = json.loads((root / 'docs/DELIVERY_SELECTION.json').read_text())
    entries = selection['active']
    paths = [x['path'] for x in entries]
    if len(paths) != len(set(paths)):
        raise ValueError('Duplicate paths in delivery selection')
    for name in paths:
        p = PurePosixPath(name)
        if p.is_absolute() or '..' in p.parts or '\\' in name or excluded(p) or name in GENERATED:
            raise ValueError(f'Unsafe/reserved selection path: {name}')
        if not (root / name).is_file() or (root / name).is_symlink():
            raise ValueError(f'Missing/non-regular selection: {name}')
    for item in entries:
        if item.get('immutable_sha256') and sha(root / item['path']) != item['immutable_sha256']:
            raise ValueError('Immutable evidence/source changed; preserve original and add a new version: '+item['path'])
    actual = {p.relative_to(root).as_posix() for p in root.rglob('*')
              if (p.is_file() or p.is_symlink()) and not excluded(p.relative_to(root))}
    unknown = actual - set(paths) - GENERATED
    if unknown:
        raise ValueError(f'Unclassified delivery files; update DELIVERY_SELECTION or move outputs into .runtime: {sorted(unknown)}')
    return sorted(entries, key=lambda x: x['path'])


def verify_test_identity(root):
    result = json.loads((root / 'verification.json').read_text())
    if result.get('latest_attempt_status') != 'PASS':
        raise ValueError('The latest offline test attempt is not a completed PASS')
    suite = result['selected_offline_tests']
    if suite['failures'] or suite['errors'] or suite['skipped'] or suite['passed'] < 279:
        raise ValueError('Selected offline suite is incomplete or unsuccessful')
    expected = result['checked_source_sha256']
    paths = {x['path'] for x in active_files(root) if x['path'].startswith('src/') and
             Path(x['path']).suffix != '.md'}
    if paths != set(expected):
        raise ValueError('Source selection changed since offline check; rerun tools/offline_checks.py --record')
    for name, digest in expected.items():
        if sha(root / name) != digest:
            raise ValueError(f'Source changed since offline check: {name}')


def refresh(root):
    verify_test_identity(root)
    source_map = root / 'docs/SOURCE_MAP.json'
    old = {x['path']: x for x in json.loads(source_map.read_text())}
    current = []
    for item in active_files(root):
        name = item['path']
        if not name.startswith(('src/', 'extensions/', 'sdk/', 'data/', 'evidence/mpc_review/')):
            continue
        row = old.get(name, {'path': name, 'origin': 'maintained_source:' + name})
        row['sha256'] = sha(root / name)
        if row.get('imported_sha256'):
            row['changed_from_import'] = row['sha256'] != row['imported_sha256']
        current.append(row)
    source_map.write_text(json.dumps(current, ensure_ascii=False, indent=2) + '\n')
    files = []
    for item in active_files(root):
        path = root / item['path']
        files.append(dict(item, bytes=path.stat().st_size, sha256=sha(path)))
    manifest = {'schema': 'robot_handoff.v2', 'files': files,
                'snapshot_map': 'docs/version_refs.json',
                'selection': 'docs/DELIVERY_SELECTION.json',
                'source_map': 'docs/SOURCE_MAP.json',
                'checksum_scope': 'Working payload; Git history is validated separately and covered by ZIP SHA.'}
    (root / 'HANDOFF_MANIFEST.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    paths = sorted([x['path'] for x in files] + ['HANDOFF_MANIFEST.json'])
    (root / 'SHA256SUMS').write_text(''.join(f'{sha(root/name)}  {name}\n' for name in paths))


def check(root):
    entries = active_files(root)
    manifest = json.loads((root / 'HANDOFF_MANIFEST.json').read_text())
    if {x['path'] for x in entries} != {x['path'] for x in manifest['files']}:
        raise ValueError('Manifest and delivery selection differ')
    for item in manifest['files']:
        if sha(root / item['path']) != item['sha256']:
            raise ValueError(f"SHA mismatch: {item['path']}")
    expected_sums = {x['path']: x['sha256'] for x in manifest['files']}
    expected_sums['HANDOFF_MANIFEST.json'] = sha(root / 'HANDOFF_MANIFEST.json')
    parsed = dict((line.split('  ', 1)[1], line.split('  ', 1)[0])
                  for line in (root/'SHA256SUMS').read_text().splitlines())
    if parsed != expected_sums:
        raise ValueError('SHA256SUMS does not match manifest')
    verify_test_identity(root)
    refs = json.loads((root / 'docs/version_refs.json').read_text())
    for node, item in refs.items():
        actual = subprocess.check_output(['git', '-C', str(root), 'rev-parse', item['ref']+'^{commit}']).decode().strip()
        if item.get('commit') and actual != item['commit']:
            raise ValueError(f'Historical ref changed: {node}')
    subprocess.run(['git', '-C', str(root), 'fsck', '--full'], check=True, capture_output=True)
    return manifest


def package(root, output):
    if not (root / '.git').is_dir():
        raise ValueError('A standalone .git directory is required; Git worktrees are not self-contained deliveries')
    common = subprocess.check_output(['git','-C',str(root),'rev-parse','--git-common-dir']).decode().strip()
    if (root / common).resolve() != (root / '.git').resolve():
        raise ValueError('Git object storage must be inside the delivery .git directory')
    manifest = check(root)
    if root == output or root in output.parents:
        raise ValueError('ZIP destination must be outside the delivery root')
    if subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain', '--untracked-files=all']).strip():
        raise ValueError('Commit reviewed changes before packaging; Git working tree must be clean')
    head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD']).strip()
    current = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'handoff/current^{commit}']).strip()
    if head != current:
        raise ValueError('Update handoff/current to HEAD after committing the delivery')
    tracked = set(subprocess.check_output(['git','-C',str(root),'ls-tree','-r','--name-only','-z','HEAD']).decode().split('\0'))
    tracked.discard('')
    expected = {x['path'] for x in manifest['files'] if not x['path'].startswith('data/handeye/')} | GENERATED
    if expected - tracked:
        raise ValueError('Selected files are absent from HEAD and would be lost in B1 export: '+str(sorted(expected-tracked)))
    if tracked - expected:
        raise ValueError('HEAD contains files outside the delivery selection: '+str(sorted(tracked-expected)))
    # Comparing each blob covers ignored/assume-unchanged files as well as the clean-status check.
    for name in sorted(expected):
        raw = subprocess.check_output(['git','-C',str(root),'show','HEAD:'+name])
        if hashlib.sha256(raw).hexdigest() != sha(root/name):
            raise ValueError('Working bytes differ from HEAD: '+name)
    subprocess.run(['git', '-C', str(root), 'repack', '-a', '-d'], check=True, capture_output=True)
    paths = sorted([x['path'] for x in manifest['files']] + sorted(GENERATED))
    for p in (root / '.git').rglob('*'):
        if not p.is_file() or p.is_symlink():
            continue
        rel = p.relative_to(root)
        if rel.parts[1] in {'hooks', 'logs'} or p.name.endswith('.lock'):
            continue
        if p.name in {'COMMIT_EDITMSG', 'ORIG_HEAD', 'FETCH_HEAD'}:
            continue
        if rel.parts[1] == 'objects' and rel.parts[2] not in {'pack', 'info'}:
            continue
        paths.append(rel.as_posix())
    output.parent.mkdir(parents=True, exist_ok=True)
    temp = output.with_name(output.name + '.tmp')
    try:
        with zipfile.ZipFile(temp, 'w', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for name in sorted(paths):
                archive.write(root / name, 'robot_handoff/' + name)
        with zipfile.ZipFile(temp) as archive:
            if archive.testzip() is not None:
                raise ValueError('ZIP CRC failure')
            for item in manifest['files']:
                if hashlib.sha256(archive.read('robot_handoff/'+item['path'])).hexdigest() != item['sha256']:
                    raise ValueError('ZIP payload mismatch: '+item['path'])
        os.replace(temp, output)
    finally:
        if temp.exists():
            temp.unlink()
    digest = sha(output)
    Path(str(output)+'.sha256').write_text(f'{digest}  {output.name}\n')
    print(json.dumps({'archive': output.name, 'sha256': digest, 'bytes': output.stat().st_size,
                      'payload_files': len(manifest['files']), 'commit': head.decode()}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true')
    group.add_argument('--refresh-manifest', action='store_true')
    group.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.refresh_manifest:
        refresh(ROOT)
        print('Manifest refreshed. Review and commit changes before building a ZIP.')
    elif args.check:
        result = check(ROOT)
        print(f"PASS: {len(result['files'])} payload files, test source identities, and Git refs.")
    else:
        package(ROOT, args.output.resolve())


if __name__ == '__main__':
    main()
