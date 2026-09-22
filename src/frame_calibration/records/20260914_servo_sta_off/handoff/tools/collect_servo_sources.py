#!/usr/bin/env python3
"""Collect a PRIVATE field snapshot. Never imports project modules or controls hardware.

Default: print the collection plan only. --collect writes a NEW zip outside root.
Source files are never modified. No SSH, network, sudo, or robot calls are made.
Raw files may contain credentials: transfer privately; do not publish this zip.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import uuid
import zipfile
from datetime import datetime, timezone

SKIP_DIRS = {'.git', '.ssh', '.venv', 'venv', 'node_modules', '__pycache__',
             '.pytest_cache', '.mypy_cache', '.ruff_cache', 'build', 'dist'}
TEXT_SUFFIXES = {'.py', '.json', '.jsonl', '.md', '.txt', '.log', '.yaml', '.yml',
                 '.toml', '.ini', '.cfg', '.urdf', '.xacro', '.sdf', '.stl', '.obj', '.dae', '.sh', '.diff', '.patch'}
NAMES = {'SHA256SUMS', 'MANIFEST', 'Dockerfile', 'Makefile'}
REQUIRED = [
    'execute_tabletop_servo_field_20ms.py', 'execute_tabletop_servo_field_25ms.py',
    'execute_tabletop_servo_field_vaj20_v3.py', 'vaj3_spline.py',
    'execute_tabletop_servo.py', 'servo_common.py', 'sdk_session.py', 'robot_lock.py',
    'tabletop_servo_contract.py', 'arm_profiles.py', 'arm_profiles.v1.json',
    'probe_pulse_latency_stationary.py',
    'tabletop_pick_place_SERVO_CANDIDATE.json', 'tabletop_pick_place_SERVO_gui_review.json',
    'tabletop_servo_20260914_101418_793393004.json',
    'tabletop_servo_20260914_101507_112677088.json',
    'tabletop_servo_20260914_101711_112650029.json',
    'tabletop_servo_20260914_105707_209790145.json',
]
EXPECTED = {
    'tabletop_pick_place_SERVO_CANDIDATE.json': '71bc5148f5aba59aa91635e20905918f777c765c105bff6b29c10f670a1db338',
    'tabletop_pick_place_SERVO_gui_review.json': '02f94490a01c2332f50a9e5381edd19eea4fb164c600fe809a596b2fa6154b13',
}

def allowed(p: Path, include_pcaps: bool) -> bool:
    name = p.name.lower()
    if name.startswith('.env') or name in {'id_rsa', 'id_ed25519', 'authorized_keys', 'known_hosts'}:
        return False
    if p.suffix.lower() in {'.key', '.pem', '.p12', '.pfx'}:
        return False
    return (p.suffix.lower() in TEXT_SUFFIXES or p.name in NAMES or
            '.py.' in name or '.json.' in name or
            (include_pcaps and p.suffix.lower() in {'.pcap', '.pcapng'}))

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, default=Path.home()/'workspace/trackA_worlds_line/trackA_servo')
    ap.add_argument('--out-dir', type=Path, default=Path.home()/'servo_handoff_exports')
    ap.add_argument('--collect', action='store_true', help='Actually write a new private zip; otherwise plan only')
    ap.add_argument('--include-pcaps', action='store_true', help='Include sensitive packet captures inside root')
    ap.add_argument('--max-file-mb', type=int, default=128)
    ap.add_argument('--max-total-mb', type=int, default=768)
    args = ap.parse_args()
    root = args.root.expanduser().resolve()
    dest = args.out_dir.expanduser().resolve()
    if not root.is_dir():
        ap.error(f'Root does not exist: {root}')
    if dest == root or root in dest.parents:
        ap.error('--out-dir must be outside the source root')
    if args.max_file_mb <= 0 or args.max_total_mb <= 0:
        ap.error('Size limits must be positive')
    selected = []; excluded = []
    for current, dirs, names in os.walk(root, followlinks=False):
        current = Path(current)
        keep = []
        for name in sorted(dirs):
            p = current/name
            if name in SKIP_DIRS or p.is_symlink():
                excluded.append({'path':str(p.relative_to(root))+'/', 'reason':'excluded directory or symlink'})
            else:
                keep.append(name)
        dirs[:] = keep
        for name in sorted(names):
            p = current/name
            rel = p.relative_to(root).as_posix()
            st = p.lstat()
            if not stat.S_ISREG(st.st_mode) or p.is_symlink():
                excluded.append({'path':rel,'reason':'not a regular file (no symlink traversal)'})
                continue
            if not allowed(p, args.include_pcaps):
                excluded.append({'path':rel,'reason':'extension/name excluded; SDK binaries and secrets are not collected'})
                continue
            if st.st_size > args.max_file_mb*1024**2:
                excluded.append({'path':rel,'reason':'exceeds per-file size limit','bytes':st.st_size})
                continue
            selected.append((p,rel,st))
    selected.sort(key=lambda x:x[1])
    selected_names = {r for _,r,_ in selected}
    missing = [r for r in REQUIRED if r not in selected_names]
    total = sum(st.st_size for _,_,st in selected)
    if total > args.max_total_mb*1024**2:
        ap.error(f'{total/1024**2:.1f} MiB exceeds max total; review exclusions or raise explicit limit')
    print(f'ROOT: {root}\nFILES: {len(selected)}\nTOTAL_MIB: {total/1024**2:.2f}')
    print('MISSING_REQUIRED:', json.dumps(missing,ensure_ascii=False))
    print('Excluded item count:',len(excluded))
    if not args.collect:
        print('\nPLAN ONLY; no files written. Selected paths:')
        for _,r,st in selected:
            print(f'{st.st_size:>10}  {r}')
        print('\nTo collect, repeat with --collect. Packet captures require --include-pcaps.')
        return 0
    if not selected:
        ap.error('No eligible files to collect')
    print('PRIVATE SNAPSHOT: raw JSON/logs/pcaps may contain tokens. Do not publish.')
    dest.mkdir(parents=True, exist_ok=True)
    tag = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
    out = dest/f'Servo_FIELD_SOURCE_{tag}.zip'
    entries = []
    try:
        with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED,compresslevel=6,allowZip64=True) as z:
            for p,rel,initial in selected:
                # O_NOFOLLOW plus descriptor checks avoid following a replacement symlink.
                fd = os.open(p, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
                with os.fdopen(fd,'rb') as src:
                    before = os.fstat(src.fileno())
                    if not stat.S_ISREG(before.st_mode) or (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(initial.st_dev,initial.st_ino,initial.st_size,initial.st_mtime_ns):
                        raise RuntimeError(f'Source changed since plan: {rel}; stop active writers and retry')
                    h = hashlib.sha256(); count=0
                    with z.open('field_snapshot/'+rel,'w',force_zip64=True) as target:
                        while True:
                            block=src.read(1024**2)
                            if not block:break
                            target.write(block);h.update(block);count+=len(block)
                    after=os.fstat(src.fileno())
                    if (before.st_size,before.st_mtime_ns,before.st_ctime_ns)!=(after.st_size,after.st_mtime_ns,after.st_ctime_ns) or count != before.st_size:
                        raise RuntimeError(f'Source changed while copying: {rel}; snapshot rejected')
                actual=h.hexdigest()
                ent={'path':'field_snapshot/'+rel,'bytes':count,'sha256':actual,'source_mtime_ns':initial.st_mtime_ns}
                if rel in EXPECTED:
                    ent['historically_reported_sha256']=EXPECTED[rel]
                    ent['matches_historical_sha256']=(EXPECTED[rel]==actual)
                entries.append(ent)
            manifest={'schema':'servo_field_snapshot.v1','created_utc':datetime.now(timezone.utc).isoformat(),
                      'source_root':str(root),'raw_bytes_unmodified':True,'private_contains_possible_credentials':True,
                      'snapshot_status':'REQUIRED_FILES_PRESENT' if not missing else 'PARTIAL_MISSING_REQUIRED',
                      'required_missing':missing,'entries':entries,'excluded':excluded,
                      'limits':['Does not include external SDK/model/dependency directories','Does not certify robot state or safety','No network snapshot is automatically executed','Stop writers before collection; current contents only']}
            z.writestr('FIELD_MANIFEST.json',json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
            z.writestr('SHA256SUMS',''.join(f'{e["sha256"]}  {e["path"]}\n' for e in entries))
            z.writestr('README_PRIVATE_SNAPSHOT.txt','Original local files; may contain credentials. Offline audit only.\nDo not replace a Mac repository blindly; inspect FIELD_MANIFEST.json and compare file hashes.\nDo not run any robot SDK script or network change as part of import.\n')
    except BaseException:
        if out.exists():out.unlink()
        raise
    with zipfile.ZipFile(out) as z:
        bad=z.testzip()
        if bad:raise RuntimeError(f'ZIP integrity failed: {bad}')
    h=hashlib.sha256()
    with out.open('rb') as f:
        for b in iter(lambda:f.read(1024**2),b''):h.update(b)
    side=out.with_suffix(out.suffix+'.sha256')
    side.write_text(f'{h.hexdigest()}  {out.name}\n')
    print('CREATED:',out)
    print('SHA256:',h.hexdigest())
    print('STATUS:',manifest['snapshot_status'])
    return 0

if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, zipfile.BadZipFile) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        raise SystemExit(1)
