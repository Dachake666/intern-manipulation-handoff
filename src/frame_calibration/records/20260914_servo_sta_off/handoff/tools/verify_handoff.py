#!/usr/bin/env python3
"""Verify package SHA256SUMS; no project imports, writes, or network access."""
import argparse
import hashlib
from pathlib import Path
import sys

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root',nargs='?',type=Path,default=Path(__file__).resolve().parents[1])
    a=ap.parse_args();root=a.root.resolve();mf=root/'SHA256SUMS'
    if not mf.is_file():ap.error('SHA256SUMS not found')
    ok=0;bad=[]
    for line in mf.read_text(encoding='utf-8').splitlines():
        if not line.strip():continue
        try:
            expected,rel=line.split('  ',1);p=root/rel
            resolved=p.resolve()
            if len(expected)!=64 or Path(rel).is_absolute() or root not in resolved.parents or p.is_symlink():
                raise ValueError('invalid entry path or digest')
            h=hashlib.sha256()
            with p.open('rb')as f:
                for b in iter(lambda:f.read(1024**2),b''):h.update(b)
            if h.hexdigest()!=expected:raise ValueError('SHA256 mismatch')
            ok+=1
        except (ValueError,OSError) as e:bad.append(f'{line}: {e}')
    print(f'FILES_VERIFIED: {ok}\nERRORS: {len(bad)}')
    for s in bad:print(s)
    print('This checks integrity, NOT robot safety or authenticity of historical approvals.')
    return 1 if bad else 0
if __name__=='__main__':sys.exit(main())
