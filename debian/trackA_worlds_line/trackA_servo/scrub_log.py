#!/usr/bin/env python3
"""日志脱敏: 归档/外发前去掉控制器 token 等敏感串。

背景(20260727): 真机日志的登录响应里带完整控制器 token, 曾有 2 份日志带着
token 被提交进 git 并推到远端。私有仓库也不该存凭据 —— 归档前必须过一遍。

用法:
    python3 scrub_log.py <日志或目录> [...]      # 就地脱敏
    python3 scrub_log.py --check <路径> [...]    # 只检查不改, 有命中则退出码 1
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

# 需要脱敏的模式: (正则, 替换)
REDACTED = "<REDACTED>"
# 注意: 模式必须【排除已脱敏的值】, 否则 --check 会把处理过的文件反复报命中。
_VAL = r'(?!<REDACTED>")[^"]*'
PATTERNS = [
    (re.compile(r'("token"\s*:\s*")' + _VAL + r'(")'), r'\1<REDACTED>\2'),
    (re.compile(r'("password"\s*:\s*")' + _VAL + r'(")'), r'\1<REDACTED>\2'),
    (re.compile(r'("secret"\s*:\s*")' + _VAL + r'(")'), r'\1<REDACTED>\2'),
    (re.compile(r'(Authorization:\s*Bearer\s+)(?!<REDACTED>)\S+'), r'\1<REDACTED>'),
]
SUFFIXES = {".log", ".txt", ".json", ".md"}


def scan(path: Path):
    text = path.read_text(encoding="utf-8", errors="replace")
    total = 0
    for pat, rep in PATTERNS:
        text, n = pat.subn(rep, text)
        total += n
    return text, total


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    check_only = "--check" in argv
    if check_only:
        argv.remove("--check")
    if not argv:
        print(__doc__)
        return 2

    targets = []
    for a in argv:
        p = Path(a)
        if p.is_dir():
            targets += [f for f in p.rglob("*") if f.suffix in SUFFIXES]
        elif p.is_file():
            targets.append(p)

    hits = 0
    for f in targets:
        try:
            new, n = scan(f)
        except Exception as exc:      # noqa: BLE001 — 跳过读不了的文件
            print(f"  跳过 {f}: {exc}")
            continue
        if n:
            hits += n
            print(f"  {'[命中]' if check_only else '[已脱敏]'} {f}: {n} 处")
            if not check_only:
                f.write_text(new, encoding="utf-8")
    if not hits:
        print("未发现敏感串 ✓")
        return 0
    print(f"\n共 {hits} 处" + ("(仅检查, 未修改)" if check_only else "(已就地脱敏)"))
    return 1 if check_only else 0


if __name__ == "__main__":
    sys.exit(main())
