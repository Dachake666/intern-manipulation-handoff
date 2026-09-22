#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import secrets
from pathlib import Path

from .contracts import canonical_bytes, sha256_bytes, validate_document
from .preflight import report_hash


def main(argv=None):
    ap = argparse.ArgumentParser(description="为 PASS 的 no-motion 报告生成一次性人工批准令牌")
    ap.add_argument("report"); ap.add_argument("--operator", required=True); ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    report = json.loads(Path(a.report).read_text(encoding="utf-8"))
    validate_document(report, "preflight_report.v1")
    if report["verdict"] != "PASS":
        raise SystemExit("报告不是 PASS，拒绝批准")
    expected = report_hash(report)
    if expected != report["approval"]["report_sha256"]:
        raise SystemExit("报告哈希不匹配，输入已变化")
    approved_at = dt.datetime.now(dt.timezone.utc).isoformat()
    nonce = secrets.token_hex(16)
    token = sha256_bytes(canonical_bytes({"report_sha256": expected,
                                         "operator": a.operator,
                                         "approved_at": approved_at, "nonce": nonce}))
    payload = {"schema_version": "approval_token.v1", "report_sha256": expected,
               "token": token, "operator": a.operator, "approved_at": approved_at,
               "expires_on_change": True}
    Path(a.out).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"批准令牌已绑定 report {expected}: {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
