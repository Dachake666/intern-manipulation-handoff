#!/usr/bin/env python3
"""Verify historical source objects and every recorded run assembly without execution."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import tempfile

from restore_run import assemble


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', action='store_true')
    args = parser.parse_args()
    versions = json.loads((root/'docs/version_refs.json').read_text())
    checked = {}
    def raw(commit, path):
        key = (commit, path)
        if key not in checked:
            checked[key] = subprocess.check_output(['git','-C',str(root),'show',commit+':'+path])
        return checked[key]
    nodes = []
    for node, ref in versions.items():
        if node in {'B1', 'PRE_PRUNE'}:
            continue
        commit = ref['commit']
        actual = subprocess.check_output(['git','-C',str(root),'rev-parse',ref['ref']+'^{commit}']).decode().strip()
        if actual != commit:
            raise ValueError('Historical tag changed: '+node)
        snapshot = json.loads(raw(commit,'SNAPSHOT.json'))
        for item in snapshot['files']:
            if hashlib.sha256(raw(commit,item['path'])).hexdigest() != item['sha256']:
                raise ValueError('Historical SHA mismatch: '+item['path'])
        nodes.append({'node':node,'files':len(snapshot['files']),'status':'PASS'})
    policy = json.loads((root/'docs/DELIVERY_SELECTION.json').read_text())
    for item in policy['historical']:
        node = versions[item['restore_node']]
        if hashlib.sha256(raw(node['commit'],item['path'])).hexdigest() != item['baseline_sha256']:
            raise ValueError('Historical omitted file missing: '+item['path'])
    bindings = json.loads((root/'docs/debian_selection.json').read_text())['run_bindings']
    runtime = root/'.runtime'; runtime.mkdir(exist_ok=True)
    assemblies = []
    with tempfile.TemporaryDirectory(prefix='history-check-',dir=runtime) as folder:
        for index, item in enumerate(bindings):
            result = assemble(root,item,Path(folder)/str(index))
            assemblies.append({key:result[key] for key in ['node','report_relative','status','unresolved']})
    result = {'schema':'robot_history_check.v1','status':'PASS','nodes':nodes,
              'history_only_files_verified':len(policy['historical']),
              'run_assemblies':assemblies,'motion_started':False,
              'scope':'Git objects and hash-bound restoration only; source gaps remain explicit.'}
    if args.record:
        target=root/'evidence/handoff_checks/history_checks.json'
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(f"PASS: {len(nodes)} historical nodes; {len(policy['historical'])} omitted files; {len(assemblies)} run assemblies.")
    print('Unbound dependencies remain partial; no robot code was executed.')


if __name__ == '__main__':
    main()
