#!/usr/bin/env python3
"""Regression checks for source/test identity and self-contained Git packaging."""
from pathlib import Path
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
import zipfile
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('delivery_builder', HERE/'build_package.py')
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class PackageToolsTests(unittest.TestCase):
    def setUp(self):
        runtime=HERE.parent/'.runtime';runtime.mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(prefix='package-tools-',dir=runtime)
        self.base=Path(self.temp.name)
        self.root=self.base/'repository';self.root.mkdir()

    def tearDown(self):
        self.temp.cleanup()

    def git(self,*args):
        return subprocess.check_output(['git','-C',str(self.root),*args],stderr=subprocess.STDOUT)

    def init_git(self):
        self.git('init','-b','test-package')
        self.git('config','user.name','Offline package test')
        self.git('config','user.email','offline-test@example.invalid')
        (self.root/'.gitignore').write_text('ignored.py\n')
        self.git('add','.gitignore')
        self.git('commit','-m','测试基线')
        self.git('tag','-a','handoff/current','-m','测试当前节点')

    def test_gitignored_delivery_file_cannot_disappear_from_b1(self):
        self.init_git()
        (self.root/'ignored.py').write_text('immutable = True\n')
        self.assertFalse(self.git('status','--porcelain').strip())
        manifest={'files':[{'path':'.gitignore'},{'path':'ignored.py'}]}
        with patch.object(builder,'check',return_value=manifest):
            with self.assertRaisesRegex(ValueError,'absent from HEAD'):
                builder.package(self.root,self.base/'delivery.zip')
        self.assertFalse((self.base/'delivery.zip').exists())

    def test_standalone_package_retains_recoverable_git_history(self):
        self.init_git()
        (self.root/'HANDOFF_MANIFEST.json').write_text('{}\n')
        (self.root/'SHA256SUMS').write_text('')
        sample=self.root/'data/handeye/sample.bin'
        sample.parent.mkdir(parents=True)
        sample.write_bytes(b'calibration sample fixture')
        self.git('add','HANDOFF_MANIFEST.json','SHA256SUMS','data/handeye/sample.bin')
        self.git('commit','-m','登记测试清单')
        self.git('tag','-f','-a','handoff/current','-m','测试交付')
        manifest={'files':[{'path':name,'sha256':builder.sha(self.root/name)}
                           for name in ['.gitignore','data/handeye/sample.bin']]}
        output=self.base/'delivery.zip'
        with patch.object(builder,'check',return_value=manifest):
            builder.package(self.root,output)
        extracted=self.base/'extracted'
        with zipfile.ZipFile(output) as archive:
            self.assertIsNone(archive.testzip())
            self.assertIn('robot_handoff/.git/HEAD',archive.namelist())
            archive.extractall(extracted)
        head=subprocess.check_output(['git','-C',str(extracted/'robot_handoff'),'rev-parse','HEAD'])
        self.assertEqual(head,self.git('rev-parse','HEAD'))
        cloned=self.base/'cloned'
        subprocess.run(['git','clone','--quiet','--no-local',str(self.root),str(cloned)],check=True)
        self.assertEqual((cloned/'data/handeye/sample.bin').read_bytes(),sample.read_bytes())

    def test_worktree_cannot_create_a_zip_without_its_git_database(self):
        self.init_git()
        worktree=self.base/'linked-worktree'
        self.git('worktree','add','-b','test-linked',str(worktree))
        self.assertTrue((worktree/'.git').is_file())
        with self.assertRaisesRegex(ValueError,'standalone .git'):
            builder.package(worktree,self.base/'delivery.zip')
        self.assertFalse((self.base/'delivery.zip').exists())

    def test_source_edit_during_tests_invalidates_the_record(self):
        tools=self.root/'tools';tools.mkdir()
        src=self.root/'src';src.mkdir()
        docs=self.root/'docs';docs.mkdir()
        (tools/'offline_checks.py').write_bytes((HERE/'offline_checks.py').read_bytes())
        fixture=src/'pick_place_coord/trajectories/traj_minimal_joint.json'
        fixture.parent.mkdir(parents=True);fixture.write_text('{}\n')
        source=src/'implementation.py';source.write_text('version = 1\n')
        # A synthetic test runner reports success but edits a selected source while running.
        # The actual recording wrapper must reject those results, not bless the new bytes.
        (src/'pytest.py').write_text('''from pathlib import Path
import sys
Path('implementation.py').write_text('version = 2\\n')
junit=next(x.split('=',1)[1] for x in sys.argv if x.startswith('--junitxml='))
Path(junit).write_text('<testsuites><testsuite tests="285" failures="0" errors="0" skipped="0" /></testsuites>')
print('285 synthetic tests passed')
''')
        names=['src/implementation.py','src/pytest.py','src/pick_place_coord/trajectories/traj_minimal_joint.json']
        (docs/'DELIVERY_SELECTION.json').write_text(json.dumps({'active':[{'path':x} for x in names]}))
        result=subprocess.run([sys.executable,'-B',str(tools/'offline_checks.py'),'--record'],
                              cwd=self.root,capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('changed during checks',result.stdout+result.stderr)
        report=json.loads((self.root/'verification.json').read_text())
        self.assertEqual(report['latest_attempt_status'],'FAILED')
        self.assertNotIn('checked_source_sha256',report)


if __name__=='__main__':
    unittest.main(verbosity=2)
