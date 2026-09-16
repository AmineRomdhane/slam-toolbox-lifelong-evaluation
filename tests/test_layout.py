"""Publication links, authoritative inputs and relocated source integrity; no ROS runtime."""
import hashlib,json,re,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'benchmark/tools'))
from layout import verify_tool_sources

class LayoutTests(unittest.TestCase):
    def test_active_source_manifest(self):
        verify_tool_sources(ROOT)

    def test_authoritative_yaml_hashes(self):
        m=json.loads((ROOT/'benchmark/manifests/layout.json').read_text())
        for old,new in m['moves'].items():
            if new.startswith(('benchmark/config/','benchmark/routes/')):
                self.assertEqual(hashlib.sha256((ROOT/new).read_bytes()).hexdigest(),m['historical_source_sha256'][old])
        self.assertFalse((ROOT/'src/slam_eval_route_controller/routes').exists())
        self.assertEqual(len(list((ROOT/'benchmark/routes').glob('*.yaml'))),3)

    def test_numerical_core_unchanged(self):
        m=json.loads((ROOT/'benchmark/manifests/layout.json').read_text())
        for old,new in [('experiments/static_v0_r1/evaluate.py','benchmark/tools/slam/evaluate.py'),('experiments/reference_v0_r1/grid.py','benchmark/tools/reference/grid.py'),('experiments/reference_v0_r1/cast_reference.cpp','benchmark/tools/reference/cast_reference.cpp'),('src/slam_eval_route_controller/scripts/control_core.py','src/slam_eval_route_controller/scripts/control_core.py'),('src/slam_eval_route_controller/scripts/controller.py','src/slam_eval_route_controller/scripts/controller.py'),('src/slam_eval_ground_truth/src/ground_truth_node.cpp','src/slam_eval_ground_truth/src/ground_truth_node.cpp')]:
            self.assertEqual(hashlib.sha256((ROOT/new).read_bytes()).hexdigest(),m['historical_source_sha256'][old],new)

    def test_markdown_links(self):
        files=[ROOT/'README.md']
        for folder in ['docs','benchmark','reports','src','tests']:
            files.extend((ROOT/folder).rglob('*.md'))
        for f in files:
            for target in re.findall(r'\]\(([^)]+)\)',f.read_text()):
                if re.match(r'(https?://|mailto:)',target):continue
                path,_,anchor=target.partition('#')
                dest=(f.parent/path).resolve() if path else f
                self.assertTrue(dest.exists(),f'{f.relative_to(ROOT)} -> {target}')
                if anchor and dest.suffix=='.md':
                    heads=re.findall(r'^#{1,6}\s+(.+)$',dest.read_text(),re.M)
                    anchors=[re.sub(r'[^\w\- ]','',h.lower()).replace(' ','-') for h in heads]
                    self.assertIn(anchor,anchors)

    def test_valid_r3_cohort(self):
        p=ROOT/'reports/static_benchmark/data'
        c=json.loads((p/'r3_valid_cohort.json').read_text())['included_runs']
        self.assertEqual(c,[f'run_{i:03d}' for i in range(1,10)]+['run_011'])
        for f in ['r3_pose_system.json','r3_map.json']:
            self.assertEqual(json.loads((p/f).read_text())['included_runs'],c)

if __name__=='__main__':unittest.main()
