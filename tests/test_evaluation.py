import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"benchmark/tools/slam"))
import unittest,tempfile,csv,math
from pathlib import Path
from evaluate import evaluate,resource_summary,failure_fraction,wrap
class EvaluationTests(unittest.TestCase):
 def reference(self):return [(i*100000000, i*.1, .2*math.sin(i*.1), .1*math.cos(i*.1)) for i in range(101)]
 def test_rigid_transform_no_scale(self):
  gt=self.reference();c,s=math.cos(.7),math.sin(.7)
  est=[(t,c*x-s*y+2,s*x+c*y-3,a+.7) for t,x,y,a in gt]
  with tempfile.TemporaryDirectory() as d:r=evaluate(est,gt,d)
  self.assertLess(r['ate_translation_m']['rmse'],1e-12);self.assertLess(r['ape_yaw_rad']['rmse'],1e-12)
  self.assertLess(r['rpe_translation_1m_m']['rmse'],1e-12);self.assertLess(r['rpe_rotation_1m_rad']['rmse'],1e-12)
  self.assertEqual(r['alignment']['scale'],1.)
 def test_scale_error_is_not_removed(self):
  gt=self.reference();est=[(t,2*x,2*y,a) for t,x,y,a in gt]
  with tempfile.TemporaryDirectory() as d:r=evaluate(est,gt,d)
  self.assertGreater(r['ate_translation_m']['rmse'],2.)
  self.assertGreater(r['rpe_translation_1m_m']['rmse'],.9)
 def test_exact_association_full_reference_coverage(self):
  gt=self.reference();est=gt[10:40]+gt[45:]
  with tempfile.TemporaryDirectory() as d:
   r=evaluate(est,gt,d);rows=list(csv.DictReader((Path(d)/'rpe_1m_errors.csv').read_text().splitlines()))
   self.assertAlmostEqual(r['trajectory_coverage'],len(est)/len(gt))
   self.assertFalse(any(int(row['start_timestamp_ns'])<4000000000 and int(row['end_timestamp_ns'])>4000000000 for row in rows))
  shifted=[(t+1,x,y,a) for t,x,y,a in gt]
  with tempfile.TemporaryDirectory() as d:self.assertEqual(evaluate(shifted,gt,d)['trajectory_coverage'],0)
 def test_yaw_wrap(self):
  gt=self.reference();est=[(t,x,y,a+2*math.pi+.2) for t,x,y,a in gt]
  with tempfile.TemporaryDirectory() as d:r=evaluate(est,gt,d)
  self.assertAlmostEqual(r['ape_yaw_rad']['rmse'],.2)
 def test_fixed_distance_rotational_rpe(self):
  gt=[(i*100000000,i*.1,0.,0.) for i in range(101)]
  est=[(t,x,y,.1*x) for t,x,y,a in gt]
  with tempfile.TemporaryDirectory() as d:r=evaluate(est,gt,d)
  self.assertAlmostEqual(r['rpe_rotation_1m_rad']['rmse'],.1,places=10)
 def test_resource_boundary_integration(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'r.csv';p.write_text('elapsed_wall_seconds,cpu_seconds,rss_bytes\n0,0,100\n1,1,200\n2,1,300\n3,1.5,400\n')
   r=resource_summary(p,.5,2.5)
   self.assertAlmostEqual(r['active_playback']['mean_cpu_percent_one_core'],37.5)
   self.assertAlmostEqual(r['active_playback']['mean_rss_bytes'],200)
   self.assertAlmostEqual(r['whole_process_sampled']['mean_cpu_percent_one_core'],50)
 def test_failed_run_fraction(self):
  gt=[(i*1000000000,float(i),0.,0.) for i in range(11)]
  self.assertEqual(failure_fraction(gt,5000000000,0,10000000000),.5)
  self.assertEqual(failure_fraction(gt,None,0,10000000000),0)
if __name__=='__main__':unittest.main()
