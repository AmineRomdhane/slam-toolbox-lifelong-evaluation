import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/slam_eval_route_controller/scripts'))
from control_core import *
P=dict(max_linear_velocity=.1,max_angular_velocity=.3,max_linear_acceleration=.1,max_angular_acceleration=.3,position_tolerance=.015,heading_tolerance=.015,waypoint_timeout=90.,max_tracking_error=.2,feedback_timeout=.5)
class Test(unittest.TestCase):
 def test_slew(self):
  self.assertAlmostEqual(slew(0,1,.1,.1),.01)
  self.assertAlmostEqual(slew(.1,0,.1,.1),.09)
 def test_geometry(self):
  self.assertAlmostEqual(lateral((.5,.1),(0,0),(1,0)),.1)
  self.assertAlmostEqual(wrap(2*math.pi+.2),.2)
 def test_abort(self):
  c=Controller(dict(waypoints=[[0,0],[1,0]],final_yaw=0),P)
  c.step(1,0,0,0); c.step(1.1,0,0,0); c.step(1.2,.2,.3,0)
  self.assertEqual(c.state,'ABORTED')
 def test_reset(self):
  c=Controller(dict(waypoints=[[0,0]],final_yaw=0),P)
  c.step(2,0,0,0);c.step(1,0,0,0);self.assertEqual(c.state,'ABORTED')
 def test_timeout(self):
  p=dict(P,waypoint_timeout=.2)
  c=Controller(dict(waypoints=[[0,0],[1,0]],final_yaw=0),p)
  for i in range(5):c.step(1+i*.1,0,0,0)
  self.assertEqual(c.state,'ABORTED')
  self.assertEqual(c.reason,'waypoint timeout')
 def test_complete(self):
  c=Controller(dict(waypoints=[[0,0]],final_yaw=0),P)
  for i in range(4): c.step(i*.1,0,0,0)
  self.assertEqual(c.state,'COMPLETED')
class FullRoute(unittest.TestCase):
 def test_complete_route_and_acceleration_bounds(self):
  import yaml
  route=yaml.safe_load((Path(__file__).resolve().parents[1]/'benchmark/routes/r1.yaml').read_text())
  c=Controller(route,P);x,y,yaw=-2.014,-.5,0.;pv=pw=0.
  for i in range(15000):
   v,w=c.step(.1+i*.02,x,y,yaw)
   self.assertLessEqual(abs(v-pv),.002+1e-8)
   self.assertLessEqual(abs(w-pw),.006+1e-8)
   pv,pw=v,w;x+=v*math.cos(yaw)*.02;y+=v*math.sin(yaw)*.02;yaw+=w*.02
   if c.state in ('COMPLETED','ABORTED'):break
  self.assertEqual(c.state,'COMPLETED')
  self.assertLessEqual(math.dist((x,y),route['waypoints'][-1]),P['position_tolerance'])
  self.assertLessEqual(abs(wrap(yaw)),P['heading_tolerance'])
class FinalCorrection(unittest.TestCase):
 def final_controller(self):
  c=Controller(dict(waypoints=[[0,0]],final_yaw=0),P)
  c.step(0,0,0,0); c.step(.1,0,0,0)
  self.assertTrue(c.final)
  return c
 def test_position_drift_reenters_approach(self):
  c=self.final_controller();deadline=c.deadline
  c.step(.2,-.025,0,0)
  self.assertEqual(c.state,'ROTATING');self.assertFalse(c.final)
  c.step(.3,-.025,0,0)
  self.assertEqual(c.state,'TRANSLATING')
  c.step(.4,-.025,0,0);self.assertGreater(c.v,0)
  for i in range(5,10): c.step(i*.1,-.001,0,.03)
  self.assertTrue(c.final);self.assertNotEqual(c.state,'COMPLETED')
  self.assertEqual(c.deadline,deadline)
  for i in range(10,20): c.step(i*.1,-.001,0,0)
  self.assertEqual(c.state,'COMPLETED')
 def test_rotation_drift_does_not_skip_corrective_turn(self):
  c=self.final_controller();c.step(.2,-.025,0,0)
  c.step(.3,-.001,0,.5)
  self.assertFalse(c.final);self.assertEqual(c.state,'ROTATING')
  self.assertLess(c.w,0.)
 def test_corrective_approach_leaves_rotation_margin(self):
  c=self.final_controller();c.step(.2,-.025,0,0);c.step(.3,-.025,0,0)
  c.step(.4,-.01,0,0)
  self.assertEqual(c.state,'TRANSLATING');self.assertGreater(c.v,0.)
 def test_repeated_drift_is_bounded_by_shared_timeout(self):
  c=self.final_controller();deadline=c.deadline
  for i in range(2,1000):
   # Each final yaw phase drifts outside tolerance; each approach returns inside.
   c.step(i*.1,-.025 if c.final else -.001,0,0)
   self.assertEqual(c.deadline,deadline)
   if c.state=='ABORTED':break
  self.assertEqual(c.state,'ABORTED');self.assertEqual(c.reason,'waypoint timeout')
  self.assertEqual((c.v,c.w),(0.,0.))
 def test_correction_preserves_tracking_abort(self):
  c=self.final_controller();c.step(.2,-.025,0,0);c.step(.3,-.3,0,0)
  self.assertEqual(c.state,'ABORTED');self.assertEqual(c.reason,'tracking limit exceeded')
 def test_converges_after_simulated_final_rotation_drift(self):
  c=self.final_controller();x,y,yaw=-.025,0,.2;pv=pw=0.
  for i in range(1,4500):
   v,w=c.step(.1+i*.02,x,y,yaw)
   self.assertLessEqual(abs(v-pv),.002+1e-8)
   self.assertLessEqual(abs(w-pw),.006+1e-8)
   pv,pw=v,w
   x+=v*math.cos(yaw)*.02;y+=v*math.sin(yaw)*.02;yaw+=w*.02
   if c.state in ('COMPLETED','ABORTED'):break
  self.assertEqual(c.state,'COMPLETED')
  self.assertLessEqual(math.hypot(x,y),P['position_tolerance'])
  self.assertLessEqual(abs(wrap(yaw)),P['heading_tolerance'])
if __name__=='__main__':unittest.main()
