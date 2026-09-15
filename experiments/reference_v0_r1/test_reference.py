import unittest
from grid import cells_on_segment
from scipy.spatial.transform import Rotation,Slerp
import numpy as np
class ReferenceTests(unittest.TestCase):
 def test_horizontal(self):self.assertEqual(cells_on_segment([.5,.5],[3.5,.5]),[(0,0),(1,0),(2,0),(3,0)])
 def test_reverse(self):self.assertEqual(cells_on_segment([3.5,.5],[.5,.5]),[(3,0),(2,0),(1,0),(0,0)])
 def test_corner(self):self.assertEqual(cells_on_segment([.5,.5],[2.5,2.5]),[(0,0),(1,1),(2,2)])
 def test_negative(self):self.assertEqual(cells_on_segment([-.5,-.5],[1.5,-.5]),[(-1,-1),(0,-1),(1,-1)])
 def test_hit_occlusion(self):
  cells=cells_on_segment([.5,.5],[2.2,.5]);self.assertEqual(cells[-1],(2,0));self.assertNotIn((3,0),cells)
 def test_boundary_start(self):self.assertEqual(cells_on_segment([2.,.5],[.5,.5]),[(2,0),(1,0),(0,0)])
 def test_shortest_rotation(self):
  q=Rotation.from_euler('z',[179,-179],degrees=True);v=Slerp([0,1],q)([.5]).as_euler('xyz',degrees=True)[0,2];self.assertAlmostEqual(abs(v),180)
 def test_full_extrinsic(self):
  r=Rotation.from_euler('y',.1);v=r.apply([-.032,0,.171]);self.assertNotAlmostEqual(v[0],-.032);self.assertAlmostEqual(np.linalg.norm(v),np.linalg.norm([-.032,0,.171]))
if __name__=='__main__':unittest.main()
