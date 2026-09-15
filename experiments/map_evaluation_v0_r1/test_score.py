import unittest,tempfile
from pathlib import Path
import numpy as np
from score import boundary,confusion,resample,load_map
class Tests(unittest.TestCase):
 def test_unknown_is_false_negative(self):
  c=confusion(np.array([1,1,0],bool),np.array([1,0,1],bool));self.assertEqual([c[k] for k in ['tp','fp','fn','tn']],[1,1,1,0]);self.assertAlmostEqual(c['occupied_iou'],1/3)
 def test_no_unknown_crop_boundary(self):
  o=np.zeros((3,3),bool);o[1,1]=1;d=o.copy();self.assertFalse(boundary(o,d).any());d[1,0]=1;self.assertTrue(boundary(o,d)[1,1])
 def test_inverse_alignment_rotation_translation(self):
  a=np.array([[0,100],[-1,0]]);m={'origin':[0,0,0],'resolution':1};alignment={'theta_rad':np.pi/2,'tx_m':10,'ty_m':20,'scale':1,'direction':'estimated map coordinates -> gazebo_world'}
  p,ij,valid=resample(a,m,alignment,np.array([[9.5,21.5],[10.5,20.5]]));self.assertEqual(p.tolist(),[100,-1]);self.assertEqual(valid.tolist(),[True,False])
 def test_pgm_row_flip(self):
  with tempfile.TemporaryDirectory() as t:
   p=Path(t);(p/'m.pgm').write_bytes(b'P5\n# comment\n2 2\n255\n'+bytes([0,205,254,0]));(p/'m.yaml').write_text('image: m.pgm\nresolution: 1\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n');a,_=load_map(p/'m.yaml');self.assertEqual(a.tolist(),[[0,100],[100,-1]])
if __name__=='__main__':unittest.main()
