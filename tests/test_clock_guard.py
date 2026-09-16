import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"benchmark/tools/slam"))
import unittest
from clock_guard import ClockGuard

def endpoint(name='rosbag2_player',gid='a'):
    return {'node':name,'gid':gid}

class ClockGuardTests(unittest.TestCase):
    def test_zero(self):
        e=ClockGuard().observe([],0)
        self.assertEqual(e['state'],'zero');self.assertIsNone(e['conflict'])
    def test_one_expected(self):
        e=ClockGuard().observe([endpoint()],0)
        self.assertEqual(e['state'],'one_expected');self.assertIsNone(e['conflict'])
    def test_one_unresolved(self):
        e=ClockGuard().observe([endpoint('_NODE_NAME_UNKNOWN_')],0)
        self.assertEqual(e['state'],'one_unresolved');self.assertIsNone(e['conflict']);self.assertIsNotNone(e['warning'])
    def test_two(self):
        e=ClockGuard().observe([endpoint(),endpoint('_NODE_NAME_UNKNOWN_','b')],0)
        self.assertEqual(e['count'],2);self.assertIsNotNone(e['conflict'])
    def test_one_to_two_during_playback(self):
        g=ClockGuard();g.start_playback(0)
        self.assertIsNone(g.observe([endpoint()],1)['conflict'])
        g.clock_received(1,1);g.clock_received(2,2)
        self.assertTrue(g.passed())
        self.assertIsNotNone(g.observe([endpoint(),endpoint('gazebo','b')],3)['conflict'])
        self.assertFalse(g.passed())
    def test_name_resolution_keeps_same_endpoint(self):
        g=ClockGuard();g.start_playback(0)
        g.observe([endpoint('_NODE_NAME_UNKNOWN_')],1);g.observe([endpoint()],2)
        g.clock_received(1,3);g.clock_received(2,3.1)
        self.assertTrue(g.passed())
    def test_identified_unexpected(self):
        self.assertIsNotNone(ClockGuard().observe([endpoint('gazebo')],0)['conflict'])
    def test_clock_advancing_watchdog(self):
        g=ClockGuard();g.start_playback(0);g.check_advancing(11)
        with self.assertRaises(RuntimeError):g.check_advancing(13)
        g.clock_received(1,13);g.clock_received(2,13.1);g.check_advancing(14)
        with self.assertRaises(RuntimeError):g.check_advancing(16)
if __name__=='__main__':unittest.main()
