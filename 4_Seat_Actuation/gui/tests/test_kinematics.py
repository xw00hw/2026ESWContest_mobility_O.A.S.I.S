import ctypes as C
from itertools import product
import json
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from kinematics import inverse,pulses,pair_pulses

GOLDEN = json.loads((Path(__file__).parent/'fixtures/kinematics.json').read_text())


class ViewerTests(unittest.TestCase):
    def test_python_matches_original_javascript(self):
        for point in GOLDEN:
            solution = inverse(point['yaw'],point['pitch'])
            for i,motor in enumerate(solution['motors']):
                self.assertAlmostEqual(motor['phi_deg'],point['phi'][i],places=9)
                self.assertAlmostEqual(motor['rx_deg'],point['rx'][i],places=9)
                self.assertLess(motor['length_error_mm'],1e-8)

    def test_zero_pose_uses_mirrored_1200_reference(self):
        self.assertEqual(pulses(0,0),[1800,1200,1200,1800])


@unittest.skipUnless(os.environ.get('OASIS_JOINT_MATH_DLL'),'native C test library not built')
class FirmwareMathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = C.CDLL(os.environ['OASIS_JOINT_MATH_DLL'])
        cls.lib.Joint_CrankAngles.argtypes = [C.c_double,C.c_double,C.POINTER(C.c_double)]
        cls.lib.Joint_Pulses.argtypes = [C.c_int32,C.c_int32,C.POINTER(C.c_uint16)]
        cls.lib.Joint_Duration.argtypes = [C.c_int32]*4
        cls.lib.Joint_Duration.restype = C.c_uint32
        cls.lib.Joint_LimitStep.argtypes = [C.c_int32,C.c_int32,C.POINTER(C.c_int32),
                                          C.POINTER(C.c_int32),C.c_uint,C.POINTER(C.c_uint16)]
        cls.lib.Joint_PosePulses.argtypes = [C.POINTER(C.c_int32),C.POINTER(C.c_uint16)]
        cls.lib.Joint_PoseStep.argtypes = [C.POINTER(C.c_int32),C.POINTER(C.c_int32),C.c_uint,C.POINTER(C.c_uint16)]

    def test_independent_left_right_pin_mapping(self):
        out = (C.c_uint16*4)()
        for y,p in product(range(0,20001,1000),range(-8000,8001,1000)):
            pose=(y,p,20000-y,-p)
            self.assertEqual(self.lib.Joint_PosePulses((C.c_int32*4)(*pose),out),1)
            left,right=pulses(y,p),pulses(20000-y,-p)
            self.assertEqual(list(out),[right[0],left[1],right[2],left[3]])
            self.assertEqual(list(out),pair_pulses(*pose))
        self.assertEqual(self.lib.Joint_PosePulses((C.c_int32*4)(0,0,27001,0),out),0)

    def test_yaw_only_extension_and_geometric_rejection(self):
        out=(C.c_uint16*4)()
        for y in range(0,27001,250):
            self.assertEqual(self.lib.Joint_Pulses(y,0,out),1)
            self.assertEqual(list(out),pulses(y,0))
        self.assertEqual(self.lib.Joint_Pulses(27000,8000,out),0)
        with self.assertRaises(ValueError):
            pulses(27000,8000)
        previous=pulses(0,0)
        y,p=0,0
        for _ in range(1000):
            ny,np=C.c_int32(27000),C.c_int32(0)
            self.assertEqual(self.lib.Joint_LimitStep(y,p,C.byref(ny),C.byref(np),2,out),1)
            self.assertTrue(all(abs(a-b)<=2 for a,b in zip(previous,out)))
            y,p=ny.value,np.value;previous=list(out)
            if y==27000:
                break
        self.assertEqual((y,p),(27000,0))

    def test_one_side_moves_with_other_side_fixed_and_pwm_slew_limited(self):
        for initial,target,fixed in [((5000,2000,0,0),(5000,2000,20000,8000),(1,3)),
                                     ((0,0,5000,2000),(20000,-8000,5000,2000),(0,2))]:
            pose=initial
            previous=pair_pulses(*pose)
            held=previous[:]
            for _ in range(1000):
                requested=(C.c_int32*4)(*target)
                out=(C.c_uint16*4)()
                self.assertEqual(self.lib.Joint_PoseStep((C.c_int32*4)(*pose),requested,2,out),1)
                self.assertEqual([out[i] for i in fixed],[held[i] for i in fixed])
                self.assertTrue(all(abs(a-b)<=2 for a,b in zip(out,previous)))
                self.assertEqual(list(out),pair_pulses(*requested))
                pose=tuple(requested);previous=list(out)
                if pose==target:
                    break
            else:
                self.fail('independent target not reached')

    def test_c_matches_original_javascript(self):
        for point in GOLDEN:
            angles = (C.c_double*2)()
            self.assertEqual(self.lib.Joint_CrankAngles(point['yaw'],point['pitch'],angles),1)
            for a,b in zip(angles,point['phi']):
                self.assertAlmostEqual(a,b,places=9)

    def test_c_and_python_agree_across_workspace(self):
        out = (C.c_uint16*4)()
        for y,p in product(range(0,20001,250),range(-8000,8001,250)):
            self.assertEqual(self.lib.Joint_Pulses(y,p,out),1)
            self.assertEqual(list(out),pulses(y,p),(y,p))
        for y,p in [(-1,0),(27001,0),(0,8001),(0,-8001)]:
            self.assertEqual(self.lib.Joint_Pulses(y,p,out),0)

    def test_duration_limits_actual_crank_speed(self):
        for start,end in [((0,0),(20000,8000)),((20000,-8000),(0,8000)),
                          ((0,8000),(0,-8000)),((20000,8000),(0,0))]:
            duration = self.lib.Joint_Duration(*start,*end)/1000
            previous = inverse(start[0]/1000,start[1]/1000)['motors']
            for i in range(1,1001):
                t = i/1000
                f = t**3*(10-15*t+6*t*t)
                now = inverse((start[0]+(end[0]-start[0])*f)/1000,
                              (start[1]+(end[1]-start[1])*f)/1000)['motors']
                for a,b in zip(previous,now):
                    speed = abs(a['phi_deg']-b['phi_deg'])/(duration/1000)
                    self.assertLessEqual(speed,15)
                previous = now

    def test_runtime_slew_guard_reaches_target_without_jumps(self):
        for start,end in [((0,0),(20000,8000)),((20000,-8000),(0,8000)),
                          ((0,8000),(0,-8000)),((20000,8000),(0,0))]:
            y,p = start
            previous = pulses(y,p)
            for _ in range(1000):
                ny,np = C.c_int32(end[0]),C.c_int32(end[1])
                out = (C.c_uint16*4)()
                self.assertEqual(self.lib.Joint_LimitStep(y,p,C.byref(ny),C.byref(np),2,out),1)
                self.assertTrue(all(abs(a-b)<=2 for a,b in zip(out,previous)))
                self.assertEqual(list(out),pulses(ny.value,np.value))
                if (ny.value,np.value)==end:
                    break
                self.assertNotEqual((ny.value,np.value),(y,p),'controller stalled')
                y,p = ny.value,np.value
                previous = list(out)
            else:
                self.fail('target not reached')


if __name__=='__main__':
    unittest.main()
