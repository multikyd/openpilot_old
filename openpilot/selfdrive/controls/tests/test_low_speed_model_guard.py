import unittest
from types import SimpleNamespace as NS
import numpy as np

from openpilot.selfdrive.controls.lib.low_speed_model_guard import LowSpeedModelGuard


def fixture(curve=0.):
  x=np.arange(33,dtype=float)
  line=lambda offset:NS(x=x,y=offset+.5*curve*x*x)
  model=NS(laneLines=[line(-5.4),line(-1.6),line(1.6),line(5.4)],
           laneLineProbs=[.5,.99,.99,.5],laneLineStds=[.3,.1,.1,.3],
           meta=NS(desire='none',laneChangeState='off'))
  cs=NS(vEgo=2.,steeringPressed=False,steeringTorque=0.,leftBlinker=False,rightBlinker=False)
  return model,cs


class TestLowSpeedModelGuard(unittest.TestCase):
  def test_large_straight_corridor_model_turn_is_bounded(self):
    h=LowSpeedModelGuard();m,cs=fixture()
    self.assertLess(h.update(m,cs,.03,0.,100.,100.,enabled=True),.01)
    self.assertEqual(h.reason,'corridor')

  def test_real_tight_curve_is_preserved(self):
    for c in (.03,-.03):
      h=LowSpeedModelGuard();m,cs=fixture(c)
      self.assertAlmostEqual(h.update(m,cs,c,0.,100.,100.,enabled=True),c)

  def test_uncertain_geometry_cannot_create_a_new_constraint(self):
    h=LowSpeedModelGuard();m,cs=fixture();m.laneLineProbs=[0.]*4
    self.assertEqual(h.update(m,cs,.03,0.,100.,100.,enabled=True),.03)

  def test_one_boundary_uses_only_recent_confirmed_width(self):
    h=LowSpeedModelGuard();m,cs=fixture()
    h.update(m,cs,0.,0.,100.,100.,enabled=True)
    m.laneLineProbs[2]=.2
    self.assertLess(h.update(m,cs,.03,0.,100.1,100.1,enabled=True),.01)
    self.assertEqual(h.update(m,cs,.03,0.,104.,104.,enabled=True),.03)

  def test_opposite_driver_torque_below_binary_threshold_is_respected(self):
    h=LowSpeedModelGuard();m,cs=fixture();cs.steeringPressed=True;cs.steeringTorque=300.
    h.observe_driver(cs,-.002,100.)
    self.assertEqual(h.update(m,cs,.03,-.002,100.,100.,enabled=True),-.002)
    cs.steeringPressed=False;cs.steeringTorque=180.
    for i in range(1,101):
      t=100.+i*.01
      self.assertEqual(h.update(m,cs,.03,-.002,t,t,enabled=True),-.002)
    self.assertEqual(h.reason,'driver_hold')

  def test_release_rejoins_smoothly_instead_of_restoring_large_goal(self):
    h=LowSpeedModelGuard();m,cs=fixture();cs.steeringPressed=True;cs.steeringTorque=300.
    h.observe_driver(cs,-.002,100.)
    h.update(m,cs,.03,-.002,100.,100.,enabled=True)
    cs.steeringPressed=False;cs.steeringTorque=0.;previous=-.002
    for i in range(1,201):
      t=100.+i*.01;result=h.update(m,cs,.03,-.002,t,t,enabled=True)
      self.assertLessEqual(abs(result-previous),.015*.01+1e-8);previous=result
    self.assertLess(result,.01)

  def test_high_speed_signal_and_lane_change_do_not_gain_new_constraint(self):
    for kind in ['high_speed','signal','lane_change']:
      h=LowSpeedModelGuard();m,cs=fixture()
      if kind=='high_speed':cs.vEgo=10.
      if kind=='signal':cs.rightBlinker=True
      if kind=='lane_change':m.meta.laneChangeState='laneChangeStarting'
      self.assertEqual(h.update(m,cs,.03,0.,100.,100.,enabled=True),.03)
