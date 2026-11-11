import unittest
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

from openpilot.selfdrive.controls.lib.mode_transition import ModeTransition, boundary_escape_direction


class TestModeTransition(unittest.TestCase):
  def test_same_mode_has_no_extra_filter(self):
    h = ModeTransition()
    self.assertEqual(h.update(0., 'model', 0., -.004), -.004)
    self.assertEqual(h.update(.01, 'model', -.004, .003), .003)

  def test_join_both_directions_is_continuous_and_finishes(self):
    for old, new in [('model', 'lane'), ('lane', 'model')]:
      h = ModeTransition();out = -.004
      h.update(0., old, out, out)
      first = h.update(.01, new, out, .002)
      self.assertLess(abs(first-out), .0001)
      out = first
      for i in range(2, 82):
        result = h.update(i*.01, new, out, .002)
        self.assertLessEqual(abs(result-out), .015*.01+1e-9)
        out = result
      self.assertAlmostEqual(out, .002)

  def test_changing_goal_and_mode_again_start_from_current_output(self):
    h = ModeTransition();h.update(0., 'model', -.004, -.004)
    out = -.004
    for i in range(1, 30): out = h.update(i*.01, 'lane', out, .002)
    result = h.update(.30, 'model', out, -.01)
    self.assertLess(abs(result-out), .00015)

  def test_escape_and_driver_override_are_not_delayed(self):
    h = ModeTransition();h.update(0., 'model', -.004, -.004)
    self.assertEqual(h.update(.01, 'lane', -.004, -.002, escape_direction=1), -.002)
    self.assertEqual(h.update(.02, 'model', -.002, .002, enabled=False), .002)

  def test_stale_gap_does_not_resume_old_transition(self):
    h = ModeTransition();h.update(0., 'model', 0., 0.)
    h.update(.01, 'lane', 0., .002)
    self.assertEqual(h.update(.5, 'lane', 0., .003), .003)

  def test_only_trusted_single_close_boundary_sets_escape(self):
    model=NS(laneLines=[NS(x=[0.],y=[y]) for y in [-5.,-1.1,2.,5.]],
             laneLineProbs=[.5,.99,.99,.5],laneLineStds=[1.,.08,.08,1.])
    self.assertEqual(boundary_escape_direction(model,.01),1)
    model.laneLineProbs[1]=.3
    self.assertEqual(boundary_escape_direction(model,.01),0)


class TestLaneDelayDirection(unittest.TestCase):
  def setUp(self):
    path=Path(__file__).parents[1]/'lib/drive_helpers.py'
    method=next(n for n in ast.parse(path.read_text(encoding='utf8')).body
                if isinstance(n,ast.FunctionDef) and n.name=='get_lag_adjusted_curvature')
    self.times=np.array([10.*(i/32.)**2 for i in range(17)])
    env=dict(np=np,MIN_SPEED=1.,CONTROL_N=17,MAX_LATERAL_JERK=5.,DT_MDL=.05,
             ModelConstants=NS(T_IDXS=self.times))
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),env)
    self.function=env['get_lag_adjusted_curvature']

  def test_compensation_cannot_invent_reverse_turn(self):
    for sign in (-1,1):
      curves=sign*np.linspace(.004,.001,17)
      result=self.function(None,6.,sign*.001*6.*self.times,curves,.6,6.*self.times)
      self.assertGreater(sign*result,0.)

  def test_genuine_s_bend_reversal_remains_available(self):
    curves=np.array([-.004]*6+[.003]*11)
    result=self.function(None,6.,.001*6.*self.times,curves,.6,6.*self.times)
    self.assertGreater(result,0.)

  def test_constant_turn_is_unchanged(self):
    for sign in (-1,1):
      result=self.function(None,6.,sign*.004*6.*self.times,np.full(17,sign*.004),.6,6.*self.times)
      self.assertAlmostEqual(result,sign*.004)

  def test_schema_list_without_slice_support_is_accepted(self):
    class Reader:
      def __init__(self, values): self.values=values
      def __len__(self): return len(self.values)
      def __getitem__(self, index):
        if isinstance(index,slice): raise TypeError('schema lists accept integer indexes only')
        return self.values[index]
    curves=Reader(list(-np.linspace(.004,.001,17)))
    result=self.function(None,6.,list(-.001*6.*self.times),curves,.6,list(6.*self.times))
    self.assertLess(result,0.)
