import ast
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
import unittest

spec=importlib.util.spec_from_file_location('overtake_plan_test_module',Path(__file__).parents[1]/'lib/overtake_plan.py')
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)


class TestOvertakePlan(unittest.TestCase):
  def setUp(self):
    self.h=m.OvertakePlan()
    self.x=m.OvertakeInput(now=10.,enabled=True,highway=True,fresh=True,
      signal_verified=True,signal_max_seconds=12.,rear_tracking_verified=True,route_clear=True,
      speed=27.,set_speed=30.,lead_id=40,lead_speed=24.,lead_distance=55.,lead_reliable=True,
      left_ready=True,right_ready=True,left_lane_valid=True,right_lane_valid=True,
      left_bsm=False,right_bsm=False)

  def tick(self,seconds=.05,**kw):
    result=None
    for _ in range(round(seconds/.05)):
      self.x=replace(self.x,now=self.x.now+.05,**kw)
      result=self.h.update(self.x)
      self.assertFalse(result.actuation_connected)
    return result

  def signal_left(self):
    self.assertEqual(self.tick(3.3).phase,'signal_left')

  def enter_left(self):
    self.signal_left()
    self.assertEqual(self.tick(1.7,left_lamp=True).phase,'enter_left')

  def passing(self):
    self.enter_left()
    self.tick(.3,change_running=-1)
    self.assertEqual(self.tick(change_completed=-1).phase,'passing')
    self.x=replace(self.x,change_running=0,change_completed=0,left_lamp=False)

  def test_complete_pass_requires_lamps_crossings_and_original_rear_track(self):
    self.passing()
    self.assertEqual(self.tick(2.2,rear_track_id=40,rear_gap=40.).phase,'signal_right')
    self.assertEqual(self.tick(1.7,right_lamp=True).phase,'return_right')
    self.tick(.3,change_running=1)
    result=self.tick(change_completed=1)
    self.assertEqual((result.phase,result.reason),('cooldown','return_completed'))
    self.assertEqual(result.original_set_speed,30.)

  def test_green_alone_never_bypasses_missing_capabilities(self):
    for kw,reason in ((dict(signal_verified=False),'signal_path_unverified'),
                      (dict(signal_max_seconds=3.),'signal_path_unverified'),
                      (dict(rear_tracking_verified=False),'rear_permission_unavailable')):
      self.setUp();result=self.tick(5.,**kw)
      self.assertEqual(result.phase,'blocked')
      self.assertIn(reason,result.blocked_by)
      self.assertFalse(result.start_lane_change)
      self.assertEqual(result.direction,0)

  def test_green_requires_continuous_confirmation(self):
    for _ in range(12):
      self.tick(.5,left_ready=True)
      self.tick(.05,left_ready=False)
    self.assertEqual(self.h.phase,'idle')

  def test_missing_left_lane_or_bsm_blocks_entry(self):
    for kw in (dict(left_lane_valid=False),dict(left_bsm=True),dict(lead_reliable=False)):
      self.setUp()
      self.assertEqual(self.tick(5.,**kw).phase,'idle')

  def test_lead_identity_changes_cannot_accumulate_slow_timer(self):
    for i in range(8):
      self.tick(.5,lead_id=i)
    self.assertEqual(self.h.phase,'idle')

  def test_no_lane_change_before_observed_lamp(self):
    self.signal_left()
    result=self.tick(3.)
    self.assertEqual(result.phase,'signal_left')
    self.assertFalse(result.start_lane_change)
    self.assertEqual(self.tick(6.).reason,'signal_confirmation_timeout')

  def test_permission_loss_before_crossing_cancels(self):
    self.enter_left()
    result=self.tick(left_ready=False)
    self.assertEqual(result.phase,'cooldown')
    self.assertFalse(result.start_lane_change)

  def test_green_loss_after_running_never_requests_reverse(self):
    self.enter_left();self.tick(change_running=-1)
    result=self.tick(.5,left_ready=False)
    self.assertEqual(result.phase,'enter_left')
    self.assertEqual(result.direction,-1)
    self.assertFalse(result.start_lane_change)
    self.assertEqual(self.tick(change_completed=-1).phase,'passing')

  def test_wrong_direction_or_missing_completion_stops_plan(self):
    self.enter_left()
    self.assertEqual(self.tick(change_running=1).reason,'unexpected_change_direction')
    self.setUp();self.enter_left()
    self.assertEqual(self.tick(8.5,change_running=-1).reason,'change_completion_unverified')

  def test_disappearing_front_vehicle_and_right_green_are_not_proof_of_passing(self):
    self.passing()
    result=self.tick(20.,lead_id=-1,lead_reliable=False,rear_track_id=-1)
    self.assertEqual(result.phase,'passing')
    self.assertEqual(result.direction,0)

  def test_another_rear_vehicle_is_not_the_original_vehicle(self):
    self.passing()
    self.assertEqual(self.tick(5.,rear_track_id=41,rear_gap=50.).phase,'passing')

  def test_return_requires_speed_dependent_gap_and_right_bsm_clear(self):
    self.passing()
    self.assertEqual(self.tick(5.,rear_track_id=40,rear_gap=20.).phase,'passing')
    self.assertEqual(self.tick(3.,rear_gap=40.,right_bsm=True).phase,'passing')
    self.assertEqual(self.tick(1.2,right_bsm=False).phase,'signal_right')

  def test_each_live_prerequisite_can_interrupt(self):
    for kw in (dict(enabled=False),dict(highway=False),dict(fresh=False),
               dict(driver_override=True),dict(route_clear=False)):
      self.setUp();self.enter_left()
      result=self.tick(**kw)
      self.assertEqual(result.phase,'cooldown')
      self.assertEqual(result.direction,0)

  def test_duplicate_clock_never_advances_confirmation(self):
    for _ in range(1000):
      self.h.update(self.x)
    self.assertEqual(self.h.phase,'idle')
    self.assertLess(self.h.slow,.1)

  def test_invalid_data_and_clock_gap_cannot_start(self):
    self.signal_left()
    self.assertEqual(self.tick(speed=float('nan')).reason,'invalid_input')
    self.setUp();self.signal_left()
    self.x=replace(self.x,now=self.x.now+1.)
    self.assertEqual(self.h.update(self.x).reason,'observation_gap')

  def test_fresh_oem_permission_can_supply_rear_monitoring_for_entry(self):
    result = self.tick(3.3, rear_tracking_verified=False, oem_permission_valid=True)
    self.assertEqual(result.phase, 'signal_left')
    self.assertEqual(result.signal_direction, -1)
    self.assertEqual(self.tick(oem_permission_valid=False).reason, 'rear_permission_unavailable')

  def test_original_vehicle_still_ahead_blocks_return_despite_right_green(self):
    self.passing()
    result = self.tick(3., rear_track_id=40, rear_gap=50., original_ahead_right=True)
    self.assertEqual(result.phase, 'passing')
    self.assertEqual(result.signal_direction, 0)

  def test_signal_releases_at_finishing_before_lane_change_completion(self):
    self.enter_left()
    result = self.tick(change_running=-1, change_finishing=-1)
    self.assertEqual(result.phase, 'enter_left')
    self.assertEqual(result.direction, -1)
    self.assertEqual(result.signal_direction, 0)
    self.assertEqual(self.tick(change_running=0, change_completed=-1).phase, 'passing')

  def test_lane_change_observation_requires_crossing_finishing_and_lane_keeping(self):
    h = m.LaneChangeObservation()
    self.assertEqual(h.update(10., 2, 1, False, True), (-1, 0, 0))
    self.assertEqual(h.update(10.05, 4, 1, True, True), (-1, 0, 0))
    self.assertEqual(h.update(10.10, 3, 1, True, True), (-1, -1, 0))
    self.assertEqual(h.update(10.15, 0, 0, False, True), (0, 0, -1))
    self.assertEqual(h.update(10.20, 0, 0, False, True), (0, 0, 0))

  def test_cancel_or_stale_lane_observation_cannot_claim_completion(self):
    for invalid in ('cancel', 'stale', 'gap'):
      h = m.LaneChangeObservation()
      h.update(10., 2, 2, True, True)
      if invalid != 'cancel':
        h.update(10.05, 3, 2, True, True)
      result = h.update(11. if invalid == 'gap' else 10.1, 0, 0, False, invalid != 'stale')
      self.assertEqual(result, (0, 0, 0))

  def test_original_vehicle_survives_corner_preference_and_new_front_lead(self):
    from types import SimpleNamespace as NS
    new_lead = NS(present=True, radar=True, radarTrackId=99, dRel=70.)
    original_right = NS(present=True, radar=True, radarTrackId=40, dRel=12.)
    corner_right = NS(present=True, radar=True, radarTrackId=1040, dRel=5.)
    self.assertEqual(m.observe_original_vehicle(40, [new_lead, original_right, corner_right]), (True, None))
    # Losing the saved identity supplies neither a rear track nor a clearance.
    self.assertEqual(m.observe_original_vehicle(40, [new_lead, corner_right]), (False, None))

  def test_original_vehicle_uses_conservative_duplicate_rear_distance(self):
    from types import SimpleNamespace as NS
    observations = [NS(present=True, radar=True, radarTrackId=40, dRel=d) for d in (-40., -12.)]
    ahead, rear = m.observe_original_vehicle(40, observations)
    self.assertFalse(ahead)
    self.assertEqual(rear.dRel, -12.)
    observations.append(NS(present=True, radar=True, radarTrackId=40, dRel=3.))
    self.assertTrue(m.observe_original_vehicle(40, observations)[0])

  def test_actual_production_observer_records_blockers_without_requesting_change(self):
    from types import SimpleNamespace as NS
    tree=ast.parse((Path(__file__).parents[1]/'plannerd.py').read_text(encoding='utf8'))
    f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='observe_overtake_plan')
    events=[]
    env=dict(OvertakeInput=m.OvertakeInput,observe_original_vehicle=m.observe_original_vehicle,math=__import__('math'),cloudlog=NS(event=lambda *a,**kw:events.append(kw)))
    exec(compile(ast.Module(body=[f],type_ignores=[]),'actual_overtake_observer','exec'),env)
    class SM(dict):
      logMonoTime={k:int(10e9) for k in ('modelV2','carState','carControl','carrotMan','radarState')}
      def all_checks(self,services):
        return True
    cs=NS(vEgo=27.,vCruise=108.,canValid=True,gasPressed=False,brakePressed=False,
          steeringPressed=False,leftBlinkerStalk=False,rightBlinkerStalk=False,
          leftBlinker=False,rightBlinker=False,leftLaneLine=10,rightLaneLine=10,oemLaneChangeStateValid=False,
          leftOemLaneChangeReady=True,rightOemLaneChangeReady=True,leftBlindspot=False,rightBlindspot=False)
    sm=SM(carState=cs,carControl=NS(latActive=True,longActive=True),
          carrotMan=NS(naviHighway=True,xTurnInfo=-1,xDistToTurn=-1),
          modelV2=NS(meta=NS(laneWidthLeft=3.5,laneWidthRight=3.5,laneChangeCrossed=False,
                            laneChangeState=NS(raw=0),laneChangeDirection=NS(raw=0)),laneLineProbs=[.95]*4),
          radarState=NS(leadsRight=[],leadsCenter=[],frontRadarLeadsRight=[],leadOne=NS(radarTrackId=40,vLead=24.,dRel=55.,present=True,radar=True,modelProb=.99)))
    result=env['observe_overtake_plan'](sm,self.h,-1.)
    self.assertEqual(result,10.)
    self.assertEqual(self.h.phase,'blocked')
    self.assertFalse(events[0]['actuation_connected'])
    self.assertIn('signal_path_unverified',events[0]['blocked_by'])
    self.assertIn('rear_permission_unavailable',events[0]['blocked_by'])

  def test_production_adapter_cannot_claim_missing_capabilities_or_crossing(self):
    path=Path(__file__).parents[1]/'plannerd.py'
    tree=ast.parse(path.read_text(encoding='utf8'))
    f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='observe_overtake_plan')
    call=next(n for n in ast.walk(f) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='OvertakeInput')
    values={k.arg:ast.literal_eval(k.value) for k in call.keywords
            if k.arg in ('signal_verified','signal_max_seconds','rear_tracking_verified')}
    self.assertEqual(values,dict(signal_verified=False,signal_max_seconds=0.,
                                rear_tracking_verified=False))
    # The adapter only updates a pure planner and records its prerequisites.
    for n in ast.walk(f):
      if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute):
        self.assertNotIn(n.func.attr,('send','sendcan','put','put_bool','write_text','make_can_msg'))


if __name__=='__main__':
  unittest.main()
