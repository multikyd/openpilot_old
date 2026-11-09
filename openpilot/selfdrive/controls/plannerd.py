#!/usr/bin/env python3
import math

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning
from openpilot.selfdrive.controls.lib.overtake_plan import OvertakePlan, OvertakeInput, observe_original_vehicle
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

from openpilot.selfdrive.controls.lib.lateral_planner import LateralPlanner
import openpilot.cereal.messaging as messaging

from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner


def log_highway_overtake_candidate(sm, last_logged):
  """Observe opportunities without issuing a lane-change or signal request."""
  now = sm.logMonoTime['modelV2'] * 1e-9
  services = ('carState', 'carControl', 'carrotMan', 'radarState')
  if (now - last_logged < 1. or not sm['carrotMan'].naviHighway or
      not sm.all_checks(services) or
      any(abs(now - sm.logMonoTime[s] * 1e-9) > .3 for s in services)):
    return last_logged

  cs, cc = sm['carState'], sm['carControl']
  lead = sm['radarState'].leadOne
  speed_kph = cs.vEgo * CV.MS_TO_KPH
  lead_kph = lead.vLead * CV.MS_TO_KPH
  if (not (cs.canValid and cc.latActive and cc.longActive and lead.present and lead.radar) or
      not (speed_kph >= 70. and speed_kph <= cs.vCruise + 5. and cs.vCruise <= V_CRUISE_MAX) or
      not (0. <= lead_kph <= cs.vCruise - 10. and 10. <= lead.dRel <= 100. and lead.modelProb >= .9) or
      not math.isfinite(lead.vRel)):
    return last_logged

  cloudlog.event('highway_overtake_observation', speed_kph=speed_kph, cruise_kph=float(cs.vCruise),
                 lead_track=int(lead.radarTrackId), lead_distance_m=float(lead.dRel), lead_speed_kph=lead_kph,
                 closing_kph=-lead.vRel * CV.MS_TO_KPH,
                 left_lane_marking=int(cs.leftLaneLine), right_lane_marking=int(cs.rightLaneLine),
                 left_oem_ready=bool(cs.leftOemLaneChangeReady), right_oem_ready=bool(cs.rightOemLaneChangeReady),
                 left_bsm=bool(cs.leftBlindspot), right_bsm=bool(cs.rightBlindspot),
                 left_blinker=bool(cs.leftBlinker), right_blinker=bool(cs.rightBlinker),
                 rear_approach_verified=False)
  return now


def observe_overtake_plan(sm, plan, last_logged):
  """Evaluate prerequisites only; this function cannot request any actuation."""
  now = sm.logMonoTime['modelV2']*1e-9
  cs, cc, nav = sm['carState'], sm['carControl'], sm['carrotMan']
  md, lead = sm['modelV2'], sm['radarState'].leadOne
  services = ('carState','carControl','carrotMan','radarState','modelV2')
  fresh = sm.all_checks(services) and all(0. <= now-sm.logMonoTime[k]*1e-9 <= .3 for k in services)
  near_route = nav.xTurnInfo >= 0 and 0. <= nav.xDistToTurn <= max(500.,20.*cs.vEgo)
  override = (cs.gasPressed or cs.brakePressed or cs.steeringPressed or
              cs.leftBlinkerStalk or cs.rightBlinkerStalk or (cs.leftBlinker and cs.rightBlinker))
  # These are geometrical candidates, not a claim that an unverified white
  # marking is legally crossable. Actuation prerequisites below remain false.
  left_lane = (cs.leftLaneLine == 10 and md.meta.laneWidthLeft >= 2.8 and
               len(md.laneLineProbs) >= 4 and md.laneLineProbs[0] >= .7)
  right_lane = (cs.rightLaneLine == 10 and md.meta.laneWidthRight >= 2.8 and
                len(md.laneLineProbs) >= 4 and md.laneLineProbs[3] >= .7)
  change_running, change_finishing, change_completed = plan.lane_change_observation.update(
    now, int(md.meta.laneChangeState.raw), int(md.meta.laneChangeDirection.raw),
    bool(md.meta.laneChangeCrossed), fresh)
  # Corner-radar preference must not hide the original front-radar track
  # when that vehicle moves from our lead into the right-hand lane.
  radar = sm['radarState']
  original_ahead, original_rear = observe_original_vehicle(
    plan.target_id, (lead, *radar.leadsCenter, *radar.frontRadarLeadsRight, *radar.leadsRight))

  x = OvertakeInput(
    now=now, enabled=bool(cc.latActive and cc.longActive and cs.canValid),
    highway=bool(nav.naviHighway), fresh=fresh, route_clear=not near_route,
    driver_override=override, speed=float(cs.vEgo), set_speed=float(cs.vCruise)/3.6,
    lead_id=int(lead.radarTrackId), lead_speed=float(lead.vLead), lead_distance=float(lead.dRel),
    lead_reliable=bool(lead.present and lead.radar and lead.modelProb >= .9),
    left_ready=bool(cs.leftOemLaneChangeReady), right_ready=bool(cs.rightOemLaneChangeReady),
    left_lane_valid=left_lane, right_lane_valid=right_lane,
    left_bsm=bool(cs.leftBlindspot), right_bsm=bool(cs.rightBlindspot),
    left_lamp=bool(cs.leftBlinker), right_lamp=bool(cs.rightBlinker),
    # Signal-path capability is separate from OEM traffic permission.
    signal_verified=False, signal_max_seconds=0.,
    oem_permission_valid=bool(cs.oemLaneChangeStateValid), rear_tracking_verified=False,
    original_ahead_right=original_ahead,
    rear_track_id=int(original_rear.radarTrackId) if original_rear is not None else -1,
    rear_gap=-float(original_rear.dRel) if original_rear is not None else 0.,
    change_running=change_running, change_finishing=change_finishing,
    change_completed=change_completed)
  result = plan.update(x)
  candidate = (x.highway and x.speed >= 70./3.6 and x.lead_reliable and
               x.lead_speed <= x.set_speed-3./3.6)
  if candidate and now-last_logged >= 5.:
    cloudlog.event('highway_overtake_plan', phase=result.phase, reason=result.reason,
                   blocked_by=list(result.blocked_by), actuation_connected=False,
                   left_green=x.left_ready, right_green=x.right_ready,
                   left_geometry=left_lane, right_geometry=right_lane,
                   left_bsm=x.left_bsm, right_bsm=x.right_bsm,
                   oem_permission_valid=x.oem_permission_valid, signal_direction=result.signal_direction,
                   change_running=change_running, change_finishing=change_finishing,
                   change_completed=change_completed, original_ahead_right=original_ahead,
                   original_rear_track=x.rear_track_id, original_rear_gap_m=x.rear_gap,
                   original_lead=x.lead_id, lead_distance_m=x.lead_distance,
                   speed_kph=x.speed*3.6, setting_kph=x.set_speed*3.6)
    return now
  return last_logged


def main():
  # Keep planning off CPU5, which runs the higher-priority UI and display IRQs.
  # CPU4 control loops retain CTRL_HIGH priority and preempt this CTRL_LOW task.
  config_realtime_process(4, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("plannerd got CarParams: %s", CP.brand)

  ldw = LaneDepartureWarning()
  longitudinal_planner = LongitudinalPlanner(CP)
  lateral_planner = LateralPlanner(CP, debug=False)

  pm = messaging.PubMaster(['longitudinalPlan', 'driverAssistance', 'lateralPlan'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'vehicleParameters', 'radarState',
                            'modelV2', 'selfdriveState', 'carrotMan', 'leadBrakeObservation'],
                           poll='modelV2', ignore_avg_freq=['radarState'],
                           ignore_alive=['leadBrakeObservation'], ignore_valid=['leadBrakeObservation'])
  carrot = CarrotPlanner()
  observe_overtake = CP.carFingerprint == 'HYUNDAI_IONIQ_5_PE' and CP.openpilotLongitudinalControl
  last_overtake_log = -1.
  overtake_plan = OvertakePlan()
  last_overtake_plan_log = -1.

  while True:
    sm.update()
    if sm.updated['modelV2']:
      lateral_planner.update(sm, carrot)
      lateral_planner.publish(sm, pm)
      if observe_overtake:
        last_overtake_log = log_highway_overtake_candidate(sm, last_overtake_log)
        last_overtake_plan_log = observe_overtake_plan(sm, overtake_plan, last_overtake_plan_log)
      longitudinal_planner.update(sm, carrot, overtake_plan.decision() if observe_overtake else None)
      longitudinal_planner.publish(sm, pm, carrot)

      ldw.update(sm.frame, sm['modelV2'], sm['carState'], sm['carControl'])
      msg = messaging.new_message('driverAssistance')
      msg.valid = sm.all_checks()
      msg.driverAssistance.leftLaneDeparture = ldw.left
      msg.driverAssistance.rightLaneDeparture = ldw.right
      pm.send('driverAssistance', msg)


if __name__ == "__main__":
  main()
