"""Pure overtake planning state machine. No CAN, steering or speed outputs.

Fresh OEM permissions can supply the surrounding-traffic permission input.
The original vehicle's clearance and the signal path have separate prerequisites.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OvertakeInput:
  now: float
  enabled: bool = False
  highway: bool = False
  fresh: bool = False
  signal_verified: bool = False
  signal_max_seconds: float = 0.
  rear_tracking_verified: bool = False
  oem_permission_valid: bool = False
  driver_override: bool = False
  route_clear: bool = False
  speed: float = 0.
  set_speed: float = 0.
  lead_id: int = -1
  lead_speed: float = 0.
  lead_distance: float = 0.
  lead_reliable: bool = False
  left_ready: bool = False
  right_ready: bool = False
  left_lane_valid: bool = False
  right_lane_valid: bool = False
  left_bsm: bool = True
  right_bsm: bool = True
  left_lamp: bool = False
  right_lamp: bool = False
  change_running: int = 0  # -1 left, +1 right
  change_completed: int = 0  # verified crossing AND settled in the adjacent lane
  rear_track_id: int = -1
  rear_gap: float = 0.
  original_ahead_right: bool = False
  change_finishing: int = 0


@dataclass(frozen=True)
class OvertakeDecision:
  phase: str
  reason: str
  direction: int = 0
  start_lane_change: bool = False
  original_set_speed: float = 0.
  blocked_by: tuple = ()
  # An intent is never an actuation command, even in offline scenarios.
  actuation_connected: bool = False
  signal_direction: int = 0


class OvertakePlan:
  SIGNAL_SECONDS = 1.5
  CHANGE_SECONDS = 8.
  PASS_SECONDS = 60.

  def __init__(self):
    self.phase = "idle"
    self.previous = None
    self.since = 0.
    self.cooldown = 0.
    self.slow = self.left_green = self.right_green = self.lamp = self.rear_clear = 0.
    self.target_id = -1
    self.original_speed = 0.
    self.running_seen = False
    self.reason = "waiting"
    self.finishing_seen = False
    self.lane_change_observation = LaneChangeObservation()

  def decision(self, reason=None, blocked=()):
    direction = -1 if self.phase in ("signal_left","enter_left") else 1 if self.phase in ("signal_right","return_right") else 0
    return OvertakeDecision(self.phase, reason or self.reason, direction,
                            self.phase in ("enter_left","return_right") and not self.running_seen,
                            self.original_speed, tuple(blocked),
                            signal_direction=0 if self.finishing_seen else direction)

  def transition(self, phase, now, reason):
    self.phase, self.since, self.reason = phase, now, reason
    self.lamp = 0.
    self.running_seen = False
    self.finishing_seen = False

  def cancel(self, now, reason):
    self.transition("cooldown", now, reason)
    self.cooldown = now+30.
    self.slow = self.left_green = self.right_green = self.rear_clear = 0.
    return self.decision()

  def update(self, x):
    finite = all(math.isfinite(v) for v in
                 (x.now,x.speed,x.set_speed,x.lead_speed,x.lead_distance,x.rear_gap,x.signal_max_seconds))
    if not finite:
      return self.cancel(self.previous if self.previous is not None else 0., "invalid_input")
    blockers = []
    for ok, reason in ((x.enabled,"not_engaged"),(x.highway,"not_highway"),
                       (x.fresh,"stale_input"),(not x.driver_override,"driver_override"),
                       (x.route_clear,"route_conflict"),(x.signal_verified and x.signal_max_seconds >= self.SIGNAL_SECONDS + self.CHANGE_SECONDS,
                                                        "signal_path_unverified"),
                       (x.rear_tracking_verified or x.oem_permission_valid,"rear_permission_unavailable"),
                       (70./3.6 <= x.speed <= 120./3.6,"speed_outside_envelope"),
                       (x.speed <= x.set_speed+5./3.6,"speed_above_setting")):
      if not ok:
        blockers.append(reason)
    if blockers:
      if self.phase not in ("idle","blocked","cooldown"):
        self.cancel(x.now,blockers[0])
      elif self.phase != "cooldown":
        self.transition("blocked",x.now,blockers[0])
      self.previous = x.now
      self.slow = self.left_green = self.right_green = self.rear_clear = 0.
      return self.decision(blockers[0],blockers)
    if self.previous is not None and x.now == self.previous:
      return self.decision()
    dt = x.now-self.previous if self.previous is not None else .05
    self.previous = x.now
    if not .0 < dt <= .2:
      return self.cancel(x.now,"observation_gap")
    if self.phase == "cooldown":
      if x.now < self.cooldown:
        return self.decision()
      self.transition("idle",x.now,"cooldown_complete")
    elif self.phase == "blocked":
      self.transition("idle",x.now,"prerequisites_available")

    left_ok = x.left_ready and x.left_lane_valid and not x.left_bsm
    right_ok = x.right_ready and x.right_lane_valid and not x.right_bsm
    self.left_green = self.left_green+dt if left_ok else 0.
    self.right_green = self.right_green+dt if right_ok else 0.
    passed = (self.target_id >= 0 and not x.original_ahead_right and
              x.rear_track_id == self.target_id and x.rear_gap >= max(15.,1.2*x.speed))
    self.rear_clear = self.rear_clear+dt if passed else 0.

    if self.phase == "idle":
      slow = (x.lead_reliable and x.lead_id >= 0 and
              0. <= x.lead_speed <= x.set_speed-10./3.6 and
              max(15.,x.speed) <= x.lead_distance <= 100.)
      if x.lead_id != self.target_id:
        self.slow = 0.
        self.target_id = x.lead_id
      self.slow = self.slow+dt if slow else 0.
      if self.slow >= 3. and self.left_green >= 1.:
        self.original_speed = x.set_speed
        self.transition("signal_left",x.now,"left_permitted")
    elif self.phase in ("signal_left","signal_right"):
      left = self.phase == "signal_left"
      ok = left_ok if left else right_ok and passed
      if not ok:
        return self.cancel(x.now,"permission_lost_before_change")
      lamp = x.left_lamp and not x.right_lamp if left else x.right_lamp and not x.left_lamp
      self.lamp = self.lamp+dt if lamp else 0.
      if x.now-self.since >= self.CHANGE_SECONDS:
        return self.cancel(x.now,"signal_confirmation_timeout")
      if x.now-self.since >= self.SIGNAL_SECONDS and self.lamp >= .3:
        self.transition("enter_left" if left else "return_right",x.now,"lamp_confirmed")
    elif self.phase in ("enter_left","return_right"):
      side = -1 if self.phase == "enter_left" else 1
      self.running_seen |= x.change_running == side
      self.finishing_seen |= self.running_seen and x.change_finishing == side
      if x.change_running not in (0,side):
        return self.cancel(x.now,"unexpected_change_direction")
      if (x.left_bsm if side < 0 else x.right_bsm):
        return self.cancel(x.now,"side_hazard_during_change")
      if not self.running_seen and not (left_ok if side < 0 else right_ok and passed):
        return self.cancel(x.now,"permission_lost_before_change")
      # OEM green may disappear as soon as its change status becomes active.
      # That alone never requests a reverse maneuver.
      if self.running_seen and x.change_completed == side:
        if side < 0:
          self.rear_clear = self.right_green = 0.
          self.transition("passing",x.now,"left_change_completed")
        else:
          return self.cancel(x.now,"return_completed")
      elif x.now-self.since >= self.CHANGE_SECONDS:
        return self.cancel(x.now,"change_completion_unverified")
    elif self.phase == "passing":
      if x.now-self.since >= self.PASS_SECONDS:
        return self.cancel(x.now,"pass_completion_unverified")
      if x.now-self.since >= 2. and self.rear_clear >= 1. and self.right_green >= 1.:
        self.transition("signal_right",x.now,"original_vehicle_cleared")
    return self.decision()


class LaneChangeObservation:
  """Confirm the running direction, observed crossing and return to lane keeping."""
  def __init__(self):
    self.previous = None
    self.direction = 0
    self.crossed = self.finishing = False

  def update(self, now, state, direction, crossed, fresh):
    if not fresh or not math.isfinite(now):
      self.__init__()
      return 0, 0, 0
    if self.previous is not None and now == self.previous:
      return 0, 0, 0
    if self.previous is not None and not 0. < now - self.previous <= .2:
      self.__init__()
    self.previous = now
    side = -1 if direction == 1 else 1 if direction == 2 else 0
    if state in (2, 3, 4) and side:
      if self.direction != side:
        self.direction = side
        self.crossed = self.finishing = False
      self.crossed |= crossed
      self.finishing |= state == 3 and self.crossed
      return side, side if self.finishing else 0, 0
    completed = self.direction if state == 0 and self.finishing else 0
    self.direction = 0
    self.crossed = self.finishing = False
    return 0, 0, completed
