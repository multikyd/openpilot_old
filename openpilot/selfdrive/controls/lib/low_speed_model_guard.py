"""Low-speed model corridor check and recovery after a driver's correction."""
import math

import numpy as np


class LowSpeedModelGuard:
  def __init__(self):
    self.width = self.width_stamp = self.bounds = self.bounds_stamp = None
    self.stamp = self.driver_stamp = self.geometry_stamp = None
    self.hold_until = -math.inf
    self.driver_sign = self.driver_threshold = 0.
    self.recovery = 1.
    self.output = None
    self.reason = 'off'

  def observe_driver(self, cs, measured, now):
    if not all(math.isfinite(v) for v in (cs.vEgo, measured, now, cs.steeringTorque)):
      return
    if not 0. <= cs.vEgo < 8.:
      self.hold_until = -math.inf
      self.driver_sign = 0.
      self.recovery = 1.
      return
    if cs.steeringPressed:
      self.driver_sign = float(np.sign(cs.steeringTorque))
      self.driver_threshold = abs(cs.steeringTorque)*.45
      self.driver_stamp = now
      self.hold_until = now+.5
      self.recovery = 0.
      self.output = measured

  def _update_bounds(self, model, now, camera_offset):
    stations = np.array([4., 7., 10.])
    try:
      lines, reliable = [], []
      for i in (1, 2):
        line = model.laneLines[i]
        x, y = np.asarray(line.x), np.asarray(line.y)
        if (len(x) < 8 or len(x) != len(y) or x[0] > .5 or x[-1] < 10. or
            not np.isfinite(x).all() or not np.isfinite(y).all() or not np.all(np.diff(x) > 0.)):
          raise ValueError('bad local corridor')
        lines.append(np.interp(stations, x, y)+camera_offset)
        reliable.append(model.laneLineProbs[i] >= .65 and 0. <= model.laneLineStds[i] < .25)
      left, right = lines
      width = right-left
      if all(reliable) and np.all((width >= 2.6) & (width <= 4.3)) and np.ptp(width) < .6:
        self.width, self.width_stamp = width.copy(), now
      width_recent = self.width_stamp is not None and 0. <= now-self.width_stamp <= 3.
      if width_recent and any(reliable):
        if not reliable[1]: right = left+self.width
        if not reliable[0]: left = right-self.width
        low = float(np.max(2.*(left+1.2)/stations**2))
        high = float(np.min(2.*(right-1.2)/stations**2))
        if low <= high:
          self.bounds, self.bounds_stamp = (low, high), now
    except (AttributeError, IndexError, TypeError, ValueError):
      self.bounds = self.bounds_stamp = None

  def update(self, model, cs, target, measured, stamp, now, *, enabled, camera_offset=.01):
    self.reason = 'off'
    if (not enabled or not all(math.isfinite(v) for v in (target, measured, stamp, now, cs.vEgo, camera_offset)) or
        not .25 <= cs.vEgo < 8. or not 0. <= now-stamp <= .2 or abs(camera_offset) > .2 or
        cs.leftBlinker or cs.rightBlinker or str(model.meta.laneChangeState) != 'off' or
        str(model.meta.desire) != 'none'):
      return target
    dt = min(.05, max(0., now-self.stamp)) if self.stamp is not None else .01
    self.stamp = now
    if stamp != self.geometry_stamp:
      self.geometry_stamp = stamp
      self._update_bounds(model, now, camera_offset)
    if self.bounds_stamp is not None and 0. <= now-self.bounds_stamp <= .5:
      bounded = float(np.clip(target, *self.bounds))
      if abs(bounded-target) > 1e-6: self.reason = 'corridor'
      target = bounded

    # The raw torque may remain a deliberate countersteer even just below the
    # car's binary steeringPressed threshold. Do not immediately pull back.
    countersteer = (self.driver_stamp is not None and 0. <= now-self.driver_stamp <= 1. and
                    self.driver_sign*cs.steeringTorque > self.driver_threshold and
                    cs.steeringTorque*(target-measured) > 0.)
    if countersteer:
      self.driver_stamp = now
      self.hold_until = now+.5
      self.recovery = 0.
    if now < self.hold_until:
      self.output = measured
      self.reason = 'driver_hold'
      return measured
    if self.recovery < 1. and self.output is not None:
      remaining = max(dt, 1.-self.recovery)
      step = float(np.clip((target-self.output)*dt/remaining, -.015*dt, .015*dt))
      self.output += step
      self.recovery = min(1., self.recovery+dt)
      if self.recovery == 1. and abs(target-self.output) > 1e-6:
        self.recovery = 1.-dt
      self.reason = 'driver_recovery'
      return self.output
    self.output = target
    return target
