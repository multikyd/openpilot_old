"""Join lane/model commands only during a mode change; retain downstream limits."""
import math


def boundary_escape_direction(model, camera_offset):
  try:
    close = []
    for index, direction in ((1, 1), (2, -1)):
      line = model.laneLines[index]
      prob, std = model.laneLineProbs[index], model.laneLineStds[index]
      x, y = line.x[0], line.y[0]+camera_offset
      if not all(math.isfinite(v) for v in (x, y, prob, std)):
        return 0
      distance = -y if direction == 1 else y
      if x <= .5 and prob >= .8 and 0. <= std <= .2 and 0. < distance < 1.3:
        close.append(direction)
    return close[0] if len(close) == 1 else 0
  except (AttributeError, IndexError, TypeError):
    return 0


class ModeTransition:
  DURATION = .6

  def __init__(self):
    self.mode = self.stamp = None
    self.elapsed = self.DURATION
    self.active = False

  def update(self, now, mode, previous, target, *, enabled=True, escape_direction=0):
    if not all(math.isfinite(v) for v in (now, previous, target)):
      self.__init__()
      return previous if math.isfinite(previous) else 0.
    dt = now-self.stamp if self.stamp is not None else .01
    self.stamp = now
    if not enabled or self.mode is None or not 0. < dt <= .1:
      self.mode = mode
      self.elapsed, self.active = self.DURATION, False
      return target
    if mode != self.mode:
      self.mode = mode
      self.elapsed = 0.
    self.active = self.elapsed < self.DURATION
    if not self.active:
      return target
    # Do not delay an existing command that moves away from a trusted close
    # boundary. This is not a new steering command and cannot add authority.
    if escape_direction*(target-previous) > 0.:
      self.elapsed, self.active = self.DURATION, False
      return target
    old = self.elapsed/self.DURATION
    self.elapsed = min(self.DURATION, self.elapsed+dt)
    new = self.elapsed/self.DURATION
    smooth = lambda x: x*x*(3.-2.*x)
    weight = min(1., (smooth(new)-smooth(old))/max(1e-9, 1.-smooth(old)))
    change = weight*(target-previous)
    # Keep late changes of the new mode's goal from jumping at the end of the
    # join. This bound exists only during the join, including a short tail.
    step = .015*dt
    result = previous+max(-step, min(step, change))
    if self.elapsed >= self.DURATION and abs(result-target) > 1e-9:
      self.elapsed = self.DURATION-dt
    return result
