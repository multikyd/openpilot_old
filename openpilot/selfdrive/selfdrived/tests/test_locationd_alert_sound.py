"""Only the temporary localizer's nonurgent sound is suppressed."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

PATH = Path(__file__).parents[1]/'events.py'
TREE = ast.parse(PATH.read_text(encoding='utf8'))
FUNCTION = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'soft_disable_alert')
FUNCTION.returns = None
for n in ast.walk(FUNCTION):
  if isinstance(n, ast.arg): n.annotation = None
  if isinstance(n, ast.FunctionDef): n.returns = None


class TestLocationdAlertSound(unittest.TestCase):
  def setUp(self):
    def soft(text): return NS(text=text, audible_alert='soft', urgent=False, creation_delay=0.)
    def immediate(text): return NS(text=text, audible_alert='immediate', urgent=True, creation_delay=0.)
    self.env = dict(AudibleAlert=NS(none='none', warningSoft='soft'), DT_CTRL=.01,
                    SoftDisableAlert=soft, ImmediateDisableAlert=immediate)
    exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), str(PATH), 'exec'), self.env)

  def alert(self, remaining, **kwargs):
    callback = self.env['soft_disable_alert']('locationd Temporary Error', **kwargs)
    return callback(None, None, None, True, remaining, None)

  def test_temporary_alert_is_silent_with_unchanged_text(self):
    a = self.alert(280, audible_alert='none')
    self.assertEqual(a.audible_alert, 'none')
    self.assertEqual(a.text, 'locationd Temporary Error')
    self.assertFalse(a.urgent)

  def test_imminent_disengagement_still_sounds(self):
    urgent = self.alert(49, audible_alert='none', creation_delay=.3)
    self.assertEqual(urgent.audible_alert, 'immediate')
    self.assertEqual(urgent.creation_delay, 0.)
    self.assertEqual(self.alert(0, audible_alert='none').audible_alert, 'immediate')

  def test_other_soft_disable_events_keep_existing_sound(self):
    self.assertEqual(self.alert(280).audible_alert, 'soft')

  def test_temporary_display_is_delayed_without_removing_event_type(self):
    a = self.alert(280, audible_alert='none', creation_delay=.3)
    self.assertEqual(a.creation_delay, .3)
    self.assertFalse(.19 >= a.creation_delay)
    self.assertTrue(.30 >= a.creation_delay)

  def test_only_locationd_event_uses_silent_override(self):
    events = next(n.value for n in TREE.body if isinstance(n, ast.AnnAssign) and
                  isinstance(n.target, ast.Name) and n.target.id == 'EVENTS')
    overrides = []
    for name, mapping in zip(events.keys, events.values):
      for node in ast.walk(mapping):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'soft_disable_alert':
          if any(k.arg == 'audible_alert' for k in node.keywords): overrides.append(name.attr)
    self.assertEqual(overrides, ['locationdTemporaryError'])
