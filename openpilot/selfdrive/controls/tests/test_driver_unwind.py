import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.driver_unwind import DriverUnwind


class TestDriverUnwind(unittest.TestCase):
  def update(self, g, request=-1., driver=200., pressed=True, **kw):
    return g.update(request, driver, pressed, active=kw.get('active', True), enabled=kw.get('enabled', True))

  def trigger(self, g, direction=1):
    for _ in range(5):
      result = self.update(g, -direction, direction * 200.)
    self.assertEqual(result, 0.)

  def test_disabled_is_identical_and_clears_state(self):
    g = DriverUnwind()
    self.trigger(g)
    self.assertEqual(self.update(g, enabled=False), -1.)
    self.assertFalse(g.engaged)

  def test_short_noise_does_not_trigger(self):
    g = DriverUnwind()
    for _ in range(4):
      self.assertEqual(self.update(g), -1.)
    self.update(g, pressed=False)
    self.assertFalse(g.engaged)

  def test_right_and_left_corrections_are_symmetric(self):
    for direction in (-1, 1):
      g = DriverUnwind()
      self.trigger(g, direction)
      self.assertEqual(self.update(g, direction * .6, direction * 200.), direction * .6)

  def test_hold_then_bounded_recovery(self):
    g = DriverUnwind()
    self.trigger(g)
    out = [self.update(g, pressed=False) for _ in range(80)]
    self.assertTrue(all(abs(v) < 1e-8 for v in out[:30]))
    self.assertAlmostEqual(out[54], -.5)
    self.assertEqual(out[-1], -1.)
    self.assertTrue(all(abs(b - a) <= .020001 for a, b in zip(out, out[1:])))
    self.assertFalse(g.engaged)

  def test_repeated_correction_restarts_hold(self):
    g = DriverUnwind()
    self.trigger(g)
    for _ in range(60):
      self.update(g, pressed=False)
    self.assertEqual(self.update(g), 0.)

  def test_inactive_resets_without_delayed_reengagement(self):
    g = DriverUnwind()
    self.trigger(g)
    self.update(g, active=False)
    self.assertEqual(self.update(g, pressed=False), -1.)

  def test_never_increases_or_reverses_torque(self):
    rng = np.random.default_rng(34)
    g = DriverUnwind()
    for _ in range(5000):
      request = rng.uniform(-1., 1.)
      result = self.update(g, request, rng.choice([-200., 200.]), bool(rng.integers(2)))
      self.assertLessEqual(abs(result), abs(request) + 1e-12)
      self.assertGreaterEqual(result * request, 0.)

  def test_invalid_timestep_does_not_leave_latched_state(self):
    g = DriverUnwind()
    self.trigger(g)
    self.assertEqual(g.update(-1., 200., True, active=True, enabled=True, dt=1.), -1.)
    self.assertFalse(g.engaged)


class TestTorqueControllerWiring(unittest.TestCase):
  """Execute the real update method with stand-ins for native vehicle dependencies.

  The fixed PID output deliberately isolates arbitration, sign conversion,
  integrator-freeze wiring, logging and inactive reset. Not a dynamics test.
  """
  def test_actual_update_method_sign_logging_and_recovery_freeze(self):
    path = Path(__file__).parents[1] / 'lib/latcontrol_torque.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LatControlTorque')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
    ns = dict(math=math, np=np, log=NS(ControlsState=NS(LateralTorqueState=NS(new_message=NS))),
              ACCELERATION_DUE_TO_GRAVITY=9.81, LOW_SPEED_X=[0, 10, 20, 30], LOW_SPEED_Y=[15, 13, 10, 5],
              LatControlInputs=lambda *a: a)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), ns)
    freezes = []
    def pid_update(*args, **kwargs):
      freezes.append(kwargs['freeze_integrator'])
      return 1.  # internal PID positive -> actuator right/negative
    controller = NS(frame=0, params=NS(get_int=lambda k: 0), lateralTorqueCustom=0,
                    use_steering_angle=True, use_nnff=False, use_nnff_lite=False,
                    steering_angle_deadzone_deg=0., torque_params=NS(),
                    torque_from_lateral_accel=lambda *a, **k: 0., steer_max=1.,
                    pid=NS(update=pid_update, p=1., i=0., d=0., f=0.),
                    driver_unwind=DriverUnwind(), driver_unwind_enabled=True,
                    _check_saturation=lambda *a: False)
    cs = NS(steeringAngleDeg=-60., steeringTorque=200., steeringPressed=True, vEgo=8., aEgo=0.)
    vm = NS(get_steer_from_curvature=lambda *a: 0., calc_curvature=lambda *a: 0.)
    params = NS(roll=0., angleOffsetDeg=0.)
    def run(active=True):
      return ns['update'](controller, active, cs, vm, params, False, .03, NS(), False)
    for _ in range(5):
      output, _, logged = run()
    self.assertEqual(output, 0.)
    self.assertEqual(logged.output, output)
    cs.steeringPressed = False
    self.assertEqual(run()[0], 0.)
    self.assertTrue(freezes[-1])
    run(False)
    self.assertFalse(controller.driver_unwind.engaged)
    controller.driver_unwind_enabled = False
    self.assertEqual(run()[0], -1.)


if __name__ == '__main__':
  unittest.main()
