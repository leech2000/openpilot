"""Experimental torque-request governor; all vehicle/safety limits remain downstream.

Uses actuator sign convention (positive = left), NOT the PID's internal sign.
This is driver-intent arbitration, not a curve-exit detector or lane controller.
"""
import math


class DriverUnwind:
  CONFIRM_SECONDS = 0.05
  HOLD_SECONDS = 0.30
  RECOVER_SECONDS = 0.50

  def __init__(self):
    self.reset()

  def reset(self):
    self.direction = 0
    self.candidate_direction = 0
    self.confirm_time = 0.0
    self.release_time = 0.0

  @property
  def engaged(self):
    return self.direction != 0

  def update(self, request, driver_torque, steering_pressed, *, active, enabled, dt=0.01):
    if not enabled or not active:
      self.reset()
      return request
    if not all(math.isfinite(v) for v in (request, driver_torque, dt)) or not 0 < dt <= 0.05:
      self.reset()
      return request

    direction = (1 if driver_torque > 0 else -1) if steering_pressed and driver_torque != 0 else 0
    opposing = direction != 0 and direction * request < 0
    if opposing:
      if direction != self.candidate_direction:
        self.candidate_direction = direction
        self.confirm_time = 0.0
      self.confirm_time += dt
      if self.confirm_time >= self.CONFIRM_SECONDS - 1e-9:
        self.direction = direction
        self.release_time = 0.0
    else:
      self.candidate_direction = 0
      self.confirm_time = 0.0

    if not self.engaged:
      return request
    # A continuing correction keeps priority even if the model briefly agrees.
    if direction == self.direction:
      self.release_time = 0.0
    else:
      self.release_time += dt
    if self.release_time >= self.HOLD_SECONDS + self.RECOVER_SECONDS - 1e-9:
      self.reset()
      return request

    # Never add torque, reverse its sign, or attenuate a request helping the driver.
    # Bound opposing magnitude, rather than multiplying a changing PID output.
    if self.direction * request < 0:
      cap = max(0.0, (self.release_time - self.HOLD_SECONDS) / self.RECOVER_SECONDS)
      return math.copysign(min(abs(request), cap), request)
    return request
