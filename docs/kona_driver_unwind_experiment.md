# Kona driver correction priority — experimental

Status: default off; not vehicle-validated. This change is a request-level
driver-override prototype, not a proven curve-exit fix. It must not be merged
as an enabled-by-default tune or deployed automatically.

## Scope

`KonaDriverUnwindExperimental` is read when the torque controller starts and
only takes effect for `HYUNDAI_KONA_HEV`. Other platforms and the disabled
case retain their torque requests. The parameter appears in Carrot Web's
steering-feel group, with localized descriptions also used by the Wiki
generator. Changing it requires a controller restart.

The governor operates in actuator torque convention (positive left). A
`steeringPressed` correction opposing the request for 50 ms starts arbitration.
It removes only requests opposing the correcting direction, holds that limit
for 300 ms after correction ends, then raises the allowable opposing magnitude
from zero to one over 500 ms. Requests assisting the correction are unchanged.
These timings are candidate constants, not validated recommendations. Disabling
lateral control resets the state. The PID integrator is frozen during recovery
to avoid accumulating error behind the governor; PID P/I/F remain the
pre-governor components while its logged output reflects the bounded request.

The governor never increases request magnitude or reverses request sign.
Existing driver-torque, torque-rate, CAN, and Panda safety checks still apply
downstream. A zero request does not imply immediate zero applied EPS torque.
Carrot's existing suspension/reengagement logic is unchanged.

## What it does not fix

- A model that retains excessive turn curvature at the exit.
- Additional curvature smoothing or clipping delay before the torque PID.
- Physical steering lag, lane-boundary estimation, or road geometry errors.
- The original pre-intervention curve-exit error.

`controlsd.py`, its fixed laneless smoothing, vTurn, NNFF inputs, maximum torque,
and Hyundai torque-rate limits are deliberately not changed without causal
validation. Removing steering support in response to an incorrectly classified
driver correction can itself worsen tracking. Brief input/noise, deliberate
left/right corrections, prolonged holds, release/reapplication, lane changes,
and different speeds must be evaluated in simulation and a controlled vehicle
test before ordinary-road use.

## Checks

```
python -m unittest openpilot.selfdrive.controls.tests.test_driver_unwind -v
python -m unittest discover -s tools/docs/wiki_settings/tests -p 'test_*.py' -q
python tools/replay_driver_unwind.py /path/to/rlog.zst
```

The first suite includes execution of the actual controller update method with
stand-ins for native dependencies; this verifies sign, log, reset and integrator
freeze wiring, not the NNFF model or full process. The replay reads the full
cereal schema and holds recorded driver inputs, PID outputs and vehicle motion
fixed. It compares the governor's request only and explicitly does not replay
downstream limits or predict lane clearance. A closed-loop dynamics test and
vehicle validation remain required. Do not commit route files or identifiers.
