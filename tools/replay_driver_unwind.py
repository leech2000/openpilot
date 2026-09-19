#!/usr/bin/env python3
"""Open-loop request comparison; NOT a vehicle or complete controller replay.

Reads full cereal rlog, excludes identifiers/GPS, and applies the candidate to
recorded actuator requests. Driver inputs and the original PID/model outputs
are held fixed. Output does not predict post-change steering angles or lanes.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from openpilot.selfdrive.controls.lib.driver_unwind import DriverUnwind


def compare(path):
  import capnp
  import zstandard

  schema = capnp.load(str(ROOT / 'openpilot/cereal/log.capnp'), imports=[
    str(ROOT / 'openpilot/cereal'), str(ROOT / 'opendbc_repo/opendbc/car')])
  with path.open('rb') as source:
    with zstandard.ZstdDecompressor().stream_reader(source) as stream:
      data = stream.read()
  events = []
  for event in schema.Event.read_multiple_bytes(data):
    kind = event.which()
    if kind in ('carState', 'carControl', 'controlsState', 'modelV2'):
      events.append((event.logMonoTime, kind, event.to_dict()[kind]))
  events.sort(key=lambda x: x[0])
  start = min(t for t, k, _ in events if k == 'carState')
  state = model = None
  governor, disabled = DriverUnwind(), DriverUnwind()
  samples, differences = [], []
  last = None
  for timestamp, kind, value in events:
    if kind == 'carState':
      state = value
    elif kind == 'modelV2':
      model = value
    elif kind == 'controlsState' and model is not None:
      time = (timestamp - start) / 1e9
      if 10 <= time <= 12.4:
        differences.append(abs(value['desiredCurvature'] - model['action']['desiredCurvature']))
    elif kind == 'carControl' and state is not None:
      time = (timestamp - start) / 1e9
      dt = .01 if last is None else (timestamp - last) / 1e9
      last = timestamp
      request = value['actuators']['torque']
      args = (request, state['steeringTorque'], state['steeringPressed'])
      unchanged = disabled.update(*args, active=value['latActive'], enabled=False, dt=dt)
      assert unchanged == request
      candidate = governor.update(*args, active=value['latActive'], enabled=True, dt=dt)
      assert abs(candidate) <= abs(request) + 1e-12 and candidate * request >= 0
      samples.append(dict(time=time, recorded=request, candidate=candidate,
                          pressed=state['steeringPressed'], engaged=governor.engaged))
  changed = [s for s in samples if abs(s['recorded'] - s['candidate']) > 1e-9]
  selected = []
  for t in (11.7, 12.1, 12.3, 12.5, 12.7, 12.9, 13.1, 13.3, 13.5, 14.):
    s = min(samples, key=lambda s: abs(s['time'] - t))
    selected.append({k: round(v, 5) if isinstance(v, float) else v for k, v in s.items()})
  return dict(
    method='Open-loop request governor on recorded inputs; not closed-loop vehicle validation',
    samples=len(samples), disabled_identical=True,
    changed_samples=len(changed),
    first_change_seconds=changed[0]['time'] if changed else None,
    last_change_seconds=changed[-1]['time'] if changed else None,
    pre_intervention_mean_absolute_model_to_target_difference=sum(differences) / len(differences),
    selected_samples=selected,
    limitations=['Recorded vehicle motion/model/PID retained; new motion not simulated',
                 'Downstream EPS/CAN rate and driver-torque limits not replayed',
                 'No inference that initial curve-exit delay is fixed'])


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('rlog', type=Path)
  parser.add_argument('--output', type=Path)
  args = parser.parse_args()
  result = json.dumps(compare(args.rlog), indent=2)
  if args.output:
    args.output.write_text(result + '\n')
  print(result)
