# Acting with this robot

The robot moves only by timed TCP waypoints (`preview` → `execute`, or `move_to`) and by gripper
commands (`gripper`, `hold`, `home`). Work out the poses yourself from what you see; check them
with `ik` and `check`, and the trajectory with `preview` and `check_batch`, before executing.

Check a trajectory before moving: a preview does not check the arm against objects.

```python
tcp = ep.observe()["robot"]["tcp_pose"]
pv = ep.preview([rh.waypoint(3.0, rh.offset_pose(tcp, dx=0.10, dz=-0.05))])
checks = ep.check_batch(pv["q"][::5])
print(pv["ok"], rh.explain(pv), all(c["collision_free"] for c in checks))
show(ep.render_plan(pv), "trajectory")
```
