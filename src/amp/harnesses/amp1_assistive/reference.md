# Acting with this robot

The robot moves by planned picks, places and moves (`grasps` → `plan_pick` → `run`, `places` →
`plan_place` → `run`, `plan_move`, or `pick_verified` and `place_verified`, which also verify),
and by timed TCP waypoints (`preview` → `execute`, or `move_to`) and gripper commands (`gripper`,
`hold`, `home`) for what the planner does not cover. Examine a grasp or place candidate before you
take it (render or annotate it); do not take the first one unexamined. Check your own poses with
`ik` and `check`, and their trajectory with `preview` and `check_batch`, before executing. Verify a
grasp before lifting it fully: stop after a small lift (`stop_after="lift_test"`), then check
`verify_grasp`, the gripper reading and the wrist camera, or let `pick_verified` do it.

## What the assisted calls add to the reference above

- `obj`/`target` arguments take an id string or an `Obj`; `grasp`/`place` a Pose7, a 4x4 or a
  candidate dict.
- A tool without fingers (`gripper` `null`) also refuses `grasps`, `plan_pick`, `plan_place`,
  `verify_grasp`, `fingers_view` and the like with a `HarnessError` naming it.
- A handle `ep.ee(name)` has the assisted calls too (`grasps`, `places`, `check_grasp`,
  `check_place`, `plan_*`, `run`, `lift`, `release`, `move`, `push`, the steps calls,
  `pick_verified`, `place_verified`, `verify_grasp`, `fingers_view`), which read that gripper and
  its wrist camera. Each arm plans for itself and treats the other as an obstacle.
- Agent objects work with `grasps`, `places` and plans like perceived ones.
- The held object is the one `observe()["held"]` names, whether a planned pick or your own
  waypoints and `gripper(True)` closed on it: planned places and moves carry it either way,
  and after a motion that loses it (waited, the default) `reply["last_chunk"]["dropped"]` and
  `observe()["last_chunk"]["dropped"]` name it.
- Planned picks, places and moves check the arm and the held object against the perceived objects
  and the support surface; their previews also report `collision`, `no_path` and `variant`.
  Manual previews still do not check the arm against objects.
- `execute` also runs an assist preview or its `plan_id`; `execute_both` runs one as `execute` does.
- `budget(pick_steps=, place_steps=)` also returns `picks_left` and `pick_place_left` for the step
  costs you give it, such as your plans' `n_steps`.
- `rh.top_down_across(obj, z=None, yaw_offset_deg=0)` is a top-down Pose7 over an object's obb
  centre, closing across its short side (z default: mid-height); a handle's follows its end
  effector.
- `annotate(cam, grasps=...)` and the render overlays `grasps`/`places` draw candidates (green
  feasible, red not, labelled g0../p0..).

## Assisted manipulation (the main tools)

| call | returns |
|---|---|
| `grasps(obj, max=8, target=None, point=None, near=None)` | `Candidates` (a list, best first): `.feasible` the `ok` ones, `.best` the first feasible or None, `.message` why none is feasible, `.table()` one line each. A candidate: `tcp` (fingers open), `pregrasp`, `width`, `score`, `ok`, `reachable`, `collision_free`, `closure` (expected reading), `notes`. **`target`/`point` are where you will PLACE it** (one or the other), ranking grasps that allow that place first; **`near=xyz` prefers contact near that point of the object**. |
| `places(obj, target=None, point=None, max=8)` | place candidates for the held object, set down on `target` (on whatever is highest under it there, by the perceived shape) or at a `point` on the support surface: `tcp`, `object_pose` (the pose of the object's frame, which for a perceived object is the world's, not at the object), `object_center` (where the object's box centre ends up), `release_height`, `ok`, `notes` |
| `check_grasp(obj, tcp)`, `check_place(obj, tcp, target=, point=)` | evaluate exactly your own pose (candidate JSON) |
| `plan_pick(obj, grasp, options=None, stop_after=None, lift_test_m=None)` | preview (`ok`, `plan_id`, `issues`, `segments`, `duration`, `n_steps`) of transit, approach, close, lift; `stop_after="lift_test"` stops after the close and a `lift_test_m` (0.015) lift; `stop_after="close"` stops at the close, which a pick of an object another end effector holds always does (a handover) |
| `plan_place(obj, place, target=, point=, options=None, force=False)` | transit, descend, release, retreat of the held object; refused with nothing held unless `force=True` |
| `plan_lift(obj=None, height_m=None)` | continue a lift-tested pick: raise the held object by `height_m` (0.10) |
| `plan_release(retreat_m=None)` | open where it is and back off `retreat_m` (0.08) along the approach |
| `plan_move(tcp)` | collision-free transit to a TCP pose **checked with the held object** |
| `plan_push(start, end, height_m=None)` | close the fingers, descend at `start`'s xy to `height_m` (0.02) above the support surface, push straight to `end`'s xy, rise |
| `run(preview)` | **motion**: executes a preview, or raises `HarnessError` naming its error issues; returns the execute reply plus `"plan"` |
| `lift(obj=None, height_m=None)`, `release(retreat_m=None)`, `move(tcp)`, `push(start, end, height_m=None)` | **motion**: `plan_*` + `run` in one call |
| `execute(preview_or_plan_id, wait=True)` | **motion**: runs an assist or manual preview; with `wait` returns after the motion with `observation` and `last_chunk` |

Options (all optional): pick `approach` 0.10, `lift` 0.10, `speed_scale` 0.5, `close_s`,
`stop_after`, `lift_test_m`; place `approach` 0.10, `retreat` 0.10, `speed_scale` 0.5, `open_s`;
lift `height_m`; release `retreat_m`; move `speed_scale`; push `height_m`, `speed_scale`.
`speed_scale` scales transits only; the slow phases near contact are fixed.

A plan is valid only from the state it was made in: after any motion plan again ("the robot moved
since this plan"). Check `pv["ok"]` before executing, and **never index `cands.feasible[0]`
without checking it is non-empty** (use `cands.best`).

Rarely needed: `steps()`, `add_step(kind, obj, tcp, target=, point=, options=)`,
`update_step(id, tcp=, options=)`, `delete_step(id)`, `clear_steps()` and `execute_steps()`
(**motion**) build a multi-step pick/place plan, each step planned from the end of the one before,
and run it back to back. Planning and running one step at a time, looking in between, does the
same.

## Verified pick and place (use these by default)

`pick_verified(obj, candidates=None, max_tries=3, target=None, point=None, near=None, lift=True,
height_m=None, lift_test_m=None, options=None, require_wrist=False)` → `Verified`. **motion.**
For each try: `plan_pick(stop_after="lift_test")` → execute → `verify_grasp()`; if held, `lift()`
and re-verify; otherwise `release()` and the next candidate. Without `candidates` it asks
`grasps(obj, target=, point=, near=)` again after each failure (the scene changed) and skips
grasps it already tried; with `candidates` it walks your list. Only executed tries count against
`max_tries`; a candidate that does not plan is skipped.

`place_verified(obj=None, target=None, point=None, candidates=None, max_tries=2, options=None,
tol_m=0.04)` → `Verified`. **motion.** `places` → `plan_place` → execute, then checks
that nothing is held and the gripper is open, `reperceive()`s, and requires a freshly perceived
object of the held object's size at the predicted pose (within `tol_m`) and, with a target, in or
on it by the perceived boxes (`rh.relation(a, b, min_overlap=0.8, tol=0.015)` → `Relation`:
`.inside`, `.on_top`, `.overlap`, z values; truthy if inside or on top). Perception does not
re-track an **agent-defined** object (`segment`/`define_object`), so for one it re-segments at the
predicted pose; if that finds nothing matching, the result is "cannot verify an agent-defined
object": look (render, annotate) or define it again. It tries the next candidate only while still
holding. When the release ran but the result cannot be confirmed (perception does not find the
object, or the episode ended right after the release), `.ok` is **None, not False**: "could not
confirm", not a failed place. Look at the images (or, if the episode is over, stop) rather than
placing again.

`Verified`: truthy only when `.ok` is True (`.ok` is True, False, or None for "could not
confirm"); `.object` (its id now), `.chosen` (the candidate that worked), `.message`, `.steps_left`, `.attempts` (one dict per try: `stage` `plan`/`lift_test`/`place`,
`ok`, `why`, `gripper_measured`, `gripper` `closed`/`empty`/`jammed`, `wrist_points`, `lift`,
`released`, `dist_m`, `relation`, `steps`, `wrist` = PIL wrist-camera image after the lift test,
when the robot has a wrist camera).
`print(res)` gives a summary. **The box checks are a hint, not proof: still look at the images.**

`verify_grasp(expected_closure=None, min_wrist_points=15)` → `{"ok", "why", "held",
"gripper_measured", "gripper", "wrist_points"}`: ok = held and a partial closure (neither jammed
nor `empty`); `gripper` is `empty` at a reading near the gripper's `closed_empty`, which always
fails, whatever the wrist view, even when `held` names the object (a thin edge gripped fully shut
reads not ok, and so does `pick_verified`); also **not ok** when the fingers closed more than
0.25 past `expected_closure` (the candidate's `closure`; within 0.15 of `closed_empty` with none
given) and the wrist depth shows nothing between the pads (closed on air). Pass `expected_closure` when you verify by hand.
`fingers_view()` → that wrist-depth point count (None without a wrist camera or depth, or when
the gap is too narrow to tell an object from the pads).

## Examples of the assisted calls

Verified pick and place (`obj5` stands for the target the instruction names):

```python
res = ep.pick_verified("obj3", target="obj5")
print(res)
if res:
    if "wrist" in res.attempts[-1]:
        show(res.attempts[-1]["wrist"], "wrist after the lift test")
    pl = ep.place_verified(res.object, target="obj5")
    print(pl)
    show(ep.annotate(cam, objects=True), "after the place")
```

Staged pick by hand (choose the grasp yourself; look before lifting):

```python
wrist = next((c["name"] for c in emb["cameras"] if c["role"] == "wrist"), None)
cands = ep.grasps("obj3", near=objs["obj3"].top - [0, 0, 0.03])   # contact 3 cm below its top
show(ep.render(rh.View.look_at(objs["obj3"].centroid, 200, 40, 0.45),
               overlays={"grasps": cands[:4]}), "grasps")
if cands.best is None:
    print(cands.message)
else:
    pv = ep.plan_pick("obj3", cands.best, stop_after="lift_test")
    print(pv["ok"], rh.explain(pv), pv.get("n_steps"))
    show(ep.render_plan(pv), "pick plan")
    if pv["ok"]:
        ep.run(pv)
        v = ep.verify_grasp(cands.best["closure"]); print(v)
        if wrist:
            show(ep.annotate(wrist), "lift test")
        ep.lift(height_m=0.12) if v["ok"] else ep.release()
```

Push (an object too wide to grasp, or one lying flat):

```python
o = objs["obj2"]
start = o.centroid - [0.10, 0, 0]                  # behind it, pushing along +x
r = ep.push(start, o.centroid + [0.08, 0, 0], height_m=0.03)
print(r["last_chunk"]); objs = ep.reperceive()
```

Move while holding (collision-checked with the held object), then release by hand:

```python
tcp = ep.observe()["robot"]["tcp_pose"]
ep.move(rh.offset_pose(tcp, dx=0.1, dz=0.05))
ep.release(retreat_m=0.06)
```

Two arms (skip on a one-arm robot): each arm picks with its own handle:

```python
names = [e["name"] for e in emb["end_effectors"]]
if len(names) > 1:
    a, b = ep.ee(names[0]), ep.ee(names[1])
    ga = a.grasps("obj3"); gb = b.grasps("obj4")
    if ga.best and gb.best:
        print(a.pick_verified("obj3", candidates=ga), b.pick_verified("obj4", candidates=gb))
```
