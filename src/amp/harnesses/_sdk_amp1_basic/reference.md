# SDK reference

`rh` is the robot SDK, `ep` your episode. Every call below is a method of `ep` unless it is
written `rh.…`. Everything here is offline (no steps) except the calls marked **motion**.

**The robot.** This API drives whatever robot the episode runs. Read it before you plan anything:
`emb = ep.observe()["embodiment"]` describes it, and nothing here assumes a particular arm,
gripper or camera rig.

| field | what it says |
|---|---|
| `emb["name"]` | the robot's name |
| `emb["cameras"]` | one dict per camera: `name`, `width`, `height`, `depth` (has a depth map), `role`: `"scene"` cameras look at the workspace, the `"wrist"` camera rides on the gripper |
| `emb["arms"]` | one dict per arm: `name`, `joints` (in `q` order), `q_min`, `q_max`, `q_home` (rad). **`q` is every arm's `joints` concatenated in the order of `emb["arms"]`**, so its length is the total joint count (one arm: that arm's `len(joints)`; through `ep.ee(name)` a `q` may be just that end effector's arm) |
| `emb["end_effectors"]` | one dict per gripper (a two-arm robot has two; the first is the primary, the one `ep` itself drives): `name`, `arm`, `closing_axis` and `approach_axis` (the TCP axes, `"x"`/`"y"`/`"z"`), `gripper` {`kind`, `model`, `stroke_m` (the full opening), `max_width_m` (the widest object it can close on), `closed_empty` (the reading when closed on nothing), `held_below` (a closed reading below this means something is held)}; **`gripper` may be `null`**: a tool with no fingers (a wiper), which has no gripper fields in `robot`, and `gripper` and `gripper_both` raise `HarnessError` naming it, while motion, `fk`, `ik` and `check` work |
| `emb["workspace"]` | `support` (what objects rest on: `"table"`, `"floor"`, `"none"`) and `box` `[xmin, xmax, ymin, ymax, zmin, zmax]`, where the robot works |

**Conventions.** Metres, robot-root frame (+x away from the robot, +y left, +z up). Pose7 =
`[x, y, z, qw, qx, qy, qz]`. TCP = between the finger pads; the end effector's `approach_axis` is the TCP axis pointing out of the gripper, between the fingers, and
its `closing_axis` the one the fingers close along. The pose helpers (`rh.top_down`, `rh.side_pose`,
`rh.pose_from_axes`, `rh.finger_yaw`, `rh.approach_axis`) follow those declared axes once you have
called `observe()`. Gripper command 0 open, 1 closed, whatever the robot; `gripper_measured` 0 open
.. 1 closed, and `robot["gripper_gap_m"]` the finger gap in metres. Right after a close, **a reading
near the gripper's `closed_empty` means nothing between the fingers, ≈0.0 means the fingers stayed
open**: jammed on top of something, or an object about as wide as the opening wedged between them
(look at the wrist camera to tell which), and a reading below `held_below` means something is held
(something only a few mm thick can read as empty). `max_width_m` is the widest object the fingers surely close on;
up to `stroke_m` (the full opening) an object can still be held wedged. Where the gripper's
geometry is modelled, `gripper["tip_reach_m"]` says how far the fingertips reach past the TCP
along the approach axis: `open`, and at most while `closing`. So a closing top-down gripper's
fingertips reach `tip_reach_m["closing"]` below its TCP.
`obj` arguments take an id string or an `Obj`; `tcp` a Pose7 or a 4x4.

**Errors.** A refused call raises `rh.HarnessError`: `.message` is the server's reason **with every
error issue appended** (print it, it says what to fix), `.status` the HTTP status, `.reply` the
JSON. Once the episode has ended (its steps or time ran out, or it was finished) every call,
offline ones too, raises `rh.EpisodeOver`, a `HarnessError` subclass: let it end your script
rather than catching it with the `HarnessError`s you retry. `rh.explain(reply)` is the same
one-line reason for a reply you hold (e.g. a preview with `ok: false`); `rh.issues(reply)` lists
its error messages (`severity=None`: warnings too).

**Renumbered ids.** Perception can renumber an object after a motion. When you name an old id,
the server uses its successor, the reply carries `"resolved": {"from", "to", "why"}`, the SDK
prints a note (`obj7 is now obj12 (...)`) and appends it to `ep.resolved`. Use the
new id from then on. If there is no successor you get a 409 `unknown object`: call `objects()` and
find it again by position, size or colour (`objs.find`, `objs.nearest`).

**Two arms.** When `emb["end_effectors"]` has more than one entry, everything above still holds
for the primary end effector, and nothing changes in a script that only uses `ep`. To drive the
others:

- `h = ep.ee(name)` (a name from `emb["end_effectors"]`) is a handle with the same calls as `ep`
  that act on that end effector: `preview`, `execute`, `move_to`, `gripper`, `hold`, `home`, `fk`,
  `ik`, `ik_batch`, `check`, `check_batch`, `robot`. A `q` through the handle is that end
  effector's arm (`len(arm["joints"])` values) or the full `q`; a `q` on `ep` itself is the full
  `q` (or, for the primary, its arm). `h.top_down`, `h.side_pose`, `h.pose_from_axes`,
  `h.finger_yaw` and `h.approach_axis` follow that end effector's axes (the grippers may differ).
  Perception, rendering, `observe`, `wait` and the flow calls are the same through any handle.
- `observe()["robot"]` keeps the primary's `q` (the full one), `tcp_pose` and gripper fields, and
  adds `robot["end_effectors"] = {name: {"ee_pose", "tcp_pose", "gripper_cmd",
  "gripper_measured", "gripper_gap_m", "q_slice": [lo, hi]}}`: `q[lo:hi]` is that end effector's
  arm.
- `ep.move_both({name: pose7, ...}, duration, grippers={name: 0|1})` moves several TCPs at once
  (an end effector not named holds), previewing first and refusing on errors like `move_to`;
  `ep.preview_both({name: [waypoints], ...}, interp="minjerk")` and `ep.execute_both(
  {"ee_waypoints": {name: [waypoints]}, "interp": "minjerk"})` are `preview` / `execute` for
  several end effectors, and `ep.gripper_both(closed)` closes or opens every gripper.
  `preview_both`'s reply has `"schedule"`: whether the arms would move `"simultaneous"`ly or
  `"serial"`ly (one after the other, because their paths would collide). `move_both` and
  `execute_both` return what `move_to` and `execute` do. Poses are in the same robot-root frame as everything else.
  A waypoint without `gripper` keeps that end effector's own command.

## Mobile base and joint groups (only for a robot whose description has a `base`)

A robot without a `base` stands where it is: skip this section (`base_pose()` returns None and
`move_base` is refused). A robot with one has a holonomic base and may have joint groups (a torso
that carries the arms). `emb["base"]` names the base; `emb["joint_groups"]` lists each group
(`name`, `joints`, `q_min`, `q_max`, `carries`: the arms it moves). **`q` is every arm's joints,
then every group's** (a torso's joints come after both arms). Poses everywhere else stay in the
robot-root frame, which moves with the base.

| call | effect |
|---|---|
| `base_pose()` | `T_world_base` 4x4, the robot root in the world; read it before and after a base move |
| `move_base(dx=0, dy=0, dyaw=0)` | **motion**: drive the base `dx`, `dy` m and turn `dyaw` rad in its own frame (x ahead, y left), arms and torso held. Refused with the reason (`base_blocked`: an occupied cell, or floor no camera sees now, along the footprint it sweeps); returns the reached `T_world_base`. Every object pose you hold moves in the root frame by the inverse of the move: observe again after it |
| `move_joints(group, q, duration=2.0)` | **motion**: joint-space move of one arm (its name or its end effector's) or a joint group such as `"torso"` to `q`; everything else holds. A torso move carries both arms and the cameras on it; arm and torso moves are checked against the perceived scene and the robot's own body. Arm plans afterwards start from the torso where it is; no arm plan ever moves the torso |

## Seeing

| call | returns |
|---|---|
| `observe()` | dict: `instruction`, `mode` (`awaiting_plan` = idle, `executing`, `episode_over`), `step`, `max_steps`, `steps_left`, `control_dt` (seconds per step), `frame_seq`, `embodiment` (the robot, above), `robot` {`q`, `gripper_cmd`, `gripper_measured`, `gripper_gap_m`, `ee_pose`, `tcp_pose`, `end_effectors` (two-arm only, above), `q_min`, `q_max`, `q_home`}, `cameras` {name: `K`, `T_world_cam`, sizes}, `held` (None or {`object`, `reason`}), `last_chunk` {`kind`, `status` `done`/`stopped`, `n_steps`, `message`}, `objects`, `perception_notes` |
| `rgb(cam)` | uint8 (H, W, 3) of any camera the embodiment lists (`[c["name"] for c in emb["cameras"]]`) |
| `depth(cam)` | float32 (h, w) metres, lower resolution than RGB; 0 = no depth |
| `camera(cam, depth=False)` | `rh.Camera`: `.project(xyz) -> (uv, z)`, `.unproject(u, v, z)`, `.ray(u, v)`, `.K`, `.T_world_cam` (current pose; a wrist camera moves) |
| `cloud(cams=None, stride=1, max_depth=None, crop=True)` | `(xyz (N,3) float32, rgb (N,3) uint8, cam_index (N,))`, `.cams` names the indices; `max_depth=None` lets the server cut off at 2 m, or for a scene camera with `crop=True` at the farthest corner of the workspace box (CaP-X's two-arm camera is over 2 m from the workspace); pass a number to cut off there |
| `pick(cam, u, v)` | `{"point", "depth", "normal"}`: the 3D point under a full-resolution pixel; on a depth hole the nearest valid pixel within 16 px (`"offset_px"` says which). **Not a robot pick.** |
| `object_at(point, max_dist=0.03)` | `{"id", "distance_m"}`: the object at a 3D point (id None if none) |
| `object_at_pixel(cam, u, v)` | `{"id", "distance_m", "point"}` |

## Objects

`objects()` → `Objects`, a list of `Obj` that is also indexed by id (`objs["obj3"]`,
`objs.get("obj3")` → None if absent), with `.ids()`, `.notes` (perception notes) and:

- `objs.find(label=None, color=None, kind=None, near=None, min_size=None, max_size=None)` →
  `Objects` matching all filters, nearest to `near` first (`color` a basic name like `"red"`;
  sizes in m, scalar or 3-vector against the aabb extent; `kind` `"object"` or `"fixture"`).
- `objs.nearest(point, kind=None, max_dist=None)` → `Obj` or None.

`Obj` fields: `.id`, `.label` (e.g. `"red · 5x5x5 cm"`), `.color` (RGB), `.kind` (`"object"`;
`"fixture"` only for objects you define as one), `.source` (`"perception"` or `"agent"`),
`.centroid` (3,),
`.aabb` [[lo], [hi]], `.size` (3,) aabb extent, **`.top` = the aabb's top-face centre, a 3-vector
[x, y, z]** (use `.top_z` / `.bottom_z` for heights, never `round(o.top)`), `.obb` (an `rh.OBB`,
below), `.n_points`, `.verts`/`.faces`
(world mesh), `.to_json()`.

Perception works from depth only: it can merge touching objects, split one object, or fill hidden
parts wrongly; check ids against the images. The robot's body is removed. A held object may drop
out of `objects()`. Call `objects()` again after each motion.

### Fixing perception (no steps)

| call | does |
|---|---|
| `segment(cam, u, v, radius_mm=11, color_tol=None)` | region-grows one object from the 3D point under pixel (u, v) of the latest fused cloud (robot and support surface removed) and adds it as an agent object → `Obj`. Click on the object you mean in a real image. `color_tol` (0.08 works) also stops at colour edges: it splits touching objects of different colours. |
| `define_object(aabb=None, obb=None, points=None, label=None, kind=None, replace=())` | adds an agent object from the cloud points inside an aabb `[[lo],[hi]]` or an obb (`rh.OBB` or `{"center", "size", "yaw_deg"}`), or from your own `points` (N, 3); `replace=["obj4"]` removes those ids (split a merged segment into two `define_object`s that replace it) → `Obj` |
| `forget_object(obj)` | removes an object from the scene (a phantom, or a fragment); it is not re-added there → `{"ok"}` |
| `object_points(obj)` | its cloud points: `(xyz float32 (N,3), rgb uint8 (N,3))` |
| `reperceive()` | runs perception now on the current frame → `Objects` (use it instead of moving `home()` to refresh) |

Agent objects persist across perception passes and work everywhere an id does (`check`,
rendering). Perceived objects overlapping an agent object by more than half are suppressed.

## Geometry utilities (`rh.…`, pure numpy)

| call | returns |
|---|---|
| `rh.OBB(center, size, yaw_deg)` | z-up oriented box: `.R`, `.long_axis`/`.short_axis` (unit horizontal), `.long_yaw_deg`, `.top_z`, `.bottom_z`, `.corners()` (8,3), `.footprint()` (4,2), `.aabb()`, `.local(pts)`, `.contains(pts, margin=0)`, `.to_json()` |
| `rh.fit_obb(points, robust=0.0)` | OBB of points: minimum-area footprint, x = long side (`robust=0.01` trims outliers) |
| `rh.pca_axes(points)` | `(mean, axes (3,3) columns longest first, half_extents)` |
| `rh.footprint_overlap(a, b)` | fraction 0..1 of a's footprint over b's (Obj, OBB or dict) |
| `rh.points_in(xyz, aabb=None, obb=None, center=None, radius=None, margin=0)` | bool mask of points in a box and/or a sphere |
| `rh.heightmap(xyz, lo, hi, res=0.005)` | `(H (ny,nx) max z, NaN empty, xs, ys)` over an xy rectangle |
| `rh.fit_circle(xy)` | `(cx, cy, r)`: a circle through points' xy |
| `rh.top_down(pos, yaw_deg=0)` | Pose7 approaching straight down, fingers closing along yaw (0 = world x, 90 = world y) |
| `rh.side_pose(pos, azimuth_deg, tilt_deg=0, closing="horizontal")` | approach horizontally along azimuth (0 = +x, away from the robot), tilted down by tilt_deg; `closing="vertical"` closes up-down |
| `rh.pose_from_axes(pos, approach, closing=None)` | Pose7 whose approach axis is `approach` and closing axis `closing` (made orthogonal; None: a horizontal one) |
| `rh.offset_pose(pose7, dx, dy, dz, frame="world"/"tool")`, `rh.finger_yaw(pose7)`, `rh.approach_axis(pose7)` | pose helpers |
| `rh.pose_to_T`, `rh.T_to_pose`, `rh.quat_to_mat`, `rh.mat_to_quat`, `rh.rpy_to_quat`, `rh.quat_to_rpy`, `rh.make_pose(pos, quat)`, `rh.pose_from_rpy(pos, rpy)` | conversions (quaternions wxyz) |
| `rh.backproject(depth, camera, min_depth, max_depth)` | world points of a depth map (use `camera(cam, depth=True)`) |
| `rh.color_name(rgb)` | basic colour name |

## Rendering (no steps)

| call | returns |
|---|---|
| `render(view="iso", layers=..., overlays=..., size=(960, 540), scene=None)` | PIL image of a 3D view |
| `scene(cloud_stride=1, cams=None)` | everything the renderer draws (cloud, objects, robot), fetched once to reuse as `scene=` |
| `views(names=("iso","top","front","side"), target=None, distance=None)` | one grid of several views |
| `closeup(obj, n_views=4, size=(480, 360))` | one grid: renders orbiting an object (highlighted, obb drawn) plus crops of the scene cameras' real images around it (`cams=[...]` names others) |
| `render_plan(preview, view=None, ghosts="segments")` | a plan's TCP path (blue start → red end) with ghost robots at its segment ends |
| `annotate(cam, points=, labels=, boxes=, plan=, poses=, objects=True)` | the camera's real image with 3D items drawn on it |
| `rh.crop(img, center_uv, size=(320, 180))`, `rh.grid(images, cols=2, titles=None)`, `rh.depth_colormap(depth)` | image helpers |

`points` (in `annotate` and in render overlays alike): an (N, 3) array, a list of xyz, dicts
`{"xyz", "label", "color", "size"}`, or `Obj`s (drawn at the centroid with the id). `boxes`: aabbs,
`rh.OBB`s, pixel boxes `[u0, v0, u1, v1]` (annotate only), or dicts `{"aabb"|"obb"|"box", "label",
"color"}`.

Views: `"iso"`, `"top"` (robot-forward is up), `"front"` (facing the robot), `"side"` (from its
left), `"back"`, a camera name (exactly what it sees), or `rh.View.look_at(target, azimuth_deg,
elevation_deg, distance, fov_deg=50)` (azimuth ccw from +x: 0 looks back at the robot, 180 from
behind it; elevation 90 = straight down). Layers (default `cloud, objects, robot, gizmo,
labels`; also `gripper`, `frames`, `cameras`). Overlays: `plan`, `ghost_q` (joint vectors), `points`, `boxes`,
`frames` (Pose7s), `grippers` ({`tcp`, `opening`, `label`}), `text`, `highlight` (ids; others
dimmed). Re-fetch `scene()` after a motion.

## Motion

| call | effect |
|---|---|
| `preview(waypoints, interp="minjerk", speed_scale=None)` | checks a TCP trajectory without moving: `ok`, `issues`, `duration`, `min_duration`, `tcp_fk`, `q` |
| `execute(waypoints or {"waypoints": [...]})` | **motion**; refused while it has an error issue |
| `move_to(pose7, duration, gripper=None)` | **motion**: one waypoint, preview, refuse on errors (quoting `min_duration` when too fast), execute, wait |
| `gripper(closed, duration=1.0)`, `hold(seconds)`, `home(duration=3.0)` (to `q_home`) | **motion** |
| `stop()` | abort the running motion |

Waypoint: `{"t": s from start, "pos", "quat", "gripper": 0|1}`, `t` increasing, at most 60
(each executed second costs `1 / control_dt` steps);
`rh.waypoint(t, pose7, gripper)`, `rh.waypoint_top_down(t, pos, yaw_deg, gripper)`,
`rh.waypoint_rpy(t, pos, rpy)`. Issue kinds: `unreachable`, `velocity` (too fast: lengthen `t`
past `min_duration`), `joint_limit`, `singular`, and while something is held
`collision` (the held object against the scene). **Previews do not check the arm against
objects**: `check_batch` the preview's `q` (every few steps; `held=` while something is held).
`execute(preview)` runs the waypoints it was made from.

## Kinematics and checks (no steps)

| call | returns |
|---|---|
| `fk(q=None)` | `{"tcp_pose", "ee_pose", "links"}` |
| `ik(tcp, seed=None, max=4)` | `{"reachable", "solutions": [{"q", "pos_err_mm", "rot_err_deg", "limit_margin_rad", "singular"}]}` |
| `ik_batch(tcps, seed=None)` | one `ik` reply per pose (any number; sent 256 at a time) |
| `check(q=None, tcp=None, held=None, ignore=())` | `{"reachable", "collision_free", "in_limits", "contacts": [{"link", "obstacle", "clearance_m"}]}` against perceived objects; `held=None` what is held now, `held=False` nothing, `held="objN"` that object; `ignore` object ids. The arm and gripper are modelled by spheres, which near the fingertips reach centimetres past the real fingers: a gripper contact there does not mean the fingers touch the object (judge that from `tip_reach_m`), while contacts of the arm's links are real |
| `check_batch(qs, held=None, ignore=())` | one `check` reply per joint vector |
| `robot(q=None)` | collision spheres, for drawing |

## Flow and budget

| call | returns |
|---|---|
| `budget()` | `{"step", "steps_left", "max_steps", "motion_s_left", "wall_left_min", "idle_timeout_s"}` |
| `rh.snapshot(ep, Path("snap0"))` | writes every camera's RGB and depth, 3D views, `objects.json` and `observation.json` into `snap0/` (then `show()` the PNGs you need) |
| `wait(until="idle", timeout_s=120)` | the observation once no motion runs |
| `status()` | `{"state": "ready"/"over", "end", "idle_s", "idle_timeout_s", "wall_s", "episode_timeout_s"}` |
| `finish()` | end now, scored as it stands (prefer the `finish_episode` tool) |

## Examples

Read the robot, then look and identify:

```python
obs = ep.observe(); print(obs["instruction"], ep.budget())
emb = obs["embodiment"]
grip = emb["end_effectors"][0]["gripper"]                  # the primary's; None for a tool
n_joints = sum(len(a["joints"]) for a in emb["arms"])      # the length of q
print(emb["name"], [(c["name"], c["role"]) for c in emb["cameras"]], n_joints, "joints",
      grip["max_width_m"] if grip else "no fingers", "m max width" if grip else "")
scene = [c["name"] for c in emb["cameras"] if c["role"] == "scene"]
cam = scene[0]
objs = ep.objects()
for o in objs:
    print(o.id, o.kind, o.label, o.centroid.round(3), o.obb.size.round(3))
show(ep.annotate(cam, objects=True), f"ids on {cam}")
reds = objs.find(color="red")                      # by colour name, nearest first with near=
show(ep.closeup("obj3"), "obj3 close up")
```

Fix a merged or missed segment:

```python
img = ep.rgb(cam); show(img, "pick a pixel on the object")
a = ep.segment(cam, 400, 256)                      # the object under that pixel
print(a.id, a.label, a.obb)
b = ep.segment(cam, 320, 262, color_tol=0.08)      # stop at the edge of a touching box
# or split merged obj4 by two boxes from its points
xyz, rgb = ep.object_points("obj4")
left = xyz[xyz[:, 1] > 0.12]; right = xyz[xyz[:, 1] <= 0.12]
o1 = ep.define_object(points=left, label="part L", replace=["obj4"])
o2 = ep.define_object(points=right, label="part R")
show(ep.annotate(cam, objects=[o1, o2]), "split")
```

Many IK/collision queries at once:

```python
poses = [...]  # your candidate TCP poses (Pose7s)
sols = ep.ik_batch(poses)
qs = [s["solutions"][0]["q"] for s in sols if s["reachable"]]
free = [c["collision_free"] for c in ep.check_batch(qs, held=False)]
```

Motion (10 cm up in 2 s, then open the gripper):

```python
tcp = ep.observe()["robot"]["tcp_pose"]
pv = ep.preview([rh.waypoint(2.0, rh.offset_pose(tcp, dz=0.10))])
print(pv["ok"], rh.explain(pv))
if pv["ok"]:
    ep.execute(pv)
ep.gripper(False)
```

Two arms (skip on a one-arm robot): read each end effector, then lift both together:

```python
names = [e["name"] for e in emb["end_effectors"]]
if len(names) > 1:
    a, b = ep.ee(names[0]), ep.ee(names[1])
    ees = ep.observe()["robot"]["end_effectors"]
    print({n: (e["tcp_pose"][:3], e["q_slice"], e["gripper_measured"]) for n, e in ees.items()})
    up = {n: rh.offset_pose(e["tcp_pose"], dz=0.10) for n, e in ees.items()}
    pv = ep.preview_both({n: [rh.waypoint(3.0, p)] for n, p in up.items()})
    print(pv["ok"], pv.get("schedule"), rh.explain(pv))
    if pv["ok"]:
        ep.move_both(up, 3.0)                         # raises with the reasons if refused
    qa = a.ik(up[names[0]])["solutions"][0]["q"]      # an arm-long q (ep.fk / ep.check take the full q)
```
