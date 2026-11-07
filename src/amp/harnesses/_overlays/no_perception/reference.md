**No perception.** This harness gives you no perceived objects, whatever the prompt or the text
below says. The server's object detection, the scene it builds and the tools for editing it are
refused (`rh.HarnessError`, 403): `objects()`, `segment()`, `define_object()`,
`forget_object()`, `reperceive()`, `object_points()`, `object_at()` and `object_at_pixel()`.
Everything that draws that scene is refused with them: `scene()`, `render()`, `views()`,
`render_plan()`, `closeup()`, `rh.snapshot()` and `annotate(objects=...)`. `observe()` has no
`objects`, `perception_notes` or `held`, and `move_base()` is refused. So the prompt's "Objects
and ids" section and its example's `objects()` and `render_plan()` lines do not apply here.

You find the objects yourself, from `rgb()`, `depth()`, `camera()`, `cloud()` and `pick()`, with
the geometry utilities (`rh.points_in`, `rh.fit_obb`), and you plan every motion as TCP waypoints.
`annotate(cam, points=..., boxes=..., plan=...)` draws your own points, boxes and a preview on a
real camera image: use it to check what you located and where a motion goes. `check()`,
`check_batch()` and `preview()` check the robot against no objects, so they cannot tell you where
the objects are, and `held=` and `ignore=` take no object ids: keep clear of what you located
yourself. To know whether you are holding something, read `gripper_measured` against the
gripper's `closed_empty` and `held_below`, and look through the wrist camera.
