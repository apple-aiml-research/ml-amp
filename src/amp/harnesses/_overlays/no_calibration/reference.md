**No camera calibration.** Observations name each camera and give its image size (`width`,
`height`), but no intrinsics or extrinsics (`K`, `K_depth`, `T_world_cam`), whatever the rest
of this reference says, the paragraphs above included. `camera(cam)` and its `.ray(u, v)`,
`annotate()` and renders of a real camera's view fail: a pixel cannot be turned into a ray or a
3D point, so nothing can be triangulated across cameras either. What remains is the images
(where you see them), the robot's state, its model and kinematics (`fk()`, `ik()`) and
collision checks.
