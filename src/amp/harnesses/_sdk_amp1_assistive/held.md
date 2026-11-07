# What is held

After every motion the server reads the gripper: when it settles closed on a
perceived object at the TCP, `observe()["held"]` names it (`{"object", "reason"}`), however the
fingers got there (your own waypoints and `gripper(True)` too). A close that reads fully shut (on
something thin) is not taken as held from the gripper reading alone. When the grip is lost or the gripper opens, `held` goes back to None, and after the
motion that lost it (waited, the default) `reply["last_chunk"]["dropped"]` and
`observe()["last_chunk"]["dropped"]` say so: `"obj3"`. The motion's reply has no top-level
`"dropped"`. A hold the gripper never read as holding (something thin) is not dropped by a fully
shut reading alone. Read `held` after a close to know whether it took the object, and after each
move while carrying it.
