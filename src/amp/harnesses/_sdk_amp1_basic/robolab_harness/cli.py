#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""CLI: `python -m robolab_harness snapshot|render|status|observe|objects|budget ...`."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

from .client import Episode, HarnessError
from .geometry import STANDARD_VIEWS, View
from .render import ALL_LAYERS, DEFAULT_LAYERS, depth_colormap, grid


def _episode(args) -> Episode:
    if args.worker_url:
        return Episode(worker_url=args.worker_url)
    return Episode.from_file(args.episode)


def _floats(text: str, n: int) -> list:
    vals = [float(v) for v in text.split(",")]
    if len(vals) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated numbers, got {text!r}")
    return vals


def snapshot(ep: Episode, out: Path, size=(960, 540),
             views=("iso", "top", "front", "side")) -> dict:
    """Write every camera's RGB and depth, standard 3D views, objects.json and observation.json."""
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()
    sd = ep.scene()
    obs = ep._last_obs()
    files = []
    rgbs = []
    for cam in obs.get("cameras") or {}:
        img = Image.fromarray(ep.rgb(cam))
        img.save(out / f"rgb_{cam}.png")
        rgbs.append(img)
        files.append(f"rgb_{cam}.png")
        try:
            depth_colormap(ep.depth(cam)).save(out / f"depth_{cam}.png")
            files.append(f"depth_{cam}.png")
        except HarnessError as e:
            print(f"no depth for {cam}: {e.message}", file=sys.stderr)
    if rgbs:
        grid([i.resize((640, int(640 * i.height / i.width))) for i in rgbs], cols=len(rgbs),
             titles=list(obs["cameras"])).save(out / "cameras.png")
        files.append("cameras.png")
    for name in views:
        ep.render(View.standard(name), scene=sd, size=size,
                  layers=DEFAULT_LAYERS).save(out / f"view_{name}.png")
        files.append(f"view_{name}.png")
    (out / "objects.json").write_text(json.dumps([o.to_json() for o in sd.objects], indent=1))
    (out / "observation.json").write_text(json.dumps(obs, indent=1))
    files += ["objects.json", "observation.json"]
    return {"out": str(out), "files": files, "seconds": round(time.monotonic() - t0, 2),
            "objects": [f"{o.id}: {o.label}" for o in sd.objects], "step": obs.get("step"),
            "steps_left": obs.get("steps_left"), "mode": obs.get("mode")}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m robolab_harness")
    p.add_argument("--episode", default="episode.json", help="episode file (./episode.json)")
    p.add_argument("--worker-url", help=argparse.SUPPRESS)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("snapshot", help="camera images, depth maps, 3D views and JSON into DIR")
    s.add_argument("--out", required=True)
    s.add_argument("--size", default="960,540")

    r = sub.add_parser("render", help="render one 3D view to a PNG")
    r.add_argument("--target", default="0.5,0,0.05", help="x,y,z the view looks at")
    r.add_argument("--azimuth", type=float, default=215.0,
                   help="deg ccw from +x (180 = behind robot)")
    r.add_argument("--elevation", type=float, default=38.0, help="deg above the horizontal")
    r.add_argument("--distance", type=float, default=1.4, help="metres from the target")
    r.add_argument("--fov", type=float, default=50.0)
    r.add_argument("--view", help=f"a standard view {sorted(STANDARD_VIEWS)} or a camera name")
    r.add_argument("--layers", default=",".join(DEFAULT_LAYERS),
                   help=f"comma-separated subset of {','.join(ALL_LAYERS)}")
    r.add_argument("--highlight", default="", help="comma-separated object ids to emphasise")
    r.add_argument("--size", default="960,540")
    r.add_argument("--out", required=True)

    sub.add_parser("status", help="the episode's status (state, end, idle time)")
    sub.add_parser("observe", help="print the latest observation JSON")
    sub.add_parser("objects", help="print the perceived objects")
    sub.add_parser("budget", help="steps and wall time left")
    args = p.parse_args(argv)
    ep = _episode(args)

    if args.cmd == "snapshot":
        size = tuple(int(v) for v in args.size.split(","))
        print(json.dumps(snapshot(ep, Path(args.out), size), indent=1))
    elif args.cmd == "render":
        if args.view:
            view = (View.standard(args.view) if args.view in STANDARD_VIEWS
                    else View.camera(args.view))
        else:
            view = View.look_at(_floats(args.target, 3), args.azimuth, args.elevation,
                                args.distance, args.fov)
        layers = [x for x in args.layers.split(",") if x]
        ov = {"highlight": [x for x in args.highlight.split(",") if x]}
        size = tuple(int(v) for v in args.size.split(","))
        t0 = time.monotonic()
        ep.render(view, layers=layers, overlays=ov, size=size).save(args.out)
        print(json.dumps({"out": args.out, "seconds": round(time.monotonic() - t0, 2)}))
    elif args.cmd == "status":
        print(json.dumps(ep.status(), indent=1))
    elif args.cmd == "observe":
        print(json.dumps(ep.observe(), indent=1))
    elif args.cmd == "objects":
        print(json.dumps([o.to_json() for o in ep.objects(ep.observe())], indent=1))
    elif args.cmd == "budget":
        print(json.dumps(ep.budget(), indent=1))
    return 0


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    sys.exit(main())
