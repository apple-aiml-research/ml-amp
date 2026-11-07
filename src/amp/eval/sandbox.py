#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The sandbox entry of EpisodeResult: the kernel sandbox self-test (once per process per backend
and interpreter) checked against the backend the episode's kernel actually started."""

from __future__ import annotations

import dataclasses
import threading
import time

from amp.kernel.sandbox import resolve_backend, self_test

_CACHE: dict[tuple[str, str], dict] = {}
_LOCK = threading.Lock()


def self_test_report(backend: str, python: str) -> dict:
    """SandboxReport as a dict, from amp.kernel's self-test; run once, then cached."""
    try:
        name = resolve_backend(backend)
    except ValueError as e:
        return {"backend": backend, "ok": False, "checks": {}, "error": str(e)}
    key = (name, python)
    with _LOCK:
        if key not in _CACHE:
            t0 = time.monotonic()
            try:
                d = dataclasses.asdict(self_test(name, python=python))
            except Exception as e:  # noqa: BLE001
                d = {"backend": name, "ok": False, "checks": {}, "details": {},
                     "error": f"{type(e).__name__}: {e}"}
            d["duration_s"] = round(time.monotonic() - t0, 3)
            _CACHE[key] = d
        return dict(_CACHE[key])


def summary(report: dict) -> dict:
    """The self-test without its free-form details, for the unit record."""
    keep = ("backend", "ok", "checks", "platform", "duration_s", "error")
    out = {k: report[k] for k in keep if k in report}
    missing = (report.get("details") or {}).get("missing_required")
    if missing:
        out["missing_required"] = missing
    return out


def record(requested: str, report: dict | None, kernel: dict) -> dict:
    """EpisodeResult.sandbox: ok only if the self-test held for the backend the kernel ran."""
    st = summary(report) if report else None
    backend = kernel.get("backend") or (st or {}).get("backend")
    ok = bool(st and st.get("ok") is True and backend == st.get("backend"))
    out = {"backend": backend, "ok": ok, "requested": requested,
           "checks": (st or {}).get("checks", {}), "self_test": st}
    if "relay" in kernel:
        out["relay"] = kernel["relay"]
    return out
