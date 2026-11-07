#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The kernel's sitecustomize, after a "_CPUS = <n>" line: os.cpu_count() reports its CPU share.

Python 3.11 has no PYTHON_CPU_COUNT, and Pool(), the executors and scipy's workers=-1 size from
os.cpu_count(); the real values stay at os.real_cpu_count and os.real_sched_getaffinity.
"""

import importlib.machinery
import importlib.util
import os
import sys


def _install(cpus):
    if hasattr(os, "real_cpu_count"):
        return
    os.real_cpu_count = os.cpu_count
    os.cpu_count = lambda: cpus
    if hasattr(os, "process_cpu_count"):
        os.process_cpu_count = lambda: cpus
    if hasattr(os, "sched_getaffinity"):
        os.real_sched_getaffinity = os.sched_getaffinity

        def _affinity(pid, _real=os.real_sched_getaffinity):
            return set(sorted(_real(pid))[:cpus])

        os.sched_getaffinity = _affinity


def _chain():
    """Run the next sitecustomize on sys.path, the one this file shadows."""
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.machinery.PathFinder.find_spec(
        "sitecustomize", [p for p in sys.path if os.path.abspath(p or ".") != here])
    if spec is not None and spec.loader is not None:
        spec.loader.exec_module(importlib.util.module_from_spec(spec))


# Only as the interpreter's sitecustomize: importing this module from amp must patch nothing.
if __name__ == "sitecustomize":
    _install(globals().get("_CPUS", 4))
    _chain()
