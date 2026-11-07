#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The robot SDK with the assisted skills: the manual SDK (in _manual, unchanged) plus grasp and
place proposals, planned motion, multi-step plans and verified pick and place (_assist's layers)."""

import sys as _sys  # noqa: I001 - _assist's names must come after _manual's star import

from ._manual import *
from ._manual import __all__ as _MANUAL_ALL
from ._manual import _robot, _trace, cli, client, geometry, mobile, render, tlf
from ._assist.proposals import Candidates, top_down_across
from ._assist.verified import (  # noqa: F401 - Episode and connect replace _manual's
    Episode,
    Relation,
    Verified,
    connect,
    relation,
)

# `from robolab_harness.cli import snapshot` works here as on the manual SDK: the same modules.
for _m in (_robot, _trace, cli, client, geometry, mobile, render, tlf):
    _sys.modules.setdefault(f"{__name__}.{_m.__name__.rpartition('.')[2]}", _m)

__all__ = list(_MANUAL_ALL)
__all__ += ["Candidates", "Relation", "Verified", "relation", "top_down_across"]
