#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""python -m amp.robots.capgym: a capgym simulator, a link client of a robot server."""

from amp.robots.capgym import affinity

affinity.apply()

from amp.robots.capgym import boundary

boundary.install()

from amp.robots.capgym.client import main

raise SystemExit(main())
