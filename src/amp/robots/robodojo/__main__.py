#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""python -m amp.robots.robodojo: the RoboDojo launcher for one slot."""

import sys

from amp.robots.robodojo.client import main

if __name__ == "__main__":
    sys.exit(main())
