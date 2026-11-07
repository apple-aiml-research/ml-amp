#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Filesystem safety every layer shares: link-refusing reads and atomic writes. Stdlib only."""

from amp.fs.atomic import write_bytes_atomic, write_json_atomic
from amp.fs.safe_files import (
    LinkRefused,
    is_safe_file,
    safe_copy,
    safe_open,
    safe_read_bytes,
    safe_read_text,
    walk,
)

__all__ = ["LinkRefused", "is_safe_file", "safe_copy", "safe_open", "safe_read_bytes", "safe_read_text",
           "walk", "write_bytes_atomic", "write_json_atomic"]
