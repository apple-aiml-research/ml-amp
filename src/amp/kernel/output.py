#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What a kernel call returns, and how the child's captured output becomes model-facing text."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from amp.fs.safe_files import LinkRefused, safe_read_bytes

CLIP_CHARS = 20_000
CAPTURE_DIR = "tmp/kernel"


@dataclass
class KernelResult:
    stdout: str
    stderr: str
    error: str | None
    images: list[dict] = field(default_factory=list)
    duration_s: float = 0.0
    timed_out: bool = False


def clip(text: str, limit: int = CLIP_CHARS, total: int | None = None) -> str:
    """Head and tail of text within `limit` chars, with a marker saying what was cut."""
    total = len(text) if total is None else total
    if len(text) <= limit and total <= len(text):
        return text
    head = limit * 3 // 5
    tail = limit - head
    cut = total - head - tail
    return (text[:head] + f"\n... [truncated: {cut} of {total} chars omitted] ...\n"
            + text[-tail:])


def part_text(part: dict, note: str = "") -> str:
    """A child's {"text", "size"[, "tail"]} capture as clipped text, plus note."""
    text = part.get("text", "")
    if "tail" not in part:
        return clip(text) + note
    total = int(part.get("size", len(text)))
    text = text + part["tail"]
    return clip(text, total=max(total, len(text))) + note


def from_reply(msg: dict, duration_s: float, *, timed_out: bool = False,
               note: str = "") -> KernelResult:
    """The KernelResult for a child's result message."""
    return KernelResult(stdout=part_text(msg["stdout"], note), stderr=part_text(msg["stderr"]),
                        error=None if msg.get("error") is None else clip(msg["error"]),
                        images=list(msg.get("images") or []), duration_s=duration_s,
                        timed_out=timed_out)


def mark_restarted(r: KernelResult, before: str | None) -> KernelResult:
    """Prefix error with KernelRestart when the child was restarted before this call ran."""
    if before:
        r.error = f"KernelRestart: {before}" + (f"\n{r.error}" if r.error else "")
    return r


def read_capture(episode_dir: Path, n: int, kernel_uid: int | None = None) -> tuple[str, str]:
    """(stdout, stderr) the child had captured for call n when it was lost; files removed."""
    out = []
    for kind in ("out", "err"):
        p = Path(episode_dir) / CAPTURE_DIR / f"{kind}_{n}.txt"
        try:
            data = safe_read_bytes(p, episode_dir, limit=2 * CLIP_CHARS, kernel_uid=kernel_uid)
            out.append(data.decode("utf-8", "replace"))
        except LinkRefused as e:
            out.append(f"[{e}]\n")
        except OSError:
            out.append("")
        try:
            p.unlink()
        except OSError:
            pass
    return out[0], out[1]
