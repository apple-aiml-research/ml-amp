#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The four tools: their schemas, and the pure helpers that turn kernel output and image files
into tool_result content."""

from __future__ import annotations

import base64
import copy
import hashlib
import io
import json
from pathlib import Path

from amp.agents_text.surface import SELF_JUDGED
from amp.fs.safe_files import safe_read_bytes

OUT_CHARS = 20000
IMAGE_LONG_SIDE = 1280
PNG_MAX_BYTES = 400_000
IMAGE_POLICY = "jpeg_q95"
PNG_KEEP_BYTES = 64_000
IMAGE_MAX_BYTES = 64 << 20
MAX_SHOWN = 6
PRELOADED = ("rh", "ep", "np", "Image", "show")
PRELOADED_TEXT = {"rh": "rh (robolab_harness)", "ep": "ep (your connected episode)",
                  "Image": "Image (PIL)", "show": "show(x, caption=None) to see an image"}
STRICT_PRELOADED_TEXT = {**PRELOADED_TEXT,
                         "show": "show(camera=<name>, caption=None) to have a camera described"}
STRICT_PYTHON_TAIL = (", and a vision model's description of what changed whenever the robot "
                      "moved or you show()ed a camera (you see no images, and your code gets no "
                      "camera, depth or perception data).")
_VISION_TEXT = {"image": ("PRELOADED_TEXT", None),
                "vdm_strict": ("STRICT_PRELOADED_TEXT", "STRICT_PYTHON_TAIL")}
REPORT_FIELDS = ("outcome_belief", "interpretation", "plan", "steps", "key_images", "lessons")


def python_tool(preloaded: tuple[str, ...] = PRELOADED, vision: str = "image",
                texts: dict | None = None) -> dict:
    """The python tool, its description listing the names in `preloaded`, worded for the harness configuration's
    vision (AgentProfile.vision); texts, when given, replaces that vision's preloaded text."""
    text_name, tail_name = _VISION_TEXT[vision]
    text = globals()[text_name] if texts is None else texts
    tail = globals()[tail_name] if tail_name else " and the images you show()ed."
    names = ", ".join(text.get(n, n) for n in preloaded)
    pre = f"Preloaded: {names}. " if preloaded else ""
    return {
        "name": "python",
        "description": (
            "Run Python code in your persistent kernel (state survives between calls). "
            f"{pre}Returns stdout, stderr and the traceback "
            f"(each cut at 20000 characters){tail}"),
        "input_schema": {
            "type": "object",
            "properties": {
                "code": {"type": "string", "description": "Python source to run."},
                "timeout_s": {"type": "number", "description": "Interrupt after this many "
                              "seconds (default 600).", "default": 600},
            },
            "required": ["code"],
        },
    }


_OTHER_TOOLS = [
    {
        "name": "view_image",
        "description": "Attach an image file from your working directory (e.g. images/0003_x.png).",
        "input_schema": {
            "type": "object",
            "properties": {"path": {"type": "string", "description": "Relative path."}},
            "required": ["path"],
        },
    },
    {
        "name": "finish_episode",
        "description": (
            "End the episode now, scored as it stands. Use it when the task is done but the "
            "episode did not end by itself, or when the task has become impossible."),
        "input_schema": {
            "type": "object",
            "properties": {"reason": {"type": "string", "description": "Why you end it now."}},
            "required": ["reason"],
        },
    },
    {
        "name": "submit_report",
        "description": (
            "Your required last action once the episode is over: your account of the episode. "
            "diagnosis is required unless the episode ended by itself before its step budget."),
        "input_schema": {
            "type": "object",
            "properties": {
                "outcome_belief": {"type": "string", "enum": ["success", "failure", "unsure"]},
                "interpretation": {"type": "string",
                                   "description": "How you read the instruction: objects, goal."},
                "plan": {"type": "string", "description": "Your high-level plan and its changes."},
                "steps": {
                    "type": "array",
                    "description": "What you did, in order.",
                    "items": {
                        "type": "object",
                        "properties": {"what": {"type": "string"}, "why": {"type": "string"},
                                       "result": {"type": "string"}},
                        "required": ["what", "why", "result"],
                    },
                },
                "key_images": {
                    "type": "array",
                    "description": "Images (paths under images/) that show how it went.",
                    "items": {
                        "type": "object",
                        "properties": {"path": {"type": "string"},
                                       "caption": {"type": "string"}},
                        "required": ["path", "caption"],
                    },
                },
                "diagnosis": {"type": "string",
                              "description": "Why it failed or may have failed; empty only for "
                                             "a clear success."},
                "lessons": {"type": "string"},
            },
            "required": ["outcome_belief", "interpretation", "plan", "steps", "key_images",
                         "diagnosis", "lessons"],
        },
    },
]


def tools_for(preloaded: tuple[str, ...] = PRELOADED, judged_by_self: bool = False,
              vision: str = "image") -> list[dict]:
    """The four tools, the python one describing `preloaded`, in surface.SELF_JUDGED's texts if
    judged_by_self; a vision other than image gets three, without view_image."""
    texts = globals()[_VISION_TEXT[vision][0]]
    if judged_by_self:
        texts = {**texts, "rh": SELF_JUDGED["rh"]}
    others = [t for t in copy.deepcopy(_OTHER_TOOLS)
              if vision == "image" or t["name"] != "view_image"]
    tools = [python_tool(tuple(preloaded), vision, texts), *others]
    if judged_by_self:
        for t in tools:
            t["description"] = SELF_JUDGED.get(t["name"], t["description"])
    return tools


def tools_sha256(tools: list[dict]) -> str:
    raw = json.dumps(tools, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(raw.encode()).hexdigest()


def truncate(text: str | None, n: int = OUT_CHARS) -> str:
    text = text or ""
    if len(text) <= n:
        return text
    head, tail = n * 2 // 5, n * 3 // 5
    return f"{text[:head]}\n...[{len(text) - head - tail} characters cut]...\n{text[-tail:]}"


def read_image(root: Path, rel: str, kernel_uid: int | None = None) -> bytes:
    """A model-written image under root through amp.fs.safe_files: no link, FIFO or hard link
    (unless host-made, per kernel_uid), at most IMAGE_MAX_BYTES."""
    data = safe_read_bytes(Path(root) / rel, root, IMAGE_MAX_BYTES + 1, kernel_uid=kernel_uid)
    if len(data) > IMAGE_MAX_BYTES:
        raise ValueError(f"larger than {IMAGE_MAX_BYTES} bytes")
    return data


def encode_image(data: bytes) -> tuple[dict, int, int]:
    """An API image block and its size, downscaled to <= 1280 px; PNG or JPEG by size (IMAGE_POLICY)."""
    from PIL import Image

    img = Image.open(io.BytesIO(data))
    img.load()
    if img.mode not in ("RGB", "L"):
        img = img.convert("RGB")
    w, h = img.size
    scale = IMAGE_LONG_SIDE / max(w, h)
    if scale < 1:
        img = img.resize((max(1, round(w * scale)), max(1, round(h * scale))),
                         Image.Resampling.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, "PNG", optimize=False)
    media = "image/png"
    if buf.tell() > PNG_MAX_BYTES:
        buf = io.BytesIO()
        img.convert("RGB").save(buf, "JPEG", quality=85)
        media = "image/jpeg"
    elif buf.tell() >= PNG_KEEP_BYTES:
        jpg = io.BytesIO()
        img.convert("RGB").save(jpg, "JPEG", quality=95)
        if jpg.tell() < buf.tell():
            buf, media = jpg, "image/jpeg"
    data = base64.b64encode(buf.getvalue()).decode()
    block = {"type": "image", "source": {"type": "base64", "media_type": media, "data": data}}
    return block, img.size[0], img.size[1]


def image_blocks(root: Path, rel: str, caption: str | None,
                 kernel_uid: int | None = None) -> list[dict]:
    """[label, image] for a file under root, or [label saying why it could not be attached]."""
    label = f"image {rel}" + (f": {caption}" if caption else "")
    try:
        block, _, _ = encode_image(read_image(root, rel, kernel_uid))
    except Exception as e:  # noqa: BLE001
        return [{"type": "text", "text": f"{label} (could not be attached: {e})"}]
    return [{"type": "text", "text": label}, block]


def python_result(res, timeout_s: float, root: Path, kernel_uid: int | None = None,
                  interrupted: str | None = None, *,
                  attach: bool = True) -> tuple[list[dict], bool]:
    """A kernel result as (tool_result content, is_error); interrupted replaces the timeout line;
    attach False leaves the shown images out (vision "vdm_strict")."""
    images = [im for im in res.images or [] if im.get("path")] if attach else []
    parts = []
    if res.stdout:
        parts.append("stdout:\n" + truncate(res.stdout))
    if res.stderr:
        parts.append("stderr:\n" + truncate(res.stderr))
    if res.error:
        err = "error:\n" + truncate(res.error)
        parts.insert(0 if images else len(parts), err)
    if res.timed_out:
        parts.append(interrupted or f"[interrupted: timed out after {timeout_s:.0f} s]")
    if not parts:
        parts.append("(no output)")
    blocks = [{"type": "text", "text": "\n".join(parts)}]
    for im in images[:MAX_SHOWN]:
        blocks += image_blocks(root, str(im.get("path")), im.get("caption"), kernel_uid)
    return blocks, bool(res.error)


def validate_report(r: dict, ended_by_itself: bool, judged_by_self: bool = False) -> str | None:
    """Why a submit_report input is refused, or None."""
    missing = [k for k in REPORT_FIELDS if k not in r]
    if missing:
        return f"missing fields: {missing}"
    if r["outcome_belief"] not in ("success", "failure", "unsure"):
        return "outcome_belief must be success, failure or unsure"
    if not isinstance(r["steps"], list) or not all(
            isinstance(s, dict) and {"what", "why", "result"} <= set(s) for s in r["steps"]):
        return "steps must be a list of {what, why, result}"
    if not isinstance(r["key_images"], list) or not all(
            isinstance(k, dict) and "path" in k for k in r["key_images"]):
        return "key_images must be a list of {path, caption}"
    no_diagnosis = not str(r.get("diagnosis") or "").strip()
    if judged_by_self:
        return SELF_JUDGED["diagnosis_required"] if no_diagnosis and \
            r["outcome_belief"] != "success" else None
    if not ended_by_itself and no_diagnosis:
        return ("diagnosis is required: the episode did not end by itself before its step "
                "budget ran out. Explain why it failed or may have failed.")
    return None


def preloaded_phrase(preloaded: tuple[str, ...]) -> str:
    """'rh, ep, np, Image and show' for the kernel-restart note."""
    names = list(preloaded)
    if len(names) <= 1:
        return "".join(names)
    return ", ".join(names[:-1]) + " and " + names[-1]
