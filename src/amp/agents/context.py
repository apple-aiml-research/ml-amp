#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Keeping the request inside the provider's limits: the image budget every model has, and
stage-by-stage elision for models with a known context window."""

from __future__ import annotations

import base64
import json

IMAGE_LONG_SIDE = 1280
MAX_IMAGES = 80
MAX_IMAGE_B64 = 18_000_000
PROMPT_BUDGET_FRACTION = 0.70
CHARS_PER_TOKEN = 3.0
KEEP_NEWEST = 6
LONG_TEXT = 2000
KEEP_TEXT = 600
LONG_INPUT = 1500
KEEP_INPUT = 500
OVERFLOW_SHRINK = 0.8
REASONING_ELIDED = "[earlier reasoning elided to fit the context window]"
RESULT_ELIDED = ("[tool result elided to fit the context window; view_image shows saved images "
                 "again]")
REFUSED_NOTE = ("[image not attached: the API refused it in this result; view_image shows it "
                "again]")
STAGES = ("thinking", "tool_text", "tool_input", "assistant_text", "images", "tool_results")


def image_pointer(label: str) -> dict:
    """The text that replaces an image dropped from the context; label is its caption block."""
    path = label.split(":")[0].removeprefix("image ").strip() if label else "?"
    return {"type": "text", "text": f"[{path}: no longer attached to keep the request small; "
                                    "view_image shows it again]"}


def image_size(data: str) -> tuple[int, int] | None:
    """(w, h) from a base64 PNG or JPEG header; None when unreadable."""
    for chunk in (data[:65536], data):
        try:
            raw = base64.b64decode(chunk[: len(chunk) // 4 * 4])
        except ValueError:
            return None
        if raw[:8] == b"\x89PNG\r\n\x1a\n" and len(raw) >= 24:
            return int.from_bytes(raw[16:20], "big"), int.from_bytes(raw[20:24], "big")
        if raw[:2] == b"\xff\xd8":
            i = 2
            while i + 9 <= len(raw) and raw[i] == 0xFF:
                kind = raw[i + 1]
                if 0xC0 <= kind <= 0xCF and kind not in (0xC4, 0xC8, 0xCC):
                    return (int.from_bytes(raw[i + 7:i + 9], "big"),
                            int.from_bytes(raw[i + 5:i + 7], "big"))
                i += 2 + int.from_bytes(raw[i + 2:i + 4], "big")
            if chunk is data:
                return None
            continue
        return None
    return None


def shorten(text: str, keep: int) -> str:
    return (f"{text[:keep]}\n...[{len(text) - 2 * keep} characters elided to fit the context "
            f"window]...\n{text[-keep:]}")


def shorten_input(v, changed: list):
    """v with every string longer than LONG_INPUT shortened; changed[0] set when one was."""
    if isinstance(v, str) and len(v) > LONG_INPUT:
        changed[0] = True
        return shorten(v, KEEP_INPUT)
    if isinstance(v, dict):
        return {k: shorten_input(x, changed) for k, x in v.items()}
    if isinstance(v, list):
        return [shorten_input(x, changed) for x in v]
    return v


class ContextWindow:
    """The images in the history (oldest first) and, given context_tokens, the token budget."""

    def __init__(self, context_tokens: int | None = None, image_cost=None,
                 max_images: int = MAX_IMAGES, max_image_b64: int = MAX_IMAGE_B64):
        self.context_tokens = context_tokens
        self.image_cost = image_cost
        self.max_images, self.max_image_b64 = max_images, max_image_b64
        self.budget_fraction = PROMPT_BUDGET_FRACTION
        self.token_ratio = 1.0
        self.images: list[tuple[list, dict, str, int]] = []
        self._image_tokens: dict[int, tuple[int, str, int]] = {}

    def register(self, content: list) -> None:
        for b in content:
            if b.get("type") == "image":
                self.images.append((content, b, "", len(b["source"]["data"])))
            elif b.get("type") == "tool_result" and isinstance(b.get("content"), list):
                label = ""
                for c in b["content"]:
                    if c.get("type") == "text":
                        label = c["text"]
                    elif c.get("type") == "image":
                        self.images.append((b["content"], c, label, len(c["source"]["data"])))

    def trim_images(self) -> int:
        """Replace the oldest images by pointers once a cap is near; how many were."""
        if len(self.images) <= self.max_images and \
                sum(n for *_, n in self.images) <= self.max_image_b64:
            return 0
        dropped = 0
        while self.images and (len(self.images) > self.max_images * 3 // 4
                               or sum(n for *_, n in self.images) > self.max_image_b64 * 3 // 4):
            container, block, label, _ = self.images.pop(0)
            for i, c in enumerate(container):
                if c is block:
                    container[i] = image_pointer(label)
            dropped += 1
        return dropped

    def strip_newest(self, messages: list[dict]) -> int:
        """Replace the image blocks of the newest user turn by a note; how many there were."""
        if not messages or messages[-1].get("role") != "user":
            return 0
        removed = set()

        def strip(blocks):
            for i, b in enumerate(blocks):
                if b.get("type") == "image":
                    removed.add(id(b))
                    blocks[i] = {"type": "text", "text": REFUSED_NOTE}
                elif b.get("type") == "tool_result" and isinstance(b.get("content"), list):
                    strip(b["content"])

        content = messages[-1].get("content")
        if isinstance(content, list):
            strip(content)
        self.images = [x for x in self.images if id(x[1]) not in removed]
        return len(removed)

    def image_tokens(self, block: dict) -> int:
        data = block.get("source", {}).get("data") or ""
        key = id(block)
        hit = self._image_tokens.get(key)
        if hit and hit[0] == len(data) and hit[1] == data[-64:]:
            return hit[2]
        w, h = image_size(data) or (IMAGE_LONG_SIDE, IMAGE_LONG_SIDE)
        n = int(self.image_cost(w, h)) if self.image_cost else w * h // 750
        self._image_tokens[key] = (len(data), data[-64:], n)
        return n

    def block_tokens(self, b: dict) -> float:
        kind = b.get("type")
        if kind == "image":
            return self.image_tokens(b)
        if kind == "tool_result":
            c = b.get("content")
            if isinstance(c, str):
                return len(c) / CHARS_PER_TOKEN
            return sum(self.block_tokens(x) for x in c or [])
        if kind == "tool_use":
            n = len(json.dumps(b.get("input") or {})) + len(b.get("name") or "")
            return n / CHARS_PER_TOKEN
        return len(b.get("text") or b.get("thinking") or "") / CHARS_PER_TOKEN

    def message_tokens(self, m: dict) -> float:
        c = m.get("content")
        if isinstance(c, str):
            return len(c) / CHARS_PER_TOKEN
        return sum(self.block_tokens(b) for b in c or [])

    def estimate(self, system: list[dict], tools: list[dict], messages: list[dict]) -> int:
        """The request's size in tokens: chars / 3 for text, image_cost for images."""
        fixed = sum(len(b.get("text") or "") for b in system) + len(json.dumps(tools))
        return round(fixed / CHARS_PER_TOKEN + sum(self.message_tokens(m) for m in messages))

    def calibrate(self, estimated: int, actual: int | None, overflow: bool = False) -> None:
        """Learn the server/estimate ratio from a reply's prompt tokens or an overflow's count."""
        if not (self.context_tokens and estimated and actual):
            return
        ratio = actual / estimated
        self.token_ratio = max(self.token_ratio, ratio) if overflow else max(1.0, ratio)

    def shrink(self) -> None:
        self.budget_fraction *= OVERFLOW_SHRINK

    def fit(self, system: list[dict], tools: list[dict], messages: list[dict]) -> dict | None:
        """Elide old context, stage by stage and oldest first, until the request fits
        budget_fraction of the window; the first user message and the newest KEEP_NEWEST messages
        are never touched."""
        if not self.context_tokens:
            return None
        budget = self.budget_fraction * self.context_tokens
        ratio = self.token_ratio
        before = total = self.estimate(system, tools, messages) * ratio
        if total <= budget:
            return None
        old = messages[1:max(1, len(messages) - KEEP_NEWEST)]

        def blocks(role):
            for m in old:
                if m.get("role") == role and isinstance(m.get("content"), list):
                    yield m["content"]

        def results():
            for cont in blocks("user"):
                for i, r in enumerate(cont):
                    if r.get("type") == "tool_result" and isinstance(r.get("content"), list):
                        yield cont, i, r

        def thinking():
            for cont in blocks("assistant"):
                for i, b in enumerate(cont):
                    if b.get("type") == "thinking":
                        yield cont, i, {**b, "thinking": REASONING_ELIDED}

        def tool_text():
            for _, _, r in results():
                for i, b in enumerate(r["content"]):
                    t = b.get("text") or ""
                    if b.get("type") == "text" and len(t) > LONG_TEXT:
                        yield r["content"], i, {**b, "text": shorten(t, KEEP_TEXT)}

        def tool_input():
            for cont in blocks("assistant"):
                for i, b in enumerate(cont):
                    changed = [False]
                    if b.get("type") == "tool_use":
                        inp = shorten_input(b.get("input"), changed)
                        if changed[0]:
                            yield cont, i, {**b, "input": inp}

        def assistant_text():
            for cont in blocks("assistant"):
                for i, b in enumerate(cont):
                    t = b.get("text") or ""
                    if b.get("type") == "text" and len(t) > LONG_TEXT:
                        yield cont, i, {**b, "text": shorten(t, KEEP_TEXT)}

        def images():
            for top in blocks("user"):
                for cont in [top] + [r["content"] for r in top if r.get("type") == "tool_result"
                                     and isinstance(r.get("content"), list)]:
                    label = ""
                    for i, b in enumerate(cont):
                        if b.get("type") == "text":
                            label = b.get("text") or ""
                        elif b.get("type") == "image":
                            yield cont, i, image_pointer(label)

        def whole_results():
            for cont, i, r in results():
                yield cont, i, {**r, "content": [{"type": "text", "text": RESULT_ELIDED}]}

        def image_ids(b):
            if b.get("type") == "image":
                return {id(b)}
            if isinstance(b.get("content"), list):
                return set().union(*(image_ids(x) for x in b["content"]))
            return set()

        stages = dict(zip(STAGES, (thinking, tool_text, tool_input, assistant_text, images,
                                   whole_results)))
        counts = dict.fromkeys(STAGES, 0)
        dropped: set[int] = set()
        for name in STAGES:
            if total <= budget:
                break
            for cont, i, repl in stages[name]():
                gain = (self.block_tokens(cont[i]) - self.block_tokens(repl)) * ratio
                if gain <= 0:
                    continue
                dropped |= image_ids(cont[i])
                cont[i] = repl
                total -= gain
                counts[name] += 1
                if total <= budget:
                    break
        if dropped:
            self.images = [x for x in self.images if id(x[1]) not in dropped]
            for k in dropped:
                self._image_tokens.pop(k, None)
        after = self.estimate(system, tools, messages) * ratio
        return {**counts, "tokens_before": round(before), "tokens_after": round(after),
                "budget_tokens": round(budget), "fraction": round(self.budget_fraction, 4),
                "token_ratio": round(ratio, 3), "context_tokens": self.context_tokens,
                "fits": after <= budget}
