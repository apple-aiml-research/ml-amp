#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Redaction applied to every event and text blob before it reaches disk."""

from __future__ import annotations

import re
import threading

from amp.contracts.trace import REDACT_KEYS

REDACTED = "[redacted]"
MIN_SECRET_LEN = 4
_KEYS = frozenset(k.lower() for k in REDACT_KEYS)
_SENSITIVE_SUFFIXES = ("_token", "-token", "_secret", "-secret", "_password", "_api_key",
                       "-api-key", "_apikey")

_JWT = re.compile(r"eyJ[A-Za-z0-9_-]{4,}\.[A-Za-z0-9_-]{4,}\.[A-Za-z0-9_-]*")
_PEM = re.compile(r"-----BEGIN [A-Z0-9 ]+-----.*?(?:-----END [A-Z0-9 ]+-----|\Z)", re.DOTALL)
_BEARER = re.compile(r"(?i)\bBearer\s+[A-Za-z0-9._~+/=-]+")
_LONG_B64 = re.compile(r"[A-Za-z0-9+/_.-]{16,}={0,2}")


def redact_key(key: str) -> bool:
    """Whether every string under this key is redacted (REDACT_KEYS, case-insensitive)."""
    return key.lower() in _KEYS


def suffix_key(key: str) -> bool:
    """Whether this key's long base64 runs are redacted ("access_token", "client_secret")."""
    return key.lower().endswith(_SENSITIVE_SUFFIXES)


class Redactor:
    """Exact registered secrets plus credential-shaped patterns become "[redacted]"."""

    def __init__(self, secrets=()):
        self._lock = threading.Lock()
        self._secrets: tuple[str, ...] = ()
        for s in secrets:
            self.register(s)

    def register(self, value: str | bytes | None) -> None:
        """Redact this exact value everywhere from now on; short or empty values are ignored."""
        if isinstance(value, bytes):
            value = value.decode("utf-8", "replace")
        value = (value or "").strip()
        if len(value) < MIN_SECRET_LEN:
            return
        with self._lock:
            self._secrets = tuple(sorted(set(self._secrets) | {value}, key=len, reverse=True))

    def exact(self, s: str) -> str:
        """s with only the registered secrets replaced; credential look-alikes stay verbatim."""
        for secret in self._secrets:
            if secret in s:
                s = s.replace(secret, REDACTED)
        return s

    def text(self, s: str) -> str:
        """s with secrets and credential patterns replaced."""
        s = self.exact(s)
        if "eyJ" in s:
            s = _JWT.sub(REDACTED, s)
        if "-----BEGIN" in s:
            s = _PEM.sub(REDACTED, s)
        if "earer" in s or "EARER" in s:
            s = _BEARER.sub("Bearer " + REDACTED, s)
        return s

    def value(self, v, *, verbatim: bool = False, _under: str = ""):
        """v redacted recursively and made JSON-safe."""
        if isinstance(v, bool) or v is None:
            return v
        if isinstance(v, (int, float)):
            return REDACTED if _under == "all" else v
        if isinstance(v, str):
            if _under == "all":
                return REDACTED if v else v
            if verbatim:
                return self.exact(v)
            v = self.text(v)
            return _LONG_B64.sub(REDACTED, v) if _under == "b64" else v
        if isinstance(v, dict):
            out: dict[str, object] = {}
            for k, x in v.items():
                k = k if isinstance(k, str) else str(k)
                under = _under or ("all" if redact_key(k) else
                                   "b64" if suffix_key(k) and not verbatim else "")
                if (self.exact(k) if verbatim else self.text(k)) != k:
                    k = f"{REDACTED[:-1]}:{len(out)}]"
                while k in out:
                    k += "#"
                out[k] = self.value(x, verbatim=verbatim, _under=under)
            return out
        if isinstance(v, (list, tuple)):
            return [self.value(x, verbatim=verbatim, _under=_under) for x in v]
        if isinstance(v, (bytes, bytearray)):
            v = bytes(v).decode("utf-8", "replace")
        else:
            v = str(v)
        return self.value(v, verbatim=verbatim, _under=_under)

    def secrets_in(self, s: str) -> bool:
        """Whether any registered secret occurs in s (a last check on serialized lines)."""
        return any(secret in s for secret in self._secrets)


DEFAULT = Redactor()


def register_secret(value: str | bytes | None) -> None:
    """Register a secret with the process-wide redactor every FileTracer uses by default."""
    DEFAULT.register(value)
