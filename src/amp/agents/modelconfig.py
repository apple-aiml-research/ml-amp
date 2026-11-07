#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Where each model is served: base URL, API key, headers, served name, price and cost header,
from the vendors' environment variables and an optional models file ($AMP_MODELS_FILE)."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.model import ModelSpec

MODELS_FILE_ENV = "AMP_MODELS_FILE"
WIRES = ("anthropic", "gemini", "openai-chat")
FALLBACK_PRICE = {"anthropic": {"input": 5.0, "output": 25.0, "cache_read": 0.5,
                                "cache_write": 6.25},
                  "gemini": {"input": 4.0, "output": 20.0, "cache_read": 0.2}}
FALLBACK_SOURCE = "fallback_price"
PRICE_KEYS = ("input", "output", "cache_read", "cache_write")
_VAR = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")


@dataclass(frozen=True)
class Wire:
    """A wire format's defaults and the environment variables that override them."""

    base_url: str
    base_env: str
    key_env: str
    key_header: str
    headers_env: str
    key_required: bool


WIRE = {
    "anthropic": Wire("https://api.anthropic.com", "ANTHROPIC_BASE_URL", "ANTHROPIC_API_KEY",
                      "x-api-key", "AMP_ANTHROPIC_HEADERS", True),
    "gemini": Wire("https://generativelanguage.googleapis.com/v1beta", "GEMINI_BASE_URL",
                   "GEMINI_API_KEY", "x-goog-api-key", "AMP_GEMINI_HEADERS", True),
    "openai-chat": Wire("http://127.0.0.1:30000/v1", "OPENAI_BASE_URL", "OPENAI_API_KEY",
                        "authorization", "AMP_OPENAI_HEADERS", False),
}


class ModelConfigError(ValueError):
    """The models file or a header variable is malformed; the message never holds a value."""


@dataclass(frozen=True)
class Server:
    """One place requests go (a models-file `servers` entry)."""

    name: str
    wire: str
    base_url: str | None = None
    api_key_env: str | None = None
    api_key_header: str | None = None
    headers: dict[str, str] = field(default_factory=dict)
    cost_header: str | None = None


@dataclass(frozen=True)
class Entry:
    """One model's place in the models file (a `models` entry)."""

    key: str
    server: str
    served_model: str | None = None
    has_price: bool = False
    price_per_mtok: dict | None = None
    context_tokens: int | None = None


@dataclass(frozen=True)
class ModelsFile:
    path: str
    servers: dict[str, Server]
    models: dict[str, Entry]


@dataclass(frozen=True)
class Resolved:
    """What a client needs to reach one model; describe() is its secret-free account."""

    wire: str
    server: str
    base_url: str
    served_model: str
    api_key_env: str | None
    api_key_header: str
    key_required: bool
    cost_header: str | None
    price_per_mtok: dict | None
    price_source: str
    context_tokens: int | None
    source: str
    api_key: str | None = field(default=None, repr=False)
    headers: dict[str, str] = field(default_factory=dict, repr=False)

    @property
    def estimated(self) -> bool:
        return self.price_source == FALLBACK_SOURCE

    def describe(self) -> dict:
        return {"server": self.server, "config": self.source, "base_url": self.base_url,
                "served_model": self.served_model, "api_key_env": self.api_key_env,
                "api_key_header": self.api_key_header if self.api_key else None,
                "api_key_set": bool(self.api_key), "extra_headers": sorted(self.headers),
                "cost_header": self.cost_header, "price_source": self.price_source}


def _parse(text: str, path: str):
    try:
        import yaml
    except ImportError:
        try:
            return json.loads(text)
        except ValueError as e:
            raise ModelConfigError(f"{path}: not JSON, and PyYAML is not installed") from e
    try:
        return yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise ModelConfigError(f"{path}: not valid YAML: {e}") from None


def _str_map(v, what: str) -> dict[str, str]:
    if v is None:
        return {}
    if not isinstance(v, dict) or not all(isinstance(k, str) and isinstance(x, str)
                                          for k, x in v.items()):
        raise ModelConfigError(f"{what}: must map header names to strings")
    return dict(v)


def _price(v, what: str) -> dict | None:
    if v is None:
        return None
    if not isinstance(v, dict):
        raise ModelConfigError(f"{what}: price_per_mtok must be a mapping or null")
    out: dict = {}
    for k, x in v.items():
        if k == "above":
            out[k] = _price(x, f"{what}.above")
        elif k in (*PRICE_KEYS, "threshold") and isinstance(x, int | float) \
                and not isinstance(x, bool) and x >= 0:
            out[k] = float(x)
        else:
            raise ModelConfigError(f"{what}: bad price key or value {k!r}")
    if ("above" in out) != ("threshold" in out):
        raise ModelConfigError(f"{what}: threshold and above go together")
    return out


def load(path: str | os.PathLike) -> ModelsFile:
    """Parse and check a models file (YAML, or JSON without PyYAML)."""
    p = str(path)
    try:
        text = Path(p).expanduser().read_text()
    except OSError as e:
        raise ModelConfigError(f"{MODELS_FILE_ENV}={p}: {e.strerror}") from None
    doc = _parse(text, p) or {}
    if not isinstance(doc, dict):
        raise ModelConfigError(f"{p}: top level must be a mapping")
    servers: dict[str, Server] = {}
    for name, s in (doc.get("servers") or {}).items():
        what = f"{p}: servers.{name}"
        if not isinstance(s, dict) or s.get("wire") not in WIRES:
            raise ModelConfigError(f"{what}: wire must be one of {WIRES}")
        unknown = set(s) - {"wire", "base_url", "api_key_env", "api_key_header", "headers",
                            "cost_header"}
        if unknown:
            raise ModelConfigError(f"{what}: unknown keys {sorted(unknown)}")
        w = WIRE[s["wire"]]
        servers[name] = Server(
            name=name, wire=s["wire"], base_url=s.get("base_url"),
            api_key_env=s.get("api_key_env", w.key_env),
            api_key_header=s.get("api_key_header"), headers=_str_map(s.get("headers"), what),
            cost_header=s.get("cost_header"))
    models: dict[str, Entry] = {}
    for key, m in (doc.get("models") or {}).items():
        what = f"{p}: models.{key}"
        if not isinstance(m, dict) or m.get("server") not in servers:
            raise ModelConfigError(f"{what}: server must name one of {sorted(servers)}")
        unknown = set(m) - {"server", "served_model", "price_per_mtok", "context_tokens"}
        if unknown:
            raise ModelConfigError(f"{what}: unknown keys {sorted(unknown)}")
        ctx = m.get("context_tokens")
        if ctx is not None and (isinstance(ctx, bool) or not isinstance(ctx, int) or ctx <= 0):
            raise ModelConfigError(f"{what}: context_tokens must be a positive int")
        models[key] = Entry(key=key, server=m["server"], served_model=m.get("served_model"),
                            has_price="price_per_mtok" in m,
                            price_per_mtok=_price(m.get("price_per_mtok"), what),
                            context_tokens=ctx)
    return ModelsFile(p, servers, models)


def models_file(environ=None) -> ModelsFile | None:
    """The file $AMP_MODELS_FILE names, None when it is unset."""
    env = os.environ if environ is None else environ
    path = env.get(MODELS_FILE_ENV)
    return load(path) if path else None


def key_envs(environ=None) -> tuple[str, ...]:
    """Every API-key variable a model may read: the wires' and the models file's."""
    names = {w.key_env for w in WIRE.values()}
    try:
        f = models_file(environ)
    except ModelConfigError:
        f = None
    if f is not None:
        names |= {s.api_key_env for s in f.servers.values() if s.api_key_env}
    return tuple(sorted(names))


def _expand(value: str, env, what: str) -> str:
    def sub(m):
        if m.group(1) not in env:
            raise ModelConfigError(f"{what}: ${{{m.group(1)}}} is not set")
        return env[m.group(1)]

    return _VAR.sub(sub, value)


def env_headers(wire: str, environ=None) -> dict[str, str]:
    """The JSON object in AMP_<WIRE>_HEADERS, {} when unset."""
    env = os.environ if environ is None else environ
    name = WIRE[wire].headers_env
    raw = env.get(name)
    if not raw:
        return {}
    try:
        v = json.loads(raw)
    except ValueError:
        raise ModelConfigError(f"{name}: not a JSON object") from None
    return _str_map(v, name)


def _header_safe(value: str, what: str) -> str:
    """value, unless it holds a control character, which no HTTP header can carry."""
    if any(ord(c) < 32 or ord(c) == 127 for c in value):
        raise ModelConfigError(f"{what} holds a control character such as a newline; "
                               "set it without one")
    return value


def resolve(spec: ModelSpec, environ=None, endpoint: str = "") -> Resolved:
    """Where spec is served."""
    env = os.environ if environ is None else environ
    wire = spec.provider
    if wire not in WIRE:
        raise ModelConfigError(f"{spec.key}: provider {wire!r} is served by no wire")
    w = WIRE[wire]
    f = models_file(env)
    entry = f.models.get(spec.key) if f else None
    headers = env_headers(wire, env)
    if entry is None:
        server = Server(name=wire, wire=wire, api_key_env=w.key_env)
        base, source = env.get(w.base_env) or w.base_url, "env"
    else:
        server = f.servers[entry.server]
        if server.wire != wire:
            raise ModelConfigError(f"{f.path}: {spec.key} is {wire} but server {server.name} "
                                   f"is {server.wire}")
        what = f"{f.path}: servers.{server.name}"
        headers.update({k: _expand(v, env, what) for k, v in server.headers.items()})
        base = _expand(server.base_url, env, what) if server.base_url else (
            env.get(w.base_env) or w.base_url)
        source = "file"
    if endpoint:
        base = endpoint
    if entry is not None and entry.has_price:
        price = entry.price_per_mtok
        price_source = "file" if price is not None else "self_hosted"
    elif spec.price_per_mtok is not None:
        price, price_source = dict(spec.price_per_mtok), "registry"
    elif wire in FALLBACK_PRICE:
        price, price_source = dict(FALLBACK_PRICE[wire]), FALLBACK_SOURCE
    else:
        price, price_source = None, "self_hosted"
    key_env = server.api_key_env
    raw_key = env.get(key_env) if key_env else None
    api_key = _header_safe(raw_key, key_env) if raw_key else None
    headers = {k: _header_safe(v, f"the {k} header") for k, v in headers.items()}
    return Resolved(
        wire=wire, server=server.name, base_url=base.rstrip("/"),
        served_model=(entry.served_model if entry and entry.served_model else spec.model_id),
        api_key_env=key_env, api_key_header=server.api_key_header or w.key_header,
        key_required=w.key_required and key_env is not None,
        cost_header=server.cost_header.lower() if server.cost_header else None,
        price_per_mtok=price, price_source=price_source,
        context_tokens=(entry.context_tokens if entry and entry.context_tokens
                        else spec.context_tokens),
        source=source, api_key=api_key, headers=headers)
