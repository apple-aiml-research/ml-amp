#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The launcher <-> deploy.py channel: one local AF_UNIX peer, pickled messages, always bounded."""

from __future__ import annotations

import hmac
import os
import secrets
import shutil
import socket
import tempfile
import threading
from multiprocessing.connection import Connection
from pathlib import Path

ADDRESS_ENV = "AMP_ROBODOJO_BRIDGE"
AUTHKEY_ENV = "AMP_ROBODOJO_BRIDGE_KEY"
SOCKET_NAME = "b.sock"
MAX_ADDRESS = 100
ACCEPT_TIMEOUT_S = 850.0
RECV_TIMEOUT_S = 300.0
CONNECT_TIMEOUT_S = 60.0
HANDSHAKE_TIMEOUT_S = 30.0
KEEPALIVE_TYPE = "keepalive"
KEEPALIVE_INTERVAL_S = 60.0
SILENCE_TIMEOUT_S = 300.0
MESSAGES = ("obs", "end", "act", "hold", "stop", KEEPALIVE_TYPE)


class BridgeError(RuntimeError):
    """The bridge could not be used."""


class BridgeTimeout(BridgeError):
    """The peer said nothing within the timeout."""


class BridgeClosed(BridgeError):
    """The peer went away (its process exited)."""


def new_authkey() -> str:
    return secrets.token_hex(16)


def keepalive_message() -> dict:
    """What the launcher sends every KEEPALIVE_INTERVAL_S while the agent plans."""
    return {"type": KEEPALIVE_TYPE}


def socket_dir(prefix: str = "amp-rdj-") -> str:
    """A private directory for one bridge socket, short enough for an AF_UNIX address."""
    tried = []
    for parent in (None, "/tmp"):
        path = tempfile.mkdtemp(prefix=prefix, dir=parent)
        if len(os.path.join(path, SOCKET_NAME)) <= MAX_ADDRESS:
            return path
        tried.append(path)
        shutil.rmtree(path, ignore_errors=True)
    raise BridgeError(f"no directory short enough for an AF_UNIX bridge socket: tried {tried}")


def _wrap(sock: socket.socket) -> Connection:
    """A blocking Connection over `sock` (Connection reads the raw fd, so clear any timeout)."""
    sock.settimeout(None)
    return Connection(sock.detach())


def _mac(authkey: bytes, nonce: bytes) -> str:
    return hmac.new(authkey, nonce, "sha256").hexdigest()


def _first(conn: Connection, timeout_s: float, what: str):
    """One message, or BridgeTimeout/BridgeClosed: never blocks forever on a silent peer."""
    try:
        ready = conn.poll(timeout_s)
    except OSError as exc:
        raise BridgeClosed(f"the bridge peer is gone: {exc}") from exc
    if not ready:
        raise BridgeTimeout(f"no {what} in {timeout_s:g} s")
    try:
        return conn.recv()
    except (EOFError, ConnectionResetError, OSError) as exc:
        raise BridgeClosed(f"the bridge peer is gone: {exc}") from exc


def _challenge(conn: Connection, authkey: bytes, timeout_s: float) -> None:
    """The launcher's side of the handshake."""
    nonce = secrets.token_bytes(32)
    conn.send({"amp_bridge": 1, "nonce": nonce})
    reply = _first(conn, timeout_s, "answer to the bridge's challenge")
    mac = reply.get("mac") if isinstance(reply, dict) else None
    if not isinstance(mac, str) or not hmac.compare_digest(mac, _mac(authkey, nonce)):
        conn.send({"ok": False})
        raise BridgeError("the bridge peer failed the authkey challenge")
    conn.send({"ok": True})


def _answer(conn: Connection, authkey: bytes, timeout_s: float) -> None:
    """deploy.py's side of the handshake."""
    message = _first(conn, timeout_s, "challenge from the AMP bridge")
    nonce = message.get("nonce") if isinstance(message, dict) else None
    if not isinstance(nonce, bytes):
        raise BridgeError(f"the AMP bridge sent {message!r}, not a challenge")
    conn.send({"mac": _mac(authkey, nonce)})
    reply = _first(conn, timeout_s, "welcome from the AMP bridge")
    if not (isinstance(reply, dict) and reply.get("ok")):
        raise BridgeError("the AMP bridge refused this process: wrong authkey")


class Bridge:
    """One end of the channel: send(dict) (atomic), recv(timeout) -> dict, close()."""

    def __init__(self, conn: Connection, *, recv_timeout_s: float = RECV_TIMEOUT_S):
        self._conn: Connection | None = conn
        self.recv_timeout_s = recv_timeout_s
        self.keepalives = 0
        self._send_lock = threading.Lock()

    def send(self, message: dict) -> None:
        conn = self._conn
        if conn is None:
            raise BridgeClosed("the bridge is closed")
        if message.get("type") not in MESSAGES:
            raise ValueError(f"bridge message type {message.get('type')!r} is not one of {MESSAGES}")
        try:
            with self._send_lock:
                conn.send(message)
        except (OSError, BrokenPipeError) as exc:
            raise BridgeClosed(f"the bridge peer is gone: {exc}") from exc

    def recv(self, timeout_s: float | None = None) -> dict:
        if self._conn is None:
            raise BridgeClosed("the bridge is closed")
        wait = self.recv_timeout_s if timeout_s is None else timeout_s
        try:
            ready = self._conn.poll(wait)
        except OSError as exc:
            raise BridgeClosed(f"the bridge peer is gone: {exc}") from exc
        if not ready:
            raise BridgeTimeout(f"no bridge message in {wait:g} s")
        try:
            message = self._conn.recv()
        except (EOFError, ConnectionResetError) as exc:
            raise BridgeClosed(f"the bridge peer is gone: {exc}") from exc
        if not isinstance(message, dict) or message.get("type") not in MESSAGES:
            raise BridgeError(f"bridge message {message!r} is not one of {MESSAGES}")
        return message

    def recv_skipping_keepalives(self, silence_timeout_s: float | None = None) -> dict:
        """The next message that is not a keepalive; a keepalive only restarts the clock."""
        wait = SILENCE_TIMEOUT_S if silence_timeout_s is None else silence_timeout_s
        while True:
            try:
                message = self.recv(wait)
            except BridgeTimeout as exc:
                raise BridgeTimeout(f"no bridge message, not even a keepalive, in {wait:g} s: "
                                    "the AMP launcher is gone") from exc
            if message.get("type") != KEEPALIVE_TYPE:
                return message
            self.keepalives += 1

    def close(self) -> None:
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except OSError:
                pass


class BridgeServer:
    """The launcher's end: binds the socket before RoboDojo starts, accepts its one peer."""

    def __init__(self, path: str | os.PathLike, authkey: str):
        self.path = str(path)
        self.authkey = authkey.encode() if isinstance(authkey, str) else bytes(authkey)
        parent = Path(self.path).parent
        parent.mkdir(parents=True, exist_ok=True)
        os.chmod(parent, 0o700)
        if os.path.exists(self.path):
            os.unlink(self.path)
        if len(self.path) > MAX_ADDRESS:
            raise BridgeError(f"{self.path} is too long for an AF_UNIX address "
                              f"({len(self.path)} > {MAX_ADDRESS}); use socket_dir()")
        self._sock: socket.socket | None = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self._sock.bind(self.path)
            os.chmod(self.path, 0o600)
            self._sock.listen(1)
        except OSError as exc:
            self.close()
            raise BridgeError(f"cannot bind the bridge at {self.path}: {exc}") from exc

    def accept(self, timeout_s: float = ACCEPT_TIMEOUT_S,
               recv_timeout_s: float = RECV_TIMEOUT_S,
               handshake_s: float = HANDSHAKE_TIMEOUT_S) -> Bridge:
        """The peer's Bridge; BridgeTimeout if deploy.py never connected."""
        if self._sock is None:
            raise BridgeClosed("the bridge server is closed")
        self._sock.settimeout(timeout_s)
        try:
            peer, _ = self._sock.accept()
        except TimeoutError as exc:
            raise BridgeTimeout(f"no bridge peer connected in {timeout_s:g} s") from exc
        conn = _wrap(peer)
        try:
            _challenge(conn, self.authkey, handshake_s)
        except BridgeError:
            conn.close()
            raise
        except Exception as exc:
            conn.close()
            raise BridgeError(f"the bridge handshake failed: {exc}") from exc
        return Bridge(conn, recv_timeout_s=recv_timeout_s)

    def env(self) -> dict[str, str]:
        """What the RoboDojo process needs to find this bridge."""
        return {ADDRESS_ENV: self.path, AUTHKEY_ENV: self.authkey.decode()}

    def close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        try:
            if os.path.exists(self.path):
                os.unlink(self.path)
        except OSError:
            pass


def connect(address: str, authkey: str, *, timeout_s: float = CONNECT_TIMEOUT_S,
            recv_timeout_s: float = RECV_TIMEOUT_S,
            handshake_s: float = HANDSHAKE_TIMEOUT_S) -> Bridge:
    """deploy.py's end."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout_s)
    try:
        sock.connect(address)
    except OSError as exc:
        sock.close()
        raise BridgeError(f"cannot reach the AMP bridge at {address}: {exc}") from exc
    conn = _wrap(sock)
    try:
        _answer(conn, authkey.encode() if isinstance(authkey, str) else authkey, handshake_s)
    except BridgeError:
        conn.close()
        raise
    except Exception as exc:
        conn.close()
        raise BridgeError(f"the AMP bridge handshake failed: {exc}") from exc
    return Bridge(conn, recv_timeout_s=recv_timeout_s)


def connect_from_env(environ=None, **kw) -> Bridge:
    """connect() with the address and authkey the launcher exported."""
    env = os.environ if environ is None else environ
    address, authkey = env.get(ADDRESS_ENV), env.get(AUTHKEY_ENV)
    if not address or not authkey:
        raise BridgeError(f"{ADDRESS_ENV} and {AUTHKEY_ENV} must name the AMP bridge; this "
                          "process was not started by AMP's RoboDojo launcher")
    return connect(address, authkey, **kw)
