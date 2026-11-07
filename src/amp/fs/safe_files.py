#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Host-side reads of untrusted-writable files that never follow a link: every component is opened
with O_NOFOLLOW relative to its parent's fd, and only single-link regular files are read."""

from __future__ import annotations

import errno
import fcntl
import os
import shutil
import stat
from collections.abc import Iterator

REFUSED = "refused: link"
_DIR = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
_FILE = os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC
_LINK_ERRNOS = (errno.ELOOP, errno.ENOTDIR, errno.EMLINK)


class LinkRefused(OSError):
    """A path the host will not read: a symlink, a hard link, outside root, or not a file."""

    def __init__(self, path, why: str):
        super().__init__(errno.ELOOP, f"{REFUSED} ({why})", str(path))
        self.why = why

    def __str__(self) -> str:
        return f"{REFUSED} ({self.why}): {self.filename}"


def trusted_hardlink(st, kernel_uid: int | None = None) -> bool:
    """A root-owned multiply-linked file counts as host-made only when the host is root, sandboxed
    code runs as another non-root uid and Linux fs.protected_hardlinks=1; otherwise pass
    kernel_uid=None."""
    return kernel_uid not in (None, 0) and os.geteuid() == 0 and st.st_uid == 0


def rel_parts(path, root) -> list[str]:
    """path's components below root, lexically; LinkRefused unless strictly inside root."""
    p = os.path.abspath(os.fspath(path))
    for base in (os.path.abspath(root), os.path.realpath(root)):
        rel = os.path.relpath(p, base)
        if rel != ".." and not rel.startswith("../"):
            break
    if rel in (".", "") or rel == ".." or rel.startswith("../") or os.path.isabs(rel):
        raise LinkRefused(path, "outside the root directory")
    return rel.split(os.sep)


def _is_link(name: str, dir_fd: int) -> bool:
    try:
        return stat.S_ISLNK(os.stat(name, dir_fd=dir_fd, follow_symlinks=False).st_mode)
    except OSError:
        return False


def _open_at(name: str, flags: int, dir_fd: int, path, why: str) -> int:
    try:
        return os.open(name, flags, dir_fd=dir_fd)
    except OSError as e:
        if e.errno in _LINK_ERRNOS and _is_link(name, dir_fd):
            raise LinkRefused(path, why) from None
        raise


def safe_fd(path, root, *, kernel_uid: int | None = None) -> int:
    """A read-only fd for path under root, or LinkRefused; never follows any link below root."""
    parts = rel_parts(path, root)
    fd = os.open(os.path.realpath(root), _DIR & ~os.O_NOFOLLOW)
    try:
        for name in parts[:-1]:
            nfd = _open_at(name, _DIR, fd, path, "symlinked directory")
            os.close(fd)
            fd = nfd
        ffd = _open_at(parts[-1], _FILE, fd, path, "symlink")
    finally:
        os.close(fd)
    try:
        st = os.fstat(ffd)
        if not stat.S_ISREG(st.st_mode):
            raise LinkRefused(path, "not a regular file")
        if st.st_nlink > 1 and not trusted_hardlink(st, kernel_uid):
            raise LinkRefused(path, f"hard link, {st.st_nlink} names")
        fcntl.fcntl(ffd, fcntl.F_SETFL, fcntl.fcntl(ffd, fcntl.F_GETFL) & ~os.O_NONBLOCK)
    except BaseException:
        os.close(ffd)
        raise
    return ffd


def safe_open(path, root, mode: str = "rb", *, kernel_uid: int | None = None, **kw):
    """open(path, mode) for reading only, through safe_fd's checks."""
    if any(c in mode for c in "wax+"):
        raise ValueError(f"safe_open is read-only, not {mode!r}")
    return os.fdopen(safe_fd(path, root, kernel_uid=kernel_uid), mode, **kw)


def safe_read_bytes(path, root, limit: int | None = None, *,
                    kernel_uid: int | None = None) -> bytes:
    """The file's bytes (at most limit), or LinkRefused."""
    with safe_open(path, root, kernel_uid=kernel_uid) as f:
        return f.read() if limit is None else f.read(limit)


def safe_read_text(path, root, errors: str = "replace", limit: int | None = None, *,
                   kernel_uid: int | None = None) -> str:
    """The file's text (utf-8), or LinkRefused."""
    return safe_read_bytes(path, root, limit, kernel_uid=kernel_uid).decode("utf-8", errors)


def is_safe_file(path, root, *, kernel_uid: int | None = None) -> bool:
    """Whether safe_fd would open path."""
    try:
        os.close(safe_fd(path, root, kernel_uid=kernel_uid))
    except (OSError, ValueError):
        return False
    return True


def safe_copy(path, root, dst, *, kernel_uid: int | None = None) -> None:
    """Copy path's bytes (read through safe_fd) to the host-chosen dst."""
    with safe_open(path, root, kernel_uid=kernel_uid) as src, open(dst, "wb") as out:
        shutil.copyfileobj(src, out, 1 << 20)


def _open_sub(root_fd: int, sub: str) -> int:
    """sub's dir fd, walking from root_fd with O_NOFOLLOW at every component."""
    fd = os.dup(root_fd)
    for name in [p for p in sub.split("/") if p]:
        try:
            nfd = os.open(name, _DIR, dir_fd=fd)
        except OSError as e:
            link = e.errno in _LINK_ERRNOS and _is_link(name, fd)
            os.close(fd)
            if link:
                raise LinkRefused(sub, "symlinked directory") from None
            raise
        os.close(fd)
        fd = nfd
    return fd


def _scan(dfd: int, rel: str, kernel_uid: int | None, todo: list[str]
          ) -> Iterator[tuple[str, str | None]]:
    with os.scandir(dfd) as it:
        names = sorted(e.name for e in it)
    for name in names:
        r = f"{rel}/{name}" if rel else name
        try:
            st = os.stat(name, dir_fd=dfd, follow_symlinks=False)
        except OSError as e:
            yield r, f"error: {e.strerror or e}"
            continue
        if stat.S_ISLNK(st.st_mode):
            yield r, "symlink"
        elif stat.S_ISDIR(st.st_mode):
            todo.append(r)
        elif not stat.S_ISREG(st.st_mode):
            yield r, "not a regular file"
        elif st.st_nlink > 1 and not trusted_hardlink(st, kernel_uid):
            yield r, "hard link"
        else:
            yield r, None


def walk(root, sub: str = "", *, kernel_uid: int | None = None
         ) -> Iterator[tuple[str, str | None]]:
    """(rel posix path, None) per readable file under root/sub, (rel, why) per refused entry."""
    sub = sub.strip("/")
    root_fd = os.open(os.path.realpath(root), _DIR & ~os.O_NOFOLLOW)
    try:
        todo = [sub]
        while todo:
            rel = todo.pop()
            try:
                dfd = _open_sub(root_fd, rel)
            except LinkRefused:
                if rel != sub:
                    yield rel, "symlink"
                continue
            except FileNotFoundError:
                continue
            except OSError as e:
                yield rel, f"error: {e.strerror or e}"
                continue
            try:
                yield from _scan(dfd, rel, kernel_uid, todo)
            except OSError as e:
                yield rel, f"error: {e.strerror or e}"
            finally:
                os.close(dfd)
    finally:
        os.close(root_fd)
