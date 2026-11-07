#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Self-test probe, run inside the sandbox as python -c <this source>, its JSON args on stdin.

Each check is True when isolation held (or, for the works_* controls, when the work still works).
A check that does not apply here raises Skip: it is reported under "skipped", never as passing.
Checks that need the host's view (files written, connections accepted, gateway hits, paths read)
are completed by selftest.py. Stdlib only; numpy and PIL required, scipy/cv2/matplotlib optional.
"""

import http.client
import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

# A 1x1 PNG, so the plain image is a real one without PIL in the probe.
PNG_1PX = bytes.fromhex("89504e470d0a1a0a0000000d4948445200000001000000010802000000907753de0000"
                        "000c49444154789c63e0129103000068003d5408a3f70000000049454e44ae426082")
SCIENCE = ("numpy", "scipy", "cv2", "matplotlib", "PIL")
REQUIRED_SCIENCE = ("numpy", "PIL")  # what the kernel's child imports
CLONE_NEWUSER = 0x10000000

WALK_FOUND, WALK_SEEN, WALK_S = 50, 20000, 30


class Skip(Exception):
    """The check does not apply on this host; the reason is reported."""


def fails(fn) -> bool:
    try:
        fn()
    except Exception:  # noqa: BLE001 - any failure means the sandbox refused
        return True
    return False


def read(path):
    with open(path, "rb") as f:
        return f.read(1 << 20)


def write(path, mode="w"):
    with open(path, mode) as f:
        f.write("selftest")


def under(p, roots):
    return any(p == r or p.startswith(r.rstrip("/") + "/") for r in roots)


def connect(addr, timeout=2):
    socket.create_connection(tuple(addr), timeout=timeout).close()


def rlimit(name):
    import resource

    return resource.getrlimit(getattr(resource, "RLIMIT_" + name.upper()))


class Probe:
    def __init__(self, args: dict):
        self.a = args
        self.res: dict = {}
        self.det: dict = {}
        self.skipped: dict = {}
        self.ep = json.loads(read(args["episode_file"]))
        self.tmp = os.path.join(args["episode_dir"], "tmp")
        self.gw = self.ep["gateway"].rstrip("/") + "/"
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.replaced = False

    def check(self, name, fn):
        try:
            self.res[name] = bool(fn())
        except Skip as e:
            self.skipped[name] = str(e)[:300]
        except Exception as e:  # noqa: BLE001 - a crashing check fails closed
            self.res[name] = False
            self.det[name] = f"{type(e).__name__}: {e}"[:300]

    def wait_for_host_replace(self, timeout=60):
        """Tell the host the sandbox is up, then wait until it has os.replace()d the masked files."""
        write(os.path.join(self.tmp, ".probe_started"))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if os.path.exists(os.path.join(self.tmp, ".host_replaced")):
                return True
            time.sleep(0.05)
        return False

    def get(self, path, method="GET"):
        req = urllib.request.Request(self.gw + path, method=method,
                                     data=b"{}" if method == "POST" else None)
        try:
            with self.opener.open(req, timeout=15) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    def raw(self, target):
        u = urllib.parse.urlsplit(self.gw)
        c = http.client.HTTPConnection(u.hostname, u.port, timeout=15)
        try:
            c.putrequest("GET", target, skip_accept_encoding=True)
            c.endheaders()
            return c.getresponse().status
        finally:
            c.close()

    def scan_for(self, marker: str) -> bool:
        m = marker.encode()
        for root, _, files in os.walk(self.a["episode_dir"]):
            for f in files:
                try:
                    if m in read(os.path.join(root, f)):
                        return False
                except OSError:
                    pass
        return marker not in json.dumps(dict(os.environ))

    def operator_routes_blocked(self):
        return (self.get("harness/status") == 403 and self.get("harness/claim", "POST") == 403
                and self.get("harness/release", "POST") == 403
                and self.raw("/" + self.ep["base"] + "../../harness/status") == 403)

    def other_tokens_blocked(self):
        return (self.get(f"s/{self.a['other_token']}/_episode") == 403
                and self.get(f"s/{self.a['real_token']}/_episode") == 403)

    def env_allowlisted(self):
        extra = sorted(set(os.environ) - set(self.a["env_allow"]))
        self.det["env_extra_names"] = json.dumps(extra)
        env = json.dumps(dict(os.environ))
        return not extra and not any(v in env for v in self.a["env_secrets"])

    def no_external(self):
        for addr in self.a["external"]:
            if not fails(lambda a=addr: connect(a, 3)):
                return False
        return fails(lambda: socket.getaddrinfo(self.a["hostname"], 443))

    def no_read_outside(self):
        a = self.a
        return (all(fails(lambda p=p: read(p)) for p in a["unreadable"])
                and all(fails(lambda d=d: os.listdir(d)) for d in a["unlistable"])
                and self.ancestors_hide_siblings())

    def ancestors_hide_siblings(self):
        # bwrap synthesizes the episode's ancestors, so listing one may work but must show no sibling.
        for anc, sib in self.a["siblings"]:
            try:
                if sib in os.listdir(anc):
                    return False
            except OSError:
                pass
        return True

    def readable_under(self, tops):
        """Every file that opens under tops, skipping what the sandbox grants (sent as granted)."""
        found, seen, t0 = [], 0, time.monotonic()
        granted = self.a["granted"]
        for top in tops:
            if under(top, granted):
                continue
            for root, dirs, files in os.walk(top):
                dirs[:] = [d for d in dirs if not under(os.path.join(root, d), granted)]
                for f in files:
                    seen += 1
                    p = os.path.join(root, f)
                    # bwrap grants the interpreter and libpython as single files, not their dirs.
                    if under(p, granted):
                        continue
                    try:
                        with open(p, "rb") as fh:
                            fh.read(1)
                        found.append(p)
                    except OSError:
                        pass
                    if len(found) > WALK_FOUND or seen > WALK_SEEN or time.monotonic() - t0 > WALK_S:
                        return found
        return found

    def no_forbidden_paths(self):
        found = [p for p in self.a["forbidden_files"] if not under(p, self.a["granted"])
                 and not fails(lambda p=p: read(p))]
        found += self.readable_under(self.a["forbidden"])
        self.det["forbidden_readable"] = json.dumps(found[:20])
        return not found

    def no_trusted_trace_write(self):
        """Every way into the host's trace dir fails; selftest also checks it is unchanged."""
        t = self.a["trace"]
        d, stream, mine = t["dir"], os.path.join(t["dir"], t["stream"]), os.path.join(self.tmp, "x")
        write(mine)
        attempts = {
            "append": lambda: write(stream, "a"),
            "create": lambda: write(os.path.join(d, "kernel.jsonl")),
            "mkdir": lambda: os.mkdir(os.path.join(d, "blobs", "x")),
            "unlink": lambda: os.unlink(stream),
            "rename_stream": lambda: os.rename(stream, os.path.join(d, "moved.jsonl")),
            "rename_dir": lambda: os.rename(d, d + "_moved"),
            "hardlink_in": lambda: os.link(mine, os.path.join(d, "linked.jsonl")),
            "symlink_in": lambda: os.symlink(mine, os.path.join(d, "sym.jsonl")),
        }
        got = {k: not fails(fn) for k, fn in attempts.items()}
        self.det["no_trusted_trace_write"] = json.dumps({k: v for k, v in got.items() if v})
        return not any(got.values())

    def no_process_view(self):
        m = self.a["canary_marker"].encode()
        if os.path.isdir("/proc"):
            for p in os.listdir("/proc"):
                if p.isdigit():
                    try:
                        if m in read(f"/proc/{p}/cmdline"):
                            return False
                    except OSError:
                        pass
        return True

    def episode_rw(self):
        p = os.path.join(self.tmp, "selftest_rw.txt")
        os.makedirs(os.path.dirname(p), exist_ok=True)
        write(p)
        return read(p) == b"selftest"

    def hidden_after_replace(self):
        if not self.replaced:
            self.det["hidden_after_replace"] = "the host never signalled its replace"
            return False
        seen = []
        for name in ("meta.json", "episode.json"):
            try:
                if self.a["replace_marker"].encode() in read(
                        os.path.join(self.a["episode_dir"], name)):
                    seen.append(name)
            except OSError:
                pass
        self.det["hidden_after_replace"] = json.dumps(seen)
        return not seen

    def limits_applied(self):
        lim = self.a["limits"]
        got = {k: list(rlimit(k)) for k in lim}
        self.det["limits_applied"] = json.dumps({"got": got, "skipped": self.a["limits_skipped"]})
        return bool(lim) and all(got[k] == [v, v] for k, v in lim.items())

    def as_enforced(self):
        lim = self.a["limits"]
        if "as" not in lim:
            raise Skip(self.a["limits_skipped"].get("as", "no RLIMIT_AS set"))
        soft = rlimit("as")[0]
        if soft != lim["as"]:
            return False  # never try a huge allocation without the cap in force
        try:
            import numpy as np

            def big():
                return np.empty(2 * soft, np.uint8)
        except ImportError:
            def big():
                return bytearray(2 * soft)
        try:
            big()
            return False
        except MemoryError:
            pass
        after = bytearray(64 << 20)
        return len(after) == 64 << 20 and not any(after[:4096])

    def nproc_enforced(self):
        """Start threads past the cap: one must fail, a fork at the cap too, and both work after."""
        lim = self.a["limits"]
        if "nproc" not in lim:
            raise Skip(self.a["limits_skipped"].get("nproc", "no RLIMIT_NPROC set"))
        if not self.a.get("nproc_enforced"):
            raise Skip("not enforced here (see nproc_note)")
        import threading

        cap = lim["nproc"]
        if rlimit("nproc")[0] != cap:
            self.det["nproc_enforced"] = "the cap is not in force"
            return False
        ev, held, err, fork_err = threading.Event(), [], None, None
        try:
            for _ in range(cap + 64):
                t = threading.Thread(target=ev.wait, daemon=True)
                t.start()
                held.append(t)
        except RuntimeError as e:
            err = str(e)
            # The relay shim's connection threads come and go, so refill any freed slot first.
            for _ in range(20):
                try:
                    while len(held) < cap + 64:
                        t = threading.Thread(target=ev.wait, daemon=True)
                        t.start()
                        held.append(t)
                except RuntimeError:
                    pass
                try:
                    subprocess.run([sys.executable, "-c", "pass"], timeout=30, check=False)
                except OSError as e2:
                    fork_err = f"{type(e2).__name__}: {e2}"
                    break
        finally:
            ev.set()
            for t in held:
                t.join()
        t = threading.Thread(target=lambda: None)
        t.start()
        t.join()
        after = subprocess.run([sys.executable, "-c", "print(3)"], capture_output=True,
                               timeout=60, check=False).stdout.strip() == b"3"
        out = {"cap": cap, "threads_started": len(held), "thread_error": err,
               "fork_error": fork_err, "serving_after": after}
        self.det["nproc_enforced"] = json.dumps(out)
        return bool(err and fork_err and len(held) < cap and after)

    def fair_cpu_count(self):
        share = self.a.get("cpu_share")
        if share is None:
            raise Skip("no CPU share: it is applied under bwrap only")
        import concurrent.futures
        import multiprocessing

        got = {"os": os.cpu_count(), "multiprocessing": multiprocessing.cpu_count(),
               "real": getattr(os, "real_cpu_count", lambda: None)()}
        if hasattr(os, "sched_getaffinity"):
            got["affinity"] = len(os.sched_getaffinity(0))
        with concurrent.futures.ThreadPoolExecutor() as ex:
            got["thread_pool_workers"] = ex._max_workers
        self.det["works_fair_cpu_count"] = json.dumps(got)
        return (got["os"] == got["multiprocessing"] == share and got.get("affinity", share) <= share
                and got["real"] is not None and got["thread_pool_workers"] == min(32, share + 4))

    def scientific_stack(self):
        import importlib.util

        missing = [m for m in SCIENCE if importlib.util.find_spec(m) is None]
        got = {"missing": missing}
        self.det["works_scientific_stack"] = json.dumps(got)
        if set(missing) & set(REQUIRED_SCIENCE):
            return False
        import numpy as np
        from PIL import Image

        a = np.arange(256 * 256, dtype=np.float64).reshape(256, 256) / 65536.0
        got["numpy_matmul"] = bool(np.allclose(a @ np.eye(256), a))
        Image.new("RGB", (8, 8), "red").save(os.path.join(self.tmp, "selftest_pil.png"))
        got["pil_save"] = os.path.getsize(os.path.join(self.tmp, "selftest_pil.png")) > 0
        if "scipy" not in missing:
            import scipy.spatial

            tree = scipy.spatial.cKDTree(np.random.default_rng(0).random((2000, 3)))
            d, _ = tree.query(np.zeros((4, 3)), k=2)
            got["scipy_ckdtree"] = d.shape == (4, 2)
        if "cv2" not in missing:
            import cv2

            ok, buf = cv2.imencode(".png", np.zeros((16, 16, 3), np.uint8))
            got["cv2_imencode"] = bool(ok) and len(buf) > 0
            got["cv2_threads"] = cv2.getNumThreads()
        if "matplotlib" not in missing:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig = plt.figure()
            plt.plot([0, 1], [1, 0])
            fig.savefig(os.path.join(self.tmp, "selftest_fig.png"))
            got["matplotlib_agg"] = os.path.getsize(os.path.join(self.tmp,
                                                                 "selftest_fig.png")) > 0
        self.det["works_scientific_stack"] = json.dumps(got)
        return all(v for k, v in got.items() if k not in ("cv2_threads", "missing"))

    def no_nested_userns(self):
        if not self.a.get("userns_disabled"):
            raise Skip("bwrap --disable-userns is not in use")
        code = ("import ctypes, sys; sys.exit(0 if ctypes.CDLL(None, use_errno=True)"
                f".unshare({CLONE_NEWUSER}) == 0 else 3)")
        r = subprocess.run([sys.executable, "-c", code], capture_output=True, timeout=60,
                           check=False)
        self.det["no_nested_userns"] = f"exit {r.returncode}"
        return r.returncode == 3

    def uid_not_root(self):
        """Not root inside, and not a root host that skipped the uid switch ($AMP_SANDBOX_UID)."""
        return os.geteuid() != 0 and not self.a.get("uid_opt_out")

    def host_files_hidden(self):
        """The agent's files at the episode root cannot be read; their content never shows."""
        m = self.a["host_marker"].encode()
        for p in self.a["host_files"]:
            try:
                if m in read(p):
                    return False
            except OSError:
                pass
        return True

    def host_file_writes(self):
        """Try to change the host's root files and kernel/; selftest checks none changed."""
        got = {}
        for p in [*self.a["host_files"], *self.a["host_new"]]:
            got[p] = not fails(lambda p=p: write(p))
        self.det["host_file_writes"] = json.dumps({k: v for k, v in got.items() if v})
        return True

    def plant_links(self):
        """Plant links where the host later reads (out files, images); selftest checks it refuses."""
        ep, pl = self.a["episode_dir"], self.a["plant"]
        for d in ("tmp/kernel", "images"):
            os.makedirs(os.path.join(ep, d), exist_ok=True)
        for rel, body in ((pl["plain_out"], b"plain-ok"), (pl["plain_image"], PNG_1PX),
                          ("tmp/.hl_out", b"hard-out"), ("images/.hl_img", PNG_1PX)):
            with open(os.path.join(ep, rel), "wb") as f:
                f.write(body)
        for rel in (pl["sym_out"], pl["sym_image"]):
            os.symlink(self.a["link_target"], os.path.join(ep, rel))
        os.symlink(self.a["link_dir"], os.path.join(ep, pl["sym_dir"]))
        os.link(os.path.join(ep, "tmp/.hl_out"), os.path.join(ep, pl["hard_out"]))
        os.link(os.path.join(ep, "images/.hl_img"), os.path.join(ep, pl["hard_image"]))
        return True

    def run(self) -> dict:
        a, check = self.a, self.check
        self.replaced = self.wait_for_host_replace()
        check("works_episode_rw", self.episode_rw)
        check("works_sdk_import", lambda: __import__(a["sdk_module"]).MARK == "sdk-ok")
        check("works_subprocess_python", lambda: subprocess.run(
            [sys.executable, "-c", "print(5)"], capture_output=True, timeout=60, check=False
        ).stdout.strip() == b"5")
        check("works_relay_to_gateway", lambda: self.get(self.ep["base"] + "_episode") == 200)
        check("relay_blocks_operator_routes", self.operator_routes_blocked)
        check("relay_blocks_other_tokens", self.other_tokens_blocked)
        check("real_token_hidden", lambda: self.scan_for(a["real_token"]))
        check("task_name_hidden", lambda: self.scan_for(a["task_marker"]))
        check("env_allowlisted", self.env_allowlisted)
        check("no_direct_gateway", lambda: fails(lambda: connect(a["gateway_addr"])))
        check("no_other_local_port", lambda: fails(lambda: connect(a["decoy_addr"])))
        check("no_host_ports", lambda: all(fails(lambda p=p: connect(("127.0.0.1", p), 1))
                                           for p in a["host_ports"]))
        check("no_external_network", self.no_external)
        check("no_read_outside", self.no_read_outside)
        check("no_other_episodes", lambda: all(fails(lambda p=p: read(p))
                                               for p in a["other_episode"]))
        check("no_forbidden_paths", self.no_forbidden_paths)
        check("relay_files_readonly", lambda: all(fails(lambda p=p: write(p, "a"))
                                                  for p in a["readonly"]))
        check("no_trusted_trace_write", self.no_trusted_trace_write)
        self.det["write_attempts"] = json.dumps(
            {p: not fails(lambda p=p: write(p)) for p in a["unwritable"]})
        check("no_process_view", self.no_process_view)
        check("hidden_after_replace", self.hidden_after_replace)
        check("limits_applied", self.limits_applied)
        check("limit_enforced", self.as_enforced)
        check("nproc_enforced", self.nproc_enforced)
        check("kernel_uid_not_root", self.uid_not_root)
        check("no_nested_userns", self.no_nested_userns)
        check("host_files_hidden", self.host_files_hidden)
        check("no_host_file_writes", self.host_file_writes)
        check("works_scientific_stack", self.scientific_stack)
        check("works_fair_cpu_count", self.fair_cpu_count)
        check("no_signal_outside", lambda: fails(lambda: os.kill(a["canary_pid"], 0)))
        check("no_exec_shell", lambda: fails(lambda: subprocess.run(
            ["/bin/sh", "-c", "true"], check=True, timeout=30)))
        check("links_planted", self.plant_links)
        self.det["euid"] = str(os.geteuid())
        return {"checks": self.res, "details": self.det, "skipped": self.skipped}


if __name__ == "__main__":
    print(json.dumps(Probe(json.loads(sys.stdin.read())).run()))
