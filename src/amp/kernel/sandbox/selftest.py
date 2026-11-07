#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The fail-closed sandbox self-test: run the real sandbox against a fake gateway and probe it.

    python -m amp.kernel.sandbox.selftest [--backend auto] [--python PY] [--out F.json]

exits 0 only when the report is ok: every check in required_checks() ran and held, and no other
check failed. A check that does not apply is listed in details["skipped"] and never counts as
passing. The eval layer runs no agent when the report is not ok.

Linux validation, as root or as a user whose bwrap may create user namespaces (docs/apparmor/bwrap),
from the checkout with the kernel interpreter as PY:

    bwrap --version; bwrap --help | grep -E -- '--size|--disable-u'
    PYTHONPATH=src $PY -m amp.kernel.sandbox.selftest --backend bwrap --python $PY --out st.json
    AMP_SANDBOX_UID=off PYTHONPATH=src $PY -m amp.kernel.sandbox.selftest --backend bwrap

The self-test must exit 0 with kernel_uid_not_root, nproc_enforced, limit_enforced and
works_fair_cpu_count among its true checks (and no_nested_userns whenever details.bwrap_flags
lists --disable-userns), details.skipped empty of those, and details.host_uid 65534. The opt-out
run must exit 1 on kernel_uid_not_root and nproc_enforced (missing). Then rerun the reproductions
(relay connections, a 4 GiB frame header, background disk writes, writes to agent.log): threads
stay at the relay's cap, the frame kills the kernel, the disk stop fails the next call, and the
host's agent.log is unchanged and unreadable inside.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import http.server
import json
import os
import secrets
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

from amp.contracts.eval import EPISODE_FILE, LIVE_PREFIX, NodeRuntime, SandboxReport
from amp.contracts.trace import TRACE_DIR
from amp.fs.safe_files import REFUSED, LinkRefused, safe_read_bytes, walk

from . import bwrap
from .core import ENV_ALLOW, HOST_FILES, KERNEL_DIR, Sandbox, SandboxError, resolve_backend
from .paths import under

PROBE_SOURCE = Path(__file__).with_name("probe.py").read_text()
SDK_MODULE = "selftest_sdk"
EXTERNAL = (("192.0.2.1", 443),)
HOSTNAME = "example.com"
REPO = Path(__file__).resolve().parents[4]
FORBIDDEN = ("/robolab", "/mnt", "/root", "/tmp", "/home")  # nosec B108
SLOTS = 24
_RT = NodeRuntime()
HOST_PORTS = (_RT.gateway_port, *range(_RT.server_base_port, _RT.server_base_port + SLOTS))
THREAD_VARS = ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
               "OPENCV_FOR_THREADS_NUM")
PLANT = {"sym_out": "tmp/kernel/out_901.txt", "hard_out": "tmp/kernel/out_902.txt",
         "plain_out": "tmp/kernel/out_903.txt", "sym_image": "images/9001_link.png",
         "hard_image": "images/9002_hard.png", "plain_image": "images/9003_plain.png",
         "sym_dir": "images/linkdir"}
TRACE_STREAM = "eval.jsonl"
PROXY_VARS = ("HTTPS_PROXY", "https_proxy", "HTTP_PROXY", "http_proxy", "ALL_PROXY", "all_proxy")
REQUIRED = (
    "spawn", "works_episode_rw", "works_sdk_import", "works_subprocess_python",
    "works_relay_to_gateway", "works_relay_log", "works_scientific_stack",
    "relay_blocks_operator_routes", "relay_blocks_other_tokens", "relay_files_readonly",
    "real_token_hidden", "task_name_hidden", "env_allowlisted", "hidden_after_replace",
    "no_direct_gateway", "no_other_local_port", "no_host_ports", "no_external_network",
    "no_read_outside", "no_other_episodes", "no_forbidden_paths", "no_write_outside",
    "no_trusted_trace_write", "no_host_link_reads", "meta_unchanged", "host_files_hidden",
    "no_host_file_writes", "no_process_view", "no_signal_outside", "no_exec_shell",
    "limits_applied", "kernel_uid_not_root")
REQUIRED_BWRAP = ("limit_enforced", "nproc_enforced", "works_fair_cpu_count")


def required_checks(bwrap_flags=frozenset()) -> tuple[str, ...]:
    """The checks that must run and hold for a report to be ok."""
    extra = ("no_nested_userns",) if bwrap.NO_USERNS in bwrap_flags else ()
    return (*REQUIRED, *REQUIRED_BWRAP, *extra)


def verdict(checks: dict, required) -> tuple[bool, list[str]]:
    """(ok, the required checks that are missing): ok needs all required present, all true."""
    missing = [n for n in required if n not in checks]
    return bool(required) and not missing and all(checks.values()), missing


def proxy_targets(environ=os.environ) -> list[tuple[str, int]]:
    """The host's egress proxies as (ip, port): the sandbox must not reach them either."""
    import urllib.parse

    out = []
    for k in PROXY_VARS:
        v = environ.get(k)
        if not v:
            continue
        u = urllib.parse.urlsplit(v if "://" in v else f"http://{v}")
        try:
            out += [(ai[4][0], u.port or 80) for ai in socket.getaddrinfo(
                u.hostname, u.port or 80, socket.AF_INET, socket.SOCK_STREAM)][:1]
        except (OSError, ValueError, TypeError):
            continue
    return sorted(set(out))


def lan_address(probe=EXTERNAL[0][0]) -> str | None:
    """The host's own non-loopback address toward probe (no packet is sent), or None."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((probe, 9))
            ip = s.getsockname()[0]
    except OSError:
        return None
    return None if ip.startswith("127.") or ip == "0.0.0.0" else ip  # nosec B104


def reachable(targets, timeout_s: float = 3.0) -> list[tuple[str, int]]:
    """The targets a TCP connection from the host reaches."""
    out = []
    for host, port in targets:
        try:
            socket.create_connection((host, port), timeout=timeout_s).close()
            out.append((host, port))
        except OSError:
            pass
    return out


class _Gateway(http.server.ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, token):
        self.token = token
        self.hits: list[str] = []
        super().__init__(("127.0.0.1", 0), _GatewayHandler)


class _GatewayHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        self.server.hits.append(self.path)
        ok = self.path.startswith(f"/s/{self.server.token}/")
        data = json.dumps({"ok": ok, "state": "ready"}).encode()
        self.send_response(200 if ok else 403)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    do_POST = do_GET


class _Decoy:
    def __init__(self, host: str = "127.0.0.1"):
        self.sock = socket.create_server((host, 0))
        self.sock.settimeout(0.2)
        self.accepted = 0
        self.stop = False
        threading.Thread(target=self._serve, daemon=True).start()

    @property
    def addr(self):
        return self.sock.getsockname()[:2]

    def _serve(self):
        while not self.stop:
            try:
                c, _ = self.sock.accept()
                self.accepted += 1
                c.close()
            except OSError:
                pass

    def close(self):
        self.stop = True
        self.sock.close()


def _layout(root: Path, token: str, gateway_port: int, task: str) -> dict:
    ep = root / "run" / "episodes" / f"{LIVE_PREFIX}{secrets.token_hex(8)}"
    sdk = ep / "sdk" / SDK_MODULE
    sdk.mkdir(parents=True)
    (sdk / "__init__.py").write_text('MARK = "sdk-ok"\n')
    (ep / EPISODE_FILE).write_text(json.dumps({
        "gateway": f"http://127.0.0.1:{gateway_port}", "token": token, "base": f"s/{token}/",
        "episode_id": "selftest", "instruction": "(self-test)"}))
    (ep / "meta.json").write_text(json.dumps({"task": task, "token": token}))
    for name in HOST_FILES:
        (ep / name).write_text(f"host-{task}\n")
    trace = ep / TRACE_DIR
    (trace / "blobs").mkdir(parents=True)
    (trace / TRACE_STREAM).write_text('{"v":1,"seq":0,"src":"eval","kind":"eval.episode.begin"}\n')
    other = ep.parent / f"{LIVE_PREFIX}{secrets.token_hex(8)}"
    other.mkdir()
    (other / EPISODE_FILE).write_text(json.dumps({"token": "other-" + token, "task": task}))
    outside = root / "outside"
    outside.mkdir()
    (outside / "secret.txt").write_text(task)
    return {"ep": ep, "sdk": ep / "sdk", "outside": outside, "other": other, "trace": trace}


def tree_digest(d: Path) -> dict:
    """{relative path: kind and content hash} for everything under d, links not followed."""
    if d.is_symlink() or not d.is_dir():
        return {"": "missing or not a directory"}
    out = {}
    for root, dirs, files in os.walk(d):
        for name in dirs + files:
            p = Path(root, name)
            rel = str(p.relative_to(d))
            if p.is_symlink():
                out[rel] = "symlink"
            elif p.is_dir():
                out[rel] = "dir"
            else:
                st = p.stat()
                out[rel] = f"{st.st_nlink}:{hashlib.sha256(p.read_bytes()).hexdigest()}"
    return out


def _sample(tops, granted, per_top: int = 150, budget_s: float = 10.0) -> list[str]:
    """Real files under each forbidden root, seen from the host: the probe reads these paths
    directly, as a walk from inside cannot list what the sandbox hides."""
    out, t0 = [], time.monotonic()
    for top in tops:
        n = 0
        for root, dirs, files in os.walk(top):
            dirs[:] = sorted(d for d in dirs if not under(os.path.join(root, d), granted))
            for f in sorted(files)[:10]:
                p = os.path.join(root, f)
                if os.path.isfile(p) and not under(p, granted):
                    out.append(p)
                    n += 1
            if n >= per_top or time.monotonic() - t0 > budget_s:
                break
    return out


def _env(ep: Path, sdk: Path, python: str, secrets_: dict) -> dict:
    """The caller's whole environment plus planted secrets: the sandbox must pass none of it."""
    return {**os.environ, **secrets_, "PATH": f"{Path(python).parent}:/usr/bin:/bin",
            "HOME": str(ep), "TMPDIR": str(ep / "tmp"), "LANG": "C.UTF-8",
            "PYTHONPATH": str(sdk), "PYTHONDONTWRITEBYTECODE": "1", "PYTHONNOUSERSITE": "1",
            "MPLBACKEND": "Agg", "MPLCONFIGDIR": str(ep / "tmp" / "matplotlib"),
            **{k: "4" for k in THREAD_VARS}}


def _replace(ep: Path, task: str, token: str, marker: str) -> None:
    """Rewrite meta.json and episode.json the way the eval layer does: temp file, os.replace."""
    for name, body in (("meta.json", {"task": task, "token": token, "marker": marker}),
                       (EPISODE_FILE, {"token": token, "task": task, "marker": marker})):
        tmp = ep / f".{name}.tmp"
        tmp.write_text(json.dumps(body))
        os.replace(tmp, ep / name)


def _run_probe(cmd, ep: Path, env: dict, args: dict, swap, timeout_s: float):
    """Run the probe; once it is up, swap() replaces the masked files under it, then it goes on."""
    started, replaced = ep / "tmp" / ".probe_started", ep / "tmp" / ".host_replaced"
    p = subprocess.Popen(cmd, cwd=ep, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, text=True, start_new_session=True)
    try:
        assert p.stdin is not None
        p.stdin.write(json.dumps(args))
        p.stdin.close()
        p.stdin = None
        deadline = time.monotonic() + timeout_s
        while not started.exists() and p.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if started.exists():
            swap()
            replaced.touch()
        out, err = p.communicate(timeout=max(1.0, deadline - time.monotonic()))
        return subprocess.CompletedProcess(cmd, p.returncode, out, err)
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()


def _host_link_reads(ep: Path, secret: str) -> tuple[bool, dict]:
    """Run the host-side readers on the planted paths: links refused, plain files read."""
    from amp.kernel.output import read_capture

    def image(rel):
        try:
            safe_read_bytes(ep / rel, ep)
        except LinkRefused:
            return "refused"
        return "ok"

    got = {n: read_capture(ep, n)[0] for n in (901, 902, 903)}
    imgs = {r: image(PLANT[r]) for r in ("sym_image", "hard_image", "plain_image")}
    imgs["sym_dir"] = image(f"{PLANT['sym_dir']}/secret.txt")
    uploaded = [rel for rel, why in walk(ep) if why is None]
    ok = (all(REFUSED in got[n] and secret not in got[n] for n in (901, 902))
          and got[903] == "plain-ok"
          and imgs == {"sym_image": "refused", "hard_image": "refused", "plain_image": "ok",
                       "sym_dir": "refused"}
          and PLANT["plain_image"] in uploaded
          and not {PLANT[r] for r in ("sym_image", "hard_image", "sym_dir")} & set(uploaded)
          and not any(u.startswith(PLANT["sym_dir"] + "/") for u in uploaded))
    seen = {f"out_{n}": "refused" if REFUSED in v else ("secret" if secret in v else v[:40])
            for n, v in got.items()}
    return ok, {**seen, **imgs}


def _probe_args(sb: Sandbox, lay: dict, root: Path, gw: _Gateway, decoy: _Decoy, canary,
                token: str, task: str, marker: str, forbidden) -> dict:
    ep = lay["ep"]
    home = os.path.expanduser("~")
    a = {"episode_file": sb.episode_file, "episode_dir": str(ep), "sdk_module": SDK_MODULE,
         "other_token": secrets.token_urlsafe(18), "real_token": token, "task_marker": task,
         "gateway_addr": ["127.0.0.1", gw.server_address[1]], "decoy_addr": list(decoy.addr),
         "external": [list(x) for x in EXTERNAL], "hostname": HOSTNAME, "canary_marker": marker,
         "canary_pid": canary.pid,
         "unreadable": [str(lay["outside"] / "secret.txt"), str(Path(__file__))],
         "unlistable": [str(lay["outside"]), str(lay["other"])],
         "unwritable": [str(lay["outside"] / "new.txt"), str(lay["sdk"] / SDK_MODULE / "x.py"),
                        f"/tmp/{root.name}-escape.txt", os.path.join(home, f"{root.name}.txt")],  # nosec B108
         "host_ports": list(HOST_PORTS),
         "siblings": [[str(root), lay["outside"].name], [str(ep.parent), lay["other"].name]],
         "other_episode": [str(lay["other"] / EPISODE_FILE)], "plant": PLANT,
         "link_target": str(lay["outside"] / "secret.txt"), "link_dir": str(lay["outside"]),
         "trace": {"dir": str(lay["trace"]), "stream": TRACE_STREAM},
         "limits": dict(sb.rlimits), "limits_skipped": dict(sb.rlimits_skipped),
         "nproc_enforced": sb.nproc_enforced,
         "cpu_share": sb.cpus if sb.backend == "bwrap" else None,
         "replace_marker": "replaced-" + secrets.token_hex(8), "env_allow": sorted(ENV_ALLOW),
         "readonly": [sb.episode_file, str(sb.relay_log)],
         "host_files": [str(ep / n) for n in HOST_FILES], "host_marker": f"host-{task}",
         "host_new": [str(ep / "selftest_root_new.txt"), str(ep / KERNEL_DIR / "new.txt")],
         "uid_opt_out": sb.uid_opted_out, "userns_disabled": bwrap.NO_USERNS in sb.bwrap_flags}
    granted = [*sb.read_roots(), str(ep), os.path.realpath(ep)]
    a["granted"] = granted
    tops = [*forbidden, tempfile.gettempdir(), home, str(REPO)]
    a["forbidden"] = sorted({p for f in tops if os.path.lexists(f)
                             for p in (f, os.path.realpath(f))})
    a["forbidden_files"] = [*_sample([str(lay["outside"]), str(lay["other"])], granted),
                            *_sample(a["forbidden"], granted)]
    return a


def host_network(checks: dict, details: dict, reached: list) -> None:
    """no_external_network means something only when the host itself reached a target."""
    checks["no_external_network"] = checks.get("no_external_network", False) and bool(reached)
    if not reached:
        details["no_external_network"] = "no external target is reachable from the host either"


def _host_checks(checks: dict, details: dict, *, sb: Sandbox, lay: dict, gw: _Gateway,
                 decoy: _Decoy, token: str, task: str, unwritable: list[str],
                 trace_before: dict, args: dict, kernel_before: set) -> None:
    """Complete the probe's checks with what only the host can see."""
    ep = lay["ep"]
    host_network(checks, details, args["external"])
    host_new = [p for p in args["host_new"] if os.path.lexists(p)]
    changed = [n for n in HOST_FILES if (ep / n).read_text() != f"host-{task}\n"]
    kernel_new = sorted(set(os.listdir(ep / KERNEL_DIR)) - kernel_before)
    checks["no_host_file_writes"] = (checks.get("no_host_file_writes", False) and not host_new
                                     and not changed and not kernel_new)
    if host_new or changed or kernel_new:
        details["host_files_changed"] = json.dumps(
            {"created": host_new, "changed": changed, "kernel_new": kernel_new})
    hits = list(gw.hits)
    checks["works_relay_to_gateway"] = (checks.get("works_relay_to_gateway", False)
                                        and f"/s/{token}/_episode" in hits)
    leaked = [h for h in hits if not h.startswith(f"/s/{token}/")]
    checks["relay_blocks_operator_routes"] = (checks.get("relay_blocks_operator_routes", False)
                                              and not leaked)
    checks["no_other_local_port"] = checks.get("no_other_local_port", False) and \
        decoy.accepted == 0
    written = [p for p in unwritable if os.path.exists(p)]
    checks["no_write_outside"] = not written
    checks["meta_unchanged"] = task in (ep / "meta.json").read_text()
    log = sb.relay_log.read_text()
    rows = [json.loads(ln) for ln in log.splitlines() if ln.strip()]
    checks["works_relay_log"] = (sum(r.get("blocked", False) for r in rows) >= 5
                                 and token not in log and "selftest" not in log)
    trace_after = tree_digest(lay["trace"])
    checks["no_trusted_trace_write"] = (checks.get("no_trusted_trace_write", False)
                                        and trace_after == trace_before)
    if trace_after != trace_before:
        details["trace_changed"] = json.dumps(sorted(
            set(trace_after.items()) ^ set(trace_before.items())))[:1000]
    planted = checks.pop("links_planted", False)
    if "links_planted" in details:
        details["no_host_link_reads"] = details.pop("links_planted")
    try:
        host_ok, seen = _host_link_reads(ep, task) if planted else (False, {})
        details["host_link_reads"] = json.dumps(seen)
    except Exception as e:  # noqa: BLE001
        host_ok = False
        details["host_link_reads"] = f"{type(e).__name__}: {e}"[:500]
    checks["no_host_link_reads"] = planted and host_ok
    checks["no_forbidden_paths"] = (checks.get("no_forbidden_paths", False)
                                    and not json.loads(details.get("forbidden_readable", "[]")))
    if leaked:
        details["gateway_leaked"] = json.dumps(leaked)[:1000]
    if written:
        details["written"] = json.dumps(written)
    desc = sb.describe()
    details.update({k: v if isinstance(v, str) else json.dumps(v)
                    for k, v in desc.items() if k != "relay"})
    details["relay"] = json.dumps(desc.get("relay"))


def self_test(backend: str = "auto", *, python: str | None = None,
              workdir: str | Path | None = None, timeout_s: float = 180.0,
              forbidden: tuple[str, ...] = FORBIDDEN) -> SandboxReport:
    """Probe the sandbox end to end."""
    name = resolve_backend(backend)
    python = os.path.abspath(python or sys.executable)
    report = SandboxReport(backend=name, ok=False, checks={}, platform=sys.platform,
                           details={"python": python})
    if name == "none":
        report.details["why"] = "no sandbox backend available or requested"
        return report

    def spawn_failed(why: str) -> SandboxReport:
        report.checks = {"spawn": False}
        report.details["spawn"] = why[:2000]
        return report

    token, task = secrets.token_urlsafe(18), "SelftestTask" + secrets.token_hex(6)
    root = Path(tempfile.mkdtemp(prefix="amp-st-", dir=workdir))
    gw = _Gateway(token)
    threading.Thread(target=gw.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True).start()
    decoy = _Decoy()
    lan = lan_address()
    lan_decoy = _Decoy(lan) if lan else None
    marker = "canary-" + secrets.token_hex(8)
    canary = subprocess.Popen([python, "-c", "import time; time.sleep(600)", marker],
                              env=bwrap.TOOL_ENV)
    sb = None
    try:
        lay = _layout(root, token, gw.server_address[1], task)
        ep = lay["ep"]
        trace_before = tree_digest(lay["trace"])
        try:
            sb = Sandbox(name, ep, python=python, sdk_dir=lay["sdk"]).start()
        except (SandboxError, OSError, RuntimeError, subprocess.SubprocessError) as e:
            return spawn_failed(f"{type(e).__name__}: {e}")
        (ep / "tmp").mkdir(exist_ok=True)
        (ep / "images").mkdir(exist_ok=True)
        args = _probe_args(sb, lay, root, gw, decoy, canary, token, task, marker, forbidden)
        targets = [*EXTERNAL, *([tuple(lan_decoy.addr)] if lan_decoy else []), *proxy_targets()]
        args["external"] = [list(t) for t in reachable(targets)]
        report.details["external_reachable_from_host"] = json.dumps(args["external"])
        kernel_before = set(os.listdir(ep / KERNEL_DIR))
        planted = {"ANTHROPIC_API_KEY": "sk-ant-" + secrets.token_hex(12),
                   "OPENAI_API_KEY": "sk-" + secrets.token_hex(12),
                   "HTTPS_PROXY": "http://proxy-" + secrets.token_hex(6) + ":1"}
        args["env_secrets"] = list(planted.values())
        cmd = sb.wrap([python, "-c", PROBE_SOURCE])
        try:
            out = _run_probe(cmd, ep, sb.env(_env(ep, lay["sdk"], python, planted)), args,
                             lambda: _replace(ep, task, token, args["replace_marker"]),
                             timeout_s)
        except subprocess.TimeoutExpired:
            return spawn_failed(f"the probe ran over {timeout_s:g} s")
        except OSError as e:
            return spawn_failed(f"{type(e).__name__}: {e}")
        try:
            got = json.loads(out.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            return spawn_failed(f"exit {out.returncode}: {(out.stderr or out.stdout)[-2000:]}")
        checks, details = dict(got["checks"]), {**report.details, **got["details"]}
        checks["spawn"] = True
        _host_checks(checks, details, sb=sb, lay=lay, gw=gw, decoy=decoy, token=token, task=task,
                     unwritable=args["unwritable"], trace_before=trace_before, args=args,
                     kernel_before=kernel_before)
        details["python"] = python
        details["skipped"] = json.dumps(got.get("skipped", {}))
        required = required_checks(sb.bwrap_flags)
        report.ok, missing = verdict(checks, required)
        details["required"] = json.dumps(list(required))
        if missing:
            details["missing_required"] = json.dumps(missing)
        report.checks, report.details = checks, details
        return report
    finally:
        if sb is not None:
            sb.close()
        canary.kill()
        canary.wait()
        decoy.close()
        if lan_decoy:
            lan_decoy.close()
        gw.shutdown()
        gw.server_close()
        for p in (f"/tmp/{root.name}-escape.txt",  # nosec B108
                  os.path.join(os.path.expanduser("~"), f"{root.name}.txt")):
            if os.path.exists(p):
                os.unlink(p)
        shutil.rmtree(root, ignore_errors=True)


def require_ok(report: SandboxReport) -> SandboxReport:
    """Raise SandboxError unless the report is ok: no agent runs without a working sandbox."""
    if not report.ok:
        missing = json.loads(report.details.get("missing_required", "[]"))
        bad = (sorted(k for k, v in report.checks.items() if not v)
               + [f"{m} (missing)" for m in missing]) or ["(no checks ran)"]
        raise SandboxError(f"sandbox {report.backend} failed its self-test: {', '.join(bad)}")
    return report


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    p.add_argument("--backend", default="auto")
    p.add_argument("--python", default=None, help="the kernel interpreter (default: this one)")
    p.add_argument("--out", default=None, help="also write the report JSON here")
    a = p.parse_args(argv)
    r = self_test(a.backend, python=a.python)
    text = json.dumps(dataclasses.asdict(r), indent=2, sort_keys=True)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n")
    return 0 if r.ok else 1


if __name__ == "__main__":
    sys.exit(main())
