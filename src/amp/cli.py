#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""amp: evaluate LLM agents on robot harnesses, and report on the runs."""

from __future__ import annotations

import sys

ARGV_NOTE = "Arguments after a delegated command are passed to it unchanged."


class Cmd:
    """One node of the command tree: a group of subcommands, or a leaf that delegates."""

    __slots__ = ("bare", "default", "help", "landing", "prepend", "sub", "target")

    def __init__(self, help, target=None, *, prepend=(), sub=None, default=None, bare=True,
                 landing=""):
        self.help = help
        self.target = target
        self.prepend = tuple(prepend)
        self.sub = sub
        self.default = default
        self.bare = bare
        self.landing = landing


def _tree() -> dict[str, Cmd]:
    return {
        "eval": Cmd("run an evaluation", default="robolab", bare=False, sub={
            "robolab": Cmd("RoboLab-120, through RoboLab's official runner", "amp.eval.robolab_official:main",
                           landing="the RoboLab launcher"),
            "capgym": Cmd("CaP-Gym (LIBERO-PRO, robosuite, BEHAVIOR-1K) on local GPUs",
                          "amp.eval.capgym_official:main", landing="the CaP-Gym launcher"),
            "robodojo": Cmd("RoboDojo, through RoboDojo's official runner, on local GPUs",
                            "amp.eval.robodojo_official:main", landing="the RoboDojo launcher"),
        }),
        "practice": Cmd("learn a workspace notebook from RoboDojo practice (docs/practice.md)",
                        bare=True, sub={
            "fewshot": Cmd("one notebook per task, practised in batches (notebook v3)",
                           "amp.practice.fewshot:main", landing="the practice drivers"),
            "continual": Cmd("one notebook learned over every task, one task at a time",
                             "amp.practice.continual:main", landing="the practice drivers"),
        }),
        "doctor": Cmd("check this machine for a RoboLab run (--robodojo: RoboDojo)", "amp.doctor:main",
                      landing="the machine check"),
    }


TREE = _tree()
ROOT = Cmd("", sub=TREE)


def _parser(prog: str, node: Cmd):
    """A group's argparse parser, for help text only: dispatch is by name, so argv is untouched."""
    import argparse

    top = node is ROOT
    epilog = ARGV_NOTE
    if node.default:
        when = "With no subcommand, or options only" if node.bare else "With options only"
        epilog += f" {when}, this runs {prog} {node.default}."
    if top:
        epilog += " Run any command with --help for its options."
    ap = argparse.ArgumentParser(prog=prog, description=__doc__ if top else node.help,
                                 epilog=epilog)
    if top:
        ap.add_argument("--version", action="store_true", help="the package version and commit")
    sub = ap.add_subparsers(title="commands", metavar="<command>")
    for name, c in node.sub.items():
        sub.add_parser(name, help=c.help)
    return ap


class Missing(Exception):
    """A delegate's package is not in this checkout."""


def _resolve(node: Cmd):
    if callable(node.target):
        return node.target
    import importlib

    module, _, attr = node.target.partition(":")
    try:
        mod = importlib.import_module(module)
    except ModuleNotFoundError as e:
        if e.name and (e.name == module or module.startswith(e.name + ".")):
            raise Missing(module) from None
        raise
    fn = getattr(mod, attr or "main", None)
    if callable(fn):
        return fn
    raise AttributeError(f"{module} has no {attr or 'main'}")


def _exit_code(code) -> int:
    if code is None:
        return 0
    if isinstance(code, int):
        return code
    print(code, file=sys.stderr)
    return 1


def _missing_text(node: Cmd, module: str) -> str:
    what = f" ({node.landing})" if node.landing else ""
    return f"this checkout has no {module}{what} yet; update AMP and retry"


def _call(path: str, node: Cmd, argv: list[str]) -> int:
    try:
        fn = _resolve(node)
    except Missing as e:
        print(f"amp {path}: {_missing_text(node, e.args[0])}", file=sys.stderr)
        return 2
    try:
        return _exit_code(fn([*node.prepend, *argv]))
    except Missing as e:
        print(f"amp {path}: {_missing_text(node, e.args[0])}", file=sys.stderr)
        return 2
    except SystemExit as e:
        return _exit_code(e.code)
    except KeyboardInterrupt:
        return 130


def _version() -> str:
    import subprocess
    from pathlib import Path

    from amp import __version__

    root = Path(__file__).resolve().parents[2]
    try:
        out = subprocess.run(["git", "-C", str(root), "rev-parse", "--show-toplevel", "HEAD"],
                             capture_output=True, text=True, timeout=5, check=False)
        top, sha = out.stdout.split()
        if out.returncode == 0 and Path(top).resolve() == root:
            dirty = subprocess.run(["git", "-C", str(root), "status", "--porcelain",
                                    "--untracked-files=no"], capture_output=True, text=True,
                                   timeout=5, check=False).stdout.strip()
            return f"amp {__version__} (git {sha[:12]}{', modified' if dirty else ''})"
    except (OSError, ValueError, subprocess.SubprocessError):
        pass
    return f"amp {__version__}"


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ["--version"]:
        print(_version())
        return 0
    node, path = ROOT, []
    while node.sub is not None:
        prog = " ".join(["amp", *path])
        head = argv[0] if argv else None
        if head in ("-h", "--help"):
            print(_parser(prog, node).format_help(), end="")
            return 0
        if head in node.sub:
            argv = argv[1:]
            node, path = node.sub[head], [*path, head]
        elif node.default and (head.startswith("-") if head else node.bare):
            node, path = node.sub[node.default], [*path, node.default]
        elif head is None:
            print(_parser(prog, node).format_help(), file=sys.stderr, end="")
            return 2
        else:
            ap = _parser(prog, node)
            print(f"{ap.format_usage().rstrip()}\n{prog}: error: unknown command {head!r} "
                  f"(choose from {', '.join(node.sub)})", file=sys.stderr)
            return 2
    return _call(" ".join(path), node, argv)


if __name__ == "__main__":
    sys.exit(main())
