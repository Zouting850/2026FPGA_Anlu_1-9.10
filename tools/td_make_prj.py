#!/usr/bin/env python3
"""Generate a TangDynasty run-directory ``.prj`` snapshot from a ``.al`` project.

WHY THIS EXISTS
---------------
The headless run-directory flow (``tools/td_flow_exit.tcl`` driving
``DefaultFlow.tcl`` inside ``<prj>_Runs/syn_1``) starts with

    open_project vision_sub_mN.prj

i.e. it opens the **.prj snapshot**, not the ``.al`` project.  TD only writes
that snapshot when it *creates* the run directories.  Once
``<prj>_Runs/{syn_1,phy_1}`` already exist -- or if the snapshot is deleted --
``open_project`` on the ``.al`` silently does nothing to them and the flow dies
with the thoroughly unhelpful

    HDL-8001 ERROR: Incorrect top-level module name "top_vision_mN", please reset

because zero design files were loaded.  This script regenerates the snapshot
deterministically so the run-directory flow can be driven at any time.

THE TRANSFORM
-------------
``.al`` and the per-run ``.prj`` are the same XML schema; only three things
differ.  The rule below was verified **byte-for-byte** against the known-good
pair ``vision_sub_m1.al`` <-> ``vision_sub_m1_Runs/syn_1/vision_sub_m1.prj``
(only the ``RunTime`` stamp and line-ending details excluded):

1. the ``<Project>`` tag swaps the absolute ``Path="<project dir>"`` attribute
   for a ``RunTime="<ISO-8601 local time>"`` attribute;
2. every ``<File Path="...">`` gains two extra ``../`` levels, because the
   snapshot is read from ``<prj>_Runs/<run>/`` instead of the project dir;
3. the ``<Runs>`` and ``<Project_Settings>`` sections are dropped -- for this
   flow the run type / start / end step live in ``settings.cfg``.

On top of that, the Verilog entries are re-marked:
4. a source file that declares **no module** (a pure ``define``/``include``
   header such as ``vision_def.v``) is tagged
   ``<Attr Name="AutoExcluded" Val="true"/>``; a file that does declare modules
   must NOT carry the attribute.  This is what TD itself does when it writes
   the snapshot, and it is exactly the attribute whose stale value silently
   broke M4/M5: a snapshot taken while the working copy was incomplete had
   ``AutoExcluded="true"`` baked onto *every* module file, so the design
   elaborated to nothing and TD reported only::

       HDL-8001 ERROR: Incorrect top-level module name "top_vision_mN"

USAGE
-----
    python tools/td_make_prj.py <path/to/project.al> <path/to/output.prj>

The output keeps the input's line endings (the TD project files in this repo
are CRLF) and never touches the input.
"""

from __future__ import annotations

import re
import sys
from datetime import datetime
from pathlib import Path

# <Project Version="3" Minor="2" Path="D:/...">  ->  ... RunTime="2026-09-14T18:14:42"
_RE_PROJECT = re.compile(r'(<Project\b[^>]*?)\s+Path="[^"]*"')
# <File Path="../user_source/...">  ->  <File Path="../../../user_source/...">
_RE_FILEPATH = re.compile(r'(<File Path=")(\.\./)+')
# the two sections the .prj drops, inclusive of their tags
_RE_SECTIONS = re.compile(
    r"[ \t]*<Runs>.*?</Runs>\s*[ \t]*<Project_Settings>.*?</Project_Settings>\s*",
    re.DOTALL,
)
# one <File ...> entry inside <Verilog>
_RE_VLOG_ENTRY = re.compile(r'<File Path="([^"]+\.v)">(.*?)</File>', re.DOTALL)
_RE_ATTR_AUTOEXCL = re.compile(r'[ \t]*<Attr Name="AutoExcluded" Val="true"/>\r?\n')
_RE_MODULE = re.compile(r'^\s*module\b', re.MULTILINE)


def _reorder_verilog_attrs(text: str, al_dir: Path) -> str:
    """Mark include-only sources AutoExcluded, clear it on everything else."""

    eol = "\r\n" if "\r\n" in text else "\n"

    def one(m: re.Match[str]) -> str:
        rel, body = m.group(1), m.group(2)
        src = (al_dir / rel).resolve()
        try:
            content = src.read_text(encoding="utf-8", errors="replace")
            include_only = _RE_MODULE.search(content) is None
        except OSError:
            # Can't read it -> leave TD's own decision alone.
            include_only = 'Name="AutoExcluded"' in body
        body = _RE_ATTR_AUTOEXCL.sub("", body)
        if include_only:
            body = body.replace(
                "<FileInfo>",
                '<FileInfo>' + eol + '                    <Attr Name="AutoExcluded" Val="true"/>',
                1,
            )
        return f'<File Path="{rel}">{body}</File>'

    return _RE_VLOG_ENTRY.sub(one, text)


def al_to_prj(text: str, runtime: str, al_dir: Path) -> str:
    """Apply the transforms.  ``text`` keeps its original line endings."""
    out, n_proj = _RE_PROJECT.subn(rf'\1 RunTime="{runtime}"', text, count=1)
    if n_proj != 1:
        raise SystemExit(f"td_make_prj: expected exactly one <Project ... Path=...>, found {n_proj}")

    # Order matters: the module probe resolves <File Path=...> against the .al's
    # own directory, so it has to run BEFORE the paths are pushed two levels
    # deeper for the run directory.
    out = _reorder_verilog_attrs(out, al_dir)

    out, n_path = _RE_FILEPATH.subn(r"\1../../../", out)
    if n_path == 0:
        raise SystemExit("td_make_prj: no <File Path=...> entries rewritten")

    out, n_sec = _RE_SECTIONS.subn("", out)
    if n_sec != 1:
        raise SystemExit(f"td_make_prj: expected <Runs>+<Project_Settings> once, found {n_sec}")
    return out


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print("usage: td_make_prj.py <project.al> <output.prj>", file=sys.stderr)
        return 2

    src, dst = Path(argv[1]), Path(argv[2])
    raw = src.read_bytes()
    text = raw.decode("utf-8")
    runtime = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    prj = al_to_prj(text, runtime, src.resolve().parent)
    dst.write_bytes(prj.encode("utf-8"))
    print(f"td_make_prj: {src.name} -> {dst}  (RunTime={runtime})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
