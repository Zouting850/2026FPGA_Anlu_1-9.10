#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Render the Mermaid blocks inside README.md to PNG using a locally installed
Chrome / Edge in headless mode, then write a manifest so drift is checkable.

README.md is the single source of truth: the Mermaid source stays in the README
as the canonical, editable form, and doc/architecture/*.png is a generated
delivery for viewers that do not render Mermaid. Never hand-edit the PNGs.

Usage:
    python tools/render_arch_png.py            # render + write manifest
    python tools/render_arch_png.py --check    # no browser: fail if README's
                                               # mermaid source no longer
                                               # matches the rendered manifest
"""

import hashlib
import http.server
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import urllib.request
from urllib.parse import quote

PIPE = "|"
FENCE = "mermaid"
CACHE_URL = "https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.min.js"
PAD = 24
MAX_CSS_WIDTH = 2400

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
README = os.path.join(REPO, "README.md")
OUTDIR = os.path.join(REPO, "doc", "architecture")
MANIFEST = os.path.join(OUTDIR, "render-manifest.json")
CACHE_DIR = os.path.join(tempfile.gettempdir(), "archrender")


def sha256(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def extract_blocks(readme_path):
    """Return [(figure_no, title, source)] for each ```mermaid fence, in order.

    The title comes from the nearest preceding heading so renaming a heading in
    the README changes the output name and forces a re-render.
    """
    with open(readme_path, encoding="utf-8") as f:
        lines = f.read().split("\n")
    blocks = []
    heading = "figure"
    i = 0
    while i < len(lines):
        stripped = lines[i].strip()
        if stripped.startswith("#"):
            heading = stripped.lstrip("#").strip()
        elif stripped == "```" + FENCE:
            j = i + 1
            body = []
            while j < len(lines) and lines[j].strip() != "```":
                body.append(lines[j])
                j += 1
            if j == len(lines):
                raise SystemExit("unterminated mermaid fence starting at line %d" % (i + 1))
            blocks.append((heading, "\n".join(body).strip() + "\n"))
            i = j
        i += 1
    return blocks


def figure_name(heading, ordinal):
    """Stable slug: the figure number when the heading carries one, else the ordinal."""
    token = ""
    for ch in heading.replace("·", " ").split():
        if ch.lower().startswith("fig") or (ch.isdigit() and len(ch) <= 2):
            token = ch.lower().lstrip("fig")
            break
    if not token:
        token = str(ordinal)
    return "arch_fig%s" % token


def ensure_mermaid_js():
    os.makedirs(CACHE_DIR, exist_ok=True)
    path = os.path.join(CACHE_DIR, "mermaid.min.js")
    if not os.path.isfile(path) or os.path.getsize(path) < 1000000:
        sys.stderr.write("fetching mermaid.min.js (one-time, cached in %s)\n" % CACHE_DIR)
        with urllib.request.urlopen(CACHE_URL, timeout=90) as resp:
            blob = resp.read()
        if len(blob) < 1000000:
            raise SystemExit("mermaid.min.js download looks truncated (%d bytes)" % len(blob))
        with open(path, "wb") as f:
            f.write(blob)
    return path


def find_browser():
    candidates = [
        r"C:\Program Files\Google\Chrome\Application\chrome.exe",
        r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
        r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
        r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise SystemExit("no Chrome or Edge found; install either or render manually")


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


def serve(directory):
    handler = lambda *a, **k: QuietHandler(*a, directory=directory, **k)  # noqa: E731
    httpd = socketserver_free_server(handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


def socketserver_free_server(handler):
    import socketserver

    httpd = socketserver.TCPServer(("127.0.0.1", 0), handler)
    httpd.allow_reuse_address = True
    return httpd


HTML_TEMPLATE = """<!doctype html>
<html><head><meta charset="utf-8"><title>PENDING</title>
<script src="./mermaid.min.js"></script>
<style>
  html, body { margin: 0; padding: 0; background: #ffffff; }
  #diagram { display: inline-block; max-width: %(maxwidth)dpx; background: #ffffff; }
  #diagram svg { max-width: %(maxwidth)dpx; height: auto !important; }
</style>
</head><body><div id="diagram"></div>
<script>
function say(m) { document.title = String(m).replace(/[\\r\\n]+/g, ' / '); }
mermaid.initialize({
  startOnLoad: false,
  securityLevel: 'loose',
  theme: 'base',
  fontFamily: '"Microsoft YaHei","PingFang SC","Noto Sans CJK SC",sans-serif',
  flowchart: { htmlLabels: false, useMaxWidth: false, curve: 'basis',
               nodeSpacing: 42, rankSpacing: 62, padding: 10 },
  themeVariables: {
    background: '#ffffff', primaryColor: '#eef4ff', primaryBorderColor: '#33507a',
    primaryTextColor: '#101820', lineColor: '#33507a', fontSize: '15px',
    edgeLabelBackground: '#ffffff'
  }
});
// This script sits after the container on purpose: mermaid.render() appends a
// temporary element to document.body, so running it from <head> throws
// "Cannot read properties of null (reading 'firstChild')".
// The source is passed as a JS string rather than read back out of the page,
// because the parser would otherwise eat the literal <br> tags these node
// labels rely on.
var SOURCE = %(source_js)s;
mermaid.render('arch', SOURCE).then(function (out) {
  var host = document.getElementById('diagram');
  host.innerHTML = out && out.svg ? out.svg : out;
  if (!host.querySelector('svg')) { say('NO-SVG'); return; }
  document.documentElement.setAttribute('data-done', '1');
  say('RENDER-OK');
}).catch(function (e) {
  say('PARSEFAIL ' + (e && e.str ? e.str : (e && e.message ? e.message : String(e))));
});
</script>
</body></html>
"""


def render_one(browser, workdir, name, source, vw, vh, budget):
    from PIL import Image, ImageChops

    html_path = os.path.join(workdir, name + ".html")
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(HTML_TEMPLATE % {"source_js": json.dumps(source), "maxwidth": MAX_CSS_WIDTH})

    httpd = serve(workdir)
    port = httpd.server_address[1]
    url = "http://127.0.0.1:%d/%s" % (port, quote(name + ".html"))
    raw = os.path.join(workdir, name + ".raw.png")

    def launch(extra):
        return subprocess.run(
            [browser, "--headless=new", "--disable-gpu", "--no-sandbox",
             "--hide-scrollbars", "--default-background-color=FFFFFFFF",
             "--force-device-scale-factor=2",
             "--window-size=%d,%d" % (vw, vh),
             "--virtual-time-budget=%d" % budget] + extra,
            capture_output=True, text=True, errors="replace", timeout=240)

    try:
        # Assert the SVG actually exists before trusting any pixels: a failed
        # render still produces a dense, non-blank page (the raw source text),
        # which a blank-frame check alone would wave through.
        dom = launch(["--dump-dom", url])
        title = re.search(r"<title>(.*?)</title>", dom.stdout or "", re.S)
        verdict = title.group(1) if title else "NO-TITLE"
        if verdict != "RENDER-OK":
            raise SystemExit("%s did not render: %s" % (name, verdict[:1500]))
        proc = launch(["--screenshot=%s" % raw, url])
    finally:
        httpd.shutdown()
        httpd.server_close()
    if not os.path.isfile(raw):
        raise SystemExit("browser produced no screenshot for %s\n%s"
                         % (name, (proc.stderr or "")[-800:]))

    img = Image.open(raw).convert("RGB")
    bg = Image.new("RGB", img.size, (255, 255, 255))
    bbox = ImageChops.difference(img, bg).getbbox()
    if bbox is None:
        raise SystemExit("%s rendered blank; increase --budget" % name)
    # --screenshot only captures the viewport, so content that reaches the right
    # or bottom edge is cropped, not merely tall. Fail rather than ship a
    # truncated diagram.
    if bbox[2] >= img.width - 3 or bbox[3] >= img.height - 3:
        raise SystemExit("%s content touches the %dx%d capture edge (bbox %s): "
                         "increase --viewport-w/--viewport-h"
                         % (name, img.width, img.height, bbox))
    left = max(0, bbox[0] - PAD)
    top = max(0, bbox[1] - PAD)
    right = min(img.width, bbox[2] + PAD)
    bottom = min(img.height, bbox[3] + PAD)
    if right - left < 400 or bottom - top < 200:
        raise SystemExit("%s crop too small (%dx%d) - diagram probably failed to lay out"
                         % (name, right - left, bottom - top))
    cropped = img.crop((left, top, right, bottom))
    out = os.path.join(OUTDIR, name + ".png")
    cropped.save(out, optimize=True)
    os.remove(raw)
    colors = len(cropped.convert("RGBA").getcolors(maxcolors=1000000) or [])
    print("  %-16s %5dx%-5d colors=%-6d %s"
          % (name, cropped.width, cropped.height, colors, os.path.relpath(out, REPO)))
    return {"file": name + ".png", "width": cropped.width, "height": cropped.height,
            "sha256_mmd": sha256(source)}


def write_manifest(entries, browser):
    with open(MANIFEST, "w", encoding="utf-8") as f:
        json.dump({"generator": "tools/render_arch_png.py",
                   "source": "README.md",
                   "browser": os.path.basename(browser),
                   "note": "generated - do not hand edit; regenerate with "
                           "python tools/render_arch_png.py",
                   "figures": entries},
                  f, ensure_ascii=False, indent=2)
        f.write("\n")
    print("wrote %s" % os.path.relpath(MANIFEST, REPO))


def check_manifest(blocks):
    if not os.path.isfile(MANIFEST):
        print("CHECK FAIL: %s missing" % os.path.relpath(MANIFEST, REPO))
        return 1
    with open(MANIFEST, encoding="utf-8") as f:
        man = json.load(f)
    recorded = {e["sha256_mmd"]: e["file"] for e in man["figures"]}
    current = [(figure_name(h, i + 1), sha256(s)) for i, (h, s) in enumerate(blocks)]
    bad = []
    for name, digest in current:
        if recorded.get(digest) != name + ".png":
            hit = [f for s, f in recorded.items() if f == name + ".png"]
            bad.append("%s: %s" % (name, "not in manifest" if not hit else "README source changed since render"))
    for e in man["figures"]:
        if e["file"] not in [n + ".png" for n, _ in current]:
            bad.append("%s: stale PNG, no such mermaid block in README" % e["file"])
        elif not os.path.isfile(os.path.join(OUTDIR, e["file"])):
            bad.append("%s: listed in manifest but missing on disk" % e["file"])
    if bad:
        print("CHECK FAIL (%d):" % len(bad))
        for b in bad:
            print("  " + b)
        print("  -> run: python tools/render_arch_png.py")
        return 1
    print("CHECK OK: %d figures match README mermaid sources" % len(current))
    return 0


def main(argv):
    do_check = "--check" in argv
    budget = 40000
    vw, vh = 2600, 3400
    for i, a in enumerate(argv):
        if a == "--budget" and i + 1 < len(argv):
            budget = int(argv[i + 1])
        if a == "--viewport-w" and i + 1 < len(argv):
            vw = int(argv[i + 1])
        if a == "--viewport-h" and i + 1 < len(argv):
            vh = int(argv[i + 1])

    blocks = extract_blocks(README)
    if not blocks:
        raise SystemExit("no mermaid blocks found in README.md")
    print("README.md has %d mermaid blocks" % len(blocks))

    if do_check:
        return check_manifest(blocks)

    os.makedirs(OUTDIR, exist_ok=True)
    js = ensure_mermaid_js()
    browser = find_browser()
    print("browser=%s mermaid=%s" % (os.path.basename(browser), os.path.getsize(js)))
    workdir = tempfile.mkdtemp(prefix="archrender-")
    import shutil
    shutil.copy(js, os.path.join(workdir, "mermaid.min.js"))
    entries = []
    try:
        for ordinal, (heading, source) in enumerate(blocks, start=1):
            name = figure_name(heading, ordinal)
            entries.append(render_one(browser, workdir, name, source, vw, vh, budget))
    finally:
        shutil.rmtree(workdir, ignore_errors=True)
    write_manifest(entries, browser)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
