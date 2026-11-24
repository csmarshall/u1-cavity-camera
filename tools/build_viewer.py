#!/usr/bin/env python3
"""
Re-embed freshly generated STLs into viewer/index.html.

The viewer is a single self-contained HTML page (three.js from jsDelivr) with the
base, lid and mock camera embedded as base64 STL so it works offline and as a
hosted artifact. Run after cad/elp_16mp_case.py:

    python3 cad/elp_16mp_case.py --out out
    python3 tools/build_viewer.py --stl-dir out

Expects in --stl-dir: elp_16mp_base.stl, view_lid.stl (assembled position),
view_module.stl (mock camera, assembled position).
"""
import argparse
import base64
import logging
import os
import re

log = logging.getLogger("build_viewer")


def b64(path: str) -> str:
    with open(path, "rb") as f:
        data = f.read()
    log.debug("Encoded %s (%d bytes)", path, len(data))
    return base64.b64encode(data).decode()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stl-dir", default="out")
    ap.add_argument("--viewer", default="viewer/index.html")
    ap.add_argument("--subtitle", help="replace the panel subtitle text")
    ap.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%Y-%m-%dT%H:%M:%S%z")

    with open(args.viewer) as f:
        html = f.read()
    data = 'const DATA={base:"%s",lid:"%s",mod:"%s"};' % (
        b64(os.path.join(args.stl_dir, "elp_16mp_base.stl")),
        b64(os.path.join(args.stl_dir, "view_lid.stl")),
        b64(os.path.join(args.stl_dir, "view_module.stl")))
    html, n = re.subn(r"const DATA=\{.*?\};", lambda _: data, html, count=1, flags=re.S)
    if n != 1:
        raise SystemExit("DATA block not found in viewer; refusing to write")
    if args.subtitle:
        html, n = re.subn(r'(<div class="sub">).*?(</div>)', lambda m: m.group(1) + args.subtitle + m.group(2),
                          html, count=1, flags=re.S)
        log.debug("Subtitle replaced: %d", n)
    with open(args.viewer, "w") as f:
        f.write(html)
    log.info("Wrote %s (%d bytes)", args.viewer, len(html))


if __name__ == "__main__":
    main()
