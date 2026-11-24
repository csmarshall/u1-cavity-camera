#!/usr/bin/env python3
"""
Re-embed freshly generated STLs into viewer/index.html.

The viewer is a single self-contained HTML page (three.js from jsDelivr) with the
base, lid and mock camera embedded as base64 STL so it works offline and as a
hosted artifact. Run after cad/elp_16mp_case.py:

    python3 cad/elp_16mp_case.py --out out
    python3 tools/build_viewer.py --stl-dir out

Expects in --stl-dir: elp_16mp_base.stl, view_lid.stl (assembled position),
view_module.stl (mock camera, assembled position), viewer_params.json (lens
position and FOV from Params, so the viewer's FOV pyramid never drifts).

Also embeds every PNG in --renders-dir (from tools/render_previews.py) into the
"Review renders" overlay, so the hosted page shows the section sheets too.
"""
import argparse
import base64
import glob
import json
import logging
import os
import re

log = logging.getLogger("build_viewer")


def b64(path: str) -> str:
    """Base64 of a file's bytes."""
    with open(path, "rb") as f:
        data = f.read()
    log.debug("Encoded %s (%d bytes)", path, len(data))
    return base64.b64encode(data).decode()


def main() -> None:
    """Rewrite the DATA and RENDERS blocks of the viewer in place."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stl-dir", default="out")
    ap.add_argument("--viewer", default="viewer/index.html")
    ap.add_argument("--renders-dir", default="docs/renders", help="PNGs to embed in the renders overlay")
    ap.add_argument("--subtitle", help="replace the panel subtitle text")
    ap.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%Y-%m-%dT%H:%M:%S%z")

    with open(args.viewer) as f:
        html = f.read()
    # Base as displayed: base + cable cap when the case script wrote it, else the printable base
    base_stl = os.path.join(args.stl_dir, "view_base.stl")
    if not os.path.exists(base_stl):
        base_stl = os.path.join(args.stl_dir, "elp_16mp_base.stl")
    log.info("Viewer base mesh: %s", base_stl)
    data = 'const DATA={base:"%s",lid:"%s",mod:"%s"};' % (
        b64(base_stl),
        b64(os.path.join(args.stl_dir, "view_lid.stl")),
        b64(os.path.join(args.stl_dir, "view_module.stl")))
    html, n = re.subn(r"const DATA=\{.*?\};", lambda _: data, html, count=1, flags=re.S)
    if n != 1:
        raise SystemExit("DATA block not found in viewer; refusing to write")
    with open(os.path.join(args.stl_dir, "viewer_params.json")) as f:
        geom = "const GEOM=%s;" % json.dumps(json.load(f), separators=(",", ":"))
    html, n = re.subn(r"const GEOM=\{.*?\};", lambda _: geom, html, count=1, flags=re.S)
    if n != 1:
        raise SystemExit("GEOM block not found in viewer; refusing to write")
    log.debug("Embedded %s", geom)
    # Mount configurations (written by cad/u1_front_mount.py --viewer): meshes + transforms
    mounts_json = os.path.join(args.stl_dir, "viewer_mounts.json")
    mounts = {}
    if os.path.exists(mounts_json):
        with open(mounts_json) as f:
            meta = json.load(f)
        for key, m in meta.items():
            entry = {k: v for k, v in m.items() if k != "files"}
            entry["fixed"] = b64(os.path.join(args.stl_dir, m["files"]["fixed"]))
            if "cradle" in m["files"]:
                entry["cradle"] = b64(os.path.join(args.stl_dir, m["files"]["cradle"]))
            for k in ("case_base", "case_lid"):
                if k in m["files"]:
                    entry[k] = b64(os.path.join(args.stl_dir, m["files"][k]))
            mounts[key] = entry
    mounts_js = "const MOUNTS=%s;" % json.dumps(mounts, separators=(",", ":"))
    html, n = re.subn(r"const MOUNTS=\{.*?\};", lambda _: mounts_js, html, count=1, flags=re.S)
    if n != 1:
        raise SystemExit("MOUNTS block not found in viewer; refusing to write")
    log.info("Embedded %d mount configurations", len(mounts))
    pngs = sorted(glob.glob(os.path.join(args.renders_dir, "*.png")))
    renders = "const RENDERS=%s;" % json.dumps(
        [[os.path.splitext(os.path.basename(f))[0], b64(f)] for f in pngs], separators=(",", ":"))
    html, n = re.subn(r"const RENDERS=\[.*?\];", lambda _: renders, html, count=1, flags=re.S)
    if n != 1:
        raise SystemExit("RENDERS block not found in viewer; refusing to write")
    log.info("Embedded %d renders from %s", len(pngs), args.renders_dir)
    if args.subtitle:
        html, n = re.subn(r'(<div class="sub">).*?(</div>)', lambda m: m.group(1) + args.subtitle + m.group(2),
                          html, count=1, flags=re.S)
        log.debug("Subtitle replaced: %d", n)
    with open(args.viewer, "w") as f:
        f.write(html)
    log.info("Wrote %s (%d bytes)", args.viewer, len(html))


if __name__ == "__main__":
    main()
