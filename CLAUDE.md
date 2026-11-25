# CLAUDE.md — snapmaker-u1-camera-mount

Context for Claude Code working in this repo. Read this before changing anything.

## What this is

A 3D-printable case, and later a tilt mount, for the **ELP-USB16MP02-AF100** USB camera board. The case mounts inside the top cover of Charles's **Snapmaker U1**, looking down at the bed. The design is fully parametric in CadQuery (`cad/elp_16mp_case.py`). STLs, STEP and the viewer are build outputs that are committed for convenience.

The camera's published specs (Amazon listing for ELP-USB16MP02-AF100):

- Board 62.3 × 9 × 5.5 mm total stack, about 20 g
- Sony IMX298 1/2.8", 4656×3496 (4:3), MJPEG 30 fps at 1080p and 10 fps at full resolution
- Autofocus, M12 lens, HFOV about 97° ("100° no distortion"), focus range 10 cm to infinity
- Built-in mic. USB 2.0 through a **4-pin 1.0 mm socket** (JST-SH style), 1 m cable, 370 mA max

ELP publishes **no mechanical drawing**. Everything beyond the envelope is an assumption, held as a parameter (see "Unknowns").

The predecessor was the ELP-USB8MP05AF-V62 (8 MP IMX179, 62.3 × 9 × 6 mm), dropped in favour of the 16 MP. Its first-pass script is in `archive/` for reference only. An Amazon review says this same board family fits the Voron "Angry CAM USB v2" covers made for USB8MP05AF.

## Printer and software context

- Snapmaker U1 with the Top Cover (enclosure). Charles slices with Snapmaker Orca on macOS.
- Runs **paxx12 Extended Firmware**: full Klipper, Mainsail/Fluidd, WebRTC. The second USB camera is enabled with `usb: paxx12` under `[camera]`, and image settings are in a V4L2 controls panel. It does **not** use Crowsnest. Known issue: the V4L2 panel polling made a Logitech C270 reset every ~13 s. If the ELP feed drops, check that first.
- Charles prints in **PC-GF** (2026-11-24). Printed PC-GF breaks at ~2.5% strain, so flexures get ~1% (`case.Params.max_strain`, the one home for the material limit). That's why the lid is screwed shut by default. The enclosure top runs warm, so PLA creeps.

## Current design state (commit a49edda + this handoff)

`cad/elp_16mp_case.py`, everything in `Params`:

- **Base tray**: the PCB drops in lens-up onto two 0.8 mm side rails, with 0.3 mm/side fit and a 2.5 mm back gap for solder and wires. The walls rise to the lens top plus 0.4 mm.
- **Lid**: a flat 1.6 mm plate. A **conical lens window** is sized from the diagonal FOV (109.4°, derived from HFOV 97° at 4:3), so the lid never vignettes; it goes 5.33 → 9.85 mm. Four hold-down ribs press on the PCB long edges.
- **Closure** (`closure`):
  - `"snap"` (default): a cantilever hook hangs from each lid end into a pocket in the base end block, and its barb clicks into a window in the outer end face. The arm is 0.9 × 4 × 7 mm, the barb 0.7 deep with 0.5 mm engagement, and flex strain about 1.93% against a 2% limit. To release, push the barb in through the window.
  - `"screw"`: 2× M2 countersunk flat-head screws. Charles has the maierke M1.4–M4 flat-head machine screw kit, so use M2×6 or ×8 into 1.7 mm pilots.
- **Cable exit** (`cable_exit`):
  - `"end"` (default): a 4 mm cable bay past the board end, with stop nubs so the board can't slide into the bay. The cable runs through a round-bottom bore with an open-top slot at y = −2.4, axis z = 3.0, then out through a **ridged heat-shrink nozzle**. The nozzle is part of the base only: a C-channel whose bottom sits on the bed, so there are no supports, 3 hose-barb ridges with sheer faces toward the case, and 10.5 mm long. Use 9 mm 3:1 adhesive-lined shrink, slid on from the JST end.
  - `"side"`: the legacy long-wall U-slot.
- Outer size is 81.9 × 13.2 × 11.6 mm, or 92.2 mm long including the nozzle.
- `validate()` checks minimum webs (0.8 mm), hook strain, window vs lid edges, the nozzle vs the snap release window, and so on. `self_check()` checks that each part is a single valid solid and that the base, lid and a mock camera don't interfere (PCB plus an assumed 8 mm square lens block).

## Unknowns: verify on the real board before trusting the fit

| Param | Assumed | How to verify |
|---|---|---|
| `lens_x`, `lens_y` | 0, 0 (centred) | Calipers from one board end to the lens centre |
| lens body | 8 × 8 mm square, in the mock only | Measure the VCM block |
| `aperture_d` | 3.0 mm | Front element clear diameter |
| `pcb_t` | 1.0 mm | Calipers |
| connector position | unknown; the cable bay assumes the plug ends within 4 mm of the board end | Which end, which face, how far the plug overhangs |
| `cable_w` | 3.6 mm | Cable OD + 0.3 |
| back-side components | under 2.5 mm (`back_gap`) | Tallest part on the back |
| sensor orientation | unknown whether the sensor's long axis is along the board's X | Matters for the tilt axis and framing |

## Open design work (what the next session is for)

1. **Tilt mount for the U1 top cover.** The camera mounts inside the cover and tilts along its back edge (the long side opposite the cable? confirm with Charles) to aim at the bed. The ideas discussed:
   - an **adjustable friction pivot**: a block on the cover, a tab on the case back, an M3 screw and nyloc nut, 0–40° with 5° detents. This was Claude's recommendation.
   - a **fixed wedge**, once the angle is known.
   - a **J-clip** over a cover panel edge, which needs the panel thickness.
   - Tilt ≈ atan(horizontal offset to bed centre ÷ height above bed). The bed is 270 × 270 mm.
   - **Needed from Charles:** the mount location on the cover, its height above the bed, VHB vs screws vs clip, and the panel or frame thickness. We don't have U1 top-cover dimensions, so don't guess them; ask.
   - Keep it out of the toolhead and gantry travel, and mount inside the cover, not shooting through the panel (glare).
2. **Margins to revisit:**
   - The nozzle-to-snap-release-window gap is only **0.10 mm**. Consider moving the hook or nozzle, or shrinking the window.
   - Snap hook strain is 1.93%, which fails the PC-GF 1% limit, so `closure="screw"` is now the default.
3. **Add a camera-view preview to the viewer**: a frustum or projected footprint on a 270 mm bed at the chosen tilt and height.
4. **Test coupons**: the snap coupon is done (`stl/elp_16mp_coupon_*.stl`, `coupon_l`). It matters because the hooks flex across layer lines and the 0.7 mm barb catch face prints as an unsupported overhang. A nozzle stub is still to do.
5. After measuring the real board, update the parameters and rerun.

## Commands

```sh
# venv lives outside Dropbox: ~/.venvs/snapmaker-u1-camera-mount (see TOOLS-INSTALLED.md)
pip install -r requirements.txt            # cadquery + matplotlib
python3 cad/elp_16mp_case.py --out out --log-level DEBUG
python3 tools/render_previews.py           # iso (VTK) + exact section PNGs -> docs/renders/
python3 tools/build_viewer.py --stl-dir out --subtitle "…"    # re-embed STLs + renders in viewer/index.html
```

Don't use cadquery's SVG exporter for review renders. It rotates side views arbitrarily and scales from the 3D bounding box. `render_previews.py` explains why.

After a successful build, copy the printable outputs into the committed folders:

- `out/elp_16mp_base.stl` and `out/elp_16mp_lid.stl` go to `stl/`.
- `out/elp_16mp_case.step` goes to `step/`.
- For the screw variant, build with `Params(closure="screw")`. It is written to `stl/*_screw.stl`; see the README.

## Conventions (Charles's preferences)

- He's a staff-level SRE/infra engineer with 20+ years of experience. Be direct, skip basics, and give reasoning for design choices.
- Code: complete files (no "rest unchanged" stubs), keep existing comments, meaningful names, and **logging with timestamps**: INFO for major steps, DEBUG for dimensions and decisions, with a configurable `--log-level`.
- Every geometry change should add or extend a check in `validate()` or `self_check()`, and the build must pass before you report done. Render a picture (`cq.exporters.export(..., "x.svg")` → cairosvg → PNG) and **look at it** for any new feature. The interference checks don't catch "looks wrong".
- Both parts must print **without supports**: base upright, lid outer face down.
- Commit messages: imperative subject plus a body explaining why.

## Repo / GitHub

- Public repo: `csmarshall/u1-cavity-camera` (MIT). Created 2026-11-24 from this folder; the local folder keeps its old name `snapmaker-u1-camera-mount`.
- A hosted copy of the viewer is a private claude.ai artifact (https://claude.ai/artifact/D1nHFNPtZcMUNYMGVeL5zH). `viewer/index.html` here is the source of truth; `tools/build_viewer.py` rebuilds it.
- The mount lives in `cad/u1_front_mount.py` (`--side above|below`, `--orientation landscape|portrait`, `--cable-exit end|side_end`, `--cable-side 1|-1`, `--wedge-angle N`, `--viewer`).
