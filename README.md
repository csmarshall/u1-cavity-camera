# snapmaker-u1-camera-mount

Printable case (and, later, a tilt mount) for the **ELP-USB16MP02-AF100** 16 MP autofocus USB camera board, for mounting on the Snapmaker U1 top cover.

The camera is a 62.3 × 9 × 5.5 mm strip board with a Sony IMX298 sensor and a lens with a ~97° horizontal field of view. It plugs into the U1's rear USB port and shows up as a second camera with the paxx12 Extended Firmware (`usb: paxx12` under `[camera]`).

## What's here

| Path | What |
|---|---|
| `cad/elp_16mp_case.py` | Parametric CadQuery model. All dimensions are in the `Params` dataclass. |
| `stl/elp_16mp_base.stl` | Base: screw-closed (2× M2 countersunk into the end blocks), with heat-shrink cable nozzle. |
| `stl/elp_16mp_lid.stl` | Lid, already flipped for printing. |
| `stl/elp_16mp_coupon_{base,lid}.stl` | Snap test coupon (snap closure only, for PETG/ASA): the -X end of the case (12 mm slice) to test the hook click and release before a full print. |
| `stl/elp_16mp_cable_cap.stl` | Top half of the cable nozzle. Print it split face down. Lay the cable in the base half, put the cap on, close the lid (its tongue closes the end-wall slot), then shrink the tubing over the whole nozzle. |
| `stl/elp_16mp_size_mockup.stl` | Solid block with the assembled case's outside shape, nozzle and lens window included, plus a crosshair on the lens axis. Print it to find a mount spot inside the printer. |
| `cad/u1_front_mount.py` | (Retired: the hinge was a dead end; kept for its shared geometry.) Tilt mount for the U1's front vertical feature: VHB/M3 wall plate, snap-in cradle, M3 friction hinge, and a fixed-angle wedge. `--side above/below`, `--orientation landscape/portrait`, `--wedge-angle N`. |
| `stl/mount_wall_plate.stl`, `stl/mount_cradle.stl` | Hinge, camera above the feature. M3 x 30 socket head, nyloc and washers through the knuckles. |
| `stl/mount_wall_plate_below.stl`, `stl/mount_cradle_below.stl` | Hinge, camera hanging below the feature. The plate goes on upside down, with an arm at each end (M3 x 20 and M3 x 35 pins). |
| `stl/mount_cradle_portrait.stl` | Cradle for the portrait (9:16) orientation, used with the standard wall plate. |
| `stl/mount_wedge*_45deg.stl` | One-piece fixed wedges at 45 degrees. Rebuild with `--wedge-angle` once the hinge has found your angle (read it with a phone inclinometer on the lid). |
| `step/u1_front_mount*.step` | Mount assemblies in printer coordinates: X along the wall, Y into the printer, Z up, wall at Y=0, plus the 16:9 view pyramid. |
| `stl/slidetest_*.stl` | **Slide test mount** (`u1_wedge_kit.py --slide-test`, no cradle or clip). Print this first to find the right camera elevation. The case slides onto the wedge's dovetail tongue from its bottom end (on the bench, before the wedge goes on the plate rail) and clicks every 1 mm, by hand with no tools, from 19 (case top level with the plate top) to 40 (just under the window keep-out). Each click moves the camera about 0.7 mm up and 0.7 mm in. Read the number at the wedge's raised pointer on the case-side scale while watching the stream. Uses `pose_lid_sidepY.stl`, `pose_cable_cap_sidepY.stl` and `kit_wall_plate.stl`. |
| `stl/pose_*.stl` | **Final mount** (`u1_wedge_kit.py --pose ANGLE DY DZ SHIFT`, no cradle or clip). Rebuild it with the reading from the slide test as SHIFT (currently 45, 17.5, 0, 35.3, cable out the X+ Y+ corner). The wedge keys into the case back and is held by 2× countersunk M3 × 10 (not × 6: too little thread; not × 12: bottoms out) driven from inside the case on the bench, before the board goes in, so nothing has to be reached on the printer. |
| `cad/u1_wedge_kit.py` | **Portrait-below test kit** (current direction). One wall plate with a dovetail rail, wedges at 30/45/60/75 degrees, and one universal cradle. Each wedge slides on the rail and carries the cradle on a square key with one countersunk M3 x 10 screw. The cradle fits the end, side +Y and side -Y cases either way up, giving 6 cable exits per angle. Built from Charles's measurements (each rounded down): lip 25 mm, a 15 mm gap under it, the shell at 45 degrees from vertical, and the toolhead at front-most travel 45 mm below and 10 mm in from the lip, as a keep-out zone. The ribs are 57 mm apart at the front centre, so the plate is 55 mm wide. |
| `stl/kit_wall_plate.stl`, `stl/kit_cradle_universal.stl`, `stl/kit_wedge_*deg_slope35.stl` | Kit parts in print orientation. The plate prints back down, the cradle floor down, and the wedges on their side (angle label up). Ready to print. Wedges are `kit_wedge_{30,45,60,75}deg_gap15_slope45.stl`; for other angles, rebuild with `--angles`. |
| `step/elp_16mp_case.step` | Assembled base + lid (screw version) for CAD. |
| `step/elp_16mp_iteration.step` | For positioning the camera in your own CAD. Named bodies: `case_base`, `case_lid`, `ref_camera_mock`, `ref_size_mockup_offset_-40y`, and two 450 mm field-of-view pyramids (`ref_fov_stream_16x9` at 97 × 65°, `ref_fov_sensor_4x3` at 97 × 81°). Units are mm. The origin is the board centre, Z = 0 is the outside of the base floor, the lens faces +Z, X is the board length, and the cable nozzle is at +X. The sensor's long axis is assumed to run along X. |
| `viewer/index.html` | Self-contained three.js viewer with explode, section cut and FOV cone. Open it in a browser. |

## Design

- **Fit**: board drops in lens-up onto two side rails, 0.3 mm clearance per side. Ribs on the lid press the board edges down.
- **Lens window**: a cone in the lid sized from the lens's ~109° diagonal FOV so the lid never vignettes the image.
- **Closure**: 2× M2 × 6 or × 8 countersunk flat heads through the lid into 1.7 mm pilots in the end blocks. Set `closure="snap"` (and `max_strain=0.02`) for the snap-hook version: a cantilever hook at each end of the lid (0.5 mm catch, ~1.9% strain), which only suits PETG/ASA.
- **Cable**: exits the short end through a cable bay (so the 4-pin plug can turn) and a ridged nozzle that is enclosed all round. The base holds the lower half, a separate cap is the upper half, and a tongue on the lid closes the end-wall slot above the cable. Shrink tubing over the nozzle clamps the halves together. The barbs' sheer faces point at the case so the tubing can't slide off. Set `cable_exit="side"` for a long-side slot instead.
- **Self-check**: the script fails if parts collide with each other or with a stand-in for the camera, if a part isn't a single valid solid, or if walls or webs get thinner than 0.8 mm.

Assembled size: 81.9 × 13.2 × 11.6 mm, 92 mm long including the nozzle.

## Assumptions to verify on the real board

ELP publishes no mechanical drawing. These are guesses, and each is a parameter:

- `lens_x`, `lens_y`: lens assumed centred on the board.
- `aperture_d`: front lens element assumed 3 mm.
- `pcb_t`: PCB assumed 1.0 mm thick.
- Connector position unknown. The cable bay assumes the plug ends within `cable_bay_l` (4 mm) of the board end.
- `cable_w`: 3.6 mm bore. Adjust to the actual cable diameter plus about 0.3 mm.

## Printing

- PC-GF (Charles's choice). Printed PC-GF breaks at about 2.5% strain, so everything that flexes is held to 1% (`max_strain` in `cad/elp_16mp_case.py`): the lid is screwed shut, and the slide test's detent finger runs at 0.6%. Glass fibre is abrasive, so use a hardened nozzle. PLA is out: the top of the enclosure runs warm and PLA creeps.
- Base upright, lid outer face down. No supports needed for either part.
- Heat shrink: about 9 mm, 3:1, adhesive-lined, about 20 mm long. Slide it on from the 4-pin plug end before connecting the cable.

## Build

```sh
pip install cadquery
python3 cad/elp_16mp_case.py --out out --log-level DEBUG
```

## Roadmap

- [ ] Tilt mount for the U1 top cover (pivot with 5° detents, or a fixed wedge)
- [ ] Update `lens_x`/`lens_y`/connector parameters from caliper measurements

## Handing off to Claude Code

`CLAUDE.md` has the full design context and conventions. `HANDOFF_PROMPT.md` is the prompt to start a session with.
