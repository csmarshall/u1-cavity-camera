# snapmaker-u1-camera-mount

Printable case (and, later, a tilt mount) for the **ELP-USB16MP02-AF100** 16 MP autofocus USB camera board, for mounting on the Snapmaker U1 top cover.

The camera is a 62.3 × 9 × 5.5 mm strip board with a Sony IMX298 sensor and a lens with a ~97° horizontal field of view. It plugs into the U1's rear USB port and shows up as a second camera with the paxx12 Extended Firmware (`usb: paxx12` under `[camera]`).

## What's here

| Path | What |
|---|---|
| `cad/elp_16mp_case.py` | Parametric CadQuery model. All dimensions are in the `Params` dataclass. |
| `stl/elp_16mp_base.stl` | Base: snap-fit version with heat-shrink cable nozzle. |
| `stl/elp_16mp_lid.stl` | Lid, already flipped for printing. |
| `stl/elp_16mp_*_screw.stl` | Same case closed with 2× M2 countersunk screws instead of snap hooks. |
| `step/elp_16mp_case.step` | Assembled base + lid (snap version) for CAD. |
| `viewer/index.html` | Self-contained three.js viewer with explode, section cut and FOV cone. Open it in a browser. |

## Design

- **Fit**: board drops in lens-up onto two side rails, 0.3 mm clearance per side. Ribs on the lid press the board edges down.
- **Lens window**: a cone in the lid sized from the lens's ~109° diagonal FOV so the lid never vignettes the image.
- **Closure**: one cantilever snap hook at each end of the lid (0.5 mm catch, ~1.9% strain). To open, push each barb in through its window in the end face and lift. Set `closure="screw"` for the screw version (M2 × 6 or × 8 countersunk flat head into 1.7 mm pilots).
- **Cable**: exits the short end through a cable bay (so the 4-pin plug can turn) and a ridged C-channel nozzle. Lay the cable in the channel, then shrink tubing over the nozzle and cable. The barbs' sheer faces point at the case so the tubing can't slide off. Set `cable_exit="side"` for a long-side slot instead.
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

- PETG or ASA. The top of the U1 enclosure gets warm, and PLA snap hooks creep.
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
