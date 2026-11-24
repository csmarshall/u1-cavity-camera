#!/usr/bin/env python3
"""
Parametric two-piece case for the ELP-USB8MP05AF-V62 strip camera module.

Published envelope: 62.3 x 9 x 6 mm (L x W x total stack). ELP publishes no
mechanical drawing, so lens position, connector position and PCB thickness are
ASSUMPTIONS exposed as parameters below. Measure the board and adjust.

Coordinate system: X = board length, Y = board width, Z = up (lens faces +Z).
Origin is the centre of the board footprint, Z=0 is the outside of the base floor.

Usage:
    python3 elp_v62_case.py [--out DIR] [--log-level DEBUG]
Requires: cadquery (pip install cadquery)
"""
import argparse
import logging
import os
from dataclasses import dataclass, asdict

import cadquery as cq

log = logging.getLogger("elp_v62_case")


@dataclass
class Params:
    # --- Board (published) ---
    pcb_l: float = 62.3          # board length
    pcb_w: float = 9.0           # board width
    stack_h: float = 6.0         # total module thickness incl. lens

    # --- Board (ASSUMED - verify with calipers) ---
    pcb_t: float = 1.0           # bare PCB thickness
    lens_x: float = 0.0          # lens centre offset from board centre along X
    lens_y: float = 0.0          # lens centre offset from board centre along Y

    # --- Fit ---
    fit: float = 0.3             # clearance per side around the PCB
    back_gap: float = 2.5        # space under PCB for solder joints / back-side parts / wires
    front_gap: float = 0.4       # air above the tallest front-side part

    # --- Shell ---
    wall: float = 1.8            # long side wall thickness
    floor: float = 1.6           # base floor thickness
    lid_t: float = 1.6           # lid plate thickness
    rail_w: float = 0.8          # width of the ledge the PCB edges sit on
    corner_r: float = 2.0        # outer vertical corner radius

    # --- Lens window ---
    window_d: float = 8.5        # through-hole diameter (clears ~76 deg diagonal FOV)
    window_chamfer: float = 0.8  # outward chamfer so the lid edge stays out of frame

    # --- Fasteners (M2 self-tapping or M2 machine screw into plastic) ---
    ear_l: float = 7.5           # solid block at each end that takes the lid screw
    screw_pilot_d: float = 1.7   # pilot in base (use 3.2 for M2 heat-set inserts)
    screw_clear_d: float = 2.4   # clearance in lid
    screw_depth: float = 7.0     # pilot depth

    # --- Cable exit: U-slot in a long wall, open to the lid, full cavity height ---
    cable_x: float = 26.0        # slot centre along X (negative = other end)
    cable_side: int = 1          # +1 = +Y wall, -1 = -Y wall
    cable_w: float = 3.6         # slot width

    # --- PCB hold-down ribs on the lid (press on the PCB long edges) ---
    rib_xs: tuple = (-20.0, 20.0)  # rib centres along X; move if they foul a component
    rib_l: float = 5.0
    rib_w: float = 0.7           # includes the fit gap, so ~0.4 mm bears on the PCB edge
    rib_preload: float = 0.1     # ribs stop this far short of the PCB (print tolerance)

    # --- Derived ---
    @property
    def cav_l(self): return self.pcb_l + 2 * self.fit

    @property
    def cav_w(self): return self.pcb_w + 2 * self.fit

    @property
    def out_l(self): return self.cav_l + 2 * self.ear_l

    @property
    def out_w(self): return self.cav_w + 2 * self.wall

    @property
    def z_pcb_bottom(self): return self.floor + self.back_gap

    @property
    def z_pcb_top(self): return self.z_pcb_bottom + self.pcb_t

    @property
    def base_h(self):
        # Walls rise to cover the whole stack plus a little air
        return self.z_pcb_top + (self.stack_h - self.pcb_t) + self.front_gap

    @property
    def screw_x(self): return self.cav_l / 2 + self.ear_l / 2


def validate(p: Params) -> None:
    """Fail early on parameter combinations that cannot produce a usable part."""
    if p.rail_w * 2 >= p.cav_w:
        raise ValueError("rail_w too large: rails would close the cavity")
    if abs(p.cable_x) + p.cable_w / 2 > p.cav_l / 2:
        raise ValueError("cable slot falls outside the cavity; reduce |cable_x|")
    if p.cable_side not in (1, -1):
        raise ValueError("cable_side must be +1 or -1")
    if p.screw_depth >= p.base_h - 0.8:
        raise ValueError("screw_depth would break through the base floor")
    if p.window_d + 2 * p.window_chamfer > p.out_w - 1.0:
        raise ValueError("lens window (incl. chamfer) is wider than the lid")
    for x in p.rib_xs:
        if abs(x) + p.rib_l / 2 > p.cav_l / 2:
            raise ValueError(f"hold-down rib at x={x} is outside the cavity")
        if abs(x - p.cable_x) < (p.rib_l + p.cable_w) / 2:
            log.warning("Rib at x=%.1f overlaps the cable slot at x=%.1f", x, p.cable_x)
    log.debug("Parameter validation passed")


def build_base(p: Params) -> cq.Workplane:
    """Tray: PCB drops in lens-up and rests on two side rails."""
    log.info("Building base: %.1f x %.1f x %.1f mm", p.out_l, p.out_w, p.base_h)

    base = (cq.Workplane("XY").box(p.out_l, p.out_w, p.base_h, centered=(True, True, False))
            .edges("|Z").fillet(p.corner_r))

    # Main cavity from the floor up
    cavity = (cq.Workplane("XY").workplane(offset=p.floor)
              .box(p.cav_l, p.cav_w, p.base_h, centered=(True, True, False)))
    base = base.cut(cavity)

    # Side rails the PCB long edges sit on (floor -> PCB underside)
    for side in (1, -1):
        y = side * (p.cav_w / 2 - p.rail_w / 2)
        rail = (cq.Workplane("XY").workplane(offset=p.floor).center(0, y)
                .box(p.cav_l, p.rail_w, p.back_gap, centered=(True, True, False)))
        base = base.union(rail)
    log.debug("PCB seat at z=%.2f, PCB top at z=%.2f", p.z_pcb_bottom, p.z_pcb_top)

    # Cable U-slot: through one long wall (and its rail), floor-top to wall-top
    slot_y = p.cable_side * (p.cav_w / 2 + p.wall / 2 - p.rail_w / 2)
    slot = (cq.Workplane("XY").workplane(offset=p.floor).center(p.cable_x, slot_y)
            .box(p.cable_w, p.wall + p.rail_w + 0.2, p.base_h, centered=(True, True, False)))
    base = base.cut(slot)
    log.debug("Cable slot at x=%.1f on %s wall", p.cable_x, "+Y" if p.cable_side > 0 else "-Y")

    # Screw pilots, drilled down from the top face of each end block
    base = (base.faces(">Z").workplane(centerOption="CenterOfBoundBox")
            .pushPoints([(p.screw_x, 0), (-p.screw_x, 0)])
            .hole(p.screw_pilot_d, p.screw_depth))
    return base


def build_lid(p: Params) -> cq.Workplane:
    """Flat lid with lens window and hold-down ribs. Modelled in assembled position."""
    z0 = p.base_h
    log.info("Building lid: %.1f x %.1f x %.1f mm, window d=%.1f at (%.1f, %.1f)",
             p.out_l, p.out_w, p.lid_t, p.window_d, p.lens_x, p.lens_y)

    lid = (cq.Workplane("XY").workplane(offset=z0)
           .box(p.out_l, p.out_w, p.lid_t, centered=(True, True, False))
           .edges("|Z").fillet(p.corner_r))

    # Lens window, chamfered on the outside face
    lid = (lid.faces(">Z").workplane(centerOption="CenterOfBoundBox")
           .center(p.lens_x, p.lens_y)
           .cskHole(p.window_d, p.window_d + 2 * p.window_chamfer, 90))

    # Screw clearance holes
    lid = (lid.faces(">Z").workplane(centerOption="CenterOfBoundBox")
           .pushPoints([(p.screw_x, 0), (-p.screw_x, 0)])
           .hole(p.screw_clear_d))

    # Hold-down ribs: hang from the lid underside to just above the PCB edges.
    # They also key the lid in Y against the base walls.
    rib_h = z0 - p.z_pcb_top - p.rib_preload
    key_gap = 0.15  # rib-to-wall clearance so the lid drops in without sanding
    for x in p.rib_xs:
        for side in (1, -1):
            y = side * (p.cav_w / 2 - key_gap - p.rib_w / 2)
            rib = (cq.Workplane("XY").workplane(offset=z0 - rib_h).center(x, y)
                   .box(p.rib_l, p.rib_w, rib_h, centered=(True, True, False)))
            lid = lid.union(rib)
    log.debug("Rib height %.2f mm, %d ribs", rib_h, 2 * len(p.rib_xs))
    return lid


def self_check(p: Params, base: cq.Workplane, lid: cq.Workplane) -> None:
    """Interference checks with a mock board so a bad parameter set is caught before printing."""
    pcb = (cq.Workplane("XY").workplane(offset=p.z_pcb_bottom)
           .box(p.pcb_l, p.pcb_w, p.pcb_t, centered=(True, True, False)))
    checks = {
        "base vs lid": base.intersect(lid),
        "base vs PCB": base.intersect(pcb),
        "lid vs PCB": lid.intersect(pcb),
    }
    for name, solid in checks.items():
        vol = solid.val().Volume() if solid.vals() else 0.0
        log.debug("Interference %s: %.4f mm^3", name, vol)
        if vol > 1e-6:
            raise RuntimeError(f"Interference detected ({name}): {vol:.3f} mm^3")
    log.info("Self-check passed: no interference between base, lid and mock PCB")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="out", help="output directory")
    ap.add_argument("--log-level", default=os.environ.get("LOG_LEVEL", "INFO"),
                    choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%Y-%m-%dT%H:%M:%S%z")

    p = Params()
    log.debug("Parameters: %s", asdict(p))
    try:
        validate(p)
        base = build_base(p)
        lid = build_lid(p)
        self_check(p, base, lid)
    except Exception:
        log.exception("Case generation failed")
        raise

    os.makedirs(args.out, exist_ok=True)
    # Lid is flipped for export so its outer face sits on the bed (ribs up, no supports)
    lid_print = lid.translate((0, 0, -(p.base_h + p.lid_t))).rotate((0, 0, 0), (1, 0, 0), 180)
    exports = {
        "elp_v62_base.stl": base,
        "elp_v62_lid.stl": lid_print,
        "elp_v62_case.step": cq.Assembly().add(base, name="base").add(lid, name="lid").toCompound(),
    }
    for name, shape in exports.items():
        path = os.path.join(args.out, name)
        cq.exporters.export(shape if not isinstance(shape, cq.Workplane) else shape, path)
        log.info("Wrote %s (%d bytes)", path, os.path.getsize(path))
    log.info("Assembled outer size: %.1f x %.1f x %.1f mm", p.out_l, p.out_w, p.base_h + p.lid_t)


if __name__ == "__main__":
    main()
