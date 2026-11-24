#!/usr/bin/env python3
"""
Parametric two-piece case for the ELP-USB16MP02-AF100 strip camera module
(Sony IMX298, 16 MP, autofocus, "100 degree" no-distortion lens, HFOV ~97 deg).

Published envelope: 62.3 x 9 x 5.5 mm (L x W x total stack). ELP publishes no
mechanical drawing, so lens position, lens aperture size, connector position
and PCB thickness are ASSUMPTIONS exposed as parameters below. Measure the
board and adjust.

Lid closure options (closure):
  "snap"  - (default) one cantilever hook hangs from each end of the lid into a
            pocket in the base end block. A barb on the hook clicks into a
            window in the outer end face. To open, push each barb in through its
            window with a pick or small screwdriver and lift the lid. No hardware.
  "screw" - two M2 screws through the lid into pilot holes in the end blocks.

Cable exit options (cable_exit):
  "end"  - (default) U-slot through the +X short end, with a cable bay so the
           4-pin plug can turn, and stop nubs so the board can't slide into it.
  "side" - U-slot through a long wall at cable_x.

Coordinate system: X = board length, Y = board width, Z = up (lens faces +Z).
Origin is the centre of the board footprint, Z=0 is the outside of the base floor.

Usage:
    python3 elp_16mp_case.py [--out DIR] [--log-level DEBUG]
Requires: cadquery (pip install cadquery)
"""
import argparse
import json
import logging
import math
import os
from dataclasses import dataclass, asdict

import cadquery as cq

log = logging.getLogger("elp_16mp_case")


@dataclass
class Params:
    # --- Board (published) ---
    pcb_l: float = 62.3          # board length
    pcb_w: float = 9.0           # board width
    stack_h: float = 5.5         # total module thickness incl. lens

    # --- Board (ASSUMED - verify with calipers) ---
    pcb_t: float = 1.0           # bare PCB thickness
    lens_x: float = 0.0          # lens centre offset from board centre along X
    lens_y: float = 0.0          # lens centre offset from board centre along Y

    # --- Optics (HFOV published, rest assumed) ---
    hfov_deg: float = 97.0       # published horizontal FOV
    aspect_w: float = 4.0        # sensor aspect 4656x3496 is 4:3
    aspect_h: float = 3.0
    aperture_d: float = 3.0      # ASSUMED clear diameter of the lens front element
    window_margin: float = 0.6   # extra radius so the lid edge never vignettes

    # --- Fit ---
    fit: float = 0.3             # clearance per side around the PCB
    back_gap: float = 2.5        # space under PCB for solder joints / back-side parts / wires
    front_gap: float = 0.4       # air between lens top and lid underside

    # --- Shell ---
    wall: float = 1.8            # long side wall thickness
    floor: float = 1.6           # base floor thickness
    lid_t: float = 1.6           # lid plate thickness
    rail_w: float = 0.8          # width of the ledge the PCB edges sit on
    corner_r: float = 2.0        # outer vertical corner radius
    ear_l: float = 7.5           # solid block at each end (holds the snap pocket or screw)

    # --- Closure ---
    closure: str = "snap"        # "snap" or "screw"
    # snap-hook options
    hook_t: float = 0.9          # arm thickness (flex direction, X)
    hook_w: float = 4.0          # arm width (Y)
    hook_len: float = 7.0        # arm length below the lid underside
    barb_h: float = 0.7          # barb engagement depth
    barb_catch: float = 1.0      # height of the flat catch face region of the barb
    barb_ramp: float = 1.0       # height of the 45-ish lead-in ramp under the barb
    hook_clear: float = 0.2      # clearance around the arm in its pocket
    end_wall: float = 1.5        # plastic between pocket and the outer end face (the window wall)
    hook_y_cable_end: float = 3.3  # hook Y on the cable end (off-centre to clear the slot and nozzle)
    max_strain: float = 0.02     # allowed flex strain (PETG/ASA ~2%)
    # screw options (M2 self-tapping or M2 machine screw into plastic)
    screw_pilot_d: float = 1.7   # pilot in base (use 3.2 for M2 heat-set inserts)
    screw_clear_d: float = 2.4   # clearance in lid
    screw_head_d: float = 4.0    # countersink diameter for M2 90-deg flat head (ISO 7046 head ~3.8)
    screw_depth: float = 7.0     # pilot depth (M2 x 6 or x 8 flat head)
    screw_y_end: float = 3.4     # Y position of the screw on the cable end

    # --- Cable exit ---
    cable_exit: str = "end"      # "end" (+X short side) or "side" (long wall)
    cable_w: float = 3.6         # slot width (fits a 4-wire bundle; widen for a flat cable)
    cable_bay_l: float = 4.0     # free space past the board end for the plug and cable bend
    cable_y: float = -2.4        # end-slot centre across the end wall
    # Heat-shrink nozzle on the cable end (end exit only). Part of the base only:
    # a ridged C-channel whose bottom sits on the print bed, so no supports are
    # needed and the lid stays flat. The cable lies in the channel's round
    # bottom; heat shrink over channel + cable closes it and grips the ridges.
    nozzle: bool = True
    nozzle_r: float = 3.4        # outer radius at the ridge peaks
    nozzle_valley_r: float = 2.7 # outer radius in the valleys
    nozzle_collar_l: float = 1.0 # plain full-radius section at the root
    nozzle_ridges: int = 3       # number of barbs
    ridge_pitch: float = 3.0     # barb spacing
    nozzle_flat: float = 0.4     # flattening of the bottom so it sits on the bed
    stop_l: float = 1.0          # length of the board stop nubs at the bay boundary
    stop_w: float = 1.0          # width of each nub (PCB edge zone only)
    cable_x: float = 26.0        # side-slot centre along X
    cable_side: int = 1          # side-slot wall: +1 = +Y, -1 = -Y

    # --- PCB hold-down ribs on the lid (press on the PCB long edges) ---
    rib_xs: tuple = (-20.0, 20.0)  # rib centres along X; move if they foul a component
    rib_l: float = 5.0
    rib_w: float = 0.7
    rib_preload: float = 0.1     # ribs stop this far short of the PCB (print tolerance)

    # --- Snap test coupon ---
    # The hooks flex across FDM layer lines (lid prints face down, arms stand
    # vertical), so the bulk-material strain check is optimistic. Print this
    # slice of the -X end (base + lid, hook at y=0) to test the click and the
    # release before committing to a full case.
    coupon_l: float = 12.0       # length of the slice from the -X outer face

    # --- Size mockup ---
    # A solid block with the assembled case's outside shape (nozzle and lens
    # window included) to hold up inside the printer and find a mount spot
    # that clears the cover, gantry and toolhead. Lens face up, prints with no
    # supports.
    mock_mark_d: float = 0.4     # depth of the engraved lens crosshair
    mock_mark_w: float = 0.6     # width of the crosshair lines
    mock_mark_lx: float = 24.0   # crosshair arm length along X (Y arm is derived from out_w)
    mock_mark_edge: float = 1.0  # plain margin left between the Y arm and the long faces

    # --- Derived geometry ---
    @property
    def bay(self): return self.cable_bay_l if self.cable_exit == "end" else 0.0

    @property
    def cav_l(self): return self.pcb_l + 2 * self.fit

    @property
    def cav_x0(self): return -self.cav_l / 2

    @property
    def cav_x1(self): return self.cav_l / 2 + self.bay

    @property
    def cav_w(self): return self.pcb_w + 2 * self.fit

    @property
    def out_x0(self): return self.cav_x0 - self.ear_l

    @property
    def out_x1(self): return self.cav_x1 + self.ear_l

    @property
    def out_l(self): return self.out_x1 - self.out_x0

    @property
    def out_w(self): return self.cav_w + 2 * self.wall

    @property
    def z_pcb_bottom(self): return self.floor + self.back_gap

    @property
    def z_pcb_top(self): return self.z_pcb_bottom + self.pcb_t

    @property
    def z_lens_top(self): return self.z_pcb_top + (self.stack_h - self.pcb_t)

    @property
    def base_h(self): return self.z_lens_top + self.front_gap

    @property
    def use_nozzle(self): return self.nozzle and self.cable_exit == "end"

    @property
    def cable_z(self):
        """Cable axis height through the end wall (nozzle axis), else None."""
        return self.nozzle_r - self.nozzle_flat if self.use_nozzle else None

    @property
    def nozzle_l(self): return self.nozzle_collar_l + self.nozzle_ridges * self.ridge_pitch + 0.5

    @property
    def screw_pts(self):
        """Absolute (x, y) of both lid screws."""
        pos_y = self.screw_y_end if self.cable_exit == "end" else 0.0
        return [(self.out_x0 + self.ear_l / 2, 0.0), (self.out_x1 - self.ear_l / 2, pos_y)]

    def hooks(self):
        """One dict per hook: direction (+1 = +X end), Y centre, and key X/Z planes."""
        out = []
        for d, x_face in ((-1, self.out_x0), (1, self.out_x1)):
            y = self.hook_y_cable_end if (d > 0 and self.cable_exit == "end") else 0.0
            x_win_in = x_face - d * self.end_wall           # inner face of the window wall
            x_arm_out = x_win_in - d * self.hook_clear      # arm outer face (barb side)
            x_arm_in = x_arm_out - d * self.hook_t          # arm inner face
            x_pocket_in = x_arm_in - d * (self.barb_h + self.hook_clear)  # room to flex
            z_tip = self.base_h - self.hook_len
            out.append(dict(d=d, y=y, x_face=x_face, x_win_in=x_win_in, x_arm_out=x_arm_out,
                            x_arm_in=x_arm_in, x_pocket_in=x_pocket_in, z_tip=z_tip,
                            z_catch=z_tip + self.barb_ramp + self.barb_catch))
        return out

    # --- Derived optics ---
    @property
    def tan_half_dfov(self):
        """tan(half diagonal FOV) for a rectilinear lens, derived from HFOV and aspect."""
        tan_h = math.tan(math.radians(self.hfov_deg / 2))
        return tan_h * math.hypot(self.aspect_w, self.aspect_h) / self.aspect_w

    @property
    def vfov_deg(self):
        """Vertical FOV from HFOV and aspect, rectilinear lens."""
        tan_h = math.tan(math.radians(self.hfov_deg / 2))
        return 2 * math.degrees(math.atan(tan_h * self.aspect_h / self.aspect_w))

    def window_d_at(self, height_above_lens: float) -> float:
        """Window diameter needed at a given height above the lens front."""
        return 2 * (self.aperture_d / 2 + height_above_lens * self.tan_half_dfov + self.window_margin)


def validate(p: Params) -> None:
    """Fail early on parameter combinations that cannot produce a usable part."""
    if p.cable_exit not in ("end", "side"):
        raise ValueError("cable_exit must be 'end' or 'side'")
    if p.closure not in ("snap", "screw"):
        raise ValueError("closure must be 'snap' or 'screw'")
    if p.rail_w * 2 >= p.cav_w:
        raise ValueError("rail_w too large: rails would close the cavity")
    min_web = 0.8

    if p.cable_exit == "side":
        if abs(p.cable_x) + p.cable_w / 2 > p.cav_l / 2:
            raise ValueError("cable slot falls outside the cavity; reduce |cable_x|")
        if p.cable_side not in (1, -1):
            raise ValueError("cable_side must be +1 or -1")
    elif abs(p.cable_y) + p.cable_w / 2 > p.cav_w / 2:
        raise ValueError("end cable slot is wider than the cavity at cable_y")

    if p.use_nozzle:
        if p.nozzle_valley_r - p.cable_w / 2 < 0.8:
            raise ValueError("nozzle wall too thin in the valleys; raise nozzle_valley_r or narrow cable_w")
        if p.cable_z - p.cable_w / 2 < 0.8:
            raise ValueError("nozzle bore too close to the bed; reduce nozzle_flat")
        if abs(p.cable_y) + p.nozzle_r > p.out_w / 2:
            raise ValueError("nozzle sticks out past the long side of the case")
        if p.closure == "snap":
            h = p.hooks()[1]
            win_lo = h["y"] - p.hook_w / 2 - p.hook_clear
            gap = win_lo - (p.cable_y + p.nozzle_r)
            log.debug("Nozzle to snap release window gap %.2f mm", gap)
            if gap < 0:
                raise ValueError("nozzle covers the cable-end snap release window")

    if p.closure == "screw":
        if p.screw_depth >= p.base_h - 0.8:
            raise ValueError("screw_depth would break through the base floor")
        if p.cable_exit == "end":
            sy = p.screw_pts[1][1]
            gap = abs(sy - p.cable_y) - p.cable_w / 2 - p.screw_pilot_d / 2
            if gap < min_web:
                raise ValueError(f"end screw is only {gap:.2f} mm from the cable slot")
    else:
        # Cantilever strain at full deflection: eps = 1.5 * t * delta / L^2
        strain = 1.5 * p.hook_t * p.barb_h / p.hook_len ** 2
        log.debug("Snap hook flex strain %.2f%% (limit %.2f%%)", strain * 100, p.max_strain * 100)
        if strain > p.max_strain:
            raise ValueError(f"hook strain {strain:.3f} > {p.max_strain}; lengthen or thin the arm")
        if p.hook_len > p.base_h - p.floor - 0.4:
            raise ValueError("hook_len would hit the base floor")
        for h in p.hooks():
            pocket_depth = abs(h["x_win_in"] - h["x_pocket_in"])
            if pocket_depth > p.ear_l - min_web:
                raise ValueError("snap pocket does not fit in the end block; increase ear_l")
            half = p.hook_w / 2 + p.hook_clear
            if abs(h["y"]) + half > p.out_w / 2 - 1.0:
                raise ValueError("snap pocket too close to the long outer wall")
            if h["d"] > 0 and p.cable_exit == "end":
                gap = abs(h["y"] - p.cable_y) - half - p.cable_w / 2
                if gap < min_web:
                    raise ValueError(f"cable-end snap pocket is only {gap:.2f} mm from the cable slot")
                log.debug("Cable-end hook web to slot %.2f mm", gap)

    if p.closure == "snap":
        h = p.hooks()[0]
        # The coupon must contain the whole pocket plus a solid wall stub past it
        if p.out_x0 + p.coupon_l < h["x_pocket_in"] + min_web + 2.0:
            raise ValueError("coupon_l too short to contain the -X snap pocket and a wall stub")

    outer_d = p.window_d_at(p.front_gap + p.lid_t)
    if p.mock_mark_lx / 2 + abs(p.lens_x) > p.out_l / 2 - min_web:
        raise ValueError("mockup crosshair X arm runs off the end of the case")
    if p.mock_mark_lx <= outer_d + 4.0:
        raise ValueError("mockup crosshair X arm would be hidden inside the lens window")
    if outer_d / 2 + abs(p.lens_y) > p.out_w / 2 - min_web:
        raise ValueError(f"lens window ({outer_d:.2f} mm) too wide for a {p.out_w:.1f} mm lid")
    for x in p.rib_xs:
        if abs(x) + p.rib_l / 2 > p.cav_l / 2:
            raise ValueError(f"hold-down rib at x={x} is outside the board pocket")
        if abs(x - p.lens_x) < p.rib_l / 2 + outer_d / 2:
            log.warning("Rib at x=%.1f sits close to the lens; check it is out of frame", x)
    log.debug("Diagonal FOV %.1f deg, window d=%.2f (inner) -> %.2f (outer)",
              2 * math.degrees(math.atan(p.tan_half_dfov)), p.window_d_at(p.front_gap), outer_d)
    log.debug("Parameter validation passed")


def _box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    """Axis-aligned box from absolute bounds (any corner order)."""
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return (cq.Workplane("XY").workplane(offset=z0).center((x0 + x1) / 2, (y0 + y1) / 2)
            .box(x1 - x0, y1 - y0, z1 - z0, centered=(True, True, False)))


def build_nozzle(p: Params) -> cq.Workplane:
    """
    Solid ridged nozzle (bore and slot are cut later together with the end block).

    Profile, moving outward along +X: full-radius collar, then barbs that ramp
    down toward the tip and step back up vertically. The vertical faces point
    at the case, so recovered heat shrink sitting in a valley has to climb a
    sheer step to slide off: same idea as a hose barb.
    """
    r, rv, c, n, pitch = p.nozzle_r, p.nozzle_valley_r, p.nozzle_collar_l, p.nozzle_ridges, p.ridge_pitch
    pts = [(0.0, 0.0), (0.0, r), (c, r)]
    for i in range(1, n + 1):
        x = c + i * pitch
        pts.append((x, rv))
        if i < n:
            pts.append((x, r))          # sheer catch face
    pts += [(p.nozzle_l, rv), (p.nozzle_l, 0.0)]
    log.debug("Nozzle profile: %s", pts)

    body = (cq.Workplane("XY").polyline(pts).close()
            .revolve(360, (0, 0, 0), (1, 0, 0))
            .translate((p.out_x1 - 0.2, p.cable_y, p.cable_z)))  # 0.2 overlap fuses it to the end face
    # Flatten the bottom so it sits on the bed (no supports)
    return body.intersect(_box(p.out_x1 - 1, p.out_x1 + p.nozzle_l + 1, -50, 50, 0, 50))


def build_base(p: Params) -> cq.Workplane:
    """Tray: PCB drops in lens-up and rests on two side rails."""
    log.info("Building base: %.1f x %.1f x %.1f mm, cable=%s, closure=%s",
             p.out_l, p.out_w, p.base_h, p.cable_exit, p.closure)
    hw, cw = p.out_w / 2, p.cav_w / 2

    base = _box(p.out_x0, p.out_x1, -hw, hw, 0, p.base_h).edges("|Z").fillet(p.corner_r)
    base = base.cut(_box(p.cav_x0, p.cav_x1, -cw, cw, p.floor, p.base_h + 1))

    for side in (1, -1):
        base = base.union(_box(p.cav_x0, p.cav_l / 2, side * cw, side * (cw - p.rail_w),
                               p.floor, p.z_pcb_bottom))
    log.debug("PCB seat z=%.2f, PCB top z=%.2f, lens top z=%.2f",
              p.z_pcb_bottom, p.z_pcb_top, p.z_lens_top)

    if p.cable_exit == "end":
        x0 = p.cav_l / 2
        for side in (1, -1):
            base = base.union(_box(x0, x0 + p.stop_l, side * cw, side * (cw - p.stop_w),
                                   p.floor, p.z_pcb_top + 1.0))
        if p.use_nozzle:
            base = base.union(build_nozzle(p))
            x_end = p.out_x1 + p.nozzle_l + 0.1
            r_b = p.cable_w / 2
            # Round-bottomed bore along the cable axis, through end block and nozzle
            bore = cq.Solid.makeCylinder(r_b, x_end - (p.cav_x1 - 0.1),
                                         pnt=cq.Vector(p.cav_x1 - 0.1, p.cable_y, p.cable_z),
                                         dir=cq.Vector(1, 0, 0))
            base = base.cut(cq.Workplane().add(bore))
            # Open-top slot above the bore so the cable lays in from above
            base = base.cut(_box(p.cav_x1 - 0.1, x_end, p.cable_y - r_b, p.cable_y + r_b,
                                 p.cable_z, p.base_h + 1))
            log.debug("Nozzle: axis z=%.2f, peak r=%.2f, valley r=%.2f, length %.1f",
                      p.cable_z, p.nozzle_r, p.nozzle_valley_r, p.nozzle_l)
        else:
            base = base.cut(_box(p.cav_x1 - 0.1, p.out_x1 + 0.1,
                                 p.cable_y - p.cable_w / 2, p.cable_y + p.cable_w / 2,
                                 p.floor, p.base_h + 1))
        log.debug("End slot y=%.2f w=%.2f, bay %.1f mm", p.cable_y, p.cable_w, p.bay)
    else:
        y_in = p.cable_side * (cw - p.rail_w - 0.1)
        y_out = p.cable_side * (hw + 0.1)
        base = base.cut(_box(p.cable_x - p.cable_w / 2, p.cable_x + p.cable_w / 2,
                             y_in, y_out, p.floor, p.base_h + 1))
        log.debug("Side slot x=%.1f on %s wall", p.cable_x, "+Y" if p.cable_side > 0 else "-Y")

    if p.closure == "screw":
        base = (base.faces(">Z").workplane(centerOption="ProjectedOrigin")
                .pushPoints(p.screw_pts).hole(p.screw_pilot_d, p.screw_depth))
        log.debug("Screw pilots at %s", p.screw_pts)
    else:
        for h in p.hooks():
            half = p.hook_w / 2 + p.hook_clear
            # Vertical pocket, open at the top, deep enough for the arm tip plus clearance
            base = base.cut(_box(h["x_win_in"], h["x_pocket_in"], h["y"] - half, h["y"] + half,
                                 h["z_tip"] - 0.4, p.base_h + 1))
            # Window through the end face: barb sits here; its top edge is the catch
            base = base.cut(_box(h["x_face"] + h["d"] * 0.1, h["x_win_in"] - h["d"] * 0.1,
                                 h["y"] - half, h["y"] + half,
                                 h["z_tip"] - 0.4, h["z_catch"] + 0.1))
            log.debug("Snap pocket %s end: y=%.2f, catch z=%.2f", "+X" if h["d"] > 0 else "-X",
                      h["y"], h["z_catch"])
    return base


def _hook(p: Params, h: dict) -> cq.Workplane:
    """Cantilever arm plus barb, modelled in assembled position."""
    d, y, hw = h["d"], h["y"], p.hook_w / 2
    arm = _box(h["x_arm_in"], h["x_arm_out"], y - hw, y + hw, h["z_tip"], p.base_h)
    # Barb profile in the XZ plane: flat catch face on top, ramp underneath toward the tip
    xa, xb = h["x_arm_out"], h["x_arm_out"] + d * p.barb_h
    z0, z1, z2 = h["z_tip"], h["z_tip"] + p.barb_ramp, h["z_catch"]
    barb = (cq.Workplane("XZ", origin=(0, y + hw, 0))
            .polyline([(xa, z0), (xb, z1), (xb, z2), (xa, z2)]).close()
            .extrude(p.hook_w))
    return arm.union(barb)


def build_lid(p: Params) -> cq.Workplane:
    """Flat lid with FOV-sized conical lens window, hold-down ribs and closure features."""
    z0, hw = p.base_h, p.out_w / 2
    d_in, d_out = p.window_d_at(p.front_gap), p.window_d_at(p.front_gap + p.lid_t)
    log.info("Building lid: %.1f x %.1f x %.1f mm, cone window %.2f -> %.2f mm",
             p.out_l, p.out_w, p.lid_t, d_in, d_out)

    lid = _box(p.out_x0, p.out_x1, -hw, hw, z0, z0 + p.lid_t).edges("|Z").fillet(p.corner_r)

    eps = 0.05
    slope = (d_out - d_in) / 2 / p.lid_t
    cone = cq.Solid.makeCone(d_in / 2 - slope * eps, d_out / 2 + slope * eps, p.lid_t + 2 * eps,
                             pnt=cq.Vector(p.lens_x, p.lens_y, z0 - eps), dir=cq.Vector(0, 0, 1))
    lid = lid.cut(cq.Workplane().add(cone))

    if p.closure == "screw":
        # Countersunk so flat-head screws sit flush with the lid top
        lid = (lid.faces(">Z").workplane(centerOption="ProjectedOrigin")
               .pushPoints(p.screw_pts).cskHole(p.screw_clear_d, p.screw_head_d, 90))
    else:
        for h in p.hooks():
            lid = lid.union(_hook(p, h))
        log.debug("Added %d snap hooks", len(p.hooks()))

    rib_h = z0 - p.z_pcb_top - p.rib_preload
    key_gap = 0.15
    for x in p.rib_xs:
        for side in (1, -1):
            lid = lid.union(_box(x - p.rib_l / 2, x + p.rib_l / 2,
                                 side * (p.cav_w / 2 - key_gap), side * (p.cav_w / 2 - key_gap - p.rib_w),
                                 z0 - rib_h, z0))
    log.debug("Rib height %.2f mm, %d ribs", rib_h, 2 * len(p.rib_xs))
    return lid


def coupon_box(p: Params) -> cq.Workplane:
    """Bounding slab for the snap test coupon: the -X end of the case, coupon_l long."""
    return _box(p.out_x0 - 1, p.out_x0 + p.coupon_l, -50, 50, -1, 50)


def build_coupon(p: Params, base: cq.Workplane, lid: cq.Workplane) -> tuple:
    """Slice the -X end off base and lid (assembled position) for a print test of the snap."""
    box = coupon_box(p)
    return base.intersect(box), lid.intersect(box)


def build_size_mockup(p: Params) -> cq.Workplane:
    """
    Solid stand-in for the assembled case: base plus lid envelope, nozzle with
    cable channel, the lid's FOV cone, and an engraved crosshair on the lens
    axis. Its outside shape matches the real assembly (checked in self_check).
    """
    hw, z_top = p.out_w / 2, p.base_h + p.lid_t
    log.info("Building size mockup: %.1f x %.1f x %.1f mm", p.out_l, p.out_w, z_top)
    body = _box(p.out_x0, p.out_x1, -hw, hw, 0, z_top).edges("|Z").fillet(p.corner_r)
    if p.use_nozzle:
        body = body.union(build_nozzle(p))
        x_end = p.out_x1 + p.nozzle_l + 0.1
        r_b = p.cable_w / 2
        bore = cq.Solid.makeCylinder(r_b, x_end - (p.out_x1 - 0.2),
                                     pnt=cq.Vector(p.out_x1 - 0.2, p.cable_y, p.cable_z),
                                     dir=cq.Vector(1, 0, 0))
        body = body.cut(cq.Workplane().add(bore))
        body = body.cut(_box(p.out_x1 - 0.2, x_end, p.cable_y - r_b, p.cable_y + r_b,
                             p.cable_z, p.base_h + 1))
    # Lens window cone, same as the lid, so the opening reads true
    d_in, d_out = p.window_d_at(p.front_gap), p.window_d_at(p.front_gap + p.lid_t)
    slope = (d_out - d_in) / 2 / p.lid_t
    eps = 0.05
    cone = cq.Solid.makeCone(d_in / 2 - slope * eps, d_out / 2 + slope * eps, p.lid_t + 2 * eps,
                             pnt=cq.Vector(p.lens_x, p.lens_y, p.base_h - eps), dir=cq.Vector(0, 0, 1))
    body = body.cut(cq.Workplane().add(cone))
    # Crosshair on the lens axis, engraved into the top face around the cone
    half_lx, half_w = p.mock_mark_lx / 2, p.mock_mark_w / 2
    half_ly = hw - p.mock_mark_edge
    z0 = z_top - p.mock_mark_d
    for (dx, dy) in ((half_lx, half_w), (half_w, half_ly)):
        body = body.cut(_box(p.lens_x - dx, p.lens_x + dx, p.lens_y - dy, p.lens_y + dy, z0, z_top + 1))
    log.debug("Mockup crosshair %.1f x %.1f mm, %.1f deep at lens (%.1f, %.1f)",
              2 * half_lx, 2 * half_ly, p.mock_mark_d, p.lens_x, p.lens_y)
    return body


def mock_module(p: Params) -> cq.Workplane:
    """Rough stand-in for the camera: PCB plus an assumed 8 mm square AF lens block."""
    pcb = _box(-p.pcb_l / 2, p.pcb_l / 2, -p.pcb_w / 2, p.pcb_w / 2, p.z_pcb_bottom, p.z_pcb_top)
    lens = _box(p.lens_x - 4, p.lens_x + 4, p.lens_y - 4, p.lens_y + 4, p.z_pcb_top, p.z_lens_top)
    return pcb.union(lens)


def self_check(p: Params, base: cq.Workplane, lid: cq.Workplane) -> None:
    """Interference and validity checks against a mock module."""
    module = mock_module(p)
    checks = {"base vs lid": base.intersect(lid),
              "base vs module": base.intersect(module),
              "lid vs module": lid.intersect(module)}
    for name, solid in checks.items():
        vol = solid.val().Volume() if solid.vals() else 0.0
        log.debug("Interference %s: %.4f mm^3", name, vol)
        if vol > 1e-6:
            raise RuntimeError(f"Interference detected ({name}): {vol:.3f} mm^3")
    for name, part in (("base", base), ("lid", lid)):
        if len(part.solids().vals()) != 1 or not part.val().isValid():
            raise RuntimeError(f"{name} is not a single valid solid")
    # Size mockup must match the assembled envelope it stands in for. Base and
    # lid are built by separate code paths, so this catches drift between them
    # and the mockup; it cannot catch a wrong Params value shared by all three.
    mockup = build_size_mockup(p)
    if len(mockup.solids().vals()) != 1 or not mockup.val().isValid():
        raise RuntimeError("size mockup is not a single valid solid")
    bb_asm = base.union(lid).val().BoundingBox()
    bb_mock = mockup.val().BoundingBox()
    for axis in ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax"):
        delta = abs(getattr(bb_asm, axis) - getattr(bb_mock, axis))
        if delta > 0.01:
            raise RuntimeError(f"size mockup {axis} differs from the assembly by {delta:.3f} mm")
    log.debug("Size mockup envelope matches assembly: %.2f x %.2f x %.2f mm",
              bb_mock.xlen, bb_mock.ylen, bb_mock.zlen)
    if p.closure == "snap":
        c_base, c_lid = build_coupon(p, base, lid)
        for name, part in (("coupon base", c_base), ("coupon lid", c_lid)):
            if len(part.solids().vals()) != 1 or not part.val().isValid():
                raise RuntimeError(f"{name} is not a single valid solid")
        # The whole -X hook must be inside the coupon, or it tests a truncated arm
        hook = _hook(p, p.hooks()[0])
        kept = hook.intersect(coupon_box(p)).val().Volume()
        if abs(kept - hook.val().Volume()) > 1e-6:
            raise RuntimeError("snap coupon truncates the -X hook")
        log.debug("Coupon: base %.1f mm^3, lid %.1f mm^3", c_base.val().Volume(), c_lid.val().Volume())
    log.info("Self-check passed: single valid solids, no interference with mock module")


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
    # Lid flipped for printing: outer face on the bed, ribs and hooks up, no supports
    def flip_lid(part: cq.Workplane) -> cq.Workplane:
        """Lid parts go outer face down on the bed: hooks and ribs up, no supports."""
        return part.translate((0, 0, -(p.base_h + p.lid_t))).rotate((0, 0, 0), (1, 0, 0), 180)

    lid_print = flip_lid(lid)
    exports = {
        "elp_16mp_base.stl": base,
        "elp_16mp_lid.stl": lid_print,
        "view_lid.stl": lid,
        "view_module.stl": mock_module(p),
        "elp_16mp_case.step": cq.Assembly().add(base, name="base").add(lid, name="lid").toCompound(),
    }
    exports["elp_16mp_size_mockup.stl"] = build_size_mockup(p)
    if p.closure == "snap":
        c_base, c_lid = build_coupon(p, base, lid)
        exports["elp_16mp_coupon_base.stl"] = c_base
        exports["elp_16mp_coupon_lid.stl"] = flip_lid(c_lid)
    for name, shape in exports.items():
        path = os.path.join(args.out, name)
        if name.endswith(".stl"):
            cq.exporters.export(shape, path, tolerance=0.02, angularTolerance=0.1)
        else:
            cq.exporters.export(shape, path)
        log.info("Wrote %s (%d bytes)", path, os.path.getsize(path))
    geom = {"lens_x": p.lens_x, "lens_y": p.lens_y, "lens_top_z": p.z_lens_top,
            "hfov_deg": p.hfov_deg, "vfov_deg": p.vfov_deg,
            "dfov_deg": 2 * math.degrees(math.atan(p.tan_half_dfov))}
    with open(os.path.join(args.out, "viewer_params.json"), "w") as f:
        json.dump(geom, f, indent=1)
    log.debug("Viewer params: %s", geom)
    log.info("Assembled outer size: %.1f x %.1f x %.1f mm", p.out_l, p.out_w, p.base_h + p.lid_t)


if __name__ == "__main__":
    main()
