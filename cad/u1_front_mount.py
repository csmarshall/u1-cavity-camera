#!/usr/bin/env python3
"""
Tilt mount for the ELP-USB16MP02-AF100 case on the Snapmaker U1's front
vertical feature (just inside the door, roughly 25-50 mm tall, flat and as
wide as the printer).

Parts:
  wall plate  - flat back for VHB tape, plus two countersunk M3 holes for
                screws if you drill the feature later. One centre hinge knuckle.
  cradle      - U-channel the assembled case snaps into (lips over the lid's
                long top edges, away from the lens). Two hinge knuckles on its
                lower long edge. Leaves both case end faces free (snap release
                windows, cable nozzle).
  hinge       - M3 socket head through the three knuckles, nyloc nut on the
                end. Nut torque sets the friction.
  wedge       - one-piece alternative: plate + fixed-angle wedge + cradle,
                once the angle is known (read it with a phone inclinometer on
                the lid while using the hinge).

World frame (mount): X along the wall (horizontal, = case X), Y out of the
wall into the printer, Z up. Wall face at Y=0. Tilt is the depression of the
optical axis below horizontal, assuming the feature is vertical: 0 = the lens
looks straight into the printer, positive = it looks down.

Case frame (from elp_16mp_case): X board length, Y board width, Z up through
the lens, Z=0 the outside of the base floor. Mounted, the case floor faces
the wall, case +Y points down, and the lens (case +Z) points along +Y.

Usage:
    python3 cad/u1_front_mount.py [--out out] [--tilt 45] [--wedge-angle 45] [--log-level DEBUG]
"""
import argparse
import json
import logging
import math
import os
from dataclasses import dataclass, asdict
from typing import List, Tuple

import cadquery as cq

import elp_16mp_case as case

log = logging.getLogger("u1_front_mount")


@dataclass
class MountParams:
    # --- Feature on the printer (MEASURE) ---
    feature_h_min: float = 25.4   # shortest the feature can be (1 in); the plate must fit inside it

    # --- Wall plate ---
    plate_l: float = 70.0         # along X
    plate_h: float = 22.0         # along Z; must be <= feature_h_min
    plate_t: float = 3.0          # along Y
    plate_r: float = 3.0          # corner radius
    screw_clear_d: float = 3.4    # M3 clearance
    screw_csk_d: float = 6.5      # M3 90-degree countersink (ISO 10642 head ~6.0)
    screw_inset: float = 7.0      # hole centre from the plate's X ends

    # --- Orientation ---
    # "landscape": case long axis along the wall, the 97 deg image axis horizontal.
    # "portrait":  case stood on its -X end (cable nozzle up), the 97 deg axis
    #              vertical; the stream is 16:9, so this gives a 9:16 frame.
    orientation: str = "landscape"
    # "above": camera sits above the hinge (hinge near the plate's bottom edge).
    # "below": the same parts built for the plate mounted upside down, camera
    #          hanging below the feature. Looking down then swings the camera's
    #          lower end toward the wall, so the plate knuckle stands off on an
    #          arm whose length is derived from the tilt range (never hand-set).
    mount_side: str = "above"

    # --- Hinge ---
    knuckle_r: float = 4.0        # outer radius of every knuckle
    pin_d: float = 3.4            # M3 clearance through all knuckles
    plate_knuckle_w: float = 8.0  # centre knuckle on the plate (X)
    cradle_knuckle_w: float = 6.0 # each of the two cradle knuckles (X)
    knuckle_gap: float = 0.3      # X gap between neighbouring knuckles
    swing_gap: float = 0.4        # radial clearance: cradle body to plate knuckle, cradle knuckles to plate face
    # "below" uses an end hinge: one plate arm at each end, outside the whole
    # swept length of the case and nozzle, each with its own M3 pin, so the
    # camera never swings through an arm.
    arm_w: float = 6.0            # plate arm thickness along X (end hinge)
    end_gap: float = 0.6          # X clearance between the case/nozzle and each arm
    hinge_z: float = 5.0          # hinge axis height above the plate's bottom edge
    tilt_max: float = 75.0        # swept range checked for collisions
    tilt_step: float = 5.0

    # --- Cradle ---
    fit: float = 0.2              # clearance around the case (Y sides and floor)
    floor_t: float = 1.6
    wall_t: float = 1.6
    lip_in: float = 0.8           # how far each lip reaches over the lid edge
    lip_t: float = 1.0            # lip thickness (Z)
    lip_gap: float = 0.15         # air between lid top and lip underside
    lip_l: float = 10.0           # length of each lip segment (X)
    lip_x: Tuple[float, ...] = (-27.0, 27.0)   # lip segment centres; away from the lens
    end_margin: float = 2.0       # cradle stops this far inside each case end face
    max_strain: float = 0.02      # wall flex strain allowed when the case snaps in

    # --- Wedge ---
    wedge_max_print: float = 50.0 # above this the tilted cradle walls overhang too far to print support-free
    # Support-free check: downward faces steeper than 45 deg above the first layer.
    # Allowed area covers the cradle lips' flat catch faces (4 lips x lip_l x
    # (lip_in + fit) = 40 mm^2 at defaults), which are short (1 mm) overhangs.
    max_overhang_area: float = 45.0   # mm^2 per part



def _box(x0, x1, y0, y1, z0, z1) -> cq.Workplane:
    return case._box(x0, x1, y0, y1, z0, z1)


def _cyl_x(r: float, x0: float, x1: float, y: float, z: float) -> cq.Workplane:
    """Cylinder along X."""
    return cq.Workplane().add(cq.Solid.makeCylinder(r, x1 - x0, pnt=cq.Vector(x0, y, z), dir=cq.Vector(1, 0, 0)))


def _prism_x(pts_yz, x0: float, x1: float) -> cq.Workplane:
    """Prism along X from a polygon given in (y, z)."""
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts_yz).close().extrude(x1 - x0)


def _teardrop_x(r: float, x0: float, x1: float, y: float, z: float, up: str) -> cq.Workplane:
    """
    Horizontal hole along X with a 45-degree peak pointing at the print's up
    direction (`up` = "z" or "y" in this part's frame), so its crown prints
    without bridging.
    """
    k = r / math.sqrt(2)
    if up == "z":
        peak = [(y - k, z + k), (y + k, z + k), (y, z + r * math.sqrt(2))]
    else:
        peak = [(y + k, z - k), (y + k, z + k), (y + r * math.sqrt(2), z)]
    return _cyl_x(r, x0, x1, y, z).union(_prism_x(peak, x0, x1))


class Geometry:
    """All derived positions, shared by builders and checks so nothing drifts."""

    def __init__(self, cp: case.Params, mp: MountParams):
        self.cp, self.mp = cp, mp
        self.case_top = cp.base_h + cp.lid_t                  # assembled case height (case Z)
        self.hw = cp.out_w / 2
        self.wall_in = self.hw + mp.fit                       # cradle wall inner face (case |Y|)
        self.wall_out = self.wall_in + mp.wall_t
        self.floor_bot = -mp.fit - mp.floor_t                 # cradle floor outer face (case Z)
        self.attach = mp.knuckle_r + mp.swing_gap            # axis to the cradle body it hangs from
        # Hinge axis in case coordinates, knuckle tangent to the floor plane (the print bed).
        # Landscape: along case X beside the +Y wall. Portrait: along case Y past the -X end.
        self.axis_y = self.wall_out + self.attach
        self.axis_x = cp.out_x0 - self.attach
        self.axis_z = self.floor_bot + mp.knuckle_r
        self.cradle_x0 = cp.out_x0 + mp.end_margin
        self.cradle_x1 = cp.out_x1 - mp.end_margin
        # Mount frame = world for "above"; for "below" the world is the mount frame
        # turned 180 deg about the wall normal, and the hinge turns the other way.
        if mp.orientation == "portrait" and mp.mount_side == "below":
            # Hanging 82 mm below the hinge, looking down swings the case's lower end
            # ~L*sin(tilt) toward the wall: the arm would need ~97 mm for 75 deg.
            raise ValueError("portrait + below is not supported: the hinge arm would have to stand "
                             "~100 mm off the wall. Use portrait above, or landscape below.")
        self.sign = 1.0 if mp.mount_side == "above" else -1.0
        self.axis_Z = mp.hinge_z
        # Hinge axis distance from the wall: knuckles clear the plate face by swing_gap,
        # and for "below" the whole swept camera must clear it too
        self.axis_Y = mp.plate_t + mp.knuckle_r + mp.swing_gap
        if mp.mount_side == "below":
            self.axis_Y = max(self.axis_Y, self._swept_standoff())
        log.debug("Hinge axis %.2f mm from the wall (%s, %s)", self.axis_Y, mp.orientation, mp.mount_side)
        sp = self.knuckle_spans()
        if mp.mount_side == "below":
            # Plate spans both end arms
            self.plate_x0, self.plate_x1 = sp["plate"][0][0], sp["plate"][1][1]
        else:
            self.plate_x0, self.plate_x1 = -mp.plate_l / 2, mp.plate_l / 2
        log.debug("Plate x %.1f..%.1f, knuckles %s", self.plate_x0, self.plate_x1, sp)

    def knuckle_spans(self) -> dict:
        """X (hinge-axis) ranges, left to right: plate knuckles and cradle knuckles."""
        mp, cp = self.mp, self.cp
        if self.mp.mount_side == "below":
            # End hinge: arms outside everything that swings (case, nozzle, cap)
            xa0 = cp.out_x0 - mp.end_gap
            xb1 = (cp.out_x1 + cp.nozzle_l if cp.use_nozzle else cp.out_x1) + mp.end_gap
            cw = mp.cradle_knuckle_w
            return {"plate": [(xa0 - mp.arm_w, xa0), (xb1, xb1 + mp.arm_w)],
                    "cradle": [(xa0 + mp.knuckle_gap, self.cradle_x0 + cw),
                               (self.cradle_x1 - cw, xb1 - mp.knuckle_gap)]}
        hp = mp.plate_knuckle_w / 2
        c0 = hp + mp.knuckle_gap
        return {"plate": [(-hp, hp)],
                "cradle": [(-c0 - mp.cradle_knuckle_w, -c0), (c0, c0 + mp.cradle_knuckle_w)]}

    def _swept_standoff(self) -> float:
        """Axis distance from the wall so the cradle and case clear the plate over the whole tilt range."""
        saved, self.axis_Y = getattr(self, "axis_Y", 0.0), 0.0
        parts = [build_cradle(self)] + [case.build_base(self.cp), case.build_lid(self.cp)]
        if self.cp.use_cap:
            parts.append(case.build_cable_cap(self.cp))
        min_y, t = 0.0, 0.0
        while t <= self.mp.tilt_max + 1e-9:
            for part in parts:
                min_y = min(min_y, self.pose(part, t).val().BoundingBox().ymin)
            t += self.mp.tilt_step
        self.axis_Y = saved
        # Bounding boxes of rotated solids are conservative, so this never under-sizes the arm
        return self.mp.plate_t + self.mp.swing_gap - min_y

    def screw_xs(self) -> Tuple[float, float]:
        """Plate screw X positions: near the ends, or just inboard of the end arms."""
        mp = self.mp
        if mp.mount_side == "below":
            sp = self.knuckle_spans()["plate"]
            return (sp[0][1] + mp.screw_inset, sp[1][0] - mp.screw_inset)
        return (self.plate_x0 + mp.screw_inset, self.plate_x1 - mp.screw_inset)

    def to_world(self, shape: cq.Workplane) -> cq.Workplane:
        """Mount frame -> world. "below" turns everything 180 deg about the wall normal at mid-plate."""
        if self.mp.mount_side == "above":
            return shape
        zc = self.mp.plate_h / 2
        return shape.rotate((0, 0, zc), (0, 1, zc), 180)

    def pose(self, shape: cq.Workplane, tilt_deg: float) -> cq.Workplane:
        """Case/cradle frame -> world at the given tilt (rotation about the hinge axis)."""
        if self.mp.orientation == "portrait":
            # Case +X -> world +Z (stood on its -X end), +Y -> world +X, +Z -> world +Y
            s = shape.rotate((0, 0, 0), (1, 1, 1), -120)
            # Axis point (x=axis_x, z=axis_z) now sits at (Y=axis_z, Z=axis_x)
            s = s.translate((0, self.axis_Y - self.axis_z, self.axis_Z - self.axis_x))
        else:
            # Case +Z -> world +Y (lens into the printer), case +Y -> world -Z (down)
            s = shape.rotate((0, 0, 0), (1, 0, 0), -90)
            # Axis point (y=axis_y, z=axis_z) now sits at (Y=axis_z, Z=-axis_y)
            s = s.translate((0, self.axis_Y - self.axis_z, self.axis_Z + self.axis_y))
        # Positive tilt dips the lens in the world. In the mount frame that is a
        # rotation about +X by -tilt ("above") or +tilt ("below", frame flipped later).
        return s.rotate((0, self.axis_Y, self.axis_Z), (1, self.axis_Y, self.axis_Z), -self.sign * tilt_deg)

    def world_dir(self, local: Tuple[float, float, float], tilt_deg: float) -> Tuple[float, float, float]:
        """World direction of a case-frame direction at this tilt, measured by posing two points."""
        p0 = self.to_world(self.pose(cq.Workplane().add(cq.Vertex.makeVertex(0, 0, 0)), tilt_deg)).val().toTuple()
        p1 = self.to_world(self.pose(cq.Workplane().add(cq.Vertex.makeVertex(*local)), tilt_deg)).val().toTuple()
        return tuple(b - a for a, b in zip(p0, p1))

    def lens_dir(self, tilt_deg: float) -> Tuple[float, float, float]:
        """World direction of the optical axis (case +Z)."""
        return self.world_dir((0, 0, 1), tilt_deg)


def build_plate(g: Geometry, with_knuckle: bool = True) -> cq.Workplane:
    """Wall plate in world coordinates: back face on Y=0."""
    mp = g.mp
    plate = (_box(g.plate_x0, g.plate_x1, 0, mp.plate_t, 0, mp.plate_h)
             .edges("|Y").fillet(mp.plate_r))
    for x in g.screw_xs():
        # Countersink opens on the front face (Y=plate_t) so the screw head sits flush
        csk = cq.Solid.makeCone(mp.screw_csk_d / 2, 0, mp.screw_csk_d / 2,
                                pnt=cq.Vector(x, mp.plate_t + 0.01, mp.plate_h / 2), dir=cq.Vector(0, -1, 0))
        hole = cq.Solid.makeCylinder(mp.screw_clear_d / 2, mp.plate_t + 1,
                                     pnt=cq.Vector(x, -0.5, mp.plate_h / 2), dir=cq.Vector(0, 1, 0))
        plate = plate.cut(cq.Workplane().add(csk)).cut(cq.Workplane().add(hole))
    if with_knuckle:
        for x0, x1 in g.knuckle_spans()["plate"]:
            # Full-width web (an arm, for "below") under the knuckle so it prints with
            # nothing overhanging (back down = world +Y up); the tilt sweep proves clearance
            web = _box(x0, x1, mp.plate_t - 0.1, g.axis_Y, g.axis_Z - mp.knuckle_r, g.axis_Z + mp.knuckle_r)
            knuckle = _cyl_x(mp.knuckle_r, x0, x1, g.axis_Y, g.axis_Z)
            plate = (plate.union(web).union(knuckle)
                     .cut(_teardrop_x(mp.pin_d / 2, x0 - 1, x1 + 1, g.axis_Y, g.axis_Z, up="y")))
    log.debug("Plate x %.1f..%.1f x %.0f x %.1f mm, holes at %s, knuckle=%s",
              g.plate_x0, g.plate_x1, mp.plate_h, mp.plate_t, g.screw_xs(), with_knuckle)
    return plate


def build_cradle(g: Geometry, with_knuckles: bool = True) -> cq.Workplane:
    """Cradle in case coordinates (assembled around the case)."""
    mp, cp = g.mp, g.cp
    x0, x1 = g.cradle_x0, g.cradle_x1
    z_lip0 = g.case_top + mp.lip_gap
    z_top = z_lip0 + mp.lip_t
    cradle = _box(x0, x1, -g.wall_out, g.wall_out, g.floor_bot, z_top)
    cradle = cradle.cut(_box(x0 - 1, x1 + 1, -g.wall_in, g.wall_in, -mp.fit, z_top + 1))
    for xc in mp.lip_x:
        for side in (1, -1):
            lip = _box(xc - mp.lip_l / 2, xc + mp.lip_l / 2, side * g.wall_in, side * (g.hw - mp.lip_in),
                       z_lip0, z_top)
            # Lead-in: the lip's top slopes from the wall down to its tip, so the case
            # pushes the walls apart going in; the underside stays flat as the catch
            tip = side * (g.hw - mp.lip_in)
            ramp = (cq.Workplane("YZ", origin=(xc - mp.lip_l / 2 - 1, 0, 0))
                    .polyline([(tip, z_top + 0.01), (tip, z_lip0 + 0.05), (side * g.wall_in, z_top + 0.01)]).close()
                    .extrude(mp.lip_l + 2))
            cradle = cradle.union(lip.cut(ramp))
    if with_knuckles:
        cradle = cradle.union(_cradle_knuckles(g))
        cradle = cradle.cut(_cradle_pin(g))
    log.debug("Cradle x %.1f..%.1f, walls |y| %.2f..%.2f, lips at %s, knuckles=%s",
              x0, x1, g.wall_in, g.wall_out, mp.lip_x, with_knuckles)
    return cradle


def _to_case(g: Geometry, block: cq.Workplane) -> cq.Workplane:
    """
    Hinge-local frame -> case frame. Hinge-local: u along the axis, v away from
    the cradle body (the knuckle's far side), w = case +Z; axis at the origin.
    """
    if g.mp.orientation == "portrait":
        # u -> case +Y, v -> case -X (past the -X end)
        return block.rotate((0, 0, 0), (0, 0, 1), 90).translate((g.axis_x, 0, g.axis_z))
    # u -> case +X, v -> case +Y (beside the +Y wall)
    return block.translate((0, g.axis_y, g.axis_z))


def _cradle_knuckles(g: Geometry) -> cq.Workplane:
    """Both cradle knuckles with their webs, built in hinge-local coordinates."""
    mp = g.mp
    k, r = mp.knuckle_r, mp.knuckle_r / math.sqrt(2)
    out = None
    for u0, u1 in g.knuckle_spans()["cradle"]:
        # Landscape: the web fuses 0.1 mm into the cradle wall. Portrait: the near
        # face is the case's -X end, so stop one fit short of it; the under-floor
        # bridge below makes the connection to the cradle instead.
        near = -g.attach - 0.1 if mp.orientation == "landscape" else -g.attach + mp.fit
        web = _box(u0, u1, near, 0, -k, k)
        knuckle = _cyl_x(k, u0, u1, 0, 0)
        # Fill under the far side of the knuckle with a 45-degree tangent face down
        # to the bed (print is floor down, w up). Its farthest point is
        # 1.082 * knuckle_r from the axis, inside the swing clearance to the plate.
        chamfer = _prism_x([(0, 0), (0, -k), (k * (math.sqrt(2) - 1), -k), (r, -r)], u0, u1)
        part = web.union(knuckle).union(chamfer)
        if mp.orientation == "portrait":
            # Reach back under the case's -X end block to the cradle floor, staying
            # below the case floor (w <= -k + floor_t)
            part = part.union(_box(u0, u1, -g.attach - mp.end_margin - 2.0, near + 0.1, -k, -k + mp.floor_t))
        out = part if out is None else out.union(part)
    return _to_case(g, out)


def _cradle_pin(g: Geometry) -> cq.Workplane:
    """Teardrop pin hole through both cradle knuckles, peak up in print."""
    spans = g.knuckle_spans()["cradle"]
    u0, u1 = min(a for a, _ in spans) - 1, max(b for _, b in spans) + 1
    return _to_case(g, _teardrop_x(g.mp.pin_d / 2, u0, u1, 0, 0, up="z"))


def build_wedge_mount(g: Geometry, angle: float) -> cq.Workplane:
    """One piece: plate (no knuckle) + wedge + cradle (no knuckles) fixed at `angle`."""
    plate = build_plate(g, with_knuckle=False)
    cradle_w = g.pose(build_cradle(g, with_knuckles=False), angle)
    # Wedge: everything between the plate face and the cradle floor's outer plane,
    # over the cradle floor's footprint. Built as a slab in case coordinates on the
    # wall side of the floor, posed, then clipped to Y >= plate face.
    mp = g.mp
    slab = g.pose(_box(g.cradle_x0, g.cradle_x1, -g.wall_out, g.wall_out, g.floor_bot - 60, g.floor_bot + 0.1),
                  angle)
    # Clip to the plate outline: wedge material outside it would float plate_t
    # above the bed in print orientation (back down) and need support
    zone = _box(max(g.cradle_x0, g.plate_x0 + mp.plate_r), min(g.cradle_x1, g.plate_x1 - mp.plate_r),
                mp.plate_t - 0.1, 200, 0, mp.plate_h)
    wedge = slab.intersect(zone)
    return plate.union(wedge).union(cradle_w)


def validate(g: Geometry) -> None:
    """Parameter-level checks."""
    mp, cp = g.mp, g.cp
    if mp.plate_h > mp.feature_h_min:
        raise ValueError(f"plate_h {mp.plate_h} is taller than the shortest feature ({mp.feature_h_min})")
    if g.axis_Z - mp.knuckle_r < 0:
        raise ValueError("plate knuckle hangs below the plate's bottom edge; raise hinge_z")
    if mp.knuckle_r - mp.pin_d / 2 < 0.8:
        raise ValueError("knuckle wall around the pin is under 0.8 mm")
    if mp.orientation not in ("landscape", "portrait"):
        raise ValueError("orientation must be 'landscape' or 'portrait'")
    if mp.mount_side not in ("above", "below"):
        raise ValueError("mount_side must be 'above' or 'below'")
    for sx in g.screw_xs():
        for x0, x1 in g.knuckle_spans()["plate"]:
            if x0 - mp.screw_csk_d / 2 - 1.0 < sx < x1 + mp.screw_csk_d / 2 + 1.0:
                raise ValueError(f"plate screw at x={sx:.1f} runs into a knuckle/arm")
    sp = g.knuckle_spans()["cradle"]
    if any(b <= a for a, b in sp):
        raise ValueError("a cradle knuckle has non-positive length; lengthen the cradle or narrow the gaps")
    # Cradle wall flex while the case pushes past the lips: cantilever from the floor
    wall_len = g.case_top + mp.lip_gap + mp.fit + mp.floor_t
    strain = 1.5 * mp.wall_t * mp.lip_in / wall_len ** 2
    log.debug("Cradle wall snap strain %.2f%% (limit %.2f%%)", strain * 100, mp.max_strain * 100)
    if strain > mp.max_strain:
        raise ValueError(f"cradle wall strain {strain:.3f} > {mp.max_strain}")
    for xc in mp.lip_x:
        if abs(xc - cp.lens_x) - mp.lip_l / 2 < cp.window_d_at(cp.front_gap + cp.lid_t) / 2 + 2:
            raise ValueError(f"lip at x={xc} is next to the lens window")
        if not (g.cradle_x0 <= xc - mp.lip_l / 2 and xc + mp.lip_l / 2 <= g.cradle_x1):
            raise ValueError(f"lip at x={xc} runs off the cradle")
    if mp.knuckle_r * math.sqrt(1 + (math.sqrt(2) - 1) ** 2) >= mp.knuckle_r + mp.swing_gap:
        raise ValueError("knuckle chamfer reaches past the swing clearance; raise swing_gap")
    if mp.mount_side == "below":
        for (p0, p1), (c0, c1) in zip(g.knuckle_spans()["plate"], g.knuckle_spans()["cradle"]):
            pin = (p1 - p0) + (c1 - c0) + mp.knuckle_gap
            log.debug("End pin stack %.1f mm: M3 x %d + nyloc + washer", pin, 5 * math.ceil((pin + 5) / 5))
    else:
        sp = g.knuckle_spans()
        pin = max(b for _, b in sp["cradle"]) - min(a for a, _ in sp["cradle"])
        log.debug("Knuckle stack %.1f mm: use M3 x %d socket head + nyloc + 2 washers",
                  pin, 5 * math.ceil((pin + 4 + 1) / 5))
    log.debug("Mount validation passed")


def overhang_area(shape: cq.Workplane, first_layer: float = 0.3) -> float:
    """Area (mm^2) of faces pointing down more steeply than 45 deg, above the first layer, as printed."""
    area = 0.0
    for solid in shape.solids().vals():
        verts, tris = solid.tessellate(0.05, 0.2)
        for a, b, c in tris:
            va, vb, vc = verts[a], verts[b], verts[c]
            n = (vb - va).cross(vc - va)
            tri_area = n.Length / 2
            if tri_area < 1e-9:
                continue
            nz = n.z / n.Length
            if nz < -0.70710678 - 1e-3 and (va.z + vb.z + vc.z) / 3 > first_layer:
                area += tri_area
    return area


def _vol(shape: cq.Workplane) -> float:
    return sum(s.Volume() for s in shape.solids().vals())


def self_check(g: Geometry, wedge_angle: float) -> None:
    """Solid validity, case fit, FOV clearance, pose sign, and collisions over the tilt sweep."""
    mp, cp = g.mp, g.cp
    plate, cradle = build_plate(g), build_cradle(g)
    base, lid = case.build_base(cp), case.build_lid(cp)
    case_parts = [base, lid] + ([case.build_cable_cap(cp)] if cp.use_cap else [])
    for name, part in (("plate", plate), ("cradle", cradle)):
        if len(part.solids().vals()) != 1 or not part.val().isValid():
            raise RuntimeError(f"{name} is not a single valid solid")
    # Case sits in the cradle with no interference
    for i, part in enumerate(case_parts):
        v = _vol(cradle.intersect(part))
        if v > 1e-3:
            raise RuntimeError(f"cradle interferes with case part {i} ({v:.3f} mm^3)")
    # Lips are retained: each lip overlaps the lid top in plan view
    if mp.lip_in <= mp.fit:
        raise RuntimeError("lips do not reach over the lid edge")
    # Cradle never enters the lens's ray envelope (full 4:3 frame, aperture-aware)
    rays = case.build_ray_envelope(cp, 40.0)
    v = _vol(cradle.intersect(rays))
    log.debug("Cradle vs lens ray envelope: %.4f mm^3", v)
    if v > 1e-3:
        raise RuntimeError(f"cradle vignettes the camera ({v:.3f} mm^3 in the view)")
    # Pose sign: at +tilt the optical axis must point into the printer and down
    for t in (0.0, 30.0, 60.0):
        d = g.lens_dir(t)
        want = (0.0, math.cos(math.radians(t)), -math.sin(math.radians(t)))
        if max(abs(a - b) for a, b in zip(d, want)) > 1e-6:
            raise RuntimeError(f"pose sign wrong at tilt {t}: lens dir {d}, expected {want}")
    # Long axis at tilt 0: landscape along the wall; portrait up ("above") or down ("below")
    want_long = (0.0, 0.0, g.sign) if mp.orientation == "portrait" else (g.sign, 0.0, 0.0)
    d = g.world_dir((1, 0, 0), 0.0)
    if max(abs(a - b) for a, b in zip(d, want_long)) > 1e-6:
        raise RuntimeError(f"{mp.orientation}: case long axis points {d}, expected {want_long}")
    # Collision sweep: cradle + case vs plate, and nothing behind the wall plane
    wall = _box(-500, 500, -50, 0, -500, 500)
    t = 0.0
    while t <= mp.tilt_max + 1e-9:
        moving = [g.pose(cradle, t)] + [g.pose(p, t) for p in case_parts]
        for i, m in enumerate(moving):
            for name, fixed in (("plate", plate), ("wall", wall)):
                v = _vol(m.intersect(fixed))
                if v > 1e-3:
                    raise RuntimeError(f"tilt {t:.0f}: moving part {i} hits the {name} ({v:.3f} mm^3)")
        t += mp.tilt_step
    log.debug("Tilt sweep 0..%.0f deg clear (%s)", mp.tilt_max, mp.mount_side)
    # "below": at tilt 0 the camera must hang below the plate's (world) bottom edge region,
    # i.e. the posed case lies below the hinge in the world
    if mp.mount_side == "below":
        bb = g.to_world(g.pose(case_parts[0], 0.0)).val().BoundingBox()
        hinge_world_z = mp.plate_h - g.axis_Z
        if bb.zmax > hinge_world_z + mp.knuckle_r + 1e-6:
            raise RuntimeError(f"below mode: case top {bb.zmax:.1f} is above the hinge ({hinge_world_z:.1f})")
    # Wedge: one valid solid, and the angle is printable without supports
    if wedge_angle > mp.wedge_max_print:
        raise RuntimeError(f"wedge angle {wedge_angle} > {mp.wedge_max_print}: cradle walls overhang too far")
    wedge = build_wedge_mount(g, wedge_angle)
    if len(wedge.solids().vals()) != 1 or not wedge.val().isValid():
        raise RuntimeError("wedge mount is not a single valid solid")
    for i, part in enumerate(case_parts):
        v = _vol(wedge.intersect(g.pose(part, wedge_angle)))
        if v > 1e-3:
            raise RuntimeError(f"wedge mount interferes with case part {i} ({v:.3f} mm^3)")
    # Every printable, in its print orientation, must be (nearly) support-free
    printables = {"wall plate": print_orient_plate(plate), "cradle": print_orient_cradle(g, cradle),
                  "wedge": print_orient_plate(wedge)}
    for name, part in printables.items():
        bb = part.val().BoundingBox()
        if abs(bb.zmin) > 1e-3:
            raise RuntimeError(f"{name} does not sit on the bed in print orientation (zmin {bb.zmin:.3f})")
        a = overhang_area(part)
        log.debug("Overhang area %s: %.1f mm^2 (limit %.0f)", name, a, mp.max_overhang_area)
        if a > mp.max_overhang_area:
            raise RuntimeError(f"{name} has {a:.0f} mm^2 of >45 deg overhang; needs supports")
    log.info("Mount self-check passed: hinge sweep 0..%.0f deg, wedge %.0f deg", mp.tilt_max, wedge_angle)


def print_orient_cradle(g: Geometry, part: cq.Workplane) -> cq.Workplane:
    """Cradle floor (case Z = floor_bot) down on the bed."""
    return part.translate((0, 0, -g.floor_bot))


def print_orient_plate(part: cq.Workplane) -> cq.Workplane:
    """Plate back (world Y=0) down on the bed: map world Y -> print Z."""
    return part.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, 0, 0))


VIEWER_CONFIGS = [
    # key, label, MountParams overrides, fixed wedge angle (None = hinge, tilt adjustable)
    ("landscape_above", "Hinge: landscape, camera above the feature", {}, None),
    ("landscape_below", "Hinge: landscape, camera below the feature (end hinge)", {"mount_side": "below"}, None),
    ("portrait_above", "Hinge: portrait 9:16, camera above the feature", {"orientation": "portrait"}, None),
    ("wedge_above_45", "Wedge 45 deg: landscape, above", {}, 45.0),
    ("wedge_below_45", "Wedge 45 deg: landscape, below", {"mount_side": "below"}, 45.0),
]


def _case_to_world(g: Geometry, tilt: float) -> List[float]:
    """Row-major 4x4 matrix taking case-frame points to world at `tilt`, measured by posing points."""
    def pt(x, y, z):
        return g.to_world(g.pose(cq.Workplane().add(cq.Vertex.makeVertex(x, y, z)), tilt)).val().toTuple()
    o = pt(0, 0, 0)
    cols = [tuple(a - b for a, b in zip(pt(*e), o)) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return [cols[0][0], cols[1][0], cols[2][0], o[0],
            cols[0][1], cols[1][1], cols[2][1], o[1],
            cols[0][2], cols[1][2], cols[2][2], o[2],
            0.0, 0.0, 0.0, 1.0]


def _rot_x_about(point, deg: float) -> List[float]:
    """Row-major 4x4: rotation about the world +X axis through `point` by `deg`."""
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    _, py, pz = point
    return [1, 0, 0, 0,
            0, c, -s_, py - c * py + s_ * pz,
            0, s_, c, pz - s_ * py - c * pz,
            0, 0, 0, 1]


def _matmul(a: List[float], b: List[float]) -> List[float]:
    return [sum(a[r * 4 + k] * b[k * 4 + c] for k in range(4)) for r in range(4) for c in range(4)]


def write_viewer_configs(out_dir: str) -> None:
    """
    Meshes and transforms for the viewer's configuration picker. The viewer
    applies world = R_x(-tilt about the hinge axis) * M0 to the case group, so
    this checks that formula against the real pose at several tilts before
    writing anything.
    """
    cp = case.Params()
    meta = {}
    for key, label, overrides, wedge_angle in VIEWER_CONFIGS:
        g = Geometry(cp, MountParams(**overrides))
        m0 = _case_to_world(g, 0.0)
        axis = g.to_world(cq.Workplane().add(cq.Vertex.makeVertex(0, g.axis_Y, g.axis_Z))).val().toTuple()
        for t in (20.0, 45.0, 70.0):
            want, got = _case_to_world(g, t), _matmul(_rot_x_about(axis, -t), m0)
            if max(abs(a - b) for a, b in zip(want, got)) > 1e-6:
                raise RuntimeError(f"viewer transform for {key} disagrees with the pose at tilt {t}")
        if wedge_angle is None:
            fixed = g.to_world(build_plate(g))
            cradle = build_cradle(g)
        else:
            fixed = g.to_world(build_wedge_mount(g, wedge_angle))
            cradle = None
        files = {"fixed": f"viewer_{key}_fixed.stl"}
        cq.exporters.export(fixed, os.path.join(out_dir, files["fixed"]), tolerance=0.05, angularTolerance=0.2)
        if cradle is not None:
            files["cradle"] = f"viewer_{key}_cradle.stl"
            cq.exporters.export(cradle, os.path.join(out_dir, files["cradle"]), tolerance=0.05, angularTolerance=0.2)
        meta[key] = {"label": label, "files": files, "m0": m0, "axis": list(axis),
                     "tilt": wedge_angle if wedge_angle is not None else 45.0,
                     "fixed_tilt": wedge_angle is not None, "plate_h": g.mp.plate_h,
                     "axis_from_wall": g.axis_Y}
        log.info("Viewer config %s: axis at %s, %s", key, tuple(round(v, 2) for v in axis), files)
    with open(os.path.join(out_dir, "viewer_mounts.json"), "w") as f:
        json.dump(meta, f, indent=1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="out")
    ap.add_argument("--tilt", type=float, default=45.0, help="hinge pose in the assembly STEP, degrees down")
    ap.add_argument("--viewer", action="store_true", help="write the viewer's configuration meshes and exit")
    ap.add_argument("--orientation", choices=["landscape", "portrait"], default="landscape")
    ap.add_argument("--side", choices=["above", "below"], default="above",
                    help="camera above the feature, or hanging below it (plate mounted upside down)")
    ap.add_argument("--wedge-angle", type=float, default=45.0, help="fixed wedge angle, degrees down")
    ap.add_argument("--log-level", default=os.environ.get("LOG_LEVEL", "INFO"),
                    choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%Y-%m-%dT%H:%M:%S%z")
    if args.viewer:
        os.makedirs(args.out, exist_ok=True)
        write_viewer_configs(args.out)
        return
    cp, mp = case.Params(), MountParams(orientation=args.orientation, mount_side=args.side)
    log.debug("Mount parameters: %s", asdict(mp))
    g = Geometry(cp, mp)
    try:
        case.validate(cp)
        validate(g)
        self_check(g, args.wedge_angle)
    except Exception:
        log.exception("Mount generation failed")
        raise

    os.makedirs(args.out, exist_ok=True)
    plate, cradle = build_plate(g), build_cradle(g)
    wedge = build_wedge_mount(g, args.wedge_angle)
    o = ("" if mp.orientation == "landscape" else "_portrait") + ("" if mp.mount_side == "above" else "_below")
    # The plate only changes with the arm (below); the cradle only with orientation
    plate_name = "mount_wall_plate.stl" if mp.mount_side == "above" else f"mount_wall_plate{o}.stl"
    # Cradle knuckles differ by orientation and by side (centre vs end hinge)
    cradle_name = f"mount_cradle{o}.stl"
    stls = {
        plate_name: print_orient_plate(plate),
        cradle_name: print_orient_cradle(g, cradle),
        f"mount_wedge{o}_{args.wedge_angle:.0f}deg.stl": print_orient_plate(wedge),
    }
    for name, shape in stls.items():
        path = os.path.join(args.out, name)
        cq.exporters.export(shape, path, tolerance=0.02, angularTolerance=0.1)
        bb = shape.val().BoundingBox()
        log.info("Wrote %s (%d bytes), print z %.2f..%.2f", path, os.path.getsize(path), bb.zmin, bb.zmax)
    base, lid = case.build_base(cp), case.build_lid(cp)
    w = lambda shape: g.to_world(shape)  # noqa: E731  (mount frame -> world)
    asm = (cq.Assembly(name="u1_front_mount")
           .add(w(plate), name="mount_wall_plate", color=cq.Color(0.69, 0.37, 0.63))
           .add(w(g.pose(cradle, args.tilt)), name=f"mount_cradle_at_{args.tilt:.0f}deg", color=cq.Color(0.6, 0.6, 0.6))
           .add(w(g.pose(base, args.tilt)), name="case_base", color=cq.Color(0.31, 0.49, 0.67))
           .add(w(g.pose(lid, args.tilt)), name="case_lid", color=cq.Color(0.85, 0.55, 0.23))
           .add(w(g.pose(case.build_fov_pyramid(cp, cp.stream_aspect_w, cp.stream_aspect_h), args.tilt)),
                name="ref_fov_stream_16x9", color=cq.Color(0.88, 0.29, 0.29, 0.25))
           .add(_box(-300, 300, -5, 0, -100, 60), name="ref_wall_plane_Y0", color=cq.Color(0.8, 0.8, 0.8, 0.3)))
    if cp.use_cap:
        asm.add(w(g.pose(case.build_cable_cap(cp), args.tilt)), name="case_cable_cap", color=cq.Color(0.85, 0.75, 0.35))
    path = os.path.join(args.out, f"u1_front_mount{o}.step")
    asm.save(path)
    log.info("Wrote %s (%d bytes); world: X along wall, Y into printer, Z up, wall at Y=0, plate Z 0..%.0f",
             path, os.path.getsize(path), mp.plate_h)


if __name__ == "__main__":
    main()
