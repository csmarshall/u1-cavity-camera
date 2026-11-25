#!/usr/bin/env python3
"""
Portrait-below test kit: try camera angles and cable-exit corners with as
few unique prints as possible, then fuse the winner into one fixed part.

Pieces:
  wall plate  - VHB/M3 plate for the U1's front lip with a dovetail rail
                along X and an end stop. Stays on the printer.
  wedges      - one per angle (30/45/60/75 by default). Each slides onto the
                rail and carries a square key on its angled face, with an M3
                pilot hole for one countersunk M3 x 10. Each is placed so the
                camera hangs below the lip, tucked under the sloping cover,
                as close to the lip as THAT angle allows.
  cradle      - universal, symmetric about the lens: square key pockets at
                both ends of its floor, so it mounts either way up (swaps
                the cable between top and bottom corners), and wall notches
                at all four corners for side-exit cases.
  cases       - end exit, side_end +Y, side_end -Y (elp_16mp_case.py); one
                cable cap fits all three.

Frames: world as in u1_front_mount.py (X along the wall, Y into the printer,
Z up, lip face at Y=0, plate on Z 0..plate_h). Portrait below: the case hangs
with its long axis vertical, lens into the printer, tilted down by the wedge
angle. Positions come from u1_front_mount.Geometry(portrait, below).

The cover slope under the lip (below_slope_deg) is ASSUMED until measured;
only the wedges depend on it, and they regenerate in seconds.

Usage:
    python3 cad/u1_wedge_kit.py [--out out] [--angles 30 45 60 75] [--slope 35] [--viewer] [--log-level DEBUG]
"""
import argparse
import json
import logging
import math
import os
from dataclasses import dataclass, asdict, replace
from typing import Dict, List, Tuple

import cadquery as cq

import elp_16mp_case as case
import u1_front_mount as fm

log = logging.getLogger("u1_wedge_kit")


@dataclass
class KitParams:
    angles: Tuple[float, ...] = (30.0, 45.0, 60.0, 75.0)
    # Measured by Charles 2026-11-24, each rounded DOWN (toward less space):
    lip_h: float = 25.0              # A: flat height of the lip
    below_gap: float = 15.0          # B: open gap behind the lip plane, under its bottom edge
    below_slope_deg: float = 45.0    # C: outer shell, outward from vertical
    toolhead_dz: float = 45.0        # D: lip bottom edge down to the toolhead top, toolhead at front-most travel
    toolhead_dy: float = 10.0        # E: lip inner face to the toolhead front, same position
    rib_spacing: float = 57.0        # F: rib spacing on the lip's inner face; the plate fits between two ribs
    toolhead_margin: float = 2.0     # extra keep-out around the toolhead
    window_z: float = 40.0           # lip bottom edge up to the removable window's bottom edge (Charles, 2026-11-24)
    window_margin: float = 2.0       # nothing may come closer than this to the window line
    rib_clear: float = 1.0           # plate clearance to each rib
    # How much lower the camera hangs (its top end below the lip's bottom edge lets it
    # tuck back into the gap; costs depth). The viewer compares several.
    drop: float = 0.0
    # Charles's pick in the Pose explorer (2026-11-24): angle_deg, dy_mm, dz_mm (+ up), camera_shift_mm,
    # and the case variant:turn (side+Y:180 = cable out the X+ Y+ corner). --pose defaults to it and
    # the viewer's explorer opens on it.
    chosen_pose: Tuple[float, float, float, float] = (45.0, 17.5, 0.0, 35.3)
    chosen_case: str = "side+Y:180"
    viewer_drops: Tuple[float, ...] = (0.0,)   # raising/lowering was compared and dropped (Charles: keep position)
    # Dovetail rail on the plate (profile in YZ, running along X)
    rail_base_w: float = 8.0         # width at the plate face (Z)
    rail_h: float = 3.0              # height off the plate face (Y)
    rail_flank_deg: float = 45.0     # flank angle from the plate face; 45 prints support-free
    rail_clear: float = 0.2          # slot clearance per side
    stop_l: float = 3.0              # end stop length at the rail's +X end
    # Square key between wedge and cradle
    key_w: float = 10.0
    key_h: float = 3.0
    key_clear: float = 0.2
    key_inset: float = 9.0           # key centre from each cradle end (case X)
    pad_w: float = 14.0              # wedge face around the key
    pilot_d: float = 2.5             # M3 self-tap pilot in the key
    pilot_depth: float = 8.0
    screw_clear_d: float = 3.4
    screw_csk_d: float = 6.5         # M3 countersunk head, flush with the cradle floor top
    # Universal cradle
    cradle_floor_t: float = 4.6      # thick enough to hold the key pockets under a flush M3 head
    # Placement
    lip_gap: float = 0.6             # min clearance from moving parts to plate/rail/cover
    label_depth: float = 0.6
    carrier_wall: float = 1.6        # wedge material outside the dovetail slot
    max_overhang_area: float = 60.0  # mm^2 per printable (cradle lips + key edges)
    # Adjustable slide (cradle-free mount, issue #10): the case rides a V-groove in its back on a
    # wedge tongue, along its own axis (the explorer's shift), locked by an M3 button-head bolt into
    # a plain hex nut that slides in a printed channel in the floor (Charles has no T-nuts).
    # Travel runs from the case top level with the plate top up to the window keep-out.
    slide_tongue_l: float = 16.0     # tongue length along the case axis
    slide_pad_l: float = 24.0        # wedge contact pad length along the case axis
    slide_groove_w: float = 7.0      # V-groove width at the case back
    slide_groove_d: float = 1.2      # groove depth; 45 deg flanks
    slide_clear: float = 0.3         # tongue-to-groove clearance (horizontal, per side)
    slide_end_clear: float = 0.5     # groove/channel ends past the travel
    slide_neck_t: float = 1.2        # floor ledge the nut pulls against
    slide_top_margin: float = 0.5    # travel stops this far short of the window keep-out
    nut_af: float = 5.5              # M3 hex nut across flats
    nut_t: float = 2.4               # M3 hex nut thickness
    nut_clear: float = 0.2           # per side: channel flats stop the nut turning (corners 6.35 > channel)
    bolt_l: float = 10.0             # M3 button-head length; Charles has 6/8/10/12 (6 is too short: check_slide)
    bolt_clear_d: float = 3.4
    bolt_head_d: float = 5.7         # ISO 7380 M3 button head
    cbore_clear: float = 0.6         # counterbore diameter over the head
    scale_depth: float = 0.4         # engraved scale and index depth
    scale_tick_w: float = 0.5
    scale_text_h: float = 2.2


def _box(*a) -> cq.Workplane:
    return case._box(*a)


def _vol(w: cq.Workplane) -> float:
    return sum(s.Volume() for s in w.solids().vals())


def _clean(w: cq.Workplane) -> cq.Workplane:
    """Merge coplanar faces when OCC can; some lofted/cut shapes make its upgrader fail, keep them as is."""
    try:
        return w.clean()
    except Exception as e:      # OCP Standard_Failure (zero-norm normal) has no narrower Python type
        log.debug("clean() skipped: %s", e)
        return w


def _convex_hull(points: List[cq.Vector]) -> cq.Solid:
    """Convex hull solid of 3D points (scipy for the facets, OCC for the solid)."""
    from scipy.spatial import ConvexHull
    pts = [(p.x, p.y, p.z) for p in points]
    hull = ConvexHull(pts)
    faces = []
    for simplex, eq in zip(hull.simplices, hull.equations):
        tri = [cq.Vector(*pts[i]) for i in simplex]
        # Orient each triangle outward (scipy's facet normal is eq[:3])
        n = (tri[1] - tri[0]).cross(tri[2] - tri[0])
        if n.dot(cq.Vector(*eq[:3])) < 0:
            tri = [tri[0], tri[2], tri[1]]
        faces.append(cq.Face.makeFromWires(cq.Wire.makePolygon(tri, close=True)))
    solid = cq.Solid.makeSolid(cq.Shell.makeShell(faces)).fix()
    return solid


def _ivol(a: cq.Workplane, b: cq.Workplane) -> float:
    """Overlap volume; OCC returns a null shape (ValueError) when two solids don't touch at all."""
    try:
        return _vol(a.intersect(b))
    except ValueError:
        return 0.0


class Kit:
    """Derived geometry shared by builders and checks."""

    def __init__(self, kp: KitParams):
        self.kp = kp
        self.cp = case.Params()                       # reference case for the pose frame
        self.variants = {
            "end": case.Params(),
            "side+Y": case.Params(cable_exit="side_end", cable_side=1),
            "side-Y": case.Params(cable_exit="side_end", cable_side=-1),
        }
        # Universal cradle length: symmetric about the lens and inside every variant's ends
        half = min(min(-v.out_x0, v.out_x1) for v in self.variants.values())
        self.mp = fm.MountParams(orientation="portrait", mount_side="below", hinge_end=-1,
                                 below_slope_deg=kp.below_slope_deg, below_gap=kp.below_gap,
                                 floor_t=kp.cradle_floor_t,
                                 tilt_max=0.0, tilt_step=5.0)
        self.g = fm.Geometry(self.cp, self.mp)
        self.base_axis_Z = self.g.axis_Z
        self.set_drop(kp.drop)
        self.shift = 0.0                              # camera slid up its own axis, mm (custom pose only)
        self.slide = None                             # Slide geometry once setup_slide() has run (cradle-free mount)
        self.half_w = None                            # wedge half-width (world X); None = the cradle's, g.wall_out
        self.g.cradle_x0, self.g.cradle_x1 = -(half - self.mp.end_margin), half - self.mp.end_margin
        # The kit plate has no hinge arms: it sits between two ribs on the lip
        hp = kp.rib_spacing / 2 - kp.rib_clear
        self.g.plate_x0, self.g.plate_x1 = -hp, hp
        inset = self.mp.screw_inset
        self.g.screw_xs = lambda: (-hp + inset, hp - inset)
        if self.mp.plate_h > kp.lip_h:
            raise ValueError(f"plate_h {self.mp.plate_h} is taller than the lip ({kp.lip_h})")
        self.rail_c = self.mp.plate_h / 2
        self.rail_top_w = kp.rail_base_w + 2 * kp.rail_h / math.tan(math.radians(kp.rail_flank_deg))
        # Key pockets at both ends and the middle: the wedge attaches at whichever is nearest the rail,
        # so a raised camera is held further down its case
        self.key_xs = (self.g.cradle_x0 + kp.key_inset, 0.0, self.g.cradle_x1 - kp.key_inset)
        log.debug("Cradle x %.2f..%.2f, keys at %s, rail top width %.1f",
                  self.g.cradle_x0, self.g.cradle_x1, self.key_xs, self.rail_top_w)

    def set_drop(self, drop: float) -> None:
        """Hang the camera `drop` mm lower (the below-mode mount frame is flipped, so +Z there is down)."""
        self.g.axis_Z = self.base_axis_Z + drop
        self.drop = drop

    @property
    def wedge_hw(self) -> float:
        """Wedge half-width: the cradle's for the kit, the case's for the cradle-free mount (flat sides either way)."""
        return self.g.wall_out if self.half_w is None else self.half_w

    def posed(self, part: cq.Workplane, angle: float) -> cq.Workplane:
        """
        Case-frame part -> world at `angle`, with the camera (and its cradle) slid
        `self.shift` mm up its own axis (toward case -X, which is up in portrait-below).
        Same composition as the viewer's Pose explorer: m(angle) * Translate_case(-shift, 0, 0).
        """
        if self.shift:
            part = part.translate((-self.shift, 0, 0))
        return self.g.to_world(self.g.pose(part, angle))

    def _pad_world_z(self, x: float, angle: float) -> float:
        return self.posed(cq.Workplane().add(cq.Vertex.makeVertex(x, 0, self.g.floor_bot)), angle).val().Z

    def rail_height_x(self, angle: float) -> float:
        """Cradle-frame X on the floor's outer centre line whose world Z is the rail centre (Z is linear in x)."""
        x0, x1 = self.g.cradle_x0, self.g.cradle_x1
        z0, z1 = self._pad_world_z(x0, angle), self._pad_world_z(x1, angle)
        if abs(z1 - z0) < 1e-9:
            return 0.0
        return x0 + (self.rail_c - z0) * (x1 - x0) / (z1 - z0)

    def pad_pair(self, angle: float) -> Tuple[float, float]:
        """
        The two adjacent key pockets the long pad spans: the pair whose centre is nearest
        rail height, so the wedge is a short, square-on fill between plate face and cradle
        back. At shift 0 this is the top + middle pair (the kit's original choice).
        """
        target = self.rail_height_x(angle)
        pairs = list(zip(self.key_xs[:-1], self.key_xs[1:]))
        best = min(pairs, key=lambda p: abs((p[0] + p[1]) / 2 - target))
        xc = (best[0] + best[1]) / 2
        log.debug("%.0f deg, shift %.1f: rail height at cradle x=%.2f; pad on pockets %s (centre %.2f, "
                  "world Z off rail by %.2f mm)", angle, self.shift, target, best, xc,
                  self._pad_world_z(xc, angle) - self.rail_c)
        return best

    # ---------- plate ----------
    def rail(self, grow: float = 0.0, x0=None, x1=None) -> cq.Workplane:
        """Dovetail rail (world), optionally grown by `grow` all round (for the slot)."""
        kp, mp = self.kp, self.mp
        x0 = self.g.plate_x0 + mp.plate_r if x0 is None else x0
        x1 = self.g.plate_x1 - mp.plate_r - kp.stop_l if x1 is None else x1
        y0, y1 = mp.plate_t - 1.0, mp.plate_t + kp.rail_h + grow   # 1 mm into the plate: no stray faces
        hb, ht = kp.rail_base_w / 2 + grow, self.rail_top_w / 2 + grow
        c = self.rail_c
        return fm._prism_x([(y0, c - hb), (y1, c - ht), (y1, c + ht), (y0, c + hb)], x0, x1)

    def build_plate(self) -> cq.Workplane:
        kp, mp = self.kp, self.mp
        plate = self.g_plate_body()
        stop_x0 = self.g.plate_x1 - mp.plate_r - kp.stop_l
        stop = _box(stop_x0, stop_x0 + kp.stop_l, mp.plate_t - 1.0, mp.plate_t + kp.rail_h,
                    self.rail_c - self.rail_top_w / 2, self.rail_c + self.rail_top_w / 2)
        return plate.union(self.rail()).union(stop)

    def g_plate_body(self) -> cq.Workplane:
        """Plate slab with countersunk M3 holes (from u1_front_mount, no hinge)."""
        return fm.build_plate(self.g, with_knuckle=False)

    # ---------- cradle ----------
    def build_cradle(self) -> cq.Workplane:
        """Universal cradle in case coordinates: notches at all four corners, key pockets at both ends."""
        kp, mp, g = self.kp, self.mp, self.g
        cradle = fm.build_cradle(g, with_knuckles=False)
        side = self.variants["side+Y"]
        half = side.nozzle_r + 0.6
        top = g.case_top + mp.lip_gap + mp.lip_t + 1
        for xs in (side.exit_x, -side.exit_x):
            for s in (1, -1):
                cradle = cradle.cut(_box(xs - half, xs + half, s * (g.wall_in - 0.1), s * (g.wall_out + 20),
                                         g.floor_bot + mp.floor_t - 0.01, top))
        kw = kp.key_w / 2 + kp.key_clear
        for xk in self.key_xs:
            cradle = cradle.cut(_box(xk - kw, xk + kw, -kw, kw, g.floor_bot - 0.01, g.floor_bot + kp.key_h + 0.2))
            # Countersunk M3 from the floor top, flush, through to the pocket
            csk = cq.Solid.makeCone(kp.screw_csk_d / 2, 0, kp.screw_csk_d / 2,
                                    pnt=cq.Vector(xk, 0, -mp.fit + 0.01), dir=cq.Vector(0, 0, -1))
            hole = cq.Solid.makeCylinder(kp.screw_clear_d / 2, mp.floor_t + 1,
                                         pnt=cq.Vector(xk, 0, g.floor_bot - 0.5), dir=cq.Vector(0, 0, 1))
            cradle = cradle.cut(cq.Workplane().add(csk)).cut(cq.Workplane().add(hole))
        return cradle

    def lip_xs_universal(self) -> Tuple[float, ...]:
        return self.g.lip_xs()

    # ---------- placement ----------
    def moving_parts(self) -> List[cq.Workplane]:
        """Cradle plus every case variant's outside envelope, both ways round, in case coordinates."""
        parts = [self.build_cradle()]
        for v in self.variants.values():
            env = case.build_size_mockup(v)
            parts += [env, env.rotate((0, 0, 0), (0, 0, 1), 180)]
        return parts

    def toolhead(self) -> cq.Workplane:
        """Toolhead keep-out (world): at front-most travel it reaches toolhead_dy from the lip face, toolhead_dz below the lip."""
        kp = self.kp
        return _box(-500, 500, kp.toolhead_dy - kp.toolhead_margin, 500, -500, -kp.toolhead_dz + kp.toolhead_margin)

    def window(self) -> cq.Workplane:
        """Keep-out (world) behind the lip from the removable window's bottom edge up: Charles doesn't want the camera in it."""
        kp = self.kp
        return _box(-500, 500, 0, 500, kp.window_z - kp.window_margin, 500)

    def place(self, angle: float, parts: List[cq.Workplane]) -> float:
        """Smallest hinge-reference distance from the wall that clears plate, rail and cover at `angle`."""
        kp, g = self.kp, self.g
        obstacles = (_box(-500, 500, -1, self.mp.plate_t + kp.rail_h + kp.lip_gap, 0, self.mp.plate_h)
                     .union(g.cover()))
        world = lambda w: g.to_world(w)  # noqa: E731
        cover_w = world(obstacles).union(self.toolhead()).union(self.window())

        def clear(ay: float) -> bool:
            g.axis_Y = ay
            return all(_ivol(self.posed(p, angle), cover_w) < 1e-3 for p in parts)

        lo = self.mp.plate_t + kp.rail_h + kp.lip_gap
        # Moving out from the lip first frees the cover, then (further out) runs into the
        # toolhead, so step outward to the first clear position, then bisect back toward the lip
        hi = None
        for i in range(41):
            ay = lo + i * 2.0
            if clear(ay):
                hi = ay
                break
        if hi is None:
            raise RuntimeError(f"{angle:.0f} deg: no position clears both the cover and the toolhead")
        if hi == lo:
            g.axis_Y = lo
            return lo
        lo = hi - 2.0
        for _ in range(6):
            mid = (lo + hi) / 2
            lo, hi = (lo, mid) if clear(mid) else (mid, hi)
        g.axis_Y = hi
        return hi

    # ---------- wedge ----------
    def build_wedge(self, angle: float) -> cq.Workplane:
        """Wedge for `angle` in world coordinates (call place() first)."""
        kp, mp, g = self.kp, self.mp, self.g
        w = self.wedge_hw
        # Face A on the plate front, tall enough to swallow the rail
        za0 = self.rail_c - self.rail_top_w / 2 - 2.0
        za1 = self.rail_c + self.rail_top_w / 2 + 2.0
        ya = mp.plate_t + 0.0
        a_pts = [cq.Vector(-w, ya, za0), cq.Vector(w, ya, za0), cq.Vector(w, ya, za1), cq.Vector(-w, ya, za1)]
        # Face B: pad on the cradle floor's outer plane around the upper key. Case -X
        # is up in portrait-below; pair corners by their world position: A's top edge
        # with the pad's upper edge, bottom with lower, -X with -X.
        hp = kp.pad_w / 2

        # Long pad across two adjacent pockets: two keys, two M3 screws, ~3x the contact
        # area of one key and no twist (Charles's call). Which pair depends on the camera
        # shift: the one nearest rail height (top + middle when unshifted).
        # Slide (cradle-free mount): one pad centred at rail height carrying the tongue and bolt.
        if self.slide is not None:
            x_top = x_mid = None
            xk, hp = self.slide["xt0"], kp.slide_pad_l / 2
        else:
            x_top, x_mid = self.pad_pair(angle)
            xk = (x_top + x_mid) / 2
            hp = (x_mid - x_top) / 2 + kp.pad_w / 2

        def pad(x, y):
            v = self.posed(cq.Workplane().add(cq.Vertex.makeVertex(x, y, g.floor_bot)), angle).val()
            return cq.Vector(*v.toTuple())

        lower = sorted([pad(xk + hp, -w), pad(xk + hp, w)], key=lambda v: v.x)   # (-X, +X) in world
        upper = sorted([pad(xk - hp, -w), pad(xk - hp, w)], key=lambda v: v.x)
        if not (upper[0].z > lower[0].z):
            raise RuntimeError(f"{angle:.0f} deg: expected the case -X end to be up in portrait-below")
        log.debug("%.0f deg, drop %.0f: wedge attaches at cradle pocket x=%.1f", angle, self.drop, xk)
        b_ord = [lower[0], lower[1], upper[1], upper[0]]   # matches A: (-X,bottom) (+X,bottom) (+X,top) (-X,top)
        # Convex hull of the 8 corners: always a valid solid. A ruled loft between the two
        # faces twists (self-intersects) when the long pad's edges pair up badly.
        loft = _convex_hull(a_pts + b_ord)
        if loft.Volume() <= 0 or not loft.isValid():
            raise RuntimeError(f"{angle:.0f} deg wedge: hull is invalid (volume {loft.Volume():.1f})")
        hull = cq.Workplane().add(loft)
        # At shallow angles the pad's lower edge tucks behind the lip face: keep the
        # wedge out of the plate slab and the cover
        # Rail carrier: a solid block that wraps the dovetail with carrier_wall all round,
        # so shallow wedges (whose loft hugs the plate) still hold the rail
        y_car = mp.plate_t + kp.rail_h + kp.rail_clear + kp.carrier_wall
        carrier = _box(-w, w, mp.plate_t, y_car, za0, za1)
        hull = hull.union(carrier)
        hull = hull.cut(_box(-500, 500, -100, mp.plate_t, 0, mp.plate_h)).cut(g.to_world(g.cover()))
        hull = _clean(hull)
        if self.slide is not None:
            wedge = self._slide_wedge_features(hull, angle)
            return self._label_wedge(wedge, angle, w)
        # Key on face B, pilot hole into it
        kw = kp.key_w / 2
        key_local, pilots_local = None, []
        for xp in (x_top, x_mid):
            kb = _box(xp - kw, xp + kw, -kw, kw, g.floor_bot - 0.01, g.floor_bot + kp.key_h)
            pb = cq.Workplane().add(cq.Solid.makeCylinder(
                kp.pilot_d / 2, kp.pilot_depth, pnt=cq.Vector(xp, 0, g.floor_bot + kp.key_h + 0.01),
                dir=cq.Vector(0, 0, -1)))
            key_local = kb if key_local is None else key_local.union(kb)
            pilots_local.append(pb)
        key = self.posed(key_local, angle)
        pilots = [self.posed(pb, angle) for pb in pilots_local]
        # Drill the body first, then add pre-drilled keys: drilling after the union
        # makes OCC return a null shape at some angles
        body = hull
        for pb in pilots:
            body = body.cut(pb)
        for pb in pilots:
            key = key.cut(pb)
        wedge = body.union(key)
        # Dovetail slot through the wedge along X
        wedge = wedge.cut(self.rail(grow=kp.rail_clear, x0=-50, x1=50))
        return self._label_wedge(wedge, angle, w)

    def _teardrop_z(self, r: float, x: float, z0: float, z1: float) -> cq.Workplane:
        """
        Hole along mount Z (the bolt axis) from z0 to z1 with a 45-degree peak toward the
        slide's world-+X side: the wedge prints on its -X face, so world +X is up and the
        crown needs no bridge.
        """
        side = self.slide["side"]
        k = r / math.sqrt(2)
        circ = cq.Workplane("XY", origin=(x, 0, z0)).circle(r).extrude(z1 - z0)
        peak = (cq.Workplane("XY", origin=(0, 0, z0))
                .polyline([(x - k, side * k), (x, side * r * math.sqrt(2)), (x + k, side * k)]).close()
                .extrude(z1 - z0))
        return circ.union(peak)

    def slide_bolt_cuts(self) -> Tuple[cq.Workplane, cq.Workplane]:
        """(bolt hole, head counterbore) in the mount frame at the nominal shift."""
        kp, fb, lay, xt = self.kp, self.g.floor_bot, self.slide_layout(), self.slide["xt0"]
        seat_z = fb - lay["seat"]
        hole = self._teardrop_z(kp.bolt_clear_d / 2, xt, seat_z - 1.0, fb + kp.slide_groove_d + 0.5)
        cbore = self._teardrop_z((kp.bolt_head_d + kp.cbore_clear) / 2, xt, seat_z - 80.0, seat_z)
        return hole, cbore

    def slide_hardware(self, angle: float) -> cq.Workplane:
        """M3 button-head bolt + hex nut as installed (world, nominal shift), for renders and the viewer."""
        kp, fb, lay, xt = self.kp, self.g.floor_bot, self.slide_layout(), self.slide["xt0"]
        seat_z = fb - lay["seat"]
        head = cq.Workplane("XY", origin=(xt, 0, seat_z - 1.65)).circle(kp.bolt_head_d / 2).extrude(1.65)
        shank = cq.Workplane("XY", origin=(xt, 0, seat_z)).circle(1.5).extrude(kp.bolt_l)
        nut = (cq.Workplane("XY", origin=(xt, 0, fb + lay["z_ch0"]))
               .polygon(6, kp.nut_af / math.cos(math.pi / 6)).extrude(kp.nut_t)
               .rotate((xt, 0, 0), (xt, 0, 1), 30))          # flats face the channel walls (+-Y)
        return self.posed(head.union(shank).union(nut), angle)

    def _slide_wedge_features(self, hull: cq.Workplane, angle: float) -> cq.Workplane:
        """Tongue, bolt hole + counterbore, rail slot and scale index on the slide wedge."""
        kp, g, sl = self.kp, self.g, self.slide
        fb, xt, w = g.floor_bot, sl["xt0"], self.wedge_hw
        th = kp.slide_groove_d - kp.slide_clear      # tongue stays short of the groove floor: the pad seats
        tongue = fm._prism_x(self._slide_profile(kp.slide_groove_w / 2 - kp.slide_clear, th + 0.01, fb - 0.01),
                             xt - kp.slide_tongue_l / 2, xt + kp.slide_tongue_l / 2)
        hole, cbore = self.slide_bolt_cuts()
        index = _box(xt - kp.scale_tick_w / 2, xt + kp.scale_tick_w / 2,
                     min(sl["side"] * (w - kp.scale_depth), sl["side"] * (w + 0.1)),
                     max(sl["side"] * (w - kp.scale_depth), sl["side"] * (w + 0.1)), fb - 4.0, fb + 0.01)
        # Drill body and tongue separately, then join (drilling after a union can give OCC a null shape)
        drill = [self.posed(hole, angle), self.posed(cbore, angle)]
        body, tng = hull, self.posed(tongue, angle)
        for d in drill:
            body, tng = body.cut(d), tng.cut(d)
        wedge = body.union(tng)
        wedge = wedge.cut(self.rail(grow=kp.rail_clear, x0=-50, x1=50))
        return _clean(wedge.cut(self.posed(index, angle)))

    def _label_wedge(self, wedge: cq.Workplane, angle: float, w: float) -> cq.Workplane:
        """Engrave the angle on the wedge's +X side face where it fits."""
        kp = self.kp
        # Angle label on the +X side face
        # Angle label on the +X side face: search a grid over that face for a spot
        # where the whole label lands on solid material (the slot, the clipped
        # corners and the face's shape vary with angle and hang depth)
        def label_at(y, z, size):
            return (cq.Workplane("YZ", origin=(w + 0.01, y, z))
                    .text(f"{angle:.0f}", size, -kp.label_depth, kind="bold", combine=False, clean=False))
        # Candidate spots on the largest +X face, nearest its centre first (a whole-bbox
        # grid made the build take >10 min)
        side = max(wedge.faces(">X").vals(), key=lambda f: f.Area())
        fb, fc = side.BoundingBox(), side.Center()
        cands = [(fb.ymin + 2 + i * 1.5, fb.zmin + 2 + j * 1.5)
                 for i in range(int((fb.ylen - 4) / 1.5) + 1) for j in range(int((fb.zlen - 4) / 1.5) + 1)]
        cands.sort(key=lambda c: (c[0] - fc.y) ** 2 + (c[1] - fc.z) ** 2)
        txt = None
        for size in (3.6, 2.8):
            # Reference: the part of the label behind the face plane (it starts 0.01 mm proud)
            ref = _vol(label_at(0, 0, size).intersect(_box(-500, w, -500, 500, -500, 500)))
            for y, z in cands[:120]:
                cand = label_at(y, z, size)
                if _ivol(cand, wedge) > 0.95 * ref:   # >= 95% of the label on material
                    txt = cand
                    break
            if txt is not None:
                break
        if txt is None:
            log.warning("%.0f deg wedge (drop %.0f): side face too small for a label; leaving it blank",
                        angle, self.drop)
            return _clean(wedge)
        wedge = _clean(wedge.cut(txt))
        if len(wedge.solids().vals()) != 1 or not wedge.val().isValid():
            log.debug("%.0f deg wedge solids: %s", angle,
                      [(round(s_.Volume(), 1), s_.isValid()) for s_ in wedge.solids().vals()])
        return wedge

    # ---------- direct mount (no cradle) ----------
    def direct_params(self, vname: str) -> case.Params:
        """
        Case variant for mounting the wedge straight to the case back. The floor grows by the
        cradle's fit + floor, so the case's outer floor lands exactly on the cradle's old outer
        face (g.floor_bot) and the lens stays where the pose put it; the extra floor holds the
        key pockets and the flush M3 heads the cradle floor used to.
        """
        return replace(self.variants[vname], floor=self.variants[vname].floor + self.mp.fit + self.mp.floor_t)

    def to_mount_frame(self, part: cq.Workplane, rot: int = 0) -> cq.Workplane:
        """Case part (outer floor at Z=0) -> mount frame (outer floor at g.floor_bot), turned `rot` deg about the lens axis."""
        return part.rotate((0, 0, 0), (0, 0, 1), rot).translate((0, 0, self.g.floor_bot))

    def from_mount_frame(self, part: cq.Workplane, rot: int = 0) -> cq.Workplane:
        """Inverse of to_mount_frame: mount frame -> case frame (outer floor at Z=0)."""
        return part.translate((0, 0, -self.g.floor_bot)).rotate((0, 0, 0), (0, 0, 1), -rot)

    def setup_slide(self, angle: float, vname: str, rot: int, held_mount: List[cq.Workplane]) -> Dict:
        """
        Slide geometry for the cradle-free mount at the current pose (axis_Y, drop and shift = the
        nominal pick, already set). The wedge is world-fixed and built at the nominal shift; the
        case slides under it. A case point at mount x sits under the wedge's mount-x `xt0` when
        x - s = xt0 - s_nom, so the tick for shift s goes at x = xt0 + (s - s_nom).
        Travel: the top of `held_mount` (mount-frame case parts) moves linearly with s, so solve for
        level-with-plate-top (s_min) and the window keep-out less slide_top_margin (s_max).
        """
        kp, g = self.kp, self.g
        s_nom = self.shift
        xt0 = self.rail_height_x(angle)

        def top_z(s: float) -> float:
            self.shift = s
            z = max(self.posed(part, angle).val().BoundingBox().zmax for part in held_mount)
            self.shift = s_nom
            return z
        z0, z1 = top_z(0.0), top_z(10.0)
        rate = (z1 - z0) / 10.0
        if rate <= 0:
            raise RuntimeError(f"{angle:.0f} deg: shifting the camera does not raise it (rate {rate:.3f})")
        s_min = (self.mp.plate_h - z0) / rate
        s_max = (kp.window_z - kp.window_margin - kp.slide_top_margin - z0) / rate
        if not s_min <= s_nom <= s_max:
            raise RuntimeError(f"nominal shift {s_nom:.1f} is outside the travel {s_min:.1f}..{s_max:.1f}")
        # Which mount-frame side faces world +X: the scale and index go there (the wedge's +X face
        # is on top as printed, and it is the side Charles reads from the front of the printer)
        probe = lambda y: self.posed(cq.Workplane().add(cq.Vertex.makeVertex(0, y, g.floor_bot)), angle).val().X
        side = 1 if probe(1.0) > probe(0.0) else -1
        lo, hi = xt0 + s_min - s_nom, xt0 + s_max - s_nom          # tongue/bolt centre sweep in case mount-x
        self.slide = dict(angle=angle, rot=rot, vname=vname, s_nom=s_nom, s_min=s_min, s_max=s_max,
                          xt0=xt0, side=side, lo=lo, hi=hi, rate=rate)
        log.info("Slide: shift %.1f..%.1f mm (nominal %.1f), %.2f mm up per mm of slide, tongue at case x %.1f",
                 s_min, s_max, s_nom, rate, xt0)
        return self.slide

    def _slide_profile(self, half_w: float, depth: float, z0: float) -> List[Tuple[float, float]]:
        """45-degree V-groove/tongue cross-section (y, z) from z0 (the case back) up `depth`."""
        return [(-half_w, z0), (half_w, z0), (half_w - depth, z0 + depth), (-half_w + depth, z0 + depth)]

    def slide_layout(self) -> Dict[str, float]:
        """Case-floor stack for the slide, in local Z above the outer floor face, and the bolt seat depth."""
        kp = self.kp
        z_neck = kp.slide_groove_d
        z_ch0 = z_neck + kp.slide_neck_t
        z_ch1 = z_ch0 + kp.nut_t + 2 * kp.nut_clear
        # Bolt tip just past the nut when it is pulled down onto the neck ledge
        seat = kp.bolt_l - (z_ch0 + kp.nut_t + kp.nut_clear)
        return dict(z_neck=z_neck, z_ch0=z_ch0, z_ch1=z_ch1, seat=seat,
                    tip=kp.bolt_l - seat, ch_hw=kp.nut_af / 2 + kp.nut_clear,
                    nut_r=kp.nut_af / math.sqrt(3) + kp.nut_clear)

    def slide_base_cuts(self, p: case.Params) -> Tuple[cq.Workplane, cq.Workplane, float]:
        """
        (track, scale, expected scale volume) in the mount frame with the case turned: the V-groove,
        the neck slot, the nut channel open at the non-cable end, and the engraved scale.
        """
        kp, g, sl, lay = self.kp, self.g, self.slide, self.slide_layout()
        fb, ec = g.floor_bot, kp.slide_end_clear
        gx0, gx1 = sl["lo"] - kp.slide_tongue_l / 2 - ec, sl["hi"] + kp.slide_tongue_l / 2 + ec
        groove = fm._prism_x(self._slide_profile(kp.slide_groove_w / 2, kp.slide_groove_d + 0.01, fb - 0.01),
                             gx0, gx1)
        nr = kp.bolt_clear_d / 2
        neck = _box(sl["lo"] - nr - ec, sl["hi"] + nr + ec, -nr, nr,
                    fb + lay["z_neck"] - 0.01, fb + lay["z_ch0"] + 0.01)
        # Cable exits at the case's local +X end in every variant, so the nut goes in at local -X
        open_dir = -1 if sl["rot"] == 0 else 1
        inner = (sl["lo"] - lay["nut_r"] - ec) if open_dir > 0 else (sl["hi"] + lay["nut_r"] + ec)
        cx0, cx1 = (inner, 500) if open_dir > 0 else (-500, inner)
        channel = _box(cx0, cx1, -lay["ch_hw"], lay["ch_hw"], fb + lay["z_ch0"], fb + lay["z_ch1"])
        track = groove.union(neck).union(channel)
        # Scale on the floor band of the world-+X side: 1 mm ticks, long ones + numbers every 5
        hw, side, d = p.out_w / 2, sl["side"], kp.scale_depth
        y_in, y_out = side * (hw - d), side * (hw + 0.1)
        ticks, expected = None, 0.0
        for s in range(math.ceil(sl["s_min"]), math.floor(sl["s_max"]) + 1):
            x = sl["xt0"] + s - sl["s_nom"]
            major = s % 5 == 0
            th = 2.4 if major else 1.4
            t = _box(x - kp.scale_tick_w / 2, x + kp.scale_tick_w / 2, min(y_in, y_out), max(y_in, y_out),
                     fb + 0.4, fb + 0.4 + th)
            expected += kp.scale_tick_w * d * th
            ticks = t if ticks is None else ticks.union(t)
            if major:
                # Reads upright (toward the lens) from outside the face
                plane = cq.Plane(origin=(x, side * (hw + 0.01), fb + 3.2), xDir=(-side, 0, 0), normal=(0, side, 0))
                txt = cq.Workplane(plane).text(f"{s}", kp.scale_text_h, -(d + 0.01), kind="bold",
                                               halign="center", valign="bottom", combine=False, clean=False)
                ticks = ticks.union(txt)
        return track, ticks, expected

    def build_direct_base(self, vname: str) -> cq.Workplane:
        """
        Thick-floor base for the cradle-free mount. With a slide set up: the V-groove, nut channel
        and engraved scale (issue #10). Without: a key pocket and a flush countersunk M3 at every
        key_xs (symmetric: either way up).
        """
        kp, p = self.kp, self.direct_params(vname)
        base = case.build_base(p)
        if self.slide is not None:
            rot = self.slide["rot"]
            track, scale, _ = self.slide_base_cuts(p)
            base = base.cut(self.from_mount_frame(track, rot)).cut(self.from_mount_frame(scale, rot))
            return _clean(base)
        kw = kp.key_w / 2 + kp.key_clear
        for xk in self.key_xs:
            base = base.cut(_box(xk - kw, xk + kw, -kw, kw, -0.01, kp.key_h + 0.2))
            csk = cq.Solid.makeCone(kp.screw_csk_d / 2, 0, kp.screw_csk_d / 2,
                                    pnt=cq.Vector(xk, 0, p.floor + 0.01), dir=cq.Vector(0, 0, -1))
            hole = cq.Solid.makeCylinder(kp.screw_clear_d / 2, p.floor + 1,
                                         pnt=cq.Vector(xk, 0, -0.5), dir=cq.Vector(0, 0, 1))
            base = base.cut(cq.Workplane().add(csk)).cut(cq.Workplane().add(hole))
        return base

    # ---------- print orientation ----------
    @staticmethod
    def print_back_down(w: cq.Workplane, back_y: float) -> cq.Workplane:
        """World Y -> print Z, back face (Y=back_y) on the bed."""
        return w.rotate((0, 0, 0), (1, 0, 0), 90).translate((0, 0, -back_y))

    def print_wedge(self, w: cq.Workplane) -> cq.Workplane:
        """
        Wedge on its -X side face: its profile is a prism along X, so the dovetail
        slot is a vertical channel, the label (+X face) is on top, and only the key's
        underside overhangs. Works for every angle, including ones that tuck behind
        the lip face and so have no single flat back.
        """
        return w.rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, self.wedge_hw))

    def print_cradle(self, w: cq.Workplane) -> cq.Workplane:
        return w.translate((0, 0, -self.g.floor_bot))


def check_pose(kit: Kit, a: float, wedge: cq.Workplane, plate: cq.Workplane,
               held: List[Tuple[str, cq.Workplane]],
               envelopes: List[Tuple[str, cq.Workplane]] = ()) -> None:
    """
    Pose checks at angle `a` (axis_Y, drop and shift already set). `held` lists, in case
    coordinates, everything the wedge carries (cradle and/or case parts):
      - the lens points where the angle says;
      - nothing held reaches the cover, the plate + rail, the toolhead keep-out or the window;
      - the wedge clears the cover, plate, toolhead, window and everything it holds, and
        wraps the rail.
    `envelopes` (solid outside envelopes, e.g. the size mockup with no key pockets) are
    checked against the keep-outs only, never against the wedge.
    """
    kp, g = kit.kp, kit.g
    d = g.world_dir((0, 0, 1), a)
    want = (0.0, math.cos(math.radians(a)), -math.sin(math.radians(a)))
    if max(abs(x - y) for x, y in zip(d, want)) > 1e-6:
        raise RuntimeError(f"{a:.0f} deg: lens points {d}, expected {want}")
    th, win = kit.toolhead(), kit.window()
    cover_w = g.to_world(g.cover())
    plate_body = kit.g_plate_body()
    keep_outs = [("cover", cover_w), ("plate", plate_body), ("rail", plate), ("toolhead keep-out", th),
                 (f"window (above {kp.window_z - kp.window_margin:.0f} mm over the lip edge)", win)]
    for name, part in list(held) + list(envelopes):
        posed = kit.posed(part, a)
        for kname, k in keep_outs:
            if _ivol(posed, k) > 1e-3:
                raise RuntimeError(f"{a:.0f} deg: the {name} reaches the {kname}")
        if (name, part) in envelopes:
            continue
        if _ivol(posed, wedge) > 1e-3:
            raise RuntimeError(f"{a:.0f} deg wedge interferes with the {name}")
    for kname, k in keep_outs:
        if _ivol(wedge, k) > 1e-3:
            raise RuntimeError(f"{a:.0f} deg wedge runs into the {kname}")
    # The slot must be fully enclosed by wedge material around the rail
    rail_env = kit.rail(x0=-kit.wedge_hw, x1=kit.wedge_hw)
    hull_band = wedge.union(kit.rail(grow=kp.rail_clear, x0=-kit.wedge_hw, x1=kit.wedge_hw))
    # Rail volume NOT covered by wedge+slot must be ~0 (an intersection's volume can
    # double-count overlapping pieces, so measure what's left over instead)
    try:
        uncovered = _vol(rail_env.cut(hull_band))
    except ValueError:
        uncovered = 0.0
    if uncovered > 0.01 * _vol(rail_env):
        raise RuntimeError(f"{a:.0f} deg wedge does not surround the rail")


def self_check(kit: Kit, wedges: Dict[float, cq.Workplane], placements: Dict[float, float],
               plate: cq.Workplane, cradle: cq.Workplane) -> None:
    """Validity, fit, clearance against the cover, rail engagement, pose sign and printability."""
    kp, g, mp = kit.kp, kit.g, kit.mp
    for name, part in [("plate", plate), ("cradle", cradle)] + [(f"wedge {a:.0f}", w) for a, w in wedges.items()]:
        if len(part.solids().vals()) != 1 or not part.val().isValid():
            raise RuntimeError(f"{name} is not a single valid solid")
    # Cradle holds every case variant both ways round
    for vname, v in kit.variants.items():
        parts = [case.build_base(v), case.build_lid(v)] + ([case.build_cable_cap(v)] if v.use_cap else [])
        for rot in (0, 180):
            for p in parts:
                vol = _ivol(cradle, p.rotate((0, 0, 0), (0, 0, 1), rot))
                if vol > 1e-3:
                    raise RuntimeError(f"cradle interferes with the {vname} case at {rot} deg ({vol:.3f} mm^3)")
    rays = case.build_ray_envelope(case.Params(), 40.0)
    if _ivol(cradle, rays) > 1e-3:
        raise RuntimeError("cradle vignettes the camera")
    held = [("cradle", cradle)]
    for vname, v in kit.variants.items():
        env = case.build_size_mockup(v)
        held += [(f"{vname} case", env), (f"{vname} case flipped", env.rotate((0, 0, 0), (0, 0, 1), 180))]
    for a, wedge in wedges.items():
        g.axis_Y = placements[a]
        check_pose(kit, a, wedge, plate, held)
    printables = {"plate": Kit.print_back_down(plate, 0.0), "cradle": kit.print_cradle(cradle)}
    printables.update({f"wedge {a:.0f}": kit.print_wedge(w) for a, w in wedges.items()})
    for name, part in printables.items():
        bb = part.val().BoundingBox()
        if abs(bb.zmin) > 1e-3:
            raise RuntimeError(f"{name} does not sit on the bed (zmin {bb.zmin:.3f})")
        area = fm.overhang_area(part)
        allowed = kp.max_overhang_area
        if name == "cradle":
            # Key-pocket ceilings are flat 10 mm bridges (printable); lips are 1 mm catches
            pocket = kp.key_w + 2 * kp.key_clear
            allowed = len(kit.key_xs) * pocket ** 2 + 4 * mp.lip_l * (mp.lip_in + mp.fit) + 10.0
        if name.startswith("wedge"):
            # Printed on its side, each key's underside is a short (key_h) overhang
            allowed = kp.max_overhang_area + 2 * kp.key_w * kp.key_h
        log.debug("Overhang %s: %.1f mm^2 (allowed %.0f)", name, area, allowed)
        if area > allowed:
            raise RuntimeError(f"{name}: {area:.0f} mm^2 of >45 deg overhang")
    log.info("Kit self-check passed: %d wedges, cradle fits 3 cases x 2 ways, cover, toolhead and window clear, printable",
             len(wedges))


def build_direct_pose(kit: Kit, plate: cq.Workplane, angle: float, dy: float, dz: float, shift: float,
                      vname: str, rot: int, out: str) -> List[str]:
    """
    The final, cradle-free mount for a pose picked in the viewer's Pose explorer: a wedge whose
    tongue rides a V-groove in a thick-floor case base, so the case slides along its own axis
    (issue #10) and locks with an M3 button head into a hex nut in the floor. Explorer semantics:
    dy is the hinge-reference axis from the lip face (axis_Y), dz is + up so drop = -dz, and
    `shift` is the nominal slide (the wedge is built there; the case travels from level with the
    plate top to the window keep-out). `vname`/`rot` pick the case variant and which end is up
    (the cable corner). Runs the case's own checks, the pose checks at both travel ends and the
    nominal, the slide's own checks and printability, then writes wedge, base, lid (and cap) STLs.
    """
    kp = kit.kp
    kit.set_drop(-dz)
    kit.shift = shift
    kit.g.axis_Y = dy
    p = kit.direct_params(vname)
    kit.half_w = p.out_w / 2
    log.info("Direct pose: %.1f deg, dy %.1f, dz %.1f, camera shift %.1f mm, %s case turned %d deg, floor %.1f mm",
             angle, dy, dz, shift, vname, rot, p.floor)
    case.validate(p)
    lid = case.build_lid(p)
    cap = case.build_cable_cap(p) if p.use_cap else None
    outer = [kit.to_mount_frame(x, rot) for x in [case.build_base(p), lid] + ([cap] if cap is not None else [])]
    sl = kit.setup_slide(angle, vname, rot, outer)
    check_slide(kit, p)
    base = kit.build_direct_base(vname)
    case.self_check(p, base, lid)                        # base/lid/board fit and the lens window
    wedge = kit.build_wedge(angle)
    for name, part in [("plate", plate), ("wedge", wedge), ("base", base)]:
        if len(part.solids().vals()) != 1 or not part.val().isValid():
            raise RuntimeError(f"{name} is not a single valid solid")
    check_wedge_slide(kit, wedge, angle)
    held = [("case base", kit.to_mount_frame(base, rot)), ("lid", kit.to_mount_frame(lid, rot))]
    if cap is not None:
        held.append(("cable cap", kit.to_mount_frame(cap, rot)))
    env = [("case envelope", kit.to_mount_frame(case.build_size_mockup(p), rot))]
    # The wedge is world-fixed; the case slides under it. Every pose check at both travel ends
    # and the nominal: this is also the tongue-in-groove and groove end-stop test.
    for s in (sl["s_min"], sl["s_nom"], sl["s_max"]):
        kit.shift = s
        check_pose(kit, angle, wedge, plate, held, envelopes=env)
        log.debug("Pose checks pass at shift %.2f", s)
    kit.shift = sl["s_nom"]
    # Scale truth through the real pose (not the tick formula): with the case slid to s, the
    # case point where the tick for s is engraved must land on the wedge's index in world
    fb = kit.g.floor_bot
    pt = lambda x: cq.Workplane().add(cq.Vertex.makeVertex(x, 0, fb))
    index_w = kit.posed(pt(sl["xt0"]), angle).val()
    for s in (sl["s_min"], sl["s_max"]):
        kit.shift = s
        tick_w = kit.posed(pt(sl["xt0"] + s - sl["s_nom"]), angle).val()
        kit.shift = sl["s_nom"]
        miss = (tick_w.Center() - index_w.Center()).Length
        if miss > 1e-6:
            raise RuntimeError(f"scale tick for shift {s:.1f} lands {miss:.3f} mm off the index")
    # Printability: the base prints floor down; its extra overhangs are the nut channel ceiling
    # (a straight bridge), the groove-to-neck ledges and the engraved scale's ceilings
    lay = kit.slide_layout()
    track, scale, _ = kit.slide_base_cuts(p)
    ch_bb = track.val().BoundingBox()
    groove_l = (sl["hi"] - sl["lo"]) + kp.slide_tongue_l + 2 * kp.slide_end_clear
    ledge = (kp.slide_groove_w / 2 - kp.slide_groove_d) - kp.bolt_clear_d / 2
    chan_l = min(ch_bb.xmax, p.out_l) - max(ch_bb.xmin, -p.out_l)
    allowed_base = (fm.overhang_area(case.build_base(p)) + 2 * lay["ch_hw"] * min(chan_l, p.out_l)
                    + 2 * max(ledge, 0) * groove_l + fm.overhang_area(scale) + 1.0)
    printables = {"plate": (Kit.print_back_down(plate, 0.0), kp.max_overhang_area),
                  # On its side, the tongue's lower flank is a 45 deg face (borderline) along its length
                  "wedge": (kit.print_wedge(wedge),
                            kp.max_overhang_area + kp.slide_tongue_l * kp.slide_groove_d * math.sqrt(2)),
                  "base": (base, allowed_base)}
    for name, (part, allowed) in printables.items():
        bb = part.val().BoundingBox()
        if abs(bb.zmin) > 1e-3:
            raise RuntimeError(f"{name} does not sit on the bed (zmin {bb.zmin:.3f})")
        area = fm.overhang_area(part)
        log.debug("Overhang %s: %.1f mm^2 (allowed %.0f)", name, area, allowed)
        if area > allowed:
            raise RuntimeError(f"{name}: {area:.0f} mm^2 of >45 deg overhang")
    log.info("Direct pose self-check passed: case fit, lens clear, cover/plate/toolhead/window clear over the "
             "whole slide (%.1f..%.1f), nut channel + scale + bolt seat OK, printable", sl["s_min"], sl["s_max"])
    os.makedirs(out, exist_ok=True)
    tag = f"{angle:g}deg_dy{dy:g}_dz{dz:g}_shift{shift:g}"
    vtag = vname.replace("+", "p").replace("-", "m")
    files = {f"pose_wedge_{tag}.stl": kit.print_wedge(wedge),
             f"pose_base_{vtag}_{tag}.stl": base,
             f"pose_lid_{vtag}.stl": lid}
    if cap is not None:
        files[f"pose_cable_cap_{vtag}.stl"] = cap
    paths = []
    for name, shape in files.items():
        path = os.path.join(out, name)
        cq.exporters.export(shape, path, tolerance=0.02, angularTolerance=0.1)
        log.info("Wrote %s", path)
        paths.append(path)
    return paths


def check_slide(kit: Kit, p: case.Params) -> None:
    """Slide stack and extents: webs around the nut channel, bolt length, groove and pad on the case back."""
    kp, sl, lay = kit.kp, kit.slide, kit.slide_layout()
    min_web = 0.8
    roof = p.floor - lay["z_ch1"]
    if roof < min_web:
        raise RuntimeError(f"nut channel roof {roof:.2f} mm under the board cavity (< {min_web})")
    side_web = p.out_w / 2 - max(lay["ch_hw"], kp.slide_groove_w / 2) - kp.scale_depth
    if side_web < min_web:
        raise RuntimeError(f"slide leaves {side_web:.2f} mm of floor beside the channel/groove")
    if not lay["z_ch0"] + 0.75 * kp.nut_t <= lay["tip"] <= lay["z_ch1"]:
        raise RuntimeError(f"bolt tip at {lay['tip']:.2f} mm does not engage the nut inside the channel")
    if lay["seat"] < 1.0:
        raise RuntimeError(f"bolt seat only {lay['seat']:.2f} mm into the wedge: use a longer bolt")
    if kp.bolt_clear_d >= kp.nut_af:
        raise RuntimeError("neck slot is wider than the nut: the nut would pull through")
    if kp.slide_groove_w / 2 - kp.slide_groove_d < kp.bolt_clear_d / 2:
        raise RuntimeError("groove narrower at its floor than the neck slot")
    # Snap pockets sit above the channel (the channel runs out through the end block)
    for h in p.hooks():
        if h["z_tip"] - 0.4 < lay["z_ch1"] + min_web:
            raise RuntimeError(f"nut channel reaches the snap pocket (pocket floor z {h['z_tip'] - 0.4:.2f})")
    # Groove and pad stay on the case back over the whole travel (mount x, case turned)
    x0, x1 = (p.out_x0, p.out_x1) if sl["rot"] == 0 else (-p.out_x1, -p.out_x0)
    gx0 = sl["lo"] - kp.slide_tongue_l / 2 - kp.slide_end_clear
    gx1 = sl["hi"] + kp.slide_tongue_l / 2 + kp.slide_end_clear
    # The groove only takes the bottom slide_groove_d of the floor (the snap pockets above it are
    # checked separately), so it may run under the end blocks; its closed ends are the travel stops
    # and must keep a wall short of the rounded corners
    if gx0 < x0 + p.corner_r + min_web or gx1 > x1 - p.corner_r - min_web:
        raise RuntimeError(f"groove {gx0:.1f}..{gx1:.1f} runs out of the case back ({x0:.1f}..{x1:.1f})")
    px0, px1 = sl["lo"] - kp.slide_pad_l / 2, sl["hi"] + kp.slide_pad_l / 2
    if px0 < x0 + p.corner_r or px1 > x1 - p.corner_r:
        raise RuntimeError(f"wedge pad leaves the case back over the travel ({px0:.1f}..{px1:.1f})")
    log.debug("Slide stack: groove 0..%.1f, neck ..%.1f, nut channel ..%.1f, roof %.2f, side web %.2f, "
              "bolt seat %.2f mm into the wedge, tip %.2f; groove %.1f..%.1f, pad sweep %.1f..%.1f",
              lay["z_neck"], lay["z_ch0"], lay["z_ch1"], roof, side_web, lay["seat"], lay["tip"],
              gx0, gx1, px0, px1)


def check_wedge_slide(kit: Kit, wedge: cq.Workplane, angle: float) -> None:
    """The bolt head has a seat (a ring of wedge under it) and the counterbore reaches open air."""
    kp, g, lay, xt = kit.kp, kit.g, kit.slide_layout(), kit.slide["xt0"]
    r_head = (kp.bolt_head_d + kp.cbore_clear) / 2
    seat_z = g.floor_bot - lay["seat"]
    ring = (cq.Workplane("XY", origin=(xt, 0, seat_z)).circle(r_head + 1.2).circle(r_head + 0.05).extrude(0.8))
    ring_w = kit.posed(ring, angle)
    got, want = _ivol(ring_w, wedge), _vol(ring_w)
    if got < 0.8 * want:
        raise RuntimeError(f"bolt head seat is only {100 * got / want:.0f}% wedge material")
    # A head-sized rod down the counterbore from the seat must leave the wedge without hitting it
    rod = cq.Workplane("XY", origin=(xt, 0, seat_z - 60.0)).circle(r_head - 0.2).extrude(60.0 - 0.05)
    if _ivol(kit.posed(rod, angle), wedge) > 1e-3:
        raise RuntimeError("the counterbore is blocked: a driver cannot reach the bolt head")
    log.debug("Bolt seat %.0f%% material, counterbore clear", 100 * got / want)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="out")
    ap.add_argument("--angles", type=float, nargs="+", default=list(KitParams.angles))
    ap.add_argument("--slope", type=float, default=KitParams.below_slope_deg,
                    help="cover face below the lip, deg from vertical (ASSUMED until measured)")
    ap.add_argument("--gap", type=float, default=KitParams.below_gap,
                    help="open gap behind the lip plane under its bottom edge, mm (ASSUMED until measured)")
    ap.add_argument("--drop", type=float, default=KitParams.drop, help="hang the camera this much lower, mm")
    ap.add_argument("--viewer", action="store_true", help="also write viewer meshes + viewer_kit.json")
    ap.add_argument("--pose", type=float, nargs="*", metavar="N",
                    help="build one wedge for a Pose-explorer export instead of the kit series: angle_deg, "
                         "dy_mm, dz_mm (+ up), camera_shift_mm. Builds the cradle-free final mount. "
                         "Bare --pose uses KitParams.chosen_pose.")
    ap.add_argument("--case", default=KitParams.chosen_case,
                    help="case variant and turn for --pose, VARIANT:0|180 (default side+Y:180 = cable out the "
                         "top end toward +X, Charles's X+ Y+ corner)")
    ap.add_argument("--log-level", default=os.environ.get("LOG_LEVEL", "INFO"),
                    choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%Y-%m-%dT%H:%M:%S%z")
    kp = KitParams(angles=tuple(args.angles), below_slope_deg=args.slope, below_gap=args.gap, drop=args.drop)
    log.debug("Kit parameters: %s", asdict(kp))
    kit = Kit(kp)
    plate, cradle = kit.build_plate(), kit.build_cradle()
    if args.pose is not None:
        pose = tuple(args.pose) or kp.chosen_pose
        if len(pose) != 4:
            raise SystemExit("--pose takes ANGLE DY DZ SHIFT (or nothing, for KitParams.chosen_pose)")
        vname, rot = args.case.rsplit(":", 1)
        if vname not in kit.variants or rot not in ("0", "180"):
            raise SystemExit(f"--case must be one of {list(kit.variants)} with :0 or :180, got {args.case}")
        build_direct_pose(kit, plate, *pose, vname=vname, rot=int(rot), out=args.out)
        return
    parts = kit.moving_parts()
    wedges, placements = {}, {}
    skipped = []
    for a in kp.angles:
        try:
            placements[a] = kit.place(a, parts)
        except RuntimeError as e:
            log.warning("Skipping %.0f deg: %s", a, e)
            skipped.append(a)
            continue
        wedges[a] = kit.build_wedge(a)
        log.info("Wedge %.0f deg: reference axis %.1f mm from the lip", a, placements[a])
    if not wedges:
        raise SystemExit("no wedge angle fits between the cover and the toolhead")
    self_check(kit, wedges, placements, plate, cradle)

    os.makedirs(args.out, exist_ok=True)
    tag = f"gap{kp.below_gap:.0f}_slope{kp.below_slope_deg:.0f}" + ("" if kp.drop == 0 else f"_drop{kp.drop:.0f}")
    files = {"kit_wall_plate.stl": Kit.print_back_down(plate, 0.0),
             "kit_cradle_universal.stl": kit.print_cradle(cradle)}
    for a, w in wedges.items():
        files[f"kit_wedge_{a:.0f}deg_{tag}.stl"] = kit.print_wedge(w)
    for name, shape in files.items():
        path = os.path.join(args.out, name)
        cq.exporters.export(shape, path, tolerance=0.02, angularTolerance=0.1)
        log.info("Wrote %s", path)

    if args.viewer:
        by_drop = {}
        for d in kp.viewer_drops:
            kit.set_drop(d)
            pl, wd = {}, {}
            for a in kp.angles:
                try:
                    pl[a] = kit.place(a, parts)
                except RuntimeError as e:
                    log.warning("Viewer: drop %.0f, %.0f deg skipped: %s", d, a, e)
                    continue
                wd[a] = kit.build_wedge(a)
            by_drop[d] = (wd, pl)
        write_viewer(kit, by_drop, plate, cradle, args.out)


def explorer_block(kit: Kit, names: Dict[str, str], out_dir: str) -> dict:
    """
    Data for the viewer's "Pose explorer": the case->world matrix at tilt 0 with the hinge
    axis on the lip face (axis_Y = 0, drop 0), plus the world point of that axis. The viewer poses
        world = Translate(0, dy, dz) * R_x(-a about axis) * m0_0 * Translate_case(-s, 0, 0)
    where s = t * (cradle_x1 - cradle_x0 - pad_w) is the camera shift up its own axis (the camera slides up its own axis; the viewer re-places the wedge pad on the cradle back
    at the case X whose world Z equals the rail centre, clamped to the cradle). It is asserted here against the real pose, fm._case_to_world, for several (a, axis_Y, drop).
    A positive drop hangs the camera lower, so dz = -drop (the mount frame is flipped for "below").
    """
    g = kit.g
    kit.set_drop(0.0)
    g.axis_Y = 0.0
    m0 = fm._case_to_world(g, 0.0)
    axis = g.to_world(cq.Workplane().add(cq.Vertex.makeVertex(0, g.axis_Y, g.axis_Z))).val().toTuple()
    for a, ay, drop in ((20.0, 0.0, 0.0), (45.0, 33.0, 0.0), (60.0, 12.5, 7.0), (85.0, 40.0, 15.0)):
        kit.set_drop(drop)
        g.axis_Y = ay
        shift = 0.0 if a < 30 else 0.7 * (g.cradle_x1 - g.cradle_x0 - kit.kp.pad_w)
        want = fm._matmul(fm._case_to_world(g, a), [1, 0, 0, -shift, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
        trans = [1, 0, 0, 0, 0, 1, 0, ay, 0, 0, 1, -drop, 0, 0, 0, 1]
        got = fm._matmul(fm._matmul(trans, fm._matmul(fm._rot_x_about(axis, -a), m0)), [1, 0, 0, -shift, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
        err = max(abs(x - y) for x, y in zip(want, got))
        log.debug("Explorer formula at a=%.0f axis_Y=%.1f drop=%.1f: max error %.2e", a, ay, drop, err)
        if err > 1e-6:
            raise RuntimeError(f"explorer pose formula disagrees with the CAD pose at a={a}, axis_Y={ay}, drop={drop}, shift={shift}")
    kit.set_drop(0.0)
    g.axis_Y = 0.0
    mp, kp = kit.mp, kit.kp
    tips = {}
    for vname, v in kit.variants.items():
        tip = v.place_exit(cq.Workplane().add(cq.Vertex.makeVertex(v.nozzle_l, 0, v.cable_z))).val().toTuple()
        tips[vname.replace("+", "p").replace("-", "m")] = list(tip)
    # Cradle-free mount (what --pose builds): thick-floor base (+ cable cap) and lid per variant,
    # already in the mount frame (outer floor at floor_bot), so the explorer poses them like the
    # cradle-held case. The nozzle tips move with the thicker floor.
    # The slide (issue #10) is built for the chosen pose: the wedge and its bolt + nut are
    # world-fixed meshes the explorer shows when its angle/dy/dz match the pick, and every
    # variant's base gets the groove, nut channel and scale (laid out for the chosen turn)
    a0, dy0, dz0, s0 = kp.chosen_pose
    cv, crot = kp.chosen_case.rsplit(":", 1)
    kit.set_drop(-dz0)
    kit.shift, g.axis_Y = s0, dy0
    cp = kit.direct_params(cv)
    kit.half_w = cp.out_w / 2
    outer = [kit.to_mount_frame(x, int(crot)) for x in [case.build_base(cp), case.build_lid(cp)]
             + ([case.build_cable_cap(cp)] if cp.use_cap else [])]
    sl = kit.setup_slide(a0, cv, int(crot), outer)
    slide = {"s_min": sl["s_min"], "s_max": sl["s_max"], "s_nom": s0, "angle": a0, "dy": dy0, "dz": dz0}
    for part, shape in {"wedge": kit.build_wedge(a0), "hw": kit.slide_hardware(a0)}.items():
        fn = f"viewer_kit_slide_{part}.stl"
        cq.exporters.export(shape, os.path.join(out_dir, fn), tolerance=0.02, angularTolerance=0.1)
        slide[part] = fn
    kit.set_drop(0.0)
    kit.shift, g.axis_Y = 0.0, 0.0
    direct = {}
    for vname in kit.variants:
        p = kit.direct_params(vname)
        vtag = vname.replace("+", "p").replace("-", "m")
        solids = [kit.to_mount_frame(kit.build_direct_base(vname)).val()]
        if p.use_cap:
            solids.append(kit.to_mount_frame(case.build_cable_cap(p)).val())
        meshes = {"base": cq.Workplane().add(cq.Compound.makeCompound(solids)), "lid": kit.to_mount_frame(case.build_lid(p))}
        direct[vtag] = {}
        for part, shape in meshes.items():
            fn = f"viewer_kit_direct_{vtag}_{part}.stl"
            cq.exporters.export(shape, os.path.join(out_dir, fn), tolerance=0.05, angularTolerance=0.2)
            direct[vtag][part] = fn
        tip = p.place_exit(cq.Workplane().add(cq.Vertex.makeVertex(p.nozzle_l, 0, p.cable_z)))
        tips[vtag] = list(kit.to_mount_frame(tip).val().toTuple())
    kit.slide, kit.half_w = None, None
    w = kit.direct_params("end").out_w / 2           # the cradle-free wedge is case-wide
    return {"m0": m0, "axis": list(axis), "cradle_x0": g.cradle_x0, "cradle_x1": g.cradle_x1,
            "floor_bot": g.floor_bot, "pad_x": kp.pad_w, "pad_y": 2 * w,
            "plate_t": mp.plate_t, "face_x": w,
            "face_z": [kit.rail_c - kit.rail_top_w / 2 - 2.0, kit.rail_c + kit.rail_top_w / 2 + 2.0],
            "slab": {"x": g.plate_x1, "y": mp.plate_t + kp.rail_h, "z": mp.plate_h},
            "nozzle_tips": tips, "plate": names["plate"], "cradle": names["cradle"],
            "slope": kp.below_slope_deg, "gap": kp.below_gap, "toolhead": [kp.toolhead_dy - kp.toolhead_margin, kp.toolhead_dz - kp.toolhead_margin],
            "window_z": kp.window_z - kp.window_margin, "direct": direct,
            "chosen": {"angle": a0, "dy": dy0, "dz": dz0, "shift": s0, "case": kp.chosen_case}, "slide": slide}


def write_viewer(kit: Kit, by_drop, plate, cradle, out: str) -> None:
    """
    Viewer data: shared meshes (plate, cradle, 3 case variants) plus one wedge
    per angle, and one config per angle x case variant x way round. Each config
    carries the case->world matrix M0 at its angle (no tilt slider: fixed).
    """
    g = kit.g
    files = {"plate": plate, "cradle": cradle}
    for vname, v in kit.variants.items():
        base = case.build_base(v).val()
        solids = [base] + ([case.build_cable_cap(v).val()] if v.use_cap else [])
        files[f"case_{vname}_base"] = cq.Workplane().add(cq.Compound.makeCompound(solids))
        files[f"case_{vname}_lid"] = case.build_lid(v)
    for d, (wd, _) in by_drop.items():
        for a, w in wd.items():
            files[f"wedge_{a:.0f}_d{d:.0f}"] = w
    names = {}
    for k, shape in files.items():
        fn = f"viewer_kit_{k.replace('+', 'p').replace('-', 'm')}.stl"
        cq.exporters.export(shape, os.path.join(out, fn), tolerance=0.05, angularTolerance=0.2)
        names[k] = fn
    def exit_label(vname: str, rot: int, angle: float) -> str:
        """Where the cable leaves, measured: pose the nozzle tip and compare with the lens."""
        v = kit.variants[vname]
        tip = v.place_exit(cq.Workplane().add(cq.Vertex.makeVertex(v.nozzle_l, 0, v.cable_z)))
        tip = tip.rotate((0, 0, 0), (0, 0, 1), rot)
        p_ = g.to_world(g.pose(tip, angle)).val().toTuple()
        lens = g.to_world(g.pose(cq.Workplane().add(cq.Vertex.makeVertex(0, 0, 0)), angle)).val().toTuple()
        vert = "top" if p_[2] > lens[2] else "bottom"
        if vname == "end":
            return f"{vert} end"
        return f"{vert} corner, {'+X' if p_[0] > lens[0] else '-X'} side"

    configs = {}
    for d, (wd, placements) in by_drop.items():
      kit.set_drop(d)
      for a in wd:
        g.axis_Y = placements[a]
        m0 = fm._case_to_world(g, a)
        low = min(g.to_world(g.pose(p, a)).val().BoundingBox().zmin for p in kit.moving_parts())   # cradle + every case envelope
        for vname in kit.variants:
            for rot in (0, 180):
                key = f"w{a:.0f}_d{d:.0f}_{vname}_{rot}".replace("+", "p").replace("-", "m")
                configs[key] = {
                    "label": f"Wedge {a:.0f}° · cable {exit_label(vname, rot, a)}",
                    "angle": a, "m0": m0, "case_rot": rot,
                    "fixed": [names["plate"], names[f"wedge_{a:.0f}_d{d:.0f}"]], "cradle": names["cradle"],
                    "drop": d, "lowest_z": low,
                    "case_base": names[f"case_{vname}_base"], "case_lid": names[f"case_{vname}_lid"],
                    "upside_down": rot == 180, "slope": kit.kp.below_slope_deg, "gap": kit.kp.below_gap,
                    "toolhead": [kit.kp.toolhead_dy, kit.kp.toolhead_dz],
                    "plate_h": kit.mp.plate_h, "axis_from_wall": placements[a]}
    configs["_explorer"] = explorer_block(kit, names, out)
    names.update({f"direct_{v}_{k}": fn for v, d in configs["_explorer"]["direct"].items() for k, fn in d.items()})
    names.update({f"slide_{k}": configs["_explorer"]["slide"][k] for k in ("wedge", "hw")})
    with open(os.path.join(out, "viewer_kit.json"), "w") as f:
        json.dump({"files": sorted(set(names.values())), "configs": configs}, f, indent=1)
    log.info("Wrote viewer kit data: %d configs, %d meshes", len(configs), len(names))


if __name__ == "__main__":
    main()
