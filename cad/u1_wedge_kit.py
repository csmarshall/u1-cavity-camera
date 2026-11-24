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
    rib_clear: float = 1.0           # plate clearance to each rib
    # How much lower the camera hangs (its top end below the lip's bottom edge lets it
    # tuck back into the gap; costs depth). The viewer compares several.
    drop: float = 0.0
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

    def place(self, angle: float, parts: List[cq.Workplane]) -> float:
        """Smallest hinge-reference distance from the wall that clears plate, rail and cover at `angle`."""
        kp, g = self.kp, self.g
        obstacles = (_box(-500, 500, -1, self.mp.plate_t + kp.rail_h + kp.lip_gap, 0, self.mp.plate_h)
                     .union(g.cover()))
        world = lambda w: g.to_world(w)  # noqa: E731
        cover_w = world(obstacles).union(self.toolhead())

        def clear(ay: float) -> bool:
            g.axis_Y = ay
            return all(_ivol(world(g.pose(p, angle)), cover_w) < 1e-3 for p in parts)

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
        w = g.wall_out
        # Face A on the plate front, tall enough to swallow the rail
        za0 = self.rail_c - self.rail_top_w / 2 - 2.0
        za1 = self.rail_c + self.rail_top_w / 2 + 2.0
        ya = mp.plate_t + 0.0
        a_pts = [cq.Vector(-w, ya, za0), cq.Vector(w, ya, za0), cq.Vector(w, ya, za1), cq.Vector(-w, ya, za1)]
        # Face B: pad on the cradle floor's outer plane around the upper key. Case -X
        # is up in portrait-below; pair corners by their world position: A's top edge
        # with the pad's upper edge, bottom with lower, -X with -X.
        hp = kp.pad_w / 2

        # Long pad from the top pocket to the middle pocket: two keys, two M3 screws,
        # ~3x the contact area of one key and no twist (Charles's call)
        x_top, x_mid = self.key_xs[0], self.key_xs[1]
        xk = (x_top + x_mid) / 2
        hp = (x_mid - x_top) / 2 + kp.pad_w / 2

        def pad(x, y):
            v = g.to_world(g.pose(cq.Workplane().add(cq.Vertex.makeVertex(x, y, g.floor_bot)), angle)).val()
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
        key = g.to_world(g.pose(key_local, angle))
        pilots = [g.to_world(g.pose(pb, angle)) for pb in pilots_local]
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
        return w.rotate((0, 0, 0), (0, 1, 0), -90).translate((0, 0, self.g.wall_out))

    def print_cradle(self, w: cq.Workplane) -> cq.Workplane:
        return w.translate((0, 0, -self.g.floor_bot))


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
    cover_w = g.to_world(g.cover())
    plate_body = kit.g_plate_body()
    rail_env = kit.rail(x0=-g.wall_out, x1=g.wall_out)
    for a, wedge in wedges.items():
        g.axis_Y = placements[a]
        d = g.world_dir((0, 0, 1), a)
        want = (0.0, math.cos(math.radians(a)), -math.sin(math.radians(a)))
        if max(abs(x - y) for x, y in zip(d, want)) > 1e-6:
            raise RuntimeError(f"{a:.0f} deg: lens points {d}, expected {want}")
        th = kit.toolhead()
        for part in [kit.build_cradle()] + kit.moving_parts()[1:]:
            if _ivol(g.to_world(g.pose(part, a)), th) > 1e-3:
                raise RuntimeError(f"{a:.0f} deg: the camera or cradle reaches the toolhead keep-out")
        if _ivol(wedge, th) > 1e-3:
            raise RuntimeError(f"{a:.0f} deg wedge reaches the toolhead keep-out")
        if _ivol(wedge, cover_w) > 1e-3 or _ivol(wedge, plate_body) > 1e-3:
            raise RuntimeError(f"{a:.0f} deg wedge runs into the cover or plate")
        if _ivol(wedge, plate) > 1e-3:
            raise RuntimeError(f"{a:.0f} deg wedge interferes with the rail")
        # The slot must be fully enclosed by wedge material around the rail
        hull_band = wedge.union(kit.rail(grow=kp.rail_clear, x0=-g.wall_out, x1=g.wall_out))
        # Rail volume NOT covered by wedge+slot must be ~0 (an intersection's volume can
        # double-count overlapping pieces, so measure what's left over instead)
        try:
            uncovered = _vol(rail_env.cut(hull_band))
        except ValueError:
            uncovered = 0.0
        if uncovered > 0.01 * _vol(rail_env):
            raise RuntimeError(f"{a:.0f} deg wedge does not surround the rail")
        posed_cradle = g.to_world(g.pose(cradle, a))
        if _ivol(wedge, posed_cradle) > 1e-3:
            raise RuntimeError(f"{a:.0f} deg wedge interferes with the cradle")
        for part in kit.moving_parts()[1:]:
            if _ivol(g.to_world(g.pose(part, a)), wedge) > 1e-3:
                raise RuntimeError(f"{a:.0f} deg wedge interferes with a case")
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
    log.info("Kit self-check passed: %d wedges, cradle fits 3 cases x 2 ways, cover clear, printable",
             len(wedges))


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
    ap.add_argument("--log-level", default=os.environ.get("LOG_LEVEL", "INFO"),
                    choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%Y-%m-%dT%H:%M:%S%z")
    kp = KitParams(angles=tuple(args.angles), below_slope_deg=args.slope, below_gap=args.gap, drop=args.drop)
    log.debug("Kit parameters: %s", asdict(kp))
    kit = Kit(kp)
    plate, cradle = kit.build_plate(), kit.build_cradle()
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


def explorer_block(kit: Kit, names: Dict[str, str]) -> dict:
    """
    Data for the viewer's "Pose explorer": the case->world matrix at tilt 0 with the hinge
    axis on the lip face (axis_Y = 0, drop 0), plus the world point of that axis. The viewer poses
        world = Translate(0, dy, dz) * R_x(-a about axis) * m0_0
    which is asserted here against the real pose, fm._case_to_world, for several (a, axis_Y, drop).
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
        want = fm._case_to_world(g, a)
        trans = [1, 0, 0, 0, 0, 1, 0, ay, 0, 0, 1, -drop, 0, 0, 0, 1]
        got = fm._matmul(trans, fm._matmul(fm._rot_x_about(axis, -a), m0))
        err = max(abs(x - y) for x, y in zip(want, got))
        log.debug("Explorer formula at a=%.0f axis_Y=%.1f drop=%.1f: max error %.2e", a, ay, drop, err)
        if err > 1e-6:
            raise RuntimeError(f"explorer pose formula disagrees with the CAD pose at a={a}, axis_Y={ay}, drop={drop}")
    kit.set_drop(0.0)
    g.axis_Y = 0.0
    mp, kp = kit.mp, kit.kp
    tips = {}
    for vname, v in kit.variants.items():
        tip = v.place_exit(cq.Workplane().add(cq.Vertex.makeVertex(v.nozzle_l, 0, v.cable_z))).val().toTuple()
        tips[vname.replace("+", "p").replace("-", "m")] = list(tip)
    w = g.wall_out
    return {"m0": m0, "axis": list(axis), "cradle_x0": g.cradle_x0, "cradle_x1": g.cradle_x1,
            "floor_bot": g.floor_bot, "pad_x": kp.pad_w, "pad_y": 2 * w,
            "plate_t": mp.plate_t, "face_x": w,
            "face_z": [kit.rail_c - kit.rail_top_w / 2 - 2.0, kit.rail_c + kit.rail_top_w / 2 + 2.0],
            "slab": {"x": g.plate_x1, "y": mp.plate_t + kp.rail_h, "z": mp.plate_h},
            "nozzle_tips": tips, "plate": names["plate"], "cradle": names["cradle"],
            "slope": kp.below_slope_deg, "gap": kp.below_gap, "toolhead": [kp.toolhead_dy - kp.toolhead_margin, kp.toolhead_dz - kp.toolhead_margin]}


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
    configs["_explorer"] = explorer_block(kit, names)
    with open(os.path.join(out, "viewer_kit.json"), "w") as f:
        json.dump({"files": sorted(set(names.values())), "configs": configs}, f, indent=1)
    log.info("Wrote viewer kit data: %d configs, %d meshes", len(configs), len(names))


if __name__ == "__main__":
    main()
