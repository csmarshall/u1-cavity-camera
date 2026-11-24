"""Section + iso of the cradle-free pose mount (wedge, plate, thick-floor case) with the cover, toolhead and window keep-outs (world). POSE_CASE=VARIANT:0|180 picks the case (default side+Y:180)."""
import argparse, os, sys, logging
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path[:0] = [R + "/cad", R + "/tools"]
import cadquery as cq
import elp_16mp_case as case
import u1_wedge_kit as uk
import render_previews as rp
ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("pose", type=float, nargs=4, metavar=("ANGLE", "DY", "DZ", "SHIFT"))
ap.add_argument("--log-level", default=os.environ.get("LOG_LEVEL", "WARNING"))
args = ap.parse_args()
logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
a, dy, dz, s = args.pose
vname, rot = os.environ.get("POSE_CASE", "side+Y:180").rsplit(":", 1)
rot = int(rot)
kit = uk.Kit(uk.KitParams())
plate = kit.build_plate()
kit.set_drop(-dz); kit.shift = s; kit.g.axis_Y = dy
cp = kit.direct_params(vname)
kit.half_w = cp.out_w / 2
wedge = kit.build_wedge(a)
base, lid = kit.build_direct_base(vname), case.build_lid(cp)
L = [rp.Layer("mount", wedge.val()), rp.Layer("plate", plate.val()),
     rp.Layer("base", kit.posed(kit.to_mount_frame(base, rot), a).val()),
     rp.Layer("lid", kit.posed(kit.to_mount_frame(lid, rot), a).val())]
if cp.use_cap:
    L.append(rp.Layer("cable", kit.posed(kit.to_mount_frame(case.build_cable_cap(cp), rot), a).val()))
n_parts = len(L)
L += [rp.Layer("cover", kit.g.to_world(kit.g.cover()).intersect(uk._box(-40, 40, -40, 40, -80, 40)).val(), 0.35),
      rp.Layer("toolhead keep-out", kit.toolhead().intersect(uk._box(-40, 40, -40, 60, -80, 40)).val(), 0.35),
      rp.Layer("window keep-out", kit.window().intersect(uk._box(-40, 40, 0, 60, 0, 48)).val(), 0.35)]
out = R + "/docs/renders"
rp.render_section(L, "x", 0.0, f"Pose {a:g} deg, dy {dy:g}, dz {dz:g}, camera shift {s:g} mm: centreline section", out + f"/pose_{a:g}_dy{dy:g}_dz{dz:g}_shift{s:g}_section.png")
rp.render_iso(L[:n_parts], f"Pose {a:g} deg, dy {dy:g}, camera shift {s:g} mm", out + f"/pose_{a:g}_dy{dy:g}_dz{dz:g}_shift{s:g}_iso.png", elev=15, azim=-30)
print("wrote", out)
