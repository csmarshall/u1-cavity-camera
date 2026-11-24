"""Section + iso of the custom-pose wedge with plate, cradle, case, cover and toolhead keep-out (world)."""
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
kit = uk.Kit(uk.KitParams())
plate, cradle = kit.build_plate(), kit.build_cradle()
kit.set_drop(-dz); kit.shift = s; kit.g.axis_Y = dy
wedge = kit.build_wedge(a)
cp = case.Params()
L = [rp.Layer("mount", wedge.val()), rp.Layer("plate", plate.val()),
     rp.Layer("cradle", kit.posed(cradle, a).val()),
     rp.Layer("base", kit.posed(case.build_base(cp), a).val()),
     rp.Layer("lid", kit.posed(case.build_lid(cp), a).val()),
     rp.Layer("cover", kit.g.to_world(kit.g.cover()).intersect(uk._box(-40, 40, -40, 40, -80, 40)).val(), 0.35),
     rp.Layer("toolhead keep-out", kit.toolhead().intersect(uk._box(-40, 40, -40, 60, -80, 40)).val(), 0.35)]
out = R + "/docs/renders"
rp.render_section(L, "x", 0.0, f"Pose {a:g} deg, dy {dy:g}, dz {dz:g}, camera shift {s:g} mm: centreline section", out + f"/pose_{a:g}_dy{dy:g}_dz{dz:g}_shift{s:g}_section.png")
rp.render_iso(L[:5], f"Pose {a:g} deg, dy {dy:g}, camera shift {s:g} mm", out + f"/pose_{a:g}_dy{dy:g}_dz{dz:g}_shift{s:g}_iso.png", elev=15, azim=-30)
print("wrote", out)
