"""
Close-up review renders of the slide joint (issue #10) for the chosen pose, in the case's
mount frame (case axis = X, case back at floor_bot, lens toward +Z) so the section planes
cut square across and along the track:

  slide_cross_section.png   across the track through the bolt: V-groove + tongue, neck, nut, bolt, counterbore
  slide_long_section.png    along the track centreline: groove ends (the travel stops), nut channel to its open end
  slide_base_back_iso.png   the case back from below: groove, nut channel mouth, engraved scale
  slide_wedge_iso.png       the wedge from the case side: tongue, bolt hole, index mark

The wedge and hardware are built in world coordinates, so they are taken back to the mount
frame through the inverse of the shared pose matrix (case_to_world * T(-shift)).
POSE_CASE=VARIANT:0|180 overrides KitParams.chosen_case.
"""
import argparse
import logging
import os
import sys

R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path[:0] = [R + "/cad", R + "/tools"]
import cadquery as cq  # noqa: E402
import elp_16mp_case as case  # noqa: E402
import u1_front_mount as fm  # noqa: E402
import u1_wedge_kit as uk  # noqa: E402
import render_previews as rp  # noqa: E402

log = logging.getLogger("render_slide")


def main() -> None:
    """Build the chosen pose's slide parts and write the four close-up renders."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log-level", default=os.environ.get("LOG_LEVEL", "WARNING"))
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    kp = uk.KitParams()
    a, dy, dz, s = kp.chosen_pose
    vname, rot = os.environ.get("POSE_CASE", kp.chosen_case).rsplit(":", 1)
    rot = int(rot)
    kit = uk.Kit(kp)
    kit.set_drop(-dz)
    kit.shift = s
    kit.g.axis_Y = dy
    cp = kit.direct_params(vname)
    kit.half_w = cp.out_w / 2
    lid = case.build_lid(cp)
    outer = [kit.to_mount_frame(x, rot) for x in [case.build_base(cp), lid]
             + ([case.build_cable_cap(cp)] if cp.use_cap else [])]
    sl = kit.setup_slide(a, vname, rot, outer)
    wedge_w, hw_w = kit.build_wedge(a), kit.slide_hardware(a)
    base_m = kit.to_mount_frame(kit.build_direct_base(vname), rot)

    m = fm._matmul(fm._case_to_world(kit.g, a), [1, 0, 0, -s, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
    # Rigid inverse built exactly (R^T, -R^T t): cq.Matrix.inverse() picks up enough roundoff
    # for OCC to reject it as non-orthogonal. det(R) is checked so a mirror cannot sneak in.
    R3 = [[m[4 * i + j] for j in range(3)] for i in range(3)]
    t = [m[3], m[7], m[11]]
    det = (R3[0][0] * (R3[1][1] * R3[2][2] - R3[1][2] * R3[2][1]) - R3[0][1] * (R3[1][0] * R3[2][2] - R3[1][2] * R3[2][0])
           + R3[0][2] * (R3[1][0] * R3[2][1] - R3[1][1] * R3[2][0]))
    if abs(det - 1.0) > 1e-6:
        raise SystemExit(f"pose matrix is not a proper rotation (det {det})")
    Rt = [[R3[j][i] for j in range(3)] for i in range(3)]
    ti = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
    from OCP.gp import gp_Trsf
    trsf = gp_Trsf()
    trsf.SetValues(*Rt[0], ti[0], *Rt[1], ti[1], *Rt[2], ti[2])
    to_mount = lambda w: cq.Shape.cast(w.val().moved(cq.Location(trsf)).wrapped)
    wedge_m, hw_m = to_mount(wedge_w), to_mount(hw_w)

    fb, xt = kit.g.floor_bot, sl["xt0"]
    out = R + "/docs/renders"
    crop_x = uk._box(-60, 60, -12, 12, fb - 14, fb + 9)
    layers = [rp.Layer("mount", wedge_m.intersect(crop_x.val())), rp.Layer("bolt + nut", hw_m),
              rp.Layer("base", base_m.val().intersect(crop_x.val()))]
    rp.render_section(layers, "x", xt,
                      f"Slide joint across the track at the bolt (case frame, x = {xt:.1f}): "
                      "V-groove + tongue, neck ledge, M3 nut in its channel, button head in the counterbore",
                      out + "/slide_cross_section.png")
    rp.render_section(layers, "y", 0.0,
                      f"Slide joint along the track centreline (case frame): groove ends are the stops, "
                      f"travel {sl['s_min']:.1f}..{sl['s_max']:.1f} mm; nut channel open at the non-cable end",
                      out + "/slide_long_section.png")
    rp.render_iso([rp.Layer("base", base_m.val())],
                  f"Case back from below: V-groove, nut channel mouth, scale (shift mm, ticks 1 mm)",
                  out + "/slide_base_back_iso.png", elev=-55, azim=-60)
    rp.render_iso([rp.Layer("mount", wedge_m), rp.Layer("bolt + nut", hw_m)],
                  "Wedge from the case side: tongue, bolt + nut, index line on the +X face",
                  out + "/slide_wedge_iso.png", elev=40, azim=-60)
    print("wrote", out)


if __name__ == "__main__":
    main()
