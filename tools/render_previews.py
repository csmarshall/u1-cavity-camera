#!/usr/bin/env python3
"""
Render iso and section PNG previews of the case parts for visual review.

The interference checks in elp_16mp_case.self_check() cannot catch geometry
that is valid but looks wrong, so every new feature gets looked at.

Why not cadquery's SVG exporter: it picks an arbitrary in-plane rotation for
each projection direction and scales from the 3D bounding box, so side views
come out rotated, tiny and with no way to tell handedness. Here:
  - iso views are z-buffered VTK renders (VTK ships with cadquery) with a
    labelled X/Y/Z triad, so orientation and handedness are visible in the
    picture itself;
  - sections are exact planar cuts (solid intersected with a plane face),
    plotted in true world coordinates on labelled 2D axes. No camera transform
    is involved, so a section cannot be mirrored by the renderer.

Usage:
    python3 tools/render_previews.py [--out docs/renders] [--only NAME ...] [--log-level DEBUG]
"""
import argparse
import logging
import math
import os
import sys
from dataclasses import dataclass
from typing import Callable, Dict, List, Sequence, Tuple

import cadquery as cq
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import vtk  # noqa: E402  (ships with cadquery)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cad"))
import elp_16mp_case as case  # noqa: E402

log = logging.getLogger("render_previews")

MESH_TOL = 0.05              # tessellation tolerance, mm
FIG_W_IN, FIG_DPI = 12.0, 130
ISO_ELEV, ISO_AZIM = 28.0, -60.0   # camera above, in front of the -Y face, +X end to the right
ISO_PX_W, ISO_PX_H = 1600, 900
PART_COLOURS = {"base": "#4f81bd", "lid": "#e0a040", "module": "#59a14f", "mount": "#b05fa0"}


@dataclass
class Layer:
    """One part to draw: its solid, a legend name and an alpha."""
    name: str
    shape: cq.Shape
    alpha: float = 1.0


def _tessellate(shape: cq.Shape) -> List[List[Tuple[float, float, float]]]:
    """Triangles of a shape as lists of (x, y, z) tuples."""
    verts, tris = shape.tessellate(MESH_TOL, 0.2)
    pts = [v.toTuple() for v in verts]
    return [[pts[a], pts[b], pts[c]] for a, b, c in tris]


def _vtk_colour(hex_colour: str) -> Tuple[float, float, float]:
    """'#rrggbb' to a VTK 0..1 RGB tuple."""
    h = hex_colour.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))  # type: ignore[return-value]


def render_iso(layers: Sequence[Layer], title: str, path_png: str,
               elev: float = ISO_ELEV, azim: float = ISO_AZIM) -> None:
    """
    Z-buffered shaded 3D view (VTK offscreen) with feature edges and an X/Y/Z triad.

    Camera sits at (elev, azim) about the bounding-box centre, Z up. With the
    defaults it looks from above the -Y face, so +X runs to the right; the
    labelled triad in the corner makes handedness checkable in the image.
    """
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(1.0, 1.0, 1.0)
    lo, hi = [1e9] * 3, [-1e9] * 3
    for layer in layers:
        poly = layer.shape.toVtkPolyData(MESH_TOL, 0.2, normals=True)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*_vtk_colour(PART_COLOURS.get(layer.name, "#999999")))
        actor.GetProperty().SetOpacity(layer.alpha)
        renderer.AddActor(actor)
        edges = vtk.vtkFeatureEdges()
        edges.SetInputData(poly)
        edges.BoundaryEdgesOn(); edges.FeatureEdgesOn(); edges.SetFeatureAngle(30)
        edges.ManifoldEdgesOff(); edges.NonManifoldEdgesOff()
        edge_mapper = vtk.vtkPolyDataMapper()
        edge_mapper.SetInputConnection(edges.GetOutputPort())
        edge_mapper.ScalarVisibilityOff()
        edge_actor = vtk.vtkActor()
        edge_actor.SetMapper(edge_mapper)
        edge_actor.GetProperty().SetColor(0.1, 0.1, 0.1)
        edge_actor.GetProperty().SetLineWidth(1.5)
        renderer.AddActor(edge_actor)
        bb = layer.shape.BoundingBox()
        lo = [min(lo[0], bb.xmin), min(lo[1], bb.ymin), min(lo[2], bb.zmin)]
        hi = [max(hi[0], bb.xmax), max(hi[1], bb.ymax), max(hi[2], bb.zmax)]
        log.debug("Iso layer %s: %d cells", layer.name, poly.GetNumberOfCells())

    # Labelled triad at the -X/-Y/-Z corner of the bounding box
    triad_len = max(5.0, 0.12 * max(hi[i] - lo[i] for i in range(3)))
    axes = vtk.vtkAxesActor()
    axes.SetTotalLength(triad_len, triad_len, triad_len)
    axes.SetShaftTypeToCylinder(); axes.SetCylinderRadius(0.02)
    for cap in (axes.GetXAxisCaptionActor2D(), axes.GetYAxisCaptionActor2D(), axes.GetZAxisCaptionActor2D()):
        cap.GetTextActor().SetTextScaleModeToNone()
        cap.GetCaptionTextProperty().SetColor(0, 0, 0)
        cap.GetCaptionTextProperty().SetFontSize(16)
        cap.GetCaptionTextProperty().ShadowOff(); cap.GetCaptionTextProperty().ItalicOff()
    triad_at = (lo[0] - 1.5 * triad_len, lo[1], lo[2])
    transform = vtk.vtkTransform(); transform.Translate(*triad_at)
    axes.SetUserTransform(transform)
    renderer.AddActor(axes)

    centre = [(lo[i] + hi[i]) / 2 for i in range(3)]
    e, a = math.radians(elev), math.radians(azim)
    view_dir = (math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))
    cam = renderer.GetActiveCamera()
    cam.SetFocalPoint(*centre)
    cam.SetPosition(*[centre[i] + 500 * view_dir[i] for i in range(3)])
    cam.SetViewUp(0, 0, 1)
    cam.SetParallelProjection(True)
    renderer.ResetCamera()
    cam.Zoom(1.15)

    text = vtk.vtkTextActor()
    text.SetInput(f"{title}  [{', '.join(layer.name for layer in layers)}]  elev {elev:.0f}, azim {azim:.0f}")
    text.GetTextProperty().SetColor(0, 0, 0); text.GetTextProperty().SetFontSize(20)
    text.SetDisplayPosition(15, 15)
    renderer.AddViewProp(text)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.AddRenderer(renderer)
    window.SetSize(ISO_PX_W, ISO_PX_H)
    window.SetMultiSamples(8)
    window.Render()
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(window); grab.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(path_png); writer.SetInputConnection(grab.GetOutputPort()); writer.Write()
    window.Finalize()
    log.info("Rendered %s", path_png)


def _plane_face(axis: str, at: float) -> cq.Face:
    """A large planar face normal to world X or Y at the given coordinate."""
    if axis == "x":
        return cq.Face.makePlane(1000, 1000, cq.Vector(at, 0, 0), cq.Vector(1, 0, 0))
    return cq.Face.makePlane(1000, 1000, cq.Vector(0, at, 0), cq.Vector(0, 1, 0))


def render_section(layers: Sequence[Layer], axis: str, at: float, title: str, path_png: str) -> None:
    """
    Exact planar section through each layer, filled, in world coordinates.

    axis="y": the XZ plane at Y=at; screen right is +X (seen from -Y).
    axis="x": the YZ plane at X=at; screen right is +Y (seen from +X).
    """
    u_idx = 0 if axis == "y" else 1
    u_name = "X" if axis == "y" else "Y"
    fig, ax = plt.subplots(figsize=(FIG_W_IN, FIG_W_IN * 0.45))
    plane = _plane_face(axis, at)
    for layer in layers:
        cut = layer.shape.intersect(plane)
        faces = cut.Faces()
        colour = PART_COLOURS.get(layer.name, "#999999")
        for face in faces:
            for tri in _tessellate(face):
                ax.fill([v[u_idx] for v in tri], [v[2] for v in tri], color=colour,
                        alpha=layer.alpha, linewidth=0)
            for edge in face.Edges():
                pts = [edge.positionAt(t / 40).toTuple() for t in range(41)]
                ax.plot([q[u_idx] for q in pts], [q[2] for q in pts], color="black", linewidth=0.6)
        ax.plot([], [], color=colour, linewidth=8, label=layer.name)
        log.debug("Section %s=%.2f layer %s: %d faces", axis.upper(), at, layer.name, len(faces))
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_xlabel(f"{u_name} (mm)"); ax.set_ylabel("Z (mm)")
    ax.set_title(f"{title}\nsection plane {axis.upper()} = {at:.2f} mm")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path_png, dpi=FIG_DPI)
    plt.close(fig)
    log.info("Rendered %s", path_png)


def views(p: case.Params) -> Dict[str, Callable[[str], None]]:
    """Named views; each takes the output PNG path and renders it."""
    base, lid = case.build_base(p).val(), case.build_lid(p).val()
    module = case.mock_module(p).val()
    hook_cable_end = p.hooks()[1]
    asm = [Layer("base", base), Layer("lid", lid), Layer("module", module)]
    return {
        "base_iso": lambda out: render_iso([Layer("base", base)], "Base, as printed (upright)", out),
        "lid_iso": lambda out: render_iso([Layer("lid", lid)], "Lid, assembled orientation (hooks down)",
                                          out, elev=-25),
        "assembly_iso": lambda out: render_iso([Layer("base", base), Layer("lid", lid, 0.55)],
                                               "Assembly", out),
        # Cable axis: bore, open-top slot, nozzle ridges, bay, stop nubs
        "section_cable_axis": lambda out: render_section(
            asm, "y", p.cable_y, "Cable axis: bore, slot, heat-shrink nozzle", out),
        # Cable-end hook: pocket, release window, barb engagement
        "section_hook": lambda out: render_section(
            asm, "y", hook_cable_end["y"], "Snap hooks: pocket, window, barb", out),
        # Through the lens: rails, PCB seat, cone window
        "section_lens": lambda out: render_section(
            asm, "x", p.lens_x, "Lens: rails, PCB seat, FOV cone", out),
    }


def main() -> None:
    """Build parts with default Params and write the preview PNGs."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="docs/renders", help="output directory for PNGs")
    ap.add_argument("--only", nargs="*", help="render only these view names")
    ap.add_argument("--log-level", default=os.environ.get("LOG_LEVEL", "INFO"),
                    choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%Y-%m-%dT%H:%M:%S%z")
    p = case.Params()
    case.validate(p)
    os.makedirs(args.out, exist_ok=True)
    for name, render in views(p).items():
        if args.only and name not in args.only:
            continue
        render(os.path.join(args.out, f"{name}.png"))


if __name__ == "__main__":
    main()
