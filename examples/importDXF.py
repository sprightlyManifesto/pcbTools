"""Import the KiCad board layers (DXF) into CadQuery, centred on the origin.

The board outline (Edge_Cuts) bounding box sets the board centre; every layer
is translated by the same vector so that centre lands on (0, 0, 0).

Regenerate the DXFs from the Gerbers with:
    convert kicadproj-F_Cu.gbr F.dxf
    convert kicadproj-B_Cu.gbr B.dxf
    convert kicadproj-Edge_Cuts.gbr Edge_Cuts.dxf

Run in CQ-editor, or headless:
    ../../cq-editor/bin/python importDXF.py
"""
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent

LAYERS = {
    "edge_cuts": "Edge_Cuts.dxf",
    "f_cu": "F.dxf",
    "b_cu": "B.dxf",
}

layers = {name: cq.importers.importDXF(str(HERE / f)) for name, f in LAYERS.items()}

# Board centre from the outline's bounding box.
bb = layers["edge_cuts"].val().BoundingBox() if len(layers["edge_cuts"].vals()) == 1 \
    else cq.Compound.makeCompound(layers["edge_cuts"].vals()).BoundingBox()
centre = cq.Vector(bb.center.x, bb.center.y, 0)
move = -centre

# Move every layer so the board centre sits at the origin.
layers = {name: wp.translate(move) for name, wp in layers.items()}

board_outline = layers["edge_cuts"]
front_copper = layers["f_cu"]
back_copper = layers["b_cu"]

print(f"board {bb.xlen:.3f} x {bb.ylen:.3f} mm, centre ({centre.x:.3f}, {centre.y:.3f})"
      f" -> moved by ({move.x:.3f}, {move.y:.3f})")
for name, wp in layers.items():
    b = cq.Compound.makeCompound(wp.vals()).BoundingBox()
    print(f"  {name:10s} {len(wp.vals()):3d} faces, "
          f"x [{b.xmin:.3f}, {b.xmax:.3f}]  y [{b.ymin:.3f}, {b.ymax:.3f}]")

    if name != "edge_cuts" and not (
        b.xmin >= -bb.xlen / 2 and b.xmax <= bb.xlen / 2
        and b.ymin >= -bb.ylen / 2 and b.ymax <= bb.ylen / 2
    ):
        print(f"  WARNING: {name} lies outside the board outline - the Gerbers were "
              "probably exported with different origins (check %TF.SameCoordinates)")

if "show_object" in globals():  # running inside CQ-editor
    show_object(board_outline, name="Edge_Cuts", options={"color": "yellow"})
    show_object(front_copper, name="F_Cu", options={"color": "red"})
    show_object(back_copper, name="B_Cu", options={"color": "blue"})
