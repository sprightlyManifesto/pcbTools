# PCBtools

`convert` is a command-line tool, written in Rust, that converts drawings between
**Gerber** (RS-274X / X2), **DXF** and **SVG**.

```
convert inputfile.grbl outputfile.dxf
convert drawing.svg drawing.dxf
convert board.dxf board.svg
convert top.gtl bottom.gbl silk.gto board.svg     # several inputs -> one multi-layer SVG
```

The file types come from the extensions. Gerber: `.gbr .grb .grbl .gerber .ger .pho .art
.gtl .gbl .gto .gbo .gts .gbs .gtp .gbp .gko .gm1 .g1 …`. DXF: `.dxf`. SVG: `.svg`.
If an extension isn't recognised, the tool looks at the file contents. You can also
force the format with `--from` / `--to`.

## Build

```
cargo build --release
./target/release/convert --help
```

## What each conversion does

| From → To | Behaviour |
|---|---|
| Gerber / SVG → DXF | Works layer by layer. Everything drawn on a layer (flashes, tracks, regions, filled and stroked SVG shapes) is merged into areas, and a **closed loop is drawn around the outside of each area** (plus a loop around each hole). Loops are made of `LINE`, `ARC` and `CIRCLE` entities. Where the boundary is not a straight line or a circular arc (Bézier curves, ellipses, splines), the loop is completed with **short line facets** that stay within `--tolerance`. |
| Gerber → SVG | Each Gerber element is converted as closely as possible: round flashes become `<circle>`, tracks become stroked `<path>`s with round caps and joins (arcs stay SVG arcs), regions and other aperture flashes become filled `<path>`s. **Each layer is an Inkscape layer with its own colour.** |
| SVG → Gerber | The reverse of the above: `<circle>` becomes a round flash, round-capped strokes become tracks, filled shapes become regions. Other stroke styles are converted to their outline region. |
| DXF → SVG / Gerber | Lines, arcs and polylines are chained into loops. By default, closed loops are filled areas and nested loops are holes (the inverse of → DXF); open chains become lines drawn `--line-width` wide. Use `--dxf-lines` to treat all DXF geometry as plain lines. |

Gerber holds only one layer per file. When the output has several layers,
`out.gbr` is written as `out-<layer>.gbr`, one file per layer.

### Layers

* **Gerber**: each input file is one layer, named after the file.
* **SVG**: each top-level `<g>` (or Inkscape layer) is one layer; elements outside a group go to the layer `default`.
* **DXF**: the entity layer names.

## Options

| Option | Default | Meaning |
|---|---|---|
| `--tolerance MM` | 0.002 | Maximum deviation when curves must be split into facets |
| `--line-width MM` | 0.1 | Pen width for zero-width lines (open DXF geometry, zero-size Gerber apertures) |
| `--dxf-lines` | off | Read DXF loops as lines instead of filled areas |
| `--from`, `--to` | from extension | Force the `gerber`, `dxf` or `svg` format |

## Supported features

* **Gerber**: `FS` (leading/trailing zero omission, absolute/incremental), `MO`, all standard
  apertures (`C R O P`, including hole sizes), aperture macros (primitives 1, 4, 5, 7, 20, 21, 22,
  variables and arithmetic, exposure on/off), `D01/D02/D03`, `G01/G02/G03`, `G74/G75`, regions
  (`G36/G37`), dark/clear polarity (`LP`), step and repeat (`SR`), the deprecated image commands
  (`MI SF OF IR`, `G70/G71`, `G90/G91`), and X2 attributes (ignored).
* **SVG**: `path` (all commands), `rect` (incl. rounded corners), `circle`, `ellipse`, `line`,
  `polyline`, `polygon`, `use`, nested `g` with `transform`; fill, fill-rule, stroke, stroke-width,
  stroke-linecap/linejoin/miterlimit, `display:none`; `viewBox` and physical units.
* **DXF** (read): `LINE`, `ARC`, `CIRCLE`, `LWPOLYLINE`/`POLYLINE` (with bulges),
  `ELLIPSE`, `SPLINE` (NURBS or fit points), `$INSUNITS`, mirrored entities (extrusion −Z).
  **DXF** (write): AutoCAD R12 ASCII, millimetres, one coloured layer per input layer.

## Limitations

* SVG text, images, gradients, clip paths, masks and CSS `<style>` sheets are not converted (text should be converted to paths first).
* Gerber moiré macro primitives and aperture transformations (`LM LR LS`) are not supported; you get a warning.
* DXF blocks (`INSERT`), `HATCH`, `TEXT` and dimensions are skipped, with a warning.
* Gerber arcs drawn with a non-round aperture are approximated with a round pen.

## How the outline (→ DXF) works

1. Every shape is turned into polygons. When an arc is split into facets, the
   facet vertices are tagged with the circle they came from.
2. A robust polygon boolean engine ([`i_overlay`](https://crates.io/crates/i_overlay))
   unions the dark shapes and subtracts the clear ones, in drawing order.
3. Each resulting boundary is walked edge by edge. Runs of facets that lie on a
   tagged circle become one true `ARC` (or `CIRCLE`), and collinear edges become one
   `LINE`. Arc end points are placed on the exact line-circle or circle-circle
   intersection. Anything left over is kept as short facets.

## Tests

`cargo test` runs the unit tests and end-to-end tests on the files in `examples/`.
