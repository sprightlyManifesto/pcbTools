//! End-to-end tests: run the `convert` binary on the example files.

use std::path::{Path, PathBuf};
use std::process::Command;

fn bin() -> &'static str {
    env!("CARGO_BIN_EXE_convert")
}

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("convert-tests-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn run(args: &[&Path]) -> String {
    let out = Command::new(bin()).args(args).output().expect("run convert");
    let stderr = String::from_utf8_lossy(&out.stderr).to_string();
    assert!(out.status.success(), "convert failed: {stderr}");
    stderr
}

fn example(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("examples").join(name)
}

/// (entity type, layer, values by group code) for every DXF entity.
fn dxf_entities(text: &str) -> Vec<(String, Vec<(i32, f64)>)> {
    let lines: Vec<&str> = text.lines().map(|l| l.trim()).collect();
    let mut out = Vec::new();
    let mut in_entities = false;
    let mut i = 0;
    while i + 1 < lines.len() {
        let (code, val) = (lines[i], lines[i + 1]);
        if code == "2" && val == "ENTITIES" {
            in_entities = true;
        } else if code == "0" && val == "ENDSEC" {
            in_entities = false;
        } else if code == "0" && in_entities {
            out.push((val.to_string(), Vec::new()));
        } else if in_entities {
            if let (Some(e), Ok(c), Ok(v)) = (out.last_mut(), code.parse::<i32>(), val.parse::<f64>()) {
                e.1.push((c, v));
            }
        }
        i += 2;
    }
    out
}

fn get(vals: &[(i32, f64)], code: i32) -> f64 {
    vals.iter().find(|(c, _)| *c == code).unwrap().1
}

/// Every LINE/ARC end point must coincide with another entity's end point.
fn assert_closed_loops(dxf: &str) {
    let mut ends: Vec<(f64, f64)> = Vec::new();
    for (kind, v) in dxf_entities(dxf) {
        match kind.as_str() {
            "LINE" => {
                ends.push((get(&v, 10), get(&v, 20)));
                ends.push((get(&v, 11), get(&v, 21)));
            }
            "ARC" => {
                let (cx, cy, r) = (get(&v, 10), get(&v, 20), get(&v, 40));
                for code in [50, 51] {
                    let a = get(&v, code).to_radians();
                    ends.push((cx + r * a.cos(), cy + r * a.sin()));
                }
            }
            _ => {}
        }
    }
    for (i, p) in ends.iter().enumerate() {
        let ok = ends.iter().enumerate().any(|(j, q)| j != i && (p.0 - q.0).hypot(p.1 - q.1) < 2e-6);
        assert!(ok, "open end point at {p:?}");
    }
}

#[test]
fn gerber_to_dxf_uses_lines_arcs_and_circles_in_closed_loops() {
    let out = tmp("board.dxf");
    run(&[&example("board.gbr"), &out]);
    let dxf = std::fs::read_to_string(&out).unwrap();
    let ents = dxf_entities(&dxf);
    let count = |k: &str| ents.iter().filter(|(e, _)| e == k).count();
    assert!(count("CIRCLE") >= 5, "round pads should become CIRCLEs");
    assert!(count("ARC") >= 10, "round track ends / arcs should become ARCs");
    assert!(count("LINE") > 0);
    assert_closed_loops(&dxf);
}

#[test]
fn svg_to_dxf_keeps_layers_and_closes_loops() {
    let out = tmp("drawing.dxf");
    run(&[&example("drawing.svg"), &out]);
    let dxf = std::fs::read_to_string(&out).unwrap();
    assert!(dxf.contains("\nCopper\n") && dxf.contains("\nOutline\n"));
    assert_closed_loops(&dxf);
}

#[test]
fn gerber_to_svg_gives_each_layer_its_own_colour() {
    let out = tmp("two.svg");
    run(&[&example("board.gbr"), &example("drawing.svg"), &out]);
    let svg = std::fs::read_to_string(&out).unwrap();
    let colours: std::collections::HashSet<&str> =
        svg.match_indices("fill=\"#").map(|(i, _)| &svg[i + 6..i + 13]).collect();
    assert_eq!(svg.matches("inkscape:groupmode=\"layer\"").count(), 3);
    assert!(colours.len() >= 3, "{colours:?}");
}

#[test]
fn round_trip_gerber_svg_gerber() {
    // A layer without clear polarity converts element by element.
    let src = tmp("simple.gbr");
    std::fs::write(
        &src,
        "%FSLAX36Y36*%\n%MOMM*%\n%ADD10C,0.25*%\n%ADD11C,1.6*%\nD11*\nX0Y0D03*\nD10*\nX0Y0D02*\nX5000000Y0D01*\n\
         G03X10000000Y5000000I0J5000000D01*\nG36*\nX0Y10000000D02*\nG01X5000000Y10000000D01*\nX5000000Y15000000D01*\n\
         X0Y10000000D01*\nG37*\nM02*\n",
    )
    .unwrap();
    let svg = tmp("rt.svg");
    let gbr = tmp("rt.gbr");
    run(&[&src, &svg]);
    let s = std::fs::read_to_string(&svg).unwrap();
    assert!(s.contains("<circle") && s.contains("stroke-linecap=\"round\"") && s.contains(" A5 5 0 0 0 "));
    run(&[&svg, &gbr]);
    let text = std::fs::read_to_string(&gbr).unwrap();
    assert!(text.contains("%FSLAX46Y46*%") && text.trim_end().ends_with("M02*"));
    // Pads come back as flashes, tracks as draws, regions as regions.
    assert!(text.contains("D03*"));
    assert!(text.contains("D01*"));
    assert!(text.contains("G36*"));
}

#[test]
fn dxf_to_gerber_and_svg() {
    let dxf = tmp("in.dxf");
    run(&[&example("board.gbr"), &dxf]);
    let gbr = tmp("from_dxf.gbr");
    run(&[&dxf, &gbr]);
    let text = std::fs::read_to_string(&gbr).unwrap();
    // The cut-out in the copper region must come back as a clear region.
    assert!(text.contains("%LPC*%"));
    let svg = tmp("from_dxf.svg");
    run(&[&dxf, &svg]);
    assert!(std::fs::read_to_string(&svg).unwrap().contains("fill-rule=\"evenodd\""));
}

#[test]
fn multi_layer_gerber_output_writes_one_file_per_layer() {
    let out = tmp("multi.gbr");
    run(&[&example("drawing.svg"), &out]);
    assert!(tmp("multi-Copper.gbr").exists());
    assert!(tmp("multi-Outline.gbr").exists());
}
