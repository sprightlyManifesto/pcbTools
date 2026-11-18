//! SVG writer. Each layer becomes an Inkscape-compatible layer group with its
//! own colour. Elements are converted one to one where possible:
//! flashes -> <circle>/<path>, tracks -> stroked <path>, regions -> filled <path>.
//! Layers that use clear (negative) polarity are written as their final area.

use crate::area;
use crate::model::*;
use std::f64::consts::TAU;
use std::fmt::Write;

const PALETTE: [&str; 12] = [
    "#d62728", "#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#7f7f7f",
    "#393b79", "#637939",
];

fn n(v: f64) -> String {
    let s = format!("{:.6}", v);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
}

/// SVG path data; model y-up is flipped to SVG y-down.
fn path_d(p: &Path, d: &mut String) {
    let _ = write!(d, "M{} {}", n(p.start.x), n(-p.start.y));
    let mut cur = p.start;
    for s in &p.segs {
        match s {
            Seg::Line(e) => {
                let _ = write!(d, " L{} {}", n(e.x), n(-e.y));
            }
            Seg::Arc { end, center, sweep } => {
                let r = cur.dist(*center);
                // y flip turns counter-clockwise into SVG sweep-flag 0.
                let sf = if *sweep > 0.0 { 0 } else { 1 };
                let pieces: Vec<(Pt, f64)> = if (sweep.abs() - TAU).abs() < 1e-6 {
                    vec![(*center * 2.0 - cur, sweep / 2.0), (*end, sweep / 2.0)]
                } else {
                    vec![(*end, *sweep)]
                };
                for (e, sw) in pieces {
                    let large = if sw.abs() > std::f64::consts::PI { 1 } else { 0 };
                    let _ = write!(d, " A{} {} 0 {} {} {} {}", n(r), n(r), large, sf, n(e.x), n(-e.y));
                }
            }
        }
        cur = s.end();
    }
    if p.closed {
        d.push_str(" Z");
    }
}

fn esc(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

pub fn write(doc: &Doc, opts: &Opts) -> String {
    let mut bb = BBox::empty();
    for l in &doc.layers {
        for it in &l.items {
            bb.union(&shape_bbox(&it.shape, opts.tol));
        }
    }
    if bb.is_empty() {
        bb = BBox { min: pt(0.0, 0.0), max: pt(1.0, 1.0) };
    }
    bb.grow(1.0);
    let (w, h) = (bb.max.x - bb.min.x, bb.max.y - bb.min.y);

    let mut out = String::new();
    let _ = writeln!(out, r#"<?xml version="1.0" encoding="UTF-8" standalone="no"?>"#);
    let _ = writeln!(
        out,
        r#"<svg xmlns="http://www.w3.org/2000/svg" xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape" width="{}mm" height="{}mm" viewBox="{} {} {} {}">"#,
        n(w),
        n(h),
        n(bb.min.x),
        n(-bb.max.y),
        n(w),
        n(h)
    );
    for (li, layer) in doc.layers.iter().enumerate() {
        let col = PALETTE[li % PALETTE.len()];
        let id: String = layer.name.chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' }).collect();
        let _ = writeln!(
            out,
            r#"  <g id="layer-{}" inkscape:groupmode="layer" inkscape:label="{}" fill="{col}" stroke="{col}" opacity="0.85">"#,
            id,
            esc(&layer.name)
        );
        let has_clear = layer.items.iter().any(|i| !i.dark);
        if has_clear {
            // Negative polarity cannot be expressed element by element.
            let solid: Vec<Item> = layer.items.iter().filter(|i| !matches!(i.shape, Shape::Hairline { .. })).cloned().collect();
            let mut d = String::new();
            for s in area::compute(&solid, opts.tol) {
                for p in std::iter::once(&s.outer).chain(&s.holes) {
                    if !d.is_empty() {
                        d.push(' ');
                    }
                    path_d(p, &mut d);
                }
            }
            if !d.is_empty() {
                let _ = writeln!(out, r#"    <path d="{d}" fill-rule="evenodd" stroke="none"/>"#);
            }
        }
        for it in &layer.items {
            match &it.shape {
                _ if has_clear && !matches!(it.shape, Shape::Hairline { .. }) => {}
                Shape::Disc { c, r } => {
                    let _ = writeln!(out, r#"    <circle cx="{}" cy="{}" r="{}" stroke="none"/>"#, n(c.x), n(-c.y), n(*r));
                }
                Shape::Stroke { path, width, cap, join } => {
                    let mut d = String::new();
                    path_d(path, &mut d);
                    let cap = match cap {
                        Cap::Round => "round",
                        Cap::Square => "square",
                        Cap::Butt => "butt",
                    };
                    let join = match join {
                        Join::Round => "round".to_string(),
                        Join::Bevel => "bevel".to_string(),
                        Join::Miter(l) => format!(r#"miter" stroke-miterlimit="{}"#, n(*l)),
                    };
                    let _ = writeln!(
                        out,
                        r#"    <path d="{d}" fill="none" stroke-width="{}" stroke-linecap="{cap}" stroke-linejoin="{join}"/>"#,
                        n(*width)
                    );
                }
                Shape::Fill { paths, rule } => {
                    let mut d = String::new();
                    for p in paths {
                        if !d.is_empty() {
                            d.push(' ');
                        }
                        let mut q = p.clone();
                        q.closed = true;
                        path_d(&q, &mut d);
                    }
                    let rule = if *rule == FillRule::EvenOdd { "evenodd" } else { "nonzero" };
                    let _ = writeln!(out, r#"    <path d="{d}" fill-rule="{rule}" stroke="none"/>"#);
                }
                Shape::Hairline { path } => {
                    let mut d = String::new();
                    path_d(path, &mut d);
                    let _ = writeln!(
                        out,
                        r#"    <path d="{d}" fill="none" stroke-width="{}" stroke-linecap="round" stroke-linejoin="round"/>"#,
                        n(opts.line_width)
                    );
                }
            }
        }
        out.push_str("  </g>\n");
    }
    out.push_str("</svg>\n");
    out
}
