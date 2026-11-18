//! Gerber (RS-274X) writer. One layer per file.

use crate::area::{self, AreaShape};
use crate::model::*;
use std::collections::HashMap;
use std::f64::consts::TAU;
use std::fmt::Write;

struct W {
    apertures: Vec<String>,
    by_size: HashMap<i64, i32>,
    body: String,
    dark: bool,
    cur_ap: Option<i32>,
}

fn c(v: f64) -> i64 {
    (v * 1e6).round() as i64
}

impl W {
    fn polarity(&mut self, dark: bool) {
        if dark != self.dark {
            self.body.push_str(if dark { "%LPD*%\n" } else { "%LPC*%\n" });
            self.dark = dark;
        }
    }

    fn select_circle(&mut self, d: f64) {
        let key = c(d);
        let next = 10 + self.apertures.len() as i32;
        let id = *self.by_size.entry(key).or_insert_with(|| next);
        if id == next {
            self.apertures.push(format!("%ADD{id}C,{:.6}*%", d));
        }
        if self.cur_ap != Some(id) {
            let _ = writeln!(self.body, "D{id}*");
            self.cur_ap = Some(id);
        }
    }

    fn segs(&mut self, path: &Path, close: bool) {
        let b = &mut self.body;
        let _ = writeln!(b, "X{}Y{}D02*", c(path.start.x), c(path.start.y));
        let mut cur = path.start;
        for s in &path.segs {
            match s {
                Seg::Line(p) => {
                    let _ = writeln!(b, "G01X{}Y{}D01*", c(p.x), c(p.y));
                }
                Seg::Arc { end, center, sweep } => {
                    let code = if *sweep < 0.0 { "G02" } else { "G03" };
                    // Split full circles into two halves for maximum compatibility.
                    let pieces: Vec<Pt> = if (sweep.abs() - TAU).abs() < 1e-6 {
                        vec![*center * 2.0 - cur, *end]
                    } else {
                        vec![*end]
                    };
                    for e in pieces {
                        let _ = writeln!(b, "{code}X{}Y{}I{}J{}D01*", c(e.x), c(e.y), c(center.x - cur.x), c(center.y - cur.y));
                        cur = e;
                    }
                }
            }
            cur = s.end();
        }
        if close && cur.dist(path.start) > 1e-9 {
            let _ = writeln!(b, "G01X{}Y{}D01*", c(path.start.x), c(path.start.y));
        }
    }

    fn region(&mut self, path: &Path) {
        if path.segs.is_empty() {
            return;
        }
        self.body.push_str("G36*\n");
        self.segs(path, true);
        self.body.push_str("G37*\n");
    }

    fn track(&mut self, path: &Path, width: f64) {
        self.select_circle(width);
        self.segs(path, path.closed);
    }
}

/// Does a fill contain paths nested inside one another (holes)?
fn fill_has_holes(paths: &[Path], tol: f64) -> bool {
    if paths.len() < 2 {
        return false;
    }
    let rings: Vec<Vec<Pt>> = paths.iter().map(|p| p.flatten(tol)).collect();
    (0..rings.len()).any(|i| (0..rings.len()).any(|j| i != j && rings[j].len() >= 3 && point_in_poly(rings[i][0], &rings[j])))
}

fn is_track(cap: Cap, join: Join) -> bool {
    cap == Cap::Round && join == Join::Round
}

pub fn write(layer: &Layer, opts: &Opts) -> String {
    let mut w = W { apertures: Vec::new(), by_size: HashMap::new(), body: String::new(), dark: true, cur_ap: None };

    // Non-round strokes have no direct Gerber equivalent: precompute their areas.
    let mut stroke_areas: HashMap<usize, Vec<AreaShape>> = HashMap::new();
    let mut needs_area = false;
    for (i, it) in layer.items.iter().enumerate() {
        match &it.shape {
            Shape::Stroke { cap, join, .. } if !is_track(*cap, *join) => {
                let a = area::compute(&[Item { shape: it.shape.clone(), dark: true }], opts.tol);
                needs_area |= a.iter().any(|s| !s.holes.is_empty());
                stroke_areas.insert(i, a);
            }
            Shape::Fill { paths, .. } => needs_area |= fill_has_holes(paths, opts.tol),
            _ => {}
        }
    }

    if needs_area {
        // Shapes with holes cannot be drawn item by item without the holes
        // clearing material drawn earlier, so draw the final area instead:
        // outer boundaries dark, holes clear, islands-in-holes after.
        let solid: Vec<Item> = layer.items.iter().filter(|i| !matches!(i.shape, Shape::Hairline { .. })).cloned().collect();
        for (s, _) in area::nest(area::compute(&solid, opts.tol)) {
            w.polarity(true);
            w.region(&s.outer);
            for h in &s.holes {
                w.polarity(false);
                w.region(h);
            }
        }
        w.polarity(true);
        for it in &layer.items {
            if let Shape::Hairline { path } = &it.shape {
                w.track(path, opts.line_width);
            }
        }
    } else {
        for (i, it) in layer.items.iter().enumerate() {
            w.polarity(it.dark);
            match &it.shape {
                Shape::Disc { c: ctr, r } => {
                    w.select_circle(r * 2.0);
                    let _ = writeln!(w.body, "X{}Y{}D03*", c(ctr.x), c(ctr.y));
                }
                Shape::Stroke { path, width, .. } => match stroke_areas.get(&i) {
                    Some(a) => {
                        for s in a {
                            w.region(&s.outer);
                        }
                    }
                    None => w.track(path, *width),
                },
                Shape::Fill { paths, .. } => {
                    for p in paths {
                        w.region(p);
                    }
                }
                Shape::Hairline { path } => w.track(path, opts.line_width),
            }
        }
    }

    let mut out = String::new();
    let _ = writeln!(out, "G04 Layer: {}*", layer.name.replace(['*', '%'], "_"));
    out.push_str("G04 Generated by convert*\n%FSLAX46Y46*%\n%MOMM*%\n%LPD*%\n");
    for a in &w.apertures {
        out.push_str(a);
        out.push('\n');
    }
    out.push_str("G75*\nG01*\n");
    out.push_str(&w.body);
    out.push_str("M02*\n");
    out
}
