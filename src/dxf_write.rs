//! DXF writer (AutoCAD R12 ASCII). Every area is drawn as closed loops made of
//! LINE, ARC and CIRCLE entities, one DXF layer per input layer.

use crate::area;
use crate::model::*;
use std::fmt::Write;

fn layer_name(n: &str) -> String {
    let s: String = n
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || "-_$. ".contains(c) { c } else { '_' })
        .collect();
    if s.trim().is_empty() { "0".into() } else { s }
}

struct Ents<'a> {
    out: &'a mut String,
    layer: String,
}

impl Ents<'_> {
    fn line(&mut self, a: Pt, b: Pt) {
        if a.dist(b) < 1e-9 {
            return;
        }
        let _ = write!(
            self.out,
            "0\nLINE\n8\n{}\n10\n{:.6}\n20\n{:.6}\n30\n0.0\n11\n{:.6}\n21\n{:.6}\n31\n0.0\n",
            self.layer, a.x, a.y, b.x, b.y
        );
    }

    fn circle(&mut self, c: Pt, r: f64) {
        let _ = write!(self.out, "0\nCIRCLE\n8\n{}\n10\n{:.6}\n20\n{:.6}\n30\n0.0\n40\n{:.6}\n", self.layer, c.x, c.y, r);
    }

    fn arc(&mut self, start: Pt, end: Pt, c: Pt, sweep: f64) {
        let r = start.dist(c);
        if r < 1e-9 {
            return;
        }
        // DXF arcs always run counter-clockwise from start angle to end angle.
        let (a, b) = if sweep >= 0.0 { (start, end) } else { (end, start) };
        let a0 = (a - c).angle().to_degrees().rem_euclid(360.0);
        let a1 = (b - c).angle().to_degrees().rem_euclid(360.0);
        let _ = write!(
            self.out,
            "0\nARC\n8\n{}\n10\n{:.6}\n20\n{:.6}\n30\n0.0\n40\n{:.6}\n50\n{:.6}\n51\n{:.6}\n",
            self.layer, c.x, c.y, r, a0, a1
        );
    }

    fn path(&mut self, p: &Path) {
        if let Some((c, r)) = p.as_circle() {
            self.circle(c, r);
            return;
        }
        let mut cur = p.start;
        for s in &p.segs {
            match s {
                Seg::Line(e) => self.line(cur, *e),
                Seg::Arc { end, center, sweep } => self.arc(cur, *end, *center, *sweep),
            }
            cur = s.end();
        }
        if p.closed {
            self.line(cur, p.start);
        }
    }
}

pub fn write(doc: &Doc, opts: &Opts) -> String {
    let names: Vec<String> = doc.layers.iter().map(|l| layer_name(&l.name)).collect();
    let mut out = String::new();
    out.push_str("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n9\n$INSUNITS\n70\n4\n9\n$MEASUREMENT\n70\n1\n0\nENDSEC\n");
    out.push_str("0\nSECTION\n2\nTABLES\n");
    out.push_str("0\nTABLE\n2\nLTYPE\n70\n1\n0\nLTYPE\n2\nCONTINUOUS\n70\n0\n3\nSolid line\n72\n65\n73\n0\n40\n0.0\n0\nENDTAB\n");
    let _ = write!(out, "0\nTABLE\n2\nLAYER\n70\n{}\n", names.len());
    for (i, n) in names.iter().enumerate() {
        // ACI colours 1..6 are red, yellow, green, cyan, blue, magenta.
        let color = [1, 5, 3, 30, 6, 4, 2, 8, 140, 200][i % 10];
        let _ = write!(out, "0\nLAYER\n2\n{n}\n70\n0\n62\n{color}\n6\nCONTINUOUS\n");
    }
    out.push_str("0\nENDTAB\n0\nENDSEC\n");
    out.push_str("0\nSECTION\n2\nENTITIES\n");
    for (layer, name) in doc.layers.iter().zip(&names) {
        let mut e = Ents { out: &mut out, layer: name.clone() };
        let solid: Vec<Item> = layer.items.iter().filter(|i| !matches!(i.shape, Shape::Hairline { .. })).cloned().collect();
        for s in area::compute(&solid, opts.tol) {
            e.path(&s.outer);
            for h in &s.holes {
                e.path(h);
            }
        }
        for it in &layer.items {
            if let Shape::Hairline { path } = &it.shape {
                e.path(path);
            }
        }
    }
    out.push_str("0\nENDSEC\n0\nEOF\n");
    out
}
