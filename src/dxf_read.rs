//! DXF reader. LINE, ARC, CIRCLE, LWPOLYLINE, POLYLINE, ELLIPSE and SPLINE
//! entities are read per layer. Connected entities are chained into loops;
//! closed loops become filled areas (even-odd, so nested loops are holes),
//! open chains become zero-width hairlines.

use crate::model::*;
use anyhow::{bail, Result};
use std::collections::HashMap;
use std::f64::consts::TAU;

const JOIN_TOL: f64 = 1e-3;

type Group = (i32, String);

fn pairs(text: &str) -> Result<Vec<Group>> {
    let lines: Vec<&str> = text.lines().collect();
    let mut out = Vec::with_capacity(lines.len() / 2);
    let mut i = 0;
    while i + 1 < lines.len() {
        let code: i32 = match lines[i].trim().parse() {
            Ok(c) => c,
            Err(_) => bail!("malformed DXF: expected a group code at line {}", i + 1),
        };
        out.push((code, lines[i + 1].trim().to_string()));
        i += 2;
    }
    Ok(out)
}

struct Ent {
    kind: String,
    g: Vec<Group>,
}

impl Ent {
    fn f(&self, code: i32) -> f64 {
        self.g.iter().find(|(c, _)| *c == code).and_then(|(_, v)| v.parse().ok()).unwrap_or(0.0)
    }
    fn fo(&self, code: i32, default: f64) -> f64 {
        self.g.iter().find(|(c, _)| *c == code).and_then(|(_, v)| v.parse().ok()).unwrap_or(default)
    }
    fn layer(&self) -> String {
        self.g.iter().find(|(c, _)| *c == 8).map(|(_, v)| v.clone()).unwrap_or_else(|| "0".into())
    }
    fn all(&self, code: i32) -> Vec<f64> {
        self.g.iter().filter(|(c, _)| *c == code).filter_map(|(_, v)| v.parse().ok()).collect()
    }
    /// Object coordinate system mirrored (extrusion direction -Z)?
    fn mirrored(&self) -> bool {
        self.fo(230, 1.0) < 0.0
    }
}

fn bulge_seg(a: Pt, b: Pt, bulge: f64) -> Seg {
    if bulge.abs() < 1e-12 || a.dist(b) < 1e-12 {
        return Seg::Line(b);
    }
    let sweep = 4.0 * bulge.atan();
    let l = a.dist(b);
    let mid = (a + b) * 0.5;
    let n = (b - a).perp() * (1.0 / l);
    let center = mid + n * (l / 2.0 * (1.0 - bulge * bulge) / (2.0 * bulge));
    Seg::Arc { end: b, center, sweep }
}

fn poly_path(verts: &[(Pt, f64)], closed: bool) -> Option<Path> {
    if verts.is_empty() {
        return None;
    }
    let mut p = Path::new(verts[0].0);
    for w in verts.windows(2) {
        p.segs.push(bulge_seg(w[0].0, w[1].0, w[0].1));
    }
    if closed && verts.len() > 1 {
        let (last, b) = verts[verts.len() - 1];
        if last.dist(verts[0].0) > 1e-12 {
            p.segs.push(bulge_seg(last, verts[0].0, b));
        }
        p.closed = true;
    }
    if p.segs.is_empty() { None } else { Some(p) }
}

/// Evaluate a (possibly rational) B-spline with de Boor's algorithm.
fn spline_points(deg: usize, knots: &[f64], ctrl: &[Pt], weights: &[f64], tol: f64) -> Vec<Pt> {
    let n = ctrl.len();
    if n < 2 || knots.len() < n + deg + 1 {
        return ctrl.to_vec();
    }
    let w = |i: usize| if weights.len() == n { weights[i] } else { 1.0 };
    let (t0, t1) = (knots[deg], knots[n]);
    let len: f64 = ctrl.windows(2).map(|p| p[0].dist(p[1])).sum();
    let steps = ((len / tol.max(1e-4)).sqrt() * 2.0).clamp(16.0, 4000.0) as usize * (n / 4).max(1);
    let steps = steps.min(20000);
    (0..=steps)
        .map(|s| {
            let t = t0 + (t1 - t0) * s as f64 / steps as f64;
            let mut k = deg;
            while k < n - 1 && t >= knots[k + 1] {
                k += 1;
            }
            let mut d: Vec<(Pt, f64)> = (0..=deg).map(|j| (ctrl[j + k - deg] * w(j + k - deg), w(j + k - deg))).collect();
            for r in 1..=deg {
                for j in (r..=deg).rev() {
                    let i = j + k - deg;
                    let den = knots[i + deg + 1 - r] - knots[i];
                    let a = if den.abs() < 1e-15 { 0.0 } else { (t - knots[i]) / den };
                    d[j] = (d[j - 1].0 * (1.0 - a) + d[j].0 * a, d[j - 1].1 * (1.0 - a) + d[j].1 * a);
                }
            }
            d[deg].0 * (1.0 / d[deg].1)
        })
        .collect()
}

/// Smooth curve through the given points (centripetal Catmull-Rom).
fn catmull_rom(p: &[Pt], tol: f64) -> Vec<Pt> {
    if p.len() < 3 {
        return p.to_vec();
    }
    let n = p.len();
    let get = |i: isize| -> Pt {
        if i < 0 {
            p[0] * 2.0 - p[1]
        } else if i as usize >= n {
            p[n - 1] * 2.0 - p[n - 2]
        } else {
            p[i as usize]
        }
    };
    let mut out = vec![p[0]];
    for i in 0..n - 1 {
        let (p0, p1, p2, p3) = (get(i as isize - 1), p[i], p[i + 1], get(i as isize + 2));
        let t = |a: Pt, b: Pt| a.dist(b).sqrt().max(1e-9);
        let (t0, t1, t2, t3) = (0.0, t(p0, p1), 0.0, 0.0);
        let t2 = t1 + t(p1, p2) + t2;
        let t3 = t2 + t(p2, p3) + t3;
        let steps = ((p1.dist(p2) / tol.max(1e-4)).sqrt().ceil() as usize).clamp(4, 200);
        for k in 1..=steps {
            let u = t1 + (t2 - t1) * k as f64 / steps as f64;
            let a1 = p0 * ((t1 - u) / (t1 - t0)) + p1 * ((u - t0) / (t1 - t0));
            let a2 = p1 * ((t2 - u) / (t2 - t1)) + p2 * ((u - t1) / (t2 - t1));
            let a3 = p2 * ((t3 - u) / (t3 - t2)) + p3 * ((u - t2) / (t3 - t2));
            let b1 = a1 * ((t2 - u) / (t2 - t0)) + a2 * ((u - t0) / (t2 - t0));
            let b2 = a2 * ((t3 - u) / (t3 - t1)) + a3 * ((u - t1) / (t3 - t1));
            out.push(b1 * ((t2 - u) / (t2 - t1)) + b2 * ((u - t1) / (t2 - t1)));
        }
    }
    out
}

fn ellipse_path(e: &Ent, tol: f64) -> Option<Path> {
    let c = pt(e.f(10), e.f(20));
    let major = pt(e.f(11), e.f(21));
    let ratio = e.fo(40, 1.0);
    let t0 = e.fo(41, 0.0);
    let mut t1 = e.fo(42, TAU);
    while t1 <= t0 {
        t1 += TAU;
    }
    let full = (t1 - t0 - TAU).abs() < 1e-9;
    let r = major.len();
    if r < 1e-12 {
        return None;
    }
    if (ratio - 1.0).abs() < 1e-9 {
        let a0 = major.angle() + t0;
        if full {
            return Some(Path::circle(c, r));
        }
        let start = c + Pt::polar(r, a0);
        let mut p = Path::new(start);
        p.segs.push(Seg::Arc { end: c + Pt::polar(r, a0 + t1 - t0), center: c, sweep: t1 - t0 });
        return Some(p);
    }
    let minor = major.perp() * ratio;
    let n = facet_count(r, t1 - t0, tol).max(16);
    let pts: Vec<Pt> = (0..=n)
        .map(|k| {
            let t = t0 + (t1 - t0) * k as f64 / n as f64;
            c + major * t.cos() + minor * t.sin()
        })
        .collect();
    let mut p = Path::new(pts[0]);
    for q in &pts[1..] {
        p.line_to(*q);
    }
    p.closed = full;
    Some(p)
}

/// Join open paths whose end points meet into longer chains.
fn chain(pieces: Vec<Path>) -> Vec<Path> {
    let cellf = |p: Pt| ((p.x / (JOIN_TOL * 4.0)).floor() as i64, (p.y / (JOIN_TOL * 4.0)).floor() as i64);
    let mut grid: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, p) in pieces.iter().enumerate() {
        grid.entry(cellf(p.start)).or_default().push(i);
        grid.entry(cellf(p.end_pt())).or_default().push(i);
    }
    let mut used = vec![false; pieces.len()];
    let find = |q: Pt, used: &Vec<bool>| -> Option<(usize, bool)> {
        let (cx, cy) = cellf(q);
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(v) = grid.get(&(cx + dx, cy + dy)) {
                    for &j in v {
                        if used[j] {
                            continue;
                        }
                        if pieces[j].start.dist(q) < JOIN_TOL {
                            return Some((j, false));
                        }
                        if pieces[j].end_pt().dist(q) < JOIN_TOL {
                            return Some((j, true));
                        }
                    }
                }
            }
        }
        None
    };
    let mut out = Vec::new();
    for i in 0..pieces.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let mut path = pieces[i].clone();
        for pass in 0..2 {
            while path.end_pt().dist(path.start) >= JOIN_TOL || path.segs.is_empty() {
                let Some((j, rev)) = find(path.end_pt(), &used) else { break };
                used[j] = true;
                let next = if rev { pieces[j].reversed() } else { pieces[j].clone() };
                path.segs.extend(next.segs);
            }
            if path.end_pt().dist(path.start) < JOIN_TOL || pass == 1 {
                break;
            }
            path = path.reversed();
        }
        if path.segs.len() > 0 && path.end_pt().dist(path.start) < JOIN_TOL && (path.segs.len() > 1 || path.as_circle().is_some()) {
            // Snap the closing point exactly.
            let s = path.start;
            match path.segs.last_mut().unwrap() {
                Seg::Line(p) => *p = s,
                Seg::Arc { end, .. } => *end = s,
            }
            path.closed = true;
        }
        out.push(path);
    }
    out
}

pub fn read(text: &str, opts: &Opts) -> Result<Doc> {
    let g = pairs(text)?;
    if !g.iter().any(|(c, v)| *c == 0 && v == "SECTION") {
        bail!("not a DXF file (no SECTION found)");
    }
    // Units.
    let mut unit = 1.0;
    for w in g.windows(2) {
        if w[0] == (9, "$INSUNITS".to_string()) {
            unit = match w[1].1.parse::<i32>().unwrap_or(4) {
                1 => 25.4,
                2 => 304.8,
                5 => 10.0,
                6 => 1000.0,
                8 => 25.4e-6,
                9 => 25.4e-3,
                10 => 914.4,
                13 => 1e-3,
                14 => 100.0,
                _ => 1.0,
            };
        }
    }
    // Collect entities of the ENTITIES section.
    let mut ents: Vec<Ent> = Vec::new();
    let mut section = String::new();
    let mut i = 0;
    while i < g.len() {
        let (code, v) = &g[i];
        if *code == 0 && v == "SECTION" {
            section = g.get(i + 1).map(|x| x.1.clone()).unwrap_or_default();
            i += 2;
            continue;
        }
        if *code == 0 && v == "ENDSEC" {
            section.clear();
            i += 1;
            continue;
        }
        if *code == 0 && section == "ENTITIES" && v != "ENDSEC" {
            let mut e = Ent { kind: v.clone(), g: Vec::new() };
            i += 1;
            while i < g.len() && g[i].0 != 0 {
                e.g.push(g[i].clone());
                i += 1;
            }
            ents.push(e);
            continue;
        }
        i += 1;
    }

    let tol = opts.tol / unit;
    let mut layer_order: Vec<String> = Vec::new();
    let mut pieces: HashMap<String, (Vec<Path>, Vec<Path>)> = HashMap::new(); // (open, closed)
    let mut k = 0;
    while k < ents.len() {
        let e = &ents[k];
        k += 1;
        let layer = e.layer();
        let mirror = Affine { a: -1.0, ..Affine::IDENTITY };
        let ocs = |p: Path| if e.mirrored() { p.transform(&mirror, tol) } else { p };
        let path: Option<Path> = match e.kind.as_str() {
            "LINE" => {
                let (a, b) = (pt(e.f(10), e.f(20)), pt(e.f(11), e.f(21)));
                let mut p = Path::new(a);
                p.line_to(b);
                Some(p)
            }
            "CIRCLE" => Some(ocs(Path::circle(pt(e.f(10), e.f(20)), e.f(40)))),
            "ARC" => {
                let (c, r) = (pt(e.f(10), e.f(20)), e.f(40));
                let a0 = e.f(50).to_radians();
                let mut a1 = e.f(51).to_radians();
                while a1 <= a0 {
                    a1 += TAU;
                }
                let mut p = Path::new(c + Pt::polar(r, a0));
                p.segs.push(Seg::Arc { end: c + Pt::polar(r, a1), center: c, sweep: a1 - a0 });
                Some(ocs(p))
            }
            "LWPOLYLINE" => {
                let closed = (e.fo(70, 0.0) as i32) & 1 == 1;
                let mut verts: Vec<(Pt, f64)> = Vec::new();
                let mut x = None;
                for (c, v) in &e.g {
                    let val: f64 = v.parse().unwrap_or(0.0);
                    match c {
                        10 => x = Some(val),
                        20 => {
                            if let Some(xx) = x.take() {
                                verts.push((pt(xx, val), 0.0));
                            }
                        }
                        42 => {
                            if let Some(l) = verts.last_mut() {
                                l.1 = val;
                            }
                        }
                        _ => {}
                    }
                }
                if e.fo(43, 0.0) > 0.0 {
                    warn("LWPOLYLINE width is ignored; centre line used");
                }
                poly_path(&verts, closed).map(ocs)
            }
            "POLYLINE" => {
                let closed = (e.fo(70, 0.0) as i32) & 1 == 1;
                let mut verts = Vec::new();
                while k < ents.len() && ents[k].kind == "VERTEX" {
                    verts.push((pt(ents[k].f(10), ents[k].f(20)), ents[k].f(42)));
                    k += 1;
                }
                if k < ents.len() && ents[k].kind == "SEQEND" {
                    k += 1;
                }
                poly_path(&verts, closed).map(ocs)
            }
            "ELLIPSE" => ellipse_path(e, tol),
            "SPLINE" => {
                let deg = e.fo(71, 3.0) as usize;
                let knots = e.all(40);
                let xs = e.all(10);
                let ys = e.all(20);
                let ctrl: Vec<Pt> = xs.iter().zip(&ys).map(|(x, y)| pt(*x, *y)).collect();
                let fx = e.all(11);
                let fy = e.all(21);
                let pts = if !ctrl.is_empty() {
                    spline_points(deg, &knots, &ctrl, &e.all(41), tol)
                } else {
                    let fit: Vec<Pt> = fx.iter().zip(&fy).map(|(x, y)| pt(*x, *y)).collect();
                    if fit.len() > 2 {
                        warn("SPLINE defined only by fit points: interpolated with a Catmull-Rom curve");
                    }
                    catmull_rom(&fit, tol)
                };
                if pts.len() >= 2 {
                    let mut p = Path::new(pts[0]);
                    for q in &pts[1..] {
                        p.line_to(*q);
                    }
                    p.closed = (e.fo(70, 0.0) as i32) & 1 == 1;
                    Some(p)
                } else {
                    None
                }
            }
            "POINT" | "VERTEX" | "SEQEND" | "ATTRIB" | "VIEWPORT" => None,
            other => {
                warn(format!("DXF entity {other} is not supported; ignored"));
                None
            }
        };
        if let Some(mut p) = path {
            if unit != 1.0 {
                p = p.transform(&Affine { a: unit, d: unit, ..Affine::IDENTITY }, opts.tol);
            }
            if !layer_order.contains(&layer) {
                layer_order.push(layer.clone());
            }
            let entry = pieces.entry(layer).or_default();
            if p.closed || p.as_circle().is_some() {
                entry.1.push(p);
            } else {
                entry.0.push(p);
            }
        }
    }

    let mut doc = Doc::default();
    for name in layer_order {
        let (open, mut closed) = pieces.remove(&name).unwrap();
        let mut layer = Layer::new(name);
        if opts.dxf_lines {
            for p in closed.into_iter().chain(chain(open)) {
                layer.push(Shape::Hairline { path: p });
            }
            doc.layers.push(layer);
            continue;
        }
        for p in chain(open) {
            if p.closed {
                closed.push(p);
            } else {
                layer.push(Shape::Hairline { path: p });
            }
        }
        if !closed.is_empty() {
            layer.items.insert(0, Item { shape: Shape::Fill { paths: closed, rule: FillRule::EvenOdd }, dark: true });
        }
        doc.layers.push(layer);
    }
    Ok(doc)
}
