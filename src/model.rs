//! Format-independent geometry model shared by all readers and writers.
//!
//! All coordinates are millimetres in a right-handed, y-up frame (the Gerber
//! and DXF convention). The SVG reader/writer flip the y axis at the boundary.

use std::f64::consts::{PI, TAU};
use std::ops::{Add, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Pt {
    pub x: f64,
    pub y: f64,
}

pub const fn pt(x: f64, y: f64) -> Pt {
    Pt { x, y }
}

impl Add for Pt {
    type Output = Pt;
    fn add(self, o: Pt) -> Pt {
        pt(self.x + o.x, self.y + o.y)
    }
}
impl Sub for Pt {
    type Output = Pt;
    fn sub(self, o: Pt) -> Pt {
        pt(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for Pt {
    type Output = Pt;
    fn mul(self, k: f64) -> Pt {
        pt(self.x * k, self.y * k)
    }
}
impl Neg for Pt {
    type Output = Pt;
    fn neg(self) -> Pt {
        pt(-self.x, -self.y)
    }
}

impl Pt {
    pub fn len(self) -> f64 {
        self.x.hypot(self.y)
    }
    pub fn dist(self, o: Pt) -> f64 {
        (self - o).len()
    }
    pub fn dot(self, o: Pt) -> f64 {
        self.x * o.x + self.y * o.y
    }
    pub fn cross(self, o: Pt) -> f64 {
        self.x * o.y - self.y * o.x
    }
    /// Rotate 90° counter-clockwise.
    pub fn perp(self) -> Pt {
        pt(-self.y, self.x)
    }
    pub fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }
    pub fn norm(self) -> Pt {
        let l = self.len();
        if l > 0.0 { self * (1.0 / l) } else { pt(0.0, 0.0) }
    }
    pub fn rotate(self, a: f64) -> Pt {
        let (s, c) = a.sin_cos();
        pt(self.x * c - self.y * s, self.x * s + self.y * c)
    }
    pub fn polar(r: f64, a: f64) -> Pt {
        pt(r * a.cos(), r * a.sin())
    }
}

/// Signed angle from `a` to `b` in (-PI, PI].
pub fn angle_between(a: Pt, b: Pt) -> f64 {
    a.cross(b).atan2(a.dot(b))
}

/// Number of facets needed to keep the chord error of an arc below `tol`.
pub fn facet_count(r: f64, sweep: f64, tol: f64) -> usize {
    let r = r.abs();
    if r <= 0.0 {
        return 1;
    }
    // Maximum angle per facet for the tolerance, never coarser than 1/8 turn.
    let step = if tol >= r { PI / 4.0 } else { (2.0 * (1.0 - tol / r).acos()).min(PI / 4.0) };
    let step = step.max(1e-4);
    ((sweep.abs() / step).ceil() as usize).clamp(1, 100_000)
}

#[derive(Clone, Debug, PartialEq)]
pub enum Seg {
    Line(Pt),
    /// Circular arc to `end`, around `center`. `sweep` is the signed angle in
    /// radians (positive = counter-clockwise in a y-up frame). A sweep of ±2π
    /// with `end` equal to the start point is a full circle.
    Arc { end: Pt, center: Pt, sweep: f64 },
}

impl Seg {
    pub fn end(&self) -> Pt {
        match self {
            Seg::Line(p) => *p,
            Seg::Arc { end, .. } => *end,
        }
    }
}

/// A sequence of connected line and arc segments.
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    pub start: Pt,
    pub segs: Vec<Seg>,
    pub closed: bool,
}

impl Path {
    pub fn new(start: Pt) -> Path {
        Path { start, segs: Vec::new(), closed: false }
    }

    pub fn circle(c: Pt, r: f64) -> Path {
        let start = c + pt(r, 0.0);
        Path { start, segs: vec![Seg::Arc { end: start, center: c, sweep: TAU }], closed: true }
    }

    pub fn polygon(pts: &[Pt]) -> Path {
        let mut p = Path::new(pts[0]);
        for q in &pts[1..] {
            p.segs.push(Seg::Line(*q));
        }
        p.closed = true;
        p
    }

    pub fn end_pt(&self) -> Pt {
        self.segs.last().map(|s| s.end()).unwrap_or(self.start)
    }

    pub fn line_to(&mut self, p: Pt) {
        self.segs.push(Seg::Line(p));
    }

    /// Returns `Some((center, radius))` when the path is one full circle.
    pub fn as_circle(&self) -> Option<(Pt, f64)> {
        if self.segs.len() == 1 {
            if let Seg::Arc { center, sweep, .. } = self.segs[0] {
                if (sweep.abs() - TAU).abs() < 1e-6 {
                    return Some((center, self.start.dist(center)));
                }
            }
        }
        None
    }

    /// Visit each segment together with its start point.
    pub fn each_seg(&self, mut f: impl FnMut(Pt, &Seg)) {
        let mut cur = self.start;
        for s in &self.segs {
            f(cur, s);
            cur = s.end();
        }
    }

    pub fn reversed(&self) -> Path {
        let mut pts = vec![self.start];
        for s in &self.segs {
            pts.push(s.end());
        }
        let mut out = Path::new(*pts.last().unwrap());
        for (i, s) in self.segs.iter().enumerate().rev() {
            let to = pts[i];
            out.segs.push(match s {
                Seg::Line(_) => Seg::Line(to),
                Seg::Arc { center, sweep, .. } => Seg::Arc { end: to, center: *center, sweep: -sweep },
            });
        }
        out.closed = self.closed;
        out
    }

    /// Flatten into a polyline. Closed paths do not repeat the start point.
    /// `on_arc` is called with (center, radius, points-on-circle, max chord)
    /// for every arc so callers can remember where arcs came from.
    pub fn flatten_with(&self, tol: f64, mut on_arc: impl FnMut(Pt, f64, &[Pt], f64)) -> Vec<Pt> {
        let mut out = vec![self.start];
        let mut cur = self.start;
        for s in &self.segs {
            match s {
                Seg::Line(p) => out.push(*p),
                Seg::Arc { end, center, sweep } => {
                    let r = cur.dist(*center);
                    let n = facet_count(r, *sweep, tol);
                    let a0 = (cur - *center).angle();
                    let first = out.len() - 1;
                    for k in 1..n {
                        out.push(*center + Pt::polar(r, a0 + sweep * k as f64 / n as f64));
                    }
                    out.push(*end);
                    let chord = 2.0 * r * (sweep.abs() / n as f64 / 2.0).sin();
                    on_arc(*center, r, &out[first..], chord);
                }
            }
            cur = s.end();
        }
        if self.closed && out.len() > 1 && out[0].dist(*out.last().unwrap()) < 1e-9 {
            out.pop();
        }
        out
    }

    pub fn flatten(&self, tol: f64) -> Vec<Pt> {
        self.flatten_with(tol, |_, _, _, _| {})
    }

    /// Apply an affine transform. Arcs survive similarity transforms; under
    /// any other transform they are flattened to lines first.
    pub fn transform(&self, m: &Affine, tol: f64) -> Path {
        let sim = m.is_similarity();
        let flip = m.det() < 0.0;
        let mut out = Path::new(m.apply(self.start));
        out.closed = self.closed;
        let mut cur = self.start;
        for s in &self.segs {
            match s {
                Seg::Line(p) => out.segs.push(Seg::Line(m.apply(*p))),
                Seg::Arc { end, center, sweep } => {
                    if sim {
                        out.segs.push(Seg::Arc {
                            end: m.apply(*end),
                            center: m.apply(*center),
                            sweep: if flip { -sweep } else { *sweep },
                        });
                    } else {
                        let tmp = Path { start: cur, segs: vec![s.clone()], closed: false };
                        let pts = tmp.flatten(tol / m.scale().max(1e-12));
                        for p in &pts[1..] {
                            out.segs.push(Seg::Line(m.apply(*p)));
                        }
                    }
                }
            }
            cur = s.end();
        }
        out
    }

    pub fn translate(&self, d: Pt) -> Path {
        self.transform(&Affine::translate(d.x, d.y), 1.0)
    }

    pub fn bbox(&self, tol: f64) -> BBox {
        let mut b = BBox::empty();
        for p in self.flatten(tol) {
            b.add(p);
        }
        b
    }
}

/// 2D affine transform: (x, y) -> (a x + c y + e, b x + d y + f), as in SVG.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Affine {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
    pub e: f64,
    pub f: f64,
}

impl Affine {
    pub const IDENTITY: Affine = Affine { a: 1.0, b: 0.0, c: 0.0, d: 1.0, e: 0.0, f: 0.0 };

    pub fn translate(x: f64, y: f64) -> Affine {
        Affine { e: x, f: y, ..Affine::IDENTITY }
    }
    pub fn scale(&self) -> f64 {
        self.det().abs().sqrt()
    }
    pub fn rotate(a: f64) -> Affine {
        let (s, c) = a.sin_cos();
        Affine { a: c, b: s, c: -s, d: c, e: 0.0, f: 0.0 }
    }
    pub fn det(&self) -> f64 {
        self.a * self.d - self.b * self.c
    }
    pub fn apply(&self, p: Pt) -> Pt {
        pt(self.a * p.x + self.c * p.y + self.e, self.b * p.x + self.d * p.y + self.f)
    }
    /// `self * o`: apply `o` first, then `self`.
    pub fn then_after(&self, o: &Affine) -> Affine {
        Affine {
            a: self.a * o.a + self.c * o.b,
            b: self.b * o.a + self.d * o.b,
            c: self.a * o.c + self.c * o.d,
            d: self.b * o.c + self.d * o.d,
            e: self.a * o.e + self.c * o.f + self.e,
            f: self.b * o.e + self.d * o.f + self.f,
        }
    }
    pub fn is_similarity(&self) -> bool {
        let s1 = self.a.hypot(self.b);
        let s2 = self.c.hypot(self.d);
        let ortho = self.a * self.c + self.b * self.d;
        (s1 - s2).abs() <= 1e-9 * s1.max(s2) && ortho.abs() <= 1e-9 * s1 * s2
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BBox {
    pub min: Pt,
    pub max: Pt,
}

impl BBox {
    pub fn empty() -> BBox {
        BBox { min: pt(f64::INFINITY, f64::INFINITY), max: pt(f64::NEG_INFINITY, f64::NEG_INFINITY) }
    }
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }
    pub fn add(&mut self, p: Pt) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
    }
    pub fn grow(&mut self, d: f64) {
        self.min = self.min - pt(d, d);
        self.max = self.max + pt(d, d);
    }
    pub fn union(&mut self, o: &BBox) {
        if !o.is_empty() {
            self.add(o.min);
            self.add(o.max);
        }
    }
    pub fn contains_box(&self, o: &BBox) -> bool {
        o.min.x >= self.min.x && o.min.y >= self.min.y && o.max.x <= self.max.x && o.max.y <= self.max.y
    }
}

/// Even-odd point in polygon test.
pub fn point_in_poly(p: Pt, poly: &[Pt]) -> bool {
    let mut inside = false;
    let n = poly.len();
    let mut j = n.wrapping_sub(1);
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

pub fn signed_area(poly: &[Pt]) -> f64 {
    let n = poly.len();
    let mut s = 0.0;
    for i in 0..n {
        s += poly[i].cross(poly[(i + 1) % n]);
    }
    s / 2.0
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cap {
    Round,
    Square,
    Butt,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Join {
    Round,
    Miter(f64),
    Bevel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FillRule {
    NonZero,
    EvenOdd,
}

#[derive(Clone, Debug)]
pub enum Shape {
    /// Filled circle (a Gerber round flash, an SVG `<circle>`).
    Disc { c: Pt, r: f64 },
    /// A path drawn with a pen of the given width.
    Stroke { path: Path, width: f64, cap: Cap, join: Join },
    /// Filled area bounded by closed paths.
    Fill { paths: Vec<Path>, rule: FillRule },
    /// Zero-width geometry (open DXF lines, zero-size Gerber apertures).
    Hairline { path: Path },
}

impl Shape {
    pub fn transform(&self, m: &Affine, tol: f64) -> Shape {
        let s = m.scale();
        match self {
            Shape::Disc { c, r } if m.is_similarity() => Shape::Disc { c: m.apply(*c), r: r * s },
            Shape::Disc { c, r } => Shape::Fill { paths: vec![Path::circle(*c, *r).transform(m, tol)], rule: FillRule::NonZero },
            Shape::Stroke { path, width, cap, join } => {
                Shape::Stroke { path: path.transform(m, tol), width: width * s, cap: *cap, join: *join }
            }
            Shape::Fill { paths, rule } => Shape::Fill { paths: paths.iter().map(|p| p.transform(m, tol)).collect(), rule: *rule },
            Shape::Hairline { path } => Shape::Hairline { path: path.transform(m, tol) },
        }
    }
}

#[derive(Clone, Debug)]
pub struct Item {
    pub shape: Shape,
    /// Dark (adds material) or clear (removes material drawn before it).
    pub dark: bool,
}

#[derive(Clone, Debug)]
pub struct Layer {
    pub name: String,
    pub items: Vec<Item>,
}

impl Layer {
    pub fn new(name: impl Into<String>) -> Layer {
        Layer { name: name.into(), items: Vec::new() }
    }
    pub fn push(&mut self, shape: Shape) {
        self.items.push(Item { shape, dark: true });
    }
}

#[derive(Clone, Debug, Default)]
pub struct Doc {
    pub layers: Vec<Layer>,
}

/// Conversion options shared by all stages.
#[derive(Clone, Copy, Debug)]
pub struct Opts {
    /// Maximum chord error (mm) when arcs must be approximated by facets.
    pub tol: f64,
    /// Pen width (mm) used to draw zero-width geometry in SVG and Gerber.
    pub line_width: f64,
    /// Read DXF geometry as lines (hairlines) instead of filled areas.
    pub dxf_lines: bool,
}

pub fn shape_bbox(s: &Shape, tol: f64) -> BBox {
    match s {
        Shape::Disc { c, r } => BBox { min: *c - pt(*r, *r), max: *c + pt(*r, *r) },
        Shape::Stroke { path, width, .. } => {
            let mut b = path.bbox(tol);
            b.grow(width * 0.75);
            b
        }
        Shape::Fill { paths, .. } => {
            let mut b = BBox::empty();
            for p in paths {
                b.union(&p.bbox(tol));
            }
            b
        }
        Shape::Hairline { path } => path.bbox(tol),
    }
}

// ---------------------------------------------------------------------------
// Warnings: collected and deduplicated, printed once at the end.

use std::collections::BTreeMap;
use std::sync::Mutex;

static WARNINGS: Mutex<BTreeMap<String, usize>> = Mutex::new(BTreeMap::new());

pub fn warn(msg: impl Into<String>) {
    *WARNINGS.lock().unwrap().entry(msg.into()).or_insert(0) += 1;
}

pub fn print_warnings() {
    for (m, n) in WARNINGS.lock().unwrap().iter() {
        if *n > 1 {
            eprintln!("warning: {m} ({n} times)");
        } else {
            eprintln!("warning: {m}");
        }
    }
}
