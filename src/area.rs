//! Area computation: turns the items of a layer into the outline loops of the
//! material they cover.
//!
//! 1. Every shape is converted to polygons. Whenever an arc is faceted, the
//!    facet vertices are remembered together with the circle they lie on.
//! 2. The polygons are merged with a robust boolean engine (dark items are
//!    unioned, clear items subtracted, in drawing order).
//! 3. The resulting boundary loops are walked edge by edge. Consecutive facet
//!    edges that sit on a remembered circle are turned back into true arcs
//!    (or a full circle); collinear edges are merged into single lines.
//!    Whatever cannot be recognised stays as short line facets.

use crate::model::*;
use i_overlay::core::fill_rule::FillRule as IFill;
use i_overlay::core::overlay_rule::OverlayRule;
use i_overlay::float::simplify::SimplifyShape;
use i_overlay::float::single::SingleFloatOverlay;
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

type Contour = Vec<[f64; 2]>;

/// One connected area: an outer boundary plus holes.
#[derive(Clone, Debug)]
pub struct AreaShape {
    pub outer: Path,
    pub holes: Vec<Path>,
    /// Faceted outer boundary, handy for containment tests.
    pub outer_pts: Vec<Pt>,
}

struct Circ {
    c: Pt,
    r: f64,
    chord: f64,
}

/// Remembers which facet vertices came from which circle.
struct Tagger {
    circles: Vec<Circ>,
    index: HashMap<(i64, i64, i64), u32>,
    grid: HashMap<(i64, i64), Vec<(Pt, u32)>>,
}

/// Arc registrations collected while polygonising one item (so items can
/// be polygonised in parallel and merged into the `Tagger` afterwards).
#[derive(Default)]
struct TagLog {
    regs: Vec<(Pt, f64, Vec<Pt>, f64)>,
}

impl TagLog {
    fn add(&mut self, c: Pt, r: f64, pts: &[Pt], chord: f64) {
        self.regs.push((c, r, pts.to_vec(), chord));
    }
}

const CELL: f64 = 1e-3;
const EPS_PT: f64 = 2e-5;

fn cell(p: Pt) -> (i64, i64) {
    ((p.x / CELL).floor() as i64, (p.y / CELL).floor() as i64)
}

impl Tagger {
    fn new() -> Tagger {
        Tagger { circles: Vec::new(), index: HashMap::new(), grid: HashMap::new() }
    }

    fn add(&mut self, c: Pt, r: f64, pts: &[Pt], chord: f64) {
        if r <= 1e-9 {
            return;
        }
        let key = ((c.x * 1e6).round() as i64, (c.y * 1e6).round() as i64, (r * 1e6).round() as i64);
        let id = *self.index.entry(key).or_insert_with(|| {
            self.circles.push(Circ { c, r, chord: 0.0 });
            (self.circles.len() - 1) as u32
        });
        let circ = &mut self.circles[id as usize];
        circ.chord = circ.chord.max(chord);
        for p in pts {
            let cell = self.grid.entry(cell(*p)).or_default();
            if !cell.iter().any(|(q, i)| *i == id && q.dist(*p) < EPS_PT) {
                cell.push((*p, id));
            }
        }
    }

    fn near(&self, p: Pt, out: &mut Vec<u32>) {
        let (cx, cy) = cell(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(v) = self.grid.get(&(cx + dx, cy + dy)) {
                    for (q, id) in v {
                        if q.dist(p) < EPS_PT && !out.contains(id) {
                            out.push(*id);
                        }
                    }
                }
            }
        }
    }
}

fn to_contour(pts: &[Pt]) -> Contour {
    pts.iter().map(|p| [p.x, p.y]).collect()
}

/// Make a ring counter-clockwise; drop degenerate rings.
fn push_ccw(out: &mut Vec<Contour>, mut pts: Vec<Pt>) {
    if pts.len() < 3 {
        return;
    }
    let a = signed_area(&pts);
    if a.abs() < 1e-14 {
        return;
    }
    if a < 0.0 {
        pts.reverse();
    }
    out.push(to_contour(&pts));
}

fn circle_ring(c: Pt, r: f64, tol: f64, tg: &mut TagLog) -> Vec<Pt> {
    Path::circle(c, r).flatten_with(tol, |c, r, p, ch| tg.add(c, r, p, ch))
}

fn tangent_start(a: Pt, s: &Seg) -> Pt {
    match s {
        Seg::Line(b) => (*b - a).norm(),
        Seg::Arc { center, sweep, .. } => ((a - *center).perp() * sweep.signum()).norm(),
    }
}

fn tangent_end(a: Pt, s: &Seg) -> Pt {
    match s {
        Seg::Line(b) => (*b - a).norm(),
        Seg::Arc { end, center, sweep } => ((*end - *center).perp() * sweep.signum()).norm(),
    }
}

/// Polygons covering a stroked path. All rings are counter-clockwise so that
/// a non-zero union merges them.
fn stroke_rings(path: &Path, w: f64, cap: Cap, join: Join, tol: f64, tg: &mut TagLog, out: &mut Vec<Contour>) {
    let h = w / 2.0;
    if h <= 0.0 {
        return;
    }
    // Collect non-degenerate segments, splitting arcs into <= 90° pieces.
    let mut segs: Vec<(Pt, Seg)> = Vec::new();
    let closed_flag = path.closed;
    let add = |a: Pt, s: &Seg, segs: &mut Vec<(Pt, Seg)>| match s {
        Seg::Line(b) => {
            if a.dist(*b) > 1e-9 {
                segs.push((a, s.clone()));
            }
        }
        Seg::Arc { end, center, sweep } => {
            let r = a.dist(*center);
            if r < 1e-9 || sweep.abs() < 1e-12 {
                if a.dist(*end) > 1e-9 {
                    segs.push((a, Seg::Line(*end)));
                }
                return;
            }
            let n = (sweep.abs() / (PI / 2.0)).ceil().max(1.0) as usize;
            let a0 = (a - *center).angle();
            let mut cur = a;
            for k in 1..=n {
                let e = if k == n { *end } else { *center + Pt::polar(r, a0 + sweep * k as f64 / n as f64) };
                segs.push((cur, Seg::Arc { end: e, center: *center, sweep: sweep / n as f64 }));
                cur = e;
            }
        }
    };
    path.each_seg(|a, s| add(a, s, &mut segs));
    let end = path.end_pt();
    if closed_flag && end.dist(path.start) > 1e-9 {
        add(end, &Seg::Line(path.start), &mut segs);
    }
    let closed = closed_flag || (!segs.is_empty() && path.end_pt().dist(path.start) < 1e-9);

    if segs.is_empty() {
        match cap {
            Cap::Round => push_ccw(out, circle_ring(path.start, h, tol, tg)),
            Cap::Square => {
                let p = path.start;
                push_ccw(out, vec![p + pt(-h, -h), p + pt(h, -h), p + pt(h, h), p + pt(-h, h)]);
            }
            Cap::Butt => {}
        }
        return;
    }

    for (a, s) in &segs {
        match s {
            Seg::Line(b) => {
                let n = (*b - *a).norm().perp() * h;
                push_ccw(out, vec![*a - n, *b - n, *b + n, *a + n]);
            }
            Seg::Arc { end, center, sweep } => {
                // Annular sector. The end points are derived from the actual
                // segment ends so neighbouring pieces share them exactly.
                let r = a.dist(*center);
                let a0 = (*a - *center).angle();
                let (ua, ue) = ((*a - *center).norm(), (*end - *center).norm());
                let ring_pts = |rad: f64, nf: usize| -> Vec<Pt> {
                    (0..=nf)
                        .map(|k| match k {
                            0 => *center + ua * rad,
                            k if k == nf => *center + ue * rad,
                            k => *center + Pt::polar(rad, a0 + sweep * k as f64 / nf as f64),
                        })
                        .collect()
                };
                let ro = r + h;
                let nf = facet_count(ro, *sweep, tol);
                let mut ring = ring_pts(ro, nf);
                tg.add(*center, ro, &ring, 2.0 * ro * (sweep.abs() / nf as f64 / 2.0).sin());
                let ri = r - h;
                if ri > 1e-9 {
                    let ni = facet_count(ri, *sweep, tol);
                    let mut inner = ring_pts(ri, ni);
                    tg.add(*center, ri, &inner, 2.0 * ri * (sweep.abs() / ni as f64 / 2.0).sin());
                    inner.reverse();
                    ring.extend(inner);
                } else {
                    ring.push(*center);
                }
                push_ccw(out, ring);
            }
        }
    }

    // Joins.
    let nseg = segs.len();
    let njoin = if closed { nseg } else { nseg - 1 };
    for i in 0..njoin {
        let (a_in, s_in) = &segs[i];
        let (a_out, s_out) = &segs[(i + 1) % nseg];
        let p = *a_out;
        let t_in = tangent_end(*a_in, s_in);
        let t_out = tangent_start(*a_out, s_out);
        let cross = t_in.cross(t_out);
        let dot = t_in.dot(t_out);
        let theta = cross.atan2(dot).abs();
        // Pieces that merely touch along an edge can leave zero-width slits
        // after the boolean engine snaps coordinates, so every join piece is
        // made to overlap its neighbours.
        if theta < 0.02 {
            // (Almost) tangent: bridge the seam with a short rectangle.
            let seg_len = |a: Pt, s: &Seg| match s {
                Seg::Line(b) => a.dist(*b),
                Seg::Arc { center, sweep, .. } => a.dist(*center) * sweep.abs(),
            };
            let d = 0.25 * h.min(seg_len(*a_in, s_in)).min(seg_len(*a_out, s_out));
            let t = (t_in + t_out).norm();
            let (n, e) = (t.perp() * h, t * d);
            push_ccw(out, vec![p - e - n, p + e - n, p + e + n, p - e + n]);
            // The corner rounding/mitre here is far below any useful
            // tolerance, so no join piece is needed.
            continue;
        }
        match join {
            Join::Round => push_ccw(out, circle_ring(p, h, tol, tg)),
            Join::Bevel | Join::Miter(_) => {
                let sgn = if cross > 0.0 { -1.0 } else { 1.0 };
                let n_in = t_in.perp() * (h * sgn);
                let n_out = t_out.perp() * (h * sgn);
                // Start from a point inside the stroke on the inner side.
                let inner = if (n_in + n_out).len() > 1e-9 * h { p - (n_in + n_out).norm() * (h * 0.5) } else { p };
                let mut ring = vec![inner, p + n_in];
                if let Join::Miter(limit) = join {
                    let phi = PI - theta;
                    let ratio = 1.0 / (phi / 2.0).sin().max(1e-12);
                    if ratio <= limit {
                        ring.push(p + (n_in + n_out).norm() * (h / (theta / 2.0).cos()));
                    }
                }
                ring.push(p + n_out);
                push_ccw(out, ring);
            }
        }
    }

    // Caps.
    if !closed {
        let (a0, s0) = &segs[0];
        let (al, sl) = &segs[nseg - 1];
        let ends = [(*a0, -tangent_start(*a0, s0)), (sl.end(), tangent_end(*al, sl))];
        for (p, t) in ends {
            // `t` points outward from the stroke.
            match cap {
                Cap::Round => push_ccw(out, circle_ring(p, h, tol, tg)),
                Cap::Square => {
                    let n = t.perp() * h;
                    let e = t * h;
                    push_ccw(out, vec![p - n, p - n + e, p + n + e, p + n]);
                }
                Cap::Butt => {}
            }
        }
    }
}

fn ifill(r: FillRule) -> IFill {
    match r {
        FillRule::NonZero => IFill::NonZero,
        FillRule::EvenOdd => IFill::EvenOdd,
    }
}

fn shape_contours(shape: &Shape, tol: f64, tg: &mut TagLog, out: &mut Vec<Contour>) {
    match shape {
        Shape::Disc { c, r } => {
            if *r > 0.0 {
                push_ccw(out, circle_ring(*c, *r, tol, tg))
            }
        }
        Shape::Stroke { path, width, cap, join } => {
            // Merge the pieces of one stroke first: the global union then
            // only sees clean outlines, which is much faster.
            let mut pieces = Vec::new();
            stroke_rings(path, *width, *cap, *join, tol, tg, &mut pieces);
            if pieces.len() > 1 {
                for shape in pieces.simplify_shape(IFill::NonZero) {
                    out.extend(shape);
                }
            } else {
                out.extend(pieces);
            }
        }
        Shape::Fill { paths, rule } => {
            let rings: Vec<Contour> = paths
                .iter()
                .map(|p| {
                    let mut q = p.clone();
                    q.closed = true;
                    to_contour(&q.flatten_with(tol, |c, r, pts, ch| tg.add(c, r, pts, ch)))
                })
                .filter(|r| r.len() >= 3)
                .collect();
            if rings.is_empty() {
                return;
            }
            // Normalise so outer rings are CCW and holes CW; then a non-zero
            // union with other shapes is correct.
            for shape in rings.simplify_shape(ifill(*rule)) {
                out.extend(shape);
            }
        }
        Shape::Hairline { .. } => {}
    }
}

/// Compute the material area covered by `items` (drawn in order, honouring
/// dark/clear polarity) and return it as outline loops built from lines,
/// arcs and circles.
pub fn compute(items: &[Item], tol: f64) -> Vec<AreaShape> {
    use rayon::prelude::*;
    // Polygonise every item in parallel.
    let polys: Vec<(Vec<Contour>, TagLog)> = items
        .par_iter()
        .map(|it| {
            let mut log = TagLog::default();
            let mut out = Vec::new();
            shape_contours(&it.shape, tol, &mut log, &mut out);
            (out, log)
        })
        .collect();
    let mut tg = Tagger::new();
    for (_, log) in &polys {
        for (c, r, pts, ch) in &log.regs {
            tg.add(*c, *r, pts, *ch);
        }
    }

    // Union dark runs, subtract clear runs, in drawing order.
    let mut shapes: Vec<Vec<Contour>> = Vec::new();
    let mut i = 0;
    while i < items.len() {
        let dark = items[i].dark;
        let mut run = Vec::new();
        while i < items.len() && items[i].dark == dark {
            run.extend(polys[i].0.iter().cloned());
            i += 1;
        }
        if run.is_empty() {
            continue;
        }
        let acc: Vec<Contour> = shapes.iter().flatten().cloned().collect();
        if dark {
            shapes = if acc.is_empty() {
                run.simplify_shape(IFill::NonZero)
            } else {
                acc.overlay(&run, OverlayRule::Union, IFill::NonZero)
            };
        } else if !acc.is_empty() {
            shapes = acc.overlay(&run, OverlayRule::Difference, IFill::NonZero);
        }
    }

    // Rebuild arcs and lines.
    shapes
        .par_iter()
        .filter(|s| !s.is_empty())
        .map(|s| {
            let outer_pts: Vec<Pt> = s[0].iter().map(|p| pt(p[0], p[1])).collect();
            AreaShape {
                outer: recover(&s[0], &tg, tol),
                holes: s[1..].iter().map(|h| recover(h, &tg, tol)).collect(),
                outer_pts,
            }
        })
        .collect()
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Line,
    Arc(u32),
}

/// Rebuild a faceted closed ring into lines and arcs.
fn recover(raw: &[[f64; 2]], tg: &Tagger, tol: f64) -> Path {
    // Merge vertices that the boolean engine left almost on top of each other.
    let mut v: Vec<Pt> = Vec::with_capacity(raw.len());
    for p in raw.iter().map(|p| pt(p[0], p[1])) {
        if v.last().map_or(true, |q: &Pt| q.dist(p) > EPS_PT) {
            v.push(p);
        }
    }
    while v.len() > 3 && v[0].dist(*v.last().unwrap()) <= EPS_PT {
        v.pop();
    }
    if v.len() < 3 {
        return Path::polygon(&v);
    }
    let n = v.len();
    let on_tol = tol * 1.5 + 1e-5;

    // Classify each edge v[i] -> v[i+1].
    let mut kind = vec![Kind::Line; n];
    let mut sweep = vec![0.0; n];
    let mut cands = Vec::new();
    for i in 0..n {
        let (a, b) = (v[i], v[(i + 1) % n]);
        cands.clear();
        tg.near(a, &mut cands);
        tg.near(b, &mut cands);
        let len = a.dist(b);
        let mut best: Option<(f64, u32, f64)> = None;
        for &id in &cands {
            let c = &tg.circles[id as usize];
            let ea = (a.dist(c.c) - c.r).abs();
            let eb = (b.dist(c.c) - c.r).abs();
            if ea > on_tol || eb > on_tol || len > c.chord * 1.05 + 1e-5 {
                continue;
            }
            let s = angle_between(a - c.c, b - c.c);
            if s.abs() > PI / 2.0 + 1e-9 || s == 0.0 {
                continue;
            }
            let err = ea + eb;
            if best.map_or(true, |(e, _, _)| err < e) {
                best = Some((err, id, s));
            }
        }
        if let Some((_, id, s)) = best {
            kind[i] = Kind::Arc(id);
            sweep[i] = s;
        }
    }

    let joinable = |i: usize| -> bool {
        let j = (i + 1) % n;
        match (kind[i], kind[j]) {
            (Kind::Arc(x), Kind::Arc(y)) => x == y && sweep[i].signum() == sweep[j].signum(),
            (Kind::Line, Kind::Line) => {
                let (p, q, r) = (v[i], v[j], v[(j + 1) % n]);
                let d = (r - p).norm();
                (q - p).cross(d).abs() < 1e-6 && (q - p).dot(r - q) > 0.0
            }
            _ => false,
        }
    };

    let start = (0..n).find(|&s| !joinable((s + n - 1) % n));
    let Some(start) = start else {
        // Every edge joins the next: the ring is a single circle.
        if let Kind::Arc(id) = kind[0] {
            let c = &tg.circles[id as usize];
            let total: f64 = sweep.iter().sum();
            if (total.abs() - TAU).abs() < 0.1 {
                let p = c.c + (v[0] - c.c).norm() * c.r;
                return Path { start: p, segs: vec![Seg::Arc { end: p, center: c.c, sweep: TAU * total.signum() }], closed: true };
            }
        }
        return Path::polygon(&v);
    };

    // Group edges into runs: (first vertex, last vertex, kind, total sweep).
    let mut runs: Vec<(usize, usize, Kind, f64)> = Vec::new();
    let mut k = 0;
    while k < n {
        let i0 = (start + k) % n;
        let mut total = sweep[i0];
        let mut len = 1;
        while k + len < n && joinable((start + k + len - 1) % n) {
            total += sweep[(start + k + len) % n];
            len += 1;
        }
        runs.push((i0, (i0 + len) % n, kind[i0], total));
        k += len;
    }

    // Snap the end points of arcs exactly onto their circles. Where an arc
    // meets a line, the true intersection of the line with the circle is
    // used (so straight edges stay exactly straight); where two arcs meet,
    // the intersection of the two circles.
    let orig = v.clone();
    let nr = runs.len();
    for r in 0..nr {
        let (a0, _, ka, _) = runs[r];
        let (_, b1, kb, _) = runs[(r + 1) % nr];
        let vi = runs[r].1;
        let p = orig[vi];
        let circ = |k: Kind| match k {
            Kind::Arc(id) => Some(&tg.circles[id as usize]),
            Kind::Line => None,
        };
        let snapped = match (circ(ka), circ(kb)) {
            (Some(c), None) => line_circle(orig[b1], p, c.c, c.r, p),
            (None, Some(c)) => line_circle(orig[a0], p, c.c, c.r, p),
            (Some(c1), Some(c2)) => circle_circle(c1.c, c1.r, c2.c, c2.r, p).or_else(|| Some(c2.c + (p - c2.c).norm() * c2.r)),
            (None, None) => None,
        };
        if let Some(q) = snapped {
            if q.dist(p) <= on_tol * 4.0 {
                v[vi] = q;
            } else if let Some(c) = circ(ka).or(circ(kb)) {
                v[vi] = c.c + (p - c.c).norm() * c.r;
            }
        }
    }

    fn set_end(s: &mut Seg, p: Pt) {
        match s {
            Seg::Line(e) => *e = p,
            Seg::Arc { end, .. } => *end = p,
        }
    }

    let mut path = Path::new(v[start]);
    path.closed = true;
    let mut cur = path.start;
    for &(_, i1, kd, total) in &runs {
        let end = v[i1];
        match kd {
            Kind::Line if end.dist(cur) <= 2.0 * EPS_PT && !matches!(path.segs.last(), Some(Seg::Arc { .. })) => {
                // Sliver left over from snapping arc ends: drop it.
                match path.segs.last_mut() {
                    Some(last) => set_end(last, end),
                    None => path.start = end,
                }
            }
            Kind::Line => path.segs.push(Seg::Line(end)),
            Kind::Arc(id) => {
                // An arc must start and end exactly on its circle. Where the
                // neighbouring geometry has no exact intersection with it
                // (e.g. nearly tangent circles), bridge the tiny gap with a
                // short line facet so the loop stays closed.
                let c = &tg.circles[id as usize];
                let on = |p: Pt| c.c + (p - c.c).norm() * c.r;
                let (ps, pe) = (on(cur), on(end));
                if ps.dist(cur) > 1e-7 {
                    path.segs.push(Seg::Line(ps));
                }
                path.segs.push(Seg::Arc { end: pe, center: c.c, sweep: total });
                if pe.dist(end) > 1e-7 {
                    path.segs.push(Seg::Line(end));
                }
            }
        }
        cur = end;
    }
    let s = path.start;
    match path.segs.last_mut() {
        Some(Seg::Line(e)) => *e = s,
        Some(Seg::Arc { end, .. }) if end.dist(s) > 1e-9 => path.segs.push(Seg::Line(s)),
        _ => {}
    }
    simplify_lines(&mut path, tol * 0.5);
    path
}

/// Intersection of the infinite line through `a` and `b` with a circle,
/// nearest to `near`.
fn line_circle(a: Pt, b: Pt, c: Pt, r: f64, near: Pt) -> Option<Pt> {
    let d = b - a;
    let l2 = d.dot(d);
    if l2 < 1e-18 {
        return Some(c + (near - c).norm() * r);
    }
    // |a + t d - c|^2 = r^2
    let f = a - c;
    let (qa, qb, qc) = (l2, 2.0 * f.dot(d), f.dot(f) - r * r);
    let disc = qb * qb - 4.0 * qa * qc;
    if disc < 0.0 {
        return None;
    }
    let s = disc.sqrt();
    let p1 = a + d * ((-qb + s) / (2.0 * qa));
    let p2 = a + d * ((-qb - s) / (2.0 * qa));
    Some(if p1.dist(near) <= p2.dist(near) { p1 } else { p2 })
}

/// Intersection of two circles nearest to `near`.
fn circle_circle(c1: Pt, r1: f64, c2: Pt, r2: f64, near: Pt) -> Option<Pt> {
    let d = c1.dist(c2);
    if d < 1e-12 || d > r1 + r2 || d < (r1 - r2).abs() {
        return None;
    }
    let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h = (r1 * r1 - a * a).max(0.0).sqrt();
    let u = (c2 - c1) * (1.0 / d);
    let m = c1 + u * a;
    let (p1, p2) = (m + u.perp() * h, m - u.perp() * h);
    Some(if p1.dist(near) <= p2.dist(near) { p1 } else { p2 })
}

/// Douglas-Peucker simplification of every run of consecutive line segments
/// (facets of curves that could not be turned into arcs).
fn simplify_lines(path: &mut Path, eps: f64) {
    fn dp(pts: &[Pt], eps: f64, keep: &mut [bool]) {
        let mut stack = vec![(0usize, pts.len() - 1)];
        while let Some((a, b)) = stack.pop() {
            if b <= a + 1 {
                continue;
            }
            let (p, q) = (pts[a], pts[b]);
            let d = q - p;
            let l = d.len();
            let (mut best, mut bi) = (0.0, 0);
            for (i, r) in pts.iter().enumerate().take(b).skip(a + 1) {
                let dist = if l < 1e-12 { r.dist(p) } else { (*r - p).cross(d).abs() / l };
                if dist > best {
                    best = dist;
                    bi = i;
                }
            }
            if best > eps {
                keep[bi] = true;
                stack.push((a, bi));
                stack.push((bi, b));
            }
        }
    }
    let mut out: Vec<Seg> = Vec::with_capacity(path.segs.len());
    let mut run: Vec<Pt> = Vec::new(); // start point + line ends
    let mut cur = path.start;
    let flush = |run: &mut Vec<Pt>, out: &mut Vec<Seg>| {
        if run.len() >= 2 {
            let mut keep = vec![false; run.len()];
            keep[0] = true;
            *keep.last_mut().unwrap() = true;
            dp(run, eps, &mut keep);
            for (i, p) in run.iter().enumerate().skip(1) {
                if keep[i] {
                    out.push(Seg::Line(*p));
                }
            }
        }
        run.clear();
    };
    for s in std::mem::take(&mut path.segs) {
        match s {
            Seg::Line(e) => {
                if run.is_empty() {
                    run.push(cur);
                }
                run.push(e);
            }
            arc => {
                flush(&mut run, &mut out);
                out.push(arc);
            }
        }
        cur = out.last().map(|s| s.end()).unwrap_or(cur);
        if let Some(e) = run.last() {
            cur = *e;
        }
    }
    flush(&mut run, &mut out);
    path.segs = out;
}

/// Compute the area of `items` and express it as a single even-odd fill.
pub fn as_fill(items: &[Item], tol: f64) -> Shape {
    let mut paths = Vec::new();
    for s in compute(items, tol) {
        paths.push(s.outer);
        paths.extend(s.holes);
    }
    Shape::Fill { paths, rule: FillRule::EvenOdd }
}

/// Order area shapes so that islands inside holes come after the shapes
/// whose holes contain them. Returns (shape, nesting depth).
pub fn nest(shapes: Vec<AreaShape>) -> Vec<(AreaShape, usize)> {
    let boxes: Vec<BBox> = shapes
        .iter()
        .map(|s| {
            let mut b = BBox::empty();
            for p in &s.outer_pts {
                b.add(*p);
            }
            b
        })
        .collect();
    let mut out: Vec<(AreaShape, usize)> = Vec::new();
    let depths: Vec<usize> = (0..shapes.len())
        .map(|i| {
            let p = shapes[i].outer_pts[0];
            (0..shapes.len())
                .filter(|&j| j != i && boxes[j].contains_box(&boxes[i]) && point_in_poly(p, &shapes[j].outer_pts))
                .count()
        })
        .collect();
    for (s, d) in shapes.into_iter().zip(depths) {
        out.push((s, d));
    }
    out.sort_by_key(|(_, d)| *d);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stroke(pts: &[Pt], cap: Cap, join: Join) -> Vec<AreaShape> {
        let mut p = Path::new(pts[0]);
        for q in &pts[1..] {
            p.line_to(*q);
        }
        compute(&[Item { shape: Shape::Stroke { path: p, width: 1.0, cap, join }, dark: true }], 0.002)
    }

    #[test]
    fn gentle_polyline_is_one_shape() {
        let pts: Vec<Pt> = (0..40).map(|i| { let t = i as f64 / 39.0; pt(t * 50.0, (t * 3.0).sin() * 5.0) }).collect();
        for join in [Join::Miter(4.0), Join::Round, Join::Bevel] {
            let a = stroke(&pts, Cap::Butt, join);
            eprintln!("{join:?}: shapes={} holes={}", a.len(), a.iter().map(|s| s.holes.len()).sum::<usize>());
            assert_eq!(a.len(), 1);
            assert!(a[0].holes.is_empty());
        }
    }
}

