//! Gerber (RS-274X / X2) reader.

use crate::area;
use crate::model::*;
use anyhow::{bail, Result};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

#[derive(Clone, Debug)]
enum Ap {
    Circle { d: f64, hole: f64 },
    Rect { w: f64, h: f64, hole: f64 },
    Obround { w: f64, h: f64, hole: f64 },
    Poly { d: f64, n: usize, rot: f64, hole: f64 },
    /// Macro primitives already evaluated, in mm, relative to the flash point.
    Macro(Vec<Item>),
}

#[derive(Clone, Copy, PartialEq)]
enum Interp {
    Linear,
    Cw,
    Ccw,
}

struct Reader {
    tol: f64,
    int_digits: u32,
    dec_digits: u32,
    trailing: bool,
    incremental: bool,
    unit: f64,
    cur: Pt,
    interp: Interp,
    multi: bool,
    ap: Option<i32>,
    aps: HashMap<i32, Ap>,
    macros: HashMap<String, Vec<String>>,
    dark: bool,
    last_d: Option<i32>,
    region: Option<(Vec<Path>, Option<Path>)>,
    /// Track being built: path + aperture number + polarity.
    track: Option<(Path, i32, bool)>,
    sr: Option<(usize, usize, f64, f64, usize)>,
    /// Deprecated whole-image transforms: mirror (MI), scale (SF), rotation (IR), offset (OF).
    mirror: (bool, bool),
    scale: (f64, f64),
    rotation: f64,
    offset: Pt,
    items: Vec<Item>,
}

enum Tok {
    Block(Vec<String>),
    Word(String),
}

fn tokenize(text: &str) -> Vec<Tok> {
    let mut out = Vec::new();
    let mut chars = text.chars().peekable();
    let mut word = String::new();
    while let Some(c) = chars.next() {
        // Inside a G04 comment a '%' is just text (Eagle writes X2 attributes
        // as "G04 #@! %TF...*").
        let in_comment = {
            let w = word.trim_start();
            w.starts_with("G04") || w.starts_with("G4 ")
        };
        match c {
            '%' if !in_comment => {
                let mut cmds = Vec::new();
                let mut cur = String::new();
                for c in chars.by_ref() {
                    match c {
                        '%' => break,
                        '*' => {
                            cmds.push(std::mem::take(&mut cur));
                        }
                        '\r' | '\n' => {}
                        _ => cur.push(c),
                    }
                }
                if !cur.trim().is_empty() {
                    cmds.push(cur);
                }
                out.push(Tok::Block(cmds));
            }
            '*' => out.push(Tok::Word(std::mem::take(&mut word))),
            '\r' | '\n' => {}
            _ => word.push(c),
        }
    }
    out
}

/// Evaluate an aperture-macro arithmetic expression.
fn eval(expr: &str, vars: &HashMap<usize, f64>) -> Result<f64> {
    let toks: Vec<char> = expr.chars().filter(|c| !c.is_whitespace()).collect();
    let mut pos = 0;
    fn num(t: &[char], pos: &mut usize, vars: &HashMap<usize, f64>) -> Result<f64> {
        let c = *t.get(*pos).ok_or_else(|| anyhow::anyhow!("unexpected end of macro expression"))?;
        if c == '-' || c == '+' {
            *pos += 1;
            let v = num(t, pos, vars)?;
            return Ok(if c == '-' { -v } else { v });
        }
        if c == '(' {
            *pos += 1;
            let v = sum(t, pos, vars)?;
            if t.get(*pos) == Some(&')') {
                *pos += 1;
            }
            return Ok(v);
        }
        if c == '$' {
            *pos += 1;
            let s = *pos;
            while *pos < t.len() && t[*pos].is_ascii_digit() {
                *pos += 1;
            }
            let n: usize = t[s..*pos].iter().collect::<String>().parse()?;
            return Ok(*vars.get(&n).unwrap_or(&0.0));
        }
        let s = *pos;
        while *pos < t.len() && (t[*pos].is_ascii_digit() || t[*pos] == '.') {
            *pos += 1;
        }
        let txt: String = t[s..*pos].iter().collect();
        if txt.is_empty() {
            bail!("bad macro expression near '{}'", t[s..].iter().collect::<String>());
        }
        Ok(txt.parse()?)
    }
    fn prod(t: &[char], pos: &mut usize, vars: &HashMap<usize, f64>) -> Result<f64> {
        let mut v = num(t, pos, vars)?;
        while let Some(&c) = t.get(*pos) {
            if c == 'x' || c == 'X' {
                *pos += 1;
                v *= num(t, pos, vars)?;
            } else if c == '/' {
                *pos += 1;
                v /= num(t, pos, vars)?;
            } else {
                break;
            }
        }
        Ok(v)
    }
    fn sum(t: &[char], pos: &mut usize, vars: &HashMap<usize, f64>) -> Result<f64> {
        let mut v = prod(t, pos, vars)?;
        while let Some(&c) = t.get(*pos) {
            if c == '+' {
                *pos += 1;
                v += prod(t, pos, vars)?;
            } else if c == '-' {
                *pos += 1;
                v -= prod(t, pos, vars)?;
            } else {
                break;
            }
        }
        Ok(v)
    }
    sum(&toks, &mut pos, vars)
}

fn rect_poly(c: Pt, w: f64, h: f64) -> Path {
    Path::polygon(&[c + pt(-w / 2.0, -h / 2.0), c + pt(w / 2.0, -h / 2.0), c + pt(w / 2.0, h / 2.0), c + pt(-w / 2.0, h / 2.0)])
}

fn obround(w: f64, h: f64) -> Path {
    if (w - h).abs() < 1e-12 {
        return Path::circle(pt(0.0, 0.0), w / 2.0);
    }
    let horizontal = w > h;
    let (len, r) = if horizontal { (w, h / 2.0) } else { (h, w / 2.0) };
    let k = len / 2.0 - r;
    // Build horizontally, rotate if vertical.
    let mut p = Path::new(pt(-k, -r));
    p.line_to(pt(k, -r));
    p.segs.push(Seg::Arc { end: pt(k, r), center: pt(k, 0.0), sweep: PI });
    p.line_to(pt(-k, r));
    p.segs.push(Seg::Arc { end: pt(-k, -r), center: pt(-k, 0.0), sweep: PI });
    p.closed = true;
    if horizontal { p } else { p.transform(&Affine::rotate(PI / 2.0), 1.0) }
}

fn reg_poly(c: Pt, d: f64, n: usize, rot_deg: f64) -> Path {
    let n = n.max(3);
    let pts: Vec<Pt> = (0..n)
        .map(|k| c + Pt::polar(d / 2.0, rot_deg.to_radians() + TAU * k as f64 / n as f64))
        .collect();
    Path::polygon(&pts)
}

fn convex_hull(mut pts: Vec<Pt>) -> Vec<Pt> {
    pts.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap().then(a.y.partial_cmp(&b.y).unwrap()));
    pts.dedup_by(|a, b| a.dist(*b) < 1e-12);
    if pts.len() < 3 {
        return pts;
    }
    let mut lower: Vec<Pt> = Vec::new();
    for p in &pts {
        while lower.len() >= 2 && (lower[lower.len() - 1] - lower[lower.len() - 2]).cross(*p - lower[lower.len() - 2]) <= 0.0 {
            lower.pop();
        }
        lower.push(*p);
    }
    let mut upper: Vec<Pt> = Vec::new();
    for p in pts.iter().rev() {
        while upper.len() >= 2 && (upper[upper.len() - 1] - upper[upper.len() - 2]).cross(*p - upper[upper.len() - 2]) <= 0.0 {
            upper.pop();
        }
        upper.push(*p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Parse "A<num>B<num>" as used by the deprecated MI/SF/OF commands.
fn ab_values(s: &str) -> (Option<f64>, Option<f64>) {
    let num = |k: char| -> Option<f64> {
        let i = s.find(k)?;
        let rest = &s[i + 1..];
        let end = rest.find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+')).unwrap_or(rest.len());
        rest[..end].parse().ok()
    };
    (num('A'), num('B'))
}

impl Reader {
    fn image_transform(&self) -> Affine {
        let sx = self.scale.0 * if self.mirror.0 { -1.0 } else { 1.0 };
        let sy = self.scale.1 * if self.mirror.1 { -1.0 } else { 1.0 };
        let m = Affine { a: sx, d: sy, ..Affine::IDENTITY };
        let m = Affine::rotate(self.rotation.to_radians()).then_after(&m);
        Affine::translate(self.offset.x, self.offset.y).then_after(&m)
    }

    fn coord(&self, s: &str) -> Result<f64> {
        if s.contains('.') {
            return Ok(s.parse::<f64>()? * self.unit);
        }
        let neg = s.starts_with('-');
        let digits = s.trim_start_matches(['-', '+']);
        let mut d = digits.to_string();
        if self.trailing {
            while (d.len() as u32) < self.int_digits + self.dec_digits {
                d.push('0');
            }
        }
        let v: f64 = if d.is_empty() { 0.0 } else { d.parse::<i64>()? as f64 };
        let v = v / 10f64.powi(self.dec_digits as i32) * self.unit;
        Ok(if neg { -v } else { v })
    }

    fn num(&self, s: &str) -> Result<f64> {
        Ok(s.trim().parse::<f64>()? * self.unit)
    }

    fn push(&mut self, shape: Shape) {
        self.items.push(Item { shape, dark: self.dark });
    }

    fn flush_track(&mut self) {
        if let Some((path, ap, dark)) = self.track.take() {
            let d = match self.aps.get(&ap) {
                Some(Ap::Circle { d, .. }) => *d,
                _ => 0.0,
            };
            let shape = if d <= 0.0 {
                Shape::Hairline { path }
            } else {
                Shape::Stroke { path, width: d, cap: Cap::Round, join: Join::Round }
            };
            self.items.push(Item { shape, dark });
        }
    }

    fn close_contour(&mut self) {
        if let Some((contours, cur)) = &mut self.region {
            if let Some(mut p) = cur.take() {
                if !p.segs.is_empty() {
                    p.closed = true;
                    contours.push(p);
                }
            }
        }
    }

    fn arc_seg(&self, from: Pt, to: Pt, i: f64, j: f64) -> Seg {
        let cw = self.interp == Interp::Cw;
        let sweep_for = |c: Pt| -> f64 {
            if from.dist(to) < 1e-9 {
                return if cw { -TAU } else { TAU };
            }
            let mut s = (to - c).angle() - (from - c).angle();
            if cw {
                while s >= 0.0 {
                    s -= TAU;
                }
                while s < -TAU {
                    s += TAU;
                }
            } else {
                while s <= 0.0 {
                    s += TAU;
                }
                while s > TAU {
                    s -= TAU;
                }
            }
            s
        };
        if self.multi {
            let c = from + pt(i, j);
            return Seg::Arc { end: to, center: c, sweep: sweep_for(c) };
        }
        // Single quadrant: I/J are unsigned; pick the centre that gives an
        // arc of at most 90 degrees with matching radii.
        let mut best: Option<(f64, Pt, f64)> = None;
        for (si, sj) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
            let c = from + pt(si * i.abs(), sj * j.abs());
            let mut s = sweep_for(c);
            if from.dist(to) < 1e-9 {
                s = 0.0;
            }
            if s.abs() > PI / 2.0 + 1e-3 {
                continue;
            }
            let err = (from.dist(c) - to.dist(c)).abs();
            if best.map_or(true, |(e, _, _)| err < e) {
                best = Some((err, c, s));
            }
        }
        match best {
            Some((_, c, s)) if s != 0.0 => Seg::Arc { end: to, center: c, sweep: s },
            _ => Seg::Line(to),
        }
    }

    fn flash(&mut self, p: Pt) {
        let Some(id) = self.ap else {
            warn("flash without a selected aperture ignored");
            return;
        };
        let Some(ap) = self.aps.get(&id).cloned() else {
            warn(format!("undefined aperture D{id}"));
            return;
        };
        let with_hole = |outline: Path, hole: f64| -> Shape {
            let mut paths = vec![outline.translate(p)];
            if hole > 0.0 {
                paths.push(Path::circle(p, hole / 2.0));
            }
            Shape::Fill { paths, rule: FillRule::EvenOdd }
        };
        match ap {
            Ap::Circle { d, hole } if hole <= 0.0 => self.push(Shape::Disc { c: p, r: d / 2.0 }),
            Ap::Circle { d, hole } => self.push(with_hole(Path::circle(pt(0.0, 0.0), d / 2.0), hole)),
            Ap::Rect { w, h, hole } => self.push(with_hole(rect_poly(pt(0.0, 0.0), w, h), hole)),
            Ap::Obround { w, h, hole } => self.push(with_hole(obround(w, h), hole)),
            Ap::Poly { d, n, rot, hole } => self.push(with_hole(reg_poly(pt(0.0, 0.0), d, n, rot), hole)),
            Ap::Macro(prims) => {
                let m = Affine::translate(p.x, p.y);
                if prims.iter().all(|i| i.dark) {
                    for it in &prims {
                        self.push(it.shape.transform(&m, self.tol));
                    }
                } else {
                    let f = area::as_fill(&prims, self.tol);
                    self.push(f.transform(&m, self.tol));
                }
            }
        }
    }

    /// Draw a segment with the current aperture.
    fn interpolate(&mut self, from: Pt, seg: Seg) {
        let Some(id) = self.ap else {
            warn("draw without a selected aperture ignored");
            return;
        };
        match self.aps.get(&id).cloned() {
            Some(Ap::Circle { .. }) => {
                let dark = self.dark;
                match &mut self.track {
                    Some((p, a, d)) if *a == id && *d == dark && p.end_pt().dist(from) < 1e-9 => p.segs.push(seg),
                    _ => {
                        self.flush_track();
                        let mut p = Path::new(from);
                        p.segs.push(seg);
                        self.track = Some((p, id, dark));
                    }
                }
            }
            Some(ap) => {
                self.flush_track();
                let outline = match &ap {
                    Ap::Rect { w, h, .. } => rect_poly(pt(0.0, 0.0), *w, *h),
                    Ap::Obround { w, h, .. } => obround(*w, *h),
                    Ap::Poly { d, n, rot, .. } => reg_poly(pt(0.0, 0.0), *d, *n, *rot),
                    _ => {
                        warn("drawing with an aperture macro is not allowed by the Gerber spec; ignored");
                        return;
                    }
                };
                match seg {
                    Seg::Line(to) => {
                        let o = outline.flatten(self.tol);
                        let mut pts: Vec<Pt> = o.iter().map(|q| *q + from).collect();
                        pts.extend(o.iter().map(|q| *q + to));
                        let hull = convex_hull(pts);
                        if hull.len() >= 3 {
                            self.push(Shape::Fill { paths: vec![Path::polygon(&hull)], rule: FillRule::NonZero });
                        }
                    }
                    Seg::Arc { .. } => {
                        warn("arc drawn with a non-circular aperture approximated with a round pen");
                        let b = outline.bbox(self.tol);
                        let w = (b.max.x - b.min.x).min(b.max.y - b.min.y);
                        let mut p = Path::new(from);
                        p.segs.push(seg);
                        self.push(Shape::Stroke { path: p, width: w, cap: Cap::Square, join: Join::Round });
                    }
                }
            }
            None => warn(format!("undefined aperture D{id}")),
        }
    }

    fn macro_prims(&self, body: &[String], params: &[f64]) -> Result<Vec<Item>> {
        let mut vars: HashMap<usize, f64> = params.iter().enumerate().map(|(i, v)| (i + 1, *v)).collect();
        let mut raw: Vec<Item> = Vec::new();
        for stmt in body {
            let s = stmt.trim();
            if s.is_empty() || s == "0" || s.starts_with("0 ") || s.starts_with("0,") {
                continue;
            }
            if s.starts_with('$') {
                if let Some((lhs, rhs)) = s.split_once('=') {
                    let n: usize = lhs[1..].trim().parse()?;
                    vars.insert(n, eval(rhs, &vars)?);
                    continue;
                }
            }
            let v: Vec<f64> = s.split(',').map(|e| eval(e, &vars)).collect::<Result<_>>()?;
            let get = |i: usize| v.get(i).copied().unwrap_or(0.0);
            let code = get(0) as i32;
            let dark = get(1) != 0.0;
            let rot = |p: &Path, deg: f64| p.transform(&Affine::rotate(deg.to_radians()), 1.0);
            match code {
                1 => {
                    let c = pt(get(3), get(4)).rotate(get(5).to_radians());
                    raw.push(Item { shape: Shape::Disc { c, r: get(2) / 2.0 }, dark });
                }
                2 | 20 => {
                    let (w, a, b) = (get(2), pt(get(3), get(4)), pt(get(5), get(6)));
                    let n = (b - a).norm().perp() * (w / 2.0);
                    if a.dist(b) > 0.0 {
                        let p = Path::polygon(&[a - n, b - n, b + n, a + n]);
                        raw.push(Item { shape: Shape::Fill { paths: vec![rot(&p, get(7))], rule: FillRule::NonZero }, dark });
                    }
                }
                21 => {
                    let p = rect_poly(pt(get(4), get(5)), get(2), get(3));
                    raw.push(Item { shape: Shape::Fill { paths: vec![rot(&p, get(6))], rule: FillRule::NonZero }, dark });
                }
                22 => {
                    let (w, h) = (get(2), get(3));
                    let p = rect_poly(pt(get(4) + w / 2.0, get(5) + h / 2.0), w, h);
                    raw.push(Item { shape: Shape::Fill { paths: vec![rot(&p, get(6))], rule: FillRule::NonZero }, dark });
                }
                4 => {
                    let n = get(2) as usize;
                    let pts: Vec<Pt> = (0..=n).map(|k| pt(get(3 + 2 * k), get(4 + 2 * k))).collect();
                    let r = get(5 + 2 * n);
                    if pts.len() >= 3 {
                        let p = Path::polygon(&pts[..pts.len() - 1]);
                        raw.push(Item { shape: Shape::Fill { paths: vec![rot(&p, r)], rule: FillRule::NonZero }, dark });
                    }
                }
                5 => {
                    let p = reg_poly(pt(get(3), get(4)), get(5), get(2) as usize, 0.0);
                    raw.push(Item { shape: Shape::Fill { paths: vec![rot(&p, get(6))], rule: FillRule::NonZero }, dark });
                }
                7 => {
                    // Thermal: centre, outer dia, inner dia, gap, rotation.
                    let (c, od, id, gap, r) = (pt(get(1), get(2)), get(3), get(4), get(5), get(6));
                    let parts = vec![
                        Item { shape: Shape::Disc { c, r: od / 2.0 }, dark: true },
                        Item { shape: Shape::Disc { c, r: id / 2.0 }, dark: false },
                        Item { shape: Shape::Fill { paths: vec![rect_poly(c, od * 1.1, gap)], rule: FillRule::NonZero }, dark: false },
                        Item { shape: Shape::Fill { paths: vec![rect_poly(c, gap, od * 1.1)], rule: FillRule::NonZero }, dark: false },
                    ];
                    let f = area::as_fill(&parts, self.tol / self.unit.max(1e-9));
                    raw.push(Item { shape: f.transform(&Affine::rotate(r.to_radians()), 1.0), dark: true });
                }
                6 => warn("moire aperture macro primitive is not supported; ignored"),
                _ => warn(format!("unknown aperture macro primitive {code}; ignored")),
            }
        }
        let m = Affine { a: self.unit, d: self.unit, ..Affine::IDENTITY };
        Ok(raw.into_iter().map(|i| Item { shape: i.shape.transform(&m, self.tol), dark: i.dark }).collect())
    }

    fn aperture_def(&mut self, s: &str) -> Result<()> {
        // ADD<n><name>[,<p>X<p>...]
        let body = &s[3..];
        let ndig = body.chars().take_while(|c| c.is_ascii_digit()).count();
        let id: i32 = body[..ndig].parse()?;
        let rest = &body[ndig..];
        let (name, params) = match rest.split_once(',') {
            Some((n, p)) => (n, p.split('X').map(|x| x.trim().parse::<f64>()).collect::<Result<Vec<_>, _>>()?),
            None => (rest, vec![]),
        };
        let g = |i: usize| params.get(i).copied().unwrap_or(0.0) * self.unit;
        let ap = match name {
            "C" => Ap::Circle { d: g(0), hole: g(1) },
            "R" => Ap::Rect { w: g(0), h: g(1), hole: g(2) },
            "O" => Ap::Obround { w: g(0), h: g(1), hole: g(2) },
            "P" => Ap::Poly { d: g(0), n: params.get(1).copied().unwrap_or(3.0) as usize, rot: params.get(2).copied().unwrap_or(0.0), hole: g(3) },
            m => match self.macros.get(m) {
                Some(body) => Ap::Macro(self.macro_prims(&body.clone(), &params)?),
                None => {
                    warn(format!("aperture D{id} uses unknown macro '{m}'"));
                    return Ok(());
                }
            },
        };
        self.aps.insert(id, ap);
        Ok(())
    }

    fn step_repeat_close(&mut self) {
        self.flush_track();
        if let Some((nx, ny, dx, dy, start)) = self.sr.take() {
            let block: Vec<Item> = self.items[start..].to_vec();
            for iy in 0..ny {
                for ix in 0..nx {
                    if ix == 0 && iy == 0 {
                        continue;
                    }
                    let m = Affine::translate(dx * ix as f64, dy * iy as f64);
                    for it in &block {
                        self.items.push(Item { shape: it.shape.transform(&m, self.tol), dark: it.dark });
                    }
                }
            }
        }
    }

    fn extended(&mut self, cmds: Vec<String>) -> Result<()> {
        let Some(first) = cmds.first() else { return Ok(()) };
        let first = first.trim().to_string();
        if let Some(name) = first.strip_prefix("AM") {
            self.macros.insert(name.to_string(), cmds[1..].to_vec());
            return Ok(());
        }
        for c in &cmds {
            let c = c.trim();
            if c.len() < 2 {
                continue;
            }
            match &c[..2] {
                "FS" => {
                    let b = c.as_bytes();
                    self.trailing = b.get(2) == Some(&b'T');
                    self.incremental = b.get(3) == Some(&b'I');
                    if self.incremental {
                        warn("incremental coordinates are deprecated; supported on a best-effort basis");
                    }
                    if let Some(xi) = c.find('X') {
                        let d: Vec<u32> = c[xi + 1..].chars().take(2).filter_map(|ch| ch.to_digit(10)).collect();
                        if d.len() == 2 {
                            self.int_digits = d[0];
                            self.dec_digits = d[1];
                        }
                    }
                }
                "MO" => self.unit = if c.starts_with("MOIN") { 25.4 } else { 1.0 },
                "AD" => self.aperture_def(c)?,
                "LP" => {
                    self.flush_track();
                    self.dark = !c.starts_with("LPC");
                }
                "SR" => {
                    self.step_repeat_close();
                    let mut nx = 1;
                    let mut ny = 1;
                    let (mut dx, mut dy) = (0.0, 0.0);
                    let mut rest = &c[2..];
                    while !rest.is_empty() {
                        let k = rest.chars().next().unwrap();
                        let len = rest[1..].find(|ch: char| ch.is_ascii_alphabetic()).map(|i| i + 1).unwrap_or(rest.len());
                        let val = &rest[1..len];
                        match k {
                            'X' => nx = val.parse().unwrap_or(1),
                            'Y' => ny = val.parse().unwrap_or(1),
                            'I' => dx = self.num(val)?,
                            'J' => dy = self.num(val)?,
                            _ => {}
                        }
                        rest = &rest[len..];
                    }
                    if nx > 1 || ny > 1 {
                        self.sr = Some((nx, ny, dx, dy, self.items.len()));
                    }
                }
                "MI" => {
                    let a = c.find('A').map(|i| c[i + 1..].starts_with('1')).unwrap_or(false);
                    let b = c.find('B').map(|i| c[i + 1..].starts_with('1')).unwrap_or(false);
                    self.mirror = (a, b);
                }
                "SF" => {
                    let (a, b) = ab_values(&c[2..]);
                    self.scale = (a.unwrap_or(1.0), b.unwrap_or(1.0));
                }
                "OF" => {
                    let (a, b) = ab_values(&c[2..]);
                    self.offset = pt(a.unwrap_or(0.0) * self.unit, b.unwrap_or(0.0) * self.unit);
                }
                "IR" => self.rotation = c[2..].trim().parse().unwrap_or(0.0),
                "IP" => {
                    if c.contains("NEG") {
                        warn("negative image polarity (%IPNEG%) is not supported; image drawn positive");
                    }
                }
                "LM" if !c.ends_with('N') => warn("aperture mirroring (%LM%) is not supported; ignored"),
                "LR" if c[2..].parse::<f64>().unwrap_or(0.0) != 0.0 => warn("aperture rotation (%LR%) is not supported; ignored"),
                "LS" if c[2..].parse::<f64>().unwrap_or(1.0) != 1.0 => warn("aperture scaling (%LS%) is not supported; ignored"),
                _ => {} // attributes (TF/TA/TO/TD), IN, LN, OF, SF, AS, IR...
            }
        }
        Ok(())
    }

    fn word(&mut self, w: &str) -> Result<bool> {
        let w = w.trim();
        if w.is_empty() || w.starts_with("G04") || w.starts_with("G4 ") {
            return Ok(true);
        }
        // Split into letter/number fields.
        let mut fields: Vec<(char, &str)> = Vec::new();
        let b = w.as_bytes();
        let mut i = 0;
        while i < b.len() {
            let c = b[i] as char;
            if !c.is_ascii_alphabetic() {
                i += 1;
                continue;
            }
            let s = i + 1;
            let mut e = s;
            while e < b.len() && !(b[e] as char).is_ascii_alphabetic() {
                e += 1;
            }
            fields.push((c.to_ascii_uppercase(), w[s..e].trim()));
            i = e;
        }
        let (mut x, mut y, mut ii, mut jj, mut d) = (None, None, None, None, None);
        for (c, v) in &fields {
            match c {
                'G' => match v.parse::<i32>().unwrap_or(-1) {
                    1 => self.interp = Interp::Linear,
                    2 => self.interp = Interp::Cw,
                    3 => self.interp = Interp::Ccw,
                    36 => {
                        self.flush_track();
                        self.region = Some((Vec::new(), None));
                    }
                    37 => {
                        self.close_contour();
                        if let Some((contours, _)) = self.region.take() {
                            for p in contours {
                                self.push(Shape::Fill { paths: vec![p], rule: FillRule::NonZero });
                            }
                        }
                    }
                    74 => self.multi = false,
                    75 => self.multi = true,
                    70 => self.unit = 25.4,
                    71 => self.unit = 1.0,
                    90 => self.incremental = false,
                    91 => self.incremental = true,
                    _ => {}
                },
                'X' => x = Some(self.coord(v)?),
                'Y' => y = Some(self.coord(v)?),
                'I' => ii = Some(self.coord(v)?),
                'J' => jj = Some(self.coord(v)?),
                'D' => d = Some(v.parse::<i32>()?),
                'M' => {
                    if matches!(v.parse::<i32>().unwrap_or(-1), 0 | 2 | 30) {
                        return Ok(false);
                    }
                }
                _ => {}
            }
        }
        if let Some(n) = d {
            if n >= 10 {
                self.flush_track();
                self.ap = Some(n);
                return Ok(true);
            }
        }
        let has_xy = x.is_some() || y.is_some();
        let op = match d {
            Some(n) => {
                self.last_d = Some(n);
                Some(n)
            }
            None if has_xy => self.last_d,
            None => None,
        };
        let Some(op) = op else { return Ok(true) };
        let target = if self.incremental {
            self.cur + pt(x.unwrap_or(0.0), y.unwrap_or(0.0))
        } else {
            pt(x.unwrap_or(self.cur.x), y.unwrap_or(self.cur.y))
        };
        let from = self.cur;
        match op {
            1 => {
                let seg = if self.interp == Interp::Linear {
                    Seg::Line(target)
                } else {
                    self.arc_seg(from, target, ii.unwrap_or(0.0), jj.unwrap_or(0.0))
                };
                if let Some((_, cur)) = &mut self.region {
                    cur.get_or_insert_with(|| Path::new(from)).segs.push(seg);
                } else {
                    self.interpolate(from, seg);
                }
            }
            2 => {
                if self.region.is_some() {
                    self.close_contour();
                } else {
                    self.flush_track();
                }
            }
            3 => {
                self.flush_track();
                self.flash(target);
            }
            _ => {}
        }
        self.cur = target;
        Ok(true)
    }
}

pub fn read(text: &str, name: &str, opts: &Opts) -> Result<Layer> {
    let mut r = Reader {
        tol: opts.tol,
        int_digits: 2,
        dec_digits: 4,
        trailing: false,
        incremental: false,
        unit: 25.4,
        cur: pt(0.0, 0.0),
        interp: Interp::Linear,
        multi: true,
        ap: None,
        aps: HashMap::new(),
        macros: HashMap::new(),
        dark: true,
        last_d: None,
        region: None,
        track: None,
        sr: None,
        mirror: (false, false),
        scale: (1.0, 1.0),
        rotation: 0.0,
        offset: pt(0.0, 0.0),
        items: Vec::new(),
    };
    let mut seen_fs = false;
    for tok in tokenize(text) {
        match tok {
            Tok::Block(cmds) => {
                if cmds.iter().any(|c| c.trim_start().starts_with("FS")) {
                    seen_fs = true;
                }
                r.extended(cmds)?;
            }
            Tok::Word(w) => {
                if !r.word(&w)? {
                    break;
                }
            }
        }
    }
    if !seen_fs {
        bail!("'{name}' does not look like an RS-274X Gerber file (no %FS% format statement)");
    }
    r.flush_track();
    r.step_repeat_close();
    let m = r.image_transform();
    if m != Affine::IDENTITY {
        let tol = r.tol;
        for it in &mut r.items {
            it.shape = it.shape.transform(&m, tol);
        }
    }
    let mut layer = Layer::new(name);
    layer.items = r.items;
    Ok(layer)
}
