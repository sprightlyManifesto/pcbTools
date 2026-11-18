//! SVG reader. Top-level groups (Inkscape layers or plain <g>) become layers.
//! Circles stay circles, circular arcs stay arcs, Bézier curves and
//! ellipses are faceted within the tolerance.

use crate::model::*;
use anyhow::{Context, Result};
use roxmltree::Node;
use std::f64::consts::TAU;
use std::str::FromStr;

const PX: f64 = 25.4 / 96.0;
const INKSCAPE: &str = "http://www.inkscape.org/namespaces/inkscape";

#[derive(Clone)]
struct Style {
    fill: bool,
    rule: FillRule,
    stroke: bool,
    width: f64,
    cap: Cap,
    join: &'static str,
    miter: f64,
}

fn style_prop(node: Node, name: &str) -> Option<String> {
    if let Some(st) = node.attribute("style") {
        for decl in st.split(';') {
            if let Some((k, v)) = decl.split_once(':') {
                if k.trim() == name {
                    return Some(v.trim().trim_end_matches("!important").trim().to_string());
                }
            }
        }
    }
    node.attribute(name).map(|s| s.trim().to_string())
}

fn number(s: &str) -> Option<f64> {
    svgtypes::Length::from_str(s).ok().map(|l| l.number)
}

fn attr_num(node: Node, name: &str) -> f64 {
    node.attribute(name).and_then(number).unwrap_or(0.0)
}

fn length_mm(s: &str) -> Option<f64> {
    use svgtypes::LengthUnit as U;
    let l = svgtypes::Length::from_str(s).ok()?;
    Some(match l.unit {
        U::None | U::Px => l.number * PX,
        U::Mm => l.number,
        U::Cm => l.number * 10.0,
        U::In => l.number * 25.4,
        U::Pt => l.number * 25.4 / 72.0,
        U::Pc => l.number * 25.4 / 6.0,
        _ => return None,
    })
}

fn paint_on(v: &str) -> bool {
    !matches!(v, "none" | "transparent")
}

fn apply_style(node: Node, parent: &Style) -> Style {
    let mut s = parent.clone();
    if let Some(v) = style_prop(node, "fill") {
        s.fill = paint_on(&v);
    }
    if let Some(v) = style_prop(node, "fill-opacity") {
        if v.parse::<f64>().map(|o| o <= 0.0).unwrap_or(false) {
            s.fill = false;
        }
    }
    if let Some(v) = style_prop(node, "fill-rule") {
        s.rule = if v == "evenodd" { FillRule::EvenOdd } else { FillRule::NonZero };
    }
    if let Some(v) = style_prop(node, "stroke") {
        s.stroke = paint_on(&v);
    }
    if let Some(v) = style_prop(node, "stroke-opacity") {
        if v.parse::<f64>().map(|o| o <= 0.0).unwrap_or(false) {
            s.stroke = false;
        }
    }
    if let Some(v) = style_prop(node, "stroke-width").and_then(|v| number(&v)) {
        s.width = v;
    }
    if let Some(v) = style_prop(node, "stroke-linecap") {
        s.cap = match v.as_str() {
            "round" => Cap::Round,
            "square" => Cap::Square,
            _ => Cap::Butt,
        };
    }
    if let Some(v) = style_prop(node, "stroke-linejoin") {
        s.join = match v.as_str() {
            "round" => "round",
            "bevel" => "bevel",
            _ => "miter",
        };
    }
    if let Some(v) = style_prop(node, "stroke-miterlimit").and_then(|v| v.parse().ok()) {
        s.miter = v;
    }
    s
}

fn transform_of(node: Node) -> Affine {
    match node.attribute("transform").map(svgtypes::Transform::from_str) {
        Some(Ok(t)) => Affine { a: t.a, b: t.b, c: t.c, d: t.d, e: t.e, f: t.f },
        Some(Err(_)) => {
            warn("unparsable transform attribute ignored");
            Affine::IDENTITY
        }
        None => Affine::IDENTITY,
    }
}

fn cubic(path: &mut Path, p0: Pt, c1: Pt, c2: Pt, p3: Pt, tol: f64) {
    let dd = (p0 - c1 * 2.0 + c2).len().max((c1 - c2 * 2.0 + p3).len());
    let n = ((0.75 * dd / tol).sqrt().ceil() as usize).clamp(1, 1000);
    for k in 1..=n {
        let t = k as f64 / n as f64;
        let u = 1.0 - t;
        path.line_to(p0 * (u * u * u) + c1 * (3.0 * u * u * t) + c2 * (3.0 * u * t * t) + p3 * (t * t * t));
    }
}

fn quad(path: &mut Path, p0: Pt, c: Pt, p2: Pt, tol: f64) {
    let dd = (p0 - c * 2.0 + p2).len();
    let n = ((dd / (4.0 * tol)).sqrt().ceil() as usize).clamp(1, 1000);
    for k in 1..=n {
        let t = k as f64 / n as f64;
        let u = 1.0 - t;
        path.line_to(p0 * (u * u) + c * (2.0 * u * t) + p2 * (t * t));
    }
}

/// SVG endpoint arc -> circular arc segment or facets.
#[allow(clippy::too_many_arguments)]
fn elliptical_arc(path: &mut Path, p1: Pt, mut rx: f64, mut ry: f64, phi_deg: f64, large: bool, sweep: bool, p2: Pt, tol: f64) {
    if p1.dist(p2) < 1e-12 {
        return;
    }
    rx = rx.abs();
    ry = ry.abs();
    if rx < 1e-12 || ry < 1e-12 {
        path.line_to(p2);
        return;
    }
    let phi = phi_deg.to_radians();
    let d = ((p1 - p2) * 0.5).rotate(-phi);
    let lambda = d.x * d.x / (rx * rx) + d.y * d.y / (ry * ry);
    if lambda > 1.0 {
        rx *= lambda.sqrt();
        ry *= lambda.sqrt();
    }
    let num = rx * rx * ry * ry - rx * rx * d.y * d.y - ry * ry * d.x * d.x;
    let den = rx * rx * d.y * d.y + ry * ry * d.x * d.x;
    let mut coef = (num / den).max(0.0).sqrt();
    if large == sweep {
        coef = -coef;
    }
    let cp = pt(coef * rx * d.y / ry, -coef * ry * d.x / rx);
    let c = cp.rotate(phi) + (p1 + p2) * 0.5;
    let u = pt((d.x - cp.x) / rx, (d.y - cp.y) / ry);
    let v = pt((-d.x - cp.x) / rx, (-d.y - cp.y) / ry);
    let t1 = u.angle();
    let mut dt = angle_between(u, v);
    if !sweep && dt > 0.0 {
        dt -= TAU;
    } else if sweep && dt < 0.0 {
        dt += TAU;
    }
    if (rx - ry).abs() <= 1e-9 * rx.max(ry) {
        path.segs.push(Seg::Arc { end: p2, center: c, sweep: dt });
        return;
    }
    let n = facet_count(rx.max(ry), dt, tol).max(4);
    for k in 1..=n {
        let t = t1 + dt * k as f64 / n as f64;
        let q = if k == n { p2 } else { c + pt(rx * t.cos(), ry * t.sin()).rotate(phi) };
        path.line_to(q);
    }
}

fn parse_d(d: &str, tol: f64) -> Vec<Path> {
    use svgtypes::PathSegment as S;
    let mut out: Vec<Path> = Vec::new();
    let mut cur_path: Option<Path> = None;
    let mut cur = pt(0.0, 0.0);
    let mut start = cur;
    let mut last_cubic: Option<Pt> = None;
    let mut last_quad: Option<Pt> = None;
    for seg in svgtypes::PathParser::from(d) {
        let Ok(seg) = seg else {
            warn("malformed SVG path data; rest of path ignored");
            break;
        };
        let rel = |abs: bool, x: f64, y: f64, cur: Pt| if abs { pt(x, y) } else { cur + pt(x, y) };
        let (mut lc, mut lq) = (None, None);
        if let S::MoveTo { abs, x, y } = seg {
            if let Some(p) = cur_path.take() {
                if !p.segs.is_empty() {
                    out.push(p);
                }
            }
            cur = rel(abs, x, y, cur);
            start = cur;
            cur_path = Some(Path::new(cur));
            last_cubic = None;
            last_quad = None;
            continue;
        }
        let p = cur_path.get_or_insert_with(|| Path::new(cur));
        match seg {
            S::LineTo { abs, x, y } => {
                cur = rel(abs, x, y, cur);
                p.line_to(cur);
            }
            S::HorizontalLineTo { abs, x } => {
                cur = if abs { pt(x, cur.y) } else { cur + pt(x, 0.0) };
                p.line_to(cur);
            }
            S::VerticalLineTo { abs, y } => {
                cur = if abs { pt(cur.x, y) } else { cur + pt(0.0, y) };
                p.line_to(cur);
            }
            S::CurveTo { abs, x1, y1, x2, y2, x, y } => {
                let (c1, c2, e) = (rel(abs, x1, y1, cur), rel(abs, x2, y2, cur), rel(abs, x, y, cur));
                cubic(p, cur, c1, c2, e, tol);
                lc = Some(c2);
                cur = e;
            }
            S::SmoothCurveTo { abs, x2, y2, x, y } => {
                let c1 = last_cubic.map(|c| cur * 2.0 - c).unwrap_or(cur);
                let (c2, e) = (rel(abs, x2, y2, cur), rel(abs, x, y, cur));
                cubic(p, cur, c1, c2, e, tol);
                lc = Some(c2);
                cur = e;
            }
            S::Quadratic { abs, x1, y1, x, y } => {
                let (c, e) = (rel(abs, x1, y1, cur), rel(abs, x, y, cur));
                quad(p, cur, c, e, tol);
                lq = Some(c);
                cur = e;
            }
            S::SmoothQuadratic { abs, x, y } => {
                let c = last_quad.map(|c| cur * 2.0 - c).unwrap_or(cur);
                let e = rel(abs, x, y, cur);
                quad(p, cur, c, e, tol);
                lq = Some(c);
                cur = e;
            }
            S::EllipticalArc { abs, rx, ry, x_axis_rotation, large_arc, sweep, x, y } => {
                let e = rel(abs, x, y, cur);
                elliptical_arc(p, cur, rx, ry, x_axis_rotation, large_arc, sweep, e, tol);
                cur = e;
            }
            S::ClosePath { .. } => {
                let mut done = cur_path.take().unwrap();
                done.closed = true;
                if !done.segs.is_empty() {
                    out.push(done);
                }
                cur = start;
            }
            S::MoveTo { .. } => unreachable!(),
        }
        last_cubic = lc;
        last_quad = lq;
    }
    if let Some(p) = cur_path {
        if !p.segs.is_empty() {
            out.push(p);
        }
    }
    out
}

struct Ctx<'a, 'i> {
    doc: &'a roxmltree::Document<'i>,
    root: Affine,
    tol: f64,
    depth: usize,
}

impl Ctx<'_, '_> {
    fn emit(&self, paths: Vec<Path>, circle: Option<(Pt, f64)>, ctm: &Affine, st: &Style, layer: &mut Layer) {
        let m = self.root.then_after(ctm);
        if st.fill {
            match circle {
                Some((c, r)) if m.is_similarity() => layer.push(Shape::Disc { c: m.apply(c), r: r * m.scale() }),
                _ => {
                    let fills: Vec<Path> = paths
                        .iter()
                        .map(|p| {
                            let mut q = p.transform(&m, self.tol);
                            q.closed = true;
                            q
                        })
                        .collect();
                    if !fills.is_empty() {
                        layer.push(Shape::Fill { paths: fills, rule: st.rule });
                    }
                }
            }
        }
        if st.stroke && st.width > 0.0 {
            let join = match st.join {
                "round" => Join::Round,
                "bevel" => Join::Bevel,
                _ => Join::Miter(st.miter),
            };
            for p in &paths {
                layer.push(Shape::Stroke { path: p.transform(&m, self.tol), width: st.width * m.scale(), cap: st.cap, join });
            }
        }
    }

    fn walk(&mut self, node: Node, ctm: &Affine, parent: &Style, layer: &mut Layer) {
        if !node.is_element() || self.depth > 64 {
            return;
        }
        if style_prop(node, "display").as_deref() == Some("none") {
            return;
        }
        let st = apply_style(node, parent);
        let ctm = ctm.then_after(&transform_of(node));
        let tol_user = self.tol / self.root.then_after(&ctm).scale().max(1e-12);
        let hidden = style_prop(node, "visibility").is_some_and(|v| v == "hidden" || v == "collapse");
        match node.tag_name().name() {
            "g" | "a" | "switch" | "svg" => {
                self.depth += 1;
                for ch in node.children() {
                    self.walk(ch, &ctm, &st, layer);
                }
                self.depth -= 1;
            }
            "use" => {
                let href = node.attribute(("http://www.w3.org/1999/xlink", "href")).or_else(|| node.attribute("href"));
                if let Some(id) = href.and_then(|h| h.strip_prefix('#')) {
                    if let Some(target) = self.doc.descendants().find(|n| n.attribute("id") == Some(id)) {
                        let t = ctm.then_after(&Affine::translate(attr_num(node, "x"), attr_num(node, "y")));
                        self.depth += 1;
                        self.walk(target, &t, &st, layer);
                        self.depth -= 1;
                    }
                }
            }
            _ if hidden => {}
            "path" => {
                let paths = parse_d(node.attribute("d").unwrap_or(""), tol_user);
                self.emit(paths, None, &ctm, &st, layer);
            }
            "rect" => {
                let (x, y, w, h) = (attr_num(node, "x"), attr_num(node, "y"), attr_num(node, "width"), attr_num(node, "height"));
                if w <= 0.0 || h <= 0.0 {
                    return;
                }
                let mut rx = node.attribute("rx").and_then(number);
                let mut ry = node.attribute("ry").and_then(number);
                if rx.is_none() {
                    rx = ry;
                }
                if ry.is_none() {
                    ry = rx;
                }
                let rx = rx.unwrap_or(0.0).clamp(0.0, w / 2.0);
                let ry = ry.unwrap_or(0.0).clamp(0.0, h / 2.0);
                let d = if rx > 0.0 && ry > 0.0 {
                    format!(
                        "M{} {} H{} A{rx} {ry} 0 0 1 {} {} V{} A{rx} {ry} 0 0 1 {} {} H{} A{rx} {ry} 0 0 1 {} {} V{} A{rx} {ry} 0 0 1 {} {} Z",
                        x + rx, y, x + w - rx, x + w, y + ry, y + h - ry, x + w - rx, y + h, x + rx, x, y + h - ry, y + ry, x + rx, y
                    )
                } else {
                    format!("M{x} {y} H{} V{} H{x} Z", x + w, y + h)
                };
                self.emit(parse_d(&d, tol_user), None, &ctm, &st, layer);
            }
            "circle" => {
                let (c, r) = (pt(attr_num(node, "cx"), attr_num(node, "cy")), attr_num(node, "r"));
                if r > 0.0 {
                    self.emit(vec![Path::circle(c, r)], Some((c, r)), &ctm, &st, layer);
                }
            }
            "ellipse" => {
                let (cx, cy) = (attr_num(node, "cx"), attr_num(node, "cy"));
                let (rx, ry) = (attr_num(node, "rx"), attr_num(node, "ry"));
                if (rx - ry).abs() < 1e-12 && rx > 0.0 {
                    self.emit(vec![Path::circle(pt(cx, cy), rx)], Some((pt(cx, cy), rx)), &ctm, &st, layer);
                } else if rx > 0.0 && ry > 0.0 {
                    let d = format!("M{} {cy} A{rx} {ry} 0 1 0 {} {cy} A{rx} {ry} 0 1 0 {} {cy} Z", cx - rx, cx + rx, cx - rx);
                    self.emit(parse_d(&d, tol_user), None, &ctm, &st, layer);
                }
            }
            "line" => {
                let mut p = Path::new(pt(attr_num(node, "x1"), attr_num(node, "y1")));
                p.line_to(pt(attr_num(node, "x2"), attr_num(node, "y2")));
                let st2 = Style { fill: false, ..st };
                self.emit(vec![p], None, &ctm, &st2, layer);
            }
            tag @ ("polyline" | "polygon") => {
                let pts: Vec<Pt> = svgtypes::PointsParser::from(node.attribute("points").unwrap_or("")).map(|(x, y)| pt(x, y)).collect();
                if pts.len() >= 2 {
                    let mut p = Path::new(pts[0]);
                    for q in &pts[1..] {
                        p.line_to(*q);
                    }
                    p.closed = tag == "polygon";
                    self.emit(vec![p], None, &ctm, &st, layer);
                }
            }
            "text" | "tspan" | "textPath" => warn("SVG text is not converted (convert text to paths first)"),
            "image" => warn("SVG embedded images are not converted"),
            _ => {} // defs, style, metadata, clipPath, mask, ...
        }
    }
}

pub fn read(text: &str, opts: &Opts) -> Result<Doc> {
    let xml = roxmltree::Document::parse_with_options(text, roxmltree::ParsingOptions { allow_dtd: true, ..Default::default() })
        .context("invalid SVG/XML")?;
    let svg = xml.root_element();
    if svg.tag_name().name() != "svg" {
        anyhow::bail!("root element is <{}>, not <svg>", svg.tag_name().name());
    }
    // User units -> millimetres, y axis flipped to point up.
    let vb: Option<Vec<f64>> = svg.attribute("viewBox").map(|v| {
        v.split(|c: char| c == ',' || c.is_whitespace()).filter(|s| !s.is_empty()).filter_map(|s| s.parse().ok()).collect()
    });
    let (sx, sy) = match vb {
        Some(v) if v.len() == 4 && v[2] > 0.0 && v[3] > 0.0 => {
            let w = svg.attribute("width").and_then(length_mm).unwrap_or(v[2] * PX);
            let h = svg.attribute("height").and_then(length_mm).unwrap_or(v[3] * PX);
            (w / v[2], h / v[3])
        }
        _ => (PX, PX),
    };
    if (sx - sy).abs() > 1e-9 * sx {
        warn("non-uniform viewBox scaling: circles are faceted");
    }
    let root = Affine { a: sx, b: 0.0, c: 0.0, d: -sy, e: 0.0, f: 0.0 };
    let base = Style { fill: true, rule: FillRule::NonZero, stroke: false, width: 1.0, cap: Cap::Butt, join: "miter", miter: 4.0 };
    let root_style = apply_style(svg, &base);
    let root_ctm = transform_of(svg);
    let mut ctx = Ctx { doc: &xml, root, tol: opts.tol, depth: 0 };

    let mut doc = Doc::default();
    let mut loose = Layer::new("default");
    for (k, ch) in svg.children().filter(|n| n.is_element()).enumerate() {
        if ch.tag_name().name() == "g" {
            let name = ch
                .attribute((INKSCAPE, "label"))
                .or_else(|| ch.attribute("id"))
                .map(|s| s.to_string())
                .unwrap_or_else(|| format!("layer{}", k + 1));
            let mut layer = Layer::new(name);
            ctx.walk(ch, &root_ctm, &root_style, &mut layer);
            if !layer.items.is_empty() {
                doc.layers.push(layer);
            }
        } else {
            ctx.walk(ch, &root_ctm, &root_style, &mut loose);
        }
    }
    if !loose.items.is_empty() {
        doc.layers.insert(0, loose);
    }
    Ok(doc)
}
