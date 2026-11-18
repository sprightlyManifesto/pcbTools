//! PCBtools `convert` — convert between Gerber (RS-274X), DXF and SVG.
//!
//!     convert input.gbr output.dxf
//!     convert top.gtl bottom.gbl board.svg      (several inputs = several layers)

mod area;
mod dxf_read;
mod dxf_write;
mod gerber_read;
mod gerber_write;
mod model;
mod svg_read;
mod svg_write;

use anyhow::{bail, Context, Result};
use clap::{Parser, ValueEnum};
use model::{Doc, Opts};
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Format {
    Gerber,
    Dxf,
    Svg,
}

/// Convert engineering drawings between Gerber, DXF and SVG.
///
/// The file types are taken from the file extensions (.gbr .grb .grbl .ger
/// .gtl .gbl .gto ... for Gerber, .dxf, .svg). The last file is the output;
/// every file before it is an input, and each input becomes one layer (SVG
/// and DXF inputs keep their own layers).
///
/// Gerber/SVG -> DXF draws a closed outline around every area of each layer
/// using LINE, ARC and CIRCLE entities; where an outline is not made of
/// straight lines and circular arcs (e.g. Bézier curves, ellipses, splines)
/// the loop is completed with short line facets.
///
/// Gerber <-> SVG converts element by element; every layer gets its own colour.
///
/// Gerber output holds one layer per file: with several layers, OUTPUT
/// "board.gbr" becomes "board-<layer>.gbr".
#[derive(Parser, Debug)]
#[command(name = "convert", version)]
struct Cli {
    /// Input file(s) followed by the output file.
    #[arg(required = true, num_args = 2.., value_name = "FILE")]
    files: Vec<PathBuf>,

    /// Force the input format instead of guessing from the extension.
    #[arg(long, value_enum)]
    from: Option<Format>,

    /// Force the output format instead of guessing from the extension.
    #[arg(long, value_enum)]
    to: Option<Format>,

    /// Maximum deviation (mm) allowed when arcs/curves must be split into
    /// facets.
    #[arg(long, default_value_t = 0.002, value_name = "MM")]
    tolerance: f64,

    /// Pen width (mm) used to draw zero-width lines (open DXF geometry) in SVG
    /// and Gerber output.
    #[arg(long, default_value_t = 0.1, value_name = "MM")]
    line_width: f64,

    /// Treat DXF input as plain lines instead of area outlines. By default
    /// closed DXF loops are read as the boundaries of filled areas (nested
    /// loops are holes), which is the inverse of Gerber/SVG -> DXF.
    #[arg(long)]
    dxf_lines: bool,
}

fn format_from_ext(p: &Path) -> Option<Format> {
    let ext = p.extension()?.to_str()?.to_ascii_lowercase();
    match ext.as_str() {
        "svg" => Some(Format::Svg),
        "dxf" => Some(Format::Dxf),
        "gbr" | "grb" | "grbl" | "gerber" | "ger" | "pho" | "art" | "gtl" | "gbl" | "gto" | "gbo" | "gts" | "gbs" | "gtp"
        | "gbp" | "gko" | "gm1" | "gml" | "gm" | "gpt" | "gpb" | "cmp" | "sol" | "stc" | "sts" | "plc" | "pls" | "crc"
        | "crs" => Some(Format::Gerber),
        e if e.len() >= 2 && (e.starts_with("gl") || e.starts_with('g')) && e[1..].chars().all(|c| c.is_ascii_digit()) => {
            Some(Format::Gerber) // .g1 .g2 ... inner layers
        }
        _ => None,
    }
}

fn sniff(text: &str) -> Option<Format> {
    let t = text.trim_start();
    if t.starts_with('<') && text.contains("<svg") {
        Some(Format::Svg)
    } else if text.contains("SECTION") && text.contains("ENTITIES") {
        Some(Format::Dxf)
    } else if text.contains("%FS") {
        Some(Format::Gerber)
    } else {
        None
    }
}

fn stem(p: &Path) -> String {
    p.file_stem().and_then(|s| s.to_str()).unwrap_or("layer").to_string()
}

fn read_input(path: &Path, forced: Option<Format>, opts: &Opts) -> Result<Doc> {
    let bytes = std::fs::read(path).with_context(|| format!("cannot read {}", path.display()))?;
    let text = String::from_utf8_lossy(&bytes);
    let fmt = forced
        .or_else(|| format_from_ext(path))
        .or_else(|| sniff(&text))
        .with_context(|| format!("cannot tell the file type of {}; use --from", path.display()))?;
    let doc = match fmt {
        Format::Gerber => Doc { layers: vec![gerber_read::read(&text, &stem(path), opts)?] },
        Format::Dxf => dxf_read::read(&text, opts)?,
        Format::Svg => svg_read::read(&text, opts)?,
    };
    Ok(doc)
}

fn run(cli: Cli) -> Result<()> {
    if !(cli.tolerance > 0.0) {
        bail!("--tolerance must be positive");
    }
    let opts = Opts { tol: cli.tolerance, line_width: cli.line_width, dxf_lines: cli.dxf_lines };
    let (output, inputs) = cli.files.split_last().unwrap();
    let out_fmt = cli
        .to
        .or_else(|| format_from_ext(output))
        .with_context(|| format!("cannot tell the output type from '{}'; use --to", output.display()))?;

    let mut doc = Doc::default();
    for inp in inputs {
        let d = read_input(inp, cli.from, &opts).with_context(|| format!("while reading {}", inp.display()))?;
        for mut l in d.layers {
            // Keep layer names unique across inputs.
            if doc.layers.iter().any(|x| x.name == l.name) {
                let base = format!("{}-{}", stem(inp), l.name);
                let mut name = base.clone();
                let mut k = 2;
                while doc.layers.iter().any(|x| x.name == name) {
                    name = format!("{base}-{k}");
                    k += 1;
                }
                l.name = name;
            }
            doc.layers.push(l);
        }
    }
    if doc.layers.is_empty() {
        model::warn("nothing to convert: the input contains no supported geometry");
    }

    match out_fmt {
        Format::Dxf => write_file(output, &dxf_write::write(&doc, &opts))?,
        Format::Svg => write_file(output, &svg_write::write(&doc, &opts))?,
        Format::Gerber => {
            if doc.layers.len() <= 1 {
                let empty = model::Layer::new(stem(output));
                let layer = doc.layers.first().unwrap_or(&empty);
                write_file(output, &gerber_write::write(layer, &opts))?;
            } else {
                let ext = output.extension().and_then(|e| e.to_str()).unwrap_or("gbr");
                for layer in &doc.layers {
                    let safe: String =
                        layer.name.chars().map(|c| if c.is_ascii_alphanumeric() || "-_.".contains(c) { c } else { '_' }).collect();
                    let p = output.with_file_name(format!("{}-{}.{}", stem(output), safe, ext));
                    write_file(&p, &gerber_write::write(layer, &opts))?;
                }
            }
        }
    }
    Ok(())
}

fn write_file(p: &Path, s: &str) -> Result<()> {
    std::fs::write(p, s).with_context(|| format!("cannot write {}", p.display()))?;
    eprintln!("wrote {}", p.display());
    Ok(())
}

fn main() {
    let cli = Cli::parse();
    let r = run(cli);
    model::print_warnings();
    if let Err(e) = r {
        eprintln!("error: {e:#}");
        std::process::exit(1);
    }
}
