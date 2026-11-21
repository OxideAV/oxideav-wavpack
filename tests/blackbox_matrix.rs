//! Live black-box matrix: streams written by an outside encoder are
//! decoded by this crate and compared sample-for-sample with an outside
//! decoder's output of the same file.
//!
//! Nothing here is committed as a fixture: every stream is generated in
//! a temporary directory at run time from deterministic synthetic PCM.
//! The whole file **skips at run time** (each test prints a note and
//! returns) when the `wavpack` / `wvunpack` executables are not on
//! `PATH`.
//!
//! Comparison rule: lossless streams and hybrid `.wv` + `.wvc` pairs
//! must equal the outside decoder's output *and* the original input;
//! hybrid lossy-only streams must equal the outside decoder's lossy
//! output bit for bit (the format's integer arithmetic determines it).
//!
//! Set `WAVPACK_MATRIX_VERBOSE=1` to print one line per case.

use std::path::{Path, PathBuf};
use std::process::Command;

use oxideav_wavpack::{
    decode_multichannel_stream, decode_multichannel_stream_f32, decode_multichannel_stream_muted,
    decode_multichannel_stream_with_correction, decode_multichannel_stream_with_correction_f32,
    first_audio_block, stream_channel_geometry, stream_sample_rate,
};

// ---------------------------------------------------------------------
// Tool discovery + scratch space
// ---------------------------------------------------------------------

fn have(tool: &str) -> bool {
    Command::new(tool)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

fn tools_present() -> bool {
    let ok = have("wavpack") && have("wvunpack");
    if !ok {
        eprintln!("blackbox_matrix: wavpack / wvunpack not on PATH — skipped");
    }
    ok
}

struct Scratch(PathBuf);

impl Scratch {
    fn new(tag: &str) -> Self {
        let dir = std::env::temp_dir().join(format!(
            "oxideav-wavpack-matrix-{}-{}",
            tag,
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("scratch dir");
        Self(dir)
    }
    fn path(&self, name: &str) -> PathBuf {
        self.0.join(name)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn verbose() -> bool {
    std::env::var_os("WAVPACK_MATRIX_VERBOSE").is_some()
}

// ---------------------------------------------------------------------
// Deterministic content
// ---------------------------------------------------------------------

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 32) as u32
    }
    /// Uniform in `-1.0..1.0`.
    fn unit(&mut self) -> f64 {
        f64::from(self.next()) / 2_147_483_648.0 - 1.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Content {
    /// Tone mixture plus a little noise; channels correlated.
    Music,
    /// Full-band noise near full scale.
    Noise,
    /// Digital silence.
    Silence,
    /// Tone bursts separated by silence stretches.
    Bursts,
    /// Square wave slamming both rails.
    Rails,
}

/// Unit-range (`-1.0..=1.0`) interleaved content.
fn content(kind: Content, frames: usize, channels: usize, rate: u32, seed: u64) -> Vec<f64> {
    let mut rng = Lcg(seed ^ 0x9E37_79B9_7F4A_7C15);
    let mut out = Vec::with_capacity(frames * channels);
    let sr = f64::from(rate.max(1));
    for n in 0..frames {
        let t = n as f64 / sr;
        let shared = (t * 440.0 * std::f64::consts::TAU).sin() * 0.45
            + (t * 1318.5 * std::f64::consts::TAU).sin() * 0.2;
        for ch in 0..channels {
            let own = (t * (97.0 + 61.0 * ch as f64) * std::f64::consts::TAU).sin() * 0.25;
            let v = match kind {
                Content::Music => shared + own + rng.unit() * 0.01,
                Content::Noise => rng.unit() * 0.98,
                Content::Silence => 0.0,
                Content::Bursts => {
                    if (n / 700) % 3 == 1 {
                        0.0
                    } else {
                        shared + own
                    }
                }
                Content::Rails => {
                    if ((n + ch * 7) / 37) % 2 == 0 {
                        1.0
                    } else {
                        -1.0
                    }
                }
            };
            out.push(v.clamp(-1.0, 1.0));
        }
    }
    out
}

// ---------------------------------------------------------------------
// Source format + WAV writer
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
struct Format {
    /// Significant bits (1..=32).
    bits: u32,
    float: bool,
    channels: usize,
    rate: u32,
    /// Speaker mask for the extensible header; `None` picks the plain
    /// header for 1–2 channels and a default mask above that.
    mask: Option<u32>,
}

impl Format {
    fn int(bits: u32, channels: usize, rate: u32) -> Self {
        Self {
            bits,
            float: false,
            channels,
            rate,
            mask: None,
        }
    }
    fn float(channels: usize, rate: u32) -> Self {
        Self {
            bits: 32,
            float: true,
            channels,
            rate,
            mask: None,
        }
    }
    fn container_bytes(&self) -> usize {
        self.bits.div_ceil(8) as usize
    }
    fn default_mask(&self) -> u32 {
        match self.channels {
            1 => 0x4,
            2 => 0x3,
            3 => 0x7,
            4 => 0x33,
            5 => 0x37,
            6 => 0x3F,
            7 => 0x13F,
            8 => 0x63F,
            n if n <= 18 => (1u32 << n) - 1,
            // More channels than speaker positions: unassigned.
            _ => 0,
        }
    }
}

/// Source samples in the crate's decode domain: sign-extended container
/// values for integer formats, IEEE-754 bit patterns for float.
fn quantize(fmt: &Format, unit: &[f64]) -> Vec<i32> {
    if fmt.float {
        return unit.iter().map(|&v| (v as f32).to_bits() as i32).collect();
    }
    let container = fmt.container_bytes() as u32 * 8;
    let pad = container - fmt.bits;
    let peak = (1i64 << (fmt.bits - 1)) - 1;
    unit.iter()
        .map(|&v| {
            let q = (v * peak as f64).round() as i64;
            let q = q.clamp(-peak - 1, peak);
            (q << pad) as i32
        })
        .collect()
}

fn write_wav(path: &Path, fmt: &Format, samples: &[i32]) {
    let bytes = fmt.container_bytes();
    let container_bits = (bytes * 8) as u16;
    let extensible =
        fmt.channels > 2 || fmt.mask.is_some() || u32::from(container_bits) != fmt.bits;
    let mut data = Vec::with_capacity(samples.len() * bytes);
    for &s in samples {
        if bytes == 1 {
            data.push((s + 128) as u8);
        } else {
            data.extend_from_slice(&s.to_le_bytes()[..bytes]);
        }
    }
    let block_align = (bytes * fmt.channels) as u16;
    let mut f = Vec::new();
    let fmt_len: u32 = if extensible { 40 } else { 16 };
    f.extend_from_slice(b"RIFF");
    f.extend_from_slice(
        &(4 + 8 + fmt_len + 8 + data.len() as u32 + (data.len() as u32 & 1)).to_le_bytes(),
    );
    f.extend_from_slice(b"WAVEfmt ");
    f.extend_from_slice(&fmt_len.to_le_bytes());
    let tag: u16 = if extensible {
        0xFFFE
    } else if fmt.float {
        3
    } else {
        1
    };
    f.extend_from_slice(&tag.to_le_bytes());
    f.extend_from_slice(&(fmt.channels as u16).to_le_bytes());
    f.extend_from_slice(&fmt.rate.to_le_bytes());
    f.extend_from_slice(&(fmt.rate * u32::from(block_align)).to_le_bytes());
    f.extend_from_slice(&block_align.to_le_bytes());
    f.extend_from_slice(&container_bits.to_le_bytes());
    if extensible {
        f.extend_from_slice(&22u16.to_le_bytes());
        f.extend_from_slice(&(fmt.bits as u16).to_le_bytes());
        f.extend_from_slice(&fmt.mask.unwrap_or(fmt.default_mask()).to_le_bytes());
        let sub: u16 = if fmt.float { 3 } else { 1 };
        f.extend_from_slice(&sub.to_le_bytes());
        f.extend_from_slice(&[
            0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71,
        ]);
    }
    f.extend_from_slice(b"data");
    f.extend_from_slice(&(data.len() as u32).to_le_bytes());
    f.extend_from_slice(&data);
    if data.len() & 1 == 1 {
        f.push(0);
    }
    std::fs::write(path, f).expect("write wav");
}

/// Raw little-endian decoder output → the crate's decode domain.
fn raw_to_samples(fmt: &Format, raw: &[u8]) -> Vec<i32> {
    let bytes = fmt.container_bytes();
    raw.chunks_exact(bytes)
        .map(|c| match bytes {
            1 => i32::from(c[0]) - 128,
            2 => i32::from(i16::from_le_bytes([c[0], c[1]])),
            3 => i32::from_le_bytes([0, c[0], c[1], c[2]]) >> 8,
            _ => i32::from_le_bytes([c[0], c[1], c[2], c[3]]),
        })
        .collect()
}

// ---------------------------------------------------------------------
// Case runner
// ---------------------------------------------------------------------

#[derive(Clone, Debug)]
struct Case {
    name: String,
    fmt: Format,
    frames: usize,
    content: Content,
    /// Extra encoder options.
    args: Vec<String>,
    /// The encoder options request hybrid mode.
    hybrid: bool,
    /// The encoder options request a correction file.
    correction: bool,
    /// The encoder options change the samples before coding, so the
    /// decode is compared with the outside decoder only.
    alters_source: bool,
    /// The stream uses a form the staged documents do not describe;
    /// this crate must refuse it with an error naming that form.
    expect_refusal: Option<&'static str>,
}

impl Case {
    fn new(name: impl Into<String>, fmt: Format, frames: usize, content: Content) -> Self {
        Self {
            name: name.into(),
            fmt,
            frames,
            content,
            args: Vec::new(),
            hybrid: false,
            correction: false,
            alters_source: false,
            expect_refusal: None,
        }
    }
    fn args(mut self, args: &[&str]) -> Self {
        for a in args {
            if a.starts_with("-b") && !a.starts_with("--") {
                self.hybrid = true;
            }
            if *a == "-c" || *a == "-cc" {
                self.correction = true;
            }
            if a.starts_with("--pre-quantize") {
                self.alters_source = true;
            }
            if *a == "--optimize-int32" && !self.fmt.float && self.fmt.bits > 24 {
                self.expect_refusal = Some("0x2C");
            }
            self.args.push((*a).to_string());
        }
        self
    }
}

fn run(cmd: &mut Command) -> Result<Vec<u8>, String> {
    let out = cmd.output().map_err(|e| format!("spawn: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "{:?} failed: {}",
            cmd.get_program(),
            String::from_utf8_lossy(&out.stderr).trim()
        ));
    }
    Ok(out.stderr)
}

fn first_diff(a: &[i32], b: &[i32]) -> String {
    if a.len() != b.len() {
        return format!("length {} vs {}", a.len(), b.len());
    }
    match a.iter().zip(b).position(|(x, y)| x != y) {
        Some(i) => format!(
            "first difference at value {i} of {}: {} vs {}",
            a.len(),
            a[i],
            b[i]
        ),
        None => "equal".to_string(),
    }
}

/// Decode `wv` (optionally with its correction twin) through this crate
/// into the decode domain.
fn ours(fmt: &Format, wv: &[u8], wvc: Option<&[u8]>) -> Result<(Vec<i32>, usize), String> {
    if fmt.float {
        let d = match wvc {
            Some(c) => decode_multichannel_stream_with_correction_f32(wv, c),
            None => decode_multichannel_stream_f32(wv),
        }
        .map_err(|e| format!("decode error: {e}"))?;
        Ok((
            d.samples.iter().map(|v| v.to_bits() as i32).collect(),
            d.channels,
        ))
    } else {
        let d = match wvc {
            Some(c) => decode_multichannel_stream_with_correction(wv, c),
            None => decode_multichannel_stream(wv),
        }
        .map_err(|e| format!("decode error: {e}"))?;
        Ok((d.samples, d.channels))
    }
}

fn reference_decode(
    scratch: &Scratch,
    fmt: &Format,
    wv: &Path,
    ignore_correction: bool,
) -> Result<(Vec<i32>, bool), String> {
    let raw = scratch.path("ref.raw");
    let _ = std::fs::remove_file(&raw);
    let mut cmd = Command::new("wvunpack");
    cmd.args(["-q", "-y", "--raw"]);
    if ignore_correction {
        cmd.arg("-i");
    }
    cmd.arg(wv).arg("-o").arg(&raw);
    // The decoder exits non-zero when a block fails its stored CRC
    // (it still writes the output, with the failing blocks muted).
    let clean = match run(&mut cmd) {
        Ok(_) => true,
        Err(e) if e.contains("crc errors") && raw.exists() => false,
        Err(e) => return Err(e),
    };
    let bytes = std::fs::read(&raw).map_err(|e| format!("read raw: {e}"))?;
    Ok((raw_to_samples(fmt, &bytes), clean))
}

/// Run one foreign → ours case. `Ok(())` when every comparison holds.
fn check_foreign(scratch: &Scratch, case: &Case, tally: &mut Tally) -> Result<(), String> {
    let fmt = &case.fmt;
    let unit = content(case.content, case.frames, fmt.channels, fmt.rate, 7);
    let source = quantize(fmt, &unit);
    let wav = scratch.path("in.wav");
    let wv = scratch.path("out.wv");
    let wvc = scratch.path("out.wvc");
    let _ = std::fs::remove_file(&wv);
    let _ = std::fs::remove_file(&wvc);
    write_wav(&wav, fmt, &source);

    let mut cmd = Command::new("wavpack");
    cmd.args(["-q", "-y"]);
    cmd.args(&case.args);
    cmd.arg(&wav).arg("-o").arg(&wv);
    run(&mut cmd).map_err(|e| format!("encoder refused the case: {e}"))?;

    let wv_bytes = std::fs::read(&wv).map_err(|e| format!("read wv: {e}"))?;
    let wvc_bytes = if case.correction {
        Some(std::fs::read(&wvc).map_err(|e| format!("read wvc: {e}"))?)
    } else {
        None
    };

    // Stream facts.
    if case.frames > 0 {
        let rate = stream_sample_rate(&wv_bytes).map_err(|e| format!("rate: {e}"))?;
        if rate != Some(fmt.rate) {
            return Err(format!("sample rate {rate:?}, expected {}", fmt.rate));
        }
        let float = first_audio_block(&wv_bytes)
            .map_err(|e| format!("first block: {e}"))?
            .is_some_and(|b| b.is_float());
        if float != fmt.float {
            return Err(format!("float flag {float}, expected {}", fmt.float));
        }
        // Plain mono / stereo with the default speaker pair carries no
        // geometry sub-block; everything else must.
        if fmt.channels > 2 || fmt.mask.is_some_and(|m| m != fmt.default_mask()) {
            let info =
                stream_channel_geometry(&wv_bytes).map_err(|e| format!("channel info: {e}"))?;
            let Some(info) = info else {
                return Err("no channel geometry on a multichannel stream".to_string());
            };
            if usize::from(info.channels) != fmt.channels {
                return Err(format!("geometry count {}", info.channels));
            }
            // The long form also states how many member blocks a set
            // has; it must agree with what the stream actually holds.
            if let Some(streams) = info.streams {
                let layout = oxideav_wavpack::multichannel_layout(&wv_bytes)
                    .map_err(|e| format!("layout: {e}"))?;
                let members = oxideav_wavpack::audio_block_count(&wv_bytes)
                    .map_err(|e| format!("block count: {e}"))?
                    / layout.sets.max(1);
                if members != usize::from(streams) {
                    return Err(format!(
                        "geometry states {streams} streams, the sets hold {members}"
                    ));
                }
            }
            let want = fmt.mask.unwrap_or(fmt.default_mask());
            if info.mask != want {
                return Err(format!(
                    "geometry mask {:#x}, expected {want:#x}",
                    info.mask
                ));
            }
        }
    }

    if let Some(needle) = case.expect_refusal {
        return match ours(fmt, &wv_bytes, None) {
            Err(e) if e.contains(needle) => {
                tally.refused += 1;
                Ok(())
            }
            Err(e) => Err(format!("refused, but not as the undocumented form: {e}")),
            Ok(_) => Err("decoded a form this crate has no description of".to_string()),
        };
    }

    // Single-file decode: lossless, or the lossy half of a hybrid encode.
    let (reference, clean) = reference_decode(scratch, fmt, &wv, true)?;
    let (mine, channels) = ours(fmt, &wv_bytes, None)?;
    if case.frames > 0 && channels != fmt.channels {
        return Err(format!(
            "channel count {channels}, expected {}",
            fmt.channels
        ));
    }
    if !clean {
        // The outside decoder rejects blocks of its own stream on a
        // single-file decode. Observed only on the lossy half of
        // correction-file encodes whose source has silent stretches:
        // the block header declares a zero maximum magnitude (the
        // source block is silent) while the lossy stream carries
        // shaped noise, and the outside decoder mutes such a block even
        // though its stored CRC describes the noise. The staged spec
        // names a per-block mute limit without defining it, so this
        // crate keeps the samples the stored CRC vouches for. A
        // lossless stream must never get here.
        if !case.correction || fmt.float {
            return Err("the outside decoder reports CRC errors on its own stream".to_string());
        }
        let (muted, ok) =
            decode_multichannel_stream_muted(&wv_bytes).map_err(|e| format!("muted: {e}"))?;
        if !ok {
            return Err("a stored block CRC did not match".to_string());
        }
        // Everywhere the outside decoder did not mute, the two agree.
        if muted.samples.len() != reference.len()
            || muted
                .samples
                .iter()
                .zip(&reference)
                .any(|(m, r)| *r != 0 && m != r)
        {
            return Err(
                "single-file decode differs from the outside decoder outside its muted blocks"
                    .to_string(),
            );
        }
        tally.self_rejected += 1;
    } else if mine != reference {
        return Err(format!(
            "single-file decode differs from the outside decoder: {}",
            first_diff(&mine, &reference)
        ));
    }
    if !case.hybrid && !case.alters_source && mine != source {
        return Err(format!(
            "lossless decode differs from the source: {}",
            first_diff(&mine, &source)
        ));
    }
    if !fmt.float && clean {
        let (muted, ok) =
            decode_multichannel_stream_muted(&wv_bytes).map_err(|e| format!("muted: {e}"))?;
        if !ok {
            return Err("a stored block CRC did not match".to_string());
        }
        if muted.samples != mine {
            return Err("muted twin differs".to_string());
        }
    }

    // Pair decode.
    if let Some(wvc_bytes) = &wvc_bytes {
        let (reference, clean) = reference_decode(scratch, fmt, &wv, false)?;
        if !clean {
            return Err("the outside decoder reports CRC errors on its own pair".to_string());
        }
        let (mine, _) = ours(fmt, &wv_bytes, Some(wvc_bytes))?;
        if mine != reference {
            return Err(format!(
                "pair decode differs from the outside decoder: {}",
                first_diff(&mine, &reference)
            ));
        }
        if mine != source {
            return Err(format!(
                "pair decode differs from the source: {}",
                first_diff(&mine, &source)
            ));
        }
    }
    Ok(())
}

/// With `WAVPACK_MATRIX_KEEP=<dir>` set, the files of a failing case
/// are copied there for inspection.
fn keep_failure(scratch: &Scratch, tag: &str, name: &str) {
    let Some(dir) = std::env::var_os("WAVPACK_MATRIX_KEEP") else {
        return;
    };
    let dir = PathBuf::from(dir);
    let _ = std::fs::create_dir_all(&dir);
    for ext in ["wav", "wv", "wvc"] {
        let from = scratch.path(if ext == "wav" {
            "in.wav"
        } else if ext == "wv" {
            "out.wv"
        } else {
            "out.wvc"
        });
        if from.exists() {
            let _ = std::fs::copy(&from, dir.join(format!("{tag}-{name}.{ext}")));
        }
    }
}

/// Per-family accounting of the cases that are not plain decode passes.
#[derive(Default)]
struct Tally {
    /// Cases whose lossy half the outside decoder itself rejects (see
    /// [`check_foreign`]); they pass on the stored CRC and the pair
    /// decode.
    self_rejected: usize,
    /// Cases this crate refuses by design (an undocumented stream
    /// form); these are **not** decode passes.
    refused: usize,
}

fn run_family(tag: &str, cases: Vec<Case>) {
    if !tools_present() {
        return;
    }
    let scratch = Scratch::new(tag);
    let mut failures = Vec::new();
    let mut tally = Tally::default();
    let total = cases.len();
    for case in &cases {
        let verdict = check_foreign(&scratch, case, &mut tally);
        if verbose() {
            eprintln!(
                "{tag}: {:<44} {}",
                case.name,
                match &verdict {
                    Ok(()) => "ok".to_string(),
                    Err(e) => format!("FAIL {e}"),
                }
            );
        }
        if let Err(e) = verdict {
            keep_failure(&scratch, tag, &case.name);
            failures.push(format!("{} [{}]: {e}", case.name, case.args.join(" ")));
        }
    }
    eprintln!(
        "blackbox_matrix {tag}: {}/{} cases behave as specified \
         ({} of them: lossy half muted by the outside decoder; {} of them: refused as an undocumented form)",
        total - failures.len(),
        total,
        tally.self_rejected,
        tally.refused
    );
    assert!(
        failures.is_empty(),
        "{} of {total} cases failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

// ---------------------------------------------------------------------
// Foreign → ours families
// ---------------------------------------------------------------------

const MODES: [&[&str]; 4] = [&["-f"], &[], &["-h"], &["-hh"]];

fn mode_name<'a>(mode: &[&'a str]) -> &'a str {
    mode.first().copied().unwrap_or("-g")
}

/// Lossless: every speed mode × extra-processing level × stereo coding,
/// 16-bit mono and stereo.
#[test]
fn foreign_lossless_modes() {
    let mut cases = Vec::new();
    for channels in [1usize, 2] {
        for mode in MODES {
            for x in ["-x0", "-x1", "-x2", "-x3", "-x4", "-x5", "-x6"] {
                let joints: &[&str] = if channels == 2 {
                    &["", "-j0", "-j1"]
                } else {
                    &[""]
                };
                for joint in joints {
                    let mut args: Vec<&str> = mode.to_vec();
                    args.push(x);
                    if !joint.is_empty() {
                        args.push(joint);
                    }
                    // The custom-filter levels are slow; keep them short.
                    let frames = if x >= "-x4" { 6_000 } else { 30_000 };
                    cases.push(
                        Case::new(
                            format!("{}ch{}{x}{joint}", channels, mode_name(mode)),
                            Format::int(16, channels, 44_100),
                            frames,
                            Content::Music,
                        )
                        .args(&args),
                    );
                }
            }
        }
    }
    run_family("lossless-modes", cases);
}

/// Lossless: every sample format the encoder accepts, each across the
/// four speed modes, mono and stereo, over three content kinds.
#[test]
fn foreign_lossless_formats() {
    let mut cases = Vec::new();
    for bits in [8u32, 12, 16, 20, 24, 32, 1, 5, 9, 13, 17, 23, 25, 31] {
        for channels in [1usize, 2] {
            for mode in MODES {
                for kind in [Content::Music, Content::Noise, Content::Rails] {
                    cases.push(
                        Case::new(
                            format!("int{bits}-{channels}ch{}-{kind:?}", mode_name(mode)),
                            Format::int(bits, channels, 48_000),
                            9_000,
                            kind,
                        )
                        .args(mode),
                    );
                }
            }
        }
    }
    for channels in [1usize, 2] {
        for mode in MODES {
            for kind in [
                Content::Music,
                Content::Noise,
                Content::Rails,
                Content::Bursts,
            ] {
                cases.push(
                    Case::new(
                        format!("float-{channels}ch{}-{kind:?}", mode_name(mode)),
                        Format::float(channels, 48_000),
                        9_000,
                        kind,
                    )
                    .args(mode),
                );
            }
        }
    }
    run_family("lossless-formats", cases);
}

/// Lossless: channel counts 1..=12 with speaker masks, and sample rates
/// (every standard table entry plus non-standard ones).
#[test]
fn foreign_channels_and_rates() {
    let mut cases = Vec::new();
    for channels in (1usize..=18).chain([24, 32, 33, 64, 255, 256]) {
        for (bits, float) in [(16u32, false), (24, false), (32, true)] {
            if channels > 18 && bits != 16 {
                continue;
            }
            let mut fmt = Format::int(bits, channels, 44_100);
            fmt.float = float;
            for mode in [&[][..], &["-hh"][..]] {
                cases.push(
                    Case::new(
                        format!(
                            "{channels}ch-{bits}{}{}",
                            if float { "f" } else { "" },
                            mode_name(mode)
                        ),
                        fmt,
                        if channels > 18 { 600 } else { 5_000 },
                        Content::Music,
                    )
                    .args(mode),
                );
            }
        }
    }
    // Explicit masks: no assignment at all, sparse, and a mask on mono /
    // stereo sources.
    for (channels, mask) in [
        (1usize, 0x4u32),
        (1, 0x8),
        (2, 0x3),
        (2, 0xC0),
        (3, 0xB),
        (4, 0x107),
        (6, 0x60F),
        (8, 0xFF),
        // (Masks with bits above the 18 speaker positions are written
        // by the outside encoder but refused by the outside decoder.)
        (13, 0x1_F0FF),
        (17, 0),
        (9, 0),
    ] {
        let mut fmt = Format::int(16, channels, 48_000);
        fmt.mask = Some(mask);
        cases.push(Case::new(
            format!("{channels}ch-mask{mask:#x}"),
            fmt,
            4_000,
            Content::Music,
        ));
    }
    for rate in [
        6_000u32, 8_000, 9_600, 11_025, 12_000, 16_000, 22_050, 24_000, 32_000, 44_100, 48_000,
        64_000, 88_200, 96_000, 192_000, 1, 7, 1_000, 12_345, 37_800, 50_000, 176_400, 352_800,
        384_000, 768_000, 1_234_567, 16_777_215,
    ] {
        for channels in [1usize, 2] {
            cases.push(Case::new(
                format!("rate{rate}-{channels}ch"),
                Format::int(16, channels, rate),
                4_000,
                Content::Music,
            ));
        }
    }
    run_family("channels-rates", cases);
}

/// Lossless: stream lengths from empty to long, and every block-size
/// option shape.
#[test]
fn foreign_lengths_and_blocks() {
    let mut cases = Vec::new();
    // (A zero-length source is refused by the outside encoder.)
    for frames in [
        1usize, 2, 3, 15, 16, 17, 127, 128, 129, 1_000, 22_049, 22_050, 22_051,
    ] {
        for channels in [1usize, 2, 6] {
            cases.push(Case::new(
                format!("len{frames}-{channels}ch"),
                Format::int(16, channels, 44_100),
                frames,
                Content::Music,
            ));
        }
    }
    for kind in [
        Content::Silence,
        Content::Bursts,
        Content::Noise,
        Content::Rails,
    ] {
        for channels in [1usize, 2] {
            cases.push(Case::new(
                format!("long-{kind:?}-{channels}ch"),
                Format::int(16, channels, 44_100),
                400_000,
                kind,
            ));
        }
    }
    cases.push(Case::new(
        "long-music-2ch-60s",
        Format::int(16, 2, 44_100),
        44_100 * 60,
        Content::Music,
    ));
    for block in [128u32, 129, 500, 1_024, 4_096, 22_050, 65_536, 131_072] {
        for channels in [1usize, 2] {
            let opt = format!("--blocksize={block}");
            cases.push(
                Case::new(
                    format!("block{block}-{channels}ch"),
                    Format::int(16, channels, 44_100),
                    150_000,
                    Content::Bursts,
                )
                .args(&[&opt]),
            );
        }
    }
    for block in [16u32, 64, 1_000] {
        let opt = format!("--blocksize={block}");
        cases.push(
            Case::new(
                format!("merge-block{block}"),
                Format::int(24, 2, 44_100),
                20_000,
                Content::Bursts,
            )
            .args(&[&opt, "--merge-blocks"]),
        );
    }
    run_family("lengths-blocks", cases);
}

/// Lossless: container options that add or change metadata blocks
/// (MD5, tags, header stripping, 32-bit integer optimisation,
/// pre-quantisation, unassigned-channel pairing).
#[test]
fn foreign_container_options() {
    let mut cases = Vec::new();
    let stereo = Format::int(16, 2, 44_100);
    for (name, args) in [
        ("md5", &["-m"][..]),
        ("no-riff", &["-r"][..]),
        (
            "tags",
            &[
                "-w",
                "Title=matrix",
                "-w",
                "Artist=nobody",
                "-w",
                "Encoder",
                "-w",
                "Settings",
            ][..],
        ),
        (
            "md5-tags-hh",
            &[
                "-m",
                "-hh",
                "-w",
                "Comment=a longer comment field for the tag",
            ][..],
        ),
        ("even-depth", &["--force-even-byte-depth"][..]),
        ("threads", &["--threads=4"][..]),
    ] {
        cases.push(Case::new(name, stereo, 30_000, Content::Music).args(args));
    }
    cases.push(
        Case::new(
            "even-depth-12bit",
            Format::int(12, 2, 44_100),
            9_000,
            Content::Music,
        )
        .args(&["--force-even-byte-depth"]),
    );
    for bits in [20u32, 16, 12] {
        let opt = format!("--pre-quantize={bits}");
        cases.push(
            Case::new(
                format!("prequant{bits}-int24"),
                Format::int(24, 2, 44_100),
                9_000,
                Content::Music,
            )
            .args(&[&opt]),
        );
        cases.push(
            Case::new(
                format!("prequant{bits}-float"),
                Format::float(2, 44_100),
                9_000,
                Content::Music,
            )
            .args(&[&opt]),
        );
    }
    for mode in MODES {
        let mut args = mode.to_vec();
        args.push("--optimize-int32");
        cases.push(
            Case::new(
                format!("opt-int32{}", mode_name(mode)),
                Format::int(32, 2, 44_100),
                9_000,
                Content::Music,
            )
            .args(&args),
        );
        cases.push(
            Case::new(
                format!("opt-int32-noise{}", mode_name(mode)),
                Format::int(32, 1, 44_100),
                9_000,
                Content::Noise,
            )
            .args(&args),
        );
    }
    for channels in [3usize, 5, 6, 8] {
        let mut fmt = Format::int(16, channels, 44_100);
        fmt.mask = Some(0);
        cases.push(Case::new(
            format!("unassigned-{channels}ch"),
            fmt,
            6_000,
            Content::Music,
        ));
        cases.push(
            Case::new(
                format!("paired-unassigned-{channels}ch"),
                fmt,
                6_000,
                Content::Music,
            )
            .args(&["--pair-unassigned-chans"]),
        );
    }
    run_family("container-options", cases);
}

/// Hybrid: bitrates × speed modes × stereo coding × shaping, lossy
/// alone and with the correction file.
#[test]
fn foreign_hybrid() {
    let mut cases = Vec::new();
    // Bitrate sweep (bits/sample and kbps forms), lossy and paired.
    for rate in [
        "-b2", "-b2.5", "-b3", "-b4", "-b6", "-b8", "-b12", "-b16", "-b23.9", "-b24", "-b64",
        "-b128", "-b320", "-b1000",
    ] {
        for channels in [1usize, 2] {
            for corr in [&[][..], &["-c"][..]] {
                let mut args = vec![rate];
                args.extend_from_slice(corr);
                cases.push(
                    Case::new(
                        format!("{rate}-{channels}ch{}", corr.first().copied().unwrap_or("")),
                        Format::int(16, channels, 44_100),
                        12_000,
                        Content::Music,
                    )
                    .args(&args),
                );
            }
        }
    }
    // Mode × extra × coding × correction kind.
    for mode in MODES {
        for x in ["-x0", "-x1", "-x3", "-x4", "-x6"] {
            for joint in ["", "-j0", "-j1"] {
                for corr in ["", "-c", "-cc"] {
                    let mut args: Vec<&str> = mode.to_vec();
                    args.push("-b3.5");
                    args.push(x);
                    for extra in [joint, corr] {
                        if !extra.is_empty() {
                            args.push(extra);
                        }
                    }
                    let frames = if x >= "-x4" { 5_000 } else { 14_000 };
                    cases.push(
                        Case::new(
                            format!("2ch{}{x}{joint}{corr}", mode_name(mode)),
                            Format::int(16, 2, 44_100),
                            frames,
                            Content::Music,
                        )
                        .args(&args),
                    );
                }
            }
        }
    }
    // Shaping + cross-decorrelation options.
    for shaping in [
        "-s0",
        "-s-1",
        "-s-0.5",
        "-s0.5",
        "-s1",
        "--use-dns",
        "--cross-decorr",
    ] {
        for channels in [1usize, 2] {
            for corr in ["", "-c", "-cc"] {
                for rate in [44_100u32, 96_000] {
                    let mut args = vec!["-b4", shaping];
                    if !corr.is_empty() {
                        args.push(corr);
                    }
                    cases.push(
                        Case::new(
                            format!("{shaping}-{channels}ch{corr}-{rate}"),
                            Format::int(16, channels, rate),
                            12_000,
                            Content::Music,
                        )
                        .args(&args),
                    );
                }
            }
        }
    }
    // Shaping across silence stretches and short blocks: zero runs,
    // and channels whose error limit drops to zero (read exactly)
    // while the shaping filter still carries error.
    for shaping in ["-s0.5", "-s1", "-s-0.5", "-s-1", "--use-dns"] {
        for channels in [1usize, 2] {
            for coding in ["", "-j0", "-cc", "-h"] {
                for block in ["--blocksize=700", "--blocksize=1100"] {
                    let mut args = vec!["-b4", "-c", shaping, block];
                    if !coding.is_empty() {
                        if coding == "-cc" {
                            args.retain(|a| *a != "-c");
                        }
                        args.push(coding);
                    }
                    cases.push(
                        Case::new(
                            format!("bursts{shaping}-{channels}ch{coding}{block}"),
                            Format::int(16, channels, 44_100),
                            6_000,
                            Content::Bursts,
                        )
                        .args(&args),
                    );
                }
            }
        }
    }
    run_family("hybrid", cases);
}

/// Hybrid across sample formats, channel layouts, content kinds and
/// lengths.
#[test]
fn foreign_hybrid_formats() {
    let mut cases = Vec::new();
    for bits in [8u32, 12, 16, 20, 24, 32, 13, 23] {
        for channels in [1usize, 2] {
            for corr in ["", "-c"] {
                for kind in [
                    Content::Music,
                    Content::Noise,
                    Content::Rails,
                    Content::Bursts,
                ] {
                    let mut args = vec!["-b3"];
                    if !corr.is_empty() {
                        args.push(corr);
                    }
                    cases.push(
                        Case::new(
                            format!("int{bits}-{channels}ch{corr}-{kind:?}"),
                            Format::int(bits, channels, 44_100),
                            9_000,
                            kind,
                        )
                        .args(&args),
                    );
                }
            }
        }
    }
    for channels in [1usize, 2] {
        for corr in ["", "-c"] {
            for kind in [
                Content::Music,
                Content::Noise,
                Content::Rails,
                Content::Bursts,
            ] {
                let mut args = vec!["-b5"];
                if !corr.is_empty() {
                    args.push(corr);
                }
                cases.push(
                    Case::new(
                        format!("float-{channels}ch{corr}-{kind:?}"),
                        Format::float(channels, 44_100),
                        9_000,
                        kind,
                    )
                    .args(&args),
                );
            }
        }
    }
    for channels in 3usize..=8 {
        for corr in ["", "-c", "-cc"] {
            let mut args = vec!["-b4"];
            if !corr.is_empty() {
                args.push(corr);
            }
            cases.push(
                Case::new(
                    format!("{channels}ch{corr}"),
                    Format::int(16, channels, 48_000),
                    6_000,
                    Content::Music,
                )
                .args(&args),
            );
        }
    }
    for frames in [1usize, 2, 17, 129, 1_000] {
        for corr in ["", "-c"] {
            let mut args = vec!["-b4"];
            if !corr.is_empty() {
                args.push(corr);
            }
            cases.push(
                Case::new(
                    format!("len{frames}{corr}"),
                    Format::int(16, 2, 44_100),
                    frames,
                    Content::Music,
                )
                .args(&args),
            );
        }
    }
    for kind in [
        Content::Silence,
        Content::Bursts,
        Content::Rails,
        Content::Music,
    ] {
        for corr in ["", "-c"] {
            let mut args = vec!["-b3", "--blocksize=4096"];
            if !corr.is_empty() {
                args.push(corr);
            }
            cases.push(
                Case::new(
                    format!("long-{kind:?}{corr}"),
                    Format::int(16, 2, 44_100),
                    200_000,
                    kind,
                )
                .args(&args),
            );
        }
    }
    run_family("hybrid-formats", cases);
}

// ---------------------------------------------------------------------
// Ours → foreign
// ---------------------------------------------------------------------

use oxideav_wavpack::{
    DecorrProfile, HybridOptions, HybridShaping, encode_multichannel_stream,
    encode_multichannel_stream_best, encode_multichannel_stream_float,
    encode_multichannel_stream_hybrid, encode_multichannel_stream_hybrid_float,
    encode_multichannel_stream_hybrid_int32, encode_multichannel_stream_int32,
    encode_multichannel_stream_smallest, encode_stream_mono, encode_stream_mono_best,
    encode_stream_mono_float, encode_stream_mono_hybrid, encode_stream_mono_hybrid_float,
    encode_stream_mono_hybrid_int32, encode_stream_mono_int32, encode_stream_mono_smallest,
    encode_stream_stereo, encode_stream_stereo_best, encode_stream_stereo_float,
    encode_stream_stereo_hybrid, encode_stream_stereo_hybrid_float,
    encode_stream_stereo_hybrid_int32, encode_stream_stereo_int32, encode_stream_stereo_smallest,
    set_stream_sample_rate,
};

const PROFILES: [DecorrProfile; 4] = [
    DecorrProfile::Fast,
    DecorrProfile::Normal,
    DecorrProfile::High,
    DecorrProfile::Extra,
];

/// One stream this crate wrote, with what a reader must get from it.
struct Emitted {
    name: String,
    fmt: Format,
    wv: Vec<u8>,
    wvc: Option<Vec<u8>>,
    /// The exact samples a lossless (or pair) decode must return.
    exact: Vec<i32>,
    /// Hybrid: the `.wv` alone is lossy.
    hybrid: bool,
}

/// Feed one emitted stream to the outside reader. It must verify the
/// stream without a word on stderr and decode it to the expected
/// samples: the source for a lossless stream or a pair, this crate's
/// own lossy decode for a hybrid `.wv` alone.
fn check_emitted(scratch: &Scratch, e: &Emitted) -> Result<(), String> {
    let wv = scratch.path("ours.wv");
    let wvc = scratch.path("ours.wvc");
    let _ = std::fs::remove_file(&wvc);
    std::fs::write(&wv, &e.wv).map_err(|x| format!("write: {x}"))?;

    let quiet = |cmd: &mut Command| -> Result<(), String> {
        let out = cmd.output().map_err(|x| format!("spawn: {x}"))?;
        let err = String::from_utf8_lossy(&out.stderr);
        if !out.status.success() || !err.trim().is_empty() {
            return Err(format!(
                "outside reader complained (exit {:?}): {}",
                out.status.code(),
                err.trim()
            ));
        }
        Ok(())
    };
    let decode = |ignore_correction: bool| -> Result<Vec<i32>, String> {
        let raw = scratch.path("ours.raw");
        let _ = std::fs::remove_file(&raw);
        let mut cmd = Command::new("wvunpack");
        cmd.args(["-q", "-y", "--raw"]);
        if ignore_correction {
            cmd.arg("-i");
        }
        cmd.arg(&wv).arg("-o").arg(&raw);
        quiet(&mut cmd)?;
        let bytes = std::fs::read(&raw).map_err(|x| format!("read raw: {x}"))?;
        Ok(raw_to_samples(&e.fmt, &bytes))
    };

    // Single file: verify, then decode.
    quiet(
        Command::new("wvunpack")
            .args(["-q", "-y", "-v", "-i"])
            .arg(&wv),
    )?;
    let theirs = decode(true)?;
    let (mine, _) = ours(&e.fmt, &e.wv, None)?;
    if theirs != mine {
        return Err(format!(
            "outside decode differs from this crate's: {}",
            first_diff(&theirs, &mine)
        ));
    }
    if !e.hybrid && theirs != e.exact {
        return Err(format!(
            "outside decode differs from the source: {}",
            first_diff(&theirs, &e.exact)
        ));
    }
    // Pair: verify and decode with the correction file alongside.
    if let Some(c) = &e.wvc {
        std::fs::write(&wvc, c).map_err(|x| format!("write wvc: {x}"))?;
        quiet(Command::new("wvunpack").args(["-q", "-y", "-v"]).arg(&wv))?;
        let theirs = decode(false)?;
        if theirs != e.exact {
            return Err(format!(
                "outside pair decode differs from the source: {}",
                first_diff(&theirs, &e.exact)
            ));
        }
    }
    Ok(())
}

fn run_emitted(tag: &str, streams: Vec<Emitted>) {
    let scratch = Scratch::new(tag);
    let total = streams.len();
    let mut failures = Vec::new();
    for e in &streams {
        let verdict = check_emitted(&scratch, e);
        if verbose() {
            eprintln!(
                "{tag}: {:<52} {}",
                e.name,
                match &verdict {
                    Ok(()) => "ok".to_string(),
                    Err(x) => format!("FAIL {x}"),
                }
            );
        }
        if let Err(x) = verdict {
            if let Some(dir) = std::env::var_os("WAVPACK_MATRIX_KEEP") {
                let dir = PathBuf::from(dir);
                let _ = std::fs::create_dir_all(&dir);
                let _ = std::fs::write(dir.join(format!("{tag}-{}.wv", e.name)), &e.wv);
                if let Some(c) = &e.wvc {
                    let _ = std::fs::write(dir.join(format!("{tag}-{}.wvc", e.name)), c);
                }
            }
            failures.push(format!("{}: {x}", e.name));
        }
    }
    eprintln!(
        "blackbox_matrix {tag}: {}/{} emitted streams accepted",
        total - failures.len(),
        total
    );
    assert!(
        failures.is_empty(),
        "{} of {total} emitted streams failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

fn floats(bits: &[i32]) -> Vec<f32> {
    bits.iter().map(|&b| f32::from_bits(b as u32)).collect()
}

/// Lossless integer encoders: every entry point × profile × block size
/// × width × content, mono and stereo, plus stamped sample rates.
#[test]
fn ours_lossless_integer() {
    if !tools_present() {
        return;
    }
    let mut out = Vec::new();
    for bits in [8u32, 16, 24, 12, 20] {
        for channels in [1usize, 2] {
            for kind in [
                Content::Music,
                Content::Noise,
                Content::Rails,
                Content::Bursts,
                Content::Silence,
            ] {
                let fmt = Format::int(bits, channels, 44_100);
                let pcm = quantize(&fmt, &content(kind, 5_000, channels, 44_100, 11));
                let bytes = fmt.container_bytes() as u8;
                for block in [0usize, 1_000, 4_096] {
                    let stem = format!("int{bits}-{channels}ch-{kind:?}-blk{block}");
                    let mut push = |name: String, wv: oxideav_wavpack::Result<Vec<u8>>| {
                        out.push(Emitted {
                            name,
                            fmt,
                            wv: wv.expect("encode"),
                            wvc: None,
                            exact: pcm.clone(),
                            hybrid: false,
                        });
                    };
                    if channels == 1 {
                        push(
                            format!("{stem}-raw"),
                            encode_stream_mono(&pcm, block, bytes),
                        );
                        for p in PROFILES {
                            push(
                                format!("{stem}-best-{p:?}"),
                                encode_stream_mono_best(&pcm, block, bytes, p),
                            );
                        }
                        if kind == Content::Music {
                            push(
                                format!("{stem}-smallest"),
                                encode_stream_mono_smallest(&pcm, block, bytes),
                            );
                        }
                    } else {
                        push(
                            format!("{stem}-raw"),
                            encode_stream_stereo(&pcm, block, bytes),
                        );
                        for p in PROFILES {
                            push(
                                format!("{stem}-best-{p:?}"),
                                encode_stream_stereo_best(&pcm, block, bytes, p),
                            );
                        }
                        if kind == Content::Music {
                            push(
                                format!("{stem}-smallest"),
                                encode_stream_stereo_smallest(&pcm, block, bytes),
                            );
                        }
                    }
                }
            }
        }
    }
    // Lengths, including a single frame.
    for frames in [1usize, 2, 3, 17, 129] {
        for channels in [1usize, 2] {
            let fmt = Format::int(16, channels, 44_100);
            let pcm = quantize(&fmt, &content(Content::Music, frames, channels, 44_100, 3));
            let wv = if channels == 1 {
                encode_stream_mono_best(&pcm, 0, 2, DecorrProfile::High)
            } else {
                encode_stream_stereo_best(&pcm, 0, 2, DecorrProfile::High)
            };
            out.push(Emitted {
                name: format!("len{frames}-{channels}ch"),
                fmt,
                wv: wv.expect("encode"),
                wvc: None,
                exact: pcm,
                hybrid: false,
            });
        }
    }
    // Stamped sample rates: table entries and custom ones.
    for rate in [
        6_000u32, 8_000, 22_050, 48_000, 96_000, 192_000, 12_345, 1, 352_800, 16_777_215,
    ] {
        let fmt = Format::int(16, 2, rate);
        let pcm = quantize(&fmt, &content(Content::Music, 3_000, 2, rate, 5));
        let wv = encode_stream_stereo_best(&pcm, 1_000, 2, DecorrProfile::Normal).expect("encode");
        let wv = set_stream_sample_rate(&wv, rate).expect("stamp rate");
        if stream_sample_rate(&wv).expect("rate") != Some(rate) {
            panic!("stamped rate {rate} does not read back");
        }
        out.push(Emitted {
            name: format!("rate{rate}"),
            fmt,
            wv,
            wvc: None,
            exact: pcm,
            hybrid: false,
        });
    }
    run_emitted("ours-lossless-int", out);
}

/// Lossless float / 32-bit integer / multichannel encoders.
#[test]
fn ours_lossless_formats_and_layouts() {
    if !tools_present() {
        return;
    }
    let mut out = Vec::new();
    for channels in [1usize, 2] {
        for kind in [
            Content::Music,
            Content::Noise,
            Content::Rails,
            Content::Bursts,
        ] {
            for p in PROFILES {
                for block in [0usize, 1_500] {
                    let fmt = Format::float(channels, 48_000);
                    let pcm = quantize(&fmt, &content(kind, 4_000, channels, 48_000, 13));
                    let f = floats(&pcm);
                    let wv = if channels == 1 {
                        encode_stream_mono_float(&f, block, p)
                    } else {
                        encode_stream_stereo_float(&f, block, p)
                    };
                    out.push(Emitted {
                        name: format!("float-{channels}ch-{kind:?}-{p:?}-blk{block}"),
                        fmt,
                        wv: wv.expect("encode"),
                        wvc: None,
                        exact: pcm,
                        hybrid: false,
                    });
                    let fmt = Format::int(32, channels, 48_000);
                    let pcm = quantize(&fmt, &content(kind, 4_000, channels, 48_000, 17));
                    let wv = if channels == 1 {
                        encode_stream_mono_int32(&pcm, block, p)
                    } else {
                        encode_stream_stereo_int32(&pcm, block, p)
                    };
                    out.push(Emitted {
                        name: format!("int32-{channels}ch-{kind:?}-{p:?}-blk{block}"),
                        fmt,
                        wv: wv.expect("encode"),
                        wvc: None,
                        exact: pcm,
                        hybrid: false,
                    });
                }
            }
        }
    }
    for channels in 3usize..=8 {
        for block in [0usize, 1_200] {
            let fmt = Format::int(16, channels, 48_000);
            let pcm = quantize(&fmt, &content(Content::Music, 3_000, channels, 48_000, 19));
            let mut push =
                |name: &str, fmt: Format, exact: &[i32], wv: oxideav_wavpack::Result<Vec<u8>>| {
                    out.push(Emitted {
                        name: format!("{channels}ch-{name}-blk{block}"),
                        fmt,
                        wv: wv.expect("encode"),
                        wvc: None,
                        exact: exact.to_vec(),
                        hybrid: false,
                    });
                };
            push(
                "members",
                fmt,
                &pcm,
                encode_multichannel_stream(&pcm, channels, block, 2),
            );
            push(
                "best",
                fmt,
                &pcm,
                encode_multichannel_stream_best(&pcm, channels, block, 2, DecorrProfile::High),
            );
            push(
                "smallest",
                fmt,
                &pcm,
                encode_multichannel_stream_smallest(&pcm, channels, block, 2),
            );
            let fmt24 = Format::int(24, channels, 48_000);
            let pcm24 = quantize(
                &fmt24,
                &content(Content::Music, 3_000, channels, 48_000, 23),
            );
            push(
                "best24",
                fmt24,
                &pcm24,
                encode_multichannel_stream_best(&pcm24, channels, block, 3, DecorrProfile::Normal),
            );
            let ffmt = Format::float(channels, 48_000);
            let fpcm = quantize(&ffmt, &content(Content::Music, 3_000, channels, 48_000, 29));
            push(
                "float",
                ffmt,
                &fpcm,
                encode_multichannel_stream_float(
                    &floats(&fpcm),
                    channels,
                    block,
                    DecorrProfile::Normal,
                ),
            );
            let ifmt = Format::int(32, channels, 48_000);
            let ipcm = quantize(&ifmt, &content(Content::Music, 3_000, channels, 48_000, 31));
            push(
                "int32",
                ifmt,
                &ipcm,
                encode_multichannel_stream_int32(&ipcm, channels, block, DecorrProfile::Normal),
            );
        }
    }
    // Wide layouts: more than 8 member blocks per set needs the long
    // geometry form, up to the 256-channel ceiling.
    for channels in [9usize, 12, 16, 17, 18, 24, 33, 64, 255, 256] {
        let fmt = Format::int(16, channels, 48_000);
        let pcm = quantize(&fmt, &content(Content::Music, 400, channels, 48_000, 67));
        for (name, wv) in [
            ("members", encode_multichannel_stream(&pcm, channels, 0, 2)),
            (
                "best",
                encode_multichannel_stream_best(&pcm, channels, 150, 2, DecorrProfile::Fast),
            ),
        ] {
            out.push(Emitted {
                name: format!("{channels}ch-{name}"),
                fmt,
                wv: wv.expect("encode"),
                wvc: None,
                exact: pcm.clone(),
                hybrid: false,
            });
        }
    }
    let wide = Format::int(16, 20, 48_000);
    let pcm = quantize(&wide, &content(Content::Music, 400, 20, 48_000, 71));
    let enc = encode_multichannel_stream_hybrid(
        &pcm,
        20,
        150,
        2,
        &HybridOptions::from_bits_per_sample(4.0),
    )
    .expect("encode");
    out.push(Emitted {
        name: "20ch-hybrid".to_string(),
        fmt: wide,
        wv: enc.wv,
        wvc: enc.wvc,
        exact: pcm,
        hybrid: true,
    });
    run_emitted("ours-lossless-formats", out);
}

/// Hybrid encoders: bitrate × shaping × stereo coding × profile ×
/// correction, in every sample format and layout.
#[test]
fn ours_hybrid() {
    if !tools_present() {
        return;
    }
    let mut out = Vec::new();
    let shapings = [
        ("s0", HybridShaping::Off),
        ("s+0.5", HybridShaping::from_weight(0.5)),
        ("s-0.5", HybridShaping::from_weight(-0.5)),
        ("s+1", HybridShaping::from_weight(1.0)),
        ("s-1", HybridShaping::from_weight(-1.0)),
        (
            "ramp",
            HybridShaping::Ramp {
                weight: -300,
                delta: 4_000,
            },
        ),
    ];
    for channels in [1usize, 2] {
        for bits_per_sample in [2.0f64, 3.0, 4.5, 8.0, 12.0] {
            for (sname, shaping) in shapings {
                for joint in [false, true] {
                    if channels == 1 && joint {
                        continue;
                    }
                    for profile in [
                        None,
                        Some(DecorrProfile::Fast),
                        Some(DecorrProfile::Normal),
                        Some(DecorrProfile::Extra),
                    ] {
                        for correction in [false, true] {
                            let kind = if joint {
                                Content::Music
                            } else {
                                Content::Bursts
                            };
                            let fmt = Format::int(16, channels, 44_100);
                            let pcm = quantize(&fmt, &content(kind, 3_000, channels, 44_100, 37));
                            let opts = HybridOptions::from_bits_per_sample(bits_per_sample)
                                .with_correction(correction)
                                .with_joint(joint)
                                .with_profile(profile)
                                .with_shaping(shaping);
                            let enc = if channels == 1 {
                                encode_stream_mono_hybrid(&pcm, 1_100, 2, &opts)
                            } else {
                                encode_stream_stereo_hybrid(&pcm, 1_100, 2, &opts)
                            }
                            .expect("encode");
                            out.push(Emitted {
                                name: format!(
                                    "{channels}ch-b{bits_per_sample}-{sname}-j{}-{profile:?}-c{}",
                                    u8::from(joint),
                                    u8::from(correction)
                                ),
                                fmt,
                                wv: enc.wv,
                                wvc: enc.wvc,
                                exact: pcm,
                                hybrid: true,
                            });
                        }
                    }
                }
            }
        }
    }
    // Other widths and content, float / int32, multichannel.
    for bits in [8u32, 24] {
        for channels in [1usize, 2] {
            for kind in [
                Content::Music,
                Content::Noise,
                Content::Rails,
                Content::Silence,
            ] {
                let fmt = Format::int(bits, channels, 44_100);
                let pcm = quantize(&fmt, &content(kind, 3_000, channels, 44_100, 41));
                let opts = HybridOptions::from_bits_per_sample(3.0)
                    .with_shaping(HybridShaping::from_weight(-0.4));
                let bytes = fmt.container_bytes() as u8;
                let enc = if channels == 1 {
                    encode_stream_mono_hybrid(&pcm, 1_000, bytes, &opts)
                } else {
                    encode_stream_stereo_hybrid(&pcm, 1_000, bytes, &opts)
                }
                .expect("encode");
                out.push(Emitted {
                    name: format!("int{bits}-{channels}ch-{kind:?}"),
                    fmt,
                    wv: enc.wv,
                    wvc: enc.wvc,
                    exact: pcm,
                    hybrid: true,
                });
            }
        }
    }
    for channels in [1usize, 2] {
        for kind in [Content::Music, Content::Noise] {
            let opts = HybridOptions::from_bits_per_sample(5.0);
            let fmt = Format::float(channels, 44_100);
            let pcm = quantize(&fmt, &content(kind, 3_000, channels, 44_100, 43));
            let f = floats(&pcm);
            let enc = if channels == 1 {
                encode_stream_mono_hybrid_float(&f, 1_000, &opts)
            } else {
                encode_stream_stereo_hybrid_float(&f, 1_000, &opts)
            }
            .expect("encode");
            out.push(Emitted {
                name: format!("float-{channels}ch-{kind:?}"),
                fmt,
                wv: enc.wv,
                wvc: enc.wvc,
                exact: pcm,
                hybrid: true,
            });
            let fmt = Format::int(32, channels, 44_100);
            let pcm = quantize(&fmt, &content(kind, 3_000, channels, 44_100, 47));
            let enc = if channels == 1 {
                encode_stream_mono_hybrid_int32(&pcm, 1_000, &opts)
            } else {
                encode_stream_stereo_hybrid_int32(&pcm, 1_000, &opts)
            }
            .expect("encode");
            out.push(Emitted {
                name: format!("int32-{channels}ch-{kind:?}"),
                fmt,
                wv: enc.wv,
                wvc: enc.wvc,
                exact: pcm,
                hybrid: true,
            });
        }
    }
    for channels in 3usize..=8 {
        for (sname, shaping) in [
            ("s0", HybridShaping::Off),
            ("s-0.5", HybridShaping::from_weight(-0.5)),
        ] {
            let opts = HybridOptions::from_bits_per_sample(4.0).with_shaping(shaping);
            let fmt = Format::int(16, channels, 48_000);
            let pcm = quantize(&fmt, &content(Content::Music, 2_500, channels, 48_000, 53));
            let enc =
                encode_multichannel_stream_hybrid(&pcm, channels, 1_000, 2, &opts).expect("encode");
            out.push(Emitted {
                name: format!("{channels}ch-{sname}"),
                fmt,
                wv: enc.wv,
                wvc: enc.wvc,
                exact: pcm,
                hybrid: true,
            });
        }
        let opts = HybridOptions::from_bits_per_sample(5.0);
        let fmt = Format::float(channels, 48_000);
        let pcm = quantize(&fmt, &content(Content::Music, 2_500, channels, 48_000, 59));
        let enc = encode_multichannel_stream_hybrid_float(&floats(&pcm), channels, 1_000, &opts)
            .expect("encode");
        out.push(Emitted {
            name: format!("{channels}ch-float"),
            fmt,
            wv: enc.wv,
            wvc: enc.wvc,
            exact: pcm,
            hybrid: true,
        });
        let fmt = Format::int(32, channels, 48_000);
        let pcm = quantize(&fmt, &content(Content::Music, 2_500, channels, 48_000, 61));
        let enc =
            encode_multichannel_stream_hybrid_int32(&pcm, channels, 1_000, &opts).expect("encode");
        out.push(Emitted {
            name: format!("{channels}ch-int32"),
            fmt,
            wv: enc.wv,
            wvc: enc.wvc,
            exact: pcm,
            hybrid: true,
        });
    }
    run_emitted("ours-hybrid", out);
}

// ---------------------------------------------------------------------
// DSD: foreign → ours
// ---------------------------------------------------------------------

/// 1-bit content for one channel, packed eight samples per byte, most
/// significant bit first.
fn dsd_channel(kind: Content, bytes: usize, native_rate: u32, channel: usize) -> Vec<u8> {
    let mut rng = Lcg(0xD5D ^ (channel as u64) << 8);
    let mut integrators = (0.0f64, 0.0f64);
    let mut out = Vec::with_capacity(bytes);
    for n in 0..bytes {
        let mut byte = 0u8;
        for k in 0..8 {
            let index = n * 8 + k;
            let bit = match kind {
                // DSD silence: the alternating pattern.
                Content::Silence => index & 1 == 1,
                Content::Noise => rng.next() & 0x8000 != 0,
                _ => {
                    // Second-order sigma-delta modulation of a tone
                    // (with silent stretches for the burst kind).
                    let t = index as f64 / f64::from(native_rate);
                    let quiet = kind == Content::Bursts && (n / 3_000) % 2 == 1;
                    let x = if quiet {
                        0.0
                    } else {
                        0.5 * (std::f64::consts::TAU * 1_000.0 * (channel + 1) as f64 * t).sin()
                    };
                    let y = if integrators.1 >= 0.0 { 1.0 } else { -1.0 };
                    integrators.0 += x - y;
                    integrators.1 += integrators.0 - y;
                    y > 0.0
                }
            };
            byte = (byte << 1) | u8::from(bit);
        }
        out.push(byte);
    }
    out
}

/// Write a DSDIFF container around interleaved DSD bytes.
fn write_dff(path: &Path, channels: usize, native_rate: u32, interleaved: &[u8]) {
    fn chunk(id: &[u8; 4], body: &[u8]) -> Vec<u8> {
        let mut c = id.to_vec();
        c.extend_from_slice(&(body.len() as u64).to_be_bytes());
        c.extend_from_slice(body);
        if body.len() & 1 == 1 {
            c.push(0);
        }
        c
    }
    let mut chnl = (channels as u16).to_be_bytes().to_vec();
    for ch in 0..channels {
        let id: [u8; 4] = match (channels, ch) {
            (2, 0) => *b"SLFT",
            (2, 1) => *b"SRGT",
            _ => {
                let s = format!("C{ch:03}");
                s.as_bytes().try_into().expect("four bytes")
            }
        };
        chnl.extend_from_slice(&id);
    }
    let mut cmpr = b"DSD ".to_vec();
    cmpr.push(14);
    cmpr.extend_from_slice(b"not compressed");
    cmpr.push(0);
    let mut prop = b"SND ".to_vec();
    prop.extend(chunk(b"FS  ", &native_rate.to_be_bytes()));
    prop.extend(chunk(b"CHNL", &chnl));
    prop.extend(chunk(b"CMPR", &cmpr));
    let mut body = b"DSD ".to_vec();
    body.extend(chunk(b"FVER", &0x0105_0000u32.to_be_bytes()));
    body.extend(chunk(b"PROP", &prop));
    body.extend(chunk(b"DSD ", interleaved));
    let mut file = b"FRM8".to_vec();
    file.extend_from_slice(&(body.len() as u64).to_be_bytes());
    file.extend(body);
    std::fs::write(path, file).expect("write dff");
}

/// One DSD case: content kind, channel count, bytes per channel, the
/// 1-bit sample rate and the encoder options.
struct DsdCase {
    name: String,
    kind: Content,
    channels: usize,
    bytes: usize,
    rate: u32,
    mode: &'static [&'static str],
}

/// DSD streams from the outside encoder — both compression modes, the
/// stored fallback (noise and tiny blocks), 1..6 channels, three
/// rates, lengths around the 280-byte coded-block threshold — must
/// decode to the source bytes with every block CRC matching.
#[test]
fn foreign_dsd() {
    if !tools_present() {
        return;
    }
    let scratch = Scratch::new("dsd");
    let mut cases: Vec<DsdCase> = Vec::new();
    let mut push = |stem: String, kind, channels, bytes, rate| {
        for mode in [&[][..], &["-h"][..]] {
            cases.push(DsdCase {
                name: format!("{stem}{}", mode.first().copied().unwrap_or("")),
                kind,
                channels,
                bytes,
                rate,
                mode,
            });
        }
    };
    for kind in [
        Content::Music,
        Content::Bursts,
        Content::Noise,
        Content::Silence,
    ] {
        for channels in [1usize, 2, 3, 6] {
            push(
                format!("{kind:?}-{channels}ch"),
                kind,
                channels,
                24_000,
                2_822_400,
            );
        }
    }
    for bytes in [
        1usize, 2, 139, 140, 141, 279, 280, 281, 4_095, 4_096, 4_097, 90_000,
    ] {
        for channels in [1usize, 2] {
            push(
                format!("len{bytes}-{channels}ch"),
                Content::Music,
                channels,
                bytes,
                2_822_400,
            );
        }
    }
    for rate in [5_644_800u32, 11_289_600, 3_072_000, 1_000_000, 2_822_408] {
        push(format!("rate{rate}"), Content::Music, 2, 12_000, rate);
    }
    let total = cases.len();
    let mut failures = Vec::new();
    let mut modes_seen = [0usize; 4];
    for DsdCase {
        name,
        kind,
        channels,
        bytes,
        rate,
        mode,
    } in &cases
    {
        let verdict = (|| -> Result<(), String> {
            let per_channel: Vec<Vec<u8>> = (0..*channels)
                .map(|ch| dsd_channel(*kind, *bytes, *rate, ch))
                .collect();
            let mut source = Vec::with_capacity(bytes * channels);
            for n in 0..*bytes {
                for ch in &per_channel {
                    source.push(ch[n]);
                }
            }
            let dff = scratch.path("in.dff");
            let wv = scratch.path("out.wv");
            let raw = scratch.path("ref.raw");
            let _ = std::fs::remove_file(&wv);
            let _ = std::fs::remove_file(&raw);
            write_dff(&dff, *channels, *rate, &source);
            let mut cmd = Command::new("wavpack");
            cmd.args(["-q", "-y"])
                .args(*mode)
                .arg(&dff)
                .arg("-o")
                .arg(&wv);
            run(&mut cmd).map_err(|e| format!("encoder refused the case: {e}"))?;
            let mut cmd = Command::new("wvunpack");
            cmd.args(["-q", "-y", "--raw"]).arg(&wv).arg("-o").arg(&raw);
            run(&mut cmd)?;
            let reference = std::fs::read(&raw).map_err(|e| format!("read raw: {e}"))?;
            if reference != source {
                return Err("the outside decoder does not return the source bytes".to_string());
            }
            let wv_bytes = std::fs::read(&wv).map_err(|e| format!("read wv: {e}"))?;
            for block in oxideav_wavpack::iter_blocks(&wv_bytes) {
                let block = block.map_err(|e| format!("block parse: {e}"))?;
                if let Some(header) = block
                    .dsd_header()
                    .map_err(|e| format!("0x0E prefix: {e}"))?
                {
                    modes_seen[usize::from(header.mode)] += 1;
                }
            }
            let (decoded, ok) = decode_multichannel_stream_muted(&wv_bytes)
                .map_err(|e| format!("decode error: {e}"))?;
            if !ok {
                return Err("a stored block CRC did not match".to_string());
            }
            if decoded.channels != *channels {
                return Err(format!("channel count {}", decoded.channels));
            }
            let want: Vec<i32> = source.iter().map(|&b| i32::from(b)).collect();
            if decoded.samples != want {
                return Err(format!(
                    "decode differs from the source: {}",
                    first_diff(&decoded.samples, &want)
                ));
            }
            // The stream rate is the byte rate: one eighth of the 1-bit rate.
            let byte_rate = stream_sample_rate(&wv_bytes).map_err(|e| format!("rate: {e}"))?;
            if byte_rate != Some(rate / 8) {
                return Err(format!("byte rate {byte_rate:?}, expected {}", rate / 8));
            }
            Ok(())
        })();
        if verbose() {
            eprintln!(
                "dsd: {name:<32} {}",
                match &verdict {
                    Ok(()) => "ok".to_string(),
                    Err(e) => format!("FAIL {e}"),
                }
            );
        }
        if let Err(e) = verdict {
            failures.push(format!("{name}: {e}"));
        }
    }
    eprintln!(
        "blackbox_matrix dsd: {}/{} cases decode to the source (blocks seen: {} stored, {} fast, {} high)",
        total - failures.len(),
        total,
        modes_seen[0],
        modes_seen[1],
        modes_seen[3]
    );
    assert!(
        failures.is_empty(),
        "{} of {total} DSD cases failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
    assert!(
        modes_seen[0] > 0 && modes_seen[1] > 0 && modes_seen[3] > 0,
        "the sweep must reach all three payload modes"
    );
}

/// The framework path: frames pushed through the registered encoder in
/// uneven pieces, the packets concatenated into one file. An outside
/// reader must accept the result like any stream the crate-level
/// encoders write (the total length is unknown when a stream is
/// written packet by packet, so the header says so).
#[test]
fn ours_framework_encoder() {
    use oxideav_core::{
        AudioFrame, CodecId, CodecParameters, Error as CoreError, Frame, SampleFormat,
    };

    if !tools_present() {
        return;
    }
    let mut out = Vec::new();
    for (name, sample_format, fmt, options) in [
        (
            "s16-stereo",
            SampleFormat::S16,
            Format::int(16, 2, 48_000),
            &[][..],
        ),
        (
            "s8-mono",
            SampleFormat::S8,
            Format::int(8, 1, 22_050),
            &[][..],
        ),
        (
            "s24-5.1",
            SampleFormat::S24,
            Format::int(24, 6, 96_000),
            &[][..],
        ),
        (
            "s32-mono",
            SampleFormat::S32,
            Format::int(32, 1, 44_100),
            &[][..],
        ),
        (
            "f32-stereo",
            SampleFormat::F32,
            Format::float(2, 12_345),
            &[][..],
        ),
        (
            "f32-quad",
            SampleFormat::F32,
            Format::float(4, 48_000),
            &[][..],
        ),
        (
            "s16-stereo-hybrid",
            SampleFormat::S16,
            Format::int(16, 2, 44_100),
            &[
                ("mode", "hybrid"),
                ("bits_per_sample", "3.5"),
                ("shaping", "-0.5"),
            ][..],
        ),
        (
            "s16-5.1-hybrid",
            SampleFormat::S16,
            Format::int(16, 6, 48_000),
            &[("mode", "hybrid"), ("joint", "false")][..],
        ),
    ] {
        let pcm = quantize(
            &fmt,
            &content(Content::Bursts, 5_000, fmt.channels, fmt.rate, 73),
        );
        let mut params = CodecParameters::audio(CodecId::new("wavpack"));
        params.channels = Some(fmt.channels as u16);
        params.sample_format = Some(sample_format);
        params.sample_rate = Some(fmt.rate);
        for (key, value) in options {
            params.options.insert(*key, *value);
        }
        let mut encoder = oxideav_wavpack::encoder::make_encoder(&params).expect("encoder");
        let width = fmt.container_bytes();
        let mut wv = Vec::new();
        let mut at = 0usize;
        // Uneven frame sizes, a one-frame piece among them.
        for frames in [1_700usize, 1, 2_048, 1_251] {
            let slice = &pcm[at * fmt.channels..(at + frames) * fmt.channels];
            let mut data = Vec::with_capacity(slice.len() * width);
            for s in slice {
                data.extend_from_slice(&s.to_le_bytes()[..width]);
            }
            encoder
                .send_frame(&Frame::Audio(AudioFrame {
                    samples: frames as u32,
                    pts: Some(at as i64),
                    data: vec![data],
                }))
                .expect("send frame");
            at += frames;
        }
        encoder.flush().expect("flush");
        loop {
            match encoder.receive_packet() {
                Ok(packet) => wv.extend_from_slice(&packet.data),
                Err(CoreError::Eof) | Err(CoreError::NeedMore) => break,
                Err(e) => panic!("receive_packet: {e}"),
            }
        }
        let hybrid = !options.is_empty();
        out.push(Emitted {
            name: name.to_string(),
            fmt,
            wv,
            wvc: None,
            exact: pcm,
            hybrid,
        });
    }
    run_emitted("ours-framework", out);
}
