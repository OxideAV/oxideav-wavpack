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
    first_audio_block, stream_channel_info, stream_sample_rate,
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
            n => (1u32 << n.min(18)) - 1,
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
            let info = stream_channel_info(&wv_bytes).map_err(|e| format!("channel info: {e}"))?;
            let Some(info) = info else {
                return Err("no channel geometry on a multichannel stream".to_string());
            };
            if usize::from(info.count) != fmt.channels {
                return Err(format!("geometry count {}", info.count));
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
    for channels in 1usize..=12 {
        for (bits, float) in [(16u32, false), (24, false), (32, true)] {
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
                        5_000,
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
