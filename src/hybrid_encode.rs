//! Hybrid (lossy + optional correction) **origination** — the encode
//! direction of the staged spec §6.5 `error_limit` model and the §4.1
//! two-file lossless recovery (round 418).
//!
//! The decode side of every piece used here was pinned in rounds
//! 408/415 (see `src/hybrid.rs`, `src/samples.rs` and the pair-decode
//! legs in `src/block.rs`); this module drives the same arithmetic in
//! the forward direction:
//!
//! * **Bracketed entropy write.** Each sample word's §4.2 ladder is
//!   unchanged (zone selector from the exact residual's magnitude,
//!   pre-adapt interval, §3.2 adaptation), but step 6 becomes the §6.5
//!   bracketing binary search: while `high - low > error_limit` the
//!   encoder emits the decision bit `exact_mag >= mid`, the decoder's
//!   midpoint of the final bracket is the **coarse** residual, and —
//!   when a correction stream is requested — the exact in-bracket
//!   offset is written to the `0x0B` stream with the same phase-in
//!   code the lossless mantissa uses (the round-408 pin). The sign bit
//!   follows in the main stream, shared by exact and coarse.
//! * **Decoder-state feedback.** The decoder reconstructs (and adapts
//!   its prediction weights on) the *coarse* values, so the encoder
//!   tracks the decoder's exact state via the round-418
//!   [`crate::decorrelation::MonoStepper`] /
//!   [`crate::decorrelation::StereoStepper`]: the per-sample exact
//!   residual is `sample − offset` where `offset` is the additive
//!   prediction the decoder will apply from its coarse history, and
//!   every coarse decision is folded back into the stepper (and into
//!   the §6.5 `slow_level` recurrence) exactly as the decoder folds
//!   it. Stereo term `-2` cannot be driven this way (channel A's
//!   stream decision would need the same frame's B) and is filtered /
//!   refused; `-1` and `-3` are driveable.
//! * **The `0x06` profile.** The per-block seed is derived from the
//!   data: the level words are the running `slow_level` (packed as
//!   log words; carried across blocks by the stream encoders, seeded
//!   for the first block by a §6.5 pre-pass over a lossless-residual
//!   estimate), the bitrate word comes from the caller (the
//!   [`HybridOptions::from_bits_per_sample`] mapping
//!   `max(0, bits·256 − 568)` mirrors the documented `-b` range
//!   observation), and the balance word is `256` on joint-stereo
//!   blocks / `0` otherwise (the round-408 observation).
//! * **Header flags.** `HYBRID_FLAG` (bit 3) plus the bits-9/10
//!   hybrid-profile sub-flags as observed on every reference hybrid
//!   block (bit 9 always, bit 10 on stereo). With shaping off (bits
//!   6/29 clear, no `0x07`) the raw §4.1 fold applies — the shape the
//!   reference's shaping-off mode produces.
//! * **Noise shaping** ([`HybridShaping`], round 420). When selected,
//!   every §6.5-bracketed word targets the **shaped** residual
//!   `exact - temp` (the spec §4.1 error-feedback term, recomputed in
//!   exact decoder lockstep via [`crate::ShapingState`]), tilting the
//!   lossy stream's quantization-noise spectrum by the weight; the
//!   `.wvc` twin leads with the block's `0x07` seed so the pair decode
//!   re-derives the same temps and stays bit-exact. Joint (mid/side)
//!   blocks run the filter per **output** channel with the round-415
//!   coded-domain temp transform. Header bits 6/29 are both set,
//!   matching every reference shaped block.
//! * **The `.wvc` twin.** Same 32-byte header fields as the `.wv`
//!   block except the CRC, which stores the §5 running CRC of the
//!   **lossless** decode (round-415 pin), then the `0x0B` correction
//!   payload (even-padded like every packed bitstream). A float /
//!   int32 pair encode moves the `0x0C` extension payload to the
//!   `.wvc` twin (round-415 structural pin) and the lossy `.wv` keeps
//!   only the profile sub-block (implied-zero fill on a `.wv`-only
//!   decode).
//!
//! Validation contract: the `.wv` alone decodes to the coarse PCM this
//! encoder computed (bit-exact through this crate's decoder and the
//! reference binary), and `.wv` + `.wvc` decode to the original input
//! bit-exactly.

use crate::decorrelation::{
    assemble_mono_passes, assemble_stereo_passes, recorrelate_mono, recorrelate_stereo,
    serialize_mono_passes, serialize_stereo_passes, DecorrPass, MonoStepper, StereoStepper,
};
use crate::encode::{
    append_sub_block, base_flags, build_block, forward_joint_stereo, magnitude_bits,
    pack_entropy_info, with_marker, DecorrProfile, FormatExtras, DEFAULT_BLOCK_SAMPLES,
};
use crate::error::{Error, Result};
use crate::hybrid::{HybridProfile, HybridState, HYBRID_FLAG};
use crate::metadata::SubBlockId;
use crate::samples::{
    emit_raw_prefix, emit_zero_run_length, split_sign, AdaptiveMedians, BitWriter, RunState,
    SampleInterval, Zone,
};

/// Wiki flag bit 9 — the first hybrid-profile sub-flag. Set on every
/// reference-encoded hybrid block (round-418 observation over the
/// round-408/415 fixture battery); mirrored on originated blocks.
const HYBRID_PROFILE_FLAG_9: u32 = 1 << 9;
/// Wiki flag bit 10 — the second hybrid-profile sub-flag. Set on every
/// reference-encoded **stereo** hybrid block (left/right and joint
/// alike); mirrored on originated stereo blocks.
const HYBRID_PROFILE_FLAG_10: u32 = 1 << 10;

/// The stereo balance word observed on reference joint (mid/side)
/// hybrid profiles (`0` on left/right profiles) — round-408 pin.
const JOINT_BALANCE_WORD: i32 = 256;

/// The empirical bitrate-word mapping of the reference `-b` range
/// (documented with the round-408 `0x06` pin): the log-domain word is
/// `bits_per_sample * 256 - 568`, floored at 0 (0 = the lossless
/// degenerate limit).
const BITRATE_WORD_BIAS: i32 = 568;

/// Noise-shaping selection for a hybrid encode (round 420) — the
/// encode side of the `0x07` `ID_SHAPING_WEIGHTS` filter whose decode
/// recurrence was pinned in rounds 408/415 ([`crate::ShapingState`]).
///
/// When shaping is on, each §6.5-bracketed word targets the **shaped**
/// residual `exact - temp` instead of the exact residual, where `temp`
/// is the error-feedback term of the staged spec §4.1 recurrence
/// (`shaping_acc += shaping_delta; weight = acc >> 16;
/// temp = -apply_weight(weight, error)`). The lossy decode then
/// carries quantization noise whose spectrum is tilted by the weight
/// (positive weights push noise upward — the reference `-s0.7` shape;
/// negative weights the other way), while the `.wvc` pair decode stays
/// bit-exact: the correction stream stores the shaped in-bracket value
/// and the decoder re-derives the same `temp` from its `0x07` seed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HybridShaping {
    /// No shaping (the reference `-s0` shape): no `0x07` sub-block, no
    /// shape flag bits, raw §4.1 correction fold.
    Off,
    /// Constant shaping weight in 1/1024 units (`±1024` = `±1.0`,
    /// clamped; the reference `-s<w>` static shape). The `0x07`
    /// payload carries the short (delta-free) per-channel layout.
    Static(i32),
    /// Linearly ramping weight: `weight` (1/1024 units) seeds the
    /// accumulator and `delta` is the raw per-sample accumulator
    /// increment (`acc += delta` each sample, `weight = acc >> 16`).
    /// The `0x07` payload carries the long (delta-bearing) layout.
    /// The accumulator carries across blocks, so a non-zero `delta`
    /// keeps ramping for the whole stream — callers pick deltas sized
    /// to the stream length (the pair decode is lossless regardless;
    /// only the lossy noise spectrum follows the weight).
    Ramp {
        /// Starting weight in 1/1024 units (clamped to `±1024`).
        weight: i32,
        /// Raw per-sample accumulator increment (`acc` domain — the
        /// weight moves by `delta / 65536` thousand-twenty-fourths per
        /// sample). The emitted per-block delta words are
        /// **rail-saturated**: each block's delta is shrunk (down to
        /// `0`) so the accumulator can never leave the full-scale
        /// `±1024` weight range inside the block — the ramp runs to
        /// the rail and holds there. Black-box finding behind the
        /// rail: trajectories that cross **below** `-1024` can decode
        /// differently under the reference decoder (the staged spec
        /// names an IIR variant for negative weights whose exact
        /// out-of-range recurrence is an open docs gap), while the
        /// validated envelope — every static weight in `±1024` and
        /// every in-range ramp — is bit-exact under it.
        delta: i32,
    },
}

impl HybridShaping {
    /// Shaping selection from a fractional weight (the reference `-s`
    /// scale, `-1.0..=1.0`): `0.0` is [`Self::Off`], anything else a
    /// [`Self::Static`] weight of `round(w * 1024)`.
    #[must_use]
    pub fn from_weight(weight: f64) -> Self {
        let units = (weight.clamp(-1.0, 1.0) * 1024.0).round() as i32;
        if units == 0 {
            HybridShaping::Off
        } else {
            HybridShaping::Static(units)
        }
    }

    /// The initial `0x07` payload for this selection (`None` when
    /// shaping is off). The accumulator seed is the clamped weight
    /// shifted into the `acc` domain and log-word quantized; the error
    /// seed is zero.
    fn initial_payload(self, stereo: bool) -> Option<Vec<u8>> {
        let (weight, delta) = match self {
            HybridShaping::Off => return None,
            HybridShaping::Static(w) => (w, None),
            HybridShaping::Ramp { weight, delta } => (weight, Some(delta)),
        };
        let acc = weight.clamp(-1024, 1024) << 16;
        let mut payload = Vec::with_capacity(12);
        let channels = if stereo { 2 } else { 1 };
        for _ in 0..channels {
            payload.extend_from_slice(&crate::logpack::pack_log_word(0));
            payload.extend_from_slice(&crate::logpack::pack_log_word(acc));
        }
        if let Some(delta) = delta {
            for _ in 0..channels {
                payload.extend_from_slice(&crate::logpack::pack_log_word(delta));
            }
        }
        Some(payload)
    }

    /// Whether the on-wire `0x07` layout carries the per-channel
    /// `delta` words.
    fn with_delta(self) -> bool {
        matches!(self, HybridShaping::Ramp { .. })
    }
}

/// Options for one hybrid encode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HybridOptions {
    /// The `0x06` log-domain bitrate word (8 fractional bits). Higher
    /// = more bits = less noise; `0` is the coarsest documented floor
    /// (the reference `-b2` profile), and a word at or above
    /// `ema + 256` drives every §6.5 limit argument non-positive —
    /// the lossless degenerate case. Build from a bits-per-sample
    /// target with [`Self::from_bits_per_sample`].
    pub bitrate_word: i32,
    /// Emit the `.wvc` correction twin (the two-file lossless mode).
    pub correction: bool,
    /// Stereo blocks: joint (mid/side) coding. Ignored for mono.
    pub joint: bool,
    /// Self-derived decorrelation search ceiling for the coded domain;
    /// `None` encodes raw (no prediction).
    pub profile: Option<DecorrProfile>,
    /// Noise shaping for the lossy stream (round 420). [`Off`]
    /// (`HybridShaping::Off`) reproduces the raw unshaped fold.
    ///
    /// [`Off`]: HybridShaping::Off
    pub shaping: HybridShaping,
}

impl HybridOptions {
    /// Options for a bits-per-sample target (the reference `-b` scale:
    /// ~2.0 aggressive … 6.0+ near-lossless), with a correction twin,
    /// joint stereo coding, the `Normal` decorrelation ceiling and no
    /// noise shaping.
    #[must_use]
    pub fn from_bits_per_sample(bits: f64) -> Self {
        let word = (bits * 256.0).round() as i32 - BITRATE_WORD_BIAS;
        HybridOptions {
            bitrate_word: word.clamp(0, i32::from(i16::MAX)),
            correction: true,
            joint: true,
            profile: Some(DecorrProfile::Normal),
            shaping: HybridShaping::Off,
        }
    }

    /// This option set with the given [`HybridShaping`] selection.
    #[must_use]
    pub fn with_shaping(mut self, shaping: HybridShaping) -> Self {
        self.shaping = shaping;
        self
    }
}

impl Default for HybridOptions {
    /// The reference `-b4`-shaped default (bitrate word 456).
    fn default() -> Self {
        HybridOptions::from_bits_per_sample(4.0)
    }
}

/// One hybrid encode's output: the lossy `.wv` bytes and (when
/// requested) the `.wvc` correction twin that restores losslessness.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HybridEncoded {
    /// The lossy main stream — a standalone `.wv` file/chain.
    pub wv: Vec<u8>,
    /// The correction stream — the companion `.wvc` file/chain —
    /// present when [`HybridOptions::correction`] was set.
    pub wvc: Option<Vec<u8>>,
}

/// A planned sample word awaiting its §4.2 step-4 holding-bit
/// resolution (the prefix's low bit pre-encodes the *next* word's
/// mode, so emission trails planning by one word).
struct PendingWord {
    /// The word's zone selector (folded ones count).
    zone: u32,
    /// Pre-rendered post-prefix bits: bracket decisions (or the
    /// lossless mantissa) followed by the sign bit.
    body: Vec<u8>,
    /// Number of valid bits in `body`.
    body_bits: usize,
}

/// Append `bits` bits of `bytes` (LSB-first stream order) to `writer`.
fn append_bits(writer: &mut BitWriter, bytes: &[u8], bits: usize) {
    for i in 0..bits {
        writer.write_bit((u32::from(bytes[i / 8]) >> (i % 8)) & 1);
    }
}

/// Emit a pending word: prefix (unless the previous word's clear low
/// bit pre-encoded this word's zone as 0) then the pre-rendered body.
/// `next` is the following stream word's `(magnitude, working med0)`
/// pair driving the holding-bit choice, `None` at end of block.
fn emit_pending(
    writer: &mut BitWriter,
    run: &mut RunState,
    pending: PendingWord,
    next: Option<(u32, u32)>,
) -> Result<()> {
    if run.last_zero {
        debug_assert_eq!(pending.zone, 0, "pre-encoded words must be zone 0");
        run.last_zero = false;
    } else {
        let hold_one = match next {
            None => false,
            Some((mag, med0)) => mag >= med0,
        };
        match run.unfold_prefix(pending.zone, hold_one) {
            Some(raw) => emit_raw_prefix(writer, raw),
            None => {
                return Err(Error::ValueNotInInterval {
                    value: pending.zone,
                    low: 0,
                    high: 0,
                })
            }
        }
    }
    append_bits(writer, &pending.body, pending.body_bits);
    Ok(())
}

/// Plan one bracketed word: write the §6.5 bracket decision bits (or
/// the lossless mantissa when `limit == 0`) plus the sign into a fresh
/// body buffer, append the exact in-bracket offset to the correction
/// writer, and return `(pending, coarse_residual)`.
fn plan_word(
    interval: &SampleInterval,
    zone: u32,
    exact: i32,
    limit: u32,
    wvc: Option<&mut BitWriter>,
) -> Result<(PendingWord, i32)> {
    let (mag, neg) = split_sign(exact);
    let mut body = BitWriter::new();
    let coarse = if limit == 0 {
        interval.encode_signed_value(&mut body, exact)?;
        exact
    } else {
        let mut low = interval.low;
        let mut high = interval.high;
        while high - low > limit {
            let mid = (low + high + 1) >> 1;
            if mag >= mid {
                body.write_bit(1);
                low = mid;
            } else {
                body.write_bit(0);
                high = mid - 1;
            }
        }
        let coarse_mag = (low + high + 1) >> 1;
        if let Some(wvc) = wvc {
            SampleInterval::new(low, high).encode_value(wvc, mag)?;
        }
        body.write_bit(u32::from(neg));
        if neg {
            !(coarse_mag as i32)
        } else {
            coarse_mag as i32
        }
    };
    let body_bits = body.bits_written();
    Ok((
        PendingWord {
            zone,
            body: body.finish(),
            body_bits,
        },
        coarse,
    ))
}

/// Pad a finished bitstream payload to an even byte count (the packed
/// bitstream sub-blocks — `0x0A`/`0x0B`/`0x0C` — are bound as 16-bit
/// words; round-393/418 pins).
fn finish_even(writer: BitWriter) -> Vec<u8> {
    let mut bytes = writer.finish();
    if bytes.len() % 2 != 0 {
        bytes.push(0);
    }
    bytes
}

/// The §6.5 `slow_level` recurrence over a residual-magnitude
/// pre-pass, used to seed the first block's level words.
fn level_prepass(residuals: &[i32], channels: usize) -> [u32; 2] {
    let mut sl = [0u32; 2];
    for (i, &r) in residuals.iter().enumerate() {
        let ch = if channels == 2 { i & 1 } else { 0 };
        let (mag, _) = split_sign(r);
        sl[ch] = sl[ch] - ((sl[ch] + 128) >> 8) + crate::logpack::wp_log2(mag) as u32;
    }
    sl
}

/// Pack a linear `slow_level` into its on-wire log word (the value the
/// decoder — and this encoder — will expand with `wp_exp2s`).
fn pack_level_word(slow_level: u32) -> i16 {
    let clamped = slow_level.min(i32::MAX as u32) as i32;
    let [lo, hi] = crate::logpack::pack_log_word(clamped);
    i16::from_le_bytes([lo, hi])
}

/// Derive (and wire-quantize) the decorrelation pass list for one
/// hybrid block over the coded-domain samples, dropping any stereo
/// `-2` pass (not driveable in the encode direction — see
/// [`crate::decorrelation::StereoStepper`]). Returns the serialized
/// `0x02`/`0x03`/`0x04` payloads plus the **assembled**
/// (decoder-identical) pass list.
#[allow(clippy::type_complexity)]
fn derive_hybrid_passes(
    coded: &[i32],
    mono: bool,
    profile: Option<DecorrProfile>,
) -> Result<(Option<(Vec<u8>, Vec<u8>, Vec<u8>)>, Vec<DecorrPass>)> {
    let Some(profile) = profile else {
        return Ok((None, Vec::new()));
    };
    let mut passes = if mono {
        crate::encode::derive_mono_passes(coded, profile)?
    } else {
        crate::encode::derive_stereo_passes(coded, profile)?
    };
    // Hybrid stacks are cross-free. Round-418 black-box probe: a
    // hybrid block carrying a `-1` cross pass decodes *differently*
    // under the reference decoder than under the lossless-identical
    // decorrelation model (its lossy decode fails the stored CRC),
    // and reference-encoded hybrid streams never carry cross terms
    // (their stereo stacks match the mono ones) — so the encode side
    // stays inside the reference-validated shape. (`-2` additionally
    // cannot be driven in the encode direction at all — see
    // [`StereoStepper`].)
    passes.retain(|p| !crate::decorrelation::is_cross_term(p.term));
    if passes.is_empty() {
        return Ok((None, Vec::new()));
    }
    let (terms, weights, samples) = if mono {
        serialize_mono_passes(&passes)?
    } else {
        serialize_stereo_passes(&passes)?
    };
    let assembled = if mono {
        assemble_mono_passes(&terms, &weights, &samples)?
    } else {
        assemble_stereo_passes(&terms, &weights, &samples)?
    };
    Ok((Some((terms, weights, samples)), assembled))
}

/// The entropy-level output of one hybrid block encode.
struct HybridStreams {
    /// The `0x0A` main payload (even-padded).
    main: Vec<u8>,
    /// The `0x0B` correction payload (even-padded), when requested.
    wvc: Option<Vec<u8>>,
    /// The decoder's coarse (lossy) output samples, coded-domain.
    coarse_out: Vec<i32>,
    /// Largest sign-folded residual magnitude bit-length seen (both
    /// exact and coarse domains) — feeds the header max-magnitude.
    max_res_bits: u32,
}

/// The §6.5 bracketed entropy encode of a whole **mono** buffer.
///
/// `shaping` is the `0x07`-seeded noise-shaping filter, advanced /
/// updated in exact decoder lockstep
/// ([`crate::decode_packed_samples_mono_hybrid_lossless`]): every
/// sample advances the weight, bracketed words target the **shaped**
/// residual `exact - temp` (the value the correction stream stores and
/// the pair decode adds `temp` back onto), and zero-run / lossless
/// dispatches apply no temp and fold `exact == coarse` into the error
/// state. A zero (absent-`0x07`) state makes every temp zero — the
/// bitstream is then bit-identical to the unshaped encode.
fn hybrid_entropy_mono(
    coded: &[i32],
    hybrid: &mut HybridState,
    stepper: &mut MonoStepper,
    correction: bool,
    shaping: &mut crate::ShapingState,
) -> Result<HybridStreams> {
    let mut writer = BitWriter::new();
    let mut wvc = correction.then(BitWriter::new);
    let mut medians = AdaptiveMedians::new([0, 0, 0]);
    let mut run = RunState::new();
    let mut run_break = false;
    let mut pending: Option<PendingWord> = None;
    let mut coarse_out = Vec::with_capacity(coded.len());
    let mut max_res_bits = 0u32;
    let n = coded.len();
    let mut i = 0usize;
    while i < n {
        // Decoder per-sample order: limit from the pre-sample state,
        // weight advance, then the word dispatch.
        let limit = hybrid.frame_limits()[0];
        let temp = shaping.advance(0);
        let exact = coded[i].wrapping_sub(stepper.offset());
        // The wire word's value: bracketed words target the shaped
        // residual; the lossless dispatch writes the exact residual.
        let wire = if limit == 0 {
            exact
        } else {
            exact.wrapping_sub(temp)
        };
        let (mag, _) = split_sign(wire);
        max_res_bits = max_res_bits.max(32 - mag.leading_zeros());
        if let Some(p) = pending.take() {
            emit_pending(&mut writer, &mut run, p, Some((mag, medians.get_med(0))))?;
        }
        if run_break {
            run_break = false;
        } else if medians.values[0] <= 1 && !run.last_one && !run.last_zero {
            // §4.2 step 1: the maximal run of zero exact residuals
            // (run members decode exact == coarse == 0, so membership
            // keys on the exact residual even under shaping).
            let mut run_len = 0u32;
            let mut exact_i = exact;
            let mut advanced = true; // the entry sample already advanced
            while i < n && exact_i == 0 {
                if !advanced {
                    let _ = shaping.advance(0);
                }
                advanced = false;
                shaping.update(0, 0, 0, 0);
                hybrid.update_signed(0, 0);
                coarse_out.push(stepper.advance(0));
                run_len += 1;
                i += 1;
                if i < n {
                    exact_i = coded[i].wrapping_sub(stepper.offset());
                }
            }
            emit_zero_run_length(&mut writer, run_len);
            if run_len > 0 {
                medians.values = [0, 0, 0];
                run_break = true;
                continue;
            }
        }
        let zone = medians.zone_for_magnitude(mag);
        let interval = medians.sample_interval_for_ones_count(zone);
        medians.adapt(Zone::from_ones_count(zone));
        let (word, coarse_res) = plan_word(&interval, zone, wire, limit, wvc.as_mut())?;
        pending = Some(word);
        let (cmag, _) = split_sign(coarse_res);
        max_res_bits = max_res_bits.max(32 - cmag.leading_zeros());
        hybrid.update_signed(0, coarse_res);
        if limit == 0 {
            shaping.update(0, exact, exact, 0);
        } else {
            shaping.update(0, exact, coarse_res, temp);
        }
        coarse_out.push(stepper.advance(coarse_res));
        i += 1;
    }
    if let Some(p) = pending.take() {
        emit_pending(&mut writer, &mut run, p, None)?;
    }
    Ok(HybridStreams {
        main: finish_even(writer),
        wvc: wvc.map(finish_even),
        coarse_out,
        max_res_bits,
    })
}

/// How the noise-shaping filter maps onto an interleaved stereo encode
/// (round 420, mirroring the two decode-side placements).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StereoShapingMode {
    /// Left/right-coded blocks: the shaping channels coincide with the
    /// coded channels and the temps fold per slot, exactly as in the
    /// mono loop ([`crate::decode_packed_samples_stereo_hybrid_lossless`]).
    PerChannel,
    /// Joint (mid/side) blocks: the shaping channels are the **output**
    /// (left/right) channels; both weights advance once per frame and
    /// the coded-domain temps are the round-415 transform
    /// (`t_m = t_l - t_r`;
    /// `t_s = t_r + ((mid_out + t_m) >> 1) - (mid_out >> 1)`), with the
    /// error states folded against the post-undo left/right values —
    /// see `WavPackBlock::decode_joint_samples_with_correction`.
    JointOutput,
}

/// Per-frame bookkeeping of the joint-output shaping transform: what
/// the even (mid) slot decided, consumed by the odd (side) slot.
#[derive(Debug, Clone, Copy, Default)]
struct JointFrame {
    /// The frame's left/right weight advances (`t_l`, `t_r`).
    t_l: i32,
    t_r: i32,
    /// The exact coded-domain mid target (`coded[2f]`).
    m_exact: i32,
    /// The applied mid temp: `t_l - t_r` when the mid slot was
    /// bracketed, `0` otherwise.
    t_m_applied: i32,
}

/// The §6.5 bracketed entropy encode of a whole interleaved **stereo**
/// buffer (frame-start limit snapshots, stream-level zero runs,
/// shared holding state).
///
/// `shaping` + `mode` thread the `0x07` noise-shaping filter through
/// the encode in exact decoder lockstep; a zero (absent-`0x07`) state
/// leaves the bitstream bit-identical to the unshaped encode in either
/// mode.
fn hybrid_entropy_stereo(
    coded: &[i32],
    hybrid: &mut HybridState,
    stepper: &mut StereoStepper,
    correction: bool,
    shaping: &mut crate::ShapingState,
    mode: StereoShapingMode,
) -> Result<HybridStreams> {
    let mut writer = BitWriter::new();
    let mut wvc = correction.then(BitWriter::new);
    let mut medians = [
        AdaptiveMedians::new([0, 0, 0]),
        AdaptiveMedians::new([0, 0, 0]),
    ];
    let mut run = RunState::new();
    let mut run_break = false;
    let mut pending: Option<PendingWord> = None;
    let mut coarse_out = Vec::with_capacity(coded.len());
    let mut max_res_bits = 0u32;
    let slots = coded.len();
    let mut limits = [0u32; 2];
    // The current frame's channel-A coarse residual (set at every even
    // slot, consumed by the odd slot's offset and the frame advance).
    let mut frame_res_a = 0i32;
    let mut frame = JointFrame::default();
    // Completes one frame at its odd slot: stepper advance, joint
    // error-state fold (the round-415 effective per-output deltas
    // against the post-undo left/right values), coarse output push.
    let complete_frame = |shaping: &mut crate::ShapingState,
                          stepper: &mut StereoStepper,
                          coarse_out: &mut Vec<i32>,
                          frame: &JointFrame,
                          res_a: i32,
                          res_b: i32,
                          s_exact: i32,
                          t_s_applied: i32,
                          half_step: i32| {
        let (a, b) = stepper.advance(res_a, res_b);
        if mode == StereoShapingMode::JointOutput {
            let d_r = t_s_applied.wrapping_sub(half_step);
            let d_l = frame.t_m_applied.wrapping_add(d_r);
            let (left, right) = crate::crc::undo_joint_stereo(frame.m_exact, s_exact);
            let (left_lossy, right_lossy) = crate::crc::undo_joint_stereo(a, b);
            shaping.update(0, left, left_lossy, d_l);
            shaping.update(1, right, right_lossy, d_r);
        }
        coarse_out.push(a);
        coarse_out.push(b);
    };
    // The joint half-step of the current frame's side slot:
    // `(mid_shaped >> 1) - (mid >> 1)` with `mid_shaped == m_exact`
    // and `mid == m_exact - t_m_applied` (both known exactly on the
    // encode side because the mid slot was constructed to reach its
    // target).
    let joint_half_step = |frame: &JointFrame| {
        (frame.m_exact >> 1).wrapping_sub(frame.m_exact.wrapping_sub(frame.t_m_applied) >> 1)
    };
    let mut i = 0usize;
    while i < slots {
        let ch = i & 1;
        if ch == 0 {
            limits = hybrid.frame_limits();
            if mode == StereoShapingMode::JointOutput {
                frame.t_l = shaping.advance(0);
                frame.t_r = shaping.advance(1);
            }
        }
        let temp = match mode {
            StereoShapingMode::PerChannel => shaping.advance(ch),
            // The candidate coded-domain temp of this slot, applied
            // only when the slot ends up bracketed.
            StereoShapingMode::JointOutput if ch == 0 => frame.t_l.wrapping_sub(frame.t_r),
            StereoShapingMode::JointOutput => frame.t_r.wrapping_add(joint_half_step(&frame)),
        };
        let exact = if ch == 0 {
            coded[i].wrapping_sub(stepper.offset_a())
        } else {
            coded[i].wrapping_sub(stepper.offset_b(frame_res_a))
        };
        if mode == StereoShapingMode::JointOutput && ch == 0 {
            frame.m_exact = coded[i];
        }
        let wire = if limits[ch] == 0 {
            exact
        } else {
            exact.wrapping_sub(temp)
        };
        let (mag, _) = split_sign(wire);
        max_res_bits = max_res_bits.max(32 - mag.leading_zeros());
        if let Some(p) = pending.take() {
            emit_pending(
                &mut writer,
                &mut run,
                p,
                Some((mag, medians[ch].get_med(0))),
            )?;
        }
        if run_break {
            run_break = false;
        } else if medians[0].values[0] <= 1
            && medians[1].values[0] <= 1
            && !run.last_one
            && !run.last_zero
        {
            // Stream-level zero run across both channels (membership
            // keys on the exact residual: run members decode
            // exact == coarse == 0).
            let mut run_len = 0u32;
            let mut exact_i = exact;
            let mut advanced = true; // the entry slot already advanced
            while i < slots && exact_i == 0 {
                let chx = i & 1;
                match mode {
                    StereoShapingMode::PerChannel => {
                        if !advanced {
                            let _ = shaping.advance(chx);
                        }
                        shaping.update(chx, 0, 0, 0);
                    }
                    StereoShapingMode::JointOutput => {
                        if chx == 0 {
                            if !advanced {
                                frame.t_l = shaping.advance(0);
                                frame.t_r = shaping.advance(1);
                            }
                            frame.m_exact = coded[i];
                            frame.t_m_applied = 0;
                        }
                    }
                }
                advanced = false;
                hybrid.update_signed(chx, 0);
                if chx == 0 {
                    frame_res_a = 0;
                } else {
                    complete_frame(
                        shaping,
                        stepper,
                        &mut coarse_out,
                        &frame,
                        frame_res_a,
                        0,
                        coded[i],
                        0,
                        joint_half_step(&frame),
                    );
                }
                run_len += 1;
                i += 1;
                if i < slots {
                    if i & 1 == 0 {
                        limits = hybrid.frame_limits();
                        exact_i = coded[i].wrapping_sub(stepper.offset_a());
                    } else {
                        exact_i = coded[i].wrapping_sub(stepper.offset_b(frame_res_a));
                    }
                }
            }
            emit_zero_run_length(&mut writer, run_len);
            if run_len > 0 {
                medians[0].values = [0, 0, 0];
                medians[1].values = [0, 0, 0];
                run_break = true;
                continue;
            }
        }
        let zone = medians[ch].zone_for_magnitude(mag);
        let interval = medians[ch].sample_interval_for_ones_count(zone);
        medians[ch].adapt(Zone::from_ones_count(zone));
        let (word, coarse_res) = plan_word(&interval, zone, wire, limits[ch], wvc.as_mut())?;
        pending = Some(word);
        let bracketed = limits[ch] != 0;
        let (cmag, _) = split_sign(coarse_res);
        max_res_bits = max_res_bits.max(32 - cmag.leading_zeros());
        hybrid.update_signed(ch, coarse_res);
        match mode {
            StereoShapingMode::PerChannel => {
                if bracketed {
                    shaping.update(ch, exact, coarse_res, temp);
                } else {
                    shaping.update(ch, exact, exact, 0);
                }
            }
            StereoShapingMode::JointOutput => {
                if ch == 0 {
                    frame.t_m_applied = if bracketed { temp } else { 0 };
                }
            }
        }
        if ch == 0 {
            frame_res_a = coarse_res;
        } else {
            let (t_s_applied, half_step) = if mode == StereoShapingMode::JointOutput {
                (if bracketed { temp } else { 0 }, joint_half_step(&frame))
            } else {
                (0, 0)
            };
            complete_frame(
                shaping,
                stepper,
                &mut coarse_out,
                &frame,
                frame_res_a,
                coarse_res,
                coded[i],
                t_s_applied,
                half_step,
            );
        }
        i += 1;
    }
    if let Some(p) = pending.take() {
        emit_pending(&mut writer, &mut run, p, None)?;
    }
    Ok(HybridStreams {
        main: finish_even(writer),
        wvc: wvc.map(finish_even),
        coarse_out,
        max_res_bits,
    })
}

/// One encoded hybrid block: the `.wv` bytes, the optional `.wvc`
/// twin, the final per-channel `slow_level` state, and the final
/// noise-shaping filter state (both carried across blocks by the
/// stream encoders).
pub(crate) type HybridBlock = (Vec<u8>, Option<Vec<u8>>, [u32; 2], crate::ShapingState);

/// One hybrid block's `.wv` (+ optional `.wvc`) assembly from the
/// pre-format integer buffer.
///
/// `shaping_payload` is the block's exact on-wire `0x07` payload
/// (`None` = shaping off): it seeds this block's encoder-side filter
/// state via the same expansion the decoder uses, is emitted verbatim
/// into the `.wvc` twin, and turns on the `HYBRID_SHAPE` /
/// `NEW_SHAPING` header bits (both set on every reference shaped
/// block — round-408/415 fixture flag observation).
#[allow(clippy::too_many_arguments)]
pub(crate) fn encode_hybrid_block_ints(
    pcm: &[i32],
    mono: bool,
    bytes_per_sample: u8,
    opts: &HybridOptions,
    level_words: [i16; 2],
    shaping_payload: Option<&[u8]>,
    format: Option<&FormatExtras>,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridBlock> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if !mono && pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let stereo = !mono;
    let joint = stereo && opts.joint;
    // The on-wire bitrate word is a 16-bit field; clamp once and use
    // the same value for both the emitted payload and the encoder-side
    // state so the two can never diverge.
    let bitrate_word = opts.bitrate_word.clamp(0, i32::from(i16::MAX));

    // Coded domain: joint-transform stereo pairs ahead of prediction.
    let mut coded = pcm.to_vec();
    if joint {
        for pair in coded.chunks_exact_mut(2) {
            let (mid, side) = forward_joint_stereo(pair[0], pair[1]);
            pair[0] = mid;
            pair[1] = side;
        }
    }

    let (decorr_payloads, passes) = derive_hybrid_passes(&coded, mono, opts.profile)?;

    // §6.5 running state, seeded exactly as the decoder will seed it.
    let profile = HybridProfile {
        level_words,
        bitrate: bitrate_word,
        balance: if joint { JOINT_BALANCE_WORD } else { 0 },
        stereo,
    };
    let mut hybrid = HybridState::from_profile(&profile);
    // The 0x07-seeded shaping filter, in exact decoder lockstep (an
    // absent payload seeds the all-zero no-op state).
    let mut shaping = crate::ShapingState::from_shaping_words(shaping_payload, stereo);

    let streams = if mono {
        let mut stepper = MonoStepper::new(passes)?;
        hybrid_entropy_mono(
            &coded,
            &mut hybrid,
            &mut stepper,
            opts.correction,
            &mut shaping,
        )?
    } else {
        let mut stepper = StereoStepper::new(passes)?;
        hybrid_entropy_stereo(
            &coded,
            &mut hybrid,
            &mut stepper,
            opts.correction,
            &mut shaping,
            if joint {
                StereoShapingMode::JointOutput
            } else {
                StereoShapingMode::PerChannel
            },
        )?
    };

    // The lossy decode output the .wv header CRC covers (post joint
    // undo for joint blocks). The round-418 output clamp does NOT
    // participate here: the reference folds the §5 CRC over the
    // UNCLAMPED reconstruction and saturates afterwards (pinned by
    // the clamp battery — a clamped-CRC stream is reported as a CRC
    // error by the reference decoder).
    let mut lossy_out = streams.coarse_out;
    if joint {
        for pair in lossy_out.chunks_exact_mut(2) {
            let (l, r) = crate::crc::undo_joint_stereo(pair[0], pair[1]);
            pair[0] = l;
            pair[1] = r;
        }
    }
    let lossy_crc = if mono {
        crate::crc::crc_mono(&lossy_out)
    } else {
        crate::crc::crc_stereo_interleaved(&lossy_out)
    };
    // The lossless decode output the .wvc header CRC covers — the
    // original pre-fixup integers (round-415 pin).
    let lossless_crc = if mono {
        crate::crc::crc_mono(pcm)
    } else {
        crate::crc::crc_stereo_interleaved(pcm)
    };

    // Header flag word.
    let mut flags_raw = with_marker(base_flags(bytes_per_sample), 0b11);
    flags_raw |= HYBRID_FLAG | HYBRID_PROFILE_FLAG_9;
    if shaping_payload.is_some() {
        // Both shape bits are set on every reference shaped block —
        // static and dynamic weights alike (fixture flag observation).
        flags_raw |= crate::hybrid::HYBRID_SHAPE_FLAG | crate::hybrid::NEW_SHAPING_FLAG;
    }
    if mono {
        flags_raw |= 1 << 2;
    } else {
        flags_raw |= HYBRID_PROFILE_FLAG_10;
    }
    if joint {
        flags_raw |= crate::crc::JOINT_STEREO_FLAG;
    }
    if let Some(format) = format {
        flags_raw |= format.flag_bit;
    }
    let max_magnitude = magnitude_bits(pcm)
        .max(magnitude_bits(&lossy_out))
        .max(streams.max_res_bits);
    flags_raw |= max_magnitude.min(0x1f) << 18;

    // .wv metadata chain: decorrelation triple, entropy info, hybrid
    // profile, sample-format profile, main bitstream (the reference
    // sub-block order).
    let mut metadata = Vec::new();
    if let Some((terms, weights, samples)) = &decorr_payloads {
        append_sub_block(
            &mut metadata,
            SubBlockId::DecorrelationTerms.as_id_byte(),
            terms,
        )?;
        append_sub_block(
            &mut metadata,
            SubBlockId::DecorrelationWeights.as_id_byte(),
            weights,
        )?;
        append_sub_block(
            &mut metadata,
            SubBlockId::DecorrelationSamples.as_id_byte(),
            samples,
        )?;
    }
    let entropy_payload = if mono {
        pack_entropy_info(&[[0, 0, 0]])
    } else {
        pack_entropy_info(&[[0, 0, 0], [0, 0, 0]])
    };
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;
    let mut profile_payload = Vec::with_capacity(8);
    profile_payload.extend_from_slice(&level_words[0].to_le_bytes());
    if stereo {
        profile_payload.extend_from_slice(&level_words[1].to_le_bytes());
    }
    profile_payload.extend_from_slice(&(bitrate_word as i16).to_le_bytes());
    if stereo {
        profile_payload.extend_from_slice(&(profile.balance as i16).to_le_bytes());
    }
    append_sub_block(
        &mut metadata,
        SubBlockId::HybridProfile.as_id_byte(),
        &profile_payload,
    )?;
    if let Some(format) = format {
        append_sub_block(&mut metadata, format.profile_id, &format.profile_payload)?;
    }
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &streams.main,
    )?;

    let per_channel = if mono { pcm.len() } else { pcm.len() / 2 };
    let block_samples =
        u32::try_from(per_channel).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let wv = build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        lossy_crc,
    )?;

    // The .wvc twin: same header fields, lossless CRC, 0x07 + 0x0B
    // (+ 0x0C).
    let wvc = match streams.wvc {
        Some(correction_payload) => {
            let mut cmeta = Vec::new();
            // The 0x07 shaping seed leads the correction chain (the
            // wiki routes it to the wvc file; reference pair encodes
            // place it ahead of 0x0B — round-415/420 observation).
            if let Some(sp) = shaping_payload {
                append_sub_block(&mut cmeta, SubBlockId::NoiseShapingProfile.as_id_byte(), sp)?;
            }
            // An all-run block spends no bracket bits; the reference
            // decoder rejects a zero-length 0x0B sub-block outright
            // (round-418 black-box pin), so an empty correction
            // payload is simply omitted — the pair decode reads the
            // missing sub-block as an empty correction stream.
            if !correction_payload.is_empty() {
                append_sub_block(
                    &mut cmeta,
                    SubBlockId::PackedCorrectionData.as_id_byte(),
                    &correction_payload,
                )?;
            }
            if let Some(ext) = format.and_then(|f| f.extension.as_deref()) {
                append_sub_block(&mut cmeta, SubBlockId::PackedOverflowBits.as_id_byte(), ext)?;
            }
            Some(build_block(
                cmeta,
                block_index,
                total_samples,
                block_samples,
                flags_raw,
                lossless_crc,
            )?)
        }
        None => None,
    };

    Ok((
        wv,
        wvc,
        [hybrid.slow_level(0), hybrid.slow_level(1)],
        shaping,
    ))
}

/// Seed level words for the first block of a stream: a §6.5 pre-pass
/// over a lossless-residual estimate of (up to) the opening samples.
fn seed_level_words(coded: &[i32], mono: bool, passes: Vec<DecorrPass>) -> Result<[i16; 2]> {
    let window = coded.len().min(4096);
    let mut estimate = coded[..window].to_vec();
    let mut passes = passes;
    if !passes.is_empty() {
        if mono {
            recorrelate_mono(&mut passes, &mut estimate)?;
        } else {
            recorrelate_stereo(&mut passes, &mut estimate)?;
        }
    }
    let sl = level_prepass(&estimate, if mono { 1 } else { 2 });
    Ok([pack_level_word(sl[0]), pack_level_word(sl[1])])
}

/// Compute the coded-domain buffer and the first-block level-word seed
/// for a hybrid encode.
fn hybrid_seed(pcm: &[i32], mono: bool, opts: &HybridOptions) -> Result<[i16; 2]> {
    let joint = !mono && opts.joint;
    let mut coded = pcm.to_vec();
    if joint {
        for pair in coded.chunks_exact_mut(2) {
            let (mid, side) = forward_joint_stereo(pair[0], pair[1]);
            pair[0] = mid;
            pair[1] = side;
        }
    }
    let (_, passes) = derive_hybrid_passes(&coded, mono, opts.profile)?;
    seed_level_words(&coded, mono, passes)
}

/// The full-scale weight accumulator rail: `±1024 << 16` (weight
/// `±1.0`). Emitted ramps saturate here — the black-box-validated
/// envelope (see [`HybridShaping::Ramp`]).
const SHAPING_ACC_RAIL: i64 = 1024 << 16;

/// Cross-block hybrid encoder state: the packed `0x06` level words and
/// the `0x07` shaping filter state for the **next** block — always
/// derived from the on-wire (log-quantized) forms, so the encoder-side
/// state can never drift from what the decoder reconstructs. Used by
/// the stream encoders and the registry [`crate::registry`] encoder
/// (which spans blocks across packets).
#[derive(Debug, Clone)]
pub(crate) struct HybridCarry {
    /// The next block's `0x06` level words; `None` until the first
    /// block seeds them from the §6.5 pre-pass.
    level: Option<[i16; 2]>,
    /// The running shaping filter state (`None` = shaping off). The
    /// per-block `0x07` payload is packed from it on demand.
    shaping: Option<crate::ShapingState>,
    /// The caller-requested per-sample accumulator delta (`0` for
    /// static shaping; per-block emissions rail-saturate it).
    requested_delta: i64,
    /// Whether the `0x07` layout carries delta words.
    with_delta: bool,
    /// Whether the shaping payloads use the stereo word interleave.
    stereo: bool,
}

impl HybridCarry {
    /// Fresh carry for a stream encode under `opts` with the given
    /// channel shape.
    pub(crate) fn new(opts: &HybridOptions, mono: bool) -> Self {
        let stereo = !mono;
        let requested_delta = match opts.shaping {
            HybridShaping::Ramp { delta, .. } => i64::from(delta),
            _ => 0,
        };
        HybridCarry {
            level: None,
            shaping: opts
                .shaping
                .initial_payload(stereo)
                .map(|p| crate::ShapingState::from_shaping_words(Some(&p), stereo)),
            requested_delta,
            with_delta: opts.shaping.with_delta(),
            stereo,
        }
    }

    /// The level words for the next block, seeding from `window` (the
    /// pre-format integer buffer) when this is the first block.
    pub(crate) fn level_words(
        &self,
        window: &[i32],
        mono: bool,
        opts: &HybridOptions,
    ) -> Result<[i16; 2]> {
        match self.level {
            Some(words) => Ok(words),
            None => hybrid_seed(window, mono, opts),
        }
    }

    /// The largest log-word-representable per-sample delta of the
    /// requested sign that keeps `acc` inside the full-scale rail for
    /// `frames` advances (`0` once the rail is reached).
    fn rail_saturated_delta(&self, acc: i64, frames: usize) -> i32 {
        let frames = frames.max(1) as i64;
        let room = if self.requested_delta >= 0 {
            (SHAPING_ACC_RAIL - acc).max(0)
        } else {
            (-SHAPING_ACC_RAIL - acc).min(0)
        };
        let mut delta = self
            .requested_delta
            .clamp(-(room.abs() / frames), room.abs() / frames) as i32;
        // The log pack rounds within its table precision; step the
        // quantized value toward zero until the whole-block excursion
        // provably stays inside the rail.
        loop {
            let q = crate::logpack::quantize_log_value(delta);
            let end = acc + i64::from(q) * frames;
            if end.abs() <= SHAPING_ACC_RAIL || q == 0 {
                return q;
            }
            // Shrink ~1% per step (at least 1); terminates at 0.
            delta = if delta > 0 {
                (delta - 1 - delta / 128).max(0)
            } else {
                (delta + 1 - delta / 128).min(0)
            };
        }
    }

    /// The next block's exact on-wire `0x07` payload for a block of
    /// `frames` per-channel samples (`None` = shaping off): the
    /// running error/acc state log-packed, plus — for ramps — the
    /// rail-saturated per-channel delta words.
    ///
    /// The re-packed accumulator words are themselves clamped to the
    /// rail: the log pack rounds within its table precision, so an
    /// end-of-block state sitting *at* the rail could otherwise
    /// re-quantize a hair past it and seed the next block outside the
    /// validated envelope.
    pub(crate) fn payload_for_block(&self, frames: usize) -> Option<Vec<u8>> {
        let state = self.shaping.as_ref()?;
        let mut payload = state.to_shaping_words(self.stereo, false);
        let channels = if self.stereo { 2 } else { 1 };
        let mut accs = [0i64; 2];
        for (ch, acc) in accs.iter_mut().enumerate().take(channels) {
            // Acc word position in the layout: mono [e, a]; stereo
            // [e0, a0, e1, a1].
            let idx = 2 * (2 * ch + 1);
            let mut word = i16::from_le_bytes([payload[idx], payload[idx + 1]]);
            loop {
                let val = i64::from(crate::logpack::wp_exp2s(i32::from(word)));
                if val.abs() <= SHAPING_ACC_RAIL || word == 0 {
                    *acc = val;
                    break;
                }
                word -= word.signum();
            }
            payload[idx..idx + 2].copy_from_slice(&word.to_le_bytes());
        }
        if self.with_delta {
            for &acc in accs.iter().take(channels) {
                let delta = self.rail_saturated_delta(acc, frames);
                payload.extend_from_slice(&crate::logpack::pack_log_word(delta));
            }
        }
        Some(payload)
    }

    /// Fold one encoded block's end state back into the carry.
    pub(crate) fn absorb(&mut self, slow_level: [u32; 2], shaping: &crate::ShapingState) {
        self.level = Some([
            pack_level_word(slow_level[0]),
            pack_level_word(slow_level[1]),
        ]);
        if self.shaping.is_some() {
            self.shaping = Some(*shaping);
        }
    }
}

/// Encode a mono PCM buffer into one hybrid `wvpk` block: a lossy
/// `.wv` at the [`HybridOptions::bitrate_word`] noise target, plus —
/// when [`HybridOptions::correction`] — the `.wvc` twin that restores
/// the input bit-exactly through the pair decode
/// (`decode_stream_with_correction(&wv, &wvc)? == pcm`; the `.wv`
/// alone decodes to the coarse PCM this encoder derived).
pub fn encode_block_mono_hybrid(
    pcm: &[i32],
    bytes_per_sample: u8,
    opts: &HybridOptions,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridEncoded> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    let carry = HybridCarry::new(opts, true);
    let level = carry.level_words(pcm, true, opts)?;
    let shaping = carry.payload_for_block(pcm.len());
    let (wv, wvc, _, _) = encode_hybrid_block_ints(
        pcm,
        true,
        bytes_per_sample,
        opts,
        level,
        shaping.as_deref(),
        None,
        block_index,
        total_samples,
    )?;
    Ok(HybridEncoded { wv, wvc })
}

/// Encode an interleaved stereo PCM buffer into one hybrid `wvpk`
/// block — the stereo twin of [`encode_block_mono_hybrid`]
/// (left/right or joint coding per [`HybridOptions::joint`]).
pub fn encode_block_stereo_hybrid(
    pcm: &[i32],
    bytes_per_sample: u8,
    opts: &HybridOptions,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridEncoded> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let carry = HybridCarry::new(opts, false);
    let level = carry.level_words(pcm, false, opts)?;
    let shaping = carry.payload_for_block(pcm.len() / 2);
    let (wv, wvc, _, _) = encode_hybrid_block_ints(
        pcm,
        false,
        bytes_per_sample,
        opts,
        level,
        shaping.as_deref(),
        None,
        block_index,
        total_samples,
    )?;
    Ok(HybridEncoded { wv, wvc })
}

/// Encode a mono PCM buffer into a multi-block hybrid `.wv` (+
/// `.wvc`) chain: per-chunk blocks with the running `slow_level`
/// carried across block boundaries (each block's `0x06` level words
/// are the packed end state of the previous block, exactly the state
/// the decoder reconstructs), the first block seeded by a §6.5
/// pre-pass. Chunking / header contract matches
/// [`crate::encode_stream_mono`].
pub fn encode_stream_mono_hybrid(
    pcm: &[i32],
    block_samples: usize,
    bytes_per_sample: u8,
    opts: &HybridOptions,
) -> Result<HybridEncoded> {
    let chunk = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut wv = Vec::new();
    let mut wvc_all = Vec::new();
    let mut index: u32 = 0;
    let mut carry = HybridCarry::new(opts, true);
    for window in pcm.chunks(chunk) {
        let level_words = carry.level_words(window, true, opts)?;
        let shaping = carry.payload_for_block(window.len());
        let (blk, cblk, sl, shape) = encode_hybrid_block_ints(
            window,
            true,
            bytes_per_sample,
            opts,
            level_words,
            shaping.as_deref(),
            None,
            index,
            total,
        )?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        carry.absorb(sl, &shape);
        index = index
            .checked_add(window.len() as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(HybridEncoded {
        wv,
        wvc: opts.correction.then_some(wvc_all),
    })
}

/// Encode an interleaved stereo PCM buffer into a multi-block hybrid
/// `.wv` (+ `.wvc`) chain — the stereo twin of
/// [`encode_stream_mono_hybrid`] (`block_samples` is a per-channel
/// pair count).
pub fn encode_stream_stereo_hybrid(
    pcm: &[i32],
    block_samples: usize,
    bytes_per_sample: u8,
    opts: &HybridOptions,
) -> Result<HybridEncoded> {
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let pairs = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut wv = Vec::new();
    let mut wvc_all = Vec::new();
    let mut index: u32 = 0;
    let mut carry = HybridCarry::new(opts, false);
    for window in pcm.chunks(pairs * 2) {
        let level_words = carry.level_words(window, false, opts)?;
        let shaping = carry.payload_for_block(window.len() / 2);
        let (blk, cblk, sl, shape) = encode_hybrid_block_ints(
            window,
            false,
            bytes_per_sample,
            opts,
            level_words,
            shaping.as_deref(),
            None,
            index,
            total,
        )?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        carry.absorb(sl, &shape);
        index = index
            .checked_add((window.len() / 2) as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(HybridEncoded {
        wv,
        wvc: opts.correction.then_some(wvc_all),
    })
}

// ---------------------------------------------------------------------
// Float / int32 hybrid origination (round 418): the sample-format
// deconstruction feeds the hybrid integer pipeline; the 0x0C extension
// payload rides the .wvc twin (round-415 structural pin), so the lossy
// .wv decodes with the implied-zero fill and the pair decode restores
// the exact input.
// ---------------------------------------------------------------------

/// Shared body of the float-hybrid block encoders. The float shape
/// uses the **raised** exponent anchor (`deconstruct_float_raised`) so
/// the coarse magnitudes keep head-room under the 24-bit mantissa
/// window; the emitted `.wv` is then verified through this crate's own
/// lossy decode, surfacing the (pathological-bitrate) corner where a
/// coarse value still overflows the window as the decoder's typed
/// error instead of shipping an undecodable stream.
pub(crate) fn encode_hybrid_block_float(
    pcm: &[f32],
    mono: bool,
    opts: &HybridOptions,
    carry: &mut HybridCarry,
    block_index: u32,
    total_samples: u32,
) -> Result<(Vec<u8>, Option<Vec<u8>>)> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if !mono && pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let d = crate::float::deconstruct_float_raised(pcm);
    let format = crate::encode::float_format_extras(&d);
    let level = carry.level_words(&d.integers, mono, opts)?;
    let shaping = carry.payload_for_block(pcm.len() / if mono { 1 } else { 2 });
    let (wv, wvc, sl, shape) = encode_hybrid_block_ints(
        &d.integers,
        mono,
        4,
        opts,
        level,
        shaping.as_deref(),
        Some(&format),
        block_index,
        total_samples,
    )?;
    // Verify the lossy stream reconstructs (the implied-zero float
    // fixup can refuse a coarse magnitude past the mantissa window).
    crate::block::decode_stream(&wv)?;
    carry.absorb(sl, &shape);
    Ok((wv, wvc))
}

/// Shared body of the int32-hybrid block encoders.
pub(crate) fn encode_hybrid_block_int32(
    pcm: &[i32],
    mono: bool,
    opts: &HybridOptions,
    carry: &mut HybridCarry,
    block_index: u32,
    total_samples: u32,
) -> Result<(Vec<u8>, Option<Vec<u8>>)> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if !mono && pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let d = crate::int32::deconstruct_int32(pcm);
    let format = crate::encode::int32_format_extras(&d);
    let level = carry.level_words(&d.reduced, mono, opts)?;
    let shaping = carry.payload_for_block(pcm.len() / if mono { 1 } else { 2 });
    let (wv, wvc, sl, shape) = encode_hybrid_block_ints(
        &d.reduced,
        mono,
        4,
        opts,
        level,
        shaping.as_deref(),
        Some(&format),
        block_index,
        total_samples,
    )?;
    carry.absorb(sl, &shape);
    Ok((wv, wvc))
}

/// Encode a mono `f32` buffer into one hybrid `FLOAT_DATA` block: a
/// lossy `.wv` (implied-zero mantissa fill on a `.wv`-only decode)
/// plus — when [`HybridOptions::correction`] — the `.wvc` twin whose
/// `0x0B` correction and `0x0C` extension streams restore the input
/// bit patterns exactly
/// (`decode_stream_with_correction_f32(&wv, &wvc)?` == input).
pub fn encode_block_mono_hybrid_float(
    pcm: &[f32],
    opts: &HybridOptions,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridEncoded> {
    let mut carry = HybridCarry::new(opts, true);
    let (wv, wvc) =
        encode_hybrid_block_float(pcm, true, opts, &mut carry, block_index, total_samples)?;
    Ok(HybridEncoded { wv, wvc })
}

/// Encode an interleaved stereo `f32` buffer into one hybrid
/// `FLOAT_DATA` block — the stereo twin of
/// [`encode_block_mono_hybrid_float`].
pub fn encode_block_stereo_hybrid_float(
    pcm: &[f32],
    opts: &HybridOptions,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridEncoded> {
    let mut carry = HybridCarry::new(opts, false);
    let (wv, wvc) =
        encode_hybrid_block_float(pcm, false, opts, &mut carry, block_index, total_samples)?;
    Ok(HybridEncoded { wv, wvc })
}

/// Encode a mono wide-integer buffer into one hybrid `INT32_DATA`
/// block: the `0x09` reduction (redundancy + `sent_bits`) feeds the
/// hybrid pipeline, the extension bits ride the `.wvc` twin, and the
/// lossy `.wv` decodes with the implied-zero `sent_bits` fill
/// (round-415 pins). `decode_stream_with_correction(&wv, &wvc)? ==
/// pcm` exactly, full `i32` range.
pub fn encode_block_mono_hybrid_int32(
    pcm: &[i32],
    opts: &HybridOptions,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridEncoded> {
    let mut carry = HybridCarry::new(opts, true);
    let (wv, wvc) =
        encode_hybrid_block_int32(pcm, true, opts, &mut carry, block_index, total_samples)?;
    Ok(HybridEncoded { wv, wvc })
}

/// Encode an interleaved stereo wide-integer buffer into one hybrid
/// `INT32_DATA` block — the stereo twin of
/// [`encode_block_mono_hybrid_int32`].
pub fn encode_block_stereo_hybrid_int32(
    pcm: &[i32],
    opts: &HybridOptions,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridEncoded> {
    let mut carry = HybridCarry::new(opts, false);
    let (wv, wvc) =
        encode_hybrid_block_int32(pcm, false, opts, &mut carry, block_index, total_samples)?;
    Ok(HybridEncoded { wv, wvc })
}

/// Multi-block hybrid `FLOAT_DATA` stream (mono) — per-chunk `0x08`
/// profiles, running level state carried across blocks.
pub fn encode_stream_mono_hybrid_float(
    pcm: &[f32],
    block_samples: usize,
    opts: &HybridOptions,
) -> Result<HybridEncoded> {
    let chunk = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut wv = Vec::new();
    let mut wvc_all = Vec::new();
    let mut index: u32 = 0;
    let mut carry = HybridCarry::new(opts, true);
    for window in pcm.chunks(chunk) {
        let (blk, cblk) = encode_hybrid_block_float(window, true, opts, &mut carry, index, total)?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        index = index
            .checked_add(window.len() as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(HybridEncoded {
        wv,
        wvc: opts.correction.then_some(wvc_all),
    })
}

/// Multi-block hybrid `FLOAT_DATA` stream (interleaved stereo).
pub fn encode_stream_stereo_hybrid_float(
    pcm: &[f32],
    block_samples: usize,
    opts: &HybridOptions,
) -> Result<HybridEncoded> {
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let pairs = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut wv = Vec::new();
    let mut wvc_all = Vec::new();
    let mut index: u32 = 0;
    let mut carry = HybridCarry::new(opts, false);
    for window in pcm.chunks(pairs * 2) {
        let (blk, cblk) = encode_hybrid_block_float(window, false, opts, &mut carry, index, total)?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        index = index
            .checked_add((window.len() / 2) as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(HybridEncoded {
        wv,
        wvc: opts.correction.then_some(wvc_all),
    })
}

/// Multi-block hybrid `INT32_DATA` stream (mono).
pub fn encode_stream_mono_hybrid_int32(
    pcm: &[i32],
    block_samples: usize,
    opts: &HybridOptions,
) -> Result<HybridEncoded> {
    let chunk = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut wv = Vec::new();
    let mut wvc_all = Vec::new();
    let mut index: u32 = 0;
    let mut carry = HybridCarry::new(opts, true);
    for window in pcm.chunks(chunk) {
        let (blk, cblk) = encode_hybrid_block_int32(window, true, opts, &mut carry, index, total)?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        index = index
            .checked_add(window.len() as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(HybridEncoded {
        wv,
        wvc: opts.correction.then_some(wvc_all),
    })
}

/// Multi-block hybrid `INT32_DATA` stream (interleaved stereo).
pub fn encode_stream_stereo_hybrid_int32(
    pcm: &[i32],
    block_samples: usize,
    opts: &HybridOptions,
) -> Result<HybridEncoded> {
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let pairs = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut wv = Vec::new();
    let mut wvc_all = Vec::new();
    let mut index: u32 = 0;
    let mut carry = HybridCarry::new(opts, false);
    for window in pcm.chunks(pairs * 2) {
        let (blk, cblk) = encode_hybrid_block_int32(window, false, opts, &mut carry, index, total)?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        index = index
            .checked_add((window.len() / 2) as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(HybridEncoded {
        wv,
        wvc: opts.correction.then_some(wvc_all),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::{
        decode_stream_muted, decode_stream_with_correction, decode_stream_with_correction_muted,
    };

    fn splitmix(seed: u64, n: usize) -> Vec<i64> {
        let mut x = seed;
        (0..n)
            .map(|_| {
                x = x.wrapping_mul(0xd1342543de82ef95).wrapping_add(1);
                x as i64
            })
            .collect()
    }

    /// A music-shaped 16-bit signal with a silence stretch (zero-run
    /// coverage) and both smooth and noisy regions.
    fn signal16(n: usize, seed: u64) -> Vec<i32> {
        splitmix(seed, n)
            .iter()
            .enumerate()
            .map(|(i, &r)| {
                if i > n / 3 && i < n / 3 + 400 {
                    0
                } else {
                    let t = i as f64 * 0.05;
                    let smooth = (t.sin() * 9000.0) as i32;
                    smooth + ((r >> 56) as i32)
                }
            })
            .collect()
    }

    fn opts(bitrate_word: i32, joint: bool, profile: Option<DecorrProfile>) -> HybridOptions {
        HybridOptions {
            bitrate_word,
            correction: true,
            joint,
            profile,
            shaping: HybridShaping::Off,
        }
    }

    fn assert_pair_round_trip(pcm: &[i32], enc: &HybridEncoded) -> Vec<i32> {
        let wvc = enc.wvc.as_ref().expect("correction requested");
        // Pair decode is bit-exactly lossless.
        let exact = decode_stream_with_correction(&enc.wv, wvc).unwrap();
        assert_eq!(exact, pcm, "pair decode must reproduce the input");
        // Pair CRC gate (lossless CRC in the .wvc header) holds.
        let (muted, ok) = decode_stream_with_correction_muted(&enc.wv, wvc).unwrap();
        assert!(ok, "pair CRC gate");
        assert_eq!(muted, pcm);
        // Lossy-only decode passes its own §5.6 gate.
        let (lossy, ok) = decode_stream_muted(&enc.wv).unwrap();
        assert!(ok, "lossy CRC gate");
        lossy
    }

    fn max_err(a: &[i32], b: &[i32]) -> i64 {
        a.iter()
            .zip(b)
            .map(|(&x, &y)| (i64::from(x) - i64::from(y)).abs())
            .max()
            .unwrap()
    }

    #[test]
    fn mono_raw_hybrid_pair_is_lossless_and_lossy_is_close() {
        let pcm = signal16(3000, 0x1234);
        let enc = encode_block_mono_hybrid(&pcm, 2, &opts(456, false, None), 0, 3000).unwrap();
        let lossy = assert_pair_round_trip(&pcm, &enc);
        assert_eq!(lossy.len(), pcm.len());
        let err = max_err(&lossy, &pcm);
        assert!(err > 0, "b4 on 16-bit content is genuinely lossy");
        assert!(err < 4096, "noise stays well under the signal ({err})");
        // The lossy stream is smaller than a lossless encode.
        let lossless = crate::encode::encode_block_mono(&pcm, 2, 0, 3000).unwrap();
        assert!(enc.wv.len() < lossless.len());
    }

    #[test]
    fn mono_decorrelated_hybrid_pair_is_lossless() {
        let pcm = signal16(3000, 0x777);
        let raw = encode_block_mono_hybrid(&pcm, 2, &opts(456, false, None), 0, 3000).unwrap();
        let dec = encode_block_mono_hybrid(
            &pcm,
            2,
            &opts(456, false, Some(DecorrProfile::Normal)),
            0,
            3000,
        )
        .unwrap();
        assert_pair_round_trip(&pcm, &dec);
        // In hybrid mode the lossy stream's size tracks the bitrate
        // target regardless of prediction (the §6.5 limit follows the
        // residual level down), so decorrelation shows up as a smaller
        // *total* pair (the correction stream shrinks with the
        // residuals) on tonal content.
        let total = |e: &HybridEncoded| e.wv.len() + e.wvc.as_ref().unwrap().len();
        assert!(
            total(&dec) < total(&raw),
            "decorr pair {} vs raw pair {}",
            total(&dec),
            total(&raw)
        );
    }

    #[test]
    fn stereo_left_right_hybrid_pair_is_lossless() {
        let mono = signal16(2000, 0xabc);
        let pcm: Vec<i32> = mono.iter().flat_map(|&s| [s, s / 2 - 17]).collect();
        for profile in [None, Some(DecorrProfile::Normal)] {
            let enc =
                encode_block_stereo_hybrid(&pcm, 2, &opts(456, false, profile), 0, 2000).unwrap();
            assert_pair_round_trip(&pcm, &enc);
        }
    }

    #[test]
    fn stereo_joint_hybrid_pair_is_lossless() {
        let mono = signal16(2000, 0x555);
        let pcm: Vec<i32> = mono
            .iter()
            .enumerate()
            .flat_map(|(i, &s)| [s, s + (i as i32 % 23) - 11])
            .collect();
        for profile in [None, Some(DecorrProfile::High)] {
            let enc =
                encode_block_stereo_hybrid(&pcm, 2, &opts(456, true, profile), 0, 2000).unwrap();
            let (parsed, _) = crate::parse_block(&enc.wv).unwrap();
            assert!(parsed.flags().joint_stereo);
            assert!(parsed.flags().hybrid);
            assert_pair_round_trip(&pcm, &enc);
        }
    }

    #[test]
    fn higher_bitrate_words_mean_less_noise_and_more_bytes() {
        let pcm = signal16(2500, 0x9e37);
        let mut last_err = i64::MAX;
        let mut last_len = 0usize;
        for word in [0i32, 200, 456, 800] {
            let enc = encode_block_mono_hybrid(
                &pcm,
                2,
                &opts(word, false, Some(DecorrProfile::Normal)),
                0,
                2500,
            )
            .unwrap();
            let lossy = assert_pair_round_trip(&pcm, &enc);
            let err = max_err(&lossy, &pcm);
            assert!(err > 0, "word {word} is genuinely lossy on this content");
            assert!(err <= last_err, "noise decreases as the word rises");
            assert!(
                enc.wv.len() >= last_len,
                "the lossy stream grows as the word rises ({} vs {last_len})",
                enc.wv.len()
            );
            last_err = err;
            last_len = enc.wv.len();
        }
        // A word past the tracked level's EMA + 256 drives every limit
        // argument non-positive: the degenerate lossless case.
        let enc = encode_block_mono_hybrid(
            &pcm,
            2,
            &opts(8000, false, Some(DecorrProfile::Normal)),
            0,
            2500,
        )
        .unwrap();
        let lossy = assert_pair_round_trip(&pcm, &enc);
        assert_eq!(max_err(&lossy, &pcm), 0, "degenerate lossless words");
    }

    #[test]
    fn silence_stretch_takes_the_zero_run_path_losslessly() {
        // Pure silence block plus a silence-bracketed signal.
        let silent = vec![0i32; 1200];
        let enc = encode_block_mono_hybrid(&silent, 2, &opts(456, false, None), 0, 1200).unwrap();
        let lossy = assert_pair_round_trip(&silent, &enc);
        assert_eq!(
            lossy, silent,
            "silence is lossless even in the lossy stream"
        );

        let mut pcm = vec![0i32; 600];
        pcm.extend(signal16(800, 0x31).iter());
        pcm.extend(std::iter::repeat_n(0i32, 500));
        let n = pcm.len() as u32;
        let enc =
            encode_block_mono_hybrid(&pcm, 2, &opts(456, false, Some(DecorrProfile::Fast)), 0, n)
                .unwrap();
        assert_pair_round_trip(&pcm, &enc);
    }

    #[test]
    fn multi_block_hybrid_streams_carry_the_level_state() {
        let pcm = signal16(5000, 0x40);
        let enc = encode_stream_mono_hybrid(
            &pcm,
            1024,
            2,
            &opts(456, false, Some(DecorrProfile::Normal)),
        )
        .unwrap();
        assert_eq!(crate::audio_block_count(&enc.wv).unwrap(), 5);
        assert_pair_round_trip(&pcm, &enc);
        // Every block's 0x06 level words differ once the signal has
        // trained the running level (block 2+ seeds from block 1's end
        // state, not from the pre-pass).
        let stereo: Vec<i32> = pcm.iter().flat_map(|&s| [s, -s]).collect();
        let enc = encode_stream_stereo_hybrid(
            &stereo,
            900,
            2,
            &opts(456, true, Some(DecorrProfile::Normal)),
        )
        .unwrap();
        assert_pair_round_trip(&stereo, &enc);
    }

    #[test]
    fn lossy_only_mode_omits_the_correction_stream() {
        let pcm = signal16(1500, 0x99);
        let mut o = opts(456, false, Some(DecorrProfile::Normal));
        o.correction = false;
        let enc = encode_block_mono_hybrid(&pcm, 2, &o, 0, 1500).unwrap();
        assert!(enc.wvc.is_none());
        let (_, ok) = decode_stream_muted(&enc.wv).unwrap();
        assert!(ok);
        let (parsed, _) = crate::parse_block(&enc.wv).unwrap();
        assert!(!parsed.has_packed_correction_data());
        assert!(parsed.expects_correction(), "hybrid flag set");
    }

    #[test]
    fn hybrid_options_bitrate_mapping_matches_the_reference_scale() {
        assert_eq!(HybridOptions::from_bits_per_sample(4.0).bitrate_word, 456);
        assert_eq!(HybridOptions::from_bits_per_sample(2.0).bitrate_word, 0);
        assert_eq!(HybridOptions::default().bitrate_word, 456);
    }

    #[test]
    fn hybrid_refuses_empty_and_odd_stereo() {
        let o = HybridOptions::default();
        assert!(matches!(
            encode_block_mono_hybrid(&[], 2, &o, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_hybrid(&[1], 2, &o, 0, 1),
            Err(Error::EncodeStereoOddLength(1))
        ));
    }

    #[test]
    fn wvc_twin_mirrors_the_header_and_carries_the_lossless_crc() {
        let pcm = signal16(1000, 0x11);
        let enc = encode_block_mono_hybrid(&pcm, 2, &opts(456, false, None), 0, 1000).unwrap();
        let (wv_blk, _) = crate::parse_block(&enc.wv).unwrap();
        let (wvc_blk, _) = crate::parse_block(enc.wvc.as_ref().unwrap()).unwrap();
        assert_eq!(wv_blk.flags().raw, wvc_blk.flags().raw);
        assert_eq!(wv_blk.block_index(), wvc_blk.block_index());
        assert_eq!(wv_blk.block_samples(), wvc_blk.block_samples());
        assert_eq!(wvc_blk.crc(), crate::crc::crc_mono(&pcm), "lossless CRC");
        assert_ne!(wv_blk.crc(), wvc_blk.crc(), "lossy vs lossless CRCs differ");
        assert!(wvc_blk.has_packed_correction_data());
    }

    // ---- float / int32 hybrid pairs ----------------------------------

    fn float_signal(n: usize, seed: u64) -> Vec<f32> {
        splitmix(seed, n)
            .iter()
            .enumerate()
            .map(|(i, &r)| {
                if i % 41 == 0 {
                    0.0
                } else {
                    let t = i as f32 * 0.037;
                    t.sin() * 0.6 + (r as f32 / i64::MAX as f32) * 0.02
                }
            })
            .collect()
    }

    fn bits_of(pcm: &[f32]) -> Vec<u32> {
        pcm.iter().map(|s| s.to_bits()).collect()
    }

    #[test]
    fn float_hybrid_pair_is_bit_exact_and_lossy_decodes() {
        let pcm = float_signal(2000, 0x5eed);
        for (joint, profile) in [(false, None), (true, Some(DecorrProfile::Normal))] {
            let enc =
                encode_block_mono_hybrid_float(&pcm, &opts(456, joint, profile), 0, 2000).unwrap();
            let wvc = enc.wvc.as_ref().unwrap();
            let exact = crate::block::decode_stream_with_correction_f32(&enc.wv, wvc).unwrap();
            assert_eq!(bits_of(&exact), bits_of(&pcm), "pair decode bit patterns");
            let (_, ok) = crate::block::decode_stream_with_correction_muted(&enc.wv, wvc).unwrap();
            assert!(ok, "pair CRC gate (incl. extension crc_x)");
            // Lossy-only decode succeeds (implied-zero fill) and is close.
            let lossy = crate::block::decode_stream_f32(&enc.wv).unwrap();
            let max_err = lossy
                .iter()
                .zip(&pcm)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0f32, f32::max);
            assert!(
                max_err > 0.0 && max_err < 0.2,
                "float noise bounded ({max_err})"
            );
            let (parsed, _) = crate::parse_block(&enc.wv).unwrap();
            assert!(parsed.is_float());
            assert!(!parsed.has_packed_overflow_bits(), "wvx rides the wvc twin");
            let (cblk, _) = crate::parse_block(wvc).unwrap();
            assert!(cblk.has_packed_overflow_bits());
        }
    }

    #[test]
    fn float_hybrid_stereo_pair_with_specials_is_bit_exact() {
        let mut pcm: Vec<f32> = float_signal(1200, 0x77)
            .iter()
            .flat_map(|&s| [s, -s * 0.5])
            .collect();
        pcm[100] = f32::INFINITY;
        pcm[101] = f32::from_bits(0x7f95_5555);
        pcm[200] = -0.0;
        pcm[201] = f32::from_bits(0x0000_0123);
        let enc = encode_block_stereo_hybrid_float(
            &pcm,
            &opts(456, true, Some(DecorrProfile::Fast)),
            0,
            1200,
        )
        .unwrap();
        let wvc = enc.wvc.as_ref().unwrap();
        let exact = crate::block::decode_stream_with_correction_f32(&enc.wv, wvc).unwrap();
        assert_eq!(bits_of(&exact), bits_of(&pcm));
    }

    #[test]
    fn float_hybrid_streams_round_trip() {
        let pcm = float_signal(3000, 0x31);
        let enc = encode_stream_mono_hybrid_float(
            &pcm,
            800,
            &opts(456, false, Some(DecorrProfile::Normal)),
        )
        .unwrap();
        let wvc = enc.wvc.as_ref().unwrap();
        assert_eq!(crate::audio_block_count(&enc.wv).unwrap(), 4);
        let exact = crate::block::decode_stream_with_correction_f32(&enc.wv, wvc).unwrap();
        assert_eq!(bits_of(&exact), bits_of(&pcm));

        let stereo: Vec<f32> = pcm.iter().flat_map(|&s| [s, s * 0.9]).collect();
        let enc = encode_stream_stereo_hybrid_float(
            &stereo,
            700,
            &opts(456, true, Some(DecorrProfile::Normal)),
        )
        .unwrap();
        let exact =
            crate::block::decode_stream_with_correction_f32(&enc.wv, enc.wvc.as_ref().unwrap())
                .unwrap();
        assert_eq!(bits_of(&exact), bits_of(&stereo));
    }

    #[test]
    fn int32_hybrid_pair_is_bit_exact_full_range() {
        let pcm: Vec<i32> = splitmix(0xfeed, 1500).iter().map(|&r| r as i32).collect();
        let enc = encode_block_mono_hybrid_int32(&pcm, &opts(456, false, None), 0, 1500).unwrap();
        let wvc = enc.wvc.as_ref().unwrap();
        assert_eq!(
            decode_stream_with_correction(&enc.wv, wvc).unwrap(),
            pcm,
            "full-range int32 pair decode"
        );
        let (_, ok) = decode_stream_with_correction_muted(&enc.wv, wvc).unwrap();
        assert!(ok);
        // Lossy-only decode succeeds with the implied-zero fill.
        let (lossy, ok) = decode_stream_muted(&enc.wv).unwrap();
        assert!(ok);
        assert_eq!(lossy.len(), pcm.len());
    }

    #[test]
    fn int32_hybrid_stereo_stream_round_trips() {
        // Correlated wide data across multiple blocks.
        let mut acc = 0i64;
        let base: Vec<i32> = splitmix(0xabc, 2600)
            .iter()
            .map(|&r| {
                acc += (r >> 48) << 9;
                acc as i32
            })
            .collect();
        let pcm: Vec<i32> = base.iter().flat_map(|&v| [v, v ^ 0x1FF]).collect();
        let enc = encode_stream_stereo_hybrid_int32(
            &pcm,
            800,
            &opts(456, true, Some(DecorrProfile::Normal)),
        )
        .unwrap();
        let wvc = enc.wvc.as_ref().unwrap();
        assert_eq!(decode_stream_with_correction(&enc.wv, wvc).unwrap(), pcm);
    }

    #[test]
    fn clipping_adjacent_content_clamps_the_lossy_decode_only() {
        // Round-418 pin: the lossy reconstruction saturates to the
        // effective bit-depth range AFTER the (unclamped) §5 CRC fold;
        // the pair stays bit-exact.
        let pcm: Vec<i32> = (0..3000)
            .map(|i| {
                let t = f64::from(i) * 0.03;
                ((t.sin() * 32300.0) as i32 + (i % 997) - 498).clamp(-32768, 32767)
            })
            .collect();
        let enc = encode_block_mono_hybrid(
            &pcm,
            2,
            &opts(0, false, Some(DecorrProfile::Normal)),
            0,
            3000,
        )
        .unwrap();
        let lossy = assert_pair_round_trip(&pcm, &enc);
        assert!(
            lossy.iter().all(|&s| (-32768..=32767).contains(&s)),
            "lossy output clamps to the 16-bit range"
        );
        // The coarse bitrate (word 0) on clipping content guarantees
        // some samples actually hit the clamp.
        assert!(
            lossy.iter().any(|&s| s == 32767 || s == -32768),
            "clamp exercised"
        );
    }

    #[test]
    fn redundancy_only_int32_hybrid_pair_restores_the_pattern() {
        // Trailing-ones content narrow enough that sent_bits == 0: the
        // pair decode re-inserts the ones pattern, the lossy decode
        // zero-fills the window (round-418 pins).
        let pcm: Vec<i32> = splitmix(0x0e5, 1500)
            .iter()
            .map(|&r| (((r >> 44) as i32) & !0xF) | 0xF)
            .collect();
        let enc = encode_block_mono_hybrid_int32(&pcm, &opts(456, false, None), 0, 1500).unwrap();
        let (parsed, _) = crate::parse_block(&enc.wv).unwrap();
        let info_sub = parsed.find_sub_block(crate::SubBlockId::Int32Info).unwrap();
        let info = crate::int32::expand_int32_info(info_sub.payload).unwrap();
        assert_eq!(info.sent_bits, 0, "redundancy-only profile");
        assert_eq!(info.ones, 4);
        let lossy = assert_pair_round_trip(&pcm, &enc);
        assert!(
            lossy.iter().all(|&s| s & 0xF == 0),
            "lossy window zero-fills"
        );
    }

    // ---- noise shaping (round 420) -----------------------------------

    fn shaped(sh: HybridShaping, joint: bool) -> HybridOptions {
        opts(456, joint, Some(DecorrProfile::Normal)).with_shaping(sh)
    }

    /// Lag-1 autocorrelation sign proxy of the lossy quantization
    /// noise: positive shaping weights tilt the noise spectrum upward
    /// (negative lag-1 correlation), negative weights downward.
    fn noise_autocorr1(lossy: &[i32], orig: &[i32]) -> f64 {
        let n: Vec<i64> = lossy
            .iter()
            .zip(orig)
            .map(|(&a, &b)| i64::from(a) - i64::from(b))
            .collect();
        let num: i64 = n.windows(2).map(|w| w[0] * w[1]).sum();
        let den: i64 = n.iter().map(|&v| v * v).sum();
        num as f64 / den.max(1) as f64
    }

    #[test]
    fn shaped_mono_pair_is_lossless_and_flags_the_shape() {
        let pcm = signal16(3000, 0x420);
        let enc =
            encode_block_mono_hybrid(&pcm, 2, &shaped(HybridShaping::Static(717), false), 0, 3000)
                .unwrap();
        assert_pair_round_trip(&pcm, &enc);
        let (wv_blk, _) = crate::parse_block(&enc.wv).unwrap();
        let flags = wv_blk.flags().raw;
        assert_ne!(flags & crate::hybrid::HYBRID_SHAPE_FLAG, 0, "bit 6");
        assert_ne!(flags & crate::hybrid::NEW_SHAPING_FLAG, 0, "bit 29");
        assert!(
            wv_blk.find_noise_shaping_profile_sub_block().is_none(),
            "0x07 rides the wvc twin, not the lossy stream"
        );
        let wvc = enc.wvc.as_ref().unwrap();
        let (wvc_blk, _) = crate::parse_block(wvc).unwrap();
        assert_eq!(wvc_blk.flags().raw, flags, "twin mirrors the flag word");
        let sp = wvc_blk
            .find_noise_shaping_profile_sub_block()
            .expect("0x07 in the correction block");
        // Mono static layout: [error, acc] (2 log words), zero error
        // seed, acc = quantized 717 << 16.
        assert_eq!(sp.payload.len(), 4);
        let st = crate::ShapingState::from_shaping_words(Some(sp.payload), false);
        assert_eq!(st.error(0), 0);
        // The 0x07 sub-block leads the correction chain (reference
        // pair-encode placement): first metadata id byte after the
        // 32-byte header.
        assert_eq!(wvc[32] & 0x3f, 0x07);
        // The unshaped encode keeps bits 6/29 clear and emits no 0x07.
        let plain = encode_block_mono_hybrid(
            &pcm,
            2,
            &opts(456, false, Some(DecorrProfile::Normal)),
            0,
            3000,
        )
        .unwrap();
        let (pb, _) = crate::parse_block(&plain.wv).unwrap();
        assert_eq!(
            pb.flags().raw & (crate::hybrid::HYBRID_SHAPE_FLAG | crate::hybrid::NEW_SHAPING_FLAG),
            0
        );
        let (pc, _) = crate::parse_block(plain.wvc.as_ref().unwrap()).unwrap();
        assert!(pc.find_noise_shaping_profile_sub_block().is_none());
    }

    #[test]
    fn shaped_noise_spectrum_tilts_with_the_weight() {
        let pcm = signal16(4000, 0x7171);
        let mut corr = Vec::new();
        for sh in [
            HybridShaping::Static(717),
            HybridShaping::Off,
            HybridShaping::Static(-717),
        ] {
            let enc = encode_block_mono_hybrid(&pcm, 2, &shaped(sh, false), 0, 4000).unwrap();
            let lossy = assert_pair_round_trip(&pcm, &enc);
            corr.push(noise_autocorr1(&lossy, &pcm));
        }
        assert!(
            corr[0] < corr[1] && corr[1] < corr[2],
            "lag-1 noise autocorrelation orders with the weight: {corr:?}"
        );
        assert!(corr[0] < -0.2, "positive weight pushes noise upward");
        assert!(corr[2] > 0.2, "negative weight pushes noise downward");
    }

    #[test]
    fn shaped_stereo_pairs_are_lossless_lr_and_joint() {
        let mono = signal16(2400, 0xbeef);
        let pcm: Vec<i32> = mono
            .iter()
            .enumerate()
            .flat_map(|(i, &s)| [s, s / 2 + (i as i32 % 37) - 18])
            .collect();
        for joint in [false, true] {
            for sh in [
                HybridShaping::Static(717),
                HybridShaping::Static(-717),
                HybridShaping::Static(1024),
            ] {
                let enc = encode_block_stereo_hybrid(&pcm, 2, &shaped(sh, joint), 0, 2400).unwrap();
                assert_pair_round_trip(&pcm, &enc);
                let (blk, _) = crate::parse_block(&enc.wv).unwrap();
                assert_eq!(blk.flags().joint_stereo, joint);
                let (cblk, _) = crate::parse_block(enc.wvc.as_ref().unwrap()).unwrap();
                let sp = cblk.find_noise_shaping_profile_sub_block().unwrap();
                assert_eq!(sp.payload.len(), 8, "stereo static layout: 4 log words");
            }
        }
    }

    #[test]
    fn ramp_shaping_carries_delta_words_and_state_across_blocks() {
        let pcm = signal16(5000, 0x9a);
        let sh = HybridShaping::Ramp {
            weight: -512,
            delta: 9000,
        };
        let enc = encode_stream_mono_hybrid(&pcm, 1000, 2, &shaped(sh, false)).unwrap();
        assert_pair_round_trip(&pcm, &enc);
        // Every block's 0x07 carries the 3-word mono dynamic layout and
        // the accumulator moves across blocks.
        let wvc = enc.wvc.as_ref().unwrap();
        let mut payloads = Vec::new();
        let mut rest: &[u8] = wvc;
        while !rest.is_empty() {
            let (blk, next) = crate::parse_block(rest).unwrap();
            let sp = blk.find_noise_shaping_profile_sub_block().unwrap();
            assert_eq!(sp.payload.len(), 6, "mono dynamic layout: 3 log words");
            payloads.push(sp.payload.to_vec());
            rest = next;
        }
        assert_eq!(payloads.len(), 5);
        assert_ne!(payloads[0], payloads[4], "acc state ramps across blocks");

        // Stereo joint ramp: the 6-word layout, still lossless.
        let stereo: Vec<i32> = pcm.iter().flat_map(|&s| [s, s / 3 - 7]).collect();
        let enc = encode_stream_stereo_hybrid(&stereo, 900, 2, &shaped(sh, true)).unwrap();
        assert_pair_round_trip(&stereo, &enc);
        let (cblk, _) = crate::parse_block(enc.wvc.as_ref().unwrap()).unwrap();
        let sp = cblk.find_noise_shaping_profile_sub_block().unwrap();
        assert_eq!(sp.payload.len(), 12, "stereo dynamic layout: 6 log words");
    }

    #[test]
    fn ramp_delta_words_saturate_at_the_full_scale_rail() {
        // Black-box-anchored envelope (round 420): trajectories that
        // cross below weight -1024 can decode differently under the
        // reference decoder (its negative-weight IIR arm past full
        // scale is an open docs gap), so every emitted block's 0x07
        // must keep the accumulator inside ±(1024 << 16) — the ramp
        // runs to the rail and holds.
        let rail = i64::from(1024i32 << 16);
        let pcm = signal16(6000, 0x420);
        for (w0, d) in [
            (-1000, -60_000),
            (1000, 60_000),
            (-1024, -600),
            (0, 600_000),
        ] {
            let sh = HybridShaping::Ramp {
                weight: w0,
                delta: d,
            };
            let enc = encode_stream_mono_hybrid(&pcm, 750, 2, &shaped(sh, false)).unwrap();
            assert_pair_round_trip(&pcm, &enc);
            let mut rest: &[u8] = enc.wvc.as_ref().unwrap();
            let mut last_delta = i32::MAX;
            while !rest.is_empty() {
                let (blk, next) = crate::parse_block(rest).unwrap();
                let sp = blk.find_noise_shaping_profile_sub_block().unwrap();
                let word = |i: usize| {
                    i32::from(i16::from_le_bytes([
                        sp.payload[2 * i],
                        sp.payload[2 * i + 1],
                    ]))
                };
                let acc = i64::from(crate::logpack::wp_exp2s(word(1)));
                let delta = i64::from(crate::logpack::wp_exp2s(word(2)));
                let frames = i64::from(blk.block_samples());
                assert!(acc.abs() <= rail, "block seed acc {acc} within the rail");
                assert!(
                    (acc + delta * frames).abs() <= rail,
                    "whole-block excursion within the rail (acc {acc}, delta {delta}, n {frames})"
                );
                last_delta = delta as i32;
                rest = next;
            }
            // The final block of every one of these ramps has hit the
            // rail: its delta word saturates to zero.
            assert_eq!(last_delta, 0, "w0={w0} d={d}: ramp holds at the rail");
        }
    }

    #[test]
    fn shaped_silence_and_lossless_dispatch_paths_stay_exact() {
        // Zero runs under shaping: run members decode exact == coarse
        // == 0 and reset the error state — the pair stays lossless
        // through silence stretches.
        let mut pcm = vec![0i32; 700];
        pcm.extend(signal16(900, 0x51).iter());
        pcm.extend(std::iter::repeat_n(0i32, 600));
        let n = pcm.len() as u32;
        for joint in [false, true] {
            let stereo: Vec<i32> = pcm.iter().flat_map(|&s| [s, s / 2]).collect();
            let enc = encode_block_stereo_hybrid(
                &stereo,
                2,
                &shaped(HybridShaping::Static(717), joint),
                0,
                n,
            )
            .unwrap();
            assert_pair_round_trip(&stereo, &enc);
        }
        // A degenerate-lossless bitrate word (every limit argument
        // non-positive) under shaping: the lossless dispatch applies no
        // temp and the lossy stream is already exact.
        let enc =
            encode_block_mono_hybrid(&pcm, 2, &shaped(HybridShaping::Static(717), false), 0, n)
                .unwrap();
        assert_pair_round_trip(&pcm, &enc);
        let mut o = shaped(HybridShaping::Static(-717), false);
        o.bitrate_word = 8000;
        let enc = encode_block_mono_hybrid(&pcm, 2, &o, 0, n).unwrap();
        let lossy = assert_pair_round_trip(&pcm, &enc);
        assert_eq!(lossy, pcm, "degenerate lossless words under shaping");
    }

    #[test]
    fn shaped_float_and_int32_pairs_are_bit_exact() {
        let pcm = float_signal(1600, 0x420f);
        let enc = encode_block_mono_hybrid_float(
            &pcm,
            &shaped(HybridShaping::Static(717), false),
            0,
            1600,
        )
        .unwrap();
        let wvc = enc.wvc.as_ref().unwrap();
        let exact = crate::block::decode_stream_with_correction_f32(&enc.wv, wvc).unwrap();
        assert_eq!(bits_of(&exact), bits_of(&pcm));

        let stereo: Vec<f32> = pcm.iter().flat_map(|&s| [s, -s * 0.4]).collect();
        let enc = encode_stream_stereo_hybrid_float(
            &stereo,
            700,
            &shaped(HybridShaping::Static(-717), true),
        )
        .unwrap();
        let exact =
            crate::block::decode_stream_with_correction_f32(&enc.wv, enc.wvc.as_ref().unwrap())
                .unwrap();
        assert_eq!(bits_of(&exact), bits_of(&stereo));

        let wide: Vec<i32> = splitmix(0x32, 1400).iter().map(|&r| r as i32).collect();
        let enc = encode_block_mono_hybrid_int32(
            &wide,
            &shaped(HybridShaping::Static(717), false),
            0,
            1400,
        )
        .unwrap();
        assert_eq!(
            decode_stream_with_correction(&enc.wv, enc.wvc.as_ref().unwrap()).unwrap(),
            wide
        );
    }

    #[test]
    fn extreme_imbalance_joint_content_round_trips_across_bitrate_words() {
        // Forward direction of the round-418 §6.5 delta-clamp probes:
        // joint content whose side (R = -L + 3) or mid (R = L + 3)
        // channel collapses drives the stereo redistribution delta far
        // past ±bitrate; the encoder and decoder share the clamped
        // frame-limit derivation, so the pair must stay bit-exact
        // across the probed word range (black-box: the reference
        // decoder reproduces these streams' lossy PCM and recovers the
        // original from the pair, all six variants).
        let l = signal16(2600, 0x1338);
        for mk in [0, 1] {
            let pcm: Vec<i32> = l
                .iter()
                .flat_map(|&s| if mk == 0 { [s, -s + 3] } else { [s, s + 3] })
                .collect();
            for word in [200, 456, 800] {
                let enc =
                    encode_block_stereo_hybrid(&pcm, 2, &opts(word, true, None), 0, 2600).unwrap();
                assert_pair_round_trip(&pcm, &enc);
            }
        }
    }

    #[test]
    fn shaped_clipping_content_keeps_the_unclamped_crc_contract() {
        // Round-418 output-clamp pin, shaped variant: the lossy §5 CRC
        // folds over the UNCLAMPED reconstruction and the output
        // saturates afterwards; shaping must not disturb either side.
        let pcm: Vec<i32> = (0..3000)
            .map(|i| {
                let t = f64::from(i) * 0.03;
                (((t.sin() * 32300.0) as i32) + (i % 997) - 498).clamp(-32768, 32767)
            })
            .collect();
        let mut o =
            opts(0, false, Some(DecorrProfile::Normal)).with_shaping(HybridShaping::Static(717));
        o.bitrate_word = 0;
        let enc = encode_block_mono_hybrid(&pcm, 2, &o, 0, 3000).unwrap();
        let lossy = assert_pair_round_trip(&pcm, &enc);
        assert!(lossy.iter().all(|&s| (-32768..=32767).contains(&s)));
        assert!(lossy.iter().any(|&s| s == 32767 || s == -32768));
    }

    #[test]
    fn shaped_trailing_ones_int32_pair_restores_the_pattern() {
        // Round-418 implied-fill pin, shaped joint variant.
        let pcm: Vec<i32> = splitmix(0x0e5e, 2000)
            .iter()
            .map(|&r| (((r >> 40) as i32) & !0xF) | 0xF)
            .collect();
        let enc = encode_block_stereo_hybrid_int32(
            &pcm,
            &shaped(HybridShaping::Static(-717), true),
            0,
            1000,
        )
        .unwrap();
        let wvc = enc.wvc.as_ref().unwrap();
        assert_eq!(decode_stream_with_correction(&enc.wv, wvc).unwrap(), pcm);
        let (lossy, ok) = decode_stream_muted(&enc.wv).unwrap();
        assert!(ok);
        assert!(
            lossy.iter().all(|&s| s & 0xF == 0),
            "lossy window zero-fills"
        );
    }

    #[test]
    fn shaping_from_weight_maps_the_reference_scale() {
        assert_eq!(HybridShaping::from_weight(0.0), HybridShaping::Off);
        assert_eq!(HybridShaping::from_weight(0.7), HybridShaping::Static(717));
        assert_eq!(
            HybridShaping::from_weight(-0.7),
            HybridShaping::Static(-717)
        );
        assert_eq!(HybridShaping::from_weight(3.0), HybridShaping::Static(1024));
        assert_eq!(
            HybridShaping::from_weight(-3.0),
            HybridShaping::Static(-1024)
        );
        // Out-of-range static weights clamp at payload build time; the
        // pair decode stays lossless.
        let pcm = signal16(1200, 0x5);
        let enc = encode_block_mono_hybrid(
            &pcm,
            2,
            &shaped(HybridShaping::Static(30000), false),
            0,
            1200,
        )
        .unwrap();
        assert_pair_round_trip(&pcm, &enc);
    }

    #[test]
    fn format_hybrid_refuses_empty_and_odd_stereo() {
        let o = HybridOptions::default();
        assert!(matches!(
            encode_block_mono_hybrid_float(&[], &o, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_hybrid_float(&[1.0], &o, 0, 1),
            Err(Error::EncodeStereoOddLength(1))
        ));
        assert!(matches!(
            encode_block_mono_hybrid_int32(&[], &o, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_hybrid_int32(&[1], &o, 0, 1),
            Err(Error::EncodeStereoOddLength(1))
        ));
    }
}
