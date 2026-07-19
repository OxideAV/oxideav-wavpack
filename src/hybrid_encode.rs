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
//!   block (bit 9 always, bit 10 on stereo). No shaping is emitted
//!   (bits 6/29 clear, no `0x07`) — the raw §4.1 fold, the shape the
//!   reference's shaping-off mode produces.
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
}

impl HybridOptions {
    /// Options for a bits-per-sample target (the reference `-b` scale:
    /// ~2.0 aggressive … 6.0+ near-lossless), with a correction twin,
    /// joint stereo coding and the `Normal` decorrelation ceiling.
    #[must_use]
    pub fn from_bits_per_sample(bits: f64) -> Self {
        let word = (bits * 256.0).round() as i32 - BITRATE_WORD_BIAS;
        HybridOptions {
            bitrate_word: word.clamp(0, i32::from(i16::MAX)),
            correction: true,
            joint: true,
            profile: Some(DecorrProfile::Normal),
        }
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
fn hybrid_entropy_mono(
    coded: &[i32],
    hybrid: &mut HybridState,
    stepper: &mut MonoStepper,
    correction: bool,
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
        let exact = coded[i].wrapping_sub(stepper.offset());
        let (mag, _) = split_sign(exact);
        max_res_bits = max_res_bits.max(32 - mag.leading_zeros());
        if let Some(p) = pending.take() {
            emit_pending(&mut writer, &mut run, p, Some((mag, medians.get_med(0))))?;
        }
        if run_break {
            run_break = false;
        } else if medians.values[0] <= 1 && !run.last_one && !run.last_zero {
            // §4.2 step 1: the maximal run of zero exact residuals.
            let mut run_len = 0u32;
            let mut exact_i = exact;
            while i < n && exact_i == 0 {
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
        let limit = hybrid.frame_limits()[0];
        let zone = medians.zone_for_magnitude(mag);
        let interval = medians.sample_interval_for_ones_count(zone);
        medians.adapt(Zone::from_ones_count(zone));
        let (word, coarse_res) = plan_word(&interval, zone, exact, limit, wvc.as_mut())?;
        pending = Some(word);
        let (cmag, _) = split_sign(coarse_res);
        max_res_bits = max_res_bits.max(32 - cmag.leading_zeros());
        hybrid.update_signed(0, coarse_res);
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

/// The §6.5 bracketed entropy encode of a whole interleaved **stereo**
/// buffer (frame-start limit snapshots, stream-level zero runs,
/// shared holding state).
fn hybrid_entropy_stereo(
    coded: &[i32],
    hybrid: &mut HybridState,
    stepper: &mut StereoStepper,
    correction: bool,
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
    let mut i = 0usize;
    while i < slots {
        let ch = i & 1;
        if ch == 0 {
            limits = hybrid.frame_limits();
        }
        let exact = if ch == 0 {
            coded[i].wrapping_sub(stepper.offset_a())
        } else {
            coded[i].wrapping_sub(stepper.offset_b(frame_res_a))
        };
        let (mag, _) = split_sign(exact);
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
            // Stream-level zero run across both channels.
            let mut run_len = 0u32;
            let mut exact_i = exact;
            while i < slots && exact_i == 0 {
                let chx = i & 1;
                hybrid.update_signed(chx, 0);
                if chx == 0 {
                    frame_res_a = 0;
                } else {
                    let (a, b) = stepper.advance(frame_res_a, 0);
                    coarse_out.push(a);
                    coarse_out.push(b);
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
        let (word, coarse_res) = plan_word(&interval, zone, exact, limits[ch], wvc.as_mut())?;
        pending = Some(word);
        let (cmag, _) = split_sign(coarse_res);
        max_res_bits = max_res_bits.max(32 - cmag.leading_zeros());
        hybrid.update_signed(ch, coarse_res);
        if ch == 0 {
            frame_res_a = coarse_res;
        } else {
            let (a, b) = stepper.advance(frame_res_a, coarse_res);
            coarse_out.push(a);
            coarse_out.push(b);
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
/// twin, and the final per-channel `slow_level` state (carried across
/// blocks by the stream encoders).
pub(crate) type HybridBlock = (Vec<u8>, Option<Vec<u8>>, [u32; 2]);

/// One hybrid block's `.wv` (+ optional `.wvc`) assembly from the
/// pre-format integer buffer.
#[allow(clippy::too_many_arguments)]
pub(crate) fn encode_hybrid_block_ints(
    pcm: &[i32],
    mono: bool,
    bytes_per_sample: u8,
    opts: &HybridOptions,
    level_words: [i16; 2],
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

    let streams = if mono {
        let mut stepper = MonoStepper::new(passes)?;
        hybrid_entropy_mono(&coded, &mut hybrid, &mut stepper, opts.correction)?
    } else {
        let mut stepper = StereoStepper::new(passes)?;
        hybrid_entropy_stereo(&coded, &mut hybrid, &mut stepper, opts.correction)?
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

    // The .wvc twin: same header fields, lossless CRC, 0x0B (+ 0x0C).
    let wvc = match streams.wvc {
        Some(correction_payload) => {
            let mut cmeta = Vec::new();
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

    Ok((wv, wvc, [hybrid.slow_level(0), hybrid.slow_level(1)]))
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
    let level = hybrid_seed(pcm, true, opts)?;
    let (wv, wvc, _) = encode_hybrid_block_ints(
        pcm,
        true,
        bytes_per_sample,
        opts,
        level,
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
    let level = hybrid_seed(pcm, false, opts)?;
    let (wv, wvc, _) = encode_hybrid_block_ints(
        pcm,
        false,
        bytes_per_sample,
        opts,
        level,
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
    let mut level: Option<[i16; 2]> = None;
    for window in pcm.chunks(chunk) {
        let level_words = match level {
            Some(words) => words,
            None => hybrid_seed(window, true, opts)?,
        };
        let (blk, cblk, sl) = encode_hybrid_block_ints(
            window,
            true,
            bytes_per_sample,
            opts,
            level_words,
            None,
            index,
            total,
        )?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        level = Some([pack_level_word(sl[0]), pack_level_word(sl[1])]);
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
    let mut level: Option<[i16; 2]> = None;
    for window in pcm.chunks(pairs * 2) {
        let level_words = match level {
            Some(words) => words,
            None => hybrid_seed(window, false, opts)?,
        };
        let (blk, cblk, sl) = encode_hybrid_block_ints(
            window,
            false,
            bytes_per_sample,
            opts,
            level_words,
            None,
            index,
            total,
        )?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        level = Some([pack_level_word(sl[0]), pack_level_word(sl[1])]);
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
fn encode_hybrid_block_float(
    pcm: &[f32],
    mono: bool,
    opts: &HybridOptions,
    level_words: Option<[i16; 2]>,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridBlock> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if !mono && pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let d = crate::float::deconstruct_float_raised(pcm);
    let format = crate::encode::float_format_extras(&d);
    let level = match level_words {
        Some(words) => words,
        None => hybrid_seed(&d.integers, mono, opts)?,
    };
    let (wv, wvc, sl) = encode_hybrid_block_ints(
        &d.integers,
        mono,
        4,
        opts,
        level,
        Some(&format),
        block_index,
        total_samples,
    )?;
    // Verify the lossy stream reconstructs (the implied-zero float
    // fixup can refuse a coarse magnitude past the mantissa window).
    crate::block::decode_stream(&wv)?;
    Ok((wv, wvc, sl))
}

/// Shared body of the int32-hybrid block encoders.
fn encode_hybrid_block_int32(
    pcm: &[i32],
    mono: bool,
    opts: &HybridOptions,
    level_words: Option<[i16; 2]>,
    block_index: u32,
    total_samples: u32,
) -> Result<HybridBlock> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if !mono && pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let d = crate::int32::deconstruct_int32(pcm);
    let format = crate::encode::int32_format_extras(&d);
    let level = match level_words {
        Some(words) => words,
        None => hybrid_seed(&d.reduced, mono, opts)?,
    };
    encode_hybrid_block_ints(
        &d.reduced,
        mono,
        4,
        opts,
        level,
        Some(&format),
        block_index,
        total_samples,
    )
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
    let (wv, wvc, _) =
        encode_hybrid_block_float(pcm, true, opts, None, block_index, total_samples)?;
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
    let (wv, wvc, _) =
        encode_hybrid_block_float(pcm, false, opts, None, block_index, total_samples)?;
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
    let (wv, wvc, _) =
        encode_hybrid_block_int32(pcm, true, opts, None, block_index, total_samples)?;
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
    let (wv, wvc, _) =
        encode_hybrid_block_int32(pcm, false, opts, None, block_index, total_samples)?;
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
    let mut level: Option<[i16; 2]> = None;
    for window in pcm.chunks(chunk) {
        let (blk, cblk, sl) = encode_hybrid_block_float(window, true, opts, level, index, total)?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        level = Some([pack_level_word(sl[0]), pack_level_word(sl[1])]);
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
    let mut level: Option<[i16; 2]> = None;
    for window in pcm.chunks(pairs * 2) {
        let (blk, cblk, sl) = encode_hybrid_block_float(window, false, opts, level, index, total)?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        level = Some([pack_level_word(sl[0]), pack_level_word(sl[1])]);
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
    let mut level: Option<[i16; 2]> = None;
    for window in pcm.chunks(chunk) {
        let (blk, cblk, sl) = encode_hybrid_block_int32(window, true, opts, level, index, total)?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        level = Some([pack_level_word(sl[0]), pack_level_word(sl[1])]);
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
    let mut level: Option<[i16; 2]> = None;
    for window in pcm.chunks(pairs * 2) {
        let (blk, cblk, sl) = encode_hybrid_block_int32(window, false, opts, level, index, total)?;
        wv.extend_from_slice(&blk);
        if let Some(cblk) = cblk {
            wvc_all.extend_from_slice(&cblk);
        }
        level = Some([pack_level_word(sl[0]), pack_level_word(sl[1])]);
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
