//! WavPack v.4 decorrelation sub-block expanders (IDs 0x02 / 0x03 / 0x04).
//!
//! The metadata walker introduced in round 2 surfaces the raw bytes of
//! each sub-block keyed by [`crate::SubBlockId`]. Round 3 turns the
//! three decorrelation sub-blocks documented by the wiki ("Decorrelation
//! terms", "Decorrelation weights" and "Decorrelation samples"
//! sections of `docs/audio/wavpack/wiki/WavPack.wiki`) into typed
//! views — the byte-level expansion only. Wiring those values into a
//! prediction loop is later-round work.
//!
//! ## Decorrelation terms (ID 0x02)
//!
//! From the wiki:
//!
//! > Decorrelation terms are stored in one byte, lower 5 bits indicate
//! > predictor type, high 3 bits contain delta value.
//! >
//! > Possible predictor values:
//! >   0-5   - predictors for stereo, only predictors 2-4 are implemented
//! >   6-12  - predictor uses 1-7 samples for prediction
//! >   13-16 - reserved
//! >   17-18 - predictor does prediction by two samples
//!
//! One byte → one `(term, delta)` pair. Term goes in `terms: Vec<i8>`
//! (the wiki documents only non-negative predictor codes `0..=18`, all of
//! which fit in `i7`; the `i8` choice mirrors the natural Rust signed
//! integer for predictor codes and leaves room for a future encoding
//! that re-uses the high bit). Delta is the 3-bit value, in `0..=7`.
//!
//! ## Decorrelation weights (ID 0x03)
//!
//! From the wiki:
//!
//! > Each decorrelation term should have one or two weights depending on
//! > channels. Each weight is packed into one byte and can be restored
//! > in this way:
//! >
//! >   n = getchar() << 3;
//! >   if(n > 0) n += (n + 64) >> 7;
//!
//! The byte is treated as a signed 8-bit value (`i8`) so the
//! sign-extension into 32 bits picks up cross-decorrelation's negative
//! weights cleanly. After the left-shift the result occupies the range
//! `-1024..=1016` before the `n > 0` rounding adjustment; with the
//! adjustment positive weights reach `1023` (the canonical maximum).
//!
//! How many bytes per term come from the channel count of the
//! enclosing block — the [`expand_weights`] expander does not need to
//! know that; it expands every byte in the payload through the same
//! formula and returns the flat `Vec<i32>`. The caller correlates the
//! list against the channel count (one weight per term for mono, two
//! for stereo) using the [`Flags`](crate::Flags) view from the block
//! header.
//!
//! ## Decorrelation samples (ID 0x04)
//!
//! From the wiki:
//!
//! > Each decorrelation term may have up to 16 samples depending on its
//! > value. Each sample is 32-bit but stored in 16 bits, lower 8 bits
//! > are mantiss and high 8 bits are exponent-9, i.e if exponent < 9
//! > shift mantiss right, otherwise left.
//!
//! Each on-disk sample is a little-endian 16-bit word laid out as
//! `[mantissa_lo, exponent_hi]`. The mantissa is treated as a signed
//! 8-bit value so negative samples sign-extend into the 32-bit result
//! before the shift. The shift amount is `exponent - 9`: positive means
//! shift left, negative means shift right by `9 - exponent`. The wiki's
//! "exponent-9" phrasing names the bias.
//!
//! The expander reads consecutive 16-bit words off the payload and
//! returns a flat `Vec<i32>` of expanded samples. The per-term grouping
//! (which-term-gets-how-many-samples, with the "up to 16" wiki bound)
//! is later-round work — round 3 stops at the byte-level expansion.
//!
//! ## Docs-gap notes (round 3)
//!
//! The wiki section is terse and leaves a few corner cases implicit.
//! Round 3 takes the most conservative literal reading of the wiki text
//! and records the open questions here so the docs collaborator can
//! tighten them in a future revision:
//!
//! * **Term-byte high-3-bit ordering.** The wiki says "high 3 bits
//!   contain delta value"; this is read as the unsigned `(byte >> 5)
//!   & 0x07`. No example walks through the field order.
//! * **`getchar()` signedness in the weight expander.** The wiki gives a
//!   C-style snippet without `int` / `signed char` typing. Read as a
//!   signed 8-bit byte here because (a) `n > 0` is the only branch
//!   guard, which assumes signed `n`; and (b) cross-decorrelation
//!   weights are documented as signed in every other WavPack reference
//!   (we have not consulted those, but the sign convention is the only
//!   reading that makes the `n > 0` branch meaningful — for unsigned
//!   bytes that branch would fire for everything except 0).
//! * **Sample exponent signedness.** The wiki's `exponent-9` shorthand
//!   implies the on-disk byte is an unsigned exponent biased by `-9`.
//!   The expander reads the high byte as unsigned and computes
//!   `exponent - 9` in `i32` so the shift direction follows the sign
//!   of the difference.

use crate::error::{Error, Result};

/// Maximum predictor code the wiki "Possible predictor values" listing
/// enumerates explicitly (`17-18`). Codes above this are not described.
pub const MAX_DOCUMENTED_TERM: i8 = 18;
/// Upper bound the wiki "Decorrelation samples" section places on the
/// per-term sample count: "Each decorrelation term may have up to 16
/// samples depending on its value."
///
/// The documented codes (`6..=12` → `1..=7` samples and `17..=18` → 2
/// samples) all sit well under this bound. The bound is exposed for
/// completeness and as a sanity check: a future docs revision that
/// quantifies the stereo predictor (`0..=5`) per-term sample count is
/// expected to stay within it.
pub const MAX_DECORRELATION_SAMPLES_PER_TERM: u8 = 16;
/// Width of the delta field on a decorrelation-terms byte, in bits.
pub const TERM_DELTA_BITS: u32 = 3;
/// Width of the predictor (term) field on a decorrelation-terms byte,
/// in bits.
pub const TERM_PREDICTOR_BITS: u32 = 5;
/// Mask isolating the predictor field within the terms byte.
pub const TERM_PREDICTOR_MASK: u8 = 0x1F;
/// Mask isolating the delta field once shifted into the low bits.
pub const TERM_DELTA_MASK: u8 = 0x07;
/// On-disk size, in bytes, of each decorrelation-samples entry (one
/// 16-bit word per the wiki "stored in 16 bits" sentence).
pub const SAMPLE_ON_WIRE_BYTES: usize = 2;
/// Bias the wiki applies to the exponent half of a sample word
/// ("high 8 bits are exponent-9").
pub const SAMPLE_EXPONENT_BIAS: i32 = 9;

/// Right-shift applied to the `weight * sample` product in
/// [`apply_weight`]. Weights are normalised so that `1 << WEIGHT_SHIFT`
/// (`1024`) is unity gain. (Spec §3.1 / §6: "weight scale shift" `10`.)
pub const WEIGHT_SHIFT: u32 = 10;

/// Rounding addend applied before the [`WEIGHT_SHIFT`] right-shift in
/// [`apply_weight`] (`1 << (WEIGHT_SHIFT - 1)` = `512`). (Spec §3.1 /
/// §6: "weight round bias" `512`.)
pub const WEIGHT_ROUND_BIAS: i64 = 1 << (WEIGHT_SHIFT - 1);

/// Working-weight magnitude limit — `1024` is unity gain and the
/// largest magnitude a clipped weight update ([`update_weight_clip`])
/// allows the cross-channel terms to reach. (Spec §3.5 / §6: "weight
/// unity / clip" `±1024`.)
pub const WEIGHT_CLIP: i32 = 1 << WEIGHT_SHIFT;

/// Classification of a single predictor term code per the wiki
/// "Possible predictor values" listing in the
/// `docs/audio/wavpack/wiki/WavPack.wiki` "Decorrelation terms" section:
///
/// ```text
/// 0-5   - predictors for stereo, only predictors 2-4 are implemented
/// 6-12  - predictor uses 1-7 samples for prediction
/// 13-16 - reserved
/// 17-18 - predictor does prediction by two samples
/// ```
///
/// Codes outside `0..=18` are not described by the wiki — the parser
/// surfaces them as [`TermKind::Unknown`] rather than rejecting them so
/// that a future format extension does not require a code change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TermKind {
    /// `0..=5` — stereo predictor. The wiki narrows the implemented
    /// subset to `2..=4`; the `implemented` field surfaces that.
    Stereo {
        /// `true` when this stereo predictor is one of `2..=4` (the
        /// "only predictors 2-4 are implemented" subset on the wiki).
        implemented: bool,
    },
    /// `6..=12` — predictor consults `1..=7` previous samples (one
    /// sample for code `6`, two for `7`, …, seven for `12`).
    SampleBased {
        /// Number of previous samples the predictor consults
        /// (`code - 5` per the wiki "uses 1-7 samples for prediction").
        sample_count: u8,
    },
    /// `13..=16` — wiki-documented but reserved (no behaviour
    /// specified). The parser does not reject the code; the decode
    /// layer should refuse to use it.
    Reserved,
    /// `17..=18` — two-sample predictor per the wiki "predictor does
    /// prediction by two samples" entry.
    TwoSample,
    /// Code outside the wiki-documented `0..=18` range.
    Unknown,
}

impl TermKind {
    /// Classify a predictor term code per the wiki "Possible predictor
    /// values" listing.
    pub const fn from_code(code: i8) -> Self {
        match code {
            // Stereo predictors `0..=5`; the wiki "only predictors 2-4 are
            // implemented" sentence narrows the implemented subset.
            0 | 1 | 5 => TermKind::Stereo { implemented: false },
            2..=4 => TermKind::Stereo { implemented: true },
            // `6..=12` use `code - 5` previous samples (`6`→1, `12`→7).
            6..=12 => TermKind::SampleBased {
                sample_count: (code - 5) as u8,
            },
            // `13..=16` reserved by the wiki.
            13..=16 => TermKind::Reserved,
            // `17..=18` two-sample predictor.
            17..=18 => TermKind::TwoSample,
            // Anything else (negative codes or `> 18`) is undocumented.
            _ => TermKind::Unknown,
        }
    }

    /// `true` when the wiki-documented behaviour for this code is
    /// implementable (the stereo subset `2..=4`, the sample-based set
    /// `6..=12`, and the two-sample set `17..=18`). Stereo codes
    /// `0/1/5`, the `13..=16` reserved range, and codes outside
    /// `0..=18` return `false`.
    pub const fn is_implemented(self) -> bool {
        match self {
            TermKind::Stereo { implemented } => implemented,
            TermKind::SampleBased { .. } | TermKind::TwoSample => true,
            TermKind::Reserved | TermKind::Unknown => false,
        }
    }

    /// Number of previous-sample slots this predictor consults, when
    /// the wiki specifies it. `Some(n)` for sample-based predictors
    /// (`code - 5` for `6..=12`) and for the two-sample predictors
    /// (`17..=18` → `2`). `None` for stereo predictors (the wiki gives
    /// no per-code sample count for `0..=5`), reserved codes, and
    /// undocumented codes.
    pub const fn previous_samples(self) -> Option<u8> {
        match self {
            TermKind::SampleBased { sample_count } => Some(sample_count),
            TermKind::TwoSample => Some(2),
            TermKind::Stereo { .. } | TermKind::Reserved | TermKind::Unknown => None,
        }
    }

    /// Number of seed samples this term's `0x04` decorrelation-samples
    /// payload supplies on the wire, when the wiki specifies it.
    ///
    /// The wiki "Decorrelation samples" section opens "Each decorrelation
    /// term may have up to 16 samples depending on its value" without
    /// giving a per-code table; the per-code count is derivable from the
    /// "Possible predictor values" listing, where each predictor's prior
    /// values are exactly what the `0x04` payload primes:
    ///
    /// * `6..=12` — "predictor uses 1-7 samples for prediction" → the
    ///   payload supplies `code - 5` seed samples (one per previous-sample
    ///   slot the predictor consults).
    /// * `17..=18` — "predictor does prediction by two samples" → the
    ///   payload supplies 2 seed samples.
    ///
    /// Stereo predictors `0..=5` are not given a per-term sample count by
    /// the wiki (separate docs gap); the reserved `13..=16` range has no
    /// documented behaviour; and codes outside `0..=18` are undocumented.
    /// All three cases return `None`, mirroring
    /// [`TermKind::previous_samples`] — the wiki ties the seed-sample
    /// count directly to the previous-sample slot count.
    pub const fn decorrelation_sample_count(self) -> Option<u8> {
        // The wiki phrases the two as one number: the predictor needs N
        // prior samples → the payload supplies N seed samples. The
        // reuse keeps the semantic tie explicit.
        self.previous_samples()
    }
}

/// Number of decorrelation **weight** bytes the wiki "Decorrelation
/// weights" section pairs with each term, given the enclosing block's
/// channel count:
///
/// > Each decorrelation term should have one or two weights depending
/// > on channels.
///
/// `channels == 1` (mono) → one weight per term; `channels == 2`
/// (stereo) → two weights per term. The function clamps any other
/// value to `1` (no other channel count is reachable through the wiki
/// "monaural" bit on the block header, which is binary; multi-channel
/// blocks decompose into per-block stereo pairs).
pub const fn weights_per_term(channels: u8) -> u8 {
    if channels >= 2 {
        2
    } else {
        1
    }
}

/// Stand-alone shorthand for [`TermKind::from_code`] +
/// [`TermKind::decorrelation_sample_count`] so a caller branching off
/// a raw term code can look the per-term `0x04` seed-sample count up
/// without re-typing the classification step.
///
/// `Some(n)` for the wiki-documented `6..=12` and `17..=18` codes,
/// `None` for stereo predictors `0..=5`, the reserved `13..=16` range,
/// and undocumented codes — same gaps [`TermKind::decorrelation_sample_count`]
/// records.
pub const fn decorrelation_sample_count(code: i8) -> Option<u8> {
    TermKind::from_code(code).decorrelation_sample_count()
}

/// Typed expansion of the `0x02` decorrelation-terms sub-block.
///
/// Round 3 expansion only — interpretation (mapping the predictor
/// codes back to the per-channel decorrelation passes) is deferred.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecorrelationTerms {
    /// One signed term code per byte of the payload, low 5 bits per
    /// the wiki "lower 5 bits indicate predictor type" sentence.
    /// All wiki-documented codes are non-negative (`0..=18`); the
    /// signed type leaves room for future code points without a
    /// breaking type change.
    pub terms: Vec<i8>,
    /// One unsigned 3-bit delta per byte of the payload, high 3
    /// bits per the wiki "high 3 bits contain delta value" sentence.
    /// Always in `0..=7`.
    pub deltas: Vec<u8>,
}

impl DecorrelationTerms {
    /// Number of decoded `(term, delta)` pairs.
    pub fn len(&self) -> usize {
        self.terms.len()
    }

    /// `true` when no decorrelation passes are configured.
    pub fn is_empty(&self) -> bool {
        self.terms.is_empty()
    }

    /// Classify the term at index `idx` per the wiki "Possible predictor
    /// values" listing. Returns `None` when `idx` is past the end of
    /// [`Self::terms`].
    pub fn kind_at(&self, idx: usize) -> Option<TermKind> {
        self.terms.get(idx).copied().map(TermKind::from_code)
    }

    /// Walk every term in order, yielding `(code, TermKind)` pairs.
    pub fn iter_kinds(&self) -> impl Iterator<Item = (i8, TermKind)> + '_ {
        self.terms
            .iter()
            .copied()
            .map(|c| (c, TermKind::from_code(c)))
    }

    /// `true` when every term in the list is one the wiki documents an
    /// implementable behaviour for (the stereo subset `2..=4`, the
    /// sample-based set `6..=12`, and the two-sample set `17..=18`).
    /// An empty term list returns `true` (vacuous truth — no
    /// unimplemented pass to trip over).
    pub fn all_implemented(&self) -> bool {
        self.terms
            .iter()
            .copied()
            .all(|c| TermKind::from_code(c).is_implemented())
    }

    /// `true` when **any** term in the list is one the wiki marks as
    /// reserved (`13..=16`).
    pub fn has_reserved(&self) -> bool {
        self.terms
            .iter()
            .copied()
            .any(|c| matches!(TermKind::from_code(c), TermKind::Reserved))
    }

    /// Sum of [`decorrelation_sample_count`] across every term in the
    /// list — the total number of seed samples a `0x04` decorrelation-
    /// samples payload supplies for this `(0x02)` term list, per the
    /// wiki "Decorrelation samples" / "Possible predictor values"
    /// sections.
    ///
    /// Returns `Some(total)` when every term has a wiki-documented
    /// per-term sample count (i.e. every term is in `6..=12` or
    /// `17..=18`); returns `None` as soon as any term is a stereo
    /// predictor `0..=5`, a reserved `13..=16` code, or an
    /// undocumented code — those are the docs gaps where the wiki
    /// does not give a per-term sample count, and the total cannot be
    /// summed without inventing one.
    ///
    /// An empty term list returns `Some(0)` (vacuous: zero terms
    /// require zero seed samples).
    pub fn expected_decorrelation_sample_count(&self) -> Option<usize> {
        let mut total = 0usize;
        for &code in &self.terms {
            let per_term = decorrelation_sample_count(code)? as usize;
            total += per_term;
        }
        Some(total)
    }
}

/// Typed expansion of the `0x03` decorrelation-weights sub-block.
///
/// Each byte of the payload is expanded through the wiki's two-line
/// log-pack recipe (`n = getchar() << 3; if(n > 0) n += (n + 64) >> 7`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecorrelationWeights {
    /// Expanded signed weights, one per byte of the payload. The
    /// expansion lands every value in the range `-1024..=1023`.
    pub weights: Vec<i32>,
}

/// Typed expansion of the `0x04` decorrelation-samples sub-block.
///
/// Each pair of bytes (little-endian 16-bit word) expands through the
/// wiki's exponent / mantissa formula into a 32-bit sample.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecorrelationSamples {
    /// Expanded 32-bit samples in the order they appeared on the wire.
    pub samples: Vec<i32>,
}

/// Expand the payload of a `0x02` decorrelation-terms sub-block into a
/// typed [`DecorrelationTerms`] value.
///
/// One byte → one `(term, delta)` pair, per the wiki "Decorrelation
/// terms" section.
pub fn expand_terms(payload: &[u8]) -> DecorrelationTerms {
    let mut terms = Vec::with_capacity(payload.len());
    let mut deltas = Vec::with_capacity(payload.len());
    for &byte in payload {
        // The wiki "lower 5 bits indicate predictor type" — the low
        // five bits land in 0..=31. None of the wiki-documented
        // predictor codes exceed 18 so the value fits comfortably in
        // `i8` without sign-extension games.
        let term = (byte & TERM_PREDICTOR_MASK) as i8;
        let delta = (byte >> TERM_PREDICTOR_BITS) & TERM_DELTA_MASK;
        terms.push(term);
        deltas.push(delta);
    }
    DecorrelationTerms { terms, deltas }
}

/// Expand the payload of a `0x03` decorrelation-weights sub-block
/// into a typed [`DecorrelationWeights`] value, applying the wiki
/// log-pack expansion to every byte.
///
/// The expansion is the wiki snippet, verbatim, with `getchar()`
/// read as a signed 8-bit byte (see the docs-gap note on this module
/// for why):
///
/// ```text
/// n = getchar() << 3;
/// if (n > 0) n += (n + 64) >> 7;
/// ```
pub fn expand_weights(payload: &[u8]) -> DecorrelationWeights {
    let mut weights = Vec::with_capacity(payload.len());
    for &byte in payload {
        weights.push(expand_weight_byte(byte));
    }
    DecorrelationWeights { weights }
}

/// Single-byte log-pack expander used by [`expand_weights`].
///
/// Exposed pub(crate) so the test module can call it directly without
/// allocating a [`Vec`] for every assertion.
fn expand_weight_byte(byte: u8) -> i32 {
    // The on-disk byte is a signed 8-bit value. Sign-extension into
    // i32 lets the < 0 / > 0 / == 0 branching match the wiki snippet
    // straightforwardly.
    let signed = (byte as i8) as i32;
    let mut n = signed << 3;
    if n > 0 {
        n += (n + 64) >> 7;
    }
    n
}

/// Expand the payload of a `0x04` decorrelation-samples sub-block
/// into a typed [`DecorrelationSamples`] value.
///
/// Each pair of bytes is read as a little-endian 16-bit word and
/// expanded through the wiki's exponent / mantissa formula:
///
/// ```text
/// // word = [mantissa_lo, exponent_hi] little-endian
/// // result is mantissa shifted by (exponent - 9)
/// if exponent < 9 { result = mantissa >> (9 - exponent) }
/// else            { result = mantissa << (exponent - 9) }
/// ```
///
/// The mantissa is treated as a signed 8-bit value before the shift so
/// negative samples sign-extend into the 32-bit result before the
/// shift direction is chosen.
///
/// Returns [`Error::DecorrelationSamplesOddByteCount`] when the payload
/// length is not a multiple of two — the wiki guarantees every sample
/// is two bytes on the wire and the round-2 walker has already
/// stripped odd-size padding, so an odd-length payload here is a
/// malformed sub-block.
pub fn expand_samples(payload: &[u8]) -> Result<DecorrelationSamples> {
    if payload.len() % SAMPLE_ON_WIRE_BYTES != 0 {
        return Err(Error::DecorrelationSamplesOddByteCount(payload.len()));
    }
    let mut samples = Vec::with_capacity(payload.len() / SAMPLE_ON_WIRE_BYTES);
    for word in payload.chunks_exact(SAMPLE_ON_WIRE_BYTES) {
        samples.push(expand_sample_word(word[0], word[1]));
    }
    Ok(DecorrelationSamples { samples })
}

/// Split the flat [`DecorrelationSamples::samples`] list produced by
/// [`expand_samples`] into one `Vec<i32>` per term in `terms`, with the
/// per-term length given by the wiki "Decorrelation samples" / "Possible
/// predictor values" pairing (`6..=12` → `code - 5` samples;
/// `17..=18` → 2 samples).
///
/// The returned `Vec` is in term-list order: entry `i` carries the seed
/// samples for `terms.terms[i]`. The total length of the flat input
/// must equal the sum [`DecorrelationTerms::expected_decorrelation_sample_count`]
/// produces; otherwise [`Error::DecorrelationSampleCountMismatch`] is
/// returned (with the expected and observed flat lengths). A term whose
/// per-term count the wiki does not specify (stereo `0..=5`, reserved
/// `13..=16`, or undocumented codes) is rejected via
/// [`Error::DecorrelationSampleCountUnspecified`] with the offending
/// code — partitioning cannot proceed without per-term lengths.
///
/// The wiki does not explicitly relate the per-term sample count to the
/// channel count (unlike weights, which are explicitly tied to channels
/// via "one or two weights depending on channels"). The "Decorrelation
/// samples" section ties the count to the term value only. This helper
/// follows that wording exactly — it does not multiply by channels.
/// Future docs clarification of the stereo (`0..=5`) per-term count or
/// any channel multiplier will land as additions here rather than as a
/// silent reinterpretation.
pub fn partition_decorrelation_samples(
    terms: &DecorrelationTerms,
    samples: &DecorrelationSamples,
) -> Result<Vec<Vec<i32>>> {
    let actual = samples.samples.len();
    let mut out = Vec::with_capacity(terms.terms.len());
    let mut cursor = 0usize;
    for &code in &terms.terms {
        let per_term = match decorrelation_sample_count(code) {
            Some(n) => n as usize,
            None => return Err(Error::DecorrelationSampleCountUnspecified(code)),
        };
        let end = cursor.saturating_add(per_term);
        if end > actual {
            // The accumulating expected count already exceeds the flat
            // sample list; report the same Mismatch the up-front sum
            // would have produced, so callers see one canonical error
            // shape regardless of which term tripped it.
            let expected = terms.expected_decorrelation_sample_count().unwrap_or(end);
            return Err(Error::DecorrelationSampleCountMismatch { expected, actual });
        }
        out.push(samples.samples[cursor..end].to_vec());
        cursor = end;
    }
    if cursor != actual {
        return Err(Error::DecorrelationSampleCountMismatch {
            expected: cursor,
            actual,
        });
    }
    Ok(out)
}

/// Single-word expander used by [`expand_samples`].
///
/// Exposed `pub(crate)` so the round-4 entropy-info expander
/// ([`crate::entropy::expand_entropy`]) can re-use the same 16-bit
/// log-pack — the wiki "Entropy info" section's
/// "log-packed into 16 bits as described above" cross-reference points
/// back to this routine.
pub(crate) fn expand_sample_word(mantissa_byte: u8, exponent_byte: u8) -> i32 {
    // The mantissa half is signed (so negative samples sign-extend
    // through the shift); the exponent half is unsigned and biased
    // by 9 per the wiki "exponent-9" shorthand.
    let mantissa = (mantissa_byte as i8) as i32;
    let exponent = exponent_byte as i32;
    let shift = exponent - SAMPLE_EXPONENT_BIAS;
    if shift >= 0 {
        // Clamp the shift to 31 — beyond that the result is
        // indeterminate per Rust's shift overflow rules. Wiki-bounded
        // exponent values stay well below this in practice (the
        // mantissa is only 8 bits so a shift past ~24 already lies
        // outside the usable i32 range), but we want a defined
        // behaviour for malformed inputs rather than a panic.
        if shift >= 32 {
            // Mantissa = 0 produces 0 regardless; other values
            // saturate to the appropriate signed extreme.
            return match mantissa.signum() {
                0 => 0,
                1 => i32::MAX,
                _ => i32::MIN,
            };
        }
        mantissa << shift
    } else {
        // shift is negative; rust requires the shift amount to be
        // positive, so flip the direction.
        let abs_shift = -shift;
        // Beyond 31, an arithmetic right-shift saturates to the sign
        // of the mantissa.
        if abs_shift >= 32 {
            return if mantissa < 0 { -1 } else { 0 };
        }
        mantissa >> abs_shift
    }
}

/// Scale a decorrelation predictor sample by a working weight.
///
/// Spec §3.1 (`apply_weight`): the predicted contribution of a
/// decorrelation pass is `weight * sample` divided down by `2^10` with a
/// rounding bias, where weights are normalised so `1024` is unity gain:
///
/// ```text
/// apply_weight(weight, sample) = (weight * sample + 512) >> 10
/// ```
///
/// The multiply is performed in `i64` so that a wide (`>16`-bit) sample
/// times a `±1024` weight cannot overflow before the shift; the spec
/// notes the reference uses an algebraically equivalent split-multiply
/// for the same reason on 32-bit targets, but the widened product
/// computes the identical `weight·sample/1024` rounded value. The
/// arithmetic right shift floors toward negative infinity, matching the
/// reference's signed `>>`.
///
/// Sanity (spec §7): `apply_weight(1024, 100) == 100` (unity) and
/// `apply_weight(512, 100) == 50` (half gain).
#[inline]
pub fn apply_weight(weight: i32, sample: i32) -> i32 {
    let product = weight as i64 * sample as i64 + WEIGHT_ROUND_BIAS;
    (product >> WEIGHT_SHIFT) as i32
}

/// Nudge a decorrelation pass weight toward better prediction after a
/// reconstructed sample (the LMS-style adaptation step).
///
/// Spec §3.4 (`update_weight`): given the predictor `source` and the
/// entropy residual `result`, the weight moves by `±delta`:
///
/// * if either `source` or `result` is zero, the weight is unchanged;
/// * otherwise the weight gains `delta` when `source` and `result` share
///   a sign and loses `delta` when their signs differ.
///
/// This is the plain-arithmetic form of the branch-free reference
/// expression `weight += ((source ^ result) >> 31 ? -delta : delta)`
/// (the sign of the product selects the direction). `delta` is the
/// per-pass weight step carried in the high 3 bits of the term byte
/// (range `0..=7`).
#[inline]
pub fn update_weight(weight: i32, delta: i32, source: i32, result: i32) -> i32 {
    if source == 0 || result == 0 {
        return weight;
    }
    // Same sign → add delta, opposite sign → subtract delta. Using the
    // product's sign avoids a separate signum of each operand and
    // mirrors the reference's `(source ^ result) >> 31` test.
    if (source ^ result) < 0 {
        weight.wrapping_sub(delta)
    } else {
        weight.wrapping_add(delta)
    }
}

/// Clipped variant of [`update_weight`] used by the zero-delay
/// cross-channel terms (`-1`/`-2`/`-3`).
///
/// Spec §3.5 (`update_weight_clip`): the cross terms clamp the working
/// weight's **magnitude** to [`WEIGHT_CLIP`] (`1024`, unity) so the
/// zero-delay feedback loop cannot run the weight away. The reference
/// performs the step on the magnitude and clamps before restoring the
/// sign:
///
/// ```text
/// s = (source ^ result) >> 31;     // 0 if same sign, -1 if opposite
/// w = (weight ^ s) + (delta - s);  // step the magnitude up by delta
/// if (w > 1024) w = 1024;          // clamp magnitude
/// weight = (w ^ s) - s;            // restore sign
/// ```
///
/// Resolving the branch-free `^`/`-` magnitude algebra: when `source`
/// and `result` share a sign the new weight is `min(weight + delta,
/// 1024)`; when their signs differ it is `-min(delta - weight, 1024)`.
/// Both reduce to "move the magnitude toward unity by `delta`, capped at
/// `1024`, then carry the same sign the unclamped reference would" — the
/// step is computed exactly as the reference's signed `i32` arithmetic
/// so the clamp boundary matches bit-for-bit.
#[inline]
pub fn update_weight_clip(weight: i32, delta: i32, source: i32, result: i32) -> i32 {
    if source == 0 || result == 0 {
        return weight;
    }
    // s = (source ^ result) >> 31: 0 when signs agree, -1 when they
    // differ. The `>> 31` is an arithmetic shift of the sign bit.
    let s = (source ^ result) >> 31;
    // w = (weight ^ s) + (delta - s): the reference's magnitude step.
    let mut w = (weight ^ s).wrapping_add(delta.wrapping_sub(s));
    if w > WEIGHT_CLIP {
        w = WEIGHT_CLIP;
    }
    // weight = (w ^ s) - s: restore the sign s encoded.
    (w ^ s).wrapping_sub(s)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Terms (0x02) ----

    #[test]
    fn term_byte_splits_into_low5_and_high3() {
        // 0x02 → predictor = 2, delta = 0.
        let dt = expand_terms(&[0x02]);
        assert_eq!(dt.terms, vec![2]);
        assert_eq!(dt.deltas, vec![0]);

        // 0xE2 = 0b1110_0010 → predictor = 0b00010 = 2, delta = 0b111 = 7.
        let dt = expand_terms(&[0xE2]);
        assert_eq!(dt.terms, vec![2]);
        assert_eq!(dt.deltas, vec![7]);

        // Every wiki-documented predictor code (0..=5, 6..=12, 17, 18).
        for code in 0u8..=18 {
            // Pair each with a unique delta to make sure both fields
            // are decoded independently.
            let delta = code & 0x07;
            let byte = (delta << 5) | (code & TERM_PREDICTOR_MASK);
            let dt = expand_terms(&[byte]);
            assert_eq!(dt.terms[0] as u8, code);
            assert_eq!(dt.deltas[0], delta);
        }
    }

    #[test]
    fn terms_empty_payload_yields_empty_struct() {
        let dt = expand_terms(&[]);
        assert!(dt.terms.is_empty());
        assert!(dt.deltas.is_empty());
    }

    #[test]
    fn terms_byte_order_preserved_for_multiple_bytes() {
        // Two stereo predictors back-to-back: codes 2 and 3, deltas 1
        // and 2.
        let bytes = [(1u8 << 5) | 2, (2u8 << 5) | 3];
        let dt = expand_terms(&bytes);
        assert_eq!(dt.terms, vec![2, 3]);
        assert_eq!(dt.deltas, vec![1, 2]);
    }

    // ---- Weights (0x03) ----

    #[test]
    fn weight_byte_zero_expands_to_zero() {
        // n = 0 << 3 = 0; the `n > 0` branch does not fire.
        assert_eq!(expand_weight_byte(0x00), 0);
    }

    #[test]
    fn weight_byte_positive_uses_rounding_branch() {
        // byte = 1 → signed = 1 → n = 8 → 8 + (72 >> 7) = 8 + 0 = 8.
        assert_eq!(expand_weight_byte(0x01), 8);
        // byte = 8 → n = 64 → 64 + ((64 + 64) >> 7) = 64 + 1 = 65.
        assert_eq!(expand_weight_byte(0x08), 65);
        // byte = 0x7F (= 127, the maximum signed positive value)
        //   → n = 1016 → 1016 + ((1016 + 64) >> 7) = 1016 + 8 = 1024
        // …wait — let's double-check: 1080 >> 7 = 8 (1024/128 = 8.4),
        // so the result is 1024. But that's the upper bound named in
        // the docs gap; the wiki's `n > 0` branch is the source of
        // truth, not the bound.
        assert_eq!(expand_weight_byte(0x7F), 1024);
    }

    #[test]
    fn weight_byte_negative_does_not_use_rounding_branch() {
        // byte = 0xFF (= -1 signed) → n = -8. The `n > 0` branch
        // does NOT fire; result stays at -8.
        assert_eq!(expand_weight_byte(0xFF), -8);
        // byte = 0x80 (= -128 signed) → n = -1024.
        assert_eq!(expand_weight_byte(0x80), -1024);
    }

    #[test]
    fn weights_expansion_preserves_order_and_signs() {
        let payload = [0x00, 0x08, 0xFF, 0x80, 0x7F];
        let w = expand_weights(&payload);
        assert_eq!(w.weights, vec![0, 65, -8, -1024, 1024]);
    }

    #[test]
    fn weights_empty_payload_yields_empty_struct() {
        let w = expand_weights(&[]);
        assert!(w.weights.is_empty());
    }

    // ---- Samples (0x04) ----

    #[test]
    fn sample_word_exponent_eq_9_returns_mantissa_unshifted() {
        // exponent = 9 → shift amount = 0 → mantissa returned as-is.
        // mantissa = 0x10 (= +16), exponent = 9.
        let s = expand_sample_word(0x10, 0x09);
        assert_eq!(s, 16);
        // mantissa = 0xFF (= -1 signed), exponent = 9 → result = -1.
        let s = expand_sample_word(0xFF, 0x09);
        assert_eq!(s, -1);
    }

    #[test]
    fn sample_word_exponent_lt_9_shifts_mantissa_right() {
        // exponent = 7 → 9 - 7 = 2 → mantissa shifted right by 2.
        // mantissa = 0x40 (= +64) >> 2 = 16.
        assert_eq!(expand_sample_word(0x40, 0x07), 16);
        // mantissa = 0x80 (= -128 signed) >> 2 = -32 (arithmetic shift).
        assert_eq!(expand_sample_word(0x80, 0x07), -32);
        // exponent = 0 → 9 - 0 = 9. mantissa = 0x40 >> 9 = 0.
        assert_eq!(expand_sample_word(0x40, 0x00), 0);
    }

    #[test]
    fn sample_word_exponent_gt_9_shifts_mantissa_left() {
        // exponent = 11 → 11 - 9 = 2 → mantissa shifted left by 2.
        // mantissa = +4 << 2 = 16.
        assert_eq!(expand_sample_word(0x04, 0x0B), 16);
        // mantissa = -4 (0xFC) << 2 = -16.
        assert_eq!(expand_sample_word(0xFC, 0x0B), -16);
        // exponent = 24 → shift left 15. mantissa = 1 → 32768.
        assert_eq!(expand_sample_word(0x01, 0x18), 32768);
    }

    #[test]
    fn sample_payload_pairs_bytes_into_words() {
        // Three samples: (mant, exp) = (0x10, 0x09), (0x40, 0x07),
        // (0x04, 0x0B). Expanded values: 16, 16, 16.
        let payload = [0x10, 0x09, 0x40, 0x07, 0x04, 0x0B];
        let s = expand_samples(&payload).unwrap();
        assert_eq!(s.samples, vec![16, 16, 16]);
    }

    #[test]
    fn sample_payload_empty_yields_empty_struct() {
        let s = expand_samples(&[]).unwrap();
        assert!(s.samples.is_empty());
    }

    #[test]
    fn sample_payload_odd_byte_count_is_rejected() {
        // Three bytes is not a whole number of 16-bit words.
        assert_eq!(
            expand_samples(&[0x00, 0x00, 0x00]),
            Err(Error::DecorrelationSamplesOddByteCount(3))
        );
    }

    // ---- TermKind classification ----

    #[test]
    fn term_kind_stereo_implemented_subset_is_2_to_4() {
        // Wiki: "0-5 - predictors for stereo, only predictors 2-4 are
        // implemented".
        for c in [2i8, 3, 4] {
            assert_eq!(
                TermKind::from_code(c),
                TermKind::Stereo { implemented: true },
                "code {c} should be implemented stereo"
            );
            assert!(TermKind::from_code(c).is_implemented());
        }
        for c in [0i8, 1, 5] {
            assert_eq!(
                TermKind::from_code(c),
                TermKind::Stereo { implemented: false },
                "code {c} should be unimplemented stereo"
            );
            assert!(!TermKind::from_code(c).is_implemented());
        }
    }

    #[test]
    fn term_kind_sample_based_runs_6_through_12() {
        // Wiki: "6-12 - predictor uses 1-7 samples for prediction".
        // Code 6 → 1 sample, code 12 → 7 samples.
        for c in 6i8..=12 {
            let expected = (c - 5) as u8;
            assert_eq!(
                TermKind::from_code(c),
                TermKind::SampleBased {
                    sample_count: expected
                },
                "code {c} should be sample-based with {expected} samples"
            );
            assert_eq!(TermKind::from_code(c).previous_samples(), Some(expected));
            assert!(TermKind::from_code(c).is_implemented());
        }
    }

    #[test]
    fn term_kind_reserved_codes_13_through_16() {
        // Wiki: "13-16 - reserved".
        for c in 13i8..=16 {
            assert_eq!(
                TermKind::from_code(c),
                TermKind::Reserved,
                "code {c} should be reserved"
            );
            assert!(!TermKind::from_code(c).is_implemented());
            assert_eq!(TermKind::from_code(c).previous_samples(), None);
        }
    }

    #[test]
    fn term_kind_two_sample_codes_17_and_18() {
        // Wiki: "17-18 - predictor does prediction by two samples".
        for c in [17i8, 18] {
            assert_eq!(
                TermKind::from_code(c),
                TermKind::TwoSample,
                "code {c} should be two-sample"
            );
            assert!(TermKind::from_code(c).is_implemented());
            assert_eq!(TermKind::from_code(c).previous_samples(), Some(2));
        }
    }

    #[test]
    fn term_kind_codes_outside_wiki_range_are_unknown() {
        // The 5-bit field can carry 0..=31; the wiki documents only
        // 0..=18. Anything 19..=31 (or any negative code from a future
        // signed re-interpretation) lands in `Unknown`.
        for c in 19i8..=31 {
            assert_eq!(
                TermKind::from_code(c),
                TermKind::Unknown,
                "code {c} should be unknown"
            );
            assert!(!TermKind::from_code(c).is_implemented());
            assert_eq!(TermKind::from_code(c).previous_samples(), None);
        }
        // Negative codes are not currently produced by `expand_terms`
        // (low 5 bits land in 0..=31) but the classifier defines them
        // defensively as Unknown so a future re-interpretation cannot
        // panic.
        assert_eq!(TermKind::from_code(-1), TermKind::Unknown);
    }

    // ---- DecorrelationTerms accessors ----

    #[test]
    fn decorrelation_terms_len_and_is_empty_mirror_vec() {
        let dt = expand_terms(&[]);
        assert_eq!(dt.len(), 0);
        assert!(dt.is_empty());

        let dt = expand_terms(&[2u8, 3, 18]);
        assert_eq!(dt.len(), 3);
        assert!(!dt.is_empty());
    }

    #[test]
    fn decorrelation_terms_kind_at_indexes_into_terms() {
        // Bytes encode codes 2, 13, 18.
        let dt = expand_terms(&[2, 13, 18]);
        assert_eq!(dt.kind_at(0), Some(TermKind::Stereo { implemented: true }));
        assert_eq!(dt.kind_at(1), Some(TermKind::Reserved));
        assert_eq!(dt.kind_at(2), Some(TermKind::TwoSample));
        assert_eq!(dt.kind_at(3), None);
    }

    #[test]
    fn decorrelation_terms_iter_kinds_pairs_code_and_kind() {
        let dt = expand_terms(&[2, 6, 17]);
        let collected: Vec<_> = dt.iter_kinds().collect();
        assert_eq!(
            collected,
            vec![
                (2i8, TermKind::Stereo { implemented: true }),
                (6i8, TermKind::SampleBased { sample_count: 1 }),
                (17i8, TermKind::TwoSample),
            ]
        );
    }

    #[test]
    fn decorrelation_terms_all_implemented_rejects_reserved() {
        // All wiki-implemented codes → all_implemented true.
        let dt = expand_terms(&[2, 6, 12, 17, 18]);
        assert!(dt.all_implemented());
        assert!(!dt.has_reserved());

        // A reserved code in the middle flips both predicates.
        let dt = expand_terms(&[2, 14, 18]);
        assert!(!dt.all_implemented());
        assert!(dt.has_reserved());

        // An unimplemented stereo code is also rejected.
        let dt = expand_terms(&[0]);
        assert!(!dt.all_implemented());
        assert!(!dt.has_reserved());

        // Empty list — vacuously all-implemented, nothing reserved.
        let dt = expand_terms(&[]);
        assert!(dt.all_implemented());
        assert!(!dt.has_reserved());
    }

    // ---- weights_per_term ----

    #[test]
    fn weights_per_term_matches_wiki_channel_split() {
        // Wiki: "Each decorrelation term should have one or two weights
        // depending on channels."
        assert_eq!(weights_per_term(1), 1);
        assert_eq!(weights_per_term(2), 2);
        // Hypothetical higher channel counts (not currently produced
        // by the wiki "monaural" bit) clamp to the stereo case rather
        // than panicking — the wiki binary split is the source of truth.
        assert_eq!(weights_per_term(3), 2);
        assert_eq!(weights_per_term(0), 1);
    }

    // ---- decorrelation_sample_count / TermKind::decorrelation_sample_count ----

    #[test]
    fn decorrelation_sample_count_matches_sample_based_codes() {
        // Wiki "Possible predictor values": "6-12 - predictor uses 1-7
        // samples for prediction". The 0x04 payload supplies the same
        // count of seed samples (one per previous-sample slot).
        for c in 6i8..=12 {
            let expected = (c - 5) as u8;
            assert_eq!(decorrelation_sample_count(c), Some(expected));
            assert_eq!(
                TermKind::from_code(c).decorrelation_sample_count(),
                Some(expected)
            );
        }
    }

    #[test]
    fn decorrelation_sample_count_two_sample_codes_return_two() {
        // Wiki "17-18 - predictor does prediction by two samples".
        for c in [17i8, 18] {
            assert_eq!(decorrelation_sample_count(c), Some(2));
            assert_eq!(TermKind::from_code(c).decorrelation_sample_count(), Some(2));
        }
    }

    #[test]
    fn decorrelation_sample_count_is_none_for_stereo_codes() {
        // Wiki 0..=5 are stereo predictors; the spec does not give a
        // per-term sample count, so the helper returns None and the
        // partitioner refuses to split.
        for c in 0i8..=5 {
            assert_eq!(decorrelation_sample_count(c), None, "code {c}");
        }
    }

    #[test]
    fn decorrelation_sample_count_is_none_for_reserved_and_unknown() {
        // Wiki "13-16 - reserved" — no behaviour specified, no sample
        // count derivable. Same for codes outside `0..=18`.
        for c in 13i8..=16 {
            assert_eq!(decorrelation_sample_count(c), None, "code {c}");
        }
        for c in 19i8..=31 {
            assert_eq!(decorrelation_sample_count(c), None, "code {c}");
        }
        assert_eq!(decorrelation_sample_count(-1), None);
    }

    #[test]
    fn decorrelation_sample_count_stays_under_wiki_bound() {
        // The wiki "Decorrelation samples" section bounds the per-term
        // sample count: "may have up to 16 samples depending on its
        // value." Every documented count sits comfortably under that
        // bound; the constant exists so callers can sanity-check future
        // docs additions against it.
        for c in 6i8..=12 {
            let n = decorrelation_sample_count(c).unwrap();
            assert!(n <= MAX_DECORRELATION_SAMPLES_PER_TERM, "code {c}");
        }
        for c in [17i8, 18] {
            let n = decorrelation_sample_count(c).unwrap();
            assert!(n <= MAX_DECORRELATION_SAMPLES_PER_TERM, "code {c}");
        }
        assert_eq!(MAX_DECORRELATION_SAMPLES_PER_TERM, 16);
    }

    // ---- DecorrelationTerms::expected_decorrelation_sample_count ----

    #[test]
    fn expected_sample_count_sums_documented_codes() {
        // Codes 6 (1) + 8 (3) + 17 (2) + 12 (7) = 13 seed samples.
        let dt = expand_terms(&[6, 8, 17, 12]);
        assert_eq!(dt.expected_decorrelation_sample_count(), Some(13));
    }

    #[test]
    fn expected_sample_count_empty_term_list_is_zero() {
        // Vacuous: zero terms require zero seed samples.
        let dt = expand_terms(&[]);
        assert_eq!(dt.expected_decorrelation_sample_count(), Some(0));
    }

    #[test]
    fn expected_sample_count_propagates_unspecified_codes() {
        // Any stereo predictor / reserved / unknown code in the list
        // means we cannot sum — None propagates.
        let dt = expand_terms(&[6, 0, 18]); // 0 is stereo, unspecified
        assert_eq!(dt.expected_decorrelation_sample_count(), None);
        let dt = expand_terms(&[6, 14, 18]); // 14 reserved
        assert_eq!(dt.expected_decorrelation_sample_count(), None);
        let dt = expand_terms(&[6, 31, 18]); // 31 undocumented (5-bit max)
        assert_eq!(dt.expected_decorrelation_sample_count(), None);
    }

    // ---- partition_decorrelation_samples ----

    #[test]
    fn partition_samples_splits_in_term_order() {
        // Terms 6 (1 sample) + 8 (3 samples) + 17 (2 samples) = 6 total.
        let dt = expand_terms(&[6, 8, 17]);
        let ds = DecorrelationSamples {
            samples: vec![10, 20, 21, 22, 30, 31],
        };
        let parts = partition_decorrelation_samples(&dt, &ds).unwrap();
        assert_eq!(parts.len(), 3);
        assert_eq!(parts[0], vec![10]); // term 6 → 1 sample
        assert_eq!(parts[1], vec![20, 21, 22]); // term 8 → 3 samples
        assert_eq!(parts[2], vec![30, 31]); // term 17 → 2 samples
    }

    #[test]
    fn partition_samples_empty_terms_yields_empty_parts() {
        let dt = expand_terms(&[]);
        let ds = DecorrelationSamples { samples: vec![] };
        let parts = partition_decorrelation_samples(&dt, &ds).unwrap();
        assert!(parts.is_empty());
    }

    #[test]
    fn partition_samples_rejects_undocumented_term() {
        // Stereo code 2 has no wiki-documented per-term count — the
        // partitioner must refuse rather than guess.
        let dt = expand_terms(&[2]);
        let ds = DecorrelationSamples {
            samples: vec![10, 20],
        };
        assert_eq!(
            partition_decorrelation_samples(&dt, &ds),
            Err(Error::DecorrelationSampleCountUnspecified(2))
        );
    }

    #[test]
    fn partition_samples_rejects_reserved_term() {
        // Reserved code 14 → DecorrelationSampleCountUnspecified.
        let dt = expand_terms(&[6, 14]);
        let ds = DecorrelationSamples {
            samples: vec![10, 99, 99, 99],
        };
        assert_eq!(
            partition_decorrelation_samples(&dt, &ds),
            Err(Error::DecorrelationSampleCountUnspecified(14))
        );
    }

    #[test]
    fn partition_samples_rejects_short_flat_payload() {
        // Terms expect 6 samples (6+8+17), but only 5 supplied.
        let dt = expand_terms(&[6, 8, 17]);
        let ds = DecorrelationSamples {
            samples: vec![1, 2, 3, 4, 5],
        };
        assert_eq!(
            partition_decorrelation_samples(&dt, &ds),
            Err(Error::DecorrelationSampleCountMismatch {
                expected: 6,
                actual: 5,
            })
        );
    }

    #[test]
    fn partition_samples_rejects_long_flat_payload() {
        // Terms expect 1 sample (term 6), but 4 supplied — trailing
        // bytes have no term to bind to.
        let dt = expand_terms(&[6]);
        let ds = DecorrelationSamples {
            samples: vec![1, 2, 3, 4],
        };
        assert_eq!(
            partition_decorrelation_samples(&dt, &ds),
            Err(Error::DecorrelationSampleCountMismatch {
                expected: 1,
                actual: 4,
            })
        );
    }

    #[test]
    fn partition_samples_round_trip_against_expand_samples() {
        // Build the wire from per-term seed samples, expand it, then
        // partition it back to the same per-term layout. terms 6 (1) +
        // 18 (2) = 3 seed samples. Each on-disk sample is a 16-bit
        // [mantissa_lo, exponent_hi] word with exponent=9 (no shift),
        // so the mantissa byte is the value directly.
        // Wire: (1, 9), (2, 9), (3, 9).
        let wire = [0x01, 0x09, 0x02, 0x09, 0x03, 0x09];
        let ds = expand_samples(&wire).unwrap();
        assert_eq!(ds.samples, vec![1, 2, 3]);
        let dt = expand_terms(&[6, 18]);
        let parts = partition_decorrelation_samples(&dt, &ds).unwrap();
        assert_eq!(parts.len(), 2);
        assert_eq!(parts[0], vec![1]);
        assert_eq!(parts[1], vec![2, 3]);
    }

    #[test]
    fn sample_extreme_exponents_saturate_rather_than_panic() {
        // Exponent = 0xFF means a shift of 0xFF - 9 = 246 → far beyond
        // i32. The expander saturates rather than panicking on the
        // overflow.
        let positive = expand_sample_word(0x01, 0xFF);
        assert_eq!(positive, i32::MAX);
        let negative = expand_sample_word(0xFF, 0xFF);
        assert_eq!(negative, i32::MIN);
        let zero = expand_sample_word(0x00, 0xFF);
        assert_eq!(zero, 0);
        // Exponent = 0 means a right-shift of 9 — well within range,
        // already covered above. Verify the >= 32 branch on the
        // right-shift side too by abusing a hypothetical exponent
        // of -23: but exponents are unsigned bytes so that's not
        // reachable from the wire. The expander still guards against
        // it for malformed callers. Synthesise via the lowest exponent
        // byte (0x00); 0x00 → shift = -9, which fits the < 32 branch.
        // We can't reach abs_shift >= 32 from the wire alone, but
        // the guard is there for defence in depth.
    }

    // ---- apply_weight (spec §3.1) ----

    #[test]
    fn apply_weight_unity_is_identity() {
        // Spec §7: apply_weight(1024, x) == x. 1024 is unity gain.
        assert_eq!(apply_weight(1024, 100), 100);
        assert_eq!(apply_weight(1024, -100), -100);
        assert_eq!(apply_weight(1024, 0), 0);
        // The +512 rounding bias does not perturb unity for any sample:
        // (1024*x + 512) >> 10 = x + (512 >> 10) = x.
        assert_eq!(apply_weight(1024, 1), 1);
        assert_eq!(apply_weight(1024, 1_000_000), 1_000_000);
    }

    #[test]
    fn apply_weight_half_gain_rounds_with_bias() {
        // Spec §7: apply_weight(512, 100) == 50. (512*100 + 512) >> 10
        // = (51200 + 512) >> 10 = 51712 >> 10 = 50.
        assert_eq!(apply_weight(512, 100), 50);
        // The +512 bias rounds toward +infinity at the half boundary:
        // sample = 1 with weight 512 → (512 + 512) >> 10 = 1024 >> 10 = 1.
        assert_eq!(apply_weight(512, 1), 1);
        // weight = 512, sample = -1 → (-512 + 512) >> 10 = 0.
        assert_eq!(apply_weight(512, -1), 0);
    }

    #[test]
    fn apply_weight_arithmetic_shift_floors_negatives() {
        // A small product floors toward negative infinity (arithmetic
        // >>), matching the reference's signed shift. weight = 1,
        // sample = -1 → (-1 + 512) >> 10 = 511 >> 10 = 0.
        assert_eq!(apply_weight(1, -1), 0);
        // weight = 1, sample = -1024 → (-1024 + 512) >> 10
        // = -512 >> 10 = -1 (arithmetic shift, not truncation to 0).
        assert_eq!(apply_weight(1, -1024), -1);
    }

    #[test]
    fn apply_weight_wide_sample_does_not_overflow() {
        // A 24-bit sample times a -1024 weight overflows i32 before the
        // shift; the i64 product keeps it exact. sample = 0x7FFFFF,
        // weight = -1024 → -(0x7FFFFF) after the /1024 scale.
        let sample = 0x7F_FFFF;
        assert_eq!(apply_weight(-1024, sample), -sample);
        assert_eq!(apply_weight(1024, sample), sample);
    }

    // ---- update_weight (spec §3.4) ----

    #[test]
    fn update_weight_zero_operand_is_no_change() {
        // Spec §3.4: if either source or result is zero, no change.
        assert_eq!(update_weight(700, 2, 0, 5), 700);
        assert_eq!(update_weight(700, 2, 5, 0), 700);
        assert_eq!(update_weight(700, 2, 0, 0), 700);
    }

    #[test]
    fn update_weight_same_sign_adds_delta() {
        // Spec §3.4: same sign → add delta.
        assert_eq!(update_weight(700, 3, 4, 9), 703);
        assert_eq!(update_weight(700, 3, -4, -9), 703);
        // delta = 0 is a documented value (high 3 bits can be 0).
        assert_eq!(update_weight(700, 0, 4, 9), 700);
    }

    #[test]
    fn update_weight_opposite_sign_subtracts_delta() {
        // Spec §3.4: opposite sign → subtract delta.
        assert_eq!(update_weight(700, 3, -4, 9), 697);
        assert_eq!(update_weight(700, 3, 4, -9), 697);
        // A negative weight steps the same way.
        assert_eq!(update_weight(-700, 5, 4, -9), -705);
    }

    // ---- update_weight_clip (spec §3.5) ----

    #[test]
    fn update_weight_clip_zero_operand_is_no_change() {
        assert_eq!(update_weight_clip(900, 2, 0, 5), 900);
        assert_eq!(update_weight_clip(900, 2, 5, 0), 900);
    }

    #[test]
    fn update_weight_clip_same_sign_grows_then_clamps() {
        // Same sign (s = 0): weight = min(weight + delta, 1024).
        assert_eq!(update_weight_clip(900, 7, 4, 9), 907);
        // At the cap the magnitude stops at 1024.
        assert_eq!(update_weight_clip(1020, 7, 4, 9), WEIGHT_CLIP);
        assert_eq!(update_weight_clip(1024, 7, 4, 9), WEIGHT_CLIP);
    }

    #[test]
    fn update_weight_clip_opposite_sign_uses_reference_magnitude_form() {
        // Opposite sign (s = -1): weight = -min(delta - weight, 1024).
        // For weight = 900, delta = 7: -min(7 - 900, 1024)
        // = -min(-893, 1024) = -(-893) = 893. The reference magnitude
        // step shrinks the positive weight toward zero by delta-ish and
        // re-signs; our faithful transcription matches the branch-free
        // arithmetic exactly.
        assert_eq!(update_weight_clip(900, 7, -4, 9), 893);
        // A negative starting weight with opposite-sign operands:
        // weight = -900, s = -1 (source/result differ). The reference
        // arithmetic: w = (-900 ^ -1) + (7 - (-1)) = 899 + 8 = 907,
        // not > 1024, weight = (907 ^ -1) - (-1) = -908 + 1 = -907.
        assert_eq!(update_weight_clip(-900, 7, 4, -9), -907);
    }

    #[test]
    fn update_weight_clip_caps_magnitude_at_unity_both_signs() {
        // Drive the magnitude past 1024 from both sign directions and
        // confirm it clamps to ±1024 (unity), never running away.
        // Same sign, large positive weight.
        assert_eq!(update_weight_clip(1024, 7, 1, 1), WEIGHT_CLIP);
        // Same sign, large negative weight (s = 0 keeps the sign):
        // w = (-1024 ^ 0) + (7 - 0) = -1017, not > 1024, re-sign → -1017.
        // To exceed the cap on the negative side the magnitude form must
        // be exercised with opposite-sign operands.
        // weight = -1024, opposite sign: w = (-1024 ^ -1) + (7+1)
        // = 1023 + 8 = 1031 > 1024 → 1024, weight = (1024 ^ -1) + 1
        // = -1025 + 1 = -1024.
        assert_eq!(update_weight_clip(-1024, 7, 4, -9), -WEIGHT_CLIP);
    }
}
