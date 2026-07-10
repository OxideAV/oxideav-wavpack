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
//!   weights are documented as signed throughout the WavPack spec
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
/// Shift pivot of the 16-bit log-word encoding (the wiki's "high 8
/// bits are exponent-9" shorthand): an integer log part of `9` leaves
/// the 9-bit mantissa unshifted. See [`crate::wp_exp2s`] (staged spec
/// `wavpack-log2-exp2.md` §4 step 3 / §7 "exp2 shift pivot").
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
/// Each pair of bytes is read as a little-endian **signed 16-bit log
/// word** and expanded through the [`crate::wp_exp2s`] log→value
/// conversion (staged spec `wavpack-log2-exp2.md` §5: "each stored
/// 16-bit field is little-endian; it is sign-extended … and passed to
/// `wp_exp2s`"). The all-zero word is the canonical exact zero (§6
/// erratum pin).
///
/// The wiki's linear "lower 8 bits are mantiss, high 8 bits are
/// exponent-9" shorthand approximates this encoding (same shift pivot
/// [`SAMPLE_EXPONENT_BIAS`]) but omits the fractional mantissa table
/// and the implicit `0x100` mantissa bit; black-box cross-validation
/// against reference-encoded files (r393, wvunpack as an opaque
/// binary) showed the linear reading diverges for every non-zero word,
/// so the staged log-domain conversion is the authoritative one
/// (round 405).
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
/// at the same encoding.
///
/// The two wire bytes form a little-endian signed 16-bit log word,
/// expanded by [`crate::wp_exp2s`] (staged spec `wavpack-log2-exp2.md`
/// §4/§5). Round 405: this replaced the wiki's linear
/// mantissa/exponent shorthand, which diverges from reference-encoded
/// files for every non-zero word.
pub(crate) fn expand_sample_word(lo_byte: u8, hi_byte: u8) -> i32 {
    crate::logpack::expand_log_word(lo_byte, hi_byte)
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
/// notes the spec uses an algebraically equivalent split-multiply
/// for the same reason on 32-bit targets, but the widened product
/// computes the identical `weight·sample/1024` rounded value. The
/// arithmetic right shift floors toward negative infinity, matching the
/// spec.s signed `>>`.
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
/// This is the plain-arithmetic form of the branch-free canonical form
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
    // mirrors the spec's `(source ^ result) >> 31` test.
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
/// zero-delay feedback loop cannot run the weight away. The spec
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
/// `1024`, then carry the same sign the unclamped update would" — the
/// step is computed exactly as the spec's signed `i32` arithmetic
/// so the clamp boundary matches bit-for-bit.
#[inline]
pub fn update_weight_clip(weight: i32, delta: i32, source: i32, result: i32) -> i32 {
    if source == 0 || result == 0 {
        return weight;
    }
    // s = (source ^ result) >> 31: 0 when signs agree, -1 when they
    // differ. The `>> 31` is an arithmetic shift of the sign bit.
    let s = (source ^ result) >> 31;
    // w = (weight ^ s) + (delta - s): the spec's magnitude step.
    let mut w = (weight ^ s).wrapping_add(delta.wrapping_sub(s));
    if w > WEIGHT_CLIP {
        w = WEIGHT_CLIP;
    }
    // weight = (w ^ s) - s: restore the sign s encoded.
    (w ^ s).wrapping_sub(s)
}

/// `+5` bias the spec applies to the low-5-bit term field of a `0x02`
/// decorr-terms byte. Spec §2.1 / §6 ("term-byte bias +5"): the on-wire
/// term is recovered as `(byte & 0x1f) - 5`.
///
/// This is the encoding used by the clean-room decorrelation trace
/// `docs/audio/wavpack/spec/wavpack-decorrelation.md`, distinct from the
/// raw `byte & 0x1f` reading of the older wiki listing consumed by
/// [`expand_terms`]. The two coexist because they describe two different
/// documented sources; the prediction loop ([`decorrelate_mono`] /
/// [`decorrelate_stereo`]) speaks the spec encoding.
pub const TERM_BYTE_BIAS: i8 = 5;

/// Maximum single-tap fixed-lag term — also the per-channel history
/// ring size. Spec §6 (`MAX_TERM`): `8`.
pub const MAX_TERM: i8 = 8;

/// Maximum number of decorrelation passes per block. Spec §6
/// (`MAX_NTERMS`): `16`.
pub const MAX_NTERMS: usize = 16;

/// `true` when `term` is one of the valid decorrelation term values the
/// spec §2 enumerates: `{1..8, 17, 18, -1, -2, -3}`. A term of `0`, or
/// anything outside that set, is invalid (spec §2.1).
pub const fn is_valid_term(term: i8) -> bool {
    matches!(term, 1..=8 | 17 | 18 | -1 | -2 | -3)
}

/// Decode the term + delta carried in one `0x02` decorr-terms byte using
/// the **spec** encoding (`docs/audio/wavpack/spec/wavpack-decorrelation.md`
/// §2.1 / §6): the low 5 bits hold the term biased by `+5`
/// (`term = (byte & 0x1f) - 5`) and the high 3 bits hold the per-pass
/// `delta` (`(byte >> 5) & 0x7`).
///
/// This is distinct from [`expand_terms`], which reads the *unbiased*
/// `byte & 0x1f` of the older wiki listing. The two coexist because they
/// describe two different documented sources; [`DecorrPass`] /
/// [`decorrelate_mono`] / [`decorrelate_stereo`] consume the spec
/// encoding, so a caller assembling passes from `0x02` bytes should use
/// this function. Returns the `(term, delta)` pair; the term is not
/// validated here (use [`is_valid_term`] / [`DecorrPass::new`] to reject
/// the invalid set).
pub const fn decode_term_byte(byte: u8) -> (i8, i32) {
    let term = (byte & TERM_PREDICTOR_MASK) as i8 - TERM_BYTE_BIAS;
    let delta = ((byte >> TERM_PREDICTOR_BITS) & TERM_DELTA_MASK) as i32;
    (term, delta)
}

/// `true` when `term` is a cross-channel (negative) predictor
/// (`-1`/`-2`/`-3`), which is valid for stereo data only (spec §2.1 /
/// §3.3).
pub const fn is_cross_term(term: i8) -> bool {
    matches!(term, -3..=-1)
}

/// One decorrelation pass: the inverse-prediction state the loop carries
/// for a single `0x02` term across the whole sample buffer.
///
/// Spec §3 (`struct decorr_pass`): a pass owns its term, the per-pass
/// `delta` weight step, the working weight(s) (one per channel — stereo
/// passes carry two), and the per-channel history the predictor reads.
/// History is a fixed ring of [`MAX_TERM`] (`8`) slots per channel so the
/// `1..8` fixed-lag terms can index `(m + t) & 7`; the `17`/`18`
/// extrapolators and the cross terms use only the first two / first slot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecorrPass {
    /// The decorrelation term (`{1..8, 17, 18, -1, -2, -3}`).
    pub term: i8,
    /// The per-pass weight adaptation step (`delta`, range `0..=7`).
    pub delta: i32,
    /// Working weight for channel A (and the only weight for mono).
    pub weight_a: i32,
    /// Working weight for channel B (unused on mono passes).
    pub weight_b: i32,
    /// Channel-A history ring (`8` slots; `0` = empty).
    history_a: [i32; MAX_TERM as usize],
    /// Channel-B history ring (`8` slots; `0` = empty).
    history_b: [i32; MAX_TERM as usize],
}

impl DecorrPass {
    /// Build a pass from its term, delta and starting weight(s), seeding
    /// the per-channel history from the `0x04` decorr-samples expansion.
    ///
    /// Spec §3.6: terms `17`/`18` are primed with 2 seeds per channel,
    /// terms `1..8` with `term` seeds per channel, and cross terms with
    /// 1 seed per channel. The seeds arrive newest-first per the spec's
    /// history convention (`s[n-1]` first); they are placed so that the
    /// predictor's first read sees the correct lag. `seed_b` is ignored
    /// on a mono pass (pass it `&[]`).
    ///
    /// An **empty** `seed_a` builds an unprimed pass (all-zero history):
    /// the `0x04` payload primes only a wire-order prefix of a block's
    /// passes, and passes beyond that prefix start from zero history
    /// (round 405; see [`assemble_mono_passes`]). A cross term with an
    /// empty `seed_a` leaves both channels unprimed.
    ///
    /// Returns [`Error::InvalidDecorrelationTerm`] for a term outside the
    /// spec's valid set, and [`Error::DecorrelationSeedUnderflow`] when a
    /// channel supplies a non-empty but short seed slice for the term.
    pub fn new(
        term: i8,
        delta: i32,
        weight_a: i32,
        weight_b: i32,
        seed_a: &[i32],
        seed_b: &[i32],
    ) -> Result<Self> {
        if !is_valid_term(term) {
            return Err(Error::InvalidDecorrelationTerm(term));
        }
        let mut history_a = [0i32; MAX_TERM as usize];
        let mut history_b = [0i32; MAX_TERM as usize];
        if !seed_a.is_empty() {
            seed_history(term, seed_a, &mut history_a)?;
        }
        // A cross term seeds both channels; a per-channel term that the
        // caller is driving in stereo also seeds B. A mono caller passes
        // an empty slice and B stays zeroed.
        if !seed_b.is_empty() || (is_cross_term(term) && !seed_a.is_empty()) {
            seed_history(term, seed_b, &mut history_b)?;
        }
        Ok(DecorrPass {
            term,
            delta,
            weight_a,
            weight_b,
            history_a,
            history_b,
        })
    }
}

/// Number of seed-history samples per channel the spec §3.6 ties to a
/// term: `17`/`18` → 2, `1..8` → `term`, cross terms → 1.
const fn seed_count(term: i8) -> usize {
    match term {
        17 | 18 => 2,
        -3..=-1 => 1,
        t if t >= 1 && t <= MAX_TERM => t as usize,
        _ => 0,
    }
}

/// Place the `0x04` seed samples for `term` into a fresh history ring.
///
/// Seeds arrive **newest-first** (`s[-1]`, `s[-2]`, …), matching the
/// `0x04` decorr-samples wire order. The placement depends on how the
/// loop reads the ring:
///
/// * **Fixed lag `1..8`**: the loop reads slot `m` (starting at `m=0`)
///   and writes to `(m+t)&7`, so the first `t` reads consume slots
///   `0..t-1` *before* any written value returns. Those slots must hold
///   the seeds oldest-first — slot `0` = `s[-t]` (read first), …,
///   slot `t-1` = `s[-1]`. Newest-first seeds are therefore reversed
///   into slots `0..t-1`.
/// * **Extrapolators `17`/`18`**: the loop reads `history[0]` as `s[-1]`
///   and `history[1]` as `s[-2]`, so the newest-first seeds drop into
///   slots `0`/`1` unchanged.
/// * **Cross terms**: read `history[0]` only; the single seed lands in
///   slot `0`.
fn seed_history(term: i8, seed: &[i32], ring: &mut [i32; MAX_TERM as usize]) -> Result<()> {
    let needed = seed_count(term);
    if seed.len() < needed {
        return Err(Error::DecorrelationSeedUnderflow {
            term,
            supplied: seed.len(),
        });
    }
    if (1..=MAX_TERM).contains(&term) {
        // Fixed lag: reverse newest-first seeds into slots 0..t-1 so the
        // oldest seed (`s[-t]`) is read first.
        for (i, &s) in seed.iter().take(needed).enumerate() {
            ring[needed - 1 - i] = s;
        }
    } else {
        // 17/18 (slot0=s[-1], slot1=s[-2]) and cross (slot0) keep order.
        for (i, &s) in seed.iter().take(needed).enumerate() {
            ring[i] = s;
        }
    }
    Ok(())
}

/// Run the configured decorrelation passes over a **mono** residual
/// buffer in place, turning entropy-decoded residuals into PCM.
///
/// Spec §3.2 / §3.7: each pass is applied over the whole buffer before
/// the next begins, and the passes are supplied in **application order**
/// (front-to-back; the caller has already reversed the on-wire
/// `0x02`/`0x03`/`0x04` order per §3.7 so the first pass here undoes the
/// last pass the encoder applied). For each sample `n` and term `t`:
///
/// 1. form the predictor from history (`s[n-t]`, `2a0-a1`, or
///    `(3a0-a1)>>1`);
/// 2. `s[n] = apply_weight(weight, pred) + residual`;
/// 3. `weight = update_weight(weight, delta, pred, residual)`;
/// 4. push `s[n]` into the channel history.
///
/// Cross terms (`-1`/`-2`/`-3`) are rejected for mono with
/// [`Error::CrossTermOnMono`]; an over-long pass list trips
/// [`Error::TooManyDecorrelationPasses`].
pub fn decorrelate_mono(passes: &mut [DecorrPass], buffer: &mut [i32]) -> Result<()> {
    if passes.len() > MAX_NTERMS {
        return Err(Error::TooManyDecorrelationPasses(passes.len()));
    }
    for pass in passes.iter() {
        if is_cross_term(pass.term) {
            return Err(Error::CrossTermOnMono(pass.term));
        }
    }
    for pass in passes.iter_mut() {
        // `m` is the rotating ring index for the 1..8 fixed-lag terms.
        let mut m = 0usize;
        for slot in buffer.iter_mut() {
            let residual = *slot;
            let pred = match pass.term {
                17 => pass.history_a[0]
                    .wrapping_mul(2)
                    .wrapping_sub(pass.history_a[1]),
                18 => {
                    pass.history_a[0]
                        .wrapping_mul(3)
                        .wrapping_sub(pass.history_a[1])
                        >> 1
                }
                // Fixed lag: read the current ring slot `m`. A value
                // written here `t` iterations ago (to `(m'+t)&7`) is read
                // again when `m` catches up, giving `s[n-t]` (§3.2 step 1
                // / step 4 `(m+t)&7` write convention).
                _ => pass.history_a[m & (MAX_TERM as usize - 1)],
            };
            let sample = apply_weight(pass.weight_a, pred).wrapping_add(residual);
            pass.weight_a = update_weight(pass.weight_a, pass.delta, pred, residual);
            *slot = sample;
            // Push s[n] into history.
            if pass.term == 17 || pass.term == 18 {
                pass.history_a[1] = pass.history_a[0];
                pass.history_a[0] = sample;
            } else {
                let w = (m + pass.term as usize) & (MAX_TERM as usize - 1);
                pass.history_a[w] = sample;
                m = m.wrapping_add(1);
            }
        }
    }
    Ok(())
}

/// Run the configured decorrelation passes over an **interleaved stereo**
/// residual buffer (`[L0, R0, L1, R1, …]`) in place.
///
/// Spec §3.2 / §3.3 / §3.7. Per-channel terms (`1..8`/`17`/`18`) run the
/// §3.2 inverse step independently on A and B; the cross terms
/// (`-1`/`-2`/`-3`) run the §3.3 zero-delay cross step with the clipped
/// weight update ([`update_weight_clip`]). Passes are supplied in
/// application order (see [`decorrelate_mono`]).
///
/// Returns [`Error::TooManyDecorrelationPasses`] for an over-long pass
/// list; the buffer length must be even (one `R` per `L`) or the trailing
/// odd sample is left untouched.
pub fn decorrelate_stereo(passes: &mut [DecorrPass], buffer: &mut [i32]) -> Result<()> {
    if passes.len() > MAX_NTERMS {
        return Err(Error::TooManyDecorrelationPasses(passes.len()));
    }
    let pairs = buffer.len() / 2;
    for pass in passes.iter_mut() {
        let mut m = 0usize;
        for p in 0..pairs {
            let li = 2 * p;
            let ri = li + 1;
            let res_a = buffer[li];
            let res_b = buffer[ri];
            match pass.term {
                // Cross terms: zero-delay, clipped weight update (§3.3).
                -1 => {
                    // A from stored history, then B from the just-built A.
                    let pred_a = pass.history_a[0];
                    let a = apply_weight(pass.weight_a, pred_a).wrapping_add(res_a);
                    pass.weight_a = update_weight_clip(pass.weight_a, pass.delta, pred_a, res_a);
                    let b = apply_weight(pass.weight_b, a).wrapping_add(res_b);
                    pass.weight_b = update_weight_clip(pass.weight_b, pass.delta, a, res_b);
                    buffer[li] = a;
                    buffer[ri] = b;
                    pass.history_a[0] = b;
                }
                -2 => {
                    // B from stored history, then A from the just-built B.
                    let pred_b = pass.history_b[0];
                    let b = apply_weight(pass.weight_b, pred_b).wrapping_add(res_b);
                    pass.weight_b = update_weight_clip(pass.weight_b, pass.delta, pred_b, res_b);
                    let a = apply_weight(pass.weight_a, b).wrapping_add(res_a);
                    pass.weight_a = update_weight_clip(pass.weight_a, pass.delta, b, res_a);
                    buffer[li] = a;
                    buffer[ri] = b;
                    pass.history_b[0] = a;
                }
                -3 => {
                    // Each from the other's stored previous value, then
                    // swap the stored history.
                    let pred_a = pass.history_b[0];
                    let pred_b = pass.history_a[0];
                    let a = apply_weight(pass.weight_a, pred_a).wrapping_add(res_a);
                    pass.weight_a = update_weight_clip(pass.weight_a, pass.delta, pred_a, res_a);
                    let b = apply_weight(pass.weight_b, pred_b).wrapping_add(res_b);
                    pass.weight_b = update_weight_clip(pass.weight_b, pass.delta, pred_b, res_b);
                    buffer[li] = a;
                    buffer[ri] = b;
                    pass.history_a[0] = a;
                    pass.history_b[0] = b;
                }
                // Per-channel two-sample extrapolators (§3.2).
                17 => {
                    let pa = pass.history_a[0]
                        .wrapping_mul(2)
                        .wrapping_sub(pass.history_a[1]);
                    let pb = pass.history_b[0]
                        .wrapping_mul(2)
                        .wrapping_sub(pass.history_b[1]);
                    let a = apply_weight(pass.weight_a, pa).wrapping_add(res_a);
                    let b = apply_weight(pass.weight_b, pb).wrapping_add(res_b);
                    pass.weight_a = update_weight(pass.weight_a, pass.delta, pa, res_a);
                    pass.weight_b = update_weight(pass.weight_b, pass.delta, pb, res_b);
                    buffer[li] = a;
                    buffer[ri] = b;
                    pass.history_a[1] = pass.history_a[0];
                    pass.history_a[0] = a;
                    pass.history_b[1] = pass.history_b[0];
                    pass.history_b[0] = b;
                }
                18 => {
                    let pa = pass.history_a[0]
                        .wrapping_mul(3)
                        .wrapping_sub(pass.history_a[1])
                        >> 1;
                    let pb = pass.history_b[0]
                        .wrapping_mul(3)
                        .wrapping_sub(pass.history_b[1])
                        >> 1;
                    let a = apply_weight(pass.weight_a, pa).wrapping_add(res_a);
                    let b = apply_weight(pass.weight_b, pb).wrapping_add(res_b);
                    pass.weight_a = update_weight(pass.weight_a, pass.delta, pa, res_a);
                    pass.weight_b = update_weight(pass.weight_b, pass.delta, pb, res_b);
                    buffer[li] = a;
                    buffer[ri] = b;
                    pass.history_a[1] = pass.history_a[0];
                    pass.history_a[0] = a;
                    pass.history_b[1] = pass.history_b[0];
                    pass.history_b[0] = b;
                }
                // Fixed-lag terms 1..8 (§3.2), per channel.
                t => {
                    let rd = m & (MAX_TERM as usize - 1);
                    let pa = pass.history_a[rd];
                    let pb = pass.history_b[rd];
                    let a = apply_weight(pass.weight_a, pa).wrapping_add(res_a);
                    let b = apply_weight(pass.weight_b, pb).wrapping_add(res_b);
                    pass.weight_a = update_weight(pass.weight_a, pass.delta, pa, res_a);
                    pass.weight_b = update_weight(pass.weight_b, pass.delta, pb, res_b);
                    buffer[li] = a;
                    buffer[ri] = b;
                    let w = (m + t as usize) & (MAX_TERM as usize - 1);
                    pass.history_a[w] = a;
                    pass.history_b[w] = b;
                    m = m.wrapping_add(1);
                }
            }
        }
    }
    Ok(())
}

/// Run the configured decorrelation passes **forward** (encode side) over a
/// **mono** PCM buffer in place, turning PCM samples into entropy-ready
/// residuals — the exact arithmetic inverse of [`decorrelate_mono`].
///
/// Spec §3.2 / §3.7, inverted. [`decorrelate_mono`] consumes the passes in
/// *application order* (front-to-back) and, for each sample, reconstructs
/// `s[n] = apply_weight(weight, pred) + residual`. The encoder forms the
/// same predictor from the same history and emits
/// `residual = s[n] - apply_weight(weight, pred)`, pushing the original
/// PCM sample `s[n]` (not the residual) into history exactly as the
/// decoder pushes its reconstructed `s[n]`. The weight is then nudged by
/// the identical [`update_weight`] step on `(pred, residual)`, so the two
/// directions evolve byte-identical pass state.
///
/// ## Pass order (§3.7)
///
/// The decoder undoes the encoder's passes in reverse: the *first* pass in
/// the application-ordered list is the *last* pass the encoder applied. So
/// this forward routine walks the same list **back-to-front** (last slot
/// first), and a `decorrelate_mono` over the application-ordered list
/// reproduces the original PCM. Callers therefore pass the *same*
/// application-ordered `DecorrPass` list to both directions; no manual
/// reversal is needed.
///
/// Each pass's `weight_a` / `history_a` mutate in place across the buffer
/// (the running encode state), so a caller wanting to re-run a pass must
/// rebuild it from its seeds (e.g. via [`DecorrPass::new`]). Cross terms
/// (`-1`/`-2`/`-3`) are rejected for mono with [`Error::CrossTermOnMono`];
/// an over-long pass list trips [`Error::TooManyDecorrelationPasses`].
pub fn recorrelate_mono(passes: &mut [DecorrPass], buffer: &mut [i32]) -> Result<()> {
    if passes.len() > MAX_NTERMS {
        return Err(Error::TooManyDecorrelationPasses(passes.len()));
    }
    for pass in passes.iter() {
        if is_cross_term(pass.term) {
            return Err(Error::CrossTermOnMono(pass.term));
        }
    }
    // Encode applies passes in the reverse of the decoder's application
    // order: the decoder undoes the encoder's last pass first.
    for pass in passes.iter_mut().rev() {
        let mut m = 0usize;
        for slot in buffer.iter_mut() {
            let sample = *slot;
            let pred = match pass.term {
                17 => pass.history_a[0]
                    .wrapping_mul(2)
                    .wrapping_sub(pass.history_a[1]),
                18 => {
                    pass.history_a[0]
                        .wrapping_mul(3)
                        .wrapping_sub(pass.history_a[1])
                        >> 1
                }
                _ => pass.history_a[m & (MAX_TERM as usize - 1)],
            };
            // Inverse of `s[n] = apply_weight(w, pred) + residual`.
            let residual = sample.wrapping_sub(apply_weight(pass.weight_a, pred));
            pass.weight_a = update_weight(pass.weight_a, pass.delta, pred, residual);
            *slot = residual;
            // Push the ORIGINAL PCM sample into history — the decoder
            // pushes its reconstructed `s[n]`, which equals this sample.
            if pass.term == 17 || pass.term == 18 {
                pass.history_a[1] = pass.history_a[0];
                pass.history_a[0] = sample;
            } else {
                let w = (m + pass.term as usize) & (MAX_TERM as usize - 1);
                pass.history_a[w] = sample;
                m = m.wrapping_add(1);
            }
        }
    }
    Ok(())
}

/// Run the configured decorrelation passes **forward** (encode side) over
/// an **interleaved stereo** PCM buffer (`[L0, R0, L1, R1, …]`) in place —
/// the exact arithmetic inverse of [`decorrelate_stereo`].
///
/// Spec §3.2 / §3.3 / §3.7, inverted. Per-channel terms (`1..8`/`17`/`18`)
/// emit `residual = sample - apply_weight(weight, pred)` independently on
/// A and B; the cross terms (`-1`/`-2`/`-3`) invert the §3.3 zero-delay
/// step with the clipped weight update ([`update_weight_clip`]). For each
/// cross term the encoder forms the predictor from the *original* PCM of
/// the partner channel (the value the decoder reconstructs), matching the
/// decoder's "predict from the just-reconstructed channel" arithmetic.
///
/// As with [`recorrelate_mono`], the passes are supplied in *application
/// order* (the same list [`decorrelate_stereo`] consumes) and walked
/// **back-to-front**, so a subsequent `decorrelate_stereo` over the
/// application-ordered list reproduces the original interleaved PCM. The
/// buffer length must be even (one `R` per `L`); a trailing odd sample is
/// left untouched. Over-long pass lists trip
/// [`Error::TooManyDecorrelationPasses`].
pub fn recorrelate_stereo(passes: &mut [DecorrPass], buffer: &mut [i32]) -> Result<()> {
    if passes.len() > MAX_NTERMS {
        return Err(Error::TooManyDecorrelationPasses(passes.len()));
    }
    let pairs = buffer.len() / 2;
    for pass in passes.iter_mut().rev() {
        let mut m = 0usize;
        for p in 0..pairs {
            let li = 2 * p;
            let ri = li + 1;
            let sa = buffer[li];
            let sb = buffer[ri];
            match pass.term {
                -1 => {
                    let pred_a = pass.history_a[0];
                    let res_a = sa.wrapping_sub(apply_weight(pass.weight_a, pred_a));
                    pass.weight_a = update_weight_clip(pass.weight_a, pass.delta, pred_a, res_a);
                    // B predicted from the original A (decoder uses the
                    // reconstructed A, which equals `sa`).
                    let res_b = sb.wrapping_sub(apply_weight(pass.weight_b, sa));
                    pass.weight_b = update_weight_clip(pass.weight_b, pass.delta, sa, res_b);
                    buffer[li] = res_a;
                    buffer[ri] = res_b;
                    pass.history_a[0] = sb;
                }
                -2 => {
                    let pred_b = pass.history_b[0];
                    let res_b = sb.wrapping_sub(apply_weight(pass.weight_b, pred_b));
                    pass.weight_b = update_weight_clip(pass.weight_b, pass.delta, pred_b, res_b);
                    let res_a = sa.wrapping_sub(apply_weight(pass.weight_a, sb));
                    pass.weight_a = update_weight_clip(pass.weight_a, pass.delta, sb, res_a);
                    buffer[li] = res_a;
                    buffer[ri] = res_b;
                    pass.history_b[0] = sa;
                }
                -3 => {
                    let pred_a = pass.history_b[0];
                    let pred_b = pass.history_a[0];
                    let res_a = sa.wrapping_sub(apply_weight(pass.weight_a, pred_a));
                    pass.weight_a = update_weight_clip(pass.weight_a, pass.delta, pred_a, res_a);
                    let res_b = sb.wrapping_sub(apply_weight(pass.weight_b, pred_b));
                    pass.weight_b = update_weight_clip(pass.weight_b, pass.delta, pred_b, res_b);
                    buffer[li] = res_a;
                    buffer[ri] = res_b;
                    pass.history_a[0] = sa;
                    pass.history_b[0] = sb;
                }
                17 => {
                    let pa = pass.history_a[0]
                        .wrapping_mul(2)
                        .wrapping_sub(pass.history_a[1]);
                    let pb = pass.history_b[0]
                        .wrapping_mul(2)
                        .wrapping_sub(pass.history_b[1]);
                    let res_a = sa.wrapping_sub(apply_weight(pass.weight_a, pa));
                    let res_b = sb.wrapping_sub(apply_weight(pass.weight_b, pb));
                    pass.weight_a = update_weight(pass.weight_a, pass.delta, pa, res_a);
                    pass.weight_b = update_weight(pass.weight_b, pass.delta, pb, res_b);
                    buffer[li] = res_a;
                    buffer[ri] = res_b;
                    pass.history_a[1] = pass.history_a[0];
                    pass.history_a[0] = sa;
                    pass.history_b[1] = pass.history_b[0];
                    pass.history_b[0] = sb;
                }
                18 => {
                    let pa = pass.history_a[0]
                        .wrapping_mul(3)
                        .wrapping_sub(pass.history_a[1])
                        >> 1;
                    let pb = pass.history_b[0]
                        .wrapping_mul(3)
                        .wrapping_sub(pass.history_b[1])
                        >> 1;
                    let res_a = sa.wrapping_sub(apply_weight(pass.weight_a, pa));
                    let res_b = sb.wrapping_sub(apply_weight(pass.weight_b, pb));
                    pass.weight_a = update_weight(pass.weight_a, pass.delta, pa, res_a);
                    pass.weight_b = update_weight(pass.weight_b, pass.delta, pb, res_b);
                    buffer[li] = res_a;
                    buffer[ri] = res_b;
                    pass.history_a[1] = pass.history_a[0];
                    pass.history_a[0] = sa;
                    pass.history_b[1] = pass.history_b[0];
                    pass.history_b[0] = sb;
                }
                t => {
                    let rd = m & (MAX_TERM as usize - 1);
                    let pa = pass.history_a[rd];
                    let pb = pass.history_b[rd];
                    let res_a = sa.wrapping_sub(apply_weight(pass.weight_a, pa));
                    let res_b = sb.wrapping_sub(apply_weight(pass.weight_b, pb));
                    pass.weight_a = update_weight(pass.weight_a, pass.delta, pa, res_a);
                    pass.weight_b = update_weight(pass.weight_b, pass.delta, pb, res_b);
                    buffer[li] = res_a;
                    buffer[ri] = res_b;
                    let w = (m + t as usize) & (MAX_TERM as usize - 1);
                    pass.history_a[w] = sa;
                    pass.history_b[w] = sb;
                    m = m.wrapping_add(1);
                }
            }
        }
    }
    Ok(())
}

/// Assemble the application-ordered [`DecorrPass`] list for a **mono**
/// block from the three decorrelation sub-block payloads.
///
/// Inputs are the raw `0x02` (terms), `0x03` (weights) and `0x04`
/// (seed samples) payloads. The returned passes are in
/// *application* order — the order [`decorrelate_mono`] consumes — so a
/// caller decodes residuals → PCM with one further call.
///
/// ## Wire vs application order (spec §3.7)
///
/// The `0x02`/`0x03`/`0x04` metadata stores the passes in the **reverse**
/// of application order (the encoder's last-applied pass is stored first).
/// This helper therefore:
///
/// 1. reads each `0x02` byte with the **spec** encoding
///    ([`decode_term_byte`]: `term = (byte & 0x1f) - 5`, delta in the top
///    3 bits) — distinct from the unbiased wiki reading of
///    [`expand_terms`];
/// 2. expands the `0x03` weight bytes ([`expand_weight_byte`]) one per
///    pass (mono = one weight per term);
/// 3. expands the `0x04` seed words and partitions them per term **in
///    wire order** (each term consumes [`seed_count`] samples), then
/// 4. reverses the term / weight / seed-group lists together so the
///    returned passes are application-ordered.
///
/// ## Errors
///
/// * [`Error::DecorrelationTermsMissing`] — `terms_payload` is empty but
///   weights/seeds are present (cannot reconstruct passes without terms).
/// * [`Error::InvalidDecorrelationTerm`] — a term byte decodes outside
///   the spec valid set `{1..8, 17, 18, -1, -2, -3}`.
/// * [`Error::CrossTermOnMono`] — a cross term (`-1`/`-2`/`-3`) appears in
///   a mono block (spec §2.1 rejects negative terms for mono).
/// * [`Error::TooManyDecorrelationPasses`] — more than [`MAX_NTERMS`]
///   terms.
/// * [`Error::DecorrelationWeightCountMismatch`] — the `0x03` payload did
///   not carry exactly one weight per term.
/// * [`Error::DecorrelationSamplesOddByteCount`] — the `0x04` payload has
///   an odd byte length.
/// * [`Error::DecorrelationSampleCountMismatch`] — the `0x04` seed count
///   does not match the sum of per-term seed counts.
/// * [`Error::DecorrelationSeedUnderflow`] — a term's seed group is short
///   (surfaced by [`DecorrPass::new`]).
pub fn assemble_mono_passes(
    terms_payload: &[u8],
    weights_payload: &[u8],
    samples_payload: &[u8],
) -> Result<Vec<DecorrPass>> {
    // Decode the spec-encoded term bytes (wire order).
    let mut terms = Vec::with_capacity(terms_payload.len());
    let mut deltas = Vec::with_capacity(terms_payload.len());
    for &byte in terms_payload {
        let (term, delta) = decode_term_byte(byte);
        if !is_valid_term(term) {
            return Err(Error::InvalidDecorrelationTerm(term));
        }
        if is_cross_term(term) {
            // Spec §2.1: negative (cross) terms are rejected for mono.
            return Err(Error::CrossTermOnMono(term));
        }
        terms.push(term);
        deltas.push(delta);
    }

    if terms.is_empty() {
        // A mono block with no terms is a no-op decorrelation; but if the
        // caller handed weights/seeds without terms there is nothing to
        // pin them to.
        if !weights_payload.is_empty() || !samples_payload.is_empty() {
            return Err(Error::DecorrelationTermsMissing);
        }
        return Ok(Vec::new());
    }
    if terms.len() > MAX_NTERMS {
        return Err(Error::TooManyDecorrelationPasses(terms.len()));
    }

    // Mono: exactly one weight per pass (spec §3.6 / wiki "one or two
    // weights depending on channels").
    let weights: Vec<i32> = weights_payload
        .iter()
        .map(|&b| expand_weight_byte(b))
        .collect();
    if weights.len() != terms.len() {
        return Err(Error::DecorrelationWeightCountMismatch {
            expected: terms.len(),
            actual: weights.len(),
        });
    }

    // Expand and partition the seed samples per term, in wire order.
    // The payload primes a wire-order **prefix** of the passes: real
    // encoders may store seeds for fewer passes than the term list
    // carries (commonly just the first wire pass), and the remaining
    // passes start from zero history — the same "unspecified passes
    // start at 0" convention spec §3.6 states for the weights. (Pinned
    // black-box in round 405: reference-encoded files carry one term's
    // worth of `0x04` seeds for a five-term stack, and the stored
    // block CRC only matches when the unprimed passes seed zero.) A
    // payload that stops mid-term is malformed.
    let seeds = expand_samples(samples_payload)?;
    let mut seed_groups: Vec<&[i32]> = Vec::with_capacity(terms.len());
    let mut cursor = 0usize;
    for &t in &terms {
        let n = seed_count(t);
        if cursor == seeds.samples.len() {
            // Seed payload exhausted: this pass (and every later one)
            // starts with zero history.
            seed_groups.push(&[]);
        } else if cursor + n <= seeds.samples.len() {
            seed_groups.push(&seeds.samples[cursor..cursor + n]);
            cursor += n;
        } else {
            // Mid-term truncation: not a whole number of per-term
            // groups.
            return Err(Error::DecorrelationSampleCountMismatch {
                expected: cursor + n,
                actual: seeds.samples.len(),
            });
        }
    }
    if cursor != seeds.samples.len() {
        // More seeds than the whole term list can consume.
        return Err(Error::DecorrelationSampleCountMismatch {
            expected: cursor,
            actual: seeds.samples.len(),
        });
    }

    // Build passes in wire order, then reverse to application order so the
    // first pass returned undoes the encoder's last-applied pass (§3.7).
    let mut passes = Vec::with_capacity(terms.len());
    for i in 0..terms.len() {
        passes.push(DecorrPass::new(
            terms[i],
            deltas[i],
            weights[i],
            0,
            seed_groups[i],
            &[],
        )?);
    }
    passes.reverse();
    Ok(passes)
}

/// Assemble the application-ordered [`DecorrPass`] list for a **stereo**
/// block from the three decorrelation sub-block payloads.
///
/// Inputs are the raw `0x02` (terms), `0x03` (weights) and `0x04`
/// (seed samples) payloads of a two-channel block. The returned passes
/// are in *application* order (the order [`decorrelate_stereo`]
/// consumes), so a caller turns an interleaved `[L0, R0, L1, R1, …]`
/// residual buffer into PCM with one further call.
///
/// ## Per-channel layout (spec §3.6 / §3.7)
///
/// A stereo block stores, **per pass**, the state for *both* channels:
///
/// * **Weights (`0x03`)**: "one signed byte per pass per channel", so a
///   stereo pass carries two weight bytes laid out channel-A-then-channel-B
///   within the pass. The payload therefore holds `2 * nterms` bytes.
/// * **Seed samples (`0x04`)**: "per channel", with the per-channel count
///   tied to the term class ([`seed_count`]: `17`/`18` → 2, `1..8` → `term`,
///   cross → 1). A stereo pass stores channel A's seeds followed by
///   channel B's seeds, so each pass consumes `2 * seed_count(term)` words.
///
/// As with mono, the `0x02`/`0x03`/`0x04` metadata stores the passes in
/// the **reverse** of application order (encoder's last-applied pass
/// first), so this helper builds the passes in wire order and reverses
/// the whole list at the end (§3.7).
///
/// Cross terms (`-1`/`-2`/`-3`) are *valid* here — they are the
/// zero-delay inter-channel predictors the stereo loop runs with the
/// clipped weight update; a cross term seeds 1 sample per channel.
///
/// ## Errors
///
/// * [`Error::DecorrelationTermsMissing`] — `terms_payload` is empty but
///   weights/seeds are present.
/// * [`Error::InvalidDecorrelationTerm`] — a term byte decodes outside
///   the spec valid set `{1..8, 17, 18, -1, -2, -3}`.
/// * [`Error::TooManyDecorrelationPasses`] — more than [`MAX_NTERMS`].
/// * [`Error::DecorrelationWeightCountMismatch`] — the `0x03` payload did
///   not carry exactly `2 * nterms` weight bytes.
/// * [`Error::DecorrelationSamplesOddByteCount`] — the `0x04` payload has
///   an odd byte length.
/// * [`Error::DecorrelationSampleCountMismatch`] — the `0x04` seed count
///   does not equal `2 * Σ seed_count(term)`.
/// * [`Error::DecorrelationSeedUnderflow`] — a per-channel seed group is
///   short (surfaced by [`DecorrPass::new`]).
pub fn assemble_stereo_passes(
    terms_payload: &[u8],
    weights_payload: &[u8],
    samples_payload: &[u8],
) -> Result<Vec<DecorrPass>> {
    // Decode the spec-encoded term bytes (wire order). Cross terms ARE
    // allowed on stereo, so we do not reject negative terms here.
    let mut terms = Vec::with_capacity(terms_payload.len());
    let mut deltas = Vec::with_capacity(terms_payload.len());
    for &byte in terms_payload {
        let (term, delta) = decode_term_byte(byte);
        if !is_valid_term(term) {
            return Err(Error::InvalidDecorrelationTerm(term));
        }
        terms.push(term);
        deltas.push(delta);
    }

    if terms.is_empty() {
        if !weights_payload.is_empty() || !samples_payload.is_empty() {
            return Err(Error::DecorrelationTermsMissing);
        }
        return Ok(Vec::new());
    }
    if terms.len() > MAX_NTERMS {
        return Err(Error::TooManyDecorrelationPasses(terms.len()));
    }

    // Stereo: two weights per pass (channel A then channel B within a
    // pass) per spec §3.6 "one signed byte per pass per channel".
    let weights: Vec<i32> = weights_payload
        .iter()
        .map(|&b| expand_weight_byte(b))
        .collect();
    if weights.len() != terms.len() * 2 {
        return Err(Error::DecorrelationWeightCountMismatch {
            expected: terms.len() * 2,
            actual: weights.len(),
        });
    }

    // Expand and partition the seed samples per pass. Each pass consumes
    // channel A's seeds (seed_count words) followed by channel B's seeds
    // (another seed_count words). As on the mono side, the payload
    // primes a wire-order **prefix** of the passes (real encoders may
    // store seeds for fewer passes than the term list carries; the
    // remaining passes start from zero history — round 405, pinned
    // black-box against reference-encoded files via the stored block
    // CRC). A payload that stops inside a pass's A+B group is malformed.
    let seeds = expand_samples(samples_payload)?;
    let mut seed_a_groups: Vec<&[i32]> = Vec::with_capacity(terms.len());
    let mut seed_b_groups: Vec<&[i32]> = Vec::with_capacity(terms.len());
    let mut cursor = 0usize;
    for &t in &terms {
        let n = seed_count(t);
        if cursor == seeds.samples.len() {
            seed_a_groups.push(&[]);
            seed_b_groups.push(&[]);
        } else if cursor + 2 * n <= seeds.samples.len() {
            seed_a_groups.push(&seeds.samples[cursor..cursor + n]);
            cursor += n;
            seed_b_groups.push(&seeds.samples[cursor..cursor + n]);
            cursor += n;
        } else {
            return Err(Error::DecorrelationSampleCountMismatch {
                expected: cursor + 2 * n,
                actual: seeds.samples.len(),
            });
        }
    }
    if cursor != seeds.samples.len() {
        return Err(Error::DecorrelationSampleCountMismatch {
            expected: cursor,
            actual: seeds.samples.len(),
        });
    }

    // Build passes in wire order (weight_a = weights[2i], weight_b =
    // weights[2i+1]), then reverse to application order (§3.7).
    let mut passes = Vec::with_capacity(terms.len());
    for i in 0..terms.len() {
        passes.push(DecorrPass::new(
            terms[i],
            deltas[i],
            weights[2 * i],
            weights[2 * i + 1],
            seed_a_groups[i],
            seed_b_groups[i],
        )?);
    }
    passes.reverse();
    Ok(passes)
}

/// Pack a working decorrelation weight into its `0x03` stored byte —
/// the nearest-value inverse of the spec §3.6 weight expansion
/// (`n = byte * 8; if (n > 0) n += (n + 64) >> 7`, the `restore_weight`
/// rule whose two endpoints are `+127 → +1024` and `-128 → -1024`).
///
/// The expansion is strictly monotonic across the 256 stored bytes (each
/// byte step moves the working value by 8 or 9), so the inverse is found
/// with a small search window around the linear `weight / 8` estimate.
/// When `weight` falls between two representable values the packer
/// returns the numerically smaller byte among the nearest (deterministic
/// tie-break); out-of-range inputs clamp to the `±1024` endpoints.
///
/// The exact-representability predicate an encoder needs before
/// serializing is `quantize_weight(w) == w` (see [`quantize_weight`]).
pub fn pack_weight_byte(weight: i32) -> u8 {
    let estimate = (weight >> 3).clamp(i32::from(i8::MIN), i32::from(i8::MAX)) as i8;
    // The positive-branch correction `(n + 64) >> 7` is at most 8 across
    // the byte range, i.e. within one byte step of the `w / 8` estimate;
    // a ±4 window is comfortably wider than the worst-case offset.
    let lo = estimate.saturating_sub(4);
    let hi = estimate.saturating_add(4);
    let mut best = lo;
    let mut best_dist = (i64::from(expand_weight_byte(lo as u8)) - i64::from(weight)).abs();
    let mut b = lo;
    while b < hi {
        b += 1;
        let dist = (i64::from(expand_weight_byte(b as u8)) - i64::from(weight)).abs();
        if dist < best_dist {
            best = b;
            best_dist = dist;
        }
    }
    best as u8
}

/// Quantize a working decorrelation weight to the nearest value the
/// `0x03` stored byte can represent: `expand ∘ pack`.
///
/// An encoder that derives a working weight (e.g. by training the §3.4
/// adaptation over the block) must start its *real* forward pass from
/// the quantized value — the decoder reconstructs its starting weight
/// from the stored byte, so only a quantized weight keeps the two
/// directions' pass state byte-identical. Idempotent:
/// `quantize_weight(quantize_weight(w)) == quantize_weight(w)`.
pub fn quantize_weight(weight: i32) -> i32 {
    expand_weight_byte(pack_weight_byte(weight))
}

/// Pack a seed-history sample into its canonical `0x04` on-wire 16-bit
/// log word — the nearest-value forward inverse of the log-domain
/// expansion ([`expand_samples`] / [`crate::wp_exp2s`]).
///
/// The magnitude is logged with [`crate::wp_log2`] and the word's sign
/// carries the value's sign (`wp_exp2s` is odd — staged spec
/// `wavpack-log2-exp2.md` §4 step 1). Zero packs to the canonical
/// all-zero word (§6 erratum pin; a round-393 black-box
/// cross-validation showed reference decoders accept only that zero
/// form). The pack is a quantizer: small magnitudes round-trip exactly
/// and wide ones to within the 8-fractional-bit table precision — the
/// exactness predicate is `quantize_seed_sample(v) == v` (see
/// [`quantize_seed_sample`]).
pub fn pack_sample_word(value: i32) -> [u8; 2] {
    crate::logpack::pack_log_word(value)
}

/// Quantize a seed-history sample to the nearest value the `0x04`
/// log-word can represent: `expand ∘ pack` ([`crate::quantize_log_value`]).
/// Values that the 16-bit log word represents exactly (all small
/// magnitudes) are returned verbatim; wider values quantize to the
/// table's 8-fractional-bit precision. Idempotent, like
/// [`quantize_weight`] — an encoder priming pass history from real PCM
/// must prime with the quantized values so the decoder's seed expansion
/// reconstructs the identical history.
pub fn quantize_seed_sample(value: i32) -> i32 {
    crate::logpack::quantize_log_value(value)
}

/// Pack a `(term, delta)` pair into its `0x02` on-wire term byte — the
/// exact inverse of [`decode_term_byte`]. Per spec §2.1 the low 5 bits
/// carry the `+5`-biased term and the top 3 bits carry the per-pass
/// weight-adaptation `delta`.
///
/// Returns [`Error::InvalidDecorrelationTerm`] for a term outside the
/// spec valid set `{1..8, 17, 18, -1, -2, -3}` and
/// [`Error::EncodeDeltaOutOfRange`] for a delta outside the 3-bit field.
pub fn encode_term_byte(term: i8, delta: i32) -> Result<u8> {
    if !is_valid_term(term) {
        return Err(Error::InvalidDecorrelationTerm(term));
    }
    if !(0..=i32::from(TERM_DELTA_MASK)).contains(&delta) {
        return Err(Error::EncodeDeltaOutOfRange(delta));
    }
    let biased = ((term + TERM_BYTE_BIAS) as u8) & TERM_PREDICTOR_MASK;
    Ok(biased | ((delta as u8) << TERM_PREDICTOR_BITS))
}

/// Pack a weight for serialization, refusing a value the stored byte
/// cannot reproduce exactly.
fn pack_weight_exact(weight: i32) -> Result<u8> {
    let byte = pack_weight_byte(weight);
    if expand_weight_byte(byte) != weight {
        return Err(Error::EncodeWeightNotRepresentable(weight));
    }
    Ok(byte)
}

/// Pack a seed for serialization, refusing a value the log-word cannot
/// reproduce exactly.
fn pack_seed_exact(value: i32) -> Result<[u8; 2]> {
    let word = pack_sample_word(value);
    if expand_sample_word(word[0], word[1]) != value {
        return Err(Error::EncodeSeedNotRepresentable(value));
    }
    Ok(word)
}

/// Read a pass's per-channel seed-history samples back out of its ring,
/// newest-first — the exact inverse of the [`DecorrPass::new`] /
/// `seed_history` placement (fixed-lag rings reverse into slots
/// `0..t-1`; `17`/`18` occupy slots `0`/`1` in order; cross terms slot
/// `0`).
fn extract_seeds(term: i8, ring: &[i32; MAX_TERM as usize]) -> Vec<i32> {
    let n = seed_count(term);
    if (1..=MAX_TERM).contains(&term) {
        (0..n).map(|i| ring[n - 1 - i]).collect()
    } else {
        ring[..n].to_vec()
    }
}

/// Serialize an application-ordered **mono** [`DecorrPass`] list into the
/// three raw decorrelation metadata payloads — `0x02` terms, `0x03`
/// weights, `0x04` seed samples — the exact forward inverse of
/// [`assemble_mono_passes`]:
/// `assemble_mono_passes(&t, &w, &s)? == passes` for the returned
/// `(t, w, s)`.
///
/// The caller's list is in *application* order (the order
/// [`decorrelate_mono`] / [`recorrelate_mono`] consume); per spec §3.7
/// the wire stores the passes in the reverse (encoder's last-applied
/// pass first), so the serializer walks the list back-to-front. Each
/// pass contributes one `0x02` term byte ([`encode_term_byte`]), one
/// `0x03` weight byte, and `seed_count(term)` `0x04` log-words from its
/// (initial) channel-A history.
///
/// The passes must carry **serializable state**: every weight must be a
/// stored-byte expansion ([`Error::EncodeWeightNotRepresentable`]
/// otherwise — quantize via [`quantize_weight`]) and every seed a
/// log-word expansion ([`Error::EncodeSeedNotRepresentable`] — quantize
/// via [`quantize_seed_sample`]); the delta must fit the 3-bit field
/// ([`Error::EncodeDeltaOutOfRange`]). Cross terms are rejected for mono
/// ([`Error::CrossTermOnMono`]) and an over-long list trips
/// [`Error::TooManyDecorrelationPasses`], matching the assembler's
/// gates. Serialize a pass list *before* running it — the loops mutate
/// weights and history in place.
pub fn serialize_mono_passes(passes: &[DecorrPass]) -> Result<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    if passes.len() > MAX_NTERMS {
        return Err(Error::TooManyDecorrelationPasses(passes.len()));
    }
    for pass in passes {
        if is_cross_term(pass.term) {
            return Err(Error::CrossTermOnMono(pass.term));
        }
    }
    let mut terms = Vec::with_capacity(passes.len());
    let mut weights = Vec::with_capacity(passes.len());
    let mut samples = Vec::new();
    // Wire order is the reverse of application order (§3.7).
    for pass in passes.iter().rev() {
        terms.push(encode_term_byte(pass.term, pass.delta)?);
        weights.push(pack_weight_exact(pass.weight_a)?);
        for seed in extract_seeds(pass.term, &pass.history_a) {
            samples.extend_from_slice(&pack_seed_exact(seed)?);
        }
    }
    Ok((terms, weights, samples))
}

/// Serialize an application-ordered **stereo** [`DecorrPass`] list into
/// the three raw decorrelation metadata payloads — the exact forward
/// inverse of [`assemble_stereo_passes`]:
/// `assemble_stereo_passes(&t, &w, &s)? == passes`.
///
/// The stereo wire layout (spec §3.6): per pass, **two** weight bytes
/// (channel A then channel B) and `2 * seed_count(term)` seed log-words
/// (channel A's seeds then channel B's). Cross terms (`-1`/`-2`/`-3`)
/// are valid here. All the [`serialize_mono_passes`] representability
/// gates apply to both channels' state.
pub fn serialize_stereo_passes(passes: &[DecorrPass]) -> Result<(Vec<u8>, Vec<u8>, Vec<u8>)> {
    if passes.len() > MAX_NTERMS {
        return Err(Error::TooManyDecorrelationPasses(passes.len()));
    }
    let mut terms = Vec::with_capacity(passes.len());
    let mut weights = Vec::with_capacity(passes.len() * 2);
    let mut samples = Vec::new();
    // Wire order is the reverse of application order (§3.7).
    for pass in passes.iter().rev() {
        terms.push(encode_term_byte(pass.term, pass.delta)?);
        weights.push(pack_weight_exact(pass.weight_a)?);
        weights.push(pack_weight_exact(pass.weight_b)?);
        for seed in extract_seeds(pass.term, &pass.history_a) {
            samples.extend_from_slice(&pack_seed_exact(seed)?);
        }
        for seed in extract_seeds(pass.term, &pass.history_b) {
            samples.extend_from_slice(&pack_seed_exact(seed)?);
        }
    }
    Ok((terms, weights, samples))
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
    fn sample_word_is_a_signed_log_word_expanded_by_wp_exp2s() {
        // The staged log2/exp2 spec §4 worked example: log word
        // 2807 = 0x0af7 expands to 1000; the sign lives on the whole
        // word (odd function), so the negated word expands to -1000.
        let s = expand_sample_word(0xf7, 0x0a);
        assert_eq!(s, 1000);
        let [lo, hi] = (-2807i16).to_le_bytes();
        assert_eq!(expand_sample_word(lo, hi), -1000);
        // Log word 256 = 0x0100 expands to 1 (spec meta anchor).
        assert_eq!(expand_sample_word(0x00, 0x01), 1);
        // The canonical all-zero word is the exact zero (spec §6 pin).
        assert_eq!(expand_sample_word(0x00, 0x00), 0);
    }

    #[test]
    fn sample_word_int_part_selects_the_mantissa_shift() {
        // Spec §4 step 3: int part 9 leaves the 9-bit mantissa
        // unshifted (0x100 with a zero fraction), 10 doubles it, 8
        // halves it.
        assert_eq!(expand_sample_word(0x00, 0x09), 0x100);
        assert_eq!(expand_sample_word(0x00, 0x0a), 0x200);
        assert_eq!(expand_sample_word(0x00, 0x08), 0x80);
    }

    #[test]
    fn sample_payload_pairs_bytes_into_words() {
        // Three log words: 0x0af7 (= 1000), 0x0100 (= 1), 0x0000 (= 0).
        let payload = [0xf7, 0x0a, 0x00, 0x01, 0x00, 0x00];
        let s = expand_samples(&payload).unwrap();
        assert_eq!(s.samples, vec![1000, 1, 0]);
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
        // 18 (2) = 3 seed samples. Each on-disk sample is a
        // little-endian signed 16-bit log word packed with
        // pack_sample_word (small magnitudes round-trip exactly).
        let mut wire = Vec::new();
        for v in [1i32, 2, 3] {
            wire.extend_from_slice(&pack_sample_word(v));
        }
        let ds = expand_samples(&wire).unwrap();
        assert_eq!(ds.samples, vec![1, 2, 3]);
        let dt = expand_terms(&[6, 18]);
        let parts = partition_decorrelation_samples(&dt, &ds).unwrap();
        assert_eq!(parts.len(), 2);
        assert_eq!(parts[0], vec![1]);
        assert_eq!(parts[1], vec![2, 3]);
    }

    #[test]
    fn sample_extreme_log_words_stay_defined_rather_than_panic() {
        // A conformant stream keeps log words within ±8447 (the
        // staged spec §7 max); hostile words with int parts far past
        // the 32-bit magnitude ceiling must expand to *some* defined
        // value without panicking (the left shift wraps).
        let _ = expand_sample_word(0x01, 0x7F);
        let _ = expand_sample_word(0xFF, 0x7F);
        let _ = expand_sample_word(0x00, 0x80); // i16::MIN word
        let _ = expand_sample_word(0xFF, 0xFF); // -1 log word
                                                // The all-zero word stays the canonical exact zero.
        assert_eq!(expand_sample_word(0x00, 0x00), 0);
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
        // >>), matching the spec's signed shift. weight = 1,
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
        // = -min(-893, 1024) = -(-893) = 893. The spec magnitude
        // step shrinks the positive weight toward zero by delta-ish and
        // re-signs; our faithful transcription matches the branch-free
        // arithmetic exactly.
        assert_eq!(update_weight_clip(900, 7, -4, 9), 893);
        // A negative starting weight with opposite-sign operands:
        // weight = -900, s = -1 (source/result differ). The spec
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

    // ---- Prediction loop (§3.2 / §3.3 / §3.7) ----

    // A standalone forward (encode) pass for a single mono term: the exact
    // arithmetic inverse of `decorrelate_mono`'s per-sample step. Used only
    // by the round-trip tests to prove the decode loop inverts the encode.
    fn forward_mono(term: i8, delta: i32, weight0: i32, seed: &[i32], pcm: &[i32]) -> Vec<i32> {
        let mut ring = [0i32; MAX_TERM as usize];
        seed_history(term, seed, &mut ring).unwrap();
        let mut weight = weight0;
        let mut m = 0usize;
        let mut out = Vec::with_capacity(pcm.len());
        for &sample in pcm {
            let pred = match term {
                17 => 2 * ring[0] - ring[1],
                18 => (3 * ring[0] - ring[1]) >> 1,
                _ => ring[m & (MAX_TERM as usize - 1)],
            };
            // Encode: residual = sample - apply_weight(weight, pred).
            let residual = sample.wrapping_sub(apply_weight(weight, pred));
            weight = update_weight(weight, delta, pred, residual);
            if term == 17 || term == 18 {
                ring[1] = ring[0];
                ring[0] = sample;
            } else {
                ring[(m + term as usize) & (MAX_TERM as usize - 1)] = sample;
                m = m.wrapping_add(1);
            }
            out.push(residual);
        }
        out
    }

    fn round_trip_mono(term: i8, delta: i32, weight0: i32, seed: &[i32], pcm: &[i32]) {
        let mut residuals = forward_mono(term, delta, weight0, seed, pcm);
        let mut pass = DecorrPass::new(term, delta, weight0, 0, seed, &[]).unwrap();
        decorrelate_mono(std::slice::from_mut(&mut pass), &mut residuals).unwrap();
        assert_eq!(residuals, pcm, "mono round-trip failed for term {term}");
    }

    #[test]
    fn decode_term_byte_applies_plus5_bias() {
        // term field 6 → 6 - 5 = 1, delta in high 3 bits.
        assert_eq!(decode_term_byte(0x06), (1, 0));
        // (3 << 5) | (8+5=13) → term 8, delta 3.
        assert_eq!(decode_term_byte((3 << 5) | 13), (8, 3));
        // term field 4 → 4 - 5 = -1 (cross), delta 7.
        assert_eq!(decode_term_byte((7 << 5) | 4), (-1, 7));
        // 17 + 5 = 22 → term 17.
        assert_eq!(decode_term_byte(22).0, 17);
    }

    #[test]
    fn is_valid_term_matches_spec_set() {
        for t in [1, 2, 3, 4, 5, 6, 7, 8, 17, 18, -1, -2, -3] {
            assert!(is_valid_term(t), "term {t} should be valid");
        }
        for t in [0, 9, 16, 19, -4, -5, 100i8.wrapping_add(0)] {
            assert!(!is_valid_term(t), "term {t} should be invalid");
        }
    }

    #[test]
    fn new_pass_rejects_invalid_term() {
        assert_eq!(
            DecorrPass::new(0, 0, 0, 0, &[], &[]),
            Err(Error::InvalidDecorrelationTerm(0))
        );
        assert_eq!(
            DecorrPass::new(9, 0, 0, 0, &[], &[]),
            Err(Error::InvalidDecorrelationTerm(9))
        );
    }

    #[test]
    fn new_pass_rejects_seed_underflow() {
        // Term 3 needs 3 seeds; supply 2.
        assert_eq!(
            DecorrPass::new(3, 0, 1024, 0, &[10, 20], &[]),
            Err(Error::DecorrelationSeedUnderflow {
                term: 3,
                supplied: 2,
            })
        );
        // Term 17 needs 2 seeds; supply 1.
        assert_eq!(
            DecorrPass::new(17, 0, 1024, 0, &[10], &[]),
            Err(Error::DecorrelationSeedUnderflow {
                term: 17,
                supplied: 1,
            })
        );
    }

    #[test]
    fn mono_round_trip_fixed_lag_terms() {
        let pcm = [
            100, 132, 90, 75, 210, -40, -33, 12, 255, 300, 280, 260, 0, -1, 5,
        ];
        // Seeds newest-first; supply 8 so any lag 1..8 is primed.
        let seed = [50, 40, 30, 20, 10, 0, -10, -20];
        for term in 1..=8i8 {
            round_trip_mono(term, 2, 700, &seed[..term as usize], &pcm);
        }
    }

    #[test]
    fn mono_round_trip_extrapolate_terms() {
        let pcm = [10, 12, 9, 7, 21, -4, -3, 1, 25, 30, 28, 26, 0, -1, 5, 8, 9];
        let seed = [5, 3]; // s[-1]=5, s[-2]=3
        round_trip_mono(17, 3, 900, &seed, &pcm);
        round_trip_mono(18, 1, 512, &seed, &pcm);
    }

    #[test]
    fn mono_round_trip_zero_weight_zero_delta() {
        // weight 0 / delta 0: residual == sample (apply_weight(0,·)=0,
        // weight never moves). The loop must be a no-op identity.
        let pcm = [7, -3, 42, 0, 99];
        round_trip_mono(1, 0, 0, &[0], &pcm);
        // delta 0, unity weight, term 1: still an exact inverse.
        round_trip_mono(1, 0, 1024, &[3], &pcm);
    }

    #[test]
    fn mono_rejects_cross_term() {
        let mut pass = DecorrPass::new(-1, 0, 0, 0, &[0], &[0]).unwrap();
        let mut buf = [1, 2, 3];
        assert_eq!(
            decorrelate_mono(std::slice::from_mut(&mut pass), &mut buf),
            Err(Error::CrossTermOnMono(-1))
        );
    }

    #[test]
    fn too_many_passes_rejected() {
        let mut passes: Vec<DecorrPass> = (0..MAX_NTERMS + 1)
            .map(|_| DecorrPass::new(1, 0, 0, 0, &[0], &[]).unwrap())
            .collect();
        let mut buf = [1, 2, 3];
        assert_eq!(
            decorrelate_mono(&mut passes, &mut buf),
            Err(Error::TooManyDecorrelationPasses(MAX_NTERMS + 1))
        );
    }

    // --- stereo ---

    #[allow(clippy::too_many_arguments)]
    fn forward_stereo(
        term: i8,
        delta: i32,
        wa0: i32,
        wb0: i32,
        seed_a: &[i32],
        seed_b: &[i32],
        pcm: &[i32],
    ) -> Vec<i32> {
        // Mirror decorrelate_stereo's per-sample arithmetic, inverted.
        let mut ra = [0i32; MAX_TERM as usize];
        let mut rb = [0i32; MAX_TERM as usize];
        seed_history(term, seed_a, &mut ra).unwrap();
        seed_history(term, seed_b, &mut rb).unwrap();
        let mut wa = wa0;
        let mut wb = wb0;
        let mut m = 0usize;
        let mut out = vec![0i32; pcm.len()];
        let pairs = pcm.len() / 2;
        for p in 0..pairs {
            let li = 2 * p;
            let ri = li + 1;
            let sa = pcm[li];
            let sb = pcm[ri];
            match term {
                -1 => {
                    let pred_a = ra[0];
                    let resa = sa.wrapping_sub(apply_weight(wa, pred_a));
                    wa = update_weight_clip(wa, delta, pred_a, resa);
                    let resb = sb.wrapping_sub(apply_weight(wb, sa));
                    wb = update_weight_clip(wb, delta, sa, resb);
                    out[li] = resa;
                    out[ri] = resb;
                    ra[0] = sb;
                }
                -2 => {
                    let pred_b = rb[0];
                    let resb = sb.wrapping_sub(apply_weight(wb, pred_b));
                    wb = update_weight_clip(wb, delta, pred_b, resb);
                    let resa = sa.wrapping_sub(apply_weight(wa, sb));
                    wa = update_weight_clip(wa, delta, sb, resa);
                    out[li] = resa;
                    out[ri] = resb;
                    rb[0] = sa;
                }
                -3 => {
                    let pred_a = rb[0];
                    let pred_b = ra[0];
                    let resa = sa.wrapping_sub(apply_weight(wa, pred_a));
                    wa = update_weight_clip(wa, delta, pred_a, resa);
                    let resb = sb.wrapping_sub(apply_weight(wb, pred_b));
                    wb = update_weight_clip(wb, delta, pred_b, resb);
                    out[li] = resa;
                    out[ri] = resb;
                    ra[0] = sa;
                    rb[0] = sb;
                }
                17 => {
                    let pa = 2 * ra[0] - ra[1];
                    let pb = 2 * rb[0] - rb[1];
                    out[li] = sa.wrapping_sub(apply_weight(wa, pa));
                    out[ri] = sb.wrapping_sub(apply_weight(wb, pb));
                    wa = update_weight(wa, delta, pa, out[li]);
                    wb = update_weight(wb, delta, pb, out[ri]);
                    ra[1] = ra[0];
                    ra[0] = sa;
                    rb[1] = rb[0];
                    rb[0] = sb;
                }
                18 => {
                    let pa = (3 * ra[0] - ra[1]) >> 1;
                    let pb = (3 * rb[0] - rb[1]) >> 1;
                    out[li] = sa.wrapping_sub(apply_weight(wa, pa));
                    out[ri] = sb.wrapping_sub(apply_weight(wb, pb));
                    wa = update_weight(wa, delta, pa, out[li]);
                    wb = update_weight(wb, delta, pb, out[ri]);
                    ra[1] = ra[0];
                    ra[0] = sa;
                    rb[1] = rb[0];
                    rb[0] = sb;
                }
                t => {
                    let rd = m & (MAX_TERM as usize - 1);
                    let pa = ra[rd];
                    let pb = rb[rd];
                    out[li] = sa.wrapping_sub(apply_weight(wa, pa));
                    out[ri] = sb.wrapping_sub(apply_weight(wb, pb));
                    wa = update_weight(wa, delta, pa, out[li]);
                    wb = update_weight(wb, delta, pb, out[ri]);
                    let w = (m + t as usize) & (MAX_TERM as usize - 1);
                    ra[w] = sa;
                    rb[w] = sb;
                    m = m.wrapping_add(1);
                }
            }
        }
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn round_trip_stereo(
        term: i8,
        delta: i32,
        wa0: i32,
        wb0: i32,
        seed_a: &[i32],
        seed_b: &[i32],
        pcm: &[i32],
    ) {
        let mut residuals = forward_stereo(term, delta, wa0, wb0, seed_a, seed_b, pcm);
        let mut pass = DecorrPass::new(term, delta, wa0, wb0, seed_a, seed_b).unwrap();
        decorrelate_stereo(std::slice::from_mut(&mut pass), &mut residuals).unwrap();
        assert_eq!(residuals, pcm, "stereo round-trip failed for term {term}");
    }

    #[test]
    fn stereo_round_trip_per_channel_terms() {
        let pcm = [
            100, -90, 132, 88, 90, -75, 75, 60, 210, -200, -40, 33, -33, 21, 12, -9,
        ];
        let seed = [50, 40, 30, 20, 10, 0, -10, -20];
        for term in 1..=8i8 {
            round_trip_stereo(
                term,
                2,
                700,
                650,
                &seed[..term as usize],
                &seed[..term as usize],
                &pcm,
            );
        }
        round_trip_stereo(17, 3, 900, 800, &[5, 3], &[6, 4], &pcm);
        round_trip_stereo(18, 1, 512, 480, &[5, 3], &[6, 4], &pcm);
    }

    #[test]
    fn stereo_round_trip_cross_terms() {
        let pcm = [
            100, -90, 132, 88, 90, -75, 75, 60, 210, -200, -40, 33, -33, 21, 12, -9,
        ];
        round_trip_stereo(-1, 2, 300, 400, &[10], &[20], &pcm);
        round_trip_stereo(-2, 3, 350, 450, &[11], &[21], &pcm);
        round_trip_stereo(-3, 1, 200, 250, &[12], &[22], &pcm);
    }

    #[test]
    fn stereo_round_trip_multi_pass() {
        // A realistic stack: a cross term, then two per-channel terms,
        // applied in order. Build residuals by running the forward of each
        // pass in reverse order, then decode forward and recover the PCM.
        let pcm: Vec<i32> = (0..40).map(|i| ((i * 37) % 211) - 100).collect();
        let terms = [(-3i8, 1i32), (2, 2), (18, 3)];
        let seeds: [(Vec<i32>, Vec<i32>); 3] = [
            (vec![3], vec![5]),
            (vec![7, 0], vec![9, 0]),
            (vec![1, 2], vec![3, 4]),
        ];
        let weights = [(200, 220), (600, 640), (512, 500)];

        // Forward-encode: apply passes in REVERSE of decode order (decode
        // undoes last-encoded first), each over the running buffer.
        let mut buf = pcm.clone();
        for idx in (0..terms.len()).rev() {
            let (t, d) = terms[idx];
            let (wa, wb) = weights[idx];
            buf = forward_stereo(t, d, wa, wb, &seeds[idx].0, &seeds[idx].1, &buf);
        }

        // Decode: passes in forward (application) order.
        let mut passes: Vec<DecorrPass> = (0..terms.len())
            .map(|idx| {
                let (t, d) = terms[idx];
                let (wa, wb) = weights[idx];
                DecorrPass::new(t, d, wa, wb, &seeds[idx].0, &seeds[idx].1).unwrap()
            })
            .collect();
        decorrelate_stereo(&mut passes, &mut buf).unwrap();
        assert_eq!(buf, pcm, "multi-pass stereo round-trip failed");
    }

    #[test]
    fn mono_round_trip_multi_pass() {
        let pcm: Vec<i32> = (0..40).map(|i| ((i * 53) % 173) - 80).collect();
        let terms = [(1i8, 2i32), (3, 1), (17, 3)];
        let seeds: [Vec<i32>; 3] = [vec![4], vec![1, 2, 3], vec![5, 6]];
        let weights = [600, 700, 512];

        let mut buf = pcm.clone();
        for idx in (0..terms.len()).rev() {
            let (t, d) = terms[idx];
            buf = forward_mono(t, d, weights[idx], &seeds[idx], &buf);
        }
        let mut passes: Vec<DecorrPass> = (0..terms.len())
            .map(|idx| {
                let (t, d) = terms[idx];
                DecorrPass::new(t, d, weights[idx], 0, &seeds[idx], &[]).unwrap()
            })
            .collect();
        decorrelate_mono(&mut passes, &mut buf).unwrap();
        assert_eq!(buf, pcm, "multi-pass mono round-trip failed");
    }

    // ---- public forward encoders: recorrelate_mono / recorrelate_stereo ----

    // `(term, delta, weight, seeds)` for a mono pass.
    type MonoSpec = (i8, i32, i32, Vec<i32>);
    // `(term, delta, weight_a, weight_b, seeds_a, seeds_b)` for a stereo pass.
    type StereoSpec = (i8, i32, i32, i32, Vec<i32>, Vec<i32>);

    // Build a fresh application-ordered pass list (rebuilt from seeds each
    // call so its state is pristine).
    fn build_mono_passes(specs: &[MonoSpec]) -> Vec<DecorrPass> {
        specs
            .iter()
            .map(|(t, d, w, s)| DecorrPass::new(*t, *d, *w, 0, s, &[]).unwrap())
            .collect()
    }

    #[test]
    fn recorrelate_mono_inverts_decorrelate_mono_single_pass() {
        let pcm = [
            100, 132, 90, 75, 210, -40, -33, 12, 255, 300, 280, 260, 0, -1, 5,
        ];
        let seed = [50, 40, 30, 20, 10, 0, -10, -20];
        for term in 1..=8i8 {
            let specs = vec![(term, 2, 700, seed[..term as usize].to_vec())];
            let mut enc = build_mono_passes(&specs);
            let mut buf = pcm.to_vec();
            recorrelate_mono(&mut enc, &mut buf).unwrap();
            // Decode the residuals with a PRISTINE pass list (same config).
            let mut dec = build_mono_passes(&specs);
            decorrelate_mono(&mut dec, &mut buf).unwrap();
            assert_eq!(buf, pcm, "public mono round-trip failed for term {term}");
        }
        // Extrapolators.
        for term in [17i8, 18] {
            let specs = vec![(term, 3, 900, vec![5, 3])];
            let mut enc = build_mono_passes(&specs);
            let mut buf = pcm.to_vec();
            recorrelate_mono(&mut enc, &mut buf).unwrap();
            let mut dec = build_mono_passes(&specs);
            decorrelate_mono(&mut dec, &mut buf).unwrap();
            assert_eq!(buf, pcm, "public mono round-trip failed for term {term}");
        }
    }

    #[test]
    fn recorrelate_mono_inverts_decorrelate_mono_multi_pass() {
        let pcm: Vec<i32> = (0..40).map(|i| ((i * 53) % 173) - 80).collect();
        let specs = vec![
            (1i8, 2i32, 600i32, vec![4]),
            (3, 1, 700, vec![1, 2, 3]),
            (17, 3, 512, vec![5, 6]),
        ];
        let mut enc = build_mono_passes(&specs);
        let mut buf = pcm.clone();
        recorrelate_mono(&mut enc, &mut buf).unwrap();
        let mut dec = build_mono_passes(&specs);
        decorrelate_mono(&mut dec, &mut buf).unwrap();
        assert_eq!(buf, pcm, "public multi-pass mono round-trip failed");
    }

    #[test]
    fn recorrelate_mono_matches_private_forward_helper() {
        // The public encoder must produce the SAME residuals as the
        // private single-pass `forward_mono` for a one-pass config.
        let pcm = [10, 12, 9, 7, 21, -4, -3, 1, 25, 30, 28, 26, 0, -1, 5];
        let seed = [40, 30, 20, 10, 0, -10, -20, -30];
        for term in 1..=8i8 {
            let expected = forward_mono(term, 2, 700, &seed[..term as usize], &pcm);
            let mut enc = build_mono_passes(&[(term, 2, 700, seed[..term as usize].to_vec())]);
            let mut buf = pcm.to_vec();
            recorrelate_mono(&mut enc, &mut buf).unwrap();
            assert_eq!(buf, expected, "recorrelate_mono mismatch for term {term}");
        }
    }

    #[test]
    fn recorrelate_mono_rejects_cross_term() {
        let mut passes = vec![DecorrPass::new(-1, 0, 0, 0, &[0], &[0]).unwrap()];
        let mut buf = [1, 2, 3];
        assert_eq!(
            recorrelate_mono(&mut passes, &mut buf),
            Err(Error::CrossTermOnMono(-1))
        );
    }

    #[test]
    fn recorrelate_mono_rejects_too_many_passes() {
        let mut passes: Vec<DecorrPass> = (0..MAX_NTERMS + 1)
            .map(|_| DecorrPass::new(1, 0, 0, 0, &[0], &[]).unwrap())
            .collect();
        let mut buf = [1, 2, 3];
        assert_eq!(
            recorrelate_mono(&mut passes, &mut buf),
            Err(Error::TooManyDecorrelationPasses(MAX_NTERMS + 1))
        );
    }

    fn build_stereo_passes(specs: &[StereoSpec]) -> Vec<DecorrPass> {
        specs
            .iter()
            .map(|(t, d, wa, wb, sa, sb)| DecorrPass::new(*t, *d, *wa, *wb, sa, sb).unwrap())
            .collect()
    }

    #[test]
    fn recorrelate_stereo_inverts_decorrelate_stereo_single_pass() {
        let pcm = [
            100, -90, 132, 88, 90, -75, 75, 60, 210, -200, -40, 33, -33, 21, 12, -9,
        ];
        let seed = [50, 40, 30, 20, 10, 0, -10, -20];
        for term in 1..=8i8 {
            let specs = vec![(
                term,
                2,
                700,
                650,
                seed[..term as usize].to_vec(),
                seed[..term as usize].to_vec(),
            )];
            let mut enc = build_stereo_passes(&specs);
            let mut buf = pcm.to_vec();
            recorrelate_stereo(&mut enc, &mut buf).unwrap();
            let mut dec = build_stereo_passes(&specs);
            decorrelate_stereo(&mut dec, &mut buf).unwrap();
            assert_eq!(buf, pcm, "public stereo round-trip failed for term {term}");
        }
        for (term, sa, sb) in [(17i8, vec![5, 3], vec![6, 4]), (18, vec![5, 3], vec![6, 4])] {
            let specs = vec![(term, 3, 900, 800, sa, sb)];
            let mut enc = build_stereo_passes(&specs);
            let mut buf = pcm.to_vec();
            recorrelate_stereo(&mut enc, &mut buf).unwrap();
            let mut dec = build_stereo_passes(&specs);
            decorrelate_stereo(&mut dec, &mut buf).unwrap();
            assert_eq!(buf, pcm, "public stereo round-trip failed for term {term}");
        }
    }

    #[test]
    fn recorrelate_stereo_inverts_decorrelate_stereo_cross_terms() {
        let pcm = [
            100, -90, 132, 88, 90, -75, 75, 60, 210, -200, -40, 33, -33, 21, 12, -9,
        ];
        for (term, d, wa, wb, sa, sb) in [
            (-1i8, 2i32, 300i32, 400i32, 10i32, 20i32),
            (-2, 3, 350, 450, 11, 21),
            (-3, 1, 200, 250, 12, 22),
        ] {
            let specs = vec![(term, d, wa, wb, vec![sa], vec![sb])];
            let mut enc = build_stereo_passes(&specs);
            let mut buf = pcm.to_vec();
            recorrelate_stereo(&mut enc, &mut buf).unwrap();
            let mut dec = build_stereo_passes(&specs);
            decorrelate_stereo(&mut dec, &mut buf).unwrap();
            assert_eq!(
                buf, pcm,
                "public stereo cross round-trip failed for term {term}"
            );
        }
    }

    #[test]
    fn recorrelate_stereo_inverts_decorrelate_stereo_multi_pass() {
        let pcm: Vec<i32> = (0..40).map(|i| ((i * 37) % 211) - 100).collect();
        let specs = vec![
            (-3i8, 1i32, 200i32, 220i32, vec![3], vec![5]),
            (2, 2, 600, 640, vec![7, 0], vec![9, 0]),
            (18, 3, 512, 500, vec![1, 2], vec![3, 4]),
        ];
        let mut enc = build_stereo_passes(&specs);
        let mut buf = pcm.clone();
        recorrelate_stereo(&mut enc, &mut buf).unwrap();
        let mut dec = build_stereo_passes(&specs);
        decorrelate_stereo(&mut dec, &mut buf).unwrap();
        assert_eq!(buf, pcm, "public multi-pass stereo round-trip failed");
    }

    #[test]
    fn recorrelate_stereo_matches_private_forward_helper() {
        let pcm = [
            100, -90, 132, 88, 90, -75, 75, 60, 210, -200, -40, 33, -33, 21, 12, -9,
        ];
        let seed = [50, 40, 30, 20, 10, 0, -10, -20];
        for term in 1..=8i8 {
            let expected = forward_stereo(
                term,
                2,
                700,
                650,
                &seed[..term as usize],
                &seed[..term as usize],
                &pcm,
            );
            let mut enc = build_stereo_passes(&[(
                term,
                2,
                700,
                650,
                seed[..term as usize].to_vec(),
                seed[..term as usize].to_vec(),
            )]);
            let mut buf = pcm.to_vec();
            recorrelate_stereo(&mut enc, &mut buf).unwrap();
            assert_eq!(buf, expected, "recorrelate_stereo mismatch for term {term}");
        }
    }

    #[test]
    fn recorrelate_stereo_rejects_too_many_passes() {
        let mut passes: Vec<DecorrPass> = (0..MAX_NTERMS + 1)
            .map(|_| DecorrPass::new(1, 0, 0, 0, &[0], &[0]).unwrap())
            .collect();
        let mut buf = [1, 2, 3, 4];
        assert_eq!(
            recorrelate_stereo(&mut passes, &mut buf),
            Err(Error::TooManyDecorrelationPasses(MAX_NTERMS + 1))
        );
    }

    #[test]
    fn recorrelate_mono_empty_passes_is_identity() {
        let pcm = [7, -3, 42, 0, 99];
        let mut buf = pcm.to_vec();
        recorrelate_mono(&mut [], &mut buf).unwrap();
        assert_eq!(buf, pcm, "no passes must leave the buffer unchanged");
    }

    #[test]
    fn recorrelate_stereo_leaves_trailing_odd_sample() {
        // Odd-length buffer: the last unpaired sample is untouched.
        let specs = vec![(1i8, 2i32, 600i32, 640i32, vec![4], vec![5])];
        let mut enc = build_stereo_passes(&specs);
        let mut buf = vec![10, 20, 30, 40, 99];
        recorrelate_stereo(&mut enc, &mut buf).unwrap();
        assert_eq!(buf[4], 99, "trailing odd sample must be untouched");
        // Re-decode the two pairs and confirm they recover the inputs.
        let mut dec = build_stereo_passes(&specs);
        decorrelate_stereo(&mut dec, &mut buf).unwrap();
        assert_eq!(&buf[..4], &[10, 20, 30, 40]);
    }

    // ---- assemble_mono_passes (0x02/0x03/0x04 → application-ordered passes) ----

    /// Encode one spec-format `0x02` term byte: low 5 bits = `term + 5`,
    /// high 3 bits = `delta`.
    fn term_byte(term: i8, delta: u8) -> u8 {
        (((term + TERM_BYTE_BIAS) as u8) & TERM_PREDICTOR_MASK) | (delta << TERM_PREDICTOR_BITS)
    }

    /// Encode a `0x04` seed word for a small value that expands back
    /// exactly: with `exponent_byte = 9` the expander leaves the signed
    /// mantissa untouched (shift = 0), so `[v as i8 as u8, 9]` round-trips
    /// to `v` for any `v` in `-128..=127`.
    fn seed_word(v: i32) -> [u8; 2] {
        // On-wire signed 16-bit log word; test seeds stay in the
        // exactly-representable small-magnitude range so the
        // forward/inverse pair agrees by construction.
        assert_eq!(quantize_seed_sample(v), v, "test seed must be exact");
        pack_sample_word(v)
    }

    #[test]
    fn assemble_mono_reverses_wire_order() {
        // Wire stores last-applied pass first; the assembler returns
        // application order, so the first returned pass is the LAST wire
        // term. Two terms, distinct so the reversal is observable.
        let terms = vec![term_byte(1, 2), term_byte(3, 4)];
        // Mono: one weight per pass. Weight byte b expands to b*8 (no +)
        // for non-positive b; pick weights that round-trip simply.
        let weights = vec![5u8, 0x80]; // 5*8=40 (+adj), -128*8=-1024
                                       // Seeds in wire order: term 1 needs 1, term 3 needs 3 → 4 words.
        let mut samples = Vec::new();
        samples.extend_from_slice(&seed_word(7)); // term 1 seed
        for v in [11, 12, 13] {
            samples.extend_from_slice(&seed_word(v)); // term 3 seeds
        }

        let passes = assemble_mono_passes(&terms, &weights, &samples).unwrap();
        assert_eq!(passes.len(), 2);
        // Application order: term-3 pass first (it was stored last), term-1 second.
        assert_eq!(passes[0].term, 3);
        assert_eq!(passes[0].delta, 4);
        assert_eq!(passes[1].term, 1);
        assert_eq!(passes[1].delta, 2);
        // Weights are paired with their own wire term, then reversed
        // together — term-3's weight (0x80 → -1024) leads.
        assert_eq!(passes[0].weight_a, expand_weight_byte(0x80));
        assert_eq!(passes[1].weight_a, expand_weight_byte(5));
    }

    #[test]
    fn assemble_mono_end_to_end_matches_hand_built_passes() {
        // Build residuals from PCM via the forward encoder (wire/encode
        // order = last applied first), feed the assembler the matching
        // 0x02/0x03/0x04 payloads, and confirm decode reconstructs PCM.
        let pcm: Vec<i32> = (0..32).map(|i| ((i * 37) % 101) - 50).collect();
        // Application order: term 1 then term 17. Weight *bytes* chosen
        // freely; the matching app weight is whatever the expander yields,
        // so the forward encoder and the assembler agree by construction.
        let app_terms = [(1i8, 2u8), (17i8, 3u8)];
        let app_seeds: [Vec<i32>; 2] = [vec![4], vec![6, 5]];
        let app_weight_bytes = [5u8, 10u8];
        let app_weights = [
            expand_weight_byte(app_weight_bytes[0]),
            expand_weight_byte(app_weight_bytes[1]),
        ];

        // Forward (encode) applies passes in reverse application order.
        let mut buf = pcm.clone();
        for idx in (0..app_terms.len()).rev() {
            let (t, d) = app_terms[idx];
            buf = forward_mono(t, d as i32, app_weights[idx], &app_seeds[idx], &buf);
        }
        let residuals = buf;

        // Wire order = reverse of application order.
        let mut terms_payload = Vec::new();
        let mut weights_payload = Vec::new();
        let mut samples_payload = Vec::new();
        for idx in (0..app_terms.len()).rev() {
            let (t, d) = app_terms[idx];
            terms_payload.push(term_byte(t, d));
            weights_payload.push(app_weight_bytes[idx]);
            for &s in &app_seeds[idx] {
                samples_payload.extend_from_slice(&seed_word(s));
            }
        }

        let mut passes =
            assemble_mono_passes(&terms_payload, &weights_payload, &samples_payload).unwrap();
        let mut decoded = residuals.clone();
        decorrelate_mono(&mut passes, &mut decoded).unwrap();
        assert_eq!(
            decoded, pcm,
            "assembled mono decode did not reconstruct PCM"
        );
    }

    #[test]
    fn assemble_mono_rejects_cross_term() {
        // term field 4 → 4 - 5 = -1 (cross), invalid for mono.
        let terms = vec![term_byte(-1, 0)];
        assert_eq!(
            assemble_mono_passes(&terms, &[0], &[]),
            Err(Error::CrossTermOnMono(-1))
        );
    }

    #[test]
    fn assemble_mono_rejects_weight_count_mismatch() {
        let terms = vec![term_byte(1, 0), term_byte(2, 0)];
        // Two terms, one weight. (Weight count is checked before seeds.)
        assert_eq!(
            assemble_mono_passes(&terms, &[5], &[]),
            Err(Error::DecorrelationWeightCountMismatch {
                expected: 2,
                actual: 1,
            })
        );
    }

    #[test]
    fn assemble_mono_accepts_prefix_seed_payloads() {
        // The 0x04 payload primes a wire-order *prefix* of the passes
        // (round 405): real encoders store seeds for fewer passes than
        // the term list carries, and the rest start from zero history.
        // Two terms (17 then 2 on the wire), seeds for the first only.
        let terms = vec![term_byte(17, 2), term_byte(2, 1)];
        let mut seeds = Vec::new();
        seeds.extend_from_slice(&seed_word(7));
        seeds.extend_from_slice(&seed_word(-9));
        let passes = assemble_mono_passes(&terms, &[5, 6], &seeds).unwrap();
        assert_eq!(passes.len(), 2);
        // Application order reverses the wire: the primed wire-first
        // pass (term 17) is applied last.
        assert_eq!(passes[0].term, 2);
        assert_eq!(passes[1].term, 17);
        // The unprimed pass equals one built with explicit zero
        // history and the primed one carries the wire seeds.
        let unprimed = DecorrPass::new(2, 1, expand_weight_byte(6), 0, &[], &[]).unwrap();
        assert_eq!(passes[0], unprimed);
        let primed = DecorrPass::new(17, 2, expand_weight_byte(5), 0, &[7, -9], &[]).unwrap();
        assert_eq!(passes[1], primed);
        // An entirely empty seed payload is the degenerate prefix: every
        // pass starts unprimed.
        let passes = assemble_mono_passes(&terms, &[5, 6], &[]).unwrap();
        assert_eq!(passes.len(), 2);
    }

    #[test]
    fn assemble_mono_rejects_mid_term_seed_truncation() {
        // term 2 needs 2 seeds; supplying 1 stops inside the term's
        // group — malformed, not a legal prefix.
        let terms = vec![term_byte(2, 0)];
        let seeds = seed_word(3);
        assert_eq!(
            assemble_mono_passes(&terms, &[5], &seeds),
            Err(Error::DecorrelationSampleCountMismatch {
                expected: 2,
                actual: 1,
            })
        );
    }

    #[test]
    fn assemble_mono_rejects_seed_surplus() {
        // term 1 consumes 1 seed; a second seed word has no pass to
        // prime.
        let terms = vec![term_byte(1, 0)];
        let mut seeds = Vec::new();
        seeds.extend_from_slice(&seed_word(3));
        seeds.extend_from_slice(&seed_word(4));
        assert_eq!(
            assemble_mono_passes(&terms, &[5], &seeds),
            Err(Error::DecorrelationSampleCountMismatch {
                expected: 1,
                actual: 2,
            })
        );
    }

    #[test]
    fn assemble_stereo_accepts_prefix_seed_payloads() {
        // Stereo prefix rule: a pass's group is channel A's seeds then
        // channel B's; the payload may stop at a group boundary.
        let terms = vec![term_byte(18, 2), term_byte(-1, 1)];
        let mut seeds = Vec::new();
        for v in [7, -9, 11, -13] {
            seeds.extend_from_slice(&seed_word(v)); // term 18: A(2) + B(2)
        }
        let passes = assemble_stereo_passes(&terms, &[5, 6, 7, 8], &seeds).unwrap();
        assert_eq!(passes.len(), 2);
        assert_eq!(passes[0].term, -1, "unprimed cross pass applied first");
        let unprimed = DecorrPass::new(
            -1,
            1,
            expand_weight_byte(7),
            expand_weight_byte(8),
            &[],
            &[],
        )
        .unwrap();
        assert_eq!(passes[0], unprimed);
        let primed = DecorrPass::new(
            18,
            2,
            expand_weight_byte(5),
            expand_weight_byte(6),
            &[7, -9],
            &[11, -13],
        )
        .unwrap();
        assert_eq!(passes[1], primed);
        // Stopping inside a pass's A+B group (3 of 4 seeds) is refused.
        assert_eq!(
            assemble_stereo_passes(&terms, &[5, 6, 7, 8], &seeds[..6]),
            Err(Error::DecorrelationSampleCountMismatch {
                expected: 4,
                actual: 3,
            })
        );
    }

    #[test]
    fn assemble_mono_weights_without_terms_rejected() {
        assert_eq!(
            assemble_mono_passes(&[], &[5], &[]),
            Err(Error::DecorrelationTermsMissing)
        );
    }

    #[test]
    fn assemble_mono_empty_is_no_op() {
        assert_eq!(assemble_mono_passes(&[], &[], &[]).unwrap(), Vec::new());
    }

    #[test]
    fn assemble_mono_rejects_too_many_passes() {
        let terms: Vec<u8> = (0..=MAX_NTERMS).map(|_| term_byte(1, 0)).collect();
        let weights: Vec<u8> = (0..=MAX_NTERMS).map(|_| 5u8).collect();
        let mut samples = Vec::new();
        for _ in 0..=MAX_NTERMS {
            samples.extend_from_slice(&seed_word(0));
        }
        assert_eq!(
            assemble_mono_passes(&terms, &weights, &samples),
            Err(Error::TooManyDecorrelationPasses(MAX_NTERMS + 1))
        );
    }

    // ---- assemble_stereo_passes (0x02/0x03/0x04 → application-ordered passes) ----

    #[test]
    fn assemble_stereo_end_to_end_per_channel_terms() {
        // Multi-pass stereo: per-channel terms + an extrapolator. Build
        // residuals via the forward encoder (reverse application order),
        // feed the assembler the matching 0x02/0x03/0x04 payloads, and
        // confirm decode reconstructs the interleaved PCM.
        let pcm: Vec<i32> = (0..32).map(|i| ((i * 41) % 137) - 68).collect();
        // Application order: term 2 then term 17.
        let app_terms = [(2i8, 2u8), (17i8, 3u8)];
        let app_seeds_a: [Vec<i32>; 2] = [vec![7, 4], vec![6, 5]];
        let app_seeds_b: [Vec<i32>; 2] = [vec![-3, 2], vec![1, -2]];
        let app_wbytes_a = [5u8, 10u8];
        let app_wbytes_b = [8u8, 3u8];
        let app_wa: Vec<i32> = app_wbytes_a
            .iter()
            .map(|&b| expand_weight_byte(b))
            .collect();
        let app_wb: Vec<i32> = app_wbytes_b
            .iter()
            .map(|&b| expand_weight_byte(b))
            .collect();

        // Forward encode applies passes in reverse application order.
        let mut buf = pcm.clone();
        for idx in (0..app_terms.len()).rev() {
            let (t, d) = app_terms[idx];
            buf = forward_stereo(
                t,
                d as i32,
                app_wa[idx],
                app_wb[idx],
                &app_seeds_a[idx],
                &app_seeds_b[idx],
                &buf,
            );
        }
        let residuals = buf;

        // Wire order = reverse of application order. Weights per pass:
        // channel A then channel B. Seeds per pass: A's seeds then B's.
        let mut terms_payload = Vec::new();
        let mut weights_payload = Vec::new();
        let mut samples_payload = Vec::new();
        for idx in (0..app_terms.len()).rev() {
            let (t, d) = app_terms[idx];
            terms_payload.push(term_byte(t, d));
            weights_payload.push(app_wbytes_a[idx]);
            weights_payload.push(app_wbytes_b[idx]);
            for &s in &app_seeds_a[idx] {
                samples_payload.extend_from_slice(&seed_word(s));
            }
            for &s in &app_seeds_b[idx] {
                samples_payload.extend_from_slice(&seed_word(s));
            }
        }

        let mut passes =
            assemble_stereo_passes(&terms_payload, &weights_payload, &samples_payload).unwrap();
        let mut decoded = residuals.clone();
        decorrelate_stereo(&mut passes, &mut decoded).unwrap();
        assert_eq!(
            decoded, pcm,
            "assembled stereo decode did not reconstruct PCM"
        );
    }

    #[test]
    fn assemble_stereo_end_to_end_cross_term() {
        // A single cross term (-1) is valid for stereo and uses the
        // clipped weight update; round-trip through the assembler.
        let pcm: Vec<i32> = (0..24).map(|i| ((i * 29) % 91) - 45).collect();
        let app_term = (-1i8, 1u8);
        let seed_a = [9];
        let seed_b = [-4];
        let wbyte_a = 6u8;
        let wbyte_b = 7u8;
        let wa = expand_weight_byte(wbyte_a);
        let wb = expand_weight_byte(wbyte_b);

        let residuals = forward_stereo(
            app_term.0,
            app_term.1 as i32,
            wa,
            wb,
            &seed_a,
            &seed_b,
            &pcm,
        );

        // Single pass: wire == application order.
        let terms_payload = vec![term_byte(app_term.0, app_term.1)];
        let weights_payload = vec![wbyte_a, wbyte_b];
        let mut samples_payload = Vec::new();
        samples_payload.extend_from_slice(&seed_word(seed_a[0]));
        samples_payload.extend_from_slice(&seed_word(seed_b[0]));

        let mut passes =
            assemble_stereo_passes(&terms_payload, &weights_payload, &samples_payload).unwrap();
        let mut decoded = residuals.clone();
        decorrelate_stereo(&mut passes, &mut decoded).unwrap();
        assert_eq!(decoded, pcm, "assembled stereo cross-term decode failed");
    }

    #[test]
    fn assemble_stereo_reverses_wire_order() {
        // Two distinct terms; the assembler must return application order
        // (the first returned pass is the LAST wire term).
        let terms = vec![term_byte(1, 2), term_byte(3, 4)];
        // 2 weights per pass (A, B).
        let weights = vec![5u8, 6u8, 0x80u8, 7u8];
        // Seeds wire order: term 1 (1/ch → A,B = 2 words), term 3 (3/ch →
        // 6 words).
        let mut samples = Vec::new();
        for v in [1, 2] {
            samples.extend_from_slice(&seed_word(v));
        }
        for v in [3, 4, 5, 6, 7, 8] {
            samples.extend_from_slice(&seed_word(v));
        }
        let passes = assemble_stereo_passes(&terms, &weights, &samples).unwrap();
        assert_eq!(passes.len(), 2);
        // Application order: first pass is wire term 3, second is wire term 1.
        assert_eq!(passes[0].term, 3);
        assert_eq!(passes[1].term, 1);
    }

    #[test]
    fn assemble_stereo_rejects_weight_count_mismatch() {
        // One term needs 2 weights; supply 1.
        let terms = vec![term_byte(1, 0)];
        assert_eq!(
            assemble_stereo_passes(&terms, &[5], &[]),
            Err(Error::DecorrelationWeightCountMismatch {
                expected: 2,
                actual: 1,
            })
        );
    }

    #[test]
    fn assemble_stereo_rejects_seed_count_mismatch() {
        // term 1 needs 1 seed per channel = 2 words; supplying only
        // channel A's word stops inside the pass's A+B group (an empty
        // payload would be a legal zero-history prefix — round 405).
        let terms = vec![term_byte(1, 0)];
        assert_eq!(
            assemble_stereo_passes(&terms, &[5, 6], &seed_word(3)),
            Err(Error::DecorrelationSampleCountMismatch {
                expected: 2,
                actual: 1,
            })
        );
    }

    #[test]
    fn assemble_stereo_accepts_cross_term_in_terms() {
        // -1 cross term must NOT be rejected (stereo allows it).
        let terms = vec![term_byte(-1, 0)];
        let weights = vec![5u8, 6u8];
        let mut samples = Vec::new();
        samples.extend_from_slice(&seed_word(1));
        samples.extend_from_slice(&seed_word(2));
        let passes = assemble_stereo_passes(&terms, &weights, &samples).unwrap();
        assert_eq!(passes.len(), 1);
        assert_eq!(passes[0].term, -1);
    }

    #[test]
    fn assemble_stereo_empty_is_no_op() {
        assert_eq!(assemble_stereo_passes(&[], &[], &[]).unwrap(), Vec::new());
    }

    #[test]
    fn assemble_stereo_weights_without_terms_rejected() {
        assert_eq!(
            assemble_stereo_passes(&[], &[5, 6], &[]),
            Err(Error::DecorrelationTermsMissing)
        );
    }

    // ---- forward serializers (round 383) ----

    /// The §3.6 weight expansion is injective, so packing an expanded
    /// byte must return the byte for every one of the 256 stored values.
    #[test]
    fn pack_weight_byte_round_trips_every_stored_byte() {
        for byte in 0..=255u8 {
            let expanded = expand_weight_byte(byte);
            assert_eq!(
                pack_weight_byte(expanded),
                byte,
                "byte {byte} (weight {expanded}) must round-trip"
            );
        }
    }

    /// The packer is a true nearest-value quantizer: across the whole
    /// working range no stored byte is closer than the one it picks.
    #[test]
    fn pack_weight_byte_is_nearest_across_working_range() {
        for w in -1100..=1100 {
            let picked = pack_weight_byte(w);
            let picked_dist = (i64::from(expand_weight_byte(picked)) - i64::from(w)).abs();
            for candidate in 0..=255u8 {
                let dist = (i64::from(expand_weight_byte(candidate)) - i64::from(w)).abs();
                assert!(
                    picked_dist <= dist,
                    "weight {w}: picked byte {picked} (dist {picked_dist}) beaten by {candidate} (dist {dist})"
                );
            }
        }
    }

    /// Quantization is idempotent and clamps the out-of-range extremes to
    /// the documented `±1024` endpoints.
    #[test]
    fn quantize_weight_idempotent_and_clamped() {
        for w in [-5000, -1024, -1000, -17, 0, 1, 8, 500, 1000, 1024, 5000] {
            let q = quantize_weight(w);
            assert_eq!(quantize_weight(q), q, "idempotence at {w}");
        }
        assert_eq!(quantize_weight(9999), 1024);
        assert_eq!(quantize_weight(-9999), -1024);
        assert_eq!(quantize_weight(1024), 1024);
        assert_eq!(quantize_weight(-1024), -1024);
        assert_eq!(quantize_weight(0), 0);
    }

    /// Every signed-8-bit value packs verbatim at the bias exponent, and
    /// power-of-two multiples pack exactly through larger exponents.
    #[test]
    fn pack_sample_word_exact_for_representable_values() {
        // The 16-bit log word resolves every magnitude below 115
        // exactly (the staged spec §1 "exact for the small magnitudes
        // that actually occur as seeds"), symmetrically for both signs.
        for v in -114..=114i32 {
            let [lo, hi] = pack_sample_word(v);
            assert_eq!(expand_sample_word(lo, hi), v, "verbatim at {v}");
        }
        // The spec §4 worked example round-trips exactly at 1000.
        let [lo, hi] = pack_sample_word(1000);
        assert_eq!(u16::from_le_bytes([lo, hi]), 2807);
        assert_eq!(expand_sample_word(lo, hi), 1000);
        // Powers of two are exact across the range (fraction 0).
        for k in 0..=30u32 {
            let v = 1i32 << k;
            let [lo, hi] = pack_sample_word(v);
            assert_eq!(expand_sample_word(lo, hi), v, "2^{k}");
            let [lo, hi] = pack_sample_word(-v);
            assert_eq!(expand_sample_word(lo, hi), -v, "-2^{k}");
        }
    }

    /// Quantization maps a value to the nearest log-word-representable
    /// one and is idempotent; small magnitudes are untouched.
    #[test]
    fn quantize_seed_sample_quantizes_and_is_idempotent() {
        assert_eq!(quantize_seed_sample(0), 0);
        assert_eq!(quantize_seed_sample(114), 114);
        assert_eq!(quantize_seed_sample(-114), -114);
        // 115 is the first magnitude the 8-fractional-bit log table
        // cannot resolve (see the logpack round-trip test).
        assert_eq!(quantize_seed_sample(115), 114);
        assert_eq!(quantize_seed_sample(-115), -114);
        for v in [-100_000, -301, -129, 129, 301, 100_000] {
            let q = quantize_seed_sample(v);
            // Log-domain rounding stays within ~0.1% of the input
            // (staged spec §1).
            let tol = (v.unsigned_abs() / 512).max(1) as i64;
            assert!(
                (i64::from(q) - i64::from(v)).abs() <= tol,
                "quantize error at {v}: {q}"
            );
            assert_eq!(quantize_seed_sample(q), q, "idempotence at {v}");
        }
        // At the i32 ceiling the quantized magnitude rounds up past
        // i32::MAX and wraps (32-bit two's-complement arithmetic, same
        // wrapping posture as the reconstruction adds); the wrapped
        // value is itself a fixpoint.
        let q = quantize_seed_sample(i32::MAX);
        assert_eq!(quantize_seed_sample(q), q);
    }

    /// `encode_term_byte` is the exact inverse of `decode_term_byte`
    /// across every valid `(term, delta)` pair, and rejects the
    /// out-of-set / out-of-field inputs.
    #[test]
    fn encode_term_byte_inverts_decode_term_byte() {
        let valid_terms: [i8; 13] = [1, 2, 3, 4, 5, 6, 7, 8, 17, 18, -1, -2, -3];
        for &t in &valid_terms {
            for d in 0..=7i32 {
                let byte = encode_term_byte(t, d).unwrap();
                assert_eq!(decode_term_byte(byte), (t, d), "term {t} delta {d}");
            }
        }
        assert_eq!(
            encode_term_byte(0, 0),
            Err(Error::InvalidDecorrelationTerm(0))
        );
        assert_eq!(
            encode_term_byte(9, 0),
            Err(Error::InvalidDecorrelationTerm(9))
        );
        assert_eq!(encode_term_byte(1, 8), Err(Error::EncodeDeltaOutOfRange(8)));
        assert_eq!(
            encode_term_byte(1, -1),
            Err(Error::EncodeDeltaOutOfRange(-1))
        );
    }

    /// serialize → assemble is the identity on a multi-term mono pass
    /// list carrying quantized weights and seeds (fixed-lag, extrapolate).
    #[test]
    fn serialize_mono_round_trips_through_assembler() {
        let passes = vec![
            DecorrPass::new(18, 2, quantize_weight(500), 0, &[10, -20], &[]).unwrap(),
            DecorrPass::new(3, 1, quantize_weight(-300), 0, &[7, -8, 9], &[]).unwrap(),
            DecorrPass::new(17, 5, 0, 0, &[64 << 3, -128], &[]).unwrap(),
        ];
        let (t, w, s) = serialize_mono_passes(&passes).unwrap();
        assert_eq!(t.len(), 3);
        assert_eq!(w.len(), 3);
        // 2 + 3 + 2 seeds, two bytes each.
        assert_eq!(s.len(), 7 * 2);
        let rebuilt = assemble_mono_passes(&t, &w, &s).unwrap();
        assert_eq!(rebuilt, passes);
    }

    /// serialize → assemble is the identity on a stereo pass list with a
    /// cross term and distinct per-channel weights / seeds.
    #[test]
    fn serialize_stereo_round_trips_through_assembler() {
        let passes = vec![
            DecorrPass::new(
                -1,
                3,
                quantize_weight(200),
                quantize_weight(-200),
                &[5],
                &[-6],
            )
            .unwrap(),
            DecorrPass::new(
                2,
                2,
                quantize_weight(999),
                quantize_weight(1),
                &[1, 2],
                &[3, 4],
            )
            .unwrap(),
            DecorrPass::new(18, 4, 0, quantize_weight(-1024), &[100, -100], &[50, -50]).unwrap(),
        ];
        let (t, w, s) = serialize_stereo_passes(&passes).unwrap();
        assert_eq!(t.len(), 3);
        assert_eq!(w.len(), 6);
        // (1 + 2 + 2) seeds per channel, two channels, two bytes each.
        assert_eq!(s.len(), 5 * 2 * 2);
        let rebuilt = assemble_stereo_passes(&t, &w, &s).unwrap();
        assert_eq!(rebuilt, passes);
    }

    /// The other direction: assemble → serialize reproduces the exact
    /// payload bytes when the seeds are in canonical (minimal-exponent)
    /// form — pinning the wire order (reverse of application order).
    #[test]
    fn assemble_then_serialize_reproduces_canonical_payloads() {
        let terms = vec![term_byte(2, 1), term_byte(17, 3)];
        let weights = vec![10u8, 0x80u8]; // +80, -1024 after expansion
        let mut samples = Vec::new();
        // term 2 seeds (canonical: mantissa, exponent 9).
        samples.extend_from_slice(&seed_word(3));
        samples.extend_from_slice(&seed_word(-4));
        // term 17 seeds.
        samples.extend_from_slice(&seed_word(100));
        samples.extend_from_slice(&seed_word(-100));
        let passes = assemble_mono_passes(&terms, &weights, &samples).unwrap();
        let (t2, w2, s2) = serialize_mono_passes(&passes).unwrap();
        assert_eq!(t2, terms);
        assert_eq!(w2, weights);
        assert_eq!(s2, samples);
    }

    /// Serializer refusal arms: unrepresentable weight, unrepresentable
    /// seed, out-of-range delta, cross term on mono, over-long list.
    #[test]
    fn serialize_refusal_arms() {
        // Weight 5 is not a stored-byte expansion (steps are 8 or 9).
        let p = DecorrPass::new(1, 0, 5, 0, &[0], &[]).unwrap();
        assert_eq!(
            serialize_mono_passes(&[p]),
            Err(Error::EncodeWeightNotRepresentable(5))
        );
        // Seed 115 is the first magnitude the 16-bit log word cannot
        // represent exactly (it quantizes to 114 — see the logpack
        // round-trip test), so the exact serializer refuses it.
        let p = DecorrPass::new(1, 0, 0, 0, &[115], &[]).unwrap();
        assert_eq!(
            serialize_mono_passes(&[p]),
            Err(Error::EncodeSeedNotRepresentable(115))
        );
        // Delta 9 does not fit the 3-bit field.
        let p = DecorrPass::new(1, 9, 0, 0, &[0], &[]).unwrap();
        assert_eq!(
            serialize_mono_passes(&[p]),
            Err(Error::EncodeDeltaOutOfRange(9))
        );
        // Cross term on the mono serializer.
        let p = DecorrPass::new(-2, 0, 0, 0, &[0], &[0]).unwrap();
        assert_eq!(serialize_mono_passes(&[p]), Err(Error::CrossTermOnMono(-2)));
        // Over-long list (17 passes).
        let long: Vec<DecorrPass> = (0..17)
            .map(|_| DecorrPass::new(1, 0, 0, 0, &[0], &[]).unwrap())
            .collect();
        assert_eq!(
            serialize_mono_passes(&long),
            Err(Error::TooManyDecorrelationPasses(17))
        );
        assert_eq!(
            serialize_stereo_passes(&long),
            Err(Error::TooManyDecorrelationPasses(17))
        );
    }

    /// Empty pass lists serialize to three empty payloads and assemble
    /// back to the empty list.
    #[test]
    fn serialize_empty_pass_list_is_three_empty_payloads() {
        let (t, w, s) = serialize_mono_passes(&[]).unwrap();
        assert!(t.is_empty() && w.is_empty() && s.is_empty());
        let (t, w, s) = serialize_stereo_passes(&[]).unwrap();
        assert!(t.is_empty() && w.is_empty() && s.is_empty());
    }

    /// A serialized pass list drives the full forward/inverse loop pair:
    /// recorrelate with the assembled passes, decorrelate with a freshly
    /// re-assembled copy, recovering the original buffer.
    #[test]
    fn serialized_passes_drive_recorrelate_decorrelate_round_trip() {
        let passes = vec![
            DecorrPass::new(18, 2, quantize_weight(700), 0, &[12, -13], &[]).unwrap(),
            DecorrPass::new(2, 1, quantize_weight(-150), 0, &[40, -40], &[]).unwrap(),
        ];
        let (t, w, s) = serialize_mono_passes(&passes).unwrap();

        let original: Vec<i32> = (0..64).map(|i| (i * 37 % 200) - 100).collect();
        let mut buffer = original.clone();
        let mut forward = assemble_mono_passes(&t, &w, &s).unwrap();
        recorrelate_mono(&mut forward, &mut buffer).unwrap();
        assert_ne!(buffer, original, "residuals differ from PCM");

        let mut inverse = assemble_mono_passes(&t, &w, &s).unwrap();
        decorrelate_mono(&mut inverse, &mut buffer).unwrap();
        assert_eq!(buffer, original);
    }

    /// Fuzz regression (round 386): adversarial metadata can seed a
    /// term-17/18 pass with near-`i32`-extreme history, so the §3.2
    /// extrapolator predictors (`2*a0 - a1`, `(3*a0 - a1) >> 1`) must
    /// use 32-bit wrapping arithmetic like every other reconstruction
    /// step — the pre-fix build tripped a debug multiply overflow.
    /// Both directions use the identical wrapping forms, so the
    /// forward/inverse identity must survive the extremes.
    #[test]
    fn extrapolator_predictors_wrap_on_extreme_history() {
        let seeds = [i32::MIN, i32::MAX];
        for term in [17i8, 18] {
            // Mono, both directions, round trip through the wrap.
            let mut inv = DecorrPass::new(term, 2, 1024, 0, &seeds, &[]).unwrap();
            let mut buf = [7i32, -3, 0, 11];
            decorrelate_mono(std::slice::from_mut(&mut inv), &mut buf).unwrap();
            let mut fwd = DecorrPass::new(term, 2, 1024, 0, &seeds, &[]).unwrap();
            recorrelate_mono(std::slice::from_mut(&mut fwd), &mut buf).unwrap();
            assert_eq!(buf, [7, -3, 0, 11]);

            // Stereo twin (independent per-channel histories).
            let mut inv = DecorrPass::new(term, 2, 1024, -1024, &seeds, &seeds).unwrap();
            let mut buf = [5i32, -9, 2, 4];
            decorrelate_stereo(std::slice::from_mut(&mut inv), &mut buf).unwrap();
            let mut fwd = DecorrPass::new(term, 2, 1024, -1024, &seeds, &seeds).unwrap();
            recorrelate_stereo(std::slice::from_mut(&mut fwd), &mut buf).unwrap();
            assert_eq!(buf, [5, -9, 2, 4]);
        }
    }
}
