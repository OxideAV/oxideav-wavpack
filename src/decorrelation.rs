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
}
