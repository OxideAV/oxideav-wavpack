//! WavPack hybrid-mode correction-fold arithmetic (decorrelation-spec §4.1).
//!
//! WavPack *hybrid* mode (wiki flag bit 3, `HYBRID_FLAG` `0x08`) splits the
//! signal into a **lossy main stream** (`0x0A`) plus an optional
//! **correction stream** (`0x0B`, normally the `.wvc` companion file). When
//! the correction stream is present the decoder reconstructs the original
//! samples *losslessly* by reading **two** residuals per sample position:
//! the lossy value from `0x0A` and a correction value from `0x0B`. The
//! staged clean-room trace
//! (`docs/audio/wavpack/spec/wavpack-decorrelation.md` §4.1) describes the
//! *fold* — where in the per-sample pipeline the correction value is added
//! to the reconstructed lossy value to recover the exact original.
//!
//! This module implements exactly that documented fold arithmetic as typed
//! primitives. The fold is a pure consumer of two already-decoded integer
//! values (one lossy main value, one correction value) — it does **not**
//! decode either entropy stream and does not depend on the (undocumented)
//! lossy main-stream `error_limit` derivation or the `read_shaping_info`
//! layout. Those gaps block the full hybrid *decode path*; the fold step
//! they feed is fully specified, so it is lifted here onto the public
//! surface the same way every other documented spec operation in this
//! crate is (the §3 weight arithmetic, the §4.2 entropy ladder, the §5 CRC
//! steps): a small, pinned, exact building block the consuming path drives
//! when the remaining gaps close.
//!
//! ## Where the fold sits (spec §4.1)
//!
//! The spec documents three placements keyed on the channel layout and the
//! `CROSS_DECORR` (`0x20`) flag:
//!
//! * **Mono / non-cross** ([`fold_correction`]): the decorrelation passes
//!   run on the lossy value, and the correction is then **added to the
//!   reconstructed lossy sample** to recover the exact original — a
//!   *post-decorrelation* fold. The spec writes this as
//!   `read_word += correction[0]`.
//! * **Stereo *without* `CROSS_DECORR`** ([`fold_correction_pair`]):
//!   decorrelation runs on the lossy left/right first, then the
//!   per-channel corrections are added afterward — the post-decorrelation
//!   fold applied to each channel of the pair.
//! * **Stereo *with* `CROSS_DECORR`** ([`fold_correction_pre_decorrelation`]
//!   / [`fold_correction_pre_decorrelation_pair`]): the correction is
//!   folded in *before* the decorrelation passes (a "no-delay" correction)
//!   — the lossy left/right *plus* their corrections form the inputs, then
//!   decorrelation runs to yield the lossless sample.
//!
//! The arithmetic of the fold itself is identical in all three cases
//! (`value + correction`); the placements differ only in *which* value the
//! correction is added to (the reconstructed output vs. the pre-decorr
//! input). The typed names above keep the two pipeline positions distinct
//! so a consuming decoder cannot accidentally apply a post-decorrelation
//! fold where the `CROSS_DECORR` pre-decorrelation fold is required (or
//! vice-versa).
//!
//! ## Encoder inverse (forward direction)
//!
//! The forward (encode) direction is the exact arithmetic inverse:
//! [`split_correction`] computes `correction = original - lossy` so that
//! `lossy + correction == original`. This is the value the encoder packs
//! into the `0x0B` correction stream; it pairs the lossy main value the
//! encoder emitted with the original sample to recover the residual the
//! decoder's [`fold_correction`] consumes.
//!
//! ## Not covered (documented gaps)
//!
//! The **noise-shaping** variant (`HYBRID_SHAPE` `0x40` / `NEW_SHAPING`
//! `0x20000000`, spec §4.1) replaces the raw add with a first-order
//! error-feedback filter whose per-channel shaping weight/state come from
//! the `read_shaping_info` metadata — a layout the staged docs name but do
//! not transcribe. The raw-add fold here is the `HYBRID_SHAPE`-clear case
//! (the common hybrid mode); the shaped variant stays a documented gap.

/// The `HYBRID_FLAG` bit (`0x08`, spec §6): the block is a hybrid (lossy
/// main + optional correction) block.
pub const HYBRID_FLAG: u32 = 0x08;

/// The `CROSS_DECORR` bit (`0x20`, spec §6): the hybrid-stereo correction
/// is folded *before* the decorrelation passes (a zero-delay correction),
/// rather than added to the reconstructed output afterward.
pub const CROSS_DECORR_FLAG: u32 = 0x20;

/// The `HYBRID_SHAPE` bit (`0x40`, spec §6): the correction is applied
/// through a first-order error-feedback (noise-shaping) filter rather than
/// added raw. The shaped fold is **not** implemented here (its
/// `read_shaping_info` state layout is a documented gap); this constant
/// names the flag so a consumer can detect and refuse the shaped case
/// before reaching the raw-add fold.
pub const HYBRID_SHAPE_FLAG: u32 = 0x40;

/// The `NEW_SHAPING` bit (`0x2000_0000`, spec §6): selects the IIR
/// negative-shaping variant of the [`HYBRID_SHAPE_FLAG`] error-feedback
/// filter. Like `HYBRID_SHAPE`, the shaped fold is not implemented here.
pub const NEW_SHAPING_FLAG: u32 = 0x2000_0000;

/// Fold one correction residual into a reconstructed lossy sample
/// (spec §4.1, the post-decorrelation fold).
///
/// Recovers the exact original sample from the decoder's reconstructed
/// **lossy** value and the matching correction residual read from the
/// `0x0B` stream:
///
/// ```text
/// original = reconstructed + correction
/// ```
///
/// The spec writes this as `read_word += correction[0]` after the
/// decorrelation passes have produced `read_word` (the reconstructed lossy
/// sample). This is the mono fold and the per-channel fold of a stereo
/// block *without* `CROSS_DECORR`.
///
/// The add uses [`i32::wrapping_add`]: the reconstructed value and the
/// correction are both decoded as 32-bit two's-complement integers and
/// the canonical decoder folds them in a 32-bit register, so a sum past
/// `i32::MAX`/`i32::MIN` wraps rather than panicking — matching the
/// wrap-around arithmetic the rest of the decode pipeline uses.
#[inline]
#[must_use]
pub fn fold_correction(reconstructed: i32, correction: i32) -> i32 {
    reconstructed.wrapping_add(correction)
}

/// Fold a pair of correction residuals into a reconstructed lossy stereo
/// pair (spec §4.1, the stereo post-decorrelation fold *without*
/// `CROSS_DECORR`).
///
/// Applies [`fold_correction`] to each channel of the reconstructed
/// `(left, right)` pair with its matching correction, returning the
/// recovered lossless `(left, right)`. This is the placement the spec
/// documents for a hybrid stereo block whose `CROSS_DECORR` flag is
/// **clear**: decorrelation runs on the lossy values first, then the
/// per-channel corrections are added afterward.
#[inline]
#[must_use]
pub fn fold_correction_pair(reconstructed: (i32, i32), correction: (i32, i32)) -> (i32, i32) {
    (
        fold_correction(reconstructed.0, correction.0),
        fold_correction(reconstructed.1, correction.1),
    )
}

/// Fold one correction residual into a lossy *input* sample before the
/// decorrelation passes (spec §4.1, the `CROSS_DECORR` pre-decorrelation
/// fold).
///
/// For a hybrid stereo block with `CROSS_DECORR` (`0x20`) set, the spec
/// folds the correction in *before* decorrelation — the lossy input plus
/// its correction becomes the decorrelation pass input:
///
/// ```text
/// input = lossy + correction
/// ```
///
/// Arithmetically the add is identical to [`fold_correction`]; the
/// distinct name marks the *pipeline position* (the value here is the
/// pre-decorrelation input, not the reconstructed output). Keeping the two
/// names separate prevents a consumer from folding a correction in the
/// wrong stage for a given `CROSS_DECORR` setting.
#[inline]
#[must_use]
pub fn fold_correction_pre_decorrelation(lossy: i32, correction: i32) -> i32 {
    lossy.wrapping_add(correction)
}

/// Fold a pair of correction residuals into a lossy *input* pair before
/// the decorrelation passes (spec §4.1, the `CROSS_DECORR` stereo
/// pre-decorrelation fold).
///
/// Applies [`fold_correction_pre_decorrelation`] to each channel of the
/// lossy `(left, right)` input pair. The resulting pair is the input the
/// stereo decorrelation passes consume when `CROSS_DECORR` is set.
#[inline]
#[must_use]
pub fn fold_correction_pre_decorrelation_pair(
    lossy: (i32, i32),
    correction: (i32, i32),
) -> (i32, i32) {
    (
        fold_correction_pre_decorrelation(lossy.0, correction.0),
        fold_correction_pre_decorrelation(lossy.1, correction.1),
    )
}

/// Compute the correction residual the encoder packs into the `0x0B`
/// stream (the forward / encode inverse of [`fold_correction`]).
///
/// ```text
/// correction = original - lossy
/// ```
///
/// so that `fold_correction(lossy, split_correction(original, lossy)) ==
/// original`. The subtraction wraps in 32 bits, mirroring the decoder's
/// wrapping add: the encoder forms the correction in the same register
/// width the decoder folds it back through.
#[inline]
#[must_use]
pub fn split_correction(original: i32, lossy: i32) -> i32 {
    original.wrapping_sub(lossy)
}

/// `true` when the flag word selects the **noise-shaped** correction fold
/// (`HYBRID_SHAPE` or `NEW_SHAPING`), which this module does not implement.
///
/// A hybrid consumer should test this before applying the raw-add fold:
/// when it is `true` the correction must be applied through the
/// error-feedback filter (spec §4.1), whose `read_shaping_info` state
/// layout is a documented gap — so the raw add here would produce wrong
/// samples. The raw-add fold is correct only when this returns `false`.
#[inline]
#[must_use]
pub fn flags_select_shaping(flags: u32) -> bool {
    flags & (HYBRID_SHAPE_FLAG | NEW_SHAPING_FLAG) != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- constants ---------------------------------------------------

    #[test]
    fn flag_constants_match_spec_section_6() {
        assert_eq!(HYBRID_FLAG, 0x08);
        assert_eq!(CROSS_DECORR_FLAG, 0x20);
        assert_eq!(HYBRID_SHAPE_FLAG, 0x40);
        assert_eq!(NEW_SHAPING_FLAG, 0x2000_0000);
    }

    // ---- fold / split round trip (the §4.1 lossless recovery) --------

    #[test]
    fn fold_recovers_original_from_lossy_and_correction() {
        // The defining property: lossy + (original - lossy) == original.
        for original in [-1000i32, -7, -1, 0, 1, 5, 1000] {
            for lossy in [-2000i32, -3, 0, 4, 2000] {
                let correction = split_correction(original, lossy);
                assert_eq!(fold_correction(lossy, correction), original);
            }
        }
    }

    #[test]
    fn fold_is_plain_addition() {
        assert_eq!(fold_correction(10, 3), 13);
        assert_eq!(fold_correction(10, -3), 7);
        assert_eq!(fold_correction(-10, 3), -7);
        assert_eq!(fold_correction(0, 0), 0);
    }

    #[test]
    fn split_is_plain_subtraction() {
        assert_eq!(split_correction(13, 10), 3);
        assert_eq!(split_correction(7, 10), -3);
        assert_eq!(split_correction(0, 0), 0);
    }

    #[test]
    fn zero_correction_is_the_identity() {
        // A hybrid block with a perfectly-lossless main stream carries a
        // zero correction; the fold must leave the reconstructed value
        // untouched.
        for v in [-5i32, 0, 5, i32::MIN, i32::MAX] {
            assert_eq!(fold_correction(v, 0), v);
            assert_eq!(fold_correction_pre_decorrelation(v, 0), v);
        }
    }

    // ---- wrap-around behaviour ---------------------------------------

    #[test]
    fn fold_wraps_past_i32_bounds_like_the_decoder_register() {
        // The decoder folds in a 32-bit register; an overshoot wraps
        // rather than panicking (debug builds would panic on a plain +).
        assert_eq!(fold_correction(i32::MAX, 1), i32::MIN);
        assert_eq!(fold_correction(i32::MIN, -1), i32::MAX);
    }

    #[test]
    fn split_wraps_past_i32_bounds() {
        assert_eq!(split_correction(i32::MIN, 1), i32::MAX);
        assert_eq!(split_correction(i32::MAX, -1), i32::MIN);
    }

    #[test]
    fn fold_and_split_round_trip_at_the_extremes() {
        // Even where the intermediate correction wraps, fold ∘ split is
        // the identity in the 32-bit register.
        for original in [i32::MIN, i32::MIN + 1, -1, 0, 1, i32::MAX - 1, i32::MAX] {
            for lossy in [i32::MIN, -1, 0, 1, i32::MAX] {
                let c = split_correction(original, lossy);
                assert_eq!(fold_correction(lossy, c), original);
            }
        }
    }

    // ---- pre-decorrelation fold (CROSS_DECORR) -----------------------

    #[test]
    fn pre_decorrelation_fold_matches_post_arithmetic() {
        // The pre- and post-decorrelation folds are the same add; only
        // the pipeline position the typed name marks differs.
        for a in [-100i32, -1, 0, 7, 100] {
            for b in [-50i32, 0, 3, 50] {
                assert_eq!(
                    fold_correction_pre_decorrelation(a, b),
                    fold_correction(a, b)
                );
            }
        }
    }

    // ---- pair folds --------------------------------------------------

    #[test]
    fn pair_fold_applies_per_channel() {
        let reconstructed = (10i32, -20i32);
        let correction = (3i32, 5i32);
        assert_eq!(fold_correction_pair(reconstructed, correction), (13, -15));
    }

    #[test]
    fn pre_decorrelation_pair_fold_applies_per_channel() {
        let lossy = (100i32, -200i32);
        let correction = (-1i32, 2i32);
        assert_eq!(
            fold_correction_pre_decorrelation_pair(lossy, correction),
            (99, -198)
        );
    }

    #[test]
    fn pair_fold_recovers_original_pair() {
        let original = (1234i32, -5678i32);
        let lossy = (1200i32, -5700i32);
        let correction = (
            split_correction(original.0, lossy.0),
            split_correction(original.1, lossy.1),
        );
        assert_eq!(fold_correction_pair(lossy, correction), original);
    }

    // ---- shaping detection -------------------------------------------

    #[test]
    fn shaping_detection_keys_on_the_shape_bits() {
        assert!(!flags_select_shaping(0));
        assert!(!flags_select_shaping(HYBRID_FLAG));
        assert!(!flags_select_shaping(CROSS_DECORR_FLAG));
        assert!(flags_select_shaping(HYBRID_SHAPE_FLAG));
        assert!(flags_select_shaping(NEW_SHAPING_FLAG));
        assert!(flags_select_shaping(HYBRID_SHAPE_FLAG | NEW_SHAPING_FLAG));
        // Mixed with unrelated bits set: still detected.
        assert!(flags_select_shaping(HYBRID_FLAG | HYBRID_SHAPE_FLAG));
    }
}
