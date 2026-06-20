//! WavPack final-normalization "shift fixup" stage.
//!
//! The last stage of a WavPack block decode — after entropy decode, after
//! the decorrelation prediction loop, and after the joint-stereo undo —
//! is a per-sample *normalization fixup* that rescales the reconstructed
//! samples to the container width. This module implements the **left-shift**
//! fixup the wiki "Flags meaning" listing of
//! `docs/audio/wavpack/wiki/WavPack.wiki` documents on flag bits 13..=17:
//!
//! > bits 13-17 - left-shift places when bitdepth is not a multiple of 8
//! >              (e.g. 12-bit, 20-bit)
//!
//! When a stream's effective bit-depth is not a whole number of bytes
//! (12-bit, 20-bit, …), the encoder reduces every sample to that narrower
//! magnitude before coding and records the number of trailing zero bits it
//! dropped in this field. The decoder reconstructs the narrow magnitude
//! through the prediction loop and then shifts it **left** by `left_shift`
//! places to restore the original container-scaled PCM value. For the
//! common whole-byte depths (8 / 16 / 24 / 32-bit) `left_shift` is `0` and
//! the fixup is the identity.
//!
//! ## Where the fixup sits relative to the CRC (pipeline order)
//!
//! The staged decorrelation/CRC trace
//! (`docs/audio/wavpack/spec/wavpack-decorrelation.md` §1) lists the
//! per-block decode order as:
//!
//! > … run each decorrelation pass → apply joint-stereo undo →
//! > accumulate CRC → shift/clip fixups → compare CRC.
//!
//! and §5.2 states the mono CRC step folds each sample "after
//! decorrelation, **before final shift**". So the running block CRC is
//! computed over the **pre-shift** samples; the left-shift fixup is applied
//! **after** the CRC fold, producing the final emitted PCM. Callers that
//! both verify the CRC and emit shifted PCM must therefore fold the CRC
//! over the buffer this module has **not** yet touched, then apply the
//! fixup. (Evidence: decorrelation-spec §1 pipeline ordering and §5.2
//! "before final shift".)
//!
//! ## Arithmetic
//!
//! The fixup is a single arithmetic left shift per sample:
//!
//! ```text
//! pcm = sample << left_shift
//! ```
//!
//! Shifting a two's-complement `i32` left preserves the sign for in-range
//! magnitudes, matching the encoder which produced the narrow value by an
//! arithmetic right shift of the original. A `left_shift` of `0` (the
//! whole-byte-depth case) leaves the sample unchanged. A `left_shift`
//! large enough to push bits past bit 31 would overflow; the wiki bounds
//! the field to 5 bits (0..=31) and a conformant stream never shifts past
//! the container width, but to stay panic-free on a malformed header this
//! module uses a wrapping shift so a hostile `left_shift` truncates rather
//! than aborts the decode.

/// Applies the WavPack left-shift fixup to a single reconstructed sample
/// (wiki flag bits 13..=17).
///
/// Returns `sample << left_shift`, restoring the original container-scaled
/// PCM value from the narrow magnitude the prediction loop reconstructed.
/// A `left_shift` of `0` (whole-byte depth) is the identity.
///
/// The shift is performed on the two's-complement `i32` with wrap-around
/// (`i32::wrapping_shl`) so a malformed header that records a shift count
/// at or beyond 32 truncates the value instead of panicking; a conformant
/// stream keeps `left_shift` within the container width and never reaches
/// that case.
#[inline]
#[must_use]
pub fn apply_left_shift(sample: i32, left_shift: u8) -> i32 {
    sample.wrapping_shl(left_shift as u32)
}

/// Applies the WavPack left-shift fixup in place to every sample of a
/// decoded buffer (wiki flag bits 13..=17).
///
/// Each element is replaced by `element << left_shift` via
/// [`apply_left_shift`]. This is the whole-buffer form the block decoder
/// runs after the CRC has been folded over the pre-shift samples (see the
/// module-level pipeline-order note). A `left_shift` of `0` leaves the
/// buffer untouched, so the decoder may call this unconditionally without a
/// special-case branch for whole-byte depths.
#[inline]
pub fn apply_left_shift_buffer(samples: &mut [i32], left_shift: u8) {
    if left_shift == 0 {
        return;
    }
    for s in samples.iter_mut() {
        *s = apply_left_shift(*s, left_shift);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_shift_is_identity() {
        for s in [-100_000i32, -1, 0, 1, 7, 100_000, i32::MIN, i32::MAX] {
            assert_eq!(apply_left_shift(s, 0), s);
        }
    }

    #[test]
    fn shift_scales_by_power_of_two() {
        // A narrow magnitude of 5 with left_shift 4 (12-bit-in-16-bit
        // container) restores to 5 * 16 = 80.
        assert_eq!(apply_left_shift(5, 4), 80);
        assert_eq!(apply_left_shift(1, 1), 2);
        assert_eq!(apply_left_shift(3, 8), 768);
    }

    #[test]
    fn shift_preserves_sign_for_in_range_values() {
        // Negative narrow magnitudes scale up while staying negative.
        assert_eq!(apply_left_shift(-5, 4), -80);
        assert_eq!(apply_left_shift(-1, 1), -2);
        assert_eq!(apply_left_shift(-3, 8), -768);
    }

    #[test]
    fn shift_is_inverse_of_encoder_right_shift() {
        // The encoder forms the narrow magnitude by an arithmetic right
        // shift of the original; the decoder's left shift recovers exactly
        // the original whenever the dropped low bits were zero (which the
        // encoder guarantees for this code path).
        for left_shift in 0u8..=12 {
            for original in [-8192i32, -64, 0, 64, 4096, 1_000_000] {
                // Zero out the low `left_shift` bits so the value is an
                // exact multiple of 2^left_shift (the encoder precondition).
                let mask = !((1i32 << left_shift) - 1);
                let aligned = original & mask;
                let narrow = aligned >> left_shift;
                assert_eq!(apply_left_shift(narrow, left_shift), aligned);
            }
        }
    }

    #[test]
    fn large_shift_wraps_instead_of_panicking() {
        // A malformed header could record a shift >= 32. wrapping_shl must
        // not panic; the exact truncated value is implementation-defined
        // but the call must return.
        let _ = apply_left_shift(1, 32);
        let _ = apply_left_shift(-1, 33);
        let _ = apply_left_shift(i32::MAX, 31);
    }

    #[test]
    fn buffer_zero_shift_leaves_buffer_untouched() {
        let mut buf = [1i32, -2, 3, -4, 5];
        let original = buf;
        apply_left_shift_buffer(&mut buf, 0);
        assert_eq!(buf, original);
    }

    #[test]
    fn buffer_applies_shift_to_every_element() {
        let mut buf = [1i32, -2, 3, -4, 5];
        apply_left_shift_buffer(&mut buf, 3);
        assert_eq!(buf, [8, -16, 24, -32, 40]);
    }

    #[test]
    fn buffer_matches_per_element_application() {
        let original = [7i32, -11, 0, 123, -456, 789];
        let mut buf = original;
        apply_left_shift_buffer(&mut buf, 5);
        for (i, &o) in original.iter().enumerate() {
            assert_eq!(buf[i], apply_left_shift(o, 5));
        }
    }

    #[test]
    fn empty_buffer_is_a_noop() {
        let mut buf: [i32; 0] = [];
        apply_left_shift_buffer(&mut buf, 7);
        assert_eq!(buf, [] as [i32; 0]);
    }
}
