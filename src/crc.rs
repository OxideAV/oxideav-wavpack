//! WavPack per-block running CRC over decoded PCM samples.
//!
//! Each WavPack block carries a 32-bit CRC over its decoded samples. The
//! decoder recomputes the CRC as it emits samples and, at the block's last
//! sample, compares it against the stored block CRC
//! ([`crate::WavPackBlockHeader::crc`]); a mismatch marks the block as
//! corrupt (the reference decoder zeroes/mutes the affected buffer).
//!
//! This module implements exactly the documented accumulation arithmetic
//! from the staged spec
//! (`docs/audio/wavpack/spec/wavpack-decorrelation.md` §5) — the running
//! CRC formulas, the seed, the joint-stereo undo that precedes the stereo
//! step, and the block-end comparison. It is a pure consumer of *already
//! decoded* PCM and does not touch the entropy or decorrelation stages.
//!
//! ## Arithmetic (spec §5)
//!
//! The CRC is a multiply-accumulate over a 32-bit register that wraps
//! (the reference uses unsigned 32-bit wrap-around, so all math here is
//! [`u32::wrapping_add`] / [`u32::wrapping_mul`] on the register, with
//! samples folded in as their two's-complement `u32` bit pattern):
//!
//! * **Seed** (§5.1): the register is initialised to `0xffff_ffff` at
//!   block open.
//! * **Mono step** (§5.2): per decoded mono sample `s`,
//!   `crc = crc * 3 + s`.
//! * **Stereo step** (§5.3): per decoded stereo pair `(L, R)` (after the
//!   joint-stereo undo below),
//!   `crc = crc * 9 + 3 * L + R`
//!   — equivalently `crc + (crc << 3) + (L << 1) + L + R`.
//! * **Joint-stereo undo** (§5.4): when the block's `JOINT_STEREO`
//!   (`0x10`) flag is set, mid/side is undone per pair *before* the
//!   stereo CRC step:
//!   `R -= L >> 1; L += R;`
//!   (an arithmetic right shift of the left/mid channel).
//! * **Block-end verification** (§5.6): compare the accumulated register
//!   against the stored block CRC; equal means the decoded samples match.
//!
//! The `s`, `L`, `R` samples are folded in as `u32` two's-complement bit
//! patterns (`sample as u32`), matching the reference register where the
//! signed sample is added to an unsigned 32-bit accumulator.
//!
//! The extension CRC (`crc_x`, spec §5.5) covers the `0x0C` wide/float
//! extension stream and lives on a separate register; it is decode-stage
//! work gated on the extension consumer and is not implemented here.

/// The CRC register seed used at block open (spec §5.1): `0xffff_ffff`.
pub const CRC_INIT: u32 = 0xffff_ffff;

/// The `JOINT_STEREO` flag bit (`0x10`, spec §5.4 / §6) that selects the
/// mid/side undo applied before the stereo CRC step.
pub const JOINT_STEREO_FLAG: u32 = 0x10;

/// Folds one decoded mono sample into a running CRC register (spec §5.2).
///
/// `crc' = crc * 3 + s`, with the sample taken as its two's-complement
/// `u32` bit pattern and the whole expression evaluated with 32-bit
/// wrap-around.
#[inline]
#[must_use]
pub fn update_mono(crc: u32, sample: i32) -> u32 {
    crc.wrapping_mul(3).wrapping_add(sample as u32)
}

/// Folds one decoded stereo pair into a running CRC register (spec §5.3).
///
/// `crc' = crc * 9 + 3 * L + R`, with both samples taken as their
/// two's-complement `u32` bit patterns and the whole expression evaluated
/// with 32-bit wrap-around. The reference writes this as
/// `crc + (crc << 3) + (L << 1) + L + R`, which is algebraically the same
/// (`crc + 8*crc = 9*crc`, `2*L + L = 3*L`).
///
/// The pair must already have had the joint-stereo undo applied (see
/// [`undo_joint_stereo`]) when the block's [`JOINT_STEREO_FLAG`] is set;
/// the spec applies that undo *before* this step.
#[inline]
#[must_use]
pub fn update_stereo(crc: u32, left: i32, right: i32) -> u32 {
    crc.wrapping_mul(9)
        .wrapping_add((left as u32).wrapping_mul(3))
        .wrapping_add(right as u32)
}

/// Undoes WavPack joint (mid/side) stereo for one decoded pair (spec §5.4).
///
/// Maps the stored `(mid, side)` pair `(L, R)` back to true `(left, right)`:
///
/// ```text
/// R -= L >> 1;   // recover the difference (side)
/// L += R;        // recover the left channel
/// ```
///
/// The shift is an arithmetic right shift (Rust `>>` on `i32` is
/// arithmetic), matching the reference's signed `>>`. Returns the
/// recovered `(left, right)`. Only invoke this when the block's
/// [`JOINT_STEREO_FLAG`] is set.
#[inline]
#[must_use]
pub fn undo_joint_stereo(mid: i32, side: i32) -> (i32, i32) {
    let right = side.wrapping_sub(mid >> 1);
    let left = mid.wrapping_add(right);
    (left, right)
}

/// A running WavPack block CRC accumulator (spec §5).
///
/// Seed with [`BlockCrc::new`], fold decoded samples in with
/// [`BlockCrc::push_mono`] / [`BlockCrc::push_stereo_pair`] /
/// [`BlockCrc::push_joint_stereo_pair`], then read the register with
/// [`BlockCrc::value`] or compare it against the stored block CRC with
/// [`BlockCrc::matches`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockCrc {
    crc: u32,
}

impl Default for BlockCrc {
    fn default() -> Self {
        Self::new()
    }
}

impl BlockCrc {
    /// Creates a fresh accumulator seeded to [`CRC_INIT`] (spec §5.1).
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self { crc: CRC_INIT }
    }

    /// Folds one decoded mono sample (spec §5.2).
    #[inline]
    pub fn push_mono(&mut self, sample: i32) {
        self.crc = update_mono(self.crc, sample);
    }

    /// Folds one decoded stereo pair *already in true L/R form* (spec §5.3).
    ///
    /// Use this for blocks without the joint-stereo flag; for joint-stereo
    /// blocks use [`BlockCrc::push_joint_stereo_pair`], which undoes
    /// mid/side first.
    #[inline]
    pub fn push_stereo_pair(&mut self, left: i32, right: i32) {
        self.crc = update_stereo(self.crc, left, right);
    }

    /// Folds one decoded mid/side stereo pair, undoing joint stereo first
    /// (spec §5.4 then §5.3).
    ///
    /// Equivalent to [`undo_joint_stereo`] followed by
    /// [`BlockCrc::push_stereo_pair`]; returns the recovered true `(left,
    /// right)` so the caller can also use them for output.
    #[inline]
    pub fn push_joint_stereo_pair(&mut self, mid: i32, side: i32) -> (i32, i32) {
        let (left, right) = undo_joint_stereo(mid, side);
        self.crc = update_stereo(self.crc, left, right);
        (left, right)
    }

    /// The current CRC register value (spec §5.6 comparison input).
    #[inline]
    #[must_use]
    pub fn value(self) -> u32 {
        self.crc
    }

    /// Whether the accumulated CRC equals a stored block CRC (spec §5.6).
    ///
    /// A `true` result means the decoded samples folded so far reproduce
    /// the CRC the block header recorded; the reference decoder mutes the
    /// block on a `false` result.
    #[inline]
    #[must_use]
    pub fn matches(self, stored: u32) -> bool {
        self.crc == stored
    }
}

/// Computes the block CRC of a slice of decoded mono samples (spec §5.2).
///
/// Seeds with [`CRC_INIT`] and folds every sample in order. Equivalent to
/// a fresh [`BlockCrc`] driven by [`BlockCrc::push_mono`].
#[must_use]
pub fn crc_mono(samples: &[i32]) -> u32 {
    let mut crc = CRC_INIT;
    for &s in samples {
        crc = update_mono(crc, s);
    }
    crc
}

/// Computes the block CRC of interleaved decoded stereo samples in true
/// L/R form (spec §5.3).
///
/// `samples` is interleaved `[L0, R0, L1, R1, ...]`; a trailing odd
/// element (an incomplete pair) is ignored, matching the reference's
/// pair-at-a-time accumulation. For joint-stereo blocks the caller must
/// undo mid/side first (see [`crc_joint_stereo_interleaved`]).
#[must_use]
pub fn crc_stereo_interleaved(samples: &[i32]) -> u32 {
    let mut crc = CRC_INIT;
    for pair in samples.chunks_exact(2) {
        crc = update_stereo(crc, pair[0], pair[1]);
    }
    crc
}

/// Computes the block CRC of interleaved decoded mid/side stereo samples,
/// undoing joint stereo per pair first (spec §5.4 then §5.3).
///
/// `samples` is interleaved `[M0, S0, M1, S1, ...]`; a trailing odd
/// element is ignored.
#[must_use]
pub fn crc_joint_stereo_interleaved(samples: &[i32]) -> u32 {
    let mut crc = CRC_INIT;
    for pair in samples.chunks_exact(2) {
        let (left, right) = undo_joint_stereo(pair[0], pair[1]);
        crc = update_stereo(crc, left, right);
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- spec §7 derived test vectors -------------------------------

    #[test]
    fn mono_crc_matches_spec_section_7_vector() {
        // spec §7: mono CRC of residuals [3,-2,5,0,-7] from seed
        // 0xffffffff is 0xfffffff0.
        assert_eq!(crc_mono(&[3, -2, 5, 0, -7]), 0xffff_fff0);
    }

    #[test]
    fn stereo_crc_matches_spec_section_7_vector() {
        // spec §7: stereo CRC of pairs [(3,-2),(5,0),(-7,9)] from seed
        // 0xffffffff is 0xffffffd9.
        assert_eq!(crc_stereo_interleaved(&[3, -2, 5, 0, -7, 9]), 0xffff_ffd9);
    }

    // ---- seed ------------------------------------------------------

    #[test]
    fn fresh_accumulator_holds_the_seed() {
        assert_eq!(BlockCrc::new().value(), CRC_INIT);
        assert_eq!(CRC_INIT, 0xffff_ffff);
    }

    #[test]
    fn empty_inputs_yield_the_seed() {
        assert_eq!(crc_mono(&[]), CRC_INIT);
        assert_eq!(crc_stereo_interleaved(&[]), CRC_INIT);
        assert_eq!(crc_joint_stereo_interleaved(&[]), CRC_INIT);
    }

    // ---- mono step -------------------------------------------------

    #[test]
    fn mono_step_is_crc_times_three_plus_sample() {
        // crc*3 + s with 32-bit wrap; check against a hand value.
        let crc = update_mono(CRC_INIT, 3);
        // 0xffffffff * 3 = 0x2_ffff_fffd, truncated to 0xffff_fffd, + 3 = 0.
        assert_eq!(crc, 0x0000_0000);
    }

    #[test]
    fn mono_step_folds_negative_samples_as_twos_complement() {
        // crc=1, s=-1: 1*3 + 0xffff_ffff = 3 + 0xffff_ffff wraps to 2.
        assert_eq!(update_mono(1, -1), 2);
    }

    #[test]
    fn mono_accumulator_matches_free_function() {
        let mut acc = BlockCrc::new();
        for &s in &[3i32, -2, 5, 0, -7] {
            acc.push_mono(s);
        }
        assert_eq!(acc.value(), crc_mono(&[3, -2, 5, 0, -7]));
    }

    // ---- stereo step -----------------------------------------------

    #[test]
    fn stereo_step_is_crc_times_nine_plus_3l_plus_r() {
        // Direct formula vs the shift form used by the reference.
        let (l, r) = (5i32, -3i32);
        let crc = 0x1234_5678u32;
        let direct = crc
            .wrapping_mul(9)
            .wrapping_add((l as u32).wrapping_mul(3))
            .wrapping_add(r as u32);
        let shift_form = crc
            .wrapping_add(crc << 3)
            .wrapping_add((l as u32) << 1)
            .wrapping_add(l as u32)
            .wrapping_add(r as u32);
        assert_eq!(update_stereo(crc, l, r), direct);
        assert_eq!(update_stereo(crc, l, r), shift_form);
    }

    #[test]
    fn stereo_accumulator_matches_free_function() {
        let mut acc = BlockCrc::new();
        for pair in [[3i32, -2], [5, 0], [-7, 9]] {
            acc.push_stereo_pair(pair[0], pair[1]);
        }
        assert_eq!(acc.value(), crc_stereo_interleaved(&[3, -2, 5, 0, -7, 9]));
    }

    #[test]
    fn stereo_interleaved_ignores_trailing_odd_sample() {
        // The lone trailing 99 must not change the CRC.
        let with_tail = crc_stereo_interleaved(&[3, -2, 5, 0, -7, 9, 99]);
        let without = crc_stereo_interleaved(&[3, -2, 5, 0, -7, 9]);
        assert_eq!(with_tail, without);
    }

    // ---- joint-stereo undo -----------------------------------------

    // Forward (encoder) joint transform: the algebraic inverse of the
    // spec §5.4 undo `R -= mid>>1; L += R`. Solving that for (mid, side)
    // given true (L, R): mid = L - R, side = R + (mid>>1).
    fn forward_joint(l: i32, r: i32) -> (i32, i32) {
        let mid = l - r;
        let side = r + (mid >> 1);
        (mid, side)
    }

    #[test]
    fn joint_stereo_undo_recovers_left_right() {
        let (l, r) = (10i32, 4i32);
        let (mid, side) = forward_joint(l, r);
        // Inverse (spec §5.4) must recover the original.
        let (rl, rr) = undo_joint_stereo(mid, side);
        assert_eq!((rl, rr), (l, r));
    }

    #[test]
    fn joint_stereo_undo_round_trips_a_range() {
        for l in -50..50i32 {
            for r in -50..50i32 {
                let (mid, side) = forward_joint(l, r);
                assert_eq!(undo_joint_stereo(mid, side), (l, r));
            }
        }
    }

    #[test]
    fn joint_stereo_pair_push_undoes_then_folds() {
        // push_joint_stereo_pair must equal undo + push_stereo_pair.
        let (l, r) = (10i32, 4i32);
        let (mid, side) = forward_joint(l, r);

        let mut a = BlockCrc::new();
        let recovered = a.push_joint_stereo_pair(mid, side);
        assert_eq!(recovered, (l, r));

        let mut b = BlockCrc::new();
        b.push_stereo_pair(l, r);

        assert_eq!(a.value(), b.value());
    }

    #[test]
    fn joint_stereo_interleaved_matches_manual_undo() {
        // Build a mid/side buffer from known L/R pairs and confirm the
        // joint helper equals the plain stereo CRC of the originals.
        let originals = [[3i32, -2], [5, 1], [-7, 9]];
        let mut wire = Vec::new();
        for [l, r] in originals {
            let (mid, side) = forward_joint(l, r);
            wire.push(mid);
            wire.push(side);
        }
        let from_joint = crc_joint_stereo_interleaved(&wire);
        let from_lr = crc_stereo_interleaved(&[3, -2, 5, 1, -7, 9]);
        assert_eq!(from_joint, from_lr);
    }

    // ---- verification ----------------------------------------------

    #[test]
    fn matches_is_true_for_the_computed_value() {
        let crc = crc_mono(&[3, -2, 5, 0, -7]);
        let acc = {
            let mut a = BlockCrc::new();
            for &s in &[3i32, -2, 5, 0, -7] {
                a.push_mono(s);
            }
            a
        };
        assert!(acc.matches(crc));
        assert!(acc.matches(0xffff_fff0));
    }

    #[test]
    fn matches_is_false_for_a_wrong_value() {
        let mut acc = BlockCrc::new();
        acc.push_mono(3);
        assert!(!acc.matches(0xdead_beef));
        assert!(!acc.matches(acc.value().wrapping_add(1)));
    }

    #[test]
    fn single_bit_change_in_samples_changes_the_crc() {
        // CRC must be sensitive to any single decoded-sample perturbation.
        let base = crc_mono(&[3, -2, 5, 0, -7]);
        let flipped = crc_mono(&[3, -2, 5, 1, -7]);
        assert_ne!(base, flipped);
    }

    #[test]
    fn default_equals_new() {
        assert_eq!(BlockCrc::default(), BlockCrc::new());
    }

    // ---- order sensitivity -----------------------------------------

    #[test]
    fn mono_crc_is_order_dependent() {
        // A multiply-accumulate CRC must depend on sample order.
        assert_ne!(crc_mono(&[1, 2, 3]), crc_mono(&[3, 2, 1]));
    }
}
