//! WavPack v.4 entropy-info sub-block expander (ID 0x05).
//!
//! The wiki "Entropy info" section
//! (`docs/audio/wavpack/wiki/WavPack.wiki`) reads, in full:
//!
//! > This section contains one or two sets of medians for samples
//! > decoding. Each median is log-packed into 16 bits as described
//! > above.
//!
//! The "Samples coding" section that follows references three
//! medians per set — `median[0]`, `median[1]`, `median[2]` — used as
//! the unary-prefix base / mantissa-width control for the Golomb-coded
//! sample stream. The wiki "one or two sets" wording matches the
//! channel count of the enclosing block: one set for mono, two sets
//! (left + right) for stereo. Combined, that gives:
//!
//! * Mono   — 3 medians × 2 bytes = **6 bytes** on the wire.
//! * Stereo — 6 medians × 2 bytes = **12 bytes** on the wire.
//!
//! ## Log-pack: "as described above"
//!
//! The wiki phrase "log-packed into 16 bits as described above"
//! points back to the immediately preceding *Decorrelation samples*
//! section, which is the only other 16-bit log-pack the wiki has
//! described to this point in the document (the *Decorrelation
//! weights* recipe packs into 8 bits, not 16):
//!
//! > Each sample is 32-bit but stored in 16 bits, lower 8 bits are
//! > mantiss and high 8 bits are exponent-9, i.e if exponent < 9
//! > shift mantiss right, otherwise left.
//!
//! Each median is therefore an `[mantissa_lo, exponent_hi]`
//! little-endian word identical to the round-3 sample word, expanded
//! through the same [`expand_sample_word`](crate::decorrelation)
//! helper (re-used here as `crate::decorrelation::expand_sample_word`).
//!
//! ## Output shape
//!
//! The expander always returns a fully-populated
//! [`EntropyInfo { medians_left: [i32; 3], medians_right: [i32; 3] }`]
//! — for mono inputs `medians_right` stays at `[0; 3]` (no second set
//! was on the wire). Callers correlate the populated channel count
//! with the [`crate::Flags::mono`] bit on the enclosing block header.
//!
//! ## Docs-gap notes (round 4)
//!
//! * **"As described above" reference.** The wiki phrasing is implicit
//!   — it doesn't say "as the decorrelation-samples 16-bit log-pack
//!   described above". Round 4 reads the closest preceding 16-bit
//!   log-pack (the *Decorrelation samples* section). The
//!   *Decorrelation weights* section is the only other log-pack but
//!   packs into 8 bits, so the 16-bit wording rules it out. A future
//!   docs revision should make the cross-reference explicit.
//! * **Median set ordering for stereo.** The wiki "one or two sets"
//!   wording does not name the order. Round 4 assumes left-then-right
//!   (matching the natural channel order used everywhere else in the
//!   WavPack v.4 stream — multichannel layout, decorrelation weights,
//!   etc.). Empirical confirmation against a real stereo encode is
//!   round-5 work and is gated on the entropy / Golomb decode loop
//!   landing first.
//! * **Allowed channel counts.** The wiki names mono and stereo
//!   ("one or two sets"). Multichannel WavPack streams are documented
//!   elsewhere in the spec as a sequence of mono / stereo blocks
//!   (the `0x0D` multichannel-info sub-block carries the channel
//!   mask), so the per-block entropy-info payload should still be
//!   either 6 or 12 bytes. Round 4 enforces that strictly.

use crate::decorrelation::expand_sample_word;
use crate::error::{Error, Result};

/// On-disk size, in bytes, of a single median value (one 16-bit
/// log-packed word per the wiki "log-packed into 16 bits" sentence
/// in the *Entropy info* section).
pub const MEDIAN_ON_WIRE_BYTES: usize = 2;
/// Number of medians per channel — the wiki "Samples coding" section
/// references `median[0]`, `median[1]`, `median[2]` exclusively.
pub const MEDIANS_PER_CHANNEL: usize = 3;
/// Wire size of a mono entropy-info payload (one set of three
/// medians × 2 bytes per median).
pub const MONO_PAYLOAD_BYTES: usize = MEDIANS_PER_CHANNEL * MEDIAN_ON_WIRE_BYTES;
/// Wire size of a stereo entropy-info payload (two sets of three
/// medians × 2 bytes per median).
pub const STEREO_PAYLOAD_BYTES: usize = 2 * MONO_PAYLOAD_BYTES;

/// Typed expansion of the `0x05` entropy-info sub-block.
///
/// `medians_left` holds the first set the wiki names; `medians_right`
/// holds the second set, which is present on stereo blocks only. For
/// mono blocks the right set is left at `[0; 3]`.
///
/// The wiki "Samples coding" section consumes these as
/// `median[0]`, `median[1]`, `median[2]` — fixed-position state for
/// the Golomb-coded sample stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntropyInfo {
    /// First set of three medians — the only set on a mono block,
    /// the left channel's medians on a stereo block.
    pub medians_left: [i32; MEDIANS_PER_CHANNEL],
    /// Second set of three medians, present on stereo blocks only.
    /// Left at `[0; 3]` on a mono block (the wiki "one or two sets"
    /// wording — only the first is on the wire).
    pub medians_right: [i32; MEDIANS_PER_CHANNEL],
}

impl EntropyInfo {
    /// Construct an `EntropyInfo` whose right-channel medians are all
    /// zero — i.e. the natural value for a mono block.
    pub const fn mono(medians_left: [i32; MEDIANS_PER_CHANNEL]) -> Self {
        Self {
            medians_left,
            medians_right: [0; MEDIANS_PER_CHANNEL],
        }
    }

    /// Construct an `EntropyInfo` with both channels populated — the
    /// natural value for a stereo block. Symmetric counterpart to
    /// [`Self::mono`]; matches the two-set form the wiki "one or two
    /// sets of medians" sentence describes.
    ///
    /// Note: if `medians_right` is `[0; 3]` the resulting value will
    /// still report [`Self::is_mono`] `== true` (the content-only check
    /// the round-4 expander uses); pass non-zero seeds to model a
    /// genuine stereo payload.
    pub const fn stereo(
        medians_left: [i32; MEDIANS_PER_CHANNEL],
        medians_right: [i32; MEDIANS_PER_CHANNEL],
    ) -> Self {
        Self {
            medians_left,
            medians_right,
        }
    }

    /// `true` when the right-channel medians are all zero — i.e. the
    /// payload was a mono (single-set) entropy-info sub-block.
    /// Convenience for callers correlating against
    /// [`crate::Flags::mono`].
    pub fn is_mono(&self) -> bool {
        self.medians_right == [0; MEDIANS_PER_CHANNEL]
    }

    /// `true` when both median sets carry independent (right-set
    /// non-zero) data — i.e. the payload was a stereo (two-set)
    /// entropy-info sub-block. Inverse of [`Self::is_mono`].
    ///
    /// Like [`Self::is_mono`] this is a content-only check: an
    /// all-zero stereo payload would look mono. Callers needing the
    /// authoritative answer should still correlate against
    /// [`crate::Flags::mono`] on the enclosing block header.
    pub fn is_stereo(&self) -> bool {
        !self.is_mono()
    }

    /// Number of populated median sets — `1` for a mono payload
    /// (right set is all zero), `2` for a stereo payload (both sets
    /// carry independent values).
    ///
    /// The wiki "Entropy info" sentence — "one or two sets of medians
    /// for samples decoding" — is reflected here as the count of sets
    /// actually on the wire.
    pub fn channels(&self) -> u8 {
        if self.is_mono() {
            1
        } else {
            2
        }
    }

    /// Per-channel median accessor — `Some([m0, m1, m2])` when
    /// `channel_idx` names a populated set (`0` for left/mono on any
    /// payload, `1` for right on a stereo payload). Returns `None` for
    /// `1` on a mono payload (the wiki put no second set on the wire)
    /// and for indices `>= 2` (the wiki names mono and stereo only,
    /// "one or two sets").
    ///
    /// The set ordering — left then right — is the same order the
    /// round-4 expander reads the bytes off the wire, matching every
    /// other left/right pairing in the WavPack v.4 stream.
    pub fn medians_for_channel(&self, channel_idx: u8) -> Option<[i32; MEDIANS_PER_CHANNEL]> {
        match channel_idx {
            0 => Some(self.medians_left),
            1 if self.is_stereo() => Some(self.medians_right),
            _ => None,
        }
    }
}

/// Expand the payload of a `0x05` entropy-info sub-block into a
/// typed [`EntropyInfo`] value.
///
/// Each median is read as a little-endian 16-bit word and expanded
/// through the same `[mantissa_lo, exponent_hi]` log-pack the wiki
/// describes for the `0x04` decorrelation-samples sub-block (mantissa
/// signed; exponent biased by `-9`).
///
/// The payload length must be either 6 bytes (one set, mono block)
/// or 12 bytes (two sets, stereo block). Any other length is a
/// malformed sub-block and is rejected with
/// [`Error::EntropyInfoLength`].
pub fn expand_entropy(payload: &[u8]) -> Result<EntropyInfo> {
    match payload.len() {
        MONO_PAYLOAD_BYTES => Ok(EntropyInfo::mono(read_median_set(
            &payload[..MONO_PAYLOAD_BYTES],
        ))),
        STEREO_PAYLOAD_BYTES => {
            let medians_left = read_median_set(&payload[..MONO_PAYLOAD_BYTES]);
            let medians_right = read_median_set(&payload[MONO_PAYLOAD_BYTES..]);
            Ok(EntropyInfo {
                medians_left,
                medians_right,
            })
        }
        other => Err(Error::EntropyInfoLength(other)),
    }
}

/// Decode three consecutive 16-bit log-packed medians from a 6-byte
/// slice.
fn read_median_set(set: &[u8]) -> [i32; MEDIANS_PER_CHANNEL] {
    debug_assert_eq!(set.len(), MONO_PAYLOAD_BYTES);
    let mut out = [0i32; MEDIANS_PER_CHANNEL];
    for (i, word) in set.chunks_exact(MEDIAN_ON_WIRE_BYTES).enumerate() {
        out[i] = expand_sample_word(word[0], word[1]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: build a 2-byte log-packed median word from an unsigned
    // (mantissa, exponent) pair — the on-disk wire order matches the
    // round-3 decorrelation-samples expander.
    fn median_word(mantissa: u8, exponent: u8) -> [u8; 2] {
        [mantissa, exponent]
    }

    #[test]
    fn mono_payload_decodes_one_set_and_zeros_right() {
        // Three medians, exponent = 9 means shift = 0, so the mantissa
        // is returned unchanged.
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&median_word(0x10, 0x09)); // +16
        bytes.extend_from_slice(&median_word(0x20, 0x09)); // +32
        bytes.extend_from_slice(&median_word(0x40, 0x09)); // +64

        let info = expand_entropy(&bytes).unwrap();
        assert_eq!(info.medians_left, [16, 32, 64]);
        assert_eq!(info.medians_right, [0, 0, 0]);
        assert!(info.is_mono());
    }

    #[test]
    fn stereo_payload_decodes_two_sets_in_left_then_right_order() {
        let mut bytes = Vec::new();
        // Left channel medians: 1, 2, 3.
        bytes.extend_from_slice(&median_word(0x01, 0x09));
        bytes.extend_from_slice(&median_word(0x02, 0x09));
        bytes.extend_from_slice(&median_word(0x03, 0x09));
        // Right channel medians: 4, 5, 6.
        bytes.extend_from_slice(&median_word(0x04, 0x09));
        bytes.extend_from_slice(&median_word(0x05, 0x09));
        bytes.extend_from_slice(&median_word(0x06, 0x09));

        let info = expand_entropy(&bytes).unwrap();
        assert_eq!(info.medians_left, [1, 2, 3]);
        assert_eq!(info.medians_right, [4, 5, 6]);
        assert!(!info.is_mono());
    }

    #[test]
    fn stereo_payload_handles_negative_mantissa_via_sign_extension() {
        // Reuse the round-3 sample expander's signed-mantissa
        // contract: 0xFF mantissa at exponent 9 → -1.
        let mut bytes = Vec::new();
        // Left: -1, -2, -3.
        bytes.extend_from_slice(&median_word(0xFF, 0x09));
        bytes.extend_from_slice(&median_word(0xFE, 0x09));
        bytes.extend_from_slice(&median_word(0xFD, 0x09));
        // Right: 7, 8, 9.
        bytes.extend_from_slice(&median_word(0x07, 0x09));
        bytes.extend_from_slice(&median_word(0x08, 0x09));
        bytes.extend_from_slice(&median_word(0x09, 0x09));

        let info = expand_entropy(&bytes).unwrap();
        assert_eq!(info.medians_left, [-1, -2, -3]);
        assert_eq!(info.medians_right, [7, 8, 9]);
    }

    #[test]
    fn log_pack_shift_left_branch_applies_to_medians() {
        // exponent = 11 → shift left 2. mantissa = +4 → 16.
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&median_word(0x04, 0x0B)); // +4 << 2 = 16
        bytes.extend_from_slice(&median_word(0x01, 0x0C)); // +1 << 3 = 8
        bytes.extend_from_slice(&median_word(0x02, 0x0A)); // +2 << 1 = 4

        let info = expand_entropy(&bytes).unwrap();
        assert_eq!(info.medians_left, [16, 8, 4]);
    }

    #[test]
    fn log_pack_shift_right_branch_applies_to_medians() {
        // exponent = 7 → shift right 2. mantissa = 0x40 (+64) → 16.
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&median_word(0x40, 0x07)); // +64 >> 2 = 16
        bytes.extend_from_slice(&median_word(0x20, 0x08)); // +32 >> 1 = 16
        bytes.extend_from_slice(&median_word(0x10, 0x09)); // +16 >> 0 = 16

        let info = expand_entropy(&bytes).unwrap();
        assert_eq!(info.medians_left, [16, 16, 16]);
    }

    #[test]
    fn empty_payload_is_rejected() {
        let err = expand_entropy(&[]).unwrap_err();
        assert_eq!(err, Error::EntropyInfoLength(0));
    }

    #[test]
    fn payload_short_of_mono_size_is_rejected() {
        let payload = [0u8; MONO_PAYLOAD_BYTES - 1];
        let err = expand_entropy(&payload).unwrap_err();
        assert_eq!(err, Error::EntropyInfoLength(MONO_PAYLOAD_BYTES - 1));
    }

    #[test]
    fn payload_between_mono_and_stereo_is_rejected() {
        // 7 / 8 / 9 / 10 / 11 bytes all fail — only 6 and 12 are
        // acceptable.
        for n in (MONO_PAYLOAD_BYTES + 1)..STEREO_PAYLOAD_BYTES {
            let payload = vec![0u8; n];
            let err = expand_entropy(&payload).unwrap_err();
            assert_eq!(err, Error::EntropyInfoLength(n), "n = {n}");
        }
    }

    #[test]
    fn payload_longer_than_stereo_size_is_rejected() {
        let payload = [0u8; STEREO_PAYLOAD_BYTES + 1];
        let err = expand_entropy(&payload).unwrap_err();
        assert_eq!(err, Error::EntropyInfoLength(STEREO_PAYLOAD_BYTES + 1));
    }

    #[test]
    fn mono_helper_zeroes_right_set() {
        let info = EntropyInfo::mono([10, 20, 30]);
        assert_eq!(info.medians_left, [10, 20, 30]);
        assert_eq!(info.medians_right, [0, 0, 0]);
        assert!(info.is_mono());
    }

    #[test]
    fn is_mono_returns_false_when_any_right_median_is_nonzero() {
        let info = EntropyInfo {
            medians_left: [0, 0, 0],
            medians_right: [0, 0, 1],
        };
        assert!(!info.is_mono());
    }

    #[test]
    fn zero_word_decodes_to_zero_median() {
        // All-zero payload → all-zero medians. (mantissa = 0, any
        // exponent — and 0 here — yields 0 after the shift.)
        let bytes = [0u8; STEREO_PAYLOAD_BYTES];
        let info = expand_entropy(&bytes).unwrap();
        assert_eq!(info.medians_left, [0, 0, 0]);
        assert_eq!(info.medians_right, [0, 0, 0]);
        // An all-zero stereo payload is structurally indistinguishable
        // from a mono block by content alone — `is_mono()` returns
        // `true` here, matching the documented "any zero-right-set"
        // contract. Callers correlate against the enclosing block's
        // mono flag for the authoritative answer.
        assert!(info.is_mono());
    }

    // ---- Round-12 channel accessors ----

    #[test]
    fn is_stereo_inverts_is_mono() {
        let mono = EntropyInfo::mono([1, 2, 3]);
        assert!(!mono.is_stereo());
        let stereo = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [4, 5, 6],
        };
        assert!(stereo.is_stereo());
    }

    #[test]
    fn channels_returns_one_for_mono_two_for_stereo() {
        let mono = EntropyInfo::mono([1, 2, 3]);
        assert_eq!(mono.channels(), 1);
        let stereo = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [4, 5, 6],
        };
        assert_eq!(stereo.channels(), 2);
    }

    #[test]
    fn medians_for_channel_yields_left_on_zero_and_right_on_one_for_stereo() {
        let stereo = EntropyInfo {
            medians_left: [10, 20, 30],
            medians_right: [40, 50, 60],
        };
        assert_eq!(stereo.medians_for_channel(0), Some([10, 20, 30]));
        assert_eq!(stereo.medians_for_channel(1), Some([40, 50, 60]));
    }

    #[test]
    fn medians_for_channel_one_is_none_on_mono() {
        let mono = EntropyInfo::mono([7, 8, 9]);
        assert_eq!(mono.medians_for_channel(0), Some([7, 8, 9]));
        // Channel 1 on a mono payload returns None — the wiki put no
        // second set on the wire.
        assert_eq!(mono.medians_for_channel(1), None);
    }

    #[test]
    fn medians_for_channel_rejects_out_of_range_indices() {
        let stereo = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [4, 5, 6],
        };
        // The wiki names only mono and stereo — index 2 and beyond are
        // not on the wire.
        assert_eq!(stereo.medians_for_channel(2), None);
        assert_eq!(stereo.medians_for_channel(3), None);
        assert_eq!(stereo.medians_for_channel(255), None);
    }

    // ---- Round-201 EntropyInfo::stereo constructor ----

    #[test]
    fn stereo_helper_populates_both_sets() {
        let info = EntropyInfo::stereo([10, 20, 30], [40, 50, 60]);
        assert_eq!(info.medians_left, [10, 20, 30]);
        assert_eq!(info.medians_right, [40, 50, 60]);
        assert!(info.is_stereo());
        assert_eq!(info.channels(), 2);
    }

    #[test]
    fn stereo_helper_matches_struct_literal() {
        let from_helper = EntropyInfo::stereo([1, 2, 3], [4, 5, 6]);
        let from_literal = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [4, 5, 6],
        };
        assert_eq!(from_helper, from_literal);
    }

    #[test]
    fn stereo_helper_with_zero_right_reports_mono_by_content() {
        // The is_mono() check is content-only; an all-zero right set
        // looks mono even when built through the stereo constructor.
        // Callers correlate against the enclosing block's Flags::mono
        // bit for the authoritative answer (see is_mono() docs).
        let info = EntropyInfo::stereo([1, 2, 3], [0, 0, 0]);
        assert!(info.is_mono());
        assert_eq!(info.channels(), 1);
    }
}
