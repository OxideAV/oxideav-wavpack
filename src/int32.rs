//! WavPack `0x09` large / shifted integer profile (`INT32_DATA`).
//!
//! Staged spec `docs/audio/wavpack/spec/wavpack-sample-formats.md` §3:
//! when block-header flag bit 8 (`INT32_DATA`) is set the block carries
//! true 32-bit integer data — or integer data wider than the entropy
//! coder's ~24-bit magnitude window — and a 4-byte `0x09` sub-block
//! describes how the low bits were reduced before entropy coding:
//!
//! | Offset | Field       | Meaning                                              |
//! | ------ | ----------- | ---------------------------------------------------- |
//! | 0      | `sent_bits` | low bits sent literally per sample via `0x0C`        |
//! | 1      | `zeros`     | redundant trailing (low) zero bits removed per sample |
//! | 2      | `ones`      | redundant trailing (low) one bits removed per sample  |
//! | 3      | `dups`      | duplicated (redundant) low bits removed per sample    |
//!
//! Only one of `zeros` / `ones` / `dups` is non-zero for a given block
//! (mutually exclusive low-bit redundancy patterns). On decode, each
//! entropy-decoded sample is completed by reading `sent_bits` explicit
//! low bits from the `0x0C` extension stream (§4), then the stripped
//! redundancy pattern is re-inserted below them:
//!
//! * `zeros` — the low bits were all `0`: shift left, low bits `0`;
//! * `ones` — the low bits were all `1`: shift left, low bits `1`;
//! * `dups` — the low bits duplicated the bit above them: shift left,
//!   low bits copies of the value's (new) lowest bit.
//!
//! Each fully reassembled sample is folded into the separate extension
//! CRC (`crc_x`, spec `wavpack-decorrelation.md` §5.5) which is compared
//! against the 32-bit little-endian CRC stored at the head of the
//! `0x0C` payload (spec `wavpack-sample-formats.md` §4). This module
//! implements the profile expansion and the per-sample reassembly; the
//! block decoder drives it during the fixup/normalise stage (after
//! decorrelation, before the header left-shift — spec
//! `wavpack-decorrelation.md` §4.2).

use crate::error::{Error, Result};
use crate::samples::BitReader;

/// On-wire byte length of the `0x09` int32-info payload (staged spec
/// `wavpack-sample-formats.md` §3: "payload is **4 bytes**, one per
/// field").
pub const INT32_INFO_PAYLOAD_BYTES: usize = 4;

/// Typed expansion of the `0x09` large/shifted-integer profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Int32Info {
    /// Number of low bits sent literally per sample via the `0x0C`
    /// extension stream.
    pub sent_bits: u8,
    /// Count of redundant trailing zero bits removed per sample.
    pub zeros: u8,
    /// Count of redundant trailing one bits removed per sample.
    pub ones: u8,
    /// Count of duplicated (copies of the lowest remaining bit) low
    /// bits removed per sample.
    pub dups: u8,
}

impl Int32Info {
    /// The redundancy count actually in effect — `zeros`, `ones` or
    /// `dups`, whichever is non-zero (they are mutually exclusive per
    /// the staged spec §3; [`expand_int32_info`] enforces that).
    #[must_use]
    pub fn redundancy_bits(&self) -> u8 {
        self.zeros | self.ones | self.dups
    }

    /// Total number of low bits re-inserted per sample on decode
    /// (`sent_bits` literal bits plus the redundancy pattern).
    #[must_use]
    pub fn total_shift(&self) -> u32 {
        u32::from(self.sent_bits) + u32::from(self.redundancy_bits())
    }

    /// `true` when the profile needs the `0x0C` extension stream
    /// (literal low bits were sent).
    #[must_use]
    pub fn requires_extension(&self) -> bool {
        self.sent_bits > 0
    }

    /// Re-insert the stripped redundancy pattern below `value` (staged
    /// spec §3): `zeros` appends zero bits, `ones` appends one bits,
    /// `dups` appends copies of the value's lowest bit. The shift wraps
    /// on a hostile count (the same truncating-malformed-input posture
    /// as the other fixups); a conformant stream keeps the total shift
    /// within the 32-bit container.
    #[must_use]
    pub fn reinsert_redundancy(&self, value: i32) -> i32 {
        if self.zeros > 0 {
            value.wrapping_shl(u32::from(self.zeros))
        } else if self.ones > 0 {
            let mask = low_bits_mask(self.ones);
            value.wrapping_shl(u32::from(self.ones)) | mask
        } else if self.dups > 0 {
            let mask = if value & 1 == 1 {
                low_bits_mask(self.dups)
            } else {
                0
            };
            value.wrapping_shl(u32::from(self.dups)) | mask
        } else {
            value
        }
    }
}

/// A mask of the low `count` bits (`count` is a redundancy field byte;
/// counts at or beyond the container width saturate to all-ones).
fn low_bits_mask(count: u8) -> i32 {
    if count >= 32 {
        -1
    } else {
        ((1i64 << count) - 1) as i32
    }
}

/// Expand the payload of a `0x09` int32-info sub-block into a typed
/// [`Int32Info`].
///
/// The payload must be exactly 4 bytes (`sent_bits, zeros, ones, dups`
/// in wire order — staged spec §3) or [`Error::Int32InfoLength`] is
/// returned, and at most one of `zeros` / `ones` / `dups` may be
/// non-zero ("mutually exclusive low-bit redundancy patterns") or
/// [`Error::Int32InfoConflict`] is returned.
pub fn expand_int32_info(payload: &[u8]) -> Result<Int32Info> {
    let [sent_bits, zeros, ones, dups] = payload else {
        return Err(Error::Int32InfoLength(payload.len()));
    };
    let info = Int32Info {
        sent_bits: *sent_bits,
        zeros: *zeros,
        ones: *ones,
        dups: *dups,
    };
    let populated = [info.zeros, info.ones, info.dups]
        .iter()
        .filter(|&&c| c > 0)
        .count();
    if populated > 1 {
        return Err(Error::Int32InfoConflict {
            zeros: info.zeros,
            ones: info.ones,
            dups: info.dups,
        });
    }
    Ok(info)
}

/// Reassemble a buffer of entropy-decoded int32-reduced samples in
/// place: append each sample's `sent_bits` literal low bits from the
/// `0x0C` extension bit reader (LSB-first, staged spec §4), re-insert
/// the redundancy pattern below them, and fold every fully reassembled
/// value into the extension CRC.
///
/// `ext` must be `Some` when [`Int32Info::requires_extension`]; the
/// caller has already stripped the 4-byte `crc_wvx` prefix off the
/// `0x0C` payload. Returns the accumulated `crc_x` register (spec
/// `wavpack-decorrelation.md` §5.5) for the block-end comparison —
/// `None` when the profile moved no bits at all (no extension stream
/// and no redundancy; the values are already final and no `crc_x` is
/// accumulated over them by this stage).
pub fn reassemble_int32(
    pcm: &mut [i32],
    info: &Int32Info,
    mut ext: Option<&mut BitReader<'_>>,
) -> Result<u32> {
    let mut crc_x = crate::crc::ExtensionCrc::new();
    for slot in pcm.iter_mut() {
        let mut value = *slot;
        if info.sent_bits > 0 {
            let reader = ext.as_deref_mut().ok_or(Error::BlockMissingOverflowBits)?;
            let literal = reader.get_bits(u32::from(info.sent_bits))?;
            value = value.wrapping_shl(u32::from(info.sent_bits)) | literal as i32;
        }
        value = info.reinsert_redundancy(value);
        crc_x.push(value);
        *slot = value;
    }
    Ok(crc_x.value())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expand_reads_the_four_wire_fields() {
        let info = expand_int32_info(&[7, 0, 0, 0]).unwrap();
        assert_eq!(
            info,
            Int32Info {
                sent_bits: 7,
                zeros: 0,
                ones: 0,
                dups: 0
            }
        );
        assert!(info.requires_extension());
        assert_eq!(info.total_shift(), 7);

        let info = expand_int32_info(&[0, 8, 0, 0]).unwrap();
        assert_eq!(info.zeros, 8);
        assert!(!info.requires_extension());
        assert_eq!(info.redundancy_bits(), 8);
        assert_eq!(info.total_shift(), 8);
    }

    #[test]
    fn expand_rejects_wrong_lengths() {
        for n in [0usize, 1, 2, 3, 5, 8] {
            let payload = vec![0u8; n];
            assert_eq!(
                expand_int32_info(&payload),
                Err(Error::Int32InfoLength(n)),
                "len {n}"
            );
        }
    }

    #[test]
    fn expand_rejects_conflicting_redundancy_fields() {
        // Staged spec §3: only one of zeros/ones/dups is non-zero.
        assert_eq!(
            expand_int32_info(&[0, 4, 2, 0]),
            Err(Error::Int32InfoConflict {
                zeros: 4,
                ones: 2,
                dups: 0
            })
        );
        assert_eq!(
            expand_int32_info(&[3, 0, 1, 1]),
            Err(Error::Int32InfoConflict {
                zeros: 0,
                ones: 1,
                dups: 1
            })
        );
    }

    #[test]
    fn zeros_reinsertion_appends_zero_bits() {
        let info = expand_int32_info(&[0, 8, 0, 0]).unwrap();
        assert_eq!(info.reinsert_redundancy(1), 256);
        assert_eq!(info.reinsert_redundancy(-1), -256);
        assert_eq!(info.reinsert_redundancy(0), 0);
    }

    #[test]
    fn ones_reinsertion_appends_one_bits() {
        let info = expand_int32_info(&[0, 0, 4, 0]).unwrap();
        // v = orig >> 4 where the low 4 bits were all ones.
        assert_eq!(info.reinsert_redundancy(1), 0x1F);
        assert_eq!(info.reinsert_redundancy(0), 0x0F);
        // Two's complement: -1 with 4 trailing ones is still -1.
        assert_eq!(info.reinsert_redundancy(-1), -1);
    }

    #[test]
    fn dups_reinsertion_copies_the_low_bit() {
        let info = expand_int32_info(&[0, 0, 0, 3]).unwrap();
        // Low bit 1 → three more ones below it.
        assert_eq!(info.reinsert_redundancy(0b101), 0b101_111);
        // Low bit 0 → three zeros below it.
        assert_eq!(info.reinsert_redundancy(0b100), 0b100_000);
        assert_eq!(info.reinsert_redundancy(0), 0);
        assert_eq!(info.reinsert_redundancy(-1), -1);
        assert_eq!(info.reinsert_redundancy(-2), -16);
    }

    #[test]
    fn reassemble_reads_sent_bits_lsb_first_per_sample() {
        // Two samples, 4 sent bits each. Extension bit stream (after
        // the CRC prefix, which the caller strips): bits are consumed
        // LSB-first, so byte 0x3A yields 0b1010 then 0b0011.
        let info = expand_int32_info(&[4, 0, 0, 0]).unwrap();
        let ext_bytes = [0x3Au8];
        let mut reader = BitReader::new(&ext_bytes);
        let mut pcm = [1i32, -1];
        let crc = reassemble_int32(&mut pcm, &info, Some(&mut reader)).unwrap();
        // 1 << 4 | 0b1010 = 26; -1 << 4 | 0b0011 = -16 | 3 = -13.
        assert_eq!(pcm, [26, -13]);
        // The CRC register folds both reassembled values.
        let mut expect = crate::crc::ExtensionCrc::new();
        expect.push(26);
        expect.push(-13);
        assert_eq!(crc, expect.value());
    }

    #[test]
    fn reassemble_combines_sent_bits_and_redundancy() {
        // sent_bits below the entropy value, redundancy below the sent
        // bits: value = ((v << sent) | literal) << zeros.
        let info = expand_int32_info(&[2, 3, 0, 0]).unwrap();
        let ext_bytes = [0b11u8];
        let mut reader = BitReader::new(&ext_bytes);
        let mut pcm = [1i32];
        reassemble_int32(&mut pcm, &info, Some(&mut reader)).unwrap();
        assert_eq!(pcm, [((1 << 2) | 0b11) << 3]);
    }

    #[test]
    fn reassemble_without_extension_when_only_redundancy() {
        let info = expand_int32_info(&[0, 0, 0, 2]).unwrap();
        let mut pcm = [0b11i32, 0b10];
        let crc = reassemble_int32(&mut pcm, &info, None).unwrap();
        assert_eq!(pcm, [0b1111, 0b1000]);
        let mut expect = crate::crc::ExtensionCrc::new();
        expect.push(0b1111);
        expect.push(0b1000);
        assert_eq!(crc, expect.value());
    }

    #[test]
    fn reassemble_requires_a_reader_when_bits_were_sent() {
        let info = expand_int32_info(&[4, 0, 0, 0]).unwrap();
        let mut pcm = [1i32];
        assert_eq!(
            reassemble_int32(&mut pcm, &info, None),
            Err(Error::BlockMissingOverflowBits)
        );
    }

    #[test]
    fn reassemble_propagates_extension_truncation() {
        let info = expand_int32_info(&[8, 0, 0, 0]).unwrap();
        let ext_bytes = [0xFFu8]; // one byte: enough for 1 sample only
        let mut reader = BitReader::new(&ext_bytes);
        let mut pcm = [1i32, 2];
        assert_eq!(
            reassemble_int32(&mut pcm, &info, Some(&mut reader)),
            Err(Error::Truncated)
        );
    }
}
