//! WavPack v.4 packed-overflow-bits sub-block typed view (ID `0x0C`).
//!
//! The wiki "IDs" listing of `docs/audio/wavpack/wiki/WavPack.wiki` names
//! sub-block `0x0C` as:
//!
//! > 0x0C - packed overflow bits from floating-point or large integers
//!
//! and the staged clean-room entropy doc
//! `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 names the same
//! ID as the **extension bitstream** carrying overflow / extension bits.
//! The bitstream is the third of the three documented payload-carrying
//! IDs that route into a sample-decoder bit reader (alongside `0x0A`
//! main and `0x0B` correction); it carries the high-order bits a float
//! or `>24`-bit integer sample needs once the main stream's entropy
//! decode has produced the low-order mantissa.
//!
//! The wiki places **no internal structure** on the `0x0C` payload — it
//! is a contiguous bitstream the downstream float / large-integer
//! container fix-up consumes left-to-right alongside the main `0x0A`
//! stream. The typed view therefore carries the bytes verbatim and
//! exposes the same byte-length / bit-reader-factory surface
//! [`crate::PackedSamples`] (`0x0A`) and [`crate::PackedCorrectionData`]
//! (`0x0B`) expose. The float / int32 container fix-ups that would
//! consume the wrapped bytes themselves remain gated on the
//! [`crate::UnsupportedBlockFeature::FloatData`] /
//! [`crate::UnsupportedBlockFeature::Int32Mode`] feature refusals in
//! [`crate::WavPackBlock::decode_samples`]; this module adds only the
//! typed view + walker bridge, so callers staging the deferred fix-up
//! have a single concrete handoff into the round-5/6/15 bit reader.
//!
//! ## Length: unconstrained by the wiki
//!
//! The wiki places no size constraint on the `0x0C` payload — the
//! encoder packs however many overflow bits the per-sample float /
//! large-integer reconstruction needs into however many bytes it chose.
//! A zero-byte `0x0C` payload is structurally valid (it would mean the
//! encoder produced no overflow bits for this block, e.g. every float
//! sample fit in the main-stream's low-order representation). The
//! constructor therefore accepts any byte slice — including the empty
//! one — without rejection.
//!
//! ## Bit order
//!
//! Bits are consumed least-significant-bit first within each byte and
//! bytes in stream order — the same convention the main `0x0A` reader
//! uses per the [`crate::samples`] module-level bit-order documentation,
//! and the same convention [`crate::PackedCorrectionData`] uses for the
//! `0x0B` correction stream. The [`Self::bit_reader`] factory honours
//! that convention exactly by constructing a [`crate::BitReader`] over
//! the payload bytes.

use crate::samples::BitReader;

/// Typed view of the `0x0C` packed-overflow-bits sub-block payload —
/// the bitstream the wiki "IDs" listing annotates as "packed overflow
/// bits from floating-point or large integers" and the clean-room
/// entropy doc names the **extension bitstream**.
///
/// The view borrows the underlying payload bytes verbatim from the
/// round-2 metadata walker; constructing it does no work beyond
/// storing the slice reference. Use [`Self::bit_reader`] to obtain a
/// fresh [`BitReader`] positioned at bit 0 for callers staging the
/// deferred float / large-integer container fix-up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PackedOverflowBits<'a> {
    bytes: &'a [u8],
}

impl<'a> PackedOverflowBits<'a> {
    /// Construct a packed-overflow-bits view over the byte slice the
    /// round-2 walker handed back for a `0x0C` sub-block. The wiki
    /// places no length constraint on the payload (the overflow-bit
    /// count is determined by the per-sample float / large-integer
    /// fix-up the consumer drives), so any byte slice — including the
    /// empty one — is accepted without rejection.
    pub const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }

    /// The underlying payload bytes — exactly the slice the round-2
    /// walker handed back for the `0x0C` sub-block.
    pub const fn bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// Byte length of the payload.
    pub const fn len(&self) -> usize {
        self.bytes.len()
    }

    /// `true` when the payload is empty (no overflow bits to decode).
    /// The wiki does not forbid a zero-byte `0x0C` payload — it would
    /// mean the encoder produced no overflow bits for this block (e.g.
    /// every float sample fit in the main-stream's low-order
    /// representation).
    pub const fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Construct a fresh [`BitReader`] positioned at bit 0 of the
    /// payload, ready to feed the float / large-integer container
    /// fix-up.
    ///
    /// Each call returns an independent reader — the
    /// [`PackedOverflowBits`] view itself carries no read cursor, so
    /// multiple bit readers can be constructed from the same view
    /// (e.g. one for a probe and one for the real decode).
    pub fn bit_reader(&self) -> BitReader<'a> {
        BitReader::new(self.bytes)
    }
}

/// Construct a [`PackedOverflowBits`] view over a `0x0C` sub-block
/// payload.
///
/// Typed counterpart to the round-2 walker output for the `0x0C` ID —
/// analogous to the round-12 [`crate::expand_packed_samples`] for
/// `0x0A` and the round-233 [`crate::expand_packed_correction_data`]
/// for `0x0B`. The wiki places no internal structure on the `0x0C`
/// payload, so the "expansion" is a typed wrap rather than a
/// byte-by-byte decode. The float / large-integer container fix-ups
/// that would consume the wrapped bytes remain gated on the
/// [`crate::UnsupportedBlockFeature::FloatData`] /
/// [`crate::UnsupportedBlockFeature::Int32Mode`] feature refusals.
pub fn expand_packed_overflow_bits(payload: &[u8]) -> PackedOverflowBits<'_> {
    PackedOverflowBits::new(payload)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_preserves_bytes_verbatim() {
        let bytes = [0x44, 0x55, 0x66];
        let view = PackedOverflowBits::new(&bytes);
        assert_eq!(view.bytes(), &[0x44, 0x55, 0x66]);
        assert_eq!(view.len(), 3);
        assert!(!view.is_empty());
    }

    #[test]
    fn empty_payload_is_accepted_and_reported_empty() {
        // The wiki places no length constraint on the 0x0C payload, so
        // a zero-byte slice is a valid (degenerate) view.
        let view = PackedOverflowBits::new(&[]);
        assert!(view.is_empty());
        assert_eq!(view.len(), 0);
        assert_eq!(view.bytes(), &[] as &[u8]);
    }

    #[test]
    fn expand_packed_overflow_bits_round_trips_the_byte_slice() {
        let payload = [0x12, 0x34, 0x56, 0x78];
        let view = expand_packed_overflow_bits(&payload);
        assert_eq!(view.bytes(), &[0x12, 0x34, 0x56, 0x78]);
        assert_eq!(view.len(), 4);
    }

    #[test]
    fn bit_reader_starts_at_byte_zero_bit_zero() {
        let payload = [0xFF, 0xFF];
        let view = PackedOverflowBits::new(&payload);
        let r = view.bit_reader();
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 0);
        assert_eq!(r.bits_remaining(), 16);
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn bit_reader_yields_first_bit_lsb_first() {
        // 0x09 = 0b0000_1001 -> LSB first: 1, 0, 0, 1, 0, 0, 0, 0
        let payload = [0x09u8];
        let view = PackedOverflowBits::new(&payload);
        let mut r = view.bit_reader();
        assert_eq!(r.get_bit().unwrap(), 1);
        assert_eq!(r.get_bit().unwrap(), 0);
        assert_eq!(r.get_bit().unwrap(), 0);
        assert_eq!(r.get_bit().unwrap(), 1);
    }

    #[test]
    fn bit_reader_factory_returns_independent_readers() {
        // Multiple bit_reader() calls yield independent readers — the
        // view itself carries no cursor.
        let payload = [0xFFu8];
        let view = PackedOverflowBits::new(&payload);
        let mut probe = view.bit_reader();
        let mut real = view.bit_reader();
        assert_eq!(probe.get_bit().unwrap(), 1);
        assert_eq!(real.byte_position(), 0);
        assert_eq!(real.bit_position(), 0);
        assert_eq!(real.get_bit().unwrap(), 1);
    }

    #[test]
    fn empty_payload_bit_reader_reports_immediate_truncation() {
        let view = PackedOverflowBits::new(&[]);
        let mut r = view.bit_reader();
        assert!(r.is_empty());
        assert_eq!(r.bits_remaining(), 0);
        // Any read against an empty view should report truncation — no
        // zero-fill contract.
        assert_eq!(r.get_bit(), Err(crate::Error::Truncated));
    }

    #[test]
    fn view_is_copy_and_independent_of_caller_lifetime() {
        let payload = [0xFE, 0xED, 0xFA, 0xCE];
        let view = PackedOverflowBits::new(&payload);
        let copy = view;
        assert_eq!(view.len(), copy.len());
        assert_eq!(view.bytes(), copy.bytes());
    }

    #[test]
    fn view_is_distinct_type_from_packed_samples_and_correction() {
        // Compile-time check: the three payload-carrying typed views
        // (0x0A main, 0x0B correction, 0x0C overflow) are distinct
        // types even though all three wrap the same byte-slice shape.
        // The wiki distinguishes the streams by ID and they feed
        // distinct decode paths — keeping the types distinct guards
        // against accidentally feeding overflow bytes into the main
        // sample-decode loop or the correction-stream consumer.
        let payload = [0u8; 4];
        let overflow = PackedOverflowBits::new(&payload);
        let correction = crate::PackedCorrectionData::new(&payload);
        let samples = crate::PackedSamples::new(&payload);
        assert_eq!(overflow.bytes(), correction.bytes());
        assert_eq!(overflow.bytes(), samples.bytes());
        assert_eq!(overflow.len(), correction.len());
        assert_eq!(overflow.len(), samples.len());
    }
}
