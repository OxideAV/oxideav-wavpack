//! WavPack v.4 packed-correction-data sub-block typed view (ID `0x0B`).
//!
//! The wiki "IDs" listing of `docs/audio/wavpack/wiki/WavPack.wiki` names
//! sub-block `0x0B` as:
//!
//! > 0x0B - packed correction data (wvc file)
//!
//! Correction data is the second half of the WavPack v.4 *hybrid* mode:
//! a lossless `.wv` decode reads only the main `0x0A` packed samples
//! sub-block per the wiki "Samples coding" section, but a hybrid stream
//! is split across two files — the lossy main `.wv` carrying `0x0A`, and
//! a `.wvc` companion carrying `0x0B` correction samples plus the `0x07`
//! noise-shaping profile that pairs with them. Concatenating the two
//! recovers a sample-exact decode (the "hybrid lossless" round-trip).
//!
//! The wiki places **no internal structure** on the `0x0B` payload — it
//! is a contiguous entropy-coded bitstream the hybrid decoder consumes
//! left-to-right alongside the matching `0x0A` stream, exactly the same
//! way [`crate::PackedSamples`] wraps the main stream. The typed view
//! therefore carries the bytes verbatim and exposes the same
//! byte-length / bit-reader-factory surface so callers staging a
//! hybrid-mode decoder have a single concrete handoff into the
//! round-5/6/15 bit-reader and sample-decode primitives.
//!
//! The hybrid sample decode itself (spec §4.2 step 6 second paragraph,
//! `error_limit != 0`) stays out of scope for this round — it remains
//! gated by the [`crate::UnsupportedBlockFeature::Hybrid`] feature
//! refusal in [`crate::WavPackBlock::decode_samples`]. This module adds
//! only the typed view + walker bridge, so callers can locate the
//! correction stream's bytes without re-implementing the metadata walk.
//!
//! ## Length: unconstrained by the wiki
//!
//! The wiki places no size constraint on the `0x0B` payload (the encoder
//! packs however many correction codewords match the main-stream sample
//! count into however many bytes it chose). Even the empty payload (zero
//! bytes) is structurally valid — it would mean the encoder produced no
//! correction codewords for this block. The constructor therefore
//! accepts any byte slice (including the empty one) without rejection.
//!
//! ## Bit order
//!
//! Bits are consumed least-significant-bit first within each byte and
//! bytes in stream order — the same convention the main `0x0A` reader
//! uses per the [`crate::samples`] module-level bit-order documentation.
//! The [`Self::bit_reader`] factory honours that convention exactly by
//! constructing a [`crate::BitReader`] over the payload bytes.

use crate::samples::BitReader;

/// Typed view of the `0x0B` packed-correction-data sub-block payload —
/// the entropy-coded correction bitstream the wiki "IDs" listing
/// annotates as carried in the `.wvc` companion file.
///
/// The view borrows the underlying payload bytes verbatim from the
/// round-2 metadata walker; constructing it does no work beyond storing
/// the slice reference. Use [`Self::bit_reader`] to obtain a fresh
/// [`BitReader`] positioned at bit 0 for callers staging the
/// hybrid-mode decode pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PackedCorrectionData<'a> {
    bytes: &'a [u8],
}

impl<'a> PackedCorrectionData<'a> {
    /// Construct a packed-correction-data view over the byte slice the
    /// round-2 walker handed back for a `0x0B` sub-block. The wiki
    /// places no length constraint on the payload (the codeword count
    /// is conveyed out-of-band by the block header's `block_samples`),
    /// so any byte slice — including the empty one — is accepted
    /// without rejection.
    pub const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }

    /// The underlying payload bytes — exactly the slice the round-2
    /// walker handed back for the `0x0B` sub-block.
    pub const fn bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// Byte length of the payload.
    pub const fn len(&self) -> usize {
        self.bytes.len()
    }

    /// `true` when the payload is empty (no entropy-coded correction
    /// bits to decode). The wiki does not forbid a zero-byte `0x0B`
    /// payload — it would mean the encoder packed no correction
    /// codewords into this block.
    pub const fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Construct a fresh [`BitReader`] positioned at bit 0 of the
    /// payload, ready to feed a hybrid-mode decode pass.
    ///
    /// Each call returns an independent reader — the
    /// [`PackedCorrectionData`] view itself carries no read cursor, so
    /// multiple bit readers can be constructed from the same view
    /// (e.g. one for a probe and one for the real decode).
    pub fn bit_reader(&self) -> BitReader<'a> {
        BitReader::new(self.bytes)
    }
}

/// Construct a [`PackedCorrectionData`] view over a `0x0B` sub-block
/// payload.
///
/// Typed counterpart to the round-2 walker output for the `0x0B` ID —
/// analogous to the round-12 [`crate::expand_packed_samples`] for
/// `0x0A`. The wiki places no internal structure on the `0x0B`
/// payload, so the "expansion" is a typed wrap rather than a
/// byte-by-byte decode. The hybrid-mode sample decode that would
/// consume the wrapped bytes remains gated on the
/// [`crate::UnsupportedBlockFeature::Hybrid`] feature refusal.
pub fn expand_packed_correction_data(payload: &[u8]) -> PackedCorrectionData<'_> {
    PackedCorrectionData::new(payload)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_preserves_bytes_verbatim() {
        let bytes = [0x11, 0x22, 0x33];
        let view = PackedCorrectionData::new(&bytes);
        assert_eq!(view.bytes(), &[0x11, 0x22, 0x33]);
        assert_eq!(view.len(), 3);
        assert!(!view.is_empty());
    }

    #[test]
    fn empty_payload_is_accepted_and_reported_empty() {
        // The wiki places no length constraint on the 0x0B payload, so
        // a zero-byte slice is a valid (degenerate) view.
        let view = PackedCorrectionData::new(&[]);
        assert!(view.is_empty());
        assert_eq!(view.len(), 0);
        assert_eq!(view.bytes(), &[] as &[u8]);
    }

    #[test]
    fn expand_packed_correction_data_round_trips_the_byte_slice() {
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        let view = expand_packed_correction_data(&payload);
        assert_eq!(view.bytes(), &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(view.len(), 4);
    }

    #[test]
    fn bit_reader_starts_at_byte_zero_bit_zero() {
        let payload = [0xFF, 0xFF];
        let view = PackedCorrectionData::new(&payload);
        let r = view.bit_reader();
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 0);
        assert_eq!(r.bits_remaining(), 16);
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn bit_reader_yields_first_bit_lsb_first() {
        // 0x06 = 0b0000_0110 -> LSB first: 0, 1, 1, 0, 0, ...
        let payload = [0x06u8];
        let view = PackedCorrectionData::new(&payload);
        let mut r = view.bit_reader();
        assert_eq!(r.get_bit().unwrap(), 0);
        assert_eq!(r.get_bit().unwrap(), 1);
        assert_eq!(r.get_bit().unwrap(), 1);
        assert_eq!(r.get_bit().unwrap(), 0);
    }

    #[test]
    fn bit_reader_factory_returns_independent_readers() {
        // Multiple bit_reader() calls yield independent readers — the
        // view itself carries no cursor.
        let payload = [0xFFu8];
        let view = PackedCorrectionData::new(&payload);
        let mut probe = view.bit_reader();
        let mut real = view.bit_reader();
        assert_eq!(probe.get_bit().unwrap(), 1);
        assert_eq!(real.byte_position(), 0);
        assert_eq!(real.bit_position(), 0);
        assert_eq!(real.get_bit().unwrap(), 1);
    }

    #[test]
    fn empty_payload_bit_reader_reports_immediate_truncation() {
        let view = PackedCorrectionData::new(&[]);
        let mut r = view.bit_reader();
        assert!(r.is_empty());
        assert_eq!(r.bits_remaining(), 0);
        // Any read against an empty view should report truncation — no
        // zero-fill contract.
        assert_eq!(r.get_bit(), Err(crate::Error::Truncated));
    }

    #[test]
    fn view_is_copy_and_independent_of_caller_lifetime() {
        let payload = [0xCA, 0xFE, 0xBA, 0xBE];
        let view = PackedCorrectionData::new(&payload);
        let copy = view;
        assert_eq!(view.len(), copy.len());
        assert_eq!(view.bytes(), copy.bytes());
    }

    #[test]
    fn view_is_distinct_type_from_packed_samples() {
        // Compile-time check: the correction view's typed handle is a
        // separate type from PackedSamples even though both wrap the
        // same byte-slice shape. The wiki distinguishes the two streams
        // by ID (`0x0A` vs `0x0B`) and the lossless decoder ignores
        // `0x0B` entirely — keeping the types distinct guards against
        // accidentally feeding `0x0B` bytes into the `0x0A` decode loop.
        let payload = [0u8; 4];
        let correction = PackedCorrectionData::new(&payload);
        let samples = crate::PackedSamples::new(&payload);
        assert_eq!(correction.bytes(), samples.bytes());
        assert_eq!(correction.len(), samples.len());
    }
}
