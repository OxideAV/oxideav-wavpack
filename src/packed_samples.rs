//! WavPack v.4 packed-samples sub-block typed view (ID `0x0A`).
//!
//! The wiki "Samples coding" section
//! (`docs/audio/wavpack/wiki/WavPack.wiki`) names the entropy-coded audio
//! payload by its metadata-sub-block ID:
//!
//! > Samples are stored in metadata block with ID=0x0A and are packed
//! > with modified Golomb codes.
//!
//! and gives the per-sample decode pseudocode that [`crate::samples`]
//! implements (the [`crate::BitReader`] primitives, the
//! [`crate::decode_run_length`] state machine and the
//! [`crate::decode_sample_value`] Golomb reconstructor). The
//! sub-block payload itself — the byte slice the round-2 walker hands
//! back for the `0x0A` ID — has no further internal structure named by
//! the wiki: it is simply the contiguous bit-stream the per-sample
//! pseudocode consumes left-to-right.
//!
//! [`PackedSamples`] elaborates that byte slice into a typed view so
//! callers staging the deferred prediction loop have a named handle
//! distinct from a bare `&[u8]`. It carries:
//!
//! * The raw bytes (`bytes()`), preserved verbatim from the walker.
//! * A `bit_reader()` factory that produces a fresh
//!   [`crate::BitReader`] positioned at bit 0 of the payload, ready to
//!   feed [`crate::decode_run_length`] / [`crate::decode_sample_value`]
//!   / [`crate::decode_sample`].
//! * `len()` / `is_empty()` byte-length introspection for callers
//!   sizing buffers or short-circuiting the (vacuous) empty case.
//!
//! No bytes are read, no state is mutated and no docs-gap is closed by
//! this round — the typed view exists to give the round-2 walker output
//! a single concrete handoff into the round-5/6/7 bit-reader and Golomb
//! decoder. The median-adaptation amount docs gap that gates the
//! stateful payload loop is unchanged.
//!
//! ## Length: unconstrained by the wiki
//!
//! The wiki places **no** size constraint on the `0x0A` payload (the
//! sample count is determined by the block header's `block_samples`,
//! and the entropy-coded bitstream packs `block_samples` Golomb
//! codewords into however many bytes the encoder chose). Even the empty
//! payload (zero bytes) is structurally valid — it would mean a
//! metadata-only block, or a block whose `block_samples` is zero. The
//! constructor therefore accepts any byte slice (including the empty
//! one) without rejection.
//!
//! ## Bit order
//!
//! Bits are consumed least-significant-bit first within each byte and
//! bytes in stream order — see the [`crate::samples`] module-level
//! "bit order within a byte" docs-gap note. The [`Self::bit_reader`]
//! factory honours that convention exactly because it constructs a
//! [`crate::BitReader`] which is the only reader the crate exposes.

use crate::samples::BitReader;

/// Typed view of the `0x0A` packed-samples sub-block payload — the
/// entropy-coded audio bitstream the wiki "Samples coding" section
/// consumes.
///
/// The view borrows the underlying payload bytes verbatim from the
/// round-2 metadata walker; constructing it does no work beyond storing
/// the slice reference. Use [`Self::bit_reader`] to obtain a fresh
/// [`BitReader`] positioned at bit 0 for feeding the per-sample
/// pseudocode in [`crate::decode_sample`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PackedSamples<'a> {
    bytes: &'a [u8],
}

impl<'a> PackedSamples<'a> {
    /// Construct a packed-samples view over the byte slice the round-2
    /// walker handed back for a `0x0A` sub-block. The wiki places no
    /// length constraint on the payload (the sample count is conveyed
    /// out-of-band by the block header's `block_samples`), so any byte
    /// slice — including the empty one — is accepted without rejection.
    pub const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }

    /// The underlying payload bytes — exactly the slice the round-2
    /// walker handed back for the `0x0A` sub-block.
    pub const fn bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// Byte length of the payload.
    pub const fn len(&self) -> usize {
        self.bytes.len()
    }

    /// `true` when the payload is empty (no entropy-coded bits to
    /// decode). The wiki does not forbid a zero-byte `0x0A` payload —
    /// it would mean a metadata-only block whose `block_samples` is
    /// zero, or a block where the encoder packed no codewords into the
    /// `0x0A` sub-block.
    pub const fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    /// Construct a fresh [`BitReader`] positioned at bit 0 of the
    /// payload, ready to feed [`crate::decode_run_length`] /
    /// [`crate::decode_sample_value`] / [`crate::decode_sample`].
    ///
    /// Each call returns an independent reader — the [`PackedSamples`]
    /// view itself carries no read cursor, so multiple bit readers can
    /// be constructed from the same view (e.g. one for a probe and one
    /// for the real decode).
    pub fn bit_reader(&self) -> BitReader<'a> {
        BitReader::new(self.bytes)
    }
}

/// Construct a [`PackedSamples`] view over a `0x0A` sub-block payload.
///
/// This is the typed counterpart to the round-2 walker output for the
/// `0x0A` ID — analogous to the round-3 [`crate::expand_samples`] /
/// round-4 [`crate::expand_entropy`] expanders for `0x04` / `0x05`, but
/// the wiki places no internal structure on the `0x0A` payload so the
/// "expansion" is a typed wrap rather than a byte-by-byte decode. The
/// stateful per-sample decode is gated on the median-adaptation amount
/// docs gap and lives in [`crate::decode_sample`] / `decode_run_length`
/// / `decode_sample_value`.
pub fn expand_packed_samples(payload: &[u8]) -> PackedSamples<'_> {
    PackedSamples::new(payload)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_preserves_bytes_verbatim() {
        let bytes = [0xAA, 0xBB, 0xCC];
        let view = PackedSamples::new(&bytes);
        assert_eq!(view.bytes(), &[0xAA, 0xBB, 0xCC]);
        assert_eq!(view.len(), 3);
        assert!(!view.is_empty());
    }

    #[test]
    fn empty_payload_is_accepted_and_reported_empty() {
        // The wiki places no length constraint on the 0x0A payload, so
        // a zero-byte slice is a valid (degenerate) packed-samples view.
        let view = PackedSamples::new(&[]);
        assert!(view.is_empty());
        assert_eq!(view.len(), 0);
        assert_eq!(view.bytes(), &[] as &[u8]);
    }

    #[test]
    fn expand_packed_samples_round_trips_the_byte_slice() {
        let payload = [0x01, 0x02, 0x03, 0x04];
        let view = expand_packed_samples(&payload);
        assert_eq!(view.bytes(), &[0x01, 0x02, 0x03, 0x04]);
        assert_eq!(view.len(), 4);
    }

    #[test]
    fn bit_reader_starts_at_byte_zero_bit_zero() {
        let payload = [0xFF, 0xFF];
        let view = PackedSamples::new(&payload);
        let r = view.bit_reader();
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 0);
        assert_eq!(r.bits_remaining(), 16);
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn bit_reader_yields_first_bit_lsb_first() {
        // 0x05 = 0b0000_0101 → LSB first: 1, 0, 1, 0, 0, ...
        let payload = [0x05u8];
        let view = PackedSamples::new(&payload);
        let mut r = view.bit_reader();
        assert_eq!(r.get_bit().unwrap(), 1);
        assert_eq!(r.get_bit().unwrap(), 0);
        assert_eq!(r.get_bit().unwrap(), 1);
    }

    #[test]
    fn bit_reader_factory_returns_independent_readers() {
        // Multiple bit_reader() calls yield independent readers — the
        // view itself carries no cursor, so a probe and the real decode
        // can each start at bit 0.
        let payload = [0xFFu8];
        let view = PackedSamples::new(&payload);
        let mut probe = view.bit_reader();
        let mut real = view.bit_reader();
        assert_eq!(probe.get_bit().unwrap(), 1);
        // probe is now one bit in; real should still be at bit 0.
        assert_eq!(real.byte_position(), 0);
        assert_eq!(real.bit_position(), 0);
        assert_eq!(real.get_bit().unwrap(), 1);
    }

    #[test]
    fn empty_payload_bit_reader_reports_immediate_truncation() {
        let view = PackedSamples::new(&[]);
        let mut r = view.bit_reader();
        assert!(r.is_empty());
        assert_eq!(r.bits_remaining(), 0);
        // Any read against an empty packed-samples view should report
        // truncation — the wiki places no zero-fill contract.
        assert_eq!(r.get_bit(), Err(crate::Error::Truncated));
    }

    #[test]
    fn view_is_copy_and_independent_of_caller_lifetime() {
        // The view stores a slice reference; copying it does not
        // re-borrow.
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        let view = PackedSamples::new(&payload);
        let copy = view;
        assert_eq!(view.len(), copy.len());
        assert_eq!(view.bytes(), copy.bytes());
    }
}
