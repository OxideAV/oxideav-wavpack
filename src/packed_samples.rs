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
//! ## Length: the spec §1 even-byte rule
//!
//! The wiki "Samples coding" section places **no** numeric size on the
//! `0x0A` payload (the sample count is determined by the block header's
//! `block_samples`, and the entropy-coded bitstream packs
//! `block_samples` Golomb codewords into however many bytes the encoder
//! chose). The clean-room entropy doc
//! `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 narrows that
//! with one structural rule: the main-bitstream payload "byte length
//! must be even or the block is rejected" — the reader binds the `0x0A`
//! payload to the per-stream main bitstream as 16-bit words. (The
//! round-2 metadata walker has already stripped the optional odd-size
//! *framing* padding byte, so an odd length here is an odd *payload*,
//! distinct from the framing pad.)
//!
//! [`PackedSamples::new`] is a verbatim, infallible view constructor and
//! does **not** apply that rule (it stays a `const fn` over any slice,
//! so a caller probing a payload — including the empty one — can always
//! wrap it). The even-byte rule is surfaced separately as the
//! [`PackedSamples::is_even_length`] predicate, the
//! [`PackedSamples::validate_length`] checked accessor, and the
//! [`validate_packed_samples`] free constructor, and is enforced by the
//! round-206 [`crate::WavPackBlock::decode_samples`] composer before the
//! per-sample loop runs. The empty payload (zero bytes) is even and so
//! passes — it would mean a metadata-only block, or a block whose
//! `block_samples` is zero.
//!
//! ## Bit order
//!
//! Bits are consumed least-significant-bit first within each byte and
//! bytes in stream order — see the [`crate::samples`] module-level
//! "bit order within a byte" docs-gap note. The [`Self::bit_reader`]
//! factory honours that convention exactly because it constructs a
//! [`crate::BitReader`] which is the only reader the crate exposes.

use crate::error::{Error, Result};
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

    /// `true` when the payload length is even, the structural rule the
    /// clean-room entropy doc
    /// `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 places on
    /// the `0x0A` main bitstream ("byte length must be even or the block
    /// is rejected"). The empty payload (zero bytes) is even and so
    /// reports `true`.
    ///
    /// Pure predicate: reads nothing and mutates nothing. Use
    /// [`Self::validate_length`] for the checked-accessor form that
    /// surfaces [`Error::PackedSamplesOddLength`] on an odd payload.
    pub const fn is_even_length(&self) -> bool {
        self.bytes.len() % 2 == 0
    }

    /// Return `Ok(*self)` when the payload satisfies the spec §1 even-byte
    /// rule (see [`Self::is_even_length`]), or
    /// [`Error::PackedSamplesOddLength`] carrying the observed byte count
    /// when the payload is odd.
    ///
    /// The view itself is always constructible from any slice via
    /// [`Self::new`]; this accessor is the explicit gate a decode path
    /// applies before binding the payload to the per-sample loop. The
    /// round-206 [`crate::WavPackBlock::decode_samples`] composer calls it
    /// for exactly that reason.
    pub const fn validate_length(&self) -> Result<Self> {
        if self.is_even_length() {
            Ok(Self { bytes: self.bytes })
        } else {
            Err(Error::PackedSamplesOddLength(self.bytes.len()))
        }
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

/// Construct a [`PackedSamples`] view over a `0x0A` sub-block payload,
/// applying the clean-room entropy doc
/// `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 even-byte rule
/// ("byte length must be even or the block is rejected").
///
/// The checked counterpart to [`expand_packed_samples`]: returns the
/// typed view for an even-length (including empty) payload and
/// [`Error::PackedSamplesOddLength`] carrying the observed byte count for
/// an odd-length payload. Equivalent to
/// `PackedSamples::new(payload).validate_length()`.
pub const fn validate_packed_samples(payload: &[u8]) -> Result<PackedSamples<'_>> {
    PackedSamples::new(payload).validate_length()
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
    fn is_even_length_classifies_payload_parity() {
        // Empty is even (spec §1 even-byte rule: empty passes).
        assert!(PackedSamples::new(&[]).is_even_length());
        // Even lengths pass.
        assert!(PackedSamples::new(&[0x00, 0x00]).is_even_length());
        assert!(PackedSamples::new(&[0x01, 0x02, 0x03, 0x04]).is_even_length());
        // Odd lengths fail.
        assert!(!PackedSamples::new(&[0x00]).is_even_length());
        assert!(!PackedSamples::new(&[0x01, 0x02, 0x03]).is_even_length());
    }

    #[test]
    fn validate_length_accepts_even_and_empty_payloads() {
        let empty = PackedSamples::new(&[]);
        assert_eq!(empty.validate_length(), Ok(empty));

        let even = PackedSamples::new(&[0xDE, 0xAD]);
        let validated = even.validate_length().expect("even payload validates");
        assert_eq!(validated.bytes(), &[0xDE, 0xAD]);
    }

    #[test]
    fn validate_length_rejects_odd_payload_with_observed_count() {
        let odd = PackedSamples::new(&[0x01, 0x02, 0x03]);
        assert_eq!(
            odd.validate_length(),
            Err(crate::Error::PackedSamplesOddLength(3))
        );
        // One-byte payload reports a count of 1.
        assert_eq!(
            PackedSamples::new(&[0xFF]).validate_length(),
            Err(crate::Error::PackedSamplesOddLength(1))
        );
    }

    #[test]
    fn validate_packed_samples_free_fn_matches_method() {
        let even = [0x11, 0x22, 0x33, 0x44];
        assert_eq!(
            validate_packed_samples(&even),
            PackedSamples::new(&even).validate_length()
        );
        let odd = [0x11, 0x22, 0x33];
        assert_eq!(
            validate_packed_samples(&odd),
            Err(crate::Error::PackedSamplesOddLength(3))
        );
        // Empty passes the free constructor too.
        assert!(validate_packed_samples(&[]).is_ok());
    }

    #[test]
    fn validate_length_is_const_evaluable() {
        const EVEN: Result<PackedSamples<'static>> =
            PackedSamples::new(&[0xAA, 0xBB]).validate_length();
        const ODD: Result<PackedSamples<'static>> = PackedSamples::new(&[0xAA]).validate_length();
        assert!(EVEN.is_ok());
        assert_eq!(ODD, Err(crate::Error::PackedSamplesOddLength(1)));
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
