//! WavPack v.4 end-to-end block parser — header + metadata-walker aggregate.
//!
//! Rounds 1 and 2 parse the two halves of a WavPack block — the 32-byte
//! fixed header ([`parse_block_header`](crate::parse_block_header)) and
//! the metadata sub-block region that follows it
//! ([`walk_metadata`](crate::walk_metadata)). This module composes them
//! into a single [`parse_block`] call returning a [`WavPackBlock`] that
//! owns the typed header alongside the parsed sub-blocks, plus the
//! unconsumed tail (the next block in a `.wv` file).
//!
//! The on-disk relationship between the two halves is documented in the
//! wiki "Block structure" listing of
//! `docs/audio/wavpack/wiki/WavPack.wiki`:
//!
//! > 32 bits - total block size (not counting this field or 'wvpk')
//!
//! `ck_size` covers every byte of the block **except** the four magic
//! bytes and `ck_size` itself, so the metadata-sub-block region's length
//! is `ck_size - 24` (the wiki-fixed 24-byte tail of the header that
//! follows `ck_size`). [`parse_block`] uses that count to extract the
//! sub-block region exactly, then hands it to the walker.
//!
//! ### What this round does **not** do
//!
//! `parse_block` is a structural composer — it asserts the header
//! parses, the byte count described by `ck_size` is present in the
//! input, and every metadata sub-block walks cleanly. It does **not**
//! decode samples, expand decorrelation triples, verify the CRC32, or
//! pair `.wvc` correction blocks: every payload interpretation
//! continues to live in its own module
//! ([`decorrelation`](crate::decorrelation),
//! [`entropy`](crate::entropy), [`samples`](crate::samples),
//! [`packed_samples`](crate::packed_samples), [`metadata`](crate::metadata)
//! for [`Md5Checksum`](crate::Md5Checksum)).
//!
//! The median-adaptation amount remains the open docs gap blocking the
//! per-sample loop the wiki "Samples coding" section depends on; this
//! round adds no sample-level state and so is not impacted by that gap.

use crate::block_header::{parse_block_header, WavPackBlockHeader, HEADER_LEN};
use crate::error::{Error, Result};
use crate::metadata::{walk_metadata, MetadataSubBlock, SubBlockId};

/// One fully-parsed WavPack block — the typed fixed header plus the
/// walked metadata sub-block list.
///
/// Carries borrowed payload slices (`MetadataSubBlock<'a>` points into
/// the same input byte slice that was passed to [`parse_block`]) so the
/// per-sub-block payload bytes don't need to be copied. [`Self::header`]
/// and [`Self::sub_blocks`] are the two structural fields the rounds-1
/// and rounds-2 parsers already produced; this aggregate just bundles
/// them.
#[derive(Debug, Clone)]
pub struct WavPackBlock<'a> {
    /// Fixed-size block header (round 1).
    pub header: WavPackBlockHeader,
    /// Metadata sub-blocks (round 2), in on-disk order. Empty when the
    /// block carries no metadata region (`ck_size == 24`, a header-only
    /// block) — a valid edge case the wiki preamble allows when
    /// `block_samples == 0`.
    pub sub_blocks: Vec<MetadataSubBlock<'a>>,
}

impl<'a> WavPackBlock<'a> {
    /// The fixed-size header for this block. Convenience accessor for
    /// callers that prefer a method to a field.
    pub fn header(&self) -> &WavPackBlockHeader {
        &self.header
    }

    /// The parsed metadata sub-blocks in on-disk order.
    pub fn sub_blocks(&self) -> &[MetadataSubBlock<'a>] {
        &self.sub_blocks
    }

    /// `true` when at least one metadata sub-block with `id` is present
    /// in the parsed sub-block list. A boolean shortcut over
    /// [`crate::find_first`] for callers that only need to test for
    /// presence (e.g. checking whether a `0x26` MD5 sub-block was
    /// emitted by the encoder).
    pub fn contains_sub_block(&self, id: SubBlockId) -> bool {
        self.sub_blocks.iter().any(|s| s.id == id)
    }

    /// Number of parsed metadata sub-blocks. Equivalent to
    /// `self.sub_blocks().len()` but spelled directly for callers that
    /// just want the count (e.g. an integrity check against a per-block
    /// sub-block budget).
    pub fn sub_block_count(&self) -> usize {
        self.sub_blocks.len()
    }

    /// `true` when no metadata sub-blocks were parsed — the
    /// `ck_size == 24` edge case the wiki "Block structure" listing
    /// allows when the block carries no metadata region at all (the
    /// fixed 24 bytes after `ck_size` cover only the header tail). Pairs
    /// with [`crate::WavPackBlockHeader::is_audio_block`] to distinguish
    /// a metadata-bare header-only block from a metadata-only block
    /// carrying RIFF / MD5 sub-blocks but no audio.
    pub fn is_metadata_empty(&self) -> bool {
        self.sub_blocks.is_empty()
    }

    /// On-disk length of this entire block in bytes — the four magic
    /// bytes plus the `ck_size` field plus the `ck_size`-spanned tail.
    ///
    /// The wiki "Block structure" defines `ck_size` as "total block size
    /// (not counting this field or 'wvpk')", so the full on-disk length
    /// is `8 + ck_size` (four magic bytes + four `ck_size` bytes + the
    /// `ck_size` value itself). Useful for callers that want to advance
    /// past this block to the next without re-parsing the header.
    pub fn on_disk_len(&self) -> u64 {
        // u32 -> u64 to keep the arithmetic free of overflow for the
        // pathological case ck_size = u32::MAX (legal on the wire even
        // if the wiki never produces it in practice).
        8u64 + self.header.ck_size as u64
    }
}

/// Parse one full WavPack block — the 32-byte fixed header plus the
/// metadata sub-block region the wiki "Block structure" listing
/// describes — and return it as a typed [`WavPackBlock`] alongside the
/// unconsumed tail.
///
/// The block's on-disk byte count is `8 + ck_size` per the wiki: four
/// magic bytes + the `ck_size` field itself + the `ck_size`-spanned
/// tail (version through CRC + metadata sub-blocks). [`parse_block`]
/// validates that the input contains at least that many bytes, walks
/// the metadata sub-block region (bytes `[32..8 + ck_size]`), and
/// returns the tail `bytes[8 + ck_size..]` for chained decoding of a
/// multi-block file.
///
/// Errors:
///
/// * [`Error::Truncated`] — input is shorter than the 32-byte fixed
///   header (raised by [`parse_block_header`]), or shorter than the
///   `8 + ck_size` total length implied by the header.
/// * [`Error::InvalidMagic`] / [`Error::InvalidCkSize`] /
///   [`Error::UnsupportedVersion`] — header rejections per
///   [`parse_block_header`].
/// * [`Error::CkSizeExceedsBuffer`] — the input is long enough for the
///   32-byte header but too short for the `ck_size`-declared payload
///   region. Reported separately from [`Error::Truncated`] so a caller
///   feeding a multi-block file from a buffered reader can distinguish
///   "need more bytes for this block's payload" (this variant) from
///   "need more bytes for the next block's header" (the buffer ran out
///   between blocks).
/// * Any metadata-walker error from [`walk_metadata`] applied to the
///   payload region.
pub fn parse_block(bytes: &[u8]) -> Result<(WavPackBlock<'_>, &[u8])> {
    let (header, after_header) = parse_block_header(bytes)?;
    // The wiki "Block structure" definition of ck_size: the 24 bytes
    // after ck_size itself (version through CRC) plus all the metadata
    // sub-block bytes. payload_bytes() subtracts that fixed 24 to give
    // the metadata-region length. parse_block_header has already
    // validated ck_size >= 24, so payload_bytes() does not underflow.
    let payload_bytes = header.payload_bytes() as usize;
    if after_header.len() < payload_bytes {
        return Err(Error::CkSizeExceedsBuffer {
            ck_size: header.ck_size,
            available: HEADER_LEN + after_header.len(),
        });
    }
    let (payload, tail) = after_header.split_at(payload_bytes);
    let sub_blocks = walk_metadata(payload)?;
    Ok((WavPackBlock { header, sub_blocks }, tail))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block_header::{HEADER_LEN, MAGIC, MIN_CK_SIZE};

    /// Synthesise a minimal valid WavPack block header with the supplied
    /// `ck_size` (must be `>= 24`). The flag word is left at zero (mono,
    /// 1-byte samples, no extra flags); track / sample fields are zero.
    fn synthesise_header_bytes(ck_size: u32) -> Vec<u8> {
        let mut buf = vec![0u8; HEADER_LEN];
        buf[0..4].copy_from_slice(MAGIC);
        buf[4..8].copy_from_slice(&ck_size.to_le_bytes());
        // version 0x0410 (top of the wiki-valid range).
        buf[8..10].copy_from_slice(&0x0410u16.to_le_bytes());
        // track_number, track_sub_index, total_samples, block_index,
        // block_samples, flags, crc all stay zero.
        buf
    }

    /// Append a small (non-large, non-odd) sub-block to a payload buffer.
    /// `id_byte` is written verbatim; `payload` becomes the sub-block
    /// body. Returns the on-disk byte length.
    fn append_small_sub_block(buf: &mut Vec<u8>, id_byte: u8, payload: &[u8]) -> usize {
        // size_words = payload.len() / 2 (wiki: every metadata block has
        // even length, size is in 16-bit words). Tests use even-length
        // payloads only.
        assert!(payload.len() % 2 == 0, "tests use even-length payloads");
        let words = (payload.len() / 2) as u8;
        buf.push(id_byte);
        buf.push(words);
        buf.extend_from_slice(payload);
        2 + payload.len()
    }

    #[test]
    fn parse_block_returns_header_and_empty_metadata_on_min_ck_size() {
        // ck_size == 24: header-only block, no metadata sub-blocks. The
        // wiki "Block structure" allows this when block_samples == 0.
        let bytes = synthesise_header_bytes(MIN_CK_SIZE);
        let (block, tail) = parse_block(&bytes).expect("parse min block");
        assert_eq!(block.header.ck_size, MIN_CK_SIZE);
        assert!(block.is_metadata_empty());
        assert_eq!(block.sub_block_count(), 0);
        assert_eq!(block.sub_blocks().len(), 0);
        assert!(tail.is_empty());
    }

    #[test]
    fn parse_block_walks_metadata_region_correctly() {
        // Build a block with two sub-blocks: a 4-byte dummy (0x00) and a
        // 16-byte MD5 sum (0x26). ck_size = 24 + (2 + 4) + (2 + 16).
        let mut payload = Vec::new();
        let len_a = append_small_sub_block(&mut payload, 0x00, &[0u8; 4]);
        let len_b = append_small_sub_block(&mut payload, 0x26, &[0xAAu8; 16]);
        assert_eq!(len_a + len_b, payload.len());
        let ck_size = (24 + payload.len()) as u32;
        let mut bytes = synthesise_header_bytes(ck_size);
        bytes.extend_from_slice(&payload);

        let (block, tail) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.header.ck_size, ck_size);
        assert_eq!(block.sub_blocks().len(), 2);
        assert!(!block.is_metadata_empty());
        assert_eq!(block.sub_blocks()[0].id, SubBlockId::Dummy);
        assert_eq!(block.sub_blocks()[1].id, SubBlockId::Md5Checksum);
        assert!(block.contains_sub_block(SubBlockId::Md5Checksum));
        assert!(!block.contains_sub_block(SubBlockId::EntropyInfo));
        assert!(tail.is_empty());
    }

    #[test]
    fn parse_block_returns_tail_for_next_block() {
        // Two minimal back-to-back blocks. parse_block on the
        // concatenation should yield the first block and a tail equal to
        // the second block's bytes verbatim.
        let first = synthesise_header_bytes(MIN_CK_SIZE);
        let second = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bytes = first.clone();
        bytes.extend_from_slice(&second);

        let (block, tail) = parse_block(&bytes).expect("parse first block");
        assert_eq!(block.header.ck_size, MIN_CK_SIZE);
        assert_eq!(tail, second.as_slice());

        // The tail itself should parse as the second block.
        let (block2, tail2) = parse_block(tail).expect("parse second block");
        assert_eq!(block2.header.ck_size, MIN_CK_SIZE);
        assert!(tail2.is_empty());
    }

    #[test]
    fn parse_block_rejects_truncated_header() {
        // Buffer shorter than HEADER_LEN: parse_block_header rejects it
        // with Truncated before parse_block sees ck_size.
        let bytes = [0u8; HEADER_LEN - 1];
        let err = parse_block(&bytes).expect_err("must reject truncated header");
        assert_eq!(err, Error::Truncated);
    }

    #[test]
    fn parse_block_rejects_when_ck_size_exceeds_buffer() {
        // Header advertises a 200-byte ck_size, but the buffer has only
        // the 32-byte header. The new CkSizeExceedsBuffer variant
        // captures both fields so a streaming caller can request the
        // missing bytes.
        let ck_size = 200u32;
        let bytes = synthesise_header_bytes(ck_size);
        let err = parse_block(&bytes).expect_err("must reject short payload");
        match err {
            Error::CkSizeExceedsBuffer {
                ck_size: e_ck_size,
                available,
            } => {
                assert_eq!(e_ck_size, ck_size);
                assert_eq!(available, HEADER_LEN);
            }
            other => panic!("expected CkSizeExceedsBuffer, got {other:?}"),
        }
    }

    #[test]
    fn parse_block_propagates_invalid_magic_from_header() {
        let mut bytes = synthesise_header_bytes(MIN_CK_SIZE);
        bytes[0] = b'X';
        let err = parse_block(&bytes).expect_err("must reject bad magic");
        assert_eq!(err, Error::InvalidMagic);
    }

    #[test]
    fn parse_block_propagates_invalid_ck_size_from_header() {
        let bytes = synthesise_header_bytes(23);
        let err = parse_block(&bytes).expect_err("must reject ck_size");
        assert_eq!(err, Error::InvalidCkSize(23));
    }

    #[test]
    fn parse_block_propagates_walker_error_on_malformed_sub_block() {
        // ck_size accommodates a sub-block whose ID byte advertises one
        // word (2 bytes) but the buffer only has the ID + size fields,
        // no payload — walk_metadata should report Truncated.
        let payload = vec![0x00, 0x01]; // ID 0x00 dummy, size = 1 word (2 bytes), no body
        let ck_size = (24 + payload.len()) as u32;
        let mut bytes = synthesise_header_bytes(ck_size);
        bytes.extend_from_slice(&payload);
        // We deliberately give the outer block payload_bytes = 2 (just
        // the sub-block header). The walker sees ID + size_words=1 but
        // no payload bytes → Truncated.
        let err = parse_block(&bytes).expect_err("must reject malformed sub-block");
        assert_eq!(err, Error::Truncated);
    }

    #[test]
    fn on_disk_len_equals_eight_plus_ck_size() {
        let ck_size = 200u32;
        // Synthesise enough bytes that parse_block succeeds — i.e. fill
        // the metadata region with a sequence of dummy sub-blocks. Easier:
        // build payload as one big dummy sub-block of exactly
        // (ck_size - 24) bytes minus the 2-byte sub-block header.
        let payload_len = (ck_size as usize) - 24;
        let body_len = payload_len - 2; // subtract the 2-byte sub-block header
        let mut payload = Vec::with_capacity(payload_len);
        payload.push(0x00); // Dummy ID
        assert!(body_len % 2 == 0, "test body must be even-length");
        assert!(body_len / 2 <= u8::MAX as usize, "body fits in 1-byte size");
        payload.push((body_len / 2) as u8);
        payload.extend(std::iter::repeat_n(0u8, body_len));
        assert_eq!(payload.len(), payload_len);

        let mut bytes = synthesise_header_bytes(ck_size);
        bytes.extend_from_slice(&payload);

        let (block, tail) = parse_block(&bytes).expect("parse block");
        assert!(tail.is_empty());
        // on_disk_len = 8 + ck_size — four magic bytes + four ck_size
        // bytes + ck_size value itself (the wiki "Block structure"
        // definition).
        assert_eq!(block.on_disk_len(), 8 + ck_size as u64);
        assert_eq!(block.on_disk_len() as usize, bytes.len());
    }

    #[test]
    fn contains_sub_block_returns_false_on_empty_metadata() {
        let bytes = synthesise_header_bytes(MIN_CK_SIZE);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.contains_sub_block(SubBlockId::PackedSamples));
        assert!(!block.contains_sub_block(SubBlockId::EntropyInfo));
        assert!(!block.contains_sub_block(SubBlockId::Md5Checksum));
    }

    #[test]
    fn sub_block_count_matches_walker_output_count() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x00, &[0u8; 2]);
        append_small_sub_block(&mut payload, 0x05, &[0u8; 12]); // mono entropy
        append_small_sub_block(&mut payload, 0x0A, &[0u8; 4]);
        append_small_sub_block(&mut payload, 0x26, &[0xCDu8; 16]);
        let ck_size = (24 + payload.len()) as u32;
        let mut bytes = synthesise_header_bytes(ck_size);
        bytes.extend_from_slice(&payload);

        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.sub_block_count(), 4);
        // The two-step round-trip (parse_block -> finders) reuses the
        // existing round-10 finders so all earlier predicates still work.
        assert!(block.contains_sub_block(SubBlockId::Dummy));
        assert!(block.contains_sub_block(SubBlockId::EntropyInfo));
        assert!(block.contains_sub_block(SubBlockId::PackedSamples));
        assert!(block.contains_sub_block(SubBlockId::Md5Checksum));
    }
}
