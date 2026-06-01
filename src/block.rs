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
use crate::entropy::expand_entropy;
use crate::error::{Error, Result};
use crate::metadata::{
    find_entropy_info, find_packed_samples, walk_metadata, MetadataSubBlock, SubBlockId,
};
use crate::samples::{
    decode_packed_samples_mono_from_entropy, decode_packed_samples_stereo_from_entropy,
};

/// Named WavPack v.4 feature the round-15/199 per-sample loop does not
/// yet support, surfaced through
/// [`Error::UnsupportedBlockFeature`](crate::Error::UnsupportedBlockFeature)
/// when [`WavPackBlock::decode_samples`] refuses a structurally
/// well-formed block. Lets the caller report a specific diagnostic
/// (rather than a generic "unsupported") and lets the parent codec
/// crate's coverage rollup track which features remain on the to-do
/// list.
///
/// Variants are the wiki "Flags meaning" / "IDs" entries the current
/// per-sample loop assumes are absent: the loop has no hybrid-mode
/// `error_limit` binary-search refinement, no float / int32 container
/// fix-up, no multi-block multichannel grouping, no decorrelation
/// pre-pass (the round-3 expanders produce typed views but no
/// prediction loop consumes them yet), and no special handling for the
/// two "experimental" / "low-latency" bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnsupportedBlockFeature {
    /// Wiki bit 3 ("hybrid profile (lossy compression)") is set. The
    /// per-sample loop's spec §4.2 step 6 binary-search refinement for
    /// `error_limit != 0` (the hybrid path) is not yet implemented.
    Hybrid,
    /// Wiki bit 7 ("floating point data present") is set. The float
    /// container fix-up uses the `0x0C` overflow-bits sub-block whose
    /// layout the wiki does not document.
    FloatData,
    /// Wiki bit 8 ("int32 mode") is set. The large/shifted-int
    /// container fix-up uses the `0x09` int32-info sub-block whose
    /// layout the wiki does not document.
    Int32Mode,
    /// Wiki bits 11..=12 ("multi-channel start and end blocks") do not
    /// carry the standalone-block degenerate marker `0b11`, i.e. the
    /// block participates in a multi-block channel grouping. The
    /// per-sample loop is per-block and cannot stitch grouped blocks.
    MultichannelMember,
    /// The block carries at least one of the `0x02` / `0x03` / `0x04`
    /// decorrelation sub-blocks (terms / weights / samples). The
    /// round-3 expanders produce typed views but no prediction-loop
    /// consumer exists yet, so the medians-only per-sample loop would
    /// emit residuals rather than reconstructed samples.
    Decorrelation,
    /// Wiki bit 31 ("low-latency block (experimental, do not decode if
    /// encountered)") is set. The wiki explicitly bars decode of this
    /// block; the composer honours that ban.
    LowLatencyBlock,
    /// Wiki bit 28 ("robust block (experimental, okay to ignore)") is
    /// set. The composer is conservative and refuses experimental
    /// blocks; the per-sample primitives themselves stay callable.
    RobustBlock,
}

impl core::fmt::Display for UnsupportedBlockFeature {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let name = match self {
            UnsupportedBlockFeature::Hybrid => "hybrid lossy profile (flag bit 3)",
            UnsupportedBlockFeature::FloatData => "float-point data (flag bit 7)",
            UnsupportedBlockFeature::Int32Mode => "int32 container mode (flag bit 8)",
            UnsupportedBlockFeature::MultichannelMember => {
                "multi-block multichannel grouping (flag bits 11..=12 != 0b11)"
            }
            UnsupportedBlockFeature::Decorrelation => {
                "decorrelation pre-pass (0x02/0x03/0x04 sub-blocks)"
            }
            UnsupportedBlockFeature::LowLatencyBlock => {
                "low-latency block (flag bit 31, wiki \"do not decode\")"
            }
            UnsupportedBlockFeature::RobustBlock => "robust experimental block (flag bit 28)",
        };
        f.write_str(name)
    }
}

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

    /// Block carries at least one of the three `0x02` / `0x03` / `0x04`
    /// decorrelation sub-blocks. The current per-sample loop has no
    /// prediction-pass consumer for the round-3 typed views, so the
    /// presence of any one of these sub-blocks gates [`Self::decode_samples`]
    /// off via [`UnsupportedBlockFeature::Decorrelation`]. Round 206.
    pub fn has_decorrelation(&self) -> bool {
        self.contains_sub_block(SubBlockId::DecorrelationTerms)
            || self.contains_sub_block(SubBlockId::DecorrelationWeights)
            || self.contains_sub_block(SubBlockId::DecorrelationSamples)
    }

    /// Compose the round-13 [`crate::parse_block`] aggregate, the
    /// round-4 [`crate::expand_entropy`] expander, and the round-15/199
    /// mono / stereo [`crate::decode_packed_samples_mono_from_entropy`] /
    /// [`crate::decode_packed_samples_stereo_from_entropy`] composers
    /// into a one-call "block → PCM" surface.
    ///
    /// The returned `Vec<i32>` carries:
    ///
    /// * mono / false-stereo block (`Flags::is_block_data_mono == true`):
    ///   `header.block_samples` PCM samples, one per `i32` slot;
    /// * stereo block: `header.block_samples * 2` samples interleaved
    ///   left-then-right (the round-199 channel-alternation loop's
    ///   output shape).
    ///
    /// The composer **refuses** blocks that exercise WavPack features
    /// the per-sample loop does not yet support; each refusal carries a
    /// typed [`UnsupportedBlockFeature`] tag through
    /// [`Error::UnsupportedBlockFeature`] so the caller can surface a
    /// precise diagnostic. The refused cases are:
    ///
    /// * [`UnsupportedBlockFeature::Hybrid`] — flag bit 3 ("hybrid
    ///   profile (lossy compression)") set; the per-sample loop's
    ///   `spec/wavpack-entropy-decode.md` §4.2 step 6 binary-search
    ///   refinement for `error_limit != 0` is not yet implemented.
    /// * [`UnsupportedBlockFeature::FloatData`] — flag bit 7 set; the
    ///   `0x0C` overflow-bits sub-block layout is undocumented.
    /// * [`UnsupportedBlockFeature::Int32Mode`] — flag bit 8 set; the
    ///   `0x09` int32-info sub-block layout is undocumented.
    /// * [`UnsupportedBlockFeature::MultichannelMember`] — wiki bits
    ///   11..=12 are not the standalone-block degenerate `0b11`; the
    ///   loop is per-block and cannot stitch grouped blocks.
    /// * [`UnsupportedBlockFeature::Decorrelation`] — any of the `0x02`
    ///   / `0x03` / `0x04` decorrelation sub-blocks are present; the
    ///   round-3 expanders produce typed views but no prediction-loop
    ///   consumer exists yet.
    /// * [`UnsupportedBlockFeature::LowLatencyBlock`] — wiki bit 31
    ///   "low-latency block (experimental, do not decode if encountered)"
    ///   set; the wiki explicitly bars decode.
    /// * [`UnsupportedBlockFeature::RobustBlock`] — wiki bit 28
    ///   "robust block (experimental, okay to ignore)" set; the
    ///   composer is conservative about experimental gating.
    ///
    /// Structural refusals (separate from feature gates):
    ///
    /// * [`Error::BlockHasNoAudio`] — `header.block_samples == 0`;
    ///   metadata-only block. Use [`crate::WavPackBlockHeader::is_audio_block`]
    ///   to filter before calling.
    /// * [`Error::BlockMissingEntropyInfo`] — the block carries no
    ///   `0x05` entropy-info sub-block, so the per-sample loop has no
    ///   medians to seed from.
    /// * [`Error::BlockMissingPackedSamples`] — the block carries no
    ///   `0x0A` packed-samples sub-block.
    ///
    /// Errors from [`crate::expand_entropy`] and the per-sample loop
    /// (truncation, EOF, malformed `EntropyInfo`, …) are propagated
    /// verbatim. The composer is itself stateless: each call seeds a
    /// fresh per-channel [`crate::AdaptiveMedians`] from the block's
    /// `0x05` payload and drops it on return, so back-to-back blocks
    /// of a multi-block file behave like independent decodes (which is
    /// what plain stereo `.wv` carries on the wire anyway — each block
    /// carries a fresh `0x05` seed). Round 206.
    pub fn decode_samples(&self) -> Result<Vec<i32>> {
        let header = &self.header;
        let flags = &header.flags;

        // Structural pre-check: zero-sample / metadata-only blocks have
        // no PCM to return.
        if !header.is_audio_block() {
            return Err(Error::BlockHasNoAudio);
        }

        // Refuse feature combinations the per-sample loop does not yet
        // support. Order roughly matches the wiki "Flags meaning"
        // listing for stable diagnostic output.
        if flags.hybrid {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::Hybrid,
            ));
        }
        if flags.float_data {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::FloatData,
            ));
        }
        if flags.int32_mode {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::Int32Mode,
            ));
        }
        if flags.robust_block {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::RobustBlock,
            ));
        }
        if flags.low_latency_block {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::LowLatencyBlock,
            ));
        }
        if flags.is_multichannel_member() {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::MultichannelMember,
            ));
        }
        if self.has_decorrelation() {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::Decorrelation,
            ));
        }

        // Structural sub-block lookup. Both must be present for the
        // per-sample loop to have inputs.
        let entropy_sub =
            find_entropy_info(&self.sub_blocks).ok_or(Error::BlockMissingEntropyInfo)?;
        let packed =
            find_packed_samples(&self.sub_blocks).ok_or(Error::BlockMissingPackedSamples)?;

        // Round-4 expander on the 0x05 payload + round-201 wrappers on
        // the 0x0A payload give us the whole pipe in two calls.
        let entropy = expand_entropy(entropy_sub.payload)?;
        let count = header.block_samples as usize;

        if flags.is_block_data_mono() {
            decode_packed_samples_mono_from_entropy(&packed, &entropy, count)
        } else {
            decode_packed_samples_stereo_from_entropy(&packed, &entropy, count)
        }
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

    // ---- Round-206 WavPackBlock::decode_samples composer ----

    /// Synthesise a header with caller-chosen `block_samples` and
    /// `flags`. `ck_size` is sized to the supplied payload length plus
    /// the fixed 24-byte header tail. The other fields (track, totals,
    /// CRC) stay zero — they are not consulted by `decode_samples`.
    ///
    /// Used to drive `WavPackBlock::decode_samples` through the various
    /// gate / reject / accept paths.
    fn synthesise_block(block_samples: u32, flags: u32, payload: &[u8]) -> Vec<u8> {
        let ck_size = (24 + payload.len()) as u32;
        let mut buf = vec![0u8; HEADER_LEN];
        buf[0..4].copy_from_slice(MAGIC);
        buf[4..8].copy_from_slice(&ck_size.to_le_bytes());
        buf[8..10].copy_from_slice(&0x0410u16.to_le_bytes());
        buf[20..24].copy_from_slice(&block_samples.to_le_bytes());
        buf[24..28].copy_from_slice(&flags.to_le_bytes());
        // Standalone-block multichannel marker (bits 11..=12 = 0b11)
        // unless the caller supplied something else.
        buf.extend_from_slice(payload);
        buf
    }

    /// Build the standard standalone-block flag word: bits 11..=12 set
    /// (standalone-marker `0b11`), plus the supplied extra-bits.
    fn flags_with(extra: u32) -> u32 {
        (0b11u32 << 11) | extra
    }

    /// Append the 0x05 entropy-info sub-block carrying the all-zero
    /// mono seed (six bytes of zeros → `EntropyInfo::mono([0,0,0])`).
    fn append_entropy_info_mono_zero(payload: &mut Vec<u8>) {
        append_small_sub_block(payload, 0x05, &[0u8; 6]);
    }

    /// Append a 0x05 entropy-info sub-block carrying a minimal-stereo
    /// seed (left = right = `[1, 0, 0]`). The non-zero left-median value
    /// is what `EntropyInfo::is_mono()` (a content-only predicate)
    /// inspects to report `false`, so the stereo decode path is taken
    /// even though `get_med(0) = (1 >> 4) + 1 = 1` still leaves both
    /// channels eligible for the spec §4.2 step 1 zero-run fast path.
    ///
    /// On-disk wire format per the wiki "Entropy info" + `0x04`
    /// log-packed expansion: each 16-bit median word is
    /// `[mantissa_lo, exponent_hi]` with mantissa signed and exponent
    /// biased by `-9`. The bytes `[0x01, 0x09]` decode as
    /// `mantissa = 1, exponent = 9, shift = 0`, giving `median = 1`.
    fn append_entropy_info_stereo_minimal(payload: &mut Vec<u8>) {
        let mut bytes = [0u8; 12];
        bytes[0] = 0x01;
        bytes[1] = 0x09; // left medians[0] = 1
        bytes[6] = 0x01;
        bytes[7] = 0x09; // right medians[0] = 1
        append_small_sub_block(payload, 0x05, &bytes);
    }

    /// Append a 0x0A packed-samples sub-block with the supplied payload
    /// bytes. Must be even-length (sub-block size is in 16-bit words and
    /// we don't set the odd-size flag here).
    fn append_packed_samples(payload: &mut Vec<u8>, bytes: &[u8]) {
        append_small_sub_block(payload, 0x0A, bytes);
    }

    #[test]
    fn decode_samples_returns_one_zero_for_mono_block_with_zero_seed_and_zero_unary() {
        // Seeds [0,0,0] make get_med(0) = 1 → zero-run fast path eligible.
        // 0x0A payload of one byte 0x00: get_unary() reads a single
        // 0 bit (run_length = 0), so the call emits one `0` sample and
        // does not zero medians (no reset for run_length == 0). The
        // composer wraps that into a Vec<i32> = [0].
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with(1 << 2); // bit 2 = mono
        let bytes = synthesise_block(1, flags, &payload);
        let (block, tail) = parse_block(&bytes).expect("parse block");
        assert!(tail.is_empty());
        assert!(block.header.flags.mono);
        assert!(block.header.flags.is_block_data_mono());
        let got = block.decode_samples().expect("decode samples");
        assert_eq!(got, vec![0]);
    }

    #[test]
    fn decode_samples_returns_two_interleaved_zeros_for_stereo_block_with_minimal_seed() {
        // Stereo block, block_samples = 1 stereo frame = 2 interleaved
        // PCM samples. With both channels at minimal-non-zero seeds
        // `[1, 0, 0]` (so `EntropyInfo::is_mono()` returns false and
        // the stereo bridge accepts the seed) both still have
        // `get_med(0) == 1`, so the spec §4.2 step 1 zero-run path is
        // eligible. A 0x0A payload of one `0x00` byte reads as
        // `get_unary() == 0` (run length 0) → emit one stereo frame of
        // (0, 0). Standalone-block marker (bits 11..=12 = 0b11) keeps
        // the block out of the multichannel-member refusal.
        let mut payload = Vec::new();
        append_entropy_info_stereo_minimal(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(0), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.header.flags.mono);
        assert!(!block.header.flags.is_block_data_mono());
        let got = block.decode_samples().expect("decode stereo samples");
        // 1 stereo frame → 2 interleaved samples (L, R).
        assert_eq!(got.len(), 2);
        assert_eq!(got, vec![0, 0]);
    }

    #[test]
    fn decode_samples_rejects_metadata_only_block_with_block_has_no_audio() {
        // block_samples == 0 → metadata-only block per the wiki
        // "Block structure" allowance. decode_samples must refuse this
        // before walking the metadata.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples().expect_err("must refuse no-audio");
        assert_eq!(err, Error::BlockHasNoAudio);
    }

    #[test]
    fn decode_samples_rejects_block_missing_entropy_info() {
        // 0x0A present, 0x05 absent — the composer cannot seed the
        // running medians.
        let mut payload = Vec::new();
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block
            .decode_samples()
            .expect_err("must refuse missing 0x05");
        assert_eq!(err, Error::BlockMissingEntropyInfo);
    }

    #[test]
    fn decode_samples_rejects_block_missing_packed_samples() {
        // 0x05 present, 0x0A absent — the composer has nothing to
        // decode against.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block
            .decode_samples()
            .expect_err("must refuse missing 0x0A");
        assert_eq!(err, Error::BlockMissingPackedSamples);
    }

    #[test]
    fn decode_samples_rejects_hybrid_lossy_profile() {
        // Bit 3 set → hybrid lossy profile. The per-sample loop has no
        // error_limit binary-search refinement, so the composer refuses
        // these blocks with a typed feature tag.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 3)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples().expect_err("must refuse hybrid");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Hybrid)
        );
    }

    #[test]
    fn decode_samples_rejects_float_data_profile() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 7)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples().expect_err("must refuse float");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::FloatData)
        );
    }

    #[test]
    fn decode_samples_rejects_int32_mode() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 8)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples().expect_err("must refuse int32");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Int32Mode)
        );
    }

    #[test]
    fn decode_samples_rejects_robust_experimental_block() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 28)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples().expect_err("must refuse robust");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::RobustBlock)
        );
    }

    #[test]
    fn decode_samples_rejects_low_latency_block() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 31)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples().expect_err("must refuse low-latency");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::LowLatencyBlock)
        );
    }

    #[test]
    fn decode_samples_rejects_multichannel_member_block() {
        // multichannel_marker != 0b11 → block participates in a
        // multi-block channel grouping the per-sample loop cannot
        // stitch. We use the 0b01 ("first block") marker for this test;
        // the symmetric `0b10` last-block and `0b00` middle-block cases
        // also reject.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        // Build flag word manually: bit 2 mono, bits 11..=12 = 0b01.
        let flag_word = (1u32 << 2) | (1u32 << 11);
        let bytes = synthesise_block(1, flag_word, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.header.flags.is_multichannel_member());
        let err = block
            .decode_samples()
            .expect_err("must refuse multichannel member");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::MultichannelMember)
        );
    }

    #[test]
    fn decode_samples_rejects_block_with_decorrelation_terms_sub_block() {
        // A `0x02` (decorrelation terms) sub-block presence is the
        // tell-tale that the encoder ran a prediction pass; without a
        // matching consumer in the decoder, the composer must refuse.
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x02, &[0u8; 2]); // 1 term
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_decorrelation());
        let err = block
            .decode_samples()
            .expect_err("must refuse decorrelation");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Decorrelation)
        );
    }

    #[test]
    fn decode_samples_rejects_block_with_decorrelation_weights_sub_block() {
        // The `has_decorrelation` predicate fires on any one of the
        // 0x02/0x03/0x04 sub-blocks. Test the 0x03 path independently
        // so a future change to the predicate doesn't silently drop
        // detection of either of the other two.
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x03, &[0u8; 2]);
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block
            .decode_samples()
            .expect_err("must refuse 0x03 decorrelation");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Decorrelation)
        );
    }

    #[test]
    fn decode_samples_rejects_block_with_decorrelation_samples_sub_block() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x04, &[0u8; 2]);
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block
            .decode_samples()
            .expect_err("must refuse 0x04 decorrelation");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Decorrelation)
        );
    }

    #[test]
    fn has_decorrelation_returns_true_for_each_of_the_three_sub_blocks() {
        // Per-sub-block sanity sweep so the OR-of-three predicate stays
        // honest as the metadata enum grows.
        for &id in &[0x02u8, 0x03, 0x04] {
            let mut payload = Vec::new();
            append_small_sub_block(&mut payload, id, &[0u8; 2]);
            let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
            let (block, _) = parse_block(&bytes).expect("parse block");
            assert!(
                block.has_decorrelation(),
                "has_decorrelation should fire on 0x{id:02x}",
            );
        }
    }

    #[test]
    fn has_decorrelation_returns_false_for_blocks_without_pre_pass_sub_blocks() {
        // A block carrying only 0x05 + 0x0A reports no decorrelation
        // pass — the round-3 typed-view consumer can stand down.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_decorrelation());
    }

    #[test]
    fn decode_samples_propagates_entropy_info_length_error_for_malformed_0x05() {
        // 0x05 payload of 8 bytes — neither mono (6) nor stereo (12).
        // expand_entropy reports EntropyInfoLength; decode_samples
        // propagates verbatim.
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x05, &[0u8; 8]);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block
            .decode_samples()
            .expect_err("must propagate malformed 0x05");
        assert_eq!(err, Error::EntropyInfoLength(8));
    }

    #[test]
    fn decode_samples_for_false_stereo_block_uses_mono_loop() {
        // Bit 30 false_stereo set with bit 2 mono cleared — wiki:
        // "stream is stereo but this block's data is mono". The
        // composer must dispatch to the mono path even though the
        // top-level flags.mono bit is 0. We confirm by feeding a
        // mono-shaped 6-byte 0x05 (stereo decode would error on length)
        // and observing successful decode of one sample.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with(1 << 30); // false_stereo, no mono bit
        let bytes = synthesise_block(1, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.header.flags.mono);
        assert!(block.header.flags.false_stereo);
        assert!(block.header.flags.is_block_data_mono());
        let got = block.decode_samples().expect("false-stereo decode");
        assert_eq!(got, vec![0]);
    }

    #[test]
    fn unsupported_block_feature_display_lists_each_variant() {
        // Display strings are part of the public error surface — pin
        // the wording so doc / log consumers don't see it drift.
        let cases: &[(UnsupportedBlockFeature, &str)] = &[
            (UnsupportedBlockFeature::Hybrid, "flag bit 3"),
            (UnsupportedBlockFeature::FloatData, "flag bit 7"),
            (UnsupportedBlockFeature::Int32Mode, "flag bit 8"),
            (
                UnsupportedBlockFeature::MultichannelMember,
                "flag bits 11..=12 != 0b11",
            ),
            (
                UnsupportedBlockFeature::Decorrelation,
                "0x02/0x03/0x04 sub-blocks",
            ),
            (UnsupportedBlockFeature::LowLatencyBlock, "flag bit 31"),
            (UnsupportedBlockFeature::RobustBlock, "flag bit 28"),
        ];
        for (feat, substring) in cases {
            let rendered = format!("{feat}");
            assert!(
                rendered.contains(substring),
                "Display for {feat:?} should mention {substring} but rendered {rendered:?}",
            );
        }
    }
}
