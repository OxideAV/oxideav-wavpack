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

use crate::block_header::{parse_block_header, Flags, WavPackBlockHeader, HEADER_LEN};
use crate::entropy::{expand_entropy, EntropyInfo};
use crate::error::{Error, Result};
use crate::metadata::{
    find_entropy_info, find_first, find_md5_checksum_block, find_multichannel_info,
    find_packed_samples, parse_md5_checksum, walk_metadata, Md5Checksum, MetadataSubBlock,
    SubBlockId,
};
use crate::packed_samples::PackedSamples;
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

    /// Convenience accessor: the parsed [`Flags`] view from the fixed
    /// block header. Equivalent to `&self.header().flags` but spelled
    /// directly for callers picking flag predicates off a borrowed
    /// `WavPackBlock` without re-binding the header. Round 214.
    pub fn flags(&self) -> &Flags {
        &self.header.flags
    }

    /// Convenience accessor: the wiki "samples in this block" field
    /// (`block_samples` in the round-1 header). Equivalent to
    /// `self.header().block_samples` but spelled directly so callers
    /// asking "how many PCM samples does this block carry?" don't need
    /// to reach through the field. Round 214.
    pub fn block_samples(&self) -> u32 {
        self.header.block_samples
    }

    /// Convenience accessor: the wiki "offset in samples for current
    /// block" field (`block_index` in the round-1 header). Round 214.
    pub fn block_index(&self) -> u32 {
        self.header.block_index
    }

    /// `true` when the round-1 [`WavPackBlockHeader::is_audio_block`]
    /// predicate fires (`block_samples != 0`). The wiki "Block structure"
    /// listing allows `block_samples == 0` for metadata-only blocks
    /// (e.g. a RIFF-header-only block at the start of a file); this
    /// accessor lifts the header's `is_audio_block` to the block level
    /// so a caller iterating a multi-block stream can filter without
    /// reaching through the header. Round 214.
    pub fn is_audio_block(&self) -> bool {
        self.header.is_audio_block()
    }

    /// `true` when a `0x05` entropy-info sub-block is present. Pairs
    /// with [`Self::entropy_info`] which decodes it; this predicate lets
    /// a caller check for presence without paying the expansion cost.
    /// Round 214.
    pub fn has_entropy_info(&self) -> bool {
        self.contains_sub_block(SubBlockId::EntropyInfo)
    }

    /// `true` when a `0x0A` packed-samples sub-block is present. Pairs
    /// with [`Self::packed_samples`] which wraps it as a typed view.
    /// Round 214.
    pub fn has_packed_samples(&self) -> bool {
        self.contains_sub_block(SubBlockId::PackedSamples)
    }

    /// `true` when a `0x26` MD5-checksum sub-block is present. Pairs
    /// with [`Self::md5_checksum`] which parses the 16-byte digest.
    /// Round 214.
    pub fn has_md5_checksum(&self) -> bool {
        self.contains_sub_block(SubBlockId::Md5Checksum)
    }

    /// `true` when a `0x20` RIFF-header sub-block is present. The wiki
    /// "IDs" listing puts this sub-block "before audio" — the original
    /// `.wav` file's RIFF header preserved verbatim so a lossless decode
    /// can re-emit a byte-identical `.wav`. Round 214.
    pub fn has_riff_header(&self) -> bool {
        self.contains_sub_block(SubBlockId::RiffHeader)
    }

    /// `true` when a `0x21` RIFF-trailer sub-block is present. The wiki
    /// "IDs" listing puts this sub-block "after audio" — any RIFF
    /// chunks the original `.wav` carried after the audio data
    /// (e.g. `LIST INFO`) preserved verbatim. Round 214.
    pub fn has_riff_trailer(&self) -> bool {
        self.contains_sub_block(SubBlockId::RiffTrailer)
    }

    /// `true` when a `0x0D` multichannel-info sub-block is present. The
    /// wiki "IDs" listing names this payload "multichannel information
    /// (including Microsoft channel mask)"; the payload layout is not
    /// documented by the wiki, so this is a presence-only predicate.
    /// Round 214.
    pub fn has_multichannel_info(&self) -> bool {
        self.contains_sub_block(SubBlockId::MultichannelInfo)
    }

    /// Locate and return a borrowed reference to the first metadata
    /// sub-block with the given ID, or `None` when no such sub-block
    /// exists in this block. Block-level convenience over the free
    /// [`crate::find_first`] function on `self.sub_blocks()`. Round 214.
    pub fn find_sub_block(&self, id: SubBlockId) -> Option<&MetadataSubBlock<'a>> {
        find_first(&self.sub_blocks, id)
    }

    /// Borrow the first `0x05` entropy-info sub-block, or `None` when
    /// none is present. Block-level pairing with the free
    /// [`crate::find_entropy_info`] finder on `self.sub_blocks()`.
    /// Use [`Self::entropy_info`] to additionally decode the payload
    /// into a typed [`EntropyInfo`]. Round 214.
    pub fn find_entropy_info_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        find_entropy_info(&self.sub_blocks)
    }

    /// Borrow the first `0x26` MD5-checksum sub-block, or `None` when
    /// none is present. Block-level pairing with the free
    /// [`crate::find_md5_checksum_block`] finder on `self.sub_blocks()`.
    /// Use [`Self::md5_checksum`] to additionally parse the 16-byte
    /// digest into a typed [`Md5Checksum`]. Round 214.
    pub fn find_md5_checksum_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        find_md5_checksum_block(&self.sub_blocks)
    }

    /// Borrow the first `0x0D` multichannel-info sub-block, or `None`
    /// when none is present. Block-level pairing with the free
    /// [`crate::find_multichannel_info`] finder on `self.sub_blocks()`.
    /// The wiki does not specify the multichannel-info payload layout,
    /// so this stays at "borrow the bytes" rather than a typed view.
    /// Round 214.
    pub fn find_multichannel_info_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        find_multichannel_info(&self.sub_blocks)
    }

    /// Borrow the first `0x20` RIFF-header sub-block, or `None` when
    /// none is present. The wiki "IDs" listing places this before any
    /// audio; the payload is the verbatim RIFF/WAVE header from the
    /// source `.wav` file. Round 214.
    pub fn find_riff_header_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        find_first(&self.sub_blocks, SubBlockId::RiffHeader)
    }

    /// Borrow the first `0x21` RIFF-trailer sub-block, or `None` when
    /// none is present. The wiki "IDs" listing places this after the
    /// audio; the payload carries any RIFF chunks following the source
    /// `.wav` file's `data` chunk. Round 214.
    pub fn find_riff_trailer_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        find_first(&self.sub_blocks, SubBlockId::RiffTrailer)
    }

    /// Locate the `0x0A` packed-samples sub-block and wrap it as a
    /// typed [`PackedSamples`] view, or return `None` when no `0x0A`
    /// sub-block is present. Block-level pairing with the free
    /// [`crate::find_packed_samples`] finder. Round 214.
    pub fn packed_samples(&self) -> Option<PackedSamples<'a>> {
        find_packed_samples(&self.sub_blocks)
    }

    /// `true` when a `0x0B` packed-correction-data sub-block is present.
    /// The wiki "IDs" listing annotates this payload as carried in the
    /// `.wvc` companion file alongside the lossy main `.wv`; presence
    /// in a block indicates the block is part of a hybrid encode whose
    /// correction stream has been merged back into the main file (a
    /// valid wire shape — the wiki places no rule against carrying
    /// `0x0B` alongside `0x0A` in the same block). Round 233.
    pub fn has_packed_correction_data(&self) -> bool {
        self.contains_sub_block(SubBlockId::PackedCorrectionData)
    }

    /// `true` when a `0x07` noise-shaping-profile sub-block is present.
    /// The wiki "IDs" listing annotates this payload as carried in the
    /// `.wvc` companion file; pairs with [`Self::has_packed_correction_data`]
    /// to identify blocks fully equipped with a correction stream.
    /// Round 233.
    pub fn has_noise_shaping_profile(&self) -> bool {
        self.contains_sub_block(SubBlockId::NoiseShapingProfile)
    }

    /// `true` when a `0x06` hybrid-profile sub-block is present. The
    /// wiki "IDs" listing names this payload alongside the `0x07`
    /// noise-shaping profile as the per-block hybrid configuration;
    /// presence indicates the block was encoded with the hybrid
    /// profile (independent of whether the `0x0B` correction data is
    /// also carried — that gates whether the decode can be sample-exact).
    /// Round 233.
    pub fn has_hybrid_profile(&self) -> bool {
        self.contains_sub_block(SubBlockId::HybridProfile)
    }

    /// `true` when the block carries **either** of the `.wvc`-side
    /// payloads — `0x07` noise-shaping profile or `0x0B` packed
    /// correction data. Composite predicate matching the existing
    /// [`crate::MetadataSubBlock::is_correction_payload`] grouping; useful
    /// for stream-level introspection that wants to count or filter
    /// blocks the hybrid decoder would consume. Round 233.
    pub fn has_correction_stream_data(&self) -> bool {
        self.has_packed_correction_data() || self.has_noise_shaping_profile()
    }

    /// Borrow the first `0x0B` packed-correction-data sub-block, or
    /// `None` when none is present. Block-level pairing with the free
    /// [`crate::find_packed_correction_data_sub_block`] finder. Use
    /// [`Self::packed_correction_data`] for the typed-view variant.
    /// Round 233.
    pub fn find_packed_correction_data_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        crate::metadata::find_packed_correction_data_sub_block(&self.sub_blocks)
    }

    /// Locate the `0x0B` packed-correction-data sub-block and wrap it
    /// as a typed [`crate::PackedCorrectionData`] view, or return
    /// `None` when no `0x0B` sub-block is present. Block-level pairing
    /// with the free [`crate::find_packed_correction_data`] finder.
    /// Round 233.
    pub fn packed_correction_data(&self) -> Option<crate::PackedCorrectionData<'a>> {
        crate::metadata::find_packed_correction_data(&self.sub_blocks)
    }

    /// Borrow the first `0x07` noise-shaping-profile sub-block, or
    /// `None` when none is present. Block-level pairing with the free
    /// [`crate::find_noise_shaping_profile`] finder. The wiki places
    /// no internal structure on the payload, so this stops at
    /// "borrow the bytes". Round 233.
    pub fn find_noise_shaping_profile_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        crate::metadata::find_noise_shaping_profile(&self.sub_blocks)
    }

    /// Borrow the first `0x06` hybrid-profile sub-block, or `None`
    /// when none is present. Block-level pairing with the free
    /// [`crate::find_hybrid_profile`] finder. The wiki places no
    /// internal structure on the payload, so this stops at "borrow
    /// the bytes". Round 233.
    pub fn find_hybrid_profile_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        crate::metadata::find_hybrid_profile(&self.sub_blocks)
    }

    /// Locate the `0x05` entropy-info sub-block and expand its payload
    /// into a typed [`EntropyInfo`].
    ///
    /// Returns `Ok(None)` when the block carries no `0x05` sub-block
    /// (a structurally legal case — metadata-only blocks have no
    /// medians to seed). Returns `Err` when the sub-block is present
    /// but its payload is malformed (the round-4 [`expand_entropy`]
    /// rejection — neither 6 nor 12 bytes; see
    /// [`Error::EntropyInfoLength`]). Round 214.
    pub fn entropy_info(&self) -> Result<Option<EntropyInfo>> {
        match find_entropy_info(&self.sub_blocks) {
            Some(sub) => Ok(Some(expand_entropy(sub.payload)?)),
            None => Ok(None),
        }
    }

    /// Locate the `0x26` MD5-checksum sub-block and parse its 16-byte
    /// payload into a typed [`Md5Checksum`].
    ///
    /// Returns `Ok(None)` when no `0x26` sub-block is present (the wiki
    /// "IDs" listing makes the MD5 optional — many older `.wv` files
    /// omit it). Returns `Err(Error::Md5ChecksumLength)` when the
    /// sub-block is present but the payload is the wrong length.
    /// Round 214.
    pub fn md5_checksum(&self) -> Result<Option<Md5Checksum>> {
        match find_md5_checksum_block(&self.sub_blocks) {
            Some(sub) => Ok(Some(parse_md5_checksum(sub.payload)?)),
            None => Ok(None),
        }
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

    /// Number of `i32` PCM slots [`Self::decode_samples`] would emit on
    /// success, computed from the parsed header alone — no entropy
    /// expansion, no per-sample-loop call.
    ///
    /// Returns `block_samples()` on a mono / false-stereo block (one
    /// `i32` per sample) and `block_samples() * 2` on a stereo block
    /// (two interleaved `i32`s per sample frame). Metadata-only blocks
    /// (`block_samples == 0`) return `0` — they carry no PCM at all.
    ///
    /// The wiki bit 2 + bit 30 union (the [`Flags::is_block_data_mono`]
    /// accessor) drives the per-block shape choice, mirroring the
    /// dispatch the round-206 [`Self::decode_samples`] composer applies.
    /// Callers sizing a buffer before calling [`Self::decode_samples`]
    /// can use this to size in one constant-time step rather than
    /// matching on the flags themselves; callers walking a multi-block
    /// stream with [`StreamDecodeIter`] can sum this across the blocks
    /// to pre-size a contiguous PCM `Vec`.
    ///
    /// The return type is `u64` so the pathological case
    /// `u32::MAX * 2` (one stereo block claiming the full 32-bit sample
    /// count — legal on the wire even if the wiki never produces it)
    /// does not overflow `u32` on the multiplication.
    ///
    /// Round 230.
    pub fn decoded_sample_count(&self) -> u64 {
        let samples = self.header.block_samples as u64;
        if self.header.flags.is_block_data_mono() {
            samples
        } else {
            samples * 2
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

/// Lazy iterator over the consecutive WavPack blocks in a byte buffer.
///
/// The wiki "File Format" section pins the file shape: a `.wv` file is a
/// concatenation of WavPack blocks, each beginning with the `wvpk` magic
/// and each declaring its own on-disk byte count via the `ck_size` field
/// (the wiki "Block structure" listing). [`BlockIter`] walks that chain
/// one block at a time, calling [`parse_block`] under the hood and using
/// its returned tail as the next call's input.
///
/// The iterator yields `Result<WavPackBlock<'_>>`. The first error
/// terminates iteration (subsequent `next()` calls return `None`) so the
/// caller can `?`-bubble the first failure without losing the
/// already-yielded blocks. An empty input is treated as zero blocks
/// (the wiki "WavPack file consists of blocks" sentence is plural but
/// nothing in the wiki forbids the empty file as a degenerate case).
///
/// Construction: [`iter_blocks`] for byte-slice input, or
/// [`BlockIter::new`] for callers that already hold a `&[u8]`. Both
/// return the same iterator type; choose by call site readability.
#[derive(Debug, Clone)]
pub struct BlockIter<'a> {
    /// Remaining bytes to parse. Shrinks by one block's on-disk length on
    /// every successful `next()` call (i.e. by `8 + ck_size` per the wiki
    /// "Block structure" definition of `ck_size`).
    remaining: &'a [u8],
    /// Set to `true` once `next()` returns `Err(_)` so subsequent calls
    /// short-circuit to `None` without re-attempting the failing parse.
    /// Matches the standard `FusedIterator` contract.
    done: bool,
}

impl<'a> BlockIter<'a> {
    /// Build a [`BlockIter`] over the supplied byte buffer. Equivalent to
    /// [`iter_blocks`] but spelled as a constructor for callers that
    /// prefer the type-first form.
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            remaining: bytes,
            done: false,
        }
    }

    /// Bytes of the input buffer the iterator has not yet consumed.
    ///
    /// Equals the original input on a freshly-constructed iterator; on a
    /// fully-iterated buffer it is the empty slice if every block parsed
    /// cleanly, or the tail starting at the first malformed block's first
    /// byte if iteration ended on an error.
    pub fn remaining(&self) -> &'a [u8] {
        self.remaining
    }

    /// `true` when this iterator will not yield any more items — either
    /// because the buffer is empty or because a previous `next()` call
    /// returned `Err(_)` (the iterator is fused on error).
    pub fn is_exhausted(&self) -> bool {
        self.done || self.remaining.is_empty()
    }

    /// Advance to and return the next **audio** block in the input,
    /// silently skipping metadata-only blocks (`block_samples == 0`,
    /// the wiki "Block structure" allowance for header-only / RIFF-only
    /// blocks).
    ///
    /// Returns `Some(Ok(block))` for the next audio block, `Some(Err(e))`
    /// on the first parse error (the iterator fuses on the underlying
    /// [`BlockIter`] error contract, so a follow-up call returns `None`),
    /// or `None` when the buffer is drained or only metadata-only blocks
    /// remain. Useful for callers that pre-flight whether decoding has
    /// any work to do before invoking the round-206
    /// [`WavPackBlock::decode_samples`] composer.
    ///
    /// Pulled directly from the iterator surface; on a metadata-only
    /// block the iterator advances past it (the metadata-only block is
    /// consumed), so a subsequent [`Self::next`] call sees only blocks
    /// further along the buffer. Round 230.
    pub fn next_audio(&mut self) -> Option<Result<WavPackBlock<'a>>> {
        loop {
            match self.next()? {
                Ok(block) => {
                    if block.is_audio_block() {
                        return Some(Ok(block));
                    }
                    // metadata-only block — skip and continue.
                }
                Err(e) => return Some(Err(e)),
            }
        }
    }
}

impl<'a> Iterator for BlockIter<'a> {
    type Item = Result<WavPackBlock<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done || self.remaining.is_empty() {
            return None;
        }
        match parse_block(self.remaining) {
            Ok((block, tail)) => {
                self.remaining = tail;
                Some(Ok(block))
            }
            Err(e) => {
                // Fuse on first error so the caller doesn't see the same
                // failure repeatedly if they `next()` again.
                self.done = true;
                Some(Err(e))
            }
        }
    }
}

impl core::iter::FusedIterator for BlockIter<'_> {}

/// Construct a lazy [`BlockIter`] over the supplied byte buffer.
///
/// Equivalent to [`BlockIter::new`]; provided as a free function for the
/// `iter_blocks(bytes)` call shape readers expect when scanning a `.wv`
/// file's worth of blocks.
///
/// Yields one `Result<WavPackBlock<'_>>` per block. See [`BlockIter`] for
/// the iteration contract (empty input → zero blocks; first error fuses
/// the iterator).
pub fn iter_blocks(bytes: &[u8]) -> BlockIter<'_> {
    BlockIter::new(bytes)
}

/// Eagerly parse every WavPack block in the supplied byte buffer.
///
/// Convenience wrapper around [`iter_blocks`] for callers who want the
/// full block list up front and the first parse error bubbled directly
/// via `?`. Returns `Ok(vec)` only when **every** block parses cleanly
/// and the buffer ends on a block boundary.
///
/// On the first malformed block the returned error is whichever variant
/// [`parse_block`] produced — see [`parse_block`] for the full
/// enumeration. Blocks parsed before the failure are discarded; callers
/// who want to inspect them should drive [`iter_blocks`] manually.
pub fn parse_blocks(bytes: &[u8]) -> Result<Vec<WavPackBlock<'_>>> {
    iter_blocks(bytes).collect()
}

/// Count the WavPack blocks in `bytes` without retaining the parsed
/// blocks.
///
/// Driven by [`iter_blocks`] for parse correctness (i.e. every block in
/// the buffer must parse cleanly for the count to be returned); a single
/// malformed block surfaces its [`parse_block`] error verbatim. The
/// implementation pulls one block at a time and drops it before
/// continuing, so the working-set memory stays at one block independent
/// of input length.
pub fn block_count(bytes: &[u8]) -> Result<usize> {
    let mut iter = iter_blocks(bytes);
    let mut count = 0;
    for block in iter.by_ref() {
        block?;
        count += 1;
    }
    Ok(count)
}

/// Sum the `block_samples` field across every block in `blocks`.
///
/// The wiki "Block structure" listing defines `block_samples` as
/// "samples in this block (may be 0 if no audio present)", so summing
/// the field across a multi-block file yields the total number of
/// sample frames carried by the file's audio blocks (metadata-only
/// blocks contribute zero). The return type is `u64` so a 4-GiB-plus
/// stream's sample count does not overflow `u32` on the way out.
///
/// Pure accessor over an already-parsed block list; performs no I/O and
/// returns no error.
pub fn total_block_samples(blocks: &[WavPackBlock<'_>]) -> u64 {
    blocks.iter().map(|b| b.block_samples() as u64).sum()
}

/// Count the blocks in `bytes` whose `block_samples > 0` — i.e. those
/// blocks carrying PCM rather than metadata only.
///
/// The wiki "Block structure" listing allows `block_samples == 0` for
/// blocks carrying only RIFF wrappers / MD5 sums / other metadata; this
/// counter splits the stream's blocks into "audio" and "metadata-only"
/// by the same criterion [`WavPackBlock::is_audio_block`] uses.
///
/// Drives [`iter_blocks`] under the hood; every block in the buffer
/// must parse cleanly for the count to be returned (any parse error
/// surfaces verbatim). Working-set memory is one block at a time
/// regardless of input length. Round 230.
pub fn audio_block_count(bytes: &[u8]) -> Result<usize> {
    let mut iter = iter_blocks(bytes);
    let mut count = 0;
    for block in iter.by_ref() {
        let block = block?;
        if block.is_audio_block() {
            count += 1;
        }
    }
    Ok(count)
}

/// Count the blocks in `bytes` whose `block_samples == 0` — i.e. the
/// metadata-only blocks the wiki "Block structure" listing allows.
///
/// Inverse of [`audio_block_count`]; together they sum to
/// [`block_count`]. The wiki examples of metadata-only blocks are
/// leading RIFF-header blocks (block + sub-block `0x20`) and trailing
/// RIFF-trailer / MD5 blocks (sub-block `0x21` / `0x26`); this counter
/// reports the count without inspecting the sub-block list.
///
/// Drives [`iter_blocks`] under the hood; any parse error surfaces
/// verbatim. Round 230.
pub fn metadata_block_count(bytes: &[u8]) -> Result<usize> {
    let mut iter = iter_blocks(bytes);
    let mut count = 0;
    for block in iter.by_ref() {
        let block = block?;
        if !block.is_audio_block() {
            count += 1;
        }
    }
    Ok(count)
}

/// Sum the `block_samples` field across the audio blocks in `bytes`.
///
/// Equivalent to filtering [`iter_blocks`] to audio blocks and summing
/// `block_samples()` across them. Because metadata-only blocks
/// contribute `0` by definition, this also equals
/// `total_block_samples(&parse_blocks(bytes)?)` — but without
/// retaining the parsed block list. Returns `u64` so a 4-GiB-plus
/// stream's sample count does not overflow `u32`. Round 230.
pub fn total_audio_samples(bytes: &[u8]) -> Result<u64> {
    let mut iter = iter_blocks(bytes);
    let mut sum: u64 = 0;
    for block in iter.by_ref() {
        let block = block?;
        if block.is_audio_block() {
            sum += block.block_samples() as u64;
        }
    }
    Ok(sum)
}

/// Sum the `i32` PCM slot count [`decode_stream`] would emit across
/// every audio block in `bytes`.
///
/// Mono / false-stereo blocks contribute `block_samples()` slots;
/// stereo blocks contribute `block_samples() * 2` slots (the
/// left-then-right interleave [`decode_stream`] produces). The sum is
/// the exact `len()` [`decode_stream`] would return on success —
/// callers sizing a destination buffer can use this to pre-allocate
/// without paying the per-sample-loop cost.
///
/// Returns `u64` so the `u32::MAX * 2` worst case (one stereo block
/// claiming the full 32-bit sample count) does not overflow. Drives
/// [`iter_blocks`] under the hood; any parse error surfaces verbatim.
/// Note this does **not** validate the per-block feature gates the
/// round-206 [`WavPackBlock::decode_samples`] composer applies, so a
/// stream whose every block decodes successfully and a stream whose
/// blocks would be refused by the composer return the same count
/// (the count is structural; the composer is semantic). Round 230.
pub fn decoded_sample_count(bytes: &[u8]) -> Result<u64> {
    let mut iter = iter_blocks(bytes);
    let mut sum: u64 = 0;
    for block in iter.by_ref() {
        let block = block?;
        sum += block.decoded_sample_count();
    }
    Ok(sum)
}

/// Peek the first audio block in `bytes` — the first block whose
/// `block_samples > 0` — without retaining the rest of the stream.
///
/// The wiki "Block structure" allowance for `block_samples == 0`
/// metadata-only blocks (RIFF headers, trailing MD5 sums, …) means a
/// `.wv` file's first block on disk is not necessarily the first
/// audio block; this accessor walks past the leading metadata-only
/// blocks to surface the first one carrying PCM.
///
/// Returns `Ok(None)` when the stream has no audio blocks (empty
/// input, all-metadata-only input). Returns `Err(_)` when a block
/// before the first audio block fails to parse — the round-13
/// [`parse_block`] errors surface verbatim. Round 230.
pub fn first_audio_block(bytes: &[u8]) -> Result<Option<WavPackBlock<'_>>> {
    let mut iter = iter_blocks(bytes);
    match iter.next_audio() {
        Some(Ok(block)) => Ok(Some(block)),
        Some(Err(e)) => Err(e),
        None => Ok(None),
    }
}

/// Lazy iterator over the audio blocks (`block_samples > 0`) in a
/// WavPack byte buffer.
///
/// Wraps [`BlockIter`] so the iteration skips metadata-only blocks
/// (`block_samples == 0`) silently. Yields `Result<WavPackBlock<'_>>`
/// once per audio block; parse errors surface verbatim and fuse the
/// iterator (the underlying [`BlockIter`] fuse mechanism). An empty
/// or all-metadata-only input yields zero items.
///
/// The wiki "Block structure" listing allows `block_samples == 0` for
/// metadata-only blocks (RIFF wrappers, MD5 sums, encoding-detail
/// payloads); callers that only care about decode-eligible blocks
/// (e.g. driving the round-206 [`WavPackBlock::decode_samples`]
/// composer per-block) would otherwise have to filter `is_audio_block`
/// on every yield. This iterator inlines the filter.
///
/// Construction: [`iter_audio_blocks`] for the call-shape twin, or
/// [`AudioBlockIter::new`] for callers that prefer the type-first form.
/// Round 230.
#[derive(Debug, Clone)]
pub struct AudioBlockIter<'a> {
    /// Underlying block iterator. Drives parse + walks the byte buffer;
    /// the audio-block filter is applied on each yield.
    blocks: BlockIter<'a>,
}

impl<'a> AudioBlockIter<'a> {
    /// Build an [`AudioBlockIter`] over the supplied byte buffer.
    /// Equivalent to [`iter_audio_blocks`] but spelled as a
    /// constructor for callers that prefer the type-first form.
    /// Round 230.
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            blocks: BlockIter::new(bytes),
        }
    }

    /// Bytes of the input buffer the underlying [`BlockIter`] has not
    /// yet consumed. Pairs with [`BlockIter::remaining`]; on a
    /// fused-error iterator this points at the malformed block's first
    /// byte for precise offset diagnostics. Round 230.
    pub fn remaining(&self) -> &'a [u8] {
        self.blocks.remaining()
    }

    /// `true` when this iterator will not yield any more items —
    /// either because the underlying [`BlockIter`] is exhausted or
    /// because a previous `next()` call returned `Err(_)` (the
    /// iterator is fused on error via the same mechanism
    /// [`BlockIter`] uses). Note this returns `true` only when the
    /// underlying iterator is exhausted — a buffer carrying only
    /// metadata-only blocks reports `false` until the iterator drains
    /// past every block. Round 230.
    pub fn is_exhausted(&self) -> bool {
        self.blocks.is_exhausted()
    }
}

impl<'a> Iterator for AudioBlockIter<'a> {
    type Item = Result<WavPackBlock<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        // Defer to BlockIter::next_audio which already implements the
        // skip-metadata-only-blocks contract.
        self.blocks.next_audio()
    }
}

impl core::iter::FusedIterator for AudioBlockIter<'_> {}

/// Construct a lazy [`AudioBlockIter`] over the supplied byte buffer.
///
/// Equivalent to [`AudioBlockIter::new`]; provided as a free function
/// for the `iter_audio_blocks(bytes)` call shape readers expect when
/// filtering a `.wv` file's blocks down to the decode-eligible ones.
///
/// Yields one `Result<WavPackBlock<'_>>` per audio block —
/// metadata-only blocks are silently skipped. See [`AudioBlockIter`]
/// for the iteration contract. Round 230.
pub fn iter_audio_blocks(bytes: &[u8]) -> AudioBlockIter<'_> {
    AudioBlockIter::new(bytes)
}

/// Lazy iterator over the blocks in a WavPack byte buffer that carry
/// `.wvc`-side correction-stream data — either the `0x0B` packed
/// correction data, the `0x07` noise-shaping profile, or both.
///
/// Wraps [`BlockIter`] so the iteration filters to blocks whose
/// [`WavPackBlock::has_correction_stream_data`] predicate fires. Yields
/// `Result<WavPackBlock<'_>>` once per correction-bearing block; parse
/// errors surface verbatim and fuse the iterator (the underlying
/// [`BlockIter`] fuse mechanism). An empty input or an input whose every
/// block carries only main-stream data yields zero items.
///
/// The wiki "IDs" listing groups the `.wvc`-side payloads (`0x07`
/// noise-shaping profile, `0x0B` packed correction data) as the
/// hybrid-mode companion content; this iterator surfaces every block
/// that carries either, regardless of whether the block is otherwise an
/// audio block (`block_samples > 0`) or a metadata-only block. The
/// hybrid-mode decode itself is gated on
/// [`UnsupportedBlockFeature::Hybrid`]; this iterator's role is
/// structural introspection — counting / locating / sizing — without
/// committing to a decode semantics. Round 233.
#[derive(Debug, Clone)]
pub struct CorrectionBlockIter<'a> {
    /// Underlying block iterator. Drives parse + walks the byte buffer;
    /// the correction-bearing filter is applied on each yield.
    blocks: BlockIter<'a>,
}

impl<'a> CorrectionBlockIter<'a> {
    /// Build a [`CorrectionBlockIter`] over the supplied byte buffer.
    /// Equivalent to [`iter_correction_blocks`] but spelled as a
    /// constructor for callers that prefer the type-first form.
    /// Round 233.
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            blocks: BlockIter::new(bytes),
        }
    }

    /// Bytes of the input buffer the underlying [`BlockIter`] has not
    /// yet consumed. Pairs with [`BlockIter::remaining`]; on a
    /// fused-error iterator this points at the malformed block's first
    /// byte for precise offset diagnostics. Round 233.
    pub fn remaining(&self) -> &'a [u8] {
        self.blocks.remaining()
    }

    /// `true` when this iterator will not yield any more items —
    /// either because the underlying [`BlockIter`] is exhausted or
    /// because a previous `next()` call returned `Err(_)` (the
    /// iterator fuses on error via the same mechanism [`BlockIter`]
    /// uses). Round 233.
    pub fn is_exhausted(&self) -> bool {
        self.blocks.is_exhausted()
    }
}

impl<'a> Iterator for CorrectionBlockIter<'a> {
    type Item = Result<WavPackBlock<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            match self.blocks.next()? {
                Ok(block) => {
                    if block.has_correction_stream_data() {
                        return Some(Ok(block));
                    }
                    // No correction-stream payload — skip and continue.
                }
                Err(e) => return Some(Err(e)),
            }
        }
    }
}

impl core::iter::FusedIterator for CorrectionBlockIter<'_> {}

/// Construct a lazy [`CorrectionBlockIter`] over the supplied byte
/// buffer.
///
/// Equivalent to [`CorrectionBlockIter::new`]; provided as a free
/// function for the `iter_correction_blocks(bytes)` call shape readers
/// expect when scanning a `.wv` file for the hybrid-mode companion
/// payloads. Round 233.
pub fn iter_correction_blocks(bytes: &[u8]) -> CorrectionBlockIter<'_> {
    CorrectionBlockIter::new(bytes)
}

/// Count the blocks in `bytes` whose sub-block list carries either of
/// the wiki `.wvc`-side payloads — `0x07` noise-shaping profile or
/// `0x0B` packed correction data.
///
/// Drives [`iter_blocks`] under the hood; every block in the buffer
/// must parse cleanly for the count to be returned (any parse error
/// surfaces verbatim). Working-set memory is one block at a time
/// regardless of input length. Round 233.
pub fn correction_block_count(bytes: &[u8]) -> Result<usize> {
    let mut iter = iter_blocks(bytes);
    let mut count = 0;
    for block in iter.by_ref() {
        let block = block?;
        if block.has_correction_stream_data() {
            count += 1;
        }
    }
    Ok(count)
}

/// Peek the first correction-bearing block in `bytes` — the first
/// block whose [`WavPackBlock::has_correction_stream_data`] predicate
/// fires — without retaining the rest of the stream.
///
/// Returns `Ok(None)` when the stream has no correction-bearing blocks
/// (empty input, or a pure lossless `.wv` file with no merged `.wvc`
/// content). Returns `Err(_)` when a block before the first
/// correction-bearing block fails to parse — the round-13
/// [`parse_block`] errors surface verbatim. Round 233.
pub fn first_correction_block(bytes: &[u8]) -> Result<Option<WavPackBlock<'_>>> {
    let mut iter = iter_correction_blocks(bytes);
    match iter.next() {
        Some(Ok(block)) => Ok(Some(block)),
        Some(Err(e)) => Err(e),
        None => Ok(None),
    }
}

/// Sum the byte lengths of every `0x0B` packed-correction-data
/// sub-block payload across every block in `bytes`.
///
/// Each `0x0B` sub-block contributes its post-walker payload byte count
/// (the wiki "Metadata" preamble guarantees an even total — the
/// odd-size flag means the round-2 walker already stripped a trailing
/// padding byte before the payload reached this counter). Returns `u64`
/// so a multi-GiB stream's aggregate correction-payload size does not
/// overflow `u32`.
///
/// Drives [`iter_blocks`] under the hood; any parse error surfaces
/// verbatim. Working-set memory is one block at a time regardless of
/// input length. Notable: this counts only `0x0B` payload bytes —
/// `0x07` noise-shaping profile bytes are excluded since they describe
/// the decoder filter rather than the correction codewords themselves.
/// Round 233.
pub fn total_correction_payload_bytes(bytes: &[u8]) -> Result<u64> {
    let mut iter = iter_blocks(bytes);
    let mut sum: u64 = 0;
    for block in iter.by_ref() {
        let block = block?;
        if let Some(view) = block.packed_correction_data() {
            sum += view.len() as u64;
        }
    }
    Ok(sum)
}

/// Lazy iterator over the PCM samples produced by every audio block in a
/// WavPack byte buffer.
///
/// Composes the round-219 [`BlockIter`] (parse) with the round-206
/// [`WavPackBlock::decode_samples`] (decode) into a single iterator that
/// yields `Result<Vec<i32>>` once per **audio** block (i.e. one element
/// per block whose `block_samples > 0`; metadata-only blocks are silently
/// skipped since they carry no PCM to return).
///
/// Each yielded `Vec<i32>` has the same shape
/// [`WavPackBlock::decode_samples`] returns: `block_samples` mono PCM
/// samples on a mono / false-stereo block, or `block_samples * 2`
/// left-then-right interleaved samples on a stereo block. Block-to-block
/// mono / stereo dispatch is per the wiki "Block structure" listing —
/// each block carries its own [`Flags::is_block_data_mono`] union of bit
/// 2 `mono` and bit 30 `false_stereo`, so the iterator does not assume a
/// uniform shape across blocks.
///
/// The iterator **fuses on the first error**: parse errors from
/// [`BlockIter`] surface verbatim ([`Error::CkSizeExceedsBuffer`] /
/// [`Error::Truncated`] / [`Error::InvalidMagic`] / …); decode errors
/// from [`WavPackBlock::decode_samples`] also surface verbatim
/// ([`Error::UnsupportedBlockFeature`] /
/// [`Error::BlockMissingEntropyInfo`] / …) — the round-219 fuse + the
/// round-206 refusal taxonomy compose without translation. Once any
/// error fires, subsequent `next()` calls return `None`.
///
/// The metadata-only-block skip is a positive contract: a `.wv` file
/// whose first block is a RIFF-header-only block (`block_samples == 0`,
/// the wiki "Block structure" allowance for metadata-only blocks)
/// still surfaces every audio block's PCM, not an [`Error::BlockHasNoAudio`]
/// refusal. Callers that want to see metadata-only blocks should drive
/// [`iter_blocks`] directly.
///
/// Construction: [`iter_decoded_blocks`] for the call-shape twin, or
/// [`StreamDecodeIter::new`] for callers that prefer the type-first form.
///
/// Round 224.
#[derive(Debug, Clone)]
pub struct StreamDecodeIter<'a> {
    /// Underlying block iterator. Drives parse + walks the byte buffer.
    blocks: BlockIter<'a>,
}

impl<'a> StreamDecodeIter<'a> {
    /// Build a [`StreamDecodeIter`] over the supplied byte buffer.
    /// Equivalent to [`iter_decoded_blocks`] but spelled as a
    /// constructor for callers that prefer the type-first form.
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            blocks: BlockIter::new(bytes),
        }
    }

    /// Bytes of the input buffer the underlying [`BlockIter`] has not yet
    /// consumed. Pairs with [`BlockIter::remaining`]; on a fused-error
    /// iterator this points at the malformed (or unsupported) block's
    /// first byte for precise offset diagnostics.
    pub fn remaining(&self) -> &'a [u8] {
        self.blocks.remaining()
    }

    /// `true` when this iterator will not yield any more items — either
    /// because the underlying [`BlockIter`] is exhausted or because a
    /// previous `next()` call returned `Err(_)` (the iterator is fused on
    /// error via the same mechanism [`BlockIter`] uses).
    pub fn is_exhausted(&self) -> bool {
        self.blocks.is_exhausted()
    }
}

impl<'a> Iterator for StreamDecodeIter<'a> {
    type Item = Result<Vec<i32>>;

    fn next(&mut self) -> Option<Self::Item> {
        // Loop over the underlying block iterator skipping metadata-only
        // blocks (block_samples == 0) until we either find an audio
        // block to decode, hit an error to fuse on, or run out of input.
        for parsed in self.blocks.by_ref() {
            match parsed {
                Ok(block) => {
                    if !block.is_audio_block() {
                        // Metadata-only block — the wiki "Block structure"
                        // allowance for block_samples == 0. No PCM to
                        // yield; advance to the next block without
                        // surfacing an Error::BlockHasNoAudio refusal.
                        continue;
                    }
                    return Some(block.decode_samples());
                }
                Err(e) => return Some(Err(e)),
            }
        }
        None
    }
}

impl core::iter::FusedIterator for StreamDecodeIter<'_> {}

/// Construct a lazy [`StreamDecodeIter`] over the supplied byte buffer.
///
/// Equivalent to [`StreamDecodeIter::new`]; provided as a free function
/// for the `iter_decoded_blocks(bytes)` call shape readers expect when
/// scanning a `.wv` file's worth of blocks and decoding each in turn.
///
/// Yields one `Result<Vec<i32>>` per **audio** block — metadata-only
/// blocks (`block_samples == 0`) are silently skipped. See
/// [`StreamDecodeIter`] for the iteration contract (empty input → zero
/// items; first error fuses the iterator).
///
/// Round 224.
pub fn iter_decoded_blocks(bytes: &[u8]) -> StreamDecodeIter<'_> {
    StreamDecodeIter::new(bytes)
}

/// Decode every audio block in a WavPack byte buffer and concatenate the
/// PCM into a single `Vec<i32>`.
///
/// Composes the round-219 [`iter_blocks`] with the round-206
/// [`WavPackBlock::decode_samples`] into a one-call "byte buffer → PCM"
/// surface for callers who hold the whole file in memory and want the
/// decoded stream up front.
///
/// Output shape:
///
/// * mono / false-stereo blocks contribute `block.block_samples()` PCM
///   `i32`s per block;
/// * stereo blocks contribute `block.block_samples() * 2` interleaved
///   left-then-right `i32`s per block (the round-199 channel-alternation
///   loop's shape preserved verbatim);
/// * metadata-only blocks (`block_samples == 0`) contribute nothing.
///
/// Blocks are concatenated in on-disk order. The returned `Vec<i32>`
/// `len()` equals `sum(block_samples * (1 if mono else 2))` across all
/// audio blocks in the input. Per-block mono / stereo dispatch is the
/// same union of wiki bit 2 `mono` + bit 30 `false_stereo` that
/// [`WavPackBlock::decode_samples`] uses; this composer applies no
/// uniform shape assumption across blocks.
///
/// Errors are surfaced from the first block that fails to parse or
/// decode — every previously-decoded block's PCM is discarded. Callers
/// who want to inspect partial output should drive [`iter_decoded_blocks`]
/// manually and collect successful elements until the iterator fuses.
///
/// Parse errors propagate verbatim from [`parse_block`]
/// ([`Error::CkSizeExceedsBuffer`] / [`Error::Truncated`] /
/// [`Error::InvalidMagic`] / [`Error::InvalidCkSize`] /
/// [`Error::UnsupportedVersion`] / metadata-walker errors). Decode
/// errors propagate verbatim from [`WavPackBlock::decode_samples`]
/// ([`Error::BlockMissingEntropyInfo`] /
/// [`Error::BlockMissingPackedSamples`] /
/// [`Error::UnsupportedBlockFeature`] / per-sample-loop errors).
///
/// Round 224.
pub fn decode_stream(bytes: &[u8]) -> Result<Vec<i32>> {
    let mut out: Vec<i32> = Vec::new();
    for chunk in iter_decoded_blocks(bytes) {
        let pcm = chunk?;
        out.extend_from_slice(&pcm);
    }
    Ok(out)
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

    // ---- Round-214 block-level discovery / accessor sweep ----

    /// Build a 0x05 entropy-info sub-block (mono) carrying the supplied
    /// three-median seed values as raw `median[i]` integers (no log-pack
    /// encoding — uses the explicit `0x09` exponent path so the on-disk
    /// median decodes to the literal seed).
    ///
    /// `wp_exp2s` with mantissa `m` and exponent `9` returns `m << 0 = m`,
    /// so a 16-bit word `[mantissa_lo, 0x09]` decodes to `median = m`.
    /// This sidesteps the log-pack helper used by
    /// `append_entropy_info_stereo_minimal` and lets the test pick the
    /// channel-0 median by value.
    fn append_entropy_info_mono_seed(payload: &mut Vec<u8>, seed: [u8; 3]) {
        let bytes = [seed[0], 0x09, seed[1], 0x09, seed[2], 0x09];
        append_small_sub_block(payload, 0x05, &bytes);
    }

    #[test]
    fn flags_accessor_returns_block_header_flags() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        // flags() borrows the same Flags the header carries.
        assert!(block.flags().mono);
        assert!(block.flags().is_block_data_mono());
        assert_eq!(block.flags().raw, block.header.flags.raw);
    }

    #[test]
    fn block_samples_accessor_returns_header_field() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(7, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.block_samples(), 7);
        assert_eq!(block.block_samples(), block.header.block_samples);
    }

    #[test]
    fn block_index_accessor_returns_header_field() {
        // synthesise_block doesn't take block_index directly, but the
        // round-1 synthesiser zeroes it; we patch the bytes in place
        // to confirm the accessor passes through.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let mut bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        // Wiki "Block structure": block_index is the 32-bit LE field at
        // offset 16 (after 4 magic + 4 ck_size + 2 version + 1 track + 1
        // sub-index + 4 total_samples).
        bytes[16..20].copy_from_slice(&12345u32.to_le_bytes());
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.block_index(), 12345);
        assert_eq!(block.block_index(), block.header.block_index);
    }

    #[test]
    fn is_audio_block_accessor_mirrors_header_predicate() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        // block_samples = 1 → audio block.
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.is_audio_block());
        assert_eq!(block.is_audio_block(), block.header.is_audio_block());

        // block_samples = 0 → metadata-only.
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.is_audio_block());
        assert_eq!(block.is_audio_block(), block.header.is_audio_block());
    }

    #[test]
    fn has_entropy_info_predicate_tracks_presence() {
        // Present: 0x05 sub-block exists.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_entropy_info());

        // Absent: no sub-blocks at all.
        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_entropy_info());
    }

    #[test]
    fn has_packed_samples_predicate_tracks_presence() {
        let mut payload = Vec::new();
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_packed_samples());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_packed_samples());
    }

    #[test]
    fn has_md5_checksum_predicate_tracks_presence() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x26, &[0u8; 16]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_md5_checksum());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_md5_checksum());
    }

    #[test]
    fn has_riff_header_predicate_tracks_presence() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x20, b"RIFF\x00\x00\x00\x00WAVEfmt ");
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_riff_header());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_riff_header());
    }

    #[test]
    fn has_riff_trailer_predicate_tracks_presence() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x21, &[0u8; 4]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_riff_trailer());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_riff_trailer());
    }

    #[test]
    fn has_multichannel_info_predicate_tracks_presence() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x0D, &[0u8; 4]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_multichannel_info());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_multichannel_info());
    }

    #[test]
    fn find_sub_block_returns_first_matching_borrow_or_none() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x00, &[0u8; 4]); // dummy
        append_small_sub_block(&mut payload, 0x26, &[0xAAu8; 16]); // md5
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let found = block
            .find_sub_block(SubBlockId::Md5Checksum)
            .expect("md5 sub-block present");
        assert_eq!(found.id, SubBlockId::Md5Checksum);
        assert_eq!(found.payload, [0xAAu8; 16].as_slice());

        // EntropyInfo not present.
        assert!(block.find_sub_block(SubBlockId::EntropyInfo).is_none());
    }

    #[test]
    fn find_entropy_info_sub_block_borrow_pairs_with_predicate() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let sub = block
            .find_entropy_info_sub_block()
            .expect("0x05 sub-block present");
        assert_eq!(sub.id, SubBlockId::EntropyInfo);
        assert_eq!(sub.payload.len(), 6);

        // Absent case.
        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.find_entropy_info_sub_block().is_none());
    }

    #[test]
    fn find_md5_checksum_sub_block_borrow_pairs_with_predicate() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x26, &[0xBBu8; 16]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let sub = block
            .find_md5_checksum_sub_block()
            .expect("0x26 sub-block present");
        assert_eq!(sub.id, SubBlockId::Md5Checksum);
        assert_eq!(sub.payload, [0xBBu8; 16].as_slice());

        // Absent case.
        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.find_md5_checksum_sub_block().is_none());
    }

    #[test]
    fn find_multichannel_info_sub_block_borrow_pairs_with_predicate() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x0D, &[0xCCu8; 4]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let sub = block
            .find_multichannel_info_sub_block()
            .expect("0x0D sub-block present");
        assert_eq!(sub.id, SubBlockId::MultichannelInfo);
        assert_eq!(sub.payload, [0xCCu8; 4].as_slice());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.find_multichannel_info_sub_block().is_none());
    }

    #[test]
    fn find_riff_header_sub_block_borrow_pairs_with_predicate() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x20, b"RIFF\x00\x00\x00\x00WAVEfmt ");
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let sub = block
            .find_riff_header_sub_block()
            .expect("0x20 sub-block present");
        assert_eq!(sub.id, SubBlockId::RiffHeader);
        assert_eq!(sub.payload, b"RIFF\x00\x00\x00\x00WAVEfmt ".as_slice());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.find_riff_header_sub_block().is_none());
    }

    #[test]
    fn find_riff_trailer_sub_block_borrow_pairs_with_predicate() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x21, &[0xDDu8; 4]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let sub = block
            .find_riff_trailer_sub_block()
            .expect("0x21 sub-block present");
        assert_eq!(sub.id, SubBlockId::RiffTrailer);
        assert_eq!(sub.payload, [0xDDu8; 4].as_slice());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.find_riff_trailer_sub_block().is_none());
    }

    #[test]
    fn packed_samples_accessor_returns_typed_view_or_none() {
        let mut payload = Vec::new();
        append_packed_samples(&mut payload, &[0xAB, 0xCD]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let view = block.packed_samples().expect("0x0A typed view present");
        assert_eq!(view.bytes(), &[0xAB, 0xCD]);
        assert_eq!(view.len(), 2);
        assert!(!view.is_empty());

        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.packed_samples().is_none());
    }

    #[test]
    fn entropy_info_returns_typed_info_for_mono_payload() {
        // Build a mono entropy-info sub-block whose seed expands to a
        // chosen channel-0 median (5). The other two median slots stay
        // at the synthesiser's chosen test values (3, 7) so the test
        // can assert the per-channel triple end-to-end.
        let mut payload = Vec::new();
        append_entropy_info_mono_seed(&mut payload, [5, 3, 7]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let info = block
            .entropy_info()
            .expect("decode entropy")
            .expect("entropy info sub-block present");
        assert!(info.is_mono());
        assert_eq!(info.medians_left, [5, 3, 7]);
        assert_eq!(info.medians_right, [0, 0, 0]);
    }

    #[test]
    fn entropy_info_returns_none_when_no_0x05_present() {
        // No 0x05 sub-block on the wire — entropy_info() returns
        // Ok(None) (the structurally legal case for metadata-only
        // blocks).
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x00, &[0u8; 2]); // dummy
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let got = block.entropy_info().expect("no decode error");
        assert!(got.is_none());
    }

    #[test]
    fn entropy_info_propagates_length_error_for_malformed_0x05() {
        // 0x05 payload of 8 bytes — neither 6 (mono) nor 12 (stereo).
        // expand_entropy reports EntropyInfoLength; entropy_info()
        // propagates verbatim.
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x05, &[0u8; 8]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let err = block.entropy_info().expect_err("must propagate length err");
        assert_eq!(err, Error::EntropyInfoLength(8));
    }

    #[test]
    fn md5_checksum_returns_typed_digest_when_0x26_present() {
        // Standard "empty input" MD5 digest as a pinned test vector
        // ("d41d8cd98f00b204e9800998ecf8427e").
        let digest_bytes: [u8; 16] = [
            0xd4, 0x1d, 0x8c, 0xd9, 0x8f, 0x00, 0xb2, 0x04, 0xe9, 0x80, 0x09, 0x98, 0xec, 0xf8,
            0x42, 0x7e,
        ];
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x26, &digest_bytes);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let md5 = block
            .md5_checksum()
            .expect("decode md5")
            .expect("0x26 sub-block present");
        assert_eq!(md5.as_bytes(), &digest_bytes);
    }

    #[test]
    fn md5_checksum_returns_none_when_no_0x26_present() {
        // No 0x26 sub-block on the wire — md5_checksum() returns
        // Ok(None) (the wiki makes the MD5 optional).
        let bytes = synthesise_block(0, flags_with(1 << 2), &[]);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let got = block.md5_checksum().expect("no decode error");
        assert!(got.is_none());
    }

    #[test]
    fn md5_checksum_propagates_length_error_for_malformed_0x26() {
        // 0x26 payload of 8 bytes instead of the wiki-fixed 16.
        // parse_md5_checksum reports Md5ChecksumLength; md5_checksum()
        // propagates verbatim.
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x26, &[0u8; 8]);
        let bytes = synthesise_block(0, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let err = block.md5_checksum().expect_err("must propagate length err");
        assert_eq!(err, Error::Md5ChecksumLength(8));
    }

    #[test]
    fn block_level_accessors_pair_with_round_206_decode_samples() {
        // End-to-end pairing: a block with both 0x05 and 0x0A and a 0x26
        // MD5 exercises the round-214 accessors alongside the round-206
        // decode loop. has_* predicates fire; entropy_info / md5_checksum
        // / packed_samples return typed views; decode_samples still works.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let digest = [0xFFu8; 16];
        append_small_sub_block(&mut payload, 0x26, &digest);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        assert!(block.is_audio_block());
        assert!(block.has_entropy_info());
        assert!(block.has_packed_samples());
        assert!(block.has_md5_checksum());
        assert!(!block.has_riff_header());
        assert!(!block.has_riff_trailer());
        assert!(!block.has_multichannel_info());
        assert!(!block.has_decorrelation());

        // Typed views are reachable through the new accessors.
        assert!(block.entropy_info().expect("decode entropy").is_some());
        assert_eq!(
            block
                .md5_checksum()
                .expect("decode md5")
                .expect("md5 present")
                .as_bytes(),
            &digest,
        );
        assert!(block.packed_samples().is_some());

        // The round-206 composer still works end-to-end.
        let pcm = block.decode_samples().expect("decode samples");
        assert_eq!(pcm, vec![0]);
    }

    // ---------------------------------------------------------------
    // Round 219 — multi-block stream iteration.
    //
    // Tests pin BlockIter / iter_blocks / parse_blocks / block_count /
    // total_block_samples against the wiki "WavPack file consists of
    // blocks each beginning with 'wvpk'" chained-block file shape.
    // ---------------------------------------------------------------

    /// Build a minimal valid block with the supplied `block_samples`
    /// header field and an empty metadata region (`ck_size == 24`).
    fn synthesise_block_with_samples(block_samples: u32) -> Vec<u8> {
        let mut bytes = synthesise_header_bytes(MIN_CK_SIZE);
        // The block_samples field lives at offset 20..24 of the 32-byte
        // header per the wiki "Block structure" listing — after magic
        // (4) + ck_size (4) + version (2) + track (2) + total_samples
        // (4) + block_index (4) = 20.
        bytes[20..24].copy_from_slice(&block_samples.to_le_bytes());
        bytes
    }

    #[test]
    fn iter_blocks_on_empty_buffer_yields_nothing() {
        // Wiki File Format makes the "no blocks" file a degenerate case;
        // the iterator returns zero items rather than erroring so the
        // caller can treat an empty buffer as a no-op.
        let mut iter = iter_blocks(&[]);
        assert!(iter.is_exhausted());
        assert!(iter.next().is_none());
        assert!(iter.remaining().is_empty());
        // FusedIterator: continued calls still return None.
        assert!(iter.next().is_none());
    }

    #[test]
    fn iter_blocks_yields_single_block_then_terminates() {
        let bytes = synthesise_header_bytes(MIN_CK_SIZE);
        let mut iter = iter_blocks(&bytes);
        let first = iter.next().expect("first item").expect("ok");
        assert_eq!(first.header.ck_size, MIN_CK_SIZE);
        assert!(iter.is_exhausted());
        assert!(iter.next().is_none());
        assert!(iter.remaining().is_empty());
    }

    #[test]
    fn iter_blocks_walks_three_back_to_back_blocks() {
        // Three identical empty blocks concatenated — the wiki "WavPack
        // file consists of blocks" shape. The iterator should yield
        // three Ok blocks in order, then terminate.
        let block = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bytes = block.clone();
        bytes.extend_from_slice(&block);
        bytes.extend_from_slice(&block);

        let mut count = 0;
        for item in iter_blocks(&bytes) {
            let b = item.expect("ok");
            assert_eq!(b.header.ck_size, MIN_CK_SIZE);
            count += 1;
        }
        assert_eq!(count, 3);
    }

    #[test]
    fn iter_blocks_remaining_shrinks_by_on_disk_len_per_step() {
        // After yielding block N, BlockIter::remaining() should equal
        // the original buffer minus the sum of the on-disk lengths of
        // every block already yielded.
        let block = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bytes = block.clone();
        bytes.extend_from_slice(&block);
        let block_on_disk = block.len();
        assert_eq!(block_on_disk, (8 + MIN_CK_SIZE) as usize);

        let mut iter = iter_blocks(&bytes);
        assert_eq!(iter.remaining().len(), 2 * block_on_disk);
        iter.next().expect("first").expect("ok");
        assert_eq!(iter.remaining().len(), block_on_disk);
        iter.next().expect("second").expect("ok");
        assert_eq!(iter.remaining().len(), 0);
        assert!(iter.next().is_none());
    }

    #[test]
    fn iter_blocks_fuses_on_first_error() {
        // First block parses; second carries a corrupt magic. After
        // yielding the first Ok and the second Err, every subsequent
        // next() must return None (FusedIterator contract).
        let good = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bad = synthesise_header_bytes(MIN_CK_SIZE);
        bad[0] = b'X';
        let mut bytes = good.clone();
        bytes.extend_from_slice(&bad);

        let mut iter = iter_blocks(&bytes);
        iter.next().expect("first").expect("ok");
        let err = iter.next().expect("second").expect_err("must reject");
        assert_eq!(err, Error::InvalidMagic);
        assert!(iter.is_exhausted());
        assert!(iter.next().is_none());
        // The remaining() slice still points at the malformed block's
        // first byte so the caller can pinpoint the offset.
        assert_eq!(iter.remaining(), bad.as_slice());
        // Fused: another call still returns None.
        assert!(iter.next().is_none());
    }

    #[test]
    fn iter_blocks_surfaces_ck_size_exceeds_buffer_on_partial_tail() {
        // First block parses cleanly. The "second" block has a valid
        // header advertising a large ck_size but the buffer is cut
        // short — the iterator should yield CkSizeExceedsBuffer (the
        // round-13 error variant that distinguishes "buffer ran out
        // inside a block" from "buffer ran out between blocks").
        let first = synthesise_header_bytes(MIN_CK_SIZE);
        let mut second = synthesise_header_bytes(200);
        // Truncate the second block to just its 32-byte header so the
        // walker sees CkSizeExceedsBuffer.
        second.truncate(HEADER_LEN);
        let mut bytes = first.clone();
        bytes.extend_from_slice(&second);

        let mut iter = iter_blocks(&bytes);
        iter.next().expect("first").expect("ok");
        match iter
            .next()
            .expect("second")
            .expect_err("partial second block")
        {
            Error::CkSizeExceedsBuffer { ck_size, available } => {
                assert_eq!(ck_size, 200);
                assert_eq!(available, HEADER_LEN);
            }
            other => panic!("expected CkSizeExceedsBuffer, got {other:?}"),
        }
        assert!(iter.is_exhausted());
    }

    #[test]
    fn iter_blocks_yields_truncated_on_partial_header_between_blocks() {
        // First block parses cleanly; the trailing buffer carries only
        // a partial header (no magic / ck_size) for the next block. The
        // iterator should report Truncated (parse_block_header's "buffer
        // ran out before HEADER_LEN" error) — distinct from
        // CkSizeExceedsBuffer per the round-13 split.
        let first = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bytes = first.clone();
        bytes.extend_from_slice(&[0u8; HEADER_LEN - 1]);

        let mut iter = iter_blocks(&bytes);
        iter.next().expect("first").expect("ok");
        let err = iter.next().expect("partial tail").expect_err("must reject");
        assert_eq!(err, Error::Truncated);
    }

    #[test]
    fn block_iter_new_matches_iter_blocks() {
        // Constructor and free function are interchangeable surfaces
        // over the same iterator type.
        let bytes = synthesise_header_bytes(MIN_CK_SIZE);
        let from_new: Vec<_> = BlockIter::new(&bytes)
            .map(|r| r.expect("ok").header.ck_size)
            .collect();
        let from_fn: Vec<_> = iter_blocks(&bytes)
            .map(|r| r.expect("ok").header.ck_size)
            .collect();
        assert_eq!(from_new, from_fn);
        assert_eq!(from_new, vec![MIN_CK_SIZE]);
    }

    #[test]
    fn parse_blocks_returns_all_blocks_in_order() {
        let block_a = synthesise_block_with_samples(100);
        let block_b = synthesise_block_with_samples(200);
        let block_c = synthesise_block_with_samples(300);
        let mut bytes = block_a.clone();
        bytes.extend_from_slice(&block_b);
        bytes.extend_from_slice(&block_c);

        let blocks = parse_blocks(&bytes).expect("parse all three");
        assert_eq!(blocks.len(), 3);
        assert_eq!(blocks[0].block_samples(), 100);
        assert_eq!(blocks[1].block_samples(), 200);
        assert_eq!(blocks[2].block_samples(), 300);
    }

    #[test]
    fn parse_blocks_bubbles_first_error() {
        // First Ok block, second malformed → parse_blocks returns the
        // first error rather than the partial Vec.
        let good = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bad = synthesise_header_bytes(MIN_CK_SIZE);
        bad[0] = b'X';
        let mut bytes = good.clone();
        bytes.extend_from_slice(&bad);

        let err = parse_blocks(&bytes).expect_err("must reject");
        assert_eq!(err, Error::InvalidMagic);
    }

    #[test]
    fn parse_blocks_returns_empty_vec_on_empty_input() {
        let blocks = parse_blocks(&[]).expect("empty input is zero blocks");
        assert!(blocks.is_empty());
    }

    #[test]
    fn block_count_returns_count_without_retaining_blocks() {
        let block = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bytes = block.clone();
        bytes.extend_from_slice(&block);
        bytes.extend_from_slice(&block);
        bytes.extend_from_slice(&block);
        bytes.extend_from_slice(&block);

        let n = block_count(&bytes).expect("count five");
        assert_eq!(n, 5);
    }

    #[test]
    fn block_count_returns_zero_on_empty_input() {
        assert_eq!(block_count(&[]).expect("zero blocks"), 0);
    }

    #[test]
    fn block_count_bubbles_first_error() {
        // Mid-stream malformed block → block_count surfaces the
        // underlying parse_block error rather than silently undercounting.
        let good = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bad = synthesise_header_bytes(MIN_CK_SIZE);
        bad[0] = b'X';
        let mut bytes = good.clone();
        bytes.extend_from_slice(&bad);

        let err = block_count(&bytes).expect_err("malformed second block");
        assert_eq!(err, Error::InvalidMagic);
    }

    #[test]
    fn total_block_samples_sums_block_samples_field_across_list() {
        let block_a = synthesise_block_with_samples(100);
        let block_b = synthesise_block_with_samples(200);
        let block_c = synthesise_block_with_samples(300);
        let mut bytes = block_a.clone();
        bytes.extend_from_slice(&block_b);
        bytes.extend_from_slice(&block_c);

        let blocks = parse_blocks(&bytes).expect("parse all");
        assert_eq!(total_block_samples(&blocks), 600);
    }

    #[test]
    fn total_block_samples_is_zero_on_empty_slice() {
        let empty: Vec<WavPackBlock<'_>> = Vec::new();
        assert_eq!(total_block_samples(&empty), 0);
    }

    #[test]
    fn total_block_samples_uses_u64_to_avoid_overflow_on_large_files() {
        // Two blocks whose 32-bit block_samples each individually fit
        // u32 but whose sum overflows u32. The u64 return type carries
        // the unrounded sum.
        let block_a = synthesise_block_with_samples(u32::MAX);
        let block_b = synthesise_block_with_samples(u32::MAX);
        let mut bytes = block_a.clone();
        bytes.extend_from_slice(&block_b);

        let blocks = parse_blocks(&bytes).expect("parse two");
        let total = total_block_samples(&blocks);
        assert_eq!(total, 2 * u32::MAX as u64);
        // Confirm we'd overflow u32 if we'd summed as u32.
        assert!(total > u32::MAX as u64);
    }

    #[test]
    fn iter_blocks_then_parse_blocks_yield_equivalent_sequences() {
        // The eager parse_blocks wrapper should be observationally
        // identical to manually draining iter_blocks.
        let block_a = synthesise_block_with_samples(10);
        let block_b = synthesise_block_with_samples(20);
        let mut bytes = block_a.clone();
        bytes.extend_from_slice(&block_b);

        let lazy: Vec<u32> = iter_blocks(&bytes)
            .map(|r| r.expect("ok").block_samples())
            .collect();
        let eager: Vec<u32> = parse_blocks(&bytes)
            .expect("eager parse")
            .iter()
            .map(|b| b.block_samples())
            .collect();
        assert_eq!(lazy, eager);
        assert_eq!(lazy, vec![10, 20]);
    }

    // ---- Round-224 decode_stream / StreamDecodeIter / iter_decoded_blocks ----

    /// Synthesise a complete one-sample mono audio block: header with
    /// `block_samples = 1`, standalone-multichannel-marker + bit 2
    /// `mono` flags, a `0x05` mono-zero entropy-info sub-block, and a
    /// `0x0A` packed-samples payload of two zero bytes (which, with the
    /// zero-medians seed, exercises the round-206 zero-run-eligible
    /// path and yields the single PCM sample `0`).
    ///
    /// Mirrors `decode_samples_returns_one_zero_for_mono_block_with_zero_seed_and_zero_unary`
    /// — kept local to the round-224 test module so the stream tests are
    /// self-contained.
    fn synthesise_decodable_mono_block_one_zero_sample() -> Vec<u8> {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with(1 << 2); // bit 2 = mono
        synthesise_block(1, flags, &payload)
    }

    #[test]
    fn decode_stream_on_empty_buffer_yields_empty_vec() {
        // The wiki "WavPack file consists of blocks" sentence is plural
        // but the empty file is a degenerate case the BlockIter accepts.
        // decode_stream inherits that: zero blocks → zero PCM samples.
        let pcm = decode_stream(&[]).expect("empty input is not an error");
        assert!(pcm.is_empty());
    }

    #[test]
    fn decode_stream_single_audio_block_yields_one_zero_pcm_sample() {
        let bytes = synthesise_decodable_mono_block_one_zero_sample();
        let pcm = decode_stream(&bytes).expect("decode stream");
        assert_eq!(pcm, vec![0]);
    }

    #[test]
    fn decode_stream_concatenates_three_audio_blocks_in_on_disk_order() {
        // Three identical one-sample mono blocks. decode_stream should
        // concatenate the PCM in on-disk order — three `0` samples.
        let block = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = block.clone();
        bytes.extend_from_slice(&block);
        bytes.extend_from_slice(&block);

        let pcm = decode_stream(&bytes).expect("decode stream of three");
        assert_eq!(pcm, vec![0, 0, 0]);
    }

    #[test]
    fn decode_stream_skips_metadata_only_blocks_between_audio_blocks() {
        // [audio][metadata-only][audio] should decode to [0, 0]; the
        // metadata-only block (block_samples == 0) is silently skipped
        // rather than triggering Error::BlockHasNoAudio.
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let metadata_only = synthesise_header_bytes(MIN_CK_SIZE);
        let mut bytes = audio.clone();
        bytes.extend_from_slice(&metadata_only);
        bytes.extend_from_slice(&audio);

        let pcm = decode_stream(&bytes).expect("decode stream skipping metadata");
        assert_eq!(pcm, vec![0, 0]);
    }

    #[test]
    fn decode_stream_with_leading_metadata_only_block_does_not_error() {
        // The wiki "Block structure" allows metadata-only blocks
        // (block_samples == 0) at the start of a `.wv` file to carry the
        // RIFF header. decode_stream must not surface BlockHasNoAudio
        // for these; it must walk past and decode the audio that follows.
        let metadata_only = synthesise_header_bytes(MIN_CK_SIZE);
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = metadata_only.clone();
        bytes.extend_from_slice(&audio);

        let pcm = decode_stream(&bytes).expect("decode stream with leading metadata");
        assert_eq!(pcm, vec![0]);
    }

    #[test]
    fn decode_stream_on_all_metadata_only_input_yields_empty_vec() {
        // No audio blocks → empty PCM. Not an error.
        let mut bytes = synthesise_header_bytes(MIN_CK_SIZE);
        bytes.extend_from_slice(&synthesise_header_bytes(MIN_CK_SIZE));

        let pcm = decode_stream(&bytes).expect("decode stream of metadata-only blocks");
        assert!(pcm.is_empty());
    }

    #[test]
    fn decode_stream_propagates_parse_error_from_malformed_block() {
        // ck_size advertises 200 bytes but only the 32-byte header is
        // present. parse_block produces CkSizeExceedsBuffer and
        // decode_stream surfaces it verbatim.
        let bytes = synthesise_header_bytes(200);
        let err = decode_stream(&bytes).expect_err("must reject malformed block");
        match err {
            Error::CkSizeExceedsBuffer { ck_size, available } => {
                assert_eq!(ck_size, 200);
                assert_eq!(available, HEADER_LEN);
            }
            other => panic!("expected CkSizeExceedsBuffer, got {other:?}"),
        }
    }

    #[test]
    fn decode_stream_propagates_unsupported_block_feature_from_decode_samples() {
        // An audio block with the hybrid (bit 3) flag set: parse cleanly,
        // but decode_samples refuses with
        // Error::UnsupportedBlockFeature(Hybrid). decode_stream surfaces
        // that verbatim.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with((1 << 2) | (1 << 3)); // mono + hybrid
        let bytes = synthesise_block(1, flags, &payload);
        let err = decode_stream(&bytes).expect_err("must refuse hybrid");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Hybrid)
        );
    }

    #[test]
    fn decode_stream_propagates_block_missing_entropy_info() {
        // Audio block (block_samples > 0) but no 0x05 sub-block →
        // decode_samples raises BlockMissingEntropyInfo. decode_stream
        // surfaces that verbatim.
        let mut payload = Vec::new();
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with(1 << 2);
        let bytes = synthesise_block(1, flags, &payload);
        let err = decode_stream(&bytes).expect_err("must require entropy info");
        assert_eq!(err, Error::BlockMissingEntropyInfo);
    }

    #[test]
    fn decode_stream_stops_at_first_decode_error_and_discards_prior_pcm() {
        // [good audio block][hybrid audio block] → decode_stream should
        // return the hybrid block's UnsupportedBlockFeature error, not
        // the leading good block's [0] PCM (eager wrapper discards
        // partial output, per the contract).
        let good = synthesise_decodable_mono_block_one_zero_sample();
        let mut hybrid_payload = Vec::new();
        append_entropy_info_mono_zero(&mut hybrid_payload);
        append_packed_samples(&mut hybrid_payload, &[0x00, 0x00]);
        let hybrid_flags = flags_with((1 << 2) | (1 << 3));
        let bad = synthesise_block(1, hybrid_flags, &hybrid_payload);

        let mut bytes = good.clone();
        bytes.extend_from_slice(&bad);

        let err = decode_stream(&bytes).expect_err("must propagate the second block's error");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Hybrid)
        );
    }

    #[test]
    fn iter_decoded_blocks_yields_one_item_per_audio_block() {
        // Three audio blocks → three Ok(Vec<i32>) items.
        let block = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = block.clone();
        bytes.extend_from_slice(&block);
        bytes.extend_from_slice(&block);

        let items: Vec<Vec<i32>> = iter_decoded_blocks(&bytes)
            .map(|r| r.expect("each block decodes"))
            .collect();
        assert_eq!(items.len(), 3);
        for it in &items {
            assert_eq!(it, &vec![0]);
        }
    }

    #[test]
    fn iter_decoded_blocks_skips_metadata_only_blocks() {
        // [metadata][audio][metadata][audio][metadata] → two Ok(Vec<i32>)
        // items; the three metadata-only blocks contribute nothing.
        let meta = synthesise_header_bytes(MIN_CK_SIZE);
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = Vec::new();
        for chunk in [&meta, &audio, &meta, &audio, &meta] {
            bytes.extend_from_slice(chunk);
        }

        let items: Vec<Vec<i32>> = iter_decoded_blocks(&bytes)
            .map(|r| r.expect("must decode"))
            .collect();
        assert_eq!(items.len(), 2);
    }

    #[test]
    fn iter_decoded_blocks_fuses_on_first_parse_error() {
        // First block parses + decodes fine; second block has bad
        // ck_size. After the first Ok, the next() must return the parse
        // error and subsequent calls must be None (fused).
        let good = synthesise_decodable_mono_block_one_zero_sample();
        let bad = synthesise_header_bytes(200); // ck_size 200 but no payload
        let mut bytes = good.clone();
        bytes.extend_from_slice(&bad);

        let mut iter = iter_decoded_blocks(&bytes);
        assert_eq!(iter.next().expect("first ok").expect("ok"), vec![0]);
        let second = iter.next().expect("second is the error");
        assert!(matches!(second, Err(Error::CkSizeExceedsBuffer { .. })));
        // Fused: third call returns None even though the underlying
        // BlockIter would re-attempt the same malformed bytes.
        assert!(iter.next().is_none());
        assert!(iter.is_exhausted());
    }

    #[test]
    fn iter_decoded_blocks_fuses_on_first_decode_error() {
        // First block parses + decodes fine; second block parses but
        // raises a decode-time UnsupportedBlockFeature error. The
        // iterator must yield the error and fuse.
        let good = synthesise_decodable_mono_block_one_zero_sample();
        let mut hybrid_payload = Vec::new();
        append_entropy_info_mono_zero(&mut hybrid_payload);
        append_packed_samples(&mut hybrid_payload, &[0x00, 0x00]);
        let bad = synthesise_block(1, flags_with((1 << 2) | (1 << 3)), &hybrid_payload);
        let mut bytes = good.clone();
        bytes.extend_from_slice(&bad);

        let mut iter = iter_decoded_blocks(&bytes);
        assert_eq!(iter.next().expect("first ok").expect("ok"), vec![0]);
        let second = iter.next().expect("second is the error");
        assert_eq!(
            second,
            Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::Hybrid
            ))
        );
        // Round-219 fuse mechanism composes through: BlockIter fuses on
        // its first error, and on a decode error the underlying iterator
        // already advanced past the bad block — so a follow-up next() may
        // return None without re-decoding. Whatever it returns, the
        // iterator's is_exhausted predicate must be true once the
        // underlying remaining-bytes slice is empty.
        let _ = iter.next();
        assert!(iter.is_exhausted());
    }

    #[test]
    fn iter_decoded_blocks_on_empty_buffer_yields_no_items() {
        let mut iter = iter_decoded_blocks(&[]);
        assert!(iter.next().is_none());
        assert!(iter.is_exhausted());
    }

    #[test]
    fn iter_decoded_blocks_on_all_metadata_only_input_yields_no_items() {
        let mut bytes = synthesise_header_bytes(MIN_CK_SIZE);
        bytes.extend_from_slice(&synthesise_header_bytes(MIN_CK_SIZE));
        let count = iter_decoded_blocks(&bytes).count();
        assert_eq!(count, 0);
    }

    #[test]
    fn stream_decode_iter_new_matches_iter_decoded_blocks() {
        // Two routes to the same iterator type must yield identical
        // sequences.
        let block = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = block.clone();
        bytes.extend_from_slice(&block);

        let from_new: Vec<Vec<i32>> = StreamDecodeIter::new(&bytes)
            .map(|r| r.expect("ok"))
            .collect();
        let from_fn: Vec<Vec<i32>> = iter_decoded_blocks(&bytes)
            .map(|r| r.expect("ok"))
            .collect();
        assert_eq!(from_new, from_fn);
        assert_eq!(from_new, vec![vec![0], vec![0]]);
    }

    #[test]
    fn stream_decode_iter_remaining_tracks_underlying_block_iter() {
        // Before any next() call, remaining() is the full buffer.
        // After draining a single audio block, remaining() advances to
        // empty.
        let block = synthesise_decodable_mono_block_one_zero_sample();
        let buf = block.clone();
        let mut iter = StreamDecodeIter::new(&buf);
        assert_eq!(iter.remaining(), buf.as_slice());
        let _ = iter.next().expect("one block");
        assert!(iter.remaining().is_empty());
        assert!(iter.is_exhausted());
    }

    #[test]
    fn stream_decode_iter_is_clone_and_fused_iterator_compliant() {
        // Static traits check: the type must implement Clone + Iterator
        // + FusedIterator. The two assertions below exercise both.
        fn assert_clone_and_fused<T: Clone + core::iter::FusedIterator>(_t: &T) {}
        let block = synthesise_decodable_mono_block_one_zero_sample();
        let bytes = block.clone();
        let iter = StreamDecodeIter::new(&bytes);
        assert_clone_and_fused(&iter);
    }

    #[test]
    fn decode_stream_handles_mixed_mono_and_stereo_blocks_in_one_input() {
        // Round-224's contract pins per-block mono / stereo dispatch.
        // A mono block followed by a stereo block must yield
        // [mono_sample, stereo_left, stereo_right] = [0, 0, 0].
        let mono = synthesise_decodable_mono_block_one_zero_sample();

        // Stereo block carrying one frame = two interleaved samples.
        // Mirrors decode_samples_returns_two_interleaved_zeros_for_stereo_block_with_minimal_seed.
        let mut stereo_payload = Vec::new();
        append_entropy_info_stereo_minimal(&mut stereo_payload);
        append_packed_samples(&mut stereo_payload, &[0x00, 0x00]);
        // Stereo block: flags carry standalone-multichannel-marker only
        // (no bit 2 mono, no bit 30 false_stereo).
        let stereo_flags = flags_with(0);
        let stereo = synthesise_block(1, stereo_flags, &stereo_payload);

        let mut bytes = mono.clone();
        bytes.extend_from_slice(&stereo);

        let pcm = decode_stream(&bytes).expect("decode mixed stream");
        // mono: [0] (1 sample), stereo: [0, 0] (1 frame × 2 channels).
        assert_eq!(pcm, vec![0, 0, 0]);
    }

    #[test]
    fn decode_stream_eager_matches_iter_decoded_blocks_flattened() {
        // The eager decode_stream must be observationally identical to
        // manually draining iter_decoded_blocks and concatenating each
        // Vec<i32> in order.
        let block = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = block.clone();
        bytes.extend_from_slice(&block);
        bytes.extend_from_slice(&block);

        let eager = decode_stream(&bytes).expect("eager decode");
        let lazy: Vec<i32> = iter_decoded_blocks(&bytes)
            .flat_map(|r| r.expect("ok"))
            .collect();
        assert_eq!(eager, lazy);
        assert_eq!(eager, vec![0, 0, 0]);
    }

    // ---- Round-230 stream-level introspection accessors ----

    /// Synthesise a metadata-only block (`block_samples == 0`) carrying
    /// no sub-blocks — the wiki "Block structure" header-only allowance.
    fn synthesise_metadata_only_block() -> Vec<u8> {
        // block_samples = 0; flags zero. The `synthesise_header_bytes`
        // helper sets the header into a parseable shape with no
        // metadata region (ck_size == MIN_CK_SIZE).
        synthesise_header_bytes(MIN_CK_SIZE)
    }

    /// Synthesise a single decodable stereo block carrying one stereo
    /// frame (two interleaved samples). Mirrors the stereo helper
    /// `decode_samples_returns_two_interleaved_zeros_for_stereo_block_with_minimal_seed`
    /// uses but as a reusable helper for the stream-level tests.
    fn synthesise_decodable_stereo_block_one_frame() -> Vec<u8> {
        let mut payload = Vec::new();
        append_entropy_info_stereo_minimal(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        synthesise_block(1, flags_with(0), &payload)
    }

    #[test]
    fn decoded_sample_count_on_mono_block_equals_block_samples() {
        // Mono / false-stereo block: one i32 per sample.
        let bytes = synthesise_decodable_mono_block_one_zero_sample();
        let (block, _) = parse_block(&bytes).expect("parse mono");
        assert!(block.header.flags.is_block_data_mono());
        assert_eq!(block.block_samples(), 1);
        assert_eq!(block.decoded_sample_count(), 1);
    }

    #[test]
    fn decoded_sample_count_on_stereo_block_equals_block_samples_times_two() {
        // Stereo block: two interleaved i32s per sample frame.
        let bytes = synthesise_decodable_stereo_block_one_frame();
        let (block, _) = parse_block(&bytes).expect("parse stereo");
        assert!(!block.header.flags.is_block_data_mono());
        assert_eq!(block.block_samples(), 1);
        assert_eq!(block.decoded_sample_count(), 2);
    }

    #[test]
    fn decoded_sample_count_on_metadata_only_block_is_zero() {
        // block_samples == 0 → metadata-only block. Mono / stereo
        // dispatch does not matter; 0 * anything = 0.
        let bytes = synthesise_metadata_only_block();
        let (block, _) = parse_block(&bytes).expect("parse metadata-only");
        assert_eq!(block.block_samples(), 0);
        assert!(!block.is_audio_block());
        assert_eq!(block.decoded_sample_count(), 0);
    }

    #[test]
    fn decoded_sample_count_matches_decode_samples_len_on_mono() {
        // Sanity: the structural count must equal the actual PCM length
        // decode_samples returns on success.
        let bytes = synthesise_decodable_mono_block_one_zero_sample();
        let (block, _) = parse_block(&bytes).expect("parse mono");
        let pcm = block.decode_samples().expect("decode");
        assert_eq!(block.decoded_sample_count() as usize, pcm.len());
    }

    #[test]
    fn decoded_sample_count_matches_decode_samples_len_on_stereo() {
        let bytes = synthesise_decodable_stereo_block_one_frame();
        let (block, _) = parse_block(&bytes).expect("parse stereo");
        let pcm = block.decode_samples().expect("decode");
        assert_eq!(block.decoded_sample_count() as usize, pcm.len());
    }

    #[test]
    fn block_iter_next_audio_skips_leading_metadata_only_block() {
        // [metadata-only, audio] → next_audio() returns the audio block.
        let meta = synthesise_metadata_only_block();
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&audio);

        let mut iter = iter_blocks(&bytes);
        let block = iter
            .next_audio()
            .expect("audio block")
            .expect("parses cleanly");
        assert!(block.is_audio_block());
        assert_eq!(block.block_samples(), 1);
        // The metadata-only block has been consumed by next_audio.
        assert!(iter.is_exhausted());
    }

    #[test]
    fn block_iter_next_audio_returns_none_on_all_metadata_only_input() {
        // Pure metadata-only stream → no audio block ever appears.
        let meta = synthesise_metadata_only_block();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&meta);

        let mut iter = iter_blocks(&bytes);
        assert!(iter.next_audio().is_none());
        assert!(iter.is_exhausted());
    }

    #[test]
    fn block_iter_next_audio_returns_none_on_empty_input() {
        let mut iter = iter_blocks(&[]);
        assert!(iter.next_audio().is_none());
    }

    #[test]
    fn block_iter_next_audio_propagates_parse_error() {
        // Audio block followed by a corrupt block: next_audio yields
        // the audio block, then the parse error on the next call.
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bad = synthesise_header_bytes(MIN_CK_SIZE);
        bad[0] = b'X';
        let mut bytes = audio.clone();
        bytes.extend_from_slice(&bad);

        let mut iter = iter_blocks(&bytes);
        let block = iter
            .next_audio()
            .expect("audio block")
            .expect("parses cleanly");
        assert!(block.is_audio_block());
        let err = iter
            .next_audio()
            .expect("second yield")
            .expect_err("must reject");
        assert_eq!(err, Error::InvalidMagic);
        // Fused after error: next_audio returns None.
        assert!(iter.next_audio().is_none());
    }

    #[test]
    fn audio_block_count_on_empty_buffer_is_zero() {
        assert_eq!(audio_block_count(&[]).expect("count empty"), 0);
    }

    #[test]
    fn audio_block_count_counts_only_audio_blocks() {
        // [metadata, audio, metadata, audio, audio] → 3 audio blocks.
        let meta = synthesise_metadata_only_block();
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&meta);
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&audio);
        assert_eq!(audio_block_count(&bytes).expect("count"), 3);
    }

    #[test]
    fn audio_block_count_on_all_metadata_only_input_is_zero() {
        let meta = synthesise_metadata_only_block();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&meta);
        assert_eq!(audio_block_count(&bytes).expect("count"), 0);
    }

    #[test]
    fn audio_block_count_propagates_parse_error() {
        let mut bad = synthesise_header_bytes(MIN_CK_SIZE);
        bad[0] = b'X';
        let err = audio_block_count(&bad).expect_err("must reject");
        assert_eq!(err, Error::InvalidMagic);
    }

    #[test]
    fn metadata_block_count_inverse_of_audio_block_count() {
        // The two counters together sum to block_count.
        let meta = synthesise_metadata_only_block();
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&meta);
        bytes.extend_from_slice(&audio);

        let total = block_count(&bytes).expect("count");
        let audio_n = audio_block_count(&bytes).expect("audio count");
        let meta_n = metadata_block_count(&bytes).expect("meta count");
        assert_eq!(total, 4);
        assert_eq!(audio_n, 2);
        assert_eq!(meta_n, 2);
        assert_eq!(audio_n + meta_n, total);
    }

    #[test]
    fn metadata_block_count_on_empty_buffer_is_zero() {
        assert_eq!(metadata_block_count(&[]).expect("count empty"), 0);
    }

    #[test]
    fn total_audio_samples_sums_block_samples_across_audio_blocks() {
        // One audio block carries block_samples = 1 (the helper sets it
        // to 1). Three such blocks plus a leading metadata-only block
        // → total = 3.
        let meta = synthesise_metadata_only_block();
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&audio);
        assert_eq!(total_audio_samples(&bytes).expect("sum"), 3u64);
    }

    #[test]
    fn total_audio_samples_on_empty_buffer_is_zero() {
        assert_eq!(total_audio_samples(&[]).expect("sum empty"), 0u64);
    }

    #[test]
    fn total_audio_samples_on_all_metadata_only_input_is_zero() {
        let meta = synthesise_metadata_only_block();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&meta);
        assert_eq!(total_audio_samples(&bytes).expect("sum"), 0u64);
    }

    #[test]
    fn total_audio_samples_propagates_parse_error() {
        let mut bad = synthesise_header_bytes(MIN_CK_SIZE);
        bad[0] = b'X';
        let err = total_audio_samples(&bad).expect_err("must reject");
        assert_eq!(err, Error::InvalidMagic);
    }

    #[test]
    fn decoded_sample_count_stream_level_sums_per_block_counts() {
        // [mono(1), stereo(1), metadata, mono(1)] → 1 + 2 + 0 + 1 = 4.
        let mono = synthesise_decodable_mono_block_one_zero_sample();
        let stereo = synthesise_decodable_stereo_block_one_frame();
        let meta = synthesise_metadata_only_block();
        let mut bytes = mono.clone();
        bytes.extend_from_slice(&stereo);
        bytes.extend_from_slice(&meta);
        bytes.extend_from_slice(&mono);
        assert_eq!(decoded_sample_count(&bytes).expect("count"), 4u64);
    }

    #[test]
    fn decoded_sample_count_stream_level_matches_decode_stream_len() {
        // Sanity: the structural count must equal the actual PCM length
        // decode_stream returns on success.
        let mono = synthesise_decodable_mono_block_one_zero_sample();
        let stereo = synthesise_decodable_stereo_block_one_frame();
        let mut bytes = mono.clone();
        bytes.extend_from_slice(&stereo);
        let count = decoded_sample_count(&bytes).expect("count");
        let pcm = decode_stream(&bytes).expect("decode");
        assert_eq!(count as usize, pcm.len());
    }

    #[test]
    fn decoded_sample_count_stream_level_on_empty_buffer_is_zero() {
        assert_eq!(decoded_sample_count(&[]).expect("count"), 0u64);
    }

    #[test]
    fn first_audio_block_skips_leading_metadata_only_blocks() {
        let meta = synthesise_metadata_only_block();
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&meta);
        bytes.extend_from_slice(&audio);

        let block = first_audio_block(&bytes)
            .expect("ok")
            .expect("audio present");
        assert!(block.is_audio_block());
        assert_eq!(block.block_samples(), 1);
    }

    #[test]
    fn first_audio_block_returns_none_on_empty_buffer() {
        assert!(first_audio_block(&[]).expect("ok").is_none());
    }

    #[test]
    fn first_audio_block_returns_none_on_all_metadata_only_input() {
        let meta = synthesise_metadata_only_block();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&meta);
        assert!(first_audio_block(&bytes).expect("ok").is_none());
    }

    #[test]
    fn first_audio_block_propagates_parse_error_before_audio() {
        // Bad block first; first_audio_block surfaces the error rather
        // than skipping past it.
        let mut bad = synthesise_header_bytes(MIN_CK_SIZE);
        bad[0] = b'X';
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = bad.clone();
        bytes.extend_from_slice(&audio);
        let err = first_audio_block(&bytes).expect_err("must reject");
        assert_eq!(err, Error::InvalidMagic);
    }

    #[test]
    fn iter_audio_blocks_skips_metadata_only_blocks_and_yields_audio() {
        // [metadata, audio, metadata, audio] → 2 audio yields.
        let meta = synthesise_metadata_only_block();
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&meta);
        bytes.extend_from_slice(&audio);

        let mut iter = iter_audio_blocks(&bytes);
        let a = iter.next().expect("first audio").expect("parses cleanly");
        assert!(a.is_audio_block());
        let b = iter.next().expect("second audio").expect("parses cleanly");
        assert!(b.is_audio_block());
        assert!(iter.next().is_none());
        assert!(iter.is_exhausted());
    }

    #[test]
    fn iter_audio_blocks_on_empty_buffer_yields_zero_items() {
        assert!(iter_audio_blocks(&[]).next().is_none());
    }

    #[test]
    fn iter_audio_blocks_on_all_metadata_only_yields_zero_items() {
        let meta = synthesise_metadata_only_block();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&meta);
        let mut iter = iter_audio_blocks(&bytes);
        assert!(iter.next().is_none());
        // After draining, the underlying BlockIter is exhausted.
        assert!(iter.is_exhausted());
    }

    #[test]
    fn iter_audio_blocks_fuses_on_parse_error() {
        // audio, then a corrupt block. The audio yields; then the
        // parse error; then None forever.
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bad = synthesise_header_bytes(MIN_CK_SIZE);
        bad[0] = b'X';
        let mut bytes = audio.clone();
        bytes.extend_from_slice(&bad);

        let mut iter = iter_audio_blocks(&bytes);
        iter.next().expect("audio").expect("ok");
        let err = iter.next().expect("second yield").expect_err("must reject");
        assert_eq!(err, Error::InvalidMagic);
        assert!(iter.is_exhausted());
        assert!(iter.next().is_none());
    }

    #[test]
    fn iter_audio_blocks_constructor_and_new_return_identical_sequences() {
        // Two construction paths must produce the same items.
        let meta = synthesise_metadata_only_block();
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&audio);

        let via_new: Vec<u32> = AudioBlockIter::new(&bytes)
            .map(|r| r.expect("ok").block_samples())
            .collect();
        let via_fn: Vec<u32> = iter_audio_blocks(&bytes)
            .map(|r| r.expect("ok").block_samples())
            .collect();
        assert_eq!(via_new, via_fn);
        assert_eq!(via_new, vec![1, 1]);
    }

    #[test]
    fn iter_audio_blocks_remaining_tracks_underlying_block_iter() {
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = audio.clone();
        bytes.extend_from_slice(&audio);
        let iter = iter_audio_blocks(&bytes);
        assert_eq!(iter.remaining().len(), bytes.len());
    }

    #[test]
    fn iter_audio_blocks_clone_and_fused_iterator_trait_bounds_hold() {
        // Compile-time check: AudioBlockIter must be Clone + FusedIterator.
        fn assert_clone_fused<I: Iterator + core::iter::FusedIterator + Clone>(_: &I) {}
        let bytes: Vec<u8> = Vec::new();
        let iter = iter_audio_blocks(&bytes);
        assert_clone_fused(&iter);
    }

    #[test]
    fn audio_block_count_equals_iter_audio_blocks_count() {
        // The two surfaces (free function counter / iterator length)
        // must agree on every input.
        let meta = synthesise_metadata_only_block();
        let audio = synthesise_decodable_mono_block_one_zero_sample();
        let mut bytes = meta.clone();
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&meta);
        bytes.extend_from_slice(&audio);
        bytes.extend_from_slice(&audio);

        let via_fn = audio_block_count(&bytes).expect("count");
        let via_iter = iter_audio_blocks(&bytes).count();
        assert_eq!(via_fn, via_iter);
        assert_eq!(via_fn, 3);
    }

    // ---- Round-233 .wvc correction-stream typed view + introspection ----

    /// Append a 0x0B packed-correction-data sub-block with the supplied
    /// payload bytes. Must be even-length (sub-block size is in 16-bit
    /// words and we don't set the odd-size flag here).
    fn append_packed_correction_data(payload: &mut Vec<u8>, bytes: &[u8]) {
        append_small_sub_block(payload, 0x0B, bytes);
    }

    /// Append a 0x07 noise-shaping-profile sub-block with the supplied
    /// payload bytes.
    fn append_noise_shaping_profile(payload: &mut Vec<u8>, bytes: &[u8]) {
        append_small_sub_block(payload, 0x07, bytes);
    }

    /// Append a 0x06 hybrid-profile sub-block with the supplied payload
    /// bytes.
    fn append_hybrid_profile(payload: &mut Vec<u8>, bytes: &[u8]) {
        append_small_sub_block(payload, 0x06, bytes);
    }

    #[test]
    fn has_packed_correction_data_returns_false_on_no_0x0b_subblock() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_packed_correction_data());
        assert!(block.packed_correction_data().is_none());
        assert!(block.find_packed_correction_data_sub_block().is_none());
    }

    #[test]
    fn has_packed_correction_data_returns_true_with_0x0b_subblock() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0xAA, 0xBB]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_packed_correction_data());

        let view = block.packed_correction_data().expect("typed view");
        assert_eq!(view.bytes(), &[0xAA, 0xBB]);
        assert_eq!(view.len(), 2);

        let sub = block
            .find_packed_correction_data_sub_block()
            .expect("borrow");
        assert_eq!(sub.id, SubBlockId::PackedCorrectionData);
        assert_eq!(sub.payload, &[0xAA, 0xBB]);
    }

    #[test]
    fn has_noise_shaping_profile_returns_false_on_no_0x07_subblock() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_noise_shaping_profile());
        assert!(block.find_noise_shaping_profile_sub_block().is_none());
    }

    #[test]
    fn has_noise_shaping_profile_returns_true_with_0x07_subblock() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_noise_shaping_profile(&mut payload, &[0x55, 0x66]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_noise_shaping_profile());
        let sub = block
            .find_noise_shaping_profile_sub_block()
            .expect("borrow");
        assert_eq!(sub.id, SubBlockId::NoiseShapingProfile);
        assert_eq!(sub.payload, &[0x55, 0x66]);
    }

    #[test]
    fn has_hybrid_profile_returns_false_on_no_0x06_subblock() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_hybrid_profile());
        assert!(block.find_hybrid_profile_sub_block().is_none());
    }

    #[test]
    fn has_hybrid_profile_returns_true_with_0x06_subblock() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_hybrid_profile(&mut payload, &[0x10, 0x20]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_hybrid_profile());
        let sub = block.find_hybrid_profile_sub_block().expect("borrow");
        assert_eq!(sub.id, SubBlockId::HybridProfile);
        assert_eq!(sub.payload, &[0x10, 0x20]);
    }

    #[test]
    fn has_correction_stream_data_is_union_of_0x07_and_0x0b() {
        // No 0x07 / 0x0B → false.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_correction_stream_data());

        // 0x0B only → true.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0x01, 0x02]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_correction_stream_data());

        // 0x07 only → true.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_noise_shaping_profile(&mut payload, &[0x03, 0x04]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_correction_stream_data());

        // Both 0x07 + 0x0B → true.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_noise_shaping_profile(&mut payload, &[0x03, 0x04]);
        append_packed_correction_data(&mut payload, &[0x05, 0x06]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_correction_stream_data());
    }

    #[test]
    fn packed_correction_data_typed_view_round_trips_payload_bytes() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0xDE, 0xAD, 0xBE, 0xEF]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let view = block.packed_correction_data().expect("typed view");
        assert_eq!(view.bytes(), &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert!(!view.is_empty());
        assert_eq!(view.len(), 4);

        // bit_reader starts at byte 0 / bit 0
        let r = view.bit_reader();
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 0);
        assert_eq!(r.bits_remaining(), 32);
    }

    #[test]
    fn correction_block_count_on_empty_buffer_is_zero() {
        let bytes: &[u8] = &[];
        assert_eq!(correction_block_count(bytes).unwrap(), 0);
    }

    #[test]
    fn correction_block_count_zero_when_no_correction_payloads() {
        // Single audio block, no 0x07 / 0x0B.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        assert_eq!(correction_block_count(&bytes).unwrap(), 0);
    }

    #[test]
    fn correction_block_count_counts_blocks_with_either_payload() {
        // Build a stream of: [plain audio][0x0B-only][0x07-only][both]
        let mut plain_payload = Vec::new();
        append_entropy_info_mono_zero(&mut plain_payload);
        append_packed_samples(&mut plain_payload, &[0x00, 0x00]);
        let plain = synthesise_block(1, flags_with(1 << 2), &plain_payload);

        let mut wvc_payload = Vec::new();
        append_entropy_info_mono_zero(&mut wvc_payload);
        append_packed_samples(&mut wvc_payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut wvc_payload, &[0x01, 0x02]);
        let wvc_only = synthesise_block(1, flags_with(1 << 2), &wvc_payload);

        let mut shape_payload = Vec::new();
        append_entropy_info_mono_zero(&mut shape_payload);
        append_packed_samples(&mut shape_payload, &[0x00, 0x00]);
        append_noise_shaping_profile(&mut shape_payload, &[0x03, 0x04]);
        let shape_only = synthesise_block(1, flags_with(1 << 2), &shape_payload);

        let mut both_payload = Vec::new();
        append_entropy_info_mono_zero(&mut both_payload);
        append_packed_samples(&mut both_payload, &[0x00, 0x00]);
        append_noise_shaping_profile(&mut both_payload, &[0x05, 0x06]);
        append_packed_correction_data(&mut both_payload, &[0x07, 0x08]);
        let both = synthesise_block(1, flags_with(1 << 2), &both_payload);

        let mut stream = Vec::new();
        stream.extend_from_slice(&plain);
        stream.extend_from_slice(&wvc_only);
        stream.extend_from_slice(&shape_only);
        stream.extend_from_slice(&both);

        // 3 of 4 blocks have correction-stream data.
        assert_eq!(correction_block_count(&stream).unwrap(), 3);
    }

    #[test]
    fn correction_block_count_propagates_parse_error() {
        // Trailing 3 bytes too short for a header → Truncated.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0x09, 0x0A]);
        let good = synthesise_block(1, flags_with(1 << 2), &payload);
        let mut bytes = good.clone();
        bytes.extend_from_slice(&[0u8; 3]);
        let err = correction_block_count(&bytes).expect_err("must reject trailing");
        assert_eq!(err, Error::Truncated);
    }

    #[test]
    fn first_correction_block_returns_none_on_empty_buffer() {
        let bytes: &[u8] = &[];
        assert!(first_correction_block(bytes).unwrap().is_none());
    }

    #[test]
    fn first_correction_block_skips_blocks_without_correction_data() {
        // [plain][plain][correction][plain] — first_correction_block
        // walks past the leading two and returns the third.
        let mut plain_payload = Vec::new();
        append_entropy_info_mono_zero(&mut plain_payload);
        append_packed_samples(&mut plain_payload, &[0x00, 0x00]);
        let plain = synthesise_block(1, flags_with(1 << 2), &plain_payload);

        let mut wvc_payload = Vec::new();
        append_entropy_info_mono_zero(&mut wvc_payload);
        append_packed_samples(&mut wvc_payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut wvc_payload, &[0xAB, 0xCD]);
        let wvc = synthesise_block(1, flags_with(1 << 2), &wvc_payload);

        let mut stream = Vec::new();
        stream.extend_from_slice(&plain);
        stream.extend_from_slice(&plain);
        stream.extend_from_slice(&wvc);
        stream.extend_from_slice(&plain);

        let block = first_correction_block(&stream).expect("ok").expect("some");
        let view = block.packed_correction_data().expect("typed view");
        assert_eq!(view.bytes(), &[0xAB, 0xCD]);
    }

    #[test]
    fn first_correction_block_returns_none_when_no_correction_blocks_present() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        assert!(first_correction_block(&bytes).unwrap().is_none());
    }

    #[test]
    fn iter_correction_blocks_on_empty_buffer_yields_zero_items() {
        let bytes: &[u8] = &[];
        let mut iter = iter_correction_blocks(bytes);
        assert!(iter.next().is_none());
        assert!(iter.is_exhausted());
    }

    #[test]
    fn iter_correction_blocks_skips_blocks_without_correction_payloads() {
        // [plain][wvc][plain][wvc][wvc] → iterator yields 3 items.
        let mut plain_payload = Vec::new();
        append_entropy_info_mono_zero(&mut plain_payload);
        append_packed_samples(&mut plain_payload, &[0x00, 0x00]);
        let plain = synthesise_block(1, flags_with(1 << 2), &plain_payload);

        let mut wvc_payload = Vec::new();
        append_entropy_info_mono_zero(&mut wvc_payload);
        append_packed_samples(&mut wvc_payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut wvc_payload, &[0xAB, 0xCD]);
        let wvc = synthesise_block(1, flags_with(1 << 2), &wvc_payload);

        let mut stream = Vec::new();
        stream.extend_from_slice(&plain);
        stream.extend_from_slice(&wvc);
        stream.extend_from_slice(&plain);
        stream.extend_from_slice(&wvc);
        stream.extend_from_slice(&wvc);

        let yielded: Vec<_> = iter_correction_blocks(&stream)
            .map(|b| b.expect("ok"))
            .collect();
        assert_eq!(yielded.len(), 3);
        for block in &yielded {
            assert!(block.has_correction_stream_data());
        }
    }

    #[test]
    fn iter_correction_blocks_fuses_on_parse_error() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0xAA, 0xBB]);
        let good = synthesise_block(1, flags_with(1 << 2), &payload);
        let mut bytes = good.clone();
        bytes.extend_from_slice(&[0u8; 3]); // truncated trailing
        let mut iter = iter_correction_blocks(&bytes);
        // First item: the good correction block.
        let first = iter.next().expect("first").expect("ok");
        assert!(first.has_correction_stream_data());
        // Second item: the parse error.
        let err = iter.next().expect("err").expect_err("truncated");
        assert_eq!(err, Error::Truncated);
        // Third call: fused → None.
        assert!(iter.next().is_none());
        assert!(iter.is_exhausted());
    }

    #[test]
    fn iter_correction_blocks_constructor_and_new_return_identical_sequences() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0x11, 0x22]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);

        let via_fn: Vec<_> = iter_correction_blocks(&bytes).map(|b| b.is_ok()).collect();
        let via_ctor: Vec<_> = CorrectionBlockIter::new(&bytes)
            .map(|b| b.is_ok())
            .collect();
        assert_eq!(via_fn, via_ctor);
    }

    #[test]
    fn iter_correction_blocks_remaining_tracks_underlying_block_iter() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0x33, 0x44]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);

        let mut iter = iter_correction_blocks(&bytes);
        assert_eq!(iter.remaining().len(), bytes.len());
        let _ = iter.next();
        assert!(iter.remaining().is_empty());
    }

    #[test]
    fn correction_block_iter_clone_and_fused_iterator_trait_bounds_hold() {
        // Compile-time trait assertions analogous to the AudioBlockIter
        // test pattern: the iterator implements Clone and FusedIterator.
        fn assert_clone<T: Clone>() {}
        fn assert_fused<T: core::iter::FusedIterator>() {}
        assert_clone::<CorrectionBlockIter<'_>>();
        assert_fused::<CorrectionBlockIter<'_>>();
    }

    #[test]
    fn correction_block_count_equals_iter_correction_blocks_count() {
        let mut wvc_payload = Vec::new();
        append_entropy_info_mono_zero(&mut wvc_payload);
        append_packed_samples(&mut wvc_payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut wvc_payload, &[0xCD, 0xEF]);
        let wvc = synthesise_block(1, flags_with(1 << 2), &wvc_payload);

        let mut plain_payload = Vec::new();
        append_entropy_info_mono_zero(&mut plain_payload);
        append_packed_samples(&mut plain_payload, &[0x00, 0x00]);
        let plain = synthesise_block(1, flags_with(1 << 2), &plain_payload);

        let mut bytes = wvc.clone();
        bytes.extend_from_slice(&plain);
        bytes.extend_from_slice(&wvc);

        let via_fn = correction_block_count(&bytes).expect("count");
        let via_iter = iter_correction_blocks(&bytes).count();
        assert_eq!(via_fn, via_iter);
        assert_eq!(via_fn, 2);
    }

    #[test]
    fn total_correction_payload_bytes_on_empty_buffer_is_zero() {
        let bytes: &[u8] = &[];
        assert_eq!(total_correction_payload_bytes(bytes).unwrap(), 0);
    }

    #[test]
    fn total_correction_payload_bytes_zero_when_no_0x0b() {
        // Plain audio block — no 0x0B → 0 correction bytes.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        assert_eq!(total_correction_payload_bytes(&bytes).unwrap(), 0);
    }

    #[test]
    fn total_correction_payload_bytes_sums_only_0x0b_payload_bytes() {
        // Block 1: 4 bytes of 0x0B
        // Block 2: 0x07 only (excluded) + 6 bytes of 0x0B
        // Block 3: nothing
        // Expected: 4 + 6 = 10
        let mut p1 = Vec::new();
        append_entropy_info_mono_zero(&mut p1);
        append_packed_samples(&mut p1, &[0x00, 0x00]);
        append_packed_correction_data(&mut p1, &[1, 2, 3, 4]);
        let b1 = synthesise_block(1, flags_with(1 << 2), &p1);

        let mut p2 = Vec::new();
        append_entropy_info_mono_zero(&mut p2);
        append_packed_samples(&mut p2, &[0x00, 0x00]);
        append_noise_shaping_profile(&mut p2, &[0xAA, 0xBB]);
        append_packed_correction_data(&mut p2, &[5, 6, 7, 8, 9, 10]);
        let b2 = synthesise_block(1, flags_with(1 << 2), &p2);

        let mut p3 = Vec::new();
        append_entropy_info_mono_zero(&mut p3);
        append_packed_samples(&mut p3, &[0x00, 0x00]);
        let b3 = synthesise_block(1, flags_with(1 << 2), &p3);

        let mut bytes = b1;
        bytes.extend_from_slice(&b2);
        bytes.extend_from_slice(&b3);

        assert_eq!(total_correction_payload_bytes(&bytes).unwrap(), 10);
    }

    #[test]
    fn total_correction_payload_bytes_propagates_parse_error() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0x01, 0x02]);
        let good = synthesise_block(1, flags_with(1 << 2), &payload);
        let mut bytes = good.clone();
        bytes.extend_from_slice(&[0u8; 3]); // truncated trailing header
        let err = total_correction_payload_bytes(&bytes).expect_err("must reject");
        assert_eq!(err, Error::Truncated);
    }

    #[test]
    fn metadata_only_block_carrying_only_0x0b_is_correction_bearing() {
        // A block with block_samples == 0 that carries only a 0x0B
        // payload — a "correction-only metadata block" — must still be
        // surfaced as correction-bearing. Wiki "Block structure" allows
        // block_samples == 0 for metadata-only blocks; the wiki places
        // no rule forbidding a wvc-side payload alongside zero audio
        // samples in a merged file.
        let mut payload = Vec::new();
        append_packed_correction_data(&mut payload, &[0xFE, 0xED]);
        let bytes = synthesise_block(0, flags_with(0), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.is_audio_block());
        assert!(block.has_packed_correction_data());
        assert!(block.has_correction_stream_data());
    }

    #[test]
    fn block_with_correction_data_but_unsupported_hybrid_flag_still_refuses_decode() {
        // The presence of a 0x0B payload does NOT make decode_samples
        // succeed on a hybrid block — the per-sample loop still gates
        // on the hybrid flag. This pins the contract: the typed view
        // is structural introspection, not a decode-enablement.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_correction_data(&mut payload, &[0x01, 0x02]);
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 3)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_packed_correction_data());
        let err = block.decode_samples().expect_err("must still refuse");
        assert_eq!(
            err,
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Hybrid)
        );
    }
}
