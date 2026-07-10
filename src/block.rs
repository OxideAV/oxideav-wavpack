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
use crate::decorrelation::{
    assemble_mono_passes, assemble_stereo_passes, decorrelate_mono, decorrelate_stereo,
};
use crate::entropy::{expand_entropy, EntropyInfo};
use crate::error::{Error, Result};
use crate::metadata::{
    find_entropy_info, find_first, find_md5_checksum_block, find_multichannel_info,
    find_packed_samples, parse_md5_checksum, walk_metadata, Md5Checksum, MetadataSubBlock,
    SubBlockId,
};
use crate::packed_samples::PackedSamples;
use crate::samples::decode_packed_samples_mono_from_entropy;

/// Anti-amplification ceiling on the per-block decoded sample count.
///
/// The wiki "Block structure" listing
/// (`docs/audio/wavpack/wiki/WavPack.wiki`) gives `block_samples` as a
/// bare 32-bit field with no documented upper bound, and the spec §4.2
/// step 1 zero-run fast path lets a single ~63-bit run word expand to
/// ~`2^31` zero samples — so the emitted count is **not** bounded by the
/// `0x0A` payload byte length. Without a ceiling, a ~44-byte block can
/// set `block_samples` near `u32::MAX` and force
/// [`WavPackBlock::decode_samples`] to grow a multi-gigabyte output
/// `Vec` before the bitstream genuinely runs dry — an allocation
/// amplification denial of service.
///
/// `1 << 26` (67,108,864) samples per channel corresponds to roughly 25
/// minutes of mono audio at 44.1 kHz carried in a single block — far
/// beyond any plausible real block — while keeping the worst-case eager
/// allocation bounded at a few hundred megabytes. A block whose
/// `block_samples` exceeds this surfaces
/// [`Error::BlockSamplesTooLarge`](crate::Error::BlockSamplesTooLarge)
/// rather than attempting the allocation. This is a defensive
/// engineering bound, not a spec-mandated limit.
pub const MAX_DECODE_SAMPLES_PER_BLOCK: u32 = 1 << 26;

/// Maximum number of decoded channels a single multichannel set may sum
/// to before [`decode_multichannel_stream`] refuses it with
/// [`Error::MultichannelTooManyChannels`](crate::Error::MultichannelTooManyChannels).
///
/// A multichannel set is a run of 1- or 2-channel member blocks (wiki
/// bits 11..=12 grouping); the sum of their channel counts is the
/// interleaved frame width. WavPack's Microsoft-channel-mask carriage is
/// a 32-bit mask, so 32 distinct speaker positions is the natural ceiling
/// for a well-formed file; this bound guards against a malformed stream
/// chaining unbounded members before the interleave buffer is sized. A
/// defensive engineering bound, not a spec-mandated limit. Round 378.
pub const MAX_MULTICHANNEL_CHANNELS: usize = 256;

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
    /// decorrelation sub-blocks (terms / weights / samples).
    ///
    /// **No longer raised by [`WavPackBlock::decode_samples`]** — both mono
    /// and stereo lossless decorrelation are now decoded (entropy →
    /// `assemble_*_passes` → `decorrelate_*` → PCM). Retained as a public
    /// variant for API stability and for callers that gate on it.
    Decorrelation,
    /// Wiki bit 31 ("low-latency block (experimental, do not decode if
    /// encountered)") is set. The wiki explicitly bars decode of this
    /// block; the composer honours that ban.
    LowLatencyBlock,
    /// Wiki bit 28 ("robust block (experimental, okay to ignore)") is
    /// set.
    ///
    /// **No longer raised by [`WavPackBlock::decode_samples`]** — the wiki
    /// marks the bit "okay to ignore if encountered", and a round-393
    /// black-box cross-validation (wvunpack as opaque binary) showed
    /// reference-encoded files set it on every block, so the earlier
    /// conservative refusal rejected every real file. Retained as a
    /// public variant for API stability.
    RobustBlock,
    /// Wiki bit 4 ("joint stereo coding scheme") is set on a stereo
    /// block: the two decoded channels are a mid/side (sum / difference)
    /// pair.
    ///
    /// **No longer raised by [`WavPackBlock::decode_samples`]** — the spec
    /// §5.4 inverse joint-stereo transform (`R -= L>>1; L += R`) is now
    /// applied per pair after decorrelation. Retained as a public variant
    /// for API stability.
    JointStereo,
    /// Wiki bit 5 (`CROSS_DECORR`, "cross-decorrelation scheme is used")
    /// is set on a stereo block. The staged decorrelation-spec doc §4.1
    /// documents this flag only in the hybrid-stereo correction-folding
    /// context (a zero-delay correction fold *before* the decorrelation
    /// passes); on a non-hybrid lossless main-stream block it has no
    /// documented meaning, so the block is refused rather than decoded
    /// with a guessed transform. (The lossless inter-channel predictors
    /// are the negative `0x02` decorr *terms* `-1`/`-2`/`-3`, which ARE
    /// decoded by the stereo path.)
    CrossChannelDecorrelation,
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
            UnsupportedBlockFeature::JointStereo => "joint-stereo (mid/side) coding (flag bit 4)",
            UnsupportedBlockFeature::CrossChannelDecorrelation => {
                "cross-channel decorrelation (flag bit 5)"
            }
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
    /// decorrelation sub-blocks. [`Self::decode_samples`] consumes these
    /// for both mono and stereo lossless blocks (entropy residuals →
    /// `assemble_mono_passes` / `assemble_stereo_passes` →
    /// `decorrelate_mono` / `decorrelate_stereo` → PCM), so this predicate
    /// reports whether the decode runs the §3 prediction loop rather than
    /// passing the entropy output through unchanged. Round 206.
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

    /// `true` when this block **wants** a `.wvc` correction twin to
    /// reconstruct losslessly: the wiki bit-3 hybrid flag is set, so the
    /// main `0x0A` stream alone is lossy and only pairing it with the
    /// companion correction words (spec §4.1 — two residuals per sample
    /// position) recovers the exact original. A pure-lossless block
    /// (`hybrid` clear) reports `false`: a paired correction block would
    /// be redundant for it. Pairs with
    /// [`crate::block::pair_correction_stream`] to classify coverage on
    /// a two-file decode. Round 386.
    pub fn expects_correction(&self) -> bool {
        self.header.flags.hybrid
    }

    /// Which decorrelation-spec §4.1 correction-fold placement this block's
    /// flag word selects ([`crate::CorrectionFold`]).
    ///
    /// Derived from the block header's raw flag word: the
    /// `HYBRID_SHAPE` / `NEW_SHAPING` bits select
    /// [`crate::CorrectionFold::NoiseShaped`], `CROSS_DECORR` selects
    /// [`crate::CorrectionFold::PreDecorrelationCross`], and otherwise the
    /// default [`crate::CorrectionFold::PostDecorrelation`] raw add applies.
    /// Only the post-decorrelation placement is folded end-to-end by
    /// [`Self::fold_hybrid_correction`]; the other two require, respectively,
    /// re-running decorrelation after a pre-pass fold or the (undocumented)
    /// noise-shaping filter. Round 367.
    pub fn hybrid_correction_placement(&self) -> crate::CorrectionFold {
        crate::CorrectionFold::from_flags(self.header.flags.raw)
    }

    /// Recover lossless PCM from an already-decoded **lossy** buffer and a
    /// matching buffer of correction residuals, applying the
    /// decorrelation-spec §4.1 post-decorrelation correction fold.
    ///
    /// In hybrid mode the lossy main stream (`0x0A`) is made lossless by
    /// folding a correction residual (read from the `0x0B` stream) into
    /// each reconstructed sample. For the common case — a mono block, or a
    /// stereo block *without* `CROSS_DECORR`, and *without* noise shaping —
    /// the spec §4.1 fold is the per-sample raw add
    /// `lossless = reconstructed + correction` after the decorrelation
    /// passes and joint-stereo undo have produced the reconstructed lossy
    /// buffer. This method applies exactly that fold (via
    /// [`crate::fold_correction`]) element-wise.
    ///
    /// `lossy` is the reconstructed lossy PCM in the same shape
    /// [`Self::decode_samples`] returns (mono samples, or interleaved
    /// `[L0, R0, …]` for stereo); `correction` is the correction residual
    /// for each of those samples in the same order. The result is the
    /// recovered lossless PCM.
    ///
    /// This is a pure arithmetic consumer: it does **not** decode either
    /// entropy stream (the lossy main stream's `error_limit`-driven decode
    /// and the correction stream's own entropy decode are separate, and the
    /// former remains a documented gap). It lets a caller that has obtained
    /// both buffers by other means recover lossless samples with one call.
    ///
    /// # Errors
    ///
    /// * [`Error::HybridFoldPlacementUnsupported`] — the block's flags
    ///   select [`crate::CorrectionFold::PreDecorrelationCross`] or
    ///   [`crate::CorrectionFold::NoiseShaped`], neither of which is a plain
    ///   post-decorrelation raw add.
    /// * [`Error::HybridCorrectionLengthMismatch`] — `correction.len() !=
    ///   lossy.len()` (the §4.1 fold reads exactly one correction residual
    ///   per decoded sample).
    ///
    /// Round 367.
    pub fn fold_hybrid_correction(&self, lossy: &[i32], correction: &[i32]) -> Result<Vec<i32>> {
        if !self.hybrid_correction_placement().is_supported_raw_fold() {
            return Err(Error::HybridFoldPlacementUnsupported);
        }
        if lossy.len() != correction.len() {
            return Err(Error::HybridCorrectionLengthMismatch {
                lossy: lossy.len(),
                correction: correction.len(),
            });
        }
        Ok(lossy
            .iter()
            .zip(correction.iter())
            .map(|(&s, &c)| crate::fold_correction(s, c))
            .collect())
    }

    /// Compute the correction residuals an encoder packs into the `0x0B`
    /// stream — the exact forward (encode) inverse of
    /// [`Self::fold_hybrid_correction`].
    ///
    /// Given the `original` lossless PCM and the `lossy` reconstruction the
    /// hybrid encoder emitted into the main `0x0A` stream, returns the
    /// per-sample correction residual `correction = original - lossy` (via
    /// [`crate::split_correction`]) so that
    /// `fold_hybrid_correction(lossy, split_hybrid_correction(original,
    /// lossy)) == original`. Both buffers are in the same per-channel shape
    /// [`Self::decode_samples`] uses (mono, or interleaved `[L0, R0, …]`).
    ///
    /// As with the decode-side fold, this is the plain post-decorrelation
    /// (spec §4.1) case: the `CROSS_DECORR` / noise-shaped placements are
    /// refused, and the two buffers must be the same length.
    ///
    /// # Errors
    ///
    /// * [`Error::HybridFoldPlacementUnsupported`] — the block's flags
    ///   select a placement other than the post-decorrelation raw add.
    /// * [`Error::HybridCorrectionLengthMismatch`] — `original.len() !=
    ///   lossy.len()`.
    ///
    /// Round 367.
    pub fn split_hybrid_correction(&self, original: &[i32], lossy: &[i32]) -> Result<Vec<i32>> {
        if !self.hybrid_correction_placement().is_supported_raw_fold() {
            return Err(Error::HybridFoldPlacementUnsupported);
        }
        if original.len() != lossy.len() {
            return Err(Error::HybridCorrectionLengthMismatch {
                lossy: lossy.len(),
                correction: original.len(),
            });
        }
        Ok(original
            .iter()
            .zip(lossy.iter())
            .map(|(&o, &s)| crate::split_correction(o, s))
            .collect())
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

    /// `true` when a `0x0C` packed-overflow-bits sub-block is
    /// present. The wiki "IDs" listing annotates this ID as "packed
    /// overflow bits from floating-point or large integers" and the
    /// clean-room entropy doc names the same ID as the extension
    /// bitstream. Presence indicates the block was encoded with the
    /// float (wiki bit 7) or int32 (wiki bit 8) container fix-up, so
    /// the main `0x0A` decode alone does not reconstruct the
    /// per-sample value — the overflow bits supply the high-order
    /// bits the consumer fix-up needs. Round 242.
    pub fn has_packed_overflow_bits(&self) -> bool {
        self.contains_sub_block(SubBlockId::PackedOverflowBits)
    }

    /// Borrow the first `0x0C` packed-overflow-bits sub-block, or
    /// `None` when none is present. Block-level pairing with the
    /// free [`crate::find_packed_overflow_bits_sub_block`] finder.
    /// Use [`Self::packed_overflow_bits`] for the typed-view variant.
    /// Round 242.
    pub fn find_packed_overflow_bits_sub_block(&self) -> Option<&MetadataSubBlock<'a>> {
        crate::metadata::find_packed_overflow_bits_sub_block(&self.sub_blocks)
    }

    /// Locate the `0x0C` packed-overflow-bits sub-block and wrap it
    /// as a typed [`crate::PackedOverflowBits`] view, or return
    /// `None` when no `0x0C` sub-block is present. Block-level
    /// pairing with the free [`crate::find_packed_overflow_bits`]
    /// finder. Round 242.
    pub fn packed_overflow_bits(&self) -> Option<crate::PackedOverflowBits<'a>> {
        crate::metadata::find_packed_overflow_bits(&self.sub_blocks)
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

    /// The block's sample rate in Hz, resolving both documented
    /// carriers (staged spec `wavpack-sample-formats.md` §5):
    ///
    /// * header flag bits 23..=26 select one of the 15 standard rates
    ///   ([`crate::STANDARD_SAMPLE_RATES`]);
    /// * the sentinel index `15` defers to the `0x27`
    ///   non-standard-sampling-rate sub-block (3-byte little-endian
    ///   Hz), which is emitted once for the stream, with the first
    ///   block.
    ///
    /// Returns `Ok(None)` when the index is the custom sentinel and
    /// this block carries no `0x27` sub-block (later blocks of a
    /// custom-rate stream — resolve via the stream's first block, or
    /// [`crate::stream_sample_rate`]). Returns
    /// [`Error::SampleRatePayloadLength`] for a malformed `0x27`
    /// payload. Round 405.
    pub fn sample_rate(&self) -> Result<Option<u32>> {
        if let Some(rate) = self.header.flags.standard_sample_rate() {
            return Ok(Some(rate));
        }
        match crate::metadata::find_non_standard_sample_rate(&self.sub_blocks) {
            Some(sub) => Ok(Some(crate::metadata::parse_non_standard_sample_rate(
                sub.payload,
            )?)),
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
    /// * [`UnsupportedBlockFeature::LowLatencyBlock`] — wiki bit 31
    ///   "low-latency block (experimental, do not decode if encountered)"
    ///   set; the wiki explicitly bars decode. (Bit 28 "robust block"
    ///   is *ignored* per its wiki "okay to ignore" labelling — see
    ///   [`UnsupportedBlockFeature::RobustBlock`].)
    /// * [`UnsupportedBlockFeature::CrossChannelDecorrelation`] — wiki
    ///   bit 5 (`CROSS_DECORR`) set on a non-hybrid stereo block; the
    ///   decorrelation-spec doc §4.1 documents this flag only in the
    ///   hybrid-stereo correction-folding context, so a lossless
    ///   main-stream block carrying it has no documented meaning.
    ///
    /// Decorrelation (the `0x02`/`0x03`/`0x04` sub-blocks) is *decoded*,
    /// not refused, for both mono and stereo blocks (spec §3), and the
    /// joint-stereo (wiki bit 4) mid/side undo is applied per spec §5.4.
    ///
    /// The cross-channel-decorrelation refusal is gated on
    /// [`Flags::is_block_data_mono`] being `false` — a mono / false-stereo
    /// block has only one decoded channel and is unaffected.
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
    /// * [`Error::PackedSamplesOddLength`] — the `0x0A` packed-samples
    ///   payload has an odd byte count, which
    ///   `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 rejects
    ///   ("byte length must be even or the block is rejected").
    ///
    /// Errors from [`crate::expand_entropy`] and the per-sample loop
    /// (truncation, EOF, malformed `EntropyInfo`, …) are propagated
    /// verbatim. The composer is itself stateless: each call seeds a
    /// fresh per-channel [`crate::AdaptiveMedians`] from the block's
    /// `0x05` payload and drops it on return, so back-to-back blocks
    /// of a multi-block file behave like independent decodes (which is
    /// what plain stereo `.wv` carries on the wire anyway — each block
    /// carries a fresh `0x05` seed). Round 206.
    ///
    /// ## Final left-shift normalization (round 354)
    ///
    /// The returned PCM has the wiki flag-bits-13..=17 *left-shift fixup*
    /// applied: when the block's effective bit-depth is not a whole number
    /// of bytes (12-bit, 20-bit, …) the encoder narrowed every sample and
    /// recorded the dropped trailing-zero count in [`Flags::left_shift`];
    /// the decoder reconstructs the narrow magnitude through the prediction
    /// loop and then shifts each sample left by that count to restore the
    /// container-scaled PCM ([`crate::apply_left_shift_buffer`]). For the
    /// common whole-byte depths `left_shift` is `0` and this is the
    /// identity. The shift is applied *after* the running block CRC is
    /// folded over the pre-shift samples (decorrelation-spec doc §1
    /// pipeline + §5.2 "before final shift"), so the CRC checkers
    /// ([`Self::verify_decoded_crc`], [`Self::decode_samples_muted`]) fold
    /// the pre-shift buffer to compare against the stored header CRC.
    pub fn decode_samples(&self) -> Result<Vec<i32>> {
        let mut pcm = self.decode_samples_preshift()?;
        // Round 405: the int32 sample-format fixup (0x0C extension bits
        // + redundancy re-insertion) runs between the CRC fold and the
        // final left shift. The plain decode does not enforce the
        // extension-CRC verdict (matching its posture on the main CRC —
        // use the muted twins for the §5.6 gate).
        self.apply_int32_fixup(&mut pcm)?;
        // Spec §1 pipeline / §5.2: the left-shift fixup is the final
        // normalization stage, applied after the CRC fold. The CRC paths
        // call `decode_samples_preshift` directly and fold the pre-shift
        // buffer, so applying the shift here keeps the public PCM correct
        // without disturbing the CRC comparison.
        crate::fixup::apply_left_shift_buffer(&mut pcm, self.header.flags.left_shift);
        Ok(pcm)
    }

    /// Decode this block's PCM up to but **not including** the final
    /// left-shift normalization fixup — i.e. the buffer in the exact form
    /// the running §5 block CRC is computed over (decorrelation-spec doc §1
    /// pipeline: "… joint-stereo undo → accumulate CRC → shift/clip
    /// fixups", and §5.2 "after decorrelation, **before final shift**").
    ///
    /// [`Self::decode_samples`] wraps this and applies the wiki
    /// flag-bits-13..=17 left-shift ([`crate::apply_left_shift_buffer`]) to
    /// produce the final container-scaled PCM; the CRC checkers
    /// ([`Self::verify_decoded_crc`] / [`Self::decode_samples_muted`]) fold
    /// this pre-shift buffer directly so the comparison matches the stored
    /// header CRC, then apply the shift to whatever PCM they return.
    fn decode_samples_preshift(&self) -> Result<Vec<i32>> {
        self.decode_samples_preshift_inner(false)
    }

    /// Shared pre-shift decode body for both the standalone-block path
    /// ([`Self::decode_samples_preshift`], `allow_member == false`) and the
    /// multichannel-member path ([`Self::decode_member_preshift`],
    /// `allow_member == true`).
    ///
    /// The only difference between a standalone block and a member of a
    /// multi-block multichannel set is the wiki bits-11..=12 grouping
    /// marker: the marker is a *stream-shape* signal (where this member's
    /// channels sit in the interleaved frame), not a decode-arithmetic
    /// signal. Every per-sample step — entropy decode, decorrelation,
    /// joint-stereo undo, the §5 CRC fold and the final left-shift — is
    /// identical for a member and a standalone block of the same channel
    /// shape. So the member path reuses this whole body verbatim and only
    /// suppresses the [`UnsupportedBlockFeature::MultichannelMember`]
    /// refusal; the grouping itself is reassembled one layer up by
    /// [`decode_multichannel_stream`].
    fn decode_samples_preshift_inner(&self, allow_member: bool) -> Result<Vec<i32>> {
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
        // INT32_DATA (bit 8) is no longer refused here: the large /
        // shifted-integer reduction is undone by the round-405
        // `apply_int32_fixup` stage the public decode paths run after
        // this pre-shift body (staged spec `wavpack-sample-formats.md`
        // §3/§4 — the `0x0C` extension bits and redundancy re-insertion
        // happen at fixup/normalise time, outside the prediction loop,
        // and the §5 main CRC folds over the buffer *this* body
        // returns).
        // Wiki bit 28 "robust block (experimental, okay to ignore if
        // encountered)" is exactly that — ignorable. Real-world
        // encoders set it routinely, and a round-393 black-box
        // cross-validation (wvunpack as opaque binary) showed
        // reference-encoded files carry it on every block, so refusing
        // it rejected every real file. Only bit 31 (below) carries the
        // wiki's "do not decode" instruction.
        if flags.low_latency_block {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::LowLatencyBlock,
            ));
        }
        if flags.is_multichannel_member() && !allow_member {
            return Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::MultichannelMember,
            ));
        }
        // Decorrelation: both the mono and the stereo lossless paths are
        // wired. The decorrelation-spec doc §3 specifies the per-channel
        // term arithmetic + weight adaptation + §3.7 reverse-order pass
        // assembly for one channel (mono) and for both interleaved
        // channels in lockstep (stereo); §3.6 ties the `0x03`/`0x04`
        // per-channel weight/seed layout to the term class. So the
        // generic decorrelation refusal is gone — the per-channel split
        // below dispatches to the matching assembler.
        let has_decorr = self.has_decorrelation();
        // Inter-channel transforms only apply to genuinely-stereo block
        // data (two decoded channels). A mono or false-stereo block
        // carries a single channel, so these flags — even if a malformed
        // encoder left them set — have no second channel to combine and
        // are not a correctness hazard for the mono path. Guard on
        // `is_block_data_mono()` so the mono / false-stereo loop is not
        // needlessly refused.
        // Wiki bit 5 (`CROSS_DECORR` 0x20) is *ignored* on a lossless
        // block: the staged decorrelation-spec doc documents the flag's
        // only consumer in the hybrid correction-fold placement (§4.1 —
        // "the correction is folded in *before* the decorrelation
        // passes" when set), and the §1 lossless per-block decode order
        // (entropy → decorr passes → joint undo → CRC → shift) has no
        // step that consults it. Hybrid blocks are refused above, so
        // every block reaching this point decodes identically with the
        // bit set or clear. A round-393 black-box cross-validation
        // (wvunpack as an opaque binary) confirmed reference encoders
        // set the bit on plain lossless stereo files — the earlier
        // conservative refusal rejected every real stereo file. (The
        // lossless inter-channel predictors are the negative `0x02`
        // decorr *terms* `-1`/`-2`/`-3`, which ARE decoded by the
        // stereo path below;
        // `UnsupportedBlockFeature::CrossChannelDecorrelation` is no
        // longer raised here.)

        // Structural sub-block lookup. Both must be present for the
        // per-sample loop to have inputs.
        let entropy_sub =
            find_entropy_info(&self.sub_blocks).ok_or(Error::BlockMissingEntropyInfo)?;
        let packed = find_packed_samples(&self.sub_blocks)
            .ok_or(Error::BlockMissingPackedSamples)?
            // Spec §1: the 0x0A main-bitstream payload byte length must
            // be even or the block is rejected (the reader binds it as
            // 16-bit words). Refuse an odd payload before the per-sample
            // loop runs.
            .validate_length()?;

        // Round-4 expander on the 0x05 payload + round-201 wrappers on
        // the 0x0A payload give us the whole pipe in two calls.
        // Anti-amplification guard: `block_samples` is an unbounded
        // 32-bit field and the spec §4.2 step 1 zero-run path lets it
        // expand far beyond the `0x0A` payload byte length, so an absurd
        // value must be rejected before it sizes the per-sample loop's
        // output `Vec`. See [`MAX_DECODE_SAMPLES_PER_BLOCK`].
        if header.block_samples > MAX_DECODE_SAMPLES_PER_BLOCK {
            return Err(Error::BlockSamplesTooLarge(header.block_samples));
        }

        let entropy = expand_entropy(entropy_sub.payload)?;
        let count = header.block_samples as usize;

        if flags.is_block_data_mono() {
            let mut residuals = decode_packed_samples_mono_from_entropy(&packed, &entropy, count)?;
            if has_decorr {
                // The entropy stream carried residuals, not PCM. Assemble
                // the §3.7 application-ordered pass list from the
                // 0x02/0x03/0x04 sub-blocks and run the §3.2 inverse
                // prediction loop over the residual buffer in place,
                // reconstructing the lossless PCM samples.
                let (terms, weights, samples) = self.decorr_payloads();
                let mut passes = assemble_mono_passes(terms, weights, samples)?;
                decorrelate_mono(&mut passes, &mut residuals)?;
            }
            Ok(residuals)
        } else {
            // Stereo-ness is decided by the block FLAGS plus the 0x05
            // payload length (wiki: one 6-byte set for mono, two sets =
            // 12 bytes for stereo) — NOT by the expanded content. An
            // all-zero right median set is a legitimate stereo payload
            // (fresh seeds), indistinguishable from a mono payload only
            // if the on-wire length is ignored; the reference encoder's
            // zero-seed form and this crate's own encoder both emit it.
            // The content-heuristic `decode_packed_samples_stereo_from_entropy`
            // wrapper (which refuses an all-zero right set) stays for
            // API compatibility; this flags-gated path checks the wire
            // length instead. Round 393.
            if entropy_sub.payload.len() != crate::entropy::STEREO_PAYLOAD_BYTES {
                return Err(Error::InvalidEntropyInfoForStereo);
            }
            let mut medians = [
                crate::samples::AdaptiveMedians::from_seed_values(entropy.medians_left)
                    .ok_or(Error::InvalidEntropyInfoForStereo)?,
                crate::samples::AdaptiveMedians::from_seed_values(entropy.medians_right)
                    .ok_or(Error::InvalidEntropyInfoForStereo)?,
            ];
            let mut residuals =
                crate::samples::decode_packed_samples_stereo(&packed, &mut medians, count)?;
            if has_decorr {
                // Stereo residuals arrive interleaved [L0, R0, L1, R1, …].
                // Assemble the §3.7 application-ordered stereo pass list
                // (two weights + per-channel seeds per pass, §3.6) and run
                // the §3.2/§3.3 inverse prediction loop over both
                // interleaved channels in lockstep, in place.
                let (terms, weights, samples) = self.decorr_payloads();
                let mut passes = assemble_stereo_passes(terms, weights, samples)?;
                decorrelate_stereo(&mut passes, &mut residuals)?;
            }
            if flags.joint_stereo {
                // Spec §5.4: undo mid/side joint stereo per pair *after*
                // decorrelation, *before* CRC accumulation. The decoded
                // pair is (mid, side); recover (left, right).
                for pair in residuals.chunks_exact_mut(2) {
                    let (left, right) = crate::crc::undo_joint_stereo(pair[0], pair[1]);
                    pair[0] = left;
                    pair[1] = right;
                }
            }
            Ok(residuals)
        }
    }

    /// `true` when this block carries `1` decoded channel (mono / false-
    /// stereo, the [`Flags::is_block_data_mono`] union), `false` when it
    /// carries `2` (interleaved stereo). The per-member channel count a
    /// multichannel set sums over its members. Round 378.
    fn member_channel_count(&self) -> usize {
        if self.header.flags.is_block_data_mono() {
            1
        } else {
            2
        }
    }

    /// Decode a **multichannel-set member** block's PCM up to but not
    /// including the final left-shift fixup.
    ///
    /// Identical to [`Self::decode_samples_preshift`] except the
    /// wiki-bits-11..=12 grouping marker is accepted instead of refused —
    /// the marker is a stream-shape signal, not a decode-arithmetic one
    /// (see [`Self::decode_samples_preshift_inner`]). The returned buffer
    /// is the member's own channels in this block's shape (one `i32` per
    /// sample for a 1-channel member, interleaved `L,R` for a 2-channel
    /// member), in the pre-shift form the §5 CRC is folded over.
    /// Round 378.
    fn decode_member_preshift(&self) -> Result<Vec<i32>> {
        self.decode_samples_preshift_inner(true)
    }

    /// Decode a **multichannel-set member** block's PCM, with the final
    /// left-shift normalization applied (the public-PCM form).
    ///
    /// This is the member twin of [`Self::decode_samples`]: it decodes a
    /// block that participates in a multi-block multichannel grouping
    /// (wiki bits 11..=12 ≠ `0b11`) without raising
    /// [`UnsupportedBlockFeature::MultichannelMember`], returning the
    /// member's own `1` or `2` channels. A standalone block (marker
    /// `0b11`) is also accepted — the result is then exactly
    /// [`Self::decode_samples`]. Callers reassembling a full multichannel
    /// frame interleave the per-member buffers via
    /// [`decode_multichannel_stream`]; this method is the per-member leg.
    ///
    /// All other refusals ([`UnsupportedBlockFeature::Hybrid`],
    /// `FloatData`, `Int32Mode`, `LowLatencyBlock`,
    /// `CrossChannelDecorrelation`) and structural errors still fire — a
    /// member exercising those is no more decodable than a standalone
    /// block that does. Round 378.
    pub fn decode_member_samples(&self) -> Result<Vec<i32>> {
        let mut pcm = self.decode_member_preshift()?;
        // Round 405: int32 sample-format fixup (see
        // `apply_int32_fixup`), verdict unenforced on the plain path.
        self.apply_int32_fixup(&mut pcm)?;
        crate::fixup::apply_left_shift_buffer(&mut pcm, self.header.flags.left_shift);
        Ok(pcm)
    }

    /// Decode a multichannel-set member's PCM with the spec §5.6 CRC
    /// *mute gate* applied — the member twin of
    /// [`Self::decode_samples_muted`].
    ///
    /// Each member block carries its own §5 running CRC over its own
    /// channels; this folds that CRC over the member's reconstructed
    /// pre-shift PCM and, on a mismatch with the stored header CRC word,
    /// zeros the returned buffer (the spec's "mute the corrupt block"
    /// behaviour). Returns `(pcm, crc_ok)` exactly as
    /// [`Self::decode_samples_muted`] does. Round 378.
    pub fn decode_member_samples_muted(&self) -> Result<(Vec<i32>, bool)> {
        let mut pcm = self.decode_member_preshift()?;
        let mut crc_ok = self.crc_of_decoded(&pcm) == self.header.crc();
        if crc_ok {
            // Round 405: extension-CRC verdict joins the §5.6 gate.
            if let Some((computed, stored)) = self.apply_int32_fixup(&mut pcm)? {
                crc_ok = computed == stored;
            }
        }
        if crc_ok {
            crate::fixup::apply_left_shift_buffer(&mut pcm, self.header.flags.left_shift);
        } else {
            pcm.iter_mut().for_each(|s| *s = 0);
        }
        Ok((pcm, crc_ok))
    }

    /// Decode this block's PCM ([`Self::decode_samples`]) and verify the
    /// running §5 block CRC against the stored header CRC word
    /// ([`Self::crc`]).
    ///
    /// Returns `Ok(true)` when the recomputed CRC matches the stored word
    /// (spec §5.6: a conformant decoder would *keep* the block), `Ok(false)`
    /// when it does not (spec §5.6: a conformant decoder would *mute* the
    /// block by zeroing it). Any error [`Self::decode_samples`] raises
    /// (unsupported feature, structural shortfall, malformed payload) is
    /// propagated verbatim.
    ///
    /// The CRC is folded over the decoded PCM in the spec's per-channel
    /// shape: [`crate::crc::crc_mono`] for a mono / false-stereo block and
    /// [`crate::crc::crc_stereo_interleaved`] for a stereo block —
    /// matching the channel dispatch [`Self::decode_samples`] itself uses.
    /// [`Self::decode_samples`] already applies the spec §5.4 mid/side undo
    /// inside the stereo path, so the buffer this checker folds is always
    /// in true L/R form; the plain (non-joint) stereo CRC step is therefore
    /// the correct fold for both joint and non-joint stereo blocks here.
    ///
    /// This is a non-mutating *checker* — it does not alter the decoded
    /// buffer on mismatch. Callers wanting the spec's mute behaviour zero
    /// the returned PCM themselves on `Ok(false)`. Round 339.
    pub fn verify_decoded_crc(&self) -> Result<bool> {
        // Spec §5.2 / §1: the CRC is folded over the *pre-shift* samples
        // (before the final left-shift normalization). Fold the pre-shift
        // buffer so the comparison matches the stored header CRC even for
        // sub-byte-depth blocks (non-zero `left_shift`).
        let mut pcm = self.decode_samples_preshift()?;
        let main_ok = self.crc_of_decoded(&pcm) == self.header.crc();
        // Spec §5.6: when a 0x0C extension stream participated, the
        // accumulated crc_x must also match its stored crc_wvx.
        let ext_ok = match self.apply_int32_fixup(&mut pcm)? {
            Some((computed, stored)) => computed == stored,
            None => true,
        };
        Ok(main_ok && ext_ok)
    }

    /// Fold the spec §5 running CRC over an already-decoded PCM buffer in
    /// the block's per-channel shape ([`crate::crc::crc_mono`] for mono /
    /// false-stereo, [`crate::crc::crc_stereo_interleaved`] for stereo).
    ///
    /// The buffer must be the true-L/R output [`Self::decode_samples`]
    /// produces (the §5.4 mid/side undo already applied), so the plain
    /// stereo CRC step is correct for both joint and non-joint blocks.
    fn crc_of_decoded(&self, pcm: &[i32]) -> u32 {
        if self.header.flags.is_block_data_mono() {
            crate::crc::crc_mono(pcm)
        } else {
            crate::crc::crc_stereo_interleaved(pcm)
        }
    }

    /// Apply the round-405 **int32 sample-format fixup** in place
    /// (staged spec `wavpack-sample-formats.md` §3/§4): when flag bit 8
    /// (`INT32_DATA`) is set, complete each entropy-decoded sample with
    /// its `sent_bits` literal low bits from the `0x0C` extension
    /// bitstream and re-insert the stripped redundancy pattern, per the
    /// block's `0x09` int32-info profile. Runs at fixup/normalise time —
    /// after decorrelation and the §5 main-CRC fold, before the header
    /// left-shift.
    ///
    /// Returns `Some((computed_crc_x, stored_crc_wvx))` when a `0x0C`
    /// extension stream participated (the spec §5.5/§5.6 comparison
    /// inputs; the caller decides whether to enforce it), `None` when
    /// the block is not int32 or moved no extension bits. Typed
    /// refusals: [`Error::BlockMissingInt32Info`],
    /// [`Error::BlockMissingOverflowBits`], [`Error::Int32InfoLength`],
    /// [`Error::Int32InfoConflict`], [`Error::OverflowBitsTooShort`].
    fn apply_int32_fixup(&self, pcm: &mut [i32]) -> Result<Option<(u32, u32)>> {
        if !self.header.flags.int32_mode {
            return Ok(None);
        }
        let info_sub = self
            .find_sub_block(SubBlockId::Int32Info)
            .ok_or(Error::BlockMissingInt32Info)?;
        let info = crate::int32::expand_int32_info(info_sub.payload)?;
        if info.requires_extension() {
            let overflow = self
                .packed_overflow_bits()
                .ok_or(Error::BlockMissingOverflowBits)?;
            let stored = overflow.crc_wvx()?;
            let mut reader = overflow.extension_bit_reader()?;
            let computed = crate::int32::reassemble_int32(pcm, &info, Some(&mut reader))?;
            Ok(Some((computed, stored)))
        } else {
            // Redundancy-only reduction: no literal bits to read. The
            // spec ties the crc_x accumulation to the presence of a
            // 0x0C stream (decorrelation doc §5.5), so compare only
            // when one is on the wire.
            let computed = crate::int32::reassemble_int32(pcm, &info, None)?;
            match self.packed_overflow_bits() {
                Some(overflow) => Ok(Some((computed, overflow.crc_wvx()?))),
                None => Ok(None),
            }
        }
    }

    /// Decode this block's PCM and apply the spec §5.6 CRC *mute gate*:
    /// recompute the running block CRC over the decoded samples and, on a
    /// mismatch with the stored header CRC word, zero the returned buffer
    /// (the spec's "mute the block" behaviour) — otherwise return the
    /// decoded PCM unchanged.
    ///
    /// Returns `(pcm, crc_ok)`:
    ///
    /// * `crc_ok == true` — the recomputed CRC matched; `pcm` is the
    ///   decoded samples.
    /// * `crc_ok == false` — the CRC did **not** match; `pcm` is a buffer
    ///   of the correct length filled with `0` (spec §5.6: a conformant
    ///   decoder mutes a corrupt block rather than emitting its samples).
    ///
    /// Unlike [`Self::verify_decoded_crc`] (a non-mutating checker), this
    /// is the spec-faithful decode-with-gate: callers that want the
    /// decoder's defined behaviour on a bad block use this and need not
    /// implement the mute themselves. Any error [`Self::decode_samples`]
    /// raises is propagated verbatim.
    pub fn decode_samples_muted(&self) -> Result<(Vec<i32>, bool)> {
        // Spec §1 pipeline / §5.2: the running CRC is computed over the
        // pre-shift samples ("before final shift"). Fold the pre-shift
        // buffer, then apply the left-shift fixup to whatever PCM we emit.
        let mut pcm = self.decode_samples_preshift()?;
        let mut crc_ok = self.crc_of_decoded(&pcm) == self.header.crc();
        if crc_ok {
            // Round 405: run the int32 sample-format fixup and fold its
            // §5.5 extension-CRC verdict into the gate (spec §5.6: the
            // block is muted when *either* CRC fails).
            if let Some((computed, stored)) = self.apply_int32_fixup(&mut pcm)? {
                crc_ok = computed == stored;
            }
        }
        if crc_ok {
            // CRC matched: emit the final container-scaled PCM (apply the
            // wiki bits-13..=17 left-shift normalization the public
            // `decode_samples` would).
            crate::fixup::apply_left_shift_buffer(&mut pcm, self.header.flags.left_shift);
        } else {
            // Spec §5.6: mute (zero) the block on a CRC mismatch. A zeroed
            // buffer is shift-invariant, so no fixup is needed.
            pcm.iter_mut().for_each(|s| *s = 0);
        }
        Ok((pcm, crc_ok))
    }

    /// Borrow the raw `0x02` / `0x03` / `0x04` decorrelation sub-block
    /// payloads (each defaulting to an empty slice when the sub-block is
    /// absent), for feeding the decorrelation-pass assembler. The first
    /// occurrence of each ID wins (the metadata walker preserves wire
    /// order). Round 339.
    fn decorr_payloads(&self) -> (&[u8], &[u8], &[u8]) {
        let pick = |id: SubBlockId| {
            self.find_sub_block(id)
                .map(|sb| sb.payload)
                .unwrap_or(&[][..])
        };
        (
            pick(SubBlockId::DecorrelationTerms),
            pick(SubBlockId::DecorrelationWeights),
            pick(SubBlockId::DecorrelationSamples),
        )
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

    /// Block-level pairing with
    /// [`WavPackBlockHeader::total_samples_in_file`] — the wiki
    /// "total samples in file" field as a typed `Option<u32>`
    /// (`Some(n)` for a known total, `None` for the wiki
    /// [`crate::TOTAL_SAMPLES_UNKNOWN`] sentinel). Round 239.
    pub fn total_samples_in_file(&self) -> Option<u32> {
        self.header.total_samples_in_file()
    }

    /// Block-level pairing with
    /// [`WavPackBlockHeader::end_sample_index`] — the wiki
    /// "offset in samples for current block" + "samples in this block"
    /// sum as a `u64`. The first-sample-after-this-block cursor the
    /// next block's `block_index` should match in a well-formed stream.
    /// Round 239.
    pub fn end_sample_index(&self) -> u64 {
        self.header.end_sample_index()
    }

    /// Block-level pairing with
    /// [`WavPackBlockHeader::samples_remaining_after`] — the count of
    /// samples remaining in the file after this block, when both the
    /// total and the end cursor are well-defined. `None` for unknown
    /// total or for end-past-total malformed combinations. Round 239.
    pub fn samples_remaining_after(&self) -> Option<u64> {
        self.header.samples_remaining_after()
    }

    /// `true` when this block is the final block of a fully-described
    /// `.wv` file — `samples_remaining_after()` is `Some(0)`. Returns
    /// `false` for `None` (unknown total) and for any non-zero remainder
    /// (more audio follows). Round 239.
    pub fn is_final_audio_block_in_file(&self) -> bool {
        matches!(self.samples_remaining_after(), Some(0))
    }

    /// Block-level pairing with [`WavPackBlockHeader::crc`] — the
    /// 32-bit CRC word the wiki "Block structure" listing places at
    /// bytes 28..32 of the fixed block header.
    ///
    /// The wiki names the field but does not specify the polynomial,
    /// the byte span the encoder computed it over, the initial value,
    /// or the byte / bit order of the computation; the staged docs do
    /// not specify those parameters either. This accessor surfaces the
    /// **stored** word verbatim (no recomputation, no verification)
    /// so callers iterating a multi-block stream can pick the
    /// per-block CRC off a borrowed `WavPackBlock` alongside the other
    /// round-214 / round-239 introspection accessors. Round 245.
    pub fn crc(&self) -> u32 {
        self.header.crc()
    }

    /// Block-level pairing with [`WavPackBlockHeader::version`] — the
    /// 16-bit stream-format version the wiki "Block structure" listing
    /// places at bytes 8..10 of the fixed block header. Always in the
    /// [`crate::MIN_VERSION`]`..=`[`crate::MAX_VERSION`] inclusive
    /// window (the parser refuses out-of-window values as
    /// [`Error::UnsupportedVersion`]). Round 252.
    pub fn version(&self) -> u16 {
        self.header.version()
    }

    /// Block-level pairing with [`WavPackBlockHeader::track_number`] —
    /// the 8-bit "track number" byte the wiki "Block structure" listing
    /// places at byte 10 of the fixed block header. Wiki marks the
    /// field "not currently implemented"; preserved verbatim. Round 252.
    pub fn track_number(&self) -> u8 {
        self.header.track_number()
    }

    /// Block-level pairing with
    /// [`WavPackBlockHeader::track_sub_index`] — the 8-bit "track sub
    /// index" byte the wiki "Block structure" listing places at byte
    /// 11 of the fixed block header. Wiki marks the field "not
    /// currently implemented"; preserved verbatim. Round 252.
    pub fn track_sub_index(&self) -> u8 {
        self.header.track_sub_index()
    }

    /// Block-level pairing with [`WavPackBlockHeader::has_track_id`] —
    /// `true` when either of the two wiki "track number" / "track sub
    /// index" bytes is non-zero. Round 252.
    pub fn has_track_id(&self) -> bool {
        self.header.has_track_id()
    }

    /// Block-level pairing with
    /// [`WavPackBlockHeader::supports_false_stereo`] — `true` when
    /// the stream version is at least `0x0410`, i.e. the version gate
    /// the wiki "Flags meaning" listing places on bit 30 "false
    /// stereo … version >= 0x410". Round 252.
    pub fn supports_false_stereo(&self) -> bool {
        self.header.supports_false_stereo()
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

/// Header-only shape of a multichannel WavPack stream: the per-frame
/// channel count and the number of member-block sets, computed by walking
/// block **headers** only (no entropy decode).
///
/// [`multichannel_layout`] returns this so a caller can size buffers and
/// route channels before paying for a full
/// [`decode_multichannel_stream`]. Round 378.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MultichannelLayout {
    /// Per-frame channel count — the summed member channel counts of one
    /// set (every well-formed set carries the same width). `0` for an
    /// audio-free stream.
    pub channels: usize,
    /// Number of complete member-block sets in the stream (one per frame
    /// range). `0` for an audio-free stream.
    pub sets: usize,
}

/// Compute the [`MultichannelLayout`] of a WavPack byte buffer by walking
/// the block headers (and the per-block mono/stereo flag) only — no
/// entropy decode, no per-sample loop.
///
/// Applies the same wiki bits-11..=12 grouping rules
/// [`decode_multichannel_stream`] enforces: a first-marker opens a set, a
/// final-marker closes it, every member of a set must agree on
/// `block_samples`, and every set must carry the same channel width.
/// Metadata-only blocks are skipped. The same malformed-grouping refusals
/// ([`Error::MultichannelSetMalformed`] /
/// [`Error::MultichannelSampleCountMismatch`] /
/// [`Error::MultichannelTooManyChannels`]) fire here too, so a stream that
/// passes `multichannel_layout` is structurally decodable. A plain mono /
/// stereo file reports `channels == 1 / 2` with `sets ==
/// audio_block_count`. Round 378.
pub fn multichannel_layout(bytes: &[u8]) -> Result<MultichannelLayout> {
    let mut stream_channels: Option<usize> = None;
    let mut sets = 0usize;
    // Channels accumulated in the currently-open set, and its agreed frame
    // count. `open` tracks whether a set is currently open.
    let mut open = false;
    let mut open_chan = 0usize;
    let mut open_frames: u32 = 0;

    for parsed in iter_blocks(bytes) {
        let block = parsed?;
        if !block.header.is_audio_block() {
            continue;
        }
        let flags = &block.header.flags;
        let is_first = flags.is_first_block();
        let is_final = flags.is_final_block();
        let block_samples = block.header.block_samples;

        if is_first {
            if open {
                return Err(Error::MultichannelSetMalformed);
            }
            open = true;
            open_chan = 0;
            open_frames = block_samples;
        } else if !open {
            return Err(Error::MultichannelSetMalformed);
        }

        if block_samples != open_frames {
            return Err(Error::MultichannelSampleCountMismatch {
                expected: open_frames,
                found: block_samples,
            });
        }

        open_chan += block.member_channel_count();
        if open_chan > MAX_MULTICHANNEL_CHANNELS {
            return Err(Error::MultichannelTooManyChannels(open_chan));
        }

        if is_final {
            match stream_channels {
                None => stream_channels = Some(open_chan),
                Some(prev) if prev != open_chan => {
                    return Err(Error::MultichannelSetMalformed);
                }
                Some(_) => {}
            }
            sets += 1;
            open = false;
        }
    }

    if open {
        return Err(Error::MultichannelSetMalformed);
    }

    Ok(MultichannelLayout {
        channels: stream_channels.unwrap_or(0),
        sets,
    })
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

/// Read the wiki "total samples in file" field from the **first**
/// block's header in `bytes`, lifted into a typed `Option<u32>`.
///
/// The wiki "Block structure" listing names `total_samples` as a
/// file-global quantity — every block of a well-formed `.wv` file
/// carries the same value, so the first block's copy is the
/// stream-level total. Returns:
///
/// * `Ok(Some(Some(n)))` — the first block declares a known total of
///   `n` samples;
/// * `Ok(Some(None))` — the first block carries the wiki
///   [`crate::TOTAL_SAMPLES_UNKNOWN`] sentinel (`0xFFFFFFFF`,
///   documented as "may be 0xFFFFFFFF if unknown") — a streaming
///   encoder that couldn't predict the total at write time emits this;
/// * `Ok(None)` — the input is empty, so there is no first block to
///   read;
/// * `Err(_)` — the first block's header could not be parsed (the
///   round-1 [`parse_block_header`] errors surface verbatim).
///
/// Only the 32-byte fixed header is read — no metadata walk, no
/// per-block on-disk length validation, so this is constant-time
/// regardless of stream size. Round 239.
pub fn stream_total_samples(bytes: &[u8]) -> Result<Option<Option<u32>>> {
    if bytes.is_empty() {
        return Ok(None);
    }
    let (header, _) = parse_block_header(bytes)?;
    Ok(Some(header.total_samples_in_file()))
}

/// The stream's sample rate in Hz, resolved from its first audio
/// block: the standard-rate table for header indices `0..=14`, or the
/// `0x27` non-standard-sampling-rate sub-block (emitted once for the
/// stream, with the first block) for the custom sentinel `15` (staged
/// spec `wavpack-sample-formats.md` §5).
///
/// Returns `Ok(None)` when the stream has no audio blocks, or when the
/// index is the custom sentinel and the first audio block carries no
/// `0x27` sub-block (rate genuinely unknown). Parse errors and a
/// malformed `0x27` payload ([`Error::SampleRatePayloadLength`])
/// surface verbatim. Round 405.
pub fn stream_sample_rate(bytes: &[u8]) -> Result<Option<u32>> {
    match first_audio_block(bytes)? {
        Some(block) => block.sample_rate(),
        None => Ok(None),
    }
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

/// Pair the audio blocks of a main `.wv` byte buffer with the audio
/// blocks of its companion `.wvc` correction-file buffer — the
/// stream-level plumbing that aligns the two files a hybrid encode
/// splits its output across (spec §4.1: the `0x0B` correction stream is
/// "normally in the companion `.wvc` file").
///
/// Both buffers are walked as ordinary `wvpk` block chains (a `.wvc`
/// file uses the identical container per the wiki "Block structure"
/// section — only the sub-block inventory differs). Audio blocks
/// (`block_samples > 0`) are aligned **by the wiki "offset in samples
/// for current block" header word**, in order:
///
/// - a main block whose `block_index` the correction chain does not
///   carry pairs with `None` (a correction file may cover only part of
///   the stream — pure-lossless blocks need no correction);
/// - an aligned pair must agree on `block_samples`
///   ([`Error::CorrectionSampleCountMismatch`]) and on the mono flag
///   ([`Error::CorrectionShapeMismatch`]) — the correction words pair
///   one-to-one with the main stream's channel samples;
/// - a correction audio block whose `block_index` is behind the next
///   main block is an orphan ([`Error::CorrectionIndexMismatch`]);
/// - correction audio blocks past the last main block are surplus
///   ([`Error::CorrectionBlockSurplus`]).
///
/// Metadata-only blocks (`block_samples == 0`) on either side are
/// skipped — they never carry sample words to pair. Parse errors from
/// either chain surface verbatim. Note this is **structural** pairing
/// only: consuming a pair's `0x0B` words in a hybrid decode stays
/// gated on the hybrid entropy derivation (the
/// [`crate::UnsupportedBlockFeature::Hybrid`] refusal), so today's
/// callers use this to locate / validate / size correction coverage
/// on the lossless path.
pub fn pair_correction_stream<'a, 'b>(
    main: &'a [u8],
    correction: &'b [u8],
) -> Result<Vec<(WavPackBlock<'a>, Option<WavPackBlock<'b>>)>> {
    let mut pairs: Vec<(WavPackBlock<'a>, Option<WavPackBlock<'b>>)> = Vec::new();
    let mut wvc = iter_audio_blocks(correction);
    let mut pending: Option<WavPackBlock<'b>> = None;
    for block in iter_audio_blocks(main) {
        let block = block?;
        // Fetch the next unconsumed correction audio block (if any).
        if pending.is_none() {
            pending = match wvc.next() {
                Some(Ok(c)) => Some(c),
                Some(Err(e)) => return Err(e),
                None => None,
            };
        }
        let paired = match pending.as_ref() {
            Some(c) if c.block_index() == block.block_index() => {
                if c.block_samples() != block.block_samples() {
                    return Err(Error::CorrectionSampleCountMismatch {
                        main: block.block_samples(),
                        correction: c.block_samples(),
                    });
                }
                if c.flags().mono != block.flags().mono {
                    return Err(Error::CorrectionShapeMismatch(block.block_index()));
                }
                pending.take()
            }
            Some(c) if c.block_index() < block.block_index() => {
                return Err(Error::CorrectionIndexMismatch {
                    main: block.block_index(),
                    correction: c.block_index(),
                });
            }
            // Correction chain is ahead (or exhausted): this main block
            // has no correction twin.
            _ => None,
        };
        pairs.push((block, paired));
    }
    // Anything left on the correction side has no main twin.
    if let Some(c) = pending {
        return Err(Error::CorrectionBlockSurplus(c.block_index()));
    }
    match wvc.next() {
        Some(Ok(c)) => Err(Error::CorrectionBlockSurplus(c.block_index())),
        Some(Err(e)) => Err(e),
        None => Ok(pairs),
    }
}

/// Count how many of a main `.wv` buffer's audio blocks have an
/// index-aligned correction twin in the companion `.wvc` buffer — the
/// coverage summary of [`pair_correction_stream`]. Returns
/// `(paired, total_main_audio_blocks)`; a full hybrid-lossless file
/// reports `paired == total`.
pub fn correction_coverage(main: &[u8], correction: &[u8]) -> Result<(usize, usize)> {
    let pairs = pair_correction_stream(main, correction)?;
    let total = pairs.len();
    let paired = pairs.iter().filter(|(_, c)| c.is_some()).count();
    Ok((paired, total))
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

/// Decode every audio block in a WavPack byte buffer with the spec §5.6
/// per-block CRC mute gate applied, concatenating the PCM.
///
/// This is the CRC-validating twin of [`decode_stream`]: each audio
/// block is decoded through [`WavPackBlock::decode_samples_muted`], so a
/// block whose recomputed running CRC does not match its stored header
/// CRC word contributes a run of zeros (a muted block) instead of its
/// samples — exactly what a conformant decoder emits for a corrupt block
/// rather than aborting the whole stream.
///
/// Returns `(pcm, all_crc_ok)`:
///
/// * `pcm` — the concatenated decoded (and per-block CRC-gated) samples,
///   in on-disk order, with the same per-block shape [`decode_stream`]
///   produces (mono = `block_samples` `i32`s, stereo = `block_samples *
///   2` interleaved).
/// * `all_crc_ok` — `true` only when *every* decoded block's CRC matched;
///   `false` if any block was muted.
///
/// Parse and decode (unsupported-feature / malformed-payload) errors are
/// still surfaced verbatim from the first failing block — a CRC mismatch
/// is *not* an error (it is the defined mute behaviour), but a structural
/// failure is. Metadata-only blocks contribute nothing and do not affect
/// `all_crc_ok`.
pub fn decode_stream_muted(bytes: &[u8]) -> Result<(Vec<i32>, bool)> {
    let mut out: Vec<i32> = Vec::new();
    let mut all_crc_ok = true;
    for parsed in iter_blocks(bytes) {
        let block = parsed?;
        // Skip metadata-only / zero-sample blocks — they carry no PCM and
        // have no CRC to gate (decode_samples would raise BlockHasNoAudio).
        if !block.header.is_audio_block() {
            continue;
        }
        let (pcm, crc_ok) = block.decode_samples_muted()?;
        all_crc_ok &= crc_ok;
        out.extend_from_slice(&pcm);
    }
    Ok((out, all_crc_ok))
}

/// Decoded shape of a WavPack stream: the interleaved PCM plus the
/// channel count of one interleaved frame.
///
/// [`decode_multichannel_stream`] returns this so a caller knows how to
/// de-interleave the flat `samples` buffer: it holds whole frames of
/// `channels` `i32`s each, in speaker order (the wiki bits-11..=12 member
/// order — first-block member's channels first, then each continuation
/// member, then the final-block member). For a plain mono file
/// `channels == 1`; for plain stereo `channels == 2`; for a multichannel
/// file `channels` is the per-frame sum across one set's members.
/// Round 378.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodedStream {
    /// Interleaved PCM, `samples.len() == frames * channels`.
    pub samples: Vec<i32>,
    /// Number of channels in one interleaved frame.
    pub channels: usize,
}

/// Decode a WavPack byte buffer that may carry **multichannel** audio
/// (more than two channels grouped across member blocks) into a single
/// interleaved [`DecodedStream`].
///
/// A multichannel WavPack file does not pack all channels into one block.
/// Instead each frame range is split across a *set* of member blocks: the
/// wiki bit-11 ("first block of multi-channel set") member opens the set,
/// zero or more continuation members (neither marker) follow, and the
/// wiki bit-12 ("last block of set") member closes it. Each member is an
/// ordinary 1-channel (mono / false-stereo) or 2-channel (stereo) block
/// and is decoded by the same lossless path standalone blocks use
/// ([`WavPackBlock::decode_member_samples`]) — the grouping marker is a
/// stream-shape signal, not a decode-arithmetic one. The set's channels
/// are then the members' channels concatenated in member order, and the
/// set's PCM is those channels interleaved per frame.
///
/// Shape contract:
///
/// * **Plain mono / stereo file** — every block is a standalone set
///   (marker `0b11`); the result is identical to [`decode_stream`] with
///   `channels` reported as `1` or `2`. (A file that mixes block shapes
///   across sets keeps the first set's per-frame channel count as the
///   stream's `channels`; see the mismatch refusal below.)
/// * **Multichannel file** — `channels` is the summed per-frame channel
///   count of one set's members, and `samples` holds whole interleaved
///   frames.
///
/// Members of one set must agree on `block_samples` (they are the same
/// frames' channels) — a mismatch raises
/// [`Error::MultichannelSampleCountMismatch`]. A stray bit-12 marker with
/// no open set, or a stream that ends mid-set, raises
/// [`Error::MultichannelSetMalformed`]. A set whose summed channel count
/// exceeds [`MAX_MULTICHANNEL_CHANNELS`] raises
/// [`Error::MultichannelTooManyChannels`]. Per-member decode and parse
/// errors propagate verbatim. Metadata-only blocks (`block_samples == 0`)
/// are skipped and do not participate in a set.
///
/// All sets in a well-formed file carry the same per-frame channel count;
/// this function reports the **first** set's count as the stream
/// `channels` and concatenates every set's interleaved frames. Round 378.
pub fn decode_multichannel_stream(bytes: &[u8]) -> Result<DecodedStream> {
    let (stream, _all_crc_ok) = decode_multichannel_inner(bytes, false)?;
    Ok(stream)
}

/// Decode a multichannel WavPack byte buffer with the spec §5.6 per-member
/// CRC *mute gate* applied — the multichannel twin of
/// [`decode_stream_muted`].
///
/// Identical to [`decode_multichannel_stream`] except each member block is
/// decoded through [`WavPackBlock::decode_member_samples_muted`]: a member
/// whose recomputed §5 running CRC does not match its stored header CRC
/// word contributes a run of zeros (its channels muted) instead of its
/// samples — exactly what a conformant decoder emits for a corrupt block.
/// The set's other (uncorrupted) members still contribute their channels,
/// so the interleaved frame width is unchanged; only the muted member's
/// channel slots are zero.
///
/// Returns `(stream, all_crc_ok)`: `all_crc_ok` is `true` only when every
/// decoded member's CRC matched. The grouping-shape refusals
/// ([`Error::MultichannelSetMalformed`] /
/// [`Error::MultichannelSampleCountMismatch`] /
/// [`Error::MultichannelTooManyChannels`]) and parse / structural errors
/// still propagate — a CRC mismatch is the defined mute behaviour, not an
/// error. Round 378.
pub fn decode_multichannel_stream_muted(bytes: &[u8]) -> Result<(DecodedStream, bool)> {
    decode_multichannel_inner(bytes, true)
}

/// Shared grouping-walk core for [`decode_multichannel_stream`] (plain,
/// `muted == false`) and [`decode_multichannel_stream_muted`]
/// (`muted == true`). Walks the wiki bits-11..=12 member sets, decodes
/// each member's channels, and interleaves each closed set's channels into
/// frames. Returns the assembled stream plus whether every member's §5 CRC
/// matched (always `true` in the non-muted mode, which does not fold CRCs).
fn decode_multichannel_inner(bytes: &[u8], muted: bool) -> Result<(DecodedStream, bool)> {
    // Per-set accumulator: the decoded per-member channel buffers of the
    // currently-open set, plus the set's agreed frame count.
    //
    // Each member contributes its own `1` or `2` channel buffers; a set's
    // interleaved output reads frame `f` as
    // `[ch0[f], ch1[f], …, chN[f]]` across the members in wire order.
    let mut out: Vec<i32> = Vec::new();
    let mut stream_channels: Option<usize> = None;
    let mut all_crc_ok = true;

    // The channels of the currently-open set, each as a flat per-channel
    // buffer of `frame_count` samples. `None` => no set is open.
    let mut open_channels: Option<Vec<Vec<i32>>> = None;
    let mut open_frames: u32 = 0;

    for parsed in iter_blocks(bytes) {
        let block = parsed?;
        if !block.header.is_audio_block() {
            // Metadata-only block: no PCM, not a set member. Per the wiki
            // "Block structure" allowance for block_samples == 0.
            continue;
        }
        let flags = &block.header.flags;
        let is_first = flags.is_first_block();
        let is_final = flags.is_final_block();
        let block_samples = block.header.block_samples;

        // A bit-11 member opens a fresh set. If a set is already open
        // when a first-marker arrives, the previous set never saw its
        // final marker — malformed.
        if is_first {
            if open_channels.is_some() {
                return Err(Error::MultichannelSetMalformed);
            }
            open_channels = Some(Vec::new());
            open_frames = block_samples;
        } else if open_channels.is_none() {
            // A continuation- or final-marker member with no open set is a
            // stray grouping marker.
            return Err(Error::MultichannelSetMalformed);
        }

        // All members of one set cover the same frame range.
        if block_samples != open_frames {
            return Err(Error::MultichannelSampleCountMismatch {
                expected: open_frames,
                found: block_samples,
            });
        }

        // Decode this member's own 1 or 2 channels (the grouping marker is
        // accepted, not refused), then split the interleaved buffer into
        // per-channel flat buffers and append them to the open set.
        let member_channels = block.member_channel_count();
        let pcm = if muted {
            let (pcm, crc_ok) = block.decode_member_samples_muted()?;
            all_crc_ok &= crc_ok;
            pcm
        } else {
            block.decode_member_samples()?
        };
        let set = open_channels
            .as_mut()
            .expect("a set is open here (opened above or pre-existing)");
        // Channel-cap guard: refuse before sizing the per-channel buffers.
        if set.len() + member_channels > MAX_MULTICHANNEL_CHANNELS {
            return Err(Error::MultichannelTooManyChannels(
                set.len() + member_channels,
            ));
        }
        if member_channels == 1 {
            set.push(pcm);
        } else {
            // Interleaved [L0, R0, L1, R1, …] → two flat per-channel
            // buffers.
            let frames = pcm.len() / 2;
            let mut left = Vec::with_capacity(frames);
            let mut right = Vec::with_capacity(frames);
            for frame in pcm.chunks_exact(2) {
                left.push(frame[0]);
                right.push(frame[1]);
            }
            set.push(left);
            set.push(right);
        }

        // A bit-12 member closes the set: interleave its channels into
        // whole frames and append to the stream output.
        if is_final {
            let set = open_channels.take().expect("set open at final marker");
            let channels = set.len();
            // Establish (or check) the stream's per-frame channel count.
            match stream_channels {
                None => stream_channels = Some(channels),
                Some(prev) if prev != channels => {
                    // Sets disagree on channel count — treat as a malformed
                    // grouping rather than silently emitting ragged frames.
                    return Err(Error::MultichannelSetMalformed);
                }
                Some(_) => {}
            }
            let frames = open_frames as usize;
            out.reserve(frames * channels);
            for f in 0..frames {
                for ch in &set {
                    // Each per-channel buffer has exactly `frames` samples
                    // (the members agreed on `block_samples` above).
                    out.push(ch[f]);
                }
            }
            open_frames = 0;
        }
    }

    // A set left open at end-of-stream never saw its final marker.
    if open_channels.is_some() {
        return Err(Error::MultichannelSetMalformed);
    }

    Ok((
        DecodedStream {
            samples: out,
            // An empty stream (no audio blocks) reports 0 channels.
            channels: stream_channels.unwrap_or(0),
        },
        all_crc_ok,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block_header::{HEADER_LEN, MAGIC, MIN_CK_SIZE, TOTAL_SAMPLES_UNKNOWN};

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

    /// Append a 0x05 entropy-info sub-block carrying a small non-zero
    /// stereo seed. The non-zero left-median value is what
    /// `EntropyInfo::is_mono()` (a content-only predicate) inspects to
    /// report `false`, so the stereo decode path is taken.
    ///
    /// Each 16-bit median word is a little-endian signed log word
    /// expanded by `crate::wp_exp2s` (round 405): `0x0901` expands to
    /// `median = 257` on both channels, so neither channel is eligible
    /// for the spec §4.2 step 1 zero-run fast path (`median[0] > 1`).
    /// Encoder-side tests seed their `AdaptiveMedians` through the same
    /// expansion, so the two directions stay in lockstep.
    fn append_entropy_info_stereo_minimal(payload: &mut Vec<u8>) {
        let mut bytes = [0u8; 12];
        bytes[0] = 0x01;
        bytes[1] = 0x09; // left medians[0] = wp_exp2s(0x0901) = 257
        bytes[6] = 0x01;
        bytes[7] = 0x09; // right medians[0] = 257
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
    fn decode_samples_mono_decorrelation_reconstructs_pcm_end_to_end() {
        use crate::decorrelation::{
            assemble_mono_passes, decorrelate_mono, TERM_BYTE_BIAS, TERM_PREDICTOR_BITS,
            TERM_PREDICTOR_MASK,
        };
        use crate::samples::{encode_packed_samples_mono, AdaptiveMedians};

        // Build a residual buffer, encode it into the 0x0A bitstream
        // (seeded from the 0x05 entropy info), attach a 0x02/0x03/0x04
        // decorrelation config, and confirm the full decode path
        // reconstructs the same PCM the standalone `decorrelate_mono`
        // produces from those residuals. This pins the entropy → residual
        // → decorrelation → PCM wiring inside `decode_samples`.
        let residuals: Vec<i32> = vec![5, -3, 8, 0, 12, -7, 4, 1, -2, 9, 6, -1];

        // Application order: term 1 (delta 2) then term 17 (delta 3).
        let term_byte = |term: i8, delta: u8| -> u8 {
            (((term + TERM_BYTE_BIAS) as u8) & TERM_PREDICTOR_MASK) | (delta << TERM_PREDICTOR_BITS)
        };
        let seed_word = |v: i32| -> [u8; 2] { [v as i8 as u8, 9] };

        // Wire order = reverse application order: [term 17, term 1].
        let app = [(1i8, 2u8, 5u8, vec![4]), (17i8, 3u8, 10u8, vec![6, 5])];
        let mut terms_payload = Vec::new();
        let mut weights_payload = Vec::new();
        let mut samples_payload = Vec::new();
        for (t, d, wbyte, seeds) in app.iter().rev() {
            terms_payload.push(term_byte(*t, *d));
            weights_payload.push(*wbyte);
            for &s in seeds {
                samples_payload.extend_from_slice(&seed_word(s));
            }
        }

        // Expected PCM: decorrelate the residuals with the same assembled
        // passes (the assembler is independently round-trip-tested).
        let mut expected_passes =
            assemble_mono_passes(&terms_payload, &weights_payload, &samples_payload)
                .expect("assemble passes");
        let mut expected = residuals.clone();
        decorrelate_mono(&mut expected_passes, &mut expected).expect("decorrelate");

        // Encode the residuals into a 0x0A payload using medians seeded
        // from the same 0x05 info the decoder will read (the mono-zero
        // helper writes a six-byte all-zero 0x05 payload → medians 0,0,0).
        let info_block = crate::entropy::expand_entropy(&[0u8; 6]).expect("expand entropy info");
        let mut enc_medians =
            AdaptiveMedians::from_entropy(&info_block, 0).expect("seed encoder medians");
        let packed_0a =
            encode_packed_samples_mono(&residuals, &mut enc_medians).expect("encode residuals");

        // Assemble the full block payload.
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x02, &terms_payload);
        append_small_sub_block(&mut payload, 0x03, &weights_payload);
        append_small_sub_block(&mut payload, 0x04, &samples_payload);
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &packed_0a);

        let flags = flags_with(1 << 2); // mono
        let bytes = synthesise_block(residuals.len() as u32, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_decorrelation());
        let got = block.decode_samples().expect("decode mono decorrelation");
        assert_eq!(got, expected, "mono decorrelation decode mismatch");
        // Sanity: the decorrelation actually changed the samples (the
        // path is not a silent identity pass-through).
        assert_ne!(got, residuals);
    }

    // ---- left-shift final-normalization fixup wiring (round 354) -------

    /// Build a known-PCM mono decorrelation block plus the unshifted PCM
    /// the decode produces, returning `(block_bytes, unshifted_pcm)`. The
    /// caller stamps `left_shift` into the flag word (bits 13..=17) to
    /// exercise the final normalization stage. Reuses the residual/term
    /// layout the end-to-end decorrelation test pins.
    fn synthesise_mono_decorr_block_with_left_shift(left_shift: u32) -> (Vec<u8>, Vec<i32>) {
        use crate::decorrelation::{
            assemble_mono_passes, decorrelate_mono, TERM_BYTE_BIAS, TERM_PREDICTOR_BITS,
            TERM_PREDICTOR_MASK,
        };
        use crate::samples::{encode_packed_samples_mono, AdaptiveMedians};

        let residuals: Vec<i32> = vec![5, -3, 8, 0, 12, -7, 4, 1, -2, 9, 6, -1];
        let term_byte = |term: i8, delta: u8| -> u8 {
            (((term + TERM_BYTE_BIAS) as u8) & TERM_PREDICTOR_MASK) | (delta << TERM_PREDICTOR_BITS)
        };
        let seed_word = |v: i32| -> [u8; 2] { [v as i8 as u8, 9] };
        let app = [(1i8, 2u8, 5u8, vec![4]), (17i8, 3u8, 10u8, vec![6, 5])];
        let mut terms_payload = Vec::new();
        let mut weights_payload = Vec::new();
        let mut samples_payload = Vec::new();
        for (t, d, wbyte, seeds) in app.iter().rev() {
            terms_payload.push(term_byte(*t, *d));
            weights_payload.push(*wbyte);
            for &s in seeds {
                samples_payload.extend_from_slice(&seed_word(s));
            }
        }

        // The unshifted PCM the prediction loop reconstructs.
        let mut passes = assemble_mono_passes(&terms_payload, &weights_payload, &samples_payload)
            .expect("assemble passes");
        let mut unshifted = residuals.clone();
        decorrelate_mono(&mut passes, &mut unshifted).expect("decorrelate");

        let info_block = crate::entropy::expand_entropy(&[0u8; 6]).expect("expand entropy info");
        let mut enc_medians =
            AdaptiveMedians::from_entropy(&info_block, 0).expect("seed encoder medians");
        let packed_0a =
            encode_packed_samples_mono(&residuals, &mut enc_medians).expect("encode residuals");

        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x02, &terms_payload);
        append_small_sub_block(&mut payload, 0x03, &weights_payload);
        append_small_sub_block(&mut payload, 0x04, &samples_payload);
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &packed_0a);

        let flags = flags_with((1 << 2) | (left_shift << 13)); // mono + left_shift
        let bytes = synthesise_block(residuals.len() as u32, flags, &payload);
        (bytes, unshifted)
    }

    #[test]
    fn decode_samples_applies_left_shift_to_reconstructed_pcm() {
        // A non-zero left_shift (bits 13..=17) must scale every decoded
        // sample left by that count — the wiki sub-byte-depth fixup.
        let shift = 4u32;
        let (bytes, unshifted) = synthesise_mono_decorr_block_with_left_shift(shift);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.header.flags.left_shift, shift as u8);

        let got = block.decode_samples().expect("decode with left shift");
        let expected: Vec<i32> = unshifted.iter().map(|&s| s << shift).collect();
        assert_eq!(got, expected, "left-shift fixup not applied");
        // The shift actually changed the buffer (non-trivial fixup).
        assert_ne!(got, unshifted);
    }

    #[test]
    fn decode_samples_zero_left_shift_is_identity() {
        // left_shift == 0 (whole-byte depth) must leave the reconstructed
        // PCM untouched — the fixup is the identity.
        let (bytes, unshifted) = synthesise_mono_decorr_block_with_left_shift(0);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.header.flags.left_shift, 0);
        let got = block.decode_samples().expect("decode no shift");
        assert_eq!(got, unshifted);
    }

    #[test]
    fn verify_decoded_crc_folds_pre_shift_samples() {
        // The §5 block CRC is computed over the *pre-shift* samples
        // (spec §1 pipeline / §5.2 "before final shift"). Stamp the
        // pre-shift CRC into the header and confirm verify reports a match
        // even though the emitted PCM is shifted.
        let shift = 3u32;
        let (mut bytes, unshifted) = synthesise_mono_decorr_block_with_left_shift(shift);
        let pre_shift_crc = crate::crc::crc_mono(&unshifted);
        bytes[28..32].copy_from_slice(&pre_shift_crc.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.crc(), pre_shift_crc);
        assert!(
            block.verify_decoded_crc().expect("verify"),
            "CRC must be folded over pre-shift samples"
        );

        // And the publicly-decoded PCM is the shifted form.
        let got = block.decode_samples().expect("decode");
        let expected: Vec<i32> = unshifted.iter().map(|&s| s << shift).collect();
        assert_eq!(got, expected);
    }

    #[test]
    fn verify_decoded_crc_fails_if_post_shift_crc_stamped() {
        // Stamping the *post-shift* CRC must NOT verify — proving the
        // checker folds pre-shift, not the emitted PCM.
        let shift = 3u32;
        let (mut bytes, unshifted) = synthesise_mono_decorr_block_with_left_shift(shift);
        let shifted: Vec<i32> = unshifted.iter().map(|&s| s << shift).collect();
        let post_shift_crc = crate::crc::crc_mono(&shifted);
        // Only meaningful if the two CRCs actually differ.
        assert_ne!(post_shift_crc, crate::crc::crc_mono(&unshifted));
        bytes[28..32].copy_from_slice(&post_shift_crc.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(
            !block.verify_decoded_crc().expect("verify"),
            "post-shift CRC must not match the pre-shift fold"
        );
    }

    #[test]
    fn decode_samples_muted_applies_left_shift_on_crc_match() {
        // The mute gate folds the pre-shift CRC; on a match it must emit
        // the shifted PCM (the same the public decode_samples returns).
        let shift = 2u32;
        let (mut bytes, unshifted) = synthesise_mono_decorr_block_with_left_shift(shift);
        let pre_shift_crc = crate::crc::crc_mono(&unshifted);
        bytes[28..32].copy_from_slice(&pre_shift_crc.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        let (pcm, crc_ok) = block.decode_samples_muted().expect("muted decode");
        assert!(crc_ok, "pre-shift CRC must match");
        let expected: Vec<i32> = unshifted.iter().map(|&s| s << shift).collect();
        assert_eq!(pcm, expected, "muted decode must emit shifted PCM on match");
    }

    #[test]
    fn decode_samples_muted_zeros_block_on_mismatch_regardless_of_shift() {
        // On a CRC mismatch the block is muted (zeroed); a zeroed buffer is
        // shift-invariant, so the output is all zeros of the right length.
        let shift = 5u32;
        let (mut bytes, unshifted) = synthesise_mono_decorr_block_with_left_shift(shift);
        let wrong = crate::crc::crc_mono(&unshifted).wrapping_add(1);
        bytes[28..32].copy_from_slice(&wrong.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        let (pcm, crc_ok) = block.decode_samples_muted().expect("muted decode");
        assert!(!crc_ok);
        assert_eq!(pcm, vec![0; unshifted.len()]);
    }

    #[test]
    fn verify_decoded_crc_matches_when_header_crc_is_correct() {
        // Decode the canonical one-zero mono block, compute its true §5
        // mono CRC, stamp it into the header CRC field (bytes 16..20), and
        // confirm verify_decoded_crc reports a match.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with(1 << 2); // mono
        let mut bytes = synthesise_block(1, flags, &payload);

        // PCM is [0]; its §5.2 mono CRC seeds 0xffffffff and folds 0.
        let expected_crc = crate::crc::crc_mono(&[0]);
        bytes[28..32].copy_from_slice(&expected_crc.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.crc(), expected_crc);
        assert!(block.verify_decoded_crc().expect("verify"));
    }

    #[test]
    fn verify_decoded_crc_fails_when_header_crc_is_wrong() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with(1 << 2); // mono
        let mut bytes = synthesise_block(1, flags, &payload);

        let wrong = crate::crc::crc_mono(&[0]).wrapping_add(1);
        bytes[28..32].copy_from_slice(&wrong.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.verify_decoded_crc().expect("verify"));
    }

    #[test]
    fn decode_samples_muted_returns_pcm_when_crc_matches() {
        // Canonical one-zero mono block with a correct stored CRC: the
        // gate keeps the decoded samples and reports crc_ok = true.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with(1 << 2); // mono
        let mut bytes = synthesise_block(1, flags, &payload);
        let expected_crc = crate::crc::crc_mono(&[0]);
        bytes[28..32].copy_from_slice(&expected_crc.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        let (pcm, crc_ok) = block.decode_samples_muted().expect("decode");
        assert!(crc_ok, "matching CRC should report ok");
        assert_eq!(pcm, vec![0]);
    }

    #[test]
    fn decode_samples_muted_zeros_pcm_on_crc_mismatch() {
        // A non-zero mono PCM block with a deliberately-wrong stored CRC:
        // the §5.6 mute gate zeros the buffer and reports crc_ok = false,
        // while preserving the decoded length.
        use crate::samples::{encode_packed_samples_mono, AdaptiveMedians};

        let pcm = vec![5i32, -3, 8, 0];
        let info_block = crate::entropy::expand_entropy(&[0u8; 6]).expect("entropy");
        let mut enc_medians = AdaptiveMedians::from_entropy(&info_block, 0).expect("medians");
        let packed = encode_packed_samples_mono(&pcm, &mut enc_medians).expect("encode");

        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &packed);
        let flags = flags_with(1 << 2); // mono
        let mut bytes = synthesise_block(pcm.len() as u32, flags, &payload);
        // Stamp a CRC that is guaranteed NOT to match.
        let wrong = crate::crc::crc_mono(&pcm).wrapping_add(1);
        bytes[28..32].copy_from_slice(&wrong.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        // Sanity: the plain decode reproduces the true PCM.
        assert_eq!(block.decode_samples().expect("decode"), pcm);
        let (muted, crc_ok) = block.decode_samples_muted().expect("decode muted");
        assert!(!crc_ok, "wrong CRC should report not-ok");
        assert_eq!(
            muted,
            vec![0; pcm.len()],
            "mismatch must mute (zero) the block"
        );
    }

    #[test]
    fn decode_samples_muted_propagates_decode_errors() {
        // A hybrid block is refused by decode_samples; the muted gate
        // surfaces the same typed error rather than a (pcm, bool) pair.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with((1 << 2) | (1 << 3)); // mono + hybrid
        let bytes = synthesise_block(1, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(
            block.decode_samples_muted(),
            Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::Hybrid
            ))
        );
    }

    #[test]
    fn verify_decoded_crc_propagates_decode_errors() {
        // A hybrid block is refused by decode_samples; verify_decoded_crc
        // surfaces the same typed error rather than a CRC result.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with((1 << 2) | (1 << 3)); // mono + hybrid
        let bytes = synthesise_block(1, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(
            block.verify_decoded_crc(),
            Err(Error::UnsupportedBlockFeature(
                UnsupportedBlockFeature::Hybrid
            ))
        );
    }

    #[test]
    fn verify_decoded_crc_matches_for_mono_decorrelation_block() {
        use crate::decorrelation::{
            assemble_mono_passes, decorrelate_mono, TERM_BYTE_BIAS, TERM_PREDICTOR_BITS,
            TERM_PREDICTOR_MASK,
        };
        use crate::samples::{encode_packed_samples_mono, AdaptiveMedians};

        // Reconstruct PCM from a decorrelation block (as the end-to-end
        // decode test does), compute its §5 CRC over the reconstructed
        // samples, stamp it, and confirm verify_decoded_crc agrees — the
        // CRC is folded over the post-decorrelation PCM, not the residuals.
        let residuals: Vec<i32> = vec![5, -3, 8, 0, 12, -7, 4, 1];
        let term_byte = |term: i8, delta: u8| -> u8 {
            (((term + TERM_BYTE_BIAS) as u8) & TERM_PREDICTOR_MASK) | (delta << TERM_PREDICTOR_BITS)
        };
        let seed_word = |v: i32| -> [u8; 2] { [v as i8 as u8, 9] };
        // Two passes (even-length 0x02/0x03 payloads). Wire order is
        // reverse application order: store [term 2, term 1].
        let terms_payload = vec![term_byte(2, 1), term_byte(1, 2)];
        let weights_payload = vec![6u8, 4u8];
        let mut samples_payload = Vec::new();
        // term 2 needs 2 seeds (wire-first), term 1 needs 1 seed.
        for &s in &[3, 4, 7] {
            samples_payload.extend_from_slice(&seed_word(s));
        }

        let mut passes = assemble_mono_passes(&terms_payload, &weights_payload, &samples_payload)
            .expect("assemble");
        let mut expected_pcm = residuals.clone();
        decorrelate_mono(&mut passes, &mut expected_pcm).expect("decorrelate");
        let expected_crc = crate::crc::crc_mono(&expected_pcm);

        let info_block = crate::entropy::expand_entropy(&[0u8; 6]).expect("entropy");
        let mut enc_medians = AdaptiveMedians::from_entropy(&info_block, 0).expect("medians");
        let packed_0a = encode_packed_samples_mono(&residuals, &mut enc_medians).expect("encode");

        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x02, &terms_payload);
        append_small_sub_block(&mut payload, 0x03, &weights_payload);
        append_small_sub_block(&mut payload, 0x04, &samples_payload);
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &packed_0a);

        let flags = flags_with(1 << 2); // mono
        let mut bytes = synthesise_block(residuals.len() as u32, flags, &payload);
        bytes[28..32].copy_from_slice(&expected_crc.to_le_bytes());

        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.verify_decoded_crc().expect("verify decorr crc"));
    }

    #[test]
    fn decode_samples_rejects_absurd_block_samples_without_oom() {
        // Regression (round 296): the spec §4.2 step 1 zero-run path lets
        // a tiny 0x0A payload's `block_samples` field expand without a
        // payload-byte bound. A block claiming a near-`u32::MAX`
        // `block_samples` while carrying a 2-byte payload must surface a
        // typed `BlockSamplesTooLarge` rejection rather than attempting a
        // multi-gigabyte `Vec` for the per-sample loop.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0xFF, 0xFF]);
        let flags = flags_with(1 << 2); // mono
        let bytes = synthesise_block(0x2100_0001, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples();
        assert_eq!(err, Err(Error::BlockSamplesTooLarge(0x2100_0001)));
    }

    /// Append a `0x0A` packed-samples sub-block whose walker-returned
    /// payload is ODD-length, using the wiki `0x40` odd-size framing flag:
    /// the on-disk size field counts `(odd_payload.len() + 1) / 2` words
    /// and the encoder appends one trailing pad byte the round-2 walker
    /// strips, so the walker hands `decode_samples` an odd payload.
    fn append_packed_samples_odd(payload: &mut Vec<u8>, odd_payload: &[u8]) {
        assert!(odd_payload.len() % 2 == 1, "helper expects an odd payload");
        let words = odd_payload.len().div_ceil(2) as u8;
        payload.push(0x0A | crate::ID_FLAG_ODD_SIZE);
        payload.push(words);
        payload.extend_from_slice(odd_payload);
        payload.push(0x00); // padding byte the odd-size flag accounts for
    }

    #[test]
    fn decode_samples_rejects_odd_length_packed_samples_payload() {
        // Spec §1: the 0x0A main-bitstream payload byte length must be
        // even or the block is rejected. A walker-stripped odd payload
        // (one real byte, framed via the 0x40 odd-size flag) must surface
        // a typed `PackedSamplesOddLength` rejection — distinct from the
        // framing pad, which the walker already removed.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples_odd(&mut payload, &[0x00]);
        let flags = flags_with(1 << 2); // mono
        let bytes = synthesise_block(1, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        // The walker should hand back a 1-byte (odd) 0x0A payload.
        let packed = find_packed_samples(&block.sub_blocks).expect("0x0A present");
        assert_eq!(packed.len(), 1);
        assert_eq!(
            block.decode_samples(),
            Err(Error::PackedSamplesOddLength(1))
        );
    }

    #[test]
    fn decode_samples_accepts_even_length_packed_samples_payload() {
        // The companion to the odd-rejection test: a plain even-length
        // 0x0A payload decodes normally (one `0` sample), proving the
        // spec §1 gate fires only on odd payloads.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let flags = flags_with(1 << 2);
        let bytes = synthesise_block(1, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.decode_samples(), Ok(vec![0]));
    }

    #[test]
    fn decode_samples_accepts_block_samples_at_the_ceiling() {
        // The guard rejects strictly above MAX_DECODE_SAMPLES_PER_BLOCK;
        // a block exactly at the ceiling is still attempted (and here
        // surfaces a truncation error from the per-sample loop, not the
        // size guard — proving the boundary is inclusive of the ceiling).
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0xFF, 0xFF]);
        let flags = flags_with(1 << 2);
        let bytes = synthesise_block(MAX_DECODE_SAMPLES_PER_BLOCK, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples();
        assert!(
            !matches!(err, Err(Error::BlockSamplesTooLarge(_))),
            "ceiling value must not trip the size guard, got {err:?}"
        );
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
    fn decode_samples_int32_mode_requires_the_0x09_profile() {
        // Round 405: INT32_DATA (bit 8) blocks are decoded, not
        // refused — but the 4-byte 0x09 int32-info profile is
        // mandatory; a block flagging bit 8 without it is malformed.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 8)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block.decode_samples().expect_err("0x09 is mandatory");
        assert_eq!(err, Error::BlockMissingInt32Info);
    }

    #[test]
    fn decode_samples_int32_zeros_profile_scales_the_pcm() {
        // An INT32_DATA block whose 0x09 profile strips 4 redundant
        // trailing zero bits: the decoded PCM is the entropy-decoded
        // value shifted left by 4 (no 0x0C needed), and the §5 main
        // CRC folds over the PRE-fixup buffer.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_small_sub_block(&mut payload, 0x09, &[0, 4, 0, 0]);
        // One-sample 0x0A stream: value 1 (zone 0 → prefix 0, no
        // mantissa with fresh medians... build via the encoder).
        let info_block = crate::entropy::expand_entropy(&[0u8; 6]).expect("entropy");
        let mut enc_medians =
            crate::samples::AdaptiveMedians::from_entropy(&info_block, 0).expect("medians");
        let packed =
            crate::samples::encode_packed_samples_mono(&[3], &mut enc_medians).expect("encode");
        append_packed_samples(&mut payload, &packed);
        let crc = crate::crc::crc_mono(&[3]); // pre-fixup fold
        let mut bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 8)), &payload);
        bytes[28..32].copy_from_slice(&crc.to_le_bytes());
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.decode_samples().expect("decode"), vec![3 << 4]);
        // The main CRC (over the pre-fixup value 3) gates cleanly.
        let (pcm, ok) = block.decode_samples_muted().expect("muted decode");
        assert!(ok, "pre-fixup CRC fold must match");
        assert_eq!(pcm, vec![3 << 4]);
        assert!(block.verify_decoded_crc().expect("verify"));
    }

    #[test]
    fn decode_samples_ignores_robust_experimental_bit() {
        // Wiki bit 28 is "robust block (experimental, okay to ignore if
        // encountered)" — the decode must proceed as if it were clear
        // (round-393 wvunpack cross-validation: reference encoders set
        // it on every block).
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let plain = synthesise_block(1, flags_with(1 << 2), &payload);
        let robust = synthesise_block(1, flags_with((1 << 2) | (1 << 28)), &payload);
        let (plain_block, _) = parse_block(&plain).expect("parse block");
        let (robust_block, _) = parse_block(&robust).expect("parse block");
        assert_eq!(
            robust_block.decode_samples().expect("robust decodes"),
            plain_block.decode_samples().expect("plain decodes"),
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
    fn decode_samples_decodes_joint_stereo_block() {
        // Bit 4 set on a stereo block → mid/side coding. The spec §5.4
        // mid/side undo (`R -= L>>1; L += R`) is now applied inside the
        // stereo decode path, so a joint-stereo block decodes rather than
        // being refused. Here the entropy stream carries one all-zero
        // pair, so the decoded (mid, side) = (0, 0); the §5.4 undo of
        // (0, 0) is (0, 0).
        let mut payload = Vec::new();
        append_entropy_info_stereo_minimal(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        // Stereo (mono bit clear) + bit 4 joint-stereo.
        let bytes = synthesise_block(1, flags_with(1 << 4), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.header.flags.mono);
        assert!(block.header.flags.joint_stereo);
        let pcm = block.decode_samples().expect("decode joint-stereo");
        assert_eq!(pcm, vec![0, 0]);
    }

    #[test]
    fn decode_samples_ignores_cross_decorr_bit_on_lossless_stereo() {
        // Bit 5 (`CROSS_DECORR`) set on a non-hybrid stereo block. The
        // staged decorrelation doc's only consumer of this flag is the
        // hybrid correction-fold placement (§4.1); the §1 lossless
        // decode order never consults it, and reference encoders set it
        // on plain lossless stereo files (round-393 wvunpack black-box
        // cross-validation) — so the lossless path decodes as if the
        // bit were clear.
        let mut payload = Vec::new();
        append_entropy_info_stereo_minimal(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let plain = synthesise_block(1, flags_with(0), &payload);
        let crossed = synthesise_block(1, flags_with(1 << 5), &payload);
        let (plain_block, _) = parse_block(&plain).expect("parse block");
        let (crossed_block, _) = parse_block(&crossed).expect("parse block");
        assert!(!crossed_block.header.flags.mono);
        assert!(crossed_block.header.flags.cross_channel_decorrelation);
        assert_eq!(
            crossed_block.decode_samples().expect("crossed decodes"),
            plain_block.decode_samples().expect("plain decodes"),
        );
    }

    #[test]
    fn decode_samples_mono_block_ignores_inter_channel_flags() {
        // A mono block carries a single decoded channel, so the bit 4 /
        // bit 5 inter-channel flags have no second channel to combine and
        // must NOT trip the stereo-only gates. Even with both bits set the
        // mono path decodes normally (here: one `0` sample). Guards
        // against the gate being placed before the mono/stereo split.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        // bit 2 mono + bit 4 joint + bit 5 cross.
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 4) | (1 << 5)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.header.flags.is_block_data_mono());
        assert!(block.header.flags.joint_stereo);
        assert!(block.header.flags.cross_channel_decorrelation);
        let got = block
            .decode_samples()
            .expect("mono decode ignores inter-channel flags");
        assert_eq!(got, vec![0]);
    }

    #[test]
    fn decode_samples_false_stereo_block_ignores_inter_channel_flags() {
        // A false-stereo block (bit 30) is stereo at the stream level but
        // mono at the block level, so it too has a single decoded channel
        // and must not trip the inter-channel gates.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        // bit 30 false_stereo + bit 4 joint + bit 5 cross (mono bit clear).
        let bytes = synthesise_block(1, flags_with((1 << 30) | (1 << 4) | (1 << 5)), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.header.flags.mono);
        assert!(block.header.flags.false_stereo);
        assert!(block.header.flags.is_block_data_mono());
        let got = block
            .decode_samples()
            .expect("false-stereo decode ignores inter-channel flags");
        assert_eq!(got, vec![0]);
    }

    #[test]
    fn decode_samples_rejects_mono_decorrelation_with_invalid_term_byte() {
        // A `0x02` term byte of `0x00` decodes (spec `+5` bias) to
        // `term = -5`, outside the valid set — the mono decorrelation
        // path is now wired, so the block is refused with the precise
        // `InvalidDecorrelationTerm` rather than the blanket
        // Decorrelation feature gate.
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x02, &[0u8; 2]); // two 0x00 term bytes
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_decorrelation());
        let err = block
            .decode_samples()
            .expect_err("must refuse invalid term byte");
        assert_eq!(err, Error::InvalidDecorrelationTerm(-5));
    }

    #[test]
    fn decode_samples_rejects_mono_decorrelation_weights_without_terms() {
        // A `0x03` weights sub-block with no `0x02` terms cannot name the
        // passes the weights belong to; the assembler reports the precise
        // `DecorrelationTermsMissing`.
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x03, &[0u8; 2]);
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block
            .decode_samples()
            .expect_err("must refuse 0x03 without 0x02");
        assert_eq!(err, Error::DecorrelationTermsMissing);
    }

    #[test]
    fn decode_samples_rejects_mono_decorrelation_samples_without_terms() {
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x04, &[0u8; 2]);
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        let err = block
            .decode_samples()
            .expect_err("must refuse 0x04 without 0x02");
        assert_eq!(err, Error::DecorrelationTermsMissing);
    }

    #[test]
    fn decode_samples_decodes_stereo_decorrelation() {
        // A genuinely-stereo block carrying 0x02/0x03/0x04 decorrelation
        // sub-blocks now decodes via the stereo prediction loop (spec §3)
        // rather than being refused. One term-1 pass with zero weights and
        // zero seeds applied to an all-zero residual stream reconstructs
        // all-zero PCM: apply_weight(w, 0) = 0, plus a 0 residual.
        let mut payload = Vec::new();
        // 0x02: two passes — term 1 (byte = term+5 = 6) and term 2 (= 7),
        // both delta 0. (Two bytes keeps the metadata payload even-length.)
        append_small_sub_block(&mut payload, 0x02, &[0x06, 0x07]);
        // 0x03: stereo → two weight bytes per pass (channel A, channel B);
        // two passes → 4 bytes, all zero.
        append_small_sub_block(&mut payload, 0x03, &[0x00, 0x00, 0x00, 0x00]);
        // 0x04: term 1 → 1 seed/channel (2 words), term 2 → 2 seeds/channel
        // (4 words) → 6 seed words = 12 bytes, all zero (exponent 0x09).
        append_small_sub_block(
            &mut payload,
            0x04,
            &[
                0x00, 0x09, 0x00, 0x09, 0x00, 0x09, 0x00, 0x09, 0x00, 0x09, 0x00, 0x09,
            ],
        );
        append_entropy_info_stereo_minimal(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        // No mono / false-stereo bit → genuinely stereo block data.
        let bytes = synthesise_block(1, flags_with(0), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_decorrelation());
        assert!(!block.header.flags.is_block_data_mono());
        let pcm = block
            .decode_samples()
            .expect("stereo decorrelation must decode");
        assert_eq!(pcm, vec![0, 0]);
    }

    #[test]
    fn decode_samples_applies_left_shift_to_both_stereo_channels() {
        // The left-shift fixup runs over the whole interleaved [L,R,L,R…]
        // buffer, so both channels of a stereo block are scaled. Build a
        // stereo decorrelation block that reconstructs known non-zero PCM,
        // stamp a non-zero left_shift, and confirm every interleaved slot
        // is shifted.
        use crate::decorrelation::{assemble_stereo_passes, decorrelate_stereo};
        use crate::samples::{encode_packed_samples_stereo, AdaptiveMedians};

        // Interleaved stereo residuals [L0,R0,L1,R1,…].
        let residuals: Vec<i32> = vec![5, -3, 8, 2, 12, -7, 4, 1];

        // Two passes (term 1 then term 2), delta 0, keeping every metadata
        // payload even-length. Stereo → 2 weight bytes (A,B) per pass.
        // Wire order is reverse application order; with symmetric content
        // here the order is immaterial to the even-length goal.
        let terms_payload = vec![0x06u8, 0x07u8]; // term 1 (=6), term 2 (=7)
        let weights_payload = vec![10u8, 12u8, 8u8, 6u8]; // (A,B) per pass
        let seed_word = |v: i32| -> [u8; 2] { [v as i8 as u8, 9] };
        let mut samples_payload = Vec::new();
        // Wire-stored last-pass-first: term 2 → 2 seeds/channel (4 words),
        // term 1 → 1 seed/channel (2 words). Total 6 words = 12 bytes.
        samples_payload.extend_from_slice(&seed_word(3)); // term2 A seed 0
        samples_payload.extend_from_slice(&seed_word(-1)); // term2 A seed 1
        samples_payload.extend_from_slice(&seed_word(2)); // term2 B seed 0
        samples_payload.extend_from_slice(&seed_word(-4)); // term2 B seed 1
        samples_payload.extend_from_slice(&seed_word(4)); // term1 A seed
        samples_payload.extend_from_slice(&seed_word(-2)); // term1 B seed

        // Expected (pre-shift) PCM from the standalone stereo engine.
        let mut passes = assemble_stereo_passes(&terms_payload, &weights_payload, &samples_payload)
            .expect("assemble stereo passes");
        let mut unshifted = residuals.clone();
        decorrelate_stereo(&mut passes, &mut unshifted).expect("decorrelate stereo");

        // Encode the residuals into a 0x0A payload seeded from the same
        // minimal-stereo 0x05 info the decoder reads.
        let mut stereo_info = Vec::new();
        append_entropy_info_stereo_minimal(&mut stereo_info);
        // The 0x05 payload bytes (skip the 2-byte sub-block header).
        let info_payload = &stereo_info[2..];
        let info_block = crate::entropy::expand_entropy(info_payload).expect("expand stereo info");
        let enc_a = AdaptiveMedians::from_entropy(&info_block, 0).expect("seed A");
        let enc_b = AdaptiveMedians::from_entropy(&info_block, 1).expect("seed B");
        let mut enc_medians = [enc_a, enc_b];
        let mut packed_0a = encode_packed_samples_stereo(&residuals, &mut enc_medians)
            .expect("encode stereo residuals");
        // The 0x0A payload must be even-length (spec §1: it binds as 16-bit
        // words). Pad with a trailing zero byte the reader never reaches
        // (decode stops after `block_samples` frames).
        if packed_0a.len() % 2 != 0 {
            packed_0a.push(0);
        }

        let shift = 3u32;
        let mut payload = Vec::new();
        append_small_sub_block(&mut payload, 0x02, &terms_payload);
        append_small_sub_block(&mut payload, 0x03, &weights_payload);
        append_small_sub_block(&mut payload, 0x04, &samples_payload);
        append_entropy_info_stereo_minimal(&mut payload);
        append_packed_samples(&mut payload, &packed_0a);

        // Genuinely-stereo block (no mono bit) + left_shift in bits 13..=17.
        let flags = flags_with(shift << 13);
        let frames = (residuals.len() / 2) as u32;
        let bytes = synthesise_block(frames, flags, &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.header.flags.is_block_data_mono());
        assert_eq!(block.header.flags.left_shift, shift as u8);

        let got = block.decode_samples().expect("stereo decode with shift");
        let expected: Vec<i32> = unshifted.iter().map(|&s| s << shift).collect();
        assert_eq!(got, expected, "both stereo channels must be left-shifted");
        assert_ne!(got, unshifted);
    }

    #[test]
    fn left_shift_zero_for_whole_byte_depth_blocks() {
        // Every existing whole-byte-depth synthesiser leaves bits 13..=17
        // clear, so the fixup is the identity and prior decode results are
        // unchanged — a regression guard that the new stage did not alter
        // the common path.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.header.flags.left_shift, 0);
        // `flags_with` leaves bits 0..=1 clear → bytes_per_sample = 1 (an
        // 8-bit container); with left_shift 0 the effective depth is 8.
        assert_eq!(block.header.flags.bytes_per_sample(), 1);
        assert_eq!(block.header.flags.effective_bit_depth(), 8);
        assert_eq!(block.decode_samples().expect("decode"), vec![0]);
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
            (UnsupportedBlockFeature::JointStereo, "flag bit 4"),
            (
                UnsupportedBlockFeature::CrossChannelDecorrelation,
                "flag bit 5",
            ),
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
    fn append_entropy_info_mono_seed(payload: &mut Vec<u8>, seed: [i32; 3]) {
        // Each median is a little-endian signed 16-bit log word
        // (crate::wp_exp2s expansion); test seeds stay in the
        // exactly-representable small-magnitude range.
        let mut bytes = Vec::with_capacity(6);
        for v in seed {
            assert_eq!(crate::quantize_log_value(v), v, "test median exact");
            bytes.extend_from_slice(&crate::pack_log_word(v));
        }
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

    /// A one-zero mono block with its §5 CRC stamped correctly, so the
    /// §5.6 mute gate keeps it.
    fn synthesise_crc_correct_mono_block_one_zero_sample() -> Vec<u8> {
        let mut bytes = synthesise_decodable_mono_block_one_zero_sample();
        let crc = crate::crc::crc_mono(&[0]);
        bytes[28..32].copy_from_slice(&crc.to_le_bytes());
        bytes
    }

    #[test]
    fn decode_stream_muted_all_blocks_pass_when_crc_correct() {
        // Two CRC-correct one-zero mono blocks: all_crc_ok is true and the
        // PCM is the concatenation of the (un-muted) decoded samples.
        let block = synthesise_crc_correct_mono_block_one_zero_sample();
        let mut bytes = block.clone();
        bytes.extend_from_slice(&block);

        let (pcm, all_crc_ok) = decode_stream_muted(&bytes).expect("decode muted stream");
        assert!(all_crc_ok, "both blocks have correct CRC");
        assert_eq!(pcm, vec![0, 0]);
    }

    #[test]
    fn decode_stream_muted_mutes_only_the_bad_block() {
        // [good][bad] where the bad block carries a non-zero sample but a
        // wrong CRC: the good block's sample survives, the bad block is
        // zeroed, and all_crc_ok is false.
        use crate::samples::{encode_packed_samples_mono, AdaptiveMedians};

        let good = synthesise_crc_correct_mono_block_one_zero_sample();

        // Bad block: a real non-zero mono PCM with a deliberately wrong CRC.
        let pcm = vec![7i32, -2];
        let info_block = crate::entropy::expand_entropy(&[0u8; 6]).expect("entropy");
        let mut enc_medians = AdaptiveMedians::from_entropy(&info_block, 0).expect("medians");
        let packed = encode_packed_samples_mono(&pcm, &mut enc_medians).expect("encode");
        let mut bad_payload = Vec::new();
        append_entropy_info_mono_zero(&mut bad_payload);
        append_packed_samples(&mut bad_payload, &packed);
        let mut bad = synthesise_block(pcm.len() as u32, flags_with(1 << 2), &bad_payload);
        let wrong = crate::crc::crc_mono(&pcm).wrapping_add(1);
        bad[28..32].copy_from_slice(&wrong.to_le_bytes());

        let mut bytes = good.clone();
        bytes.extend_from_slice(&bad);

        let (out, all_crc_ok) = decode_stream_muted(&bytes).expect("decode muted stream");
        assert!(!all_crc_ok, "one block has a wrong CRC");
        // Good block contributes [0]; bad block is muted to [0, 0].
        assert_eq!(out, vec![0, 0, 0]);
    }

    #[test]
    fn decode_stream_muted_skips_metadata_only_blocks() {
        // A leading metadata-only block must not affect all_crc_ok and
        // must contribute no PCM.
        let metadata_only = synthesise_header_bytes(MIN_CK_SIZE);
        let audio = synthesise_crc_correct_mono_block_one_zero_sample();
        let mut bytes = metadata_only;
        bytes.extend_from_slice(&audio);

        let (pcm, all_crc_ok) = decode_stream_muted(&bytes).expect("decode muted stream");
        assert!(all_crc_ok);
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

    /// Append a 0x0C packed-overflow-bits sub-block with the supplied
    /// payload bytes. Must be even-length (sub-block size is in 16-bit
    /// words and we don't set the odd-size flag here).
    fn append_packed_overflow_bits(payload: &mut Vec<u8>, bytes: &[u8]) {
        append_small_sub_block(payload, 0x0C, bytes);
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

    // ---- Round-242 0x0C packed-overflow-bits typed view + introspection ----

    #[test]
    fn has_packed_overflow_bits_returns_false_on_no_0x0c_subblock() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_packed_overflow_bits());
        assert!(block.packed_overflow_bits().is_none());
        assert!(block.find_packed_overflow_bits_sub_block().is_none());
    }

    #[test]
    fn has_packed_overflow_bits_returns_true_with_0x0c_subblock() {
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_packed_overflow_bits(&mut payload, &[0xCA, 0xFE]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.has_packed_overflow_bits());

        let view = block.packed_overflow_bits().expect("typed view");
        assert_eq!(view.bytes(), &[0xCA, 0xFE]);
        assert_eq!(view.len(), 2);

        let sub = block.find_packed_overflow_bits_sub_block().expect("borrow");
        assert_eq!(sub.id, SubBlockId::PackedOverflowBits);
        assert_eq!(sub.payload, &[0xCA, 0xFE]);
    }

    #[test]
    fn packed_overflow_bits_view_round_trips_with_bit_reader() {
        // A non-empty 0x0C payload should yield a typed view whose
        // bit_reader factory honours the LSB-first convention.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        // 0x03 = 0b0000_0011 -> LSB first: 1, 1, 0, 0, ...
        append_packed_overflow_bits(&mut payload, &[0x03, 0x00]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        let view = block.packed_overflow_bits().expect("typed view");
        let mut r = view.bit_reader();
        assert_eq!(r.bits_remaining(), 16);
        assert_eq!(r.get_bit().unwrap(), 1);
        assert_eq!(r.get_bit().unwrap(), 1);
        assert_eq!(r.get_bit().unwrap(), 0);
        assert_eq!(r.get_bit().unwrap(), 0);
    }

    #[test]
    fn has_packed_overflow_bits_is_independent_of_0x0b_and_0x07() {
        // 0x0C alongside 0x0B + 0x07 should all show up positively
        // and independently — the three IDs are distinct sub-block
        // discriminants and the accessors don't collide.
        let mut payload = Vec::new();
        append_entropy_info_mono_zero(&mut payload);
        append_packed_samples(&mut payload, &[0x00, 0x00]);
        append_noise_shaping_profile(&mut payload, &[0x11, 0x22]);
        append_packed_correction_data(&mut payload, &[0x33, 0x44]);
        append_packed_overflow_bits(&mut payload, &[0x55, 0x66]);
        let bytes = synthesise_block(1, flags_with(1 << 2), &payload);
        let (block, _) = parse_block(&bytes).expect("parse block");

        assert!(block.has_noise_shaping_profile());
        assert!(block.has_packed_correction_data());
        assert!(block.has_packed_overflow_bits());

        // Each typed view borrows its own distinct payload bytes.
        let overflow = block.packed_overflow_bits().expect("0x0C view");
        let correction = block.packed_correction_data().expect("0x0B view");
        assert_eq!(overflow.bytes(), &[0x55, 0x66]);
        assert_eq!(correction.bytes(), &[0x33, 0x44]);
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

    /// Synthesise a pairing-test block: audio header with the supplied
    /// index/samples/extra-flags, plus (for the `.wvc` side) a `0x0B`
    /// stub payload so the block looks correction-bearing.
    fn synth_pairing_block(index: u32, samples: u32, extra_flags: u32, wvc: bool) -> Vec<u8> {
        let mut payload = Vec::new();
        if wvc {
            append_packed_correction_data(&mut payload, &[0xAB, 0xCD]);
        } else {
            append_entropy_info_mono_zero(&mut payload);
            append_packed_samples(&mut payload, &[0x00, 0x00]);
        }
        let mut buf = synthesise_block(samples, flags_with(extra_flags), &payload);
        buf[16..20].copy_from_slice(&index.to_le_bytes());
        buf
    }

    #[test]
    fn pair_correction_stream_full_coverage() {
        let mono = 1u32 << 2;
        let mut main = synth_pairing_block(0, 100, mono, false);
        main.extend_from_slice(&synth_pairing_block(100, 50, mono, false));
        let mut wvc = synth_pairing_block(0, 100, mono, true);
        wvc.extend_from_slice(&synth_pairing_block(100, 50, mono, true));

        let pairs = pair_correction_stream(&main, &wvc).expect("pairing");
        assert_eq!(pairs.len(), 2);
        assert!(pairs.iter().all(|(_, c)| c.is_some()));
        assert_eq!(pairs[1].1.as_ref().unwrap().block_index(), 100);
        assert_eq!(correction_coverage(&main, &wvc).unwrap(), (2, 2));
    }

    #[test]
    fn pair_correction_stream_partial_coverage_pairs_none() {
        let mono = 1u32 << 2;
        let mut main = synth_pairing_block(0, 100, mono, false);
        main.extend_from_slice(&synth_pairing_block(100, 50, mono, false));
        // Correction file only covers the second block.
        let wvc = synth_pairing_block(100, 50, mono, true);

        let pairs = pair_correction_stream(&main, &wvc).expect("pairing");
        assert_eq!(pairs.len(), 2);
        assert!(pairs[0].1.is_none());
        assert!(pairs[1].1.is_some());
        assert_eq!(correction_coverage(&main, &wvc).unwrap(), (1, 2));
    }

    #[test]
    fn pair_correction_stream_empty_wvc_pairs_all_none() {
        let mono = 1u32 << 2;
        let mut main = synth_pairing_block(0, 100, mono, false);
        main.extend_from_slice(&synth_pairing_block(100, 50, mono, false));
        assert_eq!(correction_coverage(&main, &[]).unwrap(), (0, 2));
        // And a real lossless encode pairs cleanly with no wvc at all.
        let pcm: Vec<i32> = (0..300).map(|i| i * 7 % 101 - 50).collect();
        let stream = crate::encode::encode_stream_mono(&pcm, 100, 2).unwrap();
        let (paired, total) = correction_coverage(&stream, &[]).unwrap();
        assert_eq!(paired, 0);
        assert_eq!(total, 3);
    }

    #[test]
    fn pair_correction_stream_orphan_behind_is_index_mismatch() {
        let mono = 1u32 << 2;
        // Main starts at 100; correction claims a block at 0.
        let main = synth_pairing_block(100, 50, mono, false);
        let wvc = synth_pairing_block(0, 100, mono, true);
        assert_eq!(
            pair_correction_stream(&main, &wvc).expect_err("orphan"),
            Error::CorrectionIndexMismatch {
                main: 100,
                correction: 0
            }
        );
    }

    #[test]
    fn pair_correction_stream_sample_count_mismatch() {
        let mono = 1u32 << 2;
        let main = synth_pairing_block(0, 100, mono, false);
        let wvc = synth_pairing_block(0, 64, mono, true);
        assert_eq!(
            pair_correction_stream(&main, &wvc).expect_err("count"),
            Error::CorrectionSampleCountMismatch {
                main: 100,
                correction: 64
            }
        );
    }

    #[test]
    fn pair_correction_stream_shape_mismatch() {
        let main = synth_pairing_block(0, 100, 1u32 << 2, false); // mono
        let wvc = synth_pairing_block(0, 100, 0, true); // stereo
        assert_eq!(
            pair_correction_stream(&main, &wvc).expect_err("shape"),
            Error::CorrectionShapeMismatch(0)
        );
    }

    #[test]
    fn pair_correction_stream_surplus_wvc_blocks() {
        let mono = 1u32 << 2;
        let main = synth_pairing_block(0, 100, mono, false);
        let mut wvc = synth_pairing_block(0, 100, mono, true);
        wvc.extend_from_slice(&synth_pairing_block(100, 50, mono, true));
        assert_eq!(
            pair_correction_stream(&main, &wvc).expect_err("surplus"),
            Error::CorrectionBlockSurplus(100)
        );
        // Entirely main-less correction blocks are surplus too.
        assert_eq!(
            pair_correction_stream(&[], &synth_pairing_block(0, 10, mono, true))
                .expect_err("no main"),
            Error::CorrectionBlockSurplus(0)
        );
    }

    #[test]
    fn pair_correction_stream_skips_metadata_only_blocks() {
        let mono = 1u32 << 2;
        // A metadata-only (block_samples == 0) block sits between the
        // audio blocks on both sides; pairing must ignore it.
        let mut main = synth_pairing_block(0, 100, mono, false);
        main.extend_from_slice(&synthesise_block(0, flags_with(0), &[]));
        main.extend_from_slice(&synth_pairing_block(100, 50, mono, false));
        let mut wvc = synthesise_block(0, flags_with(0), &[]);
        wvc.extend_from_slice(&synth_pairing_block(0, 100, mono, true));
        wvc.extend_from_slice(&synth_pairing_block(100, 50, mono, true));
        assert_eq!(correction_coverage(&main, &wvc).unwrap(), (2, 2));
    }

    #[test]
    fn pair_correction_stream_propagates_parse_errors_from_both_sides() {
        let mono = 1u32 << 2;
        let good = synth_pairing_block(0, 100, mono, false);
        let mut bad_main = good.clone();
        bad_main.extend_from_slice(&[0u8; 3]);
        assert!(pair_correction_stream(&bad_main, &[]).is_err());

        let mut bad_wvc = synth_pairing_block(0, 100, mono, true);
        bad_wvc.extend_from_slice(&[0u8; 3]);
        assert!(pair_correction_stream(&good, &bad_wvc).is_err());
    }

    /// Fuzz regression (round 386): this minimized adversarial stream
    /// seeds a term-17/18 decorrelation pass with extreme history so
    /// the §3.2 extrapolator predictor overflows a non-wrapping i32
    /// multiply. The decode must return (Ok or a typed Err) without
    /// panicking; the input stays in the fuzz corpus as
    /// `regression_extrapolator_overflow.bin`.
    #[test]
    fn decode_survives_adversarial_extrapolator_history() {
        let bytes =
            include_bytes!("../fuzz/corpus/decode_stream/regression_extrapolator_overflow.bin");
        let _ = decode_stream(bytes);
        let _ = decode_stream_muted(bytes);
    }

    #[test]
    fn expects_correction_reads_the_hybrid_flag() {
        let hybrid = synth_pairing_block(0, 100, (1 << 2) | (1 << 3), false);
        let (block, _) = parse_block(&hybrid).expect("parse");
        assert!(block.expects_correction());

        let lossless = synth_pairing_block(0, 100, 1 << 2, false);
        let (block, _) = parse_block(&lossless).expect("parse");
        assert!(!block.expects_correction());
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

    // ---- Round-239 total_samples_in_file / end_sample_index /
    // samples_remaining_after / is_final_audio_block_in_file +
    // stream_total_samples ----

    /// Synthesise a block that pins the three round-239 header fields
    /// (`total_samples`, `block_index`, `block_samples`) to caller
    /// values. Standalone multichannel marker is set so the block
    /// would also pass the round-206 composer's structural gates if a
    /// decode test ever runs against this layout.
    fn synthesise_block_with_indices(
        total_samples: u32,
        block_index: u32,
        block_samples: u32,
    ) -> Vec<u8> {
        let ck_size = 24u32; // no metadata sub-blocks
        let mut buf = vec![0u8; HEADER_LEN];
        buf[0..4].copy_from_slice(MAGIC);
        buf[4..8].copy_from_slice(&ck_size.to_le_bytes());
        buf[8..10].copy_from_slice(&0x0410u16.to_le_bytes());
        buf[12..16].copy_from_slice(&total_samples.to_le_bytes());
        buf[16..20].copy_from_slice(&block_index.to_le_bytes());
        buf[20..24].copy_from_slice(&block_samples.to_le_bytes());
        // Flags = standalone multichannel marker so the block is well-formed
        // even if downstream tests round-trip it through the composer.
        let flags = 0b11u32 << 11;
        buf[24..28].copy_from_slice(&flags.to_le_bytes());
        buf
    }

    #[test]
    fn block_total_samples_in_file_passes_through_header_accessor() {
        let bytes = synthesise_block_with_indices(123_456, 0, 1024);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.total_samples_in_file(), Some(123_456));
    }

    #[test]
    fn block_total_samples_in_file_is_none_for_sentinel() {
        let bytes = synthesise_block_with_indices(TOTAL_SAMPLES_UNKNOWN, 0, 1024);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.total_samples_in_file(), None);
    }

    #[test]
    fn block_end_sample_index_sums_block_index_and_block_samples() {
        let bytes = synthesise_block_with_indices(10_000, 1_000, 1_024);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.end_sample_index(), 2_024);
        assert_eq!(
            block.block_index() as u64 + block.block_samples() as u64,
            2_024
        );
    }

    #[test]
    fn block_samples_remaining_after_returns_some_for_known_total() {
        let bytes = synthesise_block_with_indices(10_000, 1_000, 1_024);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.samples_remaining_after(), Some(7_976));
    }

    #[test]
    fn block_samples_remaining_after_returns_zero_at_exact_end_of_file() {
        // A final block ending at the file total.
        let bytes = synthesise_block_with_indices(2_048, 1_024, 1_024);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.samples_remaining_after(), Some(0));
        assert!(block.is_final_audio_block_in_file());
    }

    #[test]
    fn block_is_final_audio_block_in_file_is_false_when_total_is_unknown() {
        // Wiki sentinel: cannot answer "is this final" without the
        // declared total.
        let bytes = synthesise_block_with_indices(TOTAL_SAMPLES_UNKNOWN, 1_000, 1_024);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.is_final_audio_block_in_file());
        assert_eq!(block.samples_remaining_after(), None);
    }

    #[test]
    fn block_is_final_audio_block_in_file_is_false_when_more_samples_remain() {
        let bytes = synthesise_block_with_indices(10_000, 0, 1_024);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.is_final_audio_block_in_file());
        assert_eq!(block.samples_remaining_after(), Some(8_976));
    }

    #[test]
    fn stream_total_samples_returns_first_blocks_total() {
        // Single-block stream: stream total == that block's header field.
        let bytes = synthesise_block_with_indices(98_765, 0, 1_024);
        assert_eq!(stream_total_samples(&bytes).unwrap(), Some(Some(98_765)));
    }

    #[test]
    fn stream_total_samples_returns_none_for_sentinel_on_first_block() {
        let bytes = synthesise_block_with_indices(TOTAL_SAMPLES_UNKNOWN, 0, 0);
        assert_eq!(stream_total_samples(&bytes).unwrap(), Some(None));
    }

    #[test]
    fn stream_total_samples_returns_outer_none_for_empty_input() {
        // Empty input → no first block to read.
        assert_eq!(stream_total_samples(&[]).unwrap(), None);
    }

    #[test]
    fn stream_total_samples_reads_first_block_only_even_on_multi_block_input() {
        // Multi-block stream where the first block's total differs
        // from a (hypothetically-malformed) second block's total. The
        // accessor reads only the first block — the wiki "total
        // samples in file" is file-global, so it's the source of
        // truth.
        let a = synthesise_block_with_indices(50_000, 0, 1_024);
        let b = synthesise_block_with_indices(99_999, 1_024, 1_024);
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&a);
        bytes.extend_from_slice(&b);
        assert_eq!(stream_total_samples(&bytes).unwrap(), Some(Some(50_000)));
    }

    #[test]
    fn stream_total_samples_surfaces_parse_error_on_malformed_header() {
        // Truncated input (only 8 bytes) → Error::Truncated, not Ok.
        let bytes = vec![b'w', b'v', b'p', b'k', 0, 0, 0, 0];
        let err = stream_total_samples(&bytes).expect_err("must reject");
        assert_eq!(err, Error::Truncated);
    }

    /// Synthesise a minimal valid block with a chosen `crc` trailing
    /// word, extending the round-239 `synthesise_block_with_indices`
    /// helper to the round-245 `crc()` accessor's coverage. The wiki
    /// places the CRC word at bytes 28..32 of the fixed header (the
    /// last 4 bytes before the metadata sub-block region).
    fn synthesise_block_with_crc(crc: u32) -> Vec<u8> {
        let ck_size = 24u32;
        let mut buf = vec![0u8; HEADER_LEN];
        buf[0..4].copy_from_slice(MAGIC);
        buf[4..8].copy_from_slice(&ck_size.to_le_bytes());
        buf[8..10].copy_from_slice(&0x0410u16.to_le_bytes());
        // Standalone marker so structural composer gates pass too.
        let flags = 0b11u32 << 11;
        buf[24..28].copy_from_slice(&flags.to_le_bytes());
        buf[28..32].copy_from_slice(&crc.to_le_bytes());
        buf
    }

    #[test]
    fn block_crc_passes_through_header_crc_verbatim() {
        // Block-level crc() returns the stored word — no recomputation,
        // matches the header accessor exactly.
        let bytes = synthesise_block_with_crc(0xDEAD_BEEF);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.crc(), 0xDEAD_BEEF);
        assert_eq!(block.crc(), block.header().crc());
        assert_eq!(block.crc(), block.header().crc);
    }

    #[test]
    fn block_crc_round_trips_full_u32_range_extremes() {
        // The CRC field has no sentinel — every u32 is a valid stored
        // value. Pin both extremes and a handful of representative
        // patterns through the full parse path.
        for w in [
            0u32,
            1,
            u32::MAX,
            0xFFFF_FFFE,
            0x0000_0001,
            0x8000_0000,
            0x7FFF_FFFF,
            0x1234_5678,
            0xCAFE_BABE,
        ] {
            let bytes = synthesise_block_with_crc(w);
            let (block, _) = parse_block(&bytes).expect("parse block");
            assert_eq!(
                block.crc(),
                w,
                "block CRC 0x{w:08x} should round-trip verbatim"
            );
        }
    }

    #[test]
    fn block_crc_differs_per_block_across_a_two_block_stream() {
        // Two back-to-back blocks with distinct CRC words. Walking the
        // stream should yield each block's CRC verbatim — the field is
        // per-block, not file-global like `total_samples`.
        let a = synthesise_block_with_crc(0x1111_1111);
        let b = synthesise_block_with_crc(0x2222_2222);
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&a);
        bytes.extend_from_slice(&b);
        let parsed = parse_blocks(&bytes).expect("parse two blocks");
        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[0].crc(), 0x1111_1111);
        assert_eq!(parsed[1].crc(), 0x2222_2222);
        // Distinct values — confirms the per-block independence.
        assert_ne!(parsed[0].crc(), parsed[1].crc());
    }

    #[test]
    fn block_crc_independent_of_other_header_accessors() {
        // The CRC accessor reads bytes 28..32 only — varying other
        // header fields should not change the reported CRC.
        let mut bytes = synthesise_block_with_indices(50_000, 1_024, 2_048);
        // Stamp a CRC into the synthesised buffer's bytes 28..32.
        bytes[28..32].copy_from_slice(&0xABCD_EF01u32.to_le_bytes());
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.crc(), 0xABCD_EF01);
        // The other accessors still report the round-239 values.
        assert_eq!(block.total_samples_in_file(), Some(50_000));
        assert_eq!(block.block_index(), 1_024);
        assert_eq!(block.block_samples(), 2_048);
    }

    // ---------------- Round 252: version + track-id passthroughs ----------------

    /// Synthesise a minimal valid block with a chosen `version` /
    /// `track_number` / `track_sub_index` triple, extending the
    /// round-245 `synthesise_block_with_crc` helper to the round-252
    /// accessors' coverage.
    fn synthesise_block_with_version_track(
        version: u16,
        track_number: u8,
        track_sub_index: u8,
    ) -> Vec<u8> {
        let ck_size = 24u32;
        let mut buf = vec![0u8; HEADER_LEN];
        buf[0..4].copy_from_slice(MAGIC);
        buf[4..8].copy_from_slice(&ck_size.to_le_bytes());
        buf[8..10].copy_from_slice(&version.to_le_bytes());
        buf[10] = track_number;
        buf[11] = track_sub_index;
        // Standalone marker so structural composer gates pass too.
        let flags = 0b11u32 << 11;
        buf[24..28].copy_from_slice(&flags.to_le_bytes());
        buf
    }

    #[test]
    fn block_version_passes_through_header_version() {
        let bytes = synthesise_block_with_version_track(0x0410, 0, 0);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.version(), 0x0410);
        assert_eq!(block.version(), block.header().version());
    }

    #[test]
    fn block_version_round_trips_documented_window() {
        for v in [
            crate::MIN_VERSION,
            0x0405,
            0x040A,
            0x040F,
            crate::MAX_VERSION,
        ] {
            let bytes = synthesise_block_with_version_track(v, 0, 0);
            let (block, _) = parse_block(&bytes).expect("parse block");
            assert_eq!(block.version(), v, "block version 0x{v:04x} round-trips");
        }
    }

    #[test]
    fn block_track_number_passes_through_header_track_number() {
        let bytes = synthesise_block_with_version_track(0x0410, 0x55, 0);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.track_number(), 0x55);
        assert_eq!(block.track_number(), block.header().track_number());
        assert_eq!(block.track_sub_index(), 0);
    }

    #[test]
    fn block_track_sub_index_passes_through_header_track_sub_index() {
        let bytes = synthesise_block_with_version_track(0x0410, 0, 0xAA);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert_eq!(block.track_sub_index(), 0xAA);
        assert_eq!(block.track_sub_index(), block.header().track_sub_index());
        assert_eq!(block.track_number(), 0);
    }

    #[test]
    fn block_has_track_id_false_when_both_bytes_zero() {
        let bytes = synthesise_block_with_version_track(0x0410, 0, 0);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(!block.has_track_id());
    }

    #[test]
    fn block_has_track_id_true_when_either_or_both_bytes_set() {
        for (n, s) in [(1u8, 0u8), (0, 1), (0x42, 0x99)] {
            let bytes = synthesise_block_with_version_track(0x0410, n, s);
            let (block, _) = parse_block(&bytes).expect("parse block");
            assert!(
                block.has_track_id(),
                "track_number=0x{n:02x} track_sub_index=0x{s:02x} should report has_track_id"
            );
        }
    }

    #[test]
    fn block_supports_false_stereo_true_at_0x0410() {
        let bytes = synthesise_block_with_version_track(0x0410, 0, 0);
        let (block, _) = parse_block(&bytes).expect("parse block");
        assert!(block.supports_false_stereo());
    }

    #[test]
    fn block_supports_false_stereo_false_below_0x0410() {
        for v in [crate::MIN_VERSION, 0x0405, 0x040F] {
            let bytes = synthesise_block_with_version_track(v, 0, 0);
            let (block, _) = parse_block(&bytes).expect("parse block");
            assert!(
                !block.supports_false_stereo(),
                "block version 0x{v:04x} should not support false_stereo"
            );
        }
    }

    #[test]
    fn block_round_252_accessors_independent_across_two_block_stream() {
        // Two back-to-back blocks with distinct version + track triples.
        // Walking the stream should yield each block's stamping verbatim.
        let a = synthesise_block_with_version_track(crate::MIN_VERSION, 0x01, 0x02);
        let b = synthesise_block_with_version_track(crate::MAX_VERSION, 0x10, 0x20);
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&a);
        bytes.extend_from_slice(&b);
        let parsed = parse_blocks(&bytes).expect("parse two blocks");
        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[0].version(), crate::MIN_VERSION);
        assert_eq!(parsed[0].track_number(), 0x01);
        assert_eq!(parsed[0].track_sub_index(), 0x02);
        assert!(parsed[0].has_track_id());
        assert!(!parsed[0].supports_false_stereo());
        assert_eq!(parsed[1].version(), crate::MAX_VERSION);
        assert_eq!(parsed[1].track_number(), 0x10);
        assert_eq!(parsed[1].track_sub_index(), 0x20);
        assert!(parsed[1].has_track_id());
        assert!(parsed[1].supports_false_stereo());
    }

    #[test]
    fn block_round_239_accessors_are_consistent_across_a_three_block_stream() {
        // A three-block stream with a known total — walk it and pin
        // end_sample_index for each block, summing block_samples should
        // equal the total at the last block.
        let blocks = [
            synthesise_block_with_indices(3_072, 0, 1_024),
            synthesise_block_with_indices(3_072, 1_024, 1_024),
            synthesise_block_with_indices(3_072, 2_048, 1_024),
        ];
        let mut stream = Vec::new();
        for b in &blocks {
            stream.extend_from_slice(b);
        }
        let parsed = parse_blocks(&stream).unwrap();
        assert_eq!(parsed.len(), 3);
        assert_eq!(parsed[0].end_sample_index(), 1_024);
        assert_eq!(parsed[1].end_sample_index(), 2_048);
        assert_eq!(parsed[2].end_sample_index(), 3_072);
        // Only the last block reports zero remaining.
        assert_eq!(parsed[0].samples_remaining_after(), Some(2_048));
        assert_eq!(parsed[1].samples_remaining_after(), Some(1_024));
        assert_eq!(parsed[2].samples_remaining_after(), Some(0));
        // And only the last block claims "final".
        assert!(!parsed[0].is_final_audio_block_in_file());
        assert!(!parsed[1].is_final_audio_block_in_file());
        assert!(parsed[2].is_final_audio_block_in_file());
    }

    // ---- round 367: block-level hybrid correction fold (§4.1) -------

    #[test]
    fn hybrid_placement_is_post_decorrelation_for_a_plain_hybrid_block() {
        // mono (bit 2) + hybrid (bit 3), no cross / no shaping.
        let bytes = synthesise_block(1, flags_with((1 << 2) | (1 << 3)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        assert_eq!(
            block.hybrid_correction_placement(),
            crate::CorrectionFold::PostDecorrelation
        );
    }

    #[test]
    fn hybrid_placement_is_cross_when_cross_decorr_set() {
        // hybrid (bit 3) + CROSS_DECORR (bit 5).
        let bytes = synthesise_block(1, flags_with((1 << 3) | (1 << 5)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        assert_eq!(
            block.hybrid_correction_placement(),
            crate::CorrectionFold::PreDecorrelationCross
        );
    }

    #[test]
    fn hybrid_placement_is_shaped_when_shape_bit_set() {
        // hybrid (bit 3) + HYBRID_SHAPE (bit 6).
        let bytes = synthesise_block(1, flags_with((1 << 3) | (1 << 6)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        assert_eq!(
            block.hybrid_correction_placement(),
            crate::CorrectionFold::NoiseShaped
        );
    }

    #[test]
    fn fold_hybrid_correction_recovers_lossless_pcm() {
        // A plain (post-decorrelation) hybrid block: the fold adds the
        // correction residual to each reconstructed lossy sample.
        let bytes = synthesise_block(4, flags_with((1 << 2) | (1 << 3)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        let lossy = [100i32, -200, 300, -400];
        let correction = [3i32, -5, 0, 7];
        let lossless = block.fold_hybrid_correction(&lossy, &correction).unwrap();
        assert_eq!(lossless, vec![103, -205, 300, -393]);
    }

    #[test]
    fn fold_hybrid_correction_zero_correction_is_identity() {
        let bytes = synthesise_block(3, flags_with((1 << 2) | (1 << 3)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        let lossy = [7i32, -9, 11];
        let out = block.fold_hybrid_correction(&lossy, &[0, 0, 0]).unwrap();
        assert_eq!(out, lossy);
    }

    #[test]
    fn fold_hybrid_correction_refuses_cross_decorr() {
        let bytes = synthesise_block(2, flags_with((1 << 3) | (1 << 5)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        let err = block.fold_hybrid_correction(&[1, 2], &[0, 0]).unwrap_err();
        assert!(matches!(err, Error::HybridFoldPlacementUnsupported));
    }

    #[test]
    fn fold_hybrid_correction_refuses_noise_shaping() {
        let bytes = synthesise_block(2, flags_with((1 << 3) | (1 << 6)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        let err = block.fold_hybrid_correction(&[1, 2], &[0, 0]).unwrap_err();
        assert!(matches!(err, Error::HybridFoldPlacementUnsupported));
    }

    #[test]
    fn fold_hybrid_correction_refuses_length_mismatch() {
        let bytes = synthesise_block(3, flags_with((1 << 2) | (1 << 3)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        let err = block
            .fold_hybrid_correction(&[1, 2, 3], &[0, 0])
            .unwrap_err();
        assert!(matches!(
            err,
            Error::HybridCorrectionLengthMismatch {
                lossy: 3,
                correction: 2
            }
        ));
    }

    #[test]
    fn split_hybrid_correction_is_the_forward_inverse_of_the_fold() {
        // split(original, lossy) then fold(lossy, correction) == original.
        let bytes = synthesise_block(4, flags_with((1 << 2) | (1 << 3)), &[]);
        let (block, _) = parse_block(&bytes).unwrap();
        let original = [103i32, -205, 300, -393];
        let lossy = [100i32, -200, 300, -400];
        let correction = block.split_hybrid_correction(&original, &lossy).unwrap();
        assert_eq!(correction, vec![3, -5, 0, 7]);
        let recovered = block.fold_hybrid_correction(&lossy, &correction).unwrap();
        assert_eq!(recovered, original);
    }

    #[test]
    fn split_hybrid_correction_refuses_cross_and_shaped_and_mismatch() {
        // CROSS_DECORR placement.
        let cross = synthesise_block(2, flags_with((1 << 3) | (1 << 5)), &[]);
        let (cblock, _) = parse_block(&cross).unwrap();
        assert!(matches!(
            cblock
                .split_hybrid_correction(&[1, 2], &[0, 0])
                .unwrap_err(),
            Error::HybridFoldPlacementUnsupported
        ));
        // Noise-shaped placement.
        let shaped = synthesise_block(2, flags_with((1 << 3) | (1 << 6)), &[]);
        let (sblock, _) = parse_block(&shaped).unwrap();
        assert!(matches!(
            sblock
                .split_hybrid_correction(&[1, 2], &[0, 0])
                .unwrap_err(),
            Error::HybridFoldPlacementUnsupported
        ));
        // Length mismatch on a plain block.
        let plain = synthesise_block(3, flags_with((1 << 2) | (1 << 3)), &[]);
        let (pblock, _) = parse_block(&plain).unwrap();
        assert!(matches!(
            pblock
                .split_hybrid_correction(&[1, 2, 3], &[0])
                .unwrap_err(),
            Error::HybridCorrectionLengthMismatch {
                lossy: 1,
                correction: 3
            }
        ));
    }

    // ---- Multichannel grouping decode (round 378) ----------------------

    /// Patch the wiki bits-11..=12 multichannel grouping marker of an
    /// already-encoded block in place. `marker` is the 2-bit value
    /// (`0b01` = first, `0b10` = final, `0b00` = continuation, `0b11` =
    /// standalone). The marker bits are independent of the §5 sample CRC
    /// (which is folded over the PCM, not the flag word), so a
    /// marker-patched block stays CRC-valid.
    fn patch_marker(block: &mut [u8], marker: u32) {
        let mut flags = u32::from_le_bytes([block[24], block[25], block[26], block[27]]);
        flags &= !(0b11 << 11);
        flags |= (marker & 0b11) << 11;
        block[24..28].copy_from_slice(&flags.to_le_bytes());
    }

    /// Build a multichannel member block from a single channel of PCM
    /// (mono member) carrying the supplied grouping marker.
    fn mono_member(pcm: &[i32], block_index: u32, total: u32, marker: u32) -> Vec<u8> {
        let mut b = crate::encode::encode_block_mono(pcm, 2, block_index, total).unwrap();
        patch_marker(&mut b, marker);
        b
    }

    /// Build a stereo member block from interleaved L/R PCM carrying the
    /// supplied grouping marker.
    fn stereo_member(pcm: &[i32], block_index: u32, total: u32, marker: u32) -> Vec<u8> {
        let mut b = crate::encode::encode_block_stereo(pcm, 2, block_index, total).unwrap();
        patch_marker(&mut b, marker);
        b
    }

    #[test]
    fn multichannel_three_mono_members_interleave_in_member_order() {
        // A 3-channel set: three mono members, markers first / cont / final.
        let c0 = [10, 11, 12, 13];
        let c1 = [20, 21, 22, 23];
        let c2 = [30, 31, 32, 33];
        let total = 4;
        let mut stream = mono_member(&c0, 0, total, 0b01);
        stream.extend(mono_member(&c1, 0, total, 0b00));
        stream.extend(mono_member(&c2, 0, total, 0b10));

        let decoded = decode_multichannel_stream(&stream).unwrap();
        assert_eq!(decoded.channels, 3);
        // Frames interleaved [c0,c1,c2] per frame, in member (speaker) order.
        assert_eq!(
            decoded.samples,
            vec![10, 20, 30, 11, 21, 31, 12, 22, 32, 13, 23, 33]
        );
    }

    #[test]
    fn multichannel_mixed_mono_and_stereo_members() {
        // 4-channel set: stereo member (2ch) then stereo member (2ch).
        let front = [100, 200, 101, 201]; // L0 R0 L1 R1
        let rear = [300, 400, 301, 401];
        let total = 2;
        let mut stream = stereo_member(&front, 0, total, 0b01);
        stream.extend(stereo_member(&rear, 0, total, 0b10));

        let decoded = decode_multichannel_stream(&stream).unwrap();
        assert_eq!(decoded.channels, 4);
        // Per frame: front L, front R, rear L, rear R.
        assert_eq!(
            decoded.samples,
            vec![100, 200, 300, 400, 101, 201, 301, 401]
        );
    }

    #[test]
    fn multichannel_mono_plus_stereo_member_mix() {
        // 3-channel set: a mono centre member then a stereo member.
        let centre = [7, 8, 9];
        let lr = [1, 2, 3, 4, 5, 6]; // L0 R0 L1 R1 L2 R2
        let total = 3;
        let mut stream = mono_member(&centre, 0, total, 0b01);
        stream.extend(stereo_member(&lr, 0, total, 0b10));

        let decoded = decode_multichannel_stream(&stream).unwrap();
        assert_eq!(decoded.channels, 3);
        // Per frame: centre, L, R.
        assert_eq!(decoded.samples, vec![7, 1, 2, 8, 3, 4, 9, 5, 6]);
    }

    #[test]
    fn multichannel_multiple_sets_concatenate() {
        // Two 3-channel sets back to back (two frame ranges).
        let total = 4;
        let mut stream = mono_member(&[1, 2], 0, total, 0b01);
        stream.extend(mono_member(&[3, 4], 0, total, 0b00));
        stream.extend(mono_member(&[5, 6], 0, total, 0b10));
        // Second set, block_index advanced.
        stream.extend(mono_member(&[7, 8], 2, total, 0b01));
        stream.extend(mono_member(&[9, 10], 2, total, 0b00));
        stream.extend(mono_member(&[11, 12], 2, total, 0b10));

        let decoded = decode_multichannel_stream(&stream).unwrap();
        assert_eq!(decoded.channels, 3);
        assert_eq!(decoded.samples, vec![1, 3, 5, 2, 4, 6, 7, 9, 11, 8, 10, 12]);
    }

    #[test]
    fn multichannel_standalone_mono_matches_decode_stream() {
        // A plain mono file (standalone marker 0b11 on every block) decodes
        // to the same PCM as decode_stream, with channels == 1.
        let pcm = [3, -2, 5, 0, -7, 9];
        let stream = crate::encode::encode_block_mono(&pcm, 2, 0, pcm.len() as u32).unwrap();
        let decoded = decode_multichannel_stream(&stream).unwrap();
        assert_eq!(decoded.channels, 1);
        assert_eq!(decoded.samples, decode_stream(&stream).unwrap());
        assert_eq!(decoded.samples, pcm.to_vec());
    }

    #[test]
    fn multichannel_standalone_stereo_matches_decode_stream() {
        let pcm = [3, -2, 5, 0, -7, 9];
        let stream =
            crate::encode::encode_block_stereo(&pcm, 2, 0, (pcm.len() / 2) as u32).unwrap();
        let decoded = decode_multichannel_stream(&stream).unwrap();
        assert_eq!(decoded.channels, 2);
        assert_eq!(decoded.samples, decode_stream(&stream).unwrap());
    }

    #[test]
    fn multichannel_stray_final_marker_is_malformed() {
        // A final-marker block with no preceding first-marker.
        let stream = mono_member(&[1, 2], 0, 2, 0b10);
        assert!(matches!(
            decode_multichannel_stream(&stream).unwrap_err(),
            Error::MultichannelSetMalformed
        ));
    }

    #[test]
    fn multichannel_unterminated_set_is_malformed() {
        // A first-marker block that never sees a final marker.
        let stream = mono_member(&[1, 2], 0, 2, 0b01);
        assert!(matches!(
            decode_multichannel_stream(&stream).unwrap_err(),
            Error::MultichannelSetMalformed
        ));
    }

    #[test]
    fn multichannel_double_first_marker_is_malformed() {
        // Two first-markers without a final in between.
        let mut stream = mono_member(&[1, 2], 0, 2, 0b01);
        stream.extend(mono_member(&[3, 4], 0, 2, 0b01));
        assert!(matches!(
            decode_multichannel_stream(&stream).unwrap_err(),
            Error::MultichannelSetMalformed
        ));
    }

    #[test]
    fn multichannel_member_sample_count_mismatch() {
        // Members of one set disagree on block_samples.
        let mut stream = mono_member(&[1, 2, 3], 0, 3, 0b01);
        stream.extend(mono_member(&[4, 5], 0, 2, 0b10));
        assert!(matches!(
            decode_multichannel_stream(&stream).unwrap_err(),
            Error::MultichannelSampleCountMismatch {
                expected: 3,
                found: 2
            }
        ));
    }

    #[test]
    fn multichannel_skips_metadata_only_blocks() {
        // A leading metadata-only block (block_samples == 0) is not a set
        // member and is skipped.
        let meta_only = synthesise_block(0, flags_with(1 << 2), &[]);
        let mut stream = meta_only;
        stream.extend(mono_member(&[1, 2], 0, 2, 0b01));
        stream.extend(mono_member(&[3, 4], 0, 2, 0b10));
        let decoded = decode_multichannel_stream(&stream).unwrap();
        assert_eq!(decoded.channels, 2);
        assert_eq!(decoded.samples, vec![1, 3, 2, 4]);
    }

    #[test]
    fn multichannel_empty_stream_reports_zero_channels() {
        let decoded = decode_multichannel_stream(&[]).unwrap();
        assert_eq!(decoded.channels, 0);
        assert!(decoded.samples.is_empty());
    }

    /// Corrupt a block's stored §5 CRC word (bytes 28..32) so the muted
    /// decode path mutes it.
    fn corrupt_crc(block: &mut [u8]) {
        block[28..32].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
    }

    #[test]
    fn multichannel_muted_all_crc_ok_when_intact() {
        // Every member's CRC is valid → muted decode agrees with the plain
        // decode and reports all_crc_ok.
        let total = 2;
        let mut stream = mono_member(&[10, 11], 0, total, 0b01);
        stream.extend(mono_member(&[20, 21], 0, total, 0b00));
        stream.extend(mono_member(&[30, 31], 0, total, 0b10));

        let (decoded, all_ok) = decode_multichannel_stream_muted(&stream).unwrap();
        assert!(all_ok);
        assert_eq!(decoded.channels, 3);
        assert_eq!(
            decoded.samples,
            decode_multichannel_stream(&stream).unwrap().samples
        );
        assert_eq!(decoded.samples, vec![10, 20, 30, 11, 21, 31]);
    }

    #[test]
    fn multichannel_muted_zeros_only_the_corrupt_member() {
        // Corrupt the middle (continuation) member's CRC: its channel is
        // muted to zeros, the other two channels survive, and all_crc_ok
        // is false.
        let total = 2;
        let mut stream = mono_member(&[10, 11], 0, total, 0b01);
        let mut mid = mono_member(&[20, 21], 0, total, 0b00);
        corrupt_crc(&mut mid);
        stream.extend(mid);
        stream.extend(mono_member(&[30, 31], 0, total, 0b10));

        let (decoded, all_ok) = decode_multichannel_stream_muted(&stream).unwrap();
        assert!(!all_ok);
        assert_eq!(decoded.channels, 3);
        // Middle channel zeroed; outer two intact.
        assert_eq!(decoded.samples, vec![10, 0, 30, 11, 0, 31]);
    }

    #[test]
    fn decode_member_samples_muted_mutes_on_bad_crc() {
        let pcm = [5, -3, 8, 1];
        let mut block = mono_member(&pcm, 0, 4, 0b01);
        corrupt_crc(&mut block);
        let (parsed, _) = parse_block(&block).unwrap();
        let (muted, crc_ok) = parsed.decode_member_samples_muted().unwrap();
        assert!(!crc_ok);
        assert_eq!(muted, vec![0, 0, 0, 0]);
    }

    #[test]
    fn multichannel_layout_reports_channels_and_sets() {
        // Two 3-channel sets.
        let total = 4;
        let mut stream = mono_member(&[1, 2], 0, total, 0b01);
        stream.extend(mono_member(&[3, 4], 0, total, 0b00));
        stream.extend(mono_member(&[5, 6], 0, total, 0b10));
        stream.extend(mono_member(&[7, 8], 2, total, 0b01));
        stream.extend(mono_member(&[9, 10], 2, total, 0b00));
        stream.extend(mono_member(&[11, 12], 2, total, 0b10));

        let layout = multichannel_layout(&stream).unwrap();
        assert_eq!(layout.channels, 3);
        assert_eq!(layout.sets, 2);
    }

    #[test]
    fn multichannel_layout_counts_stereo_members() {
        // 4 channels via two stereo members.
        let total = 2;
        let mut stream = stereo_member(&[1, 2, 3, 4], 0, total, 0b01);
        stream.extend(stereo_member(&[5, 6, 7, 8], 0, total, 0b10));
        let layout = multichannel_layout(&stream).unwrap();
        assert_eq!(layout.channels, 4);
        assert_eq!(layout.sets, 1);
    }

    #[test]
    fn multichannel_layout_plain_mono_is_one_channel_per_set() {
        let pcm = [3, -2, 5];
        let stream = crate::encode::encode_block_mono(&pcm, 2, 0, 3).unwrap();
        let layout = multichannel_layout(&stream).unwrap();
        assert_eq!(layout.channels, 1);
        assert_eq!(layout.sets, 1);
    }

    #[test]
    fn multichannel_layout_empty_stream_is_zero() {
        let layout = multichannel_layout(&[]).unwrap();
        assert_eq!(layout.channels, 0);
        assert_eq!(layout.sets, 0);
    }

    #[test]
    fn multichannel_layout_refuses_malformed_grouping() {
        // Stray final marker.
        let stream = mono_member(&[1, 2], 0, 2, 0b10);
        assert!(matches!(
            multichannel_layout(&stream).unwrap_err(),
            Error::MultichannelSetMalformed
        ));
    }

    #[test]
    fn multichannel_layout_refuses_member_count_mismatch() {
        let mut stream = mono_member(&[1, 2, 3], 0, 3, 0b01);
        stream.extend(mono_member(&[4, 5], 0, 2, 0b10));
        assert!(matches!(
            multichannel_layout(&stream).unwrap_err(),
            Error::MultichannelSampleCountMismatch {
                expected: 3,
                found: 2
            }
        ));
    }

    #[test]
    fn multichannel_false_stereo_member_counts_as_one_channel() {
        // A false-stereo member (wiki bit 30: stereo container, mono data)
        // carries one decoded channel, exactly like a plain mono member.
        // Patch a mono member's flag word: clear bit 2 (mono), set bit 30
        // (false_stereo). The data is still a single channel.
        let total = 2;
        let mut first = mono_member(&[10, 11], 0, total, 0b01);
        {
            let mut f = u32::from_le_bytes([first[24], first[25], first[26], first[27]]);
            f &= !(1 << 2); // clear mono
            f |= 1 << 30; // set false_stereo
            first[24..28].copy_from_slice(&f.to_le_bytes());
        }
        let mut stream = first;
        stream.extend(mono_member(&[20, 21], 0, total, 0b10));

        let decoded = decode_multichannel_stream(&stream).unwrap();
        // 2 channels: false-stereo member (1) + mono member (1).
        assert_eq!(decoded.channels, 2);
        assert_eq!(decoded.samples, vec![10, 20, 11, 21]);
        // Layout agrees.
        assert_eq!(multichannel_layout(&stream).unwrap().channels, 2);
    }

    #[test]
    fn multichannel_too_many_channels_is_refused() {
        // A set whose summed channel count exceeds MAX_MULTICHANNEL_CHANNELS
        // is refused before the interleave buffer is sized. Build a set of
        // MAX + 1 mono members (first / continuations / final).
        let total = 1;
        let cap = MAX_MULTICHANNEL_CHANNELS;
        let mut stream = mono_member(&[0], 0, total, 0b01);
        for _ in 0..(cap - 1) {
            stream.extend(mono_member(&[0], 0, total, 0b00));
        }
        // One more continuation pushes the count to cap + 1 before the final.
        stream.extend(mono_member(&[0], 0, total, 0b00));
        stream.extend(mono_member(&[0], 0, total, 0b10));

        assert!(matches!(
            decode_multichannel_stream(&stream).unwrap_err(),
            Error::MultichannelTooManyChannels(n) if n > cap
        ));
        assert!(matches!(
            multichannel_layout(&stream).unwrap_err(),
            Error::MultichannelTooManyChannels(n) if n > cap
        ));
    }

    #[test]
    fn multichannel_layout_agrees_with_decode_channels() {
        // The header-only layout's channel count matches the decoded one.
        let pcm: Vec<i32> = (0..18).collect(); // 6 channels × 3 frames
        let stream = crate::encode::encode_multichannel_stream(&pcm, 6, 0, 2).unwrap();
        let layout = multichannel_layout(&stream).unwrap();
        let decoded = decode_multichannel_stream(&stream).unwrap();
        assert_eq!(layout.channels, decoded.channels);
        assert_eq!(layout.channels, 6);
    }

    #[test]
    fn decode_member_samples_decodes_a_marked_member_standalone() {
        // A single marked member decodes via decode_member_samples even
        // though decode_samples would refuse it as a MultichannelMember.
        let pcm = [5, -3, 8, 1];
        let block = mono_member(&pcm, 0, 4, 0b01);
        let (parsed, _) = parse_block(&block).unwrap();
        // The public decode_samples refuses the grouped member.
        assert!(matches!(
            parsed.decode_samples().unwrap_err(),
            Error::UnsupportedBlockFeature(UnsupportedBlockFeature::MultichannelMember)
        ));
        // The member path accepts it and reproduces the PCM.
        assert_eq!(parsed.decode_member_samples().unwrap(), pcm.to_vec());
    }
}
