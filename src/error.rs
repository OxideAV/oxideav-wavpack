//! Crate-local error type for the WavPack decoder.

/// Errors produced while parsing a WavPack stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// A read did not produce the requested number of bytes — the
    /// buffer ended before the WavPack block header was fully
    /// consumed.
    Truncated,
    /// The block header did not begin with the four-byte ASCII magic
    /// `'w','v','p','k'` documented in `docs/audio/wavpack/wiki/WavPack.wiki`
    /// (block structure list, first field).
    InvalidMagic,
    /// `ck_size` was below the in-header minimum of `24` bytes
    /// (the remaining 24 bytes of the 32-byte header that follow the
    /// `ck_size` field itself per the same wiki listing). A smaller
    /// value cannot describe even an empty block.
    InvalidCkSize(u32),
    /// The 16-bit `version` field fell outside the wiki-documented
    /// range `0x402..=0x410` of valid WavPack v.4 stream versions.
    UnsupportedVersion(u16),
    /// A metadata sub-block declared a word-count whose byte length
    /// (`words * 2`) overflows the platform's `usize`. Reported by
    /// `parse_metadata_sub_block` against a malformed large-flag
    /// sub-block whose 24-bit word-count would exceed available
    /// addressable memory.
    MetadataSubBlockTooLarge(u32),
    /// A metadata sub-block had the `0x40` "odd size" flag set but
    /// declared zero data words. The wiki "Metadata" section
    /// guarantees every metadata block has even length and the
    /// odd-size flag means the **last byte** of the payload is
    /// padding — there has to be at least one byte to be padding.
    MetadataOddSizeWithoutPayload,
    /// A `0x04` decorrelation-samples sub-block had a payload whose
    /// byte count was not a multiple of two. The wiki "Decorrelation
    /// samples" section states every sample is one 16-bit word on
    /// the wire, and the round-2 metadata walker already strips the
    /// optional odd-size padding byte, so any odd byte count here
    /// indicates a malformed sub-block. The contained number is the
    /// observed byte count.
    DecorrelationSamplesOddByteCount(usize),
    /// A `0x05` entropy-info sub-block had a payload whose byte count
    /// matched neither the mono layout (one set of three 16-bit
    /// log-packed medians = 6 bytes) nor the stereo layout (two sets
    /// = 12 bytes) documented in the wiki "Entropy info" section.
    /// The contained number is the observed byte count.
    EntropyInfoLength(usize),
    /// A `0x26` MD5-checksum sub-block had a payload whose byte count
    /// did not match the fixed 16-byte MD5 digest length documented in
    /// the wiki "IDs" listing ("16-byte MD5 sum of raw audio data").
    /// The contained number is the observed byte count.
    Md5ChecksumLength(usize),
    /// A `0x0A` packed-samples sub-block had an odd-length payload. The
    /// clean-room entropy doc
    /// `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 states the
    /// main-bitstream payload "byte length must be even or the block is
    /// rejected" — the reader binds the `0x0A` payload to the per-stream
    /// main bitstream as 16-bit words, so an odd byte count is a
    /// malformed sub-block. (The round-2 walker has already stripped the
    /// optional odd-size *metadata-framing* padding byte, so an odd
    /// length surviving to this check is an odd *payload*, distinct from
    /// the framing pad.) The contained number is the observed byte count.
    PackedSamplesOddLength(usize),
    /// The Golomb mantissa decode of a `0x0A` sample reached `add == 0`
    /// (a median of `1`), where the wiki "Samples coding" pseudocode's
    /// `k = log2(add)` and `getbits(k - 1)` are undefined (`log2(0)` and
    /// `getbits(-1)`). The wiki does not specify the degenerate
    /// single-codeword interval, so the reconstruction is rejected
    /// rather than guessed. The contained value is the offending `add`
    /// (always `0` today; reserved for a future docs revision that
    /// quantifies the degenerate interval).
    GolombDegenerateInterval(i32),
    /// The flat `0x04` decorrelation-samples payload supplied to a
    /// per-term partitioning helper was the wrong total length for the
    /// term list it was paired with. The wiki "Decorrelation samples"
    /// section ties the per-term sample count to the term value
    /// (`6..=12` → `code - 5` samples; `17..=18` → 2 samples). The
    /// helper sums those documented counts across the term list and
    /// reports a mismatch when the flat payload doesn't match.
    /// `expected` is the summed-from-the-term-list count; `actual` is
    /// the flat sample count actually present.
    DecorrelationSampleCountMismatch {
        /// Sum of per-term sample counts derived from the wiki
        /// "Possible predictor values" listing.
        expected: usize,
        /// Sample count actually present in the flat
        /// [`crate::DecorrelationSamples`] payload.
        actual: usize,
    },
    /// A per-term partitioning helper encountered a term whose
    /// per-term decorrelation-sample count is not specified by the wiki
    /// "Decorrelation samples" / "Possible predictor values" sections —
    /// stereo predictors `0..=5`, the reserved `13..=16` range, or any
    /// code outside the documented `0..=18` range. The contained value
    /// is the offending term code; the helper cannot split the flat
    /// payload without a per-term length.
    DecorrelationSampleCountUnspecified(i8),
    /// `parse_block` was given a buffer whose 32-byte fixed header
    /// parses cleanly but whose total length is shorter than the
    /// `8 + ck_size` extent the wiki "Block structure" listing
    /// declares ("32 bits - total block size (not counting this field
    /// or 'wvpk')"). The block header itself was valid, but the
    /// metadata sub-block region the header advertises is partially
    /// outside the buffer. Reported separately from [`Self::Truncated`]
    /// so a streaming caller can distinguish "need more bytes for this
    /// block's payload" (this variant) from "buffer ran out between
    /// blocks" ([`Self::Truncated`] on the next block's header). The
    /// `ck_size` field is the header value verbatim; `available` is
    /// the number of bytes the input buffer actually carried for the
    /// whole block (header + payload bytes that were present), so the
    /// streaming caller can size the next read against
    /// `8 + ck_size - available`.
    CkSizeExceedsBuffer {
        /// `ck_size` advertised by the parsed header.
        ck_size: u32,
        /// Bytes of the block actually carried by the input buffer
        /// (header bytes + however many payload bytes were present).
        available: usize,
    },
    /// The `0x0A` per-sample decode loop reached the wire EOF escape
    /// (`spec/wavpack-entropy-decode.md` §4.2 step 3, "if `cbits == 33`
    /// the word is EOF") inside the unary-prefix escape branch. The
    /// caller stops the per-sample loop on this variant: it is a normal
    /// end-of-stream signal, not a malformed-payload signal.
    EndOfStream,
    /// `decode_packed_samples_mono_from_entropy` was handed an
    /// [`crate::EntropyInfo`] whose channel-0 set carried a negative
    /// seed (`AdaptiveMedians::from_entropy` rejected the `i32 → u32`
    /// reinterpretation per its defensive contract). The contained
    /// values are the offending seed triple verbatim for diagnostics.
    /// Round 201.
    InvalidEntropyInfoForMono,
    /// `decode_packed_samples_stereo_from_entropy` was handed an
    /// [`crate::EntropyInfo`] that is either mono (no right-channel set
    /// on the wire — nothing to drive the right-channel adaptive state)
    /// or carries a negative seed in either set (same defensive
    /// rejection as the mono variant). Round 201.
    InvalidEntropyInfoForStereo,
    /// `WavPackBlock::decode_samples` was called on a block that does
    /// not carry a `0x05` entropy-info sub-block — the medians driving
    /// the round-15/199 `0x0A` per-sample decode must come from the
    /// `0x05` sub-block per the wiki "Entropy info" + "Samples coding"
    /// listings. The block-level composer reports the structural
    /// shortfall here rather than calling the per-sample primitive
    /// with a hand-rolled seed. Round 206.
    BlockMissingEntropyInfo,
    /// `WavPackBlock::decode_samples` was called on a block that does
    /// not carry a `0x0A` packed-samples sub-block — the entropy-coded
    /// audio payload the per-sample loop consumes. The block-level
    /// composer reports the structural shortfall here. Round 206.
    BlockMissingPackedSamples,
    /// `WavPackBlock::decode_samples` was called on a non-audio block
    /// (the wiki "Block structure" listing allows `block_samples == 0`
    /// for metadata-only blocks). The composer cannot return PCM samples
    /// from a block that advertises zero of them. Use
    /// [`crate::WavPackBlockHeader::is_audio_block`] to filter the input
    /// stream when iterating multi-block files. Round 206.
    BlockHasNoAudio,
    /// `WavPackBlock::decode_samples` was called on a block that
    /// carries one of the WavPack v.4 features the round-15/199
    /// per-sample loop does not yet support (hybrid lossy mode,
    /// floating-point or large-integer payload, multichannel
    /// participation, decorrelation pre-pass, low-latency or
    /// experimental gating). Carries a typed
    /// [`crate::UnsupportedBlockFeature`] tag naming the specific
    /// feature so the caller can surface a precise diagnostic. The
    /// per-sample decode primitives themselves remain callable for
    /// callers that have validated the block's feature set themselves.
    /// Round 206.
    UnsupportedBlockFeature(crate::UnsupportedBlockFeature),
    /// An encode-side primitive was asked to represent a value the
    /// current spec §4.2 step 5 interval cannot carry.
    /// [`crate::SampleInterval::encode_value`] /
    /// [`crate::SampleInterval::encode_signed_value`] report the
    /// magnitude against the `[low, high]` interval bounds;
    /// [`crate::SampleInterval::encode_mantissa`] reports the mantissa
    /// `code` against `[0, maxcode]`. Reachable through the packed
    /// encoders only in the 31-bit-mask corner where the zone ladder's
    /// masked interval collapses below the magnitude — exactly the
    /// magnitudes no decode of the same median state could ever
    /// produce. Round 281.
    ValueNotInInterval {
        /// The magnitude (or mantissa code) that does not fit.
        value: u32,
        /// Inclusive lower bound of the target interval.
        low: u32,
        /// Inclusive upper bound of the target interval.
        high: u32,
    },
    /// `WavPackBlock::decode_samples` was called on a block whose
    /// 32-bit `block_samples` header field
    /// (`docs/audio/wavpack/wiki/WavPack.wiki` "Block structure",
    /// "32 bits - samples in this block") exceeds the decoder's
    /// per-block sample ceiling [`crate::MAX_DECODE_SAMPLES_PER_BLOCK`].
    ///
    /// The wiki documents `block_samples` as a bare 32-bit count with no
    /// upper bound, and the spec §4.2 step 1 zero-run fast path lets a
    /// single ~63-bit run word expand to ~`2^31` zero samples — so the
    /// emitted sample count is **not** bounded by the `0x0A` payload
    /// byte length. An attacker can therefore set `block_samples` to a
    /// near-`u32::MAX` value in a tiny block and force the per-sample
    /// loop to grow its output `Vec` to billions of entries (a
    /// multi-gigabyte allocation) before the bitstream genuinely runs
    /// dry. This typed rejection caps the eager decode at a generous
    /// engineering ceiling well above any real WavPack block; it is an
    /// anti-amplification guard, not a spec-mandated limit. The
    /// contained value is the offending `block_samples` field verbatim.
    BlockSamplesTooLarge(u32),
    /// A decorrelation pass list carried a term value outside the
    /// clean-room spec's valid set `{1..8, 17, 18, -1, -2, -3}`
    /// (`docs/audio/wavpack/spec/wavpack-decorrelation.md` §2 / §2.1: a
    /// term of `0`, or any value outside that set, is rejected). The
    /// contained value is the offending term code.
    InvalidDecorrelationTerm(i8),
    /// A negative (cross-channel) decorrelation term `-1`/`-2`/`-3` was
    /// configured for a mono pass. Spec §2.1: "negative terms are
    /// rejected for mono data" — they predict one stereo channel from
    /// the other and have no meaning on a single channel. The contained
    /// value is the offending term code.
    CrossTermOnMono(i8),
    /// The number of decorrelation passes exceeded the spec's
    /// `MAX_NTERMS` ceiling
    /// (`docs/audio/wavpack/spec/wavpack-decorrelation.md` §6: 16). The
    /// contained value is the observed pass count.
    TooManyDecorrelationPasses(usize),
    /// A decorrelation pass did not carry enough seed-history samples to
    /// prime its predictor. Spec §3.6: terms `17`/`18` need 2 seeds,
    /// terms `1..8` need `term` seeds, and cross terms need 1 seed (per
    /// channel). The contained values are the term and the count of
    /// seed samples supplied.
    DecorrelationSeedUnderflow {
        /// The term whose seed history was short.
        term: i8,
        /// The number of seed samples supplied for that term/channel.
        supplied: usize,
    },
    /// The `0x03` decorrelation-weights payload did not carry one weight
    /// per decorrelation pass (mono) — the spec §3.6 / wiki "Each
    /// decorrelation term should have one or two weights depending on
    /// channels". For a mono block the weight count must equal the term
    /// count exactly; the assembler reports the expected (term) count and
    /// the observed weight count here.
    DecorrelationWeightCountMismatch {
        /// Number of decorrelation terms (`0x02`) — the expected weight
        /// count for a mono block.
        expected: usize,
        /// Number of weights actually present in the `0x03` payload.
        actual: usize,
    },
    /// A block carried a `0x03` decorrelation-weights and/or `0x04`
    /// decorrelation-samples sub-block without the `0x02`
    /// decorrelation-terms sub-block that names the passes those weights
    /// and seeds belong to. Spec §1 lists all three as the decorrelation
    /// configuration; the assembler cannot reconstruct passes from
    /// weights/seeds alone (the term list drives the per-pass arithmetic
    /// and the per-term seed/weight counts).
    DecorrelationTermsMissing,
    /// A hybrid correction-fold was requested with a correction buffer
    /// whose length does not match the decoded lossy buffer. The
    /// decorrelation-spec §4.1 fold reads exactly one correction residual
    /// per decoded sample, so the two buffers must be the same length.
    HybridCorrectionLengthMismatch {
        /// Number of samples in the decoded lossy buffer.
        lossy: usize,
        /// Number of correction residuals supplied.
        correction: usize,
    },
    /// A hybrid correction-fold was requested on a block whose flags
    /// select a fold placement the §4.1-documented raw add does not cover:
    /// the `CROSS_DECORR` (`0x20`) *pre*-decorrelation fold (which must run
    /// before the decorrelation passes, not on the reconstructed output),
    /// or the `HYBRID_SHAPE` (`0x40`) / `NEW_SHAPING` (`0x2000_0000`)
    /// noise-shaped fold (whose `read_shaping_info` state layout is a
    /// documented gap). The raw post-decorrelation fold is correct only
    /// for the non-cross, non-shaped case.
    HybridFoldPlacementUnsupported,
    /// A block-encode was requested with an empty PCM buffer. A WavPack
    /// audio block carries `block_samples >= 1` (the wiki "samples in
    /// this block" field is non-zero for an audio block); an encoder
    /// emitting a zero-sample audio block has nothing to pack.
    EncodeEmptyAudio,
    /// A stereo block-encode was requested with an odd number of
    /// interleaved samples. The stereo wire layout is whole `[L, R]`
    /// pairs, so the interleaved buffer length must be even.
    EncodeStereoOddLength(usize),
    /// A block-encode produced a metadata region whose byte length
    /// overflows the 24-bit `ck_size` field (the wiki "total block size"
    /// header word). A single block cannot carry this much data; split
    /// the PCM across multiple blocks.
    EncodeBlockTooLarge(usize),
    /// A left-shift (sub-byte bit-depth) block-encode was requested with
    /// a `left_shift` of `0`. The shifted encoders are for genuine
    /// non-whole-byte depths (12-bit, 20-bit, …); a shift of `0` is the
    /// whole-byte case the plain encoders already cover.
    EncodeLeftShiftZero,
    /// A left-shift block-encode was given a sample whose low `left_shift`
    /// bits are non-zero. The decoder reconstructs the container value as
    /// `narrow << left_shift`, so the input must already be a multiple of
    /// `2^left_shift` (its low bits zero) or the encode would not be
    /// lossless. The offending sample value is carried for diagnostics.
    EncodeLeftShiftLosesData(i32),
    /// A decorrelation-pass serializer was given a working weight that no
    /// `0x03` stored byte expands back to. The §3.6 weight store is one
    /// signed byte per pass per channel expanded through the documented
    /// log-pack (`byte * 8`, plus `(n + 64) >> 7` when positive), so only
    /// 256 working values are representable; an encoder must quantize its
    /// working weight (see [`crate::quantize_weight`]) before serializing
    /// so the decoder reconstructs the identical starting state. Carries
    /// the unrepresentable weight.
    EncodeWeightNotRepresentable(i32),
    /// A decorrelation-pass serializer was given a seed-history sample
    /// that no `0x04` stored 16-bit log-word expands back to. The §3.6
    /// seed store is a signed-log2 word (8-bit signed mantissa, shifted
    /// exponent), so a seed with more than 8 significant bits must be
    /// quantized (see [`crate::quantize_seed_sample`]) before serializing.
    /// Carries the unrepresentable seed value.
    EncodeSeedNotRepresentable(i32),
    /// A decorrelation-pass serializer was given a per-pass `delta`
    /// outside the 3-bit on-wire field. The `0x02` term byte stores the
    /// weight-adaptation step in its top 3 bits (spec §2.1,
    /// `delta = (byte >> 5) & 0x7`), so only `0..=7` is representable.
    /// Carries the out-of-range delta.
    EncodeDeltaOutOfRange(i32),
    /// A multichannel set was malformed: a member block carried the wiki
    /// bit-12 "final block of set" marker without any preceding bit-11
    /// "first block of set" marker having opened a set (or the stream
    /// ended mid-set). The grouping walker in
    /// [`crate::decode_multichannel_stream`] tracks an open set across
    /// member blocks; a stray final-marker or an unterminated set is a
    /// structural inconsistency in the wiki bits-11..=12 grouping.
    MultichannelSetMalformed,
    /// The member blocks of a multichannel set disagreed on
    /// `block_samples` (the wiki "samples in this block" header word). All
    /// members of one set cover the **same** sample range — they are the
    /// channels of the same frames — so a per-member mismatch means the
    /// blocks do not form a coherent set. Carries `(expected, found)`.
    MultichannelSampleCountMismatch {
        /// `block_samples` of the set's first (bit-11) member.
        expected: u32,
        /// `block_samples` of the offending later member.
        found: u32,
    },
    /// A multichannel set's summed channel count exceeded the supported
    /// maximum ([`crate::MAX_MULTICHANNEL_CHANNELS`]). A set is a run of
    /// 1- or 2-channel member blocks; the sum is the multichannel width.
    /// An absurd width (a malformed stream chaining unbounded members)
    /// is refused before it sizes the interleave buffer. Carries the
    /// channel count reached.
    MultichannelTooManyChannels(usize),
    /// A `.wvc` correction-file audio block could not be aligned with a
    /// `.wv` main-file audio block: walking both audio-block chains in
    /// order, a correction block's wiki "offset in samples for current
    /// block" header word was **behind** the next main block's — i.e.
    /// the correction file carries a block the main file never had (or
    /// the chains are out of order). Carries `(main, correction)` block
    /// indices at the point of divergence.
    CorrectionIndexMismatch {
        /// `block_index` of the next unpaired main audio block.
        main: u32,
        /// `block_index` of the orphaned correction audio block.
        correction: u32,
    },
    /// An index-aligned `.wv`/`.wvc` audio-block pair disagreed on
    /// `block_samples` (the wiki "samples in this block" header word).
    /// A correction stream corrects the main stream sample-for-sample
    /// (spec §4.1: two residuals per sample position), so both sides of
    /// a pair must cover the identical range. Carries
    /// `(main, correction)` sample counts.
    CorrectionSampleCountMismatch {
        /// `block_samples` of the main block.
        main: u32,
        /// `block_samples` of the correction block.
        correction: u32,
    },
    /// An index-aligned `.wv`/`.wvc` audio-block pair disagreed on
    /// channel shape (the wiki bit-2 mono flag). The correction words
    /// pair one-to-one with the main stream's channel samples, so a
    /// shape flip mid-pairing is structural corruption. Carries the
    /// pair's shared `block_index`.
    CorrectionShapeMismatch(u32),
    /// The `.wvc` buffer carried audio blocks beyond the last `.wv`
    /// audio block — corrections for samples the main file does not
    /// have. Carries the first surplus correction block's
    /// `block_index`.
    CorrectionBlockSurplus(u32),
    /// Reserved placeholder for API surface not yet wired by the
    /// clean-room rebuild rounds.
    NotImplemented,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Truncated => f.write_str("oxideav-wavpack: stream truncated"),
            Error::InvalidMagic => f.write_str("oxideav-wavpack: invalid block magic (expected wvpk)"),
            Error::InvalidCkSize(v) => {
                write!(f, "oxideav-wavpack: ck_size {v} is below the 24-byte header minimum")
            }
            Error::UnsupportedVersion(v) => {
                write!(f, "oxideav-wavpack: unsupported version 0x{v:04x} (expected 0x0402..=0x0410)")
            }
            Error::MetadataSubBlockTooLarge(words) => {
                write!(
                    f,
                    "oxideav-wavpack: metadata sub-block size {words} words overflows usize byte length"
                )
            }
            Error::MetadataOddSizeWithoutPayload => f.write_str(
                "oxideav-wavpack: metadata sub-block declared 0x40 odd-size flag with zero data words",
            ),
            Error::DecorrelationSamplesOddByteCount(n) => write!(
                f,
                "oxideav-wavpack: 0x04 decorrelation-samples payload has {n} bytes (not a multiple of 2)"
            ),
            Error::EntropyInfoLength(n) => write!(
                f,
                "oxideav-wavpack: 0x05 entropy-info payload has {n} bytes (expected 6 for mono or 12 for stereo)"
            ),
            Error::Md5ChecksumLength(n) => write!(
                f,
                "oxideav-wavpack: 0x26 MD5-checksum payload has {n} bytes (expected 16)"
            ),
            Error::PackedSamplesOddLength(n) => write!(
                f,
                "oxideav-wavpack: 0x0A packed-samples payload has {n} bytes (must be even per spec \u{00A7}1)"
            ),
            Error::GolombDegenerateInterval(add) => write!(
                f,
                "oxideav-wavpack: 0x0A Golomb mantissa hit add={add} (median 1); log2(add)/getbits(-1) undefined by the wiki"
            ),
            Error::DecorrelationSampleCountMismatch { expected, actual } => write!(
                f,
                "oxideav-wavpack: 0x04 decorrelation-samples flat length {actual} does not match the {expected} samples summed from the 0x02 term list"
            ),
            Error::DecorrelationSampleCountUnspecified(code) => write!(
                f,
                "oxideav-wavpack: 0x02 term code {code} has no wiki-documented per-term sample count (stereo / reserved / undocumented)"
            ),
            Error::CkSizeExceedsBuffer { ck_size, available } => write!(
                f,
                "oxideav-wavpack: parse_block needs {needed} bytes (8 + ck_size {ck_size}) but the buffer only carries {available}",
                needed = 8u64 + *ck_size as u64,
            ),
            Error::EndOfStream => f.write_str(
                "oxideav-wavpack: 0x0A per-sample loop reached the wire EOF escape (cbits == 33 inside the unary-prefix escape branch)",
            ),
            Error::InvalidEntropyInfoForMono => f.write_str(
                "oxideav-wavpack: decode_packed_samples_mono_from_entropy: channel-0 EntropyInfo set has a negative seed (i32 -> u32 rejected)",
            ),
            Error::InvalidEntropyInfoForStereo => f.write_str(
                "oxideav-wavpack: decode_packed_samples_stereo_from_entropy: EntropyInfo is mono or carries a negative per-channel seed",
            ),
            Error::BlockMissingEntropyInfo => f.write_str(
                "oxideav-wavpack: WavPackBlock::decode_samples: block carries no 0x05 entropy-info sub-block",
            ),
            Error::BlockMissingPackedSamples => f.write_str(
                "oxideav-wavpack: WavPackBlock::decode_samples: block carries no 0x0A packed-samples sub-block",
            ),
            Error::BlockHasNoAudio => f.write_str(
                "oxideav-wavpack: WavPackBlock::decode_samples: block_samples == 0 (metadata-only block, no audio)",
            ),
            Error::UnsupportedBlockFeature(feat) => write!(
                f,
                "oxideav-wavpack: WavPackBlock::decode_samples: block uses an unsupported feature ({feat})"
            ),
            Error::ValueNotInInterval { value, low, high } => write!(
                f,
                "oxideav-wavpack: encode: value {value} is outside the spec §4.2 step 5 interval [{low}, {high}]"
            ),
            Error::BlockSamplesTooLarge(n) => write!(
                f,
                "oxideav-wavpack: block_samples {n} exceeds the per-block decode ceiling \
                 (anti-amplification guard)"
            ),
            Error::InvalidDecorrelationTerm(t) => write!(
                f,
                "oxideav-wavpack: decorrelation term {t} is outside the valid set {{1..8, 17, 18, -1, -2, -3}}"
            ),
            Error::CrossTermOnMono(t) => write!(
                f,
                "oxideav-wavpack: cross-channel decorrelation term {t} is invalid for mono data"
            ),
            Error::TooManyDecorrelationPasses(n) => write!(
                f,
                "oxideav-wavpack: {n} decorrelation passes exceeds the MAX_NTERMS ceiling of 16"
            ),
            Error::DecorrelationSeedUnderflow { term, supplied } => write!(
                f,
                "oxideav-wavpack: decorrelation term {term} needs more seed-history samples than the {supplied} supplied"
            ),
            Error::DecorrelationWeightCountMismatch { expected, actual } => write!(
                f,
                "oxideav-wavpack: 0x03 decorrelation-weights payload carried {actual} weights for {expected} mono passes"
            ),
            Error::DecorrelationTermsMissing => f.write_str(
                "oxideav-wavpack: block carries 0x03/0x04 decorrelation metadata without the 0x02 terms sub-block",
            ),
            Error::HybridCorrectionLengthMismatch { lossy, correction } => write!(
                f,
                "oxideav-wavpack: hybrid correction-fold buffer mismatch — {correction} correction residuals for {lossy} decoded samples (must be equal)"
            ),
            Error::HybridFoldPlacementUnsupported => f.write_str(
                "oxideav-wavpack: hybrid correction-fold requested on a CROSS_DECORR / noise-shaped block — only the non-cross, non-shaped post-decorrelation raw fold is supported",
            ),
            Error::EncodeEmptyAudio => f.write_str(
                "oxideav-wavpack: encode_block: empty PCM buffer (an audio block carries block_samples >= 1)",
            ),
            Error::EncodeStereoOddLength(n) => write!(
                f,
                "oxideav-wavpack: encode_block: stereo PCM has {n} interleaved samples (must be an even count of [L, R] pairs)"
            ),
            Error::EncodeBlockTooLarge(n) => write!(
                f,
                "oxideav-wavpack: encode_block: metadata region is {n} bytes, overflowing the 24-bit ck_size field"
            ),
            Error::EncodeLeftShiftZero => f.write_str(
                "oxideav-wavpack: encode_block_*_shifted: left_shift == 0 (use the plain whole-byte encoder)",
            ),
            Error::EncodeLeftShiftLosesData(v) => write!(
                f,
                "oxideav-wavpack: encode_block_*_shifted: sample {v} has non-zero low bits that left_shift would drop (input must be a multiple of 2^left_shift)"
            ),
            Error::EncodeWeightNotRepresentable(w) => write!(
                f,
                "oxideav-wavpack: serialize: working weight {w} is not a 0x03 stored-byte expansion (quantize via quantize_weight first)"
            ),
            Error::EncodeSeedNotRepresentable(v) => write!(
                f,
                "oxideav-wavpack: serialize: seed sample {v} is not a 0x04 log-word expansion (quantize via quantize_seed_sample first)"
            ),
            Error::EncodeDeltaOutOfRange(d) => write!(
                f,
                "oxideav-wavpack: serialize: pass delta {d} does not fit the 3-bit 0x02 term-byte field (0..=7)"
            ),
            Error::MultichannelSetMalformed => f.write_str(
                "oxideav-wavpack: malformed multichannel set (stray final-block marker or unterminated set; wiki flag bits 11..=12)",
            ),
            Error::MultichannelSampleCountMismatch { expected, found } => write!(
                f,
                "oxideav-wavpack: multichannel set members disagree on block_samples (expected {expected}, found {found})"
            ),
            Error::MultichannelTooManyChannels(n) => write!(
                f,
                "oxideav-wavpack: multichannel set channel count {n} exceeds the supported maximum"
            ),
            Error::CorrectionIndexMismatch { main, correction } => write!(
                f,
                "oxideav-wavpack: correction-file block_index {correction} has no main-file twin (next main audio block is at {main})"
            ),
            Error::CorrectionSampleCountMismatch { main, correction } => write!(
                f,
                "oxideav-wavpack: correction-file block disagrees on block_samples (main {main}, correction {correction})"
            ),
            Error::CorrectionShapeMismatch(idx) => write!(
                f,
                "oxideav-wavpack: correction-file block at block_index {idx} disagrees on the mono flag"
            ),
            Error::CorrectionBlockSurplus(idx) => write!(
                f,
                "oxideav-wavpack: correction file carries audio blocks past the end of the main file (first surplus at block_index {idx})"
            ),
            Error::NotImplemented => f.write_str(
                "oxideav-wavpack: clean-room rebuild in progress — see crates/oxideav-wavpack/README.md",
            ),
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;
