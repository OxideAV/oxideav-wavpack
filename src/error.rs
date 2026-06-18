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
            Error::NotImplemented => f.write_str(
                "oxideav-wavpack: clean-room rebuild in progress — see crates/oxideav-wavpack/README.md",
            ),
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;
