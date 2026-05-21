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
            Error::NotImplemented => f.write_str(
                "oxideav-wavpack: clean-room rebuild in progress — see crates/oxideav-wavpack/README.md",
            ),
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;
