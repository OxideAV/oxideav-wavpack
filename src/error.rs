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
            Error::NotImplemented => f.write_str(
                "oxideav-wavpack: clean-room rebuild in progress — see crates/oxideav-wavpack/README.md",
            ),
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;
