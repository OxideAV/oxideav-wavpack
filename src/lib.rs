//! Pure-Rust WavPack lossless audio codec.
//!
//! **Round 1 — block-header parser.** This release lands the structural
//! 32-byte block-header parser documented in
//! `docs/audio/wavpack/wiki/WavPack.wiki` (block-structure listing).
//! The wiki page is a local snapshot of the multimedia.cx WavPack
//! reference page; this crate's round-1 scope is exactly the fields
//! listed there:
//!
//! * The four-byte `'w','v','p','k'` magic.
//! * The 32-bit little-endian `ck_size` (block size not counting the
//!   magic or this field).
//! * The 16-bit `version` (valid range `0x0402..=0x0410`).
//! * The 8-bit `track_number` and `track_sub_index`.
//! * The 32-bit `total_samples` (with the `0xFFFF_FFFF` "unknown"
//!   sentinel).
//! * The 32-bit `block_index` and `block_samples`.
//! * The 32-bit `flags` word, decoded into a typed [`Flags`] view that
//!   exposes every bit-range named on the wiki "Flags meaning"
//!   listing (bits-per-sample, mono / hybrid / joint-stereo / cross-
//!   channel decorrelation / hybrid-shaping / float / int32 / hybrid
//!   profile / multi-channel start-end markers / left-shift / maximum
//!   magnitude / sampling-rate index / reserved bit 27 / robust block /
//!   hybrid IIR noise shaping / false stereo / low-latency block).
//! * The trailing 32-bit `crc` (preserved verbatim — checksum
//!   verification requires sample decode, which lands in a later
//!   round).
//!
//! No metadata sub-block walking, no decorrelation pass, no entropy
//! decode yet — those land in subsequent rounds against the wiki
//! "Metadata", "Decorrelation terms / weights / samples",
//! "Entropy info" and "Samples coding" sections.
//!
//! ## Clean-room provenance
//!
//! Round 1 was implemented strictly against
//! `docs/audio/wavpack/wiki/WavPack.wiki`. No external library source
//! (libwavpack, wavpack-rs, FFmpeg's `wavpack.c` /
//! `wavpackenc.c`), no archived `old` branch of this crate, and no
//! online resource outside the local docs snapshot was read at any
//! phase.

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

mod block_header;
mod error;

pub use crate::block_header::{
    parse_block_header, Flags, WavPackBlockHeader, HEADER_LEN, MAGIC, MAX_VERSION, MIN_CK_SIZE,
    MIN_VERSION, TOTAL_SAMPLES_UNKNOWN,
};
pub use crate::error::{Error, Result};
