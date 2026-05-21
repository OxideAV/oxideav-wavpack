//! Pure-Rust WavPack lossless audio codec.
//!
//! **Round 2 — block-header parser + metadata sub-block walker.**
//! Round 1 landed the structural 32-byte block-header parser
//! documented in `docs/audio/wavpack/wiki/WavPack.wiki`
//! (block-structure listing). Round 2 adds the metadata sub-block
//! walker that follows the wiki "Metadata" section: ID byte +
//! 1- or 3-byte size field + payload, repeated to the end of the
//! block's `ck_size`-bounded region. See [`walk_metadata`] and
//! [`MetadataSubBlock`].
//!
//! Round-1 scope (preserved from the prior release):
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
//! Round-2 scope adds the metadata-walking API:
//!
//! * [`walk_metadata`] — consumes a byte slice (the post-header
//!   payload from [`parse_block_header`]) and returns a
//!   `Vec<MetadataSubBlock>` of typed `(SubBlockId, payload)` pairs.
//! * [`parse_metadata_sub_block`] — single-step walker the caller
//!   can drive themselves when validating against `ck_size`.
//! * [`SubBlockId`] — typed enum naming every ID listed by the
//!   wiki "IDs" section (`0x00..=0x0D` + `0x20..=0x27`). Unknown
//!   IDs are surfaced as `Unknown(u8)` rather than rejected.
//! * [`SubBlockFlags`] — typed view of the `0x20` / `0x40` /
//!   `0x80` flag triple decoded from the on-disk ID byte.
//!
//! Still out of scope (subsequent rounds): decorrelation
//! deserialisation, entropy decode of the `0x0A` packed-samples
//! sub-block, float-data / large-or-shifted-int / overflow-bits
//! interpretation, multichannel channel-mask handling, hybrid
//! correction-stream (`.wvc`) pairing, CRC32 verification, encoder.
//!
//! ## Clean-room provenance
//!
//! All work in this crate has been implemented strictly against
//! `docs/audio/wavpack/wiki/WavPack.wiki`. No external library source
//! (libwavpack, wavpack-rs, FFmpeg's `wavpack.c` /
//! `wavpackenc.c`), no archived `old` branch of this crate, and no
//! online resource outside the local docs snapshot was read at any
//! phase.

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

mod block_header;
mod error;
mod metadata;

pub use crate::block_header::{
    parse_block_header, Flags, WavPackBlockHeader, HEADER_LEN, MAGIC, MAX_VERSION, MIN_CK_SIZE,
    MIN_VERSION, TOTAL_SAMPLES_UNKNOWN,
};
pub use crate::error::{Error, Result};
pub use crate::metadata::{
    parse_metadata_sub_block, walk_metadata, MetadataSubBlock, SubBlockFlags, SubBlockId,
    ID_FLAG_LARGE_SIZE, ID_FLAG_ODD_SIZE, ID_FLAG_OPTIONAL, ID_MASK,
};
