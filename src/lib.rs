//! Pure-Rust WavPack lossless audio codec.
//!
//! **Round 6 — block-header parser + metadata sub-block walker +
//! decorrelation sub-block expanders + entropy-info expander +
//! sample-coding bit reader, run-length decoder & Golomb sample-value
//! reconstruction.**
//! Round 1 landed the structural 32-byte block-header parser
//! documented in `docs/audio/wavpack/wiki/WavPack.wiki` (block-structure
//! listing); round 2 added the metadata sub-block walker following the
//! wiki "Metadata" section; round 3 adds typed expanders for the three
//! decorrelation sub-blocks — `0x02` terms, `0x03` weights, and
//! `0x04` samples — per the wiki "Decorrelation terms",
//! "Decorrelation weights" and "Decorrelation samples" sections;
//! round 4 adds [`expand_entropy`] for the `0x05` entropy-info
//! sub-block (one or two sets of three 16-bit log-packed medians per
//! the wiki "Entropy info" section); round 5 adds the [`BitReader`]
//! primitives (`get_unary` / `get_bit` / `get_bits`) and
//! [`decode_run_length`] — the first half of the wiki "Samples coding"
//! pseudocode (the unary-prefix `n`-decoder with the `n == 16` escape
//! and the adaptive `last_zero` / `last_one` carry); round 6 adds the
//! second (value) half — [`golomb_interval`] and [`decode_sample_value`]
//! — that maps `n` + [`Medians`] onto a `(base, add)` interval and reads
//! the mantissa / sign, leaving the median-update *amount* (an open docs
//! gap) for a later round. See [`expand_terms`], [`expand_weights`],
//! [`expand_samples`], [`expand_entropy`], [`decode_run_length`] and
//! [`decode_sample_value`].
//!
//! Round-1 scope (preserved):
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
//!   listing.
//! * The trailing 32-bit `crc` (preserved verbatim — checksum
//!   verification requires sample decode, which lands in a later
//!   round).
//!
//! Round-2 scope (preserved):
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
//! Round-3 scope adds the decorrelation expanders:
//!
//! * [`expand_terms`] — converts a `0x02` payload into a
//!   [`DecorrelationTerms`] (`terms: Vec<i8>`, `deltas: Vec<u8>`),
//!   one byte → one `(term, delta)` pair per the wiki "lower 5 bits
//!   indicate predictor type, high 3 bits contain delta value"
//!   sentence.
//! * [`expand_weights`] — converts a `0x03` payload into a
//!   [`DecorrelationWeights`] (`weights: Vec<i32>`), applying the
//!   wiki two-line log-pack expansion
//!   (`n = getchar() << 3; if (n > 0) n += (n + 64) >> 7`) to every
//!   byte.
//! * [`expand_samples`] — converts a `0x04` payload into a
//!   [`DecorrelationSamples`] (`samples: Vec<i32>`), reading
//!   little-endian 16-bit words and applying the wiki exponent /
//!   mantissa expansion (mantissa is signed, exponent is biased by
//!   `-9`).
//!
//! Round-4 scope adds the entropy-info expander:
//!
//! * [`expand_entropy`] — converts a `0x05` payload into an
//!   [`EntropyInfo`] (`medians_left: [i32; 3]`,
//!   `medians_right: [i32; 3]`), reading three (mono) or six (stereo)
//!   little-endian 16-bit words through the same log-pack the round-3
//!   sample expander uses. Mono payloads (6 bytes) leave
//!   `medians_right` at `[0; 3]`; stereo payloads (12 bytes) populate
//!   both. Other lengths are rejected as malformed.
//!
//! Round-5 scope adds the sample-coding bit reader and run-length
//! decoder (first half of the wiki "Samples coding" section):
//!
//! * [`BitReader`] — least-significant-bit-first reader over a `0x0A`
//!   payload exposing the three wiki primitives `get_unary()`,
//!   `get_bit()` and `get_bits(n)`. Reads past the buffer report
//!   [`Error::Truncated`].
//! * [`decode_run_length`] — turns the unary prefix (with the
//!   `n == 16` escape) into the halved run-length index `n`, carrying
//!   the adaptive `last_zero` / `last_one` state in [`RunState`].
//!
//! Round-6 scope adds the Golomb *value* half of the same section — the
//! `(base, add)` interval selection plus the mantissa / sign
//! reconstruction — stopping short only of the median adaptation:
//!
//! * [`Medians`] — a channel's three medians (`median[0..=2]`) as the
//!   `0x05` entropy-info expander produces them.
//! * [`golomb_interval`] — pure `n` + [`Medians`] → [`GolombInterval`]
//!   `(base, add)` mapping per the wiki's three-way branch.
//! * [`decode_sample_value`] — reads `getbits(k - 1)` (with `k =
//!   log2(add)` under the wiki-derived bit-length reading), the
//!   `t2 >= ex` extra bit, and the sign, returning the reconstructed
//!   sample. Takes [`Medians`] **by value** and does not mutate them:
//!   the median "increase" / "decrease" *amount* is still an open docs
//!   gap, so the stateful full-payload loop is deferred. The degenerate
//!   `add == 0` (median `1`) interval is rejected via
//!   [`Error::GolombDegenerateInterval`] rather than guessed.
//!
//! Still out of scope (subsequent rounds): the median-adaptation
//! *amount* that turns `decode_sample_value` into a stateful payload
//! loop (blocked on a docs gap), the prediction loop that consumes the
//! decorrelation typed views, float-data / large-or-shifted-int /
//! overflow-bits interpretation, multichannel channel-mask handling,
//! hybrid correction-stream (`.wvc`) pairing, CRC32 verification,
//! encoder.
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
mod decorrelation;
mod entropy;
mod error;
mod metadata;
mod samples;

pub use crate::block_header::{
    parse_block_header, Flags, WavPackBlockHeader, HEADER_LEN, MAGIC, MAX_VERSION, MIN_CK_SIZE,
    MIN_VERSION, TOTAL_SAMPLES_UNKNOWN,
};
pub use crate::decorrelation::{
    expand_samples, expand_terms, expand_weights, DecorrelationSamples, DecorrelationTerms,
    DecorrelationWeights, MAX_DOCUMENTED_TERM, SAMPLE_EXPONENT_BIAS, SAMPLE_ON_WIRE_BYTES,
    TERM_DELTA_BITS, TERM_DELTA_MASK, TERM_PREDICTOR_BITS, TERM_PREDICTOR_MASK,
};
pub use crate::entropy::{
    expand_entropy, EntropyInfo, MEDIANS_PER_CHANNEL, MEDIAN_ON_WIRE_BYTES, MONO_PAYLOAD_BYTES,
    STEREO_PAYLOAD_BYTES,
};
pub use crate::error::{Error, Result};
pub use crate::metadata::{
    parse_metadata_sub_block, walk_metadata, MetadataSubBlock, SubBlockFlags, SubBlockId,
    ID_FLAG_LARGE_SIZE, ID_FLAG_ODD_SIZE, ID_FLAG_OPTIONAL, ID_MASK,
};
pub use crate::samples::{
    decode_run_length, decode_sample_value, golomb_interval, BitReader, GolombInterval, Medians,
    RunState, UNARY_ESCAPE,
};
