//! Pure-Rust WavPack lossless audio codec.
//!
//! **Round 219 — multi-block stream iteration on top of the round-13
//! [`parse_block`] composer, lifting the wiki "WavPack file consists of
//! blocks each beginning with 'wvpk'" file-format sentence into typed
//! public API. New [`BlockIter`] is a `Clone`-able, `FusedIterator`-
//! compliant lazy iterator yielding `Result<WavPackBlock<'_>>` over the
//! chained-block byte buffer. New free function [`iter_blocks`] is the
//! call-shape twin of [`BlockIter::new`]. New [`parse_blocks`] eagerly
//! collects the iterator into `Vec<WavPackBlock<'_>>`. New
//! [`block_count`] counts blocks without retaining them.
//! [`total_block_samples`] sums the wiki "samples in this block" field
//! across an already-parsed block list, returning `u64` so a
//! 4-GiB-plus stream's sample count does not overflow `u32`. The
//! iterator fuses on the first error so the caller can `?`-bubble the
//! failure without re-encountering it; [`BlockIter::remaining`]
//! continues to point at the malformed block's first byte for offset
//! diagnostics. No new error variants and no docs-gap-blocked surface
//! touched. 18 new tests (337 total).**
//!
//! **Round 214 — block-level discovery / accessor sweep on
//! [`WavPackBlock`] pairing the round-13 [`parse_block`] aggregate with
//! the typed views the existing free finders already build. Adds four
//! header-passthrough accessors ([`WavPackBlock::flags`] /
//! [`WavPackBlock::block_samples`] / [`WavPackBlock::block_index`] /
//! [`WavPackBlock::is_audio_block`]), six presence predicates
//! ([`WavPackBlock::has_entropy_info`] /
//! [`WavPackBlock::has_packed_samples`] /
//! [`WavPackBlock::has_md5_checksum`] /
//! [`WavPackBlock::has_riff_header`] /
//! [`WavPackBlock::has_riff_trailer`] /
//! [`WavPackBlock::has_multichannel_info`]), six borrow finders
//! ([`WavPackBlock::find_sub_block`] /
//! [`WavPackBlock::find_entropy_info_sub_block`] /
//! [`WavPackBlock::find_md5_checksum_sub_block`] /
//! [`WavPackBlock::find_multichannel_info_sub_block`] /
//! [`WavPackBlock::find_riff_header_sub_block`] /
//! [`WavPackBlock::find_riff_trailer_sub_block`]) and three typed
//! extractors ([`WavPackBlock::packed_samples`] →
//! `Option<PackedSamples<'a>>`, [`WavPackBlock::entropy_info`] →
//! `Result<Option<EntropyInfo>>`, [`WavPackBlock::md5_checksum`] →
//! `Result<Option<Md5Checksum>>`). No new error variants and no
//! docs-gap-blocked surface touched. 24 new tests (319 total) pin
//! present / absent / malformed branches across each accessor and an
//! end-to-end pairing with the round-206 [`WavPackBlock::decode_samples`]
//! composer.**
//!
//! **Round 206 — block-level [`WavPackBlock::decode_samples`] composer
//! turning the round-13 [`parse_block`] aggregate into PCM samples in
//! one call. Chains [`find_entropy_info`] + [`expand_entropy`] +
//! [`find_packed_samples`] through the round-201
//! [`decode_packed_samples_mono_from_entropy`] /
//! [`decode_packed_samples_stereo_from_entropy`] wrappers depending on
//! the new [`Flags::is_block_data_mono`] accessor (the union of wiki
//! bit 2 `mono` and wiki bit 30 `false_stereo`). Returns a `Vec<i32>`
//! of `block_samples` mono samples or `block_samples * 2` interleaved
//! stereo samples. New [`UnsupportedBlockFeature`] enum names the
//! seven gated cases ([`UnsupportedBlockFeature::Hybrid`] /
//! [`UnsupportedBlockFeature::FloatData`] /
//! [`UnsupportedBlockFeature::Int32Mode`] /
//! [`UnsupportedBlockFeature::MultichannelMember`] /
//! [`UnsupportedBlockFeature::Decorrelation`] /
//! [`UnsupportedBlockFeature::LowLatencyBlock`] /
//! [`UnsupportedBlockFeature::RobustBlock`]), surfaced through
//! [`Error::UnsupportedBlockFeature`]. New structural errors
//! [`Error::BlockHasNoAudio`] / [`Error::BlockMissingEntropyInfo`] /
//! [`Error::BlockMissingPackedSamples`] cover the
//! "block-doesn't-carry-what-the-composer-needs" shortfalls. New
//! [`WavPackBlock::has_decorrelation`] predicate detects the presence
//! of any `0x02` / `0x03` / `0x04` decorrelation sub-block. New
//! [`Flags::is_block_data_stereo`] is the inverse of the new mono
//! accessor. 23 new tests prove each gate fires and the two happy
//! paths return the expected PCM zero(s) from a synthesised block.
//! 295 tests pass (up from 272).**
//!
//! **Round 199 — stereo per-sample decode loop wiring the
//! `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §2 channel-
//! alternation rule. Adds [`StereoDecodeState`] (per-channel [`RunState`]
//! plus stream-level zero-run debt + `next_channel` parity cursor),
//! [`decode_sample_stateful_stereo`] (one stereo sample per call,
//! dispatching to the channel selected by sample-index parity; the
//! zero-run fast path is gated on BOTH channels' `median[0] <= 1` and
//! resets BOTH channels' medians on a non-zero run per spec §4.2 step
//! 1), and [`decode_packed_samples_stereo`] (end-to-end `frames`
//! → `Vec<i32>` of `frames * 2` interleaved (L,R,L,R,…) PCM samples).
//! 13 new tests prove bit-exact round-trip via per-channel adapt
//! simulators across zone-1 mixed sequences, distinct per-channel
//! seeds, mixed-zones, negative-sign reconstruction, the
//! end-to-end loop wrapper, stereo zero-run BOTH-channel reset +
//! drain across parity, the BOTH-channel zero-run gate (one-channel
//! eligibility rejected), per-channel holding-state independence,
//! truncation cursor preservation, and the EOF escape. 252 tests
//! pass (up from 239).**
//!
//! **Round 15 — stateful per-sample `0x0A` decode loop wiring the
//! staged `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §3 +
//! §3.2 + §4.2 end to end via [`decode_sample_stateful`] (the
//! per-sample primitive) and [`decode_packed_samples_mono`] (the
//! end-to-end loop returning a `Vec<i32>` of mono PCM samples). The
//! per-sample primitive runs the full spec §4.2 sequence: optional
//! §4.2 step 1 zero-run fast path (gated on `get_med(0) <= 1` and no
//! holding bits, carrying `zero_run_pending` across calls in
//! [`DecodeState`]); §4.2 steps 2 + 3 unary prefix with the
//! `LIMIT_ONES = 16` escape and the `cbits == 33` EOF marker (new
//! [`Error::EndOfStream`]); §4.2 step 4 holding-bit fold (via the
//! wiki-compressed [`RunState`] embedded in [`DecodeState::run`]);
//! §4.2 step 5 31-bit-masked `(low, high)` interval; §3.2 per-zone
//! median adaptation BEFORE the mantissa read; §4.2 step 6 first
//! paragraph truncated-binary mantissa; §4.2 step 7 sign bit. New
//! constants [`ESCAPE_EOF_CBITS`] / [`RUN_ESCAPE_CAP`] /
//! [`INTERVAL_MASK_31`] name the spec literals. Hybrid mode
//! (`error_limit != 0`, spec §4.2 step 6 second paragraph) and
//! multi-channel decoding stay out of scope. 18 new tests prove
//! bit-exact round-trip via a spec-derived inverse encoder helper.**
//!
//! **Round 13 — block-header parser + metadata sub-block walker +
//! decorrelation sub-block expanders + entropy-info expander +
//! sample-coding bit reader, run-length decoder, Golomb sample-value
//! reconstruction & single-call per-sample decode + entropy→median
//! bridge + header-accessor coverage + `TermKind` classifier +
//! `DecorrelationTerms` accessors + metadata-sub-block payload-kind
//! predicates + walker finders + MD5 typed view + per-term
//! `decorrelation_sample_count` + flat-payload partitioner + `0x0A`
//! `PackedSamples` typed view + `BitReader` position accessors +
//! channel-indexed `EntropyInfo` / `Medians::from_entropy` bridges +
//! end-to-end `parse_block` / `WavPackBlock` aggregate + `BitReader`
//! non-mutating `peek_bit` / `peek_bits` / `peek_unary` and bulk
//! `skip_bits` advance.**
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
//! Round-7 scope joins the two halves into a single per-sample call and
//! bridges the round-4 entropy-info output into the round-6 median set:
//!
//! * [`decode_sample`] — runs the whole wiki "Samples coding" per-sample
//!   pseudocode in one call: [`decode_run_length`] (with its adaptive
//!   [`RunState`] carry) followed by [`decode_sample_value`]. Still takes
//!   [`Medians`] by value and does not mutate them — the median
//!   adaptation amount remains the open docs gap — so it is a
//!   single-sample primitive, not yet the full payload loop.
//! * [`Medians::from_entropy_left`] / [`Medians::from_entropy_right`] —
//!   pull a channel's three medians straight out of an
//!   [`EntropyInfo`] so the round-4 expander output feeds the round-6
//!   Golomb decoder without the caller re-typing the array.
//!
//! Round-9 scope (the previous header-accessor round 8 is preserved) adds
//! typed classification on top of the round-3 / round-2 expander output —
//! still no bit-stream advancement past the docs-gap line:
//!
//! * [`TermKind`] — classifies a decorrelation predictor code per the
//!   wiki "Possible predictor values" listing; `is_implemented()` and
//!   `previous_samples()` surface the wiki's "only predictors 2-4 are
//!   implemented" and "6-12 uses 1-7 samples" narrowings.
//! * [`DecorrelationTerms`] accessors `len`/`is_empty`/`kind_at`/
//!   `iter_kinds`/`all_implemented`/`has_reserved` classify the
//!   round-3 term list without re-walking the bytes.
//! * [`weights_per_term`] — wiki "Each decorrelation term should have
//!   one or two weights depending on channels" mono/stereo split.
//! * [`MetadataSubBlock`] payload-kind predicates `is_optional` /
//!   `is_decorrelation_payload` / `is_correction_payload` /
//!   `is_audio_payload` / `is_riff_payload` group the round-2 walker
//!   output for callers picking the decorrelation triple or the audio
//!   stream out of a walk.
//!
//! Round-12 scope adds the typed `0x0A` packed-samples view, the
//! [`BitReader`] position accessors and the channel-indexed
//! [`Medians::from_entropy`] / [`EntropyInfo::medians_for_channel`]
//! bridges — non-prediction-loop elaborations of the existing typed
//! views while the median-adaptation amount stays a docs gap:
//!
//! * [`PackedSamples`] — typed view of the `0x0A` packed-samples
//!   sub-block payload (the entropy-coded audio bitstream the wiki
//!   "Samples coding" section consumes). Carries the borrowed payload
//!   bytes; exposes [`PackedSamples::bytes`] / [`PackedSamples::len`] /
//!   [`PackedSamples::is_empty`] introspection and a
//!   [`PackedSamples::bit_reader`] factory that yields a fresh
//!   [`BitReader`] positioned at bit 0 for feeding
//!   [`decode_run_length`] / [`decode_sample_value`] / [`decode_sample`].
//! * [`expand_packed_samples`] — the round-2 walker output ↦ typed view
//!   bridge for the `0x0A` ID (analogous to [`expand_samples`] /
//!   [`expand_entropy`] for `0x04` / `0x05`, but a typed wrap rather
//!   than a byte-by-byte decode because the wiki places no internal
//!   structure on the `0x0A` payload).
//! * [`find_packed_samples`] — walker convenience finder returning a
//!   [`PackedSamples`] directly (the typed counterpart to
//!   [`find_audio_payload`]).
//! * [`BitReader::byte_position`] / [`BitReader::bit_position`] /
//!   [`BitReader::bits_consumed`] — cursor accessors for callers that
//!   want to log the position before a truncation error fires or
//!   resume from a known offset against a fresh reader.
//! * [`EntropyInfo::is_stereo`] / [`EntropyInfo::channels`] /
//!   [`EntropyInfo::medians_for_channel`] — typed channel introspection
//!   pinning the wiki "one or two sets of medians" sentence as `1` or
//!   `2` populated sets, with a channel-indexed median getter that
//!   returns `None` for out-of-range indices and for the right channel
//!   on a mono payload.
//! * [`Medians::from_entropy`] — channel-indexed bridge over
//!   [`EntropyInfo`] returning `Some(medians)` for `0` / `1` (the
//!   latter only on stereo) so callers iterating per-channel medians
//!   skip the mono / stereo branch.
//!
//! Round-11 scope adds the per-term decorrelation-sample-count helper
//! and the flat-payload partitioner, both derived from the wiki
//! "Decorrelation samples" / "Possible predictor values" sections:
//!
//! * [`decorrelation_sample_count`] / [`TermKind::decorrelation_sample_count`]
//!   — `Some(code - 5)` for the `6..=12` sample-based codes (one seed
//!   sample per previous-sample slot), `Some(2)` for the `17..=18`
//!   two-sample codes, and `None` for stereo `0..=5`, the reserved
//!   `13..=16` range, and undocumented codes (per-term count not given
//!   by the wiki).
//! * [`DecorrelationTerms::expected_decorrelation_sample_count`] sums
//!   the above across a term list and short-circuits to `None` on
//!   any docs-gap code.
//! * [`partition_decorrelation_samples`] splits the flat
//!   [`DecorrelationSamples`] list `expand_samples` produces into one
//!   `Vec<i32>` per term, refusing docs-gap codes via
//!   [`Error::DecorrelationSampleCountUnspecified`] and length
//!   mismatches via [`Error::DecorrelationSampleCountMismatch`].
//! * [`MAX_DECORRELATION_SAMPLES_PER_TERM`] (= 16) surfaces the wiki
//!   "up to 16 samples" upper bound for callers checking future docs
//!   additions against it.
//!
//! Round-13 scope adds the end-to-end [`parse_block`] composer and the
//! non-mutating [`BitReader::peek_bit`] / [`BitReader::peek_bits`] /
//! [`BitReader::peek_unary`] look-ahead primitives plus the bulk
//! [`BitReader::skip_bits`] advance — both groups stay clear of the
//! median-adaptation docs gap and elaborate the existing structural /
//! bit-level surfaces:
//!
//! * [`parse_block`] combines round 1's
//!   [`parse_block_header`] with round 2's [`walk_metadata`], returning
//!   a [`WavPackBlock`] aggregate (header + parsed sub-blocks) and the
//!   tail bytes ready for the next block in a multi-block `.wv` file.
//!   The new [`Error::CkSizeExceedsBuffer`] variant reports the
//!   distinct "buffer ran out mid-payload" case so a streaming caller
//!   knows how many more bytes to read.
//! * [`BitReader::peek_bit`] / [`BitReader::peek_bits`] /
//!   [`BitReader::peek_unary`] read a single bit, a multi-bit value or
//!   a unary run-length without consuming the bits — implemented by
//!   reading from a clone, so the wiki bit-order rules carry through
//!   unchanged.
//! * [`BitReader::skip_bits`] advances the reader by an arbitrary count
//!   of bits without assembling a `u32`, matching the partial-consume
//!   semantics of [`BitReader::get_bits`] on truncation (cursor lands
//!   at the buffer end rather than reverting).
//!
//! Still out of scope (subsequent rounds): the median-adaptation
//! *amount* that turns `decode_sample_value` into a stateful payload
//! loop (blocked on a docs gap), the prediction loop that consumes the
//! decorrelation typed views, the per-term sample count for stereo
//! predictors `0..=5` (open docs gap — round 11's partitioner refuses
//! them), float-data / large-or-shifted-int / overflow-bits
//! interpretation, multichannel channel-mask handling, hybrid
//! correction-stream (`.wvc`) pairing, CRC32 verification, encoder.
//!
//! ## Clean-room provenance
//!
//! All work in this crate has been implemented strictly against the
//! staged WavPack documentation under `docs/audio/wavpack/` (the
//! `wiki/WavPack.wiki` snapshot in tree and, from round 15 onward,
//! the `spec/wavpack-entropy-decode.md` clean-room trace). No
//! external library source, no archived prior history of this crate,
//! and no online resource outside the staged docs were consulted at
//! any phase.

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

mod block;
mod block_header;
mod decorrelation;
mod entropy;
mod error;
mod metadata;
mod packed_samples;
mod samples;

pub use crate::block::{
    block_count, iter_blocks, parse_block, parse_blocks, total_block_samples, BlockIter,
    UnsupportedBlockFeature, WavPackBlock,
};
pub use crate::block_header::{
    parse_block_header, Flags, WavPackBlockHeader, HEADER_LEN, MAGIC, MAX_VERSION, MIN_CK_SIZE,
    MIN_VERSION, TOTAL_SAMPLES_UNKNOWN,
};
pub use crate::decorrelation::{
    decorrelation_sample_count, expand_samples, expand_terms, expand_weights,
    partition_decorrelation_samples, weights_per_term, DecorrelationSamples, DecorrelationTerms,
    DecorrelationWeights, TermKind, MAX_DECORRELATION_SAMPLES_PER_TERM, MAX_DOCUMENTED_TERM,
    SAMPLE_EXPONENT_BIAS, SAMPLE_ON_WIRE_BYTES, TERM_DELTA_BITS, TERM_DELTA_MASK,
    TERM_PREDICTOR_BITS, TERM_PREDICTOR_MASK,
};
pub use crate::entropy::{
    expand_entropy, EntropyInfo, MEDIANS_PER_CHANNEL, MEDIAN_ON_WIRE_BYTES, MONO_PAYLOAD_BYTES,
    STEREO_PAYLOAD_BYTES,
};
pub use crate::error::{Error, Result};
pub use crate::metadata::{
    find_audio_payload, find_decorrelation_triple, find_entropy_info, find_first,
    find_md5_checksum_block, find_multichannel_info, find_packed_samples, parse_md5_checksum,
    parse_metadata_sub_block, walk_metadata, Md5Checksum, MetadataSubBlock, SubBlockFlags,
    SubBlockId, ID_FLAG_LARGE_SIZE, ID_FLAG_ODD_SIZE, ID_FLAG_OPTIONAL, ID_MASK, MD5_DIGEST_BYTES,
};
pub use crate::packed_samples::{expand_packed_samples, PackedSamples};
pub use crate::samples::{
    decode_packed_samples_mono, decode_packed_samples_mono_from_entropy,
    decode_packed_samples_stereo, decode_packed_samples_stereo_from_entropy, decode_run_length,
    decode_sample, decode_sample_stateful, decode_sample_stateful_stereo, decode_sample_value,
    golomb_interval, AdaptiveMedians, BitReader, DecodeState, GolombInterval, Medians, RunState,
    StereoDecodeState, Zone, DIV0, DIV1, DIV2, ESCAPE_EOF_CBITS, GET_MED_FLOOR, GET_MED_SHIFT,
    INTERVAL_MASK_31, MEDIAN_DEC_MULTIPLIER, MEDIAN_INC_MULTIPLIER, RUN_ESCAPE_CAP, UNARY_ESCAPE,
};
