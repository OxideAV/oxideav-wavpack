# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Round 245 — block-CRC accessor on `WavPackBlockHeader` /
  `WavPackBlock`. The wiki "Block structure" listing of
  `docs/audio/wavpack/wiki/WavPack.wiki` places a `32 bits - CRC`
  word at the trailing 4 bytes of the fixed 32-byte block header.
  The round-1 parser decoded the word little-endian into the
  `WavPackBlockHeader::crc` field but no typed accessor surfaced
  it on either the header or the block-level view — every other
  documented header field had been lifted to a typed accessor
  through rounds 214 and 239, so this was the last on-disk header
  field without a method surface. This round closes the gap.

  - `WavPackBlockHeader::crc(&self) -> u32` — typed accessor
    returning the stored 32-bit word verbatim. The wiki names the
    field but does not specify the polynomial, the byte span, the
    initial value, or the byte / bit order of the computation, and
    the staged docs do not either; the accessor surfaces the
    stored bytes only and leaves recomputation / verification to a
    later round once the algorithm spec lands.
  - `WavPackBlock::crc(&self) -> u32` — block-level pass-through
    pairing the block surface with the header accessor, for
    callers iterating a multi-block stream that want to pick the
    per-block CRC off a borrowed `WavPackBlock` alongside the
    other round-214 / round-239 introspection accessors.
  - 8 new tests (484 total, up from 476): header accessor verbatim
    return; full-`u32`-range round-trip across documented extremes
    (`0`, `1`, `u32::MAX`, alternating-bit patterns); little-endian
    decode of bytes 28..32 through `parse_block_header`;
    independence from other header fields; block-level
    pass-through parity (`block.crc() == block.header().crc()`);
    full-range extremes round-trip through `parse_block`;
    per-block independence across a two-block stream; block-level
    independence from the round-239 accessors.

- Round 242 — `0x0C` packed-overflow-bits typed view + walker
  bridges + block-level introspection. The wiki "IDs" listing of
  `docs/audio/wavpack/wiki/WavPack.wiki` annotates sub-block `0x0C` as
  "packed overflow bits from floating-point or large integers", and
  the staged clean-room entropy doc
  `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 names the
  same ID as the **extension bitstream**. The round-2 metadata walker
  already routes `0x0C` payloads through the typed
  `SubBlockId::PackedOverflowBits` discriminant, but the bytes had
  not yet been elaborated into the same typed view + finder +
  block-level accessor shape the round-12 `PackedSamples` (`0x0A`)
  and round-233 `PackedCorrectionData` (`0x0B`) views expose. This
  round closes that gap.

  - `PackedOverflowBits<'a>` in `src/overflow_bits.rs` carries the
    borrowed `0x0C` payload verbatim. Mirrors the `PackedSamples` /
    `PackedCorrectionData` shape: `bytes()` / `len()` / `is_empty()`
    introspection + `bit_reader()` factory producing a fresh
    `BitReader` positioned at bit 0 (LSB-first within each byte, the
    same convention every payload-carrying view in the crate uses).
    Any byte slice — including the empty one — is accepted: the wiki
    places no length constraint on the payload.
  - `expand_packed_overflow_bits(payload: &[u8]) -> PackedOverflowBits<'_>`
    — typed constructor analogous to `expand_packed_samples` and
    `expand_packed_correction_data`.
  - `find_packed_overflow_bits_sub_block(subs)` — walker finder
    returning the borrowed `MetadataSubBlock<'a>`, for callers that
    want the raw sub-block handle.
  - `find_packed_overflow_bits(subs)` — typed walker finder returning
    `Option<PackedOverflowBits<'a>>` directly, the one-call bridge
    from the round-2 metadata walker output to the
    `BitReader`-construction handoff.
  - `WavPackBlock::has_packed_overflow_bits()` — boolean
    discriminant for the presence of a `0x0C` sub-block.
  - `WavPackBlock::find_packed_overflow_bits_sub_block()` /
    `WavPackBlock::packed_overflow_bits()` — block-level accessors
    pairing with the free walker finders.

  The float (wiki flag bit 7) / large-integer (wiki flag bit 8)
  container fix-ups that would actually consume the wrapped bytes
  remain gated on the `UnsupportedBlockFeature::FloatData` /
  `UnsupportedBlockFeature::Int32Mode` feature refusals in
  `WavPackBlock::decode_samples`. The typed view is a deferred
  handoff into the bit reader, not a decode pass.

  17 new tests (476 total, up from 459): the new module's view +
  walker + bit-reader contract (`new_preserves_bytes_verbatim` /
  `empty_payload_is_accepted_and_reported_empty` /
  `expand_packed_overflow_bits_round_trips_the_byte_slice` /
  `bit_reader_starts_at_byte_zero_bit_zero` /
  `bit_reader_yields_first_bit_lsb_first` /
  `bit_reader_factory_returns_independent_readers` /
  `empty_payload_bit_reader_reports_immediate_truncation` /
  `view_is_copy_and_independent_of_caller_lifetime` /
  `view_is_distinct_type_from_packed_samples_and_correction`) plus
  the metadata walker bridges
  (`find_packed_overflow_bits_typed_view_returns_view_when_0x0c_present`
  / `find_packed_overflow_bits_typed_view_returns_none_when_0x0c_absent`
  / `find_packed_overflow_bits_sub_block_returns_metadata_borrow` /
  `find_packed_overflow_bits_sub_block_returns_none_when_0x0c_absent`)
  plus the block-level accessor surface
  (`has_packed_overflow_bits_returns_false_on_no_0x0c_subblock` /
  `has_packed_overflow_bits_returns_true_with_0x0c_subblock` /
  `packed_overflow_bits_view_round_trips_with_bit_reader` /
  `has_packed_overflow_bits_is_independent_of_0x0b_and_0x07`).

- Round 239 — typed file-total / end-cursor accessors on
  `WavPackBlockHeader` / `WavPackBlock` and the stream-level
  `stream_total_samples` free function. The wiki "Block structure"
  listing of `docs/audio/wavpack/wiki/WavPack.wiki` names three
  sample-cursor fields the round-1 header parser preserved verbatim
  (`total samples in file` with `0xFFFFFFFF` reserved as the wiki
  "unknown" sentinel; `offset in samples for current block`; `samples
  in this block`), but only the boolean `is_total_samples_known`
  discriminant was surfaced through the typed API. This round adds:

  - `WavPackBlockHeader::total_samples_in_file` — returns
    `Option<u32>`, the typed sentinel-aware view of the file-global
    total. `Some(n)` for a known total, `None` for the
    `TOTAL_SAMPLES_UNKNOWN` constant. `Some(0)` and `None` remain
    distinguishable — the wiki allows a literal zero total as a
    legitimate "no audio at all" value, distinct from the sentinel.
  - `WavPackBlockHeader::end_sample_index` — returns `u64` =
    `block_index + block_samples`, the half-open upper bound of this
    block's sample contribution. A metadata-only block
    (`block_samples == 0`) reports the same cursor as `block_index`,
    consistent with the wiki "may be 0 if no audio present" note. The
    `u64` return type covers the pathological `u32::MAX + u32::MAX`
    summands without overflow.
  - `WavPackBlockHeader::samples_remaining_after` — returns
    `Option<u64>` = `total - end` when both the total is known and the
    end cursor lies within it. `None` for the wiki sentinel
    (cannot answer without the total) and for the malformed end-past-
    total combination (refuses to surface as a negative count).
  - `WavPackBlock::total_samples_in_file` /
    `WavPackBlock::end_sample_index` /
    `WavPackBlock::samples_remaining_after` — block-level pass-throughs
    so callers iterating parsed blocks reach the typed values without
    going through `.header`.
  - `WavPackBlock::is_final_audio_block_in_file` — boolean
    `samples_remaining_after() == Some(0)` discriminant for the
    "last block of a fully-described `.wv` file" case.
  - `stream_total_samples(&[u8]) -> Result<Option<Option<u32>>>` —
    stream-level free function reading the typed file-total from the
    first block's header. Outer `None` for empty input (no first
    block); outer `Some` carrying the inner `Option<u32>` from
    `WavPackBlockHeader::total_samples_in_file`. The wiki documents
    `total_samples` as file-global, so reading only the first block's
    32-byte fixed header (constant-time, no metadata walk) is the
    minimal call that surfaces the stream-level total.

  All four surfaces derive directly from the three explicitly
  documented wiki fields — no spec gap, no docs-gap-blocked surface
  touched. No new error variants. The `WavPackBlock` exports are
  re-exported through the existing `crate::block` block; the new
  `stream_total_samples` free function joins the existing stream-level
  free-function surface in `lib.rs`.

- 23 new unit tests (459 total, up from 436) pin: the sentinel /
  known / zero-as-distinct-from-sentinel discrimination on the typed
  `Option`; the u32-extreme-summands non-overflow on
  `end_sample_index`; the metadata-only-block end-cursor
  non-advancement (cursor stays at `block_index`); the
  exact-end / non-zero-remainder / unknown-total / end-past-total
  branches of `samples_remaining_after`; the boolean
  `is_final_audio_block_in_file` discriminant on each of those
  branches; the stream-level free function on empty / single-block /
  multi-block / sentinel / malformed-header inputs; the first-block-
  only contract (a second block's hypothetically-different total
  is not consulted); and the cross-block consistency of
  `end_sample_index` / `samples_remaining_after` /
  `is_final_audio_block_in_file` across a synthesised three-block
  stream.

- Round 233 — `.wvc` correction-stream typed view + walker bridges +
  block-level and stream-level introspection accessors. The wiki "IDs"
  listing of `docs/audio/wavpack/wiki/WavPack.wiki` annotates sub-blocks
  `0x07` (noise-shaping profile) and `0x0B` (packed correction data) as
  carried in the `.wvc` companion file alongside the lossy main `.wv`;
  this round elaborates the round-2 walker output for `0x0B` into a typed
  `PackedCorrectionData<'a>` view (analogous to the round-12
  `PackedSamples<'a>` view for `0x0A`) and threads the same finder /
  predicate / iterator pattern through the metadata-walker / block-level
  / stream-level surfaces.

  - New `PackedCorrectionData<'a>` typed view in
    `src/correction.rs` carries the borrowed `0x0B` sub-block payload
    verbatim and exposes the `bytes()` / `len()` / `is_empty()` byte
    introspection plus a `bit_reader()` factory that yields a fresh
    `BitReader<'a>` positioned at bit 0 — the same shape
    `PackedSamples::bit_reader` exposes for the main `0x0A` stream.
  - New free `expand_packed_correction_data(payload: &[u8]) ->
    PackedCorrectionData<'_>` constructor — the round-2 walker output
    bridge for the `0x0B` ID, analogous to `expand_packed_samples`.
  - New walker finders `find_packed_correction_data` (typed-view) /
    `find_packed_correction_data_sub_block` (raw-borrow) /
    `find_noise_shaping_profile` (raw-borrow) / `find_hybrid_profile`
    (raw-borrow) in `src/metadata.rs` pair the walker output with the
    three hybrid-mode sub-block IDs without re-walking the metadata.
  - New `WavPackBlock` accessors: `has_packed_correction_data` /
    `packed_correction_data` / `find_packed_correction_data_sub_block`
    / `has_noise_shaping_profile` /
    `find_noise_shaping_profile_sub_block` / `has_hybrid_profile` /
    `find_hybrid_profile_sub_block` / `has_correction_stream_data`
    (the composite predicate matching the
    `MetadataSubBlock::is_correction_payload` grouping).
  - New stream-level free functions: `correction_block_count` /
    `first_correction_block` / `iter_correction_blocks` /
    `total_correction_payload_bytes`, plus a new
    `CorrectionBlockIter<'a>` (`Clone` + `FusedIterator`) lazy
    iterator mirroring the round-230 `AudioBlockIter` shape but
    filtering to blocks whose `has_correction_stream_data` predicate
    fires.

  The hybrid-mode sample decode itself (spec §4.2 step 6 second
  paragraph, `error_limit != 0`) stays out of scope — the typed views
  give a callable handle into the bytes without committing to a decode
  semantics, and the existing
  `Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Hybrid)`
  refusal on `WavPackBlock::decode_samples` is preserved verbatim.
- 44 new unit tests (436 total, up from 392) pin: the typed
  `PackedCorrectionData` view shape (empty / non-empty / round-trip /
  bit-reader factory / `Copy` + lifetime / distinct-from-`PackedSamples`
  type discrimination); each new block-level accessor / finder on
  present / absent / both-present inputs; the composite
  `has_correction_stream_data` predicate as the union of `0x07` and
  `0x0B`; the new stream-level free functions on empty / all-plain /
  mixed / error-trailing inputs; `CorrectionBlockIter` `Clone +
  FusedIterator` trait bounds and the `new` / free-function
  call-shape twin equivalence; `total_correction_payload_bytes`
  summing only `0x0B` payload bytes (excluding `0x07`); a
  metadata-only block carrying only a `0x0B` payload still surfaces
  as correction-bearing (block-samples allowance honoured); and the
  hybrid-flag refusal contract is unchanged by the presence of a
  `0x0B` typed view (structural introspection vs. decode
  enablement).

- Round 230 — stream-level introspection accessors composing the
  round-219 [`iter_blocks`] for aggregate "how many / what shape" /
  "where's the first audio block" questions without retaining the
  parsed block list. New free functions [`audio_block_count`] /
  [`metadata_block_count`] split the wiki "Block structure"
  `block_samples > 0` audio blocks from the `block_samples == 0`
  metadata-only blocks (RIFF wrappers, MD5 sums, encoding-details);
  together they sum to [`block_count`] across any input. New
  [`total_audio_samples`] sums the wiki "samples in this block" field
  across audio blocks only, returning `u64` so a 4-GiB-plus stream's
  sample count does not overflow `u32`. New [`decoded_sample_count`]
  free function sums the `i32` PCM slot count [`decode_stream`] would
  produce across the audio blocks (mono / false-stereo contribute
  `block_samples`, stereo contribute `block_samples * 2`); the matching
  block-level [`WavPackBlock::decoded_sample_count`] reports the same
  shape per-block from the header alone — no entropy expansion, no
  per-sample-loop call. New [`first_audio_block`] peeks the first
  decode-eligible block past any leading metadata-only blocks (the wiki
  allowance for RIFF-header-only blocks); returns `Ok(None)` on empty /
  all-metadata-only input and surfaces the first [`parse_block`] error
  verbatim if a malformed block appears before the first audio block.
  New [`AudioBlockIter`] is the `Clone`-able, `FusedIterator`-compliant
  lazy iterator yielding only audio blocks; [`iter_audio_blocks`] is
  the call-shape twin. New [`BlockIter::next_audio`] adapter method
  skips metadata-only blocks on the existing block iterator. No new
  error variants and no docs-gap-blocked surface touched.
- 34 new unit tests (392 total, up from 358) pin: each accessor on
  empty / all-metadata-only / mixed inputs; the
  audio + metadata == total identity across counts; the structural
  [`WavPackBlock::decoded_sample_count`] matching the actual PCM length
  [`WavPackBlock::decode_samples`] returns on mono and stereo;
  [`decoded_sample_count`] (stream) matching the [`decode_stream`]
  result length; [`AudioBlockIter`]'s `Clone + FusedIterator` trait
  bounds; the
  `audio_block_count == iter_audio_blocks().count()` equivalence; the
  `BlockIter::next_audio` skip-then-yield-then-error-then-fuse
  contract; [`AudioBlockIter::new`] and [`iter_audio_blocks`] returning
  identical sequences; and parse-error propagation through every
  accessor.

- Round 224 — multi-block stream → PCM composer fusing the round-219
  [`BlockIter`] with the round-206 [`WavPackBlock::decode_samples`]
  into a single byte-buffer → `Vec<i32>` surface. New eager
  [`decode_stream`] walks every audio block in the input and
  concatenates the decoded PCM in on-disk order; new
  [`StreamDecodeIter<'a>`] is the `Clone`-able,
  `FusedIterator`-compliant lazy counterpart yielding
  `Result<Vec<i32>>` once per **audio** block (metadata-only blocks
  with `block_samples == 0` are silently skipped since they carry no
  PCM to return — a positive contract preventing a spurious
  [`Error::BlockHasNoAudio`] refusal on `.wv` files whose first block
  is a RIFF-header-only metadata block). New free function
  [`iter_decoded_blocks`] is the `iter_decoded_blocks(bytes)`
  call-shape twin of [`StreamDecodeIter::new`].
  [`StreamDecodeIter`] fuses on the first error (parse or decode) via
  the underlying round-219 [`BlockIter`] fuse mechanism + the
  round-206 refusal taxonomy — both compose without translation. Two
  introspection accessors on [`StreamDecodeIter`] —
  [`StreamDecodeIter::remaining`] (forwards [`BlockIter::remaining`])
  and [`StreamDecodeIter::is_exhausted`] (forwards
  [`BlockIter::is_exhausted`]) — round out the surface. No new error
  variants and no docs-gap-blocked surface touched: every error this
  composer surfaces is one [`parse_block`] or
  [`WavPackBlock::decode_samples`] already raised. Per-block mono /
  stereo dispatch (the wiki bit 2 + bit 30 union from round 206 via
  [`Flags::is_block_data_mono`]) is preserved verbatim, so a
  multi-block input may mix mono and stereo blocks and the
  concatenated `Vec<i32>` reflects each block's own shape. Per-block
  `0x05` seed re-initialisation (round 206 per-block stateless
  contract) is preserved across the stream.
- 21 new unit tests (358 total, up from 337): empty-buffer input
  yielding `Ok(vec![])` (not an error — the wiki "WavPack file
  consists of blocks" sentence is plural but [`BlockIter`] accepts the
  degenerate empty file); single audio block yielding `[0]`; three
  audio blocks concatenating to `[0, 0, 0]` in on-disk order;
  metadata-only blocks silently skipped both between audio blocks and
  at the leading position (no spurious [`Error::BlockHasNoAudio`]);
  all-metadata-only input yielding `Ok(vec![])`;
  [`Error::CkSizeExceedsBuffer`] propagated verbatim from a malformed
  second block; [`Error::UnsupportedBlockFeature(Hybrid)`] propagated
  verbatim from a hybrid-flagged audio block;
  [`Error::BlockMissingEntropyInfo`] propagated from an audio block
  lacking the `0x05` sub-block; eager [`decode_stream`] discarding
  prior-block PCM on a mid-stream decode error (the documented eager
  contract); [`iter_decoded_blocks`] yielding one item per audio
  block with metadata-only blocks omitted; [`iter_decoded_blocks`]
  fusing on first parse error AND on first decode error (both
  routes); empty input and all-metadata-only input yielding zero
  items; [`StreamDecodeIter::new`] and [`iter_decoded_blocks`]
  returning identical sequences; [`StreamDecodeIter::remaining`]
  tracking the underlying [`BlockIter`] across a `next()` call (full
  buffer → empty after draining); the `Clone + FusedIterator` trait
  bounds (a compile-time check via a generic helper); a mixed
  mono+stereo input yielding `[0, 0, 0]` confirming the per-block
  dispatch contract; and the eager / lazy equivalence over a
  three-audio-block input (`decode_stream` is observationally
  identical to draining [`iter_decoded_blocks`] and concatenating
  each `Vec<i32>` via `flat_map`).

- Round 219 — multi-block stream iteration on top of the round-13
  [`parse_block`] composer, lifting the wiki "WavPack file consists of
  blocks each beginning with 'wvpk'" file-format sentence into typed
  public API. New [`BlockIter<'a>`] (a `Clone`-able, `FusedIterator`-
  compliant lazy iterator yielding `Result<WavPackBlock<'a>>`) walks
  the chain by repeatedly calling [`parse_block`] on the previous
  call's returned tail; the iterator fuses on the first error so the
  caller can `?`-bubble without re-encountering the same failure on
  a follow-up `next()`. New free function [`iter_blocks`] is the
  `iter_blocks(bytes)` call-shape twin of [`BlockIter::new`]. New
  [`parse_blocks`] eagerly collects the iterator into a
  `Vec<WavPackBlock<'_>>`, surfacing the first parse error verbatim.
  New [`block_count`] iterates without retaining the parsed blocks
  for callers that want only the count (working-set memory stays at
  one block independent of input length). New [`total_block_samples`]
  pure accessor sums the wiki "samples in this block" field across
  an already-parsed block list, returning a `u64` so a 4-GiB-plus
  stream's sample count does not overflow `u32` on the way out.
  Two introspection accessors on [`BlockIter`] —
  [`BlockIter::remaining`] (the bytes not yet consumed; on a fused
  iterator after an error, points at the malformed block's first
  byte for precise offset diagnostics) and [`BlockIter::is_exhausted`]
  — round out the surface. No new error variants and no docs-gap-
  blocked surface touched: every iterator yield is a direct
  [`parse_block`] result and the existing round-13
  [`Error::CkSizeExceedsBuffer`] / [`Error::Truncated`] split
  surfaces verbatim across the iteration boundary.
- 18 new unit tests (337 total, up from 319): empty-buffer
  iteration yielding zero items and remaining empty on construction
  (the wiki "WavPack file consists of blocks" sentence is plural but
  the empty file is treated as zero blocks rather than an error);
  single-block iteration yielding one Ok then terminating with
  `is_exhausted() == true`; three back-to-back identical empty
  blocks yielding three Ok items in order; `remaining()` shrinking
  by exactly `8 + ck_size` per successful `next()` call (matching
  the wiki "Block structure" on-disk extent definition); the
  fused-on-error contract — first block Ok, second block with
  corrupt magic → `Err(InvalidMagic)`, then `None` on every
  subsequent call with the iterator's `remaining()` slice still
  pointing at the malformed block's first byte for offset
  recovery; `CkSizeExceedsBuffer { ck_size: 200, available: 32 }`
  surfacing on a second block whose header advertises a
  longer payload than the buffer carries; `Truncated` surfacing on
  a partial header (sub-`HEADER_LEN` tail) between blocks — the
  round-13 split between "buffer ran out mid-payload" and "buffer
  ran out between blocks" preserved across the iterator boundary;
  [`BlockIter::new`] and [`iter_blocks`] returning identical
  sequences; [`parse_blocks`] returning a `Vec<WavPackBlock>` in
  the same order as iteration on a synthesised three-block stream
  with distinct `block_samples` (`100` / `200` / `300`),
  bubbling the first error rather than the partial Vec on a
  malformed second block, and returning an empty Vec on an empty
  input; [`block_count`] returning the matching count across a
  five-block stream and bubbling the first error verbatim;
  [`total_block_samples`] summing the wiki "samples in this block"
  field across a three-block list (`100 + 200 + 300 = 600`),
  returning `0` on the empty slice, and the u64 return type
  preventing the `u32::MAX + u32::MAX` two-block sum from
  overflowing — i.e. confirming the file-scale sample count
  withstands a 4-GiB-plus stream; and a final equivalence check
  proving [`parse_blocks`] is observationally identical to
  manually draining [`iter_blocks`] on the same input.

- Round 214 — block-level discovery / accessor sweep on
  [`WavPackBlock`], surfacing the typed views the existing free
  finders already build over `self.sub_blocks` without making the
  caller reach through `block.sub_blocks()`. New header passthroughs:
  [`WavPackBlock::flags`] borrows `&self.header.flags`,
  [`WavPackBlock::block_samples`] / [`WavPackBlock::block_index`]
  return the wiki "samples in this block" / "offset in samples for
  current block" fields, and [`WavPackBlock::is_audio_block`] lifts
  the round-1 [`WavPackBlockHeader::is_audio_block`] predicate to the
  block level so a multi-block iterator can filter without reaching
  through the header. New presence predicates:
  [`WavPackBlock::has_entropy_info`] /
  [`WavPackBlock::has_packed_samples`] /
  [`WavPackBlock::has_md5_checksum`] /
  [`WavPackBlock::has_riff_header`] /
  [`WavPackBlock::has_riff_trailer`] /
  [`WavPackBlock::has_multichannel_info`] keyed on
  [`SubBlockId`] equality. New borrow finders:
  [`WavPackBlock::find_sub_block`] /
  [`WavPackBlock::find_entropy_info_sub_block`] /
  [`WavPackBlock::find_md5_checksum_sub_block`] /
  [`WavPackBlock::find_multichannel_info_sub_block`] /
  [`WavPackBlock::find_riff_header_sub_block`] /
  [`WavPackBlock::find_riff_trailer_sub_block`] each pair with the
  corresponding `has_*` predicate and return an
  `Option<&MetadataSubBlock<'a>>` borrow. New typed extractors:
  [`WavPackBlock::packed_samples`] returns
  `Option<PackedSamples<'a>>` (the round-12 typed view over the
  `0x0A` payload); [`WavPackBlock::entropy_info`] returns
  `Result<Option<EntropyInfo>>` (the round-4 expander wrapped so
  missing `0x05` reports `Ok(None)` rather than an error, and a
  malformed `0x05` propagates the existing
  [`Error::EntropyInfoLength`]); [`WavPackBlock::md5_checksum`]
  returns `Result<Option<Md5Checksum>>` with the same shape
  (missing → `Ok(None)`, malformed → `Error::Md5ChecksumLength`).
  These accessors compose with the round-206 decode loop: a typical
  pre-flight call sequence is `block.is_audio_block()` +
  `block.has_entropy_info()` + `block.has_packed_samples()` +
  `block.decode_samples()`, all on a borrowed
  [`WavPackBlock`]. Round 214 adds no new error variants and does
  not touch any docs-gap-blocked surface; the round-3 decorrelation
  expanders still lack a prediction-loop consumer and the
  median-adaptation amount remains the open docs gap for stateful
  Golomb refinement (still wired only through the round-15/199
  per-sample loop and the round-201/206 composers).
- 24 new unit tests (319 total): [`WavPackBlock::flags`] pin against
  `self.header.flags.raw` round-trip with the `mono` bit asserted;
  [`WavPackBlock::block_samples`] / [`WavPackBlock::block_index`]
  match the header values (with an explicit 12345 block_index
  patched into the synthesised header bytes); the
  [`WavPackBlock::is_audio_block`] predicate symmetric with
  [`WavPackBlockHeader::is_audio_block`] across the `block_samples`
  zero / non-zero boundary; the six `has_*` presence predicates each
  fire on the matching sub-block ID and clear on a metadata-empty
  block; [`WavPackBlock::find_sub_block`] returning the first
  matching `&MetadataSubBlock<'a>` borrow with payload pin and
  reporting `None` for an absent ID; each specialised borrow finder
  ([`find_entropy_info_sub_block`] / [`find_md5_checksum_sub_block`]
  / [`find_multichannel_info_sub_block`] /
  [`find_riff_header_sub_block`] / [`find_riff_trailer_sub_block`])
  returning the present / absent shapes with payload pins;
  [`WavPackBlock::packed_samples`] returning a typed
  [`PackedSamples`] view with byte-array round-trip and `None` on
  absence; [`WavPackBlock::entropy_info`] decoding a mono `0x05`
  payload to a typed [`EntropyInfo`] via the explicit-exponent
  log-pack path (`[mantissa_lo, 0x09]` → `median = mantissa`)
  returning `Ok(Some([5, 3, 7]))`, returning `Ok(None)` on a
  block with no `0x05`, and propagating
  `Err(Error::EntropyInfoLength(8))` for a malformed 8-byte
  payload; [`WavPackBlock::md5_checksum`] decoding the standard
  "empty input" digest
  (`d41d8cd98f00b204e9800998ecf8427e`) as a pinned test vector,
  returning `Ok(None)` on an absent `0x26`, and propagating
  `Err(Error::Md5ChecksumLength(8))` for a wrong-length payload; and
  an end-to-end pairing test confirming the round-214 accessors and
  the round-206 [`decode_samples`] composer compose on a single
  block with `0x05` + `0x0A` + `0x26` returning the expected `[0]`
  PCM sample alongside `Some` entropy / md5 / packed-samples typed
  views and `false` for the un-present `has_riff_*` /
  `has_multichannel_info` / `has_decorrelation` predicates.

- Round 206 — block-level `WavPackBlock::decode_samples()` composer
  turning the round-13 [`parse_block`] aggregate into PCM samples in
  one call. Chains [`find_entropy_info`] + [`expand_entropy`] +
  [`find_packed_samples`] through the round-201
  `decode_packed_samples_mono_from_entropy` /
  `decode_packed_samples_stereo_from_entropy` wrappers depending on
  the new [`Flags::is_block_data_mono`] accessor (the union of wiki
  bit 2 `mono` and wiki bit 30 `false_stereo`). Returns a `Vec<i32>`
  of `block_samples` mono samples or `block_samples * 2` interleaved
  stereo samples. New [`UnsupportedBlockFeature`] enum names the
  seven WavPack v.4 features the per-sample loop does not yet
  support (`Hybrid` lossy profile / `FloatData` / `Int32Mode` /
  `MultichannelMember` / `Decorrelation` / `LowLatencyBlock` /
  `RobustBlock`); each is surfaced through the new
  `Error::UnsupportedBlockFeature(feature)` variant with a Display
  impl that names the responsible wiki flag bit or sub-block ID.
  New structural errors `Error::BlockHasNoAudio` /
  `Error::BlockMissingEntropyInfo` /
  `Error::BlockMissingPackedSamples` cover the three "block doesn't
  carry what the composer needs" shortfalls. New
  [`WavPackBlock::has_decorrelation`] predicate detects the presence
  of any of the `0x02` / `0x03` / `0x04` decorrelation sub-blocks so
  the composer refuses pre-pass blocks without trying to walk the
  round-3 typed views (which exist but lack a prediction-loop
  consumer). New [`Flags::is_block_data_stereo`] is the inverse of
  `is_block_data_mono`. The composer is **stateless**: each call
  seeds a fresh [`AdaptiveMedians`] from the block's `0x05` payload
  and drops it on return, matching how real `.wv` files carry a
  fresh `0x05` seed per block.
- 23 new unit tests (295 total): a mono one-sample happy path
  (seed `[0,0,0]` + 0x0A `0x00` byte payload → `[0]` via the spec
  §4.2 step 1 zero-run path emitting a single zero sample); the
  matching stereo happy path with minimal-non-zero seeds (`[1,0,0]`
  on both channels — non-zero so `EntropyInfo::is_mono()` reports
  stereo, but still `get_med(0) == 1` so the spec §4.2 step 1 fast
  path stays eligible) yielding `[0, 0]`; the false-stereo dispatch
  (bit 30 set, bit 2 clear, mono-layout `0x05` → mono decode loop);
  `BlockHasNoAudio` on `block_samples == 0`;
  `BlockMissingEntropyInfo` on a block with only `0x0A`;
  `BlockMissingPackedSamples` on a block with only `0x05`; each of
  the seven `UnsupportedBlockFeature` variants triggered by the
  matching flag bit or sub-block presence (`Hybrid` / `FloatData` /
  `Int32Mode` / `MultichannelMember` / `Decorrelation`
  exercised through each of `0x02` / `0x03` / `0x04` /
  `LowLatencyBlock` / `RobustBlock`); the `EntropyInfoLength`
  propagation from a malformed 8-byte `0x05` payload;
  `WavPackBlock::has_decorrelation` firing on each of the three
  sub-block IDs and clearing when none is present; the four
  `is_block_data_mono` / `is_block_data_stereo` arms (plain mono,
  false stereo, plain stereo, mono + false-stereo); and the
  `UnsupportedBlockFeature` Display strings naming the relevant
  wiki bit / sub-block ID for each variant.
- Round 201 — `EntropyInfo` → `AdaptiveMedians` channel-indexed bridges
  and end-to-end `from_entropy` wrappers for the mono and stereo
  `0x0A` decode loops, removing the round-15/199 caller's hand-rolled
  per-channel seed extraction. New `EntropyInfo::stereo(left, right)`
  symmetric constructor (counterpart to the existing `EntropyInfo::mono`).
  New `AdaptiveMedians::from_entropy(info, channel_idx)` returning
  `Some(state)` for channel 0 on any payload and channel 1 on a stereo
  payload (the same shape as `Medians::from_entropy`), with `None` for
  out-of-range indices, channel 1 on mono, and negative seeds (defensive
  `i32 → u32` reject). New `AdaptiveMedians::stereo_pair_from_entropy(info)`
  returning the `[left, right]` array `decode_packed_samples_stereo`
  takes (returns `None` on mono or any negative seed). New top-level
  `decode_packed_samples_mono_from_entropy(payload, info, count)` and
  `decode_packed_samples_stereo_from_entropy(payload, info, frames)`
  wrappers that compose the bridges with the round-15/199 stateful
  loops, surfacing a new `Error::InvalidEntropyInfoForMono` /
  `Error::InvalidEntropyInfoForStereo` for the malformed-input arm.
  20 new tests (272 total): the `EntropyInfo::stereo` constructor
  populating both sets and matching its struct-literal form (plus the
  is_mono-by-content nuance with a zero right set); the
  `AdaptiveMedians::from_entropy` bridge across channel 0 / 1 / out-of-
  range indices, mono right-channel rejection, and negative-seed
  rejection on both channels; the `stereo_pair_from_entropy` mono
  rejection and per-channel negative-seed rejection; the
  `_from_entropy` wrappers proving byte-identical reconstruction to
  the explicit-seed calls, the malformed-input error paths firing
  before any bits are read, and the zero-count / zero-frame vacuous
  cases.
- Round 199 — stereo per-sample `0x0A` decode loop wiring the
  `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §2 channel-
  alternation rule on top of the round-15 mono loop. New
  `StereoDecodeState` bundles two per-channel `RunState`s (`left_run`,
  `right_run`) for the spec §4.2 step 4 holding-bit fold applied
  per-channel, a stream-level `zero_run_pending` counter and
  `ever_took_zero_run` sticky bit for the spec §4.2 step 1 zero-run
  fast path (which the spec specifies as stream-level: gated on BOTH
  channels' `median[0] <= 1` AND BOTH channels' holding state empty,
  resetting BOTH channels' medians on a non-zero run), and a
  `next_channel` parity cursor (`0` = left, `1` = right) that toggles
  only on a successful emit so a `?`-bubbled error leaves the cursor
  recoverable. New `decode_sample_stateful_stereo(reader, &mut
  [AdaptiveMedians; 2], &mut StereoDecodeState)` walks the spec §4.2
  sequence for one stereo sample, dispatching reads + adaptation +
  mantissa + sign to `medians[next_channel]` only (the other channel's
  state stays untouched). New `decode_packed_samples_stereo(payload,
  &mut [AdaptiveMedians; 2], frames)` returns a `Vec<i32>` of
  `frames * 2` interleaved (L,R,L,R,…) PCM samples and is the first
  public stereo end-to-end PCM-producing API on the crate. Pair with
  `EntropyInfo::medians_for_channel(0/1)` + `AdaptiveMedians::from_seed_values`
  on the round-4 expander's left + right median sets to seed.
- 13 new unit tests (252 total): simulator-driven zone-1 round-trip
  across 8 stereo frames with matching seeds; the same with distinct
  per-channel seeds proving per-channel dispatch never crosses
  medians; mixed zones (zone 1 / zone 2 / zone 2 overflow) per channel
  with a per-channel adapt simulator picking magnitudes against the
  current medians; negative-sign reconstruction with magnitude held in
  zone 1 by picking `signed_value = -(get_med(0) + 2)` (so the
  decoded `!signed_value` magnitude lands at `get_med(0) + 1`); the
  end-to-end `decode_packed_samples_stereo` loop wrapper matching the
  per-call sequence bit-for-bit AND finishing with identical per-
  channel medians; the stereo zero-run path (a `1101` wire bit-string
  decoding to run_length = 3) zeroing BOTH channels' medians on entry,
  emitting `0` on the current channel, draining the remaining two
  zero samples across alternating channels without consuming bits;
  the BOTH-channel zero-run gate rejecting one-channel eligibility
  (left zeroed, right at `[256, 256, 256]` → fast path off, normal
  prefix path fires and reports Truncated on an empty buffer); the
  stereo truncation path leaving `next_channel` at `0` on error; the
  EOF escape (`LIMIT_ONES + cbits == 33` wire bit-string) surfacing
  `Error::EndOfStream` with `next_channel` still at `0`; per-channel
  holding-state independence (a left-channel `last_zero` short-
  circuit doesn't touch the right channel's state); the empty-
  payload `decode_packed_samples_stereo` rejecting with Truncated
  AND leaving the medians unchanged; the `frames == 0` vacuous case
  returning an empty `Vec`; and the `StereoDecodeState::default()`
  matching `StereoDecodeState::new()` with per-channel `RunState`s
  also matching `RunState::new()`.
- Round 15 — stateful per-sample `0x0A` decode loop wiring the newly
  staged `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §3 + §3.2
  + §4.2 into the end-to-end decode path. New `decode_sample_stateful`
  primitive runs the full spec §4.2 sequence per sample: optional
  spec §4.2 step 1 zero-run fast path (gated on `get_med(0) <= 1` and
  no holding bits, owing `run_length - 1` zero samples across
  subsequent calls via a new `DecodeState::zero_run_pending` field);
  spec §4.2 steps 2 + 3 unary prefix with the `LIMIT_ONES = 16`
  escape and the `cbits == 33` EOF marker (new `Error::EndOfStream`);
  spec §4.2 step 4 holding-bit fold (re-using the existing wiki-
  compressed `RunState`); spec §4.2 step 5 `(low, high)` interval
  with 31-bit mask and `high >= low` clamp; spec §3.2 per-zone
  `AdaptiveMedians::adapt(Zone::from_ones_count(ones_count))` BEFORE
  the mantissa read; spec §4.2 step 6 first paragraph truncated-binary
  mantissa decode (`maxcode = high - low`, `bitcount` = bit-length of
  `maxcode`, `extras = (1 << bitcount) - maxcode - 1`, short
  `bitcount - 1`-bit form vs long `bitcount`-bit phase-in); spec §4.2
  step 7 sign bit returning `~mid` or `mid`. New
  `decode_packed_samples_mono(payload, &mut medians, count)` wraps the
  per-sample primitive into an end-to-end mono single-block loop and
  is the first public API on the crate that produces a `Vec<i32>` of
  PCM samples from a `0x0A` payload. New constants `ESCAPE_EOF_CBITS
  = 33`, `RUN_ESCAPE_CAP = 33` and `INTERVAL_MASK_31 = 0x7fff_ffff`
  spell out the spec §4.2 step 3 EOF marker, the spec §4.2 step 1
  zero-run unary cap and the spec §4.2 step 5 interval mask.
- Round 15 — 18 new tests covering the stateful decoder via a
  spec-derived inverse encoder (`encode_one_sample`,
  `emit_truncated_binary`, `pick_raw_unary`, `pick_zone_for_magnitude`
  test helpers) for bit-exact closure across zones 0 / 1 / 2 /
  overflow, negative-sign reconstruction, a hand-traced 2-sample
  fixture pinning the spec §4.2 step 1-7 sequence by literal bit
  string, the zero-run fast path engaging and resting eligibility,
  the `LIMIT_ONES`+`cbits=33` EOF escape, a 64-sample adaptive-median
  drift sequence using a deterministic value-from-current-medians
  simulator, the `decode_packed_samples_mono` end-to-end loop, the
  `read_truncated_binary` primitive round-trip across every code in
  a `maxcode = 16` interval, and the `form_interval` spec §4.2 step 5
  ladder for the four named zones.

## [0.0.2](https://github.com/OxideAV/oxideav-wavpack/releases/tag/v0.0.2) - 2026-05-30

### Other

- Round 14: median-adaptation amount (spec §3 + §3.2) — AdaptiveMedians + Zone
- Round 13: end-to-end parse_block aggregate + BitReader peek/skip primitives
- Round 12: 0x0A PackedSamples typed view + BitReader position accessors + channel-indexed EntropyInfo bridges
- Round 11: per-term decorrelation sample-count helper + flat-payload partitioner
- Round 10: MD5 typed view + walker finders + remaining metadata-kind predicates
- Round 9: term-kind classifier + decorrelation/metadata kind accessors
- round-8 block-header accessor coverage (lossless / sample-rate sentinel / experimental / effective bit-depth / audio-block / payload-bytes)
- Round 7: single-call per-sample decode + EntropyInfo→Medians bridge
- Golomb (base, add) interval + sample-value reconstruction (round 6)
- Round 5: sample-coding bit reader + run-length decoder
- Round 4: 0x05 entropy-info sub-block expander
- Round 3: decorrelation sub-block expanders (terms / weights / samples)
- Round 2: metadata sub-block walker
- Round 1: WavPack v.4 block-header parser
- Round 0 — clean-room rebuild scaffold (orphan master)

### Added

- Round 14 — WavPack median-adaptation amount (newly-unblocked spec
  `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §3 + §3.2). New
  `AdaptiveMedians` struct — a three-`u32` running median state with
  the spec §2.1 4-fractional-bit encoding — plus the integer
  increment / decrement primitives that the spec §3 quotes:
  `((median[i] + D) / D) * 5` up and `((median[i] + (D - 2)) / D) * 2`
  down, with `D` = `DIV0` / `DIV1` / `DIV2` (= `128` / `64` / `32`)
  per the spec table. New `Zone` enum names the four §3.2 arms
  (`Zone0` / `Zone1` / `Zone2` / `Zone2Overflow { ones_count }`)
  driven from a raw `ones_count` via `Zone::from_ones_count`, and
  `AdaptiveMedians::adapt(zone)` / `adapt_for_ones_count(ones_count)`
  apply the correct combination of `inc_median` / `dec_median` calls:
  zone 0 → dec `median[0]`; zone 1 → inc `median[0]`, dec `median[1]`;
  zone 2 → inc `median[0]` + `median[1]`, dec `median[2]`; zone 2
  overflow → all three inc. New `AdaptiveMedians::get_med(i)` returns
  the spec §2.1 working median `(median[i] >> 4) + 1` (the value the
  spec §4.2 interval ladder consumes). New seed constructors
  `AdaptiveMedians::from_seed_values([i32; 3])` and
  `AdaptiveMedians::from_medians(Medians)` bridge the round-4 / round-6
  typed views into the running state, returning `None` when any seed
  is negative (rather than silently casting). New public constants
  `DIV0` / `DIV1` / `DIV2` / `MEDIAN_INC_MULTIPLIER` /
  `MEDIAN_DEC_MULTIPLIER` / `GET_MED_SHIFT` / `GET_MED_FLOOR` record
  the spec §3 / §5 numeric facts. The `inc_median` / `dec_median`
  primitives use `saturating_add` / `saturating_sub` / `saturating_mul`
  defensively against pathological starting values — the spec §3
  arithmetic naturally stays within `u32` for any state the per-sample
  decode actually produces, but the saturating cap prevents a stray
  caller from triggering UB. None of this is wired into the round-7
  `decode_sample` call yet — that composition is gated on a follow-up
  round so the existing `Medians`-by-value primitive is preserved
  unchanged and the new `AdaptiveMedians` state is an additive,
  self-contained numeric primitive.
- 24 new unit tests (221 total): divisor / multiplier / GET_MED
  constants matching the spec §3 / §5 table; `get_med` returning the
  §2.1 floor (`1`) at zero, stripping the 4 fractional bits at the
  exemplar values 16/32/48, and truncating (not rounding) on the
  off-boundary 15/31/47 case; `inc_median` stepping by `5` at
  `median = 0` and by `10` at `median = D`, per-index divisor
  selection on indices 1/2; `dec_median` stepping to zero at
  `median = 0` and by `2` at `median = D`, plus an exhaustive sweep
  proving `((v + D - 2) / D) * 2 <= v` for `v ∈ 0..=200` (the §3
  "never below 0" invariant); a 5:2 ratio equilibrium probe at
  `median = 256` cross-checking both the inc step (`15`) and the
  follow-up dec step (`6`); `Zone::from_ones_count` mapping every
  named arm plus the `ones_count >= 3` overflow with the raw value
  preserved (including `3` / `33` / `u32::MAX`), and the round-trip
  through `Zone::ones_count`; `adapt(zone)` matching the §3.2 table
  exactly on each of the four arms (Zone0 dec m0, Zone1 inc m0 + dec
  m1, Zone2 inc m0 + m1 + dec m2, Zone2Overflow inc all three) with
  before/after equality against the primitive sequence;
  `adapt_for_ones_count` threading through to the same Zone branch
  for `ones_count = 1` and `ones_count = 7`; `from_seed_values`
  accepting non-negative seeds and rejecting any negative slot;
  `from_medians` bridging a `Medians` and rejecting a negative slot;
  a four-step §3 sequence (Zone1 → Zone0 → Zone2 → Zone2Overflow)
  walking a fresh `[0,0,0]` state through hand-computed `[5,0,0]` →
  `[3,0,0]` → `[8,5,0]` → `[13,10,5]` confirming every step matches
  the spec arithmetic exactly; saturating semantics on `u32::MAX`
  increment and `0` decrement; and `AdaptiveMedians` is `Copy` /
  `PartialEq` (pre-update vs post-update inequality on a
  `Zone0`-driven decrement).
- Round 13 (end-to-end `parse_block` aggregate + `BitReader` non-mutating
  look-ahead + bulk `skip_bits`): another non-prediction-loop advancement
  while the median-adaptation amount stays a docs gap. New stand-alone
  `parse_block(bytes) -> Result<(WavPackBlock<'_>, &[u8])>` composes
  round-1 `parse_block_header` and round-2 `walk_metadata` into a single
  end-to-end call: parses the 32-byte fixed header, validates that the
  input carries the `8 + ck_size` bytes the wiki "Block structure"
  listing declares, walks the metadata sub-block region against the
  exact byte count `ck_size` advertises (rather than the whole tail),
  and returns the typed `WavPackBlock` aggregate plus the unconsumed
  tail (i.e. the next block in a multi-block `.wv` file). New
  `WavPackBlock<'a>` carries the typed `WavPackBlockHeader` (round 1)
  alongside a `Vec<MetadataSubBlock<'a>>` (round 2; borrowed payload
  slices into the input bytes). New accessors `WavPackBlock::header`,
  `sub_blocks`, `contains_sub_block(id)` (boolean shortcut over
  `find_first` for presence checks), `sub_block_count`,
  `is_metadata_empty` (the `ck_size == 24` header-only edge case the
  wiki allows when `block_samples == 0`), and `on_disk_len` (the
  `8 + ck_size` on-disk extent in bytes — useful for callers stepping
  across blocks without re-parsing the header). New error variant
  `Error::CkSizeExceedsBuffer { ck_size, available }` distinguishes
  "header parses but payload is short" from `Error::Truncated`
  (header-boundary truncation) so a streaming caller can size the next
  read against `8 + ck_size - available`. None of these read sample
  bits, mutate state, or touch the prediction loop; they compose the
  round-1 and round-2 parsers into the one-call surface a streaming
  caller wants.
- New `BitReader` non-mutating look-ahead primitives `peek_bit()` /
  `peek_bits(count)` / `peek_unary()` — read a single bit, a multi-bit
  value, or a unary run-length without advancing the cursor.
  Implemented by reading from a clone of the reader, so the LSB-first
  bit-order rules in `get_bit` / `get_bits` / `get_unary` carry through
  unchanged. On `Error::Truncated` the original reader's cursor is
  unchanged (the truncation hit the clone, not the original) so a
  caller can retry against a freshly-extended buffer without rebuilding
  the reader. Useful for probing the wiki `n == 16` escape pattern (the
  leading unary indicating whether a second unary follows) before
  committing to a real `decode_run_length` call. New bulk
  `BitReader::skip_bits(count)` advances the cursor by `count` bits
  without assembling a `u32`; on `Truncated` the cursor lands at the
  buffer end (matching `get_bits`' partial-consume semantics).
- 24 new unit tests (197 total): `parse_block` returning header + empty
  metadata on `ck_size == 24`, walking a two-sub-block metadata region
  (dummy + MD5) and confirming both walker entries plus the
  `contains_sub_block` / `is_metadata_empty` predicates, chaining two
  back-to-back blocks through the returned tail, rejecting a sub-
  `HEADER_LEN` buffer with `Truncated`, surfacing the new
  `CkSizeExceedsBuffer { ck_size, available }` variant on a header
  advertising a longer payload than the buffer (with both fields
  asserted), propagating `InvalidMagic` / `InvalidCkSize` from the
  header parser, propagating `Truncated` from the metadata walker on a
  malformed sub-block, `on_disk_len` equalling `8 + ck_size` and the
  underlying byte count, `contains_sub_block` returning false on
  header-only blocks, and `sub_block_count` matching the walker output
  count on a four-sub-block block; `peek_bit` returning the next
  LSB-first bit without advancing (cursor stays put; follow-up
  `get_bit` returns the same value), `peek_bit` `Truncated` on an
  empty buffer leaving the cursor untouched, `peek_bits` assembling
  4 LSB-first bits of `0x0A` into `0xA` without advancing,
  `peek_bits(0)` returning zero without advancing, `peek_bits(9)` on
  an 8-bit buffer reporting `Truncated` with the cursor unchanged,
  `peek_unary` matching `get_unary` on the wiki `111110b → 5` example
  without advancing, `peek_unary` reporting `Truncated` on an
  unterminated run with the cursor unchanged, the peek-then-get
  pattern returning matching values across a 4-bit window;
  `skip_bits` advancing the cursor without assembling a value (with
  the expected `bits_consumed` / `byte_position` / `bit_position`
  after a 5-bit skip and the next `get_bits(3)` reading the remaining
  bits), `skip_bits(0)` no-op, a 10-bit cross-byte skip landing at
  `byte_position == 1` / `bit_position == 2`, `skip_bits(9)` on an
  8-bit buffer reporting `Truncated` with the cursor at the buffer
  end, and a `skip_bits`-then-`get_unary` resume reading the second
  of two back-to-back unary runs.
- Round 12 (`0x0A` packed-samples typed view + `BitReader` position
  accessors + channel-indexed `EntropyInfo` / `Medians::from_entropy`
  bridges): another non-prediction-loop advancement while the
  median-adaptation amount stays a docs gap. New `PackedSamples<'a>`
  typed view of the `0x0A` packed-samples sub-block payload — the
  entropy-coded audio bitstream the wiki "Samples coding" section
  consumes — exposing `bytes()` / `len()` / `is_empty()` introspection
  and a `bit_reader()` factory that produces a fresh `BitReader`
  positioned at bit 0 for feeding `decode_run_length` /
  `decode_sample_value` / `decode_sample`. New stand-alone
  `expand_packed_samples(payload: &[u8]) -> PackedSamples<'_>` mirrors
  the round-3/round-4 expanders' naming (typed wrap rather than a
  byte-by-byte decode because the wiki places no internal structure on
  the `0x0A` payload). New walker finder
  `find_packed_samples(&[MetadataSubBlock]) -> Option<PackedSamples>`
  is the typed counterpart to `find_audio_payload`. New cursor
  accessors `BitReader::byte_position()` / `bit_position()` /
  `bits_consumed()` name the reader's position in the underlying byte
  slice, with `bits_consumed` clamping at the buffer length when the
  reader has advanced past the end. New `EntropyInfo::is_stereo()` /
  `channels()` / `medians_for_channel(idx)` typed channel introspection
  pinning the wiki "one or two sets of medians" sentence as `1` or `2`
  populated sets, with a channel-indexed median getter returning
  `Some([m0, m1, m2])` for `0` (left/mono) or `1` (right, stereo only)
  and `None` for `1` on a mono payload and indices `>= 2`. New
  `Medians::from_entropy(info, channel_idx)` channel-indexed bridge
  with the same mono / out-of-range guards so callers iterating
  per-channel medians skip the hand-rolled mono / stereo branch. None
  of these read bits, mutate state or touch the prediction loop; they
  elaborate the round-2 walker output and the round-4 / round-5/6/7
  expanders for callers staging the deferred decode pass.
- 23 new unit tests (173 total): `PackedSamples` constructor
  round-tripping a non-empty payload, the zero-byte empty payload
  accepted and reported empty, `expand_packed_samples` round-tripping
  the byte slice, `bit_reader()` starting at byte/bit 0 with the full
  payload remaining and yielding the first bit LSB-first, the
  `bit_reader()` factory returning independent readers (so a probe and
  the real decode each start at bit 0), an empty packed-samples view
  reporting immediate `Error::Truncated` on any read, and the view
  being `Copy`; `BitReader::byte_position` / `bit_position` /
  `bits_consumed` tracking 13-bit consumption across a byte boundary,
  the `bits_consumed` clamp when the reader is past the end, and the
  cursor staying put when a read errors with `Truncated`;
  `Medians::from_entropy` yielding the left set on index `0` and the
  right set on index `1` for a stereo `EntropyInfo`, returning `None`
  for `1` on a mono `EntropyInfo`, and rejecting out-of-range indices
  (`2`, `3`, `255`); `EntropyInfo::is_stereo` inverting `is_mono`,
  `channels` returning `1` for mono and `2` for stereo,
  `medians_for_channel` yielding the matched set for `0` / `1` on
  stereo and `None` for `1` on mono / `2+` indices; and
  `find_packed_samples` returning a typed `PackedSamples` view over
  the synthesised `0x0A` payload and `None` on a stream without one.
- Round 11 (per-term decorrelation-sample-count helper + flat-payload
  partitioner): another non-prediction-loop advancement while the
  median-adaptation amount stays a docs gap. New stand-alone
  `decorrelation_sample_count(code: i8) -> Option<u8>` and matching
  `TermKind::decorrelation_sample_count()` method derive the per-term
  `0x04` seed-sample count from the wiki "Possible predictor values"
  listing: `Some(code - 5)` for the `6..=12` sample-based predictors
  (one per previous-sample slot the wiki cites in "uses 1-7 samples
  for prediction") and `Some(2)` for the `17..=18` two-sample
  predictors. Stereo predictors `0..=5`, the reserved `13..=16`
  range, and codes outside `0..=18` all return `None` — the wiki does
  not give a per-term count for them. New public constant
  `MAX_DECORRELATION_SAMPLES_PER_TERM = 16` records the wiki
  "Decorrelation samples" upper bound ("Each decorrelation term may
  have up to 16 samples depending on its value") for callers checking
  future docs additions against it.
- New `DecorrelationTerms::expected_decorrelation_sample_count()`
  sums the per-term counts above across a `(0x02)` term list and
  returns `Some(total)` when every term is documented, `None` as
  soon as any term lands in the docs gap. An empty term list returns
  `Some(0)` (vacuous).
- New `partition_decorrelation_samples(&DecorrelationTerms,
  &DecorrelationSamples) -> Result<Vec<Vec<i32>>>` splits the flat
  sample list `expand_samples` produces into one `Vec<i32>` per term
  in wiki order, with the per-term length given by the new
  `decorrelation_sample_count` helper. Two new `Error` variants
  back it: `DecorrelationSampleCountUnspecified(i8)` carries the
  offending term code when a term has no wiki-documented per-term
  count (so partitioning cannot proceed), and
  `DecorrelationSampleCountMismatch { expected, actual }` fires
  when the summed expected count does not equal the flat payload
  length. None of these read bits, mutate state, or advance the
  prediction loop; they elaborate the round-3 expander output for
  the documented predictor codes and explicitly refuse the
  docs-gap codes rather than guessing a per-term length.
- 15 new unit tests (150 total): `decorrelation_sample_count`
  returning the matching `code - 5` across the full `6..=12`
  sample-based range, `Some(2)` for `17` / `18`, and `None` across
  stereo `0..=5`, reserved `13..=16`, undocumented `19..=31`, and
  the negative-code defensive case; `MAX_DECORRELATION_SAMPLES_PER_TERM`
  wiki upper-bound sanity sweep across every documented count;
  `TermKind::decorrelation_sample_count` mirror of the stand-alone
  helper; `DecorrelationTerms::expected_decorrelation_sample_count`
  summing `[6, 8, 17, 12]` to `13`, the vacuous empty-list `0`, and
  `None` propagation across stereo / reserved / undocumented term
  presence; `partition_decorrelation_samples` splitting a
  `[6, 8, 17]` term list with matching flat input in term order,
  the empty-terms-empty-payload base case, rejecting the stereo
  `[2]` and reserved `[6, 14]` lists with
  `DecorrelationSampleCountUnspecified`, rejecting both short
  (`expected: 6, actual: 5`) and long (`expected: 1, actual: 4`)
  flat payloads with `DecorrelationSampleCountMismatch`, and a
  round-trip from `expand_samples` of a synthesised `[6, 18]`
  exponent-9 wire through the partitioner back to per-term
  `[1]` + `[2, 3]` lists.
- Round 10 (metadata sub-block extras: MD5 typed view + walker finders
  + remaining payload-kind predicates + `SubBlockId` classifiers):
  another non-prediction-loop advancement while the median-adaptation
  amount remains a docs gap. New `Md5Checksum([u8; 16])` typed view of
  the `0x26` sub-block payload — the wiki "16-byte MD5 sum of raw audio
  data" — together with `parse_md5_checksum(&[u8]) -> Result<Md5Checksum>`
  enforcing the fixed 16-byte length and a new
  `Error::Md5ChecksumLength(usize)` variant rejecting other lengths.
  Public constant `MD5_DIGEST_BYTES = 16`. New walker finders over the
  round-2 `Vec<MetadataSubBlock>`: `find_first(subs, id)` linear scan
  by `SubBlockId`; specialised wrappers `find_audio_payload` (`0x0A`),
  `find_entropy_info` (`0x05`), `find_md5_checksum_block` (`0x26`),
  `find_multichannel_info` (`0x0D`); and `find_decorrelation_triple`
  which returns the three sub-blocks `(0x02, 0x03, 0x04)` in wiki order
  or `None` if any one is missing. Eight new `MetadataSubBlock`
  payload-kind predicates filling out the wiki "IDs" listing:
  `is_dummy_payload` (`0x00`), `is_hybrid_profile_payload` (`0x06`),
  `is_float_payload` (`0x08`), `is_int32_payload` (`0x09`),
  `is_overflow_bits_payload` (`0x0C`), `is_multichannel_info_payload`
  (`0x0D`), `is_encoding_details_payload` (`0x25`), `is_md5_payload`
  (`0x26`), `is_sample_rate_payload` (`0x27`). Four new `SubBlockId`
  classifiers — `is_decorrelation`, `is_correction_stream`,
  `is_riff_wrapper`, `is_audio` — for callers that branch on the ID
  enum directly rather than on a sub-block. None of these read bits,
  mutate state, or touch the prediction loop; they elaborate the
  round-2 walker output for callers staging the deferred decode pass.
- 17 new unit tests (135 total): `SubBlockId` classifier coverage
  across all four buckets (decorrelation `0x02/0x03/0x04`,
  correction-stream `0x07/0x0B`, RIFF wrapper `0x20/0x21` with
  same-family `0x25/0x26/0x27` negative cases, audio-only `0x0A`);
  one-hot kind-predicate sweep across the eight new MetadataSubBlock
  predicates (with the four "main bucket" predicates pinned false on
  each); `is_md5_payload` discriminating `0x06` HybridProfile from
  `0x26` Md5Checksum on the low-5-bit overlap; `is_dummy_payload`
  discriminating `0x00` Dummy from `0x20` RiffHeader on the same
  overlap; `parse_md5_checksum` accept (MD5 of `""` test vector) and
  reject (0/15/17/64-byte rejections); end-to-end round-trip from a
  synthesised `0x26` sub-block through `walk_metadata` → `find_md5_*`
  → `parse_md5_checksum` (MD5 of the "quick brown fox" test vector);
  walker finder coverage for `find_first` (hit + miss), the four
  specialised finders, and `find_decorrelation_triple` (full triple
  hit + miss when either of weights / samples is dropped).
- Round 9 (decorrelation-term classification + metadata-payload kind
  predicates): non-prediction-loop advancement while the median-
  adaptation amount remains a docs gap. New `TermKind` enum classifies
  the wiki "Possible predictor values" listing (stereo `0..=5` with the
  `2..=4` implemented subset, sample-based `6..=12` with the per-code
  `previous_samples()` count = `code - 5`, reserved `13..=16`,
  two-sample `17..=18`, and `Unknown` for codes outside the documented
  range). New `TermKind::is_implemented()` / `previous_samples()`
  accessors surface the wiki's two narrowings (the stereo "only
  predictors 2-4 are implemented" subset and the per-code sample count).
  New `DecorrelationTerms` accessors: `len()`, `is_empty()`,
  `kind_at(idx)`, `iter_kinds()` (zips term code with its classified
  kind), `all_implemented()`, `has_reserved()`. New stand-alone
  `weights_per_term(channels: u8) -> u8` helper exposes the wiki
  "Each decorrelation term should have one or two weights depending on
  channels" split. None of these reads bits, mutates state, or touches
  the prediction loop; they classify the round-3 expander output for
  callers staging the deferred prediction pass.
- New `MetadataSubBlock` payload-kind predicates derived from the wiki
  "IDs" listing: `is_optional()` (wraps the `0x20` flag for callers
  branching on the sub-block value directly), `is_decorrelation_payload()`
  (`0x02` / `0x03` / `0x04`), `is_correction_payload()` (the `.wvc`
  pair: `0x07` noise-shaping profile + `0x0B` packed correction data),
  `is_audio_payload()` (`0x0A` packed samples), and
  `is_riff_payload()` (`0x20` / `0x21` RIFF header / trailer). These
  let a caller pick the decorrelation triple or the audio stream out
  of a walk without re-matching the `SubBlockId` enum.
- 15 new unit tests (118 total): `TermKind::from_code` across all four
  wiki categories — stereo implemented `2..=4`, stereo unimplemented
  `0/1/5`, sample-based `6..=12` with per-code sample count, reserved
  `13..=16`, two-sample `17..=18`, and unknown `19..=31` plus a
  negative-code defensive check; `DecorrelationTerms::len` /
  `is_empty` / `kind_at` / `iter_kinds` / `all_implemented` /
  `has_reserved` over mixed term lists including the vacuous empty
  case; `weights_per_term` matching the wiki mono / stereo split with
  a 0-channel and 3+-channel clamp; `MetadataSubBlock::is_optional`
  pinning the `0x20` bit; per-kind payload predicates round-tripping
  for `0x02` / `0x03` / `0x04` decorrelation, `0x07` / `0x0B`
  correction, `0x0A` audio, and `0x20` / `0x21` RIFF (with non-RIFF
  optional sub-block negative case).
- Round 8 (header accessor coverage): non-prediction-loop advancement
  while the median-adaptation amount remains a docs gap. Eleven
  block-header convenience accessors derived rigorously from the wiki
  "Flags meaning" / "Block structure" listings. On `Flags`:
  `is_standalone_block` / `is_multichannel_member` (degenerate-marker
  `0b11` vs everything else, exposing the wiki "multi-channel start
  and end blocks" pair); `is_lossless` / `is_lossy` (around the wiki
  bit 3 "hybrid profile (lossy compression)" label); `has_custom_sample_rate`
  (wiki bits 23..=26 sentinel `15` = "unknown/custom"); `should_skip_decode`
  (wiki bit 31 "do not decode if encountered" — bit 28 "okay to ignore"
  deliberately excluded); `is_experimental` (union of the two wiki-
  labelled experimental bits 28 + 31, diagnostic-only);
  `effective_bit_depth` (container width minus `left_shift`, saturating
  per the wiki "12-bit / 20-bit" worked examples). On `WavPackBlockHeader`:
  `is_audio_block` (wiki "may be 0 if no audio present" on `block_samples`),
  `is_total_samples_known` (sentinel-vs-real on `total_samples`), and
  `payload_bytes` (the metadata-region length advertised by `ck_size`).
  None of these touch the prediction loop or the median-adaptation
  amount — those remain gated on the open docs gap.
- 11 new unit tests (103 total): standalone vs multichannel marker
  matrix (all four `0b00..=0b11` combinations); lossless / lossy
  predicate symmetry around the hybrid bit; `has_custom_sample_rate`
  sweep across all 16 sample_rate_index values pinning the sentinel
  to `15`; `should_skip_decode` discriminating bit 31 from bit 28;
  `is_experimental` union of the two experimental bits;
  `effective_bit_depth` for the wiki 12-bit / 20-bit worked examples
  plus the no-shift baseline plus the `left_shift > container_bits`
  saturation; `is_audio_block` keyed on a non-zero `block_samples`;
  `is_total_samples_known` distinguishing the `0xFFFF_FFFF` sentinel
  from real counts (including the boundary `0`); `payload_bytes`
  subtracting the 24-byte fixed-header floor.
- Round 7: WavPack v.4 per-sample single-call decode + entropy-info →
  median bridge. New public items in the `samples` module: `decode_sample`
  fuses `decode_run_length` and `decode_sample_value` into one call per
  sample, matching the wiki's contiguous "Samples coding" pseudocode
  block (carries the adaptive `RunState`, takes `Medians` by value).
  `Medians::from_entropy_left` and `Medians::from_entropy_right` pull a
  channel's three medians directly out of a round-4 `EntropyInfo` value
  so the entropy-info expander output feeds the Golomb decoder without
  the caller re-typing the array. No median is mutated — the median
  adaptation amount remains the open docs gap blocking the multi-sample
  payload loop.
- 8 new unit tests (92 total): `Medians::from_entropy_left` and
  `from_entropy_right` for stereo input, the mono case (right set zeroed),
  `decode_sample` end-to-end through the run-length-then-value chain,
  `last_zero` short-circuit honoured (no unary bits consumed),
  degenerate-interval and truncation error propagation, and the
  `EntropyInfo` → `Medians` → `decode_sample` full chain.
- Round 6: WavPack v.4 Golomb sample-value reconstruction — the value
  part of the wiki "Samples coding" second half. New public items in
  the `samples` module: `Medians` (a channel's three `median[0..=2]` in
  wiki order, `Copy`), `GolombInterval { base, add }`, `golomb_interval`
  (pure `n` + `Medians` → `(base, add)` per the wiki's `n == 0` /
  `n == 1` / `n >= 2` branch — reads no bits, mutates no median), and
  `decode_sample_value` which picks the interval, reads `getbits(k - 1)`
  with `k = log2(add)`, applies the `if(t2 >= ex) t2 = t2*2 - ex +
  getbit()` fixup, reads the sign bit, and returns `base + t2` (or its
  ones-complement when sign is set). `k` is the **bit-length** of `add`
  — the only `log2` reading that keeps the wiki's own
  `ex = (1 << k) - add - 1` non-negative (a derivation from the wiki's
  next two lines, not an external reference; documented as a resolved
  docs gap). `decode_sample_value` takes `Medians` **by value** and does
  not mutate them: the median "increase" / "decrease" *amount* is still
  unspecified by the wiki, so the stateful loop over a whole `0x0A`
  payload is deferred. The degenerate `add == 0` interval (a median of
  `1`, where `log2(0)` / `getbits(-1)` are undefined) returns the new
  `Error::GolombDegenerateInterval(add)` rather than guessing.
- 15 new unit tests (84 total): `golomb_interval` across the three `n`
  branches (n0 → median[0], n1 → median[1], n2 → median sum with zero
  extra, large-n median[2] scaling); `golomb_k` bit-length values and a
  sweep proving `ex >= 0` across `add` 1..=1024; `decode_sample_value`
  short-mantissa (no extra bit), long-mantissa (`t2 >= ex` extra bit),
  `ex == 0` always-long branch, positive and ones-complement sign,
  `n == 0` interval, degenerate `add == 0` rejection (no bits consumed),
  mantissa- and sign-truncation reporting, and an end-to-end compose of
  `decode_run_length` then `decode_sample_value` over one contiguous
  bitstream.
- Round 5: WavPack v.4 sample-coding bit reader + run-length decoder
  (first half of the wiki "Samples coding" section). A new `samples`
  module adds `BitReader` — a least-significant-bit-first reader over
  the `0x0A` packed-samples payload exposing the three wiki primitives
  `get_unary()` (count of leading `1` bits up to the terminating
  `0`), `get_bit()` and `get_bits(n)` (assembled LSB-first into a
  `u32`, `get_bits(0) == 0`). Reads past the buffer report
  `Error::Truncated` rather than zero-filling, and `bits_remaining()`
  / `is_empty()` expose the cursor. `decode_run_length` transcribes
  the wiki pseudocode's run-length half: it short-circuits to `n = 0`
  when the carried `last_zero` flag is set, otherwise reads the unary
  prefix, applies the `n == 16` escape (second unary `n2`; `n += n2`
  for `n2 < 2` else `n += (1 << (n2-1)) | getbits(n2-1)`), halves `n`
  with the odd-round-up rule, and updates the adaptive `last_zero` /
  `last_one` carry held in a new `RunState`. Public constant
  `UNARY_ESCAPE = 16`. The second half (Golomb `(base, add)` interval
  selection + median adaptation) is deferred: the wiki names the
  median "increase" / "decrease" steps without quantifying them, so
  per-sample reconstruction is not yet bit-exact from this page —
  documented as a docs gap on the `samples` module. The `0x0A` bit
  order (LSB-first, matching WavPack's little-endian container) is
  likewise a documented assumption pending a real-payload check.
- 19 new unit tests (69 total): LSB-first `get_bit` ordering and
  byte-boundary crossing; `get_bits` LSB-first assembly, zero-count
  no-op, full 32-bit width and truncation; `get_unary` against the
  wiki worked examples (`111110b → 5`, `10b → 1`), the immediate-
  terminator zero run, terminator-only consumption, and unterminated
  truncation; and the run-length decoder's `last_zero` short-circuit
  (no bits consumed, flag cleared), even / odd unary halving with the
  matching `last_one` / `last_zero` carry, both escape arms
  (`n2 < 2` direct add and `n2 >= 2` LSB-first mantissa, two distinct
  mantissa values), the all-zero unary case, the multi-sample adaptive
  carry sequence, and unary truncation.

- Round 4: WavPack v.4 entropy-info sub-block expander.
  `expand_entropy` decodes the `0x05` payload into an
  `EntropyInfo { medians_left: [i32; 3], medians_right: [i32; 3] }`
  per the wiki "Entropy info" section ("one or two sets of medians …
  log-packed into 16 bits as described above"). Each median is a
  little-endian 16-bit log-packed word in the same format as the
  round-3 decorrelation-samples expander (`[mantissa_lo, exponent_hi]`,
  mantissa signed, exponent biased by `-9`); the shared expander is
  re-used via a new `pub(crate)` accessor on `decorrelation`. Mono
  payloads (6 bytes) populate `medians_left` and leave `medians_right`
  at `[0; 3]`; stereo payloads (12 bytes) populate both. A convenience
  `EntropyInfo::mono(...)` constructor and `is_mono()` predicate are
  exposed. Any other payload length is rejected through a new
  `Error::EntropyInfoLength(usize)` variant. Public constants
  (`MEDIANS_PER_CHANNEL`, `MEDIAN_ON_WIRE_BYTES`,
  `MONO_PAYLOAD_BYTES`, `STEREO_PAYLOAD_BYTES`) document the wire
  layout.
- 12 new unit tests (50 total): mono single-set decode + right-set-
  zeroed contract; stereo two-set decode in left-then-right order;
  signed-mantissa sign-extension on negative medians; both the
  shift-left and shift-right log-pack branches reach the median
  values; empty / sub-mono / between-mono-and-stereo / over-stereo
  length rejections; `EntropyInfo::mono` helper; `is_mono()`
  predicate returning false on any non-zero right-set median;
  all-zero stereo payload decoded sanely.

- Round 3: WavPack v.4 decorrelation sub-block expanders.
  `expand_terms` decodes the `0x02` payload into a
  `DecorrelationTerms { terms: Vec<i8>, deltas: Vec<u8> }`, one
  byte → one `(term, delta)` pair per the wiki "lower 5 bits
  indicate predictor type, high 3 bits contain delta value" rule.
  `expand_weights` decodes the `0x03` payload through the wiki's
  log-pack recipe (`n = getchar() << 3; if (n > 0) n += (n + 64) >> 7`
  with the byte read as signed) into a
  `DecorrelationWeights { weights: Vec<i32> }`. `expand_samples`
  reads the `0x04` payload as little-endian 16-bit words and applies
  the wiki exponent / mantissa expansion (mantissa signed; exponent
  biased by `-9`) into a `DecorrelationSamples { samples: Vec<i32> }`.
  Out-of-range shift counts on the sample expander saturate rather
  than panicking. A new `Error::DecorrelationSamplesOddByteCount`
  variant rejects malformed sample payloads whose byte count is not
  a multiple of two.
- 15 new unit tests (38 total): low-5/high-3 term-byte split across
  the full `0..=18` predictor range; multi-byte term order
  preservation; zero / positive-rounding / negative-no-rounding
  weight expansions and multi-byte order preservation; sample
  expansion for the `exponent == 9` / `< 9` / `> 9` branches with
  positive and negative mantissas; byte-pairing across multi-sample
  payloads; empty-payload handling for all three expanders;
  saturation behaviour for extreme exponents; rejection of odd-byte
  sample payloads.

- Round 2: WavPack v.4 metadata sub-block walker. `walk_metadata`
  consumes the post-header payload returned by
  `parse_block_header` and returns `Vec<MetadataSubBlock>`,
  driving `parse_metadata_sub_block` (also exposed for callers
  that want to validate against `ck_size` themselves). The
  `SubBlockId` enum names every documented sub-block ID listed in
  the wiki "IDs" section: `0x00..=0x0D`
  (`Dummy`/`DecorrelationTerms`/`DecorrelationWeights`/`DecorrelationSamples`/`EntropyInfo`/`HybridProfile`/`NoiseShapingProfile`/`FloatInfo`/`Int32Info`/`PackedSamples`/`PackedCorrectionData`/`PackedOverflowBits`/`MultichannelInfo`)
  plus `0x20..=0x27`
  (`RiffHeader`/`RiffTrailer`/`EncodingDetails`/`Md5Checksum`/`NonStandardSampleRate`).
  Undocumented IDs surface as `Unknown(u8)` rather than erroring
  (the wiki's `0x20` "decoder may ignore" flag is the forward-
  compat mechanism). `SubBlockFlags` decodes the structural flag
  triple (`0x20` optional / `0x40` odd-size / `0x80` large-size);
  the walker handles both the 1-byte and 3-byte (large-flag)
  size-field encodings, strips the trailing odd-size padding byte,
  and reports truncated input as `Error::Truncated`. Two new
  `Error` variants — `MetadataSubBlockTooLarge` and
  `MetadataOddSizeWithoutPayload` — cover the remaining structural
  rejections.
- 13 new unit tests (23 total): flag-triple decoding, the full
  18-entry round-trip through `SubBlockId::from_id_byte` /
  `as_id_byte`, small-format and large-format sub-block parsing,
  odd-size padding strip (small + large), back-to-back walk-to-
  exhaustion, RIFF-wrapper / non-standard-rate sub-block decode,
  unknown-ID acceptance, truncated-input rejection, and the
  zero-word-with-odd-flag rejection.

- Round 1: WavPack v.4 block-header parser. `parse_block_header`
  returns a typed `WavPackBlockHeader` (magic, ck_size, version,
  track_number, track_sub_index, total_samples, block_index,
  block_samples, flags, crc) plus the unconsumed payload tail. The
  32-bit flag word is decoded into a typed `Flags` view exposing
  every bit-range named on the wiki "Flags meaning" listing
  (bytes-per-sample, mono / hybrid / joint-stereo / cross-channel
  decorrelation / hybrid-shaping / float / int32 / hybrid-profile /
  multi-channel start-end markers / left-shift / max-magnitude /
  sampling-rate index / reserved bit 27 / robust / hybrid IIR
  noise-shaping / false-stereo / low-latency). Validates the
  `'wvpk'` magic, the `ck_size >= 24` minimum, and the
  `0x0402..=0x0410` version window. Sample decode, decorrelation,
  entropy and metadata-sub-block walking remain out of scope.
- 10-test unit suite covering the accept / reject boundaries of
  each header field plus an exhaustive bit-by-bit layout check
  on the flag word and a little-endian preamble check.

### Changed

- Clean-room rebuild from a fresh orphan `master`. The previous
  implementation was retired by the OxideAV docs audit dated
  2026-05-06; the prior history is preserved on the `old` branch.
  See `README.md` for the rebuild scope and the strict-isolation
  workspace the Implementer rounds will draw from.
