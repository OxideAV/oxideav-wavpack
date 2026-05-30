# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
