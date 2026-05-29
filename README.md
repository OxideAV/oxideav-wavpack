# oxideav-wavpack

Pure-Rust WavPack lossless audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 12 — block-header parser + metadata sub-block walker +
decorrelation sub-block expanders + entropy-info expander +
sample-coding bit reader, run-length decoder, Golomb sample-value
reconstruction & single-call per-sample decode + entropy→median
bridge + 11 header-accessor helpers + `TermKind` classifier + 7
`DecorrelationTerms` accessors + `weights_per_term` mono/stereo split
+ wiki-derived per-term `decorrelation_sample_count` + flat-to-per-term
`partition_decorrelation_samples` splitter (with explicit refusal on
the stereo / reserved / undocumented per-term-count docs gap) + 13
`MetadataSubBlock` payload-kind predicates covering every wiki "IDs"
entry + 4 `SubBlockId` classifier helpers + 7 walker finders
(`find_first` + four specialised + decorrelation-triple +
typed-`PackedSamples` packed-samples finder) + 16-byte `Md5Checksum`
typed view with strict-length `parse_md5_checksum` + typed
`PackedSamples` view of the `0x0A` packed-samples sub-block payload
(borrowed bytes + `bit_reader()` factory at bit 0) + `BitReader`
position accessors (`byte_position` / `bit_position` /
`bits_consumed`) + channel-indexed `EntropyInfo` accessors
(`is_stereo` / `channels` / `medians_for_channel`) +
`Medians::from_entropy(info, channel_idx)` channel-indexed bridge.** Round 1
landed the 32-byte fixed block-header parser; round 2 added the
metadata sub-block walker completing the structural pass over a
WavPack v.4 block; round 3 turns the `0x02` / `0x03` / `0x04`
decorrelation sub-block payloads into typed views (terms, log-pack-
expanded weights, exponent / mantissa samples); round 4 adds the
`0x05` entropy-info sub-block (one or two sets of three 16-bit
log-packed medians, decoded through the same log-pack as the round-3
sample expander); round 5 lands the first half of the wiki "Samples
coding" section — a least-significant-bit-first `BitReader`
(`get_unary` / `get_bit` / `get_bits`) and the `decode_run_length`
state machine that turns the unary prefix (with the `n == 16` escape)
into the run-length index `n`, carrying the adaptive `last_zero` /
`last_one` flags; round 6 lands the value part of the second half —
`golomb_interval` maps `n` + a three-median set onto the wiki
`(base, add)` interval, and `decode_sample_value` reads `getbits(k-1)`
(with `k = log2(add)` under the bit-length reading the wiki's own
`ex >= 0` requirement forces), the `t2 >= ex` extra bit, and the sign,
returning the reconstructed sample; round 7 fuses the two halves into
`decode_sample` (one call per sample, matching the wiki's single
pseudocode block) and adds `Medians::from_entropy_left` /
`Medians::from_entropy_right` so the round-4 `EntropyInfo` expander
output feeds the round-6 Golomb decoder directly. The medians are
still taken by value and **not** mutated — the median-adaptation
*amount* is still an open docs gap. All work follows
`docs/audio/wavpack/wiki/WavPack.wiki` (the local snapshot of the
multimedia.cx WavPack reference page).

Public API:

- [`parse_block_header`] — accepts `&[u8]`, returns
  `(WavPackBlockHeader, &[u8])` on success (header + unconsumed
  payload tail). Errors on truncated input, wrong magic,
  undersized `ck_size`, or out-of-range stream version.
- [`WavPackBlockHeader`] / [`Flags`] — typed views of the fixed
  header (round 1).
- [`walk_metadata`] — accepts the post-header payload and returns
  `Vec<MetadataSubBlock>` typed by the wiki "IDs" listing.
- [`parse_metadata_sub_block`] — single-step walker for callers
  that want to drive the walk themselves (e.g. while validating
  against `ck_size`).
- [`SubBlockId`] — enum naming every documented sub-block ID
  (`0x00..=0x0D` audio / decorrelation IDs + `0x20..=0x27` RIFF /
  encoding-details / MD5 / non-standard-rate IDs). Unknown IDs
  surface as `Unknown(u8)` rather than erroring — the wiki's
  `0x20` "decoder may ignore" flag is the forward-compat
  mechanism.
- [`SubBlockFlags`] — typed view of the structural flag bits
  (`0x20` optional / `0x40` odd-size / `0x80` large-size) decoded
  from the on-disk ID byte. Odd-size payload padding is stripped
  before the payload slice is returned.
- [`expand_terms`] — converts a `0x02` payload into a
  `DecorrelationTerms { terms: Vec<i8>, deltas: Vec<u8> }`,
  one byte → one `(term, delta)` pair per the wiki "lower 5 bits
  indicate predictor type, high 3 bits contain delta value"
  sentence.
- [`expand_weights`] — converts a `0x03` payload into a
  `DecorrelationWeights { weights: Vec<i32> }`, applying the
  wiki log-pack expansion
  `n = getchar() << 3; if (n > 0) n += (n + 64) >> 7` to every
  byte (signed-byte interpretation).
- [`expand_samples`] — converts a `0x04` payload into a
  `DecorrelationSamples { samples: Vec<i32> }`, reading
  little-endian 16-bit words and applying the wiki exponent /
  mantissa expansion (mantissa signed; exponent biased by `-9`).
- [`expand_entropy`] — converts a `0x05` payload into an
  `EntropyInfo { medians_left: [i32; 3], medians_right: [i32; 3] }`,
  reading three (mono / 6-byte) or six (stereo / 12-byte) 16-bit
  log-packed medians via the same word format as `expand_samples`.
  Mono payloads leave `medians_right` at `[0; 3]`. Other lengths are
  rejected as malformed via `Error::EntropyInfoLength`.
- [`BitReader`] — least-significant-bit-first reader over a `0x0A`
  packed-samples payload exposing the three wiki "Samples coding"
  primitives `get_unary()` (count of leading `1` bits), `get_bit()`
  and `get_bits(n)` (LSB-first into a `u32`). Reads past the buffer
  report `Error::Truncated` rather than zero-filling.
- [`decode_run_length`] — the first half of the wiki "Samples coding"
  pseudocode: turns the unary prefix (with the `n == 16` escape that
  reads a second unary `n2` and folds in `getbits(n2-1)`) into the
  halved run-length index `n`, carrying the adaptive `last_zero` /
  `last_one` state in a `RunState`.
- [`Medians`] — a channel's three medians (`median[0..=2]`) in wiki
  order, as the `0x05` entropy-info expander produces them.
- [`golomb_interval`] — pure `n` + `Medians` → `GolombInterval`
  `(base, add)` mapping per the wiki's three-way branch
  (`n == 0` / `n == 1` / `n >= 2`). Reads no bits and mutates no
  median.
- [`decode_sample_value`] — the value part of the wiki second half:
  picks the `(base, add)` interval, then reads `getbits(k - 1)` (with
  `k` = bit-length of `add`, the only `log2` reading that keeps the
  wiki's own `ex = (1 << k) - add - 1` non-negative), applies the
  `t2 >= ex` extra-bit fixup, reads the sign, and returns
  `base + t2` (or its ones-complement when the sign bit is set). Takes
  `Medians` **by value** and does not mutate them — the median
  "increase" / "decrease" *amount* is still a docs gap, so the stateful
  full-payload loop is deferred. The degenerate `add == 0` interval
  (a median of `1`, where `log2(0)` / `getbits(-1)` are undefined)
  returns `Error::GolombDegenerateInterval` rather than a guessed
  value.
- [`decode_sample`] — fuses the run-length and value halves into one
  per-sample call, matching the wiki's contiguous pseudocode block:
  reads the unary prefix (and `n == 16` escape) through
  `decode_run_length`, then the Golomb mantissa / sign through
  `decode_sample_value`. Carries the adaptive `RunState` and takes the
  three medians by value — the median-adaptation *amount* docs gap still
  blocks the stateful payload loop.
- [`Medians::from_entropy_left`] / [`Medians::from_entropy_right`] —
  pull a channel's three medians straight out of a round-4
  `EntropyInfo` value so the entropy-info expander output feeds the
  Golomb decoder without the caller re-typing the array.
- [`Flags::is_lossless`] / [`Flags::is_lossy`] — symmetric predicates
  around the wiki bit 3 "hybrid profile (lossy compression)" label.
- [`Flags::has_custom_sample_rate`] — `true` when bits 23..=26 hold
  the wiki sentinel `15` ("unknown/custom"); when set, the actual
  rate is in metadata sub-block `0x27`.
- [`Flags::should_skip_decode`] — surfaces the wiki bit 31 "do not
  decode if encountered" decode-gating instruction; bit 28
  ("experimental, okay to ignore") is deliberately **not** included.
- [`Flags::is_experimental`] — diagnostic union of the two wiki-
  labelled experimental bits (28 + 31).
- [`Flags::effective_bit_depth`] — `bytes_per_sample * 8 - left_shift`
  per the wiki "12-bit / 20-bit" worked examples; saturates to `0`
  rather than underflowing on a malformed `left_shift > container_bits`.
- [`Flags::is_standalone_block`] / [`Flags::is_multichannel_member`]
  — distinguishes the wiki "multi-channel start and end blocks"
  degenerate `0b11` marker (a plain stereo file's single-block
  set) from any other marker combination (which signals participation
  in a multi-block channel grouping).
- [`WavPackBlockHeader::is_audio_block`] — `block_samples > 0`,
  per the wiki "may be 0 if no audio present" note.
- [`WavPackBlockHeader::is_total_samples_known`] — distinguishes the
  wiki [`TOTAL_SAMPLES_UNKNOWN`] sentinel from a real count.
- [`WavPackBlockHeader::payload_bytes`] — bytes of metadata sub-block
  payload the `ck_size` field advertises (`ck_size - 24`).
- [`TermKind`] — typed classification of a decorrelation predictor
  code per the wiki "Possible predictor values" listing: `Stereo
  { implemented }` (`0..=5`, with `2..=4` flagged as implemented),
  `SampleBased { sample_count }` (`6..=12`, count = `code - 5`),
  `Reserved` (`13..=16`), `TwoSample` (`17..=18`), and `Unknown` for
  codes outside the documented range. [`TermKind::is_implemented`]
  and [`TermKind::previous_samples`] surface the wiki's two
  narrowings.
- [`DecorrelationTerms::len`] / [`DecorrelationTerms::is_empty`] /
  [`DecorrelationTerms::kind_at`] / [`DecorrelationTerms::iter_kinds`]
  / [`DecorrelationTerms::all_implemented`] /
  [`DecorrelationTerms::has_reserved`] — convenience accessors that
  classify the round-3 term list without re-walking the bytes.
- [`weights_per_term`] — wiki "Each decorrelation term should have
  one or two weights depending on channels" split: mono → 1, stereo →
  2, with a defensive clamp for any higher channel count.
- [`decorrelation_sample_count`] / [`TermKind::decorrelation_sample_count`]
  — wiki "Decorrelation samples" / "Possible predictor values" per-term
  seed-sample count: `Some(code - 5)` for `6..=12`, `Some(2)` for
  `17..=18`, `None` for stereo predictors `0..=5` (per-term count is a
  docs gap), the reserved `13..=16` range, and codes outside `0..=18`.
  Public constant [`MAX_DECORRELATION_SAMPLES_PER_TERM`] = 16 surfaces
  the wiki "up to 16 samples" upper bound.
- [`DecorrelationTerms::expected_decorrelation_sample_count`] — sums
  the per-term wiki count across a term list, returning
  `Some(total)` when every term is documented and `None` as soon as
  any one is in the docs gap (so a caller can decide whether to
  partition or treat the block as undecodable from the wiki alone).
- [`partition_decorrelation_samples`] — splits the flat
  `DecorrelationSamples::samples` list produced by `expand_samples`
  into one `Vec<i32>` per term in wiki order, using the per-term
  counts above. Returns
  [`Error::DecorrelationSampleCountUnspecified`] when any term lacks
  a wiki count, and [`Error::DecorrelationSampleCountMismatch`] when
  the summed expected count does not equal the flat payload length.
- [`MetadataSubBlock::is_optional`] /
  [`MetadataSubBlock::is_decorrelation_payload`] /
  [`MetadataSubBlock::is_correction_payload`] /
  [`MetadataSubBlock::is_audio_payload`] /
  [`MetadataSubBlock::is_riff_payload`] /
  [`MetadataSubBlock::is_dummy_payload`] /
  [`MetadataSubBlock::is_hybrid_profile_payload`] /
  [`MetadataSubBlock::is_float_payload`] /
  [`MetadataSubBlock::is_int32_payload`] /
  [`MetadataSubBlock::is_overflow_bits_payload`] /
  [`MetadataSubBlock::is_multichannel_info_payload`] /
  [`MetadataSubBlock::is_encoding_details_payload`] /
  [`MetadataSubBlock::is_md5_payload`] /
  [`MetadataSubBlock::is_sample_rate_payload`] — payload-kind
  predicates covering every entry in the wiki "IDs" listing so a
  caller can pick a specific sub-block out of a walk without
  re-matching the [`SubBlockId`] enum.
- [`SubBlockId::is_decorrelation`] / [`SubBlockId::is_correction_stream`]
  / [`SubBlockId::is_riff_wrapper`] / [`SubBlockId::is_audio`] — the
  same family classifiers on the enum value itself for callers that
  branch on an ID rather than on a parsed sub-block.
- [`Md5Checksum`] — typed view of the `0x26` payload (the wiki "16-byte
  MD5 sum of raw audio data"), with [`parse_md5_checksum`] enforcing
  the fixed 16-byte length (other lengths reported through new
  [`Error::Md5ChecksumLength`]).
- [`find_first`] / [`find_audio_payload`] / [`find_entropy_info`] /
  [`find_md5_checksum_block`] / [`find_multichannel_info`] /
  [`find_decorrelation_triple`] / [`find_packed_samples`] —
  convenience finders over a [`walk_metadata`] result. The triple
  finder returns `(terms, weights, samples)` in wiki order or `None`
  when any of the three is missing — a malformed-block signal for the
  prediction loop. [`find_packed_samples`] returns the `0x0A` payload
  already wrapped as a typed [`PackedSamples`] (the typed counterpart
  to [`find_audio_payload`]).
- [`PackedSamples`] / [`expand_packed_samples`] — typed view of the
  `0x0A` packed-samples sub-block payload (the entropy-coded audio
  bitstream the wiki "Samples coding" section consumes). Borrows the
  walker's payload bytes verbatim and exposes [`PackedSamples::bytes`]
  / [`PackedSamples::len`] / [`PackedSamples::is_empty`] introspection
  plus a [`PackedSamples::bit_reader`] factory that yields a fresh
  [`BitReader`] positioned at bit 0 — the round-2 walker → round-5/6/7
  decoder handoff in one call. The wiki places no length constraint on
  the payload (the sample count is conveyed out-of-band by the block
  header's `block_samples`), so any byte slice, including the empty
  one, is accepted without rejection.
- [`BitReader::byte_position`] / [`BitReader::bit_position`] /
  [`BitReader::bits_consumed`] — cursor accessors naming the reader's
  position in the underlying byte slice. `bits_consumed` clamps at the
  buffer length when the reader has advanced past the end so callers
  computing a percentage / progress over a `0x0A` payload don't
  overshoot.
- [`EntropyInfo::is_stereo`] / [`EntropyInfo::channels`] /
  [`EntropyInfo::medians_for_channel`] — typed channel introspection
  pinning the wiki "one or two sets of medians for samples decoding"
  sentence as `1` or `2` populated sets, with a channel-indexed median
  getter returning `Some([m0, m1, m2])` for `0` (left/mono) or `1`
  (right, stereo only) and `None` for `1` on a mono payload (where the
  wiki put no second set on the wire) and for indices `>= 2`.
- [`Medians::from_entropy`] `(info, channel_idx)` — channel-indexed
  bridge over [`EntropyInfo`] returning `Some(Medians)` for `0` and
  for `1` on a stereo block, `None` otherwise. Equivalent to
  [`Medians::from_entropy_left`] / [`Medians::from_entropy_right`] but
  with the mono guard, so callers iterating per-channel medians
  (one or two iterations against `Flags::channels_in_block`) skip the
  hand-rolled mono / stereo branch.

### Out of scope (later rounds)

- The median **adaptation amount** that turns `decode_sample` /
  `decode_sample_value` into a stateful loop over a whole `0x0A` payload
  (feeding each decoded `n` back into a mutating median set). **Blocked
  on a docs gap:** the wiki names the median update direction
  ("increase" / "decrease") but not the amount (it cites WavPack's
  `format.txt` for the fraction-of-self step without reproducing it),
  so the per-sample *sequence* cannot be made bit-exact from this page
  alone. Round 7 decodes a single sample (run-length + value) against a
  fixed, caller-supplied median set instead.
- The degenerate `add == 0` Golomb interval (selected median `1`),
  where the wiki's `k = log2(0)` and `getbits(-1)` are undefined.
  `decode_sample_value` rejects it via `Error::GolombDegenerateInterval`
  pending a docs revision that specifies the single-codeword interval.
- The bit order of the `0x0A` stream is a documented assumption
  (least-significant-bit-first, matching WavPack's little-endian
  container); empirical confirmation against a real payload is gated
  on the median-adaptation gap above.
- The prediction loop that consumes the round-3 typed views.
- Per-term grouping of the samples list for **stereo predictors
  `0..=5`** (the wiki gives no per-term sample count for them; round 11
  lands the per-term partitioner for the documented `6..=12` / `17..=18`
  codes and refuses the stereo case via
  [`Error::DecorrelationSampleCountUnspecified`]).
- Hybrid-profile (lossy) `0x06` / noise-shaping `0x07`.
- Float-data `0x08` / large-or-shifted-int `0x09` / overflow-bits
  `0x0C`.
- Multichannel `0x0D` channel-mask handling.
- Non-standard sample-rate `0x27` numeric decode.
- Hybrid correction stream (`.wvc`) pairing.
- CRC32 verification (depends on sample decode).
- Encoder.

## Clean-room provenance

Rounds 1 through 12 read **only** `docs/audio/wavpack/wiki/WavPack.wiki`
(the local multimedia.cx snapshot under the docs repo) and
`oxideav-core`'s public API. No external library source
(`libwavpack`, `wavpack-rs`, FFmpeg's `wavpack.c` / `wavpackenc.c`),
no archived `old` branch of this crate, and no online resources
were consulted at any phase.

The 173-test unit suite synthesises minimal valid headers, sub-blocks
and bitstreams and poisons each field in turn to exercise the parser's
accept / reject boundaries (truncated inputs, wrong magic, undersized
`ck_size`, out-of-range version, bogus odd-size flag with zero data
words, large-size 24-bit size field, decorrelation-term low-5-bit /
high-3-bit field splitting, weight log-pack expansion across zero /
positive / negative bytes, sample exponent / mantissa expansion for
the equal / less / greater-than-9 branches, odd-byte-count
sample-payload rejection, entropy-info mono / stereo length gating
with both the shift-left and shift-right log-pack branches, the
LSB-first bit reader's `get_bit` / `get_bits` / `get_unary` primitives
including the wiki worked examples and byte-boundary crossing, the
run-length decoder's `last_zero` short-circuit, even / odd unary
halving, both escape arms with LSB-first mantissa assembly, and the
adaptive carry across a multi-sample sequence, the Golomb
`(base, add)` interval selection across the `n == 0` / `n == 1` /
`n >= 2` branches, the `k = log2(add)` bit-length derivation with its
`ex >= 0` invariant swept across `add` 1..=1024, the short- and
long-mantissa `t2 >= ex` paths, positive / ones-complement sign
reconstruction, the degenerate `add == 0` rejection, mantissa- and
sign-truncation reporting, an end-to-end compose of the run-length and
value halves over one contiguous bitstream, the round-7
`Medians::from_entropy_left` / `from_entropy_right` bridges for
stereo and mono inputs, and `decode_sample` chained-call coverage —
run-length-then-value, `last_zero` short-circuit honoured,
degenerate-interval and truncation error propagation, and the
entropy-info → median → sample end-to-end path; and the round-8
block-header accessor sweep — `is_standalone_block` /
`is_multichannel_member` across all four marker combinations,
`is_lossless` / `is_lossy` symmetry around the hybrid bit,
`has_custom_sample_rate` sentinel pin sweep across all 16
sample_rate_index values, `should_skip_decode` discriminating bit 31
from bit 28, `is_experimental` union, `effective_bit_depth` for the
wiki 12-bit / 20-bit worked examples plus the no-shift baseline
plus the saturation case, `is_audio_block` keyed on a non-zero
`block_samples`, `is_total_samples_known` against the sentinel and
the boundary `0`, and `payload_bytes` subtracting the 24-byte fixed
header floor; and the round-9 decorrelation-term classification +
metadata-payload kind sweep — `TermKind::from_code` across all four
wiki categories (stereo implemented `2..=4`, stereo unimplemented
`0/1/5`, sample-based `6..=12` with per-code sample count, reserved
`13..=16`, two-sample `17..=18`, and undocumented `19..=31` plus a
negative-code defensive check); `DecorrelationTerms` `len`/`is_empty`/
`kind_at`/`iter_kinds`/`all_implemented`/`has_reserved` accessors over
mixed term lists; `weights_per_term` mono/stereo split with 0- and 3-
channel clamps; `MetadataSubBlock::is_optional` pinning the `0x20`
flag; and per-kind payload predicates round-tripping for `0x02`/`0x03`/
`0x04` decorrelation, `0x07`/`0x0B` correction, `0x0A` audio, and
`0x20`/`0x21` RIFF with a non-RIFF optional negative case);
and the round-10 MD5 + walker-finder + remaining-kind-predicate
sweep — `SubBlockId` classifier coverage across all four buckets
(decorrelation `0x02`/`0x03`/`0x04`, correction-stream `0x07`/`0x0B`,
RIFF-wrapper `0x20`/`0x21` with same-flag `0x25`/`0x26`/`0x27`
negative cases, audio-only `0x0A`); one-hot kind-predicate sweep
across the eight new `MetadataSubBlock` predicates (with the four
main-bucket predicates pinned false on each); `is_md5_payload`
discriminating `0x06` HybridProfile from `0x26` Md5Checksum on the
low-5-bit overlap and `is_dummy_payload` discriminating `0x00` Dummy
from `0x20` RiffHeader; `parse_md5_checksum` accept (MD5 of `""`
test vector) and reject (0 / 15 / 17 / 64-byte lengths); end-to-end
round-trip from a synthesised `0x26` sub-block through
`walk_metadata` → `find_md5_checksum_block` → `parse_md5_checksum`
(MD5 of the "quick brown fox" test vector); walker finder coverage —
`find_first` hit + miss across `SubBlockId::EntropyInfo` vs
`SubBlockId::HybridProfile`, the four specialised finders, and
`find_decorrelation_triple` returning the full triple in order and
`None` when either of weights / samples is dropped); and the round-11
per-term decorrelation-sample-count + partitioner sweep —
`decorrelation_sample_count` returning `Some(code - 5)` across the full
`6..=12` sample-based range, `Some(2)` for `17` / `18`, and `None`
across stereo `0..=5`, reserved `13..=16`, and undocumented `19..=31`
plus a negative-code defensive check; the [`MAX_DECORRELATION_SAMPLES_PER_TERM`]
= 16 wiki upper-bound sanity sweep across every documented count;
`DecorrelationTerms::expected_decorrelation_sample_count` summing a
mixed `[6, 8, 17, 12]` term list to `13`, the vacuous empty-list `0`,
and `None` propagation when a stereo / reserved / undocumented code
appears anywhere in the list; and `partition_decorrelation_samples`
splitting a `[6, 8, 17]` term list with matching 6-sample flat input
in term order, the empty-terms-empty-payload base case, refusing the
stereo `[2]` and reserved `[6, 14]` lists with
`DecorrelationSampleCountUnspecified`, rejecting both short
(`expected: 6, actual: 5`) and long (`expected: 1, actual: 4`) flat
payloads with `DecorrelationSampleCountMismatch`, and a round-trip
from `expand_samples` of a synthesised `[6, 18]` wire through the
partitioner back to per-term `[1]` + `[2, 3]` lists; and the round-12
`PackedSamples` typed view + `BitReader` position + channel-indexed
`EntropyInfo` / `Medians::from_entropy` sweep — `PackedSamples`
round-tripping a non-empty payload, the zero-byte empty payload
accepted and reported empty (the wiki places no length constraint on
the `0x0A` payload), `expand_packed_samples` round-tripping the byte
slice, the `bit_reader()` factory starting at byte/bit 0 with the
full payload remaining and yielding the first bit LSB-first, the
factory returning independent readers across multiple calls, an
empty packed-samples view reporting immediate `Error::Truncated` on
any read, and the view being `Copy`; `BitReader::byte_position` /
`bit_position` / `bits_consumed` tracking 13-bit consumption across
a byte boundary, the `bits_consumed` clamp when the reader is past
the end, and the cursor staying put when a read errors with
`Truncated`; `Medians::from_entropy` yielding the left set on index
`0` and the right set on index `1` for a stereo `EntropyInfo`,
returning `None` for `1` on a mono `EntropyInfo`, and rejecting
out-of-range indices (`2`, `3`, `255`); `EntropyInfo::is_stereo`
inverting `is_mono`, `channels` returning `1` for mono and `2` for
stereo, `medians_for_channel` yielding the matched set for `0` / `1`
on stereo and `None` for `1` on mono / `2+` indices; and
`find_packed_samples` returning a typed `PackedSamples` view over a
synthesised `0x0A` sub-block and `None` on a stream without one.
