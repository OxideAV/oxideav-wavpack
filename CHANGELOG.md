# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
