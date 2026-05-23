# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
