# oxideav-wavpack

Pure-Rust WavPack lossless audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 7 — block-header parser + metadata sub-block walker +
decorrelation sub-block expanders + entropy-info expander +
sample-coding bit reader, run-length decoder, Golomb sample-value
reconstruction & single-call per-sample decode + entropy→median
bridge.** Round 1
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
- Per-term grouping of the samples list (the wiki "up to 16 samples
  depending on its value" rule).
- Hybrid-profile (lossy) `0x06` / noise-shaping `0x07`.
- Float-data `0x08` / large-or-shifted-int `0x09` / overflow-bits
  `0x0C`.
- Multichannel `0x0D` channel-mask handling.
- Non-standard sample-rate `0x27` numeric decode.
- Hybrid correction stream (`.wvc`) pairing.
- CRC32 verification (depends on sample decode).
- Encoder.

## Clean-room provenance

Rounds 1 through 7 read **only** `docs/audio/wavpack/wiki/WavPack.wiki`
(the local multimedia.cx snapshot under the docs repo) and
`oxideav-core`'s public API. No external library source
(`libwavpack`, `wavpack-rs`, FFmpeg's `wavpack.c` / `wavpackenc.c`),
no archived `old` branch of this crate, and no online resources
were consulted at any phase.

The 92-test unit suite synthesises minimal valid headers, sub-blocks
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
entropy-info → median → sample end-to-end path).
