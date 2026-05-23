# oxideav-wavpack

Pure-Rust WavPack lossless audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 5 — block-header parser + metadata sub-block walker +
decorrelation sub-block expanders + entropy-info expander +
sample-coding bit reader & run-length decoder.** Round 1
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
`last_one` flags. All work follows `docs/audio/wavpack/wiki/WavPack.wiki`
(the local snapshot of the multimedia.cx WavPack reference page).

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
  `last_one` state in a `RunState`. The second half (Golomb
  `(base, add)` interval selection + median adaptation) is deferred —
  see the docs gap below.

### Out of scope (later rounds)

- The median-adaptation second half of the `0x0A` sample decode
  (`(base, add)` interval selection, `getbits(k-1)` mantissa, sign,
  and the median "increase" / "decrease" steps). **Blocked on a docs
  gap:** the wiki names the median update direction but not the
  amount (it cites WavPack's `format.txt` for the fraction-of-self
  step without reproducing it), so the per-sample reconstruction
  cannot be made bit-exact from this page alone.
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

Rounds 1 through 5 read **only** `docs/audio/wavpack/wiki/WavPack.wiki`
(the local multimedia.cx snapshot under the docs repo) and
`oxideav-core`'s public API. No external library source
(`libwavpack`, `wavpack-rs`, FFmpeg's `wavpack.c` / `wavpackenc.c`),
no archived `old` branch of this crate, and no online resources
were consulted at any phase.

The 69-test unit suite synthesises minimal valid headers, sub-blocks
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
including the wiki worked examples and byte-boundary crossing, and the
run-length decoder's `last_zero` short-circuit, even / odd unary
halving, both escape arms with LSB-first mantissa assembly, and the
adaptive carry across a multi-sample sequence).
