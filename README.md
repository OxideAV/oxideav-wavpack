# oxideav-wavpack

Pure-Rust WavPack lossless audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 3 — block-header parser + metadata sub-block walker +
decorrelation sub-block expanders.** Round 1 landed the 32-byte fixed
block-header parser; round 2 added the metadata sub-block walker
completing the structural pass over a WavPack v.4 block; round 3 turns
the `0x02` / `0x03` / `0x04` decorrelation sub-block payloads into
typed views (terms, log-pack-expanded weights, exponent / mantissa
samples). All work follows `docs/audio/wavpack/wiki/WavPack.wiki`
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

### Out of scope (later rounds)

- The prediction loop that consumes the round-3 typed views.
- Per-term grouping of the samples list (the wiki "up to 16 samples
  depending on its value" rule).
- Entropy decode of `0x0A` packed-samples sub-block.
- Hybrid-profile (lossy) `0x06` / noise-shaping `0x07`.
- Float-data `0x08` / large-or-shifted-int `0x09` / overflow-bits
  `0x0C`.
- Multichannel `0x0D` channel-mask handling.
- Non-standard sample-rate `0x27` numeric decode.
- Hybrid correction stream (`.wvc`) pairing.
- CRC32 verification (depends on sample decode).
- Encoder.

## Clean-room provenance

Rounds 1, 2 and 3 read **only** `docs/audio/wavpack/wiki/WavPack.wiki`
(the local multimedia.cx snapshot under the docs repo) and
`oxideav-core`'s public API. No external library source
(`libwavpack`, `wavpack-rs`, FFmpeg's `wavpack.c` / `wavpackenc.c`),
no archived `old` branch of this crate, and no online resources
were consulted at any phase.

The 38-test unit suite synthesises minimal valid headers and
sub-blocks and poisons each field in turn to exercise the parser's
accept / reject boundaries (truncated inputs, wrong magic, undersized
`ck_size`, out-of-range version, bogus odd-size flag with zero data
words, large-size 24-bit size field, decorrelation-term low-5-bit /
high-3-bit field splitting, weight log-pack expansion across zero /
positive / negative bytes, sample exponent / mantissa expansion for
the equal / less / greater-than-9 branches, and odd-byte-count
sample-payload rejection).
