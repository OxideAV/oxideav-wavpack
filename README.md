# oxideav-wavpack

Pure-Rust WavPack lossless audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 1 — block-header parser.** The crate now parses the 32-byte
fixed header that precedes every WavPack v.4 block (per
`docs/audio/wavpack/wiki/WavPack.wiki`, the local snapshot of the
multimedia.cx WavPack reference page). The public API surfaces:

- [`parse_block_header`] — accepts `&[u8]`, returns
  `(WavPackBlockHeader, &[u8])` on success (header + unconsumed
  payload tail). Errors on truncated input, wrong magic,
  undersized `ck_size`, or out-of-range stream version.
- [`WavPackBlockHeader`] — typed view of the fixed header
  fields: `ck_size`, `version`, `track_number`, `track_sub_index`,
  `total_samples` (with the [`TOTAL_SAMPLES_UNKNOWN`] sentinel),
  `block_index`, `block_samples`, [`Flags`], `crc`.
- [`Flags`] — typed decode of the 32-bit flag word: every bit-range
  named on the wiki "Flags meaning" listing is exposed
  individually (bytes-per-sample, mono / hybrid / joint-stereo /
  cross-channel decorrelation / hybrid-shaping / float / int32 /
  hybrid-profile / multi-channel start-end markers / left-shift /
  max magnitude / sampling-rate index / reserved bit 27 / robust /
  hybrid IIR noise shaping / false-stereo / low-latency).

### Out of scope (later rounds)

- Metadata sub-block walking (`0x00`-`0x0D` audio / decorrelation
  sub-blocks; `0x20`-`0x27` RIFF / MD5 / non-standard sample-rate
  sub-blocks).
- Decorrelation terms / weights / samples deserialisation.
- Entropy decode of `0x0A` packed-samples sub-block.
- Hybrid-profile (lossy) `0x06` / noise-shaping `0x07`.
- Float-data `0x08` / large-or-shifted-int `0x09` / overflow-bits
  `0x0C`.
- Multichannel `0x0D` channel-mask handling.
- Hybrid correction stream (`.wvc`) pairing.
- CRC32 verification (depends on sample decode).
- Encoder.

## Clean-room provenance

Round 1 read **only** `docs/audio/wavpack/wiki/WavPack.wiki` (the
local multimedia.cx snapshot under the docs repo) and
`oxideav-core`'s public API. No external library source
(`libwavpack`, `wavpack-rs`, FFmpeg's `wavpack.c` / `wavpackenc.c`),
no archived `old` branch of this crate, and no online resources
were consulted at any phase.

The 10-test unit suite synthesises minimal valid headers and
poisons each field in turn to exercise the parser's accept / reject
boundaries.
