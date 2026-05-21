# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
