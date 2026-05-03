# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial WavPack lossless decoder: 32-byte block header parser,
  tagged sub-block walker (`DECTERMS`, `DECWEIGHTS`, `DECSAMPLES`,
  `ENTROPY`, `DATA`, `INT32INFO`, `EXTRABITS`, `CHANINFO`,
  `SAMPLE_RATE`).
- Adaptive median entropy decoder (3-bin M0/M1/M2 with the +5/-2
  adaptation rates per spec §5.4) including the silence-region
  zero-run shortcut.
- Decorrelation cascade reverse for terms 1..8, 17, 18, and the
  cross-channel terms -1, -2, -3.
- Joint-stereo and false-stereo undo, per-block CRC verification.
- 8/16/24/32-bit integer container support (lossless mode).
- File-level frame walker that groups blocks via `INITIAL_BLOCK` /
  `FINAL_BLOCK` (spec §3.6).
- Bit-exact lossless round-trip against the system `ffmpeg` WavPack
  encoder on **digital silence** (mono / stereo / multi-frame).
  Sine-wave fixtures left as round-2 backlog (median-bin /
  holding-bit semantics need calibration).
