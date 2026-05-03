# oxideav-wavpack

Pure-Rust **WavPack** lossless audio decoder. Zero C dependencies, no
FFI, no `*-sys` crates.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## What works (round 1)

- 32-byte block header parser (magic, version range, total/index/
  block-samples, 32-bit flags field, CRC).
- Tagged sub-block walker (8-bit and 24-bit length forms, `ODD` /
  `LARGE` / `IGNORE` flag bits).
- Adaptive 3-bin median entropy decoder (`M0` / `M1` / `M2` with
  the spec's `+5/-2`-style update rates).
- Decorrelation cascade undo for **all 11 legal terms**:
  `1..=8`, `17`, `18`, and the cross-channel `-1`, `-2`, `-3`.
- Joint-stereo (mid/side) and false-stereo undo.
- `INT32INFO` post-shift restoration (`zero_bits` / `ones_bits` /
  `dup_bits` paths, plus `sent_bits` extra-bits stream).
- Per-block CRC verification (CRC of decoded sample stream, not of
  on-disk bytes — spec §5.1).
- 8 / 16 / 24 / 32-bit integer container (lossless PCM mode).
- Quantised `wp_log2` / `wp_exp2` derived from the mathematical
  definitions (no upstream lookup tables transcribed).

## Tested

- **Bit-exact lossless round-trip on digital silence** (mono 16-bit,
  stereo 16-bit, multi-block stereo) via the system `ffmpeg` binary
  as a black-box encoder: synthesise PCM via `anullsrc`, encode to
  `.wv` with `-c:a wavpack`, decode through this crate, and assert
  the recovered PCM matches the source byte-for-byte. See
  `tests/silence_roundtrip.rs`. Silence exercises the file walker,
  block parser, sub-block walker, the entropy decoder's zero-run
  shortcut (spec §5.4 step 1), the per-block CRC, and the multi-frame
  stitching path.
- Header / sub-block / DECTERMS / DECWEIGHTS / ENTROPY / INT32INFO
  unit tests under each module's `#[cfg(test)]` block plus
  `tests/handcrafted.rs` (25 tests in total, all passing).
- Sine-wave round-trips in `tests/ffmpeg_roundtrip.rs` are present
  but marked `#[ignore]` — they exercise the median-bin / shift /
  holding-bit interaction in spec §5.4 step 2, whose exact bitstream
  semantics are still being calibrated against ffmpeg's encoder. See
  "Round-2 backlog" below.

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-wavpack = "0.0"
```

## Quick use

```rust,no_run
use oxideav_wavpack::container::{decode_frame, parse_file};

let bytes = std::fs::read("song.wv")?;
let parsed = parse_file(&bytes)?;
for frame in &parsed.frames {
    let channels = decode_frame(&bytes, frame)?;
    // `channels[c][i]` is the i-th sample on channel c (signed integer).
}
# Ok::<(), oxideav_core::Error>(())
```

## Round-2 backlog

- **Sine bit-exactness — the entropy bin / shift / holding semantics**.
  The trace doc describes the magnitude-prefix decode as
  "read a unary t; if t==16 then escape; capture bottom bit of t as
  the holding state for next sample; shift t right; t indexes the
  median bin". The exact relationship between the unary read and the
  next-sample's prefix-skip behaviour for the holding state is not
  unambiguously pinned down in the trace doc, and our current
  implementation gets close-but-not-exact decoded samples on sine
  fixtures. Resolution probably needs a fixture trace at the bit-level
  tied to known input samples.
- **Hybrid (lossy) mode**. The `HYBRID` / `HYBRID_BITRATE` /
  `HYBRID_BALANCE` / `HYBRID_SHAPE` flag bits, plus `WP_ID_HYBRID`
  (id 6) and `WP_ID_SHAPING` (id 7) sub-blocks, plus the binary-
  search tail-bits decode under an adaptive `error_limit`, plus the
  `.wvc` correction-file pairing.
- **DSD path** (spec §6 — `WP_ID_DSD_DATA` modes 0/1/3 with the 2-byte
  `rate_x`/`mode` header, history-bin probability table, range coder,
  noise-shaping IIR cascade).
- **Float-data reconstruction** (`FLOATINFO` sub-block, `EXTRABITS`
  mantissa LSBs, IEEE-754 reassembly with the 5-flag descriptor).
- **Multichannel layouts above 2.0** — `CHANINFO` parsing exists at
  the file-walker level, but the per-block channel stitching beyond
  mono+stereo into a single frame is currently exercised only by
  smoke tests; full 5.1/7.1 layout assembly needs a fuller
  channel-mask → output-order mapping.
- **Encoder**: not implemented. Round-trip tests use the `ffmpeg`
  binary as a black-box encoder.

## Spec reference

Behavioural specifications:

- [`docs/audio/wavpack/wavpack-trace-reverse-engineering.md`](https://github.com/OxideAV/oxideav-workspace/blob/master/docs/audio/wavpack/wavpack-trace-reverse-engineering.md)
- [`docs/audio/wavpack/wavpack-decorr-presets.md`](https://github.com/OxideAV/oxideav-workspace/blob/master/docs/audio/wavpack/wavpack-decorr-presets.md)

This crate is a **clean-room** implementation: no third-party WavPack
source code (libwavpack, libavcodec, …) was consulted while writing
it.

## License

MIT. See `LICENSE`.
