# oxideav-wavpack

Pure-Rust WavPack lossless audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

The crate parses the WavPack block container and decodes the
modified-Rice entropy stream to PCM, with a matching exact-inverse
encoder on the write side.

Working surface:

* **Block container** — `parse_block` / `parse_block_header` decode the
  fixed 32-byte block header and walk the metadata sub-block chain;
  typed accessors expose every documented header field (version,
  block/total sample counts, CRC, track ids, flags) and the sub-block
  payloads (`PackedSamples` `0x0A`, `PackedCorrectionData` `0x0B`,
  `PackedOverflowBits` `0x0C`, entropy info `0x05`, MD5, RIFF
  header/trailer, multichannel info).
* **Stream iteration** — `BlockIter` / `iter_blocks` walk a chained
  multi-block buffer lazily; `decode_stream` / `iter_decoded_blocks`
  decode every audio block and concatenate the PCM. Stream-level
  introspection (`audio_block_count`, `total_audio_samples`,
  `decoded_sample_count`, `first_audio_block`, correction-stream
  helpers) reports shape without retaining the parsed list.
* **Entropy decode** — the full §4.2 modified-Rice sample-word ladder
  is exposed as typed primitives (`SampleInterval`, `AdaptiveMedians`,
  the zero-run fast path, raw prefix + holding-bit fold,
  truncated-binary mantissa, sign reconstruction) and composed by
  `WavPackBlock::decode_samples` for mono / stereo / false-stereo
  blocks.
* **Decorrelation primitives** — the §3 inverse-prediction scalar
  arithmetic is exposed as standalone functions: `apply_weight` (§3.1,
  `(weight*sample + 512) >> 10`, `1024` == unity), `update_weight` (§3.4
  LMS `±delta` adaptation) and `update_weight_clip` (§3.5 cross-channel
  variant clamped to `±1024`), alongside the metadata expanders for the
  `0x02`/`0x03`/`0x04` terms / weights / seed-samples sub-blocks. The
  per-term inverse-prediction loop is now assembled: `decorrelate_mono`
  and `decorrelate_stereo` run an ordered list of `DecorrPass` passes over
  a residual buffer in place (§3.7 whole-buffer-per-pass, application
  order), composing `apply_weight` / `update_weight` for the fixed-lag
  (`1..8`) and extrapolate (`17`/`18`) terms and `update_weight_clip` for
  the zero-delay cross terms (`-1`/`-2`/`-3`, stereo only). `DecorrPass`
  carries the per-channel 8-slot history ring (`MAX_TERM`), seeded from the
  `0x04` samples, and `decode_term_byte` reads the spec's `+5`-biased term
  encoding (distinct from the wiki-listing reading of `expand_terms`). The
  passes are pinned to a forward/inverse round trip across every term and
  multi-pass stacks. `assemble_mono_passes` turns the raw
  `0x02`/`0x03`/`0x04` payloads into an application-ordered `DecorrPass`
  list, applying the spec §3.7 reverse-storage convention (on-wire passes
  are stored last-applied-first; the assembler reverses them so the
  decoder undoes the encoder's last pass first), the spec `+5` term-byte
  encoding, one weight per pass, and the per-term seed partition.
  `assemble_stereo_passes` is the two-channel twin: per spec §3.6 it reads
  two weight bytes per pass (channel A then B) and partitions the `0x04`
  seeds per channel with the term-class count (2 / `term` / 1), accepting
  the cross terms (`-1`/`-2`/`-3`) that are valid only for stereo.
* **Mono + stereo lossless decode → reconstructed PCM** — `decode_samples`
  runs the full lossless pipeline for a **mono** (or false-stereo) block
  and for a **stereo** block carrying decorrelation: entropy-decode the
  `0x0A` residuals, assemble the passes from `0x02`/`0x03`/`0x04`, then run
  `decorrelate_mono` / `decorrelate_stereo` over the residual buffer in
  place to reconstruct the PCM. Joint (mid/side) stereo blocks are
  finished with the spec §5.4 `R -= L>>1; L += R` undo applied per pair
  after decorrelation. The path is pinned by forward/inverse round-trips
  (encode a residual buffer into the `0x0A` bitstream, attach a multi-pass
  decorrelation config, confirm decode reproduces the standalone engine
  output) for both channel shapes and cross terms.
* **Left-shift final normalization** — `decode_samples`' last stage applies
  the wiki flag-bits-13..=17 *left-shift fixup* (`fixup` module:
  `apply_left_shift` / `apply_left_shift_buffer`): when a block's effective
  bit-depth is not a whole number of bytes (12-bit, 20-bit, …) the encoder
  narrows each sample and records the dropped trailing-zero count, and the
  decoder shifts the reconstructed magnitude left by that count to restore
  container-scaled PCM (the identity for whole-byte depths, where
  `left_shift == 0`). Per the decorrelation-spec doc §1 pipeline and §5.2
  ("before final shift") this runs *after* the running CRC is folded over
  the pre-shift samples, so the decode body splits into a private
  pre-shift path the CRC checkers (`verify_decoded_crc`,
  `decode_samples_muted`) fold, with the shift applied to the emitted PCM
  afterward (and skipped on a muted/zeroed block). Pinned by mono and
  stereo decode tests at several shift counts, the pre-shift-CRC fold (and
  its post-shift-CRC negative), and a whole-byte-depth identity guard.
* **Entropy encode** — exact write-side inverses (`BitWriter`,
  `encode_packed_samples_mono` / `_stereo`, and the per-primitive
  interval / prefix / mantissa encoders) round-trip the decode ladder
  bit-for-bit.
* **Block CRC** — the §5 running 32-bit sample CRC is computed by a
  `BlockCrc` accumulator (`push_mono` / `push_stereo_pair` /
  `push_joint_stereo_pair`, `matches`) and the `crc_mono` /
  `crc_stereo_interleaved` / `crc_joint_stereo_interleaved` one-shot
  helpers. The mono `crc*3 + s` and stereo `crc*9 + 3L + R` steps, the
  `0xffffffff` seed, and the joint-stereo mid/side undo
  (`undo_joint_stereo`) that precedes the stereo step are implemented and
  pinned to the spec's worked CRC vectors; `matches` verifies a computed
  CRC against the stored header CRC. The block-level
  `WavPackBlock::verify_decoded_crc` ties the CRC to the decode path: it
  decodes the block's PCM, folds the §5 mono / stereo CRC over the
  reconstructed samples (post-decorrelation, post-joint-undo), and reports
  whether it matches the stored header word — a non-mutating §5.6 checker.
  `WavPackBlock::decode_samples_muted` is the spec-faithful gate: it
  returns `(pcm, crc_ok)` and, on a mismatch, zeros the buffer (the §5.6
  "mute the corrupt block" behaviour). `decode_stream_muted` lifts that
  gate to the whole stream — each audio block is CRC-gated and muted
  independently, returning the concatenated PCM plus an `all_crc_ok` flag.

## Public API sketch

```rust
use oxideav_wavpack::{parse_block, decode_stream, decode_stream_muted};

// Whole-stream decode → interleaved Vec<i32> PCM:
let pcm = decode_stream(file_bytes)?;

// Or with the spec §5.6 per-block CRC mute gate applied:
let (pcm, all_crc_ok) = decode_stream_muted(file_bytes)?;

// Or walk block-by-block:
let (block, _tail) = parse_block(file_bytes)?;
if block.is_audio_block() {
    let samples = block.decode_samples()?;        // mono / stereo lossless
    let (samples, crc_ok) = block.decode_samples_muted()?; // CRC-gated
}
```

## Not yet supported

`WavPackBlock::decode_samples` refuses the following with a typed
`Error::UnsupportedBlockFeature`: hybrid (lossy) blocks, float and
32-bit-int sample data, multichannel members, low-latency / robust block
layouts, and — on non-hybrid stereo blocks — the `CROSS_DECORR` flag
(bit 5). **Mono and stereo** lossless decorrelation are now wired all the
way through `decode_samples` (entropy → residuals →
`assemble_mono_passes` / `assemble_stereo_passes` → `decorrelate_mono` /
`decorrelate_stereo` → PCM), including the negative cross terms
(`-1`/`-2`/`-3`) on stereo and the spec §5.4 joint-stereo (mid/side) undo.
The `CROSS_DECORR` flag stays refused on a lossless stereo block because
the staged decorrelation doc §4.1 documents it only in the hybrid-stereo
correction-folding context — so it has no defined main-stream meaning
here (the lossless inter-channel predictors are the negative `0x02` decorr
*terms*, which **are** decoded). The §5 block CRC is exposed at block
level via `WavPackBlock::verify_decoded_crc` (non-mutating checker) and
`WavPackBlock::decode_samples_muted` (the spec §5.6 mute gate), and at
stream level via `decode_stream_muted`. The extension CRC (`crc_x`, §5.5)
over `0x0C` wide/float data is pending its consumer.

The **left-shift** half of the decorrelation-spec §1 "shift/clip fixups"
stage is now applied (see the working-surface bullet above); the **clip**
half is the wiki flag-bits-18..=22 `max_magnitude`, which the wiki
documents only as an optimisation *hint* ("can be used to optimize
decoding arithmetic"), not a value the decoder must clamp against, so no
clip is applied for correctness.

The **hybrid** (lossy main + `.wvc` correction) decode path is the next
milestone but is **blocked on a docs gap**: the staged docs describe the
correction-stream consumer semantics (decorr doc §4 — read two residuals
per sample, fold the correction pre- or post-decorrelation per
`CROSS_DECORR`, optional `HYBRID_SHAPE` error-feedback) and note that the
lossy main-stream `0x0A` decode replaces the §4.2 step-6 truncated-binary
mantissa with a binary-search refinement loop over `[low, high]` *until
the interval is within `error_limit`* — but the **derivation of
`error_limit`** itself (the `update_error_limit` rule from the hybrid
profile / `slow_level` / bitrate) and the **`read_shaping_info` metadata
layout** are named in the provenance index but not transcribed into the
spec. Without `error_limit` the lossy main stream cannot be decoded, so
hybrid stays refused.

A full `.wv` block *writer* (header + sub-block framing around the entropy
encoder) is not yet assembled. The crate is not yet wired into the
`oxideav-core` framework registry — there is no `Decoder` / `Encoder`
trait impl or `register` entry point.

## Provenance

Clean-room from the staged material under `docs/audio/wavpack/`: the
block-structure and sub-block-ID wiki listing, the clean-room
entropy-decode trace `docs/audio/wavpack/spec/wavpack-entropy-decode.md`,
and — for the block CRC (`§5`) and the decorrelation weight arithmetic
(`§3` + the `§6` constants summary + the `§7` sanity vectors) — the
clean-room decorrelation/CRC trace
`docs/audio/wavpack/spec/wavpack-decorrelation.md`. No external library
source, archived prior history, or online resources were consulted at
any phase. A `cargo-fuzz` harness in `fuzz/` fuzzes the `decode_stream`
entry point; 739 unit tests synthesise minimal valid headers /
sub-blocks / bitstreams and poison each field to exercise the accept /
reject boundaries, pin the §5 CRC primitives to the spec's worked
mono / stereo CRC vectors, pin the §3 decorrelation weight
arithmetic to the spec's §7 sanity vectors, and pin the wiki
flag-bits-13..=17 left-shift fixup (the decorrelation-spec §1 "shift"
normalization stage) end-to-end for mono and stereo decode plus its
pre-shift CRC ordering.

## License

MIT. See `LICENSE`.
