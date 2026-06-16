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
  CRC against the stored header CRC.

## Public API sketch

```rust
use oxideav_wavpack::{parse_block, decode_stream};

// Whole-stream decode → interleaved Vec<i32> PCM:
let pcm = decode_stream(file_bytes)?;

// Or walk block-by-block:
let (block, _tail) = parse_block(file_bytes)?;
if block.is_audio_block() {
    let samples = block.decode_samples()?;
}
```

## Not yet supported

`WavPackBlock::decode_samples` refuses the following with a typed
`Error::UnsupportedBlockFeature`: hybrid (lossy) blocks, float and
32-bit-int sample data, multichannel members, decorrelation pre-pass
blocks, low-latency / robust block layouts, and — on stereo blocks —
joint-stereo (mid/side, flag bit 4) and cross-channel decorrelation
(flag bit 5). The two inter-channel transforms have no formula in the
staged docs, so a block carrying either flag is refused (rather than
silently decoded as independent L/R, which would emit the mid/side
residuals); the gates are stereo-only, so mono / false-stereo blocks
with the bits set still decode. The decorrelation,
hybrid-correction (`.wvc`), and overflow-bit sub-block payloads have
typed views but no consuming decode pass. The §5 block CRC is now
*computed* (over decoded mono / stereo / joint-stereo PCM) and can be
checked against the stored header word via `BlockCrc::matches`, but the
end-to-end decode pipeline does not yet run the CRC at block end to mute
mismatched blocks (that is gated on the decorrelation/hybrid loop); the
extension CRC (`crc_x`, §5.5) over `0x0C` wide/float data is likewise
pending its consumer. A full `.wv` block *writer* (header + sub-block
framing around the entropy encoder) is not yet assembled. The crate is
not yet wired into the `oxideav-core` framework registry — there is no
`Decoder` / `Encoder` trait impl or `register` entry point.

## Provenance

Clean-room from the staged material under `docs/audio/wavpack/`: the
block-structure and sub-block-ID wiki listing, the clean-room
entropy-decode trace `docs/audio/wavpack/spec/wavpack-entropy-decode.md`,
and — for the block CRC — `§5` of the clean-room decorrelation/CRC trace
`docs/audio/wavpack/spec/wavpack-decorrelation.md`. No external library
source, archived prior history, or online resources were consulted at
any phase. A `cargo-fuzz` harness in `fuzz/` fuzzes the `decode_stream`
entry point; 662 unit tests synthesise minimal valid headers /
sub-blocks / bitstreams and poison each field to exercise the accept /
reject boundaries, and pin the §5 CRC primitives to the spec's worked
mono / stereo CRC vectors.

## License

MIT. See `LICENSE`.
