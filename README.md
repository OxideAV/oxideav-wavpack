# oxideav-wavpack

[![CI](https://github.com/OxideAV/oxideav-wavpack/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-wavpack/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-wavpack.svg)](https://crates.io/crates/oxideav-wavpack) [![docs.rs](https://docs.rs/oxideav-wavpack/badge.svg)](https://docs.rs/oxideav-wavpack) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Pure-Rust WavPack lossless audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

The crate parses the WavPack block container and decodes the
modified-Rice entropy stream to PCM, with a matching exact-inverse
encoder on the write side. **Arbitrary reference-encoded lossless AND
hybrid files — including every `.wv`+`.wvc` hybrid-lossless pair
shape — decode bit-exactly** (rounds 405/408/415): 16/24/32-bit
integer, 32-bit float (every documented `0x08` profile shape, inf/NaN
included), mono / stereo (left-right and joint) / false-stereo /
multichannel, every standard encoder effort mode, the hybrid `-b`
bitrate range, and the `-c`/`-cc` correction-file modes — validated
black-box against the reference decoder on a 59-fixture battery
committed under `tests/data/`. Since round 418 the encoder
**originates every axis it decodes**: `FLOAT_DATA` and `INT32_DATA`
streams (data-derived `0x08`/`0x09` profiles + `0x0C` extension
streams), and **hybrid encoding** — a lossy `.wv` at a caller-chosen
§6.5 bitrate word plus the `.wvc` correction twin that restores the
input bit-exactly — every emitted stream/pair decoding byte-identically
through this crate's own decoder *and* the reference decoder binary
(35-case black-box battery: mono/stereo × joint/left-right ×
raw/derived prediction × bitrate words 0..2000 × int/float/int32 ×
multi-block × silence × clipping-adjacent content).

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
* **Block encode (lossless round-trip)** — `encode_block_mono` /
  `encode_block_stereo` assemble a whole `wvpk` block from a PCM buffer:
  the 32-byte fixed header (spec §5 running CRC folded over the PCM, flag
  word reconstructed from the block shape, version `0x0410`, standalone
  multichannel marker so the decoder does not treat the block as a
  multichannel member) followed by the `0x05` entropy-info and `0x0A`
  packed-samples metadata sub-blocks (framed by the forward inverse of
  `parse_metadata_sub_block` — word-count size field, large-size escape,
  odd-size pad). The headline guarantee is
  `decode_stream(&encode_block_mono(pcm, …)?)? == pcm` (and the stereo
  twin): the encoded block parses, passes its own §5.6 CRC mute gate, and
  reconstructs the exact input PCM. These two are the raw (no-decorrelation)
  path; the decorrelated / joint / shifted / multi-block variants below
  cover the rest. Hybrid / float / int32 / multichannel block emission stay
  out of scope (the decoder refuses them; their wire layout is a documented
  spec gap). Exported: `ENCODE_VERSION`.
* **Lossless-with-decorrelation encode** —
  `encode_block_mono_with_decorr` / `encode_block_stereo_with_decorr` take
  the raw `0x02`/`0x03`/`0x04` metadata payloads, assemble + validate the
  pass list, run the §3 forward prediction loop (`recorrelate_*`) into
  residuals, and emit the three decorrelation sub-blocks **verbatim**
  ahead of the `0x0A` residuals — bit-exact by construction
  (`decode_stream(&out)? == pcm`) for fixed-lag (`1..8`), extrapolate
  (`17`/`18`) and stereo cross (`-1`/`-2`/`-3`) terms, single- and
  multi-pass.
* **Sub-byte-depth (left-shift) encode** — `encode_block_mono_shifted` /
  `encode_block_stereo_shifted` right-shift each container-scaled sample
  by `left_shift` (inverse of the decoder's final §1
  `apply_left_shift_buffer`), fold the §5 CRC over the narrow values, and
  set the wiki flag-bits-13..=17 field, so 12-bit / 20-bit audio round-
  trips exactly. Inputs must be a multiple of `2^left_shift`; a lossy low
  bit is refused.
* **Joint (mid/side) stereo encode** — `encode_block_stereo_joint`
  applies the forward mid/side transform (`mid = L - R; side = R + (mid >>
  1)`, the exact inverse of the decoder's §5.4 `undo_joint_stereo`) and
  sets the joint flag (bit 4). The `mid >> 1` truncation cancels between
  forward and inverse, so the block stays bit-exactly lossless
  (`decode_stream(&out)? == pcm`).
* **Multi-block stream encode** — `encode_stream_mono` /
  `encode_stream_stereo` split a long PCM buffer into a chain of `wvpk`
  blocks (default `DEFAULT_BLOCK_SAMPLES`, caller-overridable per-channel
  chunk size), each carrying the running `block_index` sample offset and
  the file-global `total_samples`, so the chain is a standalone `.wv` file
  the stream walker decodes back exactly:
  `decode_stream(&encode_stream_mono(pcm, …)?)? == pcm`.
* **Multichannel grouping decode + encode** — a WavPack stream carrying
  more than two channels splits each frame range across a *set* of member
  blocks (wiki bits 11..=12: a bit-11 member opens the set, continuation
  members follow, a bit-12 member closes it). Each member is an ordinary
  1-channel (mono / false-stereo) or 2-channel (stereo) block decoded by
  the same lossless path standalone blocks use — the grouping marker is a
  stream-shape signal, not a decode-arithmetic one. `decode_multichannel_stream`
  walks the member blocks, decodes each via `WavPackBlock::decode_member_samples`
  (which accepts the marker instead of refusing it as a
  `MultichannelMember`), and interleaves the set's channels per frame into
  a `DecodedStream { samples, channels }`; plain mono / stereo files decode
  identically to `decode_stream` with `channels` reported as `1` / `2`.
  `decode_multichannel_stream_muted` is the spec §5.6 per-member CRC-mute
  twin (a member whose stored CRC mismatches is zeroed while the set's
  other members survive). `encode_multichannel_stream` is the bit-exact
  inverse: it de-interleaves a multichannel PCM buffer into one mono
  member per channel, tags the grouping markers, and splits long buffers
  into successive sets, so
  `decode_multichannel_stream(&encode_multichannel_stream(pcm, channels, …)?)?.samples
  == pcm` for any width `1..=MAX_MULTICHANNEL_CHANNELS` and frame count.
  `multichannel_layout` reports the `MultichannelLayout { channels, sets }`
  from block headers alone (no decode). Malformed grouping (stray final
  marker, unterminated set, per-member `block_samples` disagreement,
  channel-count blowup) is refused with the typed errors
  `MultichannelSetMalformed` / `MultichannelSampleCountMismatch` /
  `MultichannelTooManyChannels`.
* **Self-deriving compression encoder (auto / best)** — the encoder
  performs real prediction-based compression without the caller
  authoring any metadata. `serialize_mono_passes` /
  `serialize_stereo_passes` are the exact forward inverses of the
  assemblers (`assemble_*(serialize_*(passes)) == passes`, applying the
  §3.7 reverse-storage convention, the §2.1 `+5`-biased term byte via
  `encode_term_byte`, and the per-term per-channel seed partition),
  backed by public quantizers for the lossy on-wire log-packs
  (`pack_weight_byte` / `quantize_weight`, pinned as a true
  nearest-value inverse of the §3.6 weight expansion;
  `pack_sample_word` / `quantize_seed_sample` for the 16-bit
  exponent/mantissa seed words). `derive_mono_passes` /
  `derive_stereo_passes` bootstrap a pass list from the PCM itself — a
  zero-state training pass lets the §3.4 `±delta` adaptation walk each
  weight toward the block's actual correlation, then the trained
  weights are quantized to stored-byte values — and the
  `encode_block_*_auto` entry points compose derive → serialize →
  verbatim-payload encode, inheriting the bit-exact lossless guarantee.
  `encode_block_stereo_joint_with_decorr` / `encode_block_stereo_joint_auto`
  combine joint (mid/side) coding with decorrelation (prediction runs
  over the joint-transformed domain, matching the decoder's
  decorrelate-then-joint-undo order). `detect_left_shift` +
  `encode_block_mono_best` / `encode_block_stereo_best` search the
  whole mode grid — {plain, joint} × ({raw} ∪ {single-sweep,
  twice-iterated derived decorrelation per profile in the
  `DecorrProfile::search_set` effort ladder}) at the auto-detected
  sub-byte-depth shift — keeping the smallest output (every candidate
  decodes back bit-exactly, so selection is size-only);
  `encode_stream_mono_best` / `encode_stream_stereo_best` lift the
  search per-block across a whole file. The effort ladder now tops out
  at `DecorrProfile::Extra` — the spec §2.1 `MAX_NTERMS` (16-pass)
  ceiling — and `derive_*_passes_iterated` refines the stored starting
  weights over multiple `quantize ∘ train` sweeps.
* **Greedy term search + union "smallest" encoders** —
  `derive_*_passes_searched` picks each pass's term greedily from the
  full spec §2 valid set (`1..8`, `17`, `18`, cross terms on stereo)
  by measuring which candidate most reduces a residual magnitude-bits
  cost proxy, stopping when nothing strictly improves;
  `encode_block_*_searched` races the searched stack against raw.
  `encode_block_*_smallest` / `encode_stream_*_smallest` take the
  union of both search families (Extra-ceiling grid ∪ greedy search)
  and keep the smaller block per window. Measured on 22 050-sample
  synthetic signals (16-bit source): music-like mono 50.8% of the PCM
  bytes (the greedy search beats the `High` profile grid by ~10%),
  ramp-plus-noise mono 27.4% (`Extra` beats `High` by ~6.6%),
  correlated stereo 20.8% — the union wins every case by construction.
* **Seeking / block index** — sample-accurate random access from the
  wiki "Block structure" header fields alone. `StreamIndex::scan` is a
  header-only O(blocks) pass (no metadata parse, no audio decode)
  mapping every block's byte span and sample range (`IndexEntry`) and
  grouping audio blocks into decodable member sets (`SetEntry`) under
  the same wiki bits-11..=12 rules — and the same typed refusals —
  `decode_multichannel_stream` applies, so every stream the decoder
  accepts can be indexed. On a *seekable* index (sets forming one
  contiguous ascending frame chain; gaps / overlaps / regressions are
  reported by `is_seekable` and refused by the seek layer as
  `StreamNotSeekable`), `locate_frame` / `set_for_frame` binary-search
  the absolute frame domain and `set_byte_span` sizes ranged
  partial-file reads. `decode_range` / `decode_range_muted` decode an
  arbitrary frame window touching only the sets it overlaps —
  bit-exactly the same window sliced from the whole-stream decode, for
  mono / stereo / joint / left-shifted / multichannel shapes, with the
  muted twin applying the spec §5.6 per-member CRC gate window-scoped.
  `StreamReader` is the playback-shaped cursor over the same machinery
  (`seek` / `read_frames(_muted)` / `position` / `frames_remaining`),
  decoding whole sets, caching the most recent one per decode mode
  (cross-mode reuse only when the CRC verdict was clean), and
  restoring the cursor on a failed read (all-or-nothing).
* **Reference-decoder conformance (black-box cross-validated)** — the
  encoder's output is **byte-conformant with real WavPack decoders**:
  a round-393 cross-validation battery against `wvunpack` 5.9 (used
  strictly as an opaque binary; no reference source consulted)
  decodes every supported shape **bit-exactly** with zero
  CRC / missing-data reports — mono + stereo × raw / `*_best` /
  `*_smallest` / joint / 12-bit shifted / 24-bit / sparse zero-run /
  full-scale extremes, plus 4- and 6-channel member-set grouping
  (13/13 fixtures). The staged §5 CRC formulas, §4.2 Rice ladder and
  §3 decorrelation arithmetic were positively confirmed against
  reference-encoded probe files along the way. Getting there fixed
  four wire divergences the staged docs under-specified (canonical
  all-zero log-word for zero medians/seeds; no run-length field after
  a completed zero-run; **stream-level** — not per-channel — stereo
  holding state; the required `max_magnitude` and first-member `0x0D`
  fields) and two over-strict decode refusals (wiki bit 28 "robust,
  okay to ignore" and bit 5 `CROSS_DECORR` on lossless blocks, whose
  only documented consumer is the hybrid §4.1 fold — reference
  encoders set both on ordinary files).
* **`wp_log2` / `wp_exp2s` log-domain conversions + foreign-file
  decode** — the `logpack` module implements the staged
  `spec/wavpack-log2-exp2.md` integer log2/exp2 pair with the 256-entry
  tables transcribed from the staged CSVs: `wp_log2` (bit-length
  integer part + 8 fractional table bits, `>>9` interpolation bias),
  `wp_exp2s` (odd, implicit `0x100` mantissa bit, shift pivot 9), and
  the wire helpers `expand_log_word` / `pack_log_word` /
  `quantize_log_value`. The `0x05` medians and `0x04` seeds expand
  through `wp_exp2s` (replacing the wiki's linear shorthand, which
  diverges on every non-zero word), and `0x04` payloads prime a
  wire-order *prefix* of the pass list (remaining passes start from
  zero history). Net effect: files this crate did not encode decode
  bit-exactly — the r405 battery covers default / `-f` / `-h` / `-hh`
  / `-hh -x4` modes, 8/16/24-bit, mono / stereo / 5.1 / custom rates,
  with every stored CRC matching.
* **int32 (`INT32_DATA`) sample-format decode** — the `int32` module
  implements the staged `spec/wavpack-sample-formats.md` §3 `0x09`
  profile (`sent_bits` / `zeros` / `ones` / `dups`,
  mutually-exclusive redundancy enforced) and the §4 per-sample
  reassembly: `sent_bits` literal low bits read LSB-first from the
  `0x0C` extension bitstream, the stripped redundancy pattern
  re-inserted below them, and the §5.5 extension CRC (`crc_x`) folded
  over every reassembled value and compared against the `crc_wvx`
  stored at the head of the `0x0C` payload — the extension-CRC verdict
  joins the §5.6 mute gate. 32-bit reference files (sent-bits and
  trailing-zeros profiles, default + `-h`) decode bit-exactly.
* **float (`FLOAT_DATA`) sample-format decode** — the `float` module
  implements the staged §2 `0x08` profile: scaled-integer → IEEE-754
  reconstruction (static `float_shift`, per-sample mantissa
  normalisation anchored on `float_max_exp`), vacated low bits filled
  as zeros / ones (`SHIFT_ONES`) / literal `0x0C` bits (`SHIFT_SENT`),
  and `ZEROS_SENT` zero samples (marker bit → literal
  mantissa23+exponent8+sign1 for sub-integer magnitudes including
  denormals, or a true ±0 with a `NEG_ZEROS`-gated sign). The wire
  layouts the staged spec names but does not bit-pin were established
  black-box via differential probes, surfacing a **spec erratum**: the
  float extension CRC folds three mono-CRC steps per sample
  (mantissa, exponent, sign — `update_float_extension`), not the §5.5
  halfword formula (which holds for int32 only). Typed f32 surface:
  `WavPackBlock::is_float` / `decode_samples_f32` /
  `decode_stream_f32`. Every documented `0x08` profile shape decodes
  (round 408): full-precision (`SHIFT_SENT`), integer-valued,
  `SHIFT_ONES`, `SHIFT_SAME` (one-bit-per-non-zero-sample wvx
  carrier; zero samples spend no bit), ±0 / denormal / >1.0,
  `ZEROS_SENT`/`NEG_ZEROS`, and `EXCEPTIONS` — ±inf / NaN samples
  ride a bit-length-25 sentinel integer plus a wvx marker bit (`0` =
  infinity, `1` = the literal 23-bit NaN mantissa payload), pinned
  black-box and bit-exact against the reference decoder including
  exact NaN payload bits.
* **Sample-rate surface + time-addressed seeking** — the staged §5
  standard-rate table (`STANDARD_SAMPLE_RATES` + `sample_rate_index_for`
  + `Flags::standard_sample_rate`) and the `0x27`
  non-standard-sampling-rate sub-block (3-byte little-endian Hz;
  `parse_non_standard_sample_rate` / `find_non_standard_sample_rate`)
  resolve through `WavPackBlock::sample_rate` and the stream-level
  `stream_sample_rate`. `StreamReader` gains `sample_rate` +
  `seek_seconds` (typed `SampleRateUnknown` when a custom-rate stream
  lacks its `0x27`). On the write side `set_stream_sample_rate` stamps
  an encoded chain post-hoc (standard rates patch every header's rate
  index; non-standard rates set the sentinel `15` and append the
  `0x27` once, with the stream's first block) and the registry
  `WavPackEncoder` applies it from the caller-declared
  `CodecParameters::sample_rate` — the reference decoder reads both
  stamped forms and decodes the streams bit-exactly.
* **`0x0D` first-member channel geometry** — `parse_channel_info` /
  `ChannelInfo` (staged §6 erratum pin: `[count, mask]` with a
  little-endian Microsoft speaker mask; zero-length mask = "no
  assignment"; the extended >32-channel form is a typed refusal),
  exposed via `WavPackBlock::channel_info` and `stream_channel_info` —
  the reference 5.1 fixture declares `count 6`, mask `0x3F`, matching
  the decoded interleave width.
* **Framework registry wiring (dual API)** — `register` installs the
  codec into an `oxideav_core::RuntimeContext` (`CodecInfo`:
  decode + encode, lossless, SW priority, the staged-wiki `WVPK`
  FourCC tag) behind the `oxideav_core::register!` entry point, and
  the direct factories are exposed per the workspace convention as
  `decoder::make_decoder` / `encoder::make_encoder`. The packet
  contract is "complete `wvpk` blocks per packet": `WavPackDecoder`
  turns each packet into one `AudioFrame` via
  `decode_multichannel_stream` (bytes packed at the stream's container
  width — S8/S16/S24/S32 interleaved, values verbatim), and
  `WavPackEncoder` emits one packet per input frame with a **running
  `block_index`** (mono / stereo through the `encode_block_*_best`
  mode search; wider layouts through the mono-member grouping via the
  new offset-aware `encode_multichannel_stream_at`), so concatenated
  packet payloads form one contiguous, seekable `.wv` chain.
* **`.wvc` correction-file pairing plumbing** —
  `pair_correction_stream` aligns a main `.wv` buffer's audio blocks
  with its companion `.wvc` buffer's by the `block_index` header word
  (per-pair agreement on `block_samples` and the mono flag enforced;
  orphan / surplus / mismatched correction blocks are typed refusals;
  partial coverage pairs `None`), `correction_coverage` summarises
  `(paired, total)`, and `WavPackBlock::expects_correction` classifies
  which blocks *want* a twin (wiki bit-3 hybrid flag). Structural
  plumbing for the two-file lossless path — folding the paired `0x0B`
  words into the (now-decoded) coarse PCM stays gated on the shaped
  correction-fold docs gap below.
* **Entropy encode** — exact write-side inverses (`BitWriter`,
  `encode_packed_samples_mono` / `_stereo`, and the per-primitive
  interval / prefix / mantissa encoders) round-trip the decode ladder
  bit-for-bit.
* **Decorrelation encode** — `recorrelate_mono` / `recorrelate_stereo`
  are the exact arithmetic inverse of `decorrelate_mono` /
  `decorrelate_stereo` (decorrelation-spec §3 forward direction:
  PCM → residuals). They emit `residual = sample - apply_weight(weight,
  pred)` for the same predictor the decoder reads, push the original PCM
  sample into history as the decoder pushes its reconstructed sample, and
  apply the identical `update_weight` / `update_weight_clip` step — so
  both directions evolve byte-identical pass state. Per spec §3.7 both
  accept the *same* application-ordered `DecorrPass` list (the encoder
  walks it back-to-front, undoing the decoder's reversal), covering every
  fixed-lag (`1..8`) / extrapolate (`17`/`18`) / cross (`-1`/`-2`/`-3`)
  term and multi-pass stacks. Pinned by `recorrelate ∘ decorrelate`
  round-trips over a shared pass list (mono + stereo, single + multi
  pass + cross terms) that reproduce the original PCM, parity against the
  private single-pass forward helpers, and the refusal arms.
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

// Encode: the smallest stream this encoder can produce (self-derived
// decorrelation, joint-stereo decision, left-shift detection, all
// bit-exactly lossless):
use oxideav_wavpack::{encode_stream_stereo_best, DecorrProfile};
let wv = encode_stream_stereo_best(&pcm, 0, 2, DecorrProfile::High)?;
assert_eq!(decode_stream(&wv)?, pcm);

// Or the union of every search family this crate has (Extra-ceiling
// profile grid ∪ greedy term search), smallest block per window:
use oxideav_wavpack::encode_stream_stereo_smallest;
let wv = encode_stream_stereo_smallest(&pcm, 0, 2)?;
assert_eq!(decode_stream(&wv)?, pcm);

// Foreign files (reference-encoded) decode the same way — and float
// streams have a typed f32 twin:
use oxideav_wavpack::{decode_stream_f32, stream_channel_info, stream_sample_rate};
let rate = stream_sample_rate(file_bytes)?;        // Some(44100) / 0x27 custom
let geometry = stream_channel_info(file_bytes)?;   // 0x0D [count, mask]
let f32_pcm = decode_stream_f32(float_file_bytes)?;

// Hybrid-lossless two-file (.wv + .wvc) decode — every pair shape the
// reference encoder produces, CRC-gated by the .wvc header's lossless
// CRC, with float / multichannel twins:
use oxideav_wavpack::{
    decode_multichannel_stream_with_correction, decode_stream_with_correction,
    decode_stream_with_correction_muted,
};
let pcm = decode_stream_with_correction(wv_bytes, wvc_bytes)?;   // bit-exact original
let (pcm, all_crc_ok) = decode_stream_with_correction_muted(wv_bytes, wvc_bytes)?;
let surround = decode_multichannel_stream_with_correction(wv_51, wvc_51)?; // 5.1 pairs

// Originate the formats you used to only decode (round 418) — all
// bit-exact through this decoder AND the reference decoder binary:
use oxideav_wavpack::{
    encode_block_stereo_float_best, encode_stream_mono_int32, encode_stream_stereo_hybrid,
    HybridOptions,
};
let wv = encode_block_stereo_float_best(&f32_pcm, DecorrProfile::High, 0, frames)?;
let wv = encode_stream_mono_int32(&wide_pcm, 0, DecorrProfile::Normal)?;
// Hybrid: lossy .wv at ~4 bits/sample + the .wvc twin that restores
// the input exactly through the pair decode:
let pair = encode_stream_stereo_hybrid(&pcm, 0, 2, &HybridOptions::from_bits_per_sample(4.0))?;
assert_eq!(decode_stream_with_correction(&pair.wv, pair.wvc.as_ref().unwrap())?, pcm);

// Seek: index the stream once (header-only), then decode windows —
// or drive the playback-shaped cursor (frame- or time-addressed):
use oxideav_wavpack::{decode_range, StreamIndex, StreamReader};
let index = StreamIndex::scan(&wv)?;
let window = decode_range(&wv, &index, 44100, 1024)?; // frames 44100..45124
let mut reader = StreamReader::new(&wv)?;
reader.seek_seconds(1.0)?; // == reader.seek(44100)? at 44.1 kHz
let frames = reader.read_frames(1024)?; // == window
```

## Not yet supported

`WavPackBlock::decode_samples` refuses only low-latency block layouts
(wiki bit 31, "do not decode") with a typed
`Error::UnsupportedBlockFeature`. **Hybrid (lossy) blocks are decoded**
(round 408 — see below). **Float and 32-bit-int sample data are
decoded in full** (rounds 405/408 — every documented `0x08` /
`0x09` profile shape, see the working-surface bullets), with the
`crc_x` extension-CRC verdict wired into the §5.6 mute gate. **Multichannel members** are
handled at stream level: `decode_samples` still refuses a grouped
member (its per-block shape can't stitch the set), but
`WavPackBlock::decode_member_samples` decodes one and
`decode_multichannel_stream` reassembles the whole interleaved frame.
The §5 block CRC is exposed at block level via
`WavPackBlock::verify_decoded_crc` / `decode_samples_muted` and at
stream level via `decode_stream_muted`.

The **left-shift** half of the decorrelation-spec §1 "shift/clip fixups"
stage is now applied (see the working-surface bullet above); the **clip**
half is the wiki flag-bits-18..=22 `max_magnitude`, which the wiki
documents only as an optimisation *hint* ("can be used to optimize
decoding arithmetic"), not a value the decoder must clamp against, so no
clip is applied for correctness.

**Hybrid (lossy) `.wv` files decode end-to-end** (round 408): the
staged spec §6.5 `error_limit` model is implemented with its exact
integer arithmetic pinned black-box against reference decodes — the
`0x06` profile seeds per-channel `slow_level` accumulators
(`HybridProfile` / `HybridState`), every sample folds
`slow_level -= (slow_level + 128) >> 8; slow_level += wp_log2(mag)`,
the per-sample limit is `wp_exp2s(ema - bitrate + 256)` with the
stereo pair redistributed at frame start by
`delta = (ema0 - ema1 - balance) >> 1`, and the step-6 mantissa read
becomes the §6.5 bracketing binary search
(`SampleInterval::decode_bracketed_value`). Nine reference-encoded
hybrid fixtures (mono at the bitrate extremes, multi-block with a
silence stretch, mid/side + balance stereo, `-j0` left/right, 5.1,
24-bit, lossy float, false-stereo) decode **bit-exactly** with every
stored block CRC matching. False-stereo blocks (bit 30) now emit both
duplicated output channels everywhere (a round-408 conformance fix).

**Hybrid-lossless (`.wv` + `.wvc`) pairs decode end-to-end** for mono
and both stereo codings (rounds 408/415):
`WavPackBlock::decode_samples_with_correction` /
`decode_stream_with_correction` read the exact in-bracket offset from
the `0x0B` correction stream (the same phase-in code as the lossless
mantissa) and run the `0x07`-seeded noise-shaping filter
(`ShapingState`: log-packed `[error, acc, (delta)]` seeds, the
`-((weight*error + 511) >> 10)` temp term, the negative-weight
unit-magnitude nudge, and the weight-sign-branched error update).
**Joint (mid/side) stereo pairs** — the reference encoder's default
stereo hybrid coding — are a round-415 black-box pin: the raw `0x0B`
fold stays in the coded domain (ahead of the §5.4 undo), but the
shaping filter's two channels are **output** (left/right) channels;
per frame the coded temps are `t_m = t_l - t_r` and
`t_s = t_r + ((mid + t_m) >> 1) - (mid >> 1)` on the output-domain
mid (bracketed samples only), with the effective per-output deltas
folded back into the error states
(`decode_packed_samples_stereo_hybrid_lossless_raw` + the
post-decorrelation shaping leg). Ten pair fixtures reproduce the
original encoder input bit-exactly across no-shaping / static
negative / static positive / dynamic-noise-shaping encodes,
left/right + joint coding, 16- and 24-bit, single- and multi-block
with silence stretches. `CROSS_DECORR`-flagged pairs (the reference
maximum-compression hybrid mode) are a further round-415 pin: the bit
is decorative on current-version files — three cross-flagged fixtures
(joint dynamic / joint unshaped / left-right dynamic) decode
bit-exactly under the same post-decorrelation fold, so
`fold_hybrid_correction` / `split_hybrid_correction` accept the cross
placement and only the noise-shaped placement still routes through
the full pair decode. **Float and int32 pairs decode losslessly**
(round 415): a pair encode carries the `0x0C` wvx extension stream in
the `.wvc` twin alongside `0x07`/`0x0B` (structural pin), so the
sample-format fixup on a pair decode reads its literal
mantissa / sent-bits from the correction block —
`decode_stream_with_correction_f32` is the typed float twin — and the
**lossy** `.wv`-only decode of a sent-bits int32 hybrid file fills
the missing wvx window with implied zeros (`reassemble_int32_implied`,
mirroring the round-408 float posture), pinned bit-exact against the
reference lossy decode. The pair decode is CRC-gated end-to-end: the
`.wvc` header stores the §5 running CRC of the **lossless** decode
(round-415 pin — folded over the pre-fixup samples, exactly as the
`.wv` header's covers the lossy decode), so
`decode_samples_with_correction_muted` /
`decode_stream_with_correction_muted` apply the §5.6 mute gate
against it (extension `crc_x` verdict included), zeroing a corrupt
block instead of emitting wrong samples. Multichannel pairs decode
through `decode_multichannel_stream_with_correction` (+ muted twin):
member blocks pair one-to-one across the two files and each decodes
hybrid-lossless before the set interleave — a 5.1 pair fixture
reassembles bit-exactly. The **seek surface is pair-aware** too:
`decode_range_with_correction` (+ muted twin) decodes an arbitrary
frame window losslessly by pairing each touched member set with the
correction chain's counterpart (matched by frame range on a
header-only scan of the `.wvc`; uncovered sets fall back lossy), and
`StreamReader::new_with_correction` is the playback-shaped cursor over
the same machinery — windows and chunked reads are pinned bit-equal
to the whole-stream pair decode for joint, multi-block, and 5.1
shapes. Every hybrid shape the reference encoder produces now
decodes: no hybrid gaps remain. Three round-418 conformance pins
harden the §6.5 model further, each backed by a committed edge-probe
fixture: the stereo redistribution **delta clamps to `±bitrate`**
(extreme-imbalance joint content; erratum to the round-408 pin, which
never left the near-balanced regime), the lossy reconstruction
**saturates to the effective bit-depth range** (±2^15 on 16-bit,
container-minus-total-shift on int32) as a final pass *after* the
unclamped §5 CRC fold, and the `.wv`-only int32 fill **zero-fills the
whole reduced window** (`ones`/`dups` patterns are only restored on
the pair path).

Both round-404 docs gaps are closed (round 405): **foreign
reference-encoded files decode bit-exactly** via the staged
`wp_log2` / `wp_exp2s` transcription, and **seeking is
time-addressed** via the staged sample-rate table + `0x27`
(`StreamReader::seek_seconds`). The crate is wired into the
`oxideav-core` framework registry: `Decoder` / `Encoder` trait impls,
the `register` entry point, and the direct `decoder::make_decoder` /
`encoder::make_encoder` factory endpoints; the registry encoder stamps
the caller-declared sample rate into every emitted chain.

**Float / int32 / hybrid origination** (round 418): the write side
now covers the full sample-format and hybrid surface. `deconstruct_float`
derives the `0x08` profile from the data itself (fill-mode selection
across zero / `SHIFT_ONES` / `SHIFT_SAME` / `SHIFT_SENT`, the raised
`SHIFT_SAME` anchor, `ZEROS_SENT`+`NEG_ZEROS` literals for `-0.0` /
denormals / below-range values, the `EXCEPTIONS` sentinel with exact
NaN payloads, the static shift from shared trailing zeros) and
`deconstruct_int32` its §3 twin (free `zeros`/`ones`/`dups` redundancy
stripping plus literal `sent_bits` to the 23-bit entropy target);
`encode_block_{mono,stereo}_{float,int32}[_best]` and the
`encode_stream_*` twins ride the ordinary lossless pipeline with the
profile / `0x0C` sub-blocks attached. `encode_block_{mono,stereo}_hybrid`
(+ `_float` / `_int32` variants and `encode_stream_*` twins) originate
the §6.5 model: the bracketing search emits `exact_mag >= mid` decision
bits, the `0x0B` stream carries the exact in-bracket offset with the
lossless phase-in code, the per-sample steppers track the decoder's
coarse-value prediction state, the `0x06` profile is data-derived with
the running `slow_level` carried across stream blocks, and the `.wvc`
twin stores the lossless §5 CRC. A hybrid float encode raises the
exponent anchor one (coarse-overshoot head-room) and sends inf/NaN
through the `ZEROS_SENT` literal path so the lossy `.wv` stays
decodable alone; a pair encode moves the `0x0C` extension stream to
the `.wvc`. Hybrid decorrelation stacks are cross-free (the reference
shape — a cross pass in a hybrid block decodes differently under the
reference decoder). No `0x07` shaping is emitted (the raw §4.1 fold).

## Provenance

Clean-room from the staged material under `docs/audio/wavpack/`: the
block-structure and sub-block-ID wiki listing, the clean-room
entropy-decode trace `docs/audio/wavpack/spec/wavpack-entropy-decode.md`,
the decorrelation/CRC trace
`docs/audio/wavpack/spec/wavpack-decorrelation.md`, and — round 405 —
the log-domain conversions `spec/wavpack-log2-exp2.md` (tables
mechanically transcribed from `tables/wp-log2.csv` / `wp-exp2.csv`),
the extended sample formats `spec/wavpack-sample-formats.md`, and the
standard-rate table `tables/sample-rates.csv`. Wire details the staged
docs name but do not bit-pin (the float `ZEROS_SENT` layout and the
float extension-CRC fold) were established black-box with the
reference binaries as opaque oracles, never their source. No external
library source, archived prior history, or online resources were
consulted at any phase. A `cargo-fuzz` harness in `fuzz/` fuzzes the `decode_stream`
and `decode_multichannel_stream` (plus its CRC-muted and `multichannel_layout`
twins) entry points, carries an `encode_roundtrip` **round-trip
oracle** target (fuzz bytes → PCM + mode-grid control → `*_best` encode
→ assert bit-exact decode + CRC gate; the control byte now sweeps all
four profiles), an `introspection_surface` target asserting the
cross-walker invariants over every non-decoding stream walker plus the
`.wvc` pairing walker at a fuzz-chosen split, and a round-393
`seek_surface` **differential** target (scan never stricter than the
decoder; index tiling / set / locate invariants; full-span +
fuzz-chosen-window `decode_range` and chunked `StreamReader` walks
bit-equal to the whole-stream decode, muted PCM + verdict parity
included), and a round-415 `correction_pair_decode` **differential**
target over the two-file pair surface (plain/muted parity, the
empty-correction identity against the lossy decode, and
plain/multichannel walker parity, at a fuzz-chosen `(main,
correction)` split), plus a round-418 `hybrid_encode_roundtrip`
**pair-decode oracle** over the origination surface (fuzz PCM +
control byte sweeping mono/stereo × joint × decorrelation ceiling ×
bitrate word × plain/float/int32; every emitted pair must decode back
bit-exactly with green CRC gates and every lossy `.wv` must decode
within the clamp range — an opening ~5M-run campaign found, and the
fix pinned, a clamp-bits underflow on hostile left-shift headers). A round-386 campaign
found (and the fix pinned) an adversarial-history overflow in the
term-17/18 extrapolator predictors — all twelve predictor sites are
now 32-bit wrapping, matching the wrapping reconstruction adds around
them, with the minimized input kept as a corpus regression seed; a
round-415 campaign did the same for two overflow sites in the `0x07`
shaping-state recurrence (accumulator add / temp bias / seed
negation, all now 32-bit wrapping);
1107 unit tests plus a 74-test
foreign-decode integration battery (59 reference-encoded fixtures
under `tests/data/` — including 18 hybrid-lossless `wv+wvc` pairs and
the round-418 hybrid edge probes (delta clamp / output clamp /
implied-fill) — all pinned bit-exact with matching stored CRCs,
plus corruption trip-wires through both CRC gates) synthesise
minimal valid headers /
sub-blocks / bitstreams and poison each field to exercise the accept /
reject boundaries, pin the §5 CRC primitives to the spec's worked
mono / stereo CRC vectors, pin the §3 decorrelation weight
arithmetic to the spec's §7 sanity vectors, pin the §3 forward
decorrelation encoder (`recorrelate_mono` / `recorrelate_stereo`) as the
exact inverse of the decode loop via shared-pass-list round-trips, pin the
multichannel grouping decode/encode (`decode_multichannel_stream` /
`encode_multichannel_stream`) as a bit-exact interleave round-trip across
channel widths and member-set splits, pin the
wiki flag-bits-13..=17 left-shift fixup (the decorrelation-spec §1 "shift"
normalization stage) end-to-end for mono and stereo decode plus its
pre-shift CRC ordering, and pin the §4.1 hybrid correction-fold
(`fold_correction` / `fold_correction_pre_decorrelation` /
`split_correction` / `CorrectionFold` + the block-level
`fold_hybrid_correction` / `split_hybrid_correction`) as the exact
fold-∘-split lossless-recovery inverse across the post-decorrelation
placement, with the `CROSS_DECORR` / noise-shaped placements refused,
and pin the forward decorrelation-metadata serializers
(`serialize_mono_passes` / `serialize_stereo_passes` as
assemble-inverses; the weight quantizer against exhaustive
stored-byte search) plus the self-deriving auto / best encoders'
lossless round-trips, mode-search dominance, and search-ceiling
monotonicity.

## License

MIT. See `LICENSE`.
