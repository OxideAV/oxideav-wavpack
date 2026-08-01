# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Round 436 — **stream-level decode budget (`*_bounded` twins).** The
  per-block anti-amplification ceiling (round 296) bounds what one
  block may claim, but the spec §4.2 step 1 zero-run fast path lets
  *every* ~50-byte block in a chain legally expand to millions of zero
  samples — a few kilobytes of hostile input could still demand
  gigabytes of concatenated output from `decode_stream`. New
  hostile-input-hardened twins `decode_stream_bounded`,
  `decode_stream_muted_bounded`, `decode_stream_f32_bounded`,
  `decode_multichannel_stream_bounded` and
  `decode_multichannel_stream_muted_bounded` take a caller-supplied
  budget on the **total emitted sample values**, charge each audio
  block's declared output size (`decoded_sample_count`, a header-only
  quantity) against it **before** decoding the block, and surface the
  new typed `Error::DecodeBudgetExceeded { budget, needed }` ahead of
  any amplified allocation. Within the budget the results are
  bit-identical to the unbounded decoders (which are unchanged, and
  now share one walker with the bounded twins). The two-file pair
  surface gets the same hardening:
  `decode_stream_with_correction_bounded` (+ `_muted` / `_f32` and
  multichannel twins) charge each `.wv` block's header-declared output
  size before its decode — the `.wvc` twin refines the same samples
  and adds no output of its own. A
  264-point black-box sweep over ramp trajectories (start weight ×
  delta × content, two block lengths) found that trajectories crossing
  **below** weight `-1024` can decode differently under the reference
  decoder — the staged spec names an IIR variant for negative shaping
  weights (bit 29) whose exact past-full-scale recurrence is an open
  docs gap — while every static weight across the full `±1024` range,
  every in-range ramp, and even far-past-full-scale **positive**
  trajectories are bit-exact under it. The encoder now keeps every
  emitted `0x07` inside the validated envelope: per block the delta
  word is shrunk (down to 0) so the accumulator cannot leave
  `±(1024 << 16)` within the block — the ramp runs to the rail and
  holds — and the re-packed accumulator words are clamped against
  log-quantization overshoot at the rail. The re-swept battery is
  clean: 264/264 sweep points at both block lengths plus the 36-case
  named battery (shaping sweep, ramp-overrun probes, 24-bit shaped
  joint, delta-clamp imbalance, clip16, int32-ones, float hybrid,
  registry lossless paths) all decode bit-exactly (lossy) and
  losslessly (pair) under the reference decoder.

- Round 420 — **forward-direction edge-probe battery.** The round-418
  hybrid-conformance pins re-validated in the encode direction:
  extreme-imbalance joint content (side- and mid-collapsing, bitrate
  words 200/456/800 — the §6.5 delta-clamp regime), clipping-adjacent
  16-bit content at the coarsest word (unclamped-CRC / post-clamp
  contract, shaped and unshaped), trailing-ones int32 hybrid pairs
  (implied-fill window, shaped joint) and shaped float hybrid pairs.
  Black-box tally: 30 originated streams (the 20-variant shaping sweep
  plus these 10 edge probes) all reproduce their lossy PCM bit-exactly
  under the reference decoder and recover the original losslessly from
  the `.wv`+`.wvc` pair; the registry's float / int32 lossless block
  paths were reference-verified the same way.

- Round 420 — **registry float / hybrid wiring.** The framework
  encoder now accepts interleaved `F32` (through the `0x08` float
  deconstruction) and routes `S32` through the `0x09` int32
  deconstruction — both lossless and both reference-validated
  black-box. A typed options schema (`WavPackEncoderOptions`, parsed
  from `CodecParameters::options`) adds `mode=lossless|hybrid`,
  `bits_per_sample` (the reference `-b` noise-target scale, default
  4.0), `shaping` (fractional noise-shaping weight, default 0 = off)
  and `joint` (hybrid mid/side coding, default true); hybrid mode
  emits lossy `.wv` packets whose blocks carry the running `0x06`
  level / `0x07` shaping-state carry across packet boundaries, so the
  concatenated packets stay one seekable conformant chain. Requesting
  the `.wvc` twin (`correction=true`) is refused with a pointer at the
  crate-level pair APIs — the framework packet contract is
  single-stream. Hybrid-only knobs without `mode=hybrid`, hybrid
  multichannel, and `F32`/`S32` multichannel are typed refusals. The
  stale registry module note claiming hybrid / float / int32 decodes
  are refused is gone: the decoder surface has decoded all three since
  rounds 408..418 (float PCM returns as IEEE-754 bit patterns in the
  4-byte slots, byte-identical to interleaved `F32`).

- Round 420 — **noise-shaping origination (`0x07` emission).**
  [`HybridOptions`] gains a `shaping` axis ([`HybridShaping`]: `Off` /
  `Static` weight in 1/1024 units / `Ramp` with a per-sample
  accumulator delta; `HybridShaping::from_weight` maps the fractional
  `-1.0..=1.0` scale). When selected, every §6.5-bracketed word
  targets the **shaped** residual `exact - temp` — the staged spec
  §4.1 error-feedback term, computed in exact decoder lockstep through
  `ShapingState` — so the lossy stream's quantization-noise spectrum
  tilts with the weight while the `.wv` + `.wvc` pair decode stays
  bit-exact. The `.wvc` twin leads its metadata chain with the block's
  `0x07` seed (short static / long delta-bearing layouts, log-word
  packed via the new `ShapingState::to_shaping_words`), header bits
  6/29 are both set (the flag combination observed on every reference
  shaped block), and multi-block streams carry the quantized filter
  state across block boundaries exactly as the level words are
  carried. Joint (mid/side) blocks run the filter per **output**
  channel with the round-415 coded-domain temp transform (including
  the half-step term and the effective per-output error-state deltas);
  left/right and mono blocks fold temps per coded channel. Zero-run
  and lossless-dispatch words apply no temp and reset the error state,
  mirroring the pinned decode paths. Float / int32 hybrid encodes
  inherit the axis unchanged. Black-box validated: 20 originated
  variants (mono / left-right / joint × off / ±0.7 / full-scale /
  ramping weights × single- and multi-block) all reproduce their lossy
  PCM bit-exactly under the reference decoder and recover the original
  losslessly from the pair — confirming the emitted `0x07` seeds drive
  the reference's own shaping arithmetic identically. The
  `hybrid_encode_roundtrip` fuzz target now sweeps the shaping axis
  from a second control byte (clean 120 s / 344 k-run opening
  campaign).

### Changed

- Internal plumbing re-exports (entropy-ladder primitives, decorrelation /
  CRC / log-pack arithmetic, metadata walkers and finders, sub-block
  expanders, pass-derivation helpers — 188 items kept `pub` for the test
  suites) are now `#[doc(hidden)]`, so rustdoc and semver tooling track
  only the stable decode / encode / seek / registry surface. No semantic
  or signature changes.

### Added

- Round 418 — **`hybrid_encode_roundtrip` fuzz target** — the
  pair-decode oracle over the origination surface: fuzz bytes become
  PCM + a control byte sweeping mono/stereo × joint × decorrelation
  ceiling (raw included) × bitrate words 0..2000 × plain/float/int32
  formats; every emitted pair must decode back bit-exactly with
  green CRC gates, every lossy `.wv` must decode and stay within the
  clamp range. Seeded with a curated starter corpus; a 7-minute
  opening campaign (~5M runs) plus regression slices over the decode
  targets came back clean after the clamp-underflow fix.

- Round 418 — **float / int32 hybrid pairs.**
  `encode_block_{mono,stereo}_hybrid_{float,int32}` and their
  `encode_stream_*` twins compose the sample-format deconstructions
  with the hybrid pipeline: the `0x0C` extension payload rides the
  `.wvc` twin (round-415 structural pin), the lossy `.wv` decodes
  with the implied-zero fill, and the pair decode restores the exact
  input — IEEE-754 bit patterns (inf / NaN / `-0.0` / denormals
  included) and full-range `i32` alike. The hybrid float shape
  raises the `0x08` exponent anchor one above the largest finite
  exponent (23-bit integer magnitudes leave head-room for the
  coarse overshoot) and routes exceptional samples through the
  `ZEROS_SENT` literal path instead of the bit-length-25 sentinel
  (whose wvx marker only exists in the `.wvc`, which the lossy
  stream doesn't have — as literals they decode to an implied
  `+0.0` lossy and the exact pattern through the pair).

- Round 418 — **hybrid origination (lossy `.wv` + lossless
  `.wv`+`.wvc` pairs).** The encoder now originates the §6.5
  `error_limit` model it learned to decode in rounds 408/415:
  `encode_block_{mono,stereo}_hybrid` / `encode_stream_*_hybrid`
  produce a lossy `.wv` at a caller-chosen bitrate word
  (`HybridOptions::from_bits_per_sample` maps the reference `-b`
  scale, `bits*256 - 568`) plus, on request, the `.wvc` correction
  twin that restores the input **bit-exactly** through the pair
  decode. The write side mirrors every decode pin: the bracketing
  binary search emits `exact_mag >= mid` decision bits and the
  `0x0B` stream carries the exact in-bracket offset with the
  lossless phase-in code; the encoder tracks the decoder's
  coarse-value state through the new per-sample steppers (prediction
  offsets from coarse history, weight updates on coarse residuals);
  the `0x06` profile is data-derived (running `slow_level` packed as
  log words, carried across stream blocks; balance 256 on joint /
  0 elsewhere; flag bits 9/10 mirrored from the reference shape);
  the `.wvc` header stores the lossless decode's §5 CRC. No `0x07`
  shaping is emitted (the raw §4.1 fold — the reference's
  shaping-off shape). Hybrid decorrelation stacks are cross-free
  (see the erratum below). Validated black-box on an 18-case
  battery: every pair reproduces the original through the reference
  decoder binary byte-exactly, and every lossy `.wv` decodes
  identically through this crate's decoder and the reference
  (mono/stereo × raw/derived stacks × joint/left-right × bitrate
  words 0..800 × multi-block × 24-bit × silence).

### Fixed

- Round 418 — **left-shift underflow in the new hybrid output
  clamp** (found within minutes by the fuzz battery; the offending
  input is kept as a corpus regression seed): a hostile header
  combining a hybrid flag with a left-shift count past the container
  width underflowed the clamp's effective-bits subtraction in debug
  builds. The subtraction now saturates (the existing 1..=32 floor
  keeps the range well-formed).

- Round 418 — **hybrid lossy output clamp** (black-box pin): the
  reference decoder saturates each lossy-path reconstructed sample
  to the signed range of the effective pre-fixup bit depth — 16-bit
  content clamps at exactly ±2^15, an int32 `sent+redundancy = 8`
  reduction clamps its pre-fixup values at ±2^23 — as a final pass
  over the pre-fixup buffer: the clamp does **not** feed back into
  the prediction state, and the §5 block CRC folds over the
  **unclamped** reconstruction (a clamped-CRC stream is reported as
  a CRC error). `WavPackBlock` now applies the clamp between the
  CRC fold and the sample-format fixups on every decode path (the
  pair fold included, where it is the identity for in-range
  samples). Pinned by clipping-adjacent probe batteries at `-b2`.

- Round 418 — **implied int32 fill covers the redundancy too**
  (erratum to the round-415 sent-bits pin): the `.wv`-only lossy
  decode of an `INT32_DATA` block shifts by the **total** reduction
  and zero-fills the whole window — the `ones` / `dups` patterns
  are *not* re-inserted (reference-encoded trailing-ones and
  duplicated-low-bit profiles decode with all-zero low windows,
  `sent_bits = 0` profiles included), while the pair decode still
  restores the exact pattern. `reassemble_int32_implied` now
  zero-shifts by `total_shift()`, and the fixup plumbing carries a
  `pair` flag so a two-file decode never selects the lossy fills.

- Round 418 — **§6.5 stereo redistribution erratum: `delta` clamps
  to `±bitrate`.** The round-408 pin derived
  `delta = (ema0 - ema1 - balance) >> 1` from near-balanced
  fixtures, where the clamp is invisible; extreme-imbalance joint
  probes (side ≈ 0 or mid ≈ 0, unclamped delta past ±900) decode
  wrongly from sample 0 under the unclamped model. A round-418
  black-box constraint-mining pass (recover the reference's coarse
  residuals by inverting the decorrelation, replay the bitstream,
  bound each bracketed word's `error_limit` by its stopping width)
  solved the effective delta to exactly `+bitrate` / `-bitrate`
  across the `-b3`/`-b4`/`-b5` range — `HybridState::frame_limits`
  now applies the clamp, and the extreme-imbalance probes decode
  bit-exactly. Two more reference-decoder pins fell out of the
  encode battery: a **zero-length `0x0B` sub-block is rejected**
  (an all-run block's empty correction payload must be omitted;
  the pair decode already treats a missing `0x0B` as an empty
  stream), and **hybrid decorrelation stacks are cross-free** —
  reference hybrid encodes never carry cross terms and a `-1`
  cross pass in a hybrid block decodes differently under the
  reference decoder than under the lossless-identical model, so
  the originator filters cross terms out.

- Round 418 — **float / int32 origination.** The encoder now
  *originates* `FLOAT_DATA` and `INT32_DATA` streams instead of only
  decoding them. `deconstruct_float` derives the `0x08` profile from
  the data itself — the `float_max_exp` anchor (raised one for the
  `SHIFT_SAME` carrier shape), the static `float_shift` from shared
  trailing zeros, the vacated-window fill mode (zero / `SHIFT_ONES` /
  `SHIFT_SAME` / `SHIFT_SENT`), `ZEROS_SENT`+`NEG_ZEROS` literals for
  `-0.0` / denormals / below-range values, and the `EXCEPTIONS`
  bit-length-25 sentinel for inf/NaN (exact payload bits preserved) —
  and emits the scaled integers plus the `0x0C` extension payload
  (`crc_wvx` + packed bits). `deconstruct_int32` is the §3 twin:
  free redundancy stripping (`zeros`/`ones`/`dups`, mutually
  exclusive, whichever pattern every sample shares) plus literal
  `sent_bits` down to the 23-bit entropy magnitude target. Public
  entry points: `encode_block_{mono,stereo}_{float,int32}` (raw),
  `..._best` (raw ∪ derived-decorrelation grid, {plain, joint} on
  stereo, over the deconstructed integer domain) and
  `encode_stream_{mono,stereo}_{float,int32}` (per-chunk profiles).
  Every emitted stream decodes bit-exactly through this crate's own
  decoder (both CRC gates green) **and** through the reference
  decoder binary (black-box: 10-case battery — full-precision /
  int16-shaped / special-value / wide-range float, mono + stereo +
  multi-block; full-range / trailing-zero / correlated int32 —
  byte-identical raw output). One wire pin fell out: the **`0x0C`
  extension payload must occupy an even byte count** like the `0x0A`
  main stream — the reference decoder rejects an odd-size-flagged
  `0x0C` sub-block outright ("not compatible"), so the packed
  extension bits are padded to even length.

- Round 415 — **pair-aware seeking.** `decode_range_with_correction`
  (+ its §5.6-muted twin) decodes an arbitrary frame window of a
  hybrid `.wv` losslessly against its `.wvc`: each member set the
  window touches pairs with the correction chain's counterpart set,
  matched by frame range on a header-only `StreamIndex::scan` of the
  correction buffer (partial coverage falls back to the coarse lossy
  decode, matching the stream walker's posture), and muted windows
  gate paired members against the `.wvc` header's stored lossless
  CRC. `StreamReader::new_with_correction` opens the playback-shaped
  cursor over the same machinery, set-cache included. Pinned by
  window-vs-whole-stream and chunked-reader-parity tests over joint,
  multi-block and 5.1 pair fixtures, plus a full-span parity
  invariant in the `correction_pair_decode` fuzz target (guarded to
  both-sides-seekable indices, where set-based pairing provably
  coincides with the stream walker's sequential pairing).

### Fixed

- Round 415 — **two adversarial-input overflow sites in the `0x07`
  shaping state** (found by the new `correction_pair_decode` fuzz
  target; minimized inputs kept as corpus regression seeds): the
  per-sample accumulator add and the `temp` product/bias now wrap in
  32 bits (the same posture as the round-386 wrapping predictors),
  the negative-weight nudge computes `|err| - 1` in `u32` (an
  `i32::MIN` error state no longer overflows), and the error-seed
  negation wraps (an adversarial log word can expand to `i32::MIN`).
  Reference-encoded streams are unaffected — every conformance
  fixture still decodes bit-exactly.

### Added

- Round 415 — **`correction_pair_decode` differential fuzz target**
  over the two-file pair surface: plain/muted parity (the muted path
  may out-succeed the plain path only on a failed gate, preserves the
  PCM shape, and is bit-identical on a clean gate), the
  empty-correction identity against the single-file lossy decode, the
  f32 twin's panic-freedom, and plain/multichannel walker parity on
  1–2-channel streams, over a fuzz-chosen 16-bit-fraction split of
  the input into `(main, correction)` halves. Corpus seeded with
  seven pair fixtures split exactly at their `.wv`/`.wvc` boundary.

- Round 415 — **multichannel (member-set) hybrid-lossless pair
  decode.** `decode_multichannel_stream_with_correction` (+ its
  §5.6-muted twin) aligns member blocks one-to-one across the `.wv`
  and `.wvc` files via `pair_correction_stream` and decodes each
  paired member hybrid-lossless before the wiki bits-11..=12 set
  interleave; unpaired members fall back to the coarse member decode.
  The single-file walker now delegates to the pair walker with an
  empty correction chain (bit-identical behaviour). Pinned by a
  reference-encoded 16-bit 5.1 pair fixture
  (`foreign_hybrid_pair_5dot1`) reassembling the encoder input
  bit-exactly with every member's lossless CRC matching.

- Round 415 — **CRC-gated (muted) pair decode: the `.wvc` header
  stores the lossless decode's §5 CRC.** Black-box pin over the whole
  13-fixture pair battery: the correction block's header CRC is the §5
  mono/stereo running CRC folded over the **lossless** pre-fixup
  decode, exactly as the `.wv` header's covers the lossy one — so a
  two-file decode is integrity-checked end-to-end from the pair's own
  headers. New `WavPackBlock::decode_samples_with_correction_muted`
  and stream-level `decode_stream_with_correction_muted` apply the
  §5.6 mute gate against it (the `0x0C` extension-CRC verdict joins
  the gate when present; unpaired blocks fall back to the coarse
  `.wv`-header gate), zeroing a failing block instead of emitting
  wrong samples — pinned by a corruption trip-wire that flips one
  `0x0B` payload bit and by muted-twin parity across every pair
  fixture.

- Round 415 — **float / int32 hybrid-lossless pairs decode, bit-exact;
  int32 hybrid lossy decode gains the implied-zeros fill.** A pair
  encode carries the `0x0C` wvx extension stream in the `.wvc` twin
  alongside `0x07`/`0x0B` (structural pin against reference pair
  encodes), so `decode_samples_with_correction` now runs the
  sample-format fixups with the correction block's `0x0C` (falling
  back to the main block's own) instead of refusing `FLOAT_DATA` /
  `INT32_DATA` blocks: four new pair fixtures
  (`foreign_hybrid_pair_{float,float_js,int32,int32_js}` — mono +
  joint-stereo of each, the float stereo pair carrying exact ±0
  samples) reproduce the original encoder input bit-exactly, composing
  the joint-stereo shaping leg with the §2/§3 reconstruction.
  `decode_stream_with_correction_f32` is the typed float pair twin of
  `decode_stream_f32`. On the **lossy** side, decoding a sent-bits
  int32 hybrid `.wv` alone no longer errors with
  `BlockMissingOverflowBits`: the missing wvx window fills with
  implied zeros before redundancy re-insertion
  (`reassemble_int32_implied`, mirroring the round-408 float
  posture), pinned bit-exact against the reference lossy decode
  (`foreign_hybrid_int32_b4`, stored block CRC matching).

- Round 415 — **`CROSS_DECORR` hybrid pairs pinned decorative;
  cross-flagged blocks fold raw.** Reference maximum-compression
  hybrid pairs set flag bit 5 (`CROSS_DECORR`), whose staged-spec §4.1
  description is a *pre*-decorrelation fold placement — but three
  cross-flagged reference pair fixtures
  (`foreign_hybrid_pair_{js_cc,js_cc_s0,lr_cc}`: joint dynamic, joint
  unshaped, left/right dynamic) decode **bit-exactly under the
  post-decorrelation fold**, establishing black-box that the bit is
  decorative on current-version (`0x410`) files exactly as it is on
  lossless stereo blocks. `CorrectionFold::is_supported_raw_fold` now
  includes `PreDecorrelationCross`, so `fold_hybrid_correction` /
  `split_hybrid_correction` accept cross-flagged blocks; only the
  noise-shaped placement still refuses the raw add.

- Round 415 — **joint (mid/side) stereo hybrid-lossless pair decode,
  bit-exact against the encoder input.** The reference encoder's
  *default* stereo hybrid coding no longer refuses a pair decode: the
  raw `0x0B` correction fold stays in the coded (joint) domain — the
  §5.4 mid/side undo runs after it — while the `0x07` noise-shaping
  filter's two channels are **output** (left/right) channels running
  the exact recurrence the mono path pins. Per frame the coded-domain
  applied temps are `t_m = t_l - t_r` (mid) and
  `t_s = t_r + ((mid + t_m) >> 1) - (mid >> 1)` (side, computed on the
  **output-domain** mid — the decorrelated coarse plus the raw fold),
  each applied only to §6.5-bracketed samples, and the error states
  update against the post-undo left/right values with the *effective*
  per-output deltas (`d_r = t_s - ((mid + t_m) >> 1) + (mid >> 1)`,
  `d_l = t_m + d_r`; equal to `t_l` / `t_r` when both channels are
  bracketed — the divergence matters when a zero-run leaves one coded
  channel unbracketed). Because the side temp needs the decorrelated
  mid, the joint path decodes the unshaped bracket values first via
  the new `decode_packed_samples_stereo_hybrid_lossless_raw` (returns
  `(raw, coarse, bracketed)`), decorrelates, then applies the
  output-domain shaping. All wire behaviour pinned black-box against
  reference joint pairs: five new fixtures
  (`foreign_hybrid_pair_js{,_s0,_sp,_multi,24}`) covering dynamic /
  off / static-**positive** shaping (a weight regime the round-408 set
  never exercised — a mono positive-shaping probe confirmed the
  per-channel recurrence unchanged), 16/24-bit, and multi-block with a
  silence stretch, every one reproducing the original encoder input
  bit-exactly. The now-unreachable
  `Error::HybridJointCorrectionUnsupported` variant is removed.

- Round 408 — **hybrid-lossless (`.wv` + `.wvc`) pair decode — mono and
  left/right stereo, bit-exact against the encoder input.** New
  `WavPackBlock::decode_samples_with_correction` and the stream-level
  `decode_stream_with_correction` combine the coarse `0x0A` stream with
  the `0x0B` correction stream per spec §4.1, with the wire behaviour
  pinned black-box (round 408): after the §6.5 bracketing search
  narrows `[low, high]`, the **exact in-bracket offset** is read from
  the correction stream with the same phase-in binary code the lossless
  mantissa uses; the `0x07` `ID_SHAPING_WEIGHTS` sub-block (carried by
  the correction block) seeds the per-channel noise-shaping filter as
  **log-packed** `[error, acc, (delta)]` words (error negated on the
  wire; stereo interleaves per-channel pairs then deltas); each
  sample's `temp = -((weight*error + 511) >> 10)` with the
  negative-weight **unit-magnitude nudge** (`|temp|` capped strictly
  below `|error|`) folds into the exact value; the error state updates
  as `exact - coarse` under a negative weight and
  `exact - coarse - temp` under a non-negative one; and the
  decorrelation passes run on the **coarse** values with the
  correction difference folded in afterward (the §4.1
  post-decorrelation placement). Exposed as `ShapingState` +
  `decode_packed_samples_{mono,stereo}_hybrid_lossless`. Five pair
  fixtures (dynamic noise shaping, `-s0`, static `-s-0.7`, multi-block
  + silence, `-j0` stereo) reproduce the **original encoder input**
  bit-exactly. Joint-stereo (mid/side) pairs are a typed
  `Error::HybridJointCorrectionUnsupported` refusal — the
  correction/shaping interplay across the §5.4 transform is the
  remaining documented gap (as are float / int32 hybrid pairs).

- Round 408 — **hybrid (lossy) decode — the §6.5 `error_limit` model,
  bit-exact.** Hybrid blocks (flag bit 3) now decode to their coarse
  lossy PCM instead of the `UnsupportedBlockFeature::Hybrid` refusal.
  Per the staged `wavpack-entropy-decode.md` §6.5 structural model,
  with the exact integer arithmetic pinned **black-box** against
  reference decodes (round 408): the `0x06` `ID_HYBRID_PROFILE`
  sub-block seeds per-channel linear `slow_level` accumulators
  (log-packed level words via `wp_exp2s`, a shared log-domain bitrate
  word, and a stereo balance word); every decoded sample folds
  `slow_level -= (slow_level + 128) >> 8; slow_level +=
  wp_log2(magnitude)`; the per-sample `error_limit` is
  `wp_exp2s(ema - bitrate + 256)` (mono) with the stereo pair
  redistributed at frame start by `delta = (ema0 - ema1 - balance) >>
  1`; and the §4.2 step-6 mantissa read becomes the bracketing binary
  search (`mid = (low + high + 1) >> 1`, bit 1 → `low = mid`, else
  `high = mid - 1`, while `high - low > error_limit`; the final
  interval's midpoint is the coarse value, sign bit last). New public
  surface: `HybridProfile` / `expand_hybrid_profile`, `HybridState`
  (seed / `frame_limits` / `update_signed`),
  `SampleInterval::decode_bracketed_value`, and the
  `decode_packed_samples_{mono,stereo}_hybrid` loops; `decode_samples`
  and every stream/seek/multichannel path dispatch automatically. The
  stored block CRC covers the coarse samples, so the §5.6 mute gate
  applies unchanged. Hybrid float blocks (lossy float) reassemble with
  implied-zero extension fills (`reassemble_float_implied`) — the
  reference encoder keeps the `SHIFT_SENT` profile flag but omits the
  wvx stream in lossy mode. Nine reference-encoded hybrid fixtures
  (mono b2/b4, multi-block + silence b3, mid/side + balance stereo,
  `-j0` left/right, 5.1, 24-bit, float, false-stereo) decode
  bit-exact with all block CRCs green. A hybrid block without `0x06`
  raises the new `Error::BlockMissingHybridProfile`; a
  shape-mismatched payload raises `Error::HybridProfileLength`.

### Fixed

- Round 408 — **false-stereo blocks emit both output channels.** A
  false-stereo block (flag bit 30: stereo container, one coded
  channel) previously decoded to a single channel; the reference
  decoder duplicates the coded channel to both outputs (black-box
  pinned on an identical-L/R hybrid encode, which the reference
  encoder routinely folds into false-stereo blocks). `decode_samples`,
  the muted twins, the member paths, `decoded_sample_count`,
  multichannel assembly and the seek index now all treat false-stereo
  as two duplicated output channels.

### Added

- Round 408 — **float `SHIFT_SAME` + `EXCEPTIONS` decode — every
  documented `0x08` profile shape now reconstructs.** Per the staged
  `docs/audio/wavpack/spec/wavpack-sample-formats.md` §2.1–§2.2 (plus
  round-408 black-box pins against reference-encoded probe files):
  `SHIFT_SAME` reads a one-bit-per-non-zero-sample carrier from the
  `0x0C` extension stream (`1` fills the vacated low-mantissa window
  with ones, `0` with zeros; zero samples spend no bit), and
  `EXCEPTIONS` decodes ±infinity / NaN samples from the bit-length-25
  sentinel integer (`magnitude << float_shift == 1 << 24`, sign on the
  normal sign path) plus a `0x0C` marker bit (`0` = infinity, `1` =
  the 23-bit NaN mantissa payload LSB-first). `FloatInfo::is_supported`
  is now always `true`; the `FloatShiftSame` / `FloatExceptions`
  refusals are no longer raised (variants retained for API stability).
  An `EXCEPTIONS`-capable block without a `0x0C` sub-block is accepted
  until an exceptional sample actually needs payload bits. Seven new
  reference-encoded fixtures (`foreign_float_shift_same`,
  `foreign_float_shift_same_zeros`, `foreign_float_shift_ones`,
  `foreign_float_exceptions`, `foreign_float_exc_stereo`,
  `foreign_float_exc_tiny`, `foreign_float_exc_mix`) decode bit-exact
  with both CRC gates passing, including exact NaN payload bits.

### Fixed

- Round 408 — **short `0x03` weights sub-blocks decode (wire-order
  prefix rule).** Reference encoders emit `0x03` decorrelation-weights
  payloads shorter than the pass list (observed black-box: two weight
  bytes for a five-term stack); `assemble_mono_passes` /
  `assemble_stereo_passes` previously refused them with
  `DecorrelationWeightCountMismatch`. A short payload now primes a
  wire-order **prefix** of the passes and every later weight slot
  starts at zero (the spec §3.6 "unspecified passes start at 0"
  convention, mirroring the round-405 short-`0x04` seed-prefix rule);
  over-long payloads remain malformed.

### Added

- Round 405 — **`wp_log2` / `wp_exp2s` log-domain conversions —
  foreign-file decode unblocked.** New `logpack` module implementing
  the staged `docs/audio/wavpack/spec/wavpack-log2-exp2.md` integer
  log2/exp2 pair: `wp_log2` (magnitude → 8-fractional-bit log word,
  bit-length integer part, `avalue >> 9` interpolation bias),
  `wp_exp2s` (signed log word → value, odd function, implicit `0x100`
  mantissa bit, shift pivot 9), the 256-entry `LOG2_TABLE` /
  `EXP2_TABLE` mechanically transcribed from the staged
  `docs/audio/wavpack/tables/wp-log2.csv` / `wp-exp2.csv`, and the
  wire helpers `expand_log_word` / `pack_log_word` /
  `quantize_log_value`. Pinned to the spec's worked example
  (`1000 ↔ 2807`), the canonical-zero erratum (`0x0000 ↔ 0`, §6), the
  meta anchors, and the §1 round-trip bounds (exact below magnitude
  115, ~0.1% across the 32-bit range).

- Round 405 — **sample-rate surface (staged table + `0x27`).**
  `STANDARD_SAMPLE_RATES` (the 15-entry index table, mechanically
  transcribed from the staged `docs/audio/wavpack/tables/sample-rates.csv`),
  `sample_rate_index_for`, `Flags::standard_sample_rate`,
  `parse_non_standard_sample_rate` / `find_non_standard_sample_rate`
  (the 3-byte little-endian `0x27` carrier for the sentinel index 15),
  `WavPackBlock::sample_rate`, and the stream-level
  `stream_sample_rate`. Seeking gains its time-addressed layer:
  `StreamReader::sample_rate` + `StreamReader::seek_seconds`
  (typed `Error::SampleRateUnknown` when a custom-rate stream lacks its
  `0x27`). On the write side, `set_stream_sample_rate` stamps an
  encoded chain post-hoc (standard rates patch every header's bits
  23..=26; non-standard rates set the sentinel and append the `0x27`
  sub-block to the stream's first block, once), and the registry
  `WavPackEncoder` applies it automatically from the caller-declared
  `CodecParameters::sample_rate` — validated black-box: the reference
  decoder reports both a stamped standard rate (44100) and a stamped
  custom rate (12345) and decodes the stamped streams bit-exactly.

- Round 405 — **int32 (`INT32_DATA`) sample-format decode.** Blocks
  with flag bit 8 are decoded, not refused: the new `int32` module
  implements the staged `wavpack-sample-formats.md` §3 `0x09` profile
  (`Int32Info` / `expand_int32_info`: 4-byte `sent_bits, zeros, ones,
  dups`, mutually-exclusive redundancy enforced) and the §4 per-sample
  reassembly (`reassemble_int32`: `sent_bits` literal low bits from the
  `0x0C` extension bitstream LSB-first, redundancy pattern re-inserted
  below them). `PackedOverflowBits` gains the §4 structured accessors
  `crc_wvx` (the 4-byte little-endian stored extension CRC at the head
  of the `0x0C` payload) and `extension_bit_reader`; the §5.5 `crc_x`
  fold runs over every reassembled value and joins the §5.6 mute gate
  in `decode_samples_muted` / `decode_member_samples_muted` /
  `verify_decoded_crc` (a block is muted when *either* CRC fails). The
  fixup runs after the main-CRC fold and before the header left-shift,
  matching the staged pipeline. Typed refusals:
  `BlockMissingInt32Info`, `Int32InfoLength`, `Int32InfoConflict`,
  `BlockMissingOverflowBits`, `OverflowBitsTooShort`. Validated
  black-box against reference-encoded 32-bit files (default + `-h`,
  mono/stereo, sent-bits and trailing-zeros profiles): bit-exact PCM,
  matching `crc_x`, and an extension-bit corruption trip-wire pinned as
  integration fixtures.

- Round 405 — **float (`FLOAT_DATA`) sample-format decode.** Blocks
  with flag bit 7 are decoded, not refused: the new `float` module
  implements the staged `wavpack-sample-formats.md` §2 `0x08` profile
  (`FloatInfo` / `expand_float_info`) and the scaled-integer →
  IEEE-754 reconstruction — static `float_shift`, per-sample mantissa
  normalisation anchored on `float_max_exp`, vacated low bits filled
  as zeros / ones (`SHIFT_ONES`) / literal `0x0C` bits (`SHIFT_SENT`),
  and `ZEROS_SENT` zero samples (marker bit; literal
  mantissa23+exponent8+sign1 for sub-integer magnitudes incl.
  denormals; `NEG_ZEROS`-gated sign for true ±0) — the wire layouts
  the staged spec names but does not bit-pin were established
  black-box via differential probes against reference-encoded files.
  **Erratum found:** the float extension CRC does *not* use the
  `wavpack-decorrelation.md` §5.5 halfword formula (that form holds
  for int32 only); it folds three mono-CRC steps per sample —
  mantissa, exponent, sign (`update_float_extension`) — pinned by
  single-bit differential probes (weights `9<<k` / `3` / `1`).
  `SHIFT_SAME` and `EXCEPTIONS` (inf/NaN) profiles are refused with
  the new typed `UnsupportedBlockFeature::FloatShiftSame` /
  `FloatExceptions`. Typed f32 surface: `WavPackBlock::is_float` /
  `decode_samples_f32` and the stream-level `decode_stream_f32`
  (refusing integer streams via `Error::BlockNotFloat`). Validated
  black-box: integer-valued, full-precision, ±0/denormal/>1.0 and
  `-h`-mode float files decode **bit-exactly** with matching `crc_x`
  (6/6), five pinned as integration fixtures plus a float
  extension-corruption trip-wire.

- Round 405 — **`0x0D` first-member channel geometry.** Typed
  `ChannelInfo` (`count`, little-endian Microsoft speaker `mask`,
  `mask_bytes`, `assigned_positions`) with `parse_channel_info`
  implementing the staged `wavpack-sample-formats.md` §6 erratum pin
  (`[count, mask]`, zero-length mask = no assignment; the extended
  >32-channel form is a typed `ChannelInfoLength` refusal), plus
  `WavPackBlock::channel_info` and the stream-level
  `stream_channel_info`. Pinned against the reference-encoded 5.1
  fixture (`count 6`, mask `0x3F`, matching the decoded width).

### Changed

- Round 405 — **`0x05` median / `0x04` seed expansion is now the
  log-domain `wp_exp2s`** (was: the wiki's linear mantissa/exponent-9
  shorthand, which diverges from reference-encoded files for every
  non-zero word). `expand_samples` / `expand_entropy` read each
  little-endian 16-bit field as a signed log word;
  `pack_sample_word` / `quantize_seed_sample` are the matching
  log-domain forward inverses. The all-zero word keeps expanding to
  the exact zero, so streams this crate's own zero-seed encoder writes
  are unchanged.
- Round 405 — **`0x04` seed payloads prime a wire-order prefix of the
  decorrelation passes.** Real encoders store seeds for fewer passes
  than the `0x02` term list carries (commonly one term's worth for a
  five-term stack); the remaining passes now start from zero history —
  the same "unspecified passes start at 0" convention spec §3.6 states
  for weights — instead of the whole block being refused with a seed
  count mismatch. Mid-group truncation and seed surplus remain typed
  refusals. Net effect of the three round-405 changes, validated
  black-box against `wvunpack` 5.9 (opaque binary): arbitrary
  reference-encoded lossless files (default / `-f` / `-h` / `-hh` /
  `-hh -x4` modes, 8/16/24-bit, mono / stereo / 5.1, custom sample
  rates) now decode **bit-exactly** with the stored block CRC
  matching, where every non-zero-median file was previously mis-seeded.
- Round 372 — `encode_block_mono` / `encode_block_stereo` dropped their
  `passes: &[DecorrPass]` parameter (all added this same round, unreleased)
  and are now the raw (no-decorrelation) lossless path only. Decorrelated
  blocks go through the `*_with_decorr` payload entry points (the bit-exact
  verbatim-payload path); the staged `Error::NotImplemented` decorr branch
  is gone.

### Fixed

- Round 386 — **adversarial-history extrapolator overflow (fuzz
  finding)**. A malformed stream can seed a term-17/18 decorrelation
  pass with near-`i32`-extreme `0x04` history samples, making the
  spec §3.2 extrapolator predictors (`2*a0 - a1`, `(3*a0 - a1) >> 1`)
  overflow a non-wrapping multiply (debug-build panic). All twelve
  predictor sites across `decorrelate_mono` / `decorrelate_stereo` /
  `recorrelate_mono` / `recorrelate_stereo` now use 32-bit wrapping
  arithmetic, matching the wrapping reconstruction adds that already
  surrounded them — both directions use identical forms, so the
  forward/inverse identity is unchanged (asserted at the extremes in
  both channel shapes). The minimized input is pinned as corpus seed
  `regression_extrapolator_overflow.bin` + an `include_bytes!`
  regression test through `decode_stream` / `decode_stream_muted`.

### Fixed

- Round 393 — **reference-decoder conformance (wvunpack black-box
  cross-validation)**. A 13/13 fixture battery against `wvunpack` 5.9
  (opaque binary; no reference source consulted) now decodes every
  encoder output shape bit-exactly. Wire fixes: canonical all-zero
  log-word (`0x0000`) for zero medians/seeds (the stereo `0x05` is
  now all-zero, stereo-ness gated by its 12-byte length instead of
  the old `[0,0,1]` content marker); no run-length field after a
  completed zero-run (`DecodeState::run_break`, both directions);
  **stream-level** stereo holding state
  (`StereoDecodeState::{left_run,right_run}` merged into a single
  shared `run`, encoder carry-lookahead retargeted to the next stream
  sample); `max_magnitude` flag emission (bits 18..=22 — empirically
  required, not just a hint: reference decoders need ≥
  `bit_length(ones_count − 3)`); first-member `0x0D`
  `[channel_count, speaker_mask]` emission on multichannel sets.
  Decode-side: wiki bit 28 (robust, "okay to ignore") and bit 5
  `CROSS_DECORR` on lossless blocks are now ignored instead of
  refused — reference encoders set both on ordinary files; the
  `UnsupportedBlockFeature` variants remain for API stability. The
  staged §5 CRC formulas and §4.2/§3 arithmetic were positively
  confirmed against reference-encoded probe files. Remaining docs
  gap (precise): the `wp_log2`/`wp_exp2s` log-pack algorithm + table
  — foreign files' non-zero median/seed words cannot be expanded from
  the staged docs, so arbitrary-file decode stays blocked while
  zero-seed streams are fully bidirectional. Post-change campaigns:
  encode_roundtrip 715k / seek_surface 227k / decode_stream 1.27M
  execs, clean; two curated new-wire corpus seeds added.

### Added

- Round 393 — **`oxideav-core` registry wiring (dual API) + streaming
  multichannel encoder**. New `registry` module: `WavPackDecoder`
  (one `AudioFrame` per packet of complete `wvpk` blocks via
  `decode_multichannel_stream`, output bytes packed at the stream's
  container width — S8/S16/S24/S32 interleaved, values verbatim —
  NeedMore/Eof discipline, `reset` re-arms) and `WavPackEncoder`
  (interleaved signed PCM in, one packet per frame out with a running
  `block_index`, so concatenated packet payloads form one contiguous
  **seekable** `.wv` chain; mono / stereo use the
  `encode_block_*_best` mode search at `DecorrProfile::Normal`, wider
  layouts the mono-member grouping; float / planar / ragged-frame
  refusals). `register` installs the `CodecInfo` (decode + encode,
  lossless, staged-wiki `WVPK` FourCC tag) behind
  `oxideav_core::register!`, and the direct factories are exposed as
  `decoder::make_decoder` / `encoder::make_encoder` per the workspace
  dual-API convention. Enabler:
  **`encode_multichannel_stream_at`** — the offset-aware
  generalization of `encode_multichannel_stream` (now its `0/frames`
  delegate), taking a first-block frame offset and a caller-supplied
  `total_samples` word (`TOTAL_SAMPLES_UNKNOWN` for streaming
  producers) so incremental chunks concatenate contiguously. 12 new
  tests (registry encoder→decoder loops at 16/24-bit, cross-packet
  seekability, metadata-only packets, refusal arms) + 3 for the
  offset-aware encoder (byte-equality at zero offset, seekable
  two-chunk streaming vs. non-offset control, u32 overflow refusal).

- Round 393 — **seeking / block-index subsystem
  (`StreamIndex` / `decode_range` / `StreamReader`)**. Sample-accurate
  random access built on the wiki "Block structure" header fields
  alone:

  - **`StreamIndex::scan`** — a header-only O(blocks) pass mapping
    every block's byte span (`IndexEntry`: offset, `8 + ck_size`
    length, `block_index`, `block_samples`, decoded flags) and
    grouping audio blocks into member sets (`SetEntry`) under the
    same wiki bits-11..=12 rules — and the same typed refusals —
    `decode_multichannel_stream` applies, so every stream the decoder
    accepts can be indexed without decoding a sample. Introspection:
    block/audio/set counts, per-frame `channels`,
    `first_frame`/`end_frame`/`frame_count`, `is_seekable`
    (contiguous ascending set chain), `locate_frame`/`set_for_frame`
    binary search in the absolute frame domain, and `set_byte_span`
    for ranged partial-file IO.
  - **`decode_range` / `decode_range_muted`** — decode an arbitrary
    absolute frame window, touching only the sets it overlaps; the
    output is bit-exactly the same window sliced from the
    whole-stream decode (asserted for mono / stereo / joint /
    left-shifted / multichannel shapes and swept window boundaries).
    The muted twin applies the spec §5.6 per-member CRC gate with a
    window-scoped `all_crc_ok`. New typed refusals:
    `Error::StreamNotSeekable` (gapped / overlapping / regressing
    frame chains) and `Error::SeekOutOfRange` (with the covered
    `[first_frame, end_frame)` carried).
  - **`StreamReader`** — a playback-shaped cursor (`seek` /
    `read_frames` / `read_frames_muted` / `position` /
    `frames_remaining` / `is_at_end`) that decodes whole sets and
    caches the most recent one with its decode mode, so sequential
    small reads decode each set once, seek-back within the cached set
    is free, and a plain buffer never serves a muted read (nor a
    corrupt muted buffer a plain one — cross-mode reuse only when the
    CRC verdict was clean, where the two modes are bit-identical by
    construction). Failed reads are all-or-nothing: no frames
    returned, cursor restored.
  - **`seek_surface` fuzz target + five corpus seeds** — differential
    campaign asserting scan-never-stricter-than-decode, buffer
    tiling, set/locate invariants, walker agreement, and full-span +
    fuzz-chosen-window + chunked-reader equality against the
    whole-stream decoders (incl. muted PCM + verdict parity).
    793,827 execs / 7 min at `-rss_limit_mb=4096`, clean.

  41 new unit tests across the four milestones.

- Round 386 — **introspection-surface fuzz target + round-386 corpus
  seeds + four clean campaigns**. New
  `fuzz/fuzz_targets/introspection_surface.rs` drives arbitrary bytes
  through every non-decoding stream walker (block/audio/metadata/
  correction counts, sample-count walkers, layout, first-block peeks,
  correction-payload sizing) plus the new `.wvc` pairing walker at a
  fuzz-chosen split, asserting the cross-walker invariants (audio +
  metadata partition the chain; correction-bearing ⊆ all;
  per-channel-sum ≤ PCM-value-count ≤ ×2; paired ≤ total). Six new
  corpus seeds cover the round-386 encode shapes (Extra 16-pass block,
  searched stereo block, 3-block smallest stream, three pairing
  splits). Campaigns this round: introspection 15.7M execs,
  decode_stream 710k (post-fix), multichannel 2.07M,
  encode_roundtrip 658k — all clean at the documented
  `-rss_limit_mb` sizing.

- Round 386 — **`.wvc` correction-file pairing plumbing
  (`pair_correction_stream` / `correction_coverage` /
  `WavPackBlock::expects_correction`)**. Stream-level alignment of a
  main `.wv` buffer with its companion `.wvc` buffer: audio blocks are
  paired by the wiki "offset in samples for current block" header
  word, with per-pair agreement enforced on `block_samples`
  (`Error::CorrectionSampleCountMismatch`) and the mono flag
  (`Error::CorrectionShapeMismatch`); orphaned
  (`Error::CorrectionIndexMismatch`) and surplus
  (`Error::CorrectionBlockSurplus`) correction blocks are typed
  refusals, partial coverage pairs `None`, and metadata-only blocks on
  either side are skipped. `expects_correction` classifies which
  blocks *want* a twin (wiki bit-3 hybrid flag) so lossless-path
  callers can validate and size correction coverage before any hybrid
  decode exists — consuming the paired `0x0B` words stays gated on
  `UnsupportedBlockFeature::Hybrid`.

- Round 386 — **union "smallest" encoders (`encode_block_*_smallest` +
  `encode_stream_*_smallest`)**. One call races the two search
  families this crate now has — the Extra-ceiling profile grid
  (raw + eight curated derived stacks, ×2 joint modes on stereo) and
  the greedy term search at the 16-pass cap — and keeps the smaller
  block; the stream twins run the union per window. The two families
  explore different stack spaces (curated profiles vs. signal-driven
  picks), so the union dominates both by construction (asserted at
  block and stream level).

- Round 386 — **greedy term-search encoder mode
  (`derive_*_passes_searched` + `encode_block_*_searched`)**. Instead
  of a fixed profile stack, each pass's term is picked greedily from
  the full spec §2 valid set (`1..8`, `17`, `18`, plus the §3.3 cross
  terms on stereo) by measuring which candidate most reduces a
  residual magnitude-bits cost proxy over the domain the previous
  picks produced; the search stops early when nothing strictly
  improves (so dead passes are never emitted) and clamps at the
  `MAX_NTERMS` = 16 cap. The picked stack is re-trained with two
  iterated sweeps so the stored starting weights match the exact
  composition the decoder runs. The block entry points race the
  searched stack against the raw candidate ({plain, joint} × {raw,
  searched} on stereo) at the auto-detected left-shift, so a searched
  encode never loses to raw; an empty pick list (e.g. constant zero
  input) falls back to raw cleanly.

- Round 386 — **iterated weight training
  (`derive_mono_passes_iterated` / `derive_stereo_passes_iterated`)**.
  Each extra sweep re-trains the §3.4 adaptation starting from the
  previous sweep's quantized `0x03` weights, walking the stored
  starting weights toward the block's own `quantize ∘ train` fixpoint
  so the early samples are predicted well immediately instead of
  adapting from zero. One sweep reproduces the round-383 derivation
  exactly (asserted); the term/delta stack is sweep-invariant — only
  the starting weights refine. The `*_best` mode searches now push a
  twice-iterated candidate per profile alongside the single-sweep one,
  so a best encode can only get smaller.

- Round 386 — **`DecorrProfile::Extra` — the spec §2.1 `MAX_NTERMS`
  (16-pass) derivation ceiling**. The new profile derives sixteen
  decorrelation passes (every fixed lag `1..8`, repeated extrapolators,
  plus `-1`/`-3` cross passes on stereo) — the deepest pass list a
  conformant block can carry, since the `0x02` reader caps the count
  at 16. `search_set()` nests it above `High`, so an `Extra` ceiling
  in the `*_best` encoders tries all four stacks and keeps the
  smallest block; the encoder round-trip fuzz target's control byte
  now maps its fourth profile value onto it.

- Round 383 — **encoder round-trip fuzz target + corpus seeds from the
  new block shapes**. New `fuzz/fuzz_targets/encode_roundtrip.rs`
  carries a *round-trip oracle* (strictly stronger than the decode
  targets' panic-freedom contract): fuzz bytes become a control header
  (mono/stereo shape, search-ceiling profile, synthetic left-shift
  scale) plus an `i32` PCM buffer, encoded through the `*_best` mode
  search and asserted `decode_stream(&encoded)? == pcm` bit-exactly
  plus the §5.6 CRC gate — any divergence in derive → serialize →
  recorrelate → entropy → decode fails the run. A ~1.8M-exec campaign
  is clean. Two new named decode-corpus seeds cover the round-383
  block shapes (shifted+decorrelated mono; joint+decorrelated stereo).
  A campaign with those seeds also surfaced that the eager
  `decode_stream`'s *inherent* format amplification (a ~50-byte
  zero-run block legitimately decodes to up to `1 << 26` zero samples —
  silence compresses enormously; verified: a 50-byte encoded block of
  `2^24` zeros decodes to 64 MiB) trips libFuzzer's default 2 GiB RSS
  accounting; the decode target now documents the required
  `-rss_limit_mb` sizing and points hard-memory-bound callers at the
  lazy per-block iterator. A re-run at the documented limit is clean
  (0 oom/timeout/crash).

- Round 383 — **profile-ceiling mode search in the best encoders**.
  `DecorrProfile::search_set` names the nested effort ladder
  (`Fast ⊂ Normal ⊂ High` as candidate sets), and the `*_best` encoders
  now treat their `profile` argument as a search **ceiling**: one
  derived-decorrelation candidate per profile in the set is tried (per
  joint arm on stereo), alongside the raw candidate(s). Which term
  stack yields the smallest block is signal-dependent — on the
  measurement signal the 2-pass Fast stack beats the 5-pass Normal
  stack — so a deeper ceiling always tries the cheaper stacks too and
  the minimum is monotone in the ceiling. Re-measured stream-best
  ratios at the High ceiling: mono ~51% of raw-stream bytes (42% of
  16-bit PCM), correlated stereo ~36% of raw (30% of PCM, down from 46%
  of raw with the single-profile search), 12-bit-in-16 material ~26% of
  raw (down from 30%). 3 new tests (881 total): the nesting pin, minimum
  monotonicity in the ceiling (mono + stereo), and best-dominates-every-
  auto-in-ceiling. Exported: `DecorrProfile::search_set`.

- Round 383 — **stream-level best encode**. `encode_stream_mono_best` /
  `encode_stream_stereo_best` lift the per-block best-of search to the
  whole-file surface: each chunk gets its own left-shift detection, its
  own trained decorrelation pass list, and its own mode-grid size
  decision (stereo: {plain, joint} × {raw, decorr}), so a file whose
  character changes over time picks the best mode per block
  independently. Chunking / header contract matches the raw stream
  encoders (`DEFAULT_BLOCK_SAMPLES` fallback, running `block_index`,
  file-global total); `decode_stream(&out)? == pcm` exactly. Measured
  on a synthetic musical signal (44100 samples, triangle mix + small
  noise): the best stream is ~51% of the raw-stream bytes for mono
  (Fast profile), ~46% for correlated stereo, and ~30% for
  12-bit-in-16-container material — ~47% of the raw 16-bit PCM byte
  count. 5 new tests (878 total): mono + stereo multi-block round-trip
  + smaller-than-raw pins with the CRC gate, mixed-material per-block
  independence, the block_index/total header contract, and the
  empty/odd edge arms. Exported: `encode_stream_mono_best`,
  `encode_stream_stereo_best`.

- Round 383 — **left-shift auto-detection + best-of block mode
  selection**. `detect_left_shift` reports the sub-byte-depth shift a
  PCM buffer announces itself with (the common low-zero-bit count across
  all samples, capped at the 5-bit flag field; `0` for all-zero or
  full-depth audio) — so 12-/20-bit-in-container material no longer
  needs the caller to know its own scaling. `encode_block_mono_best` /
  `encode_block_stereo_best` then search this encoder's whole mode grid
  at the detected shift — mono: {raw, derived decorrelation}; stereo:
  {plain, joint mid/side} × {raw, derived decorrelation} — with each
  decorrelated candidate trained over the exact domain its prediction
  loop runs in (narrow, or narrow + joint), keeping the smallest
  output. Every candidate decodes back to the input bit-exactly, so the
  selection is size-only, never correctness. Pinned: best never loses
  to any public single-mode encoder; the shift-aware best beats the
  unshifted auto encoder on 12-bit-style material; a shifted
  identical-channel buffer combines all three features (joint + decorr
  + shift flag) in one block and passes the CRC gate. 6 new tests (873
  total). Exported: `detect_left_shift`, `encode_block_mono_best`,
  `encode_block_stereo_best`.

- Round 383 — **joint (mid/side) stereo + decorrelation combined
  encode** and the single-body encoder core. Previously joint stereo
  and decorrelation were mutually exclusive encode paths; the new
  `encode_block_stereo_joint_with_decorr` (payload-driven) and
  `encode_block_stereo_joint_auto` (self-derived) combine them,
  mirroring the decoder's stage order — the §5 CRC folds over the true
  L/R, the forward §5.4 mid/side transform runs next, and the §3
  forward prediction loop runs over the *joint-transformed* buffer (the
  decoder decorrelates first, then undoes joint). The joint auto path
  trains its pass list over a joint-transformed scratch copy so the
  derivation sees the same domain the real loop will. Internally every
  public block encoder now delegates to one `encode_block_core` +
  `BlockConfig` body (mono/joint/left-shift/decorr/marker as orthogonal
  axes), replacing five hand-expanded stage pipelines — byte-identical
  output, pinned by the entire existing encode suite passing unchanged.
  On identical channels the joint auto block is pinned strictly smaller
  than the plain auto block (the mid channel collapses to the zero-run
  fast path). 5 new tests (867 total): combined-feature round-trip with
  flag + sub-block-chain checks, all-profile joint-auto round-trips,
  the joint-beats-plain compression pin, pseudo-random safety, and the
  refusal arms. Exported: `encode_block_stereo_joint_with_decorr`,
  `encode_block_stereo_joint_auto`.

- Round 383 — **self-deriving decorrelation encoder** (`*_auto`). The
  first entry points that perform real prediction-based compression
  without the caller authoring any metadata: `encode_block_mono_auto` /
  `encode_block_stereo_auto` take only PCM and a `DecorrProfile`
  (`Fast` = 2 extrapolate passes, `Normal` = 5 mixed passes + a stereo
  zero-delay cross pass, `High` = 8 passes over a wider lag spread + a
  mutual cross pass — this encoder's own choices among the spec §2
  valid term set). The derivation (`derive_mono_passes` /
  `derive_stereo_passes`) is a two-step bootstrap: a zero-state
  **training pass** runs the §3 forward prediction loop over a scratch
  copy so the §3.4 `±delta` adaptation walks each weight toward the
  block's actual correlation, then the trained weights are quantized to
  their `0x03` stored-byte values and fresh zero-seed passes are
  rebuilt — serializable by construction, so the auto path composes
  `derive → serialize → encode_block_*_with_decorr` and inherits the
  bit-exact lossless guarantee (`decode_stream(&out)? == pcm`)
  regardless of how well the training matched the signal. On a smooth
  test signal the Normal-profile block is pinned smaller than the raw
  (no-decorrelation) block for both mono and stereo. 7 new tests (862
  total): all-profile mono + stereo round-trips (with CRC gate),
  pseudo-random-input safety, the compression assertions, trained-and-
  quantized weight checks (ramp drives the extrapolate weight up;
  every derived weight is its own quantization), stereo both-channel /
  cross-pass coverage, and the shared refusal arms. Exported:
  `encode_block_mono_auto`, `encode_block_stereo_auto`,
  `derive_mono_passes`, `derive_stereo_passes`, `DecorrProfile`.

- Round 383 — **forward decorrelation-metadata serializers** (the exact
  inverses of the round-339/348 assemblers). `serialize_mono_passes` /
  `serialize_stereo_passes` turn an application-ordered `DecorrPass` list
  into the three raw `0x02` (terms) / `0x03` (weights) / `0x04` (seed
  samples) payload byte vectors, applying the spec §3.7 reverse-storage
  convention (wire stores the encoder's last-applied pass first), the
  §2.1 `+5`-biased term byte with the 3-bit delta field
  (`encode_term_byte`, the exact inverse of `decode_term_byte`), one /
  two weight bytes per pass, and the per-term per-channel seed
  partition: `assemble_*_passes(&serialize_*_passes(passes)?)? ==
  passes`, bit-exact. The on-wire stores are lossy log-packs, so the
  supporting quantizers are public: `pack_weight_byte` /
  `quantize_weight` (nearest-value inverse of the §3.6 weight expansion,
  pinned as a true nearest quantizer against exhaustive search and
  byte-exact round-trip over all 256 stored bytes) and
  `pack_sample_word` / `quantize_seed_sample` (canonical
  minimal-exponent inverse of the wiki 16-bit exponent/mantissa
  log-word). Passes carrying state the wire cannot reproduce are refused
  with the new typed errors `EncodeWeightNotRepresentable` /
  `EncodeSeedNotRepresentable` / `EncodeDeltaOutOfRange` (plus the
  assembler's existing cross-on-mono / over-long gates), so a serialized
  block always decodes back to the identical pass state. 14 new tests
  (855 total): exhaustive weight-byte round-trip + nearest-quantizer
  sweep, seed-word canonical/truncation arms, term-byte inverse across
  every valid `(term, delta)`, serialize→assemble identities (mono +
  stereo with cross terms), assemble→serialize byte identity on
  canonical payloads, every refusal arm, and a serialized-pass-list
  recorrelate→decorrelate round trip. Exported: `serialize_mono_passes`,
  `serialize_stereo_passes`, `encode_term_byte`, `pack_weight_byte`,
  `quantize_weight`, `pack_sample_word`, `quantize_seed_sample`.

- Round 378 — **multichannel layout introspection**. `multichannel_layout`
  reports a stream's per-frame channel count and member-set count by
  walking block headers (and the per-block mono/stereo flag) only — no
  entropy decode — returning a `MultichannelLayout { channels, sets }`. It
  enforces the same wiki bits-11..=12 grouping rules
  `decode_multichannel_stream` does, so a stream that passes layout is
  structurally decodable, and its `channels` agrees with the decoded
  count. Lets a caller size buffers and route channels before paying for a
  full decode. Exported: `multichannel_layout`, `MultichannelLayout`.

- Round 378 — **multichannel CRC-muted decode**.
  `decode_multichannel_stream_muted` is the multichannel twin of
  `decode_stream_muted`: each member block is gated by its own §5 running
  CRC and, on a mismatch, that member's channels are muted (zeroed) while
  the set's other members still contribute — so the interleaved frame width
  is unchanged and only the corrupt member's channel slots go to zero.
  Returns `(DecodedStream, all_crc_ok)`. Backed by the new member-level
  `WavPackBlock::decode_member_samples_muted`. Exported:
  `decode_multichannel_stream_muted`,
  `WavPackBlock::decode_member_samples_muted`.

- Round 378 — **multichannel stream encode**. `encode_multichannel_stream`
  is the bit-exact inverse of `decode_multichannel_stream`: it takes an
  interleaved multichannel PCM buffer plus a channel count and emits a
  `.wv` byte stream where each channel is a mono member block carrying the
  wiki bits-11..=12 grouping markers (first channel opens the set, last
  closes it, middle channels continue), split across successive sets of
  `block_samples` frames. `decode_multichannel_stream(&encode_multichannel_stream(pcm,
  channels, …)?)?.samples == pcm` for any channel count `1..=256` and any
  frame count, single- or multi-set. A single channel degenerates to a
  plain standalone mono file. Exported: `encode_multichannel_stream`.

- Round 378 — **multichannel grouping decode**. A WavPack stream carrying
  more than two channels splits each frame range across a *set* of member
  blocks (wiki bits 11..=12: a first-block member opens the set, a
  final-block member closes it, continuation members sit between). Each
  member is an ordinary 1-channel (mono / false-stereo) or 2-channel
  (stereo) block and decodes through the same lossless path standalone
  blocks use — the grouping marker is a stream-shape signal, not a
  decode-arithmetic one. New `decode_multichannel_stream` walks the member
  blocks, decodes each via the new `WavPackBlock::decode_member_samples`
  (which accepts the grouping marker instead of refusing it as
  `MultichannelMember`), and interleaves the set's channels per frame into
  a `DecodedStream { samples, channels }`. Standalone mono / stereo files
  decode identically to `decode_stream` (with `channels` reported as 1 /
  2); malformed grouping (stray final marker, unterminated set, per-member
  `block_samples` disagreement, channel-count blowup) is refused with the
  new typed errors `MultichannelSetMalformed` /
  `MultichannelSampleCountMismatch` / `MultichannelTooManyChannels`.
  Exported: `decode_multichannel_stream`, `DecodedStream`,
  `WavPackBlock::decode_member_samples`, `MAX_MULTICHANNEL_CHANNELS`.

- Round 372 — **sub-byte bit-depth (left-shift) block encode**.
  `encode_block_mono_shifted` / `encode_block_stereo_shifted` encode audio
  whose bit-depth is not a whole number of bytes (12-bit, 20-bit, …):
  the encoder right-shifts each container-scaled sample by `left_shift`
  (the inverse of the decoder's §1-pipeline final
  `apply_left_shift_buffer`), folds the §5 CRC over the narrow values,
  entropy-codes them, and sets the wiki flag-bits-13..=17 `left_shift`
  field — so the decoder reconstructs `narrow << left_shift` and recovers
  the input exactly: `decode_stream(&out)? == pcm`. Inputs must be a
  multiple of `2^left_shift` (genuine sub-byte audio); a lossy low bit is
  refused (`EncodeLeftShiftLosesData`), as is `left_shift == 0`
  (`EncodeLeftShiftZero`). New exports: `encode_block_mono_shifted`,
  `encode_block_stereo_shifted`. 4 new tests (811 total): 12-bit mono +
  20-bit stereo round-trip with the flag check, and the zero-shift /
  lossy-low-bit refusals.

- Round 372 — **joint (mid/side) stereo block encode**.
  `encode_block_stereo_joint` applies the forward mid/side transform
  (`mid = L - R; side = R + (mid >> 1)`, the exact inverse of the decoder's
  spec §5.4 `undo_joint_stereo`) per `(L, R)` pair before entropy coding
  and sets the joint-stereo flag (bit 4). The §5.4 `mid >> 1` truncation
  cancels between the forward and inverse transforms, so the block is
  bit-exactly lossless: `decode_stream(&out)? == pcm`. The §5 CRC is folded
  over the true L/R (the decoder undoes joint stereo before the CRC step).
  New export: `encode_block_stereo_joint`. 5 new tests (807 total): the
  forward/inverse transform identity over a wide pair range, plain +
  correlated-channel round-trips, the flag + CRC-gate check, and the
  empty/odd refusals.

- Round 372 — **lossless-with-decorrelation block encode**.
  `encode_block_mono_with_decorr` / `encode_block_stereo_with_decorr` take
  the raw `0x02` (terms) / `0x03` (weights) / `0x04` (seed samples)
  metadata payloads, assemble + validate the application-ordered pass list
  (`assemble_*_passes`), run the §3 forward prediction loop
  (`recorrelate_*`) to turn the PCM into residuals, and emit the three
  decorrelation sub-blocks **verbatim** ahead of the `0x0A` packed
  residuals. Emitting the payloads byte-for-byte makes the round trip
  bit-exact by construction (the decoder reads back the identical bytes
  and reconstructs the original PCM) without re-deriving log-packed
  weight/seed bytes from working pass state: `decode_stream(&out)? == pcm`
  for fixed-lag (`1..8`), extrapolate (`17`/`18`) and stereo cross
  (`-1`/`-2`/`-3`) terms, single- and multi-pass. An invalid term / weight
  count / seed count is surfaced verbatim from the assembler. New exports:
  `encode_block_mono_with_decorr`, `encode_block_stereo_with_decorr`. 7 new
  tests (802 total): single-pass / multi-pass / extrapolate mono, stereo
  fixed-lag + cross term, the 0x05/0x02/0x03/0x04/0x0A sub-block ordering,
  and the invalid-term refusal.

- Round 372 — **multi-block `.wv` stream encode**. `encode_stream_mono` /
  `encode_stream_stereo` split a long PCM buffer into a chain of `wvpk`
  blocks (default `DEFAULT_BLOCK_SAMPLES = 22050` per-channel samples per
  block, caller-overridable), each with its `block_index` set to the
  running per-channel sample offset and the file-global `total_samples`,
  so the chain is a well-formed standalone file the stream walker decodes
  back exactly: `decode_stream(&encode_stream_mono(pcm, …)?)? == pcm`. An
  empty buffer yields an empty stream (a no-audio file); a `block_samples`
  of `0` falls back to the default chunk. New exports:
  `encode_stream_mono`, `encode_stream_stereo`, `DEFAULT_BLOCK_SAMPLES`. 6
  new tests (795 total): mono + stereo multi-block round-trip over
  pseudo-random buffers, block_index advancement, empty-stream, default-
  chunk fallback, and the odd-length refusal arm.

- Round 372 — **first complete encode → decode lossless round-trip**: the
  `encode` module assembles a whole `wvpk` block from a PCM buffer.

  `encode_block_mono(pcm, passes, bytes_per_sample, block_index,
  total_samples)` and `encode_block_stereo(...)` frame the existing leaf
  encoders (the §4.2 modified-Rice `encode_packed_samples_*` entropy writer
  and the §3 `recorrelate_*` forward-prediction loop) into the wire byte
  layout — a 32-byte fixed header (spec §5 running CRC folded over the PCM,
  flags reconstructed from the block shape, version `0x0410`, standalone
  multichannel marker) followed by the `0x05` entropy-info and `0x0A`
  packed-samples metadata sub-blocks. The headline guarantee:
  `decode_stream(&encode_block_mono(pcm, &[], …)?)? == pcm` (and the stereo
  twin) — an encoded block parses, passes its own CRC mute gate, and
  reconstructs the exact input PCM. Covers the raw-residual (no
  decorrelation) lossless path for mono / false-stereo and plain
  (non-joint) stereo blocks; the forward-decorrelation `0x02`/`0x03`/`0x04`
  metadata serializer (`append_decorr_metadata`) is staged and refuses a
  non-empty pass list with `Error::NotImplemented`. New exports:
  `encode_block_mono`, `encode_block_stereo`, `ENCODE_VERSION`. New error
  arms: `EncodeEmptyAudio`, `EncodeStereoOddLength`, `EncodeBlockTooLarge`.
  12 new tests (789 total, up from 777): mono + stereo round-trip
  (including 500/600-sample pseudo-random buffers + a zero-run-heavy
  buffer), CRC-gate pass, header parse round-trip, sub-block ordering,
  unknown-total preservation, and the empty / odd-length / not-yet-wired
  refusal arms.

- Round 367 — forward (encode) block-level hybrid correction split.

  `WavPackBlock::split_hybrid_correction(original, lossy)` is the exact
  forward inverse of `fold_hybrid_correction`: it computes the per-sample
  correction residual `correction = original - lossy` (via
  `split_correction`) an encoder packs into the `0x0B` stream, so
  `fold_hybrid_correction(lossy, split_hybrid_correction(original, lossy))
  == original`. Like the decode-side fold it covers the plain
  post-decorrelation (spec §4.1) placement only — refusing the
  `CROSS_DECORR` / noise-shaped placements
  (`Error::HybridFoldPlacementUnsupported`) and a length mismatch
  (`Error::HybridCorrectionLengthMismatch`). 2 new tests (777 total, up
  from 775): the split-then-fold forward-inverse round trip recovering the
  original PCM, and the cross / shaped / length-mismatch refusal arms.

- Round 367 — block-level hybrid correction fold + fold-placement selector.

  `CorrectionFold` (in `hybrid`) names the three decorrelation-spec §4.1
  fold placements — `PostDecorrelation` (the mono / non-`CROSS_DECORR`
  stereo raw add), `PreDecorrelationCross` (the `CROSS_DECORR` `0x20`
  zero-delay fold before the decorrelation passes), and `NoiseShaped` (the
  `HYBRID_SHAPE` / `NEW_SHAPING` error-feedback filter) — and
  `CorrectionFold::from_flags` selects the placement from a block's 32-bit
  flag word (shaping wins, then cross, then the default post-decorrelation
  add). `is_supported_raw_fold` reports whether the placement is the plain
  raw add this crate applies end-to-end.

  `WavPackBlock::hybrid_correction_placement` surfaces that selector at the
  block level, and `WavPackBlock::fold_hybrid_correction(lossy, correction)`
  applies the §4.1 post-decorrelation fold element-wise — recovering
  lossless PCM from a reconstructed lossy buffer plus a matching
  correction-residual buffer (one residual per decoded sample). It refuses
  the `CROSS_DECORR` / noise-shaped placements
  (`Error::HybridFoldPlacementUnsupported`) and a length mismatch
  (`Error::HybridCorrectionLengthMismatch`), the two new error variants.
  It is a pure arithmetic consumer: it does not decode either entropy
  stream (the lossy main stream's `error_limit`-driven decode stays a
  documented gap), letting a caller that has both buffers recover lossless
  samples in one call.

  11 new tests (775 total, up from 764): the placement selector across the
  plain / cross / shaped flag words (shaping-over-cross precedence), the
  block-level placement accessor, the element-wise fold recovering lossless
  PCM, the zero-correction identity, and the cross / shaped / length-mismatch
  refusal arms.

- Round 367 — hybrid-mode correction-fold arithmetic (`hybrid` module),
  driving the hybrid (lossy main + `.wvc` correction) milestone to the
  documented wall.

  WavPack hybrid mode (flag bit 3, `HYBRID_FLAG` `0x08`) splits the signal
  into a lossy main stream (`0x0A`) plus an optional correction stream
  (`0x0B`); when the correction stream is present the decoder recovers the
  exact original by folding a correction residual into the reconstructed
  lossy value. The new `hybrid` module lifts that documented fold
  (decorrelation-spec §4.1) onto the public typed surface, the same way the
  §3 weight arithmetic, the §4.2 entropy ladder and the §5 CRC steps were
  lifted: a pinned, exact building block the consuming decode path drives
  when the remaining gaps close.

  - `fold_correction(reconstructed, correction)` — the spec §4.1
    post-decorrelation fold `original = reconstructed + correction` (the
    spec's `read_word += correction[0]`), used for mono and for a stereo
    block *without* `CROSS_DECORR`. `fold_correction_pair` applies it
    per-channel to a reconstructed `(L, R)` pair.
  - `fold_correction_pre_decorrelation(lossy, correction)` /
    `fold_correction_pre_decorrelation_pair` — the spec §4.1
    `CROSS_DECORR` (`0x20`) *pre*-decorrelation fold (`input = lossy +
    correction`, the "no-delay" correction folded before the decorrelation
    passes). Arithmetically the same add; the distinct typed name marks the
    pipeline position so a consumer cannot fold a correction in the wrong
    stage for a given `CROSS_DECORR` setting.
  - `split_correction(original, lossy)` — the exact encode inverse
    (`correction = original - lossy`), so
    `fold_correction(lossy, split_correction(original, lossy)) ==
    original`.
  - `flags_select_shaping(flags)` — detects the `HYBRID_SHAPE` (`0x40`) /
    `NEW_SHAPING` (`0x2000_0000`) noise-shaped fold, whose
    `read_shaping_info` state layout is a documented gap; the raw-add fold
    is correct only when this returns `false`. New `HYBRID_FLAG`,
    `CROSS_DECORR_FLAG`, `HYBRID_SHAPE_FLAG`, `NEW_SHAPING_FLAG` constants
    name the §6 flag bits.

  All folds use 32-bit wrap-around (`wrapping_add` / `wrapping_sub`),
  matching the canonical decoder's register arithmetic. 13 new tests (764
  total, up from 751) pin the fold-∘-split lossless-recovery identity over
  a value sweep and at the `i32` extremes (where the intermediate
  correction wraps), the plain-add / plain-subtract arithmetic, the
  zero-correction identity, the pre-/post-decorrelation arithmetic parity,
  the per-channel pair folds, and the shaping-bit detection.

- Round 360 — public forward (encode) decorrelation: `recorrelate_mono`
  / `recorrelate_stereo`, the exact arithmetic inverse of
  `decorrelate_mono` / `decorrelate_stereo`.

  The decorrelation decode loop (residuals → PCM) was public, but the
  forward direction (PCM → residuals) existed only as private
  single-pass test helpers (`forward_mono` / `forward_stereo`). The new
  `recorrelate_mono` / `recorrelate_stereo` lift that arithmetic to the
  public, multi-pass `&mut [DecorrPass]` surface, completing the encode
  side of the decorrelation-spec §3 pipeline. For each sample they form
  the same predictor from the same history the decoder reads and emit
  `residual = sample - apply_weight(weight, pred)` (the inverse of the
  decoder's `sample = apply_weight(weight, pred) + residual`), push the
  original PCM sample into history exactly as the decoder pushes its
  reconstructed sample, and nudge the weight with the identical
  `update_weight` / `update_weight_clip` step on `(pred, residual)` — so
  the two directions evolve byte-identical pass state. Per spec §3.7 the
  decoder undoes the encoder's passes in reverse, so both directions
  accept the *same* application-ordered pass list and the encoder walks
  it back-to-front internally (no caller-side reversal). Per-channel
  terms (`1..8`/`17`/`18`) invert §3.2, cross terms (`-1`/`-2`/`-3`)
  invert the §3.3 zero-delay step with the clipped weight update;
  `recorrelate_mono` rejects cross terms (`Error::CrossTermOnMono`) and
  both reject over-long lists (`Error::TooManyDecorrelationPasses`).
  Pinned by 12 new tests (751 total, up from 739): single-pass and
  multi-pass `recorrelate ∘ decorrelate` round-trips over a *shared*
  application-ordered pass list reproduce the original PCM for every
  fixed-lag / extrapolate / cross term and a mixed stack; the public
  encoders match the private `forward_*` helpers bit-for-bit; empty-pass
  identity; the stereo trailing-odd-sample passthrough; and the
  cross-term / over-long-list refusal arms.

- Round 354 — left-shift final-normalization fixup (`fixup` module).

  New `fixup` module implements the wiki flag-bits-13..=17 "left-shift
  places when bitdepth is not a multiple of 8 (e.g. 12-bit, 20-bit)"
  final-normalization stage: `apply_left_shift(sample, left_shift)` =
  `sample << left_shift` (the identity for whole-byte depths where
  `left_shift == 0`), and the whole-buffer `apply_left_shift_buffer`. The
  shift is the inverse of the encoder's narrowing arithmetic right shift,
  restoring container-scaled PCM from the narrow magnitude the prediction
  loop reconstructs. Per the decorrelation-spec doc §1 pipeline ordering
  and §5.2 ("after decorrelation, before final shift"), this fixup runs
  *after* the running CRC is folded over the pre-shift samples, so it is a
  standalone stage independent of the CRC fold. Pinned by tests covering
  zero-shift identity, power-of-two scaling, sign preservation, the
  encoder-right-shift inverse over a range, malformed large-shift
  wrap-safety, and the buffer/per-element equivalence.

  `WavPackBlock::decode_samples` now applies that fixup as its final stage:
  the reconstructed (post-decorrelation, post-joint-undo) buffer is shifted
  left by [`Flags::left_shift`] before return, so sub-byte-depth blocks
  (12-bit, 20-bit, …) emit correctly container-scaled PCM rather than the
  narrow magnitude. The decode body is split into a private
  `decode_samples_preshift` (the pre-shift buffer the CRC is computed over)
  wrapped by the shifting `decode_samples`; the CRC paths
  (`verify_decoded_crc`, `decode_samples_muted`) fold the pre-shift buffer
  to match the stored header CRC (spec §1 pipeline / §5.2 "before final
  shift"), then `decode_samples_muted` applies the shift to the emitted PCM
  on a CRC match and zeroes (mutes) on a mismatch. Pinned by tests that
  decode a known mono decorrelation block at several `left_shift` values
  and confirm: the emitted PCM is the shifted reconstruction, zero shift is
  the identity, the CRC verifies against the *pre-shift* fold (and fails if
  the post-shift CRC is stamped), and the mute gate shifts-on-match /
  zeroes-on-mismatch.

- Round 348 — stereo lossless decode + joint-stereo undo + §5.6 CRC mute
  gate, driving the decorrelation milestone to working stereo decode.

  `assemble_stereo_passes` builds the §3.7 application-ordered
  `DecorrPass` list for a two-channel block from the `0x02`/`0x03`/`0x04`
  payloads: per spec §3.6, two weight bytes per pass (channel A then B)
  and the `0x04` seeds partitioned per channel with the term-class count
  (2 / `term` / 1), accepting the cross terms (`-1`/`-2`/`-3`) valid only
  for stereo. `WavPackBlock::decode_samples` now decodes a stereo
  decorrelation block (entropy → `assemble_stereo_passes` →
  `decorrelate_stereo`) and applies the spec §5.4 joint (mid/side) undo
  (`R -= L>>1; L += R`) per pair after decorrelation. The stereo
  decorrelation and joint-stereo decode refusals are lifted; the
  `CROSS_DECORR` flag (bit 5) stays refused on non-hybrid stereo blocks
  (documented only in the hybrid context, §4.1).

  `WavPackBlock::decode_samples_muted` is the spec §5.6 mute gate: it
  recomputes the running §5 CRC over the decoded PCM and, on a mismatch,
  zeros the buffer (the "mute the corrupt block" behaviour), returning
  `(pcm, crc_ok)`. `decode_stream_muted` lifts that gate to the whole
  stream — each audio block is CRC-gated and muted independently —
  returning the concatenated PCM and an `all_crc_ok` flag.

  14 new tests: stereo assembler end-to-end per-channel + cross-term
  round-trips, reverse-order and count-mismatch gates, in-block stereo
  decorrelation + joint-stereo decode, the block- and stream-level mute
  gates (keep on match, mute the bad block, metadata-only skipping).

- Round 339 — block-level CRC verification ties §5 to the decode path.

  New `WavPackBlock::verify_decoded_crc` decodes the block's PCM, folds
  the §5 running CRC over the reconstructed samples (mono via `crc_mono`,
  non-joint stereo via `crc_stereo_interleaved` — the same channel
  dispatch `decode_samples` uses, so the CRC covers post-decorrelation
  PCM), and compares against the stored header CRC word
  (`WavPackBlock::crc`). Returns `Ok(true)` / `Ok(false)` per §5.6 (a
  conformant decoder mutes on `false`) and propagates any
  `decode_samples` error verbatim. Non-mutating checker — callers apply
  the mute themselves. 4 new tests pin a correct-CRC match, a wrong-CRC
  miss, error propagation through a refused (hybrid) block, and a match
  over a multi-pass mono decorrelation block (CRC folded over the
  reconstructed PCM, not the residuals).

- Round 339 — mono lossless decode reaches reconstructed PCM.

  `decode_samples` now runs the full lossless decode pipeline for a
  **mono** (or false-stereo) block carrying decorrelation: the `0x0A`
  entropy stream is decoded into residuals, the `0x02`/`0x03`/`0x04`
  sub-blocks are assembled into an application-ordered `DecorrPass` list
  by the new `decorrelation::assemble_mono_passes`, and `decorrelate_mono`
  runs the §3.2 inverse-prediction loop over the residual buffer in place
  to reconstruct PCM — the first end-to-end reconstructed-PCM path through
  the prediction stage. `assemble_mono_passes` applies the spec §3.7
  reverse-storage convention (on-wire passes stored last-applied-first;
  reversed to application order), the spec `+5` term-byte encoding
  (`decode_term_byte`), one weight per pass, and the per-term seed
  partition, with typed rejects: new `Error::DecorrelationWeightCountMismatch`
  (weight count ≠ term count) and `Error::DecorrelationTermsMissing`
  (`0x03`/`0x04` present without `0x02`), reusing
  `InvalidDecorrelationTerm` / `CrossTermOnMono` /
  `TooManyDecorrelationPasses` / `DecorrelationSampleCountMismatch` /
  `DecorrelationSeedUnderflow`. Stereo decorrelation stays refused
  (`UnsupportedBlockFeature::Decorrelation`) — the `0x04` per-channel
  seed-interleaving order for two channels is not in the staged docs. 23
  new tests (696 total): assembler reverse-order + reject paths, and a
  block-level round-trip that encodes a residual buffer into the `0x0A`
  bitstream, attaches a multi-pass config, and confirms `decode_samples`
  reproduces the standalone `decorrelate_mono` output.

- Round 335 — §3.2/§3.3/§3.7 decorrelation inverse-prediction loop.

  `decorrelation` now assembles the per-term reconstruction loop that
  composes the round-329 scalar primitives over a residual buffer, from
  the clean-room decorrelation trace
  (`docs/audio/wavpack/spec/wavpack-decorrelation.md`). New `DecorrPass`
  carries a term, per-pass `delta`, per-channel working weight(s) and an
  8-slot (`MAX_TERM`) per-channel history ring seeded from the `0x04`
  decorr-samples (`DecorrPass::new`, with `Error::InvalidDecorrelationTerm`
  / `Error::DecorrelationSeedUnderflow` rejections). `decorrelate_mono`
  and `decorrelate_stereo` run an ordered pass list in place over the
  whole buffer per pass (§3.7 application order): the fixed-lag terms
  `1..8` read `history[m]` / write `history[(m+t)&7]` and the
  extrapolators `17`/`18` shift a 2-tap history, both via `apply_weight`
  + `update_weight` (§3.2); the zero-delay cross terms `-1`/`-2`/`-3`
  (stereo only) use `update_weight_clip` (§3.3). `decode_term_byte` reads
  the spec's `+5`-biased term encoding (§2.1, distinct from `expand_terms`
  which reads the older wiki listing). New `is_valid_term` /
  `is_cross_term` predicates and `MAX_TERM` (`8`) / `MAX_NTERMS` (`16`) /
  `TERM_BYTE_BIAS` (`5`) constants name the §6 values. 13 new tests pin a
  forward-encode / inverse-decode round trip across every term and
  multi-pass mono + stereo stacks, plus the rejection boundaries
  (`CrossTermOnMono`, `TooManyDecorrelationPasses`). The loop is not yet
  wired into `WavPackBlock::decode_samples`.

- Round 329 — §3 decorrelation inverse-prediction arithmetic primitives.

  `decorrelation` now exposes the three core scalar primitives of the
  inverse-prediction loop from the clean-room decorrelation trace
  (`docs/audio/wavpack/spec/wavpack-decorrelation.md` §3): `apply_weight`
  (§3.1, `(weight*sample + 512) >> 10` with the `1024`-is-unity scaling,
  computed via an `i64` product so wide samples cannot overflow before
  the shift), `update_weight` (§3.4, the LMS-style `±delta` adaptation —
  no change when either operand is zero, `+delta` on matching signs,
  `-delta` on opposite signs), and `update_weight_clip` (§3.5, the
  cross-channel variant that performs the spec's branch-free
  magnitude step and clamps to `±1024`). New `WEIGHT_SHIFT` (`10`),
  `WEIGHT_ROUND_BIAS` (`512`), and `WEIGHT_CLIP` (`1024`) constants name
  the §6 scale/round/clip values; all three functions plus the constants
  are re-exported. 11 new tests pin the §7 sanity vectors
  (`apply_weight(1024, x) == x`, `apply_weight(512, 100) == 50`), the
  wide-sample no-overflow path, the arithmetic-shift flooring of
  negatives, the zero-operand / same-sign / opposite-sign arms of both
  weight updates, and the magnitude clamp at `±1024` from both sign
  directions. These are the building blocks of the per-term
  reconstruction loop (§3.2/§3.3); wiring them into a consuming decode
  pass over the entropy residuals remains later-round work.

- Round 325 — §5 running block-CRC primitives over decoded PCM.

  New `crc` module implements the clean-room decorrelation/CRC trace
  (`docs/audio/wavpack/spec/wavpack-decorrelation.md` §5): the
  `0xffffffff` seed (`CRC_INIT`), the mono step `crc*3 + s`
  (`update_mono`), the stereo step `crc*9 + 3L + R` (`update_stereo`),
  and the joint-stereo mid/side undo `R -= L>>1; L += R`
  (`undo_joint_stereo`) that the spec applies before the stereo step. A
  `BlockCrc` accumulator (`push_mono` / `push_stereo_pair` /
  `push_joint_stereo_pair` / `value` / `matches`) and the one-shot
  `crc_mono` / `crc_stereo_interleaved` / `crc_joint_stereo_interleaved`
  helpers are re-exported. All arithmetic uses 32-bit wrap-around with
  samples folded in as their two's-complement `u32` bit patterns. 27 new
  tests pin both of the spec §7 worked CRC vectors (mono
  `[3,-2,5,0,-7] → 0xfffffff0`, stereo `[(3,-2),(5,0),(-7,9)] →
  0xffffffd9`), the seed, the shift-form equivalence of the stereo step,
  the joint-stereo undo round-trip across a range, accumulator/free-fn
  parity, trailing-odd-sample handling, order/perturbation sensitivity,
  and the `matches` verification arms. This computes the CRC but does not
  yet wire it into the block-end mute path (gated on the
  decorrelation/hybrid decode loop); the extension CRC (`crc_x`, §5.5)
  remains pending its `0x0C` consumer.

- Round 320 — typed refusal of joint-stereo / cross-channel
  decorrelation stereo blocks.

  `WavPackBlock::decode_samples` now refuses a stereo block carrying the
  wiki "Flags meaning" bit 4 ("joint stereo coding scheme") or bit 5
  ("cross-decorrelation scheme is used") with two new
  `UnsupportedBlockFeature` variants (`JointStereo`,
  `CrossChannelDecorrelation`). Both flags select an inter-channel
  transform whose inverse arithmetic is not specified in the staged docs
  under `docs/audio/wavpack/`; the prior code decoded the two channels
  independently, silently emitting the mid/side residuals instead of
  reconstructed left/right PCM. The gates are stereo-only (keyed on
  `Flags::is_block_data_mono`), so mono and false-stereo blocks with the
  bits set still decode normally. Six new tests cover the two refusals,
  the gate-order priority, the combined-flag case, and the mono /
  false-stereo pass-through, plus two new `Display` round-trip cases.

- Round 296 — `decode_stream` cargo-fuzz target + seed corpus.

  New `fuzz/` libfuzzer sub-crate with a `decode_stream` target driving
  arbitrary bytes through the broadest public decode entry point — block
  header validation, the metadata sub-block walker, the `0x05`
  entropy-info seed expander, and the `0x0A` modified-Rice sample-word
  decoder — asserting panic / overflow / OOM freedom. Ships a five-file
  seed corpus (minimal mono / stereo audio blocks, a metadata-only
  block, back-to-back blocks, a multi-sample mono block) that decodes
  cleanly through `decode_stream`.

- Round 281 — spec §4.2 exact-inverse entropy encoder on the public
  surface.

  New write-side twins for the entire decode ladder: `BitWriter`
  (LSB-first emitter, inverse of `BitReader`); `split_sign` (const
  inverse of `apply_sign`); `SampleInterval::encode_mantissa` /
  `encode_value` / `encode_signed_value` (refusing out-of-interval
  values via the new `Error::ValueNotInInterval`); `emit_raw_prefix` /
  `emit_end_of_stream_marker` (inverse of `read_raw_prefix` including
  the `LIMIT_ONES = 16` escape and the `cbits == 33` EOF marker);
  `emit_zero_run_length` (public lift of the round-278 test-side
  inverse); `RunState::unfold_prefix` (const inverse of `fold_prefix`,
  choosing the spec §4.2 step 4 boundary carry);
  `AdaptiveMedians::zone_for_magnitude` (inverse of the §4.2 step 5
  interval ladder); and end-to-end `encode_packed_samples_mono` /
  `encode_packed_samples_stereo` (+ `_from_entropy` twins) walking the
  same per-word state machine the decode loops walk, padding the
  payload to an even byte count per the spec §1 `0x0A` length rule.
  42 net-new tests (625 total) pin every primitive against its decode
  twin, mono + stereo round-trips across zones / signs / extremes /
  zero-runs / LCG-mixed sequences with end-state median equality, and
  the 31-bit-mask corner refusal.

### Fixed

- Round 296 — three decode-side hardening fixes against malformed input,
  all found by the new `decode_stream` cargo-fuzz target.

  1. **Pre-allocation amplification.** `decode_packed_samples_mono` /
     `decode_packed_samples_stereo` sized their output
     `Vec::with_capacity` directly from the caller's `count` / `frames`,
     ultimately the raw 32-bit `block_samples` header field. A ~44-byte
     block claiming `block_samples = 0x21000001` with a 2-byte `0x0A`
     payload forced a ~2.2 GB reservation before the loop reached its
     first truncated read. The new private `prealloc_floor` clamps the
     reservation hint to `min(count, 8 * payload_len)` (spec §4.2: a
     bit-reading sample word costs at least one wire bit).

  2. **Eager-decode amplification (OOM).** The reservation cap alone is
     insufficient because the spec §4.2 step 1 zero-run fast path lets a
     single ~63-bit run word expand to ~`2^31` zero samples, so the
     emitted count is not bounded by the `0x0A` payload byte length: a
     tiny block with `block_samples ≈ u32::MAX` still grows the output
     `Vec` to billions of entries via `push`. `WavPackBlock::decode_samples`
     now rejects `block_samples` above the new
     `MAX_DECODE_SAMPLES_PER_BLOCK` ceiling (`1 << 26`, well above any
     real block) with the typed `Error::BlockSamplesTooLarge`. This is a
     defensive engineering bound, not a spec limit.

  3. **Prefix add-overflow panic.** `read_raw_prefix`'s spec §4.2 step 3
     escape arm computed `UNARY_ESCAPE + escape_value`; with `cbits = 32`
     (within the `<= 33` cap) and all-one mantissa bits `escape_value`
     reaches `u32::MAX`, overflowing the `u32` add (a debug-build panic).
     The back-add is now `checked_add`, surfacing `Error::Truncated` on
     overflow — mirroring the existing round-278 `cbits > 33` handling.

  Decode behaviour for every valid stream is unchanged. New
  `Error::BlockSamplesTooLarge` variant + `MAX_DECODE_SAMPLES_PER_BLOCK`
  public constant; 6 net-new regression tests (631 total). A sustained
  ~6M-exec `decode_stream` campaign is crash-free after the fixes.

- Round 281 — three spec §4.2 conformance corrections surfaced by the
  inverse-encoder construction:

  1. The §4.2 step 4 holding-bit fold (`RunState::fold_prefix`, the
     decode loops, and `decode_run_length`) now adds the `+1` from the
     PRIOR held-one state ("if a one **is being held**… the **new**
     held-one is the **old** low bit"), not from the raw value's own
     low bit. The previous behaviour transcribed the wiki pseudocode's
     assign-then-test order — flagged non-factual by the staged docs —
     under which a zone-0 word could never be followed by a
     non-zero-zone word. `decode_run_length` now delegates to
     `read_folded_ones_count`, gaining the typed `Error::EndOfStream`
     / `Error::Truncated` escape handling in place of round-5's
     shift-overflow debug panic on `n2 >= 33`.
  2. A zero-length spec §4.2 step 1 run is the encoder's "no zero run
     here" marker: decoding now falls through to the regular sample
     word (previously it emitted a `0` sample, making every eligible
     word decode to `0` forever).
  3. The §4.2 step 1 eligibility gate (`DecodeState::zero_run_eligible`
     / `StereoDecodeState::zero_run_eligible`) reads the RAW stored
     `median[0] <= 1` per the spec §2.1 raw-vs-working distinction —
     previously `get_med(0) <= 1`, i.e. raw `<= 15`.

- Round 278 — spec §4.2 step 1 zero-run fast path lifted onto the
  public typed surface + over-cap shift-overflow hardening.

  The clean-room entropy doc `docs/audio/wavpack/spec/wavpack-entropy-decode.md`
  §4.2 step 1 specifies the run-of-zeros fast path: when both channels'
  `median[0]` are ≤ 1 and no holding state is pending, the stream may
  carry an explicit zero-run — a leading unary count (capped at 33)
  that is the run length directly when `< 2`, otherwise `count - 1`
  bits read LSB-first with the top bit implied set; a non-zero run
  resets both channels' medians to zero and emits a `0` sample. Rounds
  255 / 260 / 261 / 274 lifted §4.2 steps 5 / 6 / 7 / 2-4 onto the
  typed surface, leaving step 1 as the only rung still living solely
  inside the private `try_zero_run_path` helpers.

  New `read_zero_run_length(reader)` is the pure on-wire step 1 decode
  (no eligibility gate, no median mutation); new
  `DecodeState::zero_run_eligible(&medians)` and
  `StereoDecodeState::zero_run_eligible(&[medians; 2])` are the pure
  eligibility predicates (working `median[0] <= 1` on every channel AND
  no `last_one` / `last_zero` pending on any `RunState`). Both private
  zero-run paths now delegate to the predicate + primitive, so the
  exact bits the decode loops consume ARE the bits the public
  primitives consume.

  Hardening (same commit): a unary count of exactly 33 in the zero-run
  context previously hit a `1u32 << 32` shift-overflow debug panic (the
  spec assigns 33 EOF semantics only in the §4.2 step 3 escape, and a
  33-count run length needs an implied bit 32 that exceeds the `u32`
  accumulator) — it now reports `Error::Truncated`, as does a §4.2
  step 3 second unary beyond the 33 cap in `read_raw_prefix` (formerly
  a `get_bits(33)` / `1u32 << 33` debug panic on adversarial input).
  Both edges are spec-silent; failing loudly replaces undefined
  arithmetic. 12 new tests (583 total, up from 571) pin: direct counts
  0 / 1 with exact bit consumption; the implied-top-bit round-trip
  across run lengths `[2, 600]` at exactly `2 * count` bits; the
  widest in-range form (`count == 32` → `u32::MAX`); the 33-cap and
  over-cap typed errors (no panic); `Truncated` on an empty buffer;
  every mono + stereo eligibility gate (median threshold at the
  `get_med` 15/16 raw boundary, each of the holding bits); bit-exact
  loop-vs-primitive parity on a length-5 zero-run including the
  no-bits drain of the pending debt; the mono + stereo loop-level cap
  errors (stereo also pinning the untouched `next_channel` error
  contract); and the `read_raw_prefix` second-unary over-cap error.

- Round 274 — spec §4.2 step 2 + 3 raw modified-Rice prefix decode and
  spec §4.2 step 4 holding-bit fold lifted onto the public typed
  surface.

  The clean-room entropy doc `docs/audio/wavpack/spec/wavpack-entropy-decode.md`
  §4.2 step 2 reads the count of consecutive `1` bits terminated by a
  `0` bit (the modified-Rice prefix), step 3 escapes that prefix when it
  reaches `LIMIT_ONES = 16` (a second unary `cbits` up to `33`, with
  `cbits == 33` the EOF marker and `cbits >= 2` carrying an implied
  top-bit mantissa), and step 4 folds the raw prefix onto the
  `ones_count` zone selector via the held `last_one` / `last_zero`
  carry. Rounds 255 / 260 / 261 lifted §4.2 steps 5 / 6 / 7 onto the
  typed `SampleInterval` / `apply_sign` surface, but steps 2-4 still
  lived only in the private `read_folded_ones_count` helper inside the
  decode loop — so callers walking the spec ladder by hand could not
  name the prefix decode or the fold as typed operations.

  New `read_raw_prefix(reader)` reads the §4.2 step 2 unary plus the
  §4.2 step 3 escape, returning the pre-fold `raw_value` and surfacing
  the `cbits == 33` EOF as `Error::EndOfStream` (distinct from a buffer
  that merely ran dry, which is `Error::Truncated`). New
  `RunState::fold_prefix(raw_value)` is the pure §4.2 step 4 fold — it
  mutates the held `last_one` / `last_zero` registers in place and
  returns the folded `ones_count` zone selector; it reads no bits and is
  `const`-evaluable. `read_folded_ones_count` is now public and
  delegates to these two primitives (after the wiki `last_zero`
  short-circuit), so the exact bits the decode loop consumes ARE the
  bits the public primitives consume. 12 new tests (571 total, up from
  559) pin: `read_raw_prefix` plain-unary returns for every raw in
  `[0, 16)` with exact bit-consumption; the escape arm for `cbits < 2`;
  the `cbits >= 2` implied-top-bit round-trip across escape values
  `[2, 256]`; the `cbits == 33` EOF marker; `Error::Truncated` on an
  empty buffer; `fold_prefix` low-bit-zero halving / low-bit-one
  half-plus-one / held-one-zero-complement invariant across raw values;
  `const` evaluability; the fused-equals-two-step bit-exact identity
  (value, cursor AND post-fold state) across the plain + escape range;
  the `last_zero` short-circuit consuming no bits and leaving `last_one`
  untouched; and EOF propagation through the fused path.

- Round 261 — spec §4.2 step 7 sign-bit reconstruction lifted onto the
  typed surface.

  The clean-room entropy doc `docs/audio/wavpack/spec/wavpack-entropy-decode.md`
  §4.2 step 7 specifies the last on-wire field of every sample word:
  "Sign bit, last. After the magnitude is fixed, read exactly one sign
  bit. If the sign bit is set the returned sample is the bitwise
  complement of the magnitude (`~mid`), otherwise the magnitude
  itself." Round 15 wired this inline at the tail of
  `decode_sample_stateful` (round 199 in
  `decode_sample_stateful_stereo`), and round 260's
  `SampleInterval::decode_value` explicitly stopped at the unsigned
  magnitude — step 7 had no typed name on the public surface.

  New free functions:

  * `apply_sign(magnitude: u32, sign_bit_set: bool) -> i32` — the pure
    `const` spec §4.2 step 7 arithmetic: `magnitude as i32` when the
    sign bit is clear; `!(magnitude as i32)` (two's-complement
    `-(magnitude + 1)`) when set. Reads no bits.
  * `read_sign_and_apply(reader, magnitude) -> Result<i32>` — reads
    exactly ONE bit per the spec sentence and folds it through
    `apply_sign`.

  New on `SampleInterval`:

  * `SampleInterval::decode_signed_value(reader) -> Result<i32>` —
    fuses spec §4.2 steps 6 + 7: `decode_value` for the unsigned
    magnitude, then the sign bit. This is the complete value tail of
    one sample word (`mantissa bits → sign bit` per the spec §4.2
    closing on-wire-order line) once the zone selector is fixed.

  Both decode loops (`decode_sample_stateful` /
  `decode_sample_stateful_stereo`) now delegate their steps 5-8 tail
  to `AdaptiveMedians::sample_interval_for_ones_count` +
  `SampleInterval::decode_signed_value`, so the exact bits the loops
  consume ARE the bits the typed surface consumes. The round-255
  private `form_interval` tuple shim no longer has production callers
  and is now `#[cfg(test)]` (kept for the round-255 parity tests).

  15 new tests (559 total, up from 544).

- Round 260 — spec §4.2 step 6 truncated-binary mantissa primitive lifted
  onto the `SampleInterval` surface + spec §3.2 zone-predicate accessors
  on `Zone`.

  The clean-room entropy doc `docs/audio/wavpack/spec/wavpack-entropy-decode.md`
  §4.2 step 6 first paragraph specifies the truncated-binary mantissa
  decode the stateful loop reads inside a `(low, high)` interval —
  `maxcode = 0` consumes no bits and returns `0`; `maxcode = 1` consumes
  one bit; `maxcode >= 2` consumes `bitcount - 1` bits LSB-first into
  `code` with `bitcount = floor(log2(maxcode)) + 1` and
  `extras = (1 << bitcount) - maxcode - 1`, and reads one MORE bit when
  `code >= extras` to form the full `bitcount`-bit phase-in codeword.
  Round 15 wired this in the private `read_truncated_binary` helper
  inside `decode_sample_stateful`, but the primitive had not yet been
  lifted to the public method surface.

  New on `SampleInterval`:

  * `SampleInterval::mantissa_bitcount() -> u32` — the pure-arithmetic
    spec `bitcount`, special-cased to `0` for `maxcode == 0`.
  * `SampleInterval::mantissa_extras() -> u32` — the pure-arithmetic
    spec `extras`, also `0` for the `maxcode < 2` arms.
  * `SampleInterval::decode_mantissa(reader) -> Result<u32>` — consumes
    a `BitReader` and returns the integer `code` in `[0, maxcode]`
    per the spec ladder.
  * `SampleInterval::decode_value(reader) -> Result<u32>` — adds
    `low` to the decoded mantissa, returning the full §4.2 step 6
    magnitude. Sign-bit reconstruction is §4.2 step 7 and stays at
    the caller.

  Round 15's private `read_truncated_binary` is now a thin delegator
  over the new typed surface, so the exact bytes the decode loop
  consumes ARE the bytes the typed accessor consumes.

  New on `Zone`:

  * `Zone::index() -> u8` — the zero-based zone arm selector
    (`0/1/2/3`, independent of the carried `ones_count` in the
    overflow arm).
  * `Zone::is_overflow() -> bool` — the `matches!(self,
    Zone::Zone2Overflow { .. })` predicate.
  * `Zone::increments_median(idx) -> bool` — `true` when spec §3.2
    increments `median[idx]` in this zone.
  * `Zone::decrements_median(idx) -> bool` — `true` when spec §3.2
    decrements `median[idx]` in this zone.
  * `Zone::touches_median(idx) -> bool` — union of `increments_median`
    and `decrements_median`.

  Per the spec §3.2 table the predicates lift:

      | Zone           | inc[0] | inc[1] | inc[2] | dec[0] | dec[1] | dec[2] |
      | -------------- | ------ | ------ | ------ | ------ | ------ | ------ |
      | Zone0          | no     | no     | no     | yes    | no     | no     |
      | Zone1          | yes    | no     | no     | no     | yes    | no     |
      | Zone2          | yes    | yes    | no     | no     | no     | yes    |
      | Zone2Overflow  | yes    | yes    | yes    | no     | no     | no     |

  The §3.2 mutation itself stays on `AdaptiveMedians::adapt` — the new
  predicates are pure (do NOT touch the median values).

  22 new tests (544 total, up from 522).

- Round 255 — typed `SampleInterval` view + `AdaptiveMedians::sample_interval`
  / `sample_interval_for_ones_count` accessors lifting the spec §4.2 step 5
  `(low, high)` interval-formation primitive onto the public method surface.

  The clean-room entropy doc `docs/audio/wavpack/spec/wavpack-entropy-decode.md`
  §4.2 step 5 specifies the four-arm interval ladder a decoder forms from the
  three running medians and the (folded) `ones_count` zone selector:

      Zone 0:          low = 0,                       high = get_med(0) - 1
      Zone 1:          low = get_med(0),              high = low + get_med(1) - 1
      Zone 2:          low = get_med(0) + get_med(1), high = low + get_med(2) - 1
      Zone2Overflow N: low = m0 + m1 + (N-2)*m2,      high = low + get_med(2) - 1

  with both ends masked to 31 bits (`INTERVAL_MASK_31 = 0x7fff_ffff`) and
  `high` clamped up to `low` on underflow. Round 15 wired this in the
  private `form_interval` helper inside `decode_sample_stateful`, but the
  primitive had not yet been lifted to the public method surface — so
  callers walking the spec ladder by hand (or building diagnostic traces
  against a known median set) could not name the interval as a typed
  value.

  - `SampleInterval { low: u32, high: u32 }` — typed value carrying the
    spec §4.2 step 5 `(low, high)` pair with `high >= low` invariant.
    Both fields are public for direct destructuring and the same values
    are reachable via `low()` / `high()` accessors. Derived predicates:
    `maxcode()` returns `high - low` (the literal `maxcode` value the
    truncated-binary mantissa decoder consumes); `width()` returns
    `high - low + 1` (the inclusive codeword count); `is_degenerate()`
    is `true` when `low == high` (single-codeword interval, mantissa
    decode reads zero bits); `contains(value)` is the
    `low <= value <= high` membership test.
  - `AdaptiveMedians::sample_interval(&self, zone: Zone) -> SampleInterval`
    — typed accessor forming the §4.2 step 5 interval from the channel's
    three working medians and the typed `Zone` selector. Masks both
    `low` and `high` to 31 bits and clamps `high` up to `low` on
    underflow per spec.
  - `AdaptiveMedians::sample_interval_for_ones_count(&self, ones_count:
    u32) -> SampleInterval` — convenience wrapper composing
    `Zone::from_ones_count` with `sample_interval` for callers that
    hold a raw `ones_count` value rather than a typed `Zone`.

  The private `form_interval` helper the round-15 decode loop calls is
  now a thin tuple-shaped delegator over the new typed surface, so the
  exact (low, high) bytes the decoder consumes ARE the bytes the new
  typed accessor returns — no parallel implementation. The spec §3.2
  median adaptation step (the mutation that happens at this point in
  the decode loop) stays on `AdaptiveMedians::adapt` and is not exposed
  by the new accessor; the typed interval formation is pure (does NOT
  mutate the medians).

  16 new tests (522 total, up from 506) pin: zone 0 / 1 / 2 / overflow
  formula correctness against hand-traced expected values for the
  seeds `[256, 256, 256]` worked example (get_med = 17 → intervals
  `[0,16] / [17,33] / [34,50] / [51,67] / [68,84] / [85,101]`);
  `sample_interval_for_ones_count` parity with the typed path through
  `Zone::from_ones_count`; `sample_interval` parity with the private
  `form_interval` across a sweep of median sets (uniform, mixed,
  zero, max) and zones (`0..=33`); 31-bit masking invariant for `low`
  and `high` across saturated median configurations; `high >= low`
  invariant across the same saturated sweep; `maxcode` /
  `width` arithmetic (`maxcode == high - low`, `width == maxcode + 1`);
  degenerate-interval predicate (zone 0 with `median[0] == 0` yields
  `(0, 0)`); zone-2-overflow stepping (`ones_count = 3` is exactly one
  m2 step past zone 2's `low`, both intervals share the same
  `maxcode`); `contains` boundary inclusivity (`low` and `high` both
  inside, `low - 1` and `high + 1` both outside); raw `new`
  constructor + accessor parity; and field-access parity with the
  accessors (`i.low == i.low()`, `i.high == i.high()`).

- Round 252 — typed `version` / `track_number` / `track_sub_index`
  accessors + derived `has_track_id` / `supports_false_stereo`
  predicates on `WavPackBlockHeader` / `WavPackBlock`. The wiki
  "Block structure" listing of
  `docs/audio/wavpack/wiki/WavPack.wiki` enumerates three
  explicitly-named header fields the round-1 parser preserved
  verbatim but had not yet lifted to typed accessors: `16 bits -
  version (current valid versions are 0x402 - 0x410)` (bytes 8..10),
  `8 bits - track number (not currently implemented)` (byte 10), and
  `8 bits - track sub index (not currently implemented)` (byte 11).
  The wiki "Flags meaning" listing further annotates bit 30 "false
  stereo" with the explicit gate "version >= 0x410". This round
  lifts all three fields onto the method surface alongside the
  round-214 / round-239 / round-245 accessors and adds the boolean
  discriminants the wiki's own annotations imply.

  - `WavPackBlockHeader::version(&self) -> u16` — typed accessor
    returning the stored 16-bit version verbatim. The parser already
    constrains the field to `MIN_VERSION..=MAX_VERSION` (the wiki's
    `0x0402..=0x0410` window) so every value the accessor returns
    is in the documented range.
  - `WavPackBlockHeader::track_number(&self) -> u8` /
    `WavPackBlockHeader::track_sub_index(&self) -> u8` — typed
    accessors returning the two single-byte fields verbatim. The
    wiki marks both "not currently implemented" but the parser
    preserves the raw byte for diagnostic round-trip / future-
    implementation use.
  - `WavPackBlockHeader::has_track_id(&self) -> bool` — boolean
    discriminant for "this block carries a non-zero stamping in
    either of the two track-id bytes". Common case (both bytes
    zero) → `false`; any non-zero combination → `true`.
  - `WavPackBlockHeader::supports_false_stereo(&self) -> bool` —
    the `version >= 0x0410` gate the wiki places on bit 30 "false
    stereo (stream is stereo but this block's data is mono, version
    >= 0x410)". Lifts the wiki's version gate into a typed boolean
    so callers branching on bit 30 don't repeat the comparison.
  - `WavPackBlock::version` / `WavPackBlock::track_number` /
    `WavPackBlock::track_sub_index` / `WavPackBlock::has_track_id`
    / `WavPackBlock::supports_false_stereo` — five block-level
    pass-throughs pairing the block surface with the header
    accessors.
  - 22 new tests (506 total, up from 484): each accessor's verbatim
    return; little-endian byte-8..10 decoding of `version` through
    the full `parse_block_header` path (with a reverse-byte
    cross-check confirming the alternate byte ordering is refused
    as `UnsupportedVersion`); window-membership of every returned
    `version`; verbatim round-trip across the byte-value extremes
    for `track_number` / `track_sub_index`; independent byte-10 vs.
    byte-11 decoding (distinct values stamped into each don't
    cross); the four `has_track_id` branches (both-zero /
    number-only / sub-index-only / both-set);
    `supports_false_stereo` `true` at the documented maximum
    `0x0410` and `false` at every below-gate in-window value;
    independence from the round-239 / round-245 accessors (varying
    `total_samples` / `block_index` / `block_samples` / `crc`
    leaves the new accessors unchanged); and the block-level
    pass-throughs' parity with the header accessors across a
    two-block stream carrying distinct `version` + track triples.

- Round 245 — block-CRC accessor on `WavPackBlockHeader` /
  `WavPackBlock`. The wiki "Block structure" listing of
  `docs/audio/wavpack/wiki/WavPack.wiki` places a `32 bits - CRC`
  word at the trailing 4 bytes of the fixed 32-byte block header.
  The round-1 parser decoded the word little-endian into the
  `WavPackBlockHeader::crc` field but no typed accessor surfaced
  it on either the header or the block-level view — every other
  documented header field had been lifted to a typed accessor
  through rounds 214 and 239, so this was the last on-disk header
  field without a method surface. This round closes the gap.

  - `WavPackBlockHeader::crc(&self) -> u32` — typed accessor
    returning the stored 32-bit word verbatim. The wiki names the
    field but does not specify the polynomial, the byte span, the
    initial value, or the byte / bit order of the computation, and
    the staged docs do not either; the accessor surfaces the
    stored bytes only and leaves recomputation / verification to a
    later round once the algorithm spec lands.
  - `WavPackBlock::crc(&self) -> u32` — block-level pass-through
    pairing the block surface with the header accessor, for
    callers iterating a multi-block stream that want to pick the
    per-block CRC off a borrowed `WavPackBlock` alongside the
    other round-214 / round-239 introspection accessors.
  - 8 new tests (484 total, up from 476): header accessor verbatim
    return; full-`u32`-range round-trip across documented extremes
    (`0`, `1`, `u32::MAX`, alternating-bit patterns); little-endian
    decode of bytes 28..32 through `parse_block_header`;
    independence from other header fields; block-level
    pass-through parity (`block.crc() == block.header().crc()`);
    full-range extremes round-trip through `parse_block`;
    per-block independence across a two-block stream; block-level
    independence from the round-239 accessors.

- Round 242 — `0x0C` packed-overflow-bits typed view + walker
  bridges + block-level introspection. The wiki "IDs" listing of
  `docs/audio/wavpack/wiki/WavPack.wiki` annotates sub-block `0x0C` as
  "packed overflow bits from floating-point or large integers", and
  the staged clean-room entropy doc
  `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 names the
  same ID as the **extension bitstream**. The round-2 metadata walker
  already routes `0x0C` payloads through the typed
  `SubBlockId::PackedOverflowBits` discriminant, but the bytes had
  not yet been elaborated into the same typed view + finder +
  block-level accessor shape the round-12 `PackedSamples` (`0x0A`)
  and round-233 `PackedCorrectionData` (`0x0B`) views expose. This
  round closes that gap.

  - `PackedOverflowBits<'a>` in `src/overflow_bits.rs` carries the
    borrowed `0x0C` payload verbatim. Mirrors the `PackedSamples` /
    `PackedCorrectionData` shape: `bytes()` / `len()` / `is_empty()`
    introspection + `bit_reader()` factory producing a fresh
    `BitReader` positioned at bit 0 (LSB-first within each byte, the
    same convention every payload-carrying view in the crate uses).
    Any byte slice — including the empty one — is accepted: the wiki
    places no length constraint on the payload.
  - `expand_packed_overflow_bits(payload: &[u8]) -> PackedOverflowBits<'_>`
    — typed constructor analogous to `expand_packed_samples` and
    `expand_packed_correction_data`.
  - `find_packed_overflow_bits_sub_block(subs)` — walker finder
    returning the borrowed `MetadataSubBlock<'a>`, for callers that
    want the raw sub-block handle.
  - `find_packed_overflow_bits(subs)` — typed walker finder returning
    `Option<PackedOverflowBits<'a>>` directly, the one-call bridge
    from the round-2 metadata walker output to the
    `BitReader`-construction handoff.
  - `WavPackBlock::has_packed_overflow_bits()` — boolean
    discriminant for the presence of a `0x0C` sub-block.
  - `WavPackBlock::find_packed_overflow_bits_sub_block()` /
    `WavPackBlock::packed_overflow_bits()` — block-level accessors
    pairing with the free walker finders.

  The float (wiki flag bit 7) / large-integer (wiki flag bit 8)
  container fix-ups that would actually consume the wrapped bytes
  remain gated on the `UnsupportedBlockFeature::FloatData` /
  `UnsupportedBlockFeature::Int32Mode` feature refusals in
  `WavPackBlock::decode_samples`. The typed view is a deferred
  handoff into the bit reader, not a decode pass.

  17 new tests (476 total, up from 459): the new module's view +
  walker + bit-reader contract (`new_preserves_bytes_verbatim` /
  `empty_payload_is_accepted_and_reported_empty` /
  `expand_packed_overflow_bits_round_trips_the_byte_slice` /
  `bit_reader_starts_at_byte_zero_bit_zero` /
  `bit_reader_yields_first_bit_lsb_first` /
  `bit_reader_factory_returns_independent_readers` /
  `empty_payload_bit_reader_reports_immediate_truncation` /
  `view_is_copy_and_independent_of_caller_lifetime` /
  `view_is_distinct_type_from_packed_samples_and_correction`) plus
  the metadata walker bridges
  (`find_packed_overflow_bits_typed_view_returns_view_when_0x0c_present`
  / `find_packed_overflow_bits_typed_view_returns_none_when_0x0c_absent`
  / `find_packed_overflow_bits_sub_block_returns_metadata_borrow` /
  `find_packed_overflow_bits_sub_block_returns_none_when_0x0c_absent`)
  plus the block-level accessor surface
  (`has_packed_overflow_bits_returns_false_on_no_0x0c_subblock` /
  `has_packed_overflow_bits_returns_true_with_0x0c_subblock` /
  `packed_overflow_bits_view_round_trips_with_bit_reader` /
  `has_packed_overflow_bits_is_independent_of_0x0b_and_0x07`).

- Round 239 — typed file-total / end-cursor accessors on
  `WavPackBlockHeader` / `WavPackBlock` and the stream-level
  `stream_total_samples` free function. The wiki "Block structure"
  listing of `docs/audio/wavpack/wiki/WavPack.wiki` names three
  sample-cursor fields the round-1 header parser preserved verbatim
  (`total samples in file` with `0xFFFFFFFF` reserved as the wiki
  "unknown" sentinel; `offset in samples for current block`; `samples
  in this block`), but only the boolean `is_total_samples_known`
  discriminant was surfaced through the typed API. This round adds:

  - `WavPackBlockHeader::total_samples_in_file` — returns
    `Option<u32>`, the typed sentinel-aware view of the file-global
    total. `Some(n)` for a known total, `None` for the
    `TOTAL_SAMPLES_UNKNOWN` constant. `Some(0)` and `None` remain
    distinguishable — the wiki allows a literal zero total as a
    legitimate "no audio at all" value, distinct from the sentinel.
  - `WavPackBlockHeader::end_sample_index` — returns `u64` =
    `block_index + block_samples`, the half-open upper bound of this
    block's sample contribution. A metadata-only block
    (`block_samples == 0`) reports the same cursor as `block_index`,
    consistent with the wiki "may be 0 if no audio present" note. The
    `u64` return type covers the pathological `u32::MAX + u32::MAX`
    summands without overflow.
  - `WavPackBlockHeader::samples_remaining_after` — returns
    `Option<u64>` = `total - end` when both the total is known and the
    end cursor lies within it. `None` for the wiki sentinel
    (cannot answer without the total) and for the malformed end-past-
    total combination (refuses to surface as a negative count).
  - `WavPackBlock::total_samples_in_file` /
    `WavPackBlock::end_sample_index` /
    `WavPackBlock::samples_remaining_after` — block-level pass-throughs
    so callers iterating parsed blocks reach the typed values without
    going through `.header`.
  - `WavPackBlock::is_final_audio_block_in_file` — boolean
    `samples_remaining_after() == Some(0)` discriminant for the
    "last block of a fully-described `.wv` file" case.
  - `stream_total_samples(&[u8]) -> Result<Option<Option<u32>>>` —
    stream-level free function reading the typed file-total from the
    first block's header. Outer `None` for empty input (no first
    block); outer `Some` carrying the inner `Option<u32>` from
    `WavPackBlockHeader::total_samples_in_file`. The wiki documents
    `total_samples` as file-global, so reading only the first block's
    32-byte fixed header (constant-time, no metadata walk) is the
    minimal call that surfaces the stream-level total.

  All four surfaces derive directly from the three explicitly
  documented wiki fields — no spec gap, no docs-gap-blocked surface
  touched. No new error variants. The `WavPackBlock` exports are
  re-exported through the existing `crate::block` block; the new
  `stream_total_samples` free function joins the existing stream-level
  free-function surface in `lib.rs`.

- 23 new unit tests (459 total, up from 436) pin: the sentinel /
  known / zero-as-distinct-from-sentinel discrimination on the typed
  `Option`; the u32-extreme-summands non-overflow on
  `end_sample_index`; the metadata-only-block end-cursor
  non-advancement (cursor stays at `block_index`); the
  exact-end / non-zero-remainder / unknown-total / end-past-total
  branches of `samples_remaining_after`; the boolean
  `is_final_audio_block_in_file` discriminant on each of those
  branches; the stream-level free function on empty / single-block /
  multi-block / sentinel / malformed-header inputs; the first-block-
  only contract (a second block's hypothetically-different total
  is not consulted); and the cross-block consistency of
  `end_sample_index` / `samples_remaining_after` /
  `is_final_audio_block_in_file` across a synthesised three-block
  stream.

- Round 233 — `.wvc` correction-stream typed view + walker bridges +
  block-level and stream-level introspection accessors. The wiki "IDs"
  listing of `docs/audio/wavpack/wiki/WavPack.wiki` annotates sub-blocks
  `0x07` (noise-shaping profile) and `0x0B` (packed correction data) as
  carried in the `.wvc` companion file alongside the lossy main `.wv`;
  this round elaborates the round-2 walker output for `0x0B` into a typed
  `PackedCorrectionData<'a>` view (analogous to the round-12
  `PackedSamples<'a>` view for `0x0A`) and threads the same finder /
  predicate / iterator pattern through the metadata-walker / block-level
  / stream-level surfaces.

  - New `PackedCorrectionData<'a>` typed view in
    `src/correction.rs` carries the borrowed `0x0B` sub-block payload
    verbatim and exposes the `bytes()` / `len()` / `is_empty()` byte
    introspection plus a `bit_reader()` factory that yields a fresh
    `BitReader<'a>` positioned at bit 0 — the same shape
    `PackedSamples::bit_reader` exposes for the main `0x0A` stream.
  - New free `expand_packed_correction_data(payload: &[u8]) ->
    PackedCorrectionData<'_>` constructor — the round-2 walker output
    bridge for the `0x0B` ID, analogous to `expand_packed_samples`.
  - New walker finders `find_packed_correction_data` (typed-view) /
    `find_packed_correction_data_sub_block` (raw-borrow) /
    `find_noise_shaping_profile` (raw-borrow) / `find_hybrid_profile`
    (raw-borrow) in `src/metadata.rs` pair the walker output with the
    three hybrid-mode sub-block IDs without re-walking the metadata.
  - New `WavPackBlock` accessors: `has_packed_correction_data` /
    `packed_correction_data` / `find_packed_correction_data_sub_block`
    / `has_noise_shaping_profile` /
    `find_noise_shaping_profile_sub_block` / `has_hybrid_profile` /
    `find_hybrid_profile_sub_block` / `has_correction_stream_data`
    (the composite predicate matching the
    `MetadataSubBlock::is_correction_payload` grouping).
  - New stream-level free functions: `correction_block_count` /
    `first_correction_block` / `iter_correction_blocks` /
    `total_correction_payload_bytes`, plus a new
    `CorrectionBlockIter<'a>` (`Clone` + `FusedIterator`) lazy
    iterator mirroring the round-230 `AudioBlockIter` shape but
    filtering to blocks whose `has_correction_stream_data` predicate
    fires.

  The hybrid-mode sample decode itself (spec §4.2 step 6 second
  paragraph, `error_limit != 0`) stays out of scope — the typed views
  give a callable handle into the bytes without committing to a decode
  semantics, and the existing
  `Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Hybrid)`
  refusal on `WavPackBlock::decode_samples` is preserved verbatim.
- 44 new unit tests (436 total, up from 392) pin: the typed
  `PackedCorrectionData` view shape (empty / non-empty / round-trip /
  bit-reader factory / `Copy` + lifetime / distinct-from-`PackedSamples`
  type discrimination); each new block-level accessor / finder on
  present / absent / both-present inputs; the composite
  `has_correction_stream_data` predicate as the union of `0x07` and
  `0x0B`; the new stream-level free functions on empty / all-plain /
  mixed / error-trailing inputs; `CorrectionBlockIter` `Clone +
  FusedIterator` trait bounds and the `new` / free-function
  call-shape twin equivalence; `total_correction_payload_bytes`
  summing only `0x0B` payload bytes (excluding `0x07`); a
  metadata-only block carrying only a `0x0B` payload still surfaces
  as correction-bearing (block-samples allowance honoured); and the
  hybrid-flag refusal contract is unchanged by the presence of a
  `0x0B` typed view (structural introspection vs. decode
  enablement).

- Round 230 — stream-level introspection accessors composing the
  round-219 [`iter_blocks`] for aggregate "how many / what shape" /
  "where's the first audio block" questions without retaining the
  parsed block list. New free functions [`audio_block_count`] /
  [`metadata_block_count`] split the wiki "Block structure"
  `block_samples > 0` audio blocks from the `block_samples == 0`
  metadata-only blocks (RIFF wrappers, MD5 sums, encoding-details);
  together they sum to [`block_count`] across any input. New
  [`total_audio_samples`] sums the wiki "samples in this block" field
  across audio blocks only, returning `u64` so a 4-GiB-plus stream's
  sample count does not overflow `u32`. New [`decoded_sample_count`]
  free function sums the `i32` PCM slot count [`decode_stream`] would
  produce across the audio blocks (mono / false-stereo contribute
  `block_samples`, stereo contribute `block_samples * 2`); the matching
  block-level [`WavPackBlock::decoded_sample_count`] reports the same
  shape per-block from the header alone — no entropy expansion, no
  per-sample-loop call. New [`first_audio_block`] peeks the first
  decode-eligible block past any leading metadata-only blocks (the wiki
  allowance for RIFF-header-only blocks); returns `Ok(None)` on empty /
  all-metadata-only input and surfaces the first [`parse_block`] error
  verbatim if a malformed block appears before the first audio block.
  New [`AudioBlockIter`] is the `Clone`-able, `FusedIterator`-compliant
  lazy iterator yielding only audio blocks; [`iter_audio_blocks`] is
  the call-shape twin. New [`BlockIter::next_audio`] adapter method
  skips metadata-only blocks on the existing block iterator. No new
  error variants and no docs-gap-blocked surface touched.
- 34 new unit tests (392 total, up from 358) pin: each accessor on
  empty / all-metadata-only / mixed inputs; the
  audio + metadata == total identity across counts; the structural
  [`WavPackBlock::decoded_sample_count`] matching the actual PCM length
  [`WavPackBlock::decode_samples`] returns on mono and stereo;
  [`decoded_sample_count`] (stream) matching the [`decode_stream`]
  result length; [`AudioBlockIter`]'s `Clone + FusedIterator` trait
  bounds; the
  `audio_block_count == iter_audio_blocks().count()` equivalence; the
  `BlockIter::next_audio` skip-then-yield-then-error-then-fuse
  contract; [`AudioBlockIter::new`] and [`iter_audio_blocks`] returning
  identical sequences; and parse-error propagation through every
  accessor.

- Round 224 — multi-block stream → PCM composer fusing the round-219
  [`BlockIter`] with the round-206 [`WavPackBlock::decode_samples`]
  into a single byte-buffer → `Vec<i32>` surface. New eager
  [`decode_stream`] walks every audio block in the input and
  concatenates the decoded PCM in on-disk order; new
  [`StreamDecodeIter<'a>`] is the `Clone`-able,
  `FusedIterator`-compliant lazy counterpart yielding
  `Result<Vec<i32>>` once per **audio** block (metadata-only blocks
  with `block_samples == 0` are silently skipped since they carry no
  PCM to return — a positive contract preventing a spurious
  [`Error::BlockHasNoAudio`] refusal on `.wv` files whose first block
  is a RIFF-header-only metadata block). New free function
  [`iter_decoded_blocks`] is the `iter_decoded_blocks(bytes)`
  call-shape twin of [`StreamDecodeIter::new`].
  [`StreamDecodeIter`] fuses on the first error (parse or decode) via
  the underlying round-219 [`BlockIter`] fuse mechanism + the
  round-206 refusal taxonomy — both compose without translation. Two
  introspection accessors on [`StreamDecodeIter`] —
  [`StreamDecodeIter::remaining`] (forwards [`BlockIter::remaining`])
  and [`StreamDecodeIter::is_exhausted`] (forwards
  [`BlockIter::is_exhausted`]) — round out the surface. No new error
  variants and no docs-gap-blocked surface touched: every error this
  composer surfaces is one [`parse_block`] or
  [`WavPackBlock::decode_samples`] already raised. Per-block mono /
  stereo dispatch (the wiki bit 2 + bit 30 union from round 206 via
  [`Flags::is_block_data_mono`]) is preserved verbatim, so a
  multi-block input may mix mono and stereo blocks and the
  concatenated `Vec<i32>` reflects each block's own shape. Per-block
  `0x05` seed re-initialisation (round 206 per-block stateless
  contract) is preserved across the stream.
- 21 new unit tests (358 total, up from 337): empty-buffer input
  yielding `Ok(vec![])` (not an error — the wiki "WavPack file
  consists of blocks" sentence is plural but [`BlockIter`] accepts the
  degenerate empty file); single audio block yielding `[0]`; three
  audio blocks concatenating to `[0, 0, 0]` in on-disk order;
  metadata-only blocks silently skipped both between audio blocks and
  at the leading position (no spurious [`Error::BlockHasNoAudio`]);
  all-metadata-only input yielding `Ok(vec![])`;
  [`Error::CkSizeExceedsBuffer`] propagated verbatim from a malformed
  second block; [`Error::UnsupportedBlockFeature(Hybrid)`] propagated
  verbatim from a hybrid-flagged audio block;
  [`Error::BlockMissingEntropyInfo`] propagated from an audio block
  lacking the `0x05` sub-block; eager [`decode_stream`] discarding
  prior-block PCM on a mid-stream decode error (the documented eager
  contract); [`iter_decoded_blocks`] yielding one item per audio
  block with metadata-only blocks omitted; [`iter_decoded_blocks`]
  fusing on first parse error AND on first decode error (both
  routes); empty input and all-metadata-only input yielding zero
  items; [`StreamDecodeIter::new`] and [`iter_decoded_blocks`]
  returning identical sequences; [`StreamDecodeIter::remaining`]
  tracking the underlying [`BlockIter`] across a `next()` call (full
  buffer → empty after draining); the `Clone + FusedIterator` trait
  bounds (a compile-time check via a generic helper); a mixed
  mono+stereo input yielding `[0, 0, 0]` confirming the per-block
  dispatch contract; and the eager / lazy equivalence over a
  three-audio-block input (`decode_stream` is observationally
  identical to draining [`iter_decoded_blocks`] and concatenating
  each `Vec<i32>` via `flat_map`).

- Round 219 — multi-block stream iteration on top of the round-13
  [`parse_block`] composer, lifting the wiki "WavPack file consists of
  blocks each beginning with 'wvpk'" file-format sentence into typed
  public API. New [`BlockIter<'a>`] (a `Clone`-able, `FusedIterator`-
  compliant lazy iterator yielding `Result<WavPackBlock<'a>>`) walks
  the chain by repeatedly calling [`parse_block`] on the previous
  call's returned tail; the iterator fuses on the first error so the
  caller can `?`-bubble without re-encountering the same failure on
  a follow-up `next()`. New free function [`iter_blocks`] is the
  `iter_blocks(bytes)` call-shape twin of [`BlockIter::new`]. New
  [`parse_blocks`] eagerly collects the iterator into a
  `Vec<WavPackBlock<'_>>`, surfacing the first parse error verbatim.
  New [`block_count`] iterates without retaining the parsed blocks
  for callers that want only the count (working-set memory stays at
  one block independent of input length). New [`total_block_samples`]
  pure accessor sums the wiki "samples in this block" field across
  an already-parsed block list, returning a `u64` so a 4-GiB-plus
  stream's sample count does not overflow `u32` on the way out.
  Two introspection accessors on [`BlockIter`] —
  [`BlockIter::remaining`] (the bytes not yet consumed; on a fused
  iterator after an error, points at the malformed block's first
  byte for precise offset diagnostics) and [`BlockIter::is_exhausted`]
  — round out the surface. No new error variants and no docs-gap-
  blocked surface touched: every iterator yield is a direct
  [`parse_block`] result and the existing round-13
  [`Error::CkSizeExceedsBuffer`] / [`Error::Truncated`] split
  surfaces verbatim across the iteration boundary.
- 18 new unit tests (337 total, up from 319): empty-buffer
  iteration yielding zero items and remaining empty on construction
  (the wiki "WavPack file consists of blocks" sentence is plural but
  the empty file is treated as zero blocks rather than an error);
  single-block iteration yielding one Ok then terminating with
  `is_exhausted() == true`; three back-to-back identical empty
  blocks yielding three Ok items in order; `remaining()` shrinking
  by exactly `8 + ck_size` per successful `next()` call (matching
  the wiki "Block structure" on-disk extent definition); the
  fused-on-error contract — first block Ok, second block with
  corrupt magic → `Err(InvalidMagic)`, then `None` on every
  subsequent call with the iterator's `remaining()` slice still
  pointing at the malformed block's first byte for offset
  recovery; `CkSizeExceedsBuffer { ck_size: 200, available: 32 }`
  surfacing on a second block whose header advertises a
  longer payload than the buffer carries; `Truncated` surfacing on
  a partial header (sub-`HEADER_LEN` tail) between blocks — the
  round-13 split between "buffer ran out mid-payload" and "buffer
  ran out between blocks" preserved across the iterator boundary;
  [`BlockIter::new`] and [`iter_blocks`] returning identical
  sequences; [`parse_blocks`] returning a `Vec<WavPackBlock>` in
  the same order as iteration on a synthesised three-block stream
  with distinct `block_samples` (`100` / `200` / `300`),
  bubbling the first error rather than the partial Vec on a
  malformed second block, and returning an empty Vec on an empty
  input; [`block_count`] returning the matching count across a
  five-block stream and bubbling the first error verbatim;
  [`total_block_samples`] summing the wiki "samples in this block"
  field across a three-block list (`100 + 200 + 300 = 600`),
  returning `0` on the empty slice, and the u64 return type
  preventing the `u32::MAX + u32::MAX` two-block sum from
  overflowing — i.e. confirming the file-scale sample count
  withstands a 4-GiB-plus stream; and a final equivalence check
  proving [`parse_blocks`] is observationally identical to
  manually draining [`iter_blocks`] on the same input.

- Round 214 — block-level discovery / accessor sweep on
  [`WavPackBlock`], surfacing the typed views the existing free
  finders already build over `self.sub_blocks` without making the
  caller reach through `block.sub_blocks()`. New header passthroughs:
  [`WavPackBlock::flags`] borrows `&self.header.flags`,
  [`WavPackBlock::block_samples`] / [`WavPackBlock::block_index`]
  return the wiki "samples in this block" / "offset in samples for
  current block" fields, and [`WavPackBlock::is_audio_block`] lifts
  the round-1 [`WavPackBlockHeader::is_audio_block`] predicate to the
  block level so a multi-block iterator can filter without reaching
  through the header. New presence predicates:
  [`WavPackBlock::has_entropy_info`] /
  [`WavPackBlock::has_packed_samples`] /
  [`WavPackBlock::has_md5_checksum`] /
  [`WavPackBlock::has_riff_header`] /
  [`WavPackBlock::has_riff_trailer`] /
  [`WavPackBlock::has_multichannel_info`] keyed on
  [`SubBlockId`] equality. New borrow finders:
  [`WavPackBlock::find_sub_block`] /
  [`WavPackBlock::find_entropy_info_sub_block`] /
  [`WavPackBlock::find_md5_checksum_sub_block`] /
  [`WavPackBlock::find_multichannel_info_sub_block`] /
  [`WavPackBlock::find_riff_header_sub_block`] /
  [`WavPackBlock::find_riff_trailer_sub_block`] each pair with the
  corresponding `has_*` predicate and return an
  `Option<&MetadataSubBlock<'a>>` borrow. New typed extractors:
  [`WavPackBlock::packed_samples`] returns
  `Option<PackedSamples<'a>>` (the round-12 typed view over the
  `0x0A` payload); [`WavPackBlock::entropy_info`] returns
  `Result<Option<EntropyInfo>>` (the round-4 expander wrapped so
  missing `0x05` reports `Ok(None)` rather than an error, and a
  malformed `0x05` propagates the existing
  [`Error::EntropyInfoLength`]); [`WavPackBlock::md5_checksum`]
  returns `Result<Option<Md5Checksum>>` with the same shape
  (missing → `Ok(None)`, malformed → `Error::Md5ChecksumLength`).
  These accessors compose with the round-206 decode loop: a typical
  pre-flight call sequence is `block.is_audio_block()` +
  `block.has_entropy_info()` + `block.has_packed_samples()` +
  `block.decode_samples()`, all on a borrowed
  [`WavPackBlock`]. Round 214 adds no new error variants and does
  not touch any docs-gap-blocked surface; the round-3 decorrelation
  expanders still lack a prediction-loop consumer and the
  median-adaptation amount remains the open docs gap for stateful
  Golomb refinement (still wired only through the round-15/199
  per-sample loop and the round-201/206 composers).
- 24 new unit tests (319 total): [`WavPackBlock::flags`] pin against
  `self.header.flags.raw` round-trip with the `mono` bit asserted;
  [`WavPackBlock::block_samples`] / [`WavPackBlock::block_index`]
  match the header values (with an explicit 12345 block_index
  patched into the synthesised header bytes); the
  [`WavPackBlock::is_audio_block`] predicate symmetric with
  [`WavPackBlockHeader::is_audio_block`] across the `block_samples`
  zero / non-zero boundary; the six `has_*` presence predicates each
  fire on the matching sub-block ID and clear on a metadata-empty
  block; [`WavPackBlock::find_sub_block`] returning the first
  matching `&MetadataSubBlock<'a>` borrow with payload pin and
  reporting `None` for an absent ID; each specialised borrow finder
  ([`find_entropy_info_sub_block`] / [`find_md5_checksum_sub_block`]
  / [`find_multichannel_info_sub_block`] /
  [`find_riff_header_sub_block`] / [`find_riff_trailer_sub_block`])
  returning the present / absent shapes with payload pins;
  [`WavPackBlock::packed_samples`] returning a typed
  [`PackedSamples`] view with byte-array round-trip and `None` on
  absence; [`WavPackBlock::entropy_info`] decoding a mono `0x05`
  payload to a typed [`EntropyInfo`] via the explicit-exponent
  log-pack path (`[mantissa_lo, 0x09]` → `median = mantissa`)
  returning `Ok(Some([5, 3, 7]))`, returning `Ok(None)` on a
  block with no `0x05`, and propagating
  `Err(Error::EntropyInfoLength(8))` for a malformed 8-byte
  payload; [`WavPackBlock::md5_checksum`] decoding the standard
  "empty input" digest
  (`d41d8cd98f00b204e9800998ecf8427e`) as a pinned test vector,
  returning `Ok(None)` on an absent `0x26`, and propagating
  `Err(Error::Md5ChecksumLength(8))` for a wrong-length payload; and
  an end-to-end pairing test confirming the round-214 accessors and
  the round-206 [`decode_samples`] composer compose on a single
  block with `0x05` + `0x0A` + `0x26` returning the expected `[0]`
  PCM sample alongside `Some` entropy / md5 / packed-samples typed
  views and `false` for the un-present `has_riff_*` /
  `has_multichannel_info` / `has_decorrelation` predicates.

- Round 206 — block-level `WavPackBlock::decode_samples()` composer
  turning the round-13 [`parse_block`] aggregate into PCM samples in
  one call. Chains [`find_entropy_info`] + [`expand_entropy`] +
  [`find_packed_samples`] through the round-201
  `decode_packed_samples_mono_from_entropy` /
  `decode_packed_samples_stereo_from_entropy` wrappers depending on
  the new [`Flags::is_block_data_mono`] accessor (the union of wiki
  bit 2 `mono` and wiki bit 30 `false_stereo`). Returns a `Vec<i32>`
  of `block_samples` mono samples or `block_samples * 2` interleaved
  stereo samples. New [`UnsupportedBlockFeature`] enum names the
  seven WavPack v.4 features the per-sample loop does not yet
  support (`Hybrid` lossy profile / `FloatData` / `Int32Mode` /
  `MultichannelMember` / `Decorrelation` / `LowLatencyBlock` /
  `RobustBlock`); each is surfaced through the new
  `Error::UnsupportedBlockFeature(feature)` variant with a Display
  impl that names the responsible wiki flag bit or sub-block ID.
  New structural errors `Error::BlockHasNoAudio` /
  `Error::BlockMissingEntropyInfo` /
  `Error::BlockMissingPackedSamples` cover the three "block doesn't
  carry what the composer needs" shortfalls. New
  [`WavPackBlock::has_decorrelation`] predicate detects the presence
  of any of the `0x02` / `0x03` / `0x04` decorrelation sub-blocks so
  the composer refuses pre-pass blocks without trying to walk the
  round-3 typed views (which exist but lack a prediction-loop
  consumer). New [`Flags::is_block_data_stereo`] is the inverse of
  `is_block_data_mono`. The composer is **stateless**: each call
  seeds a fresh [`AdaptiveMedians`] from the block's `0x05` payload
  and drops it on return, matching how real `.wv` files carry a
  fresh `0x05` seed per block.
- 23 new unit tests (295 total): a mono one-sample happy path
  (seed `[0,0,0]` + 0x0A `0x00` byte payload → `[0]` via the spec
  §4.2 step 1 zero-run path emitting a single zero sample); the
  matching stereo happy path with minimal-non-zero seeds (`[1,0,0]`
  on both channels — non-zero so `EntropyInfo::is_mono()` reports
  stereo, but still `get_med(0) == 1` so the spec §4.2 step 1 fast
  path stays eligible) yielding `[0, 0]`; the false-stereo dispatch
  (bit 30 set, bit 2 clear, mono-layout `0x05` → mono decode loop);
  `BlockHasNoAudio` on `block_samples == 0`;
  `BlockMissingEntropyInfo` on a block with only `0x0A`;
  `BlockMissingPackedSamples` on a block with only `0x05`; each of
  the seven `UnsupportedBlockFeature` variants triggered by the
  matching flag bit or sub-block presence (`Hybrid` / `FloatData` /
  `Int32Mode` / `MultichannelMember` / `Decorrelation`
  exercised through each of `0x02` / `0x03` / `0x04` /
  `LowLatencyBlock` / `RobustBlock`); the `EntropyInfoLength`
  propagation from a malformed 8-byte `0x05` payload;
  `WavPackBlock::has_decorrelation` firing on each of the three
  sub-block IDs and clearing when none is present; the four
  `is_block_data_mono` / `is_block_data_stereo` arms (plain mono,
  false stereo, plain stereo, mono + false-stereo); and the
  `UnsupportedBlockFeature` Display strings naming the relevant
  wiki bit / sub-block ID for each variant.
- Round 201 — `EntropyInfo` → `AdaptiveMedians` channel-indexed bridges
  and end-to-end `from_entropy` wrappers for the mono and stereo
  `0x0A` decode loops, removing the round-15/199 caller's hand-rolled
  per-channel seed extraction. New `EntropyInfo::stereo(left, right)`
  symmetric constructor (counterpart to the existing `EntropyInfo::mono`).
  New `AdaptiveMedians::from_entropy(info, channel_idx)` returning
  `Some(state)` for channel 0 on any payload and channel 1 on a stereo
  payload (the same shape as `Medians::from_entropy`), with `None` for
  out-of-range indices, channel 1 on mono, and negative seeds (defensive
  `i32 → u32` reject). New `AdaptiveMedians::stereo_pair_from_entropy(info)`
  returning the `[left, right]` array `decode_packed_samples_stereo`
  takes (returns `None` on mono or any negative seed). New top-level
  `decode_packed_samples_mono_from_entropy(payload, info, count)` and
  `decode_packed_samples_stereo_from_entropy(payload, info, frames)`
  wrappers that compose the bridges with the round-15/199 stateful
  loops, surfacing a new `Error::InvalidEntropyInfoForMono` /
  `Error::InvalidEntropyInfoForStereo` for the malformed-input arm.
  20 new tests (272 total): the `EntropyInfo::stereo` constructor
  populating both sets and matching its struct-literal form (plus the
  is_mono-by-content nuance with a zero right set); the
  `AdaptiveMedians::from_entropy` bridge across channel 0 / 1 / out-of-
  range indices, mono right-channel rejection, and negative-seed
  rejection on both channels; the `stereo_pair_from_entropy` mono
  rejection and per-channel negative-seed rejection; the
  `_from_entropy` wrappers proving byte-identical reconstruction to
  the explicit-seed calls, the malformed-input error paths firing
  before any bits are read, and the zero-count / zero-frame vacuous
  cases.
- Round 199 — stereo per-sample `0x0A` decode loop wiring the
  `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §2 channel-
  alternation rule on top of the round-15 mono loop. New
  `StereoDecodeState` bundles two per-channel `RunState`s (`left_run`,
  `right_run`) for the spec §4.2 step 4 holding-bit fold applied
  per-channel, a stream-level `zero_run_pending` counter and
  `ever_took_zero_run` sticky bit for the spec §4.2 step 1 zero-run
  fast path (which the spec specifies as stream-level: gated on BOTH
  channels' `median[0] <= 1` AND BOTH channels' holding state empty,
  resetting BOTH channels' medians on a non-zero run), and a
  `next_channel` parity cursor (`0` = left, `1` = right) that toggles
  only on a successful emit so a `?`-bubbled error leaves the cursor
  recoverable. New `decode_sample_stateful_stereo(reader, &mut
  [AdaptiveMedians; 2], &mut StereoDecodeState)` walks the spec §4.2
  sequence for one stereo sample, dispatching reads + adaptation +
  mantissa + sign to `medians[next_channel]` only (the other channel's
  state stays untouched). New `decode_packed_samples_stereo(payload,
  &mut [AdaptiveMedians; 2], frames)` returns a `Vec<i32>` of
  `frames * 2` interleaved (L,R,L,R,…) PCM samples and is the first
  public stereo end-to-end PCM-producing API on the crate. Pair with
  `EntropyInfo::medians_for_channel(0/1)` + `AdaptiveMedians::from_seed_values`
  on the round-4 expander's left + right median sets to seed.
- 13 new unit tests (252 total): simulator-driven zone-1 round-trip
  across 8 stereo frames with matching seeds; the same with distinct
  per-channel seeds proving per-channel dispatch never crosses
  medians; mixed zones (zone 1 / zone 2 / zone 2 overflow) per channel
  with a per-channel adapt simulator picking magnitudes against the
  current medians; negative-sign reconstruction with magnitude held in
  zone 1 by picking `signed_value = -(get_med(0) + 2)` (so the
  decoded `!signed_value` magnitude lands at `get_med(0) + 1`); the
  end-to-end `decode_packed_samples_stereo` loop wrapper matching the
  per-call sequence bit-for-bit AND finishing with identical per-
  channel medians; the stereo zero-run path (a `1101` wire bit-string
  decoding to run_length = 3) zeroing BOTH channels' medians on entry,
  emitting `0` on the current channel, draining the remaining two
  zero samples across alternating channels without consuming bits;
  the BOTH-channel zero-run gate rejecting one-channel eligibility
  (left zeroed, right at `[256, 256, 256]` → fast path off, normal
  prefix path fires and reports Truncated on an empty buffer); the
  stereo truncation path leaving `next_channel` at `0` on error; the
  EOF escape (`LIMIT_ONES + cbits == 33` wire bit-string) surfacing
  `Error::EndOfStream` with `next_channel` still at `0`; per-channel
  holding-state independence (a left-channel `last_zero` short-
  circuit doesn't touch the right channel's state); the empty-
  payload `decode_packed_samples_stereo` rejecting with Truncated
  AND leaving the medians unchanged; the `frames == 0` vacuous case
  returning an empty `Vec`; and the `StereoDecodeState::default()`
  matching `StereoDecodeState::new()` with per-channel `RunState`s
  also matching `RunState::new()`.
- Round 15 — stateful per-sample `0x0A` decode loop wiring the newly
  staged `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §3 + §3.2
  + §4.2 into the end-to-end decode path. New `decode_sample_stateful`
  primitive runs the full spec §4.2 sequence per sample: optional
  spec §4.2 step 1 zero-run fast path (gated on `get_med(0) <= 1` and
  no holding bits, owing `run_length - 1` zero samples across
  subsequent calls via a new `DecodeState::zero_run_pending` field);
  spec §4.2 steps 2 + 3 unary prefix with the `LIMIT_ONES = 16`
  escape and the `cbits == 33` EOF marker (new `Error::EndOfStream`);
  spec §4.2 step 4 holding-bit fold (re-using the existing wiki-
  compressed `RunState`); spec §4.2 step 5 `(low, high)` interval
  with 31-bit mask and `high >= low` clamp; spec §3.2 per-zone
  `AdaptiveMedians::adapt(Zone::from_ones_count(ones_count))` BEFORE
  the mantissa read; spec §4.2 step 6 first paragraph truncated-binary
  mantissa decode (`maxcode = high - low`, `bitcount` = bit-length of
  `maxcode`, `extras = (1 << bitcount) - maxcode - 1`, short
  `bitcount - 1`-bit form vs long `bitcount`-bit phase-in); spec §4.2
  step 7 sign bit returning `~mid` or `mid`. New
  `decode_packed_samples_mono(payload, &mut medians, count)` wraps the
  per-sample primitive into an end-to-end mono single-block loop and
  is the first public API on the crate that produces a `Vec<i32>` of
  PCM samples from a `0x0A` payload. New constants `ESCAPE_EOF_CBITS
  = 33`, `RUN_ESCAPE_CAP = 33` and `INTERVAL_MASK_31 = 0x7fff_ffff`
  spell out the spec §4.2 step 3 EOF marker, the spec §4.2 step 1
  zero-run unary cap and the spec §4.2 step 5 interval mask.
- Round 15 — 18 new tests covering the stateful decoder via a
  spec-derived inverse encoder (`encode_one_sample`,
  `emit_truncated_binary`, `pick_raw_unary`, `pick_zone_for_magnitude`
  test helpers) for bit-exact closure across zones 0 / 1 / 2 /
  overflow, negative-sign reconstruction, a hand-traced 2-sample
  fixture pinning the spec §4.2 step 1-7 sequence by literal bit
  string, the zero-run fast path engaging and resting eligibility,
  the `LIMIT_ONES`+`cbits=33` EOF escape, a 64-sample adaptive-median
  drift sequence using a deterministic value-from-current-medians
  simulator, the `decode_packed_samples_mono` end-to-end loop, the
  `read_truncated_binary` primitive round-trip across every code in
  a `maxcode = 16` interval, and the `form_interval` spec §4.2 step 5
  ladder for the four named zones.

## [0.0.2](https://github.com/OxideAV/oxideav-wavpack/releases/tag/v0.0.2) - 2026-05-30

### Other

- Round 14: median-adaptation amount (spec §3 + §3.2) — AdaptiveMedians + Zone
- Round 13: end-to-end parse_block aggregate + BitReader peek/skip primitives
- Round 12: 0x0A PackedSamples typed view + BitReader position accessors + channel-indexed EntropyInfo bridges
- Round 11: per-term decorrelation sample-count helper + flat-payload partitioner
- Round 10: MD5 typed view + walker finders + remaining metadata-kind predicates
- Round 9: term-kind classifier + decorrelation/metadata kind accessors
- round-8 block-header accessor coverage (lossless / sample-rate sentinel / experimental / effective bit-depth / audio-block / payload-bytes)
- Round 7: single-call per-sample decode + EntropyInfo→Medians bridge
- Golomb (base, add) interval + sample-value reconstruction (round 6)
- Round 5: sample-coding bit reader + run-length decoder
- Round 4: 0x05 entropy-info sub-block expander
- Round 3: decorrelation sub-block expanders (terms / weights / samples)
- Round 2: metadata sub-block walker
- Round 1: WavPack v.4 block-header parser
- Round 0 — clean-room rebuild scaffold (orphan master)

### Added

- Round 14 — WavPack median-adaptation amount (newly-unblocked spec
  `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §3 + §3.2). New
  `AdaptiveMedians` struct — a three-`u32` running median state with
  the spec §2.1 4-fractional-bit encoding — plus the integer
  increment / decrement primitives that the spec §3 quotes:
  `((median[i] + D) / D) * 5` up and `((median[i] + (D - 2)) / D) * 2`
  down, with `D` = `DIV0` / `DIV1` / `DIV2` (= `128` / `64` / `32`)
  per the spec table. New `Zone` enum names the four §3.2 arms
  (`Zone0` / `Zone1` / `Zone2` / `Zone2Overflow { ones_count }`)
  driven from a raw `ones_count` via `Zone::from_ones_count`, and
  `AdaptiveMedians::adapt(zone)` / `adapt_for_ones_count(ones_count)`
  apply the correct combination of `inc_median` / `dec_median` calls:
  zone 0 → dec `median[0]`; zone 1 → inc `median[0]`, dec `median[1]`;
  zone 2 → inc `median[0]` + `median[1]`, dec `median[2]`; zone 2
  overflow → all three inc. New `AdaptiveMedians::get_med(i)` returns
  the spec §2.1 working median `(median[i] >> 4) + 1` (the value the
  spec §4.2 interval ladder consumes). New seed constructors
  `AdaptiveMedians::from_seed_values([i32; 3])` and
  `AdaptiveMedians::from_medians(Medians)` bridge the round-4 / round-6
  typed views into the running state, returning `None` when any seed
  is negative (rather than silently casting). New public constants
  `DIV0` / `DIV1` / `DIV2` / `MEDIAN_INC_MULTIPLIER` /
  `MEDIAN_DEC_MULTIPLIER` / `GET_MED_SHIFT` / `GET_MED_FLOOR` record
  the spec §3 / §5 numeric facts. The `inc_median` / `dec_median`
  primitives use `saturating_add` / `saturating_sub` / `saturating_mul`
  defensively against pathological starting values — the spec §3
  arithmetic naturally stays within `u32` for any state the per-sample
  decode actually produces, but the saturating cap prevents a stray
  caller from triggering UB. None of this is wired into the round-7
  `decode_sample` call yet — that composition is gated on a follow-up
  round so the existing `Medians`-by-value primitive is preserved
  unchanged and the new `AdaptiveMedians` state is an additive,
  self-contained numeric primitive.
- 24 new unit tests (221 total): divisor / multiplier / GET_MED
  constants matching the spec §3 / §5 table; `get_med` returning the
  §2.1 floor (`1`) at zero, stripping the 4 fractional bits at the
  exemplar values 16/32/48, and truncating (not rounding) on the
  off-boundary 15/31/47 case; `inc_median` stepping by `5` at
  `median = 0` and by `10` at `median = D`, per-index divisor
  selection on indices 1/2; `dec_median` stepping to zero at
  `median = 0` and by `2` at `median = D`, plus an exhaustive sweep
  proving `((v + D - 2) / D) * 2 <= v` for `v ∈ 0..=200` (the §3
  "never below 0" invariant); a 5:2 ratio equilibrium probe at
  `median = 256` cross-checking both the inc step (`15`) and the
  follow-up dec step (`6`); `Zone::from_ones_count` mapping every
  named arm plus the `ones_count >= 3` overflow with the raw value
  preserved (including `3` / `33` / `u32::MAX`), and the round-trip
  through `Zone::ones_count`; `adapt(zone)` matching the §3.2 table
  exactly on each of the four arms (Zone0 dec m0, Zone1 inc m0 + dec
  m1, Zone2 inc m0 + m1 + dec m2, Zone2Overflow inc all three) with
  before/after equality against the primitive sequence;
  `adapt_for_ones_count` threading through to the same Zone branch
  for `ones_count = 1` and `ones_count = 7`; `from_seed_values`
  accepting non-negative seeds and rejecting any negative slot;
  `from_medians` bridging a `Medians` and rejecting a negative slot;
  a four-step §3 sequence (Zone1 → Zone0 → Zone2 → Zone2Overflow)
  walking a fresh `[0,0,0]` state through hand-computed `[5,0,0]` →
  `[3,0,0]` → `[8,5,0]` → `[13,10,5]` confirming every step matches
  the spec arithmetic exactly; saturating semantics on `u32::MAX`
  increment and `0` decrement; and `AdaptiveMedians` is `Copy` /
  `PartialEq` (pre-update vs post-update inequality on a
  `Zone0`-driven decrement).
- Round 13 (end-to-end `parse_block` aggregate + `BitReader` non-mutating
  look-ahead + bulk `skip_bits`): another non-prediction-loop advancement
  while the median-adaptation amount stays a docs gap. New stand-alone
  `parse_block(bytes) -> Result<(WavPackBlock<'_>, &[u8])>` composes
  round-1 `parse_block_header` and round-2 `walk_metadata` into a single
  end-to-end call: parses the 32-byte fixed header, validates that the
  input carries the `8 + ck_size` bytes the wiki "Block structure"
  listing declares, walks the metadata sub-block region against the
  exact byte count `ck_size` advertises (rather than the whole tail),
  and returns the typed `WavPackBlock` aggregate plus the unconsumed
  tail (i.e. the next block in a multi-block `.wv` file). New
  `WavPackBlock<'a>` carries the typed `WavPackBlockHeader` (round 1)
  alongside a `Vec<MetadataSubBlock<'a>>` (round 2; borrowed payload
  slices into the input bytes). New accessors `WavPackBlock::header`,
  `sub_blocks`, `contains_sub_block(id)` (boolean shortcut over
  `find_first` for presence checks), `sub_block_count`,
  `is_metadata_empty` (the `ck_size == 24` header-only edge case the
  wiki allows when `block_samples == 0`), and `on_disk_len` (the
  `8 + ck_size` on-disk extent in bytes — useful for callers stepping
  across blocks without re-parsing the header). New error variant
  `Error::CkSizeExceedsBuffer { ck_size, available }` distinguishes
  "header parses but payload is short" from `Error::Truncated`
  (header-boundary truncation) so a streaming caller can size the next
  read against `8 + ck_size - available`. None of these read sample
  bits, mutate state, or touch the prediction loop; they compose the
  round-1 and round-2 parsers into the one-call surface a streaming
  caller wants.
- New `BitReader` non-mutating look-ahead primitives `peek_bit()` /
  `peek_bits(count)` / `peek_unary()` — read a single bit, a multi-bit
  value, or a unary run-length without advancing the cursor.
  Implemented by reading from a clone of the reader, so the LSB-first
  bit-order rules in `get_bit` / `get_bits` / `get_unary` carry through
  unchanged. On `Error::Truncated` the original reader's cursor is
  unchanged (the truncation hit the clone, not the original) so a
  caller can retry against a freshly-extended buffer without rebuilding
  the reader. Useful for probing the wiki `n == 16` escape pattern (the
  leading unary indicating whether a second unary follows) before
  committing to a real `decode_run_length` call. New bulk
  `BitReader::skip_bits(count)` advances the cursor by `count` bits
  without assembling a `u32`; on `Truncated` the cursor lands at the
  buffer end (matching `get_bits`' partial-consume semantics).
- 24 new unit tests (197 total): `parse_block` returning header + empty
  metadata on `ck_size == 24`, walking a two-sub-block metadata region
  (dummy + MD5) and confirming both walker entries plus the
  `contains_sub_block` / `is_metadata_empty` predicates, chaining two
  back-to-back blocks through the returned tail, rejecting a sub-
  `HEADER_LEN` buffer with `Truncated`, surfacing the new
  `CkSizeExceedsBuffer { ck_size, available }` variant on a header
  advertising a longer payload than the buffer (with both fields
  asserted), propagating `InvalidMagic` / `InvalidCkSize` from the
  header parser, propagating `Truncated` from the metadata walker on a
  malformed sub-block, `on_disk_len` equalling `8 + ck_size` and the
  underlying byte count, `contains_sub_block` returning false on
  header-only blocks, and `sub_block_count` matching the walker output
  count on a four-sub-block block; `peek_bit` returning the next
  LSB-first bit without advancing (cursor stays put; follow-up
  `get_bit` returns the same value), `peek_bit` `Truncated` on an
  empty buffer leaving the cursor untouched, `peek_bits` assembling
  4 LSB-first bits of `0x0A` into `0xA` without advancing,
  `peek_bits(0)` returning zero without advancing, `peek_bits(9)` on
  an 8-bit buffer reporting `Truncated` with the cursor unchanged,
  `peek_unary` matching `get_unary` on the wiki `111110b → 5` example
  without advancing, `peek_unary` reporting `Truncated` on an
  unterminated run with the cursor unchanged, the peek-then-get
  pattern returning matching values across a 4-bit window;
  `skip_bits` advancing the cursor without assembling a value (with
  the expected `bits_consumed` / `byte_position` / `bit_position`
  after a 5-bit skip and the next `get_bits(3)` reading the remaining
  bits), `skip_bits(0)` no-op, a 10-bit cross-byte skip landing at
  `byte_position == 1` / `bit_position == 2`, `skip_bits(9)` on an
  8-bit buffer reporting `Truncated` with the cursor at the buffer
  end, and a `skip_bits`-then-`get_unary` resume reading the second
  of two back-to-back unary runs.
- Round 12 (`0x0A` packed-samples typed view + `BitReader` position
  accessors + channel-indexed `EntropyInfo` / `Medians::from_entropy`
  bridges): another non-prediction-loop advancement while the
  median-adaptation amount stays a docs gap. New `PackedSamples<'a>`
  typed view of the `0x0A` packed-samples sub-block payload — the
  entropy-coded audio bitstream the wiki "Samples coding" section
  consumes — exposing `bytes()` / `len()` / `is_empty()` introspection
  and a `bit_reader()` factory that produces a fresh `BitReader`
  positioned at bit 0 for feeding `decode_run_length` /
  `decode_sample_value` / `decode_sample`. New stand-alone
  `expand_packed_samples(payload: &[u8]) -> PackedSamples<'_>` mirrors
  the round-3/round-4 expanders' naming (typed wrap rather than a
  byte-by-byte decode because the wiki places no internal structure on
  the `0x0A` payload). New walker finder
  `find_packed_samples(&[MetadataSubBlock]) -> Option<PackedSamples>`
  is the typed counterpart to `find_audio_payload`. New cursor
  accessors `BitReader::byte_position()` / `bit_position()` /
  `bits_consumed()` name the reader's position in the underlying byte
  slice, with `bits_consumed` clamping at the buffer length when the
  reader has advanced past the end. New `EntropyInfo::is_stereo()` /
  `channels()` / `medians_for_channel(idx)` typed channel introspection
  pinning the wiki "one or two sets of medians" sentence as `1` or `2`
  populated sets, with a channel-indexed median getter returning
  `Some([m0, m1, m2])` for `0` (left/mono) or `1` (right, stereo only)
  and `None` for `1` on a mono payload and indices `>= 2`. New
  `Medians::from_entropy(info, channel_idx)` channel-indexed bridge
  with the same mono / out-of-range guards so callers iterating
  per-channel medians skip the hand-rolled mono / stereo branch. None
  of these read bits, mutate state or touch the prediction loop; they
  elaborate the round-2 walker output and the round-4 / round-5/6/7
  expanders for callers staging the deferred decode pass.
- 23 new unit tests (173 total): `PackedSamples` constructor
  round-tripping a non-empty payload, the zero-byte empty payload
  accepted and reported empty, `expand_packed_samples` round-tripping
  the byte slice, `bit_reader()` starting at byte/bit 0 with the full
  payload remaining and yielding the first bit LSB-first, the
  `bit_reader()` factory returning independent readers (so a probe and
  the real decode each start at bit 0), an empty packed-samples view
  reporting immediate `Error::Truncated` on any read, and the view
  being `Copy`; `BitReader::byte_position` / `bit_position` /
  `bits_consumed` tracking 13-bit consumption across a byte boundary,
  the `bits_consumed` clamp when the reader is past the end, and the
  cursor staying put when a read errors with `Truncated`;
  `Medians::from_entropy` yielding the left set on index `0` and the
  right set on index `1` for a stereo `EntropyInfo`, returning `None`
  for `1` on a mono `EntropyInfo`, and rejecting out-of-range indices
  (`2`, `3`, `255`); `EntropyInfo::is_stereo` inverting `is_mono`,
  `channels` returning `1` for mono and `2` for stereo,
  `medians_for_channel` yielding the matched set for `0` / `1` on
  stereo and `None` for `1` on mono / `2+` indices; and
  `find_packed_samples` returning a typed `PackedSamples` view over
  the synthesised `0x0A` payload and `None` on a stream without one.
- Round 11 (per-term decorrelation-sample-count helper + flat-payload
  partitioner): another non-prediction-loop advancement while the
  median-adaptation amount stays a docs gap. New stand-alone
  `decorrelation_sample_count(code: i8) -> Option<u8>` and matching
  `TermKind::decorrelation_sample_count()` method derive the per-term
  `0x04` seed-sample count from the wiki "Possible predictor values"
  listing: `Some(code - 5)` for the `6..=12` sample-based predictors
  (one per previous-sample slot the wiki cites in "uses 1-7 samples
  for prediction") and `Some(2)` for the `17..=18` two-sample
  predictors. Stereo predictors `0..=5`, the reserved `13..=16`
  range, and codes outside `0..=18` all return `None` — the wiki does
  not give a per-term count for them. New public constant
  `MAX_DECORRELATION_SAMPLES_PER_TERM = 16` records the wiki
  "Decorrelation samples" upper bound ("Each decorrelation term may
  have up to 16 samples depending on its value") for callers checking
  future docs additions against it.
- New `DecorrelationTerms::expected_decorrelation_sample_count()`
  sums the per-term counts above across a `(0x02)` term list and
  returns `Some(total)` when every term is documented, `None` as
  soon as any term lands in the docs gap. An empty term list returns
  `Some(0)` (vacuous).
- New `partition_decorrelation_samples(&DecorrelationTerms,
  &DecorrelationSamples) -> Result<Vec<Vec<i32>>>` splits the flat
  sample list `expand_samples` produces into one `Vec<i32>` per term
  in wiki order, with the per-term length given by the new
  `decorrelation_sample_count` helper. Two new `Error` variants
  back it: `DecorrelationSampleCountUnspecified(i8)` carries the
  offending term code when a term has no wiki-documented per-term
  count (so partitioning cannot proceed), and
  `DecorrelationSampleCountMismatch { expected, actual }` fires
  when the summed expected count does not equal the flat payload
  length. None of these read bits, mutate state, or advance the
  prediction loop; they elaborate the round-3 expander output for
  the documented predictor codes and explicitly refuse the
  docs-gap codes rather than guessing a per-term length.
- 15 new unit tests (150 total): `decorrelation_sample_count`
  returning the matching `code - 5` across the full `6..=12`
  sample-based range, `Some(2)` for `17` / `18`, and `None` across
  stereo `0..=5`, reserved `13..=16`, undocumented `19..=31`, and
  the negative-code defensive case; `MAX_DECORRELATION_SAMPLES_PER_TERM`
  wiki upper-bound sanity sweep across every documented count;
  `TermKind::decorrelation_sample_count` mirror of the stand-alone
  helper; `DecorrelationTerms::expected_decorrelation_sample_count`
  summing `[6, 8, 17, 12]` to `13`, the vacuous empty-list `0`, and
  `None` propagation across stereo / reserved / undocumented term
  presence; `partition_decorrelation_samples` splitting a
  `[6, 8, 17]` term list with matching flat input in term order,
  the empty-terms-empty-payload base case, rejecting the stereo
  `[2]` and reserved `[6, 14]` lists with
  `DecorrelationSampleCountUnspecified`, rejecting both short
  (`expected: 6, actual: 5`) and long (`expected: 1, actual: 4`)
  flat payloads with `DecorrelationSampleCountMismatch`, and a
  round-trip from `expand_samples` of a synthesised `[6, 18]`
  exponent-9 wire through the partitioner back to per-term
  `[1]` + `[2, 3]` lists.
- Round 10 (metadata sub-block extras: MD5 typed view + walker finders
  + remaining payload-kind predicates + `SubBlockId` classifiers):
  another non-prediction-loop advancement while the median-adaptation
  amount remains a docs gap. New `Md5Checksum([u8; 16])` typed view of
  the `0x26` sub-block payload — the wiki "16-byte MD5 sum of raw audio
  data" — together with `parse_md5_checksum(&[u8]) -> Result<Md5Checksum>`
  enforcing the fixed 16-byte length and a new
  `Error::Md5ChecksumLength(usize)` variant rejecting other lengths.
  Public constant `MD5_DIGEST_BYTES = 16`. New walker finders over the
  round-2 `Vec<MetadataSubBlock>`: `find_first(subs, id)` linear scan
  by `SubBlockId`; specialised wrappers `find_audio_payload` (`0x0A`),
  `find_entropy_info` (`0x05`), `find_md5_checksum_block` (`0x26`),
  `find_multichannel_info` (`0x0D`); and `find_decorrelation_triple`
  which returns the three sub-blocks `(0x02, 0x03, 0x04)` in wiki order
  or `None` if any one is missing. Eight new `MetadataSubBlock`
  payload-kind predicates filling out the wiki "IDs" listing:
  `is_dummy_payload` (`0x00`), `is_hybrid_profile_payload` (`0x06`),
  `is_float_payload` (`0x08`), `is_int32_payload` (`0x09`),
  `is_overflow_bits_payload` (`0x0C`), `is_multichannel_info_payload`
  (`0x0D`), `is_encoding_details_payload` (`0x25`), `is_md5_payload`
  (`0x26`), `is_sample_rate_payload` (`0x27`). Four new `SubBlockId`
  classifiers — `is_decorrelation`, `is_correction_stream`,
  `is_riff_wrapper`, `is_audio` — for callers that branch on the ID
  enum directly rather than on a sub-block. None of these read bits,
  mutate state, or touch the prediction loop; they elaborate the
  round-2 walker output for callers staging the deferred decode pass.
- 17 new unit tests (135 total): `SubBlockId` classifier coverage
  across all four buckets (decorrelation `0x02/0x03/0x04`,
  correction-stream `0x07/0x0B`, RIFF wrapper `0x20/0x21` with
  same-family `0x25/0x26/0x27` negative cases, audio-only `0x0A`);
  one-hot kind-predicate sweep across the eight new MetadataSubBlock
  predicates (with the four "main bucket" predicates pinned false on
  each); `is_md5_payload` discriminating `0x06` HybridProfile from
  `0x26` Md5Checksum on the low-5-bit overlap; `is_dummy_payload`
  discriminating `0x00` Dummy from `0x20` RiffHeader on the same
  overlap; `parse_md5_checksum` accept (MD5 of `""` test vector) and
  reject (0/15/17/64-byte rejections); end-to-end round-trip from a
  synthesised `0x26` sub-block through `walk_metadata` → `find_md5_*`
  → `parse_md5_checksum` (MD5 of the "quick brown fox" test vector);
  walker finder coverage for `find_first` (hit + miss), the four
  specialised finders, and `find_decorrelation_triple` (full triple
  hit + miss when either of weights / samples is dropped).
- Round 9 (decorrelation-term classification + metadata-payload kind
  predicates): non-prediction-loop advancement while the median-
  adaptation amount remains a docs gap. New `TermKind` enum classifies
  the wiki "Possible predictor values" listing (stereo `0..=5` with the
  `2..=4` implemented subset, sample-based `6..=12` with the per-code
  `previous_samples()` count = `code - 5`, reserved `13..=16`,
  two-sample `17..=18`, and `Unknown` for codes outside the documented
  range). New `TermKind::is_implemented()` / `previous_samples()`
  accessors surface the wiki's two narrowings (the stereo "only
  predictors 2-4 are implemented" subset and the per-code sample count).
  New `DecorrelationTerms` accessors: `len()`, `is_empty()`,
  `kind_at(idx)`, `iter_kinds()` (zips term code with its classified
  kind), `all_implemented()`, `has_reserved()`. New stand-alone
  `weights_per_term(channels: u8) -> u8` helper exposes the wiki
  "Each decorrelation term should have one or two weights depending on
  channels" split. None of these reads bits, mutates state, or touches
  the prediction loop; they classify the round-3 expander output for
  callers staging the deferred prediction pass.
- New `MetadataSubBlock` payload-kind predicates derived from the wiki
  "IDs" listing: `is_optional()` (wraps the `0x20` flag for callers
  branching on the sub-block value directly), `is_decorrelation_payload()`
  (`0x02` / `0x03` / `0x04`), `is_correction_payload()` (the `.wvc`
  pair: `0x07` noise-shaping profile + `0x0B` packed correction data),
  `is_audio_payload()` (`0x0A` packed samples), and
  `is_riff_payload()` (`0x20` / `0x21` RIFF header / trailer). These
  let a caller pick the decorrelation triple or the audio stream out
  of a walk without re-matching the `SubBlockId` enum.
- 15 new unit tests (118 total): `TermKind::from_code` across all four
  wiki categories — stereo implemented `2..=4`, stereo unimplemented
  `0/1/5`, sample-based `6..=12` with per-code sample count, reserved
  `13..=16`, two-sample `17..=18`, and unknown `19..=31` plus a
  negative-code defensive check; `DecorrelationTerms::len` /
  `is_empty` / `kind_at` / `iter_kinds` / `all_implemented` /
  `has_reserved` over mixed term lists including the vacuous empty
  case; `weights_per_term` matching the wiki mono / stereo split with
  a 0-channel and 3+-channel clamp; `MetadataSubBlock::is_optional`
  pinning the `0x20` bit; per-kind payload predicates round-tripping
  for `0x02` / `0x03` / `0x04` decorrelation, `0x07` / `0x0B`
  correction, `0x0A` audio, and `0x20` / `0x21` RIFF (with non-RIFF
  optional sub-block negative case).
- Round 8 (header accessor coverage): non-prediction-loop advancement
  while the median-adaptation amount remains a docs gap. Eleven
  block-header convenience accessors derived rigorously from the wiki
  "Flags meaning" / "Block structure" listings. On `Flags`:
  `is_standalone_block` / `is_multichannel_member` (degenerate-marker
  `0b11` vs everything else, exposing the wiki "multi-channel start
  and end blocks" pair); `is_lossless` / `is_lossy` (around the wiki
  bit 3 "hybrid profile (lossy compression)" label); `has_custom_sample_rate`
  (wiki bits 23..=26 sentinel `15` = "unknown/custom"); `should_skip_decode`
  (wiki bit 31 "do not decode if encountered" — bit 28 "okay to ignore"
  deliberately excluded); `is_experimental` (union of the two wiki-
  labelled experimental bits 28 + 31, diagnostic-only);
  `effective_bit_depth` (container width minus `left_shift`, saturating
  per the wiki "12-bit / 20-bit" worked examples). On `WavPackBlockHeader`:
  `is_audio_block` (wiki "may be 0 if no audio present" on `block_samples`),
  `is_total_samples_known` (sentinel-vs-real on `total_samples`), and
  `payload_bytes` (the metadata-region length advertised by `ck_size`).
  None of these touch the prediction loop or the median-adaptation
  amount — those remain gated on the open docs gap.
- 11 new unit tests (103 total): standalone vs multichannel marker
  matrix (all four `0b00..=0b11` combinations); lossless / lossy
  predicate symmetry around the hybrid bit; `has_custom_sample_rate`
  sweep across all 16 sample_rate_index values pinning the sentinel
  to `15`; `should_skip_decode` discriminating bit 31 from bit 28;
  `is_experimental` union of the two experimental bits;
  `effective_bit_depth` for the wiki 12-bit / 20-bit worked examples
  plus the no-shift baseline plus the `left_shift > container_bits`
  saturation; `is_audio_block` keyed on a non-zero `block_samples`;
  `is_total_samples_known` distinguishing the `0xFFFF_FFFF` sentinel
  from real counts (including the boundary `0`); `payload_bytes`
  subtracting the 24-byte fixed-header floor.
- Round 7: WavPack v.4 per-sample single-call decode + entropy-info →
  median bridge. New public items in the `samples` module: `decode_sample`
  fuses `decode_run_length` and `decode_sample_value` into one call per
  sample, matching the wiki's contiguous "Samples coding" pseudocode
  block (carries the adaptive `RunState`, takes `Medians` by value).
  `Medians::from_entropy_left` and `Medians::from_entropy_right` pull a
  channel's three medians directly out of a round-4 `EntropyInfo` value
  so the entropy-info expander output feeds the Golomb decoder without
  the caller re-typing the array. No median is mutated — the median
  adaptation amount remains the open docs gap blocking the multi-sample
  payload loop.
- 8 new unit tests (92 total): `Medians::from_entropy_left` and
  `from_entropy_right` for stereo input, the mono case (right set zeroed),
  `decode_sample` end-to-end through the run-length-then-value chain,
  `last_zero` short-circuit honoured (no unary bits consumed),
  degenerate-interval and truncation error propagation, and the
  `EntropyInfo` → `Medians` → `decode_sample` full chain.
- Round 6: WavPack v.4 Golomb sample-value reconstruction — the value
  part of the wiki "Samples coding" second half. New public items in
  the `samples` module: `Medians` (a channel's three `median[0..=2]` in
  wiki order, `Copy`), `GolombInterval { base, add }`, `golomb_interval`
  (pure `n` + `Medians` → `(base, add)` per the wiki's `n == 0` /
  `n == 1` / `n >= 2` branch — reads no bits, mutates no median), and
  `decode_sample_value` which picks the interval, reads `getbits(k - 1)`
  with `k = log2(add)`, applies the `if(t2 >= ex) t2 = t2*2 - ex +
  getbit()` fixup, reads the sign bit, and returns `base + t2` (or its
  ones-complement when sign is set). `k` is the **bit-length** of `add`
  — the only `log2` reading that keeps the wiki's own
  `ex = (1 << k) - add - 1` non-negative (a derivation from the wiki's
  next two lines, not an external reference; documented as a resolved
  docs gap). `decode_sample_value` takes `Medians` **by value** and does
  not mutate them: the median "increase" / "decrease" *amount* is still
  unspecified by the wiki, so the stateful loop over a whole `0x0A`
  payload is deferred. The degenerate `add == 0` interval (a median of
  `1`, where `log2(0)` / `getbits(-1)` are undefined) returns the new
  `Error::GolombDegenerateInterval(add)` rather than guessing.
- 15 new unit tests (84 total): `golomb_interval` across the three `n`
  branches (n0 → median[0], n1 → median[1], n2 → median sum with zero
  extra, large-n median[2] scaling); `golomb_k` bit-length values and a
  sweep proving `ex >= 0` across `add` 1..=1024; `decode_sample_value`
  short-mantissa (no extra bit), long-mantissa (`t2 >= ex` extra bit),
  `ex == 0` always-long branch, positive and ones-complement sign,
  `n == 0` interval, degenerate `add == 0` rejection (no bits consumed),
  mantissa- and sign-truncation reporting, and an end-to-end compose of
  `decode_run_length` then `decode_sample_value` over one contiguous
  bitstream.
- Round 5: WavPack v.4 sample-coding bit reader + run-length decoder
  (first half of the wiki "Samples coding" section). A new `samples`
  module adds `BitReader` — a least-significant-bit-first reader over
  the `0x0A` packed-samples payload exposing the three wiki primitives
  `get_unary()` (count of leading `1` bits up to the terminating
  `0`), `get_bit()` and `get_bits(n)` (assembled LSB-first into a
  `u32`, `get_bits(0) == 0`). Reads past the buffer report
  `Error::Truncated` rather than zero-filling, and `bits_remaining()`
  / `is_empty()` expose the cursor. `decode_run_length` transcribes
  the wiki pseudocode's run-length half: it short-circuits to `n = 0`
  when the carried `last_zero` flag is set, otherwise reads the unary
  prefix, applies the `n == 16` escape (second unary `n2`; `n += n2`
  for `n2 < 2` else `n += (1 << (n2-1)) | getbits(n2-1)`), halves `n`
  with the odd-round-up rule, and updates the adaptive `last_zero` /
  `last_one` carry held in a new `RunState`. Public constant
  `UNARY_ESCAPE = 16`. The second half (Golomb `(base, add)` interval
  selection + median adaptation) is deferred: the wiki names the
  median "increase" / "decrease" steps without quantifying them, so
  per-sample reconstruction is not yet bit-exact from this page —
  documented as a docs gap on the `samples` module. The `0x0A` bit
  order (LSB-first, matching WavPack's little-endian container) is
  likewise a documented assumption pending a real-payload check.
- 19 new unit tests (69 total): LSB-first `get_bit` ordering and
  byte-boundary crossing; `get_bits` LSB-first assembly, zero-count
  no-op, full 32-bit width and truncation; `get_unary` against the
  wiki worked examples (`111110b → 5`, `10b → 1`), the immediate-
  terminator zero run, terminator-only consumption, and unterminated
  truncation; and the run-length decoder's `last_zero` short-circuit
  (no bits consumed, flag cleared), even / odd unary halving with the
  matching `last_one` / `last_zero` carry, both escape arms
  (`n2 < 2` direct add and `n2 >= 2` LSB-first mantissa, two distinct
  mantissa values), the all-zero unary case, the multi-sample adaptive
  carry sequence, and unary truncation.

- Round 4: WavPack v.4 entropy-info sub-block expander.
  `expand_entropy` decodes the `0x05` payload into an
  `EntropyInfo { medians_left: [i32; 3], medians_right: [i32; 3] }`
  per the wiki "Entropy info" section ("one or two sets of medians …
  log-packed into 16 bits as described above"). Each median is a
  little-endian 16-bit log-packed word in the same format as the
  round-3 decorrelation-samples expander (`[mantissa_lo, exponent_hi]`,
  mantissa signed, exponent biased by `-9`); the shared expander is
  re-used via a new `pub(crate)` accessor on `decorrelation`. Mono
  payloads (6 bytes) populate `medians_left` and leave `medians_right`
  at `[0; 3]`; stereo payloads (12 bytes) populate both. A convenience
  `EntropyInfo::mono(...)` constructor and `is_mono()` predicate are
  exposed. Any other payload length is rejected through a new
  `Error::EntropyInfoLength(usize)` variant. Public constants
  (`MEDIANS_PER_CHANNEL`, `MEDIAN_ON_WIRE_BYTES`,
  `MONO_PAYLOAD_BYTES`, `STEREO_PAYLOAD_BYTES`) document the wire
  layout.
- 12 new unit tests (50 total): mono single-set decode + right-set-
  zeroed contract; stereo two-set decode in left-then-right order;
  signed-mantissa sign-extension on negative medians; both the
  shift-left and shift-right log-pack branches reach the median
  values; empty / sub-mono / between-mono-and-stereo / over-stereo
  length rejections; `EntropyInfo::mono` helper; `is_mono()`
  predicate returning false on any non-zero right-set median;
  all-zero stereo payload decoded sanely.

- Round 3: WavPack v.4 decorrelation sub-block expanders.
  `expand_terms` decodes the `0x02` payload into a
  `DecorrelationTerms { terms: Vec<i8>, deltas: Vec<u8> }`, one
  byte → one `(term, delta)` pair per the wiki "lower 5 bits
  indicate predictor type, high 3 bits contain delta value" rule.
  `expand_weights` decodes the `0x03` payload through the wiki's
  log-pack recipe (`n = getchar() << 3; if (n > 0) n += (n + 64) >> 7`
  with the byte read as signed) into a
  `DecorrelationWeights { weights: Vec<i32> }`. `expand_samples`
  reads the `0x04` payload as little-endian 16-bit words and applies
  the wiki exponent / mantissa expansion (mantissa signed; exponent
  biased by `-9`) into a `DecorrelationSamples { samples: Vec<i32> }`.
  Out-of-range shift counts on the sample expander saturate rather
  than panicking. A new `Error::DecorrelationSamplesOddByteCount`
  variant rejects malformed sample payloads whose byte count is not
  a multiple of two.
- 15 new unit tests (38 total): low-5/high-3 term-byte split across
  the full `0..=18` predictor range; multi-byte term order
  preservation; zero / positive-rounding / negative-no-rounding
  weight expansions and multi-byte order preservation; sample
  expansion for the `exponent == 9` / `< 9` / `> 9` branches with
  positive and negative mantissas; byte-pairing across multi-sample
  payloads; empty-payload handling for all three expanders;
  saturation behaviour for extreme exponents; rejection of odd-byte
  sample payloads.

- Round 2: WavPack v.4 metadata sub-block walker. `walk_metadata`
  consumes the post-header payload returned by
  `parse_block_header` and returns `Vec<MetadataSubBlock>`,
  driving `parse_metadata_sub_block` (also exposed for callers
  that want to validate against `ck_size` themselves). The
  `SubBlockId` enum names every documented sub-block ID listed in
  the wiki "IDs" section: `0x00..=0x0D`
  (`Dummy`/`DecorrelationTerms`/`DecorrelationWeights`/`DecorrelationSamples`/`EntropyInfo`/`HybridProfile`/`NoiseShapingProfile`/`FloatInfo`/`Int32Info`/`PackedSamples`/`PackedCorrectionData`/`PackedOverflowBits`/`MultichannelInfo`)
  plus `0x20..=0x27`
  (`RiffHeader`/`RiffTrailer`/`EncodingDetails`/`Md5Checksum`/`NonStandardSampleRate`).
  Undocumented IDs surface as `Unknown(u8)` rather than erroring
  (the wiki's `0x20` "decoder may ignore" flag is the forward-
  compat mechanism). `SubBlockFlags` decodes the structural flag
  triple (`0x20` optional / `0x40` odd-size / `0x80` large-size);
  the walker handles both the 1-byte and 3-byte (large-flag)
  size-field encodings, strips the trailing odd-size padding byte,
  and reports truncated input as `Error::Truncated`. Two new
  `Error` variants — `MetadataSubBlockTooLarge` and
  `MetadataOddSizeWithoutPayload` — cover the remaining structural
  rejections.
- 13 new unit tests (23 total): flag-triple decoding, the full
  18-entry round-trip through `SubBlockId::from_id_byte` /
  `as_id_byte`, small-format and large-format sub-block parsing,
  odd-size padding strip (small + large), back-to-back walk-to-
  exhaustion, RIFF-wrapper / non-standard-rate sub-block decode,
  unknown-ID acceptance, truncated-input rejection, and the
  zero-word-with-odd-flag rejection.

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
