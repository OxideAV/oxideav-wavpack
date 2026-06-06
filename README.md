# oxideav-wavpack

Pure-Rust WavPack lossless audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 242 — `0x0C` packed-overflow-bits typed view + walker bridges
+ block-level introspection. The wiki "IDs" listing of
`docs/audio/wavpack/wiki/WavPack.wiki` annotates sub-block `0x0C` as
"packed overflow bits from floating-point or large integers", and the
staged clean-room entropy doc
`docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 names the same
ID as the **extension bitstream**. The round-2 metadata walker
already routed `0x0C` payloads through the typed
[`SubBlockId::PackedOverflowBits`] discriminant, but the bytes had
not yet been elaborated into the same typed view + finder +
block-level accessor shape the round-12 [`PackedSamples`] (`0x0A`)
and round-233 [`PackedCorrectionData`] (`0x0B`) views expose. This
round closes that gap. New [`PackedOverflowBits<'a>`] in
`src/overflow_bits.rs` carries the borrowed `0x0C` payload verbatim
and exposes the same `bytes()` / `len()` / `is_empty()` introspection
+ `bit_reader()` factory shape (LSB-first within each byte, the same
convention every payload-carrying view in the crate uses). New
[`expand_packed_overflow_bits`] constructor + walker finders
[`find_packed_overflow_bits`] (typed-view) /
[`find_packed_overflow_bits_sub_block`] (raw-borrow) elaborate the
walker output for the `0x0C` ID without re-walking the metadata. New
block-level accessors [`WavPackBlock::has_packed_overflow_bits`] /
[`WavPackBlock::find_packed_overflow_bits_sub_block`] /
[`WavPackBlock::packed_overflow_bits`] pair the block surface with
the new walker bridges. The float (wiki flag bit 7) / large-integer
(wiki flag bit 8) container fix-ups that would actually consume the
wrapped bytes remain gated on the
[`UnsupportedBlockFeature::FloatData`] /
[`UnsupportedBlockFeature::Int32Mode`] feature refusals in
[`WavPackBlock::decode_samples`] — the typed view is a deferred
handoff into the bit reader, not a decode pass. 17 new tests (476
total, up from 459) pin: bytes-verbatim / empty-acceptance / `len` /
`is_empty` round-trip on the typed view; the `BitReader` factory's
initial cursor and `bits_remaining == 8 * len`; the LSB-first bit
order; factory independence (two readers from the same view advance
separately); the empty-payload bit-reader's immediate `Truncated` (no
zero-fill contract); the `Copy` semantics of the view; the
type-distinct check that `PackedOverflowBits` / `PackedCorrectionData`
/ `PackedSamples` are three different types even though all three
wrap the same byte-slice shape; the walker bridges'
present / absent / sub-block-borrow branches; and the block-level
accessors' false / true / view-round-trip / 3-way independence with
the existing `0x07` + `0x0B` accessors.**

**Round 239 — typed file-total / end-cursor accessors on
[`WavPackBlockHeader`] / [`WavPackBlock`] + a stream-level
[`stream_total_samples`] free function. The wiki "Block structure"
listing of `docs/audio/wavpack/wiki/WavPack.wiki` carries three
explicitly file-global / per-block sample-cursor fields — `total
samples in file` (32-bit, with `0xFFFFFFFF` reserved as the "unknown"
sentinel), `offset in samples for current block`, and `samples in this
block` — that the round-1 header parser preserved verbatim but had not
yet lifted into typed accessors with `Option`-typed sentinel semantics
or derived end-cursor arithmetic. This round adds the three header
accessors and four block-level pass-throughs:
[`WavPackBlockHeader::total_samples_in_file`] returns
`Option<u32>` — `Some(n)` for a known total, `None` for the
[`TOTAL_SAMPLES_UNKNOWN`] sentinel — so callers branching on the
sentinel don't compare against the raw constant;
[`WavPackBlockHeader::end_sample_index`] returns `u64` =
`block_index + block_samples`, the half-open upper bound the next
block's `block_index` should match in a well-formed stream
(metadata-only blocks contribute zero, so the cursor doesn't advance);
[`WavPackBlockHeader::samples_remaining_after`] returns
`Option<u64>` — `Some(total - end)` when the total is known and the
end cursor lies within it, `None` for the sentinel or for end-past-total
malformed combinations. New block-level convenience accessors
[`WavPackBlock::total_samples_in_file`] /
[`WavPackBlock::end_sample_index`] /
[`WavPackBlock::samples_remaining_after`] /
[`WavPackBlock::is_final_audio_block_in_file`] pass through to the
header and add the boolean discriminant for the "is this the last
block of a fully-described file" case (`samples_remaining_after()
== Some(0)`). New stream-level free function [`stream_total_samples`]
returns `Result<Option<Option<u32>>>` — outer `None` for empty input,
outer `Some` carrying the typed file-total from the first block's
header (the wiki documents `total_samples` as file-global, so the
first block's copy is the stream-level source of truth). All four new
surfaces are derived directly from the three explicitly documented
wiki fields — no spec gap, no docs-gap-blocked surface touched, no new
error variants. 23 new tests (459 total, up from 436) pin: the
sentinel / known / zero-as-distinct-from-sentinel discrimination on
the typed `Option`; the u32-extreme-summands non-overflow on
`end_sample_index`; the metadata-only-block end-cursor non-advancement;
the exact-end / non-zero-remainder / unknown-total / end-past-total
branches on `samples_remaining_after`; the boolean `is_final_audio_block_in_file`
discriminant on each of those branches; the stream-level free function
on empty / single-block / multi-block / sentinel / malformed-header
inputs; the cross-block consistency of `end_sample_index` and
`samples_remaining_after` across a synthesised three-block stream.**

**Round 233 — `.wvc` correction-stream typed view + walker bridges +
block-level and stream-level introspection accessors. The wiki "IDs"
listing of `docs/audio/wavpack/wiki/WavPack.wiki` annotates sub-blocks
`0x07` (noise-shaping profile) and `0x0B` (packed correction data) as
carried in the `.wvc` companion file alongside the lossy main `.wv`;
this round elaborates the round-2 walker output for `0x0B` into a typed
[`PackedCorrectionData<'a>`] view (analogous to the round-12
[`PackedSamples<'a>`] view for `0x0A`) and threads the same finder /
predicate / iterator pattern through the metadata-walker /
block-level / stream-level surfaces. New
[`PackedCorrectionData<'a>`] in `src/correction.rs` carries the
borrowed `0x0B` sub-block payload verbatim and exposes the same
`bytes()` / `len()` / `is_empty()` introspection + `bit_reader()`
factory shape [`PackedSamples`] exposes for the main `0x0A` stream.
New [`expand_packed_correction_data`] constructor + walker finders
[`find_packed_correction_data`] (typed-view) /
[`find_packed_correction_data_sub_block`] (raw-borrow) /
[`find_noise_shaping_profile`] / [`find_hybrid_profile`] elaborate the
walker output for the three hybrid-mode sub-block IDs without
re-walking the metadata. New block-level accessors
[`WavPackBlock::has_packed_correction_data`] /
[`WavPackBlock::packed_correction_data`] /
[`WavPackBlock::find_packed_correction_data_sub_block`] /
[`WavPackBlock::has_noise_shaping_profile`] /
[`WavPackBlock::find_noise_shaping_profile_sub_block`] /
[`WavPackBlock::has_hybrid_profile`] /
[`WavPackBlock::find_hybrid_profile_sub_block`] /
[`WavPackBlock::has_correction_stream_data`] (the composite
predicate matching the [`MetadataSubBlock::is_correction_payload`]
grouping) expose the new sub-block IDs at the block level. New
stream-level free functions [`correction_block_count`] /
[`first_correction_block`] / [`iter_correction_blocks`] /
[`total_correction_payload_bytes`] and the new
[`CorrectionBlockIter<'a>`] (`Clone` + `FusedIterator`) lazy iterator
mirror the round-230 [`AudioBlockIter`] pattern but filter to blocks
whose `has_correction_stream_data` predicate fires. The hybrid-mode
sample decode itself (spec §4.2 step 6 second paragraph,
`error_limit != 0`) stays out of scope — the typed views give a
callable handle into the bytes without committing to a decode
semantics, and the existing
[`Error::UnsupportedBlockFeature(UnsupportedBlockFeature::Hybrid)`]
refusal on [`WavPackBlock::decode_samples`] is preserved verbatim. No
new error variants and no docs-gap-blocked surface touched. 44 new
tests (436 total, up from 392) pin: the typed view shape (empty /
non-empty / round-trip / bit-reader factory / `Copy` + lifetime /
distinct-from-[`PackedSamples`] type discrimination); each new
block-level accessor / finder on present / absent / both-present
inputs; the composite `has_correction_stream_data` predicate as the
union of `0x07` and `0x0B`; the new stream-level free functions on
empty / all-plain / mixed / error-trailing inputs;
[`CorrectionBlockIter`]'s `Clone + FusedIterator` trait bounds and
the `new` / free-function call-shape twin equivalence;
[`total_correction_payload_bytes`] summing only `0x0B` payload bytes
(excluding `0x07`); a metadata-only block carrying only a `0x0B`
payload still surfaces as correction-bearing; and the hybrid-flag
refusal contract is unchanged by the presence of a `0x0B` typed view
(structural introspection vs. decode enablement).**

**Round 230 — stream-level introspection accessors composing the
round-219 [`iter_blocks`] for aggregate "how many / what shape" /
"where's the first audio block" questions without retaining the parsed
block list. New free functions [`audio_block_count`] /
[`metadata_block_count`] split the wiki "Block structure"
`block_samples > 0` audio blocks from the `block_samples == 0`
metadata-only blocks (RIFF wrappers, MD5 sums, encoding-details);
together they sum to [`block_count`] across any input. New
[`total_audio_samples`] sums the wiki "samples in this block" field
across audio blocks only, returning `u64` so a 4-GiB-plus stream's
sample count does not overflow `u32`. New [`decoded_sample_count`]
free function sums the `i32` PCM slot count [`decode_stream`] would
produce across the audio blocks (mono / false-stereo contribute
`block_samples`, stereo contribute `block_samples * 2` per the round-206
[`Flags::is_block_data_mono`] dispatch); the matching block-level
[`WavPackBlock::decoded_sample_count`] reports the same shape per-block
from the header alone — no entropy expansion, no per-sample-loop call.
New [`first_audio_block`] peeks the first decode-eligible block past
any leading metadata-only blocks (the wiki allowance for RIFF-header-only
blocks); returns `Ok(None)` on empty / all-metadata-only input and
surfaces the first [`parse_block`] error verbatim if a malformed block
appears before the first audio block. New [`AudioBlockIter`] is the
`Clone`-able, `FusedIterator`-compliant lazy iterator yielding only
audio blocks; [`iter_audio_blocks`] is the call-shape twin. New
[`BlockIter::next_audio`] adapter method skips metadata-only blocks on
the existing block iterator (composes [`Iterator::next`] under the
hood). No new error variants and no docs-gap-blocked surface touched:
every error these accessors surface is one [`parse_block`] already
raised. 34 new tests (392 total, up from 358) pin: each accessor on
empty / all-metadata-only / mixed inputs; the
audio + metadata == total identity across counts; the structural
[`WavPackBlock::decoded_sample_count`] matching the actual PCM length
[`WavPackBlock::decode_samples`] returns on mono and stereo;
[`decoded_sample_count`] (stream) matching the [`decode_stream`] result
length; [`AudioBlockIter`]'s `Clone + FusedIterator` trait bounds; the
`audio_block_count == iter_audio_blocks().count()` equivalence; the
`BlockIter::next_audio` skip-then-yield-then-error-then-fuse contract;
[`AudioBlockIter::new`] and [`iter_audio_blocks`] returning identical
sequences; and parse-error propagation through every accessor.**

**Round 224 — multi-block stream → PCM composer fusing the round-219
[`BlockIter`] with the round-206 [`WavPackBlock::decode_samples`] into a
single byte-buffer → `Vec<i32>` surface. New eager [`decode_stream`]
walks every audio block in the input and concatenates the decoded PCM in
on-disk order; new [`StreamDecodeIter<'a>`] is the `Clone`-able,
`FusedIterator`-compliant lazy counterpart yielding `Result<Vec<i32>>`
once per **audio** block (metadata-only blocks with `block_samples == 0`
are silently skipped since they carry no PCM to return — a positive
contract preventing a spurious [`Error::BlockHasNoAudio`] refusal on
`.wv` files whose first block is a RIFF-header-only metadata block).
New free function [`iter_decoded_blocks`] is the `iter_decoded_blocks(bytes)`
call-shape twin of [`StreamDecodeIter::new`]. [`StreamDecodeIter`] fuses
on the first error (parse or decode) via the underlying round-219
[`BlockIter`] fuse mechanism + the round-206 refusal taxonomy — both
compose without translation. Two introspection accessors on
[`StreamDecodeIter`] — [`StreamDecodeIter::remaining`] and
[`StreamDecodeIter::is_exhausted`] — round out the surface; both
forward to the inner [`BlockIter`] so the offset-diagnostics semantics
[`BlockIter::remaining`] documents (on a fused-error iterator the
remaining slice points at the malformed / unsupported block's first
byte) carry through to the stream-decode level. No new error variants
and no docs-gap-blocked surface touched: every error
[`decode_stream`] / [`StreamDecodeIter`] surfaces is one
[`parse_block`] or [`WavPackBlock::decode_samples`] already raised.
Per-block mono / stereo dispatch (the wiki bit 2 + bit 30 union from
round 206 via [`Flags::is_block_data_mono`]) is preserved verbatim, so a
multi-block input may mix mono and stereo blocks and the concatenated
`Vec<i32>` reflects each block's own shape (mono → `block_samples` i32s;
stereo → `block_samples * 2` interleaved). Per-block `0x05` seed
re-initialisation (round 206 per-block stateless contract) is preserved
across the stream — each block decodes from its own entropy seeds with
no carry across blocks. 21 new tests (358 total) pin: empty-buffer
input yielding `Ok(vec![])` (not an error — the wiki "WavPack file
consists of blocks" sentence is plural but [`BlockIter`] accepts the
degenerate empty file); single audio block yielding `[0]`; three audio
blocks concatenating to `[0, 0, 0]` in on-disk order; metadata-only
blocks silently skipped both between audio blocks and at the leading
position (no spurious [`Error::BlockHasNoAudio`]); all-metadata-only
input yielding `Ok(vec![])`; [`Error::CkSizeExceedsBuffer`] propagated
verbatim from a malformed second block; [`Error::UnsupportedBlockFeature(Hybrid)`]
propagated verbatim from a hybrid-flagged audio block;
[`Error::BlockMissingEntropyInfo`] propagated from an audio block
lacking the `0x05` sub-block; eager [`decode_stream`] discarding
prior-block PCM on a mid-stream decode error (the documented eager
contract); [`iter_decoded_blocks`] yielding one item per audio block
with metadata-only blocks omitted; [`iter_decoded_blocks`] fusing on
first parse error AND on first decode error (both routes); empty input
and all-metadata-only input yielding zero items;
[`StreamDecodeIter::new`] and [`iter_decoded_blocks`] returning
identical sequences; [`StreamDecodeIter::remaining`] tracking the
underlying [`BlockIter`] across a `next()` call (full buffer →
empty after draining); the `Clone + FusedIterator` trait bounds (a
compile-time check via a generic helper); a mixed mono+stereo input
yielding `[0, 0, 0]` (one mono PCM + one stereo frame's two
interleaved PCM) confirming the per-block dispatch contract; and the
eager / lazy equivalence over a three-audio-block input
(`decode_stream` is observationally identical to draining
[`iter_decoded_blocks`] and concatenating each `Vec<i32>` via
`flat_map`).**

**Round 219 — multi-block stream iteration on top of the round-13
[`parse_block`] composer, lifting the wiki "WavPack file consists of
blocks each beginning with 'wvpk'" file-format sentence into typed
public API. New [`BlockIter<'a>`] is a `Clone`-able,
`FusedIterator`-compliant lazy iterator yielding
`Result<WavPackBlock<'a>>` over the chained-block byte buffer; it
walks the chain by repeatedly calling [`parse_block`] on the previous
call's returned tail, fusing on the first error so the caller can
`?`-bubble without re-encountering the same failure on a follow-up
`next()`. New free function [`iter_blocks`] is the `iter_blocks(bytes)`
call-shape twin of [`BlockIter::new`]. New [`parse_blocks`] eagerly
collects the iterator into `Vec<WavPackBlock<'_>>`. New [`block_count`]
counts blocks without retaining them (one-block working set
independent of input length). New [`total_block_samples`] sums the
wiki "samples in this block" field across an already-parsed list,
returning `u64` so a 4-GiB-plus stream's sample count does not
overflow `u32`. Two introspection accessors on [`BlockIter`] —
[`BlockIter::remaining`] (the bytes not yet consumed; on a fused
iterator after an error, points at the malformed block's first byte
for offset diagnostics) and [`BlockIter::is_exhausted`] — round out
the surface. No new error variants and no docs-gap-blocked surface
touched: every iterator yield is a direct [`parse_block`] result and
the existing round-13 [`Error::CkSizeExceedsBuffer`] / [`Error::Truncated`]
split surfaces verbatim across the iteration boundary, preserving the
streaming caller's "need more bytes for this block's payload" vs
"buffer ran out between blocks" distinction. 18 new tests (337 total)
pin: empty-buffer iteration yielding zero items; single-block
iteration with `is_exhausted` after; three back-to-back empty blocks;
`remaining()` shrinking by exactly `8 + ck_size` per step; the
fused-on-error contract with `remaining()` pointing at the malformed
block's first byte; `CkSizeExceedsBuffer { ck_size: 200, available:
32 }` surfacing on a partial second block; `Truncated` surfacing on a
partial header between blocks (round-13 error split preserved across
the iteration boundary); [`BlockIter::new`] and [`iter_blocks`]
returning identical sequences; [`parse_blocks`] returning the same
ordered list as iteration on a synthesised three-block stream with
distinct `block_samples` (`100` / `200` / `300`), bubbling the first
error rather than the partial Vec, and returning an empty Vec on
empty input; [`block_count`] over five blocks; [`total_block_samples`]
summing `100 + 200 + 300 = 600`, returning `0` on the empty slice,
and the u64 return type preventing the `u32::MAX + u32::MAX` two-
block sum from overflowing; and the equivalence check confirming
[`parse_blocks`] is observationally identical to draining
[`iter_blocks`].**

**Round 214 — block-level discovery / accessor sweep on
[`WavPackBlock`], pairing the round-13 `parse_block` aggregate with
the typed views the existing free finders already build over
`self.sub_blocks`. New header passthroughs `flags()` /
`block_samples()` / `block_index()` / `is_audio_block()` lift the
round-1 header fields and predicates to the block surface; six new
`has_*` presence predicates
(`has_entropy_info` / `has_packed_samples` / `has_md5_checksum` /
`has_riff_header` / `has_riff_trailer` / `has_multichannel_info`)
key on [`SubBlockId`] equality; six new borrow finders
(`find_sub_block` / `find_entropy_info_sub_block` /
`find_md5_checksum_sub_block` /
`find_multichannel_info_sub_block` /
`find_riff_header_sub_block` /
`find_riff_trailer_sub_block`) return
`Option<&MetadataSubBlock<'a>>` borrows pairing with each predicate;
three new typed extractors return the round-4 / round-10 / round-12
typed views one call from the block — `packed_samples()` returns
`Option<PackedSamples<'a>>`; `entropy_info()` returns
`Result<Option<EntropyInfo>>` (missing `0x05` → `Ok(None)`,
malformed → propagates [`Error::EntropyInfoLength`]);
`md5_checksum()` returns `Result<Option<Md5Checksum>>` with the same
shape (missing → `Ok(None)`, malformed →
[`Error::Md5ChecksumLength`]). Round 214 composes with the round-206
decode loop: a multi-block iterator typically pre-flights with
`block.is_audio_block()` + `block.has_entropy_info()` +
`block.has_packed_samples()` before calling
`block.decode_samples()`. No new error variants and no docs-gap-
blocked surface touched. 24 new tests (319 total) pin: the four
header passthroughs against the underlying field / predicate; each
of the six `has_*` predicates firing on the matching sub-block ID
and clearing on a metadata-empty block; each of the six borrow
finders returning the present / absent shapes with payload byte
pins; `packed_samples()` returning a typed [`PackedSamples`] view
with byte-array round-trip and `None` on absence; `entropy_info()`
returning the typed `[5, 3, 7]` mono medians via an explicit-
exponent log-pack synthesise (`[m, 0x09]` → `median = m`), the
absent `Ok(None)` case, and the malformed-length error propagation;
`md5_checksum()` decoding the standard "empty input" digest
`d41d8cd98f00b204e9800998ecf8427e` as a pinned test vector, the
absent `Ok(None)` case, and the malformed-length error propagation;
and an end-to-end pairing of all the round-214 accessors and the
round-206 [`WavPackBlock::decode_samples`] composer on a single
block carrying `0x05` + `0x0A` + `0x26` returning the expected `[0]`
PCM sample alongside the typed entropy / md5 / packed-samples views
and `false` for the un-present predicates.**

**Round 206 — block-level `WavPackBlock::decode_samples()` composer
turning a [`parse_block`] aggregate into PCM samples in one call,
with typed gates for every WavPack v.4 feature the round-15/199
per-sample loop does not yet support. The composer chains
[`find_entropy_info`] + [`expand_entropy`] + [`find_packed_samples`]
through [`decode_packed_samples_mono_from_entropy`] or
[`decode_packed_samples_stereo_from_entropy`] depending on
[`Flags::is_block_data_mono`] (the new accessor combining wiki
bit 2 `mono` with wiki bit 30 `false_stereo` — "stream is stereo but
this block's data is mono"). New [`UnsupportedBlockFeature`] enum
names the seven gated cases (`Hybrid` / `FloatData` / `Int32Mode` /
`MultichannelMember` / `Decorrelation` / `LowLatencyBlock` /
`RobustBlock`), surfaced via [`Error::UnsupportedBlockFeature`]. New
structural errors [`Error::BlockHasNoAudio`] /
[`Error::BlockMissingEntropyInfo`] /
[`Error::BlockMissingPackedSamples`] cover the three "block doesn't
carry what the composer needs" shortfalls separately from the
feature gates. New [`WavPackBlock::has_decorrelation`] predicate
detects the presence of any of the `0x02` / `0x03` / `0x04`
decorrelation sub-blocks so the composer refuses pre-pass blocks
without trying to walk the round-3 typed views (which exist but lack
a prediction-loop consumer). 23 new tests (295 total) pin: a mono
one-sample happy path (seed `[0,0,0]` + 0x0A payload `0x00` → `[0]`
via the spec §4.2 step 1 zero-run path emitting a single zero); the
matching stereo happy path with minimal non-zero seeds (`[1,0,0]`
both channels — non-zero so `EntropyInfo::is_mono()` reports stereo,
but still `get_med(0) == 1` so the spec §4.2 step 1 path stays
eligible) yielding `[0, 0]`; the false-stereo mono dispatch
(bit 30 set with bit 2 clear); `BlockHasNoAudio` on
`block_samples == 0`; `BlockMissingEntropyInfo` /
`BlockMissingPackedSamples` on the matching sub-block absence; each
of the seven [`UnsupportedBlockFeature`] variants triggered by the
corresponding flag bit or sub-block presence; the
`EntropyInfoLength` propagation from a malformed `0x05` payload;
[`WavPackBlock::has_decorrelation`] firing on each of the three
sub-block IDs and clearing when none of them is present; the four
`is_block_data_mono` / `is_block_data_stereo` arms (plain mono /
false stereo / plain stereo / mono+false-stereo); and the
[`UnsupportedBlockFeature`] Display strings naming the relevant
wiki bit / sub-block ID for each variant.**

**Round 201 — `EntropyInfo` → `AdaptiveMedians` channel-indexed
bridges and end-to-end `from_entropy` wrappers for the mono + stereo
`0x0A` decode loops. New `EntropyInfo::stereo(left, right)` symmetric
constructor to the existing `EntropyInfo::mono`; new
`AdaptiveMedians::from_entropy(info, channel_idx)` returning
`Some(state)` for channel 0 on any payload and channel 1 on stereo
(matching the `Medians::from_entropy` shape, with `None` for
out-of-range indices, channel 1 on mono, and negative seeds — the same
defensive `i32 → u32` rejection `from_seed_values` performs); new
`AdaptiveMedians::stereo_pair_from_entropy(info)` returning the
`[left, right]` array `decode_packed_samples_stereo` consumes (or
`None` on mono or any negative seed); and new top-level
`decode_packed_samples_mono_from_entropy(payload, info, count)` and
`decode_packed_samples_stereo_from_entropy(payload, info, frames)`
wrappers composing the bridges with the round-15/199 stateful loops.
New `Error::InvalidEntropyInfoForMono` /
`Error::InvalidEntropyInfoForStereo` variants name the malformed-input
arm. 20 new tests (272 total) pin: the stereo constructor populating
both sets and matching its struct-literal form (plus the
is_mono-by-content nuance with a zero right set); the
`AdaptiveMedians::from_entropy` bridge across channel 0/1, out-of-
range indices, mono right-channel rejection, and negative-seed
rejection on both channels; the `stereo_pair_from_entropy` mono
rejection and per-channel negative-seed rejection; and the
`_from_entropy` wrappers proving byte-identical reconstruction to the
explicit-seed calls, the malformed-input errors firing before any
bits are read, and the zero-count / zero-frame vacuous cases.**

**Round 199 — stereo per-sample decode loop wiring the
`docs/audio/wavpack/spec/wavpack-entropy-decode.md` §2 channel-
alternation rule on top of the round-15 mono stateful loop. New
`StereoDecodeState` carries per-channel `RunState` for left + right
(spec §4.2 step 4 holding-bit fold applied per-channel), a stream-
level `zero_run_pending` counter (the §4.2 step 1 zero-run is
stream-level: gated on BOTH channels' `median[0] <= 1` AND BOTH
channels' holding state empty; on success resets BOTH channels'
medians; the drained zero samples alternate across channels by parity)
and a `next_channel` parity cursor that toggles only on a successful
emit (so a `?`-bubbled error leaves the cursor recoverable). New
`decode_sample_stateful_stereo(reader, &mut [AdaptiveMedians; 2],
&mut StereoDecodeState)` produces one stereo sample per call dispatched
to `medians[next_channel]`; new
`decode_packed_samples_stereo(payload, &mut [AdaptiveMedians; 2],
frames)` returns a `Vec<i32>` of `frames * 2` interleaved (L,R,L,R,…)
samples. 13 new tests pin: simulator-driven zone-1 round-trips across
matching and distinct per-channel seeds, mixed-zones, negative-sign
reconstruction, the end-to-end `decode_packed_samples_stereo` loop
matching the per-call sequence bit-for-bit, the stereo zero-run path
zeroing BOTH channels and draining across parity, the BOTH-channel
zero-run gate (one-channel eligibility rejected), per-channel
holding-state independence (left's `last_zero` short-circuit doesn't
touch right's state), truncation leaving `next_channel` unchanged,
EOF escape, and the `StereoDecodeState::default()` /
`StereoDecodeState::new()` equivalence. Total tests: 252 (up from
239).**

**Round 15 — stateful per-sample `0x0A` decode loop wiring the staged
`docs/audio/wavpack/spec/wavpack-entropy-decode.md` §3 + §3.2 + §4.2
end to end. The new `decode_sample_stateful` primitive walks the
spec §4.2 sequence per call (optional §4.2 step 1 zero-run fast path
gated on `get_med(0) <= 1` and no holding bits, carrying
`zero_run_pending` across calls; §4.2 steps 2 + 3 unary prefix with
the `LIMIT_ONES = 16` escape and the `cbits == 33` EOF marker;
§4.2 step 4 holding-bit fold via the existing `RunState`; §4.2 step
5 31-bit-masked `(low, high)` interval; §3.2 per-zone median
adaptation BEFORE the mantissa read; §4.2 step 6 first paragraph
truncated-binary mantissa; §4.2 step 7 sign bit). The new
`decode_packed_samples_mono(payload, &mut medians, count)` wraps it
into the first public end-to-end mono single-block PCM decode on the
crate, returning `Vec<i32>`. 18 new tests prove bit-exact round-trip
via a spec-derived inverse encoder helper across zones 0 / 1 / 2 /
overflow, negative-sign reconstruction, a hand-traced 2-sample
fixture, the zero-run fast path engaging when eligible, and the
EOF escape. 239 tests pass (up from 221).**

**Round 14 — adds the WavPack median-adaptation amount (spec §3 + §3.2)
as a self-contained `AdaptiveMedians` primitive (running `u32` state
with the 4-fractional-bit GET_MED encoding; `inc_median` / `dec_median`
per the spec integer formulas with `DIV0` / `DIV1` / `DIV2` = `128` /
`64` / `32`; `Zone` enum naming the four §3.2 arms with raw
`ones_count` carried through Zone2Overflow; `adapt` /
`adapt_for_ones_count` applying the correct per-zone combination of
inc / dec calls). The newly-unblocked
`docs/audio/wavpack/spec/wavpack-entropy-decode.md` closes the
median-adaptation-amount docs gap that previously pinned `decode_sample`
to a fixed median set; the round-7 single-call decoder is preserved
unchanged, with the per-sample composition gated on a follow-up round.**

**Round 13 — block-header parser + metadata sub-block walker +
decorrelation sub-block expanders + entropy-info expander +
sample-coding bit reader, run-length decoder, Golomb sample-value
reconstruction & single-call per-sample decode + entropy→median
bridge + 11 header-accessor helpers + `TermKind` classifier + 7
`DecorrelationTerms` accessors + `weights_per_term` mono/stereo split
+ wiki-derived per-term `decorrelation_sample_count` + flat-to-per-term
`partition_decorrelation_samples` splitter (with explicit refusal on
the stereo / reserved / undocumented per-term-count docs gap) + 13
`MetadataSubBlock` payload-kind predicates covering every wiki "IDs"
entry + 4 `SubBlockId` classifier helpers + 7 walker finders
(`find_first` + four specialised + decorrelation-triple +
typed-`PackedSamples` packed-samples finder) + 16-byte `Md5Checksum`
typed view with strict-length `parse_md5_checksum` + typed
`PackedSamples` view of the `0x0A` packed-samples sub-block payload
(borrowed bytes + `bit_reader()` factory at bit 0) + `BitReader`
position accessors (`byte_position` / `bit_position` /
`bits_consumed`) + channel-indexed `EntropyInfo` accessors
(`is_stereo` / `channels` / `medians_for_channel`) +
`Medians::from_entropy(info, channel_idx)` channel-indexed bridge +
end-to-end `parse_block` aggregating header+walker into a typed
`WavPackBlock` (header field + sub-blocks list + `contains_sub_block` /
`sub_block_count` / `is_metadata_empty` / `on_disk_len` accessors;
new `Error::CkSizeExceedsBuffer { ck_size, available }` distinguishing
mid-payload truncation from header-boundary truncation) + `BitReader`
non-mutating look-ahead (`peek_bit` / `peek_bits` / `peek_unary`) +
bulk advance (`skip_bits`).** Round 1
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
`last_one` flags; round 6 lands the value part of the second half —
`golomb_interval` maps `n` + a three-median set onto the wiki
`(base, add)` interval, and `decode_sample_value` reads `getbits(k-1)`
(with `k = log2(add)` under the bit-length reading the wiki's own
`ex >= 0` requirement forces), the `t2 >= ex` extra bit, and the sign,
returning the reconstructed sample; round 7 fuses the two halves into
`decode_sample` (one call per sample, matching the wiki's single
pseudocode block) and adds `Medians::from_entropy_left` /
`Medians::from_entropy_right` so the round-4 `EntropyInfo` expander
output feeds the round-6 Golomb decoder directly. The medians are
still taken by value and **not** mutated — the median-adaptation
*amount* is still an open docs gap. All work follows
`docs/audio/wavpack/wiki/WavPack.wiki` (the local snapshot of the
multimedia.cx WavPack reference page).

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
  `last_one` state in a `RunState`.
- [`Medians`] — a channel's three medians (`median[0..=2]`) in wiki
  order, as the `0x05` entropy-info expander produces them.
- [`golomb_interval`] — pure `n` + `Medians` → `GolombInterval`
  `(base, add)` mapping per the wiki's three-way branch
  (`n == 0` / `n == 1` / `n >= 2`). Reads no bits and mutates no
  median.
- [`decode_sample_value`] — the value part of the wiki second half:
  picks the `(base, add)` interval, then reads `getbits(k - 1)` (with
  `k` = bit-length of `add`, the only `log2` reading that keeps the
  wiki's own `ex = (1 << k) - add - 1` non-negative), applies the
  `t2 >= ex` extra-bit fixup, reads the sign, and returns
  `base + t2` (or its ones-complement when the sign bit is set). Takes
  `Medians` **by value** and does not mutate them — the median
  "increase" / "decrease" *amount* is still a docs gap, so the stateful
  full-payload loop is deferred. The degenerate `add == 0` interval
  (a median of `1`, where `log2(0)` / `getbits(-1)` are undefined)
  returns `Error::GolombDegenerateInterval` rather than a guessed
  value.
- [`decode_sample`] — fuses the run-length and value halves into one
  per-sample call, matching the wiki's contiguous pseudocode block:
  reads the unary prefix (and `n == 16` escape) through
  `decode_run_length`, then the Golomb mantissa / sign through
  `decode_sample_value`. Carries the adaptive `RunState` and takes the
  three medians by value — the median-adaptation *amount* docs gap still
  blocks the stateful payload loop.
- [`Medians::from_entropy_left`] / [`Medians::from_entropy_right`] —
  pull a channel's three medians straight out of a round-4
  `EntropyInfo` value so the entropy-info expander output feeds the
  Golomb decoder without the caller re-typing the array.
- [`Flags::is_lossless`] / [`Flags::is_lossy`] — symmetric predicates
  around the wiki bit 3 "hybrid profile (lossy compression)" label.
- [`Flags::has_custom_sample_rate`] — `true` when bits 23..=26 hold
  the wiki sentinel `15` ("unknown/custom"); when set, the actual
  rate is in metadata sub-block `0x27`.
- [`Flags::should_skip_decode`] — surfaces the wiki bit 31 "do not
  decode if encountered" decode-gating instruction; bit 28
  ("experimental, okay to ignore") is deliberately **not** included.
- [`Flags::is_experimental`] — diagnostic union of the two wiki-
  labelled experimental bits (28 + 31).
- [`Flags::effective_bit_depth`] — `bytes_per_sample * 8 - left_shift`
  per the wiki "12-bit / 20-bit" worked examples; saturates to `0`
  rather than underflowing on a malformed `left_shift > container_bits`.
- [`Flags::is_standalone_block`] / [`Flags::is_multichannel_member`]
  — distinguishes the wiki "multi-channel start and end blocks"
  degenerate `0b11` marker (a plain stereo file's single-block
  set) from any other marker combination (which signals participation
  in a multi-block channel grouping).
- [`WavPackBlockHeader::is_audio_block`] — `block_samples > 0`,
  per the wiki "may be 0 if no audio present" note.
- [`WavPackBlockHeader::is_total_samples_known`] — distinguishes the
  wiki [`TOTAL_SAMPLES_UNKNOWN`] sentinel from a real count.
- [`WavPackBlockHeader::payload_bytes`] — bytes of metadata sub-block
  payload the `ck_size` field advertises (`ck_size - 24`).
- [`TermKind`] — typed classification of a decorrelation predictor
  code per the wiki "Possible predictor values" listing: `Stereo
  { implemented }` (`0..=5`, with `2..=4` flagged as implemented),
  `SampleBased { sample_count }` (`6..=12`, count = `code - 5`),
  `Reserved` (`13..=16`), `TwoSample` (`17..=18`), and `Unknown` for
  codes outside the documented range. [`TermKind::is_implemented`]
  and [`TermKind::previous_samples`] surface the wiki's two
  narrowings.
- [`DecorrelationTerms::len`] / [`DecorrelationTerms::is_empty`] /
  [`DecorrelationTerms::kind_at`] / [`DecorrelationTerms::iter_kinds`]
  / [`DecorrelationTerms::all_implemented`] /
  [`DecorrelationTerms::has_reserved`] — convenience accessors that
  classify the round-3 term list without re-walking the bytes.
- [`weights_per_term`] — wiki "Each decorrelation term should have
  one or two weights depending on channels" split: mono → 1, stereo →
  2, with a defensive clamp for any higher channel count.
- [`decorrelation_sample_count`] / [`TermKind::decorrelation_sample_count`]
  — wiki "Decorrelation samples" / "Possible predictor values" per-term
  seed-sample count: `Some(code - 5)` for `6..=12`, `Some(2)` for
  `17..=18`, `None` for stereo predictors `0..=5` (per-term count is a
  docs gap), the reserved `13..=16` range, and codes outside `0..=18`.
  Public constant [`MAX_DECORRELATION_SAMPLES_PER_TERM`] = 16 surfaces
  the wiki "up to 16 samples" upper bound.
- [`DecorrelationTerms::expected_decorrelation_sample_count`] — sums
  the per-term wiki count across a term list, returning
  `Some(total)` when every term is documented and `None` as soon as
  any one is in the docs gap (so a caller can decide whether to
  partition or treat the block as undecodable from the wiki alone).
- [`partition_decorrelation_samples`] — splits the flat
  `DecorrelationSamples::samples` list produced by `expand_samples`
  into one `Vec<i32>` per term in wiki order, using the per-term
  counts above. Returns
  [`Error::DecorrelationSampleCountUnspecified`] when any term lacks
  a wiki count, and [`Error::DecorrelationSampleCountMismatch`] when
  the summed expected count does not equal the flat payload length.
- [`MetadataSubBlock::is_optional`] /
  [`MetadataSubBlock::is_decorrelation_payload`] /
  [`MetadataSubBlock::is_correction_payload`] /
  [`MetadataSubBlock::is_audio_payload`] /
  [`MetadataSubBlock::is_riff_payload`] /
  [`MetadataSubBlock::is_dummy_payload`] /
  [`MetadataSubBlock::is_hybrid_profile_payload`] /
  [`MetadataSubBlock::is_float_payload`] /
  [`MetadataSubBlock::is_int32_payload`] /
  [`MetadataSubBlock::is_overflow_bits_payload`] /
  [`MetadataSubBlock::is_multichannel_info_payload`] /
  [`MetadataSubBlock::is_encoding_details_payload`] /
  [`MetadataSubBlock::is_md5_payload`] /
  [`MetadataSubBlock::is_sample_rate_payload`] — payload-kind
  predicates covering every entry in the wiki "IDs" listing so a
  caller can pick a specific sub-block out of a walk without
  re-matching the [`SubBlockId`] enum.
- [`SubBlockId::is_decorrelation`] / [`SubBlockId::is_correction_stream`]
  / [`SubBlockId::is_riff_wrapper`] / [`SubBlockId::is_audio`] — the
  same family classifiers on the enum value itself for callers that
  branch on an ID rather than on a parsed sub-block.
- [`Md5Checksum`] — typed view of the `0x26` payload (the wiki "16-byte
  MD5 sum of raw audio data"), with [`parse_md5_checksum`] enforcing
  the fixed 16-byte length (other lengths reported through new
  [`Error::Md5ChecksumLength`]).
- [`find_first`] / [`find_audio_payload`] / [`find_entropy_info`] /
  [`find_md5_checksum_block`] / [`find_multichannel_info`] /
  [`find_decorrelation_triple`] / [`find_packed_samples`] —
  convenience finders over a [`walk_metadata`] result. The triple
  finder returns `(terms, weights, samples)` in wiki order or `None`
  when any of the three is missing — a malformed-block signal for the
  prediction loop. [`find_packed_samples`] returns the `0x0A` payload
  already wrapped as a typed [`PackedSamples`] (the typed counterpart
  to [`find_audio_payload`]).
- [`PackedSamples`] / [`expand_packed_samples`] — typed view of the
  `0x0A` packed-samples sub-block payload (the entropy-coded audio
  bitstream the wiki "Samples coding" section consumes). Borrows the
  walker's payload bytes verbatim and exposes [`PackedSamples::bytes`]
  / [`PackedSamples::len`] / [`PackedSamples::is_empty`] introspection
  plus a [`PackedSamples::bit_reader`] factory that yields a fresh
  [`BitReader`] positioned at bit 0 — the round-2 walker → round-5/6/7
  decoder handoff in one call. The wiki places no length constraint on
  the payload (the sample count is conveyed out-of-band by the block
  header's `block_samples`), so any byte slice, including the empty
  one, is accepted without rejection.
- [`BitReader::byte_position`] / [`BitReader::bit_position`] /
  [`BitReader::bits_consumed`] — cursor accessors naming the reader's
  position in the underlying byte slice. `bits_consumed` clamps at the
  buffer length when the reader has advanced past the end so callers
  computing a percentage / progress over a `0x0A` payload don't
  overshoot.
- [`EntropyInfo::is_stereo`] / [`EntropyInfo::channels`] /
  [`EntropyInfo::medians_for_channel`] — typed channel introspection
  pinning the wiki "one or two sets of medians for samples decoding"
  sentence as `1` or `2` populated sets, with a channel-indexed median
  getter returning `Some([m0, m1, m2])` for `0` (left/mono) or `1`
  (right, stereo only) and `None` for `1` on a mono payload (where the
  wiki put no second set on the wire) and for indices `>= 2`.
- [`Medians::from_entropy`] `(info, channel_idx)` — channel-indexed
  bridge over [`EntropyInfo`] returning `Some(Medians)` for `0` and
  for `1` on a stereo block, `None` otherwise. Equivalent to
  [`Medians::from_entropy_left`] / [`Medians::from_entropy_right`] but
  with the mono guard, so callers iterating per-channel medians
  (one or two iterations against `Flags::channels_in_block`) skip the
  hand-rolled mono / stereo branch.
- [`parse_block`] — end-to-end one-call composer: parses the 32-byte
  fixed header (round 1), validates the input carries the `8 + ck_size`
  bytes the wiki "Block structure" listing declares, walks the metadata
  sub-block region (round 2), and returns the typed [`WavPackBlock`]
  aggregate plus the unconsumed tail (the next block in a multi-block
  `.wv` file). Reports the new [`Error::CkSizeExceedsBuffer`] variant
  when the header parses but the payload is short, so a streaming caller
  can size the next read against `8 + ck_size - available`.
- [`WavPackBlock`] — typed aggregate: a [`WavPackBlockHeader`] alongside
  a `Vec<MetadataSubBlock>` (borrowed payload slices into the same
  input bytes). Accessors: [`WavPackBlock::header`],
  [`WavPackBlock::sub_blocks`], [`WavPackBlock::contains_sub_block`]
  (boolean shortcut over [`find_first`] for presence checks),
  [`WavPackBlock::sub_block_count`], [`WavPackBlock::is_metadata_empty`]
  (the `ck_size == 24` header-only edge case the wiki allows), and
  [`WavPackBlock::on_disk_len`] (`8 + ck_size`, the byte count of the
  whole block on disk — useful for callers stepping across blocks
  without re-parsing).
- [`BitReader::peek_bit`] / [`BitReader::peek_bits`] /
  [`BitReader::peek_unary`] — non-mutating look-ahead. Read a single
  bit, a multi-bit value, or a unary run-length without advancing the
  cursor; implemented by reading from a clone, so the LSB-first bit
  order rules in `get_bit` / `get_bits` / `get_unary` carry through
  unchanged. On [`Error::Truncated`] the original reader's cursor is
  unchanged, so a caller can retry against a freshly-extended buffer
  without rebuilding the reader. Useful for probing the wiki `n == 16`
  escape pattern (the leading unary indicating whether a second unary
  follows) before committing to a real `decode_run_length` call.
- [`BitReader::skip_bits`] — advance the reader by `count` bits without
  assembling a `u32`. Reports [`Error::Truncated`] when the buffer is
  exhausted before `count` bits have been skipped; on truncation the
  cursor lands at the buffer end (matching the partial-consume
  semantics of `get_bits`). Useful for stepping past a known-length
  opaque field (a padding region, an already-validated value) without
  holding the assembled value.
- [`AdaptiveMedians`] (round 14, spec §3 + §3.2) — a three-`u32`
  running median state with the spec §2.1 4-fractional-bit encoding.
  Methods:
  - [`AdaptiveMedians::new`] / [`AdaptiveMedians::from_seed_values`] /
    [`AdaptiveMedians::from_medians`] — construct from raw `u32`
    values, the round-4 expander seed values (validated non-negative)
    or a round-6 [`Medians`] (also validated non-negative).
  - [`AdaptiveMedians::get_med`] — the spec §2.1 working median
    `(median[i] >> 4) + 1` (the value the spec §4.2 interval ladder
    consumes).
  - [`AdaptiveMedians::inc_median`] / [`AdaptiveMedians::dec_median`]
    — single-index increment / decrement per the spec §3 integer
    formulas (`((median[i] + D) / D) * 5` up and
    `((median[i] + (D - 2)) / D) * 2` down with `D` =
    [`DIV0`] / [`DIV1`] / [`DIV2`]). Saturating semantics defend
    against pathological values.
  - [`AdaptiveMedians::adapt`] / [`AdaptiveMedians::adapt_for_ones_count`]
    — the spec §3.2 per-zone update — applies the correct combination
    of `inc_median` / `dec_median` for the [`Zone`] the decoder is in:
    zone 0 → dec `median[0]`; zone 1 → inc `median[0]`, dec
    `median[1]`; zone 2 → inc `median[0]` + `median[1]`, dec
    `median[2]`; zone 2 overflow → all three inc.
- [`Zone`] / [`Zone::from_ones_count`] / [`Zone::ones_count`] — typed
  view of the four spec §3.2 arms driven from a raw `ones_count`
  value. Zone 2 overflow carries the raw value through so the
  `(ones_count - 2) * get_med(2)` shift in the §4.2 `low` formula is
  still recoverable.
- Public constants: [`DIV0`] / [`DIV1`] / [`DIV2`] (the §3 per-median
  divisors `128` / `64` / `32`), [`MEDIAN_INC_MULTIPLIER`] (`5`),
  [`MEDIAN_DEC_MULTIPLIER`] (`2`), [`GET_MED_SHIFT`] (`4`),
  [`GET_MED_FLOOR`] (`1`).
- [`StereoDecodeState`] (round 199, spec §2 channel-alternation +
  §4.2 stream-level zero-run) — stereo decode state with two per-
  channel [`RunState`]s (`left_run`, `right_run`), a stream-level
  `zero_run_pending` counter, an `ever_took_zero_run` sticky bit, and
  a `next_channel` parity cursor (`0` = left, `1` = right). Built via
  [`StereoDecodeState::new`].
- [`decode_sample_stateful_stereo`] — one stereo sample per call.
  Picks the channel from `state.next_channel` and dispatches the
  spec §4.2 sequence to `medians[ch]` and the matching per-channel
  [`RunState`]. The §4.2 step 1 zero-run fast path is stream-level:
  gated on BOTH channels' `get_med(0) <= 1` AND BOTH channels'
  holding state empty, and on a non-zero run zeros BOTH channels'
  medians. Toggles `next_channel` only on a successful emit so a
  truncation cursor stays recoverable.
- [`decode_packed_samples_stereo`] — end-to-end stereo loop.
  Decodes `frames` stereo frames from a [`PackedSamples`] payload
  into a `Vec<i32>` of `frames * 2` interleaved (L,R,L,R,…) PCM
  samples. The `medians` array MUTATES in place across the loop —
  the caller's `[left_seed, right_seed]` is the running state.
- [`EntropyInfo::stereo`] (round 201) — symmetric constructor to
  [`EntropyInfo::mono`] taking both per-channel median sets at once;
  matches the two-set form the wiki "one or two sets of medians"
  sentence describes. An all-zero right set still reports
  `is_mono() == true` (content-only check).
- [`AdaptiveMedians::from_entropy`] (round 201) — channel-indexed
  bridge over [`EntropyInfo`] returning `Some(state)` for channel 0
  on any payload and channel 1 on a stereo payload, `None` for
  out-of-range indices, channel 1 on mono (the wiki put no second
  set on the wire), and negative seeds (`i32 → u32` defensive
  reject). Symmetric counterpart to [`Medians::from_entropy`] for
  the round-15 running adaptive state.
- [`AdaptiveMedians::stereo_pair_from_entropy`] (round 201) — returns
  the `[left, right]` two-element array
  [`decode_packed_samples_stereo`] takes as `medians`. `None` on a
  mono payload (no right-channel seed on the wire) or when either
  set carries a negative seed.
- [`decode_packed_samples_mono_from_entropy`] (round 201) — end-to-
  end mono decode driven directly by the round-4 `0x05` expander
  output: composes [`AdaptiveMedians::from_entropy(info, 0)`] with
  [`decode_packed_samples_mono`]. Returns
  [`Error::InvalidEntropyInfoForMono`] when the channel-0 seed is
  malformed (negative); other errors propagate verbatim from the
  inner call. Seeds are consumed by value (no caller-side state
  carry).
- [`decode_packed_samples_stereo_from_entropy`] (round 201) —
  end-to-end stereo decode driven directly by the round-4 `0x05`
  expander output: composes
  [`AdaptiveMedians::stereo_pair_from_entropy`] with
  [`decode_packed_samples_stereo`]. Returns
  [`Error::InvalidEntropyInfoForStereo`] when the input is mono or
  any per-channel seed is negative; other errors propagate verbatim.
  Seeds are consumed by value.
- [`Flags::is_block_data_mono`] / [`Flags::is_block_data_stereo`]
  (round 206) — union of wiki bit 2 `mono` and wiki bit 30
  `false_stereo` ("stream is stereo but this block's data is mono").
  The per-block decoder picks its mono / stereo loop on this
  predicate because a false-stereo block carries the same single-
  channel `0x05` + `0x0A` layout as a natively-mono block.
- [`WavPackBlock::has_decorrelation`] (round 206) — `true` when the
  block carries any of the `0x02` / `0x03` / `0x04` decorrelation
  sub-blocks. Surfaces the structural shortfall the composer uses
  to gate decode off via [`UnsupportedBlockFeature::Decorrelation`].
- [`WavPackBlock::decode_samples`] (round 206) — one-call
  "block → PCM" composer. Chains [`find_entropy_info`] +
  [`expand_entropy`] + [`find_packed_samples`] +
  [`decode_packed_samples_mono_from_entropy`] /
  [`decode_packed_samples_stereo_from_entropy`] with the round-206
  mono / stereo dispatch on [`Flags::is_block_data_mono`]. Returns
  a `Vec<i32>` of `block_samples` mono samples or
  `block_samples * 2` interleaved stereo samples. Refuses the seven
  WavPack v.4 features the per-sample loop does not yet support via
  typed [`UnsupportedBlockFeature`] tags through
  [`Error::UnsupportedBlockFeature`]; refuses structurally-incomplete
  blocks via [`Error::BlockHasNoAudio`] /
  [`Error::BlockMissingEntropyInfo`] /
  [`Error::BlockMissingPackedSamples`].
- [`UnsupportedBlockFeature`] (round 206) — typed tag naming the
  WavPack v.4 feature [`WavPackBlock::decode_samples`] refused:
  `Hybrid` (lossy profile, flag bit 3), `FloatData` (bit 7),
  `Int32Mode` (bit 8), `MultichannelMember` (bits 11..=12 != 0b11),
  `Decorrelation` (`0x02` / `0x03` / `0x04` sub-blocks present),
  `LowLatencyBlock` (bit 31), `RobustBlock` (bit 28). Carries
  through [`Error::UnsupportedBlockFeature`].
- [`WavPackBlock::flags`] (round 214) — borrow the parsed
  [`Flags`] view from the fixed block header. Equivalent to
  `&block.header().flags` but spelled directly so caller code
  picking flag predicates off a borrowed [`WavPackBlock`] doesn't
  need to re-bind the header first.
- [`WavPackBlock::block_samples`] / [`WavPackBlock::block_index`]
  (round 214) — passthrough accessors for the wiki "samples in this
  block" and "offset in samples for current block" header fields.
- [`WavPackBlock::is_audio_block`] (round 214) — block-level lift of
  the round-1 [`WavPackBlockHeader::is_audio_block`] predicate
  (`block_samples != 0`). Pairs with [`Self::decode_samples`] which
  refuses metadata-only blocks via [`Error::BlockHasNoAudio`].
- [`WavPackBlock::has_entropy_info`] /
  [`WavPackBlock::has_packed_samples`] /
  [`WavPackBlock::has_md5_checksum`] /
  [`WavPackBlock::has_riff_header`] /
  [`WavPackBlock::has_riff_trailer`] /
  [`WavPackBlock::has_multichannel_info`] (round 214) — presence
  predicates keyed on the matching wiki sub-block ID (`0x05` /
  `0x0A` / `0x26` / `0x20` / `0x21` / `0x0D`). Pair with the
  corresponding `find_*_sub_block` borrow finders.
- [`WavPackBlock::find_sub_block`] (round 214) — block-level
  convenience over [`crate::find_first`] returning an
  `Option<&MetadataSubBlock<'a>>` for the first sub-block matching
  the supplied [`SubBlockId`].
- [`WavPackBlock::find_entropy_info_sub_block`] /
  [`WavPackBlock::find_md5_checksum_sub_block`] /
  [`WavPackBlock::find_multichannel_info_sub_block`] /
  [`WavPackBlock::find_riff_header_sub_block`] /
  [`WavPackBlock::find_riff_trailer_sub_block`] (round 214) —
  block-level specialised borrow finders pairing with the
  corresponding `has_*` predicates. Each returns an
  `Option<&MetadataSubBlock<'a>>` borrow over `self.sub_blocks()`.
- [`WavPackBlock::packed_samples`] (round 214) — locate the `0x0A`
  packed-samples sub-block and wrap it as a typed [`PackedSamples`]
  view in one call. Returns `None` when no `0x0A` sub-block is
  present.
- [`WavPackBlock::entropy_info`] (round 214) — locate the `0x05`
  entropy-info sub-block and expand its payload into a typed
  [`EntropyInfo`] in one call. Returns `Ok(None)` when no `0x05`
  sub-block is present (a structurally legal case — metadata-only
  blocks have no medians to seed); returns
  `Err(Error::EntropyInfoLength)` when the sub-block is present but
  malformed.
- [`WavPackBlock::md5_checksum`] (round 214) — locate the `0x26`
  MD5-checksum sub-block and parse its 16-byte payload into a typed
  [`Md5Checksum`] in one call. Returns `Ok(None)` when no `0x26`
  sub-block is present (the wiki "IDs" listing makes the MD5
  optional); returns `Err(Error::Md5ChecksumLength)` when the
  sub-block is present but the payload is the wrong length.
- [`BlockIter`] (round 219) — `Clone`-able, `FusedIterator`-
  compliant lazy iterator yielding `Result<WavPackBlock<'a>>` over a
  chained-block byte buffer. Each `next()` calls [`parse_block`] on
  the previous tail; the iterator fuses on the first error so a
  `?`-bubble preserves the failure shape without re-trying it.
  Accessors [`BlockIter::remaining`] (bytes not yet consumed; on a
  fused-error iterator this points at the malformed block's first
  byte for precise offset diagnostics) and [`BlockIter::is_exhausted`]
  (`true` when no further items will be yielded). Construct via
  [`BlockIter::new`] or the equivalent free function [`iter_blocks`].
- [`iter_blocks`] (round 219) — free-function constructor for
  [`BlockIter`] over a byte buffer; the `iter_blocks(bytes)` call
  shape readers expect when scanning a `.wv` file's worth of blocks.
- [`parse_blocks`] (round 219) — eager wrapper around [`iter_blocks`]
  returning `Result<Vec<WavPackBlock<'_>>>` on a fully-clean input
  or bubbling the first parse error verbatim.
- [`block_count`] (round 219) — iterate via [`iter_blocks`] and
  count without retaining the parsed blocks (working-set memory
  stays at one block independent of input length). Returns the
  count on a fully-clean input or the first parse error.
- [`total_block_samples`] (round 219) — pure accessor summing the
  wiki "samples in this block" field across an already-parsed
  block list. Returns `u64` so a multi-block file whose individual
  `block_samples` fit `u32` but whose sum exceeds it (a 4-GiB-plus
  stream) does not overflow on the way out.

### Out of scope (later rounds)

- Wiring [`AdaptiveMedians::adapt`] into `decode_sample` /
  `decode_sample_value` so the per-sample call mutates the running
  medians in place. Round 14 lands the spec §3 + §3.2 adaptation
  amount as a self-contained primitive (`AdaptiveMedians`), unblocking
  the previous docs gap that pinned the wiki "increase" / "decrease"
  steps as a numeric question. Composition with the round-7 single-call
  decoder + a real `0x0A` payload-loop driver is the next round's work.
- The degenerate `add == 0` Golomb interval (selected median `1`),
  where the wiki's `k = log2(0)` and `getbits(-1)` are undefined.
  `decode_sample_value` rejects it via `Error::GolombDegenerateInterval`
  pending a docs revision that specifies the single-codeword interval.
- The bit order of the `0x0A` stream is a documented assumption
  (least-significant-bit-first, matching WavPack's little-endian
  container); empirical confirmation against a real payload is gated
  on the median-adaptation gap above.
- The prediction loop that consumes the round-3 typed views.
- Per-term grouping of the samples list for **stereo predictors
  `0..=5`** (the wiki gives no per-term sample count for them; round 11
  lands the per-term partitioner for the documented `6..=12` / `17..=18`
  codes and refuses the stereo case via
  [`Error::DecorrelationSampleCountUnspecified`]).
- Hybrid-profile (lossy) `0x06` / noise-shaping `0x07`.
- Float-data `0x08` / large-or-shifted-int `0x09` / overflow-bits
  `0x0C`.
- Multichannel `0x0D` channel-mask handling.
- Non-standard sample-rate `0x27` numeric decode.
- Hybrid correction stream (`.wvc`) pairing.
- CRC32 verification (depends on sample decode).
- Encoder.

## Clean-room provenance

Every round through round 15 has read **only** the staged WavPack
documentation under `docs/audio/wavpack/` (the local
`wiki/WavPack.wiki` snapshot and, from round 15 onward, the
`spec/wavpack-entropy-decode.md` clean-room trace) and
`oxideav-core`'s public API. No external library source, no archived
prior history of this crate, and no online resources were consulted
at any phase.

The 197-test unit suite synthesises minimal valid headers, sub-blocks
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
including the wiki worked examples and byte-boundary crossing, the
run-length decoder's `last_zero` short-circuit, even / odd unary
halving, both escape arms with LSB-first mantissa assembly, and the
adaptive carry across a multi-sample sequence, the Golomb
`(base, add)` interval selection across the `n == 0` / `n == 1` /
`n >= 2` branches, the `k = log2(add)` bit-length derivation with its
`ex >= 0` invariant swept across `add` 1..=1024, the short- and
long-mantissa `t2 >= ex` paths, positive / ones-complement sign
reconstruction, the degenerate `add == 0` rejection, mantissa- and
sign-truncation reporting, an end-to-end compose of the run-length and
value halves over one contiguous bitstream, the round-7
`Medians::from_entropy_left` / `from_entropy_right` bridges for
stereo and mono inputs, and `decode_sample` chained-call coverage —
run-length-then-value, `last_zero` short-circuit honoured,
degenerate-interval and truncation error propagation, and the
entropy-info → median → sample end-to-end path; and the round-8
block-header accessor sweep — `is_standalone_block` /
`is_multichannel_member` across all four marker combinations,
`is_lossless` / `is_lossy` symmetry around the hybrid bit,
`has_custom_sample_rate` sentinel pin sweep across all 16
sample_rate_index values, `should_skip_decode` discriminating bit 31
from bit 28, `is_experimental` union, `effective_bit_depth` for the
wiki 12-bit / 20-bit worked examples plus the no-shift baseline
plus the saturation case, `is_audio_block` keyed on a non-zero
`block_samples`, `is_total_samples_known` against the sentinel and
the boundary `0`, and `payload_bytes` subtracting the 24-byte fixed
header floor; and the round-9 decorrelation-term classification +
metadata-payload kind sweep — `TermKind::from_code` across all four
wiki categories (stereo implemented `2..=4`, stereo unimplemented
`0/1/5`, sample-based `6..=12` with per-code sample count, reserved
`13..=16`, two-sample `17..=18`, and undocumented `19..=31` plus a
negative-code defensive check); `DecorrelationTerms` `len`/`is_empty`/
`kind_at`/`iter_kinds`/`all_implemented`/`has_reserved` accessors over
mixed term lists; `weights_per_term` mono/stereo split with 0- and 3-
channel clamps; `MetadataSubBlock::is_optional` pinning the `0x20`
flag; and per-kind payload predicates round-tripping for `0x02`/`0x03`/
`0x04` decorrelation, `0x07`/`0x0B` correction, `0x0A` audio, and
`0x20`/`0x21` RIFF with a non-RIFF optional negative case);
and the round-10 MD5 + walker-finder + remaining-kind-predicate
sweep — `SubBlockId` classifier coverage across all four buckets
(decorrelation `0x02`/`0x03`/`0x04`, correction-stream `0x07`/`0x0B`,
RIFF-wrapper `0x20`/`0x21` with same-flag `0x25`/`0x26`/`0x27`
negative cases, audio-only `0x0A`); one-hot kind-predicate sweep
across the eight new `MetadataSubBlock` predicates (with the four
main-bucket predicates pinned false on each); `is_md5_payload`
discriminating `0x06` HybridProfile from `0x26` Md5Checksum on the
low-5-bit overlap and `is_dummy_payload` discriminating `0x00` Dummy
from `0x20` RiffHeader; `parse_md5_checksum` accept (MD5 of `""`
test vector) and reject (0 / 15 / 17 / 64-byte lengths); end-to-end
round-trip from a synthesised `0x26` sub-block through
`walk_metadata` → `find_md5_checksum_block` → `parse_md5_checksum`
(MD5 of the "quick brown fox" test vector); walker finder coverage —
`find_first` hit + miss across `SubBlockId::EntropyInfo` vs
`SubBlockId::HybridProfile`, the four specialised finders, and
`find_decorrelation_triple` returning the full triple in order and
`None` when either of weights / samples is dropped); and the round-11
per-term decorrelation-sample-count + partitioner sweep —
`decorrelation_sample_count` returning `Some(code - 5)` across the full
`6..=12` sample-based range, `Some(2)` for `17` / `18`, and `None`
across stereo `0..=5`, reserved `13..=16`, and undocumented `19..=31`
plus a negative-code defensive check; the [`MAX_DECORRELATION_SAMPLES_PER_TERM`]
= 16 wiki upper-bound sanity sweep across every documented count;
`DecorrelationTerms::expected_decorrelation_sample_count` summing a
mixed `[6, 8, 17, 12]` term list to `13`, the vacuous empty-list `0`,
and `None` propagation when a stereo / reserved / undocumented code
appears anywhere in the list; and `partition_decorrelation_samples`
splitting a `[6, 8, 17]` term list with matching 6-sample flat input
in term order, the empty-terms-empty-payload base case, refusing the
stereo `[2]` and reserved `[6, 14]` lists with
`DecorrelationSampleCountUnspecified`, rejecting both short
(`expected: 6, actual: 5`) and long (`expected: 1, actual: 4`) flat
payloads with `DecorrelationSampleCountMismatch`, and a round-trip
from `expand_samples` of a synthesised `[6, 18]` wire through the
partitioner back to per-term `[1]` + `[2, 3]` lists; and the round-12
`PackedSamples` typed view + `BitReader` position + channel-indexed
`EntropyInfo` / `Medians::from_entropy` sweep — `PackedSamples`
round-tripping a non-empty payload, the zero-byte empty payload
accepted and reported empty (the wiki places no length constraint on
the `0x0A` payload), `expand_packed_samples` round-tripping the byte
slice, the `bit_reader()` factory starting at byte/bit 0 with the
full payload remaining and yielding the first bit LSB-first, the
factory returning independent readers across multiple calls, an
empty packed-samples view reporting immediate `Error::Truncated` on
any read, and the view being `Copy`; `BitReader::byte_position` /
`bit_position` / `bits_consumed` tracking 13-bit consumption across
a byte boundary, the `bits_consumed` clamp when the reader is past
the end, and the cursor staying put when a read errors with
`Truncated`; `Medians::from_entropy` yielding the left set on index
`0` and the right set on index `1` for a stereo `EntropyInfo`,
returning `None` for `1` on a mono `EntropyInfo`, and rejecting
out-of-range indices (`2`, `3`, `255`); `EntropyInfo::is_stereo`
inverting `is_mono`, `channels` returning `1` for mono and `2` for
stereo, `medians_for_channel` yielding the matched set for `0` / `1`
on stereo and `None` for `1` on mono / `2+` indices; and
`find_packed_samples` returning a typed `PackedSamples` view over a
synthesised `0x0A` sub-block and `None` on a stream without one; and
the round-13 `parse_block` end-to-end aggregate + `BitReader`
look-ahead / skip sweep — header-only block at `ck_size == 24`
yielding an empty sub-blocks list, a two-sub-block walk (`0x00` dummy
+ `0x26` MD5) confirming both walker entries and `contains_sub_block`
predicates, two back-to-back blocks chained through the returned tail,
`Truncated` on a sub-`HEADER_LEN` buffer, the new
`CkSizeExceedsBuffer { ck_size, available }` variant on a header
advertising a payload longer than the buffer (with both fields
checked), header-rejection propagation (`InvalidMagic`, `InvalidCkSize`),
walker-error propagation on a malformed sub-block, `on_disk_len`
equalling `8 + ck_size` and the underlying byte count, and
`contains_sub_block` + `sub_block_count` on a synthesised four-sub-block
block; `peek_bit` returning the next LSB-first bit without advancing
(cursor stays at byte 0 / bit 0, follow-up `get_bit` returns the same
value), `peek_bit` reporting `Truncated` on an empty buffer with the
cursor unchanged, `peek_bits` assembling 4 LSB-first bits of `0x0A`
into `0xA` without advancing, `peek_bits(0)` returning zero without
advancing, `peek_bits(9)` on an 8-bit buffer reporting `Truncated`
with the cursor unchanged, `peek_unary` matching `get_unary` on the
wiki `111110b → 5` example without advancing, `peek_unary` reporting
`Truncated` on an unterminated run with the cursor unchanged, the
peek-then-get pattern returning matching values across a 4-bit window,
`skip_bits` advancing the cursor without assembling a value (with the
expected `bits_consumed` / `byte_position` / `bit_position` after a
5-bit skip and the next `get_bits(3)` reading the remaining bits),
`skip_bits(0)` as a no-op, a 10-bit cross-byte skip landing at
`byte_position == 1` and `bit_position == 2`, `skip_bits(9)` on an
8-bit buffer reporting `Truncated` with the cursor at the buffer end
(matching `get_bits` partial-consume semantics), and a `skip_bits`-
then-`get_unary` resume reading the second of two back-to-back unary
runs.
