//! Pure-Rust WavPack lossless audio codec.
//!
//! **Round 261 — spec §4.2 step 7 sign-bit reconstruction lifted onto
//! the typed surface. The clean-room entropy doc
//! `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §4.2 step 7
//! specifies the last on-wire field of every sample word: "Sign bit,
//! last. After the magnitude is fixed, read exactly one sign bit. If
//! the sign bit is set the returned sample is the bitwise complement of
//! the magnitude (`~mid`), otherwise the magnitude itself." Round 15
//! wired this inline at the tail of [`decode_sample_stateful`] (and
//! round 199 in [`decode_sample_stateful_stereo`]), and round 260's
//! [`SampleInterval::decode_value`] explicitly stopped at the unsigned
//! magnitude — so callers walking the spec ladder by hand could not
//! name step 7 as a typed operation. New [`apply_sign`] is the pure
//! `const` step 7 arithmetic (`magnitude as i32` when the sign bit is
//! clear; `!(magnitude as i32)`, i.e. `-(magnitude + 1)`, when set);
//! new [`read_sign_and_apply`] reads exactly ONE bit from a
//! [`BitReader`] and folds it through [`apply_sign`]; new
//! [`SampleInterval::decode_signed_value`] fuses §4.2 steps 6 + 7 —
//! [`SampleInterval::decode_value`] for the magnitude, then the sign
//! bit — returning the signed `i32` sample, the complete value tail of
//! one sample word after the zone selector is fixed. Both decode loops
//! ([`decode_sample_stateful`] / [`decode_sample_stateful_stereo`]) now
//! delegate their steps 5-8 tail to
//! [`AdaptiveMedians::sample_interval_for_ones_count`] +
//! [`SampleInterval::decode_signed_value`], so the exact bits the loops
//! consume ARE the bits the typed surface consumes (the round-255
//! private `form_interval` tuple shim is now test-only). 15 new tests
//! (559 total, up from 544) pin: `apply_sign` clear-arm verbatim return
//! across magnitudes up to `INTERVAL_MASK_31`; set-arm ones-complement
//! (`0 → -1`, `17 → -18`, `INTERVAL_MASK_31 → i32::MIN`); the
//! set-arm-equals-complement-of-clear-arm identity and the
//! two's-complement `-(m + 1)` identity across `m ∈ [0, 100]`; const
//! evaluability; `read_sign_and_apply` consuming exactly one bit for
//! both sign values; magnitude-verbatim / complement returns;
//! `Error::Truncated` on an empty buffer with the cursor unchanged;
//! degenerate-interval `decode_signed_value` consuming ONLY the sign
//! bit (`[5, 5]` → `5` / `-6` at one bit each); the worked `[17, 33]`
//! example with code 3 yielding `20` (sign clear, 5 bits) and `-21`
//! (sign set); end-to-end round-trip across `maxcode ∈ [0, 20]` ×
//! every code × both signs against `apply_sign(low + code, sign)`;
//! bit-exact parity (value AND cursor) between the fused
//! `decode_signed_value` and the manual `decode_value` +
//! `read_sign_and_apply` two-step; `Truncated` at a missing sign bit
//! after a fully-consumed 8-bit mantissa; and hand-walked spec-ladder
//! parity with [`decode_sample_stateful`] (same sample, same cursor,
//! same post-adapt medians) on a prefix + mantissa + sign stream.**
//!
//! **Round 260 — spec §4.2 step 6 truncated-binary mantissa primitive
//! lifted onto the [`SampleInterval`] surface + spec §3.2 zone-
//! predicate accessors on [`Zone`]. The clean-room entropy doc
//! `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §4.2 step 6
//! first paragraph specifies the truncated-binary mantissa decode the
//! stateful loop reads inside a `(low, high)` interval — `maxcode = 0`
//! consumes no bits and returns `0`; `maxcode = 1` consumes one bit and
//! returns it; `maxcode >= 2` consumes `bitcount - 1` bits LSB-first
//! into `code` with `bitcount = floor(log2(maxcode)) + 1` and
//! `extras = (1 << bitcount) - maxcode - 1`, and reads one MORE bit
//! when `code >= extras` to form the full `bitcount`-bit phase-in
//! codeword. Round 15 wired this in the private `read_truncated_binary`
//! helper inside [`decode_sample_stateful`], but the primitive had not
//! yet been lifted to the public method surface — so callers walking
//! the spec ladder by hand could not name the mantissa decode as a
//! typed operation on the [`SampleInterval`] they already held. New
//! [`SampleInterval::mantissa_bitcount`] is the pure-arithmetic spec
//! `bitcount` (special-cased to `0` for `maxcode == 0`); new
//! [`SampleInterval::mantissa_extras`] is the pure-arithmetic spec
//! `extras` (also `0` for the `maxcode < 2` arms); new
//! [`SampleInterval::decode_mantissa`] consumes a [`BitReader`] and
//! returns the integer `code` in `[0, maxcode]` per the spec ladder;
//! new [`SampleInterval::decode_value`] adds `low` to the decoded
//! mantissa, returning the full §4.2 step 6 magnitude (sign-bit
//! reconstruction still happens at the caller — that is §4.2 step 7).
//! Round 15's private `read_truncated_binary` is now a thin delegator
//! over the new typed surface, so the exact bytes the decode loop
//! consumes ARE the bytes the typed accessor consumes. New on [`Zone`]:
//! [`Zone::index`] returns the zero-based `0/1/2/3` zone arm selector
//! (independent of the carried `ones_count` in the overflow arm);
//! [`Zone::is_overflow`] is the `matches!(self, Zone::Zone2Overflow {
//! .. })` predicate; [`Zone::increments_median`] /
//! [`Zone::decrements_median`] / [`Zone::touches_median`] lift the spec
//! §3.2 per-zone adaptation table onto typed predicates so callers
//! branching on "which median mutates in this zone" do not need to
//! re-walk the inc/dec pattern. The §3.2 mutation itself stays on
//! [`AdaptiveMedians::adapt`] — the new predicates are PURE
//! (do NOT touch the median values). 22 new tests (544 total, up from
//! 522) pin: zone-index numeric mapping + independence from overflow
//! `ones_count`; `is_overflow` only-true for the overflow arm;
//! `increments_median` / `decrements_median` agreement with the spec
//! §3.2 table at every (zone, idx) cell; mutual exclusion (no median
//! is simultaneously incremented and decremented); `touches_median`
//! as the union of inc + dec; out-of-range `idx` rejection
//! (`idx >= 3 → false`); end-to-end predicate parity with the
//! observed [`AdaptiveMedians::adapt`] mutation on synthetic median
//! sets; `mantissa_bitcount` special cases (`0` / `1` / `2` / `16` /
//! `31` / `32` / `INTERVAL_MASK_31`); `mantissa_extras` special
//! cases (`0` for `maxcode < 2`, `1` for `maxcode == 2`, `0` for
//! `maxcode == 2^k - 1`); extras-bounded-by-half-long-region invariant
//! across `maxcode ∈ [2, 1023]`; `decode_mantissa` consumes-no-bits
//! for `maxcode == 0`; degenerate-interval `decode_value` returns
//! `low` directly; `maxcode == 1` reads exactly one bit LSB-first;
//! end-to-end round-trip via `emit_truncated_binary` for every code in
//! `[0, maxcode]` across `maxcode ∈ [0, 32]`; `decode_value` adds
//! `low` correctly for the worked `[17, 33]` example; bit-exact parity
//! with the private `read_truncated_binary` across `maxcode ∈ [0, 40]`
//! (same value AND same cursor position); `Error::Truncated` surfacing
//! on an empty buffer; expected bit-count consumption for the
//! `maxcode == 16` short-form (4 bits) and long-form (5 bits) cases;
//! and end-to-end pairing through [`AdaptiveMedians::sample_interval`]
//! against the worked Zone 0 / `[256, 256, 256]` seed.**
//!
//! **Round 255 — typed [`SampleInterval`] view + new
//! [`AdaptiveMedians::sample_interval`] /
//! [`AdaptiveMedians::sample_interval_for_ones_count`] accessors lifting
//! the spec §4.2 step 5 `(low, high)` interval-formation primitive onto
//! the public method surface. The clean-room entropy doc
//! `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §4.2 step 5
//! specifies the four-arm interval ladder a decoder forms from the
//! three running medians and the (folded) `ones_count` zone selector:
//! zone 0 → `[0, get_med(0) - 1]`; zone 1 → `[get_med(0), low +
//! get_med(1) - 1]`; zone 2 → `[get_med(0) + get_med(1), low +
//! get_med(2) - 1]`; zone-2-overflow `N` → `[m0 + m1 + (N - 2) * m2,
//! low + m2 - 1]`, with both ends masked to 31 bits
//! ([`INTERVAL_MASK_31`]) and `high` clamped up to `low` on underflow.
//! Round 15 wired this in the private `form_interval` helper inside
//! [`decode_sample_stateful`], but the primitive had not yet been
//! lifted to the public method surface — so callers walking the spec
//! ladder by hand (or building diagnostic traces against a known median
//! set) could not name the interval as a typed value. The new
//! [`SampleInterval`] is a `{ low: u32, high: u32 }` value with the
//! `high >= low` invariant; derived accessors expose
//! [`SampleInterval::maxcode`] (`high - low`, the literal `maxcode`
//! the truncated-binary mantissa decoder consumes), [`SampleInterval::width`]
//! (`high - low + 1`, inclusive codeword count),
//! [`SampleInterval::is_degenerate`] (`low == high` single-codeword
//! interval), and [`SampleInterval::contains`] (the `low <= value <=
//! high` membership test). The new
//! [`AdaptiveMedians::sample_interval`] takes a typed [`Zone`] and
//! returns the §4.2-step-5 interval with the 31-bit mask + underflow
//! clamp applied per spec; the [`AdaptiveMedians::sample_interval_for_ones_count`]
//! convenience wrapper composes [`Zone::from_ones_count`] with
//! [`AdaptiveMedians::sample_interval`] for callers holding a raw
//! `ones_count`. The private `form_interval` helper the round-15 decode
//! loop calls is now a thin tuple-shaped delegator over the new typed
//! surface, so the exact `(low, high)` the decoder consumes ARE the
//! values the new typed accessor returns. The spec §3.2 median
//! adaptation step (the mutation that happens at this point in the
//! decode loop) stays on [`AdaptiveMedians::adapt`] — the typed
//! interval formation is PURE (does NOT mutate the medians). 16 new
//! tests (522 total, up from 506) pin: zone 0 / 1 / 2 / overflow
//! formula correctness against hand-traced expected values for the
//! seeds `[256, 256, 256]` worked example (get_med = 17 → intervals
//! `[0,16] / [17,33] / [34,50] / [51,67] / [68,84] / [85,101]`);
//! `sample_interval_for_ones_count` parity with the typed path through
//! [`Zone::from_ones_count`]; `sample_interval` parity with the private
//! `form_interval` across a sweep of median sets (uniform, mixed, zero,
//! max) and zones (`0..=33`); 31-bit masking invariant for `low` and
//! `high` across saturated median configurations; `high >= low`
//! invariant across the same saturated sweep; `maxcode` / `width`
//! arithmetic (`maxcode == high - low`, `width == maxcode + 1`);
//! degenerate-interval predicate (zone 0 with `median[0] == 0` yields
//! `(0, 0)`); zone-2-overflow stepping (`ones_count = 3` is exactly one
//! m2 step past zone 2's `low`, both intervals share the same
//! `maxcode`); `contains` boundary inclusivity (`low` and `high` both
//! inside, `low - 1` and `high + 1` both outside); raw `new`
//! constructor + accessor parity; and field-access parity with the
//! accessors (`i.low == i.low()`, `i.high == i.high()`).**
//!
//! **Round 252 — typed `version` / `track_number` / `track_sub_index`
//! accessors + derived `has_track_id` / `supports_false_stereo`
//! predicates on [`WavPackBlockHeader`] / [`WavPackBlock`]. The wiki
//! "Block structure" listing of
//! `docs/audio/wavpack/wiki/WavPack.wiki` enumerates three explicitly-
//! named header fields the round-1 parser preserved verbatim but had
//! not yet lifted to typed accessors: `16 bits - version (current
//! valid versions are 0x402 - 0x410)` (bytes 8..10), `8 bits - track
//! number (not currently implemented)` (byte 10), and `8 bits - track
//! sub index (not currently implemented)` (byte 11). The wiki "Flags
//! meaning" listing further annotates bit 30 "false stereo" with the
//! explicit gate "version >= 0x410". This round lifts all three
//! fields onto the method surface alongside the round-214 / round-239
//! / round-245 accessors and adds the boolean discriminants the wiki's
//! own annotations imply. New [`WavPackBlockHeader::version`] /
//! [`WavPackBlockHeader::track_number`] /
//! [`WavPackBlockHeader::track_sub_index`] return the stored field
//! verbatim — the parser already constrains `version` to the
//! [`MIN_VERSION`]`..=`[`MAX_VERSION`] window, so the typed accessor
//! is always in the documented range. New
//! [`WavPackBlockHeader::has_track_id`] is the "either track byte is
//! non-zero" boolean discriminant for the wiki "not currently
//! implemented" track-id slot; new
//! [`WavPackBlockHeader::supports_false_stereo`] is the `version >=
//! 0x0410` gate the wiki places on bit 30. Five block-level
//! pass-throughs [`WavPackBlock::version`] /
//! [`WavPackBlock::track_number`] /
//! [`WavPackBlock::track_sub_index`] / [`WavPackBlock::has_track_id`]
//! / [`WavPackBlock::supports_false_stereo`] pair the block surface
//! with the header accessors. All five new surfaces derive directly
//! from explicitly documented wiki fields — no spec gap, no
//! docs-gap-blocked surface touched, no new error variants. 22 new
//! tests (506 total, up from 484) pin: each accessor's verbatim
//! return; the little-endian byte-8..10 decoding of `version` through
//! the full `parse_block_header` path (including a reverse-byte
//! cross-check that the alternate ordering is refused as
//! `UnsupportedVersion`); window-membership of every returned
//! `version`; verbatim round-trip of every byte value in the u8 range
//! for `track_number` / `track_sub_index`; independent byte-10 vs.
//! byte-11 decoding; the four `has_track_id` branches (both-zero,
//! number-only, sub-index-only, both-set); `supports_false_stereo`'s
//! `true` at `0x0410` and `false` at every below-gate in-window
//! value; independence from the round-239 / round-245 accessors; and
//! the block-level pass-throughs' parity with the header accessors
//! across a two-block stream with distinct version + track triples.**
//!
//! **Round 245 — block-CRC accessor on [`WavPackBlockHeader`] /
//! [`WavPackBlock`]. The wiki "Block structure" listing of
//! `docs/audio/wavpack/wiki/WavPack.wiki` places a `32 bits - CRC`
//! word at the trailing 4 bytes of the fixed 32-byte block header.
//! The round-1 parser decoded the word little-endian into the
//! [`WavPackBlockHeader::crc`] field, but no typed accessor surfaced
//! it on either the header or the block-level view — every other
//! documented header field had been lifted to a typed accessor
//! through rounds 214 and 239, so this was the last on-disk header
//! field without a method surface. New [`WavPackBlockHeader::crc`]
//! returns the stored 32-bit word verbatim — the wiki names the
//! field but the staged docs (the wiki listing and the clean-room
//! entropy doc `docs/audio/wavpack/spec/wavpack-entropy-decode.md`)
//! do not specify the polynomial, the byte span, the initial value,
//! or the byte / bit order of the computation, so the typed accessor
//! surfaces the stored bytes only and leaves recomputation to a
//! later round once the algorithm spec lands. New block-level
//! pass-through [`WavPackBlock::crc`] pairs the block surface with
//! the header accessor for callers iterating a multi-block stream
//! off a borrowed `WavPackBlock`. 8 new tests (484 total, up from
//! 476).**
//!
//! **Round 242 — `0x0C` packed-overflow-bits typed view + walker
//! bridges + block-level introspection. The wiki "IDs" listing of
//! `docs/audio/wavpack/wiki/WavPack.wiki` annotates sub-block `0x0C`
//! as "packed overflow bits from floating-point or large integers",
//! and the staged clean-room entropy doc
//! `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §1 names the
//! same ID as the **extension bitstream**. The round-2 metadata
//! walker already routes `0x0C` payloads through the typed
//! [`SubBlockId::PackedOverflowBits`] discriminant, but the bytes had
//! not yet been elaborated into the same typed view + finder + block-
//! level accessor shape the round-12 [`PackedSamples`] (`0x0A`) and
//! round-233 [`PackedCorrectionData`] (`0x0B`) views expose. This
//! round closes that gap. New [`PackedOverflowBits`] in
//! `src/overflow_bits.rs` carries the borrowed `0x0C` payload verbatim
//! and exposes the same `bytes()` / `len()` / `is_empty()`
//! introspection + `bit_reader()` factory shape; new
//! [`expand_packed_overflow_bits`] constructor + walker finders
//! [`find_packed_overflow_bits`] (typed-view variant) /
//! [`find_packed_overflow_bits_sub_block`] (raw-borrow variant)
//! elaborate the walker output without re-walking the metadata. New
//! block-level accessors
//! [`WavPackBlock::has_packed_overflow_bits`] /
//! [`WavPackBlock::packed_overflow_bits`] /
//! [`WavPackBlock::find_packed_overflow_bits_sub_block`] pair the
//! block surface with the new walker bridges. The float / large-
//! integer container fix-ups that would consume the wrapped bytes
//! themselves remain gated on the
//! [`UnsupportedBlockFeature::FloatData`] /
//! [`UnsupportedBlockFeature::Int32Mode`] feature refusals — the typed
//! view is a deferred handoff into the bit reader, not a decode pass.
//! 17 new tests (476 total, up from 459).**
//!
//! **Round 239 — typed file-total / end-cursor accessors on
//! [`WavPackBlockHeader`] / [`WavPackBlock`] + the stream-level
//! [`stream_total_samples`] free function. The wiki "Block structure"
//! listing names three sample-cursor fields — `total samples in file`
//! (with `0xFFFFFFFF` documented as the "unknown" sentinel), `offset
//! in samples for current block`, and `samples in this block` — that
//! the round-1 header parser preserved verbatim but had not yet
//! surfaced as typed `Option`-sentinel accessors or derived end-cursor
//! arithmetic. New [`WavPackBlockHeader::total_samples_in_file`] →
//! `Option<u32>` lifts the file-global total into the same `Option`
//! sentinel-aware shape every other "known / unknown" accessor uses;
//! new [`WavPackBlockHeader::end_sample_index`] returns the `u64`
//! half-open upper bound `block_index + block_samples` (cursor stays
//! put for a metadata-only block per the wiki "may be 0 if no audio
//! present" note); new [`WavPackBlockHeader::samples_remaining_after`]
//! returns `Option<u64>` — `Some(total - end)` for known totals with
//! the end cursor within the file, `None` for the sentinel or for the
//! malformed end-past-total case. New block-level pass-throughs
//! [`WavPackBlock::total_samples_in_file`] /
//! [`WavPackBlock::end_sample_index`] /
//! [`WavPackBlock::samples_remaining_after`] /
//! [`WavPackBlock::is_final_audio_block_in_file`] expose the same
//! header-derived quantities at the block level (the boolean
//! discriminant lights up only when the remainder is exactly zero).
//! New stream-level free function [`stream_total_samples`] returns
//! `Result<Option<Option<u32>>>` — outer `None` for an empty input,
//! outer `Some` carrying the typed file-total from the first block's
//! header (the wiki documents `total_samples` as file-global, so the
//! first block's copy is the stream-level source of truth). All four
//! new surfaces derive directly from the three explicitly-documented
//! wiki fields — no spec gap, no docs-gap-blocked surface touched, no
//! new error variants. 23 new tests (459 total, up from 436).**
//!
//! **Round 233 — `.wvc` correction-stream typed view + walker bridges +
//! block-level / stream-level introspection accessors. The wiki "IDs"
//! listing annotates sub-blocks `0x07` (noise-shaping profile) and
//! `0x0B` (packed correction data) as carried in the `.wvc` companion
//! file alongside the lossy main `.wv`; this round elaborates the
//! round-2 walker output for `0x0B` into a typed
//! [`PackedCorrectionData`] view (analogous to the round-12
//! [`PackedSamples`] view for `0x0A`) and threads the same finder /
//! predicate / iterator pattern through the metadata-walker /
//! block-level / stream-level surfaces. New [`PackedCorrectionData`] is
//! the typed wrap (raw bytes + [`PackedCorrectionData::bit_reader`]
//! factory analogous to [`PackedSamples::bit_reader`]); new
//! [`expand_packed_correction_data`] constructs it from a raw payload.
//! New walker finders [`find_packed_correction_data`] (typed-view
//! variant) / [`find_packed_correction_data_sub_block`] (raw-borrow
//! variant) / [`find_noise_shaping_profile`] / [`find_hybrid_profile`]
//! pair the walker output with the three hybrid-mode sub-block IDs
//! without re-walking the metadata. New block-level accessors
//! [`WavPackBlock::has_packed_correction_data`] /
//! [`WavPackBlock::packed_correction_data`] /
//! [`WavPackBlock::find_packed_correction_data_sub_block`] /
//! [`WavPackBlock::has_noise_shaping_profile`] /
//! [`WavPackBlock::find_noise_shaping_profile_sub_block`] /
//! [`WavPackBlock::has_hybrid_profile`] /
//! [`WavPackBlock::find_hybrid_profile_sub_block`] /
//! [`WavPackBlock::has_correction_stream_data`] expose the new
//! sub-block IDs at the block level. New stream-level free functions
//! [`correction_block_count`] / [`first_correction_block`] /
//! [`iter_correction_blocks`] / [`total_correction_payload_bytes`] and
//! the new [`CorrectionBlockIter`] mirror the round-230 audio-block
//! introspection pattern for correction-stream-bearing blocks. The
//! hybrid-mode sample decode itself (spec §4.2 step 6 second paragraph,
//! `error_limit != 0`) stays out of scope — the typed views give a
//! callable handle into the bytes without committing to a decode
//! semantics. No new error variants and no docs-gap-blocked surface
//! touched. 44 new tests (436 total, up from 392).**
//!
//! **Round 230 — stream-level introspection accessors composing the
//! round-219 [`iter_blocks`] for aggregate "how many / what shape" /
//! "where's the first audio block" questions without retaining the
//! parsed block list. New free functions [`audio_block_count`] /
//! [`metadata_block_count`] split the wiki "Block structure"
//! `block_samples > 0` audio blocks from the `block_samples == 0`
//! metadata-only blocks (RIFF wrappers, MD5 sums, encoding-details);
//! [`total_audio_samples`] sums the wiki "samples in this block" field
//! across audio blocks only (returning `u64` to handle multi-GiB
//! streams); [`decoded_sample_count`] sums the `i32` PCM slot count
//! [`decode_stream`] would produce across all audio blocks (mono /
//! false-stereo contribute `block_samples`, stereo contribute
//! `block_samples * 2`); [`first_audio_block`] peeks the first
//! decode-eligible block past any leading metadata-only blocks. New
//! [`AudioBlockIter`] is the `Clone`-able, `FusedIterator`-compliant
//! lazy iterator yielding only audio blocks; [`iter_audio_blocks`] is
//! the call-shape twin. New [`BlockIter::next_audio`] adapter method
//! skips metadata-only blocks on the existing block iterator. New
//! block-level [`WavPackBlock::decoded_sample_count`] reports the `u64`
//! slot count [`WavPackBlock::decode_samples`] would emit from the
//! header alone (no entropy expansion, no per-sample-loop call) so
//! callers can pre-size a destination buffer in one constant-time
//! step. No new error variants and no docs-gap-blocked surface
//! touched: every error these accessors surface is one [`parse_block`]
//! already raised. 34 new tests (392 total, up from 358).**
//!
//! **Round 224 — multi-block stream → PCM composer fusing the round-219
//! [`BlockIter`] with the round-206 [`WavPackBlock::decode_samples`]
//! into a single byte-buffer → `Vec<i32>` surface. New eager
//! [`decode_stream`] walks every audio block in the input and
//! concatenates the decoded PCM in on-disk order; new
//! [`StreamDecodeIter`] is the `Clone`-able `FusedIterator`-compliant
//! lazy counterpart yielding `Result<Vec<i32>>` once per **audio** block
//! (metadata-only blocks with `block_samples == 0` are silently skipped
//! since they carry no PCM to return — a positive contract preventing a
//! spurious [`Error::BlockHasNoAudio`] refusal on `.wv` files whose
//! first block is a RIFF-header-only metadata block). New free function
//! [`iter_decoded_blocks`] is the `iter_decoded_blocks(bytes)` call-shape
//! twin of [`StreamDecodeIter::new`]. [`StreamDecodeIter`] fuses on the
//! first error (parse or decode); the round-219 [`BlockIter`] fuse + the
//! round-206 refusal taxonomy compose without translation. No new error
//! variants and no docs-gap-blocked surface touched: every error this
//! composer surfaces is one [`parse_block`] or
//! [`WavPackBlock::decode_samples`] already raised. Per-block mono /
//! stereo dispatch (the wiki bit 2 + bit 30 union from round 206) is
//! preserved verbatim, so a multi-block input may mix mono and stereo
//! blocks and the concatenated `Vec<i32>` reflects each block's own
//! shape (mono → `block_samples` i32s; stereo → `block_samples * 2`
//! interleaved). Per-block `0x05` seed re-initialisation (round 206
//! per-block stateless contract) is preserved.**
//!
//! **Round 219 — multi-block stream iteration on top of the round-13
//! [`parse_block`] composer, lifting the wiki "WavPack file consists of
//! blocks each beginning with 'wvpk'" file-format sentence into typed
//! public API. New [`BlockIter`] is a `Clone`-able, `FusedIterator`-
//! compliant lazy iterator yielding `Result<WavPackBlock<'_>>` over the
//! chained-block byte buffer. New free function [`iter_blocks`] is the
//! call-shape twin of [`BlockIter::new`]. New [`parse_blocks`] eagerly
//! collects the iterator into `Vec<WavPackBlock<'_>>`. New
//! [`block_count`] counts blocks without retaining them.
//! [`total_block_samples`] sums the wiki "samples in this block" field
//! across an already-parsed block list, returning `u64` so a
//! 4-GiB-plus stream's sample count does not overflow `u32`. The
//! iterator fuses on the first error so the caller can `?`-bubble the
//! failure without re-encountering it; [`BlockIter::remaining`]
//! continues to point at the malformed block's first byte for offset
//! diagnostics. No new error variants and no docs-gap-blocked surface
//! touched. 18 new tests (337 total).**
//!
//! **Round 214 — block-level discovery / accessor sweep on
//! [`WavPackBlock`] pairing the round-13 [`parse_block`] aggregate with
//! the typed views the existing free finders already build. Adds four
//! header-passthrough accessors ([`WavPackBlock::flags`] /
//! [`WavPackBlock::block_samples`] / [`WavPackBlock::block_index`] /
//! [`WavPackBlock::is_audio_block`]), six presence predicates
//! ([`WavPackBlock::has_entropy_info`] /
//! [`WavPackBlock::has_packed_samples`] /
//! [`WavPackBlock::has_md5_checksum`] /
//! [`WavPackBlock::has_riff_header`] /
//! [`WavPackBlock::has_riff_trailer`] /
//! [`WavPackBlock::has_multichannel_info`]), six borrow finders
//! ([`WavPackBlock::find_sub_block`] /
//! [`WavPackBlock::find_entropy_info_sub_block`] /
//! [`WavPackBlock::find_md5_checksum_sub_block`] /
//! [`WavPackBlock::find_multichannel_info_sub_block`] /
//! [`WavPackBlock::find_riff_header_sub_block`] /
//! [`WavPackBlock::find_riff_trailer_sub_block`]) and three typed
//! extractors ([`WavPackBlock::packed_samples`] →
//! `Option<PackedSamples<'a>>`, [`WavPackBlock::entropy_info`] →
//! `Result<Option<EntropyInfo>>`, [`WavPackBlock::md5_checksum`] →
//! `Result<Option<Md5Checksum>>`). No new error variants and no
//! docs-gap-blocked surface touched. 24 new tests (319 total) pin
//! present / absent / malformed branches across each accessor and an
//! end-to-end pairing with the round-206 [`WavPackBlock::decode_samples`]
//! composer.**
//!
//! **Round 206 — block-level [`WavPackBlock::decode_samples`] composer
//! turning the round-13 [`parse_block`] aggregate into PCM samples in
//! one call. Chains [`find_entropy_info`] + [`expand_entropy`] +
//! [`find_packed_samples`] through the round-201
//! [`decode_packed_samples_mono_from_entropy`] /
//! [`decode_packed_samples_stereo_from_entropy`] wrappers depending on
//! the new [`Flags::is_block_data_mono`] accessor (the union of wiki
//! bit 2 `mono` and wiki bit 30 `false_stereo`). Returns a `Vec<i32>`
//! of `block_samples` mono samples or `block_samples * 2` interleaved
//! stereo samples. New [`UnsupportedBlockFeature`] enum names the
//! seven gated cases ([`UnsupportedBlockFeature::Hybrid`] /
//! [`UnsupportedBlockFeature::FloatData`] /
//! [`UnsupportedBlockFeature::Int32Mode`] /
//! [`UnsupportedBlockFeature::MultichannelMember`] /
//! [`UnsupportedBlockFeature::Decorrelation`] /
//! [`UnsupportedBlockFeature::LowLatencyBlock`] /
//! [`UnsupportedBlockFeature::RobustBlock`]), surfaced through
//! [`Error::UnsupportedBlockFeature`]. New structural errors
//! [`Error::BlockHasNoAudio`] / [`Error::BlockMissingEntropyInfo`] /
//! [`Error::BlockMissingPackedSamples`] cover the
//! "block-doesn't-carry-what-the-composer-needs" shortfalls. New
//! [`WavPackBlock::has_decorrelation`] predicate detects the presence
//! of any `0x02` / `0x03` / `0x04` decorrelation sub-block. New
//! [`Flags::is_block_data_stereo`] is the inverse of the new mono
//! accessor. 23 new tests prove each gate fires and the two happy
//! paths return the expected PCM zero(s) from a synthesised block.
//! 295 tests pass (up from 272).**
//!
//! **Round 199 — stereo per-sample decode loop wiring the
//! `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §2 channel-
//! alternation rule. Adds [`StereoDecodeState`] (per-channel [`RunState`]
//! plus stream-level zero-run debt + `next_channel` parity cursor),
//! [`decode_sample_stateful_stereo`] (one stereo sample per call,
//! dispatching to the channel selected by sample-index parity; the
//! zero-run fast path is gated on BOTH channels' `median[0] <= 1` and
//! resets BOTH channels' medians on a non-zero run per spec §4.2 step
//! 1), and [`decode_packed_samples_stereo`] (end-to-end `frames`
//! → `Vec<i32>` of `frames * 2` interleaved (L,R,L,R,…) PCM samples).
//! 13 new tests prove bit-exact round-trip via per-channel adapt
//! simulators across zone-1 mixed sequences, distinct per-channel
//! seeds, mixed-zones, negative-sign reconstruction, the
//! end-to-end loop wrapper, stereo zero-run BOTH-channel reset +
//! drain across parity, the BOTH-channel zero-run gate (one-channel
//! eligibility rejected), per-channel holding-state independence,
//! truncation cursor preservation, and the EOF escape. 252 tests
//! pass (up from 239).**
//!
//! **Round 15 — stateful per-sample `0x0A` decode loop wiring the
//! staged `docs/audio/wavpack/spec/wavpack-entropy-decode.md` §3 +
//! §3.2 + §4.2 end to end via [`decode_sample_stateful`] (the
//! per-sample primitive) and [`decode_packed_samples_mono`] (the
//! end-to-end loop returning a `Vec<i32>` of mono PCM samples). The
//! per-sample primitive runs the full spec §4.2 sequence: optional
//! §4.2 step 1 zero-run fast path (gated on `get_med(0) <= 1` and no
//! holding bits, carrying `zero_run_pending` across calls in
//! [`DecodeState`]); §4.2 steps 2 + 3 unary prefix with the
//! `LIMIT_ONES = 16` escape and the `cbits == 33` EOF marker (new
//! [`Error::EndOfStream`]); §4.2 step 4 holding-bit fold (via the
//! wiki-compressed [`RunState`] embedded in [`DecodeState::run`]);
//! §4.2 step 5 31-bit-masked `(low, high)` interval; §3.2 per-zone
//! median adaptation BEFORE the mantissa read; §4.2 step 6 first
//! paragraph truncated-binary mantissa; §4.2 step 7 sign bit. New
//! constants [`ESCAPE_EOF_CBITS`] / [`RUN_ESCAPE_CAP`] /
//! [`INTERVAL_MASK_31`] name the spec literals. Hybrid mode
//! (`error_limit != 0`, spec §4.2 step 6 second paragraph) and
//! multi-channel decoding stay out of scope. 18 new tests prove
//! bit-exact round-trip via a spec-derived inverse encoder helper.**
//!
//! **Round 13 — block-header parser + metadata sub-block walker +
//! decorrelation sub-block expanders + entropy-info expander +
//! sample-coding bit reader, run-length decoder, Golomb sample-value
//! reconstruction & single-call per-sample decode + entropy→median
//! bridge + header-accessor coverage + `TermKind` classifier +
//! `DecorrelationTerms` accessors + metadata-sub-block payload-kind
//! predicates + walker finders + MD5 typed view + per-term
//! `decorrelation_sample_count` + flat-payload partitioner + `0x0A`
//! `PackedSamples` typed view + `BitReader` position accessors +
//! channel-indexed `EntropyInfo` / `Medians::from_entropy` bridges +
//! end-to-end `parse_block` / `WavPackBlock` aggregate + `BitReader`
//! non-mutating `peek_bit` / `peek_bits` / `peek_unary` and bulk
//! `skip_bits` advance.**
//! Round 1 landed the structural 32-byte block-header parser
//! documented in `docs/audio/wavpack/wiki/WavPack.wiki` (block-structure
//! listing); round 2 added the metadata sub-block walker following the
//! wiki "Metadata" section; round 3 adds typed expanders for the three
//! decorrelation sub-blocks — `0x02` terms, `0x03` weights, and
//! `0x04` samples — per the wiki "Decorrelation terms",
//! "Decorrelation weights" and "Decorrelation samples" sections;
//! round 4 adds [`expand_entropy`] for the `0x05` entropy-info
//! sub-block (one or two sets of three 16-bit log-packed medians per
//! the wiki "Entropy info" section); round 5 adds the [`BitReader`]
//! primitives (`get_unary` / `get_bit` / `get_bits`) and
//! [`decode_run_length`] — the first half of the wiki "Samples coding"
//! pseudocode (the unary-prefix `n`-decoder with the `n == 16` escape
//! and the adaptive `last_zero` / `last_one` carry); round 6 adds the
//! second (value) half — [`golomb_interval`] and [`decode_sample_value`]
//! — that maps `n` + [`Medians`] onto a `(base, add)` interval and reads
//! the mantissa / sign, leaving the median-update *amount* (an open docs
//! gap) for a later round. See [`expand_terms`], [`expand_weights`],
//! [`expand_samples`], [`expand_entropy`], [`decode_run_length`] and
//! [`decode_sample_value`].
//!
//! Round-1 scope (preserved):
//!
//! * The four-byte `'w','v','p','k'` magic.
//! * The 32-bit little-endian `ck_size` (block size not counting the
//!   magic or this field).
//! * The 16-bit `version` (valid range `0x0402..=0x0410`).
//! * The 8-bit `track_number` and `track_sub_index`.
//! * The 32-bit `total_samples` (with the `0xFFFF_FFFF` "unknown"
//!   sentinel).
//! * The 32-bit `block_index` and `block_samples`.
//! * The 32-bit `flags` word, decoded into a typed [`Flags`] view that
//!   exposes every bit-range named on the wiki "Flags meaning"
//!   listing.
//! * The trailing 32-bit `crc` (preserved verbatim — checksum
//!   verification requires sample decode, which lands in a later
//!   round).
//!
//! Round-2 scope (preserved):
//!
//! * [`walk_metadata`] — consumes a byte slice (the post-header
//!   payload from [`parse_block_header`]) and returns a
//!   `Vec<MetadataSubBlock>` of typed `(SubBlockId, payload)` pairs.
//! * [`parse_metadata_sub_block`] — single-step walker the caller
//!   can drive themselves when validating against `ck_size`.
//! * [`SubBlockId`] — typed enum naming every ID listed by the
//!   wiki "IDs" section (`0x00..=0x0D` + `0x20..=0x27`). Unknown
//!   IDs are surfaced as `Unknown(u8)` rather than rejected.
//! * [`SubBlockFlags`] — typed view of the `0x20` / `0x40` /
//!   `0x80` flag triple decoded from the on-disk ID byte.
//!
//! Round-3 scope adds the decorrelation expanders:
//!
//! * [`expand_terms`] — converts a `0x02` payload into a
//!   [`DecorrelationTerms`] (`terms: Vec<i8>`, `deltas: Vec<u8>`),
//!   one byte → one `(term, delta)` pair per the wiki "lower 5 bits
//!   indicate predictor type, high 3 bits contain delta value"
//!   sentence.
//! * [`expand_weights`] — converts a `0x03` payload into a
//!   [`DecorrelationWeights`] (`weights: Vec<i32>`), applying the
//!   wiki two-line log-pack expansion
//!   (`n = getchar() << 3; if (n > 0) n += (n + 64) >> 7`) to every
//!   byte.
//! * [`expand_samples`] — converts a `0x04` payload into a
//!   [`DecorrelationSamples`] (`samples: Vec<i32>`), reading
//!   little-endian 16-bit words and applying the wiki exponent /
//!   mantissa expansion (mantissa is signed, exponent is biased by
//!   `-9`).
//!
//! Round-4 scope adds the entropy-info expander:
//!
//! * [`expand_entropy`] — converts a `0x05` payload into an
//!   [`EntropyInfo`] (`medians_left: [i32; 3]`,
//!   `medians_right: [i32; 3]`), reading three (mono) or six (stereo)
//!   little-endian 16-bit words through the same log-pack the round-3
//!   sample expander uses. Mono payloads (6 bytes) leave
//!   `medians_right` at `[0; 3]`; stereo payloads (12 bytes) populate
//!   both. Other lengths are rejected as malformed.
//!
//! Round-5 scope adds the sample-coding bit reader and run-length
//! decoder (first half of the wiki "Samples coding" section):
//!
//! * [`BitReader`] — least-significant-bit-first reader over a `0x0A`
//!   payload exposing the three wiki primitives `get_unary()`,
//!   `get_bit()` and `get_bits(n)`. Reads past the buffer report
//!   [`Error::Truncated`].
//! * [`decode_run_length`] — turns the unary prefix (with the
//!   `n == 16` escape) into the halved run-length index `n`, carrying
//!   the adaptive `last_zero` / `last_one` state in [`RunState`].
//!
//! Round-6 scope adds the Golomb *value* half of the same section — the
//! `(base, add)` interval selection plus the mantissa / sign
//! reconstruction — stopping short only of the median adaptation:
//!
//! * [`Medians`] — a channel's three medians (`median[0..=2]`) as the
//!   `0x05` entropy-info expander produces them.
//! * [`golomb_interval`] — pure `n` + [`Medians`] → [`GolombInterval`]
//!   `(base, add)` mapping per the wiki's three-way branch.
//! * [`decode_sample_value`] — reads `getbits(k - 1)` (with `k =
//!   log2(add)` under the wiki-derived bit-length reading), the
//!   `t2 >= ex` extra bit, and the sign, returning the reconstructed
//!   sample. Takes [`Medians`] **by value** and does not mutate them:
//!   the median "increase" / "decrease" *amount* is still an open docs
//!   gap, so the stateful full-payload loop is deferred. The degenerate
//!   `add == 0` (median `1`) interval is rejected via
//!   [`Error::GolombDegenerateInterval`] rather than guessed.
//!
//! Round-7 scope joins the two halves into a single per-sample call and
//! bridges the round-4 entropy-info output into the round-6 median set:
//!
//! * [`decode_sample`] — runs the whole wiki "Samples coding" per-sample
//!   pseudocode in one call: [`decode_run_length`] (with its adaptive
//!   [`RunState`] carry) followed by [`decode_sample_value`]. Still takes
//!   [`Medians`] by value and does not mutate them — the median
//!   adaptation amount remains the open docs gap — so it is a
//!   single-sample primitive, not yet the full payload loop.
//! * [`Medians::from_entropy_left`] / [`Medians::from_entropy_right`] —
//!   pull a channel's three medians straight out of an
//!   [`EntropyInfo`] so the round-4 expander output feeds the round-6
//!   Golomb decoder without the caller re-typing the array.
//!
//! Round-9 scope (the previous header-accessor round 8 is preserved) adds
//! typed classification on top of the round-3 / round-2 expander output —
//! still no bit-stream advancement past the docs-gap line:
//!
//! * [`TermKind`] — classifies a decorrelation predictor code per the
//!   wiki "Possible predictor values" listing; `is_implemented()` and
//!   `previous_samples()` surface the wiki's "only predictors 2-4 are
//!   implemented" and "6-12 uses 1-7 samples" narrowings.
//! * [`DecorrelationTerms`] accessors `len`/`is_empty`/`kind_at`/
//!   `iter_kinds`/`all_implemented`/`has_reserved` classify the
//!   round-3 term list without re-walking the bytes.
//! * [`weights_per_term`] — wiki "Each decorrelation term should have
//!   one or two weights depending on channels" mono/stereo split.
//! * [`MetadataSubBlock`] payload-kind predicates `is_optional` /
//!   `is_decorrelation_payload` / `is_correction_payload` /
//!   `is_audio_payload` / `is_riff_payload` group the round-2 walker
//!   output for callers picking the decorrelation triple or the audio
//!   stream out of a walk.
//!
//! Round-12 scope adds the typed `0x0A` packed-samples view, the
//! [`BitReader`] position accessors and the channel-indexed
//! [`Medians::from_entropy`] / [`EntropyInfo::medians_for_channel`]
//! bridges — non-prediction-loop elaborations of the existing typed
//! views while the median-adaptation amount stays a docs gap:
//!
//! * [`PackedSamples`] — typed view of the `0x0A` packed-samples
//!   sub-block payload (the entropy-coded audio bitstream the wiki
//!   "Samples coding" section consumes). Carries the borrowed payload
//!   bytes; exposes [`PackedSamples::bytes`] / [`PackedSamples::len`] /
//!   [`PackedSamples::is_empty`] introspection and a
//!   [`PackedSamples::bit_reader`] factory that yields a fresh
//!   [`BitReader`] positioned at bit 0 for feeding
//!   [`decode_run_length`] / [`decode_sample_value`] / [`decode_sample`].
//! * [`expand_packed_samples`] — the round-2 walker output ↦ typed view
//!   bridge for the `0x0A` ID (analogous to [`expand_samples`] /
//!   [`expand_entropy`] for `0x04` / `0x05`, but a typed wrap rather
//!   than a byte-by-byte decode because the wiki places no internal
//!   structure on the `0x0A` payload).
//! * [`find_packed_samples`] — walker convenience finder returning a
//!   [`PackedSamples`] directly (the typed counterpart to
//!   [`find_audio_payload`]).
//! * [`BitReader::byte_position`] / [`BitReader::bit_position`] /
//!   [`BitReader::bits_consumed`] — cursor accessors for callers that
//!   want to log the position before a truncation error fires or
//!   resume from a known offset against a fresh reader.
//! * [`EntropyInfo::is_stereo`] / [`EntropyInfo::channels`] /
//!   [`EntropyInfo::medians_for_channel`] — typed channel introspection
//!   pinning the wiki "one or two sets of medians" sentence as `1` or
//!   `2` populated sets, with a channel-indexed median getter that
//!   returns `None` for out-of-range indices and for the right channel
//!   on a mono payload.
//! * [`Medians::from_entropy`] — channel-indexed bridge over
//!   [`EntropyInfo`] returning `Some(medians)` for `0` / `1` (the
//!   latter only on stereo) so callers iterating per-channel medians
//!   skip the mono / stereo branch.
//!
//! Round-11 scope adds the per-term decorrelation-sample-count helper
//! and the flat-payload partitioner, both derived from the wiki
//! "Decorrelation samples" / "Possible predictor values" sections:
//!
//! * [`decorrelation_sample_count`] / [`TermKind::decorrelation_sample_count`]
//!   — `Some(code - 5)` for the `6..=12` sample-based codes (one seed
//!   sample per previous-sample slot), `Some(2)` for the `17..=18`
//!   two-sample codes, and `None` for stereo `0..=5`, the reserved
//!   `13..=16` range, and undocumented codes (per-term count not given
//!   by the wiki).
//! * [`DecorrelationTerms::expected_decorrelation_sample_count`] sums
//!   the above across a term list and short-circuits to `None` on
//!   any docs-gap code.
//! * [`partition_decorrelation_samples`] splits the flat
//!   [`DecorrelationSamples`] list `expand_samples` produces into one
//!   `Vec<i32>` per term, refusing docs-gap codes via
//!   [`Error::DecorrelationSampleCountUnspecified`] and length
//!   mismatches via [`Error::DecorrelationSampleCountMismatch`].
//! * [`MAX_DECORRELATION_SAMPLES_PER_TERM`] (= 16) surfaces the wiki
//!   "up to 16 samples" upper bound for callers checking future docs
//!   additions against it.
//!
//! Round-13 scope adds the end-to-end [`parse_block`] composer and the
//! non-mutating [`BitReader::peek_bit`] / [`BitReader::peek_bits`] /
//! [`BitReader::peek_unary`] look-ahead primitives plus the bulk
//! [`BitReader::skip_bits`] advance — both groups stay clear of the
//! median-adaptation docs gap and elaborate the existing structural /
//! bit-level surfaces:
//!
//! * [`parse_block`] combines round 1's
//!   [`parse_block_header`] with round 2's [`walk_metadata`], returning
//!   a [`WavPackBlock`] aggregate (header + parsed sub-blocks) and the
//!   tail bytes ready for the next block in a multi-block `.wv` file.
//!   The new [`Error::CkSizeExceedsBuffer`] variant reports the
//!   distinct "buffer ran out mid-payload" case so a streaming caller
//!   knows how many more bytes to read.
//! * [`BitReader::peek_bit`] / [`BitReader::peek_bits`] /
//!   [`BitReader::peek_unary`] read a single bit, a multi-bit value or
//!   a unary run-length without consuming the bits — implemented by
//!   reading from a clone, so the wiki bit-order rules carry through
//!   unchanged.
//! * [`BitReader::skip_bits`] advances the reader by an arbitrary count
//!   of bits without assembling a `u32`, matching the partial-consume
//!   semantics of [`BitReader::get_bits`] on truncation (cursor lands
//!   at the buffer end rather than reverting).
//!
//! Still out of scope (subsequent rounds): the median-adaptation
//! *amount* that turns `decode_sample_value` into a stateful payload
//! loop (blocked on a docs gap), the prediction loop that consumes the
//! decorrelation typed views, the per-term sample count for stereo
//! predictors `0..=5` (open docs gap — round 11's partitioner refuses
//! them), float-data / large-or-shifted-int / overflow-bits
//! interpretation, multichannel channel-mask handling, hybrid
//! correction-stream (`.wvc`) pairing, CRC32 verification, encoder.
//!
//! ## Clean-room provenance
//!
//! All work in this crate has been implemented strictly against the
//! staged WavPack documentation under `docs/audio/wavpack/` (the
//! `wiki/WavPack.wiki` snapshot in tree and, from round 15 onward,
//! the `spec/wavpack-entropy-decode.md` clean-room trace). No
//! external library source, no archived prior history of this crate,
//! and no online resource outside the staged docs were consulted at
//! any phase.

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

mod block;
mod block_header;
mod correction;
mod decorrelation;
mod entropy;
mod error;
mod metadata;
mod overflow_bits;
mod packed_samples;
mod samples;

pub use crate::block::{
    audio_block_count, block_count, correction_block_count, decode_stream, decoded_sample_count,
    first_audio_block, first_correction_block, iter_audio_blocks, iter_blocks,
    iter_correction_blocks, iter_decoded_blocks, metadata_block_count, parse_block, parse_blocks,
    stream_total_samples, total_audio_samples, total_block_samples, total_correction_payload_bytes,
    AudioBlockIter, BlockIter, CorrectionBlockIter, StreamDecodeIter, UnsupportedBlockFeature,
    WavPackBlock,
};
pub use crate::block_header::{
    parse_block_header, Flags, WavPackBlockHeader, HEADER_LEN, MAGIC, MAX_VERSION, MIN_CK_SIZE,
    MIN_VERSION, TOTAL_SAMPLES_UNKNOWN,
};
pub use crate::correction::{expand_packed_correction_data, PackedCorrectionData};
pub use crate::decorrelation::{
    decorrelation_sample_count, expand_samples, expand_terms, expand_weights,
    partition_decorrelation_samples, weights_per_term, DecorrelationSamples, DecorrelationTerms,
    DecorrelationWeights, TermKind, MAX_DECORRELATION_SAMPLES_PER_TERM, MAX_DOCUMENTED_TERM,
    SAMPLE_EXPONENT_BIAS, SAMPLE_ON_WIRE_BYTES, TERM_DELTA_BITS, TERM_DELTA_MASK,
    TERM_PREDICTOR_BITS, TERM_PREDICTOR_MASK,
};
pub use crate::entropy::{
    expand_entropy, EntropyInfo, MEDIANS_PER_CHANNEL, MEDIAN_ON_WIRE_BYTES, MONO_PAYLOAD_BYTES,
    STEREO_PAYLOAD_BYTES,
};
pub use crate::error::{Error, Result};
pub use crate::metadata::{
    find_audio_payload, find_decorrelation_triple, find_entropy_info, find_first,
    find_hybrid_profile, find_md5_checksum_block, find_multichannel_info,
    find_noise_shaping_profile, find_packed_correction_data, find_packed_correction_data_sub_block,
    find_packed_overflow_bits, find_packed_overflow_bits_sub_block, find_packed_samples,
    parse_md5_checksum, parse_metadata_sub_block, walk_metadata, Md5Checksum, MetadataSubBlock,
    SubBlockFlags, SubBlockId, ID_FLAG_LARGE_SIZE, ID_FLAG_ODD_SIZE, ID_FLAG_OPTIONAL, ID_MASK,
    MD5_DIGEST_BYTES,
};
pub use crate::overflow_bits::{expand_packed_overflow_bits, PackedOverflowBits};
pub use crate::packed_samples::{expand_packed_samples, PackedSamples};
pub use crate::samples::{
    apply_sign, decode_packed_samples_mono, decode_packed_samples_mono_from_entropy,
    decode_packed_samples_stereo, decode_packed_samples_stereo_from_entropy, decode_run_length,
    decode_sample, decode_sample_stateful, decode_sample_stateful_stereo, decode_sample_value,
    golomb_interval, read_folded_ones_count, read_raw_prefix, read_sign_and_apply, AdaptiveMedians,
    BitReader, DecodeState, GolombInterval, Medians, RunState, SampleInterval, StereoDecodeState,
    Zone, DIV0, DIV1, DIV2, ESCAPE_EOF_CBITS, GET_MED_FLOOR, GET_MED_SHIFT, INTERVAL_MASK_31,
    MEDIAN_DEC_MULTIPLIER, MEDIAN_INC_MULTIPLIER, RUN_ESCAPE_CAP, UNARY_ESCAPE,
};
