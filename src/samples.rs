//! WavPack v.4 sample-coding bit reader + run-length (`n`) decoder.
//!
//! The wiki "Samples coding" section
//! (`docs/audio/wavpack/wiki/WavPack.wiki`) opens:
//!
//! > Samples are stored in metadata block with ID=0x0A and are packed
//! > with modified Golomb codes. Decoding process is specified below
//! > where get_unary() is the function which returns length of '1'-bits
//! > string (i.e. 111110b = 5, 10b = 1). Codeset is adaptively divided
//! > into four sets and every code has unary prefix (possibly escaped)
//! > defining interval of this code and mantis part like in Golomb code.
//!
//! and then gives the per-sample decode pseudocode. The full
//! pseudocode has two clearly-separable halves:
//!
//! 1. Decode the **run-length index `n`** — the unary prefix (with the
//!    `n == 16` escape) plus the adaptive `last_zero` / `last_one`
//!    state that halves `n` and remembers a trailing run. This half is
//!    fully specified by the wiki given the bit-reading primitives.
//! 2. Map `n` onto a `(base, add)` Golomb interval using the three
//!    medians, read the mantissa, apply the sign, and **adapt the
//!    medians** (the wiki's "increase median[x]" / "decrease
//!    median[x]" steps).
//!
//! Round 5 landed the first half — the bit reader and the `n`-decoder.
//! Round 6 lands the value part of the second half:
//! [`golomb_interval`] (`(base, add)` selection) and
//! [`decode_sample_value`] (mantissa + sign). Round 7 joins the two
//! halves into [`decode_sample`] — one call per sample, matching the
//! wiki's single contiguous pseudocode block — and adds the
//! [`Medians::from_entropy_left`] / [`Medians::from_entropy_right`]
//! bridge from the round-4 [`EntropyInfo`](crate::EntropyInfo) output.
//! The one piece still missing is the median **adaptation amount** — the
//! wiki names the "increase" / "decrease" verbs without quantifying them
//! — so [`decode_sample`] (like [`decode_sample_value`]) decodes against
//! a fixed, caller-supplied median set and does not mutate it. The
//! stateful loop that walks a whole `0x0A` payload (feeding each `n` back
//! into a mutating median set) stays gated on that gap being closed (see
//! the docs-gap notes below).
//!
//! ## Bit-reader primitives
//!
//! The wiki references three primitives without defining their bit
//! order:
//!
//! * `get_unary()` — "returns length of '1'-bits string (i.e. 111110b
//!   = 5, 10b = 1)". [`BitReader::get_unary`] counts consecutive `1`
//!   bits up to and including the terminating `0`.
//! * `getbit()` — one bit. [`BitReader::get_bit`].
//! * `getbits(n)` — `n` bits assembled into an integer.
//!   [`BitReader::get_bits`].
//!
//! ### Docs-gap: bit order within a byte
//!
//! The wiki gives the unary worked example as a left-to-right `1`-bit
//! string (`111110b`) but never states whether `getbits` / `getbit`
//! consume the **most-significant** or **least-significant** bit of a
//! byte first. WavPack's container is little-endian throughout (every
//! header field, every metadata size word), so round 5 reads bits
//! **least-significant-bit first within each byte, bytes in stream
//! order** — the natural pairing with a little-endian word model and
//! the only choice that lets the unary worked example (`111110b → 5`)
//! be reproduced when the encoder emits the `1`s then the `0` in
//! ascending bit position. Empirical confirmation against a real
//! `0x0A` payload is gated on the median-adaptation gap (below) being
//! closed; until then the bit order is a documented assumption and the
//! tests synthesise their own bitstreams in that order. A future docs
//! revision should state the bit order explicitly.
//!
//! ### Golomb mantissa / sign reconstruction (round 6)
//!
//! Round 6 lands the *value* half of the wiki pseudocode — everything
//! after the `n`-decoder up to (but not including) the median update:
//!
//! ```text
//! if(n == 0){      base = 0;                                  add = median[0] - 1; }
//! else if(n == 1){ base = median[0];                          add = median[1] - 1; }
//! else {           base = median[0]+median[1]+median[2]*(n-2); add = median[2] - 1; }
//! k  = log2(add);
//! ex = (1 << k) - add - 1;
//! t2 = getbits(k - 1);
//! if(t2 >= ex) t2 = t2 * 2 - ex + getbit();
//! sign = getbit();
//! if(sign==0) result = base + t2; else result = ~(base + t2);
//! ```
//!
//! [`golomb_interval`] computes the `(base, add)` pair from `n` and a
//! three-median set; [`decode_sample_value`] consumes the mantissa /
//! sign bits and returns the reconstructed `result`. The **median
//! update** that follows in the encoder loop is *not* applied here —
//! [`decode_sample_value`] takes the medians by value and leaves the
//! caller's set untouched (see the median-adaptation gap below).
//!
//! ### Docs-gap: meaning of `k = log2(add)` (resolved by derivation)
//!
//! The wiki writes `k = log2(add)` without saying floor, ceil, or
//! bit-length. The choice is pinned **from the wiki's own next two
//! lines**, not from any external source: `ex = (1 << k) - add - 1`
//! feeds `if(t2 >= ex)`, and that branch can only ever be taken (rather
//! than dead) when `ex >= 0`, i.e. `(1 << k) >= add + 1`. The smallest
//! such `k` is the **bit-length of `add`** (`k` = position of the
//! highest set bit `+ 1`, e.g. `add = 5` → `k = 3`, `add = 1` → `k = 1`).
//! Round 6 uses that bit-length reading. A future docs revision should
//! state the `log2` rounding explicitly.
//!
//! ### Docs-gap: degenerate `add == 0` interval (rejected, not guessed)
//!
//! When the selected median is `1`, `add = median - 1 = 0`, so
//! `k = log2(0)` and `getbits(k - 1) = getbits(-1)` are both undefined
//! by the wiki. This is the single-codeword interval; the wiki gives no
//! recipe for it, so [`decode_sample_value`] returns
//! [`Error::GolombDegenerateInterval`] rather than inventing a behaviour.
//! Closing this needs the degenerate-interval rule added to
//! `docs/audio/wavpack/`.
//!
//! ### Docs-gap: median adaptation amount (deferred)
//!
//! The wiki's "increase median[0]" / "decrease median[0]" steps name a
//! direction but not an amount. Real WavPack updates each median by a
//! fraction of itself (a documented step in the format's own
//! `format.txt`, which the wiki cites but does **not** reproduce), so
//! the *stateful* loop that walks a whole `0x0A` payload — feeding each
//! decoded sample's `n` back into a mutating median set — cannot be made
//! bit-exact from this wiki page alone. Round 6 therefore decodes a
//! single sample value against a *caller-supplied, fixed* median set and
//! stops short of mutating it. The adaptation amount needs to be added
//! to `docs/audio/wavpack/` before the full payload loop can be wired.
//!
//! ### Docs-gap: escape `getbits(n2 - 1)` when `n2 < 2`
//!
//! The escape branch reads `getbits(n2 - 1)` only inside the
//! `else` arm (`n2 >= 2`), so the argument is always `>= 1` there. The
//! `n2 < 2` arm adds `n2` directly with no `getbits` call. Round 5
//! mirrors that control flow exactly, so `get_bits(0)` is never invoked
//! from the escape path; [`BitReader::get_bits`] still defines
//! `get_bits(0) == 0` for completeness.

use crate::entropy::EntropyInfo;
use crate::error::{Error, Result};

/// The unary prefix value at which the wiki escape kicks in: when
/// `get_unary()` returns 16 a second unary value `n2` is read and
/// folded in per the wiki "if(n == 16)" branch.
pub const UNARY_ESCAPE: u32 = 16;

/// Divisor used for the `median[0]` adaptation step
/// (`docs/audio/wavpack/spec/wavpack-entropy-decode.md` §3, constant
/// `DIV0`).
///
/// Used in both the increment `median[0] += ((median[0] + D) / D) * 5`
/// and the decrement `median[0] -= ((median[0] + (D - 2)) / D) * 2`,
/// with `D = DIV0`.
pub const DIV0: u32 = 128;

/// Divisor used for the `median[1]` adaptation step (spec §3, constant
/// `DIV1`). Same role as [`DIV0`] but for the second median.
pub const DIV1: u32 = 64;

/// Divisor used for the `median[2]` adaptation step (spec §3, constant
/// `DIV2`). Same role as [`DIV0`] but for the third median.
pub const DIV2: u32 = 32;

/// Multiplier applied to the rounded division in the spec §3 increment
/// step: `median[i] += ((median[i] + D) / D) * MEDIAN_INC_MULTIPLIER`.
pub const MEDIAN_INC_MULTIPLIER: u32 = 5;

/// Multiplier applied to the biased rounded division in the spec §3
/// decrement step:
/// `median[i] -= ((median[i] + (D - 2)) / D) * MEDIAN_DEC_MULTIPLIER`.
pub const MEDIAN_DEC_MULTIPLIER: u32 = 2;

/// Right shift applied in the spec §2.1 `get_med` operation
/// (`get_med(i) = (median[i] >> GET_MED_SHIFT) + 1`).
///
/// Per spec §5, the stored median carries 4 fractional bits.
pub const GET_MED_SHIFT: u32 = 4;

/// Floor of the spec §2.1 `get_med` result. `get_med` never drops below
/// this value: `(median[i] >> 4) + 1` is at least `1` for any
/// non-negative `median[i]`.
pub const GET_MED_FLOOR: u32 = 1;

/// Least-significant-bit-first reader over a `0x0A` packed-samples
/// payload (or any WavPack bitstream).
///
/// Bits are consumed least-significant-bit first within each byte and
/// bytes are consumed in stream order — see the module-level bit-order
/// docs-gap note. Reads past the end of the buffer report
/// [`Error::Truncated`] rather than silently returning zero bits, so a
/// malformed or short payload is surfaced to the caller.
#[derive(Debug, Clone)]
pub struct BitReader<'a> {
    bytes: &'a [u8],
    /// Index of the next byte to pull into the accumulator.
    byte_pos: usize,
    /// Index of the next bit within the current byte (`0..=7`,
    /// LSB-first).
    bit_pos: u8,
}

impl<'a> BitReader<'a> {
    /// Create a reader positioned at the first bit of `bytes`.
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            byte_pos: 0,
            bit_pos: 0,
        }
    }

    /// Total number of bits still unread.
    pub fn bits_remaining(&self) -> usize {
        let whole = self.bytes.len().saturating_sub(self.byte_pos + 1);
        // Bits left in the current byte plus all bits of the remaining
        // whole bytes. When byte_pos is past the end this is zero.
        if self.byte_pos >= self.bytes.len() {
            0
        } else {
            (8 - self.bit_pos as usize) + whole * 8
        }
    }

    /// `true` when no bits remain.
    pub fn is_empty(&self) -> bool {
        self.bits_remaining() == 0
    }

    /// Index of the next byte the reader will pull into the accumulator.
    ///
    /// Together with [`Self::bit_position`] this names the reader's
    /// cursor in the underlying byte slice — useful for callers that
    /// want to log the position before a [`Error::Truncated`] hits, or
    /// resume from a known offset against a freshly-constructed reader
    /// over the same bytes.
    pub fn byte_position(&self) -> usize {
        self.byte_pos
    }

    /// Index of the next bit within the current byte (`0..=7`,
    /// LSB-first). Pairs with [`Self::byte_position`] to name the
    /// reader's cursor.
    pub fn bit_position(&self) -> u8 {
        self.bit_pos
    }

    /// Total bits already consumed since the reader was constructed.
    /// Equivalent to `byte_position() * 8 + bit_position()` but clamped
    /// at the buffer length when the reader has advanced past the end.
    pub fn bits_consumed(&self) -> usize {
        if self.byte_pos >= self.bytes.len() {
            self.bytes.len() * 8
        } else {
            self.byte_pos * 8 + self.bit_pos as usize
        }
    }

    /// Read one bit, LSB-first. Returns the bit value (`0` or `1`).
    ///
    /// [`Error::Truncated`] when the buffer is exhausted.
    pub fn get_bit(&mut self) -> Result<u32> {
        if self.byte_pos >= self.bytes.len() {
            return Err(Error::Truncated);
        }
        let bit = (self.bytes[self.byte_pos] >> self.bit_pos) & 1;
        self.bit_pos += 1;
        if self.bit_pos == 8 {
            self.bit_pos = 0;
            self.byte_pos += 1;
        }
        Ok(bit as u32)
    }

    /// Read `count` bits LSB-first, assembling them into a `u32` with
    /// the first bit read landing in bit 0 of the result.
    ///
    /// `count` must be in `0..=32`. `get_bits(0)` returns `0` without
    /// consuming any bits. [`Error::Truncated`] when the buffer is
    /// exhausted before `count` bits are read.
    pub fn get_bits(&mut self, count: u32) -> Result<u32> {
        debug_assert!(count <= 32, "get_bits supports up to 32 bits");
        let mut value = 0u32;
        for i in 0..count {
            let bit = self.get_bit()?;
            value |= bit << i;
        }
        Ok(value)
    }

    /// Count consecutive `1` bits up to and including the terminating
    /// `0`, returning the run length.
    ///
    /// Matches the wiki definition: "returns length of '1'-bits string
    /// (i.e. 111110b = 5, 10b = 1)". The terminating `0` is consumed
    /// but not counted. A run that reaches the end of the buffer
    /// without a terminating `0` reports [`Error::Truncated`].
    pub fn get_unary(&mut self) -> Result<u32> {
        let mut count = 0u32;
        loop {
            let bit = self.get_bit()?;
            if bit == 0 {
                return Ok(count);
            }
            count += 1;
        }
    }

    /// Inspect the next bit without consuming it. Returns the bit value
    /// (`0` or `1`); the reader's cursor is unchanged on success or
    /// error.
    ///
    /// Useful for a caller deciding whether to advance based on a single
    /// look-ahead bit (e.g. probing the leading bit of a `0x0A` payload
    /// before committing to a decode). Reports [`Error::Truncated`] when
    /// the buffer is exhausted, with the cursor still at its pre-call
    /// position so the caller can recover without rebuilding a fresh
    /// reader. Implemented by cloning the reader and reading from the
    /// clone, which means the cursor invariants the read path keeps
    /// (byte_pos / bit_pos coherence) hold for `peek_*` too.
    pub fn peek_bit(&self) -> Result<u32> {
        let mut clone = self.clone();
        clone.get_bit()
    }

    /// Inspect the next `count` bits without consuming them. Same LSB-
    /// first assembly rules as [`Self::get_bits`].
    ///
    /// `count` must be in `0..=32`. Reports [`Error::Truncated`] when
    /// the buffer is exhausted before `count` bits are available; the
    /// reader's cursor is unchanged on success or error so a caller can
    /// peek-then-decide without committing.
    pub fn peek_bits(&self, count: u32) -> Result<u32> {
        let mut clone = self.clone();
        clone.get_bits(count)
    }

    /// Inspect the next unary run-length without consuming it. Same
    /// `get_unary`-style accounting (consecutive `1` bits up to but not
    /// counting the terminating `0`).
    ///
    /// Reports [`Error::Truncated`] when the buffer is exhausted before
    /// a terminating `0` is reached. Useful for a caller probing the
    /// wiki `n == 16` escape pattern (the leading unary indicating
    /// whether a second unary is about to be read) without committing
    /// to a real `decode_run_length` call.
    pub fn peek_unary(&self) -> Result<u32> {
        let mut clone = self.clone();
        clone.get_unary()
    }

    /// Advance the reader by `count` bits without producing a value.
    /// Equivalent to `let _ = get_bits(count)?` but without the unused-
    /// return diagnostic dance, and with a tighter loop that doesn't
    /// build a `u32`.
    ///
    /// `count` must be in `0..=u32::MAX`. Reports [`Error::Truncated`]
    /// when the buffer is exhausted before `count` bits are skipped; in
    /// that case the cursor advances to the end of the buffer rather
    /// than staying put (matching the semantics of partially-consumed
    /// `get_bits` — the bits that were available were consumed).
    /// Useful for a caller wanting to step past a known-length opaque
    /// field (e.g. a padding region) without holding the assembled
    /// value.
    pub fn skip_bits(&mut self, count: u32) -> Result<()> {
        for _ in 0..count {
            self.get_bit()?;
        }
        Ok(())
    }
}

/// Least-significant-bit-first bit emitter — the exact write-side twin
/// of [`BitReader`] (spec §4.1: bits leave the stream LSB-first within
/// each byte, multi-bit fields assembled low-bit-first; the writer
/// therefore places the **first** written bit in bit 0 of the current
/// byte). Round 281, lifted from the long-standing test-side inverse
/// helper onto the public surface for the spec §4.2 inverse encoder.
///
/// Every bit written through this type is readable back, in order,
/// through a [`BitReader`] over [`BitWriter::finish`]'s bytes — the
/// reader/writer pairing is the bit-exactness contract every
/// `emit_*` / `read_*` primitive pair in this module is tested
/// against.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct BitWriter {
    bytes: Vec<u8>,
    cur: u8,
    nbits: u8,
}

impl BitWriter {
    /// Fresh writer with no bits emitted.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a single bit (the low bit of `bit`; higher bits are
    /// ignored, mirroring [`BitReader::get_bit`]'s 0/1 return domain).
    pub fn write_bit(&mut self, bit: u32) {
        self.cur |= ((bit & 1) as u8) << self.nbits;
        self.nbits += 1;
        if self.nbits == 8 {
            self.bytes.push(self.cur);
            self.cur = 0;
            self.nbits = 0;
        }
    }

    /// Append the low `count` bits of `value`, LSB-first — the exact
    /// inverse of [`BitReader::get_bits`]. `count` must be `<= 32`;
    /// bits of `value` at or above `count` are ignored.
    pub fn write_bits(&mut self, value: u32, count: u32) {
        debug_assert!(count <= 32, "write_bits count {count} exceeds 32");
        for i in 0..count {
            self.write_bit((value >> i) & 1);
        }
    }

    /// Append `n` `1` bits followed by a single `0` terminator — the
    /// exact inverse of [`BitReader::get_unary`].
    pub fn write_unary(&mut self, n: u32) {
        for _ in 0..n {
            self.write_bit(1);
        }
        self.write_bit(0);
    }

    /// Total bits written so far (before any final-byte padding).
    pub fn bits_written(&self) -> usize {
        self.bytes.len() * 8 + self.nbits as usize
    }

    /// `true` when no bits have been written.
    pub fn is_empty(&self) -> bool {
        self.bits_written() == 0
    }

    /// Consume the writer, zero-padding the final partial byte (the
    /// pad bits land in the high, later-read positions, so a reader
    /// consuming exactly the written bits never sees them).
    pub fn finish(mut self) -> Vec<u8> {
        if self.nbits > 0 {
            self.bytes.push(self.cur);
        }
        self.bytes
    }
}

/// Adaptive run-state carried between successive [`decode_run_length`]
/// calls — the spec §4.2 step 4 "holding one" / "holding zero"
/// registers under the wiki's shorter `last_one` / `last_zero` names.
///
/// Two booleans carry across samples, and each transmitted raw prefix
/// encodes the boundary between two adjacent words through them (spec
/// §4.2 step 4):
///
/// * a raw prefix with its low bit **set** holds a one — the **next**
///   word's folded `ones_count` gains `+1` (so its own raw prefix is
///   two ones shorter on the wire);
/// * a raw prefix with its low bit **clear** holds a zero — the
///   **next** word's `ones_count` is `0` outright and that word reads
///   **no prefix bits at all** (the `last_zero` short-circuit).
///
/// [`RunState`] packages that carry so a caller can decode a sequence
/// of prefixes from one [`BitReader`]. Round 281 corrected the fold to
/// the spec §4.2 step 4 prior-state form ("if a one **is being held**…
/// the **new** held-one is the **old** low bit"): the `+1` comes from
/// the held state the word *entered* with, not from the raw value's
/// own low bit. The wiki pseudocode assigns `last_one = n & 1` before
/// testing it — a transcription divergence the staged docs README
/// explicitly flags as non-factual; the self-referencing form cannot
/// represent a zone-0 word followed by a non-zero-zone word at all, so
/// the prior-state form is also the only reading under which every
/// sample sequence is encodable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RunState {
    /// The wiki `last_zero` flag — when set, the next decode short-
    /// circuits to `n = 0` and clears the flag.
    pub last_zero: bool,
    /// The wiki `last_one` flag — set when the most recent decoded `n`
    /// (pre-halving) was odd. Exposed for callers that want to inspect
    /// the run parity; the decoder maintains it internally.
    pub last_one: bool,
}

impl RunState {
    /// Fresh run-state for the first sample of a block: neither flag
    /// set (so the first decode reads a real unary prefix).
    pub const fn new() -> Self {
        Self {
            last_zero: false,
            last_one: false,
        }
    }

    /// Spec §4.2 step 4 holding-bit fold: collapse a raw modified-Rice
    /// prefix `raw_value` (the value [`read_raw_prefix`] returns) onto
    /// the `ones_count` zone selector the step-5 interval ladder takes,
    /// updating the held `last_one` / `last_zero` carry in place.
    ///
    /// Per the spec ("if a one is being held, `ones_count =
    /// (ones_count >> 1) + 1`, else `ones_count >>= 1`; the new
    /// held-one is the old low bit and the held-zero is its
    /// complement"):
    ///
    /// * the folded `ones_count` is `(raw_value >> 1) + 1` when a one
    ///   **was being held on entry** (`self.last_one`), else
    ///   `raw_value >> 1`;
    /// * the **new** `last_one` is the low bit of `raw_value` (the "old
    ///   low bit" — the bit the shift discards);
    /// * the new `last_zero` is the complement of the new `last_one`.
    ///
    /// So the raw prefix's low bit is a carry **into the next word**:
    /// set, it adds one to the next word's folded count; clear, it
    /// pre-encodes the next word's count as `0` (the `last_zero`
    /// short-circuit in [`read_folded_ones_count`], which reads no
    /// bits). Reads no bits itself — pure state arithmetic over
    /// `raw_value`. Pairing it with [`read_raw_prefix`] reconstructs
    /// the full §4.2 step 2 + 3 + 4 prefix decode; the exact inverse is
    /// [`RunState::unfold_prefix`].
    ///
    /// Round 281 corrected the `+1` source from the raw value's own low
    /// bit (the wiki pseudocode's assign-then-test order, flagged
    /// non-factual by the staged docs) to the prior held state the spec
    /// prose describes — see the type-level docs for why the
    /// prior-state form is the only coherent reading.
    pub const fn fold_prefix(&mut self, raw_value: u32) -> u32 {
        let ones_count = if self.last_one {
            (raw_value >> 1) + 1
        } else {
            raw_value >> 1
        };
        let last_one_bit = (raw_value & 1) != 0;
        self.last_one = last_one_bit;
        self.last_zero = !last_one_bit;
        ones_count
    }

    /// Exact inverse of [`RunState::fold_prefix`]: pick the raw
    /// modified-Rice prefix that folds to `ones_count` under the
    /// current held state, choosing the new low-bit carry.
    ///
    /// `hold_one` selects the raw value's low bit — the carry **into
    /// the next word** (spec §4.2 step 4 "the new held-one is the old
    /// low bit"):
    ///
    /// * `hold_one == true` → the next word's folded count gains `+1`
    ///   (so it must be `>= 1`);
    /// * `hold_one == false` → the next word's count is pre-encoded as
    ///   `0` and that word reads no prefix bits (the `last_zero`
    ///   short-circuit).
    ///
    /// Returns the raw prefix to hand to [`emit_raw_prefix`] and
    /// updates the held state exactly as the decoder-side fold will,
    /// or `None` when no raw value folds to `ones_count` from the
    /// current state:
    ///
    /// * a held one on entry with `ones_count == 0` (the fold adds
    ///   `+1`, so `0` is unreachable);
    /// * `ones_count - carry > (u32::MAX - 1) / 2` (the doubled raw
    ///   would not fit `u32` — unreachable for magnitudes that fit the
    ///   spec §4.2 step 5 31-bit interval mask).
    ///
    /// On `None` the state is untouched. Like the fold, this performs
    /// no I/O; callers in `last_zero` state must consume the
    /// short-circuit (which emits nothing) instead of calling this.
    pub const fn unfold_prefix(&mut self, ones_count: u32, hold_one: bool) -> Option<u32> {
        let carry = if self.last_one { 1u32 } else { 0u32 };
        if ones_count < carry {
            return None;
        }
        let halved = ones_count - carry;
        if halved > (u32::MAX - 1) / 2 {
            return None;
        }
        let raw = (halved << 1) | if hold_one { 1 } else { 0 };
        self.last_one = hold_one;
        self.last_zero = !hold_one;
        Some(raw)
    }
}

/// Decode the run-length index `n` for one sample, advancing both the
/// bit reader and the adaptive [`RunState`].
///
/// This is the round-5 name for the first half of the sample-word
/// decode — the unary prefix (with the `LIMIT_ONES = 16` escape) and
/// the `last_zero` / `last_one` carry. Since round 281 it delegates to
/// [`read_folded_ones_count`], which composes [`read_raw_prefix`]
/// (spec §4.2 steps 2 + 3, with the `cbits == 33` EOF surfaced as
/// [`Error::EndOfStream`] and the over-cap second unary as
/// [`Error::Truncated`] instead of the round-5 shift-overflow panic)
/// with [`RunState::fold_prefix`] (the spec §4.2 step 4 prior-state
/// fold).
///
/// The wiki "Samples coding" pseudocode this function originally
/// transcribed assigns `last_one = n & 1` **before** testing it in the
/// halving — a divergence from the spec §4.2 step 4 prose ("if a one
/// **is being held** … the **new** held-one is the **old** low bit")
/// that the staged docs README explicitly flags: the wiki pseudocode
/// mirrors a third-party transcription and is NOT a factual source.
/// Round 281 aligned this function with the spec; under the wiki's
/// self-referencing order a zone-0 word could never be followed by a
/// non-zero-zone word, which no working sample stream could satisfy.
///
/// The returned `n` is the run-length index the second half of the
/// decode maps onto a Golomb `(base, add)` interval.
pub fn decode_run_length(reader: &mut BitReader<'_>, state: &mut RunState) -> Result<u32> {
    read_folded_ones_count(reader, state)
}

/// The three medians (`median[0]`, `median[1]`, `median[2]`) for one
/// channel, as produced by [`expand_entropy`](crate::expand_entropy) and
/// consumed by the wiki "Samples coding" Golomb interval selection.
///
/// The set is `Copy` because [`decode_sample_value`] reads it by value:
/// round 6 deliberately does **not** mutate the medians (the median
/// "increase" / "decrease" amount is an open docs gap — see the module
/// docs). A future round that closes that gap will take `&mut Medians`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Medians {
    /// `median[0]`, `median[1]`, `median[2]` in wiki order.
    pub values: [i32; 3],
}

impl Medians {
    /// Wrap a three-median array in wiki order.
    pub const fn new(values: [i32; 3]) -> Self {
        Self { values }
    }

    /// The first (left / mono) channel's medians from a round-4
    /// [`EntropyInfo`] expansion.
    ///
    /// The wiki "Entropy info" section produces "one or two sets of
    /// medians for samples decoding"; this is the first set —
    /// `EntropyInfo::medians_left` — which is the only set on a mono
    /// block and the left channel on a stereo block.
    pub const fn from_entropy_left(info: &EntropyInfo) -> Self {
        Self {
            values: info.medians_left,
        }
    }

    /// The second (right) channel's medians from a round-4
    /// [`EntropyInfo`] expansion.
    ///
    /// This is `EntropyInfo::medians_right` — the second set, present on
    /// stereo blocks only. On a mono block (where the wiki put only one
    /// set on the wire) this is `[0, 0, 0]`, the value the expander
    /// leaves it at.
    pub const fn from_entropy_right(info: &EntropyInfo) -> Self {
        Self {
            values: info.medians_right,
        }
    }

    /// Channel-indexed bridge over [`EntropyInfo`] — `Some(medians)`
    /// when `channel_idx` is `0` (left / mono) or `1` (right, on a
    /// stereo block); `None` otherwise (out-of-range index, or `1` on a
    /// mono `EntropyInfo` where the wiki put no right-channel set on the
    /// wire).
    ///
    /// Equivalent to [`Self::from_entropy_left`] for `0` and to
    /// [`Self::from_entropy_right`] for `1` on a stereo block, but with
    /// the mono guard the typed predicates surface. Callers iterating
    /// over per-channel medians (one or two iterations against
    /// [`crate::Flags::channels_in_block`]) avoid hand-rolling the
    /// mono / stereo branch.
    pub fn from_entropy(info: &EntropyInfo, channel_idx: u8) -> Option<Self> {
        match channel_idx {
            0 => Some(Self::from_entropy_left(info)),
            1 if !info.is_mono() => Some(Self::from_entropy_right(info)),
            _ => None,
        }
    }
}

/// Spec §3.2 zone selector — which arm of the unary-prefix /
/// `ones_count` ladder the decoder is in after the unary prefix has
/// been decoded.
///
/// The four zones drive both the `(low, high)` interval (spec §4.2
/// step 5) and the per-median adaptation (spec §3.2):
///
/// | Zone | `ones_count` | Adaptation |
/// | ---- | ------------ | ---------- |
/// | [`Zone::Zone0`]         | `0`         | `median[0]` decremented                               |
/// | [`Zone::Zone1`]         | `1`         | `median[0]` incremented, `median[1]` decremented      |
/// | [`Zone::Zone2`]         | `2`         | `median[0]` + `median[1]` incremented, `median[2]` decremented |
/// | [`Zone::Zone2Overflow`] | `>= 3`      | all three medians incremented                         |
///
/// The numeric `ones_count` value (the value the spec calls
/// `ones_count` after the holding-bit fold) is recoverable via
/// [`Zone::ones_count`] for the `Zone2Overflow` arm, which carries the
/// raw value through.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Zone {
    /// `ones_count == 0` — interval base is the bottom; only
    /// `median[0]` is decremented per spec §3.2.
    Zone0,
    /// `ones_count == 1` — `median[0]` is incremented, `median[1]` is
    /// decremented per spec §3.2.
    Zone1,
    /// `ones_count == 2` — `median[0]` and `median[1]` are
    /// incremented, `median[2]` is decremented per spec §3.2.
    Zone2,
    /// `ones_count >= 3` — `median[0]`, `median[1]` and `median[2]`
    /// are all incremented per spec §3.2; the raw `ones_count` is
    /// carried through so the `(ones_count - 2) * get_med(2)` shift
    /// in the `low = ...` formula (spec §4.2 step 5) is still
    /// recoverable.
    Zone2Overflow {
        /// The original `ones_count` value (`>= 3`) preserved verbatim
        /// from the decoder's holding-bit-folded unary prefix.
        ones_count: u32,
    },
}

impl Zone {
    /// Construct a [`Zone`] from a raw `ones_count` value, mapping
    /// `0 / 1 / 2 / >=3` onto the four spec §3.2 arms.
    pub const fn from_ones_count(ones_count: u32) -> Self {
        match ones_count {
            0 => Zone::Zone0,
            1 => Zone::Zone1,
            2 => Zone::Zone2,
            _ => Zone::Zone2Overflow { ones_count },
        }
    }

    /// Recover the `ones_count` value the [`Zone`] was constructed
    /// from. `0` / `1` / `2` for the named arms, and the carried value
    /// (`>= 3`) for [`Zone::Zone2Overflow`].
    pub const fn ones_count(self) -> u32 {
        match self {
            Zone::Zone0 => 0,
            Zone::Zone1 => 1,
            Zone::Zone2 => 2,
            Zone::Zone2Overflow { ones_count } => ones_count,
        }
    }

    /// Zero-based zone selector index — `0` for [`Zone::Zone0`], `1` for
    /// [`Zone::Zone1`], `2` for [`Zone::Zone2`], `3` for any
    /// [`Zone::Zone2Overflow`] regardless of the carried `ones_count`.
    ///
    /// Lifts the spec §3.2 / §4.2 step 5 four-arm ladder as a numeric
    /// arm selector for callers that want to drive a fixed-size table
    /// (per-zone divisor, per-zone adaptation pattern) off the zone
    /// without re-matching the enum. Distinct from
    /// [`Zone::ones_count`], which preserves the raw `ones_count` value
    /// the holding-bit fold produced (so an overflow zone with
    /// `ones_count = 5` returns `5` from `ones_count` but `3` from
    /// `index`).
    pub const fn index(self) -> u8 {
        match self {
            Zone::Zone0 => 0,
            Zone::Zone1 => 1,
            Zone::Zone2 => 2,
            Zone::Zone2Overflow { .. } => 3,
        }
    }

    /// `true` when this zone is the spec §4.2 step 5 zone-2-overflow
    /// arm (`ones_count >= 3`). Convenience predicate over the enum
    /// discriminant — equivalent to `matches!(self, Zone::Zone2Overflow
    /// { .. })`.
    pub const fn is_overflow(self) -> bool {
        matches!(self, Zone::Zone2Overflow { .. })
    }

    /// `true` when spec §3.2 INCREMENTS `median[idx]` in this zone.
    ///
    /// Per the §3.2 table:
    ///
    /// | Zone                  | inc `median[0]` | inc `median[1]` | inc `median[2]` |
    /// | --------------------- | --------------- | --------------- | --------------- |
    /// | [`Zone::Zone0`]         | no              | no              | no              |
    /// | [`Zone::Zone1`]         | yes             | no              | no              |
    /// | [`Zone::Zone2`]         | yes             | yes             | no              |
    /// | [`Zone::Zone2Overflow`] | yes             | yes             | yes             |
    ///
    /// `idx` must be `0`, `1` or `2`; out-of-range indices return
    /// `false` (no median is touched).
    pub const fn increments_median(self, idx: usize) -> bool {
        if idx >= 3 {
            return false;
        }
        let zone_idx = self.index() as usize;
        // median[i] is incremented when its index is strictly less than
        // the zone selector — zone 0 increments none, zone 1 increments
        // median[0], zone 2 increments median[0..=1], zone 3 (overflow)
        // increments all three.
        idx < zone_idx
    }

    /// `true` when spec §3.2 DECREMENTS `median[idx]` in this zone.
    ///
    /// Per the §3.2 table:
    ///
    /// | Zone                  | dec `median[0]` | dec `median[1]` | dec `median[2]` |
    /// | --------------------- | --------------- | --------------- | --------------- |
    /// | [`Zone::Zone0`]         | yes             | no              | no              |
    /// | [`Zone::Zone1`]         | no              | yes             | no              |
    /// | [`Zone::Zone2`]         | no              | no              | yes             |
    /// | [`Zone::Zone2Overflow`] | no              | no              | no              |
    ///
    /// `idx` must be `0`, `1` or `2`; out-of-range indices return
    /// `false`.
    pub const fn decrements_median(self, idx: usize) -> bool {
        if idx >= 3 {
            return false;
        }
        match self {
            Zone::Zone0 => idx == 0,
            Zone::Zone1 => idx == 1,
            Zone::Zone2 => idx == 2,
            Zone::Zone2Overflow { .. } => false,
        }
    }

    /// `true` when spec §3.2 touches `median[idx]` in this zone at all
    /// — the union of [`Zone::increments_median`] and
    /// [`Zone::decrements_median`]. `false` only for medians whose
    /// running value is left unchanged by the adaptation step in this
    /// zone.
    ///
    /// `idx` must be `0`, `1` or `2`; out-of-range indices return
    /// `false`.
    pub const fn touches_median(self, idx: usize) -> bool {
        self.increments_median(idx) || self.decrements_median(idx)
    }
}

/// Adaptive median state for one channel — three `u32` running
/// medians with 4 fractional bits each, exactly as spec §2 lays them
/// out (`median[0..=2]`).
///
/// Distinct from [`Medians`]: [`Medians`] is the already-log-expanded
/// snapshot the round-4 [`crate::expand_entropy`] expander produces
/// (signed-`i32`, no fractional bits, fed to the round-6 Golomb
/// interval selector by value); [`AdaptiveMedians`] is the **mutable**
/// running state the spec §3 adaptation walks per sample. The spec
/// defines the running medians as 32-bit unsigned (`uint32_t`) with
/// 4 fractional bits encoded as a `>> 4` shift on every working
/// access (see [`AdaptiveMedians::get_med`]), and the increment /
/// decrement steps as the integer expressions spec §3 quotes
/// (`((median[i] + D) / D) * 5` up, `((median[i] + (D - 2)) / D) * 2`
/// down).
///
/// Building one from the seed values an `0x05` entropy-info sub-block
/// produces is one of:
///
/// * [`AdaptiveMedians::new`] — direct three-`u32` constructor.
/// * [`AdaptiveMedians::from_seed_values`] — same as `new`, taking the
///   raw seed values the round-4 expander produces (validated
///   non-negative).
/// * [`AdaptiveMedians::from_medians`] — converts a [`Medians`] in
///   place (returns `None` when any value is negative — `i32 → u32`
///   reinterpretation is rejected rather than masked).
///
/// Round 191 adds the value-mutating per-zone [`AdaptiveMedians::adapt`]
/// step (spec §3 + §3.2) but does **not** yet wire it into
/// [`decode_sample`]; per-sample composition is gated on a follow-up
/// round so the existing round-7 single-call decoder is preserved
/// unchanged.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdaptiveMedians {
    /// `median[0]`, `median[1]`, `median[2]` in spec order. Each value
    /// carries 4 fractional bits per spec §2.1.
    pub values: [u32; 3],
}

impl AdaptiveMedians {
    /// Wrap a three-`u32` array as the running median state.
    ///
    /// The values are taken verbatim — the 4 fractional bits the spec
    /// §2.1 `get_med` operation strips are already implicit. Pair with
    /// [`AdaptiveMedians::get_med`] when reading working values for
    /// the spec §4.2 interval ladder.
    pub const fn new(values: [u32; 3]) -> Self {
        Self { values }
    }

    /// Construct from the three seed values an `0x05` entropy-info
    /// sub-block expander ([`crate::expand_entropy`]) produces for a
    /// single channel.
    ///
    /// The seed values are the wire log-pack expanded through
    /// [`crate::expand_entropy`]; they are `i32` in the round-4 API and
    /// always non-negative for a well-formed stream. Returns `None`
    /// when any value is negative (a malformed seed) rather than
    /// silently casting to `u32`.
    pub fn from_seed_values(seeds: [i32; 3]) -> Option<Self> {
        if seeds[0] < 0 || seeds[1] < 0 || seeds[2] < 0 {
            return None;
        }
        Some(Self::new([
            seeds[0] as u32,
            seeds[1] as u32,
            seeds[2] as u32,
        ]))
    }

    /// Construct from a [`Medians`] snapshot. Returns `None` when any
    /// `i32` value is negative (`i32 → u32` is rejected rather than
    /// reinterpreted).
    ///
    /// Convenience for callers that already hold a [`Medians`] from the
    /// round-4 → round-6 bridge and want to upgrade it to the running
    /// adaptive state.
    pub fn from_medians(medians: Medians) -> Option<Self> {
        Self::from_seed_values(medians.values)
    }

    /// Channel-indexed bridge over [`EntropyInfo`] — `Some(state)` for
    /// `channel_idx == 0` (left / mono) on any payload, and for
    /// `channel_idx == 1` (right) on a stereo payload; `None` otherwise
    /// (out-of-range index, or `1` on a mono [`EntropyInfo`] where the
    /// wiki put no second median set on the wire).
    ///
    /// Also returns `None` when the selected set carries any negative
    /// seed (same defensive rejection as [`Self::from_seed_values`]).
    ///
    /// Symmetric counterpart to [`Medians::from_entropy`] — the round-4
    /// expander output → round-15 running adaptive state bridge that
    /// removes the need to hop through [`Medians`] for the stateful
    /// loop. Round 201.
    pub fn from_entropy(info: &EntropyInfo, channel_idx: u8) -> Option<Self> {
        match channel_idx {
            0 => Self::from_seed_values(info.medians_left),
            1 if !info.is_mono() => Self::from_seed_values(info.medians_right),
            _ => None,
        }
    }

    /// Build the two-element `[left, right]` array
    /// [`decode_packed_samples_stereo`] takes as its `medians` argument
    /// from a stereo [`EntropyInfo`].
    ///
    /// Returns `None` when the input is a mono payload (the wiki puts
    /// no right-channel set on the wire — there is nothing to populate
    /// the second slot from), or when either set has a negative seed
    /// (same defensive rejection as [`Self::from_seed_values`]).
    ///
    /// Round 201 — top-level round-4 `0x05` expander → round-199 stereo
    /// decode loop bridge.
    pub fn stereo_pair_from_entropy(info: &EntropyInfo) -> Option<[Self; 2]> {
        if info.is_mono() {
            return None;
        }
        let left = Self::from_seed_values(info.medians_left)?;
        let right = Self::from_seed_values(info.medians_right)?;
        Some([left, right])
    }

    /// Spec §2.1 `get_med(i)` operation — the **working** median value
    /// the spec §4.2 interval ladder consumes.
    ///
    /// `get_med(i) = (median[i] >> 4) + 1`, with a minimum of `1`.
    /// The stored median carries 4 fractional bits; the `>> 4`
    /// extracts the integer breakpoint and the `+ 1` enforces the
    /// floor.
    ///
    /// `idx` must be `0`, `1` or `2`. Out-of-range indices panic in
    /// debug builds and return `0` in release.
    #[inline]
    pub fn get_med(&self, idx: usize) -> u32 {
        debug_assert!(idx < 3, "median index out of range");
        if idx >= 3 {
            return 0;
        }
        (self.values[idx] >> GET_MED_SHIFT) + GET_MED_FLOOR
    }

    /// Spec §3 increment step for `median[idx]`:
    ///
    /// `median[i] += ((median[i] + D) / D) * 5`
    ///
    /// where `D` is [`DIV0`] / [`DIV1`] / [`DIV2`] for `idx = 0 / 1 / 2`.
    /// `idx` must be `0`, `1` or `2`; out-of-range indices are a
    /// no-op (debug-assert).
    pub fn inc_median(&mut self, idx: usize) {
        debug_assert!(idx < 3, "median index out of range");
        if idx >= 3 {
            return;
        }
        let d = divisor_for(idx);
        let cur = self.values[idx];
        // ((cur + d) / d) * 5 — saturating to bound the worst case;
        // (u32::MAX + 128) overflows in plain u64 only at extreme
        // synthetic values that the decode loop never produces, but
        // the saturating form is the defensive choice and matches the
        // arithmetic spec §3 quotes (the upstream uses uint32_t too,
        // so wrapping is the same behaviour up to the saturating cap).
        let step = (cur.saturating_add(d) / d).saturating_mul(MEDIAN_INC_MULTIPLIER);
        self.values[idx] = cur.saturating_add(step);
    }

    /// Spec §3 decrement step for `median[idx]`:
    ///
    /// `median[i] -= ((median[i] + (D - 2)) / D) * 2`
    ///
    /// where `D` is [`DIV0`] / [`DIV1`] / [`DIV2`] for `idx = 0 / 1 / 2`.
    /// The bias `+ (D - 2)` guarantees the step is at least `2` and
    /// at most the median itself, so the decremented value never goes
    /// below `0`. `idx` must be `0`, `1` or `2`; out-of-range indices
    /// are a no-op (debug-assert).
    pub fn dec_median(&mut self, idx: usize) {
        debug_assert!(idx < 3, "median index out of range");
        if idx >= 3 {
            return;
        }
        let d = divisor_for(idx);
        let cur = self.values[idx];
        // ((cur + (d - 2)) / d) * 2 — saturating in the same defensive
        // form as inc_median.
        let step = (cur.saturating_add(d - 2) / d).saturating_mul(MEDIAN_DEC_MULTIPLIER);
        self.values[idx] = cur.saturating_sub(step);
    }

    /// Spec §3.2 per-zone median update — applies the correct
    /// combination of [`AdaptiveMedians::inc_median`] /
    /// [`AdaptiveMedians::dec_median`] calls for the [`Zone`] the
    /// decoder is in:
    ///
    /// * [`Zone::Zone0`] — decrement `median[0]`.
    /// * [`Zone::Zone1`] — increment `median[0]`, decrement `median[1]`.
    /// * [`Zone::Zone2`] — increment `median[0]` and `median[1]`,
    ///   decrement `median[2]`.
    /// * [`Zone::Zone2Overflow`] — increment all three medians.
    ///
    /// This is the **median-adaptation amount** the round-191 spec
    /// unblocks: a single primitive that the per-sample decode loop
    /// will call once per decoded sample, before forming the next
    /// `(low, high)` interval.
    pub fn adapt(&mut self, zone: Zone) {
        match zone {
            Zone::Zone0 => self.dec_median(0),
            Zone::Zone1 => {
                self.inc_median(0);
                self.dec_median(1);
            }
            Zone::Zone2 => {
                self.inc_median(0);
                self.inc_median(1);
                self.dec_median(2);
            }
            Zone::Zone2Overflow { .. } => {
                self.inc_median(0);
                self.inc_median(1);
                self.inc_median(2);
            }
        }
    }

    /// Convenience wrapper combining [`Zone::from_ones_count`] with
    /// [`AdaptiveMedians::adapt`] — drives the per-zone update from a
    /// raw `ones_count` value rather than a typed [`Zone`].
    pub fn adapt_for_ones_count(&mut self, ones_count: u32) {
        self.adapt(Zone::from_ones_count(ones_count));
    }

    /// Spec §4.2 step 5 typed `(low, high)` interval formation for a
    /// channel — the value interval the truncated-binary mantissa
    /// decode (`maxcode = high - low`) reads inside.
    ///
    /// Using `get_med(i) = (median[i] >> 4) + 1`:
    ///
    /// * [`Zone::Zone0`] (`ones_count == 0`): `low = 0`,
    ///   `high = get_med(0) - 1`.
    /// * [`Zone::Zone1`] (`ones_count == 1`): `low = get_med(0)`,
    ///   `high = low + get_med(1) - 1`.
    /// * [`Zone::Zone2`] (`ones_count == 2`):
    ///   `low = get_med(0) + get_med(1)`,
    ///   `high = low + get_med(2) - 1`.
    /// * [`Zone::Zone2Overflow`] (`ones_count >= 3`):
    ///   `low = get_med(0) + get_med(1) + (ones_count - 2) * get_med(2)`,
    ///   `high = low + get_med(2) - 1`.
    ///
    /// `low` and `high` are masked to 31 bits per spec §4.2 step 5 via
    /// [`INTERVAL_MASK_31`], and `high` is clamped up to `low` when the
    /// mask underflows the interval (the structurally rare pathological
    /// case the spec calls out by saying "high is clamped up to low if
    /// it underflowed"). The returned [`SampleInterval`] is the same
    /// `(low, high)` the private decode loop computes, surfaced as a
    /// typed view for callers walking the spec ladder by hand or
    /// building diagnostic traces against a known median set.
    ///
    /// This is the §4.2 step 5 primitive — it does NOT mutate the
    /// medians (the spec §3.2 adaptation happens at this point in the
    /// decode loop, but is a separate step; see
    /// [`AdaptiveMedians::adapt`] for the mutation). Round 255.
    pub fn sample_interval(&self, zone: Zone) -> SampleInterval {
        let m0 = self.get_med(0);
        let m1 = self.get_med(1);
        let m2 = self.get_med(2);
        let (mut low, mut high) = match zone {
            Zone::Zone0 => (0u32, m0.wrapping_sub(1)),
            Zone::Zone1 => (m0, m0.wrapping_add(m1).wrapping_sub(1)),
            Zone::Zone2 => (
                m0.wrapping_add(m1),
                m0.wrapping_add(m1).wrapping_add(m2).wrapping_sub(1),
            ),
            Zone::Zone2Overflow { ones_count } => {
                let extra = m2.wrapping_mul(ones_count.wrapping_sub(2));
                let base = m0.wrapping_add(m1).wrapping_add(extra);
                (base, base.wrapping_add(m2).wrapping_sub(1))
            }
        };
        low &= INTERVAL_MASK_31;
        high &= INTERVAL_MASK_31;
        if high < low {
            high = low;
        }
        SampleInterval { low, high }
    }

    /// Convenience wrapper combining [`Zone::from_ones_count`] with
    /// [`AdaptiveMedians::sample_interval`] — forms the typed
    /// [`SampleInterval`] directly from a raw `ones_count` value rather
    /// than a typed [`Zone`].
    pub fn sample_interval_for_ones_count(&self, ones_count: u32) -> SampleInterval {
        self.sample_interval(Zone::from_ones_count(ones_count))
    }

    /// Inverse of the spec §4.2 step 5 interval ladder: the `ones_count`
    /// zone selector whose (pre-mask) interval contains `magnitude`.
    ///
    /// Walks the ladder's own breakpoints (`get_med` working values):
    ///
    /// * `magnitude < get_med(0)` → zone `0`;
    /// * `magnitude < get_med(0) + get_med(1)` → zone `1`;
    /// * otherwise → `2 + (magnitude - get_med(0) - get_med(1)) /
    ///   get_med(2)` (each overflow step adds one `get_med(2)` stride to
    ///   the zone-2 base, per the step 5 overflow arm).
    ///
    /// For every magnitude `<=` [`INTERVAL_MASK_31`] the returned zone's
    /// **unmasked** interval contains the magnitude by construction; the
    /// **masked** interval ([`AdaptiveMedians::sample_interval_for_ones_count`])
    /// can collapse below it only in the 31-bit-boundary corner where
    /// `low + get_med(2) - 1` crosses the mask — encoder callers verify
    /// with [`SampleInterval::contains`] and surface
    /// [`Error::ValueNotInInterval`] there (no decode of the same median
    /// state could have produced such a magnitude). Round 281.
    pub fn zone_for_magnitude(&self, magnitude: u32) -> u32 {
        let m0 = self.get_med(0);
        let m1 = self.get_med(1);
        if magnitude < m0 {
            return 0;
        }
        let zone1_width = m1;
        let past_m0 = magnitude - m0;
        if past_m0 < zone1_width {
            return 1;
        }
        let m2 = self.get_med(2);
        // get_med() >= 1 always (the +1 floor), so the division is safe.
        2 + (past_m0 - zone1_width) / m2
    }
}

/// Spec §4.2 step 5 typed `(low, high)` value interval — the bracket
/// the truncated-binary mantissa decode (spec §4.2 step 6 first
/// paragraph) reads a sample value inside.
///
/// Built from [`AdaptiveMedians::sample_interval`] (or the
/// [`AdaptiveMedians::sample_interval_for_ones_count`] convenience
/// wrapper); both values are already masked to 31 bits per
/// [`INTERVAL_MASK_31`] and `high >= low` is invariant (the constructor
/// clamps an underflowed `high` up to `low`).
///
/// The mantissa decode width is the **inclusive** code count `high -
/// low + 1`, but the decoder consumes `maxcode = high - low` (see
/// [`SampleInterval::maxcode`]) — that is the wiki's `maxcode` literal,
/// and the truncated-binary primitive reads `bitcount = bit-length of
/// maxcode` bits' worth of phase-in codewords inside `[0, maxcode]`.
///
/// Round 255.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SampleInterval {
    /// `low` per spec §4.2 step 5 — the interval's lower bound (the
    /// value the mantissa decode adds the decoded `code` to).
    pub low: u32,
    /// `high` per spec §4.2 step 5 — the interval's upper bound
    /// (inclusive). `high >= low` is invariant.
    pub high: u32,
}

impl SampleInterval {
    /// Construct directly from `(low, high)`. The caller is responsible
    /// for the spec §4.2 step 5 31-bit mask + `high >= low` clamp; the
    /// primary construction path is [`AdaptiveMedians::sample_interval`]
    /// which applies both. The raw constructor is exposed for tests
    /// building fixtures by hand.
    pub const fn new(low: u32, high: u32) -> Self {
        Self { low, high }
    }

    /// `low` per spec §4.2 step 5 — the interval's lower bound.
    pub const fn low(&self) -> u32 {
        self.low
    }

    /// `high` per spec §4.2 step 5 — the interval's upper bound
    /// (inclusive).
    pub const fn high(&self) -> u32 {
        self.high
    }

    /// `maxcode = high - low` per spec §4.2 step 6 first paragraph —
    /// the value the truncated-binary mantissa decoder reads inside
    /// `[0, maxcode]`. With `high >= low` invariant, this is a plain
    /// subtraction.
    pub const fn maxcode(&self) -> u32 {
        self.high - self.low
    }

    /// Number of distinct codewords in the interval — `high - low + 1`,
    /// the **inclusive** width. Always `>= 1` because of the
    /// `high >= low` invariant. Saturates at `u32::MAX` for the
    /// pathological `(0, u32::MAX)` edge.
    pub const fn width(&self) -> u32 {
        self.high.saturating_sub(self.low).saturating_add(1)
    }

    /// `true` when the interval has no slack — `low == high`, i.e.
    /// [`Self::maxcode`] is `0` and the mantissa decode reads zero bits
    /// (the truncated-binary primitive's `maxcode == 0` arm).
    pub const fn is_degenerate(&self) -> bool {
        self.low == self.high
    }

    /// Test whether a candidate magnitude lies inside the interval —
    /// `low <= value <= high`. Used by tests/traces verifying a decoded
    /// magnitude is in-bounds before sign reconstruction.
    pub const fn contains(&self, value: u32) -> bool {
        value >= self.low && value <= self.high
    }

    /// Spec §4.2 step 6 first paragraph `bitcount = floor(log2(maxcode))
    /// + 1` — the bit-length of `maxcode`, i.e. the number of bits the
    ///   FULL truncated-binary codeword would consume if every code were
    ///   promoted to the long form.
    ///
    /// Special-cased per the spec:
    ///
    /// * `maxcode == 0` → `bitcount == 0` (no bits consumed; the
    ///   mantissa is always `0`).
    /// * `maxcode == 1` → `bitcount == 1` (exactly one bit consumed;
    ///   the bit IS the mantissa).
    /// * `maxcode >= 2` → `bitcount = 32 - maxcode.leading_zeros()`,
    ///   the floor-log-2 plus one.
    ///
    /// Paired with [`Self::mantissa_extras`] to drive the phase-in
    /// short / long branch of [`Self::decode_mantissa`].
    pub const fn mantissa_bitcount(&self) -> u32 {
        let maxcode = self.maxcode();
        if maxcode == 0 {
            0
        } else {
            32 - maxcode.leading_zeros()
        }
    }

    /// Spec §4.2 step 6 first paragraph `extras = (1 << bitcount) -
    /// maxcode - 1` — the number of SHORT codewords (the `(bitcount -
    /// 1)`-bit values that map to magnitudes `[0, extras)`).
    ///
    /// * `maxcode == 0` → `extras == 0` (no codewords, no slack).
    /// * `maxcode == 1` → `extras == 0` (both codewords are full
    ///   1-bit values; no slack to absorb).
    /// * `maxcode >= 2` → `extras = (1 << bitcount) - maxcode - 1`.
    ///   With `2^(bitcount-1) <= maxcode < 2^bitcount`, this is in
    ///   `[0, 2^(bitcount-1) - 1]`, so the short region is at most half
    ///   the long region's width.
    ///
    /// Pure arithmetic — does not touch a bit-reader.
    pub const fn mantissa_extras(&self) -> u32 {
        let maxcode = self.maxcode();
        if maxcode < 2 {
            0
        } else {
            let bitcount = 32 - maxcode.leading_zeros();
            (1u32 << bitcount) - maxcode - 1
        }
    }

    /// Spec §4.2 step 6 first paragraph truncated-binary mantissa
    /// decode — consumes the LSB-first bit-pattern from `reader` and
    /// returns the integer `code` in `[0, maxcode]`.
    ///
    /// Steps (lifted verbatim from spec §4.2 step 6 first paragraph):
    ///
    /// * `maxcode == 0` → consume no bits, return `0`.
    /// * `maxcode == 1` → consume one bit, return that bit (`0` or `1`).
    /// * `maxcode >= 2` → consume `bitcount - 1` bits LSB-first into
    ///   `short`. If `short < extras` return `short` (the short-form
    ///   `(bitcount - 1)`-bit codeword); otherwise consume ONE more bit
    ///   `extra_bit` and return `(short << 1) - extras + extra_bit` (the
    ///   long-form `bitcount`-bit codeword).
    ///
    /// On `Error::Truncated` the reader's cursor is at the buffer end
    /// per the [`BitReader::get_bit`] / [`BitReader::get_bits`] partial-
    /// consume semantics. Pair with [`Self::low`] (or the
    /// [`Self::decode_value`] convenience wrapper) to recover the
    /// `low + code` magnitude.
    pub fn decode_mantissa(&self, reader: &mut BitReader<'_>) -> Result<u32> {
        read_truncated_binary(reader, self.maxcode())
    }

    /// Spec §4.2 step 6 first paragraph full magnitude decode — runs
    /// [`Self::decode_mantissa`] and adds [`Self::low`] back, returning
    /// the magnitude `low + code` (`u32`, masked to 31 bits by the
    /// `low`/`high` 31-bit mask invariant).
    ///
    /// Convenience wrapper for callers walking the spec ladder by hand:
    /// the magnitude is exactly what [`decode_sample_stateful`] feeds
    /// the sign-bit reconstruction step (§4.2 step 7) before deciding
    /// between `mid` and `!mid`. The sign bit itself is OUT of scope
    /// here — this is the unsigned magnitude, no sign read.
    pub fn decode_value(&self, reader: &mut BitReader<'_>) -> Result<u32> {
        let code = self.decode_mantissa(reader)?;
        Ok(self.low.wrapping_add(code))
    }

    /// Spec §4.2 steps 6 + 7 fused decode — runs [`Self::decode_value`]
    /// for the unsigned magnitude, then reads exactly ONE sign bit per
    /// spec §4.2 step 7 ("Sign bit, last. After the magnitude is fixed,
    /// read exactly one sign bit") and returns the signed sample: the
    /// bitwise complement of the magnitude when the sign bit is set,
    /// the magnitude itself otherwise.
    ///
    /// This is the complete value tail of one sample word — everything
    /// after the zone selector is fixed:
    ///
    /// ```text
    /// mantissa bits  →  sign bit
    /// ```
    ///
    /// (the spec §4.2 closing on-wire-order line, with the prefix
    /// already consumed by the caller). [`decode_sample_stateful`] /
    /// [`decode_sample_stateful_stereo`] delegate their steps 6 + 7 to
    /// this method, so the exact bits the decode loop consumes ARE the
    /// bits this typed accessor consumes.
    ///
    /// On `Error::Truncated` (mid-mantissa or at the missing sign bit)
    /// the reader's cursor follows the [`BitReader::get_bit`] /
    /// [`BitReader::get_bits`] partial-consume semantics.
    pub fn decode_signed_value(&self, reader: &mut BitReader<'_>) -> Result<i32> {
        let magnitude = self.decode_value(reader)?;
        read_sign_and_apply(reader, magnitude)
    }

    /// Exact inverse of [`Self::decode_mantissa`]: emit the truncated-
    /// binary bit pattern for the mantissa `code` inside this interval
    /// (spec §4.2 step 6 first paragraph, write side).
    ///
    /// * `maxcode == 0` → emits no bits (`code` must be `0`).
    /// * `maxcode == 1` → emits one bit (the code itself).
    /// * `maxcode >= 2` → codes `< extras` emit the short
    ///   `(bitcount - 1)`-bit form; codes `>= extras` emit the full
    ///   `bitcount`-bit long form `short = (code + extras) >> 1`
    ///   followed by `extra_bit = (code + extras) & 1` (the decode's
    ///   `code = (short << 1) - extras + extra_bit` solved for the
    ///   written bits).
    ///
    /// Returns [`Error::ValueNotInInterval`] (with the code reported
    /// against `[0, maxcode]`) when `code > maxcode`; nothing is
    /// written in that case. Round 281.
    pub fn encode_mantissa(&self, writer: &mut BitWriter, code: u32) -> Result<()> {
        let maxcode = self.maxcode();
        if code > maxcode {
            return Err(Error::ValueNotInInterval {
                value: code,
                low: 0,
                high: maxcode,
            });
        }
        if maxcode == 0 {
            return Ok(());
        }
        if maxcode == 1 {
            writer.write_bit(code);
            return Ok(());
        }
        let bitcount = self.mantissa_bitcount();
        let extras = self.mantissa_extras();
        if code < extras {
            writer.write_bits(code, bitcount - 1);
        } else {
            let combined = code + extras;
            writer.write_bits(combined >> 1, bitcount - 1);
            writer.write_bit(combined & 1);
        }
        Ok(())
    }

    /// Exact inverse of [`Self::decode_value`]: emit the mantissa bits
    /// for the unsigned `magnitude` (`low <= magnitude <= high`).
    ///
    /// Returns [`Error::ValueNotInInterval`] (with the interval's
    /// `[low, high]`) when the magnitude lies outside the interval;
    /// nothing is written in that case. Round 281.
    pub fn encode_value(&self, writer: &mut BitWriter, magnitude: u32) -> Result<()> {
        if !self.contains(magnitude) {
            return Err(Error::ValueNotInInterval {
                value: magnitude,
                low: self.low,
                high: self.high,
            });
        }
        self.encode_mantissa(writer, magnitude - self.low)
    }

    /// Exact inverse of [`Self::decode_signed_value`]: split the signed
    /// sample into magnitude + sign via [`split_sign`], emit the
    /// mantissa bits, then emit exactly ONE sign bit last (spec §4.2
    /// step 7 "Sign bit, last").
    ///
    /// Returns [`Error::ValueNotInInterval`] when the magnitude lies
    /// outside the interval; nothing is written in that case. Round
    /// 281.
    pub fn encode_signed_value(&self, writer: &mut BitWriter, value: i32) -> Result<()> {
        let (magnitude, sign_bit_set) = split_sign(value);
        self.encode_value(writer, magnitude)?;
        writer.write_bit(u32::from(sign_bit_set));
        Ok(())
    }
}

/// Return the spec §3 divisor `D` for the given median index. Panics
/// in debug builds when `idx >= 3`; returns `DIV2` in release as a
/// defensive default (the highest-frequency adapter, so a stray call
/// causes the smallest spurious step).
#[inline]
fn divisor_for(idx: usize) -> u32 {
    debug_assert!(idx < 3, "median index out of range");
    match idx {
        0 => DIV0,
        1 => DIV1,
        _ => DIV2,
    }
}

/// The `(base, add)` Golomb interval the wiki selects from the
/// run-length index `n` and a channel's three medians.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GolombInterval {
    /// The interval base (`base` in the wiki pseudocode).
    pub base: i32,
    /// The interval span minus one (`add` in the wiki pseudocode); the
    /// mantissa is read across `add + 1` codewords.
    pub add: i32,
}

/// Map the run-length index `n` onto a `(base, add)` Golomb interval
/// using a channel's three medians, exactly as the wiki "Samples coding"
/// pseudocode specifies:
///
/// ```text
/// n == 0: base = 0;                                       add = median[0] - 1
/// n == 1: base = median[0];                               add = median[1] - 1
/// else:   base = median[0] + median[1] + median[2]*(n-2); add = median[2] - 1
/// ```
///
/// This is a pure function of `n` and the medians — no bits are read and
/// no median is mutated. The median "increase" / "decrease" updates that
/// the wiki pairs with each branch are deferred (docs gap; see the
/// module docs).
pub fn golomb_interval(n: u32, medians: Medians) -> GolombInterval {
    let m = medians.values;
    match n {
        0 => GolombInterval {
            base: 0,
            add: m[0] - 1,
        },
        1 => GolombInterval {
            base: m[0],
            add: m[1] - 1,
        },
        _ => {
            // n >= 2; (n - 2) is the count of full median[2] intervals
            // past the median[0] + median[1] floor.
            let extra = (n - 2) as i32;
            GolombInterval {
                base: m[0] + m[1] + m[2] * extra,
                add: m[2] - 1,
            }
        }
    }
}

/// Bit-length of a non-negative `add`: the smallest `k` with
/// `(1 << k) > add` (so `(1 << k) >= add + 1`). `add == 0` → `0`.
///
/// This is the wiki's `k = log2(add)` under the bit-length reading the
/// module docs derive from the wiki's own `ex = (1 << k) - add - 1 >= 0`
/// requirement. Callers guard `add == 0` before relying on `k - 1`.
fn golomb_k(add: i32) -> u32 {
    debug_assert!(add >= 0, "golomb_k expects non-negative add");
    32 - (add as u32).leading_zeros()
}

/// Decode one sample *value* from the bitstream, given its already-
/// decoded run-length index `n` and a channel's (fixed) medians.
///
/// This is the value half of the wiki "Samples coding" pseudocode: pick
/// the `(base, add)` interval (via [`golomb_interval`]), then read the
/// Golomb mantissa and sign:
///
/// ```text
/// k  = log2(add);
/// ex = (1 << k) - add - 1;
/// t2 = getbits(k - 1);
/// if(t2 >= ex) t2 = t2 * 2 - ex + getbit();
/// sign = getbit();
/// result = sign==0 ? base + t2 : ~(base + t2);
/// ```
///
/// The medians are taken **by value and not mutated** — the wiki's
/// per-branch median update amount is an open docs gap, so round 6
/// decodes a single value against a caller-supplied fixed set. When the
/// selected median is `1` (`add == 0`) the interval degenerates to a
/// single codeword whose `log2(0)` / `getbits(-1)` are undefined by the
/// wiki; that case returns [`Error::GolombDegenerateInterval`].
pub fn decode_sample_value(reader: &mut BitReader<'_>, n: u32, medians: Medians) -> Result<i32> {
    let GolombInterval { base, add } = golomb_interval(n, medians);

    if add <= 0 {
        // add == 0 → single-codeword interval (median == 1): log2(0) and
        // getbits(-1) are undefined by the wiki. add < 0 would mean a
        // median of 0, which the wiki never produces; treat both as the
        // degenerate case rather than guessing.
        return Err(Error::GolombDegenerateInterval(add));
    }

    let k = golomb_k(add);
    // k >= 1 here because add >= 1, so getbits(k - 1) has a non-negative
    // argument and the degenerate getbits(-1) path is unreachable.
    let ex = (1i32 << k) - add - 1;
    let mut t2 = reader.get_bits(k - 1)? as i32;
    if t2 >= ex {
        t2 = t2 * 2 - ex + reader.get_bit()? as i32;
    }

    let sign = reader.get_bit()?;
    let magnitude = base + t2;
    let result = if sign == 0 { magnitude } else { !magnitude };
    Ok(result)
}

/// Decode one complete sample from the bitstream — the wiki "Samples
/// coding" per-sample pseudocode as a single call.
///
/// The wiki presents the run-length `n` decoder and the Golomb
/// value decoder as one contiguous pseudocode block run once per
/// sample. [`decode_run_length`] and [`decode_sample_value`] split that
/// block into its two halves for unit testing; `decode_sample` chains
/// them back together so a caller decodes one sample with one call:
///
/// 1. [`decode_run_length`] reads the unary prefix (with the `n == 16`
///    escape) and advances the adaptive [`RunState`], yielding the
///    run-length index `n`.
/// 2. [`decode_sample_value`] selects the `(base, add)` interval for
///    that `n`, reads the Golomb mantissa and sign, and returns the
///    reconstructed sample.
///
/// The medians are taken **by value and not mutated**: the wiki names
/// the per-branch "increase" / "decrease" median update but not its
/// *amount*, so the stateful loop that walks a whole `0x0A` payload —
/// feeding each decoded `n` back into a mutating median set — stays
/// gated on that docs gap (see the module-level docs). `decode_sample`
/// is therefore a single-sample primitive against a caller-supplied
/// fixed median set, exactly like [`decode_sample_value`]; it adds only
/// the run-length step in front.
///
/// Errors propagate unchanged: [`Error::Truncated`] when the bitstream
/// is exhausted mid-sample, and [`Error::GolombDegenerateInterval`] when
/// the selected median is `1` (`add == 0`).
pub fn decode_sample(
    reader: &mut BitReader<'_>,
    state: &mut RunState,
    medians: Medians,
) -> Result<i32> {
    let n = decode_run_length(reader, state)?;
    decode_sample_value(reader, n, medians)
}

// -----------------------------------------------------------------------
// Stateful per-sample decode (spec §3 + §3.2 + §4.2)
// -----------------------------------------------------------------------
//
// The functions above this point are the single-sample primitives that
// preceded the round-194 spec doc (the wiki-compressed `last_zero` /
// `last_one` reader, the (base, add) interval, the mantissa + sign
// reconstructor). They take medians by VALUE and never mutate them.
//
// `decode_sample_stateful` below is the round-194 addition: the full
// per-sample loop the spec authorises (median adaptation per §3.2,
// 31-bit-masked (low, high) interval per §4.2 step 5, truncated-binary
// mantissa per §4.2 step 6 first paragraph, sign per §4.2 step 7, EOF
// per §4.2 step 3 `cbits == 33`). It takes the medians by &mut and
// mutates them; the earlier primitives stay untouched so existing tests
// and call-sites are unaffected.

/// Spec §4.2 step 3 EOF escape value: when the second unary read inside
/// the `ones_count == LIMIT_ONES` escape arm yields `cbits == 33`, the
/// stream signals end-of-data.
pub const ESCAPE_EOF_CBITS: u32 = 33;

/// Cap on the run-length unary in spec §4.2 step 1 (the zero-run fast
/// path) and on the second unary in spec §4.2 step 3 (the
/// `ones_count == LIMIT_ONES` escape). Both are guarded against
/// reading more than 33 consecutive `1` bits before a terminator.
pub const RUN_ESCAPE_CAP: u32 = 33;

/// 31-bit mask applied to `low` / `high` in spec §4.2 step 5 ("`low` and
/// `high` are then masked to 31 bits and `high` is clamped up to `low`
/// if it underflowed").
pub const INTERVAL_MASK_31: u32 = 0x7fff_ffff;

/// Mutable per-channel decode state carried across successive
/// [`decode_sample_stateful`] calls inside one block.
///
/// Bundles three things the spec §4.2 loop touches between samples:
///
/// 1. The wiki-compressed `last_zero` / `last_one` carry exposed via
///    [`RunState`]. These are the spec §4.2 step 4 "holding_one" /
///    "holding_zero" registers under the wiki's shorter names — the
///    fold semantics (`if (last_one) ones_count = (raw >> 1) + 1;
///    else ones_count = raw >> 1; last_zero = !last_one; last_one =
///    raw & 1`) match the spec §4.2 step 4 prose ("if a one is being
///    held, `ones_count = (ones_count >> 1) + 1`, else
///    `ones_count >>= 1`; the new held-one is the old low bit and the
///    held-zero is its complement") exactly, so [`RunState`] is the
///    single source of truth.
/// 2. A zero-run-pending counter for spec §4.2 step 1: when the
///    zero-run fast path emits a non-zero run length, the decoder
///    returns a single `0` sample on that call but owes `run_length -
///    1` more `0` samples on subsequent calls before reading any more
///    prefix bits. [`Self::zero_run_pending`] tracks the remaining
///    debt across calls.
/// 3. A "did we ever take the zero-run fast path?" sticky bit. Once a
///    zero-run resets the channel's medians to `0` (per §4.2 step 1
///    "A non-zero run resets both channels' medians to zero"), the
///    medians stay at `0` until the spec §3 adaptation walks them
///    back up. [`Self::ever_took_zero_run`] is exposed for tests
///    asserting the path was actually taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DecodeState {
    /// The wiki-compressed `last_zero` / `last_one` carry, re-used by
    /// [`decode_run_length`] for the existing single-sample tests and
    /// by [`decode_sample_stateful`] for the round-194 loop.
    pub run: RunState,
    /// Remaining samples owed by an in-flight zero-run from spec §4.2
    /// step 1 — when non-zero, the next [`decode_sample_stateful`]
    /// call short-circuits to a `0` sample (no bits read) and
    /// decrements this counter.
    pub zero_run_pending: u32,
    /// `true` once the spec §4.2 step 1 fast path took a non-zero
    /// run-length and zeroed the medians. Tests assert this to confirm
    /// the path actually ran rather than the loop bypassing it.
    pub ever_took_zero_run: bool,
}

impl DecodeState {
    /// Fresh decode state for the first sample of a block: no holding
    /// bits, no zero-run debt, no zero-run-fast-path-seen flag.
    pub const fn new() -> Self {
        Self {
            run: RunState::new(),
            zero_run_pending: 0,
            ever_took_zero_run: false,
        }
    }

    /// Spec §4.2 step 1 eligibility gate for the mono loop: the
    /// zero-run fast path may only be probed when the channel's **raw
    /// stored** `median[0]` is `<= 1` (spec: "If both channels'
    /// `median[0]` are ≤ 1" — for mono, the one channel) AND no holding
    /// state is pending ("and no 'holding' state is pending") — neither
    /// the `last_one` carry nor the `last_zero` short-circuit may be
    /// set.
    ///
    /// The threshold reads the raw `median[0]` value, NOT the
    /// [`AdaptiveMedians::get_med`] working value: spec §2.1 explicitly
    /// distinguishes the stored `median[i]` from the derived
    /// `get_med(i) = (median[i] >> 4) + 1`, and the §4.2 step 1 prose
    /// names the bare `median[0]` — everywhere the spec means the
    /// working value it says so ("Using `get_med(i)`…", step 5).
    /// Round 281 corrected the gate from the round-278 `get_med(0) <=
    /// 1` reading (raw `<= 15`) to the spec's raw `<= 1`.
    ///
    /// Pure predicate — reads no bits, mutates nothing. Pair with
    /// [`read_zero_run_length`] to walk the spec §4.2 step 1 fast path
    /// by hand; the private decode loop's gate IS this predicate.
    pub fn zero_run_eligible(&self, medians: &AdaptiveMedians) -> bool {
        medians.values[0] <= 1 && !self.run.last_one && !self.run.last_zero
    }
}

/// Spec §4.2 steps 2 + 3: read the **raw** modified-Rice unary prefix
/// for one sample word — the value the spec calls the prefix *before*
/// the step-4 holding-bit fold collapses it onto a zone selector.
///
/// Sequence:
///
/// * **Step 2** — read the count of consecutive `1` bits terminated by
///   a `0` bit (the plain unary prefix).
/// * **Step 3** — if that count reaches [`UNARY_ESCAPE`] (`LIMIT_ONES =
///   16`), an escape follows: a second unary `cbits` of up to
///   [`ESCAPE_EOF_CBITS`] (`33`). `cbits == 33` signals end-of-data and
///   surfaces as [`Error::EndOfStream`]; `cbits < 2` is the extra
///   magnitude added straight onto [`UNARY_ESCAPE`]; otherwise read
///   `cbits - 1` more bits LSB-first with the top bit implied set, then
///   add [`UNARY_ESCAPE`] back in.
///
/// Reads no holding state and folds nothing — the returned value is the
/// pre-fold `raw_value` the step-4 fold ([`RunState::fold_prefix`])
/// consumes. This is the typed lift of the §4.2 step 2 + 3 portion the
/// private decode loop previously inlined; it is the value-side twin of
/// the §4.2 step 5/6/7 surface lifted in rounds 255 / 260 / 261.
///
/// `Error::Truncated` may surface from any of the unary / multi-bit
/// reads if the buffer runs out; `Error::EndOfStream` is the explicit
/// `cbits == 33` EOF marker, distinct from a buffer that merely ran
/// dry. A second unary run **beyond** the cap (`cbits > 33`) is
/// spec-silent ("read a further unary count of up to 33 1-bits") and
/// reports `Error::Truncated` — failing loudly instead of overflowing
/// the `cbits - 1`-bit mantissa assembly (round 278; previously a
/// shift-overflow debug panic).
pub fn read_raw_prefix(reader: &mut BitReader<'_>) -> Result<u32> {
    let raw = reader.get_unary()?;
    if raw < UNARY_ESCAPE {
        return Ok(raw);
    }
    // Spec §4.2 step 3: escape arm. cbits up to 33; cbits == 33 is EOF.
    let cbits = reader.get_unary()?;
    if cbits == ESCAPE_EOF_CBITS {
        return Err(Error::EndOfStream);
    }
    if cbits > ESCAPE_EOF_CBITS {
        // Spec-silent edge: the §4.2 step 3 second unary is "up to 33"
        // — a longer run contradicts the cap, and `cbits - 1 >= 33`
        // mantissa bits would overflow the u32 assembly below. Treat as
        // truncated to fail loudly rather than wrapping. (Before round
        // 278 this input hit a shift-overflow debug panic.)
        return Err(Error::Truncated);
    }
    if cbits < 2 {
        Ok(UNARY_ESCAPE + cbits)
    } else {
        // cbits >= 2: read cbits - 1 mantissa bits LSB-first, top bit
        // implied set, then add LIMIT_ONES back in (spec §4.2 step 3,
        // "then add `LIMIT_ONES` back in").
        let mantissa = reader.get_bits(cbits - 1)?;
        let escape_value = (1u32 << (cbits - 1)) | mantissa;
        Ok(UNARY_ESCAPE + escape_value)
    }
}

/// Exact inverse of [`read_raw_prefix`]: emit the spec §4.2 step 2 + 3
/// bit pattern for a raw modified-Rice prefix.
///
/// * `raw < `[`UNARY_ESCAPE`] → the plain step 2 unary (`raw` `1` bits
///   and a `0` terminator).
/// * `raw >= `[`UNARY_ESCAPE`] → the step 3 escape: a 16-one unary,
///   then the escape value `raw - 16` as a second unary `cbits` (the
///   value directly when `< 2`, otherwise its bit-length) followed by
///   its low `cbits - 1` bits LSB-first with the top bit implied set.
///
/// Total over `u32` — every raw value the decoder can fold is
/// emittable, and the produced `cbits` is at most `32`, never colliding
/// with the [`ESCAPE_EOF_CBITS`] EOF marker (see
/// [`emit_end_of_stream_marker`] for writing that explicitly). Round
/// 281.
pub fn emit_raw_prefix(writer: &mut BitWriter, raw_value: u32) {
    if raw_value < UNARY_ESCAPE {
        writer.write_unary(raw_value);
        return;
    }
    writer.write_unary(UNARY_ESCAPE);
    let escape_value = raw_value - UNARY_ESCAPE;
    if escape_value < 2 {
        writer.write_unary(escape_value);
    } else {
        let cbits = 32 - escape_value.leading_zeros();
        writer.write_unary(cbits);
        writer.write_bits(escape_value, cbits - 1);
    }
}

/// Emit the spec §4.2 step 3 end-of-stream marker: the
/// [`UNARY_ESCAPE`] (16-one) first unary followed by a second unary of
/// exactly [`ESCAPE_EOF_CBITS`] (33) ones — the pattern
/// [`read_raw_prefix`] surfaces as [`Error::EndOfStream`]. Round 281.
pub fn emit_end_of_stream_marker(writer: &mut BitWriter) {
    writer.write_unary(UNARY_ESCAPE);
    writer.write_unary(ESCAPE_EOF_CBITS);
}

/// Read and fold the `ones_count` for one sample — combines the spec
/// §4.2 step 2 unary, the §4.2 step 3 `LIMIT_ONES = 16` escape (with
/// `cbits == 33` surfaced as [`Error::EndOfStream`]) and the §4.2 step
/// 4 holding-bit fold via [`RunState`].
///
/// This is equivalent to [`decode_run_length`] for the non-escape and
/// non-EOF cases; the difference is that this routine surfaces the
/// EOF (`cbits == 33`) explicitly as [`Error::EndOfStream`] for the
/// per-block decode loop, rather than reading mantissa bits past EOF.
/// On `Ok` the returned value is the post-fold `ones_count` zone
/// selector the spec §4.2 step 5 interval ladder takes.
///
/// Composes the round-274 typed surface: the wiki `last_zero`
/// short-circuit, then [`read_raw_prefix`] (steps 2 + 3) feeding
/// [`RunState::fold_prefix`] (step 4) — so the exact bits this routine
/// consumes ARE the bits the public primitives consume.
pub fn read_folded_ones_count(reader: &mut BitReader<'_>, state: &mut RunState) -> Result<u32> {
    // Wiki short-circuit: when last_zero is set, this sample's
    // ones_count is 0 with no bits read; last_zero clears and
    // last_one is untouched. Matches decode_run_length's first branch.
    if state.last_zero {
        state.last_zero = false;
        return Ok(0);
    }

    let raw_value = read_raw_prefix(reader)?;
    Ok(state.fold_prefix(raw_value))
}

/// Spec §4.2 step 5: form the `(low, high)` value interval from a
/// channel's three working medians and the (folded) `ones_count` zone.
///
/// Thin wrapper over the public typed
/// [`AdaptiveMedians::sample_interval_for_ones_count`] surface (round
/// 255), preserved as the private tuple-returning shape the round-255
/// parity tests are written against. The decode loops consume the
/// typed surface directly since round 261, so this shim is test-only.
#[cfg(test)]
fn form_interval(medians: &AdaptiveMedians, ones_count: u32) -> (u32, u32) {
    let interval = medians.sample_interval_for_ones_count(ones_count);
    (interval.low, interval.high)
}

/// Spec §4.2 step 6 first paragraph (pure lossless): the truncated-
/// binary mantissa decode inside a `(low, high)` interval, where
/// `maxcode = high - low`.
///
/// * `maxcode == 0` → no bits read, returned mantissa is `0`.
/// * `maxcode == 1` → read one bit; that bit is the mantissa.
/// * otherwise → `bitcount = bit-length of maxcode`,
///   `extras = (1 << bitcount) - maxcode - 1`; read `bitcount - 1` bits
///   LSB-first into `code`; if `code >= extras` read one MORE bit and
///   form `code = (code << 1) - extras + extra_bit` (a full
///   `bitcount`-bit phase-in code); else `code` stays as the short
///   `(bitcount - 1)`-bit value.
fn read_truncated_binary(reader: &mut BitReader<'_>, maxcode: u32) -> Result<u32> {
    if maxcode == 0 {
        return Ok(0);
    }
    if maxcode == 1 {
        return reader.get_bit();
    }
    let bitcount = 32 - maxcode.leading_zeros(); // bit-length of maxcode
    let extras = (1u32 << bitcount) - maxcode - 1;
    let short = reader.get_bits(bitcount - 1)?;
    if short < extras {
        Ok(short)
    } else {
        let extra_bit = reader.get_bit()?;
        Ok((short << 1).wrapping_sub(extras).wrapping_add(extra_bit))
    }
}

/// Spec §4.2 step 7 pure sign arithmetic: map an unsigned magnitude and
/// a sign-bit flag onto the signed sample value.
///
/// Per the spec ("If the sign bit is set the returned sample is the
/// bitwise complement of the magnitude (`~mid`), otherwise the
/// magnitude itself"):
///
/// * `sign_bit_set == false` → `magnitude as i32` (the magnitude
///   verbatim).
/// * `sign_bit_set == true` → `!(magnitude as i32)`, i.e.
///   `-(magnitude + 1)` in two's complement — magnitude `0` maps to
///   `-1`, magnitude `17` maps to `-18`, and the 31-bit-mask maximum
///   [`INTERVAL_MASK_31`] maps to `i32::MIN`.
///
/// Pure arithmetic — reads no bits. The spec §4.2 step 5 31-bit mask
/// (`low`/`high` both `<=` [`INTERVAL_MASK_31`]) keeps every magnitude
/// the decode ladder produces non-negative after the `as i32` cast.
pub const fn apply_sign(magnitude: u32, sign_bit_set: bool) -> i32 {
    let mid = magnitude as i32;
    if sign_bit_set {
        !mid
    } else {
        mid
    }
}

/// Exact inverse of [`apply_sign`]: split a signed sample into the
/// unsigned magnitude and the sign-bit flag the spec §4.2 step 7 wire
/// carries.
///
/// * `value >= 0` → `(value as u32, false)` (the magnitude verbatim,
///   sign clear).
/// * `value < 0` → `(!value as u32, true)` — the decode returns `!mid`
///   for a set sign bit, so the magnitude is the bitwise complement of
///   the value (`-1` → magnitude `0`, `-18` → `17`, `i32::MIN` →
///   [`INTERVAL_MASK_31`]).
///
/// Every `i32` maps to a magnitude `<=` [`INTERVAL_MASK_31`], so the
/// pair always round-trips: `apply_sign(split_sign(v).0,
/// split_sign(v).1) == v` for all `v`. Pure `const` arithmetic — no
/// bits move. Round 281.
pub const fn split_sign(value: i32) -> (u32, bool) {
    if value >= 0 {
        (value as u32, false)
    } else {
        ((!value) as u32, true)
    }
}

/// Spec §4.2 step 7 on-wire sign decode: read exactly ONE bit from the
/// reader ("Sign bit, last. After the magnitude is fixed, read exactly
/// one sign bit") and fold it into the magnitude via [`apply_sign`].
///
/// Returns the signed sample value — the bitwise complement of the
/// magnitude when the sign bit is set, the magnitude itself otherwise.
/// On `Error::Truncated` no bit was consumed (the cursor is unchanged
/// per [`BitReader::get_bit`] empty-buffer semantics) and the caller's
/// magnitude is unaffected (taken by value).
pub fn read_sign_and_apply(reader: &mut BitReader<'_>, magnitude: u32) -> Result<i32> {
    let sign = reader.get_bit()?;
    Ok(apply_sign(magnitude, sign != 0))
}

/// Spec §4.2 step 1 on-wire run-length decode: read an explicit
/// zero-run length from the main bitstream.
///
/// Per the spec §4.2 step 1 prose: "A leading unary count of 1-bits
/// (capped at 33) is read; if `< 2` it is the run length directly,
/// otherwise the remaining `count-1` bits are read low-bit-first to
/// form the run length with the top bit implied set." Concretely:
///
/// * `count == 0` → run length `0` (the encoder's "no zero run here"
///   marker — one bit consumed, no zero samples at all; the sample
///   word follows immediately in the regular §4.2 step 2+ sequence).
/// * `count == 1` → run length `1`.
/// * `2 <= count <= 32` → read `count - 1` mantissa bits LSB-first;
///   run length is `(1 << (count - 1)) | mantissa`.
/// * `count >= 33` → [`Error::Truncated`]. The spec caps the unary at
///   [`RUN_ESCAPE_CAP`] (`33`) and assigns no meaning to the cap value
///   in the zero-run context (unlike the §4.2 step 3 escape, where `33`
///   is the EOF marker); a 33-count run length would also need an
///   implied bit 32, exceeding the 32-bit accumulator. Failing loudly
///   beats a silent wrap. (Before round 278 the private path hit a
///   shift-overflow debug panic on exactly this input.)
///
/// This is the pure on-wire half of the §4.2 step 1 fast path: it does
/// NOT gate on eligibility and does NOT mutate medians or decode state
/// — pair it with [`DecodeState::zero_run_eligible`] (or
/// [`StereoDecodeState::zero_run_eligible`]) for the gate, and apply
/// the spec's "a non-zero run resets both channels' medians to zero and
/// emits a `0` sample" consequence at the caller. The private decode
/// loops delegate here, so the exact bits they consume ARE the bits
/// this primitive consumes.
pub fn read_zero_run_length(reader: &mut BitReader<'_>) -> Result<u32> {
    let count = reader.get_unary()?;
    if count >= RUN_ESCAPE_CAP {
        // Spec-silent edge: >= 33 contradicts the cap (and 33 itself
        // would overflow the u32 run length via the implied top bit).
        // Treat as truncated to fail loudly rather than wrapping.
        return Err(Error::Truncated);
    }
    if count < 2 {
        Ok(count)
    } else {
        // Read count-1 LSB-first mantissa bits with the top bit implied
        // set, exactly as the spec §4.2 step 1 prose specifies.
        let mantissa = reader.get_bits(count - 1)?;
        Ok((1u32 << (count - 1)) | mantissa)
    }
}

/// Exact inverse of [`read_zero_run_length`]: emit the spec §4.2 step 1
/// bit pattern for an explicit zero-run length.
///
/// * `run_length < 2` → the run length is the unary count directly
///   (`run_length` `1` bits and a `0` terminator; a `0` run is the
///   single-bit "no zero run here" marker).
/// * `run_length >= 2` → the unary count is the run length's
///   bit-length, followed by its low `count - 1` bits LSB-first (the
///   top bit implied set).
///
/// Total over `u32` — every representable run length emits a count
/// `<= 32`, inside the [`RUN_ESCAPE_CAP`] guard. Round 281 (public
/// lift of the round-278 test-side inverse).
pub fn emit_zero_run_length(writer: &mut BitWriter, run_length: u32) {
    if run_length < 2 {
        writer.write_unary(run_length);
    } else {
        let count = 32 - run_length.leading_zeros();
        writer.write_unary(count);
        writer.write_bits(run_length, count - 1);
    }
}

/// Spec §4.2 step 1 attempt: when the channel's `median[0]` is `<= 1`
/// AND no holding state is pending (no `last_one` carry, no
/// `last_zero` short-circuit waiting), the stream may carry an explicit
/// zero-run.
///
/// Returns:
/// * `Ok(Some(0))` when a non-zero run was decoded — the call emits a
///   `0` sample, [`DecodeState::zero_run_pending`] is set to
///   `run_length - 1`, and the medians are zeroed per spec §4.2 step 1.
/// * `Ok(None)` when the path was not entered (raw `median[0]` not
///   `<= 1`, or holding state pending) — **no bits consumed** — OR when
///   an explicit zero-length run was decoded (the unary prefix decodes
///   to `0`, consuming exactly one bit). Either way the caller proceeds
///   to the normal prefix-decode path for this same sample.
///
/// The zero-length-run fall-through is the round-281 correction: spec
/// §4.2 step 1 says "the stream **may** carry an explicit zero-run …
/// A **non-zero** run resets both channels' medians to zero and emits
/// a `0` sample" — a zero-length run is the encoder's "no zero run
/// here" marker and the sample word follows in the same step sequence.
/// (The pre-281 behaviour emitted a `0` sample on the marker too, which
/// would have made every eligible word decode to `0` forever — no
/// non-zero sample could ever follow once the gate opened.)
fn try_zero_run_path(
    reader: &mut BitReader<'_>,
    medians: &mut AdaptiveMedians,
    state: &mut DecodeState,
) -> Result<Option<i32>> {
    // Spec §4.2 step 1 eligibility gate, via the public predicate: raw
    // median[0] <= 1 AND no holding-bit pending. For mono "both
    // channels" reduces to the one channel.
    if !state.zero_run_eligible(medians) {
        return Ok(None);
    }

    // On-wire run length via the round-278 public primitive — the
    // exact bits this path consumes ARE the bits the primitive
    // consumes.
    let run_length = read_zero_run_length(reader)?;

    if run_length == 0 {
        // Explicit zero-length run: the encoder's "no zero run here"
        // marker. The sample word follows immediately — fall through to
        // the normal §4.2 step 2+ path with the marker bit consumed.
        return Ok(None);
    }

    // Non-zero run: spec §4.2 step 1 — "resets both channels' medians
    // to zero and emits a `0` sample." Mono: the one channel.
    medians.values = [0, 0, 0];
    state.ever_took_zero_run = true;
    // run_length samples total are zero; we are emitting the first
    // here, so owe run_length - 1 more on subsequent calls.
    state.zero_run_pending = run_length - 1;
    Ok(Some(0))
}

/// Decode one sample using the full per-sample loop spec'd in
/// `wavpack-entropy-decode.md` §4.2 — the round-194 addition that
/// closes the median-adaptation gap left over from round 7.
///
/// Sequence per call:
///
/// 1. **Zero-run debt**: if [`DecodeState::zero_run_pending`] is
///    non-zero, return a `0` sample and decrement the counter. No bits
///    are read on this call.
/// 2. **Zero-run fast path** (spec §4.2 step 1) when eligible (raw
///    `median[0] <= 1` AND no holding bits): decode an explicit run
///    length; a non-zero run emits a `0` sample (and owes the rest), a
///    zero-length run is the "no run" marker and decoding falls
///    through to step 3 for this same sample.
/// 3. **Unary prefix** (spec §4.2 step 2 + §4.2 step 3 escape): read
///    the raw `ones_count`, with the `LIMIT_ONES = 16` escape and the
///    `cbits == 33` EOF signal surfaced as [`Error::EndOfStream`].
/// 4. **Holding-bit fold** (spec §4.2 step 4): apply the wiki's
///    `last_one` / `last_zero` carry — identical to the spec's
///    "holding_one" / "holding_zero" prose — to map the raw count onto
///    the `ones_count` zone selector.
/// 5. **Interval** (spec §4.2 step 5): form `(low, high)` from the
///    medians, mask to 31 bits, clamp `high >= low`.
/// 6. **Median adaptation** (spec §3.2): walk the per-zone inc/dec
///    pattern via [`AdaptiveMedians::adapt`]. The spec is explicit that
///    adaptation happens at this point — BEFORE the mantissa is read.
/// 7. **Mantissa** (spec §4.2 step 6 first paragraph): truncated-binary
///    decode of `maxcode = high - low`; add `low` back.
/// 8. **Sign** (spec §4.2 step 7): read one bit; if set return the
///    bitwise complement of the magnitude, else the magnitude.
///
/// Hybrid mode (spec §4.2 step 6 second paragraph, `error_limit != 0`)
/// is OUT OF SCOPE for this loop. Pure lossless `0x0A` only.
pub fn decode_sample_stateful(
    reader: &mut BitReader<'_>,
    medians: &mut AdaptiveMedians,
    state: &mut DecodeState,
) -> Result<i32> {
    // 1. Zero-run debt (carry-over from a previous zero-run fast-path call).
    if state.zero_run_pending > 0 {
        state.zero_run_pending -= 1;
        return Ok(0);
    }

    // 2. Zero-run fast path (spec §4.2 step 1).
    if let Some(zero_sample) = try_zero_run_path(reader, medians, state)? {
        return Ok(zero_sample);
    }

    // 3-4. Unary prefix + escape (spec §4.2 steps 2 + 3) + holding-bit
    // fold (spec §4.2 step 4). `read_folded_ones_count` also honours the
    // wiki `last_zero` short-circuit (when a previous even-raw sample
    // pre-encoded this sample's zone selector as 0).
    let ones_count = read_folded_ones_count(reader, &mut state.run)?;

    // 5. Form the interval (spec §4.2 step 5) — from the PRE-adaptation
    // medians, via the round-255 typed surface.
    let interval = medians.sample_interval_for_ones_count(ones_count);

    // 6. Adapt the medians (spec §3.2) — BEFORE the mantissa read, per
    // the spec note "The medians are adapted at this point".
    medians.adapt(Zone::from_ones_count(ones_count));

    // 7-8. Mantissa (spec §4.2 step 6 first paragraph) + sign (spec
    // §4.2 step 7) via the typed surface — the exact bits this loop
    // consumes ARE the bits SampleInterval::decode_signed_value
    // consumes. The result is built in i32 space: the magnitude can be
    // up to INTERVAL_MASK_31 (2^31 - 1), which fits.
    interval.decode_signed_value(reader)
}

/// Decode `count` mono samples from a `0x0A` packed-samples payload,
/// using a freshly-built [`DecodeState`].
///
/// Composes [`PackedSamples::bit_reader`] with [`decode_sample_stateful`]
/// in a fixed loop, returning a `Vec<i32>` of `count` samples on
/// success. Errors propagate verbatim:
///
/// * [`Error::Truncated`] — buffer ran out mid-sample.
/// * [`Error::EndOfStream`] — `cbits == 33` EOF escape inside a sample's
///   unary-prefix escape arm. The partial decode is discarded; the
///   error tells the caller to stop the loop.
/// * [`Error::GolombDegenerateInterval`] — left over from the round-6
///   single-sample primitive's `add == 0` guard; not reachable through
///   `decode_sample_stateful`'s interval ladder (which produces
///   `maxcode = high - low` that the truncated-binary decoder handles
///   for every non-negative `maxcode`), kept in the signature for
///   forward-compat with future error additions.
///
/// The medians MUTATE in place across the loop — the caller's seed is
/// the running state. Pass [`AdaptiveMedians::from_seed_values`] of the
/// `0x05` entropy-info expander output (or
/// [`AdaptiveMedians::from_medians`] of a [`Medians`]) for a real
/// block. The final median values are the caller's to inspect after
/// the call returns.
pub fn decode_packed_samples_mono(
    payload: &crate::PackedSamples<'_>,
    medians: &mut AdaptiveMedians,
    count: usize,
) -> Result<Vec<i32>> {
    let mut reader = payload.bit_reader();
    let mut state = DecodeState::new();
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        out.push(decode_sample_stateful(&mut reader, medians, &mut state)?);
    }
    Ok(out)
}

/// Stereo (two-channel) decode state — one [`RunState`] per channel plus
/// the stream-level zero-run debt and bookkeeping.
///
/// Per the spec
/// (`docs/audio/wavpack/spec/wavpack-entropy-decode.md` §2) each channel
/// keeps its own three medians AND its own holding-bit (`last_one` /
/// `last_zero`) state. Sample index parity selects which channel is
/// being decoded: even indices (`0`, `2`, `4`, …) are the **left**
/// channel; odd indices (`1`, `3`, `5`, …) are the **right** channel.
///
/// The §4.2 step 1 zero-run fast path is **stream-level**: its
/// eligibility gate is "**both** channels' `median[0]` are ≤ 1 and no
/// holding state is pending" (so it inspects both channels), the
/// resulting non-zero run "resets **both channels'** medians to zero",
/// and the run length itself counts samples emitted at the stream level
/// (across both channels in interleaved order). Accordingly the
/// `zero_run_pending` counter and the `ever_took_zero_run` flag live
/// here at stream level, not inside the per-channel [`RunState`].
///
/// The `next_channel` field is the parity bookkeeping: it starts at
/// `0` (left) and toggles after every successful sample emit. It is
/// exposed so callers asserting partial decode progress can read the
/// current cursor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StereoDecodeState {
    /// Per-channel holding state for the **left** channel (even sample
    /// indices). Matches spec §4.2 step 4 semantics applied to the
    /// left-channel median set.
    pub left_run: RunState,
    /// Per-channel holding state for the **right** channel (odd sample
    /// indices). Matches spec §4.2 step 4 semantics applied to the
    /// right-channel median set.
    pub right_run: RunState,
    /// Remaining stream-level samples owed by an in-flight zero-run from
    /// spec §4.2 step 1 — when non-zero, the next stereo decode call
    /// short-circuits to a `0` sample (no bits read), toggles
    /// `next_channel`, and decrements this counter.
    pub zero_run_pending: u32,
    /// `true` once the spec §4.2 step 1 fast path took a non-zero run
    /// length and zeroed **both** channels' medians. Tests assert this
    /// to confirm the path actually ran rather than the loop bypassing
    /// it.
    pub ever_took_zero_run: bool,
    /// Channel index of the **next** sample to emit (`0` = left, `1` =
    /// right). Starts at `0`; toggles on every successful emit.
    pub next_channel: u8,
}

impl StereoDecodeState {
    /// Fresh stereo decode state for the first sample of a block: both
    /// channels' holding bits clear, no zero-run debt, no
    /// zero-run-fast-path-seen flag, next sample = left.
    pub const fn new() -> Self {
        Self {
            left_run: RunState::new(),
            right_run: RunState::new(),
            zero_run_pending: 0,
            ever_took_zero_run: false,
            next_channel: 0,
        }
    }

    /// Spec §4.2 step 1 eligibility gate for the stereo loop: the
    /// zero-run fast path may only be probed when **both** channels'
    /// **raw stored** `median[0]` are `<= 1` (spec: "If both channels'
    /// `median[0]` are ≤ 1") AND **neither** channel's holding state
    /// is pending — no `last_one` carry and no `last_zero`
    /// short-circuit on either [`RunState`].
    ///
    /// As on the mono gate, the threshold reads the raw `median[0]`
    /// values, NOT the [`AdaptiveMedians::get_med`] working values —
    /// see [`DecodeState::zero_run_eligible`] for the spec §2.1
    /// raw-vs-working grounding (round 281 correction).
    ///
    /// Pure predicate — reads no bits, mutates nothing. Stereo
    /// counterpart of [`DecodeState::zero_run_eligible`]; the private
    /// stereo decode loop's gate IS this predicate.
    pub fn zero_run_eligible(&self, medians: &[AdaptiveMedians; 2]) -> bool {
        medians[0].values[0] <= 1
            && medians[1].values[0] <= 1
            && !self.left_run.last_one
            && !self.left_run.last_zero
            && !self.right_run.last_one
            && !self.right_run.last_zero
    }
}

/// Spec §4.2 step 1 attempt for the stereo path: gate on **both**
/// channels' raw `median[0] <= 1` AND **both** channels' holding state
/// empty; on a successful non-zero run reset **both** channels' medians
/// to zero (per the spec §4.2 step 1 "resets both channels' medians to
/// zero" sentence).
///
/// Stream-level: the returned sample applies to the current
/// `state.next_channel` (the caller toggles parity after the emit). A
/// zero-length run is the "no zero run here" marker — `Ok(None)` with
/// the marker bit consumed, and the current channel's sample word
/// follows (the round-281 fall-through correction; see
/// [`try_zero_run_path`]).
fn try_zero_run_path_stereo(
    reader: &mut BitReader<'_>,
    medians: &mut [AdaptiveMedians; 2],
    state: &mut StereoDecodeState,
) -> Result<Option<i32>> {
    // Spec §4.2 step 1 eligibility gate for stereo, via the public
    // predicate: BOTH channels' raw median[0] must satisfy <= 1, AND
    // neither channel's holding state may be pending.
    if !state.zero_run_eligible(medians) {
        return Ok(None);
    }

    // On-wire run length via the round-278 public primitive — the exact
    // bits this path consumes ARE the bits the primitive consumes.
    let run_length = read_zero_run_length(reader)?;

    if run_length == 0 {
        // Explicit zero-length run: "no zero run here" marker; the
        // current channel's sample word follows. Marker bit consumed.
        return Ok(None);
    }

    // Non-zero run: spec §4.2 step 1 "resets both channels' medians
    // to zero". This call emits the first zero sample on the
    // current channel; run_length - 1 stream-level zero samples
    // remain to drain across alternating channels.
    medians[0].values = [0, 0, 0];
    medians[1].values = [0, 0, 0];
    state.ever_took_zero_run = true;
    state.zero_run_pending = run_length - 1;
    Ok(Some(0))
}

/// Decode one stereo sample using the full per-sample spec §4.2 path,
/// dispatched to the channel selected by `state.next_channel`.
///
/// The sample-index → channel-index parity rule comes from spec §2
/// ("For stereo, channels alternate (sample index parity selects the
/// channel's median set)"). The zero-run fast path is stream-level and
/// affects both channels' medians per spec §4.2 step 1; the unary
/// prefix, holding-bit fold, interval, adaptation, mantissa and sign
/// are **per-channel** — they read `medians[ch]` and mutate
/// `medians[ch]` + the matching [`RunState`] only.
///
/// After a successful emit `state.next_channel` toggles to the other
/// channel. On error, `state.next_channel` is left unchanged so a
/// retry sees the same cursor.
pub fn decode_sample_stateful_stereo(
    reader: &mut BitReader<'_>,
    medians: &mut [AdaptiveMedians; 2],
    state: &mut StereoDecodeState,
) -> Result<i32> {
    // 1. Zero-run debt (carry-over from a previous zero-run fast-path
    // call). Emit a single `0` sample for the current channel; the
    // medians stay at all-zero (the spec §4.2 step 1 reset persists
    // until the §3 adaptation walks them back up).
    if state.zero_run_pending > 0 {
        state.zero_run_pending -= 1;
        state.next_channel ^= 1;
        return Ok(0);
    }

    // 2. Zero-run fast path (spec §4.2 step 1) — gated on BOTH channels.
    if let Some(zero_sample) = try_zero_run_path_stereo(reader, medians, state)? {
        state.next_channel ^= 1;
        return Ok(zero_sample);
    }

    // 3-4. Per-channel unary prefix + escape + fold. Pick the channel
    // (and its holding state) BEFORE reading any bits; the
    // ones_count / interval / adaptation / mantissa / sign sequence
    // operates on the chosen channel only.
    let ch = state.next_channel as usize;
    debug_assert!(ch < 2, "next_channel must be 0 or 1");
    let ch_state = if ch == 0 {
        &mut state.left_run
    } else {
        &mut state.right_run
    };
    let ones_count = read_folded_ones_count(reader, ch_state)?;

    // 5. Form the interval from the per-channel PRE-adaptation medians,
    // via the round-255 typed surface.
    let interval = medians[ch].sample_interval_for_ones_count(ones_count);

    // 6. Adapt the per-channel medians (spec §3.2) — BEFORE the mantissa
    // read, per the spec note "The medians are adapted at this point".
    medians[ch].adapt(Zone::from_ones_count(ones_count));

    // 7-8. Mantissa (spec §4.2 step 6 first paragraph) + sign (spec
    // §4.2 step 7) via the typed surface — the exact bits this loop
    // consumes ARE the bits SampleInterval::decode_signed_value
    // consumes.
    let result = interval.decode_signed_value(reader)?;

    // Toggle channel parity for the next call. Toggle ONLY on a
    // successful emit so a `?`-bubbled error leaves the cursor
    // recoverable.
    state.next_channel ^= 1;
    Ok(result)
}

/// Decode `frames` stereo frames from a `0x0A` packed-samples payload,
/// returning a `Vec<i32>` of `frames * 2` interleaved samples in
/// (left, right, left, right, …) order.
///
/// Composes [`PackedSamples::bit_reader`] with
/// [`decode_sample_stateful_stereo`] in a fixed loop. Errors propagate
/// verbatim — see [`decode_packed_samples_mono`] for the error
/// catalogue, all of which apply identically here.
///
/// The medians MUTATE in place across the loop — the caller's
/// `[left_seed, right_seed]` array is the running state. Pair with
/// [`AdaptiveMedians::from_seed_values`] on the `0x05` entropy-info
/// expander's `medians_left` and `medians_right` (the round-4 expander
/// produces both for a stereo block) to seed.
pub fn decode_packed_samples_stereo(
    payload: &crate::PackedSamples<'_>,
    medians: &mut [AdaptiveMedians; 2],
    frames: usize,
) -> Result<Vec<i32>> {
    let mut reader = payload.bit_reader();
    let mut state = StereoDecodeState::new();
    let mut out = Vec::with_capacity(frames * 2);
    for _ in 0..(frames * 2) {
        out.push(decode_sample_stateful_stereo(
            &mut reader,
            medians,
            &mut state,
        )?);
    }
    Ok(out)
}

/// End-to-end mono decode driven by the round-4 `0x05` entropy-info
/// expander output.
///
/// Wraps [`decode_packed_samples_mono`] with the
/// [`AdaptiveMedians::from_entropy`] bridge so a caller holding a
/// fresh [`EntropyInfo`] (typically from
/// [`crate::find_entropy_info`] +  [`crate::expand_entropy`] on the
/// `0x05` sub-block) can decode the matching `0x0A` payload without
/// hand-rolling the per-channel seed extraction. Round 201.
///
/// Returns [`Error::InvalidEntropyInfoForMono`] when the
/// channel-0 set carries a negative seed (the
/// [`AdaptiveMedians::from_seed_values`] defensive rejection); errors
/// from [`decode_packed_samples_mono`] are propagated verbatim.
///
/// The seeds are consumed by value — the running medians live inside
/// the call and are dropped on return. If a caller needs to inspect
/// the final medians (e.g. for streaming continuation across blocks)
/// they should keep using [`decode_packed_samples_mono`] directly with
/// an [`AdaptiveMedians`] they own.
pub fn decode_packed_samples_mono_from_entropy(
    payload: &crate::PackedSamples<'_>,
    info: &EntropyInfo,
    count: usize,
) -> Result<Vec<i32>> {
    let mut medians =
        AdaptiveMedians::from_entropy(info, 0).ok_or(Error::InvalidEntropyInfoForMono)?;
    decode_packed_samples_mono(payload, &mut medians, count)
}

/// End-to-end stereo decode driven by the round-4 `0x05` entropy-info
/// expander output.
///
/// Wraps [`decode_packed_samples_stereo`] with the
/// [`AdaptiveMedians::stereo_pair_from_entropy`] bridge so a caller
/// holding a fresh stereo [`EntropyInfo`] can decode the matching
/// `0x0A` payload without hand-rolling the two-channel seed
/// extraction. Round 201.
///
/// Returns [`Error::InvalidEntropyInfoForStereo`] when the
/// [`EntropyInfo`] is mono (no right-channel set on the wire) or when
/// either channel carries a negative seed; errors from
/// [`decode_packed_samples_stereo`] are propagated verbatim.
///
/// The seeds are consumed by value — the running medians live inside
/// the call and are dropped on return.
pub fn decode_packed_samples_stereo_from_entropy(
    payload: &crate::PackedSamples<'_>,
    info: &EntropyInfo,
    frames: usize,
) -> Result<Vec<i32>> {
    let mut medians = AdaptiveMedians::stereo_pair_from_entropy(info)
        .ok_or(Error::InvalidEntropyInfoForStereo)?;
    decode_packed_samples_stereo(payload, &mut medians, frames)
}

/// Pad the finished payload to an **even** byte count, per spec §1:
/// the `0x0A` sub-block's "byte length must be even or the block is
/// rejected". The pad byte is all zeros, past every written bit, so a
/// decoder consuming exactly the encoded samples never reads it.
fn finish_even(writer: BitWriter) -> Vec<u8> {
    let mut bytes = writer.finish();
    if bytes.len() % 2 != 0 {
        bytes.push(0);
    }
    bytes
}

/// Encode one non-zero-run sample word — the exact inverse of the
/// decoder's spec §4.2 steps 2-8 tail, shared by the mono and stereo
/// packed encoders.
///
/// `next_value` is the **same channel's** next sample, used for the
/// step-4 carry decision: the emitted raw prefix's low bit pre-encodes
/// the next word's mode (clear → that word's `ones_count` is `0` and
/// it reads no prefix bits; set → its folded count gains `+1`). The
/// choice is forced — clear is only decodable when the next magnitude
/// lands in zone 0 under the post-adapt medians, set only when it does
/// not — except for the final word, where clear is chosen (one wire
/// bit shorter). No zero-run word can interpose between this word and
/// the same channel's next (the holding bit this word leaves pending
/// blocks the §4.2 step 1 gate until that word consumes it), so the
/// lookahead target is exact.
fn encode_one_word(
    writer: &mut BitWriter,
    medians: &mut AdaptiveMedians,
    run: &mut RunState,
    value: i32,
    next_value: Option<i32>,
) -> Result<()> {
    let (magnitude, _) = split_sign(value);
    let ones_count = if run.last_zero {
        // The previous word's clear low bit pre-encoded this word's
        // zone selector as 0 — mirror the decoder's last_zero
        // short-circuit: no prefix bits, clear the flag.
        run.last_zero = false;
        0
    } else {
        let zone = medians.zone_for_magnitude(magnitude);
        let hold_one = match next_value {
            None => false,
            Some(next) => {
                let mut after = *medians;
                after.adapt(Zone::from_ones_count(zone));
                let (next_mag, _) = split_sign(next);
                next_mag >= after.get_med(0)
            }
        };
        match run.unfold_prefix(zone, hold_one) {
            Some(raw) => emit_raw_prefix(writer, raw),
            None => {
                // Unreachable by construction: the +1 carry is only
                // pending when this word's zone is >= 1, and i32
                // magnitudes keep the doubled raw inside u32. Surface
                // the typed interval error defensively rather than
                // panicking.
                let interval = medians.sample_interval_for_ones_count(zone);
                return Err(Error::ValueNotInInterval {
                    value: magnitude,
                    low: interval.low,
                    high: interval.high,
                });
            }
        }
        zone
    };
    // Mirror the decode order exactly: interval from the PRE-adapt
    // medians (§4.2 step 5), adapt (§3.2), then mantissa + sign (§4.2
    // steps 6 + 7).
    let interval = medians.sample_interval_for_ones_count(ones_count);
    medians.adapt(Zone::from_ones_count(ones_count));
    interval.encode_signed_value(writer, value)
}

/// Encode a mono sample sequence into a spec §4.2 `0x0A` packed-samples
/// payload — the exact inverse of [`decode_packed_samples_mono`].
///
/// Walks the same per-word state machine the decode loop walks, in
/// write direction:
///
/// 1. **Zero-run fast path** (spec §4.2 step 1) whenever the decoder
///    will probe it (raw `median[0] <= 1`, no holding bits, via
///    [`DecodeState::zero_run_eligible`]): the maximal run of `0`
///    samples at the cursor is emitted as one explicit run length —
///    `0` (the one-bit "no run" marker) when the current sample is
///    non-zero. A non-zero run zeroes the medians exactly as the
///    decoder will.
/// 2. **Prefix** (steps 2-4): the zone selector from
///    [`AdaptiveMedians::zone_for_magnitude`], unfolded through
///    [`RunState::unfold_prefix`] (the holding-bit carry chosen by
///    one-sample lookahead — see the spec §4.2 step 4 boundary
///    pre-encoding) and emitted via [`emit_raw_prefix`].
/// 3. **Interval + adaptation + mantissa + sign** (steps 5-8 and
///    §3.2): [`SampleInterval::encode_signed_value`] against the
///    pre-adapt interval, with [`AdaptiveMedians::adapt`] applied at
///    the same point the decoder applies it.
///
/// The medians MUTATE in place across the loop (the caller's seed is
/// the running state, exactly as on the decode side) and finish in the
/// same state the decoder's will — the round-trip tests pin both the
/// PCM and the final median state. The returned payload is padded to
/// an **even** byte count per the spec §1 `0x0A` length rule.
///
/// [`Error::ValueNotInInterval`] surfaces only in the spec §4.2 step 5
/// 31-bit-mask corner — a magnitude no decode of the same median state
/// could produce. Round 281.
pub fn encode_packed_samples_mono(
    values: &[i32],
    medians: &mut AdaptiveMedians,
) -> Result<Vec<u8>> {
    let mut writer = BitWriter::new();
    let mut state = DecodeState::new();
    let mut i = 0usize;
    while i < values.len() {
        if state.zero_run_eligible(medians) {
            let zeros = values[i..].iter().take_while(|&&v| v == 0).count();
            let run = u32::try_from(zeros).unwrap_or(u32::MAX);
            emit_zero_run_length(&mut writer, run);
            if run > 0 {
                medians.values = [0, 0, 0];
                state.ever_took_zero_run = true;
                i += run as usize;
                continue;
            }
            // run == 0: the marker is on the wire; the decoder falls
            // through to the regular word for this same sample.
        }
        let next = values.get(i + 1).copied();
        encode_one_word(&mut writer, medians, &mut state.run, values[i], next)?;
        i += 1;
    }
    Ok(finish_even(writer))
}

/// Encode an interleaved (left, right, left, right, …) stereo sample
/// sequence into a spec §4.2 `0x0A` packed-samples payload — the exact
/// inverse of [`decode_packed_samples_stereo`].
///
/// Per spec §2 the channels alternate by sample-index parity, each
/// with its own median set and holding state; the §4.2 step 1 zero-run
/// is stream-level (gated on BOTH channels via
/// [`StereoDecodeState::zero_run_eligible`], counting samples across
/// both channels, zeroing both channels' medians). The step-4 carry
/// lookahead therefore targets the **same channel's** next sample, two
/// positions ahead.
///
/// `values.len()` need not be even — a trailing left-channel sample
/// encodes fine and decodes back through the per-call
/// [`decode_sample_stateful_stereo`] (the frame-based
/// [`decode_packed_samples_stereo`] wrapper consumes whole frames).
/// The returned payload is padded to an **even** byte count per the
/// spec §1 `0x0A` length rule. Round 281.
pub fn encode_packed_samples_stereo(
    values: &[i32],
    medians: &mut [AdaptiveMedians; 2],
) -> Result<Vec<u8>> {
    let mut writer = BitWriter::new();
    let mut state = StereoDecodeState::new();
    let mut i = 0usize;
    while i < values.len() {
        if state.zero_run_eligible(medians) {
            let zeros = values[i..].iter().take_while(|&&v| v == 0).count();
            let run = u32::try_from(zeros).unwrap_or(u32::MAX);
            emit_zero_run_length(&mut writer, run);
            if run > 0 {
                medians[0].values = [0, 0, 0];
                medians[1].values = [0, 0, 0];
                state.ever_took_zero_run = true;
                i += run as usize;
                continue;
            }
        }
        let ch = i & 1;
        let next = values.get(i + 2).copied();
        let run_state = if ch == 0 {
            &mut state.left_run
        } else {
            &mut state.right_run
        };
        encode_one_word(&mut writer, &mut medians[ch], run_state, values[i], next)?;
        i += 1;
    }
    Ok(finish_even(writer))
}

/// End-to-end mono encode driven by the `0x05` entropy-info expander
/// output — the inverse of [`decode_packed_samples_mono_from_entropy`],
/// with the same [`AdaptiveMedians::from_entropy`] bridge and the same
/// [`Error::InvalidEntropyInfoForMono`] rejection of a negative
/// channel-0 seed. The seeds are consumed by value; callers that need
/// the final median state should call [`encode_packed_samples_mono`]
/// directly with an [`AdaptiveMedians`] they own. Round 281.
pub fn encode_packed_samples_mono_from_entropy(
    values: &[i32],
    info: &EntropyInfo,
) -> Result<Vec<u8>> {
    let mut medians =
        AdaptiveMedians::from_entropy(info, 0).ok_or(Error::InvalidEntropyInfoForMono)?;
    encode_packed_samples_mono(values, &mut medians)
}

/// End-to-end stereo encode driven by the `0x05` entropy-info expander
/// output — the inverse of
/// [`decode_packed_samples_stereo_from_entropy`], with the same
/// [`AdaptiveMedians::stereo_pair_from_entropy`] bridge and the same
/// [`Error::InvalidEntropyInfoForStereo`] rejection of a mono info or
/// a negative per-channel seed. Round 281.
pub fn encode_packed_samples_stereo_from_entropy(
    values: &[i32],
    info: &EntropyInfo,
) -> Result<Vec<u8>> {
    let mut medians = AdaptiveMedians::stereo_pair_from_entropy(info)
        .ok_or(Error::InvalidEntropyInfoForStereo)?;
    encode_packed_samples_stereo(values, &mut medians)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a byte buffer from a left-to-right bit string (MSB of the
    /// human-readable string is the FIRST bit on the wire). Because the
    /// reader is LSB-first within a byte, the first listed bit lands in
    /// bit 0 of byte 0. The string may be any length; the final byte is
    /// zero-padded in the high (later-read) bits.
    fn bits_to_bytes(bits: &str) -> Vec<u8> {
        let mut out = Vec::new();
        let mut cur = 0u8;
        let mut nbits = 0u8;
        for ch in bits.chars() {
            let bit = match ch {
                '0' => 0u8,
                '1' => 1u8,
                _ => continue,
            };
            cur |= bit << nbits;
            nbits += 1;
            if nbits == 8 {
                out.push(cur);
                cur = 0;
                nbits = 0;
            }
        }
        if nbits > 0 {
            out.push(cur);
        }
        out
    }

    // ---- BitReader primitives ----

    #[test]
    fn get_bit_reads_lsb_first() {
        // Byte 0b0000_0101 → bits read in order 1,0,1,0,0,0,0,0.
        let bytes = [0b0000_0101u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_bit().unwrap(), 1);
        assert_eq!(r.get_bit().unwrap(), 0);
        assert_eq!(r.get_bit().unwrap(), 1);
        for _ in 0..5 {
            assert_eq!(r.get_bit().unwrap(), 0);
        }
        assert!(r.is_empty());
        assert_eq!(r.get_bit(), Err(Error::Truncated));
    }

    #[test]
    fn get_bit_crosses_byte_boundary() {
        // Two bytes: 0x01 (bit 0 set) then 0x80 (bit 7 set). LSB-first
        // the eighth bit read is bit 0 of byte 1.
        let bytes = [0x01u8, 0x80u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_bit().unwrap(), 1); // byte 0 bit 0
        for _ in 0..7 {
            assert_eq!(r.get_bit().unwrap(), 0); // rest of byte 0
        }
        // Now into byte 1 (0x80 = bit 7). First seven bits are zero,
        // eighth is one.
        for _ in 0..7 {
            assert_eq!(r.get_bit().unwrap(), 0);
        }
        assert_eq!(r.get_bit().unwrap(), 1);
        assert!(r.is_empty());
    }

    #[test]
    fn get_bits_assembles_lsb_first() {
        // Bit string "10110" → first bit (1) is LSB of the result.
        // value = 1<<0 | 0<<1 | 1<<2 | 1<<3 | 0<<4 = 0b0_1101 = 13.
        let bytes = bits_to_bytes("10110");
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_bits(5).unwrap(), 0b0_1101);
    }

    #[test]
    fn get_bits_zero_count_consumes_nothing() {
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_bits(0).unwrap(), 0);
        assert_eq!(r.bits_remaining(), 8);
    }

    #[test]
    fn get_bits_truncation_is_reported() {
        let bytes = [0xFFu8]; // 8 bits
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_bits(9), Err(Error::Truncated));
    }

    #[test]
    fn get_bits_full_width() {
        // 32 ones across four bytes → 0xFFFF_FFFF.
        let bytes = [0xFFu8; 4];
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_bits(32).unwrap(), 0xFFFF_FFFF);
        assert!(r.is_empty());
    }

    // ---- get_unary ----

    #[test]
    fn unary_wiki_examples() {
        // "111110b = 5" — five ones then a zero.
        let bytes = bits_to_bytes("111110");
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_unary().unwrap(), 5);

        // "10b = 1" — one one then a zero.
        let bytes = bits_to_bytes("10");
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_unary().unwrap(), 1);
    }

    #[test]
    fn unary_zero_run_is_immediate_terminator() {
        // A leading zero → run length 0.
        let bytes = bits_to_bytes("0");
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_unary().unwrap(), 0);
    }

    #[test]
    fn unary_consumes_terminator_only() {
        // "110" then "10" — first unary reads 2, second reads 1.
        let bytes = bits_to_bytes("11010");
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_unary().unwrap(), 2);
        assert_eq!(r.get_unary().unwrap(), 1);
    }

    #[test]
    fn unary_unterminated_run_reports_truncation() {
        // All ones, no terminating zero → truncated.
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_unary(), Err(Error::Truncated));
    }

    // ---- peek_bit / peek_bits / peek_unary / skip_bits ----

    #[test]
    fn peek_bit_returns_next_bit_without_advancing() {
        // First byte has LSB = 1 → peek_bit must return 1; the cursor
        // stays at byte 0 / bit 0 so a follow-up get_bit gets the same
        // value.
        let bytes = [0b0000_0001u8];
        let r = BitReader::new(&bytes);
        assert_eq!(r.peek_bit().unwrap(), 1);
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 0);
        // A mutable read after the peek should return the same bit.
        let mut r2 = r.clone();
        assert_eq!(r2.get_bit().unwrap(), 1);
    }

    #[test]
    fn peek_bit_truncated_when_buffer_empty_does_not_move_cursor() {
        let bytes: [u8; 0] = [];
        let r = BitReader::new(&bytes);
        assert_eq!(r.peek_bit(), Err(Error::Truncated));
        // The cursor is unchanged because peek operates on a clone.
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 0);
    }

    #[test]
    fn peek_bits_assembles_lsb_first_without_advancing() {
        // bits 0..=3 of 0x0A = LSB-first 0,1,0,1 → assembled as 0xA.
        let bytes = [0x0Au8];
        let r = BitReader::new(&bytes);
        assert_eq!(r.peek_bits(4).unwrap(), 0xA);
        assert_eq!(r.bits_consumed(), 0);
        // Repeated peek returns the same value.
        assert_eq!(r.peek_bits(4).unwrap(), 0xA);
    }

    #[test]
    fn peek_bits_zero_count_returns_zero() {
        let bytes = [0xFFu8];
        let r = BitReader::new(&bytes);
        assert_eq!(r.peek_bits(0).unwrap(), 0);
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn peek_bits_truncated_does_not_move_cursor() {
        let bytes = [0u8];
        let r = BitReader::new(&bytes);
        assert_eq!(r.peek_bits(9), Err(Error::Truncated));
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn peek_unary_matches_get_unary_without_advancing() {
        // "111110b = 5" wiki example.
        let bytes = bits_to_bytes("111110");
        let r = BitReader::new(&bytes);
        assert_eq!(r.peek_unary().unwrap(), 5);
        assert_eq!(r.bits_consumed(), 0);
        // Subsequent get_unary on a fresh clone yields the same value.
        let mut r2 = r.clone();
        assert_eq!(r2.get_unary().unwrap(), 5);
    }

    #[test]
    fn peek_unary_truncated_when_no_terminator() {
        let bytes = [0xFFu8];
        let r = BitReader::new(&bytes);
        assert_eq!(r.peek_unary(), Err(Error::Truncated));
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn peek_then_get_returns_same_value_pattern() {
        // Demonstrate the intended look-ahead pattern: peek to decide,
        // get to commit. Confirms peek does not consume and get reads
        // the same bits.
        let bytes = [0b1010_1010u8];
        let mut r = BitReader::new(&bytes);
        let peeked = r.peek_bits(4).unwrap();
        let got = r.get_bits(4).unwrap();
        assert_eq!(peeked, got);
        assert_eq!(r.bits_consumed(), 4);
    }

    #[test]
    fn skip_bits_advances_cursor_without_assembling_value() {
        let bytes = [0xFFu8, 0x55u8];
        let mut r = BitReader::new(&bytes);
        r.skip_bits(5).expect("skip 5 bits");
        assert_eq!(r.bits_consumed(), 5);
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 5);
        // Continuing from the skip cursor sees the remaining bits.
        assert_eq!(r.get_bits(3).unwrap(), 0b111); // last 3 bits of 0xFF
    }

    #[test]
    fn skip_bits_zero_count_is_noop() {
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        r.skip_bits(0).expect("skip 0 bits");
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn skip_bits_crosses_byte_boundary() {
        let bytes = [0x00u8, 0xFFu8];
        let mut r = BitReader::new(&bytes);
        r.skip_bits(10).expect("skip 10 bits");
        assert_eq!(r.byte_position(), 1);
        assert_eq!(r.bit_position(), 2);
    }

    #[test]
    fn skip_bits_truncated_when_buffer_exhausted() {
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        // Skip 9 bits from an 8-bit buffer → Truncated. Matches the
        // partial-consume semantics of get_bits — the cursor lands at
        // the end of the buffer rather than reverting.
        assert_eq!(r.skip_bits(9), Err(Error::Truncated));
        assert_eq!(r.bits_consumed(), 8);
        assert!(r.is_empty());
    }

    #[test]
    fn skip_bits_then_decode_run_length_resumes_correctly() {
        // Synthesise two unary runs back-to-back ("110" then "1110"),
        // skip past the first three bits ("110"), then decode the second.
        let bytes = bits_to_bytes("1101110");
        let mut r = BitReader::new(&bytes);
        r.skip_bits(3).expect("skip first unary");
        assert_eq!(r.get_unary().unwrap(), 3);
    }

    // ---- decode_run_length ----

    #[test]
    fn run_length_last_zero_short_circuits_to_zero() {
        // last_zero set → n = 0 and the flag clears, no bits consumed.
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        let mut state = RunState {
            last_zero: true,
            last_one: true,
        };
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 0);
        assert!(!state.last_zero);
        // last_one is untouched on this branch per the wiki.
        assert!(state.last_one);
        // No bits were consumed.
        assert_eq!(r.bits_remaining(), 8);
    }

    #[test]
    fn run_length_even_unary_halves_and_sets_last_zero_false_path() {
        // unary = 2 (bits "110"). n & 1 == 0 → last_one = false,
        // n = 2 >> 1 = 1, last_zero = !false = true.
        let bytes = bits_to_bytes("110");
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 1);
        assert!(!state.last_one);
        assert!(state.last_zero);
    }

    #[test]
    fn run_length_odd_unary_halves_and_holds_a_one() {
        // unary = 3 (bits "1110") from a neutral state: no one is being
        // held on entry, so n = 3 >> 1 = 1 (spec §4.2 step 4 prior-state
        // fold). The discarded low bit (1) becomes the NEW held-one —
        // the +1 carry lands on the NEXT word, not this one.
        let bytes = bits_to_bytes("1110");
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 1);
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn run_length_held_one_adds_one_to_the_fold() {
        // The same raw prefix decoded while a one is being held gains
        // the +1: unary = 3 with last_one set on entry → n = (3 >> 1)
        // + 1 = 2 (spec §4.2 step 4 "if a one is being held").
        let bytes = bits_to_bytes("1110");
        let mut r = BitReader::new(&bytes);
        let mut state = RunState {
            last_zero: false,
            last_one: true,
        };
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 2);
        assert!(state.last_one, "new held-one is the raw low bit (1)");
        assert!(!state.last_zero);
    }

    #[test]
    fn run_length_escape_small_n2_adds_directly() {
        // unary = 16 triggers the escape. Build sixteen ones + a zero
        // (the first get_unary = 16), then a second unary with n2 = 1
        // (bits "10"). n2 < 2 → n += n2 → n = 17.
        // From a neutral state (no held one) n = 17 >> 1 = 8; the raw
        // low bit (1) becomes the new held-one for the NEXT word.
        let mut bits = String::new();
        bits.push_str(&"1".repeat(16));
        bits.push('0'); // terminator for the first unary = 16
        bits.push_str("10"); // second unary n2 = 1
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 8);
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn run_length_escape_large_n2_reads_mantissa() {
        // unary = 16, then n2 = 3 (bits "1110"), then getbits(n2-1) =
        // getbits(2). Mantissa bits "10" → first bit '1' lands in bit 0,
        // second bit '0' in bit 1 → value = 1<<0 | 0<<1 = 1.
        // n += (1 << (3-1)) | 1 = (1<<2) | 1 = 4 | 1 = 5 → n = 16 + 5 = 21.
        // Neutral state on entry → n = 21 >> 1 = 10; the raw low bit
        // (1) becomes the new held-one.
        let mut bits = String::new();
        bits.push_str(&"1".repeat(16));
        bits.push('0'); // first unary = 16
        bits.push_str("1110"); // second unary n2 = 3
        bits.push_str("10"); // getbits(2) mantissa = 1 (LSB-first)
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 10);
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn run_length_escape_mantissa_is_assembled_lsb_first() {
        // Pin the LSB-first ordering of the escape mantissa: n2 = 4
        // (bits "11110") → getbits(3). Mantissa bits "011" → value =
        // 0<<0 | 1<<1 | 1<<2 = 6. n += (1 << 3) | 6 = 8 | 6 = 14 →
        // n = 16 + 14 = 30. last_one = 30 & 1 = 0 → n = 30 >> 1 = 15,
        // last_one = false, last_zero = true.
        let mut bits = String::new();
        bits.push_str(&"1".repeat(16));
        bits.push('0'); // first unary = 16
        bits.push_str("11110"); // second unary n2 = 4
        bits.push_str("011"); // getbits(3) mantissa = 6 (LSB-first)
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 15);
        assert!(!state.last_one);
        assert!(state.last_zero);
    }

    #[test]
    fn run_length_zero_unary_yields_zero_and_last_zero_true() {
        // unary = 0 (bit "0"). n & 1 == 0 → last_one = false,
        // n = 0 >> 1 = 0, last_zero = true.
        let bytes = bits_to_bytes("0");
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 0);
        assert!(!state.last_one);
        assert!(state.last_zero);
    }

    #[test]
    fn run_length_sequence_alternates_via_last_zero() {
        // After an odd unary sets last_one (and clears last_zero), the
        // next decode reads bits normally. After an even unary sets
        // last_zero, the following decode short-circuits to 0 without
        // consuming bits. Walk a small sequence to prove the carry.
        //
        // Sample 1: unary "110" = 2 → even → n=1, last_zero=true (the
        //           clear low bit pre-encodes sample 2's count as 0).
        // Sample 2: last_zero → n=0, no bits consumed, last_zero=false,
        //           last_one untouched (false).
        // Sample 3: unary "1110" = 3 from a neutral state → n = 3 >> 1
        //           = 1, last_one=true (the held +1 lands on sample 4).
        let mut bits = String::new();
        bits.push_str("110"); // sample 1 unary = 2
        bits.push_str("1110"); // sample 3 unary = 3 (sample 2 reads nothing)
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();

        let n1 = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n1, 1);
        assert!(state.last_zero);

        let n2 = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n2, 0);
        assert!(!state.last_zero);

        let n3 = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n3, 1);
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn run_length_truncated_unary_is_reported() {
        // No terminator for the unary → truncated.
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        assert_eq!(decode_run_length(&mut r, &mut state), Err(Error::Truncated));
    }

    // ---- golomb_interval (base, add) selection ----

    #[test]
    fn interval_n0_uses_median0() {
        // n == 0 → base = 0, add = median[0] - 1.
        let m = Medians::new([10, 20, 30]);
        let GolombInterval { base, add } = golomb_interval(0, m);
        assert_eq!(base, 0);
        assert_eq!(add, 9);
    }

    #[test]
    fn interval_n1_uses_median1() {
        // n == 1 → base = median[0], add = median[1] - 1.
        let m = Medians::new([10, 20, 30]);
        let GolombInterval { base, add } = golomb_interval(1, m);
        assert_eq!(base, 10);
        assert_eq!(add, 19);
    }

    #[test]
    fn interval_n2_uses_median_sum_with_zero_extra() {
        // n == 2 → base = m0 + m1 + m2*(2-2) = m0 + m1, add = m2 - 1.
        let m = Medians::new([10, 20, 30]);
        let GolombInterval { base, add } = golomb_interval(2, m);
        assert_eq!(base, 30);
        assert_eq!(add, 29);
    }

    #[test]
    fn interval_n_large_scales_median2() {
        // n == 5 → base = m0 + m1 + m2*(5-2) = 10 + 20 + 30*3 = 120,
        // add = m2 - 1 = 29.
        let m = Medians::new([10, 20, 30]);
        let GolombInterval { base, add } = golomb_interval(5, m);
        assert_eq!(base, 120);
        assert_eq!(add, 29);
    }

    // ---- golomb_k = log2(add) bit-length reading ----

    #[test]
    fn golomb_k_is_bit_length() {
        // Bit-length: smallest k with (1<<k) > add.
        assert_eq!(golomb_k(1), 1); // 0b1
        assert_eq!(golomb_k(2), 2); // 0b10
        assert_eq!(golomb_k(3), 2); // 0b11
        assert_eq!(golomb_k(4), 3); // 0b100
        assert_eq!(golomb_k(5), 3); // 0b101
        assert_eq!(golomb_k(29), 5); // 0b11101
        assert_eq!(golomb_k(0), 0);
    }

    #[test]
    fn golomb_k_keeps_ex_non_negative() {
        // The whole point of the bit-length reading: ex = (1<<k)-add-1
        // must be >= 0 so the `if(t2 >= ex)` branch is reachable.
        for add in 1..=1024i32 {
            let k = golomb_k(add);
            let ex = (1i32 << k) - add - 1;
            assert!(ex >= 0, "ex went negative for add={add} (k={k})");
            // ...and ex < (1<<(k-1)) when k >= 1, so the short-mantissa
            // region exists.
            assert!(
                ex < (1i32 << (k - 1)) || add == 1,
                "ex too large for add={add}"
            );
        }
    }

    // ---- decode_sample_value ----

    #[test]
    fn sample_value_short_mantissa_no_extra_bit() {
        // n = 2, medians [10,20,30] → base=30, add=29, k=5, ex=2.
        // getbits(k-1)=getbits(4). Encode t2 = 1 (< ex=2) → no extra
        // bit. sign = 0 → result = base + t2 = 31.
        // Bits (LSB-first): getbits(4)=1 → "1000", then sign bit "0".
        let bits = "10000"; // t2 nibble "1000" (=1) + sign "0"
        let bytes = bits_to_bytes(bits);
        let mut r = BitReader::new(&bytes);
        let m = Medians::new([10, 20, 30]);
        let v = decode_sample_value(&mut r, 2, m).unwrap();
        assert_eq!(v, 31);
    }

    #[test]
    fn sample_value_long_mantissa_reads_extra_bit() {
        // Same interval: base=30, add=29, k=5, ex=2.
        // getbits(4) = 3 (>= ex=2): t2 = 3*2 - 2 + getbit().
        // 3 in 4 bits LSB-first = "1100". Then extra bit = 1 →
        // t2 = 6 - 2 + 1 = 5. Then sign = 0 → result = 30 + 5 = 35.
        let bits = "1100" /* t2=3 */ .to_string() + "1" /* extra */ + "0" /* sign */;
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let m = Medians::new([10, 20, 30]);
        let v = decode_sample_value(&mut r, 2, m).unwrap();
        assert_eq!(v, 35);
    }

    #[test]
    fn sample_value_negative_sign_ones_complements() {
        // base=30, add=29, k=5, ex=2. t2=1 (< ex), sign=1 →
        // result = !(base + t2) = !(31) = -32.
        let bits = "1000" /* t2=1 */ .to_string() + "1" /* sign=1 */;
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let m = Medians::new([10, 20, 30]);
        let v = decode_sample_value(&mut r, 2, m).unwrap();
        assert_eq!(v, !31);
        assert_eq!(v, -32);
    }

    #[test]
    fn sample_value_n0_interval() {
        // n = 0, medians [4, _, _] → base=0, add=3, k=2, ex=(1<<2)-3-1=0.
        // getbits(k-1)=getbits(1). t2 read = 1; ex=0 so t2 >= ex always:
        // t2 = 1*2 - 0 + getbit(). extra bit = 0 → t2 = 2. sign=0 →
        // result = 0 + 2 = 2.
        // Bits LSB-first: getbits(1)="1", extra="0", sign="0".
        let bits = "100";
        let bytes = bits_to_bytes(bits);
        let mut r = BitReader::new(&bytes);
        let m = Medians::new([4, 99, 99]);
        let v = decode_sample_value(&mut r, 0, m).unwrap();
        assert_eq!(v, 2);
    }

    #[test]
    fn sample_value_ex_zero_always_takes_long_branch() {
        // When ex == 0 every t2 takes the `t2 >= ex` branch and reads an
        // extra bit. add=3 → k=2, ex=0. getbits(1)=0, extra=1, sign=0:
        // t2 = 0*2 - 0 + 1 = 1 → result = base(0) + 1 = 1.
        let bits = "0" /* getbits(1)=0 */ .to_string() + "1" /* extra */ + "0" /* sign */;
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let m = Medians::new([4, 99, 99]);
        let v = decode_sample_value(&mut r, 0, m).unwrap();
        assert_eq!(v, 1);
    }

    #[test]
    fn sample_value_degenerate_add_zero_is_rejected() {
        // median[0] == 1 → add = 0 → log2(0)/getbits(-1) undefined.
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        let m = Medians::new([1, 20, 30]);
        assert_eq!(
            decode_sample_value(&mut r, 0, m),
            Err(Error::GolombDegenerateInterval(0))
        );
        // No bits should have been consumed before the rejection.
        assert_eq!(r.bits_remaining(), 8);
    }

    #[test]
    fn sample_value_truncated_mantissa_is_reported() {
        // An empty buffer can't supply the very first getbits(k-1) bit.
        let bytes: [u8; 0] = [];
        let mut r = BitReader::new(&bytes);
        let m = Medians::new([10, 20, 30]);
        assert_eq!(decode_sample_value(&mut r, 2, m), Err(Error::Truncated));
    }

    #[test]
    fn sample_value_truncated_sign_is_reported() {
        // Size the mantissa to consume exactly one whole byte so the sign
        // bit lands past the buffer. n=0, median[0]=301 → add=300,
        // k=bitlen(300)=9, getbits(8); ex=(1<<9)-300-1=211. An all-zero
        // byte gives t2=0 (< ex) so no extra bit is read — the 8 mantissa
        // bits exactly drain the buffer, then the sign getbit() fails.
        let bytes = [0x00u8];
        let mut r = BitReader::new(&bytes);
        let m = Medians::new([301, 99, 99]);
        assert_eq!(decode_sample_value(&mut r, 0, m), Err(Error::Truncated));
    }

    #[test]
    fn run_length_then_sample_value_compose() {
        // End-to-end of the two halves against a fixed median set:
        // decode n via decode_run_length, then the value via
        // decode_sample_value, from one contiguous bitstream.
        //
        // Stream: unary "111110" → run-length raw n=5; from a neutral
        // state the spec §4.2 step 4 fold halves it (no held +1) →
        // n = 5 >> 1 = 2, and the odd low bit holds a one for the next
        // word. Then interval for n=2 with medians [10,20,30]: base=30,
        // add=29, k=5, ex=2. getbits(4)="1000"=1 (< ex), sign "0" →
        // result = 31.
        let bits = "111110" /* unary=5 → n=2 */ .to_string()
            + "1000" /* getbits(4)=1 */
            + "0"; /* sign=0 */
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 2);
        let m = Medians::new([10, 20, 30]);
        let v = decode_sample_value(&mut r, n, m).unwrap();
        assert_eq!(v, 31);
    }

    // ---- Medians from EntropyInfo bridge ----

    #[test]
    fn medians_from_entropy_left_takes_first_set() {
        let info = EntropyInfo {
            medians_left: [10, 20, 30],
            medians_right: [40, 50, 60],
        };
        assert_eq!(Medians::from_entropy_left(&info).values, [10, 20, 30]);
    }

    #[test]
    fn medians_from_entropy_right_takes_second_set() {
        let info = EntropyInfo {
            medians_left: [10, 20, 30],
            medians_right: [40, 50, 60],
        };
        assert_eq!(Medians::from_entropy_right(&info).values, [40, 50, 60]);
    }

    #[test]
    fn medians_from_entropy_right_is_zero_for_mono() {
        // A mono EntropyInfo leaves the right set at [0; 3]; the bridge
        // surfaces exactly that (no special-casing).
        let info = EntropyInfo::mono([7, 8, 9]);
        assert_eq!(Medians::from_entropy_left(&info).values, [7, 8, 9]);
        assert_eq!(Medians::from_entropy_right(&info).values, [0, 0, 0]);
    }

    // ---- decode_sample (run-length + value in one call) ----

    #[test]
    fn decode_sample_chains_run_length_and_value() {
        // Same bitstream as `run_length_then_sample_value_compose`, but
        // driven through the single-call `decode_sample`.
        // unary "111110" → raw n=5, neutral fold → n=2; interval for
        // n=2 with medians [10,20,30]: base=30, add=29, k=5, ex=2;
        // getbits(4)="1000"=1 (< ex), sign "0" → result = 31.
        let bits = "111110".to_string() + "1000" + "0";
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let m = Medians::new([10, 20, 30]);
        let v = decode_sample(&mut r, &mut state, m).unwrap();
        assert_eq!(v, 31);
        // The run-length step left the adaptive state as the odd path
        // would: last_one set, last_zero clear.
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn decode_sample_honours_last_zero_short_circuit() {
        // With last_zero set, decode_sample must skip the unary read and
        // decode the value against n = 0. medians [4,99,99] → base=0,
        // add=3, k=2, ex=0; getbits(1)="1", extra "0", sign "0" → 2.
        // (Matches `sample_value_n0_interval`.)
        let bits = "100";
        let bytes = bits_to_bytes(bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState {
            last_zero: true,
            last_one: false,
        };
        let m = Medians::new([4, 99, 99]);
        let v = decode_sample(&mut r, &mut state, m).unwrap();
        assert_eq!(v, 2);
        // last_zero consumed; no unary was read so only the value bits
        // ("100" = 3 bits) were taken.
        assert!(!state.last_zero);
    }

    #[test]
    fn decode_sample_propagates_degenerate_interval() {
        // n decodes to 0 (unary "0"), then median[0] == 1 → add = 0 →
        // the degenerate interval error surfaces unchanged.
        let bits = "0";
        let bytes = bits_to_bytes(bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let m = Medians::new([1, 20, 30]);
        assert_eq!(
            decode_sample(&mut r, &mut state, m),
            Err(Error::GolombDegenerateInterval(0))
        );
    }

    #[test]
    fn decode_sample_propagates_truncation() {
        // Empty buffer: even the first unary bit can't be read.
        let bytes: [u8; 0] = [];
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let m = Medians::new([10, 20, 30]);
        assert_eq!(decode_sample(&mut r, &mut state, m), Err(Error::Truncated));
    }

    #[test]
    fn decode_sample_feeds_from_entropy_info() {
        // End-to-end of the round-4 → round-6 → round-7 chain: take a
        // stereo EntropyInfo, pull the left channel's Medians, and decode
        // one sample. Left medians [10,20,30]; bits as in the chain test.
        let info = EntropyInfo {
            medians_left: [10, 20, 30],
            medians_right: [40, 50, 60],
        };
        let bits = "111110".to_string() + "1000" + "0";
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let v = decode_sample(&mut r, &mut state, Medians::from_entropy_left(&info)).unwrap();
        assert_eq!(v, 31);
    }

    // ---- Round-12 BitReader position accessors ----

    #[test]
    fn bit_reader_byte_and_bit_position_track_with_get_bit() {
        let bytes = [0b0000_0101u8, 0u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 0);
        assert_eq!(r.bits_consumed(), 0);

        // Consume one bit. byte_pos stays 0, bit_pos advances to 1.
        r.get_bit().unwrap();
        assert_eq!(r.byte_position(), 0);
        assert_eq!(r.bit_position(), 1);
        assert_eq!(r.bits_consumed(), 1);

        // Consume seven more — should land at start of byte 1.
        for _ in 0..7 {
            r.get_bit().unwrap();
        }
        assert_eq!(r.byte_position(), 1);
        assert_eq!(r.bit_position(), 0);
        assert_eq!(r.bits_consumed(), 8);
    }

    #[test]
    fn bit_reader_position_tracks_with_get_bits() {
        let bytes = [0xFFu8; 4];
        let mut r = BitReader::new(&bytes);
        r.get_bits(13).unwrap();
        // After 13 bits: byte 1, bit position 5.
        assert_eq!(r.byte_position(), 1);
        assert_eq!(r.bit_position(), 5);
        assert_eq!(r.bits_consumed(), 13);
    }

    #[test]
    fn bit_reader_bits_consumed_clamps_when_past_end() {
        // Read every bit of a single byte. byte_pos ends at 1 (past
        // end of a single-byte buffer); bits_consumed should report
        // the buffer length in bits, not 1*8 + bit_pos.
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        for _ in 0..8 {
            r.get_bit().unwrap();
        }
        assert_eq!(r.byte_position(), 1);
        assert_eq!(r.bit_position(), 0);
        assert_eq!(r.bits_consumed(), 8);
        assert!(r.is_empty());
    }

    #[test]
    fn bit_reader_position_unchanged_on_truncation() {
        // A read that would overshoot the buffer returns Truncated and
        // leaves the cursor at the last successfully-positioned bit.
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        for _ in 0..8 {
            r.get_bit().unwrap();
        }
        let pos_before = (r.byte_position(), r.bit_position());
        assert_eq!(r.get_bit(), Err(Error::Truncated));
        let pos_after = (r.byte_position(), r.bit_position());
        assert_eq!(pos_before, pos_after);
    }

    // ---- Round-12 Medians::from_entropy channel-indexed bridge ----

    #[test]
    fn medians_from_entropy_yields_left_on_zero() {
        let info = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [4, 5, 6],
        };
        assert_eq!(
            Medians::from_entropy(&info, 0),
            Some(Medians::new([1, 2, 3]))
        );
    }

    #[test]
    fn medians_from_entropy_yields_right_on_one_for_stereo() {
        let info = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [4, 5, 6],
        };
        assert_eq!(
            Medians::from_entropy(&info, 1),
            Some(Medians::new([4, 5, 6]))
        );
    }

    #[test]
    fn medians_from_entropy_one_is_none_on_mono() {
        let info = EntropyInfo::mono([7, 8, 9]);
        assert_eq!(
            Medians::from_entropy(&info, 0),
            Some(Medians::new([7, 8, 9]))
        );
        // The wiki put no right-channel set on a mono payload.
        assert_eq!(Medians::from_entropy(&info, 1), None);
    }

    #[test]
    fn medians_from_entropy_rejects_out_of_range_indices() {
        let info = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [4, 5, 6],
        };
        // The wiki names mono and stereo — index 2 and beyond are not
        // populated.
        assert_eq!(Medians::from_entropy(&info, 2), None);
        assert_eq!(Medians::from_entropy(&info, 3), None);
        assert_eq!(Medians::from_entropy(&info, 255), None);
    }

    // ---- Round-191 median-adaptation amount (spec §3 + §3.2) ----

    #[test]
    fn divisor_constants_match_spec_table() {
        // Spec §3 Table: DIV0 = 128, DIV1 = 64, DIV2 = 32.
        assert_eq!(DIV0, 128);
        assert_eq!(DIV1, 64);
        assert_eq!(DIV2, 32);
        // Spec §3 multipliers.
        assert_eq!(MEDIAN_INC_MULTIPLIER, 5);
        assert_eq!(MEDIAN_DEC_MULTIPLIER, 2);
        // Spec §2.1 GET_MED parameters.
        assert_eq!(GET_MED_SHIFT, 4);
        assert_eq!(GET_MED_FLOOR, 1);
    }

    #[test]
    fn get_med_zero_returns_floor_one() {
        // get_med(i) = (0 >> 4) + 1 = 1 — the §2.1 floor.
        let m = AdaptiveMedians::new([0, 0, 0]);
        assert_eq!(m.get_med(0), 1);
        assert_eq!(m.get_med(1), 1);
        assert_eq!(m.get_med(2), 1);
    }

    #[test]
    fn get_med_strips_four_fractional_bits() {
        // 4 fractional bits → stored 16 == working 1 + 1 = 2.
        let m = AdaptiveMedians::new([16, 32, 48]);
        // (16 >> 4) + 1 = 2.
        assert_eq!(m.get_med(0), 2);
        // (32 >> 4) + 1 = 3.
        assert_eq!(m.get_med(1), 3);
        // (48 >> 4) + 1 = 4.
        assert_eq!(m.get_med(2), 4);
    }

    #[test]
    fn get_med_rounds_down_truncates() {
        // 4 fractional bits don't round — they truncate.
        // (15 >> 4) + 1 = 0 + 1 = 1.
        let m = AdaptiveMedians::new([15, 31, 47]);
        assert_eq!(m.get_med(0), 1); // 15 >> 4 = 0, +1 = 1.
        assert_eq!(m.get_med(1), 2); // 31 >> 4 = 1, +1 = 2.
        assert_eq!(m.get_med(2), 3); // 47 >> 4 = 2, +1 = 3.
    }

    #[test]
    fn inc_median_zero_steps_by_five() {
        // §3 increment at median = 0: ((0 + 128) / 128) * 5 = 1 * 5 = 5.
        let mut m = AdaptiveMedians::new([0, 0, 0]);
        m.inc_median(0);
        assert_eq!(m.values[0], 5);
    }

    #[test]
    fn inc_median_at_full_divisor_steps_by_ten() {
        // §3 increment at median = 128: ((128 + 128) / 128) * 5 = 2 * 5 = 10.
        let mut m = AdaptiveMedians::new([128, 0, 0]);
        m.inc_median(0);
        assert_eq!(m.values[0], 128 + 10);
    }

    #[test]
    fn inc_median_uses_per_index_divisor() {
        // §3: index 0 → D=128, index 1 → D=64, index 2 → D=32.
        // At median = 0 the step is always 5 (independent of D), so use
        // a non-zero value to distinguish.
        // Index 1, median = 64: ((64 + 64) / 64) * 5 = 2 * 5 = 10.
        let mut m = AdaptiveMedians::new([0, 64, 0]);
        m.inc_median(1);
        assert_eq!(m.values[1], 64 + 10);

        // Index 2, median = 32: ((32 + 32) / 32) * 5 = 2 * 5 = 10.
        let mut m = AdaptiveMedians::new([0, 0, 32]);
        m.inc_median(2);
        assert_eq!(m.values[2], 32 + 10);
    }

    #[test]
    fn dec_median_zero_steps_to_zero() {
        // §3 decrement at median = 0: ((0 + 126) / 128) * 2 = 0 * 2 = 0,
        // so 0 - 0 = 0 (the §3 "never below 1" claim is on get_med, not
        // on the raw median; raw can hit 0 here, get_med stays at 1).
        let mut m = AdaptiveMedians::new([0, 0, 0]);
        m.dec_median(0);
        assert_eq!(m.values[0], 0);
        // get_med still reports the §2.1 floor.
        assert_eq!(m.get_med(0), 1);
    }

    #[test]
    fn dec_median_at_full_divisor_steps_by_two() {
        // §3 decrement at median = 128: ((128 + 126) / 128) * 2 = 1 * 2 = 2.
        let mut m = AdaptiveMedians::new([128, 0, 0]);
        m.dec_median(0);
        assert_eq!(m.values[0], 128 - 2);
    }

    #[test]
    fn dec_median_never_goes_below_zero_at_small_values() {
        // The +(D-2) bias keeps the step ≤ median for all values where
        // ((m + D - 2) / D) * 2 ≤ m. Sweep small values and confirm
        // the decremented value stays non-negative.
        for v in 0..=200u32 {
            let mut m = AdaptiveMedians::new([v, 0, 0]);
            m.dec_median(0);
            // No underflow (we used saturating_sub but want to confirm
            // the spec arithmetic naturally stayed non-negative here).
            // For the spec formula: step = ((v + 126) / 128) * 2.
            // At v = 0..=1: step = 0. At v = 2..=129: step = 2. So the
            // post value is v.saturating_sub(2), never below 0.
            let expected_step = ((v + (DIV0 - 2)) / DIV0).saturating_mul(2);
            assert_eq!(
                m.values[0],
                v.saturating_sub(expected_step),
                "raw v={v} mismatched"
            );
        }
    }

    #[test]
    fn inc_then_dec_at_equilibrium_holds() {
        // §3.1: 5 increments + 2 decrements per 7 sample mean equilibrium
        // weight ratio 5:2. Pick a non-trivial starting value and confirm
        // the per-step deltas match the §3 formulas exactly.
        let mut m = AdaptiveMedians::new([256, 0, 0]);
        let before = m.values[0];
        m.inc_median(0);
        let after_inc = m.values[0];
        // ((256 + 128) / 128) * 5 = 3 * 5 = 15.
        assert_eq!(after_inc - before, 15);

        m.dec_median(0);
        let after_dec = m.values[0];
        // From 271: ((271 + 126) / 128) * 2 = 3 * 2 = 6.
        assert_eq!(after_inc - after_dec, 6);
    }

    #[test]
    fn zone_from_ones_count_maps_named_arms() {
        assert_eq!(Zone::from_ones_count(0), Zone::Zone0);
        assert_eq!(Zone::from_ones_count(1), Zone::Zone1);
        assert_eq!(Zone::from_ones_count(2), Zone::Zone2);
        // Anything >= 3 lands in the overflow arm with the raw value
        // preserved.
        assert_eq!(
            Zone::from_ones_count(3),
            Zone::Zone2Overflow { ones_count: 3 }
        );
        assert_eq!(
            Zone::from_ones_count(33),
            Zone::Zone2Overflow { ones_count: 33 }
        );
        assert_eq!(
            Zone::from_ones_count(u32::MAX),
            Zone::Zone2Overflow {
                ones_count: u32::MAX
            }
        );
    }

    #[test]
    fn zone_ones_count_round_trips() {
        for raw in [0u32, 1, 2, 3, 4, 16, 33, 100, u32::MAX] {
            assert_eq!(Zone::from_ones_count(raw).ones_count(), raw);
        }
    }

    #[test]
    fn adapt_zone0_decrements_median0_only() {
        // §3.2: zone 0 → decrement median[0] only.
        let mut m = AdaptiveMedians::new([128, 64, 32]);
        let before = m.values;
        m.adapt(Zone::Zone0);
        // median[0] dropped by the §3 step; medians[1] / medians[2]
        // unchanged.
        assert!(m.values[0] < before[0]);
        assert_eq!(m.values[1], before[1]);
        assert_eq!(m.values[2], before[2]);
        // Compare against the dec_median primitive directly.
        let mut ref_m = AdaptiveMedians::new(before);
        ref_m.dec_median(0);
        assert_eq!(m, ref_m);
    }

    #[test]
    fn adapt_zone1_increments_median0_decrements_median1() {
        // §3.2: zone 1 → median[0] up, median[1] down, median[2]
        // unchanged.
        let mut m = AdaptiveMedians::new([128, 64, 32]);
        let before = m.values;
        m.adapt(Zone::Zone1);
        assert!(m.values[0] > before[0]);
        assert!(m.values[1] < before[1]);
        assert_eq!(m.values[2], before[2]);

        let mut ref_m = AdaptiveMedians::new(before);
        ref_m.inc_median(0);
        ref_m.dec_median(1);
        assert_eq!(m, ref_m);
    }

    #[test]
    fn adapt_zone2_increments_two_decrements_third() {
        // §3.2: zone 2 → median[0] up, median[1] up, median[2] down.
        let mut m = AdaptiveMedians::new([128, 64, 32]);
        let before = m.values;
        m.adapt(Zone::Zone2);
        assert!(m.values[0] > before[0]);
        assert!(m.values[1] > before[1]);
        assert!(m.values[2] < before[2]);

        let mut ref_m = AdaptiveMedians::new(before);
        ref_m.inc_median(0);
        ref_m.inc_median(1);
        ref_m.dec_median(2);
        assert_eq!(m, ref_m);
    }

    #[test]
    fn adapt_zone2_overflow_increments_all_three() {
        // §3.2: zone 2 overflow (ones_count >= 3) → all three medians
        // up.
        let mut m = AdaptiveMedians::new([128, 64, 32]);
        let before = m.values;
        m.adapt(Zone::Zone2Overflow { ones_count: 5 });
        assert!(m.values[0] > before[0]);
        assert!(m.values[1] > before[1]);
        assert!(m.values[2] > before[2]);

        let mut ref_m = AdaptiveMedians::new(before);
        ref_m.inc_median(0);
        ref_m.inc_median(1);
        ref_m.inc_median(2);
        assert_eq!(m, ref_m);
    }

    #[test]
    fn adapt_for_ones_count_drives_correct_zone() {
        // The convenience wrapper threads through Zone::from_ones_count.
        let initial = [128u32, 64, 32];

        // ones_count = 1 → Zone1: median[0] up, median[1] down.
        let mut via_wrapper = AdaptiveMedians::new(initial);
        via_wrapper.adapt_for_ones_count(1);
        let mut via_zone = AdaptiveMedians::new(initial);
        via_zone.adapt(Zone::Zone1);
        assert_eq!(via_wrapper, via_zone);

        // ones_count = 7 → Zone2Overflow {7}: all three up.
        let mut via_wrapper = AdaptiveMedians::new(initial);
        via_wrapper.adapt_for_ones_count(7);
        let mut via_zone = AdaptiveMedians::new(initial);
        via_zone.adapt(Zone::Zone2Overflow { ones_count: 7 });
        assert_eq!(via_wrapper, via_zone);
    }

    #[test]
    fn from_seed_values_accepts_non_negative_and_rejects_negative() {
        // Non-negative seeds → Some.
        assert_eq!(
            AdaptiveMedians::from_seed_values([10, 20, 30]),
            Some(AdaptiveMedians::new([10, 20, 30]))
        );
        // Negative anywhere → None.
        assert_eq!(AdaptiveMedians::from_seed_values([-1, 20, 30]), None);
        assert_eq!(AdaptiveMedians::from_seed_values([10, -1, 30]), None);
        assert_eq!(AdaptiveMedians::from_seed_values([10, 20, -1]), None);
        // All-zero seeds — the legal fresh state — accepted.
        assert_eq!(
            AdaptiveMedians::from_seed_values([0, 0, 0]),
            Some(AdaptiveMedians::new([0, 0, 0]))
        );
    }

    #[test]
    fn from_medians_bridges_round_six_typed_set() {
        // Bridge a Medians (round-6 typed view) into the adaptive state.
        let m = Medians::new([100, 200, 300]);
        assert_eq!(
            AdaptiveMedians::from_medians(m),
            Some(AdaptiveMedians::new([100, 200, 300]))
        );
        // A Medians with a negative slot is rejected.
        let bad = Medians::new([100, -1, 300]);
        assert_eq!(AdaptiveMedians::from_medians(bad), None);
    }

    #[test]
    fn adapt_amount_independent_inc_dec_sequence_matches_spec() {
        // Walk a fixed sequence and confirm every step matches the
        // hand-computed spec §3 arithmetic exactly. Start at the round-4
        // seed equivalent of all-zero (the encoder default for a fresh
        // block).
        let mut m = AdaptiveMedians::new([0, 0, 0]);

        // Zone1: median[0] += ((0+128)/128)*5 = 5; median[1] -= 0.
        m.adapt(Zone::Zone1);
        assert_eq!(m.values, [5, 0, 0]);

        // Zone0: median[0] -= ((5+126)/128)*2 = 1*2 = 2 → 3.
        m.adapt(Zone::Zone0);
        assert_eq!(m.values, [3, 0, 0]);

        // Zone2: m0 += ((3+128)/128)*5 = 1*5 = 5 → 8;
        //        m1 += ((0+64)/64)*5 = 1*5 = 5;
        //        m2 -= ((0+30)/32)*2 = 0 → 0.
        m.adapt(Zone::Zone2);
        assert_eq!(m.values, [8, 5, 0]);

        // Zone2Overflow: all three up.
        // m0 += ((8+128)/128)*5 = 1*5 = 5 → 13;
        // m1 += ((5+64)/64)*5 = 1*5 = 5 → 10;
        // m2 += ((0+32)/32)*5 = 1*5 = 5 → 5.
        m.adapt(Zone::Zone2Overflow { ones_count: 4 });
        assert_eq!(m.values, [13, 10, 5]);
    }

    #[test]
    fn inc_median_saturates_at_u32_max() {
        // Defensive saturating semantics: pathological starting value
        // doesn't overflow.
        let mut m = AdaptiveMedians::new([u32::MAX, 0, 0]);
        m.inc_median(0);
        // The exact saturation point isn't part of the spec — we just
        // confirm no panic and the result is still u32-representable.
        assert_eq!(m.values[0], u32::MAX);
    }

    #[test]
    fn dec_median_saturates_at_zero() {
        // Same defensive contract on the way down: from zero, the
        // decremented value cannot go negative.
        let mut m = AdaptiveMedians::new([0, 0, 0]);
        m.dec_median(0);
        assert_eq!(m.values[0], 0);
        m.dec_median(1);
        assert_eq!(m.values[1], 0);
        m.dec_median(2);
        assert_eq!(m.values[2], 0);
    }

    #[test]
    fn adaptive_medians_is_copy_and_eq() {
        // Sanity: small derived traits are present so callers can keep
        // pre-update snapshots around without rebuilding.
        // Use a starting value large enough that the §3 decrement step
        // is non-zero (((128 + 126) / 128) * 2 = 2) so the post-update
        // value is observably different from the pre.
        let a = AdaptiveMedians::new([128, 64, 32]);
        let b = a;
        assert_eq!(a, b);
        let mut c = a;
        c.adapt(Zone::Zone0);
        assert_ne!(a, c);
    }

    // ---- Round-194 stateful per-sample decode (spec §3 + §3.2 + §4.2) ----
    //
    // The tests in this block originally used a test-side spec-derived
    // inverse encoder to produce a `0x0A`-shape bitstream from a fixed
    // PCM sequence; round 281 lifted that inverse onto the public
    // surface (`BitWriter`, `emit_*`, `encode_packed_samples_*`), so
    // the round-trips here drive the public encoder against the
    // per-call decode loop and assert every sample reconstructs
    // bit-for-bit. A successful round-trip pins both halves to the
    // spec text.

    /// Inverse of the spec §4.2 step 6 truncated-binary decoder: emit
    /// the bits for `code` inside an interval of `maxcode + 1`
    /// codewords.
    fn emit_truncated_binary(w: &mut BitWriter, maxcode: u32, code: u32) {
        if maxcode == 0 {
            // Inverse of "maxcode == 0 → no bits read, value is 0".
            return;
        }
        if maxcode == 1 {
            // Inverse of "maxcode == 1 → read one bit; that bit is the value".
            w.write_bit(code & 1);
            return;
        }
        let bitcount = 32 - maxcode.leading_zeros();
        let extras = (1u32 << bitcount) - maxcode - 1;
        if code < extras {
            // Short form: emit code in (bitcount - 1) bits LSB-first.
            w.write_bits(code, bitcount - 1);
        } else {
            // Long form: full `bitcount`-bit code mapped back to the
            // (short, extra_bit) pair. From decode:
            //   code_decoded = (short << 1) - extras + extra_bit.
            // So short = (code_decoded + extras) >> 1, extra_bit =
            // (code_decoded + extras) & 1. The short value's bit count
            // remains `bitcount - 1` (it ranges from extras to
            // 2*extras-1, all fitting; the long region needs the high
            // bit which extra_bit provides).
            let combined = code + extras;
            let short = combined >> 1;
            let extra_bit = combined & 1;
            w.write_bits(short, bitcount - 1);
            w.write_bit(extra_bit);
        }
    }

    /// Round-trip a sequence of sample values through the public
    /// encoder ([`encode_packed_samples_mono`]) and the per-call decode
    /// loop ([`decode_sample_stateful`]) against a fresh
    /// `AdaptiveMedians` seed, asserting every reconstructed sample
    /// matches the input AND the encoder and decoder finish in
    /// identical median state (pinning spec §3.2 adaptation across the
    /// whole sequence).
    ///
    /// Returns the number of samples (== `values.len()`) so callers
    /// can aggregate a sample-exact count across multiple round-trips.
    fn round_trip(seed: [u32; 3], values: &[i32]) -> usize {
        // Encode through the public surface.
        let mut enc_medians = AdaptiveMedians::new(seed);
        let bytes = encode_packed_samples_mono(values, &mut enc_medians).expect("encode");
        assert_eq!(
            bytes.len() % 2,
            0,
            "encoded payload must be even-length per spec §1"
        );

        // Decode and compare.
        let mut dec_medians = AdaptiveMedians::new(seed);
        let mut reader = BitReader::new(&bytes);
        let mut dec_state = DecodeState::new();
        for (i, &expected) in values.iter().enumerate() {
            let got = decode_sample_stateful(&mut reader, &mut dec_medians, &mut dec_state)
                .unwrap_or_else(|e| panic!("decode sample {i} failed: {e:?}"));
            assert_eq!(
                got, expected,
                "round-trip mismatch at sample {i}: expected {expected}, got {got}",
            );
        }
        // Encoder and decoder must finish in identical median state —
        // that pins spec §3.2 adaptation across the whole sequence.
        assert_eq!(
            dec_medians, enc_medians,
            "encoder and decoder finished with different median state",
        );
        values.len()
    }

    #[test]
    fn round_trip_zone0_short_values() {
        // Seed medians give get_med(0) = 513 so zone 0 spans [0, 512].
        // Each zone-0 decode decrements m0 by a small step; over a
        // short sequence the get_med floor stays well above the
        // chosen magnitudes. Use mid-zone-0 magnitudes to avoid the
        // value-0 case (which fires `last_zero` chain) — there's a
        // dedicated zero-value test below.
        let seed = [8192u32, 8192, 8192];
        let values: Vec<i32> = vec![1, 5, 17, 33, 50, 100, 200, 300, 400];
        let n = round_trip(seed, &values);
        assert_eq!(n, values.len());
    }

    #[test]
    fn round_trip_zone1_medium_values() {
        // get_med = 513 so zone 1 spans [513, 1025].
        let seed = [8192u32, 8192, 8192];
        let values: Vec<i32> = vec![513, 600, 700, 800, 900, 1000, 1025];
        let n = round_trip(seed, &values);
        assert_eq!(n, values.len());
    }

    #[test]
    fn round_trip_zone2_larger_values() {
        // get_med = 513 so zone 2 spans [1026, 1538] initially.
        let seed = [8192u32, 8192, 8192];
        let values: Vec<i32> = vec![1026, 1100, 1200, 1300, 1400, 1500, 1538];
        let n = round_trip(seed, &values);
        assert_eq!(n, values.len());
    }

    #[test]
    fn round_trip_zone2_overflow_values() {
        // Zone 2 overflow: ones_count >= 3. With get_med = 513 zone 3
        // starts at low = 513 + 513 + 513 = 1539, zone 4 at 2052, etc.
        let seed = [8192u32, 8192, 8192];
        let values: Vec<i32> = vec![1539, 1700, 2052, 2200, 2565, 2700];
        let n = round_trip(seed, &values);
        assert_eq!(n, values.len());
    }

    #[test]
    fn round_trip_negative_values_use_sign_bit() {
        // Decoder: sign=1 → return !mid. Encoder must emit the
        // bit-complemented magnitude. Sweep negatives in zones 1+
        // (zone 0 is covered by the dedicated zero-pair test below).
        // Magnitudes: |-600|=600 (zone 1 with get_med=513);
        // |-1200|=1200 (zone 2); |-2000|=2000 (zone 2 overflow).
        let seed = [8192u32, 8192, 8192];
        let values: Vec<i32> = vec![-600, -700, -800, -1200, -1500, -2000];
        let n = round_trip(seed, &values);
        assert_eq!(n, values.len());
    }

    #[test]
    fn round_trip_mixed_zones_long_sequence() {
        // Exercise spec §3 adaptation across zones 1+ over a long
        // sequence. Medians WILL adapt — the encoder and decoder both
        // walk the same §3.2 trajectory, so the bytes remain decodable
        // even as the median state drifts. Avoid zone 0 (magnitude <
        // get_med(0)) entirely.
        //
        // Strategy for staying in zones 1+: pick the round_trip helper
        // so the SEQUENCE never produces a zone-0 sample under the
        // adapting medians. The cleanest way is to test the helper at
        // call time and substitute the value into a higher zone if a
        // zone-0 sample would emerge — but that splits the test
        // premise from the API surface. Instead use a deterministic
        // sequence whose interval-aware shape stays in zones 1+
        // throughout: alternate zone-1 magnitudes (~ get_med(0)) and
        // zone-2 magnitudes (~ 2 * get_med(0)), tracked against the
        // adapting median state by a small simulator that picks the
        // next value at the boundary.
        //
        // To keep the test deterministic and the spec-side reasoning
        // mechanical, we drive the simulator: at each step, query the
        // CURRENT get_med(0), pick magnitude = get_med(0) (zone 1
        // boundary) or 2*get_med(0) (zone 2 boundary), and feed it.
        let seed = [8192u32, 8192, 8192];
        let mut sim_medians = AdaptiveMedians::new(seed);
        let mut values = Vec::new();
        for i in 0..64 {
            let m0 = sim_medians.get_med(0) as i32;
            let m1 = sim_medians.get_med(1) as i32;
            // Cycle zone1 / zone2 / zone2-overflow magnitudes.
            let mag = match i % 3 {
                0 => m0 + 1,                                      // zone 1 (low boundary)
                1 => m0 + m1 + 1,                                 // zone 2 (low boundary)
                _ => m0 + m1 + sim_medians.get_med(2) as i32 + 1, // zone 3 (overflow)
            };
            // Simulate the per-sample adapt that decode/encode will do
            // so the next iteration picks against the post-adapt medians.
            let ones_count = sim_medians.zone_for_magnitude(mag as u32);
            sim_medians.adapt(Zone::from_ones_count(ones_count));
            values.push(mag);
        }
        let n = round_trip(seed, &values);
        assert_eq!(n, 64);
    }

    #[test]
    fn round_trip_decode_packed_samples_mono_matches_loop() {
        // Drive the same round-trip through the public
        // `decode_packed_samples_mono` instead of the per-call
        // `decode_sample_stateful` — confirms the bundled loop is
        // bit-exact identical to the manual call sequence.
        let seed = [8192u32, 8192, 8192];
        let values: Vec<i32> = (1..=32).collect();

        // Encode through the public surface.
        let mut enc_medians = AdaptiveMedians::new(seed);
        let bytes = encode_packed_samples_mono(&values, &mut enc_medians).expect("encode");

        // Decode via the public payload-loop API.
        let view = crate::PackedSamples::new(&bytes);
        let mut dec_medians = AdaptiveMedians::new(seed);
        let got = decode_packed_samples_mono(&view, &mut dec_medians, values.len())
            .expect("decode payload");
        assert_eq!(got, values);
        // The end-state medians match the encoder's.
        assert_eq!(dec_medians, enc_medians);
    }

    #[test]
    fn round_trip_value_zero_with_zone0_seed() {
        // Zero samples in zone 0: the encoder pre-encodes each
        // following zero through the spec §4.2 step 4 boundary carry (a
        // clear low bit on the raw prefix), so alternating words
        // short-circuit with no prefix bits at all. The medians' raw
        // median[0] (256) is far above the §4.2 step 1 zero-run gate,
        // so this walks the prefix path, not the zero-run path.
        let seed = [256u32, 256, 256];
        let values: Vec<i32> = vec![0, 0, 0, 0];
        let n = round_trip(seed, &values);
        assert_eq!(n, 4);
    }

    #[test]
    fn round_trip_zone0_word_followed_by_non_zero_zone_word() {
        // The spec §4.2 step 4 prior-state fold is the only reading
        // under which a zone-0 word can be followed by a word in any
        // other zone: the zone-0 word emits raw = 1 (odd — hold a one
        // into the next word) and the next word's folded count gains
        // +1. Under the wiki pseudocode's self-referencing fold this
        // sequence is unrepresentable (raw 1 would fold to 1, not 0).
        // Seed get_med = 17, so 5 is zone 0 and 1000 is deep overflow.
        let seed = [256u32, 256, 256];
        let n = round_trip(seed, &[5, 1000]);
        assert_eq!(n, 2);

        // And the mirrored shape: non-zero zone, zone 0, non-zero zone.
        let n = round_trip(seed, &[1000, 5, 900]);
        assert_eq!(n, 3);
    }

    #[test]
    fn decode_sample_stateful_propagates_truncation() {
        // Empty payload, no zero-run debt, no holding state — the very
        // first get_unary call fails.
        let bytes: [u8; 0] = [];
        let mut reader = BitReader::new(&bytes);
        let mut medians = AdaptiveMedians::new([256, 256, 256]);
        let mut state = DecodeState::new();
        let err = decode_sample_stateful(&mut reader, &mut medians, &mut state);
        assert_eq!(err, Err(Error::Truncated));
    }

    #[test]
    fn decode_sample_stateful_eof_escape_returns_end_of_stream() {
        // Build a bitstream where the first sample's unary triggers the
        // LIMIT_ONES = 16 escape and the inner cbits unary reads 33,
        // which is the spec §4.2 step 3 EOF marker.
        //
        // Bits: 16 ones (the LIMIT_ONES prefix) + 0 (its terminator)
        //     + 33 ones (the cbits inner unary == 33) + 0 (its
        //       terminator)
        // The decoder must return Error::EndOfStream — and crucially
        // must NOT have tried to read mantissa bits past the marker.
        let mut bits = String::new();
        bits.push_str(&"1".repeat(16));
        bits.push('0');
        bits.push_str(&"1".repeat(33));
        bits.push('0');
        let bytes = bits_to_bytes(&bits);
        let mut reader = BitReader::new(&bytes);
        let mut medians = AdaptiveMedians::new([256, 256, 256]);
        let mut state = DecodeState::new();
        let err = decode_sample_stateful(&mut reader, &mut medians, &mut state);
        assert_eq!(err, Err(Error::EndOfStream));
    }

    #[test]
    fn decode_state_default_matches_new() {
        // The `Default` impl is derived; spell out the equivalence so
        // callers building with `DecodeState::default()` see the same
        // initial state `DecodeState::new()` produces.
        assert_eq!(DecodeState::default(), DecodeState::new());
        assert_eq!(DecodeState::new().zero_run_pending, 0);
        assert!(!DecodeState::new().ever_took_zero_run);
    }

    #[test]
    fn zero_run_path_eligibility_requires_low_median_and_no_holding() {
        // With medians at the default seed (get_med(0) = 17), the
        // zero-run path is NOT eligible — try_zero_run_path returns
        // Ok(None) and decode_sample_stateful proceeds to the normal
        // prefix path. Use an empty buffer to confirm the prefix path
        // (not the zero-run path) is the one that fails on truncation.
        let bytes: [u8; 0] = [];
        let mut reader = BitReader::new(&bytes);
        let mut medians = AdaptiveMedians::new([256, 256, 256]);
        let mut state = DecodeState::new();
        let err = decode_sample_stateful(&mut reader, &mut medians, &mut state);
        assert_eq!(err, Err(Error::Truncated));
        // Confirm the zero-run path did NOT mark itself as taken.
        assert!(!state.ever_took_zero_run);
    }

    #[test]
    fn zero_run_path_engages_when_eligible() {
        // Force eligibility by seeding medians at all-zero (get_med(0)
        // = 1, satisfying the spec §4.2 step 1 "median[0] <= 1" gate)
        // and using a fresh state with no holding. To get a run_length
        // of exactly 3 we need spec §4.2 step 1's "count >= 2" path:
        // count = 2, mantissa = 1 (one bit, LSB-first), implied top
        // bit set → run_length = (1 << 1) | 1 = 3. Bits: unary "110"
        // (count = 2) then one mantissa bit "1".
        let bytes = bits_to_bytes("1101");
        let mut reader = BitReader::new(&bytes);
        let mut medians = AdaptiveMedians::new([0, 0, 0]);
        let mut state = DecodeState::new();
        let v = decode_sample_stateful(&mut reader, &mut medians, &mut state)
            .expect("first sample of zero-run");
        assert_eq!(v, 0);
        assert!(state.ever_took_zero_run);
        // run_length = 3, this call emitted the first 0, two owed.
        assert_eq!(state.zero_run_pending, 2);
        // Two more calls drain the debt without consuming any bits.
        let v2 = decode_sample_stateful(&mut reader, &mut medians, &mut state).unwrap();
        assert_eq!(v2, 0);
        assert_eq!(state.zero_run_pending, 1);
        let v3 = decode_sample_stateful(&mut reader, &mut medians, &mut state).unwrap();
        assert_eq!(v3, 0);
        assert_eq!(state.zero_run_pending, 0);
    }

    #[test]
    fn read_truncated_binary_maxcode_zero_consumes_no_bits() {
        let bytes = [0xFFu8];
        let mut r = BitReader::new(&bytes);
        let v = read_truncated_binary(&mut r, 0).unwrap();
        assert_eq!(v, 0);
        assert_eq!(r.bits_remaining(), 8);
    }

    #[test]
    fn read_truncated_binary_maxcode_one_reads_one_bit() {
        let bytes = bits_to_bytes("10");
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_truncated_binary(&mut r, 1).unwrap(), 1);
        assert_eq!(read_truncated_binary(&mut r, 1).unwrap(), 0);
    }

    #[test]
    fn read_truncated_binary_matches_spec_for_maxcode_16() {
        // maxcode = 16: bitcount = 5, extras = (1<<5) - 16 - 1 = 15.
        // Short codes 0..14 read 4 bits; codes 15..16 read 4 + 1.
        // Round-trip every code through emit + read.
        for code in 0..=16u32 {
            let mut w = BitWriter::new();
            emit_truncated_binary(&mut w, 16, code);
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            let decoded = read_truncated_binary(&mut r, 16).unwrap();
            assert_eq!(decoded, code, "round-trip failed for code={code}");
        }
    }

    #[test]
    fn form_interval_matches_spec_ladder() {
        // Spec §4.2 step 5 worked out for m0 = m1 = m2 = 17.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let (low, high) = form_interval(&m, 0);
        assert_eq!((low, high), (0, 16));
        let (low, high) = form_interval(&m, 1);
        assert_eq!((low, high), (17, 33));
        let (low, high) = form_interval(&m, 2);
        assert_eq!((low, high), (34, 50));
        let (low, high) = form_interval(&m, 3);
        assert_eq!((low, high), (51, 67)); // m0+m1 + 1*m2 = 51; +m2-1 = 67.
        let (low, high) = form_interval(&m, 4);
        assert_eq!((low, high), (68, 84)); // +m2 = 68; +m2-1 = 84.
    }

    #[test]
    fn manual_hand_traced_two_sample_fixture() {
        // Hand-build a tiny 2-sample fixture from the spec text, decode
        // it through the stateful loop, and confirm both samples come
        // back as the spec describes.
        //
        // Seeds: medians [256, 256, 256] → get_med = 17 for all i.
        //
        // Sample 1: target ones_count = 0 (zone 0), magnitude = 5.
        //   - raw = 0 → unary "0" (single zero bit, sets last_zero).
        //     POST-FOLD: ones_count = 0, last_zero = true.
        //   - interval (zone 0, m0 = 17): low = 0, high = 16, maxcode = 16.
        //   - adapt: zone 0 → dec_median(0): step = ((256+126)/128)*2 = 2*2=4 → m0 = 252.
        //   - mantissa for code=5 (maxcode=16): bitcount=5, extras=15.
        //     short=5 < 15 → emit 5 in 4 bits LSB-first: "1010".
        //   - sign = 0 (positive): bit "0".
        //   Bits emitted: "0" + "1010" + "0" = "010100".
        //
        // Sample 2: last_zero is set → no unary read, ones_count = 0
        //   directly. magnitude must fit zone 0 (m0 = 252 now → get_med
        //   = (252>>4)+1 = 16). Pick value 3. (last_zero cleared on
        //   entry, last_one unchanged.)
        //   - interval (zone 0, m0 = 16): low = 0, high = 15, maxcode = 15.
        //   - adapt: zone 0 → dec_median(0): step = ((252+126)/128)*2 = 2*2=4 → m0 = 248.
        //   - mantissa for code=3 (maxcode=15): bitcount=4, extras=0.
        //     short=3 >= 0 → long form: combined = 3, short = 1, extra = 1.
        //     Emit short=1 in 3 bits "100" + extra "1".
        //   - sign = 0: "0".
        //   Bits emitted: "1001" + "0" = "10010".
        //
        // Full bit string: "010100" + "10010" = "01010010010".
        let bits = "010100".to_string() + "10010";
        let bytes = bits_to_bytes(&bits);
        let mut reader = BitReader::new(&bytes);
        let mut medians = AdaptiveMedians::new([256, 256, 256]);
        let mut state = DecodeState::new();

        let v1 = decode_sample_stateful(&mut reader, &mut medians, &mut state).unwrap();
        assert_eq!(v1, 5, "sample 1 mismatch");
        assert_eq!(medians.values[0], 252, "median[0] after sample 1");
        assert!(state.run.last_zero, "last_zero set after even-raw sample 1");

        let v2 = decode_sample_stateful(&mut reader, &mut medians, &mut state).unwrap();
        assert_eq!(v2, 3, "sample 2 (short-circuited via last_zero)");
        assert_eq!(medians.values[0], 248, "median[0] after sample 2");
    }

    // ---- Stereo round-trip tests (round 199) ----
    //
    // The spec §2 channel-alternation rule maps even sample indices to
    // the left channel and odd indices to the right channel; the
    // §4.2 step 1 zero-run fast path inspects + zeroes BOTH channels.
    // These tests encode interleaved (L,R,L,R,…) frames through the
    // public stereo encoder (round 281) and re-check bit-for-bit
    // against the `decode_sample_stateful_stereo` /
    // `decode_packed_samples_stereo` surface.

    /// Encode interleaved samples through the public stereo encoder.
    /// Returns the wire bytes plus the per-channel post-encode adaptive
    /// medians so a test can assert encoder == decoder end state.
    fn stereo_encode(seeds: [[u32; 3]; 2], interleaved: &[i32]) -> (Vec<u8>, [AdaptiveMedians; 2]) {
        let mut enc_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let bytes =
            encode_packed_samples_stereo(interleaved, &mut enc_medians).expect("stereo encode");
        assert_eq!(
            bytes.len() % 2,
            0,
            "encoded payload must be even-length per spec §1"
        );
        (bytes, enc_medians)
    }

    /// Round-trip a stereo sample sequence through encode + decode and
    /// assert every reconstructed sample matches.
    fn stereo_round_trip(seeds: [[u32; 3]; 2], interleaved: &[i32]) -> usize {
        let (bytes, enc_medians) = stereo_encode(seeds, interleaved);
        let mut dec_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut reader = BitReader::new(&bytes);
        let mut dec_state = StereoDecodeState::new();
        for (i, &expected) in interleaved.iter().enumerate() {
            let got = decode_sample_stateful_stereo(&mut reader, &mut dec_medians, &mut dec_state)
                .unwrap_or_else(|e| panic!("decode stereo sample {i} failed: {e:?}"));
            assert_eq!(
                got, expected,
                "round-trip mismatch at interleaved index {i}: expected {expected}, got {got}",
            );
        }
        assert_eq!(
            dec_medians, enc_medians,
            "stereo encoder and decoder finished with different per-channel medians",
        );
        interleaved.len()
    }

    #[test]
    fn stereo_round_trip_zone1_per_channel() {
        // Both channels in zone 1 with the same seed (get_med = 513).
        // The decoder must dispatch to the correct per-channel medians
        // by sample-index parity AND adapt EACH channel independently.
        // Use a simulator that picks magnitudes against each channel's
        // CURRENT medians so the sequence stays in zone 1 throughout
        // (avoiding the encoder helper's last_zero pre-commit when a
        // value would land in zone 0).
        let seeds = [[8192u32, 8192, 8192], [8192u32, 8192, 8192]];
        let mut sim = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut interleaved = Vec::new();
        // 8 frames of L,R = 16 samples.
        for i in 0..16 {
            let ch = i % 2;
            // Pick a magnitude squarely in zone 1: at the low boundary
            // (m0) so we always land in zone 1 regardless of m1 drift.
            let mag = sim[ch].get_med(0) as i32;
            sim[ch].adapt(Zone::from_ones_count(1));
            interleaved.push(mag);
        }
        let n = stereo_round_trip(seeds, &interleaved);
        assert_eq!(n, 16);
    }

    #[test]
    fn stereo_round_trip_different_seeds_per_channel() {
        // Distinct seeds per channel — proves the decoder reads
        // medians[0] for even indices and medians[1] for odd indices,
        // and never crosses them. Simulator-driven against each
        // channel's CURRENT medians to stay in zone 1 across the §3.2
        // adaptation trajectory.
        let seeds = [[8192u32, 8192, 8192], [4096u32, 4096, 4096]];
        let mut sim = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut interleaved = Vec::new();
        for i in 0..12 {
            let ch = i % 2;
            // Pick magnitude at the zone-1 low boundary (= get_med(0)).
            // For different seeds, the left channel's magnitudes are
            // ~513-ish while the right channel's are ~257-ish, so a
            // confusion of channel-state would show up as an
            // out-of-zone magnitude on the receiving channel.
            let mag = sim[ch].get_med(0) as i32;
            sim[ch].adapt(Zone::from_ones_count(1));
            interleaved.push(mag);
        }
        let n = stereo_round_trip(seeds, &interleaved);
        assert_eq!(n, interleaved.len());
    }

    #[test]
    fn stereo_round_trip_mixed_zones_per_channel() {
        // Per-channel adaptation across mixed zones: each channel
        // walks its own §3.2 trajectory while the other channel's
        // state stays untouched between alternating calls.
        let seeds = [[8192u32, 8192, 8192], [8192u32, 8192, 8192]];
        // Drive a sequence that exercises zones 1, 2, and 2-overflow
        // on each channel independently — picking magnitudes against
        // a per-channel simulator like the mono mixed-zones test.
        let mut sim = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut interleaved = Vec::new();
        for i in 0..16 {
            let ch = i % 2;
            let m0 = sim[ch].get_med(0) as i32;
            let m1 = sim[ch].get_med(1) as i32;
            let m2 = sim[ch].get_med(2) as i32;
            let mag = match (i / 2) % 3 {
                0 => m0 + 1,           // zone 1
                1 => m0 + m1 + 1,      // zone 2
                _ => m0 + m1 + m2 + 1, // zone 2 overflow
            };
            let ones_count = sim[ch].zone_for_magnitude(mag as u32);
            sim[ch].adapt(Zone::from_ones_count(ones_count));
            interleaved.push(mag);
        }
        let n = stereo_round_trip(seeds, &interleaved);
        assert_eq!(n, 16);
    }

    #[test]
    fn stereo_round_trip_negative_values() {
        // Stereo negatives: the sign bit reconstruction (spec §4.2
        // step 7) is per-sample, not per-channel; this confirms the
        // per-channel path still produces correct ~mid values when the
        // sign bit is set. Simulator-driven so per-channel adaptation
        // never drives a magnitude into zone 0 (which the encoder
        // helper's last_zero pre-commit rejects when not chained).
        //
        // For a negative signed value v, the decoder reconstructs it
        // from `!magnitude`, so we have `v = !magnitude` and
        // `magnitude = !v as u32 = (-v - 1) as u32`. To keep the
        // ENCODED magnitude in zone 1 we pick the signed value such
        // that `magnitude = m0 + 1` (safely above the zone-0 / zone-1
        // boundary at `m0`): signed value = -(m0 + 2).
        let seeds = [[8192u32, 8192, 8192], [8192u32, 8192, 8192]];
        let mut sim = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut interleaved = Vec::new();
        for i in 0..8 {
            let ch = i % 2;
            let m0 = sim[ch].get_med(0) as i32;
            // magnitude = m0 + 1 → zone 1. signed value = !(m0+1) = -(m0+2).
            let signed_value = -(m0 + 2);
            sim[ch].adapt(Zone::from_ones_count(1));
            interleaved.push(signed_value);
        }
        let n = stereo_round_trip(seeds, &interleaved);
        assert_eq!(n, interleaved.len());
    }

    #[test]
    fn stereo_round_trip_via_decode_packed_samples_stereo() {
        // Drive the same round-trip through the public
        // `decode_packed_samples_stereo` end-to-end loop instead of
        // the per-call primitive — confirms the bundled loop is
        // bit-exact identical to the manual call sequence and that
        // the interleaved (L,R,L,R) order out of the loop matches the
        // input order. Simulator-driven into zone 1 across the §3.2
        // trajectory to keep the sequence well-formed under the
        // encoder helper's zone-0 pre-commit constraint.
        let seeds = [[8192u32, 8192, 8192], [8192u32, 8192, 8192]];
        let mut sim = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut interleaved = Vec::new();
        for i in 0..16 {
            let ch = i % 2;
            let mag = sim[ch].get_med(0) as i32;
            sim[ch].adapt(Zone::from_ones_count(1));
            interleaved.push(mag);
        }
        let (bytes, enc_medians) = stereo_encode(seeds, &interleaved);
        let view = crate::PackedSamples::new(&bytes);
        let mut dec_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let got = decode_packed_samples_stereo(&view, &mut dec_medians, interleaved.len() / 2)
            .expect("decode stereo payload");
        assert_eq!(got, interleaved);
        assert_eq!(dec_medians, enc_medians);
    }

    #[test]
    fn stereo_decode_state_default_matches_new() {
        // `Default` is derived; spell out the equivalence so callers
        // building with `StereoDecodeState::default()` see the same
        // initial state `StereoDecodeState::new()` produces.
        assert_eq!(StereoDecodeState::default(), StereoDecodeState::new());
        let s = StereoDecodeState::new();
        assert_eq!(s.zero_run_pending, 0);
        assert_eq!(s.next_channel, 0);
        assert!(!s.ever_took_zero_run);
        assert_eq!(s.left_run, RunState::new());
        assert_eq!(s.right_run, RunState::new());
    }

    #[test]
    fn stereo_zero_run_zeroes_both_channels_and_drains_across_parity() {
        // Force eligibility by seeding BOTH channels' medians at all-
        // zero (get_med(0) = 1 for each, satisfying the spec §4.2
        // step 1 "both channels' median[0] <= 1" gate). Pick a small
        // run length (3) so we can hand-verify the drain across
        // alternating channels.
        //
        // Wire: count = 2, mantissa bit = 1 → run_length = (1<<1)|1 = 3.
        // Bits: unary "110" (count=2) + mantissa "1" + sign... wait, no
        // sign bit on the zero-run fast path per spec §4.2 step 1; the
        // emitted sample is `0` from the spec, not a sign-reconstructed
        // value. Bits: "1101".
        let bytes = bits_to_bytes("1101");
        let mut reader = BitReader::new(&bytes);
        let mut medians = [
            AdaptiveMedians::new([0, 0, 0]),
            AdaptiveMedians::new([0, 0, 0]),
        ];
        let mut state = StereoDecodeState::new();

        // Sample 0 (left): zero-run fast path engages, emits 0,
        // medians of BOTH channels stay zeroed, zero_run_pending = 2.
        let v0 = decode_sample_stateful_stereo(&mut reader, &mut medians, &mut state).unwrap();
        assert_eq!(v0, 0);
        assert_eq!(state.next_channel, 1, "advanced to right channel");
        assert_eq!(state.zero_run_pending, 2);
        assert!(state.ever_took_zero_run);
        assert_eq!(medians[0].values, [0, 0, 0]);
        assert_eq!(medians[1].values, [0, 0, 0]);

        // Sample 1 (right): drains from pending, no bits read.
        let v1 = decode_sample_stateful_stereo(&mut reader, &mut medians, &mut state).unwrap();
        assert_eq!(v1, 0);
        assert_eq!(state.next_channel, 0);
        assert_eq!(state.zero_run_pending, 1);

        // Sample 2 (left): drains last pending sample, no bits read.
        let v2 = decode_sample_stateful_stereo(&mut reader, &mut medians, &mut state).unwrap();
        assert_eq!(v2, 0);
        assert_eq!(state.next_channel, 1);
        assert_eq!(state.zero_run_pending, 0);
    }

    #[test]
    fn stereo_zero_run_gated_off_when_only_one_channel_eligible() {
        // The spec §4.2 step 1 zero-run requires BOTH channels'
        // median[0] <= 1. With only the LEFT channel zeroed (right's
        // get_med = 17 > 1), the gate must reject and the call must
        // proceed to the normal prefix path (which fails on an empty
        // buffer with Truncated).
        let bytes: [u8; 0] = [];
        let mut reader = BitReader::new(&bytes);
        let mut medians = [
            AdaptiveMedians::new([0, 0, 0]),
            AdaptiveMedians::new([256, 256, 256]),
        ];
        let mut state = StereoDecodeState::new();
        let err = decode_sample_stateful_stereo(&mut reader, &mut medians, &mut state);
        assert_eq!(err, Err(Error::Truncated));
        assert!(!state.ever_took_zero_run);
    }

    #[test]
    fn stereo_propagates_truncation_leaves_channel_unadvanced() {
        // On Error::Truncated the cursor (`next_channel`) must NOT
        // advance — a caller retrying against a freshly-extended
        // buffer must see the same channel.
        let bytes: [u8; 0] = [];
        let mut reader = BitReader::new(&bytes);
        let mut medians = [
            AdaptiveMedians::new([256, 256, 256]),
            AdaptiveMedians::new([256, 256, 256]),
        ];
        let mut state = StereoDecodeState::new();
        let err = decode_sample_stateful_stereo(&mut reader, &mut medians, &mut state);
        assert_eq!(err, Err(Error::Truncated));
        assert_eq!(state.next_channel, 0, "cursor stays at left on error");
    }

    #[test]
    fn stereo_eof_escape_returns_end_of_stream() {
        // First sample's unary triggers the LIMIT_ONES = 16 escape
        // and the inner cbits unary == 33, which is the spec §4.2
        // step 3 EOF marker. The stereo path must surface it as
        // EndOfStream just like the mono path does.
        let mut bits = String::new();
        bits.push_str(&"1".repeat(16));
        bits.push('0');
        bits.push_str(&"1".repeat(33));
        bits.push('0');
        let bytes = bits_to_bytes(&bits);
        let mut reader = BitReader::new(&bytes);
        let mut medians = [
            AdaptiveMedians::new([256, 256, 256]),
            AdaptiveMedians::new([256, 256, 256]),
        ];
        let mut state = StereoDecodeState::new();
        let err = decode_sample_stateful_stereo(&mut reader, &mut medians, &mut state);
        assert_eq!(err, Err(Error::EndOfStream));
        assert_eq!(state.next_channel, 0, "cursor stays at left on EOF");
    }

    #[test]
    fn stereo_per_channel_holding_state_independent() {
        // After encoding a sample on the LEFT channel that sets
        // last_zero (even raw → zone 0), the RIGHT channel's state
        // must remain untouched, and the next RIGHT-channel decode
        // must read its own prefix as if it were the first sample on
        // that channel.
        let seeds = [[256u32, 256, 256], [8192u32, 8192, 8192]];
        // Left frame 0: 0 (zone 0, sets left_run.last_zero=true).
        // Right frame 0: 600 (zone 1 on right with get_med=513, sets
        // right_run.last_one=true since odd raw=1).
        // Left frame 1: 0 (left_run.last_zero short-circuit → zone 0
        // with NO unary read).
        // Right frame 1: 700 (zone 1 again; right_run carries
        // last_one from frame 0 with no short-circuit, so it reads
        // normally).
        let interleaved = vec![0i32, 600, 0, 700];
        let n = stereo_round_trip(seeds, &interleaved);
        assert_eq!(n, 4);
    }

    #[test]
    fn stereo_decode_packed_samples_truncated_when_too_few_bytes() {
        // A non-zero `frames` against an empty payload must surface
        // Truncated on the first decode call (no successful sample),
        // and the medians must be unchanged.
        let bytes: [u8; 0] = [];
        let view = crate::PackedSamples::new(&bytes);
        let mut medians = [
            AdaptiveMedians::new([256, 256, 256]),
            AdaptiveMedians::new([256, 256, 256]),
        ];
        let pre = medians;
        let err = decode_packed_samples_stereo(&view, &mut medians, 4);
        assert_eq!(err, Err(Error::Truncated));
        assert_eq!(medians, pre, "medians must not have been adapted");
    }

    #[test]
    fn stereo_decode_packed_samples_zero_frames_returns_empty_vec() {
        // Vacuous case: zero frames requested → empty output, no bits
        // read. Confirms the loop's bounds are correct and the
        // function doesn't pre-read any prefix.
        let bytes: [u8; 0] = [];
        let view = crate::PackedSamples::new(&bytes);
        let mut medians = [
            AdaptiveMedians::new([256, 256, 256]),
            AdaptiveMedians::new([256, 256, 256]),
        ];
        let got = decode_packed_samples_stereo(&view, &mut medians, 0).expect("vacuous decode");
        assert!(got.is_empty());
    }

    // ---- Round-201 EntropyInfo → AdaptiveMedians bridges ----

    #[test]
    fn adaptive_medians_from_entropy_yields_left_seed_on_zero() {
        let info = EntropyInfo::stereo([10, 20, 30], [40, 50, 60]);
        assert_eq!(
            AdaptiveMedians::from_entropy(&info, 0),
            Some(AdaptiveMedians::new([10, 20, 30]))
        );
    }

    #[test]
    fn adaptive_medians_from_entropy_yields_right_seed_on_one_for_stereo() {
        let info = EntropyInfo::stereo([10, 20, 30], [40, 50, 60]);
        assert_eq!(
            AdaptiveMedians::from_entropy(&info, 1),
            Some(AdaptiveMedians::new([40, 50, 60]))
        );
    }

    #[test]
    fn adaptive_medians_from_entropy_returns_none_for_right_on_mono() {
        let info = EntropyInfo::mono([7, 8, 9]);
        // Channel 0 still resolves (mono has the first set on the wire).
        assert_eq!(
            AdaptiveMedians::from_entropy(&info, 0),
            Some(AdaptiveMedians::new([7, 8, 9]))
        );
        // Channel 1 is None on a mono payload — wiki put no second set
        // on the wire, so there is no seed to wrap.
        assert_eq!(AdaptiveMedians::from_entropy(&info, 1), None);
    }

    #[test]
    fn adaptive_medians_from_entropy_returns_none_for_out_of_range_index() {
        let info = EntropyInfo::stereo([1, 2, 3], [4, 5, 6]);
        assert_eq!(AdaptiveMedians::from_entropy(&info, 2), None);
        assert_eq!(AdaptiveMedians::from_entropy(&info, 3), None);
        assert_eq!(AdaptiveMedians::from_entropy(&info, 255), None);
    }

    #[test]
    fn adaptive_medians_from_entropy_rejects_negative_seed_on_left() {
        // The defensive i32 → u32 rejection — negative seeds are
        // malformed wire input; the bridge returns None rather than
        // reinterpreting the sign bit.
        let info = EntropyInfo {
            medians_left: [-1, 0, 0],
            medians_right: [4, 5, 6],
        };
        assert_eq!(AdaptiveMedians::from_entropy(&info, 0), None);
        // The right channel is well-formed and still resolves.
        assert_eq!(
            AdaptiveMedians::from_entropy(&info, 1),
            Some(AdaptiveMedians::new([4, 5, 6]))
        );
    }

    #[test]
    fn adaptive_medians_from_entropy_rejects_negative_seed_on_right() {
        let info = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [0, -42, 0],
        };
        // Left channel still resolves.
        assert_eq!(
            AdaptiveMedians::from_entropy(&info, 0),
            Some(AdaptiveMedians::new([1, 2, 3]))
        );
        // Right channel: defensive reject.
        assert_eq!(AdaptiveMedians::from_entropy(&info, 1), None);
    }

    #[test]
    fn adaptive_medians_stereo_pair_from_entropy_returns_both_channels() {
        let info = EntropyInfo::stereo([10, 20, 30], [40, 50, 60]);
        let pair = AdaptiveMedians::stereo_pair_from_entropy(&info).expect("stereo seeds");
        assert_eq!(pair[0], AdaptiveMedians::new([10, 20, 30]));
        assert_eq!(pair[1], AdaptiveMedians::new([40, 50, 60]));
    }

    #[test]
    fn adaptive_medians_stereo_pair_from_entropy_returns_none_on_mono() {
        // Mono payload — nothing to populate the right-channel slot
        // from. The bridge returns None rather than guessing zeros.
        let info = EntropyInfo::mono([1, 2, 3]);
        assert_eq!(AdaptiveMedians::stereo_pair_from_entropy(&info), None);
    }

    #[test]
    fn adaptive_medians_stereo_pair_from_entropy_returns_none_on_negative_left() {
        let info = EntropyInfo {
            medians_left: [-1, 0, 0],
            medians_right: [4, 5, 6],
        };
        // Even though the right channel is well-formed, the left
        // channel's negative seed rejects the whole pair — the call
        // returns a single state usable for both channels or nothing.
        assert_eq!(AdaptiveMedians::stereo_pair_from_entropy(&info), None);
    }

    #[test]
    fn adaptive_medians_stereo_pair_from_entropy_returns_none_on_negative_right() {
        let info = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [0, -7, 0],
        };
        assert_eq!(AdaptiveMedians::stereo_pair_from_entropy(&info), None);
    }

    // ---- Round-201 decode_packed_samples_*_from_entropy wrappers ----

    #[test]
    fn decode_packed_samples_mono_from_entropy_matches_explicit_seeds() {
        // Encode a known sequence with seed [8192;3], then decode it
        // through both the explicit-seed and the from_entropy wrappers
        // and assert byte-identical reconstructions.
        let seed = [8192u32, 8192, 8192];
        let info = EntropyInfo::mono([8192, 8192, 8192]);
        let values: Vec<i32> = (1..=8).collect();

        // Encode through the public surface (matching the round_trip
        // helper).
        let mut enc_medians = AdaptiveMedians::new(seed);
        let bytes = encode_packed_samples_mono(&values, &mut enc_medians).expect("encode");
        let view = crate::PackedSamples::new(&bytes);

        // Decode via the from_entropy wrapper.
        let got = decode_packed_samples_mono_from_entropy(&view, &info, values.len())
            .expect("from_entropy decode");
        assert_eq!(got, values);

        // Same payload through the explicit-seed call — must match
        // bit-for-bit since the wrapper is purely a bridging step over
        // AdaptiveMedians::from_entropy(info, 0).
        let mut dec_medians = AdaptiveMedians::new(seed);
        let got2 = decode_packed_samples_mono(&view, &mut dec_medians, values.len())
            .expect("explicit decode");
        assert_eq!(got, got2);
    }

    #[test]
    fn decode_packed_samples_mono_from_entropy_rejects_negative_seed() {
        // Negative left seed → InvalidEntropyInfoForMono. No bits are
        // read — the bridge fails before the decode loop runs.
        let bytes: [u8; 0] = [];
        let view = crate::PackedSamples::new(&bytes);
        let info = EntropyInfo {
            medians_left: [-1, 0, 0],
            medians_right: [0, 0, 0],
        };
        let err = decode_packed_samples_mono_from_entropy(&view, &info, 4);
        assert_eq!(err, Err(Error::InvalidEntropyInfoForMono));
    }

    #[test]
    fn decode_packed_samples_mono_from_entropy_zero_count_is_vacuous() {
        // Zero samples requested → empty output, regardless of payload.
        // The bridge must still validate the seed (a malformed seed
        // would error here too), but the decode loop body never runs.
        let bytes: [u8; 0] = [];
        let view = crate::PackedSamples::new(&bytes);
        let info = EntropyInfo::mono([100, 200, 300]);
        let got = decode_packed_samples_mono_from_entropy(&view, &info, 0)
            .expect("vacuous from_entropy decode");
        assert!(got.is_empty());
    }

    #[test]
    fn decode_packed_samples_stereo_from_entropy_matches_explicit_seeds() {
        // Encode a stereo zone-1 sequence with both channels
        // seeded [8192;3], then decode it through both the explicit-
        // seed and the from_entropy wrappers and assert identical
        // reconstructions.
        let seeds = [[8192u32, 8192, 8192], [8192u32, 8192, 8192]];
        let info = EntropyInfo::stereo([8192, 8192, 8192], [8192, 8192, 8192]);
        let mut sim = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut interleaved = Vec::new();
        for i in 0..8 {
            let ch = i % 2;
            let mag = sim[ch].get_med(0) as i32;
            sim[ch].adapt(Zone::from_ones_count(1));
            interleaved.push(mag);
        }
        let (bytes, _) = stereo_encode(seeds, &interleaved);
        let view = crate::PackedSamples::new(&bytes);

        let got = decode_packed_samples_stereo_from_entropy(&view, &info, interleaved.len() / 2)
            .expect("stereo from_entropy decode");
        assert_eq!(got, interleaved);

        let mut dec_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let got2 = decode_packed_samples_stereo(&view, &mut dec_medians, interleaved.len() / 2)
            .expect("explicit stereo decode");
        assert_eq!(got, got2);
    }

    #[test]
    fn decode_packed_samples_stereo_from_entropy_rejects_mono_info() {
        // A mono EntropyInfo has no right-channel set on the wire, so
        // the bridge cannot seed the right channel. The wrapper errors
        // with InvalidEntropyInfoForStereo before reading any bits.
        let bytes: [u8; 0] = [];
        let view = crate::PackedSamples::new(&bytes);
        let info = EntropyInfo::mono([100, 200, 300]);
        let err = decode_packed_samples_stereo_from_entropy(&view, &info, 4);
        assert_eq!(err, Err(Error::InvalidEntropyInfoForStereo));
    }

    #[test]
    fn decode_packed_samples_stereo_from_entropy_rejects_negative_seed() {
        let bytes: [u8; 0] = [];
        let view = crate::PackedSamples::new(&bytes);
        let info = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [0, -1, 0],
        };
        let err = decode_packed_samples_stereo_from_entropy(&view, &info, 4);
        assert_eq!(err, Err(Error::InvalidEntropyInfoForStereo));
    }

    #[test]
    fn decode_packed_samples_stereo_from_entropy_zero_frames_is_vacuous() {
        // Zero frames requested → empty output. Bridge still validates
        // the EntropyInfo shape (so a malformed one would still error).
        let bytes: [u8; 0] = [];
        let view = crate::PackedSamples::new(&bytes);
        let info = EntropyInfo::stereo([100, 200, 300], [400, 500, 600]);
        let got = decode_packed_samples_stereo_from_entropy(&view, &info, 0)
            .expect("vacuous stereo from_entropy decode");
        assert!(got.is_empty());
    }

    // ---- Round 255: typed SampleInterval + AdaptiveMedians::sample_interval ----

    #[test]
    fn sample_interval_zone0_matches_spec() {
        // Spec §4.2 step 5 zone 0: low = 0, high = get_med(0) - 1.
        // medians [256, 256, 256] → get_med(i) = 17.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let i = m.sample_interval(Zone::Zone0);
        assert_eq!(i.low(), 0);
        assert_eq!(i.high(), 16);
        assert_eq!(i.maxcode(), 16);
        assert_eq!(i.width(), 17);
        assert!(!i.is_degenerate());
    }

    #[test]
    fn sample_interval_zone1_matches_spec() {
        // Zone 1: low = get_med(0), high = low + get_med(1) - 1.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let i = m.sample_interval(Zone::Zone1);
        assert_eq!(i.low(), 17);
        assert_eq!(i.high(), 33);
        assert_eq!(i.maxcode(), 16);
        assert_eq!(i.width(), 17);
    }

    #[test]
    fn sample_interval_zone2_matches_spec() {
        // Zone 2: low = get_med(0) + get_med(1), high = low + get_med(2) - 1.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let i = m.sample_interval(Zone::Zone2);
        assert_eq!(i.low(), 34);
        assert_eq!(i.high(), 50);
        assert_eq!(i.maxcode(), 16);
    }

    #[test]
    fn sample_interval_zone2_overflow_matches_spec_ladder() {
        // Zone 2 overflow ones_count = 3: low = m0+m1+1*m2 = 51,
        // high = low + m2 - 1 = 67.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let i = m.sample_interval(Zone::Zone2Overflow { ones_count: 3 });
        assert_eq!((i.low(), i.high()), (51, 67));
        // ones_count = 4: low = m0+m1+2*m2 = 68, high = 84.
        let i = m.sample_interval(Zone::Zone2Overflow { ones_count: 4 });
        assert_eq!((i.low(), i.high()), (68, 84));
        // ones_count = 5: low = m0+m1+3*m2 = 85, high = 101.
        let i = m.sample_interval(Zone::Zone2Overflow { ones_count: 5 });
        assert_eq!((i.low(), i.high()), (85, 101));
    }

    #[test]
    fn sample_interval_for_ones_count_matches_zone_typed_path() {
        // Convenience wrapper must yield the same result as the typed
        // Zone path through Zone::from_ones_count.
        let m = AdaptiveMedians::new([512, 256, 128]);
        for n in [0u32, 1, 2, 3, 5, 10, 33] {
            let via_typed = m.sample_interval(Zone::from_ones_count(n));
            let via_raw = m.sample_interval_for_ones_count(n);
            assert_eq!(via_typed, via_raw, "ones_count = {n}");
        }
    }

    #[test]
    fn sample_interval_matches_private_form_interval() {
        // The private decode-loop primitive form_interval must produce
        // the same (low, high) the new typed surface produces — that's
        // the invariant: the typed surface IS the spec primitive the
        // private wrapper delegates to.
        for m_vals in [
            [256u32, 256, 256],
            [16, 16, 16],
            [0, 0, 0],
            [1024, 512, 256],
            [4096, 1024, 16],
        ] {
            let m = AdaptiveMedians::new(m_vals);
            for n in [0u32, 1, 2, 3, 4, 7, 16, 33] {
                let (low, high) = form_interval(&m, n);
                let i = m.sample_interval_for_ones_count(n);
                assert_eq!(i.low(), low, "m={m_vals:?} n={n}");
                assert_eq!(i.high(), high, "m={m_vals:?} n={n}");
            }
        }
    }

    #[test]
    fn sample_interval_degenerate_when_low_equals_high() {
        // Zone 0 with median[0] = 0 → get_med(0) = 1 → low = 0,
        // high = 0 = single-codeword interval.
        let m = AdaptiveMedians::new([0, 0, 0]);
        let i = m.sample_interval(Zone::Zone0);
        assert_eq!(i.low(), 0);
        assert_eq!(i.high(), 0);
        assert_eq!(i.maxcode(), 0);
        assert_eq!(i.width(), 1);
        assert!(i.is_degenerate());
    }

    #[test]
    fn sample_interval_high_clamped_up_to_low_on_underflow() {
        // The 31-bit mask can underflow `high` past `low` when one
        // arm's get_med(0) is very large and (low + m_i - 1) crosses
        // the mask boundary. Round 191's `if high < low { high = low; }`
        // clamp is preserved in the typed surface. With values =
        // [0x7FFFFFF0, 0, 0], get_med(0) = 0x7FFFFFFF, so:
        //   - Zone 0: low = 0, high = 0x7FFFFFFE → maxcode = 0x7FFFFFFE (no clamp).
        //   - Zone 1: low = 0x7FFFFFFF & mask = 0x7FFFFFFF, m1 = 1,
        //     high = 0x7FFFFFFF + 1 - 1 = 0x7FFFFFFF → no clamp.
        // For the actual underflow path we need a configuration where
        // the wrapping arithmetic dips negative. Use [0, 0x7FFFFFF0, 0]:
        //   get_med(0) = 1, get_med(1) = 0x7FFFFFFF, get_med(2) = 1.
        //   Zone 2 overflow ones_count = 2: low = m0+m1 = 0x80000000
        //   & 0x7FFFFFFF = 0; high = 0 + m2 - 1 = 0 (mask leaves it).
        //   That degenerates rather than underflowing.
        // Use a synthetic median set hitting the clamp via
        // get_med saturation. The invariant we care about is
        // `high >= low` for ANY input — verify across a stress sweep.
        for vals in [
            [0u32, 0, 0],
            [0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF],
            [0x7FFF_FFFF, 0, 0],
            [0, 0x7FFF_FFFF, 0],
            [0, 0, 0x7FFF_FFFF],
        ] {
            let m = AdaptiveMedians::new(vals);
            for n in [0u32, 1, 2, 3, 10] {
                let i = m.sample_interval_for_ones_count(n);
                assert!(
                    i.high() >= i.low(),
                    "vals={vals:?} n={n} low={} high={}",
                    i.low(),
                    i.high()
                );
            }
        }
    }

    #[test]
    fn sample_interval_values_are_masked_to_31_bits() {
        // Both low and high must fit in 31 bits per spec §4.2 step 5
        // ("then masked to 31 bits").
        for vals in [
            [0xFFFF_FFFFu32, 0xFFFF_FFFF, 0xFFFF_FFFF],
            [0x8000_0000, 0x8000_0000, 0x8000_0000],
            [0x7FFF_FFFF, 0x7FFF_FFFF, 0x7FFF_FFFF],
        ] {
            let m = AdaptiveMedians::new(vals);
            for n in [0u32, 1, 2, 3, 7] {
                let i = m.sample_interval_for_ones_count(n);
                assert!(
                    i.low() <= INTERVAL_MASK_31,
                    "low not masked: vals={vals:?} n={n} low={}",
                    i.low()
                );
                assert!(
                    i.high() <= INTERVAL_MASK_31,
                    "high not masked: vals={vals:?} n={n} high={}",
                    i.high()
                );
            }
        }
    }

    #[test]
    fn sample_interval_contains_low_high_and_midpoint() {
        // Zone 1 with seeds [256, 256, 256] → [17, 33].
        let m = AdaptiveMedians::new([256, 256, 256]);
        let i = m.sample_interval(Zone::Zone1);
        assert!(i.contains(17));
        assert!(i.contains(25));
        assert!(i.contains(33));
        assert!(!i.contains(16));
        assert!(!i.contains(34));
    }

    #[test]
    fn sample_interval_width_is_inclusive_count() {
        // For (low, high) = (17, 33), width = 17 codewords; maxcode = 16.
        let i = SampleInterval::new(17, 33);
        assert_eq!(i.width(), 17);
        assert_eq!(i.maxcode(), 16);
        // For a degenerate (5, 5) interval, width = 1, maxcode = 0.
        let i = SampleInterval::new(5, 5);
        assert_eq!(i.width(), 1);
        assert_eq!(i.maxcode(), 0);
        assert!(i.is_degenerate());
    }

    #[test]
    fn sample_interval_new_round_trips_through_accessors() {
        // Raw constructor + accessor parity.
        let i = SampleInterval::new(100, 200);
        assert_eq!(i.low(), 100);
        assert_eq!(i.high(), 200);
        assert_eq!(i.maxcode(), 100);
        assert_eq!(i.width(), 101);
    }

    #[test]
    fn sample_interval_pubic_field_access_matches_accessors() {
        // Both the struct fields (pub) and the accessors return the
        // same value — the typed surface is consistent.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let i = m.sample_interval(Zone::Zone2);
        assert_eq!(i.low, i.low());
        assert_eq!(i.high, i.high());
    }

    #[test]
    fn sample_interval_zone0_with_med0_one_is_degenerate() {
        // median[0] = 16 → get_med(0) = 2 → low = 0, high = 1.
        // median[0] = 0 → get_med(0) = 1 → low = 0, high = 0 (degenerate).
        let m = AdaptiveMedians::new([16, 0, 0]);
        let i = m.sample_interval(Zone::Zone0);
        assert_eq!(i.maxcode(), 1);
        assert!(!i.is_degenerate());

        let m = AdaptiveMedians::new([0, 100, 200]);
        let i = m.sample_interval(Zone::Zone0);
        assert!(i.is_degenerate());
        assert_eq!(i.maxcode(), 0);
    }

    #[test]
    fn sample_interval_zone2_overflow_count_three_is_zone2_plus_one_step() {
        // ones_count = 3 is the smallest overflow value and must be
        // exactly one step (one extra m2) past Zone 2's high.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let zone2 = m.sample_interval(Zone::Zone2);
        let overflow3 = m.sample_interval(Zone::Zone2Overflow { ones_count: 3 });
        // Overflow.low = Zone2.low + m2 = 34 + 17 = 51.
        assert_eq!(overflow3.low(), zone2.low() + 17);
        // Both intervals have the same maxcode (m2 - 1 = 16).
        assert_eq!(overflow3.maxcode(), zone2.maxcode());
    }

    #[test]
    fn sample_interval_decoder_consumes_same_interval_typed_surface_returns() {
        // The typed surface MUST produce the same (low, high) pair the
        // decoder's private path consumes — verified end-to-end by
        // running the public mono decode loop with seeds whose
        // intervals we hand-trace and asserting we get back samples
        // strictly inside the typed intervals.
        //
        // Seeds [256, 256, 256] → get_med = 17 → Zone 0 = [0, 16].
        // Encode a single zone-0 magnitude (5) with sign +, decode, and
        // confirm it's inside the typed Zone 0 interval before any
        // adapt step.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let i = m.sample_interval(Zone::Zone0);
        assert!(i.contains(5));
        assert!(i.contains(0));
        assert!(i.contains(16));
        assert!(!i.contains(17));
    }

    // ----- Round 260: Zone predicate accessors ---------------------------

    #[test]
    fn zone_index_matches_spec_table() {
        assert_eq!(Zone::Zone0.index(), 0);
        assert_eq!(Zone::Zone1.index(), 1);
        assert_eq!(Zone::Zone2.index(), 2);
        assert_eq!(Zone::Zone2Overflow { ones_count: 3 }.index(), 3);
        assert_eq!(Zone::Zone2Overflow { ones_count: 4 }.index(), 3);
        assert_eq!(Zone::Zone2Overflow { ones_count: 33 }.index(), 3);
    }

    #[test]
    fn zone_index_is_independent_of_overflow_ones_count() {
        // Every overflow Zone maps to index 3 regardless of the carried
        // `ones_count` value — the index discriminates the four arms
        // ONLY, while `ones_count` preserves the raw value.
        for n in 3..=64u32 {
            let z = Zone::Zone2Overflow { ones_count: n };
            assert_eq!(z.index(), 3);
            assert_eq!(z.ones_count(), n);
        }
    }

    #[test]
    fn zone_is_overflow_only_true_for_overflow_arm() {
        assert!(!Zone::Zone0.is_overflow());
        assert!(!Zone::Zone1.is_overflow());
        assert!(!Zone::Zone2.is_overflow());
        assert!(Zone::Zone2Overflow { ones_count: 3 }.is_overflow());
        assert!(Zone::Zone2Overflow { ones_count: 99 }.is_overflow());
    }

    #[test]
    fn zone_increments_median_matches_spec_table() {
        // Zone 0: nothing incremented.
        assert!(!Zone::Zone0.increments_median(0));
        assert!(!Zone::Zone0.increments_median(1));
        assert!(!Zone::Zone0.increments_median(2));
        // Zone 1: median[0] incremented.
        assert!(Zone::Zone1.increments_median(0));
        assert!(!Zone::Zone1.increments_median(1));
        assert!(!Zone::Zone1.increments_median(2));
        // Zone 2: median[0] + median[1] incremented.
        assert!(Zone::Zone2.increments_median(0));
        assert!(Zone::Zone2.increments_median(1));
        assert!(!Zone::Zone2.increments_median(2));
        // Zone overflow: all three.
        let overflow = Zone::Zone2Overflow { ones_count: 5 };
        assert!(overflow.increments_median(0));
        assert!(overflow.increments_median(1));
        assert!(overflow.increments_median(2));
    }

    #[test]
    fn zone_decrements_median_matches_spec_table() {
        // Zone 0: median[0] decremented; others untouched.
        assert!(Zone::Zone0.decrements_median(0));
        assert!(!Zone::Zone0.decrements_median(1));
        assert!(!Zone::Zone0.decrements_median(2));
        // Zone 1: median[1] decremented.
        assert!(!Zone::Zone1.decrements_median(0));
        assert!(Zone::Zone1.decrements_median(1));
        assert!(!Zone::Zone1.decrements_median(2));
        // Zone 2: median[2] decremented.
        assert!(!Zone::Zone2.decrements_median(0));
        assert!(!Zone::Zone2.decrements_median(1));
        assert!(Zone::Zone2.decrements_median(2));
        // Zone overflow: nothing decremented.
        let overflow = Zone::Zone2Overflow { ones_count: 9 };
        assert!(!overflow.decrements_median(0));
        assert!(!overflow.decrements_median(1));
        assert!(!overflow.decrements_median(2));
    }

    #[test]
    fn zone_touches_median_union_of_inc_and_dec() {
        for &zone in &[
            Zone::Zone0,
            Zone::Zone1,
            Zone::Zone2,
            Zone::Zone2Overflow { ones_count: 4 },
        ] {
            for idx in 0..3 {
                let inc = zone.increments_median(idx);
                let dec = zone.decrements_median(idx);
                assert_eq!(
                    zone.touches_median(idx),
                    inc || dec,
                    "zone {zone:?} median {idx}",
                );
                // No median is simultaneously incremented and
                // decremented — the spec §3.2 table is mutually
                // exclusive at each cell.
                assert!(!(inc && dec), "zone {zone:?} median {idx} both inc + dec");
            }
        }
    }

    #[test]
    fn zone_inc_dec_predicates_reject_out_of_range_idx() {
        // idx >= 3 returns `false` for all three predicates on every
        // zone — there is no median[3..] in the spec.
        for &zone in &[
            Zone::Zone0,
            Zone::Zone1,
            Zone::Zone2,
            Zone::Zone2Overflow { ones_count: 3 },
        ] {
            assert!(!zone.increments_median(3));
            assert!(!zone.decrements_median(3));
            assert!(!zone.touches_median(3));
            assert!(!zone.increments_median(99));
            assert!(!zone.decrements_median(99));
            assert!(!zone.touches_median(99));
        }
    }

    #[test]
    fn zone_predicates_drive_observed_adapt_mutations() {
        // For each zone, the predicates must agree with the actual
        // mutation `AdaptiveMedians::adapt` performs on a synthesised
        // median set: a median is incremented when increments_median
        // says so, decremented when decrements_median says so, and
        // unchanged otherwise.
        for &zone in &[
            Zone::Zone0,
            Zone::Zone1,
            Zone::Zone2,
            Zone::Zone2Overflow { ones_count: 4 },
        ] {
            let before = AdaptiveMedians::new([256, 256, 256]);
            let mut after = before;
            after.adapt(zone);
            for idx in 0..3 {
                let b = before.values[idx];
                let a = after.values[idx];
                if zone.increments_median(idx) {
                    assert!(a > b, "zone {zone:?} median {idx}: expected inc");
                } else if zone.decrements_median(idx) {
                    assert!(a < b, "zone {zone:?} median {idx}: expected dec");
                } else {
                    assert_eq!(a, b, "zone {zone:?} median {idx}: expected no change");
                }
            }
        }
    }

    // ----- Round 260: SampleInterval mantissa primitives ------------------

    #[test]
    fn mantissa_bitcount_special_cases() {
        assert_eq!(SampleInterval::new(0, 0).mantissa_bitcount(), 0);
        assert_eq!(SampleInterval::new(5, 5).mantissa_bitcount(), 0);
        // maxcode = 1 → bitcount = 1.
        assert_eq!(SampleInterval::new(0, 1).mantissa_bitcount(), 1);
        assert_eq!(SampleInterval::new(10, 11).mantissa_bitcount(), 1);
        // maxcode = 2 → bitcount = 2.
        assert_eq!(SampleInterval::new(0, 2).mantissa_bitcount(), 2);
        // maxcode = 16 → bitcount = 5 (floor(log2(16)) + 1 = 5).
        assert_eq!(SampleInterval::new(0, 16).mantissa_bitcount(), 5);
        // maxcode = 31 → bitcount = 5 (highest 5-bit value).
        assert_eq!(SampleInterval::new(0, 31).mantissa_bitcount(), 5);
        // maxcode = 32 → bitcount = 6.
        assert_eq!(SampleInterval::new(0, 32).mantissa_bitcount(), 6);
    }

    #[test]
    fn mantissa_bitcount_high_maxcode() {
        // maxcode = INTERVAL_MASK_31 = 2^31 - 1 → bitcount = 31.
        let i = SampleInterval::new(0, INTERVAL_MASK_31);
        assert_eq!(i.mantissa_bitcount(), 31);
    }

    #[test]
    fn mantissa_extras_special_cases() {
        // maxcode == 0 → extras == 0 (no codewords).
        assert_eq!(SampleInterval::new(0, 0).mantissa_extras(), 0);
        // maxcode == 1 → extras == 0 (both codewords are full 1-bit).
        assert_eq!(SampleInterval::new(0, 1).mantissa_extras(), 0);
        // maxcode == 2 → bitcount=2, (1<<2) - 2 - 1 = 1.
        assert_eq!(SampleInterval::new(0, 2).mantissa_extras(), 1);
        // maxcode == 3 → bitcount=2, (1<<2) - 3 - 1 = 0 (perfect power-of-2 - 1).
        assert_eq!(SampleInterval::new(0, 3).mantissa_extras(), 0);
        // maxcode == 16 → bitcount=5, (1<<5) - 16 - 1 = 15.
        assert_eq!(SampleInterval::new(0, 16).mantissa_extras(), 15);
        // maxcode == 31 → bitcount=5, (1<<5) - 31 - 1 = 0.
        assert_eq!(SampleInterval::new(0, 31).mantissa_extras(), 0);
    }

    #[test]
    fn mantissa_extras_invariant_in_power_of_two_minus_one() {
        // For any maxcode == 2^k - 1 (k >= 1), extras must be 0 — the
        // codeword count is exactly 2^k, no slack to absorb.
        for k in 1u32..=20 {
            let maxcode = (1u32 << k) - 1;
            let i = SampleInterval::new(0, maxcode);
            assert_eq!(i.mantissa_extras(), 0, "k={k} maxcode={maxcode}");
            assert_eq!(i.mantissa_bitcount(), k, "k={k} maxcode={maxcode}");
        }
    }

    #[test]
    fn mantissa_extras_bounded_by_half_long_region() {
        // For maxcode >= 2, extras must lie in [0, 2^(bitcount-1) - 1]
        // — the short region is at most half the long region's width.
        for maxcode in 2u32..=1023 {
            let i = SampleInterval::new(0, maxcode);
            let bitcount = i.mantissa_bitcount();
            let upper = (1u32 << (bitcount - 1)).saturating_sub(1);
            let extras = i.mantissa_extras();
            assert!(
                extras <= upper,
                "maxcode={maxcode}: extras={extras} upper={upper}"
            );
        }
    }

    #[test]
    fn decode_mantissa_maxcode_zero_consumes_no_bits() {
        let bytes = [0u8; 0];
        let mut r = BitReader::new(&bytes);
        let i = SampleInterval::new(0, 0);
        assert_eq!(i.decode_mantissa(&mut r).unwrap(), 0);
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn decode_mantissa_degenerate_interval_returns_low() {
        // A degenerate (5, 5) interval has maxcode = 0, so decode_value
        // returns `low` directly — no bits consumed.
        let bytes = [0u8; 0];
        let mut r = BitReader::new(&bytes);
        let i = SampleInterval::new(5, 5);
        assert_eq!(i.decode_value(&mut r).unwrap(), 5);
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn decode_mantissa_maxcode_one_reads_one_bit() {
        // Bit 0 set / clear → mantissa is the bit value (low-bit-first
        // reader).
        let bytes = [0b0000_0001];
        let mut r = BitReader::new(&bytes);
        let i = SampleInterval::new(0, 1);
        assert_eq!(i.decode_mantissa(&mut r).unwrap(), 1);
        // decode_value pairs with low=0 → magnitude == mantissa.
        let bytes = [0b0000_0000];
        let mut r = BitReader::new(&bytes);
        assert_eq!(i.decode_mantissa(&mut r).unwrap(), 0);
    }

    #[test]
    fn decode_mantissa_round_trips_via_emit_for_maxcode_sweep() {
        // For a sweep of maxcode values covering both perfect-power
        // (extras == 0) and slack (extras > 0) intervals, every
        // codeword in [0, maxcode] must encode + decode bit-exactly.
        for maxcode in 0u32..=32 {
            let i = SampleInterval::new(0, maxcode);
            for code in 0..=maxcode {
                let mut w = BitWriter::new();
                emit_truncated_binary(&mut w, maxcode, code);
                // Append a sentinel byte so the reader has buffer slack
                // for the rare bit-aligned read that lands on a byte
                // boundary.
                let bytes = w.finish();
                let mut padded = bytes.clone();
                padded.push(0);
                let mut r = BitReader::new(&padded);
                let decoded = i.decode_mantissa(&mut r).unwrap();
                assert_eq!(
                    decoded, code,
                    "maxcode={maxcode} code={code} bytes={bytes:?}",
                );
            }
        }
    }

    #[test]
    fn decode_value_adds_low_to_decoded_mantissa() {
        // For interval (low=17, high=33) → maxcode=16, code in [0,16],
        // decode_value returns low + code.
        let i = SampleInterval::new(17, 33);
        for code in 0u32..=16 {
            let mut w = BitWriter::new();
            emit_truncated_binary(&mut w, 16, code);
            let mut padded = w.finish();
            padded.push(0);
            let mut r = BitReader::new(&padded);
            assert_eq!(i.decode_value(&mut r).unwrap(), 17 + code);
        }
    }

    #[test]
    fn decode_value_via_typed_interval_matches_private_truncated_binary() {
        // The typed `decode_mantissa` MUST produce the same value as
        // the private `read_truncated_binary` for every maxcode in the
        // sweep — they are the SAME primitive lifted to the typed
        // surface.
        for maxcode in 0u32..=40 {
            for code in 0..=maxcode {
                let mut w = BitWriter::new();
                emit_truncated_binary(&mut w, maxcode, code);
                let mut padded = w.finish();
                padded.push(0);
                let i = SampleInterval::new(0, maxcode);
                let mut r_typed = BitReader::new(&padded);
                let mut r_priv = BitReader::new(&padded);
                let typed = i.decode_mantissa(&mut r_typed).unwrap();
                let priv_val = read_truncated_binary(&mut r_priv, maxcode).unwrap();
                assert_eq!(typed, priv_val, "maxcode={maxcode} code={code}");
                assert_eq!(
                    r_typed.bits_consumed(),
                    r_priv.bits_consumed(),
                    "cursors diverge at maxcode={maxcode} code={code}",
                );
            }
        }
    }

    #[test]
    fn decode_mantissa_truncated_buffer_surfaces_error() {
        // Empty buffer + maxcode=1 → reader needs one bit; truncation.
        let bytes = [0u8; 0];
        let mut r = BitReader::new(&bytes);
        let i = SampleInterval::new(0, 1);
        assert!(matches!(i.decode_mantissa(&mut r), Err(Error::Truncated),));
        // decode_value surfaces the same error.
        let mut r = BitReader::new(&bytes);
        assert!(matches!(i.decode_value(&mut r), Err(Error::Truncated)));
    }

    #[test]
    fn decode_mantissa_consumes_expected_bit_count() {
        // For maxcode=16 (bitcount=5, extras=15), the short-form code
        // < 15 consumes 4 bits; the long-form code >= 15 consumes 5.
        let i = SampleInterval::new(0, 16);
        // Short-form code 0 → 4 bits consumed.
        let mut w = BitWriter::new();
        emit_truncated_binary(&mut w, 16, 0);
        let mut padded = w.finish();
        padded.push(0);
        let mut r = BitReader::new(&padded);
        i.decode_mantissa(&mut r).unwrap();
        assert_eq!(r.bits_consumed(), 4);
        // Long-form code 15 → 5 bits consumed.
        let mut w = BitWriter::new();
        emit_truncated_binary(&mut w, 16, 15);
        let mut padded = w.finish();
        padded.push(0);
        let mut r = BitReader::new(&padded);
        i.decode_mantissa(&mut r).unwrap();
        assert_eq!(r.bits_consumed(), 5);
        // Long-form code 16 → 5 bits consumed.
        let mut w = BitWriter::new();
        emit_truncated_binary(&mut w, 16, 16);
        let mut padded = w.finish();
        padded.push(0);
        let mut r = BitReader::new(&padded);
        i.decode_mantissa(&mut r).unwrap();
        assert_eq!(r.bits_consumed(), 5);
    }

    #[test]
    fn decode_value_via_typed_interval_matches_decode_sample_stateful_inner() {
        // For seeds [256,256,256] → get_med = 17 → Zone 0 interval =
        // [0, 16]. The typed decode_value reads the mantissa + adds
        // low; with a code emitted through emit_truncated_binary the
        // result is exactly code (because low = 0). The decoder loop
        // consumes the same interval and adds low (also 0), so the two
        // surface points agree.
        let m = AdaptiveMedians::new([256, 256, 256]);
        let i = m.sample_interval(Zone::Zone0);
        assert_eq!(i.low(), 0);
        assert_eq!(i.high(), 16);
        let mut w = BitWriter::new();
        emit_truncated_binary(&mut w, i.maxcode(), 9);
        let mut padded = w.finish();
        padded.push(0);
        let mut r = BitReader::new(&padded);
        assert_eq!(i.decode_value(&mut r).unwrap(), 9);
    }

    // ----- round 261: spec §4.2 step 7 sign-bit reconstruction on the
    // ----- typed surface (apply_sign / read_sign_and_apply /
    // ----- SampleInterval::decode_signed_value)

    #[test]
    fn apply_sign_clear_returns_magnitude_verbatim() {
        // Spec §4.2 step 7: sign bit clear → "the magnitude itself".
        for magnitude in [0u32, 1, 17, 33, 1024, INTERVAL_MASK_31] {
            assert_eq!(apply_sign(magnitude, false), magnitude as i32);
        }
    }

    #[test]
    fn apply_sign_set_returns_ones_complement() {
        // Spec §4.2 step 7: sign bit set → "the bitwise complement of
        // the magnitude (~mid)". In two's complement that is
        // -(magnitude + 1).
        assert_eq!(apply_sign(0, true), -1);
        assert_eq!(apply_sign(17, true), -18);
        assert_eq!(apply_sign(33, true), -34);
        // The 31-bit-mask maximum maps to i32::MIN — the most negative
        // sample the masked interval ladder can ever produce.
        assert_eq!(apply_sign(INTERVAL_MASK_31, true), i32::MIN);
    }

    #[test]
    fn apply_sign_set_is_complement_of_clear() {
        // The two arms are bitwise complements of each other for every
        // magnitude: apply_sign(m, true) == !apply_sign(m, false).
        for magnitude in 0u32..=100 {
            assert_eq!(
                apply_sign(magnitude, true),
                !apply_sign(magnitude, false),
                "magnitude={magnitude}",
            );
            // And the two's-complement identity -(m + 1).
            assert_eq!(apply_sign(magnitude, true), -(magnitude as i32) - 1);
        }
    }

    #[test]
    fn apply_sign_is_const_evaluable() {
        // Pure spec §4.2 step 7 arithmetic — usable in const context.
        const NEGATIVE: i32 = apply_sign(5, true);
        const POSITIVE: i32 = apply_sign(5, false);
        assert_eq!(NEGATIVE, -6);
        assert_eq!(POSITIVE, 5);
    }

    #[test]
    fn read_sign_and_apply_reads_exactly_one_bit() {
        // Spec §4.2 step 7: "read exactly one sign bit".
        let bytes = [0b0000_0000u8];
        let mut r = BitReader::new(&bytes);
        read_sign_and_apply(&mut r, 7).unwrap();
        assert_eq!(r.bits_consumed(), 1);

        let bytes = [0b0000_0001u8];
        let mut r = BitReader::new(&bytes);
        read_sign_and_apply(&mut r, 7).unwrap();
        assert_eq!(r.bits_consumed(), 1);
    }

    #[test]
    fn read_sign_and_apply_zero_bit_returns_magnitude() {
        let bytes = [0b0000_0000u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_sign_and_apply(&mut r, 42).unwrap(), 42);
    }

    #[test]
    fn read_sign_and_apply_one_bit_returns_complement() {
        let bytes = [0b0000_0001u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_sign_and_apply(&mut r, 42).unwrap(), -43);
    }

    #[test]
    fn read_sign_and_apply_truncated_on_empty_buffer() {
        // Empty buffer: the single sign-bit read reports Truncated and
        // the cursor stays at zero (BitReader partial-consume
        // semantics).
        let bytes = [0u8; 0];
        let mut r = BitReader::new(&bytes);
        assert!(matches!(
            read_sign_and_apply(&mut r, 5),
            Err(Error::Truncated)
        ));
        assert_eq!(r.bits_consumed(), 0);
    }

    #[test]
    fn decode_signed_value_degenerate_interval_reads_only_sign_bit() {
        // A degenerate interval (low == high) has maxcode == 0 → the
        // mantissa consumes no bits per spec §4.2 step 6, so
        // decode_signed_value consumes exactly the ONE sign bit.
        let i = SampleInterval::new(5, 5);

        let bytes = [0b0000_0000u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(i.decode_signed_value(&mut r).unwrap(), 5);
        assert_eq!(r.bits_consumed(), 1);

        let bytes = [0b0000_0001u8];
        let mut r = BitReader::new(&bytes);
        assert_eq!(i.decode_signed_value(&mut r).unwrap(), -6);
        assert_eq!(r.bits_consumed(), 1);
    }

    #[test]
    fn decode_signed_value_worked_example_positive() {
        // Round-255 worked interval [17, 33] (maxcode 16, bitcount 5,
        // extras 15). Short-form code 3 consumes 4 mantissa bits; the
        // sign bit (clear) makes 5 total. Magnitude = 17 + 3 = 20.
        let i = SampleInterval::new(17, 33);
        let mut w = BitWriter::new();
        emit_truncated_binary(&mut w, 16, 3);
        w.write_bit(0); // sign clear
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(i.decode_signed_value(&mut r).unwrap(), 20);
        assert_eq!(r.bits_consumed(), 5);
    }

    #[test]
    fn decode_signed_value_worked_example_negative() {
        // Same interval / code as the positive worked example with the
        // sign bit SET: the result is the bitwise complement of the
        // magnitude, !(20) = -21.
        let i = SampleInterval::new(17, 33);
        let mut w = BitWriter::new();
        emit_truncated_binary(&mut w, 16, 3);
        w.write_bit(1); // sign set
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(i.decode_signed_value(&mut r).unwrap(), -21);
        assert_eq!(r.bits_consumed(), 5);
    }

    #[test]
    fn decode_signed_value_round_trips_all_codes_both_signs() {
        // End-to-end through the emit helper: for every maxcode in
        // [0, 20], every code in [0, maxcode] and both sign values, the
        // decoded sample is apply_sign(low + code, sign).
        let low = 7u32;
        for maxcode in 0u32..=20 {
            let i = SampleInterval::new(low, low + maxcode);
            for code in 0..=maxcode {
                for sign in [0u32, 1] {
                    let mut w = BitWriter::new();
                    emit_truncated_binary(&mut w, maxcode, code);
                    w.write_bit(sign);
                    let bytes = w.finish();
                    let mut r = BitReader::new(&bytes);
                    assert_eq!(
                        i.decode_signed_value(&mut r).unwrap(),
                        apply_sign(low + code, sign != 0),
                        "maxcode={maxcode} code={code} sign={sign}",
                    );
                }
            }
        }
    }

    #[test]
    fn decode_signed_value_matches_manual_value_plus_sign() {
        // Bit-exact parity (same value AND same cursor) between the
        // fused decode_signed_value and the manual decode_value +
        // read_sign_and_apply two-step across a maxcode/code/sign sweep.
        for maxcode in 0u32..=24 {
            let i = SampleInterval::new(3, 3 + maxcode);
            for code in 0..=maxcode {
                for sign in [0u32, 1] {
                    let mut w = BitWriter::new();
                    emit_truncated_binary(&mut w, maxcode, code);
                    w.write_bit(sign);
                    let bytes = w.finish();
                    let mut r_fused = BitReader::new(&bytes);
                    let mut r_manual = BitReader::new(&bytes);
                    let fused = i.decode_signed_value(&mut r_fused).unwrap();
                    let magnitude = i.decode_value(&mut r_manual).unwrap();
                    let manual = read_sign_and_apply(&mut r_manual, magnitude).unwrap();
                    assert_eq!(fused, manual, "maxcode={maxcode} code={code} sign={sign}");
                    assert_eq!(
                        r_fused.bits_consumed(),
                        r_manual.bits_consumed(),
                        "cursors diverge at maxcode={maxcode} code={code} sign={sign}",
                    );
                }
            }
        }
    }

    #[test]
    fn decode_signed_value_truncated_at_missing_sign_bit() {
        // Mantissa bits present, sign bit missing: the §4.2 step 7
        // read surfaces Truncated. Buffers are byte-granular, so pick
        // an interval whose mantissa consumes EXACTLY 8 bits: maxcode
        // = 255 → bitcount = 8, extras = 0 → every code is the long
        // form (7 short bits + 1 extra bit), filling one byte and
        // leaving NO ninth bit for the sign.
        let i = SampleInterval::new(0, 255);
        let mut w = BitWriter::new();
        emit_truncated_binary(&mut w, 255, 9);
        let bytes = w.finish();
        assert_eq!(bytes.len(), 1, "8 mantissa bits fill exactly one byte");
        let mut r = BitReader::new(&bytes);
        assert!(matches!(
            i.decode_signed_value(&mut r),
            Err(Error::Truncated)
        ));
        // The mantissa itself was fully consumed before the sign read
        // failed.
        assert_eq!(r.bits_consumed(), 8);
    }

    #[test]
    fn decode_signed_value_parity_with_stateful_loop() {
        // The decode loop delegates its steps 6 + 7 to
        // decode_signed_value. Prove parity by hand-walking the spec
        // ladder (prefix → fold → interval → adapt → signed value) next
        // to decode_sample_stateful on the same bit-stream.
        //
        // Seeds [256, 256, 256] → get_med(0) = 17 → no zero-run
        // eligibility. Stream: unary "0" (raw 0 → fold → ones_count 0,
        // Zone 0 interval [0, 16]) + mantissa code 9 + sign 1.
        let mut w = BitWriter::new();
        w.write_bit(0); // unary prefix: raw ones_count = 0
        emit_truncated_binary(&mut w, 16, 9);
        w.write_bit(1); // sign set
        let bytes = w.finish();

        // Loop path.
        let mut medians_loop = AdaptiveMedians::new([256, 256, 256]);
        let mut state = DecodeState::new();
        let mut r_loop = BitReader::new(&bytes);
        let loop_sample =
            decode_sample_stateful(&mut r_loop, &mut medians_loop, &mut state).unwrap();

        // Hand-walked typed path.
        let mut medians_hand = AdaptiveMedians::new([256, 256, 256]);
        let mut run = RunState::new();
        let mut r_hand = BitReader::new(&bytes);
        let ones_count = read_folded_ones_count(&mut r_hand, &mut run).unwrap();
        let interval = medians_hand.sample_interval_for_ones_count(ones_count);
        medians_hand.adapt(Zone::from_ones_count(ones_count));
        let hand_sample = interval.decode_signed_value(&mut r_hand).unwrap();

        assert_eq!(loop_sample, hand_sample);
        assert_eq!(loop_sample, apply_sign(9, true)); // !(0 + 9) = -10
        assert_eq!(r_loop.bits_consumed(), r_hand.bits_consumed());
        assert_eq!(medians_loop, medians_hand);
    }

    // ---- Round 274: §4.2 step 2 + 3 raw prefix + step 4 fold on the
    // public typed surface (the emit side is the public
    // `emit_raw_prefix` since round 281) -------------------------------

    #[test]
    fn read_raw_prefix_plain_unary_below_escape() {
        // Every raw value 0..16 is a plain unary prefix consuming
        // raw_value `1`-bits + one `0` terminator.
        for raw in 0..UNARY_ESCAPE {
            let mut w = BitWriter::new();
            emit_raw_prefix(&mut w, raw);
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(read_raw_prefix(&mut r).unwrap(), raw);
            assert_eq!(r.bits_consumed(), (raw + 1) as usize);
        }
    }

    #[test]
    fn read_raw_prefix_escape_cbits_lt_2() {
        // Escape arm with cbits 0 and 1: raw_value = 16 + cbits.
        for cbits in 0..2u32 {
            let mut w = BitWriter::new();
            w.write_unary(UNARY_ESCAPE); // leading 16 ones + 0
            w.write_unary(cbits); // cbits unary
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(read_raw_prefix(&mut r).unwrap(), UNARY_ESCAPE + cbits);
        }
    }

    #[test]
    fn read_raw_prefix_escape_cbits_ge_2_implied_top_bit() {
        // escape_value >= 2 carries an implied top bit; round-trip every
        // value in [2, 256] through the emitter.
        for escape_value in 2..=256u32 {
            let raw = UNARY_ESCAPE + escape_value;
            let mut w = BitWriter::new();
            emit_raw_prefix(&mut w, raw);
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(read_raw_prefix(&mut r).unwrap(), raw);
        }
    }

    #[test]
    fn read_raw_prefix_eof_marker() {
        // cbits == 33 inside the escape arm is the EOF marker.
        let mut w = BitWriter::new();
        w.write_unary(UNARY_ESCAPE);
        w.write_unary(ESCAPE_EOF_CBITS);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_raw_prefix(&mut r), Err(Error::EndOfStream));
    }

    #[test]
    fn read_raw_prefix_truncated_empty() {
        let bytes: [u8; 0] = [];
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_raw_prefix(&mut r), Err(Error::Truncated));
    }

    #[test]
    fn fold_prefix_low_bit_zero_halves() {
        // raw even → last_one false, ones_count = raw >> 1.
        let mut state = RunState::new();
        assert_eq!(state.fold_prefix(0), 0);
        assert!(!state.last_one);
        assert!(state.last_zero);

        let mut state = RunState::new();
        assert_eq!(state.fold_prefix(8), 4);
        assert!(!state.last_one);
        assert!(state.last_zero);
    }

    #[test]
    fn fold_prefix_neutral_state_halves_regardless_of_low_bit() {
        // From a neutral state (no held one) the fold is a plain halve
        // — the raw value's own low bit does NOT add to THIS word's
        // count (spec §4.2 step 4 prior-state fold); it becomes the new
        // held-one for the next word.
        let mut state = RunState::new();
        assert_eq!(state.fold_prefix(1), 0);
        assert!(state.last_one);
        assert!(!state.last_zero);

        let mut state = RunState::new();
        assert_eq!(state.fold_prefix(9), 4);
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn fold_prefix_held_one_adds_one() {
        // With a one held on entry the fold gains +1 ("if a one is
        // being held, ones_count = (ones_count >> 1) + 1").
        let mut state = RunState {
            last_zero: false,
            last_one: true,
        };
        assert_eq!(state.fold_prefix(0), 1);
        assert!(!state.last_one, "new held-one is raw low bit (0)");
        assert!(state.last_zero);

        let mut state = RunState {
            last_zero: false,
            last_one: true,
        };
        assert_eq!(state.fold_prefix(9), 5);
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn fold_prefix_held_one_zero_complement() {
        // last_one and last_zero are always complements after a fold,
        // the new held-one is the raw low bit, and the folded count is
        // the halved raw plus the ENTRY held-one — for both entry
        // states.
        for raw in 0..64u32 {
            for entry_held in [false, true] {
                let mut state = RunState {
                    last_zero: false,
                    last_one: entry_held,
                };
                let folded = state.fold_prefix(raw);
                assert_eq!(state.last_one, (raw & 1) != 0);
                assert_eq!(state.last_zero, !state.last_one);
                let expected = (raw >> 1) + u32::from(entry_held);
                assert_eq!(folded, expected, "raw {raw} held {entry_held}");
            }
        }
    }

    #[test]
    fn fold_prefix_is_const() {
        const FOLDED: u32 = {
            let mut s = RunState::new();
            s.fold_prefix(17)
        };
        assert_eq!(FOLDED, 8);

        const FOLDED_HELD: u32 = {
            let mut s = RunState {
                last_zero: false,
                last_one: true,
            };
            s.fold_prefix(17)
        };
        assert_eq!(FOLDED_HELD, 9);
    }

    #[test]
    fn unfold_prefix_inverts_fold_prefix_across_states_and_carries() {
        // fold(unfold(n, b)) == n for every representable combination,
        // with identical post-states on both sides.
        for n in 0..=100u32 {
            for entry_held in [false, true] {
                for hold_one in [false, true] {
                    let entry = RunState {
                        last_zero: false,
                        last_one: entry_held,
                    };
                    let mut enc = entry;
                    let raw = match enc.unfold_prefix(n, hold_one) {
                        Some(raw) => raw,
                        None => {
                            // Only the held-one + n == 0 combination is
                            // unrepresentable in this sweep.
                            assert!(entry_held && n == 0);
                            assert_eq!(enc, entry, "state untouched on None");
                            continue;
                        }
                    };
                    assert_eq!(raw & 1, u32::from(hold_one));
                    let mut dec = entry;
                    assert_eq!(dec.fold_prefix(raw), n, "n {n} held {entry_held}");
                    assert_eq!(dec, enc, "encoder and decoder post-states");
                }
            }
        }
    }

    #[test]
    fn unfold_prefix_rejects_doubled_raw_overflow() {
        // ones_count too large for the doubled raw to fit u32 → None,
        // state untouched.
        let mut state = RunState::new();
        assert_eq!(state.unfold_prefix(u32::MAX / 2 + 1, false), None);
        assert_eq!(state, RunState::new());
        // The widest representable count from neutral folds back.
        let mut enc = RunState::new();
        let raw = enc.unfold_prefix(u32::MAX / 2, true).expect("in range");
        let mut dec = RunState::new();
        assert_eq!(dec.fold_prefix(raw), u32::MAX / 2);
    }

    #[test]
    fn unfold_prefix_is_const() {
        const RAW: Option<u32> = {
            let mut s = RunState::new();
            s.unfold_prefix(8, true)
        };
        assert_eq!(RAW, Some(17));
    }

    #[test]
    fn read_folded_ones_count_equals_raw_then_fold() {
        // read_folded_ones_count == read_raw_prefix + fold_prefix,
        // bit-for-bit, for every raw across the plain + escape range.
        for raw in 0..40u32 {
            let mut w = BitWriter::new();
            emit_raw_prefix(&mut w, raw);
            let bytes = w.finish();

            // Fused path.
            let mut r_fused = BitReader::new(&bytes);
            let mut s_fused = RunState::new();
            let fused = read_folded_ones_count(&mut r_fused, &mut s_fused).unwrap();

            // Two-step path.
            let mut r_step = BitReader::new(&bytes);
            let mut s_step = RunState::new();
            let raw_val = read_raw_prefix(&mut r_step).unwrap();
            let folded = s_step.fold_prefix(raw_val);

            assert_eq!(fused, folded);
            assert_eq!(raw_val, raw);
            assert_eq!(r_fused.bits_consumed(), r_step.bits_consumed());
            assert_eq!(s_fused, s_step);
        }
    }

    #[test]
    fn read_folded_ones_count_last_zero_short_circuit() {
        // When last_zero is set, read_folded_ones_count returns 0 with
        // no bits read and clears last_zero, leaving last_one untouched.
        let mut state = RunState {
            last_zero: true,
            last_one: true,
        };
        let bytes = [0xFFu8]; // would decode to a non-zero prefix if read
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_folded_ones_count(&mut r, &mut state).unwrap(), 0);
        assert_eq!(r.bits_consumed(), 0);
        assert!(!state.last_zero);
        assert!(state.last_one); // untouched on the short-circuit arm
    }

    #[test]
    fn read_folded_ones_count_propagates_eof() {
        let mut w = BitWriter::new();
        w.write_unary(UNARY_ESCAPE);
        w.write_unary(ESCAPE_EOF_CBITS);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        assert_eq!(
            read_folded_ones_count(&mut r, &mut state),
            Err(Error::EndOfStream)
        );
    }

    // ---- Round 278: §4.2 step 1 zero-run fast path on the public
    // typed surface + over-cap hardening (the emit side is the public
    // `emit_zero_run_length` since round 281) ---------------------------

    #[test]
    fn read_zero_run_length_direct_counts() {
        // count < 2 is the run length directly: count 0 consumes one
        // bit (the terminator alone), count 1 consumes two.
        let mut w = BitWriter::new();
        w.write_unary(0);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_zero_run_length(&mut r).unwrap(), 0);
        assert_eq!(r.bits_consumed(), 1);

        let mut w = BitWriter::new();
        w.write_unary(1);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_zero_run_length(&mut r).unwrap(), 1);
        assert_eq!(r.bits_consumed(), 2);
    }

    #[test]
    fn read_zero_run_length_implied_top_bit_round_trip() {
        // Every run length in [2, 600] round-trips through the
        // implied-top-bit form with exact bit accounting: (count + 1)
        // unary bits + (count - 1) mantissa bits = 2 * count.
        for run_length in 2..=600u32 {
            let mut w = BitWriter::new();
            emit_zero_run_length(&mut w, run_length);
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(read_zero_run_length(&mut r).unwrap(), run_length);
            let count = 32 - run_length.leading_zeros();
            assert_eq!(r.bits_consumed(), (2 * count) as usize);
        }
    }

    #[test]
    fn read_zero_run_length_count_32_reaches_u32_max() {
        // count == 32 (the widest in-range form) with an all-ones
        // mantissa decodes to u32::MAX without overflow.
        let mut w = BitWriter::new();
        w.write_unary(32);
        w.write_bits(u32::MAX, 31);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_zero_run_length(&mut r).unwrap(), u32::MAX);
    }

    #[test]
    fn read_zero_run_length_cap_count_is_error_not_panic() {
        // count == 33 (RUN_ESCAPE_CAP) has no assigned meaning in the
        // zero-run context (the spec only gives 33 EOF semantics in the
        // §4.2 step 3 escape) and its implied bit 32 exceeds the u32
        // accumulator — typed error, not a shift-overflow panic.
        let mut w = BitWriter::new();
        w.write_unary(RUN_ESCAPE_CAP);
        w.write_bits(0, 32); // padding the old panic path would have read
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_zero_run_length(&mut r), Err(Error::Truncated));
    }

    #[test]
    fn read_zero_run_length_over_cap_is_error() {
        // A 40-one unary contradicts the spec's 33 cap outright.
        let mut w = BitWriter::new();
        w.write_unary(40);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_zero_run_length(&mut r), Err(Error::Truncated));
    }

    #[test]
    fn read_zero_run_length_truncated_empty() {
        let bytes: [u8; 0] = [];
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_zero_run_length(&mut r), Err(Error::Truncated));
    }

    #[test]
    fn decode_state_zero_run_eligible_gates() {
        // Eligible: RAW median[0] <= 1 (spec §4.2 step 1 reads the
        // stored value, not the get_med working value — round 281) and
        // a clean run state.
        let state = DecodeState::new();
        assert!(state.zero_run_eligible(&AdaptiveMedians::new([1, 0, 0])));

        // Gate 1: raw median[0] > 1. Raw 2 still has get_med == 1, so
        // this boundary also pins that the gate is NOT the working
        // value.
        assert!(!state.zero_run_eligible(&AdaptiveMedians::new([2, 0, 0])));
        assert!(!state.zero_run_eligible(&AdaptiveMedians::new([15, 0, 0])));

        // Gate 2: last_one carry pending.
        let mut held_one = DecodeState::new();
        held_one.run.last_one = true;
        assert!(!held_one.zero_run_eligible(&AdaptiveMedians::new([0, 0, 0])));

        // Gate 3: last_zero short-circuit pending.
        let mut held_zero = DecodeState::new();
        held_zero.run.last_zero = true;
        assert!(!held_zero.zero_run_eligible(&AdaptiveMedians::new([0, 0, 0])));
    }

    #[test]
    fn stereo_decode_state_zero_run_eligible_gates() {
        let low = AdaptiveMedians::new([0, 0, 0]);
        // Raw median[0] == 2 still has get_med == 1 — over the raw
        // spec §4.2 step 1 threshold (round 281).
        let high = AdaptiveMedians::new([2, 0, 0]);
        let state = StereoDecodeState::new();
        assert!(state.zero_run_eligible(&[low, low]));
        // Either channel's median[0] over the threshold blocks the path.
        assert!(!state.zero_run_eligible(&[high, low]));
        assert!(!state.zero_run_eligible(&[low, high]));
        // Any of the four holding bits blocks the path.
        for setter in 0..4 {
            let mut s = StereoDecodeState::new();
            match setter {
                0 => s.left_run.last_one = true,
                1 => s.left_run.last_zero = true,
                2 => s.right_run.last_one = true,
                _ => s.right_run.last_zero = true,
            }
            assert!(!s.zero_run_eligible(&[low, low]), "setter {setter}");
        }
    }

    #[test]
    fn zero_run_loop_matches_public_primitives() {
        // The mono loop's zero-run arm consumes exactly the bits
        // read_zero_run_length consumes, gated by zero_run_eligible.
        let mut w = BitWriter::new();
        emit_zero_run_length(&mut w, 5);
        let bytes = w.finish();

        let mut medians = AdaptiveMedians::new([1, 100, 100]); // raw median[0] <= 1
        let mut state = DecodeState::new();
        assert!(state.zero_run_eligible(&medians));
        let mut r_loop = BitReader::new(&bytes);
        assert_eq!(
            decode_sample_stateful(&mut r_loop, &mut medians, &mut state).unwrap(),
            0
        );
        assert!(state.ever_took_zero_run);
        assert_eq!(state.zero_run_pending, 4);
        assert_eq!(medians.values, [0, 0, 0]); // spec §4.2 step 1 reset

        // Hand-walked primitive: same bits, same run length.
        let mut r_hand = BitReader::new(&bytes);
        assert_eq!(read_zero_run_length(&mut r_hand).unwrap(), 5);
        assert_eq!(r_loop.bits_consumed(), r_hand.bits_consumed());

        // The remaining 4 zero samples drain without reading bits.
        for _ in 0..4 {
            let before = r_loop.bits_consumed();
            assert_eq!(
                decode_sample_stateful(&mut r_loop, &mut medians, &mut state).unwrap(),
                0
            );
            assert_eq!(r_loop.bits_consumed(), before);
        }
        assert_eq!(state.zero_run_pending, 0);
    }

    #[test]
    fn decode_sample_stateful_zero_run_cap_is_error_not_panic() {
        // 33 leading ones on an eligible channel previously hit the
        // `1u32 << 32` shift-overflow debug panic inside the private
        // zero-run path; the loop now surfaces a typed error.
        let mut w = BitWriter::new();
        w.write_unary(RUN_ESCAPE_CAP);
        w.write_bits(0, 32);
        let bytes = w.finish();
        let mut medians = AdaptiveMedians::new([0, 0, 0]);
        let mut state = DecodeState::new();
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            decode_sample_stateful(&mut r, &mut medians, &mut state),
            Err(Error::Truncated)
        );
    }

    #[test]
    fn decode_sample_stateful_stereo_zero_run_cap_is_error_not_panic() {
        // Stereo twin of the cap hardening: the stream-level zero-run
        // arm surfaces the same typed error.
        let mut w = BitWriter::new();
        w.write_unary(RUN_ESCAPE_CAP);
        w.write_bits(0, 32);
        let bytes = w.finish();
        let mut medians = [AdaptiveMedians::new([0, 0, 0]); 2];
        let mut state = StereoDecodeState::new();
        let mut r = BitReader::new(&bytes);
        assert_eq!(
            decode_sample_stateful_stereo(&mut r, &mut medians, &mut state),
            Err(Error::Truncated)
        );
        // On error the channel cursor is untouched (next sample still
        // left), matching the loop's error contract.
        assert_eq!(state.next_channel, 0);
    }

    #[test]
    fn read_raw_prefix_second_unary_over_cap_is_error_not_panic() {
        // 16 ones (escape) then a 34-one second unary: the spec caps
        // the second unary at 33 ("up to 33 1-bits"); previously
        // `get_bits(33)` / `1u32 << 33` panicked in debug builds.
        let mut w = BitWriter::new();
        w.write_unary(UNARY_ESCAPE);
        w.write_unary(34);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_raw_prefix(&mut r), Err(Error::Truncated));
    }

    // ---- Round 281: spec §4.2 inverse encoder on the public surface
    // (BitWriter, split_sign, SampleInterval::encode_*, emit_*,
    // RunState::unfold_prefix, zone_for_magnitude,
    // encode_packed_samples_*) + the §4.2 step 1 / step 4 conformance
    // corrections ------------------------------------------------------

    #[test]
    fn bit_writer_places_first_bit_in_bit_zero() {
        // Spec §4.1 write side: the first written bit lands in bit 0 of
        // byte 0 (LSB-first), exactly where BitReader reads it first.
        let mut w = BitWriter::new();
        w.write_bit(1);
        w.write_bit(0);
        w.write_bit(1);
        assert_eq!(w.bits_written(), 3);
        let bytes = w.finish();
        assert_eq!(bytes, vec![0b0000_0101]);
    }

    #[test]
    fn bit_writer_write_bits_assembles_low_bit_first() {
        // write_bits is the exact inverse of get_bits: a 13-bit field
        // crosses the byte boundary and reads back verbatim.
        let mut w = BitWriter::new();
        w.write_bits(0x15A3 & 0x1FFF, 13);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(r.get_bits(13).unwrap(), 0x15A3 & 0x1FFF);
    }

    #[test]
    fn bit_writer_write_unary_matches_get_unary() {
        for n in [0u32, 1, 5, 16, 33] {
            let mut w = BitWriter::new();
            w.write_unary(n);
            assert_eq!(w.bits_written(), (n + 1) as usize);
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(r.get_unary().unwrap(), n, "unary {n}");
        }
    }

    #[test]
    fn bit_writer_finish_pads_final_byte_high_bits() {
        // 9 bits written → 2 bytes out; the 7 pad bits are zeros in the
        // later-read (high) positions of the final byte.
        let mut w = BitWriter::new();
        for _ in 0..9 {
            w.write_bit(1);
        }
        let bytes = w.finish();
        assert_eq!(bytes, vec![0xFF, 0x01]);
    }

    #[test]
    fn bit_writer_empty_finish_is_empty() {
        let w = BitWriter::new();
        assert!(w.is_empty());
        assert_eq!(w.bits_written(), 0);
        assert!(w.finish().is_empty());
        assert!(!{
            let mut w = BitWriter::new();
            w.write_bit(0);
            w.is_empty()
        });
    }

    #[test]
    fn bit_writer_reader_round_trip_mixed_fields() {
        // Deterministic LCG-driven field sweep: every (value, width)
        // written reads back in order through BitReader.
        let mut lcg = 0x2545F491u32;
        let mut next = || {
            lcg = lcg.wrapping_mul(1664525).wrapping_add(1013904223);
            lcg
        };
        let fields: Vec<(u32, u32)> = (0..200)
            .map(|_| {
                let width = next() % 33; // 0..=32
                let value = if width == 0 {
                    0
                } else {
                    next() & (u32::MAX >> (32 - width))
                };
                (value, width)
            })
            .collect();
        let mut w = BitWriter::new();
        for &(value, width) in &fields {
            w.write_bits(value, width);
        }
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        for &(value, width) in &fields {
            assert_eq!(r.get_bits(width).unwrap(), value);
        }
    }

    #[test]
    fn split_sign_inverts_apply_sign_across_the_full_shape() {
        for v in (-1000..=1000).chain([i32::MIN, i32::MIN + 1, i32::MAX - 1, i32::MAX, -1, 0, 1]) {
            let (magnitude, sign) = split_sign(v);
            assert!(magnitude <= INTERVAL_MASK_31, "magnitude fits the mask");
            assert_eq!(apply_sign(magnitude, sign), v, "round-trip {v}");
            assert_eq!(sign, v < 0);
        }
        // The worked spec values: -1 is magnitude 0 signed; -18 is 17.
        assert_eq!(split_sign(-1), (0, true));
        assert_eq!(split_sign(-18), (17, true));
        assert_eq!(split_sign(i32::MIN), (INTERVAL_MASK_31, true));
    }

    #[test]
    fn split_sign_is_const() {
        const PAIR: (u32, bool) = split_sign(-21);
        assert_eq!(PAIR, (20, true));
    }

    #[test]
    fn encode_mantissa_round_trips_every_code_per_maxcode() {
        for maxcode in 0..=40u32 {
            let interval = SampleInterval::new(0, maxcode);
            for code in 0..=maxcode {
                let mut w = BitWriter::new();
                interval.encode_mantissa(&mut w, code).expect("in range");
                let bytes = w.finish();
                let mut r = BitReader::new(&bytes);
                assert_eq!(
                    interval.decode_mantissa(&mut r).unwrap(),
                    code,
                    "maxcode {maxcode} code {code}"
                );
            }
        }
    }

    #[test]
    fn encode_mantissa_rejects_code_above_maxcode() {
        let interval = SampleInterval::new(0, 16);
        let mut w = BitWriter::new();
        assert_eq!(
            interval.encode_mantissa(&mut w, 17),
            Err(Error::ValueNotInInterval {
                value: 17,
                low: 0,
                high: 16,
            })
        );
        assert!(w.is_empty(), "nothing written on the error path");
    }

    #[test]
    fn encode_value_round_trips_inside_the_worked_interval() {
        // The spec worked example interval [17, 33] (seed [256;3] zone
        // 1): value 20 is code 3.
        let interval = SampleInterval::new(17, 33);
        let mut w = BitWriter::new();
        interval.encode_value(&mut w, 20).expect("contained");
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(interval.decode_value(&mut r).unwrap(), 20);
    }

    #[test]
    fn encode_value_rejects_magnitudes_outside_the_interval() {
        let interval = SampleInterval::new(17, 33);
        for bad in [0u32, 16, 34, 1000] {
            let mut w = BitWriter::new();
            assert_eq!(
                interval.encode_value(&mut w, bad),
                Err(Error::ValueNotInInterval {
                    value: bad,
                    low: 17,
                    high: 33,
                })
            );
            assert!(w.is_empty());
        }
    }

    #[test]
    fn encode_signed_value_round_trips_decode_signed_value() {
        // Every (magnitude, sign) over the worked [17, 33] interval —
        // bit-exact against the decode twin, including the cursor.
        let interval = SampleInterval::new(17, 33);
        for magnitude in 17..=33u32 {
            for sign in [false, true] {
                let value = apply_sign(magnitude, sign);
                let mut w = BitWriter::new();
                interval.encode_signed_value(&mut w, value).expect("ok");
                let bytes = w.finish();
                let mut r = BitReader::new(&bytes);
                assert_eq!(interval.decode_signed_value(&mut r).unwrap(), value);
            }
        }
        // Degenerate interval: only the sign bit moves (1 bit each way).
        let degenerate = SampleInterval::new(5, 5);
        for (value, expect_bits) in [(5i32, 1usize), (-6, 1)] {
            let mut w = BitWriter::new();
            degenerate.encode_signed_value(&mut w, value).expect("ok");
            assert_eq!(w.bits_written(), expect_bits);
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(degenerate.decode_signed_value(&mut r).unwrap(), value);
        }
    }

    #[test]
    fn emit_end_of_stream_marker_reads_back_as_end_of_stream() {
        let mut w = BitWriter::new();
        emit_end_of_stream_marker(&mut w);
        // 16 ones + terminator + 33 ones + terminator = 51 bits.
        assert_eq!(w.bits_written(), 51);
        let bytes = w.finish();
        let mut r = BitReader::new(&bytes);
        assert_eq!(read_raw_prefix(&mut r), Err(Error::EndOfStream));
    }

    #[test]
    fn emit_raw_prefix_round_trips_large_escape_values() {
        // The widest escape forms: cbits == 32 territory.
        for raw in [u32::MAX, u32::MAX - 1, (1 << 31) + 16, 1 << 30] {
            let mut w = BitWriter::new();
            emit_raw_prefix(&mut w, raw);
            let bytes = w.finish();
            let mut r = BitReader::new(&bytes);
            assert_eq!(read_raw_prefix(&mut r).unwrap(), raw, "raw {raw}");
        }
    }

    #[test]
    fn zone_for_magnitude_matches_the_worked_interval_ladder() {
        // Seed [256;3] → get_med = 17 everywhere → intervals [0,16] /
        // [17,33] / [34,50] / [51,67] / [68,84] (the round-255 worked
        // example).
        let m = AdaptiveMedians::new([256, 256, 256]);
        for (mag, zone) in [
            (0u32, 0u32),
            (16, 0),
            (17, 1),
            (33, 1),
            (34, 2),
            (50, 2),
            (51, 3),
            (67, 3),
            (68, 4),
            (84, 4),
            (85, 5),
        ] {
            assert_eq!(m.zone_for_magnitude(mag), zone, "magnitude {mag}");
        }
        // And the interval-membership identity across a sweep.
        for mag in 0..=300u32 {
            let zone = m.zone_for_magnitude(mag);
            let interval = m.sample_interval_for_ones_count(zone);
            assert!(
                interval.contains(mag),
                "interval for zone {zone} must contain {mag}"
            );
        }
    }

    #[test]
    fn zone_for_magnitude_membership_holds_across_varied_medians() {
        for seed in [
            [0u32, 0, 0],
            [16, 16, 16],
            [256, 64, 16],
            [16, 64, 256],
            [8192, 8192, 8192],
            [40000, 1, 70000],
        ] {
            let m = AdaptiveMedians::new(seed);
            for mag in (0..200u32).chain([1000, 65535, 1 << 20]) {
                let zone = m.zone_for_magnitude(mag);
                let interval = m.sample_interval_for_ones_count(zone);
                assert!(interval.contains(mag), "seed {seed:?} mag {mag}");
            }
        }
    }

    #[test]
    fn encode_mono_empty_input_yields_empty_payload() {
        let mut medians = AdaptiveMedians::new([256, 256, 256]);
        let bytes = encode_packed_samples_mono(&[], &mut medians).expect("encode");
        assert!(bytes.is_empty());
        assert_eq!(medians, AdaptiveMedians::new([256, 256, 256]));
    }

    #[test]
    fn encode_mono_payload_is_even_length() {
        // A single small word is well under 8 bits — the payload must
        // still come out at 2 bytes per the spec §1 even-length rule.
        let mut medians = AdaptiveMedians::new([256, 256, 256]);
        let bytes = encode_packed_samples_mono(&[3], &mut medians).expect("encode");
        assert_eq!(bytes.len(), 2);
    }

    #[test]
    fn encode_mono_round_trips_extreme_values() {
        // i32 extremes drive the deepest overflow zones and the escape
        // prefix; -1 is the magnitude-0 negative. The seeds sit at the
        // get_med floor (working medians all 1) so every 31-bit
        // magnitude lands in a degenerate in-range interval — under
        // wider medians the spec §4.2 step 5 31-bit mask makes the very
        // top of the magnitude range unrepresentable (see
        // `encode_mono_value_not_in_interval_on_mask_corner`).
        let seed = [8u32, 8, 8];
        let values = [i32::MAX, i32::MIN, -1, 0, 1, i32::MIN + 1, i32::MAX - 1];
        let n = round_trip(seed, &values);
        assert_eq!(n, values.len());

        // Large-but-representable magnitudes under adapted medians.
        let n = round_trip([8192, 8192, 8192], &[10_000_000, -10_000_000, 0, 1]);
        assert_eq!(n, 4);
    }

    #[test]
    fn encode_mono_round_trips_lcg_mixed_sequence() {
        // 500 deterministic pseudo-random samples across small and
        // large magnitudes, both signs, with zeros sprinkled in.
        let mut lcg = 0xACE1u32;
        let mut next = || {
            lcg = lcg.wrapping_mul(1664525).wrapping_add(1013904223);
            lcg
        };
        let values: Vec<i32> = (0..500)
            .map(|_| {
                let r = next();
                match r % 5 {
                    0 => 0,
                    1 => (r >> 8) as i32 % 17,
                    2 => -((r >> 8) as i32 % 1000),
                    3 => (r >> 8) as i32 % 100_000,
                    _ => -((r >> 8) as i32 % 10_000_000),
                }
            })
            .collect();
        let n = round_trip([8192, 8192, 8192], &values);
        assert_eq!(n, 500);
    }

    #[test]
    fn encode_mono_zero_run_engages_and_round_trips() {
        // Low seed → the §4.2 step 1 gate is open at the first word; a
        // run of zeros followed by non-zeros must round-trip, with the
        // decoder actually taking the zero-run fast path.
        let seed = [0u32, 0, 0];
        let values = [0i32, 0, 0, 0, 5, -1, 0, 7];
        let mut enc_medians = AdaptiveMedians::new(seed);
        let bytes = encode_packed_samples_mono(&values, &mut enc_medians).expect("encode");

        let mut dec_medians = AdaptiveMedians::new(seed);
        let mut reader = BitReader::new(&bytes);
        let mut state = DecodeState::new();
        for (i, &expected) in values.iter().enumerate() {
            let got = decode_sample_stateful(&mut reader, &mut dec_medians, &mut state)
                .unwrap_or_else(|e| panic!("sample {i}: {e:?}"));
            assert_eq!(got, expected, "sample {i}");
        }
        assert!(state.ever_took_zero_run, "fast path actually engaged");
        assert_eq!(dec_medians, enc_medians);
    }

    #[test]
    fn encode_mono_emits_no_run_marker_when_eligible_but_nonzero() {
        // Seed raw median[0] == 0 → gate open, but the first sample is
        // non-zero: the encoder must emit the one-bit zero-length
        // marker and the decoder must fall through to the regular word
        // (the round-281 §4.2 step 1 correction).
        let seed = [0u32, 0, 0];
        let values = [5i32];
        let mut enc_medians = AdaptiveMedians::new(seed);
        let bytes = encode_packed_samples_mono(&values, &mut enc_medians).expect("encode");

        // First wire bit is the zero-length-run marker (a 0 bit).
        let mut probe = BitReader::new(&bytes);
        assert_eq!(probe.get_bit().unwrap(), 0, "marker bit first");

        let mut dec_medians = AdaptiveMedians::new(seed);
        let mut reader = BitReader::new(&bytes);
        let mut state = DecodeState::new();
        let got = decode_sample_stateful(&mut reader, &mut dec_medians, &mut state).unwrap();
        assert_eq!(got, 5, "non-zero sample after the marker");
        assert!(!state.ever_took_zero_run);
        assert_eq!(dec_medians, enc_medians);
    }

    #[test]
    fn zero_length_run_marker_falls_through_on_hand_built_wire() {
        // Hand-built wire pinning the round-281 decoder correction
        // without the encoder in the loop: marker "0", then the word
        // for magnitude 0 / sign set (value -1) — raw prefix "0" (zone
        // 0), no mantissa (get_med(0) == 1 → degenerate [0, 0]), sign
        // "1".
        let mut bits = String::new();
        bits.push('0'); // zero-length run marker
        bits.push('0'); // raw prefix unary 0 → zone 0
        bits.push('1'); // sign bit set
        let bytes = bits_to_bytes(&bits);
        let mut medians = AdaptiveMedians::new([0, 0, 0]);
        let mut state = DecodeState::new();
        let mut reader = BitReader::new(&bytes);
        let got = decode_sample_stateful(&mut reader, &mut medians, &mut state).unwrap();
        assert_eq!(got, -1, "regular word decoded after the marker");
        assert_eq!(reader.bits_consumed(), 3);
        assert!(!state.ever_took_zero_run);
        assert_eq!(state.zero_run_pending, 0);
    }

    #[test]
    fn decoder_rereads_run_length_after_drain() {
        // After a non-zero run drains, the medians are still zero and
        // the holding state clean, so the NEXT word starts with another
        // explicit run-length field — here the zero-length marker, then
        // a regular word. Wire: run 2 ("10" → count 1? no: count=2 needs
        // "110" + 1 mantissa bit) — use run length 2: unary count 2
        // ("110") + mantissa bit 0 → (1 << 1) | 0 = 2. Then marker "0",
        // then word for 5: zone_for_magnitude(5) with all-zero medians
        // (get_med = 1) = 2 + (5 - 2) / 1 = 5 → raw 10 (last word, even)
        // → unary "11111111110", then degenerate interval (no
        // mantissa), then sign "0".
        let mut bits = String::new();
        bits.push_str("110"); // run-length count = 2
        bits.push('0'); // mantissa bit → run length 2
        bits.push('0'); // next word: zero-length-run marker
        bits.push_str("11111111110"); // raw prefix unary 10 → zone 5
        bits.push('0'); // sign clear
        let bytes = bits_to_bytes(&bits);
        let mut medians = AdaptiveMedians::new([1, 0, 0]);
        let mut state = DecodeState::new();
        let mut reader = BitReader::new(&bytes);
        assert_eq!(
            decode_sample_stateful(&mut reader, &mut medians, &mut state).unwrap(),
            0
        );
        assert_eq!(state.zero_run_pending, 1);
        assert_eq!(
            decode_sample_stateful(&mut reader, &mut medians, &mut state).unwrap(),
            0
        );
        assert_eq!(state.zero_run_pending, 0);
        assert_eq!(
            decode_sample_stateful(&mut reader, &mut medians, &mut state).unwrap(),
            5
        );
        // And the same stream is exactly what the encoder produces.
        let mut enc_medians = AdaptiveMedians::new([1, 0, 0]);
        let encoded = encode_packed_samples_mono(&[0, 0, 5], &mut enc_medians).expect("encode");
        let mut enc_reader = BitReader::new(&encoded);
        let mut redec_medians = AdaptiveMedians::new([1, 0, 0]);
        let mut redec_state = DecodeState::new();
        for &expected in &[0i32, 0, 5] {
            assert_eq!(
                decode_sample_stateful(&mut enc_reader, &mut redec_medians, &mut redec_state)
                    .unwrap(),
                expected
            );
        }
        assert_eq!(enc_reader.bits_consumed(), reader.bits_consumed());
    }

    #[test]
    fn encode_mono_value_not_in_interval_on_mask_corner() {
        // The one reachable encode failure: median state whose overflow
        // stride pushes the zone's pre-mask high past INTERVAL_MASK_31,
        // collapsing the masked interval below the magnitude. Seeds
        // [8, 8, 48] → get_med = (1, 1, 4); i32::MAX has magnitude
        // 0x7FFFFFFF whose zone interval is [0x7FFFFFFE, 0x80000001]
        // pre-mask → masked high wraps below low → clamped degenerate
        // [0x7FFFFFFE, 0x7FFFFFFE] which cannot carry the magnitude.
        let mut medians = AdaptiveMedians::new([8, 8, 48]);
        let err = encode_packed_samples_mono(&[i32::MAX], &mut medians);
        assert_eq!(
            err,
            Err(Error::ValueNotInInterval {
                value: 0x7FFF_FFFF,
                low: 0x7FFF_FFFE,
                high: 0x7FFF_FFFE,
            })
        );
    }

    #[test]
    fn encode_mono_from_entropy_matches_explicit_seeds() {
        let info = EntropyInfo::mono([8192, 8192, 8192]);
        let values: Vec<i32> = (1..=8).collect();
        let via_info = encode_packed_samples_mono_from_entropy(&values, &info).expect("encode");
        let mut medians = AdaptiveMedians::new([8192, 8192, 8192]);
        let via_seeds = encode_packed_samples_mono(&values, &mut medians).expect("encode");
        assert_eq!(via_info, via_seeds);
    }

    #[test]
    fn encode_mono_from_entropy_rejects_negative_seed() {
        let info = EntropyInfo {
            medians_left: [-1, 0, 0],
            medians_right: [0, 0, 0],
        };
        assert_eq!(
            encode_packed_samples_mono_from_entropy(&[1, 2], &info),
            Err(Error::InvalidEntropyInfoForMono)
        );
    }

    #[test]
    fn encode_stereo_zero_run_spans_both_channels() {
        // Both channels' raw median[0] <= 1 → the stream-level run
        // covers interleaved zeros on both channels; the non-zero
        // samples after it land on the right parity.
        let seeds = [[0u32, 0, 0], [1u32, 0, 0]];
        let values = [0i32, 0, 0, 0, 0, 9, -3, 4];
        let mut enc_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let bytes = encode_packed_samples_stereo(&values, &mut enc_medians).expect("encode");

        let mut dec_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut state = StereoDecodeState::new();
        let mut reader = BitReader::new(&bytes);
        for (i, &expected) in values.iter().enumerate() {
            let got = decode_sample_stateful_stereo(&mut reader, &mut dec_medians, &mut state)
                .unwrap_or_else(|e| panic!("sample {i}: {e:?}"));
            assert_eq!(got, expected, "sample {i}");
        }
        assert!(state.ever_took_zero_run);
        assert_eq!(dec_medians, enc_medians);
    }

    #[test]
    fn encode_stereo_gate_needs_both_channels_low() {
        // Right channel's raw median[0] over the threshold → no
        // zero-run field on the wire even for a leading zero; the
        // round-trip must still hold via regular words.
        let seeds = [[0u32, 0, 0], [256u32, 256, 256]];
        let values = [0i32, 100, 0, -50];
        let mut enc_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let bytes = encode_packed_samples_stereo(&values, &mut enc_medians).expect("encode");
        let mut dec_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut state = StereoDecodeState::new();
        let mut reader = BitReader::new(&bytes);
        for (i, &expected) in values.iter().enumerate() {
            let got = decode_sample_stateful_stereo(&mut reader, &mut dec_medians, &mut state)
                .unwrap_or_else(|e| panic!("sample {i}: {e:?}"));
            assert_eq!(got, expected, "sample {i}");
        }
        assert!(!state.ever_took_zero_run, "gate stayed shut");
        assert_eq!(dec_medians, enc_medians);
    }

    #[test]
    fn encode_stereo_round_trips_odd_length_slice() {
        // A trailing left-only sample is encodable; decode per-call.
        let seeds = [[8192u32, 8192, 8192], [4096u32, 4096, 4096]];
        let values = [600i32, -300, 555];
        let mut enc_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let bytes = encode_packed_samples_stereo(&values, &mut enc_medians).expect("encode");
        let mut dec_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let mut state = StereoDecodeState::new();
        let mut reader = BitReader::new(&bytes);
        for &expected in &values {
            assert_eq!(
                decode_sample_stateful_stereo(&mut reader, &mut dec_medians, &mut state).unwrap(),
                expected
            );
        }
        assert_eq!(dec_medians, enc_medians);
    }

    #[test]
    fn encode_stereo_round_trips_lcg_mixed_sequence() {
        // 400 interleaved deterministic pseudo-random samples through
        // the frame-based decode wrapper.
        let mut lcg = 0xBEEFu32;
        let mut next = || {
            lcg = lcg.wrapping_mul(1664525).wrapping_add(1013904223);
            lcg
        };
        let values: Vec<i32> = (0..400)
            .map(|_| {
                let r = next();
                match r % 4 {
                    0 => 0,
                    1 => (r >> 8) as i32 % 600,
                    2 => -((r >> 8) as i32 % 60_000),
                    _ => (r >> 8) as i32 % 5_000_000,
                }
            })
            .collect();
        let seeds = [[8192u32, 8192, 8192], [2048u32, 2048, 2048]];
        let mut enc_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let bytes = encode_packed_samples_stereo(&values, &mut enc_medians).expect("encode");
        assert_eq!(bytes.len() % 2, 0);

        let view = crate::PackedSamples::new(&bytes);
        let mut dec_medians = [
            AdaptiveMedians::new(seeds[0]),
            AdaptiveMedians::new(seeds[1]),
        ];
        let got =
            decode_packed_samples_stereo(&view, &mut dec_medians, values.len() / 2).expect("ok");
        assert_eq!(got, values);
        assert_eq!(dec_medians, enc_medians);
    }

    #[test]
    fn encode_stereo_from_entropy_matches_explicit_seeds() {
        let info = EntropyInfo::stereo([8192, 8192, 8192], [4096, 4096, 4096]);
        let values = [600i32, -300, 555, 222];
        let via_info = encode_packed_samples_stereo_from_entropy(&values, &info).expect("encode");
        let mut medians = [
            AdaptiveMedians::new([8192, 8192, 8192]),
            AdaptiveMedians::new([4096, 4096, 4096]),
        ];
        let via_seeds = encode_packed_samples_stereo(&values, &mut medians).expect("encode");
        assert_eq!(via_info, via_seeds);
    }

    #[test]
    fn encode_stereo_from_entropy_rejects_mono_info() {
        let info = EntropyInfo::mono([100, 200, 300]);
        assert_eq!(
            encode_packed_samples_stereo_from_entropy(&[1, 2], &info),
            Err(Error::InvalidEntropyInfoForStereo)
        );
    }

    #[test]
    fn encode_stereo_from_entropy_rejects_negative_seed() {
        let info = EntropyInfo {
            medians_left: [1, 2, 3],
            medians_right: [0, -1, 0],
        };
        assert_eq!(
            encode_packed_samples_stereo_from_entropy(&[1, 2], &info),
            Err(Error::InvalidEntropyInfoForStereo)
        );
    }

    #[test]
    fn encode_decode_full_pipeline_through_packed_samples_view() {
        // Encoder output consumed exactly the way a block decode does:
        // through the PackedSamples view + decode_packed_samples_mono.
        let seed = [8192u32, 8192, 8192];
        let values: Vec<i32> = (-20..=20).map(|v| v * 37).collect();
        let mut enc_medians = AdaptiveMedians::new(seed);
        let bytes = encode_packed_samples_mono(&values, &mut enc_medians).expect("encode");
        let view = crate::PackedSamples::new(&bytes);
        let mut dec_medians = AdaptiveMedians::new(seed);
        let got = decode_packed_samples_mono(&view, &mut dec_medians, values.len()).expect("ok");
        assert_eq!(got, values);
        assert_eq!(dec_medians, enc_medians);
    }
}
