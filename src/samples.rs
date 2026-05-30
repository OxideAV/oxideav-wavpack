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

/// Adaptive run-state carried between successive [`decode_run_length`]
/// calls, mirroring the wiki's `last_zero` / `last_one` locals.
///
/// The wiki keeps two booleans across samples: after a sample whose
/// run-length index `n` was odd it sets `last_one`, and the next call
/// is forced to `n = 0` (via `last_zero`). [`RunState`] packages that
/// carry so a caller can decode a sequence of run-lengths from one
/// [`BitReader`].
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
}

/// Decode the run-length index `n` for one sample, advancing both the
/// bit reader and the adaptive [`RunState`].
///
/// This is the first half of the wiki "Samples coding" pseudocode —
/// the part that turns the unary prefix (with the `n == 16` escape)
/// into the halved run-length index and updates `last_zero` /
/// `last_one`:
///
/// ```text
/// if(last_zero){ n = 0; last_zero = 0; }
/// else{
///   n = get_unary();
///   if(n == 16){
///     n2 = get_unary();
///     if(n2 < 2) n += n2;
///     else       n += (1 << (n2-1)) | getbits(n2-1);
///   }
///   last_one = n & 1;
///   if(last_one) n = (n>>1) + 1;
///   else         n = n >> 1;
///   last_zero = !last_one;
/// }
/// ```
///
/// The returned `n` is the run-length index the (deferred) second half
/// of the pseudocode maps onto a Golomb `(base, add)` interval. The
/// escape `getbits(n2 - 1)` is only reached when `n2 >= 2`, so the
/// argument is always `>= 1`.
pub fn decode_run_length(reader: &mut BitReader<'_>, state: &mut RunState) -> Result<u32> {
    if state.last_zero {
        state.last_zero = false;
        // The wiki leaves last_one untouched on this branch; it is only
        // (re)assigned inside the else arm. Mirror that exactly.
        return Ok(0);
    }

    let mut n = reader.get_unary()?;
    if n == UNARY_ESCAPE {
        let n2 = reader.get_unary()?;
        if n2 < 2 {
            n += n2;
        } else {
            // getbits(n2 - 1): n2 >= 2 here so the argument is >= 1.
            let mantissa = reader.get_bits(n2 - 1)?;
            n += (1u32 << (n2 - 1)) | mantissa;
        }
    }

    let last_one = (n & 1) != 0;
    n = if last_one { (n >> 1) + 1 } else { n >> 1 };
    state.last_one = last_one;
    state.last_zero = !last_one;
    Ok(n)
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
fn read_folded_ones_count(reader: &mut BitReader<'_>, state: &mut RunState) -> Result<u32> {
    // Wiki short-circuit: when last_zero is set, this sample's
    // ones_count is 0 with no bits read; last_zero clears and
    // last_one is untouched. Matches decode_run_length's first branch.
    if state.last_zero {
        state.last_zero = false;
        return Ok(0);
    }

    let raw = reader.get_unary()?;
    let raw_value = if raw < UNARY_ESCAPE {
        raw
    } else {
        // Spec §4.2 step 3: escape arm. cbits up to 33; cbits == 33 is EOF.
        let cbits = reader.get_unary()?;
        if cbits == ESCAPE_EOF_CBITS {
            return Err(Error::EndOfStream);
        }
        if cbits < 2 {
            UNARY_ESCAPE + cbits
        } else {
            // cbits >= 2: read cbits - 1 mantissa bits LSB-first, top
            // bit implied set, then add LIMIT_ONES back in (spec §4.2
            // step 3, "then add `LIMIT_ONES` back in").
            let mantissa = reader.get_bits(cbits - 1)?;
            let escape_value = (1u32 << (cbits - 1)) | mantissa;
            UNARY_ESCAPE + escape_value
        }
    };

    // Spec §4.2 step 4 fold.
    let last_one_bit = (raw_value & 1) != 0;
    let ones_count = if last_one_bit {
        (raw_value >> 1) + 1
    } else {
        raw_value >> 1
    };
    state.last_one = last_one_bit;
    state.last_zero = !last_one_bit;
    Ok(ones_count)
}

/// Spec §4.2 step 5: form the `(low, high)` value interval from a
/// channel's three working medians and the (folded) `ones_count` zone.
///
/// Working medians come from [`AdaptiveMedians::get_med`]. The result
/// is masked to 31 bits per the spec; `high` is clamped up to `low`
/// when the mask underflows the interval (which can happen for
/// pathological median sets but is structurally rare on real fixtures).
fn form_interval(medians: &AdaptiveMedians, ones_count: u32) -> (u32, u32) {
    let m0 = medians.get_med(0);
    let m1 = medians.get_med(1);
    let m2 = medians.get_med(2);
    let (mut low, mut high) = match ones_count {
        0 => (0u32, m0.wrapping_sub(1)),
        1 => (m0, m0.wrapping_add(m1).wrapping_sub(1)),
        2 => (
            m0.wrapping_add(m1),
            m0.wrapping_add(m1).wrapping_add(m2).wrapping_sub(1),
        ),
        n => {
            // n >= 3: low = m0 + m1 + (n - 2) * m2; high = low + m2 - 1.
            let extra = m2.wrapping_mul(n - 2);
            let base = m0.wrapping_add(m1).wrapping_add(extra);
            (base, base.wrapping_add(m2).wrapping_sub(1))
        }
    };
    low &= INTERVAL_MASK_31;
    high &= INTERVAL_MASK_31;
    if high < low {
        high = low;
    }
    (low, high)
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

/// Spec §4.2 step 1 attempt: when the channel's `median[0]` is `<= 1`
/// AND no holding state is pending (no `last_one` carry, no
/// `last_zero` short-circuit waiting), the stream may carry an explicit
/// zero-run.
///
/// Returns:
/// * `Ok(Some(0))` when a non-zero run was decoded — the call emits a
///   `0` sample, [`DecodeState::zero_run_pending`] is set to
///   `run_length - 1`, and the medians are zeroed per spec §4.2 step 1.
/// * `Ok(None)` when the path was not entered (medians not <= 1, or
///   holding state pending), so the caller proceeds to the normal
///   prefix-decode path.
/// * `Ok(Some(0))` is ALSO emitted on an explicit zero-length run (the
///   unary prefix decodes to 0) — that is the spec's "no zero run here"
///   signal, written into the stream when the encoder wanted to clear
///   the zero-run fast path's eligibility without emitting any zero
///   samples; the next call reads a normal sample value. In that case
///   `zero_run_pending` is left at `0` so the caller returns the `0`
///   sample (the explicit-zero-prefix's documented behaviour is to emit
///   the single `0` sample).
fn try_zero_run_path(
    reader: &mut BitReader<'_>,
    medians: &mut AdaptiveMedians,
    state: &mut DecodeState,
) -> Result<Option<i32>> {
    // Spec §4.2 step 1 eligibility: median[0] <= 1 AND no holding-bit
    // pending. For mono "both channels" reduces to the one channel.
    if medians.get_med(0) > 1 {
        return Ok(None);
    }
    if state.run.last_one || state.run.last_zero {
        return Ok(None);
    }

    // Read the zero-run unary, capped at RUN_ESCAPE_CAP per spec.
    let count = reader.get_unary()?;
    if count > RUN_ESCAPE_CAP {
        // Defensive: a >33 run from a well-formed stream contradicts
        // the spec cap. Treat as truncated to fail loudly rather than
        // silently truncating.
        return Err(Error::Truncated);
    }
    let run_length = if count < 2 {
        count
    } else {
        // Read count-1 LSB-first mantissa bits with the top bit implied
        // set, exactly as the spec §4.2 step 1 prose specifies.
        let mantissa = reader.get_bits(count - 1)?;
        (1u32 << (count - 1)) | mantissa
    };

    if run_length > 0 {
        // Non-zero run: spec §4.2 step 1 — "resets both channels'
        // medians to zero and emits a `0` sample." Mono: the one
        // channel.
        medians.values = [0, 0, 0];
        state.ever_took_zero_run = true;
        // run_length samples total are zero; we are emitting the first
        // here, so owe run_length - 1 more on subsequent calls.
        state.zero_run_pending = run_length - 1;
    }
    // Whether the run was zero or non-zero, the call's emitted sample
    // is `0`. (count == 0 case: the encoder said "no zero run", but the
    // spec path STILL emits one zero sample per the §4.2 step 1 prose;
    // the next call resumes the normal prefix path. This is the wiki
    // mapping where `0` is the legal sentinel.)
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
/// 2. **Zero-run fast path** (spec §4.2 step 1) when eligible
///    (`medians.get_med(0) <= 1` AND no holding bits): try to decode an
///    explicit run length; emit a `0` sample on success.
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

    // 5. Form the interval (spec §4.2 step 5).
    let (low, high) = form_interval(medians, ones_count);

    // 6. Adapt the medians (spec §3.2) — BEFORE the mantissa read, per
    // the spec note "The medians are adapted at this point".
    medians.adapt(Zone::from_ones_count(ones_count));

    // 7. Mantissa (spec §4.2 step 6 first paragraph).
    let maxcode = high.wrapping_sub(low);
    let code = read_truncated_binary(reader, maxcode)?;
    let magnitude = low.wrapping_add(code);

    // 8. Sign (spec §4.2 step 7). The result is built in i32 space:
    // magnitude can be up to INTERVAL_MASK_31 (2^31 - 1), which fits.
    let sign = reader.get_bit()?;
    let mid = magnitude as i32;
    let result = if sign == 0 { mid } else { !mid };
    Ok(result)
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
    fn run_length_odd_unary_rounds_up_and_sets_last_one() {
        // unary = 3 (bits "1110"). n & 1 == 1 → last_one = true,
        // n = (3 >> 1) + 1 = 1 + 1 = 2, last_zero = false.
        let bytes = bits_to_bytes("1110");
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 2);
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn run_length_escape_small_n2_adds_directly() {
        // unary = 16 triggers the escape. Build sixteen ones + a zero
        // (the first get_unary = 16), then a second unary with n2 = 1
        // (bits "10"). n2 < 2 → n += n2 → n = 17.
        // Then last_one = 17 & 1 = 1 → n = (17 >> 1) + 1 = 8 + 1 = 9.
        let mut bits = String::new();
        bits.push_str(&"1".repeat(16));
        bits.push('0'); // terminator for the first unary = 16
        bits.push_str("10"); // second unary n2 = 1
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 9);
        assert!(state.last_one);
        assert!(!state.last_zero);
    }

    #[test]
    fn run_length_escape_large_n2_reads_mantissa() {
        // unary = 16, then n2 = 3 (bits "1110"), then getbits(n2-1) =
        // getbits(2). Mantissa bits "10" → first bit '1' lands in bit 0,
        // second bit '0' in bit 1 → value = 1<<0 | 0<<1 = 1.
        // n += (1 << (3-1)) | 1 = (1<<2) | 1 = 4 | 1 = 5 → n = 16 + 5 = 21.
        // last_one = 21 & 1 = 1 → n = (21 >> 1) + 1 = 10 + 1 = 11.
        let mut bits = String::new();
        bits.push_str(&"1".repeat(16));
        bits.push('0'); // first unary = 16
        bits.push_str("1110"); // second unary n2 = 3
        bits.push_str("10"); // getbits(2) mantissa = 1 (LSB-first)
        let bytes = bits_to_bytes(&bits);
        let mut r = BitReader::new(&bytes);
        let mut state = RunState::new();
        let n = decode_run_length(&mut r, &mut state).unwrap();
        assert_eq!(n, 11);
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
        // Sample 1: unary "110" = 2 → even → n=1, last_zero=true.
        // Sample 2: last_zero → n=0, no bits consumed, last_zero=false.
        // Sample 3: unary "1110" = 3 → odd → n=2, last_one=true,
        //           last_zero=false.
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
        assert_eq!(n3, 2);
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
        // Stream: unary "1110" → run-length raw n=3 (odd) → n = (3>>1)+1
        // = 2. Then interval for n=2 with medians [10,20,30]: base=30,
        // add=29, k=5, ex=2. getbits(4)="1000"=1 (< ex), sign "0" →
        // result = 31.
        let bits = "1110" /* unary=3 → n=2 */ .to_string()
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
        // unary "1110" → raw n=3 (odd) → n=2; interval for n=2 with
        // medians [10,20,30]: base=30, add=29, k=5, ex=2;
        // getbits(4)="1000"=1 (< ex), sign "0" → result = 31.
        let bits = "1110".to_string() + "1000" + "0";
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
        let bits = "1110".to_string() + "1000" + "0";
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
    // The tests in this block use a spec-derived inverse encoder
    // (`encode_one_sample`) to produce a `0x0A`-shape bitstream from a
    // fixed PCM sequence, then decode it back through
    // `decode_sample_stateful` / `decode_packed_samples_mono` and assert
    // every sample reconstructs bit-for-bit. The encoder is the
    // bit-for-bit INVERSE of the decoder spec (§4.2 read order: unary
    // prefix → mantissa → sign; spec §3.2 zone adaptation interleaved
    // BEFORE the mantissa), so a successful round-trip pins both halves
    // to the spec text.
    //
    // The encoder is also `#[cfg(test)]`-only — production code only
    // ships the decode side, since real WavPack `0x0A` bytes come from
    // an external encoder. The round-trip is the closure proof.

    /// Push a single bit (LSB-first into the running byte) onto an
    /// encoder buffer. Mirrors `bits_to_bytes` but for incremental
    /// emission instead of a one-shot bit string.
    #[derive(Debug, Default)]
    struct BitWriter {
        bytes: Vec<u8>,
        cur: u8,
        nbits: u8,
    }

    impl BitWriter {
        fn new() -> Self {
            Self::default()
        }

        fn write_bit(&mut self, bit: u32) {
            self.cur |= ((bit & 1) as u8) << self.nbits;
            self.nbits += 1;
            if self.nbits == 8 {
                self.bytes.push(self.cur);
                self.cur = 0;
                self.nbits = 0;
            }
        }

        fn write_bits(&mut self, value: u32, count: u32) {
            for i in 0..count {
                self.write_bit((value >> i) & 1);
            }
        }

        /// Write `n` `1` bits followed by a single `0` terminator.
        fn write_unary(&mut self, n: u32) {
            for _ in 0..n {
                self.write_bit(1);
            }
            self.write_bit(0);
        }

        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.bytes.push(self.cur);
            }
            self.bytes
        }
    }

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

    /// Pick the post-fold `ones_count` for a non-negative magnitude
    /// `mag` given the channel's current medians, mirroring spec §4.2
    /// step 5 in reverse.
    ///
    /// Returns `(ones_count, low)` where `low` is the interval base
    /// and `code = mag - low` is the mantissa to emit through
    /// [`emit_truncated_binary`].
    fn pick_zone_for_magnitude(medians: &AdaptiveMedians, mag: u32) -> (u32, u32) {
        let m0 = medians.get_med(0);
        let m1 = medians.get_med(1);
        let m2 = medians.get_med(2);
        if mag < m0 {
            (0, 0)
        } else if mag < m0 + m1 {
            (1, m0)
        } else {
            // n >= 2: each subsequent zone adds m2 to the base.
            let above = mag - (m0 + m1);
            let extra = above / m2;
            (2 + extra, m0 + m1 + extra * m2)
        }
    }

    /// Pick a raw unary count whose fold yields the requested
    /// `ones_count` AND whose carry (`last_one` / `last_zero`) is
    /// compatible with the encoder's intent. Returns `raw` to write
    /// (with `raw + 1` bits on the wire — `raw` `1` bits + a single `0`
    /// terminator).
    ///
    /// `prefer_last_one` picks between the two raw choices for a given
    /// `ones_count > 0`: odd raw (= 2k-1, sets last_one) vs even raw
    /// (= 2k, sets last_zero and pre-encodes the NEXT sample as zone 0).
    /// For tests where we want a long sequence of independent zone
    /// reads, use `prefer_last_one = true` (odd raw) — every sample
    /// reads its own prefix and adjacent samples don't tangle.
    fn pick_raw_unary(ones_count: u32, prefer_last_one: bool) -> u32 {
        if ones_count == 0 {
            // Only legal raw is 0 (even, last_zero=true). Forces the
            // NEXT sample to short-circuit to ones_count=0.
            0
        } else if prefer_last_one {
            2 * ones_count - 1
        } else {
            2 * ones_count
        }
    }

    /// Emit one sample value through the spec-derived inverse encoder.
    /// MUTATES `medians` per spec §3.2 EXACTLY like the decoder does,
    /// so encoder + decoder walk identical median trajectories.
    ///
    /// `value` is the signed sample to encode. `prefer_last_one`
    /// controls the raw-unary parity tie-break (see `pick_raw_unary`).
    fn encode_one_sample(
        w: &mut BitWriter,
        medians: &mut AdaptiveMedians,
        state: &mut RunState,
        value: i32,
        prefer_last_one: bool,
    ) {
        // Decoder reconstruction: sign=0 → mid = magnitude; sign=1 →
        // mid = !magnitude → signed value = -(magnitude+1). So:
        let (sign_bit, magnitude) = if value >= 0 {
            (0u32, value as u32)
        } else {
            // value < 0 → returned by decoder as !magnitude; so
            // value = !magnitude → magnitude = !value as u32 (because
            // !value as i32 == !(value)).
            (1u32, (!value) as u32)
        };

        if state.last_zero {
            // Encoder MUST emit ones_count = 0 on this sample — the
            // previous even-raw pre-committed us. The magnitude must
            // fit zone 0 (i.e. magnitude < m0). If the caller picked a
            // value outside that interval the sequence is illegal; the
            // test helper panics so contributors notice immediately.
            let (ones_count, low) = pick_zone_for_magnitude(medians, magnitude);
            assert_eq!(
                ones_count, 0,
                "encoder: last_zero pre-commits ones_count=0 but magnitude {magnitude} lands in zone {ones_count}",
            );
            state.last_zero = false;
            // No unary written; the decoder short-circuits.
            // Adapt + mantissa + sign exactly as the non-short-circuit
            // path does (spec §3.2 + §4.2 step 6 + step 7).
            let (low_chk, high) = form_interval(medians, ones_count);
            assert_eq!(low_chk, low);
            medians.adapt(Zone::from_ones_count(ones_count));
            let maxcode = high.wrapping_sub(low);
            let code = magnitude - low;
            assert!(code <= maxcode, "encoder: code {code} > maxcode {maxcode}");
            emit_truncated_binary(w, maxcode, code);
            w.write_bit(sign_bit);
            return;
        }

        // Normal path: pick the zone, the raw unary, emit prefix +
        // mantissa + sign, then adapt.
        let (ones_count, low) = pick_zone_for_magnitude(medians, magnitude);
        let raw = pick_raw_unary(ones_count, prefer_last_one);
        w.write_unary(raw);
        // Update encoder state to match the decoder fold.
        let last_one_bit = (raw & 1) != 0;
        state.last_one = last_one_bit;
        state.last_zero = !last_one_bit;
        let (low_chk, high) = form_interval(medians, ones_count);
        assert_eq!(low_chk, low);
        medians.adapt(Zone::from_ones_count(ones_count));
        let maxcode = high.wrapping_sub(low);
        let code = magnitude - low;
        assert!(code <= maxcode, "encoder: code {code} > maxcode {maxcode}");
        emit_truncated_binary(w, maxcode, code);
        w.write_bit(sign_bit);
    }

    /// Round-trip a sequence of sample values through encode + decode
    /// against a fresh `AdaptiveMedians` seed and assert every
    /// reconstructed sample matches the input.
    ///
    /// Returns the number of samples (== `values.len()`) so callers
    /// can aggregate a sample-exact count across multiple round-trips.
    fn round_trip(seed: [u32; 3], values: &[i32]) -> usize {
        // Encode.
        let mut enc_medians = AdaptiveMedians::new(seed);
        let mut enc_state = RunState::new();
        let mut w = BitWriter::new();
        for &v in values {
            encode_one_sample(&mut w, &mut enc_medians, &mut enc_state, v, true);
        }
        let bytes = w.finish();

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
            let (ones_count, _) = pick_zone_for_magnitude(&sim_medians, mag as u32);
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

        // Encode (matching the round_trip helper).
        let mut enc_medians = AdaptiveMedians::new(seed);
        let mut enc_state = RunState::new();
        let mut w = BitWriter::new();
        for &v in &values {
            encode_one_sample(&mut w, &mut enc_medians, &mut enc_state, v, true);
        }
        let bytes = w.finish();

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
        // Encoding `0` with last_zero==false picks ones_count=0,
        // raw=0 (the only legal raw for ones_count=0), which sets
        // last_zero=true. The encoder then expects the next sample to
        // also be zone-0 — the assertion in encode_one_sample's
        // last_zero branch catches a caller that violates this. So
        // sequence two zeros to walk the short-circuit explicitly.
        let seed = [256u32, 256, 256];
        let values: Vec<i32> = vec![0, 0, 0, 0];
        let n = round_trip(seed, &values);
        assert_eq!(n, 4);
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
}
