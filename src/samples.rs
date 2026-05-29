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
}
