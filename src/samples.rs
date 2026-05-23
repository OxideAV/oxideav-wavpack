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
//! Round 5 lands the first half — the bit reader and the `n`-decoder —
//! plus the [`BitReader`] primitives the second half will reuse. The
//! second half is **not** implemented this round because the wiki does
//! not quantify its "increase" / "decrease" verbs (see the docs-gap
//! note below); decoding a real `0x0A` stream is gated on that gap
//! being closed.
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
//! ### Docs-gap: median adaptation amount (second half, deferred)
//!
//! The wiki's "increase median[0]" / "decrease median[0]" steps name a
//! direction but not an amount. Real WavPack updates each median by a
//! fraction of itself (a documented `±(median/128 + …)` style step in
//! the format's own `format.txt`, which the wiki cites but does **not**
//! reproduce), so the per-sample reconstruction cannot be made
//! bit-exact from this wiki page alone. Round 5 therefore stops at the
//! `n`-decoder. The second half — `(base, add)` interval selection,
//! `getbits(k-1)` mantissa, sign, and median adaptation — needs the
//! adaptation formula added to `docs/audio/wavpack/` before it can be
//! implemented without guessing.
//!
//! ### Docs-gap: escape `getbits(n2 - 1)` when `n2 < 2`
//!
//! The escape branch reads `getbits(n2 - 1)` only inside the
//! `else` arm (`n2 >= 2`), so the argument is always `>= 1` there. The
//! `n2 < 2` arm adds `n2` directly with no `getbits` call. Round 5
//! mirrors that control flow exactly, so `get_bits(0)` is never invoked
//! from the escape path; [`BitReader::get_bits`] still defines
//! `get_bits(0) == 0` for completeness.

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
}
