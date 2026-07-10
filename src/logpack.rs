//! WavPack log-domain conversions — `wp_log2` and `wp_exp2s`.
//!
//! WavPack stores several 32-bit quantities in a **logarithmic domain**
//! so they compress well and fit a 16-bit field: the `0x05` entropy
//! medians, the `0x04` decorrelation history seeds, and internal level
//! trackers. The stored form is a signed 16-bit "log word"; this module
//! implements the two integer conversions the staged spec doc
//! `docs/audio/wavpack/spec/wavpack-log2-exp2.md` defines:
//!
//! * [`wp_log2`] — unsigned magnitude → integer log with 8 fractional
//!   bits (§3): to 8-bit precision `wp_log2(v) ≈ 256 * (log2(v) + 1)`,
//!   with `wp_log2(0) = 0` (§6 canonical-zero pin). Maximum input is
//!   about `0xff800000`; maximum result is `8447`.
//! * [`wp_exp2s`] — signed log word → signed 32-bit value (§4): the
//!   exact inverse, `wp_exp2s(l) ≈ 2^((l - 256) / 256)` for `l > 0`,
//!   odd (`wp_exp2s(-l) == -wp_exp2s(l)`), with `wp_exp2s(0) = 0`.
//!
//! Both directions share the same `+256` / `-256` offset convention
//! (the integer part of the log is the value's **bit-length**, not its
//! MSB position — spec §1), so the offset cancels: a value logged with
//! `wp_log2` is always recovered with `wp_exp2s`.
//!
//! ## The lookup tables (spec §2)
//!
//! Two 256-entry byte tables staged as CSV data under
//! `docs/audio/wavpack/tables/`:
//!
//! * [`LOG2_TABLE`] (`wp-log2.csv`) —
//!   `log2_table[i] = round(256 * log2(1 + i/256))`, the fractional
//!   log-bits of a mantissa normalised to `[1, 2)` whose low 8 bits
//!   are `i`.
//! * [`EXP2_TABLE`] (`wp-exp2.csv`) —
//!   `exp2_table[i] = round(256 * 2^(i/256)) - 256`, the low 8 bits of
//!   the 9-bit mantissa (the implicit leading bit `0x100` is ORed in
//!   at use).
//!
//! A third helper, `nbits` (the bit-length of a byte), is trivially
//! derivable (`wp-log2-exp2.meta.md`) and is computed via
//! `u32::leading_zeros` here rather than stored.
//!
//! ## Where the conversions are used in decode (spec §5)
//!
//! | Log word source       | Field                | Decoder step                    |
//! | --------------------- | -------------------- | ------------------------------- |
//! | `0x05` entropy vars   | 3 medians / channel  | each median = `wp_exp2s(word)`  |
//! | `0x04` decorr samples | per-pass seeds       | each seed = `wp_exp2s(word)`    |
//!
//! Each stored 16-bit field is little-endian and **sign-extended** (it
//! is a signed log word) before expansion.
//!
//! ## Canonical zero (spec §6, issue #204 erratum pin)
//!
//! The log word `0x0000` canonically represents the value `0`, not
//! `2^(0/256)`. Both directions special-case it — `wp_log2(0) = 0` and
//! `wp_exp2s(0) = 0` — and the zero behaviour falls out of the table
//! arithmetic (`exp2_table[0] | 0x100 = 0x100`, shifted right by 9,
//! gives 0), but a naive "always `2^(l/256)`" reading diverges here.

/// Fractional log-bits table — `log2_table[i] = round(256 * log2(1 + i/256))`.
///
/// Mechanically transcribed from the staged CSV
/// `docs/audio/wavpack/tables/wp-log2.csv` (256 byte entries; anchors
/// `00 01 03 04 06 07 09 0a … ff` per `wp-log2-exp2.meta.md`).
pub const LOG2_TABLE: [u8; 256] = [
    0x00, 0x01, 0x03, 0x04, 0x06, 0x07, 0x09, 0x0a, 0x0b, 0x0d, 0x0e, 0x10, 0x11, 0x12, 0x14, 0x15,
    0x16, 0x18, 0x19, 0x1a, 0x1c, 0x1d, 0x1e, 0x20, 0x21, 0x22, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2a,
    0x2c, 0x2d, 0x2e, 0x2f, 0x31, 0x32, 0x33, 0x34, 0x36, 0x37, 0x38, 0x39, 0x3b, 0x3c, 0x3d, 0x3e,
    0x3f, 0x41, 0x42, 0x43, 0x44, 0x45, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4d, 0x4e, 0x4f, 0x50, 0x51,
    0x52, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63,
    0x64, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85,
    0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95,
    0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4,
    0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb2,
    0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc0,
    0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcb, 0xcc, 0xcd, 0xce,
    0xcf, 0xd0, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd8, 0xd9, 0xda, 0xdb,
    0xdc, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe4, 0xe5, 0xe6, 0xe7, 0xe7,
    0xe8, 0xe9, 0xea, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xee, 0xef, 0xf0, 0xf1, 0xf1, 0xf2, 0xf3, 0xf4,
    0xf4, 0xf5, 0xf6, 0xf7, 0xf7, 0xf8, 0xf9, 0xf9, 0xfa, 0xfb, 0xfc, 0xfc, 0xfd, 0xfe, 0xff, 0xff,
];

/// Mantissa-fraction table — `exp2_table[i] = round(256 * 2^(i/256)) - 256`.
///
/// Mechanically transcribed from the staged CSV
/// `docs/audio/wavpack/tables/wp-exp2.csv` (256 byte entries; anchors
/// `00 01 01 02 03 03 04 05 … ff` per `wp-log2-exp2.meta.md`). The
/// implicit leading mantissa bit `0x100` is ORed in at use (spec §4
/// step 2).
pub const EXP2_TABLE: [u8; 256] = [
    0x00, 0x01, 0x01, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x08, 0x09, 0x0a, 0x0b,
    0x0b, 0x0c, 0x0d, 0x0e, 0x0e, 0x0f, 0x10, 0x10, 0x11, 0x12, 0x13, 0x13, 0x14, 0x15, 0x16, 0x16,
    0x17, 0x18, 0x19, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1d, 0x1e, 0x1f, 0x20, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x24, 0x25, 0x26, 0x27, 0x28, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2c, 0x2d, 0x2e, 0x2f, 0x30,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3a, 0x3b, 0x3c, 0x3d,
    0x3e, 0x3f, 0x40, 0x41, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a,
    0x5b, 0x5c, 0x5d, 0x5e, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x87, 0x88, 0x89, 0x8a,
    0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b,
    0x9c, 0x9d, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad,
    0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0,
    0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc8, 0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf, 0xd0, 0xd2, 0xd3, 0xd4,
    0xd6, 0xd7, 0xd8, 0xd9, 0xdb, 0xdc, 0xdd, 0xde, 0xe0, 0xe1, 0xe2, 0xe4, 0xe5, 0xe6, 0xe8, 0xe9,
    0xea, 0xec, 0xed, 0xee, 0xf0, 0xf1, 0xf2, 0xf4, 0xf5, 0xf6, 0xf8, 0xf9, 0xfa, 0xfc, 0xfd, 0xff,
];

/// Largest magnitude `wp_log2` accepts (spec §7: "max `wp_log2` in").
///
/// Inputs above this are clamped before the interpolation bias so the
/// biased value cannot overflow 32 bits; the corresponding maximum
/// result is [`MAX_LOG_WORD`].
pub const MAX_LOG2_INPUT: u32 = 0xff80_0000;

/// Largest log word `wp_log2` can produce (spec §7: "max `wp_log2` out").
pub const MAX_LOG_WORD: i32 = 8447;

/// Mantissa alignment pivot: the normalised mantissa's MSB sits at bit
/// 8 (a 9-bit value in `0x100..=0x1ff`), so an integer log part of `9`
/// returns the mantissa unshifted (spec §4 step 3 / §7 "exp2 shift
/// pivot").
const EXP_SHIFT_PIVOT: i32 = 9;

/// Bit-length of a 32-bit value — the number of significant bits,
/// `⌊log2⌋ + 1`, with `0 → 0`. This is the `nbits_table` helper of
/// spec §2 lifted to the full 32-bit range (spec §3 step 2 applies it
/// to the highest non-zero byte and adds that byte's position; the
/// composition is exactly the 32-bit bit-length).
#[inline]
fn bit_length(v: u32) -> i32 {
    (32 - v.leading_zeros()) as i32
}

/// `wp_log2` — unsigned magnitude → integer log word with 8 fractional
/// bits (spec `wavpack-log2-exp2.md` §3).
///
/// To 8-bit precision the result is `256 * (log2(v) + 1)` — the `+1`
/// arises because the integer part is the value's **bit-length**
/// (`⌊log2⌋ + 1`), not its MSB position (spec §1). The steps:
///
/// 1. add the `1/512` interpolation bias (`avalue += avalue >> 9`);
/// 2. take `dbits`, the biased value's bit-length;
/// 3. normalise the mantissa so its MSB sits at bit 8 and look up the
///    fractional bits in [`LOG2_TABLE`] by the low byte;
/// 4. return `(dbits << 8) + fraction`.
///
/// `wp_log2(0) == 0` (the §6 canonical zero: the bias leaves `0`, the
/// bit-length is `0`, `LOG2_TABLE[0] == 0`). Inputs above
/// [`MAX_LOG2_INPUT`] are clamped to it (the spec bounds the input at
/// "about `0xff800000`"); the maximum result is [`MAX_LOG_WORD`]
/// (`8447`).
#[must_use]
pub fn wp_log2(magnitude: u32) -> i32 {
    // Spec §3 step 1: interpolation bias, centring the piecewise-linear
    // table error. Clamp first so `avalue + (avalue >> 9)` stays inside
    // u32 (the spec's stated max input).
    let clamped = magnitude.min(MAX_LOG2_INPUT);
    let avalue = clamped + (clamped >> 9);

    // Spec §3 step 2: integer part = bit-length of the biased value.
    let dbits = bit_length(avalue);

    // Spec §3 step 3: shift the value so its MSB sits at bit 8 (a
    // 9-bit mantissa in 0x100..=0x1ff) and index the fraction table
    // with the low byte. `dbits < 9` shifts left, `dbits >= 9` shifts
    // right; `dbits == 0` (zero input) indexes LOG2_TABLE[0] == 0.
    let fraction = if dbits < EXP_SHIFT_PIVOT {
        LOG2_TABLE[((avalue << (EXP_SHIFT_PIVOT - dbits)) & 0xff) as usize]
    } else {
        LOG2_TABLE[((avalue >> (dbits - EXP_SHIFT_PIVOT)) & 0xff) as usize]
    };

    // Spec §3 step 4: bit-length in the high bits (256 per binary order
    // of magnitude) plus the 8-bit fractional log.
    (dbits << 8) + i32::from(fraction)
}

/// `wp_exp2s` — signed log word → signed 32-bit value (spec
/// `wavpack-log2-exp2.md` §4). The exact inverse of [`wp_log2`] with a
/// sign convention: the function is odd (`wp_exp2s(-l) == -wp_exp2s(l)`,
/// §4 step 1), so a negative log word decodes to the negated magnitude.
///
/// For a non-negative word: form the 9-bit mantissa
/// `EXP2_TABLE[l & 0xff] | 0x100` and shift it into place by the
/// integer part `l >> 8` — right by `9 - int` when `int <= 9`, left by
/// `int - 9` otherwise (§4 steps 2–3). `wp_exp2s(0) == 0` (the §6
/// canonical zero) and `wp_exp2s(256) == 1`.
///
/// A conformant stream keeps log words within `±`[`MAX_LOG_WORD`]
/// (magnitudes fit 32 bits); a hostile word beyond that wraps the left
/// shift (`wrapping_shl`) rather than panicking, mirroring the
/// truncating-malformed-input posture of the other fixups.
#[must_use]
pub fn wp_exp2s(log_word: i32) -> i32 {
    if log_word < 0 {
        // Spec §4 step 1: odd function. The smallest sign-extended
        // 16-bit word (-32768) negates within i32 range.
        return wp_exp2s(-log_word).wrapping_neg();
    }

    // Spec §4 step 2: 9-bit mantissa with the implicit leading bit.
    let mantissa = u32::from(EXP2_TABLE[(log_word & 0xff) as usize]) | 0x100;

    // Spec §4 step 3: apply the exponent around the pivot 9.
    let int_part = log_word >> 8;
    if int_part <= EXP_SHIFT_PIVOT {
        (mantissa >> (EXP_SHIFT_PIVOT - int_part)) as i32
    } else {
        mantissa.wrapping_shl((int_part - EXP_SHIFT_PIVOT) as u32) as i32
    }
}

/// Expand a little-endian on-wire 16-bit log word (`0x05` median /
/// `0x04` seed field) to its signed 32-bit value.
///
/// Spec `wavpack-log2-exp2.md` §5: "each stored 16-bit field is
/// little-endian; it is sign-extended (it is a signed log word) and
/// passed to `wp_exp2s`."
#[inline]
#[must_use]
pub fn expand_log_word(lo: u8, hi: u8) -> i32 {
    wp_exp2s(i32::from(i16::from_le_bytes([lo, hi])))
}

/// Pack a signed value into its little-endian on-wire 16-bit log word —
/// the nearest-value forward inverse of [`expand_log_word`].
///
/// The magnitude is logged with [`wp_log2`] and the word's sign carries
/// the value's sign (spec §4 step 1: `wp_exp2s` is odd). Zero packs to
/// the canonical all-zero word (spec §6 pin). The pack is a
/// **quantizer**: `expand_log_word` of the result reproduces the value
/// exactly for small magnitudes (every seed that occurs in practice)
/// and to within the table precision (~0.1%, spec §1) for wide ones —
/// the exactness predicate is [`quantize_log_value`]`(v) == v`.
#[must_use]
pub fn pack_log_word(value: i32) -> [u8; 2] {
    let magnitude = value.unsigned_abs();
    let log = wp_log2(magnitude);
    let signed = if value < 0 { -log } else { log };
    (signed as i16).to_le_bytes()
}

/// Quantize a signed value to the nearest value its on-wire 16-bit log
/// word can represent: `expand ∘ pack`. Idempotent; an encoder that
/// primes state from real values must prime with the quantized values
/// so the decoder's expansion reconstructs identical state.
#[must_use]
pub fn quantize_log_value(value: i32) -> i32 {
    let [lo, hi] = pack_log_word(value);
    expand_log_word(lo, hi)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_log_word_is_canonical_zero_both_directions() {
        // Spec §6 erratum pin: 0x0000 ↔ 0, special-cased both ways.
        assert_eq!(wp_log2(0), 0);
        assert_eq!(wp_exp2s(0), 0);
        assert_eq!(expand_log_word(0, 0), 0);
        assert_eq!(pack_log_word(0), [0, 0]);
    }

    #[test]
    fn exp2s_of_256_is_one() {
        // Spec meta verification anchor: wp_exp2s(256) == 1
        // (mantissa 0x100 >> (9 - 1) == 1).
        assert_eq!(wp_exp2s(256), 1);
    }

    #[test]
    fn worked_example_log2_of_1000() {
        // Spec §4 worked example: biased 1001, dbits = 10, mantissa low
        // byte 0xf4, LOG2_TABLE[0xf4] = 0xf7, result 0x0af7 = 2807.
        assert_eq!(LOG2_TABLE[0xf4], 0xf7);
        assert_eq!(wp_log2(1000), 2807);
    }

    #[test]
    fn worked_example_exp2s_of_2807() {
        // Spec §4 worked example: frac 0xf7, EXP2_TABLE[0xf7] = 0xf4,
        // mantissa 0x1f4 = 500, int part 10 > 9, 500 << 1 = 1000.
        assert_eq!(EXP2_TABLE[0xf7], 0xf4);
        assert_eq!(wp_exp2s(2807), 1000);
    }

    #[test]
    fn exp2s_is_odd() {
        // Spec §4 step 1: wp_exp2s(-l) == -wp_exp2s(l).
        for l in [1, 255, 256, 257, 1000, 2807, 4096, MAX_LOG_WORD] {
            assert_eq!(wp_exp2s(-l), -wp_exp2s(l), "l = {l}");
        }
        // The full sign-extended 16-bit word range stays defined and
        // odd: -32768 negates to +32768 inside i32 before expansion.
        assert_eq!(
            wp_exp2s(i32::from(i16::MIN)),
            wp_exp2s(32768).wrapping_neg()
        );
    }

    #[test]
    fn table_anchor_bytes_match_the_staged_meta() {
        // wp-log2-exp2.meta.md anchor rows.
        assert_eq!(
            &LOG2_TABLE[..8],
            &[0x00, 0x01, 0x03, 0x04, 0x06, 0x07, 0x09, 0x0a]
        );
        assert_eq!(LOG2_TABLE[255], 0xff);
        assert_eq!(
            &EXP2_TABLE[..8],
            &[0x00, 0x01, 0x01, 0x02, 0x03, 0x03, 0x04, 0x05]
        );
        assert_eq!(EXP2_TABLE[255], 0xff);
    }

    #[test]
    fn tables_are_monotonic_non_decreasing() {
        // Both closed forms are monotonic in i, so the transcribed
        // tables must be non-decreasing.
        for i in 1..256 {
            assert!(LOG2_TABLE[i] >= LOG2_TABLE[i - 1], "log2 at {i}");
            assert!(EXP2_TABLE[i] >= EXP2_TABLE[i - 1], "exp2 at {i}");
        }
    }

    #[test]
    fn log2_result_is_monotonic_in_the_magnitude() {
        let mut prev = wp_log2(0);
        for v in 1..4096u32 {
            let cur = wp_log2(v);
            assert!(cur >= prev, "wp_log2 must be monotonic at {v}");
            prev = cur;
        }
    }

    #[test]
    fn max_input_yields_max_log_word() {
        // Spec §7: max input ~0xff800000 → max result 8447; larger
        // inputs clamp to the same word.
        assert_eq!(wp_log2(MAX_LOG2_INPUT), MAX_LOG_WORD);
        assert_eq!(wp_log2(u32::MAX), MAX_LOG_WORD);
    }

    #[test]
    fn round_trip_is_exact_for_small_magnitudes() {
        // Spec §1: "the two round-trip: wp_exp2s(wp_log2(v)) == v for
        // the small magnitudes that actually occur as seeds". The
        // 8-fractional-bit table resolves every magnitude below 115
        // exactly (the first quantized magnitude is 115 → 114);
        // beyond that the error stays within the table precision.
        for v in 0..=114i32 {
            assert_eq!(wp_exp2s(wp_log2(v as u32)), v, "round trip at {v}");
            assert_eq!(quantize_log_value(v), v, "quantize at {v}");
            assert_eq!(quantize_log_value(-v), -v, "quantize at -{v}");
        }
        assert_eq!(quantize_log_value(115), 114);
        for v in 115..=4096i32 {
            let back = wp_exp2s(wp_log2(v as u32));
            assert!((back - v).abs() <= v / 100 + 3, "err at {v}: {back}");
        }
    }

    #[test]
    fn round_trip_is_within_a_tenth_percent_across_the_range() {
        // Spec §1: "to within ~0.1% across the whole 32-bit range".
        let mut v = 1u64;
        while v <= 0x7fff_ff00 {
            for probe in [v, v + v / 3, v + v / 2] {
                if probe > 0x7fff_ff00 {
                    continue;
                }
                let back = wp_exp2s(wp_log2(probe as u32)) as i64 as f64;
                let orig = probe as f64;
                let rel = ((back - orig) / orig).abs();
                assert!(rel < 0.001, "rel err {rel} at {probe}");
            }
            v *= 2;
        }
    }

    #[test]
    fn quantize_is_idempotent() {
        for v in [
            0i32,
            1,
            -1,
            100,
            -100,
            1000,
            65535,
            -65535,
            1 << 20,
            i32::MAX,
            i32::MIN + 1,
        ] {
            let q = quantize_log_value(v);
            assert_eq!(quantize_log_value(q), q, "idempotent at {v}");
        }
    }

    #[test]
    fn pack_log_word_writes_sign_into_the_word() {
        // The worked-example value: 1000 logs to 2807 = 0x0af7.
        assert_eq!(pack_log_word(1000), 2807i16.to_le_bytes());
        assert_eq!(pack_log_word(-1000), (-2807i16).to_le_bytes());
        assert_eq!(expand_log_word(0xf7, 0x0a), 1000);
        let [lo, hi] = (-2807i16).to_le_bytes();
        assert_eq!(expand_log_word(lo, hi), -1000);
    }

    #[test]
    fn expand_log_word_sign_extends_the_wire_field() {
        // A word with the top bit set is a negative log word, not a
        // large positive one (spec §5: sign-extended).
        let word = -256i16; // == wp_log2 word for magnitude 1, negated
        let [lo, hi] = word.to_le_bytes();
        assert_eq!(expand_log_word(lo, hi), -1);
    }

    #[test]
    fn log2_bit_length_integer_part() {
        // The integer part is the bit-length: values just below a power
        // of two carry that power's bit-length; exact powers of two
        // land on (bits << 8) exactly (bias never bumps them over).
        assert_eq!(wp_log2(1) >> 8, 1);
        assert_eq!(wp_log2(2) >> 8, 2);
        assert_eq!(wp_log2(4) >> 8, 3);
        assert_eq!(wp_log2(255) >> 8, 8);
        assert_eq!(wp_log2(256) >> 8, 9);
        // wp_log2(2^k) == (k + 1) << 8 for small k (fraction 0).
        for k in 0..20 {
            assert_eq!(wp_log2(1 << k), (k + 1) << 8, "power 2^{k}");
        }
    }

    #[test]
    fn exp2s_shift_pivot_behaviour() {
        // Spec §4 step 3: int_part == 9 returns the mantissa unshifted;
        // smaller shifts right, larger shifts left.
        assert_eq!(wp_exp2s(9 << 8), 0x100);
        assert_eq!(wp_exp2s(10 << 8), 0x200);
        assert_eq!(wp_exp2s(8 << 8), 0x80);
    }

    #[test]
    fn max_log_word_expands_to_the_documented_max_magnitude() {
        // Spec §7: max wp_log2 input ~0xff800000 ↔ max output 8447.
        // Expanding the max word reproduces that magnitude bit pattern
        // (as the wrapped i32 the u32 magnitude maps to).
        assert_eq!(wp_exp2s(MAX_LOG_WORD) as u32, MAX_LOG2_INPUT);
    }

    #[test]
    fn hostile_words_beyond_the_conformant_range_do_not_panic() {
        // Log words a conformant stream never carries (int parts past
        // the 32-bit magnitude ceiling) must stay defined: the left
        // shift wraps rather than aborts.
        for l in [
            i32::from(i16::MAX),
            i32::from(i16::MIN),
            MAX_LOG_WORD + 1,
            0x7f00,
            -0x7f00,
        ] {
            let _ = wp_exp2s(l);
        }
    }
}
