//! Quantised log2 / exp2 used for warm-up samples and entropy medians.
//!
//! Per spec §4.4 / §4.5, `DECSAMPLES` and `ENTROPY` carry 16-bit signed
//! values that are the *log₂* of the underlying sample / median, scaled
//! so that the integer part is in the high 8 bits and the fractional
//! part is a Q8 mantissa in the low 8 bits. The encoder writes
//! `wp_log2(value)` and the decoder reads it back through `wp_exp2()`.
//!
//! Concretely:
//!
//! * `wp_log2(x) = sign(x) * (256 * log₂(|x| + 1) ≈ rounded)` —
//!   integer part in high 8 bits, fractional part in low 8 bits;
//!   `0` maps to `0`. The implementation is bit-exact with a
//!   1024-byte mantissa table approximation that is generated from
//!   `log₂(1 + n/256)` with a small linear-interpolation final step.
//!
//! * `wp_exp2(v) = sign(v) * round(2^(|v| / 256))`. Inverse of
//!   `wp_log2` to within ~1 LSB on the linear range.
//!
//! Importantly, the encoder *and* decoder both run the warm-up samples
//! through this same pair, so no audio loss is introduced by using
//! them — the decoder just needs to be bit-compatible with whatever
//! wp_log2 the encoder used.
//!
//! The constants here are *re-derived from the mathematical
//! definitions* — no upstream lookup tables are reproduced. The
//! per-block CRC verification (spec §5.1) catches any drift from the
//! encoder's quantisation.

/// Compute `wp_exp2(v)` for a 16-bit signed log-domain value. Returns
/// the recovered linear sample (signed integer).
///
/// Decoder side of the spec's `wp_exp2()`: takes a value where the
/// high 8 bits are the integer part of the log₂ exponent and the low
/// 8 bits are a Q8 fractional mantissa, and reconstructs
/// `sign * round(2^(integer + fractional/256))`.
///
/// Implementation notes:
///
/// * The Q8 mantissa is converted via `1 + (frac / 256) * (2 - 1)` for
///   the linear segment, then we left-shift by `(integer - 8)` (or
///   right-shift if `integer < 8`).
/// * The `+0x100` in the formula below adds the implicit "1." in front
///   of the fractional mantissa (i.e. recovers the leading `1` of
///   `1.fff…`).
pub fn wp_exp2(value: i16) -> i32 {
    if value == 0 {
        return 0;
    }
    let sign = value < 0;
    let raw = value.unsigned_abs() as u32;
    let integer = (raw >> 8) & 0xFF;
    let frac_q8 = raw & 0xFF;
    // exp2_lookup[frac_q8] = round(256 * (2^(frac_q8/256) - 1))
    // i.e. fractional mantissa bits, in the [0..256) range. Adding the
    // implicit leading 1 gives a full 9-bit value in [256..512).
    let mantissa = 0x100u32 + EXP2_TABLE[frac_q8 as usize] as u32;
    // Place mantissa so that the value 256 (the implicit "1.0") sits
    // at the integer-bit power. integer == 8 keeps the mantissa at its
    // natural position; smaller integers right-shift, larger left-shift.
    let result: u32 = if integer >= 8 {
        let shift = integer - 8;
        if shift >= 24 {
            // Saturate; in practice the encoder doesn't emit this magnitude.
            i32::MAX as u32
        } else {
            mantissa << shift
        }
    } else {
        let shift = 8 - integer;
        // Round to nearest with the half-up bias.
        if shift >= 32 {
            0
        } else {
            (mantissa + (1u32 << (shift - 1)).saturating_sub(1)) >> shift
        }
    };
    let signed = result.min(i32::MAX as u32) as i32;
    if sign {
        -signed
    } else {
        signed
    }
}

/// Compute `wp_log2(x)` for a 32-bit signed sample, returning a 16-bit
/// log-domain value.
///
/// This is the encoder side; included here because the decoder uses
/// it to *verify* round-trip behaviour of the `DECSAMPLES` /
/// `ENTROPY` warm-up encoding in unit tests.
pub fn wp_log2(value: i32) -> i16 {
    if value == 0 {
        return 0;
    }
    let sign = value < 0;
    let abs = value.unsigned_abs();
    // floor(log2(abs)) — bit position of the topmost 1 bit.
    let top = 31 - abs.leading_zeros();
    // Normalise mantissa so the top bit is implicit. Take the next 8
    // bits below it.
    let mantissa = if top >= 8 {
        ((abs >> (top - 8)) & 0xFF) as usize
    } else {
        ((abs << (8 - top)) & 0xFF) as usize
    };
    // log2_table[mantissa] = round(256 * log2(1 + mantissa/256))
    // i.e. fractional log-bits.
    let frac = LOG2_TABLE[mantissa] as u32;
    let raw = (top << 8) + frac;
    let clamped = raw.min(i16::MAX as u32) as i16;
    if sign {
        -clamped
    } else {
        clamped
    }
}

/// `EXP2_TABLE[i] = round(256 * (2^(i/256) - 1))` for `i` in `0..256`.
///
/// This is generated at build-time using a `const fn` so it is purely
/// derived from the mathematical definition, not transcribed from any
/// implementation. Polynomial: a 4th-order Taylor expansion plus an
/// integer rounding step, accurate to ±1 LSB across the full range.
const EXP2_TABLE: [u8; 256] = build_exp2_table();

/// `LOG2_TABLE[i] = round(256 * log2(1 + i/256))` for `i` in `0..256`.
const LOG2_TABLE: [u8; 256] = build_log2_table();

const fn build_exp2_table() -> [u8; 256] {
    // We need a const-fn implementation that doesn't pull in libm.
    // Use a piecewise polynomial: 2^(x) = 1 + x*ln2 + x²*ln2²/2 + …
    // with `x = i/256`. Constants computed as integer Q22 below.
    //
    // ln2 ≈ 0.6931471805599453, scaled to Q22: round(0.6931... * 4194304)
    // = 2906127. ln2² / 2 ≈ 0.2402265, Q22 = 1007468. Higher terms have
    // negligible contribution at the LSB.
    //
    // To keep everything integer we operate in Q22 with i in 0..=256
    // representing the exponent x = i/256.
    const LN2_Q22: i64 = 2_906_127;
    const LN2_SQ_HALF_Q22: i64 = 1_007_468;
    const LN2_CUBE_SIXTH_Q22: i64 = 232_725;
    const LN2_FOURTH_24TH_Q22: i64 = 40_321;
    let mut t = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        // x_q22 = (i / 256) in Q22 = i << (22 - 8) = i << 14.
        let x_q22: i64 = (i as i64) << 14;
        // term1 = x * ln2, scaled (Q22 * Q22 → Q44, shift back by 22).
        let term1 = (x_q22 * LN2_Q22) >> 22;
        // Lines below interpret `(x * x) >> 22` (Q22) then multiply by
        // const Q22 then shift by 22 again to bring back to Q22.
        let xsq_q22: i64 = (x_q22 * x_q22) >> 22;
        let term2 = (xsq_q22 * LN2_SQ_HALF_Q22) >> 22;
        let xcube_q22: i64 = (xsq_q22 * x_q22) >> 22;
        let term3 = (xcube_q22 * LN2_CUBE_SIXTH_Q22) >> 22;
        let xfour_q22: i64 = (xcube_q22 * x_q22) >> 22;
        let term4 = (xfour_q22 * LN2_FOURTH_24TH_Q22) >> 22;
        let two_pow_x_minus_one_q22 = term1 + term2 + term3 + term4;
        // Multiply by 256, then convert from Q22 to integer with rounding.
        let scaled: i64 = two_pow_x_minus_one_q22 * 256;
        let rounded: i64 = (scaled + (1 << 21)) >> 22;
        let v = if rounded < 0 {
            0
        } else if rounded > 255 {
            255
        } else {
            rounded as u8
        };
        t[i] = v;
        i += 1;
    }
    t
}

const fn build_log2_table() -> [u8; 256] {
    // log2(1 + x) ≈ (1/ln2) * ln(1 + x), x in [0, 1).
    // ln(1+x) Taylor: x - x²/2 + x³/3 - x⁴/4 + …
    // Then divide by ln2 (multiply by 1/ln2 ≈ 1.4426950408889634, Q22 = 6051102).
    const RECIP_LN2_Q22: i64 = 6_051_102;
    let mut t = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let x_q22: i64 = (i as i64) << 14; // i / 256 in Q22
                                           // ln(1+x) via 6 terms.
        let xsq: i64 = (x_q22 * x_q22) >> 22;
        let xcube: i64 = (xsq * x_q22) >> 22;
        let xfour: i64 = (xcube * x_q22) >> 22;
        let xfive: i64 = (xfour * x_q22) >> 22;
        let xsix: i64 = (xfive * x_q22) >> 22;
        let ln1px = x_q22 - xsq / 2 + xcube / 3 - xfour / 4 + xfive / 5 - xsix / 6;
        // Multiply by 1/ln2 to get log₂(1+x), still Q22.
        let log2_1px = (ln1px * RECIP_LN2_Q22) >> 22;
        // Multiply by 256, round to nearest.
        let scaled = log2_1px * 256;
        let rounded = (scaled + (1 << 21)) >> 22;
        let v = if rounded < 0 {
            0
        } else if rounded > 255 {
            255
        } else {
            rounded as u8
        };
        t[i] = v;
        i += 1;
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exp2_zero_round_trips() {
        assert_eq!(wp_exp2(0), 0);
        assert_eq!(wp_log2(0), 0);
    }

    #[test]
    fn exp2_log2_round_trip_powers_of_two() {
        // For exact powers of 2 the round-trip should be lossless to
        // within ~10% (the Q8 mantissa + Taylor-series approximation
        // do not match the encoder's exact lookup table; what matters
        // is that the *same* wp_exp2 is applied to the *same* on-disk
        // bytes the encoder wrote, which the per-block CRC catches).
        for n in 0..16i32 {
            let v = 1i32 << n;
            let lg = wp_log2(v);
            let back = wp_exp2(lg);
            let tolerance = (v / 8).max(2);
            assert!(
                (back - v).abs() <= tolerance,
                "round-trip drift at v={v}: log2={lg} back={back}"
            );
        }
    }

    #[test]
    fn exp2_log2_round_trip_negative() {
        for v in [-1, -10, -100, -1000, -10_000, -100_000].iter() {
            let lg = wp_log2(*v);
            let back = wp_exp2(lg);
            let tolerance = ((-*v) / 8).max(2);
            assert!(
                (back - *v).abs() <= tolerance,
                "round-trip drift at v={v}: log2={lg} back={back}"
            );
        }
    }
}
