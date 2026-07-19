//! WavPack `0x08` floating-point data profile (`FLOAT_DATA`).
//!
//! Staged spec `docs/audio/wavpack/spec/wavpack-sample-formats.md` §2:
//! when block-header flag bit 7 (`FLOAT_DATA`) is set the block carries
//! 32-bit IEEE-754 floats, compressed as a **scaled integer stream**;
//! a 4-byte `0x08` sub-block describes how to turn each reconstructed
//! integer back into a float:
//!
//! | Offset | Field            | Meaning                                            |
//! | ------ | ---------------- | -------------------------------------------------- |
//! | 0      | `float_flags`    | how the mantissa low bits / zeros are coded        |
//! | 1      | `float_shift`    | bits the decoded integer is left-shifted           |
//! | 2      | `float_max_exp`  | maximum IEEE exponent present                      |
//! | 3      | `float_norm_exp` | exponent of the normalisation reference (unity)    |
//!
//! ## Reconstruction model (spec §2 "Decode model")
//!
//! Each entropy-decoded integer is turned back into an IEEE-754 single
//! by re-inserting the sign, the exponent, and the mantissa:
//!
//! 1. take the magnitude and apply the static `float_shift`;
//! 2. normalise the 24-bit mantissa (implicit leading bit at bit 23):
//!    the per-sample normalisation shift is `24 − bit_length`, and
//!    `float_max_exp` anchors the exponent — a full 24-bit magnitude
//!    carries exponent `float_max_exp`, every bit short of that
//!    decrements it (`exponent = float_max_exp − (24 − bit_length)`);
//! 3. the vacated low mantissa bits "that could not be predicted" are
//!    filled per `float_flags`: zeros by default, ones under
//!    `SHIFT_ONES` (`0x01`), or read literally from the `0x0C`
//!    extension stream under `SHIFT_SENT` (`0x04`), LSB-first;
//! 4. a zero integer decodes to `+0.0` unless `ZEROS_SENT` (`0x08`) is
//!    set, in which case the zero sample's payload is read from the
//!    `0x0C` stream: one **marker bit** — set = a literal float
//!    follows (23 mantissa bits, 8 exponent bits, 1 sign bit,
//!    LSB-first: any value too small for the scaled-integer stream,
//!    denormals included); clear = the sample is a true zero, and one
//!    sign bit follows **only when `NEG_ZEROS` (`0x10`) is set**
//!    (distinguishing `-0.0` from `+0.0`). The staged spec names the
//!    flag semantics but not the bit order; the layout here is pinned
//!    black-box against reference-encoded probe files (round 405).
//!
//! ## `SHIFT_SAME` (staged spec §2.1, round 408)
//!
//! `SHIFT_SAME` (`0x02`) means the vacated low mantissa bits are all
//! identical *within* each sample but the shared value *varies* from
//! sample to sample: each non-zero sample with a non-empty vacated
//! window reads **one carrier bit** from the `0x0C` extension stream —
//! `1` fills the window with ones, `0` with zeros. Zero samples spend
//! no carrier bit (round-408 black-box pin: a stream interleaving
//! exact zeros carries exactly one wvx bit per *non-zero* sample), and
//! reference-encoded `SHIFT_SAME` blocks anchor `float_max_exp` one
//! above the largest present exponent so every non-zero sample has a
//! non-empty window.
//!
//! ## `EXCEPTIONS` (staged spec §2.2 + round-408 black-box pin)
//!
//! `EXCEPTIONS` (`0x20`) marks a block that carries infinities / NaNs.
//! On the wire an exceptional sample's decoded integer magnitude
//! reconstructs to bit length **25** after the static shift (the
//! sentinel `magnitude << float_shift == 1 << 24`, one bit above the
//! 24-bit mantissa window that anchors `float_max_exp`); its sign bit
//! travels the normal §4.2 sign path. The `0x0C` extension stream then
//! carries, per exceptional sample, one **marker bit**: `0` ⇒ the
//! mantissa is zero (`±infinity`), `1` ⇒ the full 23-bit NaN mantissa
//! payload follows LSB-first. The reconstructed exponent is the IEEE
//! maximum `0xFF`. (The staged spec's §2.2 wording reads the 23-bit
//! mantissa unconditionally and pins `float_max_exp` at `0xFF`;
//! round-408 black-box probes show the marker bit and show
//! `float_max_exp` anchoring the *finite* samples — reported back as a
//! docs erratum ask.)
//!
//! ## The float extension CRC (round-405 erratum)
//!
//! Each reassembled float is folded into the extension CRC and
//! compared against the `crc_wvx` stored at the head of the `0x0C`
//! payload — but the **fold input differs from the int32 path**: the
//! staged `wavpack-decorrelation.md` §5.5 halfword formula
//! (`crc_x*9 + 3*lo16 + hi16`) holds for int32 data only. For float
//! data the fold is three mono-CRC steps per sample over the float's
//! three fields:
//!
//! ```text
//! crc_x = crc_x*3 + mantissa23
//! crc_x = crc_x*3 + exponent8
//! crc_x = crc_x*3 + sign1
//! ```
//!
//! (equivalently `crc_x*27 + 9*mantissa + 3*exponent + sign`), seeded
//! `0xffffffff`. Pinned black-box by differential probes against
//! reference-encoded files (single-bit mantissa/exponent/sign flips
//! move the stored CRC by exactly `9<<k` / `3` / `1`); reported as a
//! docs erratum ask.

use crate::error::{Error, Result};
use crate::samples::{BitReader, BitWriter};

/// On-wire byte length of the `0x08` float-info payload (staged spec
/// `wavpack-sample-formats.md` §2: "payload is **4 bytes**, one per
/// field").
pub const FLOAT_INFO_PAYLOAD_BYTES: usize = 4;

/// `float_flags` bit `0x01` — shifted-in low mantissa bits are `1`.
pub const FLOAT_SHIFT_ONES: u8 = 0x01;
/// `float_flags` bit `0x02` — shifted-in low mantissa bits are all
/// identical within each sample; the shared value is a one-bit-per-
/// non-zero-sample carrier in the `0x0C` extension stream (staged
/// spec §2.1).
pub const FLOAT_SHIFT_SAME: u8 = 0x02;
/// `float_flags` bit `0x04` — shifted-in low mantissa bits are sent in
/// the `0x0C` extension stream.
pub const FLOAT_SHIFT_SENT: u8 = 0x04;
/// `float_flags` bit `0x08` — "zero" samples are sent literally.
pub const FLOAT_ZEROS_SENT: u8 = 0x08;
/// `float_flags` bit `0x10` — negative zeros occur and are preserved.
pub const FLOAT_NEG_ZEROS: u8 = 0x10;
/// `float_flags` bit `0x20` — exceptional values (inf / NaN) occur;
/// each is a bit-length-25 sentinel integer whose mantissa payload is
/// carried in the `0x0C` extension stream (staged spec §2.2 +
/// round-408 black-box pin, see the module doc).
pub const FLOAT_EXCEPTIONS: u8 = 0x20;

/// Typed expansion of the `0x08` floating-point profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FloatInfo {
    /// Bit-field describing how the mantissa low bits / zeros are
    /// coded (`FLOAT_*` constants).
    pub float_flags: u8,
    /// Number of bits the decoded integer is left-shifted before
    /// mantissa normalisation.
    pub float_shift: u8,
    /// Maximum IEEE exponent present (anchors the per-sample exponent
    /// reconstruction).
    pub float_max_exp: u8,
    /// Exponent of the normalisation reference (unity scaling).
    pub float_norm_exp: u8,
}

impl FloatInfo {
    /// `true` when the profile needs the `0x0C` extension stream on
    /// every applicable sample (literal mantissa bits, per-sample
    /// carrier bits, and/or literal zero samples). `EXCEPTIONS` is
    /// deliberately *not* included: an exceptions-capable profile only
    /// touches the stream when an exceptional sample actually occurs,
    /// so its absence is not structural (the per-sample read errors if
    /// an exception arrives with no stream).
    #[must_use]
    pub fn requires_extension(&self) -> bool {
        self.float_flags & (FLOAT_SHIFT_SENT | FLOAT_SHIFT_SAME | FLOAT_ZEROS_SENT) != 0
    }

    /// `true` when the profile only uses coding shapes the staged spec
    /// pins down. Since round 408 (staged spec §2.1–§2.2 + black-box
    /// pins) every documented `float_flags` shape decodes, so this is
    /// always `true`; the method is kept for callers written against
    /// the earlier `SHIFT_SAME` / `EXCEPTIONS` refusals.
    #[must_use]
    pub fn is_supported(&self) -> bool {
        true
    }
}

/// Expand the payload of a `0x08` float-info sub-block into a typed
/// [`FloatInfo`].
///
/// The payload must be exactly 4 bytes (`float_flags, float_shift,
/// float_max_exp, float_norm_exp` in wire order — staged spec §2) or
/// [`Error::FloatInfoLength`] is returned.
pub fn expand_float_info(payload: &[u8]) -> Result<FloatInfo> {
    let [float_flags, float_shift, float_max_exp, float_norm_exp] = payload else {
        return Err(Error::FloatInfoLength(payload.len()));
    };
    Ok(FloatInfo {
        float_flags: *float_flags,
        float_shift: *float_shift,
        float_max_exp: *float_max_exp,
        float_norm_exp: *float_norm_exp,
    })
}

/// Reassemble a buffer of entropy-decoded scaled integers into IEEE-754
/// single bit patterns in place (each `i32` slot becomes the bit
/// pattern of the sample's `f32`), reading literal mantissa bits /
/// literal zero samples from the `0x0C` extension bit reader where the
/// profile calls for them, and folding every reassembled pattern into
/// the extension CRC.
///
/// `ext` must be `Some` when [`FloatInfo::requires_extension`]. Returns
/// the accumulated `crc_x` register (spec `wavpack-decorrelation.md`
/// §5.5) for the block-end comparison. Profiles carrying `SHIFT_SAME`
/// or `EXCEPTIONS` are refused by the caller before this runs (see
/// [`FloatInfo::is_supported`]).
pub fn reassemble_float(
    pcm: &mut [i32],
    info: &FloatInfo,
    mut ext: Option<&mut BitReader<'_>>,
) -> Result<u32> {
    let mut crc_x = crate::crc::CRC_INIT;
    for slot in pcm.iter_mut() {
        let integer = *slot;
        let bits = reassemble_one(integer, info, ext.as_deref_mut(), false)?;
        crc_x = update_float_extension(crc_x, bits);
        *slot = bits as i32;
    }
    Ok(crc_x)
}

/// [`reassemble_float`] for a block that carries **no** `0x0C`
/// extension stream even though the profile names extension-fed fills
/// — the shape a **hybrid (lossy)** float block takes on the wire
/// (round-408 black-box pin: reference hybrid float files keep the
/// `SHIFT_SENT` profile flag but omit the wvx sub-block; the dropped
/// low mantissa bits are implied zero in the lossy reconstruction).
///
/// Extension-fed fields default: `SHIFT_SENT` / `SHIFT_SAME` windows
/// fill with zeros, a `ZEROS_SENT` zero integer decodes to implied
/// `+0.0`. An `EXCEPTIONS` sentinel still needs its mantissa payload
/// and errors with [`Error::BlockMissingOverflowBits`] — an
/// exceptional value cannot be implied.
pub fn reassemble_float_implied(pcm: &mut [i32], info: &FloatInfo) -> Result<()> {
    for slot in pcm.iter_mut() {
        *slot = reassemble_one(*slot, info, None, true)? as i32;
    }
    Ok(())
}

/// One float-sample step of the extension CRC (module-doc erratum):
/// three mono-CRC folds over the float's mantissa, exponent and sign
/// fields.
#[inline]
#[must_use]
pub fn update_float_extension(crc_x: u32, float_bits: u32) -> u32 {
    let crc_x = crate::crc::update_mono(crc_x, (float_bits & 0x007f_ffff) as i32);
    let crc_x = crate::crc::update_mono(crc_x, ((float_bits >> 23) & 0xff) as i32);
    crate::crc::update_mono(crc_x, (float_bits >> 31) as i32)
}

/// Reconstruct one float bit pattern from its scaled integer.
/// `implied` selects the no-extension-stream lossy defaults (see
/// [`reassemble_float_implied`]) instead of erroring on a missing
/// reader.
fn reassemble_one(
    integer: i32,
    info: &FloatInfo,
    ext: Option<&mut BitReader<'_>>,
    implied: bool,
) -> Result<u32> {
    let sign = if integer < 0 { 1u32 << 31 } else { 0 };
    let magnitude = integer.unsigned_abs();

    if magnitude == 0 {
        if info.float_flags & FLOAT_ZEROS_SENT == 0 {
            // Zeros are implied: +0.0.
            return Ok(0);
        }
        // ZEROS_SENT: one marker bit. Set = a literal float follows
        // (23 mantissa bits, 8 exponent bits, 1 sign bit — any value
        // too small for the scaled-integer stream, denormals
        // included). Clear = a true zero, whose sign bit follows only
        // under NEG_ZEROS. All fields LSB-first. (Round-405 black-box
        // pin; see the module doc.)
        let Some(reader) = ext else {
            if implied {
                // Lossy stream without wvx: every zero is an implied
                // +0.0.
                return Ok(0);
            }
            return Err(Error::BlockMissingOverflowBits);
        };
        if reader.get_bit()? == 1 {
            let mantissa = reader.get_bits(23)?;
            let exponent = reader.get_bits(8)?;
            let literal_sign = reader.get_bit()? << 31;
            return Ok(literal_sign | (exponent << 23) | mantissa);
        }
        let zero_sign = if info.float_flags & FLOAT_NEG_ZEROS != 0 {
            reader.get_bit()? << 31
        } else {
            0
        };
        return Ok(zero_sign);
    }

    // Static shift, then per-sample mantissa normalisation.
    let value = magnitude.wrapping_shl(u32::from(info.float_shift));
    let bit_length = 32 - value.leading_zeros();
    if bit_length > 24 {
        // One bit above the 24-bit mantissa window is the EXCEPTIONS
        // sentinel (module doc): the sample is an infinity or a NaN
        // whose mantissa payload lives in the extension stream — one
        // marker bit, then (marker == 1) the 23-bit NaN mantissa
        // LSB-first. The sign travelled the normal sign path.
        if info.float_flags & FLOAT_EXCEPTIONS != 0 && bit_length == 25 {
            // An exceptional value cannot be implied — its mantissa
            // payload only exists in the extension stream, so a missing
            // reader is an error even in the lossy (implied) mode.
            let reader = ext.ok_or(Error::BlockMissingOverflowBits)?;
            let mantissa = if reader.get_bit()? == 1 {
                reader.get_bits(23)?
            } else {
                0
            };
            return Ok(sign | (0xFFu32 << 23) | mantissa);
        }
        // Otherwise the scaled integer overflows the 24-bit mantissa
        // window — a conformant encoder never produces this
        // (float_max_exp anchors the largest finite magnitude at
        // exactly 24 bits).
        return Err(Error::FloatMagnitudeOverflow(value));
    }
    let shift_needed = 24 - bit_length;
    let exponent = i32::from(info.float_max_exp) - shift_needed as i32;
    if exponent <= 0 {
        // Denormal territory: the mantissa loses its implicit bit and
        // shifts right instead. (No literal low bits participate — the
        // whole magnitude is below the normalised range.)
        let denorm = value >> (1 - exponent).min(31);
        return Ok(sign | (denorm & 0x007f_ffff));
    }

    let low_bits = if shift_needed == 0 {
        0
    } else if info.float_flags & FLOAT_SHIFT_SENT != 0 {
        match ext {
            Some(reader) => reader.get_bits(shift_needed)?,
            // Lossy stream without wvx: the sent bits are implied zero.
            None if implied => 0,
            None => return Err(Error::BlockMissingOverflowBits),
        }
    } else if info.float_flags & FLOAT_SHIFT_ONES != 0 {
        (1u32 << shift_needed) - 1
    } else if info.float_flags & FLOAT_SHIFT_SAME != 0 {
        // Staged spec §2.1: one carrier bit per non-zero sample —
        // `1` fills the vacated window with ones, `0` with zeros.
        // (§2.1 precedence: SENT wins over ONES wins over SAME.)
        match ext {
            Some(reader) => {
                if reader.get_bit()? == 1 {
                    (1u32 << shift_needed) - 1
                } else {
                    0
                }
            }
            // Lossy stream without wvx: the carrier is implied zero.
            None if implied => 0,
            None => return Err(Error::BlockMissingOverflowBits),
        }
    } else {
        0
    };

    let mantissa24 = (value << shift_needed) | low_bits;
    Ok(sign | ((exponent as u32) << 23) | (mantissa24 & 0x007f_ffff))
}

// ---------------------------------------------------------------------
// Encode side (round 418): float **origination** — the exact forward
// inverse of `reassemble_float`.
// ---------------------------------------------------------------------

/// The encode-side deconstruction of an `f32` buffer into the on-wire
/// pieces a `FLOAT_DATA` block carries: the `0x08` profile, the
/// scaled-integer stream the entropy coder compresses, and (when the
/// profile calls for it) the `0x0C` extension payload.
///
/// Produced by [`deconstruct_float`]; the exact forward inverse of
/// [`reassemble_float`] — feeding `integers` + `info` + the extension
/// bitstream back through the decoder's fixup reproduces the input
/// bit patterns exactly, and the extension payload's leading `crc_wvx`
/// equals the decoder's accumulated `crc_x` fold.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FloatDeconstruction {
    /// The derived `0x08` profile (see [`deconstruct_float`] for how
    /// the fill mode / anchor / shift are chosen).
    pub info: FloatInfo,
    /// The scaled-integer stream, one `i32` per input sample in the
    /// same order — the buffer the entropy/decorrelation pipeline
    /// compresses and the §5 block CRC folds over.
    pub integers: Vec<i32>,
    /// The complete `0x0C` sub-block payload — the 32-bit little-endian
    /// `crc_wvx` (the module-doc float extension fold over the input
    /// bit patterns) followed by the packed extension bitstream — or
    /// `None` when the profile moves no extension bits at all.
    pub extension: Option<Vec<u8>>,
}

/// Deconstruct an `f32` buffer into its scaled-integer + `0x08` +
/// `0x0C` wire form (staged spec `wavpack-sample-formats.md` §2,
/// §2.1–§2.3, mirrored from the decode pins of rounds 405/408).
///
/// Profile derivation, in the order the decode-side precedence
/// (`SENT` > `ONES` > `SAME` > zero fill) makes cheapest-first:
///
/// 1. `float_max_exp` anchors at the largest finite exponent present
///    (the round-408 pin: exceptional samples ride the bit-length-25
///    sentinel instead of the anchor). Samples whose exponent sits
///    more than 23 below the anchor — denormals included — cannot ride
///    the scaled-integer stream and are sent as `ZEROS_SENT` literals.
/// 2. The vacated low-mantissa windows (`d = anchor − exponent` bits
///    per sample) pick the fill: all-zero windows need nothing;
///    all-ones windows use `SHIFT_ONES`; per-sample-uniform windows
///    (checked at an anchor one above the largest exponent, the
///    round-408 `SHIFT_SAME` shape where every non-zero sample has a
///    non-empty window) use the one-carrier-bit `SHIFT_SAME`; anything
///    else sends the window bits literally via `SHIFT_SENT`.
/// 3. `float_shift` strips the trailing zero bits **every** scaled
///    integer shares, shrinking the entropy-coded magnitudes.
/// 4. `NEG_ZEROS` (and thus `ZEROS_SENT`) is set only when a `-0.0`
///    actually occurs; `EXCEPTIONS` only when an inf/NaN occurs.
///
/// Infallible: every `f32` bit pattern has an exact wire form.
#[must_use]
pub fn deconstruct_float(pcm: &[f32]) -> FloatDeconstruction {
    deconstruct_float_impl(pcm, false)
}

/// [`deconstruct_float`] in the **hybrid** shape (round 418):
///
/// * the exponent anchor is **raised one above** the largest finite
///   exponent, so every integer-path sample has a non-empty vacated
///   window and the scaled-integer magnitudes stay within 23 bits —
///   a coarse residual can overshoot the exact magnitude by up to
///   the §6.5 `error_limit`, and the head-room keeps the lossy
///   reconstruction inside the 24-bit mantissa window;
/// * exceptional samples (inf / NaN) ride the **`ZEROS_SENT`
///   literal path** instead of the bit-length-25 sentinel: the
///   sentinel's wvx marker only exists in the extension stream,
///   which a pair encode moves to the `.wvc` twin, so a
///   sentinel-coded exceptional sample would make the lossy `.wv`
///   undecodable on its own. As a zero-integer literal the sample
///   decodes to an implied `+0.0` in the lossy stream and to the
///   exact 32-bit pattern (NaN payload included) through the pair.
#[must_use]
pub(crate) fn deconstruct_float_raised(pcm: &[f32]) -> FloatDeconstruction {
    deconstruct_float_impl(pcm, true)
}

fn deconstruct_float_impl(pcm: &[f32], raised: bool) -> FloatDeconstruction {
    let bits: Vec<u32> = pcm.iter().map(|s| s.to_bits()).collect();
    let mut has_neg_zero = false;
    let mut has_exception = false;
    let mut has_denormal = false;
    let mut max_e: Option<u32> = None;
    for &b in &bits {
        let exp = (b >> 23) & 0xFF;
        let man = b & 0x007f_ffff;
        match (exp, man) {
            (0, 0) => has_neg_zero |= b >> 31 == 1,
            (0, _) => has_denormal = true,
            // The hybrid shape sends exceptional values as ZEROS_SENT
            // literals (see `deconstruct_float_raised`).
            (0xFF, _) if raised => has_denormal = true,
            (0xFF, _) => has_exception = true,
            _ => max_e = Some(max_e.map_or(exp, |m| m.max(exp))),
        }
    }
    let anchor_plain = max_e.unwrap_or(126);

    // Survey the vacated windows at a candidate anchor: are they all
    // zeros / all ones / per-sample uniform, and does any finite
    // sample fall out of the integer range (d > 23)?
    let survey = |anchor: u32| {
        let (mut all_zero, mut all_ones, mut uniform) = (true, true, true);
        let mut any_window = false;
        let mut any_literal = false;
        for &b in &bits {
            let exp = (b >> 23) & 0xFF;
            if exp == 0 || exp == 0xFF {
                continue;
            }
            let d = anchor - exp;
            if d > 23 {
                any_literal = true;
                continue;
            }
            if d == 0 {
                continue;
            }
            any_window = true;
            let mask = (1u32 << d) - 1;
            let w = (0x0080_0000 | (b & 0x007f_ffff)) & mask;
            all_zero &= w == 0;
            all_ones &= w == mask;
            uniform &= w == 0 || w == mask;
        }
        (all_zero, any_window && all_ones, uniform, any_literal)
    };

    let base = anchor_plain + u32::from(raised);
    let (w_zero, w_ones, w_uniform, _) = survey(base);
    let (fill, anchor) = if w_zero {
        (0u8, base)
    } else if w_ones {
        (FLOAT_SHIFT_ONES, base)
    } else if raised {
        // The raised anchor already gives every integer-path sample a
        // window, so the SHIFT_SAME carrier applies at `base` itself.
        if w_uniform {
            (FLOAT_SHIFT_SAME, base)
        } else {
            (FLOAT_SHIFT_SENT, base)
        }
    } else {
        // The round-408 `SHIFT_SAME` shape anchors one above the
        // largest exponent so every integer-path sample has a window.
        let (_, _, uniform_up, _) = survey(base + 1);
        if uniform_up {
            (FLOAT_SHIFT_SAME, base + 1)
        } else {
            (FLOAT_SHIFT_SENT, base)
        }
    };
    let (_, _, _, any_literal) = survey(anchor);

    // Common trailing zeros of the post-window scaled integers.
    let mut shift_min: Option<u32> = None;
    for &b in &bits {
        let exp = (b >> 23) & 0xFF;
        if exp == 0 || exp == 0xFF {
            continue;
        }
        let d = anchor - exp;
        if d > 23 {
            continue;
        }
        let v = (0x0080_0000 | (b & 0x007f_ffff)) >> d;
        let tz = v.trailing_zeros();
        shift_min = Some(shift_min.map_or(tz, |m| m.min(tz)));
    }
    let float_shift = shift_min.unwrap_or(0).min(23);

    let mut flags = fill;
    if any_literal || has_denormal || has_neg_zero {
        flags |= FLOAT_ZEROS_SENT;
    }
    if has_neg_zero {
        flags |= FLOAT_NEG_ZEROS;
    }
    if has_exception {
        flags |= FLOAT_EXCEPTIONS;
    }
    let info = FloatInfo {
        float_flags: flags,
        float_shift: float_shift as u8,
        float_max_exp: anchor.min(0xFF) as u8,
        float_norm_exp: 127,
    };

    let zeros_sent = flags & FLOAT_ZEROS_SENT != 0;
    let mut wvx = BitWriter::new();
    let mut crc = crate::crc::CRC_INIT;
    let mut integers = Vec::with_capacity(bits.len());
    for &b in &bits {
        crc = update_float_extension(crc, b);
        let neg = b >> 31 == 1;
        let exp = (b >> 23) & 0xFF;
        let man = b & 0x007f_ffff;
        let integer: i32 = if exp == 0xFF && raised {
            // Hybrid shape: full literal via the ZEROS_SENT path.
            wvx.write_bit(1);
            wvx.write_bits(man, 23);
            wvx.write_bits(exp, 8);
            wvx.write_bit(u32::from(neg));
            0
        } else if exp == 0xFF {
            // Exceptional sample: the bit-length-25 sentinel integer;
            // wvx carries the marker (+ NaN mantissa payload).
            if man == 0 {
                wvx.write_bit(0);
            } else {
                wvx.write_bit(1);
                wvx.write_bits(man, 23);
            }
            let mag = 1i32 << (24 - float_shift);
            if neg {
                -mag
            } else {
                mag
            }
        } else if exp == 0 || anchor - exp > 23 {
            if exp == 0 && man == 0 {
                // True ±0: integer 0; marker + optional sign under
                // ZEROS_SENT.
                if zeros_sent {
                    wvx.write_bit(0);
                    if flags & FLOAT_NEG_ZEROS != 0 {
                        wvx.write_bit(u32::from(neg));
                    }
                }
                0
            } else {
                // Too small for the scaled-integer stream (denormals
                // included): literal mantissa + exponent + sign.
                wvx.write_bit(1);
                wvx.write_bits(man, 23);
                wvx.write_bits(exp, 8);
                wvx.write_bit(u32::from(neg));
                0
            }
        } else {
            let d = anchor - exp;
            let mantissa24 = 0x0080_0000 | man;
            if d > 0 {
                let mask = (1u32 << d) - 1;
                let w = mantissa24 & mask;
                if fill == FLOAT_SHIFT_SENT {
                    wvx.write_bits(w, d);
                } else if fill == FLOAT_SHIFT_SAME {
                    wvx.write_bit(u32::from(w != 0));
                }
            }
            let mag = ((mantissa24 >> d) >> float_shift) as i32;
            if neg {
                -mag
            } else {
                mag
            }
        };
        integers.push(integer);
    }

    let wrote_bits = !wvx.is_empty();
    let extension = if info.requires_extension() || wrote_bits {
        let mut payload = crc.to_le_bytes().to_vec();
        payload.extend_from_slice(&wvx.finish());
        // The extension payload must occupy an even byte count, like
        // the 0x0A main bitstream (round-418 black-box pin: the
        // reference decoder rejects a 0x0C sub-block framed with the
        // odd-size pad flag as "not compatible"; every reference-
        // encoded 0x0C payload is even).
        if payload.len() % 2 != 0 {
            payload.push(0);
        }
        Some(payload)
    } else {
        None
    };
    FloatDeconstruction {
        info,
        integers,
        extension,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(flags: u8, shift: u8, max_exp: u8, norm_exp: u8) -> FloatInfo {
        FloatInfo {
            float_flags: flags,
            float_shift: shift,
            float_max_exp: max_exp,
            float_norm_exp: norm_exp,
        }
    }

    #[test]
    fn expand_reads_the_four_wire_fields() {
        let fi = expand_float_info(&[0x04, 9, 126, 127]).unwrap();
        assert_eq!(fi, info(0x04, 9, 126, 127));
        assert!(fi.requires_extension());
        assert!(fi.is_supported());
        let fi = expand_float_info(&[0, 9, 126, 127]).unwrap();
        assert!(!fi.requires_extension());
    }

    #[test]
    fn expand_rejects_wrong_lengths() {
        for n in [0usize, 1, 2, 3, 5, 8] {
            let payload = vec![0u8; n];
            assert_eq!(
                expand_float_info(&payload),
                Err(Error::FloatInfoLength(n)),
                "len {n}"
            );
        }
    }

    #[test]
    fn every_documented_flag_shape_is_supported() {
        // Round 408: SHIFT_SAME (§2.1) and EXCEPTIONS (§2.2) decode,
        // so no profile shape is refused any more.
        for flags in [
            0,
            FLOAT_SHIFT_SAME,
            FLOAT_EXCEPTIONS,
            FLOAT_SHIFT_SENT | FLOAT_ZEROS_SENT | FLOAT_NEG_ZEROS | FLOAT_EXCEPTIONS,
        ] {
            assert!(info(flags, 0, 126, 127).is_supported(), "flags {flags:#x}");
        }
        // SHIFT_SAME reads its per-sample carrier from the extension
        // stream, so it requires one; EXCEPTIONS alone does not (the
        // stream is only touched when an exceptional sample occurs).
        assert!(info(FLOAT_SHIFT_SAME, 0, 126, 127).requires_extension());
        assert!(!info(FLOAT_EXCEPTIONS, 0, 126, 127).requires_extension());
    }

    #[test]
    fn integer_valued_floats_reconstruct_without_an_extension_stream() {
        // The int16-sourced profile observed on reference files:
        // flags 0, shift 9, max_exp 126 (largest value 0.5), norm 127.
        let fi = info(0, 9, 126, 127);
        // 16384 << 9 = 2^23 → mantissa 0x800000, exponent 126 → 0.5.
        let mut pcm = [16384i32, -16384, 8192, 0];
        reassemble_float(&mut pcm, &fi, None).unwrap();
        assert_eq!(f32::from_bits(pcm[0] as u32), 0.5);
        assert_eq!(f32::from_bits(pcm[1] as u32), -0.5);
        assert_eq!(f32::from_bits(pcm[2] as u32), 0.25);
        assert_eq!(pcm[3], 0, "implied zero is +0.0");
    }

    #[test]
    fn shift_ones_fills_the_vacated_low_bits() {
        let fi = info(FLOAT_SHIFT_ONES, 0, 126, 127);
        let mut pcm = [1i32 << 20]; // bit_length 21 → 3 vacated bits
        reassemble_float(&mut pcm, &fi, None).unwrap();
        let bits = pcm[0] as u32;
        assert_eq!(bits & 0x7, 0x7, "low bits are ones");
        assert_eq!((bits >> 23) & 0xff, 123, "exponent 126 - 3");
    }

    #[test]
    fn shift_sent_reads_the_vacated_bits_from_the_extension_stream() {
        let fi = info(FLOAT_SHIFT_SENT, 0, 126, 127);
        // One sample of bit_length 21 → 3 literal bits, LSB-first.
        let ext_bytes = [0b101u8];
        let mut reader = BitReader::new(&ext_bytes);
        let mut pcm = [1i32 << 20];
        reassemble_float(&mut pcm, &fi, Some(&mut reader)).unwrap();
        let bits = pcm[0] as u32;
        assert_eq!(bits & 0x7, 0b101, "literal low bits");
    }

    #[test]
    fn shift_sent_without_a_stream_is_refused() {
        let fi = info(FLOAT_SHIFT_SENT, 0, 126, 127);
        let mut pcm = [1i32 << 20];
        assert_eq!(
            reassemble_float(&mut pcm, &fi, None),
            Err(Error::BlockMissingOverflowBits)
        );
    }

    #[test]
    fn zeros_sent_marker_bit_selects_a_zero_or_a_literal() {
        // Round-405 black-box pin: marker 0 → ±0 (sign bit only under
        // NEG_ZEROS); marker 1 → literal mantissa23 + exponent8 +
        // sign1.
        let fi = info(FLOAT_ZEROS_SENT | FLOAT_NEG_ZEROS, 0, 126, 127);
        // Sample 1: marker 0, sign 0 → +0.0.
        // Sample 2: marker 0, sign 1 → -0.0.
        // Sample 3: marker 1, mantissa 0x123, exponent 0, sign 0 → the
        //           denormal 0x00000123.
        let mut wire_bits = vec![0u8, 0, 0, 1, 1];
        // mantissa 0x123 LSB-first (23 bits)...
        for k in 0..23 {
            wire_bits.push(((0x123u32 >> k) & 1) as u8);
        }
        // ...exponent 0 (8 bits), sign 0.
        wire_bits.extend(std::iter::repeat_n(0u8, 9));
        let mut bytes = vec![0u8; wire_bits.len().div_ceil(8)];
        for (i, &b) in wire_bits.iter().enumerate() {
            bytes[i / 8] |= b << (i % 8);
        }
        let mut reader = BitReader::new(&bytes);
        let mut pcm = [0i32, 0, 0];
        reassemble_float(&mut pcm, &fi, Some(&mut reader)).unwrap();
        assert_eq!(pcm[0] as u32, 0, "+0.0");
        assert_eq!(pcm[1] as u32, 0x8000_0000, "-0.0");
        assert_eq!(pcm[2] as u32, 0x0000_0123, "literal denormal");
    }

    #[test]
    fn zeros_without_neg_zeros_read_no_sign_bit() {
        let fi = info(FLOAT_ZEROS_SENT, 0, 126, 127);
        // Two zero samples: marker 0 each, no sign bits — exactly two
        // wire bits consumed.
        let bytes = [0b00u8];
        let mut reader = BitReader::new(&bytes);
        let mut pcm = [0i32, 0];
        reassemble_float(&mut pcm, &fi, Some(&mut reader)).unwrap();
        assert_eq!(pcm, [0, 0]);
        assert_eq!(reader.bits_consumed(), 2);
    }

    #[test]
    fn shift_same_carrier_bit_selects_all_ones_or_all_zeros() {
        // §2.1: one carrier bit per non-zero sample fills the vacated
        // window — 1 = ones, 0 = zeros. Zero samples spend no bit.
        let fi = info(FLOAT_SHIFT_SAME, 0, 126, 127);
        // Samples: 1<<20 (3 vacated bits, carrier 1), 0 (no bit),
        // 1<<22 (1 vacated bit, carrier 0), 1<<23 (0 vacated bits —
        // no bit read; reference streams anchor max_exp one above the
        // largest exponent so this is the boundary case).
        let wire = [0b01u8]; // bit0 = 1 (first sample), bit1 = 0 (third)
        let mut reader = BitReader::new(&wire);
        let mut pcm = [1i32 << 20, 0, 1 << 22, 1 << 23];
        reassemble_float(&mut pcm, &fi, Some(&mut reader)).unwrap();
        assert_eq!(pcm[0] as u32 & 0x7, 0x7, "carrier 1 fills with ones");
        assert_eq!(pcm[1], 0, "zero sample is implied +0.0");
        assert_eq!(pcm[2] as u32 & 0x1, 0, "carrier 0 fills with zeros");
        assert_eq!((pcm[3] as u32 >> 23) & 0xFF, 126, "full-window sample");
        assert_eq!(
            reader.bits_consumed(),
            2,
            "exactly one bit per fillable sample"
        );
    }

    #[test]
    fn shift_same_without_a_stream_is_refused() {
        let fi = info(FLOAT_SHIFT_SAME, 0, 126, 127);
        let mut pcm = [1i32 << 20];
        assert_eq!(
            reassemble_float(&mut pcm, &fi, None),
            Err(Error::BlockMissingOverflowBits)
        );
    }

    #[test]
    fn exceptions_sentinel_reads_marker_and_nan_mantissa() {
        // §2.2 + round-408 pin: bit-length-25 sentinel = exceptional;
        // wvx marker 0 = infinity, 1 = 23-bit NaN mantissa LSB-first.
        let fi = info(FLOAT_EXCEPTIONS, 0, 126, 127);
        // Wire: marker 0 (+inf), marker 0 (-inf), marker 1 + 0x155555.
        let mut bits = vec![0u8, 0, 1];
        bits.extend((0..23).map(|k| ((0x155555u32 >> k) & 1) as u8));
        let mut bytes = vec![0u8; bits.len().div_ceil(8)];
        for (i, &b) in bits.iter().enumerate() {
            bytes[i / 8] |= b << (i % 8);
        }
        let mut reader = BitReader::new(&bytes);
        let mut pcm = [1i32 << 24, -(1i32 << 24), 1 << 24, 16384];
        reassemble_float(&mut pcm, &fi, Some(&mut reader)).unwrap();
        assert_eq!(pcm[0] as u32, 0x7F80_0000, "+infinity");
        assert_eq!(pcm[1] as u32, 0xFF80_0000, "-infinity (normal sign path)");
        assert_eq!(pcm[2] as u32, 0x7F95_5555, "NaN payload preserved");
        // A finite sample in the same block still reconstructs
        // normally (16384 → bit_length 15 → denormal-free path).
        assert_eq!((pcm[3] as u32 >> 23) & 0xFF, 117, "finite exponent anchor");
    }

    #[test]
    fn exceptions_respect_the_static_shift_in_the_sentinel() {
        // The sentinel is 1 << 24 *after* the static shift: with
        // float_shift = 23 the on-wire integer is ±2 (round-408 pin).
        let fi = info(FLOAT_EXCEPTIONS, 23, 126, 127);
        let bytes = [0b0u8];
        let mut reader = BitReader::new(&bytes);
        let mut pcm = [2i32];
        reassemble_float(&mut pcm, &fi, Some(&mut reader)).unwrap();
        assert_eq!(pcm[0] as u32, 0x7F80_0000);
    }

    #[test]
    fn exception_without_a_stream_is_refused() {
        let fi = info(FLOAT_EXCEPTIONS, 0, 126, 127);
        let mut pcm = [1i32 << 24];
        assert_eq!(
            reassemble_float(&mut pcm, &fi, None),
            Err(Error::BlockMissingOverflowBits)
        );
    }

    #[test]
    fn deep_overflow_is_still_refused_with_exceptions_set() {
        // Only bit length exactly 25 is the sentinel; deeper overflow
        // is malformed even on an EXCEPTIONS-capable profile.
        let fi = info(FLOAT_EXCEPTIONS, 0, 126, 127);
        let bytes = [0u8];
        let mut reader = BitReader::new(&bytes);
        let mut pcm = [1i32 << 26];
        assert_eq!(
            reassemble_float(&mut pcm, &fi, Some(&mut reader)),
            Err(Error::FloatMagnitudeOverflow(1u32 << 26))
        );
    }

    #[test]
    fn magnitude_overflowing_the_mantissa_window_is_refused() {
        let fi = info(0, 9, 126, 127);
        let mut pcm = [1i32 << 22]; // << 9 → bit_length 32 > 24
        assert_eq!(
            reassemble_float(&mut pcm, &fi, None),
            Err(Error::FloatMagnitudeOverflow((1u32 << 22) << 9))
        );
    }

    // ---- encode side: deconstruct_float (round 418) -------------------

    /// Push a deconstruction back through the decoder's fixup and
    /// assert bit-exactness plus the crc_wvx agreement.
    fn assert_deconstruction_round_trips(pcm: &[f32]) -> FloatInfo {
        let d = deconstruct_float(pcm);
        let mut slots = d.integers.clone();
        match &d.extension {
            Some(payload) => {
                let stored = u32::from_le_bytes(payload[..4].try_into().unwrap());
                let mut reader = BitReader::new(&payload[4..]);
                let crc = reassemble_float(&mut slots, &d.info, Some(&mut reader)).unwrap();
                assert_eq!(crc, stored, "crc_wvx must match the decoder's fold");
            }
            None => {
                reassemble_float(&mut slots, &d.info, None).unwrap();
            }
        }
        let got: Vec<u32> = slots.iter().map(|&s| s as u32).collect();
        let want: Vec<u32> = pcm.iter().map(|s| s.to_bits()).collect();
        assert_eq!(got, want, "bit patterns must survive the round trip");
        d.info
    }

    #[test]
    fn deconstruct_integer_valued_floats_needs_no_extension() {
        // Scaled-integer content: all vacated windows are zero, so the
        // profile needs neither fill bits nor an extension stream.
        let pcm = [0.5f32, -0.5, 0.25, 0.0, 0.125];
        let info = assert_deconstruction_round_trips(&pcm);
        assert_eq!(info.float_flags, 0);
        assert_eq!(info.float_max_exp, 126);
        assert!(deconstruct_float(&pcm).extension.is_none());
    }

    #[test]
    fn deconstruct_full_precision_floats_uses_shift_sent() {
        // Noisy mantissas: the windows are non-uniform, so the literal
        // SHIFT_SENT profile carries them in the extension stream.
        let mut pcm = Vec::new();
        let mut x = 0x9e3779b97f4a7c15u64;
        for _ in 0..200 {
            x = x.wrapping_mul(0xd1342543de82ef95).wrapping_add(1);
            let exp = 120 + ((x >> 58) as u32 & 7); // exponent spread
            let sign = ((x & 1) as u32) << 31;
            let bits = sign | (exp << 23) | ((x >> 40) as u32 & 0x007f_ffff);
            pcm.push(f32::from_bits(bits));
        }
        let info = assert_deconstruction_round_trips(&pcm);
        assert_eq!(info.float_flags & FLOAT_SHIFT_SENT, FLOAT_SHIFT_SENT);
    }

    #[test]
    fn deconstruct_uniform_windows_uses_shift_same_with_raised_anchor() {
        // Windows all-ones or all-zeros per sample (but mixed across
        // samples): the SHIFT_SAME carrier applies, anchored one above
        // the largest exponent (the round-408 reference shape).
        let pcm = [
            0.5f32,                      // exp 126: window all zeros
            0.25f32,                     // exp 125, mantissa 0: zeros
            f32::from_bits(0x3EFF_FFFF), // exp 125, mantissa 0x7fffff: ones
            -0.5f32,
            0.0f32,
        ];
        let info = assert_deconstruction_round_trips(&pcm);
        assert_eq!(info.float_flags & FLOAT_SHIFT_SAME, FLOAT_SHIFT_SAME);
        assert_eq!(info.float_max_exp, 127, "anchor one above the largest e");
    }

    #[test]
    fn deconstruct_zeros_and_denormals_use_the_literal_path() {
        let pcm = [
            1.0f32,
            0.0,
            -0.0,
            f32::from_bits(0x0000_0123), // denormal
            f32::from_bits(0x8000_4567), // negative denormal
            1.0e-30,                     // far below the anchor: literal
        ];
        let info = assert_deconstruction_round_trips(&pcm);
        assert_eq!(
            info.float_flags & (FLOAT_ZEROS_SENT | FLOAT_NEG_ZEROS),
            FLOAT_ZEROS_SENT | FLOAT_NEG_ZEROS
        );
    }

    #[test]
    fn deconstruct_exceptions_ride_the_sentinel() {
        let pcm = [
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::from_bits(0x7f95_5555), // NaN with a payload
            f32::from_bits(0xffc0_0001), // negative NaN
            0.75,
            -0.375,
        ];
        let info = assert_deconstruction_round_trips(&pcm);
        assert_eq!(info.float_flags & FLOAT_EXCEPTIONS, FLOAT_EXCEPTIONS);
    }

    #[test]
    fn deconstruct_all_zero_buffer_is_trivial() {
        let pcm = [0.0f32; 16];
        let info = assert_deconstruction_round_trips(&pcm);
        assert_eq!(info.float_flags, 0);
        assert!(deconstruct_float(&pcm).integers.iter().all(|&i| i == 0));
    }

    #[test]
    fn deconstruct_common_trailing_zeros_become_float_shift() {
        // int16-shaped: every scaled integer shares trailing zeros,
        // which the static shift strips (the reference int16 profile
        // shape: flags 0, non-zero shift).
        let pcm: Vec<f32> = (1..=64).map(|k| k as f32 / 32768.0).collect();
        let info = assert_deconstruction_round_trips(&pcm);
        assert_eq!(info.float_flags, 0);
        assert!(info.float_shift > 0, "shared trailing zeros stripped");
    }

    #[test]
    fn deconstruct_wide_dynamic_range_round_trips() {
        // Exponents spanning more than the 23-bit window: the low ones
        // fall out of the integer range and ride the literal path.
        let pcm = [1.0e30f32, 1.0, 1.0e-30, -1.0e-38, 3.5e28, -7.0];
        let info = assert_deconstruction_round_trips(&pcm);
        assert_eq!(info.float_flags & FLOAT_ZEROS_SENT, FLOAT_ZEROS_SENT);
    }

    #[test]
    fn crc_x_folds_mantissa_exponent_sign_triples() {
        let fi = info(0, 9, 126, 127);
        let mut pcm = [16384i32, -16384];
        let crc = reassemble_float(&mut pcm, &fi, None).unwrap();
        let mut expect = crate::crc::CRC_INIT;
        for bits in [0.5f32.to_bits(), (-0.5f32).to_bits()] {
            expect = update_float_extension(expect, bits);
        }
        assert_eq!(crc, expect);
        // The per-field weights the differential black-box probes
        // pinned: a mantissa bit flip moves the register by 9 << k, an
        // exponent step by 3, the sign by 1 (single-sample fold).
        let base = update_float_extension(0, 0x3f40_0000);
        assert_eq!(update_float_extension(0, 0x3f40_0001).wrapping_sub(base), 9);
        assert_eq!(update_float_extension(0, 0x3fc0_0000).wrapping_sub(base), 3);
        assert_eq!(update_float_extension(0, 0xbf40_0000).wrapping_sub(base), 1);
    }
}
