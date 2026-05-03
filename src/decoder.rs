//! WavPack lossless block decoder.
//!
//! The pipeline matches spec §5:
//!
//! 1. Pull a residual integer from the entropy stream (§5.4).
//! 2. Apply the cascade of `terms` decorrelation passes in *reverse*
//!    encoder order (§5.2).
//! 3. Apply joint-stereo / cross-channel decorrelation undo (§5.3).
//! 4. Apply the `INT32INFO` post-shift (§4.6).
//! 5. Update the per-block CRC (§5.1) and verify against the header.

use oxideav_core::bits::BitReaderLsb;
use oxideav_core::{Error, Result};

use crate::block::{
    BlockHeader, SubBlock, WP_ID_DATA, WP_ID_DECSAMPLES, WP_ID_DECTERMS, WP_ID_DECWEIGHTS,
    WP_ID_ENTROPY, WP_ID_EXTRABITS, WP_ID_INT32INFO,
};
use crate::entropy::EntropyDecoder;
use crate::log2::wp_exp2;

/// Per-pass decorrelation term + delta + per-channel weights + history.
#[derive(Debug, Clone)]
pub struct DecorrPass {
    pub term: i32,
    pub delta: u32,
    /// Per-channel weight (in Q10). Stereo blocks have both filled;
    /// mono blocks only use slot 0.
    pub weight: [i32; 2],
    /// Per-channel history buffer. Maximum depth is `max_term_history`.
    /// History is indexed `H[0]` = most recent, `H[1]` = one back, etc.
    pub history: [Vec<i32>; 2],
}

impl DecorrPass {
    /// History depth required for `term` (per spec §4.4 table).
    pub fn history_depth(term: i32) -> usize {
        match term {
            1..=8 => term as usize,
            17 | 18 => 2,
            -1 | -2 | -3 => 1,
            _ => 0,
        }
    }
}

/// `INT32INFO` (spec §4.6): controls per-sample LSB restoration plus
/// optional `EXTRABITS` payload width.
#[derive(Debug, Clone, Copy, Default)]
pub struct Int32Info {
    pub sent_bits: u8,
    pub zero_bits: u8,
    pub ones_bits: u8,
    pub dup_bits: u8,
}

impl Int32Info {
    pub fn parse(buf: &[u8]) -> Result<Self> {
        if buf.len() != 4 {
            return Err(Error::invalid(format!(
                "WavPack INT32INFO: expected 4 bytes, got {}",
                buf.len()
            )));
        }
        Ok(Self {
            sent_bits: buf[0],
            zero_bits: buf[1],
            ones_bits: buf[2],
            dup_bits: buf[3],
        })
    }

    /// Post-shift left-count and (and, or) bit-pair per spec §4.6.
    /// Returns `(shift, and, or)`.
    pub fn shift_and_or(&self) -> (u32, i32, i32) {
        if self.zero_bits != 0 {
            (self.zero_bits as u32, 0, 0)
        } else if self.ones_bits != 0 {
            (self.ones_bits as u32, 1, 1)
        } else if self.dup_bits != 0 {
            (self.dup_bits as u32, 1, 0)
        } else {
            (0, 0, 0)
        }
    }
}

/// Decode `WP_ID_DECTERMS`: one byte per pass, encoder-application
/// order. Output is in *reverse* (decoder-application) order so the
/// caller can iterate `0..N` to undo passes last-to-first.
pub fn parse_decterms(buf: &[u8]) -> Result<Vec<(i32, u32)>> {
    let mut out: Vec<(i32, u32)> = Vec::with_capacity(buf.len());
    for &byte in buf.iter().rev() {
        let term = ((byte & 0x1F) as i32) - 5;
        let delta = ((byte >> 5) & 0x7) as u32;
        // Validate: legal term set is {-3, -2, -1, 17, 18, 1..=8}.
        if !matches!(term, -3 | -2 | -1 | 17 | 18 | 1..=8) {
            return Err(Error::invalid(format!(
                "WavPack DECTERMS: illegal term {term}"
            )));
        }
        out.push((term, delta));
    }
    Ok(out)
}

/// Decode `WP_ID_DECWEIGHTS`: one signed byte per active weight, in
/// encoder-application order; stereo emits two bytes per term, mono
/// emits one. Trailing zero-weight terms may be omitted.
///
/// Returns weights indexed by **decoder-application order** (i.e.
/// `weights[0]` is for the *last* encoder pass, matching how
/// [`parse_decterms`] reverses its term list). This keeps all
/// per-pass arrays in lockstep.
pub fn parse_decweights(buf: &[u8], n_terms: usize, channels: usize) -> Vec<[i32; 2]> {
    let mut weights = vec![[0i32; 2]; n_terms];
    // We fill `weights[encoder_idx]` first, then reverse at the end.
    let mut idx = 0usize;
    let mut bytes_read = 0usize;
    while bytes_read < buf.len() && idx < n_terms {
        for c in 0..channels {
            if bytes_read >= buf.len() {
                break;
            }
            let raw = buf[bytes_read] as i8 as i32;
            let mut w8 = raw * 8;
            if w8 > 0 {
                w8 += (w8 + 64) >> 7;
            }
            weights[idx][c] = w8;
            bytes_read += 1;
        }
        idx += 1;
    }
    weights.reverse();
    weights
}

/// Decode `WP_ID_DECSAMPLES`: warm-up history per pass. Stored as
/// 16-bit signed log-domain values; `wp_exp2` recovers each sample.
///
/// Layout per pass (in encoder-application order, same as DECTERMS):
///
/// * `term ∈ 1..=8`:  for stereo, 2 samples × 2 channels (one each from
///   first 2 history slots — remaining slots zero on disk;
///   the FFmpeg decoder zero-fills past index 1). For mono,
///   2 samples × 1 channel.
/// * `term ∈ {17, 18}`:  2 samples × `channels` (4 / 2 bytes).
/// * `term ∈ {-1, -2, -3}`: 1 sample × `channels` (4 / 2 bytes).
///
/// If the encoder truncated trailing zero passes (matching DECWEIGHTS
/// behaviour), the missing samples are zero-filled.
pub fn parse_decsamples(
    buf: &[u8],
    terms: &[(i32, u32)],
    channels: usize,
) -> Result<Vec<[Vec<i32>; 2]>> {
    let mut histories: Vec<[Vec<i32>; 2]> = terms
        .iter()
        .map(|(t, _)| {
            let depth = DecorrPass::history_depth(*t);
            [vec![0i32; depth], vec![0i32; depth]]
        })
        .collect();

    let mut byte_pos = 0usize;
    // Iterate in encoder-application order — that is the order in
    // which DECSAMPLES bytes are laid out on disk. `terms` here is
    // already reversed (decoder-application order) so we walk it back.
    for (decoder_idx, (term, _)) in terms.iter().enumerate().rev() {
        let depth = DecorrPass::history_depth(*term);
        if depth == 0 {
            continue;
        }
        // Per spec §4.4 the on-disk count is fixed at 2 samples per
        // channel for `term ∈ {17, 18}` and 1 for `term ∈ {-1,-2,-3}`.
        // For `term ∈ 1..=8` it is 2 samples per channel (only the
        // first two history slots are stored; deeper slots stay zero).
        let stored_per_channel: usize = match *term {
            1..=8 => 2.min(depth),
            17 | 18 => 2,
            -1 | -2 | -3 => 1,
            _ => unreachable!("validated in parse_decterms"),
        };
        for c in 0..channels {
            for slot in 0..stored_per_channel {
                if byte_pos + 2 > buf.len() {
                    // Truncated: leave remaining as zero.
                    return Ok(histories);
                }
                let lg = i16::from_le_bytes([buf[byte_pos], buf[byte_pos + 1]]);
                byte_pos += 2;
                histories[decoder_idx][c][slot] = wp_exp2(lg);
            }
        }
    }
    Ok(histories)
}

/// Decode `WP_ID_ENTROPY`: 6 bytes per channel (3 log-domain medians,
/// 16-bit LE each).
pub fn parse_entropy_log_medians(buf: &[u8], channels: usize) -> Result<Vec<i16>> {
    let need = 6 * channels;
    if buf.len() < need {
        return Err(Error::invalid(format!(
            "WavPack ENTROPY: expected {need} bytes, got {}",
            buf.len()
        )));
    }
    let mut out = Vec::with_capacity(3 * channels);
    for c in 0..channels {
        for j in 0..3 {
            let off = c * 6 + j * 2;
            out.push(i16::from_le_bytes([buf[off], buf[off + 1]]));
        }
    }
    Ok(out)
}

/// Apply post-shift restoration to a freshly-decoded sample per
/// `INT32INFO` semantics: `S = (S << shift) | ((S & and) ^ or)`.
pub fn apply_int32_postshift(s: i32, info: &Int32Info) -> i32 {
    let (shift, and, or) = info.shift_and_or();
    if shift == 0 {
        s
    } else if shift >= 32 {
        0
    } else {
        let pre = (s & and) ^ or;
        (s << shift) | pre
    }
}

/// Apply the global `SHIFT` field's post-decode left-shift.
pub fn apply_global_shift(s: i32, shift: u32) -> i32 {
    if shift == 0 {
        s
    } else if shift >= 32 {
        0
    } else {
        s << shift
    }
}

/// CRC update per spec §5.1: `crc = crc * 3 + sample` (mod 2^32).
#[inline]
pub fn crc_update(crc: u32, sample: i32) -> u32 {
    crc.wrapping_mul(3).wrapping_add(sample as u32)
}

/// Q10 weight clip range per spec §4.2.1.
const WEIGHT_MIN: i32 = -1024;
const WEIGHT_MAX: i32 = 1024;

/// Update an LMS-style decorrelation weight after one sample.
/// Sign convention matches spec §5.2: `weight ± delta` with sign
/// chosen by `sign(residual) * sign(history-sample)`.
#[inline]
fn update_weight(weight: i32, delta: i32, sample_sign: i32, hist_sign: i32) -> i32 {
    let sign = sample_sign * hist_sign;
    let new = if sign > 0 {
        weight + delta
    } else if sign < 0 {
        weight - delta
    } else {
        weight
    };
    new.clamp(WEIGHT_MIN, WEIGHT_MAX)
}

#[inline]
fn sign_i32(v: i32) -> i32 {
    if v > 0 {
        1
    } else if v < 0 {
        -1
    } else {
        0
    }
}

/// Apply `(weight * value + 512) >> 10` rounded toward nearest, per
/// spec §4.2.1.
#[inline]
fn apply_weight(weight: i32, value: i32) -> i32 {
    if value == 0 || weight == 0 {
        return 0;
    }
    let prod = (weight as i64) * (value as i64);
    ((prod + 512) >> 10) as i32
}

/// Push a freshly-reconstructed sample into a depth-`d` history ring.
/// `H[0]` is the most recent; `H[1]..=H[d-1]` shift down by one.
fn push_history(hist: &mut Vec<i32>, sample: i32) {
    if hist.is_empty() {
        return;
    }
    for i in (1..hist.len()).rev() {
        hist[i] = hist[i - 1];
    }
    hist[0] = sample;
}

/// Reconstruct one sample-frame's worth of channels from a single
/// WavPack PCM block. The output is one `Vec<i32>` per channel
/// (block-local channel order: mono = 1, stereo = 2).
///
/// Lossless mode only — hybrid / DSD / float are rejected at the
/// public entry points.
pub fn decode_block_samples(
    header: &BlockHeader,
    sub_blocks: &[SubBlock<'_>],
) -> Result<Vec<Vec<i32>>> {
    decode_block_samples_inner(header, sub_blocks, true)
}

/// Same as [`decode_block_samples`] but skips the per-block CRC check.
/// Used by the diagnostic `tests/inspect.rs` to dump the (possibly
/// wrong) decoder output side-by-side with the source.
#[doc(hidden)]
pub fn decode_block_samples_no_crc(
    header: &BlockHeader,
    sub_blocks: &[SubBlock<'_>],
) -> Result<Vec<Vec<i32>>> {
    decode_block_samples_inner(header, sub_blocks, false)
}

fn decode_block_samples_inner(
    header: &BlockHeader,
    sub_blocks: &[SubBlock<'_>],
    check_crc: bool,
) -> Result<Vec<Vec<i32>>> {
    let channels = header.channels_in_block() as usize;
    let n_samples = header.block_samples as usize;

    // Locate sub-blocks by id.
    let mut decterms_buf: Option<&[u8]> = None;
    let mut decweights_buf: Option<&[u8]> = None;
    let mut decsamples_buf: Option<&[u8]> = None;
    let mut entropy_buf: Option<&[u8]> = None;
    let mut data_buf: Option<&[u8]> = None;
    let mut int32info: Option<Int32Info> = None;
    let mut extrabits_buf: Option<&[u8]> = None;

    for sb in sub_blocks {
        match sb.ty() {
            WP_ID_DECTERMS => decterms_buf = Some(sb.data),
            WP_ID_DECWEIGHTS => decweights_buf = Some(sb.data),
            WP_ID_DECSAMPLES => decsamples_buf = Some(sb.data),
            WP_ID_ENTROPY => entropy_buf = Some(sb.data),
            WP_ID_DATA => data_buf = Some(sb.data),
            WP_ID_INT32INFO => int32info = Some(Int32Info::parse(sb.data)?),
            WP_ID_EXTRABITS => extrabits_buf = Some(sb.data),
            _ => {} // ignore the rest in lossless mode (CHANINFO etc handled at the frame level)
        }
    }

    let decterms_buf =
        decterms_buf.ok_or_else(|| Error::invalid("WavPack: missing DECTERMS sub-block"))?;
    let entropy_buf =
        entropy_buf.ok_or_else(|| Error::invalid("WavPack: missing ENTROPY sub-block"))?;
    let data_buf = data_buf.ok_or_else(|| Error::invalid("WavPack: missing DATA sub-block"))?;

    let terms = parse_decterms(decterms_buf)?;
    let weights = if let Some(w) = decweights_buf {
        parse_decweights(w, terms.len(), channels)
    } else {
        vec![[0i32; 2]; terms.len()]
    };
    let mut histories = if let Some(s) = decsamples_buf {
        parse_decsamples(s, &terms, channels)?
    } else {
        terms
            .iter()
            .map(|(t, _)| {
                let d = DecorrPass::history_depth(*t);
                [vec![0i32; d], vec![0i32; d]]
            })
            .collect()
    };

    let log_medians = parse_entropy_log_medians(entropy_buf, channels)?;
    let mut entropy = EntropyDecoder::new(channels as u32, &log_medians)?;

    // Convert the weight slices into per-pass mutable arrays.
    let mut passes: Vec<DecorrPass> = terms
        .iter()
        .enumerate()
        .map(|(i, (t, d))| DecorrPass {
            term: *t,
            delta: *d,
            weight: weights[i],
            history: [
                std::mem::take(&mut histories[i][0]),
                std::mem::take(&mut histories[i][1]),
            ],
        })
        .collect();

    // Per-channel `WP_ID_EXTRABITS` reader (lossless integer modes).
    // Round 1 supports `INT32INFO.sent_bits > 0` for s32 streams.
    let (mut extra_br, extra_per_sample) = if let Some(buf) = extrabits_buf {
        if buf.len() < 4 {
            return Err(Error::invalid("WavPack EXTRABITS: missing 4-byte CRC"));
        }
        let bits = int32info.map(|i| i.sent_bits).unwrap_or(0) as u32;
        // Skip the 4-byte CRC header (we don't validate it in round 1).
        (Some(BitReaderLsb::new(&buf[4..])), bits)
    } else {
        (None, 0)
    };

    let mut out: Vec<Vec<i32>> = (0..channels)
        .map(|_| Vec::with_capacity(n_samples))
        .collect();
    let mut br = BitReaderLsb::new(data_buf);

    let mut crc: u32 = 0xFFFF_FFFF;

    for _ in 0..n_samples {
        // Decode the residuals for this sample-frame.
        let mut sf: [i32; 2] = [0; 2];
        for c in 0..channels {
            sf[c] = entropy.decode_sample(&mut br, c)?;
        }

        // Cross-channel and same-channel decorrelation undo, applied
        // last-to-first (passes are stored in reverse encoder order
        // by `parse_decterms`).
        for pass in passes.iter_mut() {
            apply_pass_undo(pass, &mut sf, channels);
        }

        // Joint-stereo undo (only meaningful when stereo + JS bit set).
        if channels == 2 && header.is_joint_stereo() {
            // Spec §5.3:  R = R' - (L' >> 1);   L = R + L'.
            sf[1] = sf[1].wrapping_sub(sf[0] >> 1);
            sf[0] = sf[1].wrapping_add(sf[0]);
        }

        // EXTRABITS (s32 only — append `sent_bits` LSBs per sample).
        if extra_per_sample > 0 {
            if let Some(br_e) = extra_br.as_mut() {
                for c in 0..channels {
                    let lsbs = br_e.read_u32(extra_per_sample)?;
                    sf[c] = (sf[c] << extra_per_sample) | (lsbs as i32);
                }
            }
        }

        // INT32INFO post-shift restoration.
        if let Some(info) = int32info {
            for c in 0..channels {
                sf[c] = apply_int32_postshift(sf[c], &info);
            }
        }

        // Global flag-shift post-decode.
        let g_shift = header.shift_count();
        if g_shift != 0 {
            for c in 0..channels {
                sf[c] = apply_global_shift(sf[c], g_shift);
            }
        }

        // CRC update (over decoded samples).
        for c in 0..channels {
            crc = crc_update(crc, sf[c]);
        }

        if channels == 1 {
            out[0].push(sf[0]);
        } else {
            out[0].push(sf[0]);
            out[1].push(sf[1]);
        }
    }

    if check_crc && crc != header.crc {
        return Err(Error::invalid(format!(
            "WavPack: CRC mismatch (computed {:#010x} vs header {:#010x})",
            crc, header.crc
        )));
    }
    // For false-stereo blocks the decoder must duplicate the single
    // channel back to two.
    if header.is_false_stereo() && out.len() == 1 {
        let dup = out[0].clone();
        out.push(dup);
    }
    Ok(out)
}

/// Undo a single decorrelation pass on the current sample-frame.
fn apply_pass_undo(pass: &mut DecorrPass, sf: &mut [i32; 2], channels: usize) {
    match pass.term {
        1..=8 => {
            let lag = pass.term as usize;
            for c in 0..channels {
                if pass.history[c].len() < lag {
                    continue;
                }
                let pred_value = pass.history[c][lag - 1];
                let pred = apply_weight(pass.weight[c], pred_value);
                let recovered = sf[c].wrapping_add(pred);
                pass.weight[c] = update_weight(
                    pass.weight[c],
                    pass.delta as i32,
                    sign_i32(sf[c]),
                    sign_i32(pred_value),
                );
                push_history(&mut pass.history[c], recovered);
                sf[c] = recovered;
            }
        }
        17 => {
            // pred = 2*H[0] - H[1]
            for c in 0..channels {
                if pass.history[c].len() < 2 {
                    continue;
                }
                let pred_value = pass.history[c][0]
                    .wrapping_mul(2)
                    .wrapping_sub(pass.history[c][1]);
                let pred = apply_weight(pass.weight[c], pred_value);
                let recovered = sf[c].wrapping_add(pred);
                pass.weight[c] = update_weight(
                    pass.weight[c],
                    pass.delta as i32,
                    sign_i32(sf[c]),
                    sign_i32(pred_value),
                );
                push_history(&mut pass.history[c], recovered);
                sf[c] = recovered;
            }
        }
        18 => {
            // pred = (3*H[0] - H[1]) >> 1
            for c in 0..channels {
                if pass.history[c].len() < 2 {
                    continue;
                }
                let pred_value = pass.history[c][0]
                    .wrapping_mul(3)
                    .wrapping_sub(pass.history[c][1])
                    >> 1;
                let pred = apply_weight(pass.weight[c], pred_value);
                let recovered = sf[c].wrapping_add(pred);
                pass.weight[c] = update_weight(
                    pass.weight[c],
                    pass.delta as i32,
                    sign_i32(sf[c]),
                    sign_i32(pred_value),
                );
                push_history(&mut pass.history[c], recovered);
                sf[c] = recovered;
            }
        }
        -1 => {
            // Cross: A → B then B → A
            // Stereo only — pred for ch0 uses ch1 latest, then for
            // ch1 uses the freshly-updated ch0.
            if channels < 2 {
                return;
            }
            // ch 0 first
            let pred_a_value = if pass.history[1].is_empty() {
                0
            } else {
                pass.history[1][0]
            };
            let pred_a = apply_weight(pass.weight[0], pred_a_value);
            let new_a = sf[0].wrapping_add(pred_a);
            pass.weight[0] = update_weight(
                pass.weight[0],
                pass.delta as i32,
                sign_i32(sf[0]),
                sign_i32(pred_a_value),
            );
            sf[0] = new_a;
            push_history(&mut pass.history[0], new_a);
            // ch 1 uses updated ch0
            let pred_b_value = if pass.history[0].is_empty() {
                0
            } else {
                pass.history[0][0]
            };
            let pred_b = apply_weight(pass.weight[1], pred_b_value);
            let new_b = sf[1].wrapping_add(pred_b);
            pass.weight[1] = update_weight(
                pass.weight[1],
                pass.delta as i32,
                sign_i32(sf[1]),
                sign_i32(pred_b_value),
            );
            sf[1] = new_b;
            push_history(&mut pass.history[1], new_b);
        }
        -2 => {
            // Mirror of -1: B → A then A → B.
            if channels < 2 {
                return;
            }
            // ch 1 first using ch0 latest
            let pred_b_value = if pass.history[0].is_empty() {
                0
            } else {
                pass.history[0][0]
            };
            let pred_b = apply_weight(pass.weight[1], pred_b_value);
            let new_b = sf[1].wrapping_add(pred_b);
            pass.weight[1] = update_weight(
                pass.weight[1],
                pass.delta as i32,
                sign_i32(sf[1]),
                sign_i32(pred_b_value),
            );
            sf[1] = new_b;
            push_history(&mut pass.history[1], new_b);
            // ch 0 uses updated ch1
            let pred_a_value = if pass.history[1].is_empty() {
                0
            } else {
                pass.history[1][0]
            };
            let pred_a = apply_weight(pass.weight[0], pred_a_value);
            let new_a = sf[0].wrapping_add(pred_a);
            pass.weight[0] = update_weight(
                pass.weight[0],
                pass.delta as i32,
                sign_i32(sf[0]),
                sign_i32(pred_a_value),
            );
            sf[0] = new_a;
            push_history(&mut pass.history[0], new_a);
        }
        -3 => {
            // Two-way swap: apply -1 then update with channel buffers
            // swapped. Implementation here mirrors -1 first then
            // re-feeds in mirror direction.
            if channels < 2 {
                return;
            }
            let pred_a_value = if pass.history[1].is_empty() {
                0
            } else {
                pass.history[1][0]
            };
            let pred_b_value = if pass.history[0].is_empty() {
                0
            } else {
                pass.history[0][0]
            };
            let pred_a = apply_weight(pass.weight[0], pred_a_value);
            let pred_b = apply_weight(pass.weight[1], pred_b_value);
            let new_a = sf[0].wrapping_add(pred_a);
            let new_b = sf[1].wrapping_add(pred_b);
            pass.weight[0] = update_weight(
                pass.weight[0],
                pass.delta as i32,
                sign_i32(sf[0]),
                sign_i32(pred_a_value),
            );
            pass.weight[1] = update_weight(
                pass.weight[1],
                pass.delta as i32,
                sign_i32(sf[1]),
                sign_i32(pred_b_value),
            );
            sf[0] = new_a;
            sf[1] = new_b;
            push_history(&mut pass.history[0], new_a);
            push_history(&mut pass.history[1], new_b);
        }
        _ => unreachable!("unknown term should have been rejected in parse_decterms"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_decterms_handles_known_pairs() {
        // Spec trace: -compression_level 0 ships terms=[(18,2),(17,2)]
        // serialised as 2 bytes (term + 5 in low 5 bits, delta in high 3).
        // 18 + 5 = 23 = 0x17; 17 + 5 = 22 = 0x16.
        // delta=2 in top 3 bits = 0x40.
        // So encoder-application order on disk = [0x57, 0x56]; reversed
        // by parse_decterms gives [(17, 2), (18, 2)].
        let buf = [0x57u8, 0x56];
        let r = parse_decterms(&buf).unwrap();
        assert_eq!(r, vec![(17, 2), (18, 2)]);
    }

    #[test]
    fn parse_decterms_rejects_illegal_term() {
        // term = 9 (in 1..=8 invalid). 9 + 5 = 14, delta=0 → byte=0x0E.
        let buf = [0x0Eu8];
        assert!(parse_decterms(&buf).is_err());
    }

    #[test]
    fn weight_decode_and_clip() {
        // Disk byte 126 → w8 = 1008 + (1008+64)>>7 = 1008 + 8 = 1016.
        // Disk byte -1 → w8 = -8 (no rounding for negative).
        // Single-term case: encoder == decoder order, so weights[0]
        // carries the channel pair.
        let weights = parse_decweights(&[126_i8 as u8, -1_i8 as u8], 1, 2);
        assert_eq!(weights[0][0], 1016);
        assert_eq!(weights[0][1], -8);
    }

    #[test]
    fn weight_two_term_reverses_order() {
        // Two terms × mono. Encoder order on disk = [10, 20], so
        // decoder-order weights[] is reversed.
        let weights = parse_decweights(&[10u8, 20u8], 2, 1);
        // Disk byte 10 → 80 + (80+64)>>7 = 80 + 1 = 81.
        // Disk byte 20 → 160 + (160+64)>>7 = 160 + 1 = 161.
        // After reverse: weights[0] should hold 161 (the encoder-last
        // pass), weights[1] should hold 81.
        assert_eq!(weights[0][0], 161);
        assert_eq!(weights[1][0], 81);
    }

    #[test]
    fn entropy_log_medians_round_trip() {
        // 3 medians × 1 channel × 2 bytes = 6 bytes.
        let buf = [
            0x00u8, 0x00, // 0
            0xCFu8, 0x06, // 1743
            0x0Bu8, 0x06, // 1547
        ];
        let lm = parse_entropy_log_medians(&buf, 1).unwrap();
        assert_eq!(lm, vec![0i16, 1743, 1547]);
    }

    #[test]
    fn int32info_post_shift() {
        // zero=8 case: shift left by 8, no LSB pattern.
        let info = Int32Info {
            sent_bits: 0,
            zero_bits: 8,
            ones_bits: 0,
            dup_bits: 0,
        };
        assert_eq!(apply_int32_postshift(0x1234, &info), 0x1234 << 8);
        let ones = Int32Info {
            sent_bits: 0,
            zero_bits: 0,
            ones_bits: 4,
            dup_bits: 0,
        };
        assert_eq!(apply_int32_postshift(0x10, &ones), (0x10 << 4) | 1);
    }
}
