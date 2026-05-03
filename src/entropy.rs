//! WavPack adaptive 3-bin median entropy decoder (lossless mode).
//!
//! Per spec §5.4 the bitstream packs a sequence of signed residual
//! integers using a Rice/Golomb-like code with a 3-bin median state
//! (M0, M1, M2) per channel. The reader is LSB-first within each byte,
//! same as `EXTRABITS`.
//!
//! Key features in lossless mode:
//!
//! * **Zero-run shortcut**: when both channels' M0 < 2 (and we are not
//!   already inside a run), a unary-prefix length is read; if the
//!   prefix ≥ 2 the next `prefix - 1` bits give a zero-run length and
//!   that many output samples are forced to zero.
//! * **Magnitude-tier code**: a unary prefix `t` with a `t == 16`
//!   escape that reads a second prefix and either small additional
//!   bits or a 2..32-bit binary refinement.
//! * **Adaptive medians**: each successful read updates one or more
//!   of the three medians using rate constants `+5 / -2` scaled by
//!   `(median + 128/(2^n)) / (128/(2^n))` per bin n.
//! * **Tail bits**: `floor(log2(add))` or `+1` bits to disambiguate
//!   within `[base, base+add]`, written in a Golomb-style minimal code.
//! * **Sign bit**: one bit; the value is bit-inverted (`!v`) when the
//!   sign bit is set, so 0 and -1 share a magnitude encoding.

use oxideav_core::bits::BitReaderLsb;
use oxideav_core::{Error, Result};

/// Per-channel adaptive medians + slow-level tracker. The slow level
/// is currently unused in pure-lossless mode (it drives hybrid-mode
/// error-limit balancing), but is kept here so that the field layout
/// matches what hybrid will need in round 2.
#[derive(Debug, Clone, Copy, Default)]
pub struct ChannelMedians {
    pub m: [u32; 3],
    pub slow_level: u32,
}

impl ChannelMedians {
    /// Initialise from the three log-domain values stored in
    /// `WP_ID_ENTROPY` for one channel. `wp_exp2` recovers each
    /// median from the on-disk Q8 log value.
    pub fn from_log_medians(log_med: [i16; 3]) -> Self {
        Self {
            m: [
                crate::log2::wp_exp2(log_med[0]).max(0) as u32,
                crate::log2::wp_exp2(log_med[1]).max(0) as u32,
                crate::log2::wp_exp2(log_med[2]).max(0) as u32,
            ],
            slow_level: 0,
        }
    }
}

/// Median-update step granularity per bin (spec §5.4 INC/DEC table):
/// bin 0 uses 128, bin 1 uses 64, bin 2 uses 32.
const DIV_BY_BIN: [u32; 3] = [128, 64, 32];

#[inline]
fn get_med(med: u32, _bin: usize) -> u32 {
    (med >> 4) + 1
}

#[inline]
fn inc_med(med: &mut u32, bin: usize) {
    let div = DIV_BY_BIN[bin];
    *med = med.wrapping_add(((*med + div) / div) * 5);
}

#[inline]
fn dec_med(med: &mut u32, bin: usize) {
    let div = DIV_BY_BIN[bin];
    let step = ((*med + div - 2) / div) * 2;
    *med = med.saturating_sub(step);
}

/// Stateful entropy decoder for a single block. Holds two channels'
/// medians (mono blocks ignore the second) plus the active zero-run
/// counter and the holding bit for unary-prefix sharing.
pub struct EntropyDecoder {
    /// Per-channel medians; `medians[1]` is unused for mono blocks.
    pub medians: [ChannelMedians; 2],
    /// True if the encoder is allowed to use the zero-run / holding
    /// optimisations (cleared in hybrid mode).
    pub allow_holding: bool,
    /// Active zero-run countdown.
    pub zero_run: u32,
    /// Holding bit for unary-prefix sharing (one of {`Empty`,
    /// `One`, `Zero`}).
    pub holding: HoldingState,
    /// Number of channels in the block (1 or 2).
    pub channels: u32,
    /// True if the zero-run shortcut is available (per spec §5.4
    /// step 1 it triggers whenever both channels' `M0 < 2`). It is
    /// disabled in hybrid mode by the encoder; in lossless mode it
    /// is always allowed.
    pub allow_zero_run: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldingState {
    Empty,
    One,
    Zero,
}

impl EntropyDecoder {
    pub fn new(channels: u32, log_medians: &[i16]) -> Result<Self> {
        // log_medians has length 3 * channels.
        if log_medians.len() != (3 * channels) as usize {
            return Err(Error::invalid(format!(
                "WavPack ENTROPY: expected {} log-medians, got {}",
                3 * channels,
                log_medians.len()
            )));
        }
        let mut medians = [ChannelMedians::default(); 2];
        for c in 0..channels as usize {
            let lm = [
                log_medians[c * 3],
                log_medians[c * 3 + 1],
                log_medians[c * 3 + 2],
            ];
            medians[c] = ChannelMedians::from_log_medians(lm);
        }
        Ok(Self {
            medians,
            // Round-1 keeps the holding-bit shared-terminator
            // optimisation off — its exact semantics in the upstream
            // bitstream were not unambiguously characterised in the
            // trace doc and disabling it costs at most a few bits per
            // sample on average. The zero-run shortcut is independent
            // and stays enabled (spec §5.4 step 1).
            allow_holding: false,
            allow_zero_run: true,
            zero_run: 0,
            holding: HoldingState::Empty,
            channels,
        })
    }

    /// Decode one signed residual sample for channel `c`. Returns
    /// `Ok(None)` if the sample falls inside an active zero-run (the
    /// caller treats this as a literal `0` and skips decorrelation
    /// state updates accordingly — but per spec §5.4 the zero is
    /// still pushed through the predictors).
    pub fn decode_sample(&mut self, br: &mut BitReaderLsb<'_>, c: usize) -> Result<i32> {
        // ------------------------------------------------------------
        // Zero-run shortcut. Only attempt it if we're not already
        // inside one and both channels' M0 are tiny.
        // ------------------------------------------------------------
        if self.zero_run > 0 {
            self.zero_run -= 1;
            return Ok(0);
        }
        // Only check at the start of a sample-frame (channel 0 of
        // the per-frame loop) to match the encoder's emit cadence.
        if self.allow_zero_run
            && c == 0
            && self.holding == HoldingState::Empty
            && self.medians[0].m[0] < 2
            && (self.channels == 1 || self.medians[1].m[0] < 2)
        {
            // Read a unary-prefix length (terminated by 0 bit, max 33).
            let prefix = read_unary(br, 33)?;
            if prefix >= 2 {
                let extra_bits = prefix - 1;
                let extra = br.read_u32(extra_bits)?;
                let run_len = (1u32 << extra_bits) | extra;
                // `run_len` counts sample-frames; the entropy decoder
                // is called once per channel per sample-frame, so the
                // remaining call count is `(run_len - 1) * channels +
                // (channels - 1)` (we already emit ch0's zero now,
                // remaining channels of THIS frame, then the next
                // (run_len - 1) full frames).
                let remaining_in_this_frame = self.channels.saturating_sub(1);
                let next_full_frames = run_len.saturating_sub(1);
                self.zero_run = remaining_in_this_frame
                    .wrapping_add(next_full_frames.wrapping_mul(self.channels));
                // All medians decay to zero after a run.
                self.medians[0].m = [0; 3];
                if self.channels == 2 {
                    self.medians[1].m = [0; 3];
                }
                return Ok(0);
            } else if prefix == 1 {
                // prefix == 1 ⇒ this *single* sample is zero (no run,
                // but a one-shot zero — saves the magnitude / sign
                // bits for an isolated zero in the run-of-near-silence
                // regime).
                return Ok(0);
            }
            // prefix == 0 ⇒ this sample is non-zero. Fall through to
            // the magnitude decode below.
        }

        let med = &mut self.medians[c];

        // ------------------------------------------------------------
        // Magnitude prefix `t` — direct unary read, with the
        // `t == 16` escape per spec §5.4 step 2.
        // ------------------------------------------------------------
        let mut t = read_unary(br, 33)?;
        if t == 16 {
            let t2 = read_unary(br, 33)?;
            if t2 < 2 {
                t += t2;
            } else {
                let bits = t2 - 1;
                let extra = br.read_u32(bits)?;
                t = (1u32 << bits) | extra;
                t += 16;
            }
        }

        let t_shifted = t;

        // ------------------------------------------------------------
        // Bin selection from `t_shifted`.
        // ------------------------------------------------------------
        let (base, add) = if t_shifted == 0 {
            let m0 = get_med(med.m[0], 0);
            dec_med(&mut med.m[0], 0);
            (0u32, m0.saturating_sub(1))
        } else if t_shifted == 1 {
            let m0 = get_med(med.m[0], 0);
            let m1 = get_med(med.m[1], 1);
            inc_med(&mut med.m[0], 0);
            dec_med(&mut med.m[1], 1);
            (m0, m1.saturating_sub(1))
        } else if t_shifted == 2 {
            let m0 = get_med(med.m[0], 0);
            let m1 = get_med(med.m[1], 1);
            let m2 = get_med(med.m[2], 2);
            inc_med(&mut med.m[0], 0);
            inc_med(&mut med.m[1], 1);
            dec_med(&mut med.m[2], 2);
            (m0 + m1, m2.saturating_sub(1))
        } else {
            // t_shifted >= 3
            let m0 = get_med(med.m[0], 0);
            let m1 = get_med(med.m[1], 1);
            let m2 = get_med(med.m[2], 2);
            inc_med(&mut med.m[0], 0);
            inc_med(&mut med.m[1], 1);
            inc_med(&mut med.m[2], 2);
            let base = m0 + m1 + (t_shifted - 2).saturating_mul(m2);
            (base, m2.saturating_sub(1))
        };

        // ------------------------------------------------------------
        // Tail bits: get_tail() emits the value within [base, base+add]
        // using a Golomb-style minimal code.
        // ------------------------------------------------------------
        let tail = if add == 0 {
            0
        } else {
            // log2(add+1) bits => p, then either p-1 bits or p bits
            // depending on the value range. This matches the FFmpeg
            // get_tail() pattern referenced in spec §5.4 step 4.
            let p = 31 - add.leading_zeros();
            let mut e = ((1u32 << (p + 1)).wrapping_sub(add)).wrapping_sub(1);
            let mut res = if p == 0 { 0 } else { br.read_u32(p)? };
            if res >= e {
                let extra = br.read_u32(1)?;
                res = (res << 1) - e + extra;
            } else {
                let _ = &mut e; // silence unused-mut lint when p==0
            }
            res
        };

        let magnitude = base + tail;

        // ------------------------------------------------------------
        // Sign bit and bit-invert mapping. Per spec §5.4 step 5,
        // an output of `magnitude=0` and a sign bit of 1 produces
        // the value `-1` (because `!(0 as i32) == -1`); this lets
        // 0 and -1 share a magnitude encoding. Since magnitude=0
        // would otherwise collide with both signs, the spec emits
        // only one (the encoder picks the negative half).
        // ------------------------------------------------------------
        let sign = br.read_u32(1)?;
        let value = if sign != 0 {
            !(magnitude as i32)
        } else {
            magnitude as i32
        };

        Ok(value)
    }
}

/// Unary-prefix reader: count consecutive 1 bits, terminated by a 0.
/// `max` caps the prefix length; an over-long prefix is rejected.
pub fn read_unary(br: &mut BitReaderLsb<'_>, max: u32) -> Result<u32> {
    let mut n = 0u32;
    loop {
        let b = br.read_u32(1)?;
        if b == 0 {
            return Ok(n);
        }
        n += 1;
        if n > max {
            return Err(Error::invalid("WavPack: unary prefix exceeds max"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn medians_initialise_from_zero_logs() {
        let m = ChannelMedians::from_log_medians([0, 0, 0]);
        assert_eq!(m.m, [0, 0, 0]);
    }

    #[test]
    fn read_unary_basic() {
        // 0b00010111 in LSB-first reads as bit 1, bit 1, bit 1, bit 0
        // = unary prefix of 3.
        let buf = [0b0001_0111_u8];
        let mut br = BitReaderLsb::new(&buf);
        let n = read_unary(&mut br, 33).unwrap();
        assert_eq!(n, 3);
    }

    #[test]
    fn inc_dec_med_monotone() {
        let mut m: u32 = 100;
        let initial = m;
        inc_med(&mut m, 0);
        assert!(m > initial);
        let after_inc = m;
        dec_med(&mut m, 0);
        assert!(m < after_inc);
    }
}
