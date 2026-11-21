//! DSD (1-bit) blocks: the `0x0E` sub-block and its three payload forms.
//!
//! Staged spec `docs/audio/wavpack/spec/wavpack-dsd.md`. A block whose
//! header flag bit 31 is set carries DSD audio: its container width is
//! one byte, each "sample" is a byte holding eight consecutive 1-bit
//! samples of one channel (most-significant bit first, §2.1), and the
//! whole coded audio lives in one `0x0E` sub-block — there is no `0x0A`
//! bitstream, no decorrelation and no hybrid mode (§1.3, §7).
//!
//! Every `0x0E` payload starts with a rate-shift byte and a mode byte
//! (§3). The rest is one of:
//!
//! * **mode 0, stored** (§4) — the decoded bytes verbatim;
//! * **mode 1, "fast"** (§5) — a byte-alphabet range code over a static
//!   per-context frequency table transmitted in the payload, the
//!   context being the low bits of the previous byte of the same
//!   channel;
//! * **mode 3, "high"** (§6) — a bit-level adaptive binary range code
//!   whose probability bin is picked by an integer prediction from a
//!   per-channel cascade of leaky integrators.
//!
//! The decoded bytes come back as `i32` values `0..=255` in interleaved
//! order — the same shape the PCM paths return — and the block CRC is
//! the mono recurrence `crc * 3 + byte` over that order, stereo
//! included (§2.4).

use crate::error::{Error, Result};

/// Largest valid rate-shift byte (§3.1).
pub const DSD_MAX_RATE_SHIFT: u8 = 31;
/// Mode byte of the stored form (§4).
pub const DSD_MODE_STORED: u8 = 0;
/// Mode byte of the "fast" form (§5).
pub const DSD_MODE_FAST: u8 = 1;
/// Mode byte of the "high" form (§6).
pub const DSD_MODE_HIGH: u8 = 3;
/// The byte a muted DSD block is filled with: the alternating bit
/// pattern `01010101`, which is DSD silence (§1.4) — a zero byte would
/// be full-scale negative.
pub const DSD_SILENCE: u8 = 0x55;

/// Largest history-bit count of a mode-1 model (§5.1): 32 contexts.
const FAST_MAX_HISTORY_BITS: u8 = 5;
/// Per-context ceiling on the summed frequencies of a mode-1 model
/// (§5.3 storage bound).
const FAST_CONTEXT_BOUND: u32 = 1280;
/// The only valid mode-3 table-slope byte (§6.1).
const HIGH_SLOPE: i32 = 20;
/// Mode-3 fixed-point unit (§6.3).
const HIGH_ONE: i32 = 1 << 20;

/// The parsed two-byte prefix of a `0x0E` payload (§3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DsdHeader {
    /// `multiplier = 1 << rate_shift` turns the header's reduced
    /// sample rate into the stream's **byte** rate (§2.3).
    pub rate_shift: u8,
    /// Payload form: [`DSD_MODE_STORED`], [`DSD_MODE_FAST`] or
    /// [`DSD_MODE_HIGH`].
    pub mode: u8,
}

/// Parse the common prefix of a `0x0E` payload (§3), validating the
/// rate shift and the mode enumeration.
pub fn parse_dsd_header(payload: &[u8]) -> Result<DsdHeader> {
    let [rate_shift, mode, ..] = *payload else {
        return Err(Error::InvalidDsdBlock(
            "payload shorter than its two-byte prefix",
        ));
    };
    if rate_shift > DSD_MAX_RATE_SHIFT {
        return Err(Error::InvalidDsdBlock("rate shift above 31"));
    }
    if !matches!(mode, DSD_MODE_STORED | DSD_MODE_FAST | DSD_MODE_HIGH) {
        return Err(Error::InvalidDsdBlock("undefined mode byte"));
    }
    Ok(DsdHeader { rate_shift, mode })
}

/// Decode a whole `0x0E` payload into `frames × channels` byte values
/// (as `i32`, `0..=255`) in interleaved order. `channels` is the number
/// of **coded** channels: 1 for a mono or false-stereo block, 2 for
/// stereo.
pub fn decode_dsd_payload(payload: &[u8], frames: usize, channels: usize) -> Result<Vec<i32>> {
    let header = parse_dsd_header(payload)?;
    let body = &payload[2..];
    let count = frames
        .checked_mul(channels)
        .ok_or(Error::InvalidDsdBlock("sample count overflows"))?;
    match header.mode {
        DSD_MODE_STORED => decode_stored(body, count),
        DSD_MODE_FAST => decode_fast(body, count, channels),
        _ => decode_high(body, frames, channels),
    }
}

/// Mode 0 (§4): the payload is the byte sequence, and its length must
/// equal the sample count exactly.
fn decode_stored(body: &[u8], count: usize) -> Result<Vec<i32>> {
    if body.len() != count {
        return Err(Error::InvalidDsdBlock(
            "stored payload length is not the sample count",
        ));
    }
    Ok(body.iter().map(|&b| i32::from(b)).collect())
}

/// The 32-bit range-decoder registers shared by modes 1 and 3 (§5.4).
struct RangeDecoder<'a> {
    low: u32,
    high: u32,
    value: u32,
    input: &'a [u8],
}

impl<'a> RangeDecoder<'a> {
    /// Load the 4-byte big-endian seed and open the full interval.
    fn new(input: &'a [u8]) -> Result<Self> {
        let Some((seed, rest)) = input.split_first_chunk::<4>() else {
            return Err(Error::InvalidDsdBlock("range seed missing"));
        };
        Ok(Self {
            low: 0,
            high: u32::MAX,
            value: u32::from_be_bytes(*seed),
            input: rest,
        })
    }

    /// Shift whole bytes in while the top bytes of `low` and `high`
    /// agree. Running out of payload invalidates the block.
    #[inline]
    fn renormalise(&mut self) -> Result<()> {
        while (self.high ^ self.low) & 0xFF00_0000 == 0 {
            let Some((&byte, rest)) = self.input.split_first() else {
                return Err(Error::InvalidDsdBlock("range-coded data exhausted"));
            };
            self.input = rest;
            self.value = (self.value << 8) | u32::from(byte);
            self.high = (self.high << 8) | 0xFF;
            self.low <<= 8;
        }
        Ok(())
    }
}

/// One context of a mode-1 model (§5.3).
struct FastContext {
    /// Unnormalised frequency of each byte value.
    freq: [u8; 256],
    /// `cumulative[s]` = sum of `freq[..=s]`.
    cumulative: [u16; 256],
    /// Byte value `s` repeated `freq[s]` times, ascending: the symbol
    /// owning each cumulative slot.
    symbols: Vec<u8>,
}

/// Read the mode-1 frequency table (§5.2) and return it with the
/// payload bytes that follow.
fn read_fast_table(body: &[u8], bins: usize, max_literal: u8) -> Result<(Vec<u8>, &[u8])> {
    let entries = 256 * bins;
    if max_literal == 255 {
        // Verbatim form: the table, with at least one byte after it.
        if body.len() <= entries {
            return Err(Error::InvalidDsdBlock("verbatim model table truncated"));
        }
        return Ok((body[..entries].to_vec(), &body[entries..]));
    }
    let mut table = Vec::with_capacity(entries);
    let mut rest = body;
    while table.len() < entries {
        let Some((&code, tail)) = rest.split_first() else {
            return Err(Error::InvalidDsdBlock("model table truncated"));
        };
        rest = tail;
        if code == 0 {
            return Err(Error::InvalidDsdBlock("model table ends early"));
        }
        if code <= max_literal {
            table.push(code);
        } else {
            let run = usize::from(code - max_literal);
            if table.len() + run > entries {
                return Err(Error::InvalidDsdBlock("model table zero run overruns"));
            }
            table.resize(table.len() + run, 0);
        }
    }
    // Exactly one terminator follows a full table and is consumed.
    match rest.split_first() {
        Some((0, tail)) => Ok((table, tail)),
        _ => Err(Error::InvalidDsdBlock("model table terminator missing")),
    }
}

/// Mode 1 (§5): byte-alphabet range decode over a transmitted static
/// model, one context per value of the previous same-channel byte's
/// low `history_bits` bits.
fn decode_fast(body: &[u8], count: usize, channels: usize) -> Result<Vec<i32>> {
    let [history_bits, max_literal, ref after @ ..] = *body else {
        return Err(Error::InvalidDsdBlock("fast-mode header truncated"));
    };
    if history_bits > FAST_MAX_HISTORY_BITS {
        return Err(Error::InvalidDsdBlock("more than five history bits"));
    }
    let bins = 1usize << history_bits;
    let (table, coded) = read_fast_table(after, bins, max_literal)?;

    let mut contexts = Vec::with_capacity(bins);
    let mut total_slots = 0u32;
    for row in table.as_chunks::<256>().0 {
        let mut cumulative = [0u16; 256];
        let mut sum = 0u32;
        for (slot, &f) in cumulative.iter_mut().zip(row.iter()) {
            sum += u32::from(f);
            // 256 × 255 fits 16 bits; the storage bound below is far
            // tighter than that for any accepted model.
            *slot = sum as u16;
        }
        total_slots += sum;
        if total_slots > bins as u32 * FAST_CONTEXT_BOUND {
            return Err(Error::InvalidDsdBlock("model exceeds its storage bound"));
        }
        let mut symbols = Vec::with_capacity(sum as usize);
        for (value, &f) in row.iter().enumerate() {
            symbols.resize(symbols.len() + usize::from(f), value as u8);
        }
        contexts.push(FastContext {
            freq: *row,
            cumulative,
            symbols,
        });
    }

    let mut rc = RangeDecoder::new(coded)?;
    let mask = bins - 1;
    // The context of a byte is the low bits of the previous byte of
    // the same channel; both start at 0 (§5.6).
    let mut context = [0usize; 2];
    // Every decoded byte costs at least nothing from the payload (a
    // near-certain symbol), so the output is bounded by `count` alone;
    // the caller has already capped that.
    let mut out = Vec::with_capacity(count.min(coded.len().saturating_mul(64).max(1024)));
    for slot in 0..count {
        let ch = if channels == 2 { slot & 1 } else { 0 };
        let model = &contexts[context[ch]];
        let sum = u32::from(model.cumulative[255]);
        if sum == 0 {
            return Err(Error::InvalidDsdBlock("empty context selected"));
        }
        let mut mult = rc.high.wrapping_sub(rc.low) / sum;
        if mult == 0 {
            // Interval collapse: restart the coder from four fresh bytes.
            let Some((fresh, rest)) = rc.input.split_first_chunk::<4>() else {
                return Err(Error::InvalidDsdBlock("range-coded data exhausted"));
            };
            rc.input = rest;
            rc.value = u32::from_be_bytes(*fresh);
            rc.low = 0;
            rc.high = u32::MAX;
            mult = rc.high / sum;
            if mult == 0 {
                return Err(Error::InvalidDsdBlock("range interval collapsed"));
            }
        }
        let index = rc.value.wrapping_sub(rc.low) / mult;
        if index >= sum {
            return Err(Error::InvalidDsdBlock("range code outside the model"));
        }
        let symbol = model.symbols[index as usize];
        if symbol != 0 {
            rc.low = rc
                .low
                .wrapping_add(u32::from(model.cumulative[usize::from(symbol) - 1]) * mult);
        }
        rc.high = rc
            .low
            .wrapping_add(u32::from(model.freq[usize::from(symbol)]) * mult)
            .wrapping_sub(1);
        out.push(i32::from(symbol));
        context[ch] = usize::from(symbol) & mask;
        rc.renormalise()?;
    }
    Ok(out)
}

/// Build the mode-3 probability table from its rate byte (§6.2).
fn high_probability_table(rate_byte: u8) -> [i32; 256] {
    let decay = |v: i32| v + ((0x0001_0000 - v) >> 8);
    let mut table = [0i32; 256];
    let mut v = 0x0080_8000i32;
    let mut rate = i32::from(rate_byte) << 8;
    for _ in 0..(rate + 128) >> 8 {
        v = decay(v);
    }
    for i in 0..128 {
        table[i] = v;
        table[255 - i] = 0x0100_FFFF - v;
        if v > 0x0001_0000 {
            rate += (rate * HIGH_SLOPE + 128) >> 8;
            for _ in 0..(rate + 64) >> 7 {
                v = decay(v);
            }
        }
    }
    table
}

/// Per-channel mode-3 predictor state (§6.3, §6.6).
#[derive(Clone, Copy, Default)]
struct HighChannel {
    /// The cascade accumulators `A1..A6`.
    a: [i32; 6],
    /// The adaptive trim term `T`.
    trim: i32,
    /// The current prediction `P`.
    prediction: i32,
    /// The byte being assembled, most-significant bit first.
    byte: i32,
}

impl HighChannel {
    fn from_seed(seed: &[u8; 7]) -> Self {
        let mut a = [0i32; 6];
        for (acc, &byte) in a.iter_mut().zip(&seed[..5]) {
            *acc = i32::from(byte) << 12;
        }
        Self {
            a,
            trim: i32::from(i16::from_le_bytes([seed[5], seed[6]])),
            prediction: 0,
            byte: 0,
        }
    }

    /// `P = A1 − A5 + ((A6 × T) >> 2)`.
    #[inline]
    fn predict(&mut self) {
        self.prediction = self.a[0]
            .wrapping_sub(self.a[4])
            .wrapping_add(self.a[5].wrapping_mul(self.trim) >> 2);
    }
}

/// Mode 3 (§6): bit-level adaptive binary range decode. Stereo blocks
/// interleave the two channels bit by bit inside the one coder.
fn decode_high(body: &[u8], frames: usize, channels: usize) -> Result<Vec<i32>> {
    if body.len() < 2 + 7 * channels + 4 {
        return Err(Error::InvalidDsdBlock("high-mode header truncated"));
    }
    if i32::from(body[1]) != HIGH_SLOPE {
        return Err(Error::InvalidDsdBlock("table slope is not 20"));
    }
    let mut table = high_probability_table(body[0]);
    let (seeds, coded) = body[2..].split_at(7 * channels);
    let mut state = [HighChannel::default(); 2];
    for (ch, seed) in state.iter_mut().zip(seeds.as_chunks::<7>().0) {
        *ch = HighChannel::from_seed(seed);
    }
    let state = &mut state[..channels];
    let mut rc = RangeDecoder::new(coded)?;

    let mut out = Vec::with_capacity(
        frames
            .saturating_mul(channels)
            .min(coded.len().saturating_mul(64).max(1024)),
    );
    for _ in 0..frames {
        for ch in state.iter_mut() {
            ch.predict();
        }
        for _ in 0..8 {
            for ch in state.iter_mut() {
                // Bin select and binary split.
                let bin = ((ch.prediction >> 8) & 0xFF) as usize;
                let p = (table[bin] >> 16) as u32;
                let split = rc
                    .low
                    .wrapping_add((rc.high.wrapping_sub(rc.low) >> 8).wrapping_mul(p));
                // `mask` is all-ones for a 1 bit, zero for a 0 bit.
                let mask = if rc.value <= split {
                    rc.high = split;
                    table[bin] += (0x0100_00FE - table[bin]) >> 8;
                    -1i32
                } else {
                    rc.low = split.wrapping_add(1);
                    table[bin] += (0x0001_0000 - table[bin]) >> 8;
                    0i32
                };
                rc.renormalise()?;
                // Prediction nudge, byte assembly, trim update.
                let a6 = ch.a[5];
                let p_now = ch.prediction.wrapping_add(a6.wrapping_mul(8));
                ch.byte = (ch.byte << 1) | (mask & 1);
                let step = ((p_now ^ mask) >> 31) | 1;
                let apply = (p_now ^ p_now.wrapping_sub(a6.wrapping_mul(16))) >> 31;
                ch.trim = ch.trim.wrapping_add(step & apply);
                // Cascade update.
                let bit = mask & HIGH_ONE;
                ch.a[0] = ch.a[0].wrapping_add(bit.wrapping_sub(ch.a[0]) >> 6);
                ch.a[1] = ch.a[1].wrapping_add(bit.wrapping_sub(ch.a[1]) >> 4);
                ch.a[2] = ch.a[2].wrapping_add(ch.a[1].wrapping_sub(ch.a[2]) >> 4);
                ch.a[3] = ch.a[3].wrapping_add(ch.a[2].wrapping_sub(ch.a[3]) >> 4);
                let d = ch.a[3].wrapping_sub(ch.a[4]) >> 4;
                ch.a[4] = ch.a[4].wrapping_add(d);
                ch.a[5] = ch.a[5].wrapping_add(d.wrapping_sub(ch.a[5]) >> 3);
                ch.predict();
            }
        }
        for ch in state.iter_mut() {
            out.push(ch.byte & 0xFF);
            ch.trim = ch.trim.wrapping_sub(ch.trim.wrapping_add(512) >> 10);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prefix_validation() {
        assert!(parse_dsd_header(&[]).is_err());
        assert!(parse_dsd_header(&[0]).is_err());
        assert_eq!(
            parse_dsd_header(&[2, 1]).unwrap(),
            DsdHeader {
                rate_shift: 2,
                mode: DSD_MODE_FAST
            }
        );
        assert!(parse_dsd_header(&[32, 0]).is_err());
        for mode in [2u8, 4, 5, 255] {
            assert!(parse_dsd_header(&[0, mode]).is_err(), "mode {mode}");
        }
    }

    #[test]
    fn stored_mode_is_a_copy_with_an_exact_length() {
        let payload = [0u8, 0, 0x55, 0xAA, 0x0F, 0xF0];
        assert_eq!(
            decode_dsd_payload(&payload, 2, 2).unwrap(),
            vec![0x55, 0xAA, 0x0F, 0xF0]
        );
        assert_eq!(
            decode_dsd_payload(&payload, 4, 1).unwrap(),
            vec![0x55, 0xAA, 0x0F, 0xF0]
        );
        assert!(decode_dsd_payload(&payload, 3, 1).is_err());
        assert!(decode_dsd_payload(&payload, 5, 1).is_err());
    }

    /// §6.2's stated invariants of the table initialiser, for every
    /// rate byte: exact antisymmetry about the half-probability point,
    /// coded probability confined to 1..=255 and monotone in signed
    /// index order, and a flat table at rate 0.
    #[test]
    fn high_table_invariants_hold_for_every_rate() {
        for rate in 0..=255u8 {
            let t = high_probability_table(rate);
            let mut previous = 256;
            for signed in -128i32..=127 {
                let i = (signed & 0xFF) as usize;
                assert_eq!(t[i] + t[255 - i], 0x0100_FFFF, "rate {rate} index {i}");
                let p = t[i] >> 16;
                assert!((1..=255).contains(&p), "rate {rate} index {i} p {p}");
                assert!(p <= previous, "rate {rate}: not monotone at {signed}");
                previous = p;
            }
            if rate == 0 {
                assert!(t.iter().all(|&v| v >> 16 == 128));
            }
        }
    }

    #[test]
    fn fast_mode_table_forms_and_refusals() {
        // More than five history bits.
        assert!(decode_dsd_payload(&[0, 1, 6, 0xA0], 1, 1).is_err());
        // Zero-run table (one context): 0x55 with frequency 1, every
        // other value 0 — runs of 85 zeros, a literal, 170 zeros.
        let mut payload = vec![0u8, 1, 0, 0xA0];
        payload.push(0xA0 + 85);
        payload.push(1);
        payload.extend_from_slice(&[0xA0 + 95, 0xA0 + 75]);
        let table_only = payload.clone();
        payload.push(0); // terminator
        payload.extend_from_slice(&[0, 0, 0, 0]); // range seed
        payload.extend_from_slice(&[0; 8]); // renormalisation bytes
        // A single-symbol model decodes that symbol whatever the code.
        assert_eq!(decode_dsd_payload(&payload, 3, 1).unwrap(), vec![0x55; 3]);
        // Missing terminator / missing seed.
        assert!(decode_dsd_payload(&table_only, 1, 1).is_err());
        let mut no_seed = table_only.clone();
        no_seed.push(0);
        assert!(decode_dsd_payload(&no_seed, 1, 1).is_err());
        // A zero run that overruns the table.
        let mut overrun = vec![0u8, 1, 0, 0xA0];
        overrun.extend_from_slice(&[0xFF, 0xFF, 0xFF]);
        assert!(decode_dsd_payload(&overrun, 1, 1).is_err());
        // The verbatim form needs a byte after the table.
        let mut verbatim = vec![0u8, 1, 0, 0xFF];
        verbatim.extend_from_slice(&[1; 256]);
        assert!(decode_dsd_payload(&verbatim, 1, 1).is_err());
        verbatim.extend_from_slice(&[0x80, 0, 0, 0, 0, 0, 0, 0]);
        // Uniform model: the code 0x80000000 lands on symbol 0x80.
        assert_eq!(decode_dsd_payload(&verbatim, 1, 1).unwrap(), vec![0x80]);
    }

    #[test]
    fn high_mode_header_refusals() {
        // Too short for its seeds, then a wrong slope byte.
        assert!(decode_dsd_payload(&[0, 3, 6, 20, 0, 0], 1, 1).is_err());
        let mut payload = vec![0u8, 3, 6, 19];
        payload.extend_from_slice(&[0; 7 + 4 + 8]);
        assert!(decode_dsd_payload(&payload, 1, 1).is_err());
        payload[3] = 20;
        assert_eq!(decode_dsd_payload(&payload, 1, 1).unwrap().len(), 1);
    }
}
