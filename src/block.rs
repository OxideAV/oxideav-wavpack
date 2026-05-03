//! WavPack 32-byte block header + tagged sub-block walker.
//!
//! Per spec §3 a WavPack file is a bare concatenation of self-delimiting
//! blocks. There is no per-file magic. Each block begins with the ASCII
//! magic `wvpk`, followed by 28 more bytes of fixed-layout little-endian
//! header. Everything between byte 32 of one block and the start of the
//! next is a sequence of tagged sub-blocks (1-byte id, then a length).

use oxideav_core::{Error, Result};

/// Block magic: ASCII `wvpk` (`77 76 70 6B`).
pub const BLOCK_MAGIC: [u8; 4] = *b"wvpk";

/// Format-version range supported by this decoder. Per spec §1 the
/// reference encoder targets `0x402..=0x410`; an out-of-range version
/// is rejected before sub-block parsing.
pub const MIN_VERSION: u16 = 0x0402;
pub const MAX_VERSION: u16 = 0x0410;

/// Maximum permitted block size in bytes (per spec §8.4 `WV_BLOCK_LIMIT`).
pub const MAX_BLOCK_SIZE: u32 = 1 << 20;

/// Maximum permitted samples-per-channel in a single block (per spec
/// §8.4 — keeps per-frame buffers manageable).
pub const MAX_BLOCK_SAMPLES: u32 = 150_000;

// ---------------------------------------------------------------------
// Flag-bit positions (spec §3.4.1)
// ---------------------------------------------------------------------

pub const WV_BPS_MASK: u32 = 0x0000_0003;
pub const WV_MONO: u32 = 0x0000_0004;
pub const WV_HYBRID: u32 = 0x0000_0008;
pub const WV_JOINT_STEREO: u32 = 0x0000_0010;
pub const WV_CROSS_DECORR: u32 = 0x0000_0020;
pub const WV_HYBRID_SHAPE: u32 = 0x0000_0040;
pub const WV_FLOAT_DATA: u32 = 0x0000_0080;
pub const WV_INT32_DATA: u32 = 0x0000_0100;
pub const WV_HYBRID_BITRATE: u32 = 0x0000_0200;
pub const WV_HYBRID_BALANCE: u32 = 0x0000_0400;
pub const WV_INITIAL_BLOCK: u32 = 0x0000_0800;
pub const WV_FINAL_BLOCK: u32 = 0x0000_1000;
pub const WV_SHIFT_MASK: u32 = 0x0003_E000;
pub const WV_SHIFT_LSB: u32 = 13;
pub const WV_MAG_MASK: u32 = 0x007C_0000;
pub const WV_MAG_LSB: u32 = 18;
pub const WV_SR_IDX_MASK: u32 = 0x0780_0000;
pub const WV_SR_IDX_LSB: u32 = 23;
pub const WV_FALSE_STEREO: u32 = 0x4000_0000;
pub const WV_DSD_DATA: u32 = 0x8000_0000;

/// Sample-rate index table (spec §3.4.2). `sr_idx == 15` means the
/// rate is custom and arrives in a `SAMPLE_RATE` sub-block.
pub const WV_RATES: [u32; 15] = [
    6_000, 8_000, 9_600, 11_025, 12_000, 16_000, 22_050, 24_000, 32_000, 44_100, 48_000, 64_000,
    88_200, 96_000, 192_000,
];

/// Decode the 4-bit sample-rate index field; returns `None` for the
/// custom (`15`) sentinel.
pub fn rate_for_index(idx: u32) -> Option<u32> {
    if (idx as usize) < WV_RATES.len() {
        Some(WV_RATES[idx as usize])
    } else {
        None
    }
}

/// Container bit-depth from the 2-bit `BPS` field: 8, 16, 24, 32.
pub fn container_bps(flags: u32) -> u32 {
    ((flags & WV_BPS_MASK) + 1) * 8
}

/// `original_bits_per_sample` from the 5-bit `MAG` field.
pub fn magnitude_bits(flags: u32) -> u32 {
    ((flags & WV_MAG_MASK) >> WV_MAG_LSB) + 1
}

/// Global pre-shift count from the 5-bit `SHIFT` field.
pub fn shift_count(flags: u32) -> u32 {
    (flags & WV_SHIFT_MASK) >> WV_SHIFT_LSB
}

/// Sample-rate index (raw 4-bit value; 15 = custom).
pub fn sr_index(flags: u32) -> u32 {
    (flags & WV_SR_IDX_MASK) >> WV_SR_IDX_LSB
}

/// 32-byte WavPack block header. All fields are little-endian on disk.
#[derive(Debug, Clone, Copy)]
pub struct BlockHeader {
    /// Block size − 8 (i.e. payload + last 24 bytes of header).
    pub block_size: u32,
    /// Bitstream format version (`0x402..=0x410`).
    pub version: u16,
    pub track_number: u8,
    pub index_number: u8,
    /// Total samples in the *first* block of the stream (`0` in
    /// later blocks; `0xFFFFFFFF` = unknown).
    pub total_samples: u32,
    /// Sample index of the first sample in this block.
    pub block_index: u32,
    /// Number of samples per channel in this block.
    pub block_samples: u32,
    /// 32-bit flags field (see §3.4 / `WV_*` constants).
    pub flags: u32,
    /// Running CRC of the *decoded* sample stream (see spec §5.1).
    pub crc: u32,
}

impl BlockHeader {
    /// Parse a 32-byte WavPack block header. Validates magic, version
    /// range, and the block-size envelope.
    pub fn parse(buf: &[u8]) -> Result<Self> {
        if buf.len() < 32 {
            return Err(Error::invalid("WavPack: block header truncated"));
        }
        if buf[..4] != BLOCK_MAGIC {
            return Err(Error::invalid("WavPack: missing 'wvpk' magic"));
        }
        let block_size = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        if block_size < 24 || block_size > MAX_BLOCK_SIZE {
            return Err(Error::invalid(format!(
                "WavPack: block_size {block_size} outside [24, {MAX_BLOCK_SIZE}]"
            )));
        }
        let version = u16::from_le_bytes([buf[8], buf[9]]);
        if !(MIN_VERSION..=MAX_VERSION).contains(&version) {
            return Err(Error::invalid(format!(
                "WavPack: unsupported format version {version:#06x}"
            )));
        }
        let track_number = buf[10];
        let index_number = buf[11];
        let total_samples = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
        let block_index = u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]);
        let block_samples = u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]);
        if block_samples > MAX_BLOCK_SAMPLES {
            return Err(Error::invalid(format!(
                "WavPack: block_samples {block_samples} above {MAX_BLOCK_SAMPLES}"
            )));
        }
        let flags = u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]);
        let crc = u32::from_le_bytes([buf[28], buf[29], buf[30], buf[31]]);
        Ok(Self {
            block_size,
            version,
            track_number,
            index_number,
            total_samples,
            block_index,
            block_samples,
            flags,
            crc,
        })
    }

    /// True if the block is the first of a multi-block frame.
    pub fn is_initial(&self) -> bool {
        (self.flags & WV_INITIAL_BLOCK) != 0
    }

    /// True if the block is the last of a multi-block frame.
    pub fn is_final(&self) -> bool {
        (self.flags & WV_FINAL_BLOCK) != 0
    }

    /// True if the block carries a single mono channel
    /// (either truly mono or the false-stereo collapsed case).
    pub fn is_mono_data(&self) -> bool {
        (self.flags & (WV_MONO | WV_FALSE_STEREO)) != 0
    }

    /// True if the block uses joint-stereo (mid/side) coding.
    pub fn is_joint_stereo(&self) -> bool {
        (self.flags & WV_JOINT_STEREO) != 0
    }

    /// True if the block uses cross-channel decorrelation passes
    /// (terms -1, -2, -3).
    pub fn is_cross_decorr(&self) -> bool {
        (self.flags & WV_CROSS_DECORR) != 0
    }

    /// True if the encoded samples are IEEE-754 floats.
    pub fn is_float(&self) -> bool {
        (self.flags & WV_FLOAT_DATA) != 0
    }

    /// True if the encoded samples are 32-bit integer with an
    /// `INT32INFO` descriptor.
    pub fn is_int32(&self) -> bool {
        (self.flags & WV_INT32_DATA) != 0
    }

    /// True if the block uses hybrid (lossy) coding.
    pub fn is_hybrid(&self) -> bool {
        (self.flags & WV_HYBRID) != 0
    }

    /// True if the block carries DSD instead of PCM.
    pub fn is_dsd(&self) -> bool {
        (self.flags & WV_DSD_DATA) != 0
    }

    /// True if the input was stereo but L≡R for the whole block
    /// (encoder dropped to one channel).
    pub fn is_false_stereo(&self) -> bool {
        (self.flags & WV_FALSE_STEREO) != 0
    }

    /// Number of channels carried by the on-disk block (1 or 2).
    pub fn channels_in_block(&self) -> u32 {
        if self.is_mono_data() {
            1
        } else {
            2
        }
    }

    /// Container bit-depth: 8, 16, 24, 32.
    pub fn container_bps(&self) -> u32 {
        container_bps(self.flags)
    }

    /// `original_bits_per_sample` (post-shift effective magnitude).
    pub fn magnitude_bits(&self) -> u32 {
        magnitude_bits(self.flags)
    }

    /// Global pre-shift count (post-decode left-shift, 0..=31).
    pub fn shift_count(&self) -> u32 {
        shift_count(self.flags)
    }

    /// Sample-rate index (4 bits; 15 = custom).
    pub fn sr_index(&self) -> u32 {
        sr_index(self.flags)
    }

    /// Length in bytes of the block payload (everything after the
    /// 32-byte header). The on-disk `block_size` field counts the
    /// payload plus the last 24 bytes of header, so payload length =
    /// `block_size - 24`.
    pub fn payload_len(&self) -> usize {
        (self.block_size - 24) as usize
    }

    /// Length on disk including the 8-byte (magic + size) preamble.
    /// The next block begins `header_offset + on_disk_len()` bytes
    /// into the file.
    pub fn on_disk_len(&self) -> usize {
        8 + self.block_size as usize
    }
}

// ---------------------------------------------------------------------
// Sub-block id namespace + flag bits (spec §3.5)
// ---------------------------------------------------------------------

pub const WP_ID_TYPE_MASK: u8 = 0x1F;
pub const WP_IDF_IGNORE: u8 = 0x20;
pub const WP_IDF_ODD: u8 = 0x40;
pub const WP_IDF_LARGE: u8 = 0x80;

pub const WP_ID_DUMMY: u8 = 0x00;
pub const WP_ID_ENCINFO: u8 = 0x01;
pub const WP_ID_DECTERMS: u8 = 0x02;
pub const WP_ID_DECWEIGHTS: u8 = 0x03;
pub const WP_ID_DECSAMPLES: u8 = 0x04;
pub const WP_ID_ENTROPY: u8 = 0x05;
pub const WP_ID_HYBRID: u8 = 0x06;
pub const WP_ID_SHAPING: u8 = 0x07;
pub const WP_ID_FLOATINFO: u8 = 0x08;
pub const WP_ID_INT32INFO: u8 = 0x09;
pub const WP_ID_DATA: u8 = 0x0A;
pub const WP_ID_WVC_BITSTREAM: u8 = 0x0B;
pub const WP_ID_EXTRABITS: u8 = 0x0C;
pub const WP_ID_CHANINFO: u8 = 0x0D;
pub const WP_ID_DSD_DATA: u8 = 0x0E;
/// SAMPLE_RATE: the 5-bit type slot is `0x07` (same as SHAPING) but
/// the on-disk id always carries the IGNORE flag so older decoders
/// skip it. We expose the full-id form `0x27` for direct comparison
/// against the byte stripped of the LARGE/ODD bits via
/// [`SubBlock::id_no_size_flags`].
pub const WP_ID_SAMPLE_RATE: u8 = 0x27;

/// One parsed sub-block — its type id plus a slice into the block
/// payload. The slice excludes any trailing pad byte that may have
/// been added because of `WP_IDF_ODD`.
#[derive(Debug)]
pub struct SubBlock<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

impl<'a> SubBlock<'a> {
    /// Sub-block type (low 5 bits of `id`).
    pub fn ty(&self) -> u8 {
        self.id & WP_ID_TYPE_MASK
    }

    /// `IGNORE` flag — unknown type is non-fatal when set.
    pub fn ignore_flag(&self) -> bool {
        (self.id & WP_IDF_IGNORE) != 0
    }

    /// On-disk id with the size-encoding flags (LARGE / ODD) stripped
    /// out, but `IGNORE` left in place. Use this for matches against
    /// the `WP_ID_SAMPLE_RATE`-style "full id" constants where the
    /// upstream tag distinguishes itself by always carrying IGNORE.
    pub fn id_no_size_flags(&self) -> u8 {
        self.id & !(WP_IDF_LARGE | WP_IDF_ODD)
    }
}

/// Walk the `payload` buffer and collect every sub-block. Returns an
/// error if the on-disk size word would over-run the buffer.
pub fn parse_sub_blocks(payload: &[u8]) -> Result<Vec<SubBlock<'_>>> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < payload.len() {
        if i + 1 > payload.len() {
            return Err(Error::invalid("WavPack: sub-block id byte missing"));
        }
        let id = payload[i];
        i += 1;

        // Length is in 16-bit words. `LARGE` toggles 8-bit vs 24-bit
        // length encoding; `ODD` says the last word holds only one
        // payload byte (with a trailing pad byte that we strip).
        let words: usize = if (id & WP_IDF_LARGE) != 0 {
            if i + 3 > payload.len() {
                return Err(Error::invalid("WavPack: 24-bit sub-block size truncated"));
            }
            let n = u32::from_le_bytes([payload[i], payload[i + 1], payload[i + 2], 0]) as usize;
            i += 3;
            n
        } else {
            if i + 1 > payload.len() {
                return Err(Error::invalid("WavPack: 8-bit sub-block size truncated"));
            }
            let n = payload[i] as usize;
            i += 1;
            n
        };
        let on_disk_bytes = words * 2;
        if i + on_disk_bytes > payload.len() {
            return Err(Error::invalid(format!(
                "WavPack: sub-block id={id:#04x} would over-run block payload"
            )));
        }
        let payload_bytes = if (id & WP_IDF_ODD) != 0 {
            on_disk_bytes - 1
        } else {
            on_disk_bytes
        };
        out.push(SubBlock {
            id,
            data: &payload[i..i + payload_bytes],
        });
        i += on_disk_bytes;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flag_decoding_baseline_stereo() {
        // Hex from spec §3.4: 0x04bc1831 = stereo s16 joint-stereo
        // cross-decorr initial+final, mag=15, sr_idx=9 (44100 Hz).
        let f: u32 = 0x04bc_1831;
        assert_eq!(container_bps(f), 16);
        assert_eq!(magnitude_bits(f), 16);
        assert_eq!(sr_index(f), 9);
        assert_eq!(rate_for_index(sr_index(f)), Some(44_100));
        assert_eq!(shift_count(f), 0);
        assert!((f & WV_INITIAL_BLOCK) != 0);
        assert!((f & WV_FINAL_BLOCK) != 0);
        assert!((f & WV_MONO) == 0);
        assert!((f & WV_JOINT_STEREO) != 0);
        assert!((f & WV_CROSS_DECORR) != 0);
    }

    #[test]
    fn flag_decoding_24bit_stereo() {
        // Spec §3.4: 0x05501933 = bps=32 stereo INT32_DATA mag=20
        // sr_idx=10 (48000 Hz).
        let f: u32 = 0x0550_1933;
        assert_eq!(container_bps(f), 32);
        assert_eq!(magnitude_bits(f), 21);
        assert_eq!(sr_index(f), 10);
        assert_eq!(rate_for_index(sr_index(f)), Some(48_000));
        assert!((f & WV_INT32_DATA) != 0);
    }

    #[test]
    fn flag_decoding_float_stereo() {
        // Spec §3.4: 0x056018b3 = float stereo mag=24 sr_idx=10.
        let f: u32 = 0x0560_18b3;
        assert_eq!(container_bps(f), 32);
        assert_eq!(magnitude_bits(f), 25);
        assert_eq!(sr_index(f), 10);
        assert!((f & WV_FLOAT_DATA) != 0);
        assert!((f & WV_INT32_DATA) == 0);
    }

    #[test]
    fn parse_sub_block_8bit_size() {
        // id=0x02 (DECTERMS), size=2 words = 4 bytes.
        // Followed by 4 byte payload.
        let p = [0x02_u8, 0x02, 0xAA, 0xBB, 0xCC, 0xDD];
        let subs = parse_sub_blocks(&p).unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0].ty(), WP_ID_DECTERMS);
        assert_eq!(subs[0].data, &[0xAA, 0xBB, 0xCC, 0xDD]);
    }

    #[test]
    fn parse_sub_block_odd_pads_last_byte() {
        // id=0x67 = SAMPLE_RATE | ODD. The 5-bit type slot is 0x07
        // (which is also SHAPING) plus the IGNORE bit (0x20), so
        // SAMPLE_RATE matches against `id_no_size_flags() == 0x27`.
        // Size=2 words = 4 bytes on disk but ODD strips the trailing
        // pad byte → 3 bytes payload.
        let p = [0x67_u8, 0x02, 0x88, 0x90, 0x00, 0xFF];
        let subs = parse_sub_blocks(&p).unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0].id_no_size_flags(), WP_ID_SAMPLE_RATE);
        assert!(subs[0].ignore_flag());
        assert_eq!(subs[0].data, &[0x88, 0x90, 0x00]);
    }

    #[test]
    fn parse_sub_block_large_uses_24bit_size() {
        // id=0x8A = DATA | LARGE. size = 3 little-endian bytes = 2 words
        // = 4 bytes payload.
        let p = [0x8A_u8, 0x02, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44];
        let subs = parse_sub_blocks(&p).unwrap();
        assert_eq!(subs.len(), 1);
        assert_eq!(subs[0].ty(), WP_ID_DATA);
        assert_eq!(subs[0].data, &[0x11, 0x22, 0x33, 0x44]);
    }
}
