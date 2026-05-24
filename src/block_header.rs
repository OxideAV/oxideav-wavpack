//! WavPack v.4 block-header parser.
//!
//! Parses the 32-byte fixed header that precedes every WavPack block
//! (each `.wv` file is a concatenation of these blocks). The on-disk
//! layout — magic, total block size, version, track id, total samples,
//! block index, block samples, flags, CRC — is documented in the
//! "Block structure" listing of
//! `docs/audio/wavpack/wiki/WavPack.wiki` (block header section, all
//! fields little-endian).
//!
//! The bit-level meaning of every field in [`Flags`] is the
//! "Flags meaning" listing of the same wiki page. Bits whose meaning
//! the wiki marks as "reserved" or "experimental, okay to ignore" are
//! preserved verbatim so a caller can still inspect them; the parser
//! itself does not reject them.
//!
//! Round-1 scope is **structural only** — sample decode, decorrelation
//! pass and metadata-sub-block walking land in later rounds.

use crate::error::{Error, Result};

/// On-disk fixed length, in bytes, of the WavPack block header
/// (`docs/audio/wavpack/wiki/WavPack.wiki` block structure listing —
/// 4 + 4 + 2 + 1 + 1 + 4 + 4 + 4 + 4 + 4 = 32 bytes).
pub const HEADER_LEN: usize = 32;

/// Four-byte block magic — ASCII `'w','v','p','k'`, the first field of
/// every WavPack block per the same wiki listing.
pub const MAGIC: &[u8; 4] = b"wvpk";

/// Minimum value of the on-disk `ck_size` field. `ck_size` covers all
/// bytes of the block **except** the magic and `ck_size` fields
/// themselves; the remaining six fields of the fixed header sum to 24
/// bytes (2 + 1 + 1 + 4 + 4 + 4 + 4 + 4). A smaller value cannot even
/// describe an empty block.
pub const MIN_CK_SIZE: u32 = 24;

/// Lowest valid stream version per the wiki "valid versions are
/// 0x402 - 0x410" sentence.
pub const MIN_VERSION: u16 = 0x0402;
/// Highest valid stream version per the same sentence.
pub const MAX_VERSION: u16 = 0x0410;

/// Sentinel value for `total_samples` documented in the wiki as
/// "may be 0xFFFFFFFF if unknown".
pub const TOTAL_SAMPLES_UNKNOWN: u32 = 0xFFFF_FFFF;

/// Parsed WavPack block header.
///
/// All multi-byte integer fields are stored in little-endian on disk
/// per the wiki preamble "all data is stored in little-endian words".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WavPackBlockHeader {
    /// `ck_size` — total block size, **not** counting the four magic
    /// bytes nor the `ck_size` field itself. Always `>= 24`.
    pub ck_size: u32,
    /// Stream format version, currently `0x0402..=0x0410`.
    pub version: u16,
    /// Multi-track number (wiki: "not currently implemented"). Stored
    /// verbatim — callers may surface it for diagnostic round-trip.
    pub track_number: u8,
    /// Multi-track sub index (wiki: "not currently implemented").
    /// Stored verbatim for the same reason.
    pub track_sub_index: u8,
    /// Total samples in the file. Equal to [`TOTAL_SAMPLES_UNKNOWN`]
    /// when the encoder couldn't predict the total at write time.
    pub total_samples: u32,
    /// Sample offset of this block — how many samples should already
    /// have been emitted by the time this block is decoded (wiki:
    /// "how many samples should be decoded by now").
    pub block_index: u32,
    /// Number of samples this block contributes (`0` when the block
    /// carries metadata only and no audio).
    pub block_samples: u32,
    /// Decoded flag word — see [`Flags`].
    pub flags: Flags,
    /// CRC32 of the block payload, stored verbatim. The polynomial
    /// and span are spelled out by the wiki "Samples coding" section
    /// and the WavPack format-spec PDF the wiki page links to;
    /// recomputation is left to a later round (samples decode is a
    /// prerequisite for verifying the value).
    pub crc: u32,
}

/// Decoded view of the 32-bit `flags` word.
///
/// Every bit listed in the wiki "Flags meaning" section is exposed
/// here either as a typed sub-field (when the wiki defines a contiguous
/// multi-bit field) or a `bool` flag (when the wiki gives a single bit
/// a name).
///
/// Bits the wiki labels "reserved (okay to ignore)" /
/// "experimental, okay to ignore" / "do not decode if encountered" are
/// preserved as `reserved_bit_27`, `robust_block`, and
/// `low_latency_block` respectively, so a caller can still inspect
/// the underlying bit. The parser does **not** error on them — gating
/// happens at the decode layer landed in later rounds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Flags {
    /// Raw 32-bit word, preserved verbatim for callers that want to
    /// re-emit the block without round-tripping every accessor.
    pub raw: u32,
    /// Bits 0..=1: bytes-per-sample minus one. Decoded sample width
    /// in bytes is `bytes_per_sample_minus_one + 1` per the wiki
    /// "bytes per sample minus one" entry — so `0` -> 1 byte,
    /// `1` -> 2 bytes, `2` -> 3 bytes, `3` -> 4 bytes.
    pub bytes_per_sample_minus_one: u8,
    /// Bit 2: stream is monaural.
    pub mono: bool,
    /// Bit 3: hybrid (lossy) profile is in use.
    pub hybrid: bool,
    /// Bit 4: joint stereo coding scheme is in use.
    pub joint_stereo: bool,
    /// Bit 5: cross-channel decorrelation is in use.
    pub cross_channel_decorrelation: bool,
    /// Bit 6: hybrid-profile noise-shaping data is present in the
    /// metadata sub-blocks.
    pub hybrid_shaping: bool,
    /// Bit 7: floating-point sample data is present (otherwise the
    /// samples are integer).
    pub float_data: bool,
    /// Bit 8: int32 mode (samples occupy the full 32-bit container).
    pub int32_mode: bool,
    /// Bits 9..=10: hybrid-profile sub-flags (raw 2-bit value).
    pub hybrid_profile: u8,
    /// Bits 11..=12: multi-channel start / end block markers (raw
    /// 2-bit value). Bit 11 = "first block of multichannel set",
    /// bit 12 = "last block of multichannel set"; combined into the
    /// raw 2-bit value here so callers can decode either ordering.
    pub multichannel_marker: u8,
    /// Bits 13..=17: left-shift count when the bit-depth is not a
    /// multiple of 8 (e.g. 12-bit / 20-bit).
    pub left_shift: u8,
    /// Bits 18..=22: maximum magnitude of the decoded data, in bits
    /// (the wiki notes this is an optimisation hint for the decoder
    /// arithmetic).
    pub max_magnitude: u8,
    /// Bits 23..=26: sampling-rate index. The value `15` carries the
    /// wiki-defined meaning "unknown/custom" — when seen, the actual
    /// rate is carried in metadata sub-block `0x27`.
    pub sample_rate_index: u8,
    /// Bit 27: wiki "reserved (okay to ignore if encountered)" —
    /// preserved verbatim.
    pub reserved_bit_27: bool,
    /// Bit 28: wiki "robust block (experimental, okay to ignore)".
    pub robust_block: bool,
    /// Bit 29: IIR filter for negative noise shaping in hybrid mode.
    pub hybrid_noise_shaping_iir: bool,
    /// Bit 30: false stereo (the stream is stereo, but this specific
    /// block carries mono data). Wiki marks this as version >= 0x410.
    pub false_stereo: bool,
    /// Bit 31: wiki "low-latency block (experimental, do not decode
    /// if encountered)". Stored verbatim; gating is the decode
    /// layer's job.
    pub low_latency_block: bool,
}

impl Flags {
    /// Decode the 32-bit on-disk flag word per the wiki "Flags
    /// meaning" listing.
    pub fn from_raw(raw: u32) -> Self {
        let bytes_per_sample_minus_one = (raw & 0b11) as u8;
        let mono = (raw >> 2) & 1 != 0;
        let hybrid = (raw >> 3) & 1 != 0;
        let joint_stereo = (raw >> 4) & 1 != 0;
        let cross_channel_decorrelation = (raw >> 5) & 1 != 0;
        let hybrid_shaping = (raw >> 6) & 1 != 0;
        let float_data = (raw >> 7) & 1 != 0;
        let int32_mode = (raw >> 8) & 1 != 0;
        let hybrid_profile = ((raw >> 9) & 0b11) as u8;
        let multichannel_marker = ((raw >> 11) & 0b11) as u8;
        let left_shift = ((raw >> 13) & 0b1_1111) as u8;
        let max_magnitude = ((raw >> 18) & 0b1_1111) as u8;
        let sample_rate_index = ((raw >> 23) & 0b1111) as u8;
        let reserved_bit_27 = (raw >> 27) & 1 != 0;
        let robust_block = (raw >> 28) & 1 != 0;
        let hybrid_noise_shaping_iir = (raw >> 29) & 1 != 0;
        let false_stereo = (raw >> 30) & 1 != 0;
        let low_latency_block = (raw >> 31) & 1 != 0;

        Self {
            raw,
            bytes_per_sample_minus_one,
            mono,
            hybrid,
            joint_stereo,
            cross_channel_decorrelation,
            hybrid_shaping,
            float_data,
            int32_mode,
            hybrid_profile,
            multichannel_marker,
            left_shift,
            max_magnitude,
            sample_rate_index,
            reserved_bit_27,
            robust_block,
            hybrid_noise_shaping_iir,
            false_stereo,
            low_latency_block,
        }
    }

    /// Decoded sample width in bytes (`1..=4`) per the wiki
    /// "bytes per sample minus one" entry on bits 0..=1.
    pub fn bytes_per_sample(&self) -> u8 {
        self.bytes_per_sample_minus_one + 1
    }

    /// Number of channels carried by this block — `1` when the
    /// [`Self::mono`] bit is set, `2` otherwise. This is the local
    /// per-block channel count; multi-channel files convey the global
    /// layout through metadata sub-block `0x0D`.
    pub fn channels_in_block(&self) -> u8 {
        if self.mono {
            1
        } else {
            2
        }
    }

    /// `true` when bit 11 of the flag word is set — this block is the
    /// **first** block of a multi-channel set (wiki bits 11..=12
    /// "multi-channel start and end blocks").
    pub fn is_first_block(&self) -> bool {
        (self.multichannel_marker & 0b01) != 0
    }

    /// `true` when bit 12 of the flag word is set — this block is the
    /// **last** block of a multi-channel set.
    pub fn is_final_block(&self) -> bool {
        (self.multichannel_marker & 0b10) != 0
    }

    /// `true` when this block is both the first and last block of a
    /// multi-channel set — i.e. the set has exactly one block, the
    /// stand-alone mono / stereo case the wiki "multi-channel start
    /// and end blocks" pair degenerates to when there is no multi-
    /// channel grouping. Mathematically `multichannel_marker == 0b11`.
    ///
    /// This is the common case for plain stereo `.wv` files: every
    /// block carries both markers because every block is its own
    /// (single-block) set.
    pub fn is_standalone_block(&self) -> bool {
        self.multichannel_marker == 0b11
    }

    /// `true` when this block is part of a multi-block multi-channel
    /// set — i.e. it is **not** both first and last simultaneously.
    /// Inverse of [`Self::is_standalone_block`]. The wiki "multi-channel
    /// start and end blocks" pair is degenerate (both bits set) when
    /// the set contains a single block; any other marker combination
    /// (`0b00`, `0b01`, `0b10`) means this block participates in a
    /// multi-block channel grouping whose layout is conveyed via the
    /// `0x0D` multichannel-info sub-block.
    pub fn is_multichannel_member(&self) -> bool {
        self.multichannel_marker != 0b11
    }

    /// `true` when the stream is encoded with the **lossless** profile —
    /// the wiki bit 3 "hybrid profile (lossy compression)" is **not**
    /// set. WavPack's two profiles are lossless and hybrid; the absence
    /// of the hybrid bit is the lossless indicator.
    pub fn is_lossless(&self) -> bool {
        !self.hybrid
    }

    /// `true` when the stream is encoded with the **hybrid** (lossy)
    /// profile. Equivalent to [`Self::hybrid`]; provided as the natural
    /// counterpart to [`Self::is_lossless`] for callers that want to
    /// branch on profile by name.
    pub fn is_lossy(&self) -> bool {
        self.hybrid
    }

    /// `true` when the wiki bits 23..=26 sampling-rate index is the
    /// sentinel value `15` documented as "unknown/custom". When set the
    /// actual sample rate is carried out-of-band in metadata sub-block
    /// `0x27` ("non-standard sampling rate" per the wiki IDs listing).
    pub fn has_custom_sample_rate(&self) -> bool {
        self.sample_rate_index == 15
    }

    /// `true` when the wiki bit 31 "low-latency block (experimental,
    /// **do not decode if encountered**)" is set. The wiki gates the
    /// decode-time response on this bit explicitly; the parser preserves
    /// the flag and this accessor surfaces the wiki contract for the
    /// decode layer.
    pub fn should_skip_decode(&self) -> bool {
        self.low_latency_block
    }

    /// `true` when **any** of the two wiki-labelled experimental bits
    /// — bit 28 "robust block (experimental, okay to ignore)" and
    /// bit 31 "low-latency block (experimental, do not decode if
    /// encountered)" — is set. Use this for diagnostic surfaces;
    /// callers gating decode should use [`Self::should_skip_decode`]
    /// (only bit 31 carries the wiki "do not decode" instruction).
    pub fn is_experimental(&self) -> bool {
        self.robust_block || self.low_latency_block
    }

    /// Effective sample bit-depth, derived from the container width
    /// minus the [`Self::left_shift`] correction documented in the wiki
    /// "left-shift places when bitdepth is not a multiple of 8 (e.g.
    /// 12-bit, 20-bit)" entry on bits 13..=17.
    ///
    /// For a 16-bit container (`bytes_per_sample = 2`) with `left_shift
    /// = 4`, the effective bit-depth is `16 - 4 = 12`. Returns `0` when
    /// `left_shift` exceeds the container width (a malformed combination
    /// the wiki does not specifically forbid but the parser does not
    /// reject either — it preserves the raw fields verbatim).
    pub fn effective_bit_depth(&self) -> u8 {
        let container_bits = self.bytes_per_sample() * 8;
        container_bits.saturating_sub(self.left_shift)
    }
}

impl WavPackBlockHeader {
    /// `true` when this block carries audio samples — the wiki
    /// "block samples" field on the block-structure listing is
    /// non-zero. The wiki notes block_samples "may be 0 if no audio
    /// present", i.e. a metadata-only block (RIFF chunks, MD5 sums,
    /// etc.). Decoders should typically skip the decorrelation /
    /// samples-coding pipeline for non-audio blocks.
    pub fn is_audio_block(&self) -> bool {
        self.block_samples != 0
    }

    /// `true` when the on-disk `total_samples` field carried a real
    /// count rather than the [`TOTAL_SAMPLES_UNKNOWN`] sentinel the
    /// wiki documents as "may be 0xFFFFFFFF if unknown". A streaming
    /// encoder that cannot know the total at write time emits the
    /// sentinel.
    pub fn is_total_samples_known(&self) -> bool {
        self.total_samples != TOTAL_SAMPLES_UNKNOWN
    }

    /// Number of payload bytes the block-header `ck_size` field
    /// advertises — i.e. the byte count of the metadata sub-block
    /// region that follows the 32-byte fixed header.
    ///
    /// The wiki defines `ck_size` as "total block size (not counting
    /// this field or 'wvpk')" — the 24 fixed bytes after `ck_size`
    /// itself (version through CRC) plus whatever metadata sub-blocks
    /// trail. This accessor subtracts the 24 fixed bytes to give
    /// callers the metadata region length directly.
    ///
    /// `ck_size` is validated to be at least 24 by [`parse_block_header`]
    /// so this subtraction never underflows on a parser-returned header.
    pub fn payload_bytes(&self) -> u32 {
        self.ck_size - 24
    }
}

/// Parse the 32-byte WavPack block header at the start of `bytes`.
///
/// Returns the typed [`WavPackBlockHeader`] alongside the unconsumed
/// payload (`bytes[HEADER_LEN..]`). The payload length is **not**
/// validated against `ck_size` here — caller-side framing logic
/// (sub-block walking, multi-block streaming) will do that against
/// the spec's `ck_size` definition in the same wiki listing.
pub fn parse_block_header(bytes: &[u8]) -> Result<(WavPackBlockHeader, &[u8])> {
    if bytes.len() < HEADER_LEN {
        return Err(Error::Truncated);
    }
    if &bytes[0..4] != MAGIC {
        return Err(Error::InvalidMagic);
    }
    let ck_size = u32_le(&bytes[4..8]);
    if ck_size < MIN_CK_SIZE {
        return Err(Error::InvalidCkSize(ck_size));
    }
    let version = u16_le(&bytes[8..10]);
    if !(MIN_VERSION..=MAX_VERSION).contains(&version) {
        return Err(Error::UnsupportedVersion(version));
    }
    let track_number = bytes[10];
    let track_sub_index = bytes[11];
    let total_samples = u32_le(&bytes[12..16]);
    let block_index = u32_le(&bytes[16..20]);
    let block_samples = u32_le(&bytes[20..24]);
    let flags = Flags::from_raw(u32_le(&bytes[24..28]));
    let crc = u32_le(&bytes[28..32]);

    let header = WavPackBlockHeader {
        ck_size,
        version,
        track_number,
        track_sub_index,
        total_samples,
        block_index,
        block_samples,
        flags,
        crc,
    };
    Ok((header, &bytes[HEADER_LEN..]))
}

#[inline]
fn u16_le(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

#[inline]
fn u32_le(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Round-trip the smallest valid header — a minimum-sized
    /// metadata-only block (ck_size = 24, no payload).
    fn synthesise_minimal_header(flags: u32) -> [u8; HEADER_LEN] {
        let mut buf = [0u8; HEADER_LEN];
        buf[0..4].copy_from_slice(MAGIC);
        buf[4..8].copy_from_slice(&MIN_CK_SIZE.to_le_bytes()); // ck_size = 24
        buf[8..10].copy_from_slice(&0x0410u16.to_le_bytes()); // current version
        buf[10] = 0; // track_number
        buf[11] = 0; // track_sub_index
        buf[12..16].copy_from_slice(&TOTAL_SAMPLES_UNKNOWN.to_le_bytes());
        buf[16..20].copy_from_slice(&0u32.to_le_bytes()); // block_index
        buf[20..24].copy_from_slice(&0u32.to_le_bytes()); // block_samples
        buf[24..28].copy_from_slice(&flags.to_le_bytes());
        buf[28..32].copy_from_slice(&0u32.to_le_bytes()); // crc
        buf
    }

    #[test]
    fn parses_minimal_metadata_block() {
        // Flags = 0 ⇒ 1 byte/sample, stereo, lossless, integer, etc.
        let buf = synthesise_minimal_header(0);
        let (h, tail) = parse_block_header(&buf).unwrap();
        assert_eq!(h.ck_size, MIN_CK_SIZE);
        assert_eq!(h.version, 0x0410);
        assert_eq!(h.track_number, 0);
        assert_eq!(h.track_sub_index, 0);
        assert_eq!(h.total_samples, TOTAL_SAMPLES_UNKNOWN);
        assert_eq!(h.block_index, 0);
        assert_eq!(h.block_samples, 0);
        assert_eq!(h.flags.raw, 0);
        assert_eq!(h.crc, 0);
        assert!(tail.is_empty());
    }

    #[test]
    fn rejects_truncated_input() {
        let buf = synthesise_minimal_header(0);
        assert_eq!(
            parse_block_header(&buf[..HEADER_LEN - 1]),
            Err(Error::Truncated)
        );
        assert_eq!(parse_block_header(&[]), Err(Error::Truncated));
    }

    #[test]
    fn rejects_bad_magic() {
        let mut buf = synthesise_minimal_header(0);
        buf[0] = b'X';
        assert_eq!(parse_block_header(&buf), Err(Error::InvalidMagic));
    }

    #[test]
    fn rejects_undersized_ck_size() {
        let mut buf = synthesise_minimal_header(0);
        buf[4..8].copy_from_slice(&0u32.to_le_bytes());
        assert_eq!(parse_block_header(&buf), Err(Error::InvalidCkSize(0)));
        buf[4..8].copy_from_slice(&23u32.to_le_bytes());
        assert_eq!(parse_block_header(&buf), Err(Error::InvalidCkSize(23)));
    }

    #[test]
    fn rejects_version_outside_window() {
        let mut buf = synthesise_minimal_header(0);
        buf[8..10].copy_from_slice(&0x0401u16.to_le_bytes());
        assert_eq!(
            parse_block_header(&buf),
            Err(Error::UnsupportedVersion(0x0401))
        );
        buf[8..10].copy_from_slice(&0x0411u16.to_le_bytes());
        assert_eq!(
            parse_block_header(&buf),
            Err(Error::UnsupportedVersion(0x0411))
        );
        // Both bounds of the inclusive window are accepted.
        for v in [MIN_VERSION, MAX_VERSION] {
            let mut b = synthesise_minimal_header(0);
            b[8..10].copy_from_slice(&v.to_le_bytes());
            assert!(
                parse_block_header(&b).is_ok(),
                "version 0x{v:04x} should be accepted"
            );
        }
    }

    #[test]
    fn decodes_field_layout_exhaustively() {
        // Construct a flag word that puts a distinct nonzero value in
        // every multi-bit sub-field so each bit-range can be checked
        // independently.
        //
        // bytes_per_sample_minus_one (0..=1)   = 0b11 (4 bytes/sample)
        // mono                       (2)        = 1
        // hybrid                     (3)        = 0
        // joint_stereo               (4)        = 1
        // cross_channel_decorr       (5)        = 0
        // hybrid_shaping             (6)        = 1
        // float_data                 (7)        = 0
        // int32_mode                 (8)        = 1
        // hybrid_profile             (9..=10)  = 0b10
        // multichannel_marker        (11..=12) = 0b11 (both first + final)
        // left_shift                 (13..=17) = 0b1_0101 (21)
        // max_magnitude              (18..=22) = 0b0_1010 (10)
        // sample_rate_index          (23..=26) = 0b0111 (7)
        // reserved_bit_27            (27)       = 1
        // robust_block               (28)       = 0
        // hybrid_noise_shaping_iir   (29)       = 1
        // false_stereo               (30)       = 0
        // low_latency_block          (31)       = 1
        // Bits cleared (0 << N) are dropped from the mask since they
        // contribute nothing; the comment above each set bit names
        // which wiki "Flags meaning" entry it covers.
        let raw: u32 = 0b11             // bits 0..=1 bytes_per_sample-1 = 3
            | (1u32 << 2)               // bit 2  mono
            | (1u32 << 4)               // bit 4  joint_stereo
            | (1u32 << 6)               // bit 6  hybrid_shaping
            | (1u32 << 8)               // bit 8  int32_mode
            | (0b10u32 << 9)            // bits 9..=10 hybrid_profile = 2
            | (0b11u32 << 11)           // bits 11..=12 multichannel_marker = 3
            | (0b1_0101u32 << 13)       // bits 13..=17 left_shift = 21
            | (0b0_1010u32 << 18)       // bits 18..=22 max_magnitude = 10
            | (0b0111u32 << 23)         // bits 23..=26 sample_rate_index = 7
            | (1u32 << 27)              // bit 27 reserved_bit_27
            | (1u32 << 29)              // bit 29 hybrid_noise_shaping_iir
            | (1u32 << 31); // bit 31 low_latency_block

        let f = Flags::from_raw(raw);
        assert_eq!(f.raw, raw);
        assert_eq!(f.bytes_per_sample_minus_one, 0b11);
        assert_eq!(f.bytes_per_sample(), 4);
        assert!(f.mono);
        assert!(!f.hybrid);
        assert!(f.joint_stereo);
        assert!(!f.cross_channel_decorrelation);
        assert!(f.hybrid_shaping);
        assert!(!f.float_data);
        assert!(f.int32_mode);
        assert_eq!(f.hybrid_profile, 0b10);
        assert_eq!(f.multichannel_marker, 0b11);
        assert!(f.is_first_block());
        assert!(f.is_final_block());
        assert_eq!(f.left_shift, 0b1_0101);
        assert_eq!(f.max_magnitude, 0b0_1010);
        assert_eq!(f.sample_rate_index, 0b0111);
        assert!(f.reserved_bit_27);
        assert!(!f.robust_block);
        assert!(f.hybrid_noise_shaping_iir);
        assert!(!f.false_stereo);
        assert!(f.low_latency_block);
        assert_eq!(f.channels_in_block(), 1);
    }

    #[test]
    fn channel_count_falls_back_to_stereo_when_mono_bit_clear() {
        let f = Flags::from_raw(0);
        assert!(!f.mono);
        assert_eq!(f.channels_in_block(), 2);
    }

    #[test]
    fn multichannel_first_only_and_last_only_are_independent() {
        // bit 11 only — first block, not final
        let only_first = Flags::from_raw(0b01 << 11);
        assert!(only_first.is_first_block());
        assert!(!only_first.is_final_block());
        // bit 12 only — final block, not first
        let only_last = Flags::from_raw(0b10 << 11);
        assert!(!only_last.is_first_block());
        assert!(only_last.is_final_block());
    }

    #[test]
    fn parses_block_with_payload_and_returns_tail() {
        // ck_size advertises 24 + 8 payload bytes; the parser doesn't
        // validate the payload length, but it must hand back the
        // post-header slice verbatim.
        let mut buf = vec![0u8; HEADER_LEN + 8];
        let hdr = synthesise_minimal_header(0);
        buf[..HEADER_LEN].copy_from_slice(&hdr);
        // Overwrite ck_size to advertise the 8-byte payload.
        buf[4..8].copy_from_slice(&(MIN_CK_SIZE + 8).to_le_bytes());
        buf[HEADER_LEN..].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44]);
        let (h, tail) = parse_block_header(&buf).unwrap();
        assert_eq!(h.ck_size, MIN_CK_SIZE + 8);
        assert_eq!(tail, &[0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44]);
    }

    #[test]
    fn is_standalone_block_requires_both_markers() {
        // Both bits 11 and 12 set — the wiki degenerate "single-block
        // set" case (also the natural state for a plain stereo file).
        let f = Flags::from_raw(0b11 << 11);
        assert!(f.is_standalone_block());
        assert!(!f.is_multichannel_member());
    }

    #[test]
    fn is_multichannel_member_when_any_marker_combo_other_than_both() {
        for raw_marker in [0b00u32, 0b01, 0b10] {
            let f = Flags::from_raw(raw_marker << 11);
            assert!(
                f.is_multichannel_member(),
                "marker {raw_marker:02b} should mark this block as a multichannel member"
            );
            assert!(
                !f.is_standalone_block(),
                "marker {raw_marker:02b} should NOT be standalone"
            );
        }
    }

    #[test]
    fn is_lossless_when_hybrid_bit_clear() {
        // Bit 3 clear → lossless profile per the wiki "hybrid profile
        // (lossy compression)" labelling.
        let f = Flags::from_raw(0);
        assert!(f.is_lossless());
        assert!(!f.is_lossy());
        // And vice-versa: bit 3 set → lossy / hybrid.
        let f = Flags::from_raw(1 << 3);
        assert!(!f.is_lossless());
        assert!(f.is_lossy());
        assert!(f.hybrid);
    }

    #[test]
    fn has_custom_sample_rate_only_at_sentinel_15() {
        // bits 23..=26 = 15 → unknown/custom per the wiki sentinel.
        let f = Flags::from_raw(15u32 << 23);
        assert_eq!(f.sample_rate_index, 15);
        assert!(f.has_custom_sample_rate());
        // Every other index in the 4-bit range is a regular table entry.
        for idx in 0u32..15 {
            let f = Flags::from_raw(idx << 23);
            assert_eq!(f.sample_rate_index as u32, idx);
            assert!(
                !f.has_custom_sample_rate(),
                "sample_rate_index {idx} should not be 'custom'"
            );
        }
    }

    #[test]
    fn should_skip_decode_tracks_bit_31_only() {
        // Wiki: bit 31 carries the "do not decode if encountered"
        // instruction. Bit 28 ("experimental, okay to ignore") does
        // NOT instruct a skip — it's tolerated.
        let f = Flags::from_raw(1u32 << 31);
        assert!(f.should_skip_decode());
        let f = Flags::from_raw(1u32 << 28);
        assert!(!f.should_skip_decode());
        let f = Flags::from_raw(0);
        assert!(!f.should_skip_decode());
    }

    #[test]
    fn is_experimental_is_union_of_robust_and_low_latency() {
        // Both wiki "experimental" bits feed the diagnostic predicate.
        let f = Flags::from_raw(0);
        assert!(!f.is_experimental());
        let f = Flags::from_raw(1u32 << 28);
        assert!(f.is_experimental());
        let f = Flags::from_raw(1u32 << 31);
        assert!(f.is_experimental());
        let f = Flags::from_raw((1u32 << 28) | (1u32 << 31));
        assert!(f.is_experimental());
    }

    #[test]
    fn effective_bit_depth_subtracts_left_shift() {
        // 16-bit container, left_shift = 4 → 12-bit effective depth
        // per the wiki "left-shift places when bitdepth is not a
        // multiple of 8 (e.g. 12-bit, 20-bit)" entry.
        let raw = 0b01u32 // bytes_per_sample_minus_one = 1 → 2 bytes
            | (4u32 << 13); // left_shift = 4
        let f = Flags::from_raw(raw);
        assert_eq!(f.bytes_per_sample(), 2);
        assert_eq!(f.left_shift, 4);
        assert_eq!(f.effective_bit_depth(), 12);

        // 24-bit container, left_shift = 4 → 20-bit effective depth
        // (the other wiki worked example).
        let raw = 0b10u32 // bytes_per_sample_minus_one = 2 → 3 bytes
            | (4u32 << 13); // left_shift = 4
        let f = Flags::from_raw(raw);
        assert_eq!(f.bytes_per_sample(), 3);
        assert_eq!(f.effective_bit_depth(), 20);

        // No left-shift → effective bit-depth matches the container.
        let f = Flags::from_raw(0b11); // 4 bytes/sample, no shift
        assert_eq!(f.effective_bit_depth(), 32);
    }

    #[test]
    fn effective_bit_depth_saturates_when_left_shift_overflows() {
        // 1-byte container with left_shift = 31 → saturate to 0 rather
        // than underflow. The wiki does not specifically forbid this
        // combination so the parser preserves the raw fields; the
        // accessor degrades gracefully.
        let raw = (31u32 << 13) & ((1u32 << 18) - 1); // left_shift = 31
        let f = Flags::from_raw(raw);
        assert_eq!(f.bytes_per_sample(), 1);
        assert_eq!(f.left_shift, 31);
        assert_eq!(f.effective_bit_depth(), 0);
    }

    #[test]
    fn is_audio_block_keys_on_block_samples_field() {
        // block_samples > 0 → audio block.
        let mut buf = synthesise_minimal_header(0);
        buf[20..24].copy_from_slice(&1024u32.to_le_bytes());
        let (h, _) = parse_block_header(&buf).unwrap();
        assert!(h.is_audio_block());

        // block_samples == 0 → metadata-only block per the wiki note
        // "may be 0 if no audio present".
        let buf = synthesise_minimal_header(0);
        let (h, _) = parse_block_header(&buf).unwrap();
        assert!(!h.is_audio_block());
        assert_eq!(h.block_samples, 0);
    }

    #[test]
    fn is_total_samples_known_distinguishes_sentinel() {
        // The minimal header uses the sentinel by construction.
        let buf = synthesise_minimal_header(0);
        let (h, _) = parse_block_header(&buf).unwrap();
        assert!(!h.is_total_samples_known());
        assert_eq!(h.total_samples, TOTAL_SAMPLES_UNKNOWN);

        // Any other value → "known".
        let mut buf = synthesise_minimal_header(0);
        buf[12..16].copy_from_slice(&123_456u32.to_le_bytes());
        let (h, _) = parse_block_header(&buf).unwrap();
        assert!(h.is_total_samples_known());
        assert_eq!(h.total_samples, 123_456);

        // And the boundary case `0` (a real count of zero samples) is
        // also "known" — it differs from the sentinel.
        let mut buf = synthesise_minimal_header(0);
        buf[12..16].copy_from_slice(&0u32.to_le_bytes());
        let (h, _) = parse_block_header(&buf).unwrap();
        assert!(h.is_total_samples_known());
    }

    #[test]
    fn payload_bytes_subtracts_fixed_header_minimum() {
        // Minimum ck_size = 24 → zero payload bytes (metadata-empty
        // block).
        let buf = synthesise_minimal_header(0);
        let (h, _) = parse_block_header(&buf).unwrap();
        assert_eq!(h.payload_bytes(), 0);

        // ck_size = 24 + 100 → 100 payload bytes.
        let mut buf = vec![0u8; HEADER_LEN + 100];
        let hdr = synthesise_minimal_header(0);
        buf[..HEADER_LEN].copy_from_slice(&hdr);
        buf[4..8].copy_from_slice(&(MIN_CK_SIZE + 100).to_le_bytes());
        let (h, _) = parse_block_header(&buf).unwrap();
        assert_eq!(h.payload_bytes(), 100);
    }

    #[test]
    fn endianness_matches_wiki_preamble() {
        // The wiki preamble says "all data is stored in little-endian
        // words". Force a high-byte value into each multi-byte field
        // and assert the parser decodes it as little-endian.
        let mut buf = synthesise_minimal_header(0);
        // ck_size = 0x0000_0100 (LE bytes 0x00, 0x01, 0x00, 0x00)
        buf[4..8].copy_from_slice(&[0x00, 0x01, 0x00, 0x00]);
        // version = 0x0410 (LE bytes 0x10, 0x04)
        buf[8..10].copy_from_slice(&[0x10, 0x04]);
        // total_samples = 0x1234_5678 (LE bytes 0x78, 0x56, 0x34, 0x12)
        buf[12..16].copy_from_slice(&[0x78, 0x56, 0x34, 0x12]);
        let (h, _) = parse_block_header(&buf).unwrap();
        assert_eq!(h.ck_size, 0x0000_0100);
        assert_eq!(h.version, 0x0410);
        assert_eq!(h.total_samples, 0x1234_5678);
    }
}
