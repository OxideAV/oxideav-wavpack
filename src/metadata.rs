//! WavPack v.4 metadata sub-block walker.
//!
//! Walks the metadata sub-blocks that follow the 32-byte fixed header
//! inside a WavPack block. The on-disk layout — ID byte + size field
//! (1-byte or 3-byte) + payload — is documented in the "Metadata"
//! section of `docs/audio/wavpack/wiki/WavPack.wiki`:
//!
//! > Metadata can be divided into three parts: ID, length and data.
//! > Every metadata block has even length and data size is stored in
//! > words in either one or three bytes depending on ID flag.
//! >
//! > Flags for ID:
//! >   0x20 - decoder may ignore data contained here
//! >   0x40 - data size is odd
//! >   0x80 - data size is large
//!
//! The two flag bits `0x40` (odd-size) and `0x80` (large-size) are
//! structural — they affect how the walker decodes the size field
//! and payload length. The `0x20` "optional" flag is part of the
//! ID identification: the wiki lists separate names for the
//! `0x00..=0x0D` IDs and for the same low-5-bit numbers with the
//! `0x20` bit set (`0x20..=0x27`). The walker therefore preserves
//! both views — the raw ID byte AND a typed [`SubBlockId`] that
//! combines the low-5-bit number with the `0x20` optional flag.
//!
//! Size is stored as a count of 16-bit **words**. Whether the size
//! field is one byte or three bytes (little-endian 24-bit) depends
//! on the `0x80` "large" flag in the ID byte. The byte length of
//! the payload is `words * 2`, with the trailing byte being padding
//! when the `0x40` "odd" flag is set (the wiki preamble guarantees
//! every metadata block has even total length).
//!
//! Round-2 scope is **structural only** — the walker produces a
//! `Vec<MetadataSubBlock>` of typed `(SubBlockId, payload bytes)`
//! pairs. Interpreting the payload for any specific ID
//! (decorrelation terms / weights / samples, entropy info, packed
//! samples, channel mask, MD5, etc.) lands in subsequent rounds.

use crate::error::{Error, Result};

/// Flag bit on the ID byte: "decoder may ignore data contained here"
/// (wiki "Flags for ID" listing, first entry). The wiki's separate
/// `0x20..=0x27` ID listing is exactly the `0x00..=0x07` low-5-bit
/// numbers re-named for the case where this flag is set.
pub const ID_FLAG_OPTIONAL: u8 = 0x20;
/// Flag bit on the ID byte: "data size is odd" — the size field
/// stores a count of 16-bit words and the final byte of the payload
/// is padding the encoder added to keep the total even.
pub const ID_FLAG_ODD_SIZE: u8 = 0x40;
/// Flag bit on the ID byte: "data size is large" — the size field
/// occupies three bytes (little-endian 24-bit) instead of the default
/// one byte.
pub const ID_FLAG_LARGE_SIZE: u8 = 0x80;
/// Mask isolating the 5-bit ID number from the ID byte.
pub const ID_MASK: u8 = 0x1F;

/// Sub-block ID enumerated by the wiki "IDs" listing.
///
/// The low 5 bits of the on-disk ID byte and the `0x20` optional
/// flag together discriminate the variant — the wiki's "IDs"
/// section names `0x00..=0x0D` (without the optional flag) and
/// `0x20..=0x27` (with the optional flag set) as separate entries
/// even though the low-5-bit ID number overlaps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubBlockId {
    /// `0x00` — dummy (used for padding).
    Dummy,
    /// `0x02` — decorrelation terms.
    DecorrelationTerms,
    /// `0x03` — decorrelation weights.
    DecorrelationWeights,
    /// `0x04` — decorrelation samples.
    DecorrelationSamples,
    /// `0x05` — entropy info.
    EntropyInfo,
    /// `0x06` — hybrid profile.
    HybridProfile,
    /// `0x07` — noise shaping profile (carried in the `.wvc`
    /// correction file).
    NoiseShapingProfile,
    /// `0x08` — floating-point data profile.
    FloatInfo,
    /// `0x09` — large or shifted integer profile.
    Int32Info,
    /// `0x0A` — packed samples (the entropy-coded audio payload).
    PackedSamples,
    /// `0x0B` — packed correction data (carried in the `.wvc`
    /// correction file).
    PackedCorrectionData,
    /// `0x0C` — packed overflow bits from floating-point or large
    /// integer profiles.
    PackedOverflowBits,
    /// `0x0D` — multichannel information (including Microsoft
    /// channel mask).
    MultichannelInfo,
    /// `0x20` — RIFF header for `.wav` files (before audio).
    /// Same low-5-bit number as [`Self::Dummy`], distinguished by
    /// the [`ID_FLAG_OPTIONAL`] bit being set on the ID byte.
    RiffHeader,
    /// `0x21` — RIFF trailer for `.wav` files (after audio).
    RiffTrailer,
    /// `0x25` — some encoding details for informational purposes.
    EncodingDetails,
    /// `0x26` — 16-byte MD5 checksum of the raw audio data.
    Md5Checksum,
    /// `0x27` — non-standard sampling rate.
    NonStandardSampleRate,
    /// An ID number whose `(low-5-bits, optional-flag)` combination
    /// isn't named by the wiki "IDs" listing. The walker surfaces
    /// the raw payload so a caller can still inspect it — the
    /// `0x20` "decoder may ignore" mechanism makes forward
    /// compatibility a first-class concern. The contained byte is
    /// the masked-and-merged ID identifier (low 5 bits of the ID
    /// byte OR'ed with the `0x20` flag, i.e. always in the
    /// `0x00..=0x3F` range).
    Unknown(u8),
}

impl SubBlockId {
    /// Decode the sub-block ID from the on-disk ID byte. Only the
    /// low 5 bits and the `0x20` optional-flag bit are inspected;
    /// the `0x40` odd-size and `0x80` large-size structural flags
    /// are decoded separately into [`SubBlockFlags`].
    pub fn from_id_byte(id_byte: u8) -> Self {
        // Combine the low 5 bits of the ID number with the 0x20
        // optional flag — this is the same 6-bit identifier the
        // wiki "IDs" listing uses.
        let key = id_byte & (ID_MASK | ID_FLAG_OPTIONAL);
        match key {
            0x00 => SubBlockId::Dummy,
            0x02 => SubBlockId::DecorrelationTerms,
            0x03 => SubBlockId::DecorrelationWeights,
            0x04 => SubBlockId::DecorrelationSamples,
            0x05 => SubBlockId::EntropyInfo,
            0x06 => SubBlockId::HybridProfile,
            0x07 => SubBlockId::NoiseShapingProfile,
            0x08 => SubBlockId::FloatInfo,
            0x09 => SubBlockId::Int32Info,
            0x0A => SubBlockId::PackedSamples,
            0x0B => SubBlockId::PackedCorrectionData,
            0x0C => SubBlockId::PackedOverflowBits,
            0x0D => SubBlockId::MultichannelInfo,
            0x20 => SubBlockId::RiffHeader,
            0x21 => SubBlockId::RiffTrailer,
            0x25 => SubBlockId::EncodingDetails,
            0x26 => SubBlockId::Md5Checksum,
            0x27 => SubBlockId::NonStandardSampleRate,
            other => SubBlockId::Unknown(other),
        }
    }

    /// Recover the canonical 6-bit ID identifier for [`Self::from_id_byte`]'s
    /// reverse direction. The returned byte includes the `0x20`
    /// optional flag for the `0x20..=0x27` entries but not the
    /// `0x40` / `0x80` structural flags.
    pub fn as_id_byte(&self) -> u8 {
        match self {
            SubBlockId::Dummy => 0x00,
            SubBlockId::DecorrelationTerms => 0x02,
            SubBlockId::DecorrelationWeights => 0x03,
            SubBlockId::DecorrelationSamples => 0x04,
            SubBlockId::EntropyInfo => 0x05,
            SubBlockId::HybridProfile => 0x06,
            SubBlockId::NoiseShapingProfile => 0x07,
            SubBlockId::FloatInfo => 0x08,
            SubBlockId::Int32Info => 0x09,
            SubBlockId::PackedSamples => 0x0A,
            SubBlockId::PackedCorrectionData => 0x0B,
            SubBlockId::PackedOverflowBits => 0x0C,
            SubBlockId::MultichannelInfo => 0x0D,
            SubBlockId::RiffHeader => 0x20,
            SubBlockId::RiffTrailer => 0x21,
            SubBlockId::EncodingDetails => 0x25,
            SubBlockId::Md5Checksum => 0x26,
            SubBlockId::NonStandardSampleRate => 0x27,
            SubBlockId::Unknown(v) => *v & (ID_MASK | ID_FLAG_OPTIONAL),
        }
    }
}

/// Decoded view of the structural flag bits on the ID byte
/// (everything except the `0x20` optional bit, which is folded into
/// [`SubBlockId`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubBlockFlags {
    /// `0x20` set — the wiki marks this sub-block as "decoder may
    /// ignore data contained here". Exposed here as a bool for
    /// callers that want to inspect the bit directly; the
    /// [`SubBlockId`] enum also folds it into the variant name
    /// (e.g. `RiffHeader` is `Dummy` + optional flag).
    pub optional: bool,
    /// `0x40` set — the size field counts 16-bit words, but the last
    /// byte of the payload is encoder-supplied padding (the actual
    /// data is one byte shorter).
    pub odd_size: bool,
    /// `0x80` set — the size field is three bytes (little-endian
    /// 24-bit) instead of one byte.
    pub large_size: bool,
}

impl SubBlockFlags {
    /// Decode the flag triple from an ID byte.
    pub fn from_id_byte(id_byte: u8) -> Self {
        Self {
            optional: id_byte & ID_FLAG_OPTIONAL != 0,
            odd_size: id_byte & ID_FLAG_ODD_SIZE != 0,
            large_size: id_byte & ID_FLAG_LARGE_SIZE != 0,
        }
    }
}

/// One typed metadata sub-block produced by [`walk_metadata`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MetadataSubBlock<'a> {
    /// The full ID byte as it appears on disk, including the high
    /// 3 flag bits — preserved verbatim for callers that want to
    /// re-emit the block without re-encoding the flag bits.
    pub id_byte: u8,
    /// Decoded sub-block ID (low 5 bits + 0x20 optional bit of
    /// [`Self::id_byte`]).
    pub id: SubBlockId,
    /// Decoded flag triple (the `0x20` / `0x40` / `0x80` bits of
    /// [`Self::id_byte`]).
    pub flags: SubBlockFlags,
    /// Size field as it appears on disk, in 16-bit words. The
    /// payload byte length is `size_words * 2`, minus one when
    /// [`SubBlockFlags::odd_size`] is set.
    pub size_words: u32,
    /// The sub-block payload. When [`SubBlockFlags::odd_size`] is
    /// set the trailing padding byte (the encoder added to keep the
    /// stream even-length) has already been stripped, so
    /// `payload.len()` is the meaningful data length only.
    pub payload: &'a [u8],
}

/// Walk all metadata sub-blocks in the given byte slice (typically
/// the post-header payload returned by
/// [`crate::block_header::parse_block_header`]).
///
/// The walker consumes one sub-block per iteration and stops cleanly
/// when the input is exhausted. A partial sub-block at the end of
/// the stream is reported as [`Error::Truncated`].
pub fn walk_metadata(mut bytes: &[u8]) -> Result<Vec<MetadataSubBlock<'_>>> {
    let mut out = Vec::new();
    while !bytes.is_empty() {
        let (sub, rest) = parse_metadata_sub_block(bytes)?;
        out.push(sub);
        bytes = rest;
    }
    Ok(out)
}

/// Parse one metadata sub-block at the start of `bytes`, returning
/// the typed [`MetadataSubBlock`] and the unconsumed tail.
///
/// Exposed alongside [`walk_metadata`] so callers can drive the walk
/// themselves (e.g. while validating against `ck_size` from the
/// fixed header).
pub fn parse_metadata_sub_block(bytes: &[u8]) -> Result<(MetadataSubBlock<'_>, &[u8])> {
    if bytes.is_empty() {
        return Err(Error::Truncated);
    }
    let id_byte = bytes[0];
    let flags = SubBlockFlags::from_id_byte(id_byte);
    let id = SubBlockId::from_id_byte(id_byte);

    // Size field: 1 byte by default, 3 bytes when the 0x80 large
    // flag is set. The wiki specifies the size unit is words
    // (16-bit) regardless of the field width.
    let (size_words, header_len) = if flags.large_size {
        if bytes.len() < 4 {
            return Err(Error::Truncated);
        }
        let b1 = bytes[1] as u32;
        let b2 = bytes[2] as u32;
        let b3 = bytes[3] as u32;
        (b1 | (b2 << 8) | (b3 << 16), 4usize)
    } else {
        if bytes.len() < 2 {
            return Err(Error::Truncated);
        }
        (bytes[1] as u32, 2usize)
    };

    // Convert words to bytes. The wiki guarantees the total block
    // length is even, so the byte count is always `2 * words` —
    // the odd-size flag merely tells the decoder that the last
    // byte of that payload is padding it should strip.
    let payload_bytes = (size_words as usize)
        .checked_mul(2)
        .ok_or(Error::MetadataSubBlockTooLarge(size_words))?;
    let total = header_len
        .checked_add(payload_bytes)
        .ok_or(Error::MetadataSubBlockTooLarge(size_words))?;
    if bytes.len() < total {
        return Err(Error::Truncated);
    }
    let raw_payload = &bytes[header_len..total];
    // Strip the trailing padding byte when the odd-size flag is set.
    let payload = if flags.odd_size {
        if raw_payload.is_empty() {
            return Err(Error::MetadataOddSizeWithoutPayload);
        }
        &raw_payload[..raw_payload.len() - 1]
    } else {
        raw_payload
    };
    Ok((
        MetadataSubBlock {
            id_byte,
            id,
            flags,
            size_words,
            payload,
        },
        &bytes[total..],
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a one-byte-size sub-block on the wire from the supplied
    /// ID byte and payload. Useful for synthesising fixtures byte by
    /// byte without needing a real `.wv` file.
    fn synth_small(id_byte: u8, payload: &[u8]) -> Vec<u8> {
        // 1-byte-size field, no large flag.
        assert!(id_byte & ID_FLAG_LARGE_SIZE == 0);
        let mut on_wire_payload = payload.to_vec();
        // If odd-size flag is set, add a single padding byte the
        // decoder will strip back off.
        if id_byte & ID_FLAG_ODD_SIZE != 0 {
            on_wire_payload.push(0);
        }
        assert!(on_wire_payload.len() % 2 == 0, "metadata is even-length");
        let words = on_wire_payload.len() / 2;
        assert!(words <= 0xFF, "use synth_large for >255-word payloads");
        let mut out = Vec::with_capacity(2 + on_wire_payload.len());
        out.push(id_byte);
        out.push(words as u8);
        out.extend_from_slice(&on_wire_payload);
        out
    }

    /// Build a three-byte-size (large-flag) sub-block on the wire.
    fn synth_large(id_byte: u8, payload: &[u8]) -> Vec<u8> {
        assert!(id_byte & ID_FLAG_LARGE_SIZE != 0);
        let mut on_wire_payload = payload.to_vec();
        if id_byte & ID_FLAG_ODD_SIZE != 0 {
            on_wire_payload.push(0);
        }
        assert!(on_wire_payload.len() % 2 == 0);
        let words = on_wire_payload.len() / 2;
        assert!(words <= 0xFF_FFFF);
        let mut out = Vec::with_capacity(4 + on_wire_payload.len());
        out.push(id_byte);
        out.push((words & 0xFF) as u8);
        out.push(((words >> 8) & 0xFF) as u8);
        out.push(((words >> 16) & 0xFF) as u8);
        out.extend_from_slice(&on_wire_payload);
        out
    }

    #[test]
    fn id_byte_decoded_into_id_and_flags() {
        // 0x02 = DecorrelationTerms, no flag bits set.
        let f = SubBlockFlags::from_id_byte(0x02);
        assert!(!f.optional);
        assert!(!f.odd_size);
        assert!(!f.large_size);
        assert_eq!(
            SubBlockId::from_id_byte(0x02),
            SubBlockId::DecorrelationTerms
        );

        // 0x20 = id 0 + 0x20 optional → RiffHeader
        let f = SubBlockFlags::from_id_byte(0x20);
        assert!(f.optional);
        assert!(!f.odd_size);
        assert!(!f.large_size);
        assert_eq!(SubBlockId::from_id_byte(0x20), SubBlockId::RiffHeader);

        // 0x66 = id 6 + 0x20 optional + 0x40 odd → Md5Checksum.
        // Bits set: 6 (0x40 odd), 5 (0x20 optional), 1 + 2 (low-id 6).
        let f = SubBlockFlags::from_id_byte(0x66);
        assert!(f.optional);
        assert!(f.odd_size);
        assert!(!f.large_size);
        assert_eq!(SubBlockId::from_id_byte(0x66), SubBlockId::Md5Checksum);

        // 0x82 = id 2 + 0x80 large, no optional/odd → DecorrelationTerms.
        // (0xC2 would be id 2 + 0x80 + 0x40 — large + odd — leaving
        // out the optional bit; we want a pure large-flag case here.)
        let f = SubBlockFlags::from_id_byte(0x82);
        assert!(!f.optional);
        assert!(!f.odd_size);
        assert!(f.large_size);
        assert_eq!(
            SubBlockId::from_id_byte(0x82),
            SubBlockId::DecorrelationTerms
        );
    }

    #[test]
    fn documented_ids_round_trip_through_as_id_byte() {
        // Every entry on the wiki "IDs" listing round-trips through
        // SubBlockId::from_id_byte / as_id_byte.
        let ids: &[(u8, SubBlockId)] = &[
            (0x00, SubBlockId::Dummy),
            (0x02, SubBlockId::DecorrelationTerms),
            (0x03, SubBlockId::DecorrelationWeights),
            (0x04, SubBlockId::DecorrelationSamples),
            (0x05, SubBlockId::EntropyInfo),
            (0x06, SubBlockId::HybridProfile),
            (0x07, SubBlockId::NoiseShapingProfile),
            (0x08, SubBlockId::FloatInfo),
            (0x09, SubBlockId::Int32Info),
            (0x0A, SubBlockId::PackedSamples),
            (0x0B, SubBlockId::PackedCorrectionData),
            (0x0C, SubBlockId::PackedOverflowBits),
            (0x0D, SubBlockId::MultichannelInfo),
            (0x20, SubBlockId::RiffHeader),
            (0x21, SubBlockId::RiffTrailer),
            (0x25, SubBlockId::EncodingDetails),
            (0x26, SubBlockId::Md5Checksum),
            (0x27, SubBlockId::NonStandardSampleRate),
        ];
        for (raw, want) in ids {
            assert_eq!(SubBlockId::from_id_byte(*raw), *want);
            assert_eq!(want.as_id_byte(), *raw);
        }
    }

    #[test]
    fn unlisted_id_numbers_fall_into_unknown() {
        // The wiki "IDs" listing skips `0x01` and `0x0E..=0x1F`
        // (and `0x22..=0x24` in the high half). The walker surfaces
        // those as `Unknown` rather than rejecting them — the
        // `0x20` "decoder may ignore" flag is the documented
        // forward-compat mechanism.
        assert_eq!(SubBlockId::from_id_byte(0x01), SubBlockId::Unknown(0x01));
        assert_eq!(SubBlockId::from_id_byte(0x0E), SubBlockId::Unknown(0x0E));
        assert_eq!(SubBlockId::from_id_byte(0x1F), SubBlockId::Unknown(0x1F));
        assert_eq!(SubBlockId::from_id_byte(0x22), SubBlockId::Unknown(0x22));
        assert_eq!(SubBlockId::from_id_byte(0x24), SubBlockId::Unknown(0x24));
        // 0x40 and 0x80 are flag bits, not part of the ID identifier —
        // an id-byte of 0x44 means id 4 + odd-size flag, not Unknown.
        assert_eq!(
            SubBlockId::from_id_byte(0x44),
            SubBlockId::DecorrelationSamples
        );
    }

    #[test]
    fn parses_small_decorrelation_terms_sub_block() {
        // Two-byte payload, no flags, even size.
        let wire = synth_small(0x02, &[0xAA, 0xBB]);
        let (sub, tail) = parse_metadata_sub_block(&wire).unwrap();
        assert_eq!(sub.id, SubBlockId::DecorrelationTerms);
        assert!(!sub.flags.optional);
        assert!(!sub.flags.odd_size);
        assert!(!sub.flags.large_size);
        assert_eq!(sub.size_words, 1);
        assert_eq!(sub.payload, &[0xAA, 0xBB]);
        assert!(tail.is_empty());
    }

    #[test]
    fn parses_md5_checksum_sub_block() {
        // The MD5 checksum is 16 bytes (8 words) — the canonical
        // even-sized small sub-block. ID byte is `0x26`.
        let md5 = [0x11u8; 16];
        let wire = synth_small(0x26, &md5);
        let (sub, tail) = parse_metadata_sub_block(&wire).unwrap();
        assert_eq!(sub.id, SubBlockId::Md5Checksum);
        assert!(sub.flags.optional);
        assert_eq!(sub.size_words, 8);
        assert_eq!(sub.payload, &md5);
        assert!(tail.is_empty());
    }

    #[test]
    fn parses_odd_size_sub_block_and_strips_padding() {
        // Three-byte payload (odd). On the wire it's padded to four
        // bytes; the walker strips the trailing pad byte.
        let id_byte = 0x05 | ID_FLAG_ODD_SIZE; // EntropyInfo + odd
        let wire = synth_small(id_byte, &[0xDE, 0xAD, 0xBE]);
        let (sub, tail) = parse_metadata_sub_block(&wire).unwrap();
        assert_eq!(sub.id, SubBlockId::EntropyInfo);
        assert!(sub.flags.odd_size);
        assert_eq!(sub.size_words, 2); // 4 bytes on wire = 2 words
        assert_eq!(sub.payload, &[0xDE, 0xAD, 0xBE]); // pad stripped
        assert!(tail.is_empty());
    }

    #[test]
    fn parses_large_sub_block_with_3_byte_size_field() {
        // A 600-byte payload exceeds the 255-word ceiling of the
        // small size field, forcing the 0x80 large flag.
        let payload = vec![0x7A; 600];
        let id_byte = 0x0A | ID_FLAG_LARGE_SIZE; // PackedSamples + large
        let wire = synth_large(id_byte, &payload);
        let (sub, tail) = parse_metadata_sub_block(&wire).unwrap();
        assert_eq!(sub.id, SubBlockId::PackedSamples);
        assert!(sub.flags.large_size);
        assert!(!sub.flags.odd_size);
        assert_eq!(sub.size_words, 300);
        assert_eq!(sub.payload, &payload[..]);
        assert!(tail.is_empty());
    }

    #[test]
    fn parses_large_odd_sub_block_strips_padding() {
        // 601-byte payload via the large size field + odd-size pad.
        let payload = vec![0x42; 601];
        let id_byte = 0x0A | ID_FLAG_LARGE_SIZE | ID_FLAG_ODD_SIZE;
        let wire = synth_large(id_byte, &payload);
        let (sub, tail) = parse_metadata_sub_block(&wire).unwrap();
        assert_eq!(sub.id, SubBlockId::PackedSamples);
        assert!(sub.flags.large_size);
        assert!(sub.flags.odd_size);
        assert_eq!(sub.size_words, 301);
        assert_eq!(sub.payload, &payload[..]);
        assert!(tail.is_empty());
    }

    #[test]
    fn walks_back_to_back_sub_blocks_until_exhausted() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x02, &[0x01, 0x02])); // terms
        stream.extend(synth_small(0x03, &[0x10, 0x20, 0x30, 0x40])); // weights
        stream.extend(synth_small(0x26, &[0xAB; 16])); // md5
        stream.extend(synth_small(0x00, &[])); // empty dummy padding

        let subs = walk_metadata(&stream).unwrap();
        assert_eq!(subs.len(), 4);
        assert_eq!(subs[0].id, SubBlockId::DecorrelationTerms);
        assert_eq!(subs[0].payload, &[0x01, 0x02]);
        assert_eq!(subs[1].id, SubBlockId::DecorrelationWeights);
        assert_eq!(subs[1].payload, &[0x10, 0x20, 0x30, 0x40]);
        assert_eq!(subs[2].id, SubBlockId::Md5Checksum);
        assert_eq!(subs[2].payload.len(), 16);
        assert_eq!(subs[3].id, SubBlockId::Dummy);
        assert!(subs[3].payload.is_empty());
    }

    #[test]
    fn riff_wrapper_and_sample_rate_sub_blocks_decode() {
        // A `0x20` RIFF header sub-block carrying a tiny fake header
        // and a `0x27` non-standard-rate sub-block carrying a four-
        // byte rate.
        let mut stream = Vec::new();
        stream.extend(synth_small(0x20, b"RIFF\x00\x00\x00\x00WAVEfmt "));
        stream.extend(synth_small(0x27, &96000u32.to_le_bytes()));
        let subs = walk_metadata(&stream).unwrap();
        assert_eq!(subs.len(), 2);
        assert_eq!(subs[0].id, SubBlockId::RiffHeader);
        assert!(subs[0].flags.optional);
        assert_eq!(subs[0].payload, b"RIFF\x00\x00\x00\x00WAVEfmt ");
        assert_eq!(subs[1].id, SubBlockId::NonStandardSampleRate);
        assert_eq!(subs[1].payload, &96000u32.to_le_bytes());
    }

    #[test]
    fn unknown_id_is_walked_not_rejected() {
        // ID byte 0x10 → low-5-bits 0x10, optional flag clear; not
        // listed in the wiki.
        let wire = synth_small(0x10, &[0xCA, 0xFE]);
        let (sub, tail) = parse_metadata_sub_block(&wire).unwrap();
        assert_eq!(sub.id, SubBlockId::Unknown(0x10));
        assert_eq!(sub.payload, &[0xCA, 0xFE]);
        assert!(tail.is_empty());
    }

    #[test]
    fn truncated_inputs_are_reported() {
        // Empty stream — first byte (id) missing.
        assert_eq!(parse_metadata_sub_block(&[]), Err(Error::Truncated));
        // ID byte alone — size byte missing.
        assert_eq!(parse_metadata_sub_block(&[0x02]), Err(Error::Truncated));
        // Large-flag ID + only one size byte — needs three.
        assert_eq!(
            parse_metadata_sub_block(&[0x82, 0x00]),
            Err(Error::Truncated)
        );
        assert_eq!(
            parse_metadata_sub_block(&[0x82, 0x00, 0x00]),
            Err(Error::Truncated)
        );
        // ID + size announce 1 word (2 bytes), only 1 byte provided.
        assert_eq!(
            parse_metadata_sub_block(&[0x02, 0x01, 0xAA]),
            Err(Error::Truncated)
        );
    }

    #[test]
    fn odd_flag_with_zero_word_payload_is_invalid() {
        // Odd-size flag requires at least one byte of payload (the
        // padding) — a zero-word odd sub-block is meaningless.
        let wire = [0x02 | ID_FLAG_ODD_SIZE, 0x00];
        assert_eq!(
            parse_metadata_sub_block(&wire),
            Err(Error::MetadataOddSizeWithoutPayload)
        );
    }
}
