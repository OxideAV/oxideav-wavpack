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
    /// `true` when this ID names one of the **decorrelation** payloads
    /// the wiki "IDs" listing places at `0x02` / `0x03` / `0x04`
    /// (terms, weights, samples). The decode layer's prediction loop
    /// consumes exactly these three sub-block kinds in lockstep, so
    /// the predicate is useful for callers picking the decorrelation
    /// triple out of a walk.
    pub fn is_decorrelation(&self) -> bool {
        matches!(
            self,
            SubBlockId::DecorrelationTerms
                | SubBlockId::DecorrelationWeights
                | SubBlockId::DecorrelationSamples
        )
    }

    /// `true` when this ID names a payload the wiki "IDs" listing
    /// annotates with "(wvc file)" — `0x07` noise-shaping profile or
    /// `0x0B` packed correction data. The lossless decoder ignores
    /// these; a hybrid decoder pairs them with the main stream.
    pub fn is_correction_stream(&self) -> bool {
        matches!(
            self,
            SubBlockId::NoiseShapingProfile | SubBlockId::PackedCorrectionData
        )
    }

    /// `true` when this ID names a **RIFF wrapper** payload (`0x20`
    /// header before audio, `0x21` trailer after audio) the wiki "IDs"
    /// listing carries verbatim from the original `.wav` framing.
    pub fn is_riff_wrapper(&self) -> bool {
        matches!(self, SubBlockId::RiffHeader | SubBlockId::RiffTrailer)
    }

    /// `true` when this ID is `0x0A` packed samples — the entropy-coded
    /// audio payload the wiki "Samples coding" section consumes.
    pub fn is_audio(&self) -> bool {
        matches!(self, SubBlockId::PackedSamples)
    }

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

impl<'a> MetadataSubBlock<'a> {
    /// `true` when the wiki "decoder may ignore data contained here"
    /// `0x20` flag is set on the ID byte. Convenience wrapper around
    /// [`SubBlockFlags::optional`] for callers that branch on the
    /// `MetadataSubBlock` value directly.
    pub fn is_optional(&self) -> bool {
        self.flags.optional
    }

    /// `true` when this sub-block carries one of the three
    /// **decorrelation** payloads enumerated by the wiki "IDs"
    /// listing — terms (`0x02`), weights (`0x03`), or samples (`0x04`).
    /// The decode layer's prediction loop consumes exactly these three
    /// sub-block kinds in lockstep, so the predicate is useful for
    /// callers picking the decorrelation triple out of a walk.
    pub fn is_decorrelation_payload(&self) -> bool {
        matches!(
            self.id,
            SubBlockId::DecorrelationTerms
                | SubBlockId::DecorrelationWeights
                | SubBlockId::DecorrelationSamples
        )
    }

    /// `true` when this sub-block belongs to the **correction stream**
    /// the wiki notes as living in the `.wvc` companion file:
    /// `0x07` noise-shaping profile or `0x0B` packed correction data.
    /// The lossless decoder ignores these; a hybrid decoder pairs them
    /// with the main stream.
    pub fn is_correction_payload(&self) -> bool {
        matches!(
            self.id,
            SubBlockId::NoiseShapingProfile | SubBlockId::PackedCorrectionData
        )
    }

    /// `true` when this sub-block carries the **packed samples** entropy
    /// stream the wiki "Samples coding" section consumes — `0x0A`.
    pub fn is_audio_payload(&self) -> bool {
        matches!(self.id, SubBlockId::PackedSamples)
    }

    /// `true` when this sub-block carries one of the **RIFF wrapper**
    /// payloads (`0x20` header, `0x21` trailer) the wiki "IDs" listing
    /// notes as the original `.wav` framing surrounding the audio.
    pub fn is_riff_payload(&self) -> bool {
        matches!(self.id, SubBlockId::RiffHeader | SubBlockId::RiffTrailer)
    }

    /// `true` when this sub-block is the wiki `0x00` "dummy (used for
    /// padding)" payload — the encoder uses it to align an even-byte
    /// boundary, the decoder discards the body.
    pub fn is_dummy_payload(&self) -> bool {
        matches!(self.id, SubBlockId::Dummy)
    }

    /// `true` when this sub-block is the wiki `0x06` "hybrid profile"
    /// payload. The lossless decoder ignores it; a hybrid decoder
    /// consumes it alongside the matching `0x07` noise-shaping data.
    pub fn is_hybrid_profile_payload(&self) -> bool {
        matches!(self.id, SubBlockId::HybridProfile)
    }

    /// `true` when this sub-block is the wiki `0x08` "floating-point
    /// data profile" payload — present when the enclosing block's
    /// [`crate::Flags::float_data`] bit is set.
    pub fn is_float_payload(&self) -> bool {
        matches!(self.id, SubBlockId::FloatInfo)
    }

    /// `true` when this sub-block is the wiki `0x09` "large or shifted
    /// integer profile" payload — present for `>24`-bit / non-byte-
    /// aligned integer streams.
    pub fn is_int32_payload(&self) -> bool {
        matches!(self.id, SubBlockId::Int32Info)
    }

    /// `true` when this sub-block is the wiki `0x0C` "packed overflow
    /// bits from floating-point or large integers" payload, paired
    /// with either [`Self::is_float_payload`] or [`Self::is_int32_payload`].
    pub fn is_overflow_bits_payload(&self) -> bool {
        matches!(self.id, SubBlockId::PackedOverflowBits)
    }

    /// `true` when this sub-block is the wiki `0x0D` "multichannel
    /// information (including Microsoft channel mask)" payload — the
    /// global channel-layout descriptor for multi-block multi-channel
    /// files.
    pub fn is_multichannel_info_payload(&self) -> bool {
        matches!(self.id, SubBlockId::MultichannelInfo)
    }

    /// `true` when this sub-block is the wiki `0x25` "some encoding
    /// details for info purposes" payload. The decoder treats it as
    /// opaque diagnostic text — it does not affect the reconstructed
    /// samples.
    pub fn is_encoding_details_payload(&self) -> bool {
        matches!(self.id, SubBlockId::EncodingDetails)
    }

    /// `true` when this sub-block is the wiki `0x26` "16-byte MD5 sum
    /// of raw audio data" payload. Pair with [`parse_md5_checksum`]
    /// to recover the typed [`Md5Checksum`] view.
    pub fn is_md5_payload(&self) -> bool {
        matches!(self.id, SubBlockId::Md5Checksum)
    }

    /// `true` when this sub-block is the wiki `0x27` "non-standard
    /// sampling rate" payload — carried when the enclosing block's
    /// [`crate::Flags::has_custom_sample_rate`] sentinel (sample-rate
    /// index `15`) is set.
    pub fn is_sample_rate_payload(&self) -> bool {
        matches!(self.id, SubBlockId::NonStandardSampleRate)
    }
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

/// On-disk byte length of an MD5 message-digest value (the size of an
/// MD5 hash; the wiki "IDs" listing names sub-block `0x26` as
/// "16-byte MD5 sum of raw audio data" — the byte count is the
/// hash's natural length and is fixed by RFC 1321, not by WavPack).
pub const MD5_DIGEST_BYTES: usize = 16;

/// Typed view of a `0x26` metadata sub-block payload — the wiki
/// "16-byte MD5 sum of raw audio data". The bytes are stored
/// verbatim as they appear on disk; comparing two `Md5Checksum`
/// values is the same as comparing the raw digests.
///
/// The MD5 covers the **raw audio data** that the WavPack stream
/// reconstructs; verifying it requires running a full sample-decode
/// pass over the stream and re-hashing the result. The walker exposes
/// only the parse + typed-view layer here; the verification pass is
/// gated on the full sample loop (which itself remains gated on the
/// median-adaptation docs gap).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Md5Checksum(pub [u8; MD5_DIGEST_BYTES]);

impl Md5Checksum {
    /// Return the digest bytes as a slice.
    pub fn as_bytes(&self) -> &[u8; MD5_DIGEST_BYTES] {
        &self.0
    }
}

/// Parse a `0x26` MD5 sub-block payload into a typed [`Md5Checksum`].
///
/// The wiki "IDs" listing fixes the on-disk length at 16 bytes (one
/// MD5 digest). Any other length is rejected as
/// [`Error::Md5ChecksumLength`].
pub fn parse_md5_checksum(payload: &[u8]) -> Result<Md5Checksum> {
    if payload.len() != MD5_DIGEST_BYTES {
        return Err(Error::Md5ChecksumLength(payload.len()));
    }
    let mut digest = [0u8; MD5_DIGEST_BYTES];
    digest.copy_from_slice(payload);
    Ok(Md5Checksum(digest))
}

/// Find the first metadata sub-block matching `id` in `subs`, returning
/// the matching [`MetadataSubBlock`] borrow or `None` if no sub-block
/// has that ID. A linear scan over the round-2 walker output that lets
/// callers pull out a specific payload (e.g. the `0x0A` audio data or
/// the `0x26` MD5 digest) without re-implementing the match arm.
///
/// Matches are by [`SubBlockId`] equality — the `0x20` optional flag
/// is part of the ID identifier, so e.g. `SubBlockId::RiffHeader`
/// matches only the `0x20`-prefixed entry and not the bare `0x00`
/// `SubBlockId::Dummy`.
pub fn find_first<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
    id: SubBlockId,
) -> Option<&'walk MetadataSubBlock<'a>> {
    subs.iter().find(|s| s.id == id)
}

/// Convenience wrapper for [`find_first`] specialised to the `0x0A`
/// packed-samples payload — the entropy-coded audio stream the wiki
/// "Samples coding" section consumes.
pub fn find_audio_payload<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::PackedSamples)
}

/// Convenience wrapper for [`find_first`] specialised to the `0x05`
/// entropy-info payload — the medians the round-4 expander consumes.
pub fn find_entropy_info<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::EntropyInfo)
}

/// Convenience wrapper for [`find_first`] specialised to the `0x26`
/// MD5 payload — the wiki "16-byte MD5 sum of raw audio data".
pub fn find_md5_checksum_block<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::Md5Checksum)
}

/// Convenience wrapper for [`find_first`] specialised to the `0x0D`
/// multichannel-info payload — the wiki "(including Microsoft channel
/// mask)" descriptor.
pub fn find_multichannel_info<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::MultichannelInfo)
}

/// Convenience wrapper for [`find_first`] specialised to the `0x27`
/// non-standard-sampling-rate payload, present when the block header's
/// bits 23..=26 rate index is the custom sentinel `15`. Pair with
/// [`parse_non_standard_sample_rate`] to obtain the rate in Hz.
pub fn find_non_standard_sample_rate<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::NonStandardSampleRate)
}

/// Parse the payload of a `0x27` non-standard-sampling-rate sub-block.
///
/// Staged spec `wavpack-sample-formats.md` §5: the payload is a
/// **3-byte little-endian** unsigned integer giving the exact sample
/// rate in Hz (`b[0] + (b[1] << 8) + (b[2] << 16)`, 24-bit range). Any
/// other payload length is malformed and rejected with
/// [`Error::SampleRatePayloadLength`].
pub fn parse_non_standard_sample_rate(payload: &[u8]) -> Result<u32> {
    let [b0, b1, b2] = payload else {
        return Err(Error::SampleRatePayloadLength(payload.len()));
    };
    Ok(u32::from(*b0) | (u32::from(*b1) << 8) | (u32::from(*b2) << 16))
}

/// Walk a metadata list and return the first `0x0A` packed-samples
/// sub-block already wrapped as a typed [`crate::PackedSamples`]
/// view — the round-12 typed counterpart to [`find_audio_payload`].
/// `None` when no `0x0A` sub-block is present in the walk.
///
/// Equivalent to `find_audio_payload(subs).map(|s|
/// PackedSamples::new(s.payload()))` but spelled directly so callers
/// staging the deferred per-sample decode loop have a one-call bridge
/// from the walker output to the [`crate::BitReader`] factory.
pub fn find_packed_samples<'a>(subs: &[MetadataSubBlock<'a>]) -> Option<crate::PackedSamples<'a>> {
    find_first(subs, SubBlockId::PackedSamples).map(|s| crate::PackedSamples::new(s.payload))
}

/// Convenience wrapper for [`find_first`] specialised to the `0x0B`
/// packed-correction-data payload — the entropy-coded correction stream
/// the wiki "IDs" listing annotates as carried in the `.wvc` companion
/// file. Returns the borrowed metadata sub-block; pair with
/// [`crate::expand_packed_correction_data`] (or the
/// [`find_packed_correction_data`] typed wrapper just below) to obtain
/// the typed [`crate::PackedCorrectionData`] view.
pub fn find_packed_correction_data_sub_block<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::PackedCorrectionData)
}

/// Walk a metadata list and return the first `0x0B` packed-correction-data
/// sub-block already wrapped as a typed [`crate::PackedCorrectionData`]
/// view — the typed counterpart to [`find_packed_correction_data_sub_block`].
/// `None` when no `0x0B` sub-block is present in the walk.
///
/// Equivalent to `find_packed_correction_data_sub_block(subs).map(|s|
/// PackedCorrectionData::new(s.payload()))` but spelled directly so
/// callers staging the deferred hybrid-mode decode have a one-call
/// bridge from the walker output to the [`crate::BitReader`] factory.
pub fn find_packed_correction_data<'a>(
    subs: &[MetadataSubBlock<'a>],
) -> Option<crate::PackedCorrectionData<'a>> {
    find_first(subs, SubBlockId::PackedCorrectionData)
        .map(|s| crate::PackedCorrectionData::new(s.payload))
}

/// Convenience wrapper for [`find_first`] specialised to the `0x07`
/// noise-shaping-profile payload — the wiki "IDs" listing annotates this
/// as carried in the `.wvc` companion file. Returns the borrowed
/// metadata sub-block; the wiki places no internal structure on the
/// payload, so the typed surface stops at the raw bytes.
pub fn find_noise_shaping_profile<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::NoiseShapingProfile)
}

/// Convenience wrapper for [`find_first`] specialised to the `0x0C`
/// packed-overflow-bits payload — the wiki "IDs" listing annotates
/// this ID as "packed overflow bits from floating-point or large
/// integers" and the clean-room entropy doc names the same ID as the
/// extension bitstream. Returns the borrowed metadata sub-block; pair
/// with [`crate::expand_packed_overflow_bits`] (or the
/// [`find_packed_overflow_bits`] typed wrapper just below) to obtain
/// the typed [`crate::PackedOverflowBits`] view.
pub fn find_packed_overflow_bits_sub_block<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::PackedOverflowBits)
}

/// Walk a metadata list and return the first `0x0C` packed-overflow-bits
/// sub-block already wrapped as a typed [`crate::PackedOverflowBits`]
/// view — the typed counterpart to [`find_packed_overflow_bits_sub_block`].
/// `None` when no `0x0C` sub-block is present in the walk.
///
/// Equivalent to `find_packed_overflow_bits_sub_block(subs).map(|s|
/// PackedOverflowBits::new(s.payload()))` but spelled directly so
/// callers staging the deferred float / large-integer container fix-up
/// have a one-call bridge from the walker output to the
/// [`crate::BitReader`] factory.
pub fn find_packed_overflow_bits<'a>(
    subs: &[MetadataSubBlock<'a>],
) -> Option<crate::PackedOverflowBits<'a>> {
    find_first(subs, SubBlockId::PackedOverflowBits)
        .map(|s| crate::PackedOverflowBits::new(s.payload))
}

/// Convenience wrapper for [`find_first`] specialised to the `0x06`
/// hybrid-profile payload — the wiki "IDs" listing names this payload
/// alongside the `0x07` noise-shaping profile as the hybrid decoder's
/// per-block configuration. Returns the borrowed metadata sub-block;
/// the wiki places no internal structure on the payload, so the typed
/// surface stops at the raw bytes.
pub fn find_hybrid_profile<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<&'walk MetadataSubBlock<'a>> {
    find_first(subs, SubBlockId::HybridProfile)
}

/// Locate the **decorrelation triple** in a metadata walk and return
/// the three sub-blocks in wiki order — `0x02` terms, `0x03` weights,
/// `0x04` samples. Returns `None` when any one of the three is missing
/// (a malformed lossless block; the decoder's prediction loop needs
/// all three to be present).
pub fn find_decorrelation_triple<'walk, 'a>(
    subs: &'walk [MetadataSubBlock<'a>],
) -> Option<(
    &'walk MetadataSubBlock<'a>,
    &'walk MetadataSubBlock<'a>,
    &'walk MetadataSubBlock<'a>,
)> {
    let terms = find_first(subs, SubBlockId::DecorrelationTerms)?;
    let weights = find_first(subs, SubBlockId::DecorrelationWeights)?;
    let samples = find_first(subs, SubBlockId::DecorrelationSamples)?;
    Some((terms, weights, samples))
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

    // ---- MetadataSubBlock kind predicates ----

    #[test]
    fn is_optional_mirrors_flag_bit() {
        // A `0x05` EntropyInfo sub-block — optional bit clear.
        let wire = synth_small(0x05, &[0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(!sub.is_optional());

        // A `0x25` EncodingDetails sub-block — optional bit set
        // (low-5 = 5, optional flag = 0x20, total = 0x25).
        let wire = synth_small(0x25, b"oxideav-");
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_optional());
    }

    #[test]
    fn is_decorrelation_payload_covers_terms_weights_samples() {
        for id in [0x02u8, 0x03, 0x04] {
            let wire = synth_small(id, &[0u8; 2]);
            let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
            assert!(
                sub.is_decorrelation_payload(),
                "id 0x{id:02x} should be a decorrelation payload"
            );
            assert!(!sub.is_audio_payload());
            assert!(!sub.is_correction_payload());
            assert!(!sub.is_riff_payload());
        }
    }

    #[test]
    fn is_audio_payload_only_for_0x0a_packed_samples() {
        let wire = synth_small(0x0A, &[0u8; 2]);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_audio_payload());
        assert!(!sub.is_decorrelation_payload());
        assert!(!sub.is_correction_payload());
        assert!(!sub.is_riff_payload());

        // Adjacent IDs do not flip the predicate.
        let wire = synth_small(0x0B, &[0u8; 2]);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(!sub.is_audio_payload());
    }

    #[test]
    fn is_correction_payload_covers_noise_shaping_and_packed_correction() {
        for id in [0x07u8, 0x0B] {
            let wire = synth_small(id, &[0u8; 2]);
            let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
            assert!(
                sub.is_correction_payload(),
                "id 0x{id:02x} should be a correction-stream payload"
            );
            assert!(!sub.is_audio_payload());
            assert!(!sub.is_decorrelation_payload());
        }
    }

    #[test]
    fn is_riff_payload_covers_0x20_and_0x21() {
        for id in [0x20u8, 0x21] {
            let wire = synth_small(id, b"RIFF");
            let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
            assert!(
                sub.is_riff_payload(),
                "id 0x{id:02x} should be a RIFF payload"
            );
            assert!(sub.is_optional()); // 0x20 bit is set
        }

        // 0x25 EncodingDetails shares the optional flag but is not RIFF.
        let wire = synth_small(0x25, b"oxideav-");
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(!sub.is_riff_payload());
    }

    // ---- SubBlockId classification helpers ----

    #[test]
    fn sub_block_id_is_decorrelation_covers_terms_weights_samples() {
        for id in [
            SubBlockId::DecorrelationTerms,
            SubBlockId::DecorrelationWeights,
            SubBlockId::DecorrelationSamples,
        ] {
            assert!(id.is_decorrelation(), "{id:?} should be a decorrelation ID");
            assert!(!id.is_correction_stream());
            assert!(!id.is_riff_wrapper());
            assert!(!id.is_audio());
        }
        // EntropyInfo (0x05) shares the lossless main-stream family but
        // isn't one of the three decorrelation IDs.
        assert!(!SubBlockId::EntropyInfo.is_decorrelation());
    }

    #[test]
    fn sub_block_id_is_correction_stream_covers_0x07_and_0x0b() {
        for id in [
            SubBlockId::NoiseShapingProfile,
            SubBlockId::PackedCorrectionData,
        ] {
            assert!(
                id.is_correction_stream(),
                "{id:?} should be correction-stream"
            );
            assert!(!id.is_audio());
            assert!(!id.is_decorrelation());
        }
        // 0x0A audio is not a correction-stream payload.
        assert!(!SubBlockId::PackedSamples.is_correction_stream());
    }

    #[test]
    fn sub_block_id_is_riff_wrapper_only_for_0x20_and_0x21() {
        assert!(SubBlockId::RiffHeader.is_riff_wrapper());
        assert!(SubBlockId::RiffTrailer.is_riff_wrapper());
        // Adjacent IDs that share the 0x20 optional flag (0x25/0x26/0x27)
        // are NOT RIFF wrappers — they carry their own payloads.
        assert!(!SubBlockId::EncodingDetails.is_riff_wrapper());
        assert!(!SubBlockId::Md5Checksum.is_riff_wrapper());
        assert!(!SubBlockId::NonStandardSampleRate.is_riff_wrapper());
        // Dummy (0x00) shares the low-5-bit value with RiffHeader (0x20)
        // but lacks the optional flag — it's not a RIFF wrapper.
        assert!(!SubBlockId::Dummy.is_riff_wrapper());
    }

    #[test]
    fn sub_block_id_is_audio_only_for_packed_samples() {
        assert!(SubBlockId::PackedSamples.is_audio());
        // Adjacent IDs are not audio.
        assert!(!SubBlockId::PackedCorrectionData.is_audio());
        assert!(!SubBlockId::PackedOverflowBits.is_audio());
        assert!(!SubBlockId::EntropyInfo.is_audio());
    }

    // ---- Additional MetadataSubBlock kind predicates ----

    /// Helper: parse a small sub-block from synthesised bytes and assert
    /// the four "main bucket" predicates (decorrelation / audio /
    /// correction / RIFF) are all false, and the MD5 predicate is also
    /// false (MD5 has its own test that uses 16-byte payloads). Returns
    /// the parsed sub-block so the caller can run its specific kind
    /// predicate.
    fn parse_non_main_bucket(id_byte: u8, payload: &[u8]) -> Vec<u8> {
        let wire = synth_small(id_byte, payload);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(
            !sub.is_decorrelation_payload(),
            "id 0x{id_byte:02x} should not be decorrelation"
        );
        assert!(
            !sub.is_audio_payload(),
            "id 0x{id_byte:02x} should not be audio"
        );
        assert!(
            !sub.is_correction_payload(),
            "id 0x{id_byte:02x} should not be correction"
        );
        assert!(
            !sub.is_riff_payload(),
            "id 0x{id_byte:02x} should not be RIFF"
        );
        if id_byte != 0x26 {
            assert!(
                !sub.is_md5_payload(),
                "id 0x{id_byte:02x} should not be MD5"
            );
        }
        wire
    }

    #[test]
    fn metadata_kind_predicates_one_hot() {
        // Every documented ID lights exactly one of the per-kind
        // predicates (except dummy + RIFF which have separate buckets).
        // We assert the predicate inline so the function-pointer table
        // doesn't have to satisfy the higher-ranked lifetime bound.
        let stub = [0u8; 2];

        let wire = parse_non_main_bucket(0x00, &[]);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_dummy_payload());

        let wire = parse_non_main_bucket(0x06, &stub);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_hybrid_profile_payload());

        let wire = parse_non_main_bucket(0x08, &stub);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_float_payload());

        let wire = parse_non_main_bucket(0x09, &stub);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_int32_payload());

        let wire = parse_non_main_bucket(0x0C, &stub);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_overflow_bits_payload());

        let wire = parse_non_main_bucket(0x0D, &stub);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_multichannel_info_payload());

        let wire = parse_non_main_bucket(0x25, &stub);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_encoding_details_payload());

        let wire = parse_non_main_bucket(0x27, &stub);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_sample_rate_payload());
    }

    #[test]
    fn is_md5_payload_only_for_0x26() {
        let md5 = [0xABu8; 16];
        let wire = synth_small(0x26, &md5);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_md5_payload());
        assert!(sub.is_optional()); // 0x20 bit set on 0x26
                                    // 0x06 has the same low-5-bit value but no optional flag, so
                                    // SubBlockId::from_id_byte resolves it to HybridProfile, not MD5.
        let wire = synth_small(0x06, &[0u8; 2]);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(!sub.is_md5_payload());
        assert!(sub.is_hybrid_profile_payload());
    }

    #[test]
    fn is_dummy_payload_only_for_0x00() {
        let wire = synth_small(0x00, &[]);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_dummy_payload());
        assert!(!sub.is_riff_payload()); // 0x20 is RiffHeader, NOT dummy.
                                         // 0x20 (low-5-bit 0 + optional flag) is RiffHeader, not Dummy.
        let wire = synth_small(0x20, b"RIFF");
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(!sub.is_dummy_payload());
        assert!(sub.is_riff_payload());
    }

    // ---- MD5 typed view ----

    #[test]
    fn parse_md5_checksum_accepts_exact_16_bytes() {
        let digest = [
            0xD4u8, 0x1D, 0x8C, 0xD9, 0x8F, 0x00, 0xB2, 0x04, 0xE9, 0x80, 0x09, 0x98, 0xEC, 0xF8,
            0x42, 0x7E,
        ]; // MD5("") for a recognisable test vector
        let md5 = parse_md5_checksum(&digest).unwrap();
        assert_eq!(md5.as_bytes(), &digest);
        assert_eq!(md5.0, digest);
    }

    #[test]
    fn parse_md5_checksum_rejects_other_lengths() {
        // Empty payload.
        assert_eq!(parse_md5_checksum(&[]), Err(Error::Md5ChecksumLength(0)));
        // Too short.
        assert_eq!(
            parse_md5_checksum(&[0u8; 15]),
            Err(Error::Md5ChecksumLength(15))
        );
        // Too long (one byte past).
        assert_eq!(
            parse_md5_checksum(&[0u8; 17]),
            Err(Error::Md5ChecksumLength(17))
        );
        // Way too long.
        assert_eq!(
            parse_md5_checksum(&[0u8; 64]),
            Err(Error::Md5ChecksumLength(64))
        );
    }

    #[test]
    fn md5_checksum_round_trips_through_sub_block_payload() {
        // End-to-end: synthesise a 0x26 sub-block, walk it, and pull
        // the MD5 back out through parse_md5_checksum.
        let digest = [
            0x9Eu8, 0x10, 0x7D, 0x9D, 0x37, 0x2B, 0xB6, 0x82, 0x6B, 0xD8, 0x1D, 0x35, 0x42, 0xA4,
            0x19, 0xD6,
        ]; // MD5("The quick brown fox jumps over the lazy dog")
        let wire = synth_small(0x26, &digest);
        let (sub, _) = parse_metadata_sub_block(&wire).unwrap();
        assert!(sub.is_md5_payload());
        let md5 = parse_md5_checksum(sub.payload).unwrap();
        assert_eq!(md5.as_bytes(), &digest);
    }

    // ---- 0x27 non-standard sample rate (round 405) ----

    #[test]
    fn parse_non_standard_sample_rate_reads_three_le_bytes() {
        // Staged spec wavpack-sample-formats.md §5:
        // rate = b[0] + (b[1] << 8) + (b[2] << 16).
        assert_eq!(
            parse_non_standard_sample_rate(&[0x39, 0x30, 0x00]),
            Ok(12345)
        );
        assert_eq!(parse_non_standard_sample_rate(&[0x00, 0x00, 0x00]), Ok(0));
        assert_eq!(
            parse_non_standard_sample_rate(&[0xFF, 0xFF, 0xFF]),
            Ok(16_777_215)
        );
    }

    #[test]
    fn parse_non_standard_sample_rate_rejects_wrong_lengths() {
        for n in [0usize, 1, 2, 4, 5, 8] {
            let payload = vec![0u8; n];
            assert_eq!(
                parse_non_standard_sample_rate(&payload),
                Err(Error::SampleRatePayloadLength(n)),
                "len {n}"
            );
        }
    }

    #[test]
    fn find_non_standard_sample_rate_locates_0x27() {
        let mut stream = synth_small(0x05, &[0u8; 6]);
        // 3-byte payload -> odd-size flag + one wire pad byte.
        stream.extend(synth_small(0x27 | ID_FLAG_ODD_SIZE, &[0x39, 0x30, 0x00]));
        let subs = walk_metadata(&stream).unwrap();
        let sub = find_non_standard_sample_rate(&subs).expect("0x27 present");
        assert!(sub.is_sample_rate_payload());
        assert_eq!(parse_non_standard_sample_rate(sub.payload), Ok(12345));
        // Absent -> None.
        let bare = synth_small(0x05, &[0u8; 6]);
        let subs = walk_metadata(&bare).unwrap();
        assert!(find_non_standard_sample_rate(&subs).is_none());
    }

    // ---- Walker convenience finders ----

    fn synth_full_stream() -> Vec<u8> {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x02, &[0x01, 0x02])); // terms
        stream.extend(synth_small(0x03, &[0x10, 0x20, 0x30, 0x40])); // weights
        stream.extend(synth_small(0x04, &[0u8; 4])); // samples (2 words)
        stream.extend(synth_small(0x05, &[0u8; 6])); // mono entropy
        stream.extend(synth_small(0x0A, &[0xAA, 0xBB, 0xCC, 0xDD])); // audio
        stream.extend(synth_small(0x0D, &[0x01, 0x02, 0x03, 0x04])); // multichannel info
        stream.extend(synth_small(0x26, &[0x42u8; 16])); // md5
        stream
    }

    #[test]
    fn find_first_returns_matching_sub_block() {
        let stream = synth_full_stream();
        let subs = walk_metadata(&stream).unwrap();

        let entropy = find_first(&subs, SubBlockId::EntropyInfo);
        assert!(entropy.is_some());
        assert_eq!(entropy.unwrap().id, SubBlockId::EntropyInfo);

        // Unrelated ID returns None.
        let missing = find_first(&subs, SubBlockId::HybridProfile);
        assert!(missing.is_none());
    }

    #[test]
    fn find_audio_payload_returns_packed_samples_block() {
        let stream = synth_full_stream();
        let subs = walk_metadata(&stream).unwrap();
        let audio = find_audio_payload(&subs).expect("audio block present");
        assert_eq!(audio.id, SubBlockId::PackedSamples);
        assert_eq!(audio.payload, &[0xAA, 0xBB, 0xCC, 0xDD]);

        // Walk a stream without 0x0A — finder returns None.
        let mut without_audio = Vec::new();
        without_audio.extend(synth_small(0x02, &[0x01, 0x02]));
        without_audio.extend(synth_small(0x05, &[0u8; 6]));
        let subs = walk_metadata(&without_audio).unwrap();
        assert!(find_audio_payload(&subs).is_none());
    }

    #[test]
    fn find_entropy_info_returns_0x05_block() {
        let stream = synth_full_stream();
        let subs = walk_metadata(&stream).unwrap();
        let entropy = find_entropy_info(&subs).expect("entropy block present");
        assert_eq!(entropy.id, SubBlockId::EntropyInfo);
        assert_eq!(entropy.payload.len(), 6);
    }

    #[test]
    fn find_md5_checksum_block_returns_0x26_block() {
        let stream = synth_full_stream();
        let subs = walk_metadata(&stream).unwrap();
        let md5_block = find_md5_checksum_block(&subs).expect("md5 block present");
        assert_eq!(md5_block.id, SubBlockId::Md5Checksum);
        let md5 = parse_md5_checksum(md5_block.payload).unwrap();
        assert_eq!(md5.0, [0x42u8; 16]);
    }

    #[test]
    fn find_multichannel_info_returns_0x0d_block() {
        let stream = synth_full_stream();
        let subs = walk_metadata(&stream).unwrap();
        let info = find_multichannel_info(&subs).expect("multichannel-info present");
        assert_eq!(info.id, SubBlockId::MultichannelInfo);
        assert_eq!(info.payload, &[0x01, 0x02, 0x03, 0x04]);
    }

    #[test]
    fn find_decorrelation_triple_returns_terms_weights_samples_in_order() {
        let stream = synth_full_stream();
        let subs = walk_metadata(&stream).unwrap();
        let (terms, weights, samples) = find_decorrelation_triple(&subs).expect("triple present");
        assert_eq!(terms.id, SubBlockId::DecorrelationTerms);
        assert_eq!(weights.id, SubBlockId::DecorrelationWeights);
        assert_eq!(samples.id, SubBlockId::DecorrelationSamples);
        assert_eq!(terms.payload, &[0x01, 0x02]);
        assert_eq!(weights.payload, &[0x10, 0x20, 0x30, 0x40]);
        assert_eq!(samples.payload.len(), 4);
    }

    #[test]
    fn find_decorrelation_triple_returns_none_when_any_id_is_missing() {
        // Drop the weights (0x03) sub-block — triple finder should fail.
        let mut stream = Vec::new();
        stream.extend(synth_small(0x02, &[0x01, 0x02]));
        // (no 0x03)
        stream.extend(synth_small(0x04, &[0u8; 4]));
        let subs = walk_metadata(&stream).unwrap();
        assert!(find_decorrelation_triple(&subs).is_none());

        // Conversely, drop the samples — same failure.
        let mut stream = Vec::new();
        stream.extend(synth_small(0x02, &[0x01, 0x02]));
        stream.extend(synth_small(0x03, &[0x10, 0x20, 0x30, 0x40]));
        // (no 0x04)
        let subs = walk_metadata(&stream).unwrap();
        assert!(find_decorrelation_triple(&subs).is_none());
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

    // ---- Round-12 find_packed_samples typed-view finder ----

    #[test]
    fn find_packed_samples_returns_typed_view_over_audio_payload() {
        let stream = synth_full_stream();
        let subs = walk_metadata(&stream).unwrap();
        let ps = find_packed_samples(&subs).expect("packed samples present");
        // The fixture's 0x0A payload from synth_full_stream is the
        // four-byte 0xAA/0xBB/0xCC/0xDD sequence.
        assert_eq!(ps.bytes(), &[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(ps.len(), 4);
        assert!(!ps.is_empty());
    }

    #[test]
    fn find_packed_samples_returns_none_when_no_audio_block() {
        // A stream without a 0x0A sub-block — only entropy info (0x05).
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        let subs = walk_metadata(&stream).unwrap();
        assert!(find_packed_samples(&subs).is_none());
    }

    // ---- Round-233 .wvc-side finders ----

    #[test]
    fn find_packed_correction_data_typed_view_returns_view_when_0x0b_present() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        stream.extend(synth_small(0x0A, &[0x00, 0x00]));
        stream.extend(synth_small(0x0B, &[0xAA, 0xBB, 0xCC, 0xDD]));
        let subs = walk_metadata(&stream).unwrap();
        let view = find_packed_correction_data(&subs).expect("0x0B present");
        assert_eq!(view.bytes(), &[0xAA, 0xBB, 0xCC, 0xDD]);
        assert_eq!(view.len(), 4);
    }

    #[test]
    fn find_packed_correction_data_typed_view_returns_none_when_0x0b_absent() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        stream.extend(synth_small(0x0A, &[0x00, 0x00]));
        let subs = walk_metadata(&stream).unwrap();
        assert!(find_packed_correction_data(&subs).is_none());
    }

    #[test]
    fn find_packed_correction_data_sub_block_returns_metadata_borrow() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        stream.extend(synth_small(0x0B, &[0x11, 0x22]));
        let subs = walk_metadata(&stream).unwrap();
        let sub = find_packed_correction_data_sub_block(&subs).expect("0x0B");
        assert_eq!(sub.id, SubBlockId::PackedCorrectionData);
        assert_eq!(sub.payload, &[0x11, 0x22]);
    }

    #[test]
    fn find_noise_shaping_profile_returns_metadata_borrow_when_0x07_present() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        stream.extend(synth_small(0x07, &[0x33, 0x44]));
        let subs = walk_metadata(&stream).unwrap();
        let sub = find_noise_shaping_profile(&subs).expect("0x07");
        assert_eq!(sub.id, SubBlockId::NoiseShapingProfile);
        assert_eq!(sub.payload, &[0x33, 0x44]);
    }

    #[test]
    fn find_noise_shaping_profile_returns_none_when_0x07_absent() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        let subs = walk_metadata(&stream).unwrap();
        assert!(find_noise_shaping_profile(&subs).is_none());
    }

    #[test]
    fn find_hybrid_profile_returns_metadata_borrow_when_0x06_present() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        stream.extend(synth_small(0x06, &[0x55, 0x66]));
        let subs = walk_metadata(&stream).unwrap();
        let sub = find_hybrid_profile(&subs).expect("0x06");
        assert_eq!(sub.id, SubBlockId::HybridProfile);
        assert_eq!(sub.payload, &[0x55, 0x66]);
    }

    #[test]
    fn find_hybrid_profile_returns_none_when_0x06_absent() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        let subs = walk_metadata(&stream).unwrap();
        assert!(find_hybrid_profile(&subs).is_none());
    }

    #[test]
    fn find_packed_overflow_bits_typed_view_returns_view_when_0x0c_present() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        stream.extend(synth_small(0x0A, &[0x00, 0x00]));
        stream.extend(synth_small(0x0C, &[0x77, 0x88, 0x99, 0xAA]));
        let subs = walk_metadata(&stream).unwrap();
        let view = find_packed_overflow_bits(&subs).expect("0x0C present");
        assert_eq!(view.bytes(), &[0x77, 0x88, 0x99, 0xAA]);
        assert_eq!(view.len(), 4);
    }

    #[test]
    fn find_packed_overflow_bits_typed_view_returns_none_when_0x0c_absent() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        stream.extend(synth_small(0x0A, &[0x00, 0x00]));
        let subs = walk_metadata(&stream).unwrap();
        assert!(find_packed_overflow_bits(&subs).is_none());
    }

    #[test]
    fn find_packed_overflow_bits_sub_block_returns_metadata_borrow() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        stream.extend(synth_small(0x0C, &[0x33, 0x44]));
        let subs = walk_metadata(&stream).unwrap();
        let sub = find_packed_overflow_bits_sub_block(&subs).expect("0x0C");
        assert_eq!(sub.id, SubBlockId::PackedOverflowBits);
        assert_eq!(sub.payload, &[0x33, 0x44]);
    }

    #[test]
    fn find_packed_overflow_bits_sub_block_returns_none_when_0x0c_absent() {
        let mut stream = Vec::new();
        stream.extend(synth_small(0x05, &[0u8; 6]));
        let subs = walk_metadata(&stream).unwrap();
        assert!(find_packed_overflow_bits_sub_block(&subs).is_none());
    }
}
