//! Forward (encode) block assembly — turn a PCM buffer into a complete,
//! self-describing `wvpk` block the decoder reads back bit-exactly.
//!
//! The decode side ([`crate::block::WavPackBlock::decode_samples`] and the
//! stream walkers) consumes a 32-byte fixed header followed by a chain of
//! metadata sub-blocks; this module is its forward inverse. It composes
//! the existing leaf encoders — the spec §4.2 modified-Rice
//! [`crate::samples::encode_packed_samples_mono`] /
//! [`crate::samples::encode_packed_samples_stereo`] entropy writer and the
//! spec §3.7 [`crate::decorrelation::recorrelate_mono`] /
//! [`crate::decorrelation::recorrelate_stereo`] forward-prediction loop —
//! and frames their output into the wire byte layout:
//!
//! 1. `0x05` entropy-info sub-block (the three median seeds per channel,
//!    log-packed per the wiki "Entropy info" section).
//! 2. The `0x02`/`0x03`/`0x04` decorrelation metadata for the
//!    `*_with_decorr` entry points (emitted verbatim from the caller's
//!    payloads).
//! 3. `0x0A` packed-samples sub-block (the entropy-coded residuals).
//! 4. The 32-byte fixed header with the spec §5 running CRC folded over
//!    the PCM and the [`crate::block_header::Flags`] word reconstructed
//!    from the block shape.
//!
//! ## Lossless round-trip
//!
//! The headline guarantee is `decode_stream(&encode_block_mono(pcm,
//! …)?)? == pcm` (and the stereo twin): an encoded block parses, passes
//! its CRC gate, and reconstructs the exact input PCM. The encode surface
//! covers, all bit-exactly lossless:
//!
//! * raw (no-decorrelation) mono / stereo blocks
//!   ([`encode_block_mono`] / [`encode_block_stereo`]);
//! * decorrelated blocks driven by their raw `0x02`/`0x03`/`0x04`
//!   payloads ([`encode_block_mono_with_decorr`] /
//!   [`encode_block_stereo_with_decorr`]);
//! * joint (mid/side) stereo ([`encode_block_stereo_joint`]);
//! * sub-byte bit-depth via the left-shift fixup
//!   ([`encode_block_mono_shifted`] / [`encode_block_stereo_shifted`]);
//! * multi-block `.wv` streams ([`encode_stream_mono`] /
//!   [`encode_stream_stereo`]).
//!
//! ## Scope
//!
//! This is the lossless integer encoder. Hybrid (`0x0B`/shaping), float
//! (`0x08`/`0x0C`), int32 container mode, and multichannel (`0x0D` stream
//! chaining) block emission remain out of scope — the decoder refuses
//! them and their wire layout is a documented spec gap.

use crate::block_header::{MAGIC, MIN_CK_SIZE};
use crate::decorrelation::{
    quantize_weight, recorrelate_mono, recorrelate_stereo, serialize_mono_passes,
    serialize_stereo_passes, DecorrPass, MAX_TERM,
};
use crate::error::{Error, Result};
use crate::metadata::{SubBlockId, ID_FLAG_LARGE_SIZE, ID_FLAG_ODD_SIZE};
use crate::samples::{encode_packed_samples_mono, encode_packed_samples_stereo, AdaptiveMedians};

/// The stream-format version this encoder writes into the 16-bit header
/// `version` field. `0x0410` is the highest version
/// [`crate::block_header::parse_block_header`] accepts
/// ([`crate::block_header::MAX_VERSION`]) and the one the wiki "Block
/// structure" listing notes false-stereo (bit 30) requires.
pub const ENCODE_VERSION: u16 = 0x0410;

/// The two-bit multichannel start/end marker value (header flag bits
/// 11..=12) that marks a block as **not** part of a multichannel set —
/// both the "first" and "last" markers set, which the decoder's
/// [`crate::block_header::Flags::is_multichannel_member`] reads as a
/// standalone (single-block-per-frame) block. A genuine multichannel
/// member clears one of these bits; this encoder never emits a
/// multichannel set, so every block it writes is self-contained.
const STANDALONE_MULTICHANNEL_MARKER: u32 = 0b11 << 11;

/// On-wire byte length of one log-packed 16-bit median / seed word
/// (mantissa byte + exponent byte).
const MEDIAN_WORD_BYTES: usize = 2;

/// Bias subtracted from the stored exponent byte by the decoder's
/// log-pack expander ([`crate::decorrelation`] `expand_sample_word`):
/// an exponent byte of `9` means "shift 0", i.e. the mantissa byte is
/// the value verbatim. Mirrored here so the seed packer is the forward
/// inverse for the zero-seed case this encoder writes.
const EXPONENT_BIAS_BYTE: u8 = 9;

/// Append a metadata sub-block (`id` byte, word-count size field,
/// payload, trailing odd-size pad) to `out`, mirroring the byte layout
/// [`crate::metadata::parse_metadata_sub_block`] reads back.
///
/// The wiki "Metadata" section fixes the framing: an ID byte, then the
/// size **in 16-bit words** as either a 1-byte field (default) or a
/// 3-byte little-endian field when the `0x80` large-size flag is set,
/// then the payload. The total sub-block length is always even, so an
/// odd payload is padded with one trailing zero byte and the `0x40`
/// odd-size flag is OR'd into the ID byte (the decoder strips that pad).
///
/// Returns [`Error::EncodeBlockTooLarge`] when the payload's word count
/// overflows the 24-bit large-size field — far beyond any real block.
fn append_sub_block(out: &mut Vec<u8>, id: u8, payload: &[u8]) -> Result<()> {
    let odd = payload.len() % 2 == 1;
    // Word count rounds the (possibly padded) byte length up to a whole
    // number of 16-bit words.
    let padded_len = payload.len() + usize::from(odd);
    let words = padded_len / 2;

    let mut id_byte = id;
    if odd {
        id_byte |= ID_FLAG_ODD_SIZE;
    }

    if words <= 0xFF {
        out.push(id_byte);
        out.push(words as u8);
    } else if words <= 0x00FF_FFFF {
        out.push(id_byte | ID_FLAG_LARGE_SIZE);
        out.push((words & 0xFF) as u8);
        out.push(((words >> 8) & 0xFF) as u8);
        out.push(((words >> 16) & 0xFF) as u8);
    } else {
        return Err(Error::EncodeBlockTooLarge(padded_len));
    }

    out.extend_from_slice(payload);
    if odd {
        out.push(0);
    }
    Ok(())
}

/// Log-pack one median / seed value into its 2-byte wire word for the
/// zero-or-small-seed case this encoder uses.
///
/// The decoder expands a word `(mantissa, exponent)` to
/// `mantissa << (exponent - 9)` (signed mantissa). For a value in the
/// signed-8-bit range we can store it verbatim with exponent byte `9`
/// (shift 0), so the round trip is exact. This encoder only ever needs
/// the zero seed (`[0, 0, 0]`), which packs to `[0x00, 0x09]`, but the
/// small-value path is kept so future seeds in `-128..=127` round-trip.
fn pack_median_word(value: i32) -> Option<[u8; MEDIAN_WORD_BYTES]> {
    if (i8::MIN as i32..=i8::MAX as i32).contains(&value) {
        Some([(value as i8) as u8, EXPONENT_BIAS_BYTE])
    } else {
        None
    }
}

/// Serialize a per-channel median seed set into the `0x05` entropy-info
/// payload bytes — `6` bytes for one set (mono), `12` for two (stereo).
///
/// `seeds` carries one `[m0, m1, m2]` set per channel in left-then-right
/// wire order (the order [`crate::entropy::expand_entropy`] reads).
/// Every seed must be in the signed-8-bit range so [`pack_median_word`]
/// can represent it exactly; this encoder always passes the zero seed.
fn pack_entropy_info(seeds: &[[i32; 3]]) -> Vec<u8> {
    let mut out = Vec::with_capacity(seeds.len() * 3 * MEDIAN_WORD_BYTES);
    for set in seeds {
        for &m in set {
            // The zero seed (and any value in -128..=127) is always
            // representable; the caller only ever passes the zero seed,
            // so the `unwrap_or` default is unreachable in practice but
            // keeps the helper total.
            let word = pack_median_word(m).unwrap_or([0, EXPONENT_BIAS_BYTE]);
            out.extend_from_slice(&word);
        }
    }
    out
}

/// Write the 32-byte fixed block header into the first 32 bytes of a
/// freshly-built buffer, prepended to the already-assembled metadata
/// region `metadata`.
///
/// `block_samples` is the per-channel sample count (mono / false-stereo:
/// the buffer length; stereo: the pair count). `flags` is the decoded
/// flag view re-packed into its 32-bit word via
/// [`crate::block_header::Flags::raw`]. `crc` is the spec §5 running CRC
/// already folded over the PCM. The header's `ck_size` is
/// `24 + metadata.len()` (the wiki "total block size not counting this
/// field or 'wvpk'": the 24 fixed bytes after `ck_size` plus the
/// metadata region).
fn build_block(
    metadata: Vec<u8>,
    block_index: u32,
    total_samples: u32,
    block_samples: u32,
    flags_raw: u32,
    crc: u32,
) -> Result<Vec<u8>> {
    let ck_size = MIN_CK_SIZE
        .checked_add(
            u32::try_from(metadata.len())
                .map_err(|_| Error::EncodeBlockTooLarge(metadata.len()))?,
        )
        .ok_or(Error::EncodeBlockTooLarge(metadata.len()))?;

    let mut out = Vec::with_capacity(crate::block_header::HEADER_LEN + metadata.len());
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&ck_size.to_le_bytes());
    out.extend_from_slice(&ENCODE_VERSION.to_le_bytes());
    out.push(0); // track_number — wiki "not currently implemented"
    out.push(0); // track_sub_index — wiki "not currently implemented"
    out.extend_from_slice(&total_samples.to_le_bytes());
    out.extend_from_slice(&block_index.to_le_bytes());
    out.extend_from_slice(&block_samples.to_le_bytes());
    out.extend_from_slice(&flags_raw.to_le_bytes());
    out.extend_from_slice(&crc.to_le_bytes());
    out.extend_from_slice(&metadata);
    Ok(out)
}

/// The base flag word shared by every block this encoder writes:
/// bytes-per-sample (bits 0..=1), the standalone multichannel marker
/// (bits 11..=12 both set so the decoder does not treat the block as a
/// multichannel member), and the version-dependent bits left clear.
///
/// `bytes_per_sample` is clamped to `1..=4` and stored minus one in
/// bits 0..=1 per the wiki "bytes per sample minus one" entry.
fn base_flags(bytes_per_sample: u8) -> u32 {
    let bps = bytes_per_sample.clamp(1, 4);
    u32::from(bps - 1) | STANDALONE_MULTICHANNEL_MARKER
}

/// Replace the wiki bits-11..=12 multichannel grouping marker in a flag
/// word. `marker` is the 2-bit grouping value (`0b11` standalone, `0b01`
/// first-of-set, `0b00` continuation, `0b10` final-of-set). Used by the
/// multichannel-member encoders to override the default standalone marker
/// [`base_flags`] sets. Round 378.
fn with_marker(flags: u32, marker: u32) -> u32 {
    (flags & !(0b11 << 11)) | ((marker & 0b11) << 11)
}

/// Forward mid/side joint-stereo transform — the exact inverse of the
/// decoder's spec §5.4 [`crate::crc::undo_joint_stereo`].
///
/// The decoder recovers `(left, right)` from a stored `(mid, side)` pair
/// via `right = side - (mid >> 1); left = mid + right`. Inverting that
/// for the encoder:
///
/// ```text
/// mid  = left - right
/// side = right + (mid >> 1)
/// ```
///
/// The `mid >> 1` term the encoder adds is the *same* value the decoder
/// subtracts (both derive `mid` identically), so the arithmetic-shift
/// truncation cancels exactly and the transform is bit-reversible for
/// every `(left, right)` pair — `undo_joint_stereo(forward_joint_stereo(l,
/// r)) == (l, r)`.
fn forward_joint_stereo(left: i32, right: i32) -> (i32, i32) {
    let mid = left.wrapping_sub(right);
    let side = right.wrapping_add(mid >> 1);
    (mid, side)
}

/// Shape / feature selection for one block encode, consumed by
/// [`encode_block_core`]. Every public block encoder is a thin wrapper
/// choosing a combination; the core runs the stages in the exact inverse
/// of the decoder's documented order (entropy → decorrelate → joint undo
/// → CRC → final shift), i.e. narrow → CRC → joint forward → recorrelate
/// → entropy.
struct BlockConfig<'a> {
    /// Mono / false-stereo shape (bit 2) vs interleaved stereo.
    mono: bool,
    /// Joint (mid/side) stereo (bit 4); stereo only.
    joint: bool,
    /// Wiki flag-bits-13..=17 sub-byte-depth shift (`0` = whole-byte).
    left_shift: u8,
    /// Raw `0x02`/`0x03`/`0x04` decorrelation payloads to emit verbatim
    /// and drive the §3 forward prediction loop with; `None` = raw path.
    decorr: Option<(&'a [u8], &'a [u8], &'a [u8])>,
    /// Header bits 0..=1 width hint (1..=4).
    bytes_per_sample: u8,
    /// Wiki bits-11..=12 multichannel grouping marker (`0b11` standalone).
    marker: u32,
}

/// Assemble one complete `wvpk` block from a container-scaled PCM buffer
/// and a [`BlockConfig`] — the single body every public block encoder
/// delegates to.
///
/// Stage order (each the forward inverse of the decoder's documented
/// stage, in reverse):
///
/// 1. **Narrow** (`left_shift > 0`): right-shift each sample (inverse of
///    the §1 pipeline final [`crate::fixup::apply_left_shift_buffer`]),
///    refusing non-zero dropped bits.
/// 2. **CRC**: fold the spec §5 running CRC over the narrow true
///    (pre-joint) samples — the exact buffer the decoder folds.
/// 3. **Joint forward** (`joint`): the §5.4 inverse per pair.
/// 4. **Recorrelate** (`decorr`): assemble the pass list from the raw
///    payloads and run the §3 forward prediction loop into residuals.
/// 5. **Entropy**: the §4.2 writer, framed as the `0x05` + (`0x02` /
///    `0x03` / `0x04`) + `0x0A` metadata chain behind the 32-byte fixed
///    header.
fn encode_block_core(
    pcm: &[i32],
    config: BlockConfig<'_>,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if !config.mono && pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }

    // Stage 1: narrow container-scaled samples for a sub-byte-depth
    // block; the identity when left_shift == 0.
    let mut work = pcm.to_vec();
    if config.left_shift > 0 {
        narrow_left_shift(&mut work, config.left_shift)?;
    }

    // Stage 2: the §5 running CRC over the narrow true samples (the
    // decoder undoes joint stereo before its CRC step, and folds the
    // pre-shift buffer).
    let crc = if config.mono {
        crate::crc::crc_mono(&work)
    } else {
        crate::crc::crc_stereo_interleaved(&work)
    };

    // Stage 3: forward mid/side ahead of prediction, mirroring the
    // decoder's decorrelate-then-joint-undo order.
    if config.joint {
        for pair in work.chunks_exact_mut(2) {
            let (mid, side) = forward_joint_stereo(pair[0], pair[1]);
            pair[0] = mid;
            pair[1] = side;
        }
    }

    // Stage 4: §3 forward prediction into residuals (validating the
    // payloads exactly as the decoder's assembler will).
    if let Some((terms, weights, samples)) = config.decorr {
        if config.mono {
            let mut passes = crate::decorrelation::assemble_mono_passes(terms, weights, samples)?;
            recorrelate_mono(&mut passes, &mut work)?;
        } else {
            let mut passes = crate::decorrelation::assemble_stereo_passes(terms, weights, samples)?;
            recorrelate_stereo(&mut passes, &mut work)?;
        }
    }

    // Flag word: width bits, grouping marker, then the shape bits.
    let mut flags_raw = with_marker(base_flags(config.bytes_per_sample), config.marker);
    if config.mono {
        flags_raw |= 1 << 2;
    }
    if config.joint {
        flags_raw |= crate::crc::JOINT_STEREO_FLAG;
    }
    if config.left_shift > 0 {
        flags_raw = with_left_shift(flags_raw, config.left_shift);
    }

    let mut metadata = Vec::new();

    // 0x05 entropy info. Mono: a single zero-seed set. Stereo: two sets,
    // the right one carrying a minimal non-zero seed ([0, 0, 1]) because
    // the decoder distinguishes a stereo entropy payload from a mono one
    // by content (an all-zero right set reads as mono and is refused on
    // the stereo path); the encode medians are seeded from the exact
    // same sets so the round trip stays bit-exact whatever the seed.
    let left_seed = [0, 0, 0];
    let right_seed = [0, 0, 1];
    let entropy_payload = if config.mono {
        pack_entropy_info(&[left_seed])
    } else {
        pack_entropy_info(&[left_seed, right_seed])
    };
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;

    // The three decorrelation sub-blocks, verbatim, in wire order.
    if let Some((terms, weights, samples)) = config.decorr {
        append_sub_block(
            &mut metadata,
            SubBlockId::DecorrelationTerms.as_id_byte(),
            terms,
        )?;
        append_sub_block(
            &mut metadata,
            SubBlockId::DecorrelationWeights.as_id_byte(),
            weights,
        )?;
        append_sub_block(
            &mut metadata,
            SubBlockId::DecorrelationSamples.as_id_byte(),
            samples,
        )?;
    }

    // Stage 5: the §4.2 entropy writer over the residuals.
    let packed = if config.mono {
        let mut medians = AdaptiveMedians::new([0, 0, 0]);
        encode_packed_samples_mono(&work, &mut medians)?
    } else {
        let mut medians = [
            AdaptiveMedians::from_seed_values(left_seed)
                .ok_or(Error::InvalidEntropyInfoForStereo)?,
            AdaptiveMedians::from_seed_values(right_seed)
                .ok_or(Error::InvalidEntropyInfoForStereo)?,
        ];
        encode_packed_samples_stereo(&work, &mut medians)?
    };
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &packed,
    )?;

    let per_channel = if config.mono {
        pcm.len()
    } else {
        pcm.len() / 2
    };
    let block_samples =
        u32::try_from(per_channel).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        crc,
    )
}

/// The default standalone [`BlockConfig`]: raw (no decorrelation),
/// whole-byte, non-joint, standalone marker.
fn raw_config(mono: bool, bytes_per_sample: u8) -> BlockConfig<'static> {
    BlockConfig {
        mono,
        joint: false,
        left_shift: 0,
        decorr: None,
        bytes_per_sample,
        marker: 0b11,
    }
}

/// Encode a mono (single-channel) PCM buffer into one complete `wvpk`
/// block via the raw (no-decorrelation) lossless path — the entropy
/// stream carries the PCM verbatim and the decoder's no-decorrelation
/// branch returns it unchanged.
///
/// `pcm` is the channel's samples. `bytes_per_sample` (1..=4) sets the
/// header bits 0..=1 width hint; `block_index` / `total_samples` populate
/// the stream-position header fields (use
/// [`crate::block_header::TOTAL_SAMPLES_UNKNOWN`] for a streaming total).
///
/// For a decorrelated block use [`encode_block_mono_with_decorr`]; for
/// sub-byte depth use [`encode_block_mono_shifted`].
///
/// The returned bytes decode back to `pcm` exactly:
/// `decode_stream(&out)? == pcm`.
pub fn encode_block_mono(
    pcm: &[i32],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    // The standalone marker (0b11) is the default — a self-contained
    // single-block file. Multichannel members reuse the body below with a
    // grouping marker via `encode_block_mono_marker`.
    encode_block_mono_marker(pcm, bytes_per_sample, block_index, total_samples, 0b11)
}

/// Encode a mono PCM buffer into one `wvpk` block carrying the supplied
/// wiki bits-11..=12 multichannel grouping `marker` (2 bits: `0b11`
/// standalone, `0b01` first-of-set, `0b00` continuation, `0b10`
/// final-of-set).
///
/// This is the marker-aware core of [`encode_block_mono`]; the public
/// raw mono encoder passes `0b11` (standalone) and the multichannel
/// encoder [`encode_multichannel_stream`] passes the per-member grouping
/// markers. The marker bits sit outside the §5 sample CRC, so a member
/// block stays CRC-valid for whatever marker is chosen. Round 378.
fn encode_block_mono_marker(
    pcm: &[i32],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
    marker: u32,
) -> Result<Vec<u8>> {
    let config = BlockConfig {
        marker,
        ..raw_config(true, bytes_per_sample)
    };
    encode_block_core(pcm, config, block_index, total_samples)
}

/// Right-shift a container-scaled PCM buffer by `left_shift` in place,
/// recovering the narrow sample values the decoder's prediction loop
/// reconstructs (the buffer the §5 CRC is folded over and the entropy
/// stream carries). The exact inverse of the decoder's §1 pipeline final
/// stage [`crate::fixup::apply_left_shift_buffer`].
///
/// Every sample must already be a multiple of `2^left_shift` (its low
/// `left_shift` bits zero) — the decode reconstructs the container value
/// as `narrow << left_shift`, so a non-zero low bit would be lost.
/// [`Error::EncodeLeftShiftLosesData`] names the first offending sample.
fn narrow_left_shift(buffer: &mut [i32], left_shift: u8) -> Result<()> {
    let mask: i32 = (1i32 << left_shift) - 1;
    for s in buffer.iter_mut() {
        if *s & mask != 0 {
            return Err(Error::EncodeLeftShiftLosesData(*s));
        }
        *s >>= left_shift;
    }
    Ok(())
}

/// Set the wiki flag-bits-13..=17 `left_shift` field in a flag word.
fn with_left_shift(flags_raw: u32, left_shift: u8) -> u32 {
    flags_raw | ((u32::from(left_shift) & 0b1_1111) << 13)
}

/// Encode a mono PCM buffer at a **sub-byte bit-depth** (e.g. 12-bit,
/// 20-bit) into one complete `wvpk` block, setting the wiki
/// flag-bits-13..=17 `left_shift` field so the decoder restores the
/// container scale.
///
/// `left_shift` (`1..=31`) is the number of low zero bits the container
/// format pads the narrow samples with. The encoder right-shifts each
/// sample by `left_shift` (the inverse of the decoder's final §1
/// normalization), folds the §5 CRC over those narrow values, and entropy-
/// codes them; the decoder reads the narrow stream, verifies the CRC over
/// the pre-shift buffer, then left-shifts back to the container scale —
/// recovering `pcm` exactly: `decode_stream(&out)? == pcm`.
///
/// Every input sample must be a multiple of `2^left_shift` (its low
/// `left_shift` bits zero, as genuine sub-byte-depth audio is); otherwise
/// the shift would drop data and [`Error::EncodeLeftShiftLosesData`] is
/// returned. A `left_shift` of `0` is rejected
/// ([`Error::EncodeLeftShiftZero`]) — use [`encode_block_mono`] for the
/// whole-byte case.
pub fn encode_block_mono_shifted(
    pcm: &[i32],
    left_shift: u8,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if left_shift == 0 {
        return Err(Error::EncodeLeftShiftZero);
    }
    let config = BlockConfig {
        left_shift,
        ..raw_config(true, bytes_per_sample)
    };
    encode_block_core(pcm, config, block_index, total_samples)
}

/// Encode an interleaved stereo PCM buffer at a sub-byte bit-depth into
/// one complete `wvpk` block — the stereo twin of
/// [`encode_block_mono_shifted`]. The interleaved length must be even.
/// `decode_stream(&out)? == pcm` exactly.
pub fn encode_block_stereo_shifted(
    pcm: &[i32],
    left_shift: u8,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if left_shift == 0 {
        return Err(Error::EncodeLeftShiftZero);
    }
    let config = BlockConfig {
        left_shift,
        ..raw_config(false, bytes_per_sample)
    };
    encode_block_core(pcm, config, block_index, total_samples)
}

/// Encode a mono PCM buffer into one complete `wvpk` block that carries a
/// **decorrelation pass list described by its raw `0x02`/`0x03`/`0x04`
/// metadata payloads** — the lossless-with-decorrelation encode path.
///
/// `terms` / `weights` / `samples` are the exact on-wire payloads of the
/// `0x02` (decorr terms), `0x03` (decorr weights) and `0x04` (decorr seed
/// samples) sub-blocks, in the same byte layout
/// [`crate::decorrelation::assemble_mono_passes`] reads (wire order:
/// encoder's last-applied pass first; spec §3.7). The function:
///
/// 1. Assembles the application-ordered pass list from those payloads
///    (validating them — an invalid term / weight count / seed count is
///    surfaced verbatim).
/// 2. Runs the §3 forward prediction loop ([`recorrelate_mono`]) to turn
///    the PCM into residuals.
/// 3. Emits the three decorrelation sub-blocks **verbatim** (the exact
///    bytes passed in) ahead of the `0x0A` packed residuals, so the
///    decoder assembles the identical pass list and its inverse loop
///    reconstructs the original PCM.
///
/// Emitting the payloads verbatim makes the round trip bit-exact by
/// construction — the decoder reads back the same bytes it would have read
/// from a real file — without re-deriving the log-packed weight / seed
/// bytes from the working pass state. `decode_stream(&out)? == pcm`.
pub fn encode_block_mono_with_decorr(
    pcm: &[i32],
    terms: &[u8],
    weights: &[u8],
    samples: &[u8],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    let config = BlockConfig {
        decorr: Some((terms, weights, samples)),
        ..raw_config(true, bytes_per_sample)
    };
    encode_block_core(pcm, config, block_index, total_samples)
}

/// Encode an interleaved (`[L0, R0, L1, R1, …]`) stereo PCM buffer into
/// one complete `wvpk` block via the raw (no-decorrelation) lossless path.
///
/// The interleaved length must be even (whole `[L, R]` pairs). The block
/// is plain (non-joint, independent-channel) stereo. For a joint (mid/
/// side) block use [`encode_block_stereo_joint`]; for a decorrelated
/// block use [`encode_block_stereo_with_decorr`]; for sub-byte depth use
/// [`encode_block_stereo_shifted`].
///
/// `decode_stream(&out)? == pcm` exactly.
pub fn encode_block_stereo(
    pcm: &[i32],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    encode_block_core(
        pcm,
        raw_config(false, bytes_per_sample),
        block_index,
        total_samples,
    )
}

/// Encode an interleaved stereo PCM buffer into one complete **joint
/// (mid/side) stereo** `wvpk` block — the raw (no-decorrelation) lossless
/// path with the spec §5.4 joint-stereo flag (bit 4) set.
///
/// The forward mid/side transform ([`forward_joint_stereo`]) is applied
/// per `(L, R)` pair before entropy coding; the decoder runs the inverse
/// ([`crate::crc::undo_joint_stereo`]) after decode and computes the §5
/// CRC over the recovered true L/R. The spec §5.4 `mid >> 1` truncation
/// cancels between the forward and inverse transforms, so the block is
/// bit-exactly lossless: `decode_stream(&out)? == pcm`.
///
/// Joint coding decorrelates the inter-channel redundancy of typical
/// stereo material, so this is the compression-favouring stereo encode;
/// [`encode_block_stereo`] is the plain (independent-channel) twin.
pub fn encode_block_stereo_joint(
    pcm: &[i32],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    let config = BlockConfig {
        joint: true,
        ..raw_config(false, bytes_per_sample)
    };
    encode_block_core(pcm, config, block_index, total_samples)
}

/// Encode an interleaved stereo PCM buffer into one complete `wvpk` block
/// carrying a decorrelation pass list described by its raw
/// `0x02`/`0x03`/`0x04` metadata payloads — the stereo twin of
/// [`encode_block_mono_with_decorr`].
///
/// The payloads use the stereo wire layout
/// [`crate::decorrelation::assemble_stereo_passes`] reads (two weight
/// bytes per pass, per-channel seeds, cross terms allowed). The three
/// sub-blocks are emitted verbatim so the round trip is bit-exact:
/// `decode_stream(&out)? == pcm`.
pub fn encode_block_stereo_with_decorr(
    pcm: &[i32],
    terms: &[u8],
    weights: &[u8],
    samples: &[u8],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    let config = BlockConfig {
        decorr: Some((terms, weights, samples)),
        ..raw_config(false, bytes_per_sample)
    };
    encode_block_core(pcm, config, block_index, total_samples)
}

/// Encode an interleaved stereo PCM buffer into one complete **joint
/// (mid/side) stereo** `wvpk` block carrying a decorrelation pass list
/// described by its raw `0x02`/`0x03`/`0x04` metadata payloads — the
/// combination of [`encode_block_stereo_joint`] and
/// [`encode_block_stereo_with_decorr`].
///
/// Mirroring the decoder's stage order (entropy → decorrelate → joint
/// undo → CRC), the encoder folds the §5 CRC over the true L/R, applies
/// the forward mid/side transform, and *then* runs the §3 forward
/// prediction loop over the mid/side buffer — so the decorrelation
/// payloads describe passes over the joint-transformed domain. The
/// three sub-blocks are emitted verbatim, so the round trip is
/// bit-exact: `decode_stream(&out)? == pcm`.
pub fn encode_block_stereo_joint_with_decorr(
    pcm: &[i32],
    terms: &[u8],
    weights: &[u8],
    samples: &[u8],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    let config = BlockConfig {
        joint: true,
        decorr: Some((terms, weights, samples)),
        ..raw_config(false, bytes_per_sample)
    };
    encode_block_core(pcm, config, block_index, total_samples)
}

/// Decorrelation strength profile for the self-deriving (`*_auto`)
/// encoders — how many prediction passes the encoder derives and runs.
///
/// The term lists are **this encoder's own choices** among the spec §2
/// valid set (`1..8`, `17`, `18`, and the stereo cross terms): any
/// ordered valid list is a conformant block, because the decoder
/// reconstructs whatever pass list the `0x02`/`0x03`/`0x04` metadata
/// describes. More passes model more structure (usually smaller blocks)
/// at more encode/decode arithmetic per sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DecorrProfile {
    /// Two extrapolate passes — cheapest derivation, catches the
    /// dominant smooth/linear structure.
    Fast,
    /// Five passes mixing extrapolators and short fixed lags (plus a
    /// zero-delay cross pass on stereo).
    Normal,
    /// Eight passes over a wider lag spread (plus a mutual cross pass on
    /// stereo).
    High,
    /// Sixteen passes — the spec §2.1 `MAX_NTERMS` ceiling, covering
    /// every fixed lag `1..8`, both extrapolators repeatedly, and (on
    /// stereo) two cross passes. The deepest derivation this encoder
    /// offers: the pass count cannot legally grow past this.
    Extra,
}

impl DecorrProfile {
    /// The nested effort ladder up to and including this profile — the
    /// candidate set the `*_best` mode search walks. The profiles are
    /// ordered by derivation cost (`Fast ⊂ Normal ⊂ High` as search
    /// sets), and which term stack yields the smallest block is
    /// signal-dependent, so a deeper ceiling always *tries* the cheaper
    /// stacks too and can only match or beat them.
    pub fn search_set(self) -> &'static [DecorrProfile] {
        match self {
            DecorrProfile::Fast => &[DecorrProfile::Fast],
            DecorrProfile::Normal => &[DecorrProfile::Fast, DecorrProfile::Normal],
            DecorrProfile::High => &[
                DecorrProfile::Fast,
                DecorrProfile::Normal,
                DecorrProfile::High,
            ],
            DecorrProfile::Extra => &[
                DecorrProfile::Fast,
                DecorrProfile::Normal,
                DecorrProfile::High,
                DecorrProfile::Extra,
            ],
        }
    }

    /// The application-ordered `(term, delta)` list for a mono (or
    /// false-stereo / per-channel) derivation.
    fn mono_terms(self) -> &'static [(i8, i32)] {
        match self {
            DecorrProfile::Fast => &[(18, 2), (17, 2)],
            DecorrProfile::Normal => &[(18, 2), (18, 2), (2, 2), (17, 2), (3, 2)],
            DecorrProfile::High => &[
                (18, 2),
                (18, 2),
                (18, 2),
                (1, 2),
                (2, 2),
                (3, 2),
                (5, 2),
                (17, 2),
            ],
            DecorrProfile::Extra => &[
                (18, 2),
                (18, 2),
                (18, 2),
                (18, 2),
                (1, 2),
                (2, 2),
                (3, 2),
                (4, 2),
                (5, 2),
                (6, 2),
                (7, 2),
                (8, 2),
                (17, 2),
                (17, 2),
                (2, 2),
                (3, 2),
            ],
        }
    }

    /// The application-ordered `(term, delta)` list for a stereo
    /// derivation: the per-channel list plus a trailing zero-delay cross
    /// pass (spec §3.3) on the deeper profiles.
    fn stereo_terms(self) -> &'static [(i8, i32)] {
        match self {
            DecorrProfile::Fast => &[(18, 2), (17, 2)],
            DecorrProfile::Normal => &[(18, 2), (18, 2), (2, 2), (17, 2), (3, 2), (-1, 2)],
            DecorrProfile::High => &[
                (18, 2),
                (18, 2),
                (18, 2),
                (1, 2),
                (2, 2),
                (3, 2),
                (5, 2),
                (17, 2),
                (-3, 2),
            ],
            DecorrProfile::Extra => &[
                (18, 2),
                (18, 2),
                (18, 2),
                (1, 2),
                (2, 2),
                (3, 2),
                (4, 2),
                (5, 2),
                (6, 2),
                (7, 2),
                (8, 2),
                (17, 2),
                (17, 2),
                (2, 2),
                (-1, 2),
                (-3, 2),
            ],
        }
    }
}

/// Derive a serializable, application-ordered decorrelation pass list
/// for a **mono** PCM buffer by training the spec §3.4 weight
/// adaptation over the block.
///
/// The derivation is a two-step bootstrap:
///
/// 1. **Training pass** — run the forward prediction loop
///    ([`recorrelate_mono`]) over a scratch copy of the block with
///    zero-state passes (zero weights, zero seeds). The §3.4 `±delta`
///    adaptation walks each pass's weight toward the block's actual
///    inter-sample correlation.
/// 2. **Quantize + rebuild** — quantize each trained weight to its
///    `0x03` stored-byte value ([`quantize_weight`]) and rebuild fresh
///    zero-seed passes carrying those starting weights.
///
/// The returned list is serializable by construction
/// ([`serialize_mono_passes`] cannot refuse it) and ready for
/// [`encode_block_mono_with_decorr`] — the decoder reconstructs the
/// identical starting state from the metadata, so the round trip stays
/// bit-exact regardless of how well the training matched the signal.
pub fn derive_mono_passes(pcm: &[i32], profile: DecorrProfile) -> Result<Vec<DecorrPass>> {
    derive_mono_passes_for_spec(pcm, profile.mono_terms(), 1)
}

/// Spec-driven core of the mono derivation: train an arbitrary
/// application-ordered `(term, delta)` list over the block for
/// `iterations` sweeps and return zero-seed passes carrying the final
/// quantized starting weights.
///
/// One sweep is the round-383 bootstrap (train from the current
/// starting weights via [`recorrelate_mono`], quantize the trained end
/// weights into the next starting weights). Additional sweeps re-train
/// from the previous sweep's quantized result: the §3.4 adaptation then
/// starts near the block's own correlation instead of at zero, so the
/// early samples of the block are predicted well immediately. The
/// sequence converges toward a per-block fixpoint of
/// `quantize ∘ train`; `iterations == 1` reproduces the round-383
/// behaviour exactly. `iterations` is clamped to at least one sweep.
fn derive_mono_passes_for_spec(
    pcm: &[i32],
    spec: &[(i8, i32)],
    iterations: u32,
) -> Result<Vec<DecorrPass>> {
    let zeros = [0i32; MAX_TERM as usize];
    let mut weights = vec![0i32; spec.len()];
    for _ in 0..iterations.max(1) {
        let mut training: Vec<DecorrPass> = spec
            .iter()
            .zip(weights.iter())
            .map(|(&(term, delta), &w)| DecorrPass::new(term, delta, w, 0, &zeros, &[]))
            .collect::<Result<_>>()?;
        let mut scratch = pcm.to_vec();
        recorrelate_mono(&mut training, &mut scratch)?;
        for (w, trained) in weights.iter_mut().zip(training.iter()) {
            *w = quantize_weight(trained.weight_a);
        }
    }
    spec.iter()
        .zip(weights.iter())
        .map(|(&(term, delta), &w)| DecorrPass::new(term, delta, w, 0, &zeros, &[]))
        .collect()
}

/// Derive a mono pass list with **iterated** weight training — the
/// multi-sweep refinement of [`derive_mono_passes`].
///
/// Each sweep re-trains the §3.4 adaptation starting from the previous
/// sweep's quantized weights, walking the stored starting weights
/// toward the block's own `quantize ∘ train` fixpoint (see
/// [`derive_mono_passes_for_spec`]'s sweep semantics). One iteration is
/// exactly [`derive_mono_passes`]; two is usually where most of the
/// refinement lands, since the stored `0x03` byte only resolves the
/// weight to steps of 8. The result is serializable by construction and
/// round-trips bit-exactly regardless of the iteration count — the
/// decoder rebuilds whatever starting weights the metadata stores.
pub fn derive_mono_passes_iterated(
    pcm: &[i32],
    profile: DecorrProfile,
    iterations: u32,
) -> Result<Vec<DecorrPass>> {
    derive_mono_passes_for_spec(pcm, profile.mono_terms(), iterations)
}

/// Derive a serializable decorrelation pass list for an **interleaved
/// stereo** buffer — the two-channel twin of [`derive_mono_passes`]
/// (training via [`recorrelate_stereo`], both channels' weights
/// quantized, cross passes included per the profile).
///
/// `pcm` is the buffer the block will actually entropy-code: for a
/// joint (mid/side) block, pass the buffer *after* the forward joint
/// transform, so the training sees the same values the real prediction
/// loop will.
pub fn derive_stereo_passes(pcm: &[i32], profile: DecorrProfile) -> Result<Vec<DecorrPass>> {
    derive_stereo_passes_for_spec(pcm, profile.stereo_terms(), 1)
}

/// Spec-driven core of the stereo derivation — the two-channel twin of
/// [`derive_mono_passes_for_spec`]: `iterations` training sweeps over
/// the interleaved buffer, both channels' weights quantized between
/// sweeps, zero-seed passes carrying the final starting weights.
fn derive_stereo_passes_for_spec(
    pcm: &[i32],
    spec: &[(i8, i32)],
    iterations: u32,
) -> Result<Vec<DecorrPass>> {
    let zeros = [0i32; MAX_TERM as usize];
    let mut weights = vec![(0i32, 0i32); spec.len()];
    for _ in 0..iterations.max(1) {
        let mut training: Vec<DecorrPass> = spec
            .iter()
            .zip(weights.iter())
            .map(|(&(term, delta), &(wa, wb))| DecorrPass::new(term, delta, wa, wb, &zeros, &zeros))
            .collect::<Result<_>>()?;
        let mut scratch = pcm.to_vec();
        recorrelate_stereo(&mut training, &mut scratch)?;
        for (w, trained) in weights.iter_mut().zip(training.iter()) {
            *w = (
                quantize_weight(trained.weight_a),
                quantize_weight(trained.weight_b),
            );
        }
    }
    spec.iter()
        .zip(weights.iter())
        .map(|(&(term, delta), &(wa, wb))| DecorrPass::new(term, delta, wa, wb, &zeros, &zeros))
        .collect()
}

/// Derive a stereo pass list with **iterated** weight training — the
/// two-channel twin of [`derive_mono_passes_iterated`] (training via
/// [`recorrelate_stereo`], both channels refined per sweep, cross
/// passes included per the profile). One iteration is exactly
/// [`derive_stereo_passes`].
pub fn derive_stereo_passes_iterated(
    pcm: &[i32],
    profile: DecorrProfile,
    iterations: u32,
) -> Result<Vec<DecorrPass>> {
    derive_stereo_passes_for_spec(pcm, profile.stereo_terms(), iterations)
}

/// The `delta` (weight step, spec §2.1 top-3-bits field) every
/// greedy-search candidate pass uses.
const GREEDY_DELTA: i32 = 2;

/// The per-channel term candidates the greedy search draws from — the
/// full spec §2 non-cross valid set: every fixed lag plus both
/// extrapolators.
const GREEDY_CHANNEL_TERMS: &[i8] = &[1, 2, 3, 4, 5, 6, 7, 8, 17, 18];

/// The cross-term candidates added on stereo (spec §3.3).
const GREEDY_CROSS_TERMS: &[i8] = &[-1, -2, -3];

/// Magnitude-bits cost proxy the greedy search minimizes: the summed
/// bit length of every residual's absolute value. The entropy coder's
/// per-word cost grows with the residual's magnitude bits (the §4.2
/// interval ladder walks median multiples), so shrinking this sum is a
/// faithful, cheap stand-in for shrinking the coded block without
/// running the full entropy coder per candidate.
fn residual_cost(buffer: &[i32]) -> u64 {
    buffer
        .iter()
        .map(|&v| u64::from(32 - v.unsigned_abs().leading_zeros()))
        .sum()
}

/// Greedy core shared by the mono/stereo searched derivations: pick up
/// to `cap` terms, each the candidate that most reduces the residual
/// cost of the current domain, stopping early when no candidate
/// strictly improves. Returns the picked `(term, delta)` list in
/// pick order (= encode-application order, first-applied first).
fn greedy_pick_terms(
    domain: &mut Vec<i32>,
    candidates: &[i8],
    cap: usize,
    stereo: bool,
) -> Result<Vec<(i8, i32)>> {
    let zeros = [0i32; MAX_TERM as usize];
    let mut cost = residual_cost(domain);
    let mut picked: Vec<(i8, i32)> = Vec::new();
    while picked.len() < cap {
        let mut winner: Option<(i8, Vec<i32>, u64)> = None;
        for &term in candidates {
            let mut pass = DecorrPass::new(
                term,
                GREEDY_DELTA,
                0,
                0,
                &zeros,
                if stereo { &zeros[..] } else { &[] },
            )?;
            let mut scratch = domain.clone();
            let single = std::slice::from_mut(&mut pass);
            if stereo {
                recorrelate_stereo(single, &mut scratch)?;
            } else {
                recorrelate_mono(single, &mut scratch)?;
            }
            let c = residual_cost(&scratch);
            if c < winner.as_ref().map_or(cost, |w| w.2) {
                winner = Some((term, scratch, c));
            }
        }
        let Some((term, next, c)) = winner else {
            break;
        };
        picked.push((term, GREEDY_DELTA));
        *domain = next;
        cost = c;
    }
    Ok(picked)
}

/// Derive a **searched** decorrelation pass list for a mono PCM buffer:
/// instead of a fixed [`DecorrProfile`] term stack, each pass's term is
/// chosen greedily from the full spec §2 valid set by measuring which
/// candidate most reduces the residual magnitude-bits cost of the
/// buffer the previous picks produced.
///
/// The search stops as soon as no candidate strictly improves the
/// domain (so trailing dead passes are never emitted) or at
/// `max_passes` (clamped to the spec §2.1 `MAX_NTERMS` = 16 cap). The
/// picked stack is then re-trained from the original PCM with two
/// iterated sweeps ([`derive_mono_passes_iterated`] semantics), so the
/// stored starting weights match the exact composition the decoder
/// will run. An empty result (nothing improved — e.g. constant zero
/// input) is valid: it means "encode raw".
///
/// Like every derivation in this module the result is the encoder's own
/// choice among conformant pass lists — any ordered valid list decodes
/// per the `0x02`/`0x03`/`0x04` metadata, and the round trip stays
/// bit-exact regardless of how well the search matched the signal.
pub fn derive_mono_passes_searched(pcm: &[i32], max_passes: usize) -> Result<Vec<DecorrPass>> {
    let cap = max_passes.min(crate::decorrelation::MAX_NTERMS);
    let mut domain = pcm.to_vec();
    let picked = greedy_pick_terms(&mut domain, GREEDY_CHANNEL_TERMS, cap, false)?;
    // The forward recorrelation walks the application-ordered list
    // back-to-front, so the first-picked (first-applied) term sits last.
    let spec: Vec<(i8, i32)> = picked.into_iter().rev().collect();
    derive_mono_passes_for_spec(pcm, &spec, 2)
}

/// Derive a **searched** decorrelation pass list for an interleaved
/// stereo buffer — the two-channel twin of
/// [`derive_mono_passes_searched`], with the spec §3.3 cross terms
/// (`-1`/`-2`/`-3`) added to the candidate set. `pcm` is the buffer the
/// block will actually entropy-code (post-joint for a mid/side block).
pub fn derive_stereo_passes_searched(pcm: &[i32], max_passes: usize) -> Result<Vec<DecorrPass>> {
    let cap = max_passes.min(crate::decorrelation::MAX_NTERMS);
    let candidates: Vec<i8> = GREEDY_CHANNEL_TERMS
        .iter()
        .chain(GREEDY_CROSS_TERMS.iter())
        .copied()
        .collect();
    let mut domain = pcm.to_vec();
    let picked = greedy_pick_terms(&mut domain, &candidates, cap, true)?;
    let spec: Vec<(i8, i32)> = picked.into_iter().rev().collect();
    derive_stereo_passes_for_spec(pcm, &spec, 2)
}

/// Encode a mono PCM buffer into one complete `wvpk` block with a
/// **self-derived** decorrelation pass list — the first entry point
/// that performs real prediction-based compression without the caller
/// authoring any metadata.
///
/// Derives the pass list from the PCM ([`derive_mono_passes`] — a
/// training pass over the block, trained weights quantized to their
/// stored-byte values), serializes it to the raw `0x02`/`0x03`/`0x04`
/// payloads ([`serialize_mono_passes`]), and encodes through the
/// verbatim-payload path ([`encode_block_mono_with_decorr`]), so the
/// lossless guarantee is inherited unchanged:
/// `decode_stream(&out)? == pcm`.
pub fn encode_block_mono_auto(
    pcm: &[i32],
    profile: DecorrProfile,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    let passes = derive_mono_passes(pcm, profile)?;
    let (terms, weights, samples) = serialize_mono_passes(&passes)?;
    encode_block_mono_with_decorr(
        pcm,
        &terms,
        &weights,
        &samples,
        bytes_per_sample,
        block_index,
        total_samples,
    )
}

/// Encode an interleaved stereo PCM buffer into one complete `wvpk`
/// block with a self-derived decorrelation pass list — the stereo twin
/// of [`encode_block_mono_auto`] (plain, independent-channel stereo;
/// the deeper profiles add a zero-delay cross pass).
/// `decode_stream(&out)? == pcm`.
pub fn encode_block_stereo_auto(
    pcm: &[i32],
    profile: DecorrProfile,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let passes = derive_stereo_passes(pcm, profile)?;
    let (terms, weights, samples) = serialize_stereo_passes(&passes)?;
    encode_block_stereo_with_decorr(
        pcm,
        &terms,
        &weights,
        &samples,
        bytes_per_sample,
        block_index,
        total_samples,
    )
}

/// Encode an interleaved stereo PCM buffer into one complete **joint
/// (mid/side) stereo** `wvpk` block with a self-derived decorrelation
/// pass list — the compression-favouring stereo auto path.
///
/// The prediction loop runs over the joint-transformed (mid/side)
/// buffer, so the derivation trains over the same domain: the forward
/// §5.4 transform is applied to a scratch copy first, the pass list is
/// derived from that ([`derive_stereo_passes`]), and the block is
/// assembled through [`encode_block_stereo_joint_with_decorr`].
/// `decode_stream(&out)? == pcm`.
pub fn encode_block_stereo_joint_auto(
    pcm: &[i32],
    profile: DecorrProfile,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    // Train over the joint (mid/side) domain the real loop will see.
    let mut joint = pcm.to_vec();
    for pair in joint.chunks_exact_mut(2) {
        let (mid, side) = forward_joint_stereo(pair[0], pair[1]);
        pair[0] = mid;
        pair[1] = side;
    }
    let passes = derive_stereo_passes(&joint, profile)?;
    let (terms, weights, samples) = serialize_stereo_passes(&passes)?;
    encode_block_stereo_joint_with_decorr(
        pcm,
        &terms,
        &weights,
        &samples,
        bytes_per_sample,
        block_index,
        total_samples,
    )
}

/// Detect the sub-byte-depth left-shift of a PCM buffer: the number of
/// low zero bits **every** sample shares (capped at the 5-bit wiki
/// flag-field maximum of 31), i.e. the largest `s` for which the buffer
/// is genuine `2^s`-scaled audio the shifted encoders can narrow
/// losslessly. Returns `0` for an all-zero buffer (nothing to gain — the
/// zero-run fast path already collapses it) and for ordinary full-depth
/// audio.
///
/// This is how 12-/20-/24-bit-in-32-bit-container material announces
/// itself: e.g. 12-bit samples scaled `<< 4` into a 16-bit container
/// detect as `4`.
pub fn detect_left_shift(pcm: &[i32]) -> u8 {
    let mut common = 31u32;
    let mut any_nonzero = false;
    for &s in pcm {
        if s != 0 {
            any_nonzero = true;
            common = common.min(s.trailing_zeros());
            if common == 0 {
                return 0;
            }
        }
    }
    if any_nonzero {
        common as u8
    } else {
        0
    }
}

/// Keep `candidate` when it is strictly smaller than the current best
/// (or when there is no best yet).
fn keep_smaller(best: &mut Option<Vec<u8>>, candidate: Vec<u8>) {
    let improves = match best {
        None => true,
        Some(b) => candidate.len() < b.len(),
    };
    if improves {
        *best = Some(candidate);
    }
}

/// Encode a mono PCM buffer into the **smallest** block this encoder can
/// produce: auto-detects the sub-byte-depth left-shift
/// ([`detect_left_shift`]) and searches the raw candidate plus a
/// single-sweep and a twice-iterated derived-decorrelation candidate
/// per profile in the search set
/// ([`derive_mono_passes_iterated`]).
///
/// `profile` is the **search ceiling**, not a single choice: the
/// profiles form a nested effort ladder ([`DecorrProfile::search_set`]),
/// so `High` tries the `Fast` and `Normal` derivations too and keeps the
/// smallest output — which term stack wins is signal-dependent, and
/// trying the cheaper stacks is nearly free next to the entropy coding.
/// Every candidate decodes back to `pcm` bit-exactly, so the choice is
/// purely a size decision: `decode_stream(&out)? == pcm`.
pub fn encode_block_mono_best(
    pcm: &[i32],
    profile: DecorrProfile,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    let left_shift = detect_left_shift(pcm);
    let mut narrow = pcm.to_vec();
    if left_shift > 0 {
        narrow_left_shift(&mut narrow, left_shift)?;
    }

    let mut best: Option<Vec<u8>> = None;
    keep_smaller(
        &mut best,
        encode_block_core(
            pcm,
            BlockConfig {
                left_shift,
                ..raw_config(true, bytes_per_sample)
            },
            block_index,
            total_samples,
        )?,
    );

    // Two decorrelated candidates per profile in the search set — the
    // single-sweep and the twice-iterated derivation — each trained
    // over the narrow domain the real prediction loop sees.
    for &p in profile.search_set() {
        for iterations in [1u32, 2] {
            let passes = derive_mono_passes_iterated(&narrow, p, iterations)?;
            let (terms, weights, samples) = serialize_mono_passes(&passes)?;
            keep_smaller(
                &mut best,
                encode_block_core(
                    pcm,
                    BlockConfig {
                        left_shift,
                        decorr: Some((&terms, &weights, &samples)),
                        ..raw_config(true, bytes_per_sample)
                    },
                    block_index,
                    total_samples,
                )?,
            );
        }
    }
    // At least the raw candidate was pushed.
    Ok(best.expect("mode search produced no candidate"))
}

/// Encode an interleaved stereo PCM buffer into the **smallest** block
/// this encoder can produce, searching the mode grid: {plain, joint
/// mid/side} × ({raw} ∪ {single-sweep, twice-iterated derived
/// decorrelation per profile in the search set}), all at the
/// auto-detected left-shift. Like
/// [`encode_block_mono_best`], `profile` is the search **ceiling**
/// ([`DecorrProfile::search_set`]). Each decorrelated candidate trains
/// over the exact domain its prediction loop will run in (narrow, or
/// narrow + joint). Every candidate decodes back to `pcm` bit-exactly:
/// `decode_stream(&out)? == pcm`.
pub fn encode_block_stereo_best(
    pcm: &[i32],
    profile: DecorrProfile,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let left_shift = detect_left_shift(pcm);
    let mut narrow = pcm.to_vec();
    if left_shift > 0 {
        narrow_left_shift(&mut narrow, left_shift)?;
    }
    let mut joint_narrow = narrow.clone();
    for pair in joint_narrow.chunks_exact_mut(2) {
        let (mid, side) = forward_joint_stereo(pair[0], pair[1]);
        pair[0] = mid;
        pair[1] = side;
    }

    let mut best: Option<Vec<u8>> = None;
    for joint in [false, true] {
        let domain = if joint { &joint_narrow } else { &narrow };
        keep_smaller(
            &mut best,
            encode_block_core(
                pcm,
                BlockConfig {
                    joint,
                    left_shift,
                    ..raw_config(false, bytes_per_sample)
                },
                block_index,
                total_samples,
            )?,
        );
        for &p in profile.search_set() {
            for iterations in [1u32, 2] {
                let derived = derive_stereo_passes_iterated(domain, p, iterations)?;
                let (terms, weights, samples) = serialize_stereo_passes(&derived)?;
                keep_smaller(
                    &mut best,
                    encode_block_core(
                        pcm,
                        BlockConfig {
                            joint,
                            left_shift,
                            decorr: Some((&terms[..], &weights[..], &samples[..])),
                            ..raw_config(false, bytes_per_sample)
                        },
                        block_index,
                        total_samples,
                    )?,
                );
            }
        }
    }
    // At least the two raw candidates were pushed.
    Ok(best.expect("mode search produced no candidate"))
}

/// Encode a mono PCM buffer through the **greedy term search**
/// ([`derive_mono_passes_searched`]): the searched-stack candidate is
/// raced against the raw (no-decorrelation) candidate at the
/// auto-detected left-shift, and the smaller block wins — so the
/// searched encode never loses to raw even on signals where no term
/// helps. Every candidate decodes back to `pcm` bit-exactly:
/// `decode_stream(&out)? == pcm`.
pub fn encode_block_mono_searched(
    pcm: &[i32],
    max_passes: usize,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    let left_shift = detect_left_shift(pcm);
    let mut narrow = pcm.to_vec();
    if left_shift > 0 {
        narrow_left_shift(&mut narrow, left_shift)?;
    }

    let mut best: Option<Vec<u8>> = None;
    keep_smaller(
        &mut best,
        encode_block_core(
            pcm,
            BlockConfig {
                left_shift,
                ..raw_config(true, bytes_per_sample)
            },
            block_index,
            total_samples,
        )?,
    );
    let passes = derive_mono_passes_searched(&narrow, max_passes)?;
    if !passes.is_empty() {
        let (terms, weights, samples) = serialize_mono_passes(&passes)?;
        keep_smaller(
            &mut best,
            encode_block_core(
                pcm,
                BlockConfig {
                    left_shift,
                    decorr: Some((&terms, &weights, &samples)),
                    ..raw_config(true, bytes_per_sample)
                },
                block_index,
                total_samples,
            )?,
        );
    }
    Ok(best.expect("searched encode produced no candidate"))
}

/// Encode an interleaved stereo PCM buffer through the **greedy term
/// search** — the two-channel twin of [`encode_block_mono_searched`],
/// racing {plain, joint mid/side} × {raw, searched stack} and keeping
/// the smallest block. The searched derivation runs once per joint
/// mode over the exact domain that mode entropy-codes. Every candidate
/// decodes back to `pcm` bit-exactly.
pub fn encode_block_stereo_searched(
    pcm: &[i32],
    max_passes: usize,
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let left_shift = detect_left_shift(pcm);
    let mut narrow = pcm.to_vec();
    if left_shift > 0 {
        narrow_left_shift(&mut narrow, left_shift)?;
    }
    let mut joint_narrow = narrow.clone();
    for pair in joint_narrow.chunks_exact_mut(2) {
        let (mid, side) = forward_joint_stereo(pair[0], pair[1]);
        pair[0] = mid;
        pair[1] = side;
    }

    let mut best: Option<Vec<u8>> = None;
    for joint in [false, true] {
        let domain = if joint { &joint_narrow } else { &narrow };
        keep_smaller(
            &mut best,
            encode_block_core(
                pcm,
                BlockConfig {
                    joint,
                    left_shift,
                    ..raw_config(false, bytes_per_sample)
                },
                block_index,
                total_samples,
            )?,
        );
        let passes = derive_stereo_passes_searched(domain, max_passes)?;
        if !passes.is_empty() {
            let (terms, weights, samples) = serialize_stereo_passes(&passes)?;
            keep_smaller(
                &mut best,
                encode_block_core(
                    pcm,
                    BlockConfig {
                        joint,
                        left_shift,
                        decorr: Some((&terms[..], &weights[..], &samples[..])),
                        ..raw_config(false, bytes_per_sample)
                    },
                    block_index,
                    total_samples,
                )?,
            );
        }
    }
    Ok(best.expect("searched encode produced no candidate"))
}

/// Encode a mono PCM buffer into the smallest block **any** encoder in
/// this crate can currently produce: the union of the full
/// profile-ceiling mode search ([`encode_block_mono_best`] at
/// [`DecorrProfile::Extra`] — raw + eight derived-stack candidates)
/// and the greedy term search ([`encode_block_mono_searched`] at the
/// `MAX_NTERMS` cap). The two searches explore different stack spaces
/// (fixed curated profiles vs. signal-driven term picks), so their
/// union can only match or beat either alone. Bit-exact:
/// `decode_stream(&out)? == pcm`.
pub fn encode_block_mono_smallest(
    pcm: &[i32],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    let best = encode_block_mono_best(
        pcm,
        DecorrProfile::Extra,
        bytes_per_sample,
        block_index,
        total_samples,
    )?;
    let searched = encode_block_mono_searched(
        pcm,
        crate::decorrelation::MAX_NTERMS,
        bytes_per_sample,
        block_index,
        total_samples,
    )?;
    Ok(if searched.len() < best.len() {
        searched
    } else {
        best
    })
}

/// Encode an interleaved stereo PCM buffer into the smallest block any
/// encoder in this crate can currently produce — the stereo twin of
/// [`encode_block_mono_smallest`] (profile-ceiling grid ∪ greedy term
/// search, both already racing plain vs. joint mid/side internally).
pub fn encode_block_stereo_smallest(
    pcm: &[i32],
    bytes_per_sample: u8,
    block_index: u32,
    total_samples: u32,
) -> Result<Vec<u8>> {
    let best = encode_block_stereo_best(
        pcm,
        DecorrProfile::Extra,
        bytes_per_sample,
        block_index,
        total_samples,
    )?;
    let searched = encode_block_stereo_searched(
        pcm,
        crate::decorrelation::MAX_NTERMS,
        bytes_per_sample,
        block_index,
        total_samples,
    )?;
    Ok(if searched.len() < best.len() {
        searched
    } else {
        best
    })
}

/// Default per-block sample count the stream encoders split a long PCM
/// buffer into. A whole `.wv` file is a chain of `wvpk` blocks — the
/// walker ([`crate::block::iter_decoded_blocks`]) concatenates their PCM
/// — so a streaming encoder emits one block per fixed-size chunk. The
/// value is a per-channel sample count (the wiki "samples in this block"
/// header field), comfortably below the
/// [`crate::block::MAX_DECODE_SAMPLES_PER_BLOCK`] decode ceiling.
pub const DEFAULT_BLOCK_SAMPLES: usize = 22_050;

/// Encode a mono PCM buffer into a multi-block `.wv` byte stream — a
/// chain of `wvpk` blocks each carrying up to `block_samples` samples,
/// in the order [`crate::block::decode_stream`] concatenates them.
///
/// Each block's `block_index` is set to the running per-channel sample
/// offset (the wiki "offset in samples for current block" field) and
/// every block carries the same file-global `total_samples`
/// (`pcm.len()`), so the chain is a well-formed standalone file the
/// stream walker decodes back to `pcm` exactly:
/// `decode_stream(&encode_stream_mono(pcm, …)?)? == pcm`.
///
/// `block_samples` of `0` is treated as [`DEFAULT_BLOCK_SAMPLES`]. An
/// empty `pcm` yields an empty stream (no blocks) rather than an error —
/// a file with no audio.
pub fn encode_stream_mono(
    pcm: &[i32],
    block_samples: usize,
    bytes_per_sample: u8,
) -> Result<Vec<u8>> {
    let chunk = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut out = Vec::new();
    let mut index: u32 = 0;
    for window in pcm.chunks(chunk) {
        let block = encode_block_mono(window, bytes_per_sample, index, total)?;
        out.extend_from_slice(&block);
        index = index
            .checked_add(window.len() as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(out)
}

/// Encode an interleaved stereo PCM buffer into a multi-block `.wv` byte
/// stream. The stereo twin of [`encode_stream_mono`]: `block_samples` is
/// a per-channel pair count, so each block (bar the last) carries
/// `block_samples * 2` interleaved `i32`s. The interleaved length must be
/// even (whole `[L, R]` pairs).
///
/// `decode_stream(&encode_stream_stereo(pcm, …)?)? == pcm` exactly.
pub fn encode_stream_stereo(
    pcm: &[i32],
    block_samples: usize,
    bytes_per_sample: u8,
) -> Result<Vec<u8>> {
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let pairs = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut out = Vec::new();
    let mut index: u32 = 0;
    // Two i32s per pair, so the interleaved chunk size is `pairs * 2`.
    for window in pcm.chunks(pairs * 2) {
        let block = encode_block_stereo(window, bytes_per_sample, index, total)?;
        out.extend_from_slice(&block);
        index = index
            .checked_add((window.len() / 2) as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(out)
}

/// Encode a mono PCM buffer into a multi-block `.wv` byte stream where
/// **every block is the smallest this encoder can produce** — the
/// stream-level lift of [`encode_block_mono_best`]. Each chunk gets its
/// own left-shift detection, its own trained decorrelation pass list,
/// and its own raw-vs-decorrelated size decision, so a file whose
/// character changes over time (smooth passages, noisy passages,
/// silence) picks the best mode per block independently.
///
/// The chunking / header contract matches [`encode_stream_mono`]
/// (`block_samples` of `0` = [`DEFAULT_BLOCK_SAMPLES`]; running
/// `block_index`; file-global `total_samples`), and the stream decodes
/// back exactly: `decode_stream(&out)? == pcm`.
pub fn encode_stream_mono_best(
    pcm: &[i32],
    block_samples: usize,
    bytes_per_sample: u8,
    profile: DecorrProfile,
) -> Result<Vec<u8>> {
    let chunk = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut out = Vec::new();
    let mut index: u32 = 0;
    for window in pcm.chunks(chunk) {
        let block = encode_block_mono_best(window, profile, bytes_per_sample, index, total)?;
        out.extend_from_slice(&block);
        index = index
            .checked_add(window.len() as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(out)
}

/// Encode an interleaved stereo PCM buffer into a multi-block `.wv`
/// byte stream where every block is the smallest this encoder can
/// produce — the stream-level lift of [`encode_block_stereo_best`]
/// (per-block mode grid: {plain, joint} × {raw, derived decorrelation}
/// at the per-block detected left-shift). The chunking / header
/// contract matches [`encode_stream_stereo`].
/// `decode_stream(&out)? == pcm` exactly.
pub fn encode_stream_stereo_best(
    pcm: &[i32],
    block_samples: usize,
    bytes_per_sample: u8,
    profile: DecorrProfile,
) -> Result<Vec<u8>> {
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let pairs = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut out = Vec::new();
    let mut index: u32 = 0;
    for window in pcm.chunks(pairs * 2) {
        let block = encode_block_stereo_best(window, profile, bytes_per_sample, index, total)?;
        out.extend_from_slice(&block);
        index = index
            .checked_add((window.len() / 2) as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(out)
}

/// Encode a mono PCM buffer into the smallest multi-block `.wv` stream
/// this crate can currently produce: every block goes through the
/// union search ([`encode_block_mono_smallest`] — profile-ceiling grid
/// ∪ greedy term search), each window winning independently.
/// `block_samples` of `0` is [`DEFAULT_BLOCK_SAMPLES`]; an empty `pcm`
/// yields an empty stream. Bit-exact:
/// `decode_stream(&encode_stream_mono_smallest(pcm, …)?)? == pcm`.
pub fn encode_stream_mono_smallest(
    pcm: &[i32],
    block_samples: usize,
    bytes_per_sample: u8,
) -> Result<Vec<u8>> {
    let chunk = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut out = Vec::new();
    let mut index: u32 = 0;
    for window in pcm.chunks(chunk) {
        let block = encode_block_mono_smallest(window, bytes_per_sample, index, total)?;
        out.extend_from_slice(&block);
        index = index
            .checked_add(window.len() as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(out)
}

/// Encode an interleaved stereo PCM buffer into the smallest
/// multi-block `.wv` stream this crate can currently produce — the
/// stereo twin of [`encode_stream_mono_smallest`], one union search
/// ([`encode_block_stereo_smallest`]) per window.
pub fn encode_stream_stereo_smallest(
    pcm: &[i32],
    block_samples: usize,
    bytes_per_sample: u8,
) -> Result<Vec<u8>> {
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let pairs = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };
    let total = u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let mut out = Vec::new();
    let mut index: u32 = 0;
    for window in pcm.chunks(pairs * 2) {
        let block = encode_block_stereo_smallest(window, bytes_per_sample, index, total)?;
        out.extend_from_slice(&block);
        index = index
            .checked_add((window.len() / 2) as u32)
            .ok_or(Error::EncodeBlockTooLarge(pcm.len()))?;
    }
    Ok(out)
}

/// Encode an interleaved multichannel PCM buffer into a `.wv` byte stream
/// that [`crate::block::decode_multichannel_stream`] decodes back exactly.
///
/// `pcm` is interleaved by frame: `[ch0[0], ch1[0], …, chN-1[0], ch0[1],
/// …]`, `channels` `i32`s per frame, `pcm.len() == frames * channels`.
/// Each channel is emitted as its own **mono member block** (the simplest
/// unambiguously-lossless grouping): for each frame range the member
/// blocks carry the wiki bits-11..=12 grouping markers — the first
/// channel's block is the first-of-set (`0b01`), the last channel's is the
/// final-of-set (`0b10`), and the channels between are continuations
/// (`0b00`). A long buffer is split into successive sets of
/// `block_samples` frames each, all sharing the file-global
/// `total_samples` (frame count) and advancing `block_index`.
///
/// The decoder reassembles the per-frame interleave from the member order
/// (see `decode_multichannel_stream`), so:
/// `decode_multichannel_stream(&encode_multichannel_stream(pcm, channels,
/// …)?)?.samples == pcm` and `.channels == channels`.
///
/// `channels` must be `1..=MAX_MULTICHANNEL_CHANNELS` and divide
/// `pcm.len()` evenly; a `block_samples` of `0` uses
/// [`DEFAULT_BLOCK_SAMPLES`]. `channels == 1` produces a plain mono file
/// (every block standalone-equivalent); `channels == 2` produces two
/// mono members per frame range (a valid multichannel encoding of stereo,
/// distinct from the joint/interleaved single-block stereo path). An
/// empty `pcm` yields an empty stream. Round 378.
pub fn encode_multichannel_stream(
    pcm: &[i32],
    channels: usize,
    block_samples: usize,
    bytes_per_sample: u8,
) -> Result<Vec<u8>> {
    if channels == 0 || channels > crate::block::MAX_MULTICHANNEL_CHANNELS {
        return Err(Error::MultichannelTooManyChannels(channels));
    }
    if pcm.is_empty() {
        return Ok(Vec::new());
    }
    if pcm.len() % channels != 0 {
        // The interleaved buffer must be whole frames of `channels` each.
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let frames = pcm.len() / channels;
    let total = u32::try_from(frames).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    let chunk_frames = if block_samples == 0 {
        DEFAULT_BLOCK_SAMPLES
    } else {
        block_samples
    };

    let mut out = Vec::new();
    let mut frame_start: usize = 0;
    while frame_start < frames {
        let frame_end = (frame_start + chunk_frames).min(frames);
        let set_frames = frame_end - frame_start;
        let block_index =
            u32::try_from(frame_start).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;

        // De-interleave this frame range into one mono buffer per channel
        // and emit each as a member block with the right grouping marker.
        for ch in 0..channels {
            let mut channel_pcm = Vec::with_capacity(set_frames);
            for f in frame_start..frame_end {
                channel_pcm.push(pcm[f * channels + ch]);
            }
            // A single-channel set degenerates to a standalone block
            // (both first- and final-of-set, i.e. marker 0b11); otherwise
            // the first channel opens the set, the last closes it, and the
            // channels between are continuations.
            let marker = if channels == 1 {
                0b11 // standalone (first + final)
            } else if ch == 0 {
                0b01 // first-of-set
            } else if ch == channels - 1 {
                0b10 // final-of-set
            } else {
                0b00 // continuation
            };
            let block = encode_block_mono_marker(
                &channel_pcm,
                bytes_per_sample,
                block_index,
                total,
                marker,
            )?;
            out.extend_from_slice(&block);
        }
        frame_start = frame_end;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block::decode_stream;
    use crate::block_header::{parse_block_header, TOTAL_SAMPLES_UNKNOWN};
    use crate::metadata::{parse_metadata_sub_block, SubBlockId};

    /// The core lossless guarantee: a mono block encodes and decodes back
    /// to the exact input PCM, via the no-decorrelation raw-residual path.
    #[test]
    fn mono_raw_round_trip_recovers_pcm() {
        let pcm: Vec<i32> = vec![0, 1, -1, 100, -100, 32767, -32768, 5, 5, 5, 0, 0, 12345];
        let block = encode_block_mono(&pcm, 2, 0, pcm.len() as u32).unwrap();
        let decoded = decode_stream(&block).unwrap();
        assert_eq!(decoded, pcm);
    }

    /// A stereo block encodes and decodes back to the exact interleaved
    /// input PCM.
    #[test]
    fn stereo_raw_round_trip_recovers_pcm() {
        let pcm: Vec<i32> = vec![0, 0, 1, -1, 100, -100, -5, 5, 32767, -32768, 7, 7];
        let block = encode_block_stereo(&pcm, 2, 0, (pcm.len() / 2) as u32).unwrap();
        let decoded = decode_stream(&block).unwrap();
        assert_eq!(decoded, pcm);
    }

    /// The encoded block carries a valid header the parser accepts, with
    /// the correct magic, version, sample counts and an even ck_size.
    #[test]
    fn mono_block_header_round_trips_through_parser() {
        let pcm: Vec<i32> = vec![1, 2, 3, 4, 5];
        let block = encode_block_mono(&pcm, 3, 17, 99).unwrap();
        let (hdr, _payload) = parse_block_header(&block).unwrap();
        assert_eq!(hdr.version, ENCODE_VERSION);
        assert_eq!(hdr.block_samples, 5);
        assert_eq!(hdr.block_index, 17);
        assert_eq!(hdr.total_samples, 99);
        assert!(hdr.flags.mono);
        assert_eq!(hdr.flags.bytes_per_sample(), 3);
        // ck_size + 8 (magic + ck_size field) is the whole block length.
        assert_eq!(hdr.ck_size as usize + 8, block.len());
        // Not flagged a multichannel member (markers both set).
        assert!(!hdr.flags.is_multichannel_member());
    }

    /// The CRC the encoder writes matches the decoder's recomputed running
    /// CRC: the block passes the spec §5.6 mute gate (decode_stream_muted
    /// reports all-ok).
    #[test]
    fn encoded_block_passes_crc_gate() {
        let pcm: Vec<i32> = vec![3, -2, 5, 0, -7, 42, -42, 1000];
        let block = encode_block_mono(&pcm, 2, 0, pcm.len() as u32).unwrap();
        let (decoded, all_ok) = crate::block::decode_stream_muted(&block).unwrap();
        assert!(all_ok, "encoded block must pass its own CRC gate");
        assert_eq!(decoded, pcm);
    }

    /// The metadata region is exactly the two expected sub-blocks in
    /// order: 0x05 entropy info then 0x0A packed samples.
    #[test]
    fn mono_metadata_region_is_entropy_then_packed_samples() {
        let pcm: Vec<i32> = vec![10, 20, 30];
        let block = encode_block_mono(&pcm, 2, 0, 3).unwrap();
        let (_hdr, mut payload) = parse_block_header(&block).unwrap();
        let (first, rest) = parse_metadata_sub_block(payload).unwrap();
        assert_eq!(first.id, SubBlockId::EntropyInfo);
        assert_eq!(first.payload.len(), 6); // one mono median set
        payload = rest;
        let (second, rest2) = parse_metadata_sub_block(payload).unwrap();
        assert_eq!(second.id, SubBlockId::PackedSamples);
        assert!(
            rest2.is_empty(),
            "no trailing bytes after the two sub-blocks"
        );
    }

    /// An empty PCM buffer is rejected (an audio block carries >= 1
    /// sample).
    #[test]
    fn empty_pcm_is_rejected() {
        assert!(matches!(
            encode_block_mono(&[], 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo(&[], 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
    }

    /// An odd-length interleaved stereo buffer is rejected.
    #[test]
    fn stereo_odd_length_is_rejected() {
        assert!(matches!(
            encode_block_stereo(&[1, 2, 3], 2, 0, 1),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }

    /// A streaming total (TOTAL_SAMPLES_UNKNOWN) is preserved in the
    /// header and the block still round-trips.
    #[test]
    fn unknown_total_samples_round_trips() {
        let pcm: Vec<i32> = vec![7, 8, 9, 10];
        let block = encode_block_mono(&pcm, 2, 0, TOTAL_SAMPLES_UNKNOWN).unwrap();
        let (hdr, _) = parse_block_header(&block).unwrap();
        assert_eq!(hdr.total_samples_in_file(), None);
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// A buffer dominated by zeros exercises the spec §4.2 step-1 zero-run
    /// fast path on both encode and decode and still round-trips.
    #[test]
    fn mono_zero_run_heavy_round_trips() {
        let mut pcm = vec![0i32; 64];
        pcm[0] = 1;
        pcm[40] = -3;
        pcm[63] = 7;
        let block = encode_block_mono(&pcm, 2, 0, pcm.len() as u32).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// A larger pseudo-random mono buffer round-trips — broad coverage of
    /// the interval ladder and median adaptation across many words.
    #[test]
    fn mono_large_pseudo_random_round_trips() {
        let mut pcm = Vec::with_capacity(500);
        let mut state: u32 = 0x1234_5678;
        for _ in 0..500 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            // Map into a +-20000 range so the words exercise the ladder.
            pcm.push((state >> 16) as i16 as i32 / 3);
        }
        let block = encode_block_mono(&pcm, 2, 0, pcm.len() as u32).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// A larger pseudo-random stereo buffer round-trips.
    #[test]
    fn stereo_large_pseudo_random_round_trips() {
        let mut pcm = Vec::with_capacity(600);
        let mut state: u32 = 0x9E37_79B9;
        for _ in 0..600 {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            pcm.push((state >> 17) as i16 as i32 / 2);
        }
        let block = encode_block_stereo(&pcm, 2, 0, (pcm.len() / 2) as u32).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// A multi-block mono stream round-trips: a PCM buffer longer than the
    /// per-block chunk splits into several `wvpk` blocks the walker
    /// concatenates back to the exact input.
    #[test]
    fn mono_multi_block_stream_round_trips() {
        let mut pcm = Vec::with_capacity(1000);
        let mut state: u32 = 0xDEAD_BEEF;
        for _ in 0..1000 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            pcm.push((state >> 18) as i16 as i32 / 4);
        }
        // 7 samples per block forces ~143 blocks.
        let stream = encode_stream_mono(&pcm, 7, 2).unwrap();
        assert_eq!(decode_stream(&stream).unwrap(), pcm);
        // More than one block was emitted.
        assert!(crate::block::audio_block_count(&stream).unwrap() > 1);
    }

    /// A multi-block mono stream's block_index fields advance by each
    /// block's sample count, and every block carries the file total.
    #[test]
    fn mono_stream_block_indices_advance() {
        let pcm: Vec<i32> = (0..25).collect();
        let stream = encode_stream_mono(&pcm, 10, 2).unwrap();
        let mut payload = stream.as_slice();
        let mut expected_index = 0u32;
        let mut seen = 0;
        while !payload.is_empty() {
            let (hdr, _) = parse_block_header(payload).unwrap();
            assert_eq!(hdr.block_index, expected_index);
            assert_eq!(hdr.total_samples, 25);
            expected_index += hdr.block_samples;
            payload = &payload[8 + hdr.ck_size as usize..];
            seen += 1;
        }
        assert_eq!(seen, 3); // 10 + 10 + 5
        assert_eq!(expected_index, 25);
    }

    /// A multi-block stereo stream round-trips.
    #[test]
    fn stereo_multi_block_stream_round_trips() {
        let mut pcm = Vec::with_capacity(800);
        let mut state: u32 = 0x0BAD_F00D;
        for _ in 0..800 {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            pcm.push((state >> 19) as i16 as i32 / 3);
        }
        // 11 pairs per block.
        let stream = encode_stream_stereo(&pcm, 11, 2).unwrap();
        assert_eq!(decode_stream(&stream).unwrap(), pcm);
        assert!(crate::block::audio_block_count(&stream).unwrap() > 1);
    }

    /// An empty PCM buffer yields an empty stream (a file with no audio),
    /// not an error.
    #[test]
    fn empty_stream_is_empty_not_an_error() {
        assert!(encode_stream_mono(&[], 0, 2).unwrap().is_empty());
        assert!(encode_stream_stereo(&[], 0, 2).unwrap().is_empty());
    }

    /// A block-samples of 0 falls back to the default chunk size and still
    /// round-trips (a single block for a short buffer).
    #[test]
    fn zero_block_samples_uses_default_chunk() {
        let pcm: Vec<i32> = vec![1, 2, 3, 4, 5];
        let stream = encode_stream_mono(&pcm, 0, 2).unwrap();
        assert_eq!(decode_stream(&stream).unwrap(), pcm);
        assert_eq!(crate::block::audio_block_count(&stream).unwrap(), 1);
    }

    /// An odd-length interleaved stereo stream is rejected.
    #[test]
    fn stereo_stream_odd_length_is_rejected() {
        assert!(matches!(
            encode_stream_stereo(&[1, 2, 3], 0, 2),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }

    // ---- sub-byte-depth (left-shift) encode ----

    /// A 12-bit mono buffer (values shifted left into a 16-bit container)
    /// round-trips through the left-shift encode path.
    #[test]
    fn mono_shifted_round_trips() {
        // 12-bit samples (in -2048..=2047) scaled into a 16-bit container
        // by << 4 (left_shift = 4).
        let narrow = [0i32, 1, -1, 2047, -2048, 100, -100, 500];
        let pcm: Vec<i32> = narrow.iter().map(|&v| v << 4).collect();
        let block = encode_block_mono_shifted(&pcm, 4, 2, 0, pcm.len() as u32).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
        let (hdr, _) = parse_block_header(&block).unwrap();
        assert_eq!(hdr.flags.left_shift, 4);
    }

    /// A 20-bit stereo buffer (<< 12 into a 32-bit container) round-trips.
    #[test]
    fn stereo_shifted_round_trips() {
        let narrow = [0i32, 5, -7, 1000, -1000, 524287, -524288, 42];
        let pcm: Vec<i32> = narrow.iter().map(|&v| v << 12).collect();
        let block = encode_block_stereo_shifted(&pcm, 12, 4, 0, (pcm.len() / 2) as u32).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
        let (hdr, _) = parse_block_header(&block).unwrap();
        assert_eq!(hdr.flags.left_shift, 12);
    }

    /// A left-shift of 0 is rejected (use the whole-byte encoder).
    #[test]
    fn shifted_rejects_zero_shift() {
        assert!(matches!(
            encode_block_mono_shifted(&[16, 32], 0, 2, 0, 2),
            Err(Error::EncodeLeftShiftZero)
        ));
        assert!(matches!(
            encode_block_stereo_shifted(&[16, 32], 0, 2, 0, 1),
            Err(Error::EncodeLeftShiftZero)
        ));
    }

    /// A sample whose low bits the shift would drop is rejected (the
    /// encode would not be lossless).
    #[test]
    fn shifted_rejects_lossy_low_bits() {
        // 0b101 has a set bit below left_shift = 2.
        assert!(matches!(
            encode_block_mono_shifted(&[0b100, 0b101], 2, 2, 0, 2),
            Err(Error::EncodeLeftShiftLosesData(0b101))
        ));
    }

    // ---- joint (mid/side) stereo encode ----

    /// The forward joint-stereo transform is the exact inverse of the
    /// decoder's undo over a wide range of pairs.
    #[test]
    fn forward_joint_stereo_inverts_undo() {
        for left in [-32768, -1000, -1, 0, 1, 7, 1000, 32767, 1_000_000] {
            for right in [-32768, -3, 0, 5, 999, 32767, -1_000_000] {
                let (mid, side) = forward_joint_stereo(left, right);
                let (l2, r2) = crate::crc::undo_joint_stereo(mid, side);
                assert_eq!((l2, r2), (left, right), "pair ({left}, {right})");
            }
        }
    }

    /// A joint-stereo block round-trips: the decoder undoes mid/side and
    /// recovers the exact input L/R PCM.
    #[test]
    fn joint_stereo_block_round_trips() {
        let pcm: Vec<i32> = vec![100, 98, 105, 103, 110, 108, 90, 92, 0, 0, -50, -48];
        let block = encode_block_stereo_joint(&pcm, 2, 0, (pcm.len() / 2) as u32).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// A joint-stereo block sets the §5.4 joint flag (bit 4) and passes
    /// its own CRC gate.
    #[test]
    fn joint_stereo_block_sets_flag_and_passes_crc() {
        let pcm: Vec<i32> = vec![10, 11, 12, 13, 14, 15];
        let block = encode_block_stereo_joint(&pcm, 2, 0, (pcm.len() / 2) as u32).unwrap();
        let (hdr, _) = parse_block_header(&block).unwrap();
        assert!(hdr.flags.joint_stereo);
        assert!(!hdr.flags.mono);
        let (decoded, ok) = crate::block::decode_stream_muted(&block).unwrap();
        assert!(ok);
        assert_eq!(decoded, pcm);
    }

    /// A larger correlated (near-equal L/R) pseudo-random joint block
    /// round-trips — the case joint coding is designed for.
    #[test]
    fn joint_stereo_correlated_round_trips() {
        let mut pcm = Vec::with_capacity(400);
        let mut state: u32 = 0xFEED_FACE;
        for _ in 0..200 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let l = (state >> 20) as i16 as i32 / 8;
            // R close to L (high inter-channel correlation).
            let r = l + ((state >> 8) as i8 as i32 % 5);
            pcm.push(l);
            pcm.push(r);
        }
        let block = encode_block_stereo_joint(&pcm, 2, 0, (pcm.len() / 2) as u32).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// Joint-stereo encode rejects an empty / odd-length buffer.
    #[test]
    fn joint_stereo_rejects_empty_and_odd() {
        assert!(matches!(
            encode_block_stereo_joint(&[], 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_joint(&[1, 2, 3], 2, 0, 1),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }

    // ---- decorrelation-with-payload encode round-trips ----

    /// Spec-format `0x02` term byte: low 5 bits = `term + 5`, high 3 bits
    /// = `delta`.
    fn term_byte(term: i8, delta: u8) -> u8 {
        (((term + 5) as u8) & 0x1f) | (delta << 5)
    }

    /// `0x04` seed word for a value in -128..=127 (exponent 9 = shift 0).
    fn seed_word(v: i32) -> [u8; 2] {
        [v as i8 as u8, 9]
    }

    /// A single fixed-lag (term 1) decorrelation pass round-trips through
    /// the verbatim-payload encode path.
    #[test]
    fn mono_single_fixedlag_decorr_round_trips() {
        let pcm: Vec<i32> = vec![5, 9, 14, 20, 27, 35, 30, 22, 10, -4, -20];
        let terms = vec![term_byte(1, 2)];
        let weights = vec![40u8]; // arbitrary representable weight byte
        let samples = seed_word(3).to_vec(); // 1 seed for term 1
        let block =
            encode_block_mono_with_decorr(&pcm, &terms, &weights, &samples, 2, 0, pcm.len() as u32)
                .unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// A multi-pass mono decorrelation (term 2 then term 1, in wire order)
    /// round-trips.
    #[test]
    fn mono_multi_pass_decorr_round_trips() {
        let mut pcm = Vec::with_capacity(300);
        let mut state: u32 = 0xCAFE_F00D;
        for _ in 0..300 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            pcm.push((state >> 20) as i16 as i32 / 8);
        }
        // Wire order (encoder's last-applied first): term 2 then term 1.
        let terms = vec![term_byte(2, 1), term_byte(1, 1)];
        let weights = vec![30u8, 50u8];
        // term 2 needs 2 seeds, term 1 needs 1 seed; flat in wire order.
        let mut samples = Vec::new();
        samples.extend_from_slice(&seed_word(1));
        samples.extend_from_slice(&seed_word(-2));
        samples.extend_from_slice(&seed_word(4));
        let block =
            encode_block_mono_with_decorr(&pcm, &terms, &weights, &samples, 2, 0, pcm.len() as u32)
                .unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// An extrapolate term (17) round-trips.
    #[test]
    fn mono_extrapolate_term_round_trips() {
        let pcm: Vec<i32> = vec![100, 110, 119, 127, 134, 140, 145, 149, 150];
        let terms = vec![term_byte(17, 1)];
        let weights = vec![60u8];
        let mut samples = Vec::new();
        samples.extend_from_slice(&seed_word(2)); // s[-1]
        samples.extend_from_slice(&seed_word(1)); // s[-2]
        let block =
            encode_block_mono_with_decorr(&pcm, &terms, &weights, &samples, 2, 0, pcm.len() as u32)
                .unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// A stereo decorrelation block with a fixed-lag term per channel
    /// round-trips through the verbatim-payload path.
    #[test]
    fn stereo_decorr_round_trips() {
        let mut pcm = Vec::with_capacity(240);
        let mut state: u32 = 0x1357_9BDF;
        for _ in 0..240 {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            pcm.push((state >> 21) as i16 as i32 / 6);
        }
        let terms = vec![term_byte(1, 1)];
        // Stereo: two weight bytes per pass (channel A then B).
        let weights = vec![45u8, 35u8];
        // term 1: 1 seed per channel → A then B.
        let mut samples = Vec::new();
        samples.extend_from_slice(&seed_word(2)); // A
        samples.extend_from_slice(&seed_word(-1)); // B
        let block = encode_block_stereo_with_decorr(
            &pcm,
            &terms,
            &weights,
            &samples,
            2,
            0,
            (pcm.len() / 2) as u32,
        )
        .unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// A stereo cross term (-2) round-trips.
    #[test]
    fn stereo_cross_term_round_trips() {
        let pcm: Vec<i32> = vec![10, 12, 14, 11, 9, 13, 8, 16, 6, 18, 4, 20];
        let terms = vec![term_byte(-2, 1)];
        let weights = vec![20u8, 25u8];
        // cross term: 1 seed per channel.
        let mut samples = Vec::new();
        samples.extend_from_slice(&seed_word(1));
        samples.extend_from_slice(&seed_word(1));
        let block = encode_block_stereo_with_decorr(
            &pcm,
            &terms,
            &weights,
            &samples,
            2,
            0,
            (pcm.len() / 2) as u32,
        )
        .unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// The decorr-with-payload path emits all five sub-blocks in the
    /// documented order: 0x05, 0x02, 0x03, 0x04, 0x0A.
    #[test]
    fn decorr_block_sub_block_order() {
        use crate::metadata::SubBlockId as Id;
        let pcm: Vec<i32> = vec![1, 2, 3, 4, 5];
        let terms = vec![term_byte(1, 0)];
        let weights = vec![0u8];
        let samples = seed_word(0).to_vec();
        let block =
            encode_block_mono_with_decorr(&pcm, &terms, &weights, &samples, 2, 0, 5).unwrap();
        let (_hdr, mut payload) = parse_block_header(&block).unwrap();
        let expected = [
            Id::EntropyInfo,
            Id::DecorrelationTerms,
            Id::DecorrelationWeights,
            Id::DecorrelationSamples,
            Id::PackedSamples,
        ];
        for want in expected {
            let (sub, rest) = parse_metadata_sub_block(payload).unwrap();
            assert_eq!(sub.id, want);
            payload = rest;
        }
        assert!(payload.is_empty());
    }

    /// An invalid term byte in the payload is surfaced verbatim from the
    /// assembler rather than producing a corrupt block.
    #[test]
    fn decorr_invalid_term_is_rejected() {
        let pcm: Vec<i32> = vec![1, 2, 3];
        // term 0 (byte & 0x1f == 5 → term 0) is invalid.
        let terms = vec![term_byte(0, 0)];
        let weights = vec![0u8];
        let samples: Vec<u8> = Vec::new();
        assert!(matches!(
            encode_block_mono_with_decorr(&pcm, &terms, &weights, &samples, 2, 0, 3),
            Err(Error::InvalidDecorrelationTerm(0))
        ));
    }

    // ---- Multichannel stream encode round-trips (round 378) ------------

    #[test]
    fn multichannel_three_channel_round_trips() {
        use crate::block::decode_multichannel_stream;
        // 3 channels, 4 frames, interleaved [c0,c1,c2] per frame.
        let pcm: Vec<i32> = vec![
            10, 20, 30, // frame 0
            11, 21, 31, // frame 1
            -12, 22, -32, // frame 2
            13, -23, 33, // frame 3
        ];
        let out = encode_multichannel_stream(&pcm, 3, 0, 2).unwrap();
        let decoded = decode_multichannel_stream(&out).unwrap();
        assert_eq!(decoded.channels, 3);
        assert_eq!(decoded.samples, pcm);
    }

    #[test]
    fn multichannel_six_channel_round_trips() {
        use crate::block::decode_multichannel_stream;
        // 6 channels (5.1 layout shape), 3 frames.
        let mut pcm = Vec::new();
        for f in 0..3i32 {
            for ch in 0..6i32 {
                pcm.push(f * 100 + ch);
            }
        }
        let out = encode_multichannel_stream(&pcm, 6, 0, 2).unwrap();
        let decoded = decode_multichannel_stream(&out).unwrap();
        assert_eq!(decoded.channels, 6);
        assert_eq!(decoded.samples, pcm);
    }

    #[test]
    fn multichannel_split_into_multiple_sets_round_trips() {
        use crate::block::decode_multichannel_stream;
        // 4 channels, 5 frames, block_samples = 2 → 3 sets (2 + 2 + 1).
        let mut pcm = Vec::new();
        for f in 0..5i32 {
            for ch in 0..4i32 {
                pcm.push(f * 10 + ch);
            }
        }
        let out = encode_multichannel_stream(&pcm, 4, 2, 2).unwrap();
        let decoded = decode_multichannel_stream(&out).unwrap();
        assert_eq!(decoded.channels, 4);
        assert_eq!(decoded.samples, pcm);
    }

    #[test]
    fn multichannel_single_channel_is_plain_mono() {
        use crate::block::{decode_multichannel_stream, decode_stream};
        let pcm: Vec<i32> = vec![3, -2, 5, 0, -7, 9];
        let out = encode_multichannel_stream(&pcm, 1, 0, 2).unwrap();
        let decoded = decode_multichannel_stream(&out).unwrap();
        assert_eq!(decoded.channels, 1);
        assert_eq!(decoded.samples, pcm);
        // A single-channel set is a standalone block — also decodes via the
        // plain stream walker.
        assert_eq!(decode_stream(&out).unwrap(), pcm);
    }

    #[test]
    fn multichannel_empty_pcm_yields_empty_stream() {
        let out = encode_multichannel_stream(&[], 3, 0, 2).unwrap();
        assert!(out.is_empty());
    }

    #[test]
    fn multichannel_zero_channels_is_refused() {
        assert!(matches!(
            encode_multichannel_stream(&[1, 2, 3], 0, 0, 2),
            Err(Error::MultichannelTooManyChannels(0))
        ));
    }

    #[test]
    fn multichannel_ragged_buffer_is_refused() {
        // 7 samples is not a whole number of 3-channel frames.
        assert!(matches!(
            encode_multichannel_stream(&[1, 2, 3, 4, 5, 6, 7], 3, 0, 2),
            Err(Error::EncodeStereoOddLength(7))
        ));
    }

    #[test]
    fn multichannel_round_trips_under_muted_member_crc() {
        // Every member block carries a valid §5 CRC, so the muted decode
        // path agrees with the plain one and reports all_crc_ok.
        use crate::block::decode_multichannel_stream;
        let pcm: Vec<i32> = vec![1, 2, 3, 4, 5, 6, 7, 8];
        let out = encode_multichannel_stream(&pcm, 4, 0, 2).unwrap();
        let decoded = decode_multichannel_stream(&out).unwrap();
        assert_eq!(decoded.channels, 4);
        assert_eq!(decoded.samples, pcm);
    }

    // ---- self-deriving (auto) decorrelation encode (round 383) ----

    /// A smooth mono signal (ramp plus a small wiggle) the extrapolate
    /// terms model well.
    fn smooth_mono(len: usize) -> Vec<i32> {
        (0..len)
            .map(|i| {
                let i = i as i32;
                i * 13 - 1500 + ((i % 7) - 3)
            })
            .collect()
    }

    /// A correlated stereo buffer: right channel tracks left with a small
    /// offset, both smooth.
    fn smooth_stereo(pairs: usize) -> Vec<i32> {
        let mut pcm = Vec::with_capacity(pairs * 2);
        for i in 0..pairs {
            let i = i as i32;
            let l = i * 9 - 1000 + ((i % 5) - 2);
            pcm.push(l);
            pcm.push(l + 37 + ((i % 3) - 1));
        }
        pcm
    }

    /// Every profile's mono auto encode is bit-exactly lossless.
    #[test]
    fn mono_auto_round_trips_all_profiles() {
        let pcm = smooth_mono(400);
        for profile in [
            DecorrProfile::Fast,
            DecorrProfile::Normal,
            DecorrProfile::High,
        ] {
            let block = encode_block_mono_auto(&pcm, profile, 2, 0, pcm.len() as u32).unwrap();
            assert_eq!(
                decode_stream(&block).unwrap(),
                pcm,
                "profile {profile:?} must round-trip"
            );
            let (_, all_ok) = crate::block::decode_stream_muted(&block).unwrap();
            assert!(all_ok, "profile {profile:?} must pass its CRC gate");
        }
    }

    /// Every profile's stereo auto encode is bit-exactly lossless
    /// (including the cross-term passes on Normal / High).
    #[test]
    fn stereo_auto_round_trips_all_profiles() {
        let pcm = smooth_stereo(300);
        for profile in [
            DecorrProfile::Fast,
            DecorrProfile::Normal,
            DecorrProfile::High,
        ] {
            let block =
                encode_block_stereo_auto(&pcm, profile, 2, 0, (pcm.len() / 2) as u32).unwrap();
            assert_eq!(
                decode_stream(&block).unwrap(),
                pcm,
                "profile {profile:?} must round-trip"
            );
        }
    }

    /// Auto encode also round-trips pseudo-random (uncorrelated) input —
    /// the derivation may not help there, but it must never hurt
    /// correctness.
    #[test]
    fn auto_round_trips_pseudo_random_input() {
        let mut pcm = Vec::with_capacity(600);
        let mut state: u32 = 0xC0FF_EE01;
        for _ in 0..600 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            pcm.push((state >> 16) as i16 as i32);
        }
        let mono = encode_block_mono_auto(&pcm, DecorrProfile::Normal, 2, 0, 600).unwrap();
        assert_eq!(decode_stream(&mono).unwrap(), pcm);
        let stereo = encode_block_stereo_auto(&pcm, DecorrProfile::High, 2, 0, 300).unwrap();
        assert_eq!(decode_stream(&stereo).unwrap(), pcm);
    }

    /// The headline compression claim: on a smooth signal the derived
    /// decorrelation produces a smaller block than the raw path.
    #[test]
    fn auto_beats_raw_on_smooth_signal() {
        let pcm = smooth_mono(1000);
        let raw = encode_block_mono(&pcm, 2, 0, 1000).unwrap();
        let auto = encode_block_mono_auto(&pcm, DecorrProfile::Normal, 2, 0, 1000).unwrap();
        assert!(
            auto.len() < raw.len(),
            "decorrelated block ({}) must be smaller than raw ({}) on a smooth signal",
            auto.len(),
            raw.len()
        );

        let stereo = smooth_stereo(500);
        let raw_s = encode_block_stereo(&stereo, 2, 0, 500).unwrap();
        let auto_s = encode_block_stereo_auto(&stereo, DecorrProfile::Normal, 2, 0, 500).unwrap();
        assert!(
            auto_s.len() < raw_s.len(),
            "stereo decorrelated block ({}) must be smaller than raw ({})",
            auto_s.len(),
            raw_s.len()
        );
    }

    /// The training pass actually moves the weights: a linear ramp drives
    /// the leading extrapolate pass's weight well away from zero, and
    /// every derived weight is exactly its own quantization (serializable
    /// by construction).
    #[test]
    fn derived_passes_are_trained_and_quantized() {
        let pcm: Vec<i32> = (0..500).map(|i| i * 11 - 2700).collect();
        let passes = derive_mono_passes(&pcm, DecorrProfile::Fast).unwrap();
        assert_eq!(passes.len(), 2);
        assert!(
            passes[0].weight_a > 256,
            "ramp training must push the extrapolate weight up (got {})",
            passes[0].weight_a
        );
        for p in &passes {
            assert_eq!(
                quantize_weight(p.weight_a),
                p.weight_a,
                "derived weight must be storable verbatim"
            );
        }
        // And the serializer accepts the list without refusal.
        assert!(serialize_mono_passes(&passes).is_ok());
    }

    /// Stereo derivation trains both channels' weights independently and
    /// keeps the profile's cross pass serializable.
    #[test]
    fn derived_stereo_passes_cover_both_channels() {
        let pcm = smooth_stereo(400);
        let passes = derive_stereo_passes(&pcm, DecorrProfile::Normal).unwrap();
        assert_eq!(passes.len(), 6);
        assert!(passes.iter().any(|p| p.term < 0), "cross pass present");
        for p in &passes {
            assert_eq!(quantize_weight(p.weight_a), p.weight_a);
            assert_eq!(quantize_weight(p.weight_b), p.weight_b);
        }
        assert!(serialize_stereo_passes(&passes).is_ok());
    }

    /// Joint + decorrelation combined: the block carries the joint flag
    /// AND the three decorrelation sub-blocks, and round-trips exactly.
    #[test]
    fn joint_with_decorr_round_trips_with_both_features() {
        let pcm = smooth_stereo(300);
        // Derive over the joint domain by hand, mirroring the auto path.
        let mut joint = pcm.clone();
        for pair in joint.chunks_exact_mut(2) {
            let (mid, side) = forward_joint_stereo(pair[0], pair[1]);
            pair[0] = mid;
            pair[1] = side;
        }
        let passes = derive_stereo_passes(&joint, DecorrProfile::Normal).unwrap();
        let (t, w, s) = serialize_stereo_passes(&passes).unwrap();
        let block =
            encode_block_stereo_joint_with_decorr(&pcm, &t, &w, &s, 2, 0, (pcm.len() / 2) as u32)
                .unwrap();

        let (hdr, _) = parse_block_header(&block).unwrap();
        assert!(hdr.flags.joint_stereo, "joint flag set");
        assert_eq!(decode_stream(&block).unwrap(), pcm);
        let (_, all_ok) = crate::block::decode_stream_muted(&block).unwrap();
        assert!(all_ok, "CRC folds over true L/R");

        // The metadata chain carries 0x05, 0x02, 0x03, 0x04, 0x0A in order.
        let (_hdr, payload) = parse_block_header(&block).unwrap();
        let ids: Vec<SubBlockId> = {
            let mut rest = payload;
            let mut out = Vec::new();
            while !rest.is_empty() {
                let (sb, tail) = parse_metadata_sub_block(rest).unwrap();
                out.push(sb.id);
                rest = tail;
            }
            out
        };
        assert_eq!(
            ids,
            vec![
                SubBlockId::EntropyInfo,
                SubBlockId::DecorrelationTerms,
                SubBlockId::DecorrelationWeights,
                SubBlockId::DecorrelationSamples,
                SubBlockId::PackedSamples,
            ]
        );
    }

    /// The joint auto encoder round-trips on every profile.
    #[test]
    fn joint_auto_round_trips_all_profiles() {
        let pcm = smooth_stereo(250);
        for profile in [
            DecorrProfile::Fast,
            DecorrProfile::Normal,
            DecorrProfile::High,
        ] {
            let block = encode_block_stereo_joint_auto(&pcm, profile, 2, 0, 250).unwrap();
            assert_eq!(
                decode_stream(&block).unwrap(),
                pcm,
                "profile {profile:?} must round-trip"
            );
        }
    }

    /// On identical channels the mid channel is all-zero after the joint
    /// transform, so the joint auto block must beat the plain auto block.
    #[test]
    fn joint_auto_beats_plain_auto_on_identical_channels() {
        let mut pcm = Vec::with_capacity(600);
        for i in 0..300 {
            let v = i * 6 - 900 + ((i % 4) - 2);
            pcm.push(v);
            pcm.push(v);
        }
        let plain = encode_block_stereo_auto(&pcm, DecorrProfile::Normal, 2, 0, 300).unwrap();
        let joint = encode_block_stereo_joint_auto(&pcm, DecorrProfile::Normal, 2, 0, 300).unwrap();
        assert!(
            joint.len() < plain.len(),
            "joint auto ({}) must beat plain auto ({}) on identical channels",
            joint.len(),
            plain.len()
        );
        assert_eq!(decode_stream(&joint).unwrap(), pcm);
    }

    /// The joint auto encoder round-trips pseudo-random input too.
    #[test]
    fn joint_auto_round_trips_pseudo_random() {
        let mut pcm = Vec::with_capacity(500);
        let mut state: u32 = 0x1357_9BDF;
        for _ in 0..500 {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            pcm.push((state >> 15) as i16 as i32);
        }
        let block = encode_block_stereo_joint_auto(&pcm, DecorrProfile::High, 2, 0, 250).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), pcm);
    }

    /// Joint refusal arms carry over to the combined paths.
    #[test]
    fn joint_combined_refusal_arms() {
        assert!(matches!(
            encode_block_stereo_joint_with_decorr(&[], &[], &[], &[], 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_joint_auto(&[1, 2, 3], DecorrProfile::Fast, 2, 0, 1),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }

    // ---- left-shift detection + best-of mode selection (round 383) ----

    /// `detect_left_shift` reports the common low-zero-bit count, with
    /// the documented zero-buffer and full-depth-audio arms.
    #[test]
    fn detect_left_shift_arms() {
        assert_eq!(detect_left_shift(&[]), 0);
        assert_eq!(detect_left_shift(&[0, 0, 0]), 0);
        assert_eq!(detect_left_shift(&[1, 2, 3]), 0);
        // 12-bit audio scaled << 4.
        assert_eq!(detect_left_shift(&[16, -32, 4096, 0]), 4);
        // The minimum across samples wins.
        assert_eq!(detect_left_shift(&[64, 128, 8]), 3);
        // Cap at 31 (i32::MIN alone has 31 trailing zeros).
        assert_eq!(detect_left_shift(&[i32::MIN]), 31);
    }

    /// The best mono encoder auto-detects a 12-bit-style shift: the
    /// output carries the flag, round-trips, and beats the plain auto
    /// encoder that codes the wide container values.
    #[test]
    fn mono_best_detects_shift_and_beats_unshifted_auto() {
        let pcm: Vec<i32> = smooth_mono(600).iter().map(|&v| v << 4).collect();
        let best = encode_block_mono_best(&pcm, DecorrProfile::Normal, 2, 0, 600).unwrap();
        let (hdr, _) = parse_block_header(&best).unwrap();
        assert_eq!(hdr.flags.left_shift, 4, "detected shift stored in flags");
        assert_eq!(decode_stream(&best).unwrap(), pcm);

        let auto = encode_block_mono_auto(&pcm, DecorrProfile::Normal, 2, 0, 600).unwrap();
        assert!(
            best.len() < auto.len(),
            "shift-aware best ({}) must beat unshifted auto ({})",
            best.len(),
            auto.len()
        );
    }

    /// The best encoders never lose to any public single-mode candidate.
    #[test]
    fn best_is_no_larger_than_any_single_mode() {
        let mono = smooth_mono(500);
        let best = encode_block_mono_best(&mono, DecorrProfile::Normal, 2, 0, 500).unwrap();
        let raw = encode_block_mono(&mono, 2, 0, 500).unwrap();
        let auto = encode_block_mono_auto(&mono, DecorrProfile::Normal, 2, 0, 500).unwrap();
        assert!(best.len() <= raw.len() && best.len() <= auto.len());
        assert_eq!(decode_stream(&best).unwrap(), mono);

        let stereo = smooth_stereo(400);
        let best = encode_block_stereo_best(&stereo, DecorrProfile::Normal, 2, 0, 400).unwrap();
        for candidate in [
            encode_block_stereo(&stereo, 2, 0, 400).unwrap(),
            encode_block_stereo_auto(&stereo, DecorrProfile::Normal, 2, 0, 400).unwrap(),
            encode_block_stereo_joint(&stereo, 2, 0, 400).unwrap(),
            encode_block_stereo_joint_auto(&stereo, DecorrProfile::Normal, 2, 0, 400).unwrap(),
        ] {
            assert!(
                best.len() <= candidate.len(),
                "best ({}) lost to a single-mode candidate ({})",
                best.len(),
                candidate.len()
            );
        }
        assert_eq!(decode_stream(&best).unwrap(), stereo);
    }

    /// Stereo best on shifted identical channels combines all three
    /// features (joint + decorr + shift) and still round-trips.
    #[test]
    fn stereo_best_combines_shift_joint_decorr() {
        let mut pcm = Vec::with_capacity(500);
        for i in 0..250 {
            let v = (i * 5 - 600 + ((i % 6) - 3)) << 3;
            pcm.push(v);
            pcm.push(v);
        }
        let best = encode_block_stereo_best(&pcm, DecorrProfile::Normal, 2, 0, 250).unwrap();
        let (hdr, _) = parse_block_header(&best).unwrap();
        assert_eq!(hdr.flags.left_shift, 3, "detected shift stored");
        assert_eq!(decode_stream(&best).unwrap(), pcm);
        let (_, all_ok) = crate::block::decode_stream_muted(&best).unwrap();
        assert!(all_ok);
    }

    /// Best on pseudo-random data still round-trips (raw candidate may
    /// win — the choice is size-only, never correctness).
    #[test]
    fn best_round_trips_pseudo_random() {
        let mut pcm = Vec::with_capacity(400);
        let mut state: u32 = 0xFEED_5EED;
        for _ in 0..400 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            pcm.push((state >> 16) as i16 as i32);
        }
        let mono = encode_block_mono_best(&pcm, DecorrProfile::High, 2, 0, 400).unwrap();
        assert_eq!(decode_stream(&mono).unwrap(), pcm);
        let stereo = encode_block_stereo_best(&pcm, DecorrProfile::High, 2, 0, 200).unwrap();
        assert_eq!(decode_stream(&stereo).unwrap(), pcm);
    }

    // ---- stream-level best encode (round 383) ----

    /// A long smooth mono buffer through the best stream encoder:
    /// multi-block, bit-exact, and measurably smaller than the raw
    /// stream encoder.
    #[test]
    fn stream_mono_best_round_trips_and_compresses() {
        let pcm = smooth_mono(2000);
        let best = encode_stream_mono_best(&pcm, 250, 2, DecorrProfile::Normal).unwrap();
        assert_eq!(decode_stream(&best).unwrap(), pcm);
        assert!(crate::block::audio_block_count(&best).unwrap() > 1);

        let raw = encode_stream_mono(&pcm, 250, 2).unwrap();
        assert!(
            best.len() < raw.len(),
            "best stream ({}) must beat raw stream ({}) on smooth audio",
            best.len(),
            raw.len()
        );
        let (_, all_ok) = crate::block::decode_stream_muted(&best).unwrap();
        assert!(all_ok, "every block passes its CRC gate");
    }

    /// The stereo best stream encoder round-trips a correlated signal
    /// across blocks and beats the raw stream.
    #[test]
    fn stream_stereo_best_round_trips_and_compresses() {
        let pcm = smooth_stereo(1200);
        let best = encode_stream_stereo_best(&pcm, 200, 2, DecorrProfile::Normal).unwrap();
        assert_eq!(decode_stream(&best).unwrap(), pcm);
        assert!(crate::block::audio_block_count(&best).unwrap() > 1);

        let raw = encode_stream_stereo(&pcm, 200, 2).unwrap();
        assert!(
            best.len() < raw.len(),
            "best stream ({}) must beat raw stream ({})",
            best.len(),
            raw.len()
        );
    }

    /// Per-block independence: a stream whose first half is identical
    /// channels and second half uncorrelated noise still round-trips,
    /// with block headers reflecting per-block mode choices (at least
    /// one joint block in the correlated half is expected but not
    /// mandated — the pinned contract is exact decode + block count).
    #[test]
    fn stream_stereo_best_mixed_material_round_trips() {
        let mut pcm = Vec::with_capacity(1200);
        for i in 0..300 {
            let v = i * 4 - 600;
            pcm.push(v);
            pcm.push(v);
        }
        let mut state: u32 = 0xABCD_EF01;
        for _ in 0..300 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            pcm.push((state >> 16) as i16 as i32);
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            pcm.push((state >> 16) as i16 as i32);
        }
        let best = encode_stream_stereo_best(&pcm, 150, 2, DecorrProfile::Normal).unwrap();
        assert_eq!(decode_stream(&best).unwrap(), pcm);
        assert_eq!(crate::block::audio_block_count(&best).unwrap(), 4);
    }

    /// Stream-best headers carry the same block_index / total_samples
    /// contract as the plain stream encoders.
    #[test]
    fn stream_best_header_contract() {
        let pcm = smooth_mono(55);
        let stream = encode_stream_mono_best(&pcm, 20, 2, DecorrProfile::Fast).unwrap();
        let mut payload = stream.as_slice();
        let mut expected_index = 0u32;
        let mut seen = 0;
        while !payload.is_empty() {
            let (hdr, _) = parse_block_header(payload).unwrap();
            assert_eq!(hdr.block_index, expected_index);
            assert_eq!(hdr.total_samples, 55);
            expected_index += hdr.block_samples;
            payload = &payload[8 + hdr.ck_size as usize..];
            seen += 1;
        }
        assert_eq!(seen, 3); // 20 + 20 + 15
        assert_eq!(expected_index, 55);
    }

    /// Empty inputs yield empty streams; odd stereo is refused.
    #[test]
    fn stream_best_edge_arms() {
        assert!(encode_stream_mono_best(&[], 0, 2, DecorrProfile::Fast)
            .unwrap()
            .is_empty());
        assert!(encode_stream_stereo_best(&[], 0, 2, DecorrProfile::Fast)
            .unwrap()
            .is_empty());
        assert!(matches!(
            encode_stream_stereo_best(&[1, 2, 3], 0, 2, DecorrProfile::Fast),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }

    /// The search sets nest: Fast ⊂ Normal ⊂ High ⊂ Extra.
    #[test]
    fn profile_search_sets_nest() {
        assert_eq!(DecorrProfile::Fast.search_set(), &[DecorrProfile::Fast]);
        assert_eq!(
            DecorrProfile::Normal.search_set(),
            &[DecorrProfile::Fast, DecorrProfile::Normal]
        );
        assert_eq!(
            DecorrProfile::High.search_set(),
            &[
                DecorrProfile::Fast,
                DecorrProfile::Normal,
                DecorrProfile::High
            ]
        );
        assert_eq!(
            DecorrProfile::Extra.search_set(),
            &[
                DecorrProfile::Fast,
                DecorrProfile::Normal,
                DecorrProfile::High,
                DecorrProfile::Extra
            ]
        );
    }

    /// The Extra profile sits exactly at the spec §2.1 `MAX_NTERMS`
    /// pass-count ceiling in both channel shapes, and every derived pass
    /// list serializes (i.e. every term is in the spec §2 valid set).
    #[test]
    fn extra_profile_is_the_max_nterms_ceiling() {
        let mono = smooth_mono(400);
        let passes = derive_mono_passes(&mono, DecorrProfile::Extra).unwrap();
        assert_eq!(passes.len(), crate::decorrelation::MAX_NTERMS);
        serialize_mono_passes(&passes).unwrap();

        let stereo = smooth_stereo(200);
        let passes = derive_stereo_passes(&stereo, DecorrProfile::Extra).unwrap();
        assert_eq!(passes.len(), crate::decorrelation::MAX_NTERMS);
        serialize_stereo_passes(&passes).unwrap();
    }

    /// Extra-profile auto encodes stay bit-exact through the full block
    /// decoder in every channel shape.
    #[test]
    fn extra_profile_auto_round_trips_bit_exact() {
        let mono = smooth_mono(500);
        let block = encode_block_mono_auto(&mono, DecorrProfile::Extra, 2, 0, 500).unwrap();
        assert_eq!(decode_stream(&block).unwrap(), mono);

        let stereo = smooth_stereo(250);
        let plain = encode_block_stereo_auto(&stereo, DecorrProfile::Extra, 2, 0, 250).unwrap();
        assert_eq!(decode_stream(&plain).unwrap(), stereo);
        let joint =
            encode_block_stereo_joint_auto(&stereo, DecorrProfile::Extra, 2, 0, 250).unwrap();
        assert_eq!(decode_stream(&joint).unwrap(), stereo);
    }

    /// An Extra search ceiling can only match or beat High — its
    /// candidate set is a strict superset — and still decodes bit-exact.
    #[test]
    fn extra_ceiling_dominates_high() {
        let mono = smooth_mono(700);
        let high = encode_block_mono_best(&mono, DecorrProfile::High, 2, 0, 700).unwrap();
        let extra = encode_block_mono_best(&mono, DecorrProfile::Extra, 2, 0, 700).unwrap();
        assert!(extra.len() <= high.len());
        assert_eq!(decode_stream(&extra).unwrap(), mono);

        let stereo = smooth_stereo(350);
        let high = encode_block_stereo_best(&stereo, DecorrProfile::High, 2, 0, 350).unwrap();
        let extra = encode_block_stereo_best(&stereo, DecorrProfile::Extra, 2, 0, 350).unwrap();
        assert!(extra.len() <= high.len());
        assert_eq!(decode_stream(&extra).unwrap(), stereo);
    }

    /// One training sweep of the iterated derivation is exactly the
    /// single-sweep derivation — same terms, weights, and seeds.
    #[test]
    fn iterated_once_matches_single_sweep_derivation() {
        let mono = smooth_mono(400);
        assert_eq!(
            derive_mono_passes_iterated(&mono, DecorrProfile::High, 1).unwrap(),
            derive_mono_passes(&mono, DecorrProfile::High).unwrap()
        );
        // Zero iterations clamp to one sweep rather than skipping training.
        assert_eq!(
            derive_mono_passes_iterated(&mono, DecorrProfile::Normal, 0).unwrap(),
            derive_mono_passes(&mono, DecorrProfile::Normal).unwrap()
        );

        let stereo = smooth_stereo(200);
        assert_eq!(
            derive_stereo_passes_iterated(&stereo, DecorrProfile::High, 1).unwrap(),
            derive_stereo_passes(&stereo, DecorrProfile::High).unwrap()
        );
    }

    /// Iterated-training pass lists stay serializable and bit-exact
    /// through the verbatim-payload encoders at any iteration count.
    #[test]
    fn iterated_derivation_round_trips_bit_exact() {
        let mono = smooth_mono(500);
        for iterations in [2u32, 3, 5] {
            let passes =
                derive_mono_passes_iterated(&mono, DecorrProfile::High, iterations).unwrap();
            let (terms, weights, samples) = serialize_mono_passes(&passes).unwrap();
            let block = encode_block_mono_with_decorr(&mono, &terms, &weights, &samples, 2, 0, 500)
                .unwrap();
            assert_eq!(decode_stream(&block).unwrap(), mono);
        }

        let stereo = smooth_stereo(250);
        let passes = derive_stereo_passes_iterated(&stereo, DecorrProfile::Extra, 3).unwrap();
        let (terms, weights, samples) = serialize_stereo_passes(&passes).unwrap();
        let block = encode_block_stereo_with_decorr(&stereo, &terms, &weights, &samples, 2, 0, 250)
            .unwrap();
        assert_eq!(decode_stream(&block).unwrap(), stereo);
    }

    /// Extra sweeps refine only the starting weights — the term/delta
    /// stack (and thus the `0x02` payload) is sweep-invariant.
    #[test]
    fn iterated_derivation_keeps_the_term_stack() {
        let mono = smooth_mono(300);
        let one = derive_mono_passes_iterated(&mono, DecorrProfile::Normal, 1).unwrap();
        let three = derive_mono_passes_iterated(&mono, DecorrProfile::Normal, 3).unwrap();
        assert_eq!(one.len(), three.len());
        let (terms_one, _, _) = serialize_mono_passes(&one).unwrap();
        let (terms_three, _, _) = serialize_mono_passes(&three).unwrap();
        assert_eq!(terms_one, terms_three);
    }

    /// The greedy search respects the `MAX_NTERMS` clamp and any smaller
    /// caller cap, only ever emits serializable spec-valid terms, and
    /// stops early rather than padding with dead passes.
    #[test]
    fn searched_derivation_respects_caps_and_stays_valid() {
        let mono = smooth_mono(400);
        for cap in [1usize, 3, 16, 64] {
            let passes = derive_mono_passes_searched(&mono, cap).unwrap();
            assert!(passes.len() <= cap.min(crate::decorrelation::MAX_NTERMS));
            serialize_mono_passes(&passes).unwrap();
        }
        let stereo = smooth_stereo(200);
        let passes = derive_stereo_passes_searched(&stereo, 64).unwrap();
        assert!(passes.len() <= crate::decorrelation::MAX_NTERMS);
        serialize_stereo_passes(&passes).unwrap();
    }

    /// On a signal no term can improve (constant zero), the search picks
    /// nothing and the searched encoders fall back to the raw candidate.
    #[test]
    fn searched_derivation_picks_nothing_on_zeros() {
        let zeros = vec![0i32; 128];
        assert!(derive_mono_passes_searched(&zeros, 16).unwrap().is_empty());
        let out = encode_block_mono_searched(&zeros, 16, 2, 0, 128).unwrap();
        assert_eq!(decode_stream(&out).unwrap(), zeros);
    }

    /// Searched encodes round-trip bit-exactly in every channel shape
    /// and never lose to the raw encoder (the raw candidate is in the
    /// race).
    #[test]
    fn searched_encode_round_trips_and_beats_raw() {
        let mono = smooth_mono(600);
        let searched = encode_block_mono_searched(&mono, 16, 2, 0, 600).unwrap();
        let raw = encode_block_mono(&mono, 2, 0, 600).unwrap();
        assert!(searched.len() <= raw.len());
        assert_eq!(decode_stream(&searched).unwrap(), mono);

        let stereo = smooth_stereo(300);
        let searched = encode_block_stereo_searched(&stereo, 16, 2, 0, 300).unwrap();
        let raw = encode_block_stereo(&stereo, 2, 0, 300).unwrap();
        assert!(searched.len() <= raw.len());
        assert_eq!(decode_stream(&searched).unwrap(), stereo);
    }

    /// The searched mode compresses a correlated signal — it actually
    /// picks passes and lands strictly below raw, not just at parity.
    #[test]
    fn searched_encode_compresses_correlated_signal() {
        let mono = smooth_mono(800);
        let passes = derive_mono_passes_searched(&mono, 16).unwrap();
        assert!(!passes.is_empty(), "smooth signal must pick terms");
        let searched = encode_block_mono_searched(&mono, 16, 2, 0, 800).unwrap();
        let raw = encode_block_mono(&mono, 2, 0, 800).unwrap();
        assert!(
            searched.len() < raw.len(),
            "searched ({}) must beat raw ({}) on a smooth signal",
            searched.len(),
            raw.len()
        );
    }

    /// Searched encodes preserve the sub-byte-depth left-shift arm and
    /// the shared refusal arms.
    #[test]
    fn searched_encode_shift_and_refusals() {
        let mono: Vec<i32> = smooth_mono(300).iter().map(|s| s << 3).collect();
        let out = encode_block_mono_searched(&mono, 16, 2, 0, 300).unwrap();
        assert_eq!(decode_stream(&out).unwrap(), mono);

        assert!(matches!(
            encode_block_mono_searched(&[], 16, 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_searched(&[1, 2, 3], 16, 2, 0, 1),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }

    /// The union search can only match or beat both of its members and
    /// stays bit-exact in every channel shape.
    #[test]
    fn smallest_union_dominates_both_searches() {
        let mono = smooth_mono(700);
        let best = encode_block_mono_best(&mono, DecorrProfile::Extra, 2, 0, 700).unwrap();
        let searched = encode_block_mono_searched(&mono, 16, 2, 0, 700).unwrap();
        let smallest = encode_block_mono_smallest(&mono, 2, 0, 700).unwrap();
        assert!(smallest.len() <= best.len());
        assert!(smallest.len() <= searched.len());
        assert_eq!(decode_stream(&smallest).unwrap(), mono);

        let stereo = smooth_stereo(350);
        let best = encode_block_stereo_best(&stereo, DecorrProfile::Extra, 2, 0, 350).unwrap();
        let searched = encode_block_stereo_searched(&stereo, 16, 2, 0, 350).unwrap();
        let smallest = encode_block_stereo_smallest(&stereo, 2, 0, 350).unwrap();
        assert!(smallest.len() <= best.len());
        assert!(smallest.len() <= searched.len());
        assert_eq!(decode_stream(&smallest).unwrap(), stereo);
    }

    /// Stream-level smallest: multi-block chains round-trip bit-exactly,
    /// honour the zero-means-default chunking rule, and inherit the
    /// refusal arms.
    #[test]
    fn stream_smallest_round_trips_multi_block() {
        let mono = smooth_mono(700);
        let stream = encode_stream_mono_smallest(&mono, 256, 2).unwrap();
        assert!(crate::block::block_count(&stream).unwrap() > 1);
        assert_eq!(decode_stream(&stream).unwrap(), mono);

        let stereo = smooth_stereo(300);
        let stream = encode_stream_stereo_smallest(&stereo, 128, 2).unwrap();
        assert!(crate::block::block_count(&stream).unwrap() > 1);
        assert_eq!(decode_stream(&stream).unwrap(), stereo);

        // Zero chunk = DEFAULT_BLOCK_SAMPLES (single block here).
        let one = encode_stream_mono_smallest(&mono, 0, 2).unwrap();
        assert_eq!(crate::block::block_count(&one).unwrap(), 1);
        assert_eq!(decode_stream(&one).unwrap(), mono);

        // Empty PCM = empty stream; odd stereo refuses.
        assert!(encode_stream_mono_smallest(&[], 0, 2).unwrap().is_empty());
        assert!(matches!(
            encode_stream_stereo_smallest(&[1, 2, 3], 0, 2),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }

    /// A stream-level smallest encode is never larger than the
    /// stream-level Extra-ceiling best encode.
    #[test]
    fn stream_smallest_dominates_stream_best() {
        let mono = smooth_mono(600);
        let best = encode_stream_mono_best(&mono, 200, 2, DecorrProfile::Extra).unwrap();
        let smallest = encode_stream_mono_smallest(&mono, 200, 2).unwrap();
        assert!(smallest.len() <= best.len());
        assert_eq!(decode_stream(&smallest).unwrap(), mono);
    }

    /// The residual-cost proxy orders buffers by magnitude bits.
    #[test]
    fn residual_cost_orders_by_magnitude_bits() {
        assert_eq!(residual_cost(&[0, 0, 0]), 0);
        assert_eq!(residual_cost(&[1, -1]), 2); // |−1| = 1 → 1 bit each
        assert_eq!(residual_cost(&[3]), 2);
        assert_eq!(residual_cost(&[4]), 3);
        assert_eq!(residual_cost(&[i32::MIN]), 32);
        assert!(residual_cost(&[100, 100]) < residual_cost(&[10_000, 10_000]));
    }

    /// A deeper search ceiling can only match or beat a shallower one —
    /// the candidate sets nest, so the minimum is monotone.
    #[test]
    fn best_is_monotone_in_the_search_ceiling() {
        let mono = smooth_mono(700);
        let fast = encode_block_mono_best(&mono, DecorrProfile::Fast, 2, 0, 700).unwrap();
        let normal = encode_block_mono_best(&mono, DecorrProfile::Normal, 2, 0, 700).unwrap();
        let high = encode_block_mono_best(&mono, DecorrProfile::High, 2, 0, 700).unwrap();
        assert!(normal.len() <= fast.len());
        assert!(high.len() <= normal.len());
        for out in [fast, normal, high] {
            assert_eq!(decode_stream(&out).unwrap(), mono);
        }

        let stereo = smooth_stereo(350);
        let fast = encode_block_stereo_best(&stereo, DecorrProfile::Fast, 2, 0, 350).unwrap();
        let high = encode_block_stereo_best(&stereo, DecorrProfile::High, 2, 0, 350).unwrap();
        assert!(high.len() <= fast.len());
        assert_eq!(decode_stream(&high).unwrap(), stereo);
    }

    /// The ceiling search also dominates every single-profile auto
    /// encoder inside the ceiling.
    #[test]
    fn best_dominates_all_autos_in_ceiling() {
        let mono = smooth_mono(600);
        let best = encode_block_mono_best(&mono, DecorrProfile::High, 2, 0, 600).unwrap();
        for p in [
            DecorrProfile::Fast,
            DecorrProfile::Normal,
            DecorrProfile::High,
        ] {
            let auto = encode_block_mono_auto(&mono, p, 2, 0, 600).unwrap();
            assert!(
                best.len() <= auto.len(),
                "best-High ({}) lost to auto-{p:?} ({})",
                best.len(),
                auto.len()
            );
        }
        assert_eq!(decode_stream(&best).unwrap(), mono);
    }

    /// The best encoders share the standard refusal arms.
    #[test]
    fn best_refusal_arms() {
        assert!(matches!(
            encode_block_mono_best(&[], DecorrProfile::Fast, 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_best(&[], DecorrProfile::Fast, 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_best(&[1, 2, 3], DecorrProfile::Fast, 2, 0, 1),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }

    /// The auto encoders share the plain encoders' refusal arms.
    #[test]
    fn auto_refusal_arms() {
        assert!(matches!(
            encode_block_mono_auto(&[], DecorrProfile::Fast, 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_auto(&[], DecorrProfile::Fast, 2, 0, 0),
            Err(Error::EncodeEmptyAudio)
        ));
        assert!(matches!(
            encode_block_stereo_auto(&[1, 2, 3], DecorrProfile::Fast, 2, 0, 1),
            Err(Error::EncodeStereoOddLength(3))
        ));
    }
}
