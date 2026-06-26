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
use crate::decorrelation::{recorrelate_mono, recorrelate_stereo};
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
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }

    // The spec §5 CRC is folded over the decoded PCM — i.e. the input
    // samples themselves (the raw path carries them verbatim).
    let crc = crate::crc::crc_mono(pcm);

    let residuals = pcm.to_vec();
    // Bit 2 (mono) per the wiki "Flags meaning".
    let flags_raw = base_flags(bytes_per_sample) | (1 << 2);
    let mut metadata = Vec::new();

    // 0x05 entropy info: a single zero-seed median set.
    let entropy_payload = pack_entropy_info(&[[0, 0, 0]]);
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;

    let mut medians = AdaptiveMedians::new([0, 0, 0]);
    let packed = encode_packed_samples_mono(&residuals, &mut medians)?;
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &packed,
    )?;

    let block_samples =
        u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        crc,
    )
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
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if left_shift == 0 {
        return Err(Error::EncodeLeftShiftZero);
    }

    // Narrow the container-scaled PCM to the values the decoder
    // reconstructs *before* its final left-shift; the §5 CRC folds these.
    let mut narrow = pcm.to_vec();
    narrow_left_shift(&mut narrow, left_shift)?;
    let crc = crate::crc::crc_mono(&narrow);

    let flags_raw = with_left_shift(base_flags(bytes_per_sample) | (1 << 2), left_shift);
    let mut metadata = Vec::new();

    let entropy_payload = pack_entropy_info(&[[0, 0, 0]]);
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;

    let mut medians = AdaptiveMedians::new([0, 0, 0]);
    let packed = encode_packed_samples_mono(&narrow, &mut medians)?;
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &packed,
    )?;

    let block_samples =
        u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        crc,
    )
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
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    if left_shift == 0 {
        return Err(Error::EncodeLeftShiftZero);
    }

    let mut narrow = pcm.to_vec();
    narrow_left_shift(&mut narrow, left_shift)?;
    let crc = crate::crc::crc_stereo_interleaved(&narrow);

    let flags_raw = with_left_shift(base_flags(bytes_per_sample), left_shift);
    let mut metadata = Vec::new();

    let left_seed = [0, 0, 0];
    let right_seed = [0, 0, 1];
    let entropy_payload = pack_entropy_info(&[left_seed, right_seed]);
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;

    let mut medians = [
        AdaptiveMedians::from_seed_values(left_seed).ok_or(Error::InvalidEntropyInfoForStereo)?,
        AdaptiveMedians::from_seed_values(right_seed).ok_or(Error::InvalidEntropyInfoForStereo)?,
    ];
    let packed = encode_packed_samples_stereo(&narrow, &mut medians)?;
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &packed,
    )?;

    let block_samples =
        u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        crc,
    )
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
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    let crc = crate::crc::crc_mono(pcm);

    // Validate the payloads + build the application-ordered passes, then
    // run the forward prediction loop into the residual buffer.
    let mut passes = crate::decorrelation::assemble_mono_passes(terms, weights, samples)?;
    let mut residuals = pcm.to_vec();
    recorrelate_mono(&mut passes, &mut residuals)?;

    let flags_raw = base_flags(bytes_per_sample) | (1 << 2);
    let mut metadata = Vec::new();

    let entropy_payload = pack_entropy_info(&[[0, 0, 0]]);
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;
    // The three decorrelation sub-blocks, verbatim, in wire order.
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

    let mut medians = AdaptiveMedians::new([0, 0, 0]);
    let packed = encode_packed_samples_mono(&residuals, &mut medians)?;
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &packed,
    )?;

    let block_samples =
        u32::try_from(pcm.len()).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        crc,
    )
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
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }

    let crc = crate::crc::crc_stereo_interleaved(pcm);

    let residuals = pcm.to_vec();
    let flags_raw = base_flags(bytes_per_sample); // mono bit clear

    let mut metadata = Vec::new();

    // 0x05 entropy info: two median seed sets (left, right). The decoder
    // distinguishes a *stereo* entropy payload from a mono one by content
    // (`EntropyInfo::is_mono()` — right set all-zero reads as mono), not
    // by the 12-byte length, and refuses an all-zero right set on the
    // stereo path ([`Error::InvalidEntropyInfoForStereo`]). So the right
    // set carries a minimal non-zero seed (`[0, 0, 1]`) to mark the
    // payload stereo; the encode medians are seeded from the exact same
    // sets so the round trip stays bit-exact whatever the seed value.
    let left_seed = [0, 0, 0];
    let right_seed = [0, 0, 1];
    let entropy_payload = pack_entropy_info(&[left_seed, right_seed]);
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;

    // Seed the encode medians from the same sets written to 0x05 so the
    // encoder and decoder share an identical median start.
    let mut medians = [
        AdaptiveMedians::from_seed_values(left_seed).ok_or(Error::InvalidEntropyInfoForStereo)?,
        AdaptiveMedians::from_seed_values(right_seed).ok_or(Error::InvalidEntropyInfoForStereo)?,
    ];
    let packed = encode_packed_samples_stereo(&residuals, &mut medians)?;
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &packed,
    )?;

    let block_samples =
        u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        crc,
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
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }

    // The §5 CRC is folded over the *true* L/R PCM (the decoder undoes
    // joint stereo before the CRC step), so it is the same as the plain
    // stereo CRC over the input.
    let crc = crate::crc::crc_stereo_interleaved(pcm);

    // Forward mid/side transform into the residual buffer the entropy
    // stream carries.
    let mut residuals = pcm.to_vec();
    for pair in residuals.chunks_exact_mut(2) {
        let (mid, side) = forward_joint_stereo(pair[0], pair[1]);
        pair[0] = mid;
        pair[1] = side;
    }

    // base flags + joint-stereo bit 4.
    let flags_raw = base_flags(bytes_per_sample) | crate::crc::JOINT_STEREO_FLAG;
    let mut metadata = Vec::new();

    let left_seed = [0, 0, 0];
    let right_seed = [0, 0, 1];
    let entropy_payload = pack_entropy_info(&[left_seed, right_seed]);
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;

    let mut medians = [
        AdaptiveMedians::from_seed_values(left_seed).ok_or(Error::InvalidEntropyInfoForStereo)?,
        AdaptiveMedians::from_seed_values(right_seed).ok_or(Error::InvalidEntropyInfoForStereo)?,
    ];
    let packed = encode_packed_samples_stereo(&residuals, &mut medians)?;
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &packed,
    )?;

    let block_samples =
        u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        crc,
    )
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
    if pcm.is_empty() {
        return Err(Error::EncodeEmptyAudio);
    }
    if pcm.len() % 2 != 0 {
        return Err(Error::EncodeStereoOddLength(pcm.len()));
    }
    let crc = crate::crc::crc_stereo_interleaved(pcm);

    let mut passes = crate::decorrelation::assemble_stereo_passes(terms, weights, samples)?;
    let mut residuals = pcm.to_vec();
    recorrelate_stereo(&mut passes, &mut residuals)?;

    let flags_raw = base_flags(bytes_per_sample);
    let mut metadata = Vec::new();

    let left_seed = [0, 0, 0];
    let right_seed = [0, 0, 1];
    let entropy_payload = pack_entropy_info(&[left_seed, right_seed]);
    append_sub_block(
        &mut metadata,
        SubBlockId::EntropyInfo.as_id_byte(),
        &entropy_payload,
    )?;
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

    let mut medians = [
        AdaptiveMedians::from_seed_values(left_seed).ok_or(Error::InvalidEntropyInfoForStereo)?,
        AdaptiveMedians::from_seed_values(right_seed).ok_or(Error::InvalidEntropyInfoForStereo)?,
    ];
    let packed = encode_packed_samples_stereo(&residuals, &mut medians)?;
    append_sub_block(
        &mut metadata,
        SubBlockId::PackedSamples.as_id_byte(),
        &packed,
    )?;

    let block_samples =
        u32::try_from(pcm.len() / 2).map_err(|_| Error::EncodeBlockTooLarge(pcm.len()))?;
    build_block(
        metadata,
        block_index,
        total_samples,
        block_samples,
        flags_raw,
        crc,
    )
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
}
