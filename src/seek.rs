//! Stream indexing and sample-accurate seeking.
//!
//! A WavPack file is a chain of self-describing `wvpk` blocks, each of
//! whose 32-byte fixed header carries the two fields random access
//! needs (wiki "Block structure" listing): the block's absolute sample
//! offset ("offset in samples for current block — how many samples
//! should be decoded by now", the `block_index` word) and its sample
//! count ("samples in this block", the `block_samples` word). Because
//! every block also declares its own byte length (`ck_size`), the whole
//! stream can be mapped **without decoding any audio**: a header-only
//! walk yields one [`IndexEntry`] per block in O(blocks) time.
//!
//! Seeking is defined in the *frame* domain of
//! [`crate::DecodedStream`]: one frame is one interleaved sample across
//! all channels. For plain mono / stereo files every block is its own
//! single-block set (wiki bits 11..=12 both set); for multichannel
//! files each frame range is split across a member *set* (a bit-11
//! member opens it, a bit-12 member closes it). The index therefore
//! groups its audio entries into [`SetEntry`] units — the smallest
//! independently-decodable frame ranges — mirroring the exact grouping
//! rules [`crate::decode_multichannel_stream`] applies (same typed
//! refusals for malformed grouping), so **any stream that decoder
//! accepts can be indexed**.
//!
//! [`StreamIndex::is_seekable`] reports whether the sets form one
//! contiguous ascending frame chain (each set starting exactly where
//! the previous ended). On a seekable index,
//! [`StreamIndex::locate_frame`] answers "which set covers frame `n`"
//! by binary search, and the higher layers ([`decode_range`] /
//! [`StreamReader`](crate::StreamReader)) decode only the sets a
//! requested window touches.

use crate::block::MAX_MULTICHANNEL_CHANNELS;
use crate::block_header::{parse_block_header, Flags};
use crate::error::{Error, Result};

/// Header-level index record for one `wvpk` block.
///
/// Produced by [`StreamIndex::scan`] from the 32-byte fixed header
/// alone — no metadata sub-block is parsed and no audio is decoded.
/// The byte span lets a caller slice the original buffer (or issue a
/// ranged read) to re-parse exactly this block later.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IndexEntry {
    /// Byte offset of the block's `wvpk` magic within the scanned
    /// buffer.
    pub byte_offset: usize,
    /// On-disk length of the whole block: `8 + ck_size` (the wiki
    /// `ck_size` excludes the magic and the size field itself).
    pub byte_len: usize,
    /// The header's absolute sample offset (wiki "offset in samples
    /// for current block").
    pub block_index: u32,
    /// The header's sample count (wiki "samples in this block"; `0`
    /// for a metadata-only block).
    pub block_samples: u32,
    /// The decoded 32-bit flag word, preserved so shape predicates
    /// (mono / false-stereo, grouping markers, hybrid bit, …) are
    /// answerable without re-parsing.
    pub flags: Flags,
}

impl IndexEntry {
    /// `true` when this block carries audio samples (`block_samples !=
    /// 0`), mirroring
    /// [`WavPackBlockHeader::is_audio_block`](crate::WavPackBlockHeader::is_audio_block).
    pub fn is_audio(&self) -> bool {
        self.block_samples != 0
    }

    /// The block's byte span within the scanned buffer
    /// (`byte_offset .. byte_offset + byte_len`).
    pub fn byte_range(&self) -> core::ops::Range<usize> {
        self.byte_offset..self.byte_offset + self.byte_len
    }

    /// Number of decoded channels this block contributes as a
    /// multichannel-set member: `1` for true mono (flag bit 2), `2`
    /// for interleaved stereo AND for false-stereo (bit 30 — one coded
    /// channel duplicated to both outputs by the decoder; round 408).
    pub fn member_channels(&self) -> usize {
        if self.flags.mono {
            1
        } else {
            2
        }
    }
}

/// One decodable frame range: a multichannel member set (or the
/// degenerate single-block set every plain mono / stereo block forms).
///
/// The set is the smallest unit random access can decode independently:
/// all its members cover the same `frames` frame range, and its
/// channels are the members' channels concatenated in wire order —
/// exactly the interleave [`crate::decode_multichannel_stream`]
/// produces for the same members.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetEntry {
    /// Absolute frame index of the set's first frame (the first
    /// member's `block_index` header word).
    pub first_frame: u32,
    /// Number of frames the set covers (the members' shared
    /// `block_samples`).
    pub frames: u32,
    /// Summed per-frame channel count across the members.
    pub channels: usize,
    /// Indices into [`StreamIndex::entries`] of the member blocks, in
    /// wire order. Metadata-only blocks interleaved with the members
    /// are not listed (they carry no channels).
    members: Vec<usize>,
}

impl SetEntry {
    /// Absolute frame index one past the set's last frame.
    pub fn end_frame(&self) -> u64 {
        u64::from(self.first_frame) + u64::from(self.frames)
    }

    /// `true` when the absolute frame `frame` falls inside this set's
    /// `[first_frame, end_frame)` range.
    pub fn contains_frame(&self, frame: u64) -> bool {
        frame >= u64::from(self.first_frame) && frame < self.end_frame()
    }

    /// Indices into [`StreamIndex::entries`] of this set's member
    /// blocks, in wire order.
    pub fn member_entries(&self) -> &[usize] {
        &self.members
    }
}

/// Header-only map of a WavPack stream: every block's byte span and
/// sample range, grouped into decodable member sets.
///
/// Built by [`StreamIndex::scan`] in one O(blocks) pass that reads only
/// each block's 32-byte fixed header (metadata sub-blocks are skipped
/// via the header's own `ck_size`). The scan applies the same
/// stream-shape refusals [`crate::decode_multichannel_stream`] applies
/// — malformed grouping markers
/// ([`Error::MultichannelSetMalformed`]), per-member `block_samples`
/// disagreement ([`Error::MultichannelSampleCountMismatch`]), channel
/// blow-up ([`Error::MultichannelTooManyChannels`]) and inter-set
/// channel-width disagreement — so indexing succeeds for every stream
/// that decoder accepts, and refuses the same malformed shapes with the
/// same typed errors.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamIndex {
    entries: Vec<IndexEntry>,
    sets: Vec<SetEntry>,
    channels: usize,
    seekable: bool,
}

impl StreamIndex {
    /// Build a [`StreamIndex`] over a complete WavPack byte buffer.
    ///
    /// Walks the block chain header-by-header: each 32-byte fixed
    /// header is parsed ([`parse_block_header`] refusals — bad magic,
    /// bad `ck_size`, unsupported version, truncation — propagate
    /// verbatim), the block's payload is skipped via `ck_size`, and a
    /// buffer that ends mid-payload raises
    /// [`Error::CkSizeExceedsBuffer`] with the same field semantics as
    /// [`crate::parse_block`]. Audio blocks are grouped into member
    /// sets per the wiki bits-11..=12 grouping rules (see the type
    /// docs); metadata-only blocks are indexed but join no set.
    ///
    /// The scan never parses a metadata sub-block and never decodes
    /// audio, so it is cheap even for large files and succeeds on
    /// streams whose *payloads* are malformed — payload errors surface
    /// later, from the decode layer, exactly as they do for the
    /// whole-stream decoders.
    pub fn scan(bytes: &[u8]) -> Result<Self> {
        let mut entries: Vec<IndexEntry> = Vec::new();
        let mut sets: Vec<SetEntry> = Vec::new();
        let mut stream_channels: Option<usize> = None;

        // The currently-open member set, if any.
        let mut open_members: Vec<usize> = Vec::new();
        let mut open_channels: usize = 0;
        let mut open_first_frame: u32 = 0;
        let mut open_frames: u32 = 0;
        let mut set_open = false;

        let mut offset = 0usize;
        while offset < bytes.len() {
            let (header, _tail) = parse_block_header(&bytes[offset..])?;
            let byte_len = 8usize + header.ck_size as usize;
            if offset + byte_len > bytes.len() {
                // Same distinction parse_block draws: the fixed header
                // parsed cleanly but the advertised payload extends
                // past the buffer.
                return Err(Error::CkSizeExceedsBuffer {
                    ck_size: header.ck_size,
                    available: bytes.len() - offset,
                });
            }
            let entry = IndexEntry {
                byte_offset: offset,
                byte_len,
                block_index: header.block_index,
                block_samples: header.block_samples,
                flags: header.flags,
            };
            let entry_idx = entries.len();
            entries.push(entry);
            offset += byte_len;

            if !entry.is_audio() {
                // Metadata-only block: indexed, but never a set member
                // (mirrors the decoder's skip).
                continue;
            }

            let is_first = entry.flags.is_first_block();
            let is_final = entry.flags.is_final_block();

            if is_first {
                if set_open {
                    // Previous set never saw its final marker.
                    return Err(Error::MultichannelSetMalformed);
                }
                set_open = true;
                open_members = Vec::new();
                open_channels = 0;
                open_first_frame = entry.block_index;
                open_frames = entry.block_samples;
            } else if !set_open {
                // Continuation / final marker with no open set.
                return Err(Error::MultichannelSetMalformed);
            }

            if entry.block_samples != open_frames {
                return Err(Error::MultichannelSampleCountMismatch {
                    expected: open_frames,
                    found: entry.block_samples,
                });
            }

            let member_channels = entry.member_channels();
            if open_channels + member_channels > MAX_MULTICHANNEL_CHANNELS {
                return Err(Error::MultichannelTooManyChannels(
                    open_channels + member_channels,
                ));
            }
            open_channels += member_channels;
            open_members.push(entry_idx);

            if is_final {
                // Close the set; enforce inter-set channel-width
                // agreement exactly as the whole-stream decoder does.
                match stream_channels {
                    None => stream_channels = Some(open_channels),
                    Some(prev) if prev != open_channels => {
                        return Err(Error::MultichannelSetMalformed);
                    }
                    Some(_) => {}
                }
                sets.push(SetEntry {
                    first_frame: open_first_frame,
                    frames: open_frames,
                    channels: open_channels,
                    members: core::mem::take(&mut open_members),
                });
                set_open = false;
                open_channels = 0;
                open_frames = 0;
            }
        }

        if set_open {
            // Stream ended mid-set.
            return Err(Error::MultichannelSetMalformed);
        }

        // Seekable = the sets form one contiguous ascending frame
        // chain: each set starts exactly where the previous ended.
        // (Uniform channel width is already enforced above.)
        let seekable = sets
            .windows(2)
            .all(|w| w[1].first_frame as u64 == w[0].end_frame());

        Ok(Self {
            entries,
            sets,
            channels: stream_channels.unwrap_or(0),
            seekable,
        })
    }

    /// Every indexed block, in wire order (audio and metadata-only
    /// alike).
    pub fn entries(&self) -> &[IndexEntry] {
        &self.entries
    }

    /// The audio member sets, in wire order.
    pub fn sets(&self) -> &[SetEntry] {
        &self.sets
    }

    /// Total number of indexed blocks.
    pub fn block_count(&self) -> usize {
        self.entries.len()
    }

    /// Number of indexed **audio** blocks (`block_samples != 0`).
    pub fn audio_block_count(&self) -> usize {
        self.entries.iter().filter(|e| e.is_audio()).count()
    }

    /// Number of audio member sets.
    pub fn set_count(&self) -> usize {
        self.sets.len()
    }

    /// Per-frame channel count of the stream (`0` when the stream has
    /// no audio sets). All sets share this width — the scan refuses
    /// disagreement, mirroring [`crate::decode_multichannel_stream`].
    pub fn channels(&self) -> usize {
        self.channels
    }

    /// `true` when the index holds no audio sets.
    pub fn is_empty(&self) -> bool {
        self.sets.is_empty()
    }

    /// Absolute frame index of the stream's first frame (the first
    /// set's `first_frame`; `0` for an empty index).
    pub fn first_frame(&self) -> u64 {
        self.sets.first().map_or(0, |s| u64::from(s.first_frame))
    }

    /// Absolute frame index one past the stream's last frame (equal to
    /// [`Self::first_frame`] for an empty index).
    pub fn end_frame(&self) -> u64 {
        self.sets.last().map_or(0, SetEntry::end_frame)
    }

    /// Total number of frames across all sets. On a seekable index
    /// this equals `end_frame() - first_frame()`; on a non-seekable
    /// index (gaps / overlaps) it is still the exact frame count a
    /// whole-stream decode would emit, because the decoders emit every
    /// set's frames regardless of the header frame numbering.
    pub fn frame_count(&self) -> u64 {
        self.sets.iter().map(|s| u64::from(s.frames)).sum()
    }

    /// `true` when the sets form one contiguous ascending frame chain
    /// (each set starting exactly where the previous ended), the
    /// property [`Self::locate_frame`] and the range decoders require
    /// so a frame number maps to exactly one set. Trivially `true` for
    /// zero or one set.
    pub fn is_seekable(&self) -> bool {
        self.seekable
    }

    /// Locate the set covering the absolute frame index `frame`.
    ///
    /// Returns the index into [`Self::sets`], or `None` when `frame`
    /// falls outside `[first_frame, end_frame)` **or** the index is
    /// not seekable (without the contiguous-chain property a frame
    /// number does not map to a unique set).
    pub fn locate_frame(&self, frame: u64) -> Option<usize> {
        if !self.seekable {
            return None;
        }
        // First set whose end is past the frame; contiguity makes
        // this the unique candidate.
        let idx = self.sets.partition_point(|s| s.end_frame() <= frame);
        (idx < self.sets.len() && self.sets[idx].contains_frame(frame)).then_some(idx)
    }

    /// The [`SetEntry`] covering the absolute frame index `frame`
    /// (the entry-returning twin of [`Self::locate_frame`]).
    pub fn set_for_frame(&self, frame: u64) -> Option<&SetEntry> {
        self.locate_frame(frame).map(|i| &self.sets[i])
    }

    /// Byte span of the scanned buffer covering **all** of set
    /// `set_idx`'s member blocks (first member's start to last
    /// member's end — any metadata-only blocks interleaved between
    /// members fall inside the span). `None` for an out-of-range set
    /// index.
    ///
    /// This is the ranged-read a caller doing partial-file IO needs
    /// to decode the set without the rest of the stream.
    pub fn set_byte_span(&self, set_idx: usize) -> Option<core::ops::Range<usize>> {
        let set = self.sets.get(set_idx)?;
        let first = self.entries[*set.members.first()?];
        let last = self.entries[*set.members.last()?];
        Some(first.byte_offset..last.byte_offset + last.byte_len)
    }
}

/// Decode one member set into its interleaved PCM frames.
///
/// `bytes` must be the same buffer `index` was scanned from — each
/// member block is re-parsed at its recorded byte span and decoded via
/// [`crate::WavPackBlock::decode_member_samples`] (or the CRC-muted
/// twin when `muted`), then the members' channels are interleaved per
/// frame in wire order, exactly as
/// [`crate::decode_multichannel_stream`] interleaves the same set.
/// Returns `(pcm, all_crc_ok)`; `all_crc_ok` is only meaningful in
/// muted mode (`true` otherwise).
///
/// A buffer that no longer matches the index (shorter, or reshaped
/// blocks) surfaces as [`Error::Truncated`] / parse errors / the
/// grouping refusals — never as an out-of-bounds access.
fn decode_set(
    bytes: &[u8],
    index: &StreamIndex,
    set: &SetEntry,
    muted: bool,
) -> Result<(Vec<i32>, bool)> {
    let mut channels: Vec<Vec<i32>> = Vec::with_capacity(set.channels);
    let mut all_crc_ok = true;
    for &member_idx in set.member_entries() {
        let entry = &index.entries()[member_idx];
        let slice = bytes.get(entry.byte_range()).ok_or(Error::Truncated)?;
        let (block, _tail) = crate::block::parse_block(slice)?;
        // Consistency guards against a caller passing a different
        // buffer than the one scanned: the re-parsed shape must match
        // the indexed shape.
        if block.header().block_samples != set.frames {
            return Err(Error::MultichannelSampleCountMismatch {
                expected: set.frames,
                found: block.header().block_samples,
            });
        }
        let member_channels = if block.flags().mono { 1 } else { 2 };
        if member_channels != entry.member_channels() {
            return Err(Error::MultichannelSetMalformed);
        }
        let pcm = if muted {
            let (pcm, crc_ok) = block.decode_member_samples_muted()?;
            all_crc_ok &= crc_ok;
            pcm
        } else {
            block.decode_member_samples()?
        };
        if member_channels == 1 {
            channels.push(pcm);
        } else {
            let frames = pcm.len() / 2;
            let mut left = Vec::with_capacity(frames);
            let mut right = Vec::with_capacity(frames);
            for pair in pcm.chunks_exact(2) {
                left.push(pair[0]);
                right.push(pair[1]);
            }
            channels.push(left);
            channels.push(right);
        }
    }
    let frames = set.frames as usize;
    let mut out = Vec::with_capacity(frames * set.channels);
    for f in 0..frames {
        for ch in &channels {
            out.push(ch[f]);
        }
    }
    Ok((out, all_crc_ok))
}

/// Locate and parse the correction (`.wvc`) counterpart members of the
/// main-stream set `set`, on the correction buffer's own header-only
/// index (round 415).
///
/// The `.wv` and `.wvc` files have identical block structure, so a main
/// set's counterpart is the correction set covering the **same frame
/// range** — found here by `first_frame` / `frames` equality (partial
/// `.wvc` coverage simply has no counterpart: `Ok(None)`, and the
/// caller decodes the set lossy). A counterpart with a different member
/// count, or a member disagreeing on the mono flag, is refused with
/// [`Error::CorrectionShapeMismatch`] — the same agreement rules
/// [`crate::pair_correction_stream`] enforces per pair.
fn correction_set_members<'b>(
    corr_bytes: &'b [u8],
    corr_index: &StreamIndex,
    index: &StreamIndex,
    set: &SetEntry,
) -> Result<Option<Vec<crate::WavPackBlock<'b>>>> {
    let Some(corr_set) = corr_index
        .sets()
        .iter()
        .find(|s| s.first_frame == set.first_frame && s.frames == set.frames)
    else {
        return Ok(None);
    };
    if corr_set.member_entries().len() != set.member_entries().len() {
        return Err(Error::CorrectionShapeMismatch(set.first_frame));
    }
    let mut members = Vec::with_capacity(corr_set.member_entries().len());
    for (&corr_member, &main_member) in corr_set.member_entries().iter().zip(set.member_entries()) {
        let corr_entry = &corr_index.entries()[corr_member];
        let main_entry = &index.entries()[main_member];
        if corr_entry.flags.mono != main_entry.flags.mono {
            return Err(Error::CorrectionShapeMismatch(set.first_frame));
        }
        let slice = corr_bytes
            .get(corr_entry.byte_range())
            .ok_or(Error::Truncated)?;
        let (block, _tail) = crate::block::parse_block(slice)?;
        members.push(block);
    }
    Ok(Some(members))
}

/// [`decode_set`] with an optional `.wvc` correction source (round
/// 415): when the correction index carries this set's counterpart,
/// every member decodes hybrid-lossless
/// ([`crate::WavPackBlock::decode_samples_with_correction`], CRC-gated
/// against the `.wvc` header's lossless CRC in muted mode); otherwise
/// the set decodes exactly as [`decode_set`] does.
fn decode_set_with_correction(
    bytes: &[u8],
    index: &StreamIndex,
    set: &SetEntry,
    correction: Option<(&[u8], &StreamIndex)>,
    muted: bool,
) -> Result<(Vec<i32>, bool)> {
    let twins = match correction {
        Some((corr_bytes, corr_index)) => {
            correction_set_members(corr_bytes, corr_index, index, set)?
        }
        None => None,
    };
    let Some(twins) = twins else {
        return decode_set(bytes, index, set, muted);
    };

    let mut channels: Vec<Vec<i32>> = Vec::with_capacity(set.channels);
    let mut all_crc_ok = true;
    for (&member_idx, twin) in set.member_entries().iter().zip(&twins) {
        let entry = &index.entries()[member_idx];
        let slice = bytes.get(entry.byte_range()).ok_or(Error::Truncated)?;
        let (block, _tail) = crate::block::parse_block(slice)?;
        if block.header().block_samples != set.frames {
            return Err(Error::MultichannelSampleCountMismatch {
                expected: set.frames,
                found: block.header().block_samples,
            });
        }
        let member_channels = if block.flags().mono { 1 } else { 2 };
        if member_channels != entry.member_channels() {
            return Err(Error::MultichannelSetMalformed);
        }
        let pcm = if muted {
            let (pcm, crc_ok) = block.decode_samples_with_correction_muted(twin)?;
            all_crc_ok &= crc_ok;
            pcm
        } else {
            block.decode_samples_with_correction(twin)?
        };
        if member_channels == 1 {
            channels.push(pcm);
        } else {
            let frames = pcm.len() / 2;
            let mut left = Vec::with_capacity(frames);
            let mut right = Vec::with_capacity(frames);
            for pair in pcm.chunks_exact(2) {
                left.push(pair[0]);
                right.push(pair[1]);
            }
            channels.push(left);
            channels.push(right);
        }
    }
    let frames = set.frames as usize;
    let mut out = Vec::with_capacity(frames * set.channels);
    for f in 0..frames {
        for ch in &channels {
            out.push(ch[f]);
        }
    }
    Ok((out, all_crc_ok))
}

/// Shared range-decode core for [`decode_range`] (`muted == false`)
/// and [`decode_range_muted`].
fn decode_range_inner(
    bytes: &[u8],
    index: &StreamIndex,
    start_frame: u64,
    frames: u64,
    muted: bool,
) -> Result<(Vec<i32>, bool)> {
    decode_range_pair_inner(bytes, index, None, start_frame, frames, muted)
}

/// Range-decode core with an optional correction source (round 415).
fn decode_range_pair_inner(
    bytes: &[u8],
    index: &StreamIndex,
    correction: Option<(&[u8], &StreamIndex)>,
    start_frame: u64,
    frames: u64,
    muted: bool,
) -> Result<(Vec<i32>, bool)> {
    if !index.is_seekable() {
        return Err(Error::StreamNotSeekable);
    }
    if frames == 0 {
        return Ok((Vec::new(), true));
    }
    let end_frame = start_frame
        .checked_add(frames)
        .ok_or(Error::SeekOutOfRange {
            requested: u64::MAX,
            first_frame: index.first_frame(),
            end_frame: index.end_frame(),
        })?;
    if start_frame < index.first_frame() || end_frame > index.end_frame() {
        return Err(Error::SeekOutOfRange {
            requested: if start_frame < index.first_frame() {
                start_frame
            } else {
                end_frame
            },
            first_frame: index.first_frame(),
            end_frame: index.end_frame(),
        });
    }
    let mut set_idx = index
        .locate_frame(start_frame)
        .expect("in-range frame on a seekable index locates a set");
    let channels = index.channels();
    let mut out: Vec<i32> = Vec::with_capacity(usize::try_from(frames).unwrap_or(0) * channels);
    let mut all_crc_ok = true;
    let mut cursor = start_frame;
    while cursor < end_frame {
        let set = &index.sets()[set_idx];
        let (pcm, crc_ok) = decode_set_with_correction(bytes, index, set, correction, muted)?;
        all_crc_ok &= crc_ok;
        // Overlap of [cursor, end_frame) with this set, in set-local
        // frame offsets.
        let from = (cursor - u64::from(set.first_frame)) as usize;
        let upto = (end_frame.min(set.end_frame()) - u64::from(set.first_frame)) as usize;
        out.extend_from_slice(&pcm[from * channels..upto * channels]);
        cursor = set.end_frame().min(end_frame);
        set_idx += 1;
    }
    Ok((out, all_crc_ok))
}

/// Decode an arbitrary frame window `[start_frame, start_frame +
/// frames)` from an indexed WavPack stream, touching only the member
/// sets the window overlaps.
///
/// `bytes` must be the buffer `index` was scanned from ([`StreamIndex::scan`]).
/// Frame indices are **absolute** (the wiki `block_index` domain — for
/// a normal file the first frame is `0`; see
/// [`StreamIndex::first_frame`]). The result is interleaved PCM,
/// `frames * index.channels()` values, bit-exactly equal to the same
/// window sliced out of the whole-stream
/// [`crate::decode_multichannel_stream`] output (which for plain mono /
/// stereo files equals [`crate::decode_stream`]).
///
/// Refusals: [`Error::StreamNotSeekable`] when the index's sets do not
/// form a contiguous frame chain, [`Error::SeekOutOfRange`] when the
/// window falls outside `[first_frame, end_frame)`. Per-set parse /
/// decode errors propagate verbatim. `frames == 0` yields an empty
/// buffer.
pub fn decode_range(
    bytes: &[u8],
    index: &StreamIndex,
    start_frame: u64,
    frames: u64,
) -> Result<Vec<i32>> {
    decode_range_inner(bytes, index, start_frame, frames, false).map(|(pcm, _)| pcm)
}

/// The spec §5.6 CRC-mute twin of [`decode_range`]: every member block
/// the window touches is decoded through the per-member CRC gate
/// ([`crate::WavPackBlock::decode_member_samples_muted`]), so a member
/// whose stored CRC mismatches contributes zeros in its channel slots
/// instead of failing the decode.
///
/// Returns `(pcm, all_crc_ok)` where `all_crc_ok` covers **only the
/// member blocks the window touched** — a corrupt block outside the
/// window is not decoded and therefore not reported. A zero-length
/// window reports `true`.
pub fn decode_range_muted(
    bytes: &[u8],
    index: &StreamIndex,
    start_frame: u64,
    frames: u64,
) -> Result<(Vec<i32>, bool)> {
    decode_range_inner(bytes, index, start_frame, frames, true)
}

/// [`decode_range`] over a hybrid-lossless `.wv` + `.wvc` **pair**
/// (round 415): the window's member sets decode losslessly by pairing
/// each set with the correction buffer's counterpart set (matched by
/// frame range on a header-only [`StreamIndex::scan`] of `correction`,
/// run internally per call — no audio decode outside the window). Sets
/// the correction chain does not cover fall back to their coarse lossy
/// decode, matching [`crate::decode_stream_with_correction`]'s partial
/// coverage posture. The result is bit-exactly the same window sliced
/// from [`crate::decode_multichannel_stream_with_correction`]'s
/// whole-stream output.
pub fn decode_range_with_correction(
    bytes: &[u8],
    index: &StreamIndex,
    correction: &[u8],
    start_frame: u64,
    frames: u64,
) -> Result<Vec<i32>> {
    let corr_index = StreamIndex::scan(correction)?;
    decode_range_pair_inner(
        bytes,
        index,
        Some((correction, &corr_index)),
        start_frame,
        frames,
        false,
    )
    .map(|(pcm, _)| pcm)
}

/// The spec §5.6 CRC-mute twin of [`decode_range_with_correction`]:
/// every paired member the window touches is gated against its `.wvc`
/// header's stored **lossless** CRC (round-415 pin), unpaired members
/// against their own `.wv` header CRC; a failing member contributes
/// zeros. `all_crc_ok` covers only the members the window touched.
pub fn decode_range_with_correction_muted(
    bytes: &[u8],
    index: &StreamIndex,
    correction: &[u8],
    start_frame: u64,
    frames: u64,
) -> Result<(Vec<i32>, bool)> {
    let corr_index = StreamIndex::scan(correction)?;
    decode_range_pair_inner(
        bytes,
        index,
        Some((correction, &corr_index)),
        start_frame,
        frames,
        true,
    )
}

/// A seekable decoding cursor over an indexed WavPack stream.
///
/// Wraps a byte buffer plus its [`StreamIndex`] and exposes the
/// classic reader trio — [`StreamReader::seek`],
/// [`StreamReader::read_frames`], [`StreamReader::position`] — in the
/// absolute frame domain (see [`StreamIndex::first_frame`]; `0` for a
/// normal file). Reads decode whole member sets and cache the most
/// recently decoded one, so sequential small reads inside one set (the
/// common playback pattern) decode each set exactly once, and a
/// seek-back within the cached set costs nothing.
///
/// Construction refuses a non-seekable stream
/// ([`Error::StreamNotSeekable`]) — whole-stream decoding via
/// [`crate::decode_stream`] / [`crate::decode_multichannel_stream`]
/// remains available for those. An empty (no-audio) stream is
/// trivially seekable: `seek(0)` succeeds and reads return no frames.
#[derive(Debug, Clone)]
pub struct StreamReader<'a> {
    bytes: &'a [u8],
    index: StreamIndex,
    /// Companion `.wvc` correction buffer + its header-only index
    /// (round 415): when present, every set whose counterpart the
    /// correction chain carries decodes hybrid-lossless.
    correction: Option<(&'a [u8], StreamIndex)>,
    /// Absolute frame index of the next frame a read will return.
    position: u64,
    /// Most recently decoded set.
    cache: Option<CachedSet>,
}

/// The [`StreamReader`] set cache: one decoded set plus the mode it
/// was decoded in, so a cached buffer never leaks across plain / muted
/// reads with different contents (the mute gate zeros a
/// CRC-mismatching member, so the two modes agree only when every
/// member's CRC matched).
#[derive(Debug, Clone)]
struct CachedSet {
    set_idx: usize,
    pcm: Vec<i32>,
    /// `true` when [`CachedSet::pcm`] came from a CRC-gated (muted)
    /// decode.
    muted: bool,
    /// Whether every member's CRC matched — only meaningful when
    /// [`CachedSet::muted`] is set.
    crc_ok: bool,
}

impl<'a> StreamReader<'a> {
    /// Scan `bytes` ([`StreamIndex::scan`]) and open a cursor
    /// positioned at the stream's first frame.
    ///
    /// Scan refusals propagate verbatim; a stream whose sets do not
    /// form a contiguous ascending frame chain raises
    /// [`Error::StreamNotSeekable`].
    pub fn new(bytes: &'a [u8]) -> Result<Self> {
        Self::with_index(bytes, StreamIndex::scan(bytes)?)
    }

    /// Open a cursor over a hybrid-lossless `.wv` + `.wvc` **pair**
    /// (round 415): reads decode each member set losslessly whenever
    /// the correction chain carries its counterpart (matched by frame
    /// range on a header-only scan of `correction`, done once here),
    /// falling back to the coarse lossy decode for uncovered sets —
    /// the seek-shaped twin of
    /// [`crate::decode_stream_with_correction`]. Muted reads gate
    /// paired members against the `.wvc` header's stored lossless CRC.
    pub fn new_with_correction(bytes: &'a [u8], correction: &'a [u8]) -> Result<Self> {
        let mut reader = Self::with_index(bytes, StreamIndex::scan(bytes)?)?;
        reader.correction = Some((correction, StreamIndex::scan(correction)?));
        Ok(reader)
    }

    /// Open a cursor over an already-scanned index. `bytes` must be
    /// the buffer `index` was scanned from.
    pub fn with_index(bytes: &'a [u8], index: StreamIndex) -> Result<Self> {
        if !index.is_seekable() {
            return Err(Error::StreamNotSeekable);
        }
        let position = index.first_frame();
        Ok(Self {
            bytes,
            index,
            correction: None,
            position,
            cache: None,
        })
    }

    /// The underlying index (byte spans, sets, frame ranges).
    pub fn index(&self) -> &StreamIndex {
        &self.index
    }

    /// Per-frame channel count of every read (`0` for a no-audio
    /// stream).
    pub fn channels(&self) -> usize {
        self.index.channels()
    }

    /// Absolute frame index of the next frame a read will return.
    pub fn position(&self) -> u64 {
        self.position
    }

    /// Number of frames between the cursor and the end of the stream.
    pub fn frames_remaining(&self) -> u64 {
        self.index.end_frame() - self.position
    }

    /// `true` when the cursor is at the end of the stream (a read
    /// returns no frames).
    pub fn is_at_end(&self) -> bool {
        self.position == self.index.end_frame()
    }

    /// The stream's sample rate in Hz (standard-rate table index or
    /// the `0x27` non-standard rate — [`crate::stream_sample_rate`]),
    /// or `None` when unknown. Round 405.
    pub fn sample_rate(&self) -> Result<Option<u32>> {
        crate::stream_sample_rate(self.bytes)
    }

    /// Move the cursor to the frame nearest `seconds` (time-addressed
    /// seek): `frame = round(seconds * sample_rate)`, clamped to the
    /// stream's frame range like a plain [`Self::seek`].
    ///
    /// Requires the stream's sample rate to be resolvable
    /// ([`Self::sample_rate`]); a custom-rate stream missing its
    /// `0x27` sub-block is refused with [`Error::SampleRateUnknown`],
    /// and a negative / non-finite `seconds` with
    /// [`Error::SeekOutOfRange`] (frame domain bounds). Round 405.
    pub fn seek_seconds(&mut self, seconds: f64) -> Result<()> {
        let Some(rate) = self.sample_rate()? else {
            return Err(Error::SampleRateUnknown);
        };
        if !seconds.is_finite() || seconds < 0.0 {
            return Err(Error::SeekOutOfRange {
                requested: u64::MAX,
                first_frame: self.index.first_frame(),
                end_frame: self.index.end_frame(),
            });
        }
        let frame = (seconds * f64::from(rate)).round();
        // Clamp the continuous time domain onto the discrete frame
        // range; the plain seek re-validates.
        let frame = if frame >= self.index.end_frame() as f64 {
            self.index.end_frame()
        } else {
            frame as u64
        };
        self.seek(frame.max(self.index.first_frame()))
    }

    /// Move the cursor to the absolute frame index `frame`.
    ///
    /// Any position in `[first_frame, end_frame]` is accepted —
    /// seeking **to** the end is a valid cursor state from which reads
    /// return no frames. Outside that range the seek is refused with
    /// [`Error::SeekOutOfRange`] and the cursor is unchanged. Seeking
    /// never decodes; the set cache is kept (a seek back into the
    /// cached set stays free).
    pub fn seek(&mut self, frame: u64) -> Result<()> {
        if frame < self.index.first_frame() || frame > self.index.end_frame() {
            return Err(Error::SeekOutOfRange {
                requested: frame,
                first_frame: self.index.first_frame(),
                end_frame: self.index.end_frame(),
            });
        }
        self.position = frame;
        Ok(())
    }

    /// Decode and return up to `max_frames` interleaved frames from
    /// the cursor, advancing it by the frames returned.
    ///
    /// Returns `max_frames * channels` PCM values, or fewer when the
    /// stream ends first (an empty buffer at end-of-stream). A failed
    /// read returns no frames and leaves the cursor **unchanged**, so
    /// a caller that repairs the buffer (or narrows the request to the
    /// intact region) retries from the same position.
    pub fn read_frames(&mut self, max_frames: usize) -> Result<Vec<i32>> {
        self.read_frames_inner(max_frames, false)
            .map(|(pcm, _)| pcm)
    }

    /// The spec §5.6 CRC-mute twin of [`StreamReader::read_frames`]:
    /// member blocks are decoded through the per-member CRC gate, a
    /// mismatching member contributes zeros in its channel slots, and
    /// the returned flag is `true` only when every member decoded for
    /// **this read** matched. (A set decoded from the cache reports
    /// the CRC state observed when it was decoded.)
    pub fn read_frames_muted(&mut self, max_frames: usize) -> Result<(Vec<i32>, bool)> {
        self.read_frames_inner(max_frames, true)
    }

    fn read_frames_inner(&mut self, max_frames: usize, muted: bool) -> Result<(Vec<i32>, bool)> {
        // All-or-nothing cursor contract: a failed read restores the
        // starting position so no frames are silently skipped.
        let start_position = self.position;
        self.read_frames_advance(max_frames, muted)
            .inspect_err(|_| self.position = start_position)
    }

    fn read_frames_advance(&mut self, max_frames: usize, muted: bool) -> Result<(Vec<i32>, bool)> {
        let channels = self.channels();
        let mut out: Vec<i32> = Vec::new();
        let mut all_crc_ok = true;
        let mut remaining = max_frames as u64;
        while remaining > 0 && self.position < self.index.end_frame() {
            let set_idx = self
                .index
                .locate_frame(self.position)
                .expect("cursor inside coverage on a seekable index");
            // A cached set is reusable when it was decoded in the same
            // mode; across modes only a muted-and-CRC-clean buffer is
            // byte-identical to the plain decode (the mute gate only
            // changes the output by zeroing a mismatching member).
            let reusable = match &self.cache {
                Some(c) if c.set_idx == set_idx => {
                    if c.muted == muted {
                        true
                    } else {
                        // Cross-mode: muted-clean == plain; a plain
                        // cache never serves a muted read (no CRC
                        // verdict was recorded).
                        c.muted && c.crc_ok
                    }
                }
                _ => false,
            };
            if !reusable {
                let correction = self
                    .correction
                    .as_ref()
                    .map(|(bytes, index)| (*bytes, index));
                let (pcm, crc_ok) = decode_set_with_correction(
                    self.bytes,
                    &self.index,
                    &self.index.sets()[set_idx],
                    correction,
                    muted,
                )?;
                self.cache = Some(CachedSet {
                    set_idx,
                    pcm,
                    muted,
                    crc_ok,
                });
            }
            let cached = self.cache.as_ref().expect("cache populated above");
            if muted {
                all_crc_ok &= cached.crc_ok;
            }
            let set = &self.index.sets()[set_idx];
            let from = (self.position - u64::from(set.first_frame)) as usize;
            let want = remaining.min(set.end_frame() - self.position) as usize;
            out.extend_from_slice(&cached.pcm[from * channels..(from + want) * channels]);
            self.position += want as u64;
            remaining -= want as u64;
        }
        Ok((out, all_crc_ok))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::block_header::MAGIC;
    use crate::encode::{
        encode_block_stereo_joint, encode_multichannel_stream, encode_stream_mono,
        encode_stream_stereo,
    };

    /// Synthesise a header-only (metadata-free) block with the given
    /// header words. `block_samples == 0` makes it metadata-only.
    fn raw_block(block_index: u32, block_samples: u32, flags: u32) -> Vec<u8> {
        let mut b = Vec::with_capacity(32);
        b.extend_from_slice(MAGIC);
        b.extend_from_slice(&24u32.to_le_bytes()); // ck_size: header only
        b.extend_from_slice(&0x0410u16.to_le_bytes()); // version
        b.push(0); // track_number
        b.push(0); // track_sub_index
        b.extend_from_slice(&u32::MAX.to_le_bytes()); // total_samples unknown
        b.extend_from_slice(&block_index.to_le_bytes());
        b.extend_from_slice(&block_samples.to_le_bytes());
        b.extend_from_slice(&flags.to_le_bytes());
        b.extend_from_slice(&0u32.to_le_bytes()); // crc
        b
    }

    /// Flag word helpers: standalone marker (bits 11+12), optional
    /// mono bit 2, plus extras.
    const STANDALONE: u32 = 0b11 << 11;
    const MONO: u32 = 1 << 2;
    const FIRST: u32 = 1 << 11;
    const FINAL: u32 = 1 << 12;

    fn mono_pcm(n: usize) -> Vec<i32> {
        (0..n).map(|i| (i as i32 * 37) % 1000 - 500).collect()
    }

    #[test]
    fn scan_empty_buffer_yields_empty_index() {
        let index = StreamIndex::scan(&[]).expect("empty scan");
        assert_eq!(index.block_count(), 0);
        assert_eq!(index.set_count(), 0);
        assert_eq!(index.channels(), 0);
        assert!(index.is_empty());
        assert!(index.is_seekable());
        assert_eq!(index.first_frame(), 0);
        assert_eq!(index.end_frame(), 0);
        assert_eq!(index.frame_count(), 0);
        assert_eq!(index.locate_frame(0), None);
    }

    #[test]
    fn scan_indexes_encoded_mono_stream() {
        let pcm = mono_pcm(700);
        let wv = encode_stream_mono(&pcm, 256, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        // 700 samples at 256/block = 3 blocks, each its own set.
        assert_eq!(index.block_count(), 3);
        assert_eq!(index.audio_block_count(), 3);
        assert_eq!(index.set_count(), 3);
        assert_eq!(index.channels(), 1);
        assert!(index.is_seekable());
        assert_eq!(index.first_frame(), 0);
        assert_eq!(index.end_frame(), 700);
        assert_eq!(index.frame_count(), 700);
        // Entries tile the buffer exactly.
        let mut offset = 0usize;
        for e in index.entries() {
            assert_eq!(e.byte_offset, offset);
            offset += e.byte_len;
            assert_eq!(e.member_channels(), 1);
        }
        assert_eq!(offset, wv.len());
        // Set frame ranges are the block ranges.
        assert_eq!(index.sets()[0].first_frame, 0);
        assert_eq!(index.sets()[0].frames, 256);
        assert_eq!(index.sets()[2].first_frame, 512);
        assert_eq!(index.sets()[2].frames, 188);
    }

    #[test]
    fn scan_indexes_encoded_stereo_stream() {
        let pcm: Vec<i32> = (0..600).map(|i| (i * 13) % 800 - 400).collect();
        let wv = encode_stream_stereo(&pcm, 100, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(index.channels(), 2);
        assert_eq!(index.set_count(), 3);
        assert_eq!(index.frame_count(), 300);
        assert!(index.is_seekable());
        for e in index.entries() {
            assert_eq!(e.member_channels(), 2);
            assert!(e.is_audio());
        }
    }

    #[test]
    fn scan_indexes_multichannel_sets() {
        // 4 channels × 300 frames, split into 128-frame sets of four
        // mono members each.
        let channels = 4usize;
        let frames = 300usize;
        let pcm: Vec<i32> = (0..frames * channels)
            .map(|i| (i as i32 * 7) % 512 - 256)
            .collect();
        let wv = encode_multichannel_stream(&pcm, channels, 128, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(index.channels(), 4);
        assert_eq!(index.set_count(), 3); // 128 + 128 + 44
        assert_eq!(index.audio_block_count(), 12); // 4 mono members per set
        assert_eq!(index.frame_count(), 300);
        assert!(index.is_seekable());
        for set in index.sets() {
            assert_eq!(set.channels, 4);
            assert_eq!(set.member_entries().len(), 4);
        }
        assert_eq!(index.sets()[2].first_frame, 256);
        assert_eq!(index.sets()[2].frames, 44);
    }

    #[test]
    fn scan_joint_stereo_block_is_one_two_channel_set() {
        let pcm: Vec<i32> = (0..128).map(|i| ((i * 31) % 700) - 350).collect();
        let wv = encode_block_stereo_joint(&pcm, 2, 0, 64).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(index.set_count(), 1);
        assert_eq!(index.channels(), 2);
        assert_eq!(index.frame_count(), 64);
    }

    #[test]
    fn scan_skips_metadata_only_blocks() {
        let pcm = mono_pcm(200);
        let mut wv = raw_block(0, 0, STANDALONE); // leading metadata-only
        wv.extend_from_slice(&encode_stream_mono(&pcm, 100, 2).expect("encode"));
        wv.extend_from_slice(&raw_block(0, 0, STANDALONE)); // trailing
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(index.block_count(), 4);
        assert_eq!(index.audio_block_count(), 2);
        assert_eq!(index.set_count(), 2);
        assert_eq!(index.frame_count(), 200);
        assert!(index.is_seekable());
        assert!(!index.entries()[0].is_audio());
        assert!(!index.entries()[3].is_audio());
    }

    #[test]
    fn scan_metadata_only_block_inside_open_set_does_not_break_grouping() {
        // first member .. metadata-only .. final member — the decoder
        // skips the metadata block mid-set; the scan must too.
        let mut wv = raw_block(0, 64, MONO | FIRST);
        wv.extend_from_slice(&raw_block(0, 0, 0)); // metadata-only, no markers
        wv.extend_from_slice(&raw_block(0, 64, MONO | FINAL));
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(index.set_count(), 1);
        assert_eq!(index.sets()[0].member_entries(), &[0, 2]);
        assert_eq!(index.channels(), 2);
    }

    #[test]
    fn scan_refuses_stray_final_marker() {
        let wv = raw_block(0, 64, MONO | FINAL);
        assert!(matches!(
            StreamIndex::scan(&wv),
            Err(Error::MultichannelSetMalformed)
        ));
    }

    #[test]
    fn scan_refuses_unterminated_set() {
        let wv = raw_block(0, 64, MONO | FIRST);
        assert!(matches!(
            StreamIndex::scan(&wv),
            Err(Error::MultichannelSetMalformed)
        ));
    }

    #[test]
    fn scan_refuses_double_first_marker() {
        let mut wv = raw_block(0, 64, MONO | FIRST);
        wv.extend_from_slice(&raw_block(0, 64, MONO | FIRST));
        assert!(matches!(
            StreamIndex::scan(&wv),
            Err(Error::MultichannelSetMalformed)
        ));
    }

    #[test]
    fn scan_refuses_member_sample_count_mismatch() {
        let mut wv = raw_block(0, 64, MONO | FIRST);
        wv.extend_from_slice(&raw_block(0, 65, MONO | FINAL));
        assert!(matches!(
            StreamIndex::scan(&wv),
            Err(Error::MultichannelSampleCountMismatch {
                expected: 64,
                found: 65
            })
        ));
    }

    #[test]
    fn scan_refuses_inter_set_channel_width_disagreement() {
        // A mono standalone set followed by a stereo standalone set —
        // the multichannel decoder refuses ragged frames; so does the
        // scan.
        let mut wv = raw_block(0, 64, MONO | STANDALONE);
        wv.extend_from_slice(&raw_block(64, 64, STANDALONE));
        assert!(matches!(
            StreamIndex::scan(&wv),
            Err(Error::MultichannelSetMalformed)
        ));
    }

    #[test]
    fn scan_refuses_channel_count_blowup() {
        // 2-channel members chained until the sum exceeds the cap.
        let mut wv = raw_block(0, 64, FIRST);
        let members_needed = MAX_MULTICHANNEL_CHANNELS / 2;
        for _ in 0..members_needed {
            wv.extend_from_slice(&raw_block(0, 64, 0));
        }
        let err = StreamIndex::scan(&wv);
        assert!(
            matches!(err, Err(Error::MultichannelTooManyChannels(_))),
            "{err:?}"
        );
    }

    #[test]
    fn scan_propagates_header_refusals() {
        // Bad magic.
        let mut wv = raw_block(0, 64, MONO | STANDALONE);
        wv[0] = b'X';
        assert!(matches!(StreamIndex::scan(&wv), Err(Error::InvalidMagic)));
        // Truncated between blocks (partial header).
        let mut wv = raw_block(0, 64, MONO | STANDALONE);
        wv.extend_from_slice(&MAGIC[..2]);
        assert!(matches!(StreamIndex::scan(&wv), Err(Error::Truncated)));
    }

    #[test]
    fn scan_reports_ck_size_exceeding_buffer() {
        let mut wv = raw_block(0, 64, MONO | STANDALONE);
        // Inflate ck_size beyond the buffer.
        wv[4..8].copy_from_slice(&100u32.to_le_bytes());
        match StreamIndex::scan(&wv) {
            Err(Error::CkSizeExceedsBuffer { ck_size, available }) => {
                assert_eq!(ck_size, 100);
                assert_eq!(available, 32);
            }
            other => panic!("expected CkSizeExceedsBuffer, got {other:?}"),
        }
    }

    #[test]
    fn gap_between_sets_makes_index_non_seekable() {
        let mut wv = raw_block(0, 64, MONO | STANDALONE);
        wv.extend_from_slice(&raw_block(100, 64, MONO | STANDALONE)); // gap 64..100
        let index = StreamIndex::scan(&wv).expect("scan");
        assert!(!index.is_seekable());
        assert_eq!(index.frame_count(), 128);
        assert_eq!(index.locate_frame(0), None);
        assert_eq!(index.set_for_frame(0), None);
    }

    #[test]
    fn overlapping_sets_make_index_non_seekable() {
        let mut wv = raw_block(0, 64, MONO | STANDALONE);
        wv.extend_from_slice(&raw_block(32, 64, MONO | STANDALONE));
        let index = StreamIndex::scan(&wv).expect("scan");
        assert!(!index.is_seekable());
    }

    #[test]
    fn regressing_block_index_makes_index_non_seekable() {
        let mut wv = raw_block(64, 64, MONO | STANDALONE);
        wv.extend_from_slice(&raw_block(0, 64, MONO | STANDALONE));
        let index = StreamIndex::scan(&wv).expect("scan");
        assert!(!index.is_seekable());
    }

    #[test]
    fn locate_frame_binary_search_hits_every_set() {
        let pcm = mono_pcm(700);
        let wv = encode_stream_mono(&pcm, 256, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(index.locate_frame(0), Some(0));
        assert_eq!(index.locate_frame(255), Some(0));
        assert_eq!(index.locate_frame(256), Some(1));
        assert_eq!(index.locate_frame(511), Some(1));
        assert_eq!(index.locate_frame(512), Some(2));
        assert_eq!(index.locate_frame(699), Some(2));
        assert_eq!(index.locate_frame(700), None);
        assert_eq!(index.locate_frame(u64::MAX), None);
        let set = index.set_for_frame(300).expect("set");
        assert!(set.contains_frame(300));
        assert_eq!(set.first_frame, 256);
    }

    #[test]
    fn locate_frame_respects_nonzero_stream_start() {
        // A stream whose first block starts at frame 1000 (e.g. a
        // mid-file extract) is still seekable in absolute terms.
        let mut wv = raw_block(1000, 64, MONO | STANDALONE);
        wv.extend_from_slice(&raw_block(1064, 64, MONO | STANDALONE));
        let index = StreamIndex::scan(&wv).expect("scan");
        assert!(index.is_seekable());
        assert_eq!(index.first_frame(), 1000);
        assert_eq!(index.end_frame(), 1128);
        assert_eq!(index.locate_frame(999), None);
        assert_eq!(index.locate_frame(1000), Some(0));
        assert_eq!(index.locate_frame(1127), Some(1));
        assert_eq!(index.locate_frame(1128), None);
    }

    #[test]
    fn set_byte_span_covers_all_members() {
        let channels = 4usize;
        let pcm: Vec<i32> = (0..128 * channels).map(|i| i as i32 % 100).collect();
        let wv = encode_multichannel_stream(&pcm, channels, 64, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(index.set_count(), 2);
        let span0 = index.set_byte_span(0).expect("span 0");
        let span1 = index.set_byte_span(1).expect("span 1");
        assert_eq!(span0.start, 0);
        assert_eq!(span0.end, span1.start);
        assert_eq!(span1.end, wv.len());
        assert_eq!(index.set_byte_span(2), None);
        // The span alone re-scans to a single equal set.
        let sub = StreamIndex::scan(&wv[span1.clone()]).expect("sub-scan");
        assert_eq!(sub.set_count(), 1);
        assert_eq!(sub.sets()[0].frames, index.sets()[1].frames);
        assert_eq!(sub.channels(), 4);
    }

    #[test]
    fn decode_range_full_span_equals_decode_stream_mono() {
        let pcm = mono_pcm(700);
        let wv = encode_stream_mono(&pcm, 256, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let ranged = decode_range(&wv, &index, 0, 700).expect("range");
        assert_eq!(ranged, crate::block::decode_stream(&wv).expect("full"));
        assert_eq!(ranged, pcm);
    }

    #[test]
    fn decode_range_every_window_equals_full_decode_slice_mono() {
        let pcm = mono_pcm(300);
        let wv = encode_stream_mono(&pcm, 128, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let full = crate::block::decode_stream(&wv).expect("full");
        // Sweep window starts and lengths across all block boundaries.
        for start in (0..300).step_by(37) {
            for len in [1usize, 7, 128, 129, 300 - start] {
                let len = len.min(300 - start);
                let ranged = decode_range(&wv, &index, start as u64, len as u64).expect("range");
                assert_eq!(ranged, full[start..start + len], "start={start} len={len}");
            }
        }
    }

    #[test]
    fn decode_range_windows_equal_full_decode_slice_stereo() {
        let pcm: Vec<i32> = (0..600).map(|i| (i * 13) % 800 - 400).collect();
        let wv = encode_stream_stereo(&pcm, 100, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let full = crate::block::decode_stream(&wv).expect("full");
        assert_eq!(index.channels(), 2);
        for (start, len) in [(0u64, 300u64), (50, 100), (99, 2), (100, 100), (250, 50)] {
            let ranged = decode_range(&wv, &index, start, len).expect("range");
            let s = start as usize * 2;
            let e = s + len as usize * 2;
            assert_eq!(ranged, full[s..e], "start={start} len={len}");
        }
    }

    #[test]
    fn decode_range_windows_equal_full_decode_slice_multichannel() {
        let channels = 5usize;
        let frames = 250usize;
        let pcm: Vec<i32> = (0..frames * channels)
            .map(|i| (i as i32 * 11) % 600 - 300)
            .collect();
        let wv = encode_multichannel_stream(&pcm, channels, 100, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let full = crate::block::decode_multichannel_stream(&wv).expect("full");
        assert_eq!(full.channels, channels);
        assert_eq!(index.channels(), channels);
        for (start, len) in [
            (0u64, 250u64),
            (0, 1),
            (99, 2),
            (100, 150),
            (249, 1),
            (60, 130),
        ] {
            let ranged = decode_range(&wv, &index, start, len).expect("range");
            let s = start as usize * channels;
            let e = s + len as usize * channels;
            assert_eq!(ranged, full.samples[s..e], "start={start} len={len}");
        }
    }

    #[test]
    fn decode_range_zero_frames_is_empty() {
        let pcm = mono_pcm(100);
        let wv = encode_stream_mono(&pcm, 0, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(
            decode_range(&wv, &index, 0, 0).expect("range"),
            Vec::<i32>::new()
        );
        assert_eq!(
            decode_range(&wv, &index, 100, 0).expect("range"),
            Vec::<i32>::new()
        );
        let (pcm0, ok) = decode_range_muted(&wv, &index, 50, 0).expect("range");
        assert!(pcm0.is_empty());
        assert!(ok);
    }

    #[test]
    fn decode_range_refuses_out_of_coverage_windows() {
        let pcm = mono_pcm(100);
        let wv = encode_stream_mono(&pcm, 0, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        assert!(matches!(
            decode_range(&wv, &index, 0, 101),
            Err(Error::SeekOutOfRange {
                requested: 101,
                first_frame: 0,
                end_frame: 100
            })
        ));
        assert!(matches!(
            decode_range(&wv, &index, 100, 1),
            Err(Error::SeekOutOfRange { .. })
        ));
        // Overflowing start + frames must not wrap.
        assert!(matches!(
            decode_range(&wv, &index, u64::MAX, 2),
            Err(Error::SeekOutOfRange { .. })
        ));
    }

    #[test]
    fn decode_range_refuses_non_seekable_index() {
        let mut wv = raw_block(0, 64, MONO | STANDALONE);
        wv.extend_from_slice(&raw_block(100, 64, MONO | STANDALONE));
        let index = StreamIndex::scan(&wv).expect("scan");
        assert!(matches!(
            decode_range(&wv, &index, 0, 1),
            Err(Error::StreamNotSeekable)
        ));
        assert!(matches!(
            decode_range_muted(&wv, &index, 0, 1),
            Err(Error::StreamNotSeekable)
        ));
    }

    #[test]
    fn decode_range_muted_matches_decode_stream_muted_on_corruption() {
        let pcm = mono_pcm(384);
        let mut wv = encode_stream_mono(&pcm, 128, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        // Corrupt the stored CRC of the middle block (header offset 28).
        let mid = index.entries()[1].byte_offset;
        wv[mid + 28] ^= 0xff;
        let (full, full_ok) = crate::block::decode_stream_muted(&wv).expect("full muted");
        assert!(!full_ok);
        let (ranged, ok) = decode_range_muted(&wv, &index, 0, 384).expect("range muted");
        assert!(!ok);
        assert_eq!(ranged, full);
        // The muted block's frames are zeros; its neighbours survive.
        assert_eq!(&ranged[128..256], &[0i32; 128]);
        assert_eq!(&ranged[..128], &pcm[..128]);
        // A window that avoids the corrupt block reports all-ok.
        let (clean, ok) = decode_range_muted(&wv, &index, 256, 128).expect("range muted");
        assert!(ok);
        assert_eq!(clean, pcm[256..384]);
        // A window touching only the corrupt block reports the mute.
        let (muted, ok) = decode_range_muted(&wv, &index, 200, 10).expect("range muted");
        assert!(!ok);
        assert_eq!(muted, [0i32; 10]);
        // The plain (non-muted) range decoder still decodes — CRC is
        // not consulted outside the muted path.
        let plain = decode_range(&wv, &index, 128, 128).expect("plain");
        assert_eq!(plain, pcm[128..256]);
    }

    #[test]
    fn decode_range_on_shifted_and_joint_blocks() {
        // Joint stereo single block.
        let pcm: Vec<i32> = (0..256).map(|i| ((i * 31) % 700) - 350).collect();
        let wv = encode_block_stereo_joint(&pcm, 2, 0, 128).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let full = crate::block::decode_stream(&wv).expect("full");
        let ranged = decode_range(&wv, &index, 40, 50).expect("range");
        assert_eq!(ranged, full[80..180]);
        // Left-shifted (12-bit) mono block.
        let pcm12: Vec<i32> = (0..100).map(|i| ((i * 37) % 2000 - 1000) * 16).collect();
        let wv = crate::encode::encode_block_mono_shifted(&pcm12, 4, 2, 0, 100).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let ranged = decode_range(&wv, &index, 25, 50).expect("range");
        assert_eq!(ranged, pcm12[25..75]);
    }

    #[test]
    fn decode_range_truncated_buffer_after_scan_is_typed() {
        // Scan a full buffer, then hand the range decoder a shorter
        // one — the recorded byte spans no longer fit.
        let pcm = mono_pcm(200);
        let wv = encode_stream_mono(&pcm, 100, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let short = &wv[..wv.len() - 4];
        assert!(matches!(
            decode_range(short, &index, 150, 10),
            Err(Error::Truncated)
        ));
        // Windows inside the intact prefix still decode.
        assert_eq!(
            decode_range(short, &index, 0, 100).expect("range"),
            pcm[..100]
        );
    }

    #[test]
    fn reader_sequential_chunked_reads_equal_full_decode() {
        let pcm = mono_pcm(700);
        let wv = encode_stream_mono(&pcm, 256, 2).expect("encode");
        let full = crate::block::decode_stream(&wv).expect("full");
        // Chunk sizes that straddle set boundaries in different ways.
        for chunk in [1usize, 7, 100, 256, 257, 1000] {
            let mut reader = StreamReader::new(&wv).expect("reader");
            assert_eq!(reader.channels(), 1);
            assert_eq!(reader.position(), 0);
            let mut got: Vec<i32> = Vec::new();
            loop {
                let frames = reader.read_frames(chunk).expect("read");
                if frames.is_empty() {
                    break;
                }
                got.extend_from_slice(&frames);
            }
            assert_eq!(got, full, "chunk={chunk}");
            assert!(reader.is_at_end());
            assert_eq!(reader.frames_remaining(), 0);
        }
    }

    #[test]
    fn reader_seek_and_read_equal_range_decode() {
        let channels = 3usize;
        let frames = 250usize;
        let pcm: Vec<i32> = (0..frames * channels)
            .map(|i| (i as i32 * 17) % 900 - 450)
            .collect();
        let wv = encode_multichannel_stream(&pcm, channels, 100, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let mut reader = StreamReader::new(&wv).expect("reader");
        assert_eq!(reader.channels(), 3);
        for (start, len) in [(200u64, 50usize), (0, 10), (99, 3), (120, 130), (249, 1)] {
            reader.seek(start).expect("seek");
            assert_eq!(reader.position(), start);
            let got = reader.read_frames(len).expect("read");
            let want = decode_range(&wv, &index, start, len as u64).expect("range");
            assert_eq!(got, want, "start={start} len={len}");
            assert_eq!(reader.position(), start + len as u64);
        }
    }

    #[test]
    fn reader_seek_back_within_cached_set_rereads_identically() {
        let pcm = mono_pcm(300);
        let wv = encode_stream_mono(&pcm, 300, 2).expect("encode");
        let mut reader = StreamReader::new(&wv).expect("reader");
        let first = reader.read_frames(200).expect("read");
        reader.seek(50).expect("seek back");
        let again = reader.read_frames(150).expect("re-read");
        assert_eq!(again, first[50..200]);
    }

    #[test]
    fn reader_seek_bounds() {
        let pcm = mono_pcm(100);
        let wv = encode_stream_mono(&pcm, 0, 2).expect("encode");
        let mut reader = StreamReader::new(&wv).expect("reader");
        // Seeking to the end is a valid cursor state...
        reader.seek(100).expect("seek to end");
        assert!(reader.is_at_end());
        assert_eq!(
            reader.read_frames(10).expect("read at end"),
            Vec::<i32>::new()
        );
        // ...one past is not.
        assert!(matches!(
            reader.seek(101),
            Err(Error::SeekOutOfRange {
                requested: 101,
                first_frame: 0,
                end_frame: 100
            })
        ));
        // The failed seek left the cursor unchanged.
        assert_eq!(reader.position(), 100);
        reader.seek(0).expect("rewind");
        assert_eq!(reader.read_frames(100).expect("read"), pcm);
    }

    #[test]
    fn reader_refuses_non_seekable_stream() {
        let mut wv = raw_block(0, 64, MONO | STANDALONE);
        wv.extend_from_slice(&raw_block(100, 64, MONO | STANDALONE));
        assert!(matches!(
            StreamReader::new(&wv),
            Err(Error::StreamNotSeekable)
        ));
        // with_index refuses the same way.
        let index = StreamIndex::scan(&wv).expect("scan");
        assert!(matches!(
            StreamReader::with_index(&wv, index),
            Err(Error::StreamNotSeekable)
        ));
    }

    #[test]
    fn reader_on_empty_stream() {
        let mut reader = StreamReader::new(&[]).expect("reader");
        assert_eq!(reader.channels(), 0);
        assert!(reader.is_at_end());
        assert_eq!(reader.read_frames(16).expect("read"), Vec::<i32>::new());
        reader.seek(0).expect("seek 0");
        assert!(matches!(reader.seek(1), Err(Error::SeekOutOfRange { .. })));
        // Metadata-only stream behaves the same.
        let wv = raw_block(0, 0, STANDALONE);
        let reader = StreamReader::new(&wv).expect("reader");
        assert!(reader.is_at_end());
    }

    #[test]
    fn reader_starts_at_nonzero_first_frame() {
        let mut wv = raw_block(1000, 64, MONO | STANDALONE);
        wv.extend_from_slice(&raw_block(1064, 64, MONO | STANDALONE));
        // Raw header-only blocks have no audio payload to decode, so
        // only check the cursor geometry.
        let reader = StreamReader::new(&wv).expect("reader");
        assert_eq!(reader.position(), 1000);
        assert_eq!(reader.frames_remaining(), 128);
        let mut reader = reader;
        assert!(matches!(
            reader.seek(999),
            Err(Error::SeekOutOfRange { .. })
        ));
        reader.seek(1128).expect("seek to end");
    }

    #[test]
    fn reader_muted_reads_gate_and_cache_correctly() {
        let pcm = mono_pcm(384);
        let mut wv = encode_stream_mono(&pcm, 128, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        // Corrupt the middle block's stored CRC.
        let mid = index.entries()[1].byte_offset;
        wv[mid + 28] ^= 0xff;

        let mut reader = StreamReader::new(&wv).expect("reader");
        // Clean first set.
        let (a, ok) = reader.read_frames_muted(128).expect("read");
        assert!(ok);
        assert_eq!(a, pcm[..128]);
        // Corrupt second set mutes.
        let (b, ok) = reader.read_frames_muted(128).expect("read");
        assert!(!ok);
        assert_eq!(b, [0i32; 128]);
        // Seek back inside the (cached, muted) corrupt set: the CRC
        // verdict is re-reported, not lost with the cache hit.
        reader.seek(130).expect("seek");
        let (c, ok) = reader.read_frames_muted(10).expect("read");
        assert!(!ok);
        assert_eq!(c, [0i32; 10]);
        // A plain read over the same corrupt set must NOT be served
        // from the muted cache (the plain decode still yields the
        // block's decoded samples — CRC is not consulted).
        reader.seek(128).expect("seek");
        let plain = reader.read_frames(128).expect("plain read");
        assert_eq!(plain, pcm[128..256]);
        // And a muted read after the plain one re-decodes rather than
        // serving the plain cache.
        reader.seek(128).expect("seek");
        let (again, ok) = reader.read_frames_muted(128).expect("read");
        assert!(!ok);
        assert_eq!(again, [0i32; 128]);
        // Cross-mode cache reuse on a CLEAN set is allowed: a muted
        // read of set 3 then a plain re-read of the same frames.
        reader.seek(256).expect("seek");
        let (clean, ok) = reader.read_frames_muted(128).expect("read");
        assert!(ok);
        assert_eq!(clean, pcm[256..384]);
        reader.seek(256).expect("seek");
        assert_eq!(reader.read_frames(128).expect("read"), pcm[256..384]);
    }

    #[test]
    fn reader_failed_read_resumes_at_first_unconsumed_frame() {
        // Two blocks; truncate the buffer inside the second so its
        // decode fails, then verify the cursor lands at the set
        // boundary and the intact prefix was consumable.
        let pcm = mono_pcm(200);
        let wv = encode_stream_mono(&pcm, 100, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        let short = &wv[..wv.len() - 4];
        let mut reader =
            StreamReader::with_index(short, index).expect("reader over truncated buffer");
        // Read across both sets: fails on the second.
        assert!(reader.read_frames(150).is_err());
        // Nothing was returned; cursor did not advance past the first
        // set — retry semantics from frame 0.
        assert_eq!(reader.position(), 0);
        // Reading only the intact set succeeds.
        let got = reader.read_frames(100).expect("read intact set");
        assert_eq!(got, pcm[..100]);
        assert_eq!(reader.position(), 100);
    }

    #[test]
    fn scan_matches_stream_walkers_on_encoded_streams() {
        let pcm = mono_pcm(500);
        let wv = encode_stream_mono(&pcm, 128, 2).expect("encode");
        let index = StreamIndex::scan(&wv).expect("scan");
        assert_eq!(
            index.block_count(),
            crate::block::block_count(&wv).expect("block_count")
        );
        assert_eq!(
            index.audio_block_count(),
            crate::block::audio_block_count(&wv).expect("audio_block_count")
        );
        let layout = crate::block::multichannel_layout(&wv).expect("layout");
        assert_eq!(index.channels(), layout.channels);
        assert_eq!(index.set_count(), layout.sets);
    }
}
