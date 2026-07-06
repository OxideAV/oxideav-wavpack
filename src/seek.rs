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
    /// multichannel-set member: `1` for mono / false-stereo data,
    /// `2` for interleaved stereo (the
    /// [`Flags::is_block_data_mono`] union the decoder itself
    /// dispatches on).
    pub fn member_channels(&self) -> usize {
        if self.flags.is_block_data_mono() {
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
