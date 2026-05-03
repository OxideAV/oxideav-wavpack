//! Bare-bones WavPack file walker (`.wv`).
//!
//! A `.wv` file is just a concatenation of `wvpk` blocks (spec §3.1).
//! For the round-1 round-trip tests we walk the file at the block
//! level and group blocks into frames using `INITIAL_BLOCK` /
//! `FINAL_BLOCK`. APE / ID3v1 trailers (after the last block) are
//! not parsed in round 1 — they are not required for bit-exact PCM
//! reconstruction.

use oxideav_core::{Error, Result};

use crate::block::{parse_sub_blocks, BlockHeader, SubBlock, WP_ID_CHANINFO, WP_ID_SAMPLE_RATE};

/// One parsed block: header + the byte range of its payload in the
/// source buffer.
#[derive(Debug, Clone)]
pub struct ParsedBlock {
    pub header: BlockHeader,
    /// Byte offset of the payload (i.e. the byte just after the
    /// 32-byte header) in the source buffer.
    pub payload_offset: usize,
    pub payload_len: usize,
}

impl ParsedBlock {
    pub fn payload<'a>(&self, src: &'a [u8]) -> &'a [u8] {
        &src[self.payload_offset..self.payload_offset + self.payload_len]
    }
}

/// One frame: a contiguous run of blocks from `INITIAL_BLOCK` to
/// `FINAL_BLOCK` (per spec §3.6).
#[derive(Debug, Clone)]
pub struct Frame {
    pub blocks: Vec<ParsedBlock>,
}

/// Result of walking a complete `.wv` byte buffer.
#[derive(Debug, Clone)]
pub struct ParsedFile {
    pub frames: Vec<Frame>,
}

/// Walk every block of a WavPack `.wv` file. Stops at the first byte
/// that is not a recognised `wvpk` magic (allowing optional APE / ID3
/// trailer to live undisturbed at the file tail).
pub fn parse_file(src: &[u8]) -> Result<ParsedFile> {
    let mut frames: Vec<Frame> = Vec::new();
    let mut current: Vec<ParsedBlock> = Vec::new();
    let mut pos = 0usize;
    while pos + 32 <= src.len() {
        if &src[pos..pos + 4] != b"wvpk" {
            // Stop on the first non-block byte; APE/ID3 trailer territory.
            break;
        }
        let hdr = BlockHeader::parse(&src[pos..pos + 32])?;
        let on_disk_len = hdr.on_disk_len();
        if pos + on_disk_len > src.len() {
            return Err(Error::invalid("WavPack: block extends past end of file"));
        }
        let pb = ParsedBlock {
            header: hdr,
            payload_offset: pos + 32,
            payload_len: hdr.payload_len(),
        };
        let is_initial = pb.header.is_initial();
        let is_final = pb.header.is_final();
        if is_initial && !current.is_empty() {
            return Err(Error::invalid("WavPack: INITIAL_BLOCK seen mid-frame"));
        }
        current.push(pb);
        if is_final {
            frames.push(Frame {
                blocks: std::mem::take(&mut current),
            });
        }
        pos += on_disk_len;
    }
    if !current.is_empty() {
        return Err(Error::invalid("WavPack: file ended without FINAL_BLOCK"));
    }
    Ok(ParsedFile { frames })
}

/// Find the `CHANINFO` payload in a block's sub-block list (or `None`).
pub fn find_chaninfo<'a>(subs: &'a [SubBlock<'a>]) -> Option<&'a [u8]> {
    subs.iter()
        .find(|s| s.ty() == WP_ID_CHANINFO)
        .map(|s| s.data)
}

/// Find the custom-rate payload in a block's sub-block list. Returns
/// the 24-bit LE Hz value when present.
///
/// `WP_ID_SAMPLE_RATE` shares its 5-bit type slot (0x07) with
/// `WP_ID_SHAPING`; the wire-format distinguishes them by always
/// setting the IGNORE bit on SAMPLE_RATE, so we match against the
/// full id (LARGE/ODD stripped).
pub fn find_sample_rate(subs: &[SubBlock<'_>]) -> Option<u32> {
    subs.iter()
        .find(|s| s.id_no_size_flags() == WP_ID_SAMPLE_RATE)
        .map(|s| {
            // 3-byte LE Hz value; pad MSB with 0.
            (s.data[0] as u32) | ((s.data[1] as u32) << 8) | ((s.data[2] as u32) << 16)
        })
}

/// Decode one frame and return one `Vec<i32>` per stream-level channel,
/// in the order dictated by the multi-channel grouping (block 0
/// first, then block 1, etc — within a block channels are in the
/// natural pair order).
pub fn decode_frame(src: &[u8], frame: &Frame) -> Result<Vec<Vec<i32>>> {
    let mut out: Vec<Vec<i32>> = Vec::new();
    for blk in &frame.blocks {
        let payload = blk.payload(src);
        let subs = parse_sub_blocks(payload)?;
        let mut decoded = crate::decoder::decode_block_samples(&blk.header, &subs)?;
        out.append(&mut decoded);
    }
    Ok(out)
}
