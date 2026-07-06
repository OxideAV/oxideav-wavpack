#![no_main]

//! Drive arbitrary fuzz-supplied bytes through the **seeking
//! subsystem** — the header-only [`StreamIndex`] scan, the
//! random-access [`decode_range`] window decoder, and the
//! [`StreamReader`] cursor — and assert the cross-layer invariants
//! against the whole-stream decoders.
//!
//! Contracts under test:
//!
//! - `StreamIndex::scan` returns (no panic / overflow / OOB) on any
//!   input; on success the entries tile the buffer exactly and the
//!   sets' members are in-range audio entries.
//! - The scan is **never stricter than the decoder**: whenever
//!   `decode_multichannel_stream` accepts the bytes, the scan must
//!   too, and `frame_count * channels == samples.len()`.
//! - `locate_frame` agrees with the set table on a seekable index
//!   (first / last frame of every set, one-past-the-end miss).
//! - On a seekable, decodable stream, `decode_range` over the full
//!   span — and over a fuzz-chosen window — is bit-exactly the
//!   whole-stream decode (slice), the muted twin matches
//!   `decode_multichannel_stream_muted` (PCM and full-span verdict),
//!   and a `StreamReader` chunked-read walk reproduces the stream.
//! - Cross-walker: block / audio counts and the multichannel layout
//!   agree with the metadata-parsing walkers whenever both sides
//!   succeed.
//!
//! The first three input bytes steer the window start / length and
//! the reader chunk size; the remainder is the stream under test.
//!
//! **RSS sizing note:** the same inherent zero-run amplification the
//! `decode_stream` target documents applies here, and the equality
//! oracles hold both the whole-stream decode and one ranged copy
//! alive at once — campaigns must run with an `-rss_limit_mb` sized
//! for a few `MAX_DECODE_SAMPLES_PER_BLOCK` expansions (4096 is
//! comfortable).

use libfuzzer_sys::fuzz_target;
use oxideav_wavpack::{
    audio_block_count, block_count, decode_multichannel_stream, decode_multichannel_stream_muted,
    decode_range, decode_range_muted, multichannel_layout, StreamIndex, StreamReader,
};

fuzz_target!(|data: &[u8]| {
    if data.len() < 3 {
        return;
    }
    let (control, bytes) = data.split_at(3);

    let Ok(index) = StreamIndex::scan(bytes) else {
        // A refused scan must also be refused (or shape-error'd) by
        // the multichannel decoder — the scan applies a subset of its
        // checks. (The decoder may fail with a *different* typed
        // error; only Ok-ness is asserted.)
        assert!(
            decode_multichannel_stream(bytes).is_err(),
            "scan refused a stream the decoder accepts"
        );
        return;
    };

    // Structural invariants: entries tile the buffer exactly.
    let mut offset = 0usize;
    for e in index.entries() {
        assert_eq!(e.byte_offset, offset, "entries must be contiguous");
        assert!(e.byte_len >= 32, "a block is at least its fixed header");
        offset += e.byte_len;
    }
    assert_eq!(offset, bytes.len(), "entries must cover the whole buffer");

    // Set invariants: members are in-range audio entries; the frame
    // count is the per-set sum; channel widths agree.
    let mut sum_frames = 0u64;
    for set in index.sets() {
        assert!(!set.member_entries().is_empty(), "a set has members");
        let mut member_channels = 0usize;
        for &m in set.member_entries() {
            let e = &index.entries()[m];
            assert!(e.is_audio(), "set members carry audio");
            assert_eq!(e.block_samples, set.frames, "members agree on frames");
            member_channels += e.member_channels();
        }
        assert_eq!(member_channels, set.channels, "set channel sum");
        assert_eq!(set.channels, index.channels(), "uniform channel width");
        sum_frames += u64::from(set.frames);
    }
    assert_eq!(index.frame_count(), sum_frames, "frame_count is the set sum");

    // locate_frame agrees with the set table on a seekable index.
    if index.is_seekable() {
        for (i, set) in index.sets().iter().enumerate() {
            assert_eq!(index.locate_frame(u64::from(set.first_frame)), Some(i));
            assert_eq!(index.locate_frame(set.end_frame() - 1), Some(i));
        }
        assert_eq!(index.locate_frame(index.end_frame()), None);
    } else {
        assert_eq!(index.locate_frame(index.first_frame()), None);
    }

    // Cross-walker agreement (both sides parse metadata differently;
    // compare only when both succeed).
    if let Ok(b) = block_count(bytes) {
        assert_eq!(b, index.block_count(), "block_count walker agreement");
    }
    if let Ok(a) = audio_block_count(bytes) {
        assert_eq!(a, index.audio_block_count(), "audio walker agreement");
    }
    if let Ok(layout) = multichannel_layout(bytes) {
        assert_eq!(layout.channels, index.channels(), "layout channels");
        assert_eq!(layout.sets, index.set_count(), "layout set count");
    }

    // Decode oracles: only on streams the whole-stream decoder accepts.
    let Ok(stream) = decode_multichannel_stream(bytes) else {
        return;
    };
    if index.channels() > 0 {
        assert_eq!(
            index.frame_count() * index.channels() as u64,
            stream.samples.len() as u64,
            "frame_count * channels == decoded PCM values"
        );
    } else {
        assert!(stream.samples.is_empty());
    }

    if !index.is_seekable() || index.channels() == 0 {
        return;
    }
    let first = index.first_frame();
    let total = index.frame_count();
    let channels = index.channels();

    // Full-span ranged decode == whole-stream decode.
    let full = decode_range(bytes, &index, first, total).expect("full-span range decode");
    assert_eq!(full, stream.samples, "full-span range == stream decode");

    // Muted full span == muted whole-stream decode (PCM and verdict —
    // the span touches every member).
    let (full_muted, ok) =
        decode_range_muted(bytes, &index, first, total).expect("full-span muted");
    let (stream_muted, stream_ok) =
        decode_multichannel_stream_muted(bytes).expect("stream muted");
    assert_eq!(full_muted, stream_muted.samples, "muted PCM parity");
    assert_eq!(ok, stream_ok, "muted verdict parity");

    // Fuzz-chosen window == the same slice of the whole decode.
    let start = first + (u64::from(control[0]) * total) / 256;
    let len = 1 + (u64::from(control[1]) * (total - (start - first)).saturating_sub(1)) / 256;
    let ranged = decode_range(bytes, &index, start, len).expect("window range decode");
    let s = usize::try_from((start - first) * channels as u64).expect("fits");
    let e = s + usize::try_from(len * channels as u64).expect("fits");
    assert_eq!(ranged, stream.samples[s..e], "window == stream slice");

    // StreamReader chunked walk reproduces the stream.
    let chunk = 1 + control[2] as usize;
    let mut reader = StreamReader::with_index(bytes, index).expect("seekable reader");
    let mut got: Vec<i32> = Vec::with_capacity(stream.samples.len());
    loop {
        let frames = reader.read_frames(chunk).expect("reader read");
        if frames.is_empty() {
            break;
        }
        got.extend_from_slice(&frames);
    }
    assert_eq!(got, stream.samples, "reader walk == stream decode");
    assert!(reader.is_at_end());
});
