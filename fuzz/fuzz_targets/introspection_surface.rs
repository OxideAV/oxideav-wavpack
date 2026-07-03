#![no_main]

//! Drive arbitrary fuzz-supplied bytes through the **non-decoding
//! introspection surface** — every stream-level walker that parses
//! headers and metadata without running the entropy coder.
//!
//! The contract under test is that every call *returns* (no panic, no
//! debug-build overflow, no out-of-bounds) and that the counters agree
//! with each other on parseable inputs:
//!
//! - `block_count == audio_block_count + metadata_block_count`
//!   whenever all three succeed (every block is exactly one of the
//!   two kinds — the wiki `block_samples > 0` audio predicate);
//! - `correction_block_count <= block_count`;
//! - `total_audio_samples <= decoded_sample_count <= 2 *
//!   total_audio_samples` (the first sums per-channel header samples,
//!   the second counts returned PCM values — ×2 on stereo blocks —
//!   so the two agree exactly on all-mono chains and differ by at
//!   most the ×2 channel factor otherwise);
//! - `correction_coverage(main, wvc)` — the round-386 `.wvc` pairing
//!   walker — reports `paired <= total` and never panics when the
//!   input is split into an arbitrary (main, correction) pair at a
//!   fuzz-chosen boundary.
//!
//! The first input byte selects the split point for the pairing walk;
//! the remainder is the byte stream under test (whole stream for the
//! single-buffer walkers, split halves for the pairing walker).

use libfuzzer_sys::fuzz_target;
use oxideav_wavpack::{
    audio_block_count, block_count, correction_block_count, correction_coverage,
    decoded_sample_count, first_audio_block, first_correction_block, metadata_block_count,
    multichannel_layout, stream_total_samples, total_audio_samples, total_block_samples,
    total_correction_payload_bytes,
};

fuzz_target!(|data: &[u8]| {
    if data.is_empty() {
        return;
    }
    let (control, bytes) = data.split_first().unwrap();

    // Single-buffer walkers: every result shape is legal; panics are not.
    let blocks = block_count(bytes);
    let audio = audio_block_count(bytes);
    let metadata = metadata_block_count(bytes);
    if let (Ok(b), Ok(a), Ok(m)) = (&blocks, &audio, &metadata) {
        assert_eq!(*b, *a + *m, "block kinds must partition the chain");
    }
    if let (Ok(c), Ok(b)) = (correction_block_count(bytes), &blocks) {
        assert!(c <= *b, "correction-bearing blocks are a subset");
    }
    if let (Ok(d), Ok(t)) = (decoded_sample_count(bytes), total_audio_samples(bytes)) {
        assert!(
            d >= t && d <= t.saturating_mul(2),
            "PCM-value count must be the per-channel sum times 1..=2 (d={d}, t={t})"
        );
    }
    let _ = stream_total_samples(bytes);
    let _ = multichannel_layout(bytes);
    let _ = first_audio_block(bytes);
    let _ = first_correction_block(bytes);
    let _ = total_correction_payload_bytes(bytes);
    if let Ok(parsed) = oxideav_wavpack::parse_blocks(bytes) {
        let _ = total_block_samples(&parsed);
    }

    // Pairing walker: split the input at a fuzz-chosen boundary and
    // treat the halves as (main, correction).
    let split = (*control as usize * bytes.len()) / 256;
    let (main, wvc) = bytes.split_at(split.min(bytes.len()));
    if let Ok((paired, total)) = correction_coverage(main, wvc) {
        assert!(paired <= total, "coverage cannot exceed the block count");
    }
});
