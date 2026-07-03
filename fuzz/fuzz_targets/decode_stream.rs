#![no_main]

//! Decode arbitrary fuzz-supplied bytes through the WavPack block-stream
//! decoder.
//!
//! The contract under test is purely that the call *returns*: a
//! malformed stream yields `Err(oxideav_wavpack::Error::…)`, a
//! well-formed one yields `Ok(Vec<i32>)`, and neither path may panic,
//! integer-overflow (in a debug build), index out of bounds, or
//! allocate an attacker-controlled sample buffer sized from the raw
//! `block_samples` header field rather than from the bounded payload.
//!
//! [`oxideav_wavpack::decode_stream`] is the broadest public decode
//! entry point: it drives [`parse_block`] (header validation +
//! ck_size/buffer bounds), the metadata walker (sub-block size-word
//! arithmetic), the `0x05` entropy-info seed expander, and the `0x0A`
//! packed-sample-word decoder (the modified-Rice zone ladder, unary
//! escape, phase-in mantissa, and adaptive medians) for every audio
//! block in the input, concatenating the per-block PCM. The return
//! value is intentionally discarded.
//!
//! **RSS sizing note:** decompression amplification is *inherent* to
//! the format — a ~50-byte block whose `0x0A` stream is a spec §4.2
//! step-1 zero-run legitimately decodes to up to
//! `MAX_DECODE_SAMPLES_PER_BLOCK` (`1 << 26`) zero samples (256 MiB of
//! `i32`s), because silence compresses enormously; the eager
//! `decode_stream` then concatenates per-block output across the
//! chain. That per-block ceiling is the documented anti-amplification
//! bound (see the round-296 hardening notes), not a leak, so campaigns
//! must run with an `-rss_limit_mb` sized for a few such expansions
//! (e.g. `-rss_limit_mb=8192`) or libFuzzer reports a spurious OOM
//! against its default 2 GiB limit. Callers needing hard memory bounds
//! use the lazy per-block iterator instead of the eager composer.

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = oxideav_wavpack::decode_stream(data);
});
