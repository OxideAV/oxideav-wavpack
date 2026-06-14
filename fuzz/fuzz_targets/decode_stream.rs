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

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = oxideav_wavpack::decode_stream(data);
});
