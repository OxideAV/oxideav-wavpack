#![no_main]

//! Decode arbitrary fuzz-supplied bytes through the WavPack multichannel
//! grouping decoder.
//!
//! The contract under test is purely that the call *returns*: a malformed
//! stream — including one with an inconsistent wiki bits-11..=12 member
//! grouping (a stray final marker, an unterminated set, members of one set
//! disagreeing on `block_samples`, or a set whose summed channel count
//! blows past the bound) — yields `Err(oxideav_wavpack::Error::…)`, a
//! well-formed one yields `Ok(DecodedStream)`, and neither path may panic,
//! integer-overflow (in a debug build), index out of bounds, or allocate
//! an attacker-controlled buffer sized from the raw header fields rather
//! than from the bounded payload.
//!
//! [`oxideav_wavpack::decode_multichannel_stream`] drives the same per-
//! member decode path [`decode_stream`] does (header validation, the
//! metadata walker, the `0x05` seed expander, the `0x0A` modified-Rice
//! sample-word decoder, decorrelation and joint-stereo undo) plus the
//! grouping state machine that stitches member blocks into interleaved
//! multichannel frames. The companion target exercises the CRC-muted
//! variant so the per-member mute gate is fuzzed too. The return value is
//! intentionally discarded.

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = oxideav_wavpack::decode_multichannel_stream(data);
    let _ = oxideav_wavpack::decode_multichannel_stream_muted(data);
    let _ = oxideav_wavpack::multichannel_layout(data);
});
