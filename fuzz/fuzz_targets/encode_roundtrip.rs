#![no_main]

//! Fuzz the round-383 self-deriving encoder surface with a **round-trip
//! oracle**: arbitrary fuzz bytes are interpreted as a small control
//! header plus an `i32` PCM buffer, encoded through the best-of mode
//! search ([`oxideav_wavpack::encode_block_mono_best`] /
//! [`oxideav_wavpack::encode_block_stereo_best`]), and the encoded block
//! is decoded back — the assertion is the crate's own headline
//! guarantee, `decode_stream(&encode(...)) == pcm`, bit-exact for every
//! input.
//!
//! This is strictly stronger than the panic-freedom contract of the
//! decode targets: any divergence anywhere in the derive → serialize →
//! recorrelate → entropy-encode → decode pipeline (trained-weight
//! quantization, seed log-packing, the §3.7 pass-order reversal, the
//! joint mid/side truncation cancellation, the left-shift narrow /
//! restore pair, the §5 CRC gate) fails the assert. The control byte
//! sweeps the mode grid: mono vs stereo shape, the search-ceiling
//! profile, and a synthetic left-shift scaling applied to the samples so
//! the shift-detection arm is exercised.

use libfuzzer_sys::fuzz_target;
use oxideav_wavpack::{
    decode_stream, decode_stream_muted, encode_block_mono_best, encode_block_stereo_best,
    DecorrProfile,
};

fuzz_target!(|data: &[u8]| {
    if data.len() < 5 {
        return;
    }
    let control = data[0];
    let stereo = control & 0x01 != 0;
    let profile = match (control >> 1) & 0x03 {
        0 => DecorrProfile::Fast,
        1 => DecorrProfile::Normal,
        2 => DecorrProfile::High,
        _ => DecorrProfile::Extra,
    };
    // A 0..=3 bit scale so some inputs detect a non-zero left shift.
    let scale = (control >> 3) & 0x03;

    // Interpret the remaining bytes as little-endian i32 samples,
    // bounded so a fuzz run stays fast. Keep the samples within a
    // 24-bit-ish range so the synthetic shift cannot overflow.
    let mut pcm: Vec<i32> = data[1..]
        .chunks_exact(4)
        .take(512)
        .map(|c| (i32::from_le_bytes([c[0], c[1], c[2], c[3]]) >> 8) << scale)
        .collect();
    if pcm.is_empty() {
        return;
    }
    if stereo && !pcm.len().is_multiple_of(2) {
        pcm.pop();
        if pcm.is_empty() {
            return;
        }
    }

    let encoded = if stereo {
        encode_block_stereo_best(&pcm, profile, 2, 0, (pcm.len() / 2) as u32)
    } else {
        encode_block_mono_best(&pcm, profile, 2, 0, pcm.len() as u32)
    }
    .expect("best-of encode of a non-empty even buffer must succeed");

    let decoded = decode_stream(&encoded).expect("own block must decode");
    assert_eq!(decoded, pcm, "lossless round-trip violated");

    let (muted, crc_ok) = decode_stream_muted(&encoded).expect("muted decode");
    assert!(crc_ok, "own block must pass its CRC gate");
    assert_eq!(muted, pcm);
});
