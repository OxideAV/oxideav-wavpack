#![no_main]

//! Fuzz the round-447 **multichannel origination** surface with a
//! round-trip oracle: arbitrary fuzz bytes become a control header
//! plus an interleaved PCM buffer, encoded through the stereo-pair
//! member-plan encoders, and the assertions are the surface's own
//! headline guarantees:
//!
//! * lossless (`best` / `smallest` / int32 / float): the emitted
//!   member-set chain decodes back **bit-exactly** through
//!   `decode_multichannel_stream` (bit patterns for the float shape),
//!   channel width preserved, per-member §5.6 CRC gates green;
//! * hybrid: the `.wv` + `.wvc` pair reproduces the input bit-exactly
//!   through `decode_multichannel_stream_with_correction` with green
//!   gates, and the lossy `.wv` alone decodes channel-complete within
//!   the 16-bit container clamp.
//!
//! The control bytes sweep the channel width (1..=8, so the
//! degenerate mono / stereo shapes and the odd-trailing-mono member
//! plan are all hit), the per-member search (`best` ceiling vs the
//! union `smallest`), the sample format (plain / float / int32 /
//! hybrid), the hybrid joint + shaping axes, and a small
//! `block_samples` so multi-set chains with running `block_index`
//! words are the common case.

use libfuzzer_sys::fuzz_target;
use oxideav_wavpack::{
    decode_multichannel_stream, decode_multichannel_stream_f32, decode_multichannel_stream_muted,
    decode_multichannel_stream_with_correction, decode_multichannel_stream_with_correction_muted,
    encode_multichannel_stream_best, encode_multichannel_stream_float,
    encode_multichannel_stream_hybrid, encode_multichannel_stream_int32,
    encode_multichannel_stream_smallest, DecorrProfile, HybridOptions, HybridShaping,
};

fuzz_target!(|data: &[u8]| {
    if data.len() < 7 {
        return;
    }
    let channels = 1 + usize::from(data[0] % 8);
    let mode = data[1] % 5; // 0 best, 1 smallest, 2 float, 3 int32, 4 hybrid
    let profile = match data[2] & 0x03 {
        0 => DecorrProfile::Fast,
        1 => DecorrProfile::Normal,
        2 => DecorrProfile::High,
        _ => DecorrProfile::Extra,
    };
    // Small per-set frame counts keep runs fast and make multi-set
    // chains (running block_index, repeated 0x0D emission) common.
    let block_samples = match (data[2] >> 2) & 0x03 {
        0 => 3,
        1 => 7,
        2 => 16,
        _ => 61,
    };
    let joint = data[2] & 0x10 != 0;
    let shaping = match (data[2] >> 5) & 0x03 {
        0 => HybridShaping::Off,
        1 => HybridShaping::Static(717),
        2 => HybridShaping::Static(-1024),
        _ => HybridShaping::Ramp {
            weight: -512,
            delta: -40_000,
        },
    };

    // Trim the raw samples to whole interleaved frames.
    let whole = |mut v: Vec<i32>| {
        v.truncate(v.len() - v.len() % channels);
        v
    };

    match mode {
        2 => {
            // Float: any bit pattern is a legal sample (NaN payloads,
            // infinities, denormals included).
            let raw: Vec<i32> = data[3..]
                .chunks_exact(4)
                .take(360)
                .map(|c| i32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect();
            let pcm: Vec<f32> = whole(raw)
                .into_iter()
                .map(|bits| f32::from_bits(bits as u32))
                .collect();
            if pcm.is_empty() {
                return;
            }
            let wv = encode_multichannel_stream_float(&pcm, channels, block_samples, profile)
                .expect("float multichannel encode");
            let decoded = decode_multichannel_stream_f32(&wv).expect("own chain must decode");
            assert_eq!(decoded.channels, channels);
            let got: Vec<u32> = decoded.samples.iter().map(|s| s.to_bits()).collect();
            let want: Vec<u32> = pcm.iter().map(|s| s.to_bits()).collect();
            assert_eq!(got, want, "float multichannel bit patterns");
            let (_, ok) = decode_multichannel_stream_muted(&wv).expect("muted decode");
            assert!(ok, "per-member CRC gates");
        }
        3 => {
            // Int32: the full i32 range rides the 0x09 deconstruction.
            let pcm = whole(
                data[3..]
                    .chunks_exact(4)
                    .take(360)
                    .map(|c| i32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                    .collect(),
            );
            if pcm.is_empty() {
                return;
            }
            let wv = encode_multichannel_stream_int32(&pcm, channels, block_samples, profile)
                .expect("int32 multichannel encode");
            let decoded = decode_multichannel_stream(&wv).expect("own chain must decode");
            assert_eq!(decoded.channels, channels);
            assert_eq!(decoded.samples, pcm, "int32 multichannel round trip");
            let (muted, ok) = decode_multichannel_stream_muted(&wv).expect("muted decode");
            assert!(ok, "per-member CRC gates");
            assert_eq!(muted.samples, pcm);
        }
        4 => {
            // Hybrid pair over 16-bit-container integers.
            let pcm = whole(
                data[3..]
                    .chunks_exact(2)
                    .take(600)
                    .map(|c| i32::from(i16::from_le_bytes([c[0], c[1]])))
                    .collect(),
            );
            if pcm.is_empty() {
                return;
            }
            let opts = HybridOptions {
                bitrate_word: match data[0] >> 4 {
                    0 => 0,
                    v if v < 8 => 456,
                    _ => 2000,
                },
                correction: true,
                joint,
                profile: Some(profile),
                shaping,
            };
            let enc = encode_multichannel_stream_hybrid(&pcm, channels, block_samples, 2, &opts)
                .expect("hybrid multichannel encode");
            let wvc = enc.wvc.as_ref().expect("correction requested");
            let exact =
                decode_multichannel_stream_with_correction(&enc.wv, wvc).expect("pair decode");
            assert_eq!(exact.channels, channels);
            assert_eq!(exact.samples, pcm, "hybrid multichannel pair losslessness");
            let (muted, ok) =
                decode_multichannel_stream_with_correction_muted(&enc.wv, wvc).expect("muted pair");
            assert!(ok, "pair CRC gates");
            assert_eq!(muted.samples, pcm);
            let lossy = decode_multichannel_stream(&enc.wv).expect("lossy decode");
            assert_eq!(lossy.channels, channels);
            assert_eq!(lossy.samples.len(), pcm.len());
            assert!(
                lossy.samples.iter().all(|&s| (-32768..=32767).contains(&s)),
                "lossy output clamps to the container range"
            );
        }
        _ => {
            // Lossless integer members through best / smallest.
            let pcm = whole(
                data[3..]
                    .chunks_exact(2)
                    .take(600)
                    .map(|c| i32::from(i16::from_le_bytes([c[0], c[1]])))
                    .collect(),
            );
            if pcm.is_empty() {
                return;
            }
            let wv = if mode == 0 {
                encode_multichannel_stream_best(&pcm, channels, block_samples, 2, profile)
            } else {
                encode_multichannel_stream_smallest(&pcm, channels, block_samples, 2)
            }
            .expect("lossless multichannel encode");
            let decoded = decode_multichannel_stream(&wv).expect("own chain must decode");
            assert_eq!(decoded.channels, channels);
            assert_eq!(decoded.samples, pcm, "lossless multichannel round trip");
            let (muted, ok) = decode_multichannel_stream_muted(&wv).expect("muted decode");
            assert!(ok, "per-member CRC gates");
            assert_eq!(muted.samples, pcm);
        }
    }
});
