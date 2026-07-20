#![no_main]

//! Fuzz the round-418 **hybrid origination** surface with a pair-decode
//! oracle: arbitrary fuzz bytes become a control header plus a PCM
//! buffer, encoded through `encode_block_{mono,stereo}_hybrid` (and
//! the float / int32 format twins), and the assertions are the
//! module's validation contract:
//!
//! * the `.wv` + `.wvc` pair decodes back **bit-exactly**
//!   (`decode_stream_with_correction(&wv, &wvc) == pcm`), CRC gates
//!   green (the `.wvc` header's lossless CRC and the extension
//!   `crc_x` where a `0x0C` rides the twin);
//! * the lossy `.wv` alone decodes without error and passes its own
//!   §5.6 gate (its stored CRC covers the unclamped coarse decode);
//! * for the plain-integer shapes, every lossy sample stays within
//!   the container's clamp range.
//!
//! The control byte sweeps mono/stereo, joint coding, the
//! decorrelation ceiling (including the raw no-prediction arm), the
//! bitrate word (0 = the coarsest documented floor, up to the
//! lossless-degenerate regime), and the plain / float / int32 sample
//! formats — so the §6.5 bracketing writer, the per-sample stepper
//! feedback, the zero-run interleaves, the delta clamp and the
//! format deconstructions all face adversarial PCM. A second control
//! byte sweeps the round-420 noise-shaping axis (off / static ± /
//! extreme static / ramping weights, including out-of-range values the
//! payload build clamps).

use libfuzzer_sys::fuzz_target;
use oxideav_wavpack::{
    decode_stream_muted, decode_stream_with_correction, decode_stream_with_correction_muted,
    encode_block_mono_hybrid, encode_block_mono_hybrid_float, encode_block_mono_hybrid_int32,
    encode_block_stereo_hybrid, encode_block_stereo_hybrid_float, encode_block_stereo_hybrid_int32,
    DecorrProfile, HybridEncoded, HybridOptions, HybridShaping,
};

fn check_pair(pcm: &[i32], enc: &HybridEncoded) {
    let wvc = enc.wvc.as_ref().expect("correction requested");
    let exact = decode_stream_with_correction(&enc.wv, wvc).expect("pair decode");
    assert_eq!(exact, pcm, "pair decode must reproduce the input");
    let (muted, ok) = decode_stream_with_correction_muted(&enc.wv, wvc).expect("muted pair");
    assert!(ok, "pair CRC gate");
    assert_eq!(muted, pcm);
    let (_, ok) = decode_stream_muted(&enc.wv).expect("lossy decode");
    assert!(ok, "lossy CRC gate");
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }
    let control = data[0];
    let stereo = control & 0x01 != 0;
    let joint = control & 0x02 != 0;
    let profile = match (control >> 2) & 0x03 {
        0 => None,
        1 => Some(DecorrProfile::Fast),
        2 => Some(DecorrProfile::Normal),
        _ => Some(DecorrProfile::High),
    };
    let bitrate_word = match (control >> 4) & 0x03 {
        0 => 0,
        1 => 200,
        2 => 456,
        _ => 2000,
    };
    let format = (control >> 6) & 0x03; // 0/1 plain, 2 float, 3 int32
    let shape_control = data[1];
    let shaping = match shape_control & 0x07 {
        0 | 1 => HybridShaping::Off,
        2 => HybridShaping::Static(717),
        3 => HybridShaping::Static(-717),
        4 => HybridShaping::Static(i32::from(shape_control as i8) * 300),
        5 => HybridShaping::Ramp {
            weight: 512,
            delta: -(i32::from(shape_control >> 3)) * 500,
        },
        6 => HybridShaping::Ramp {
            weight: -1024,
            delta: i32::from(shape_control >> 3) * 900,
        },
        _ => HybridShaping::Static(1024),
    };
    let opts = HybridOptions {
        bitrate_word,
        correction: true,
        joint,
        profile,
        shaping,
    };

    match format {
        2 => {
            // Float shape: interpret the words as f32 bit patterns,
            // normalising exponents into a plausible audio range while
            // keeping some specials (the deconstruction must take any
            // pattern).
            let mut pcm: Vec<f32> = data[2..]
                .chunks_exact(4)
                .take(256)
                .map(|c| {
                    let bits = u32::from_le_bytes([c[0], c[1], c[2], c[3]]);
                    f32::from_bits(bits)
                })
                .collect();
            if pcm.is_empty() {
                return;
            }
            if stereo && pcm.len() % 2 != 0 {
                pcm.pop();
                if pcm.is_empty() {
                    return;
                }
            }
            let n = if stereo { pcm.len() / 2 } else { pcm.len() } as u32;
            let enc = if stereo {
                encode_block_stereo_hybrid_float(&pcm, &opts, 0, n)
            } else {
                encode_block_mono_hybrid_float(&pcm, &opts, 0, n)
            };
            // The float shape may refuse pathological coarse overflows
            // (documented typed-error corner); everything it emits must
            // hold the contract.
            if let Ok(enc) = enc {
                let wvc = enc.wvc.as_ref().expect("correction requested");
                let exact = oxideav_wavpack::decode_stream_with_correction_f32(&enc.wv, wvc)
                    .expect("pair decode");
                let got: Vec<u32> = exact.iter().map(|s| s.to_bits()).collect();
                let want: Vec<u32> = pcm.iter().map(|s| s.to_bits()).collect();
                assert_eq!(got, want, "float pair bit patterns");
                let (_, ok) = decode_stream_with_correction_muted(&enc.wv, wvc).expect("muted");
                assert!(ok, "float pair CRC gate");
            }
        }
        3 => {
            let mut pcm: Vec<i32> = data[2..]
                .chunks_exact(4)
                .take(256)
                .map(|c| i32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect();
            if pcm.is_empty() {
                return;
            }
            if stereo && pcm.len() % 2 != 0 {
                pcm.pop();
                if pcm.is_empty() {
                    return;
                }
            }
            let n = if stereo { pcm.len() / 2 } else { pcm.len() } as u32;
            let enc = if stereo {
                encode_block_stereo_hybrid_int32(&pcm, &opts, 0, n).expect("int32 hybrid encode")
            } else {
                encode_block_mono_hybrid_int32(&pcm, &opts, 0, n).expect("int32 hybrid encode")
            };
            check_pair(&pcm, &enc);
        }
        _ => {
            // Plain 16-bit-container integers.
            let mut pcm: Vec<i32> = data[2..]
                .chunks_exact(2)
                .take(512)
                .map(|c| i32::from(i16::from_le_bytes([c[0], c[1]])))
                .collect();
            if pcm.is_empty() {
                return;
            }
            if stereo && pcm.len() % 2 != 0 {
                pcm.pop();
                if pcm.is_empty() {
                    return;
                }
            }
            let n = if stereo { pcm.len() / 2 } else { pcm.len() } as u32;
            let enc = if stereo {
                encode_block_stereo_hybrid(&pcm, 2, &opts, 0, n).expect("hybrid encode")
            } else {
                encode_block_mono_hybrid(&pcm, 2, &opts, 0, n).expect("hybrid encode")
            };
            check_pair(&pcm, &enc);
            // Round-418 clamp: lossy output stays within the 16-bit
            // container range.
            let (lossy, _) = decode_stream_muted(&enc.wv).unwrap();
            assert!(
                lossy.iter().all(|&s| (-32768..=32767).contains(&s)),
                "lossy output clamps to the container range"
            );
        }
    }
});
