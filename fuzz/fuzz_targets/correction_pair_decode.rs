#![no_main]

//! Differential fuzz over the round-415 hybrid-lossless **pair decode**
//! surface — the `.wv` + `.wvc` two-file paths (plain, §5.6-muted,
//! multichannel) driven from an arbitrary fuzz-chosen split of the
//! input into (main, correction) halves.
//!
//! Contract under test (panic / overflow / OOM freedom, plus the
//! cross-path invariants):
//!
//! - **plain / muted parity** — `decode_stream_with_correction` and its
//!   muted twin agree on success (same pipeline, the gate only zeroes),
//!   emit the same PCM length, and are bit-identical whenever every
//!   block's CRC gate passed (`all_ok`);
//! - **empty-correction identity** — a pair decode against an empty
//!   correction chain is exactly the single-file lossy decode (every
//!   block pairs `None` and falls back);
//! - **multichannel parity** — on 1/2-channel streams (every block a
//!   standalone set) the multichannel pair walker returns the same PCM
//!   as the plain pair path, and its muted twin obeys the same
//!   `all_ok ⇒ bit-identical` rule.

use libfuzzer_sys::fuzz_target;
use oxideav_wavpack::{
    decode_multichannel_stream_with_correction, decode_multichannel_stream_with_correction_muted,
    decode_stream, decode_stream_with_correction, decode_stream_with_correction_f32,
    decode_stream_with_correction_muted,
};

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }
    // 16-bit split fraction: fine enough to land exactly on a real
    // .wv/.wvc boundary for corpus-sized inputs.
    let control = u16::from_le_bytes([data[0], data[1]]) as usize;
    let bytes = &data[2..];
    let split = (control * bytes.len()) / 65536;
    let (main, wvc) = bytes.split_at(split.min(bytes.len()));

    // Plain vs muted pair decode. The gate mirrors `decode_samples_muted`:
    // when a block's CRC fails, the muted path mutes it WITHOUT running
    // the sample-format fixups — so it may return `Ok` (with
    // `all_ok == false`) where the plain path surfaces a fixup error.
    // The valid cross-invariants are:
    // - both Ok: same PCM shape; bit-identical when every gate passed;
    // - plain Err + muted Ok: only possible with a failed gate;
    // - muted Err implies plain Err (the muted path runs a subset).
    let plain = decode_stream_with_correction(main, wvc);
    let muted = decode_stream_with_correction_muted(main, wvc);
    match (&plain, &muted) {
        (Ok(p), Ok((m, all_ok))) => {
            assert_eq!(p.len(), m.len(), "gate must not change the PCM shape");
            if *all_ok {
                assert_eq!(p, m, "clean gate must be bit-identical");
            }
        }
        (Err(_), Ok((_, all_ok))) => {
            assert!(
                !all_ok,
                "muted path may only out-succeed the plain path on a failed gate"
            );
        }
        (Err(_), Err(_)) => {}
        (Ok(_), Err(_)) => panic!("muted pair decode errored where plain succeeded"),
    }

    // The f32 twin must never panic (refusal or reinterpretation only).
    let _ = decode_stream_with_correction_f32(main, wvc);

    // Empty-correction identity: every block pairs None and falls back
    // to its lossy decode.
    match (decode_stream_with_correction(main, &[]), decode_stream(main)) {
        (Ok(pair), Ok(lossy)) => {
            assert_eq!(pair, lossy, "empty correction must be the lossy decode");
        }
        (Err(_), Err(_)) | (Err(_), Ok(_)) | (Ok(_), Err(_)) => {}
    }

    // Multichannel pair walker: never panics; on 1/2-channel streams it
    // matches the plain pair path sample-for-sample.
    let mc = decode_multichannel_stream_with_correction(main, wvc);
    if let (Ok(p), Ok(d)) = (&plain, &mc) {
        if d.channels <= 2 {
            assert_eq!(
                p, &d.samples,
                "stereo-or-narrower streams must decode identically"
            );
        }
    }
    if let Ok((dm, all_ok)) = decode_multichannel_stream_with_correction_muted(main, wvc) {
        if let Ok(d) = &mc {
            assert_eq!(d.samples.len(), dm.samples.len());
            if all_ok {
                assert_eq!(d.samples, dm.samples);
            }
        }
    }
});
