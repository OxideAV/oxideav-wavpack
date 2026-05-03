//! Bit-exact lossless round-trip on **digital silence** through the
//! system `ffmpeg` WavPack encoder.
//!
//! Silence is a clean exercise of the file walker, the block parser,
//! the sub-block walker, the entropy decoder's zero-run shortcut
//! (spec §5.4 step 1), and the per-block CRC. The decorrelation
//! cascade and median update path never see non-zero residuals on
//! this input — they are exercised by the (currently `#[ignore]`)
//! sine roundtrips in `tests/ffmpeg_roundtrip.rs`.

#![allow(clippy::needless_range_loop)]

use std::process::{Command, Stdio};

use oxideav_wavpack::container::{decode_frame, parse_file};

fn have_ffmpeg() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

#[test]
fn stereo_16bit_silence_roundtrip_bit_exact() {
    if !have_ffmpeg() {
        eprintln!("skipping silence roundtrip: no ffmpeg");
        return;
    }
    let dir = std::env::temp_dir();
    let id = format!("oxideav-wp-silence-stereo-{}", std::process::id());
    let wav = dir.join(format!("{id}.wav"));
    let wv = dir.join(format!("{id}.wv"));

    let status = Command::new("ffmpeg")
        .args([
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "lavfi",
            "-i",
            "anullsrc=channel_layout=stereo:sample_rate=44100:duration=0.05",
            "-c:a",
            "pcm_s16le",
        ])
        .arg(&wav)
        .status()
        .expect("ffmpeg run");
    assert!(status.success());

    let status = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error", "-i"])
        .arg(&wav)
        .args(["-c:a", "wavpack", "-compression_level", "0"])
        .arg(&wv)
        .status()
        .expect("ffmpeg run");
    assert!(status.success());

    let wav_bytes = std::fs::read(&wav).unwrap();
    let wv_bytes = std::fs::read(&wv).unwrap();
    let _ = std::fs::remove_file(&wav);
    let _ = std::fs::remove_file(&wv);

    let parsed = parse_file(&wv_bytes).expect("parse .wv");
    let expected_pcm = extract_data(&wav_bytes);
    let mut got_pcm: Vec<u8> = Vec::new();
    for frame in &parsed.frames {
        let chans = decode_frame(&wv_bytes, frame).expect("decode frame");
        let n = chans[0].len();
        for i in 0..n {
            for c in 0..chans.len() {
                got_pcm.extend_from_slice(&(chans[c][i] as i16).to_le_bytes());
            }
        }
    }
    assert_eq!(got_pcm.len(), expected_pcm.len());
    assert_eq!(got_pcm, expected_pcm, "PCM bytes differ on stereo silence");
}

#[test]
fn stereo_16bit_long_silence_roundtrip_bit_exact() {
    // Longer duration triggers ffmpeg's encoder to emit *multiple*
    // blocks (block_samples ≈ sample_rate / 2 = 22050 → at 1.0 s we
    // get exactly 2 blocks per channel-pair).
    if !have_ffmpeg() {
        eprintln!("skipping silence roundtrip: no ffmpeg");
        return;
    }
    let dir = std::env::temp_dir();
    let id = format!("oxideav-wp-silence-long-{}", std::process::id());
    let wav = dir.join(format!("{id}.wav"));
    let wv = dir.join(format!("{id}.wv"));

    let status = Command::new("ffmpeg")
        .args([
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "lavfi",
            "-i",
            "anullsrc=channel_layout=stereo:sample_rate=44100:duration=1.0",
            "-c:a",
            "pcm_s16le",
        ])
        .arg(&wav)
        .status()
        .expect("ffmpeg run");
    assert!(status.success());

    let status = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error", "-i"])
        .arg(&wav)
        .args(["-c:a", "wavpack", "-compression_level", "0"])
        .arg(&wv)
        .status()
        .expect("ffmpeg run");
    assert!(status.success());

    let wav_bytes = std::fs::read(&wav).unwrap();
    let wv_bytes = std::fs::read(&wv).unwrap();
    let _ = std::fs::remove_file(&wav);
    let _ = std::fs::remove_file(&wv);

    let parsed = parse_file(&wv_bytes).expect("parse .wv");
    assert!(
        parsed.frames.len() >= 2,
        "expected multi-block file, got {} frames",
        parsed.frames.len()
    );
    let expected_pcm = extract_data(&wav_bytes);
    let mut got_pcm: Vec<u8> = Vec::new();
    for frame in &parsed.frames {
        let chans = decode_frame(&wv_bytes, frame).expect("decode frame");
        let n = chans[0].len();
        for i in 0..n {
            for c in 0..chans.len() {
                got_pcm.extend_from_slice(&(chans[c][i] as i16).to_le_bytes());
            }
        }
    }
    assert_eq!(got_pcm.len(), expected_pcm.len());
    assert_eq!(got_pcm, expected_pcm);
}

#[test]
fn mono_16bit_silence_roundtrip_bit_exact() {
    if !have_ffmpeg() {
        eprintln!("skipping silence roundtrip: no ffmpeg");
        return;
    }
    let dir = std::env::temp_dir();
    let id = format!("oxideav-wp-silence-{}", std::process::id());
    let wav = dir.join(format!("{id}.wav"));
    let wv = dir.join(format!("{id}.wv"));

    let status = Command::new("ffmpeg")
        .args([
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "lavfi",
            "-i",
            "anullsrc=channel_layout=mono:sample_rate=44100:duration=0.05",
            "-c:a",
            "pcm_s16le",
        ])
        .arg(&wav)
        .status()
        .expect("ffmpeg run");
    assert!(status.success(), "WAV gen failed");

    let status = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error", "-i"])
        .arg(&wav)
        .args(["-c:a", "wavpack", "-compression_level", "0"])
        .arg(&wv)
        .status()
        .expect("ffmpeg run");
    assert!(status.success(), "WavPack encode failed");

    let wav_bytes = std::fs::read(&wav).unwrap();
    let wv_bytes = std::fs::read(&wv).unwrap();
    let _ = std::fs::remove_file(&wav);
    let _ = std::fs::remove_file(&wv);

    let parsed = parse_file(&wv_bytes).expect("parse .wv");
    assert!(!parsed.frames.is_empty(), "no frames parsed");

    let expected_pcm = extract_data(&wav_bytes);
    let mut got_pcm: Vec<u8> = Vec::new();
    for frame in &parsed.frames {
        let chans = decode_frame(&wv_bytes, frame).expect("decode frame");
        assert_eq!(chans.len(), 1, "expected mono");
        for s in &chans[0] {
            got_pcm.extend_from_slice(&(*s as i16).to_le_bytes());
        }
    }
    assert_eq!(
        got_pcm.len(),
        expected_pcm.len(),
        "PCM length mismatch: got {} vs expected {}",
        got_pcm.len(),
        expected_pcm.len()
    );
    assert_eq!(got_pcm, expected_pcm, "PCM bytes differ on silence");
}

fn extract_data(wav: &[u8]) -> Vec<u8> {
    let mut i = 12;
    while i + 8 <= wav.len() {
        if &wav[i..i + 4] == b"data" {
            let size =
                u32::from_le_bytes([wav[i + 4], wav[i + 5], wav[i + 6], wav[i + 7]]) as usize;
            return wav[i + 8..i + 8 + size].to_vec();
        }
        let sz = u32::from_le_bytes([wav[i + 4], wav[i + 5], wav[i + 6], wav[i + 7]]) as usize;
        i += 8 + sz + (sz & 1);
    }
    panic!("no data chunk");
}
