//! Smoke test that prints the first 16 decoded samples side-by-side
//! with the source PCM, for tuning the entropy / decorrelation
//! pipeline against ffmpeg-encoded fixtures.
//!
//! Run with `cargo test --test inspect -- --nocapture inspect_mono`.

use std::process::{Command, Stdio};

use oxideav_wavpack::block::parse_sub_blocks;
use oxideav_wavpack::container::parse_file;
use oxideav_wavpack::decoder::decode_block_samples_no_crc;

fn have_ffmpeg() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

#[ignore = "diagnostic only — prints first samples from the decoded ffmpeg stream"]
#[test]
fn inspect_mono_sine() {
    if !have_ffmpeg() {
        eprintln!("skip: no ffmpeg");
        return;
    }
    let dir = tempfile_dir();
    let wav = dir.join("src.wav");
    let wv = dir.join("enc.wv");
    Command::new("ffmpeg")
        .args([
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "lavfi",
            "-i",
            "sine=frequency=440:sample_rate=44100:duration=0.05:beep_factor=0",
            "-c:a",
            "pcm_s16le",
        ])
        .arg(&wav)
        .status()
        .unwrap();
    Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error", "-i"])
        .arg(&wav)
        .args(["-c:a", "wavpack", "-compression_level", "0"])
        .arg(&wv)
        .status()
        .unwrap();

    let wav_bytes = std::fs::read(&wav).unwrap();
    let wv_bytes = std::fs::read(&wv).unwrap();
    let pcm = extract_data(&wav_bytes);
    let src_samples: Vec<i16> = pcm
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();

    let parsed = parse_file(&wv_bytes).unwrap();
    eprintln!("frames: {}", parsed.frames.len());
    let frame = &parsed.frames[0];
    eprintln!(
        "first block flags={:#010x} samples={} crc={:#010x}",
        frame.blocks[0].header.flags,
        frame.blocks[0].header.block_samples,
        frame.blocks[0].header.crc,
    );
    // Use the no-CRC entry so we can dump samples even if the
    // pipeline isn't bit-exact yet.
    let blk = &frame.blocks[0];
    let payload = blk.payload(&wv_bytes);
    let subs = parse_sub_blocks(payload).unwrap();
    let chans = decode_block_samples_no_crc(&blk.header, &subs).expect("decode");
    eprintln!("decoded channels: {}", chans.len());

    let n_show = 16.min(src_samples.len()).min(chans[0].len());
    eprintln!("\nidx |   source |  decoded | diff");
    for i in 0..n_show {
        let src = src_samples[i] as i32;
        let dec = chans[0][i];
        eprintln!("{i:3} | {src:8} | {dec:8} | {}", dec - src);
    }
}

fn extract_data(wav: &[u8]) -> Vec<u8> {
    let mut i = 12;
    while i + 8 < wav.len() {
        if &wav[i..i + 4] == b"data" {
            let size =
                u32::from_le_bytes([wav[i + 4], wav[i + 5], wav[i + 6], wav[i + 7]]) as usize;
            return wav[i + 8..i + 8 + size].to_vec();
        }
        let sz = u32::from_le_bytes([wav[i + 4], wav[i + 5], wav[i + 6], wav[i + 7]]) as usize;
        i += 8 + sz + (sz & 1);
    }
    Vec::new()
}

fn tempfile_dir() -> std::path::PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!("oxideav-wavpack-inspect-{}", std::process::id()));
    let _ = std::fs::create_dir_all(&p);
    p
}
