//! Single-thread speed and size measurements.
//!
//! ```text
//! cargo run --release --example wvbench -- encode source.wav [runs]
//! cargo run --release --example wvbench -- decode file.wv [file.wvc] [runs]
//! ```
//!
//! `encode` reads a 16-bit PCM WAV (mono or stereo), encodes it with
//! every lossless profile and a hybrid setting, and prints the stream
//! size, the size as a share of the PCM bytes, and encode / decode
//! speed as a multiple of real time. `decode` times this crate's
//! decoder on any `.wv` file (optionally with its correction file).
//! Every figure is the median of `runs` runs (default 5) on one thread,
//! with the fastest run alongside (the steadier number on a busy host).

use std::time::Instant;

use oxideav_wavpack::{
    DecorrProfile, HybridOptions, decode_multichannel_stream,
    decode_multichannel_stream_with_correction, encode_stream_mono_best, encode_stream_mono_hybrid,
    encode_stream_mono_smallest, encode_stream_stereo_best, encode_stream_stereo_hybrid,
    encode_stream_stereo_smallest, set_stream_sample_rate, stream_sample_rate,
};

/// Median and fastest wall time of `runs` runs.
fn timed(runs: usize, mut f: impl FnMut()) -> (f64, f64) {
    let mut times: Vec<f64> = (0..runs.max(1))
        .map(|_| {
            let start = Instant::now();
            f();
            start.elapsed().as_secs_f64()
        })
        .collect();
    times.sort_by(|a, b| a.total_cmp(b));
    (times[times.len() / 2], times[0])
}

struct Wav {
    channels: usize,
    rate: u32,
    samples: Vec<i32>,
}

fn read_wav16(path: &str) -> Wav {
    let d = std::fs::read(path).expect("read wav");
    assert!(
        d.len() > 44 && &d[..4] == b"RIFF" && &d[8..12] == b"WAVE",
        "not a RIFF/WAVE file"
    );
    let mut at = 12;
    let (mut channels, mut rate, mut bits) = (0usize, 0u32, 0u16);
    while at + 8 <= d.len() {
        let size = u32::from_le_bytes([d[at + 4], d[at + 5], d[at + 6], d[at + 7]]) as usize;
        let body = &d[at + 8..(at + 8 + size).min(d.len())];
        match &d[at..at + 4] {
            b"fmt " => {
                channels = usize::from(u16::from_le_bytes([body[2], body[3]]));
                rate = u32::from_le_bytes([body[4], body[5], body[6], body[7]]);
                bits = u16::from_le_bytes([body[14], body[15]]);
            }
            b"data" => {
                assert_eq!(bits, 16, "wvbench reads 16-bit PCM only");
                assert!(
                    (1..=2).contains(&channels),
                    "wvbench reads mono or stereo only"
                );
                let samples = body
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .map(|c| i32::from(i16::from_le_bytes(*c)))
                    .collect();
                return Wav {
                    channels,
                    rate,
                    samples,
                };
            }
            _ => {}
        }
        at += 8 + size + (size & 1);
    }
    panic!("no data chunk");
}

fn report(label: &str, wav: &Wav, runs: usize, encode: &dyn Fn() -> Vec<u8>) {
    let seconds = wav.samples.len() as f64 / wav.channels as f64 / f64::from(wav.rate);
    let pcm_bytes = wav.samples.len() * 2;
    let stream = set_stream_sample_rate(&encode(), wav.rate).expect("stamp rate");
    let (enc, enc_best) = timed(runs, || {
        std::hint::black_box(encode());
    });
    let (dec, dec_best) = timed(runs, || {
        std::hint::black_box(decode_multichannel_stream(&stream).expect("decode"));
    });
    println!(
        "{label:<18} {:>9} bytes  {:>5.1} %   encode {:>7.1} x (best {:>7.1})   decode {:>7.1} x (best {:>7.1})",
        stream.len(),
        100.0 * stream.len() as f64 / pcm_bytes as f64,
        seconds / enc,
        seconds / enc_best,
        seconds / dec,
        seconds / dec_best,
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let usage =
        "usage: wvbench encode <source.wav> [runs] | wvbench decode <file.wv> [file.wvc] [runs]";
    let runs = args
        .last()
        .and_then(|a| a.parse::<usize>().ok())
        .unwrap_or(5);
    match args.get(1).map(String::as_str) {
        Some("encode") => {
            let wav = read_wav16(args.get(2).expect(usage));
            let pcm = &wav.samples;
            let stereo = wav.channels == 2;
            println!(
                "{} — {} ch, {} Hz, {:.1} s, {} PCM bytes; median of {runs} run(s), one thread",
                args[2],
                wav.channels,
                wav.rate,
                pcm.len() as f64 / wav.channels as f64 / f64::from(wav.rate),
                pcm.len() * 2
            );
            for profile in [
                DecorrProfile::Fast,
                DecorrProfile::Normal,
                DecorrProfile::High,
                DecorrProfile::Extra,
            ] {
                report(&format!("best {profile:?}"), &wav, runs, &|| {
                    if stereo {
                        encode_stream_stereo_best(pcm, 0, 2, profile)
                    } else {
                        encode_stream_mono_best(pcm, 0, 2, profile)
                    }
                    .expect("encode")
                });
            }
            report("smallest", &wav, runs.min(3), &|| {
                if stereo {
                    encode_stream_stereo_smallest(pcm, 0, 2)
                } else {
                    encode_stream_mono_smallest(pcm, 0, 2)
                }
                .expect("encode")
            });
            let opts = HybridOptions::from_bits_per_sample(4.0).with_correction(false);
            report("hybrid 4 bits", &wav, runs, &|| {
                if stereo {
                    encode_stream_stereo_hybrid(pcm, 0, 2, &opts)
                } else {
                    encode_stream_mono_hybrid(pcm, 0, 2, &opts)
                }
                .expect("encode")
                .wv
            });
        }
        Some("decode") => {
            let wv = std::fs::read(args.get(2).expect(usage)).expect("read wv");
            let wvc = args
                .get(3)
                .filter(|a| a.parse::<usize>().is_err())
                .map(|p| std::fs::read(p).expect("read wvc"));
            let decode = || {
                match &wvc {
                    Some(c) => decode_multichannel_stream_with_correction(&wv, c),
                    None => decode_multichannel_stream(&wv),
                }
                .expect("decode")
            };
            let decoded = decode();
            let frames = decoded.samples.len() / decoded.channels.max(1);
            let rate = stream_sample_rate(&wv).expect("rate").unwrap_or(44_100);
            let seconds = frames as f64 / f64::from(rate);
            let (t, best) = timed(runs, || {
                std::hint::black_box(decode());
            });
            println!(
                "{} — {} ch, {rate} Hz, {seconds:.1} s, {} bytes: decode {:.1} x real time ({:.1} ms; best {:.1} x; median of {runs} run(s), one thread)",
                args[2],
                decoded.channels,
                wv.len(),
                seconds / t,
                t * 1e3,
                seconds / best
            );
        }
        _ => {
            eprintln!("{usage}");
            std::process::exit(2);
        }
    }
}
