//! Foreign-file (reference-encoded) decode conformance.
//!
//! The fixtures under `tests/data/` were produced **black-box** with the
//! reference WavPack 5.9.0 command-line tools (round 405): synthetic
//! deterministic WAV content (sine mixtures + seeded noise) encoded with
//! `wavpack` across its lossless effort modes, and the expected PCM taken
//! from `wvunpack --raw` (the reference decode of the same file),
//! converted to little-endian `i32` container values. No reference source
//! code was consulted — the binaries were used strictly as opaque
//! encode/decode oracles per the workspace clean-room policy.
//!
//! Every `.wv` here carries what this crate's own encoder never writes:
//! non-zero `0x05` medians, non-zero `0x04` decorrelation seeds (stored
//! for a wire-order prefix of the pass list), reference flag conventions
//! (bit 28, `CROSS_DECORR` on lossless stereo) and multi-term stacks —
//! so this battery pins the staged `wp_log2`/`wp_exp2s` log-domain
//! conversions (`docs/audio/wavpack/spec/wavpack-log2-exp2.md`) and the
//! seed-prefix rule end-to-end: decode must be **bit-exact** and the
//! stored block CRCs must match.
//!
//! Fixture matrix:
//!
//! | fixture                        | encoder mode | shape            |
//! | ------------------------------ | ------------ | ---------------- |
//! | `foreign_default_mono16`       | default      | 16-bit mono      |
//! | `foreign_default_stereo16`     | default      | 16-bit stereo    |
//! | `foreign_fast_stereo16`        | `-f`         | 16-bit stereo    |
//! | `foreign_high_stereo16`        | `-h`         | 16-bit stereo    |
//! | `foreign_vhigh_stereo16`       | `-hh`        | 16-bit stereo    |
//! | `foreign_vhigh_x4_stereo16`    | `-hh -x4`    | 16-bit stereo    |
//! | `foreign_default_mono8`        | default      | 8-bit mono       |
//! | `foreign_default_stereo24`     | default      | 24-bit stereo    |
//! | `foreign_default_5dot1_16`     | default      | 16-bit 5.1       |
//! | `foreign_custom_rate_mono16`   | default      | 16-bit mono, non-standard 12345 Hz (`0x27`) |
//! | `foreign_int32_mono`           | default      | 32-bit int mono (`INT32_DATA`, `0x09` + `0x0C` sent bits) |
//! | `foreign_int32_zeros_mono`     | default      | 32-bit int mono, 12 redundant trailing zero bits |
//! | `foreign_int32_stereo`         | default      | 32-bit int stereo |
//! | `foreign_int32_high_stereo`    | `-h`         | 32-bit int stereo |
//! | `foreign_float_intvalued_mono` | default      | 32-bit float mono, integer-valued (`float_flags = 0`) |
//! | `foreign_float_mono`           | default      | 32-bit float mono, full precision (`SHIFT_SENT`) |
//! | `foreign_float_zeros_mono`     | default      | 32-bit float mono with ±0 (`SHIFT_SENT\|ZEROS_SENT\|NEG_ZEROS`) |
//! | `foreign_float_stereo`         | default      | 32-bit float stereo, >1.0 amplitudes + denormals + ±0 |
//! | `foreign_float_high_mono`      | `-h`         | 32-bit float mono, full precision |
//! | `foreign_float_shift_same`     | default      | 32-bit float mono, `SHIFT_SAME` one-bit carrier (round 408) |
//! | `foreign_float_shift_same_zeros` | default    | `SHIFT_SAME` interleaved with exact zeros (no carrier bit on zeros) |
//! | `foreign_float_shift_ones`     | default      | 32-bit float mono, `SHIFT_ONES` (no `0x0C` stream) |
//! | `foreign_float_exceptions`     | default      | `EXCEPTIONS\|SHIFT_SENT` — scattered ±inf / NaN payloads |
//! | `foreign_float_exc_stereo`     | default      | stereo `EXCEPTIONS\|SHIFT_SENT\|ZEROS_SENT\|NEG_ZEROS` combo |
//! | `foreign_float_exc_tiny`       | default      | 64-sample `EXCEPTIONS` + short `0x03` weights prefix |
//! | `foreign_float_exc_mix`        | default      | `EXCEPTIONS` with `float_shift = 1` sentinel scaling |
//! | `foreign_hybrid_mono_b4`       | `-b4`        | 16-bit mono hybrid (lossy), 4 bits/sample (round 408) |
//! | `foreign_hybrid_mono_b2`       | `-b2`        | 16-bit mono hybrid at the minimum bitrate |
//! | `foreign_hybrid_multiblock_b3` | `-b3`        | 16-bit mono hybrid, 2 blocks + a silence stretch (zero-run + level decay) |
//! | `foreign_hybrid_stereo_b4`     | `-b4`        | 16-bit stereo hybrid, mid/side + balance, unbalanced levels + silence |
//! | `foreign_hybrid_lr_b4`         | `-b4 -j0`    | 16-bit stereo hybrid, left/right coding (balance word 0) |
//! | `foreign_hybrid_5dot1_b4`      | `-b4`        | 16-bit 5.1 hybrid (stereo + mono members) |
//! | `foreign_hybrid_stereo24_b4`   | `-b4`        | 24-bit stereo hybrid |
//! | `foreign_hybrid_float_b4`      | `-b4`        | 32-bit float mono hybrid (`SHIFT_SENT` profile, no wvx — implied zeros) |
//! | `foreign_hybrid_false_stereo_b4` | `-b4`      | identical-L/R hybrid → false-stereo blocks (bit 30, duplicated output) |
//! | `foreign_hybrid_pair_mono`     | `-b4 -c`     | wv + wvc lossless pair, dynamic noise shaping (round 408) |
//! | `foreign_hybrid_pair_mono_s0`  | `-b4 -c -s0` | wv + wvc lossless pair, shaping off (raw §4.1 fold) |
//! | `foreign_hybrid_pair_mono_sn`  | `-b4 -c -s-0.7` | wv + wvc lossless pair, static negative shaping |
//! | `foreign_hybrid_pair_multi`    | `-b3 -c`     | wv + wvc lossless pair, 2 blocks + silence stretch |
//! | `foreign_hybrid_pair_lr`       | `-b4 -c -j0` | wv + wvc lossless STEREO pair, left/right coding |
//! | `foreign_hybrid_pair_js`       | `-b4 -c`     | wv + wvc lossless JOINT-stereo pair, dynamic noise shaping (round 415) |
//! | `foreign_hybrid_pair_js_s0`    | `-b4 -c -s0` | joint pair, shaping off (pins the coded-domain fold + §5.4 undo order) |
//! | `foreign_hybrid_pair_js_sp`    | `-b4 -c -s0.7` | joint pair, static POSITIVE shaping (weight regime the r408 set never hit) |
//! | `foreign_hybrid_pair_js_multi` | `-b3 -c`     | joint pair, 2+ blocks with a silence stretch (zero-run × shaping-state interplay) |
//! | `foreign_hybrid_pair_js24`     | `-b4 -c`     | 24-bit joint pair |
//! | `foreign_hybrid_pair_js_cc`    | `-b4 -cc`    | joint pair, `CROSS_DECORR` flag + dynamic shaping (round 415 — the bit is decorative; post-decorrelation fold is bit-exact) |
//! | `foreign_hybrid_pair_js_cc_s0` | `-b4 -cc -s0` | joint `CROSS_DECORR` pair, shaping off |
//! | `foreign_hybrid_pair_lr_cc`    | `-b4 -cc -j0` | left/right `CROSS_DECORR` pair, dynamic shaping |
//! | `foreign_hybrid_pair_float`    | `-b4 -c`     | 32-bit float mono pair (`0x08` + wvc-carried `0x0C`, round 415) |
//! | `foreign_hybrid_pair_float_js` | `-b4 -c`     | 32-bit float JOINT-stereo pair with exact ±0 samples |
//! | `foreign_hybrid_pair_int32`    | `-b4 -c`     | 32-bit int mono pair (`0x09` sent-bits + wvc-carried `0x0C`) |
//! | `foreign_hybrid_pair_int32_js` | `-b4 -c`     | 32-bit int JOINT-stereo pair |
//! | `foreign_hybrid_int32_b4`      | `-b4 -c` (wv only) | 32-bit int hybrid LOSSY decode — the pair's `.wv` alone; sent-bits fill with implied zeros |
//!
//! The 8-bit expectation is in signed container values (the WAV source is
//! unsigned 8-bit; WavPack codes the signed offset-removed value, which
//! is what `decode_stream` returns and what `wvunpack --raw`'s unsigned
//! bytes map to via `b - 128`).

use oxideav_wavpack::{
    decode_multichannel_stream, decode_multichannel_stream_muted, decode_stream,
    decode_stream_muted, StreamIndex,
};

/// Decode an expected-PCM sidecar (`.pcm32le`: little-endian `i32`s).
fn expected_pcm(bytes: &[u8]) -> Vec<i32> {
    assert_eq!(bytes.len() % 4, 0, "expected-PCM sidecar must be i32s");
    bytes
        .chunks_exact(4)
        .map(|c| i32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

macro_rules! foreign_fixture {
    ($name:ident, $stem:literal, $channels:expr) => {
        #[test]
        fn $name() {
            let wv = include_bytes!(concat!("data/", $stem, ".wv"));
            let expected = expected_pcm(include_bytes!(concat!("data/", $stem, ".pcm32le")));

            // Stream-shape decode (handles 1, 2 and multichannel-set
            // widths uniformly).
            let decoded = decode_multichannel_stream(wv).expect("foreign file must decode");
            assert_eq!(decoded.channels, $channels, "channel count");
            assert_eq!(
                decoded.samples.len(),
                expected.len(),
                "decoded sample count"
            );
            assert_eq!(decoded.samples, expected, "bit-exact PCM");

            // The stored block CRCs must match — the spec §5.6 mute gate
            // passes every member untouched.
            let (muted, all_ok) =
                decode_multichannel_stream_muted(wv).expect("muted twin must decode");
            assert!(all_ok, "stored block CRC(s) must match");
            assert_eq!(muted.samples, expected, "muted-path PCM");

            // The header-only index accepts everything the decoder does.
            let index = StreamIndex::scan(wv).expect("index scan");
            assert!(index.is_seekable(), "single-chain stream is seekable");

            // Plain mono/stereo files also go through the historical
            // whole-stream path.
            if $channels <= 2 {
                assert_eq!(
                    decode_stream(wv).expect("decode_stream"),
                    expected,
                    "decode_stream parity"
                );
                let (pcm, ok) = decode_stream_muted(wv).expect("decode_stream_muted");
                assert!(ok, "decode_stream_muted CRC verdict");
                assert_eq!(pcm, expected);
            }
        }
    };
}

foreign_fixture!(default_mono16_is_bit_exact, "foreign_default_mono16", 1);
foreign_fixture!(default_stereo16_is_bit_exact, "foreign_default_stereo16", 2);
foreign_fixture!(fast_stereo16_is_bit_exact, "foreign_fast_stereo16", 2);
foreign_fixture!(high_stereo16_is_bit_exact, "foreign_high_stereo16", 2);
foreign_fixture!(vhigh_stereo16_is_bit_exact, "foreign_vhigh_stereo16", 2);
foreign_fixture!(
    vhigh_x4_stereo16_is_bit_exact,
    "foreign_vhigh_x4_stereo16",
    2
);
foreign_fixture!(default_mono8_is_bit_exact, "foreign_default_mono8", 1);
foreign_fixture!(default_stereo24_is_bit_exact, "foreign_default_stereo24", 2);
foreign_fixture!(default_5dot1_is_bit_exact, "foreign_default_5dot1_16", 6);
foreign_fixture!(
    custom_rate_mono16_is_bit_exact,
    "foreign_custom_rate_mono16",
    1
);
foreign_fixture!(int32_mono_is_bit_exact, "foreign_int32_mono", 1);
foreign_fixture!(int32_zeros_mono_is_bit_exact, "foreign_int32_zeros_mono", 1);
foreign_fixture!(int32_stereo_is_bit_exact, "foreign_int32_stereo", 2);
foreign_fixture!(
    int32_high_stereo_is_bit_exact,
    "foreign_int32_high_stereo",
    2
);
foreign_fixture!(
    float_intvalued_mono_is_bit_exact,
    "foreign_float_intvalued_mono",
    1
);
foreign_fixture!(float_mono_is_bit_exact, "foreign_float_mono", 1);
foreign_fixture!(float_zeros_mono_is_bit_exact, "foreign_float_zeros_mono", 1);
foreign_fixture!(float_stereo_is_bit_exact, "foreign_float_stereo", 2);
foreign_fixture!(float_high_mono_is_bit_exact, "foreign_float_high_mono", 1);
foreign_fixture!(float_shift_same_is_bit_exact, "foreign_float_shift_same", 1);
foreign_fixture!(
    float_shift_same_zeros_is_bit_exact,
    "foreign_float_shift_same_zeros",
    1
);
foreign_fixture!(float_shift_ones_is_bit_exact, "foreign_float_shift_ones", 1);
foreign_fixture!(float_exceptions_is_bit_exact, "foreign_float_exceptions", 1);
foreign_fixture!(float_exc_stereo_is_bit_exact, "foreign_float_exc_stereo", 2);
foreign_fixture!(float_exc_tiny_is_bit_exact, "foreign_float_exc_tiny", 1);
foreign_fixture!(float_exc_mix_is_bit_exact, "foreign_float_exc_mix", 1);
foreign_fixture!(hybrid_mono_b4_is_bit_exact, "foreign_hybrid_mono_b4", 1);
foreign_fixture!(hybrid_mono_b2_is_bit_exact, "foreign_hybrid_mono_b2", 1);
foreign_fixture!(
    hybrid_multiblock_b3_is_bit_exact,
    "foreign_hybrid_multiblock_b3",
    1
);
foreign_fixture!(hybrid_stereo_b4_is_bit_exact, "foreign_hybrid_stereo_b4", 2);
foreign_fixture!(hybrid_lr_b4_is_bit_exact, "foreign_hybrid_lr_b4", 2);
foreign_fixture!(hybrid_5dot1_b4_is_bit_exact, "foreign_hybrid_5dot1_b4", 6);
foreign_fixture!(
    hybrid_stereo24_b4_is_bit_exact,
    "foreign_hybrid_stereo24_b4",
    2
);
foreign_fixture!(hybrid_float_b4_is_bit_exact, "foreign_hybrid_float_b4", 1);
foreign_fixture!(hybrid_int32_b4_is_bit_exact, "foreign_hybrid_int32_b4", 1);
foreign_fixture!(
    hybrid_false_stereo_b4_is_bit_exact,
    "foreign_hybrid_false_stereo_b4",
    2
);

/// Hybrid-specific shape assertions on top of the bit-exact battery:
/// the lossy decode carries real coding error against the WAV source
/// (it IS lossy), the false-stereo fixture duplicates its coded
/// channel, and a corrupted hybrid payload still trips the CRC gate.
#[test]
fn foreign_hybrid_false_stereo_duplicates_channels() {
    let wv = include_bytes!("data/foreign_hybrid_false_stereo_b4.wv");
    let decoded = oxideav_wavpack::decode_multichannel_stream(wv).expect("decode");
    assert_eq!(decoded.channels, 2);
    for pair in decoded.samples.chunks_exact(2) {
        assert_eq!(pair[0], pair[1], "false stereo duplicates L to R");
    }
}

/// wv + wvc hybrid-lossless pairs (round 408): combining the coarse
/// stream with the correction stream must reproduce the ORIGINAL PCM
/// bit-exactly — the expected sidecars here are the WAV sources the
/// reference encoder consumed, not reference decodes, so this pins the
/// whole §4.1/§6.5 lossless path end-to-end (bracket completion,
/// 0x07 shaping seeds, the unit-magnitude nudge, the post-decorrelation
/// fold).
macro_rules! hybrid_pair_fixture {
    ($name:ident, $stem:literal) => {
        #[test]
        fn $name() {
            let wv = include_bytes!(concat!("data/", $stem, ".wv"));
            let wvc = include_bytes!(concat!("data/", $stem, ".wvc"));
            let expected = expected_pcm(include_bytes!(concat!("data/", $stem, ".pcm32le")));
            let pcm = oxideav_wavpack::decode_stream_with_correction(wv, wvc).expect("pair decode");
            assert_eq!(pcm, expected, "lossless PCM must equal the encoder input");
            // The lossy decode of the same .wv alone must differ (the
            // coarse stream IS lossy) — guards against the pair path
            // silently ignoring the correction stream.
            let lossy = decode_stream(wv).expect("lossy decode");
            assert_ne!(lossy, expected, "coarse-only decode must be lossy");
            // Round 415: the .wvc header stores the §5 CRC of the
            // LOSSLESS decode — the muted pair twin must pass its gate
            // on every block and emit the same PCM.
            let (muted, all_ok) = oxideav_wavpack::decode_stream_with_correction_muted(wv, wvc)
                .expect("muted pair decode");
            assert!(all_ok, "stored .wvc lossless CRC(s) must match");
            assert_eq!(muted, expected, "muted-path PCM");
        }
    };
}

hybrid_pair_fixture!(hybrid_pair_mono_is_lossless, "foreign_hybrid_pair_mono");
hybrid_pair_fixture!(hybrid_pair_js_is_lossless, "foreign_hybrid_pair_js");
hybrid_pair_fixture!(hybrid_pair_js_s0_is_lossless, "foreign_hybrid_pair_js_s0");
hybrid_pair_fixture!(hybrid_pair_js_sp_is_lossless, "foreign_hybrid_pair_js_sp");
hybrid_pair_fixture!(
    hybrid_pair_js_multi_is_lossless,
    "foreign_hybrid_pair_js_multi"
);
hybrid_pair_fixture!(hybrid_pair_js24_is_lossless, "foreign_hybrid_pair_js24");
hybrid_pair_fixture!(hybrid_pair_js_cc_is_lossless, "foreign_hybrid_pair_js_cc");
hybrid_pair_fixture!(
    hybrid_pair_js_cc_s0_is_lossless,
    "foreign_hybrid_pair_js_cc_s0"
);
hybrid_pair_fixture!(hybrid_pair_lr_cc_is_lossless, "foreign_hybrid_pair_lr_cc");
hybrid_pair_fixture!(hybrid_pair_float_is_lossless, "foreign_hybrid_pair_float");
hybrid_pair_fixture!(
    hybrid_pair_float_js_is_lossless,
    "foreign_hybrid_pair_float_js"
);
hybrid_pair_fixture!(hybrid_pair_int32_is_lossless, "foreign_hybrid_pair_int32");
hybrid_pair_fixture!(
    hybrid_pair_int32_js_is_lossless,
    "foreign_hybrid_pair_int32_js"
);

/// The typed f32 pair twin: bit patterns reinterpret, ±0 signs survive,
/// and an integer pair is refused.
#[test]
fn hybrid_pair_float_f32_twin_matches_bit_patterns() {
    let wv = include_bytes!("data/foreign_hybrid_pair_float_js.wv");
    let wvc = include_bytes!("data/foreign_hybrid_pair_float_js.wvc");
    let expected = expected_pcm(include_bytes!("data/foreign_hybrid_pair_float_js.pcm32le"));
    let floats =
        oxideav_wavpack::decode_stream_with_correction_f32(wv, wvc).expect("f32 pair decode");
    assert_eq!(floats.len(), expected.len());
    for (f, bits) in floats.iter().zip(&expected) {
        assert_eq!(f.to_bits(), *bits as u32, "bit-pattern parity");
    }
    // The synthesized source interleaves exact +0.0 and -0.0 samples.
    assert!(floats.iter().any(|f| *f == 0.0 && f.is_sign_positive()));
    assert!(floats.iter().any(|f| *f == 0.0 && f.is_sign_negative()));

    let int_wv = include_bytes!("data/foreign_hybrid_pair_int32.wv");
    let int_wvc = include_bytes!("data/foreign_hybrid_pair_int32.wvc");
    assert!(matches!(
        oxideav_wavpack::decode_stream_with_correction_f32(int_wv, int_wvc),
        Err(oxideav_wavpack::Error::BlockNotFloat)
    ));
}
hybrid_pair_fixture!(
    hybrid_pair_mono_s0_is_lossless,
    "foreign_hybrid_pair_mono_s0"
);
hybrid_pair_fixture!(
    hybrid_pair_mono_sn_is_lossless,
    "foreign_hybrid_pair_mono_sn"
);
hybrid_pair_fixture!(hybrid_pair_multi_is_lossless, "foreign_hybrid_pair_multi");
hybrid_pair_fixture!(hybrid_pair_lr_is_lossless, "foreign_hybrid_pair_lr");

/// Corrupting the `.wvc` correction payload must trip the round-415
/// pair mute gate (the `.wvc` header CRC covers the LOSSLESS decode):
/// the block is zeroed and `all_crc_ok` is false — never silently
/// wrong samples.
#[test]
fn corrupted_correction_stream_is_muted_or_refused() {
    let wv = include_bytes!("data/foreign_hybrid_pair_js.wv");
    let clean = include_bytes!("data/foreign_hybrid_pair_js.wvc");
    let expected = expected_pcm(include_bytes!("data/foreign_hybrid_pair_js.pcm32le"));
    // Flip one bit deep inside the correction block's 0x0B payload.
    let mut wvc = clean.to_vec();
    let target = wvc.len() / 2;
    wvc[target] ^= 0x04;
    // (a structural refusal instead of `Ok` is equally acceptable)
    if let Ok((pcm, all_ok)) = oxideav_wavpack::decode_stream_with_correction_muted(wv, &wvc) {
        assert!(!all_ok, "corruption must fail the lossless CRC gate");
        assert!(pcm.iter().all(|&s| s == 0), "muted block must be zeroed");
    }
    // The unmuted path (no gate) decodes to WRONG samples on the same
    // corruption — demonstrating the gate is what catches it.
    if let Ok(pcm) = oxideav_wavpack::decode_stream_with_correction(wv, &wvc) {
        assert_ne!(pcm, expected);
    }
}

/// Pair-decode refusal: a non-hybrid block has nothing to correct.
/// (Joint-stereo pairs — the default stereo coding — decode since
/// round 415; see the `hybrid_pair_js*` fixtures above.)
#[test]
fn hybrid_pair_on_non_hybrid_block_is_refused() {
    let wvc = include_bytes!("data/foreign_hybrid_pair_mono.wvc");
    let (corr, _) = oxideav_wavpack::parse_block(wvc).expect("parse corr");
    let lossless = include_bytes!("data/foreign_default_mono16.wv");
    let (plain, _) = oxideav_wavpack::parse_block(lossless).expect("parse");
    assert_eq!(
        plain.decode_samples_with_correction(&corr),
        Err(oxideav_wavpack::Error::BlockNotHybrid)
    );
}

#[test]
fn corrupted_hybrid_block_is_muted_or_refused() {
    let mut wv = include_bytes!("data/foreign_hybrid_mono_b4.wv").to_vec();
    wv[600] ^= 0x08; // deep inside the 0x0A payload
    if let Ok((pcm, all_ok)) = decode_stream_muted(&wv) {
        assert!(!all_ok, "corruption must fail the coarse-value CRC gate");
        assert!(pcm.iter().all(|&s| s == 0), "muted block must be zeroed");
    }
}

/// A corrupted foreign block must trip the CRC mute gate, not decode to
/// wrong PCM silently: flip one payload bit deep inside the first
/// block's audio data and require either a typed decode error or a
/// muted (all-zero) block with `all_crc_ok == false`.
#[test]
fn corrupted_foreign_block_is_muted_or_refused() {
    let mut wv = include_bytes!("data/foreign_default_mono16.wv").to_vec();
    // Flip a bit well past the 32-byte header, inside the metadata /
    // bitstream region of the first block.
    let target = 200;
    wv[target] ^= 0x10;
    // A structural refusal (Err) is equally acceptable; only a clean
    // decode with a passing CRC verdict would be wrong.
    if let Ok((_, all_ok)) = decode_stream_muted(&wv) {
        assert!(!all_ok, "corruption must fail the CRC gate");
    }
}

/// The foreign stereo fixture exercises seeking: a mid-stream window
/// decoded via the index must equal the same slice of the whole-stream
/// decode.
#[test]
fn foreign_stereo_window_decode_matches_whole_stream() {
    let wv = include_bytes!("data/foreign_default_stereo16.wv");
    let whole = decode_stream(wv).expect("whole decode");
    let index = StreamIndex::scan(wv).expect("scan");
    let window = oxideav_wavpack::decode_range(wv, &index, 500, 300).expect("window decode");
    assert_eq!(window, whole[500 * 2..(500 + 300) * 2].to_vec());
}

/// Foreign fixtures carry real sample-rate indices (and the `0x27`
/// custom-rate sub-block): `stream_sample_rate` resolves both carriers
/// (round 405; staged spec `wavpack-sample-formats.md` §5).
#[test]
fn foreign_fixture_sample_rates_resolve() {
    use oxideav_wavpack::stream_sample_rate;
    // The WAV sources were synthesized at these rates (see the module
    // doc); reference `wavpack` stamps the table index — or, for
    // 12345 Hz, the sentinel 15 plus a 0x27 sub-block.
    let cases: [(&[u8], u32); 5] = [
        (include_bytes!("data/foreign_default_mono16.wv"), 44_100),
        (include_bytes!("data/foreign_default_stereo24.wv"), 48_000),
        (include_bytes!("data/foreign_default_mono8.wv"), 8_000),
        (include_bytes!("data/foreign_default_5dot1_16.wv"), 44_100),
        (include_bytes!("data/foreign_custom_rate_mono16.wv"), 12_345),
    ];
    for (wv, rate) in cases {
        assert_eq!(stream_sample_rate(wv).expect("rate resolution"), Some(rate));
    }
}

/// Time-addressed seeking over a foreign file: `seek_seconds` lands on
/// `round(seconds * rate)` and reads the same frames as a frame seek.
#[test]
fn foreign_time_seek_matches_frame_seek() {
    use oxideav_wavpack::StreamReader;
    let wv = include_bytes!("data/foreign_default_stereo16.wv");
    let mut reader = StreamReader::new(wv).expect("reader");
    assert_eq!(reader.sample_rate().expect("rate"), Some(44_100));

    reader.seek_seconds(0.01).expect("time seek"); // 441 frames
    assert_eq!(reader.position(), 441);
    let via_time = reader.read_frames(64).expect("read");
    reader.seek(441).expect("frame seek");
    assert_eq!(reader.read_frames(64).expect("read"), via_time);

    // Past-the-end times clamp to the end-of-stream cursor state.
    reader.seek_seconds(1e9).expect("clamped seek");
    assert!(reader.is_at_end());
    // Negative / non-finite times are refused.
    assert!(reader.seek_seconds(-0.5).is_err());
    assert!(reader.seek_seconds(f64::NAN).is_err());
}

/// Corrupting the `0x0C` extension bitstream of an int32 block must
/// trip the §5.5 extension-CRC arm of the mute gate even when the main
/// sample CRC (folded over the pre-fixup buffer) still matches.
#[test]
fn corrupted_int32_extension_bits_fail_the_crc_x_gate() {
    let clean = include_bytes!("data/foreign_int32_mono.wv");
    // Find the 0x0C sub-block in the first block and flip a bit deep
    // inside its extension payload (past the 4-byte crc_wvx prefix).
    let (block, _) = oxideav_wavpack::parse_block(clean).expect("parse");
    let overflow = block
        .find_packed_overflow_bits_sub_block()
        .expect("int32 fixture carries a 0x0C sub-block");
    // Locate the payload inside the file bytes by searching for its
    // (unique, high-entropy) contents.
    let payload = overflow.payload;
    let pos = clean
        .windows(payload.len())
        .position(|w| w == payload)
        .expect("payload bytes locatable");
    let mut wv = clean.to_vec();
    wv[pos + payload.len() / 2] ^= 0x01;

    if let Ok((pcm, all_ok)) = decode_stream_muted(&wv) {
        assert!(!all_ok, "extension-bit corruption must fail the gate");
        assert!(pcm.iter().all(|&s| s == 0), "muted block must be zeroed");
    }
}

/// The round-408 exception fixtures must reproduce genuine IEEE
/// specials — including a NaN whose exact payload bits survive — and
/// the tiny fixture pins the short-`0x03`-weights wire-order-prefix
/// rule (2 weight bytes for a 5-term stack) through the stored block
/// CRC.
#[test]
fn foreign_exception_fixtures_reproduce_ieee_specials() {
    let floats =
        oxideav_wavpack::decode_stream_f32(include_bytes!("data/foreign_float_exceptions.wv"))
            .expect("f32 decode");
    assert!(floats
        .iter()
        .any(|f| f.is_infinite() && f.is_sign_positive()));
    assert!(floats
        .iter()
        .any(|f| f.is_infinite() && f.is_sign_negative()));
    assert!(floats.iter().any(|f| f.is_nan()));

    // exc_tiny: 0.5 everywhere except +inf / NaN(0x155555) / -inf.
    let tiny = oxideav_wavpack::decode_stream_f32(include_bytes!("data/foreign_float_exc_tiny.wv"))
        .expect("f32 decode");
    assert_eq!(tiny.len(), 64);
    assert_eq!(tiny[10], f32::INFINITY);
    assert_eq!(
        tiny[20].to_bits(),
        0x7F95_5555,
        "NaN payload bits preserved"
    );
    assert_eq!(tiny[30], f32::NEG_INFINITY);
    assert!(tiny
        .iter()
        .enumerate()
        .filter(|(i, _)| ![10, 20, 30].contains(i))
        .all(|(_, &f)| f == 0.5));
}

/// The typed f32 surface reinterprets a float stream's decoded bit
/// patterns; a non-float (integer) stream is refused.
#[test]
fn float_typed_decode_matches_bit_patterns_and_refuses_integers() {
    let wv = include_bytes!("data/foreign_float_stereo.wv");
    let expected = expected_pcm(include_bytes!("data/foreign_float_stereo.pcm32le"));
    let floats = oxideav_wavpack::decode_stream_f32(wv).expect("f32 decode");
    assert_eq!(floats.len(), expected.len());
    for (f, bits) in floats.iter().zip(&expected) {
        assert_eq!(f.to_bits(), *bits as u32, "bit-pattern parity");
    }
    // ±0 and denormals survive the typed reinterpretation.
    assert!(floats.iter().any(|f| *f == 0.0 && f.is_sign_negative()));

    let int_wv = include_bytes!("data/foreign_default_mono16.wv");
    assert!(matches!(
        oxideav_wavpack::decode_stream_f32(int_wv),
        Err(oxideav_wavpack::Error::BlockNotFloat)
    ));
}

/// Corrupting a float block's extension bitstream trips the crc_x arm
/// of the mute gate (the float fold — mantissa/exponent/sign triples).
#[test]
fn corrupted_float_extension_bits_fail_the_crc_x_gate() {
    let clean = include_bytes!("data/foreign_float_mono.wv");
    let (block, _) = oxideav_wavpack::parse_block(clean).expect("parse");
    let payload = block
        .find_packed_overflow_bits_sub_block()
        .expect("float fixture carries a 0x0C sub-block")
        .payload;
    let pos = clean
        .windows(payload.len())
        .position(|w| w == payload)
        .expect("payload bytes locatable");
    let mut wv = clean.to_vec();
    wv[pos + payload.len() / 2] ^= 0x01;
    if let Ok((pcm, all_ok)) = decode_stream_muted(&wv) {
        assert!(!all_ok, "extension-bit corruption must fail the gate");
        assert!(pcm.iter().all(|&s| s == 0), "muted block must be zeroed");
    }
}

/// The 5.1 foreign fixture's first member declares its geometry via
/// the `0x0D` `[count, mask]` sub-block (staged spec
/// `wavpack-sample-formats.md` §6): 6 channels, the standard 5.1
/// speaker mask 0x3F — matching the decoded interleave width.
#[test]
fn foreign_5dot1_channel_info_declares_count_and_mask() {
    let wv = include_bytes!("data/foreign_default_5dot1_16.wv");
    let info = oxideav_wavpack::stream_channel_info(wv)
        .expect("parse")
        .expect("5.1 stream carries 0x0D");
    assert_eq!(info.count, 6);
    assert_eq!(info.mask, 0x3F, "standard 5.1 speaker mask");
    assert_eq!(info.assigned_positions(), 6);
    let decoded = decode_multichannel_stream(wv).expect("decode");
    assert_eq!(decoded.channels, usize::from(info.count));
    // Plain mono / stereo files carry no 0x0D.
    let mono = include_bytes!("data/foreign_default_mono16.wv");
    assert!(oxideav_wavpack::stream_channel_info(mono)
        .expect("parse")
        .is_none());
}
