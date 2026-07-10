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
