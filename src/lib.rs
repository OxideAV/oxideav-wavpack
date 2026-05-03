//! Pure-Rust **WavPack** lossless audio decoder.
//!
//! WavPack is a lossless audio codec with optional hybrid-lossy
//! support. This crate implements the lossless mode (round 1):
//!
//! - 32-byte block header parsing.
//! - Tagged sub-block walk (`DECTERMS`, `DECWEIGHTS`, `DECSAMPLES`,
//!   `ENTROPY`, `DATA`, `INT32INFO`, `EXTRABITS`, `CHANINFO`,
//!   `SAMPLE_RATE`).
//! - Adaptive 3-bin median entropy decoder (M0/M1/M2 with
//!   `+5 ⌊(m + 128/(2^n))/(128/(2^n))⌋`-style update rates).
//! - Decorrelation cascade reverse for terms `1..=8`, `17`, `18`, and
//!   the cross-channel terms `-1`, `-2`, `-3`.
//! - Joint-stereo (mid/side) and false-stereo undo.
//! - Per-block CRC verification (CRC of the *decoded* sample stream).
//! - Container bit-depths: 8 / 16 / 24 / 32 (lossless integer PCM).
//!
//! Lossy hybrid (`HYBRID*` flag bits, `WP_ID_HYBRID` / `WP_ID_SHAPING`
//! sub-blocks, `.wvc` correction file), DSD, and float-data paths are
//! deferred to round 2.
//!
//! See `docs/audio/wavpack/wavpack-trace-reverse-engineering.md` and
//! `docs/audio/wavpack/wavpack-decorr-presets.md` for the clean-room
//! behavioural spec this crate was written from.

#![allow(clippy::needless_range_loop)]

pub mod block;
pub mod codec;
pub mod container;
pub mod decoder;
pub mod entropy;
pub mod log2;

use oxideav_core::CodecRegistry;

/// Stable codec id string this crate registers under.
pub const CODEC_ID_STR: &str = "wavpack";

/// Register the WavPack decoder with `reg`. After this call the
/// registry can construct a decoder for `CodecId::new("wavpack")`.
pub fn register_codecs(reg: &mut CodecRegistry) {
    codec::register(reg);
}
