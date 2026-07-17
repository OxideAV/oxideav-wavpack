//! WavPack hybrid-mode correction-fold arithmetic (decorrelation-spec §4.1).
//!
//! WavPack *hybrid* mode (wiki flag bit 3, `HYBRID_FLAG` `0x08`) splits the
//! signal into a **lossy main stream** (`0x0A`) plus an optional
//! **correction stream** (`0x0B`, normally the `.wvc` companion file). When
//! the correction stream is present the decoder reconstructs the original
//! samples *losslessly* by reading **two** residuals per sample position:
//! the lossy value from `0x0A` and a correction value from `0x0B`. The
//! staged clean-room trace
//! (`docs/audio/wavpack/spec/wavpack-decorrelation.md` §4.1) describes the
//! *fold* — where in the per-sample pipeline the correction value is added
//! to the reconstructed lossy value to recover the exact original.
//!
//! This module carries two layers:
//!
//! * the documented **fold arithmetic** as typed primitives — a pure
//!   consumer of two already-decoded integer values (one lossy main
//!   value, one correction value);
//! * since round 408, the **`error_limit` model** that decodes the
//!   lossy `0x0A` stream itself: [`HybridProfile`] (the `0x06` seed),
//!   [`HybridState`] (the per-channel slow-level recurrence and the
//!   per-frame limit derivation, including the stereo balance
//!   redistribution), consumed by the `samples` module's bracketing
//!   loops. The staged spec §6.5 gives the structural model; the exact
//!   integer recurrence was pinned **black-box** against reference
//!   hybrid decodes (bit-exact over a mono/stereo/multi-block/
//!   silence/bitrate-sweep fixture battery).
//!
//! ## Where the fold sits (spec §4.1)
//!
//! The spec documents three placements keyed on the channel layout and the
//! `CROSS_DECORR` (`0x20`) flag:
//!
//! * **Mono / non-cross** ([`fold_correction`]): the decorrelation passes
//!   run on the lossy value, and the correction is then **added to the
//!   reconstructed lossy sample** to recover the exact original — a
//!   *post-decorrelation* fold. The spec writes this as
//!   `read_word += correction[0]`.
//! * **Stereo *without* `CROSS_DECORR`** ([`fold_correction_pair`]):
//!   decorrelation runs on the lossy left/right first, then the
//!   per-channel corrections are added afterward — the post-decorrelation
//!   fold applied to each channel of the pair.
//! * **Stereo *with* `CROSS_DECORR`** ([`fold_correction_pre_decorrelation`]
//!   / [`fold_correction_pre_decorrelation_pair`]): the correction is
//!   folded in *before* the decorrelation passes (a "no-delay" correction)
//!   — the lossy left/right *plus* their corrections form the inputs, then
//!   decorrelation runs to yield the lossless sample.
//!
//! The arithmetic of the fold itself is identical in all three cases
//! (`value + correction`); the placements differ only in *which* value the
//! correction is added to (the reconstructed output vs. the pre-decorr
//! input). The typed names above keep the two pipeline positions distinct
//! so a consuming decoder cannot accidentally apply a post-decorrelation
//! fold where the `CROSS_DECORR` pre-decorrelation fold is required (or
//! vice-versa).
//!
//! ## Encoder inverse (forward direction)
//!
//! The forward (encode) direction is the exact arithmetic inverse:
//! [`split_correction`] computes `correction = original - lossy` so that
//! `lossy + correction == original`. This is the value the encoder packs
//! into the `0x0B` correction stream; it pairs the lossy main value the
//! encoder emitted with the original sample to recover the residual the
//! decoder's [`fold_correction`] consumes.
//!
//! ## Not covered (documented gaps)
//!
//! The **noise-shaping** variant of the correction fold (`HYBRID_SHAPE`
//! `0x40` / `NEW_SHAPING` `0x20000000`, spec §4.1) replaces the raw add
//! with a first-order error-feedback filter whose per-channel shaping
//! weight/state come from the `0x07` metadata — a seed layout the
//! staged docs name but do not transcribe. The raw-add fold here is the
//! `HYBRID_SHAPE`-clear case; the shaped **fold** stays a documented
//! gap. (Note the shaping bits do NOT affect the `.wv`-only lossy
//! decode — round-408 black-box fixtures carry `HYBRID_SHAPE` and
//! decode bit-exact without any shaping arithmetic; the filter only
//! participates when folding a `0x0B` correction stream.)

use crate::error::{Error, Result};

/// The `HYBRID_FLAG` bit (`0x08`, spec §6): the block is a hybrid (lossy
/// main + optional correction) block.
pub const HYBRID_FLAG: u32 = 0x08;

/// The `CROSS_DECORR` bit (`0x20`, spec §6): the hybrid-stereo correction
/// is folded *before* the decorrelation passes (a zero-delay correction),
/// rather than added to the reconstructed output afterward.
pub const CROSS_DECORR_FLAG: u32 = 0x20;

/// The `HYBRID_SHAPE` bit (`0x40`, spec §6): the correction is applied
/// through a first-order error-feedback (noise-shaping) filter rather than
/// added raw. The shaped fold is **not** implemented here (its
/// `read_shaping_info` state layout is a documented gap); this constant
/// names the flag so a consumer can detect and refuse the shaped case
/// before reaching the raw-add fold.
pub const HYBRID_SHAPE_FLAG: u32 = 0x40;

/// The `NEW_SHAPING` bit (`0x2000_0000`, spec §6): selects the IIR
/// negative-shaping variant of the [`HYBRID_SHAPE_FLAG`] error-feedback
/// filter. Like `HYBRID_SHAPE`, the shaped fold is not implemented here.
pub const NEW_SHAPING_FLAG: u32 = 0x2000_0000;

/// Fold one correction residual into a reconstructed lossy sample
/// (spec §4.1, the post-decorrelation fold).
///
/// Recovers the exact original sample from the decoder's reconstructed
/// **lossy** value and the matching correction residual read from the
/// `0x0B` stream:
///
/// ```text
/// original = reconstructed + correction
/// ```
///
/// The spec writes this as `read_word += correction[0]` after the
/// decorrelation passes have produced `read_word` (the reconstructed lossy
/// sample). This is the mono fold and the per-channel fold of a stereo
/// block *without* `CROSS_DECORR`.
///
/// The add uses [`i32::wrapping_add`]: the reconstructed value and the
/// correction are both decoded as 32-bit two's-complement integers and
/// the canonical decoder folds them in a 32-bit register, so a sum past
/// `i32::MAX`/`i32::MIN` wraps rather than panicking — matching the
/// wrap-around arithmetic the rest of the decode pipeline uses.
#[inline]
#[must_use]
pub fn fold_correction(reconstructed: i32, correction: i32) -> i32 {
    reconstructed.wrapping_add(correction)
}

/// Fold a pair of correction residuals into a reconstructed lossy stereo
/// pair (spec §4.1, the stereo post-decorrelation fold *without*
/// `CROSS_DECORR`).
///
/// Applies [`fold_correction`] to each channel of the reconstructed
/// `(left, right)` pair with its matching correction, returning the
/// recovered lossless `(left, right)`. This is the placement the spec
/// documents for a hybrid stereo block whose `CROSS_DECORR` flag is
/// **clear**: decorrelation runs on the lossy values first, then the
/// per-channel corrections are added afterward.
#[inline]
#[must_use]
pub fn fold_correction_pair(reconstructed: (i32, i32), correction: (i32, i32)) -> (i32, i32) {
    (
        fold_correction(reconstructed.0, correction.0),
        fold_correction(reconstructed.1, correction.1),
    )
}

/// Fold one correction residual into a lossy *input* sample before the
/// decorrelation passes (spec §4.1, the `CROSS_DECORR` pre-decorrelation
/// fold).
///
/// For a hybrid stereo block with `CROSS_DECORR` (`0x20`) set, the spec
/// folds the correction in *before* decorrelation — the lossy input plus
/// its correction becomes the decorrelation pass input:
///
/// ```text
/// input = lossy + correction
/// ```
///
/// Arithmetically the add is identical to [`fold_correction`]; the
/// distinct name marks the *pipeline position* (the value here is the
/// pre-decorrelation input, not the reconstructed output). Keeping the two
/// names separate prevents a consumer from folding a correction in the
/// wrong stage for a given `CROSS_DECORR` setting.
#[inline]
#[must_use]
pub fn fold_correction_pre_decorrelation(lossy: i32, correction: i32) -> i32 {
    lossy.wrapping_add(correction)
}

/// Fold a pair of correction residuals into a lossy *input* pair before
/// the decorrelation passes (spec §4.1, the `CROSS_DECORR` stereo
/// pre-decorrelation fold).
///
/// Applies [`fold_correction_pre_decorrelation`] to each channel of the
/// lossy `(left, right)` input pair. The resulting pair is the input the
/// stereo decorrelation passes consume when `CROSS_DECORR` is set.
#[inline]
#[must_use]
pub fn fold_correction_pre_decorrelation_pair(
    lossy: (i32, i32),
    correction: (i32, i32),
) -> (i32, i32) {
    (
        fold_correction_pre_decorrelation(lossy.0, correction.0),
        fold_correction_pre_decorrelation(lossy.1, correction.1),
    )
}

/// Compute the correction residual the encoder packs into the `0x0B`
/// stream (the forward / encode inverse of [`fold_correction`]).
///
/// ```text
/// correction = original - lossy
/// ```
///
/// so that `fold_correction(lossy, split_correction(original, lossy)) ==
/// original`. The subtraction wraps in 32 bits, mirroring the decoder's
/// wrapping add: the encoder forms the correction in the same register
/// width the decoder folds it back through.
#[inline]
#[must_use]
pub fn split_correction(original: i32, lossy: i32) -> i32 {
    original.wrapping_sub(lossy)
}

/// On-wire byte length of a `0x06` hybrid-profile payload for a block
/// whose data is mono / false-stereo: two little-endian 16-bit words —
/// the log-packed initial `slow_level` and the bitrate word (round-408
/// black-box pin; see [`expand_hybrid_profile`]).
pub const HYBRID_PROFILE_MONO_BYTES: usize = 4;

/// On-wire byte length of a `0x06` hybrid-profile payload for a stereo
/// block: four little-endian 16-bit words — the two per-channel
/// log-packed initial `slow_level`s, the shared bitrate word, and the
/// balance word (round-408 black-box pin).
pub const HYBRID_PROFILE_STEREO_BYTES: usize = 8;

/// The constant offset in the `error_limit` argument
/// (`ema - bitrate + 256`, staged spec §6.5 model + round-408 pin).
pub const HYBRID_LIMIT_BIAS: i32 = 256;

/// Defensive ceiling on the `error_limit` log argument before
/// [`crate::wp_exp2s`] expansion: `30 << 8 | 0xff` keeps the expanded
/// limit within `u32` (int part 30 → mantissa shifted left 21, max
/// `0x1ff << 21 < 2^31`). Arguments above it saturate the limit to
/// `u32::MAX` (an interval is never wider than `2^31 - 1`, so any such
/// limit means "stop immediately"). Unreachable on conformant streams
/// (it needs a tracked signal level near 2^30).
pub const HYBRID_LIMIT_ARG_CEILING: i32 = (30 << 8) | 0xff;

/// Typed expansion of the `0x06` `ID_HYBRID_PROFILE` sub-block — the
/// per-block seed of the hybrid `error_limit` state (staged spec
/// `wavpack-entropy-decode.md` §6.5; exact wire layout pinned black-box
/// in round 408 against reference-encoded hybrid files).
///
/// Layout (little-endian 16-bit words):
///
/// | Word | Mono / false-stereo         | Stereo                          |
/// | ---- | --------------------------- | ------------------------------- |
/// | 0    | `slow_level` seed (log word)| channel-0 `slow_level` seed     |
/// | 1    | bitrate word                | channel-1 `slow_level` seed     |
/// | 2    | —                           | bitrate word (shared)           |
/// | 3    | —                           | balance word                    |
///
/// The `slow_level` seeds are **log-packed** exactly like the `0x05`
/// medians: the linear state is recovered with [`crate::wp_exp2s`].
/// The bitrate word is in the same 8-fractional-bit log domain
/// (empirically `max(0, bits_per_sample * 256 - 568)` across the
/// reference encoder's `-b` range — the decoder just consumes it). The
/// stereo balance word anchors the per-channel limit split (observed
/// `0x0100` on mid/side blocks, `0` on left/right blocks).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HybridProfile {
    /// Log-packed per-channel `slow_level` seeds (`[word, 0]` for a
    /// mono profile).
    pub level_words: [i16; 2],
    /// The bitrate word (log-domain bits-per-sample target).
    pub bitrate: i32,
    /// The stereo balance word (`0` for a mono profile).
    pub balance: i32,
    /// `true` when the profile carries the 4-word stereo layout.
    pub stereo: bool,
}

/// Expand a `0x06` hybrid-profile payload into a typed
/// [`HybridProfile`].
///
/// `stereo` selects the expected layout from the block's channel shape
/// (`!Flags::is_block_data_mono()`): 4 bytes for mono / false-stereo,
/// 8 bytes for stereo. Any other length is
/// [`Error::HybridProfileLength`].
pub fn expand_hybrid_profile(payload: &[u8], stereo: bool) -> Result<HybridProfile> {
    let word = |i: usize| i16::from_le_bytes([payload[2 * i], payload[2 * i + 1]]);
    if stereo {
        if payload.len() != HYBRID_PROFILE_STEREO_BYTES {
            return Err(Error::HybridProfileLength(payload.len()));
        }
        Ok(HybridProfile {
            level_words: [word(0), word(1)],
            bitrate: i32::from(word(2)),
            balance: i32::from(word(3)),
            stereo: true,
        })
    } else {
        if payload.len() != HYBRID_PROFILE_MONO_BYTES {
            return Err(Error::HybridProfileLength(payload.len()));
        }
        Ok(HybridProfile {
            level_words: [word(0), 0],
            bitrate: i32::from(word(1)),
            balance: 0,
            stereo: false,
        })
    }
}

/// The running hybrid `error_limit` state of one block (staged spec
/// §6.5 "where `error_limit` comes from"; exact recurrence pinned
/// black-box in round 408, bit-exact against reference decodes over
/// mono/stereo/multi-block/silence/bitrate-sweep fixtures).
///
/// Per coded channel the state is a linear `slow_level` accumulator
/// seeded from the profile's log-packed level word
/// (`wp_exp2s(level_word)`). Every decoded sample updates its
/// channel's accumulator with the sample's pre-sign magnitude:
///
/// ```text
/// slow_level -= (slow_level + 128) >> 8;
/// slow_level += wp_log2(magnitude);
/// ```
///
/// (zero samples — including zero-run members — update with
/// `wp_log2(0) == 0`, decaying the level through silence).
///
/// The per-sample `error_limit` is derived **at frame start** (per
/// sample for mono; once per L/R pair for stereo, both channels from
/// the same pre-frame states):
///
/// ```text
/// ema_ch  = (slow_level_ch + 128) >> 8            (log domain)
/// mono:    arg = ema_0 - bitrate + 256
/// stereo:  delta = (ema_0 - ema_1 - balance) >> 1  (arithmetic shift)
///          arg_0 = ema_0 - delta - bitrate + 256
///          arg_1 = ema_1 + delta - bitrate + 256
/// limit_ch = wp_exp2s(arg_ch)   when arg_ch > 0, else 0 (lossless)
/// ```
///
/// The stereo `delta` redistributes precision between the channels
/// around the profile's balance anchor (the flag-bit-10 "hybrid noise
/// balanced" behaviour); with `balance == 0` and equal levels it
/// degenerates to two independent mono channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HybridState {
    slow_level: [u32; 2],
    bitrate: i32,
    balance: i32,
    stereo: bool,
}

impl HybridState {
    /// Seed the running state from a block's expanded `0x06` profile.
    ///
    /// Each `slow_level` accumulator starts at `wp_exp2s(level_word)`
    /// (negative words clamp to 0 — a level is a magnitude).
    #[must_use]
    pub fn from_profile(profile: &HybridProfile) -> Self {
        let seed = |w: i16| crate::logpack::wp_exp2s(i32::from(w)).max(0) as u32;
        HybridState {
            slow_level: [seed(profile.level_words[0]), seed(profile.level_words[1])],
            bitrate: profile.bitrate,
            balance: profile.balance,
            stereo: profile.stereo,
        }
    }

    /// The linear `slow_level` accumulator of `channel` (0 or 1).
    #[must_use]
    pub fn slow_level(&self, channel: usize) -> u32 {
        self.slow_level[channel & 1]
    }

    /// Expand one limit argument, with the defensive
    /// [`HYBRID_LIMIT_ARG_CEILING`] saturation.
    fn limit_from_arg(arg: i32) -> u32 {
        if arg <= 0 {
            0
        } else if arg > HYBRID_LIMIT_ARG_CEILING {
            u32::MAX
        } else {
            crate::logpack::wp_exp2s(arg) as u32
        }
    }

    /// Compute the frame's per-channel `error_limit`s from the current
    /// (pre-frame) states. Index 0 is the mono limit / stereo channel
    /// 0; index 1 is stereo channel 1 (0 for mono states — a mono
    /// frame is one sample).
    #[must_use]
    pub fn frame_limits(&self) -> [u32; 2] {
        let ema0 = ((self.slow_level[0] + 128) >> 8) as i32;
        if !self.stereo {
            return [
                Self::limit_from_arg(ema0 - self.bitrate + HYBRID_LIMIT_BIAS),
                0,
            ];
        }
        let ema1 = ((self.slow_level[1] + 128) >> 8) as i32;
        let delta = (ema0 - ema1 - self.balance) >> 1;
        [
            Self::limit_from_arg(ema0 - delta - self.bitrate + HYBRID_LIMIT_BIAS),
            Self::limit_from_arg(ema1 + delta - self.bitrate + HYBRID_LIMIT_BIAS),
        ]
    }

    /// Fold one decoded sample's **pre-sign magnitude** (the bracket /
    /// mantissa value before the §4.2 step-7 complement) into
    /// `channel`'s running level. Must be called for **every** emitted
    /// sample of that channel, including zeros from the §4.2 step-1
    /// zero-run path (`wp_log2(0) == 0` decays the level).
    pub fn update(&mut self, channel: usize, magnitude: u32) {
        let sl = &mut self.slow_level[channel & 1];
        *sl = *sl - ((*sl + 128) >> 8) + crate::logpack::wp_log2(magnitude) as u32;
    }

    /// [`Self::update`] keyed on a signed decoded sample (the §4.2
    /// step-7 output): a negative sample's magnitude is its bitwise
    /// complement.
    pub fn update_signed(&mut self, channel: usize, sample: i32) {
        let magnitude = if sample < 0 {
            !sample as u32
        } else {
            sample as u32
        };
        self.update(channel, magnitude);
    }
}

/// Per-channel noise-shaping filter state for the hybrid-lossless
/// (`.wv` + `.wvc`) decode — the `0x07` `ID_SHAPING_WEIGHTS` seed and
/// its per-sample recurrence (round-408 black-box pin, bit-exact over
/// mono no-shaping / static-shaping / dynamic-shaping and left/right
/// stereo pair fixtures; extended to static-positive weights in round
/// 415).
///
/// The two channels are **output** channels: for mono and left/right
/// stereo they coincide with the coded channels and the temps fold
/// in-line in the entropy loop, while joint (mid/side) blocks keep the
/// same left/right states but transform the temps into the coded
/// domain per frame (`t_m = t_l - t_r`;
/// `t_s = t_r + ((mid + t_m) >> 1) - (mid >> 1)` on the output-domain
/// mid) and fold the effective per-output deltas back into
/// [`Self::update`] — the round-415 joint pair pin (see
/// `WavPackBlock::decode_samples_with_correction`).
///
/// ## Wire layout of `0x07` (in the **correction** block)
///
/// Little-endian 16-bit **log-packed** words (each unpacked with
/// [`crate::wp_exp2s`]):
///
/// | Shape                | Words                                              |
/// | -------------------- | -------------------------------------------------- |
/// | mono, static shaping | `[error, acc]`                                     |
/// | mono, dynamic (DNS)  | `[error, acc, delta]`                              |
/// | stereo, static       | `[error0, acc0, error1, acc1]`                     |
/// | stereo, dynamic      | `[error0, acc0, error1, acc1, delta0, delta1]`     |
///
/// The **error seed is negated** on the wire (`error = -wp_exp2s(word)`);
/// `acc` / `delta` unpack directly. A missing `0x07` (the no-shaping
/// `-s0` encode) seeds everything at zero, making every `temp` zero —
/// the raw §4.1 fold.
///
/// ## Per-sample recurrence
///
/// ```text
/// acc += delta;  weight = acc >> 16
/// temp = -((weight * error + 511) >> 10)
/// if weight < 0 and |temp| >= |error| != 0:
///     temp = sign(temp) * (|error| - 1)      // unit-magnitude nudge
/// exact = wvc_bracket_value + temp           // bracketed samples only
/// error = (exact - coarse)                   // weight <  0
/// error = (exact - coarse) - temp            // weight >= 0
/// ```
///
/// Zero-run / lossless-dispatch samples apply no `temp` and update the
/// error state with `exact == coarse` (both zero for run members).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ShapingState {
    error: [i32; 2],
    acc: [i32; 2],
    delta: [i32; 2],
}

impl ShapingState {
    /// Seed from a `0x07` payload (`None` / empty → all-zero state, the
    /// no-shaping raw fold). `stereo` selects the per-channel word
    /// interleave shown in the type-level docs.
    #[must_use]
    pub fn from_shaping_words(payload: Option<&[u8]>, stereo: bool) -> Self {
        let words: Vec<i32> = payload
            .unwrap_or(&[])
            .chunks_exact(2)
            .map(|c| crate::logpack::wp_exp2s(i32::from(i16::from_le_bytes([c[0], c[1]]))))
            .collect();
        let g = |i: usize| words.get(i).copied().unwrap_or(0);
        // The error seed negation wraps: an adversarial log word can
        // expand to `i32::MIN`, whose two's-complement negation is
        // itself (round-415 fuzz find; 32-bit-register semantics as
        // everywhere else in the recurrence).
        let e = |i: usize| g(i).wrapping_neg();
        if stereo {
            ShapingState {
                error: [e(0), e(2)],
                acc: [g(1), g(3)],
                delta: [g(4), g(5)],
            }
        } else {
            ShapingState {
                error: [e(0), 0],
                acc: [g(1), 0],
                delta: [g(2), 0],
            }
        }
    }

    /// Advance `channel`'s weight one sample (`acc += delta`) and return
    /// this sample's `temp` term from the pre-update error state.
    ///
    /// The accumulator add and the `temp` product/bias arithmetic wrap in
    /// 32 bits: adversarial `0x07` seeds can drive `acc`/`delta` (and the
    /// error state) to magnitudes where the canonical 32-bit-register
    /// arithmetic wraps, and the decoder must follow it rather than
    /// overflow (round-415 fuzz find; same posture as the round-386
    /// wrapping-predictor fix).
    pub fn advance(&mut self, channel: usize) -> i32 {
        let ch = channel & 1;
        self.acc[ch] = self.acc[ch].wrapping_add(self.delta[ch]);
        let weight = self.acc[ch] >> 16;
        let err = self.error[ch];
        if err == 0 {
            return 0;
        }
        let mut temp = -(weight.wrapping_mul(err).wrapping_add(511) >> 10);
        if weight < 0 && temp.unsigned_abs() >= err.unsigned_abs() {
            // The unit-magnitude nudge: |temp| stays strictly below
            // |error| under a negative weight. (`|err| - 1` is computed
            // in u32 so an `i32::MIN` error state cannot overflow the
            // subtraction; the result always fits `i32`.)
            temp = temp.signum().wrapping_mul((err.unsigned_abs() - 1) as i32);
        }
        temp
    }

    /// Fold one decoded sample's outcome back into `channel`'s error
    /// state. `exact` / `coarse` are the §4.1 lossless and coarse
    /// values; `temp` is what [`Self::advance`] returned for this
    /// sample (0 for non-bracketed samples).
    pub fn update(&mut self, channel: usize, exact: i32, coarse: i32, temp: i32) {
        let ch = channel & 1;
        let q = exact.wrapping_sub(coarse);
        self.error[ch] = if self.acc[ch] >> 16 < 0 {
            q
        } else {
            q.wrapping_sub(temp)
        };
    }

    /// The current error state of `channel` (test / introspection).
    #[must_use]
    pub fn error(&self, channel: usize) -> i32 {
        self.error[channel & 1]
    }
}

/// `true` when the flag word selects the **noise-shaped** correction fold
/// (`HYBRID_SHAPE` or `NEW_SHAPING`), which this module does not implement.
///
/// A hybrid consumer should test this before applying the raw-add fold:
/// when it is `true` the correction must be applied through the
/// error-feedback filter (spec §4.1), whose `read_shaping_info` state
/// layout is a documented gap — so the raw add here would produce wrong
/// samples. The raw-add fold is correct only when this returns `false`.
#[inline]
#[must_use]
pub fn flags_select_shaping(flags: u32) -> bool {
    flags & (HYBRID_SHAPE_FLAG | NEW_SHAPING_FLAG) != 0
}

/// Which §4.1 correction-fold placement a block's flag word selects.
///
/// The fold *arithmetic* is the same raw add in every documented case
/// (`value + correction`); what differs is **where** in the per-sample
/// pipeline it runs and whether it is even a raw add. This enum names the
/// placement so a consumer can dispatch (or refuse) correctly. Derive it
/// from a block's 32-bit flag word with [`CorrectionFold::from_flags`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CorrectionFold {
    /// The correction is added to the reconstructed sample **after** the
    /// decorrelation passes (spec §4.1, `read_word += correction`). This
    /// is the mono case and the stereo case *without* `CROSS_DECORR`. The
    /// raw [`fold_correction`] / [`fold_correction_pair`] primitives apply.
    PostDecorrelation,
    /// The correction is folded into the lossy input **before** the
    /// decorrelation passes (spec §4.1, the `CROSS_DECORR` `0x20`
    /// zero-delay correction); [`fold_correction_pre_decorrelation`]
    /// applies at the input stage.
    ///
    /// **Round-415 empirical pin:** current-version (`0x410`) reference
    /// encoders set `CROSS_DECORR` on their maximum-compression hybrid
    /// pairs, yet those files decode **bit-exactly with the
    /// post-decorrelation fold** — mono / left-right / joint, shaped and
    /// unshaped alike (see the `foreign_hybrid_pair_*cc*` fixtures). The
    /// bit is decorative on such files, exactly as it is on lossless
    /// stereo blocks; a genuinely pre-decorrelation-folded stream (the
    /// staged spec's description, presumably an earlier-version layout)
    /// has not been observed black-box. The end-to-end pair decoder
    /// therefore applies the post-decorrelation placement regardless of
    /// this flag.
    PreDecorrelationCross,
    /// The correction is applied through the `HYBRID_SHAPE` / `NEW_SHAPING`
    /// error-feedback filter (spec §4.1). Not a raw add; its
    /// `read_shaping_info` state layout is a documented gap, so no fold is
    /// available here.
    NoiseShaped,
}

impl CorrectionFold {
    /// Select the §4.1 fold placement from a block's 32-bit flag word.
    ///
    /// Precedence (a shaped block may also set `CROSS_DECORR`): the
    /// noise-shaping bits win first (the whole fold is filtered, so the
    /// placement question is moot), then `CROSS_DECORR`, then the default
    /// post-decorrelation raw add.
    #[must_use]
    pub fn from_flags(flags: u32) -> Self {
        if flags_select_shaping(flags) {
            CorrectionFold::NoiseShaped
        } else if flags & CROSS_DECORR_FLAG != 0 {
            CorrectionFold::PreDecorrelationCross
        } else {
            CorrectionFold::PostDecorrelation
        }
    }

    /// `true` when the block's corrections fold as a plain
    /// post-decorrelation raw add. Since round 415 this includes
    /// [`Self::PreDecorrelationCross`]: `CROSS_DECORR` is set decoratively
    /// by current-version reference encoders and their pairs are
    /// bit-exact under the post-decorrelation placement (see the variant
    /// docs). Only [`Self::NoiseShaped`] is excluded — a shaped fold
    /// routes each correction through the `0x07` error-feedback filter,
    /// so an after-the-fact raw add over a decoded buffer cannot
    /// reproduce it (use the end-to-end pair decode instead).
    #[must_use]
    pub fn is_supported_raw_fold(self) -> bool {
        matches!(
            self,
            CorrectionFold::PostDecorrelation | CorrectionFold::PreDecorrelationCross
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- 0x07 shaping state (round 408) --------------------------------

    #[test]
    fn shaping_seed_layouts_unpack_log_words() {
        // Mono dynamic: [error, acc, delta]; error word is negated.
        // Words from a reference-encoded correction block:
        // [0, -6736, -3602] -> error 0, acc wp_exp2s(-6736),
        // delta wp_exp2s(-3602).
        let mut p = Vec::new();
        for w in [0i16, -6736, -3602] {
            p.extend_from_slice(&w.to_le_bytes());
        }
        let st = ShapingState::from_shaping_words(Some(&p), false);
        assert_eq!(st.error(0), 0);
        let mut probe = st;
        // First advance: acc += delta, weight = acc >> 16.
        assert_eq!(probe.advance(0), 0, "zero error seed gives zero temp");

        // Stereo static: [err0, acc0, err1, acc1] (no deltas).
        let mut p = Vec::new();
        for w in [-1825i16, -6716, -1825, -6716] {
            p.extend_from_slice(&w.to_le_bytes());
        }
        let st = ShapingState::from_shaping_words(Some(&p), true);
        assert_eq!(st.error(0), -crate::logpack::wp_exp2s(-1825));
        assert_eq!(st.error(0), 70, "negated log-packed error seed");
        assert_eq!(st.error(1), 70);
    }

    #[test]
    fn shaping_absent_payload_is_the_raw_fold() {
        let mut st = ShapingState::from_shaping_words(None, false);
        for _ in 0..8 {
            assert_eq!(st.advance(0), 0);
            st.update(0, 123, 120, 0);
            // weight stays 0 (acc 0, delta 0) → temp stays 0 even with
            // a non-zero error state.
            assert_eq!(st.advance(0), 0);
        }
    }

    #[test]
    fn shaping_advance_wraps_on_adversarial_seeds() {
        // Round-415 fuzz find: adversarial 0x07 seed words can drive
        // `acc` / `delta` to magnitudes where `acc += delta` overflows
        // an i32 in debug builds, and an `i32::MIN` error state
        // overflowed the nudge's `|err| - 1`. The recurrence wraps in
        // 32 bits instead (the same posture as the round-386 wrapping
        // predictors); each call must simply return.
        let mut st = ShapingState {
            error: [i32::MIN, 5],
            acc: [i32::MAX, i32::MIN],
            delta: [i32::MAX, i32::MIN],
        };
        for _ in 0..8 {
            let t0 = st.advance(0);
            let t1 = st.advance(1);
            st.update(0, i32::MIN, i32::MAX, t0);
            st.update(1, i32::MAX, i32::MIN, t1);
        }
        // Nudge arm specifically: negative weight with err == i32::MIN.
        let mut nudge = ShapingState {
            error: [i32::MIN, 0],
            acc: [i32::MIN, 0],
            delta: [0, 0],
        };
        let t = nudge.advance(0);
        // |temp| stays strictly below |error|.
        assert!(t.unsigned_abs() < (i32::MIN).unsigned_abs());
    }

    #[test]
    fn shaping_seed_expansion_accepts_every_log_word() {
        // Round-415 fuzz find: the error-seed negation must wrap (a log
        // word can expand to i32::MIN). Every 16-bit word — and thus
        // every possible 0x07 seed byte pair — must construct.
        for w in i16::MIN..=i16::MAX {
            let b = w.to_le_bytes();
            let payload = [b[0], b[1], b[0], b[1], b[0], b[1]];
            let _ = ShapingState::from_shaping_words(Some(&payload), false);
            let _ = ShapingState::from_shaping_words(Some(&payload), true);
        }
    }

    #[test]
    fn shaping_recurrence_matches_the_black_box_pins() {
        // Weight -637 (acc seed wp_exp2s(-6736) after one delta of
        // wp_exp2s(-3602)), error 62 → temp +39; error -55 → temp -34
        // (round-408 ground-truth rows).
        let mut p = Vec::new();
        for w in [0i16, -6736, -3602] {
            p.extend_from_slice(&w.to_le_bytes());
        }
        let mut st = ShapingState::from_shaping_words(Some(&p), false);
        // Prime the error state as if the previous sample left q = 62.
        st.update(0, 62, 0, 0);
        assert_eq!(st.advance(0), 39);
        st.update(0, -55, 0, 0);
        assert_eq!(st.advance(0), -34);
    }

    #[test]
    fn shaping_unit_magnitude_nudge_caps_small_errors() {
        // Negative weight, |error| == 1 → temp 0; |error| == 2 with a
        // strong weight (-768 = -0.75) would round to 2 but is capped
        // at 1 (round-408 pin: the unit-magnitude nudge).
        let mut p = Vec::new();
        // acc word: wp_log2-domain value expanding to -768 << 16.
        // wp_exp2s(-6806) == -50331648 == -768 * 65536.
        for w in [0i16, -6806, 0] {
            p.extend_from_slice(&w.to_le_bytes());
        }
        let mut st = ShapingState::from_shaping_words(Some(&p), false);
        assert_eq!(crate::logpack::wp_exp2s(-6806), -768 << 16);
        st.update(0, 1, 0, 0);
        assert_eq!(st.advance(0), 0, "unit error is inert");
        st.update(0, -1, 0, 0);
        assert_eq!(st.advance(0), 0);
        st.update(0, 2, 0, 0);
        assert_eq!(st.advance(0), 1, "capped strictly below |error|");
        st.update(0, 90, 0, 0);
        assert_eq!(st.advance(0), 68, "half products round away from zero");
    }

    #[test]
    fn shaping_error_update_branches_on_weight_sign() {
        // Positive weight: error accumulates q - temp; negative: q.
        let mut p = Vec::new();
        for w in [0i16, 6806, 0] {
            p.extend_from_slice(&w.to_le_bytes());
        }
        let mut pos = ShapingState::from_shaping_words(Some(&p), false);
        pos.update(0, 100, 60, 7);
        assert_eq!(pos.error(0), 100 - 60 - 7);

        let mut p = Vec::new();
        for w in [0i16, -6806, 0] {
            p.extend_from_slice(&w.to_le_bytes());
        }
        let mut neg = ShapingState::from_shaping_words(Some(&p), false);
        neg.update(0, 100, 60, 7);
        assert_eq!(neg.error(0), 40);
    }

    // ---- 0x06 profile expansion (round 408) ---------------------------

    #[test]
    fn expand_mono_profile_reads_level_and_bitrate() {
        // The b4 mono profile observed on reference files:
        // level word 0x143f, bitrate word 0x01c8 (456).
        let p = expand_hybrid_profile(&[0x3f, 0x14, 0xc8, 0x01], false).unwrap();
        assert_eq!(p.level_words, [0x143f, 0]);
        assert_eq!(p.bitrate, 456);
        assert_eq!(p.balance, 0);
        assert!(!p.stereo);
    }

    #[test]
    fn expand_stereo_profile_reads_levels_rate_and_balance() {
        // The b4 mid/side stereo profile observed on reference files:
        // level words 0x1412 / 0x13e3, shared bitrate 456, balance 256.
        let p =
            expand_hybrid_profile(&[0x12, 0x14, 0xe3, 0x13, 0xc8, 0x01, 0x00, 0x01], true).unwrap();
        assert_eq!(p.level_words, [0x1412, 0x13e3]);
        assert_eq!(p.bitrate, 456);
        assert_eq!(p.balance, 256);
        assert!(p.stereo);
    }

    #[test]
    fn expand_rejects_shape_mismatched_lengths() {
        for (payload, stereo) in [
            (&[0u8; 4][..], true),  // stereo block, mono-sized payload
            (&[0u8; 8][..], false), // mono block, stereo-sized payload
            (&[0u8; 2][..], false),
            (&[0u8; 6][..], true),
            (&[0u8; 0][..], false),
        ] {
            assert_eq!(
                expand_hybrid_profile(payload, stereo),
                Err(Error::HybridProfileLength(payload.len())),
                "len {} stereo {stereo}",
                payload.len()
            );
        }
    }

    // ---- slow-level state + limits (round-408 black-box pins) ---------

    #[test]
    fn mono_state_seeds_and_derives_the_pinned_limit() {
        // Level word 0x143f log-unpacks to 622592; ema = 2432;
        // arg = 2432 - 456 + 256 = 2232; wp_exp2s(2232) = 210. These
        // are the exact opening values of the reference-encoded b4
        // mono fixture the recurrence was pinned against.
        let p = expand_hybrid_profile(&[0x3f, 0x14, 0xc8, 0x01], false).unwrap();
        let state = HybridState::from_profile(&p);
        assert_eq!(state.slow_level(0), crate::logpack::wp_exp2s(0x143f) as u32);
        assert_eq!(state.slow_level(0), 622_592);
        assert_eq!(state.frame_limits(), [210, 0]);
    }

    #[test]
    fn update_applies_the_slow_level_recurrence() {
        // sl' = sl - ((sl + 128) >> 8) + wp_log2(mag).
        let p = expand_hybrid_profile(&[0x3f, 0x14, 0xc8, 0x01], false).unwrap();
        let mut state = HybridState::from_profile(&p);
        let sl = state.slow_level(0);
        state.update(0, 1000);
        let expect = sl - ((sl + 128) >> 8) + crate::logpack::wp_log2(1000) as u32;
        assert_eq!(state.slow_level(0), expect);
        // Zero magnitudes decay the level (wp_log2(0) == 0).
        let sl = state.slow_level(0);
        state.update(0, 0);
        assert_eq!(state.slow_level(0), sl - ((sl + 128) >> 8));
    }

    #[test]
    fn update_signed_uses_the_complement_magnitude() {
        // A negative sample's magnitude is its bitwise complement
        // (spec §4.2 step 7 sign rule): -1000 → 999.
        let p = expand_hybrid_profile(&[0x3f, 0x14, 0xc8, 0x01], false).unwrap();
        let mut a = HybridState::from_profile(&p);
        let mut b = HybridState::from_profile(&p);
        a.update_signed(0, -1000);
        b.update(0, 999);
        assert_eq!(a.slow_level(0), b.slow_level(0));
    }

    #[test]
    fn stereo_limits_redistribute_around_the_balance_word() {
        // The frame-start delta rule: delta = (ema0 - ema1 - balance)
        // >> 1 (arithmetic); arg0 = ema0 - delta - rate + 256, arg1 =
        // ema1 + delta - rate + 256. With ema0 - ema1 == balance the
        // channels behave as two independent mono channels.
        let p =
            expand_hybrid_profile(&[0x12, 0x14, 0xe3, 0x13, 0xc8, 0x01, 0x00, 0x01], true).unwrap();
        let state = HybridState::from_profile(&p);
        let ema0 = ((state.slow_level(0) + 128) >> 8) as i32;
        let ema1 = ((state.slow_level(1) + 128) >> 8) as i32;
        let delta = (ema0 - ema1 - 256) >> 1;
        let expect0 = crate::logpack::wp_exp2s(ema0 - delta - 456 + 256) as u32;
        let expect1 = crate::logpack::wp_exp2s(ema1 + delta - 456 + 256) as u32;
        assert_eq!(state.frame_limits(), [expect0, expect1]);
    }

    #[test]
    fn non_positive_limit_argument_means_lossless() {
        // A large bitrate word pushes the argument non-positive: the
        // limit is 0 and the §6.5 dispatch takes the lossless mantissa
        // path.
        let p = expand_hybrid_profile(&[0x3f, 0x14, 0xff, 0x7f], false).unwrap();
        let state = HybridState::from_profile(&p);
        assert_eq!(state.frame_limits(), [0, 0]);
    }

    #[test]
    fn negative_level_words_clamp_to_zero() {
        // A negative log word would unpack to a negative "magnitude";
        // the seed clamps at zero instead of wrapping through u32.
        let p = expand_hybrid_profile(&[0x00, 0x80, 0x00, 0x00], false).unwrap();
        let state = HybridState::from_profile(&p);
        assert_eq!(state.slow_level(0), 0);
    }

    // ---- constants ---------------------------------------------------

    #[test]
    fn flag_constants_match_spec_section_6() {
        assert_eq!(HYBRID_FLAG, 0x08);
        assert_eq!(CROSS_DECORR_FLAG, 0x20);
        assert_eq!(HYBRID_SHAPE_FLAG, 0x40);
        assert_eq!(NEW_SHAPING_FLAG, 0x2000_0000);
    }

    // ---- fold / split round trip (the §4.1 lossless recovery) --------

    #[test]
    fn fold_recovers_original_from_lossy_and_correction() {
        // The defining property: lossy + (original - lossy) == original.
        for original in [-1000i32, -7, -1, 0, 1, 5, 1000] {
            for lossy in [-2000i32, -3, 0, 4, 2000] {
                let correction = split_correction(original, lossy);
                assert_eq!(fold_correction(lossy, correction), original);
            }
        }
    }

    #[test]
    fn fold_is_plain_addition() {
        assert_eq!(fold_correction(10, 3), 13);
        assert_eq!(fold_correction(10, -3), 7);
        assert_eq!(fold_correction(-10, 3), -7);
        assert_eq!(fold_correction(0, 0), 0);
    }

    #[test]
    fn split_is_plain_subtraction() {
        assert_eq!(split_correction(13, 10), 3);
        assert_eq!(split_correction(7, 10), -3);
        assert_eq!(split_correction(0, 0), 0);
    }

    #[test]
    fn zero_correction_is_the_identity() {
        // A hybrid block with a perfectly-lossless main stream carries a
        // zero correction; the fold must leave the reconstructed value
        // untouched.
        for v in [-5i32, 0, 5, i32::MIN, i32::MAX] {
            assert_eq!(fold_correction(v, 0), v);
            assert_eq!(fold_correction_pre_decorrelation(v, 0), v);
        }
    }

    // ---- wrap-around behaviour ---------------------------------------

    #[test]
    fn fold_wraps_past_i32_bounds_like_the_decoder_register() {
        // The decoder folds in a 32-bit register; an overshoot wraps
        // rather than panicking (debug builds would panic on a plain +).
        assert_eq!(fold_correction(i32::MAX, 1), i32::MIN);
        assert_eq!(fold_correction(i32::MIN, -1), i32::MAX);
    }

    #[test]
    fn split_wraps_past_i32_bounds() {
        assert_eq!(split_correction(i32::MIN, 1), i32::MAX);
        assert_eq!(split_correction(i32::MAX, -1), i32::MIN);
    }

    #[test]
    fn fold_and_split_round_trip_at_the_extremes() {
        // Even where the intermediate correction wraps, fold ∘ split is
        // the identity in the 32-bit register.
        for original in [i32::MIN, i32::MIN + 1, -1, 0, 1, i32::MAX - 1, i32::MAX] {
            for lossy in [i32::MIN, -1, 0, 1, i32::MAX] {
                let c = split_correction(original, lossy);
                assert_eq!(fold_correction(lossy, c), original);
            }
        }
    }

    // ---- pre-decorrelation fold (CROSS_DECORR) -----------------------

    #[test]
    fn pre_decorrelation_fold_matches_post_arithmetic() {
        // The pre- and post-decorrelation folds are the same add; only
        // the pipeline position the typed name marks differs.
        for a in [-100i32, -1, 0, 7, 100] {
            for b in [-50i32, 0, 3, 50] {
                assert_eq!(
                    fold_correction_pre_decorrelation(a, b),
                    fold_correction(a, b)
                );
            }
        }
    }

    // ---- pair folds --------------------------------------------------

    #[test]
    fn pair_fold_applies_per_channel() {
        let reconstructed = (10i32, -20i32);
        let correction = (3i32, 5i32);
        assert_eq!(fold_correction_pair(reconstructed, correction), (13, -15));
    }

    #[test]
    fn pre_decorrelation_pair_fold_applies_per_channel() {
        let lossy = (100i32, -200i32);
        let correction = (-1i32, 2i32);
        assert_eq!(
            fold_correction_pre_decorrelation_pair(lossy, correction),
            (99, -198)
        );
    }

    #[test]
    fn pair_fold_recovers_original_pair() {
        let original = (1234i32, -5678i32);
        let lossy = (1200i32, -5700i32);
        let correction = (
            split_correction(original.0, lossy.0),
            split_correction(original.1, lossy.1),
        );
        assert_eq!(fold_correction_pair(lossy, correction), original);
    }

    // ---- shaping detection -------------------------------------------

    #[test]
    fn shaping_detection_keys_on_the_shape_bits() {
        assert!(!flags_select_shaping(0));
        assert!(!flags_select_shaping(HYBRID_FLAG));
        assert!(!flags_select_shaping(CROSS_DECORR_FLAG));
        assert!(flags_select_shaping(HYBRID_SHAPE_FLAG));
        assert!(flags_select_shaping(NEW_SHAPING_FLAG));
        assert!(flags_select_shaping(HYBRID_SHAPE_FLAG | NEW_SHAPING_FLAG));
        // Mixed with unrelated bits set: still detected.
        assert!(flags_select_shaping(HYBRID_FLAG | HYBRID_SHAPE_FLAG));
    }

    // ---- fold placement selector -------------------------------------

    #[test]
    fn placement_defaults_to_post_decorrelation() {
        // A plain hybrid block (no cross, no shaping) folds the correction
        // after decorrelation.
        assert_eq!(
            CorrectionFold::from_flags(HYBRID_FLAG),
            CorrectionFold::PostDecorrelation
        );
        assert_eq!(
            CorrectionFold::from_flags(0),
            CorrectionFold::PostDecorrelation
        );
        assert!(CorrectionFold::from_flags(HYBRID_FLAG).is_supported_raw_fold());
    }

    #[test]
    fn placement_selects_cross_for_cross_decorr() {
        assert_eq!(
            CorrectionFold::from_flags(HYBRID_FLAG | CROSS_DECORR_FLAG),
            CorrectionFold::PreDecorrelationCross
        );
        // Round 415: the raw post-decorrelation fold covers cross-flagged
        // blocks too — current-version reference encoders set the bit
        // decoratively and their pairs are bit-exact under the raw add.
        assert!(CorrectionFold::from_flags(HYBRID_FLAG | CROSS_DECORR_FLAG).is_supported_raw_fold());
    }

    #[test]
    fn placement_selects_shaped_when_shaping_bits_set() {
        assert_eq!(
            CorrectionFold::from_flags(HYBRID_FLAG | HYBRID_SHAPE_FLAG),
            CorrectionFold::NoiseShaped
        );
        assert_eq!(
            CorrectionFold::from_flags(HYBRID_FLAG | NEW_SHAPING_FLAG),
            CorrectionFold::NoiseShaped
        );
        // Shaping wins over cross when both are set.
        assert_eq!(
            CorrectionFold::from_flags(HYBRID_FLAG | CROSS_DECORR_FLAG | HYBRID_SHAPE_FLAG),
            CorrectionFold::NoiseShaped
        );
        assert!(
            !CorrectionFold::from_flags(HYBRID_FLAG | HYBRID_SHAPE_FLAG).is_supported_raw_fold()
        );
    }
}
