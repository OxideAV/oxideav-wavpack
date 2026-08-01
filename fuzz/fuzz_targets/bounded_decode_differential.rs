#![no_main]

//! Differential target over the round-436 `*_bounded` decode surface.
//!
//! The bounded twins must be **pure budget gates**: within the budget
//! they are bit-identical to the unbounded decoders (same PCM, same
//! errors), and past it they surface the typed
//! [`oxideav_wavpack::Error::DecodeBudgetExceeded`] instead of any
//! other outcome. This target asserts, per fuzz input:
//!
//! 1. **Unlimited parity** — `*_bounded(…, u64::MAX)` returns exactly
//!    what the unbounded twin returns (Ok payload and Err value alike)
//!    for `decode_stream`, `decode_stream_muted`,
//!    `decode_multichannel_stream` and its muted twin, plus the pair
//!    walkers at the empty-correction identity.
//! 2. **Exact-budget parity** — when the unbounded decode succeeds
//!    with `n` emitted values, a budget of exactly `n` succeeds with
//!    identical output, and (for `n > 0`) a budget of `n - 1` fails
//!    with `DecodeBudgetExceeded`.
//! 3. **Fuzz-chosen budget** — with a budget steered by the leading
//!    control byte, a bounded success is bit-equal to the unbounded
//!    result and never exceeds the budget; a budget refusal implies
//!    the unbounded outcome would have emitted (or charged) past it,
//!    reported with `needed > budget`.
//!
//! **RSS sizing note:** the unbounded reference decode still carries
//! the format's inherent zero-run amplification (a ~50-byte block may
//! decode to `MAX_DECODE_SAMPLES_PER_BLOCK` zeros), so campaigns use
//! the same `-rss_limit_mb` sizing as the `decode_stream` target.

use libfuzzer_sys::fuzz_target;
use oxideav_wavpack::Error;

/// Check invariants 1–3 for one `(unbounded, bounded)` decoder pair
/// whose success payload is `T`.
fn check_surface<T: PartialEq + core::fmt::Debug>(
    name: &str,
    unbounded: impl Fn() -> Result<T, Error>,
    bounded: impl Fn(u64) -> Result<T, Error>,
    emitted: impl Fn(&T) -> u64,
    fuzz_budget: u64,
) {
    let reference = unbounded();
    // 1. Unlimited parity.
    assert_eq!(
        bounded(u64::MAX),
        reference,
        "{name}: unlimited budget must match the unbounded twin"
    );
    // 2. Exact-budget parity.
    if let Ok(ref value) = reference {
        let n = emitted(value);
        assert_eq!(
            bounded(n),
            reference,
            "{name}: exact budget {n} must match the unbounded twin"
        );
        if n > 0 {
            match bounded(n - 1) {
                Err(Error::DecodeBudgetExceeded { budget, needed }) => {
                    assert_eq!(budget, n - 1, "{name}: refusal echoes the budget");
                    assert!(needed > budget, "{name}: needed must exceed the budget");
                }
                other => panic!("{name}: budget {} must refuse, got {other:?}", n - 1),
            }
        }
    }
    // 3. Fuzz-chosen budget.
    match bounded(fuzz_budget) {
        Ok(value) => {
            assert!(
                emitted(&value) <= fuzz_budget,
                "{name}: bounded output exceeds its budget"
            );
            assert_eq!(
                Ok(value),
                reference,
                "{name}: bounded success must be bit-equal to the unbounded result"
            );
        }
        Err(Error::DecodeBudgetExceeded { budget, needed }) => {
            assert_eq!(budget, fuzz_budget, "{name}: refusal echoes the budget");
            assert!(needed > budget, "{name}: needed must exceed the budget");
            // A budget refusal is only legitimate when the unbounded
            // outcome (had it succeeded) really emits past the budget —
            // the gate must never fire below a successful decode's size.
            if let Ok(ref value) = reference {
                assert!(
                    emitted(value) > fuzz_budget,
                    "{name}: refused a stream the unbounded twin decodes within budget"
                );
            }
        }
        Err(other) => {
            // Any non-budget error must be the unbounded twin's error.
            assert_eq!(
                Err(other),
                reference,
                "{name}: non-budget errors must match the unbounded twin"
            );
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let Some((&ctl, stream)) = data.split_first() else {
        return;
    };
    // Budget spread: 0, small counts, and multi-KiB values.
    let fuzz_budget = u64::from(ctl) * 257;

    check_surface(
        "decode_stream",
        || oxideav_wavpack::decode_stream(stream),
        |b| oxideav_wavpack::decode_stream_bounded(stream, b),
        |pcm| pcm.len() as u64,
        fuzz_budget,
    );
    check_surface(
        "decode_stream_muted",
        || oxideav_wavpack::decode_stream_muted(stream),
        |b| oxideav_wavpack::decode_stream_muted_bounded(stream, b),
        |(pcm, _ok)| pcm.len() as u64,
        fuzz_budget,
    );
    check_surface(
        "decode_multichannel_stream",
        || oxideav_wavpack::decode_multichannel_stream(stream),
        |b| oxideav_wavpack::decode_multichannel_stream_bounded(stream, b),
        |s| s.samples.len() as u64,
        fuzz_budget,
    );
    check_surface(
        "decode_multichannel_stream_muted",
        || oxideav_wavpack::decode_multichannel_stream_muted(stream),
        |b| oxideav_wavpack::decode_multichannel_stream_muted_bounded(stream, b),
        |(s, _ok)| s.samples.len() as u64,
        fuzz_budget,
    );
    // Pair walkers at the empty-correction identity (every block pairs
    // with `None`, so the budget arithmetic is the single-file one).
    check_surface(
        "decode_stream_with_correction",
        || oxideav_wavpack::decode_stream_with_correction(stream, &[]),
        |b| oxideav_wavpack::decode_stream_with_correction_bounded(stream, &[], b),
        |pcm| pcm.len() as u64,
        fuzz_budget,
    );
    check_surface(
        "decode_stream_with_correction_muted",
        || oxideav_wavpack::decode_stream_with_correction_muted(stream, &[]),
        |b| oxideav_wavpack::decode_stream_with_correction_muted_bounded(stream, &[], b),
        |(pcm, _ok)| pcm.len() as u64,
        fuzz_budget,
    );
});
