//! `oxideav-core` framework wiring: the [`Decoder`] / [`Encoder`]
//! trait impls, the direct [`make_decoder`] / [`make_encoder`] factory
//! endpoints, and the [`register`] entry point the umbrella's
//! `register_all` dispatch calls.
//!
//! # Packet contract
//!
//! WavPack blocks are self-describing (`wvpk` magic + 32-byte header +
//! metadata sub-blocks), so the packet payload is simply **one or more
//! complete blocks** — a whole member set (or several) per packet. The
//! decoder hands each packet to
//! [`crate::decode_multichannel_stream`], which covers plain mono /
//! stereo files and multichannel member-set grouping alike; the
//! encoder emits one packet per input frame, each carrying the blocks
//! for that frame's samples with a **running `block_index`**, so the
//! concatenated packet payloads form a single contiguous — and
//! therefore seekable ([`crate::StreamIndex::is_seekable`]) — `.wv`
//! chain.
//!
//! # Sample formats
//!
//! WavPack stores the container width in the flags word (wiki bits
//! 0..=1, "bytes per sample minus one"). The decoder emits the exact
//! matching interleaved format — [`SampleFormat::S8`] /
//! [`SampleFormat::S16`] / [`SampleFormat::S24`] /
//! [`SampleFormat::S32`] — with the decoded values verbatim (no
//! rescaling), so the PCM bytes are lossless round-trip material;
//! `FLOAT_DATA` streams decode to their IEEE-754 bit patterns in the
//! 4-byte slots (byte-identical to interleaved [`SampleFormat::F32`]),
//! and hybrid `.wv` streams decode to their coarse (lossy) PCM —
//! rounds 408..418 wired the full lossy / float / int32 decode paths
//! underneath this surface.
//!
//! The encoder accepts the four signed interleaved widths (from
//! `CodecParameters::sample_format`, default [`SampleFormat::S16`])
//! plus [`SampleFormat::F32`] since round 420: `S32` input routes
//! through the `0x09` int32 deconstruction and `F32` through the
//! `0x08` float deconstruction, both lossless. Planar layouts stay
//! refused.
//!
//! # Encoder options (round 420)
//!
//! Declared via [`WavPackEncoderOptions`] and parsed from
//! [`CodecParameters::options`]:
//!
//! * `mode` — `"lossless"` (default) or `"hybrid"` (lossy `.wv` at a
//!   bits-per-sample noise target).
//! * `bits_per_sample` — hybrid noise target on the reference `-b`
//!   scale (default 4.0; ~2.0 aggressive … 6.0+ near-lossless).
//! * `shaping` — hybrid noise-shaping weight in `-1.0..=1.0`
//!   (default 0.0 = off; positive tilts the quantization noise
//!   upward). See [`crate::HybridShaping`].
//! * `joint` — hybrid stereo joint (mid/side) coding (default true).
//! * `correction` — requesting the `.wvc` twin is **refused**: the
//!   framework packet contract is single-stream, so two-file hybrid
//!   pairs are only available through the crate-level
//!   [`crate::encode_stream_mono_hybrid`]-family APIs
//!   ([`crate::HybridOptions::correction`]).
//!
//! Hybrid mode spans blocks across packets with the same running
//! `0x06` level-word / `0x07` shaping-state carry the crate's stream
//! encoders use, so concatenated packets form one conformant chain.
//!
//! # Decoder options (round 436)
//!
//! Declared via [`WavPackDecoderOptions`]:
//!
//! * `max_packet_samples` — per-packet decoded-sample budget (total
//!   emitted values across all channels; the spec §4.2 zero-run path
//!   makes a block's output unbounded by its payload bytes, so this is
//!   the registry surface of [`crate::decode_stream_bounded`]'s
//!   anti-amplification guard). Default
//!   [`DEFAULT_PACKET_SAMPLE_BUDGET`]; `0` disables the budget.

use std::collections::VecDeque;

use oxideav_core::{
    options::{parse_options, CodecOptionsStruct, OptionField, OptionKind, OptionValue},
    CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecTag, Decoder, Encoder,
    Error as CoreError, Frame, Packet, Result as CoreResult, RuntimeContext, SampleFormat,
    TimeBase,
};

use crate::block_header::TOTAL_SAMPLES_UNKNOWN;
use crate::encode::{
    encode_block_mono_best, encode_block_mono_float_best, encode_block_mono_int32_best,
    encode_block_stereo_best, encode_block_stereo_float_best, encode_block_stereo_int32_best,
    encode_multichannel_stream_best_at, encode_multichannel_stream_float_at,
    encode_multichannel_stream_int32_at, DecorrProfile, DEFAULT_BLOCK_SAMPLES,
};
use crate::hybrid_encode::{
    encode_hybrid_block_float, encode_hybrid_block_int32, encode_hybrid_block_ints,
    encode_multichannel_hybrid_members, new_multichannel_carries, HybridCarry,
};
use crate::{HybridOptions, HybridShaping};

/// The registry identifier this crate registers under.
pub const CODEC_ID: &str = "wavpack";

/// Map a crate-level error into the framework's invalid-data error,
/// keeping the typed message.
fn invalid(e: crate::Error) -> CoreError {
    CoreError::invalid(format!("wavpack: {e}"))
}

// ───────────────────────── decoder ─────────────────────────

/// Default per-packet decoded-sample budget for [`WavPackDecoder`]:
/// `2^28` emitted sample values (~1 GiB of packed `i32` slots).
///
/// The spec §4.2 step 1 zero-run fast path makes a block's output
/// unbounded by its payload bytes, so without a budget a few hundred
/// hostile packet bytes could demand gigabytes of frame allocation
/// (round 436 — see [`crate::decode_stream_bounded`]). One packet is
/// one frame's worth of blocks, so `2^28` values (over 25 minutes of
/// 44.1 kHz stereo in a single packet) is far beyond any plausible
/// real frame while keeping the worst-case allocation bounded. A
/// defensive engineering bound, not a spec limit — override it (or
/// disable it with `0`) via the `max_packet_samples` decoder option.
pub const DEFAULT_PACKET_SAMPLE_BUDGET: u32 = 1 << 28;

/// Typed decoder options (parsed from [`CodecParameters::options`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WavPackDecoderOptions {
    /// Per-packet decoded-sample budget (total emitted sample values
    /// across all channels). `0` disables the budget. Default
    /// [`DEFAULT_PACKET_SAMPLE_BUDGET`].
    pub max_packet_samples: u32,
}

impl Default for WavPackDecoderOptions {
    fn default() -> Self {
        WavPackDecoderOptions {
            max_packet_samples: DEFAULT_PACKET_SAMPLE_BUDGET,
        }
    }
}

impl CodecOptionsStruct for WavPackDecoderOptions {
    const SCHEMA: &'static [OptionField] = &[OptionField {
        name: "max_packet_samples",
        kind: OptionKind::U32,
        default: OptionValue::U32(DEFAULT_PACKET_SAMPLE_BUDGET),
        help: "per-packet decoded-sample budget (anti-amplification guard; 0 = unlimited)",
    }];

    fn apply(&mut self, key: &str, value: &OptionValue) -> CoreResult<()> {
        match key {
            "max_packet_samples" => self.max_packet_samples = value.as_u32()?,
            _ => unreachable!("guarded by SCHEMA"),
        }
        Ok(())
    }
}

/// Framework [`Decoder`] over
/// [`crate::decode_multichannel_stream_bounded`].
///
/// Each packet must carry complete `wvpk` blocks (whole member sets).
/// One [`oxideav_core::AudioFrame`] is produced per packet, with the
/// per-frame channel count taken from the decoded member-set grouping
/// and the sample bytes packed at the stream's container width. Every
/// packet's decode is bounded by the per-packet sample budget
/// ([`WavPackDecoderOptions::max_packet_samples`], default
/// [`DEFAULT_PACKET_SAMPLE_BUDGET`]) — a hostile packet whose block
/// headers demand more surfaces an invalid-data error before any
/// amplified allocation.
#[derive(Debug)]
pub struct WavPackDecoder {
    codec_id: CodecId,
    /// Per-packet decoded-sample budget (`u64::MAX` = disabled).
    packet_budget: u64,
    queue: VecDeque<oxideav_core::AudioFrame>,
    eof: bool,
}

impl WavPackDecoder {
    fn new(opts: &WavPackDecoderOptions) -> Self {
        Self {
            codec_id: CodecId::new(CODEC_ID),
            packet_budget: match opts.max_packet_samples {
                0 => u64::MAX,
                n => u64::from(n),
            },
            queue: VecDeque::new(),
            eof: false,
        }
    }
}

/// Pack decoded `i32` container-scaled samples into interleaved bytes
/// at the given container width (`1..=4` bytes per sample,
/// little-endian, low bytes of the `i32` value verbatim — the exact
/// inverse of [`widen_bytes`]).
fn pack_bytes(samples: &[i32], bytes_per_sample: u8) -> Vec<u8> {
    let width = usize::from(bytes_per_sample);
    let mut out = Vec::with_capacity(samples.len() * width);
    for s in samples {
        out.extend_from_slice(&s.to_le_bytes()[..width]);
    }
    out
}

/// Sign-extend interleaved little-endian bytes at the given container
/// width back into `i32` samples (the exact inverse of [`pack_bytes`]).
fn widen_bytes(bytes: &[u8], bytes_per_sample: u8) -> Vec<i32> {
    let width = usize::from(bytes_per_sample);
    let mut out = Vec::with_capacity(bytes.len() / width);
    for chunk in bytes.chunks_exact(width) {
        let mut v = [0u8; 4];
        v[..width].copy_from_slice(chunk);
        let raw = i32::from_le_bytes(v);
        // Sign-extend from the container width.
        let shift = 32 - 8 * width as u32;
        out.push(raw.wrapping_shl(shift).wrapping_shr(shift));
    }
    out
}

/// The WavPack container width for a supported interleaved
/// [`SampleFormat`] (the wiki flags bits 0..=1 "bytes per sample minus
/// one" field, plus one). `F32` maps to the 4-byte `FLOAT_DATA`
/// container.
fn width_for_format(format: SampleFormat) -> CoreResult<u8> {
    match format {
        SampleFormat::S8 => Ok(1),
        SampleFormat::S16 => Ok(2),
        SampleFormat::S24 => Ok(3),
        SampleFormat::S32 | SampleFormat::F32 => Ok(4),
        other => Err(CoreError::unsupported(format!(
            "wavpack: sample format {other:?} not supported (signed interleaved S8/S16/S24/S32 or F32)"
        ))),
    }
}

impl Decoder for WavPackDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> CoreResult<()> {
        if self.eof {
            return Err(CoreError::invalid(
                "wavpack: send_packet after flush (reset the decoder to reuse it)",
            ));
        }
        // Container width from the first audio block's header (wiki
        // flags bits 0..=1) — needed to pick the output byte packing.
        let first_audio = crate::block::first_audio_block(&packet.data).map_err(invalid)?;
        let Some(first_audio) = first_audio else {
            // Metadata-only packet: nothing to emit.
            return Ok(());
        };
        let bytes_per_sample = first_audio.flags().bytes_per_sample();
        let stream =
            crate::block::decode_multichannel_stream_bounded(&packet.data, self.packet_budget)
                .map_err(invalid)?;
        let Some(frames) = stream.samples.len().checked_div(stream.channels) else {
            // channels == 0: no audio sets survived (nothing to emit).
            return Ok(());
        };
        self.queue.push_back(oxideav_core::AudioFrame {
            samples: u32::try_from(frames)
                .map_err(|_| CoreError::invalid("wavpack: packet decodes to > u32::MAX frames"))?,
            pts: packet.pts,
            data: vec![pack_bytes(&stream.samples, bytes_per_sample)],
        });
        Ok(())
    }

    fn receive_frame(&mut self) -> CoreResult<Frame> {
        match self.queue.pop_front() {
            Some(frame) => Ok(Frame::Audio(frame)),
            None if self.eof => Err(CoreError::Eof),
            None => Err(CoreError::NeedMore),
        }
    }

    fn flush(&mut self) -> CoreResult<()> {
        self.eof = true;
        Ok(())
    }

    fn reset(&mut self) -> CoreResult<()> {
        // Blocks are independently decodable — no carry-over state
        // beyond the output queue and the EOS latch.
        self.queue.clear();
        self.eof = false;
        Ok(())
    }
}

/// Direct decoder factory (the historical `decoder::make_decoder`
/// endpoint; also the factory [`register`] installs).
///
/// WavPack blocks are self-describing, so no `extradata` or up-front
/// parameter agreement is required. The optional typed
/// [`WavPackDecoderOptions`] schema (`max_packet_samples`) is parsed
/// from [`CodecParameters::options`]. A decoder is commonly built from
/// the *same* `CodecParameters` an encoder was (one parameter set for
/// both directions of a loop), so keys owned by the
/// [`WavPackEncoderOptions`] schema are tolerated and ignored here;
/// keys neither schema owns are refused.
pub fn make_decoder(params: &CodecParameters) -> CoreResult<Box<dyn Decoder>> {
    let mut decoder_bag = oxideav_core::options::CodecOptions::new();
    for (k, v) in params.options.iter() {
        if WavPackDecoderOptions::SCHEMA.iter().any(|f| f.name == k) {
            decoder_bag.insert(k, v);
        } else if !WavPackEncoderOptions::SCHEMA.iter().any(|f| f.name == k) {
            return Err(CoreError::invalid(format!("wavpack: unknown option '{k}'")));
        }
    }
    let opts: WavPackDecoderOptions = parse_options(&decoder_bag)?;
    Ok(Box::new(WavPackDecoder::new(&opts)))
}

// ───────────────────────── encoder ─────────────────────────

/// Typed encoder options (see the module docs for the schema).
#[derive(Debug, Clone, PartialEq)]
pub struct WavPackEncoderOptions {
    /// `"lossless"` (default) or `"hybrid"`.
    pub mode: String,
    /// Hybrid bits-per-sample noise target (the reference `-b` scale).
    pub bits_per_sample: f32,
    /// Hybrid noise-shaping weight, `-1.0..=1.0` (`0.0` = off).
    pub shaping: f32,
    /// Hybrid stereo joint (mid/side) coding.
    pub joint: bool,
    /// Request the `.wvc` correction twin — always refused here (the
    /// packet contract is single-stream); use the crate-level pair
    /// APIs instead.
    pub correction: bool,
}

impl Default for WavPackEncoderOptions {
    fn default() -> Self {
        WavPackEncoderOptions {
            mode: "lossless".into(),
            bits_per_sample: 4.0,
            shaping: 0.0,
            joint: true,
            correction: false,
        }
    }
}

impl CodecOptionsStruct for WavPackEncoderOptions {
    const SCHEMA: &'static [OptionField] = &[
        OptionField {
            name: "mode",
            kind: OptionKind::Enum(&["lossless", "hybrid"]),
            default: OptionValue::String(String::new()),
            help: "encode mode: lossless (default) or hybrid (lossy .wv at a noise target)",
        },
        OptionField {
            name: "bits_per_sample",
            kind: OptionKind::F32,
            default: OptionValue::F32(4.0),
            help: "hybrid noise target in bits per sample (~2.0 aggressive .. 6.0+ near-lossless)",
        },
        OptionField {
            name: "shaping",
            kind: OptionKind::F32,
            default: OptionValue::F32(0.0),
            help:
                "hybrid noise-shaping weight in -1.0..=1.0 (0 = off, positive tilts noise upward)",
        },
        OptionField {
            name: "joint",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(true),
            help: "hybrid stereo joint (mid/side) coding",
        },
        OptionField {
            name: "correction",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(false),
            help: "emit the .wvc correction twin (unsupported here; use the crate pair APIs)",
        },
    ];

    fn apply(&mut self, key: &str, value: &OptionValue) -> CoreResult<()> {
        match key {
            "mode" => self.mode = value.as_str()?.to_string(),
            "bits_per_sample" => self.bits_per_sample = value.as_f32()?,
            "shaping" => self.shaping = value.as_f32()?,
            "joint" => self.joint = value.as_bool()?,
            "correction" => self.correction = value.as_bool()?,
            _ => unreachable!("guarded by SCHEMA"),
        }
        Ok(())
    }
}

/// The encoder's per-stream mode state.
#[derive(Debug)]
enum EncodeMode {
    /// Lossless mode-searched blocks (the historical behaviour).
    Lossless,
    /// Hybrid lossy `.wv` blocks with the cross-packet level / shaping
    /// carry (`HybridCarry` is created on the first frame, once the
    /// channel shape is known to be mono or stereo).
    Hybrid {
        opts: HybridOptions,
        carry: Option<HybridCarry>,
        /// Per-member-chain carries for a multichannel (>2-channel)
        /// stream (round 447); created on the first frame.
        mc_carries: Option<Vec<HybridCarry>>,
    },
}

/// Framework [`Encoder`] over the crate's self-deriving `*_best`
/// entry points.
///
/// Consumes interleaved signed PCM frames
/// ([`CodecParameters::sample_format`], default [`SampleFormat::S16`])
/// and emits one packet per input frame carrying complete `wvpk`
/// blocks with a running `block_index`, so the concatenated packet
/// payloads are one contiguous, seekable `.wv` chain:
/// `decode_multichannel_stream(concat(packets))` reproduces the
/// concatenated input PCM bit-exactly.
///
/// Mono and stereo use the full mode-search encoders
/// ([`encode_block_mono_best`] / [`encode_block_stereo_best`], joint /
/// decorrelation / left-shift decisions included, at
/// [`DecorrProfile::Normal`]); wider layouts use the mono-member
/// multichannel grouping ([`encode_multichannel_stream_at`]).
#[derive(Debug)]
pub struct WavPackEncoder {
    params: CodecParameters,
    channels: usize,
    bytes_per_sample: u8,
    /// `true` when the input format is [`SampleFormat::F32`] (the
    /// 4-byte samples are IEEE-754 bit patterns, encoded through the
    /// `0x08` float deconstruction).
    float: bool,
    mode: EncodeMode,
    /// Running absolute frame offset across packets (the wiki
    /// `block_index` header word of the next block).
    next_index: u32,
    queue: VecDeque<Packet>,
    eof: bool,
}

impl WavPackEncoder {
    fn new(params: &CodecParameters) -> CoreResult<Self> {
        let format = params.sample_format.unwrap_or(SampleFormat::S16);
        let bytes_per_sample = width_for_format(format)?;
        let float = format == SampleFormat::F32;
        let channels = usize::from(params.channels.unwrap_or(2));
        if channels == 0 || channels > crate::block::MAX_MULTICHANNEL_CHANNELS {
            return Err(CoreError::invalid(format!(
                "wavpack: unsupported channel count {channels}"
            )));
        }
        let o: WavPackEncoderOptions = parse_options(&params.options)?;
        if o.correction {
            return Err(CoreError::unsupported(
                "wavpack: the .wvc correction twin is a second stream; the registry packet \
                 contract is single-stream — use the crate-level hybrid pair APIs \
                 (HybridOptions::correction) instead",
            ));
        }
        let mode = if o.mode == "hybrid" {
            // Round 447: integer multichannel hybrid encodes through
            // the per-member-chain carries; the float / int32 hybrid
            // deconstructions stay mono/stereo-only (matching the
            // crate-level origination surface).
            if channels > 2 && (float || bytes_per_sample == 4) {
                return Err(CoreError::unsupported(
                    "wavpack: hybrid multichannel supports 8/16/24-bit integer input only",
                ));
            }
            let mut hopts = HybridOptions::from_bits_per_sample(f64::from(o.bits_per_sample));
            hopts.correction = false;
            hopts.joint = o.joint;
            hopts.shaping = HybridShaping::from_weight(f64::from(o.shaping));
            EncodeMode::Hybrid {
                opts: hopts,
                carry: None,
                mc_carries: None,
            }
        } else {
            // Hybrid-only knobs set without hybrid mode are a caller
            // mistake — surface it instead of silently ignoring them.
            for key in ["bits_per_sample", "shaping", "joint"] {
                if params.options.get(key).is_some() {
                    return Err(CoreError::invalid(format!(
                        "wavpack: option '{key}' requires mode=hybrid"
                    )));
                }
            }
            EncodeMode::Lossless
        };
        let mut out_params = CodecParameters::audio(CodecId::new(CODEC_ID));
        out_params.sample_rate = params.sample_rate;
        out_params.channels = Some(channels as u16);
        out_params.sample_format = Some(format);
        out_params.channel_layout = params.channel_layout;
        Ok(Self {
            params: out_params,
            channels,
            bytes_per_sample,
            float,
            mode,
            next_index: 0,
            queue: VecDeque::new(),
            eof: false,
        })
    }

    /// Time base for emitted packets: `1/sample_rate` when known,
    /// `1/1` otherwise.
    fn time_base(&self) -> TimeBase {
        match self.params.sample_rate {
            Some(rate) if rate > 0 => TimeBase::new(1, i64::from(rate)),
            _ => TimeBase::new(1, 1),
        }
    }
}

impl Encoder for WavPackEncoder {
    fn codec_id(&self) -> &CodecId {
        &self.params.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.params
    }

    fn send_frame(&mut self, frame: &Frame) -> CoreResult<()> {
        if self.eof {
            return Err(CoreError::invalid(
                "wavpack: send_frame after flush (encoder is drained)",
            ));
        }
        let Frame::Audio(audio) = frame else {
            return Err(CoreError::unsupported(
                "wavpack: only audio frames can be encoded",
            ));
        };
        let [plane] = audio.data.as_slice() else {
            return Err(CoreError::unsupported(
                "wavpack: interleaved (single-plane) input required",
            ));
        };
        let pcm = widen_bytes(plane, self.bytes_per_sample);
        if pcm.len() % self.channels != 0 {
            return Err(CoreError::invalid(format!(
                "wavpack: frame carries {} samples, not whole {}-channel frames",
                pcm.len(),
                self.channels
            )));
        }
        if pcm.is_empty() {
            return Ok(());
        }
        let frames = pcm.len() / self.channels;

        let mut data: Vec<u8> = Vec::new();
        match self.channels {
            1 | 2 => {
                // Chunk into blocks with the running index; the full
                // mode-search block encoders handle joint / decorr /
                // shift decisions per block, and hybrid mode carries
                // its level / shaping state across blocks and packets.
                let mono = self.channels == 1;
                let chunk_values = DEFAULT_BLOCK_SAMPLES * self.channels;
                let mut index = self.next_index;
                for window in pcm.chunks(chunk_values) {
                    let block = match &mut self.mode {
                        EncodeMode::Lossless => encode_lossless_block(
                            window,
                            mono,
                            self.float,
                            self.bytes_per_sample,
                            index,
                        ),
                        EncodeMode::Hybrid { opts, carry, .. } => {
                            let carry = carry.get_or_insert_with(|| HybridCarry::new(opts, mono));
                            encode_hybrid_lossy_block(
                                window,
                                mono,
                                self.float,
                                self.bytes_per_sample,
                                opts,
                                carry,
                                index,
                            )
                        }
                    }
                    .map_err(invalid)?;
                    data.extend_from_slice(&block);
                    index = index
                        .checked_add((window.len() / self.channels) as u32)
                        .ok_or_else(|| {
                            CoreError::invalid("wavpack: stream exceeds u32 sample indexing")
                        })?;
                }
            }
            _ if matches!(self.mode, EncodeMode::Hybrid { .. }) => {
                // Round 447: integer multichannel hybrid — the shared
                // per-set member loop with per-chain carries persisted
                // across packets (opts.correction is false, so the
                // wvc sink stays empty).
                let channels = self.channels;
                let bytes_per_sample = self.bytes_per_sample;
                let next_index = self.next_index;
                let EncodeMode::Hybrid {
                    opts, mc_carries, ..
                } = &mut self.mode
                else {
                    unreachable!("guarded by the match arm");
                };
                let carries =
                    mc_carries.get_or_insert_with(|| new_multichannel_carries(opts, channels));
                let mut wv = Vec::new();
                let mut wvc_sink = Vec::new();
                encode_multichannel_hybrid_members(
                    &pcm,
                    channels,
                    DEFAULT_BLOCK_SAMPLES,
                    bytes_per_sample,
                    opts,
                    carries,
                    next_index,
                    TOTAL_SAMPLES_UNKNOWN,
                    &mut wv,
                    &mut wvc_sink,
                )
                .map_err(invalid)?;
                data = wv;
            }
            _ => {
                // Round 447: wider layouts get real per-member
                // compression — stereo-pair members through the same
                // mode search the mono/stereo paths run — and the F32 /
                // S32 formats route through the same 0x08 / 0x09
                // deconstructions the narrow paths use.
                data = if self.float {
                    let f = as_f32_bits(&pcm);
                    encode_multichannel_stream_float_at(
                        &f,
                        self.channels,
                        DEFAULT_BLOCK_SAMPLES,
                        self.next_index,
                        TOTAL_SAMPLES_UNKNOWN,
                        DecorrProfile::Normal,
                    )
                } else if self.bytes_per_sample == 4 {
                    encode_multichannel_stream_int32_at(
                        &pcm,
                        self.channels,
                        DEFAULT_BLOCK_SAMPLES,
                        self.next_index,
                        TOTAL_SAMPLES_UNKNOWN,
                        DecorrProfile::Normal,
                    )
                } else {
                    encode_multichannel_stream_best_at(
                        &pcm,
                        self.channels,
                        DEFAULT_BLOCK_SAMPLES,
                        self.bytes_per_sample,
                        self.next_index,
                        TOTAL_SAMPLES_UNKNOWN,
                        DecorrProfile::Normal,
                    )
                }
                .map_err(invalid)?;
            }
        }

        // Stamp the caller-declared sample rate into every block
        // header (standard-rate index, or the sentinel 15 + a 0x27
        // sub-block on the stream's first block for non-standard
        // rates) so the emitted chain self-describes its rate
        // (round 405).
        if let Some(rate) = self.params.sample_rate {
            if rate > 0 {
                data = crate::encode::set_stream_sample_rate(&data, rate).map_err(invalid)?;
            }
        }

        let mut packet = Packet::new(0, self.time_base(), data);
        packet.pts = audio.pts;
        packet.dts = audio.pts;
        packet.duration = Some(frames as i64);
        packet.flags.keyframe = true;
        self.queue.push_back(packet);

        self.next_index = self
            .next_index
            .checked_add(frames as u32)
            .ok_or_else(|| CoreError::invalid("wavpack: stream exceeds u32 sample indexing"))?;
        Ok(())
    }

    fn receive_packet(&mut self) -> CoreResult<Packet> {
        match self.queue.pop_front() {
            Some(p) => Ok(p),
            None if self.eof => Err(CoreError::Eof),
            None => Err(CoreError::NeedMore),
        }
    }

    fn flush(&mut self) -> CoreResult<()> {
        // Every send_frame is fully encoded eagerly; flushing only
        // latches EOS for receive_packet.
        self.eof = true;
        Ok(())
    }
}

/// Reinterpret container-scaled 4-byte samples as their IEEE-754 bit
/// patterns (the `F32` input path).
fn as_f32_bits(pcm: &[i32]) -> Vec<f32> {
    pcm.iter().map(|&v| f32::from_bits(v as u32)).collect()
}

/// One lossless mode-searched block for the registry chunk loop:
/// plain widths through the historical mode search, `S32` through the
/// `0x09` int32 deconstruction, `F32` through the `0x08` float
/// deconstruction.
fn encode_lossless_block(
    window: &[i32],
    mono: bool,
    float: bool,
    bytes_per_sample: u8,
    index: u32,
) -> crate::Result<Vec<u8>> {
    if float {
        let f = as_f32_bits(window);
        return if mono {
            encode_block_mono_float_best(&f, DecorrProfile::Normal, index, TOTAL_SAMPLES_UNKNOWN)
        } else {
            encode_block_stereo_float_best(&f, DecorrProfile::Normal, index, TOTAL_SAMPLES_UNKNOWN)
        };
    }
    if bytes_per_sample == 4 {
        return if mono {
            encode_block_mono_int32_best(
                window,
                DecorrProfile::Normal,
                index,
                TOTAL_SAMPLES_UNKNOWN,
            )
        } else {
            encode_block_stereo_int32_best(
                window,
                DecorrProfile::Normal,
                index,
                TOTAL_SAMPLES_UNKNOWN,
            )
        };
    }
    if mono {
        encode_block_mono_best(
            window,
            DecorrProfile::Normal,
            bytes_per_sample,
            index,
            TOTAL_SAMPLES_UNKNOWN,
        )
    } else {
        encode_block_stereo_best(
            window,
            DecorrProfile::Normal,
            bytes_per_sample,
            index,
            TOTAL_SAMPLES_UNKNOWN,
        )
    }
}

/// One hybrid lossy block for the registry chunk loop, with the
/// cross-block `0x06` level / `0x07` shaping carry (`opts.correction`
/// is false here, so no `.wvc` twin is produced).
fn encode_hybrid_lossy_block(
    window: &[i32],
    mono: bool,
    float: bool,
    bytes_per_sample: u8,
    opts: &HybridOptions,
    carry: &mut HybridCarry,
    index: u32,
) -> crate::Result<Vec<u8>> {
    if float {
        let f = as_f32_bits(window);
        let (wv, _) =
            encode_hybrid_block_float(&f, mono, opts, carry, index, TOTAL_SAMPLES_UNKNOWN)?;
        return Ok(wv);
    }
    if bytes_per_sample == 4 {
        let (wv, _) =
            encode_hybrid_block_int32(window, mono, opts, carry, index, TOTAL_SAMPLES_UNKNOWN)?;
        return Ok(wv);
    }
    let level = carry.level_words(window, mono, opts)?;
    let shaping = carry.payload_for_block(window.len() / if mono { 1 } else { 2 });
    let (wv, _, sl, shape) = encode_hybrid_block_ints(
        window,
        mono,
        bytes_per_sample,
        opts,
        level,
        shaping.as_deref(),
        None,
        index,
        TOTAL_SAMPLES_UNKNOWN,
    )?;
    carry.absorb(sl, &shape);
    Ok(wv)
}

/// Direct encoder factory (the historical `encoder::make_encoder`
/// endpoint; also the factory [`register`] installs).
pub fn make_encoder(params: &CodecParameters) -> CoreResult<Box<dyn Encoder>> {
    Ok(Box::new(WavPackEncoder::new(params)?))
}

// ───────────────────────── registration ─────────────────────────

/// Install this crate's codec registration into a
/// [`RuntimeContext`]. Called by the umbrella's `register_all`
/// dispatch through the [`oxideav_core::register!`]-generated entry
/// point, or directly by standalone consumers.
pub fn register(ctx: &mut RuntimeContext) {
    let caps = CodecCapabilities {
        decode: true,
        encode: true,
        lossless: true,
        ..CodecCapabilities::audio("wavpack_sw")
    };
    ctx.codecs.register(
        CodecInfo::new(CodecId::new(CODEC_ID))
            .capabilities(caps)
            .decoder(make_decoder)
            .encoder(make_encoder)
            // The staged wiki snapshot documents the (unofficial)
            // FourCC `WVPK`.
            .tag(CodecTag::fourcc(b"WVPK")),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::AudioFrame;

    fn audio_params(channels: u16, format: SampleFormat, rate: Option<u32>) -> CodecParameters {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID));
        p.channels = Some(channels);
        p.sample_format = Some(format);
        p.sample_rate = rate;
        p
    }

    fn frame_from_pcm(pcm: &[i32], width: u8, pts: Option<i64>) -> Frame {
        Frame::Audio(AudioFrame {
            samples: 0, // encoder derives its own frame count
            pts,
            data: vec![pack_bytes(pcm, width)],
        })
    }

    #[test]
    fn pack_widen_round_trip_all_widths() {
        for (width, samples) in [
            (1u8, vec![0i32, 1, -1, 127, -128]),
            (2, vec![0, 1, -1, 32767, -32768, 12345]),
            (3, vec![0, 1, -1, (1 << 23) - 1, -(1 << 23), -777777]),
            (4, vec![0, 1, -1, i32::MAX, i32::MIN, 0x1234_5678]),
        ] {
            let bytes = pack_bytes(&samples, width);
            assert_eq!(bytes.len(), samples.len() * usize::from(width));
            assert_eq!(widen_bytes(&bytes, width), samples, "width={width}");
        }
    }

    #[test]
    fn decoder_round_trips_stereo_packet() {
        let pcm: Vec<i32> = (0..400).map(|i| (i * 31) % 1000 - 500).collect();
        let wv = crate::encode::encode_stream_stereo(&pcm, 100, 2).unwrap();
        let params = audio_params(2, SampleFormat::S16, Some(44_100));
        let mut dec = make_decoder(&params).unwrap();
        assert_eq!(dec.codec_id().as_str(), "wavpack");

        // No packet yet → NeedMore.
        assert!(matches!(dec.receive_frame(), Err(CoreError::NeedMore)));

        let mut packet = Packet::new(0, TimeBase::new(1, 44_100), wv);
        packet.pts = Some(7);
        dec.send_packet(&packet).unwrap();
        let Frame::Audio(frame) = dec.receive_frame().unwrap() else {
            panic!("audio frame expected");
        };
        assert_eq!(frame.samples, 200);
        assert_eq!(frame.pts, Some(7));
        assert_eq!(frame.data.len(), 1);
        assert_eq!(widen_bytes(&frame.data[0], 2), pcm);

        // Drained again → NeedMore; after flush → Eof.
        assert!(matches!(dec.receive_frame(), Err(CoreError::NeedMore)));
        dec.flush().unwrap();
        assert!(matches!(dec.receive_frame(), Err(CoreError::Eof)));
        // Reset re-arms the decoder.
        dec.reset().unwrap();
        assert!(matches!(dec.receive_frame(), Err(CoreError::NeedMore)));
    }

    #[test]
    fn decoder_handles_multichannel_and_metadata_only_packets() {
        let channels = 4usize;
        let pcm: Vec<i32> = (0..channels * 60).map(|i| i as i32 % 256 - 128).collect();
        let wv = crate::encode::encode_multichannel_stream(&pcm, channels, 25, 2).unwrap();
        let mut dec = make_decoder(&audio_params(4, SampleFormat::S16, None)).unwrap();
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 1), wv))
            .unwrap();
        let Frame::Audio(frame) = dec.receive_frame().unwrap() else {
            panic!("audio frame expected");
        };
        assert_eq!(frame.samples, 60);
        assert_eq!(widen_bytes(&frame.data[0], 2), pcm);

        // A metadata-only packet contributes nothing (and is not an error).
        let mut header_only = Vec::new();
        header_only.extend_from_slice(crate::block_header::MAGIC);
        header_only.extend_from_slice(&24u32.to_le_bytes());
        header_only.extend_from_slice(&0x0410u16.to_le_bytes());
        header_only.extend_from_slice(&[0, 0]);
        header_only.extend_from_slice(&u32::MAX.to_le_bytes());
        header_only.extend_from_slice(&0u32.to_le_bytes());
        header_only.extend_from_slice(&0u32.to_le_bytes()); // block_samples = 0
        header_only.extend_from_slice(&(0b11u32 << 11).to_le_bytes());
        header_only.extend_from_slice(&0u32.to_le_bytes());
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 1), header_only))
            .unwrap();
        assert!(matches!(dec.receive_frame(), Err(CoreError::NeedMore)));
    }

    #[test]
    fn decoder_surfaces_malformed_packets_as_invalid() {
        let mut dec = make_decoder(&audio_params(2, SampleFormat::S16, None)).unwrap();
        let err = dec
            .send_packet(&Packet::new(
                0,
                TimeBase::new(1, 1),
                b"not wavpack".to_vec(),
            ))
            .unwrap_err();
        assert!(matches!(err, CoreError::InvalidData(_)), "{err:?}");
    }

    #[test]
    fn encoder_stereo_packets_form_contiguous_seekable_stream() {
        let params = audio_params(2, SampleFormat::S16, Some(48_000));
        let mut enc = make_encoder(&params).unwrap();
        assert_eq!(enc.codec_id().as_str(), "wavpack");
        assert_eq!(enc.output_params().channels, Some(2));
        assert_eq!(enc.output_params().sample_format, Some(SampleFormat::S16));

        // Two frames of interleaved stereo (odd sizes to cross block
        // boundaries when chunked).
        let pcm: Vec<i32> = (0..2 * 700).map(|i| (i * 13) % 900 - 450).collect();
        let split = 2 * 300;
        enc.send_frame(&frame_from_pcm(&pcm[..split], 2, Some(0)))
            .unwrap();
        enc.send_frame(&frame_from_pcm(&pcm[split..], 2, Some(300)))
            .unwrap();
        enc.flush().unwrap();

        let mut wv: Vec<u8> = Vec::new();
        let p1 = enc.receive_packet().unwrap();
        assert_eq!(p1.pts, Some(0));
        assert_eq!(p1.duration, Some(300));
        assert!(p1.flags.keyframe);
        wv.extend_from_slice(&p1.data);
        let p2 = enc.receive_packet().unwrap();
        assert_eq!(p2.pts, Some(300));
        assert_eq!(p2.duration, Some(400));
        wv.extend_from_slice(&p2.data);
        assert!(matches!(enc.receive_packet(), Err(CoreError::Eof)));

        // The concatenated packets are one lossless, seekable stream.
        assert_eq!(crate::block::decode_stream(&wv).unwrap(), pcm);
        let index = crate::seek::StreamIndex::scan(&wv).unwrap();
        assert!(index.is_seekable(), "running block_index across packets");
        assert_eq!(index.frame_count(), 700);
        assert_eq!(index.channels(), 2);
    }

    #[test]
    fn encoder_decoder_loop_is_lossless_per_packet() {
        let params = audio_params(1, SampleFormat::S24, None);
        let mut enc = make_encoder(&params).unwrap();
        let mut dec = make_decoder(&params).unwrap();
        // 24-bit mono content exercising the wide container path.
        let pcm: Vec<i32> = (0..500)
            .map(|i| ((i * 40_503) % (1 << 24)) - (1 << 23))
            .collect();
        enc.send_frame(&frame_from_pcm(&pcm, 3, None)).unwrap();
        let packet = enc.receive_packet().unwrap();
        dec.send_packet(&packet).unwrap();
        let Frame::Audio(frame) = dec.receive_frame().unwrap() else {
            panic!("audio frame expected");
        };
        assert_eq!(widen_bytes(&frame.data[0], 3), pcm);
    }

    #[test]
    fn encoder_multichannel_path_round_trips() {
        let params = audio_params(6, SampleFormat::S16, Some(48_000));
        let mut enc = make_encoder(&params).unwrap();
        let pcm: Vec<i32> = (0..6 * 120).map(|i| (i * 7) % 400 - 200).collect();
        enc.send_frame(&frame_from_pcm(&pcm, 2, None)).unwrap();
        let packet = enc.receive_packet().unwrap();
        let decoded = crate::block::decode_multichannel_stream(&packet.data).unwrap();
        assert_eq!(decoded.channels, 6);
        assert_eq!(decoded.samples, pcm);
    }

    #[test]
    fn encoder_multichannel_float_and_int32_round_trip() {
        // Round 447: F32 / S32 multichannel encode through the paired
        // 0x08 / 0x09 member paths (formerly a typed refusal).
        let params = audio_params(4, SampleFormat::F32, Some(48_000));
        let mut enc = make_encoder(&params).unwrap();
        let pcm: Vec<f32> = (0..4 * 200)
            .map(|i| ((i as f32) * 0.017).sin() * 0.9)
            .collect();
        let bytes: Vec<u8> = pcm.iter().flat_map(|s| s.to_bits().to_le_bytes()).collect();
        enc.send_frame(&Frame::Audio(AudioFrame {
            samples: 0,
            pts: None,
            data: vec![bytes.clone()],
        }))
        .unwrap();
        let packet = enc.receive_packet().unwrap();
        let decoded = crate::block::decode_multichannel_stream_f32(&packet.data).unwrap();
        assert_eq!(decoded.channels, 4);
        let got: Vec<u32> = decoded.samples.iter().map(|s| s.to_bits()).collect();
        let want: Vec<u32> = pcm.iter().map(|s| s.to_bits()).collect();
        assert_eq!(got, want, "multichannel float bit patterns");
        // And through the registry decoder the plane bytes come back
        // verbatim.
        let mut dec = make_decoder(&params).unwrap();
        dec.send_packet(&packet).unwrap();
        let Frame::Audio(frame) = dec.receive_frame().unwrap() else {
            panic!("audio frame expected");
        };
        assert_eq!(frame.data[0], bytes);

        let params = audio_params(6, SampleFormat::S32, Some(48_000));
        let mut enc = make_encoder(&params).unwrap();
        let pcm: Vec<i32> = (0..6i32 * 150)
            .map(|i| i.wrapping_mul(0x00fe_dcba))
            .collect();
        enc.send_frame(&frame_from_pcm(&pcm, 4, None)).unwrap();
        let packet = enc.receive_packet().unwrap();
        let decoded = crate::block::decode_multichannel_stream(&packet.data).unwrap();
        assert_eq!(decoded.channels, 6);
        assert_eq!(decoded.samples, pcm, "multichannel int32 full range");
    }

    #[test]
    fn encoder_refusals() {
        // Interleaved float input is supported since round 420.
        let mut p = audio_params(2, SampleFormat::F32, None);
        assert!(make_encoder(&p).is_ok());
        // Planar input format.
        p.sample_format = Some(SampleFormat::S16P);
        assert!(make_encoder(&p).is_err());
        // Zero channels.
        let p = audio_params(0, SampleFormat::S16, None);
        assert!(make_encoder(&p).is_err());
        // Ragged frame payload (not whole frames).
        let mut enc = make_encoder(&audio_params(2, SampleFormat::S16, None)).unwrap();
        let err = enc
            .send_frame(&frame_from_pcm(&[1, 2, 3], 2, None))
            .unwrap_err();
        assert!(matches!(err, CoreError::InvalidData(_)), "{err:?}");
        // Non-audio frame.
        let err = enc
            .send_frame(&Frame::Video(oxideav_core::VideoFrame {
                pts: None,
                planes: Vec::new(),
            }))
            .unwrap_err();
        assert!(matches!(err, CoreError::Unsupported(_)), "{err:?}");
    }

    // ---- round 420: options, hybrid mode, float / int32 wiring ------

    #[test]
    fn encoder_options_reject_misuse() {
        // correction twin: two streams, refused with a pointer at the
        // crate pair APIs.
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("mode", "hybrid");
        p.options.insert("correction", "true");
        let Err(err) = make_encoder(&p) else {
            panic!("correction=true must be refused");
        };
        assert!(matches!(err, CoreError::Unsupported(_)), "{err:?}");
        // Hybrid-only knobs without mode=hybrid.
        for key in ["bits_per_sample", "shaping", "joint"] {
            let mut p = audio_params(2, SampleFormat::S16, None);
            p.options
                .insert(key, if key == "joint" { "true" } else { "3.5" });
            let Err(err) = make_encoder(&p) else {
                panic!("{key} without mode=hybrid must be refused");
            };
            assert!(matches!(err, CoreError::InvalidData(_)), "{key}: {err:?}");
        }
        // Unknown key / malformed enum value.
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("bogus", "1");
        assert!(make_encoder(&p).is_err());
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("mode", "psychic");
        assert!(make_encoder(&p).is_err());
        // Integer hybrid multichannel is an encode shape since round
        // 447; the float / int32 hybrid deconstructions stay
        // mono/stereo-only.
        let mut p = audio_params(6, SampleFormat::S16, None);
        p.options.insert("mode", "hybrid");
        assert!(make_encoder(&p).is_ok());
        for fmt in [SampleFormat::F32, SampleFormat::S32] {
            let mut p = audio_params(6, fmt, None);
            p.options.insert("mode", "hybrid");
            let Err(err) = make_encoder(&p) else {
                panic!("{fmt:?} hybrid multichannel must be refused");
            };
            assert!(matches!(err, CoreError::Unsupported(_)), "{err:?}");
        }
        // Planar float stays refused.
        let p = audio_params(2, SampleFormat::F32P, None);
        assert!(make_encoder(&p).is_err());
    }

    #[test]
    fn hybrid_multichannel_packets_carry_state_and_decode() {
        // Round 447: 4-channel hybrid through the registry — every
        // packet decodes as a member-set chain, the concatenated
        // packets decode as one contiguous chain, and the second
        // packet's 0x06 level words differ from the first (the
        // cross-packet per-member carry, not a re-seed).
        let mut params = audio_params(4, SampleFormat::S16, None);
        params.options.insert("mode", "hybrid");
        params.options.insert("bits_per_sample", "4.0");
        params.options.insert("shaping", "-0.5");
        let mut enc = make_encoder(&params).unwrap();
        let pcm: Vec<i32> = (0..4 * 900)
            .map(|i| {
                let t = (i / 4) as f64 * 0.03;
                ((t.sin() * 9000.0) as i32 + ((i * 37) % 200 - 100)).clamp(-32768, 32767)
            })
            .collect();
        let half = pcm.len() / 2;
        let mut chain = Vec::new();
        let mut level_words = Vec::new();
        for part in [&pcm[..half], &pcm[half..]] {
            enc.send_frame(&frame_from_pcm(part, 2, None)).unwrap();
            let packet = enc.receive_packet().unwrap();
            let decoded = crate::block::decode_multichannel_stream(&packet.data).unwrap();
            assert_eq!(decoded.channels, 4);
            assert_eq!(decoded.samples.len(), part.len());
            let (blk, _) = crate::parse_block(&packet.data).unwrap();
            level_words.push(
                crate::metadata::find_hybrid_profile(&blk.sub_blocks)
                    .expect("hybrid profile on the set's first member")
                    .payload
                    .to_vec(),
            );
            chain.extend_from_slice(&packet.data);
        }
        assert_ne!(
            level_words[0], level_words[1],
            "level words must carry across packets"
        );
        let whole = crate::block::decode_multichannel_stream(&chain).unwrap();
        assert_eq!(whole.channels, 4);
        assert_eq!(whole.samples.len(), pcm.len());
        assert!(whole.samples.iter().all(|&s| (-32768..=32767).contains(&s)));
        let index = crate::seek::StreamIndex::scan(&chain).unwrap();
        assert!(index.is_seekable(), "packets form one seekable chain");
    }

    #[test]
    fn hybrid_mode_packets_are_lossy_shaped_and_contiguous() {
        let mut params = audio_params(2, SampleFormat::S16, Some(44_100));
        params.options.insert("mode", "hybrid");
        params.options.insert("bits_per_sample", "4.0");
        params.options.insert("shaping", "0.7");
        let mut enc = make_encoder(&params).unwrap();

        let pcm: Vec<i32> = (0..2 * 2200)
            .map(|i| {
                let t = f64::from(i / 2) * 0.045;
                ((t.sin() * 9000.0) as i32) + (i % 61) - 30
            })
            .collect();
        let split = 2 * 900;
        enc.send_frame(&frame_from_pcm(&pcm[..split], 2, Some(0)))
            .unwrap();
        enc.send_frame(&frame_from_pcm(&pcm[split..], 2, Some(900)))
            .unwrap();
        enc.flush().unwrap();
        let mut wv = Vec::new();
        while let Ok(p) = enc.receive_packet() {
            wv.extend_from_slice(&p.data);
        }

        // The chain decodes standalone (lossy), is seekable, and the
        // noise stays bounded by the b4 target.
        let (lossy, ok) = crate::block::decode_stream_muted(&wv).unwrap();
        assert!(ok, "lossy CRC gate across packets");
        assert_eq!(lossy.len(), pcm.len());
        let max_err = lossy
            .iter()
            .zip(&pcm)
            .map(|(&a, &b)| (i64::from(a) - i64::from(b)).abs())
            .max()
            .unwrap();
        assert!(max_err > 0, "hybrid mode is genuinely lossy");
        assert!(max_err < 4096, "noise bounded ({max_err})");
        let index = crate::seek::StreamIndex::scan(&wv).unwrap();
        assert!(index.is_seekable());
        assert_eq!(index.frame_count(), 2200);
        // Every block flags hybrid + both shape bits; the lossy chain
        // carries no 0x07 (it rides the wvc twin, which this mode does
        // not produce).
        let mut rest: &[u8] = &wv;
        while !rest.is_empty() {
            let (blk, next) = crate::parse_block(rest).unwrap();
            assert!(blk.flags().hybrid);
            assert_ne!(blk.flags().raw & crate::hybrid::HYBRID_SHAPE_FLAG, 0);
            assert_ne!(blk.flags().raw & crate::hybrid::NEW_SHAPING_FLAG, 0);
            assert!(blk.find_noise_shaping_profile_sub_block().is_none());
            assert!(!blk.has_packed_correction_data());
            rest = next;
        }
        // The registry decoder consumes its own hybrid packets.
        let mut dec = make_decoder(&params).unwrap();
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 44_100), wv))
            .unwrap();
        let Frame::Audio(frame) = dec.receive_frame().unwrap() else {
            panic!("audio frame expected");
        };
        assert_eq!(widen_bytes(&frame.data[0], 2), lossy);
    }

    #[test]
    fn float_input_round_trips_bit_patterns() {
        let params = audio_params(2, SampleFormat::F32, Some(48_000));
        let mut enc = make_encoder(&params).unwrap();
        let pcm: Vec<f32> = (0..2 * 1500)
            .map(|i| {
                if i % 37 == 0 {
                    0.0
                } else {
                    ((i as f32) * 0.021).sin() * 0.8
                }
            })
            .collect();
        let bytes: Vec<u8> = pcm.iter().flat_map(|s| s.to_bits().to_le_bytes()).collect();
        enc.send_frame(&Frame::Audio(AudioFrame {
            samples: 0,
            pts: None,
            data: vec![bytes.clone()],
        }))
        .unwrap();
        let packet = enc.receive_packet().unwrap();
        let decoded = crate::block::decode_stream_f32(&packet.data).unwrap();
        let got: Vec<u32> = decoded.iter().map(|s| s.to_bits()).collect();
        let want: Vec<u32> = pcm.iter().map(|s| s.to_bits()).collect();
        assert_eq!(got, want, "float lossless bit patterns");
        // Through the registry decoder the raw plane bytes come back
        // verbatim (f32 bit patterns in the 4-byte slots).
        let mut dec = make_decoder(&params).unwrap();
        dec.send_packet(&packet).unwrap();
        let Frame::Audio(frame) = dec.receive_frame().unwrap() else {
            panic!("audio frame expected");
        };
        assert_eq!(frame.data[0], bytes);

        // Hybrid float: lossy but decodable and close.
        let mut hp = audio_params(1, SampleFormat::F32, None);
        hp.options.insert("mode", "hybrid");
        hp.options.insert("shaping", "-0.5");
        let mut enc = make_encoder(&hp).unwrap();
        let mono: Vec<f32> = pcm.iter().step_by(2).copied().collect();
        let bytes: Vec<u8> = mono
            .iter()
            .flat_map(|s| s.to_bits().to_le_bytes())
            .collect();
        enc.send_frame(&Frame::Audio(AudioFrame {
            samples: 0,
            pts: None,
            data: vec![bytes],
        }))
        .unwrap();
        let packet = enc.receive_packet().unwrap();
        let lossy = crate::block::decode_stream_f32(&packet.data).unwrap();
        let max_err = lossy
            .iter()
            .zip(&mono)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0f32, f32::max);
        assert!(
            max_err > 0.0 && max_err < 0.2,
            "float noise bounded ({max_err})"
        );
    }

    #[test]
    fn s32_input_uses_the_int32_path_full_range() {
        let params = audio_params(1, SampleFormat::S32, None);
        let mut enc = make_encoder(&params).unwrap();
        let mut x = 0x9e3779b97f4a7c15u64;
        let pcm: Vec<i32> = (0..1400)
            .map(|_| {
                x = x.wrapping_mul(0xd1342543de82ef95).wrapping_add(1);
                (x >> 32) as i32
            })
            .collect();
        enc.send_frame(&frame_from_pcm(&pcm, 4, None)).unwrap();
        let packet = enc.receive_packet().unwrap();
        assert_eq!(
            crate::block::decode_stream(&packet.data).unwrap(),
            pcm,
            "full-range i32 lossless"
        );
        let (blk, _) = crate::parse_block(&packet.data).unwrap();
        assert!(
            blk.find_sub_block(crate::SubBlockId::Int32Info).is_some(),
            "0x09 int32 profile emitted"
        );

        // Hybrid S32: decodable, implied-fill lossy.
        let mut hp = audio_params(1, SampleFormat::S32, None);
        hp.options.insert("mode", "hybrid");
        hp.options.insert("bits_per_sample", "5.0");
        let mut enc = make_encoder(&hp).unwrap();
        enc.send_frame(&frame_from_pcm(&pcm, 4, None)).unwrap();
        let packet = enc.receive_packet().unwrap();
        let (lossy, ok) = crate::block::decode_stream_muted(&packet.data).unwrap();
        assert!(ok);
        assert_eq!(lossy.len(), pcm.len());
    }

    // ---- round 436: per-packet decoded-sample budget ----------------

    /// Synthesise a hostile packet: eight ~44-byte blocks each claiming
    /// the per-block decode ceiling, so the headers demand 8 × 2^26
    /// samples (2 GiB of i32 — over the default 2^28 packet budget)
    /// from ~350 bytes of input.
    fn hostile_amplification_packet() -> Vec<u8> {
        let mut payload = Vec::new();
        // 0x05 entropy info (all-zero mono seed).
        payload.extend_from_slice(&[0x05, 0x03, 0, 0, 0, 0, 0, 0]);
        // 0x0A packed samples (2 garbage bytes).
        payload.extend_from_slice(&[0x0A, 0x01, 0xFF, 0xFF]);
        let mut block = Vec::new();
        block.extend_from_slice(crate::block_header::MAGIC);
        block.extend_from_slice(&((24 + payload.len()) as u32).to_le_bytes());
        block.extend_from_slice(&0x0410u16.to_le_bytes());
        block.extend_from_slice(&[0, 0]);
        block.extend_from_slice(&0u32.to_le_bytes()); // total_samples
        block.extend_from_slice(&0u32.to_le_bytes()); // block_index
        block.extend_from_slice(&crate::MAX_DECODE_SAMPLES_PER_BLOCK.to_le_bytes());
        // Flags: mono + standalone set markers (bits 11..=12).
        block.extend_from_slice(&((0b11u32 << 11) | (1 << 2)).to_le_bytes());
        block.extend_from_slice(&0u32.to_le_bytes()); // crc
        block.extend_from_slice(&payload);
        let mut bytes = Vec::new();
        for _ in 0..8 {
            bytes.extend_from_slice(&block);
        }
        bytes
    }

    #[test]
    fn decoder_refuses_hostile_amplification_packet_before_decoding() {
        let bytes = hostile_amplification_packet();
        let input_len = bytes.len();
        assert!(input_len < 500, "hostile input is tiny ({input_len} bytes)");
        // With a budget below one block's claim the refusal fires on
        // the FIRST block's header — before any decode work — so the
        // surfaced error is the budget refusal, not the truncation the
        // garbage payload would produce if the per-sample loop ran.
        // (The default 2^28 budget takes the same path; it just permits
        // the first four 2^26-sample charges — each individually capped
        // by the per-block ceiling — before refusing, bounding the
        // worst-case per-packet allocation instead of leaving it
        // unbounded.)
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("max_packet_samples", "1000000");
        let mut dec = make_decoder(&p).unwrap();
        let err = dec
            .send_packet(&Packet::new(0, TimeBase::new(1, 1), bytes))
            .unwrap_err();
        assert!(matches!(err, CoreError::InvalidData(_)), "{err:?}");
        assert!(
            format!("{err}").contains("budget"),
            "budget refusal expected, got: {err}"
        );
        // The default budget is the documented constant.
        assert_eq!(
            WavPackDecoderOptions::default().max_packet_samples,
            DEFAULT_PACKET_SAMPLE_BUDGET
        );
    }

    #[test]
    fn decoder_packet_budget_option_is_honored_and_zero_disables() {
        let pcm: Vec<i32> = (0..400).map(|i| (i * 31) % 1000 - 500).collect();
        let wv = crate::encode::encode_stream_stereo(&pcm, 100, 2).unwrap();

        // A budget below the packet's 400 emitted values refuses it.
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("max_packet_samples", "399");
        let mut dec = make_decoder(&p).unwrap();
        let err = dec
            .send_packet(&Packet::new(0, TimeBase::new(1, 1), wv.clone()))
            .unwrap_err();
        assert!(matches!(err, CoreError::InvalidData(_)), "{err:?}");
        assert!(format!("{err}").contains("budget"), "{err}");

        // An exact budget decodes bit-identically.
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("max_packet_samples", "400");
        let mut dec = make_decoder(&p).unwrap();
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 1), wv.clone()))
            .unwrap();
        let Frame::Audio(frame) = dec.receive_frame().unwrap() else {
            panic!("audio frame expected");
        };
        assert_eq!(widen_bytes(&frame.data[0], 2), pcm);

        // `0` disables the budget entirely — even the hostile packet's
        // charge passes (its garbage payload then fails the real
        // decode, which is the pre-round-436 behaviour).
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("max_packet_samples", "0");
        let mut dec = make_decoder(&p).unwrap();
        let err = dec
            .send_packet(&Packet::new(
                0,
                TimeBase::new(1, 1),
                hostile_amplification_packet(),
            ))
            .unwrap_err();
        assert!(
            !format!("{err}").contains("budget"),
            "budget disabled, expected a decode error instead: {err}"
        );

        // A malformed value and an unknown key are refused; the
        // encoder-owned keys are tolerated (a decoder is commonly built
        // from the encoder's parameter set).
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("max_packet_samples", "many");
        assert!(make_decoder(&p).is_err());
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("bogus_key", "1");
        assert!(make_decoder(&p).is_err());
        let mut p = audio_params(2, SampleFormat::S16, None);
        p.options.insert("mode", "hybrid");
        p.options.insert("shaping", "0.5");
        assert!(make_decoder(&p).is_ok());
    }

    #[test]
    fn register_installs_decoder_and_encoder_factories() {
        let mut ctx = RuntimeContext::new();
        register(&mut ctx);
        assert!(ctx.codecs.has_encoder(&CodecId::new(CODEC_ID)));
        let params = audio_params(2, SampleFormat::S16, Some(44_100));
        let mut dec = ctx.codecs.first_decoder(&params).expect("registry decoder");
        let mut enc = ctx.codecs.first_encoder(&params).expect("registry encoder");

        // End-to-end through the registry-built pair.
        let pcm: Vec<i32> = (0..2 * 200).map(|i| (i * 3) % 100 - 50).collect();
        enc.send_frame(&frame_from_pcm(&pcm, 2, None)).unwrap();
        let packet = enc.receive_packet().unwrap();
        dec.send_packet(&packet).unwrap();
        let Frame::Audio(frame) = dec.receive_frame().unwrap() else {
            panic!("audio frame expected");
        };
        assert_eq!(widen_bytes(&frame.data[0], 2), pcm);
    }
}
