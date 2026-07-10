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
//! rescaling), so the PCM bytes are lossless round-trip material. The
//! encoder accepts those same four interleaved formats (from
//! `CodecParameters::sample_format`, default [`SampleFormat::S16`])
//! and refuses float / planar layouts as unsupported.
//!
//! Hybrid (lossy), float and int32 WavPack streams stay refused by the
//! underlying decode layer (documented docs gaps — see the crate
//! README); the registry decoder surfaces those as invalid-data
//! errors.

use std::collections::VecDeque;

use oxideav_core::{
    CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecTag, Decoder, Encoder,
    Error as CoreError, Frame, Packet, Result as CoreResult, RuntimeContext, SampleFormat,
    TimeBase,
};

use crate::block_header::TOTAL_SAMPLES_UNKNOWN;
use crate::encode::{
    encode_block_mono_best, encode_block_stereo_best, encode_multichannel_stream_at, DecorrProfile,
    DEFAULT_BLOCK_SAMPLES,
};

/// The registry identifier this crate registers under.
pub const CODEC_ID: &str = "wavpack";

/// Map a crate-level error into the framework's invalid-data error,
/// keeping the typed message.
fn invalid(e: crate::Error) -> CoreError {
    CoreError::invalid(format!("wavpack: {e}"))
}

// ───────────────────────── decoder ─────────────────────────

/// Framework [`Decoder`] over [`crate::decode_multichannel_stream`].
///
/// Each packet must carry complete `wvpk` blocks (whole member sets).
/// One [`oxideav_core::AudioFrame`] is produced per packet, with the
/// per-frame channel count taken from the decoded member-set grouping
/// and the sample bytes packed at the stream's container width.
#[derive(Debug)]
pub struct WavPackDecoder {
    codec_id: CodecId,
    queue: VecDeque<oxideav_core::AudioFrame>,
    eof: bool,
}

impl WavPackDecoder {
    fn new() -> Self {
        Self {
            codec_id: CodecId::new(CODEC_ID),
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
/// one" field, plus one).
fn width_for_format(format: SampleFormat) -> CoreResult<u8> {
    match format {
        SampleFormat::S8 => Ok(1),
        SampleFormat::S16 => Ok(2),
        SampleFormat::S24 => Ok(3),
        SampleFormat::S32 => Ok(4),
        other => Err(CoreError::unsupported(format!(
            "wavpack: sample format {other:?} not supported (signed interleaved S8/S16/S24/S32 only)"
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
        let stream = crate::block::decode_multichannel_stream(&packet.data).map_err(invalid)?;
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
/// parameter agreement is required — the parameters are accepted for
/// signature compatibility and future options.
pub fn make_decoder(_params: &CodecParameters) -> CoreResult<Box<dyn Decoder>> {
    Ok(Box::new(WavPackDecoder::new()))
}

// ───────────────────────── encoder ─────────────────────────

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
        let channels = usize::from(params.channels.unwrap_or(2));
        if channels == 0 || channels > crate::block::MAX_MULTICHANNEL_CHANNELS {
            return Err(CoreError::invalid(format!(
                "wavpack: unsupported channel count {channels}"
            )));
        }
        let mut out_params = CodecParameters::audio(CodecId::new(CODEC_ID));
        out_params.sample_rate = params.sample_rate;
        out_params.channels = Some(channels as u16);
        out_params.sample_format = Some(format);
        out_params.channel_layout = params.channel_layout;
        Ok(Self {
            params: out_params,
            channels,
            bytes_per_sample,
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
                // shift decisions per block.
                let chunk_values = DEFAULT_BLOCK_SAMPLES * self.channels;
                let mut index = self.next_index;
                for window in pcm.chunks(chunk_values) {
                    let block = if self.channels == 1 {
                        encode_block_mono_best(
                            window,
                            DecorrProfile::Normal,
                            self.bytes_per_sample,
                            index,
                            TOTAL_SAMPLES_UNKNOWN,
                        )
                    } else {
                        encode_block_stereo_best(
                            window,
                            DecorrProfile::Normal,
                            self.bytes_per_sample,
                            index,
                            TOTAL_SAMPLES_UNKNOWN,
                        )
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
            _ => {
                data = encode_multichannel_stream_at(
                    &pcm,
                    self.channels,
                    DEFAULT_BLOCK_SAMPLES,
                    self.bytes_per_sample,
                    self.next_index,
                    TOTAL_SAMPLES_UNKNOWN,
                )
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
    fn encoder_refusals() {
        // Float input format.
        let mut p = audio_params(2, SampleFormat::F32, None);
        assert!(make_encoder(&p).is_err());
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
