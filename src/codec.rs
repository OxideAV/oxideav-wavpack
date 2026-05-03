//! Codec registration glue. Round 1 ships a placeholder decoder
//! constructor — the heavy lifting is in [`crate::decoder`] +
//! [`crate::container`] which are easy to drive from tests directly.
//!
//! The full streaming `Decoder` trait wiring is deferred to the round
//! that integrates with the WavPack demuxer; for round 1 the
//! integration test reaches into `container::parse_file` +
//! `container::decode_frame` to reconstruct PCM.

use oxideav_core::{
    AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecRegistry, Decoder,
    Error, Frame, Packet, Result, SampleFormat,
};

pub fn register(reg: &mut CodecRegistry) {
    let caps = CodecCapabilities::audio("wavpack_sw")
        .with_lossless(true)
        .with_intra_only(true)
        .with_max_channels(8)
        .with_max_sample_rate(192_000);
    reg.register(
        CodecInfo::new(CodecId::new(super::CODEC_ID_STR))
            .capabilities(caps)
            .decoder(make_decoder),
    );
}

fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(WavPackDecoder {
        codec_id: params.codec_id.clone(),
        pending: None,
    }))
}

/// Per-packet-frame WavPack decoder. The packet is expected to contain
/// one *frame* — i.e. one `INITIAL_BLOCK..FINAL_BLOCK` group of
/// concatenated `wvpk` blocks. This matches FFmpeg's WavPack demuxer
/// packet boundary (spec §8.3).
struct WavPackDecoder {
    codec_id: CodecId,
    pending: Option<Packet>,
}

impl Decoder for WavPackDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if self.pending.is_some() {
            return Err(Error::other(
                "WavPack decoder: receive_frame must be called before sending another packet",
            ));
        }
        self.pending = Some(packet.clone());
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        let Some(pkt) = self.pending.take() else {
            return Err(Error::NeedMore);
        };
        // Walk the packet as a single-frame `.wv` slice.
        let parsed = super::container::parse_file(&pkt.data)?;
        if parsed.frames.len() != 1 {
            return Err(Error::invalid(format!(
                "WavPack: expected 1 frame per packet, found {}",
                parsed.frames.len()
            )));
        }
        let frame = &parsed.frames[0];
        let channels = super::container::decode_frame(&pkt.data, frame)?;
        let n_ch = channels.len();
        if n_ch == 0 {
            return Err(Error::invalid("WavPack: empty frame"));
        }
        let n_samples = channels[0].len();

        // Output sample format is dictated by the *first* block's
        // container BPS field. Per spec §3.4 the bps field is fixed
        // for the whole stream so this is safe.
        let bps = frame.blocks[0].header.container_bps();
        let format = match bps {
            8 => SampleFormat::U8,
            16 => SampleFormat::S16,
            24 => SampleFormat::S32,
            32 => SampleFormat::S32,
            other => {
                return Err(Error::unsupported(format!(
                    "WavPack: container bps {other}"
                )))
            }
        };

        let mut interleaved: Vec<u8> =
            Vec::with_capacity(n_samples * n_ch * format.bytes_per_sample());
        for i in 0..n_samples {
            for c in 0..n_ch {
                let s = channels[c][i];
                match format {
                    SampleFormat::U8 => {
                        interleaved.push((s.wrapping_add(0x80) & 0xFF) as u8);
                    }
                    SampleFormat::S16 => {
                        interleaved.extend_from_slice(&(s as i16).to_le_bytes());
                    }
                    SampleFormat::S32 => {
                        interleaved.extend_from_slice(&s.to_le_bytes());
                    }
                    _ => {
                        return Err(Error::unsupported(
                            "WavPack: unsupported output sample format",
                        ))
                    }
                }
            }
        }
        Ok(Frame::Audio(AudioFrame {
            samples: n_samples as u32,
            pts: pkt.pts,
            data: vec![interleaved],
        }))
    }

    fn flush(&mut self) -> Result<()> {
        Ok(())
    }
}
