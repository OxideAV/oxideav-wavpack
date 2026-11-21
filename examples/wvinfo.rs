//! Dump the block structure of a WavPack file: one line per block
//! (header fields and flags) followed by one line per metadata
//! sub-block, with the decorrelation term list spelled out.
//!
//! ```text
//! cargo run --example wvinfo -- file.wv
//! ```

use oxideav_wavpack::iter_blocks;

fn main() {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: wvinfo <file.wv>");
        std::process::exit(2);
    };
    let bytes = std::fs::read(&path).expect("read input");
    for (n, block) in iter_blocks(&bytes).enumerate() {
        let block = match block {
            Ok(b) => b,
            Err(e) => {
                println!("block {n}: {e}");
                break;
            }
        };
        let h = block.header();
        let f = &h.flags;
        println!(
            "block {n}: ver {:#x} index {} samples {} total {:?} crc {:#010x} flags {:#010x} \
             [bytes {} {}{}{}{}{}{}{}{} shift {} mag {} rate {} mc {}]",
            h.version,
            h.block_index,
            h.block_samples,
            h.total_samples_in_file(),
            h.crc,
            f.raw,
            f.bytes_per_sample(),
            if f.mono { "mono " } else { "stereo " },
            if f.hybrid { "hybrid " } else { "" },
            if f.joint_stereo { "joint " } else { "" },
            if f.cross_channel_decorrelation {
                "cross "
            } else {
                ""
            },
            if f.hybrid_shaping { "shape " } else { "" },
            if f.float_data { "float " } else { "" },
            if f.hybrid_noise_shaping_iir {
                "newshape "
            } else {
                ""
            },
            if f.false_stereo { "false-stereo " } else { "" },
            f.left_shift,
            f.max_magnitude,
            f.sample_rate_index,
            f.multichannel_marker,
        );
        for sub in block.sub_blocks() {
            let p = sub.payload;
            let head: Vec<String> = p.iter().take(24).map(|b| format!("{b:02x}")).collect();
            print!(
                "  id {:#04x} len {:>6}  {}",
                sub.id_byte,
                p.len(),
                head.join(" ")
            );
            if sub.id_byte & 0x3f == 0x02 {
                let terms: Vec<String> = p
                    .iter()
                    .map(|b| format!("{}/d{}", i32::from(b & 0x1f) - 5, b >> 5))
                    .collect();
                print!("   terms(wire order) {}", terms.join(" "));
            }
            println!();
        }
    }
}
