//! Hand-crafted byte-level smoke tests for the block parser and the
//! sub-block walker. These exercise the on-disk layout without
//! invoking ffmpeg.

use oxideav_wavpack::block::{parse_sub_blocks, BlockHeader};

#[test]
fn block_header_parses_baseline_stereo_example() {
    // First 32 bytes from spec §3.1 hex dump:
    //   77 76 70 6B 90 39 00 00 10 04 00 00 44 AC 00 00
    //   00 00 00 00 22 56 00 00 31 18 BC 04 97 92 3F 28
    let buf = [
        0x77, 0x76, 0x70, 0x6B, // 'wvpk'
        0x90, 0x39, 0x00, 0x00, // block_size = 0x3990 = 14736
        0x10, 0x04, // version = 0x0410
        0x00, 0x00, // track_number / index_number
        0x44, 0xAC, 0x00, 0x00, // total_samples = 0xAC44 = 44100
        0x00, 0x00, 0x00, 0x00, // block_index = 0
        0x22, 0x56, 0x00, 0x00, // block_samples = 0x5622 = 22050
        0x31, 0x18, 0xBC, 0x04, // flags = 0x04bc1831
        0x97, 0x92, 0x3F, 0x28, // crc
    ];
    let h = BlockHeader::parse(&buf).expect("parse");
    assert_eq!(h.block_size, 14736);
    assert_eq!(h.version, 0x0410);
    assert_eq!(h.total_samples, 44100);
    assert_eq!(h.block_index, 0);
    assert_eq!(h.block_samples, 22050);
    assert_eq!(h.flags, 0x04bc_1831);
    assert_eq!(h.crc, 0x283F_9297);
    assert!(h.is_initial());
    assert!(h.is_final());
    assert!(!h.is_mono_data());
    assert!(h.is_joint_stereo());
    assert!(h.is_cross_decorr());
    assert_eq!(h.container_bps(), 16);
    assert_eq!(h.channels_in_block(), 2);
}

#[test]
fn block_header_rejects_bad_magic() {
    let mut buf = [0u8; 32];
    buf[..4].copy_from_slice(b"XYZW");
    buf[4..8].copy_from_slice(&100u32.to_le_bytes());
    buf[8..10].copy_from_slice(&0x0410u16.to_le_bytes());
    assert!(BlockHeader::parse(&buf).is_err());
}

#[test]
fn block_header_rejects_old_version() {
    let mut buf = [0u8; 32];
    buf[..4].copy_from_slice(b"wvpk");
    buf[4..8].copy_from_slice(&100u32.to_le_bytes());
    buf[8..10].copy_from_slice(&0x0401u16.to_le_bytes());
    assert!(BlockHeader::parse(&buf).is_err());
}

#[test]
fn parse_sub_blocks_walks_concatenated_subs() {
    // Build a payload with three sub-blocks.
    // sub 0: id=0x02 (DECTERMS), 8-bit size=2 words (4 bytes payload)
    // sub 1: id=0x05 (ENTROPY), 8-bit size=3 words (6 bytes payload)
    // sub 2: id=0x8A (DATA|LARGE), 24-bit size=2 words (4 bytes payload)
    let mut p = Vec::new();
    p.push(0x02);
    p.push(0x02);
    p.extend_from_slice(&[1, 2, 3, 4]);
    p.push(0x05);
    p.push(0x03);
    p.extend_from_slice(&[10, 11, 12, 13, 14, 15]);
    p.push(0x8A);
    p.extend_from_slice(&[0x02, 0x00, 0x00]);
    p.extend_from_slice(&[0x99, 0xAA, 0xBB, 0xCC]);

    let subs = parse_sub_blocks(&p).expect("walk");
    assert_eq!(subs.len(), 3);
    assert_eq!(subs[0].ty(), 0x02);
    assert_eq!(subs[0].data, &[1, 2, 3, 4]);
    assert_eq!(subs[1].ty(), 0x05);
    assert_eq!(subs[1].data.len(), 6);
    assert_eq!(subs[2].ty(), 0x0A);
    assert_eq!(subs[2].data, &[0x99, 0xAA, 0xBB, 0xCC]);
}
