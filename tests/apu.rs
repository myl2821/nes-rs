use nes::FrameSequencer;

macro_rules! test_idle {
    ($sequencer: expr) => {
        for i in 0..89489 {
            let evt = $sequencer.tick();
            assert_eq!(evt.bits(), 0);
        }
    };
}

#[test]
fn sequencer() {
    let mut sequencer = FrameSequencer::new();
    for i in 0..10 {
        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x04);

        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x06);

        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x04);

        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x07);
    }

    let evt = sequencer.update(0xc0);
    assert_eq!(evt.bits(), 0x06);

    sequencer.reset();
    for i in 0..10 {
        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x06);

        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x04);

        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x06);

        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x04);

        test_idle!(sequencer);
        let evt = sequencer.tick();
        assert_eq!(evt.bits(), 0x00);
    }
}
