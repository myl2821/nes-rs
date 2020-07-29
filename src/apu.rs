// ref: http://wiki.nesdev.com/w/index.php/APU
//      http://nesdev.com/apu_ref.txt

const LENGTH_TABLE: [u8; 32] = [
    0x0a, 0xfe, 0x14, 0x02, 0x28, 0x04, 0x50, 0x06, 0xa0, 0x08, 0x3c, 0x0a, 0x0e, 0x0c, 0x1a, 0x0e,
    0x0c, 0x10, 0x18, 0x12, 0x30, 0x14, 0x60, 0x16, 0xc0, 0x18, 0x48, 0x1a, 0x10, 0x1c, 0x20, 0x1e,
];

struct Divider {
    cycle: u32,
    period: u32,
}

impl Divider {
    pub fn new(period: u32) -> Self {
        Self { cycle: 0, period }
    }

    pub fn tick(&mut self) -> bool {
        self.cycle += 1;
        if self.cycle >= self.period {
            self.cycle = 0;
            return true;
        }
        false
    }

    pub fn update(&mut self, period: u32) {
        self.period = period;
        self.reset();
    }

    pub fn reset(&mut self) {
        self.cycle = 0;
    }
}

bitflags! {
    pub struct SeqEvent: u8 {
        // Set interrupt flag
        const F = 0x01;
        // clock length counters
        const L = 0x02;
        // clock envelopes
        const E = 0x04;
    }
}

struct FrameSequencer {
    cycle: u8,
    seq_len: u8, // 4 (mode 0) / 5 (mode 1)
    irq_disable: bool,
    divider: Divider,
}

impl FrameSequencer {
    pub fn new() -> Self {
        Self {
            cycle: 0,
            seq_len: 4,
            irq_disable: false,
            divider: Divider::new(89490),
        }
    }

    // update by a write to $4017
    pub fn update(&mut self, val: u8) -> SeqEvent {
        match val & 0x80 {
            0 => self.seq_len = 4,
            _ => self.seq_len = 5,
        }

        self.irq_disable = (val & 0x40 != 0);
        self.reset();

        if self.seq_len == 5 {
            return self.step();
        }

        SeqEvent::empty()
    }

    pub fn reset(&mut self) {
        self.divider.reset();
        self.cycle = 0;
    }

    fn step(&mut self) -> SeqEvent {
        let mut evt = match self.seq_len {
            4 => match self.cycle {
                0 => SeqEvent::E,
                1 => SeqEvent::L | SeqEvent::E,
                2 => SeqEvent::E,
                _ => SeqEvent::F | SeqEvent::L | SeqEvent::E,
            },
            _ => match self.cycle {
                0 => SeqEvent::L | SeqEvent::E,
                1 => SeqEvent::E,
                2 => SeqEvent::L | SeqEvent::E,
                3 => SeqEvent::E,
                _ => SeqEvent::empty(),
            },
        };

        if (self.irq_disable) {
            evt.remove(SeqEvent::F);
        }

        self.cycle = (self.cycle + 1) % self.seq_len;
        return evt;
    }

    pub fn tick(&mut self) -> SeqEvent {
        if self.divider.tick() {
            return self.step();
        }
        SeqEvent::empty()
    }
}

// A length counter allows automatic duration control
struct LengthCounter {
    pub enabled: bool,
    pub halt: bool,
    counter: u8,
}

impl LengthCounter {
    pub fn new(length_index: u8) -> Self {
        Self {
            enabled: true,
            halt: false,
            counter: LENGTH_TABLE[(length_index as usize >> 3) & 0x1f],
        }
    }

    pub fn update(&mut self, length_index: u8) {
        self.counter = LENGTH_TABLE[(length_index as usize >> 3) & 0x1f]
    }

    pub fn tick(&mut self) {
        if self.counter > 0 {
            self.counter -= 1;
        }
    }

    pub fn activated(&self) -> bool {
        self.counter > 0
    }
}

// https://wiki.nesdev.com/w/index.php/APU_Envelope
struct Envelope {
    volume: u8,
    loop_enable: bool,
    disable: bool,
    n: u8, // constant volume/envelope divider period
    counter: u8,
    reset: bool,
    divider: Divider,
}

impl Envelope {
    pub fn new() -> Self {
        Self {
            volume: 0,
            loop_enable: false,
            disable: false,
            n: 0,
            counter: 15,
            reset: false,
            divider: Divider::new(1),
        }
    }

    // $4000/$4004
    // --ld nnnn       loop, disable, n
    pub fn update(&mut self, v: u8) {
        self.loop_enable = (v>>5)&0x01 == 1;
        self.disable = (v>>4)&0x01 == 1;
        self.n = v&0x0f;

        if self.disable {
            self.volume = self.n;
        } else {
            self.divider.update((self.n + 1) as u32);
        }
    }

    pub fn reset(&mut self) {
        self.reset = true;
    }

    pub fn tick(&mut self) {
        if self.reset {
            self.reset = false;
            self.counter = 15;
            self.divider.reset();
        } else {
            if self.divider.tick() {
                if self.counter == 0 && self.loop_enable {
                    self.counter = 15;
                } else if self.counter != 0 {
                    self.counter -= 1
                }
            }
        }

        if self.disable {
            self.volume = self.n;
        } else {
            self.volume = self.counter;
        }
    }
}


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
    for _ in 0..10 {
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
    for _ in 0..10 {
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

#[test]
fn length_counter() {
    let mut lc = LengthCounter::new(1 << 3);
    for _ in 0..0xfd {
        lc.tick();
        assert!(lc.activated());
    }

    lc.tick();
    assert!(!lc.activated());
}

#[test]
fn envelope() {
    let mut envelope = Envelope::new();

    // constant volume
    envelope.update(0x1f);
    envelope.tick();
    assert_eq!(0x0f, envelope.volume);

    // saw envelope and loop disable
    envelope.reset();
    envelope.update(0x00);
    for i in 0..0x10 {
        envelope.tick();
        assert_eq!(15-i, envelope.volume);
    }
    envelope.tick();
    assert_eq!(0, envelope.volume);

    // saw envelope and loop enable
    envelope.reset();
    envelope.update(0x20);
    for i in 0..0x10 {
        envelope.tick();
        assert_eq!(15-i, envelope.volume);
    }
    envelope.tick();
    assert_eq!(15, envelope.volume)
}
