// ref: http://wiki.nesdev.com/w/index.php/APU
//      http://nesdev.com/apu_ref.txt

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

pub struct FrameSequencer {
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
