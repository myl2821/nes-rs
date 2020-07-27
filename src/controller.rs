bitflags! {
    pub struct Button: u8 {
        const A = 0x01;
        const B = 0x02;
        const SELECT = 0x04;
        const START = 0x08;
        const UP = 0x10;
        const DOWN = 0x20;
        const LEFT = 0x40;
        const RIGHT = 0x80;
    }
}

pub struct Controller {
    button: Button,
    idx: usize,
    strobe: u8,
}

impl Controller {
    pub fn new() -> Self {
        Self {
            button: Button::empty(),
            idx: 0,
            strobe: 0x00,
        }
    }

    pub fn press(&mut self, b: Button) {
        self.button.insert(b)
    }

    pub fn release(&mut self, b: Button) {
        self.button.remove(b)
    }

    pub fn read(&mut self) -> u8 {
        let mut v = 0;
        if self.idx < 8 {
            v = self.get_button()
        }
        self.idx += 1;
        if self.strobe & 0x01 == 0x01 {
            self.idx = 0
        }
        v
    }

    fn get_button(&self) -> u8 {
        match self
            .button
            .contains(Button::from_bits(1 << self.idx).unwrap())
        {
            true => 1,
            false => 0,
        }
    }

    // Input ($4016 write)
    // While S (strobe) is high, the shift registers in the controllers
    // are continuously reloaded from the button states, and reading $4016/$4017
    // will keep returning the current state of the first button (A).
    // Once S goes low, this reloading will stop.
    pub fn write(&mut self, v: u8) {
        self.strobe = v;
        if self.strobe & 0x01 == 0x01 {
            self.idx = 0
        }
    }
}
