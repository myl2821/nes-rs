use crate::{Bus, Mapper};
use std::cell::RefCell;
use std::rc::Rc;

const VERCTOR_NMI: u16 = 0xFFFA;
const VERCTOR_RST: u16 = 0xFFFC;
const VERCTOR_IRQ: u16 = 0xFFFE;

bitflags! {
    struct F: u8 {
        // Carry Flag (C)
        const C = 0x01;
        // Zero Flag (Z)
        const Z = 0x02;
        // Interrupt Disable (I)
        const I = 0x04;
        // Decimal Mode (D)
        const D = 0x08;
        // Break Command (B)
        const B = 0x10;
        // Unused Bit (U)
        const U = 0x20;
        // Overflow Flag (V)
        const V = 0x40;
        // Negative Flag (N)
        const N = 0x80;
    }
}

pub struct CPU<T: Mapper> {
    // The program counter is a 16-bit register which holds the address of the next instruction to be executed
    PC: u16,

    //The stack pointer is an 8-bit register which serves as an offset from $0100
    SP: u8,

    // The accumulator is an 8-bit register which stores the results of arithmetic and logic operations
    A: u8,

    // The X register is an 8-bit register typically used as a counter or an offset for certain addressing modes
    X: u8,

    // The Y register is an 8-bit register which is used in the same way as the X register, as a counter or to store an offset
    Y: u8,

    // The status register contains a number of single bit flags which are set or cleared when instructions are executed
    // Carry Flag (C)
    // Zero Flag (Z)
    // Interrupt Disable (I)
    // Decimal Mode (D)
    // Break Command (B)
    // Unused Bit (U)
    // Overflow Flag (V)
    // Negative Flag (N)
    // Status register layout
    // ---------------------------------
    //   7 | 6 | 5 | 4 | 3 | 2 | 1 | 0
    //   N | V | U | B | D | I | Z | C
    P: F,

    // Instruction function table
    ins: [fn(&mut CPU<T>, &Info); 256],

    cycles: u64,

    bus: Option<Rc<RefCell<Bus<T>>>>,
}

// Address Modes:
// A     .... Accumulator             OPC A           operand is AC (implied single byte instruction)
// abs   .... absolute                OPC $LLHH       operand is address $HHLL *
// abs,X .... absolute, X-indexed     OPC $LLHH,X     operand is address; effective address is address incremented by X with carry **
// abs,Y .... absolute, Y-indexed     OPC $LLHH,Y     operand is address; effective address is address incremented by Y with carry **
// #     .... immediate               OPC #$BB        operand is byte BB
// impl  .... implied                 OPC             operand implied
// ind   .... indirect                OPC ($LLHH)     operand is address; effective address is contents of word at address: C.w($HHLL)
// X,ind .... X-indexed, indirect     OPC ($LL,X)     operand is zeropage address; effective address is word in (LL + X, LL + X + 1), inc. without carry: C.w($00LL + X)
// ind,Y .... indirect, Y-indexed     OPC ($LL),Y     operand is zeropage address; effective address is word in (LL, LL + 1) incremented by Y with carry: C.w($00LL) + Y
// rel   .... relative                OPC $BB         branch target is PC + signed offset BB ***
// zpg   .... zeropage                OPC $LL         operand is zeropage address (hi-byte is zero, address = $00LL)
// zpg,X .... zeropage, X-indexed     OPC $LL,X       operand is zeropage address; effective address is address incremented by X without carry **
// zpg,Y .... zeropage, Y-indexed     OPC $LL,Y       operand is zeropage address; effective address is address incremented by Y without carry **
// *   16-bit address words are little endian, lo(w)-byte first, followed by the hi(gh)-byte.
// (An assembler will use a human readable, big-endian notation as in $HHLL.)
//
// **  The available 16-bit address space is conceived as consisting of pages of 256 bytes each, with
// address hi-bytes represententing the page index. An increment with carry may affect the hi-byte
// and may thus result in a crossing of page boundaries, adding an extra cycle to the execution.
// Increments without carry do not affect the hi-byte of an address and no page transitions do occur.
// Generally, increments of 16-bit addresses include a carry, increments of zeropage addresses don't.
// Notably this is not related in any way to the state of the carry bit of the accumulator.
//
// *** Branch offsets are signed 8-bit values, -128 ... +127, negative offsets in two's complement.
// Page transitions may occur and add an extra cycle to the exucution.
#[derive(Debug, Clone, Copy)]
enum Mode {
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Accumulator,
    Immediate,
    Implied,
    IndexedIndirect,
    Indirect,
    IndirectIndexed,
    Relative,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
}

pub struct OP {
    name: &'static str,
    mode: Mode,
    len: u16,
    cycle: u64,
    page_cycle: u64,
}

struct Info {
    addr: u16,
    pc: u16,
    mode: Mode,
}

impl<T: Mapper> CPU<T> {
    pub fn new() -> Self {
        let mut cpu = CPU {
            PC: 0x0000,
            SP: 0x00,
            A: 0x00,
            X: 0x00,
            Y: 0x00,
            P: F::empty(),
            ins: [CPU::jmp; 256],
            cycles: 0,
            bus: None,
        };
        cpu.init_ins_table();
        cpu
    }

    pub fn connect_bus(&mut self, bus: Rc<RefCell<Bus<T>>>) {
        self.bus = Some(bus);
        self.reset();
    }

    pub fn read8(&self, addr: u16) -> u8 {
        self.bus.as_ref().unwrap().borrow_mut().cpu_read8(addr)
    }

    pub fn write8(&mut self, addr: u16, v: u8) {
        self.bus.as_ref().unwrap().borrow_mut().cpu_write8(addr, v)
    }

    pub fn read16(&self, addr: u16) -> u16 {
        let lo = self.read8(addr) as u16;
        let hi = self.read8(addr + 1) as u16;
        lo | hi << 8
    }

    // FIXME
    // operand is address
    // addr+1 may cross page boundary, reset it to page's start address
    pub fn readbug(&self, addr: u16) -> u16 {
        let lo = self.read8(addr) as u16;
        let hi = self.read8((addr & 0xff00) | ((addr + 1) & 0x00ff)) as u16;

        lo | hi << 8
    }

    // FIXME
    // Zero Page 0x0000..=0x0100
    // operand is zeropage address
    pub fn readbug_zero_page(&self, addr: u16) -> u16 {
        let lo = self.read8(addr & 0x00ff) as u16;
        let hi = self.read8((addr + 1) & 0x00ff) as u16;

        lo | hi << 8
    }

    pub fn nmi(&self) -> u16 {
        self.read16(VERCTOR_NMI)
    }

    pub fn rst(&self) -> u16 {
        self.read16(VERCTOR_RST)
    }

    pub fn irq(&self) -> u16 {
        self.read16(VERCTOR_IRQ)
    }

    pub fn reset(&mut self) {
        self.PC = self.rst();
        self.SP = 0xfd;
        self.P = F::U | F::I;
        self.cycles = 7;
    }

    pub fn set_PC(&mut self, pc: u16) {
        self.PC = pc
    }

    pub fn set_cycles(&mut self, cycles: u64) {
        self.cycles = cycles
    }

    pub fn step(&mut self) -> u64 {
        let cycles = self.cycles;

        // TODO: Detect interrupts

        // Read instruction
        let opcode = self.read8(self.PC) as usize;
        let op = &OP_MAP[opcode];

        // Get address by different addressing modes
        let addr = self.addr(op.mode);

        self.cycles += op.cycle;

        // Page transitions may occur and add an extra cycle to the exucution.
        if self.has_crossed(op.mode) {
            self.cycles += op.page_cycle
        }

        self.PC += op.len;

        let info = Info {
            addr: addr,
            pc: self.PC,
            mode: op.mode,
        };

        // Execute instruction
        (self.ins[opcode])(self, &info);

        self.cycles - cycles
    }

    // For Debug
    pub fn debug_info(&self) -> (String, u16, String, u8, u8, u8, u8, u8, u64) {
        let opcode = self.read8(self.PC) as usize;
        let op = &OP_MAP[opcode];
        let addr = self.addr(op.mode);

        let byte0 = format!("{:02X}", self.read8(self.PC));
        let mut byte1 = format!("{:02X}", self.read8(self.PC + 1));
        let mut byte2 = format!("{:02X}", self.read8(self.PC + 2));

        if op.len < 3 {
            byte2 = "  ".to_string()
        }
        if op.len < 2 {
            byte1 = "  ".to_string()
        }

        let op_str = match op.mode {
            Mode::Absolute => format!("${:04X}", addr),
            Mode::AbsoluteX => format!("${:04X},X", addr),
            Mode::AbsoluteY => format!("${:04X},Y", addr),
            Mode::Accumulator => format!("{:02X}", self.A),
            Mode::Immediate => format!("#${:02X}", self.read8(addr)),
            Mode::Implied => format!("\t"),
            Mode::IndexedIndirect => format!("(${:02X},X)", addr),
            Mode::Indirect => format!("(${:04X})", addr),
            Mode::IndirectIndexed => format!("(${:02X},Y)", addr),
            Mode::Relative => format!("${:02X}", self.addr(op.mode)),
            Mode::ZeroPage => format!("${:02X} = 00", addr),
            Mode::ZeroPageX => format!("${:02X},X", addr),
            Mode::ZeroPageY => format!("${:02X},Y", addr),
        };

        // TODO: PPU
        let s = format!(
        "{:04X}  {} {} {}  {} {}\t\t\tA:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} PPU:{:3o},{:3o} CYC:{}",
        self.PC, byte0, byte1, byte2, op.name, op_str, self.A, self.X, self.Y,
        self.P, self.SP, 0, (self.cycles*3)%341, self.cycles,
        );
        (
            s,
            self.PC,
            op.name.into(),
            self.A,
            self.X,
            self.Y,
            self.P.bits,
            self.SP,
            self.cycles,
        )
    }

    fn addr(&self, mode: Mode) -> u16 {
        match mode {
            Mode::Absolute => self.read16(self.PC + 1),
            Mode::AbsoluteX => self.read16(self.PC + 1).wrapping_add(self.X as u16),
            Mode::AbsoluteY => self.read16(self.PC + 1).wrapping_add(self.Y as u16),
            Mode::Accumulator => 0,
            Mode::Immediate => self.PC + 1,
            Mode::Implied => 0,
            Mode::IndexedIndirect => {
                self.readbug_zero_page(self.read8(self.PC + 1).wrapping_add(self.X) as u16)
            }
            Mode::Indirect => self.readbug(self.read16(self.PC + 1)),
            Mode::IndirectIndexed => self
                .readbug_zero_page(self.read8(self.PC + 1) as u16)
                .wrapping_add(self.Y as u16),
            Mode::Relative => {
                let offset = self.read8(self.PC + 1);
                if offset < 0x80 {
                    self.PC + 2 + offset as u16
                } else {
                    self.PC + 2 + offset as u16 - 0x100
                }
            }
            Mode::ZeroPage => self.read8(self.PC + 1) as u16,
            Mode::ZeroPageX => self.read8(self.PC + 1).wrapping_add(self.X) as u16,
            Mode::ZeroPageY => self.read8(self.PC + 1).wrapping_add(self.Y) as u16,
        }
    }

    fn has_crossed(&self, mode: Mode) -> bool {
        let addr = self.addr(mode);
        match mode {
            Mode::AbsoluteX => self.page_diff(addr.wrapping_sub(self.X as u16), addr),
            Mode::AbsoluteY | Mode::IndirectIndexed => {
                self.page_diff(addr.wrapping_sub(self.Y as u16), addr)
            }
            _ => false,
        }
    }

    // page_diff tests if addr1 and addr2 reference different pages
    // pages of 256 bytes each
    fn page_diff(&self, addr1: u16, addr2: u16) -> bool {
        addr1 & 0xff00 != addr2 & 0xff00
    }

    // add 1 to cycles if branch occurs on same page
    // add 2 to cycles if branch occurs to different page
    fn add_branch_cycle(&mut self, info: &Info) {
        self.cycles += 1;
        if self.page_diff(info.pc, info.addr) {
            self.cycles += 1
        }
    }

    fn without_flag(&self, f: F) -> bool {
        !self.P.contains(f)
    }

    fn contains_flag(&self, f: F) -> bool {
        self.P.contains(f)
    }

    fn clear_flag(&mut self, f: F) {
        self.P.remove(f)
    }

    fn set_flag(&mut self, f: F) {
        self.P.insert(f)
    }

    fn get_flag(&mut self, f: F) -> u8 {
        match self.without_flag(f) {
            true => 0,
            _ => 1,
        }
    }

    fn update_flag(&mut self, f: F, v: u8) {
        match v {
            0x00 => self.clear_flag(f),
            _ => self.set_flag(f),
        }
    }

    fn set_Z(&mut self, v: u8) {
        match v {
            0x00 => self.set_flag(F::Z),
            _ => self.clear_flag(F::Z),
        }
    }

    fn set_N(&mut self, v: u8) {
        match v & F::N.bits {
            0x00 => self.clear_flag(F::N),
            _ => self.set_flag(F::N),
        }
    }

    fn set_NZ(&mut self, v: u8) {
        self.set_N(v);
        self.set_Z(v);
    }

    // The stack is located at memory locations $0100-$01FF. The stack pointer is
    // an 8-bit register which serves as an offset from $0100. The stack works
    // top-down, so when a byte is pushed on to the stack, the stack pointer is
    // decremented and when a byte is pulled from the stack, the stack pointer is
    // incremented. There is no detection of stack overflow and the stack pointer
    // will just wrap around from $00 to $FF.
    fn push8(&mut self, v: u8) {
        self.write8(0x0100 | self.SP as u16, v);
        self.SP -= 1
    }

    fn pull8(&mut self) -> u8 {
        self.SP += 1;
        self.read8(0x0100 | self.SP as u16)
    }

    fn push16(&mut self, v: u16) {
        self.push8((v >> 8) as u8);
        self.push8(v as u8)
    }

    fn pull16(&mut self) -> u16 {
        let l = self.pull8() as u16;
        let h = self.pull8() as u16;
        h << 8 | l
    }

    // Branch on condition
    fn branch_on(&mut self, condition: bool, info: &Info) {
        if condition {
            self.PC = info.addr;
            self.add_branch_cycle(info)
        }
    }

    fn compare(&mut self, m: u8, n: u8) {
        self.set_NZ(m.wrapping_sub(n));
        if m >= n {
            self.set_flag(F::C)
        } else {
            self.clear_flag(F::C)
        }
    }

    ///////////////////////////////////////////////////////////////
    //    6502 Instruction Set                                   //
    //    https://www.masswerk.at/6502/6502_instruction_set.html //
    ///////////////////////////////////////////////////////////////

    // ADC Add Memory to Accumulator with Carry
    // A + M + C -> A, C
    // N Z C I D V
    // + + + - - +
    fn adc(&mut self, info: &Info) {
        let a = self.A;
        let m = self.read8(info.addr);
        let c = self.get_flag(F::C);

        self.A = a.wrapping_add(m).wrapping_add(c);
        self.set_NZ(self.A);

        if a as u16 + m as u16 + c as u16 > 0xff {
            self.set_flag(F::C)
        } else {
            self.clear_flag(F::C)
        }

        if ((a ^ m) >> 7) & 1 == 0 && ((a ^ self.A) >> 7) & 1 != 0 {
            self.set_flag(F::V)
        } else {
            self.clear_flag(F::V)
        }
    }

    // AND AND Memory with Accumulator
    // A AND M -> A
    // N Z C I D V
    // + + - - - -
    fn and(&mut self, info: &Info) {
        self.A &= self.read8(info.addr);
        self.set_NZ(self.A)
    }

    // ASL Shift Left One Bit (Memory or Accumulator)
    // C <- [76543210] <- 0
    // N Z C I D V
    // + + + - - -
    fn asl(&mut self, info: &Info) {
        match info.mode {
            Mode::Accumulator => {
                self.update_flag(F::C, (self.A >> 7) & 0x01);
                self.A <<= 1;
                self.set_NZ(self.A)
            }
            _ => {
                let mut m = self.read8(info.addr);
                self.update_flag(F::C, (m >> 7) & 0x01);
                m <<= 1;
                self.write8(info.addr, m);
                self.set_NZ(m)
            }
        }
    }

    // BCC Branch on Carry Clear
    // branch on C = 0
    // N Z C I D V
    // - - - - - -
    fn bcc(&mut self, info: &Info) {
        self.branch_on(self.without_flag(F::C), info)
    }

    // BCS Branch on Carry Set
    // branch on C = 1
    // N Z C I D V
    // - - - - - -
    fn bcs(&mut self, info: &Info) {
        self.branch_on(self.contains_flag(F::C), info)
    }

    // BEQ Branch on Result Zero
    // branch on Z = 1
    // N Z C I D V
    // - - - - - -
    fn beq(&mut self, info: &Info) {
        self.branch_on(self.contains_flag(F::Z), info)
    }

    // BIT Test Bits in Memory with Accumulator
    // bits 7 and 6 of operand are transfered to bit 7 and 6 of SR (N,V);
    // the zeroflag is set to the result of operand AND accumulator.
    // A AND M, M7 -> N, M6 -> V
    //  N Z C I D V
    // M7 + - - - M6
    fn bit(&mut self, info: &Info) {
        let m = self.read8(info.addr);
        self.set_Z(self.A & m);
        self.set_N(m);
        self.update_flag(F::V, (m >> 6) & 0x01)
    }

    // BMI Branch on Result Minus
    // branch on N = 1
    // N Z C I D V
    // - - - - - -
    fn bmi(&mut self, info: &Info) {
        self.branch_on(self.contains_flag(F::N), info)
    }

    // Branch on Result not Zero
    // branch on Z = 0
    // N Z C I D V
    // - - - - - -
    fn bne(&mut self, info: &Info) {
        self.branch_on(self.without_flag(F::Z), info)
    }

    // BPL Branch on Result Plus
    // branch on N = 0
    // N Z C I D V
    // - - - - - -
    fn bpl(&mut self, info: &Info) {
        self.branch_on(self.without_flag(F::N), info)
    }

    fn brk(&mut self, info: &Info) {
        unimplemented!()
    }

    // BVC Branch on Overflow Clear
    // branch on V = 0
    // N Z C I D V
    // - - - - - -
    fn bvc(&mut self, info: &Info) {
        self.branch_on(self.without_flag(F::V), info)
    }

    // BVS Branch on Overflow Set
    // branch on V = 1
    // N Z C I D V
    // - - - - - -
    fn bvs(&mut self, info: &Info) {
        self.branch_on(self.contains_flag(F::V), info)
    }

    // CLC Clear Carry Flag
    // 0 -> C
    // N Z C I D V
    // - - 0 - - -
    fn clc(&mut self, _info: &Info) {
        self.clear_flag(F::C)
    }

    // CLD Clear Decimal Mode
    // 0 -> D
    // N Z C I D V
    // - - - - 0 -
    fn cld(&mut self, _info: &Info) {
        self.clear_flag(F::D)
    }

    fn cli(&mut self, info: &Info) {
        unimplemented!()
    }

    // CLV Clear Overflow Flag
    // 0 -> V
    // N Z C I D V
    // - - - - - 0
    fn clv(&mut self, _info: &Info) {
        self.clear_flag(F::V)
    }

    // CMP Compare Memory with Accumulator
    // A - M
    // N Z C I D V
    // + + + - - -
    fn cmp(&mut self, info: &Info) {
        self.compare(self.A, self.read8(info.addr))
    }

    // CPX Compare Memory and Index X
    // X - M
    // N Z C I D V
    // + + + - - -
    fn cpx(&mut self, info: &Info) {
        self.compare(self.X, self.read8(info.addr))
    }

    // CPY Compare Memory and Index Y
    // Y - M
    // N Z C I D V
    // + + + - - -
    fn cpy(&mut self, info: &Info) {
        self.compare(self.Y, self.read8(info.addr))
    }

    // DEC Decrement Memory by One
    // M - 1 -> M
    // N Z C I D V
    // + + - - - -
    fn dec(&mut self, info: &Info) {
        let m = self.read8(info.addr).wrapping_sub(1);
        self.write8(info.addr, m);
        self.set_NZ(m)
    }

    // DEX Decrement Index X by One
    // X - 1 -> X
    // N Z C I D V
    // + + - - - -
    fn dex(&mut self, _info: &Info) {
        self.X = self.X.wrapping_sub(1);
        self.set_NZ(self.X)
    }

    // DEY Decrement Index Y by One
    // Y - 1 -> Y
    // N Z C I D V
    // + + - - - -
    fn dey(&mut self, _info: &Info) {
        self.Y = self.Y.wrapping_sub(1);
        self.set_NZ(self.Y)
    }

    // EOR Exclusive-OR Memory with Accumulator
    // A EOR M -> A
    // N Z C I D V
    // + + - - - -
    fn eor(&mut self, info: &Info) {
        let m = self.read8(info.addr);
        self.A ^= m;
        self.set_NZ(self.A)
    }

    // INC Increment Memory by One
    // M + 1 -> M
    // N Z C I D V
    // + + - - - -
    fn inc(&mut self, info: &Info) {
        let m = self.read8(info.addr).wrapping_add(1) as u8;
        self.write8(info.addr, m);
        self.set_NZ(m)
    }

    // INX Increment Index X by One
    // X + 1 -> X
    // N Z C I D V
    // + + - - - -
    fn inx(&mut self, _info: &Info) {
        self.X = self.X.wrapping_add(1);
        self.set_NZ(self.X)
    }

    // INY Increment Index Y by One
    // Y + 1 -> Y
    // N Z C I D V
    // + + - - - -
    fn iny(&mut self, _info: &Info) {
        self.Y = self.Y.wrapping_add(1);
        self.set_NZ(self.Y)
    }

    // JMP - Jump to New Location
    // (PC+1) -> PCL
    // (PC+2) -> PCH
    // N Z C I D V
    // - - - - - -
    fn jmp(&mut self, info: &Info) {
        self.PC = info.addr
    }

    // JSR Jump to New Location Saving Return Address
    // push (PC+2)
    // (PC+1) -> PCL
    // (PC+2) -> PCH
    // N Z C I D V
    // - - - - - -
    fn jsr(&mut self, info: &Info) {
        self.push16(self.PC - 1);
        self.PC = info.addr
    }

    // LDA Load Accumulator with Memory
    // M -> A
    // N Z C I D V
    // + + - - - -
    fn lda(&mut self, info: &Info) {
        self.A = self.read8(info.addr);
        self.set_NZ(self.A)
    }

    // LDX Load Index X with Memory
    // M -> X
    // N Z C I D V
    // + + - - - -
    fn ldx(&mut self, info: &Info) {
        self.X = self.read8(info.addr);
        self.set_NZ(self.X)
    }

    // LDY Load Index Y with Memory
    // M -> Y
    // N Z C I D V
    // + + - - - -
    fn ldy(&mut self, info: &Info) {
        self.Y = self.read8(info.addr);
        self.set_NZ(self.Y)
    }

    // LSR Shift One Bit Right (Memory or Accumulator)
    // 0 -> [76543210] -> C
    // N Z C I D V
    // 0 + + - - -
    fn lsr(&mut self, info: &Info) {
        match info.mode {
            Mode::Accumulator => {
                self.update_flag(F::C, self.A & 0x01);
                self.A >>= 1;
                self.set_NZ(self.A)
            }
            _ => {
                let mut m = self.read8(info.addr);
                self.update_flag(F::C, m & 0x01);
                m >>= 1;
                self.write8(info.addr, m);
                self.set_NZ(m)
            }
        }
    }

    // NOP No Operation
    // N Z C I D V
    // - - - - - -
    fn nop(&mut self, _info: &Info) {}

    // ORA OR Memory with Accumulator
    // A OR M -> A
    // N Z C I D V
    // + + - - - -
    fn ora(&mut self, info: &Info) {
        let m = self.read8(info.addr);
        self.A |= m;
        self.set_NZ(self.A)
    }

    // PHA Push Accumulator on Stack
    // push A
    // N Z C I D V
    // - - - - - -
    fn pha(&mut self, _info: &Info) {
        self.push8(self.A)
    }

    // PHP Push Processor Status on Stack
    // push SR
    // N Z C I D V
    // - - - - - -
    fn php(&mut self, _info: &Info) {
        self.push8(self.P.bits | 0x10) // FIXME
    }

    // PLA Pull Accumulator from Stack
    // N Z C I D V
    // + + - - - -
    fn pla(&mut self, _info: &Info) {
        self.A = self.pull8();
        self.set_NZ(self.A)
    }

    // PLP Pull Processor Status from Stack
    // pull SR
    // N Z C I D V
    // from stack
    fn plp(&mut self, _info: &Info) {
        self.P = F::from_bits(self.pull8() & 0xef | 0x20).unwrap()
    }

    // ROL Rotate One Bit Left (Memory or Accumulator)
    // C <- [76543210] <- C
    // N Z C I D V
    // + + + - - -
    fn rol(&mut self, info: &Info) {
        match info.mode {
            Mode::Accumulator => {
                let c = self.get_flag(F::C);
                self.update_flag(F::C, (self.A >> 7) & 0x01);
                self.A = (self.A << 1) | c;
                self.set_NZ(self.A)
            }
            _ => {
                let mut m = self.read8(info.addr);
                let c = self.get_flag(F::C);
                self.update_flag(F::C, (m >> 7) & 0x01);
                m = (m << 1) | c;
                self.write8(info.addr, m);
                self.set_NZ(m)
            }
        }
    }

    // ROR Rotate One Bit Right (Memory or Accumulator)
    // C -> [76543210] -> C
    // N Z C I D V
    // + + + - - -
    fn ror(&mut self, info: &Info) {
        match info.mode {
            Mode::Accumulator => {
                let c = self.get_flag(F::C);
                self.update_flag(F::C, self.A & 0x01);
                self.A = (self.A >> 1) | (c << 7);
                self.set_NZ(self.A)
            }
            _ => {
                let mut m = self.read8(info.addr);
                let c = self.get_flag(F::C);
                self.update_flag(F::C, m & 0x01);
                m = (m >> 1) | (c << 7);
                self.write8(info.addr, m);
                self.set_NZ(m)
            }
        }
    }

    // RTI Return from Interrupt
    // pull SR, pull PC
    // N Z C I D V
    // from stack
    fn rti(&mut self, info: &Info) {
        self.plp(info);
        self.PC = self.pull16()
    }

    // RTS Return from Subroutine
    // pull PC, PC+1 -> PC
    // N Z C I D V
    // - - - - - -
    fn rts(&mut self, _info: &Info) {
        self.PC = self.pull16() + 1
    }

    // SBC Subtract Memory from Accumulator with Borrow
    //         _
    // A - M - C -> A
    // N Z C I D V
    // + + + - - +
    fn sbc(&mut self, info: &Info) {
        let a = self.A;
        let m = self.read8(info.addr);
        let c = self.get_flag(F::C);

        self.A = a.wrapping_sub(m).wrapping_sub(1 - c);
        self.set_NZ(self.A);

        if a as i16 - m as i16 - (1 - c) as i16 >= 0 {
            self.set_flag(F::C)
        } else {
            self.clear_flag(F::C)
        }

        if ((a ^ m) >> 7) & 1 != 0 && ((a ^ self.A) >> 7) & 1 != 0 {
            self.set_flag(F::V)
        } else {
            self.clear_flag(F::V)
        }
    }

    // SEC Set Carry Flag
    // 1 -> C
    // N Z C I D V
    // - - 1 - - -
    fn sec(&mut self, _info: &Info) {
        self.set_flag(F::C)
    }

    // SED Set Decimal Flag
    // 1 -> D
    // N Z C I D V
    // - - - - 1 -
    fn sed(&mut self, _info: &Info) {
        self.set_flag(F::D)
    }

    // SEI Set Interrupt Disable Status
    // 1 -> I
    // N Z C I D V
    // - - - 1 - -
    fn sei(&mut self, _info: &Info) {
        self.set_flag(F::I)
    }

    // TA Store Accumulator in Memory
    // A -> M
    // N Z C I D V
    // - - - - - -
    fn sta(&mut self, info: &Info) {
        self.write8(info.addr, self.A)
    }

    // STX Store Index X in Memory
    // X -> M
    // N Z C I D V
    // - - - - - -
    fn stx(&mut self, info: &Info) {
        self.write8(info.addr, self.X)
    }

    // STY Sore Index Y in Memory
    // Y -> M
    // N Z C I D V
    // - - - - - -
    fn sty(&mut self, info: &Info) {
        self.write8(info.addr, self.Y)
    }

    // TAX Transfer Accumulator to Index X
    // A -> X
    // N Z C I D V
    // + + - - - -
    fn tax(&mut self, _info: &Info) {
        self.X = self.A;
        self.set_NZ(self.X)
    }

    // TAY Transfer Accumulator to Index Y
    // A -> Y
    // N Z C I D V
    // + + - - - -
    fn tay(&mut self, _info: &Info) {
        self.Y = self.A;
        self.set_NZ(self.Y)
    }

    // TSX Transfer Stack Pointer to Index X
    // SP -> X
    // N Z C I D V
    // + + - - - -
    fn tsx(&mut self, _info: &Info) {
        self.X = self.SP;
        self.set_NZ(self.X)
    }

    // TXA Transfer Index X to Accumulator
    // X -> A
    // N Z C I D V
    // + + - - - -
    fn txa(&mut self, _info: &Info) {
        self.A = self.X;
        self.set_NZ(self.A)
    }

    // TXS Transfer Index X to Stack Register
    // X -> SP
    // N Z C I D V
    // - - - - - -
    fn txs(&mut self, _info: &Info) {
        self.SP = self.X
    }

    // TYA Transfer Index Y to Accumulator
    // Y -> A
    // N Z C I D V
    // + + - - - -
    fn tya(&mut self, _info: &Info) {
        self.A = self.Y;
        self.set_NZ(self.A)
    }

    //////////////////////////////////////////////////////////////////
    //    6502 Extra Instructions                                   //
    //    http://www.ffd2.com/fridge/docs/6502-NMOS.extra.opcodes   //
    //////////////////////////////////////////////////////////////////

    fn ahx(&mut self, info: &Info) {
        unimplemented!()
    }

    fn alr(&mut self, info: &Info) {
        unimplemented!()
    }

    fn anc(&mut self, info: &Info) {
        unimplemented!()
    }

    fn arr(&mut self, info: &Info) {
        unimplemented!()
    }

    fn axs(&mut self, info: &Info) {
        unimplemented!()
    }

    // DCP This opcode DECs the contents of a memory location and then CMPs the result
    // with the A register.
    fn dcp(&mut self, info: &Info) {
        self.dec(info);
        self.cmp(info);
    }

    fn las(&mut self, info: &Info) {
        unimplemented!()
    }

    // LAX This opcode loads both the accumulator and the X register with the contents
    // of a memory location.
    // Sub-instructions: LDA, LDX
    fn lax(&mut self, info: &Info) {
        self.lda(info);
        self.ldx(info)
    }

    // RLA ROLs the contents of a memory location and then ANDs the result with
    // the accumulator.
    // Sub-instructions: ROL, AND
    fn rla(&mut self, info: &Info) {
        self.rol(info);
        self.and(info);
    }

    // RRA RORs the contents of a memory location and then ADCs the result with
    // the accumulator.
    // Sub-instructions: ROR, ADC
    fn rra(&mut self, info: &Info) {
        self.ror(info);
        self.adc(info)
    }

    // SAX ANDs the contents of the A and X registers (leaving the contents of A
    // intact), subtracts an immediate value, and then stores the result in X.
    // ... A few points might be made about the action of subtracting an immediate
    // value.  It actually works just like the CMP instruction, except that CMP
    // does not store the result of the subtraction it performs in any register.
    // This subtract operation is not affected by the state of the Carry flag,
    // though it does affect the Carry flag.  It does not affect the Overflow
    // flag.
    fn sax(&mut self, info: &Info) {
        self.write8(info.addr, self.A & self.X)
    }

    fn sha(&mut self, info: &Info) {
        unimplemented!()
    }

    fn shx(&mut self, info: &Info) {
        unimplemented!()
    }

    fn shy(&mut self, info: &Info) {
        unimplemented!()
    }

    // SLO This opcode ASLs the contents of a memory location and then ORs the result
    // with the accumulator.
    // Sub-instructions:
    // ASL
    // ORA
    fn slo(&mut self, info: &Info) {
        self.asl(info);
        self.ora(info);
    }

    // SRE LSRs the contents of a memory location and then EORs the result with
    // the accumulator.
    // Sub-instructions: LSR, EOR
    fn sre(&mut self, info: &Info) {
        self.lsr(info);
        self.eor(info);
    }

    fn tas(&mut self, info: &Info) {
        unimplemented!()
    }

    fn xaa(&mut self, info: &Info) {
        unimplemented!()
    }

    /////////////////////////////////////////////////////
    //    6502 Undocumented Instructions               //
    //    http://nesdev.com/undocumented_opcodes.txt   //
    /////////////////////////////////////////////////////

    // ISB
    // Increase memory by one, then subtract memory from accu-mulator (with
    // borrow). Status flags: N,V,Z,C
    fn isb(&mut self, info: &Info) {
        self.inc(info);
        self.sbc(info);
    }

    fn kil(&mut self, info: &Info) {
        unimplemented!()
    }

    pub fn init_ins_table(&mut self) {
        self.ins = [
            CPU::brk,
            CPU::ora,
            CPU::kil,
            CPU::slo,
            CPU::nop,
            CPU::ora,
            CPU::asl,
            CPU::slo,
            CPU::php,
            CPU::ora,
            CPU::asl,
            CPU::anc,
            CPU::nop,
            CPU::ora,
            CPU::asl,
            CPU::slo,
            CPU::bpl,
            CPU::ora,
            CPU::kil,
            CPU::slo,
            CPU::nop,
            CPU::ora,
            CPU::asl,
            CPU::slo,
            CPU::clc,
            CPU::ora,
            CPU::nop,
            CPU::slo,
            CPU::nop,
            CPU::ora,
            CPU::asl,
            CPU::slo,
            CPU::jsr,
            CPU::and,
            CPU::kil,
            CPU::rla,
            CPU::bit,
            CPU::and,
            CPU::rol,
            CPU::rla,
            CPU::plp,
            CPU::and,
            CPU::rol,
            CPU::anc,
            CPU::bit,
            CPU::and,
            CPU::rol,
            CPU::rla,
            CPU::bmi,
            CPU::and,
            CPU::kil,
            CPU::rla,
            CPU::nop,
            CPU::and,
            CPU::rol,
            CPU::rla,
            CPU::sec,
            CPU::and,
            CPU::nop,
            CPU::rla,
            CPU::nop,
            CPU::and,
            CPU::rol,
            CPU::rla,
            CPU::rti,
            CPU::eor,
            CPU::kil,
            CPU::sre,
            CPU::nop,
            CPU::eor,
            CPU::lsr,
            CPU::sre,
            CPU::pha,
            CPU::eor,
            CPU::lsr,
            CPU::alr,
            CPU::jmp,
            CPU::eor,
            CPU::lsr,
            CPU::sre,
            CPU::bvc,
            CPU::eor,
            CPU::kil,
            CPU::sre,
            CPU::nop,
            CPU::eor,
            CPU::lsr,
            CPU::sre,
            CPU::cli,
            CPU::eor,
            CPU::nop,
            CPU::sre,
            CPU::nop,
            CPU::eor,
            CPU::lsr,
            CPU::sre,
            CPU::rts,
            CPU::adc,
            CPU::kil,
            CPU::rra,
            CPU::nop,
            CPU::adc,
            CPU::ror,
            CPU::rra,
            CPU::pla,
            CPU::adc,
            CPU::ror,
            CPU::arr,
            CPU::jmp,
            CPU::adc,
            CPU::ror,
            CPU::rra,
            CPU::bvs,
            CPU::adc,
            CPU::kil,
            CPU::rra,
            CPU::nop,
            CPU::adc,
            CPU::ror,
            CPU::rra,
            CPU::sei,
            CPU::adc,
            CPU::nop,
            CPU::rra,
            CPU::nop,
            CPU::adc,
            CPU::ror,
            CPU::rra,
            CPU::nop,
            CPU::sta,
            CPU::nop,
            CPU::sax,
            CPU::sty,
            CPU::sta,
            CPU::stx,
            CPU::sax,
            CPU::dey,
            CPU::nop,
            CPU::txa,
            CPU::xaa,
            CPU::sty,
            CPU::sta,
            CPU::stx,
            CPU::sax,
            CPU::bcc,
            CPU::sta,
            CPU::kil,
            CPU::ahx,
            CPU::sty,
            CPU::sta,
            CPU::stx,
            CPU::sax,
            CPU::tya,
            CPU::sta,
            CPU::txs,
            CPU::tas,
            CPU::shy,
            CPU::sta,
            CPU::shx,
            CPU::ahx,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::lax,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::lax,
            CPU::tay,
            CPU::lda,
            CPU::tax,
            CPU::lax,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::lax,
            CPU::bcs,
            CPU::lda,
            CPU::kil,
            CPU::lax,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::lax,
            CPU::clv,
            CPU::lda,
            CPU::tsx,
            CPU::las,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::lax,
            CPU::cpy,
            CPU::cmp,
            CPU::nop,
            CPU::dcp,
            CPU::cpy,
            CPU::cmp,
            CPU::dec,
            CPU::dcp,
            CPU::iny,
            CPU::cmp,
            CPU::dex,
            CPU::axs,
            CPU::cpy,
            CPU::cmp,
            CPU::dec,
            CPU::dcp,
            CPU::bne,
            CPU::cmp,
            CPU::kil,
            CPU::dcp,
            CPU::nop,
            CPU::cmp,
            CPU::dec,
            CPU::dcp,
            CPU::cld,
            CPU::cmp,
            CPU::nop,
            CPU::dcp,
            CPU::nop,
            CPU::cmp,
            CPU::dec,
            CPU::dcp,
            CPU::cpx,
            CPU::sbc,
            CPU::nop,
            CPU::isb,
            CPU::cpx,
            CPU::sbc,
            CPU::inc,
            CPU::isb,
            CPU::inx,
            CPU::sbc,
            CPU::nop,
            CPU::sbc,
            CPU::cpx,
            CPU::sbc,
            CPU::inc,
            CPU::isb,
            CPU::beq,
            CPU::sbc,
            CPU::kil,
            CPU::isb,
            CPU::nop,
            CPU::sbc,
            CPU::inc,
            CPU::isb,
            CPU::sed,
            CPU::sbc,
            CPU::nop,
            CPU::isb,
            CPU::nop,
            CPU::sbc,
            CPU::inc,
            CPU::isb,
        ]
    }
}

macro_rules! op {
    ($name:expr, $mode:expr, $len:expr, $cycle:expr, $page_cycle:expr) => {
        OP {
            name: $name,
            mode: $mode,
            len: $len,
            cycle: $cycle,
            page_cycle: $page_cycle,
        }
    };
}

// https://www.masswerk.at/6502/6502_instruction_set.html
// Generated by script. Please do not change this file by hand.
const OP_MAP: [OP; 256] = [
    op!("BRK", Mode::Implied, 1, 7, 0),
    op!("ORA", Mode::IndexedIndirect, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("SLO", Mode::IndexedIndirect, 2, 8, 0),
    op!("NOP", Mode::ZeroPage, 2, 3, 0),
    op!("ORA", Mode::ZeroPage, 2, 3, 0),
    op!("ASL", Mode::ZeroPage, 2, 5, 0),
    op!("SLO", Mode::ZeroPage, 2, 5, 0),
    op!("PHP", Mode::Implied, 1, 3, 0),
    op!("ORA", Mode::Immediate, 2, 2, 0),
    op!("ASL", Mode::Accumulator, 1, 2, 0),
    op!("ANC", Mode::Immediate, 0, 2, 0),
    op!("NOP", Mode::Absolute, 3, 4, 0),
    op!("ORA", Mode::Absolute, 3, 4, 0),
    op!("ASL", Mode::Absolute, 3, 6, 0),
    op!("SLO", Mode::Absolute, 3, 6, 0),
    op!("BPL", Mode::Relative, 2, 2, 1),
    op!("ORA", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("SLO", Mode::IndirectIndexed, 2, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("ORA", Mode::ZeroPageX, 2, 4, 0),
    op!("ASL", Mode::ZeroPageX, 2, 6, 0),
    op!("SLO", Mode::ZeroPageX, 2, 6, 0),
    op!("CLC", Mode::Implied, 1, 2, 0),
    op!("ORA", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("SLO", Mode::AbsoluteY, 3, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("ORA", Mode::AbsoluteX, 3, 4, 1),
    op!("ASL", Mode::AbsoluteX, 3, 7, 0),
    op!("SLO", Mode::AbsoluteX, 3, 7, 0),
    op!("JSR", Mode::Absolute, 3, 6, 0),
    op!("AND", Mode::IndexedIndirect, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("RLA", Mode::IndexedIndirect, 2, 8, 0),
    op!("BIT", Mode::ZeroPage, 2, 3, 0),
    op!("AND", Mode::ZeroPage, 2, 3, 0),
    op!("ROL", Mode::ZeroPage, 2, 5, 0),
    op!("RLA", Mode::ZeroPage, 2, 5, 0),
    op!("PLP", Mode::Implied, 1, 4, 0),
    op!("AND", Mode::Immediate, 2, 2, 0),
    op!("ROL", Mode::Accumulator, 1, 2, 0),
    op!("ANC", Mode::Immediate, 0, 2, 0),
    op!("BIT", Mode::Absolute, 3, 4, 0),
    op!("AND", Mode::Absolute, 3, 4, 0),
    op!("ROL", Mode::Absolute, 3, 6, 0),
    op!("RLA", Mode::Absolute, 3, 6, 0),
    op!("BMI", Mode::Relative, 2, 2, 1),
    op!("AND", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("RLA", Mode::IndirectIndexed, 2, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("AND", Mode::ZeroPageX, 2, 4, 0),
    op!("ROL", Mode::ZeroPageX, 2, 6, 0),
    op!("RLA", Mode::ZeroPageX, 2, 6, 0),
    op!("SEC", Mode::Implied, 1, 2, 0),
    op!("AND", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("RLA", Mode::AbsoluteY, 3, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("AND", Mode::AbsoluteX, 3, 4, 1),
    op!("ROL", Mode::AbsoluteX, 3, 7, 0),
    op!("RLA", Mode::AbsoluteX, 3, 7, 0),
    op!("RTI", Mode::Implied, 1, 6, 0),
    op!("EOR", Mode::IndexedIndirect, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("SRE", Mode::IndexedIndirect, 2, 8, 0),
    op!("NOP", Mode::ZeroPage, 2, 3, 0),
    op!("EOR", Mode::ZeroPage, 2, 3, 0),
    op!("LSR", Mode::ZeroPage, 2, 5, 0),
    op!("SRE", Mode::ZeroPage, 2, 5, 0),
    op!("PHA", Mode::Implied, 1, 3, 0),
    op!("EOR", Mode::Immediate, 2, 2, 0),
    op!("LSR", Mode::Accumulator, 1, 2, 0),
    op!("ALR", Mode::Immediate, 0, 2, 0),
    op!("JMP", Mode::Absolute, 3, 3, 0),
    op!("EOR", Mode::Absolute, 3, 4, 0),
    op!("LSR", Mode::Absolute, 3, 6, 0),
    op!("SRE", Mode::Absolute, 3, 6, 0),
    op!("BVC", Mode::Relative, 2, 2, 1),
    op!("EOR", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("SRE", Mode::IndirectIndexed, 2, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("EOR", Mode::ZeroPageX, 2, 4, 0),
    op!("LSR", Mode::ZeroPageX, 2, 6, 0),
    op!("SRE", Mode::ZeroPageX, 2, 6, 0),
    op!("CLI", Mode::Implied, 1, 2, 0),
    op!("EOR", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("SRE", Mode::AbsoluteY, 3, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("EOR", Mode::AbsoluteX, 3, 4, 1),
    op!("LSR", Mode::AbsoluteX, 3, 7, 0),
    op!("SRE", Mode::AbsoluteX, 3, 7, 0),
    op!("RTS", Mode::Implied, 1, 6, 0),
    op!("ADC", Mode::IndexedIndirect, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("RRA", Mode::IndexedIndirect, 2, 8, 0),
    op!("NOP", Mode::ZeroPage, 2, 3, 0),
    op!("ADC", Mode::ZeroPage, 2, 3, 0),
    op!("ROR", Mode::ZeroPage, 2, 5, 0),
    op!("RRA", Mode::ZeroPage, 2, 5, 0),
    op!("PLA", Mode::Implied, 1, 4, 0),
    op!("ADC", Mode::Immediate, 2, 2, 0),
    op!("ROR", Mode::Accumulator, 1, 2, 0),
    op!("ARR", Mode::Immediate, 0, 2, 0),
    op!("JMP", Mode::Indirect, 3, 5, 0),
    op!("ADC", Mode::Absolute, 3, 4, 0),
    op!("ROR", Mode::Absolute, 3, 6, 0),
    op!("RRA", Mode::Absolute, 3, 6, 0),
    op!("BVS", Mode::Relative, 2, 2, 1),
    op!("ADC", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("RRA", Mode::IndirectIndexed, 2, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("ADC", Mode::ZeroPageX, 2, 4, 0),
    op!("ROR", Mode::ZeroPageX, 2, 6, 0),
    op!("RRA", Mode::ZeroPageX, 2, 6, 0),
    op!("SEI", Mode::Implied, 1, 2, 0),
    op!("ADC", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("RRA", Mode::AbsoluteY, 3, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("ADC", Mode::AbsoluteX, 3, 4, 1),
    op!("ROR", Mode::AbsoluteX, 3, 7, 0),
    op!("RRA", Mode::AbsoluteX, 3, 7, 0),
    op!("NOP", Mode::Immediate, 2, 2, 0),
    op!("STA", Mode::IndexedIndirect, 2, 6, 0),
    op!("NOP", Mode::Immediate, 0, 2, 0),
    op!("SAX", Mode::IndexedIndirect, 2, 6, 0),
    op!("STY", Mode::ZeroPage, 2, 3, 0),
    op!("STA", Mode::ZeroPage, 2, 3, 0),
    op!("STX", Mode::ZeroPage, 2, 3, 0),
    op!("SAX", Mode::ZeroPage, 2, 3, 0),
    op!("DEY", Mode::Implied, 1, 2, 0),
    op!("NOP", Mode::Immediate, 0, 2, 0),
    op!("TXA", Mode::Implied, 1, 2, 0),
    op!("XAA", Mode::Immediate, 0, 2, 0),
    op!("STY", Mode::Absolute, 3, 4, 0),
    op!("STA", Mode::Absolute, 3, 4, 0),
    op!("STX", Mode::Absolute, 3, 4, 0),
    op!("SAX", Mode::Absolute, 3, 4, 0),
    op!("BCC", Mode::Relative, 2, 2, 1),
    op!("STA", Mode::IndirectIndexed, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("AHX", Mode::IndirectIndexed, 0, 6, 0),
    op!("STY", Mode::ZeroPageX, 2, 4, 0),
    op!("STA", Mode::ZeroPageX, 2, 4, 0),
    op!("STX", Mode::ZeroPageY, 2, 4, 0),
    op!("SAX", Mode::ZeroPageY, 2, 4, 0),
    op!("TYA", Mode::Implied, 1, 2, 0),
    op!("STA", Mode::AbsoluteY, 3, 5, 0),
    op!("TXS", Mode::Implied, 1, 2, 0),
    op!("TAS", Mode::AbsoluteY, 0, 5, 0),
    op!("SHY", Mode::AbsoluteX, 0, 5, 0),
    op!("STA", Mode::AbsoluteX, 3, 5, 0),
    op!("SHX", Mode::AbsoluteY, 0, 5, 0),
    op!("AHX", Mode::AbsoluteY, 0, 5, 0),
    op!("LDY", Mode::Immediate, 2, 2, 0),
    op!("LDA", Mode::IndexedIndirect, 2, 6, 0),
    op!("LDX", Mode::Immediate, 2, 2, 0),
    op!("LAX", Mode::IndexedIndirect, 2, 6, 0),
    op!("LDY", Mode::ZeroPage, 2, 3, 0),
    op!("LDA", Mode::ZeroPage, 2, 3, 0),
    op!("LDX", Mode::ZeroPage, 2, 3, 0),
    op!("LAX", Mode::ZeroPage, 2, 3, 0),
    op!("TAY", Mode::Implied, 1, 2, 0),
    op!("LDA", Mode::Immediate, 2, 2, 0),
    op!("TAX", Mode::Implied, 1, 2, 0),
    op!("LAX", Mode::Immediate, 2, 2, 0),
    op!("LDY", Mode::Absolute, 3, 4, 0),
    op!("LDA", Mode::Absolute, 3, 4, 0),
    op!("LDX", Mode::Absolute, 3, 4, 0),
    op!("LAX", Mode::Absolute, 3, 4, 0),
    op!("BCS", Mode::Relative, 2, 2, 1),
    op!("LDA", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("LAX", Mode::IndirectIndexed, 2, 5, 1),
    op!("LDY", Mode::ZeroPageX, 2, 4, 0),
    op!("LDA", Mode::ZeroPageX, 2, 4, 0),
    op!("LDX", Mode::ZeroPageY, 2, 4, 0),
    op!("LAX", Mode::ZeroPageY, 2, 4, 0),
    op!("CLV", Mode::Implied, 1, 2, 0),
    op!("LDA", Mode::AbsoluteY, 3, 4, 1),
    op!("TSX", Mode::Implied, 1, 2, 0),
    op!("LAS", Mode::AbsoluteY, 0, 4, 1),
    op!("LDY", Mode::AbsoluteX, 3, 4, 1),
    op!("LDA", Mode::AbsoluteX, 3, 4, 1),
    op!("LDX", Mode::AbsoluteY, 3, 4, 1),
    op!("LAX", Mode::AbsoluteY, 3, 4, 1),
    op!("CPY", Mode::Immediate, 2, 2, 0),
    op!("CMP", Mode::IndexedIndirect, 2, 6, 0),
    op!("NOP", Mode::Immediate, 0, 2, 0),
    op!("DCP", Mode::IndexedIndirect, 2, 8, 0),
    op!("CPY", Mode::ZeroPage, 2, 3, 0),
    op!("CMP", Mode::ZeroPage, 2, 3, 0),
    op!("DEC", Mode::ZeroPage, 2, 5, 0),
    op!("DCP", Mode::ZeroPage, 2, 5, 0),
    op!("INY", Mode::Implied, 1, 2, 0),
    op!("CMP", Mode::Immediate, 2, 2, 0),
    op!("DEX", Mode::Implied, 1, 2, 0),
    op!("AXS", Mode::Immediate, 0, 2, 0),
    op!("CPY", Mode::Absolute, 3, 4, 0),
    op!("CMP", Mode::Absolute, 3, 4, 0),
    op!("DEC", Mode::Absolute, 3, 6, 0),
    op!("DCP", Mode::Absolute, 3, 6, 0),
    op!("BNE", Mode::Relative, 2, 2, 1),
    op!("CMP", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("DCP", Mode::IndirectIndexed, 2, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("CMP", Mode::ZeroPageX, 2, 4, 0),
    op!("DEC", Mode::ZeroPageX, 2, 6, 0),
    op!("DCP", Mode::ZeroPageX, 2, 6, 0),
    op!("CLD", Mode::Implied, 1, 2, 0),
    op!("CMP", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("DCP", Mode::AbsoluteY, 3, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("CMP", Mode::AbsoluteX, 3, 4, 1),
    op!("DEC", Mode::AbsoluteX, 3, 7, 0),
    op!("DCP", Mode::AbsoluteX, 3, 7, 0),
    op!("CPX", Mode::Immediate, 2, 2, 0),
    op!("SBC", Mode::IndexedIndirect, 2, 6, 0),
    op!("NOP", Mode::Immediate, 0, 2, 0),
    op!("ISB", Mode::IndexedIndirect, 2, 8, 0),
    op!("CPX", Mode::ZeroPage, 2, 3, 0),
    op!("SBC", Mode::ZeroPage, 2, 3, 0),
    op!("INC", Mode::ZeroPage, 2, 5, 0),
    op!("ISB", Mode::ZeroPage, 2, 5, 0),
    op!("INX", Mode::Implied, 1, 2, 0),
    op!("SBC", Mode::Immediate, 2, 2, 0),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("SBC", Mode::Immediate, 2, 2, 0),
    op!("CPX", Mode::Absolute, 3, 4, 0),
    op!("SBC", Mode::Absolute, 3, 4, 0),
    op!("INC", Mode::Absolute, 3, 6, 0),
    op!("ISB", Mode::Absolute, 3, 6, 0),
    op!("BEQ", Mode::Relative, 2, 2, 1),
    op!("SBC", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("ISB", Mode::IndirectIndexed, 2, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("SBC", Mode::ZeroPageX, 2, 4, 0),
    op!("INC", Mode::ZeroPageX, 2, 6, 0),
    op!("ISB", Mode::ZeroPageX, 2, 6, 0),
    op!("SED", Mode::Implied, 1, 2, 0),
    op!("SBC", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("ISB", Mode::AbsoluteY, 3, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("SBC", Mode::AbsoluteX, 3, 4, 1),
    op!("INC", Mode::AbsoluteX, 3, 7, 0),
    op!("ISB", Mode::AbsoluteX, 3, 7, 0),
];
