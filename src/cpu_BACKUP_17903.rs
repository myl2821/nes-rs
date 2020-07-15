use crate::{Mapper, Result};

const VERCTOR_NMI: u16 = 0xFFFA;
const VERCTOR_RST: u16 = 0xFFFC;
const VERCTOR_IRQ: u16 = 0xFFFE;

// Carry Flag (C)
const C_F: u8 = 0x01;
// Zero Flag (Z)
const Z_F: u8 = 0x02;
// Interrupt Disable (I)
const I_F: u8 = 0x04;
// Decimal Mode (D)
const D_F: u8 = 0x08;
// Break Command (B)
const B_F: u8 = 0x10;
// Overflow Flag (V)
const V_F: u8 = 0x40;
// Negative Flag (N)
const N_F: u8 = 0x80;

pub struct CPU<T: Mapper> {
<<<<<<< HEAD
    mapper: T,
=======
    // Memory mapper
    mapper: T,

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
    // Overflow Flag (V)
    // Negative Flag (N)
    // Status register layout
    // ---------------------------------
    //   7 | 6 | 5 | 4 | 3 | 2 | 1 | 0
    //   N | V |   | B | D | I | Z | C
    P: u8,

    // Instruction function table
    ins: [fn(&mut CPU<T>, &info); 256],
}

// Addressing modes
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

pub struct OP<'a> {
    name: &'a str,
    mode: Mode,
    len: u16,
}

struct info {
    addr: u16,
    pc: u16,
    mode: Mode,
>>>>>>> feat: basic cpu impl
}

// CPU Memory map
// -------------------------------------------------------------------------------------------------
// Address range |  Size   |  Device
// $0000-$07FF	 |  $0800  |  2KB internal RAM
// $0800-$0FFF	 |  $0800  |  Mirrors of $0000-$07FF
// $1000-$17FF	 |  $0800  |  Mirrors of $0000-$07FF
// $1800-$1FFF	 |  $0800  |  Mirrors of $0000-$07FF
// $2000-$2007	 |  $0008  |  NES PPU registers
// $2008-$3FFF	 |  $1FF8  |  Mirrors of $2000-2007 (repeats every 8 bytes)
// $4000-$4017	 |  $0018  |  NES APU and I/O registers
// $4018-$401F	 |  $0008  |  APU and I/O functionality that is normally disabled. See CPU Test Mode
// $4020-$FFFF	 |  $BFE0  |  Cartridge space: PRG ROM, PRG RAM, and mapper registers (See Note)
impl<T: Mapper> CPU<T> {
    pub fn new(mapper: T) -> Self {
        let mut cpu = CPU {
            mapper: mapper,
            PC: 0x0000,
            SP: 0x00,
            A: 0x00,
            X: 0x00,
            Y: 0x00,
            P: 0x00,
            ins: [CPU::jmp; 256],
        };
        cpu.init_ins_table();
        cpu.reset();
        cpu
    }

    pub fn read(&self, addr: u16) -> Result<u8> {
        match addr {
            0..=0x1fff => todo!(),
            0x2000..=0x3fff => todo!(),
            0x4000..=0x4013 => Ok(0),
            0x4014 => todo!(),
            0x4015 => todo!(),
            0x4016 => todo!(),
            0x4017 => todo!(),
            0x4018..=0x401f => Ok(0), // normally disabled, maybe should return Err
            0x4020..=0xffff => self.mapper.read(addr),
        }
    }

    pub fn read16(&self, addr: u16) -> Result<u16> {
        let lo = self.read(addr)? as u16;
        let hi = self.read(addr + 1)? as u16;
        Ok(lo | hi << 8)
    }

    pub fn nmi(&self) -> Result<u16> {
        self.read16(VERCTOR_NMI)
    }

    pub fn rst(&self) -> Result<u16> {
        self.read16(VERCTOR_RST)
    }

    pub fn irq(&self) -> Result<u16> {
        self.read16(VERCTOR_IRQ)
    }
<<<<<<< HEAD
}
=======

    pub fn reset(&mut self) {
        self.PC = self.rst().unwrap();
        self.SP = 0xfd;
        self.P = 0x24;
    }

    pub fn run(&mut self) {
        // TODO: Detect interrupts

        // Read instruction
        let opcode = self.read(self.PC).unwrap() as usize;
        let op = &OP_MAP[opcode];

        // Get address by different addressing modes
        let addr = self.addr(op.mode);

        self.PC += op.len;

        let info = info {
            addr: addr,
            pc: self.PC,
            mode: op.mode,
        };

        // Execute instruction
        (self.ins[opcode])(self, &info);
    }

    fn addr(&self, mode: Mode) -> u16 {
        match mode {
            Mode::Absolute => self.read16(self.PC + 1).unwrap(),
            Mode::AbsoluteX => self.read16(self.PC + 1).unwrap() + self.X as u16,
            Mode::AbsoluteY => self.read16(self.PC + 1).unwrap() + self.Y as u16,
            Mode::Accumulator => 0,
            Mode::Immediate => self.PC + 1,
            Mode::Implied => 0,
            Mode::IndexedIndirect => self
                .read16((self.read(self.PC + 1).unwrap() + self.X) as u16)
                .unwrap(),
            Mode::Indirect => self.read16(self.read16(self.PC + 1).unwrap()).unwrap(),
            Mode::IndirectIndexed => {
                self.read16(self.read16(self.PC + 1).unwrap()).unwrap() + self.Y as u16
            }
            Mode::Relative => {
                let offset = self.read(self.PC + 1).unwrap();
                if offset < 0x80 {
                    self.PC + 2 + offset as u16
                } else {
                    self.PC + 2 + offset as u16 - 0x100
                }
            }
            Mode::ZeroPage => self.read(self.PC + 1).unwrap() as u16,
            Mode::ZeroPageX => (self.read(self.PC + 1).unwrap() + self.X) as u16 & 0xff,
            Mode::ZeroPageY => (self.read(self.PC + 1).unwrap() + self.Y) as u16 & 0xff,
        }
    }

    // Just for test
    pub fn jmp_c000(&mut self) {
        self.PC = 0xc000
    }

    // For Debug
    pub fn debug_info(&self) -> String {
        let opcode = self.read(self.PC).unwrap() as usize;
        let op = &OP_MAP[opcode];
        let addr = self.addr(op.mode);

        let byte0 = format!("{:02X}", self.read(self.PC).unwrap());
        let mut byte1 = format!("{:02X}", self.read(self.PC + 1).unwrap());
        let mut byte2 = format!("{:02X}", self.read(self.PC + 2).unwrap());

        if op.len < 3 {
            byte2 = "  ".to_string()
        }
        if op.len < 2 {
            byte1 = "  ".to_string()
        }

        // FIXME
        return format!(
            "{:04X}  {} {} {}  {} ${:04X}\tA:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X}",
            self.PC, byte0, byte1, byte2, op.name, addr, self.A, self.X, self.Y, self.P, self.SP
        );
    }

    fn set_flag_to_0(&mut self, f: u8) {
        self.P &= !f
    }

    fn set_flag_to_1(&mut self, f: u8) {
        self.P |= f
    }

    fn set_flag(&mut self, f: u8, v: u8) {
        match v {
            0x00 => self.set_flag_to_1(f),
            _ => self.set_flag_to_0(f),
        }
    }

    fn set_Z(&mut self, v: u8) {
        self.set_flag(Z_F, v)
    }

    fn set_N(&mut self, v: u8) {
        self.set_flag(N_F, v & N_F)
    }

    fn set_NZ(&mut self, v: u8) {
        self.set_N(v);
        self.set_Z(v);
    }

    fn adc(&mut self, info: &info) {}
    fn and(&mut self, info: &info) {}
    fn asl(&mut self, info: &info) {}
    fn bcc(&mut self, info: &info) {}
    fn bcs(&mut self, info: &info) {}
    fn beq(&mut self, info: &info) {}
    fn bit(&mut self, info: &info) {}
    fn bmi(&mut self, info: &info) {}
    fn bne(&mut self, info: &info) {}
    fn bpl(&mut self, info: &info) {}
    fn brk(&mut self, info: &info) {}
    fn bvc(&mut self, info: &info) {}
    fn bvs(&mut self, info: &info) {}
    fn clc(&mut self, info: &info) {}
    fn cld(&mut self, info: &info) {}
    fn cli(&mut self, info: &info) {}
    fn clv(&mut self, info: &info) {}
    fn cmp(&mut self, info: &info) {}
    fn cpx(&mut self, info: &info) {}
    fn cpy(&mut self, info: &info) {}
    fn dec(&mut self, info: &info) {}
    fn dex(&mut self, info: &info) {}
    fn dey(&mut self, info: &info) {}
    fn eor(&mut self, info: &info) {}
    fn inc(&mut self, info: &info) {}
    fn inx(&mut self, info: &info) {}
    fn iny(&mut self, info: &info) {}

    // JMP - Jump to New Location
    // N Z C I D V
    // - - - - - -
    fn jmp(&mut self, info: &info) {
        self.PC = info.addr
    }
    fn jsr(&mut self, info: &info) {}
    fn lda(&mut self, info: &info) {}

    // LDX Load Index X with Memory
    // M -> X
    // N Z C I D V
    // + + - - - -
    fn ldx(&mut self, info: &info) {
        self.X = self.read(info.addr).unwrap();
        self.set_NZ(self.X)
    }

    fn ldy(&mut self, info: &info) {}
    fn lsr(&mut self, info: &info) {}
    fn nop(&mut self, info: &info) {}
    fn ora(&mut self, info: &info) {}
    fn pha(&mut self, info: &info) {}
    fn php(&mut self, info: &info) {}
    fn pla(&mut self, info: &info) {}
    fn plp(&mut self, info: &info) {}
    fn rol(&mut self, info: &info) {}
    fn ror(&mut self, info: &info) {}
    fn rti(&mut self, info: &info) {}
    fn rts(&mut self, info: &info) {}
    fn sbc(&mut self, info: &info) {}
    fn sec(&mut self, info: &info) {}
    fn sed(&mut self, info: &info) {}
    fn sei(&mut self, info: &info) {}
    fn sta(&mut self, info: &info) {}
    fn stx(&mut self, info: &info) {}
    fn sty(&mut self, info: &info) {}
    fn tax(&mut self, info: &info) {}
    fn tay(&mut self, info: &info) {}
    fn tsx(&mut self, info: &info) {}
    fn txa(&mut self, info: &info) {}
    fn txs(&mut self, info: &info) {}
    fn tya(&mut self, info: &info) {}
    fn err(&mut self, info: &info) {}

    pub fn init_ins_table(&mut self) {
        self.ins = [
            CPU::brk,
            CPU::ora,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::ora,
            CPU::asl,
            CPU::err,
            CPU::php,
            CPU::ora,
            CPU::asl,
            CPU::err,
            CPU::nop,
            CPU::ora,
            CPU::asl,
            CPU::err,
            CPU::bpl,
            CPU::ora,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::ora,
            CPU::asl,
            CPU::err,
            CPU::clc,
            CPU::ora,
            CPU::nop,
            CPU::err,
            CPU::nop,
            CPU::ora,
            CPU::asl,
            CPU::err,
            CPU::jsr,
            CPU::and,
            CPU::err,
            CPU::err,
            CPU::bit,
            CPU::and,
            CPU::rol,
            CPU::err,
            CPU::plp,
            CPU::and,
            CPU::rol,
            CPU::err,
            CPU::bit,
            CPU::and,
            CPU::rol,
            CPU::err,
            CPU::bmi,
            CPU::and,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::and,
            CPU::rol,
            CPU::err,
            CPU::sec,
            CPU::and,
            CPU::nop,
            CPU::err,
            CPU::nop,
            CPU::and,
            CPU::rol,
            CPU::err,
            CPU::rti,
            CPU::eor,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::eor,
            CPU::lsr,
            CPU::err,
            CPU::pha,
            CPU::eor,
            CPU::lsr,
            CPU::err,
            CPU::jmp,
            CPU::eor,
            CPU::lsr,
            CPU::err,
            CPU::bvc,
            CPU::eor,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::eor,
            CPU::lsr,
            CPU::err,
            CPU::cli,
            CPU::eor,
            CPU::nop,
            CPU::err,
            CPU::nop,
            CPU::eor,
            CPU::lsr,
            CPU::err,
            CPU::rts,
            CPU::adc,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::adc,
            CPU::ror,
            CPU::err,
            CPU::pla,
            CPU::adc,
            CPU::ror,
            CPU::err,
            CPU::jmp,
            CPU::adc,
            CPU::ror,
            CPU::err,
            CPU::bvs,
            CPU::adc,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::adc,
            CPU::ror,
            CPU::err,
            CPU::sei,
            CPU::adc,
            CPU::nop,
            CPU::err,
            CPU::nop,
            CPU::adc,
            CPU::ror,
            CPU::err,
            CPU::nop,
            CPU::sta,
            CPU::nop,
            CPU::err,
            CPU::sty,
            CPU::sta,
            CPU::stx,
            CPU::err,
            CPU::dey,
            CPU::nop,
            CPU::txa,
            CPU::err,
            CPU::sty,
            CPU::sta,
            CPU::stx,
            CPU::err,
            CPU::bcc,
            CPU::sta,
            CPU::err,
            CPU::err,
            CPU::sty,
            CPU::sta,
            CPU::stx,
            CPU::err,
            CPU::tya,
            CPU::sta,
            CPU::txs,
            CPU::err,
            CPU::err,
            CPU::sta,
            CPU::err,
            CPU::err,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::err,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::err,
            CPU::tay,
            CPU::lda,
            CPU::tax,
            CPU::err,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::err,
            CPU::bcs,
            CPU::lda,
            CPU::err,
            CPU::err,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::err,
            CPU::clv,
            CPU::lda,
            CPU::tsx,
            CPU::err,
            CPU::ldy,
            CPU::lda,
            CPU::ldx,
            CPU::err,
            CPU::cpy,
            CPU::cmp,
            CPU::nop,
            CPU::err,
            CPU::cpy,
            CPU::cmp,
            CPU::dec,
            CPU::err,
            CPU::iny,
            CPU::cmp,
            CPU::dex,
            CPU::err,
            CPU::cpy,
            CPU::cmp,
            CPU::dec,
            CPU::err,
            CPU::bne,
            CPU::cmp,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::cmp,
            CPU::dec,
            CPU::err,
            CPU::cld,
            CPU::cmp,
            CPU::nop,
            CPU::err,
            CPU::nop,
            CPU::cmp,
            CPU::dec,
            CPU::err,
            CPU::cpx,
            CPU::sbc,
            CPU::nop,
            CPU::err,
            CPU::cpx,
            CPU::sbc,
            CPU::inc,
            CPU::err,
            CPU::inx,
            CPU::sbc,
            CPU::nop,
            CPU::sbc,
            CPU::cpx,
            CPU::sbc,
            CPU::inc,
            CPU::err,
            CPU::beq,
            CPU::sbc,
            CPU::err,
            CPU::err,
            CPU::nop,
            CPU::sbc,
            CPU::inc,
            CPU::err,
            CPU::sed,
            CPU::sbc,
            CPU::nop,
            CPU::err,
            CPU::nop,
            CPU::sbc,
            CPU::inc,
            CPU::err,
        ]
    }
}

// https://www.masswerk.at/6502/6502_instruction_set.html
// Generated by script. Please do not change this file by hand.
const OP_MAP: [OP; 256] = [
    OP {
        name: "BRK",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "ORA",
        mode: Mode::IndexedIndirect,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "SLO",
        mode: Mode::IndexedIndirect,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "ORA",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "ASL",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "SLO",
        mode: Mode::ZeroPage,
        len: 0,
    },
    OP {
        name: "PHP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "ORA",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "ASL",
        mode: Mode::Accumulator,
        len: 1,
    },
    OP {
        name: "ANC",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "ORA",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "ASL",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "SLO",
        mode: Mode::Absolute,
        len: 0,
    },
    OP {
        name: "BPL",
        mode: Mode::Relative,
        len: 2,
    },
    OP {
        name: "ORA",
        mode: Mode::IndirectIndexed,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "SLO",
        mode: Mode::IndirectIndexed,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "ORA",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "ASL",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "SLO",
        mode: Mode::ZeroPageX,
        len: 0,
    },
    OP {
        name: "CLC",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "ORA",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "NOP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "SLO",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "ORA",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "ASL",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "SLO",
        mode: Mode::AbsoluteX,
        len: 0,
    },
    OP {
        name: "JSR",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "AND",
        mode: Mode::IndexedIndirect,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "RLA",
        mode: Mode::IndexedIndirect,
        len: 0,
    },
    OP {
        name: "BIT",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "AND",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "ROL",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "RLA",
        mode: Mode::ZeroPage,
        len: 0,
    },
    OP {
        name: "PLP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "AND",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "ROL",
        mode: Mode::Accumulator,
        len: 1,
    },
    OP {
        name: "ANC",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "BIT",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "AND",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "ROL",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "RLA",
        mode: Mode::Absolute,
        len: 0,
    },
    OP {
        name: "BMI",
        mode: Mode::Relative,
        len: 2,
    },
    OP {
        name: "AND",
        mode: Mode::IndirectIndexed,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "RLA",
        mode: Mode::IndirectIndexed,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "AND",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "ROL",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "RLA",
        mode: Mode::ZeroPageX,
        len: 0,
    },
    OP {
        name: "SEC",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "AND",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "NOP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "RLA",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "AND",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "ROL",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "RLA",
        mode: Mode::AbsoluteX,
        len: 0,
    },
    OP {
        name: "RTI",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "EOR",
        mode: Mode::IndexedIndirect,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "SRE",
        mode: Mode::IndexedIndirect,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "EOR",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "LSR",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "SRE",
        mode: Mode::ZeroPage,
        len: 0,
    },
    OP {
        name: "PHA",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "EOR",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "LSR",
        mode: Mode::Accumulator,
        len: 1,
    },
    OP {
        name: "ALR",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "JMP",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "EOR",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "LSR",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "SRE",
        mode: Mode::Absolute,
        len: 0,
    },
    OP {
        name: "BVC",
        mode: Mode::Relative,
        len: 2,
    },
    OP {
        name: "EOR",
        mode: Mode::IndirectIndexed,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "SRE",
        mode: Mode::IndirectIndexed,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "EOR",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "LSR",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "SRE",
        mode: Mode::ZeroPageX,
        len: 0,
    },
    OP {
        name: "CLI",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "EOR",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "NOP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "SRE",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "EOR",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "LSR",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "SRE",
        mode: Mode::AbsoluteX,
        len: 0,
    },
    OP {
        name: "RTS",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "ADC",
        mode: Mode::IndexedIndirect,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "RRA",
        mode: Mode::IndexedIndirect,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "ADC",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "ROR",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "RRA",
        mode: Mode::ZeroPage,
        len: 0,
    },
    OP {
        name: "PLA",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "ADC",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "ROR",
        mode: Mode::Accumulator,
        len: 1,
    },
    OP {
        name: "ARR",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "JMP",
        mode: Mode::Indirect,
        len: 3,
    },
    OP {
        name: "ADC",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "ROR",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "RRA",
        mode: Mode::Absolute,
        len: 0,
    },
    OP {
        name: "BVS",
        mode: Mode::Relative,
        len: 2,
    },
    OP {
        name: "ADC",
        mode: Mode::IndirectIndexed,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "RRA",
        mode: Mode::IndirectIndexed,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "ADC",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "ROR",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "RRA",
        mode: Mode::ZeroPageX,
        len: 0,
    },
    OP {
        name: "SEI",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "ADC",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "NOP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "RRA",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "ADC",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "ROR",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "RRA",
        mode: Mode::AbsoluteX,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "STA",
        mode: Mode::IndexedIndirect,
        len: 2,
    },
    OP {
        name: "NOP",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "SAX",
        mode: Mode::IndexedIndirect,
        len: 0,
    },
    OP {
        name: "STY",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "STA",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "STX",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "SAX",
        mode: Mode::ZeroPage,
        len: 0,
    },
    OP {
        name: "DEY",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "NOP",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "TXA",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "XAA",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "STY",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "STA",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "STX",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "SAX",
        mode: Mode::Absolute,
        len: 0,
    },
    OP {
        name: "BCC",
        mode: Mode::Relative,
        len: 2,
    },
    OP {
        name: "STA",
        mode: Mode::IndirectIndexed,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "AHX",
        mode: Mode::IndirectIndexed,
        len: 0,
    },
    OP {
        name: "STY",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "STA",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "STX",
        mode: Mode::ZeroPageY,
        len: 2,
    },
    OP {
        name: "SAX",
        mode: Mode::ZeroPageY,
        len: 0,
    },
    OP {
        name: "TYA",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "STA",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "TXS",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "TAS",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "SHY",
        mode: Mode::AbsoluteX,
        len: 0,
    },
    OP {
        name: "STA",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "SHX",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "AHX",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "LDY",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "LDA",
        mode: Mode::IndexedIndirect,
        len: 2,
    },
    OP {
        name: "LDX",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "LAX",
        mode: Mode::IndexedIndirect,
        len: 0,
    },
    OP {
        name: "LDY",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "LDA",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "LDX",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "LAX",
        mode: Mode::ZeroPage,
        len: 0,
    },
    OP {
        name: "TAY",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "LDA",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "TAX",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "LAX",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "LDY",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "LDA",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "LDX",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "LAX",
        mode: Mode::Absolute,
        len: 0,
    },
    OP {
        name: "BCS",
        mode: Mode::Relative,
        len: 2,
    },
    OP {
        name: "LDA",
        mode: Mode::IndirectIndexed,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "LAX",
        mode: Mode::IndirectIndexed,
        len: 0,
    },
    OP {
        name: "LDY",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "LDA",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "LDX",
        mode: Mode::ZeroPageY,
        len: 2,
    },
    OP {
        name: "LAX",
        mode: Mode::ZeroPageY,
        len: 0,
    },
    OP {
        name: "CLV",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "LDA",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "TSX",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "LAS",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "LDY",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "LDA",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "LDX",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "LAX",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "CPY",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "CMP",
        mode: Mode::IndexedIndirect,
        len: 2,
    },
    OP {
        name: "NOP",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "DCP",
        mode: Mode::IndexedIndirect,
        len: 0,
    },
    OP {
        name: "CPY",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "CMP",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "DEC",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "DCP",
        mode: Mode::ZeroPage,
        len: 0,
    },
    OP {
        name: "INY",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "CMP",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "DEX",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "AXS",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "CPY",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "CMP",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "DEC",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "DCP",
        mode: Mode::Absolute,
        len: 0,
    },
    OP {
        name: "BNE",
        mode: Mode::Relative,
        len: 2,
    },
    OP {
        name: "CMP",
        mode: Mode::IndirectIndexed,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "DCP",
        mode: Mode::IndirectIndexed,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "CMP",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "DEC",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "DCP",
        mode: Mode::ZeroPageX,
        len: 0,
    },
    OP {
        name: "CLD",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "CMP",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "NOP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "DCP",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "CMP",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "DEC",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "DCP",
        mode: Mode::AbsoluteX,
        len: 0,
    },
    OP {
        name: "CPX",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "SBC",
        mode: Mode::IndexedIndirect,
        len: 2,
    },
    OP {
        name: "NOP",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "ISC",
        mode: Mode::IndexedIndirect,
        len: 0,
    },
    OP {
        name: "CPX",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "SBC",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "INC",
        mode: Mode::ZeroPage,
        len: 2,
    },
    OP {
        name: "ISC",
        mode: Mode::ZeroPage,
        len: 0,
    },
    OP {
        name: "INX",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "SBC",
        mode: Mode::Immediate,
        len: 2,
    },
    OP {
        name: "NOP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "SBC",
        mode: Mode::Immediate,
        len: 0,
    },
    OP {
        name: "CPX",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "SBC",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "INC",
        mode: Mode::Absolute,
        len: 3,
    },
    OP {
        name: "ISC",
        mode: Mode::Absolute,
        len: 0,
    },
    OP {
        name: "BEQ",
        mode: Mode::Relative,
        len: 2,
    },
    OP {
        name: "SBC",
        mode: Mode::IndirectIndexed,
        len: 2,
    },
    OP {
        name: "KIL",
        mode: Mode::Implied,
        len: 0,
    },
    OP {
        name: "ISC",
        mode: Mode::IndirectIndexed,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "SBC",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "INC",
        mode: Mode::ZeroPageX,
        len: 2,
    },
    OP {
        name: "ISC",
        mode: Mode::ZeroPageX,
        len: 0,
    },
    OP {
        name: "SED",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "SBC",
        mode: Mode::AbsoluteY,
        len: 3,
    },
    OP {
        name: "NOP",
        mode: Mode::Implied,
        len: 1,
    },
    OP {
        name: "ISC",
        mode: Mode::AbsoluteY,
        len: 0,
    },
    OP {
        name: "NOP",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "SBC",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "INC",
        mode: Mode::AbsoluteX,
        len: 3,
    },
    OP {
        name: "ISC",
        mode: Mode::AbsoluteX,
        len: 0,
    },
];
>>>>>>> feat: basic cpu impl
