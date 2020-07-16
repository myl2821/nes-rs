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
    // Memory mapper
    mapper: T,

    ram: [u8; 2048],

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

    cycles: u64,
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

pub struct OP<'a> {
    name: &'a str,
    mode: Mode,
    len: u16,
    cycle: u64,
    page_cycle: u64,
}

struct info {
    addr: u16,
    pc: u16,
    mode: Mode,
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
            ram: [0; 2048],
            PC: 0x0000,
            SP: 0x00,
            A: 0x00,
            X: 0x00,
            Y: 0x00,
            P: 0x00,
            ins: [CPU::jmp; 256],
            cycles: 0,
        };
        cpu.init_ins_table();
        cpu.reset();
        cpu
    }

    pub fn read8(&self, addr: u16) -> u8 {
        match addr {
            0..=0x1fff => self.ram[addr as usize],
            0x2000..=0x3fff => todo!(),
            0x4000..=0x4013 => 0,
            0x4014 => todo!(),
            0x4015 => todo!(),
            0x4016 => todo!(),
            0x4017 => todo!(),
            0x4018..=0x401f => 0, // normally disabled, maybe should return Err
            0x4020..=0xffff => self.mapper.read(addr),
        }
    }

    pub fn write8(&mut self, addr: u16, v: u8) {
        match addr {
            0..=0x1fff => self.ram[addr as usize] = v,
            0x2000..=0x3fff => todo!(),
            0x4000..=0x4013 => todo!(),
            0x4014 => todo!(),
            0x4015 => todo!(),
            0x4016 => todo!(),
            0x4017 => todo!(),
            0x4018..=0x401f => todo!(),
            0x4020..=0xffff => self.mapper.write(addr, v),
        }
    }

    pub fn read16(&self, addr: u16) -> u16 {
        let lo = self.read8(addr) as u16;
        let hi = self.read8(addr + 1) as u16;
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
        self.P = 0x24;
        self.cycles = 0;
    }

    pub fn set_PC(&mut self, pc: u16) {
        self.PC = pc
    }

    pub fn set_cycles(&mut self, cycles: u64) {
        self.cycles = cycles
    }

    pub fn run(&mut self) -> u64 {
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

        let info = info {
            addr: addr,
            pc: self.PC,
            mode: op.mode,
        };

        // Execute instruction
        (self.ins[opcode])(self, &info);

        self.cycles - cycles
    }

    // For Debug
    pub fn debug_info(&self) -> String {
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
        return format!(
            "{:04X}  {} {} {}  {} {}\t\t\tA:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} PPU:{:3o},{:3o} CYC:{}",
            self.PC, byte0, byte1, byte2, op.name, op_str, self.A, self.X, self.Y,
            self.P, self.SP, 0, (self.cycles*3)%341,self.cycles
            );
    }

    fn addr(&self, mode: Mode) -> u16 {
        match mode {
            Mode::Absolute => self.read16(self.PC + 1),
            Mode::AbsoluteX => self.read16(self.PC + 1) + self.X as u16,
            Mode::AbsoluteY => self.read16(self.PC + 1) + self.Y as u16,
            Mode::Accumulator => 0,
            Mode::Immediate => self.PC + 1,
            Mode::Implied => 0,
            Mode::IndexedIndirect => self.read16((self.read8(self.PC + 1) + self.X) as u16),
            Mode::Indirect => self.read16(self.read16(self.PC + 1)),
            Mode::IndirectIndexed => self.read16(self.read16(self.PC + 1)) + self.Y as u16,
            Mode::Relative => {
                let offset = self.read8(self.PC + 1);
                if offset < 0x80 {
                    self.PC + 2 + offset as u16
                } else {
                    self.PC + 2 + offset as u16 - 0x100
                }
            }
            Mode::ZeroPage => self.read8(self.PC + 1) as u16,
            Mode::ZeroPageX => (self.read8(self.PC + 1) + self.X) as u16 & 0xff,
            Mode::ZeroPageY => (self.read8(self.PC + 1) + self.Y) as u16 & 0xff,
        }
    }

    fn has_crossed(&self, mode: Mode) -> bool {
        let addr: u16;
        match mode {
            Mode::AbsoluteX => {
                addr = self.read16(self.PC + 1) + self.X as u16;
                self.page_diff(addr - self.X as u16, addr)
            }
            Mode::AbsoluteY => {
                addr = self.read16(self.PC + 1) + self.Y as u16;
                self.page_diff(addr - self.Y as u16, addr)
            }
            Mode::IndirectIndexed => {
                addr = self.read16(self.read16(self.PC + 1)) + self.Y as u16;
                self.page_diff(addr - self.Y as u16, addr)
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
    fn add_branch_cycle(&mut self, info: &info) {
        self.cycles += 1;
        if self.page_diff(self.PC, info.addr) {
            self.cycles += 1
        }
    }

    fn is_flag_0(&self, f: u8) -> bool {
        self.P & f == 0x00
    }

    fn is_flag_1(&self, f: u8) -> bool {
        !self.is_flag_0(f)
    }

    fn set_flag_to_0(&mut self, f: u8) {
        self.P &= !f
    }

    fn set_flag_to_1(&mut self, f: u8) {
        self.P |= f
    }

    fn set_Z(&mut self, v: u8) {
        match v {
            0x00 => self.set_flag_to_1(Z_F),
            _ => self.set_flag_to_0(Z_F),
        }
    }

    fn set_N(&mut self, v: u8) {
        match v & N_F {
            0x00 => self.set_flag_to_0(N_F),
            _ => self.set_flag_to_1(N_F),
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
    fn push(&mut self, v: u8) {
        self.write8(0x0100 | self.SP as u16, v);
        self.SP -= 1
    }

    fn pull(&mut self) -> u8 {
        self.SP += 1;
        self.read8(0x0100 | self.SP as u16)
    }

    fn push16(&mut self, v: u16) {
        self.push((v >> 8) as u8);
        self.push(v as u8)
    }

    fn pull16(&mut self) -> u16 {
        let l = self.pull() as u16;
        let h = self.pull() as u16;
        h << 8 | l
    }

    fn adc(&mut self, info: &info) {
        unimplemented!()
    }
    fn and(&mut self, info: &info) {
        unimplemented!()
    }
    fn asl(&mut self, info: &info) {
        unimplemented!()
    }

    // BCC Branch on Carry Clear
    // branch on C = 0
    // N Z C I D V
    // - - - - - -
    fn bcc(&mut self, info: &info) {
        if self.is_flag_0(C_F) {
            self.PC = info.addr;
            self.add_branch_cycle(info)
        }
    }

    // BCS Branch on Carry Set
    // branch on C = 1
    // N Z C I D V
    // - - - - - -
    fn bcs(&mut self, info: &info) {
        if self.is_flag_1(C_F) {
            self.PC = info.addr;
            self.add_branch_cycle(info)
        }
    }

    fn beq(&mut self, info: &info) {
        unimplemented!()
    }
    fn bit(&mut self, info: &info) {
        unimplemented!()
    }
    fn bmi(&mut self, info: &info) {
        unimplemented!()
    }
    fn bne(&mut self, info: &info) {
        unimplemented!()
    }
    fn bpl(&mut self, info: &info) {
        unimplemented!()
    }
    fn brk(&mut self, info: &info) {
        unimplemented!()
    }
    fn bvc(&mut self, info: &info) {
        unimplemented!()
    }
    fn bvs(&mut self, info: &info) {
        unimplemented!()
    }

    // CLC Clear Carry Flag
    // 0 -> C
    // N Z C I D V
    // - - 0 - - -
    fn clc(&mut self, info: &info) {
        self.set_flag_to_0(C_F)
    }

    fn cld(&mut self, info: &info) {
        unimplemented!()
    }
    fn cli(&mut self, info: &info) {
        unimplemented!()
    }
    fn clv(&mut self, info: &info) {
        unimplemented!()
    }
    fn cmp(&mut self, info: &info) {
        unimplemented!()
    }
    fn cpx(&mut self, info: &info) {
        unimplemented!()
    }
    fn cpy(&mut self, info: &info) {
        unimplemented!()
    }
    fn dec(&mut self, info: &info) {
        unimplemented!()
    }
    fn dex(&mut self, info: &info) {
        unimplemented!()
    }
    fn dey(&mut self, info: &info) {
        unimplemented!()
    }
    fn eor(&mut self, info: &info) {
        unimplemented!()
    }
    fn inc(&mut self, info: &info) {
        unimplemented!()
    }
    fn inx(&mut self, info: &info) {
        unimplemented!()
    }
    fn iny(&mut self, info: &info) {
        unimplemented!()
    }

    // JMP - Jump to New Location
    // (PC+1) -> PCL
    // (PC+2) -> PCH
    // N Z C I D V
    // - - - - - -
    fn jmp(&mut self, info: &info) {
        self.PC = info.addr
    }

    // JSR Jump to New Location Saving Return Address
    // push (PC+2)
    // (PC+1) -> PCL
    // (PC+2) -> PCH
    // N Z C I D V
    // - - - - - -
    fn jsr(&mut self, info: &info) {
        self.push16(self.PC - 1);
        self.PC = info.addr
    }

    // LDA Load Accumulator with Memory
    // M -> A
    // N Z C I D V
    // + + - - - -
    fn lda(&mut self, info: &info) {
        self.A = self.read8(info.addr);
        self.set_NZ(self.A)
    }

    // LDX Load Index X with Memory
    // M -> X
    // N Z C I D V
    // + + - - - -
    fn ldx(&mut self, info: &info) {
        self.X = self.read8(info.addr);
        self.set_NZ(self.X)
    }

    fn ldy(&mut self, info: &info) {
        unimplemented!()
    }
    fn lsr(&mut self, info: &info) {
        unimplemented!()
    }

    // NOP No Operation
    // N Z C I D V
    // - - - - - -
    fn nop(&mut self, info: &info) {}

    fn ora(&mut self, info: &info) {
        unimplemented!()
    }
    fn pha(&mut self, info: &info) {
        unimplemented!()
    }
    fn php(&mut self, info: &info) {
        unimplemented!()
    }
    fn pla(&mut self, info: &info) {
        unimplemented!()
    }
    fn plp(&mut self, info: &info) {
        unimplemented!()
    }
    fn rol(&mut self, info: &info) {
        unimplemented!()
    }
    fn ror(&mut self, info: &info) {
        unimplemented!()
    }
    fn rti(&mut self, info: &info) {
        unimplemented!()
    }
    fn rts(&mut self, info: &info) {
        unimplemented!()
    }
    fn sbc(&mut self, info: &info) {
        unimplemented!()
    }

    // SEC Set Carry Flag
    // 1 -> C
    // N Z C I D V
    // - - 1 - - -
    fn sec(&mut self, info: &info) {
        self.set_flag_to_1(C_F)
    }

    fn sed(&mut self, info: &info) {
        unimplemented!()
    }
    fn sei(&mut self, info: &info) {
        unimplemented!()
    }
    fn sta(&mut self, info: &info) {
        unimplemented!()
    }

    // STX Store Index X in Memory
    // X -> M
    // N Z C I D V
    // - - - - - -
    fn stx(&mut self, info: &info) {
        self.write8(info.addr, self.X)
    }

    fn sty(&mut self, info: &info) {
        unimplemented!()
    }
    fn tax(&mut self, info: &info) {
        unimplemented!()
    }
    fn tay(&mut self, info: &info) {
        unimplemented!()
    }
    fn tsx(&mut self, info: &info) {
        unimplemented!()
    }
    fn txa(&mut self, info: &info) {
        unimplemented!()
    }
    fn txs(&mut self, info: &info) {
        unimplemented!()
    }
    fn tya(&mut self, info: &info) {
        unimplemented!()
    }
    fn err(&mut self, info: &info) {
        unimplemented!()
    }

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
    op!("SLO", Mode::IndexedIndirect, 0, 8, 0),
    op!("NOP", Mode::ZeroPage, 2, 3, 0),
    op!("ORA", Mode::ZeroPage, 2, 3, 0),
    op!("ASL", Mode::ZeroPage, 2, 5, 0),
    op!("SLO", Mode::ZeroPage, 0, 5, 0),
    op!("PHP", Mode::Implied, 1, 3, 0),
    op!("ORA", Mode::Immediate, 2, 2, 0),
    op!("ASL", Mode::Accumulator, 1, 2, 0),
    op!("ANC", Mode::Immediate, 0, 2, 0),
    op!("NOP", Mode::Absolute, 3, 4, 0),
    op!("ORA", Mode::Absolute, 3, 4, 0),
    op!("ASL", Mode::Absolute, 3, 6, 0),
    op!("SLO", Mode::Absolute, 0, 6, 0),
    op!("BPL", Mode::Relative, 2, 2, 1),
    op!("ORA", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("SLO", Mode::IndirectIndexed, 0, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("ORA", Mode::ZeroPageX, 2, 4, 0),
    op!("ASL", Mode::ZeroPageX, 2, 6, 0),
    op!("SLO", Mode::ZeroPageX, 0, 6, 0),
    op!("CLC", Mode::Implied, 1, 2, 0),
    op!("ORA", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("SLO", Mode::AbsoluteY, 0, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("ORA", Mode::AbsoluteX, 3, 4, 1),
    op!("ASL", Mode::AbsoluteX, 3, 7, 0),
    op!("SLO", Mode::AbsoluteX, 0, 7, 0),
    op!("JSR", Mode::Absolute, 3, 6, 0),
    op!("AND", Mode::IndexedIndirect, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("RLA", Mode::IndexedIndirect, 0, 8, 0),
    op!("BIT", Mode::ZeroPage, 2, 3, 0),
    op!("AND", Mode::ZeroPage, 2, 3, 0),
    op!("ROL", Mode::ZeroPage, 2, 5, 0),
    op!("RLA", Mode::ZeroPage, 0, 5, 0),
    op!("PLP", Mode::Implied, 1, 4, 0),
    op!("AND", Mode::Immediate, 2, 2, 0),
    op!("ROL", Mode::Accumulator, 1, 2, 0),
    op!("ANC", Mode::Immediate, 0, 2, 0),
    op!("BIT", Mode::Absolute, 3, 4, 0),
    op!("AND", Mode::Absolute, 3, 4, 0),
    op!("ROL", Mode::Absolute, 3, 6, 0),
    op!("RLA", Mode::Absolute, 0, 6, 0),
    op!("BMI", Mode::Relative, 2, 2, 1),
    op!("AND", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("RLA", Mode::IndirectIndexed, 0, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("AND", Mode::ZeroPageX, 2, 4, 0),
    op!("ROL", Mode::ZeroPageX, 2, 6, 0),
    op!("RLA", Mode::ZeroPageX, 0, 6, 0),
    op!("SEC", Mode::Implied, 1, 2, 0),
    op!("AND", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("RLA", Mode::AbsoluteY, 0, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("AND", Mode::AbsoluteX, 3, 4, 1),
    op!("ROL", Mode::AbsoluteX, 3, 7, 0),
    op!("RLA", Mode::AbsoluteX, 0, 7, 0),
    op!("RTI", Mode::Implied, 1, 6, 0),
    op!("EOR", Mode::IndexedIndirect, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("SRE", Mode::IndexedIndirect, 0, 8, 0),
    op!("NOP", Mode::ZeroPage, 2, 3, 0),
    op!("EOR", Mode::ZeroPage, 2, 3, 0),
    op!("LSR", Mode::ZeroPage, 2, 5, 0),
    op!("SRE", Mode::ZeroPage, 0, 5, 0),
    op!("PHA", Mode::Implied, 1, 3, 0),
    op!("EOR", Mode::Immediate, 2, 2, 0),
    op!("LSR", Mode::Accumulator, 1, 2, 0),
    op!("ALR", Mode::Immediate, 0, 2, 0),
    op!("JMP", Mode::Absolute, 3, 3, 0),
    op!("EOR", Mode::Absolute, 3, 4, 0),
    op!("LSR", Mode::Absolute, 3, 6, 0),
    op!("SRE", Mode::Absolute, 0, 6, 0),
    op!("BVC", Mode::Relative, 2, 2, 1),
    op!("EOR", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("SRE", Mode::IndirectIndexed, 0, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("EOR", Mode::ZeroPageX, 2, 4, 0),
    op!("LSR", Mode::ZeroPageX, 2, 6, 0),
    op!("SRE", Mode::ZeroPageX, 0, 6, 0),
    op!("CLI", Mode::Implied, 1, 2, 0),
    op!("EOR", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("SRE", Mode::AbsoluteY, 0, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("EOR", Mode::AbsoluteX, 3, 4, 1),
    op!("LSR", Mode::AbsoluteX, 3, 7, 0),
    op!("SRE", Mode::AbsoluteX, 0, 7, 0),
    op!("RTS", Mode::Implied, 1, 6, 0),
    op!("ADC", Mode::IndexedIndirect, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("RRA", Mode::IndexedIndirect, 0, 8, 0),
    op!("NOP", Mode::ZeroPage, 2, 3, 0),
    op!("ADC", Mode::ZeroPage, 2, 3, 0),
    op!("ROR", Mode::ZeroPage, 2, 5, 0),
    op!("RRA", Mode::ZeroPage, 0, 5, 0),
    op!("PLA", Mode::Implied, 1, 4, 0),
    op!("ADC", Mode::Immediate, 2, 2, 0),
    op!("ROR", Mode::Accumulator, 1, 2, 0),
    op!("ARR", Mode::Immediate, 0, 2, 0),
    op!("JMP", Mode::Indirect, 3, 5, 0),
    op!("ADC", Mode::Absolute, 3, 4, 0),
    op!("ROR", Mode::Absolute, 3, 6, 0),
    op!("RRA", Mode::Absolute, 0, 6, 0),
    op!("BVS", Mode::Relative, 2, 2, 1),
    op!("ADC", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("RRA", Mode::IndirectIndexed, 0, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("ADC", Mode::ZeroPageX, 2, 4, 0),
    op!("ROR", Mode::ZeroPageX, 2, 6, 0),
    op!("RRA", Mode::ZeroPageX, 0, 6, 0),
    op!("SEI", Mode::Implied, 1, 2, 0),
    op!("ADC", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("RRA", Mode::AbsoluteY, 0, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("ADC", Mode::AbsoluteX, 3, 4, 1),
    op!("ROR", Mode::AbsoluteX, 3, 7, 0),
    op!("RRA", Mode::AbsoluteX, 0, 7, 0),
    op!("NOP", Mode::Immediate, 2, 2, 0),
    op!("STA", Mode::IndexedIndirect, 2, 6, 0),
    op!("NOP", Mode::Immediate, 0, 2, 0),
    op!("SAX", Mode::IndexedIndirect, 0, 6, 0),
    op!("STY", Mode::ZeroPage, 2, 3, 0),
    op!("STA", Mode::ZeroPage, 2, 3, 0),
    op!("STX", Mode::ZeroPage, 2, 3, 0),
    op!("SAX", Mode::ZeroPage, 0, 3, 0),
    op!("DEY", Mode::Implied, 1, 2, 0),
    op!("NOP", Mode::Immediate, 0, 2, 0),
    op!("TXA", Mode::Implied, 1, 2, 0),
    op!("XAA", Mode::Immediate, 0, 2, 0),
    op!("STY", Mode::Absolute, 3, 4, 0),
    op!("STA", Mode::Absolute, 3, 4, 0),
    op!("STX", Mode::Absolute, 3, 4, 0),
    op!("SAX", Mode::Absolute, 0, 4, 0),
    op!("BCC", Mode::Relative, 2, 2, 1),
    op!("STA", Mode::IndirectIndexed, 2, 6, 0),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("AHX", Mode::IndirectIndexed, 0, 6, 0),
    op!("STY", Mode::ZeroPageX, 2, 4, 0),
    op!("STA", Mode::ZeroPageX, 2, 4, 0),
    op!("STX", Mode::ZeroPageY, 2, 4, 0),
    op!("SAX", Mode::ZeroPageY, 0, 4, 0),
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
    op!("LAX", Mode::IndexedIndirect, 0, 6, 0),
    op!("LDY", Mode::ZeroPage, 2, 3, 0),
    op!("LDA", Mode::ZeroPage, 2, 3, 0),
    op!("LDX", Mode::ZeroPage, 2, 3, 0),
    op!("LAX", Mode::ZeroPage, 0, 3, 0),
    op!("TAY", Mode::Implied, 1, 2, 0),
    op!("LDA", Mode::Immediate, 2, 2, 0),
    op!("TAX", Mode::Implied, 1, 2, 0),
    op!("LAX", Mode::Immediate, 0, 2, 0),
    op!("LDY", Mode::Absolute, 3, 4, 0),
    op!("LDA", Mode::Absolute, 3, 4, 0),
    op!("LDX", Mode::Absolute, 3, 4, 0),
    op!("LAX", Mode::Absolute, 0, 4, 0),
    op!("BCS", Mode::Relative, 2, 2, 1),
    op!("LDA", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("LAX", Mode::IndirectIndexed, 0, 5, 1),
    op!("LDY", Mode::ZeroPageX, 2, 4, 0),
    op!("LDA", Mode::ZeroPageX, 2, 4, 0),
    op!("LDX", Mode::ZeroPageY, 2, 4, 0),
    op!("LAX", Mode::ZeroPageY, 0, 4, 0),
    op!("CLV", Mode::Implied, 1, 2, 0),
    op!("LDA", Mode::AbsoluteY, 3, 4, 1),
    op!("TSX", Mode::Implied, 1, 2, 0),
    op!("LAS", Mode::AbsoluteY, 0, 4, 1),
    op!("LDY", Mode::AbsoluteX, 3, 4, 1),
    op!("LDA", Mode::AbsoluteX, 3, 4, 1),
    op!("LDX", Mode::AbsoluteY, 3, 4, 1),
    op!("LAX", Mode::AbsoluteY, 0, 4, 1),
    op!("CPY", Mode::Immediate, 2, 2, 0),
    op!("CMP", Mode::IndexedIndirect, 2, 6, 0),
    op!("NOP", Mode::Immediate, 0, 2, 0),
    op!("DCP", Mode::IndexedIndirect, 0, 8, 0),
    op!("CPY", Mode::ZeroPage, 2, 3, 0),
    op!("CMP", Mode::ZeroPage, 2, 3, 0),
    op!("DEC", Mode::ZeroPage, 2, 5, 0),
    op!("DCP", Mode::ZeroPage, 0, 5, 0),
    op!("INY", Mode::Implied, 1, 2, 0),
    op!("CMP", Mode::Immediate, 2, 2, 0),
    op!("DEX", Mode::Implied, 1, 2, 0),
    op!("AXS", Mode::Immediate, 0, 2, 0),
    op!("CPY", Mode::Absolute, 3, 4, 0),
    op!("CMP", Mode::Absolute, 3, 4, 0),
    op!("DEC", Mode::Absolute, 3, 6, 0),
    op!("DCP", Mode::Absolute, 0, 6, 0),
    op!("BNE", Mode::Relative, 2, 2, 1),
    op!("CMP", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("DCP", Mode::IndirectIndexed, 0, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("CMP", Mode::ZeroPageX, 2, 4, 0),
    op!("DEC", Mode::ZeroPageX, 2, 6, 0),
    op!("DCP", Mode::ZeroPageX, 0, 6, 0),
    op!("CLD", Mode::Implied, 1, 2, 0),
    op!("CMP", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("DCP", Mode::AbsoluteY, 0, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("CMP", Mode::AbsoluteX, 3, 4, 1),
    op!("DEC", Mode::AbsoluteX, 3, 7, 0),
    op!("DCP", Mode::AbsoluteX, 0, 7, 0),
    op!("CPX", Mode::Immediate, 2, 2, 0),
    op!("SBC", Mode::IndexedIndirect, 2, 6, 0),
    op!("NOP", Mode::Immediate, 0, 2, 0),
    op!("ISC", Mode::IndexedIndirect, 0, 8, 0),
    op!("CPX", Mode::ZeroPage, 2, 3, 0),
    op!("SBC", Mode::ZeroPage, 2, 3, 0),
    op!("INC", Mode::ZeroPage, 2, 5, 0),
    op!("ISC", Mode::ZeroPage, 0, 5, 0),
    op!("INX", Mode::Implied, 1, 2, 0),
    op!("SBC", Mode::Immediate, 2, 2, 0),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("SBC", Mode::Immediate, 0, 2, 0),
    op!("CPX", Mode::Absolute, 3, 4, 0),
    op!("SBC", Mode::Absolute, 3, 4, 0),
    op!("INC", Mode::Absolute, 3, 6, 0),
    op!("ISC", Mode::Absolute, 0, 6, 0),
    op!("BEQ", Mode::Relative, 2, 2, 1),
    op!("SBC", Mode::IndirectIndexed, 2, 5, 1),
    op!("KIL", Mode::Implied, 0, 2, 0),
    op!("ISC", Mode::IndirectIndexed, 0, 8, 0),
    op!("NOP", Mode::ZeroPageX, 2, 4, 0),
    op!("SBC", Mode::ZeroPageX, 2, 4, 0),
    op!("INC", Mode::ZeroPageX, 2, 6, 0),
    op!("ISC", Mode::ZeroPageX, 0, 6, 0),
    op!("SED", Mode::Implied, 1, 2, 0),
    op!("SBC", Mode::AbsoluteY, 3, 4, 1),
    op!("NOP", Mode::Implied, 1, 2, 0),
    op!("ISC", Mode::AbsoluteY, 0, 7, 0),
    op!("NOP", Mode::AbsoluteX, 3, 4, 1),
    op!("SBC", Mode::AbsoluteX, 3, 4, 1),
    op!("INC", Mode::AbsoluteX, 3, 7, 0),
    op!("ISC", Mode::AbsoluteX, 0, 7, 0),
];
