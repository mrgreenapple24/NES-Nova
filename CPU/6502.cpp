#include "6502.h"
#include "Bus.h"
#include <cstdint>
cpu6502::cpu6502() {
    using a = cpu6502;
    lookup = {
        #include "6502_opcodes.inl" // whatever you do here, dont look inside
    };
}

cpu6502::~cpu6502() {
    // just chillin
}

// Reads an 8-bit byte from the bus, located at the specified 16-bit address
uint8_t cpu6502::read(uint16_t addr) {
    return bus->read(addr, false);
}

// Writes a byte to the bus at the specified address
void cpu6502::write(uint16_t addr, uint8_t data) {
    bus -> write(addr, data);
}

// push data onto stack
void cpu6502::stck_push(uint8_t data) {
    write(0x0100 + stkp--, data);
}

// pull data from stack
uint8_t cpu6502::stck_pull() {
    return read(0x0100 + ++stkp);
}

// helper function for branch instructions
void cpu6502::branch() {
    cycles++;
    addr_abs = addr_rel + pc;
    if ((addr_abs & 0xFF00) != (pc & 0xFF00)) {
        cycles++;
    }
    pc = addr_abs;
}

// Forces the 6502 into a known state. This is hard-wired inside the CPU. The
// registers are set to 0x00, the status register is cleared except for unused
// bit which remains at 1. An absolute address is read from location 0xFFFC
// which contains a second address that the program counter is set to. This
// allows the programmer to jump to a known and programmable location in the
// memory to start executing from. Typically the programmer would set the value
// at location 0xFFFC at compile time.
void cpu6502::reset()
{
	// Get address to set program counter to
	addr_abs = 0xFFFC;
	uint16_t lo = read(addr_abs + 0);
	uint16_t hi = read(addr_abs + 1);

	// Set it
	pc = (hi << 8) | lo;

	// Reset internal registers
	a = 0;
	x = 0;
	y = 0;
	stkp = 0xFD;
	status = 0x00 | U;

	// Clear internal helper variables
	addr_rel = 0x0000;
	addr_abs = 0x0000;
	fetched = 0x00;

	// Reset takes 8 clk cycles to complete
	cycles = 8;
}

// Interrupt requests are a complex operation and only happen if the
// "disable interrupt" flag is 0. IRQs can happen at any time, but
// you dont want them to be destructive to the operation of the running
// program. Therefore the current instruction is allowed to finish
// (which I facilitate by doing the whole thing when cycles == 0) and
// then the current program counter is stored on the stack. Then the
// current status register is stored on the stack. When the routine
// that services the interrupt has finished, the status register
// and program counter can be restored to how they where before it
// occurred. This is impemented by the "RTI" instruction. Once the IRQ
// has happened, in a similar way to a reset, a programmable address
// is read form hard coded location 0xFFFE, which is subsequently
// set to the program counter.
void cpu6502::irq()
{
	// If interrupts are allowed
	if (GetFlag(I) == 0)
	{
		// Push the program counter to the stack. It's 16-bits dont
		// forget so that takes two pushes
		stck_push((pc >> 8) & 0x00FF);
		stck_push(pc & 0x00FF);

		// Then Push the status register to the stack
		SetFlag(B, 0);
		SetFlag(U, 1);
		SetFlag(I, 1);
		stck_push(status);

		// Read new program counter location from fixed address
		addr_abs = 0xFFFE;
		uint16_t lo = read(addr_abs + 0);
		uint16_t hi = read(addr_abs + 1);
		pc = (hi << 8) | lo;

		// IRQs take time
		cycles = 7;
	}
}

// It behaves in exactly the same way as a regular IRQ,
// but reads the new program counter address
// form location 0xFFFA.
void cpu6502::nmi()
{
	write(0x0100 + stkp, (pc >> 8) & 0x00FF);
	stkp--;
	write(0x0100 + stkp, pc & 0x00FF);
	stkp--;

	SetFlag(B, 0);
	SetFlag(U, 1);
	SetFlag(I, 1);
	write(0x0100 + stkp, status);
	stkp--;

	addr_abs = 0xFFFA;
	uint16_t lo = read(addr_abs + 0);
	uint16_t hi = read(addr_abs + 1);
	pc = (hi << 8) | lo;

	cycles = 8;
}

// Perform one clock cycles worth of emulation
void cpu6502::clock()
{
	// Each instruction requires a variable number of clock cycles to execute.
	// To remain compliant with connected devices, it's important that the
	// emulation also takes "time" in order to execute instructions, so I
	// implement that delay by simply counting down the cycles required by
	// the instruction. When it reaches 0, the instruction is complete, and
	// the next one is ready to be executed.
	if (cycles == 0)
	{
		// Read next instruction byte. This 8-bit value is used to index
		// the translation table to get the relevant information about
		// how to implement the instruction
		opcode = read(pc);

#ifdef LOGMODE
		uint16_t log_pc = pc;
#endif

		// Always set the unused status flag bit to 1
		SetFlag(U, true);

		// Increment program counter, we read the opcode byte
		pc++;

		// Get Starting number of cycles
		cycles = lookup[opcode].cycles;

		// Perform fetch of intermmediate data using the
		// required addressing mode
		uint8_t additional_cycle1 = (this->*lookup[opcode].addrmode)();

		// Perform operation
		uint8_t additional_cycle2 = (this->*lookup[opcode].operate)();

		// The addressmode and opcode may have altered the number
		// of cycles this instruction requires before its completed
		cycles += (additional_cycle1 & additional_cycle2);

		// Always set the unused status flag bit to 1
		SetFlag(U, true);

#ifdef LOGMODE
		// This logger dumps every cycle the entire processor state for analysis.
		// This can be used for debugging the emulation, but has little utility
		// during emulation. Its also very slow, so only use if you have to.
		if (logfile == nullptr)	logfile = fopen("olc6502.txt", "wt");
		if (logfile != nullptr)
		{
			fprintf(logfile, "%10d:%02d PC:%04X %s A:%02X X:%02X Y:%02X %s%s%s%s%s%s%s%s STKP:%02X\n",
				clock_count, 0, log_pc, "XXX", a, x, y,
				GetFlag(N) ? "N" : ".",	GetFlag(V) ? "V" : ".",	GetFlag(U) ? "U" : ".",
				GetFlag(B) ? "B" : ".",	GetFlag(D) ? "D" : ".",	GetFlag(I) ? "I" : ".",
				GetFlag(Z) ? "Z" : ".",	GetFlag(C) ? "C" : ".",	stkp);
		}
#endif
	}

	// Increment global clock count - This is actually unused unless logging is enabled
	// but I've kept it in because its a handy watch variable for debugging
	clock_count++;

	// Decrement the number of cycles remaining for this instruction
	cycles--;
}

// Returns the value of a specific bit of the status register
uint8_t cpu6502::GetFlag(FLAGS6502 f)
{
	return ((status & f) > 0) ? 1 : 0;
}

// Sets or clears a specific bit of the status register
void cpu6502::SetFlag(FLAGS6502 f, bool v)
{
	if (v)
		status |= f;
	else
		status &= ~f;
}

// ADDRESSING MODES

// Instructions like RTS or CLC have no address operand, the destination of results are implied.
// There is no additional data required for this instruction.
uint8_t cpu6502::IMP() {
    fetched = a;
    return 0;
}

// Uses the 8-bit operand itself as the value for the operation, rather than fetching a value from a memory address.
void cpu6502::IMM() {
    addr_abs = pc++;
    // The instruction expects the next byte to be used as a value, so we'll prep
    // the read address to point to the next byte
}

// Fetches the value from an 8-bit address on the zero page.
// Address Mode: Zero Page
// To save program bytes, zero page addressing allows you to absolutely address
// a location in first 0xFF bytes of address range. Clearly this only requires
// one byte instead of the usual two.
void cpu6502::ZP0() {
    addr_abs = read(pc);
    pc++;
    addr_abs &= 0x00FF;
}

// Zero Page with X offset
void cpu6502::ZPX() {
    addr_abs = read(pc) + x;
    pc++;
    addr_abs &= 0x00FF;
}

// Zero Page with Y offset
void cpu6502::ZPY() {
    addr_abs = read(pc) + y;
    pc++;
    addr_abs &= 0x00FF;
}

// Address Mode: Relative
// This address mode is exclusive to branch instructions. The address
// must reside within -128 to +127 of the branch instruction, i.e.
// you cant directly branch to any address in the addressable range.
void cpu6502::REL() {
    addr_rel = read(pc);
    pc++;
    if (addr_rel & 0x80) {
        addr_rel |= 0xFF00;
    }
}

// Fetches the value from a 16-bit address anywhere in memory.
void cpu6502::ABS() {
    uint16_t low = read(pc);
    pc++;
    uint16_t high = read(pc);
    pc++;

    addr_abs = (high << 8) | low;
}

// Address Mode: Absolute with X Offset
// Fundamentally the same as absolute addressing, but the contents of the X Register
// is added to the supplied two byte address. If the resulting address changes
// the page, an additional clock cycle is required
bool cpu6502::ABX() {
    uint16_t low = read(pc);
    pc++;
    uint16_t high = read(pc);
    pc++;

    addr_abs = (high << 8) | low;
    addr_abs += x;

    if ((addr_abs & 0xFF00) != (high << 8))
        return true;
    else return false;
}

// Address Mode: Absolute with Y Offset
// Fundamentally the same as absolute addressing, but the contents of the Y Register
// is added to the supplied two byte address. If the resulting address changes
// the page, an additional clock cycle is required
bool cpu6502::ABY() {
    uint16_t low = read(pc);
    pc++;
    uint16_t high = read(pc);
    pc++;

    addr_abs = (high << 8) | low;
    addr_abs += y;

    if ((addr_abs & 0xFF00) != (high << 8))
        return true;
    else return false;
}

// Address Mode: Indirect
// The supplied 16-bit address is read to get the actual 16-bit address. This is
// instruction is unusual in that it has a bug in the hardware! To emulate its
// function accurately, we also need to emulate this bug. If the low byte of the
// supplied address is 0xFF, then to read the high byte of the actual address
// we need to cross a page boundary. This doesnt actually work on the chip as
// designed, instead it wraps back around in the same page, yielding an
// invalid actual address
void cpu6502::IND() {
    uint16_t ptr_lo = read(pc);
	pc++;
	uint16_t ptr_hi = read(pc);
	pc++;

	uint16_t ptr = (ptr_hi << 8) | ptr_lo;

	if (ptr_lo == 0x00FF) // Simulate page boundary hardware bug
	{
		addr_abs = (read(ptr & 0xFF00) << 8) | read(ptr + 0);
	}
	else // Behave normally
	{
		addr_abs = (read(ptr + 1) << 8) | read(ptr + 0);
	}
}

// Address Mode: Indirect X
// The supplied 8-bit address is offset by X Register to index
// a location in page 0x00. The actual 16-bit address is read
// from this location
void cpu6502::IZX() {
    uint16_t t = read(pc);
	pc++;

	uint16_t lo = read((uint16_t)(t + (uint16_t)x) & 0x00FF);
	uint16_t hi = read((uint16_t)(t + (uint16_t)x + 1) & 0x00FF);

	addr_abs = (hi << 8) | lo;
}

// Address Mode: Indirect Y
// The supplied 8-bit address indexes a location in page 0x00. From
// here the actual 16-bit address is read, and the contents of
// Y Register is added to it to offset it. If the offset causes a
// change in page then an additional clock cycle is required.
bool cpu6502::IZY() {
    uint16_t t = read(pc);
	pc++;

	uint16_t lo = read(t & 0x00FF);
	uint16_t hi = read((t + 1) & 0x00FF);

	addr_abs = (hi << 8) | lo;
	addr_abs += y;

	if ((addr_abs & 0xFF00) != (hi << 8))
		return true;
	else
		return false;
}

// This function sources the data used by the instruction into
// a convenient numeric variable. Some instructions dont have to
// fetch data as the source is implied by the instruction. For example
// "INX" increments the X register. There is no additional data
// required. For all other addressing modes, the data resides at
// the location held within addr_abs, so it is read from there.
// Immediate adress mode exploits this slightly, as that has
// set addr_abs = pc + 1, so it fetches the data from the
// next byte for example "LDA $FF" just loads the accumulator with
// 256, i.e. no far reaching memory fetch is required. "fetched"
// is a variable global to the CPU, and is set by calling this
// function. It also returns it for convenience.
uint8_t cpu6502::fetch()
{
	if (!(lookup[opcode].addrmode == &cpu6502::IMP))
		fetched = read(addr_abs);
	return fetched;
}


// OPCODES

uint8_t cpu6502::ADC() {
    // Grab the data that we are adding to the accumulator
	fetch();

	// Add is performed in 16-bit domain for emulation to capture any
	// carry bit, which will exist in bit 8 of the 16-bit word
	temp = (uint16_t)a + (uint16_t)fetched + (uint16_t)GetFlag(C);

	// The carry flag out exists in the high byte bit 0
	SetFlag(C, temp > 255);

	// The Zero flag is set if the result is 0
	SetFlag(Z, (temp & 0x00FF) == 0);

	// The signed Overflow flag is set based on all that up there! :D
	SetFlag(V, (~((uint16_t)a ^ (uint16_t)fetched) & ((uint16_t)a ^ (uint16_t)temp)) & 0x0080);

	// The negative flag is set to the most significant bit of the result
	SetFlag(N, temp & 0x80);

	// Load the result into the accumulator (it's 8-bit dont forget!)
	a = temp & 0x00FF;

	// This instruction has the potential to require an additional clock cycle
	return 1;
}

// A = A & memory
// This ANDs a memory value and the accumulator, bit by bit.
// If both input bits are 1, the resulting bit is 1. Otherwise, it is 0.

uint8_t cpu6502::AND() {
    fetch();
    a &= fetched;
    SetFlag(Z, a==0);
    SetFlag(N, a & 0x80);
    return 1;
}

// value = value << 1, or visually: C <- [76543210] <- 0
// ASL shifts all of the bits of a memory value or the accumulator one position to the left,
// moving the value of each bit into the next bit. Bit 7 is shifted into the carry flag, and 0 is shifted into bit 0.
// This is equivalent to multiplying an unsigned value by 2, with carry indicating overflow.

uint8_t cpu6502::ASL() {
    fetch();
    uint16_t temp = fetched << 1;
    SetFlag(C, (temp & fetched) > 0);
    SetFlag(Z, (temp & 0x00FF) == 0);
    SetFlag(N, temp & 0x80);

    if (lookup[opcode].addrmode == &cpu6502::IMP)
        a = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
}

// If the carry flag is clear, BCC branches to a nearby location
// by adding the relative offset to the program counter.
// The offset is signed and has a range of [-128, 127] relative to
// the first byte after the branch instruction
uint8_t cpu6502::BCC() {
    if (!GetFlag(C)) {
        branch();
    }
    return 0;
}

// just branches if carry flag is set
uint8_t cpu6502::BCS() {
    if (GetFlag(C)) {
        branch();
    }
    return 0;
}

// Branch if zero flag is set
uint8_t cpu6502::BEQ() {
    if (GetFlag(Z)) {
        branch();
    }
    return 0;
}

// just sets the flags no real change in memory or registers
uint8_t cpu6502::BIT() {
    fetch();
    temp = fetched & a;

    SetFlag(Z, (temp & 0x00FF) == 0);
    SetFlag(N, temp & 0x80);
    SetFlag(V, temp & 0x40);
    return 0;
}

// branch if negative flag is set
uint8_t cpu6502::BMI() {
    if (GetFlag(N)) {
        branch();
    }
    return 0;
}

// branch if zero flag is clear
uint8_t cpu6502::BNE() {
    if (!GetFlag(Z)) {
        branch();
    }
    return 0;
}

// branch if negative flag is clear
uint8_t cpu6502::BPL() {
    if (!GetFlag(N)) {
        branch();
    }
    return 0;
}

// BRK triggers an interrupt request (IRQ).
// IRQs are normally triggered by external hardware,
// and BRK is the only way to do it in software. Like a typical IRQ,
// it pushes the current program counter and processor flags to the stack,
// sets the interrupt disable flag, and jumps to the IRQ handler.
uint8_t cpu6502::BRK() {
    pc++;
    SetFlag(I, 1);
    stck_push((pc >> 8) & 0x00FF);
    stck_push(pc & 0x00FF);
    stck_push(status | (1 << 4));
    pc = (uint16_t)read(0xFFFE) | ((uint16_t)read(0xFFFF) << 8);
    return 0;
}

// branch if overflow flag is clear
uint8_t cpu6502::BVC() {
    if (!GetFlag(V)) {
        cycles++;
        addr_abs = addr_rel + pc;
        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) {
            cycles++;
        }
        pc = addr_abs;
    }
    return 0;
}

// branch if overflow flag is set
uint8_t cpu6502::BVS() {
    if (GetFlag(V)) {
        cycles++;
        addr_abs = addr_rel + pc;
        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) {
            cycles++;
        }
        pc = addr_abs;
    }
    return 0;
}

// clears carry flag
uint8_t cpu6502::CLC() {
    SetFlag(C, 0);
    return 0;
}

// clears decimal flag
uint8_t cpu6502::CLD() {
    SetFlag(D, 0);
    return 0;
}

// clears interrupt flag
uint8_t cpu6502::CLI() {
    SetFlag(I, 0);
    return 0;
}

// clears overflow flag
uint8_t cpu6502::CLV() {
    SetFlag(V, 0);
    return 0;
}

// compares accumulator with memory
uint8_t cpu6502::CMP() {
    fetch();
    SetFlag(C, (a >= fetched));
    SetFlag(Z, (a == fetched));
    SetFlag(N, (a - fetched) & 0x80);
    return 0;
}

// compares register x with memory
uint8_t cpu6502::CPX() {
    fetch();
    SetFlag(C, (x >= fetched));
    SetFlag(Z, (x == fetched));
    SetFlag(N, (x - fetched) & 0x80);
    return 0;
}

// compares register y with memoryl
uint8_t cpu6502::CPY() {
    fetch();
    SetFlag(C, (y >= fetched));
    SetFlag(Z, (y == fetched));
    SetFlag(N, (y - fetched) & 0x80);
    return 0;
}

// decrements memory
uint8_t cpu6502::DEC() {
    fetch();
    temp = fetched - 1;
    SetFlag(Z, (temp == 0));
    SetFlag(N, (temp & 0x0080));
    write(addr_abs, temp & 0x00FF);
    return 0;
}

// decrements x
uint8_t cpu6502::DEX() {
    x--;
    SetFlag(Z, (x == 0));
    SetFlag(N, (x & 0x80));
    return 0;
}

// decrements y
uint8_t cpu6502::DEY() {
    y--;
    SetFlag(Z, (y == 0));
    SetFlag(N, (y & 0x80));
    return 0;
}

// exclusive or memory with accumulator
uint8_t cpu6502::EOR() {
    fetch();
    a ^= fetched;
    SetFlag(Z, (a == 0));
    SetFlag(N, (a & 0x80));
    return 0;
}

// increments memory
uint8_t cpu6502::INC() {
    fetch();
    temp = fetched + 1;
    SetFlag(Z, (temp == 0));
    SetFlag(N, (temp & 0x0080));
    write(addr_abs, temp & 0x00FF);
    return 0;
}

// increments x
uint8_t cpu6502::INX() {
    x++;
    SetFlag(Z, (x == 0));
    SetFlag(N, (x & 0x80));
    return 0;
}

// increments y
uint8_t cpu6502::INY() {
    y++;
    SetFlag(Z, (y == 0));
    SetFlag(N, (y & 0x80));
    return 0;
}

// jump to address
uint8_t cpu6502::JMP() {
    pc = addr_abs;
    return 0;
}

// jump to subroutine
uint8_t cpu6502::JSR() {
    stck_push(pc >> 8);
    stck_push(pc & 0xFF);
    pc = addr_abs;
    return 0;
}

// load accumulator with memory
uint8_t cpu6502::LDA() {
    fetch();
    a = fetched;
    SetFlag(Z, (a == 0));
    SetFlag(N, (a & 0x80));
    return 0;
}

// load register x with memory
uint8_t cpu6502::LDX() {
    fetch();
    x = fetched;
    SetFlag(Z, (x == 0));
    SetFlag(N, (x & 0x80));
    return 0;
}

// load register y with memory
uint8_t cpu6502::LDY() {
    fetch();
    y = fetched;
    SetFlag(Z, (y == 0));
    SetFlag(N, (y & 0x80));
    return 0;
}

// logical shift right
uint8_t cpu6502::LSR() {
    fetch();
	SetFlag(C, fetched & 0x0001);
	temp = fetched >> 1;
	SetFlag(Z, (temp & 0x00FF) == 0x0000);
	SetFlag(N, temp & 0x0080);

	if (lookup[opcode].addrmode == &cpu6502::IMP)
		a = temp & 0x00FF;
	else
		write(addr_abs, temp & 0x00FF);
    return 0;
}

// no operation
uint8_t cpu6502::NOP() {
    return 0;
}

// OR accumulator with memory
uint8_t cpu6502::ORA() {
    fetch();
    a |= fetched;
    SetFlag(Z, (a == 0));
    SetFlag(N, (a & 0x80));
    return 0;
}

// push accumulator onto stack
uint8_t cpu6502::PHA() {
    stck_push(a);
    return 0;
}

// push status register onto stack
uint8_t cpu6502::PHP() {
    stck_push(status | (1<<4) | (1<<5));
    return 0;
}

// pull accumulator from stack
uint8_t cpu6502::PLA() {
    a = stck_pull();
    SetFlag(Z, (a == 0));
    SetFlag(N, (a & 0x80));
    return 0;
}

// pull status register from stack
uint8_t cpu6502::PLP() {
    status = stck_pull();
    SetFlag(C, status & (1<<0));
    SetFlag(Z, status & (1<<1));
    SetFlag(I, status & (1<<2));
    SetFlag(D, status & (1<<3));
    SetFlag(V, status & (1<<6));
    SetFlag(N, status & (1<<7));
    return 0;
}

// rotate left
uint8_t cpu6502::ROL() {
    fetch();
	temp = (uint16_t)(fetched << 1) | GetFlag(C);
	SetFlag(C, temp & 0xFF00);
	SetFlag(Z, (temp & 0x00FF) == 0x0000);
	SetFlag(N, temp & 0x0080);
	if (lookup[opcode].addrmode == &cpu6502::IMP)
		a = temp & 0x00FF;
	else
		write(addr_abs, temp & 0x00FF);
	return 0;
}

// rotate right
uint8_t cpu6502::ROR() {
    fetch();
	temp = (uint16_t)(fetched >> 1) | (GetFlag(C) << 7);
	SetFlag(C, fetched & 0x01);
	SetFlag(Z, (temp & 0x00FF) == 0x0000);
	SetFlag(N, temp & 0x0080);
	if (lookup[opcode].addrmode == &cpu6502::IMP)
		a = temp & 0x00FF;
	else
		write(addr_abs, temp & 0x00FF);
	return 0;
}

// Return from interrupt
uint8_t cpu6502::RTI() {
    temp = stck_pull();
    SetFlag(C, temp & (1<<0));
    SetFlag(Z, temp & (1<<1));
    SetFlag(I, temp & (1<<2));
    SetFlag(D, temp & (1<<3));
    SetFlag(V, temp & (1<<6));
    SetFlag(N, temp & (1<<7));

    pc = stck_pull();
    pc |= (uint16_t)stck_pull() << 8;
    return 0;
}

// Return from subroutine
uint8_t cpu6502::RTS() {
    pc = stck_pull();
    pc |= (uint16_t)stck_pull() << 8;
    pc++;
    return 0;
}

// subtract with carry
uint8_t cpu6502::SBC()
{
	fetch();

	// Operating in 16-bit domain to capture carry out

	// We can invert the bottom 8 bits with bitwise xor
	uint16_t value = ((uint16_t)fetched) ^ 0x00FF;

	// Notice this is exactly the same as addition from here!
	temp = (uint16_t)a + value + (uint16_t)GetFlag(C);
	SetFlag(C, temp & 0xFF00);
	SetFlag(Z, ((temp & 0x00FF) == 0));
	SetFlag(V, (temp ^ (uint16_t)a) & (temp ^ value) & 0x0080);
	SetFlag(N, temp & 0x0080);
	a = temp & 0x00FF;
	return 1;
}

// set carry flag
uint8_t cpu6502::SEC()
{
	SetFlag(C, 1);
	return 1;
}

// set decimal flag
uint8_t cpu6502::SED()
{
	SetFlag(D, 1);
	return 1;
}

// set interrupt flag
uint8_t cpu6502::SEI()
{
	SetFlag(I, 1);
	return 1;
}

// Store accumulator in memory
uint8_t cpu6502::STA() {
    write(addr_abs, a);
    return 0;
}

uint8_t cpu6502::STX() {
    write(addr_abs, x);
    return 0;
}
uint8_t cpu6502::STY() {
    write(addr_abs, y);
    return 0;
}
uint8_t cpu6502::TAX() {
    a = x;
    SetFlag(Z, a == 0);
    SetFlag(N, a & 0x80);
    return 0;
}
uint8_t cpu6502::TAY() {
    a = y;
    SetFlag(Z, a == 0);
    SetFlag(N, a & 0x80);
    return 0;
}
uint8_t cpu6502::TSX() {
    x = stkp;
    SetFlag(Z, x == 0);
    SetFlag(N, x & 0x80);
    return 0;
}
uint8_t cpu6502::TXA() {
    x = a;
    SetFlag(Z, x == 0);
    SetFlag(N, x & 0x80);
    return 0;
}
uint8_t cpu6502::TXS() {
    stkp = x;
    return 0;
}
uint8_t cpu6502::TYA() {
    a = y;
    SetFlag(Z, a == 0);
    SetFlag(N, a & 0x80);
    return 0;
}

uint8_t cpu6502::XXX() {
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
/*
bool cpu6502::complete()
{
	return cycles == 0;
}

// This is the disassembly function. Its workings are not required for emulation.
// It is merely a convenience function to turn the binary instruction code into
// human readable form. Its included as part of the emulator because it can take
// advantage of many of the CPUs internal operations to do this.
std::map<uint16_t, std::string> cpu6502::disassemble(uint16_t nStart, uint16_t nStop)
{
	uint32_t addr = nStart;
	uint8_t value = 0x00, lo = 0x00, hi = 0x00;
	std::map<uint16_t, std::string> mapLines;
	uint16_t line_addr = 0;

	// A convenient utility to convert variables into
	// hex strings because "modern C++"'s method with
	// streams is atrocious
	auto hex = [](uint32_t n, uint8_t d)
	{
		std::string s(d, '0');
		for (int i = d - 1; i >= 0; i--, n >>= 4)
			s[i] = "0123456789ABCDEF"[n & 0xF];
		return s;
	};

	// Starting at the specified address we read an instruction
	// byte, which in turn yields information from the lookup table
	// as to how many additional bytes we need to read and what the
	// addressing mode is. I need this info to assemble human readable
	// syntax, which is different depending upon the addressing mode

	// As the instruction is decoded, a std::string is assembled
	// with the readable output
	while (addr <= (uint32_t)nStop)
	{
		line_addr = addr;

		// Prefix line with instruction address
		std::string sInst = "$" + hex(addr, 4) + ": ";

		// Read instruction, and get its readable name
		uint8_t opcode = bus->read(addr, true); addr++;
		sInst += lookup[opcode].name + " ";

		// Get oprands from desired locations, and form the
		// instruction based upon its addressing mode. These
		// routines mimmick the actual fetch routine of the
		// 6502 in order to get accurate data as part of the
		// instruction
		if (lookup[opcode].addrmode == &cpu6502::IMP)
		{
			sInst += " {IMP}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::IMM)
		{
			value = bus->read(addr, true); addr++;
			sInst += "#$" + hex(value, 2) + " {IMM}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::ZP0)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "$" + hex(lo, 2) + " {ZP0}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::ZPX)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "$" + hex(lo, 2) + ", X {ZPX}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::ZPY)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "$" + hex(lo, 2) + ", Y {ZPY}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::IZX)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "($" + hex(lo, 2) + ", X) {IZX}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::IZY)
		{
			lo = bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "($" + hex(lo, 2) + "), Y {IZY}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::ABS)
		{
			lo = bus->read(addr, true); addr++;
			hi = bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + " {ABS}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::ABX)
		{
			lo = bus->read(addr, true); addr++;
			hi = bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", X {ABX}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::ABY)
		{
			lo = bus->read(addr, true); addr++;
			hi = bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", Y {ABY}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::IND)
		{
			lo = bus->read(addr, true); addr++;
			hi = bus->read(addr, true); addr++;
			sInst += "($" + hex((uint16_t)(hi << 8) | lo, 4) + ") {IND}";
		}
		else if (lookup[opcode].addrmode == &cpu6502::REL)
		{
			value = bus->read(addr, true); addr++;
			sInst += "$" + hex(value, 2) + " [$" + hex(addr + value, 4) + "] {REL}";
		}

		// Add the formed string to a std::map, using the instruction's
		// address as the key. This makes it convenient to look for later
		// as the instructions are variable in length, so a straight up
		// incremental index is not sufficient.
		mapLines[line_addr] = sInst;
	}

	return mapLines;
}

*/
