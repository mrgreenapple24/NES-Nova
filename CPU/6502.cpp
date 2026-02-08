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
		write(0x0100 + stkp, (pc >> 8) & 0x00FF);
		stkp--;
		write(0x0100 + stkp, pc & 0x00FF);
		stkp--;

		// Then Push the status register to the stack
		SetFlag(B, 0);
		SetFlag(U, 1);
		SetFlag(I, 1);
		write(0x0100 + stkp, status);
		stkp--;

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
uint8_t cpu6502::AND() {
    return 0;
}
uint8_t cpu6502::ASL() {
    return 0;
}
uint8_t cpu6502::BCC() {
    return 0;
}
uint8_t cpu6502::BCS() {
    return 0;
}
uint8_t cpu6502::BEQ() {
    return 0;
}
uint8_t cpu6502::BIT() {
    return 0;
}
uint8_t cpu6502::BMI() {
    return 0;
}
uint8_t cpu6502::BNE() {
    return 0;
}
uint8_t cpu6502::BPL() {
    return 0;
}
uint8_t cpu6502::BRK() {
    return 0;
}
uint8_t cpu6502::BVC() {
    return 0;
}
uint8_t cpu6502::BVS() {
    return 0;
}
uint8_t cpu6502::CLC() {
    return 0;
}
uint8_t cpu6502::CLD() {
    return 0;
}
uint8_t cpu6502::CLI() {
    return 0;
}
uint8_t cpu6502::CLV() {
    return 0;
}
uint8_t cpu6502::CMP() {
    return 0;
}
uint8_t cpu6502::CPX() {
    return 0;
}
uint8_t cpu6502::CPY() {
    return 0;
}
uint8_t cpu6502::DEC() {
    return 0;
}
uint8_t cpu6502::DEX() {
    return 0;
}
uint8_t cpu6502::DEY() {
    return 0;
}
uint8_t cpu6502::EOR() {
    return 0;
}
uint8_t cpu6502::INC() {
    return 0;
}
uint8_t cpu6502::INX() {
    return 0;
}
uint8_t cpu6502::INY() {
    return 0;
}
uint8_t cpu6502::JMP() {
    return 0;
}
uint8_t cpu6502::JSR() {
    return 0;
}
uint8_t cpu6502::LDA() {
    return 0;
}
uint8_t cpu6502::LDX() {
    return 0;
}
uint8_t cpu6502::LDY() {
    return 0;
}
uint8_t cpu6502::LSR() {
    return 0;
}
uint8_t cpu6502::NOP() {
    return 0;
}
uint8_t cpu6502::ORA() {
    return 0;
}
uint8_t cpu6502::PHA() {
    return 0;
}
uint8_t cpu6502::PHP() {
    return 0;
}
uint8_t cpu6502::PLA() {
    return 0;
}
uint8_t cpu6502::PLP() {
    return 0;
}
uint8_t cpu6502::ROL() {
    return 0;
}
uint8_t cpu6502::ROR() {
    return 0;
}
uint8_t cpu6502::RTI() {
    return 0;
}
uint8_t cpu6502::RTS() {
    return 0;
}
uint8_t cpu6502::SBC() {
    return 0;
}
uint8_t cpu6502::SEC() {
    return 0;
}
uint8_t cpu6502::SED() {
    return 0;
}
uint8_t cpu6502::SEI() {
    return 0;
}
uint8_t cpu6502::STA() {
    return 0;
}
uint8_t cpu6502::STX() {
    return 0;
}
uint8_t cpu6502::STY() {
    return 0;
}
uint8_t cpu6502::TAX() {
    return 0;
}
uint8_t cpu6502::TAY() {
    return 0;
}
uint8_t cpu6502::TSX() {
    return 0;
}
uint8_t cpu6502::TXA() {
    return 0;
}
uint8_t cpu6502::TXS() {
    return 0;
}
uint8_t cpu6502::TYA() {
    return 0;
}
