#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <map>

class Bus;

class cpu6502 {
    public:
        cpu6502();
        ~cpu6502();

        void ConnectBus(Bus *n) { bus = n; } // links this cpu to a bus


        // core cpu registers
        uint8_t  a      = 0x00;		// Accumulator Register
        uint8_t  x      = 0x00;		// X Register
        uint8_t  y      = 0x00;		// Y Register
	    uint8_t  stkp   = 0x00;		// Stack Pointer
		uint16_t pc     = 0x0000;	// Program Counter
		uint8_t  status = 0x00;		// Status Register

		void reset();	// Reset Interrupt - Forces CPU into known state
		void irq();		// Interrupt Request - Executes an instruction at a specific location
		void nmi();		// Non-Maskable Interrupt Request - As above, but cannot be disabled
		void clock();	// Perform one clock cycle's worth of update
		bool complete(); // Returns true if the CPU has completed its current instruction
		std::map<uint16_t, std::string> disassemble(uint16_t nStart, uint16_t nStop);

		enum FLAGS6502 // the 8 flags stored in the status register
			{
				C = (1 << 0),	// Carry Bit
				Z = (1 << 1),	// Zero
				I = (1 << 2),	// Disable Interrupts
				D = (1 << 3),	// Decimal Mode
				B = (1 << 4),	// Break
				U = (1 << 5),	// Unused
				V = (1 << 6),	// Overflow
				N = (1 << 7),	// Negative
			};

    private:
        // links to the bus
        Bus *bus = nullptr;
        uint8_t read(uint16_t addr);
        void write(uint16_t addr, uint8_t data);

        uint8_t  fetched     = 0x00;   // Represents the working input value to the ALU
        uint16_t temp        = 0x0000; // A convenience variable used everywhere
        uint16_t addr_abs    = 0x0000; // All used memory addresses end up in here
        uint16_t addr_rel    = 0x00;   // Represents absolute address following a branch
        uint8_t  opcode      = 0x00;   // Is the instruction byte
        uint8_t  cycles      = 0;	   // Counts how many cycles the instruction has remaining
        uint32_t clock_count = 0;	   // A global accumulation of the number of clocks

        uint8_t fetch(); // function to fetch instructions

        void branch();
        void stck_push(uint8_t data); // function to push stuff onto stack
        uint8_t stck_pull(); // function to pull stuff from stack

        // used to access status register
        uint8_t GetFlag(FLAGS6502 f);
        void    SetFlag(FLAGS6502 f, bool v);

        // Addressing Modes
        // The address mode changes the number of bytes that
        // makes up the full instruction, so we implement addressing
        // before executing the instruction, to make sure the program
        // counter is at the correct location, the instruction is
    	// primed with the addresses it needs, and the number of clock
        // cycles the instruction requires is calculated.
        uint8_t IMP(); // Instructions like RTS or CLC have no address operand, the destination of results are implied.
        uint8_t IMM(); // Uses the 8-bit operand itself as the value for the operation, rather than fetching a value from a memory address.
        uint8_t ZP0(); // Fetches the value from an 8-bit address on the zero page.
        uint8_t ZPX(); // Zero Page with X Offset
        uint8_t ZPY(); // Zero Page with Y Offset
        uint8_t REL(); // Branch instructions (e.g. BEQ, BCS) have a relative addressing mode that specifies an 8-bit signed offset relative to the current PC.
        uint8_t ABS(); // Fetches the value from a 16-bit address anywhere in memory.
        uint8_t ABX(); // Absolute with X offset
        uint8_t ABY(); // Absolute with Y offset
        uint8_t IND(); // The JMP instruction has a special indirect addressing mode that can jump to the address stored in a 16-bit pointer anywhere in memory.
        uint8_t IZX(); // Indirect with X offset
        uint8_t IZY(); // Indirect with Y offset

        // opcodes
        // to see details about each operation refer to the 6502.cpp file
        uint8_t ADC();	uint8_t AND();	uint8_t ASL();	uint8_t BCC();
        uint8_t BCS();	uint8_t BEQ();	uint8_t BIT();	uint8_t BMI();
        uint8_t BNE();	uint8_t BPL();	uint8_t BRK();	uint8_t BVC();
        uint8_t BVS();	uint8_t CLC();	uint8_t CLD();	uint8_t CLI();
        uint8_t CLV();	uint8_t CMP();	uint8_t CPX();	uint8_t CPY();
        uint8_t DEC();	uint8_t DEX();	uint8_t DEY();	uint8_t EOR();
        uint8_t INC();	uint8_t INX();	uint8_t INY();	uint8_t JMP();
        uint8_t JSR();	uint8_t LDA();	uint8_t LDX();	uint8_t LDY();
        uint8_t LSR();	uint8_t NOP();	uint8_t ORA();	uint8_t PHA();
        uint8_t PHP();	uint8_t PLA();	uint8_t PLP();	uint8_t ROL();
        uint8_t ROR();	uint8_t RTI();	uint8_t RTS();	uint8_t SBC();
        uint8_t SEC();	uint8_t SED();	uint8_t SEI();	uint8_t STA();
        uint8_t STX();	uint8_t STY();	uint8_t TAX();	uint8_t TAY();
        uint8_t TSX();	uint8_t TXA();	uint8_t TXS();	uint8_t TYA();

        uint8_t XXX(); // illegal opcode

        // This structure and the following vector are used to compile and store
        // the opcode translation table. The 6502 can effectively have 256
        // different instructions. Each of these are stored in a table in numerical
        // order so they can be looked up easily, with no decoding required.

        struct INSTRUCTION
        {
		std::string name; // Pneumonic : A textual representation of the instruction (used for disassembly)
		uint8_t (cpu6502::*operate )(void) = nullptr; // Opcode Function: A function pointer to the implementation of the opcode
		uint8_t (cpu6502::*addrmode)(void) = nullptr; // Opcode Address Mode : A function pointer to the implementation of the addressing mechanism used by the instruction
		uint8_t cycles = 0; // An integer that represents the base number of clock cycles the CPU requires to perform the instruction
        };

        std::vector<INSTRUCTION> lookup;
};
