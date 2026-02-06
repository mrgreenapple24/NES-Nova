#pragma once
#include <array>
#include <cstdint>
#include "6502.h"

class Bus {
    public:
        Bus();
        ~Bus();

    public:
        void write(uint16_t addr, uint8_t data); // writes data into ram
        uint8_t read(uint16_t addr, bool valid); // reads data from ram

        cpu6502 cpu;
        std::array<uint8_t, 64*1024> ram;
};
