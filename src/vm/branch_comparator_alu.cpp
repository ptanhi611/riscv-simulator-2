#include "vm/branch_comparator_alu.h"

bool Branch_Comparision::result(uint8_t f3, uint64_t rs1, uint64_t rs2){

    switch (f3) {
        case 0b000: // BEQ
            return (rs1 == rs2);
            
        case 0b001: // BNE
            return (rs1 != rs2);
            
        case 0b100: // BLT (Signed)
            return (static_cast<int64_t>(rs1) < static_cast<int64_t>(rs2));
            
        case 0b101: // BGE (Signed)
            return (static_cast<int64_t>(rs1) >= static_cast<int64_t>(rs2));
            
        case 0b110: // BLTU (Unsigned)
            return (rs1 < rs2);
            
        case 0b111: // BGEU (Unsigned)
            return (rs1 >= rs2);
            
        default:
            return false;
    }

}