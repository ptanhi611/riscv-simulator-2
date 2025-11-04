#pragma once
#include<cstdint>


struct Control_signals{
    uint8_t alu_op = 0;
    bool alu_src_ = false;

    bool mem_read_ = false;
    bool mem_write_ = false;

    bool reg_write_ = false;
    bool mem_to_reg = false;
};





struct IF_ID_registers{
    uint64_t pc = 0;
    uint64_t pc_plus_4 = 0;
    uint32_t inst = 0;

    
    bool stall = false;

    
};



struct ID_EX_registers{
    uint64_t pc_plus_4 = 0;

    uint64_t rs1_data = 0;
    uint64_t rs_2_data = 0;

    int64_t imm = 0;

    uint32_t instruction_bits;

    uint8_t rs1 = 0;
    uint8_t rs2 = 0;
    uint8_t rd = 0;

    bool stall = false;

    Control_signals signals;

    uint8_t funct3;
    uint8_t funct7;
};




struct EX_MEM_registers{
    uint64_t alu_ans = 0;

    uint64_t data = 0;
    uint8_t des_address = 0;

    bool stall = false;

    Control_signals signals;

    uint8_t funct3;
    

};





struct MEM_WB_registers{
    uint64_t alu_ans = 0;

    uint64_t mem_data  = 0;
    uint8_t des_address = 0;

    bool stall = false;

    Control_signals signals;
};