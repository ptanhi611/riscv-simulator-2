#include "vm/hazard_detection_unit.h"
#include "include/pipelining_register.h" // Need the full definitions
#include "common/instructions.h" // For isFInstruction, etc.
#include <iostream>

using instruction_set::Instruction;
using instruction_set::get_instr_encoding;

PipelineControlSignals HazardDetectionUnit::compute_signals(
    int mode,
    uint32_t if_id_inst,
    const ID_EX_registers& id_ex_reg,
    const EX_MEM_registers& ex_mem_reg,
    const MEM_WB_registers& mem_wb_reg) 
{
    uint8_t rs1 = (if_id_inst >> 15) & 0b11111;
    uint8_t rs2 = (if_id_inst >> 20) & 0b11111;

    PipelineControlSignals signals;

    // --- MODE 1: PIPELINE_NO_HAZARD ---
    if (mode == 1) {
        return signals; // Return default (no stalls, no forwards)
    }

    // --- Check for STRUCTURAL and CONTROL hazards (Stall) ---
    // These hazards cause a stall in ALL modes (2 and 3)
    
    uint8_t ex_opcode = id_ex_reg.instruction_bits & 0b1111111;
    bool is_ex_complex = instruction_set::isFInstruction(id_ex_reg.instruction_bits) ||
                         instruction_set::isDInstruction(id_ex_reg.instruction_bits) ||
                         ex_opcode == get_instr_encoding(Instruction::kmul).opcode;

    
    if (id_ex_reg.valid && is_ex_complex) {
        std::cout << "[HDU] STALL: Structural hazard (EX stage busy)" << std::endl;
        signals.stall_fetch = true;
        signals.flush_decode = true;
        return signals;
    }

    
    if (id_ex_reg.valid && id_ex_reg.signals.branch) {
        std::cout << "[HDU] STALL: Control hazard (branch in EX)" << std::endl;
        signals.stall_fetch = true;
        signals.flush_decode = true;
        return signals;
    }

    if (mode == 2) {
        
        if (id_ex_reg.valid && id_ex_reg.signals.reg_write_ && id_ex_reg.rd != 0) {
            if (id_ex_reg.rd == rs1 || id_ex_reg.rd == rs2) {
                std::cout << "[HDU] STALL (Mode 2): EX->ID hazard on x" << (int)id_ex_reg.rd << std::endl;
                signals.stall_fetch = true;
                signals.flush_decode = true;
                return signals;
            }
        }
        
        
        if (ex_mem_reg.valid && ex_mem_reg.signals.reg_write_ && ex_mem_reg.des_address != 0) {
            if (ex_mem_reg.des_address == rs1 || ex_mem_reg.des_address == rs2) {
                std::cout << "[HDU] STALL (Mode 2): MEM->ID hazard on x" << (int)ex_mem_reg.des_address << std::endl;
                signals.stall_fetch = true;
                signals.flush_decode = true;
                return signals;
            }
        }
        
        
        if (mem_wb_reg.valid && mem_wb_reg.signals.reg_write_ && mem_wb_reg.des_address != 0) {
            if (mem_wb_reg.des_address == rs1 || mem_wb_reg.des_address == rs2) {
                std::cout << "[HDU] STALL (Mode 2): WB->ID hazard on x" << (int)mem_wb_reg.des_address << std::endl;
                signals.stall_fetch = true;
                signals.flush_decode = true;
                return signals;
            }
        }
        
        return signals; // No data hazards found
    }

   


    if (mode >= 3) {
        
        // 1. Check for Load-Use Hazard (The only one that *must* stall)
        if (id_ex_reg.valid && id_ex_reg.signals.mem_read_) {
            if (id_ex_reg.rd != 0 && (id_ex_reg.rd == rs1 || id_ex_reg.rd == rs2)) {
                std::cout << "[HDU] STALL (Mode 3): Load-Use on x" << (int)id_ex_reg.rd << std::endl;
                signals.stall_fetch = true;
                signals.flush_decode = true;
                return signals;
            }
        }

        // 2. Handle Forwarding for rs1
        // Check EX/MEM register
        if (ex_mem_reg.valid && ex_mem_reg.signals.reg_write_ && ex_mem_reg.des_address != 0) {
            if (ex_mem_reg.des_address == rs1) {
                std::cout << "[HDU] FORWARD-A: EX->EX for x" << (int)ex_mem_reg.des_address << std::endl; 
                signals.forward_A = ForwardSource::FROM_EXECUTE;
            }
        }
        
        // Check MEM/WB register (if not already forwarded from EX/MEM)
        if (mem_wb_reg.valid && mem_wb_reg.signals.reg_write_ && mem_wb_reg.des_address != 0) {
            if (mem_wb_reg.des_address == rs1 && signals.forward_A == ForwardSource::FROM_DECODE) {
                std::cout << "[HDU] FORWARD-A: MEM->EX for x" << (int)mem_wb_reg.des_address << std::endl;
                signals.forward_A = ForwardSource::FROM_MEMORY;
            }
        }

        // 3. Handle Forwarding for rs2
        // Check EX/MEM register
        if (ex_mem_reg.valid && ex_mem_reg.signals.reg_write_ && ex_mem_reg.des_address != 0) {
            if (ex_mem_reg.des_address == rs2) {
                std::cout << "[HDU] FORWARD-B: EX->EX for x" << (int)ex_mem_reg.des_address << std::endl; 
                signals.forward_B = ForwardSource::FROM_EXECUTE;
            }
        }

        // Check MEM/WB register (if not already forwarded from EX/MEM)
        if (mem_wb_reg.valid && mem_wb_reg.signals.reg_write_ && mem_wb_reg.des_address != 0) {
            if (mem_wb_reg.des_address == rs2 && signals.forward_B == ForwardSource::FROM_DECODE) {
                std::cout << "[HDU] FORWARD-B: MEM->EX for x" << (int)mem_wb_reg.des_address << std::endl;
                signals.forward_B = ForwardSource::FROM_MEMORY;
            }
        }
    }

    return signals;
}