#include "vm/hazard_detection_unit.h"

#include "common/instructions.h" // For isFInstruction, etc.
#include <iostream>

using instruction_set::Instruction;
using instruction_set::get_instr_encoding;

PipelineControlSignals HazardDetectionUnit::compute_signals(
    int mode,
    uint32_t if_id_inst,
    const ID_EX_registers& id_ex_reg,
    const EX_MEM_registers& ex_mem,
    const MEM_WB_registers& mem_wb_reg) 
{
    uint8_t rs1 = (if_id_inst >> 15) & 0b11111;
    uint8_t rs2 = (if_id_inst >> 20) & 0b11111;

    PipelineControlSignals signals;

    if (mode == 1) {
        return signals;
    }

    
    // --- MODE 2: STALL-ONLY ---
    if (mode == 2) {
        if (id_ex_reg.valid && id_ex_reg.signals.reg_write_ && id_ex_reg.rd != 0) {
            if (id_ex_reg.rd == rs1 || id_ex_reg.rd == rs2) {
                std::cout << "[HDU] STALL (Mode 2): EX->ID hazard on x" << (int)id_ex_reg.rd << std::endl;
                signals.stall_fetch = true;
                signals.flush_decode = true;
                return signals;
            }
        }
        
        if (ex_mem.valid && ex_mem.signals.reg_write_ && ex_mem.des_address != 0) {
            if (ex_mem.des_address == rs1 || ex_mem.des_address == rs2) {
                std::cout << "[HDU] STALL (Mode 2): MEM->ID hazard on x" << (int)ex_mem.des_address << std::endl;
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
        
        return signals; 
    }

    // --- MODE 3+: FORWARDING ---
    // --- MODE 3+: FORWARDING ---
    // --- MODE 3+: FORWARDING ---
    if (mode >= 3) {

        // 1. Load-Use Hazard STALL
        if (id_ex_reg.valid && id_ex_reg.signals.mem_read_ && id_ex_reg.rd != 0) {
            if (id_ex_reg.rd == rs1 || id_ex_reg.rd == rs2) {
                std::cout << "[HDU] STALL (Mode 3): Load-Use on x" << (int)id_ex_reg.rd << std::endl;
                signals.stall_fetch = true;
                signals.flush_decode = true;
                return signals;
            }
        }

        // --- FORWARDING LOGIC FOR RS1 ---
        // Priority 1: Hazard from EX stage (id_ex_reg)
        // Data will be in ex_mem next cycle. MUX must select ex_mem.
        if (id_ex_reg.valid && id_ex_reg.signals.reg_write_ && id_ex_reg.rd != 0 &&
            id_ex_reg.rd == rs1)
        {
            std::cout << "[HDU] FORWARD-A: EX->EX for x" << (int)id_ex_reg.rd << std::endl; 
            signals.forward_A = ForwardSource::FROM_EXECUTE;
        }
        // Priority 2: Hazard from MEM stage (ex_mem)
        // Data will be in mem_wb_reg next cycle. MUX must select mem_wb_reg.
        else if (ex_mem.valid && ex_mem.signals.reg_write_ && ex_mem.des_address != 0 &&
                 ex_mem.des_address == rs1) 
        {
            std::cout << "[HDU] FORWARD-A: MEM->EX for x" << (int)ex_mem.des_address << std::endl; 
            signals.forward_A = ForwardSource::FROM_MEMORY;
        }
        // Priority 3: Hazard from WB stage (mem_wb_reg)


        // --- FORWARDING LOGIC FOR RS2 ---
        // Priority 1: Hazard from EX stage (id_ex_reg)
        // Data will be in ex_mem next cycle. MUX must select ex_mem.
        if (id_ex_reg.valid && id_ex_reg.signals.reg_write_ && id_ex_reg.rd != 0 &&
            id_ex_reg.rd == rs2)
        {
            std::cout << "[HDU] FORWARD-B: EX->EX for x" << (int)id_ex_reg.rd << std::endl; 
            signals.forward_B = ForwardSource::FROM_EXECUTE;
        }
        // Priority 2: Hazard from MEM stage (ex_mem)
        // Data will be in mem_wb_reg next cycle. MUX must select mem_wb_reg.
        else if (ex_mem.valid && ex_mem.signals.reg_write_ && ex_mem.des_address != 0 &&
                 ex_mem.des_address == rs2)
        {
            std::cout << "[HDU] FORWARD-B: MEM->EX for x" << (int)ex_mem.des_address << std::endl; 
            signals.forward_B = ForwardSource::FROM_MEMORY;
        }
        // Priority 3: Hazard from WB stage (mem_wb_reg)
       
    }
    return signals;
}