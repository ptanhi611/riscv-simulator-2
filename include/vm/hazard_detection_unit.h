#pragma once

#include "pipelining_register.h" 
#include <cstdint>

/**
 * @brief Defines the source for the ALU inputs in the EX stage.
 * This is used to control the forwarding MUXes.
 */


/**
 * @brief Output signals from the Hazard/Forwarding Unit.
 * These signals control the entire pipeline to manage all hazards.
 */


struct PipelineControlSignals {
    // --- STALL/FLUSH Signals ---

    /**
     * @brief Freezes the PC and the IF/ID register.
     * (Used for Load-Use stalls and by branch logic).
     */
    bool stall_fetch = false;
    
    /**
     * @brief Injects a NOP ("bubble") into the ID/EX register.
     * (Used for stalls).
     */
    bool flush_decode = false;

    // --- FORWARDING Signals (for Mode 4+) ---
    // These control the MUXes before the ALU in the EX stage.
    
    /**
     * @brief Selects the data source for the ALU's first input (rs1_data).
     */
    ForwardSource forward_A = ForwardSource::FROM_DECODE;
    
    /**
     * @brief Selects the data source for the ALU's second input (rs2_data).
     */
    ForwardSource forward_B = ForwardSource::FROM_DECODE;

  
    bool pc_select_branch = false;
    uint32_t branch_target_add;
};

/**
 * @brief Implements hazard detection AND forwarding logic for the 5-stage pipeline.
 * This unit is the "brains" of the pipeline, coordinating stalls and data forwarding
 * based on the current mode.
 */
class HazardDetectionUnit {
public:
    HazardDetectionUnit() = default;

    /**
     * @brief Computes all control signals for the pipeline based on the current state.
     * This single function can handle all modes.
     *
     * @param mode The current pipeline mode (0=SC, 1=NoHazard, 2=Stall, 3=Forward).
     * @param if_id_inst The instruction in the IF/ID register (being decoded).
     * @param id_ex_reg The state of the ID/EX register (in EX stage).
     * @param ex_mem_reg The state of the EX/MEM register (in MEM stage).
     * @param mem_wb_reg The state of the MEM/WB register (in WB stage).
     * @return PipelineControlSignals struct with all necessary control commands.
     */
    PipelineControlSignals compute_signals(
        int mode, // 0=SC, 1=NoHazard, 2=StallOnly, 3=StallAndForward
        uint32_t if_id_inst,
        const ID_EX_registers& id_ex_reg,
        const EX_MEM_registers& ex_mem_reg,
        const MEM_WB_registers& mem_wb_reg
    );
};

