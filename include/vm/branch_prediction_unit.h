#pragma once
#include <cstdint>
#include <map>

/**
 * @brief Holds the result of a branch prediction.
 */
struct BranchPrediction {
    bool is_taken;          // The prediction (true = taken, false = not taken)
    uint64_t target_pc;     // The predicted target PC
};

/**
 * @brief Implements a 2-bit saturating counter for dynamic branch prediction.
 *
 * This unit maintains a Branch History Table (BHT) with 2-bit counters
 * and a Branch Target Buffer (BTB) to store target addresses.
 */
class BranchPredictionUnit {
public:
    BranchPredictionUnit() = default;

    /**
     * @brief Predicts the outcome of a branch at a given PC.
     * @param pc The address of the branch instruction.
     * @param default_target The non-predicted target (usually PC+4).
     * @return A BranchPrediction struct with the predicted outcome and target.
     */
    BranchPrediction predict(uint64_t pc, uint64_t default_target);

    /**
     * @brief Updates the predictor with the *actual* outcome of a branch.
     * This is how the predictor learns.
     *
     * @param pc The address of the branch instruction.
     * @param actual_outcome True if the branch was actually taken.
     * @param actual_target The actual target PC (if taken).
     */
    void update(uint64_t pc, bool actual_outcome, uint64_t actual_target);

    /**
     * @brief Resets the prediction tables.
     */
    void reset();

private:
    // Branch History Table (BHT)
    // Maps a PC to a 2-bit saturating counter (0, 1, 2, 3)
    // 00 (0) = Strongly Not Taken
    // 01 (1) = Weakly Not Taken
    // 10 (2) = Weakly Taken
    // 11 (3) = Strongly Taken
    std::map<uint64_t, int> bht_;

    // Branch Target Buffer (BTB)
    // Maps a PC to its last known target address
    std::map<uint64_t, uint64_t> btb_;
};