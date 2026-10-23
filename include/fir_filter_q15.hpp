#ifndef FIR_FILTER_Q15_HPP
#define FIR_FILTER_Q15_HPP

#include <vector>
#include <cstdint>
#include <cstddef>
#include <stdexcept>

namespace fir {

/**
 * @brief Fixed-point Q15 FIR Filter designed for microcontrollers lacking Hardware FPU (e.g. ARM Cortex-M0/M3).
 *
 * Characteristics:
 * - 16-bit signed integer representation (Q15 format: 1 sign bit, 15 fractional bits).
 * - Dynamic range: [-1.0, +0.999969482421875].
 * - 64-bit wide accumulator prevents overflow during convolution summation.
 * - Convergent rounding (+ 0.5 LSB, or + 16384) prior to right shift of 15 bits.
 * - Saturating arithmetic clamps overflow/underflow to [INT16_MIN, INT16_MAX].
 * - Deterministic, zero heap allocation in streaming execution.
 */
class FIRFilterQ15 {
public:
    explicit FIRFilterQ15(const std::vector<int16_t>& coefficients);
    FIRFilterQ15(const int16_t* coefficients, std::size_t numTaps);

    /**
     * @brief Helper to convert floating-point coefficients to Q15 format with clamping.
     */
    static std::vector<int16_t> floatToQ15(const std::vector<float>& floatCoeffs);

    /**
     * @brief Helper to convert Q15 value to float.
     */
    static float q15ToFloat(int16_t val) noexcept;

    /**
     * @brief Helper to convert float to a single Q15 sample with saturation.
     */
    static int16_t floatToQ15Sample(float val) noexcept;

    int16_t processSample(int16_t input) noexcept;
    void processBlock(const int16_t* input, int16_t* output, std::size_t length);
    void reset() noexcept;

    std::size_t numTaps() const noexcept {
        return m_coefficients.size();
    }

    const std::vector<int16_t>& getCoefficients() const noexcept {
        return m_coefficients;
    }

    std::size_t memoryFootprintBytes() const noexcept {
        return sizeof(FIRFilterQ15)
             + (m_coefficients.capacity() * sizeof(int16_t))
             + (m_buffer.capacity() * sizeof(int16_t));
    }

private:
    std::vector<int16_t> m_coefficients;
    std::vector<int16_t> m_buffer;
    std::size_t m_head{0};
};

} // namespace fir

#endif // FIR_FILTER_Q15_HPP
