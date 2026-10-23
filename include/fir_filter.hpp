#ifndef FIR_FILTER_HPP
#define FIR_FILTER_HPP

#include <vector>
#include <cstddef>
#include <stdexcept>

namespace fir {

/**
 * @brief High-performance, embedded-friendly Finite Impulse Response (FIR) digital filter.
 *
 * Implements the discrete-time FIR difference equation:
 *     y[n] = sum_{k=0}^{N-1} h[k] * x[n-k]
 *
 * Architecture and Ring Buffer Design:
 * ------------------------------------
 * - Stores previous input samples in a pre-allocated circular ring buffer of length N (numTaps).
 * - Zero heap memory allocation occurs during per-sample or block processing (`noexcept` streaming path).
 * - Coefficient ordering:
 *     h[0] is multiplied by current sample x[n] (delay k = 0).
 *     h[1] is multiplied by previous sample x[n-1] (delay k = 1).
 *     ...
 *     h[N-1] is multiplied by oldest sample x[n-(N-1)] (delay k = N-1).
 * - Circular Buffer Indexing:
 *     Sample x[n] is stored at `m_buffer[m_head]`.
 *     Previous sample x[n-k] resides at `(m_head - k + N) % N`.
 *     The inner dot product is executed in two contiguous linear segments without modulo operations,
 *     enabling compiler vectorization (SIMD) and bounded deterministic execution cycles.
 */
class FIRFilter {
public:
    /**
     * @brief Constructs an FIR filter from a vector of coefficients.
     * @param coefficients Non-empty vector of filter tap weights h[0] ... h[N-1].
     * @throws std::invalid_argument if coefficients vector is empty.
     */
    explicit FIRFilter(const std::vector<float>& coefficients);

    /**
     * @brief Constructs an FIR filter from a raw coefficient array.
     * @param coefficients Pointer to array of filter tap weights.
     * @param numTaps Number of taps in array (must be > 0).
     * @throws std::invalid_argument if coefficients pointer is null or numTaps is 0.
     */
    FIRFilter(const float* coefficients, std::size_t numTaps);

    /**
     * @brief Copy and Move constructors / assignments.
     */
    FIRFilter(const FIRFilter&) = default;
    FIRFilter& operator=(const FIRFilter&) = default;
    FIRFilter(FIRFilter&&) noexcept = default;
    FIRFilter& operator=(FIRFilter&&) noexcept = default;
    ~FIRFilter() = default;

    /**
     * @brief Processes a single input sample through the filter.
     * Guaranteed zero dynamic memory allocations and deterministic execution time.
     *
     * @param input Input sample x[n].
     * @return Filtered output sample y[n].
     */
    float processSample(float input) noexcept;

    /**
     * @brief Processes a contiguous block of samples.
     *
     * @param input Pointer to contiguous input array of size `length`.
     * @param output Pointer to contiguous output array of size `length`.
     * @param length Number of samples to process.
     * @throws std::invalid_argument if length > 0 and input or output pointer is null.
     */
    void processBlock(const float* input, float* output, std::size_t length);

    /**
     * @brief Updates filter coefficients and resets the internal ring buffer.
     *
     * @param coefficients New filter coefficients vector.
     * @throws std::invalid_argument if coefficients vector is empty.
     */
    void setCoefficients(const std::vector<float>& coefficients);

    /**
     * @brief Updates filter coefficients and resets the internal ring buffer.
     *
     * @param coefficients Pointer to new coefficient array.
     * @param numTaps Number of taps in array.
     * @throws std::invalid_argument if coefficients pointer is null or numTaps is 0.
     */
    void setCoefficients(const float* coefficients, std::size_t numTaps);

    /**
     * @brief Clears the ring buffer delay history to zero.
     * Preserves filter coefficients.
     */
    void reset() noexcept;

    /**
     * @brief Returns the number of filter taps N.
     */
    std::size_t numTaps() const noexcept {
        return m_coefficients.size();
    }

    /**
     * @brief Returns a reference to the active filter coefficients.
     */
    const std::vector<float>& getCoefficients() const noexcept {
        return m_coefficients;
    }

    /**
     * @brief Returns the current state of the internal ring buffer (for testing/diagnostics).
     */
    const std::vector<float>& getBuffer() const noexcept {
        return m_buffer;
    }

    /**
     * @brief Returns the current ring buffer write head position.
     */
    std::size_t getHeadIndex() const noexcept {
        return m_head;
    }

    /**
     * @brief Computes total memory consumed by this filter instance in bytes.
     * Includes object footprint, coefficient storage, and delay line buffer.
     */
    std::size_t memoryFootprintBytes() const noexcept {
        return sizeof(FIRFilter)
             + (m_coefficients.capacity() * sizeof(float))
             + (m_buffer.capacity() * sizeof(float));
    }

private:
    std::vector<float> m_coefficients; ///< Filter impulse response h[0]..h[N-1]
    std::vector<float> m_buffer;       ///< Circular delay line buffer x[n]..x[n-(N-1)]
    std::size_t m_head{0};             ///< Write pointer into circular buffer
};

} // namespace fir

#endif // FIR_FILTER_HPP
