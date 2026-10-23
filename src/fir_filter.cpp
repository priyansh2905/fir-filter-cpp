#include "fir_filter.hpp"

#include <algorithm>
#include <cstring>

namespace fir {

FIRFilter::FIRFilter(const std::vector<float>& coefficients) {
    setCoefficients(coefficients);
}

FIRFilter::FIRFilter(const float* coefficients, std::size_t numTaps) {
    setCoefficients(coefficients, numTaps);
}

void FIRFilter::setCoefficients(const std::vector<float>& coefficients) {
    if (coefficients.empty()) {
        throw std::invalid_argument("FIR filter coefficient array cannot be empty.");
    }
    m_coefficients = coefficients;
    m_buffer.assign(m_coefficients.size(), 0.0f);
    m_head = 0;
}

void FIRFilter::setCoefficients(const float* coefficients, std::size_t numTaps) {
    if (!coefficients) {
        throw std::invalid_argument("FIR filter coefficient pointer cannot be null.");
    }
    if (numTaps == 0) {
        throw std::invalid_argument("FIR filter tap count must be at least 1.");
    }
    m_coefficients.assign(coefficients, coefficients + numTaps);
    m_buffer.assign(numTaps, 0.0f);
    m_head = 0;
}

float FIRFilter::processSample(float input) noexcept {
    const std::size_t N = m_coefficients.size();
    if (N == 0) {
        return 0.0f;
    }

    // 1. Write the current input sample x[n] to the circular buffer head
    m_buffer[m_head] = input;

    // 2. Convolution sum: y[n] = sum_{k=0}^{N-1} h[k] * x[n-k]
    // The delay element x[n-k] resides at ring index (m_head - k + N) % N.
    // To achieve zero modulo overhead and optimal cache locality, the dot product
    // is split into two contiguous linear unrolled passes:
    //
    // Segment A (k = 0 ... m_head):
    //   Indices step backward from m_head down to 0.
    //
    // Segment B (k = m_head + 1 ... N - 1):
    //   Indices step backward from N - 1 down to m_head + 1.
    float acc = 0.0f;
    const float* const h = m_coefficients.data();
    const float* const buf = m_buffer.data();

    std::size_t k = 0;

    // Segment A
    for (std::size_t b = m_head; ; --b) {
        acc += h[k++] * buf[b];
        if (b == 0) {
            break;
        }
    }

    // Segment B
    for (std::size_t b = N - 1; b > m_head; --b) {
        acc += h[k++] * buf[b];
    }

    // 3. Advance circular buffer write pointer with branchless wraparound
    ++m_head;
    if (m_head >= N) {
        m_head = 0;
    }

    return acc;
}

void FIRFilter::processBlock(const float* input, float* output, std::size_t length) {
    if (length == 0) {
        return;
    }
    if (!input || !output) {
        throw std::invalid_argument("Input and output pointers must not be null.");
    }

    for (std::size_t i = 0; i < length; ++i) {
        output[i] = processSample(input[i]);
    }
}

void FIRFilter::reset() noexcept {
    std::fill(m_buffer.begin(), m_buffer.end(), 0.0f);
    m_head = 0;
}

} // namespace fir
