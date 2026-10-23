#include "fir_filter_q15.hpp"

#include <algorithm>
#include <cmath>

namespace fir {

FIRFilterQ15::FIRFilterQ15(const std::vector<int16_t>& coefficients) {
    if (coefficients.empty()) {
        throw std::invalid_argument("Q15 coefficients vector cannot be empty.");
    }
    m_coefficients = coefficients;
    m_buffer.assign(coefficients.size(), 0);
    m_head = 0;
}

FIRFilterQ15::FIRFilterQ15(const int16_t* coefficients, std::size_t numTaps) {
    if (!coefficients) {
        throw std::invalid_argument("Q15 coefficients pointer cannot be null.");
    }
    if (numTaps == 0) {
        throw std::invalid_argument("Q15 tap count must be at least 1.");
    }
    m_coefficients.assign(coefficients, coefficients + numTaps);
    m_buffer.assign(numTaps, 0);
    m_head = 0;
}

int16_t FIRFilterQ15::floatToQ15Sample(float val) noexcept {
    // Scale by 2^15 = 32768.0
    float scaled = std::round(val * 32768.0f);
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return static_cast<int16_t>(scaled);
}

float FIRFilterQ15::q15ToFloat(int16_t val) noexcept {
    return static_cast<float>(val) / 32768.0f;
}

std::vector<int16_t> FIRFilterQ15::floatToQ15(const std::vector<float>& floatCoeffs) {
    std::vector<int16_t> q15Coeffs;
    q15Coeffs.reserve(floatCoeffs.size());
    for (float c : floatCoeffs) {
        q15Coeffs.push_back(floatToQ15Sample(c));
    }
    return q15Coeffs;
}

int16_t FIRFilterQ15::processSample(int16_t input) noexcept {
    const std::size_t N = m_coefficients.size();
    if (N == 0) {
        return 0;
    }

    m_buffer[m_head] = input;

    // Convolution with 64-bit accumulator to avoid intermediate overflow
    int64_t acc = 0;
    const int16_t* const h = m_coefficients.data();
    const int16_t* const buf = m_buffer.data();

    std::size_t k = 0;
    // Segment A
    for (std::size_t b = m_head; ; --b) {
        acc += static_cast<int64_t>(h[k++]) * static_cast<int64_t>(buf[b]);
        if (b == 0) {
            break;
        }
    }

    // Segment B
    for (std::size_t b = N - 1; b > m_head; --b) {
        acc += static_cast<int64_t>(h[k++]) * static_cast<int64_t>(buf[b]);
    }

    // Advance ring buffer pointer
    ++m_head;
    if (m_head >= N) {
        m_head = 0;
    }

    // Rounding (+ 0.5 LSB = 1 << 14) and scaling down by 2^15
    acc += (1LL << 14);
    int64_t shifted = acc >> 15;

    // Saturation logic
    if (shifted > 32767LL) {
        return 32767;
    }
    if (shifted < -32768LL) {
        return -32768;
    }

    return static_cast<int16_t>(shifted);
}

void FIRFilterQ15::processBlock(const int16_t* input, int16_t* output, std::size_t length) {
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

void FIRFilterQ15::reset() noexcept {
    std::fill(m_buffer.begin(), m_buffer.end(), 0);
    m_head = 0;
}

} // namespace fir
