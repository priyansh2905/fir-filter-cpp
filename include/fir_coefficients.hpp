#ifndef FIR_COEFFICIENTS_HPP
#define FIR_COEFFICIENTS_HPP

#include <vector>
#include <cstddef>

namespace fir {

/**
 * @brief Designs a Finite Impulse Response (FIR) low-pass filter using the windowed-sinc method with a Hamming window.
 *
 * Mathematical formulation:
 * - Normalized cutoff frequency fc in cycles/sample, where 0 < fc < 0.5 (Nyquist frequency is 0.5 cycles/sample).
 * - Filter length is N taps (numTaps >= 1).
 * - For centered index m = n - (N - 1) / 2.0:
 *     h_ideal[m] = 2 * fc,                         when m == 0 (odd N center tap)
 *     h_ideal[m] = sin(2 * pi * fc * m) / (pi * m), otherwise
 *   Note: For even N, m is a half-integer (e.g., -0.5, +0.5), so m != 0 for all indices n.
 *
 * - Hamming window:
 *     w[n] = 0.54 - 0.46 * cos(2 * pi * n / (N - 1))   for N > 1
 *     w[0] = 1.0                                       for N == 1
 *
 * - Windowed coefficients:
 *     h[n] = h_ideal[n] * w[n]
 *
 * - DC Gain Normalization:
 *     Coefficients are normalized such that sum(h[n]) == 1.0, guaranteeing unity gain (0 dB) at 0 Hz (DC).
 *
 * @param numTaps Number of filter taps (must be >= 1).
 * @param cutoffFrequency Normalized cutoff frequency fc in cycles/sample (0.0 < fc < 0.5).
 * @return std::vector<float> Normalized filter coefficients h[0] through h[N-1].
 * @throws std::invalid_argument if numTaps == 0, cutoffFrequency <= 0.0f, cutoffFrequency >= 0.5f,
 *         or if cutoffFrequency is NaN/Inf.
 */
std::vector<float> designLowPassHamming(std::size_t numTaps, float cutoffFrequency);

} // namespace fir

#endif // FIR_COEFFICIENTS_HPP
