#include "fir_coefficients.hpp"

#include <cmath>
#include <stdexcept>
#include <numeric>

namespace fir {

namespace {
constexpr double PI = 3.141592653589793238462643383279502884;
} // anonymous namespace

std::vector<float> designLowPassHamming(std::size_t numTaps, float cutoffFrequency) {
    if (numTaps == 0) {
        throw std::invalid_argument("FIR tap count must be at least 1.");
    }

    if (std::isnan(cutoffFrequency) || std::isinf(cutoffFrequency)) {
        throw std::invalid_argument("Cutoff frequency must be a finite real number.");
    }

    if (cutoffFrequency <= 0.0f || cutoffFrequency >= 0.5f) {
        throw std::invalid_argument("Cutoff frequency must satisfy 0.0 < fc < 0.5 (cycles/sample).");
    }

    std::vector<float> coefficients(numTaps);

    if (numTaps == 1) {
        // A single-tap filter with unity gain is an all-pass with DC gain = 1.0
        coefficients[0] = 1.0f;
        return coefficients;
    }

    const double fc = static_cast<double>(cutoffFrequency);
    const double center = static_cast<double>(numTaps - 1) / 2.0;

    double sum = 0.0;
    std::vector<double> doubleCoeffs(numTaps, 0.0);

    for (std::size_t n = 0; n < numTaps; ++n) {
        const double m = static_cast<double>(n) - center;

        // 1. Ideal sinc impulse response
        double h_ideal = 0.0;
        if (std::abs(m) < 1e-12) {
            // Center tap for odd tap count (m == 0)
            h_ideal = 2.0 * fc;
        } else {
            // Sinc function
            h_ideal = std::sin(2.0 * PI * fc * m) / (PI * m);
        }

        // 2. Hamming window: w[n] = 0.54 - 0.46 * cos(2 * pi * n / (N - 1))
        const double w = 0.54 - 0.46 * std::cos(2.0 * PI * static_cast<double>(n) / static_cast<double>(numTaps - 1));

        // 3. Windowed coefficient
        const double h_windowed = h_ideal * w;
        doubleCoeffs[n] = h_windowed;
        sum += h_windowed;
    }

    // 4. DC Gain Normalization: Scale coefficients so sum(h[n]) == 1.0
    if (std::abs(sum) < 1e-12) {
        throw std::runtime_error("Sum of filter coefficients is zero; cannot normalize DC gain.");
    }

    for (std::size_t n = 0; n < numTaps; ++n) {
        coefficients[n] = static_cast<float>(doubleCoeffs[n] / sum);
    }

    return coefficients;
}

} // namespace fir
