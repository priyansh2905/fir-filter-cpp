#include "test_framework.hpp"
#include "fir_filter.hpp"
#include "fir_coefficients.hpp"
#include "fir_filter_q15.hpp"

#include <vector>
#include <cmath>
#include <numeric>
#include <limits>

constexpr double PI = 3.14159265358979323846;

// =========================================================================
// Test 1: Empty coefficients are rejected
// =========================================================================
TEST(FIRFilterTest, EmptyCoefficientsRejected) {
    std::vector<float> emptyCoeffs;
    EXPECT_THROW(fir::FIRFilter filter(emptyCoeffs), std::invalid_argument);

    // Also test raw pointer constructor with null or zero size
    EXPECT_THROW(fir::FIRFilter filter(nullptr, 10), std::invalid_argument);
    float dummy[3] = {1.0f, 2.0f, 3.0f};
    EXPECT_THROW(fir::FIRFilter filter(dummy, 0), std::invalid_argument);

    // Test setCoefficients rejecting empty
    fir::FIRFilter validFilter({1.0f});
    EXPECT_THROW(validFilter.setCoefficients(emptyCoeffs), std::invalid_argument);
    EXPECT_THROW(validFilter.setCoefficients(nullptr, 5), std::invalid_argument);
    EXPECT_THROW(validFilter.setCoefficients(dummy, 0), std::invalid_argument);
}

// =========================================================================
// Test 2: Single-tap filter behaves as expected
// =========================================================================
TEST(FIRFilterTest, SingleTapFilter) {
    // Identity filter (gain = 1.0)
    fir::FIRFilter identityFilter({1.0f});
    EXPECT_EQ(identityFilter.numTaps(), 1);
    EXPECT_NEAR(identityFilter.processSample(3.14f), 3.14f, 1e-6f);
    EXPECT_NEAR(identityFilter.processSample(-2.5f), -2.5f, 1e-6f);
    EXPECT_NEAR(identityFilter.processSample(0.0f), 0.0f, 1e-6f);

    // Scaling filter (gain = 0.5)
    fir::FIRFilter scalingFilter({0.5f});
    EXPECT_NEAR(scalingFilter.processSample(10.0f), 5.0f, 1e-6f);
    EXPECT_NEAR(scalingFilter.processSample(-4.0f), -2.0f, 1e-6f);
}

// =========================================================================
// Test 3: Impulse input produces the filter coefficients as its output
// =========================================================================
TEST(FIRFilterTest, ImpulseResponseMatchesCoefficients) {
    const std::vector<float> h = {0.05f, 0.15f, 0.60f, 0.15f, 0.05f};
    fir::FIRFilter filter(h);

    const std::size_t N = h.size();

    // First sample is 1.0 (impulse), remaining samples are 0.0
    for (std::size_t n = 0; n < N; ++n) {
        float input = (n == 0) ? 1.0f : 0.0f;
        float y = filter.processSample(input);
        EXPECT_NEAR(y, h[n], 1e-6f);
    }

    // After N samples, delay line should be entirely filled with 0.0
    for (std::size_t n = 0; n < 5; ++n) {
        float y = filter.processSample(0.0f);
        EXPECT_NEAR(y, 0.0f, 1e-6f);
    }
}

// =========================================================================
// Test 4: Constant input approaches the same constant at steady state (DC gain = 1.0)
// =========================================================================
TEST(FIRFilterTest, ConstantInputSteadyState) {
    const std::size_t taps = 31;
    const float fc = 0.15f; // normalized cutoff
    std::vector<float> coeffs = fir::designLowPassHamming(taps, fc);

    fir::FIRFilter filter(coeffs);
    const float dcInput = 4.25f;

    float lastOutput = 0.0f;
    // Run for 2 * taps samples so steady state is completely reached
    for (std::size_t n = 0; n < taps * 2; ++n) {
        lastOutput = filter.processSample(dcInput);
    }

    // Output at steady state should be input * sum(h) = 4.25 * 1.0 = 4.25
    EXPECT_NEAR(lastOutput, dcInput, 1e-4f);
}

// =========================================================================
// Test 5: Low-frequency sinusoid passes with minimal attenuation
// =========================================================================
TEST(FIRFilterTest, LowFrequencyPassband) {
    const std::size_t taps = 51;
    const float fc = 0.20f; // Cutoff at 0.20
    const float f_low = 0.02f; // Low frequency far inside passband

    std::vector<float> coeffs = fir::designLowPassHamming(taps, fc);
    fir::FIRFilter filter(coeffs);

    const std::size_t numSamples = 300;
    std::vector<float> output(numSamples);

    for (std::size_t n = 0; n < numSamples; ++n) {
        float in = std::sin(static_cast<float>(2.0 * PI * f_low * n));
        output[n] = filter.processSample(in);
    }

    // Measure peak amplitude in steady state (last 100 samples)
    float maxAmp = 0.0f;
    for (std::size_t n = 200; n < numSamples; ++n) {
        maxAmp = std::max(maxAmp, std::abs(output[n]));
    }

    // Passband attenuation should be minimal (magnitude near 1.0, >= 0.97)
    EXPECT_TRUE(maxAmp >= 0.97f);
    EXPECT_TRUE(maxAmp <= 1.03f);
}

// =========================================================================
// Test 6: High-frequency sinusoid is heavily attenuated
// =========================================================================
TEST(FIRFilterTest, HighFrequencyStopband) {
    const std::size_t taps = 51;
    const float fc = 0.15f; // Cutoff at 0.15
    const float f_high = 0.38f; // High frequency deep inside stopband

    std::vector<float> coeffs = fir::designLowPassHamming(taps, fc);
    fir::FIRFilter filter(coeffs);

    const std::size_t numSamples = 300;
    std::vector<float> output(numSamples);

    for (std::size_t n = 0; n < numSamples; ++n) {
        float in = std::sin(static_cast<float>(2.0 * PI * f_high * n));
        output[n] = filter.processSample(in);
    }

    // Measure peak amplitude in steady state (last 100 samples)
    float maxAmp = 0.0f;
    for (std::size_t n = 200; n < numSamples; ++n) {
        maxAmp = std::max(maxAmp, std::abs(output[n]));
    }

    // Hamming window stopband attenuation is typically > 40 dB (amplitude < 0.02)
    EXPECT_TRUE(maxAmp < 0.025f);
}

// =========================================================================
// Test 7: Resetting the filter clears its history
// =========================================================================
TEST(FIRFilterTest, ResetClearsHistory) {
    std::vector<float> coeffs = {0.25f, 0.50f, 0.25f};
    fir::FIRFilter filter(coeffs);

    // Pump some large values
    filter.processSample(10.0f);
    filter.processSample(20.0f);
    filter.processSample(30.0f);

    // Buffer is currently full of past data
    EXPECT_TRUE(filter.getBuffer()[0] != 0.0f || filter.getBuffer()[1] != 0.0f);

    // Call reset()
    filter.reset();

    // Check internal buffer is zeroed and head is reset
    EXPECT_EQ(filter.getHeadIndex(), 0);
    for (float b : filter.getBuffer()) {
        EXPECT_EQ(b, 0.0f);
    }

    // Processing 0.0f must immediately return 0.0f
    float y = filter.processSample(0.0f);
    EXPECT_NEAR(y, 0.0f, 1e-6f);
}

// =========================================================================
// Test 8: Block processing produces identical output to sample-by-sample
// =========================================================================
TEST(FIRFilterTest, BlockProcessingMatchesSampleBySample) {
    const std::size_t taps = 35;
    const float fc = 0.18f;
    std::vector<float> coeffs = fir::designLowPassHamming(taps, fc);

    fir::FIRFilter filterA(coeffs);
    fir::FIRFilter filterB(coeffs);

    const std::size_t len = 128;
    std::vector<float> input(len);
    std::vector<float> outA(len);
    std::vector<float> outB(len);

    for (std::size_t i = 0; i < len; ++i) {
        input[i] = std::sin(static_cast<float>(i) * 0.15f) + 0.5f * std::cos(static_cast<float>(i) * 0.8f);
    }

    // Process sample-by-sample with filterA
    for (std::size_t i = 0; i < len; ++i) {
        outA[i] = filterA.processSample(input[i]);
    }

    // Process block with filterB
    filterB.processBlock(input.data(), outB.data(), len);

    // Verify exact equality
    for (std::size_t i = 0; i < len; ++i) {
        EXPECT_NEAR(outA[i], outB[i], 1e-6f);
    }
}

// =========================================================================
// Test 9: Streaming processing remains correct when blocks have different lengths
// =========================================================================
TEST(FIRFilterTest, VariableBlockSizesStreaming) {
    const std::size_t taps = 25;
    const float fc = 0.12f;
    std::vector<float> coeffs = fir::designLowPassHamming(taps, fc);

    fir::FIRFilter refFilter(coeffs);
    fir::FIRFilter streamFilter(coeffs);

    const std::size_t totalSamples = 200;
    std::vector<float> input(totalSamples);
    std::vector<float> refOutput(totalSamples);
    std::vector<float> streamOutput(totalSamples);

    for (std::size_t i = 0; i < totalSamples; ++i) {
        input[i] = std::sin(static_cast<float>(i) * 0.25f);
        refOutput[i] = refFilter.processSample(input[i]);
    }

    // Process stream in variable chunk sizes: 1, 7, 3, 16, 2, 45, 1, 5, etc.
    const std::vector<std::size_t> chunkSizes = {1, 7, 3, 16, 2, 45, 1, 5, 20, 100};
    std::size_t offset = 0;

    for (std::size_t chunkSize : chunkSizes) {
        if (offset >= totalSamples) break;
        std::size_t actualChunk = std::min(chunkSize, totalSamples - offset);
        streamFilter.processBlock(input.data() + offset, streamOutput.data() + offset, actualChunk);
        offset += actualChunk;
    }

    // Process any remaining tail
    if (offset < totalSamples) {
        streamFilter.processBlock(input.data() + offset, streamOutput.data() + offset, totalSamples - offset);
    }

    // Check all samples match reference
    for (std::size_t i = 0; i < totalSamples; ++i) {
        EXPECT_NEAR(streamOutput[i], refOutput[i], 1e-6f);
    }
}

// =========================================================================
// Test 10: Invalid cutoff frequencies and tap counts are rejected
// =========================================================================
TEST(FIRCoefficientsTest, InvalidParametersRejected) {
    // Zero taps
    EXPECT_THROW(fir::designLowPassHamming(0, 0.2f), std::invalid_argument);

    // Negative or zero cutoff
    EXPECT_THROW(fir::designLowPassHamming(31, 0.0f), std::invalid_argument);
    EXPECT_THROW(fir::designLowPassHamming(31, -0.1f), std::invalid_argument);

    // Cutoff at or above Nyquist (0.5)
    EXPECT_THROW(fir::designLowPassHamming(31, 0.5f), std::invalid_argument);
    EXPECT_THROW(fir::designLowPassHamming(31, 0.8f), std::invalid_argument);

    // Non-finite cutoff (NaN or Inf)
    EXPECT_THROW(fir::designLowPassHamming(31, std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
    EXPECT_THROW(fir::designLowPassHamming(31, std::numeric_limits<float>::infinity()), std::invalid_argument);
}

// =========================================================================
// Test 11: Even number of taps designs symmetric filter with DC unity gain
// =========================================================================
TEST(FIRCoefficientsTest, EvenTapCountDesign) {
    const std::size_t evenTaps = 32;
    const float fc = 0.2f;
    std::vector<float> coeffs = fir::designLowPassHamming(evenTaps, fc);

    EXPECT_EQ(coeffs.size(), evenTaps);

    // Check symmetry: h[n] == h[N - 1 - n]
    for (std::size_t i = 0; i < evenTaps / 2; ++i) {
        EXPECT_NEAR(coeffs[i], coeffs[evenTaps - 1 - i], 1e-5f);
    }

    // Check DC gain normalization: sum(h) == 1.0
    double sum = 0.0;
    for (float c : coeffs) sum += c;
    EXPECT_NEAR(sum, 1.0, 1e-5);
}

// =========================================================================
// Test 12: Fixed-point Q15 FIR Filter
// =========================================================================
TEST(FIRFilterQ15Test, FixedPointBasicBehavior) {
    // Design float coefficients and convert to Q15
    const std::size_t taps = 21;
    const float fc = 0.15f;
    auto floatCoeffs = fir::designLowPassHamming(taps, fc);
    auto q15Coeffs = fir::FIRFilterQ15::floatToQ15(floatCoeffs);

    fir::FIRFilterQ15 q15Filter(q15Coeffs);
    EXPECT_EQ(q15Filter.numTaps(), taps);

    // Feed constant DC input (e.g. 0.5f in Q15 = 16384)
    int16_t dcIn = 16384;
    int16_t lastOut = 0;
    for (std::size_t i = 0; i < taps * 2; ++i) {
        lastOut = q15Filter.processSample(dcIn);
    }

    // Steady state output in Q15 should be very close to 16384
    float outFloat = fir::FIRFilterQ15::q15ToFloat(lastOut);
    EXPECT_NEAR(outFloat, 0.5f, 0.02f);
}

int main() {
    return test_framework::TestRegistry::instance().runAll();
}
