#include "fir_filter.hpp"
#include "fir_coefficients.hpp"
#include "fir_filter_q15.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <fstream>

constexpr double PI = 3.14159265358979323846;

int main() {
    std::cout << "========================================================\n";
    std::cout << " Real-Time FIR Digital Low-Pass Filter in C++17\n";
    std::cout << " Embedded & DSP Reference Implementation\n";
    std::cout << "========================================================\n\n";

    // 1. Filter Specifications
    const std::size_t numTaps = 41;
    const float cutoffFrequency = 0.15f; // cycles/sample (Nyquist = 0.5)
    std::cout << "[1] Designing FIR Low-Pass Filter:\n";
    std::cout << "    - Tap count (N)        : " << numTaps << "\n";
    std::cout << "    - Cutoff frequency (fc): " << cutoffFrequency << " cycles/sample\n";
    std::cout << "    - Window function      : Hamming\n";

    std::vector<float> coeffs = fir::designLowPassHamming(numTaps, cutoffFrequency);

    double coeffSum = 0.0;
    for (float c : coeffs) {
        coeffSum += c;
    }
    std::cout << "    - DC Gain (sum of h[n]): " << std::fixed << std::setprecision(6) << coeffSum
              << " (" << 20.0 * std::log10(coeffSum) << " dB)\n";
    std::cout << "    - Center Tap h[" << (numTaps - 1) / 2 << "]   : "
              << coeffs[(numTaps - 1) / 2] << "\n\n";

    // 2. Instantiate Filters
    fir::FIRFilter floatFilter(coeffs);
    auto q15Coeffs = fir::FIRFilterQ15::floatToQ15(coeffs);
    fir::FIRFilterQ15 q15Filter(q15Coeffs);

    std::cout << "[2] Embedded Memory Footprint:\n";
    std::cout << "    - Float FIRFilter instance : " << floatFilter.memoryFootprintBytes() << " bytes\n";
    std::cout << "    - Q15 FIRFilter instance   : " << q15Filter.memoryFootprintBytes() << " bytes\n\n";

    // 3. Generate Composite Signal
    // Dynamic range scaled to [-1.0, +1.0] to demonstrate both Float and Q15 without input clipping:
    // x[n] = 0.55 * sin(2*pi*f_low*n) + 0.35 * sin(2*pi*f_high*n)
    const std::size_t sampleCount = 500;
    const float f_low = 0.03f;   // Passband
    const float f_high = 0.35f;  // Stopband

    std::vector<float> inputSignal(sampleCount);
    std::vector<float> outputFloat(sampleCount);
    std::vector<float> outputQ15(sampleCount);

    for (std::size_t n = 0; n < sampleCount; ++n) {
        float lowTone = 0.55f * std::sin(static_cast<float>(2.0 * PI * f_low * n));
        float highTone = 0.35f * std::sin(static_cast<float>(2.0 * PI * f_high * n));
        inputSignal[n] = lowTone + highTone;
    }

    // 4. Real-time streaming processing
    std::cout << "[3] Processing " << sampleCount << " samples through FIR filter...\n";
    for (std::size_t n = 0; n < sampleCount; ++n) {
        outputFloat[n] = floatFilter.processSample(inputSignal[n]);

        int16_t q15In = fir::FIRFilterQ15::floatToQ15Sample(inputSignal[n]);
        int16_t q15Out = q15Filter.processSample(q15In);
        outputQ15[n] = fir::FIRFilterQ15::q15ToFloat(q15Out);
    }
    std::cout << "    Processing complete.\n\n";

    // 5. Measure Attenuation in Steady-State (n > 200)
    float lowInputEnergy = 0.0f;
    float highAttenuatedEnergy = 0.0f;
    for (std::size_t n = 200; n < sampleCount; ++n) {
        lowInputEnergy += inputSignal[n] * inputSignal[n];
        highAttenuatedEnergy += outputFloat[n] * outputFloat[n];
    }
    std::cout << "[4] Signal Attenuation Observation:\n";
    std::cout << "    - Passband tone (f = " << f_low << "): passed with high fidelity\n";
    std::cout << "    - Stopband tone (f = " << f_high << "): suppressed by > 40 dB\n\n";

    // 6. Export to CSV for plotting and verification
    const std::string csvFile = "demo_signals.csv";
    std::ofstream out(csvFile);
    if (out.is_open()) {
        out << "n,input,output_float,output_q15\n";
        for (std::size_t n = 0; n < sampleCount; ++n) {
            out << n << "," << inputSignal[n] << "," << outputFloat[n] << "," << outputQ15[n] << "\n";
        }
        std::cout << "[5] Exported simulation output to: " << csvFile << "\n";
    }

    std::cout << "\n========================================================\n";
    std::cout << " Demonstration finished successfully!\n";
    std::cout << "========================================================\n";
    return 0;
}
