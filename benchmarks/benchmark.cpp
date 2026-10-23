#include "fir_filter.hpp"
#include "fir_coefficients.hpp"
#include "fir_filter_q15.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <numeric>
#include <cmath>
#include <cstring>

// Pseudo-random linear congruential generator for reproducible input signals
static inline float nextSample(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(static_cast<int32_t>(state)) / 2147483648.0f;
}

struct BenchmarkResult {
    std::string testName;
    std::size_t numTaps;
    std::size_t sampleCount;
    double elapsedSeconds;
    double throughputMSamplesPerSec;
    double nsPerSample;
    float checksum;
};

void printResultRow(const BenchmarkResult& res) {
    uint32_t rawChecksum = 0;
    std::memcpy(&rawChecksum, &res.checksum, sizeof(uint32_t));
    std::cout << "| " << std::left << std::setw(30) << res.testName
              << " | " << std::right << std::setw(6) << res.numTaps
              << " | " << std::setw(10) << res.sampleCount
              << " | " << std::fixed << std::setprecision(3) << std::setw(10) << (res.elapsedSeconds * 1000.0)
              << " | " << std::fixed << std::setprecision(2) << std::setw(12) << res.throughputMSamplesPerSec
              << " | " << std::fixed << std::setprecision(2) << std::setw(10) << res.nsPerSample
              << " | " << std::hex << rawChecksum << std::dec
              << " |\n";
}

int main() {
    std::cout << "=========================================================================================\n";
    std::cout << " FIR Digital Filter Performance Benchmark\n";
    std::cout << "=========================================================================================\n";

    // Compiler / platform details
    std::cout << "Compiler Information:\n";
#if defined(__clang__)
    std::cout << "  Compiler : Clang " << __clang_version__ << "\n";
#elif defined(__GNUC__)
    std::cout << "  Compiler : GCC " << __VERSION__ << "\n";
#elif defined(_MSC_VER)
    std::cout << "  Compiler : MSVC " << _MSC_VER << "\n";
#else
    std::cout << "  Compiler : Standard C++17 Compiler\n";
#endif
    std::cout << "  Target   : x86_64 Desktop Benchmark\n";
    std::cout << "  Note     : Desktop numbers illustrate relative throughput; hard real-time guarantees\n";
    std::cout << "             require deterministic microcontroller timer peripherals and RTOS trace.\n\n";

    const std::size_t sampleCount = 2000000; // 2 million samples
    std::cout << "Generating " << sampleCount << " reproducible test samples...\n";
    std::vector<float> inputSignal(sampleCount);
    uint32_t seed = 123456789u;
    for (std::size_t i = 0; i < sampleCount; ++i) {
        inputSignal[i] = nextSample(seed);
    }
    std::vector<float> outputBuffer(sampleCount, 0.0f);

    std::cout << "Initialization complete.\n\n";

    std::cout << "| " << std::left << std::setw(30) << "Benchmark Test"
              << " | " << std::right << std::setw(6) << "Taps"
              << " | " << std::setw(10) << "Samples"
              << " | " << std::setw(10) << "Time (ms)"
              << " | " << std::setw(12) << "MSamples/s"
              << " | " << std::setw(10) << "ns/Sample"
              << " | " << std::setw(8) << "Checksum"
              << " |\n";
    std::cout << "|--------------------------------|--------|------------|------------|--------------|------------|----------|\n";

    const std::vector<std::size_t> tapConfigs = {15, 31, 63, 127};

    // 1. Benchmark Sample-by-Sample Streaming
    for (std::size_t taps : tapConfigs) {
        auto coeffs = fir::designLowPassHamming(taps, 0.15f);
        fir::FIRFilter filter(coeffs);

        // Warm-up phase
        for (std::size_t i = 0; i < 10000; ++i) {
            filter.processSample(inputSignal[i]);
        }
        filter.reset();

        volatile float accumulator = 0.0f;
        auto start = std::chrono::high_resolution_clock::now();

        for (std::size_t i = 0; i < sampleCount; ++i) {
            float y = filter.processSample(inputSignal[i]);
            accumulator += y;
        }

        auto end = std::chrono::high_resolution_clock::now();
        double elapsedSec = std::chrono::duration<double>(end - start).count();

        BenchmarkResult res;
        res.testName = "Float Stream (Sample-by-Sample)";
        res.numTaps = taps;
        res.sampleCount = sampleCount;
        res.elapsedSeconds = elapsedSec;
        res.throughputMSamplesPerSec = (static_cast<double>(sampleCount) / elapsedSec) / 1e6;
        res.nsPerSample = (elapsedSec / static_cast<double>(sampleCount)) * 1e9;
        res.checksum = accumulator;

        printResultRow(res);
    }

    // 2. Benchmark Block Processing (Block Size = 256)
    const std::size_t blockSize = 256;
    for (std::size_t taps : tapConfigs) {
        auto coeffs = fir::designLowPassHamming(taps, 0.15f);
        fir::FIRFilter filter(coeffs);

        // Warm-up
        filter.processBlock(inputSignal.data(), outputBuffer.data(), 10000);
        filter.reset();

        volatile float accumulator = 0.0f;
        auto start = std::chrono::high_resolution_clock::now();

        std::size_t offset = 0;
        while (offset < sampleCount) {
            std::size_t currentChunk = std::min(blockSize, sampleCount - offset);
            filter.processBlock(inputSignal.data() + offset, outputBuffer.data() + offset, currentChunk);
            offset += currentChunk;
        }

        for (std::size_t i = 0; i < sampleCount; i += 100) {
            accumulator += outputBuffer[i];
        }

        auto end = std::chrono::high_resolution_clock::now();
        double elapsedSec = std::chrono::duration<double>(end - start).count();

        BenchmarkResult res;
        res.testName = "Float Block (Size 256)";
        res.numTaps = taps;
        res.sampleCount = sampleCount;
        res.elapsedSeconds = elapsedSec;
        res.throughputMSamplesPerSec = (static_cast<double>(sampleCount) / elapsedSec) / 1e6;
        res.nsPerSample = (elapsedSec / static_cast<double>(sampleCount)) * 1e9;
        res.checksum = accumulator;

        printResultRow(res);
    }

    // 3. Benchmark Fixed-Point Q15 Stream
    std::vector<int16_t> q15Input(sampleCount);
    for (std::size_t i = 0; i < sampleCount; ++i) {
        q15Input[i] = fir::FIRFilterQ15::floatToQ15Sample(inputSignal[i] * 0.9f);
    }

    for (std::size_t taps : tapConfigs) {
        auto coeffs = fir::designLowPassHamming(taps, 0.15f);
        auto q15Coeffs = fir::FIRFilterQ15::floatToQ15(coeffs);
        fir::FIRFilterQ15 filter(q15Coeffs);

        // Warm-up
        for (std::size_t i = 0; i < 10000; ++i) {
            filter.processSample(q15Input[i]);
        }
        filter.reset();

        volatile int32_t accumulator = 0;
        auto start = std::chrono::high_resolution_clock::now();

        for (std::size_t i = 0; i < sampleCount; ++i) {
            int16_t y = filter.processSample(q15Input[i]);
            accumulator += y;
        }

        auto end = std::chrono::high_resolution_clock::now();
        double elapsedSec = std::chrono::duration<double>(end - start).count();

        BenchmarkResult res;
        res.testName = "Fixed-Point Q15 Stream";
        res.numTaps = taps;
        res.sampleCount = sampleCount;
        res.elapsedSeconds = elapsedSec;
        res.throughputMSamplesPerSec = (static_cast<double>(sampleCount) / elapsedSec) / 1e6;
        res.nsPerSample = (elapsedSec / static_cast<double>(sampleCount)) * 1e9;
        res.checksum = static_cast<float>(accumulator);

        printResultRow(res);
    }

    std::cout << "|--------------------------------|--------|------------|------------|--------------|------------|----------|\n\n";
    return 0;
}
