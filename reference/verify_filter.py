#!/usr/bin/env python3
"""
FIR Digital Low-Pass Filter - Python Reference Verification and Plotting
========================================================================

This script:
1. Designs reference Hamming-window FIR coefficients using NumPy.
2. Generates a multi-frequency composite test signal (passband + stopband tones).
3. Performs reference convolution using NumPy (`np.convolve`).
4. Compares Python reference output against C++ FIR implementation output (from demo_signals.csv).
5. Computes Maximum Absolute Error (MAE) and Root Mean Square Error (RMSE).
6. Generates publication-quality plots:
   - Time-domain input vs. filtered output vs. residual error.
   - Frequency response (Magnitude in dB and Phase in degrees) showing passband,
     cutoff frequency fc, transition band, and stopband attenuation.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

def design_fir_hamming(num_taps: int, fc: float) -> np.ndarray:
    """
    Computes Hamming-windowed low-pass FIR filter coefficients.
    Normalized cutoff fc in cycles/sample (0 < fc < 0.5).
    """
    n = np.arange(num_taps)
    center = (num_taps - 1) / 2.0
    m = n - center

    # Sinc impulse response
    # Avoid divide-by-zero RuntimeWarning in NumPy vectorized evaluation
    denom = np.where(np.abs(m) < 1e-12, 1.0, np.pi * m)
    num = np.sin(2.0 * np.pi * fc * m)
    h_ideal = np.where(np.abs(m) < 1e-12, 2.0 * fc, num / denom)

    # Hamming window: w[n] = 0.54 - 0.46 * cos(2 * pi * n / (N - 1))
    if num_taps > 1:
        w = 0.54 - 0.46 * np.cos(2.0 * np.pi * n / (num_taps - 1))
    else:
        w = np.array([1.0])

    h_windowed = h_ideal * w

    # DC Gain Normalization: sum(h) == 1.0
    h_norm = h_windowed / np.sum(h_windowed)
    return h_norm

def compute_frequency_response(h: np.ndarray, num_points: int = 1024):
    """
    Computes Discrete-Time Fourier Transform (DTFT) of filter impulse response h.
    Returns normalized frequency w (0 to 0.5 cycles/sample), magnitude in dB, and phase in degrees.
    """
    frequencies = np.linspace(0.0, 0.5, num_points)
    # H(w) = sum_{k=0}^{N-1} h[k] * exp(-j * 2 * pi * w * k)
    k = np.arange(len(h))
    dtft = np.zeros(num_points, dtype=complex)
    for i, f in enumerate(frequencies):
        dtft[i] = np.sum(h * np.exp(-1j * 2.0 * np.pi * f * k))

    magnitude_db = 20.0 * np.log10(np.maximum(np.abs(dtft), 1e-10))
    phase_deg = np.unwrap(np.angle(dtft)) * (180.0 / np.pi)
    return frequencies, magnitude_db, phase_deg

def main():
    print("================================================================")
    print(" FIR Filter Python Reference Verification & Performance Analysis")
    print("================================================================")

    num_taps = 41
    fc = 0.15  # cycles/sample (Nyquist = 0.5)
    f_low = 0.03
    f_high = 0.35
    num_samples = 500

    print(f"Filter Parameters: Taps N={num_taps}, Cutoff fc={fc} cycles/sample")

    # 1. Design Reference Coefficients
    h_ref = design_fir_hamming(num_taps, fc)
    print(f"Reference DC gain sum(h): {np.sum(h_ref):.8f}")

    # 2. Synthetic Test Signal within [-1.0, 1.0] dynamic range
    n = np.arange(num_samples)
    input_signal = 0.55 * np.sin(2.0 * np.pi * f_low * n) + 0.35 * np.sin(2.0 * np.pi * f_high * n)

    # 3. Reference Convolution (causal FIR, length equal to input)
    y_ref = np.convolve(input_signal, h_ref, mode='full')[:num_samples]

    # 4. Check for C++ Generated Signals
    csv_paths = [
        "demo_signals.csv",
        os.path.join("..", "demo_signals.csv"),
        os.path.join(os.path.dirname(__file__), "..", "build", "demo_signals.csv"),
        os.path.join(os.path.dirname(__file__), "..", "demo_signals.csv")
    ]
    csv_file = None
    for p in csv_paths:
        if os.path.exists(p):
            csv_file = p
            break

    y_cpp = None
    y_q15 = None
    if csv_file:
        print(f"Found C++ simulation output: {csv_file}")
        data = np.genfromtxt(csv_file, delimiter=',', skip_header=1)
        # Columns: n, input, output_float, output_q15
        cpp_in = data[:, 1]
        y_cpp = data[:, 2]
        y_q15 = data[:, 3]

        # Verify numerical match with C++ output
        max_abs_err = np.max(np.abs(y_cpp - y_ref))
        rmse = np.sqrt(np.mean((y_cpp - y_ref) ** 2))
        print("\n--- Numerical Verification Results (C++ vs Python Reference) ---")
        print(f"Maximum Absolute Error (MAE) : {max_abs_err:.4e}")
        print(f"Root Mean Square Error (RMSE) : {rmse:.4e}")

        q15_max_err = np.max(np.abs(y_q15 - y_ref))
        q15_rmse = np.sqrt(np.mean((y_q15 - y_ref) ** 2))
        print(f"Fixed-Point Q15 Max Error    : {q15_max_err:.4e}")
        print(f"Fixed-Point Q15 RMSE         : {q15_rmse:.4e}")

        if max_abs_err < 1e-4:
            print(">>> SUCCESS: C++ FIR output matches Python reference within floating-point tolerance!")
        else:
            print(">>> WARNING: Discrepancy detected between C++ and Python output.")
    else:
        print("Note: demo_signals.csv not found yet. Using Python-simulated outputs for visualization.")
        y_cpp = y_ref

    # 5. Plot 1: Time-Domain Signals & Residuals
    output_dir = os.path.dirname(os.path.abspath(__file__))
    os.makedirs(output_dir, exist_ok=True)

    plt.figure(figsize=(12, 7), dpi=150)
    plt.subplot(2, 1, 1)
    plt.plot(n[:150], input_signal[:150], 'b-', alpha=0.5, label='Input Signal (f_low=0.03 + f_high=0.35)')
    plt.plot(n[:150], y_ref[:150], 'r-', linewidth=2, label='Python Reference Convolution')
    if y_cpp is not None and csv_file:
        plt.plot(n[:150], y_cpp[:150], 'g--', linewidth=1.5, label='C++ FIRFilter (Float)')
        plt.plot(n[:150], y_q15[:150], 'm:', linewidth=1.5, label='C++ FIRFilterQ15 (Fixed-Point)')
    plt.title('FIR Digital Low-Pass Filter: Time-Domain Transient and Steady-State Response', fontsize=13, fontweight='bold')
    plt.xlabel('Sample Index (n)', fontsize=11)
    plt.ylabel('Amplitude', fontsize=11)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(loc='upper right', framealpha=0.9)

    plt.subplot(2, 1, 2)
    if y_cpp is not None and csv_file:
        residual = y_cpp - y_ref
        plt.plot(n, residual, 'k-', label=f'Error (y_cpp - y_ref), Max Abs Err: {max_abs_err:.2e}')
        plt.ylabel('Error Residual', fontsize=11)
        plt.xlabel('Sample Index (n)', fontsize=11)
        plt.title('Numerical Residual: C++ Single-Precision vs Python Double-Precision Reference', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc='upper right')
    else:
        # Plot filtered signal only
        plt.plot(n, y_ref, 'r-', label='Filtered Signal y[n]')
        plt.xlabel('Sample Index (n)', fontsize=11)
        plt.ylabel('Amplitude', fontsize=11)
        plt.title('Filtered Output Signal (Full Duration)', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc='upper right')

    plt.tight_layout()
    time_plot_path = os.path.join(output_dir, "signals_comparison.png")
    plt.savefig(time_plot_path)
    plt.close()
    print(f"Saved time-domain verification plot to: {time_plot_path}")

    # 6. Plot 2: Frequency Response (Bode Plot)
    freqs, mag_db, phase_deg = compute_frequency_response(h_ref, 2048)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), dpi=150)

    # Magnitude
    ax1.plot(freqs, mag_db, 'b-', linewidth=2, label='Filter Magnitude Response |H(f)|')
    ax1.axvline(fc, color='r', linestyle='--', linewidth=1.5, label=f'Cutoff Frequency fc = {fc}')
    ax1.axhline(0, color='gray', linestyle=':', alpha=0.7)
    ax1.axhline(-3, color='orange', linestyle=':', label='-3 dB Half-Power Level')

    # Shade passband and stopband
    ax1.axvspan(0.0, fc, color='green', alpha=0.1, label='Passband (Minimal Attenuation)')
    ax1.axvspan(fc + 0.08, 0.5, color='red', alpha=0.1, label='Stopband (> 40 dB Attenuation)')

    ax1.set_title(f'FIR Low-Pass Filter Frequency Response (Hamming Window, N={num_taps} Taps)', fontsize=13, fontweight='bold')
    ax1.set_ylabel('Magnitude (dB)', fontsize=11)
    ax1.set_xlim([0.0, 0.5])
    ax1.set_ylim([-80, 5])
    ax1.grid(True, which='both', linestyle='--', alpha=0.6)
    ax1.legend(loc='upper right', framealpha=0.9)

    # Phase
    ax2.plot(freqs, phase_deg, 'purple', linewidth=1.8, label='Phase Response (Linear Phase / Constant Group Delay)')
    ax2.axvline(fc, color='r', linestyle='--', linewidth=1.5)
    ax2.set_xlabel('Normalized Frequency (cycles/sample, Nyquist = 0.5)', fontsize=11)
    ax2.set_ylabel('Unwrapped Phase (degrees)', fontsize=11)
    ax2.set_xlim([0.0, 0.5])
    ax2.grid(True, which='both', linestyle='--', alpha=0.6)
    ax2.legend(loc='upper right', framealpha=0.9)

    plt.tight_layout()
    bode_plot_path = os.path.join(output_dir, "frequency_response.png")
    plt.savefig(bode_plot_path)
    plt.close()
    print(f"Saved frequency response plot to: {bode_plot_path}")
    print("Verification completed successfully.\n")

if __name__ == "__main__":
    main()
