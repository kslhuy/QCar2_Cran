# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.signal import butter, lfilter

# # Fonction pour créer un filtre passe-bas
# def butter_lowpass(cutoff, fs, order=5):
#     nyq = 0.5 * fs  # Fréquence de Nyquist
#     normal_cutoff = cutoff / nyq  # Normalisation de la fréquence de coupure
#     # Coefficients du filtre Butterworth
#     b, a = butter(order, normal_cutoff, btype='low', analog=False)
#     return b, a

# # Fonction pour appliquer le filtre à un signal
# def lowpass_filter(data, cutoff, fs, order=5):
#     b, a = butter_lowpass(cutoff, fs, order=order)
#     y = lfilter(b, a, data)
#     return y

# # Paramètres du filtre
# order = 6         # Ordre du filtre
# fs = 500.0        # Fréquence d'échantillonnage (en Hz)
# cutoff = 50.0     # Fréquence de coupure (en Hz)

# # Génération d'un signal d'exemple : un signal à deux fréquences
# t = np.linspace(0, 1.0, int(fs))  # 1 seconde d'échantillons
# signal = np.sin(1.2 * 2 * np.pi * t) + 1.5 * np.cos(9 * 2 * np.pi * t)

# # Application du filtre passe-bas
# filtered_signal = lowpass_filter(signal, cutoff, fs, order)

# # Affichage du signal original et du signal filtré
# plt.figure(figsize=(10,6))
# plt.plot(t, signal, label='Signal original')
# plt.plot(t, filtered_signal, label='Signal filtré', linestyle='--')
# plt.xlabel('Temps [s]')
# plt.ylabel('Amplitude')
# plt.title('Filtrage passe-bas')
# plt.legend()
# plt.grid()
# plt.show()


import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# Create a random number generator for adding noise to the signal
rng = np.random.default_rng()

# Create a time vector 't' from -1 to 1 with 201 points
t = np.linspace(-1, 1, 201)

# Generate a signal 'x' with three different sinusoidal components:
# - A sine wave with a frequency that varies over time (chirp-like)
# - A small-amplitude sine wave with a constant frequency
# - A cosine wave with a higher frequency
x = (np.sin(2*np.pi*0.75*t*(1-t) + 2.1) +  # Chirp component
     0.1*np.sin(2*np.pi*1.25*t + 1) +      # Low-frequency sine wave
     0.18*np.cos(2*np.pi*3.85*t))          # Higher-frequency cosine wave

# Add some Gaussian noise to the signal to simulate a noisy measurement
xn = x + rng.standard_normal(len(t)) * 0.08

# Design a low-pass Butterworth filter of order 3 with a normalized cutoff frequency of 0.05
b, a = signal.butter(3, 0.05)

# Initialize the filter state using the `lfilter_zi` function
# It returns the initial conditions for the filter that ensures no transient at the beginning
zi = signal.lfilter_zi(b, a)

# Apply the filter to the noisy signal 'xn' once using the `lfilter` function
# `zi` is scaled by the initial value of the signal (xn[0]) to avoid filtering artifacts
z, _ = signal.lfilter(b, a, xn, zi=zi*xn[0])

# Apply the filter a second time to the output of the first filtering step
z2, _ = signal.lfilter(b, a, z, zi=zi*z[0])

# Apply zero-phase filtering using `filtfilt`, which performs forward and backward filtering
# This removes phase distortions introduced by the filter
y = signal.filtfilt(b, a, xn)

# Time domain
plt.subplot(2, 1, 1)
plt.plot(t, xn, 'b', alpha=0.75, label='Noisy Signal')  # Noisy signal
plt.plot(t, z, 'r--', label='Filtered Once')  # Filtered signal (once)
plt.plot(t, z2, 'r', label='Filtered Twice')  # Filtered signal (twice)
plt.plot(t, y, 'k', label='Zero-phase Filtered')  # Zero-phase filtered signal
plt.legend(loc='best')
plt.title('Time Domain Signals')
plt.xlabel('Time (seconds)')
plt.ylabel('Amplitude')
plt.grid(True)


# Frequency analysis of the signal using FFT
N = len(t)  # Number of points in the signal
T = t[1] - t[0]  # Sampling interval (assuming uniform time intervals)
fs = 1 / T  # Sampling frequency

# Compute the FFT of the noisy signal xn
X_f = np.fft.fft(xn)
# Frequency bins corresponding to FFT values
freqs = np.fft.fftfreq(N, T)

# Only take the positive frequencies (symmetry in FFT output)
X_f = X_f[:N//2]
freqs = freqs[:N//2]

# Plotting the frequency domain (FFT result)
plt.subplot(2, 1, 2)
plt.plot(freqs, np.abs(X_f), 'b')
plt.title('Frequency Domain - FFT of Noisy Signal')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude')
plt.grid(True)

plt.tight_layout()
plt.show()
