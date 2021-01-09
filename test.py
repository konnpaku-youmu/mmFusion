import os
import ctypes
import numpy as np
import matplotlib.pyplot as plt

samples = ""

with open("/home/hcrd/Projects/mmFusion/rx_0_chirp.tsv") as f:
    samples = f.readlines()

sample_2d = []

for i, sample in enumerate(samples):
    sample = sample.split(" ")
    sample = list(filter(lambda a: a != '', sample))

    sample = [complex(eval(elem)[0], eval(elem)[1]) for elem in sample]

    sample_real = [elem.real for elem in sample]
    sample_imag = [elem.imag for elem in sample]

    signal = []

    for I, Q in list(zip(sample_real, sample_imag)):
        signal.append(I + Q * 1j)


    signal_amp = [abs(sig) for sig in signal]
    sig_fft = np.fft.fft(signal)

    sig_fft_amp = [abs(num) for num in sig_fft]

    sample_2d.append(max(sig_fft_amp))

    plt.subplot(8,4,i+1)
    plt.title("Chirp {}".format(i))
    plt.stem(sig_fft_amp)

# print(sample_2d)
# fft_2d = np.fft.fft(sample_2d)
# fft_2d_amp = [abs(num) for num in fft_2d]
# plt.stem(fft_2d_amp)
plt.show()
