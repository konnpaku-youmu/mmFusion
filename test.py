import os
import ctypes
import numpy as np
import matplotlib.pyplot as plt

samples = ""

with open("./sample.txt") as f:
    samples = f.readlines()

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

    plt.subplot(2,2,i+1)
    plt.title("Rx {}".format(i))
    # plt.plot(sample_r)
    # plt.plot(sample_i)
    # plt.subplot(2,2,2)
    plt.stem(sig_fft_amp)

plt.show()
