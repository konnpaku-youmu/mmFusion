import os
import ctypes
import numpy as np
import matplotlib.pyplot as plt

sample = ""

with open("./sample.txt") as f:
    sample = f.readline()
sample = sample.split(" ")
sample = list(filter(lambda a: a != '', sample))



sample = [complex(eval(elem)[0], eval(elem)[1]) for elem in sample]

sample_real = [elem.real for elem in sample]
sample_r = []
for num in sample_real:
    _n = num if num < 32767 else num - 65536
    sample_r.append(_n)

sample_i = []
sample_imag = [elem.imag for elem in sample]
for num in sample_imag:
    _n = num if num < 32767 else num - 65536
    sample_i.append(_n)

signal = []

for I, Q in list(zip(sample_r, sample_i)):
    signal.append(complex(I,Q))

sig_fft = np.fft.fft(signal)

sig_fft_amp = [abs(num) for num in sig_fft]

plt.stem(sig_fft_amp)
plt.show()
