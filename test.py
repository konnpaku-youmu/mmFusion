import numpy as np
import matplotlib.pyplot as plt


def fft(src):
    pass


def squareWave(length, periods, amplitude):
    signal = [0] * length
    for n in range(periods):
        signal[n*(length//periods):n*(length//periods)+(length//(2*periods))] = \
            [amplitude] * (length//(2*periods))
        pass
    return signal


def attenuate(src, factor):
    return [num * (factor ** idx) for idx, num in enumerate(src)]

x_n = squareWave(256, 40, 2)
x_n = attenuate(x_n, 0.2)
fft = np.fft.fft(x_n)
plt.stem([20*np.log10(abs(z)) for z in fft])
plt.yscale('log')
plt.show()
