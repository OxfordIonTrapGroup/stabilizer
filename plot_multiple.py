"""Some plotting of adc-logs"""

import numpy as np
import math
import os
from matplotlib import pyplot as plt
from numpy.fft import rfft, rfftfreq
from scipy.signal import savgol_filter

def normalised_fft(t, y):
    """Noramlised FFT in signal processing convention"""
    f = rfftfreq(len(t), t[1]-t[0])
    y_ = rfft(y)/(max(f)*2)
    return f, y_
if False:
    x = np.linspace(-10,100,10000)
    y = np.exp(- x*x/2)
    plt.plot(x, np.sqrt(2*np.pi) * np.exp(-x*x*2*np.pi**2))
    plt.plot(*np.abs(normalised_fft(x,y)))
    print(max(np.abs(normalised_fft(x,y))[1]))

if __name__=="__main__":
    # Get filename as an argument
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("-f", "--files", nargs='+', default=["adc_log.npy"], type=str,
                   help="Names of files to plot")
    args = p.parse_args()

    plt.figure(figsize=(10,10))
    for filename in args.files:
        # Load data from file
        dirpath = os.getcwd()
        samples = np.load(dirpath+"//adc_log_data//"+filename)

        # Convert LSB to uV and plot
        # CHECK CONVERSION
        V_fact = 1e6*20.2/65535

        # Compute the noise spectral density
        plt_every_n = 1
        sample_rate = 5e5
        t = np.arange(samples.shape[0]) / sample_rate
        noise = (samples - samples.mean()) * V_fact
        f, nsd = normalised_fft(t, noise)

        nsd = np.abs(nsd) / np.sqrt(np.max(t))
        plt.loglog(f[::plt_every_n],
                 savgol_filter(nsd,
                               np.max((plt_every_n, 10))+1,
                               1)[::plt_every_n])

    # plt.ylabel(r"NSD [uV/sqrt(Hz)]")
    # plt.xlabel(r"frequency /Hz")
    # plt.title(r"Noise from PSU")
    # plt.grid()

    plt.show()