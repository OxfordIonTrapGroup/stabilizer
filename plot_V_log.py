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
    plt.show()

if __name__=="__main__":
    # Get filename as an argument
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("-f", "--file", default="adc_log.npy",
                   help="log file name")
    args = p.parse_args()

    # Load data from file
    dirpath = os.getcwd()
    samples = np.load(dirpath+"//adc_log_data//"+args.file)

    # Plot data from file, in LSB
    plt_every_n = 1
    plt.figure(figsize=(10,10))
    plt.plot(np.arange(samples.shape[0])[::plt_every_n], samples[::plt_every_n],
             linestyle='', marker='x', markersize=1.5)
    plt.ylabel("LSB")
    plt.xlabel(r"$n^{th}$ sample")
    plt.title(r"stabilizer_current_sense open loop noise")
    plt.grid()

    # Convert LSB to uV
    # CHECK CONVERSION
    V_fact = 1e6*20.2/65535
    plt.figure(figsize=(10,10))
    plt.plot(np.arange(samples.shape[0])[::plt_every_n], V_fact * samples[::plt_every_n],
             linestyle='', marker='x', markersize=1.5)
    plt.ylabel("uV")
    plt.xlabel(r"$n^{th}$ sample")
    plt.title(r"Magnetic field noise from PSU")
    plt.grid()

    print("np.diff(data).std()/2**.5 in LSB", np.diff(samples).std()/2**.5)
    print("np.diff(data).std()/2**.5 in uV", np.diff(samples).std()/2**.5 *V_fact)

    print("rms in LSB", samples.std())
    print("rms in uV", samples.std() *V_fact)

    # Compute the noise spectral density
    sample_rate = 5e5
    t = np.arange(samples.shape[0]) / sample_rate
    plt.figure(figsize=(10,10))
    noise = (samples - samples.mean()) * V_fact
    f, nsd = normalised_fft(t, noise)

    nsd = np.abs(nsd) / np.sqrt(np.max(t))
    plt.loglog(f[::plt_every_n],
             savgol_filter(nsd,
                           np.max((plt_every_n, 10))+1,
                           1)[::plt_every_n])

    plt.ylabel(r"NSD [uV/sqrt(Hz)]")
    plt.xlabel(r"frequency /Hz")
    plt.title(r"Noise from PSU")
    plt.grid()
    plt.show()

    print("test uV rms from NSD", np.sqrt(2*(f[1]-f[0])*np.sum(nsd**2)) )
