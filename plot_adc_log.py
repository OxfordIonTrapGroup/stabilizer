"""Some plotting of adc-logs"""

import numpy as np
import math
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
# if False:
    plt_every_n = 1

    samples = np.load("adc_log.npy")
    # samples = np.load("open_loop_noise.npy")


    plt.figure(figsize=(10,10))
    plt.plot(np.arange(samples.shape[0])[::plt_every_n], samples[::plt_every_n],
             linestyle='', marker='x', markersize=1.5)
    plt.ylabel("LSB")
    plt.xlabel(r"$n^{th}$ sample")
    plt.title(r"stabilizer_current_sense open loop noise")
    plt.grid()
    plt.savefig("LSB_log.png", dpi=600)
    plt.savefig("LSB_log.pdf")
    # plt.show()

    # LSB to V at ADC
    v_fact = 2 * 4.096 / 2**16
    # ADC voltage to input voltage
    v_fact *= 5 / 2  # /5 op-amp and inverting unity amp on negative side
    v_fact /= 1  # instrumentation amplifier
    plt.figure(figsize=(10,10))
    plt.plot(np.arange(samples.shape[0])[::plt_every_n], v_fact * samples[::plt_every_n],
             linestyle='', marker='x', markersize=1.5)
    plt.ylabel("Volt")
    plt.xlabel(r"$n^{th}$ sample")
    plt.title(r"stabilizer_current_sense open loop noise")
    plt.grid()
    plt.savefig("V_log.png", dpi=600)
    plt.savefig("V_log.pdf")
    # plt.show()

    print("np.diff(data).std()/2**.5 in LSB", np.diff(samples).std()/2**.5)
    print("np.diff(data).std()/2**.5 in V", np.diff(samples).std()/2**.5 *v_fact)

    print("rms in LSB", samples.std())
    print("rms in V", samples.std() *v_fact)

    sample_rate = 5e5
    t = np.arange(samples.shape[0]) / sample_rate
    plt.figure(figsize=(10,10))
    noise = (samples - samples.mean()) * v_fact
    f, nsd = normalised_fft(t, noise)

    nsd = np.abs(nsd) / np.sqrt(np.max(t))
    plt.semilogx(f[::plt_every_n],
             savgol_filter(nsd,
                           np.max((plt_every_n, 10))+1,
                           1)[::plt_every_n]*1e6)
    # plt.xlim([0,5e3])

    # noise = (samples[:10000] - samples[:10000].mean()) * v_fact
    # f, nsd = normalised_fft(t[:10000], noise)

    # nsd = np.abs(nsd) / np.sqrt(np.max(t[:10000]))
    # plt.plot(f[::], nsd[::]/2000)



    plt.ylabel(r"NSD [$\mu$Volt/sqrt(Hz)]")
    plt.xlabel(r"frequency /Hz")
    plt.title(r"stabilizer_current_sense open loop noise")
    plt.grid()
    plt.savefig("nsd.png", dpi=600)
    plt.savefig("nsd.pdf")
    plt.show()
    # np.save("noise_spectrum.npy",savgol_filter(nsd,
                           #np.max((plt_every_n, 10))+1,
                           #1)[::plt_every_n]*1e6)
    #np.save("f.npy",f)
    print("test rms voltage from NSD", np.sqrt(2*(f[1]-f[0])*np.sum(nsd**2)) )
