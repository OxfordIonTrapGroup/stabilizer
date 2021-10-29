from math import pi

# The base Stabilizer tick rate in Hz.
STABILIZER_TICK_RATE = 100e6

def calculate_notch_coefficients(sampling_period, f0, Q, K):
    """ Calculate notch IIR filter coefficients. """
    f0_bar = pi * f0 * sampling_period

    denominator = (1 + f0_bar / Q + f0_bar ** 2)

    a1 = 2 * (1 - f0_bar ** 2) / denominator
    a2 = - (1 - f0_bar / Q + f0_bar ** 2) / denominator
    b0 = K * (1 + f0_bar ** 2) / denominator
    b1 = - (2 * K * (1 - f0_bar ** 2)) / denominator
    b2 = K * (1 + f0_bar ** 2) / denominator

    return [b0, b1, b2, a1, a2]

if __name__=="__main__":
    sample_ticks = 128
    print(calculate_notch_coefficients(2e-6, 15840., 10, 1))
    print(calculate_notch_coefficients(2e-6, 15840., 1, 1))
    print(calculate_notch_coefficients(2e-6, 15840., 0.1, 1))