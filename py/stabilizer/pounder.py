PHASE_TURNS_PER_LSB = 1/(1 << 14)

def phase_to_pow(phase_turns):
    """Phase in turns to phase offset word"""
    return int((phase_turns / PHASE_TURNS_PER_LSB) & 0x3FFF)

def pow_to_phase(pow):
    return pow * PHASE_TURNS_PER_LSB

def amplitude_to_acr(amplitude):
    if not (0.0 <= amplitude <= 1.0):
        raise ValueError("Amplitude must be between 0.0 and 1.0")

    acr = int(amplitude * (1 << 10)) & 0x3FF
    acr |= ((amplitude != 1.0 ) << 12)  # Set the amplitude bit

    return int(acr)

def frequency_to_ftw(dds_frequency, system_clock_frequency):
    if not (0.0 <= dds_frequency <= system_clock_frequency / 2.0):
        raise ValueError("DDS frequency must not exceed the Nyquist frequency.")

    # The function for channel frequency is `f_out = FTW * f_s / 2^32`, where FTW is the
    # frequency tuning word and f_s is the system clock rate.
    return int((dds_frequency / system_clock_frequency) * (1 << 32))

def ftw_to_frequency(tuning_word, system_clock_frequency):
    return (tuning_word / (1 << 32)) * system_clock_frequency

def acr_to_amplitude(acr):
    if acr > 0x3FF:
        raise ValueError("ACR must be less than or equal to 0x3FF")

    if (acr & 0x1000) == 0:
        return 1.0
    else:
        return (acr & 0x3FF) / (1 << 10)