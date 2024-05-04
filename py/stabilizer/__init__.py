#!/usr/bin/python3
"""Stabilizer data conversion and streaming utilities"""

# Number of timer ticks per sample various applications
DUAL_IIR_SAMPLE_TICKS = 128
LOCKIN_SAMPLE_TICKS = 128
L674_SAMPLE_TICKS = 128
FNC_SAMPLE_TICKS = 500

# Sample period in seconds for the default 100 MHz timer clock for various applications
DEFAULT_DUAL_IIR_SAMPLE_PERIOD = 10e-9*DUAL_IIR_SAMPLE_TICKS
DEFAULT_LOCKIN_SAMPLE_PERIOD = 10e-9*LOCKIN_SAMPLE_TICKS
DEFAULT_L674_SAMPLE_PERIOD = 10e-9*L674_SAMPLE_TICKS
DEFAULT_FNC_SAMPLE_PERIOD = 10e-9*FNC_SAMPLE_TICKS

# The number of DAC LSB codes per volt on Stabilizer outputs.
DAC_LSB_PER_VOLT = (1 << 16) / (4.096 * 5)

# The number of volts per ADC LSB.
ADC_VOLTS_PER_LSB = (5.0 / 2.0 * 4.096)  / (1 << 15)

# The number of volts per DAC LSB.
DAC_VOLTS_PER_LSB = 1 / DAC_LSB_PER_VOLT

# The absolute full-scale output voltage in either positive or negative direction exposed by the
# DAC.
DAC_FULL_SCALE = float(0x7FFF / DAC_LSB_PER_VOLT)

def voltage_to_machine_units(voltage):
    """Convert a voltage to machine units."""
    code = int(round(voltage * DAC_LSB_PER_VOLT))
    if abs(code) > 0x7FFF:
        raise ValueError(f"Voltage out-of-range ({hex(code)}")
    return code
