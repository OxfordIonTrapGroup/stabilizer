#!/usr/bin/python3
"""
Authors:
    Étienne Wodey, Leibniz University Hannover, Institute of Quantum Optics
    Ryan Summers, Vertigo Designs
    Robert Jördens, QUARTIQ

Description: Algorithms to generate biquad (second order IIR) coefficients.

All functions return `[b0, b1, b2, a1, a2]` such that
`y0 = b0*x0 + b1*x1 + b2*x2 + a1*y1 + a2*y2`, the convention used by `idsp`
for the `Raw` biquad representation (`ba`) of `dual-iir` and `fnc`.

Note that `dual-iir` also supports designing filters on-device via its `Pid`
and `Filter` biquad representations.
"""
import collections

from math import pi, inf

# Disable pylint warnings about a0, b1 etc
#pylint: disable=invalid-name


# Generic type for containing a command-line argument.
# Use `add_argument` for simple construction.
Argument = collections.namedtuple("Argument", ["positionals", "keywords"])


def add_argument(*args, **kwargs):
    """ Convert arguments into an Argument tuple. """
    return Argument(args, kwargs)


# Represents a generic filter that can be implemented by a biquad.
#
# Fields
#     * `help`: This field specifies helpful human-readable information that
#       will be presented to users on the command line.
#     * `arguments`: A list of `Argument` objects representing available
#       command-line arguments for the filter. Use the `add_argument()`
#       function to easily parse options as they would be provided to
#       argparse.
#     * `coefficients`: A function, provided with parsed arguments, that
#       returns the IIR coefficients. See below for more information on this
#       function.
#
# # Coefficients Calculation Function
#     Description:
#       This function takes in two input arguments and returns the IIR filter
#       coefficients for Stabilizer to represent the necessary filter.
#
#     Args:
#       args: The filter command-line arguments. Any filter-related arguments
#       may be accessed via their name. E.g. `args.K`.
#
#     Returns:
#       [b0, b1, b2, a1, a2] IIR coefficients to be programmed into a
#       Stabilizer IIR filter configuration.
Filter = collections.namedtuple(
    "Filter", ["help", "arguments", "coefficients"])


def get_filters():
    """ Get a dictionary of all available filters.

    Note:
        Calculations coefficient largely taken using the derivations in
        page 9 of https://arxiv.org/pdf/1508.06319.pdf

        PII/PID coefficient equations are taken from the PID-IIR primer
        written by Robert Jördens at https://hackmd.io/IACbwcOTSt6Adj3_F9bKuw
    """
    return {
        "lowpass": Filter(help="Gain-limited low-pass filter",
                          arguments=[
                              add_argument("--f0", required=True, type=float,
                                           help="Corner frequency (Hz)"),
                              add_argument("--K", required=True, type=float,
                                           help="Lowpass filter gain"),
                          ],
                          coefficients=lowpass_coefficients),
        "highpass": Filter(help="Gain-limited high-pass filter",
                           arguments=[
                               add_argument("--f0", required=True, type=float,
                                            help="Corner frequency (Hz)"),
                               add_argument("--K", required=True, type=float,
                                            help="Highpass filter gain"),
                           ],
                           coefficients=highpass_coefficients),
        "allpass": Filter(help="Gain-limited all-pass filter",
                          arguments=[
                              add_argument("--f0", required=True, type=float,
                                           help="Corner frequency (Hz)"),
                              add_argument("--K", required=True, type=float,
                                           help="Allpass filter gain"),
                          ],
                          coefficients=allpass_coefficients),
        "notch": Filter(help="Notch filter",
                        arguments=[
                            add_argument("--f0", required=True, type=float,
                                         help="Corner frequency (Hz)"),
                            add_argument("--Q", required=True, type=float,
                                         help="Filter quality factor"),
                            add_argument("--K", required=True, type=float,
                                         help="Filter gain"),
                        ],
                        coefficients=notch_coefficients),
        "pid": Filter(help="PID controller. Gains at 1 Hz and often negative.",
                      arguments=[
                          add_argument("--Kii", default=0, type=float,
                                       help="Double Integrator (I^2) gain"),
                          add_argument("--Kii_limit", default=inf, type=float,
                                       help="Integral gain limit"),
                          add_argument("--Ki", default=0, type=float,
                                       help="Integrator (I) gain"),
                          add_argument("--Ki_limit", default=inf, type=float,
                                       help="Integral gain limit"),
                          add_argument("--Kp", default=0, type=float,
                                       help="Proportional (P) gain"),
                          add_argument("--Kd", default=0, type=float,
                                       help="Derivative (D) gain"),
                          add_argument("--Kd_limit", default=inf, type=float,
                                       help="Derivative gain limit"),
                          add_argument("--Kdd", default=0, type=float,
                                       help="Double Derivative (D^2) gain"),
                          add_argument("--Kdd_limit", default=inf, type=float,
                                       help="Derivative gain limit"),
                      ],
                      coefficients=pid_coefficients),
    }


def lowpass_coefficients(args):
    """Calculate low-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = args.K * f0_bar / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def highpass_coefficients(args):
    """Calculate high-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K / (1 + f0_bar)
    b1 = - args.K / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def allpass_coefficients(args):
    """Calculate all-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)

    b0 = args.K * (1 - f0_bar) / (1 + f0_bar)
    b1 = - args.K

    return [b0, b1, 0, a1, 0]


def notch_coefficients(args):
    """Calculate notch IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    denominator = 1 + f0_bar / args.Q + f0_bar ** 2

    a1 = 2 * (1 - f0_bar ** 2) / denominator
    a2 = - (1 - f0_bar / args.Q + f0_bar ** 2) / denominator
    b0 = args.K * (1 + f0_bar ** 2) / denominator
    b1 = - (2 * args.K * (1 - f0_bar ** 2)) / denominator
    b2 = args.K * (1 + f0_bar ** 2) / denominator

    return [b0, b1, b2, a1, a2]


def pid_coefficients(args):
    """Calculate PID IIR filter coefficients."""

    # Determine filter order
    if args.Kii != 0:
        assert (args.Kdd, args.Kd, args.Kdd_limit, args.Kd_limit) == \
        (0, 0, float('inf'), float('inf')), \
            "IIR filters I^2 and D or D^2 gain/limit are unsupported"
        order = 2
    elif args.Ki != 0:
        assert (args.Kdd, args.Kdd_limit) == (0, float('inf')), \
            "IIR filters with I and D^2 gain/limit are unsupported"
        order = 1
    else:
        order = 0

    kernels = [
        [1, 0, 0],
        [1, -1, 0],
        [1, -2, 1]
    ]

    gains = [args.Kii, args.Ki, args.Kp, args.Kd, args.Kdd]
    limits = [args.Kii/args.Kii_limit, args.Ki/args.Ki_limit,
              1, args.Kd/args.Kd_limit, args.Kdd/args.Kdd_limit]
    w = 2*pi*args.sample_period
    b = [sum(gains[2 - order + i] * w**(order - i) * kernels[i][j]
             for i in range(3)) for j in range(3)]

    a = [sum(limits[2 - order + i] * w**(order - i) * kernels[i][j]
             for i in range(3)) for j in range(3)]
    b = [i/a[0] for i in b]
    a = [i/a[0] for i in a]
    assert a[0] == 1
    return b + [-ai for ai in a[1:]]
