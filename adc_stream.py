"""
Stream ADC samples from Stabilizer and save to file.
"""

import json
import serial
import logging
import numpy as np

from matplotlib import pyplot as plt
from artiq.tools import add_common_args, init_logger

logger = logging.getLogger(__name__)


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("-s", "--stabilizer", default="10.255.6.56")
    p.add_argument("-c", "--channel", default=0, type=int,
                   help="Stabilizer channel to configure")
    p.add_argument("-f", "--file", default="adc_log.npy",
                   help="log file name")
    p.add_argument("-n", "--n_data", default=int(1e4), type=int,
                   help="Stabilizer channel to configure")
    add_common_args(p)  # This adds the -q and -v handling
    args = p.parse_args()
    init_logger(args)

    # logging.basicConfig(level=logging.DEBUG)

    n_data = args.n_data
    with serial.serial_for_url("socket://" + args.stabilizer + ":1236") as s:
        data_arr = np.frombuffer(s.read(n_data * 2), dtype=np.int16)

    logging.info(data_arr)
    logging.debug(data_arr.view(np.uint8))
    logging.debug([bin(i) for i in data_arr.view(np.uint8)])

    np.save(args.file, data_arr)

    plt.plot(data_arr)
    plt.show()
