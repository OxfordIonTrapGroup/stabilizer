import json
import asyncio
from collections import OrderedDict as OD
import logging
import argparse
import socket

import numpy as np

"""
class FeedfowardConfig:
    async def connect(self, host, port=1237):
        self.reader, self.writer = await asyncio.open_connection(host, port)

    async def set(self, cos_amplitudes, sin_amplitudes, offset):
        up = OD([("cos_amplitudes", cos_amplitudes),("sin_amplitudes", sin_amplitudes), ("offset", offset)])
        raw_msg = json.dumps(up, separators=(",", ":"))
        s.sendall(raw_msg.encode() + b"\n")
        data = s.recv(1024)
        assert "\n" not in s
        logger.debug("send %s", s)
        self.writer.write(s.encode() + b"\n")
        r = (await self.reader.readline()).decode()
        logger.debug("recv %s", r)
        ret = json.loads(r, object_pairs_hook=OD)
        if ret["code"] != 200:
            raise StabilizerError(ret)
        return ret
"""
class Feedforward:
	conversion_factor = 1/500 # [0, 500uA] maps to [0, 1]
	def __init__(self):
		pass

	def set_amplitudes(self, amplitudes):
		amplitudes = [a * self.conversion_factor for a in amplitudes]
		return amplitudes

	def set_offset(self, offset):
		offset = offset * self.conversion_factor
		return offset

class Stabilizer:
	# Eventually want to make this a bit cleaner such that
	# set_feedback and set_feedforward are similar

	def __init__(self):
		self.ip = "10.255.6.1"

		# Feedback parameters
		self.channel = 0
		self.channel_offset = 0
		self.proportional_gain = 0
		self.integral_gain = 0
		self.cpu_dac_en = 1
		self.cpu_dac_output = 0
		self.gpio_hdr = 0

		# Feedforward parameters
		self.num_harmonics = 5
		self.sin_amps = [0 for n in range(self.num_harmonics)]
		self.cos_amps = [0 for n in range(self.num_harmonics)]
		self.ff_offset = 0

if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("-s", "--stabilizer", default="10.255.6.58")
    p.add_argument("-S", "--sin_amps", nargs='+', default=[0.]*5, type=float,
                   help="Sin amplitudes as [50Hz 100Hz 150Hz 200Hz 250Hz] in micro-amps, [0, 500uA].")
    p.add_argument("-C", "--cos_amps", nargs='+', default=[0.]*5, type=float,
                   help="Cos amplitudes as [50Hz 100Hz 150Hz 200Hz 250Hz] in micro-amps, [0, 500uA].")
    p.add_argument("-o", "--offset", default=0., type=float,
                   help="Feed forward offset, in micro-amps, [0, 500uA].")
    args = p.parse_args()

    loop = asyncio.get_event_loop()
    # loop.set_debug(True)
    logging.basicConfig(level=logging.DEBUG)

    async def main():
        ff = Feedforward()
        cos_amps = ff.set_amplitudes(args.cos_amps)
        sin_amps = ff.set_amplitudes(args.sin_amps)
        offset = ff.set_offset(args.offset)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((args.stabilizer, 1237))
            msg = OD([("sin_amplitudes", sin_amps), ("cos_amplitudes", cos_amps), ("offset", offset)])
            raw_msg = json.dumps(msg, separators=(",", ":"))
            s.sendall(raw_msg.encode() + b"\n")
            data = s.recv(1024)
            print(data.decode())
        """
	    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    		s.connect((self.ip, 1237))
		    msg = OD([("sin_amplitudes", sin_amplitudes), ("cos_amplitudes", cos_amplitudes), ("offset", offset)])
	        raw_msg = json.dumps(msg, separators=(",", ":"))
	        s.sendall(raw_msg.encode() + b"\n")
	        data = s.recv(1024)
	        print(data.decode())
        """

    loop.run_until_complete(main())
