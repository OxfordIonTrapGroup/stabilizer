import json
import socket
from collections import OrderedDict as OD
import argparse

ip = "10.255.6.56"

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("-s", "--stabilizer", default="10.255.6.56")
    p.add_argument("-S", "--sin_amps", nargs='+', default=[0.]*5, type=float,
                   help="Sin amplitudes as [50Hz 100Hz 150Hz 200Hz 250Hz] in units of full scale")
    p.add_argument("-C", "--cos_amps", nargs='+', default=[0.]*5, type=float,
                   help="Cos amplitudes as [50Hz 100Hz 150Hz 200Hz 250Hz] in units of full scale")
    p.add_argument("-o", "--offset", default=0., type=float,
                   help="Feed forward offset")
    args = p.parse_args()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ip, 1237))

        sin_amplitudes = args.sin_amps
        cos_amplitudes = args.cos_amps
        offset = args.offset

        msg = OD([("sin_amplitudes", sin_amplitudes), ("cos_amplitudes", cos_amplitudes), ("offset", offset)])
        raw_msg = json.dumps(msg, separators=(",", ":"))
        s.sendall(raw_msg.encode() + b"\n")
        data = s.recv(1024)
        print(data.decode())
